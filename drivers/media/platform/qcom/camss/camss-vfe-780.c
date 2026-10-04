// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module 780
 *
 * Copyright (c) 2024 Qualcomm Technologies, Inc.
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include "camss.h"
#include "camss-csid-gen3.h"
#include "camss-vfe.h"
#include "camss-vfe-880-pp.h"

typedef enum {
	VFE_WM_VIDEO_FULL_Y = 0,
	VFE_WM_VIDEO_FULL_C,
	VFE_WM_VIDEO_DS_4,
	VFE_WM_VIDEO_DS_16,
	VFE_WM_DISPLAY_FULL_Y,
	VFE_WM_DISPLAY_FULL_C,
	VFE_WM_DISPLAY_DS_4,
	VFE_WM_DISPLAY_DS_16,
	VFE_WM_FD_Y,
	VFE_WM_FD_C,
	VFE_WM_PIXEL_RAW,
	VFE_WM_STATS_AEC_BE,
	VFE_WM_STATS_AEC_BHIST,
	VFE_WM_STATS_TINTLESS_BG,
	VFE_WM_STATS_AWB_BG,
	VFE_WM_STATS_AWB_BFW,
	VFE_WM_STATS_BF,
	VFE_WM_STATS_GTM_BHIST,
	VFE_WM_STATS_RS,
	VFE_WM_STATS_IHIST,
	VFE_WM_PDAF_2PD,
	VFE_WM_PDAF_PREPROCESS_2PD,
	VFE_WM_PDAF_PARSED,
	VFE_WM_RDI0,
	VFE_WM_RDI1,
	VFE_WM_RDI2,
	VFE_WM_LTM,
	VFE_WM_STATS_ALSC,
}vfe_wm_t;

typedef enum {
	VFE_LITE_WM_RDI0 = 0,
	VFE_LITE_WM_RDI1,
	VFE_LITE_WM_RDI2,
	VFE_LITE_WM_RDI3,
	VFE_LITE_WM_PREPROCESS_RAW,
	VFE_LITE_WM_STATS_BG,
} vfe_lite_wm_t;

#define BUS_REG_BASE (vfe_is_lite(vfe) ? 0x200 : 0xC00)

#define VFE_BUS_WM_TEST_BUS_CTRL (BUS_REG_BASE + 0xDC)

#define VFE_BUS_WM_CGC_OVERRIDE		(BUS_REG_BASE + 0x08)
#define		WM_CGC_OVERRIDE_ALL		(0x7FFFFFF)

#define VFE_BUS_WM_CFG(n)		(BUS_REG_BASE + 0x200 + (n) * 0x100)
#define		WM_CFG_EN			BIT(0)
#define		WM_VIR_FRM_EN			BIT(1)
#define		WM_CFG_MODE			BIT(16)
#define VFE_BUS_WM_IMAGE_ADDR(n)	(BUS_REG_BASE + 0x204 + (n) * 0x100)
#define VFE_BUS_WM_FRAME_INCR(n)	(BUS_REG_BASE + 0x208 + (n) * 0x100)
#define VFE_BUS_WM_IMAGE_CFG_0(n)	(BUS_REG_BASE + 0x20c + (n) * 0x100)
#define		WM_IMAGE_CFG_0_DEFAULT_WIDTH	(0xFFFF)
#define VFE_BUS_WM_IMAGE_CFG_1(n)	(BUS_REG_BASE + 0x210 + (n) * 0x100)
#define VFE_BUS_WM_IMAGE_CFG_2(n)	(BUS_REG_BASE + 0x214 + (n) * 0x100)
#define		WM_IMAGE_CFG_2_DEFAULT_STRIDE	(0xFFFF)
#define VFE_BUS_WM_PACKER_CFG(n)	(BUS_REG_BASE + 0x218 + (n) * 0x100)

#define VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(n)	(BUS_REG_BASE + 0x230 + (n) * 0x100)
#define VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(n)	(BUS_REG_BASE + 0x234 + (n) * 0x100)
#define VFE_BUS_WM_FRAMEDROP_PERIOD(n)		(BUS_REG_BASE + 0x238 + (n) * 0x100)
#define VFE_BUS_WM_FRAMEDROP_PATTERN(n)		(BUS_REG_BASE + 0x23c + (n) * 0x100)

#define VFE_BUS_WM_MMU_PREFETCH_CFG(n)		(BUS_REG_BASE + 0x260 + (n) * 0x100)
#define VFE_BUS_WM_MMU_PREFETCH_MAX_OFFSET(n)	(BUS_REG_BASE + 0x264 + (n) * 0x100)

/* One write master per plane of the PIX output: Y, then CbCr at half height */
static void vfe_wm_start_pix(struct vfe_device *vfe, struct vfe_line *line)
{
	struct vfe_output *output = &line->output[0];
	struct v4l2_pix_format_mplane *pix = &output->video_out.active_fmt.fmt.pix_mp;
	unsigned int i;

	for (i = 0; i < output->wm_num; i++) {
		u8 wm = output->wm[i].bus_client;
		u8 plane = output->wm[i].plane;

		writel((pix->height >> plane) << 16 | pix->width,
		       vfe->base + VFE_BUS_WM_IMAGE_CFG_0(wm));
		writel(0, vfe->base + VFE_BUS_WM_IMAGE_CFG_1(wm));
		writel(pix->plane_fmt[0].bytesperline,
		       vfe->base + VFE_BUS_WM_IMAGE_CFG_2(wm));
		writel(0x3, vfe->base + VFE_BUS_WM_PACKER_CFG(wm));

		/* no dropped frames, one irq per frame */
		writel(0, vfe->base + VFE_BUS_WM_FRAMEDROP_PERIOD(wm));
		writel(1, vfe->base + VFE_BUS_WM_FRAMEDROP_PATTERN(wm));
		writel(0, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(wm));
		writel(1, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(wm));

		writel(1, vfe->base + VFE_BUS_WM_MMU_PREFETCH_CFG(wm));
		writel(0xFFFFFFFF, vfe->base + VFE_BUS_WM_MMU_PREFETCH_MAX_OFFSET(wm));

		/* frame based */
		writel(WM_CFG_EN, vfe->base + VFE_BUS_WM_CFG(wm));
	}
}

static void vfe_wm_start(struct vfe_device *vfe, u8 wm, struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix =
		&line->output[0].video_out.active_fmt.fmt.pix_mp;

	wm = line->output[0].wm[0].bus_client;

	/* no clock gating at bus input */
	writel(WM_CGC_OVERRIDE_ALL, vfe->base + VFE_BUS_WM_CGC_OVERRIDE);

	writel(0x0, vfe->base + VFE_BUS_WM_TEST_BUS_CTRL);

	if (line->is_pix) {
		vfe_wm_start_pix(vfe, line);
		return;
	}

	writel(ALIGN(pix->plane_fmt[0].bytesperline, 16) * pix->height >> 8,
	       vfe->base + VFE_BUS_WM_FRAME_INCR(wm));

	writel((WM_IMAGE_CFG_0_DEFAULT_WIDTH & 0xFFFF),
	       vfe->base + VFE_BUS_WM_IMAGE_CFG_0(wm));
	writel(WM_IMAGE_CFG_2_DEFAULT_STRIDE,
	       vfe->base + VFE_BUS_WM_IMAGE_CFG_2(wm));
	writel(0, vfe->base + VFE_BUS_WM_PACKER_CFG(wm));

	/* no dropped frames, one irq per frame */
	writel(0, vfe->base + VFE_BUS_WM_FRAMEDROP_PERIOD(wm));
	writel(1, vfe->base + VFE_BUS_WM_FRAMEDROP_PATTERN(wm));
	writel(0, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(wm));
	writel(1, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(wm));

	writel(1, vfe->base + VFE_BUS_WM_MMU_PREFETCH_CFG(wm));
	writel(0xFFFFFFFF, vfe->base + VFE_BUS_WM_MMU_PREFETCH_MAX_OFFSET(wm));

	writel(WM_CFG_EN | WM_CFG_MODE, vfe->base + VFE_BUS_WM_CFG(wm));
}

static void vfe_wm_stop(struct vfe_device *vfe, u8 wm, struct vfe_line *line)
{
	struct vfe_output *output = &line->output[0];
	unsigned int i;

	for (i = 0; i < output->wm_num; i++)
		writel(0, vfe->base + VFE_BUS_WM_CFG(output->wm[i].bus_client));
}

static void vfe_wm_update(struct vfe_device *vfe, u8 wm, struct camss_buffer *buffer,
			  struct vfe_line *line)
{
	struct vfe_output *output = &line->output[0];
	unsigned int i;

	/* camss-video sets addr[1] = addr[0] + bytesperline * height for NV12 */
	for (i = 0; i < output->wm_num; i++) {
		u32 addr = buffer->addr[output->wm[i].plane];

		wm = output->wm[i].bus_client;
		writel(addr >> 8, vfe->base + VFE_BUS_WM_IMAGE_ADDR(wm));

		dev_dbg(vfe->camss->dev, "wm:%d, image buf addr:0x%x\n",
			wm, addr);
	}
}

/* The PIX line is latched through the CSID's IPP path, RDIs through their own */
static int vfe_reg_update_port(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	return vfe->line[line_id].is_pix ? CSID_780_CUSTOM_PORT_ID_IPP0 : line_id;
}

static void vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	camss_reg_update(vfe->camss, vfe->id, vfe_reg_update_port(vfe, line_id), false);
}

static inline void vfe_reg_update_clear(struct vfe_device *vfe,
					enum vfe_line_id line_id)
{
	camss_reg_update(vfe->camss, vfe->id, vfe_reg_update_port(vfe, line_id), true);
}

/*
 * Program the pixel pipe outside output_lock, before the write masters and
 * the IPP reg update that latches both.
 */
static void vfe_line_prepare(struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix = &line->output[0].video_out.active_fmt.fmt.pix_mp;

	if (line->is_pix && pix->pixelformat == V4L2_PIX_FMT_NV12)
		vfe_880_setup_pixel_pipe(to_vfe(line), pix->width, pix->height);
}

static const struct camss_video_ops vfe_video_ops_780 = {
	.queue_buffer = vfe_queue_buffer_v2,
	.flush_buffers = vfe_flush_buffers,
};
static void vfe_subdev_init(struct device *dev, struct vfe_device *vfe)
{
	struct vfe_line *pix = &vfe->line[VFE_LINE_PIX];

	vfe->video_ops = vfe_video_ops_780;

	/* RDI lite */
	if (vfe_is_lite(vfe)) {
		vfe_add_rdi(&vfe->line[VFE_LINE_RDI0].output[0], VFE_LITE_WM_RDI0, VFE_V3_COMP_GRP_1);
		vfe_add_rdi(&vfe->line[VFE_LINE_RDI1].output[0], VFE_LITE_WM_RDI1, VFE_V3_COMP_GRP_2);
		vfe_add_rdi(&vfe->line[VFE_LINE_RDI2].output[0], VFE_LITE_WM_RDI2, VFE_V3_COMP_GRP_3);
		vfe_add_rdi(&pix->output[0], VFE_LITE_WM_RDI3, VFE_V3_COMP_GRP_4);
		return;
	}

	/* RDI */
	vfe_add_rdi(&vfe->line[VFE_LINE_RDI0].output[0], VFE_WM_RDI0, VFE_V3_COMP_GRP_14);
	vfe_add_rdi(&vfe->line[VFE_LINE_RDI1].output[0], VFE_WM_RDI1, VFE_V3_COMP_GRP_15);
	vfe_add_rdi(&vfe->line[VFE_LINE_RDI2].output[0], VFE_WM_RDI2, VFE_V3_COMP_GRP_16);

	/* Add YUV, DS4, DS16, stats and params descriptors */
	vfe_add_yuv(&pix->output[0], VFE_WM_VIDEO_FULL_Y, VFE_WM_VIDEO_FULL_C, VFE_V3_COMP_GRP_0);
	vfe_add_ds(&pix->output[1], MSM_VFE_PAD_SRC_DS4, VFE_WM_VIDEO_DS_4, VFE_V3_COMP_GRP_0);
	vfe_add_ds(&pix->output[2], MSM_VFE_PAD_SRC_DS16, VFE_WM_VIDEO_DS_16, VFE_V3_COMP_GRP_0);
	vfe_add_stats(&pix->output[3], VFE_WM_STATS_AEC_BHIST, VFE_V3_COMP_GRP_4);
	vfe_add_params(&pix->output[4], VFE_V3_COMP_GRP_0);

	/* PIX mode */
	vfe_set_pix(pix, 5);
}

static void vfe_global_reset(struct vfe_device *vfe)
{
	vfe_isr_reset_ack(vfe);
}

static irqreturn_t vfe_isr(int irq, void *dev)
{
	/* nop */
	return IRQ_HANDLED;
}

static int vfe_halt(struct vfe_device *vfe)
{
	/* rely on vfe_disable_output() to stop the VFE */
	return 0;
}

const struct vfe_hw_ops vfe_ops_780 = {
	.global_reset = vfe_global_reset,
	.hw_version = vfe_hw_version,
	.isr = vfe_isr,
	.pm_domain_off = vfe_pm_domain_off,
	.pm_domain_on = vfe_pm_domain_on,
	.reg_update = vfe_reg_update,
	.reg_update_clear = vfe_reg_update_clear,
	.subdev_init = vfe_subdev_init,
	.vfe_disable = vfe_disable,
	.vfe_enable = vfe_enable_v2,
	.vfe_line_prepare = vfe_line_prepare,
	.vfe_halt = vfe_halt,
	.vfe_wm_start = vfe_wm_start,
	.vfe_wm_stop = vfe_wm_stop,
	.vfe_buf_done = vfe_buf_done,
	.vfe_wm_update = vfe_wm_update,
};
