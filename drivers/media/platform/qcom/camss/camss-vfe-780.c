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
#include "camss-vfe.h"

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
#define VFE_BUS_WM_IMAGE_CFG_2(n)	(BUS_REG_BASE + 0x214 + (n) * 0x100)
#define		WM_IMAGE_CFG_2_DEFAULT_STRIDE	(0xFFFF)
#define VFE_BUS_WM_PACKER_CFG(n)	(BUS_REG_BASE + 0x218 + (n) * 0x100)

#define VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(n)	(BUS_REG_BASE + 0x230 + (n) * 0x100)
#define VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(n)	(BUS_REG_BASE + 0x234 + (n) * 0x100)
#define VFE_BUS_WM_FRAMEDROP_PERIOD(n)		(BUS_REG_BASE + 0x238 + (n) * 0x100)
#define VFE_BUS_WM_FRAMEDROP_PATTERN(n)		(BUS_REG_BASE + 0x23c + (n) * 0x100)

#define VFE_BUS_WM_MMU_PREFETCH_CFG(n)		(BUS_REG_BASE + 0x260 + (n) * 0x100)
#define VFE_BUS_WM_MMU_PREFETCH_MAX_OFFSET(n)	(BUS_REG_BASE + 0x264 + (n) * 0x100)

static void vfe_wm_start(struct vfe_device *vfe, u8 wm, struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix =
		&line->output[0].video_out.active_fmt.fmt.pix_mp;

	wm = line->output[0].wm[0].bus_client;

	/* no clock gating at bus input */
	writel(WM_CGC_OVERRIDE_ALL, vfe->base + VFE_BUS_WM_CGC_OVERRIDE);

	writel(0x0, vfe->base + VFE_BUS_WM_TEST_BUS_CTRL);

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
	wm = line->output[0].wm[0].bus_client;
	writel(0, vfe->base + VFE_BUS_WM_CFG(wm));
}

static void vfe_wm_update(struct vfe_device *vfe, u8 wm, u32 addr,
			  struct vfe_line *line)
{
	wm = line->output[0].wm[0].bus_client;

	writel((addr >> 8), vfe->base + VFE_BUS_WM_IMAGE_ADDR(wm));

	dev_dbg(vfe->camss->dev, "wm:%d, image buf addr:0x%x\n",
		wm, addr);
}

static void vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	int port_id = line_id;

	camss_reg_update(vfe->camss, vfe->id, port_id, false);
}

static inline void vfe_reg_update_clear(struct vfe_device *vfe,
					enum vfe_line_id line_id)
{
	int port_id = line_id;

	camss_reg_update(vfe->camss, vfe->id, port_id, true);
}

static const struct camss_video_ops vfe_video_ops_780 = {
	.queue_buffer = vfe_queue_buffer_v2,
	.flush_buffers = vfe_flush_buffers,
};

static void vfe_add_rdi(struct vfe_line *line, int bus_client, int comp_group)
{
	line->output[0].wm_num = 1;
	line->output[0].wm[0].bus_client = bus_client;
	line->output[0].wm[0].plane = 0;
	line->output[0].comp_group = comp_group;
}

static void vfe_subdev_init(struct device *dev, struct vfe_device *vfe)
{
	vfe->video_ops = vfe_video_ops_780;

	if (vfe_is_lite(vfe)) {
		vfe_add_rdi(&vfe->line[VFE_LINE_RDI0], VFE_LITE_WM_RDI0, VFE_V3_COMP_GRP_1);
		vfe_add_rdi(&vfe->line[VFE_LINE_RDI1], VFE_LITE_WM_RDI1, VFE_V3_COMP_GRP_2);
		vfe_add_rdi(&vfe->line[VFE_LINE_RDI2], VFE_LITE_WM_RDI2, VFE_V3_COMP_GRP_3);
		vfe_add_rdi(&vfe->line[VFE_LINE_PIX], VFE_LITE_WM_RDI3, VFE_V3_COMP_GRP_4);
		return;
	}

	/* RDI0 */
	vfe_add_rdi(&vfe->line[VFE_LINE_RDI0], VFE_WM_RDI0, VFE_V3_COMP_GRP_14);

	/* RDI1 */
	vfe_add_rdi(&vfe->line[VFE_LINE_RDI1], VFE_WM_RDI1, VFE_V3_COMP_GRP_15);

	/* RDI2 */
	vfe_add_rdi(&vfe->line[VFE_LINE_RDI2], VFE_WM_RDI2, VFE_V3_COMP_GRP_16);

	/* PIX YUV: FULL Y + C, one plane each, group 0 */
	vfe->line[VFE_LINE_PIX].is_pix = true;
	vfe->line[VFE_LINE_PIX].output[0].wm_num = 2;
	vfe->line[VFE_LINE_PIX].output[0].wm[0].bus_client = VFE_WM_VIDEO_FULL_Y;
	vfe->line[VFE_LINE_PIX].output[0].wm[0].plane = 0;
	vfe->line[VFE_LINE_PIX].output[0].wm[1].bus_client = VFE_WM_VIDEO_FULL_C;
	vfe->line[VFE_LINE_PIX].output[0].wm[1].plane = 1;
	vfe->line[VFE_LINE_PIX].output[0].comp_group = VFE_V3_COMP_GRP_0;
	vfe->line[VFE_LINE_PIX].output[0].type = VFE_OUTPUT_TYPE_PIXEL_YUV;

	return;
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
	.vfe_halt = vfe_halt,
	.vfe_wm_start = vfe_wm_start,
	.vfe_wm_stop = vfe_wm_stop,
	.vfe_buf_done = vfe_buf_done,
	.vfe_wm_update = vfe_wm_update,
};
