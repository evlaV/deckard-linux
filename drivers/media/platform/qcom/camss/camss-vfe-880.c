// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module v880 (SM8650)
 *
 * Copyright (c) 2024 Linaro Ltd.
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include "camss.h"
#include "camss-vfe.h"

#define BUS_REG_BASE			(vfe_is_lite(vfe) ? 0x200 : 0xC00)

#define VFE_BUS_WM_CGC_OVERRIDE		(BUS_REG_BASE + 0x08)
#define		WM_CGC_OVERRIDE_ALL		(0x7FFFFFF)

#define VFE_BUS_WM_TEST_BUS_CTRL	(BUS_REG_BASE + 0xDC)

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
#define VFE_BUS_WM_HEADER_ADDR(n)	(BUS_REG_BASE + 0x220 + (n) * 0x100)
#define VFE_BUS_WM_HEADER_INCR(n)	(BUS_REG_BASE + 0x224 + (n) * 0x100)
#define VFE_BUS_WM_HEADER_CFG(n)	(BUS_REG_BASE + 0x228 + (n) * 0x100)

#define VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(n)	(BUS_REG_BASE + 0x230 + (n) * 0x100)
#define VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(n)	(BUS_REG_BASE + 0x234 + (n) * 0x100)
#define VFE_BUS_WM_FRAMEDROP_PERIOD(n)		(BUS_REG_BASE + 0x238 + (n) * 0x100)
#define VFE_BUS_WM_FRAMEDROP_PATTERN(n)		(BUS_REG_BASE + 0x23c + (n) * 0x100)

#define VFE_BUS_WM_MMU_PREFETCH_CFG(n)		(BUS_REG_BASE + 0x260 + (n) * 0x100)
#define VFE_BUS_WM_MMU_PREFETCH_MAX_OFFSET(n)	(BUS_REG_BASE + 0x264 + (n) * 0x100)
#define VFE_BUS_WM_SYSTEM_CACHE_CFG(n)		(BUS_REG_BASE + 0x268 + (n) * 0x100)

#define RDI_WM(n)			((vfe_is_lite(vfe) ? 0x0 : 0x17) + (n))

static void vfe_wm_start(struct vfe_device *vfe, u8 wm, struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix = &line->video_out.active_fmt.fmt.pix_mp;
	if (pix->pixelformat == V4L2_PIX_FMT_NV12) {
		// IPP path config
		u32 wm;
		for(wm = 0; wm < 2; wm++) {
			writel((pix->height >> wm) << 16 | pix->width, vfe->base + VFE_BUS_WM_IMAGE_CFG_0(wm));
			writel(0, vfe->base + VFE_BUS_WM_IMAGE_CFG_1(wm)); // X_INIT = 0
			writel(pix->plane_fmt[0].bytesperline, vfe->base + VFE_BUS_WM_IMAGE_CFG_2(wm)); // stride
			writel(0x3, vfe->base + VFE_BUS_WM_PACKER_CFG(wm));
			writel(WM_CFG_EN, vfe->base + VFE_BUS_WM_CFG(wm)); // EN=1 (+ all other settings to 0)
		}
	}
	else
	{
		wm = RDI_WM(wm);

		writel(/*WM_CGC_OVERRIDE_ALL*/0, vfe->base + VFE_BUS_WM_CGC_OVERRIDE);

		writel(0x0, vfe->base + VFE_BUS_WM_TEST_BUS_CTRL);

#if 0
		writel(ALIGN(pix->plane_fmt[0].bytesperline, 16) * pix->height >> 8,
			vfe->base + VFE_BUS_WM_FRAME_INCR(wm));
#endif
		writel(WM_IMAGE_CFG_0_DEFAULT_WIDTH & 0xFFFF,
			vfe->base + VFE_BUS_WM_IMAGE_CFG_0(wm));
		writel(0, vfe->base + VFE_BUS_WM_IMAGE_CFG_1(wm));
		writel(WM_IMAGE_CFG_2_DEFAULT_STRIDE,
			vfe->base + VFE_BUS_WM_IMAGE_CFG_2(wm));
		writel(0, vfe->base + VFE_BUS_WM_PACKER_CFG(wm));

#if 0
		/* no dropped frames, one irq per frame */
		writel(0, vfe->base + VFE_BUS_WM_FRAMEDROP_PERIOD(wm));
		writel(1, vfe->base + VFE_BUS_WM_FRAMEDROP_PATTERN(wm));
		writel(0, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(wm));
		writel(1, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(wm));

		writel(1, vfe->base + VFE_BUS_WM_MMU_PREFETCH_CFG(wm));
		writel(0xFFFFFFFF, vfe->base + VFE_BUS_WM_MMU_PREFETCH_MAX_OFFSET(wm));
#endif

		writel(WM_CFG_EN | WM_CFG_MODE, vfe->base + VFE_BUS_WM_CFG(wm));
	}

}

static void vfe_wm_stop(struct vfe_device *vfe, u8 wm)
{
	wm = RDI_WM(wm);
	writel(0, vfe->base + VFE_BUS_WM_CFG(wm));
}

// custom buf_done is needed for 880
// TODO: add buff done status to check for outputs FIFO and handle synchronized multi-outputs streams
static void vfe_buf_done_880(struct vfe_device *vfe, int wm)
{
	struct vfe_line *line = &vfe->line[vfe->wm_output_map[wm]];
	struct camss_buffer *ready_buf;
	struct vfe_output *output;
	unsigned long flags;
	u32 index;
	u64 ts = ktime_get_ns();

	struct v4l2_pix_format_mplane *pix = &line->video_out.active_fmt.fmt.pix_mp;

	spin_lock_irqsave(&vfe->output_lock, flags);

	if (vfe->wm_output_map[wm] == VFE_LINE_NONE) {
		dev_err_ratelimited(vfe->camss->dev,
				    "Received wm done for unmapped index\n");
		goto out_unlock;
	}
	output = &vfe->line[vfe->wm_output_map[wm]].output;

	ready_buf = output->buf[0];
	if (!ready_buf) {
		dev_err_ratelimited(vfe->camss->dev,
				    "Missing ready buf %d!\n", output->state);
		goto out_unlock;
	}

	ready_buf->vb.vb2_buf.timestamp = ts;
	ready_buf->vb.sequence = output->sequence++;

	index = 0;
	output->buf[0] = output->buf[1];
	if (output->buf[0])
		index = 1;

	output->buf[index] = vfe_buf_get_pending(output);

	if (output->buf[index]) {
		vfe->res->hw_ops->vfe_wm_update(vfe, output->wm_idx[0],
						output->buf[index],
						line);
		vfe->res->hw_ops->reg_update(vfe, line->id);
	} else
		output->gen2.active_num--;

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	vb2_buffer_done(&ready_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

	return;

out_unlock:
	spin_unlock_irqrestore(&vfe->output_lock, flags);
}

static void vfe_wm_update(struct vfe_device *vfe, u8 wm, struct camss_buffer *buffer,
			  struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix = &line->video_out.active_fmt.fmt.pix_mp;
	if (pix->pixelformat == V4L2_PIX_FMT_NV12) {
		writel((buffer->addr[0] >> 8) & 0xFFFFFFFF, vfe->base + VFE_BUS_WM_IMAGE_ADDR(0));
		writel((buffer->addr[1] >> 8) & 0xFFFFFFFF, vfe->base + VFE_BUS_WM_IMAGE_ADDR(1));
	}
	else {
		wm = RDI_WM(wm);
		writel((buffer->addr[0] >> 8) & 0xFFFFFFFF, vfe->base + VFE_BUS_WM_IMAGE_ADDR(wm));
		dev_dbg(vfe->camss->dev, "%s wm:%d, image buf addr:0x%x\n",
			__func__, wm, buffer->addr[0]);
	}
}

static void vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	struct vfe_line *line = &vfe->line[line_id];
	struct v4l2_pix_format_mplane *pix = &line->video_out.active_fmt.fmt.pix_mp;
	if (pix->pixelformat == V4L2_PIX_FMT_NV12) {
		camss_reg_update(vfe->camss, vfe->id, -4, false); // HACK: needed to set IPP bits in RUP_AUP_CMD
	}
	else {
		int port_id = line_id;
		camss_reg_update(vfe->camss, vfe->id, port_id, false);
	}
}

static inline void vfe_reg_update_clear(struct vfe_device *vfe,
					enum vfe_line_id line_id)
{
	struct vfe_line *line = &vfe->line[line_id];
	struct v4l2_pix_format_mplane *pix = &line->video_out.active_fmt.fmt.pix_mp;
	if (pix->pixelformat == V4L2_PIX_FMT_NV12) {
		camss_reg_update(vfe->camss, vfe->id, -4, true); // HACK: needed to set IPP bits in RUP_AUP_CMD
	}
	else {
		int port_id = line_id;
		camss_reg_update(vfe->camss, vfe->id, port_id, true);
	}
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

static int vfe_halt(struct vfe_device *vfe) {
        /* rely on vfe_disable_output() to stop the VFE */
	return 0;
}

static const struct camss_video_ops vfe_video_ops_880 = {
	.queue_buffer = vfe_queue_buffer_v2,
	.flush_buffers = vfe_flush_buffers,
};

static void vfe_subdev_init(struct device *dev, struct vfe_device *vfe)
{
	vfe->video_ops = vfe_video_ops_880;
}

const struct vfe_hw_ops vfe_ops_880 = {
	.global_reset = vfe_global_reset,
	.hw_version = vfe_hw_version,
	.isr = vfe_isr,
	.pm_domain_off = vfe_pm_domain_off,
	.pm_domain_on = vfe_pm_domain_on,
	.subdev_init = vfe_subdev_init,
	.vfe_disable = vfe_disable,
	.vfe_enable = vfe_enable_v2,
	.vfe_halt = vfe_halt,
	.vfe_wm_start = vfe_wm_start,
	.vfe_wm_stop = vfe_wm_stop,
	.vfe_buf_done = vfe_buf_done_880,
	.vfe_wm_update = vfe_wm_update,
	.reg_update = vfe_reg_update,
	.reg_update_clear = vfe_reg_update_clear,
};
