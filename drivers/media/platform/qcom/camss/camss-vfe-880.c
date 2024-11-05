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
#define		WM_HEADER_EN			BIT(2)
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

#define VFE_BUS_WM_ADDR_STATUS_0(wm)		(BUS_REG_BASE + 0x278 + 0x100 * (wm))
#define VFE_BUS_WM_ADDR_STATUS_1(wm)		(BUS_REG_BASE + 0x27c + 0x100 * (wm))
#define VFE_BUS_WM_ADDR_STATUS_2(wm)		(BUS_REG_BASE + 0x280 + 0x100 * (wm))
#define VFE_BUS_WM_ADDR_STATUS_3(wm)		(BUS_REG_BASE + 0x284 + 0x100 * (wm))

#define RDI_WM(n)			((vfe_is_lite(vfe) ? 0x0 : 0x17) + (n))
#define RDI_COMP_GROUP(n)			((vfe_is_lite(vfe) ? 0x1 : 0xe) + (n))

#define STATS_BHIST_WM 17
#define STATS_IHIST_WM 19
#define STATS_BHIST_COMP_GROUP 8
#define STATS_IHIST_COMP_GROUP 10

// User-provided statistics config bits
#define STATS_CFG_FH_OUTPUT_EN BIT(0)
#define STATS_CFG_IHIST_OUTPUT_EN BIT(1)
#define STATS_CFG_BHIST_OUTPUT_EN BIT(2)
// Extrabuffer memory layout (we need the start address to be aligned to 256, or we can set the byte offset when configuring the address)
#define STATS_FH_DATA_OFFSET 0
#define STATS_FH_DATA_SIZE ALIGN(64, 256)
#define STATS_IHIST_FH_DATA_OFFSET (STATS_FH_DATA_OFFSET + STATS_FH_DATA_SIZE)
#define STATS_IHIST_FH_DATA_SIZE ALIGN(64, 256)
#define STATS_IHIST_DATA_OFFSET (STATS_IHIST_FH_DATA_OFFSET + STATS_IHIST_FH_DATA_SIZE)
#define STATS_IHIST_DATA_SIZE ALIGN(2048, 256)
#define STATS_BHIST_FH_DATA_OFFSET (STATS_IHIST_DATA_OFFSET + STATS_IHIST_DATA_SIZE)
#define STATS_BHIST_FH_DATA_SIZE ALIGN(64, 256)
#define STATS_BHIST_DATA_OFFSET (STATS_BHIST_FH_DATA_OFFSET + STATS_BHIST_FH_DATA_SIZE)
#define STATS_BHIST_DATA_SIZE ALIGN(4096, 256)

#define CAM_IFE_CSID_MUL_FACTOR                 10000
#define CAM_IFE_CSID_DIV_FACTOR                 192

struct frameheader_data {
	u64 ts;
    u32 id;
    u32 local_id;
};

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

		vfe->required_comp_done_mask = BIT(0); // need composite group 0 (full)

		// Enable extra output paths depending on the requested statistics
		if(vfe->stats_cfg) {
			if(vfe->stats_cfg & STATS_CFG_FH_OUTPUT_EN) {
				// Enable the frameheader for the first plane only
				writel(WM_CFG_EN | WM_HEADER_EN, vfe->base + VFE_BUS_WM_CFG(0));
			}
			if(vfe->stats_cfg & STATS_CFG_IHIST_OUTPUT_EN) {
				writel(0, vfe->base + VFE_BUS_WM_IMAGE_CFG_0(STATS_IHIST_WM));
				writel(1, vfe->base + VFE_BUS_WM_IMAGE_CFG_2(STATS_IHIST_WM));
				writel(0x8, vfe->base + VFE_BUS_WM_PACKER_CFG(STATS_IHIST_WM)); // PACKER_FMT_PLAIN_16_16BPP
				writel(WM_CFG_EN | WM_HEADER_EN | WM_CFG_MODE, vfe->base + VFE_BUS_WM_CFG(STATS_IHIST_WM));
				vfe->required_comp_done_mask |= BIT(STATS_IHIST_COMP_GROUP); // need composite group 10 (IHIST stats)
			}
			if(vfe->stats_cfg & STATS_CFG_BHIST_OUTPUT_EN) {
				writel(0, vfe->base + VFE_BUS_WM_IMAGE_CFG_0(STATS_BHIST_WM));
				writel(1, vfe->base + VFE_BUS_WM_IMAGE_CFG_2(STATS_BHIST_WM));
				writel(0xa, vfe->base + VFE_BUS_WM_PACKER_CFG(STATS_BHIST_WM)); // PACKER_FMT_VER3_PLAIN_64
				writel(WM_CFG_EN | WM_HEADER_EN | WM_CFG_MODE, vfe->base + VFE_BUS_WM_CFG(STATS_BHIST_WM));
				vfe->required_comp_done_mask |= BIT(STATS_BHIST_COMP_GROUP); // need composite group 8 (BHIST stats)
			}
		}
	}
	else
	{
		u32 rdi_comp_group = RDI_COMP_GROUP(wm);
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

		// Frameheader streaming can be enabled on LITE IFEs
		u32 wm_cfg = WM_CFG_EN | WM_CFG_MODE;
		if(vfe->stats_cfg & STATS_CFG_FH_OUTPUT_EN)
			wm_cfg |= WM_HEADER_EN;
		writel(wm_cfg, vfe->base + VFE_BUS_WM_CFG(wm));

		vfe->required_comp_done_mask = BIT(rdi_comp_group); // need RDI composite group
	}

}

static void vfe_wm_stop(struct vfe_device *vfe, u8 wm)
{
	wm = RDI_WM(wm);
	writel(0, vfe->base + VFE_BUS_WM_CFG(wm));
}

static void vfe_update_buffer_comp_mask_from_fifo(struct vfe_device *vfe, u32 comp, u32 wm, u32 buff0_wm_addr, u32 buff1_wm_addr)
{
	u32 last_wm_done_addr = readl_relaxed(vfe->base + VFE_BUS_WM_ADDR_STATUS_0(wm));
	
	buff0_wm_addr = buff0_wm_addr >> 8;
	buff1_wm_addr = buff1_wm_addr >> 8;

	if( buff0_wm_addr && (buff0_wm_addr == last_wm_done_addr))
	{
		vfe->line[0].output.buf[0]->comp_done_status |= BIT(comp);
	}
	else if((buff0_wm_addr && buff1_wm_addr) && (buff1_wm_addr == last_wm_done_addr))
	{
		vfe->line[0].output.buf[0]->comp_done_status |= BIT(comp);
		vfe->line[0].output.buf[1]->comp_done_status |= BIT(comp);
	}
}

static void vfe_update_buffer_status(struct vfe_device *vfe, u32 buf_done_status)
{
	struct v4l2_pix_format_mplane *pix = &vfe->line[0].video_out.active_fmt.fmt.pix_mp;

	if (pix->pixelformat == V4L2_PIX_FMT_NV12)
	{
		// IPP streaming path with non lite VFEs

		// We check the full image composite group as well as the statistics comp group if enabled
		if(buf_done_status & BIT(0))
		{
			u32 buff0_addr = vfe->line[0].output.buf[0] ? vfe->line[0].output.buf[0]->addr[0] : 0;
			u32 buff1_addr = vfe->line[0].output.buf[1] ? vfe->line[0].output.buf[1]->addr[0] : 0;
			vfe_update_buffer_comp_mask_from_fifo(vfe, 0, 0, buff0_addr, buff1_addr);
		}
		// BHIST statistics composite group
		if(buf_done_status & BIT(STATS_BHIST_COMP_GROUP) && (vfe->stats_cfg & STATS_CFG_BHIST_OUTPUT_EN))
		{
			u32 buff0_addr = vfe->line[0].output.buf[0] ? (vfe->line[0].output.buf[0]->extrabuffer_dma_addr+STATS_BHIST_DATA_OFFSET) : 0;
			u32 buff1_addr = vfe->line[0].output.buf[1] ? (vfe->line[0].output.buf[1]->extrabuffer_dma_addr+STATS_BHIST_DATA_OFFSET) : 0;
			vfe_update_buffer_comp_mask_from_fifo(vfe, STATS_BHIST_COMP_GROUP, STATS_BHIST_WM, buff0_addr, buff1_addr);
		}
		// IHIST statistics composite group
		if(buf_done_status & BIT(STATS_IHIST_COMP_GROUP) && vfe->stats_cfg & STATS_CFG_IHIST_OUTPUT_EN)
		{
			u32 buff0_addr = vfe->line[0].output.buf[0] ? (vfe->line[0].output.buf[0]->extrabuffer_dma_addr+STATS_IHIST_DATA_OFFSET) : 0;
			u32 buff1_addr = vfe->line[0].output.buf[1] ? (vfe->line[0].output.buf[1]->extrabuffer_dma_addr+STATS_IHIST_DATA_OFFSET) : 0;
			vfe_update_buffer_comp_mask_from_fifo(vfe, STATS_IHIST_COMP_GROUP, STATS_IHIST_WM, buff0_addr, buff1_addr);
		}
	}
	else
	{
		// RDI streaming path (both main and lite VFEs): check only the RDI0 composite group
		if(buf_done_status & BIT(RDI_COMP_GROUP(0)))
		{
			u32 buff0_addr = vfe->line[0].output.buf[0] ? vfe->line[0].output.buf[0]->addr[0] : 0;
			u32 buff1_addr = vfe->line[0].output.buf[1] ? vfe->line[0].output.buf[1]->addr[0] : 0;
			vfe_update_buffer_comp_mask_from_fifo(vfe, RDI_COMP_GROUP(0), RDI_WM(0), buff0_addr, buff1_addr);
		}
	}
}

// custom buf_done is needed for 880 to handle multiple outputs and IPP path
static void vfe_buf_done_880(struct vfe_device *vfe, int wm, struct camss_buf_done_info *buf_done_info)
{
	struct vfe_line *line = &vfe->line[vfe->wm_output_map[wm]];
	struct camss_buffer *ready_buf;
	struct vfe_output *output;
	unsigned long flags;
	u32 index;
	u64 ts_monotonic_raw = ktime_get_raw_ns();
	// Data buffer pointers to retrieve for the metadata output planes
	void* sensorMetaDataBufferAddr = NULL;
	void* frameheaderDataAddr = NULL;
	void* ihistFHDataAddr = NULL;
	void* ihistDataAddr = NULL;
	void* bhistFHDataAddr = NULL;
	void* bhistDataAddr = NULL;
	struct frameheader_data *frameheaderData;
	u64 sof_isp_timestamp_converted = buf_done_info->sof_isp_ts_conv;

	struct v4l2_pix_format_mplane *pix = &line->video_out.active_fmt.fmt.pix_mp;

	spin_lock_irqsave(&vfe->output_lock, flags);

	if (vfe->wm_output_map[wm] == VFE_LINE_NONE) {
		dev_err_ratelimited(vfe->camss->dev,
				    "Received wm done for unmapped index\n");
		goto out_unlock;
	}
	output = &vfe->line[vfe->wm_output_map[wm]].output;

	if(!output->buf[0]) {
		dev_err_ratelimited(vfe->camss->dev,
				    "Missing ready buf %d!\n", output->state);
		goto out_unlock;
	}

	// Update the buffer status with the bus status to keep track of which composite groups are done 
	vfe_update_buffer_status(vfe, buf_done_info->status);

	// If we have received anything for the buffer 1 we need to skip buffer 0
	// We won't be able to send out buffer 0 in any case since it needs frame data stored by the sensor and
	// CSID that are now corresponding to buffer 1
	if(output->buf[1] && (output->buf[1]->comp_done_status != 0x0))
	{
		struct camss_buffer *buf = output->buf[0];
		output->buf[0] = output->buf[1];
		output->buf[1] = NULL;
		// vfe_queue_buffer_v2 will take output_lock
		spin_unlock_irqrestore(&vfe->output_lock, flags);
		vfe_queue_buffer_v2(&line->video_out, buf);
		spin_lock_irqsave(&vfe->output_lock, flags);
	}

	if((output->buf[0] && output->buf[0]->comp_done_status != vfe->required_comp_done_mask)) {
		// Buffer is not complete so not doing anything and wait for the remaining comp done to happen
		goto out_unlock;
	}

	ready_buf = output->buf[0];
	ready_buf->vb.vb2_buf.timestamp = ts_monotonic_raw;
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


	if(buf_done_info->sensor_metadata_buffer_addr != NULL)
	{
		sensorMetaDataBufferAddr = buf_done_info->sensor_metadata_buffer_addr;
	}
	if(vfe->stats_cfg)
	{
		// Make buffer changes visible to CPU
		dma_sync_single_for_cpu(vfe->camss->dev, ready_buf->extrabuffer_dma_addr, ready_buf->extrabuffer_size, DMA_BIDIRECTIONAL);


		// If stats are enabled, use the frameheader timestamp which should be guaranteed to be properly
		// associated with the buffer.
		if(vfe->stats_cfg & STATS_CFG_FH_OUTPUT_EN) {
			frameheaderDataAddr = ready_buf->extrabuffer_vaddr + STATS_FH_DATA_OFFSET;
			frameheaderData = (struct frameheader_data*)frameheaderDataAddr;
			uint64_t frameheader_ts = frameheaderData->ts;

			// The timestamp is 8 bits left shifted because the lower 8 bits contain virtual channel 
			// and other information.
			frameheader_ts >>= 8;

			// Apply the same transformation as in the SOF handler
			frameheader_ts = mul_u64_u32_div(
				frameheader_ts,
				CAM_IFE_CSID_MUL_FACTOR,
				CAM_IFE_CSID_DIV_FACTOR);

			// Use the existing offset to adjust the frame header timestamp
			frameheader_ts += buf_done_info->isp_to_kernel_raw_ts_offset;
			sof_isp_timestamp_converted = frameheader_ts;
		}
		if(vfe->stats_cfg & STATS_CFG_IHIST_OUTPUT_EN) {
			ihistFHDataAddr = ready_buf->extrabuffer_vaddr + STATS_IHIST_FH_DATA_OFFSET;
			ihistDataAddr = ready_buf->extrabuffer_vaddr + STATS_IHIST_DATA_OFFSET;
		}
		if(vfe->stats_cfg & STATS_CFG_BHIST_OUTPUT_EN) {
			bhistFHDataAddr = ready_buf->extrabuffer_vaddr + STATS_BHIST_FH_DATA_OFFSET;
			bhistDataAddr = ready_buf->extrabuffer_vaddr + STATS_BHIST_DATA_OFFSET;
		}
	}

	metadata_send(&vfe->camss->metadata[vfe->id], ts_monotonic_raw, sof_isp_timestamp_converted, buf_done_info->eof_isp_ts_conv, buf_done_info->sof_kernel_ts, ts_monotonic_raw,
		ready_buf->vb.sequence, sensorMetaDataBufferAddr, frameheaderDataAddr, ihistFHDataAddr, ihistDataAddr, bhistFHDataAddr, bhistDataAddr);

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

	buffer->comp_done_status = 0; // reset the comp done status

	if (pix->pixelformat == V4L2_PIX_FMT_NV12) {
		writel((buffer->addr[0] >> 8) & 0xFFFFFFFF, vfe->base + VFE_BUS_WM_IMAGE_ADDR(0));
		writel((buffer->addr[1] >> 8) & 0xFFFFFFFF, vfe->base + VFE_BUS_WM_IMAGE_ADDR(1));

		if(vfe->stats_cfg) {
			dma_sync_single_for_device(vfe->camss->dev, buffer->extrabuffer_dma_addr, buffer->extrabuffer_size, DMA_BIDIRECTIONAL);
			// Enqueue the buffers address for the frameheaders and extra outputs
			if(vfe->stats_cfg & STATS_CFG_FH_OUTPUT_EN) {
				writel((buffer->extrabuffer_dma_addr + STATS_FH_DATA_OFFSET) >> 8, vfe->base + VFE_BUS_WM_HEADER_ADDR(0));
			}
			if(vfe->stats_cfg & STATS_CFG_IHIST_OUTPUT_EN) {
				writel((buffer->extrabuffer_dma_addr + STATS_IHIST_DATA_OFFSET) >> 8, vfe->base + VFE_BUS_WM_IMAGE_ADDR(STATS_IHIST_WM));
				writel((buffer->extrabuffer_dma_addr + STATS_IHIST_FH_DATA_OFFSET) >> 8, vfe->base + VFE_BUS_WM_HEADER_ADDR(STATS_IHIST_WM));

			}
			if(vfe->stats_cfg & STATS_CFG_BHIST_OUTPUT_EN) {
				writel((buffer->extrabuffer_dma_addr + STATS_BHIST_DATA_OFFSET) >> 8, vfe->base + VFE_BUS_WM_IMAGE_ADDR(STATS_BHIST_WM));
				writel((buffer->extrabuffer_dma_addr + STATS_BHIST_FH_DATA_OFFSET) >> 8, vfe->base + VFE_BUS_WM_HEADER_ADDR(STATS_BHIST_WM));
			}
		}
	}
	else {
		wm = RDI_WM(wm);
		writel((buffer->addr[0] >> 8) & 0xFFFFFFFF, vfe->base + VFE_BUS_WM_IMAGE_ADDR(wm));
		dev_dbg(vfe->camss->dev, "%s wm:%d, image buf addr:0x%x\n",
			__func__, wm, buffer->addr[0]);

		// Also enqueue the buffers address for the frameheader if enabled
		if(vfe->stats_cfg & STATS_CFG_FH_OUTPUT_EN) {
			dma_sync_single_for_device(vfe->camss->dev, buffer->extrabuffer_dma_addr, buffer->extrabuffer_size, DMA_BIDIRECTIONAL);
			writel((buffer->extrabuffer_dma_addr + STATS_FH_DATA_OFFSET) >> 8, vfe->base + VFE_BUS_WM_HEADER_ADDR(RDI_WM(0)));
		}

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

static long vfe_set_statistics_config_880(struct vfe_device *vfe, u32 stats_cfg)
{
	// HACK: We assume only line 0 is used in this function

	if(vfe_is_lite(vfe) && (stats_cfg & ~STATS_CFG_FH_OUTPUT_EN)) {
		dev_err(vfe->camss->dev, "Invalid statistics streaming configuration. Only frameheader streaming is supported for LITE VFEs\n");
		return -EINVAL;
	}

	if(vfe->was_streaming) {
		// We cannot set the config if streaming already started, otherwise the write bus
		// won't be configured to enable the statistic output paths.
		dev_err(vfe->camss->dev, "vfe_set_statistics_config_880 called after streaming started\n");
		return -EBUSY;
	}
	if(vfe->line[0].output.gen2.active_num) {
		// We cannot set config if we already have some buffer queued since all buffers must share the same config.
		dev_err(vfe->camss->dev, "vfe_set_statistics_config_880 called with buffers already enqueued");
		return -EBUSY;
	}

	// Set the statistics buffer size
	vfe->stats_cfg = stats_cfg;
	// We reserve for the extra buffer:
	// - 64 bytes for the camera frame header
	// - 64 + 2048 bytes for the image histogram and its header
	// - 64 + 4096 bytes for the bayer histogram and its header
	// (For LITE IFEs only reserve a small size since we can only stream frameheaders)
	vfe->line[0].video_out.required_extrabuffer_size = vfe_is_lite(vfe) ? 256 : 8192;

	// Setup the write bus COMP DONE mask used to test the IRQ status
	vfe->stats_comp_done_mask = 0;
	if(!vfe_is_lite(vfe)) {
		if(vfe->stats_cfg & STATS_CFG_IHIST_OUTPUT_EN) {
			vfe->stats_comp_done_mask |= BIT(STATS_IHIST_COMP_GROUP);
		}
		if(vfe->stats_cfg & STATS_CFG_BHIST_OUTPUT_EN) {
			vfe->stats_comp_done_mask |= BIT(STATS_BHIST_COMP_GROUP);
		}
	}

	dev_warn(vfe->camss->dev, "vfe(%d) statistics streaming config set to 0x%x", vfe->id, vfe->stats_cfg);
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
	.set_statistics_config = vfe_set_statistics_config_880,
};
