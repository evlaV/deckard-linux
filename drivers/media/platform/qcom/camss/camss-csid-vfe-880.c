// SPDX-License-Identifier: GPL-2.0
/*
 * camss-csid-4-7.c
 *
 * Qualcomm MSM Camera Subsystem - CSID (CSI Decoder) Module
 *
 * Copyright (C) 2020 Linaro Ltd.
 */
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/delay.h>

#include "camss-csid.h"
#include "camss-csid-gen2.h"
#include "camss.h"

/* The CSID 2 IP-block is different from the others,
 * and is of a bare-bones Lite version, with no PIX
 * interface support. As a result of that it has an
 * alternate register layout.
 */
#define IS_LITE		(csid->id >= 3 ? 1 : 0)

#define CSID_HW_VERSION		0x0
#define		HW_VERSION_STEPPING	0
#define		HW_VERSION_REVISION	16
#define		HW_VERSION_GENERATION	28

#define CSID_RST_CFG		0x0c
#define CSID_RST_CMD		0x10
#define CSID_IRQ_CMD		0x14
#define	     IRQ_CMD_CLEAR	0
#define	     IRQ_CMD_SET	4
#define CSID_RUP_AUP_CMD	0x18

#define CSID_TOP_IRQ_STATUS	0x7c
#define	     TOP_IRQ_STATUS_RESET_DONE	0
#define	     TOP_IRQ_STATUS_BUF_DONE	13
#define CSID_TOP_IRQ_MASK	0x80
#define CSID_TOP_IRQ_CLEAR	0x84
#define CSID_TOP_IRQ_SET	0x88

#define CSID_BUF_DONE_IRQ_STATUS 0x8c
#define	CSID_BUF_DONE_IRQ_MASK	0x90
#define CSID_BUF_DONE_IRQ_CLEAR	0x94
#define CSID_BUF_DONE_IRQ_SET	0x98

#define CSID_CSI2_RX_IRQ_STATUS	0x9c
#define	CSID_CSI2_RX_IRQ_MASK	0xa0
#define CSID_CSI2_RX_IRQ_CLEAR	0xa4
#define CSID_CSI2_RX_IRQ_SET	0xa8

#define CSID_CSI2_RDI0_IRQ_STATUS	0xec
#define CSID_CSI2_RDI0_IRQ_MASK		0xf0
#define CSID_CSI2_RDI0_IRQ_CLEAR	0xf4
#define CSID_CSI2_RDI0_IRQ_SET		0xf8

#define CSID_CSI2_RX_CFG0	0x200
#define		CSI2_RX_CFG0_NUM_ACTIVE_LANES	0
#define		CSI2_RX_CFG0_DL0_INPUT_SEL	4
#define		CSI2_RX_CFG0_DL1_INPUT_SEL	8
#define		CSI2_RX_CFG0_DL2_INPUT_SEL	12
#define		CSI2_RX_CFG0_DL3_INPUT_SEL	16
#define		CSI2_RX_CFG0_PHY_NUM_SEL	20
#define		CSI2_RX_CFG0_PHY_TYPE_SEL	24

#define CSID_CSI2_RX_CFG1	0x204
#define		CSI2_RX_CFG1_PACKET_ECC_CORRECTION_EN		0
#define		CSI2_RX_CFG1_DE_SCRAMBLE_EN			1
#define		CSI2_RX_CFG1_VC_MODE				2
#define		CSI2_RX_CFG1_COMPLETE_STREAM_EN			4
#define		CSI2_RX_CFG1_COMPLETE_STREAM_FRAME_TIMING	5
#define		CSI2_RX_CFG1_MISR_EN				6

#define CSID_RDI_CFG0(rdi)			(0x500 + 0x100 * (rdi))
#define		RDI_CFG0_DECODE_FORMAT		12
#define		RDI_CFG0_DATA_TYPE		16
#define		RDI_CFG0_VIRTUAL_CHANNEL	22
#define		RDI_CFG0_DT_ID			27
#define		RDI_CFG0_ENABLE			31
#define		RDI_CFG0_TIMESTAMP_EN	6

#define CSID_RDI_CTRL(rdi)			(0x504 + 0x100 * (rdi))
#define		RDI_CTRL_HALT_CMD		0
#define			HALT_CMD_HALT_AT_FRAME_BOUNDARY		0
#define			HALT_CMD_RESUME_AT_FRAME_BOUNDARY	1
#define		RDI_CTRL_HALT_MODE		2

/* IFE regs */

#define VFE_HW_VERSION			(IS_LITE ? 0x2000 : 0x00)

#define VFE_IRQ_CMD			(IS_LITE ? 0x2038 : 0x30)
#define     IRQ_CMD_GLOBAL_CLEAR	BIT(0)

#define VFE_IRQ_MASK(n)			((IS_LITE ? 0x2024 : 0x34) + (n) * 4)
#define	    IRQ_MASK_0_BUS_TOP_IRQ	BIT(0)
#define VFE_IRQ_CLEAR(n)		((IS_LITE ? 0x202c : 0x3c) + (n) * 4)
#define VFE_IRQ_STATUS(n)		((IS_LITE ? 0x201c : 0x44) + (n) * 4)

#define BUS_REG_BASE			(IS_LITE ? 0x2200 : 0x0c00)

#define VFE_BUS_WM_CGC_OVERRIDE		(BUS_REG_BASE + 0x08)
#define		WM_CGC_OVERRIDE_ALL	(0x3FFFFFF)

#define VFE_BUS_WM_TEST_BUS_CTRL	(BUS_REG_BASE + 0xdc)

#define VFE_BUS_IRQ_MASK(n)		(BUS_REG_BASE + 0x18 + (n) * 4)
#define VFE_BUS_IRQ_CLEAR(n)		(BUS_REG_BASE + 0x20 + (n) * 4)
#define VFE_BUS_IRQ_STATUS(n)		(BUS_REG_BASE + 0x28 + (n) * 4)
#define VFE_BUS_IRQ_CLEAR_GLOBAL	(BUS_REG_BASE + 0x30)

#define RDI_WM(n)			((IS_LITE ? 0 : 23) + (n))
#define RDI_COMP_GROUP(n)		((IS_LITE ? 1 : 14) + (n))

#define CAM_IFE_CSID_MUL_FACTOR                 10000
#define CAM_IFE_CSID_DIV_FACTOR                 192

static inline void vfe_write(struct csid_device *csid, u32 reg, u32 val)
{
	writel_relaxed(val, csid->ife_base + reg);
}

#define BUS_REG_BASE			(IS_LITE ? 0x2200 : 0x0c00)

#define BUS_CLIENT_CFG(wm)		(BUS_REG_BASE + 0x200 + 0x100 * (wm))
#define BUS_CLIENT_ADDR_IMAGE(wm)   	(BUS_REG_BASE + 0x204 + 0x100 * (wm))
#define BUS_CLIENT_IMAGE_CFG_0(wm)	(BUS_REG_BASE + 0x20c + 0x100 * (wm))
#define BUS_CLIENT_IMAGE_CFG_1(wm)	(BUS_REG_BASE + 0x210 + 0x100 * (wm))
#define BUS_CLIENT_IMAGE_CFG_2(wm)	(BUS_REG_BASE + 0x214 + 0x100 * (wm))
#define BUS_CLIENT_PACKER_CFG(wm)	(BUS_REG_BASE + 0x218 + 0x100 * (wm))
#define BUS_CLIENT_ADDR_CFG(wm)		(BUS_REG_BASE + 0x270 + 0x100 * (wm))

static void vfe_write_buf(struct csid_device *csid, struct camss_buffer *buf)
{
	vfe_write(csid, BUS_CLIENT_ADDR_IMAGE(RDI_WM(0)), buf->addr[0] >> 8);
	// vfe_write(csid, BUS_CLIENT_ADDR_CFG(RDI_WM(0)), 0); // byte offset
	// RUP_RDI0 | AUP_RDI0 | MUP (which one is needed here?)
	writel_relaxed(BIT(4) | BIT(20) | BIT(28), csid->base + CSID_RUP_AUP_CMD);
}

static const struct csid_format csid_formats[] = {
	{
		MEDIA_BUS_FMT_Y8_1X8,
		DATA_TYPE_RAW_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		1,
	},
};

static int csid_reset(struct csid_device *csid);
struct camss_buffer *vfe_buf_get_pending(struct csid_device *csid);

static void csid_configure_stream(struct csid_device *csid, u8 enable)
{
	csid->isp_to_kernel_raw_ts_offset = ULONG_MAX;

	u32 val;
	u32 phy_sel = 0;
	u8 lane_cnt = csid->phy.lane_cnt;
	/* Source pads matching RDI channels on hardware. Pad 1 -> RDI0, Pad 2 -> RDI1, etc. */
	//struct v4l2_mbus_framefmt *input_format = &csid->fmt[MSM_CSID_PAD_FIRST_SRC];
	//const struct csid_format *format = csid_get_fmt_entry(csid->formats, csid->nformats,
	//						      input_format->code);
	unsigned long flags;

	// disabling, wait for HW to idle
	if (!enable) {
		int timeout = 0;
		bool done;
		do {
			if (timeout++ == 100) {
				dev_err(csid->camss->dev, "idle timeout - resetting\n");
				csid_reset(csid);
				spin_lock_irqsave(&csid->output_lock, flags);
				if (csid->out_buf[0]) {
					vb2_buffer_done(&csid->out_buf[0]->vb.vb2_buf, VB2_BUF_STATE_DONE);
					csid->out_buf[0] = NULL;
				}
				if (csid->out_buf[1]) {
					vb2_buffer_done(&csid->out_buf[1]->vb.vb2_buf, VB2_BUF_STATE_DONE);
					csid->out_buf[1] = NULL;
				}
				spin_unlock_irqrestore(&csid->output_lock, flags);
				break;
			}

			spin_lock_irqsave(&csid->output_lock, flags);
			done = (csid->out_buf[0] == NULL);
			spin_unlock_irqrestore(&csid->output_lock, flags);
			usleep_range(10000, 20000);
		} while (!done);
	}

	if (enable && !IS_LITE) {
		// PATH_CFG0: OUTPUT_IFE_EN=1, INPUT_CORE_SEL=1
		writel_relaxed(0x101, csid->camss->csid_top + 0x04 * csid->id);
	}

	if (!lane_cnt)
		lane_cnt = 4;

	phy_sel = csid->phy.csiphy_id + 1;

	if (enable) {
		//val = 1 << RDI_CFG0_BYTE_CNTR_EN;
		//val |= 1 << RDI_CFG0_FORMAT_MEASURE_EN;
		val |= 1 << RDI_CFG0_TIMESTAMP_EN;
		// val = 2 << RDI_CFG1_TIMESTAMP_STB_SEL;
		val |= 0xf << RDI_CFG0_DECODE_FORMAT;
		//val |= format->data_type << RDI_CFG0_DATA_TYPE;
		val |= 0x2a << RDI_CFG0_DATA_TYPE;
		val |= 0 << RDI_CFG0_VIRTUAL_CHANNEL;
		val |= 0 << RDI_CFG0_DT_ID;
		writel_relaxed(val, csid->base + CSID_RDI_CFG0(0));

		writel_relaxed(0, csid->base + CSID_RDI_CTRL(0));

		val = readl_relaxed(csid->base + CSID_RDI_CFG0(0));
		val |=  1 << RDI_CFG0_ENABLE;
		writel_relaxed(val, csid->base + CSID_RDI_CFG0(0));
	}

	val = (lane_cnt - 1) << CSI2_RX_CFG0_NUM_ACTIVE_LANES;
	val |= csid->phy.lane_assign << CSI2_RX_CFG0_DL0_INPUT_SEL;
	val |= phy_sel << CSI2_RX_CFG0_PHY_NUM_SEL;
	writel_relaxed(val, csid->base + CSID_CSI2_RX_CFG0);

	val = 1 << CSI2_RX_CFG1_PACKET_ECC_CORRECTION_EN;
	val |= 1 << CSI2_RX_CFG1_MISR_EN;
	writel_relaxed(val, csid->base + CSID_CSI2_RX_CFG1);

	if (enable)
		val = HALT_CMD_RESUME_AT_FRAME_BOUNDARY << RDI_CTRL_HALT_CMD;
	else
		val = HALT_CMD_HALT_AT_FRAME_BOUNDARY << RDI_CTRL_HALT_CMD;
	writel_relaxed(val, csid->base + CSID_RDI_CTRL(0));

	// RUP_RDI0 | AUP_RDI0 | MUP
	writel_relaxed(BIT(4) | BIT(20) | BIT(28), csid->base + CSID_RUP_AUP_CMD);

	// bus client config
	if (enable)
	{
		//struct v4l2_pix_format_mplane *pix =
		//	&csid->video_out.active_fmt.fmt.pix_mp;
		u32 mode = 1;
		u32 wm = RDI_WM(0);

		//vfe_write(csid, BUS_CLIENT_IMAGE_CFG_0(wm), pix->height << 16 | pix->width);
		vfe_write(csid, BUS_CLIENT_IMAGE_CFG_0(wm), 0xffff); // for RDI mode
		vfe_write(csid, BUS_CLIENT_IMAGE_CFG_1(wm), 0); // X_INIT = 0
		vfe_write(csid, BUS_CLIENT_IMAGE_CFG_2(wm), 0xffff);//pix->plane_fmt[0].bytesperline);
		vfe_write(csid, BUS_CLIENT_PACKER_CFG(wm), 0); // for RDI mode
		vfe_write(csid, BUS_CLIENT_CFG(wm), 1 | mode << 16); // EN=1 (+ all other settings to 0)

		spin_lock_irqsave(&csid->output_lock, flags);

		WARN_ON(csid->out_buf[0]);
		WARN_ON(csid->out_buf[1]);

		csid->out_buf[0] = vfe_buf_get_pending(csid);
		csid->out_buf[1] = vfe_buf_get_pending(csid);

		if (csid->out_buf[0])
			vfe_write_buf(csid, csid->out_buf[0]);
		if (csid->out_buf[1])
			vfe_write_buf(csid, csid->out_buf[1]);

		spin_unlock_irqrestore(&csid->output_lock, flags);
	}

	csid->sequence = 0;
	csid->stream_on = enable;
}

/*
 * csid_hw_version - CSID hardware version query
 * @csid: CSID device
 *
 * Return HW version or error
 */
static u32 csid_hw_version(struct csid_device *csid)
{
	u32 hw_version;
	u32 hw_gen;
	u32 hw_rev;
	u32 hw_step;

	hw_version = readl_relaxed(csid->base + CSID_HW_VERSION);
	hw_gen = (hw_version >> HW_VERSION_GENERATION) & 0xF;
	hw_rev = (hw_version >> HW_VERSION_REVISION) & 0xFFF;
	hw_step = (hw_version >> HW_VERSION_STEPPING) & 0xFFFF;
	dev_dbg(csid->camss->dev, "CSID HW Version = %u.%u.%u\n",
		hw_gen, hw_rev, hw_step);

	hw_version = readl_relaxed(csid->ife_base + VFE_HW_VERSION);
	hw_gen = (hw_version >> 28) & 0xF;
	hw_rev = (hw_version >> 16) & 0xFFF;
	hw_step = hw_version & 0xFFFF;
	dev_dbg(csid->camss->dev, "VFE HW Version = %u.%u.%u\n",
		 hw_gen, hw_rev, hw_step);

	return hw_version;
}

/*
 * csid_isr - CSID module interrupt service routine
 * @irq: Interrupt line
 * @dev: CSID device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t csid_isr(int irq, void *dev)
{
	struct csid_device *csid = dev;
	u32 top_status, rx_status, buf_status, rdi_status;
	unsigned long flags;
	u64 ts_monotonic_raw = ktime_get_raw_ns();
	u64 ts_monotonic = ktime_get_ns();

	top_status = readl_relaxed(csid->base + CSID_TOP_IRQ_STATUS);
	writel_relaxed(top_status, csid->base + CSID_TOP_IRQ_CLEAR);

	rx_status = readl_relaxed(csid->base + CSID_CSI2_RX_IRQ_STATUS);
	writel_relaxed(rx_status, csid->base + CSID_CSI2_RX_IRQ_CLEAR);

	rdi_status = readl_relaxed(csid->base + CSID_CSI2_RDI0_IRQ_STATUS);
	//printk("rdi_status:0x%x ts:%lu", rdi_status, ts_monotonic);
	writel_relaxed(rdi_status, csid->base + CSID_CSI2_RDI0_IRQ_CLEAR);
	if(rdi_status & 0b100000010001000100010000) // Find the actual mask
	{
		//printk("csid sof");
		u64 kernel_ts_raw_to_monotonic_offset = ts_monotonic-ts_monotonic_raw;
		u32 sof_timestamp_lower = readl_relaxed(csid->base + 0x594);
		u32 sof_timestamp_upper = readl_relaxed(csid->base + 0x598);
		u64 sof_isp_timestamp = ((u64)sof_timestamp_upper) << 32 | sof_timestamp_lower;

		sof_isp_timestamp = mul_u64_u32_div(
			sof_isp_timestamp,
			CAM_IFE_CSID_MUL_FACTOR,
			CAM_IFE_CSID_DIV_FACTOR);
		//printk("ts:%lu sof_isp_timestamp:%lu",ts_monotonic, sof_isp_timestamp);
		u64 detected_isp_to_kernel_raw_ts_offset = ts_monotonic_raw - sof_isp_timestamp;

		// This works because the ISP clock does not drift from the monotonic_raw timestamps and the lower the distance
		// between the raw timestamp and the ISP clock, the closer that offset is to the "actual offset"
		if(csid->isp_to_kernel_raw_ts_offset > detected_isp_to_kernel_raw_ts_offset) {
			csid->isp_to_kernel_raw_ts_offset = detected_isp_to_kernel_raw_ts_offset;
		}
		csid->kernel_ts_raw_to_monotonic_offset = kernel_ts_raw_to_monotonic_offset;
		csid->cur_sof_isp_timestamp_converted = sof_isp_timestamp + csid->isp_to_kernel_raw_ts_offset + csid->kernel_ts_raw_to_monotonic_offset;
		if(csid->do_post_sof_work != NULL) {
			csid->do_post_sof_work(csid->capture_subdev, csid->cur_sof_isp_timestamp_converted, ts_monotonic);
		}
	}

	if (top_status & BIT(TOP_IRQ_STATUS_BUF_DONE)) {
		buf_status = readl_relaxed(csid->base + CSID_BUF_DONE_IRQ_STATUS);
		writel_relaxed(buf_status, csid->base + CSID_BUF_DONE_IRQ_CLEAR);

		if (buf_status & BIT(RDI_COMP_GROUP(0))) {

			// read and print timestamp
			//printk("comp group done ts_monotonic - csid->cur_sof_isp_timestamp_converted:%lu  \n", ts_monotonic - csid->cur_sof_isp_timestamp_converted );
			void* sensorMetaDataBufferAddr = NULL;

			spin_lock_irqsave(&csid->output_lock, flags);

			struct camss_buffer *buf = csid->out_buf[0];
			csid->out_buf[0] = csid->out_buf[1];

			if(csid->get_metadata != NULL) 
				{
					sensorMetaDataBufferAddr = csid->get_metadata(csid->capture_subdev );
				}
			
			if (buf) {
				buf->vb.vb2_buf.timestamp = ts_monotonic;
				buf->vb.sequence = csid->sequence++;

				// HACK: Using approximate readout time to get the sof_timestamp. This will not be
				// as precise as CSID register timestamps
				u64 sof_timestamp = ts_monotonic - 7740000;
				u64 eof_timestamp = ts_monotonic;
				metadata_send(&csid->camss->metadata[csid->id], ts_monotonic, csid->cur_sof_isp_timestamp_converted, eof_timestamp, sof_timestamp, ts_monotonic,
        			buf->vb.sequence, sensorMetaDataBufferAddr, NULL, NULL, NULL, NULL, NULL);
				vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
			} else {
				dev_warn(csid->camss->dev, "buf done without a corresponding buffer\n");
			}

			csid->out_buf[csid->out_buf[0] != NULL] = buf = vfe_buf_get_pending(csid);
			if (buf)
				vfe_write_buf(csid, buf);

			

			spin_unlock_irqrestore(&csid->output_lock, flags);

		}
	}

	writel_relaxed(BIT(IRQ_CMD_CLEAR), csid->base + CSID_IRQ_CMD);

	if (top_status & BIT(TOP_IRQ_STATUS_RESET_DONE))
		complete(&csid->reset_complete);



	return IRQ_HANDLED;
}

static irqreturn_t vfe_isr(int irq, void *dev)
{
	struct csid_device *csid = dev;
	void __iomem *vfe_base = csid->ife_base;
	u32 status;

	// note: should not be getting IFE interrupts unless there's an error

	status = readl_relaxed(vfe_base + VFE_IRQ_STATUS(0));
	writel_relaxed(status, vfe_base + VFE_IRQ_CLEAR(0));
	writel_relaxed(IRQ_CMD_GLOBAL_CLEAR, vfe_base + VFE_IRQ_CMD);

	dev_warn(csid->camss->dev, "vfe irq status=%.8x\n", status);

	return IRQ_HANDLED;
}

/*
 * csid_reset - Trigger reset on CSID module and wait to complete
 * @csid: CSID device
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_reset(struct csid_device *csid)
{
	unsigned long time;

	reinit_completion(&csid->reset_complete);

	writel_relaxed(1, csid->base + CSID_TOP_IRQ_CLEAR);
	writel_relaxed(1, csid->base + CSID_IRQ_CMD);
	writel_relaxed(1, csid->base + CSID_TOP_IRQ_MASK);
	writel_relaxed(1, csid->base + CSID_IRQ_CMD);

	// writel_relaxed(1, csid->base + CSID_CSI2_RDI0_IRQ_MASK);
	// writel_relaxed(1, csid->base + CSID_IRQ_CMD);

	writel_relaxed(0x11, csid->base + CSID_RST_CFG); // ENTIRE_CSID, IMMEDIATE
	writel_relaxed(0x1e, csid->base + CSID_RST_CMD); // everything, except registers?


	time = wait_for_completion_timeout(&csid->reset_complete,
					   msecs_to_jiffies(CSID_RESET_TIMEOUT_MS));
	if (!time) {
		dev_err(csid->camss->dev, "CSID reset timeout\n");
		return -EIO;
	}

	writel_relaxed(~0u, csid->base + CSID_TOP_IRQ_MASK);
	writel_relaxed(~0u, csid->base + CSID_BUF_DONE_IRQ_MASK);
	writel_relaxed(0x10, csid->base + CSID_CSI2_RDI0_IRQ_MASK);

	return 0;
}

static u32 csid_src_pad_code(struct csid_device *csid, u32 sink_code,
			     unsigned int match_format_idx, u32 match_code)
{
	return sink_code;
}

struct camss_buffer *vfe_buf_get_pending(struct csid_device *csid)
{
	struct camss_buffer *buffer = NULL;

	if (!list_empty(&csid->pending_bufs)) {
		buffer = list_first_entry(&csid->pending_bufs,
					  struct camss_buffer,
					  queue);
		list_del(&buffer->queue);
	}

	return buffer;
}

static void vfe_buf_add_pending(struct csid_device *csid,
				struct camss_buffer *buffer)
{
	INIT_LIST_HEAD(&buffer->queue);
	list_add_tail(&buffer->queue, &csid->pending_bufs);
}

static int vfe_queue_buffer(struct camss_video *vid,
			    struct camss_buffer *buf)
{
	struct csid_device *csid = container_of(vid, struct csid_device, video_out);
	unsigned long flags;

	spin_lock_irqsave(&csid->output_lock, flags);

	if (csid->stream_on && !csid->out_buf[1]) {
		// out_buf[] is a fifo list with 2 elements, insert in first position if both slots are empty
		csid->out_buf[csid->out_buf[0] != NULL] = buf;
		vfe_write_buf(csid, buf);
	} else {
		vfe_buf_add_pending(csid, buf);
	}

	spin_unlock_irqrestore(&csid->output_lock, flags);

	return 0;
}

int vfe_flush_buffers(struct camss_video *vid,
		      enum vb2_buffer_state state)
{
	struct csid_device *csid = container_of(vid, struct csid_device, video_out);
	struct camss_buffer *buf;
	struct camss_buffer *t;
	unsigned long flags;

	spin_lock_irqsave(&csid->output_lock, flags);

	list_for_each_entry_safe(buf, t, &csid->pending_bufs, queue) {
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		list_del(&buf->queue);
	}

	// flush should only happen after stream off ?
	WARN_ON(csid->out_buf[0]);
	WARN_ON(csid->out_buf[1]);

	spin_unlock_irqrestore(&csid->output_lock, flags);

	return 0;
}



static const struct camss_video_ops vfe_video_ops_880 = {
	.queue_buffer = vfe_queue_buffer,
	.flush_buffers = vfe_flush_buffers,
};

static void csid_subdev_init(struct csid_device *csid)
{
	csid->get_metadata = NULL;
	csid->do_post_sof_work = NULL;
	csid->capture_subdev = NULL;
	csid->formats = csid_formats;
	csid->nformats = ARRAY_SIZE(csid_formats);
	csid->video_ops = vfe_video_ops_880;

	INIT_LIST_HEAD(&csid->pending_bufs);
	spin_lock_init(&csid->output_lock);
}

const struct csid_hw_ops csid_ops_gen2 = {
	.configure_stream = csid_configure_stream,
	.hw_version = csid_hw_version,
	.isr = csid_isr,
	.ife_isr = vfe_isr,
	.reset = csid_reset,
	.src_pad_code = csid_src_pad_code,
	.subdev_init = csid_subdev_init,
};
