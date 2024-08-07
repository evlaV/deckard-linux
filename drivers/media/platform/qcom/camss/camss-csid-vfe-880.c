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
#include <linux/minmax.h>

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

#define CSID_PPP0_IRQ_STATUS 0xcc
#define CSID_PPP0_IRQ_MASK 0xd0
#define CSID_PPP0_IRQ_CLEAR 0xd4
#define CSID_PPP0_IRQ_SET 0xd8

#define CSID_IPP0_IRQ_STATUS 0xac
#define CSID_IPP0_IRQ_MASK 0xb0
#define CSID_IPP0_IRQ_CLEAR 0xb4
#define CSID_IPP0_IRQ_SET 0xb8


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
#define		RDI_CFG0_SOF_RETIME_DIS  BIT(5)

#define CSID_RDI_CTRL(rdi)			(0x504 + 0x100 * (rdi))
#define		RDI_CTRL_HALT_CMD		0
#define			HALT_CMD_HALT_AT_FRAME_BOUNDARY		0
#define			HALT_CMD_RESUME_AT_FRAME_BOUNDARY	1
#define		RDI_CTRL_HALT_MODE		2

#define CSID_RDI_MULTIVCDT_CFG(rdi)	(0x50c + 0x100 * (rdi))
#define		RDI_MULTIVCDT_EN				0
#define		RDI_MULTIVCDT_VC1				2
#define		RDI_MULTIVCDT_DT1				7
#define		RDI_MULTIVCDT_TS_COMB_EN		13
#define		RDI_MULTIVCDT_DECODE_FMT		16

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
#define BUS_CLIENT_ADDR_FRAME_HEADER(wm)		(BUS_REG_BASE + 0x220 + 0x100 * (wm))
#define BUS_CLIENT_ADDR_CFG(wm)		(BUS_REG_BASE + 0x270 + 0x100 * (wm))
#define BUS_CLIENT_ADDR_STATUS_0(wm)		(BUS_REG_BASE + 0x278 + 0x100 * (wm))
#define BUS_CLIENT_ADDR_STATUS_1(wm)		(BUS_REG_BASE + 0x27c + 0x100 * (wm))
#define BUS_CLIENT_ADDR_STATUS_2(wm)		(BUS_REG_BASE + 0x280 + 0x100 * (wm))
#define BUS_CLIENT_ADDR_STATUS_3(wm)		(BUS_REG_BASE + 0x284 + 0x100 * (wm))

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
#define STATS_FH_DATA_SIZE ALIGN(16, 256)
#define STATS_IHIST_FH_DATA_OFFSET (STATS_FH_DATA_OFFSET + STATS_FH_DATA_SIZE)
#define STATS_IHIST_FH_DATA_SIZE ALIGN(16, 256)
#define STATS_IHIST_DATA_OFFSET (STATS_IHIST_FH_DATA_OFFSET + STATS_IHIST_FH_DATA_SIZE)
#define STATS_IHIST_DATA_SIZE ALIGN(2048, 256)
#define STATS_BHIST_FH_DATA_OFFSET (STATS_IHIST_DATA_OFFSET + STATS_IHIST_DATA_SIZE)
#define STATS_BHIST_FH_DATA_SIZE ALIGN(16, 256)
#define STATS_BHIST_DATA_OFFSET (STATS_BHIST_FH_DATA_OFFSET + STATS_BHIST_FH_DATA_SIZE)
#define STATS_BHIST_DATA_SIZE ALIGN(4096, 256)


// Shared offset for all CSIDs
#define OFFSET_ESTIMATION_STATE_INIT 0
#define OFFSET_ESTIMATION_STATE_REFINE 1

#define MAX_OFFSET_DELTA_FOR_ADJUSTMENT_IN_NS 10000
#define MAX_OFFSET_DELTA_FOR_VOTE_IN_NS 30000
#define MAX_NUM_OFFSETS 100
#define VOTE_COUNT_THRESHOLD_FOR_OFFSET_RESET (MAX_NUM_OFFSETS/4)
DEFINE_SPINLOCK(isp_to_kernel_raw_ts_offset_estimation_spinlock);
u32 isp_to_kernel_raw_ts_offset_stream_cnt = 0;
u32 isp_to_kernel_raw_ts_offset_estimation_state;
u64 isp_to_kernel_raw_ts_offset_counter;
u32 isp_to_kernel_raw_ts_offset_refinement_vote_cnt;
s64 isp_to_kernel_raw_ts_offset_floor;

static void vfe_write_buf(struct csid_device *csid, struct camss_buffer *buf)
{
	struct v4l2_pix_format_mplane *pix = &csid->video_out.active_fmt.fmt.pix_mp;

	buf->comp_done_status = 0; // reset the comp done status

	if (pix->pixelformat == V4L2_PIX_FMT_NV12) {
		vfe_write(csid, BUS_CLIENT_ADDR_IMAGE(0), buf->addr[0] >> 8);
		vfe_write(csid, BUS_CLIENT_ADDR_IMAGE(1), buf->addr[1] >> 8);

		if(csid->stats_cfg) {
			dma_sync_single_for_device(csid->camss->dev, buf->extrabuffer_dma_addr, buf->extrabuffer_size, DMA_BIDIRECTIONAL);
			// Enqueue the buffers address for the frameheaders and extra outputs
			if(csid->stats_cfg & STATS_CFG_FH_OUTPUT_EN) {
				vfe_write(csid, BUS_CLIENT_ADDR_FRAME_HEADER(0), (buf->extrabuffer_dma_addr + STATS_FH_DATA_OFFSET) >> 8);
			}
			if(csid->stats_cfg & STATS_CFG_IHIST_OUTPUT_EN) {
				vfe_write(csid, BUS_CLIENT_ADDR_IMAGE(STATS_IHIST_WM), (buf->extrabuffer_dma_addr + STATS_IHIST_DATA_OFFSET) >> 8);
				vfe_write(csid, BUS_CLIENT_ADDR_FRAME_HEADER(STATS_IHIST_WM), (buf->extrabuffer_dma_addr + STATS_IHIST_FH_DATA_OFFSET) >> 8);
			}
			if(csid->stats_cfg & STATS_CFG_BHIST_OUTPUT_EN) {
				vfe_write(csid, BUS_CLIENT_ADDR_IMAGE(STATS_BHIST_WM), (buf->extrabuffer_dma_addr + STATS_BHIST_DATA_OFFSET) >> 8);
				vfe_write(csid, BUS_CLIENT_ADDR_FRAME_HEADER(STATS_BHIST_WM), (buf->extrabuffer_dma_addr + STATS_BHIST_FH_DATA_OFFSET) >> 8);
			}
		}

		// RUP_IPP0 | AUP_IPP0 | MUP
		writel_relaxed(BIT(0) | BIT(16) | BIT(28), csid->base + CSID_RUP_AUP_CMD);
	}
	else {
		// vfe_write(csid, BUS_CLIENT_ADDR_CFG(RDI_WM(0)), 0); // byte offset
		vfe_write(csid, BUS_CLIENT_ADDR_IMAGE(RDI_WM(0)), buf->addr[0] >> 8);

		// Also enqueue the buffers address for the frameheader if enabled
		if(csid->stats_cfg & STATS_CFG_FH_OUTPUT_EN) {
			dma_sync_single_for_device(csid->camss->dev, buf->extrabuffer_dma_addr, buf->extrabuffer_size, DMA_BIDIRECTIONAL);
			vfe_write(csid, BUS_CLIENT_ADDR_FRAME_HEADER(RDI_WM(0)), (buf->extrabuffer_dma_addr + STATS_FH_DATA_OFFSET) >> 8);
		}

		// RUP_RDI0 | AUP_RDI0 | MUP (which one is needed here?)
		writel_relaxed(BIT(4) | BIT(20) | BIT(28), csid->base + CSID_RUP_AUP_CMD);
	}
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
	spin_lock(&isp_to_kernel_raw_ts_offset_estimation_spinlock);
	if(enable && (isp_to_kernel_raw_ts_offset_stream_cnt == 0))
	{
		isp_to_kernel_raw_ts_offset_estimation_state = OFFSET_ESTIMATION_STATE_INIT;
		isp_to_kernel_raw_ts_offset_counter = 0;
		isp_to_kernel_raw_ts_offset_refinement_vote_cnt = 0;
		isp_to_kernel_raw_ts_offset_floor = LONG_MAX;
	}
	isp_to_kernel_raw_ts_offset_stream_cnt += (enable ? 1 : -1);
	spin_unlock(&isp_to_kernel_raw_ts_offset_estimation_spinlock);

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
		val = 0;
		//val = 1 << RDI_CFG0_BYTE_CNTR_EN;
		//val |= 1 << RDI_CFG0_FORMAT_MEASURE_EN;
		val |= 1 << RDI_CFG0_TIMESTAMP_EN;
		// val = 2 << RDI_CFG1_TIMESTAMP_STB_SEL;
		val |= 0xf << RDI_CFG0_DECODE_FORMAT;
		//val |= format->data_type << RDI_CFG0_DATA_TYPE;
		val |= 0x2a << RDI_CFG0_DATA_TYPE;
		val |= 0 << RDI_CFG0_VIRTUAL_CHANNEL;
		val |= 0 << RDI_CFG0_DT_ID;
		val |= RDI_CFG0_SOF_RETIME_DIS; // Disable SOF RETIME?
		writel_relaxed(val, csid->base + CSID_RDI_CFG0(0));

		// Enable multi-vcdt by default to accept VC=1/DT=0x2a (could be enabled by a config ioctl)
		val = 0x1 << RDI_MULTIVCDT_EN;
		val |= 0x1 << RDI_MULTIVCDT_VC1;
		val |= 0x2a << RDI_MULTIVCDT_DT1;
		val |= 0xf << RDI_MULTIVCDT_DECODE_FMT;
		// with that on the last 2 bits of IFE timestamps will be set to 0x1 if we receive from this secondary VCDT
		val |= 0x1 << RDI_MULTIVCDT_TS_COMB_EN;
		writel_relaxed(val, csid->base + CSID_RDI_MULTIVCDT_CFG(0));

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
	writel_relaxed(BIT(4) | BIT(20) | BIT(28) | BIT(0) | BIT(16), csid->base + CSID_RUP_AUP_CMD);

	// bus client config
	if (enable)
	{
		struct v4l2_pix_format_mplane *pix = &csid->video_out.active_fmt.fmt.pix_mp;
		if (pix->pixelformat == V4L2_PIX_FMT_NV12) {
			// IPP path config

			csid->sof_addr_upper = csid->base + 0x39c; // CSID_IPP0_TIMESTAMP_CURR1_SOF
			csid->sof_addr_lower = csid->base + 0x398; // CSID_IPP0_TIMESTAMP_CURR0_SOF
			csid->eof_addr_upper = csid->base + 0x3ac; // CSID_IPP0_TIMESTAMP_CURR1_EOF
			csid->eof_addr_lower = csid->base + 0x3a8; // CSID_IPP0_TIMESTAMP_CURR0_EOF

			u32 wm;
			for(wm = 0; wm < 2; wm++) {
				vfe_write(csid, BUS_CLIENT_IMAGE_CFG_0(wm), (pix->height >> wm) << 16 | pix->width);
				vfe_write(csid, BUS_CLIENT_IMAGE_CFG_1(wm), 0); // X_INIT = 0
				vfe_write(csid, BUS_CLIENT_IMAGE_CFG_2(wm), pix->plane_fmt[0].bytesperline); // stride
				vfe_write(csid, BUS_CLIENT_PACKER_CFG(wm), 0x3);
				vfe_write(csid, BUS_CLIENT_CFG(wm), 0x1); // EN=1 (+ all other settings to 0)
			}

			csid->required_comp_done_mask = BIT(0); // need composite group 0 (full)

			// Enable extra output paths depending on the requested statistics
			if(csid->stats_cfg) {
				if(csid->stats_cfg & STATS_CFG_FH_OUTPUT_EN) {
					// Enable the frameheader for the first plane only
					vfe_write(csid, BUS_CLIENT_CFG(0), 0x5); // FRAMEHEADER_EN | EN
				}
				if(csid->stats_cfg & STATS_CFG_IHIST_OUTPUT_EN) {
					vfe_write(csid, BUS_CLIENT_IMAGE_CFG_0(STATS_IHIST_WM), 0);
					vfe_write(csid, BUS_CLIENT_IMAGE_CFG_2(STATS_IHIST_WM), 1);
					vfe_write(csid, BUS_CLIENT_PACKER_CFG(STATS_IHIST_WM), 0x8); // PACKER_FMT_PLAIN_16_16BPP
					vfe_write(csid, BUS_CLIENT_CFG(STATS_IHIST_WM), 0x10005); // MIPI_RAW mode | FRAMEHEADER_EN | EN
					csid->required_comp_done_mask |= BIT(STATS_IHIST_COMP_GROUP); // need composite group 10 (IHIST stats)
				}
				if(csid->stats_cfg & STATS_CFG_BHIST_OUTPUT_EN) {
					vfe_write(csid, BUS_CLIENT_IMAGE_CFG_0(STATS_BHIST_WM), 0);
					vfe_write(csid, BUS_CLIENT_IMAGE_CFG_2(STATS_BHIST_WM), 1);
					vfe_write(csid, BUS_CLIENT_PACKER_CFG(STATS_BHIST_WM), 0xa); // PACKER_FMT_VER3_PLAIN_64
					vfe_write(csid, BUS_CLIENT_CFG(STATS_BHIST_WM), 0x10005); // MIPI_RAW mode | FRAMEHEADER_EN | EN
					csid->required_comp_done_mask |= BIT(STATS_BHIST_COMP_GROUP); // need composite group 8 (BHIST stats)
				}
			}

		}
		else {
			// RDI path config

			csid->sof_addr_upper = csid->base + 0x598; // CSID_RDI0_TIMESTAMP_CURR1_SOF
			csid->sof_addr_lower = csid->base + 0x594; // CSID_RDI0_TIMESTAMP_CURR0_SOF
			csid->eof_addr_upper = csid->base + 0x5a8; // CSID_RDI0_TIMESTAMP_CURR1_EOF
			csid->eof_addr_lower = csid->base + 0x5a4; // CSID_RDI0_TIMESTAMP_CURR0_EOF

			//struct v4l2_pix_format_mplane *pix =
			//	&csid->video_out.active_fmt.fmt.pix_mp;
			u32 mode = 1;
			u32 wm = RDI_WM(0);

			//vfe_write(csid, BUS_CLIENT_IMAGE_CFG_0(wm), pix->height << 16 | pix->width);
			vfe_write(csid, BUS_CLIENT_IMAGE_CFG_0(wm), 0xffff); // for RDI mode
			vfe_write(csid, BUS_CLIENT_IMAGE_CFG_1(wm), 0); // X_INIT = 0
			vfe_write(csid, BUS_CLIENT_IMAGE_CFG_2(wm), 0xffff);//pix->plane_fmt[0].bytesperline);
			vfe_write(csid, BUS_CLIENT_PACKER_CFG(wm), 0); // for RDI mode

			// Frameheader streaming can be enabled on LITE IFEs
			u32 use_fh = !!(csid->stats_cfg & STATS_CFG_FH_OUTPUT_EN);
			vfe_write(csid, BUS_CLIENT_CFG(wm), 1 | (use_fh << 2) | mode << 16); // EN=1 (+ all other settings to 0)

			csid->required_comp_done_mask = BIT(RDI_COMP_GROUP(0)); // need RDI0 composite group
		}

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

static void csid_isr_wm_done(struct csid_device *csid, u32 buf_done_status);
static int vfe_queue_buffer_nolock(struct camss_video *vid, struct camss_buffer *buf);

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
	u32 top_status, rx_status, buf_status, rdi_status, ppp_status, ipp_status;
	u64 ts_monotonic_raw = ktime_get_raw_ns();
	u64 ts_monotonic = ktime_get_ns();

	top_status = readl_relaxed(csid->base + CSID_TOP_IRQ_STATUS);
	writel_relaxed(top_status, csid->base + CSID_TOP_IRQ_CLEAR);

	rx_status = readl_relaxed(csid->base + CSID_CSI2_RX_IRQ_STATUS);
	writel_relaxed(rx_status, csid->base + CSID_CSI2_RX_IRQ_CLEAR);

	rdi_status = readl_relaxed(csid->base + CSID_CSI2_RDI0_IRQ_STATUS);
	writel_relaxed(rdi_status, csid->base + CSID_CSI2_RDI0_IRQ_CLEAR);

	ppp_status = readl_relaxed(csid->base + CSID_PPP0_IRQ_STATUS);
	writel_relaxed(ppp_status, csid->base + CSID_PPP0_IRQ_CLEAR); 

	ipp_status = readl_relaxed(csid->base + CSID_IPP0_IRQ_STATUS);
	writel_relaxed(ipp_status, csid->base + CSID_IPP0_IRQ_CLEAR);

	if((rdi_status & 0x0010) || (ipp_status & 0x0010)) // Test if PATH_INFO_SOF is set
	{
		//printk("csid sof");
		// Just use monotonic raw 
		u64 kernel_ts_raw_to_monotonic_offset = ts_monotonic-ts_monotonic_raw;
		u32 sof_timestamp_lower = readl_relaxed(csid->sof_addr_lower);
		u32 sof_timestamp_upper = readl_relaxed(csid->sof_addr_upper);
		u64 sof_isp_timestamp = ((u64)sof_timestamp_upper) << 32 | sof_timestamp_lower;

		sof_isp_timestamp = mul_u64_u32_div(
			sof_isp_timestamp,
			CAM_IFE_CSID_MUL_FACTOR,
			CAM_IFE_CSID_DIV_FACTOR);
		//printk("csid:%d sof_isp_timestamp:%lu current ts:%lu", csid->id, sof_isp_timestamp, ts_monotonic);
		if(sof_isp_timestamp != 0) {
			s64 detected_isp_to_kernel_raw_ts_offset = ts_monotonic_raw - sof_isp_timestamp;

			isp_to_kernel_raw_ts_offset_counter++;
			
			// Handle estimation state change
			if(isp_to_kernel_raw_ts_offset_counter == MAX_NUM_OFFSETS)
			{
				if(isp_to_kernel_raw_ts_offset_estimation_state == OFFSET_ESTIMATION_STATE_INIT)
				{
					isp_to_kernel_raw_ts_offset_estimation_state = OFFSET_ESTIMATION_STATE_REFINE;
				}
				else
				{
					// If the a percentage of the recent offsets disagreed with the current floor
					// reset the floor and go back to INIT state
					if(isp_to_kernel_raw_ts_offset_refinement_vote_cnt < VOTE_COUNT_THRESHOLD_FOR_OFFSET_RESET)
					{
						isp_to_kernel_raw_ts_offset_estimation_state = OFFSET_ESTIMATION_STATE_INIT;
						isp_to_kernel_raw_ts_offset_floor = detected_isp_to_kernel_raw_ts_offset;
					}
				}
				isp_to_kernel_raw_ts_offset_counter = 0;
				isp_to_kernel_raw_ts_offset_refinement_vote_cnt = 0;
			}

			// Compute the difference in nanoseconds between the current floor and the detected offset
			s64 offsets_delta = (detected_isp_to_kernel_raw_ts_offset > isp_to_kernel_raw_ts_offset_floor) ?
				(detected_isp_to_kernel_raw_ts_offset - isp_to_kernel_raw_ts_offset_floor) :
				(isp_to_kernel_raw_ts_offset_floor - detected_isp_to_kernel_raw_ts_offset);

			// In INIT just update the floor with the latest measurement if lower
			if(isp_to_kernel_raw_ts_offset_estimation_state == OFFSET_ESTIMATION_STATE_INIT)
			{
				if(detected_isp_to_kernel_raw_ts_offset < isp_to_kernel_raw_ts_offset_floor)
				{
					isp_to_kernel_raw_ts_offset_floor = detected_isp_to_kernel_raw_ts_offset;
				}
			}
			else if(isp_to_kernel_raw_ts_offset_estimation_state == OFFSET_ESTIMATION_STATE_REFINE)
			{
				// Update the vote counter if detected offset is close to the current floor
				if(offsets_delta < MAX_OFFSET_DELTA_FOR_VOTE_IN_NS)
				{
					isp_to_kernel_raw_ts_offset_refinement_vote_cnt++;
				}

				// If detected value is lower and very close to the current floor we adjust to it
				if((detected_isp_to_kernel_raw_ts_offset < isp_to_kernel_raw_ts_offset_floor) && (offsets_delta < MAX_OFFSET_DELTA_FOR_ADJUSTMENT_IN_NS))
				{
					isp_to_kernel_raw_ts_offset_floor = detected_isp_to_kernel_raw_ts_offset;
				}
			}

			csid->kernel_ts_raw_to_monotonic_offset = kernel_ts_raw_to_monotonic_offset;
			csid->cur_sof_isp_timestamp_converted = sof_isp_timestamp + isp_to_kernel_raw_ts_offset_floor;
			if(csid->do_post_sof_work != NULL) {
				u8 group = (sof_timestamp_lower & 1) ? 1 : 0;
				csid->do_post_sof_work(csid->capture_subdev, csid->cur_sof_isp_timestamp_converted, ts_monotonic_raw, group);
			}

			csid->last_sof_kernel_ts = ts_monotonic_raw;
		}
		else
		{
			csid->last_sof_kernel_ts = ts_monotonic_raw;
			csid->cur_sof_isp_timestamp_converted = 0;
		}
	}

	if (top_status & BIT(TOP_IRQ_STATUS_BUF_DONE)) {
		buf_status = readl_relaxed(csid->base + CSID_BUF_DONE_IRQ_STATUS);
		writel_relaxed(buf_status, csid->base + CSID_BUF_DONE_IRQ_CLEAR);


		u32 eof_timestamp_lower = readl_relaxed(csid->eof_addr_lower);
		u32 eof_timestamp_upper = readl_relaxed(csid->eof_addr_upper);
		u64 eof_isp_timestamp = ((u64)eof_timestamp_upper) << 32 | eof_timestamp_lower;

		eof_isp_timestamp = mul_u64_u32_div(
			eof_isp_timestamp,
			CAM_IFE_CSID_MUL_FACTOR,
			CAM_IFE_CSID_DIV_FACTOR);

		csid->cur_eof_isp_timestamp_converted = eof_isp_timestamp + isp_to_kernel_raw_ts_offset_floor;

		// Also test comp group 0 and the enabled statistics paths on non lite IFEs to support the IPP path
		if ( (buf_status & BIT(RDI_COMP_GROUP(0))) || (!IS_LITE && (buf_status & (BIT(0) | csid->stats_comp_done_mask)))) {
			csid_isr_wm_done(csid, buf_status);
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

static void csid_update_buffer_comp_mask_from_fifo(struct csid_device *csid, u32 comp, u32 wm, u32 buff0_wm_addr, u32 buff1_wm_addr)
{
	u32 last_wm_done_addr = readl_relaxed(csid->ife_base + BUS_CLIENT_ADDR_STATUS_0(wm));
	
	buff0_wm_addr = buff0_wm_addr >> 8;
	buff1_wm_addr = buff1_wm_addr >> 8;

	if( buff0_wm_addr && (buff0_wm_addr == last_wm_done_addr))
	{
		csid->out_buf[0]->comp_done_status |= BIT(comp);
	}
	else if((buff0_wm_addr && buff1_wm_addr) && (buff1_wm_addr == last_wm_done_addr))
	{
		csid->out_buf[0]->comp_done_status |= BIT(comp);
		csid->out_buf[1]->comp_done_status |= BIT(comp);
	}
}

static void csid_update_buffer_status(struct csid_device *csid, u32 buf_done_status)
{
	struct v4l2_pix_format_mplane *pix = &csid->video_out.active_fmt.fmt.pix_mp;

	if (pix->pixelformat == V4L2_PIX_FMT_NV12)
	{
		// IPP streaming path with non lite VFEs

		// We check the full image composite group as well as the statistics comp group if enabled
		if(buf_done_status & BIT(0))
		{
			u32 buff0_addr = csid->out_buf[0] ? csid->out_buf[0]->addr[0] : 0;
			u32 buff1_addr = csid->out_buf[1] ? csid->out_buf[1]->addr[0] : 0;
			csid_update_buffer_comp_mask_from_fifo(csid, 0, 0, buff0_addr, buff1_addr);
		}
		// BHIST statistics composite group
		if(buf_done_status & BIT(STATS_BHIST_COMP_GROUP) && (csid->stats_cfg & STATS_CFG_BHIST_OUTPUT_EN))
		{
			u32 buff0_addr = csid->out_buf[0] ? (csid->out_buf[0]->extrabuffer_dma_addr+STATS_BHIST_DATA_OFFSET) : 0;
			u32 buff1_addr = csid->out_buf[1] ? (csid->out_buf[1]->extrabuffer_dma_addr+STATS_BHIST_DATA_OFFSET) : 0;
			csid_update_buffer_comp_mask_from_fifo(csid, STATS_BHIST_COMP_GROUP, STATS_BHIST_WM, buff0_addr, buff1_addr);
		}
		// IHIST statistics composite group
		if(buf_done_status & BIT(STATS_IHIST_COMP_GROUP) && csid->stats_cfg & STATS_CFG_IHIST_OUTPUT_EN)
		{
			u32 buff0_addr = csid->out_buf[0] ? (csid->out_buf[0]->extrabuffer_dma_addr+STATS_IHIST_DATA_OFFSET) : 0;
			u32 buff1_addr = csid->out_buf[1] ? (csid->out_buf[1]->extrabuffer_dma_addr+STATS_IHIST_DATA_OFFSET) : 0;
			csid_update_buffer_comp_mask_from_fifo(csid, STATS_IHIST_COMP_GROUP, STATS_IHIST_WM, buff0_addr, buff1_addr);
		}
	}
	else
	{
		// RDI streaming path (both main and lite VFEs): check only the RDI0 composite group
		if(buf_done_status & BIT(RDI_COMP_GROUP(0)))
		{
			u32 buff0_addr = csid->out_buf[0] ? csid->out_buf[0]->addr[0] : 0;
			u32 buff1_addr = csid->out_buf[1] ? csid->out_buf[1]->addr[0] : 0;
			csid_update_buffer_comp_mask_from_fifo(csid, RDI_COMP_GROUP(0), RDI_WM(0), buff0_addr, buff1_addr);
		}
	}
}

static void csid_isr_wm_done(struct csid_device *csid, u32 buf_done_status)
{
	struct camss_buffer *ready_buf = NULL;
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

	spin_lock_irqsave(&csid->output_lock, flags);

	if(!csid->out_buf[0]) {
		dev_warn(csid->camss->dev, "buf done without a corresponding buffer\n");
		goto out_unlock;
	}

	// Update the buffer status with the bus status to keep track of which composite groups are done 
	csid_update_buffer_status(csid, buf_done_status);

	// If we have received anything for the buffer 1 we need to skip buffer 0
	// We won't be able to send out buffer 0 in any case since it needs frame data stored by the sensor and
	// CSID that are now corresponding to buffer 1
	if(csid->out_buf[1] && (csid->out_buf[1]->comp_done_status != 0x0))
	{
		struct camss_buffer *buf = csid->out_buf[0];
		csid->out_buf[0] = csid->out_buf[1];
		csid->out_buf[1] = NULL;
		vfe_queue_buffer_nolock(&csid->video_out, buf);
	}
	
	if((csid->out_buf[0] && csid->out_buf[0]->comp_done_status != csid->required_comp_done_mask)) {
		// Buffer is not complete so not doing anything and wait for the remaining comp done to happen
		goto out_unlock;
	}


	ready_buf = csid->out_buf[0];
	ready_buf->vb.vb2_buf.timestamp = ts_monotonic_raw;
	ready_buf->vb.sequence = csid->sequence++;

	index = 0;
	csid->out_buf[0] = csid->out_buf[1];
	if (csid->out_buf[0])
		index = 1;
	
	// try to get a new buffer from the pending buffers
	// if we get one push it into the IFE FIFO
	csid->out_buf[index] = vfe_buf_get_pending(csid);
	if (csid->out_buf[index]) {
		vfe_write_buf(csid, csid->out_buf[index]);
	}


	if(csid->get_metadata != NULL) 
	{
		sensorMetaDataBufferAddr = csid->get_metadata(csid->capture_subdev );
	}
	if(csid->stats_cfg)
	{
		// Make buffer changes visible to CPU
		dma_sync_single_for_cpu(csid->camss->dev, ready_buf->extrabuffer_dma_addr, ready_buf->extrabuffer_size, DMA_BIDIRECTIONAL);
				
		if(csid->stats_cfg & STATS_CFG_FH_OUTPUT_EN) {
			frameheaderDataAddr = ready_buf->extrabuffer_vaddr + STATS_FH_DATA_OFFSET;
		}
		if(csid->stats_cfg & STATS_CFG_IHIST_OUTPUT_EN) {
			ihistFHDataAddr = ready_buf->extrabuffer_vaddr + STATS_IHIST_FH_DATA_OFFSET;
			ihistDataAddr = ready_buf->extrabuffer_vaddr + STATS_IHIST_DATA_OFFSET;
		}
		if(csid->stats_cfg & STATS_CFG_BHIST_OUTPUT_EN) {
			bhistFHDataAddr = ready_buf->extrabuffer_vaddr + STATS_BHIST_FH_DATA_OFFSET;
			bhistDataAddr = ready_buf->extrabuffer_vaddr + STATS_BHIST_DATA_OFFSET;
		}
	}


	metadata_send(&csid->camss->metadata[csid->id], ts_monotonic_raw, csid->cur_sof_isp_timestamp_converted, csid->cur_eof_isp_timestamp_converted, csid->last_sof_kernel_ts, ts_monotonic_raw,
		ready_buf->vb.sequence, sensorMetaDataBufferAddr, frameheaderDataAddr, ihistFHDataAddr, ihistDataAddr, bhistFHDataAddr, bhistDataAddr);

	spin_unlock_irqrestore(&csid->output_lock, flags);
	vb2_buffer_done(&ready_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	return;

out_unlock:
	spin_unlock_irqrestore(&csid->output_lock, flags);
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
	writel_relaxed(0x1010, csid->base + CSID_CSI2_RDI0_IRQ_MASK); // PATH_INFO_SOF | CAMIF_SOF
	writel_relaxed(0x1010, csid->base + CSID_IPP0_IRQ_MASK); // PATH_INFO_SOF | CAMIF_SOF

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

/*
 * vfe_queue_buffer_nolock - Add empty buffer wihthout acquiring a the lock
 * This should only be used in contexts where vfe->output_lock is already locked
 * @vid: Video device structure
 * @buf: Buffer to be enqueued
 *
 * Add an empty buffer - depending on the current number of buffers it will be
 * put in pending buffer queue or directly given to the hardware to be filled.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int vfe_queue_buffer_nolock(struct camss_video *vid,
			    struct camss_buffer *buf)
{
	struct csid_device *csid = container_of(vid, struct csid_device, video_out);
	
	if (csid->stream_on && !csid->out_buf[1]) {
		// out_buf[] is a fifo list with 2 elements, insert in first position if both slots are empty
		csid->out_buf[csid->out_buf[0] != NULL] = buf;
		vfe_write_buf(csid, buf);
	} else {
		vfe_buf_add_pending(csid, buf);
	}

	return 0;
}

static int vfe_queue_buffer(struct camss_video *vid,
			    struct camss_buffer *buf)
{
	struct csid_device *csid = container_of(vid, struct csid_device, video_out);
	unsigned long flags;

	spin_lock_irqsave(&csid->output_lock, flags);
	vfe_queue_buffer_nolock(vid, buf);
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

int csid_vfe_set_registers(struct csid_device *csid, struct csid_vfe_private_registers_config* regs_config)
{
	// Register access only supported for main CSID/VFEs
	if(IS_LITE)
		return -EINVAL;
		
	// No register specified, return 0 (this can be used to test if the feature is supported)
	if(regs_config->count == 0)
		return 0;

	// CSID must be powered to access the registers
	if(!csid->power_on || regs_config->regs == NULL)
		return -EINVAL;

	u32 max_csid_offset = 0x1000;
	u32 max_vfe_offset = 0xf000;
	struct csid_vfe_private_register reg;

	for(u32 i=0; i<regs_config->count; i++)
	{
		void* base;
		u32 max_offset;

		// Read the register configuration from userspace memory
		if(copy_from_user(&reg, regs_config->regs + i, sizeof(struct csid_vfe_private_register)) != 0)
		{
			return -EFAULT;
		}

		if(reg.type == CSID_VFE_PRV_REG_TYPE_CSID) {
			base = csid->base;
			max_offset = max_csid_offset;
		}
		else if(reg.type == CSID_VFE_PRV_REG_TYPE_VFE) {
			base = csid->ife_base;
			max_offset = max_vfe_offset;
		}
		else {
			// Invalid register type
			return -EINVAL;
		}

		if(reg.offset < max_offset) {
			writel_relaxed(reg.value, base + reg.offset);
		}
		else {
			// Invalid parameter, the register address requested exceeds the mapped VFE memory range
			return -EINVAL;
		}
	}
	return 0;
}

int csid_vfe_get_registers(struct csid_device *csid, struct csid_vfe_private_registers_config* regs_config)
{
	// Register access only supported for main CSID/VFEs
	if(IS_LITE)
		return -EINVAL;

	// No register specified, return 0 (this can be used to test if the feature is supported)
	if(regs_config->count == 0)
		return 0;

	// CSID must be powered to access the registers
	if(!csid->power_on || regs_config->regs == NULL)
		return -EINVAL;

	u32 max_csid_offset = 0x1000;
	u32 max_vfe_offset = 0xf000;
	struct csid_vfe_private_register reg;

	for(u32 i=0; i<regs_config->count; i++)
	{
		void* base;
		u32 max_offset;
		
		// Read the register configuration from userspace memory
		if(copy_from_user(&reg, regs_config->regs + i, sizeof(struct csid_vfe_private_register)) != 0)
		{
			return -EFAULT;
		}

		if(reg.type == CSID_VFE_PRV_REG_TYPE_CSID) {
			base = csid->base;
			max_offset = max_csid_offset;
		}
		else if(reg.type == CSID_VFE_PRV_REG_TYPE_VFE) {
			base = csid->ife_base;
			max_offset = max_vfe_offset;
		}
		else {
			// Invalid register type
			return -EINVAL;
		}

		if(reg.offset < max_offset) {
			// Read the value at the register address and write it back to userspace memory
			reg.value = readl_relaxed(base + reg.offset);
			if(copy_to_user(regs_config->regs + i, &reg,  sizeof(struct csid_vfe_private_register)) != 0)
			{
				return -EFAULT;
			}
		}
		else {
			// Invalid parameter, the register address requested exceeds the mapped VFE memory range
			return -EINVAL;
		}
	}
	return 0;
}

static long csid_vfe_set_statistics_config(struct csid_device *csid, u32 stats_cfg)
{
	if(IS_LITE && (stats_cfg & ~STATS_CFG_FH_OUTPUT_EN)) {
		dev_err(csid->camss->dev, "Invalid statistics streaming configuration. Only frameheader streaming is supported for LITE CSID/VFEs\n");
		return -EINVAL;
	}

	if(csid->stream_on) {
		// We cannot set the config if streaming already started, otherwise the write bus
		// won't be configured to enable the statistic output paths.
		dev_err(csid->camss->dev, "csid_vfe_set_statistics_config called after streaming started\n");
		return -EBUSY;
	}
	if(csid->video_out.vb2_q.num_buffers) {
		// We cannot set config if we already have some buffer queued since all buffers must share the same config.
		dev_err(csid->camss->dev, "csid_vfe_set_statistics_config called with buffers already enqueued");
		return -EBUSY;
	}

	// Set the statistics buffer size
	csid->stats_cfg = stats_cfg;
	// We reserve for the extra buffer:
	// - 16 bytes for the camera frame header
	// - 16 + 2048 bytes for the image histogram and its header
	// - 16 + 4096 bytes for the bayer histogram and its header
	// (For LITE IFEs only reserve a small size since we can only stream frameheaders)
	csid->video_out.required_extrabuffer_size = IS_LITE ? 256 : 8192;

	// Setup the write bus COMP DONE mask used to test the IRQ status
	csid->stats_comp_done_mask = 0;
	if(!IS_LITE) {
		if(csid->stats_cfg & STATS_CFG_IHIST_OUTPUT_EN) {
			csid->stats_comp_done_mask |= BIT(STATS_IHIST_COMP_GROUP);
		}
		if(csid->stats_cfg & STATS_CFG_BHIST_OUTPUT_EN) {
			csid->stats_comp_done_mask |= BIT(STATS_BHIST_COMP_GROUP);
		}
	}

	dev_warn(csid->camss->dev, "csid/vfe(%d) statistics streaming config set to 0x%x", csid->id, csid->stats_cfg);
	return 0;
}

const struct csid_hw_ops csid_ops_gen2 = {
	.configure_stream = csid_configure_stream,
	.hw_version = csid_hw_version,
	.isr = csid_isr,
	.ife_isr = vfe_isr,
	.reset = csid_reset,
	.src_pad_code = csid_src_pad_code,
	.subdev_init = csid_subdev_init,
	.set_csid_vfe_registers = csid_vfe_set_registers,
	.get_csid_vfe_registers = csid_vfe_get_registers,
	.set_statistics_config = csid_vfe_set_statistics_config,
};
