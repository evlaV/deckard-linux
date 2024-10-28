// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm MSM Camera Subsystem - CSID (CSI Decoder) Module
 *
 * Copyright (c) 2024 Qualcomm Technologies, Inc.
 */
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>

#include "camss.h"
#include "camss-csid.h"
#include "camss-csid-gen3.h"

#define CSID_IO_PATH_CFG0(csid)		(0x4 * (csid))
#define		OUTPUT_IFE_EN			0x100
#define		INTERNAL_CSID			1

#define CSID_RST_CFG			0xC
#define		RST_MODE			BIT(0)
#define		RST_LOCATION			BIT(4)

#define CSID_RST_CMD			0x10
#define		SELECT_HW_RST			BIT(0)
#define		SELECT_IRQ_RST			BIT(2)

#define CSID_IRQ_CMD			0x14
#define		IRQ_CMD_CLEAR			BIT(0)

#define CSID_RUP_AUP_CMD		0x18
#define		CSID_RUP_AUP_RDI(rdi)		((BIT(4) | BIT(20)) << (rdi) | BIT(28))

#define CSID_TOP_IRQ_STATUS		0x7C
#define		 TOP_IRQ_STATUS_RESET_DONE	BIT(0)

#define CSID_TOP_IRQ_MASK		0x80
#define CSID_TOP_IRQ_CLEAR		0x84
#define CSID_TOP_IRQ_SET		0x88

#define CSID_CSI2_RX_IRQ_STATUS		0x9C
#define CSID_CSI2_RX_IRQ_MASK		0xA0
#define CSID_CSI2_RX_IRQ_CLEAR		0xA4
#define CSID_CSI2_RX_IRQ_SET		0xA8

#define IS_CSID_690(csid)	((csid->camss->res->version == CAMSS_8775P) \
				 || (csid->camss->res->version == CAMSS_8300))
#define CSID_BUF_DONE_IRQ_STATUS	0x8C
#define BUF_DONE_IRQ_STATUS_RDI_OFFSET  (csid_is_lite(csid) ?\
						1 : (IS_CSID_690(csid) ?\
						13 : 14))
#define CSID_BUF_DONE_IRQ_MASK		0x90
#define CSID_BUF_DONE_IRQ_CLEAR		0x94
#define CSID_BUF_DONE_IRQ_SET		0x98

#define CSID_CSI2_RDIN_IRQ_STATUS(rdi)	(0xEC + 0x10 * (rdi))
#define		RUP_DONE_IRQ_STATUS		BIT(23)

#define CSID_CSI2_RDIN_IRQ_MASK(rdi)	(0xF0 + 0x10 * (rdi))
#define CSID_CSI2_RDIN_IRQ_CLEAR(rdi)	(0xF4 + 0x10 * (rdi))
#define CSID_CSI2_RDIN_IRQ_SET(rdi)	(0xF8 + 0x10 * (rdi))

#define CSID_CSI2_RX_CFG0		0x200
#define		CSI2_RX_CFG0_NUM_ACTIVE_LANES	0
#define		CSI2_RX_CFG0_VC_MODE		3
#define		CSI2_RX_CFG0_DL0_INPUT_SEL	4
#define		CSI2_RX_CFG0_PHY_NUM_SEL	20

#define CSID_CSI2_RX_CFG1		0x204
#define		CSI2_RX_CFG1_ECC_CORRECTION_EN	BIT(0)
#define		CSI2_RX_CFG1_VC_MODE		BIT(2)
#define		CSI2_RX_CFG1_MISR_EN		BIT(6)

#define CSID_RDI_CFG0(rdi)	(csid_is_lite(csid) && IS_CSID_690(csid) ?\
					(0x300 + 0x100 * (rdi)) :\
					(0x500 + 0x100 * (rdi)))
#define		RDI_CFG0_TIMESTAMP_EN		BIT(6)
#define		RDI_CFG0_TIMESTAMP_STB_SEL	BIT(8)
#define		RDI_CFG0_DECODE_FORMAT		12
#define		RDI_CFG0_DT			16
#define		RDI_CFG0_VC			22
#define		RDI_CFG0_DT_ID			27
#define		RDI_CFG0_EN			BIT(31)

#define CSID_RDI_CTRL(rdi)	(csid_is_lite(csid) && IS_CSID_690(csid) ?\
					(0x304 + 0x100 * (rdi)) :\
					(0x504 + 0x100 * (rdi)))
#define		RDI_CTRL_START_CMD		BIT(0)

#define CSID_RDI_CFG1(rdi)	(csid_is_lite(csid) && IS_CSID_690(csid) ?\
					(0x310 + 0x100 * (rdi)) :\
					(0x510 + 0x100 * (rdi)))
#define		RDI_CFG1_DROP_H_EN		BIT(5)
#define		RDI_CFG1_DROP_V_EN		BIT(6)
#define		RDI_CFG1_CROP_H_EN		BIT(7)
#define		RDI_CFG1_CROP_V_EN		BIT(8)
#define		RDI_CFG1_PIX_STORE		BIT(10)
#define		RDI_CFG1_PACKING_FORMAT_MIPI	BIT(15)

#define CSID_RDI_IRQ_SUBSAMPLE_PATTERN(rdi)	(csid_is_lite(csid) && IS_CSID_690(csid) ?\
							(0x348 + 0x100 * (rdi)) :\
							(0x548 + 0x100 * (rdi)))
#define CSID_RDI_IRQ_SUBSAMPLE_PERIOD(rdi)	(csid_is_lite(csid) && IS_CSID_690(csid) ?\
							(0x34C + 0x100 * (rdi)) :\
							(0x54C + 0x100 * (rdi)))
#define CSI2_RX_CFG0_PHY_SEL_BASE_IDX	1

#define CAM_IFE_CSID_MUL_FACTOR                 10000
#define CAM_IFE_CSID_DIV_FACTOR                 192

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

static int csid_reset(struct csid_device *csid);

static inline int reg_update_rdi(struct csid_device *csid, int n)
{
	/*  RUP_RDI0 | AUP_RDI0 | MUP */
	return BIT(n + 4) + BIT(20 + n) + BIT(28);
}
#define REG_UPDATE_RDI			reg_update_rdi

static void __csid_configure_rx(struct csid_device *csid,
				struct csid_phy_config *phy, int vc)
{
	int val;

	val = (phy->lane_cnt - 1) << CSI2_RX_CFG0_NUM_ACTIVE_LANES;
	val |= phy->lane_assign << CSI2_RX_CFG0_DL0_INPUT_SEL;
	val |= (phy->csiphy_id + CSI2_RX_CFG0_PHY_SEL_BASE_IDX) << CSI2_RX_CFG0_PHY_NUM_SEL;

	writel(val, csid->base + CSID_CSI2_RX_CFG0);

	val = CSI2_RX_CFG1_ECC_CORRECTION_EN;
	val |= CSI2_RX_CFG1_MISR_EN;
	if (vc > 3)
		val |= CSI2_RX_CFG1_VC_MODE;

	writel(val, csid->base + CSID_CSI2_RX_CFG1);
}

static void __csid_ctrl_rdi(struct csid_device *csid, int enable, u8 rdi)
{
	int val = 0;

	if (enable)
		val = RDI_CTRL_START_CMD;

	writel(val, csid->base + CSID_RDI_CTRL(rdi));

	/* HACK */
	camss_reg_update(csid->camss, 0, 0, true);
}

static void __csid_configure_wrapper(struct csid_device *csid)
{
	u32 val;

	/* csid lite doesn't need to configure top register */
	if (csid->res->is_lite)
		return;

	val = OUTPUT_IFE_EN | INTERNAL_CSID;
	writel(val, csid->camss->csid_wrapper_base + CSID_IO_PATH_CFG0(csid->id));
}

static void __csid_configure_rdi_stream(struct csid_device *csid, u8 enable, u8 vc)
{
	u32 val;
	u8 lane_cnt = csid->phy.lane_cnt;
	/* Source pads matching RDI channels on hardware. Pad 1 -> RDI0, Pad 2 -> RDI1, etc. */
	struct v4l2_mbus_framefmt *input_format = &csid->fmt[MSM_CSID_PAD_FIRST_SRC + vc];
	const struct csid_format_info *format = csid_get_fmt_entry(csid->res->formats->formats,
								   csid->res->formats->nformats,
								   input_format->code);

	if (!lane_cnt)
		lane_cnt = 4;

	/*
	 * DT_ID is a two bit bitfield that is concatenated with
	 * the four least significant bits of the five bit VC
	 * bitfield to generate an internal CID value.
	 *
	 * CSID_RDI_CFG0(vc)
	 * DT_ID : 28:27
	 * VC    : 26:22
	 * DT    : 21:16
	 *
	 * CID   : VC 3:0 << 2 | DT_ID 1:0
	 */
	u8 dt_id = vc & 0x03;

	val = RDI_CFG0_TIMESTAMP_EN;
	val |= RDI_CFG0_TIMESTAMP_STB_SEL;
	/* RDI_CFG0_SOF_RETIME_DIS */
	val |= 1 << 5;
	/* note: for non-RDI path, this should be format->decode_format */
	val |= DECODE_FORMAT_PAYLOAD_ONLY << RDI_CFG0_DECODE_FORMAT;
	val |= vc << RDI_CFG0_VC;
	val |= format->data_type << RDI_CFG0_DT;
	val |= dt_id << RDI_CFG0_DT_ID;

	writel(val, csid->base + CSID_RDI_CFG0(vc));

	val = RDI_CFG1_PACKING_FORMAT_MIPI;
	val |= RDI_CFG1_PIX_STORE;
	val |= RDI_CFG1_DROP_H_EN;
	val |= RDI_CFG1_DROP_V_EN;
	val |= RDI_CFG1_CROP_H_EN;
	val |= RDI_CFG1_CROP_V_EN;

	writel(val, csid->base + CSID_RDI_CFG1(vc));

	val = 0;
	writel(val, csid->base + CSID_RDI_IRQ_SUBSAMPLE_PERIOD(vc));

	val = 1;
	writel(val, csid->base + CSID_RDI_IRQ_SUBSAMPLE_PATTERN(vc));

	val = 0;
	writel(val, csid->base + CSID_RDI_CTRL(vc));

	val = readl(csid->base + CSID_RDI_CFG0(vc));

	if (enable)
		val |= RDI_CFG0_EN;
	writel(val, csid->base + CSID_RDI_CFG0(vc));
}

static int csid_reset(struct csid_device *csid);

static void csid_configure_stream(struct csid_device *csid, u8 enable)
{
	u8 i;

	if (enable)
		__csid_configure_wrapper(csid);

	if (!enable) {
		csid_reset(csid);
		struct camss_buf_done_info buf_done_info;
		memset(&buf_done_info, 0 ,sizeof(struct camss_buf_done_info));
		camss_buf_done(csid->camss, csid->id, 0, &buf_done_info);
		usleep_range(10000, 20000);
	}

	/* Loop through all enabled VCs and configure stream for each */
	for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS; i++)
		//csid->phy.en_vc = 1;
		if (csid->phy.en_vc & BIT(i)) {
			__csid_configure_rdi_stream(csid, enable, i);
			__csid_configure_rx(csid, &csid->phy, i);
			__csid_ctrl_rdi(csid, enable, i);
		}
}

static int csid_configure_testgen_pattern(struct csid_device *csid, s32 val)
{
	return 0;
}

static void csid_subdev_reg_update(struct csid_device *csid, int port_id, bool clear)
{
	if (clear) {
		csid->reg_update &= ~CSID_RUP_AUP_RDI(port_id);
	} else {
		csid->reg_update |= CSID_RUP_AUP_RDI(port_id);
		writel(csid->reg_update, csid->base + CSID_RUP_AUP_CMD);
	}
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
	u32 top_val, val, buf_done_val, rdi_status, ipp_status;
	u8 reset_done;
	u64 ts_monotonic_raw = ktime_get_raw_ns();
	void *sof_addr_lower, *sof_addr_upper, *eof_addr_lower, *eof_addr_upper;
	int i = 0;

	// if IPP path is enabled use it as the source for SOF/EOF timestamps
	if(readl(csid->base +0x300) & BIT(31)) {
		sof_addr_upper = csid->base + 0x39c; // CSID_IPP0_TIMESTAMP_CURR1_SOF
		sof_addr_lower = csid->base + 0x398; // CSID_IPP0_TIMESTAMP_CURR0_SOF
		eof_addr_upper = csid->base + 0x3ac; // CSID_IPP0_TIMESTAMP_CURR1_EOF
		eof_addr_lower = csid->base + 0x3a8; // CSID_IPP0_TIMESTAMP_CURR0_EOF
	} else {
		sof_addr_upper = csid->base + 0x598; // CSID_RDI0_TIMESTAMP_CURR1_SOF
		sof_addr_lower = csid->base + 0x594; // CSID_RDI0_TIMESTAMP_CURR0_SOF
		eof_addr_upper = csid->base + 0x5a8; // CSID_RDI0_TIMESTAMP_CURR1_EOF
		eof_addr_lower = csid->base + 0x5a4; // CSID_RDI0_TIMESTAMP_CURR0_EOF
	}

	top_val = readl(csid->base + CSID_TOP_IRQ_STATUS);
	writel(top_val, csid->base + CSID_TOP_IRQ_CLEAR);
	reset_done = top_val & TOP_IRQ_STATUS_RESET_DONE;

	val = readl(csid->base + CSID_CSI2_RX_IRQ_STATUS);
	writel(val, csid->base + CSID_CSI2_RX_IRQ_CLEAR);

	rdi_status = readl(csid->base + CSID_CSI2_RDIN_IRQ_STATUS(i));
	writel(rdi_status, csid->base + CSID_CSI2_RDIN_IRQ_CLEAR(i));

	ipp_status = readl(csid->base + 0xac);
	writel(ipp_status, csid->base + 0xb4);

	if((rdi_status & 0x0010) || (ipp_status & 0x0010)) // Test if PATH_INFO_SOF is set
	{
		u32 sof_timestamp_lower = readl_relaxed(sof_addr_lower);
		u32 sof_timestamp_upper = readl_relaxed(sof_addr_upper);
		u64 sof_isp_timestamp = ((u64)sof_timestamp_upper) << 32 | sof_timestamp_lower;

		sof_isp_timestamp = mul_u64_u32_div(
			sof_isp_timestamp,
			CAM_IFE_CSID_MUL_FACTOR,
			CAM_IFE_CSID_DIV_FACTOR);

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

#if 0
	if (val & 0x1000) {
		u32 sof_timestamp_lower = readl_relaxed(csid->base + 0x594);
		u32 sof_timestamp_upper = readl_relaxed(csid->base + 0x598);
	}
#endif
#if 0
	/* Read and clear IRQ status for each enabled RDI channel */
	for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS; i++)
		if (csid->phy.en_vc & BIT(i)) {
			val = readl(csid->base + CSID_CSI2_RDIN_IRQ_STATUS(i));
			writel(val, csid->base + CSID_CSI2_RDIN_IRQ_CLEAR(i));

			if (val & RUP_DONE_IRQ_STATUS)
				/* clear the reg update bit */
				csid_subdev_reg_update(csid, i, true);

			if (buf_done_val & BIT(BUF_DONE_IRQ_STATUS_RDI_OFFSET + i)) {
				/*
				 * For Titan Gen3, bus done and RUP IRQ have been moved to
				 * CSID from VFE. Once CSID received bus done, need notify
				 * VFE of this event. Trigger VFE to handle bus done process.
				 */
				camss_buf_done(csid->camss, csid->id, i);
			}
		}
#endif

	if (top_val & 0x2000) {
		buf_done_val = readl(csid->base + CSID_BUF_DONE_IRQ_STATUS);
		writel(buf_done_val, csid->base + CSID_BUF_DONE_IRQ_CLEAR);

		u32 eof_timestamp_lower = readl_relaxed(eof_addr_lower);
		u32 eof_timestamp_upper = readl_relaxed(eof_addr_upper);
		u64 eof_isp_timestamp = ((u64)eof_timestamp_upper) << 32 | eof_timestamp_lower;

		eof_isp_timestamp = mul_u64_u32_div(
			eof_isp_timestamp,
			CAM_IFE_CSID_MUL_FACTOR,
			CAM_IFE_CSID_DIV_FACTOR);

		csid->cur_eof_isp_timestamp_converted = eof_isp_timestamp + isp_to_kernel_raw_ts_offset_floor;

		struct camss_buf_done_info buf_done_info;
		memset(&buf_done_info, 0 ,sizeof(struct camss_buf_done_info));
		// TODO: fill with info, including sensor metadata buffer addr and size
		buf_done_info.status = buf_done_val;
		buf_done_info.sof_isp_ts_conv = csid->cur_sof_isp_timestamp_converted;
		buf_done_info.eof_isp_ts_conv = csid->cur_eof_isp_timestamp_converted;
		buf_done_info.isp_to_kernel_raw_ts_offset = isp_to_kernel_raw_ts_offset_floor;
		buf_done_info.sof_kernel_ts = csid->last_sof_kernel_ts;
		// Provide sensor metadata if available
		if(csid->get_metadata != NULL)
		{
			buf_done_info.sensor_metadata_buffer_addr = csid->get_metadata(csid->capture_subdev );
			buf_done_info.sensor_metadata_buffer_size = csid->metadata_size;
		}

		camss_buf_done(csid->camss, csid->id, 0, &buf_done_info);
	}

	val = IRQ_CMD_CLEAR;
	writel(val, csid->base + CSID_IRQ_CMD);

	if (reset_done)
		complete(&csid->reset_complete);

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
	u32 val;
#if 0
	int i;
#endif

	reinit_completion(&csid->reset_complete);

	writel(1, csid->base + CSID_TOP_IRQ_CLEAR);
	writel(1, csid->base + CSID_IRQ_CMD);
	writel(1, csid->base + CSID_TOP_IRQ_MASK);
	writel(1, csid->base + CSID_IRQ_CMD);

#if 0
	for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS; i++)
		if (csid->phy.en_vc & BIT(i)) {
			writel(BIT(BUF_DONE_IRQ_STATUS_RDI_OFFSET + i),
			       csid->base + CSID_BUF_DONE_IRQ_CLEAR);
			writel(IRQ_CMD_CLEAR, csid->base + CSID_IRQ_CMD);
			writel(BIT(BUF_DONE_IRQ_STATUS_RDI_OFFSET + i),
			       csid->base + CSID_BUF_DONE_IRQ_MASK);
		}
#endif

	/* preserve registers */
	val = RST_LOCATION | RST_MODE;
	writel(val, csid->base + CSID_RST_CFG);

	/* val = SELECT_HW_RST | SELECT_IRQ_RST; */
	val = 0x1e;
	writel(val, csid->base + CSID_RST_CMD);

	time = wait_for_completion_timeout(&csid->reset_complete,
					   msecs_to_jiffies(CSID_RESET_TIMEOUT_MS));
	if (!time) {
		dev_err(csid->camss->dev, "CSID reset timeout\n");
		return -EIO;
	}

	writel_relaxed(~0u, csid->base + CSID_TOP_IRQ_MASK);
	writel_relaxed(~0u, csid->base + CSID_BUF_DONE_IRQ_MASK);
	/* PATH_INFO_SOF | CAMIF_SOF */

	val = 0x1010;
	//val = 0x10881004;
	writel_relaxed(val, csid->base + CSID_CSI2_RDIN_IRQ_MASK(0));
	writel_relaxed(val, csid->base + 0xb0); // IPP SOF

	return 0;
}

static void csid_subdev_init(struct csid_device *csid)
{
	csid->testgen.nmodes = CSID_PAYLOAD_MODE_DISABLED;
}

static int csid_vfe_set_registers(struct csid_device *csid, struct csid_vfe_private_registers_config* regs_config)
{
	// Register access only supported for main CSID/VFEs
	if(csid->res->is_lite)
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
			// base = csid->ife_base;
			base = csid->res->parent_dev_ops->get_base_address(csid->camss, csid->id);
			if(!base)
				return -EFAULT;
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

static int csid_vfe_get_registers(struct csid_device *csid, struct csid_vfe_private_registers_config* regs_config)
{
	// Register access only supported for main CSID/VFEs
	if(csid->res->is_lite)
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
			// base = csid->ife_base;
			base = csid->res->parent_dev_ops->get_base_address(csid->camss, csid->id);
			if(!base)
				return -EFAULT;
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

const struct csid_hw_ops csid_ops_gen3 = {
	.configure_stream = csid_configure_stream,
	.configure_testgen_pattern = csid_configure_testgen_pattern,
	.hw_version = csid_hw_version,
	.isr = csid_isr,
	.reset = csid_reset,
	.src_pad_code = csid_src_pad_code,
	.subdev_init = csid_subdev_init,
	.reg_update = csid_subdev_reg_update,
	.set_csid_vfe_registers = csid_vfe_set_registers,
	.get_csid_vfe_registers = csid_vfe_get_registers,
};
