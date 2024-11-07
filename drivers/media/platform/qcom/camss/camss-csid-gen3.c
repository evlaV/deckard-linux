// SPDX-License-Identifier: GPL-2.0
/*
 * camss-csid-gen3.c
 *
 * Qualcomm MSM Camera Subsystem - CSID (CSI Decoder) Module
 *
 * Copyright (c) 2024 Qualcomm Technologies, Inc.
 */
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>

#include "camss.h"
#include "camss-csid.h"
#include "camss-csid-gen3.h"

#define CREATE_TRACE_POINTS
#include "trace.h"

#define CSID_TOP_IO_PATH_CFG0(csid)	(0x4 * (csid))
#define		OUTPUT_IFE_EN			0x100
#define		INTERNAL_CSID			1

#define CSID_RST_CFG			0xC
#define		RST_MODE			0
#define		RST_LOCATION			4

#define CSID_RST_CMD			0x10
#define		SELECT_HW_RST			0
#define		SELECT_SW_RST			1
#define		SELECT_IRQ_RST			2

#define CSID_CSI2_RX_IRQ_STATUS		0x9C
#define CSID_CSI2_RX_IRQ_MASK		0xA0
#define CSID_CSI2_RX_IRQ_CLEAR		0xA4
#define CSID_CSI2_RX_IRQ_SET		0xA8

#define CSID_TOP_IRQ_STATUS		0x7C
#define		 TOP_IRQ_STATUS_RESET_DONE	0

#define CSID_TOP_IRQ_MASK		0x80
#define CSID_TOP_IRQ_CLEAR		0x84
#define CSID_TOP_IRQ_SET		0x88

#define CSID_IRQ_CMD			0x14
#define		IRQ_CMD_CLEAR			0
#define		IRQ_CMD_SET			4

#define CSID_REG_UPDATE_CMD		0x18

#define CSID_BUF_DONE_IRQ_STATUS	0x8C
#define		BUF_DONE_IRQ_STATUS_RDI_OFFSET	(csid_is_lite(csid) ? 1 : 14)
#define CSID_BUF_DONE_IRQ_MASK		0x90
#define CSID_BUF_DONE_IRQ_CLEAR		0x94
#define CSID_BUF_DONE_IRQ_SET		0x98

#define	CSI2_RX_CFG0_PHY_SEL_BASE_IDX	1

#define CSID_CSI2_RX_CFG0		0x200
#define		CSI2_RX_CFG0_NUM_ACTIVE_LANES	0
#define		CSI2_RX_CFG0_DL0_INPUT_SEL	4
#define		CSI2_RX_CFG0_PHY_NUM_SEL	20

#define CSID_CSI2_RX_CFG1		0x204
#define		CSI2_RX_CFG1_ECC_CORRECTION_EN	0
#define		CSI2_RX_CFG1_VC_MODE		2
#define		CSI2_RX_CFG1_MISR_EN		6

#define		PATH_CFG0_TIMESTAMP_EN		6
#define		PATH_CFG0_TIMESTAMP_STB_SEL	8
#define		PATH_CFG0_DECODE_FORMAT		12
#define		PATH_CFG0_DT			16
#define		PATH_CFG0_VC			22
#define		PATH_CFG0_DT_ID			27
#define		PATH_CFG0_EN			31

#define		PATH_CFG1_DROP_H_EN		5
#define		PATH_CFG1_DROP_V_EN		6
#define		PATH_CFG1_CROP_H_EN		7
#define		PATH_CFG1_CROP_V_EN		8
#define		PATH_CFG1_PIX_STORE		10
#define		PATH_CFG1_PACKING_FORMAT		15

#define		PATH_CTRL_START_CMD		0

#define 	PATH_MULTI_VCDT_CFG0_EN				0
#define 	PATH_MULTI_VCDT_CFG0_VC				2
#define 	PATH_MULTI_VCDT_CFG0_DT				7
#define 	PATH_MULTI_VCDT_CFG0_TS_COMB_EN		13
#define 	PATH_MULTI_VCDT_CFG0_DECODE_FORMAT	16

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

static void update_isp_to_kernel_raw_ts_offset_estimate(u64 ts_monotonic_raw, u64 sof_isp_timestamp)
{
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
}

static int csid_reset(struct csid_device *csid);

static inline int reg_update_rdi(struct csid_device *csid, int n)
{
	/*  RUP_RDI0 | AUP_RDI0 | MUP */
	return BIT(n + 4) + BIT(20 + n) + BIT(28);
}
#define REG_UPDATE_RDI			reg_update_rdi

static void __csid_configure_rx(struct csid_device *csid,
				struct csid_phy_config *phy, u32 vc_mode)
{
	int val;

	val = (phy->lane_cnt - 1) << CSI2_RX_CFG0_NUM_ACTIVE_LANES;
	val |= phy->lane_assign << CSI2_RX_CFG0_DL0_INPUT_SEL;
	val |= (phy->csiphy_id + CSI2_RX_CFG0_PHY_SEL_BASE_IDX) << CSI2_RX_CFG0_PHY_NUM_SEL;

	writel(val, csid->base + CSID_CSI2_RX_CFG0);

	val = 1 << CSI2_RX_CFG1_ECC_CORRECTION_EN;
	val |= 1 << CSI2_RX_CFG1_MISR_EN;
	val |= vc_mode << CSI2_RX_CFG1_VC_MODE;
	writel(val, csid->base + CSID_CSI2_RX_CFG1);
}

static void __csid_ctrl_path(struct csid_device *csid, int enable, enum camss_ext_csid_path_type path_type)
{
	int val = 0;

	if (enable)
		val = 1 << PATH_CTRL_START_CMD;

	writel(val, csid->base + csid_path_regs[path_type].ctrl);

	if(path_type == CAMSS_EXT_CSID_PATH_IPP0)
		camss_reg_update(csid->camss, 0, -4, true);
	else
		camss_reg_update(csid->camss, 0, 0, true);
}

static void __csid_configure_top(struct csid_device *csid)
{
	u32 val;

	/* csid lite doesn't need to configure top register */
	if (csid->res->is_lite)
		return;

	/* CSID top is a new function in Titan780.
	 * CSID can connect to VFE & SFE(Sensor Front End).
	 * This connection is controlled by CSID top.
	 * Only enable VFE path in current driver.
	 */
	val = OUTPUT_IFE_EN | INTERNAL_CSID;
	writel(val, csid->camss->csid_top_base + CSID_TOP_IO_PATH_CFG0(csid->id));
}

static void __csid_configure_path_stream(struct csid_device *csid, u8 enable, enum camss_ext_csid_path_type path_type, struct csid_path_config *path_config)
{
	u32 val;
	u32 decode_format;
	/* Source pads matching RDI channels on hardware. Pad 1 -> RDI0, Pad 2 -> RDI1, etc. */
	struct v4l2_mbus_framefmt *input_format = &csid->fmt[MSM_CSID_PAD_FIRST_SRC + path_config->owner_stream_idx];
	const struct csid_format_info *format = csid_get_fmt_entry(csid->res->formats->formats,
								   csid->res->formats->nformats,
								   input_format->code);

	// payload-only for RDI paths
	decode_format = (path_type == CAMSS_EXT_CSID_PATH_IPP0) ? format->decode_format : DECODE_FORMAT_PAYLOAD_ONLY;

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

	val = 1 << PATH_CFG0_TIMESTAMP_EN;
	/*
	val |= 1 << RDI_CFG0_TIMESTAMP_STB_SEL;
	*/
	/* RDI_CFG0_SOF_RETIME_DIS */
	val |= 1 << 5;
	val |= decode_format << PATH_CFG0_DECODE_FORMAT;
	val |= path_config->vc << PATH_CFG0_VC;
	val |= format->data_type << PATH_CFG0_DT;
	val |= path_config->dt_id << PATH_CFG0_DT_ID;

	writel(val, csid->base + csid_path_regs[path_type].cfg0);

	val = 1 << PATH_CFG1_PACKING_FORMAT;
	val |= 1 << PATH_CFG1_PIX_STORE;
	val |= 1 << PATH_CFG1_DROP_H_EN;
	val |= 1 << PATH_CFG1_DROP_V_EN;
	val |= 1 << PATH_CFG1_CROP_H_EN;
	val |= 1 << PATH_CFG1_CROP_V_EN;

	writel(val, csid->base + csid_path_regs[path_type].cfg1);

	if(path_config->multi_vcdt_en) {
		val = 0x1 << PATH_MULTI_VCDT_CFG0_EN;
		val |= path_config->multi_vcdt_vc << PATH_MULTI_VCDT_CFG0_VC;
		val |= format->data_type << PATH_MULTI_VCDT_CFG0_DT;
		val |= decode_format << PATH_MULTI_VCDT_CFG0_DECODE_FORMAT;
		val |= 0x1 << PATH_MULTI_VCDT_CFG0_TS_COMB_EN;
		writel(val, csid->base + csid_path_regs[path_type].multi_vcdt_cfg0);
	}

	val = 0;
	writel(val, csid->base + csid_path_regs[path_type].irq_subsample_period);

	val = 1;
	writel(val, csid->base + csid_path_regs[path_type].irq_subsample_pattern);

	val = 0;
	writel(val, csid->base + csid_path_regs[path_type].ctrl);

	val = readl(csid->base + csid_path_regs[path_type].cfg0);

	if (enable)
		val |= 1 << PATH_CFG0_EN;
	writel(val, csid->base + csid_path_regs[path_type].cfg0);

	// enable SOF IRQs for this path
	val = 0x1010; // PATH_INFO_SOF | CAMIF_SOF
	writel_relaxed(val, csid->base + csid_path_regs[path_type].irq_mask);
}

static void csid_configure_stream(struct csid_device *csid, u8 enable)
{
	u8 i;

	csid->stream_count += (enable ? 1 : -1);

	if((enable && csid->stream_count!=1) || (!enable && (csid->stream_count!=0))) {
		// NOTE: Taking a big shortcut here, we totally ignore configure_stream calls that are not the first enable
		// or the last disable request. We don't know for which source pad the stream was started or stopped
		// so we have to assume that we have the full config when the first CSID user call STREAMON. If we knew which stream 
		// started/stopped we could dynamically enable/disable CSID paths, be more flexible and avoid wasting resources... 
		return;
	}

	if (!enable) {
		csid_reset(csid);
		struct camss_buf_done_info buf_done_info;
		memset(&buf_done_info, 0 ,sizeof(struct camss_buf_done_info));
		camss_buf_done(csid->camss, csid->id, 0, &buf_done_info);
		usleep_range(10000, 20000);
	}

	if (enable)
		__csid_configure_top(csid);

	if(enable && isp_to_kernel_raw_ts_offset_stream_cnt == 0) {
		// reset isp->monotonic_raw offset estimation data (shared between csids since all csid hw use the same clock)
		isp_to_kernel_raw_ts_offset_estimation_state = OFFSET_ESTIMATION_STATE_INIT;
		isp_to_kernel_raw_ts_offset_counter = 0;
		isp_to_kernel_raw_ts_offset_refinement_vote_cnt = 0;
		isp_to_kernel_raw_ts_offset_floor = LONG_MAX;
	}
	isp_to_kernel_raw_ts_offset_stream_cnt += (enable ? 1 : -1);

	// If we don't have the extended configuration fill the paths config with a default one based on the active media links
	if(!csid->has_extended_cfg) {
		memset(csid->extended_cfg_paths, 0, sizeof(csid->extended_cfg_paths));
		for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS; i++) {
			//csid->phy.en_vc = 1;
			if (csid->phy.en_vc & BIT(i)) {
				enum camss_ext_csid_path_type path = CAMSS_EXT_CSID_PATH_RDI0 + i;
				struct csid_path_config path_cfg = {0};
				path_cfg.en = true;
				path_cfg.owner_stream_idx = i;
				path_cfg.vc = i;
				path_cfg.dt_id = i & 0x3;
				// TODO: HACK FOR XRSERVICE COMPAT, remove when appropriate
				if(path == CAMSS_EXT_CSID_PATH_RDI0) {
					path_cfg.multi_vcdt_en = true;
					path_cfg.multi_vcdt_vc = 1;
				}
				path_cfg.trigger_post_sof_work = true; // always trigger sof work by default
				csid->extended_cfg_paths[path] = path_cfg;
			}
		}
	}

	for(int i=0; i<CAMSS_EXT_CSID_PATH_TYPE_MAX; i++) {
		if(csid->extended_cfg_paths[i].en) {
			__csid_configure_path_stream(csid, enable, i, &csid->extended_cfg_paths[i]);
		}
	}

	// we need this extra loop since we cannot configure paths while one is active
	for(int i=0; i<CAMSS_EXT_CSID_PATH_TYPE_MAX; i++) {
		if(csid->extended_cfg_paths[i].en) {
			__csid_ctrl_path(csid, enable, i);
		}
	}

	u32 vc_mode = 0;
	for(int i=0; i<CAMSS_EXT_CSID_PATH_TYPE_MAX; i++) {
		if(csid->extended_cfg_paths[i].en && (csid->extended_cfg_paths[i].vc > 3 || 
			(csid->extended_cfg_paths[i].multi_vcdt_en && csid->extended_cfg_paths[i].multi_vcdt_vc > 3))) {
			vc_mode = 1;
		}
	}
	__csid_configure_rx(csid, &csid->phy, vc_mode);
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
	int i = 0;

	trace_csid_isr(0, csid->id);

	top_val = readl(csid->base + CSID_TOP_IRQ_STATUS);
	writel(top_val, csid->base + CSID_TOP_IRQ_CLEAR);
	reset_done = top_val & BIT(TOP_IRQ_STATUS_RESET_DONE);

	val = readl(csid->base + CSID_CSI2_RX_IRQ_STATUS);
	writel(val, csid->base + CSID_CSI2_RX_IRQ_CLEAR);

#if CSID_RX_DEBUG
	// increment the rx debug irq counters when enabled and we received a csi2 rx irq
	if(csid->rx_dbg_en && top_val & 0x4) {
		// increment counters based on irq status
		for(int bit=0; bit<32; bit++) {
			if(val & BIT(bit))
				csid->rx_dbg_irq_counters[bit]++;
		}
	}
#endif

	for(int path_type=0; path_type<CAMSS_EXT_CSID_PATH_TYPE_MAX; path_type++) {
		// always check IPP0 for compatibility with apps that used this assumption
		// TODO: HACK FOR XRSERVICE COMPAT, remove IPP0 check
		if(csid->extended_cfg_paths[path_type].en || (path_type == CAMSS_EXT_CSID_PATH_IPP0)) {
			// read irq status
			u32 path_status = readl(csid->base + csid_path_regs[path_type].irq_status);
			writel(path_status, csid->base + csid_path_regs[path_type].irq_clear);

			if(path_status & 0x0010) { // Test if PATH_INFO_SOF is set
				u32 sof_timestamp_lower = readl_relaxed(csid->base + csid_path_regs[path_type].curr_sof_lower);
				u32 sof_timestamp_upper = readl_relaxed(csid->base + csid_path_regs[path_type].curr_sof_upper);
				u64 sof_isp_timestamp = ((u64)sof_timestamp_upper) << 32 | sof_timestamp_lower;

				sof_isp_timestamp = mul_u64_u32_div(
					sof_isp_timestamp,
					CAM_IFE_CSID_MUL_FACTOR,
					CAM_IFE_CSID_DIV_FACTOR);

				if(sof_isp_timestamp != 0) {
					// update the isp->kernel_raw timestamp offset estimate with the new SOF measurement
					update_isp_to_kernel_raw_ts_offset_estimate(ts_monotonic_raw, sof_isp_timestamp);

					csid->path_cur_sof_isp_timestamp_converted[path_type] = sof_isp_timestamp + isp_to_kernel_raw_ts_offset_floor;

					if(csid->do_post_sof_work != NULL && (csid->extended_cfg_paths[path_type].trigger_post_sof_work || path_type == CAMSS_EXT_CSID_PATH_IPP0)) {
						u8 group = (sof_timestamp_lower & 1) ? 1 : 0;
						csid->do_post_sof_work(csid->capture_subdev, csid->path_cur_sof_isp_timestamp_converted[path_type], ts_monotonic_raw, group);
					}

					csid->path_last_sof_kernel_ts[path_type] = ts_monotonic_raw;
				}
				else
				{
					csid->path_last_sof_kernel_ts[path_type] = ts_monotonic_raw;
					csid->path_cur_sof_isp_timestamp_converted[path_type] = 0;
				}

				trace_csid_isr_sof(1, csid->id);
			}
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

			if (buf_done_val & BIT(BUF_DONE_IRQ_STATUS_RDI_OFFSET + i)) {
				/* For Titan 780, Buf Done IRQ&REG has been moved to CSID from VFE.
				 * Once CSID received Buf Done, need notify this event to VFE.
				 * Trigger VFE to handle Buf Done process.
				 */
				camss_buf_done(csid->camss, csid->id, i);
			}
		}
#endif

	if (top_val & 0x2000) {
		buf_done_val = readl(csid->base + CSID_BUF_DONE_IRQ_STATUS);
		writel(buf_done_val, csid->base + CSID_BUF_DONE_IRQ_CLEAR);
		// printk("got buf_done_val 0x%x\n", buf_done_val);

#if CSID_RX_DEBUG
		if(csid->rx_dbg_en) {
			// NOTE: Enabling CSI2 RX IRQs seems to cause some issue where streaming works properly but we randomly
			// miss some of the bits in BUF_DONE_STATUS after the BUF_DONE bit has been set in the top IRQ status
			// For the moment we set all the buf done bits to compensate for that
			buf_done_val = 0xffffffff;
		}
#endif

		// check all paths to see if the buf_done status corresponds to one of it output
		// if that's the case fill up the eof info and trigger the vfe wm_done with only
		// the relevant bits in the buf_done_info.status variable
		for(int path_type=0; path_type<CAMSS_EXT_CSID_PATH_TYPE_MAX; path_type++) {

			// skip disabled paths
			// TODO: HACK FOR XRSERVICE COMPAT, remove IPP0 check
			if(csid->extended_cfg_paths[path_type].en || (path_type==CAMSS_EXT_CSID_PATH_IPP0))
			{
				u32 comp_grp_test_mask = 0;
				u32 path_buf_done_val = 0;
				if(!csid_is_lite(csid) && path_type == CAMSS_EXT_CSID_PATH_IPP0) {
					comp_grp_test_mask = 0x3fff;
				}
				else {
					u32 rdi_idx = path_type - CAMSS_EXT_CSID_PATH_RDI0;
					comp_grp_test_mask = csid_is_lite(csid) ? BIT(1 + rdi_idx) : BIT(14 + rdi_idx);
				}

				// get the buf_done_val with only bits corresponding to this path
				path_buf_done_val = buf_done_val & comp_grp_test_mask;

				// no buf done triggered for this path
				if(!path_buf_done_val)
					continue;

				u32 eof_timestamp_lower = readl_relaxed(csid->base + csid_path_regs[path_type].curr_eof_lower);
				u32 eof_timestamp_upper = readl_relaxed(csid->base + csid_path_regs[path_type].curr_eof_upper);
				u64 eof_isp_timestamp = ((u64)eof_timestamp_upper) << 32 | eof_timestamp_lower;

				eof_isp_timestamp = mul_u64_u32_div(
					eof_isp_timestamp,
					CAM_IFE_CSID_MUL_FACTOR,
					CAM_IFE_CSID_DIV_FACTOR);

				u64 cur_eof_isp_timestamp_converted = eof_isp_timestamp + isp_to_kernel_raw_ts_offset_floor;

				struct camss_buf_done_info buf_done_info;
				memset(&buf_done_info, 0 ,sizeof(struct camss_buf_done_info));
				// Fill with buf_done info and sof/eof timestamps, including sensor metadata buffer addr and size
				buf_done_info.status = path_buf_done_val; // path buf_done bits only!
				buf_done_info.sof_isp_ts_conv = csid->path_cur_sof_isp_timestamp_converted[path_type];
				buf_done_info.eof_isp_ts_conv = cur_eof_isp_timestamp_converted;
				buf_done_info.isp_to_kernel_raw_ts_offset = isp_to_kernel_raw_ts_offset_floor;
				buf_done_info.sof_kernel_ts = csid->path_last_sof_kernel_ts[path_type];
				// Provide sensor metadata if available
				if(csid->get_metadata != NULL)
				{
					buf_done_info.sensor_metadata_buffer_addr = csid->get_metadata(csid->capture_subdev );
					buf_done_info.sensor_metadata_buffer_size = csid->metadata_size;
				}

				trace_csid_isr_wm_done(0, csid->id);
				// printk("csid%d calling camss_buf_done for path %d with status 0x%x\n", csid->id, path_type, buf_done_info.status);
				camss_buf_done(csid->camss, csid->id, 0, &buf_done_info);
			}
		}
	}

	val = 1 << IRQ_CMD_CLEAR;
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
			writel(0x1 << IRQ_CMD_CLEAR, csid->base + CSID_IRQ_CMD);
			writel(BIT(BUF_DONE_IRQ_STATUS_RDI_OFFSET + i),
						csid->base + CSID_BUF_DONE_IRQ_MASK);
		}
#endif

	/* preserve registers */
	val = (0x1 << RST_LOCATION) | (0x1 << RST_MODE);
	writel(val, csid->base + CSID_RST_CFG);

	/* val = (0x1 << SELECT_HW_RST) | (0x1 << SELECT_IRQ_RST); */
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

#if CSID_RX_DEBUG
	// re-configure rx irqs after reset since these got cleared
	writel_relaxed(csid->rx_dbg_en ? 0x9ffffff : 0x0, csid->base + CSID_CSI2_RX_IRQ_MASK);
#endif

	// TODO: HACK FOR XRSERVICE COMPAT, that's properly configured in that in __csid_configure_path_stream for enabled paths now, remove that
	// once we stop assuming that we always have IPP0 IRQs
	/* PATH_INFO_SOF | CAMIF_SOF */
	val = 0x1010;
	//val = 0x10881004;
	writel_relaxed(val, csid->base + csid_path_regs[CAMSS_EXT_CSID_PATH_RDI0].irq_mask);
	writel_relaxed(val, csid->base + csid_path_regs[CAMSS_EXT_CSID_PATH_IPP0].irq_mask);

	return 0;
}

static void csid_subdev_reg_update(struct csid_device *csid, int port_id, bool is_clear)
{
	if (is_clear) {
		csid->reg_update &= ~REG_UPDATE_RDI(csid, port_id);
	} else {
		csid->reg_update |= REG_UPDATE_RDI(csid, port_id);
		writel(csid->reg_update, csid->base + CSID_REG_UPDATE_CMD);
	}
}

#if CSID_RX_DEBUG
static ssize_t rx_dbg_en_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct csid_device *csid = container_of(attr, struct csid_device, rx_dbg_en_attr);
	return sysfs_emit(buf, "%d\n", csid->rx_dbg_en);
}

static ssize_t rx_dbg_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct csid_device *csid = container_of(attr, struct csid_device, rx_dbg_en_attr);

	if(!count || (buf[0]!='0' && buf[0]!='1'))
		return -EINVAL;

	bool en = (buf[0]=='1');

	if(csid->rx_dbg_en != en) {
		csid->rx_dbg_en = en;

		// configure rx irqs
		writel_relaxed(csid->rx_dbg_en ? 0x9ffffff : 0x0, csid->base + CSID_CSI2_RX_IRQ_MASK);

		if(csid->rx_dbg_en) {
			// reset debug rx irq counters only when rx debug is enabled
			memset(csid->rx_dbg_irq_counters, 0, sizeof(csid->rx_dbg_irq_counters));
		}

		dev_warn(csid->camss->dev, "csid%d csi2 rx stats enabled=%d\n", csid->id, csid->rx_dbg_en);
	}

	return count;
}

struct csid_rx_stats_info {
	u32 bit;
	const char* name
};

struct csid_rx_stats_info rx_stats_info[] = {
	{0, "DL0_EOT_CAPTURED"}, {1, "DL1_EOT_CAPTURED"}, {2, "DL2_EOT_CAPTURED"}, {3, "DL3_EOT_CAPTURED"},
	{4, "DL0_SOT_CAPTURED"}, {5, "DL1_SOT_CAPTURED"}, {6, "DL2_SOT_CAPTURED"}, {7, "DL3_SOT_CAPTURED"},
	{8, "LONG_PKT_CAPTURED"}, {9, "SHORT_PKT_CAPTURED"}, {10, "CPHY_PKT_HDR_CAPTURED"}, {11, "CPHY_EOT_RECEPTION"},
	{12, "CPHY_SOT_RECEPTION"}, {13, "ERROR_CPHY_PH_CRC"}, {14, "WARNING_ECC"}, {15, "LANE0_FIFO_OVERFLOW"},
	{16, "LANE1_FIFO_OVERFLOW"}, {17, "LANE2_FIFO_OVERFLOW"}, {18, "LANE3_FIFO_OVERFLOW"}, {19, "ERROR_CRC"},
	{20, "ERROR_ECC"}, {20, "MMAPPED_VC_DT"}, {20, "UNMAPPED_VC_DT"}, {20, "STREAM_UNDERFLOW"},
	{24, "UNBOUNDED_FRAME"}, {27, "RST_DONE"}
};

static ssize_t rx_dbg_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct csid_device *csid = container_of(attr, struct csid_device, rx_dbg_stats_attr);
	int i;
	u32 count = 0;

	for(i=0; i<ARRAY_SIZE(rx_stats_info); i++) {
		count += sysfs_emit_at(buf, count, "%s;%u\n", rx_stats_info[i].name, csid->rx_dbg_irq_counters[rx_stats_info[i].bit]);
	}

	return count;
}
#endif

static void csid_subdev_init(struct csid_device *csid)
{
#if CSID_RX_DEBUG
	// Add attributes to enable/retrieve csi2 rx statistics
	csid->rx_dbg_en_attr = (struct device_attribute)__ATTR_RW(rx_dbg_en);
	csid->rx_dbg_stats_attr = (struct device_attribute)__ATTR_RO(rx_dbg_stats);
	csid->rx_dbg_attrs[0] = &csid->rx_dbg_en_attr.attr;
	csid->rx_dbg_attrs[1] = &csid->rx_dbg_stats_attr.attr;
	csid->rx_dbg_attrs[2] = NULL;
	snprintf(csid->rx_dbg_attr_group_name, sizeof(csid->rx_dbg_attr_group_name), "csid%d", csid->id);
	csid->rx_dbg_attr_group.name = csid->rx_dbg_attr_group_name;
	csid->rx_dbg_attr_group.attrs = csid->rx_dbg_attrs;
	devm_device_add_group(csid->camss->dev, &csid->rx_dbg_attr_group);
#endif
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

int csid_set_extended_config_for_pad(struct csid_device *csid, struct camss_ext_config *config, int pad_idx)
{
	int i;
	u32 src_stream_idx;

	if(pad_idx < MSM_CSID_PAD_FIRST_SRC || pad_idx >= (MSM_CSID_PAD_FIRST_SRC | MSM_CSID_MAX_SRC_STREAMS)) {
		dev_err(csid->camss->dev, "Invalid pad index %d in csid_set_extended_config_for_pad", pad_idx);
		return -EINVAL;
	}

	src_stream_idx = pad_idx - MSM_CSID_PAD_FIRST_SRC;

	// first clear all enabled paths for this src_stream_idx
	for(i=0; i<CAMSS_EXT_CSID_PATH_TYPE_MAX; i++) {
		// if the src_stream already had this path configured fully clear it since this config will overwrite the previous one
		if(csid->extended_cfg_paths[i].owner_stream_idx == src_stream_idx) {
			memset(&csid->extended_cfg_paths[i], 0, sizeof(struct csid_path_config));
		}

		// if new config enable a csid path save that to the internal config
		if(config->csid_path_configs[i].en) {
			// abort configuration if the path was already enabled by another stream
			if(csid->extended_cfg_paths[i].en) {
				dev_err(csid->camss->dev, "csid(%d) source pad %d tried to acquire path %d already in use by source pad %d",
					csid->id, src_stream_idx, i, csid->extended_cfg_paths[i].owner_stream_idx);
				return -EINVAL;
			}

			if(i == CAMSS_EXT_CSID_PATH_IPP0 && csid_is_lite(csid)) {
				dev_err(csid->camss->dev, "csid(%d) source pad %d tried to acquire IPP0 path that is not supported on lite csids", csid->id, src_stream_idx);
				return -EINVAL;
			}

			csid->extended_cfg_paths[i].en = true;
			csid->extended_cfg_paths[i].owner_stream_idx = src_stream_idx;
			csid->extended_cfg_paths[i].vc = config->csid_path_configs[i].vc;
			csid->extended_cfg_paths[i].dt_id = config->csid_path_configs[i].dt_id;
			csid->extended_cfg_paths[i].multi_vcdt_en = config->csid_path_configs[i].multi_vcdt_en;
			csid->extended_cfg_paths[i].multi_vcdt_vc = config->csid_path_configs[i].multi_vcdt_vc;
			csid->extended_cfg_paths[i].trigger_post_sof_work = config->csid_path_configs[i].trigger_post_sof_work;
		}
	}

	// will cause csid_configure_stream to use the extended configuration(s) instead of the default rdi only config
	csid->has_extended_cfg = true;

	return 0;
}

const struct csid_hw_ops csid_ops_gen3 = {
	/* No testgen pattern hw in csid gen3 HW */
	.configure_testgen_pattern = NULL,
	.configure_stream = csid_configure_stream,
	.hw_version = csid_hw_version,
	.isr = csid_isr,
	.reset = csid_reset,
	.src_pad_code = csid_src_pad_code,
	.subdev_init = csid_subdev_init,
	.reg_update = csid_subdev_reg_update,
	.set_csid_vfe_registers = csid_vfe_set_registers,
	.get_csid_vfe_registers = csid_vfe_get_registers,
	.set_extended_config_for_pad = csid_set_extended_config_for_pad,
};
