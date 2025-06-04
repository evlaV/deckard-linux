// SPDX-License-Identifier: GPL-2.0
/*
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

#define CSID_CSI2_RDIN_SOF_TIMESTAMP_LOWER(rdi) (0x594 + 0x100 * (rdi))
#define CSID_CSI2_RDIN_SOF_TIMESTAMP_UPPER(rdi) (0x598 + 0x100 * (rdi))
#define CSID_CSI2_RDIN_EOF_TIMESTAMP_LOWER(rdi) (0x5a4 + 0x100 * (rdi))
#define CSID_CSI2_RDIN_EOF_TIMESTAMP_UPPER(rdi) (0x5a8 + 0x100 * (rdi))

#define CSID_CSI2_IPP0_SOF_TIMESTAMP_LOWER 0x398
#define CSID_CSI2_IPP0_SOF_TIMESTAMP_UPPER 0x39c
#define CSID_CSI2_IPP0_EOF_TIMESTAMP_LOWER 0x3a8
#define CSID_CSI2_IPP0_EOF_TIMESTAMP_UPPER 0x3ac

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
#define		RDI_CFG0_TIMESTAMP_STB_SEL	8
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

#define NUM_OFFSET_ESTIMATORS 5
#define NUM_OFFSET_ESTIMATORS_NEEDED_IN_AGREEMENT 3
#define MIN_FRAMES_BEFORE_SWITCH 100
#define ESTIMATOR_CHECK_INTERVAL 200
#define DROP_THRESHOLD_NS 1000
#define ESTIMATOR_AGREEMENT_THRESHOLD_NS 1000

DEFINE_SPINLOCK(isp_to_kernel_raw_ts_offset_estimation_spinlock);
u32 isp_to_kernel_raw_ts_offset_stream_cnt = 0;
u64 isp_to_kernel_raw_ts_offset_counter;
s64 isp_to_kernel_raw_ts_offset_floor[NUM_OFFSET_ESTIMATORS];
static int offset_estimator_update_idx = 0;
static int current_active_estimator = 0;
static u64 total_frames_since_reset = 0;

static void check_and_switch_estimators(struct csid_device *csid) {
    int agreement_count;
    int highest_estimator = current_active_estimator;
    s64 highest_estimate = isp_to_kernel_raw_ts_offset_floor[current_active_estimator];
    bool found_good_estimator = false;
    
    // Try each estimator starting with current
    for (int start = 0; start < NUM_OFFSET_ESTIMATORS && !found_good_estimator; start++) {
        int test_estimator = (current_active_estimator + start) % NUM_OFFSET_ESTIMATORS;
        agreement_count = 0;
        
        // Compare against all other estimators
        for (int other = 0; other < NUM_OFFSET_ESTIMATORS; other++) {
            if (other == test_estimator)
                continue;
                
            s64 diff = isp_to_kernel_raw_ts_offset_floor[test_estimator] - 
                      isp_to_kernel_raw_ts_offset_floor[other];
            
            if (abs(diff) < ESTIMATOR_AGREEMENT_THRESHOLD_NS)
                agreement_count++;
        }
        
        // If this estimator agrees with at least 3 others, use it
        if (agreement_count >= NUM_OFFSET_ESTIMATORS_NEEDED_IN_AGREEMENT) {
            if (test_estimator != current_active_estimator) {
                dev_info(csid->camss->dev, "Switching from estimator %d to %d based on agreement\n", 
                       current_active_estimator, test_estimator);
                current_active_estimator = test_estimator;
            }
            found_good_estimator = true;
            break;
        }
        
        // Track highest estimate in case we don't find agreement
        if (isp_to_kernel_raw_ts_offset_floor[test_estimator] > highest_estimate) {
            highest_estimate = isp_to_kernel_raw_ts_offset_floor[test_estimator];
            highest_estimator = test_estimator;
        }
    }
    
    // If no estimator had good agreement, use the one with highest estimate
    if (!found_good_estimator) {
        if (highest_estimator != current_active_estimator) {
            dev_err(csid->camss->dev, "No good agreement found, switching to highest estimator %d\n", highest_estimator);
            current_active_estimator = highest_estimator;
        }
        
        // Reset all other estimators to match the highest
        for (int i = 0; i < NUM_OFFSET_ESTIMATORS; i++) {
            if (i != current_active_estimator)
                isp_to_kernel_raw_ts_offset_floor[i] = LONG_MAX;
        }
    }
}

static void update_isp_to_kernel_raw_ts_offset_estimate(struct csid_device *csid, u64 ts_monotonic_raw, u64 sof_isp_timestamp)
{
	s64 detected_isp_to_kernel_raw_ts_offset = ts_monotonic_raw - sof_isp_timestamp;
	int estimator_idx;

	// Cycle through estimators for each update
	estimator_idx = offset_estimator_update_idx;
	offset_estimator_update_idx = (offset_estimator_update_idx + 1) % NUM_OFFSET_ESTIMATORS;

	isp_to_kernel_raw_ts_offset_counter++;
	total_frames_since_reset++;

	// If the detected offset is lower than the current floor, update the floor
	if (detected_isp_to_kernel_raw_ts_offset < isp_to_kernel_raw_ts_offset_floor[estimator_idx]) {
		s64 drop = isp_to_kernel_raw_ts_offset_floor[estimator_idx] - detected_isp_to_kernel_raw_ts_offset;
		dev_info(csid->camss->dev, "Estimator %d lowered the floor by %lld ns on csid: %d\n", estimator_idx, drop, csid->id);
		
		// Check if we should switch estimators due to large drop
		if (total_frames_since_reset >= MIN_FRAMES_BEFORE_SWITCH && 
		    estimator_idx == current_active_estimator && 
		    drop > DROP_THRESHOLD_NS) {
		    current_active_estimator = (current_active_estimator + 1) % NUM_OFFSET_ESTIMATORS;

		    dev_warn(csid->camss->dev, "Active estimator %d, Large drop detected. Running switch now", current_active_estimator);
			check_and_switch_estimators(csid);
		}
		
		isp_to_kernel_raw_ts_offset_floor[estimator_idx] = detected_isp_to_kernel_raw_ts_offset;
	}

	// Check estimators periodically
	if (isp_to_kernel_raw_ts_offset_counter == ESTIMATOR_CHECK_INTERVAL) {
		isp_to_kernel_raw_ts_offset_counter = 0;
		
		// Print the difference between each estimator and estimator 0
		// for (int j = 1; j < NUM_OFFSET_ESTIMATORS; j++) {
		// 	s64 diff = isp_to_kernel_raw_ts_offset_floor[j] - isp_to_kernel_raw_ts_offset_floor[0];
		// 	dev_info(csid->camss->dev, "Estimator %d diff from 0: %lld ns\n", j, diff);
		// }
		u64 check_start = ktime_get_raw_ns();
		check_and_switch_estimators(csid);
		u64 check_end = ktime_get_raw_ns();
		//dev_info(csid->camss->dev, "Estimator check took %lld ns\n", check_end - check_start);
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
	// RDI_CFG0_TIMESTAMP_STB_SEL (on bits [9,8]) can be PRE_HALT (0x0), POST_HALT (0x1) or POST_IRQ (0x2)
	// Use CSID_TIMESTAMP_STB_POST_IRQ by default
	val |=  2 << RDI_CFG0_TIMESTAMP_STB_SEL;
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

	if(csid->multi_vcdt_en)
	{
		val = 0x1; // EN
		val |= (csid->multi_vcdt_vc & 0x1f) << 2; // VC1
		val |= format->data_type << 7; //DT1
		val |= DECODE_FORMAT_PAYLOAD_ONLY << 16; // DECODE_FORMAT1
		val |= 0x1 << 13; // TS_COMB_VCDT_EN
		writel(val, csid->base + 0x50c + 0x100 * (vc)); // MULTI_VCDT_CFG0
	}

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

	// Reset isp/kernel_raw_ts offset computation when starting streaming
	spin_lock(&isp_to_kernel_raw_ts_offset_estimation_spinlock);
	if(enable && (isp_to_kernel_raw_ts_offset_stream_cnt == 0))
	{
		isp_to_kernel_raw_ts_offset_counter = 0;
		total_frames_since_reset = 0;
		current_active_estimator = 0;
		for (int j = 0; j < NUM_OFFSET_ESTIMATORS; ++j)
			isp_to_kernel_raw_ts_offset_floor[j] = LONG_MAX;
	}
	isp_to_kernel_raw_ts_offset_stream_cnt += (enable ? 1 : -1);
	spin_unlock(&isp_to_kernel_raw_ts_offset_estimation_spinlock);

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
	u32 val = 0;
	if (port_id == CSID_780_CUSTOM_PORT_ID_IPP0) {
		val = BIT(0) | BIT(16) | BIT(28); // RUP_IPP0 | AUP_IPP0 | MUP
	}
	else {
		val = CSID_RUP_AUP_RDI(port_id);
	}

	if (clear) {
		csid->reg_update &= ~val;
	} else {
		csid->reg_update |= val;
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
	u32 top_val, val, buf_done_val;
	u8 reset_done;
	u64 ts_monotonic_raw = ktime_get_raw_ns();
	int i = 0;

	trace_csid_isr(0, csid->id);

	top_val = readl(csid->base + CSID_TOP_IRQ_STATUS);
	writel(top_val, csid->base + CSID_TOP_IRQ_CLEAR);
	reset_done = top_val & TOP_IRQ_STATUS_RESET_DONE;

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

	// Loop through enabled rdi paths
	for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS; i++) {
		if ((csid->phy.en_vc & BIT(i)) || (i == VFE_LINE_PIX)) {

			// read and clear the path status
			u32 path_status = 0;
			if(i == VFE_LINE_PIX) {
				path_status = readl(csid->base + 0xac);
				writel(path_status, csid->base + 0xb4);
			}
			else {
				path_status = readl(csid->base + CSID_CSI2_RDIN_IRQ_STATUS(i));
				writel(path_status, csid->base + CSID_CSI2_RDIN_IRQ_CLEAR(i));
			}

			if(path_status & 0x0010) { // Test if PATH_INFO_SOF is set

				u32 sof_timestamp_lower = 0;
				u32 sof_timestamp_upper = 0;

				if(i == VFE_LINE_PIX) {
					sof_timestamp_lower = readl_relaxed(csid->base + CSID_CSI2_IPP0_SOF_TIMESTAMP_LOWER);
					sof_timestamp_upper = readl_relaxed(csid->base + CSID_CSI2_IPP0_SOF_TIMESTAMP_UPPER);
				}
				else {
					sof_timestamp_lower = readl_relaxed(csid->base + CSID_CSI2_RDIN_SOF_TIMESTAMP_LOWER(i));
					sof_timestamp_upper = readl_relaxed(csid->base + CSID_CSI2_RDIN_SOF_TIMESTAMP_UPPER(i));
				}

				u64 sof_isp_timestamp = ((u64)sof_timestamp_upper) << 32 | sof_timestamp_lower;

				sof_isp_timestamp = mul_u64_u32_div(
					sof_isp_timestamp,
					CAM_IFE_CSID_MUL_FACTOR,
					CAM_IFE_CSID_DIV_FACTOR);

				if(sof_isp_timestamp != 0) {
					// update the isp->kernel_raw timestamp offset estimate with the new SOF measurement
					update_isp_to_kernel_raw_ts_offset_estimate(csid,ts_monotonic_raw, sof_isp_timestamp);

					csid->path_cur_sof_isp_timestamp_converted[i] = sof_isp_timestamp + isp_to_kernel_raw_ts_offset_floor[current_active_estimator];
					if(csid->do_post_sof_work != NULL) {
						u8 group = (sof_timestamp_lower & 1) ? 1 : 0;
						csid->do_post_sof_work(csid->capture_subdev, csid->path_cur_sof_isp_timestamp_converted[i], ts_monotonic_raw, group);
					}

					csid->path_last_sof_kernel_ts[i] = ts_monotonic_raw;
				}
				else
				{
					csid->path_last_sof_kernel_ts[i] = ts_monotonic_raw;
					csid->path_cur_sof_isp_timestamp_converted[i] = 0;
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
		for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS; i++) {
			if ((csid->phy.en_vc & BIT(i)) || (i == VFE_LINE_PIX)) {

				// TODO: check bits
				u32 comp_grp_test_mask = 0;
				u32 path_buf_done_val = 0;
				if(!csid_is_lite(csid) && (i == VFE_LINE_PIX)) {
					comp_grp_test_mask = 0x3fff;
				}
				else {
					u32 rdi_idx = i;
					comp_grp_test_mask = csid_is_lite(csid) ? BIT(1 + rdi_idx) : BIT(14 + rdi_idx);
				}

				// get the buf_done_val with only bits corresponding to this path
				path_buf_done_val = buf_done_val & comp_grp_test_mask;

				// no buf done triggered for this path
				if(!path_buf_done_val)
					continue;

				u32 eof_timestamp_lower = 0;
				u32 eof_timestamp_upper = 0;
				if(i == VFE_LINE_PIX) {
					eof_timestamp_lower = readl_relaxed(csid->base + CSID_CSI2_IPP0_EOF_TIMESTAMP_LOWER);
					eof_timestamp_upper = readl_relaxed(csid->base + CSID_CSI2_IPP0_EOF_TIMESTAMP_UPPER);
				}
				else {
					eof_timestamp_lower = readl_relaxed(csid->base + CSID_CSI2_RDIN_EOF_TIMESTAMP_LOWER(i));
					eof_timestamp_upper = readl_relaxed(csid->base + CSID_CSI2_RDIN_EOF_TIMESTAMP_UPPER(i));
				}
				u64 eof_isp_timestamp = ((u64)eof_timestamp_upper) << 32 | eof_timestamp_lower;

				eof_isp_timestamp = mul_u64_u32_div(
					eof_isp_timestamp,
					CAM_IFE_CSID_MUL_FACTOR,
					CAM_IFE_CSID_DIV_FACTOR);

				u64 cur_eof_isp_timestamp_converted = eof_isp_timestamp + isp_to_kernel_raw_ts_offset_floor[current_active_estimator];

				struct camss_buf_done_info buf_done_info;
				memset(&buf_done_info, 0 ,sizeof(struct camss_buf_done_info));
				// Fill with buf_done info and sof/eof timestamps, including sensor metadata buffer addr and size
				buf_done_info.status = path_buf_done_val; // path buf_done bits only!
				buf_done_info.sof_isp_ts_conv = csid->path_cur_sof_isp_timestamp_converted[i];
				buf_done_info.eof_isp_ts_conv = cur_eof_isp_timestamp_converted;
				buf_done_info.isp_to_kernel_raw_ts_offset = isp_to_kernel_raw_ts_offset_floor[current_active_estimator];
				buf_done_info.sof_kernel_ts = csid->path_last_sof_kernel_ts[i];
				// Provide sensor metadata if available
				if(csid->get_metadata != NULL)
				{
					buf_done_info.sensor_metadata_buffer_addr = csid->get_metadata(csid->capture_subdev );
					buf_done_info.sensor_metadata_buffer_size = csid->metadata_size;
				}

				u32 port_id = i;
				if(i == VFE_LINE_PIX) {
					port_id = 0; // route all IPP events to the RDI0 path until the PIX path is properly enabled
				}

				trace_csid_isr_wm_done(0, csid->id);
				// printk("csid%d calling camss_buf_done for port %d with status 0x%x\n", csid->id, port_id, buf_done_info.status);
				camss_buf_done(csid->camss, csid->id, port_id, &buf_done_info);
			}
		}
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

#if CSID_RX_DEBUG
	// re-configure rx irqs after reset since these got cleared
	writel_relaxed(csid->rx_dbg_en ? 0x9ffffff : 0x0, csid->base + CSID_CSI2_RX_IRQ_MASK);
#endif

	/* PATH_INFO_SOF | CAMIF_SOF */

	val = 0x1010;
	//val = 0x10881004;
	writel_relaxed(val, csid->base + CSID_CSI2_RDIN_IRQ_MASK(0));
	writel_relaxed(val, csid->base + 0xb0); // IPP SOF

	return 0;
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
	const char* name;
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
	csid->testgen.nmodes = CSID_PAYLOAD_MODE_DISABLED;
#if CSID_RX_DEBUG
	int ret = 0;
	// Add attributes to enable/retrieve csi2 rx statistics
	csid->rx_dbg_en_attr = (struct device_attribute)__ATTR_RW(rx_dbg_en);
	csid->rx_dbg_stats_attr = (struct device_attribute)__ATTR_RO(rx_dbg_stats);
	csid->rx_dbg_attrs[0] = &csid->rx_dbg_en_attr.attr;
	csid->rx_dbg_attrs[1] = &csid->rx_dbg_stats_attr.attr;
	csid->rx_dbg_attrs[2] = NULL;
	snprintf(csid->rx_dbg_attr_group_name, sizeof(csid->rx_dbg_attr_group_name), "csid%d", csid->id);
	csid->rx_dbg_attr_group.name = csid->rx_dbg_attr_group_name;
	csid->rx_dbg_attr_group.attrs = csid->rx_dbg_attrs;
	ret = devm_device_add_group(csid->camss->dev, &csid->rx_dbg_attr_group);
	if(ret) {
		dev_err(csid->camss->dev, "Failed to setup CSI2 RX debug attributes");
	}
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
