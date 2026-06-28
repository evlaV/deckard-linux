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

#include "camss-csid.h"
#include "camss-csid-gen2.h"
#include "camss-csid-480.h"
#include "camss.h"

static void __csid_configure_rx(struct csid_device *csid,
				struct csid_phy_config *phy, int vc)
{
	u8 lane_cnt = csid->phy.lane_cnt;
	int val;

	if (!lane_cnt)
		lane_cnt = 4;

	val = (lane_cnt - 1) << CSI2_RX_CFG0_NUM_ACTIVE_LANES;
	val |= phy->lane_assign << CSI2_RX_CFG0_DL0_INPUT_SEL;
	val |= phy->csiphy_id << CSI2_RX_CFG0_PHY_NUM_SEL;
	writel_relaxed(val, csid->base + CSID_CSI2_RX_CFG0);

	val = 1 << CSI2_RX_CFG1_PACKET_ECC_CORRECTION_EN;
	if (vc > 3)
		val |= 1 << CSI2_RX_CFG1_VC_MODE;
	val |= 1 << CSI2_RX_CFG1_MISR_EN;
	writel_relaxed(val, csid->base + CSID_CSI2_RX_CFG1);
}

static void __csid_ctrl_rdi(struct csid_device *csid, int enable, u8 rdi)
{
	int val;

	if (enable)
		val = HALT_CMD_RESUME_AT_FRAME_BOUNDARY << RDI_CTRL_HALT_CMD;
	else
		val = HALT_CMD_HALT_AT_FRAME_BOUNDARY << RDI_CTRL_HALT_CMD;
	writel_relaxed(val, csid->base + CSID_RDI_CTRL(rdi));
}

static void __csid_configure_testgen(struct csid_device *csid, u8 enable, u8 port, u8 vc)
{
	struct csid_testgen_config *tg = &csid->testgen;
	struct v4l2_mbus_framefmt *input_format = &csid->fmt[MSM_CSID_PAD_FIRST_SRC + port];
	const struct csid_format_info *format = csid_get_fmt_entry(csid->res->formats->formats,
								   csid->res->formats->nformats,
								   input_format->code);
	u8 lane_cnt = csid->phy.lane_cnt;
	u32 val;

	if (!lane_cnt)
		lane_cnt = 4;

	/* configure one DT, infinite frames */
	val = vc << TPG_VC_CFG0_VC_NUM;
	val |= INTELEAVING_MODE_ONE_SHOT << TPG_VC_CFG0_LINE_INTERLEAVING_MODE;
	val |= 0 << TPG_VC_CFG0_NUM_FRAMES;
	writel_relaxed(val, csid->base + CSID_TPG_VC_CFG0);

	val = 0x740 << TPG_VC_CFG1_H_BLANKING_COUNT;
	val |= 0x3ff << TPG_VC_CFG1_V_BLANKING_COUNT;
	writel_relaxed(val, csid->base + CSID_TPG_VC_CFG1);

	writel_relaxed(0x12345678, csid->base + CSID_TPG_LFSR_SEED);

	val = (input_format->height & 0x1fff) << TPG_DT_n_CFG_0_FRAME_HEIGHT;
	val |= (input_format->width & 0x1fff) << TPG_DT_n_CFG_0_FRAME_WIDTH;
	writel_relaxed(val, csid->base + CSID_TPG_DT_n_CFG_0(0));

	val = format->data_type << TPG_DT_n_CFG_1_DATA_TYPE;
	writel_relaxed(val, csid->base + CSID_TPG_DT_n_CFG_1(0));

	val = (tg->mode - 1) << TPG_DT_n_CFG_2_PAYLOAD_MODE;
	val |= 0xBE << TPG_DT_n_CFG_2_USER_SPECIFIED_PAYLOAD;
	val |= format->decode_format << TPG_DT_n_CFG_2_ENCODE_FORMAT;
	writel_relaxed(val, csid->base + CSID_TPG_DT_n_CFG_2(0));

	writel_relaxed(0, csid->base + CSID_TPG_COLOR_BARS_CFG);

	writel_relaxed(0, csid->base + CSID_TPG_COLOR_BOX_CFG);

	val = enable << TPG_CTRL_TEST_EN;
	val |= 1 << TPG_CTRL_FS_PKT_EN;
	val |= 1 << TPG_CTRL_FE_PKT_EN;
	val |= (lane_cnt - 1) << TPG_CTRL_NUM_ACTIVE_LANES;
	val |= 0x64 << TPG_CTRL_CYCLES_BETWEEN_PKTS;
	val |= 0xA << TPG_CTRL_NUM_TRAIL_BYTES;
	writel_relaxed(val, csid->base + CSID_TPG_CTRL);
}

static void __csid_configure_rdi_stream(struct csid_device *csid, u8 enable, u8 port, u8 vc)
{
	/* Source pads matching RDI channels on hardware. Pad 1 -> RDI0, Pad 2 -> RDI1, etc. */
	struct v4l2_mbus_framefmt *input_format = &csid->fmt[MSM_CSID_PAD_FIRST_SRC + port];
	const struct csid_format_info *format = csid_get_fmt_entry(csid->res->formats->formats,
								   csid->res->formats->nformats,
								   input_format->code);
	u32 val;

	/*
	 * DT_ID is a two bit bitfield that is concatenated with
	 * the four least significant bits of the five bit VC
	 * bitfield to generate an internal CID value.
	 *
	 * CSID_RDI_CFG0(port)
	 * DT_ID : 28:27
	 * VC    : 26:22
	 * DT    : 21:16
	 *
	 * CID   : VC 3:0 << 2 | DT_ID 1:0
	 */
	u8 dt_id = port & 0x03;

	val = 1 << RDI_CFG0_BYTE_CNTR_EN;
	val |= 1 << RDI_CFG0_FORMAT_MEASURE_EN;
	val |= 1 << RDI_CFG0_TIMESTAMP_EN;
	/* note: for non-RDI path, this should be format->decode_format */
	val |= DECODE_FORMAT_PAYLOAD_ONLY << RDI_CFG0_DECODE_FORMAT;
	val |= format->data_type << RDI_CFG0_DATA_TYPE;
	val |= vc << RDI_CFG0_VIRTUAL_CHANNEL;
	val |= dt_id << RDI_CFG0_DT_ID;
	writel_relaxed(val, csid->base + CSID_RDI_CFG0(port));

	/* CSID_TIMESTAMP_STB_POST_IRQ */
	val = 2 << RDI_CFG1_TIMESTAMP_STB_SEL;
	writel_relaxed(val, csid->base + CSID_RDI_CFG1(port));

	val = 1;
	writel_relaxed(val, csid->base + CSID_RDI_FRM_DROP_PERIOD(port));

	val = 0;
	writel_relaxed(val, csid->base + CSID_RDI_FRM_DROP_PATTERN(port));

	val = 1;
	writel_relaxed(val, csid->base + CSID_RDI_IRQ_SUBSAMPLE_PERIOD(port));

	val = 0;
	writel_relaxed(val, csid->base + CSID_RDI_IRQ_SUBSAMPLE_PATTERN(port));

	val = 1;
	writel_relaxed(val, csid->base + CSID_RDI_RPP_PIX_DROP_PERIOD(port));

	val = 0;
	writel_relaxed(val, csid->base + CSID_RDI_RPP_PIX_DROP_PATTERN(port));

	val = 1;
	writel_relaxed(val, csid->base + CSID_RDI_RPP_LINE_DROP_PERIOD(port));

	val = 0;
	writel_relaxed(val, csid->base + CSID_RDI_RPP_LINE_DROP_PATTERN(port));

	val = 0;
	writel_relaxed(val, csid->base + CSID_RDI_CTRL(port));

	val = readl_relaxed(csid->base + CSID_RDI_CFG0(port));
	val |=  enable << RDI_CFG0_ENABLE;
	writel_relaxed(val, csid->base + CSID_RDI_CFG0(port));
}

static void csid_configure_stream(struct csid_device *csid, u8 enable)
{
	struct csid_testgen_config *tg = &csid->testgen;
	u8 i;

	/* Loop through all enabled ports and configure a stream for each */
	for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS; i++)
		if (csid->phy.en_vc & BIT(i)) {
			if (tg->enabled)
				__csid_configure_testgen(csid, enable, i, 0);

			__csid_configure_rdi_stream(csid, enable, i, 0);
			__csid_configure_rx(csid, &csid->phy, 0);
			__csid_ctrl_rdi(csid, enable, i);
		}
}

static int csid_configure_testgen_pattern(struct csid_device *csid, s32 val)
{
	if (val > 0 && val <= csid->testgen.nmodes)
		csid->testgen.mode = val;

	return 0;
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
	u32 val;
	u8 reset_done;
	int i;

	val = readl_relaxed(csid->base + CSID_TOP_IRQ_STATUS);
	writel_relaxed(val, csid->base + CSID_TOP_IRQ_CLEAR);
	reset_done = val & BIT(TOP_IRQ_STATUS_RESET_DONE);

	val = readl_relaxed(csid->base + CSID_CSI2_RX_IRQ_STATUS);
	writel_relaxed(val, csid->base + CSID_CSI2_RX_IRQ_CLEAR);

	/* Read and clear IRQ status for each enabled RDI channel */
	for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS; i++)
		if (csid->phy.en_vc & BIT(i)) {
			val = readl_relaxed(csid->base + CSID_CSI2_RDIN_IRQ_STATUS(i));
			writel_relaxed(val, csid->base + CSID_CSI2_RDIN_IRQ_CLEAR(i));
		}

	val = 1 << IRQ_CMD_CLEAR;
	writel_relaxed(val, csid->base + CSID_IRQ_CMD);

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

	reinit_completion(&csid->reset_complete);

	writel_relaxed(1, csid->base + CSID_TOP_IRQ_CLEAR);
	writel_relaxed(1, csid->base + CSID_IRQ_CMD);
	writel_relaxed(1, csid->base + CSID_TOP_IRQ_MASK);
	writel_relaxed(1, csid->base + CSID_IRQ_CMD);

	/* preserve registers */
	val = 0x1e << RST_STROBES;
	writel_relaxed(val, csid->base + CSID_RST_STROBES);

	time = wait_for_completion_timeout(&csid->reset_complete,
					   msecs_to_jiffies(CSID_RESET_TIMEOUT_MS));
	if (!time) {
		dev_err(csid->camss->dev, "CSID reset timeout\n");
		return -EIO;
	}

	return 0;
}

static void csid_subdev_init(struct csid_device *csid)
{
	csid->testgen.modes = csid_testgen_modes;
	csid->testgen.nmodes = CSID_PAYLOAD_MODE_NUM_SUPPORTED_GEN2;
}

const struct csid_hw_ops csid_ops_gen2 = {
	.configure_stream = csid_configure_stream,
	.configure_testgen_pattern = csid_configure_testgen_pattern,
	.hw_version = csid_hw_version,
	.isr = csid_isr,
	.reset = csid_reset,
	.src_pad_code = csid_src_pad_code,
	.subdev_init = csid_subdev_init,
};
