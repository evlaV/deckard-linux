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
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>

#include "camss.h"
#include "camss-csid.h"
#include "camss-csid-gen3.h"

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

#define CSID_CSI2_RDIN_IRQ_STATUS(rdi)	(0xEC + 0x10 * (rdi))

#define CSID_CSI2_RDIN_IRQ_MASK(rdi)	(0xF0 + 0x10 * (rdi))
#define CSID_CSI2_RDIN_IRQ_CLEAR(rdi)	(0xF4 + 0x10 * (rdi))
#define CSID_CSI2_RDIN_IRQ_SET(rdi)	(0xF8 + 0x10 * (rdi))

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

#define CSID_RDI_CFG0(rdi)		(0x500 + 0x100 * (rdi))
#define		RDI_CFG0_TIMESTAMP_EN		6
#define		RDI_CFG0_TIMESTAMP_STB_SEL	8
#define		RDI_CFG0_DECODE_FORMAT		12
#define		RDI_CFG0_DT			16
#define		RDI_CFG0_VC			22
#define		RDI_CFG0_DT_ID			27
#define		RDI_CFG0_EN			31

#define CSID_RDI_CFG1(rdi)		(0x510 + 0x100 * (rdi))
#define		RDI_CFG1_DROP_H_EN		5
#define		RDI_CFG1_DROP_V_EN		6
#define		RDI_CFG1_CROP_H_EN		7
#define		RDI_CFG1_CROP_V_EN		8
#define		RDI_CFG1_PIX_STORE		10
#define		RDI_CFG1_PACKING_FORMAT		15

#define CSID_RDI_CTRL(rdi)		(0x504 + 0x100 * (rdi))
#define		RDI_CTRL_START_CMD		0

#define CSID_RDI_IRQ_SUBSAMPLE_PATTERN(rdi)	(0x548 + 0x100 * (rdi))
#define CSID_RDI_IRQ_SUBSAMPLE_PERIOD(rdi)	(0x54C + 0x100 * (rdi))

static int csid_reset(struct csid_device *csid);

static inline int reg_update_rdi(struct csid_device *csid, int n)
{
	/*  RUP_RDI0 | AUP_RDI0 | MUP */
	return BIT(n + 4) + BIT(20 + n) + BIT(28 + n);
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

	val = 1 << CSI2_RX_CFG1_ECC_CORRECTION_EN;
	val |= 1 << CSI2_RX_CFG1_MISR_EN;
	if (vc > 3)
		val |= 1 << CSI2_RX_CFG1_VC_MODE;
	writel(val, csid->base + CSID_CSI2_RX_CFG1);
}

static void __csid_ctrl_rdi(struct csid_device *csid, int enable, u8 rdi)
{
	int val = 0;

	if (enable)
		val = 1 << RDI_CTRL_START_CMD;

	writel(val, csid->base + CSID_RDI_CTRL(rdi));

	/* HACK */
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

	val = 1 << RDI_CFG0_TIMESTAMP_EN;
	/*
	val |= 1 << RDI_CFG0_TIMESTAMP_STB_SEL;
	*/
	/* RDI_CFG0_SOF_RETIME_DIS */
	val |= 1 << 5;
	/* note: for non-RDI path, this should be format->decode_format */
	val |= DECODE_FORMAT_PAYLOAD_ONLY << RDI_CFG0_DECODE_FORMAT;
	val |= vc << RDI_CFG0_VC;
	val |= format->data_type << RDI_CFG0_DT;
	val |= dt_id << RDI_CFG0_DT_ID;

	writel(val, csid->base + CSID_RDI_CFG0(vc));

	val = 1 << RDI_CFG1_PACKING_FORMAT;
	val |= 1 << RDI_CFG1_PIX_STORE;
	val |= 1 << RDI_CFG1_DROP_H_EN;
	val |= 1 << RDI_CFG1_DROP_V_EN;
	val |= 1 << RDI_CFG1_CROP_H_EN;
	val |= 1 << RDI_CFG1_CROP_V_EN;

	val = 0x00040000;
	writel(val, csid->base + CSID_RDI_CFG1(vc));

	val = 0;
	writel(val, csid->base + CSID_RDI_IRQ_SUBSAMPLE_PERIOD(vc));

	val = 1;
	writel(val, csid->base + CSID_RDI_IRQ_SUBSAMPLE_PATTERN(vc));

	val = 0;
	writel(val, csid->base + CSID_RDI_CTRL(vc));

	val = readl(csid->base + CSID_RDI_CFG0(vc));

	if (enable)
		val |= 1 << RDI_CFG0_EN;
	writel(val, csid->base + CSID_RDI_CFG0(vc));
}

static void csid_configure_stream(struct csid_device *csid, u8 enable)
{
	u8 i;

	if (!enable) {
		csid_reset(csid);
		camss_buf_done(csid->camss, csid->id, 0);
		usleep_range(10000, 20000);
	}

	if (enable)
		__csid_configure_top(csid);

	/* Loop through all enabled VCs and configure stream for each */
	for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS; i++)
		//csid->phy.en_vc = 1;
		if (csid->phy.en_vc & BIT(i)) {
			__csid_configure_rdi_stream(csid, enable, i);
			__csid_configure_rx(csid, &csid->phy, i);
			__csid_ctrl_rdi(csid, enable, i);
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
	int i = 0;

	top_val = readl(csid->base + CSID_TOP_IRQ_STATUS);
	writel(top_val, csid->base + CSID_TOP_IRQ_CLEAR);
	reset_done = top_val & BIT(TOP_IRQ_STATUS_RESET_DONE);

	val = readl(csid->base + CSID_CSI2_RX_IRQ_STATUS);
	writel(val, csid->base + CSID_CSI2_RX_IRQ_CLEAR);

	val = readl(csid->base + CSID_CSI2_RDIN_IRQ_STATUS(i));
	writel(val, csid->base + CSID_CSI2_RDIN_IRQ_CLEAR(i));

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

		camss_buf_done(csid->camss, csid->id, 0);
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
	/* PATH_INFO_SOF | CAMIF_SOF */

	val = 0x1010;
	//val = 0x10881004;
	writel_relaxed(val, csid->base + CSID_CSI2_RDIN_IRQ_MASK(csid->id));

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

static void csid_subdev_init(struct csid_device *csid)
{
	/* nop */
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
};
