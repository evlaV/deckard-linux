// SPDX-License-Identifier: GPL-2.0
/*
 * camss-csiphy-3ph-1-0.c
 *
 * Qualcomm MSM Camera Subsystem - CSIPHY Module 3phase v1.0
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2016-2018 Linaro Ltd.
 */

#include "camss.h"
#include "camss-csiphy.h"

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#define CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(n)	(0x1000 + 0x4 * (n))
#define CSIPHY_3PH_CMN_CSI_COMMON_CTRL5_CLK_ENABLE	BIT(7)
#define CSIPHY_3PH_CMN_CSI_COMMON_CTRL6_COMMON_PWRDN_B	BIT(0)
#define CSIPHY_3PH_CMN_CSI_COMMON_CTRL6_SHOW_REV_ID	BIT(1)
#define CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(n)	(0x10b0 + 0x4 * (n))

#define CSIPHY_DEFAULT_PARAMS            0
#define CSIPHY_LANE_ENABLE               1
#define CSIPHY_SETTLE_CNT_LOWER_BYTE     2
#define CSIPHY_SETTLE_CNT_HIGHER_BYTE    3
#define CSIPHY_2PH_REGS                  4
#define CSIPHY_3PH_REGS                  5
#define CSIPHY_SKEW_CAL                  6

struct csiphy_reg_t {
	s32 reg_addr;
	s32 reg_data;
	s32 delay;
	u32 csiphy_param_type;
};

static const struct csiphy_reg_t csiphy_2ph_v2_2_0_reg[] = {
	{0x0E94, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0EA0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E90, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E98, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E94, 0x07, 0xD1, CSIPHY_DEFAULT_PARAMS},
	{0x0094, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x00A0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0090, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0098, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0094, 0x07, 0xD1, CSIPHY_DEFAULT_PARAMS},
	{0x0494, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x04A0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0490, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0498, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0494, 0x07, 0xD1, CSIPHY_DEFAULT_PARAMS},
	{0x0894, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x08A0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0890, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0898, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0894, 0x07, 0xD1, CSIPHY_DEFAULT_PARAMS},
	{0x0C94, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0CA0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C90, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C98, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C94, 0x07, 0xD1, CSIPHY_DEFAULT_PARAMS},
	{0x0E30, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E28, 0x04, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E00, 0x80, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E0C, 0xFF, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E38, 0x1F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E2C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E34, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E1C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E14, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E3C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E04, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E20, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E08, 0x19, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0E10, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0030, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0000, 0x8E, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0038, 0xFE, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x002C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0034, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x001C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0014, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x003C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0004, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0020, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0008, 0x19, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0010, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0430, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0400, 0x8E, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0438, 0xFE, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x042C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0434, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x041C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0414, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x043C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0404, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0420, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0408, 0x19, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0410, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0830, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0800, 0x8E, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0838, 0xFE, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x082C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0834, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x081C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0814, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x083C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0804, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0820, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0808, 0x19, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0810, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C30, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C00, 0x8E, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C38, 0xFE, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C2C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C34, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C1C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C14, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C3C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C04, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C20, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C08, 0x19, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0C10, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0094, 0xD7, 0x00, CSIPHY_SKEW_CAL},
	{0x005C, 0x04, 0x00, CSIPHY_SKEW_CAL},
	{0x0060, 0xBD, 0x00, CSIPHY_SKEW_CAL},
	{0x0064, 0x7F, 0x00, CSIPHY_SKEW_CAL},
	{0x0494, 0xD7, 0x00, CSIPHY_SKEW_CAL},
	{0x045C, 0x04, 0x00, CSIPHY_SKEW_CAL},
	{0x0460, 0xBD, 0x00, CSIPHY_SKEW_CAL},
	{0x0464, 0x7F, 0x00, CSIPHY_SKEW_CAL},
	{0x0894, 0xD7, 0x00, CSIPHY_SKEW_CAL},
	{0x085C, 0x04, 0x00, CSIPHY_SKEW_CAL},
	{0x0860, 0xBD, 0x00, CSIPHY_SKEW_CAL},
	{0x0864, 0x7F, 0x00, CSIPHY_SKEW_CAL},
	{0x0C94, 0xD7, 0x00, CSIPHY_SKEW_CAL},
	{0x0C5C, 0x04, 0x00, CSIPHY_SKEW_CAL},
	{0x0C60, 0xBD, 0x00, CSIPHY_SKEW_CAL},
	{0x0C64, 0x7F, 0x00, CSIPHY_SKEW_CAL},
};

static const struct csiphy_reg_t csiphy_2ph_v2_2_0_combo_mode_reg[] = {
	{0x0E94, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0EA0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E90, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E98, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E94, 0x07, 0xD1, CSIPHY_DEFAULT_PARAMS},
	{0x0094, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x00A0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0090, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0098, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0094, 0x07, 0xD1, CSIPHY_DEFAULT_PARAMS},
	{0x0494, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x04A0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0490, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0498, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0494, 0x07, 0xD1, CSIPHY_DEFAULT_PARAMS},
	{0x0894, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x08A0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0890, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0898, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0894, 0x07, 0xD1, CSIPHY_DEFAULT_PARAMS},
	{0x0C94, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0CA0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C90, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C98, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C94, 0x07, 0xD1, CSIPHY_DEFAULT_PARAMS},
	{0x0E30, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E28, 0x04, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E00, 0x80, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E0C, 0xFF, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E38, 0x1F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E2C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E34, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E1C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E14, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E3C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E04, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E20, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E08, 0x19, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0E10, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0030, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0000, 0x8E, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0038, 0xFE, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x002C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0034, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x001C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0014, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x003C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0004, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0020, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0008, 0x19, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0010, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0430, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0400, 0x8E, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0438, 0xFE, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x042C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0434, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x041C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0414, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x043C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0404, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0420, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0408, 0x19, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0410, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0830, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0800, 0x8E, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0838, 0xFE, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0828, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x082C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0834, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x081C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0814, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x083C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0804, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0820, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0808, 0x19, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0810, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C30, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C28, 0x04, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C00, 0x80, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C0C, 0xFF, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C38, 0x1F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C28, 0x0E, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C2C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C34, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C1C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C14, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C3C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C04, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C20, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C08, 0x19, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0C10, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0094, 0xD7, 0x00, CSIPHY_SKEW_CAL},
	{0x005C, 0x04, 0x00, CSIPHY_SKEW_CAL},
	{0x0060, 0xBD, 0x00, CSIPHY_SKEW_CAL},
	{0x0064, 0x7F, 0x00, CSIPHY_SKEW_CAL},
	{0x0494, 0xD7, 0x00, CSIPHY_SKEW_CAL},
	{0x045C, 0x04, 0x00, CSIPHY_SKEW_CAL},
	{0x0460, 0xBD, 0x00, CSIPHY_SKEW_CAL},
	{0x0464, 0x7F, 0x00, CSIPHY_SKEW_CAL},
	{0x0894, 0xD7, 0x00, CSIPHY_SKEW_CAL},
	{0x085C, 0x04, 0x00, CSIPHY_SKEW_CAL},
	{0x0860, 0xBD, 0x00, CSIPHY_SKEW_CAL},
	{0x0864, 0x7F, 0x00, CSIPHY_SKEW_CAL},
};

static void csiphy_hw_version_read(struct csiphy_device *csiphy,
				   struct device *dev)
{
	u32 hw_version;

	writel(CSIPHY_3PH_CMN_CSI_COMMON_CTRL6_SHOW_REV_ID,
	       csiphy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(6));

	hw_version = readl_relaxed(csiphy->base +
				   CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(12));
	hw_version |= readl_relaxed(csiphy->base +
				   CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(13)) << 8;
	hw_version |= readl_relaxed(csiphy->base +
				   CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(14)) << 16;
	hw_version |= readl_relaxed(csiphy->base +
				   CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(15)) << 24;

	dev_dbg(dev, "CSIPHY 3PH HW Version = 0x%08x\n", hw_version);
}

/*
 * csiphy_reset - Perform software reset on CSIPHY module
 * @csiphy: CSIPHY device
 */
static void csiphy_reset(struct csiphy_device *csiphy)
{
	writel_relaxed(0x1, csiphy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(0));
	usleep_range(5000, 8000);
	// note: downstream sets 0x02 here for non-combo 2ph
	writel_relaxed(0x0, csiphy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(0));
}

static irqreturn_t csiphy_isr(int irq, void *dev)
{
	struct csiphy_device *csiphy = dev;
	int i;

	for (i = 0; i < 11; i++) {
		int c = i + 22;
		u8 val = readl_relaxed(csiphy->base +
				       CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(i));

		writel_relaxed(val, csiphy->base +
				    CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(c));
	}

	writel_relaxed(0x1, csiphy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(10));
	writel_relaxed(0x0, csiphy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(10));

	for (i = 22; i < 33; i++)
		writel_relaxed(0x0, csiphy->base +
				    CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(i));

	return IRQ_HANDLED;
}

/*
 * csiphy_settle_cnt_calc - Calculate settle count value
 *
 * Helper function to calculate settle count value. This is
 * based on the CSI2 T_hs_settle parameter which in turn
 * is calculated based on the CSI2 transmitter link frequency.
 *
 * Return settle count value or 0 if the CSI2 link frequency
 * is not available
 */
static u8 csiphy_settle_cnt_calc(s64 link_freq, u32 timer_clk_rate)
{
	u32 ui; /* ps */
	u32 timer_period; /* ps */
	u32 t_hs_prepare_max; /* ps */
	u32 t_hs_settle; /* ps */
	u8 settle_cnt;

	if (link_freq <= 0)
		return 0;

	ui = div_u64(1000000000000LL, link_freq);
	ui /= 2;
	t_hs_prepare_max = 85000 + 6 * ui;
	t_hs_settle = t_hs_prepare_max;

	timer_period = div_u64(1000000000000LL, timer_clk_rate);
	settle_cnt = t_hs_settle / timer_period - 6;

	return settle_cnt;
}

static void csiphy_gen2_config_lanes(struct csiphy_device *csiphy,
				     u8 settle_cnt,
				     u8 combo_mode)
{
	const struct csiphy_reg_t *r;
	int i, array_size;
	u32 val;

	r = csiphy_2ph_v2_2_0_reg;
	array_size = ARRAY_SIZE(csiphy_2ph_v2_2_0_reg);

	if (combo_mode) {
		r = csiphy_2ph_v2_2_0_combo_mode_reg;
		array_size = ARRAY_SIZE(csiphy_2ph_v2_2_0_combo_mode_reg);
	}

	for (i = 0; i < array_size; i++, r++) {
		switch (r->csiphy_param_type) {
		case CSIPHY_SETTLE_CNT_LOWER_BYTE:
			val = settle_cnt & 0xff;
			break;
		case CSIPHY_SKEW_CAL:
			continue;
		default:
			val = r->reg_data;
			break;
		}
		writel_relaxed(val, csiphy->base + r->reg_addr);
	}
}

static u8 csiphy_get_lane_mask(struct csiphy_lanes_cfg *lane_cfg)
{
	return 0; // passed to csiphy_lanes_enable which doesn't use it
}

static void csiphy_lanes_enable(struct csiphy_device *csiphy,
				struct csiphy_config *cfg,
				s64 link_freq, u8 lane_mask)
{
	struct csiphy_lanes_cfg *c = &cfg->csi2->lane_cfg;
	u8 settle_cnt;
	u8 val;
	int i;

	writel_relaxed(0, csiphy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(33));
	writel_relaxed(0, csiphy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(34));

	settle_cnt = csiphy_settle_cnt_calc(link_freq, csiphy->timer_clk_rate);
	// check settle_cnt

	val = BIT(7); // DPHY_CLK_LN
	for (i = 0; i < c->num_data; i++)
		val |= BIT(c->data[i].pos * 2); // DPHY_LANE_i

	// note: DPHY lane enable bits are BIT(0/2/4/6/7) for LN0/LN1/LN2/LN3/CLK
	if (cfg->combo_mode)
		val |= BIT(4) | BIT(6); // enable LN2/LN3 (LN3 is second clock lane)

	writel_relaxed(val, csiphy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(5));

	val = CSIPHY_3PH_CMN_CSI_COMMON_CTRL6_COMMON_PWRDN_B;
	writel_relaxed(val, csiphy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(6));

	val = 0x7a;
	writel_relaxed(val, csiphy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(7));

	val = 0x00;
	writel_relaxed(val, csiphy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(0));

	csiphy_gen2_config_lanes(csiphy, settle_cnt, cfg->combo_mode);

	/* IRQ_MASK registers - disable all interrupts */
	for (i = 11; i < 22; i++)
		writel_relaxed(0, csiphy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(i));
}

static void csiphy_lanes_disable(struct csiphy_device *csiphy,
				 struct csiphy_config *cfg)
{
	writel_relaxed(0, csiphy->base +
			  CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(5));

	writel_relaxed(0, csiphy->base +
			  CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(6));
}

const struct csiphy_hw_ops csiphy_ops_3ph_1_0 = {
	.get_lane_mask = csiphy_get_lane_mask,
	.hw_version_read = csiphy_hw_version_read,
	.reset = csiphy_reset,
	.lanes_enable = csiphy_lanes_enable,
	.lanes_disable = csiphy_lanes_disable,
	.isr = csiphy_isr,
};
