// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_clock.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/qcom,lpasscorecc-sm8650.h>

#include "clk-alpha-pll.h"
#include "clk-branch.h"
#include "clk-rcg.h"
#include "clk-regmap.h"
#include "common.h"
#include "gdsc.h"

enum {
	P_BI_TCXO,
	P_SLEEP_CLK,
};

static const struct parent_map lpass_core_cc_parent_map_0[] = {
	{ P_BI_TCXO, 0 },
};

static const struct clk_parent_data lpass_core_cc_parent_data_0[] = {
	{ .fw_name = "bi_tcxo" },
};

static struct clk_rcg2 lpass_qdsp6ss_core_cc_core_clk_src = {
	.cmd_rcgr = 0x0,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = lpass_core_cc_parent_map_0,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "lpass_qdsp6ss_core_cc_core_clk_src",
		.parent_data = &(const struct clk_parent_data){
			.fw_name = "bi_tcxo",
		},
		.num_parents = 1,
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_branch lpass_qdsp6ss_core_cc_core_clk = {
	.halt_reg = 0x20,
	.halt_check = BRANCH_HALT,
	.hwcg_reg = 0x20,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x0,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "lpass_qdsp6ss_core_cc_core_clk",
			.parent_data = &(const struct clk_parent_data){
				.hw = &lpass_qdsp6ss_core_cc_core_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_regmap *lpass_qdsp6ss_core_cc_sm8650_clocks[] = {
	[LPASS_QDSP6SS_CORE_CC_CORE_CLK_SRC] = &lpass_qdsp6ss_core_cc_core_clk_src.clkr,
	[LPASS_QDSP6SS_CORE_CC_CORE_CLK] = &lpass_qdsp6ss_core_cc_core_clk.clkr,
};

static struct clk_rcg2 lpass_core_cc_core_clk_src = {
	.cmd_rcgr = 0x1d000,
	.mnd_width = 8,
	.hid_width = 5,
	.hw_clk_ctrl = true,
	.parent_map = lpass_core_cc_parent_map_0,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "lpass_core_cc_core_clk_src",
		.parent_data = &(const struct clk_parent_data){
			.fw_name = "bi_tcxo",
		},
		.num_parents = 1,
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_regmap *lpass_core_cc_sm8650_clocks[] = {
	[LPASS_CORE_CC_CORE_CLK_SRC] = &lpass_core_cc_core_clk_src.clkr,
};

static struct clk_branch lpass_aon_cc_at_clk = {
	.halt_reg = 0xa020,
	.halt_check = BRANCH_HALT,
	.hwcg_reg = 0xa020,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0xa020,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "lpass_aon_cc_at_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch lpass_aon_cc_pclkdbg_clk = {
	.halt_reg = 0xa014,
	.halt_check = BRANCH_HALT,
	.hwcg_reg = 0xa014,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0xa014,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "lpass_aon_cc_pclkdbg_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch lpass_aon_cc_lpi_noc_hs_clk = {
	.halt_reg = 0x19008,
	.halt_check = BRANCH_HALT,
	.hwcg_reg = 0x19008,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x19008,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "lpass_aon_cc_lpi_noc_hs_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch lpass_aon_cc_q6_axim_clk = {
	.halt_reg = 0x19034,
	.halt_check = BRANCH_HALT,
	.hwcg_reg = 0x19034,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x19034,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "lpass_aon_cc_q6_axim_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_regmap *lpass_aon_cc_sm8650_clocks[] = {
	[LPASS_AON_CC_AT_CLK] = &lpass_aon_cc_at_clk.clkr,
	[LPASS_AON_CC_PCLKDBG_CLK] = &lpass_aon_cc_pclkdbg_clk.clkr,
	[LPASS_AON_CC_LPI_NOC_HS_CLK] = &lpass_aon_cc_lpi_noc_hs_clk.clkr,
	[LPASS_AON_CC_Q6_AXIM_CLK] = &lpass_aon_cc_q6_axim_clk.clkr,
};

static struct gdsc lpass_core_hm_gdsc = {
	.gdscr = 0x0,
	.pd = {
		.name = "lpass_core_hm_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
	.flags = RETAIN_FF_ENABLE | ALWAYS_ON,
};

static struct gdsc *lpass_core_hm_sm8650_gdscs[] = {
	[LPASS_CORE_HM_GDSC] = &lpass_core_hm_gdsc,
};

static struct regmap_config lpass_core_cc_sm8650_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.fast_io = true,
};

static const struct qcom_cc_desc lpass_qdsp6ss_core_cc_sm8650_desc = {
	.config = &lpass_core_cc_sm8650_regmap_config,
	.clks = lpass_qdsp6ss_core_cc_sm8650_clocks,
	.num_clks = ARRAY_SIZE(lpass_qdsp6ss_core_cc_sm8650_clocks),
};

static const struct qcom_cc_desc lpass_lpi_tcm_sm8650_desc = {
	.config = &lpass_core_cc_sm8650_regmap_config,
};

static const struct qcom_cc_desc lpass_core_cc_sm8650_desc = {
	.config = &lpass_core_cc_sm8650_regmap_config,
	.clks = lpass_core_cc_sm8650_clocks,
	.num_clks = ARRAY_SIZE(lpass_core_cc_sm8650_clocks),
};

static const struct qcom_cc_desc lpass_core_hm_sm8650_desc = {
	.config = &lpass_core_cc_sm8650_regmap_config,
	.gdscs = lpass_core_hm_sm8650_gdscs,
	.num_gdscs = ARRAY_SIZE(lpass_core_hm_sm8650_gdscs),
};

static const struct qcom_cc_desc lpass_aon_cc_sm8650_desc = {
	.config = &lpass_core_cc_sm8650_regmap_config,
	.clks = lpass_aon_cc_sm8650_clocks,
	.num_clks = ARRAY_SIZE(lpass_aon_cc_sm8650_clocks),
};

//LPASS+0x00848000
static int lpass_qdsp6ss_core_cc_sm8650_probe(struct platform_device *pdev)
{
	struct regmap *regmap;

	lpass_core_cc_sm8650_regmap_config.name = "qdsp6ss_core_cc";
	regmap = qcom_cc_map(pdev, &lpass_qdsp6ss_core_cc_sm8650_desc);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return qcom_cc_really_probe(&pdev->dev, &lpass_qdsp6ss_core_cc_sm8650_desc, regmap);
};

//LPASS+0x0126A000
static int lpass_lpi_tcm_sm8650_probe(struct platform_device *pdev)
{
	struct regmap *regmap;

	lpass_core_cc_sm8650_regmap_config.name = "lpass_lpi_tcm";
	regmap = qcom_cc_map(pdev, &lpass_lpi_tcm_sm8650_desc);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	// LPI_TCM_AXIS_HS
	regmap_update_bits(regmap, 0x0300, BIT(0), BIT(0));
	// LPASS_LPI_TCM_256KB_BLOCKn_CBCR(0..7)
	regmap_update_bits(regmap, 0x0000, BIT(14) | BIT(0), BIT(14) | BIT(0));
	regmap_update_bits(regmap, 0x0004, BIT(14) | BIT(0), BIT(14) | BIT(0));
	regmap_update_bits(regmap, 0x0008, BIT(14) | BIT(0), BIT(14) | BIT(0));
	regmap_update_bits(regmap, 0x000c, BIT(14) | BIT(0), BIT(14) | BIT(0));
	regmap_update_bits(regmap, 0x0010, BIT(14) | BIT(0), BIT(14) | BIT(0));
	regmap_update_bits(regmap, 0x0014, BIT(14) | BIT(0), BIT(14) | BIT(0));
	regmap_update_bits(regmap, 0x0018, BIT(14) | BIT(0), BIT(14) | BIT(0));
	regmap_update_bits(regmap, 0x001c, BIT(14) | BIT(0), BIT(14) | BIT(0));

	return qcom_cc_really_probe(&pdev->dev, &lpass_lpi_tcm_sm8650_desc, regmap);
};

static int lpass_core_cc_sm8650_probe(struct platform_device *pdev)
{
	struct regmap *regmap;

	lpass_core_cc_sm8650_regmap_config.name = "lpass_core_cc";
	regmap = qcom_cc_map(pdev, &lpass_core_cc_sm8650_desc);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return qcom_cc_really_probe(&pdev->dev, &lpass_core_cc_sm8650_desc, regmap);
}

static int lpass_core_hm_sm8650_probe(struct platform_device *pdev)
{
	struct regmap *regmap;

	lpass_core_cc_sm8650_regmap_config.name = "lpass_core_hm";
	regmap = qcom_cc_map(pdev, &lpass_core_hm_sm8650_desc);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	// lpass_top_cc_lpass_core_sway_ahb_ls
	regmap_update_bits(regmap, 0x9000, BIT(0), BIT(0));

	return qcom_cc_really_probe(&pdev->dev, &lpass_core_hm_sm8650_desc, regmap);
}

static int lpass_aon_cc_sm8650_probe(struct platform_device *pdev)
{
	struct regmap *regmap;

	lpass_core_cc_sm8650_regmap_config.name = "lpass_aon_cc";
	regmap = qcom_cc_map(pdev, &lpass_aon_cc_sm8650_desc);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return qcom_cc_really_probe(&pdev->dev, &lpass_aon_cc_sm8650_desc, regmap);
}


static const struct of_device_id lpass_core_cc_sm8650_match_table[] = {
	{
		.compatible = "qcom,sm8650-qdsp6ss-core-cc",
		.data = lpass_qdsp6ss_core_cc_sm8650_probe,
	},
	{
		.compatible = "qcom,sm8650-lpass-lpi-tcm",
		.data = lpass_lpi_tcm_sm8650_probe,
	},
	{
		.compatible = "qcom,sm8650-lpass-core-cc",
		.data = lpass_core_cc_sm8650_probe,
	},
	{
		.compatible = "qcom,sm8650-lpass-core-hm",
		.data = lpass_core_hm_sm8650_probe,
	},
	{
		.compatible = "qcom,sm8650-lpass-aon-cc",
		.data = lpass_aon_cc_sm8650_probe,
	},

	{ }
};
MODULE_DEVICE_TABLE(of, lpass_core_cc_sm8650_match_table);

static int lpass_core_sm8650_probe(struct platform_device *pdev)
{
	int (*clk_probe)(struct platform_device *p);
	int ret;

	pm_runtime_enable(&pdev->dev);
	ret = pm_clk_create(&pdev->dev);
	if (ret)
		goto disable_pm_runtime;

	ret = -EINVAL;
	clk_probe = of_device_get_match_data(&pdev->dev);
	if (!clk_probe)
		goto destroy_pm_clk;

	ret = clk_probe(pdev);
	if (ret)
		goto destroy_pm_clk;

	return 0;

destroy_pm_clk:
	pm_clk_destroy(&pdev->dev);

disable_pm_runtime:
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static const struct dev_pm_ops lpass_core_cc_pm_ops = {
	SET_RUNTIME_PM_OPS(pm_clk_suspend, pm_clk_resume, NULL)
};

static struct platform_driver lpass_core_cc_sm8650_driver = {
	.probe = lpass_core_sm8650_probe,
	.driver = {
		.name = "lpass_core_cc-sm8650",
		.of_match_table = lpass_core_cc_sm8650_match_table,
		.pm = &lpass_core_cc_pm_ops,
	},
};

static int __init lpass_core_cc_sm8650_init(void)
{
	return platform_driver_register(&lpass_core_cc_sm8650_driver);
}
subsys_initcall(lpass_core_cc_sm8650_init);

static void __exit lpass_core_cc_sm8650_exit(void)
{
	platform_driver_unregister(&lpass_core_cc_sm8650_driver);
}
module_exit(lpass_core_cc_sm8650_exit);

MODULE_DESCRIPTION("QTI LPASS_CORE_CC SM8650 Driver");
MODULE_LICENSE("GPL v2");
