// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017-18 Linaro Limited

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/reboot-mode.h>
#include <linux/regmap.h>

#define PON_SOFT_RB_SPARE		0x8f

#define GEN1_REASON_SHIFT		2
#define GEN2_REASON_SHIFT		1

#define NO_REASON_SHIFT			0

#define PON_PBS_PS_HOLD_SW_CTL		0x52
#define  PON_PS_HOLD_ENABLE		BIT(7)
#define PON_PBS_PS_HOLD_RESET_CTL2	0x53
#define  PON_PS_HOLD_TYPE_MASK		0x0f
#define  PON_PS_HOLD_TYPE_WARM_RESET	1
#define  PON_PS_HOLD_TYPE_SHUTDOWN	4
#define  PON_PS_HOLD_TYPE_HARD_RESET	7

struct pm8916_pon_data {
	long reason_shift;
	bool needs_pbs_ps_hold_sw_ctl;
};

struct pm8916_pon {
	struct device *dev;
	struct regmap *regmap;
	u32 baseaddr;
	u32 pbs_baseaddr;
	struct reboot_mode_driver reboot_mode;
	long reason_shift;
	bool needs_pbs_ps_hold_sw_ctl;
};

static void pm8916_reboot_ps_hold_type_setup(struct pm8916_pon *pon, unsigned int type)
{
	regmap_write(pon->regmap, pon->pbs_baseaddr + PON_PBS_PS_HOLD_RESET_CTL2, 0);
	usleep_range(100, 1000);
	regmap_write(pon->regmap, pon->pbs_baseaddr + PON_PBS_PS_HOLD_SW_CTL,
		     type);
	regmap_write(pon->regmap, pon->pbs_baseaddr + PON_PBS_PS_HOLD_RESET_CTL2,
		     PON_PS_HOLD_ENABLE);
}

static int pm8916_pon_restart_prepare(struct sys_off_data *data)
{
	struct pm8916_pon *pon = data->cb_data;

	dev_info(pon->dev, "Setting PON-PBS for Reset\n");

	pm8916_reboot_ps_hold_type_setup(pon, PON_PS_HOLD_TYPE_HARD_RESET);

	return NOTIFY_OK;
}

static int pm8916_pon_poweroff_prepare(struct sys_off_data *data)
{
	struct pm8916_pon *pon = data->cb_data;

	dev_info(pon->dev, "Setting PON-PBS for Shutdown\n");

	pm8916_reboot_ps_hold_type_setup(pon, PON_PS_HOLD_TYPE_SHUTDOWN);

	return NOTIFY_OK;
}

static int pm8916_reboot_mode_write(struct reboot_mode_driver *reboot,
				    unsigned int magic)
{
	struct pm8916_pon *pon = container_of
			(reboot, struct pm8916_pon, reboot_mode);
	int ret;

	ret = regmap_update_bits(pon->regmap,
				 pon->baseaddr + PON_SOFT_RB_SPARE,
				 GENMASK(7, pon->reason_shift),
				 magic << pon->reason_shift);
	if (ret < 0)
		dev_err(pon->dev, "update reboot mode bits failed\n");

	return ret;
}

static int pm8916_pon_probe(struct platform_device *pdev)
{
	const struct pm8916_pon_data *data;
	struct pm8916_pon *pon;
	int error;

	data = device_get_match_data(&pdev->dev);
	if (!data)
		return -EINVAL;

	pon = devm_kzalloc(&pdev->dev, sizeof(*pon), GFP_KERNEL);
	if (!pon)
		return -ENOMEM;

	pon->dev = &pdev->dev;

	pon->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pon->regmap) {
		dev_err(&pdev->dev, "failed to locate regmap\n");
		return -ENODEV;
	}

	error = of_property_read_u32(pdev->dev.of_node, "reg", &pon->baseaddr);
	if (error)
		return error;

	if (data->needs_pbs_ps_hold_sw_ctl) {
		error = of_property_read_u32_index(pdev->dev.of_node, "reg", 1,
						   &pon->pbs_baseaddr);
		if (error)
			return error;

		devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_RESTART_PREPARE,
				      SYS_OFF_PRIO_FIRMWARE, pm8916_pon_restart_prepare,
				      pon);

		devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_POWER_OFF_PREPARE,
				      SYS_OFF_PRIO_FIRMWARE, pm8916_pon_poweroff_prepare,
				      pon);
	}

	if (data->reason_shift != NO_REASON_SHIFT) {
		pon->reboot_mode.dev = &pdev->dev;
		pon->reason_shift = data->reason_shift;
		pon->reboot_mode.write = pm8916_reboot_mode_write;
		error = devm_reboot_mode_register(&pdev->dev, &pon->reboot_mode);
		if (error) {
			dev_err(&pdev->dev, "can't register reboot mode\n");
			return error;
		}
	}

	platform_set_drvdata(pdev, pon);

	return devm_of_platform_populate(&pdev->dev);
}

static const struct pm8916_pon_data pm8916_pon_data = {
	.reason_shift = GEN1_REASON_SHIFT,
};

static const struct pm8916_pon_data pm8941_pon_data = { };

static const struct pm8916_pon_data pm8998_pon_data = {
	.reason_shift = GEN2_REASON_SHIFT,
};

static const struct pm8916_pon_data pmk8350_el2_pon_data = {
	.reason_shift = GEN2_REASON_SHIFT,
	.needs_pbs_ps_hold_sw_ctl = true,
};

static const struct of_device_id pm8916_pon_id_table[] = {
	{ .compatible = "qcom,pm8916-pon", .data = &pm8916_pon_data },
	{ .compatible = "qcom,pm8941-pon", .data = &pm8941_pon_data },
	{ .compatible = "qcom,pms405-pon", .data = &pm8916_pon_data },
	{ .compatible = "qcom,pm8998-pon", .data = &pm8998_pon_data },
	{ .compatible = "qcom,pmk8350-pon", .data = &pm8998_pon_data },
	{ .compatible = "qcom,pmk8350-el2-pon", .data = &pmk8350_el2_pon_data },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8916_pon_id_table);

static struct platform_driver pm8916_pon_driver = {
	.probe = pm8916_pon_probe,
	.driver = {
		.name = "pm8916-pon",
		.of_match_table = pm8916_pon_id_table,
	},
};
module_platform_driver(pm8916_pon_driver);

MODULE_DESCRIPTION("pm8916 Power On driver");
MODULE_LICENSE("GPL v2");
