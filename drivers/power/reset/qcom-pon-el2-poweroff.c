// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 Linaro Limited

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

/*
 * When running in EL2 without the hypervisor, the PSCI handler
 * doesn't properly implement power-off, so handle it here.
 *
 * Reboot reason is handled directly by writing into an sdam nvmem
 * cell.
 *
 * PON reason can be read from an sdam nvmem cell aswell.
 */

#define PON_PBS_PS_HOLD_SW_CTL		0x52
#define  PON_PS_HOLD_ENABLE		BIT(7)
#define PON_PBS_PS_HOLD_RESET_CTL2	0x53
#define  PON_PS_HOLD_TYPE_MASK		0x0f
#define  PON_PS_HOLD_TYPE_WARM_RESET	1
#define  PON_PS_HOLD_TYPE_SHUTDOWN	4
#define  PON_PS_HOLD_TYPE_HARD_RESET	7

struct qcom_pon_el2_poweroff {
	struct device *dev;
	struct regmap *regmap;
	u32 pbs_baseaddr;
};

static int qcom_pon_el2_poweroff_poweroff_prepare(struct sys_off_data *data)
{
	struct qcom_pon_el2_poweroff *pon = data->cb_data;

	dev_info(pon->dev, "Setting PON-PBS for Shutdown\n");

	// PSCI will write the PON_PBS_PS_HOLD_RESET_CTL2 so we only need to update PON_PBS_PS_HOLD_SW_CTL
	regmap_write(pon->regmap, pon->pbs_baseaddr + PON_PBS_PS_HOLD_SW_CTL,
		     PON_PS_HOLD_TYPE_SHUTDOWN);

	return NOTIFY_OK;
}

static int qcom_pon_el2_poweroff_probe(struct platform_device *pdev)
{
	const struct qcom_pon_el2_poweroff_data *data;
	struct qcom_pon_el2_poweroff *pon;
	int err;

	pon = devm_kzalloc(&pdev->dev, sizeof(*pon), GFP_KERNEL);
	if (!pon)
		return -ENOMEM;

	pon->dev = &pdev->dev;

	pon->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pon->regmap) {
		dev_err(&pdev->dev, "failed to locate regmap\n");
		return -ENODEV;
	}

	err = of_property_read_u32_index(pdev->dev.of_node, "reg", 1,
					 &pon->pbs_baseaddr);
	if (err)
		return err;

	devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_POWER_OFF_PREPARE,
				      SYS_OFF_PRIO_FIRMWARE,
				      qcom_pon_el2_poweroff_poweroff_prepare,
				      pon);

	platform_set_drvdata(pdev, pon);

	return devm_of_platform_populate(&pdev->dev);
}

static const struct of_device_id qcom_pon_el2_poweroff_id_table[] = {
	{ .compatible = "qcom,pmk8350-el2-pon" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_pon_el2_poweroff_id_table);

static struct platform_driver qcom_pon_el2_poweroff_driver = {
	.probe = qcom_pon_el2_poweroff_probe,
	.driver = {
		.name = "qcom-pon-el2-poweroff",
		.of_match_table = qcom_pon_el2_poweroff_id_table,
	},
};
module_platform_driver(qcom_pon_el2_poweroff_driver);

MODULE_DESCRIPTION("Qualcomm Power On EL2 Power Off driver");
MODULE_LICENSE("GPL v2");
