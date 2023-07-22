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

#include <asm/system_misc.h>

#define PON_INT_MID_SEL			0x1a

#define PON_PBS_PS_HOLD_SW_CTL		0x852
#define PON_PBS_PS_HOLD_RESET_CTL2	0x853
#define  PON_PS_HOLD_ENABLE		BIT(7)
#define  PON_PS_HOLD_TYPE_MASK		0x0f
#define  PON_PS_HOLD_TYPE_WARM_RESET	1
#define  PON_PS_HOLD_TYPE_SHUTDOWN	4
#define  PON_PS_HOLD_TYPE_HARD_RESET	7

#define PON_SOFT_RB_SPARE		0x8f
#define PON_PON_REASON1		    0xc0


#define GEN1_REASON_SHIFT		2
#define GEN2_REASON_SHIFT		1

#define NO_REASON_SHIFT			0
struct pm8916_pon {
	struct device *dev;
	struct regmap *regmap;
	long reason_shift;

	struct notifier_block reboot_notifier;
	unsigned long code;
	void __iomem *ps_hold;
	void __iomem *tcsr_boot_misc_detect;
	u32 pmic_pon_reason_regval;
};
static struct pm8916_pon *pon_global;

static void pon_set_mode(struct pm8916_pon *pon, unsigned reset_type)
{
	//u32 type = 0, cookie = 0;
	//regmap_read(pon->regmap, PON_PBS_PS_HOLD_SW_CTL, &type);
	//regmap_read(pon->regmap, 0x6109, &cookie);
	//printk("pon_set_mode was %d %d\n", type, cookie);

	//regmap_write(pon->regmap, 0x6109, 0);

	regmap_write(pon->regmap, PON_PBS_PS_HOLD_RESET_CTL2, 0);

	/*
	 * Updates of PON_PS_HOLD_ENABLE requires 3 sleep cycles between
	 * writes.
	 */
	usleep_range(100, 1000);

	regmap_write(pon->regmap, PON_PBS_PS_HOLD_SW_CTL, reset_type);
	regmap_write(pon->regmap, PON_PBS_PS_HOLD_RESET_CTL2, BIT(7));
}

static void do_msm_poweroff(void)
{
	struct pm8916_pon *pon = pon_global;

	printk("do_msm_poweroff: %ld\n", pon->code);

	/* not a reboot: */
	if (pon->code == SYS_HALT || pon->code == SYS_POWER_OFF)
		pon_set_mode(pon, PON_PS_HOLD_TYPE_SHUTDOWN);
	else
		pon_set_mode(pon, PON_PS_HOLD_TYPE_HARD_RESET);

	writel(0, pon->ps_hold);
	mdelay(10000);
}

static int pon_reboot_notify(struct notifier_block *nb, unsigned long code, void *cmd)
{
	struct pm8916_pon *pon = container_of(nb, struct pm8916_pon, reboot_notifier);

	pon->code = code;

	if (!cmd) {
		/* normal reboot - already set to hard reset on probe */
	} else if (!strcmp(cmd, "bootloader")) {
		pon_set_mode(pon, PON_PS_HOLD_TYPE_WARM_RESET);
		writel(1, pon->tcsr_boot_misc_detect); /* USB boot mode DLOAD direct mode / QDL / EDL mode */
	}

	wmb();

	do_msm_poweroff();
	return 0;
}

static ssize_t pon_reason_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pm8916_pon *pon = pon_global;
	return scnprintf(buf, PAGE_SIZE, "0x%02X\n", pon->pmic_pon_reason_regval);
}

static DEVICE_ATTR_RO(pon_reason);

static struct attribute *attrs[] = {
	&dev_attr_pon_reason.attr,
	NULL,
};

static struct attribute_group sysfs_group = {
	.attrs = attrs,
};

static int pm8916_pon_probe(struct platform_device *pdev)
{
	struct pm8916_pon *pon;
	int error;
	//u32 pmic_pon_reason_regval;
	int ret;

	pon = devm_kzalloc(&pdev->dev, sizeof(*pon), GFP_KERNEL);
	if (!pon)
		return -ENOMEM;

	ret = sysfs_create_group(&pdev->dev.kobj, &sysfs_group);
	if (ret)
		return ret;

	pon->dev = &pdev->dev;

	pon->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pon->regmap) {
		dev_err(&pdev->dev, "failed to locate regmap\n");
		return -ENODEV;
	}

	pon->reboot_notifier.notifier_call = pon_reboot_notify,
	error = devm_register_reboot_notifier(&pdev->dev, &pon->reboot_notifier);
	if (error) {
		dev_err(&pdev->dev, "failed to register reboot notifier: %d\n", error);
		return error;
	}

	pon->ps_hold = devm_ioremap(&pdev->dev, 0xc264000, 0x4);
	pon->tcsr_boot_misc_detect = devm_ioremap(&pdev->dev, 0x01fd9000, 0x4);

	platform_set_drvdata(pdev, pon);

	/* set hard reset by default (this is for holding power button for 15s) */
	pon_set_mode(pon, PON_PS_HOLD_TYPE_HARD_RESET);
	/* reset "reboot reason" by to 0 */
	//regmap_write(pon->regmap, pon->baseaddr + PON_SOFT_RB_SPARE, 0);
	/* set PON interrupts to go to APSS (the reset value is 3??) */
	//regmap_write(pon->regmap, pon->baseaddr + PON_INT_MID_SEL, 0);

	pon->code = SYS_POWER_OFF;

	WARN_ON(pon_global);
	pon_global = pon;

	pm_power_off = do_msm_poweroff;

	//ret = regmap_read(pon->regmap, pon->baseaddr + PON_PON_REASON1, &pmic_pon_reason_regval);
	//dev_warn(&pdev->dev, "PON_PON_REASON1 0x%02X ret=%d\n", pmic_pon_reason_regval,ret);
	pon->pmic_pon_reason_regval = 0;//pmic_pon_reason_regval;

	return devm_of_platform_populate(&pdev->dev);
}

static const struct of_device_id pm8916_pon_id_table[] = {
	{ .compatible = "qcom,pm8916-pon", .data = (void *)GEN1_REASON_SHIFT },
	{ .compatible = "qcom,pm8941-pon", .data = (void *)NO_REASON_SHIFT },
	{ .compatible = "qcom,pms405-pon", .data = (void *)GEN1_REASON_SHIFT },
	{ .compatible = "qcom,pm8998-pon", .data = (void *)GEN2_REASON_SHIFT },
	{ .compatible = "qcom,pmk8350-pon", .data = (void *)GEN2_REASON_SHIFT },
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
