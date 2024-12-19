// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * driver for Valve 5V Expansion Rail
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/gpio/consumer.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>

struct valve_rgb_3v0_vreg {
	struct regulator_desc desc;
	unsigned int enable_counter;
	struct gpio_desc *enable;
	struct gpio_desc *detect;
};

static int reg_valve_rgb_3v0_enable(struct regulator_dev *rdev)
{
	struct valve_rgb_3v0_vreg *priv = rdev_get_drvdata(rdev);

	// detect pin must be low to enable power
	if ( gpiod_get_value(priv->detect) == 0 ) {
		gpiod_set_value(priv->enable, 1);
		priv->enable_counter=1;
		return 0;
	} else {
		return -EAGAIN;
	}
}

static int reg_valve_rgb_3v0_disable(struct regulator_dev *rdev)
{
	struct valve_rgb_3v0_vreg *priv = rdev_get_drvdata(rdev);

	gpiod_set_value(priv->enable, 0);
	priv->enable_counter=0;

	return 0;
}

static int reg_valve_rgb_3v0_is_enabled(struct regulator_dev *rdev)
{
	struct valve_rgb_3v0_vreg *priv = rdev_get_drvdata(rdev);
	return priv->enable_counter != 0;
}

static const struct regulator_ops valve_rgb_3v0_ops = {
	.enable = reg_valve_rgb_3v0_enable,
	.disable = reg_valve_rgb_3v0_disable,
	.is_enabled = reg_valve_rgb_3v0_is_enabled,
};

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	struct valve_rgb_3v0_vreg *priv = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", priv->enable_counter != 0);
}
static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret, enable;
	struct valve_rgb_3v0_vreg *priv = dev_get_drvdata(dev);
	ret = sscanf(buf, "%d", &enable);

	if ( ret != 1 ){
		pr_debug("Invalid number of params for enable\n");
		return -1;
	}

	// force enable to be 1 or 0
	if ( enable ) {
		enable = 1;
	} else {
		enable = 0;
	}

	// detect pin must be low to enable power
	if ( !(gpiod_get_value(priv->detect) == 0) && enable ) {
		dev_err(dev, "Cannot enable RGB 3V0, detect pin not low\n");
		return -EAGAIN;
	} else {
		gpiod_set_value(priv->enable, enable);
		priv->enable_counter = enable;
		return count;
	}
}
static DEVICE_ATTR_RW(enable);

static struct attribute *control_attrs[] = {
		&dev_attr_enable.attr,
		NULL,
};

static const struct attribute_group control_attr_group = {
		.name = "control",
		.attrs = control_attrs,
};

static int reg_valve_rgb_3v0_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node;
	struct regulator_dev *rdev;
	struct valve_rgb_3v0_vreg *vreg;
	struct regulator_init_data *init_data;
	struct regulator_config reg_config = {};
	int ret;

	vreg = devm_kzalloc(dev, sizeof(*vreg), GFP_KERNEL);
	if (!vreg) {
		of_node_put(node);
		return -ENOMEM;
	}

	init_data = of_get_regulator_init_data(dev, dev->of_node, &vreg->desc);
	if (!init_data)
		return -ENOMEM;

	vreg->desc.supply_name = "VPH_PWR";
	vreg->desc.name = "rgb_3v0_out";
	vreg->desc.owner = THIS_MODULE;
	vreg->desc.ops = &valve_rgb_3v0_ops;

	reg_config.dev = dev;
	reg_config.init_data = init_data;
	reg_config.of_node	= node;
	reg_config.driver_data	= (void*) ((size_t) vreg);

	vreg->desc.n_voltages = 1;
	vreg->desc.fixed_uV = init_data->constraints.min_uV;

	vreg->enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(vreg->enable))
		return PTR_ERR(vreg->enable);

	vreg->detect = devm_gpiod_get(dev, "detect", GPIOD_IN);
	if (IS_ERR(vreg->detect))
		return PTR_ERR(vreg->detect);

	rdev = devm_regulator_register(dev, &vreg->desc, &reg_config);
	if (IS_ERR(rdev)) {
		// free init_data on failure?
		printk("devm_regulator_register failed\n");
		of_node_put(node);
		return PTR_ERR(rdev);
	}

	platform_set_drvdata(pdev, vreg);

	ret = sysfs_create_group(&pdev->dev.kobj, &control_attr_group);
	if (ret != 0)
		return ret;

	return 0;
}

#if defined(CONFIG_OF)

static const struct of_device_id fixed_of_match[] = {
	{
		.compatible = "regulator-valve_rgb_vout",
	},
	{
	},
};
MODULE_DEVICE_TABLE(of, fixed_of_match);
#endif

static struct platform_driver regulator_fixed_voltage_driver = {
	.probe		= reg_valve_rgb_3v0_probe,
	.driver		= {
		.name		= "reg-valve-rgb-3v0",
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = of_match_ptr(fixed_of_match),
	},
};

static int __init regulator_fixed_voltage_init(void)
{
	return platform_driver_register(&regulator_fixed_voltage_driver);
}
subsys_initcall(regulator_fixed_voltage_init);

static void __exit regulator_fixed_voltage_exit(void)
{
	platform_driver_unregister(&regulator_fixed_voltage_driver);
}
module_exit(regulator_fixed_voltage_exit);

MODULE_DESCRIPTION("Valve RGB 3V0 Regulator");
MODULE_LICENSE("GPL");
