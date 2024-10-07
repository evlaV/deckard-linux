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

struct valve_5v_exp_vreg {
	struct regulator_desc desc;
	unsigned int enable_counter;
	struct gpio_desc *gpio;
};

static int reg_valve_5v_exp_enable(struct regulator_dev *rdev)
{
	struct valve_5v_exp_vreg *priv = rdev_get_drvdata(rdev);

	gpiod_set_value(priv->gpio, 1);
	priv->enable_counter=1;

	return 0;
}

static int reg_valve_5v_exp_disable(struct regulator_dev *rdev)
{
	struct valve_5v_exp_vreg *priv = rdev_get_drvdata(rdev);

	gpiod_set_value(priv->gpio, 0);
	priv->enable_counter=0;

	return 0;
}

static int reg_valve_5v_exp_is_enabled(struct regulator_dev *rdev)
{
	struct valve_5v_exp_vreg *priv = rdev_get_drvdata(rdev);
	return priv->enable_counter != 0;
}

static const struct regulator_ops valve_5v_exp_ops = {
	.enable = reg_valve_5v_exp_enable,
	.disable = reg_valve_5v_exp_disable,
	.is_enabled = reg_valve_5v_exp_is_enabled,
};

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	struct valve_5v_exp_vreg *priv = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", priv->enable_counter != 0);
}
static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret, enable;
	struct valve_5v_exp_vreg *priv = dev_get_drvdata(dev);
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

	gpiod_set_value(priv->gpio, enable);
	priv->enable_counter = enable;

	return count;
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

static int reg_valve_5v_exp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node;
	struct regulator_dev *rdev;
	struct valve_5v_exp_vreg *vreg;
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
	vreg->desc.name = "exp_5v_out";
	vreg->desc.owner = THIS_MODULE;
	vreg->desc.ops = &valve_5v_exp_ops;

	reg_config.dev = dev;
	reg_config.init_data = init_data;
	reg_config.of_node	= node;
	reg_config.driver_data	= (void*) ((size_t) vreg);

	vreg->desc.n_voltages = 1;
	vreg->desc.fixed_uV = init_data->constraints.min_uV;

	vreg->gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(vreg->gpio))
		return PTR_ERR(vreg->gpio);

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
		.compatible = "regulator-valve_5v_exp",
	},
	{
	},
};
MODULE_DEVICE_TABLE(of, fixed_of_match);
#endif

static struct platform_driver regulator_fixed_voltage_driver = {
	.probe		= reg_valve_5v_exp_probe,
	.driver		= {
		.name		= "reg-valve_5v_exp",
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

MODULE_DESCRIPTION("Valve 5V Expansion Regulator");
MODULE_LICENSE("GPL");
