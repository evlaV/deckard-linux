// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * driver for 4 regulators, with 2 supplies and 2 gpios:
 * r0 supplied by r-supply, enabled by first gpio
 * r1 supplied by r-supply, enabled by second gpio
 * l0 supplied by l-supply, enabled by first gpio
 * l1 supplied by l-supply, enabled by second gpio
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

struct fixed2_vreg {
	struct regulator_desc desc[4];
	unsigned int enable_counter2[4];
	unsigned int enable_counter[2];
	struct gpio_desc *gpio[2];
	spinlock_t lock;
};

#define GET_DRIVER_DATA() ({ \
	size_t data = (size_t) rdev_get_drvdata(rdev); \
	vreg = (void*) (data & ~3ul); i = data & 3; \
})

static int reg_fixed2_enable(struct regulator_dev *rdev)
{
	struct fixed2_vreg *vreg;
	unsigned long flags, i;
	GET_DRIVER_DATA();

	spin_lock_irqsave(&vreg->lock, flags);
	if (vreg->enable_counter[i/2]++ == 0)
		gpiod_set_value(vreg->gpio[i/2], 1);
	spin_unlock_irqrestore(&vreg->lock, flags);

	vreg->enable_counter2[i]++;

	return 0;
}

static int reg_fixed2_disable(struct regulator_dev *rdev)
{
	struct fixed2_vreg *vreg;
	unsigned long flags, i;
	GET_DRIVER_DATA();

	spin_lock_irqsave(&vreg->lock, flags);
	if (--vreg->enable_counter[i/2] == 0)
		gpiod_set_value(vreg->gpio[i/2], 0);
	spin_unlock_irqrestore(&vreg->lock, flags);

	vreg->enable_counter2[i]--;

	return 0;
}

static int reg_is_enabled(struct regulator_dev *rdev)
{
	struct fixed2_vreg *vreg;
	unsigned long i;
	GET_DRIVER_DATA();

	return vreg->enable_counter2[i] != 0;
}

static const struct regulator_ops fixed2_ops = {
	.enable = reg_fixed2_enable,
	.disable = reg_fixed2_disable,
	.is_enabled = reg_is_enabled,
};

static const char *name[] = {
	"r0",
	"l0",
	"r1",
	"l1",
};

static int reg_fixed2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node;
	struct regulator_dev *rdev;
	struct fixed2_vreg *vreg;
	int i;

	vreg = devm_kzalloc(dev, sizeof(*vreg), GFP_KERNEL);
	if (!vreg) {
		of_node_put(node);
		return -ENOMEM;
	}

	spin_lock_init(&vreg->lock);

	for (i = 0; i < ARRAY_SIZE(vreg->gpio); i++) {
		vreg->gpio[i] = devm_gpiod_get_index(dev, NULL, i, GPIOD_OUT_LOW);
		if (IS_ERR(vreg->gpio[i]))
			return PTR_ERR(vreg->gpio[i]);
	}

	for (i = 0; i < ARRAY_SIZE(vreg->desc); i++) {
		struct regulator_config reg_config = {};
		struct regulator_init_data *init_data;

		node = of_get_child_by_name(dev->of_node, name[i]);
		if (!node)
			continue;

		vreg->desc[i].name = name[i];
		vreg->desc[i].supply_name = (i & 1) ? "l" : "r";
		vreg->desc[i].owner = THIS_MODULE;
		vreg->desc[i].ops = &fixed2_ops;

		init_data = of_get_regulator_init_data(dev, node, &vreg->desc[i]);
		if (!init_data) {
			of_node_put(node);
			return -ENOMEM;
		}

		reg_config.dev		= dev;
		reg_config.init_data	= init_data;
		reg_config.of_node	= node;
		reg_config.driver_data	= (void*) ((size_t) vreg | i);

		vreg->desc[i].n_voltages = 1;
		vreg->desc[i].fixed_uV = init_data->constraints.min_uV;

		rdev = devm_regulator_register(dev, &vreg->desc[i], &reg_config);
		if (IS_ERR(rdev)) {
			// free init_data on failure?
			printk("devm_regulator_register failed\n");
			of_node_put(node);
			return PTR_ERR(rdev);
		}
	}

	return 0;
}

#if defined(CONFIG_OF)

static const struct of_device_id fixed_of_match[] = {
	{
		.compatible = "regulator-fixed2",
	},
	{
	},
};
MODULE_DEVICE_TABLE(of, fixed_of_match);
#endif

static struct platform_driver regulator_fixed_voltage_driver = {
	.probe		= reg_fixed2_probe,
	.driver		= {
		.name		= "reg-fixed-voltage2",
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

MODULE_DESCRIPTION("Fixed voltage regulator 2");
MODULE_LICENSE("GPL");
