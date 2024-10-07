// SPDX-License-Identifier: GPL-2.0
/*
 * Basic driver to enable power and clock to Efinix T8F81C2 FPGA.
 * QSPI interactions are handled via user space application
 *
 *  Copyright (C) Valve Corporation
 */
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pm.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>

struct t8f81c2_fpga
{
    struct clk *clk;
    struct gpio_desc *pwren_gpio;
    struct regulator *v1v1;
};

static int t8f81c2_fpga_suspend(struct platform_device * dev, pm_message_t state)
{
    struct t8f81c2_fpga *t8f81c2 = platform_get_drvdata(dev);
    // Power off and disable clock
    clk_disable_unprepare(t8f81c2->clk);
    regulator_disable(t8f81c2->v1v1);
    gpiod_set_value(t8f81c2->pwren_gpio, 0); // needs to be enabled within 10msec of 1v1
    return 0;
}


static int t8f81c2_fpga_resume(struct platform_device * dev)
{
    struct t8f81c2_fpga *t8f81c2 = platform_get_drvdata(dev);
    // Power on and enable clock
    regulator_enable(t8f81c2->v1v1);
    gpiod_set_value(t8f81c2->pwren_gpio, 1); // needs to be enabled within 10msec of 1v1
    clk_prepare_enable(t8f81c2->clk);
    return 0;
}

static ssize_t 
enable_show(struct device *dev, struct device_attribute *attr,
    char *buf)
{
    struct t8f81c2_fpga *t8f81c2 = dev_get_drvdata(dev);
	buf[0] = gpiod_get_value(t8f81c2->pwren_gpio) + '0';
	buf[1] = '\n';
	return 2;
}

static ssize_t
enable_store(struct device *dev, struct device_attribute *attr,
	  const char *buf, size_t count)
{
    int ret;
	int enable;
    struct t8f81c2_fpga *t8f81c2 = dev_get_drvdata(dev);
	ret = sscanf(buf, "%d", &enable);

    if ( ret != 1){
        pr_debug("Invalid number of params for enable\n");
        return -1;
    }

    if (enable){
        // Power on and enable clock
        regulator_enable(t8f81c2->v1v1);
        gpiod_set_value(t8f81c2->pwren_gpio, 1); // needs to be enabled within 10msec of 1v1
        clk_prepare_enable(t8f81c2->clk);
    }
    else{
        // Power off and disable clock
        clk_disable_unprepare(t8f81c2->clk);
        regulator_disable(t8f81c2->v1v1);
        gpiod_set_value(t8f81c2->pwren_gpio, 0); // needs to be enabled within 10msec of 1v1
    }

	return count;
}
static DEVICE_ATTR_RW(enable);

static struct attribute *t8f81c2_attrs[] = {
    &dev_attr_enable.attr,
	NULL,
};

static const struct attribute_group t8f81c2_attr_group = {
	.name = "t8f81c2",
    .attrs = t8f81c2_attrs,
};

static int t8f81c2_fpga_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
    struct t8f81c2_fpga *t8f81c2;
    int ret;

	t8f81c2 = devm_kzalloc(&pdev->dev, sizeof(*t8f81c2), GFP_KERNEL);
    if (!t8f81c2){
        return -ENOMEM;
    }

	t8f81c2->pwren_gpio = devm_gpiod_get(dev, "pwren", GPIOD_OUT_LOW);
	if (IS_ERR(t8f81c2->pwren_gpio)) {
		dev_err(dev, "cannot get power enable gpio %ld\n",
			PTR_ERR(t8f81c2->pwren_gpio));
		return PTR_ERR(t8f81c2->pwren_gpio);
	};

    t8f81c2->clk = devm_clk_get(&pdev->dev, "fpga_clk");
	if (IS_ERR(t8f81c2->clk)) {
        dev_err(dev, "cannot get clock %ld\n", PTR_ERR(t8f81c2->clk));
		return PTR_ERR(t8f81c2->clk);
	}

	t8f81c2->v1v1 = devm_regulator_get(dev, "v1v1");
	if (IS_ERR(t8f81c2->v1v1))
		return PTR_ERR(t8f81c2->v1v1);

    dev_set_drvdata(dev, t8f81c2);

    // Power on and enable clock
    //t8f81c2_fpga_resume(pdev);

    ret = sysfs_create_group(&pdev->dev.kobj, &t8f81c2_attr_group);
    if (ret < 0) {
        dev_err(&pdev->dev, "failed to create t8f81c2 sysfs files\n");
        return ret;
    }

    printk("t8f81c2 probe done\n");

    return 0;
}

static const struct of_device_id t8f81c2_fpga_of_match[] = {
	{ .compatible = "efinix,t8f81c2", },
	{},
};

MODULE_DEVICE_TABLE(of, t8f81c2_fpga_of_match);

static struct platform_driver t8f81c2_fpga_driver = {
	.probe = t8f81c2_fpga_probe,
    .suspend = t8f81c2_fpga_suspend,
    .resume = t8f81c2_fpga_resume,
	.driver = {
		.name	= "t8f81c2_fpga_pwr",
		.of_match_table = of_match_ptr(t8f81c2_fpga_of_match),
	},
};

module_platform_driver(t8f81c2_fpga_driver);

MODULE_DESCRIPTION("Efinix t8f81c2 FPGA Power");
MODULE_LICENSE("GPL v2");
