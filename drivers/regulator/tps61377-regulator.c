#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>

#include <linux/workqueue.h>

#define TPS61377_ENABLE_TIME_USEC  20000

struct tps61377_data {
	struct regulator_desc desc;
	struct device *dev;
	struct gpio_desc *en_gpio;
    struct gpio_desc *pgood_gpio;
};

static int tps61377_reg_enable(struct regulator_dev *rdev)
{
	struct tps61377_data *tps61377 = rdev_get_drvdata(rdev);
	gpiod_set_value(tps61377->en_gpio, 1);
	return 0;
}

static int tps61377_reg_disable(struct regulator_dev *rdev)
{
	struct tps61377_data *tps61377 = rdev_get_drvdata(rdev);

	gpiod_set_value(tps61377->en_gpio, 0);
	return 0;
}

static int tps61377_reg_is_enabled(struct regulator_dev *rdev)
{
	struct tps61377_data *tps61377 = rdev_get_drvdata(rdev);
    return gpiod_get_value(tps61377->en_gpio);
}

static int tps61377_regulator_enable_time(struct regulator_dev *rdev) {
	return TPS61377_ENABLE_TIME_USEC;
}
static int tps61377_reg_get_status(struct regulator_dev *rdev)
{
	struct tps61377_data *tps61377 = rdev_get_drvdata(rdev);
	if (gpiod_get_value(tps61377->pgood_gpio)){
        return REGULATOR_STATUS_ON;
    } else {
        return REGULATOR_STATUS_OFF;
    }
}

static const struct regulator_ops tps61377_ops = {
	.enable = tps61377_reg_enable,
	.disable = tps61377_reg_disable,
	.is_enabled = tps61377_reg_is_enabled,
    .enable_time = tps61377_regulator_enable_time,
    .get_status = tps61377_reg_get_status,
};

static ssize_t pgood_show(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	struct tps61377_data *tps61377= dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", gpiod_get_value(tps61377->pgood_gpio));
}
static DEVICE_ATTR_RO(pgood);

static struct attribute *status_attrs[] = {
		&dev_attr_pgood.attr,
		NULL,
};

static const struct attribute_group control_attr_group = {
		.name = "status",
		.attrs = status_attrs,
};

static int tps61377_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config cfg = { };
	struct regulator_dev *rdev;
	struct tps61377_data *tps61377;
	int ret;

	tps61377 = devm_kzalloc(dev, sizeof(*tps61377), GFP_KERNEL);
	if (!tps61377)
		return -ENOMEM;

	tps61377->en_gpio = devm_gpiod_get(dev, "en", GPIOD_OUT_LOW);
	if (IS_ERR(tps61377->en_gpio))
		return PTR_ERR(tps61377->en_gpio);

	tps61377->pgood_gpio = devm_gpiod_get(dev, "pgood", GPIOD_IN);
	if (IS_ERR(tps61377->pgood_gpio))
		return PTR_ERR(tps61377->pgood_gpio);

	cfg.dev = dev;
	cfg.init_data = of_get_regulator_init_data(dev, dev->of_node, &tps61377->desc);
	cfg.driver_data = tps61377;
	cfg.of_node = dev->of_node;

	if (!cfg.init_data)
		return -EINVAL;

	tps61377->desc.name = "tps61377";
	tps61377->desc.ops = &tps61377_ops;
	tps61377->desc.type = REGULATOR_VOLTAGE;

	rdev = devm_regulator_register(dev, &tps61377->desc, &cfg);
	if (IS_ERR(rdev)) {
		dev_err(dev, "Failed to register regulator: %d\n", (int) PTR_ERR(rdev));
		return PTR_ERR(rdev);
	}

	tps61377->dev = dev;
	platform_set_drvdata(pdev, tps61377);

	ret = sysfs_create_group(&pdev->dev.kobj, &control_attr_group);
	if (ret != 0)
		return ret;

	return 0;
}

static const struct of_device_id tps61377_of_match[] = {
	{ .compatible = "ti,tps61377", },
	{ },
};

static struct platform_driver tps61377_driver = {
	.probe		= tps61377_probe,
	.driver		= {
		.name		= "tps61377",
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = of_match_ptr(tps61377_of_match),
	},
};
module_platform_driver(tps61377_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI 23-VIN, 25-VOUT 6-A synchronous boost converter with programmable peak current limit");
MODULE_AUTHOR("Valve");
