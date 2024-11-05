#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>

#include <linux/workqueue.h>

struct mp9184_data {
	struct regulator_desc desc;
	struct device *dev;
	bool is_enabled;
	struct gpio_desc *en_gpio;
};

static int reg_enable(struct regulator_dev *rdev)
{
	struct mp9184_data *mp9184 = rdev_get_drvdata(rdev);

	//the BOOST_CYCLE_COUNT is here to help the preboost boot up faster to the target voltage
	#define BOOST_CYCLE_COUNT 40
	dev_err(mp9184->dev, "starting preboost: cycle count = %d\n", BOOST_CYCLE_COUNT);
	for(int i=0; i < BOOST_CYCLE_COUNT; i++) {
		//these times are fairly tightly tuned
		//to avoid going into some high current conditions
		gpiod_set_value(mp9184->en_gpio, 0); usleep_range(200, 400);
		gpiod_set_value(mp9184->en_gpio, 1); usleep_range(1200, 1300);
	}
	gpiod_set_value(mp9184->en_gpio, 1);

	dev_err(mp9184->dev, "preboost startup done\n");
	//we need to delay to wait for hiccup mode to exit --
	//at a minimum it is about 70ms we need to wait here...
	usleep_range(400000, 450000);

	mp9184->is_enabled = true;

	return 0;
}

static int reg_disable(struct regulator_dev *rdev)
{
	struct mp9184_data *mp9184 = rdev_get_drvdata(rdev);

	dev_err(mp9184->dev, "disabling preboost\n");
	gpiod_set_value(mp9184->en_gpio, 0);

	mp9184->is_enabled = false;
	return 0;
}

static int reg_is_enabled(struct regulator_dev *rdev)
{
	struct mp9184_data *mp9184 = rdev_get_drvdata(rdev);
	return mp9184->is_enabled;
}

static const struct regulator_ops mp9184_ops = {
	.enable = reg_enable,
	.disable = reg_disable,
	.is_enabled = reg_is_enabled,
};

static int mp9184_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config cfg = { };
	struct regulator_dev *rdev;
	struct mp9184_data *mp9184;
	int ret = 0;

	mp9184 = devm_kzalloc(dev, sizeof(*mp9184), GFP_KERNEL);
	if (!mp9184)
		return -ENOMEM;

	mp9184->en_gpio = devm_gpiod_get(dev, "en", GPIOD_OUT_LOW);
	if (IS_ERR(mp9184->en_gpio))
		return PTR_ERR(mp9184->en_gpio);

	cfg.dev = dev;
	cfg.init_data = of_get_regulator_init_data(dev, dev->of_node, &mp9184->desc);
	cfg.driver_data = mp9184;
	cfg.of_node = dev->of_node;

	if (!cfg.init_data)
		return -EINVAL;

	mp9184->desc.name = "mp9184";
	mp9184->desc.ops = &mp9184_ops;
	mp9184->desc.type = REGULATOR_VOLTAGE;

	rdev = devm_regulator_register(dev, &mp9184->desc, &cfg);
	if (IS_ERR(rdev)) {
		dev_err(dev, "Failed to register regulator: %d\n", (int) PTR_ERR(rdev));
		return PTR_ERR(rdev);
	}

	mp9184->dev = dev;
	platform_set_drvdata(pdev, mp9184);
	return 0;
}

static const struct of_device_id mp9184_of_match[] = {
	{ .compatible = "mps,mp9184", },
	{ },
};

static struct platform_driver mp9184_driver = {
	.probe		= mp9184_probe,
	.driver		= {
		.name		= "mp9184",
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = of_match_ptr(mp9184_of_match),
	},
};
module_platform_driver(mp9184_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MPS MP9184A 19A, 600kHz, 20V Wide Input Range, Synchronous Boost Converter driver");
MODULE_AUTHOR("Valve");
