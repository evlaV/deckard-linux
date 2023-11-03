#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>

#include <linux/workqueue.h>

#define KTD_REGISTER_VPOS 0x00
#define KTD_REGISTER_VNEG 0x01

struct ktd2151 {
	struct regulator_desc desc;
	struct regmap *regmap;
	struct regulator *vin;
	bool is_enabled;
};


static int reg_enable(struct regulator_dev *rdev)
{
	struct ktd2151 *ktd = rdev_get_drvdata(rdev);
	struct device *dev = &rdev->dev;
	int ret;
	int loopctr=0;

	/* default setting */
	dev_info(dev, "ktd2151: reg enable\n");
	
	/* set vpos to 5.7 */
	ret = regmap_write(ktd->regmap, KTD_REGISTER_VPOS, 0x11); //+5.7V
	if (ret < 0) printk("ktd2151 failed regmap write1\n");
	else printk("ktd2151 OK regmap write1\n");

    /* set vneg to 5.7 */
	ret = regmap_write(ktd->regmap, KTD_REGISTER_VNEG, 0x11); //-5.7V
	if (ret < 0) printk("ktd2151 failed regmap write2\n");
	else printk("ktd2151 OK regmap write2\n");

	ktd->is_enabled = true;

	return 0;
}

static int reg_disable(struct regulator_dev *rdev)
{
	struct ktd2151 *ktd = rdev_get_drvdata(rdev);

	ktd->is_enabled = false;
	return 0;
}

static int reg_is_enabled(struct regulator_dev *rdev)
{
    int ret = 0;
	struct ktd2151 *ktd = rdev_get_drvdata(rdev);

	return ktd->is_enabled;
}

static const struct regulator_ops reg_ops = {
	.enable = reg_enable,
	.disable = reg_disable,
	.is_enabled = reg_is_enabled,
};

static const struct regmap_config ktd2151_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x03, // XXX
};

static int
ktd2151_probe(struct i2c_client *client)
{
	struct regulator_config cfg = { };
	struct device *dev = &client->dev;
	struct regulator_dev *rdev;
	struct ktd2151 *ktd;

	ktd = devm_kzalloc(dev, sizeof(*ktd), GFP_KERNEL);
	if (!ktd)
		return -ENOMEM;

    dev_info(dev, "ktd2151 probe\n");

	ktd->regmap = devm_regmap_init_i2c(client, &ktd2151_regmap_config);
	if (IS_ERR(ktd->regmap)) {
        dev_err(dev, "ktd2151 failed regmap\n");
		return PTR_ERR(ktd->regmap);
    }

	cfg.dev = dev;
	cfg.init_data = of_get_regulator_init_data(dev, dev->of_node, &ktd->desc);
	cfg.driver_data = ktd;
	cfg.of_node = dev->of_node;

	if (!cfg.init_data) {
        dev_err(dev, "ktd2151 failed init_data\n");
		return -EINVAL;
    }

	ktd->desc.name = "ktd";
	ktd->desc.ops = &reg_ops;
	ktd->desc.type = REGULATOR_VOLTAGE;

	rdev = devm_regulator_register(dev, &ktd->desc, &cfg);
	if (IS_ERR(rdev)) {
		dev_err(dev, "ktd2151 Failed to register regulator: %d\n", (int) PTR_ERR(rdev));
		return PTR_ERR(rdev);
	}

    dev_info(dev, "ktd2151 probe OK\n");

	dev_set_drvdata(dev, ktd);
	return 0;
}

static const struct of_device_id ktd2151_dt_match[] = {
	{ .compatible = "ktd2151" },
	{ }
};

static struct i2c_driver ktd2151_driver = {
	.driver = {
	    .name = "ktd2151",
	    .of_match_table = ktd2151_dt_match,
	},
	.probe = ktd2151_probe,
};

module_i2c_driver(ktd2151_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ktd2151 driver");
