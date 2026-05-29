#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/regmap.h>

#include <linux/workqueue.h>

// MPQ3367 register addresses
#define MPQ3367_FUNC1_SET_REG   0x00
#define MPQ3367_FUNC2_SET_REG   0x01
#define MPQ3367_FAULT_REG       0x02
#define MPQ3367_ID_REG          0x03

const char *mpq3367_dimming_modes[] = {"mix", "pwm", "analog"};

#define MPQ3367_DIMMING_SHIFT   5
#define MPQ3367_DIMMING_MASK    0x03
#define MPQ3367_MIX_DIMMING     0x00
#define MPQ3367_PWM_DIMMING     0x01
#define MPQ3367_ANALOG_DIMMING  0x02
#define MPQ3367_RSVD_DIMMING    0x03

#define MPQ3367_I2C_MAX_RETRY   20

struct mpq3367_data {
	struct i2c_client *client;
	struct regmap *regmap;
	struct device *dev;
	struct regulator_desc desc;
	bool is_enabled;
	struct gpio_desc *en_gpio;
	unsigned int dimming_mode;
	struct delayed_work backlight_work;
};

static int mpq_read(struct mpq3367_data *mpq, unsigned int reg, unsigned int *val)
{
	return regmap_read(mpq->regmap, reg, val);
}

static int mpq3367_set_dimming_mode(struct mpq3367_data *mpq) {
	unsigned int mask = MPQ3367_DIMMING_MASK << MPQ3367_DIMMING_SHIFT;
	unsigned int val;

	if (mpq->dimming_mode >= MPQ3367_RSVD_DIMMING)
		return -EINVAL;

	val = mpq->dimming_mode << MPQ3367_DIMMING_SHIFT;

	return regmap_update_bits(mpq->regmap, MPQ3367_FUNC1_SET_REG, mask, val);
}

static void backlight_delayed_work_handler(struct work_struct *work)
{
	// check to see if backlight is still in the correct mode
	unsigned int mode;
	int ret;
	int loopctr=0;

	struct mpq3367_data *mpq = container_of(to_delayed_work(work),
						struct mpq3367_data, backlight_work);

	ret = mpq_read(mpq, MPQ3367_FUNC1_SET_REG, &mode);
	if (ret) {
		dev_err(mpq->dev, "read failed in work handler (ret=%d)\n", ret);
		return;
	}

	mode = (mode >>  MPQ3367_DIMMING_SHIFT) & MPQ3367_DIMMING_MASK;

	if (ret || mode != mpq->dimming_mode) {
		dev_err(mpq->dev, "failed init check, resetting mode\n");
		do {
			ret = mpq3367_set_dimming_mode(mpq);
			if(!ret)
				break;

			if(loopctr > MPQ3367_I2C_MAX_RETRY) {
				dev_err(mpq->dev, "i2c write failed too many times, quitting\n");
				break;
			} else {
				loopctr++;
				usleep_range(50000, 100000);
				dev_err(mpq->dev, "i2c write fail, retrying... [%d]\n",
					loopctr);
			}
		} while (ret < 0);

		dev_err(mpq->dev, "reinit complete\n");
	}
}

static int reg_enable(struct regulator_dev *rdev)
{
	struct mpq3367_data *mpq = rdev_get_drvdata(rdev);
	struct device *dev = mpq->dev;
	int ret;
	int i;

	if (mpq->is_enabled)
		return 0;

	gpiod_set_value(mpq->en_gpio, 1);
	usleep_range(10000, 20000);

	for (i = 0; i < MPQ3367_I2C_MAX_RETRY; i++) {
		ret = mpq3367_set_dimming_mode(mpq);
		if (!ret)
			break;
	}

	if (ret) {
		dev_err(dev, "i2c write failed too many times, quitting\n");
		return -EIO;
	}

	mpq->is_enabled = true;

	// found empirically at 120Hz it takes 10 frames to boost backlight capacitors up to voltage
	// the backlight may brown out during this process, and need to be re-initialized
	// 10 frames @120Hz ~ 85 msec
	// 10 frames @80Hz  ~ 125 msec
	// backlight pulses are only output when panel starts receving frames, so we give ourselves a bit of a buffer and
	// schedule the timer to come back in 1 sec
	if (delayed_work_pending(&mpq->backlight_work))
		cancel_delayed_work_sync(&mpq->backlight_work);

	queue_delayed_work(system_freezable_wq, &mpq->backlight_work, HZ);

	return 0;
}

static int reg_disable(struct regulator_dev *rdev)
{
	struct mpq3367_data *mpq = rdev_get_drvdata(rdev);

	if (!mpq->is_enabled)
		return 0;

	cancel_delayed_work_sync(&mpq->backlight_work);
	gpiod_set_value(mpq->en_gpio, 0);

	mpq->is_enabled = false;

	return 0;
}

static int reg_is_enabled(struct regulator_dev *rdev)
{
	struct mpq3367_data *mpq = rdev_get_drvdata(rdev);

	return mpq->is_enabled;
}

static const struct regulator_ops mpq3367_ops = {
	.enable = reg_enable,
	.disable = reg_disable,
	.is_enabled = reg_is_enabled,
};

/* -----------------------------------------------------------------------------
 * sysfs
 */
static ssize_t mpq3367_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mpq3367_data *mpq = dev_get_drvdata(dev);
	unsigned int reg[4];
	int ret;

	ret = mpq_read(mpq, MPQ3367_FUNC1_SET_REG, &reg[0]);
	ret |= mpq_read(mpq, MPQ3367_FUNC2_SET_REG, &reg[1]);
	ret |= mpq_read(mpq, MPQ3367_FAULT_REG, &reg[2]);
	ret |= mpq_read(mpq, MPQ3367_ID_REG, &reg[3]);
	if (ret) {
		dev_err(mpq->dev, "read failed (ret=%d)\n", ret);
		memset(reg, 0xff, sizeof(reg));
	}

	return sprintf(buf,
		"%.2x %.2x %.2x %.2x\n",
		reg[0], reg[1], reg[2], reg[3]);
}

static DEVICE_ATTR_RO(mpq3367_status);

static struct attribute *mpq3367_attrs[] = {
	&dev_attr_mpq3367_status.attr,
	NULL,
};

static const struct attribute_group mpq3367_attr_group = {
	.name = "mpq3367",
	.attrs = mpq3367_attrs,
};

static bool mpq3367_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MPQ3367_FUNC1_SET_REG:
	case MPQ3367_FUNC2_SET_REG:
	case MPQ3367_FAULT_REG:
	case MPQ3367_ID_REG:
		return true;
	default:
		return false;
	}
}

static bool mpq3367_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MPQ3367_FUNC1_SET_REG:
	case MPQ3367_FUNC2_SET_REG:
	case MPQ3367_FAULT_REG:
	case MPQ3367_ID_REG:
		return true;
	default:
		return false;
	}
}

static bool mpq3367_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MPQ3367_FUNC1_SET_REG:
	case MPQ3367_FUNC2_SET_REG:
	case MPQ3367_FAULT_REG:
	case MPQ3367_ID_REG:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config mpq3367_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = mpq3367_writeable_reg,
	.readable_reg = mpq3367_readable_reg,
	.volatile_reg = mpq3367_volatile_reg,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_NONE,
};

static struct of_regulator_match match = { .name = "backlight" };

static int mpq3367_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config cfg = { };
	struct regulator_dev *rdev;
	struct mpq3367_data *mpq;
	struct device_node *regulator_node;
	const char *mode = NULL;
	int ret = 0, i;

	mpq = devm_kzalloc(dev, sizeof(*mpq), GFP_KERNEL);
	if (!mpq)
		return -ENOMEM;

	mpq->en_gpio = devm_gpiod_get(dev, "en", GPIOD_OUT_LOW);
	if (IS_ERR(mpq->en_gpio)) {
		dev_err(dev, "cannot get en gpio %ld\n",
			PTR_ERR(mpq->en_gpio));
	}

	// default to PWM dimming if not set in DTS
	mpq->dimming_mode = MPQ3367_PWM_DIMMING;
	if ( !of_property_read_string(dev->of_node, "mode", &mode) ) {
		for(i = 0; i < ARRAY_SIZE(mpq3367_dimming_modes); i++){
			if (!strcmp(mpq3367_dimming_modes[i], mode)) {
				mpq->dimming_mode = i;
				break;
			}
		}
	}

	mpq->regmap = devm_regmap_init_i2c(client, &mpq3367_regmap_config);
	if (IS_ERR(mpq->regmap)) {
		dev_err(dev, "Failed to allocate register map: %ld\n",
			PTR_ERR(mpq->regmap));
		return PTR_ERR(mpq->regmap);
	}

	regulator_node = of_get_child_by_name(dev->of_node, "regulator");

	ret = of_regulator_match(dev, regulator_node, &match, 1);
	if (ret < 0) {
		dev_err(dev, "Regulator match failed, e %d\n", ret);
		return ret;
	}

	cfg.dev = dev;
	cfg.init_data = match.init_data;
	cfg.driver_data = mpq;
	cfg.of_node = match.of_node;

	if (!cfg.init_data)
		return -EINVAL;

	of_property_read_string(match.of_node, "regulator-name", &mpq->desc.name);
	mpq->desc.ops = &mpq3367_ops;
	mpq->desc.type = REGULATOR_VOLTAGE;

	rdev = devm_regulator_register(dev, &mpq->desc, &cfg);
	if (IS_ERR(rdev)) {
		dev_err(dev, "Failed to register regulator: %d\n", (int) PTR_ERR(rdev));
		return PTR_ERR(rdev);
	}

	ret = sysfs_create_group(&dev->kobj, &mpq3367_attr_group);
	if (ret < 0) {
		dev_err(dev, "failed to create mpq sysfs files\n");
		return ret;
	}

	INIT_DELAYED_WORK(&mpq->backlight_work, backlight_delayed_work_handler);

	mpq->client = client;
	mpq->dev = dev;
	i2c_set_clientdata(client, mpq);
	return 0;
}

static const struct of_device_id mpq3367_of_match[] = {
	{ .compatible = "mps,mpq3367", },
	{ },
};

static const struct i2c_device_id mpq3367_i2c_ids[] = {
	{ "mpq3367", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mpq3367_i2c_ids);

static struct i2c_driver mpq3367_driver = {
	.probe		= mpq3367_probe,
	.driver		= {
		.name		= "mpq3367",
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = of_match_ptr(mpq3367_of_match),
	},
};
module_i2c_driver(mpq3367_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Valve");
MODULE_DESCRIPTION("MPS MPQ3367 6-Channel, Max.150mA/Ch Boost WLED Driver");
