#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>

#include <linux/workqueue.h>

struct mpq3367 {
	struct regulator_desc desc;
	struct regmap *regmap;
	bool is_enabled;
	uint8_t required_reinit;
    struct delayed_work backlight_work;

    struct gpio_desc *en_gpio;
	struct gpio_desc *preboost_gpio;
	u32	backlight_index;
};

void backlight_delayed_work_handler(struct work_struct *work)
{
	// check to see if backlight is still in the correct mode
	uint8_t mpq3367_mode_reg;
	int ret;
	int loopctr=0;

	struct mpq3367 *mpq_device = container_of((struct delayed_work *)work,struct mpq3367,backlight_work);

	printk("mpq3367: init check\n");
	ret = regmap_bulk_read(mpq_device->regmap, 0x00, &mpq3367_mode_reg, sizeof(mpq3367_mode_reg));
	if(mpq3367_mode_reg != 0xa6)
	{
		printk("mpq3367: failed init check, resetting mode\n");
		do
		{
			ret = regmap_write(mpq_device->regmap, 0x00, 0xa6);

			if(loopctr++ > 0) {
				usleep_range(50000, 100000);
				printk("mpq3367: i2c write fail, retrying... [%d]\n", loopctr);
			}

			if(loopctr>20) {
				printk("mpq3367: i2c write failed too many times, quitting\n");
				break;
			}
		}while(ret<0);
		printk("mpq3367: reinit complete\n");
		mpq_device->required_reinit = 1;
	}
	else
	{
		printk("mpq3367: passed init check\n");
		mpq_device->required_reinit = 2;
	}
}

static void backlight_check_work_init(struct mpq3367 *mpq)
{
	INIT_DELAYED_WORK(&mpq->backlight_work, backlight_delayed_work_handler);
	schedule_delayed_work(&mpq->backlight_work, HZ);
}

static int reg_enable(struct regulator_dev *rdev)
{
	struct mpq3367 *mpq = rdev_get_drvdata(rdev);
	struct device *dev = &rdev->dev;
	int ret;
	int loopctr=0;

	if(mpq->backlight_index == 1) {
		//the BOOST_CYCLE_COUNT is here to help the preboost boot up faster to the target voltage
		#define BOOST_CYCLE_COUNT 20
		dev_err(dev, "mpq3367: starting preboost: cycle count = %d\n", BOOST_CYCLE_COUNT);
		for(int i=0; i < BOOST_CYCLE_COUNT; i++) {
			//these times are fairly tightly tuned 
			//to avoid going into some high current conditions
			gpiod_set_value(mpq->preboost_gpio, 0); usleep_range(200, 400);
			gpiod_set_value(mpq->preboost_gpio, 1); usleep_range(1200, 1300);
		}
		gpiod_set_value(mpq->preboost_gpio, 1);
	}
	dev_err(dev, "mpq3367: preboost startup done\n");
	//we need to delay to wait for hiccup mode to exit -- 
	//at a minimum it is about 70ms we need to wait here...
	usleep_range(200000, 250000);
	
	dev_err(dev, "mpq3367: gpio enable\n");
	gpiod_set_value(mpq->en_gpio, 1);

	/* default setting */
	dev_err(dev, "mpq3367: reg enable\n");
	do {
	 ret = regmap_write(mpq->regmap, 0x00, 0xa6);
	 
	 if(loopctr++ > 0) {
	  usleep_range(50000, 100000); 
	  dev_err(dev, "mpq3367: i2c write fail, retrying... [%d]\n", loopctr);
	 }

	 if(loopctr>20) {
		 dev_err(dev, "mpq3367: i2c write failed too many times, quitting\n");
		 break;
	 }
	} while(ret<0);

	mpq->is_enabled = true;

	// found empirically at 120Hz it takes 10 frames to boost backlight capacitors up to voltage
    // the backlight may brown out during this process, and need to be re-initialized
	// 10 frames @120Hz ~ 85 msec
	// 10 frames @80Hz  ~ 125 msec
    // backlight pulses are only output when panel starts receving frames, so we give ourselves a bit of a buffer and 
	// schedule the timer to come back in 1 sec
	backlight_check_work_init(mpq);

	return 0;
}

static int reg_disable(struct regulator_dev *rdev)
{
	struct mpq3367 *mpq = rdev_get_drvdata(rdev);

	printk("mpq3367: disabling backlight\n");
	gpiod_set_value(mpq->en_gpio, 0);

	if(mpq->backlight_index == 1) {
		printk("mpq3367: disabling preboost\n");
		gpiod_set_value(mpq->preboost_gpio, 0);
	}

	mpq->is_enabled = false;
	return 0;
}

static int reg_is_enabled(struct regulator_dev *rdev)
{
	struct mpq3367 *mpq = rdev_get_drvdata(rdev);

	return mpq->is_enabled;
}

static const struct regulator_ops reg_ops = {
	.enable = reg_enable,
	.disable = reg_disable,
	.is_enabled = reg_is_enabled,
};

static const struct regmap_config mpq3367_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x10, // XXX
};

/* -----------------------------------------------------------------------------
 * sysfs
 */
static ssize_t mpq_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	ssize_t len = 0;
	uint reg0, reg1, reg2, reg3;

	struct mpq3367 * mpq_device = dev_get_drvdata(dev);

	if (mpq_device == NULL)
	{
		reg0 = 0xFF;
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", reg0);
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", reg0);
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", reg0);
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", reg0);
		buf[len - 1] = '\n';
		dev_err(dev, "failed to retrieve mpq_device driver data\n");
		return len;
	}

	if (mpq_device->is_enabled)
	{
		printk("reading mpq_device registers\n");

		ret = regmap_read(mpq_device->regmap, 0x00, &reg0);
		ret = regmap_read(mpq_device->regmap, 0x01, &reg1);
		ret = regmap_read(mpq_device->regmap, 0x02, &reg2);
		ret = regmap_read(mpq_device->regmap, 0x03, &reg3);
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", reg0);
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", reg1);
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", reg2);
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", reg3);
		buf[len - 1] = '\n';
	}
	else
	{
		reg0 = 0xFF;
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", reg0);
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", reg0);
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", reg0);
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", reg0);
		buf[len - 1] = '\n';
		printk("mpq_device not enabled\n");
	}
    return len;
}

static ssize_t mpq_reinit_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	struct mpq3367 * mpq_device = dev_get_drvdata(dev);

	if (mpq_device == NULL)
	{
		printk("failed to retrieve mpq_device driver data\n");
		return 0;
	}

	if (mpq_device->is_enabled)
	{
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ", mpq_device->required_reinit);
		buf[len - 1] = '\n';
	}
	else
	{
		printk("mpq_device not enabled\n");
	}
    return len;
}

static ssize_t mpq_reinit_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int loopctr=0;
	int input;
	uint8_t reg_setting;

	struct mpq3367 * mpq_device = dev_get_drvdata(dev);

	if (mpq_device == NULL)
	{
		printk("failed to retrieve mpq_device driver data\n");
		return 0;
	}

	ret = sscanf(buf, "%d", &input);
	if(input == 1)
	{
		printk("setting mpq direct drive mode\n");
		reg_setting = 0xa6;
	}
	else if(input == 2)
	{
		printk("setting mpq default mode\n");
		reg_setting = 0x86;
	}
	else
	{
		printk("invalid mpq setting\n");
	}

	do
	{
		ret = regmap_write(mpq_device->regmap, 0x00, reg_setting);

		if(loopctr++ > 0) {
			usleep_range(50000, 100000);
			printk("mpq3367: i2c write fail, retrying... [%d]\n", loopctr);
		}

		if(loopctr>20) {
			printk("mpq3367: i2c write failed too many times, quitting\n");
			break;
		}
	}while(ret<0);
	printk("mpq_reinit complete\n");
	return count;
}

static DEVICE_ATTR_RO(mpq_status);
static DEVICE_ATTR_RW(mpq_reinit);

static struct attribute *mpq_attrs[] = {
    &dev_attr_mpq_status.attr,
	&dev_attr_mpq_reinit.attr,
	NULL,
};

static const struct attribute_group mpq_attr_group = {
	.name = "mpq",
    .attrs = mpq_attrs,
};

/* -----------------------------------------------------------------------------
 * sysfs
 */

static int
mpq3367_probe(struct i2c_client *client)
{
	struct regulator_config cfg = { };
	struct device *dev = &client->dev;
	struct regulator_dev *rdev;
	struct mpq3367 *mpq;
	int ret = 0;
	u32 val32;

	mpq = devm_kzalloc(dev, sizeof(*mpq), GFP_KERNEL);
	if (!mpq)
		return -ENOMEM;

	mpq->regmap = devm_regmap_init_i2c(client, &mpq3367_regmap_config);
	if (IS_ERR(mpq->regmap))
		return PTR_ERR(mpq->regmap);


	mpq->en_gpio = devm_gpiod_get(dev, "en", GPIOD_OUT_LOW);
	if (IS_ERR(mpq->en_gpio))
		return PTR_ERR(mpq->en_gpio);

	if (of_property_read_u32(dev->of_node, "backlight-index", &val32) >= 0)
		mpq->backlight_index = val32;

	if(mpq->backlight_index == 1) {
		mpq->preboost_gpio = devm_gpiod_get(dev, "preboost", GPIOD_OUT_LOW);
		if (IS_ERR(mpq->preboost_gpio))
			return PTR_ERR(mpq->preboost_gpio);
	}

	cfg.dev = dev;
	cfg.init_data = of_get_regulator_init_data(dev, dev->of_node, &mpq->desc);
	cfg.driver_data = mpq;
	cfg.of_node = dev->of_node;

	if (!cfg.init_data)
		return -EINVAL;

	mpq->desc.name = "mpq";
	mpq->desc.ops = &reg_ops;
	mpq->desc.type = REGULATOR_VOLTAGE;

	rdev = devm_regulator_register(dev, &mpq->desc, &cfg);
	if (IS_ERR(rdev)) {
		dev_err(dev, "Failed to register regulator: %d\n", (int) PTR_ERR(rdev));
		return PTR_ERR(rdev);
	}

    ret = sysfs_create_group(&dev->kobj, &mpq_attr_group);
    if (ret < 0) {
        dev_err(dev, "failed to create mpq sysfs files\n");
        return ret;
    }

	dev_set_drvdata(dev, mpq);
	return 0;
}

static const struct of_device_id mpq3367_dt_match[] = {
	{ .compatible = "mpq3367" },
	{ }
};

static struct i2c_driver mpq3367_driver = {
	.driver = {
		.name = "mpq3367",
		.of_match_table = mpq3367_dt_match,
	},
	.probe = mpq3367_probe,
};

module_i2c_driver(mpq3367_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("mpq3367 driver");
