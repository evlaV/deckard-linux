#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>

#include <linux/workqueue.h>

struct mpq3367_backlight {
	struct regulator_desc desc;
	struct i2c_adapter *adapter[2];
	bool is_enabled;

	struct gpio_desc *preboost_gpio;
	struct gpio_desc *en_gpio[2];
	struct gpio_desc *bias_gpio[2];

	uint8_t required_reinit;
	struct delayed_work backlight_work;
};

#define MPQ_I2C_ADDR 0x38

#define KTD_I2C_ADDR 0x3e
#define KTD_REGISTER_VPOS 0x00
#define KTD_REGISTER_VNEG 0x01

static int i2c_write_both(struct mpq3367_backlight *bl, u8 addr, u8 reg, u8 val)
{
	struct i2c_msg xfer;
	u8 buf[2] = { reg, val };
	int ret;

	xfer.addr = addr;
	xfer.flags = 0;
	xfer.len = 2;
	xfer.buf = buf;

	ret = i2c_transfer(bl->adapter[0], &xfer, 1);
	if (ret == 1) // success
		ret = i2c_transfer(bl->adapter[1], &xfer, 1);

	if (ret == 1)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int mpq_write(struct mpq3367_backlight *bl, u8 reg, u8 val)
{
	return i2c_write_both(bl, MPQ_I2C_ADDR, reg, val);
}

// read 4 bytes state from both mpq
static int mpq_read(struct mpq3367_backlight *bl, u8 *val0, u8 *val1)
{
	struct i2c_msg xfer[2];
	u8 reg = 0;
	int ret;

	xfer[0].addr = MPQ_I2C_ADDR;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &reg;

	xfer[1].addr = MPQ_I2C_ADDR;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 4;
	xfer[1].buf = val0;

	ret = i2c_transfer(bl->adapter[0], xfer, 2);
	if (ret == 2) { // success
		xfer[1].buf = val1;
		ret = i2c_transfer(bl->adapter[0], xfer, 2);
	}

	if (ret == 2)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static void backlight_delayed_work_handler(struct work_struct *work)
{
	// check to see if backlight is still in the correct mode
	uint8_t mpq3367_reg[2][4];
	int ret;
	int loopctr=0;

	struct mpq3367_backlight *bl = container_of((struct delayed_work *)work,struct mpq3367_backlight,backlight_work);

	printk("mpq3367: init check\n");
	ret = mpq_read(bl, mpq3367_reg[0], mpq3367_reg[1]);
	if (ret)
		printk("mpq3367: read failed (ret=%d)\n", ret);
	printk("mpq3367_reg[0][0]=%.2x\n", mpq3367_reg[0][0]);
	printk("mpq3367_reg[1][0]=%.2x\n", mpq3367_reg[1][0]);
	if(ret || mpq3367_reg[0][0] != 0xa6 || mpq3367_reg[1][0] != 0xa6)
	{
		printk("mpq3367: failed init check, resetting mode\n");
		do
		{
			ret = mpq_write(bl, 0x00, 0xa6);

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
		bl->required_reinit = 1;
	}
	else
	{
		printk("mpq3367: passed init check\n");
		bl->required_reinit = 2;
	}
}

static void backlight_check_work_init(struct mpq3367_backlight *bl)
{
	INIT_DELAYED_WORK(&bl->backlight_work, backlight_delayed_work_handler);
	schedule_delayed_work(&bl->backlight_work, HZ);
}

static int reg_enable(struct regulator_dev *rdev)
{
	struct mpq3367_backlight *bl = rdev_get_drvdata(rdev);
	struct device *dev = &rdev->dev;
	int ret;
	int loopctr=0;

	//the BOOST_CYCLE_COUNT is here to help the preboost boot up faster to the target voltage
	#define BOOST_CYCLE_COUNT 20
	dev_err(dev, "mpq3367: starting preboost: cycle count = %d\n", BOOST_CYCLE_COUNT);
	for(int i=0; i < BOOST_CYCLE_COUNT; i++) {
		//these times are fairly tightly tuned
		//to avoid going into some high current conditions
		gpiod_set_value(bl->preboost_gpio, 0); usleep_range(200, 400);
		gpiod_set_value(bl->preboost_gpio, 1); usleep_range(1200, 1300);
	}
	gpiod_set_value(bl->preboost_gpio, 1);

	dev_err(dev, "mpq3367: preboost startup done\n");
	//we need to delay to wait for hiccup mode to exit --
	//at a minimum it is about 70ms we need to wait here...
	usleep_range(200000, 250000);

	dev_err(dev, "mpq3367: gpio enable\n");
	gpiod_set_value(bl->en_gpio[0], 1);
	gpiod_set_value(bl->en_gpio[1], 1);

	/* default setting */
	dev_err(dev, "mpq3367: reg enable\n");
	do {
	 ret = mpq_write(bl, 0x00, 0xa6);

	 if(loopctr++ > 0) {
	  usleep_range(50000, 100000);
	  dev_err(dev, "mpq3367: i2c write fail, retrying... [%d]\n", loopctr);
	 }

	 if(loopctr>20) {
		 dev_err(dev, "mpq3367: i2c write failed too many times, quitting\n");
		 break;
	 }
	} while(ret<0);

	bl->is_enabled = true;

	// found empirically at 120Hz it takes 10 frames to boost backlight capacitors up to voltage
	// the backlight may brown out during this process, and need to be re-initialized
	// 10 frames @120Hz ~ 85 msec
	// 10 frames @80Hz  ~ 125 msec
	// backlight pulses are only output when panel starts receving frames, so we give ourselves a bit of a buffer and
	// schedule the timer to come back in 1 sec
	backlight_check_work_init(bl);

	return 0;
}

static int reg_disable(struct regulator_dev *rdev)
{
	struct mpq3367_backlight *bl = rdev_get_drvdata(rdev);

	printk("mpq3367: disabling backlight\n");
	gpiod_set_value(bl->en_gpio[0], 0);
	gpiod_set_value(bl->en_gpio[1], 1);

	printk("mpq3367: disabling preboost\n");
	gpiod_set_value(bl->preboost_gpio, 0);

	bl->is_enabled = false;
	return 0;
}

static int reg_is_enabled(struct regulator_dev *rdev)
{
	struct mpq3367_backlight *bl = rdev_get_drvdata(rdev);

	return bl->is_enabled;
}

static const struct regulator_ops reg_ops = {
	.enable = reg_enable,
	.disable = reg_disable,
	.is_enabled = reg_is_enabled,
};

/* -----------------------------------------------------------------------------
 * sysfs
 */
static ssize_t mpq_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mpq3367_backlight * bl = dev_get_drvdata(dev);
	u8 reg[2][4];
	int ret;


	ret = mpq_read(bl, reg[0], reg[1]);
	if (ret) {
		printk("mpq3367: read failed (ret=%d)\n", ret);
		memset(reg, 0xff, sizeof(reg));
	}

	return sprintf(buf,
		"R: %.2x %.2x %.2x %.2x\n"
		"L: %.2x %.2x %.2x %.2x\n",
		reg[0][0], reg[0][1], reg[0][2], reg[0][3],
		reg[1][0], reg[1][1], reg[1][2], reg[1][3]);
}

static ssize_t mpq_reinit_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mpq3367_backlight *bl = dev_get_drvdata(dev);
	return bl->is_enabled ? sprintf(buf, "%d\n", bl->required_reinit) : 0;
}

static ssize_t mpq_reinit_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mpq3367_backlight * bl = dev_get_drvdata(dev);
	int ret;
	int loopctr=0;
	int input;
	uint8_t reg_setting;



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
		ret = mpq_write(bl, 0x00, reg_setting);

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

static int
backlight_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config cfg = { };
	struct regulator_dev *rdev;
	struct mpq3367_backlight *bl;
	int ret = 0, i;

	bl = devm_kzalloc(dev, sizeof(*bl), GFP_KERNEL);
	if (!bl)
		return -ENOMEM;

	bl->preboost_gpio = devm_gpiod_get(dev, "preboost", GPIOD_OUT_LOW);
	if (IS_ERR(bl->preboost_gpio))
		return PTR_ERR(bl->preboost_gpio);

	for (i = 0; i < 2; i++) {
		bl->en_gpio[i] = devm_gpiod_get_index(dev, "en", i, GPIOD_OUT_LOW);
		if (IS_ERR(bl->en_gpio[i]))
			return PTR_ERR(bl->en_gpio[i]);

		// GPIOD_OUT_HIGH: set bias gpios to be always on
		bl->bias_gpio[i] = devm_gpiod_get_index(dev, "bias", i, GPIOD_OUT_HIGH);
		if (IS_ERR(bl->bias_gpio[i]))
			return PTR_ERR(bl->bias_gpio[i]);
	}

	for (i = 0; i < 2; i++) {
		struct device_node *node = of_parse_phandle(dev->of_node, "i2c-adapter", i);
		if (!node)
			return -ENOENT;

		bl->adapter[i] = i2c_get_adapter_by_fwnode(of_fwnode_handle(node));
		of_node_put(node);
		if (!bl->adapter[i])
			return -ENOENT;

	}
	// XXX: missing cleanup: i2c_put_adapter() on bl->adapter[i] on fail/remove

	cfg.dev = dev;
	cfg.init_data = of_get_regulator_init_data(dev, dev->of_node, &bl->desc);
	cfg.driver_data = bl;
	cfg.of_node = dev->of_node;

	if (!cfg.init_data)
		return -EINVAL;

	bl->desc.name = "mpq3367-backlight";
	bl->desc.ops = &reg_ops;
	bl->desc.type = REGULATOR_VOLTAGE;

	rdev = devm_regulator_register(dev, &bl->desc, &cfg);
	if (IS_ERR(rdev)) {
		dev_err(dev, "Failed to register regulator: %d\n", (int) PTR_ERR(rdev));
		return PTR_ERR(rdev);
	}

	ret = sysfs_create_group(&dev->kobj, &mpq_attr_group);
	if (ret < 0) {
		dev_err(dev, "failed to create mpq sysfs files\n");
		return ret;
	}

	ret = i2c_write_both(bl, KTD_I2C_ADDR, KTD_REGISTER_VPOS, 0x11); //+5.7V
	if (ret)
		dev_err(dev, "failed to write VPOS (ret=%d)\n", ret);
	ret = i2c_write_both(bl, KTD_I2C_ADDR, KTD_REGISTER_VNEG, 0x11); //-5.7V
	if (ret)
		dev_err(dev, "failed to write VNEG (ret=%d)\n", ret);

	platform_set_drvdata(pdev, bl);
	return 0;
}

static const struct of_device_id backlight_of_match[] = {
	{
		.compatible = "mpq3367-backlight",
	},
	{ },
};

static struct platform_driver mpq3367_backlight_driver = {
	.probe		= backlight_probe,
	.driver		= {
		.name		= "mpq3367-backlight",
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = of_match_ptr(backlight_of_match),
	},
};
module_platform_driver(mpq3367_backlight_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("mpq3367-backlight driver");
