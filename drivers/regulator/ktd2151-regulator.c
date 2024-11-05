// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ktd2151  --  dual dual output LCD bias power supply
 */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/regulator/of_regulator.h>
#include <linux/delay.h>

#define ktd2151_MAX_VSEL             23
#define ktd2151_VOLTAGE_MIN          4000000
#define ktd2151_STEP_100mV           100000

#define ktd2151_I2C_ADDR                0x3e
#define ktd2151_REG_VOUT_POS			0x00
#define ktd2151_REG_VOUT_NEG			0x01
#define ktd2151_VOUT_MASK			    0x1F

#define ktd2151_ENABLE_TIME_USEC		2000

enum ktd2151_regulator_id {
	ktd2151_LDO_POS_LEFT,
	ktd2151_LDO_NEG_LEFT,
	ktd2151_LDO_POS_RIGHT,
	ktd2151_LDO_NEG_RIGHT,
	ktd2151_NUM_REGULATORS,
};

struct ktd2151_data {
	struct i2c_adapter *adapter[2];
	struct device *dev;

	unsigned int pos_enabled;
	unsigned int neg_enabled;

	unsigned int pos_sel;
	unsigned int neg_sel;

	/* pos/neg enable gpio */
	struct gpio_desc *enp_gpio;
	struct gpio_desc *enn_gpio;

	struct regulator_init_data *reg_init_data[ktd2151_NUM_REGULATORS];
	struct regulator_dev *rdev[ktd2151_NUM_REGULATORS];
};

static int ktd2151_i2c_write(struct ktd2151_data *ktd, u8 id, u8 reg, u8 val)
{
	struct i2c_msg xfer;
	u8 buf[2] = { reg, val };
	int ret;

	xfer.addr = ktd2151_I2C_ADDR;
	xfer.flags = 0;
	xfer.len = 2;
	xfer.buf = buf;

	if ((id == ktd2151_LDO_POS_LEFT) || (id == ktd2151_LDO_NEG_LEFT)) {
		ret = i2c_transfer(ktd->adapter[0], &xfer, 1);
	} else {
		ret = i2c_transfer(ktd->adapter[1], &xfer, 1);
	}
	if (ret != 1) {
		return -EIO;
	} else {
		return 0;
	}
}

static int ktd2151_regulator_enable_time(struct regulator_dev *rdev) {
	return ktd2151_ENABLE_TIME_USEC;
}

static int ktd2151_regulator_enable(struct regulator_dev *rdev) {
	struct ktd2151_data *ktd2151 = rdev_get_drvdata(rdev);
	enum ktd2151_regulator_id id = rdev_get_id(rdev);
	switch(id) {
		case ktd2151_LDO_POS_LEFT:
		case ktd2151_LDO_POS_RIGHT:
			if ( ktd2151->enp_gpio ) {
				gpiod_set_value(ktd2151->enp_gpio, 1);
			}
			ktd2151->pos_enabled = 1;
			return ktd2151->pos_enabled;
		case ktd2151_LDO_NEG_LEFT:
		case ktd2151_LDO_NEG_RIGHT:
			if ( ktd2151->enn_gpio ) {
				gpiod_set_value(ktd2151->enn_gpio, 1);
			}
			ktd2151->neg_enabled = 1;
			return ktd2151->neg_enabled;
		default:
			return 0;
			break;
	}

}

static int ktd2151_regulator_disable(struct regulator_dev *rdev) {
	struct ktd2151_data *ktd2151 = rdev_get_drvdata(rdev);
	enum ktd2151_regulator_id id = rdev_get_id(rdev);
	switch(id) {
		case ktd2151_LDO_POS_LEFT:
		case ktd2151_LDO_POS_RIGHT:
			if ( ktd2151->enp_gpio ) {
				gpiod_set_value(ktd2151->enp_gpio, 0);
			}
			ktd2151->pos_enabled = 0;
			return ktd2151->pos_enabled;
			break;
		case ktd2151_LDO_NEG_LEFT:
		case ktd2151_LDO_NEG_RIGHT:
			if ( ktd2151->enn_gpio ) {
				gpiod_set_value(ktd2151->enn_gpio, 0);
			}
			ktd2151->neg_enabled = 0;
			return ktd2151->neg_enabled;
			break;
		default:
			return -EINVAL;
			break;
	}
}

static int ktd2151_regulator_is_enabled(struct regulator_dev *rdev) {
	struct ktd2151_data *ktd2151 = rdev_get_drvdata(rdev);
	enum ktd2151_regulator_id id = rdev_get_id(rdev);
	switch(id) {
		case ktd2151_LDO_POS_LEFT:
		case ktd2151_LDO_POS_RIGHT:
			return ktd2151->pos_enabled;
			break;
		case ktd2151_LDO_NEG_LEFT:
		case ktd2151_LDO_NEG_RIGHT:
			return ktd2151->neg_enabled;
			break;
		default:
			return -EINVAL;
			break;
	}
}

// XXX KTD fails to communicate on i2c at runtime, so we simply save the requested voltage here
// Actual voltage is set at start in probe()
static int ktd2151_set_voltage_sel(struct regulator_dev *rdev, unsigned sel) {
	int ret = 0;
	struct ktd2151_data *ktd2151 = rdev_get_drvdata(rdev);
	enum ktd2151_regulator_id id = rdev_get_id(rdev);

	switch(id) {
		case ktd2151_LDO_POS_LEFT:
		case ktd2151_LDO_POS_RIGHT:
			ktd2151->pos_sel = sel;
			break;
		case ktd2151_LDO_NEG_LEFT:
		case ktd2151_LDO_NEG_RIGHT:
			ktd2151->neg_sel = sel;                       
			break;
		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}

static int ktd2151_get_voltage_sel(struct regulator_dev *rdev) {
	struct ktd2151_data *ktd2151 = rdev_get_drvdata(rdev);
	enum ktd2151_regulator_id id = rdev_get_id(rdev);	
	// save selection for when enable is called
	switch(id) {
		case ktd2151_LDO_POS_LEFT:
		case ktd2151_LDO_POS_RIGHT:
			return ktd2151->pos_sel;
		case ktd2151_LDO_NEG_LEFT:
		case ktd2151_LDO_NEG_RIGHT:
			return ktd2151->neg_sel;                      
		default:
			return -EINVAL;
	}
}

static const struct regulator_ops ktd2151_regulator_voltage_table_ops = {
	.list_voltage     = regulator_list_voltage_linear,
	.set_voltage_sel  = ktd2151_set_voltage_sel,
	.get_voltage_sel  = ktd2151_get_voltage_sel,
	.enable           = ktd2151_regulator_enable,
	.disable          = ktd2151_regulator_disable,
	.is_enabled       = ktd2151_regulator_is_enabled,
	.enable_time      = ktd2151_regulator_enable_time,
};

static const struct regulator_desc ktd2151_regulator_desc[] = {
	{
		.name           = "ldo_vposr",
		.of_match       = "vpos_r",
		.id             = ktd2151_LDO_POS_RIGHT,
		.ops            = &ktd2151_regulator_voltage_table_ops,
		.n_voltages     = ktd2151_MAX_VSEL + 1,
		.min_uV         = ktd2151_VOLTAGE_MIN,
		.uV_step        = ktd2151_STEP_100mV,
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
		.vsel_reg       = ktd2151_REG_VOUT_POS,
		.vsel_mask      = ktd2151_VOUT_MASK,
	},
	{
		.name           = "ldo_vnegr",
		.of_match       = "vneg_r",
		.id             = ktd2151_LDO_NEG_RIGHT,
		.ops            = &ktd2151_regulator_voltage_table_ops,
		.n_voltages     = ktd2151_MAX_VSEL + 1,
		.min_uV         = ktd2151_VOLTAGE_MIN,
		.uV_step        = ktd2151_STEP_100mV,
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
		.vsel_reg       = ktd2151_REG_VOUT_NEG,
		.vsel_mask      = ktd2151_VOUT_MASK,
	},
	{
		.name           = "ldo_vposl",
		.of_match       = "vpos_l",
		.id             = ktd2151_LDO_POS_LEFT,
		.ops            = &ktd2151_regulator_voltage_table_ops,
		.n_voltages     = ktd2151_MAX_VSEL + 1,
		.min_uV         = ktd2151_VOLTAGE_MIN,
		.uV_step        = ktd2151_STEP_100mV,
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
		.vsel_reg       = ktd2151_REG_VOUT_POS,
		.vsel_mask      = ktd2151_VOUT_MASK,
	},
	{
		.name           = "ldo_vnegl",
		.of_match       = "vneg_l",
		.id             = ktd2151_LDO_NEG_LEFT,
		.ops            = &ktd2151_regulator_voltage_table_ops,
		.n_voltages     = ktd2151_MAX_VSEL + 1,
		.min_uV         = ktd2151_VOLTAGE_MIN,
		.uV_step        = ktd2151_STEP_100mV,
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
		.vsel_reg       = ktd2151_REG_VOUT_NEG,
		.vsel_mask      = ktd2151_VOUT_MASK,
	},
};

static struct ktd2151_data *ktd2151_parse_regulator_dt(
		struct device *dev,
		struct of_regulator_match *ktd2151_matches, unsigned int num)
{
	struct device_node *np = dev->of_node;
	struct device_node *regs;
	unsigned int i;
	struct ktd2151_data *ktd2151;
	int err;

	regs = of_get_child_by_name(np, "regulators");
	if (!regs) {
		dev_err(dev, "regulator node not found\n");
		return NULL;
	}

	err = of_regulator_match(dev, regs, ktd2151_matches, num);
	if (err < 0) {
		dev_err(dev, "Regulator match failed ( %d )\n", err);
		return NULL;
	}

	ktd2151 = devm_kzalloc(dev, sizeof(*ktd2151), GFP_KERNEL);
	if (!ktd2151)
		return NULL;

	ktd2151->dev = dev;

	for (i = 0; i < num; i++) {
		if (!ktd2151_matches[i].init_data) {
			dev_err(dev, "init data not valid!!\n");
			continue;
		}
		ktd2151->reg_init_data[i] = ktd2151_matches[i].init_data;
	}
	return ktd2151;
}

static int ktd2151_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ktd2151_data *ktd2151;
	struct regulator_config config = { };
	unsigned int id;
	int ret;
	unsigned int min_uV_pos, min_uV_neg;
	int i;

	struct of_regulator_match ktd2151_matches[] = {
		{ .name = "vposr",     .driver_data = (void *)ktd2151_LDO_POS_RIGHT},
		{ .name = "vnegr",     .driver_data = (void *)ktd2151_LDO_NEG_RIGHT},
		{ .name = "vposl",     .driver_data = (void *)ktd2151_LDO_POS_LEFT},
		{ .name = "vnegl",     .driver_data = (void *)ktd2151_LDO_NEG_LEFT},
	};

	ktd2151 = ktd2151_parse_regulator_dt(dev, ktd2151_matches, ARRAY_SIZE(ktd2151_matches));
	if ( !ktd2151 ) {
		return -EINVAL;
	}

	for (i = 0; i < 2; i++) {
		struct device_node *node = of_parse_phandle(dev->of_node, "i2c-adapter", i);
		if (!node)
				return -ENOENT;

		ktd2151->adapter[i] = i2c_get_adapter_by_fwnode(of_fwnode_handle(node));
		of_node_put(node);
		if (!ktd2151->adapter[i])
				return -ENOENT;
	}

	ktd2151->enp_gpio = devm_gpiod_get(dev, "enp", GPIOD_OUT_LOW);
	if (IS_ERR(ktd2151->enp_gpio)) {
		dev_err(dev, "cannot get enp gpio %ld\n",
			PTR_ERR(ktd2151->enp_gpio));
	}

	ktd2151->enn_gpio = devm_gpiod_get(dev, "enn", GPIOD_OUT_LOW);
	if (IS_ERR(ktd2151->enn_gpio)) {
		dev_err(dev, "cannot get enn gpio %ld\n",
			PTR_ERR(ktd2151->enn_gpio));
	}

	// XXX KTD regulator does not communicate on i2c once the system is up and running
	// We do not know why
	// Instead, we enable the KTD here briefly so that we can set the default value, and then
	// turn the regulator back off, to be enabled later with the correct voltage
	min_uV_pos = ktd2151_matches[ktd2151_LDO_POS_LEFT].init_data->constraints.min_uV;
	min_uV_neg = ktd2151_matches[ktd2151_LDO_NEG_LEFT].init_data->constraints.min_uV;

	gpiod_set_value(ktd2151->enp_gpio, 1);
	gpiod_set_value(ktd2151->enn_gpio, 1);

	usleep_range(1000,2000);

	for ( i = 0 ; i < 10; i++ ){
		ret = ktd2151_i2c_write(ktd2151, ktd2151_LDO_POS_RIGHT, ktd2151_REG_VOUT_POS, ((min_uV_pos - ktd2151_VOLTAGE_MIN)/ktd2151_STEP_100mV));
		ret |= ktd2151_i2c_write(ktd2151, ktd2151_LDO_NEG_RIGHT, ktd2151_REG_VOUT_NEG, ((min_uV_pos - ktd2151_VOLTAGE_MIN)/ktd2151_STEP_100mV));
		if (ret == 0)
			break;
	}
	if ( ret ) {
		dev_err(dev, "failed to write right voltages, will be default ( %d )\n", ret);
	}

	min_uV_pos = ktd2151_matches[ktd2151_LDO_POS_LEFT].init_data->constraints.min_uV;
	min_uV_neg = ktd2151_matches[ktd2151_LDO_NEG_LEFT].init_data->constraints.min_uV;

	for ( i = 0 ; i < 10; i++ ){
		ret = ktd2151_i2c_write(ktd2151, ktd2151_LDO_POS_LEFT, ktd2151_REG_VOUT_POS, ((min_uV_pos - ktd2151_VOLTAGE_MIN)/ktd2151_STEP_100mV));
		ret |= ktd2151_i2c_write(ktd2151, ktd2151_LDO_NEG_LEFT, ktd2151_REG_VOUT_NEG, ((min_uV_pos - ktd2151_VOLTAGE_MIN)/ktd2151_STEP_100mV));
		if (ret == 0)
			break;
	}
	if ( ret ) {
		dev_err(dev, "failed to write left voltages, will be default ( %d )\n", ret);
	}

	gpiod_set_value(ktd2151->enp_gpio, 0);
	gpiod_set_value(ktd2151->enn_gpio, 0);

	for (id = 0; id < ktd2151_NUM_REGULATORS; ++id) {
		config.dev = dev;
		config.init_data = ktd2151->reg_init_data[id];
		config.driver_data = ktd2151;
		config.of_node = ktd2151_matches[id].of_node;

		ktd2151->rdev[id] = devm_regulator_register(dev, &ktd2151_regulator_desc[id], &config);
		if (IS_ERR(ktd2151->rdev[id])) {
			dev_err(dev, "failed to register regulator %s\n",
					ktd2151_regulator_desc[id].name);
			return PTR_ERR(ktd2151->rdev[id]);
		}
	}
	platform_set_drvdata(pdev, ktd2151);

	return 0;
}

static const struct of_device_id ktd2151_of_match[] = {
	{ .compatible = "kinetic,ktd2151", },
	{},
};
MODULE_DEVICE_TABLE(of, ktd2151_of_match);

static struct platform_driver ktd2151_driver = {
	.probe = ktd2151_probe,
	.driver		= {
		.name	= "ktd2151",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = of_match_ptr(ktd2151_of_match),
	},
};
module_platform_driver(ktd2151_driver);

/* Module information */
MODULE_DESCRIPTION("Kinetic ktd2151 dual dual output LCD bias power supply driver");
MODULE_AUTHOR("Valve");
MODULE_LICENSE("GPL v2");
