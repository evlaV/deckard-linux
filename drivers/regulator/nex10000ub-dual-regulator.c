// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * nex10000ub  --  dual dual output LCD bias power supply
 */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/of_regulator.h>
#include <linux/delay.h>

#define NEX10000UB_MAX_VSEL             20
#define NEX10000UB_VOLTAGE_MIN          4000000
#define NEX10000UB_STEP_100mV           100000

#define NEX10000UB_I2C_ADDR             0x3e
#define NEX10000UB_REG_VOUT_POS			0x00
#define NEX10000UB_REG_VOUT_NEG			0x01
#define NEX10000UB_VOUT_MASK			0x1F
#define NEX10000UB_VENDOR_ID            0x02

#define NEX10000UB_ENABLE_TIME_USEC		2000

enum nex10000ub_regulator_id {
	NEX10000UB_LDO_POS_LEFT,
	NEX10000UB_LDO_NEG_LEFT,
	NEX10000UB_LDO_POS_RIGHT,
	NEX10000UB_LDO_NEG_RIGHT,
	NEX10000UB_NUM_REGULATORS,
};

struct nex10000ub_data {
	struct i2c_adapter *adapter[2];
	struct device *dev;

	unsigned int pos_enabled;
	unsigned int neg_enabled;

	unsigned int voltage_sel[NEX10000UB_NUM_REGULATORS];

	/* pos/neg enable gpio */
	struct gpio_desc *enp_gpio;
	struct gpio_desc *enn_gpio;

	struct regulator_init_data *reg_init_data[NEX10000UB_NUM_REGULATORS];
	struct regulator_dev *rdev[NEX10000UB_NUM_REGULATORS];
};

static int nex10000ub_i2c_write(struct nex10000ub_data *nex, u8 id, u8 reg, u8 val)
{
	struct i2c_msg xfer;
	u8 buf[2] = { reg, val };
	int ret;

	xfer.addr = NEX10000UB_I2C_ADDR;
	xfer.flags = 0;
	xfer.len = 2;
	xfer.buf = buf;

	if ((id == NEX10000UB_LDO_POS_LEFT) || (id == NEX10000UB_LDO_NEG_LEFT)) {
		ret = i2c_transfer(nex->adapter[0], &xfer, 1);
	} else {
		ret = i2c_transfer(nex->adapter[1], &xfer, 1);
	}
	if (ret != 1) {
		return -EIO;
	} else {
		return 0;
	}
}

static int nex10000ub_regulator_enable_time(struct regulator_dev *rdev) {
	return NEX10000UB_ENABLE_TIME_USEC;
}

static int nex10000ub_regulator_enable(struct regulator_dev *rdev) {
	int ret, i;
	struct nex10000ub_data *nex10000ub = rdev_get_drvdata(rdev);
	enum nex10000ub_regulator_id id = rdev_get_id(rdev);
	switch(id) {
		case NEX10000UB_LDO_POS_LEFT:
		case NEX10000UB_LDO_POS_RIGHT:
			gpiod_set_value(nex10000ub->enp_gpio, 1);
			msleep(1);
			for(i=0;i<10;i++) {
				ret = nex10000ub_i2c_write(nex10000ub, id, NEX10000UB_REG_VOUT_POS, nex10000ub->voltage_sel[id]);
				if(  ret == 0 )
					break;
			}			
			if(  ret  ) {
				dev_err(nex10000ub->dev, "Failed to set VPOS ( %d ) during enable\n", id);
			}
			nex10000ub->pos_enabled = 1;
			return nex10000ub->pos_enabled;
		case NEX10000UB_LDO_NEG_LEFT:
		case NEX10000UB_LDO_NEG_RIGHT:
			gpiod_set_value(nex10000ub->enn_gpio, 1);
			msleep(1);
			for(i=0;i<10;i++) {
				ret = nex10000ub_i2c_write(nex10000ub, id, NEX10000UB_REG_VOUT_NEG, nex10000ub->voltage_sel[id]);
				if(  ret == 0 )
					break;
			}	
			if(  ret  ) {
				dev_err(nex10000ub->dev, "Failed to set VNEG ( %d )during enable\n", id);
			}
			nex10000ub->neg_enabled = 1;
			return nex10000ub->neg_enabled;
		default:
			return 0;
			break;
	}

}

static int nex10000ub_regulator_disable(struct regulator_dev *rdev) {
	struct nex10000ub_data *nex10000ub = rdev_get_drvdata(rdev);
	enum nex10000ub_regulator_id id = rdev_get_id(rdev);
	switch(id) {
		case NEX10000UB_LDO_POS_LEFT:
		case NEX10000UB_LDO_POS_RIGHT:
			gpiod_set_value(nex10000ub->enp_gpio, 0);
			nex10000ub->pos_enabled = 0;
			return nex10000ub->pos_enabled;
			break;
		case NEX10000UB_LDO_NEG_LEFT:
		case NEX10000UB_LDO_NEG_RIGHT:
			gpiod_set_value(nex10000ub->enn_gpio, 0);
			nex10000ub->neg_enabled = 0;
			return nex10000ub->neg_enabled;
			break;
		default:
			return -EINVAL;
			break;
	}
}

static int nex10000ub_regulator_is_enabled(struct regulator_dev *rdev) {
	struct nex10000ub_data *nex10000ub = rdev_get_drvdata(rdev);
	enum nex10000ub_regulator_id id = rdev_get_id(rdev);
	switch(id) {
		case NEX10000UB_LDO_POS_LEFT:
		case NEX10000UB_LDO_POS_RIGHT:
			return nex10000ub->pos_enabled;
			break;
		case NEX10000UB_LDO_NEG_LEFT:
		case NEX10000UB_LDO_NEG_RIGHT:
			return nex10000ub->neg_enabled;
			break;
		default:
			return -EINVAL;
			break;
	}
}

static int nex10000ub_set_voltage_sel(struct regulator_dev *rdev, unsigned sel) {
	struct nex10000ub_data *nex10000ub = rdev_get_drvdata(rdev);
	enum nex10000ub_regulator_id id = rdev_get_id(rdev);
	// save selection for when enable is called
	if ( id < NEX10000UB_NUM_REGULATORS ) {
		nex10000ub->voltage_sel[id] = sel;
		return 0;
	} else {
		return -EINVAL;
	}
}

static int nex10000ub_get_voltage_sel(struct regulator_dev *rdev) {
	struct nex10000ub_data *nex10000ub = rdev_get_drvdata(rdev);
	enum nex10000ub_regulator_id id = rdev_get_id(rdev);
	if ( id < NEX10000UB_NUM_REGULATORS ) {
		return nex10000ub->voltage_sel[id];
	} else {
		return -EINVAL;
	}
}

static const struct regulator_ops nex10000ub_regulator_voltage_table_ops = {
	.list_voltage     = regulator_list_voltage_linear,
	.set_voltage_sel  = nex10000ub_set_voltage_sel,
	.get_voltage_sel  = nex10000ub_get_voltage_sel,
	.enable           = nex10000ub_regulator_enable,
	.disable          = nex10000ub_regulator_disable,
	.is_enabled       = nex10000ub_regulator_is_enabled,
	.enable_time      = nex10000ub_regulator_enable_time,
};

static const struct regulator_desc nex10000ub_regulator_desc[] = {
	{
		.name           = "ldo_vposr",
		.of_match       = "vpos_r",
		.id             = NEX10000UB_LDO_POS_RIGHT,
		.ops            = &nex10000ub_regulator_voltage_table_ops,
		.n_voltages     = NEX10000UB_MAX_VSEL + 1,
		.min_uV         = NEX10000UB_VOLTAGE_MIN,
		.uV_step        = NEX10000UB_STEP_100mV,
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
		.vsel_reg       = NEX10000UB_REG_VOUT_POS,
		.vsel_mask      = NEX10000UB_VOUT_MASK,
	},
	{
		.name           = "ldo_vnegr",
		.of_match       = "vneg_r",
		.id             = NEX10000UB_LDO_NEG_RIGHT,
		.ops            = &nex10000ub_regulator_voltage_table_ops,
		.n_voltages     = NEX10000UB_MAX_VSEL + 1,
		.min_uV         = NEX10000UB_VOLTAGE_MIN,
		.uV_step        = NEX10000UB_STEP_100mV,
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
		.vsel_reg       = NEX10000UB_REG_VOUT_NEG,
		.vsel_mask      = NEX10000UB_VOUT_MASK,
	},
	{
		.name           = "ldo_vposl",
		.of_match       = "vpos_l",
		.id             = NEX10000UB_LDO_POS_LEFT,
		.ops            = &nex10000ub_regulator_voltage_table_ops,
		.n_voltages     = NEX10000UB_MAX_VSEL + 1,
		.min_uV         = NEX10000UB_VOLTAGE_MIN,
		.uV_step        = NEX10000UB_STEP_100mV,
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
		.vsel_reg       = NEX10000UB_REG_VOUT_NEG,
		.vsel_mask      = NEX10000UB_VOUT_MASK,
	},
	{
		.name           = "ldo_vnegl",
		.of_match       = "vneg_l",
		.id             = NEX10000UB_LDO_NEG_LEFT,
		.ops            = &nex10000ub_regulator_voltage_table_ops,
		.n_voltages     = NEX10000UB_MAX_VSEL + 1,
		.min_uV         = NEX10000UB_VOLTAGE_MIN,
		.uV_step        = NEX10000UB_STEP_100mV,
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
		.vsel_reg       = NEX10000UB_REG_VOUT_NEG,
		.vsel_mask      = NEX10000UB_VOUT_MASK,
	},
};

static struct nex10000ub_data *nex10000ub_parse_regulator_dt(
		struct device *dev,
		struct of_regulator_match *nex10000ub_matches, unsigned int num)
{
	struct device_node *np = dev->of_node;
	struct device_node *regs;
	unsigned int i;
	struct nex10000ub_data *nex10000ub;
	int err;

	regs = of_get_child_by_name(np, "regulators");
	if (!regs) {
		dev_err(dev, "regulator node not found\n");
		return NULL;
	}

	err = of_regulator_match(dev, regs, nex10000ub_matches, num);
	if (err < 0) {
		dev_err(dev, "Regulator match failed, e %d\n", err);
		return NULL;
	}

	nex10000ub = devm_kzalloc(dev, sizeof(*nex10000ub), GFP_KERNEL);
	if (!nex10000ub)
		return NULL;

	nex10000ub->dev = dev;

	for (i = 0; i < num; i++) {
		if (!nex10000ub_matches[i].init_data) {
			dev_err(dev, "init data not valid!!\n");
			continue;
		}
		nex10000ub->reg_init_data[i] = nex10000ub_matches[i].init_data;
	}
	return nex10000ub;
}

static int nex10000ub_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nex10000ub_data *nex10000ub;
	struct regulator_config config = { };
	unsigned int id, i;

	static struct of_regulator_match nex10000ub_matches[] = {
		{ .name = "vposr",     .driver_data = (void *)NEX10000UB_LDO_POS_RIGHT},
		{ .name = "vnegr",     .driver_data = (void *)NEX10000UB_LDO_NEG_RIGHT},
		{ .name = "vposl",     .driver_data = (void *)NEX10000UB_LDO_POS_LEFT},
		{ .name = "vnegl",     .driver_data = (void *)NEX10000UB_LDO_NEG_LEFT},
	};

	nex10000ub = nex10000ub_parse_regulator_dt(dev, nex10000ub_matches, ARRAY_SIZE(nex10000ub_matches));
	if ( !nex10000ub ) {
		return -EINVAL;
	}

	for (i = 0; i < 2; i++) {
		struct device_node *node = of_parse_phandle(dev->of_node, "i2c-adapter", i);
		if (!node)
				return -ENOENT;

		nex10000ub->adapter[i] = i2c_get_adapter_by_fwnode(of_fwnode_handle(node));
		of_node_put(node);
		if (!nex10000ub->adapter[i])
				return -EPROBE_DEFER;
	}

	nex10000ub->enp_gpio = devm_gpiod_get_optional(dev, "enp", GPIOD_OUT_LOW);
	if (IS_ERR(nex10000ub->enp_gpio)) {
		dev_err(dev, "cannot get enp gpio %ld\n",
			PTR_ERR(nex10000ub->enp_gpio));
	}

	nex10000ub->enn_gpio = devm_gpiod_get_optional(dev, "enn", GPIOD_OUT_LOW);
	if (IS_ERR(nex10000ub->enn_gpio)) {
		dev_err(dev, "cannot get enn gpio %ld\n",
			PTR_ERR(nex10000ub->enn_gpio));
	}

	for (id = 0; id < NEX10000UB_NUM_REGULATORS; ++id) {
		config.dev = dev;
		config.init_data = nex10000ub->reg_init_data[id];
		config.driver_data = nex10000ub;
		config.of_node = nex10000ub_matches[id].of_node;

		nex10000ub->rdev[id] = devm_regulator_register(dev, &nex10000ub_regulator_desc[id], &config);
		if (IS_ERR(nex10000ub->rdev[id])) {
			dev_err(dev, "failed to register regulator %s\n",
					nex10000ub_regulator_desc[id].name);
			return PTR_ERR(nex10000ub->rdev[id]);
		}
	}
	platform_set_drvdata(pdev, nex10000ub);

	return 0;
}

static const struct of_device_id nex10000ub_of_match[] = {
	{ .compatible = "nexperia,dual-nex10000ub", },
	{},
};
MODULE_DEVICE_TABLE(of, nex10000ub_of_match);

static struct platform_driver nex10000ub_driver = {
	.probe = nex10000ub_probe,
	.driver		= {
		.name	= "nex10000ub",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = of_match_ptr(nex10000ub_of_match),
	},
};
module_platform_driver(nex10000ub_driver);

/* Module information */
MODULE_DESCRIPTION("NEXPERIA nex10000ub dual dual output LCD bias power supply driver with combined enable pins");
MODULE_AUTHOR("Valve");
MODULE_LICENSE("GPL v2");
