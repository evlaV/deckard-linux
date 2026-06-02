// SPDX-License-Identifier: GPL-2.0
/*
 * MP2652 charger driver
 *
 * Monolithic Power Systems MP2652
 * 2S-5S buck/boost charger
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/power_supply.h>
#include <linux/of_device.h>
#include <linux/bitfield.h>
#include <linux/property.h>
#include <linux/delay.h>

/* Registers */
#define REG_IINVMIN     0x06
#define REG_IINLIM      0x08
#define REG_JEITA       0x0C
#define REG_TEMP        0x0D
#define REG_CONFIG0     0x0E
#define REG_PRECHARGE   0x0F
#define REG_CONFIG2     0x10
#define REG_CONTROL     0x12
#define REG_ICHG        0x14
#define REG_VREG        0x15
#define REG_STATUS0     0x16
#define REG_STATUS1     0x17

/* REG_IINVMIN (0x06) */
#define VINMIN_MASK     GENMASK(7, 0)    /* 80mV/bit */

/* REG_IINLIM (0x08) */
#define IINLIM_MASK     GENMASK(6, 0)    /* 50mA/bit */

/* REG_JEITA (0x0C) */
#define WARM_ACT        GENMASK(14, 13)  /* 0b10 = only reduce ICC */
#define JEITA_ISET      GENMASK(5, 4)    /* 0b00 = 1/2 * ICHG */

/* REG_TEMP (0x0D) */
#define NTC_WARM_THRESH GENMASK(5, 4)    /* 0b10 = 50°C */

/* REG_CONFIG0 (0x0E) */
#define ADC_CONV        BIT(7)           /* 0=disable, 1=continuous */

/* REG_PRECHARGE (0x0F) */
#define IPRE_MASK       GENMASK(7, 4)    /* 100mA/bit */
#define ITERM_MASK      GENMASK(3, 0)    /* 50mA/bit */

/* REG_CONFIG2 (0x10) */
#define BAT_NUM_MASK    GENMASK(10, 9)   /* 00=5cell, 01=2cell, 10=3cell, 11=4cell */

/* REG_CONTROL (0x12) */
#define CHG_EN          BIT(0)

/* REG_ICHG (0x14) */
#define ICHG_MASK       GENMASK(13, 6)   /* 50mA/bit */

/* REG_VREG (0x15) */
#define VREG_MASK       GENMASK(14, 4)   /* 10mV/bit; for 2-4 cell = total voltage,
					  * for 5-cell = per-cell voltage */

/* REG_STATUS0 (0x16) */
#define CHG_STAT        GENMASK(8, 6)    /* 000=none 001=trickle 010=pre 011=CC 100=CV 101=term */

/* REG_STATUS1 (0x17) */
#define NTC_STAT        GENMASK(2, 0)    /* 000=normal 001=cold 010=cool 011=warm 100=hot 111=float */

struct mp2652 {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct power_supply *psy;
	int num_cells;
	u32 max_ichg_ua;
	u32 max_vreg_uv;
};

static const struct regmap_config mp2652_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0x2D,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int mp2652_enable_charging(struct mp2652 *mp, bool enable)
{
	return regmap_update_bits(mp->regmap, REG_CONTROL, CHG_EN,
				  enable ? CHG_EN : 0);
}

static int mp2652_set_ichg(struct mp2652 *mp, int ua)
{
	u16 code = ua / 50000;

	if (code > 0xff)
		code = 0xff;

	return regmap_update_bits(mp->regmap, REG_ICHG, ICHG_MASK,
				  FIELD_PREP(ICHG_MASK, code));
}

static int mp2652_set_vreg(struct mp2652 *mp, int uv)
{
	u16 code;

	if (mp->num_cells == 5)
		code = (uv / mp->num_cells) / 10000;
	else
		code = uv / 10000;

	return regmap_update_bits(mp->regmap, REG_VREG, VREG_MASK,
				  FIELD_PREP(VREG_MASK, code));
}

static int mp2652_set_iinlim(struct mp2652 *mp, int ua)
{
	u16 code = ua / 50000;

	if (code > 0x7f)
		code = 0x7f;

	return regmap_update_bits(mp->regmap, REG_IINLIM, IINLIM_MASK,
				  FIELD_PREP(IINLIM_MASK, code));
}

static int mp2652_set_vinmin(struct mp2652 *mp, int uv)
{
	u16 code = uv / 80000;

	if (code > 0xff)
		code = 0xff;

	return regmap_update_bits(mp->regmap, REG_IINVMIN, VINMIN_MASK,
				  FIELD_PREP(VINMIN_MASK, code));
}

static int mp2652_set_ipre(struct mp2652 *mp, int ua)
{
	u16 code = ua / 100000;

	if (code > 0xf)
		code = 0xf;

	return regmap_update_bits(mp->regmap, REG_PRECHARGE, IPRE_MASK,
				  FIELD_PREP(IPRE_MASK, code));
}

static int mp2652_set_iterm(struct mp2652 *mp, int ua)
{
	u16 code = ua / 50000;

	if (code > 0xf)
		code = 0xf;

	return regmap_update_bits(mp->regmap, REG_PRECHARGE, ITERM_MASK,
				  FIELD_PREP(ITERM_MASK, code));
}

static enum power_supply_property mp2652_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
};

static int mp2652_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct mp2652 *mp = power_supply_get_drvdata(psy);
	unsigned int reg;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = regmap_read(mp->regmap, REG_STATUS0, &reg);
		if (ret)
			return ret;
		switch (FIELD_GET(CHG_STAT, reg)) {
		case 0:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case 5:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		}
		return 0;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = regmap_read(mp->regmap, REG_STATUS0, &reg);
		if (ret)
			return ret;
		switch (FIELD_GET(CHG_STAT, reg)) {
		case 0:
		case 5:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		case 1:
		case 2:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		}
		return 0;

	case POWER_SUPPLY_PROP_HEALTH:
		ret = regmap_read(mp->regmap, REG_STATUS1, &reg);
		if (ret)
			return ret;
		switch (FIELD_GET(NTC_STAT, reg)) {
		case 0:
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case 1:
			val->intval = POWER_SUPPLY_HEALTH_COLD;
			break;
		case 2:
			val->intval = POWER_SUPPLY_HEALTH_COOL;
			break;
		case 3:
			val->intval = POWER_SUPPLY_HEALTH_WARM;
			break;
		case 4:
			val->intval = POWER_SUPPLY_HEALTH_HOT;
			break;
		default:
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
			break;
		}
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = regmap_read(mp->regmap, REG_ICHG, &reg);
		if (ret)
			return ret;
		val->intval = FIELD_GET(ICHG_MASK, reg) * 50000;
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = mp->max_ichg_ua;
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = regmap_read(mp->regmap, REG_VREG, &reg);
		if (ret)
			return ret;
		if (mp->num_cells == 5)
			val->intval = FIELD_GET(VREG_MASK, reg) * 10000 * mp->num_cells;
		else
			val->intval = FIELD_GET(VREG_MASK, reg) * 10000;
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = mp->max_vreg_uv;
		return 0;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = regmap_read(mp->regmap, REG_IINLIM, &reg);
		if (ret)
			return ret;
		val->intval = FIELD_GET(IINLIM_MASK, reg) * 50000;
		return 0;

	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = regmap_read(mp->regmap, REG_IINVMIN, &reg);
		if (ret)
			return ret;
		val->intval = FIELD_GET(VINMIN_MASK, reg) * 80000;
		return 0;

	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = regmap_read(mp->regmap, REG_PRECHARGE, &reg);
		if (ret)
			return ret;
		val->intval = FIELD_GET(IPRE_MASK, reg) * 100000;
		return 0;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = regmap_read(mp->regmap, REG_PRECHARGE, &reg);
		if (ret)
			return ret;
		val->intval = FIELD_GET(ITERM_MASK, reg) * 50000;
		return 0;

	default:
		return -EINVAL;
	}
}

static int mp2652_set_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       const union power_supply_propval *val)
{
	struct mp2652 *mp = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return mp2652_enable_charging(mp,
			val->intval == POWER_SUPPLY_STATUS_CHARGING);

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return mp2652_set_ichg(mp, val->intval);

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		return mp2652_set_vreg(mp, val->intval);

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return mp2652_set_iinlim(mp, val->intval);

	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		return mp2652_set_vinmin(mp, val->intval);

	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		return mp2652_set_ipre(mp, val->intval);

	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return mp2652_set_iterm(mp, val->intval);

	default:
		return -EINVAL;
	}
}

static int mp2652_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return 1;
	default:
		return 0;
	}
}

static const struct power_supply_desc mp2652_psy_desc = {
	.name = "mp2652-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = mp2652_props,
	.num_properties = ARRAY_SIZE(mp2652_props),
	.get_property = mp2652_get_property,
	.set_property = mp2652_set_property,
	.property_is_writeable = mp2652_property_is_writeable,
};

static int mp2652_read_num_cells(struct mp2652 *mp)
{
	static const int cell_table[] = { 5, 2, 3, 4 };
	unsigned int reg;
	int ret;

	ret = regmap_read(mp->regmap, REG_CONFIG2, &reg);
	if (ret)
		return ret;

	mp->num_cells = cell_table[FIELD_GET(BAT_NUM_MASK, reg)];
	return 0;
}

static int mp2652_hw_init(struct mp2652 *mp)
{
	int ret;

	ret = mp2652_enable_charging(mp, false);
	if (ret)
		return ret;

	/* VREG = 8900mV (10mV/bit, bits 14:4) */
	ret = regmap_update_bits(mp->regmap, REG_VREG, VREG_MASK,
				 FIELD_PREP(VREG_MASK, 890));
	if (ret)
		return ret;

	/* NTC Warm Threshold = 50°C (bits 5:4 = 0b10) */
	ret = regmap_update_bits(mp->regmap, REG_TEMP, NTC_WARM_THRESH,
				 FIELD_PREP(NTC_WARM_THRESH, 2));
	if (ret)
		return ret;

	/* WARM_ACT = only reduce ICC (bits 14:13 = 0b10) */
	ret = regmap_update_bits(mp->regmap, REG_JEITA, WARM_ACT,
				 FIELD_PREP(WARM_ACT, 2));
	if (ret)
		return ret;

	/* JEITA_ISET = 1/2 * ICHG (bits 5:4 = 0b00) */
	ret = regmap_update_bits(mp->regmap, REG_JEITA, JEITA_ISET, 0);
	if (ret)
		return ret;

	/* Disable ADC (bit 7 = 0; preserve reserved bit 3) */
	ret = regmap_update_bits(mp->regmap, REG_CONFIG0, ADC_CONV, 0);
	if (ret)
		return ret;

	return 0;
}

static int mp2652_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct mp2652 *mp;
	int ret;

	mp = devm_kzalloc(&client->dev, sizeof(*mp), GFP_KERNEL);
	if (!mp)
		return -ENOMEM;

	mp->dev = &client->dev;
	mp->client = client;

	mp->regmap = devm_regmap_init_i2c(client, &mp2652_regmap_config);
	if (IS_ERR(mp->regmap))
		return PTR_ERR(mp->regmap);

	i2c_set_clientdata(client, mp);

	/* Give the MP2652 time to power up - seen 2 ms in the field, undocumented time */
	msleep(5);

	ret = mp2652_read_num_cells(mp);
	if (ret) {
		dev_err(&client->dev, "mp2652: failed to read num_cells: %d\n", ret);
		return ret;
	}

	device_property_read_u32(&client->dev,
				 "constant-charge-current-max-microamp",
				 &mp->max_ichg_ua);
	device_property_read_u32(&client->dev,
				 "constant-charge-voltage-max-microvolt",
				 &mp->max_vreg_uv);

	ret = mp2652_hw_init(mp);
	if (ret) {
		dev_err(&client->dev, "mp2652: hw_init failed: %d\n", ret);
		return ret;
	}

	psy_cfg.drv_data = mp;

	mp->psy = devm_power_supply_register(mp->dev, &mp2652_psy_desc, &psy_cfg);
	if (IS_ERR(mp->psy))
		return PTR_ERR(mp->psy);

	dev_info(mp->dev, "MP2652 charger driver loaded\n");

	return 0;
}

static void mp2652_remove(struct i2c_client *client)
{
	struct mp2652 *mp = i2c_get_clientdata(client);

	mp2652_enable_charging(mp, false);
}

static const struct of_device_id mp2652_of_match[] = {
	{ .compatible = "mps,mp2652" },
	{}
};
MODULE_DEVICE_TABLE(of, mp2652_of_match);

static const struct i2c_device_id mp2652_i2c_ids[] = { { "mp2652", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, mp2652_i2c_ids);

static struct i2c_driver mp2652_driver = {
	.driver = {
		.name = "mp2652",
		.of_match_table = mp2652_of_match,
	},
	.probe = mp2652_probe,
	.remove = mp2652_remove,
	.id_table = mp2652_i2c_ids,
};

module_i2c_driver(mp2652_driver);

MODULE_AUTHOR("Todd Goyen <todd@tgoyen.com>");
MODULE_DESCRIPTION("MP2652 Charger Driver");
MODULE_LICENSE("GPL");
