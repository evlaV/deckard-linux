// SPDX-License-Identifier: GPL-2.0
/*
 * northgate-hotswap.c - skeleton power-supply driver with threaded GPIO IRQ, I2C 0x18, PM, and sysfs bit-level attributes
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>

/* --- Registers --- */
#define REG_PATTERN_ID 0xE6
#define REG_LATCHED_FAULTS 0xF1
#define REG_REALTIME_STATUS 0xF2
#define REG_I2C_VIRTUAL_IN 0xF4
#define REG_FET_STATUS 0xF5
#define REG_REALTIME_STATUS2 0xF7

/* --- Bit macros --- */
/* LATCHED_FAULTS (RO) */
#define LT_HW_CONTROL BIT(3)
#define LT_SLOW_CHARGER BIT(5)
#define LT_PACK_STATE_CHANGED BIT(6)
#define LT_INTERNAL_UVLO BIT(7)

/* REALTIME_STATUS (RO) */
#define RT_PACK_CONNECTED BIT(2)
#define RT_SLOW_CHARGER_ENABLED BIT(6)

/* I2C_VIRTUAL_IN (RW) */
#define VI_USE_INTERNAL_BATTERY BIT(0)
#define VI_ENABLE_SLOW_CHARGER BIT(2)
#define VI_CLEAR_CHARGER_FAULT BIT(3)
#define VI_CLEAR_FAULT_EDGE BIT(4)

// TODO Add clear charger FAULT

/* FET_STATUS (RO) */
#define FET_INT_CHG BIT(0)
#define FET_INT_DIS BIT(1)
#define FET_REM_CHG BIT(2)
#define FET_REM_DSG BIT(3)
#define FET_INT_ACT BIT(5)
#define FET_REM_ACT BIT(6)

/* REALTIME_STATUS2 (RO) */
#define RT2_REMOVABLE_V_OK BIT(1)
#define RT2_INTERNAL_V_OK BIT(2)

/* --- Driver state --- */
struct northgate_hotswap_psy {
	struct device *dev;
	struct regmap *regmap;
	struct i2c_client *client;
	struct power_supply *psy;

	int removable_pack_connected;
};

/* --- Sysfs helpers with DEVICE_ATTR integrated --- */
#define NORTHGATE_HOTSWAP_BIT_SHOW(name, reg, bit_macro)                                         \
	static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) \
	{                                                                                        \
		struct power_supply *psy = dev_get_drvdata(dev);                                 \
		struct northgate_hotswap_psy *northgate = power_supply_get_drvdata(psy);         \
		unsigned int val;                                                                \
                                                                                                 \
		regmap_read(northgate->regmap, reg, &val);                                       \
		return sysfs_emit(buf, "%d\n", !!(val & (bit_macro)));                           \
	}

#define NORTHGATE_HOTSWAP_BIT_STORE(name, reg, bit_macro)                                                             \
	static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) \
	{                                                                                                             \
		struct power_supply *psy = dev_get_drvdata(dev);                                                      \
		struct northgate_hotswap_psy *northgate = power_supply_get_drvdata(psy);                              \
		unsigned long v;                                                                                      \
                                                                                                                      \
		if (kstrtoul(buf, 0, &v))                                                                             \
			return -EINVAL;                                                                               \
                                                                                                                      \
		regmap_update_bits(northgate->regmap, reg, bit_macro, (v ? bit_macro : 0));                           \
		return count;                                                                                         \
	}

/* --- Define attributes --- */

/* F1 RO bits */
NORTHGATE_HOTSWAP_BIT_SHOW(latched_hw_control, REG_LATCHED_FAULTS, LT_HW_CONTROL);
static DEVICE_ATTR_RO(latched_hw_control);

NORTHGATE_HOTSWAP_BIT_SHOW(latched_slow_charger, REG_LATCHED_FAULTS, LT_SLOW_CHARGER);
static DEVICE_ATTR_RO(latched_slow_charger);

NORTHGATE_HOTSWAP_BIT_SHOW(latched_pack_state_changed, REG_LATCHED_FAULTS, LT_PACK_STATE_CHANGED);
static DEVICE_ATTR_RO(latched_pack_state_changed);

NORTHGATE_HOTSWAP_BIT_SHOW(latched_internal_uvlo, REG_LATCHED_FAULTS, LT_INTERNAL_UVLO);
static DEVICE_ATTR_RO(latched_internal_uvlo);

/* F2 RO bits */
NORTHGATE_HOTSWAP_BIT_SHOW(realtime_pack_connected, REG_REALTIME_STATUS, RT_PACK_CONNECTED);
static DEVICE_ATTR_RO(realtime_pack_connected);
NORTHGATE_HOTSWAP_BIT_SHOW(realtime_slow_charger_enabled, REG_REALTIME_STATUS, RT_SLOW_CHARGER_ENABLED);
static DEVICE_ATTR_RO(realtime_slow_charger_enabled);

/* F4 RW bits */
NORTHGATE_HOTSWAP_BIT_SHOW(use_internal_battery, REG_I2C_VIRTUAL_IN, VI_USE_INTERNAL_BATTERY);
NORTHGATE_HOTSWAP_BIT_STORE(use_internal_battery, REG_I2C_VIRTUAL_IN, VI_USE_INTERNAL_BATTERY);
static DEVICE_ATTR_RW(use_internal_battery);

NORTHGATE_HOTSWAP_BIT_SHOW(enable_slow_charger, REG_I2C_VIRTUAL_IN, VI_ENABLE_SLOW_CHARGER);
NORTHGATE_HOTSWAP_BIT_STORE(enable_slow_charger, REG_I2C_VIRTUAL_IN, VI_ENABLE_SLOW_CHARGER);
static DEVICE_ATTR_RW(enable_slow_charger);

static ssize_t clear_fault_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct northgate_hotswap_psy *northgate = power_supply_get_drvdata(psy);
	unsigned int val;

	regmap_read(northgate->regmap, REG_I2C_VIRTUAL_IN, &val);
	// Toggle the Clear bit
	regmap_update_bits(northgate->regmap, REG_I2C_VIRTUAL_IN, VI_CLEAR_FAULT_EDGE, val ^ VI_CLEAR_FAULT_EDGE);

	return count;
}
static DEVICE_ATTR_WO(clear_fault);

/* F5 RO bits */
NORTHGATE_HOTSWAP_BIT_SHOW(fet_internal_charge, REG_FET_STATUS, FET_INT_CHG);
static DEVICE_ATTR_RO(fet_internal_charge);

NORTHGATE_HOTSWAP_BIT_SHOW(fet_internal_discharge, REG_FET_STATUS, FET_INT_DIS);
static DEVICE_ATTR_RO(fet_internal_discharge);

NORTHGATE_HOTSWAP_BIT_SHOW(fet_removable_charge, REG_FET_STATUS, FET_REM_CHG);
static DEVICE_ATTR_RO(fet_removable_charge);

NORTHGATE_HOTSWAP_BIT_SHOW(fet_removable_discharge, REG_FET_STATUS, FET_REM_DSG);
static DEVICE_ATTR_RO(fet_removable_discharge);

NORTHGATE_HOTSWAP_BIT_SHOW(internal_active, REG_FET_STATUS, FET_INT_ACT);
static DEVICE_ATTR_RO(internal_active);

NORTHGATE_HOTSWAP_BIT_SHOW(removable_active, REG_FET_STATUS, FET_REM_ACT);
static DEVICE_ATTR_RO(removable_active);

/* F7 RO bits */
NORTHGATE_HOTSWAP_BIT_SHOW(removable_voltage_ok, REG_REALTIME_STATUS2, RT2_REMOVABLE_V_OK);
static DEVICE_ATTR_RO(removable_voltage_ok);
NORTHGATE_HOTSWAP_BIT_SHOW(internal_voltage_ok, REG_REALTIME_STATUS2, RT2_INTERNAL_V_OK);
static DEVICE_ATTR_RO(internal_voltage_ok);

/* --- Attribute array --- */
static struct attribute *northgate_hotswap_attrs[] = { &dev_attr_latched_hw_control.attr,
						       &dev_attr_latched_slow_charger.attr,
						       &dev_attr_latched_pack_state_changed.attr,
						       &dev_attr_latched_internal_uvlo.attr,
						       &dev_attr_realtime_pack_connected.attr,
						       &dev_attr_realtime_slow_charger_enabled.attr,
						       &dev_attr_use_internal_battery.attr,
						       &dev_attr_enable_slow_charger.attr,
						       &dev_attr_clear_fault.attr,
						       &dev_attr_fet_internal_charge.attr,
						       &dev_attr_fet_internal_discharge.attr,
						       &dev_attr_fet_removable_charge.attr,
						       &dev_attr_fet_removable_discharge.attr,
						       &dev_attr_internal_active.attr,
						       &dev_attr_removable_active.attr,
						       &dev_attr_removable_voltage_ok.attr,
						       &dev_attr_internal_voltage_ok.attr,
						       NULL };
ATTRIBUTE_GROUPS(northgate_hotswap);

static int northgate_hotswap_get_property(struct power_supply *psy, enum power_supply_property psp,
					  union power_supply_propval *val)
{
	struct northgate_hotswap_psy *northgate = power_supply_get_drvdata(psy);
	unsigned int reg_val;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		regmap_read(northgate->regmap, REG_I2C_VIRTUAL_IN, &reg_val);
		reg_val &= LT_SLOW_CHARGER;
		val->intval = reg_val ? POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		// TODO - support this if we can detect a dead battery
		// POWER_SUPPLY_HEALTH_GOOD, // Battery Present and DSG FET Closed
		// POWER_SUPPLY_HEALTH_DEAD, // Battery Present and DSG FET Open
		// POWER_SUPPLY_HEALTH_NO_BATTERY, // Battery Removed
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = northgate->removable_pack_connected;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property northgate_hotswap_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc northgate_hotswap_psy_desc = {
	.name = "northgate-hotswap",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = northgate_hotswap_psy_props,
	.num_properties = ARRAY_SIZE(northgate_hotswap_psy_props),
	.get_property = northgate_hotswap_get_property,
	.no_thermal = true,
};

static int northgate_hotswap_removable_pack_connected(struct northgate_hotswap_psy *northgate)
{
	int val = 0;

	regmap_read(northgate->regmap, REG_REALTIME_STATUS, &val);
	if (val >= 0) {
		val = !!(val & RT_PACK_CONNECTED);
	}

	return val;
}

/* --- Threaded IRQ handler --- */
static irqreturn_t northgate_hotswap_irq_thread(int irq, void *dev_id)
{
	struct northgate_hotswap_psy *northgate = dev_id;
	int val;

	val = northgate_hotswap_removable_pack_connected(northgate);
	if (val >= 0) {
		northgate->removable_pack_connected = val;
		pr_info("northgate-hotswap installed: %d\n", val);
	}

	power_supply_changed(northgate->psy);

	return IRQ_HANDLED;
}

static const struct regmap_config northgate_hotswap_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.cache_type = REGCACHE_NONE, // We could cache two values if the aDSP wasn't modifying them during suspend
};
// TODO add cache and use regcache_mark_dirty()
// TODO Check this for the charger as well

/* --- Probe / Remove --- */
static int northgate_hotswap_probe(struct i2c_client *client)
{
	struct northgate_hotswap_psy *northgate;
	struct power_supply_config psy_cfg = {};
	struct device *dev = &client->dev;
	int val;
	int ret;

	northgate = devm_kzalloc(dev, sizeof(*northgate), GFP_KERNEL);
	if (!northgate)
		return -ENOMEM;

	northgate->dev = dev;
	northgate->client = client;
	i2c_set_clientdata(client, northgate);

	northgate->regmap = devm_regmap_init_i2c(client, &northgate_hotswap_regmap);
	if (IS_ERR(northgate->regmap))
		return PTR_ERR(northgate->regmap);

	// Check for the Device, if its not there, just bail
	val = -1;
	regmap_read(northgate->regmap, REG_PATTERN_ID, &val);
	if (val != 0x04) {
		if (val > 0) {
			dev_info(dev, "northgate-hotswap load aborted, unknown pattern id: %d\n", val);
		}
		return 0;
	}

	// setup of GPIO alert failed: -1
	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL, northgate_hotswap_irq_thread,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "northgate-hotswap-alert",
						northgate);
		if (ret) {
			dev_err(&client->dev, "Unable to register IRQ %d err %d\n", client->irq, ret);
			return ret;
		}
	}

	northgate->removable_pack_connected = northgate_hotswap_removable_pack_connected(northgate);

	psy_cfg.drv_data = northgate;
	psy_cfg.fwnode = dev_fwnode(dev);
	psy_cfg.attr_grp = northgate_hotswap_groups;
	northgate->psy = devm_power_supply_register(dev, &northgate_hotswap_psy_desc, &psy_cfg);
	if (IS_ERR(northgate->psy))
		return PTR_ERR(northgate->psy);

	dev_info(dev, "northgate-hotswap loaded\n");
	return 0;
}

static const struct i2c_device_id northgate_hotswap_id_table[] = { { "slg4ad48669m" }, {} };
MODULE_DEVICE_TABLE(i2c, northgate_hotswap_id_table);

#ifdef CONFIG_OF
static const struct of_device_id northgate_hotswap_of_match[] = {
	{ .compatible = "silego,northgate_hotswap" },
	{},
};
MODULE_DEVICE_TABLE(of, northgate_hotswap_of_match);
#endif

static struct i2c_driver northgate_hotswap_i2c_driver = {
	.probe = northgate_hotswap_probe,
	.id_table = northgate_hotswap_id_table,
	.driver = {
		.name = "northgate-hotswap",
		.of_match_table = of_match_ptr(northgate_hotswap_of_match),
	},
};
module_i2c_driver(northgate_hotswap_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Todd Goyen <todd@tgoyen.com>");
