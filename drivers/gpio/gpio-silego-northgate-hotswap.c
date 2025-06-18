// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Deckard Silego Northgate Hotswap I2C GPIO expander
 *
 * Copyright (C) 2024 Todd Goyen
 */

#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

static const struct i2c_device_id slg_northgate_id[] = {
	{ "slg-northgate-hotswap", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, slg_northgate_id);

static const struct of_device_id slg_northgate_of_table[] = {
	{ .compatible = "silego,northgate_hotswap", (void *)8 },
	{}
};
MODULE_DEVICE_TABLE(of, slg_northgate_of_table);

#define SLG_NORTHGATE_GPIO_MAX 3

#define SLG_NORTHGATE_REG_LUT2_3 0xF1
#define SLG_NORTHGATE_REG_VIRTUAL_INPUT 0xF4

#define SLG_NORTHGATE_GPIO_INPUT_MASK 0x01
#define SLG_NORTHGATE_GPIO_OUTPUT_MASK 0x06

struct northgate_gpio {
	u8 address;
	u8 mask;
};

static const char *gpio_names[SLG_NORTHGATE_GPIO_MAX] = { "pack_connected",
							  "ra2e1_bootloader",
							  "ra2e1_reset" };

static struct northgate_gpio gpios[SLG_NORTHGATE_GPIO_MAX] = {
	{ SLG_NORTHGATE_REG_LUT2_3, BIT(2) },
	{ SLG_NORTHGATE_REG_VIRTUAL_INPUT, BIT(0) },
	{ SLG_NORTHGATE_REG_VIRTUAL_INPUT, BIT(5) },
};

struct slg_northgate {
	struct gpio_chip chip;
	struct regmap *regmap;
};

static bool slg_northgate_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SLG_NORTHGATE_REG_LUT2_3:
	case SLG_NORTHGATE_REG_VIRTUAL_INPUT:
		return true;
	default:
		return false;
	}
}

static bool slg_northgate_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SLG_NORTHGATE_REG_VIRTUAL_INPUT:
		return true;
	case SLG_NORTHGATE_REG_LUT2_3:
	default:
		return false;
	}
}

static bool slg_northgate_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SLG_NORTHGATE_REG_LUT2_3:
		return true;
	case SLG_NORTHGATE_REG_VIRTUAL_INPUT:
	default:
		return false;
	}
}

static const struct regmap_config slg_northgate_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = slg_northgate_writeable_reg,
	.readable_reg = slg_northgate_readable_reg,
	.volatile_reg = slg_northgate_volatile_reg,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_NONE,
};

/*-------------------------------------------------------------------------*/

static int slg_northgate_get(struct gpio_chip *chip, unsigned int offset)
{
	struct slg_northgate *slg_ng = gpiochip_get_data(chip);
	unsigned int value;
	int ret;

	ret = regmap_read(slg_ng->regmap, gpios[offset].address, &value);
	if (ret != 0)
		return ret;

	return ((value & gpios[offset].mask) ? 1 : 0);
}

static int slg_northgate_get_multiple(struct gpio_chip *chip,
				      unsigned long *mask, unsigned long *bits)
{
	struct slg_northgate *slg_ng = gpiochip_get_data(chip);
	unsigned long full_value = 0;
	unsigned int value;
	int ret;
	int i;

	for (i = 0; i < SLG_NORTHGATE_GPIO_MAX; i++) {
		ret = regmap_read(slg_ng->regmap, gpios[i].address, &value);
		if (ret != 0)
			return ret;

		if (value & gpios[i].mask)
			full_value |= (1 << i);
	}

	*bits &= ~*mask;
	*bits |= full_value & *mask;

	return 0;
}

static int slg_northgate_set(struct gpio_chip *chip, unsigned int offset,
			      int value)
{
	struct slg_northgate *slg_ng = gpiochip_get_data(chip);

	if (value)
		return regmap_set_bits(slg_ng->regmap, gpios[offset].address,
				gpios[offset].mask);
	else
		return regmap_clear_bits(slg_ng->regmap, gpios[offset].address,
				  gpios[offset].mask);
}

static int slg_northgate_set_multiple(struct gpio_chip *chip,
				       unsigned long *mask, unsigned long *bits)
{
	struct slg_northgate *slg_ng = gpiochip_get_data(chip);
	unsigned long index_mask;
	int i, ret = 0;

	for (i = 0; i < SLG_NORTHGATE_GPIO_MAX; i++) {
		index_mask = 1 << i;

		if (*bits & index_mask)
			ret = regmap_set_bits(slg_ng->regmap, gpios[i].address,
					gpios[i].mask);
		else if (*mask & index_mask)
			ret = regmap_clear_bits(slg_ng->regmap, gpios[i].address,
					  gpios[i].mask);

		if (ret)
			return ret;
	}

	return 0;
}

static int slg_northgate_get_direction(struct gpio_chip *chip,
				       unsigned int offset)
{
	const unsigned long mask = 1 << offset;

	if (offset >= SLG_NORTHGATE_GPIO_MAX) {
		return -EINVAL;
	}

	if (mask & SLG_NORTHGATE_GPIO_INPUT_MASK) {
		return GPIO_LINE_DIRECTION_IN;
	} else if (mask & SLG_NORTHGATE_GPIO_OUTPUT_MASK) {
		return GPIO_LINE_DIRECTION_OUT;
	} else {
		return -EINVAL;
	}
}

/*-------------------------------------------------------------------------*/

static int slg_northgate_probe(struct i2c_client *client)
{
	struct slg_northgate *slg_ng;
	int err;
	static int probed = 0;

	/* Allocate, initialize, and register this gpio_chip. */
	slg_ng = devm_kzalloc(&client->dev, sizeof(*slg_ng), GFP_KERNEL);
	if (!slg_ng)
		return -ENOMEM;

	dev_set_drvdata(&client->dev, slg_ng);

	slg_ng->regmap =
		devm_regmap_init_i2c(client, &slg_northgate_regmap_config);
	if (IS_ERR(slg_ng->regmap)) {
		err = PTR_ERR(slg_ng->regmap);
		dev_err(&client->dev, "regmap init failed: %d", err);
		return err;
	}

	slg_ng->chip.label = client->name;
	slg_ng->chip.base = -1;
	slg_ng->chip.can_sleep = true;
	slg_ng->chip.parent = &client->dev;
	slg_ng->chip.owner = THIS_MODULE;
	slg_ng->chip.get = slg_northgate_get;
	slg_ng->chip.get_multiple = slg_northgate_get_multiple;
	slg_ng->chip.set = slg_northgate_set;
	slg_ng->chip.set_multiple = slg_northgate_set_multiple;
	slg_ng->chip.get_direction = slg_northgate_get_direction;
	slg_ng->chip.ngpio = SLG_NORTHGATE_GPIO_MAX;
	slg_ng->chip.names = gpio_names;

	err = devm_gpiochip_add_data(&client->dev, &slg_ng->chip, slg_ng);
	if (err < 0)
		goto fail;

	dev_info(&client->dev, "probed\n");

	// // Power Down the Espressif
	// dev_info(&client->dev, "booting espressif\n");
	// slg_northgate_set(&slg_ng->chip, 4, 0);
	// slg_northgate_set(&slg_ng->chip, 5, 1);

	return 0;

fail:
	// Limit future error message on other headstraps
	if (probed == 0) {
		dev_dbg(&client->dev, "probe error %d for '%s'\n", err,
			client->name);
		probed = 1;
	}

	return err;
}

static void slg_northgate_remove(struct i2c_client *client)
{
	struct slg_northgate *slg_ng = dev_get_drvdata(&client->dev);

	// Power Down the Espressif
	slg_northgate_set(&slg_ng->chip, 5, 0);
}

static struct i2c_driver slg_northgate_driver = {
	.driver = {
		.name	= "slg_northgate_hotswap",
		.of_match_table = slg_northgate_of_table,
	},
	.probe = slg_northgate_probe,
	.remove = slg_northgate_remove,
	.shutdown = slg_northgate_remove,
	.id_table = slg_northgate_id,
};

static int __init slg_northgate_init(void)
{
	return i2c_add_driver(&slg_northgate_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(slg_northgate_init);

static void __exit slg_northgate_exit(void)
{
	i2c_del_driver(&slg_northgate_driver);
}
module_exit(slg_northgate_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Todd Goyen <todd@tgoyen.com>");
