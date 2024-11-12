// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Deckard Silego Northgate Paddle Card I2C GPIO expander
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

static const struct i2c_device_id slg_northgate_id[] = { { "slg-northgate", 0 },
							 {} };
MODULE_DEVICE_TABLE(i2c, slg_northgate_id);

static const struct of_device_id slg_northgate_of_table[] = {
	{ .compatible = "silego,northgate", (void *)8 },
	{}
};
MODULE_DEVICE_TABLE(of, slg_northgate_of_table);

#define SLG_NORTHGATE_GPIO_MAX 7

#define SLG_NORTHGATE_REG_BUTTON 0xF2
#define SLG_NORTHGATE_REG_OUTPUT 0xF4

#define SLG_NORTHGATE_GPIO_INPUT_MASK 0x0F
#define SLG_NORTHGATE_GPIO_OUTPUT_MASK 0x70

#define SLG_NORTHGATE_GPIO_OUTPUT_MASK_BOOTLOADER 0x10
#define SLG_NORTHGATE_GPIO_OUTPUT_MASK_BOOT 0x20
#define SLG_NORTHGATE_GPIO_OUTPUT_MASK_DISABLE_3P3 0x40

static const char *gpio_names[SLG_NORTHGATE_GPIO_MAX] = {
	"Button1",	  "Button2", "Button3",	  "Button4",
	"LoadBootloader", "Run",     "Disable3P3"
};

static const int index_map[SLG_NORTHGATE_GPIO_MAX] = {
	0x02, 0x04, 0x08, 0x10, 0x01, 0x20, 0x04,
};

struct slg_northgate {
	struct gpio_chip chip;
	struct regmap *regmap;
};

static bool slg_northgate_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SLG_NORTHGATE_REG_BUTTON:
	case SLG_NORTHGATE_REG_OUTPUT:
		return true;
	default:
		return false;
	}
}

static bool slg_northgate_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SLG_NORTHGATE_REG_OUTPUT:
		return true;
	case SLG_NORTHGATE_REG_BUTTON:
	default:
		return false;
	}
}

static bool slg_northgate_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SLG_NORTHGATE_REG_BUTTON:
		return true;
	case SLG_NORTHGATE_REG_OUTPUT:
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

	const unsigned long mask = 1 << offset;
	unsigned int value;
	int ret;

	if (mask & SLG_NORTHGATE_GPIO_INPUT_MASK) {
		ret = regmap_read(slg_ng->regmap, SLG_NORTHGATE_REG_BUTTON,
				  &value);
		if (ret != 0)
			return ret;

		// Button Logic is inverted
		return (value & index_map[offset] ? 0 : 1);

	} else if (mask & SLG_NORTHGATE_GPIO_OUTPUT_MASK) {
		ret = regmap_read(slg_ng->regmap, SLG_NORTHGATE_REG_OUTPUT,
				  &value);
		if (ret != 0)
			return ret;

		return (value & index_map[offset] ? 1 : 0);

	} else {
		return -EINVAL;
	}
}

static int slg_northgate_get_multiple(struct gpio_chip *chip,
				      unsigned long *mask, unsigned long *bits)
{
	struct slg_northgate *slg_ng = gpiochip_get_data(chip);
	unsigned int button_reg, output_reg;
	unsigned long value;
	int ret;

	/* Update Button Reg */
	ret = regmap_read(slg_ng->regmap, SLG_NORTHGATE_REG_BUTTON,
			  &button_reg);
	if (ret != 0)
		return ret;
	ret = regmap_read(slg_ng->regmap, SLG_NORTHGATE_REG_OUTPUT,
			  &output_reg);
	if (ret != 0)
		return ret;

	// Rebuild the Value register in the packed order for linux userspace
	value = (output_reg & 0x01) << 4;
	value |= (output_reg & 0x20);
	value |= (output_reg & 0x04) << 4;
	value |= (button_reg >> 1);

	*bits &= ~*mask;
	*bits |= value & *mask;

	return 0;
}

static void slg_northgate_set(struct gpio_chip *chip, unsigned int offset,
			      int value)
{
	struct slg_northgate *slg_ng = gpiochip_get_data(chip);

	const unsigned long mask = 1 << offset;
	unsigned int reg_value;
	int ret;

	if (mask & SLG_NORTHGATE_GPIO_OUTPUT_MASK) {
		ret = regmap_read(slg_ng->regmap, SLG_NORTHGATE_REG_OUTPUT,
				  &reg_value);
		if (ret != 0)
			return;

		reg_value &= ~mask;

		if (mask & SLG_NORTHGATE_GPIO_OUTPUT_MASK_BOOTLOADER && value)
			reg_value |= 0x01;
		else if (mask & SLG_NORTHGATE_GPIO_OUTPUT_MASK_BOOT && value)
			reg_value |= 0x20;
		else if (mask & SLG_NORTHGATE_GPIO_OUTPUT_MASK_DISABLE_3P3 &&
			 value)
			reg_value |= 0x04;

		regmap_write(slg_ng->regmap, SLG_NORTHGATE_REG_OUTPUT,
			     reg_value);
	}
}

static void slg_northgate_set_multiple(struct gpio_chip *chip,
				       unsigned long *mask, unsigned long *bits)
{
	struct slg_northgate *slg_ng = gpiochip_get_data(chip);
	unsigned int reg_value;
	int ret;

	// Ignore Inputs
	*mask &= SLG_NORTHGATE_GPIO_OUTPUT_MASK;
	*bits &= SLG_NORTHGATE_GPIO_OUTPUT_MASK;

	// Check for NOOP
	if (*mask == 0)
		return;

	ret = regmap_read(slg_ng->regmap, SLG_NORTHGATE_REG_OUTPUT, &reg_value);
	if (ret != 0)
		return;

	reg_value &= ~*mask;

	if (*bits & SLG_NORTHGATE_GPIO_OUTPUT_MASK_BOOTLOADER)
		reg_value |= 0x01;

	if (*bits & SLG_NORTHGATE_GPIO_OUTPUT_MASK_BOOT)
		reg_value |= 0x20;

	if (*bits & SLG_NORTHGATE_GPIO_OUTPUT_MASK_DISABLE_3P3)
		reg_value |= 0x04;

	regmap_write(slg_ng->regmap, SLG_NORTHGATE_REG_OUTPUT, reg_value);
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

	// Power Down the Espressif
	dev_info(&client->dev, "booting espressif\n");
	slg_northgate_set(&slg_ng->chip, 5, 1);

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
		.name	= "slg_northgate",
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
MODULE_AUTHOR("Todd Goyen");
