// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Deckard Silego Northgate Paddle Card MCU
 *
 * Copyright (C) 2025 Kevin Wolfe
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

static const struct i2c_device_id renesas_northgate_paddle[] = {
	{ "renesas-northgate-paddle", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, renesas_northgate_paddle);

static const struct of_device_id renesas_northgate_of_table[] = {
	{ .compatible = "renesas,northgate_paddle_mcu", (void *)8 },
	{}
};
MODULE_DEVICE_TABLE(of, renesas_northgate_of_table);

/*-------------------------------------------------------------------------*/
static int i2c_reg_read(struct i2c_client *client, u8 reg, u8 *buf, size_t len)
{
    struct i2c_msg msgs[2] = {
        {
            .addr  = client->addr,
            .flags = 0,            // Write
            .len   = 1,
            .buf   = &reg,
        },
        {
            .addr  = client->addr,
            .flags = I2C_M_RD,     // Read
            .len   = len,
            .buf   = buf,
        },
    };

    int ret = i2c_transfer(client->adapter, msgs, 2);
    return (ret == 2) ? 0 : -EIO;
}

static int i2c_reg_write(struct i2c_client *client, u8 reg, const u8 *data, size_t len)
{
    u8 buf[65];  // reg + 64 bytes max
    if (len > 64)
        return -EINVAL;

    buf[0] = reg;
    memcpy(&buf[1], data, len);

    struct i2c_msg msg = {
        .addr  = client->addr,
        .flags = 0,
        .len   = len + 1,
        .buf   = buf,
    };

    int ret = i2c_transfer(client->adapter, &msg, 1);
    return (ret == 1) ? 0 : -EIO;
}

/*-------------------------------------------------------------------------*/

static int renesas_northgate_probe(struct i2c_client *client)
{
	int err;
	static int probed = 0;

	dev_info(&client->dev, "probed\n");

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

static void renesas_northgate_remove(struct i2c_client *client)
{
}

static struct i2c_driver renesas_northgate_driver = {
	.driver = {
		.name	= "renesas_northgate_paddle",
		.of_match_table = renesas_northgate_of_table,
	},
	.probe = renesas_northgate_probe,
	.remove = renesas_northgate_remove,
	.shutdown = renesas_northgate_remove,
	.id_table = renesas_northgate_paddle,
};

static int __init renesas_northgate_init(void)
{
	return i2c_add_driver(&renesas_northgate_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(renesas_northgate_init);

static void __exit renesas_northgate_exit(void)
{
	i2c_del_driver(&renesas_northgate_driver);
}
module_exit(renesas_northgate_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin Wolfe <kevinw@valvesoftware.com>");
