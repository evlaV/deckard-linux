// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  i2c-broadcaster - sw broadcaster for multi client
 */


#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/xarray.h>

struct i2c_broadcaster {
	struct i2c_client *i2c;
	struct i2c_adapter adapter;
	struct notifier_block notifier;
	struct xarray clients;
};

static int i2c_broadcaster_notify(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct i2c_broadcaster *bcaster = container_of(nb, struct i2c_broadcaster,
							notifier);
	struct i2c_client *i2c = bcaster->i2c;
	struct device_node *bnode = i2c->dev.of_node;
	struct device_node *phandle_node;
	struct i2c_client *client;
	struct device *dev = data;
	int ret;

	if (action != BUS_NOTIFY_BOUND_DRIVER && action != BUS_NOTIFY_UNBOUND_DRIVER)
		return NOTIFY_DONE;

	client = i2c_verify_client(dev);
	if (!client)
		return NOTIFY_DONE;

	phandle_node = of_parse_phandle(dev->of_node, "broadcaster-parent", 0);
	if (phandle_node == bnode) {
		dev_info(&i2c->dev, "broadcaster: new device %s\n", dev_name(dev));

		if (action == BUS_NOTIFY_BOUND_DRIVER) {
			ret = xa_err(xa_store(&bcaster->clients, client->addr, client,
						GFP_KERNEL));
			if (ret < 0) {
				dev_info(&i2c->dev, "broadcaster: %s failed to xa_store\n",
					 dev_name(dev));
				goto node_put;
			}
		} else if (action == BUS_NOTIFY_UNBOUND_DRIVER) {
			xa_erase(&bcaster->clients, client->addr);
		}
	}

node_put:
	of_node_put(phandle_node);

	return NOTIFY_DONE;
}

static s32 i2c_broadcaster_xfer(struct i2c_adapter *adap, u16 addr,
				unsigned short flags, char read_write,
				u8 command, int size,
				union i2c_smbus_data *data)
{
	struct i2c_broadcaster *bcaster = container_of(adap, struct i2c_broadcaster,
							adapter);
	struct i2c_client *client;
	unsigned long index;
	int ret = 0;

	if (size != I2C_SMBUS_BYTE)
		return 0;

	if (read_write != I2C_SMBUS_WRITE)
		return 0;

	/* for now we only handle the write byte */
	xa_for_each(&bcaster->clients, index , client) {
		dev_dbg(&adap->dev, "%s: broadcaster transaction write: %d size: %d addr: 0x%x\n",
				 __func__, read_write, size, client->addr);
		ret = i2c_smbus_write_byte(client, command);
	}

	return 0;
}

static u32 i2c_broadcaster_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_SMBUS_BYTE;
}

static const struct i2c_algorithm smbus_algorithm = {
	.functionality	= i2c_broadcaster_func,
	.smbus_xfer	= i2c_broadcaster_xfer,
};

static int i2c_broadcaster_probe(struct i2c_client *client)
{
	struct i2c_broadcaster *bcaster;
	int ret;

	bcaster = devm_kzalloc(&client->dev, sizeof(struct i2c_broadcaster),
				GFP_KERNEL);
	if (!bcaster)
		return -ENOMEM;

	bcaster->i2c = client;

	i2c_set_clientdata(client, bcaster);

	bcaster->adapter.owner = THIS_MODULE;
	bcaster->adapter.class = I2C_CLASS_HWMON;
	bcaster->adapter.algo = &smbus_algorithm;
	snprintf(bcaster->adapter.name, sizeof(bcaster->adapter.name),
		 "SMBus broadcaster adapter 0x%x", client->addr);

	i2c_set_adapdata(&bcaster->adapter, bcaster);

	ret = devm_i2c_add_adapter(&client->dev, &bcaster->adapter);
	if (ret < 0)
		return ret;

	client->adapter = &bcaster->adapter;

	bcaster->notifier.notifier_call = i2c_broadcaster_notify;

	xa_init(&bcaster->clients);

	ret = bus_register_notifier(&i2c_bus_type, &bcaster->notifier);
	if (ret)
		return ret;

	return 0;
}

static void i2c_broadcaster_remove(struct i2c_client *client)
{
	struct i2c_broadcaster *bcaster = i2c_get_clientdata(client);

	xa_destroy(&bcaster->clients);
}

static const struct of_device_id i2c_broadcaster_of_match[] = {
	{
		.compatible = "i2c-sw-broadcaster"
	},
	{ },
};

static struct i2c_driver i2c_broadcaster_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "i2c-sw-broadcaster",
		.of_match_table = of_match_ptr(i2c_broadcaster_of_match),
	},
	.probe = i2c_broadcaster_probe,
	.remove = i2c_broadcaster_remove,
};
module_i2c_driver(i2c_broadcaster_driver);
