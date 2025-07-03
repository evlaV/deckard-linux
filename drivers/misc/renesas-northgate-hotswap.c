// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/crc32.h>

#define WHOAMI_REG         0x00
#define WHOAMI_EXPECTED    0xA3
#define FW_VERSION_REG     0x01
#define FW_VERSION_LEN     4
#define FW_UPDATE_MAX_SIZE 1024

// Firmware update protocol registers
#define I2C_REG_FWU_STATE    0x80
#define I2C_REG_FWU_DATA     0x81
#define I2C_REG_FWU_VALIDATE 0x82
#define I2C_REG_FWU_APPLY    0x83

struct northgate_data {
	u8 fw_version[FW_VERSION_LEN];
	u8 *fw_update_buf;
	size_t fw_update_len;
	struct mutex fw_update_lock;
};

static const struct i2c_device_id renesas_northgate_hotswap[] = {
	{ "renesas-northgate-hotswap", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, renesas_northgate_hotswap);

static const struct of_device_id renesas_northgate_of_table[] = {
	{ .compatible = "renesas,northgate_hotswap_mcu" },
	{}
};
MODULE_DEVICE_TABLE(of, renesas_northgate_of_table);

static int i2c_reg_read(struct i2c_client *client, u8 reg, u8 *buf, size_t len)
{
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .flags = 0,        .len = 1,   .buf = &reg },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = len, .buf = buf  },
	};

	int ret = i2c_transfer(client->adapter, msgs, 2);
	return (ret == 2) ? 0 : -EIO;
}

static int i2c_reg_write(struct i2c_client *client, u8 reg, const u8 *data, size_t len)
{
	u8 buf[65];
	if (len > 64)
		return -EINVAL;

	buf[0] = reg;
	memcpy(&buf[1], data, len);

	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = len + 1,
		.buf = buf,
	};

	int ret = i2c_transfer(client->adapter, &msg, 1);
	return (ret == 1) ? 0 : -EIO;
}

static int northgate_flash_firmware(struct i2c_client *client, struct northgate_data *data)
{
	u8 val;
	int err;
	size_t offset = 0;

	// Step 1: initiate update
	val = 1;
	err = i2c_reg_write(client, I2C_REG_FWU_STATE, &val, 1);
	if (err)
		return err;

	// Step 2: send data in 64-byte chunks
	while (offset < data->fw_update_len) {
		size_t chunk = min((size_t)64, data->fw_update_len - offset);
		err = i2c_reg_write(client, I2C_REG_FWU_DATA, data->fw_update_buf + offset, chunk);
		if (err)
			return err;
		offset += chunk;
	}

	// Step 3: compute CRC32 and send it
	u32 crc = crc32(~0, data->fw_update_buf, data->fw_update_len) ^ ~0;
	u8 crc_bytes[4] = {
		(u8)(crc & 0xFF),
		(u8)((crc >> 8) & 0xFF),
		(u8)((crc >> 16) & 0xFF),
		(u8)((crc >> 24) & 0xFF)
	};
	err = i2c_reg_write(client, I2C_REG_FWU_VALIDATE, crc_bytes, 4);
	if (err)
		return err;

	// Step 4: apply (reboot)
	val = 1;
	err = i2c_reg_write(client, I2C_REG_FWU_APPLY, &val, 1);
	if (err)
		return err;

	dev_info(&client->dev, "Firmware update sent and apply command issued.\n");
	return 0;
}

static ssize_t fw_version_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct northgate_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%u.%u.%u.%u\n",
	               data->fw_version[0], data->fw_version[1],
	               data->fw_version[2], data->fw_version[3]);
}
static DEVICE_ATTR_RO(fw_version);

static ssize_t fw_update_write(struct file *filp, struct kobject *kobj,
                               struct bin_attribute *attr, char *buf,
                               loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct i2c_client *client = to_i2c_client(dev);
	struct northgate_data *data = i2c_get_clientdata(client);
	int err;

	if (off != 0)
		return -EINVAL; // Full image writes only

	if (count > FW_UPDATE_MAX_SIZE)
		return -EFBIG;

	mutex_lock(&data->fw_update_lock);

	if (!data->fw_update_buf) {
		data->fw_update_buf = kmalloc(FW_UPDATE_MAX_SIZE, GFP_KERNEL);
		if (!data->fw_update_buf) {
			mutex_unlock(&data->fw_update_lock);
			return -ENOMEM;
		}
	}

	memcpy(data->fw_update_buf, buf, count);
	data->fw_update_len = count;

	err = northgate_flash_firmware(client, data);

	mutex_unlock(&data->fw_update_lock);

	return err ? err : count;
}

static struct bin_attribute fw_update_attr = {
	.attr = {
		.name = "fw_update",
		.mode = 0220, // write-only
	},
	.size  = FW_UPDATE_MAX_SIZE,
	.write = fw_update_write,
};

static int renesas_northgate_probe(struct i2c_client *client)
{
	int err;
	u8 whoami;
	struct northgate_data *data;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	// Check WHOAMI
	err = i2c_reg_read(client, WHOAMI_REG, &whoami, 1);
	if (err) {
		dev_err(&client->dev, "Failed to read WHOAMI: %d\n", err);
		return err;
	}
	if (whoami != WHOAMI_EXPECTED) {
		dev_err(&client->dev, "Unexpected WHOAMI value: 0x%02X\n", whoami);
		return -ENODEV;
	}

	// Read firmware version
	err = i2c_reg_read(client, FW_VERSION_REG, data->fw_version, FW_VERSION_LEN);
	if (err) {
		dev_err(&client->dev, "Failed to read firmware version: %d\n", err);
		return err;
	}

	mutex_init(&data->fw_update_lock);
	i2c_set_clientdata(client, data);

	err = device_create_file(&client->dev, &dev_attr_fw_version);
	if (err)
		dev_warn(&client->dev, "Failed to create fw_version sysfs entry\n");

	err = device_create_bin_file(&client->dev, &fw_update_attr);
	if (err)
		dev_warn(&client->dev, "Failed to create fw_update sysfs entry\n");

	dev_info(&client->dev,
	         "Renesas Northgate hotswap detected (WHOAMI=0x%02X), FW %u.%u.%u.%u\n",
	         whoami,
	         data->fw_version[0], data->fw_version[1],
	         data->fw_version[2], data->fw_version[3]);

	return 0;
}

static void renesas_northgate_remove(struct i2c_client *client)
{
	struct northgate_data *data = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_fw_version);
	device_remove_bin_file(&client->dev, &fw_update_attr);

	mutex_destroy(&data->fw_update_lock);
	kfree(data->fw_update_buf);
}

static struct i2c_driver renesas_northgate_driver = {
	.driver = {
		.name = "renesas_northgate_hotswap",
		.of_match_table = renesas_northgate_of_table,
	},
	.probe    = renesas_northgate_probe,
	.remove   = renesas_northgate_remove,
	.shutdown = renesas_northgate_remove,
	.id_table = renesas_northgate_hotswap,
};

static int __init renesas_northgate_init(void)
{
	return i2c_add_driver(&renesas_northgate_driver);
}
subsys_initcall(renesas_northgate_init);

static void __exit renesas_northgate_exit(void)
{
	i2c_del_driver(&renesas_northgate_driver);
}
module_exit(renesas_northgate_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin Wolfe <kevinw@valvesoftware.com>");
MODULE_DESCRIPTION("Renesas Northgate Hotswap Controller I2C Driver");
