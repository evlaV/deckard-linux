// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allegro ALS31300 3-D Linear Hall Effect driver
 *
 * Copyright (C) 2021 Valve
 * Author: Todd Goyen <todd@tgoyen.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>

#define ALS31300_DRV_NAME "als31300"

#define EEPROM_WRITE_DELAY_MS 50
#define POLL_INTERVAL 50
#define POLL_INTERVAL_MAX 100

/* register definitions */
#define ALS31300_EEPROM_CONFIG 0x02
#define ALS31300_EEPROM_INTERRUPT 0x03
#define ALS31300_EEPROM_CUSTOMER_1 0x0D
#define ALS31300_EEPROM_CUSTOMER_2 0x0E
#define ALS31300_EEPROM_CUSTOMER_3 0x0F
#define ALS31300_VOLATILE_MODE 0x27
#define ALS31300_VOLATILE_MSB 0x28
#define ALS31300_VOLATILE_LSB 0x29
#define ALS31300_CUSTOMER_ACCESS 0x35

#define ALS31300_NEW_DATA_MASK 0x80

#define ALS31300_LPDCM_MASK 0x70
#define ALS31300_MODE_MASK 0x03

#define ALS31300_LPDCM_INACTIVE_0_5_MS 0x00
#define ALS31300_LPDCM_INACTIVE_1_0_MS 0x10
#define ALS31300_LPDCM_INACTIVE_5_0_MS 0x20
#define ALS31300_LPDCM_INACTIVE_10_0_MS 0x30
#define ALS31300_LPDCM_INACTIVE_50_0_MS 0x40
#define ALS31300_LPDCM_INACTIVE_100_0_MS 0x50
#define ALS31300_LPDCM_INACTIVE_500_0_MS 0x60
#define ALS31300_LPDCM_INACTIVE_1000_0_MS 0x70

#define ALS31300_MODE_ACTIVE 0x00
#define ALS31300_MODE_SLEEP 0x01
#define ALS31300_MODE_LPDCM 0x02

#define ALS31300_CUSTOMER_ACCESS_DATA 0x2C413534UL

struct als31300_sample {
	u64 timestamp;
	int x, y, z;
	u32 temperature;
};

struct als31300_priv {
	struct device *dev;
	struct i2c_client *client;
	struct mutex lock;
	struct als31300_sample last_raw_sample;
};

static int als31300_read_block(struct i2c_client *client, unsigned int off,
			       u8 *buf, size_t size)
{
	int err;

	err = i2c_smbus_read_i2c_block_data(client, off, size, buf);
	if (err < 0) {
		dev_err(&client->dev,
			"failed to read block data at 0x%02x, error %d\n", off,
			err);
		return err;
	}

	return 0;
}

static int als31300_write_block(struct i2c_client *client, unsigned int off,
				const u8 *const buf, size_t size)
{
	int err;

	err = i2c_smbus_write_i2c_block_data(client, off, size, buf);
	if (err < 0) {
		dev_err(&client->dev,
			"failed to write block data at 0x%02x, error %d\n", off,
			err);
		return err;
	}

	return 0;
}

static void als31300_poll(struct input_dev *input)
{
	struct als31300_priv *priv = input_get_drvdata(input);
	struct i2c_client *client = priv->client;
	int ret;
	u8 buf[8];
	struct als31300_sample sample;

	/* Read Data */
	ret = als31300_read_block(client, ALS31300_VOLATILE_MSB, buf,
				  sizeof(buf));
	if (ret < 0) {
		dev_err(&client->dev, "read data failed, error %d\n", ret);
		return;
	}

	/* Check if the data is new */
	if ((buf[3] & ALS31300_NEW_DATA_MASK) == 0) {
		/* Data is stale */
		/* NOTE That this happens with evtest after the open, there is an immediate read
		 * and the system needs tPOD (600 us) to wake up. Rather than keep track of this,
		 * we just ignore the read if the data is stale and move on.
		 */
		// u32 data[2];
		// memcpy(data, buf, sizeof(buf));
		// dev_info(&client->dev, "read data stale, 0x%08X 0x%08X\n", data[0], data[1]);
		return;
	}

	/* Convert Readings to X, Y, Z, and Temperature */
	sample.x = ((int)(s8)buf[0] << 4) | (buf[5] & 0x0F);
	sample.y = ((int)(s8)buf[1] << 4) | ((buf[6] & 0xF0) >> 4);
	sample.z = ((int)(s8)buf[2] << 4) | (buf[6] & 0x0F);
	sample.temperature = ((u32)(u8)(buf[3] & 0x3F) << 6) | (buf[7] & 0x3F);
	/* Convert Temperature to milliC */
	sample.temperature = 302 * (sample.temperature - 1708) * 1000 / 4096;
	sample.timestamp = ktime_get_ns();

	// store the sample so that it can be read through the device attribute
	mutex_lock(&priv->lock);
	priv->last_raw_sample = sample;
	mutex_unlock(&priv->lock);

	/* Report Readings to Input Device */
	input_report_abs(input, ABS_X, sample.x);
	input_report_abs(input, ABS_Y, sample.y);
	input_report_abs(input, ABS_Z, sample.z);
	input_report_abs(input, ABS_MISC, sample.temperature);
	input_sync(input);
}

/* Initialize the ALS31300 chip */
static int als31300_open(struct input_dev *input)
{
	struct als31300_priv *priv = input_get_drvdata(input);
	struct i2c_client *client = priv->client;
	int ret;
	u8 buf[4];

	/* Read Mode */
	ret = als31300_read_block(client, ALS31300_VOLATILE_MODE, buf,
				  sizeof(buf));
	if (ret < 0) {
		dev_err(&client->dev, "Read Mode failed, error %d\n", ret);
		return ret;
	}

	/* Set Mode to 20 Hz Low Power Sampling */
	buf[3] &= ~(ALS31300_LPDCM_MASK | ALS31300_MODE_MASK);
	buf[3] |= ALS31300_LPDCM_INACTIVE_50_0_MS;
	// TODO - LPDCM seems to cause the chip to be intermittently non-responsive during I2C transactions.
	// It is recoverable, but annoying
	// For now we will just use Active mode
	// buf[3] |= ALS31300_MODE_LPDCM;
	buf[3] |= ALS31300_MODE_ACTIVE;

	/* Write Mode */
	ret = als31300_write_block(client, ALS31300_VOLATILE_MODE, buf,
				   sizeof(buf));
	if (ret < 0) {
		dev_err(&client->dev, "Write Mode failed, error %d\n", ret);
		return ret;
	}

	return 0;
}

static void als31300_close(struct input_dev *input)
{
	struct als31300_priv *priv = input_get_drvdata(input);
	struct i2c_client *client = priv->client;
	int ret;
	u8 buf[4];

	/* Read Mode */
	ret = als31300_read_block(client, ALS31300_VOLATILE_MODE, buf,
				  sizeof(buf));
	if (ret < 0) {
		dev_err(&client->dev, "Read Mode failed, error %d\n", ret);
		return;
	}

	/* Set Mode to Sleep */
	buf[3] &= ~(ALS31300_MODE_MASK);
	buf[3] |= ALS31300_MODE_SLEEP;

	/* Write Mode */
	ret = als31300_write_block(client, ALS31300_VOLATILE_MODE, buf,
				   sizeof(buf));
	if (ret < 0) {
		dev_err(&client->dev, "Write Mode failed, error %d\n", ret);
		return;
	}
}

static ssize_t last_raw_sample_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct als31300_priv *priv = dev_get_drvdata(dev);
	struct als31300_sample sample;

	mutex_lock(&priv->lock);
	sample = priv->last_raw_sample;
	mutex_unlock(&priv->lock);

	return scnprintf(buf, PAGE_SIZE, "%llu;%d;%d;%d;%u\n", sample.timestamp,
			 sample.x, sample.y, sample.z, sample.temperature);
}

static DEVICE_ATTR_RO(last_raw_sample);

/*
 * I2C init/probing/exit functions
 */
static int als31300_probe(struct i2c_client *client)
{
	struct input_dev *input;
	struct als31300_priv *priv;
	int err;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &client->dev;
	priv->client = client;
	mutex_init(&priv->lock);
	memset(&priv->last_raw_sample, 0, sizeof(struct als31300_sample));

	err = sysfs_create_file(&priv->dev->kobj,
				&dev_attr_last_raw_sample.attr);
	if (err)
		return err;

	dev_set_drvdata(priv->dev, priv);

	input = devm_input_allocate_device(priv->dev);
	if (!input)
		return -ENOMEM;

	input_set_drvdata(input, priv);

	input->name = ALS31300_DRV_NAME;
	input->id.bustype = BUS_I2C;

	input->open = als31300_open;
	input->close = als31300_close;

	/* Limit the output change, in POC-C noise is higher than data sheet, probably from the system */
	/* TODO Set the fuzz to 0 if some user side averaging is performed */
	input_set_abs_params(input, ABS_X, -2048, 2047, 20, 0);
	input_set_abs_params(input, ABS_Y, -2048, 2047, 20, 0);
	input_set_abs_params(input, ABS_Z, -2048, 2047, 8, 0);
	input_set_abs_params(input, ABS_MISC, -40000, 85000, 100, 0);

	err = input_setup_polling(input, als31300_poll);
	if (err) {
		dev_err(&client->dev, "failed to set up polling\n");
		return err;
	}

	input_set_poll_interval(input, POLL_INTERVAL);
	input_set_max_poll_interval(input, POLL_INTERVAL_MAX);

	err = input_register_device(input);
	if (err) {
		dev_err(&client->dev, "failed to register input device\n");
		return err;
	}

	return 0;
}

static const struct i2c_device_id als31300_id[] = {
	{ ALS31300_DRV_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, als31300_id);

static const struct of_device_id als31300_dt_ids[] = {
	{
		.compatible = "allegro,als31300",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, als31300_dt_ids);

static struct i2c_driver als31300_driver = {
	.driver = {
		.name	= ALS31300_DRV_NAME,
		.of_match_table = als31300_dt_ids,
	},
	.probe		= als31300_probe,
	.id_table	= als31300_id,
};

module_i2c_driver(als31300_driver);

MODULE_DESCRIPTION("ALS31300 3-D Linear Hall Effect Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Todd Goyen <todd@tgoyen.com>");
