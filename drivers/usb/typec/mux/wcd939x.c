// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021-2022 Linaro Ltd.
 * Copyright (C) 2018-2020 The Linux Foundation
 */

#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>

#include "wcd-usbss-registers.h"
#include "wcd-usbss-reg-masks.h"
#include "wcd-usbss-reg-shifts.h"

#define AUXP_M_EN_MASK	(WCD_USBSS_SWITCH_SETTINGS_ENABLE_DP_AUXM_TO_MGX_SWITCHES_MASK |\
			WCD_USBSS_SWITCH_SETTINGS_ENABLE_DP_AUXP_TO_MGX_SWITCHES_MASK)

struct wcd939x {
	struct i2c_client *client;

	/* used to serialize concurrent change requests */
	struct mutex lock;

	struct typec_switch_dev *sw;
	struct typec_mux_dev *mux;

	enum typec_orientation orientation;
	unsigned long mode;
	unsigned int svid;

	u8 switch_settings_enable;
	u8 switch_select0;
};

static int wcd939x_write_reg(struct i2c_client *client, u16 reg, u8 value)
{
	struct i2c_msg msg[2];
	u8 data[4];
	int ret;

	msg[0].addr = client->addr;
	msg[0].len = 2;
	msg[0].flags = 0;
	msg[0].buf = &data[0];

	msg[1].addr = client->addr;
	msg[1].len = 2;
	msg[1].flags = 0;
	msg[1].buf = &data[2];

	data[0] = 0;
	data[1] = reg >> 8;
	data[2] = reg & 0xff;
	data[3] = value;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2)
		printk("wcd939x_write_reg failed %d\n", ret);
	return ret;
}

static int wcd939x_read_reg(struct i2c_client *client, u16 reg, u8 *value)
{
	struct i2c_msg msg[3];
	u8 data[3];
	int ret;

	msg[0].addr = client->addr;
	msg[0].len = 2;
	msg[0].flags = 0;
	msg[0].buf = &data[0];

	msg[1].addr = client->addr;
	msg[1].len = 1;
	msg[1].flags = 0;
	msg[1].buf = &data[2];

	msg[2].addr = client->addr;
	msg[2].len = 1;
	msg[2].flags = I2C_M_RD;
	msg[2].buf = value;

	data[0] = 0;
	data[1] = reg >> 8;
	data[2] = reg & 0xff;

	ret = i2c_transfer(client->adapter, msg, 3);
	if (ret != 3)
		printk("wcd939x_read_reg failed %d\n", ret);
	return ret;
}

static int wcd939x_set(struct wcd939x *fsa)
{
	u8 sel = fsa->switch_select0 & ~0xc0;
	u8 status;

	printk("wcd939x_set: %x %d\n", fsa->svid, fsa->orientation == TYPEC_ORIENTATION_REVERSE);

	if (fsa->svid) {
		// wcd_usbss_display_port_switch_update

		/* Disable AUX switches */
		wcd939x_write_reg(fsa->client, WCD_USBSS_SWITCH_SETTINGS_ENABLE, fsa->switch_settings_enable & ~AUXP_M_EN_MASK);

		/* Select MG1 for AUXP and MG2 for AUXM */
		if (fsa->orientation != TYPEC_ORIENTATION_REVERSE)
			sel |= 0x40;
		/* Select MG2 for AUXP and MG1 for AUXM */
		else
			sel |= 0x80;
		wcd939x_write_reg(fsa->client, WCD_USBSS_SWITCH_SELECT0, sel);

		/* Enable DP_AUXP_TO_MGX and DP_AUXM_TO_MGX switches */
		wcd939x_write_reg(fsa->client, WCD_USBSS_SWITCH_SETTINGS_ENABLE, fsa->switch_settings_enable | AUXP_M_EN_MASK);
	} else {
		/* Disable AUX switches */
		wcd939x_write_reg(fsa->client, WCD_USBSS_SWITCH_SELECT0, sel);
		wcd939x_write_reg(fsa->client, WCD_USBSS_SWITCH_SETTINGS_ENABLE, fsa->switch_settings_enable & ~AUXP_M_EN_MASK);
	}

	wcd939x_read_reg(fsa->client, WCD_USBSS_SWITCH_SETTINGS_ENABLE, &fsa->switch_settings_enable);
	wcd939x_read_reg(fsa->client, WCD_USBSS_SWITCH_SELECT0, &fsa->switch_select0);
	wcd939x_read_reg(fsa->client, WCD_USBSS_SWITCH_STATUS1, &status);
	printk("wcd939x state=%x %x %x\n", fsa->switch_settings_enable, fsa->switch_select0, status);

	return 0;
}

static int wcd939x_switch_set(struct typec_switch_dev *sw,
			      enum typec_orientation orientation)
{
	struct wcd939x *fsa = typec_switch_get_drvdata(sw);
	int ret = 0;

	mutex_lock(&fsa->lock);

	if (fsa->orientation != orientation) {
		fsa->orientation = orientation;

		ret = wcd939x_set(fsa);
	}

	mutex_unlock(&fsa->lock);

	return ret;
}

static int wcd939x_mux_set(struct typec_mux_dev *mux, struct typec_mux_state *state)
{
	struct wcd939x *fsa = typec_mux_get_drvdata(mux);
	int ret = 0;

	mutex_lock(&fsa->lock);

	if (fsa->mode != state->mode) {
		fsa->mode = state->mode;

		if (state->alt)
			fsa->svid = state->alt->svid;
		else
			fsa->svid = 0; // No SVID

		ret = wcd939x_set(fsa);
	}

	mutex_unlock(&fsa->lock);

	return ret;
}

static int wcd939x_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct typec_switch_desc sw_desc = { };
	struct typec_mux_desc mux_desc = { };
	struct wcd939x *fsa;
	u8 ver = 0;
	static int defer;

	// XXX wait for power on..
	if (defer++ < 5)
		return -EPROBE_DEFER;

	fsa = devm_kzalloc(dev, sizeof(*fsa), GFP_KERNEL);
	if (!fsa)
		return -ENOMEM;

	fsa->client = client;
	mutex_init(&fsa->lock);

	/* Safe mode */
	//fsa->cur_enable = FSA4480_ENABLE_DEVICE | FSA4480_ENABLE_USB;
	fsa->mode = TYPEC_STATE_SAFE;
	fsa->orientation = TYPEC_ORIENTATION_NONE;

	wcd939x_read_reg(fsa->client, WCD_USBSS_CHIP_ID0, &ver);
	dev_warn(dev, "WCD_USBSS_CHIP_ID0=%d\n", ver);
	wcd939x_read_reg(fsa->client, WCD_USBSS_CHIP_ID1, &ver);
	dev_warn(dev, "WCD_USBSS_CHIP_ID1=%d\n", ver);

	wcd939x_read_reg(fsa->client, WCD_USBSS_SWITCH_SETTINGS_ENABLE, &fsa->switch_settings_enable);
	wcd939x_read_reg(fsa->client, WCD_USBSS_SWITCH_SELECT0, &fsa->switch_select0);
	dev_warn(dev, "fsa->switch_settings_enable=%x %x\n", fsa->switch_settings_enable, fsa->switch_select0);

#if 0
	// wcd_usbss_update_reg_init

	//regmap_update_bits(fsa->regmap, WCD_USBSS_FUNCTION_ENABLE, 0x03, 0x02); /* AUDIO_FSM mode */
	regmap_update_bits(fsa->regmap, WCD_USBSS_FUNCTION_ENABLE, 0x03, 0x01); /* AUDIO_MANUAL mode */

	/* Enable dynamic boosting for DP and DN */
	regmap_update_bits(fsa->regmap, WCD_USBSS_DP_DN_MISC1, 0x09, 0x09);
	/* Enable dynamic boosting for MG1 OVP */
	regmap_update_bits(fsa->regmap, WCD_USBSS_MG1_MISC, 0x20, 0x20);
	/* Enable dynamic boosting for MG2 OVP */
	regmap_update_bits(fsa->regmap, WCD_USBSS_MG2_MISC, 0x20, 0x20);
	/* Disable Equalizer */
	regmap_update_bits(fsa->regmap, WCD_USBSS_EQUALIZER1, WCD_USBSS_EQUALIZER1_EQ_EN_MASK, 0x00);
	/* For surge reset routine: Write WCD_USBSS_CPLDO_CTL2 --> 0xFF */
	regmap_update_bits(fsa->regmap, WCD_USBSS_CPLDO_CTL2, 0xFF, 0xFF);

	// wcd_usbss_switch_update_defaults

	/* Disable all switches */
	regmap_update_bits(fsa->regmap, WCD_USBSS_SWITCH_SETTINGS_ENABLE, 0x7F, 0x00);
	/* Select MG1 for AGND_SWITCHES */
	regmap_update_bits(fsa->regmap, WCD_USBSS_SWITCH_SELECT1, 0x01, 0x00);
	/* Select GSBU1 and MG1 for MIC_SWITCHES */
	regmap_update_bits(fsa->regmap, WCD_USBSS_SWITCH_SELECT0, 0x03, 0x00);
	/* Enable OVP_MG1_BIAS PCOMP_DYN_BST_EN */
	regmap_update_bits(fsa->regmap, WCD_USBSS_MG1_BIAS, 0x08, 0x08);
	/* Enable OVP_MG2_BIAS PCOMP_DYN_BST_EN */
	regmap_update_bits(fsa->regmap, WCD_USBSS_MG2_BIAS, 0x08, 0x08);
	regmap_update_bits_base(fsa->regmap, WCD_USBSS_AUDIO_FSM_START, 0x01, 0x01, NULL, false, true);
	/* Select DN for DNL_SWITHCES and DP for DPR_SWITCHES */
	regmap_update_bits(fsa->regmap, WCD_USBSS_SWITCH_SELECT0, 0x3C, 0x14);
	/* Enable DNL_SWITCHES and DPR_SWITCHES */
	regmap_update_bits(fsa->regmap, WCD_USBSS_SWITCH_SETTINGS_ENABLE, 0x18, 0x18);
	/* Disable Equalizer */
	regmap_update_bits(fsa->regmap, WCD_USBSS_EQUALIZER1, WCD_USBSS_EQUALIZER1_EQ_EN_MASK, 0x00);
	regmap_update_bits(fsa->regmap, WCD_USBSS_USB_SS_CNTL, 0x07, 0x05); /* Mode5: USB*/
	//if (wcd_usbss_ctxt_->version == WCD_USBSS_2_0)
	//	regmap_update_bits(fsa->regmap, WCD_USBSS_PMP_OUT1,
	//			0x40, 0x00);

	// ...

	/* Disable AUX switches */
	regmap_update_bits(fsa->regmap, WCD_USBSS_SWITCH_SETTINGS_ENABLE, AUXP_M_EN_MASK, 0x00);

	/* Update Leakage Canceller Coefficient for AUXP pins */
	regmap_update_bits(fsa->regmap, WCD_USBSS_DISP_AUXP_CTL, 0x07, 0x05);
	regmap_update_bits(fsa->regmap, WCD_USBSS_DISP_AUXP_THRESH, 0xE0, 0xE0);
#endif

	sw_desc.drvdata = fsa;
	sw_desc.fwnode = dev_fwnode(dev);
	sw_desc.set = wcd939x_switch_set;

	fsa->sw = typec_switch_register(dev, &sw_desc);
	if (IS_ERR(fsa->sw))
		return dev_err_probe(dev, PTR_ERR(fsa->sw), "failed to register typec switch\n");

	mux_desc.drvdata = fsa;
	mux_desc.fwnode = dev_fwnode(dev);
	mux_desc.set = wcd939x_mux_set;

	fsa->mux = typec_mux_register(dev, &mux_desc);
	if (IS_ERR(fsa->mux)) {
		typec_switch_unregister(fsa->sw);
		return dev_err_probe(dev, PTR_ERR(fsa->mux), "failed to register typec mux\n");
	}

	i2c_set_clientdata(client, fsa);
	return 0;
}

static void wcd939x_remove(struct i2c_client *client)
{
	struct wcd939x *fsa = i2c_get_clientdata(client);

	typec_mux_unregister(fsa->mux);
	typec_switch_unregister(fsa->sw);
}

static const struct i2c_device_id wcd939x_table[] = {
	{ "wcd939x" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wcd939x_table);

static const struct of_device_id wcd939x_of_table[] = {
	{ .compatible = "qcom,wcd939x-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, wcd939x_of_table);

static struct i2c_driver wcd939x_driver = {
	.driver = {
		.name = "wcd939x",
		.of_match_table = wcd939x_of_table,
	},
	.probe		= wcd939x_probe,
	.remove		= wcd939x_remove,
	.id_table	= wcd939x_table,
};
module_i2c_driver(wcd939x_driver);

MODULE_DESCRIPTION("ON Semiconductor FSA4480 driver");
MODULE_LICENSE("GPL v2");
