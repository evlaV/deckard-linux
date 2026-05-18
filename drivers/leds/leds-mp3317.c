// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MP3317 Led driver
 *
 * Copyright 2024 Monolithic Power Systems, Inc
 *
 * Author: Yuxi Wang <Yuxi.Wang@monolithicpower.com>
 */
#include <linux/bits.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/led-class-multicolor.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/suspend.h>

#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>

#include <linux/regulator/machine.h>

#define MP3317_MODE_CTL 0
#define MP3317_EN_CTL 1
#define MP3317_OVP_IMAX 0x02
#define MP3317_TPWM 0x03
#define MP3317_ILEDL_L 0x04
#define MP3317_ILEDL_H 0x05
#define MP3317_ILEDR_L 0x06
#define MP3317_ILEDR_H 0x07
#define MP3317_VOUT 0x08
#define MP3317_HR_TH 0x09
#define MP3317_HR_STEP 0x0A
#define MP3317_FAULT1 0x0B
#define MP3317_FAULT2 0x0C
#define MP3317_REG08_SET_VOUT 0x08

#define EN_MASK BIT(7)
#define CURRENT_STATE_MASK BIT(6)
#define BLMOD_MASK BIT(7)
#define CPHYST_MASK GENMASK(7,6)
#define IMAX_MASK GENMASK(2,0)
#define HR_L_MASK GENMASK(3,0)
#define TPWM_MASK GENMASK(3,0)
#define CBC_LIMIT_MASK GENMASK(5,3)
#define OVP_MASK GENMASK(5,3)
#define FAULT1_MASK GENMASK(6,0)
#define FAULT2_MASK GENMASK(2,0)
#define FSW_MASK GENMASK(3,2)
#define INCL_MASK GENMASK(2,0)

#define MAX_BRIGHTNESS 4095

// MODE_CTL (0x00) - b1000 1001
//    BLMOD - (b1)   - Adaptive control enabled
//    PWMLR - (b0)   - L for CH1-3, R for CH4-6
//    CBCL  - (b011) - 0.9A cycle-by-cycle limit
//    INCL  - (b001) - 0.8A input current limit
// #define CONFIG_MODE_CL_DEFAULT          0x89
#define CONFIG_MODE_CTL_DEFAULT          0x9E

// EN_CTRL (0x01)
//    EN - Chip Enable
//    ISET_EN - Sets ILED current to internal
//    CHEN - Enable channels 1,2,4,5
#define CONFIG_EN_CTRL_DEFAULT          0x9B

// OVP_MAX (0x02) - b0100 1111
//   DMSL    - (b0)    - Constant voltage mode
//   SBY_PWM - (b0)    - Disable standby
//   OVP     - (b001)  - 21V
//   IMAX    - (b111)  - 160mA max current
#define CONFIG_OVP_MAX_DEFAULT          0x0F
// #define CONFIG_OVP_MAX_DEFAULT          0x57 // OVP = 010 (25V)
// #define CONFIG_OVP_MAX_DEFAULT          0x5F // OVP = 011 (30V)
// #define CONFIG_OVP_MAX_DEFAULT          0x7F // OVP = 111 (34V)

// TPWM (0x03) - bxxxx 1010
//   TPWM    - (b1010) - 5ms max PWM on time
#define CONFIG_TPWM_DEFAULT             0x0A

// ILEDL_H/L (0x05/0x04) - bxxxx 0100 1111 1111
//   ILEDL   - b0100 1111 1111 - Left current at 100mA of 160mA max
#define CONFIG_ILEDL_H_DEFAULT          0x04
#define CONFIG_ILEDL_L_DEFAULT          0xFF

// ILEDR_H/L (0x07/0x06) - bxxxx 0100 1111 1111
//   ILEDR   - b0100 1111 1111 - Right current at 100mA of 160mA max
#define CONFIG_ILEDR_H_DEFAULT          0x04
#define CONFIG_ILEDR_L_DEFAULT          0xFF

// VOUT (0x08) - b0001 1000
//   VOUT    - (b0001 1000) - VOUT = 9.544V
#define CONFIG_VOUT_DEFAULT             0x18
// Can set VOUT with this voltage define. Comment this to use CONFIG_VOUT_DEFAULT directly
#define CONFIG_VOUT_MV                  9500

// HR_TH (0x09) - b0100 1001
//   CPYHyst - (b00)   - 750mV headroom
//   EN_INUV - (b1)    - UVLO enabled
//   TCOMP   - (b0)    - 20msec Short-circuit comparator
//   FSW     - (b10)   - 1.25MHz switch frequency
//   LEDS    - (b01)   - 5V short-circuit
#define CONFIG_HR_TH_DEFAULT            0x49

// STEP_HR (0x0A) - b0100 0001
//   Step_UP - (b01)   - 210mV Step-up size
//   Step_DWN- (b00)   - 105mV Step-down size
//   HeadR_L - (b0001) - 375mV comparison threshold
#define CONFIG_STEP_HR_DEFAULT          0x41

uint8_t MP3317_DEFAULT_MAP[] = 
{
	CONFIG_MODE_CTL_DEFAULT,
	CONFIG_EN_CTRL_DEFAULT,
	CONFIG_OVP_MAX_DEFAULT,
	CONFIG_TPWM_DEFAULT,
	CONFIG_ILEDL_L_DEFAULT,
	CONFIG_ILEDL_H_DEFAULT,
	CONFIG_ILEDR_L_DEFAULT,
	CONFIG_ILEDR_H_DEFAULT,
	CONFIG_VOUT_DEFAULT,
	CONFIG_HR_TH_DEFAULT,
	CONFIG_STEP_HR_DEFAULT
};

struct mp3317 {
	struct i2c_client *client;
	struct led_classdev wled;
	struct regmap *regmap;
	struct gpio_desc *en_gpio;
	struct regulator_desc desc;
	unsigned long rst;
    u8 default_map[sizeof(MP3317_DEFAULT_MAP)];
    unsigned int use_pm_ops;
};

static ssize_t reg_dump_show(struct device *dev, struct device_attribute *attr, char *buf);

static struct device_attribute dev_attr_led_state;

static bool mp3317_verify_config(struct mp3317 *led)
{
	// Verify all registers are configured to desired defaults
	bool config = true;
	unsigned int reg, rval;
	int ret;
	for(reg = 0; reg < sizeof(MP3317_DEFAULT_MAP); reg++)
	{
		ret = regmap_read(led->regmap, reg, &rval);
		if(ret)
		{
			dev_dbg(&led->client->dev, "could not read %d ( %d )\n", reg, ret);
			config = false;
		}
		if(rval != led->default_map[reg])
		{
			dev_dbg(&led->client->dev, "config failed on %d ( %x != %x )\n", reg, rval, led->default_map[reg]);
			config = false;
			break;
		}
	}

	return config;
}

/* Configure and enable the MP3317 with the desired settings from dts node */
static int mp3317_config(struct mp3317 *led)
{
	struct device *dev = &led->client->dev;
	int ret;
	uint rval;

	// get driver defaults
	memcpy(led->default_map, MP3317_DEFAULT_MAP, sizeof(MP3317_DEFAULT_MAP));

	// Disable MP3317 before setting config, just disable all EN bits for config
	// ret = regmap_update_bits(led->regmap, MP3317_EN_CTL,EN_MASK, 0);
	ret = regmap_write(led->regmap, MP3317_EN_CTL, 0);
	if(ret)
	{
		dev_err(dev, "MP3317_EN_CTL disable error\n");
		return ret;
	}

	// Clear BLMOD_MASK before updating VOUT
	ret =  regmap_update_bits(led->regmap, MP3317_MODE_CTL, BLMOD_MASK, 0);
	if(ret)
	{
		dev_err(dev, "MP3317_MODE_CTL->BLMOD_MASK disable error\n");
		return ret;
	}

	// VOUT
	uint vout;
	if ( device_property_read_u32(dev, "mp3317,vout",
				 &vout) ) {
		vout = CONFIG_VOUT_DEFAULT;
	} else {
		if(vout < 7000){
			vout = 0;
			// ret = regmap_write(led->regmap, MP3317_VOUT, 0);
		}else if(vout > 34030){
			vout = 0xFF;
			// ret = regmap_write(led->regmap, MP3317_VOUT, 0xff);
		}else{
			vout = (vout - 7000 + 106 / 2) / 106;
			// ret = regmap_write(led->regmap, MP3317_VOUT, vout);
		}
	}

	led->default_map[MP3317_VOUT] = vout;

	// Apply VOUT
	ret = regmap_write(led->regmap, MP3317_VOUT, vout);
	if(ret)
	{
		dev_err(dev, "MP3317_VOUT write error\n");
		return ret;
	}

	// OVP_MAX
	uint ovp_max;
	if ( device_property_read_u32(dev, "mp3317,ovp-max",
				 &ovp_max) ) {
		ovp_max = CONFIG_OVP_MAX_DEFAULT;
	}
	ret = regmap_write(led->regmap, MP3317_OVP_IMAX, ovp_max);
	led->default_map[MP3317_OVP_IMAX] = ovp_max;
	if(ret)
	{
		dev_err(dev, "MP3317_OVP_IMAX update error\n");
		return ret;
	}

	// TPWM
	uint tpwm;
	if ( device_property_read_u32(dev, "mp3317,tpwm",
				 &tpwm) ) {
		tpwm = CONFIG_TPWM_DEFAULT;
	}
	ret = regmap_write(led->regmap, MP3317_TPWM, tpwm);
	led->default_map[MP3317_TPWM] = tpwm;
	if(ret)
	{
		dev_err(dev, "MP3317_TPWM update error\n");
		return ret;
	}

	// ILEDL_L - Write LSB (L) before MSB (H)
	uint iledl;
	if ( device_property_read_u32(dev, "mp3317,iledl",
				 &iledl) ) {
		iledl = CONFIG_ILEDL_H_DEFAULT << 8 | CONFIG_ILEDL_L_DEFAULT;
	}
	ret = regmap_write(led->regmap, MP3317_ILEDL_L, iledl & 0x00FF);
	led->default_map[MP3317_ILEDL_L] = iledl & 0x00FF;
	if(ret)
	{
		dev_err(dev, "MP3317_ILEDL_L update error\n");
		return ret;
	}

	// ILEDL_H
	ret = regmap_write(led->regmap, MP3317_ILEDL_H, ( iledl & 0xFF00 ) >> 8);
    regmap_read(led->regmap, MP3317_ILEDL_H, &rval);
	led->default_map[MP3317_ILEDL_H] = (( iledl & 0xFF00 ) >> 8);
	if(ret)
	{
		dev_err(dev, "MP3317_ILEDL_H update error\n");
		return ret;
	}

	// ILEDL_R - Write LSB (L) before MSB (H)
	uint iledr;
	if ( device_property_read_u32(dev, "mp3317,iledr",
				 &iledr) ) {
		iledr = CONFIG_ILEDR_H_DEFAULT << 8 | CONFIG_ILEDR_L_DEFAULT;
	}
	ret = regmap_write(led->regmap, MP3317_ILEDR_L, iledr & 0x00FF);
	led->default_map[MP3317_ILEDR_L] = iledr & 0x00FF;
	if(ret)
	{
		dev_err(dev, "MP3317_ILEDL_R update error\n");
		return ret;
	}

	// ILEDR_H
	ret = regmap_write(led->regmap, MP3317_ILEDR_H, ( iledr & 0xFF00 ) >> 8);
	led->default_map[MP3317_ILEDR_H] = (( iledr & 0xFF00 ) >> 8);
	if(ret)
	{
		dev_err(dev, "MP3317_ILEDR_H update error\n");
		return ret;
	}

	// HR_TH
	uint hr;
	if ( device_property_read_u32(dev, "mp3317,hr",
				 &hr) ) {
		hr = CONFIG_HR_TH_DEFAULT;
	}
	ret = regmap_write(led->regmap, MP3317_HR_TH, hr);
	led->default_map[MP3317_HR_TH] = hr;
	if(ret)
	{
		dev_err(dev, "MP3317_HR_TH update error\n");
		return ret;
	}

	// STEP_HR
	uint step_hr;
	if ( device_property_read_u32(dev, "mp3317,step-hr",
				 &step_hr) ) {
		step_hr = CONFIG_STEP_HR_DEFAULT;
	}
	ret = regmap_write(led->regmap, MP3317_HR_STEP, step_hr);
	led->default_map[MP3317_HR_STEP] = step_hr;
	if(ret)
	{
		dev_err(dev, "MP3317_HR_STEP update error\n");
		return ret;
	}

	// Re-enable MP3317 with our desired mode control and enable fields LAST
	// MODE_CTL
	uint mode_ctl;
	if ( device_property_read_u32(dev, "mp3317,mode-ctl",
				 &mode_ctl) ) {
		mode_ctl = CONFIG_MODE_CTL_DEFAULT;
	}
	ret = regmap_write(led->regmap, MP3317_MODE_CTL, mode_ctl);
	led->default_map[MP3317_MODE_CTL] = mode_ctl;
	if(ret)
	{
		dev_err(dev, "MP3317_MODE_CTL update error\n");
		return ret;
	}

	// EN_CTL
	uint en_ctl;
	if ( device_property_read_u32(dev, "mp3317,en-ctl",
				 &en_ctl) ) {
		en_ctl = CONFIG_EN_CTRL_DEFAULT;
	}
	ret = regmap_write(led->regmap, MP3317_EN_CTL, en_ctl);
	led->default_map[MP3317_EN_CTL] = en_ctl;
	if(ret)
	{
		dev_err(dev, "MP3317_EN_CTL update error\n");
		return ret;
	}

	return mp3317_verify_config(led) ? 0 : -EIO;
}

static int set_headroom(struct mp3317 *led, int val)
{
	int ret;

	ret = regmap_update_bits(led->regmap, MP3317_HR_STEP, HR_L_MASK, val);
	if(ret)
		return ret;

	return 0;
}

static int set_cphyst(struct mp3317 *led, int val)
{
	int ret;

	ret = regmap_update_bits(led->regmap, MP3317_HR_TH, CPHYST_MASK, val);
	if(ret)
		return ret;

	return 0;
}

static int set_cur_state(struct mp3317 *led, int val)
{
	int ret;

	ret = regmap_update_bits(led->regmap, MP3317_EN_CTL, CURRENT_STATE_MASK, val);
	if(ret)
		return ret;

	return 0;
}

static int get_max_current(struct mp3317 *led)
{
	int ret;
	uint rval;

	ret = regmap_read(led->regmap, MP3317_OVP_IMAX, &rval);
	if(ret)
		return ret;
	
	rval &= IMAX_MASK;

	switch (rval) {
	case 0:
		return  5;
	case 1:
		return 10;
	case 2:
		return 20;
	case 3:
		return 40;
	case 4:
		return 80;
	default:
		return 160;
	}
}

static int set_max_current(struct mp3317 *led, int val)
{
	int ret;
	
	if(val < 5) {
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, IMAX_MASK, 0);
	}else if(val < 10){
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, IMAX_MASK, 0x01);  
	}else if (val < 20){
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, IMAX_MASK, 0x02);   
	}else if(val < 40){
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, IMAX_MASK, 0x03);  
	}else if(val < 80){
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, IMAX_MASK, 0x04);  
	}else{
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, IMAX_MASK, 0x07);         
	}

	if(ret)
		return ret;
	return 0;
}

static ssize_t led_boost_mode_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	uint rval;
	
	ret = regmap_read(led->regmap, MP3317_MODE_CTL, &rval);
	if(ret)
		return ret;
	
	rval &= BLMOD_MASK;

	if(rval)
		return sysfs_emit(buf, "%s\n", "Adaptive");
	else
		return sysfs_emit(buf, "%s\n", "Manual");
}

/*
 *  Input value > 0, enable this mode. Input value = 0, disable this mode. 
 */
static ssize_t led_boost_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	u8 val;

	ret = kstrtou8(buf, 10, &val);
	if(ret)
		return ret;
	
	if(val)
		ret = regmap_update_bits(led->regmap, MP3317_MODE_CTL, BLMOD_MASK, 0xff);
	else
		ret = regmap_update_bits(led->regmap, MP3317_MODE_CTL, BLMOD_MASK, 0);
	if(ret)
		return ret;

	return count;
}

static ssize_t led_vout_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	uint rval;
	int val;

	ret = regmap_read(led->regmap, MP3317_VOUT, &rval);
	if(ret)
		return ret;

	//0.106
	val = 106 * rval + 7000;

	return sysfs_emit(buf, "%dmV\n", val);
}

/*
 *    Input unit: mV
 */
static ssize_t led_vout_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 10, &val);
	if(ret)
		return ret;

	if(val < 7000) 
		ret = regmap_write(led->regmap, MP3317_VOUT, 0);
	else if(val > 34030)
		ret = regmap_write(led->regmap, MP3317_VOUT, 0xff);
	else{
		val = (val - 7000 + 106 / 2) / 106;
		ret = regmap_write(led->regmap, MP3317_VOUT, val);
	}

	printk("Set new vout voltage to %ld\n", val);

	if(ret) return ret;
	return count;
}

static DEVICE_ATTR_RW(led_boost_mode);
static DEVICE_ATTR_RW(led_vout);

static struct attribute *vout_set_attrs[] = {
	&dev_attr_led_boost_mode.attr,
	&dev_attr_led_vout.attr,
	NULL,
};

static struct attribute_group Vout_Set = {
		.name = "Vout Set",
		.attrs = vout_set_attrs
};

static ssize_t led_current_state_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	uint rval;
	
	ret = regmap_read(led->regmap, MP3317_EN_CTL, &rval);
	if(ret)
		return ret;
	
	rval &= CURRENT_STATE_MASK;

	if(rval)
		return sysfs_emit(buf, "%s\n", "Enable");
	else
		return sysfs_emit(buf, "%s\n", "Disable");

}
/*
 *  Input value > 0, enable . Input value = 0, disable . 
 */
static ssize_t led_current_state_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	u8 val;
	
	ret = kstrtou8(buf, 10, &val);
	if(ret)
		return ret;
	
	if(val)
		ret = regmap_update_bits(led->regmap, MP3317_EN_CTL, CURRENT_STATE_MASK, 0xff);
	else
		ret = regmap_update_bits(led->regmap, MP3317_EN_CTL, CURRENT_STATE_MASK, 0);
	
	if(ret)
		return ret;

	return count;
}

static ssize_t led_max_current_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	uint rval;

	ret = regmap_read(led->regmap, MP3317_OVP_IMAX, &rval);
	if(ret)
		return ret;

	rval &= IMAX_MASK;

	switch (rval) {
	case 0:
		return sysfs_emit(buf, "%dmA\n", 5);
	case 1:
		return sysfs_emit(buf, "%dmA\n", 10);
	case 2:
		return sysfs_emit(buf, "%dmA\n", 20);
	case 3:
		return sysfs_emit(buf, "%dmA\n", 40);
	case 4:
		return sysfs_emit(buf, "%dmA\n", 80);
	default:
		return sysfs_emit(buf, "%dmA\n", 160);
	}
}

/*
 * Input mA
 */
static ssize_t led_max_current_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	u8 val;

	ret = kstrtou8(buf, 10, &val);
	if(ret)
		return ret;

	if(val < 5){
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, IMAX_MASK, 0);
	}else if(val < 10){
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, IMAX_MASK, 0x01);  
	}else if (val < 20){
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, IMAX_MASK, 0x02);   
	}else if(val < 40){
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, IMAX_MASK, 0x03);  
	}else if(val < 80){
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, IMAX_MASK, 0x04);  
	}else{
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, IMAX_MASK, 0x05);         
	}

	if(ret)
		return ret;
	return count;
}

static ssize_t led_left_current_show(struct device *dev,
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	int max_current;
	uint rval;
	int val;

	ret = regmap_read(led->regmap, MP3317_ILEDL_H, &rval);
	if(ret)
		return ret;
	val = rval & 0x0f;
	ret = regmap_read(led->regmap, MP3317_ILEDL_L, &rval);
	if(ret)
		return ret;
	val = val << 8;
	val |= rval;

	max_current = get_max_current(led);

	return sysfs_emit(buf, "%dmA\n", (val  * max_current + 4096 / 2) /4096);
}

static ssize_t led_right_current_show(struct device *dev,
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	int max_current;
	uint rval;
	int val;

	ret = regmap_read(led->regmap, MP3317_ILEDR_H, &rval);
	if(ret)
		return ret;
	val = rval & 0x0f;
	ret = regmap_read(led->regmap, MP3317_ILEDR_L, &rval);
	if(ret)
		return ret;
	val = val << 8;
	val |= rval;

	max_current = get_max_current(led);
	
	return sysfs_emit(buf, "%dmA\n", (val  * max_current + 4096 / 2) /4096);
}
/*
 * Input mA
 */
static ssize_t led_current_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	int max_current;
	int left_val, right_val, val;

	ret = sscanf(buf, "%d %d", &left_val, &right_val);
	if(ret != 2){
		dev_err(dev, "invalid params for set current <left> <right> ( %d )\n", ret);
		return -EINVAL;
	}

	val = left_val > right_val ? left_val : right_val;

	if(val < 60)
		set_headroom(led, 0);
	else if (val < 100)
		set_headroom(led, 1);
	else if (val < 140)
		set_headroom(led, 2);
	else if (val < 180)
		set_headroom(led, 3);
	else if (val < 220)
		set_headroom(led, 4);
	else if (val < 260)
		set_headroom(led, 5);
	else if (val < 300)
		set_headroom(led, 6);
	else if (val < 340)
		set_headroom(led, 7);
	else if (val < 380)
		set_headroom(led, 8);
	else if (val < 420)
		set_headroom(led, 9);
	else if (val < 460)
		set_headroom(led, 10);
	else if (val < 500)
		set_headroom(led, 11);
	else if (val < 540)
		set_headroom(led, 12);
	else if (val < 580)
		set_headroom(led, 13);
	else if (val < 620)
		set_headroom(led, 14);
	else
		set_headroom(led, 15);

	if(val < 160) {
		ret = set_cur_state(led, 0);
		if(ret)
			return ret;

		ret = set_max_current (led, val);
		if(ret)
			return ret;
		
		max_current = get_max_current(led);
		if(max_current < 0)
			return max_current;

		ret = set_cphyst(led, 0xC0);
		if(ret)
			return ret;

		max_current *=1000;
		val = (val * 1000 * 4096 + max_current /2) / max_current;
		
	}else {
		ret = set_cur_state(led, 0xff);
		if(ret)
			return ret;

		ret = set_max_current (led, 160);
		if(ret)
			return ret;
		ret = set_cphyst(led, 0x80);
		if(ret)
			return ret;

		max_current = (160 * 60 * 1000  + led->rst / 2) / led->rst;

		val = (val * 1000 * 4096 + (max_current* 1000) /2) / (max_current* 1000);
	}

	ret = regmap_write(led->regmap, MP3317_ILEDL_L, left_val & 0xff);
	if(ret)
		return ret;
	ret = regmap_write(led->regmap, MP3317_ILEDL_H, (left_val & 0xf00) >> 8);
	if(ret)
		return ret;

	ret = regmap_write(led->regmap, MP3317_ILEDR_L, right_val & 0xff);
	if(ret)
		return ret;
	ret = regmap_write(led->regmap, MP3317_ILEDR_H, (right_val & 0xf00) >> 8);
	if(ret)
		return ret;

	return count;
}

static ssize_t led_cphyst_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	uint rval;

	ret = regmap_read(led->regmap, MP3317_HR_TH, &rval);
	if(ret)
		return ret;
	
	rval &= CPHYST_MASK;

	switch (rval) {
	case 0:
		return sysfs_emit(buf, "%dmV\n", 1000);
	case 0x40:
		return sysfs_emit(buf, "%dmV\n", 750);
	case 0x80:
		return sysfs_emit(buf, "%dmV\n", 500);
	case 0xC0:
		return sysfs_emit(buf, "%dmV\n", 250);
	}

	return 0;
}
/*
 * Input range 0~3
 * 0: 1000mV; 1: 750mV; 2:500mV; 3:250mV
 */
static ssize_t led_cphyst_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	u8 val;

	ret = kstrtou8(buf, 10, &val);
	if(ret)
		return ret;

	ret = set_cphyst(led, val << 6);
	if(ret)
		return ret;

	return count;        
}

static ssize_t led_headroom_l_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	uint rval;
	u16 val;

	ret = regmap_read(led->regmap, MP3317_HR_STEP, &rval);
	if(ret)
		return ret;
	
	rval &= HR_L_MASK;

	val = 125 * rval + 250;

	return sysfs_emit(buf, "%dmV\n", val);
   
}
/*
 * Input range 0~15
 * 0: 250mV; 1: 375mV; 2:500mV; 3:625mV; 4: 750mV; 5: 875mV; 6: 1000mV; 7:1125mV
 * 8: 1250mV; 9: 1375mV; 10: 1500mV; 11: 1625mV; 12: 1750mV; 13: 1875mV; 14: 2000mV; 15: 2125mV
 */
static ssize_t led_headroom_l_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	u8 val;	

	ret = kstrtou8(buf, 10, &val);
	if(ret)
		return ret;
	
	ret = set_headroom(led, val & 0x0f);
	if(ret)
		return ret;

	return count;
}

static ssize_t led_rst_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);

	return sysfs_emit(buf, "%ldΩ\n", led->rst);
}

/*
 * Input ohm
 */
static ssize_t led_rst_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	unsigned long val;
	u8 ret;
	
	ret = kstrtoul(buf, 10 , &val);
	if(ret)
		return ret;

	led->rst = val;
	return count;
}

static DEVICE_ATTR_RW(led_current_state);
static DEVICE_ATTR_RW(led_max_current);
static DEVICE_ATTR_WO(led_current);
static DEVICE_ATTR_RO(led_left_current);
static DEVICE_ATTR_RO(led_right_current);
static DEVICE_ATTR_RW(led_cphyst);
static DEVICE_ATTR_RW(led_headroom_l);
static DEVICE_ATTR_RW(led_rst);

static struct attribute *current_attrs[] = {
	&dev_attr_led_current_state.attr,
	&dev_attr_led_max_current.attr,
	&dev_attr_led_current.attr,
	&dev_attr_led_left_current.attr,
	&dev_attr_led_right_current.attr,
	&dev_attr_led_cphyst.attr,
	&dev_attr_led_headroom_l.attr,
	&dev_attr_led_rst.attr,
	NULL,
};

static const struct attribute_group current_set = {
		.name = "Current Set",
		.attrs = current_attrs
};

static ssize_t led_fsw_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	uint rval;

	ret = regmap_read(led->regmap, MP3317_HR_TH, &rval);
	if(ret)
		return ret;
	
	rval &= FSW_MASK;
	printk("%d\r\n",rval);
	switch (rval) {
	case 0:
		return sysfs_emit(buf, "%dkHz\n", 312);
	case 0x04:
		return sysfs_emit(buf, "%dkHz\n", 625);
	case 0x08:
		return sysfs_emit(buf, "%dkHz\n", 1250);
	default:
		return sysfs_emit(buf, "%s\n", "NC");
	}
}

/*
 *  Input: 0~3
 *  0:312kHz   1:625kHz   2:1250kHz
 */
static ssize_t led_fsw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	u8 val;

	ret = kstrtou8(buf, 10, &val);
	if (ret) 
		return ret;

	switch (val) {
	case 0:
		ret = regmap_update_bits(led->regmap, MP3317_HR_TH, FSW_MASK, 0); 
		break;
	case 1:
		ret = regmap_update_bits(led->regmap, MP3317_HR_TH, FSW_MASK, 1<<2); 
		break;
	case 2:
		ret = regmap_update_bits(led->regmap, MP3317_HR_TH, FSW_MASK, 1<<3); 
		break;
	default:
		break;
	}

	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR_RW(led_fsw);

static ssize_t led_config_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	bool config;

	config = mp3317_verify_config(led);

	if(!config)
	{
		return sysfs_emit(buf, "Config not applied\n");
	}
	else
	{
		return sysfs_emit(buf, "Config already applied\n");
	}
}

static ssize_t led_config_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);

	int ret;
	u8 val;

	ret = kstrtou8(buf, 10, &val);
	if(ret) return ret;

	// If val !=0, apply the MP3317 configuration
	if(val)
	{
		ret = mp3317_config(led);
		if(ret) return ret;
	}
	else
	{
		// Do nothing
	}

	return count;
}
static DEVICE_ATTR_RW(led_config);

static ssize_t led_state_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	uint rval;

	if ( !gpiod_get_value(led->en_gpio) ) {
		return sysfs_emit(buf, "Disable\n");
	}

	ret = regmap_read(led->regmap, MP3317_EN_CTL, &rval);
	if(ret)
		return ret;
	rval &= EN_MASK;

	if(rval){
		return sysfs_emit(buf, "Enable\n");
	}else{
		return sysfs_emit(buf, "Disable\n");
	}
}

/*
 *  0:Disable; other: Enable
 */
static ssize_t led_state_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	u8 val;

	// If in use a kernel regulator, don't allow enable/disable from userspace
	if ( led->desc.ops ) {
		return -EIO;
	}

	ret = kstrtou8(buf, 10, &val);
	if(ret)
		return ret;
	
	if (val && !gpiod_get_value(led->en_gpio)) {
		gpiod_set_value(led->en_gpio, 1);
		usleep_range(10000,20000);
		mp3317_config(led);

		sysfs_notify(&dev->kobj, NULL, dev_attr_led_state.attr.name);

	} else if (!val && gpiod_get_value(led->en_gpio)) {
		ret = regmap_update_bits(led->regmap, MP3317_EN_CTL, EN_MASK, 0);
		gpiod_set_value(led->en_gpio, 0);

		sysfs_notify(&dev->kobj, NULL, dev_attr_led_state.attr.name);
	}

	if(ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(led_state);

static ssize_t reg_dump_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	uint rval;
	int at = 0;

	regmap_read(led->regmap, MP3317_MODE_CTL, &rval);
	at += sysfs_emit_at(buf, at, "MP3317_MODE_CTL: %x\n", rval);

	regmap_read(led->regmap, MP3317_EN_CTL, &rval);
	at += sysfs_emit_at(buf, at, "MP3317_EN_CTL: %x\n", rval);

	regmap_read(led->regmap, MP3317_OVP_IMAX, &rval);
	at += sysfs_emit_at(buf, at, "MP3317_OVP_IMAX: %x\n", rval);

	regmap_read(led->regmap, MP3317_TPWM, &rval);
	at += sysfs_emit_at(buf, at, "MP3317_TPWM: %x\n", rval);

	regmap_read(led->regmap, MP3317_ILEDL_L, &rval);
	at += sysfs_emit_at(buf, at, "MP3317_ILEDL_L: %x\n", rval);

	regmap_read(led->regmap, MP3317_ILEDL_H, &rval);
	at += sysfs_emit_at(buf, at, "MP3317_ILEDL_H: %x\n", rval);

	regmap_read(led->regmap, MP3317_ILEDR_L, &rval);
	at += sysfs_emit_at(buf, at, "MP3317_ILEDR_L: %x\n", rval);

	regmap_read(led->regmap, MP3317_ILEDR_H, &rval);
	at += sysfs_emit_at(buf, at, "MP3317_ILEDR_H: %x\n", rval);

	regmap_read(led->regmap, MP3317_VOUT, &rval);
	at += sysfs_emit_at(buf, at, "MP3317_VOUT: %x\n", rval);

	regmap_read(led->regmap, MP3317_HR_TH, &rval);
	at += sysfs_emit_at(buf, at, "MP3317_HR_TH: %x\n", rval);

	regmap_read(led->regmap, MP3317_HR_STEP, &rval);
	at += sysfs_emit_at(buf, at, "MP3317_HR_STEP: %x\n", rval);

	regmap_read(led->regmap, MP3317_FAULT1, &rval);
	at += sysfs_emit_at(buf, at, "MP3317_FAULT1: %x\n", rval);

	regmap_read(led->regmap, MP3317_FAULT2, &rval);
	at += sysfs_emit_at(buf, at, "MP3317_FAULT2: %x\n", rval);

	regmap_read(led->regmap, MP3317_REG08_SET_VOUT, &rval);
	at += sysfs_emit_at(buf, at, "MP3317_REG08_SET_VOUT: %x\n", rval);

	return at;
}

static ssize_t reg_dump_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	unsigned int addr, val;

	// get the base parameters
	ret = sscanf(buf, "%2x %2x", &addr, &val);

	printk("MP3317: setting %x = %x\n", addr, val);
	regmap_write(led->regmap, addr, val);  

	return count;
}

static DEVICE_ATTR_RW(reg_dump);

static ssize_t led_fault_show(struct device *dev, 
							  struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	uint rval;
	uint val;
	int at = 0;

	*buf = '\0';
	
	ret = regmap_read(led->regmap, MP3317_FAULT1, &rval);
	if(ret)
		return ret;
	val = rval & FAULT1_MASK;

	ret = regmap_read(led->regmap, MP3317_FAULT2, &rval);
	if(ret)
		return ret;
	val |= (rval & FAULT2_MASK) <<8;

	if(!val){
		at += sysfs_emit_at(buf, at, "%s ", "None");
	} else {
		if(val& 0x01){
			at += sysfs_emit_at(buf, at, "%s ", "[Short]");
		}
		if(val& 0x02){
			at += sysfs_emit_at(buf, at, "%s ", "[Open]");
		}
		if(val& 0x04){
			at += sysfs_emit_at(buf, at, "%s ", "[Over Voltage]");
		}
		if(val& 0x08){
			at += sysfs_emit_at(buf, at, "%s ", "[Input current over current]");
		}
		if(val& 0x10){
			at += sysfs_emit_at(buf, at, "%s ", "[Cycle-by-cycle current over-current]");
		}
		if(val& 0x20){
			at += sysfs_emit_at(buf, at, "%s ", "[Over temperature]");
		}
		if(val& 0x40){
			at += sysfs_emit_at(buf, at, "%s ", "[Short to Gnd]");
		}
		if(val& 0x100){
			at += sysfs_emit_at(buf, at, "%s ", "[ISET pin short to gnd]");
		}
		if(val& 0x200){
			at += sysfs_emit_at(buf, at, "%s ", "[ISET pin open]");
		}
		if(val& 0x400){
			at += sysfs_emit_at(buf, at, "%s ", "[Latch off over-current]");
		}
	}

	ret = regmap_write(led->regmap, MP3317_FAULT1, 0xff);
	if(ret)
		return ret;
	ret = regmap_write(led->regmap, MP3317_FAULT2, 0xff);
	if(ret)
		return ret;

	at += sysfs_emit_at(buf, at, "\n");

	return (at);
}

static DEVICE_ATTR_RO(led_fault);

static ssize_t led_ovp_threshold_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	uint rval;

	ret = regmap_read(led->regmap, MP3317_OVP_IMAX, &rval);
	if(ret)
		return ret;
	
	rval &= OVP_MASK;

	switch (rval) {
	case 0:
		return sysfs_emit(buf, "%dV\n", 17);
	case 0x08:
		return sysfs_emit(buf, "%dV\n", 21);
	case 0x10:
		return sysfs_emit(buf, "%dV\n", 25);
	case 0x18:
		return sysfs_emit(buf, "%dV\n", 30);
	default:
		return sysfs_emit(buf, "%dV\n", 34);
	}

	return 0;
}

/*
 * Input 0, 1, 2, 3, others
 * 0: 17V; 1: 21V; 2: 25V; 3: 30V; others: 34V
 */
static ssize_t led_ovp_threshold_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	u8 val;

	ret = kstrtou8(buf, 10, &val);
	if(ret )
		return ret;

	switch(val){
	case 0:
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, OVP_MASK, val << 3);
		break;
	case 1:
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, OVP_MASK, val << 3);
		break;
	case 2:
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, OVP_MASK, val << 3);
		break;
	case 3:
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, OVP_MASK, val << 3);
		break;
	default:
		ret = regmap_update_bits(led->regmap, MP3317_OVP_IMAX, OVP_MASK, 7 << 3);
		break;
	}
	
	if(ret)
		return ret;

	return count; 
}

static DEVICE_ATTR_RW(led_ovp_threshold);

static ssize_t led_max_pwm_time_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	uint rval;

	ret = regmap_read(led->regmap, MP3317_TPWM, &rval);
	if(ret)
		return ret;
	
	rval &= TPWM_MASK;

	switch (rval) {
	case 0:
		return sysfs_emit(buf, "%s\n", "NO limitation");
	case 0x01:
		return sysfs_emit(buf, "%sms\n", "0.5");
	case 0x02:
		return sysfs_emit(buf, "%sms\n", "1.0");
	case 0x03:
		return sysfs_emit(buf, "%sms\n", "1.5");
	case 0x04:
		return sysfs_emit(buf, "%sms\n", "2.0");
	case 0x05:
		return sysfs_emit(buf, "%sms\n", "2.5");
	case 0x06:
		return sysfs_emit(buf, "%sms\n", "3.0");
	case 0x07:
		return sysfs_emit(buf, "%sms\n", "3.5");
	case 0x08:
		return sysfs_emit(buf, "%sms\n", "4.0");
	case 0x09:
		return sysfs_emit(buf, "%sms\n", "4.5");
	case 0x0A:
		return sysfs_emit(buf, "%sms\n", "5.0");
	default:
		return 0;
	}
}
/*
 *    Input range :0~10
 *    0: no limitation; 1: 0.5ms; 2: 1ms; 3: 1.5ms; 4: 2ms; 5: 2.5ms; 6: 3ms; 7: 3.5ms
 *    8: 4ms; 9: 4.5ms; 10: 5ms
 */
static ssize_t led_max_pwm_time_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	u8 val;

	ret = kstrtou8(buf, 10, &val);
	if(ret)
		return ret;

	if(val > 10)
		return count;

	ret = regmap_update_bits(led->regmap, MP3317_TPWM, TPWM_MASK, val);
	if(ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(led_max_pwm_time);

static ssize_t led_cycle_by_cycle_cur_limit_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	uint rval;

	ret = regmap_read(led->regmap, MP3317_MODE_CTL, &rval);
	if(ret)
		return ret;
	
	rval &= CBC_LIMIT_MASK;
	printk("%x\r\n",rval);
	switch (rval) {
	case 0:
		return sysfs_emit(buf, "%sA\n", "0.3");
	case 0x08:
		return sysfs_emit(buf, "%sA\n", "0.5");
	case 0x10:
		return sysfs_emit(buf, "%sA\n", "0.7");
	case 0x18:
		return sysfs_emit(buf, "%sA\n", "0.9");
	case 0x20:
		return sysfs_emit(buf, "%sA\n", "1.2");
	case 0x28:
		return sysfs_emit(buf, "%sA\n", "1.5");
	case 0x30:
		return sysfs_emit(buf, "%sA\n", "2.0");
	case 0x38:
		return sysfs_emit(buf, "%sA\n", "3.0");
	}

	return 0;
}

/*
 *    Input range :0~7
 *    0: 0.3A; 1: 0.5A; 2: 0.7A; 3: 0.9A; 4: 1.2A; 5: 1.5A; 6: 2.0A; 7: 3.0A
 */
static ssize_t led_cycle_by_cycle_cur_limit_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	u8 val;

	ret = kstrtou8(buf, 10, &val);
	if(ret)
		return ret;

	val = val << 3;
	
	ret = regmap_update_bits(led->regmap, MP3317_MODE_CTL, CBC_LIMIT_MASK, val);
	if(ret) 
		return ret;

	return count;
}

static DEVICE_ATTR_RW(led_cycle_by_cycle_cur_limit);

static ssize_t led_incl_show(struct device *dev, 
								   struct device_attribute *attr, char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	uint rval;

	ret = regmap_read(led->regmap, MP3317_MODE_CTL, &rval);
	if(ret)
		return ret;
	
	rval &= INCL_MASK;
	switch (rval) {
	case 0:
		return sysfs_emit(buf, "%sA\n", "0.5");
	case 1:
		return sysfs_emit(buf, "%sA\n", "0.8");
	case 2:
		return sysfs_emit(buf, "%sA\n", "1.0");
	case 3:
		return sysfs_emit(buf, "%sA\n", "1.5");
	case 4:
		return sysfs_emit(buf, "%sA\n", "2.0");
	case 5:
		return sysfs_emit(buf, "%sA\n", "2.5");
	case 6:
		return sysfs_emit(buf, "%sA\n", "3.0");
	case 7:
		return sysfs_emit(buf, "%sA\n", "no limitation");
	}

	return 0;
}

/*
 *    Input range :0~7
 *    0: 0.5A; 1: 0.8A; 2: 1.0A; 3: 1.5A; 4: 2.0A; 5: 2.5A; 6: 3.0A; 7: no limitation
 */
static ssize_t led_incl_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	u8 val;

	ret = kstrtou8(buf, 10, &val);
	if(ret)
			return ret;

	val = val;

	ret = regmap_update_bits(led->regmap, MP3317_MODE_CTL, INCL_MASK, val);
	if(ret) return ret;
	return count;
}

static DEVICE_ATTR_RW(led_incl);

static ssize_t led_channel_n_show (struct device *dev, 
							uint channel,
							char *buf)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	uint rval;

	ret = regmap_read(led->regmap, MP3317_EN_CTL, &rval);
	if(ret)
		return ret;

	rval &= 1 << (channel - 1);

	return sysfs_emit(buf, "CH%d:%s\n", channel, rval ? "Enable" : "Disable");
}

static ssize_t led_channel_n_store (struct device *dev, 
							uint channel,
							const char *buf,
							size_t count)
{
	struct led_classdev *lcdev = dev_get_drvdata(dev);
	struct mp3317 *led = container_of(lcdev, struct mp3317, wled);
	int ret;
	u8 val;
	u8 mask;

	ret = kstrtou8(buf, 10, &val);
	if(ret < 0){
		dev_err(dev, "Value too large!\n");
		return ret;
	}

	mask = 1 << (channel - 1);

	ret = regmap_update_bits(led->regmap, MP3317_EN_CTL, mask, val ? mask : 0);
	if(ret)
		return ret;

	return count;     
}

static ssize_t led_channel1_show(struct device *dev, 
								 struct device_attribute *attr, char *buf)
{
	return led_channel_n_show(dev, 1, buf);
}

static ssize_t led_channel1_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	return led_channel_n_store(dev, 1, buf, count);
}

static ssize_t led_channel2_show(struct device *dev, 
								 struct device_attribute *attr, char *buf)
{
	return led_channel_n_show(dev, 2, buf);
}

static ssize_t led_channel2_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	return led_channel_n_store(dev, 2, buf, count);
}

static ssize_t led_channel3_show(struct device *dev, 
								 struct device_attribute *attr, char *buf)
{
	return led_channel_n_show(dev, 3, buf);
}

static ssize_t led_channel3_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	return led_channel_n_store(dev, 3, buf, count);
}

static ssize_t led_channel4_show(struct device *dev, 
								 struct device_attribute *attr, char *buf)
{
	return led_channel_n_show(dev, 4, buf);
}

static ssize_t led_channel4_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	return led_channel_n_store(dev, 4, buf, count);
}

static ssize_t led_channel5_show(struct device *dev, 
								 struct device_attribute *attr, char *buf)
{
	return led_channel_n_show(dev, 5, buf);
}

static ssize_t led_channel5_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	return led_channel_n_store(dev, 5, buf, count);
}

static ssize_t led_channel6_show(struct device *dev, 
								 struct device_attribute *attr, char *buf)
{
	return led_channel_n_show(dev, 6, buf);
}

static ssize_t led_channel6_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	return led_channel_n_store(dev, 6, buf, count);
}

static DEVICE_ATTR_RW(led_channel1);
static DEVICE_ATTR_RW(led_channel2);
static DEVICE_ATTR_RW(led_channel3);
static DEVICE_ATTR_RW(led_channel4);
static DEVICE_ATTR_RW(led_channel5);
static DEVICE_ATTR_RW(led_channel6);

static struct attribute *channel_left_attrs[] = {
	&dev_attr_led_channel1.attr,
	&dev_attr_led_channel2.attr,
	&dev_attr_led_channel3.attr,
	NULL,
};

static struct attribute *channel_right_attrs[] = {
	&dev_attr_led_channel4.attr,
	&dev_attr_led_channel5.attr,
	&dev_attr_led_channel6.attr,
	NULL,
};

static const struct attribute_group channel_left_set = {
		.name = "Channel left",
		.attrs = channel_left_attrs
};

static const struct attribute_group channel_right_set = {
		.name = "Channel right",
		.attrs = channel_right_attrs
};

static struct attribute *main_attrs[] = {
	&dev_attr_led_config.attr,
	&dev_attr_led_fsw.attr,
	&dev_attr_led_state.attr,
	&dev_attr_led_ovp_threshold.attr,
	&dev_attr_led_max_pwm_time.attr,
	&dev_attr_led_cycle_by_cycle_cur_limit.attr,
	&dev_attr_led_incl.attr,
	&dev_attr_led_fault.attr,
	&dev_attr_reg_dump.attr,
	NULL,
};

static const struct attribute_group main_set = {
		.attrs = main_attrs
};
static const struct attribute_group *mp3317_groups[] = {	
	&Vout_Set,						
	&current_set,						
	&main_set,						
	&channel_left_set,
	&channel_right_set,					
	NULL,							
};

static const struct regmap_config MP3317_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

/*
 * Control ILED value (reg04h,reg05h|reg06h,reg07h)
 */
 // sets current of both leds
static int led_brightness_set(struct led_classdev *cdev,
				  enum led_brightness brightness)
{
    // TODO
    // Convert linux 0-255 brightness range to current setting
	return 0;
}

// returns brightness of left led
static enum led_brightness mp3317_led_get(struct led_classdev *cdev)
{
	struct mp3317 *led = container_of(cdev, struct mp3317, wled);
	int ret;
	int rval, val;

	ret = regmap_read(led->regmap, MP3317_ILEDL_L, &rval);
	if(ret)
			return ret;

	val = rval;
	ret = regmap_read(led->regmap, MP3317_ILEDL_H, &rval);
	if(ret)
			return ret;
	val |= (rval & 0x0f) << 8;

	return val;
}

static int mp3317_leds_suspend(struct device *dev)
{
	struct mp3317 *led = dev_get_drvdata(dev);

	if ( !led->use_pm_ops )
		return 0;

	gpiod_set_value_cansleep(led->en_gpio, 0);

	return 0;
}

/* On resume restore the default settings */
static int mp3317_leds_resume(struct device *dev)
{
	struct mp3317 *led = dev_get_drvdata(dev);

	if ( !led->use_pm_ops )
		return 0;

	gpiod_set_value_cansleep(led->en_gpio, 1);
	usleep_range(10000,20000);
	// set default config led
	mp3317_config(led);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mp3317_leds_pm, mp3317_leds_suspend, mp3317_leds_resume);

static int mp3317_reg_enable(struct regulator_dev *rdev)
{
	int ret = 0;
	struct mp3317 *led = rdev_get_drvdata(rdev);

	if ( led == NULL ) {
		dev_err(&rdev->dev, "could not get led device");
		return 0;
	}

	gpiod_set_value(led->en_gpio, 1);
	usleep_range(10000,20000);
	ret = mp3317_config(led);

	// retry a few times in case the i2c bus is busy coming out of suspend
	for ( int i = 0; i < 10 && ret; i++ ) {
		ret = mp3317_config(led);
		usleep_range(2000,4000);
	}

	if ( ret ) {
		dev_err(&led->client->dev, "config failed with %d \n", ret );
	}

	return 0;
}

static int mp3317_reg_disable(struct regulator_dev *rdev)
{
	struct mp3317 *led = rdev_get_drvdata(rdev);

	if ( led == NULL ) {
		dev_err(&rdev->dev, "could not get led device");
		return 0;
	}

	regmap_update_bits(led->regmap, MP3317_EN_CTL, EN_MASK, 0);
	gpiod_set_value(led->en_gpio, 0);

	return 0;
}

static int mp3317_reg_is_enabled(struct regulator_dev *rdev)
{
	struct mp3317 *led = rdev_get_drvdata(rdev);

	int ret;
	uint rval;

	if ( led == NULL ) {
		dev_err(&rdev->dev, "could not get led device");
		return 0;
	}

	if (!gpiod_get_value(led->en_gpio))
		return 0;

	ret = regmap_read(led->regmap, MP3317_EN_CTL, &rval);
	if(ret)
		return ret;
	rval &= EN_MASK;

	return rval ? 1 : 0;
}

static const struct regulator_ops reg_ops = {
	.enable = mp3317_reg_enable,
	.disable = mp3317_reg_disable,
	.is_enabled = mp3317_reg_is_enabled,
};

static struct of_regulator_match match = { .name = "backlight" };

static int mp3317_leds_probe(struct i2c_client *client)
{
	struct mp3317 *led;
	int ret;
	unsigned int rval;
	struct regulator_config cfg = { };
	struct regulator_dev *rdev;
	struct device *dev = &client->dev;
	struct device_node *regulator_node;

	led = devm_kzalloc(&client->dev, sizeof(*led), GFP_KERNEL);
	if(!led)
		return -ENOMEM;

	led->en_gpio = devm_gpiod_get(&client->dev, "en", GPIOD_OUT_LOW);
	if (IS_ERR(led->en_gpio)) {
		ret = PTR_ERR(led->en_gpio);
		dev_err(&client->dev, "cannot get en gpio %d\n", ret);
		return ret;
	}

	led->client = client;

	i2c_set_clientdata(client, led);

	led->regmap = devm_regmap_init_i2c(client, &MP3317_regmap_config);
	if(IS_ERR(led->regmap))
		return PTR_ERR(led->regmap);

	// reset the chip here
	gpiod_set_value(led->en_gpio, 0);
	usleep_range(10000,20000);
	gpiod_set_value(led->en_gpio, 1);
	usleep_range(10000,20000);

	// check for i2c comms
	ret = regmap_read(led->regmap, MP3317_EN_CTL, &rval);
	if ( ret ) {
		dev_err(&led->client->dev, "I2C comms failed: %d\n", ret );
		return ret;
	}

	// turn off until requested
	gpiod_set_value(led->en_gpio, 0);

	// setup led platform device
	led->rst = 60000;
	if ( device_property_read_string(&client->dev, "mp3317,label", &led->wled.name) ) {
		led->wled.name = "mp3317_led";
	}
	led->wled.max_brightness = MAX_BRIGHTNESS;
	led->wled.brightness_set_blocking = led_brightness_set;
	led->wled.brightness_get = mp3317_led_get;
	led->wled.groups = mp3317_groups;
	led->wled.flags |= LED_CORE_SUSPENDRESUME;

	ret = devm_led_classdev_register_ext(&client->dev, &led->wled, NULL);
	if(ret){
		dev_err(&client->dev, "Unable register led: %s\n", led->wled.name);
		return ret;
	}

	regulator_node = of_get_child_by_name(dev->of_node, "regulator");
	if (!regulator_node) {
		dev_err(dev, "regulator node not found\n");
	} else {
		ret = of_regulator_match(dev, regulator_node, &match, 1);
		if (ret == 1) {
			// setup regulator if included
			cfg.dev = dev;
			cfg.init_data = match.init_data;
			if (!cfg.init_data) {
				dev_err(dev, "did NOT find mp3317 init data\n");
			} else {
				dev_err(dev, "found mp3317 init data %s\n", cfg.init_data->constraints.name );
			}
			cfg.driver_data = led;
			cfg.of_node = match.of_node;

			if ( of_property_read_string(match.of_node, "regulator-name", &led->desc.name) ) {
				dev_err(dev, "Could not get name of regulator\n");
			}
			led->desc.ops = &reg_ops;
			led->desc.type = REGULATOR_VOLTAGE;

			rdev = devm_regulator_register(dev, &led->desc, &cfg);
			if (IS_ERR(rdev)) {
				dev_err(dev, "Failed to register regulator: %d\n", (int) PTR_ERR(rdev));
			}
		} else {
			dev_err(dev, "Regulator match failed, %d\n", ret);
		}
	}

	// check if pm-ops should be used, default to true
	if ( device_property_read_u32(dev, "mp3317,use-pm-ops",
				&led->use_pm_ops) ) {
		led->use_pm_ops = 1;
	}

	return 0;
}

static void mp3317_leds_remove(struct i2c_client *client)
{
	struct mp3317 *led = i2c_get_clientdata(client);

	// Turn off
	gpiod_set_value(led->en_gpio, 0);
}

static const struct i2c_device_id mp3317_id[] = {
	{ "mp3317", 0 }, 
	{} 
};
MODULE_DEVICE_TABLE(i2c, mp3317_id);

static const struct of_device_id mp3317_of_match[] = {
	{ .compatible ="mps,mp3317" },
	{}
};
MODULE_DEVICE_TABLE(of, mp3317_of_match);

static struct i2c_driver mp3317_driver = {
	.probe = mp3317_leds_probe,
	.remove = mp3317_leds_remove,
	.driver = {
		.name = "mp3317_led",
		.of_match_table = mp3317_of_match,
		.pm = pm_sleep_ptr(&mp3317_leds_pm),
	},
	.id_table = mp3317_id,
};

module_i2c_driver(mp3317_driver);
MODULE_AUTHOR("Yuxi Wang <Yuxi.Wang@monolithicpower.com>");
MODULE_DESCRIPTION("MPS MP3317 LED driver");
MODULE_LICENSE("GPL");
