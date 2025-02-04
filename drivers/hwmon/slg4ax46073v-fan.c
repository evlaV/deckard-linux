/* Silego SLG4AX46073V PWM Fan Controller
 *
 */

#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

/*
 * SLG4AX46073V registers.
 * Indentation helps identify how these constants apply:
 *      Register number.
 *          Per-register bit fields.
 *              Values for multi-bit fields.
 */
// Base Version 
#define SLG4AX46073V_REG_OUT_EN       0x4C /* Output enable registger */
#define     SLG4AX46073V_OUT_WDTW         0x01
#define     SLG4AX46073V_OUT_WDT_RESET    0x02
#define     SLG4AX46073V_OUT_CMP          0x04
#define     SLG4AX46073V_OUT_OSC          0x08
#define SLG4AX46073V_REG_TACH        0xA5 /* Tachometer Rev / 5msec */
#define SLG4AX46073V_REG_PWM_DC      0xA0 /* PWM Duty Cycle, default 90% = 0x49 */
#define     SLG4AX46073V_PWM_DC_MAX         98
// The min of 1 here will translate to a PWM of 0
#define     SLG4AX46073V_PWM_DC_MIN          2
#define SLG4AX46073V_REG_PWM_PERIOD  0x98 /* PWM Period, default 25kHz = 0x28 */
#define SLG4AX46073V_PWM_PERIOD_LSB       100 /* TODO figure out PWM period resolution */
#define SLG4AX46073V_PWM_PERIOD_DEFAULT   0x28 /* 25 kHz */
#define SLG4AX46073V_PWM_PERIOD_MIN       0x32 /* 20 kHz */
#define SLG4AX46073V_PWM_FREQ_MIN         20000
#define SLG4AX46073V_PWM_PERIOD_MAX       0x1E /* 33 kHz */
#define SLG4AX46073V_PWM_FREQ_MAX         33000
// this rate also defines the tach read rate for versions >= 7.
// at 3 Hz the minimum readable RPM is 540
#define SLG4AX56073V_WDT_RESET_RATE       HZ*3

// Version 7 chnages
#define SLG4AX46073V_REG_VIRTUAL_INPUT        0x4C /* Output enable registger */
#define     SLG4AX46073V_VI_WDT_ARM           0x01
#define     SLG4AX46073V_VI_WDT_PET           0x02
#define     SLG4AX46073V_VI_DISABLE_PWM_CLK0  0x08
#define     SLG4AX46073V_VI_TACH_ARM          0x20 
#define SLG4AX46073V_REG_VERSION     0xFA
#define SLG4AX46073V_REG_TACH_ARMED  0x49
#define     SLG4AX46073V_TACH_ARMED       0x01
#define SLG4AX46073V_REG_WDT_STATUS  0x4A
#define     SLG4AX46073V_WDT_OK        0x10
#define SLG4AX46073V_REG_RESET       0xF5
#define     SLG4AX46073V_RESET                0x01
#define SLG4AX46073V_REG_WDT_FREQ    0x9F
#define     SLG4AX46073V_WDT_FULL             0xCF
#define     SLG4AX46073V_WDT_HALF             0xCA

#define VERSION_7                    0x07

#define TOGGLE_BIT(VAR, MASK) VAR = (VAR & ~MASK) | ((VAR ^ MASK) & MASK);

#define SLG4AX46073V_I2C_FAILURE   0x01
#define SLG4AX46073V_RPM_FAILURE   0x02
#define SLG4AX46073V_TACH_FAILURE  0x04

/*
 * struct slg4ax46073v - device data
 * @regmap:	Register map.
 */
struct slg4ax46073v {
	struct regmap *regmap;
	struct gpio_desc *fan_alert;
	struct gpio_desc *bp_gpiod;
	struct gpio_desc *en_gpiod;
    int fan_alert_gpio_irq;
	uint16_t fan_rpm; // stores most recent rpm calculation
    unsigned int fan_alarm; // stores fan alarm state
    unsigned int reset_attempts;
	struct delayed_work wdt_work;
	const struct attribute_group *attr_groups;
    unsigned int version;
    u32 pm_duty, pm_period, pm_enabled;
};

static bool slg4ax46073v_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
    case SLG4AX46073V_REG_VIRTUAL_INPUT:
	case SLG4AX46073V_REG_TACH:
	case SLG4AX46073V_REG_PWM_DC:
	case SLG4AX46073V_REG_PWM_PERIOD:
    case SLG4AX46073V_REG_TACH_ARMED:
    case SLG4AX46073V_REG_WDT_STATUS:
    case SLG4AX46073V_REG_VERSION:
    case SLG4AX46073V_REG_RESET:
    case SLG4AX46073V_REG_WDT_FREQ:
		return true;
	default:
		return false;
	}
}

static bool slg4ax46073v_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
    case SLG4AX46073V_REG_VIRTUAL_INPUT:
	case SLG4AX46073V_REG_TACH:
	case SLG4AX46073V_REG_PWM_DC:
	case SLG4AX46073V_REG_PWM_PERIOD:
    case SLG4AX46073V_REG_TACH_ARMED:
    case SLG4AX46073V_REG_WDT_STATUS:
    case SLG4AX46073V_REG_VERSION:
    case SLG4AX46073V_REG_RESET:
    case SLG4AX46073V_REG_WDT_FREQ:
		return true;
	default:
		return false;
	}
}

static bool slg4ax46073v_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
    case SLG4AX46073V_REG_VIRTUAL_INPUT:
	case SLG4AX46073V_REG_TACH:
	case SLG4AX46073V_REG_PWM_DC:
	case SLG4AX46073V_REG_PWM_PERIOD:
    case SLG4AX46073V_REG_TACH_ARMED:
    case SLG4AX46073V_REG_WDT_STATUS:
    case SLG4AX46073V_REG_VERSION:
    case SLG4AX46073V_REG_RESET:
    case SLG4AX46073V_REG_WDT_FREQ:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config slg4ax46073v_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = slg4ax46073v_writeable_reg,
	.readable_reg = slg4ax46073v_readable_reg,
	.volatile_reg = slg4ax46073v_volatile_reg,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_NONE,
};

static int slg4ax46073v_read_reg(struct slg4ax46073v *slg4ax46073v, unsigned int reg_addr, unsigned int *val)
{
	return regmap_read(slg4ax46073v->regmap, reg_addr, val);
}

static int slg4ax46073v_write_reg(struct slg4ax46073v *slg4ax46073v, unsigned int reg_addr, unsigned int val)
{
	return regmap_write(slg4ax46073v->regmap, reg_addr, val);
}

// curves based on plot of datasheet values ds.10
static unsigned int slg4ax46073v_pwm_duty_cycle_to_reg_value(struct slg4ax46073v *slg4ax46073v, unsigned int requested_pwm_dc)
{
    unsigned int pwm_dc = requested_pwm_dc;

    if ( pwm_dc > SLG4AX46073V_PWM_DC_MAX ) {
        pwm_dc = SLG4AX46073V_PWM_DC_MAX;
    } else if ( pwm_dc < SLG4AX46073V_PWM_DC_MIN && slg4ax46073v->version != VERSION_7 ) {
        pwm_dc = SLG4AX46073V_PWM_DC_MIN;
    }

    if ( slg4ax46073v-> version >= VERSION_7 ) {
        if(requested_pwm_dc == 0) {
            return 0;
        } else {
            unsigned int pwm_period;
            slg4ax46073v_read_reg(slg4ax46073v, SLG4AX46073V_REG_PWM_PERIOD, &pwm_period);
            return (pwm_dc*pwm_period)/50+1;
        }
    } else {
        return pwm_dc*82/100-1;
    }
}

static unsigned int slg4ax46073v_pwm_reg_value_to_duty_cycle(struct slg4ax46073v *slg4ax46073v, unsigned int reg_value)
{
    if ( slg4ax46073v-> version >= VERSION_7 ) {
        unsigned int pwd_period;
        slg4ax46073v_read_reg(slg4ax46073v, SLG4AX46073V_REG_PWM_PERIOD, &pwd_period);
       return reg_value*50/pwd_period;
    } else {
        return reg_value*120/100+1;
    }
}

/* Convert slg4ax46073v value to RPM.  value is rev/5msec */
static uint16_t slg4ax46073v_rpm_from_tach( struct slg4ax46073v *slg4ax46073v, uint16_t tach_count )
{
    if ( slg4ax46073v-> version >= VERSION_7 ) {
        return ( 256000 * 60 ) / tach_count;
    } else {
        return ( tach_count * 5 * 60 ) / 2; // ds.9
    }      
}

static int slg4ax46073v_arm_tach(struct slg4ax46073v *slg4ax46073v)
{
    unsigned int virtual_in;
    unsigned int reg_val;
    regmap_read(slg4ax46073v->regmap, SLG4AX46073V_REG_TACH_ARMED, &reg_val);
    if(reg_val & SLG4AX46073V_TACH_ARMED) {
        pr_err( "slg4a_fan: arming tach while its already armed ( %x )\n", reg_val);
        return -EIO;
    } else {
        slg4ax46073v_read_reg( slg4ax46073v, SLG4AX46073V_REG_VIRTUAL_INPUT, &virtual_in);
        TOGGLE_BIT(virtual_in,SLG4AX46073V_VI_TACH_ARM);
        return slg4ax46073v_write_reg( slg4ax46073v, SLG4AX46073V_REG_VIRTUAL_INPUT, virtual_in);
    }
}

static int slg4ax46073v_disarm_wdt(struct slg4ax46073v *slg4ax46073v)
{
    unsigned int virtual_in;
    if ( slg4ax46073v_read_reg( slg4ax46073v, SLG4AX46073V_REG_VIRTUAL_INPUT, &virtual_in) ) {
        return -EIO;
    } else {
        virtual_in &= ~SLG4AX46073V_VI_WDT_ARM;
    }
    return slg4ax46073v_write_reg( slg4ax46073v, SLG4AX46073V_REG_VIRTUAL_INPUT, virtual_in);
}

static int slg4ax46073v_arm_wdt(struct slg4ax46073v *slg4ax46073v)
{
    unsigned int virtual_in;
    if ( slg4ax46073v_read_reg( slg4ax46073v, SLG4AX46073V_REG_VIRTUAL_INPUT, &virtual_in) ) {
        return -EIO;
    } else {
        virtual_in |= SLG4AX46073V_VI_WDT_ARM;
    }
    return slg4ax46073v_write_reg( slg4ax46073v, SLG4AX46073V_REG_VIRTUAL_INPUT, virtual_in);
}

static int slg4ax46073v_pet_wdt(struct slg4ax46073v *slg4ax46073v)
{
    unsigned int virtual_in;
    if ( slg4ax46073v_read_reg( slg4ax46073v, SLG4AX46073V_REG_VIRTUAL_INPUT, &virtual_in) ) {
        return -EIO;
    } else {
        TOGGLE_BIT( virtual_in, SLG4AX46073V_VI_WDT_PET); 
    }
    return slg4ax46073v_write_reg( slg4ax46073v, SLG4AX46073V_REG_VIRTUAL_INPUT, virtual_in);
}

static int slg4ax46073v_get_version(struct slg4ax46073v *slg4ax46073v, unsigned int * version)
{
    int ret = regmap_bulk_read( slg4ax46073v->regmap, SLG4AX46073V_REG_VERSION, version, 2);
    if ( !ret ) {
        *version &= 0xFF;
    }
    return ret;
}

static int slg4ax46073v_reset_wdt( struct slg4ax46073v *slg4ax46073v )
{
    int ret = 0;
    ret |= slg4ax46073v_disarm_wdt( slg4ax46073v );
    ret |= slg4ax46073v_arm_wdt( slg4ax46073v );
    ret |= slg4ax46073v_pet_wdt( slg4ax46073v );
    // set WDT PWM value to 50%
    slg4ax46073v_write_reg( slg4ax46073v, SLG4AX46073V_REG_WDT_FREQ, SLG4AX46073V_WDT_HALF);
    cancel_delayed_work(&slg4ax46073v->wdt_work);
    queue_delayed_work(system_freezable_wq, &slg4ax46073v->wdt_work, SLG4AX56073V_WDT_RESET_RATE);
    return ret;
}

static bool slg4ax46073v_wdt_fired( struct slg4ax46073v *slg4ax46073v )
{
    unsigned int wdt_status;
    slg4ax46073v_read_reg( slg4ax46073v, SLG4AX46073V_REG_WDT_STATUS, &wdt_status);
    return ( ( wdt_status & SLG4AX46073V_WDT_OK ) != SLG4AX46073V_WDT_OK );
}

static void slg4ax46073v_reset(struct slg4ax46073v *slg4ax46073v)
{
    // disable Oscillator, disable WDT, disable tach
    regmap_write(slg4ax46073v->regmap, SLG4AX46073V_REG_VIRTUAL_INPUT, SLG4AX46073V_VI_DISABLE_PWM_CLK0);

    // turn off fan power supply
    gpiod_set_value_cansleep(slg4ax46073v->en_gpiod, 0);
    usleep_range(100000,150000);

    // reset fan controller
    regmap_write(slg4ax46073v->regmap, SLG4AX46073V_REG_RESET, SLG4AX46073V_RESET);

    // turn fan power supply back on
    gpiod_set_value_cansleep(slg4ax46073v->en_gpiod, 1);
    usleep_range(100000,150000);

    // re arm tach
    slg4ax46073v_arm_tach(slg4ax46073v);

    // reset wdt
    slg4ax46073v_reset_wdt( slg4ax46073v );
}

static irqreturn_t fan_alert_irq_handler_top(int irq, void *dev_id)
{
    return IRQ_WAKE_THREAD;
}

static irqreturn_t fan_alert_irq_handler(int irq, void *dev_id)
{
    unsigned int tach_reg;
    int ret;
    struct i2c_client *client = dev_id;
    struct slg4ax46073v * slg4ax46073v = dev_get_drvdata(&client->dev);

    if ( slg4ax46073v->fan_alert_gpio_irq != irq ) {
        dev_err( &client->dev, "invalid irq ( %d )\n", irq );
        return IRQ_NONE;
    }

    if ( slg4ax46073v->version <= VERSION_7 ) {
        // Read tachometer register
        ret = slg4ax46073v_read_reg( slg4ax46073v, SLG4AX46073V_REG_TACH, &tach_reg );
        if (ret) {
            dev_err( &client->dev, "Failed to read slg4a tach register" );
        }
        else {
            slg4ax46073v->fan_rpm = slg4ax46073v_rpm_from_tach( slg4ax46073v, tach_reg );
        }
    }
    return IRQ_HANDLED;
}

static void slg4ax46073v_delayed_work_handler(struct work_struct *work)
{
    unsigned int tach_reg;
	unsigned int reg_val;
    unsigned int pwm;
    unsigned int alarm = 0;
	int ret;

	struct slg4ax46073v *slg4ax46073v = container_of((struct delayed_work *)work,struct slg4ax46073v, wdt_work);

    // first check i2c communiction to the device.
    if ( slg4ax46073v_get_version(slg4ax46073v, &reg_val) < 0 ) {
        if ( !( slg4ax46073v->fan_alarm & SLG4AX46073V_I2C_FAILURE ))
            pr_err("slg4ax46073v: i2c comms failed, setting alarm\n");
        slg4ax46073v->fan_alarm |= SLG4AX46073V_I2C_FAILURE;
    } else if ( slg4ax46073v->version >= VERSION_7 ) {
        if ( slg4ax46073v_wdt_fired( slg4ax46073v ) ) {
            pr_err( "slg4ax46073v: wdt fired, resetting in work handler\n" );
            slg4ax46073v_reset_wdt( slg4ax46073v );
        }
        slg4ax46073v_pet_wdt( slg4ax46073v );

        // get current pwm setting 
        ret = regmap_read(slg4ax46073v->regmap, SLG4AX46073V_REG_PWM_DC, &reg_val);
        pwm = slg4ax46073v_pwm_reg_value_to_duty_cycle(slg4ax46073v, reg_val);

        // read tach armed register to see if there is a settled value
        regmap_read( slg4ax46073v->regmap, SLG4AX46073V_REG_TACH_ARMED, &tach_reg );
        if ( !(tach_reg & SLG4AX46073V_TACH_ARMED) ) {
            // Read tachometer register
            ret = regmap_bulk_read( slg4ax46073v->regmap, SLG4AX46073V_REG_TACH, &tach_reg, 2 );
            if (ret) {
                pr_err("slg4ax46073v: Failed to read slg4a tach register");
            } else {
                slg4ax46073v->fan_rpm = slg4ax46073v_rpm_from_tach( slg4ax46073v, (uint16_t)tach_reg );
            }
            slg4ax46073v_arm_tach( slg4ax46073v );
        } else if ( pwm > 0 ) {
            pr_err("slg4ax46073v: tach did not complete in expected period\n");
            alarm |= SLG4AX46073V_TACH_FAILURE;
        }

        // check for valid fan rpm reading if fan pwm is set
        if(pwm > 0 && slg4ax46073v->fan_rpm < 100) {
            pr_err("slg4ax46073v: invalid fan rpm ( %d ) for pwm setting ( %d ), attempting reset\n", slg4ax46073v->fan_rpm, pwm);
            alarm |= SLG4AX46073V_RPM_FAILURE;
        }

        if (alarm && !slg4ax46073v->reset_attempts) {
            pr_err("slg4ax46073v: detected alarm %x, attempting reset\n", alarm);
            // attempt to reset the fan controller before raising an alarm
            slg4ax46073v_reset(slg4ax46073v);
            slg4ax46073v->reset_attempts = 1;
        } else if ( alarm ){
            pr_err("slg4ax46073v: reset unsuccessful, raising alarm\n");
            // keep attempting reset
            slg4ax46073v_reset(slg4ax46073v);
            slg4ax46073v->fan_alarm = alarm;
        } else {
            // all clear
            slg4ax46073v->reset_attempts = 0;
            slg4ax46073v->fan_alarm = 0;
        }
    } else {
        // To pet ( reset ) the WDT we have to write a 0 and then 1 to the SLG4AX46073V_OUT_WDT_RESET bit
        ret = slg4ax46073v_read_reg(slg4ax46073v, SLG4AX46073V_REG_OUT_EN, &reg_val);
        if (ret) {
            pr_err("slg4ax46073v: failed to reset the slg4ax46073v watch dog\n");
            return;
        }
        
        slg4ax46073v_write_reg(slg4ax46073v, SLG4AX46073V_REG_OUT_EN, reg_val & ~SLG4AX46073V_OUT_WDT_RESET);
        slg4ax46073v_write_reg(slg4ax46073v, SLG4AX46073V_REG_OUT_EN, reg_val | SLG4AX46073V_OUT_WDT_RESET);
    }

	schedule_delayed_work(&slg4ax46073v->wdt_work, SLG4AX56073V_WDT_RESET_RATE);
}

static void slg4ax46073v_wdt_work_init(struct slg4ax46073v *slg4ax46073v)
{
	INIT_DELAYED_WORK(&slg4ax46073v->wdt_work, slg4ax46073v_delayed_work_handler);
	queue_delayed_work(system_freezable_wq, &slg4ax46073v->wdt_work, SLG4AX56073V_WDT_RESET_RATE);
}

// DEBUG Functions
static ssize_t slg4ax46073v_write_reg_store(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	int ret;
	struct slg4ax46073v *slg4ax46073v = dev_get_drvdata(dev);
	unsigned int reg_addr, val;
	ret = sscanf(buf, "%x %x", &reg_addr, &val);
	if(ret != 2) {
        pr_err("slg4ax46073v: missing params for slg4ax46073v write reg\n");
        return -EINVAL;
	} else {
        slg4ax46073v_write_reg(slg4ax46073v, reg_addr, val);
        return 0;
	}
}

static ssize_t slg4ax46073v_write_reg_show(struct device *dev, struct device_attribute *devattr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "slg4ax46073v: Invalid operation\n");
}

unsigned int read_reg_addr;
static ssize_t slg4ax46073v_read_reg_store(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	int ret;
	ret = sscanf(buf, "%x", &read_reg_addr);
	if(ret != 1) {
		pr_err("slg4ax46073v: missing params for slg4ax46073v read reg");
		return -EINVAL;
	} else {
        return 0;
	}
};

static ssize_t slg4ax46073v_read_reg_show(struct device *dev, struct device_attribute *devattr, char *buf)
{
	int ret;
	struct slg4ax46073v *slg4ax46073v = dev_get_drvdata(dev);
	unsigned int val;
	ret = slg4ax46073v_read_reg(slg4ax46073v,read_reg_addr, &val);
	return snprintf(buf, PAGE_SIZE, "Read %x from slg4ax46073v reg %x\n", val, read_reg_addr);
};


static SENSOR_DEVICE_ATTR(write_reg, 0644, slg4ax46073v_write_reg_show, slg4ax46073v_write_reg_store, 0);
static SENSOR_DEVICE_ATTR(read_reg, 0644, slg4ax46073v_read_reg_show, slg4ax46073v_read_reg_store, 0);

static struct attribute *slg4ax46073v_attrs[] = {
	&sensor_dev_attr_write_reg.dev_attr.attr,
	&sensor_dev_attr_read_reg.dev_attr.attr,
	NULL
};

ATTRIBUTE_GROUPS(slg4ax46073v);

static umode_t slg4ax46073v_is_visible(const void *dvrdata,
				   enum hwmon_sensor_types type,
				   u32 attr, int channel)
{
	switch (type) {
		case hwmon_pwm:
			switch (attr) {
			case hwmon_pwm_input:
			case hwmon_pwm_enable: // enable doesn't work pre version 7
			case hwmon_pwm_freq:
				return 0644;
			}
			break;
        case hwmon_fan:
            switch (attr) {
            case hwmon_fan_input:
            case hwmon_fan_alarm:
                return 0444;
            }
		break;
		default:
			break;
	}
	return 0;
}

static int slg4ax46073v_read_temp(struct device *dev, u32 attr, int channel,
			      long *val)
{
	return -EOPNOTSUPP;
}

static int slg4ax46073v_read_fan(struct device *dev, u32 attr, int channel,
			     long *val)
{
	struct slg4ax46073v *slg4ax46073v = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_fan_input:
		*val = slg4ax46073v->fan_rpm;
		break;
	case hwmon_fan_alarm:
		*val = slg4ax46073v->fan_alarm;
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

// TODO add PWM dutcy cycle reads /conversions
static int slg4ax46073v_read_pwm(struct device *dev, u32 attr, int channel,
			     long *val)
{
	struct slg4ax46073v *slg4ax46073v = dev_get_drvdata(dev);
	unsigned int regval;
	int ret;

	switch (attr) {
    case hwmon_pwm_input: // this is the duty cycle
	    ret = regmap_read(slg4ax46073v->regmap, SLG4AX46073V_REG_PWM_DC, &regval);
		*val = slg4ax46073v_pwm_reg_value_to_duty_cycle(slg4ax46073v, regval);
		return ret;
	case hwmon_pwm_enable:
        if ( slg4ax46073v->version >= VERSION_7 ) {
            ret = regmap_read(slg4ax46073v->regmap, SLG4AX46073V_REG_VIRTUAL_INPUT, &regval);
            if (!ret)
                *val = 1; // if i2c reads are successful, its enabled
            return ret;
        }
		break;
	case hwmon_pwm_freq:
        if ( slg4ax46073v->version >= VERSION_7 ) {
            ret = regmap_read(slg4ax46073v->regmap, SLG4AX46073V_REG_PWM_PERIOD, &regval);
            if (!ret)
                *val = 2048000 / 2 / ((uint32_t)regval + 1);
            return ret;
        } else {
            ret = regmap_read(slg4ax46073v->regmap, SLG4AX46073V_REG_PWM_PERIOD, &regval);
            if (ret)
                return ret;
            *val = 1000 / regval * 1000; // TODO should this return Hz or kHz?
        }
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int slg4ax46073v_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_temp:
		return slg4ax46073v_read_temp(dev, attr, channel, val);
	case hwmon_fan:
		return slg4ax46073v_read_fan(dev, attr, channel, val);
	case hwmon_pwm:
		return slg4ax46073v_read_pwm(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int slg4ax46073v_read_string(struct device *dev,
				enum hwmon_sensor_types type, u32 attr,
				int channel, const char **str)
{
	return -EOPNOTSUPP;

}

static int slg4ax46073v_write_pwm(struct device *dev, u32 attr, int channel,
			      long val)
{
	struct slg4ax46073v *slg4ax46073v = dev_get_drvdata(dev);
	unsigned int regval;

	switch (attr) {
    case hwmon_pwm_input: // this is the duty cycle
		regval = slg4ax46073v_pwm_duty_cycle_to_reg_value(slg4ax46073v,(unsigned int)val);
		return regmap_write(slg4ax46073v->regmap, SLG4AX46073V_REG_PWM_DC,regval);
	case hwmon_pwm_enable:
        if ( val ) {
            slg4ax46073v_reset(slg4ax46073v);
        }
        break;
	case hwmon_pwm_freq: // TODO fix out of range checks based on slg4 roll over bounds
        if (val < SLG4AX46073V_PWM_PERIOD_MIN)
            regval = SLG4AX46073V_PWM_PERIOD_MIN;
        else if (val > SLG4AX46073V_PWM_FREQ_MAX)
            regval = SLG4AX46073V_PWM_PERIOD_MAX;
        else if ( slg4ax46073v->version >= VERSION_7 ) {
            regval = (1024000 - val)/val;
        } else {
            regval = 1000/(val/1000);
        }
		return regmap_write(slg4ax46073v->regmap, SLG4AX46073V_REG_PWM_PERIOD,regval);
	}
	return 0;
}

static int slg4ax46073v_write_fan(struct device *dev, u32 attr, int channel,
			      long val)
{
	return -EOPNOTSUPP;
}

static int slg4ax46073v_write(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	switch (type) {
	case hwmon_fan:
		return slg4ax46073v_write_fan(dev, attr, channel, val);
	case hwmon_pwm:
		return slg4ax46073v_write_pwm(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static const struct hwmon_ops slg4ax46073v_hwmon_ops = {
	.is_visible = slg4ax46073v_is_visible,
	.read = slg4ax46073v_read,
	.read_string = slg4ax46073v_read_string,
	.write = slg4ax46073v_write,
};

static u32 slg4ax46073v_chip_config[] = {
	HWMON_C_REGISTER_TZ,
	0
};

static const struct hwmon_channel_info slg4ax46073v_chip = {
	.type = hwmon_chip,
	.config = slg4ax46073v_chip_config,
};

static u32 slg4ax46073v_pwm_config[] = {
	HWMON_PWM_INPUT | HWMON_PWM_FREQ | HWMON_PWM_ENABLE,
	0
};

static const struct hwmon_channel_info slg4ax46073v_pwm = {
	.type = hwmon_pwm,
	.config = slg4ax46073v_pwm_config,
};

static u32 slg4ax46073v_fan_config[] = {
	HWMON_F_INPUT | HWMON_F_ALARM,
	0
};

static const struct hwmon_channel_info slg4ax46073v_fan = {
	.type = hwmon_fan,
	.config = slg4ax46073v_fan_config,
};

static const struct hwmon_channel_info *slg4ax46073v_info[] = {
	&slg4ax46073v_chip,
	&slg4ax46073v_pwm,
	&slg4ax46073v_fan,
	NULL
};

static const struct hwmon_chip_info slg4ax46073v_chip_info = {
	.ops = &slg4ax46073v_hwmon_ops,
	.info = slg4ax46073v_info,
};

static int slg4ax46073v_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct slg4ax46073v *slg4ax46073v;
	struct device *hwmon_dev;
	int err;

	slg4ax46073v = devm_kzalloc(dev, sizeof(*slg4ax46073v), GFP_KERNEL);
	if (!slg4ax46073v)
		return -ENOMEM;

	dev_set_drvdata(dev, slg4ax46073v);

	// Enable FAN1 VDD power
	slg4ax46073v->bp_gpiod = devm_gpiod_get_optional(dev, "bp", GPIOD_OUT_LOW);
	if ( IS_ERR(slg4ax46073v->bp_gpiod ) ) {
		dev_err(dev, "failed to get fan bp gpio %ld\n",
			PTR_ERR(slg4ax46073v->bp_gpiod));
	} else {
		gpiod_set_value_cansleep(slg4ax46073v->bp_gpiod, 1);
	}
	slg4ax46073v->en_gpiod = devm_gpiod_get(dev, "en", GPIOD_OUT_LOW);
	if ( IS_ERR(slg4ax46073v->en_gpiod) ){
		dev_err(dev, "failed to get fan en gpio %ld\n",
			PTR_ERR(slg4ax46073v->en_gpiod));
		return PTR_ERR(slg4ax46073v->en_gpiod);
	} else {
        gpiod_set_value_cansleep(slg4ax46073v->en_gpiod, 1);
    }

	slg4ax46073v->regmap = devm_regmap_init_i2c(client, &slg4ax46073v_regmap_config);
	if (IS_ERR(slg4ax46073v->regmap)) {
		err = PTR_ERR(slg4ax46073v->regmap);
		dev_err(dev, "regmap init failed: %d", err);
		return err;
	}

    // need to wait > 2msec after VDD set to read the version
    // we attempt to read this multiple times as the actual start up time is variable
    {
        unsigned int i;
        for ( i = 0; i < 3; i++ ) {
            usleep_range(4000,5000);
            err = slg4ax46073v_get_version( slg4ax46073v, &slg4ax46073v->version );
            if ( err ) {
                dev_err(dev, "Failed to get version ( %d )\n", err);
                slg4ax46073v->version = 0;
            } else {
                dev_info(dev, "version: %x", slg4ax46073v->version );
                break;
            }
        }
    }

    // set up periodic resets of the wdt and tachometer reading
    slg4ax46073v_wdt_work_init(slg4ax46073v);

    if ( slg4ax46073v->version <= VERSION_7 ) {

        // setup fan alert irq
        slg4ax46073v->fan_alert = devm_gpiod_get(dev, "fan-alert", GPIOD_IN);
        if (IS_ERR(slg4ax46073v->fan_alert)) {
            dev_err(dev, "cannot get fan alert gpio %ld\n",
                    PTR_ERR(slg4ax46073v->fan_alert));
            return PTR_ERR(slg4ax46073v->fan_alert);
        }

        if ((err = gpiod_direction_input(slg4ax46073v->fan_alert)) !=0) {
            gpiod_put(slg4ax46073v->fan_alert);
            dev_err(dev, "cannot set fan alert gpio to input %ld\n", PTR_ERR(slg4ax46073v->fan_alert));
            return PTR_ERR(slg4ax46073v->fan_alert);
        }

        slg4ax46073v->fan_alert_gpio_irq = gpiod_to_irq(slg4ax46073v->fan_alert);
        if ( slg4ax46073v->version >= VERSION_7 ) {
            err = devm_request_threaded_irq(dev, slg4ax46073v->fan_alert_gpio_irq, fan_alert_irq_handler_top, fan_alert_irq_handler,
                                                                IRQF_TRIGGER_FALLING, "FAN_ALERT", client);
        } else {
            err = devm_request_threaded_irq(dev, slg4ax46073v->fan_alert_gpio_irq, fan_alert_irq_handler_top, fan_alert_irq_handler,
                                                                IRQF_TRIGGER_RISING, "FAN_ALERT", client);
        }

        if (err) {
            dev_err(dev, "request fan alert interrupt failed with %d\n", err);
            gpiod_put(slg4ax46073v->fan_alert);
            return err;
        }
    } else {
        slg4ax46073v_reset(slg4ax46073v);
    }

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							 slg4ax46073v,
							 &slg4ax46073v_chip_info,
							 slg4ax46073v_groups);

	if (IS_ERR(hwmon_dev)) {
		dev_err(dev, "unable to register hwmon device\n");
		return PTR_ERR(hwmon_dev);
	}

	return PTR_ERR_OR_ZERO(hwmon_dev);;
}

static void slg4ax46073v_remove(struct i2c_client *client) {
	struct slg4ax46073v * slg4ax46073v = dev_get_drvdata(&client->dev);
	cancel_delayed_work(&slg4ax46073v->wdt_work);
	slg4ax46073v_pet_wdt( slg4ax46073v );
}

static int slg4ax46073v_suspend(struct device *dev)
{
	struct slg4ax46073v *slg4ax46073v = dev_get_drvdata(dev);
	u32 duty, period, enabled;
	int ret;

	ret = regmap_read(slg4ax46073v->regmap, SLG4AX46073V_REG_VIRTUAL_INPUT, &enabled);
	if (!ret) {
		ret = regmap_read(slg4ax46073v->regmap, SLG4AX46073V_REG_PWM_PERIOD, &period);
		if (ret)
			return ret;

		ret = regmap_read(slg4ax46073v->regmap, SLG4AX46073V_REG_PWM_DC, &duty);
		if (ret)
			return ret;

		regmap_write(slg4ax46073v->regmap, SLG4AX46073V_REG_VIRTUAL_INPUT, SLG4AX46073V_VI_DISABLE_PWM_CLK0);

		// turn off fan power supply
		gpiod_set_value_cansleep(slg4ax46073v->en_gpiod, 0);
		usleep_range(100000,150000);

		// reset fan controller
		regmap_write(slg4ax46073v->regmap, SLG4AX46073V_REG_RESET, SLG4AX46073V_RESET);

		// disarm wdt
		slg4ax46073v_disarm_wdt(slg4ax46073v);

		// Cancel the wdt so the work handler does not run in suspend
		cancel_delayed_work(&slg4ax46073v->wdt_work);

		slg4ax46073v->pm_enabled = 1;
		slg4ax46073v->pm_duty = duty;
		slg4ax46073v->pm_period = period;
	} else
		slg4ax46073v->pm_enabled = 0;

	return 0;
}

static int slg4ax46073v_resume(struct device *dev)
{
	struct slg4ax46073v *slg4ax46073v = dev_get_drvdata(dev);

	if (slg4ax46073v->pm_enabled) {
		// turn fan power supply back on
		gpiod_set_value_cansleep(slg4ax46073v->en_gpiod, 1);
		usleep_range(100000,150000);

		slg4ax46073v_reset(slg4ax46073v);

		regmap_write(slg4ax46073v->regmap, SLG4AX46073V_REG_PWM_DC,slg4ax46073v->pm_duty);
		regmap_write(slg4ax46073v->regmap, SLG4AX46073V_REG_PWM_PERIOD,slg4ax46073v->pm_period);
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(slg4ax46073v_pm, slg4ax46073v_suspend, slg4ax46073v_resume);

static const struct i2c_device_id slg4ax46073v_i2c_ids[] = {
	{ "slg4ax46073v", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, slg4ax46073v_i2c_ids);

static const struct of_device_id slg4ax46073v_of_ids[] = {
	{ .compatible = "silego,slg4ax46073v", },
	{}
};
MODULE_DEVICE_TABLE(of, slg4ax46073v_of_ids);

static struct i2c_driver slg4ax46073v_basic_driver = {
	.driver = {
		.name	= "slg4ax46073v-fan",
		.of_match_table = of_match_ptr(slg4ax46073v_of_ids),
		.pm		= pm_sleep_ptr(&slg4ax46073v_pm),
	},
	.probe		= slg4ax46073v_probe,
	.id_table	= slg4ax46073v_i2c_ids,
	.remove     = slg4ax46073v_remove,
	.shutdown   = slg4ax46073v_remove,
};

module_i2c_driver(slg4ax46073v_basic_driver);

MODULE_DESCRIPTION("Valve Silego slg4ax46073v custom fan Driver");
MODULE_LICENSE("GPL");
