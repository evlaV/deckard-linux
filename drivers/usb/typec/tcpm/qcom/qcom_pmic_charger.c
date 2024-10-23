#include <linux/iio/consumer.h>
#include <linux/jiffies.h>

#define UPPER_8_BITS(data)      (((data) >> 8) & 0xFF)
#define LOWER_8_BITS(data)      ((data) & 0xFF)

#define CHGR_BASE 0x2600
#define BATIF_BASE 0x2800
#define USB_BASE 0x2900
#define BALANCE_BASE 0x3000

#define CHGR_CHG_EN_ADDR  (CHGR_BASE + 0x46)
#define CHGR_CHG_EN_CHARGING_PAUSE_CMD 	BIT(4)
#define CHGR_CHG_EN_CHARGING_ENABLE_CMD BIT(0)

#define CHGR_CHG_STATUS_ADDR  (CHGR_BASE + 0x06)

#define BALANCE_TARGET (BALANCE_BASE + 0x41)
#define BALANCE_PERIOD                 HZ*60

#define DISABLE_CELL_BALANCING          BIT(0)

enum pre_charge_current {
	IPRE_100MA,
	IPRE_150MA,
	IPRE_200MA,
	IPRE_250MA,
	// note: PM8550B has 100mA step size above 300mA
	IPRE_300MA,
	IPRE_400MA,
	IPRE_500MA,
	IPRE_600MA,
};
#define CHGR_IPRE_CFG_ADDR (CHGR_BASE + 0x50)

// fast charge current: 50mA units, max 10000mA
#define CHGR_ICHG_CFG_ADDR (CHGR_BASE + 0x54)
// float voltage: min 3600, max 4790, step 10
#define CHGR_VFLT_CFG_ADDR (CHGR_BASE + 0x58)

#define CHGR_VFLT_CFG_ADDR (CHGR_BASE + 0x58)

#define CHGR_TERM_CFG_ADDR (CHGR_BASE + 0x60)
#define CHGR_TERM_CFG_I_TERM 				BIT(7)
#define CHGR_TERM_CFG_USE_ANALOG_TERMINATION 		BIT(3)
#define CHGR_TERM_CFG_ANALOG_CHARGE_CURRENT_TERMINATION GENMASK(2,0) // 100mA+50mA*(x)

#define CHGR_WD_CFG_ADDR (CHGR_BASE + 0x94)
#define CHGR_WD_CFG_BITE_WDOG_DISABLE_CHARGING_CFG 	BIT(7) //disable charging if bite wdog expire
#define CHGR_WD_CFG_BITE_WDOG_INT_EN 			BIT(5)
#define CHGR_WD_CFG_BITE_BARK_WDOG_INT_EN 		BIT(4)
#define CHGR_WD_CFG_BITE_WDOG_TIMER_EN_ON_PLUGIN 	BIT(1)
#define CHGR_WD_CFG_BITE_WDOG_TIMER_EN 			BIT(0)

#define CHGR_BAT_2S_CHARGE_CFG_ADDR (CHGR_BASE + 0xa0)

#define BATIF_JEITA_EN (BATIF_BASE + 0x7F)
#define BATIF_JEITA_FV_WARM (BATIF_BASE + 0x80)
#define BATIF_JEITA_FV_COOL (BATIF_BASE + 0x81)
#define BATIF_JEITA_CC_WARM (BATIF_BASE + 0x82)
#define BATIF_JEITA_CC_COOL (BATIF_BASE + 0x83)
#define BATIF_JEITA_THOT_MSB (BATIF_BASE + 0x84)
#define BATIF_JEITA_THOT_LSB (BATIF_BASE + 0x85)
#define BATIF_JEITA_WARM_MSB (BATIF_BASE + 0x86)
#define BATIF_JEITA_WARM_LSB (BATIF_BASE + 0x87)
#define BATIF_JEITA_COOL_MSB (BATIF_BASE + 0x88)
#define BATIF_JEITA_COOL_LSB (BATIF_BASE + 0x89)
#define BATIF_JEITA_COLD_MSB (BATIF_BASE + 0x8A)
#define BATIF_JEITA_COLD_LSB (BATIF_BASE + 0x8B)
#define BATIF_JEITA_THOT_AFP_MSB (BATIF_BASE + 0x8C)
#define BATIF_JEITA_THOT_AFP_LSB (BATIF_BASE + 0x8D)
#define BATIF_JEITA_COLD_AFP_MSB (BATIF_BASE + 0x8E)
#define BATIF_JEITA_COLD_AFP_LSB (BATIF_BASE + 0x8F)

#define USB_USB_ICL_OVERRIDE_ADDR (USB_BASE + 0x51) // bit(0) OVERRIDE, and bit(4) for OVERRIDE_AFTER_APSD
#define USB_USB_ICL_CFG_ADDR (USB_BASE + 0x52) // 50mA steps, max 6000mA

typedef enum
{
	CHGR_CHG_STATUS_INHIBIT,
	CHGR_CHG_STATUS_TRICKLE,
	CHGR_CHG_STATUS_PRECHARGE,
	CHGR_CHG_STATUS_FULLON,
	CHGR_CHG_STATUS_TAPER,
	CHGR_CHG_STATUS_TERMINATION,
	CHGR_CHG_STATUS_PAUSE,
	CHGR_CHG_STATUS_CHARGING_DISABLED
} CHGR_CHG_STATUS;

const char charger_status_string [][16] = {
	"Inhibit",
	"Trickle",
	"Precharge",
	"Full On",
	"Taper",
	"Termination",
	"Paused",
	"Disabled"
};

struct pmic_charger {
	struct device      *dev;
	struct regmap      *regmap;
	struct iio_channel *vmid_ch;
	struct iio_channel *vbat_ch;
	struct delayed_work balance_work;
};

struct pmic_charger *chg;

static ssize_t charging_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	unsigned long num;

	if (kstrtoul(buf, 0, &num))
		return -EINVAL;

	if (num == 0) {
		// Disable Charging
		regmap_write(chg->regmap, CHGR_CHG_EN_ADDR, 0x00);
	} else {
		// Enable Charging
		regmap_write(chg->regmap, CHGR_CHG_EN_ADDR, CHGR_CHG_EN_CHARGING_ENABLE_CMD);
	}

	return count;
}

static ssize_t charging_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	ssize_t rc;
	uint reg;
	CHGR_CHG_STATUS status;

	regmap_read(chg->regmap, CHGR_CHG_STATUS_ADDR, &reg);
	status = (CHGR_CHG_STATUS)(reg & 0x07);
	rc = snprintf(buf, PAGE_SIZE, "%s", charger_status_string[status]);

	return rc;
}
static DEVICE_ATTR_RW(charging);

static ssize_t balancing_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	ssize_t rc;
	uint balance;
	regmap_read(chg->regmap, BALANCE_TARGET, &balance);
	rc = snprintf(buf, PAGE_SIZE, "%x", balance);
	return rc;
}
static DEVICE_ATTR_RO(balancing);

static ssize_t delta_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	ssize_t rc;
	int vbat,lower,upper,delta,ret;

	// read vbat and mid voltages
	ret = iio_read_channel_processed(chg->vbat_ch, &vbat);
	if (ret < 0) {
		pr_err("qcom-pmic: failed to read vbat_ch\n");
		return 0;
	}
	ret = iio_read_channel_processed(chg->vmid_ch, &lower);
	if (ret < 0) {
		pr_err("qcom-pmic: failed to read vmid_ch\n");
		return 0;
	}
	
	// calculate delta in mV    
	upper = vbat-lower;
	delta = ( upper - lower ) / 1000;
	rc = snprintf(buf, PAGE_SIZE, "%d", delta);
	return rc;
}
static DEVICE_ATTR_RO(delta);

static struct attribute *balancing_attrs[] = {
	&dev_attr_charging.attr,
	&dev_attr_balancing.attr,
	&dev_attr_delta.attr,
	NULL,
};

static const struct attribute_group balancing_attr_group = {
	.name = "balancing",
	.attrs = balancing_attrs,
};

void set_voltage(struct pmic_charger *chg, uint float_voltage, int delta)
{
	u32 abs_delta = abs(delta);
	uint voltage_reg = (float_voltage-3600 - abs_delta/2)/10;

	regmap_write(chg->regmap, CHGR_VFLT_CFG_ADDR, voltage_reg); // 4.45V float (8.9V for 2S)
}

int charger_connected(struct pmic_charger *chg)
{
	int ret;
	int val;

	// Read USB Sink Status
	ret = regmap_read(chg->regmap, 0x2B06, &val);
	if (ret) {
		// Flag disconnected on PMIC read failure
		return 0;
	}

	switch (val & DETECTED_SNK_TYPE_MASK) {
	case SNK_RP_STD:
	case SNK_RP_1P5:
	case SNK_RP_3P0:
		ret = 1;
		break;
	default:
		ret = 0;
		break;

	}

	return ret;
}


void balance_delayed_work_handler(struct work_struct *work)
{
	int ret;
	int vbat,lower,upper,delta;
	uint balance;
	struct pmic_charger *chg = container_of((struct delayed_work *)work, struct pmic_charger, balance_work);
	
	regmap_read(chg->regmap, BALANCE_TARGET, &balance);

	// Get Charger Status
	if (!charger_connected(chg)) {
		// Charger Disconnected
		if ( balance != 0x00 ) {
			pr_warn("qcom-pmic: charger disconnected, disabling cell balancing\n");
			regmap_write(chg->regmap, BALANCE_TARGET, 0x00);
		}

		goto end;
	}

	// read vbat and mid voltages
	ret = iio_read_channel_processed(chg->vbat_ch, &vbat);
	if (ret < 0) {
		pr_err("qcom-pmic: failed to read vbat_ch\n");
		goto end;
	}
	ret = iio_read_channel_processed(chg->vmid_ch, &lower);
	if (ret < 0) {
		pr_err("qcom-pmic: failed to read vmid_ch\n");
		goto end;
	}
	
	// calculate delta in mV    
	upper = vbat-lower;
	delta = ( upper - lower ) / 1000;

	// set balance target and float voltage according to delta
	if ( delta > 20 ) {
		set_voltage(chg, 4450, delta);
		if ( balance != 0x02 ) {
			pr_warn("qcom-pmic: setting balance target to upper\n");
			regmap_write(chg->regmap, BALANCE_TARGET, 0x02);
		}
	}
	else if ( delta < -20 ) {
		set_voltage(chg, 4450, delta);
		regmap_write(chg->regmap, BALANCE_TARGET, 0x01);
		if ( balance != 0x01 ) {
			pr_warn("qcom-pmic: setting balance target to lower\n");
			regmap_write(chg->regmap, BALANCE_TARGET, 0x01);
		}
	}
	else if ( delta < 10 && delta > -10 ) {
		set_voltage(chg, 4450, 0);
		regmap_write(chg->regmap, BALANCE_TARGET, 0x00);
	}

end:
	schedule_delayed_work(&chg->balance_work, BALANCE_PERIOD);
};

static void qcom_pmic_charger_setup_jeita(struct regmap *regmap)
{
	// Nominal Charging Window is 5 to 45 C
	// Extended Charging Window is 0 to 60 C. We limit this to 46, since the battery shouldn't get that hot.
	// Discharging Window is -20 to 65 C
	// In Extended Charging Window Current is limited to 0.2 C
	// At High End, 

	// TODO Test this, because at 2S voltages are typically still 1S levels in PMIC
	// 8.2 Volts is 700 mV below
	regmap_write(regmap, BATIF_JEITA_FV_WARM, (8900 - 8200) / 10);
	regmap_write(regmap, BATIF_JEITA_FV_COOL, 0);
	// TODO These need to be dynamic based on FCC and are an offset from it
	// TODO Verify the scaling here is really 50 mA and not 60 mA
	regmap_write(regmap, BATIF_JEITA_CC_WARM, 43);
	regmap_write(regmap, BATIF_JEITA_CC_COOL, 43);

	// These are from Beta Equation with 100k thermistor, 100k pullup, Beta=4250, and Full Scale ADC of 0x4000
	// const u16 temp_p75 = 0x0751;
	const u16 temp_p65 = 0x0A00;
	// const u16 temp_p60 = 0x0BB2;
	// const u16 temp_p50 = 0x0DAC;
	const u16 temp_p46 = 0x1201;
	const u16 temp_p45 = 0x128C;
	// const u16 temp_p44 = 0x131C;
	// const u16 temp_p25 = 0x2000;
	// const u16 temp_p10 = 0x2B8A;
	// const u16 temp_p6  = 0x2E69;
	const u16 temp_p5  = 0x2F19;
	const u16 temp_p4  = 0x2FC6;
	// const u16 temp_0   = 0x3257;
	const u16 temp_n20 = 0x3B4B;

	// Too Hot to Charge - 46 C
	regmap_write(regmap, BATIF_JEITA_THOT_MSB, UPPER_8_BITS(temp_p46));
	regmap_write(regmap, BATIF_JEITA_THOT_LSB, LOWER_8_BITS(temp_p46));

	// Too Warm for Full Charging - 45 C
	regmap_write(regmap, BATIF_JEITA_WARM_MSB, UPPER_8_BITS(temp_p45));
	regmap_write(regmap, BATIF_JEITA_WARM_LSB, LOWER_8_BITS(temp_p45));

	// Too Cool for Full Charging - 5 C
	regmap_write(regmap, BATIF_JEITA_COOL_MSB, UPPER_8_BITS(temp_p5));
	regmap_write(regmap, BATIF_JEITA_COOL_LSB, LOWER_8_BITS(temp_p5));

	// Too Cold to Charge - 4 C
	regmap_write(regmap, BATIF_JEITA_COLD_MSB, UPPER_8_BITS(temp_p4));
	regmap_write(regmap, BATIF_JEITA_COLD_LSB, LOWER_8_BITS(temp_p4));

	// Too Hot to Run - 65 C
	regmap_write(regmap, BATIF_JEITA_THOT_AFP_MSB, UPPER_8_BITS(temp_p65));
	regmap_write(regmap, BATIF_JEITA_THOT_AFP_LSB, LOWER_8_BITS(temp_p65));

	// Too Cold to Run - -20 C
	regmap_write(regmap, BATIF_JEITA_COLD_AFP_MSB, UPPER_8_BITS(temp_n20));
	regmap_write(regmap, BATIF_JEITA_COLD_AFP_LSB, LOWER_8_BITS(temp_n20));

	// Enable JEITA Window (Everything but Float Voltage Compenstation in Constant Voltage Mode)
	// This is the same thing XBL sets up.
	regmap_write(regmap, BATIF_JEITA_EN, 0xEF);
}

int qcom_pmic_charger_init(struct device *dev, struct regmap *regmap, uint32_t flags)
{
	int ret;
	int vbat,vmid,delta;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->regmap = regmap;
#if 1
	// just print the bootloader charging settings
	// expected: ipre=300mA, ichg=750mA, vflt=4350mV, term=300mA
#define PRINT_REG(x) ({ int ret, val; ret = regmap_read(regmap, x, &val); if (!ret) printk(#x "=%.2x", val); })
	PRINT_REG(CHGR_IPRE_CFG_ADDR);
	PRINT_REG(CHGR_ICHG_CFG_ADDR);
	PRINT_REG(CHGR_VFLT_CFG_ADDR);
	PRINT_REG(CHGR_TERM_CFG_ADDR);
	PRINT_REG(CHGR_WD_CFG_ADDR);
	PRINT_REG(CHGR_CHG_EN_ADDR);
#endif

	chg->vmid_ch = devm_iio_channel_get(dev, "vmid");
	if (IS_ERR(chg->vmid_ch)) {
		pr_warn("qcom-pmic: vmid get failed\n");
		goto enable_charger;
	}

	chg->vbat_ch = devm_iio_channel_get(dev, "vbat");
	if (IS_ERR(chg->vbat_ch)) {
		pr_warn("qcom-pmic: vbat get failed\n");
		goto enable_charger;
	}

	ret = sysfs_create_group(&dev->kobj, &balancing_attr_group);
	if (ret < 0) {
		pr_err("qcom-pmic: Can't register sysfs attr group: %d\n", ret);
	}

enable_charger:
	// Disable QBG - This lowers ADC Reads and Reads on VMid
	regmap_write(regmap, 0x4F46, 0x00);

	qcom_pmic_charger_setup_jeita(regmap);

	regmap_write(regmap, CHGR_IPRE_CFG_ADDR, IPRE_250MA); // (250 mA for 2S?)
	regmap_write(regmap, CHGR_ICHG_CFG_ADDR, 1320/60); // 1.32 A for 2S? NOTE This might actually be 50 mA steps
	regmap_write(regmap, CHGR_VFLT_CFG_ADDR, (4450-3600)/10); // 4.45V float (8.9V for 2S)
	regmap_write(regmap, CHGR_TERM_CFG_ADDR, CHGR_TERM_CFG_I_TERM); // enabled and min (100mA)

	regmap_write(regmap, CHGR_WD_CFG_ADDR, CHGR_WD_CFG_BITE_WDOG_DISABLE_CHARGING_CFG); // disable watchdog

	// Clear Latched Registers (???)
	// regmap_write(regmap, 0x26af, 0xff);

	//regmap_write(regmap, CHGR_CHG_EN_ADDR, 0); // disable charging
	// sleep 10 seconds ??
	regmap_write(regmap, CHGR_CHG_EN_ADDR, CHGR_CHG_EN_CHARGING_ENABLE_CMD); // enable charging

	if ( ( flags & DISABLE_CELL_BALANCING ) != DISABLE_CELL_BALANCING ) {
		ret = iio_read_channel_processed(chg->vbat_ch, &vbat);
		if (ret < 0) {
			pr_warn("qcom-pmic: failed to read vbat\n");
			goto enable_charger;
		}

		ret = iio_read_channel_processed(chg->vmid_ch, &vmid);
		if (ret < 0) {
			pr_warn("qcom-pmic: failed to read vmid\n");
			goto enable_charger;
		}

		delta = abs( vmid - ( vbat - vmid ) ) / 1000;

		if ( delta > 20 ) {
			pr_warn("qcom-pmic: Battery cell imbalance detected: %d mV\n", delta);
		}

		INIT_DELAYED_WORK(&chg->balance_work, balance_delayed_work_handler);
		schedule_delayed_work(&chg->balance_work, BALANCE_PERIOD / 4);
	}

	return 0;
}
