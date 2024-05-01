#include <linux/iio/consumer.h>

#define CHGR_BASE 0x2600
#define USB_BASE 0x2900

#define CHGR_CHG_EN_ADDR  (CHGR_BASE + 0x46)
#define CHGR_CHG_EN_CHARGING_PAUSE_CMD 	BIT(4)
#define CHGR_CHG_EN_CHARGING_ENABLE_CMD BIT(0)

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

#define USB_USB_ICL_OVERRIDE_ADDR (USB_BASE + 0x51) // bit(0) OVERRIDE, and bit(4) for OVERRIDE_AFTER_APSD
#define USB_USB_ICL_CFG_ADDR (USB_BASE + 0x52) // 50mA steps, max 6000mA

struct pmic_charger {
	struct device		*dev;
	struct iio_channel *vmid_ch;
	struct iio_channel *vbat_ch;
};


int qcom_pmic_charger_init(struct device *dev, struct regmap *regmap)
{
	struct pmic_charger *chg;
	int ret;
	int result;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

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
	// settings todd posted on slack:

	chg->vmid_ch = devm_iio_channel_get(dev, "vmid");
	if (IS_ERR(chg->vmid_ch)) {
		pr_warn("vmid get failed");
	} else {
		pr_warn("vmid get success");
	}

	chg->vbat_ch = devm_iio_channel_get(dev, "vbat");
	if (IS_ERR(chg->vbat_ch)) {
		pr_warn("vbat get failed");
	} else {
		pr_warn("vbat get success");
	}

	ret = iio_read_channel_processed(chg->vbat_ch, &result);
	if (ret < 0)
		return ret;

	pr_warn("vbat: %07d", result);

	ret = iio_read_channel_processed(chg->vmid_ch, &result);
	if (ret < 0)
		return ret;

	pr_warn("vmid: %07d", result);


	regmap_write(regmap, CHGR_IPRE_CFG_ADDR, IPRE_250MA); // (250 mA for 2S?)
	regmap_write(regmap, CHGR_ICHG_CFG_ADDR, 1320/60); // 1.32 A for 2S?
	regmap_write(regmap, CHGR_VFLT_CFG_ADDR, (4450-3600)/10); // 4.45V float (8.9V for 2S)
	regmap_write(regmap, CHGR_TERM_CFG_ADDR, CHGR_TERM_CFG_I_TERM); // enabled and min (100mA)

	regmap_write(regmap, CHGR_WD_CFG_ADDR, CHGR_WD_CFG_BITE_WDOG_DISABLE_CHARGING_CFG); // disable watchdog

	// Clear Latched Registers (???)
	// regmap_write(regmap, 0x26af, 0xff);

	//regmap_write(regmap, CHGR_CHG_EN_ADDR, 0); // disable charging
	// sleep 10 seconds ??
	regmap_write(regmap, CHGR_CHG_EN_ADDR, CHGR_CHG_EN_CHARGING_ENABLE_CMD); // enable charging

	return 0;
}
