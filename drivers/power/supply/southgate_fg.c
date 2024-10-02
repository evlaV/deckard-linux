// #include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
// #include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/iio/consumer.h>

#define DRV_NAME "southgate"

// Loaded Southgate Battery Voltage When New
static const int soc_voltage_lut[] = {
	[0] = 5993713,	 [1] = 6356419,	 [2] = 6519049,	 [3] = 6633968,
	[4] = 6715422,	 [5] = 6775099,	 [6] = 6818571,	 [7] = 6854963,
	[8] = 6884556,	 [9] = 6907888,	 [10] = 6928726, [11] = 6946724,
	[12] = 6962542,	 [13] = 6976204, [14] = 6989580, [15] = 7002291,
	[16] = 7014286,	 [17] = 7025108, [18] = 7036093, [19] = 7046659,
	[20] = 7056330,	 [21] = 7066277, [22] = 7075977, [23] = 7085430,
	[24] = 7094213,	 [25] = 7103348, [26] = 7112337, [27] = 7120813,
	[28] = 7129709,	 [29] = 7138613, [30] = 7147675, [31] = 7156389,
	[32] = 7165720,	 [33] = 7175289, [34] = 7184647, [35] = 7194742,
	[36] = 7205166,	 [37] = 7215923, [38] = 7226416, [39] = 7237883,
	[40] = 7249653,	 [41] = 7261814, [42] = 7273726, [43] = 7286644,
	[44] = 7299986,	 [45] = 7312878, [46] = 7326990, [47] = 7341450,
	[48] = 7356230,	 [49] = 7370640, [50] = 7386181, [51] = 7402140,
	[52] = 7417622,	 [53] = 7434248, [54] = 7451351, [55] = 7468723,
	[56] = 7485653,	 [57] = 7503750, [58] = 7522148, [59] = 7539896,
	[60] = 7559067,	 [61] = 7578560, [62] = 7598360, [63] = 7617500,
	[64] = 7637871,	 [65] = 7658566, [66] = 7679354, [67] = 7699495,
	[68] = 7720694,	 [69] = 7742149, [70] = 7762453, [71] = 7784073,
	[72] = 7805750,	 [73] = 7827497, [74] = 7847988, [75] = 7868101,
	[76] = 7891132,	 [77] = 7911295, [78] = 7932533, [79] = 7953396,
	[80] = 7972345,	 [81] = 7992872, [82] = 8013006, [83] = 8032196,
	[84] = 8049957,	 [85] = 8068144, [86] = 8083931, [87] = 8101477,
	[88] = 8116167,	 [89] = 8134022, [90] = 8149233, [91] = 8164236,
	[92] = 8176041,	 [93] = 8192533, [94] = 8204882, [95] = 8219154,
	[96] = 8235035,	 [97] = 8252947, [98] = 8274612, [99] = 8310051,
	[100] = 8445625,
};

// ESR of the pack is 110 mOhms
// ESR of pack to motherboard is 186 mOhms
const int esr = 186;

struct southgate_priv {
	struct device *dev;
	struct power_supply *battery;
	struct iio_channel *vbat_ch;
	struct iio_channel *ibat_ch;
};

static enum power_supply_property southgate_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static int get_capacity(struct southgate_priv *priv)
{
	int ret;
	int bat_current;
	int voltage;
	int soc;

	ret = iio_read_channel_processed(priv->vbat_ch, &voltage);
	if (ret < 0)
		return ret;

	ret = iio_read_channel_processed(priv->ibat_ch, &bat_current);
	if (ret < 0)
		return ret;

	// Fuel Gauge LUT corresponds to battery at 2.7 Amps
	bat_current -= 2700000;
	voltage = voltage + (bat_current * esr) / 1000;

	for (soc = 0; soc < ARRAY_SIZE(soc_voltage_lut); soc++) {
		// pr_warn("southgate soc: %d %d\n", soc, soc_voltage_lut[soc]);
		if (voltage < soc_voltage_lut[soc]) {
			soc--;
			break;
		}
	}

	return soc;
}

static int southgate_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct southgate_priv *priv = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = iio_read_channel_processed(priv->vbat_ch, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = iio_read_channel_processed(priv->ibat_ch, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_capacity(priv);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct power_supply_desc southgate_fg_desc = {
	.name = "southgate_battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = southgate_battery_props,
	.num_properties = ARRAY_SIZE(southgate_battery_props),
	.get_property = southgate_get_property,
};

static int southgate_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct southgate_priv *priv;
	struct power_supply_config psy_cfg = {};
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = dev;

	priv->vbat_ch = devm_iio_channel_get(dev, "vbat");
	if (IS_ERR(priv->vbat_ch)) {
		pr_warn("southgate_probe: vbat get failed\n");
		goto err;
	}

	priv->ibat_ch = devm_iio_channel_get(dev, "ibat");
	if (IS_ERR(priv->ibat_ch)) {
		pr_warn("southgate_probe: ibat get failed\n");
		goto err;
	}

	psy_cfg.drv_data = priv;
	priv->battery =
		power_supply_register(dev, &southgate_fg_desc, &psy_cfg);
	if (IS_ERR(priv->battery)) {
		ret = PTR_ERR(priv->battery);
		dev_err(dev, "failed to register fuel gauge: %d\n", ret);
		goto err_supply;
	}

	return 0;

err_supply:
	power_supply_unregister(priv->battery);
err:

	return ret;
}

static void southgate_remove(struct platform_device *pdev)
{
	struct southgate_priv *priv = platform_get_drvdata(pdev);

	power_supply_unregister(priv->battery);
}

#ifdef CONFIG_OF
static const struct of_device_id southgate_match[] = {
	{
		.compatible = "deckard,southgate_fg",
	},
	{},
};
MODULE_DEVICE_TABLE(of, southgate_match);
#endif

static struct platform_driver southgate_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(southgate_match),
	},
	.probe = southgate_probe,
	.remove = southgate_remove,
};
module_platform_driver(southgate_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("todd@tgoyen.com");
MODULE_DESCRIPTION("Southgate Fuel Gauge Driver");
