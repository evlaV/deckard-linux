
#define UPPER_8_BITS(data) (((data) >> 8) & 0xFF)
#define LOWER_8_BITS(data) ((data) & 0xFF)

#define BATIF_BASE 0x2800

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
	const u16 temp_p5 = 0x2F19;
	const u16 temp_p4 = 0x2FC6;
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

int qcom_pmic_charger_init(struct regmap *regmap)
{
	// Disable QBG - This lowers ADC Reads and Reads on VMid
	regmap_write(regmap, 0x4F46, 0x00);

	qcom_pmic_charger_setup_jeita(regmap);

	return 0;
}
