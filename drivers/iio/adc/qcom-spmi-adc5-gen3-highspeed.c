// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
// #include <linux/thermal.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/adc/qcom-vadc-common.h>

#include <dt-bindings/iio/adc/qcom,spmi-vadc.h>

#define ADC5_GEN3_HS 0x45
#define ADC5_GEN3_HS_BUSY BIT(7)
#define ADC5_GEN3_HS_READY BIT(0)

#define ADC5_GEN3_STATUS1 0x46
#define ADC5_GEN3_STATUS1_CONV_FAULT BIT(7)
#define ADC5_GEN3_STATUS1_THR_CROSS BIT(6)
#define ADC5_GEN3_STATUS1_EOC BIT(0)

#define ADC5_GEN3_EOC_STS 0x4a
#define ADC5_GEN3_EOC_CHAN_0 BIT(0)

#define ADC5_GEN3_EOC_CLR 0x4b

#define ADC5_GEN3_CONV_ERR_CLR 0x4e
#define ADC5_GEN3_CONV_ERR_CLR_REQ BIT(0)

#define ADC5_GEN3_SID 0x4f
#define ADC5_GEN3_SID_MASK GENMASK(3, 0)

#define ADC5_GEN3_PERPH_CH 0x50
#define ADC5_GEN3_CHAN_CONV_REQ BIT(7)

#define ADC5_GEN3_TIMER_SEL 0x51
#define ADC5_GEN3_TIME_IMMEDIATE 0x1

#define ADC5_GEN3_DIG_PARAM 0x52
#define ADC5_GEN3_DIG_PARAM_CAL_SEL_MASK GENMASK(5, 4)
#define ADC5_GEN3_DIG_PARAM_CAL_SEL_SHIFT 4
#define ADC5_GEN3_DIG_PARAM_DEC_RATIO_SEL_MASK GENMASK(3, 2)
#define ADC5_GEN3_DIG_PARAM_DEC_RATIO_SEL_SHIFT 2

#define ADC5_GEN3_FAST_AVG 0x53
#define ADC5_GEN3_FAST_AVG_CTL_EN BIT(7)
#define ADC5_GEN3_FAST_AVG_CTL_SAMPLES_MASK GENMASK(2, 0)

#define ADC5_GEN3_ADC_CH_SEL_CTL 0x54

#define ADC5_GEN3_DELAY_CTL 0x55
#define ADC5_GEN3_HW_SETTLE_DELAY_MASK 0xf

#define ADC5_GEN3_CH_EN 0x56

#define ADC5_GEN3_LOW_THR0 0x57

#define ADC5_GEN3_LOW_THR1 0x58

#define ADC5_GEN3_HIGH_THR0 0x59

#define ADC5_GEN3_HIGH_THR1 0x5a

#define ADC5_GEN3_CH0_DATA0 0x5c
#define ADC5_GEN3_CH0_DATA1 0x5d
#define ADC5_GEN3_CH1_DATA0 0x5e
#define ADC5_GEN3_CH1_DATA1 0x5f
#define ADC5_GEN3_CH2_DATA0 0x60
#define ADC5_GEN3_CH2_DATA1 0x61
#define ADC5_GEN3_CH3_DATA0 0x62
#define ADC5_GEN3_CH3_DATA1 0x63
#define ADC5_GEN3_CH4_DATA0 0x64
#define ADC5_GEN3_CH4_DATA1 0x65
#define ADC5_GEN3_CH5_DATA0 0x66
#define ADC5_GEN3_CH5_DATA1 0x67
#define ADC5_GEN3_CH6_DATA0 0x68
#define ADC5_GEN3_CH6_DATA1 0x69
#define ADC5_GEN3_CH7_DATA0 0x6a
#define ADC5_GEN3_CH7_DATA1 0x6b

#define ADC5_GEN3_CONV_REQ 0xe5
#define ADC5_GEN3_CONV_REQ_REQ BIT(0)

#define ADC5_GEN3_SID_OFFSET 0x8
#define ADC5_GEN3_CHANNEL_MASK 0xff
#define V_CHAN(x) (((x).sid << ADC5_GEN3_SID_OFFSET) | (x).channel)

#define DEFAULT_SAMPLE_DELAY_USEC 1000
#define DATA_LENGTH (24000)
#define POWER_DATA_LENGTH 50

enum adc5_cal_method { ADC5_NO_CAL = 0, ADC5_RATIOMETRIC_CAL, ADC5_ABSOLUTE_CAL };

enum adc_time_select {
	MEAS_INT_DISABLE = 0,
	MEAS_INT_IMMEDIATE,
	MEAS_INT_50MS,
	MEAS_INT_100MS,
	MEAS_INT_1S,
	MEAS_INT_NONE,
};

enum adc_state {
	ADC_STATE_IDLE,
	ADC_STATE_SAMPLING,
	ADC_STATE_COMPLETE,
};

/**
 * struct adc5_channel_prop - ADC channel property.
 * @channel: channel number, refer to the channel list.
 * @cal_method: calibration method.
 * @decimation: sampling rate supported for the channel.
 * @sid: slave id of PMIC owning the channel, for PMIC5 Gen2 and above.
 * @prescale: channel scaling performed on the input signal.
 * @hw_settle_time: the time between AMUX being configured and the
 *	start of conversion.
 * @avg_samples: ability to provide single result from the ADC
 *	that is an average of multiple measurements.
 * @scale_fn_type: Represents the scaling function to convert voltage
 *	physical units desired by the client for the channel.
 * @datasheet_name: Channel name used in device tree.
 * @chip: pointer to top-level ADC device structure.
 * @channel_index: Channel number used (ranging from 0-7).
 * @buffer: buffer data to configure channel
 */
struct adc5_channel_prop {
	unsigned int channel;
	enum adc5_cal_method cal_method;
	unsigned int decimation;
	unsigned int sid;
	unsigned int prescale;
	unsigned int hw_settle_time;
	unsigned int avg_samples;

	enum vadc_scale_fn_type scale_fn_type;
	const char *datasheet_name;

	struct adc5_chip_hs *chip;
	unsigned int channel_index;

	u8 buffer[7];
};

struct ibatt_trace_data {
	ktime_t t;
	int16_t data[4];
};

/**
 * struct adc5_chip_hs - ADC private structure.
 * @regmap: SPMI ADC5 peripheral register map field.
 * @dev: SPMI ADC5 device.
 * @base: pointer to array of ADC peripheral base and interrupt.
 * @nchannels: number of ADC channels.
 * @chan_props: array of ADC channel properties.
 * @iio_chans: array of IIO channels specification.
 * @complete: ADC result notification after interrupt is received.
 * @lock: ADC lock for access to the peripheral.
 * @data: software configuration data.
 */
struct adc5_chip_hs {
	struct regmap *regmap;
	struct device *dev;

	u16 base_addr;
	const char *irq_name;
	int irq;

	/* Channels */
	unsigned int nchannels;
	struct adc5_channel_prop *chan_props;
	struct iio_chan_spec *iio_chans;
	struct completion complete;
	struct mutex lock;
	const struct adc5_data *data;

	struct hrtimer sample_timer;
	ktime_t sample_period; /* Use specificed or default sample period. When off, keep running at slow_period */
	ktime_t slow_period; /* 1 Hz always on period */
	ktime_t start_time;
	enum adc_state state;
	int skipped_conversions;

	// TODO TGoyen Ensure all of these are allocated
	int timer_state;
	uint include_usb;

	int data_idx;
	struct ibatt_trace_data sample_data[DATA_LENGTH];
	int wrapbuffer; // data is circular

	int power_data[POWER_DATA_LENGTH];
	int power_data_samples;
};

static int adc5_read(struct adc5_chip_hs *adc, u16 offset, u8 *data, int len)
{
	int ret;

	ret = regmap_bulk_read(adc->regmap, adc->base_addr + offset, data, len);
	if (ret < 0)
		pr_err("adc read to register 0x%x of length:%d failed, ret=%d\n", offset, len, ret);

	return ret;
}

static int adc5_write(struct adc5_chip_hs *adc, u16 offset, u8 *data, int len)
{
	int ret;

	ret = regmap_bulk_write(adc->regmap, adc->base_addr + offset, data, len);
	if (ret < 0)
		pr_err("adc write to register 0x%x of length:%d failed, ret=%d\n", offset, len, ret);

	return ret;
}

/* Get the number of skipped conversions */
static ssize_t skipped_conversions_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adc5_chip_hs *adc = iio_priv(indio_dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", adc->skipped_conversions);
}

/* Get the ring bufffer index in data */
static ssize_t idx_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adc5_chip_hs *adc = iio_priv(indio_dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", adc->data_idx);
}

static void stop_acquisition(struct adc5_chip_hs *adc)
{
	// Clear any active acquisition
	u8 val;
	int ret;

	hrtimer_cancel(&adc->sample_timer);

	ret = mutex_lock_interruptible(&adc->lock);
	if (ret != 0)
		return;

	adc->state = ADC_STATE_IDLE;

	/* Clear EOC Status and let the PMK know any previous conversion is complete */
	val = 0;
	adc5_write(adc, ADC5_GEN3_EOC_STS, &val, 1);
	adc5_write(adc, ADC5_GEN3_PERPH_CH, &val, 1);
	val = ADC5_GEN3_CONV_REQ_REQ;
	adc5_write(adc, ADC5_GEN3_CONV_REQ, &val, 1);

	mutex_unlock(&adc->lock);
}

/* Start High Speed Acquisition of Data until the buffer fills up */
static ssize_t start_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adc5_chip_hs *adc = iio_priv(indio_dev);

	uint work_delay_usec, include_usb;
	int ret;

	// Clear any active acquisition
	if (adc->timer_state) {
		stop_acquisition(adc);
	}

	pr_info("starting power trace\n");

	ret = sscanf(buf, "%d %d", &include_usb, &work_delay_usec);
	if (ret != 2) {
		include_usb = 1;
		work_delay_usec = DEFAULT_SAMPLE_DELAY_USEC;
		pr_info("power profile default to include usb with %d usec sample delay\n", work_delay_usec);
	} else {
		if (work_delay_usec == 0) {
			work_delay_usec = DEFAULT_SAMPLE_DELAY_USEC;
		};
		pr_info("battery profile usb included %d with %d usec between adc reads\n", include_usb,
			work_delay_usec);
	}

	// Store the Settings
	adc->include_usb = include_usb;
	adc->sample_period = ktime_set(0, work_delay_usec * NSEC_PER_USEC);

	// Restart the Sample Timer
	memset(adc->sample_data, 0, sizeof(adc->sample_data));
	adc->data_idx = 0;
	adc->skipped_conversions = 0;
	adc->wrapbuffer = 0;
	adc->timer_state = 1;
	hrtimer_start(&adc->sample_timer, adc->sample_period, HRTIMER_MODE_REL);

	return count;
}

static ssize_t stop_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adc5_chip_hs *adc = iio_priv(indio_dev);

	// Clear any active acquisition
	if (adc->timer_state) {
		stop_acquisition(adc);
	}

	adc->timer_state = 0; //stopped

	pr_info("stopping battery power trace\n");

	return count;
}

static ssize_t wrapbuffer_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adc5_chip_hs *adc = iio_priv(indio_dev);

	bool v;

	adc->wrapbuffer = 0;
	if (kstrtobool(buf, &v))
		return -EINVAL;

	adc->wrapbuffer = v;

	return count;
}

// get the average power in mW over the previous num_samples
static int32_t get_power_avg(struct adc5_chip_hs *adc, int num_samples_requested)
{
	int64_t average_power = 0;
	int num_samples = num_samples_requested;

	// cannot read more than DATA_LENGTH after wrapping or more than data_idx otherwise
	if (adc->wrapbuffer && num_samples_requested > DATA_LENGTH) {
		num_samples = DATA_LENGTH;
	} else if (!adc->wrapbuffer && num_samples_requested > adc->data_idx) {
		num_samples = adc->data_idx;
	}

	for (int i = (adc->data_idx - num_samples + DATA_LENGTH); i < (adc->data_idx + DATA_LENGTH); i++) {
		average_power +=
			((adc->sample_data[i % DATA_LENGTH].data[0] * adc->sample_data[i % DATA_LENGTH].data[1]) / 1000);
	}

	return (int32_t)(average_power / num_samples);
}

static ssize_t power_avg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adc5_chip_hs *adc = iio_priv(indio_dev);

	u32 power_avg;

	// if we have samples to average, return the avg power
	if (adc->data_idx) {
		power_avg = get_power_avg(adc, adc->data_idx);
		return scnprintf(buf, PAGE_SIZE, "%d\n", power_avg); //mw
	} else {
		return scnprintf(buf, PAGE_SIZE, "0\n");
	}

	return 0;
}

static ssize_t power_avg_recent_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adc5_chip_hs *adc = iio_priv(indio_dev);

	u32 power_avg;

	// if we have samples to average, return the recent avg power
	if (adc->data_idx) {
		power_avg = get_power_avg(adc, 100);
		return scnprintf(buf, PAGE_SIZE, "%d\n", power_avg); //mw
	} else {
		return scnprintf(buf, PAGE_SIZE, "0\n");
	}

	return 0;
}

/* Show the timer state */
static ssize_t timer_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adc5_chip_hs *adc = iio_priv(indio_dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", adc->timer_state);
}

static ssize_t dump_read(struct file *file, struct kobject *obj, const struct bin_attribute *attr, char *buf, loff_t off,
			 size_t count)
{
	struct adc5_chip_hs *adc = iio_priv(dev_to_iio_dev(kobj_to_dev(obj)));

	/* Clamp Count to available data */
	count = min(count, ((size_t)sizeof(adc->sample_data) - off));
	if (count <= 0)
		return 0;

	// Copy data to userspace
	memcpy(buf, (void *)adc->sample_data + off, count);

	return count;
}

static IIO_DEVICE_ATTR_RO(skipped_conversions, 0);
static IIO_DEVICE_ATTR_RO(idx, 0);
static IIO_DEVICE_ATTR_WO(start, 0);
static IIO_DEVICE_ATTR_WO(stop, 0);
static IIO_DEVICE_ATTR_WO(wrapbuffer, 0);
static IIO_DEVICE_ATTR_RO(power_avg, 0);
static IIO_DEVICE_ATTR_RO(power_avg_recent, 0);
static IIO_DEVICE_ATTR_RO(timer_state, 0);

static BIN_ATTR_RO(dump, DATA_LENGTH * sizeof(struct ibatt_trace_data));

// clang-format off
static struct attribute *attrs[] = {
	&iio_dev_attr_skipped_conversions.dev_attr.attr,
	&iio_dev_attr_idx.dev_attr.attr,
	&iio_dev_attr_start.dev_attr.attr,
	&iio_dev_attr_stop.dev_attr.attr,
	&iio_dev_attr_wrapbuffer.dev_attr.attr,
	&iio_dev_attr_power_avg.dev_attr.attr,
	&iio_dev_attr_power_avg_recent.dev_attr.attr,
	&iio_dev_attr_timer_state.dev_attr.attr,
	NULL,
};
// clang-format on

static const struct bin_attribute *bin_attrs[] = {
	&bin_attr_dump,
	NULL,
};

static const struct attribute_group adc5_gen3_hs_attr_group = {
	.attrs = attrs,
	.bin_attrs = bin_attrs,
};

static const struct iio_info adc5_gen3_hs_info = {
	.attrs = &adc5_gen3_hs_attr_group,
};

static int adc5_hw_settle_time_from_dt(u32 value, const unsigned int *hw_settle)
{
	unsigned int i;

	for (i = 0; i < VADC_HW_SETTLE_SAMPLES_MAX; i++) {
		if (value == hw_settle[i])
			return i;
	}

	return -ENOENT;
}

static int adc5_avg_samples_from_dt(u32 value)
{
	if (!is_power_of_2(value) || value > ADC5_AVG_SAMPLES_MAX)
		return -EINVAL;

	return __ffs(value);
}

static int adc5_decimation_from_dt(u32 value, const unsigned int *decimation)
{
	unsigned int i;

	for (i = 0; i < ADC5_DECIMATION_SAMPLES_MAX; i++) {
		if (value == decimation[i])
			return i;
	}

	return -ENOENT;
}

static int adc5_gen3_hs_sample_channel(struct adc5_chip_hs *adc, struct adc5_channel_prop *chan_props)
{
	int ret;
	u8 conv_req = 0;

	ret = adc5_write(adc, ADC5_GEN3_SID, chan_props->buffer, sizeof(chan_props->buffer));
	if (ret < 0)
		return ret;

	// Tell the PMK8650 to start the conversion on the desired PMIC ...
	conv_req = ADC5_GEN3_CONV_REQ_REQ;
	ret = adc5_write(adc, ADC5_GEN3_CONV_REQ, &conv_req, 1);

	return ret;
}

static irqreturn_t adc5_gen3_hs_isr(int irq, void *dev_id)
{
	struct adc5_chip_hs *adc = dev_id;
	u8 status, eoc_status, val;
	int ret;
	int last_channel;

	/* Check End of Conversion (EOC) Status */
	ret = adc5_read(adc, ADC5_GEN3_EOC_STS, &eoc_status, 1);
	if (ret < 0) {
		pr_err("adc read eoc status failed with %d\n", ret);
		goto handler_end;
	}

	/* Determine the last channel sampled 
       EOC status is a bit field, where a set bit indicates
       that channel has finished a conversion */
	last_channel = __builtin_ctz(eoc_status);
    // make sure we got at least one end of conversion
    if ( last_channel > adc->nchannels ) {
        // check for conversion fault
        ret = adc5_read(adc, ADC5_GEN3_STATUS1, &status, 1);
        if (ret < 0) {
            pr_err("adc read status1 failed with %d\n", ret);
            goto handler_end;
        }

        if (status & ADC5_GEN3_STATUS1_CONV_FAULT) {
            pr_err_ratelimited("Unexpected conversion fault, status:%#x, eoc_status:%#x\n", status, eoc_status);

            /* Clear Active Conversion */
            adc->state = ADC_STATE_IDLE;

            val = ADC5_GEN3_CONV_ERR_CLR_REQ;
            ret = adc5_write(adc, ADC5_GEN3_CONV_ERR_CLR, &val, 1);
            if (ret < 0)
                goto handler_end;

            /* To indicate conversion request is only to clear a status */
            val = 0;
            ret = adc5_write(adc, ADC5_GEN3_PERPH_CH, &val, 1);
            if (ret < 0)
                goto handler_end;

            val = ADC5_GEN3_CONV_REQ_REQ;
            ret = adc5_write(adc, ADC5_GEN3_CONV_REQ, &val, 1);
            if (ret < 0)
                goto handler_end;
        } else {
            goto handler_end;
        }

    } else if ( adc->state == ADC_STATE_SAMPLING ) {
		/* Clear EOC Status and let the PMK now conversion is complete */
		val = 0;
		adc5_write(adc, ADC5_GEN3_EOC_STS, &val, 1);
		adc5_write(adc, ADC5_GEN3_PERPH_CH, &val, 1);
		val = ADC5_GEN3_CONV_REQ_REQ;
		adc5_write(adc, ADC5_GEN3_CONV_REQ, &val, 1);

        // we finished our last channel conversion
        if ( last_channel == 0 ) {
            /* Flag Transaction complete */
            adc->state = ADC_STATE_COMPLETE;
        } else {
            /* Sample the next channel */
		    adc5_gen3_hs_sample_channel(adc, &adc->chan_props[last_channel - 1]);
        }
    }

	return IRQ_HANDLED;

handler_end:
	return IRQ_NONE;
}

struct adc5_channels {
	const char *datasheet_name;
	unsigned int prescale_index;
	enum iio_chan_type type;
	long info_mask;
	enum vadc_scale_fn_type scale_fn_type;
};

/* In these definitions, _pre refers to an index into adc5_prescale_ratios. */
#define ADC5_CHAN(_dname, _type, _mask, _pre, _scale) \
	{                                             \
		.datasheet_name = _dname,             \
		.prescale_index = _pre,               \
		.type = _type,                        \
		.info_mask = _mask,                   \
		.scale_fn_type = _scale,              \
	},

#define ADC5_CHAN_TEMP(_dname, _pre, _scale) ADC5_CHAN(_dname, IIO_TEMP, BIT(IIO_CHAN_INFO_PROCESSED), _pre, _scale)

#define ADC5_CHAN_VOLT(_dname, _pre, _scale) ADC5_CHAN(_dname, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_PROCESSED), _pre, _scale)

#define ADC5_CHAN_CUR(_dname, _pre, _scale) ADC5_CHAN(_dname, IIO_CURRENT, BIT(IIO_CHAN_INFO_PROCESSED), _pre, _scale)

// clang-format off
static const struct adc5_channels adc5_chans_pmic[ADC5_MAX_CHANNEL] = {
	[ADC5_GEN3_OFFSET_REF]		= ADC5_CHAN_VOLT("ref_gnd", 0,
						SCALE_HW_CALIB_DEFAULT)
	[ADC5_GEN3_1P25VREF]		= ADC5_CHAN_VOLT("vref_1p25", 0,
						SCALE_HW_CALIB_DEFAULT)
	[ADC5_GEN3_VPH_PWR]		= ADC5_CHAN_VOLT("vph_pwr", 1,
						SCALE_HW_CALIB_DEFAULT)
	[ADC5_GEN3_VBAT_SNS_QBG]	= ADC5_CHAN_VOLT("vbat_sns", 1,
						SCALE_HW_CALIB_DEFAULT)
	[ADC5_GEN3_AMUX3_THM]		= ADC5_CHAN_TEMP("smb_temp", 9,
						SCALE_HW_CALIB_PM7_SMB_TEMP)
	[ADC5_GEN3_CHG_TEMP]		= ADC5_CHAN_TEMP("chg_temp", 0,
						SCALE_HW_CALIB_PM7_CHG_TEMP)
	[ADC5_GEN3_USB_SNS_V_16]	= ADC5_CHAN_VOLT("usb_sns_v_div_16", 1,
						SCALE_HW_CALIB_DEFAULT)
	[ADC5_GEN3_VIN_DIV16_MUX]	= ADC5_CHAN_VOLT("vin_div_16", 1,
						SCALE_HW_CALIB_DEFAULT)
	[ADC5_GEN3_IIN_FB]		= ADC5_CHAN_CUR("iin_fb", 10,
						SCALE_HW_CALIB_CUR)
	[ADC5_GEN3_ICHG_SMB]		= ADC5_CHAN_CUR("ichg_smb", 13,
						SCALE_HW_CALIB_CUR)
	[ADC5_GEN3_IIN_SMB]		= ADC5_CHAN_CUR("iin_smb", 12,
						SCALE_HW_CALIB_CUR)
	[ADC5_GEN3_ICHG_FB]		= ADC5_CHAN_CUR("ichg_fb", 16,
						SCALE_HW_CALIB_CUR_RAW)
	[ADC5_GEN3_DIE_TEMP]		= ADC5_CHAN_TEMP("die_temp", 0,
						SCALE_HW_CALIB_PMIC_THERM_PM7)
	[ADC5_GEN3_TEMP_ALARM_LITE]	= ADC5_CHAN_TEMP("die_temp_lite", 0,
						SCALE_HW_CALIB_PMIC_THERM_PM7)
	[ADC5_GEN3_AMUX1_THM_100K_PU]	= ADC5_CHAN_TEMP("amux_thm1_pu2", 0,
						SCALE_HW_CALIB_THERM_100K_PU_PM7)
	[ADC5_GEN3_AMUX2_THM_100K_PU]	= ADC5_CHAN_TEMP("amux_thm2_pu2", 0,
						SCALE_HW_CALIB_THERM_100K_PU_PM7)
	[ADC5_GEN3_AMUX3_THM_100K_PU]	= ADC5_CHAN_TEMP("amux_thm3_pu2", 0,
						SCALE_HW_CALIB_THERM_100K_PU_PM7)
	[ADC5_GEN3_AMUX4_THM_100K_PU]	= ADC5_CHAN_TEMP("amux_thm4_pu2", 0,
						SCALE_HW_CALIB_THERM_100K_PU_PM7)
	[ADC5_GEN3_AMUX5_THM_100K_PU]	= ADC5_CHAN_TEMP("amux_thm5_pu2", 0,
						SCALE_HW_CALIB_THERM_100K_PU_PM7)
	[ADC5_GEN3_AMUX6_THM_100K_PU]	= ADC5_CHAN_TEMP("amux_thm6_pu2", 0,
						SCALE_HW_CALIB_THERM_100K_PU_PM7)
	[ADC5_GEN3_AMUX1_GPIO_100K_PU]	= ADC5_CHAN_TEMP("amux1_gpio_pu2", 0,
						SCALE_HW_CALIB_THERM_100K_PU_PM7)
	[ADC5_GEN3_AMUX2_GPIO_100K_PU]	= ADC5_CHAN_TEMP("amux2_gpio_pu2", 0,
						SCALE_HW_CALIB_THERM_100K_PU_PM7)
	[ADC5_GEN3_AMUX3_GPIO_100K_PU]	= ADC5_CHAN_TEMP("amux3_gpio_pu2", 0,
						SCALE_HW_CALIB_THERM_100K_PU_PM7)
	[ADC5_GEN3_AMUX4_GPIO_100K_PU]	= ADC5_CHAN_TEMP("amux4_gpio_pu2", 0,
						SCALE_HW_CALIB_THERM_100K_PU_PM7)
	[ADC5_GEN3_VBAT_SNS_CHGR]	= ADC5_CHAN_VOLT("vbat_sns_chgr", 1,
						SCALE_HW_CALIB_DEFAULT)
	[ADC5_GEN3_VBAT_2S_MID_QBG]	= ADC5_CHAN_VOLT("vbat_2s_sns", 1,
						SCALE_HW_CALIB_DEFAULT)
	[ADC5_GEN3_VBAT_2S_MID_CHGR]	= ADC5_CHAN_VOLT("vbat_2s_sns_chgr", 1,
						SCALE_HW_CALIB_DEFAULT)
};
// clang-format on

static const struct adc5_data adc5_gen3_hs_data_pmic = {
	.name = "pm-adc5-gen3-hs",
	.full_scale_code_volt = 0x70e4,
	.full_scale_code_cur = 0x2ee0,
	.adc_chans = adc5_chans_pmic,
	.decimation = (unsigned int[ADC5_DECIMATION_SAMPLES_MAX]){ 85, 340, 1360 },
	.hw_settle_1 = (unsigned int[VADC_HW_SETTLE_SAMPLES_MAX]){ 15, 100, 200, 300, 400, 500, 600, 700, 1000, 2000,
								   4000, 8000, 16000, 32000, 64000, 128000 },
};

static const struct of_device_id adc5_match_table[] = { {
								.compatible = "qcom,spmi-adc5-gen3-hs",
								.data = &adc5_gen3_hs_data_pmic,
							},
							{} };
MODULE_DEVICE_TABLE(of, adc5_match_table);

/* Read the Channel configuration from the device tree */
static int adc5_get_dt_channel_data(struct adc5_chip_hs *adc, struct adc5_channel_prop *prop, struct device_node *node,
				    const struct adc5_data *data)
{
	const char *name = node->name, *channel_name;
	u32 chan, value, varr[2];
	u32 sid = 0;
	int ret;
	struct device *dev = adc->dev;

	ret = of_property_read_u32(node, "reg", &chan);
	if (ret < 0) {
		dev_err(dev, "invalid channel number %s\n", name);
		return ret;
	}

	/*
	 * Value read from "reg" is virtual channel number
	 * virtual channel number = (sid << 8 | channel number).
	 */
	sid = (chan >> ADC5_GEN3_SID_OFFSET);
	chan = (chan & ADC5_GEN3_CHANNEL_MASK);

	if (chan > ADC5_OFFSET_EXT2 || !data->adc_chans[chan].datasheet_name) {
		dev_err(dev, "%s invalid channel number %d\n", name, chan);
		return -EINVAL;
	}

	/* the channel has DT description */
	prop->channel = chan;
	prop->sid = sid;

	channel_name = of_get_property(node, "label", NULL) ?: node->name;
	if (!channel_name) {
		pr_err("Invalid channel name\n");
		return -EINVAL;
	}
	prop->datasheet_name = channel_name;

	ret = of_property_read_u32(node, "qcom,decimation", &value);
	if (!ret) {
		ret = adc5_decimation_from_dt(value, data->decimation);
		if (ret < 0) {
			dev_err(dev, "%02x invalid decimation %d\n", chan, value);
			return ret;
		}
		prop->decimation = ret;
	} else {
		prop->decimation = ADC5_DECIMATION_DEFAULT;
	}

	ret = of_property_read_u32_array(node, "qcom,pre-scaling", varr, 2);
	if (!ret) {
		ret = qcom_adc5_prescaling_from_dt(varr[0], varr[1]);
		if (ret < 0) {
			dev_err(dev, "%02x invalid pre-scaling <%d %d>\n", chan, varr[0], varr[1]);
			return ret;
		}
		prop->prescale = ret;
	} else {
		prop->prescale = adc->data->adc_chans[prop->channel].prescale_index;
	}

	ret = of_property_read_u32(node, "qcom,hw-settle-time", &value);
	if (!ret) {
		ret = adc5_hw_settle_time_from_dt(value, data->hw_settle_1);
		if (ret < 0) {
			dev_err(dev, "%02x invalid hw-settle-time %d us\n", chan, value);
			return ret;
		}
		prop->hw_settle_time = ret;
	} else {
		prop->hw_settle_time = VADC_DEF_HW_SETTLE_TIME;
	}

	ret = of_property_read_u32(node, "qcom,avg-samples", &value);
	if (!ret) {
		ret = adc5_avg_samples_from_dt(value);
		if (ret < 0) {
			dev_err(dev, "%02x invalid avg-samples %d\n", chan, value);
			return ret;
		}
		prop->avg_samples = ret;
	} else {
		prop->avg_samples = VADC_DEF_AVG_SAMPLES;
	}

	prop->scale_fn_type = -EINVAL;
	ret = of_property_read_u32(node, "qcom,scale-fn-type", &value);
	if (!ret && value < SCALE_HW_CALIB_INVALID)
		prop->scale_fn_type = value;

	if (of_property_read_bool(node, "qcom,ratiometric"))
		prop->cal_method = ADC5_RATIOMETRIC_CAL;
	else
		prop->cal_method = ADC5_ABSOLUTE_CAL;

	dev_dbg(dev, "%02x name %s\n", chan, name);

	return 0;
}

static int adc5_gen3_hs_configure_channel_buffer(struct adc5_chip_hs *adc, struct adc5_channel_prop *prop, u8 channel)
{
	int ret;

	ret = mutex_lock_interruptible(&adc->lock);
	if (ret != 0)
		return ret;

	/* Read Current Settings */
	ret = adc5_read(adc, ADC5_GEN3_SID, prop->buffer, sizeof(prop->buffer));
	if (ret < 0)
		goto unlock;

	/* PMIC's SID */
	prop->buffer[0] = prop->sid & ADC5_GEN3_SID_MASK;

	/* Set Channel and Conversion Request */
	prop->buffer[1] = ADC5_GEN3_CHAN_CONV_REQ | channel;

	/* Immediate Conversions */
	prop->buffer[2] = ADC5_GEN3_TIME_IMMEDIATE;

	/* Update calibration select */
	prop->buffer[3] &= ~ADC5_GEN3_DIG_PARAM_CAL_SEL_MASK;
	prop->buffer[3] |= (prop->cal_method << ADC5_GEN3_DIG_PARAM_CAL_SEL_SHIFT);

	/* Update decimation ratio select */
	prop->buffer[3] &= ~ADC5_GEN3_DIG_PARAM_DEC_RATIO_SEL_MASK;
	prop->buffer[3] |= (prop->decimation << ADC5_GEN3_DIG_PARAM_DEC_RATIO_SEL_SHIFT);

	/* Update fast average sample value */
	prop->buffer[4] &= (u8)~ADC5_GEN3_FAST_AVG_CTL_SAMPLES_MASK;
	prop->buffer[4] |= prop->avg_samples | ADC5_GEN3_FAST_AVG_CTL_EN;

	/* Select ADC channel */
	prop->buffer[5] = prop->channel;

	/* Select HW settle delay for channel */
	prop->buffer[6] &= (u8)~ADC5_GEN3_HW_SETTLE_DELAY_MASK;
	prop->buffer[6] |= prop->hw_settle_time;

unlock:
	mutex_unlock(&adc->lock);

	return ret;
}

static int adc5_gen3_hs_get_dt_data(struct adc5_chip_hs *adc, struct device_node *node)
{
	const struct adc5_channels *adc_chan;
	struct iio_chan_spec *iio_chan;
	struct adc5_channel_prop *chan_props;
	struct device_node *child;
	unsigned int index = 0;
	const struct of_device_id *id;
	const struct adc5_data *data;
	int ret, i;

	adc->nchannels = of_get_available_child_count(node);
	if (!adc->nchannels)
		return -EINVAL;

	/* Limit support to 8 channels */
	if (adc->nchannels > 8)
		return -EINVAL;

	adc->iio_chans = devm_kcalloc(adc->dev, adc->nchannels, sizeof(*adc->iio_chans), GFP_KERNEL);
	if (!adc->iio_chans)
		return -ENOMEM;

	adc->chan_props = devm_kcalloc(adc->dev, adc->nchannels, sizeof(*adc->chan_props), GFP_KERNEL);
	if (!adc->chan_props)
		return -ENOMEM;

	chan_props = adc->chan_props;
	iio_chan = adc->iio_chans;
	id = of_match_node(adc5_match_table, node);
	if (id)
		data = id->data;
	else
		data = &adc5_gen3_hs_data_pmic;
	adc->data = data;

	for_each_available_child_of_node(node, child) {
		ret = adc5_get_dt_channel_data(adc, chan_props, child, data);
		if (ret < 0) {
			of_node_put(child);
			return ret;
		}

		chan_props->chip = adc;
		if (chan_props->scale_fn_type == -EINVAL)
			chan_props->scale_fn_type = data->adc_chans[chan_props->channel].scale_fn_type;
		adc_chan = &data->adc_chans[chan_props->channel];
		// TODO Convert this to a triggered stream
		iio_chan->channel = chan_props->channel;
		iio_chan->datasheet_name = chan_props->datasheet_name;
		iio_chan->extend_name = chan_props->datasheet_name;
		iio_chan->info_mask_separate = adc_chan->info_mask;
		iio_chan->type = adc_chan->type;
		iio_chan->address = index;
		iio_chan++;
		chan_props++;
		index++;
	}

	/* Prebuild the buffer for each channel */
	for (i = 0; i < adc->nchannels; i++) {
		ret = adc5_gen3_hs_configure_channel_buffer(adc, &adc->chan_props[i], i);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static enum hrtimer_restart adc5_gen3_hs_start_sample(struct hrtimer *timer)
{
	struct adc5_chip_hs *adc = container_of(timer, struct adc5_chip_hs, sample_timer);
	int ret;
	struct adc5_channel_prop *prop;

	if (adc->state == ADC_STATE_COMPLETE) {
		/* Sample Complete - Read data into buffer */
		u16 data[4];
		int i;

		adc->state = ADC_STATE_IDLE;

		ret = adc5_read(adc, ADC5_GEN3_CH0_DATA0, (u8 *)data, sizeof(data));
		/* Convert units */
		for (i = 0; i < adc->nchannels; i++) {
			int val;

			prop = &adc->chan_props[i];
			ret = qcom_adc5_hw_scale(prop->scale_fn_type, prop->prescale, adc->data, data[i], &val);

			adc->sample_data[adc->data_idx].data[i] = (s16)(val / 1000);
		}
		adc->sample_data[adc->data_idx].t = adc->start_time;
		adc->data_idx++;

		if (adc->data_idx >= ARRAY_SIZE(adc->sample_data)) {
			if(!adc->wrapbuffer){
				pr_info("battery power profile wrapping data_idx: %d\n", adc->data_idx);
			}
			adc->wrapbuffer = 1;
			adc->data_idx = 0;
		}

	} else if (adc->state == ADC_STATE_SAMPLING) {
		/* Drop sample if we are busy */
		adc->skipped_conversions++;
		goto queue_next;
	}

	/* Record the start time */
	adc->start_time = hrtimer_cb_get_time(&adc->sample_timer);

	mutex_lock(&adc->lock);

	/* Flag Conversion Active */
	adc->state = ADC_STATE_SAMPLING;

	/* Queue the Conversion on the last ( highest index ) channel */
	if (adc->nchannels == 4 && (adc->include_usb || !adc->timer_state)) {
		/* if we have 4 channels and usb is turned on or we are in slow mode,
		 * Then Start with Current */
		prop = &adc->chan_props[3];
	} else if (adc->nchannels >= 2) {
		/* Start with Battery */
		prop = &adc->chan_props[1];
	} else {
		/* Not correctly configured */
		goto unlock;
	}
	adc5_gen3_hs_sample_channel(adc, prop);

unlock:
	mutex_unlock(&adc->lock);

queue_next:
	/* Queue the next sample */
	hrtimer_forward_now(timer, adc->timer_state ? adc->sample_period : adc->slow_period);

	return HRTIMER_RESTART;
}

static int adc5_gen3_hs_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct adc5_chip_hs *adc;
	struct regmap *regmap;
	int ret;
	u32 reg;

	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap)
		return -ENODEV;

	/* Allocate the IIO Device */
	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->regmap = regmap;
	adc->dev = dev;

	/* Count the number of elements in reg */
	ret = of_property_count_u32_elems(node, "reg");
	if (ret < 0)
		return ret;
	/* We only support 1, fail if there are more */
	if (ret != 1)
		return -EPERM;

	/* Get SPMI Address */
	ret = of_property_read_u32_index(node, "reg", 0, &reg);
	if (ret < 0)
		return ret;
	adc->base_addr = reg;

	/* Get IRQ SPMI Address */
	ret = of_irq_get_byname(node, "adc-hs-sdam0");
	if (ret < 0) {
		pr_err("Failed to get irq for ADC5 GEN3 SDAM0, ret=%d\n", ret);
		return ret;
	}
	adc->irq = ret;

	/* Get IRQ for Name */
	adc->irq_name = devm_kstrdup(adc->dev, "adc-hs-sdam0", GFP_KERNEL);
	if (!adc->irq_name)
		return -ENOMEM;

	platform_set_drvdata(pdev, adc);

	/* Setup our Lock */
	mutex_init(&adc->lock);

	ret = adc5_gen3_hs_get_dt_data(adc, node);
	if (ret < 0) {
		pr_err("adc get dt data failed\n");
		goto fail;
	}

	/* Register Interrupt Function */
	ret = devm_request_irq(dev, adc->irq, adc5_gen3_hs_isr, 0, adc->irq_name, adc);
	if (ret < 0) {
		pr_warn("adc5_gen3_hs_probe: %d %s ret: %d\n", adc->irq, adc->irq_name, ret);
		goto fail;
	}

	/* Setup IIO Device Info */
	dev_set_drvdata(dev, indio_dev);
	indio_dev->info = &adc5_gen3_hs_info;
	indio_dev->dev.parent = dev;
	indio_dev->dev.of_node = node;
	indio_dev->name = pdev->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adc->iio_chans;
	indio_dev->num_channels = adc->nchannels;

	// Start the Timer at the end
	adc->slow_period = ktime_set(0, 1000 * NSEC_PER_MSEC);
	adc->sample_period = ktime_set(0, 1 * NSEC_PER_MSEC);
	hrtimer_init(&adc->sample_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	adc->sample_timer.function = &adc5_gen3_hs_start_sample;

	return devm_iio_device_register(dev, indio_dev);

fail:
	return ret;
}

static void adc5_gen3_hs_exit(struct platform_device *pdev)
{
	struct adc5_chip_hs *adc = platform_get_drvdata(pdev);
	u8 data = 0;

	mutex_lock(&adc->lock);

	/* Disable all available channels */
	data = MEAS_INT_DISABLE;
	adc5_write(adc, ADC5_GEN3_TIMER_SEL, &data, 1);

	/* To indicate there is an actual conversion request */
	data = ADC5_GEN3_CHAN_CONV_REQ;
	adc5_write(adc, ADC5_GEN3_PERPH_CH, &data, 1);

	data = ADC5_GEN3_CONV_REQ_REQ;
	adc5_write(adc, ADC5_GEN3_CONV_REQ, &data, 1);

	mutex_unlock(&adc->lock);

	mutex_destroy(&adc->lock);

	return;
}

static int adc5_gen3_hs_freeze(struct device *dev)
{
	struct adc5_chip_hs *adc = dev_get_drvdata(dev);

	mutex_lock(&adc->lock);

	devm_free_irq(dev, adc->irq, adc);

	mutex_unlock(&adc->lock);

	return 0;
}

static int adc5_gen3_hs_restore(struct device *dev)
{
	struct adc5_chip_hs *adc = dev_get_drvdata(dev);
	int ret = 0;

	ret = devm_request_irq(dev, adc->irq, adc5_gen3_hs_isr, 0, adc->irq_name, adc);
	if (ret < 0)
		return ret;

	return ret;
}

static const struct dev_pm_ops adc5_gen3_hs_pm_ops = {
	.freeze = adc5_gen3_hs_freeze,
	.restore = adc5_gen3_hs_restore,
};

static struct platform_driver adc5_gen3_hs_driver = {
	.driver = {
		.name = "qcom-spmi-adc5-gen3-highspeed",
		.of_match_table = adc5_match_table,
		.pm = &adc5_gen3_hs_pm_ops,
	},
	.probe = adc5_gen3_hs_probe,
	.remove = adc5_gen3_hs_exit,
};
module_platform_driver(adc5_gen3_hs_driver);

MODULE_ALIAS("platform:qcom-spmi-adc5-gen3-highspeed");
MODULE_DESCRIPTION("Qualcomm Technologies Inc. PMIC5 Gen3 ADC driver");
MODULE_LICENSE("GPL v2");
