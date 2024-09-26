// SPDX-License-Identifier: GPL-2.0
/*
 * HID over SPI protocol implementation
 *
 * Copyright (c) 2021 Microsoft Corporation
 *
 * This code is partly based on "HID over I2C protocol implementation:
 *
 *  Copyright (c) 2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 *  Copyright (c) 2012 Ecole Nationale de l'Aviation Civile, France
 *  Copyright (c) 2012 Red Hat, Inc
 *
 *  which in turn is partly based on "USB HID support for Linux":
 *
 *  Copyright (c) 1999 Andreas Gal
 *  Copyright (c) 2000-2005 Vojtech Pavlik <vojtech@suse.cz>
 *  Copyright (c) 2005 Michael Haboustak <mike-@cinci.rr.com> for Concept2, Inc
 *  Copyright (c) 2007-2008 Oliver Neukum
 *  Copyright (c) 2006-2010 Jiri Kosina
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "spi-hid-core.h"
#include "spi-hid_trace.h"
#include "spi-hid-of.h"
#include "../hid-ids.h"

static unsigned int debug_mask = 0;
module_param(debug_mask, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(debug_mask, "set debug level");

static unsigned int blhacks = 0;
module_param(blhacks, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(blhacks, "enable bootloader hacks");

static unsigned int dropir = 0;
module_param(dropir, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(dropir, "drop input reports");

static unsigned int speed = 8000000;
module_param(speed, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(dropir, "spi speed");

static bool fill = true;
module_param(fill, bool, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(fill, "debug fill structures");

static bool serialize = false;
module_param(serialize, bool, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(serialize, "serialize input reports");



#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN_REPORT_PACKET_LENGTH (5+77)

#define DLEVEL_DEFAULT (1<<0)  // 1
#define DLEVEL_TRACE   (1<<1)  // 2
#define DLEVEL_MUTEX   (1<<2)  // 4
#define DLEVEL_RETRY   (1<<3)  // 8
#define DLEVEL_DEBUG   (1<<4)  // 16
#define DLEVEL_DEBUG2  (1<<5)  // 32
#define DLEVEL_DUMPIR  (1<<6)  // 64
#define DLEVEL_A6      (1<<7)  // 128
#define DLEVEL_TRACE2  (1<<8)  // 256

#define DPRINT(level) if (debug_mask&level) printk
#define DEVPRINT(level, ...) if (debug_mask&level) dev_warn(dev, __VA_ARGS__)

#define tmutex_lock(lock) { if (!mutex_trylock(lock)) { DEVPRINT(DLEVEL_MUTEX, "%s:%d (WAITING TO LOCK)",__func__,__LINE__); mutex_lock(lock); } DEVPRINT(DLEVEL_MUTEX, "%s:%d LOCKED",__func__,__LINE__); }
#define tmutex_unlock(lock) { if (!mutex_is_locked(lock)) { DEVPRINT(DLEVEL_MUTEX, "%s:%d mutex wasn't locked",__func__,__LINE__); } else DEVPRINT(DLEVEL_MUTEX, "%s:%d UNLOCK",__func__,__LINE__); mutex_unlock(lock); }

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

#define SPI_HID_MAX_RESET_ATTEMPTS 3 // 0 disables
#define SPI_FIXED_SPEED_HZ speed

//#define SPI_DELAY 1000     // adds time to CS low
//#define SPI_CS_DELAY 1000  // no effect?

#define SPI_MIN_DELAY 100
#define SPI_RETRY_DELAY 50
#define RETRY_COUNT 3

//#undef dev_dbg
//#define dev_dbg dev_warn

//#define MONTY

// at 1mhz descriptors (1489 bytes, 12Kbits) take 12ms
#define PROTOCOL_TIMEOUT_MSEC 20  // needs to be long for reset only, should separate  1mhz = 40msec, 4mhz = 15msec

#ifdef MONTY
#define PROTOCOL_TIMEOUT_MSEC_SYNC 2
#else
#define PROTOCOL_TIMEOUT_MSEC_SYNC  PROTOCOL_TIMEOUT_MSEC
#endif

#if !defined(PROTOCOL_TIMEOUT_MSEC)
#define PROTOCOL_TIMEOUT_MSEC 5000
#endif

#define PRINT_HEX_DUMP(dev,level,prefix,a,b,c,d,e,f) {\
						char temp[64];\
						snprintf(temp, sizeof(temp), "%s %s: %s",dev_driver_string(dev),dev_name(dev),prefix);\
						print_hex_dump(level,temp,a,b,c,d,e,f);\
						}

static int spi_hid_input_report_handler(struct spi_hid *shid,
		struct spi_hid_input_buf *buf);

static int spi_hid_bus_validate_header(struct spi_hid *shid,
					struct spi_hid_input_header *header);

static struct hid_ll_driver spi_hid_ll_driver;

static void spi_hid_populate_read_approvals(struct spi_hid_of_config *conf,
	u8 *header_buf, u8 *body_buf)
{
	header_buf[0] = conf->read_opcode;
	header_buf[1] = (conf->input_report_header_address >> 16) & 0xff;
	header_buf[2] =	(conf->input_report_header_address >> 8) & 0xff;
	header_buf[3] =	(conf->input_report_header_address >> 0) & 0xff;
	header_buf[4] = SPI_HID_READ_APPROVAL_CONSTANT;

	body_buf[0] = conf->read_opcode;
	body_buf[1] = (conf->input_report_body_address >> 16) & 0xff;
	body_buf[2] = (conf->input_report_body_address >> 8) & 0xff;
	body_buf[3] = (conf->input_report_body_address >> 0) & 0xff;
	body_buf[4] = SPI_HID_READ_APPROVAL_CONSTANT;
}

static void spi_hid_parse_dev_desc(struct spi_hid_device_desc_raw *raw,
					struct spi_hid_device_descriptor *desc)
{
	char type[32] = "";
	unsigned int etime = 0;

	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "%s\n",__func__);
	desc->hid_version = le16_to_cpu(raw->bcdVersion);
	desc->report_descriptor_length = le16_to_cpu(raw->wReportDescLength);
	desc->max_input_length = le16_to_cpu(raw->wMaxInputLength);
	desc->max_output_length = le16_to_cpu(raw->wMaxOutputLength);
	desc->extra_read_length = raw->read_size;
	desc->spi_protocol_level = 0;  // 0 = oldest bootloader

	desc->max_fragment_length = le16_to_cpu(raw->wMaxFragmentLength);

	desc->vendor_id = le16_to_cpu(raw->wVendorID);
	desc->product_id = le16_to_cpu(raw->wProductID);
	desc->version_id = le16_to_cpu(raw->wVersionID);
	desc->no_output_report_ack = le16_to_cpu(raw->wFlags) & BIT(0);
	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "%s %s XRL:%d\n",__func__, desc->no_output_report_ack ? "NOACK" : "ACK", desc->extra_read_length );

	snprintf(desc->sn_string, sizeof(desc->sn_string), "%.*s", (int)sizeof_field(struct spi_hid_device_desc_raw,sn), raw->sn);
	snprintf(desc->name_string, sizeof(desc->name_string), "%.*s", (int)sizeof_field(struct spi_hid_device_desc_raw,name), raw->name);

	sscanf(desc->name_string, "%s %u", type, &etime);
	if (etime >= 1687893892) {
		desc->spi_protocol_level = 4;
	}
	else if (etime > 12345) {
		desc->spi_protocol_level = 1;
	}

	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "  hid_version: %X\n",desc->hid_version);
	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "  report_descriptor_length: %d\n",desc->report_descriptor_length);
	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "  max_input_length: %d\n",desc->max_input_length);
	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "  max_output_length: %d\n",desc->max_output_length);
	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "  max_fragment_length: %d\n",desc->max_fragment_length);
	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "  vendor_id: %X\n",desc->vendor_id);
	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "  product_id: %X\n",desc->product_id);
	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "  version_id: %X\n",desc->version_id);
	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "  no_output_report_ack: %d\n",desc->no_output_report_ack);
	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "  sn_string: %s\n",desc->sn_string);
	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "  name_string: %s\n",desc->name_string);
	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "  extra_read_length: %d\n",desc->extra_read_length);
	DPRINT(DLEVEL_DEFAULT) (KERN_WARNING "  spi_protocol_level: %d\n",desc->spi_protocol_level);
}

static void spi_hid_populate_input_header(u8 *buf,
		struct spi_hid_input_header *header)
{
	header->version            = buf[0] & 0xf;
	header->have_body_flag     = (buf[0] & 0x10) >> 4;
	header->report_length      = le16_to_cpu((buf[1] | ((buf[2] & 0x3f) << 8)) * 4);
	header->last_fragment_flag = (buf[2] & 0x40) >> 6;
	header->sync_const         = buf[3];
	header->body_ptr           = &buf[4];
}

static void spi_hid_populate_input_body(u8 *buf,
		struct spi_hid_input_body *body)
{
	body->report_type = (buf[0] & 0x0F);
	body->stall_flag = (buf[0] & 0x10) >> 4;
	body->content_length = le16_to_cpu(buf[1] | (buf[2] << 8));
	body->content_id = buf[3];
}

static void spi_hid_input_report_prepare(struct spi_hid_input_buf *buf,
		struct spi_hid_input_report *report)
{
	struct spi_hid_input_header header;
	struct spi_hid_input_body body;

	spi_hid_populate_input_header(buf->header, &header);
	spi_hid_populate_input_body(buf->body, &body);
	report->report_type = body.report_type;
	report->content_length = body.content_length;
	report->content_id = body.content_id;
	memcpy(report->content, buf->content, MIN(sizeof(report->content),body.content_length));
}

static void spi_hid_populate_output_header(u8 *buf,
		struct spi_hid_of_config *conf,
		struct spi_hid_output_report *report)
{
	buf[0] = conf->write_opcode;
	buf[1] = (conf->output_report_address >> 16) & 0xff;
	buf[2] = (conf->output_report_address >> 8) & 0xff;
	buf[3] = (conf->output_report_address >> 0) & 0xff;
	buf[4] = report->report_type;
	buf[5] = report->content_length & 0xff;
	buf[6] = (report->content_length >> 8) & 0xff;
	buf[7] = report->content_id;
}


// called by spi_hid_dev_irq ONLY
static int spi_hid_input_sync(struct spi_hid *shid, void *buf, u16 length,
				bool is_header)
{
	int ret;
	struct device *dev = &shid->spi->dev;

	uint8_t slave_state;
	uint32_t retries = 3;

	struct spi_transfer transfer;

	s64 diff_us;

	DEVPRINT(DLEVEL_DEFAULT, "%s is_header=%d len=%d\n", __func__, is_header, length);

	memset(&transfer, 0, sizeof(transfer));

	memcpy(shid->temp_spi_tx_buf, is_header ? shid->read_approval_header : shid->read_approval_body, SPI_HID_READ_APPROVAL_LEN);

	transfer.tx_buf = shid->temp_spi_tx_buf;
	transfer.rx_buf = shid->temp_spi_rx_buf;
	transfer.len = SPI_HID_READ_APPROVAL_LEN + length;
	transfer.speed_hz = SPI_FIXED_SPEED_HZ;

#ifdef SPI_DELAY
	transfer.delay.value = SPI_DELAY;
	transfer.delay.unit = SPI_DELAY_UNIT_USECS;
#endif
#ifdef SPI_CS_DELAY
	transfer.cs_change_delay.value = SPI_CS_DELAY;
	transfer.cs_change_delay.unit = SPI_DELAY_UNIT_USECS;
#endif



	spi_message_init_with_transfers(&shid->input_message, &transfer, 1);

	tmutex_lock(&shid->sync_lock);
	diff_us = ktime_to_us(ktime_sub(ktime_get(), shid->last_event_finished));
	if (diff_us < SPI_MIN_DELAY) {
		usleep_range(SPI_MIN_DELAY - diff_us, SPI_MIN_DELAY - diff_us + 100);
	}
	do {
		ret = spi_sync(shid->spi, &shid->input_message);
		if (ret) {
			dev_err(dev, "Error starting async transfer: %d, resetting\n",
									ret);
			shid->bus_error_count++;
			shid->bus_last_error = ret;
			//schedule_work(&shid->error_work);  // pointless considering spi_sync returned error
		}
		shid->stats.spi_total_count++;
		slave_state = shid->temp_spi_rx_buf[0];
		if (slave_state == SLAVE_STATE_NOTREADY) shid->stats.spi_retry_count++;
		DEVPRINT(DLEVEL_RETRY, "%s:%d s_s:0x%x r:%d\n",__func__, __LINE__, slave_state, retries);
	}
	while ( !ret && (slave_state == SLAVE_STATE_NOTREADY || (blhacks && slave_state == 0xFF)) && retries-- && (usleep_range(SPI_RETRY_DELAY, SPI_RETRY_DELAY+100), 1));
	if (slave_state != SLAVE_STATE_NOTREADY) shid->last_event_finished = ktime_get();
	tmutex_unlock(&shid->sync_lock);

	if (ret) {
		dev_warn(dev, "%s:%d spi_sync failed\n", __func__, __LINE__);
		ret = -EIO;
		goto out;
	}

	if (retries < 0) {
		dev_warn(dev, "%s:%d retries exhausted\n", __func__, __LINE__);
		ret = -EAGAIN;
		goto out;
	}

	if (is_header && slave_state != SLAVE_STATE_HEADER && slave_state != SLAVE_STATE_INPUT_REPORT && slave_state != SLAVE_STATE_IDLE)
	{
		dev_warn(dev, "%s: Unexpected slave state 0x%x, expecting (header or I.R.) 0x%x or 0x%x\n", __func__, slave_state, SLAVE_STATE_HEADER, SLAVE_STATE_INPUT_REPORT);
	}
	else if (!is_header && slave_state != SLAVE_STATE_BODY )
	{
		dev_warn(dev, "%s: Unexpected slave state 0x%x, expecting (body) 0x%x\n", __func__, slave_state, SLAVE_STATE_BODY);
	}

out:
	memcpy(buf, shid->temp_spi_rx_buf+SPI_HID_READ_APPROVAL_LEN, length);

	if (debug_mask&DLEVEL_TRACE) PRINT_HEX_DUMP(dev, KERN_WARNING, "htx: ", DUMP_PREFIX_OFFSET, 32, 1, shid->temp_spi_tx_buf, SPI_HID_READ_APPROVAL_LEN, 0);
	if (debug_mask&DLEVEL_TRACE) PRINT_HEX_DUMP(dev, KERN_WARNING, "hrx: ", DUMP_PREFIX_OFFSET, 32, 1, shid->temp_spi_rx_buf, SPI_HID_READ_APPROVAL_LEN, 0);
	if (debug_mask&DLEVEL_TRACE2) PRINT_HEX_DUMP(dev, KERN_WARNING, "brx: ", DUMP_PREFIX_OFFSET, 32, 1, shid->temp_spi_rx_buf+SPI_HID_READ_APPROVAL_LEN, length, 0);

	return ret;
}

static int spi_hid_output(struct spi_hid *shid, void *buf, u16 length)
{
	struct spi_transfer transfer;
	struct spi_message message;
	int ret;
	struct device *dev = &shid->spi->dev;

	uint8_t slave_state;
	uint32_t retries = 3;

	s64 diff_us;

	DEVPRINT(DLEVEL_DEFAULT, "%s len=%d\n",__func__, length);

	memset(&transfer, 0, sizeof(transfer));

	transfer.tx_buf = buf;
	transfer.rx_buf = &shid->out_input.state;
	transfer.len = length; // MIN(sizeof(temp_rx_buf),length);
	transfer.speed_hz = SPI_FIXED_SPEED_HZ;

#ifdef SPI_DELAY
	transfer.delay.value = SPI_DELAY;
	transfer.delay.unit = SPI_DELAY_UNIT_USECS;
#endif
#ifdef SPI_CS_DELAY
	transfer.cs_change_delay.value = SPI_CS_DELAY;
	transfer.cs_change_delay.unit = SPI_DELAY_UNIT_USECS;
#endif

	spi_message_init_with_transfers(&message, &transfer, 1);

	trace_spi_hid_output_begin(shid, transfer.tx_buf,
			transfer.len, NULL, 0, 0);


	tmutex_lock(&shid->sync_lock);
	diff_us = ktime_to_us(ktime_sub(ktime_get(), shid->last_event_finished));
	if (diff_us < SPI_MIN_DELAY) {
		usleep_range(SPI_MIN_DELAY - diff_us, SPI_MIN_DELAY - diff_us + 100);
	}
	do {
		ret = spi_sync(shid->spi, &message);
		slave_state = ((uint8_t *)transfer.rx_buf)[0];
		shid->stats.spi_total_count++;
		if (slave_state == SLAVE_STATE_NOTREADY) shid->stats.spi_retry_count++;
		DEVPRINT(DLEVEL_DEFAULT, "%s s_s:0x%x r:%d\n",__func__, slave_state, retries);
	}
	while (!ret && slave_state == SLAVE_STATE_NOTREADY && retries-- && (usleep_range(SPI_RETRY_DELAY, SPI_RETRY_DELAY+100), 1));
	if (slave_state != SLAVE_STATE_NOTREADY) shid->last_event_finished = ktime_get();
	tmutex_unlock(&shid->sync_lock);

	if (ret) {
		dev_warn(dev, "%s:%d spi_sync failed\n", __func__, __LINE__);
		shid->bus_error_count++;
		shid->bus_last_error = ret;
		ret = -EIO;
		goto out;
	}

	if (retries < 0) {
		dev_warn(dev, "%s:%d retries exhausted\n", __func__, __LINE__);
		ret = -EAGAIN;
		goto out;
	}


	trace_spi_hid_output_end(shid, transfer.tx_buf,
			transfer.len, NULL, 0, ret);

	if (shid->desc.spi_protocol_level < 4) {

	if (shid->out_input.state == SLAVE_STATE_INPUT_REPORT )
	{
		DEVPRINT(DLEVEL_DEFAULT, "%s ss:0x%x EATER TRIGGERED\n",__func__, slave_state);
		atomic_inc(&shid->irq_eater);
	}
	else if (shid->out_input.state != SLAVE_STATE_IDLE )
	{
		dev_warn(dev, "%s: Unexpected slave state 0x%x, expecting (idle) 0x%x\n", __func__, shid->out_input.state, SLAVE_STATE_IDLE);
	}
	
	} else {  // !(shid->desc.spi_protocol_level < 4)

	if (slave_state == 0) {
		ret = -EAGAIN;
		dev_warn(dev, "********slave state 0\n");
		goto out;
	}

	if (shid->out_input.state == SLAVE_STATE_INPUT_REPORT )
	{
		struct spi_hid_input_header header;
		struct spi_hid_input_body body;
		int ret = 0;

		DEVPRINT(DLEVEL_A6, "%s: got an A6 input report\n", __func__);

		spi_hid_populate_input_header(shid->out_input.inbuf.header, &header);

		if (header.sync_const != 0x5a) {
			dev_err(dev, "sneak A6 bad header sync:%x",header.sync_const);
		}

		if (!header.have_body_flag || !shid->desc.extra_read_length) {
			
			dev_err(dev, "sneak A6 didn't have body");
		} else {
			memcpy(&shid->out_input.inbuf.body, header.body_ptr, header.report_length);

			spi_hid_populate_input_body(header.body_ptr, &body);

 			ret = spi_hid_bus_validate_header(shid, &header);
			if (ret) {
				dev_err(dev, "failed to validate sneak A6");
			}

			if (header.sync_const != 0x5a) {
				dev_warn(dev, "%s: %d version:%d hbf:%d len:%d lff:%d sync:%x \n", __func__, __LINE__, header.version, header.have_body_flag, header.report_length, header.last_fragment_flag, header.sync_const);
				dev_warn(dev, "%s: %d type:%d stall:%d len:%d id:%d\n", __func__, __LINE__, body.report_type, body.stall_flag, body.content_length, body.content_id);
			}

			switch (body.report_type) {
				case SPI_HID_INPUT_REPORT_TYPE_DATA:
					ret = spi_hid_input_report_handler(shid, &shid->out_input.inbuf);
					if (ret)
						dev_warn(dev, "%s: A6 input report fail\n", __func__);
					break;
				default:
					dev_warn(dev, "%s: !!! A6 input report wrong type %d\n", __func__, body.report_type);
					break;
			}
		}
	}
    } // shid->desc.spi_protocol_level < 4

out:
	if (debug_mask&DLEVEL_TRACE2) PRINT_HEX_DUMP(dev, KERN_WARNING, "otx: ", DUMP_PREFIX_OFFSET, 32, 1, transfer.tx_buf, transfer.len, 0); // printk
	if (debug_mask&DLEVEL_TRACE2) PRINT_HEX_DUMP(dev, KERN_WARNING, "orx: ", DUMP_PREFIX_OFFSET, 32, 1, &shid->out_input.state, transfer.len, 0); // printk

	return ret;
}

static const char *const spi_hid_power_mode_string(u8 power_state)
{
	switch (power_state) {
	case SPI_HID_POWER_MODE_ON:
		return "d0";
	case SPI_HID_POWER_MODE_SLEEP:
		return "d2";
	case SPI_HID_POWER_MODE_OFF:
		return "d3";
	case SPI_HID_POWER_MODE_WAKING_SLEEP:
		return "d3*";
	default:
		return "unknown";
	}
}

static void spi_hid_suspend(struct spi_hid *shid)
{
	struct device *dev = &shid->spi->dev;
#if 0
	int ret;

	DEVPRINT(DLEVEL_DEFAULT, "%s %d\n",__func__, __LINE__);
	if (shid->power_state == SPI_HID_POWER_MODE_OFF)
		return;

	DEVPRINT(DLEVEL_DEFAULT, "%s %d\n",__func__, __LINE__);
	if (shid->irq_enabled) {
		disable_irq(shid->spi->irq);
		shid->irq_enabled = false;
	} else {
		dev_err(dev, "%s called with interrupt already disabled\n",
								__func__);
	}

	shid->ready = false;

	spi_hid_of_assert_reset(&shid->conf, dev);

	DEVPRINT(DLEVEL_DEFAULT, "%s %d\n",__func__, __LINE__);
	ret = spi_hid_of_power_down(&shid->conf);
	if (ret) {
		dev_err(dev, "%s: could not power down\n", __func__);
		shid->regulator_error_count++;
		shid->regulator_last_error = ret;
		return;
	}

	shid->power_state = SPI_HID_POWER_MODE_OFF;
#else
	DEVPRINT(DLEVEL_DEFAULT, "%s:%d from:%pS\n",__func__, __LINE__, __builtin_return_address(0));
#endif
}

static int spi_hid_resume(struct spi_hid *shid)
{
	int ret;
	struct device *dev = &shid->spi->dev;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	if (shid->power_state == SPI_HID_POWER_MODE_ON)
		return -1;

	if (!shid->irq_enabled) {
		enable_irq(shid->spi->irq);
		shid->irq_enabled = true;
	} else {
		dev_err(dev, "%s called with interrupt already enabled\n",
								__func__);
	}

	ret = spi_hid_of_power_up(&shid->conf);
	if (ret) {
		dev_err(dev, "%s: could not power up\n", __func__);
		shid->regulator_error_count++;
		shid->regulator_last_error = ret;
		//return -2;
	}
	shid->power_state = SPI_HID_POWER_MODE_ON;

	spi_hid_of_deassert_reset(&shid->conf, dev);

	DEVPRINT(DLEVEL_DEFAULT, "%s    exiting\n",__func__);

	return 0;
}

static struct hid_device *spi_hid_disconnect_hid(struct spi_hid *shid)
{
	struct hid_device *hid = shid->hid;
	struct device *dev = &shid->spi->dev;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	shid->hid = NULL;

	return hid;
}

static void spi_hid_stop_hid(struct spi_hid *shid)
{
	struct hid_device *hid;
	struct device *dev = &shid->spi->dev;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	hid = spi_hid_disconnect_hid(shid);
	if (hid) {
		cancel_work_sync(&shid->create_device_work);
		cancel_work_sync(&shid->refresh_device_work);
		hid_destroy_device(hid);
	}
}

static void spi_hid_error_work(struct work_struct *work)
{
	struct spi_hid *shid = container_of(work, struct spi_hid, error_work);
	struct device *dev = &shid->spi->dev;
	int ret;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	mutex_lock(&shid->power_lock);
	if (shid->power_state == SPI_HID_POWER_MODE_OFF)
		goto out;

	if (SPI_HID_MAX_RESET_ATTEMPTS && shid->attempts++ >= SPI_HID_MAX_RESET_ATTEMPTS) {
		dev_err(dev, "unresponsive device, aborting.\n");
		spi_hid_stop_hid(shid);
		spi_hid_of_assert_reset(&shid->conf, dev);
		ret = spi_hid_of_power_down(&shid->conf);
		if (ret) {
			dev_err(dev, "failed to disable regulator\n");
			shid->regulator_error_count++;
			shid->regulator_last_error = ret;
		}
		goto out;
	}

	trace_spi_hid_error_work(shid);

	shid->ready = false;

	spi_hid_of_assert_reset(&shid->conf, dev);

	shid->power_state = SPI_HID_POWER_MODE_OFF;
	DEVPRINT(DLEVEL_DEFAULT, "%s cancel reset_response_work\n",__func__);
	cancel_work_sync(&shid->reset_response_work);

	spi_hid_of_sleep_minimal_reset_delay(&shid->conf);

	shid->power_state = SPI_HID_POWER_MODE_ON;

	spi_hid_of_deassert_reset(&shid->conf, dev);
out:
	mutex_unlock(&shid->power_lock);
	DEVPRINT(DLEVEL_DEBUG, "%s     exits\n",__func__);
}

static int spi_hid_send_output_report(struct spi_hid *shid,
		struct spi_hid_output_report *report)
{
	struct spi_hid_output_buf *buf = &shid->output;
	struct device *dev = &shid->spi->dev;
	u16 report_length;
	u16 padded_length;
	u8 padding;
	int ret;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	if (report->content_length > shid->desc.max_output_length) {
		dev_err(dev, "Output report too big, content_length 0x%x\n",
						report->content_length);
		ret = -E2BIG;
		goto out;
	}

	memset(buf->content, 0, MIN_REPORT_PACKET_LENGTH);

	spi_hid_populate_output_header(buf->header, &shid->conf, report);

	if (report->content_length)
		memcpy(&buf->content, report->content, report->content_length);

	report_length = sizeof(buf->header) + report->content_length;
	padded_length = round_up(report_length,	4);
	padding = padded_length - report_length;
	memset(&buf->content[report->content_length], 0, padding);

	if (shid->desc.spi_protocol_level > 3) {
	padded_length = MAX(MIN_REPORT_PACKET_LENGTH, padded_length);
	} // (shid->desc.spi_protocol_level > 3)

	ret = spi_hid_output(shid, buf, padded_length);
	if (ret) {
		dev_err(dev, "Failed output transfer\n");
		goto out;
	}

	return 0;

out:
	return ret;
}

static int spi_hid_sync_request(struct spi_hid *shid,
		struct spi_hid_output_report *report)
{
	struct device *dev = &shid->spi->dev;
	int ret = 0;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	ret = spi_hid_send_output_report(shid, report);
	if (ret) {
		dev_err(dev, "Failed to transfer output report\n");
		return ret;
	}

	tmutex_unlock(&shid->output_lock);
#ifndef MONTY
	ret = wait_for_completion_interruptible_timeout(&shid->output_done,
			msecs_to_jiffies(PROTOCOL_TIMEOUT_MSEC_SYNC));
#else
	ret = wait_for_completion_interruptible_timeout(&shid->output_done,
			msecs_to_jiffies(shid->timeout_msec));
#endif
	tmutex_lock(&shid->output_lock);
	if (ret == 0) {
#ifndef MONTY
		dev_err(dev, "Response timed out\n");
		return -EAGAIN;
#else
		dev_err(dev, "Response timed out monty mode EPIPE msec:%d\n",shid->timeout_msec);
		return -EPIPE;
#endif
	}

	return 0;
}

/**
 * Handle the reset response from the FW by sending a request for the device
 * descriptor.
 */
static void spi_hid_reset_response_work(struct work_struct *work)
{
	struct spi_hid *shid =
		container_of(work, struct spi_hid, reset_response_work);
	struct device *dev = &shid->spi->dev;
	struct spi_hid_output_report report = {
		.report_type = SPI_HID_OUTPUT_REPORT_TYPE_DEVICE_DESC_REQUEST,
		.content_length = 0x0,
		.content_id = SPI_HID_OUTPUT_REPORT_CONTENT_ID_DESC_REQUEST,
		.content = NULL,
	};
	int ret;

	DEVPRINT(DLEVEL_DEBUG, "%s\n",__func__);
	trace_spi_hid_reset_response_work(shid);

	if (shid->ready) {
		dev_err(dev, "Spontaneous FW reset!");
		shid->ready = false;
		shid->dir_count++;
	}

	if (shid->power_state == SPI_HID_POWER_MODE_OFF)
		return;

	if (flush_work(&shid->create_device_work))
		dev_err(dev, "Reset handler waited for create_device_work");

	if (flush_work(&shid->refresh_device_work))
		dev_err(dev, "Reset handler waited for refresh_device_work");

	tmutex_lock(&shid->request_lock);
	tmutex_lock(&shid->output_lock);
	ret = spi_hid_sync_request(shid, &report);
	tmutex_unlock(&shid->output_lock);
	tmutex_unlock(&shid->request_lock);

	if (ret) {
		dev_WARN_ONCE(dev, true,
				"Failed to send device descriptor request\n");
		schedule_work(&shid->error_work);
	}
	DEVPRINT(DLEVEL_DEBUG, "%s exits\n",__func__);
}

static int spi_hid_input_report_handler(struct spi_hid *shid,
		struct spi_hid_input_buf *buf)
{
	struct device *dev = &shid->spi->dev;
	struct spi_hid_input_report r;
	int ret;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	trace_spi_hid_input_report_handler(shid);

	if (!shid->ready || shid->refresh_in_progress || !shid->hid)
		return 0;

	if (dropir)
		return 0;

	spi_hid_input_report_prepare(buf, &r);

	if (serialize) tmutex_lock(&shid->hid_input_lock);
	ret = hid_input_report(shid->hid, HID_INPUT_REPORT,
			&r.content_id,
			r.content_length + 1, 1);
	if (serialize) tmutex_unlock(&shid->hid_input_lock);

	if (debug_mask&DLEVEL_DUMPIR) PRINT_HEX_DUMP(dev, KERN_WARNING, "HIR: ", DUMP_PREFIX_OFFSET, 32, 1, r.content - 1, r.content_length + 1, 0);

	if (ret == -EBUSY) {
		shid->stats.hid_input_busy++;
		return 0;
	}

	if (ret == -ENODEV) {
		dev_err(dev, "ignoring report ENODEV\n");
		return 0;
	} else if (ret) {
		dev_err(dev, "Bad input report, error %d\n", ret);
	}

	return ret;
}

// called by spi_hid_process_input_report ONLY
static void spi_hid_response_handler(struct spi_hid *shid,
		struct spi_hid_input_buf *buf)
{
	struct device *dev = &shid->spi->dev;
	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	trace_spi_hid_response_handler(shid);

	/* completion_done returns 0 if there are waiters, otherwise 1 */
	if (completion_done(&shid->output_done)) {
		dev_err(&shid->spi->dev, "Unexpected response report\n");
	} else {
		if ( (shid->input.body[0] & 0xf) ==
				SPI_HID_INPUT_REPORT_TYPE_REPORT_DESC ||
			(shid->input.body[0] & 0xf) ==
				SPI_HID_INPUT_REPORT_TYPE_GET_FEATURE_RESP) {
			size_t response_length = (shid->input.body[1] |
					(shid->input.body[2] << 8)) +
					sizeof(shid->input.body);
			memcpy(shid->response.body, shid->input.body,
							response_length);
		} else if ( (shid->input.body[0] & 0xf) ==
				SPI_HID_INPUT_REPORT_TYPE_SET_FEATURE_RESP) {
			memcpy(shid->response.body, shid->input.body, sizeof(shid->input.body));
		}
		complete(&shid->output_done);
	}
}

/*
 * This function returns the length of the report descriptor, or a negative
 * error code if something went wrong.
 */
static int spi_hid_report_descriptor_request(struct spi_hid *shid)
{
	int ret;
	struct device *dev = &shid->spi->dev;
	struct spi_hid_output_report report = {
		.report_type = SPI_HID_OUTPUT_REPORT_TYPE_REPORT_DESC_REQUEST,
		.content_length = 0,
		.content_id = SPI_HID_OUTPUT_REPORT_CONTENT_ID_DESC_REQUEST,
		.content = NULL,
	};

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	ret =  spi_hid_sync_request(shid, &report);
	if (ret) {
		dev_err(dev,
			"Expected report descriptor not received! Error %d\n",
			ret);
		schedule_work(&shid->error_work);
		goto out;
	}

	ret = (shid->response.body[1] | ((0x3f & shid->response.body[2]) << 8));
	if (ret != shid->desc.report_descriptor_length) {
		ret = min_t(unsigned int, ret,
			shid->desc.report_descriptor_length);
		dev_err(dev,
			"Received report descriptor length doesn't match device descriptor field, using min of the two: %d (rcvd rdl: %d desc rdl:%d)\n",
			ret,(shid->response.body[1] | ((0x3f & shid->response.body[2]) << 8)), shid->desc.report_descriptor_length);
	}
out:
	return ret;
}

// called by spi_hid_dev_irq() ONLY
static void spi_hid_process_input_report(struct spi_hid *shid,
		struct spi_hid_input_buf *buf)
{
	struct spi_hid_input_header header;
	struct spi_hid_input_body body;
	struct device *dev = &shid->spi->dev;
	struct spi_hid_device_desc_raw *raw;
	int ret;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	trace_spi_hid_process_input_report(shid);

	spi_hid_populate_input_header(buf->header, &header);
	spi_hid_populate_input_body(buf->body, &body);

	if (body.content_length > header.report_length) {
		dev_err(dev, "Bad body length %d > %d\n", body.content_length,
							header.report_length);
		schedule_work(&shid->error_work);
		return;
	}

	switch (body.report_type) {
	case SPI_HID_INPUT_REPORT_TYPE_DATA:
		ret = spi_hid_input_report_handler(shid, buf);
		if (ret)
			schedule_work(&shid->error_work);
		break;
	case SPI_HID_INPUT_REPORT_TYPE_RESET_RESP:
		DEVPRINT(DLEVEL_DEFAULT, "%s schedule reset_response_work\n",__func__);
		if (!schedule_work(&shid->reset_response_work))
			{
				dev_err(dev, "schedule_work(reset_response_work) failed, already scheduled?\n");
			}
		break;
	case SPI_HID_INPUT_REPORT_TYPE_DEVICE_DESC:
		/* Mark the completion done to avoid timeout */
		spi_hid_response_handler(shid, buf);

		/* Reset attempts at every device descriptor fetch */
		shid->attempts = 0;

		raw = (struct spi_hid_device_desc_raw *)buf->content;

		/* Validate device descriptor length before parsing */
		if (body.content_length != SPI_HID_DEVICE_DESCRIPTOR_LENGTH) {
			dev_err(dev,
				"Invalid content length %d, expected %d\n",
				body.content_length,
				SPI_HID_DEVICE_DESCRIPTOR_LENGTH);
			schedule_work(&shid->error_work);
			break;
		}

		if (le16_to_cpu(raw->wDeviceDescLength) !=
					SPI_HID_DEVICE_DESCRIPTOR_LENGTH) {
			dev_err(dev,
				"Invalid wDeviceDescLength %d, expected %d\n",
				raw->wDeviceDescLength,
				SPI_HID_DEVICE_DESCRIPTOR_LENGTH);
			schedule_work(&shid->error_work);
			break;
		}

		spi_hid_parse_dev_desc(raw, &shid->desc);

		if (shid->desc.hid_version != SPI_HID_SUPPORTED_VERSION) {
			dev_err(dev,
				"Unsupported device descriptor version %4x\n",
				shid->desc.hid_version);
			schedule_work(&shid->error_work);
			break;
		}

		if (shid->desc.spi_protocol_level < 4) {
			dev_warn(dev,
				"using a compat mode, firmware protocol version=%d\n",
				shid->desc.spi_protocol_level);
		}

		if (!shid->hid)
			schedule_work(&shid->create_device_work);
		else
			schedule_work(&shid->refresh_device_work);

		break;
	case SPI_HID_INPUT_REPORT_TYPE_SET_OUTPUT_REPORT_RESP:
		if (shid->desc.no_output_report_ack) {
			dev_err(dev, "Unexpected output report response\n");
			break;
		}
		fallthrough;
	case SPI_HID_INPUT_REPORT_TYPE_GET_FEATURE_RESP:
	case SPI_HID_INPUT_REPORT_TYPE_SET_FEATURE_RESP:
		if (!shid->ready) {
			dev_err(dev,
				"Unexpected response report while not ready: 0x%x\n",
				body.report_type);
			break;
		}
		fallthrough;
	case SPI_HID_INPUT_REPORT_TYPE_REPORT_DESC:
		spi_hid_response_handler(shid, buf);
		break;

	case SPI_HID_INPUT_REPORT_TYPE_GET_INPUT_REPORT_RESP:
	case SPI_HID_INPUT_REPORT_TYPE_COMMAND_RESP:
		dev_err(dev, "Not a supported report type: 0x%x\n",
							body.report_type);
		break;
	default:
		dev_err(dev, "Unknown input report: 0x%x\n",
							body.report_type);
		schedule_work(&shid->error_work);
		break;
	}
}

static int spi_hid_bus_validate_header(struct spi_hid *shid,
					struct spi_hid_input_header *header)
{
	struct device *dev = &shid->spi->dev;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	if (header->version != SPI_HID_INPUT_HEADER_VERSION) {
		dev_err(dev, "Unknown input report version (v 0x%x)\n",
				header->version);
		return -EINVAL;
	}

	if (shid->desc.max_input_length != 0 &&
			header->report_length > shid->desc.max_input_length) {
		dev_err(dev, "Input report body size %u > max expected of %u\n",
				header->report_length,
				shid->desc.max_input_length);
		return -EMSGSIZE;
	}

	if (header->last_fragment_flag != 1) {
		dev_err(dev, "Multi-fragment reports not supported\n");
		return -EOPNOTSUPP;
	}

	if (header->sync_const != SPI_HID_INPUT_HEADER_SYNC_BYTE) {
		dev_err(dev, "Invalid input report sync constant (0x%x)\n",
				header->sync_const);
		return -EINVAL;
	}

	return 0;
}

static int spi_hid_create_device(struct spi_hid *shid)
{
	struct hid_device *hid;
	struct device *dev = &shid->spi->dev;
	int ret;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);

	hid = hid_allocate_device();

	if (IS_ERR(hid)) {
		dev_err(dev, "Failed to allocate hid device: %ld\n",
				PTR_ERR(hid));
		ret = PTR_ERR(hid);
		return ret;
	}

	hid->driver_data = shid->spi;
	hid->ll_driver = &spi_hid_ll_driver;
	hid->dev.parent = &shid->spi->dev;
	hid->bus = BUS_SPI;
	hid->version = shid->desc.hid_version;
	hid->vendor = shid->desc.vendor_id;
	hid->product = shid->desc.product_id;

	strscpy(hid->phys, dev_name(&shid->spi->dev), sizeof(hid->phys));

	strscpy(hid->name, shid->desc.name_string, sizeof(hid->name));
	strscpy(hid->uniq, shid->desc.sn_string, sizeof(hid->uniq));

	shid->hid = hid;

	ret = hid_add_device(hid);
	if (ret) {
		dev_err(dev, "Failed to add hid device: %d\n", ret);
		/*
		 * We likely got here because report descriptor request timed
		 * out. Let's disconnect and destroy the hid_device structure.
		 */
		hid = spi_hid_disconnect_hid(shid);
		if (hid)
			hid_destroy_device(hid);
		return ret;
	}

	return 0;
}

static void spi_hid_create_device_work(struct work_struct *work)
{
	struct spi_hid *shid =
		container_of(work, struct spi_hid, create_device_work);
	struct device *dev = &shid->spi->dev;
	u8 prev_state = shid->power_state;
	int ret;

	shid->timeout_msec = 30;
	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	trace_spi_hid_create_device_work(shid);

	mutex_lock(&shid->power_lock);
	if (prev_state == SPI_HID_POWER_MODE_OFF) {
		dev_err(dev, "%s: Powered off, returning", __func__);
		goto out;
	}

	ret = spi_hid_create_device(shid);
	if (ret) {
		dev_err(dev, "%s: Failed to create hid device\n", __func__);
		goto out;
	}

	//spi_hid_suspend(shid);
	DEVPRINT(DLEVEL_DEFAULT, "%s exits\n",__func__);

out:
	mutex_unlock(&shid->power_lock);

	shid->timeout_msec = 2;

	dev_dbg(dev, "%s: %s -> %s\n", __func__,
			spi_hid_power_mode_string(prev_state),
			spi_hid_power_mode_string(shid->power_state));
}

static void spi_hid_refresh_device_work(struct work_struct *work)
{
	struct spi_hid *shid =
		container_of(work, struct spi_hid, refresh_device_work);
	struct device *dev = &shid->spi->dev;
	struct hid_device *hid;
	int ret;
	u32 new_crc32;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	trace_spi_hid_refresh_device_work(shid);

	mutex_lock(&shid->power_lock);

	if (shid->power_state == SPI_HID_POWER_MODE_OFF) {
		dev_err(dev, "%s: Powered off, returning", __func__);
		goto out2;
	}

	tmutex_lock(&shid->output_lock);
	ret = spi_hid_report_descriptor_request(shid);
	tmutex_unlock(&shid->output_lock);
	if (ret < 0) {
		dev_err(dev,
			"Refresh: failed report descriptor request, error %d",
			ret);
		goto out2;
	}

	new_crc32 = crc32_le(0, (unsigned char const *)shid->response.content,
								(size_t)ret);
	if (new_crc32 == shid->report_descriptor_crc32)
		goto out1;

	shid->report_descriptor_crc32 = new_crc32;
	shid->refresh_in_progress = true;

	hid = spi_hid_disconnect_hid(shid);
	if (hid)
		hid_destroy_device(hid);

	ret = spi_hid_create_device(shid);
	if (ret) {
		dev_err(dev, "%s: Failed to create hid device\n", __func__);
		goto out2;
	}

	shid->refresh_in_progress = false;

out1:
	shid->ready = true; // ??? spi_hid_create_device takes a long time
	if (completion_done(&shid->ready_done))
		dev_err(&shid->spi->dev, "Nobody waiting for ready_done\n");
	else
		complete(&shid->ready_done);
out2:
	mutex_unlock(&shid->power_lock);
	DEVPRINT(DLEVEL_DEFAULT, ">%s exit\n",__func__);
}

static int spi_hid_get_request(struct spi_hid *shid, u8 content_id)
{
	int ret;
	struct device *dev = &shid->spi->dev;
	struct spi_hid_output_report report = {
		.report_type = SPI_HID_OUTPUT_REPORT_TYPE_HID_GET_FEATURE,
		.content_length = 0,
		.content_id = content_id,
		.content = NULL,
	};

	DEVPRINT(DLEVEL_DEFAULT, "%s [0x%x]\n",__func__, content_id);
	ret = spi_hid_sync_request(shid, &report);
	if (ret) {
		dev_err(dev,
			"Expected get request response not received! Error %d\n",
			ret);
		schedule_work(&shid->error_work);
	}

	return ret;
}

static int spi_hid_set_request(struct spi_hid *shid,
		u8 *arg_buf, u16 arg_len, u8 content_id)
{
	struct device *dev = &shid->spi->dev;
	struct spi_hid_output_report report = {
		.report_type = SPI_HID_OUTPUT_REPORT_TYPE_HID_SET_FEATURE,
		.content_length = arg_len,
		.content_id = content_id,
		.content = arg_buf,
	};

	DEVPRINT(DLEVEL_DEFAULT, "%s [0x%x 0x%x]\n", __func__, content_id, arg_buf[0]);
	return spi_hid_sync_request(shid, &report);
}

static irqreturn_t spi_hid_dev_irq(int irq, void *_shid)
{
	struct spi_hid *shid = _shid;
	struct device *dev = &shid->spi->dev;
	struct spi_hid_input_header header;
	int ret = 0;

	DEVPRINT(DLEVEL_DEFAULT, "%s enter\n",__func__);

	if (atomic_dec_if_positive(&shid->irq_eater) == 0)  // == 0 means it was 1 before decrement
	{
		DEVPRINT(DLEVEL_DEFAULT, "   %s IRQ EATER EATING\n",__func__);
		goto out;
	}		

	if (!shid->irq_enabled) {
		DEVPRINT(DLEVEL_DEFAULT, "   %s IRQ off, skipping\n",__func__);
		goto out;
	}

	trace_spi_hid_dev_irq(shid, irq);
	trace_spi_hid_header_transfer(shid);

	if (fill) memset(&shid->input, 0xee, sizeof(shid->input));

	if (!shid->desc.extra_read_length) {
		ret = spi_hid_input_sync(shid, shid->input.header,
				sizeof(shid->input.header), true);
	} else {
		ret = spi_hid_input_sync(shid, shid->input.header,
				sizeof(shid->input.header) + shid->desc.extra_read_length + 4, true);
	}

	if (ret) {
		dev_err(dev, "Failed to transfer header: %d\n", ret);
		goto out;
	}

	if (shid->power_state == SPI_HID_POWER_MODE_OFF) {
		dev_warn(dev, "Device is off after header was received\n");
		goto out;
	}

	if (shid->input_message.status < 0) {
		dev_warn(dev, "%s:%d Error reading header: %d\n",
				__func__, __LINE__, shid->input_message.status);
		shid->bus_error_count++;
		shid->bus_last_error = shid->input_message.status;
		//schedule_work(&shid->error_work);
		goto out;
	}

	// sent a read but got back an idle
	// device probably irq'd for an input report that got accidentally
	// sent in an 02
	// typically value is A5
	if (shid->temp_spi_rx_buf[0] == SLAVE_STATE_IDLE) {
		/* completion_done returns 0 if there are waiters, otherwise 1 */
		DEVPRINT(DLEVEL_A6, "got idle in 0B after irq. waiters:%s\n",completion_done(&shid->output_done) ? "NONE": "YES");
		goto out;
	}

	spi_hid_populate_input_header(shid->input.header, &header);

	ret = spi_hid_bus_validate_header(shid, &header);

	if (ret) {
		dev_err(dev, "ERR Failed to validate header: %d\n", ret);
		PRINT_HEX_DUMP(dev, KERN_WARNING, "htx: ", DUMP_PREFIX_OFFSET, 32, 1, shid->temp_spi_tx_buf, SPI_HID_READ_APPROVAL_LEN, 0);
		PRINT_HEX_DUMP(dev, KERN_WARNING, "rx_buf: ", DUMP_PREFIX_OFFSET, 32, 1, shid->temp_spi_rx_buf, 32, 0);
		/* completion_done returns 0 if there are waiters, otherwise 1 */
		if (0 == completion_done(&shid->output_done)) {
			dev_err(dev, "ERR waiters\n");
			shid->response.body[0] = 0x10;
			complete(&shid->output_done);
		} else {
			dev_err(dev, "ERR no waiters\n");
		}
		goto out;
	}

	DEVPRINT(DLEVEL_DEFAULT, "   %s BODYFLAG:%d\n",__func__, header.have_body_flag);

	// if body does not follow set up next cycle to fetch (original code path)
	// else memcpy appended body and process as if we had done above

	if (!header.have_body_flag || !shid->desc.extra_read_length) {

		ret = spi_hid_input_sync(shid, shid->input.body, header.report_length,
					false);
		if (ret)
			dev_err(dev, "Failed to transfer body: %d\n", ret);

		if (shid->power_state == SPI_HID_POWER_MODE_OFF) {
			dev_warn(dev, "Device is off after body was received\n");
			goto out;
		}

		if (shid->input_message.status < 0) {
			dev_warn(dev, "Error reading body: %d\n",
					shid->input_message.status);
			shid->bus_error_count++;
			shid->bus_last_error = shid->input_message.status;
			//schedule_work(&shid->error_work);
			goto out;
		}

	} else {
		memcpy(&shid->input.body, header.body_ptr, header.report_length);
	}

	spi_hid_process_input_report(shid, &shid->input);

out:
	DEVPRINT(DLEVEL_DEFAULT, "%s complete\n",__func__);
	return IRQ_HANDLED;
}

/* hid_ll_driver interface functions */

static int spi_hid_ll_start(struct hid_device *hid)
{
	struct spi_device *spi = hid->driver_data;
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct device *dev = &shid->spi->dev;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	if (shid->desc.max_input_length < HID_MIN_BUFFER_SIZE) {
		dev_err(&shid->spi->dev,
			"HID_MIN_BUFFER_SIZE > max_input_length (%d)\n",
			shid->desc.max_input_length);
		return -EINVAL;
	}

	return 0;
}

static void spi_hid_ll_stop(struct hid_device *hid)
{
	struct spi_device *spi = hid->driver_data;
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct device *dev = &shid->spi->dev;
	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	hid->claimed = 0;
}

static int spi_hid_ll_open(struct hid_device *hid)
{
	struct spi_device *spi = hid->driver_data;
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct device *dev = &spi->dev;
	u8 prev_state = shid->power_state;
	int ret;

	DEVPRINT(DLEVEL_DEFAULT, "%s from:%pS\n",__func__, __builtin_return_address(0));
	if (shid->refresh_in_progress)
		return 0;

	ret = spi_hid_resume(shid);

	dev_dbg(dev, "%s: %s -> %s\n", __func__,
			spi_hid_power_mode_string(prev_state),
			spi_hid_power_mode_string(shid->power_state));

	if (ret == 0)
	{
		reinit_completion(&shid->ready_done);
		ret = wait_for_completion_interruptible_timeout(&shid->ready_done,
			msecs_to_jiffies(MAX(200,PROTOCOL_TIMEOUT_MSEC)));  // reset needs at least 20 at 2Mhz
	}

	if (ret == 0) {
		dev_err(dev, "Handshake timed out\n");
		return -EAGAIN;
	} else {
		dev_dbg(dev, "%s: ready\n", __func__);
		shid->ready = true;
		return 0;
	}
}

static void spi_hid_ll_close(struct hid_device *hid)
{
	struct spi_device *spi = hid->driver_data;
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct device *dev = &spi->dev;
	u8 prev_state = shid->power_state;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	if (shid->refresh_in_progress)
		return;

	mutex_lock(&shid->power_lock);
	spi_hid_suspend(shid);
	mutex_unlock(&shid->power_lock);

	shid->attempts = 0;

	dev_dbg(dev, "%s: %s -> %s\n", __func__,
			spi_hid_power_mode_string(prev_state),
			spi_hid_power_mode_string(shid->power_state));
}

static int spi_hid_ll_power(struct hid_device *hid, int level)
{
	struct spi_device *spi = hid->driver_data;
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct device *dev = &spi->dev;
	int ret = 0;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	tmutex_lock(&shid->output_lock);
	if (!shid->hid)
		ret = -ENODEV;
	tmutex_unlock(&shid->output_lock);

	return ret;
}

static int spi_hid_ll_parse(struct hid_device *hid)
{
	struct spi_device *spi = hid->driver_data;
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct device *dev = &spi->dev;
	int ret, len;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	tmutex_lock(&shid->request_lock);
	tmutex_lock(&shid->output_lock);

	len = spi_hid_report_descriptor_request(shid);
	if (len < 0) {
		dev_err(dev, "Report descriptor request failed, %d\n", len);
		ret = len;
		goto out;
	}

	/*
	 * FIXME: below call returning 0 doesn't mean that the report descriptor
	 * is good. We might be caching a crc32 of a corrupted r. d. or who
	 * knows what the FW sent. Need to have a feedback loop about r. d.
	 * being ok and only then cache it.
	 */
	ret = hid_parse_report(hid, (u8 *)shid->response.content, len);
	if (ret)
		dev_err(dev, "failed parsing report: %d\n", ret);
	else
		shid->report_descriptor_crc32 = crc32_le(0,
				(unsigned char const *)shid->response.content,
				len);

out:
	tmutex_unlock(&shid->output_lock);
	tmutex_unlock(&shid->request_lock);

	return ret;
}

static int spi_hid_ll_raw_request(struct hid_device *hid,
		unsigned char reportnum, u8 *buf, size_t len,
		unsigned char rtype, int reqtype)
{
	struct spi_device *spi = hid->driver_data;
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct device *dev = &spi->dev;
	int ret;

	ktime_t enter_time;
	u32 *bin_array;
	s64 diff_us;

	if (!shid->ready) {
		dev_err_ratelimited(&shid->spi->dev, "%s called in unready state\n",
								__func__);
		return -ENODEV;
	}

	enter_time = ktime_get();

	tmutex_lock(&shid->request_lock);
	tmutex_lock(&shid->output_lock);

	switch (reqtype) {
	case HID_REQ_SET_REPORT:
		shid->stats.llraw_set++;
	        DEVPRINT(DLEVEL_DEFAULT, "%s:%d SET_REPORT [0x%x 0x%x]\n",__func__, __LINE__, reportnum, buf[1]);
		if (buf[0] != reportnum) {
			dev_err(dev, "report id mismatch\n");
			ret = -EINVAL;
			break;
		}

		ret = spi_hid_set_request(shid, &buf[1], len - 1,
				reportnum);
		if (ret) {
			dev_err(dev, "failed to set report\n");
			break;
		}

	        DEVPRINT(DLEVEL_DEFAULT, "%s:%d SET_REPORT [0x%x 0x%x] %s\n\n",__func__, __LINE__, reportnum, buf[1], (shid->response.body[0] & 0x10) ? "STALL" : "NOstall");

	        if (shid->response.body[0] & 0x10) {  // stall_flag
			ret = -EPIPE;
			break;
		}

		ret = len;
		break;
	case HID_REQ_GET_REPORT:
		shid->stats.llraw_get++;
	        DEVPRINT(DLEVEL_DEFAULT, "%s:%d GET_REPORT [0x%x]\n",__func__, __LINE__, reportnum);
		ret = spi_hid_get_request(shid, reportnum);
		if (ret) {
			dev_err(dev, "failed to get report\n");
			break;
		}

	        DEVPRINT(DLEVEL_DEFAULT, "%s:%d GET_REPORT [0x%x] %s\n\n",__func__, __LINE__, reportnum, (shid->response.body[0] & 0x10) ? "STALL" : "NOstall");
	        if (shid->response.body[0] & 0x10) {  // stall_flag
			ret = -EPIPE;
			break;
		} else {
			ret = min_t(size_t, len,
				shid->response.body[1] | ((0x3f & shid->response.body[2]) << 8));
			memcpy(buf, &shid->response.content, ret);
		}
		break;
	default:
		dev_err(dev, "invalid request type\n");
		ret = -EIO;
	}

	tmutex_unlock(&shid->output_lock);
	tmutex_unlock(&shid->request_lock);

	shid->stats.llraw_total++;


	if (ret >= 0) {
		shid->stats.llraw_ok++;
		bin_array = shid->stats.llraw_ok_bins;
	} else if (ret == -EPIPE) {
		shid->stats.llraw_epipe++;
		bin_array = shid->stats.llraw_epipe_bins;
	} else if (ret == -EAGAIN) {
		shid->stats.llraw_eagain++;
		bin_array = shid->stats.llraw_eagain_bins;
	} else {
		shid->stats.llraw_other++;
	}

	diff_us = ktime_to_us(ktime_sub(ktime_get(), enter_time));
	if (diff_us >= 0) {
		if (diff_us / LLRAW_BIN_SIZE >= LLRAW_BIN_COUNT) {
			bin_array[LLRAW_BIN_COUNT-1]++;
		} else {
			bin_array[diff_us / LLRAW_BIN_SIZE]++;
		}
	}


	return ret;
}

static int spi_hid_ll_output_report(struct hid_device *hid,
		u8 *buf, size_t len)
{
	int ret;
	struct spi_device *spi = hid->driver_data;
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct device *dev = &spi->dev;
	struct spi_hid_output_report report = {
		.report_type = SPI_HID_OUTPUT_REPORT_TYPE_HID_OUTPUT_REPORT,
		.content_length = len - 1,
		.content_id = buf[0],
		.content = &buf[1],
	};

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	tmutex_lock(&shid->request_lock);
	tmutex_lock(&shid->output_lock);
	if (!shid->ready) {
		dev_err_ratelimited(dev, "%s called in unready state\n", __func__);
		ret = -ENODEV;
		goto out;
	}

	if (shid->desc.no_output_report_ack)
		ret = spi_hid_send_output_report(shid, &report);
	else
		ret = spi_hid_sync_request(shid, &report);

	if (ret)
		dev_err(dev, "failed to send output report\n");

out:
	tmutex_unlock(&shid->output_lock);
	tmutex_unlock(&shid->request_lock);

	if (ret > 0)
		return -ret;

	if (ret < 0)
		return ret;

	return len;
}

static struct hid_ll_driver spi_hid_ll_driver = {
	.start = spi_hid_ll_start,
	.stop = spi_hid_ll_stop,
	.open = spi_hid_ll_open,
	.close = spi_hid_ll_close,
	.power = spi_hid_ll_power,
	.parse = spi_hid_ll_parse,
	.output_report = spi_hid_ll_output_report,
	.raw_request = spi_hid_ll_raw_request,
};

const struct of_device_id spi_hid_of_match[] = {
	{ .compatible = "hid-over-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, spi_hid_of_match);

static ssize_t spihid_stats_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct spi_hid *shid = dev_get_drvdata(dev);

	ret += snprintf(buf + ret, PAGE_SIZE, "%s:\n", shid->desc.sn_string);

	ret += snprintf(buf + ret, PAGE_SIZE, "    hid_input_busy:%d\n",shid->stats.hid_input_busy);

	ret += snprintf(buf + ret, PAGE_SIZE, "    total:%d ok:%d retry:%d\n",
			shid->stats.spi_total_count, shid->stats.spi_total_count - shid->stats.spi_retry_count, shid->stats.spi_retry_count);

	ret += snprintf(buf + ret, PAGE_SIZE, "    llraw_set:%d llraw_get:%d llraw_total:%d\n",shid->stats.llraw_set,shid->stats.llraw_get,shid->stats.llraw_total);
	ret += snprintf(buf + ret, PAGE_SIZE, "    llraw_ok:%d\tllraw_epipe:%d\tllraw_eagain:%d llraw_other:%d\n",shid->stats.llraw_ok,shid->stats.llraw_epipe,shid->stats.llraw_eagain,shid->stats.llraw_other);
	ret += snprintf(buf + ret, PAGE_SIZE, "                      \tok\tepipe\teagain\n");

	for (int i = 0; i<LLRAW_BIN_COUNT; i++) {
		ret += snprintf(buf + ret, PAGE_SIZE, "         [%d-%d]:\t%d\t%d\t%d\n",i*LLRAW_BIN_SIZE,((i+1)*LLRAW_BIN_SIZE)-1,
			shid->stats.llraw_ok_bins[i], shid->stats.llraw_epipe_bins[i], shid->stats.llraw_eagain_bins[i]);
	}

	memset(&shid->stats,0,sizeof(shid->stats));

	return ret;

}
static DEVICE_ATTR_RO(spihid_stats);

static ssize_t spihid_bus_error_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct spi_hid *shid = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d (%d)\n",
			shid->bus_error_count, shid->bus_last_error);
}
static DEVICE_ATTR_RO(spihid_bus_error_count);

static ssize_t spihid_regulator_error_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct spi_hid *shid = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d (%d)\n",
			shid->regulator_error_count,
			shid->regulator_last_error);
}
static DEVICE_ATTR_RO(spihid_regulator_error_count);

static ssize_t spihid_device_initiated_reset_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct spi_hid *shid = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", shid->dir_count);
}
static DEVICE_ATTR_RO(spihid_device_initiated_reset_count);

static const struct attribute *const spi_hid_attributes[] = {
	&dev_attr_spihid_bus_error_count.attr,
	&dev_attr_spihid_regulator_error_count.attr,
	&dev_attr_spihid_device_initiated_reset_count.attr,
	&dev_attr_spihid_stats.attr,
	NULL	/* Terminator */
};

static int spi_hid_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct spi_hid *shid;
	unsigned long irqflags;
	int ret;

	DEVPRINT(DLEVEL_DEFAULT, "%s starts\n",__func__);
	if (spi->irq <= 0) {
		dev_err(dev, "Missing IRQ\n");
		ret = spi->irq ?: -EINVAL;
		goto err0;
	}

	shid = devm_kzalloc(dev, sizeof(struct spi_hid), GFP_KERNEL);
	if (!shid) {
		ret = -ENOMEM;
		goto err0;
	}

	// force a default that will apply even before reading descriptor
	shid->desc.extra_read_length = MIN_REPORT_PACKET_LENGTH - 13; // calculate properly!

	shid->timeout_msec = 30;
	shid->spi = spi;
	shid->power_state = SPI_HID_POWER_MODE_ON;
	spi_set_drvdata(spi, shid);

	ret = sysfs_create_files(&dev->kobj, spi_hid_attributes);
	if (ret) {
		dev_err(dev, "Unable to create sysfs attributes\n");
		goto err0;
	}

	ret = spi_hid_of_populate_config(&shid->conf, dev);
	if (ret) {
		dev_err(dev, "%s: unable to populate config data\n", __func__);
		goto err1;
	}

	/* Using now populated conf let's pre-calculate the read approvals */
	spi_hid_populate_read_approvals(&shid->conf, shid->read_approval_header,
						shid->read_approval_body);

	mutex_init(&shid->hid_input_lock);
	mutex_init(&shid->output_lock);
	mutex_init(&shid->request_lock);
	mutex_init(&shid->power_lock);
	mutex_init(&shid->sync_lock);
	init_completion(&shid->output_done);
	init_completion(&shid->ready_done);

	INIT_WORK(&shid->reset_response_work, spi_hid_reset_response_work);
	INIT_WORK(&shid->create_device_work, spi_hid_create_device_work);
	INIT_WORK(&shid->refresh_device_work, spi_hid_refresh_device_work);
	INIT_WORK(&shid->error_work, spi_hid_error_work);

	/*
	 * At the end of probe we initialize the device:
	 *   0) Default pinctrl in DT: assert reset, bias the interrupt line
	 *   1) sleep minimal reset delay
	 *   2) request IRQ
	 *   3) power up the device
	 *   4) deassert reset (high)
	 * After this we expect an IRQ with a reset response.
	 */

	spi_hid_of_assert_reset(&shid->conf, dev);

	spi_hid_of_sleep_minimal_reset_delay(&shid->conf);

	irqflags = irq_get_trigger_type(spi->irq) | IRQF_ONESHOT;
	ret = devm_request_threaded_irq(dev, spi->irq, NULL, spi_hid_dev_irq,
			irqflags, dev_name(&spi->dev), shid);
	if (ret) {
		dev_err(dev, "%s: unable to request threaded IRQ\n", __func__);
		goto err1;
	}
	shid->irq_enabled = true;

	ret = spi_hid_of_power_up(&shid->conf);
	if (ret) {
		dev_err(dev, "%s: could not power up\n", __func__);
		shid->regulator_error_count++;
		shid->regulator_last_error = ret;
		goto err1;
	}

	spi_hid_of_deassert_reset(&shid->conf, dev);

	shid->last_event_finished = ktime_get();

	dev_dbg(dev, "%s: d3 -> %s\n", __func__,
			spi_hid_power_mode_string(shid->power_state));

	DEVPRINT(DLEVEL_DEFAULT, "%s complete\n",__func__);
	return 0;

err1:
	sysfs_remove_files(&dev->kobj, spi_hid_attributes);

err0:
	return ret;
}

static void spi_hid_remove(struct spi_device *spi)
{
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct device *dev = &spi->dev;
	int ret;

	DEVPRINT(DLEVEL_DEFAULT, "%s\n",__func__);
	spi_hid_of_assert_reset(&shid->conf, dev);
	ret = spi_hid_of_power_down(&shid->conf);
	if (ret) {
		dev_err(dev, "failed to disable regulator\n");
		shid->regulator_error_count++;
		shid->regulator_last_error = ret;
	}
	sysfs_remove_files(&dev->kobj, spi_hid_attributes);
	spi_hid_stop_hid(shid);

	return;
}

static const struct spi_device_id spi_hid_id_table[] = {
	{ "hid", 0 },
	{ "hid-over-spi", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, spi_hid_id_table);

static struct spi_driver spi_hid_driver = {
	.driver = {
		.name	= "spi_hid",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(spi_hid_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= spi_hid_probe,
	.remove		= spi_hid_remove,
	.id_table	= spi_hid_id_table,
};

module_spi_driver(spi_hid_driver);

MODULE_DESCRIPTION("HID over SPI transport driver");
MODULE_AUTHOR("Dmitry Antipov <dmanti@microsoft.com>");
MODULE_LICENSE("GPL");
