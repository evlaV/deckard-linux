/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 Microsoft Corporation
 */

#ifndef SPI_HID_CORE_H
#define SPI_HID_CORE_H

#include <linux/completion.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "spi-hid-of.h"

/* Protocol constants */
#define SPI_HID_READ_APPROVAL_CONSTANT		0xff
#define SPI_HID_INPUT_HEADER_SYNC_BYTE		0x5a
#define SPI_HID_INPUT_HEADER_VERSION		0x03
#define SPI_HID_INPUT_HEADER_VERSION_NEXT	0x04	// NB: must skip LSN of "not ready" pattern from slave, currently 0x1d (0x0d)
#define SPI_HID_SUPPORTED_VERSION		0x0300

/* Protocol message size constants */
#define SPI_HID_READ_APPROVAL_LEN		5
#define SPI_HID_INPUT_HEADER_LEN		4
#define SPI_HID_INPUT_BODY_LEN			4
#define SPI_HID_OUTPUT_HEADER_LEN		8
#define SPI_HID_DEVICE_DESCRIPTOR_LENGTH	(24+14+16+2)

/* Protocol message type constants */
#define SPI_HID_INPUT_REPORT_TYPE_DATA				0x01
#define SPI_HID_INPUT_REPORT_TYPE_RESET_RESP			0x03
#define SPI_HID_INPUT_REPORT_TYPE_COMMAND_RESP			0x04
#define SPI_HID_INPUT_REPORT_TYPE_GET_FEATURE_RESP		0x05
#define SPI_HID_INPUT_REPORT_TYPE_DEVICE_DESC			0x07
#define SPI_HID_INPUT_REPORT_TYPE_REPORT_DESC			0x08
#define SPI_HID_INPUT_REPORT_TYPE_SET_FEATURE_RESP		0x09
#define SPI_HID_INPUT_REPORT_TYPE_SET_OUTPUT_REPORT_RESP	0x0a
#define SPI_HID_INPUT_REPORT_TYPE_GET_INPUT_REPORT_RESP		0x0b

#define SPI_HID_OUTPUT_REPORT_TYPE_DEVICE_DESC_REQUEST	0x01
#define SPI_HID_OUTPUT_REPORT_TYPE_REPORT_DESC_REQUEST	0x02
#define SPI_HID_OUTPUT_REPORT_TYPE_HID_SET_FEATURE	0x03
#define SPI_HID_OUTPUT_REPORT_TYPE_HID_GET_FEATURE	0x04
#define SPI_HID_OUTPUT_REPORT_TYPE_HID_OUTPUT_REPORT	0x05
#define SPI_HID_OUTPUT_REPORT_TYPE_INPUT_REPORT_REQUEST	0x06
#define SPI_HID_OUTPUT_REPORT_TYPE_COMMAND		0x07

#define SPI_HID_OUTPUT_REPORT_CONTENT_ID_DESC_REQUEST	0x00

/* Power mode constants */
#define SPI_HID_POWER_MODE_ON			0x01
#define SPI_HID_POWER_MODE_SLEEP		0x02
#define SPI_HID_POWER_MODE_OFF			0x03
#define SPI_HID_POWER_MODE_WAKING_SLEEP		0x04

#define SPI_DEF 0x1D
#define SPI_ORC 0xEE

#define SLAVE_STATE_IDLE    		0xA3
#define SLAVE_STATE_HEADER  		0xA5
#define SLAVE_STATE_INPUT_REPORT    	0xA6
#define SLAVE_STATE_BODY    		0xA7
#define SLAVE_STATE_NOTREADY    	0x1d

#define LLRAW_BIN_COUNT		24
#define LLRAW_BIN_SIZE		250

/* Raw input buffer with data from the bus */
struct spi_hid_input_buf {
	u8 header[SPI_HID_INPUT_HEADER_LEN];
	u8 header_body[SZ_8K];
	u8 body[SPI_HID_INPUT_BODY_LEN];
	u8 content[SZ_8K];
} __packed;

/* Processed data from  input report header */
struct spi_hid_input_header {
	u8 version;
	u8 have_body_flag;
	u16 report_length;
	u8 last_fragment_flag;
	u8 sync_const;
	u8 *body_ptr;
};

/* Processed data from input report body, excluding the content */
struct spi_hid_input_body {
	u8 report_type;
	u8 stall_flag;
	u16 content_length;
	u8 content_id;
};

/* Processed data from an input report */
struct spi_hid_input_report {
	u8 report_type;
	u16 content_length;
	u8 content_id;
	u8 content[257];
} __packed;

/* Raw output report buffer to be put on the bus */
struct spi_hid_output_buf {
	u8 header[SPI_HID_OUTPUT_HEADER_LEN];
	u8 content[SZ_8K];
};

/* Data necessary to send an output report */
struct spi_hid_output_report {
	u8 report_type;
	u16 content_length;
	u8 content_id;
	u8 *content;
};

/* Raw content in device descriptor */
struct spi_hid_device_desc_raw {
	__le16 wDeviceDescLength;
	__le16 bcdVersion;
	__le16 wReportDescLength;
	__le16 wMaxInputLength;
	__le16 wMaxOutputLength;
	__le16 wMaxFragmentLength;
	__le16 wVendorID;
	__le16 wProductID;
	__le16 wVersionID;
	__le16 wFlags;
	u8 sn[14];
	u8 name[16];
	u8 read_size;
	u8 reserved[5];
} __packed;

/* Processed data from a device descriptor */
struct spi_hid_device_descriptor {
	u16 hid_version;
	u16 report_descriptor_length;
	u16 max_input_length;
	u16 max_output_length;
	u16 max_fragment_length;
	u16 vendor_id;
	u16 product_id;
	u16 version_id;
	u8 no_output_report_ack;
	u8 extra_read_length;
	char sn_string[sizeof_field(struct spi_hid_device_desc_raw,sn)+1];
	char name_string[sizeof_field(struct spi_hid_device_desc_raw,name)+1];
	u8 spi_protocol_level;
};

struct spi_hid_input_raw {
	u8 state;
	u8 unused[4];
	struct spi_hid_input_buf inbuf; 
} __packed;


/* Driver context */
struct spi_hid {
	struct spi_device	*spi;
	struct hid_device	*hid;

	//struct spi_transfer	input_transfer[2];
	struct spi_transfer	output_transfer;
	struct spi_message	input_message;	// IRQ & spi_hid_input_sync ONLY
	struct spi_message	output_message;

	struct spi_hid_of_config conf;
	struct spi_hid_device_descriptor desc;
	struct spi_hid_output_buf output;
	struct spi_hid_input_buf input;			// Belongs to IRQ
	struct spi_hid_input_buf response;
	struct spi_hid_input_raw out_input;

	u8 temp_spi_rx_buf[4096];		// IRQ & spi_hid_input_sync ONLY
	u8 temp_spi_tx_buf[4096];		// IRQ & spi_hid_input_sync ONLY

	u32 input_transfer_pending;

	u8 power_state;

	u8 attempts;

	/*
	 * ready flag indicates that the FW is ready to accept commands and
	 * requests. The FW becomes ready after sending the report descriptor.
	 */
	bool ready;
	/*
	 * refresh_in_progress is set to true while the refresh_device worker
	 * thread is destroying and recreating the hidraw device. When this flag
	 * is set to true, the ll_close and ll_open functions will not cause
	 * power state changes.
	 */
	bool refresh_in_progress;

	bool irq_enabled;

	struct work_struct reset_response_work;
	struct work_struct create_device_work;
	struct work_struct refresh_device_work;
	struct work_struct error_work;

	struct mutex hid_input_lock;
	struct mutex output_lock;
	struct mutex request_lock;
	struct mutex power_lock;
	struct mutex sync_lock;
	struct completion output_done;
	struct completion ready_done;

	u8 read_approval_header[SPI_HID_READ_APPROVAL_LEN];
	u8 read_approval_body[SPI_HID_READ_APPROVAL_LEN];

	u32 report_descriptor_crc32;

	u32 regulator_error_count;
	int regulator_last_error;
	u32 bus_error_count;
	int bus_last_error;
	u32 dir_count;

	struct Stats {
		u32 hid_input_busy;
		u32 spi_retry_count;
		u32 spi_total_count;
		u32 llraw_set;
		u32 llraw_get;
		u32 llraw_total;
		u32 llraw_ok;
		u32 llraw_epipe;
		u32 llraw_eagain;
		u32 llraw_other;
		u32 llraw_ok_bins[LLRAW_BIN_COUNT];
		u32 llraw_epipe_bins[LLRAW_BIN_COUNT];
		u32 llraw_eagain_bins[LLRAW_BIN_COUNT];
	} stats;


	atomic_t irq_eater;

	ktime_t last_event_finished;

	u8 timeout_msec;
};

#endif
