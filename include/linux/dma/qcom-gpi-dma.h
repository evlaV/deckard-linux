/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020, Linaro Limited
 */

#ifndef QCOM_GPI_DMA_H
#define QCOM_GPI_DMA_H

/**
 * enum spi_transfer_cmd - spi transfer commands
 */
enum spi_transfer_cmd {
	SPI_TX = 1,
	SPI_RX,
	SPI_DUPLEX,
};

/**
 * struct gpi_spi_config - spi config for peripheral
 *
 * @loopback_en: spi loopback enable when set
 * @clock_pol_high: clock polarity
 * @data_pol_high: data polarity
 * @pack_en: process tx/rx buffers as packed
 * @word_len: spi word length
 * @clk_div: source clock divider
 * @clk_src: serial clock
 * @cmd: spi cmd
 * @fragmentation: keep CS asserted at end of sequence
 * @cs: chip select toggle
 * @set_config: set peripheral config
 * @rx_len: receive length for buffer
 */
struct gpi_spi_config {
	u8 set_config;
	u8 loopback_en;
	u8 clock_pol_high;
	u8 data_pol_high;
	u8 pack_en;
	u8 word_len;
	u8 fragmentation;
	u8 cs;
	u32 clk_div;
	u32 clk_src;
	enum spi_transfer_cmd cmd;
	u32 rx_len;
};

enum i2c_op {
	I2C_WRITE = 1,
	I2C_READ,
};

/**
 * struct gpi_i2c_config - i2c config for peripheral
 *
 * @pack_enable: process tx/rx buffers as packed
 * @cycle_count: clock cycles to be sent
 * @high_count: high period of clock
 * @low_count: low period of clock
 * @clk_div: source clock divider
 * @addr: i2c bus address
 * @stretch: stretch the clock at eot
 * @set_config: set peripheral config
 * @rx_len: receive length for buffer
 * @op: i2c cmd
 * @muli-msg: is part of multi i2c r-w msgs
 */
struct gpi_i2c_config {
	u8 set_config;
	u8 pack_enable;
	u8 cycle_count;
	u8 high_count;
	u8 low_count;
	u8 addr;
	u8 stretch;
	u16 clk_div;
	u32 rx_len;
	enum i2c_op op;
	bool multi_msg;
};

#define GSI_QSI_FW_MAGIC       0x20495351UL

/**
 * struct gpi_qsi_fw_hdr - GPI firmware header
 *
 * @magic: magic value 'QSI '
 * @version: Structure version number = 1
 * @core_version: QUPV3_HW_VERSION
 * @fw_version: from GSI FW
 * @fw_size_in_items: Items count to load into IRAM
 * @fw_offset: Bytes offset of IRAM Items
 * @iep_size_in_items: Items count of IEP entries
 * @iep_offset: Byte offset of IEP entries
 */
struct __packed gpi_qsi_fw_hdr {
	u32 magic;
	u32 version;
	u32 core_version;
	u32 fw_version;
	u16 fw_size_in_items;
	u16 fw_offset;
	u16 iep_size_in_items;
	u16 iep_offset;
};

struct gpi_qsi_fw_iram {
	u32 iram_dword0;
	u32 iram_dword1;
};

struct gpi_qsi_fw_iep {
	u32 offset;
	u32 value;
};

#endif /* QCOM_GPI_DMA_H */
