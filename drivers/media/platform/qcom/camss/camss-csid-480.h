/* SPDX-License-Identifier: GPL-2.0 */
/*
 * camss-csid-480.h
 *
 * Qualcomm MSM Camera Subsystem - CSID (CSI Decoder) version 480
 *
 * Copyright (C) 2026 Linaro Ltd.
 */
#ifndef __CAMSS_CSID_480_H__
#define __CAMSS_CSID_480_H__

/* The CSID 2 IP-block is different from the others,
 * and is of a bare-bones Lite version, with no PIX
 * interface support. As a result of that it has an
 * alternate register layout.
 */

#define CSID_RST_STROBES	0x10
#define		RST_STROBES	0

#define CSID_CSI2_RX_IRQ_STATUS	0x20
#define	CSID_CSI2_RX_IRQ_MASK	0x24
#define CSID_CSI2_RX_IRQ_CLEAR	0x28

#define CSID_CSI2_RDIN_IRQ_STATUS(rdi)		((csid_is_lite(csid) ? 0x30 : 0x40) \
						 + 0x10 * (rdi))
#define CSID_CSI2_RDIN_IRQ_MASK(rdi)		((csid_is_lite(csid) ? 0x34 : 0x44) \
						 + 0x10 * (rdi))
#define CSID_CSI2_RDIN_IRQ_CLEAR(rdi)		((csid_is_lite(csid) ? 0x38 : 0x48) \
						 + 0x10 * (rdi))
#define CSID_CSI2_RDIN_IRQ_SET(rdi)		((csid_is_lite(csid) ? 0x3C : 0x4C) \
						 + 0x10 * (rdi))

#define CSID_TOP_IRQ_STATUS	0x70
#define		TOP_IRQ_STATUS_RESET_DONE 0
#define CSID_TOP_IRQ_MASK	0x74
#define CSID_TOP_IRQ_CLEAR	0x78
#define CSID_TOP_IRQ_SET	0x7C
#define CSID_IRQ_CMD		0x80
#define		IRQ_CMD_CLEAR	0
#define		IRQ_CMD_SET	4

#define CSID_CSI2_RX_CFG0	0x100
#define		CSI2_RX_CFG0_NUM_ACTIVE_LANES	0
#define		CSI2_RX_CFG0_DL0_INPUT_SEL	4
#define		CSI2_RX_CFG0_DL1_INPUT_SEL	8
#define		CSI2_RX_CFG0_DL2_INPUT_SEL	12
#define		CSI2_RX_CFG0_DL3_INPUT_SEL	16
#define		CSI2_RX_CFG0_PHY_NUM_SEL	20
#define		CSI2_RX_CFG0_PHY_TYPE_SEL	24

#define CSID_CSI2_RX_CFG1	0x104
#define		CSI2_RX_CFG1_PACKET_ECC_CORRECTION_EN		0
#define		CSI2_RX_CFG1_DE_SCRAMBLE_EN			1
#define		CSI2_RX_CFG1_VC_MODE				2
#define		CSI2_RX_CFG1_COMPLETE_STREAM_EN			4
#define		CSI2_RX_CFG1_COMPLETE_STREAM_FRAME_TIMING	5
#define		CSI2_RX_CFG1_MISR_EN				6
#define		CSI2_RX_CFG1_CGC_MODE				7
#define			CGC_MODE_DYNAMIC_GATING		0
#define			CGC_MODE_ALWAYS_ON		1

#define CSID_RDI_CFG0(rdi)			((csid_is_lite(csid) ? 0x200 : 0x300) \
						 + 0x100 * (rdi))
#define		RDI_CFG0_BYTE_CNTR_EN		0
#define		RDI_CFG0_FORMAT_MEASURE_EN	1
#define		RDI_CFG0_TIMESTAMP_EN		2
#define		RDI_CFG0_DROP_H_EN		3
#define		RDI_CFG0_DROP_V_EN		4
#define		RDI_CFG0_CROP_H_EN		5
#define		RDI_CFG0_CROP_V_EN		6
#define		RDI_CFG0_MISR_EN		7
#define		RDI_CFG0_CGC_MODE		8
#define			CGC_MODE_DYNAMIC	0
#define			CGC_MODE_ALWAYS_ON	1
#define		RDI_CFG0_PLAIN_ALIGNMENT	9
#define			PLAIN_ALIGNMENT_LSB	0
#define			PLAIN_ALIGNMENT_MSB	1
#define		RDI_CFG0_PLAIN_FORMAT		10
#define		RDI_CFG0_DECODE_FORMAT		12
#define		RDI_CFG0_DATA_TYPE		16
#define		RDI_CFG0_VIRTUAL_CHANNEL	22
#define		RDI_CFG0_DT_ID			27
#define		RDI_CFG0_EARLY_EOF_EN		29
#define		RDI_CFG0_PACKING_FORMAT		30
#define		RDI_CFG0_ENABLE			31

#define CSID_RDI_CFG1(rdi)			((csid_is_lite(csid) ? 0x204 : 0x304)\
						+ 0x100 * (rdi))
#define		RDI_CFG1_TIMESTAMP_STB_SEL	0

#define CSID_RDI_CTRL(rdi)			((csid_is_lite(csid) ? 0x208 : 0x308)\
						+ 0x100 * (rdi))
#define		RDI_CTRL_HALT_CMD		0
#define			HALT_CMD_HALT_AT_FRAME_BOUNDARY		0
#define			HALT_CMD_RESUME_AT_FRAME_BOUNDARY	1
#define		RDI_CTRL_HALT_MODE		2

#define CSID_RDI_FRM_DROP_PATTERN(rdi)			((csid_is_lite(csid) ? 0x20C : 0x30C)\
							+ 0x100 * (rdi))
#define CSID_RDI_FRM_DROP_PERIOD(rdi)			((csid_is_lite(csid) ? 0x210 : 0x310)\
							+ 0x100 * (rdi))
#define CSID_RDI_IRQ_SUBSAMPLE_PATTERN(rdi)		((csid_is_lite(csid) ? 0x214 : 0x314)\
							+ 0x100 * (rdi))
#define CSID_RDI_IRQ_SUBSAMPLE_PERIOD(rdi)		((csid_is_lite(csid) ? 0x218 : 0x318)\
							+ 0x100 * (rdi))
#define CSID_RDI_RPP_PIX_DROP_PATTERN(rdi)		((csid_is_lite(csid) ? 0x224 : 0x324)\
							+ 0x100 * (rdi))
#define CSID_RDI_RPP_PIX_DROP_PERIOD(rdi)		((csid_is_lite(csid) ? 0x228 : 0x328)\
							+ 0x100 * (rdi))
#define CSID_RDI_RPP_LINE_DROP_PATTERN(rdi)		((csid_is_lite(csid) ? 0x22C : 0x32C)\
							+ 0x100 * (rdi))
#define CSID_RDI_RPP_LINE_DROP_PERIOD(rdi)		((csid_is_lite(csid) ? 0x230 : 0x330)\
							+ 0x100 * (rdi))

#define CSID_TPG_CTRL		0x600
#define		TPG_CTRL_TEST_EN		0
#define		TPG_CTRL_FS_PKT_EN		1
#define		TPG_CTRL_FE_PKT_EN		2
#define		TPG_CTRL_NUM_ACTIVE_LANES	4
#define		TPG_CTRL_CYCLES_BETWEEN_PKTS	8
#define		TPG_CTRL_NUM_TRAIL_BYTES	20

#define CSID_TPG_VC_CFG0	0x604
#define		TPG_VC_CFG0_VC_NUM			0
#define		TPG_VC_CFG0_NUM_ACTIVE_SLOTS		8
#define			NUM_ACTIVE_SLOTS_0_ENABLED	0
#define			NUM_ACTIVE_SLOTS_0_1_ENABLED	1
#define			NUM_ACTIVE_SLOTS_0_1_2_ENABLED	2
#define			NUM_ACTIVE_SLOTS_0_1_3_ENABLED	3
#define		TPG_VC_CFG0_LINE_INTERLEAVING_MODE	10
#define			INTELEAVING_MODE_INTERLEAVED	0
#define			INTELEAVING_MODE_ONE_SHOT	1
#define		TPG_VC_CFG0_NUM_FRAMES			16

#define CSID_TPG_VC_CFG1	0x608
#define		TPG_VC_CFG1_H_BLANKING_COUNT		0
#define		TPG_VC_CFG1_V_BLANKING_COUNT		12
#define		TPG_VC_CFG1_V_BLANK_FRAME_WIDTH_SEL	24

#define CSID_TPG_LFSR_SEED	0x60C

#define CSID_TPG_DT_n_CFG_0(n)	(0x610 + (n) * 0xC)
#define		TPG_DT_n_CFG_0_FRAME_HEIGHT	0
#define		TPG_DT_n_CFG_0_FRAME_WIDTH	16

#define CSID_TPG_DT_n_CFG_1(n)	(0x614 + (n) * 0xC)
#define		TPG_DT_n_CFG_1_DATA_TYPE	0
#define		TPG_DT_n_CFG_1_ECC_XOR_MASK	8
#define		TPG_DT_n_CFG_1_CRC_XOR_MASK	16

#define CSID_TPG_DT_n_CFG_2(n)	(0x618 + (n) * 0xC)
#define		TPG_DT_n_CFG_2_PAYLOAD_MODE		0
#define		TPG_DT_n_CFG_2_USER_SPECIFIED_PAYLOAD	4
#define		TPG_DT_n_CFG_2_ENCODE_FORMAT		16

#define CSID_TPG_COLOR_BARS_CFG	0x640
#define		TPG_COLOR_BARS_CFG_UNICOLOR_BAR_EN	0
#define		TPG_COLOR_BARS_CFG_UNICOLOR_BAR_SEL	4
#define		TPG_COLOR_BARS_CFG_SPLIT_EN		5
#define		TPG_COLOR_BARS_CFG_ROTATE_PERIOD	8

#define CSID_TPG_COLOR_BOX_CFG	0x644
#define		TPG_COLOR_BOX_CFG_MODE		0
#define		TPG_COLOR_BOX_PATTERN_SEL	2

#endif /* __CAMSS_CSID_480_H__ */
