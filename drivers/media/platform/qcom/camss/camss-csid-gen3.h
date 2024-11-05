/* SPDX-License-Identifier: GPL-2.0 */
/*
 * camss-csid-gen3.h
 *
 * Qualcomm MSM Camera Subsystem - CSID (CSI Decoder) Module Generation 3
 *
 * Copyright (c) 2024 Qualcomm Technologies, Inc.
 */
#ifndef QC_MSM_CAMSS_CSID_GEN3_H
#define QC_MSM_CAMSS_CSID_GEN3_H

#define DECODE_FORMAT_UNCOMPRESSED_8_BIT	0x1
#define DECODE_FORMAT_UNCOMPRESSED_10_BIT	0x2
#define DECODE_FORMAT_UNCOMPRESSED_12_BIT	0x3
#define DECODE_FORMAT_UNCOMPRESSED_14_BIT	0x4
#define DECODE_FORMAT_UNCOMPRESSED_16_BIT	0x5
#define DECODE_FORMAT_UNCOMPRESSED_20_BIT	0x6
#define DECODE_FORMAT_UNCOMPRESSED_24_BIT	0x7
#define DECODE_FORMAT_PAYLOAD_ONLY		0xf


#define PLAIN_FORMAT_PLAIN8	0x0 /* supports DPCM, UNCOMPRESSED_6/8_BIT */
#define PLAIN_FORMAT_PLAIN16	0x1 /* supports DPCM, UNCOMPRESSED_10/16_BIT */
#define PLAIN_FORMAT_PLAIN32	0x2 /* supports UNCOMPRESSED_20_BIT */

struct csid_path_regs_info {
	u32 irq_status;
	u32 irq_mask;
	u32 irq_clear;
	u32 irq_set;
	u32 cfg0;
	u32 cfg1;
	u32 ctrl;
	u32 multi_vcdt_cfg0;
	u32 irq_subsample_pattern;
	u32 irq_subsample_period;
	u32 curr_sof_lower;
	u32 curr_sof_upper;
	u32 curr_eof_lower;
	u32 curr_eof_upper;
};

static struct csid_path_regs_info csid_path_regs[CAMSS_EXT_CSID_PATH_TYPE_MAX] = {
	// RDI0
	{
		.irq_status = 0xec,
		.irq_mask = 0xf0,
		.irq_clear = 0xf4,
		.irq_set = 0xf8,
		.cfg0 = 0x500,
		.cfg1 = 0x510,
		.ctrl = 0x504,
		.multi_vcdt_cfg0 = 0x50c,
		.irq_subsample_pattern = 0x548,
		.irq_subsample_period = 0x54c,
		.curr_sof_lower = 0x594,
		.curr_sof_upper = 0x598,
		.curr_eof_lower = 0x5a4,
		.curr_eof_upper = 0x5a8,
	},
	// RDI1
	{
		.irq_status = 0xfc,
		.irq_mask = 0x100,
		.irq_clear = 0x104,
		.irq_set = 0x108,
		.cfg0 = 0x600,
		.cfg1 = 0x610,
		.ctrl = 0x604,
		.multi_vcdt_cfg0 = 0x60c,
		.irq_subsample_pattern = 0x648,
		.irq_subsample_period = 0x64c,
		.curr_sof_lower = 0x694,
		.curr_sof_upper = 0x698,
		.curr_eof_lower = 0x6a4,
		.curr_eof_upper = 0x6a8,
	},
	// RDI2
	{
		.irq_status = 0x10c,
		.irq_mask = 0x110,
		.irq_clear = 0x114,
		.irq_set = 0x118,
		.cfg0 = 0x700,
		.cfg1 = 0x710,
		.ctrl = 0x704,
		.multi_vcdt_cfg0 = 0x70c,
		.irq_subsample_pattern = 0x748,
		.irq_subsample_period = 0x74c,
		.curr_sof_lower = 0x794,
		.curr_sof_upper = 0x798,
		.curr_eof_lower = 0x7a4,
		.curr_eof_upper = 0x7a8,
	},
	// IPP0
	{
		.irq_status = 0xac,
		.irq_mask = 0xb0,
		.irq_clear = 0xb4,
		.irq_set = 0xb8,
		.cfg0 = 0x300,
		.cfg1 = 0x310,
		.ctrl = 0x304,
		.multi_vcdt_cfg0 = 0x30c,
		.irq_subsample_pattern = 0x37c,
		.irq_subsample_period = 0x380,
		.curr_sof_lower = 0x398,
		.curr_sof_upper = 0x39c,
		.curr_eof_lower = 0x3a8,
		.curr_eof_upper = 0x3ac,
	}
};

#endif /* QC_MSM_CAMSS_CSID_GEN3_H */
