/* SPDX-License-Identifier: GPL-2.0 */
/*
 * camss-csid-gen3.h
 *
 * Qualcomm MSM Camera Subsystem - CSID (CSI Decoder) Module Generation 3
 *
 * Copyright (c) 2024 Qualcomm Technologies, Inc.
 */
#ifndef __QC_MSM_CAMSS_CSID_GEN3_H__
#define __QC_MSM_CAMSS_CSID_GEN3_H__

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

// Custom port_id values used in camss_reg_update to allow setting/clearing the RUP/AUP bits on the IPP path
// Start at 0x100 but since port_id=rdi_id in the current codebase any value out of the [0,5] range should be fine
#define CSID_780_CUSTOM_PORT_ID_IPP0 0x100

#endif /* __QC_MSM_CAMSS_CSID_GEN3_H__ */
