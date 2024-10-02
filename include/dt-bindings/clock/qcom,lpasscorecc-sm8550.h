/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#ifndef _DT_BINDINGS_CLK_QCOM_LPASS_CORE_CC_SC7180_H
#define _DT_BINDINGS_CLK_QCOM_LPASS_CORE_CC_SC7180_H

/* LPASS_CORE_CC clocks */
#define LPASS_CORE_CC_CORE_CLK_SRC		0
#define LPASS_CORE_CC_SYSNOC_MPORT_CORE_CLK	1
#define LPASS_CORE_CC_EXT_IF2_CLK_SRC		2
#define LPASS_CORE_CC_EXT_IF2_IBIT_CLK		3

/* LPASS_AUDIO_CC */
#define LPASS_AUDIO_CC_WSA_MCLK_CLK_SRC 	0
#define LPASS_AUDIO_CC_WSA_MCLK_CLK 		1
#define LPASS_AUDIO_CC_WSA_2X_MCLK_CLK 		2

/* LPASS Core power domains */
#define LPASS_CORE_HM_GDSCR				0

/* LPASS Audio power domains */
#define LPASS_AUDIO_HM_GDSCR				0
#define LPASS_PDC_HM_GDSCR				1

#define LPASS_AON_CC_TX_MCLK_CLK			0
#define LPASS_AON_CC_LPI_NOC_HS_CLK			1
#define LPASS_AON_CC_Q6_AXIM_CLK			2


#endif
