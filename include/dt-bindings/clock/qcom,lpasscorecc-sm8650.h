/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#ifndef _DT_BINDINGS_CLK_QCOM_LPASS_CORE_CC_SM8650_H
#define _DT_BINDINGS_CLK_QCOM_LPASS_CORE_CC_SM8650_H

/* QDSP6SS Core Clock Controller */
#define LPASS_QDSP6SS_CORE_CC_CORE_CLK_SRC		0
#define LPASS_QDSP6SS_CORE_CC_CORE_CLK			1

/* LPASS Core Clock Controller */
#define LPASS_CORE_CC_CORE_CLK_SRC				0

/* LPASS Core power domains */
#define LPASS_CORE_HM_GDSC				0

/* LPASS AON clocks */
#define LPASS_AON_CC_AT_CLK				0
#define LPASS_AON_CC_PCLKDBG_CLK			1
#define LPASS_AON_CC_LPI_NOC_HS_CLK			2
#define LPASS_AON_CC_Q6_AXIM_CLK			3

#endif
