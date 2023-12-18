/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _FIRMWARE_H_
#define _FIRMWARE_H_

#include <linux/device.h>
#include <linux/types.h>

bool use_tz(struct device *core_dev);
int load_fw(struct device *dev, const char *fw_name, phys_addr_t *mem_phys,
	    size_t *mem_size, u32 pas_id, bool use_tz);
int auth_reset_fw(u32 pas_id);
int protect_secure_region(u32 cp_start, u32 cp_size, u32 cp_nonpixel_start,
			  u32 cp_nonpixel_size, u32 pas_id);
void unload_fw(u32 pas_id);
int set_hw_state(bool resume);

#endif
