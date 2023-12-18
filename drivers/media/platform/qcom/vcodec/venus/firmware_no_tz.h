/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2017 Linaro Ltd.
 */
#ifndef __FIRMWARE_NO_TZ_H__
#define __FIRMWARE_NO_TZ_H__

struct device;

#define VENUS_PAS_ID	9

int init_fw_no_tz(struct venus_core *core);
void deinit_fw_no_tz(struct venus_core *core);
int auth_reset_fw_no_tz(struct venus_core *core, phys_addr_t mem_phys,
			size_t mem_size);
void unload_fw_no_tz(struct venus_core *core);
void set_hw_state_no_tz(struct venus_core *core, bool resume);

#endif
