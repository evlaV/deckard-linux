/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2017 Linaro Ltd.
 */
#ifndef __FIRMWARE_NO_TZ_H__
#define __FIRMWARE_NO_TZ_H__

struct device;

static inline bool use_tz(struct device *core_dev)
{
	struct device_node *np;

	np = of_get_child_by_name(core_dev->of_node, "video-firmware");
	if (!np)
		return true;

	return false;
}

int init_fw_no_tz(struct iris_core *core);
void deinit_fw_no_tz(struct iris_core *core);
int auth_reset_fw_no_tz(struct iris_core *core, phys_addr_t mem_phys,
			size_t mem_size);
void unload_fw_no_tz(struct iris_core *core);
int set_hw_state_no_tz(struct iris_core *core, bool resume);

#endif
