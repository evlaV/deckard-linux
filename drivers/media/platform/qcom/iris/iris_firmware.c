// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/firmware.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/soc/qcom/mdt_loader.h>

#include "iris_core.h"
#include "iris_firmware.h"
#include "firmware_no_tz.h"

#define MAX_FIRMWARE_NAME_SIZE	128

static int iris_protect_cp_mem(struct iris_core *core)
{
	struct tz_cp_config *cp_config;
	int ret;

	cp_config = core->iris_platform_data->tz_cp_config_data;

	ret = qcom_scm_mem_protect_video_var(cp_config->cp_start,
					     cp_config->cp_size,
					     cp_config->cp_nonpixel_start,
					     cp_config->cp_nonpixel_size);
	if (ret)
		dev_err(core->dev, "failed to protect memory(%d)\n", ret);

	return ret;
}

static int iris_load_fw_to_memory(struct iris_core *core, const char *fw_name)
{
	const struct firmware *firmware = NULL;
	struct device_node *node = NULL;
	struct reserved_mem *rmem;
	phys_addr_t mem_phys = 0;
	void *mem_virt = NULL;
	size_t res_size = 0;
	ssize_t fw_size = 0;
	struct device *dev;
	int pas_id = 0;
	int ret;

	if (!fw_name || !(*fw_name) || !core)
		return -EINVAL;

	dev = core->dev;

	if (strlen(fw_name) >= MAX_FIRMWARE_NAME_SIZE - 4)
		return -EINVAL;

	pas_id = core->iris_platform_data->pas_id;

	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!node)
		return -EINVAL;

	rmem = of_reserved_mem_lookup(node);
	if (!rmem) {
		ret = -EINVAL;
		goto err_put_node;
	}

	mem_phys = rmem->base;
	res_size = rmem->size;

	ret = request_firmware(&firmware, fw_name, dev);
	if (ret) {
		dev_err(core->dev, "failed to request fw \"%s\", error %d\n",
			fw_name, ret);
		goto err_put_node;
	}

	fw_size = qcom_mdt_get_size(firmware);
	if (fw_size < 0 || res_size < (size_t)fw_size) {
		ret = -EINVAL;
		dev_err(core->dev, "out of bound fw image fw size: %ld, res_size: %lu\n",
			fw_size, res_size);
		goto err_release_fw;
	}

	mem_virt = memremap(mem_phys, res_size, MEMREMAP_WC);
	if (!mem_virt) {
		dev_err(core->dev, "failed to remap fw memory phys %pa[p]\n",
			&mem_phys);
		goto err_release_fw;
	}

	if (core->use_tz) {
		ret = qcom_mdt_load(dev, firmware, fw_name,
				    pas_id, mem_virt, mem_phys, res_size, NULL);
		if (ret) {
			dev_err(core->dev, "error %d loading fw \"%s\"\n",
				ret, fw_name);
			goto err_mem_unmap;
		}
		ret = qcom_scm_pas_auth_and_reset(pas_id);
		if (ret) {
			dev_err(core->dev, "error %d authenticating fw \"%s\"\n",
				ret, fw_name);
			goto err_mem_unmap;
		}
	} else {
		ret = qcom_mdt_load_no_init(dev, firmware, fw_name,
				    pas_id, mem_virt, mem_phys, res_size, NULL);
		if (ret) {
			dev_err(core->dev, "error %d loading fw \"%s\"\n",
				ret, fw_name);
			goto err_mem_unmap;
		}

		ret = auth_reset_fw_no_tz(core, mem_phys, res_size);
		if (ret) {
			dev_err(core->dev, "error %d authenticating fw \"%s\"\n",
				ret, fw_name);
			goto err_mem_unmap;
		}
	}

	return ret;

err_mem_unmap:
	memunmap(mem_virt);
err_release_fw:
	release_firmware(firmware);
err_put_node:
	of_node_put(node);
	return ret;
}

int iris_fw_load(struct iris_core *core)
{
	int ret;

	core->use_tz = use_tz(core->dev);
	if (!core->use_tz) {
		ret = init_fw_no_tz(core);
		if (ret)
			return ret;
	}

	ret = iris_load_fw_to_memory(core, core->iris_platform_data->fwname);
	if (ret) {
		dev_err(core->dev, "firmware download failed\n");
		return -ENOMEM;
	}

	if (core->use_tz) {
		ret = iris_protect_cp_mem(core);
		if (ret) {
			dev_err(core->dev, "protect memory failed\n");
			qcom_scm_pas_shutdown(core->iris_platform_data->pas_id);
			return ret;
		}
	}

	return ret;
}

int iris_fw_unload(struct iris_core *core)
{
	int ret;

	if (!core->use_tz) {
		unload_fw_no_tz(core);
		return 0;
	}

	ret = qcom_scm_pas_shutdown(core->iris_platform_data->pas_id);
	if (ret)
		dev_err(core->dev, "firmware unload failed with ret %d\n", ret);

	return ret;
}

int iris_set_hw_state(struct iris_core *core, bool resume)
{
	if (core->use_tz)
		return qcom_scm_set_remote_state(resume, 0);
	else
		return set_hw_state_no_tz(core, resume);
}
