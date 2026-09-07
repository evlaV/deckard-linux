// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Enhanced Green AP (EGAP) support for ath12k.
 */

#include "core.h"
#include "debug.h"
#include "egap.h"

#define ATH12K_EGAP_PARAM_FIELDS 4
#define ATH12K_EGAP_PARAM_INIT {					\
	.enable = false,						\
	.flags = WMI_AP_PS_EGAP_F_ENABLE_PWRSAVE_BY_INACTIVITY,		\
	.inactivity_time = 1000,					\
	.wait_time = 1000,						\
}

static const struct ath12k_egap_cfg ath12k_egap_param_defaults = ATH12K_EGAP_PARAM_INIT;
static struct ath12k_egap_cfg ath12k_egap_param = ATH12K_EGAP_PARAM_INIT;
static DEFINE_MUTEX(ath12k_egap_param_lock);

int ath12k_egap_parse_param(const char *val)
{
	struct ath12k_egap_cfg cfg = ath12k_egap_param_defaults;
	char *dup, *p, *tok;
	unsigned int i = 0;
	int ret = 0;

	dup = kstrdup(val, GFP_KERNEL);
	if (!dup)
		return -ENOMEM;

	p = strim(dup);
	while ((tok = strsep(&p, ",")) != NULL) {
		if (i >= ATH12K_EGAP_PARAM_FIELDS) {
			ret = -EINVAL;
			break;
		}

		tok = strim(tok);
		if (*tok) {
			switch (i) {
			case 0:
				ret = kstrtobool(tok, &cfg.enable);
				break;
			case 1:
				ret = kstrtou32(tok, 0, &cfg.flags);
				break;
			case 2:
				ret = kstrtou32(tok, 0, &cfg.inactivity_time);
				break;
			case 3:
				ret = kstrtou32(tok, 0, &cfg.wait_time);
				break;
			}
			if (ret)
				break;
		}
		i++;
	}
	kfree(dup);

	if (ret)
		return ret;

	if (!ath12k_egap_time_valid(cfg.inactivity_time) ||
	    !ath12k_egap_time_valid(cfg.wait_time))
		return -EINVAL;

	if (cfg.enable && !cfg.flags)
		return -EINVAL;

	mutex_lock(&ath12k_egap_param_lock);
	ath12k_egap_param = cfg;
	mutex_unlock(&ath12k_egap_param_lock);

	return 0;
}

int ath12k_egap_format_param(char *buf, size_t size)
{
	struct ath12k_egap_cfg cfg;

	mutex_lock(&ath12k_egap_param_lock);
	cfg = ath12k_egap_param;
	mutex_unlock(&ath12k_egap_param_lock);

	return scnprintf(buf, size, "%u,0x%x,%u,%u\n", cfg.enable, cfg.flags,
			 cfg.inactivity_time, cfg.wait_time);
}

static bool ath12k_egap_fw_supported(struct ath12k *ar)
{
	return test_bit(WMI_TLV_SERVICE_EGAP, ar->ab->wmi_ab.svc_map);
}

static void ath12k_egap_cfg_from_params(struct ath12k_egap_cfg *cfg)
{
	mutex_lock(&ath12k_egap_param_lock);
	*cfg = ath12k_egap_param;
	mutex_unlock(&ath12k_egap_param_lock);
}

static int ath12k_egap_push(struct ath12k *ar, const struct ath12k_egap_cfg *cfg)
{
	struct ath12k_wmi_egap_params params = {
		.enable = cfg->enable,
		.inactivity_time = cfg->inactivity_time,
		.wait_time = cfg->wait_time,
		.flags = cfg->flags,
		/* firmware computes the reduced chainmask, the reference driver leaves this zero too */
		.reduced_txrx_chainmask = 0,
	};
	int ret;

	ret = ath12k_wmi_send_egap_conf_params_cmd(ar, &params);
	if (ret) {
		ath12k_warn(ar->ab, "failed to %s EGAP: %d\n",
			    cfg->enable ? "enable" : "disable", ret);
		return ret;
	}

	if (cfg->enable)
		ath12k_info(ar->ab,
			    "egap enabled inactivity %u ms wait %u ms flags 0x%x\n",
			    cfg->inactivity_time, cfg->wait_time, cfg->flags);
	else
		ath12k_dbg(ar->ab, ATH12K_DBG_MAC, "egap disabled\n");

	return 0;
}

void ath12k_egap_apply(struct ath12k *ar)
{
	struct ath12k_egap_cfg cfg;

	lockdep_assert_wiphy(ath12k_ar_to_hw(ar)->wiphy);

	if (!ath12k_egap_fw_supported(ar))
		return;

	ath12k_egap_cfg_from_params(&cfg);
	ath12k_egap_push(ar, &cfg);
}
