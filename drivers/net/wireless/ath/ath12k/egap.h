/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Enhanced Green AP (EGAP) support for ath12k.
 */

#ifndef ATH12K_EGAP_H
#define ATH12K_EGAP_H

struct ath12k;

#define ATH12K_EGAP_TIME_MIN_MS 1
#define ATH12K_EGAP_TIME_MAX_MS 300000

static inline bool ath12k_egap_time_valid(u32 ms)
{
	return ms >= ATH12K_EGAP_TIME_MIN_MS && ms <= ATH12K_EGAP_TIME_MAX_MS;
}

struct ath12k_egap_cfg {
	bool enable;
	/* ms without traffic from any associated station */
	u32 inactivity_time;
	/* ms without traffic after all stations entered power save */
	u32 wait_time;
	/* enum wmi_ap_ps_egap_flag_type bitmap */
	u32 flags;
};

/* The "egap" module parameter: "<enable>[,<flags>[,<inactivity_ms>[,<wait_ms>]]]" */
int ath12k_egap_parse_param(const char *val);
int ath12k_egap_format_param(char *buf, size_t size);

/* push the parameter to firmware (AP vdev up, parameter write) */
void ath12k_egap_apply(struct ath12k *ar);

#endif /* ATH12K_EGAP_H */
