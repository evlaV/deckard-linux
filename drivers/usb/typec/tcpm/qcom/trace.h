/* SPDX-License-Identifier: GPL-2.0 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM typec_qcom

#if !defined(__TYPEC_QCOM_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define __TYPEC_QCOM_TRACE_H

#include <linux/tracepoint.h>


TRACE_EVENT(typec_qcom_log,
	TP_PROTO(char *logstr),
	TP_ARGS(logstr),

	TP_STRUCT__entry(
		__string(name, logstr)
	),
	TP_fast_assign(
		__assign_str(name);
	),
	TP_printk("%s",
		__get_str(name)
	)
);

TRACE_EVENT(typec_qcom_pmic_probe,
	TP_PROTO(uint32_t arg),
	TP_ARGS(arg),
	TP_STRUCT__entry(
		__field(uint32_t, arg)
	),
	TP_fast_assign(
		__entry->arg = arg;
	),
	TP_printk("arg=%u",
		__entry->arg)
);

TRACE_EVENT(typec_qcom_port_debounce,
	TP_PROTO(uint32_t arg),
	TP_ARGS(arg),
	TP_STRUCT__entry(
		__field(uint32_t, arg)
		),
	TP_fast_assign(
		__entry->arg = arg;
	),
	TP_printk("arg=%u",
		__entry->arg
	)
);


TRACE_EVENT(typec_qcom_port_isr,
	TP_PROTO(char *eventname, int arg),
	TP_ARGS(eventname, arg),

	TP_STRUCT__entry(
		__string(name, eventname)
		__field(int, arg)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->arg = arg;
	),
	TP_printk("[%s], %d",
		__get_str(name),
		__entry->arg
	)
);

TRACE_EVENT(typec_qcom_port_get_vbus,
	TP_PROTO(uint32_t misc, uint32_t vbusdet),
	TP_ARGS(misc,vbusdet),
	TP_STRUCT__entry(
		__field(uint32_t, misc)
		__field(uint32_t, vbusdet)
	),
	TP_fast_assign(
		__entry->misc = misc;
		__entry->vbusdet = vbusdet;
	),
	TP_printk("misc=%u,vbusdet=%u",
		__entry->misc,
		__entry->vbusdet
	)
);

TRACE_EVENT(typec_qcom_port_is_vbus_vsafe0v,  
	TP_PROTO(uint32_t sensor_status, uint32_t safe),
	TP_ARGS(sensor_status,safe),
	TP_STRUCT__entry(
		__field(uint32_t, sensor_status)
		__field(uint32_t, safe)
	),
	TP_fast_assign(
		__entry->sensor_status = sensor_status;
		__entry->safe = safe;
	),
	TP_printk("sensor_status=%u,safe=%u",
		__entry->sensor_status,
		__entry->safe
	)
);

TRACE_EVENT(typec_qcom_port_set_vbus,  
	TP_PROTO(uint32_t vbus),
	TP_ARGS(vbus),
	TP_STRUCT__entry(
		__field(uint32_t, vbus)
	),
	TP_fast_assign(
		__entry->vbus = vbus;
	),
	TP_printk("vbus=%u",
		__entry->vbus
	)
);


TRACE_EVENT(typec_qcom_port_get_cc_src_status,
	TP_PROTO(char *status, int arg),
	TP_ARGS(status, arg),

	TP_STRUCT__entry(
		__string(name, status)
		__field(int, arg)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->arg = arg;
	),
	TP_printk("[%s], %d",
		__get_str(name),
		__entry->arg
	)
);

TRACE_EVENT(typec_qcom_port_get_cc_snk_status,
	TP_PROTO(char *status, int arg),
	TP_ARGS(status, arg),

	TP_STRUCT__entry(
		__string(name, status)
		__field(int, arg)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->arg = arg;
	),
	TP_printk("[%s], %d",
		__get_str(name),
		__entry->arg
	)
);

TRACE_EVENT(typec_qcom_port_get_cc_misc_status,
	TP_PROTO(int arg),
	TP_ARGS(arg),

	TP_STRUCT__entry(
		__field(int, arg)
	),
	TP_fast_assign(
		__entry->arg = arg;
	),
	TP_printk("[0x%02X]",
		__entry->arg
	)
);

TRACE_EVENT(typec_qcom_port_get_cc_misc_status_attached,
	TP_PROTO(int arg),
	TP_ARGS(arg),

	TP_STRUCT__entry(
		__field(int, arg)
	),
	TP_fast_assign(
		__entry->arg = arg;
	),
	TP_printk("%d",
		__entry->arg
	)
);

TRACE_EVENT(typec_qcom_port_get_cc_misc_status_sink_src_mode,
	TP_PROTO(int arg),
	TP_ARGS(arg),

	TP_STRUCT__entry(
		__field(int, arg)
	),
	TP_fast_assign(
		__entry->arg = arg;
	),
	TP_printk("%d",
		__entry->arg
	)
);

TRACE_EVENT(typec_qcom_port_get_legacy_cable_status,
	TP_PROTO(int arg),
	TP_ARGS(arg),

	TP_STRUCT__entry(
		__field(int, arg)
	),
	TP_fast_assign(
		__entry->arg = arg;
	),
	TP_printk("%d",
		__entry->arg
	)
);



TRACE_EVENT(typec_qcom_port_regmap_write,
	TP_PROTO(int debug, int base, int reg, int val),
	TP_ARGS(debug, base, reg, val),

	TP_STRUCT__entry(
		__field(int, debug)
		__field(int, base)
		__field(int, reg)
		__field(int, val)
	),
	TP_fast_assign(
		__entry->debug = debug;
		__entry->base = base;
		__entry->reg = reg;
		__entry->val = val;
	),
	TP_printk("[%d], 0x%02X+0x%02X = 0x%02X",
		__entry->debug,
		__entry->base,
		__entry->reg,
		__entry->val
	)
);



TRACE_EVENT(typec_qcom_port_regmap_bulk_write,
	TP_PROTO(int debug, int base, int reg, int val, int len),
	TP_ARGS(debug, base, reg, val, len),

	TP_STRUCT__entry(
		__field(int, debug)
		__field(int, base)
		__field(int, reg)
		__field(int, val)
		__field(int, len)
	),
	TP_fast_assign(
		__entry->debug = debug;
		__entry->base = base;
		__entry->reg = reg;
		__entry->val = val;
		__entry->len = len;
	),
	TP_printk("[%d], 0x%02X+0x%02X = 0x%02X, len=%d",
		__entry->debug,
		__entry->base,
		__entry->reg,
		__entry->val,
		__entry->len
	)
);


TRACE_EVENT(typec_qcom_port_regmap_bulk_read,
	TP_PROTO(int debug, int base, int reg, int val, int len),
	TP_ARGS(debug, base, reg, val, len),

	TP_STRUCT__entry(
		__field(int, debug)
		__field(int, base)
		__field(int, reg)
		__field(int, val)
		__field(int, len)
	),
	TP_fast_assign(
		__entry->debug = debug;
		__entry->base = base;
		__entry->reg = reg;
		__entry->val = val;
		__entry->len = len;
	),
	TP_printk("[%d], 0x%02X+0x%02X = 0x%02X, len=%d",
		__entry->debug,
		__entry->base,
		__entry->reg,
		__entry->val,
		__entry->len
	)
);


TRACE_EVENT(typec_qcom_port_regmap_update_bits,
	TP_PROTO(int debug, int base, int reg, int mask, int val),
	TP_ARGS(debug, base, reg, mask, val),

	TP_STRUCT__entry(
		__field(int, debug)
		__field(int, base)
		__field(int, reg)
		__field(int, mask)
		__field(int, val)
	),
	TP_fast_assign(
		__entry->debug = debug;
		__entry->base = base;
		__entry->reg = reg;
		__entry->mask = mask;
		__entry->val = val;
	),
	TP_printk("[%d], 0x%02X+0x%02X mask=(0x%02X) = 0x%02X",
		__entry->debug,
		__entry->base,
		__entry->reg,
		__entry->mask,
		__entry->val
	)
);


TRACE_EVENT(typec_qcom_port_regmap_read,
	TP_PROTO(int debug, int base, int reg, int val),
	TP_ARGS(debug, base, reg, val),

	TP_STRUCT__entry(
		__field(int, debug)
		__field(int, base)
		__field(int, reg)
		__field(int, val)
	),
	TP_fast_assign(
		__entry->debug = debug;
		__entry->base = base;
		__entry->reg = reg;
		__entry->val = val;
	),
	TP_printk("[%d], 0x%02X+0x%02X = 0x%02X",
		__entry->debug,
		__entry->base,
		__entry->reg,
		__entry->val
	)
);


TRACE_EVENT(typec_qcom_port_regmap_read_poll_timeout,
	TP_PROTO(int debug, int base, int reg, int val, int cond),
	TP_ARGS(debug, base, reg, val, cond),

	TP_STRUCT__entry(
		__field(int, debug)
		__field(int, base)
		__field(int, reg)
		__field(int, val)
		__field(int, cond)
	),
	TP_fast_assign(
		__entry->debug = debug;
		__entry->base = base;
		__entry->reg = reg;
		__entry->val = val;
		__entry->cond = cond;
	),
	TP_printk("[%d], 0x%02X+0x%02X = 0x%02X == 0x%02X ?",
		__entry->debug,
		__entry->base,
		__entry->reg,
		__entry->val,
		__entry->cond
	)
);

TRACE_EVENT(typec_qcom_pdphy_isr,
	TP_PROTO(char *eventname, int arg),
	TP_ARGS(eventname, arg),

	TP_STRUCT__entry(
		__string(name, eventname)
		__field(int, arg)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->arg = arg;
	),
	TP_printk("[%s], %d",
		__get_str(name),
		__entry->arg
	)
);

__printf(1, 2)
void trace_typec_log(const char *fmt, ...);

#endif /* __TYPEC_QCOM_TRACE_H */

/* This part must be outside protection */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace

#include <trace/define_trace.h>
