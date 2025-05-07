/* SPDX-License-Identifier: GPL-2.0 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM typec

#if !defined(__TYPEC_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define __TYPEC_TRACE_H

#include <linux/tracepoint.h>

TRACE_EVENT(typec_tcpm_event_handler,
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

TRACE_EVENT(typec_tcpm_log,
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

#endif /* __TYPEC_TRACE_H */

/* This part must be outside protection */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace

#include <trace/define_trace.h>
