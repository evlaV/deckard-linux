/* SPDX-License-Identifier: GPL-2.0 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM qcom_camss

#if !defined(QCOM_CAMSS_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define QCOM_CAMSS_TRACE_H

#include <linux/tracepoint.h>

TRACE_EVENT(csid_isr_wm_done,
	TP_PROTO(unsigned int arg1, unsigned int csid_id),
	TP_ARGS(arg1, csid_id),
	TP_STRUCT__entry(
		__field(unsigned int, arg1)
		__field(unsigned int, csid_id)
	),
	TP_fast_assign(
		__entry->arg1 = arg1;
		__entry->csid_id = csid_id;
	),
	TP_printk("arg1 = %u, csid_id = %u", __entry->arg1, __entry->csid_id)
);

TRACE_EVENT(csid_isr_wm_done_buffer_status,
	TP_PROTO(unsigned int buffer_status, unsigned int csid_id),
	TP_ARGS(buffer_status, csid_id),
	TP_STRUCT__entry(
		__field(unsigned int, buffer_status)
		__field(unsigned int, csid_id)
	),
	TP_fast_assign(
		__entry->buffer_status = buffer_status;
		__entry->csid_id = csid_id;
	),
	TP_printk("buffer_status = %u, csid_id = %u", __entry->buffer_status, __entry->csid_id)
);

TRACE_EVENT(csid_isr_sof,
	TP_PROTO(unsigned int arg1, unsigned int csid_id),
	TP_ARGS(arg1, csid_id),
	TP_STRUCT__entry(
		__field(unsigned int, arg1)
		__field(unsigned int, csid_id)
	),
	TP_fast_assign(
		__entry->arg1 = arg1;
		__entry->csid_id = csid_id;
	),
	TP_printk("arg1 = %u, csid_id = %u", __entry->arg1, __entry->csid_id)
);

TRACE_EVENT(csid_isr,
	TP_PROTO(unsigned int arg1, unsigned int csid_id),
	TP_ARGS(arg1, csid_id),
	TP_STRUCT__entry(
		__field(unsigned int, arg1)
		__field(unsigned int, csid_id)
	),
	TP_fast_assign(
		__entry->arg1 = arg1;
		__entry->csid_id = csid_id;
	),
	TP_printk("arg1 = %u, csid_id = %u", __entry->arg1, __entry->csid_id)
);

TRACE_EVENT(vfe_isr,
	TP_PROTO(unsigned int arg1),
	TP_ARGS(arg1),
	TP_STRUCT__entry(
		__field(unsigned int, arg1)
	),
	TP_fast_assign(
		__entry->arg1 = arg1;
	),
	TP_printk("arg1 = %u", __entry->arg1)
);

#endif /* QCOM_CAMSS_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/media/platform/qcom/camss
#define TRACE_INCLUDE_FILE trace
#include <trace/define_trace.h>
