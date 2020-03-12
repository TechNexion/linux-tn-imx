/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM pcidwc

#if !defined(_TRACE_PCIDWC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_PCIDWC_H

#include <linux/tracepoint.h>

TRACE_EVENT(dbi_write,

	TP_PROTO(unsigned reg, unsigned size, unsigned val),

	TP_ARGS(reg, size, val),

	TP_STRUCT__entry(
		__field(unsigned, reg)
		__field(unsigned, size)
		__field(unsigned, val)
	),

	TP_fast_assign(
		__entry->reg = reg;
		__entry->size = size;
		__entry->val = val;
	),

	TP_printk("0x%x %d 0x%x", __entry->reg, __entry->size, __entry->val)
);

TRACE_EVENT(dbi_read,

	TP_PROTO(unsigned reg, unsigned size, unsigned val),

	TP_ARGS(reg, size, val),

	TP_STRUCT__entry(
		__field(unsigned, reg)
		__field(unsigned, size)
		__field(unsigned, val)
	),

	TP_fast_assign(
		__entry->reg = reg;
		__entry->size = size;
		__entry->val = val;
	),

	TP_printk("0x%x %d 0x%x", __entry->reg, __entry->size, __entry->val)
);

#endif /* if !defined(_TRACE_PCIDWC_H) || defined(TRACE_HEADER_MULTI_READ) */

/* This part must be outside protection */
#include <trace/define_trace.h>
