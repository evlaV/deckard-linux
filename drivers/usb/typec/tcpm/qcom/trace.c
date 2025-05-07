// SPDX-License-Identifier: GPL-2.0
#define CREATE_TRACE_POINTS
#include "trace.h"

__printf(1, 2)
void trace_typec_log(const char *fmt, ...)
{
	char tmpbuffer[128];
	va_list args;
	va_start(args, fmt);
	vsnprintf(tmpbuffer, sizeof(tmpbuffer), fmt, args);
	va_end(args);
	trace_printk(tmpbuffer);
}
