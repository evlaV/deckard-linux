/* SPDX-License-Identifier: GPL-2.0 */
/*
 * <linux/gpuvis_helpers_userspace.h>
 *
 * These are ftrace helpers that can be used from userspace in the same
 * way as `gpuvis_helpers.h` can be.  Read `gpuvis_helpers_common.h` for
 * full usage information, this file contains the shims necessary to make
 * the common code work in userspace.
 */

#include <stdlib.h>
#include <fcntl.h>

// While not required, you may wish to run this at the start of your program to reduce
// first-trace latency where we have to open two files lazily.  It could increase the
// time of your first operation a bit.
int trace_fd = 0;
void init_trace_fd(void) {
	if (trace_fd == 0) {
		trace_fd = open("/sys/kernel/tracing/trace_marker", O_WRONLY);
	}
}

// Read `/sys/kernel/tracing/trace_ctx_id` to get a unique context ID for whatever event we're about to do.
uint64_t trace_ctx_id(void) {
	char ctx_buff[22];

	// Open and read the number, converting it to a uint64_t
	int ctx_fd = open("/sys/kernel/tracing/trace_ctx_id", O_RDONLY);
	read(ctx_fd, ctx_buff, 22);
	uint64_t val = strtoull(ctx_buff, NULL, 10);
	close(ctx_fd);
	return val;
}

// Our `trace_printk()` substitute is to write to `/sys/kernel/tracing/
void trace_printk(char * data) {
	init_trace_fd();
	write(trace_fd, data, strlen(data));
}

// Just always pretend we have tracing support.  Ideally, we'd have some way of checking to see
// if `/sys/kernel/tracing` is mounted or something, but that's way too fancy for right now.
#define CONFIG_TRACING
#include "gpuvis_helpers_common.h"