/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note*/
/*
 * <uapi/linux/trace_gpuvis_userspace.h>
 *
 * Read trace_gpuvis.h for the documentation.
 * This file defines a bunch of implementations for functions that don't
 * exist in userspace, but we want to use the same definitions of what we
 * use in kernelspace, so we fill these in such that everything works from
 * userspace as well.
 */


#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

// Manually inline `<linux/stringify.h>`
#define __stringify_1(x...)	#x
#define __stringify(x...)	__stringify_1(x)
#define FILE_LINE	__FILE__ ":" __stringify(__LINE__)

// While not required, you may wish to run this at the start of your program to reduce
// first-trace latency where we have to open this file lazily; It could increase the
// time of your first operation a bit.
int trace_fd = 0;
void init_trace_fd(void) {
	if (trace_fd != 0) {
		return;
	}

    trace_fd = open("/sys/kernel/tracing/trace_marker", O_WRONLY);
}

// Convert trace clock names to the clock IDs we can actually use.
// Note that I've only ever tested `mono_raw` because that's all we use.
clockid_t word_to_clock_id(const char * word) {
    if (strcmp(word, "mono_raw") == 0) {
        return CLOCK_MONOTONIC_RAW;
    }
    if (strcmp(word, "mono") == 0) {
        return CLOCK_MONOTONIC;
    }
    if (strcmp(word, "boot") == 0) {
        return CLOCK_BOOTTIME;
    }
    if (strcmp(word, "tai") == 0) {
        return CLOCK_TAI;
    }

    // Otherwise, just return this:
    return CLOCK_MONOTONIC_RAW;
}

// Figure out which clock to use based on what the tracing system is using.
clockid_t clock_id = -1;
void get_trace_clock() {
    if (clock_id != -1) {
        return;
    }

    int clock_fd = open("/sys/kernel/tracing/trace_clock", O_RDONLY);
    char clock_buff[96];
    int nread = read(clock_fd, clock_buff, sizeof(clock_buff));
    if (nread < 0) {
        // default to this in the event of error
        clock_id = CLOCK_MONOTONIC_RAW;
        return;
    }
    clock_buff[nread] = '\0';

    char * word = &clock_buff[0];
    while (*word != '\0') {
        while (*word == ' ')
            word++;
        // This is the selected word, trim it then convert it:
        if (word[0] == '[') {
            int idx = 1;
            while (word[idx] != ']' && word[idx] != '\0')
                idx++;
            word[idx] = '\0';
            clock_id = word_to_clock_id(word+1);
            return;
        }
    }

    // If we couldn't find the word, just return this one
    clock_id = CLOCK_MONOTONIC_RAW;
}

// For userspace, we call `clock_gettime()` with the appropriate `clock_id`.
#define ktime_t uint64_t
static inline
ktime_t trace_clock_gettime_top()
{
    get_trace_clock();
    struct timespec ts;
    clock_gettime(clock_id, &ts);
    return ( ( uint64_t )ts.tv_sec * 1000000000LL) + ts.tv_nsec;
}

// Read `/sys/kernel/tracing/trace_ctx_id` to get a unique context ID for whatever event we're about to do.
// If we fail to do that, we'll just increment an ID locally.
uint64_t local_counter = 0;
uint64_t trace_ctx_id(void) {
	char ctx_buff[22];

	// Open and read the number, converting it to a uint64_t
	int ctx_fd = open("/sys/kernel/tracing/trace_ctx_id", O_RDONLY);
	int nread = read(ctx_fd, ctx_buff, 22);
    if (nread < 0)
        local_counter += 1;
    else
        local_counter = strtoull(ctx_buff, NULL, 10);
	close(ctx_fd);
	return local_counter;
}

// Write to `/sys/kernel/tracing/trace_marker` as the userspace implementation of tracing fprintf.
void trace_marker_printf(const char * format, ...) {
    va_list args;
    va_start(args, format);

    // Open `trace_fd`, if needed, then `vprintf()` straight to that fd.
	init_trace_fd();
#ifdef TRACE_STDOUT_OVERRIDE
    int ret = vdprintf(0, format, args);
#else
    int ret = vdprintf(trace_fd, format, args);
#endif

    // If we failed to write, try to reopen the fd for next time.
    if (ret < 0) {
        close(trace_fd);
        trace_fd = 0;
    }
    va_end(args);
}

// Userspace `TRACE_EVENT()` implementation that subs out to `trace_marker_printf()`.
#define TP_STRUCT__entry(args...) args
#define TP_ARGS(args...) args
#define TP_PROTO(args...) args
#define TP_fast_assign(args...) args
#define __field(type, item) type item;
#define __string(name, source) const char * name;
#define __assign_str_workaround(name, source) __entry->name = source
#define __get_str(name) __entry->name
#define tracepoint_string(arg)  arg
#define TP_printk(fmt, args...) fmt "\n", args
#define KERN_PREFIX   ""

#define TRACE_EVENT(event_name, proto, args, layout, assign_block, printf_args) \
struct entry_##event_name { \
    layout \
}; \
static inline void trace_##event_name(proto) \
{ \
    struct entry_##event_name _entry; \
    struct entry_##event_name *__entry = &_entry; \
    assign_block; \
    trace_marker_printf(printf_args); \
}
