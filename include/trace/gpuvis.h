// Include all the kernel things we need
#include <linux/tracepoint.h>
#include <linux/stringify.h>
#include <linux/types.h>

// This is needed for `trace_clock_gettime_top()`.
#include "../../kernel/trace/trace.h"

// NOTE: We do this to better conform to what gpuvis expects
#undef TRACE_SYSTEM
#define TRACE_SYSTEM gpuvis

// Denote that these events are coming from the kernel
#define KERN_PREFIX   "[kern] "
#define PRIu64 "llu"
#define PRId64 "lld"

// Workaround for https://github.com/torvalds/linux/commit/2c92ca849fcc6ee7d0c358e9959abc9f58661aea
// which makes our lives much harder when trying to replicate this in userspace.
#define __assign_str_workaround(name, source) __assign_str(name)

// Include the meat of the implementation, which is stored in `uapi`
#include <uapi/linux/trace_gpuvis.h>

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE gpuvis
#include <trace/define_trace.h>