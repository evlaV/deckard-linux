/* SPDX-License-Identifier: GPL-2.0 */
/*
 * <linux/gpuvis_helpers_common.h>
 *
 * These are ftrace helpers that generate ftrace `print` events that can
 * be consumed from `gpuvis` [0].  Typical usage looks like this:
 *
 *     void some_kernel_function()
 *     {
 *         TRACE_CTX("mystuff");
 *         // do something...
 *     }
 *
 * This will automatically emit ftrace events to mark when this function
 * starts and finishes.  The full set of macros to be used are:
 *
 *    - TRACE_CTX("mystuff"): Instantly mark function start, use `__attribute__((cleanup)`
 *      compiler support to mark function end.  The one downside is that it does
 *      not know the `__FILE__` and `__LINE__` of the function end code path, so
 *      if reporting that is important to you, use the next two macros.
 *
 *    - TRACE_BEGIN_CTX()/TRACE_END_CTX(): Allows manual specification of the
 *      begin/end points of the function, which also allows for automatic file
 *      and line reporting for both points.  A little bit more typing, and you
 *      have to be careful to not miss an exit point (otherwise gpuvis will make
 *      mistakes on matching up begin/end event pairs) but super helpful when
 *      debugging complex functions with multiple exit points.
 *
 *    - TRACE_BEGIN_CTX_PRINTF()/TRACE_END_CTX_PRINTF(): Allows specification of
 *      a printf format string to report local variable values while tracing.
 *      Helpful when debugging strange occurences and locals are important.
 *      Note that you can freely mix and match printf macros with non-printf
 *      macros, they just add an extra payload to a trace.  Example usage:
 *
 *    void gooblebox(const char * flooble, unsigned int num_cranks)
 *    {
 *        TRACE_BEGIN_CTX_PRINTF("flooble=%s num_cranks=%d", flooble, num_cranks);
 *        ...
 *        num_cranks = alter_num_cranks(flooble, num_cranks);
 *        if (num_cranks < MIN_NUM_CRANKS) {
 *            TRACE_END_CTX_PRINTF("num_cranks=%d", num_cranks);
 *            return;
 *        }
 *        ...
 *        TRACE_END_CTX();
 *    }
 *
 * These can be used from either the kernel or userspace; to use from the kernel,
 * simply add `#include <linux/gpuvis_helpers.h>` to your kernel module.  To use
 * from userspace, add `#include <linux/gpuvis_helpers_userspace.h>`.
 *
 * [0] https://github.com/mikesart/gpuvis
 */

// If we don't have `CONFIG_TRACING` enabled, just null these all out.
#ifdef CONFIG_TRACING

#include <linux/ktime.h>

// Lowest-level function that formats and prints.
// NOTE: As an implementation detail, the total string length is
// restricted to 192 characters.
// quick search of files in the linux kernel shows the longest file name length of 104 characters
static void inline trace_print_ctx(const char *name, const char *ctx_op,
                                   uint64_t ctx_id, int64_t dtime, const char *file, int line,
								   const char *payload_format, va_list args)
{
	char buff[192];

	// First, write the required header chunk
	int len = snprintf(buff, sizeof(buff), "[knl:%s] %s=%llu ", name, ctx_op, ctx_id);

	if (file != NULL) {
		len += snprintf(buff + len, sizeof(buff)-len, "file=%s:%d ", file, line);
	}

	if (dtime != 0) {
		//delta time -- interpreted by gpuvis
		len += snprintf(buff + len, sizeof(buff)-len, "dtime=%lld ", dtime); //can we use PRId64 instead?
	}

	// Next, write the payload, if any
	if (payload_format != NULL) {
		vsnprintf(buff+len, sizeof(buff)-len, payload_format, args);
	}

	// Finally, `trace_printk()` the whole thing.
	trace_printk(buff);
}

// Helpers to absorb varargs and insert the appropriate `begin_ctx`/`end_ctx` strings
static void inline trace_begin_ctx(const char *name, uint64_t ctx_id, int64_t dtime,
                                   const char *file, int line, const char *format, ...)
{
	va_list args;
	va_start(args, format);
	trace_print_ctx(name, "begin_ctx", ctx_id, dtime, file, line, format, args);
}

static void inline trace_end_ctx(const char *name, uint64_t ctx_id, int64_t dtime,
                                 const char *file, int line, const char *format, ...)
{
	va_list args;
	va_start(args, format);
	if( dtime != 0) 
	{
		trace_print_ctx(name, "begin_ctx", ctx_id, dtime, file, line, format, args);
	}
	trace_print_ctx(name, "end_ctx", ctx_id, 0, file, line, format, args);
}

// __attribute__((cleanup)) support!  It's easiest with a context struct.
struct trace_ctx_metadata_t {
    uint64_t id;
	
	// Automatically collected information
    const char *name;
    const char *file;
    unsigned int line;

    // If context end happens within this time threshold, squelch it completely.
    ktime_t starttime;
    uint64_t time_threshold_us;

	// User provided string to help differentiate this context -- will be sprintf'd into upon context creation and remain static
	char userstr[96];
};

static void inline make_trace_context_metadata(struct trace_ctx_metadata_t * ctx, ktime_t starttime, uint64_t time_threshold_us, const char * name, const char * file, int line, const char *format, ...)
{
	va_list args;
	va_start(args, format);

	ctx->id = trace_ctx_id();

	ctx->name = name;
	ctx->file = file;
	ctx->line = line;

	ctx->starttime = starttime;
	ctx->time_threshold_us = time_threshold_us;

	if (format != NULL) {
		vsnprintf(ctx->userstr, sizeof(ctx->userstr), format, args);
	}

	if(time_threshold_us == 0) {
		//if doing a normal context, just print the begin ctx now
		trace_begin_ctx(ctx->name, ctx->id, 0, file, line, format, args);
	}
}

static void inline cleanup_trace_context_metadata(struct trace_ctx_metadata_t * ctx)
{
	int64_t delta_time = (int64_t) (ktime_get_raw() - ctx->starttime);
	if( ctx->time_threshold_us != 0 ) {
		
		if( delta_time > ctx->time_threshold_us*1000 )
		{
			trace_begin_ctx(ctx->name, ctx->id, -delta_time, ctx->file, ctx->line, ctx->userstr ); //put the begin marker in timeline in the past (negative dtime) so it shows up correctly
			trace_end_ctx(ctx->name,   ctx->id,           0, ctx->file, ctx->line, ctx->userstr ); 
		}
	}
	else
	{
		trace_end_ctx(ctx->name, ctx->id, 0, NULL, 0, ctx->userstr);
	}
}

// These are the top-level macros that the user would actually use.
#define TRACE_BEGIN_CTX_PRINTF(fmt, ...) \
	uint64_t _ctx_id = trace_ctx_id(); \
	trace_begin_ctx(__func__, _ctx_id, 0, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

#define TRACE_END_CTX_PRINTF(fmt, ...) \
	trace_end_ctx(__func__, _ctx_id, 0, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

// A null payload format just means no payload at all.
#define TRACE_BEGIN_CTX() TRACE_BEGIN_CTX_PRINTF(NULL)
#define TRACE_END_CTX() TRACE_END_CTX_PRINTF(NULL)

#define TRACE_CTX(fmt, ...) \
	struct trace_ctx_metadata_t __attribute__((cleanup(cleanup_trace_context_metadata))) _ctx; \
	make_trace_context_metadata(&_ctx, ktime_get_raw(), 0, __func__, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

#define TRACE_CTX_TIMED(time_threshold_us, fmt, ...) \
	struct trace_ctx_metadata_t __attribute__((cleanup(cleanup_trace_context_metadata))) _ctx; \
	make_trace_context_metadata(&_ctx, ktime_get_raw(), time_threshold_us, __func__, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

#else // CONFIG_TRACING
#define TRACE_BEGIN_CTX_PRINTF(fmt, ...)
#define TRACE_END_CTX_PRINTF(fmt, ...)
#define TRACE_BEGIN_CTX()
#define TRACE_END_CTX()
#define TRACE_CTX(fmt, ...)
#define TRACE_CTX_TIMED(time_threshold_us, fmt, ...)
#endif // CONFIG_TRACING