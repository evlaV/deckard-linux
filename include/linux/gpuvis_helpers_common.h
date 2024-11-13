/* SPDX-License-Identifier: GPL-2.0 */
/*
 * <linux/gpuvis_helpers_common.h>
 *
 * These are ftrace helpers that generate ftrace `print` events that can
 * be consumed from `gpuvis` [0].  Typical usage looks like this:
 *
 *     void some_kernel_function()
 *     {
 *         TRACE_CTX();
 *         // do something...
 *     }
 *
 * This will automatically emit ftrace events to mark when this function
 * starts and finishes.  The full set of macros to be used are:
 *
 *    - TRACE_CTX(): Instantly mark function start, use `__attribute__((cleanup)`
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

// Lowest-level function that formats and prints.
// NOTE: As an implementation detail, the total string length is
// restricted to 192 characters.
static void inline trace_print_ctx(const char *name, const char *ctx_op,
                                   uint64_t ctx_id, const char *file, int line,
								   const char *payload_format, va_list args)
{
	char buff[192];

	// First, write the required header chunk
	int len = snprintf(buff, sizeof(buff), "[%s] %s=%llu ", name, ctx_op, ctx_id);

	if (file != NULL) {
		len += snprintf(buff + len, sizeof(buff)-len, "file=%s:%d ", file, line);
	}

	// Next, write the payload, if any
	if (payload_format != NULL) {
		vsnprintf(buff+len, sizeof(buff)-len, payload_format, args);
	}

	// Finally, `trace_printk()` the whole thing.
	trace_printk(buff);
}

// Helpers to absorb varargs and insert the appropriate `begin_ctx`/`end_ctx` strings
static void inline trace_begin_ctx(const char *name, uint64_t ctx_id,
                                   const char *file, int line, const char *format, ...)
{
	va_list args;
	va_start(args, format);
	trace_print_ctx(name, "begin_ctx", ctx_id, file, line, format, args);
}
static void inline trace_end_ctx(const char *name, uint64_t ctx_id,
                                 const char *file, int line, const char *format, ...)
{
	va_list args;
	va_start(args, format);
	trace_print_ctx(name, "end_ctx", ctx_id, file, line, format, args);
}


// __attribute__((cleanup)) support!  It's easiest with a context struct.
// Note that the automatic cleanup macros don't allow printf formatting.
struct trace_ctx_metadata_t {
	const char * name;
	uint64_t id;
};

static void inline make_trace_context_metadata(struct trace_ctx_metadata_t * ctx, const char * name, const char * file, int line)
{
	ctx->name = name;
	ctx->id = trace_ctx_id();
	trace_begin_ctx(ctx->name, ctx->id, file, line, "");
}

static void inline cleanup_trace_context_metadata(struct trace_ctx_metadata_t * ctx)
{
	trace_end_ctx(ctx->name, ctx->id, NULL, 0, "");
}


// These are the top-level macros that the user would actually use.
#define TRACE_BEGIN_CTX_PRINTF(fmt, ...) \
	uint64_t _ctx_id = trace_ctx_id(); \
	trace_begin_ctx(__func__, _ctx_id, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

#define TRACE_END_CTX_PRINTF(fmt, ...) \
	trace_end_ctx(__func__, _ctx_id, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

// A null payload format just means no payload at all.
#define TRACE_BEGIN_CTX() TRACE_BEGIN_CTX_PRINTF(NULL)
#define TRACE_END_CTX() TRACE_END_CTX_PRINTF(NULL)

#define TRACE_CTX() \
	struct trace_ctx_metadata_t __attribute__((cleanup(cleanup_trace_context_metadata))) _ctx; \
	make_trace_context_metadata(&_ctx, __func__, __FILE__, __LINE__)


#else // CONFIG_TRACING
#define TRACE_BEGIN_CTX_PRINTF(fmt, ...)
#define TRACE_END_CTX_PRINTF(fmt, ...)
#define TRACE_BEGIN_CTX()
#define TRACE_END_CTX()
#define TRACE_CTX()
#endif // CONFIG_TRACING