/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note*/
/*
 * <uapi/linux/trace_gpuvis_context.h>
 *
 * Read trace_gpuvis.h for the documentation.
 */

#ifndef UAPI_TRACE_GPUVIS_CONTEXT_H
#define UAPI_TRACE_GPUVIS_CONTEXT_H

struct gpuvis_trace_context_t {
    // Automatically-collected information
    pid_t tgid;
    uint64_t id;
    const char * func;
    const char * sub_func;
    const char * file_line;
    ktime_t time;

    // Optional, user-provided information
    const char * payload;

    // State, tracks things like whether this context
    // has already reported.
    uint8_t flags;

    struct gpuvis_trace_context_t * end;
};

#define GPUVIS_TRACE_REPORTED   0x01

// These flags are set via sentinel flags at the beginning of the format string
#define GPUVIS_BASIC_STYLE      0x02
#define TRACE_BASIC             "\x02"

#define GPUVIS_UNCONDITIONAL    0x04
#define TRACE_UNCONDITIONAL     "\x04"


static inline
bool gpuvis_trace_reported(struct gpuvis_trace_context_t *ctx)
{
    return (ctx->flags & GPUVIS_TRACE_REPORTED) != 0;
}

static inline
bool gpuvis_trace_unconditional(struct gpuvis_trace_context_t *ctx)
{
    return (ctx->flags & GPUVIS_UNCONDITIONAL) != 0;
}

static inline
bool gpuvs_basic_style(struct gpuvis_trace_context_t *ctx)
{
    return (ctx->flags & GPUVIS_BASIC_STYLE) != 0;
}

static inline
void gpuvis_trace_set_reported(struct gpuvis_trace_context_t *ctx)
{
    ctx->flags |= GPUVIS_TRACE_REPORTED;
}

static inline
void gpuvis_trace_set_basic_style(struct gpuvis_trace_context_t *ctx)
{
    ctx->flags |= GPUVIS_BASIC_STYLE;
}

static inline
void gpuvis_apply_flags(struct gpuvis_trace_context_t *ctx)
{
    if ((ctx->flags & GPUVIS_BASIC_STYLE) == GPUVIS_BASIC_STYLE)
    {
        ctx->file_line = tracepoint_string("");
    }
}

// Helper functions to create a purposefully "uninitialized" `ctx`.
// This is used to tell if an `end_ctx` has actually been used or not.
static inline
void gpuvis_trace_uninit(struct gpuvis_trace_context_t *ctx)
{
    ctx->func = (const char *)ctx;
    ctx->end = NULL;
}

static inline
bool gpuvis_trace_is_uninit(struct gpuvis_trace_context_t *ctx)
{
    return ctx->func == (const char *)ctx && ctx->end == NULL;
}

#endif // UAPI_TRACE_GPUVIS_CONTEXT_H
