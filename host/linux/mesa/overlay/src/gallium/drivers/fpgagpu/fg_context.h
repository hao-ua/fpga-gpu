/* SPDX-License-Identifier: MIT */
/*
 * fpgagpu context. A clear is held until the next flush or until something reads or writes one of its buffers, and
 * then sent as one submission: FRAME_BEGIN, VS_STATE (the target's size), FLUSH with the clear.
 */
#ifndef FG_CONTEXT_H
#define FG_CONTEXT_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pipe/p_context.h"
#include "pipe/p_state.h"

/* a message on stderr the first time an operation the driver does not carry out yet is asked for */
#define FG_NOTE_ONCE(what)                                                                 \
   do {                                                                                    \
      static bool fg_noted_;                                                               \
      if (!fg_noted_) {                                                                    \
         fg_noted_ = true;                                                                 \
         fprintf(stderr, "fpgagpu: %s is not implemented yet; ignored\n", what);           \
      }                                                                                    \
   } while (0)

/* the clear waiting to be sent: the FLUSH's fields */
struct fg_pending_clear {
   uint32_t flags;		/* FG_FL_* */
   uint32_t c16[4];		/* R, G, B, A in the target's channel units */
   uint32_t depth;		/* in the depth buffer's units */
   uint32_t stencil;
   uint32_t stencil_mask;
   uint32_t sc[4];		/* x0, y0, x1, y1 with FG_FL_CLEAR_SCISSOR */
   struct pipe_resource *cbuf;	/* referenced while the clear waits */
   struct pipe_resource *zsbuf;
};

struct fg_context {
   struct pipe_context base;

   struct pipe_framebuffer_state fb;
   const struct pipe_blend_state *blend;

   struct fg_pending_clear clear;

   bool submitted;		/* last_fence is valid */
   uint64_t last_fence;		/* the fence of this context's latest submission */
};

static inline struct fg_context *
fg_context(struct pipe_context *pctx)
{
   return (struct fg_context *)pctx;
}

struct pipe_context *fg_context_create(struct pipe_screen *pscreen, void *priv, unsigned flags);

/* sends the pending clear, if there is one */
void fg_context_flush_pending(struct fg_context *ctx);

/* whether the pending clear writes the resource */
bool fg_context_pending_uses(struct fg_context *ctx, struct pipe_resource *prsc);

/* records a submission's fence as the context's latest */
static inline void
fg_context_fenced(struct fg_context *ctx, uint64_t fence)
{
   ctx->submitted = true;
   ctx->last_fence = fence;
}

#endif /* FG_CONTEXT_H */
