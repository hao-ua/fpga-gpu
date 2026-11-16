/* SPDX-License-Identifier: MIT */
/*
 * fpgagpu resources: textures, render targets and depth-stencil buffers are images in board B's memory (BO_CREATE
 * of an FPGAGPU_BO_B_IMAGE); PIPE_BUFFER resources live in host memory.
 */
#ifndef FG_RESOURCE_H
#define FG_RESOURCE_H

#include <stdbool.h>
#include <stdint.h>

#include "pipe/p_state.h"

/* how a pipe format is stored on board B and moved to and from it */
struct fg_format {
   enum pipe_format pformat;
   uint8_t storage;		/* FG_FMT_* */
   uint8_t tformat;		/* FG_TF_*: the transfer format whose bytes are the pipe format's */
   uint8_t bpp;			/* bytes per pixel, in host memory */
   uint8_t bits[4];		/* R, G, B, A widths of the stored colour; 0 = no such channel */
   bool force_alpha;		/* an X channel: the stored alpha is kept at 1.0 */
   bool depth;
   uint8_t expand;		/* FG_EXPAND_*: one board B has no storage for, kept as RGBA8 and converted on the PC */
};

/* L8, A8 and LA88 are stored as RGBA8 (L, L, L, 1), (0, 0, 0, A), (L, L, L, A): what the TMU reads is the format's */
enum { FG_EXPAND_NONE, FG_EXPAND_L, FG_EXPAND_A, FG_EXPAND_LA };

/* bytes per pixel in board memory */
static inline unsigned
fg_storage_bpp(const struct fg_format *f)
{
   return f->expand ? 4 : f->bpp;
}

/* whether board B can render into it */
bool fg_format_renderable(const struct fg_format *f);

struct fg_resource {
   struct pipe_resource base;

   /* images */
   const struct fg_format *fmt;
   uint32_t gem;		/* GEM handle; 0 for host memory */
   uint16_t board_handle;	/* the handle commands name the image by */
   uint32_t pitch;
   uint64_t bo_size;
   bool initialized;		/* a clear or a write has reached it */

   /* PIPE_BUFFER */
   void *data;
};

static inline struct fg_resource *
fg_resource(struct pipe_resource *prsc)
{
   return (struct fg_resource *)prsc;
}

struct fg_transfer {
   struct pipe_transfer base;
   void *staging;
};

const struct fg_format *fg_format_info(enum pipe_format format);
bool fg_vertex_format_supported(enum pipe_format format);

void fg_resource_screen_init(struct pipe_screen *pscreen);
void fg_resource_context_init(struct pipe_context *pctx);

#endif /* FG_RESOURCE_H */
