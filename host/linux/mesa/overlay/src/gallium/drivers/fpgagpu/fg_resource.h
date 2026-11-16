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

/* L8, A8, LA88, R8 and RG8 are stored as RGBA8 (L, L, L, 1), (0, 0, 0, A), (L, L, L, A), (R, 0, 0, 1), (R, G, 0, 1):
 * what the TMU reads is the format's */
enum { FG_EXPAND_NONE, FG_EXPAND_L, FG_EXPAND_A, FG_EXPAND_LA, FG_EXPAND_R, FG_EXPAND_RG };

/* the channels of an expanded format that rendering writes (Gallium's mask order: R 1, G 2, B 4, A 8); the others
 * keep the values the format reads as */
static inline unsigned
fg_format_wmask(const struct fg_format *f)
{
   switch (f->expand) {
   case FG_EXPAND_A: return 0x8;
   case FG_EXPAND_R: return 0x1;
   case FG_EXPAND_RG: return 0x3;
   default: return 0xF;
   }
}

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
   uint32_t sample_gem;		/* a sampled D24S8 image: an RGBA8 alias of its bytes, which the TMU reads */
   uint16_t sample_handle;	/* (the TMU has no D24S8 format); 0 none */
   uint32_t pitch;
   uint64_t bo_size;
   bool initialized;		/* a clear or a write has reached it */

   /* board B renders into a pitch that is a multiple of 32, the TMU reads tight rows: an image whose tight rows are
    * not that is rendered in a copy (shadow), copied back after each pass */
   struct pipe_resource *shadow;
   bool shadow_stale;		/* the image changed since the shadow was copied from it */

   /* the other way round: an image with padded rows (one board B renders into or shows) that is sampled is read from
    * a copy with tight rows, taken before a pass that samples it when the image changed. Another process may write a
    * shared image (imported or handed out): its copy is taken again in every pass that samples it */
   struct pipe_resource *tight;
   bool tight_stale;		/* the image changed since tight was copied from it */
   bool shared;
   uint32_t tight_seq;		/* the screen's pass count when tight was copied */

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
