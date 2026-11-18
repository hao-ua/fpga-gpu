/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026 Dmytro Ivanysh */
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
 * what the TMU reads is the format's. Z16 is stored as D24S8 (board B loads and stores depth as D24S8 words only; a
 * FLUSH with DEPTH16 keeps the depth there as 0 .. 65535) */
enum { FG_EXPAND_NONE, FG_EXPAND_L, FG_EXPAND_A, FG_EXPAND_LA, FG_EXPAND_R, FG_EXPAND_RG, FG_EXPAND_Z16 };

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

   /* board B renders into level 0 of a one-layer image whose pitch is a multiple of 32, the TMU reads tight rows: a
    * target it cannot render in place (rows that are not such a pitch, a level above 0, a layer or cube face) is
    * rendered in a one-level image of that level's size (shadow[level]), copied back after each pass.
    * shadow_layer[level]: the layer the shadow holds a copy of; -1 none (or the image changed since) */
#define FG_MAX_LEVELS 12
   struct pipe_resource *shadow[FG_MAX_LEVELS];
   int shadow_layer[FG_MAX_LEVELS];

   /* the other way round: an image with padded rows (one board B renders into or shows) that is sampled is read from
    * a copy with tight rows, taken before a pass that samples it when the image changed. Another process may write a
    * shared image (imported or handed out): its copy is taken again in every pass that samples it */
   struct pipe_resource *tight;
   bool tight_stale;		/* the image changed since tight was copied from it */

   /* the TMU starts at an image's level 0 and reads its chain to the end: a view of levels lview_first .. + lview_n - 1
    * (GL's base and max level, a blit from a level) reads a copy of those levels of every layer, taken again when the
    * image changed */
   struct pipe_resource *lview;
   unsigned lview_first, lview_n;
   bool lview_stale;
   uint32_t lview_seq;
   bool shared;
   uint32_t tight_seq;		/* the screen's pass count when tight was copied */

   /* rectangles of it that region passes rendered into (aliases at an offset), the last few kept */
   struct fg_rect_alias {
      uint32_t gem;		/* 0: free */
      uint16_t handle;
      uint32_t offset;
      uint16_t width, height;
   } ralias[8];
   unsigned ralias_next;

   /* PIPE_BUFFER */
   void *data;
};

static inline struct fg_resource *
fg_resource(struct pipe_resource *prsc)
{
   return (struct fg_resource *)prsc;
}

/* the image changed: the copies it is sampled through (tight rows, levels) are to be taken again */
static inline void
fg_resource_written(struct fg_resource *rsc)
{
   rsc->tight_stale = true;
   rsc->lview_stale = true;
}

/* the image changed other than through a shadow: every shadow is to take it again */
static inline void
fg_shadows_stale(struct fg_resource *rsc)
{
   for (unsigned l = 0; l < FG_MAX_LEVELS; l++)
      rsc->shadow_layer[l] = -1;
}

struct fg_transfer {
   struct pipe_transfer base;
   void *staging;
};

const struct fg_format *fg_format_info(enum pipe_format format);

/* the board handle of the rectangle (x, y, width, height) of a single-level image, an alias of its memory made the
 * first time (x a multiple of 32 bytes); *gem its GEM handle for the SUBMIT's list. 0: none could be made */
uint16_t fg_rect_alias(struct pipe_screen *pscreen, struct fg_resource *rsc, unsigned x, unsigned y, unsigned width,
                       unsigned height, uint32_t *gem);
bool fg_vertex_format_supported(enum pipe_format format);

/* level 0, layer 0 of a colour image, w x h from its corner, as it is stored: RGBA8 as bytes R, G, B, A, RGB565 as its
 * 16-bit words; read (after what was sent before is done) or written */
struct fg_context;
bool fg_image_io(struct fg_context *ctx, struct pipe_resource *prsc, bool write, unsigned w, unsigned h, void *data);

void fg_resource_screen_init(struct pipe_screen *pscreen);
void fg_resource_context_init(struct pipe_context *pctx);

#endif /* FG_RESOURCE_H */
