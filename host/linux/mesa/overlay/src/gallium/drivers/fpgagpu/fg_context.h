/* SPDX-License-Identifier: MIT */
/*
 * fpgagpu context: state, render passes and the command stream.
 *
 * A pass is what the hardware renders in one go: FRAME_BEGIN, then board A's state and draws and board B's draw
 * states, then a FLUSH that renders it all into the colour image (with the pass's clear) and the depth-stencil image.
 * The stream builds up in the context and goes to the kernel in SUBMITs; a pass longer than one SUBMIT's limit is
 * cut into several, in order.
 */
#ifndef FG_CONTEXT_H
#define FG_CONTEXT_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pipe/p_context.h"
#include "pipe/p_state.h"

#include "fg_compiler.h"

/* a message on stderr the first time an operation the driver does not carry out is asked for */
#define FG_NOTE_ONCE(what)                                                                 \
   do {                                                                                    \
      static bool fg_noted_;                                                               \
      if (!fg_noted_) {                                                                    \
         fg_noted_ = true;                                                                 \
         fprintf(stderr, "fpgagpu: %s is not implemented; ignored\n", what);               \
      }                                                                                    \
   } while (0)

#define FG_MAX_ATTRIBS  16
#define FG_MAX_SAMPLERS 8
#define FG_MAX_IDS      8192  /* board B's draw-state buckets */
#define FG_ID_SLOTS     16384

/* a buffer object of the context's own in a board's memory, filled from the front */
struct fg_ctx_bo {
   uint32_t gem;
   uint16_t handle;
   uint32_t size, used;
};

/* shaders */
struct fg_vs_state {
   nir_shader *nir;           /* lowered (fg_nir_lower) */
   struct fg_vs_eval ev;      /* its position and varyings on the CPU */
   struct fg_program prog;
   bool ok;
   uint64_t vmask;            /* fg_varying_mask of its outputs */
   uint32_t off;              /* its words in the vertex program buffer; ~0 = not placed yet */
   uint32_t off_gen;          /* the context's prog_gen it was placed under */
   unsigned id;
};

struct fg_fs_variant {
   struct fg_compile_key key;
   struct fg_program prog;
   bool ok;
   uint32_t off;
   uint32_t off_gen;
   struct fg_fs_variant *next;
};

struct fg_fs_state {
   nir_shader *nir;
   struct fg_fs_variant *variants;
   struct fg_fs_copy copy;    /* it copies an image (a draw with it may be done as COPY_IMAGE) */
};

/* fixed-function state, as DRAW_STATE words */
struct fg_blend_state {
   uint32_t h0;               /* blend [6], dither [7], colour mask [19:16] */
   uint32_t h4;               /* factors and equations */
   bool dither;
};

struct fg_dsa_state {
   uint32_t h0;               /* depth test [0], write [1], function [4:2], stencil [5] */
   uint32_t h1, h2;           /* front / back stencil without the reference: value mask, write mask, function */
   uint32_t h3;               /* stencil operations */
};

struct fg_sampler_state {
   uint32_t filter_nomip;     /* sampler word [19:16] for an image of one level */
   uint32_t filter;           /* with the mip filter */
   uint32_t wrap;             /* [22:20] wrap s, t; [26:25] r */
   uint8_t compare;           /* a depth compare (shadow sampler): its PIPE_FUNC_* + 1; 0 none */
};

struct fg_velem {
   struct pipe_vertex_element e;
   uint8_t type, size, normalized; /* VERTEX_ATTR's */
   uint8_t bytes;             /* of one element in memory */
};

struct fg_velems_state {
   unsigned count;
   struct fg_velem v[FG_MAX_ATTRIBS];
};

/* the target of the open pass */
struct fg_target {
   struct pipe_resource *color, *ds;
   struct pipe_resource *shadow_of; /* color is this image's render shadow: copied back after the FLUSH */
   uint8_t level, layer;      /* the level and layer of shadow_of it renders */
   uint16_t color_handle, ds_handle;
   uint32_t width, height;
   bool depth16, has_stencil;
   uint8_t bits[4];           /* colour channel widths: the clear colour's and blend constant's units */
   bool no_alpha;             /* the colour format has no alpha (X): its alpha stays 1.0 */
   uint8_t wmask;             /* the channels rendering writes (Gallium's order): fewer for A8, R8, RG8 */
};

struct fg_id_entry {
   uint64_t hash;
   uint32_t id;
};

/* A region pass renders into the rows of its target its draws and clears reach: a pass costs board B time for every
 * pixel of what the FLUSH names (board A walks its bands, board B loads and stores them), so a pass that draws little
 * names an alias of those rows. Whole rows: board B loads and stores a FLUSH image's lines to their pitch. The first
 * draw fixes the origin row (a multiple of 8): every coordinate the pass sends is relative to it, and the rows grow
 * down with each draw. A draw above the origin ends the pass. Passes with a depth-stencil image, and small targets,
 * cover the whole target */
struct fg_region {
   bool on;                   /* the origin is fixed */
   bool full;                 /* the whole target (no alias) */
   int ox, oy;                /* the origin, in the target (ox is 0: whole rows) */
   int x1, y1;                /* the furthest pixel reached + 1, in the target */
};

struct fg_context {
   struct pipe_context base;
   struct primconvert_context *primconvert;
   struct blitter_context *blitter;   /* blits drawn (what board B's copy engine does not do) */

   /* bound state */
   struct pipe_framebuffer_state fb;
   const struct fg_blend_state *blend;
   const struct fg_dsa_state *dsa;
   const struct pipe_rasterizer_state *rast;
   const struct fg_velems_state *velems;
   struct fg_vs_state *vs;
   struct fg_fs_state *fs;
   struct pipe_vertex_buffer vb[PIPE_MAX_ATTRIBS];
   unsigned nvb;
   struct pipe_sampler_view *views[FG_MAX_SAMPLERS];
   const struct fg_sampler_state *samplers[FG_MAX_SAMPLERS];
   struct pipe_viewport_state viewport;
   struct pipe_scissor_state scissor;
   struct pipe_blend_color blend_color;
   struct pipe_stencil_ref stencil_ref;
   uint8_t cb[2][FG_VS_PARAMS * 16]; /* constant buffer 0 of the vertex / fragment stage */
   uint32_t cb_size[2];

   /* the stream not yet submitted, and the BOs it names */
   uint8_t *cmd;
   uint32_t ncmd, cmdcap;
   uint32_t *bos;
   unsigned nbos, boscap;

   /* the context's own board memory */
   struct fg_ctx_bo prog_a, prog_b, ring;
   uint64_t ring_fence;       /* the fence after which what was streamed before the last wrap is no longer read */

   /* the pass */
   bool pass_open;
   unsigned pass_draws;
   uint64_t draw_count;       /* draws made, in all (not the blitter's: queries_off) */
   bool queries_off;
   struct fg_target tgt;
   bool have_tgt;
   struct pipe_resource **pass_res; /* what the open pass reads or writes, held until its FLUSH is submitted */
   unsigned npass_res, pass_res_cap;
   uint32_t cl_flags, cl_depth, cl_stencil, cl_smask, cl_sc[4], cl_c16[4];
   struct pipe_resource *discard_color; /* the next pass into it need not load its colour */
   struct pipe_resource *stencil_ds;    /* the depth-stencil image whose stencil value is known */
   bool stencil_known;
   uint32_t stencil_value;
   struct fg_vs_state clear_vs;          /* the quad of a clear inside a pass */
   struct fg_fs_variant clear_fs;
   struct fg_region rg;                  /* the open pass's rectangle */
   unsigned rg_splits;                   /* passes ended in a row by a draw above or left of the origin */
   bool rg_split_end;                    /* the pass that just ended was ended so */
   bool no_region;                       /* FPGAGPU_NOREGION=1: every pass covers its whole target */

   /* what the boards hold from this context (valid while it was the last to submit) */
   bool hw_valid;
   const struct fg_vs_state *hw_prog;
   uint32_t hw_attr[FG_MAX_ATTRIBS][8];
   bool hw_attr_ok;
   uint32_t hw_uni[FG_VS_PARAMS][4];
   uint8_t hw_uni_ok[FG_VS_PARAMS];
   uint8_t hw_vs_state[60];
   bool hw_vs_state_ok;

   /* draw ids */
   struct fg_id_entry ids[FG_ID_SLOTS];
   uint32_t next_id;
   uint32_t id_gen;

   uint32_t prog_gen;         /* programs placed under an older one are placed again (a SUBMIT carrying them failed) */

   unsigned next_vs_id;
   bool submitted;            /* last_fence is valid */
   uint64_t last_fence;
   uint64_t frame_fence;      /* the end of the previous frame: one frame is kept in flight */
   bool frame_fence_valid;
   bool debug;

   /* draws that copy an image one to one, done as COPY_IMAGE (FPGAGPU_NOCOPY=1: drawn; FPGAGPU_DEBUG_COPY=1: why a
    * draw with a copying fragment program was drawn, and each copy, on stderr) */
   bool no_copy;
   bool debug_copy;

   /* the fences of the last flushes that submitted work, a ring: the next such flush waits for the one flush_depth
    * flushes back (FPGAGPU_FLUSH_DEPTH, default 4: the X server flushes a few times a window move; 0: no wait) */
#define FG_FLUSH_RING 8
   uint64_t flush_fence[FG_FLUSH_RING];
   unsigned flush_n;          /* flushes recorded */
   unsigned flush_depth;
};

static inline struct fg_context *
fg_context(struct pipe_context *pctx)
{
   return (struct fg_context *)pctx;
}

struct pipe_context *fg_context_create(struct pipe_screen *pscreen, void *priv, unsigned flags);

/* ends the open pass (or sends a pending clear) and submits the stream */
void fg_context_flush_pending(struct fg_context *ctx);

/* whether the open pass or the pending clear reads or writes the resource */
bool fg_context_pending_uses(struct fg_context *ctx, struct pipe_resource *prsc);

/* records a submission's fence as the context's latest */
static inline void
fg_context_fenced(struct fg_context *ctx, uint64_t fence)
{
   ctx->submitted = true;
   ctx->last_fence = fence;
}

/* the stream (fg_context.c) */
void fg_emit(struct fg_context *ctx, uint8_t type, const void *head, uint32_t head_len, const void *tail,
             uint32_t tail_len);
void fg_use_bo(struct fg_context *ctx, uint32_t gem);
void fg_use_resource(struct fg_context *ctx, struct pipe_resource *prsc);
void fg_submit_stream(struct fg_context *ctx);
void fg_pass_open(struct fg_context *ctx);
void fg_end_pass(struct fg_context *ctx);
bool fg_set_target(struct fg_context *ctx);
void fg_target_hold(struct fg_context *ctx, const struct fg_target *t);
void fg_write_buffer(struct fg_context *ctx, struct fg_ctx_bo *bo, uint32_t offset, const void *data, uint32_t len);
uint32_t fg_stream(struct fg_context *ctx, const void *data, uint32_t bytes, uint32_t align);
void fg_emit_copy(struct fg_context *ctx, struct pipe_resource *src, unsigned slevel, unsigned slayer, unsigned sx,
                  unsigned sy, struct pipe_resource *dst, unsigned dlevel, unsigned dlayer, unsigned dx, unsigned dy,
                  unsigned w, unsigned h);
/* the stored alpha of a rectangle of an RGBA8 image set to 1.0 (FILL_IMAGE with the alpha channel alone) */
void fg_emit_alpha_one(struct fg_context *ctx, struct pipe_resource *dst, unsigned level, unsigned layer, unsigned x,
                       unsigned y, unsigned w, unsigned h);

/* COPY_IMAGE's flags. ALPHA_ONE: an RGBA8 destination's alpha = 255 (a FILL_IMAGE after the copy where board B lacks
 * the copy extensions); SWIZZLE: destination channel c (R, G, B, A) <- source channel FG_COPY_SEL(flags, c), on the
 * RGBA8 values (before ALPHA_ONE); TILE: the source rectangle repeats over the destination rectangle from a phase in it
 * (no mirrors); LIST: more 1:1 rectangles after the first, each at the first one's offset. SWIZZLE, TILE and LIST
 * only where the screen has copy_ext */
#define FG_COPY_MIRROR_X  0x01u
#define FG_COPY_MIRROR_Y  0x02u
#define FG_COPY_ALPHA_ONE 0x08u
#define FG_COPY_SWIZZLE   0x10u
#define FG_COPY_TILE      0x20u
#define FG_COPY_LIST      0x40u
#define FG_COPY_SEL(flags, c) ((flags) >> (8 + 2 * (c)) & 3u)
#define FG_COPY_LIST_MAX  160   /* rectangles in one COPY_IMAGE: 36 + 8 * 159 bytes fit one datagram */

/* one 1:1 rectangle of a copy; a tile (tile set): the destination rectangle (dx, dy, w, h) from the source rectangle
 * (sx, sy, tw, th) repeated, starting at (px, py) in it */
struct fg_copy_piece {
   int sx, sy, dx, dy, w, h;
   bool tile;
   int tw, th, px, py;
};

/* 1:1 copies in the given order, each with flags (ALPHA_ONE, SWIZZLE); a run of them at one offset is one COPY_IMAGE
 * with a list where the screen has copy_ext. Tiles go one a command */
void fg_emit_copies(struct fg_context *ctx, struct pipe_resource *src, struct pipe_resource *dst,
                    const struct fg_copy_piece *p, unsigned n, uint32_t flags);
/* what a copy from an image stored as sf, read as the view format vf through the swizzle sw (PIPE_SWIZZLE_* per
 * channel), into one of format df (the same storage) writes: COPY_IMAGE's SWIZZLE and ALPHA_ONE added to *flags.
 * false: board B cannot copy it so (a constant 0, a 1 other than in alpha, or a swizzle without copy_ext) */
struct fg_format;
bool fg_copy_channels(struct fg_context *ctx, const struct fg_format *sf, const struct fg_format *vf,
                      const struct fg_format *df, const unsigned char sw[4], uint32_t *flags);
/* a stretch or a mirror (FG_COPY_MIRROR_*), with ALPHA_ONE and SWIZZLE */
void fg_emit_copy_scaled(struct fg_context *ctx, struct pipe_resource *src, unsigned slevel, unsigned slayer,
                         unsigned sx, unsigned sy, unsigned sw, unsigned sh, struct pipe_resource *dst, unsigned dlevel,
                         unsigned dlayer, unsigned dx, unsigned dy, unsigned dw, unsigned dh, uint32_t flags);
/* before COPY_IMAGEs from src into dst: a pass that reads or writes either ends (board B copies at once and renders
 * at a FLUSH); after them: what dst's copies (shadow, tight rows) hold is old, and the stream goes out when no pass
 * is open */
void fg_copy_begin(struct fg_context *ctx, struct pipe_resource *src, struct pipe_resource *dst);
void fg_copy_end(struct fg_context *ctx, struct pipe_resource *dst);
/* the handle the TMU reads the image by: its tight-row copy when its rows are padded, or when it is the pass's target
 * (snapshot) */
uint16_t fg_sample_handle(struct fg_context *ctx, struct pipe_resource *prsc, bool snapshot);
/* the handle the TMU reads levels first .. last of the image by: a copy of those levels (fg_resource.lview) */
uint16_t fg_level_handle(struct fg_context *ctx, struct pipe_resource *prsc, unsigned first, unsigned last);

/* region passes: whether the open target may have one; a draw's or clear's pixels b (x0, y0, x1, y1 in the target)
 * admitted before anything of it is sent (it may end the pass first, or fix the origin); what it reached added after */
bool fg_region_ok(const struct fg_context *ctx);
void fg_region_admit(struct fg_context *ctx, const int b[4]);
void fg_region_grow(struct fg_context *ctx, const int b[4]);

/* the origin of the pass's coordinates in the target, and the size they span (the rest of the target past it) */
static inline int fg_rg_ox(const struct fg_context *ctx) { return ctx->rg.on ? ctx->rg.ox : 0; }
static inline int fg_rg_oy(const struct fg_context *ctx) { return ctx->rg.on ? ctx->rg.oy : 0; }
static inline uint32_t fg_rg_w(const struct fg_context *ctx) { return ctx->tgt.width - fg_rg_ox(ctx); }
static inline uint32_t fg_rg_h(const struct fg_context *ctx) { return ctx->tgt.height - fg_rg_oy(ctx); }

/* draws (fg_draw.c) */
void fg_draw_init(struct pipe_context *pctx);
void fg_draw_fini(struct fg_context *ctx);
void fg_hw_invalidate(struct fg_context *ctx);
void fg_hw_lost(struct fg_context *ctx);
void fg_clear_quad(struct fg_context *ctx, uint32_t flags, const uint32_t c16[4], uint32_t depth, uint32_t stencil,
                   uint32_t smask, const uint32_t sc[4]);
void fg_send_target_size(struct fg_context *ctx, uint32_t width, uint32_t height);

uint32_t fg_c16(double f, unsigned bits);

/* a Gallium colour mask (R 1, G 2, B 4, A 8) in the board's order (R 8, G 4, B 2, A 1): FLUSH's clear mask and
 * DRAW_STATE's write mask */
static inline uint32_t
fg_board_cmask(uint32_t m)
{
   return (m & 1u) << 3 | (m & 2u) << 1 | (m & 4u) >> 1 | (m & 8u) >> 3;
}

#endif /* FG_CONTEXT_H */
