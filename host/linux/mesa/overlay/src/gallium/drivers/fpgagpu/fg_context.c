/* SPDX-License-Identifier: MIT */
/*
 * fpgagpu context: the command stream, render passes, clears and flushes.
 *
 * The hardware keeps a pass's clear in its FLUSH: a clear before the pass's first draw merges into it, a clear after
 * a draw of the whole target's colour ends the pass, and one of a part of the target (or of depth / stencil alone) is
 * drawn as a quad inside the pass (fg_clear_quad). These are the Windows libraries' rules (host/win/core/g2_render.c).
 */
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "util/format/u_format.h"
#include "util/u_atomic.h"
#include "util/u_blitter.h"
#include "util/u_debug.h"
#include "util/u_framebuffer.h"
#include "util/u_inlines.h"
#include "util/u_memory.h"
#include "util/u_transfer.h"
#include "util/u_upload_mgr.h"

#include "xf86drm.h"
#include "drm-uapi/fpgagpu_drm.h"

#include "fg_context.h"
#include "fg_resource.h"
#include "fg_screen.h"

#define FG_T_WRITE_BUFFER 0x20
#define FG_T_COPY_IMAGE   0x25
#define FG_STREAM_CUT     (3u << 20) /* a SUBMIT is sent once its stream passes this (the kernel takes 4 MB) */
#define FG_PROG_A_BYTES   (4u << 20)
#define FG_PROG_B_BYTES   (16u << 20)
#define FG_RING_BYTES     (64u << 20)

uint32_t
fg_c16(double f, unsigned bits)
{
   double c = f > 0.0 ? (f < 1.0 ? f : 1.0) : 0.0;

   return bits ? (uint32_t)rint(c * 16.0 * (double)((1u << bits) - 1)) : 0;
}

/* ------------------------------------------------------------------ the stream */

void
fg_use_bo(struct fg_context *ctx, uint32_t gem)
{
   for (unsigned i = 0; i < ctx->nbos; i++)
      if (ctx->bos[i] == gem)
         return;
   if (ctx->nbos == ctx->boscap) {
      unsigned cap = ctx->boscap ? 2 * ctx->boscap : 64;
      uint32_t *n = realloc(ctx->bos, cap * sizeof(*n));

      if (!n)
         return;
      ctx->bos = n;
      ctx->boscap = cap;
   }
   ctx->bos[ctx->nbos++] = gem;
}

/* an image the open pass reads or writes: in the SUBMIT's list, and held until the pass's FLUSH is submitted (a BO
 * destroyed before it would be DESTROYed on the board ahead of the FLUSH that renders with it) */
void
fg_use_resource(struct fg_context *ctx, struct pipe_resource *prsc)
{
   struct fg_resource *rsc = fg_resource(prsc);

   if (!rsc->gem)
      return;
   fg_use_bo(ctx, rsc->gem);
   if (rsc->sample_gem)
      fg_use_bo(ctx, rsc->sample_gem);
   for (unsigned i = 0; i < ctx->npass_res; i++)
      if (ctx->pass_res[i] == prsc)
         return;
   if (ctx->npass_res == ctx->pass_res_cap) {
      unsigned cap = ctx->pass_res_cap ? 2 * ctx->pass_res_cap : 32;
      struct pipe_resource **n = realloc(ctx->pass_res, cap * sizeof(*n));

      if (!n)
         return;
      ctx->pass_res = n;
      ctx->pass_res_cap = cap;
   }
   ctx->pass_res[ctx->npass_res] = NULL;
   pipe_resource_reference(&ctx->pass_res[ctx->npass_res++], prsc);
}

void
fg_submit_stream(struct fg_context *ctx)
{
   struct fg_screen *screen = fg_screen(ctx->base.screen);
   uint64_t fence;
   int r;

   if (!ctx->ncmd)
      return;
   r = fg_submit(screen, ctx->cmd, ctx->ncmd, ctx->bos, ctx->nbos, &fence);
   if (r) {
      /* nothing of it reached the boards: what it sent (programs, state, draw ids) is sent again */
      fprintf(stderr, "fpgagpu: SUBMIT of %u bytes failed: %s\n", ctx->ncmd, strerror(-r));
      if (ctx->debug) {
         fprintf(stderr, "fpgagpu:   its %u GEM handles:", ctx->nbos);
         for (unsigned i = 0; i < ctx->nbos && i < 24; i++)
            fprintf(stderr, " %u", ctx->bos[i]);
         fprintf(stderr, "\n");
      }
      fg_hw_lost(ctx);
   } else {
      fg_context_fenced(ctx, fence);
   }
   ctx->ncmd = 0;
   /* in an open pass the list stays: a draw names its images and programs before the commands that use them, so a
    * cut in between must not leave them out of the next SUBMIT; the pass holds every image on the list */
   if (!ctx->pass_open) {
      /* no pass waits for a FLUSH: what the stream named is in the kernel's hands now */
      ctx->nbos = 0;
      for (unsigned i = 0; i < ctx->npass_res; i++)
         pipe_resource_reference(&ctx->pass_res[i], NULL);
      ctx->npass_res = 0;
   }
}

void
fg_emit(struct fg_context *ctx, uint8_t type, const void *head, uint32_t head_len, const void *tail, uint32_t tail_len)
{
   uint32_t need = 4 + ((head_len + tail_len + 3u) & ~3u);

   if (ctx->ncmd + need > FG_STREAM_CUT && ctx->ncmd)
      fg_submit_stream(ctx); /* the BOs of what follows are named again */
   if (ctx->ncmd + need > ctx->cmdcap) {
      uint32_t cap = ctx->cmdcap ? ctx->cmdcap : 65536;
      uint8_t *n;

      while (cap < ctx->ncmd + need)
         cap *= 2;
      n = realloc(ctx->cmd, cap);
      if (!n) {
         fprintf(stderr, "fpgagpu: out of memory for the command stream\n");
         return;
      }
      ctx->cmd = n;
      ctx->cmdcap = cap;
   }
   ctx->ncmd += fg_put_cmd(ctx->cmd + ctx->ncmd, type, head, head_len, tail, tail_len);
}

void
fg_write_buffer(struct fg_context *ctx, struct fg_ctx_bo *bo, uint32_t offset, const void *data, uint32_t len)
{
   const uint8_t *p = data;

   for (uint32_t i = 0; i < len; i += FG_MAX_DATA) {
      uint32_t n = MIN2(len - i, FG_MAX_DATA);
      uint8_t hdr[8];

      fg_put16(hdr, bo->handle);
      fg_put16(hdr + 2, 0);
      fg_put32(hdr + 4, offset + i);
      fg_emit(ctx, FG_T_WRITE_BUFFER, hdr, sizeof(hdr), p + i, n);
      fg_use_bo(ctx, bo->gem);
   }
}

/* per-draw data for board A: copied into the ring, the offset returned. On a wrap what was submitted before is
 * waited for: board A fetches a draw's vertices when it takes the DRAW, so a fence after it proves they are read */
uint32_t
fg_stream(struct fg_context *ctx, const void *data, uint32_t bytes, uint32_t align)
{
   uint32_t a = (ctx->ring.used + align - 1) & ~(align - 1);

   if (a + bytes > ctx->ring.size) {
      fg_submit_stream(ctx);
      if (ctx->submitted)
         fg_wait_fence(fg_screen(ctx->base.screen), ctx->last_fence, FG_WAIT_NS);
      a = 0;
   }
   fg_write_buffer(ctx, &ctx->ring, a, data, bytes);
   ctx->ring.used = a + bytes;
   return a;
}

static bool
fg_ctx_bo_create(struct fg_context *ctx, struct fg_ctx_bo *bo, uint32_t kind, uint32_t size)
{
   struct drm_fpgagpu_bo_create c = { .kind = kind, .size = size };

   if (drmIoctl(fg_screen(ctx->base.screen)->fd, DRM_IOCTL_FPGAGPU_BO_CREATE, &c)) {
      fprintf(stderr, "fpgagpu: BO_CREATE of a %u MB buffer failed: %s\n", size >> 20, strerror(errno));
      return false;
   }
   bo->gem = c.handle;
   bo->handle = (uint16_t)c.board_handle;
   bo->size = size;
   bo->used = 0;
   return true;
}

static void
fg_ctx_bo_destroy(struct fg_context *ctx, struct fg_ctx_bo *bo)
{
   if (bo->gem) {
      struct drm_gem_close c = { .handle = bo->gem };

      drmIoctl(fg_screen(ctx->base.screen)->fd, DRM_IOCTL_GEM_CLOSE, &c);
   }
   bo->gem = 0;
}

/* ------------------------------------------------------------------ region passes */

bool
fg_region_ok(const struct fg_context *ctx)
{
   return !ctx->no_region && ctx->have_tgt && ctx->tgt.color && !ctx->tgt.ds && fg_resource(ctx->tgt.color)->gem &&
          ctx->tgt.width * ctx->tgt.height >= 128u * 128u;
}

void
fg_region_grow(struct fg_context *ctx, const int b[4])
{
   if (!ctx->rg.on || ctx->rg.full || b[2] <= b[0] || b[3] <= b[1])
      return;
   ctx->rg.x1 = MAX2(ctx->rg.x1, MIN2(b[2], (int)ctx->tgt.width));
   ctx->rg.y1 = MAX2(ctx->rg.y1, MIN2(b[3], (int)ctx->tgt.height));
}

void
fg_region_admit(struct fg_context *ctx, const int b[4])
{
   int y0 = b[1];

   if (ctx->rg.on) {
      if (ctx->rg.full || y0 >= ctx->rg.oy || b[2] <= b[0] || b[3] <= b[1])
         return;
      /* above the origin: this pass ends here, the draw starts the next one */
      ctx->rg_splits++;
      ctx->rg_split_end = true;
      fg_end_pass(ctx);
   }
   if (!ctx->rg_split_end)
      ctx->rg_splits = 0;
   ctx->rg_split_end = false;
   ctx->rg.on = true;
   /* draws that keep landing above each other: the whole target from here */
   ctx->rg.full = !fg_region_ok(ctx) || ctx->rg_splits >= 4;
   /* a clear already in the pass is part of it: the whole target for one without a scissor */
   if (ctx->cl_flags) {
      if (!(ctx->cl_flags & FG_FL_CLEAR_SCISSOR))
         ctx->rg.full = true;
      else
         y0 = MIN2(y0, (int)ctx->cl_sc[1]);
   }
   if (ctx->rg.full) {
      ctx->rg.ox = ctx->rg.oy = 0;
      ctx->rg.x1 = ctx->tgt.width;
      ctx->rg.y1 = ctx->tgt.height;
      return;
   }
   /* whole rows: board B loads and stores each line of a FLUSH's image to its pitch, so a narrower rectangle with the
    * target's pitch would write past its right edge into the next row */
   y0 = CLAMP(y0, 0, (int)ctx->tgt.height - 1);
   ctx->rg.ox = 0;
   ctx->rg.oy = y0 & ~7;
   ctx->rg.x1 = ctx->tgt.width;
   ctx->rg.y1 = ctx->rg.oy;
   if (ctx->cl_flags & FG_FL_CLEAR_SCISSOR) {
      const int c[4] = { (int)ctx->cl_sc[0], (int)ctx->cl_sc[1], (int)ctx->cl_sc[2], (int)ctx->cl_sc[3] };

      fg_region_grow(ctx, c);
   }
}

/* what the ending pass's FLUSH names: the target, or the alias of the rectangle its draws and clears reached */
struct fg_flush_rect {
   int x, y;
   uint32_t w, h;
   uint16_t handle;
};

static void
fg_region_finish(struct fg_context *ctx, struct fg_flush_rect *r)
{
   struct pipe_screen *pscreen = ctx->base.screen;
   struct fg_resource *c = fg_resource(ctx->tgt.color);
   const int W = ctx->tgt.width, H = ctx->tgt.height;
   int x0 = 0, y0, x1 = W, y1;
   uint32_t gem = 0;
   uint16_t h;

   r->x = r->y = 0;
   r->w = W;
   r->h = H;
   r->handle = ctx->tgt.color_handle;
   if (!fg_region_ok(ctx) || (ctx->rg.on && ctx->rg.full))
      return;
   if (ctx->rg.on) {
      y0 = ctx->rg.oy;
      y1 = ctx->rg.y1;
   } else if (ctx->cl_flags & FG_FL_CLEAR_SCISSOR) {
      /* a clear alone: its rows (the FLUSH's clear is made relative to them) */
      y0 = (int)ctx->cl_sc[1] & ~7;
      y1 = ctx->cl_sc[3];
   } else if (!ctx->cl_flags) {
      y0 = y1 = 0;                        /* nothing reached: the fewest rows */
   } else {
      return;
   }
   /* whole bands of 8 rows, which lets the aliases be used again */
   y1 = MIN2(MAX2(y1, y0 + 1), H);
   y1 = MIN2(y0 + (int)align(y1 - y0, 8), H);
   if (y0 == 0 && y1 == H)
      return;
   h = fg_rect_alias(pscreen, c, x0, y0, x1 - x0, y1 - y0, &gem);
   if (!h) {
      /* the handles may have run out: this image's other rectangles go, then once more */
      for (unsigned i = 0; i < ARRAY_SIZE(c->ralias); i++) {
         if (c->ralias[i].gem)
            fg_gem_unref(fg_screen(pscreen), c->ralias[i].gem);
         c->ralias[i].gem = 0;
      }
      h = fg_rect_alias(pscreen, c, x0, y0, x1 - x0, y1 - y0, &gem);
   }
   if (!h) {
      /* the draws were sent relative to the origin: they land there in the target (seen as misplaced) */
      fprintf(stderr, "fpgagpu: no alias for a %dx%d rectangle at %d, %d; the pass renders into the whole target\n",
              x1 - x0, y1 - y0, x0, y0);
      return;
   }
   fg_use_bo(ctx, gem);
   r->x = x0;
   r->y = y0;
   r->w = x1 - x0;
   r->h = y1 - y0;
   r->handle = h;
}

/* ------------------------------------------------------------------ passes */

/* FLUSH: {target, flags}, clear colour {R, G} and {B, A} (13 bits each), {depth 24, stencil 8}, the clear's
 * scissor {x0, y0}, {x1, y1}, {depth-stencil image, stencil write mask}. The target is r's (its clear scissor made
 * relative to it) */
static void
fg_emit_flush(struct fg_context *ctx, uint32_t flags, const struct fg_flush_rect *r)
{
   uint8_t b[28];

   fg_put32(b + 0, (uint32_t)r->handle | (flags & 0xFFFFu) << 16);
   fg_put32(b + 4, (ctx->cl_c16[0] & 0x1FFFu) | (ctx->cl_c16[1] & 0x1FFFu) << 13);
   fg_put32(b + 8, (ctx->cl_c16[2] & 0x1FFFu) | (ctx->cl_c16[3] & 0x1FFFu) << 13);
   fg_put32(b + 12, (ctx->cl_depth & 0xFFFFFFu) | (ctx->cl_stencil & 0xFFu) << 24);
   if (flags & FG_FL_CLEAR_SCISSOR) {
      uint32_t sx0 = (uint32_t)MAX2((int)ctx->cl_sc[0] - r->x, 0), sy0 = (uint32_t)MAX2((int)ctx->cl_sc[1] - r->y, 0);
      uint32_t sx1 = (uint32_t)MAX2((int)ctx->cl_sc[2] - r->x, 0), sy1 = (uint32_t)MAX2((int)ctx->cl_sc[3] - r->y, 0);

      fg_put32(b + 16, (sx0 & 0x1FFFu) | (sy0 & 0x1FFFu) << 16);
      fg_put32(b + 20, (MIN2(sx1, r->w) & 0x1FFFu) | (MIN2(sy1, r->h) & 0x1FFFu) << 16);
   } else {
      fg_put32(b + 16, 0);
      fg_put32(b + 20, 0);
   }
   fg_put32(b + 24, (uint32_t)ctx->tgt.ds_handle | (ctx->cl_smask & 0xFFu) << 16);
   fg_emit(ctx, FG_T_FLUSH, b, sizeof(b), NULL, 0);
   fg_use_resource(ctx, ctx->tgt.color);
   if (ctx->tgt.ds)
      fg_use_resource(ctx, ctx->tgt.ds);
   if (ctx->tgt.ds_alias_gem)
      fg_use_bo(ctx, ctx->tgt.ds_alias_gem);
}

/* the boards' render state for this process until the pass ends (another process's pass would land in the middle of
 * one that takes several SUBMITs); what another process sent since this one last held it replaced its state. After a
 * board's restart (state_lost 2) the board memory is empty too: the programs are placed again as well */
static void
fg_pass_take(struct fg_context *ctx, bool take)
{
   struct drm_fpgagpu_pass p = {
      .op = take ? FPGAGPU_PASS_BEGIN : FPGAGPU_PASS_END,
      .timeout_ns = 5ll * 1000 * 1000 * 1000,
   };

   if (drmIoctl(fg_screen(ctx->base.screen)->fd, DRM_IOCTL_FPGAGPU_PASS, &p))
      return;
   if (take && p.state_lost == 2)
      fg_hw_lost(ctx);
   else if (take && p.state_lost)
      fg_hw_invalidate(ctx);
}

#define FG_PASS_WAIT_NS (5ll * 1000 * 1000 * 1000)

/* the screen's boards for this context's pass: another context of the screen with a pass open (another thread) ends it
 * first; after FG_PASS_WAIT_NS they are taken anyway, as the kernel does between processes */
static void
fg_screen_pass_hold(struct fg_context *ctx, bool hold)
{
   struct fg_screen *screen = fg_screen(ctx->base.screen);

   mtx_lock(&screen->pass_mtx);
   if (hold) {
      struct timespec until;

      timespec_get(&until, TIME_UTC);
      until.tv_sec += FG_PASS_WAIT_NS / 1000000000ll;
      while (screen->pass_ctx && screen->pass_ctx != ctx) {
         if (cnd_timedwait(&screen->pass_cnd, &screen->pass_mtx, &until) == thrd_timedout) {
            fprintf(stderr, "fpgagpu: another context's pass did not end in 5 s: its boards are taken\n");
            break;
         }
      }
      screen->pass_ctx = ctx;
   } else if (screen->pass_ctx == ctx) {
      screen->pass_ctx = NULL;
      cnd_broadcast(&screen->pass_cnd);
   }
   mtx_unlock(&screen->pass_mtx);
}

void
fg_pass_open(struct fg_context *ctx)
{
   uint8_t z[4] = { 0, 0, 0, 0 };

   if (ctx->pass_open)
      return;
   fg_screen_pass_hold(ctx, true);
   fg_pass_take(ctx, true);
   fg_emit(ctx, FG_T_FRAME_BEGIN, z, sizeof(z), NULL, 0);
   ctx->pass_open = true;
   ctx->pass_draws = 0;
}

void
fg_end_pass(struct fg_context *ctx)
{
   struct fg_flush_rect r;
   uint32_t fl;

   if (!ctx->pass_open) {
      if (!ctx->cl_flags || !ctx->have_tgt) {
         ctx->rg.on = false;
         return;
      }
      fg_pass_open(ctx);
   }
   /* board A ends the frame's bands by the size the last VS_STATE gave: the rectangle's */
   fg_region_finish(ctx, &r);
   fg_send_target_size(ctx, r.w, r.h);
   if (ctx->discard_color && ctx->discard_color == ctx->tgt.color) {
      /* the colour is undefined: a clear costs nothing, loading the band does */
      if (!(ctx->cl_flags & (FG_FL_CLEAR_COLOR | FG_FL_CLEAR_SCISSOR))) {
         ctx->cl_flags |= FG_FL_CLEAR_COLOR | FG_FL_CMASK(0xF);
         memset(ctx->cl_c16, 0, sizeof(ctx->cl_c16));
      }
      pipe_resource_reference(&ctx->discard_color, NULL);
   }
   /* depth and stencil share the D24S8 word: a whole-image depth clear keeps the stencil by clearing it to what it
    * already is, so the band need not be loaded */
   if ((ctx->cl_flags & FG_FL_CLEAR_DEPTH) && !(ctx->cl_flags & (FG_FL_CLEAR_STENCIL | FG_FL_CLEAR_SCISSOR)) &&
       ctx->tgt.ds && (!ctx->tgt.has_stencil || (ctx->stencil_known && ctx->stencil_ds == ctx->tgt.ds))) {
      ctx->cl_flags |= FG_FL_CLEAR_STENCIL;
      ctx->cl_stencil = ctx->tgt.has_stencil ? ctx->stencil_value : 0;
      ctx->cl_smask = 0xFF;
   }
   /* the bands no draw and no clear reaches are left alone where both boards can (each band costs board B a load and
    * a store otherwise, whatever is drawn) */
   fl = ctx->cl_flags | (ctx->tgt.depth16 ? FG_FL_DEPTH16 : 0) |
        (fg_screen(ctx->base.screen)->band_skip ? FG_FL_SKIP_BANDS : 0);
   fg_emit_flush(ctx, fl, &r);
   if (ctx->tgt.shadow_of) {
      /* rendered in the shadow: back into the image's level and layer (what the FLUSH covered), after the FLUSH that
       * rendered it */
      fg_emit_copy(ctx, ctx->tgt.color, 0, 0, r.x, r.y, ctx->tgt.shadow_of, ctx->tgt.level, ctx->tgt.layer, r.x, r.y,
                   r.w, r.h);
      fg_resource(ctx->tgt.shadow_of)->initialized = true;
   }
   if (ctx->tgt.ds_shadow_of) {
      fg_emit_copy(ctx, ctx->tgt.ds, 0, 0, r.x, r.y, ctx->tgt.ds_shadow_of, ctx->tgt.ds_level, ctx->tgt.ds_layer, r.x,
                   r.y, r.w, r.h);
      fg_resource(ctx->tgt.ds_shadow_of)->initialized = true;
   }
   fg_submit_stream(ctx);
   if (ctx->cl_flags & FG_FL_CLEAR_COLOR)
      fg_resource(ctx->tgt.color)->initialized = true;
   if (ctx->tgt.ds && (ctx->cl_flags & (FG_FL_CLEAR_DEPTH | FG_FL_CLEAR_STENCIL)))
      fg_resource(ctx->tgt.ds)->initialized = true;
   if (ctx->pass_draws) {
      fg_resource(ctx->tgt.color)->initialized = true;
      if (ctx->tgt.ds)
         fg_resource(ctx->tgt.ds)->initialized = true;
   }
   /* what the pass wrote: a tight-row copy of it is taken again before it is sampled */
   if (ctx->tgt.color)
      fg_resource_written(fg_resource(ctx->tgt.color));
   if (ctx->tgt.shadow_of)
      fg_resource_written(fg_resource(ctx->tgt.shadow_of));
   if (ctx->tgt.ds)
      fg_resource_written(fg_resource(ctx->tgt.ds));
   if (ctx->tgt.ds_shadow_of)
      fg_resource_written(fg_resource(ctx->tgt.ds_shadow_of));
   p_atomic_inc(&fg_screen(ctx->base.screen)->pass_seq);
   for (unsigned i = 0; i < ctx->npass_res; i++)
      pipe_resource_reference(&ctx->pass_res[i], NULL);
   ctx->npass_res = 0;
   ctx->nbos = 0;
   ctx->pass_open = false;
   fg_pass_take(ctx, false);
   fg_screen_pass_hold(ctx, false);
   ctx->pass_draws = 0;
   ctx->cl_flags = 0;
   ctx->cl_depth = 0xFFFFFF;
   ctx->cl_smask = 0xFF;
   ctx->rg.on = false;
}

/* the pass's target images are held while they are the target: the frontend may let go of a framebuffer's image while
 * the pass that renders into it is still open (its FLUSH and copy back come later) */
void
fg_target_hold(struct fg_context *ctx, const struct fg_target *t)
{
   struct fg_target old = ctx->tgt;
   bool had = ctx->have_tgt;

   ctx->tgt = t ? *t : (struct fg_target){ 0 };
   ctx->have_tgt = t != NULL;
   if (t) {
      struct pipe_resource *hold[4] = { t->color, t->ds, t->shadow_of, t->ds_shadow_of }, *p;

      for (unsigned i = 0; i < 4; i++) {
         p = NULL;
         pipe_resource_reference(&p, hold[i]);    /* one more reference, kept by ctx->tgt */
      }
   }
   if (had) {
      pipe_resource_reference(&old.color, NULL);
      pipe_resource_reference(&old.ds, NULL);
      pipe_resource_reference(&old.shadow_of, NULL);
      pipe_resource_reference(&old.ds_shadow_of, NULL);
   }
}

/* the image board B renders into in place of a level of c (that level's size, pitch a multiple of 32, one level, one
 * layer), made the first time */
static struct pipe_resource *
fg_render_shadow(struct pipe_resource *c, unsigned level)
{
   struct fg_resource *r = fg_resource(c);

   if (!r->shadow[level]) {
      struct pipe_resource templ = *c;

      templ.target = PIPE_TEXTURE_2D;
      templ.bind = r->fmt->depth ? PIPE_BIND_DEPTH_STENCIL : PIPE_BIND_RENDER_TARGET;
      templ.width0 = u_minify(c->width0, level);
      templ.height0 = u_minify(c->height0, level);
      templ.last_level = 0;
      templ.array_size = 1;
      r->shadow[level] = c->screen->resource_create(c->screen, &templ);
      r->shadow_layer[level] = -1;
   }
   return r->shadow[level];
}

/* the framebuffer's first colour buffer and depth-stencil buffer as the pass's target; a change ends the pass.
 * false: nothing board B can render into */
bool
fg_set_target(struct fg_context *ctx)
{
   struct pipe_framebuffer_state *fb = &ctx->fb;
   struct pipe_resource *c = fb->nr_cbufs ? fb->cbufs[0].texture : NULL, *rt;
   struct pipe_resource *z = fb->zsbuf.texture, *zt = z;
   struct fg_target t;
   const struct fg_format *f;
   unsigned level, layer, w, h, zlevel = fb->zsbuf.level, zlayer = fb->zsbuf.first_layer;

   if (!c) {
      FG_NOTE_ONCE("rendering without a colour buffer");
      if (ctx->debug) {
         static unsigned shown;

         if (shown++ < 8)
            fprintf(stderr, "fpgagpu: framebuffer %ux%u with %u colour buffers (first %p), depth-stencil %p\n",
                    fb->width, fb->height, fb->nr_cbufs, (void *)c, (void *)z);
      }
      return false;
   }
   level = fb->cbufs[0].level;
   layer = fb->cbufs[0].first_layer;
   w = u_minify(c->width0, level);
   h = u_minify(c->height0, level);
   if (level >= FG_MAX_LEVELS || w > FG_FLUSH_MAX_WIDTH ||
       (z && (zlevel >= FG_MAX_LEVELS || u_minify(z->width0, zlevel) < w || u_minify(z->height0, zlevel) < h))) {
      FG_NOTE_ONCE("rendering into a target wider than 1280, or with depth or stencil smaller than the colour");
      return false;
   }
   f = fg_resource(c)->fmt;
   if (!fg_format_renderable(f))
      return false;
   rt = c;
   if (level || layer || fg_resource(c)->pitch % 32) {
      rt = fg_render_shadow(c, level);
      if (!rt)
         return false;
   }
   /* depth and stencil of a level or layer above 0, or of rows that are not such a pitch, or narrower than board B
    * loads them (as many bursts as the colour row's times the bursts of a group, b_rop: pitch x 4 / colour bytes): in a
    * shadow the same way (its pitch a multiple of 64) */
   if (z && (zlevel || zlayer || fg_resource(z)->pitch % 32 ||
             fg_resource(z)->pitch * fg_storage_bpp(f) < 4 * fg_resource(rt)->pitch)) {
      zt = fg_render_shadow(z, zlevel);
      if (!zt)
         return false;
      if (fg_resource(zt)->pitch * fg_storage_bpp(f) < 4 * fg_resource(rt)->pitch)
         FG_NOTE_ONCE("a depth-stencil image narrower than its colour target's rows");
   }
   memset(&t, 0, sizeof(t));
   t.color = rt;
   t.shadow_of = rt != c ? c : NULL;
   t.level = (uint8_t)level;
   t.layer = (uint8_t)layer;
   t.ds = zt;
   t.ds_shadow_of = zt != z ? z : NULL;
   t.ds_level = (uint8_t)zlevel;
   t.ds_layer = (uint8_t)zlayer;
   t.color_handle = fg_resource(rt)->board_handle;
   t.ds_handle = zt ? fg_resource(zt)->board_handle : 0;
   /* board B's FLUSH takes a depth-stencil image of the colour image's size only: a bigger one is named by an alias of
    * its top left corner (its pitch kept) */
   if (zt && (zt->width0 != w || zt->height0 != h)) {
      t.ds_handle = fg_rect_alias(ctx->base.screen, fg_resource(zt), 0, 0, w, h, &t.ds_alias_gem);
      if (!t.ds_handle) {
         FG_NOTE_ONCE("no alias of a depth-stencil image bigger than its colour target");
         return false;
      }
   }
   t.width = w;
   t.height = h;
   t.depth16 = z && z->format == PIPE_FORMAT_Z16_UNORM;
   t.has_stencil = z && util_format_has_stencil(util_format_description(z->format));
   memcpy(t.bits, f->bits, sizeof(t.bits));
   t.no_alpha = f->force_alpha;
   t.wmask = (uint8_t)fg_format_wmask(f);
   if (!ctx->have_tgt || memcmp(&ctx->tgt, &t, sizeof(t))) {
      fg_end_pass(ctx);
      fg_target_hold(ctx, &t);
   }
   if (t.shadow_of && fg_resource(c)->shadow_layer[level] != (int)layer) {
      /* the image changed (an upload, a copy, a first use), or the shadow holds another layer: it takes the level and
       * layer before the pass renders. Board B copies at once and renders at the FLUSH, so this lands under the
       * pass's draws whenever it is sent */
      fg_emit_copy(ctx, c, level, layer, 0, 0, rt, 0, 0, 0, 0, t.width, t.height);
      fg_resource(c)->shadow_layer[level] = (int)layer;
   }
   if (t.ds_shadow_of && fg_resource(z)->shadow_layer[zlevel] != (int)zlayer) {
      fg_emit_copy(ctx, z, zlevel, zlayer, 0, 0, zt, 0, 0, 0, 0, u_minify(z->width0, zlevel),
                   u_minify(z->height0, zlevel));
      fg_resource(z)->shadow_layer[zlevel] = (int)zlayer;
   }
   return true;
}

bool
fg_context_pending_uses(struct fg_context *ctx, struct pipe_resource *prsc)
{
   if (!prsc)
      return false;
   if (ctx->cl_flags && ctx->have_tgt && (ctx->tgt.color == prsc || ctx->tgt.ds == prsc))
      return true;
   if ((ctx->pass_open || ctx->cl_flags) && ctx->have_tgt &&
       (ctx->tgt.color == prsc || ctx->tgt.ds == prsc || ctx->tgt.shadow_of == prsc || ctx->tgt.ds_shadow_of == prsc))
      return true;
   for (unsigned i = 0; i < ctx->npass_res; i++)
      if (ctx->pass_res[i] == prsc)
         return true;
   return false;
}

void
fg_context_flush_pending(struct fg_context *ctx)
{
   fg_end_pass(ctx);
   fg_submit_stream(ctx);
}

/* ------------------------------------------------------------------ clears */

static void
fg_clear(struct pipe_context *pctx, unsigned buffers, uint32_t color_clear_mask, uint8_t stencil_clear_mask,
         const struct pipe_scissor_state *scissor, const union pipe_color_union *color, double depth,
         unsigned stencil)
{
   struct fg_context *ctx = fg_context(pctx);
   const uint32_t kinds = FG_FL_CLEAR_COLOR | FG_FL_CLEAR_DEPTH | FG_FL_CLEAR_STENCIL;
   uint32_t want = 0, flags, have, sc[4] = { 0, 0, 0, 0 }, cmask, dz = 0, c16[4] = { 0, 0, 0, 0 };
   struct fg_target *t;
   bool same_sc;

   if (!fg_set_target(ctx))
      return;
   t = &ctx->tgt;
   cmask = color_clear_mask & t->wmask;
   if (t->no_alpha)
      cmask = cmask ? cmask | 0x8 : 0;
   else if (!t->bits[3])
      cmask &= 0x7;
   if ((buffers & PIPE_CLEAR_COLOR0) && cmask)
      want |= FG_FL_CLEAR_COLOR;
   if ((buffers & PIPE_CLEAR_DEPTH) && t->ds)
      want |= FG_FL_CLEAR_DEPTH;
   if ((buffers & PIPE_CLEAR_STENCIL) && t->has_stencil)
      want |= FG_FL_CLEAR_STENCIL;
   if (!want)
      return;
   flags = want;
   if (want & FG_FL_CLEAR_COLOR) {
      flags |= FG_FL_CMASK(fg_board_cmask(cmask));
      if (ctx->blend && ctx->blend->dither)
         flags |= FG_FL_DITHER;
      for (unsigned i = 0; i < 3; i++)
         c16[i] = fg_c16(color->f[i], t->bits[i]);
      c16[3] = t->no_alpha ? fg_c16(1.0, 8) : fg_c16(color->f[3], t->bits[3]);
   }
   if (want & FG_FL_CLEAR_DEPTH) {
      double d = depth > 0.0 ? (depth < 1.0 ? depth : 1.0) : 0.0;

      dz = (uint32_t)rint(d * (t->depth16 ? 65535.0 : 16777215.0));
   }
   if (scissor && (scissor->minx > 0 || scissor->miny > 0 || scissor->maxx < t->width || scissor->maxy < t->height)) {
      if (scissor->maxx <= scissor->minx || scissor->maxy <= scissor->miny)
         return;
      flags |= FG_FL_CLEAR_SCISSOR;
      sc[0] = scissor->minx;
      sc[1] = scissor->miny;
      sc[2] = MIN2(scissor->maxx, t->width);
      sc[3] = MIN2(scissor->maxy, t->height);
   }

   /* what the stencil holds from here on */
   if (want & FG_FL_CLEAR_STENCIL) {
      if (!(flags & FG_FL_CLEAR_SCISSOR) && stencil_clear_mask == 0xFF) {
         pipe_resource_reference(&ctx->stencil_ds, t->ds);
         ctx->stencil_known = true;
         ctx->stencil_value = stencil & 0xFF;
      } else if (stencil_clear_mask &&
                 !(ctx->stencil_known && (stencil & stencil_clear_mask) == (ctx->stencil_value & stencil_clear_mask))) {
         ctx->stencil_known = false;
      }
   }

   /* a region pass takes a clear inside its coordinates only (right of and below its origin) */
   if (ctx->pass_draws && ctx->rg.on && !ctx->rg.full &&
       (!(flags & FG_FL_CLEAR_SCISSOR) || (int)sc[0] < ctx->rg.ox || (int)sc[1] < ctx->rg.oy))
      fg_end_pass(ctx);

   /* inside a pass that has draws: a quad, unless it is a whole-target colour clear (that one ends the pass, and the
    * next starts with the FLUSH's fast clear). A quad reaches depth 0 and all ones exactly */
   if (ctx->pass_draws && (!(want & FG_FL_CLEAR_COLOR) || (flags & FG_FL_CLEAR_SCISSOR)) &&
       (!(want & FG_FL_CLEAR_DEPTH) || dz == 0 || dz == (t->depth16 ? 0xFFFFu : 0xFFFFFFu))) {
      fg_clear_quad(ctx, flags, c16, dz, stencil & 0xFF, stencil_clear_mask, sc);
      return;
   }

   /* clears merge into the pass while it has no draw, when the two agree on the rectangle and share nothing */
   have = ctx->cl_flags & kinds;
   same_sc = (ctx->cl_flags & FG_FL_CLEAR_SCISSOR) == (flags & FG_FL_CLEAR_SCISSOR) &&
             (!(flags & FG_FL_CLEAR_SCISSOR) || !memcmp(ctx->cl_sc, sc, sizeof(sc)));
   if (ctx->pass_draws || ctx->rg.on || (have && (!same_sc || (have & want))))
      fg_end_pass(ctx);
   if (want & FG_FL_CLEAR_COLOR)
      memcpy(ctx->cl_c16, c16, sizeof(c16));
   if (want & FG_FL_CLEAR_DEPTH)
      ctx->cl_depth = dz;
   if (want & FG_FL_CLEAR_STENCIL) {
      ctx->cl_stencil = stencil & 0xFF;
      ctx->cl_smask = stencil_clear_mask;
   }
   ctx->cl_flags |= flags;
   if (flags & FG_FL_CLEAR_SCISSOR)
      memcpy(ctx->cl_sc, sc, sizeof(sc));
}

/* ------------------------------------------------------------------ flushes */

static void
fg_flush(struct pipe_context *pctx, struct pipe_fence_handle **fence, unsigned flags)
{
   struct fg_context *ctx = fg_context(pctx);
   struct fg_screen *screen = fg_screen(pctx->screen);

   fg_context_flush_pending(ctx);
   /* a process that makes work faster than the boards do it (the X server moves a window at every pointer motion, a
    * big one costing board B several ms) would queue it ever deeper, and what shows would trail behind: a flush that
    * submitted work waits for the one flush_depth flushes back, so the boards do those while the process makes the
    * next */
   if (ctx->flush_depth && ctx->submitted &&
       (!ctx->flush_n || ctx->flush_fence[(ctx->flush_n - 1) % FG_FLUSH_RING] != ctx->last_fence)) {
      if (ctx->flush_n >= ctx->flush_depth)
         fg_wait_fence(screen, ctx->flush_fence[(ctx->flush_n - ctx->flush_depth) % FG_FLUSH_RING], FG_WAIT_NS);
      ctx->flush_fence[ctx->flush_n % FG_FLUSH_RING] = ctx->last_fence;
      ctx->flush_n++;
   }
   if (flags & PIPE_FLUSH_END_OF_FRAME) {
      /* one frame in flight: board A bins the next while board B renders this one */
      if (ctx->frame_fence_valid)
         fg_wait_fence(screen, ctx->frame_fence, FG_WAIT_NS);
      ctx->frame_fence = ctx->last_fence;
      ctx->frame_fence_valid = ctx->submitted;
   }
   if (fence) {
      struct pipe_fence_handle *f = fg_fence_create(ctx->submitted, ctx->last_fence);

      pctx->screen->fence_reference(pctx->screen, fence, NULL);
      *fence = f;
   }
}

static void
fg_set_framebuffer_state(struct pipe_context *pctx, const struct pipe_framebuffer_state *state)
{
   util_copy_framebuffer_state(&fg_context(pctx)->fb, state);
}

/* the next pass into this colour image need not keep its contents (a back buffer after a swap) */
static void
fg_invalidate_resource(struct pipe_context *pctx, struct pipe_resource *prsc)
{
   struct fg_context *ctx = fg_context(pctx);

   if (prsc && prsc->target != PIPE_BUFFER && fg_resource(prsc)->fmt && !fg_resource(prsc)->fmt->depth)
      pipe_resource_reference(&ctx->discard_color, prsc);
}

static void fg_blit(struct pipe_context *pctx, const struct pipe_blit_info *info);

static void
fg_resource_copy_region(struct pipe_context *pctx, struct pipe_resource *dst, unsigned dst_level, unsigned dstx,
                        unsigned dsty, unsigned dstz, struct pipe_resource *src, unsigned src_level,
                        const struct pipe_box *src_box)
{
   struct fg_context *ctx = fg_context(pctx);
   struct fg_resource *s = fg_resource(src), *d = fg_resource(dst);

   if (dst->target == PIPE_BUFFER && src->target == PIPE_BUFFER) {
      memcpy((uint8_t *)d->data + dstx, (uint8_t *)s->data + src_box->x, src_box->width);
      return;
   }
   if (dst->target == PIPE_BUFFER || src->target == PIPE_BUFFER || !s->fmt || !d->fmt ||
       s->fmt->storage != d->fmt->storage || s->fmt->depth || src_box->width <= 0 || src_box->height <= 0) {
      FG_NOTE_ONCE("resource_copy_region between a buffer and an image, of depth, or between storage formats");
      return;
   }
   fg_copy_begin(ctx, src, dst);
   for (int z = 0; z < src_box->depth; z++)
      fg_emit_copy(ctx, src, src_level, src_box->z + z, src_box->x, src_box->y, dst, dst_level, dstz + z, dstx, dsty,
                   src_box->width, src_box->height);
   fg_copy_end(ctx, dst);
}

void
fg_copy_begin(struct fg_context *ctx, struct pipe_resource *src, struct pipe_resource *dst)
{
   /* board B copies at once and renders at a FLUSH: a pass that reads or writes either image ends first */
   if (fg_context_pending_uses(ctx, src) || fg_context_pending_uses(ctx, dst))
      fg_end_pass(ctx);
   /* the image holds what the copies write: the next pass into it keeps its contents */
   if (ctx->discard_color == dst)
      pipe_resource_reference(&ctx->discard_color, NULL);
}

void
fg_copy_end(struct fg_context *ctx, struct pipe_resource *dst)
{
   struct fg_resource *d = fg_resource(dst);

   d->initialized = true;
   fg_shadows_stale(d);
   fg_resource_written(d);
   if (!ctx->pass_open)
      fg_submit_stream(ctx);
}

static void fg_emit_fill(struct fg_context *ctx, struct pipe_resource *dst, unsigned level, unsigned layer, uint8_t tf,
                         unsigned mask, unsigned x, unsigned y, unsigned w, unsigned h, uint32_t value);

void
fg_emit_alpha_one(struct fg_context *ctx, struct pipe_resource *dst, unsigned level, unsigned layer, unsigned x,
                  unsigned y, unsigned w, unsigned h)
{
   fg_emit_fill(ctx, dst, level, layer, FG_TF_RGBA8888, 0x8, x, y, w, h, 0xFF000000u);
}

/* COPY_IMAGE: src, level, layer, dst, level, layer, source x y w h, destination x y w h, flags, colour key low (a
 * tile's phase, py << 16 | px) and high, then a list's rectangles {x, y, w, h} after the first. Both images are held
 * until the stream is submitted: the frontend often lets go of a source right after a copy */
static void
fg_emit_copy_cmd(struct fg_context *ctx, uint16_t src, unsigned slevel, unsigned slayer, unsigned sx, unsigned sy,
                 unsigned sw, unsigned sh, uint16_t dst, unsigned dlevel, unsigned dlayer, unsigned dx, unsigned dy,
                 unsigned dw, unsigned dh, uint32_t flags, uint32_t key_lo, const struct fg_copy_piece *list,
                 unsigned nlist)
{
   uint8_t b[36 + 8 * (FG_COPY_LIST_MAX - 1)];

   assert(nlist < FG_COPY_LIST_MAX);
   if (nlist)
      flags |= FG_COPY_LIST;
   for (unsigned i = 0; i < nlist; i++) {
      fg_put16(b + 36 + 8 * i, list[i].dx);
      fg_put16(b + 38 + 8 * i, list[i].dy);
      fg_put16(b + 40 + 8 * i, list[i].w);
      fg_put16(b + 42 + 8 * i, list[i].h);
   }
   fg_put16(b + 0, src);
   b[2] = (uint8_t)slevel;
   b[3] = (uint8_t)slayer;
   fg_put16(b + 4, dst);
   b[6] = (uint8_t)dlevel;
   b[7] = (uint8_t)dlayer;
   fg_put16(b + 8, sx);
   fg_put16(b + 10, sy);
   fg_put16(b + 12, sw);
   fg_put16(b + 14, sh);
   fg_put16(b + 16, dx);
   fg_put16(b + 18, dy);
   fg_put16(b + 20, dw);
   fg_put16(b + 22, dh);
   fg_put32(b + 24, flags);
   fg_put32(b + 28, key_lo);
   fg_put32(b + 32, 0);
   fg_emit(ctx, FG_T_COPY_IMAGE, b, 36 + 8 * nlist, NULL, 0);
}

static void
fg_emit_copy_handles(struct fg_context *ctx, uint16_t src, unsigned slevel, unsigned slayer, unsigned sx, unsigned sy,
                     unsigned sw, unsigned sh, uint16_t dst, unsigned dlevel, unsigned dlayer, unsigned dx, unsigned dy,
                     unsigned dw, unsigned dh, uint32_t flags)
{
   fg_emit_copy_cmd(ctx, src, slevel, slayer, sx, sy, sw, sh, dst, dlevel, dlayer, dx, dy, dw, dh, flags, 0, NULL, 0);
}

/* flags as the board takes them: ALPHA_ONE stays in the command only where board B has the copy extensions (the
 * caller fills the alpha after it otherwise) */
static uint32_t
fg_copy_board_flags(struct fg_context *ctx, uint32_t flags)
{
   if (!fg_screen(ctx->base.screen)->copy_ext) {
      assert(!(flags & (FG_COPY_SWIZZLE | FG_COPY_TILE | FG_COPY_LIST)));
      flags &= ~FG_COPY_ALPHA_ONE;
   }
   return flags;
}

static bool
fg_copy_fill_after(struct fg_context *ctx, uint32_t flags)
{
   return (flags & FG_COPY_ALPHA_ONE) && !fg_screen(ctx->base.screen)->copy_ext;
}

void
fg_emit_copy_scaled(struct fg_context *ctx, struct pipe_resource *src, unsigned slevel, unsigned slayer, unsigned sx,
                    unsigned sy, unsigned sw, unsigned sh, struct pipe_resource *dst, unsigned dlevel, unsigned dlayer,
                    unsigned dx, unsigned dy, unsigned dw, unsigned dh, uint32_t flags)
{
   fg_emit_copy_handles(ctx, fg_resource(src)->board_handle, slevel, slayer, sx, sy, sw, sh,
                        fg_resource(dst)->board_handle, dlevel, dlayer, dx, dy, dw, dh, fg_copy_board_flags(ctx, flags));
   fg_use_resource(ctx, src);
   fg_use_resource(ctx, dst);
   if (fg_copy_fill_after(ctx, flags))
      fg_emit_alpha_one(ctx, dst, dlevel, dlayer, dx, dy, dw, dh);
}

void
fg_emit_copies(struct fg_context *ctx, struct pipe_resource *src, struct pipe_resource *dst,
               const struct fg_copy_piece *p, unsigned n, uint32_t flags)
{
   const bool ext = fg_screen(ctx->base.screen)->copy_ext;
   const uint16_t sh = fg_resource(src)->board_handle, dh = fg_resource(dst)->board_handle;
   const uint32_t bf = fg_copy_board_flags(ctx, flags);

   for (unsigned i = 0; i < n;) {
      unsigned k = 1;

      if (p[i].tile) {
         assert(ext);
         fg_emit_copy_cmd(ctx, sh, 0, 0, p[i].sx, p[i].sy, p[i].tw, p[i].th, dh, 0, 0, p[i].dx, p[i].dy, p[i].w,
                          p[i].h, bf | FG_COPY_TILE, (uint32_t)p[i].py << 16 | (uint32_t)p[i].px, NULL, 0);
      } else {
         /* the run of rectangles after it at its offset goes in its list */
         while (ext && i + k < n && k < FG_COPY_LIST_MAX && !p[i + k].tile &&
                p[i + k].sx - p[i + k].dx == p[i].sx - p[i].dx && p[i + k].sy - p[i + k].dy == p[i].sy - p[i].dy)
            k++;
         if (ctx->debug_copy && k > 1)
            fprintf(stderr, "fpgagpu: one COPY_IMAGE with a list: %u rectangles\n", k);
         fg_emit_copy_cmd(ctx, sh, 0, 0, p[i].sx, p[i].sy, p[i].w, p[i].h, dh, 0, 0, p[i].dx, p[i].dy, p[i].w, p[i].h,
                          bf, 0, p + i + 1, k - 1);
      }
      if (fg_copy_fill_after(ctx, flags))
         fg_emit_alpha_one(ctx, dst, 0, 0, p[i].dx, p[i].dy, p[i].w, p[i].h);
      i += k;
   }
   fg_use_resource(ctx, src);
   fg_use_resource(ctx, dst);
}

bool
fg_copy_channels(struct fg_context *ctx, const struct fg_format *sf, const struct fg_format *vf,
                 const struct fg_format *df, const unsigned char sw[4], uint32_t *flags)
{
   const bool rgba8 = df->storage == FG_FMT_RGBA8;
   uint32_t sel = 0;
   bool ident = true;

   for (unsigned c = 0; c < 4; c++) {
      unsigned s = sw[c];

      /* the view reads W as 1.0 (an X format) where the stored alpha may be anything */
      if (s == PIPE_SWIZZLE_W && vf->force_alpha && !sf->force_alpha)
         s = PIPE_SWIZZLE_1;
      if (c == 3) {
         if (!rgba8)
            s = PIPE_SWIZZLE_W;          /* RGB565 stores no alpha */
         else if (df->force_alpha)
            s = PIPE_SWIZZLE_1;          /* an X image's stored alpha is kept at 1.0 */
         if (s == PIPE_SWIZZLE_1) {
            if (!sf->force_alpha)        /* (an X source's stored alpha is 1.0 already) */
               *flags |= FG_COPY_ALPHA_ONE;
            s = PIPE_SWIZZLE_W;
         }
      }
      if (s > PIPE_SWIZZLE_W)
         return false;
      sel |= s << (8 + 2 * c);
      ident = ident && s == c;
   }
   if (!ident) {
      /* an expanded format's stored channels other than its own hold fixed values a swizzle would overwrite */
      if (!fg_screen(ctx->base.screen)->copy_ext || sf->expand || df->expand)
         return false;
      *flags |= FG_COPY_SWIZZLE | sel;
   }
   return true;
}

/* the handle the TMU reads an image by. The TMU reads tight rows: an image whose rows are padded is read from its
 * tight-row copy, taken here when the image changed (every pass for a shared image). Board B copies at once and
 * renders at the FLUSH, so the copy lands under the pass's draws. A sampled D24S8 image is copied through its RGBA8
 * alias into an RGBA8 copy, which the fragment program reads the same way. snapshot: the image is the pass's target,
 * which board B rewrites band by band while the pass's fragments read it: they read that copy (taken once a pass, so
 * what the image held before the pass), whatever its rows, and a draw that reads its own target overlapping where it
 * writes gives what GL_MESA_tile_raster_order promises in any raster order */
uint16_t
fg_sample_handle(struct fg_context *ctx, struct pipe_resource *prsc, bool snapshot)
{
   struct fg_resource *r = fg_resource(prsc), *t;
   uint16_t src = r->sample_handle ? r->sample_handle : r->board_handle;
   uint32_t seq = p_atomic_read(&fg_screen(ctx->base.screen)->pass_seq);

   if (!snapshot && r->pitch == prsc->width0 * fg_storage_bpp(r->fmt))
      return src;
   if (!r->tight) {
      struct pipe_resource templ = *prsc;

      templ.target = PIPE_TEXTURE_2D;
      templ.format = r->sample_handle ? PIPE_FORMAT_R8G8B8A8_UNORM : prsc->format;
      templ.bind = PIPE_BIND_SAMPLER_VIEW;
      templ.last_level = 0;
      templ.array_size = 1;
      r->tight = prsc->screen->resource_create(prsc->screen, &templ);
      if (!r->tight)
         return src;
      r->tight_stale = true;
   }
   t = fg_resource(r->tight);
   if (r->tight_stale || (r->shared && r->tight_seq != seq)) {
      fg_emit_copy_handles(ctx, src, 0, 0, 0, 0, prsc->width0, prsc->height0, t->board_handle, 0, 0, 0, 0,
                           prsc->width0, prsc->height0, 0);
      fg_use_resource(ctx, prsc);
      r->tight_stale = false;
      r->tight_seq = seq;
      t->initialized = true;
   }
   fg_use_resource(ctx, r->tight);
   return t->board_handle;
}

uint16_t
fg_level_handle(struct fg_context *ctx, struct pipe_resource *prsc, unsigned first, unsigned last)
{
   struct fg_resource *r = fg_resource(prsc), *v;
   uint32_t seq = p_atomic_read(&fg_screen(ctx->base.screen)->pass_seq);
   uint16_t src = r->sample_handle ? r->sample_handle : r->board_handle;
   unsigned n;

   last = MIN2(last, prsc->last_level);
   first = MIN2(first, last);
   n = last - first + 1;
   if (r->lview && (r->lview_first != first || r->lview_n != n))
      pipe_resource_reference(&r->lview, NULL);
   if (!r->lview) {
      struct pipe_resource templ = *prsc;

      /* a D24S8 image's levels go through its RGBA8 alias into RGBA8, as the TMU reads them */
      templ.format = r->sample_handle ? PIPE_FORMAT_R8G8B8A8_UNORM : prsc->format;
      templ.width0 = u_minify(prsc->width0, first);
      templ.height0 = u_minify(prsc->height0, first);
      templ.last_level = n - 1;
      templ.bind = PIPE_BIND_SAMPLER_VIEW;
      r->lview = prsc->screen->resource_create(prsc->screen, &templ);
      if (!r->lview)
         return src;
      r->lview_first = first;
      r->lview_n = n;
      r->lview_stale = true;
   }
   v = fg_resource(r->lview);
   if (r->lview_stale || (r->shared && r->lview_seq != seq)) {
      for (unsigned z = 0; z < prsc->array_size; z++) {
         for (unsigned l = 0; l < n; l++) {
            unsigned w = u_minify(prsc->width0, first + l), h = u_minify(prsc->height0, first + l);

            fg_emit_copy_handles(ctx, src, first + l, z, 0, 0, w, h, v->board_handle, l, z, 0, 0, w, h, 0);
         }
      }
      fg_use_resource(ctx, prsc);
      r->lview_stale = false;
      r->lview_seq = seq;
      v->initialized = true;
   }
   fg_use_resource(ctx, r->lview);
   return v->board_handle;
}

void
fg_emit_copy(struct fg_context *ctx, struct pipe_resource *src, unsigned slevel, unsigned slayer, unsigned sx,
             unsigned sy, struct pipe_resource *dst, unsigned dlevel, unsigned dlayer, unsigned dx, unsigned dy,
             unsigned w, unsigned h)
{
   fg_emit_copy_scaled(ctx, src, slevel, slayer, sx, sy, w, h, dst, dlevel, dlayer, dx, dy, w, h, 0);
}

/* Mesa's blitter saves the bound state, binds its own and gives the saved back after it */
static void
fg_blitter_save(struct fg_context *ctx)
{
   util_blitter_save_blend(ctx->blitter, (void *)ctx->blend);
   util_blitter_save_depth_stencil_alpha(ctx->blitter, (void *)ctx->dsa);
   util_blitter_save_stencil_ref(ctx->blitter, &ctx->stencil_ref);
   util_blitter_save_rasterizer(ctx->blitter, (void *)ctx->rast);
   util_blitter_save_fragment_shader(ctx->blitter, ctx->fs);
   util_blitter_save_vertex_shader(ctx->blitter, ctx->vs);
   util_blitter_save_viewport(ctx->blitter, &ctx->viewport);
   util_blitter_save_scissor(ctx->blitter, &ctx->scissor);
   util_blitter_save_vertex_elements(ctx->blitter, (void *)ctx->velems);
   util_blitter_save_vertex_buffers(ctx->blitter, ctx->vb, ctx->nvb);
   util_blitter_save_framebuffer(ctx->blitter, &ctx->fb);
   util_blitter_save_fragment_sampler_states(ctx->blitter, FG_MAX_SAMPLERS, (void **)ctx->samplers);
   util_blitter_save_fragment_sampler_views(ctx->blitter, FG_MAX_SAMPLERS, ctx->views);
}

/* a blit drawn: Mesa's blitter renders the source through the TMU (its level a copy of its own, fg_level_handle)
 * into the destination's level and layer (a shadow, fg_set_target), with linear filtering, a conversion between
 * storage formats, blending, a colour mask or a swizzle. false: it cannot (a destination wider than board B renders) */
static bool
fg_blit_draw(struct fg_context *ctx, const struct pipe_blit_info *info)
{
   uint8_t cb[2][sizeof(ctx->cb[0])];
   uint32_t cb_size[2];

   if (!ctx->blitter || u_minify(info->dst.resource->width0, info->dst.level) > FG_FLUSH_MAX_WIDTH ||
       !util_blitter_is_blit_supported(ctx->blitter, info))
      return false;
   /* (the constant buffers are held as copies: the blitter gives back only what it saved) */
   memcpy(cb, ctx->cb, sizeof(cb));
   memcpy(cb_size, ctx->cb_size, sizeof(cb_size));
   fg_blitter_save(ctx);
   util_blitter_blit(ctx->blitter, info, NULL);
   memcpy(ctx->cb, cb, sizeof(cb));
   memcpy(ctx->cb_size, cb_size, sizeof(cb_size));
   return true;
}

/* a blit as board B's COPY_IMAGE where it can: images of the same storage, a nearest stretch, mirrors (negative box
 * sizes), the scissor when the blit does not stretch, the alpha of an X destination kept at 1.0, a swizzle where board
 * B has the copy extensions; drawn otherwise (fg_blit_draw). Depth and stencil are copied only */
static void
fg_blit(struct pipe_context *pctx, const struct pipe_blit_info *info)
{
   struct fg_context *ctx = fg_context(pctx);
   struct pipe_resource *src = info->src.resource, *dst = info->dst.resource;
   struct fg_resource *s = fg_resource(src), *d = fg_resource(dst);
   int sx = info->src.box.x, sy = info->src.box.y, sw = info->src.box.width, sh = info->src.box.height;
   int dx = info->dst.box.x, dy = info->dst.box.y, dw = info->dst.box.width, dh = info->dst.box.height;
   uint32_t flags = 0;

   if (src->target == PIPE_BUFFER || dst->target == PIPE_BUFFER || !s->fmt || !d->fmt ||
       info->src.box.depth != info->dst.box.depth || !sw || !sh || !dw || !dh) {
      FG_NOTE_ONCE("a blit of a buffer, or of nothing");
      return;
   }
   if (s->fmt->depth || d->fmt->depth) {
      /* (Z16 and Z24S8 share their storage, not what a word means) */
      if (s->fmt != d->fmt || info->alpha_blend) {
         FG_NOTE_ONCE("a blit of depth or stencil between formats");
         return;
      }
      if ((info->mask & PIPE_MASK_ZS) != (util_format_has_stencil(util_format_description(src->format)) ?
                                          PIPE_MASK_ZS : PIPE_MASK_Z)) {
         FG_NOTE_ONCE("a blit of depth or stencil alone out of a combined image");
         return;
      }
   } else {
      const struct fg_format *vf = fg_format_info(info->src.format);
      unsigned char swz[4] = { PIPE_SWIZZLE_X, PIPE_SWIZZLE_Y, PIPE_SWIZZLE_Z, PIPE_SWIZZLE_W };
      const bool stretch = abs(sw) != abs(dw) || abs(sh) != abs(dh);
      bool channels, same;

      if (!vf || vf->storage != s->fmt->storage)
         vf = s->fmt;
      if (info->swizzle_enable)
         memcpy(swz, info->swizzle, sizeof(swz));
      channels = fg_copy_channels(ctx, s->fmt, vf, d->fmt, swz, &flags);
      same = s->fmt->storage == d->fmt->storage && !info->alpha_blend;
      if ((!same || !channels || (info->mask & PIPE_MASK_RGBA) != PIPE_MASK_RGBA ||
           (stretch && (info->filter != PIPE_TEX_FILTER_NEAREST || info->scissor_enable))) &&
          fg_blit_draw(ctx, info))
         return;
      if (!same || !channels) {
         FG_NOTE_ONCE("a blit neither copied nor drawn (between storage formats, blended or swizzled)");
         return;
      }
      if ((info->mask & PIPE_MASK_RGBA) != PIPE_MASK_RGBA)
         FG_NOTE_ONCE("a blit's colour mask is not applied");
   }
   if (sw < 0) {
      sx += sw;
      sw = -sw;
      flags ^= FG_COPY_MIRROR_X;
   }
   if (sh < 0) {
      sy += sh;
      sh = -sh;
      flags ^= FG_COPY_MIRROR_Y;
   }
   if (dw < 0) {
      dx += dw;
      dw = -dw;
      flags ^= FG_COPY_MIRROR_X;
   }
   if (dh < 0) {
      dy += dh;
      dh = -dh;
      flags ^= FG_COPY_MIRROR_Y;
   }
   if (info->scissor_enable) {
      const struct pipe_scissor_state *c = &info->scissor;
      int l = MAX2((int)c->minx - dx, 0), t = MAX2((int)c->miny - dy, 0);
      int r = MAX2(dx + dw - (int)c->maxx, 0), b = MAX2(dy + dh - (int)c->maxy, 0);

      if (sw != dw || sh != dh) {
         FG_NOTE_ONCE("a stretching blit's scissor is not applied");
      } else if (l || t || r || b) {
         if (l + r >= dw || t + b >= dh)
            return;
         sx += (flags & FG_COPY_MIRROR_X) ? r : l;
         sy += (flags & FG_COPY_MIRROR_Y) ? b : t;
         dx += l;
         dy += t;
         dw -= l + r;
         dh -= t + b;
         sw = dw;
         sh = dh;
      }
   }
   if (info->filter != PIPE_TEX_FILTER_NEAREST && (sw != dw || sh != dh))
      FG_NOTE_ONCE("a linear stretching blit is done nearest");
   fg_copy_begin(ctx, src, dst);
   for (int z = 0; z < info->dst.box.depth; z++)
      fg_emit_copy_scaled(ctx, src, info->src.level, info->src.box.z + z, sx, sy, sw, sh, dst, info->dst.level,
                          info->dst.box.z + z, dx, dy, dw, dh, flags);
   fg_copy_end(ctx, dst);
}

/* FILL_IMAGE of a rectangle of a level and layer: value in the transfer format tf, channel mask (R 1, G 2, B 4, A 8;
 * in a D24S8 word R is the stencil byte and G, B, A the depth) */
static void
fg_emit_fill(struct fg_context *ctx, struct pipe_resource *dst, unsigned level, unsigned layer, uint8_t tf,
             unsigned mask, unsigned x, unsigned y, unsigned w, unsigned h, uint32_t value)
{
   uint8_t b[20] = { 0 };

   fg_put16(b, fg_resource(dst)->board_handle);
   b[2] = (uint8_t)level;
   b[3] = (uint8_t)layer;
   b[4] = tf;
   b[5] = (uint8_t)mask;
   fg_put16(b + 8, x);
   fg_put16(b + 10, y);
   fg_put16(b + 12, w);
   fg_put16(b + 14, h);
   fg_put32(b + 16, value);
   fg_emit(ctx, FG_T_FILL_IMAGE, b, sizeof(b), NULL, 0);
   fg_use_resource(ctx, dst);
}

/* a colour as FILL_IMAGE takes it for an image of format f: RGBA8 bytes (R, G, B, A) as the format reads them (an X
 * format's alpha 1.0, the fixed channels of one kept as RGBA8), or RGB565 rounded to its bits */
static uint32_t
fg_fill_color(const struct fg_format *f, const float c[4], uint8_t *tf)
{
   uint8_t v[4];

   if (f->storage == FG_FMT_RGB565) {
      uint16_t p = 0;

      util_format_pack_rgba(PIPE_FORMAT_B5G6R5_UNORM, &p, c, 1);
      *tf = FG_TF_RGB565;
      return p;
   }
   for (unsigned i = 0; i < 4; i++)
      v[i] = float_to_ubyte(c[i]);
   switch (f->expand) {
   case FG_EXPAND_L: v[1] = v[2] = v[0]; v[3] = 255; break;
   case FG_EXPAND_A: v[0] = v[1] = v[2] = 0; break;
   case FG_EXPAND_LA: v[1] = v[2] = v[0]; break;
   case FG_EXPAND_R: v[1] = v[2] = 0; v[3] = 255; break;
   case FG_EXPAND_RG: v[2] = 0; v[3] = 255; break;
   default:
      if (f->force_alpha)
         v[3] = 255;
      break;
   }
   *tf = FG_TF_RGBA8888;
   return (uint32_t)v[0] | (uint32_t)v[1] << 8 | (uint32_t)v[2] << 16 | (uint32_t)v[3] << 24;
}

/* a rectangle of a colour image's level and layers as one colour: FILL_IMAGE */
static void
fg_fill_rect(struct fg_context *ctx, struct pipe_resource *prsc, unsigned level, unsigned first_layer,
             unsigned last_layer, unsigned x, unsigned y, unsigned w, unsigned h, const float c[4])
{
   const struct fg_format *f = fg_resource(prsc)->fmt;
   uint8_t tf;
   uint32_t v;

   if (!f || f->depth || !w || !h)
      return;
   v = fg_fill_color(f, c, &tf);
   fg_copy_begin(ctx, prsc, prsc);
   for (unsigned z = first_layer; z <= last_layer; z++)
      fg_emit_fill(ctx, prsc, level, z, tf, 0xF, x, y, w, h, v);
   fg_copy_end(ctx, prsc);
}

/* a rectangle of a depth-stencil image's level and layers: depth (clamped to [0, 1], rounded to its bits) and / or
 * stencil, FILL_IMAGE of the raw word with the channels of each */
static void
fg_fill_ds(struct fg_context *ctx, struct pipe_resource *z, unsigned level, unsigned first_layer, unsigned last_layer,
           unsigned flags, double depth, unsigned stencil, unsigned x, unsigned y, unsigned w, unsigned h)
{
   const struct fg_format *f = fg_resource(z)->fmt;
   double d = depth > 0.0 ? (depth < 1.0 ? depth : 1.0) : 0.0;
   uint32_t v = 0;
   unsigned mask = 0;

   if (!f || !f->depth || !w || !h)
      return;
   if (f->expand == FG_EXPAND_Z16) {
      /* (a D24S8 word whose depth DEPTH16 keeps as 0 .. 65535; no stencil) */
      if (flags & PIPE_CLEAR_DEPTH) {
         v = (uint32_t)rint(d * 65535.0);
         mask = 0xE;
      }
   } else {
      if (flags & PIPE_CLEAR_DEPTH) {
         v |= (uint32_t)rint(d * 16777215.0);
         mask |= 0xE;
      }
      if (flags & PIPE_CLEAR_STENCIL) {
         v |= (stencil & 0xFFu) << 24;
         mask |= 0x1;
      }
   }
   if (!mask)
      return;
   fg_copy_begin(ctx, z, z);
   for (unsigned l = first_layer; l <= last_layer; l++)
      fg_emit_fill(ctx, z, level, l, FG_TF_RAW, mask, x, y, w, h, v);
   fg_copy_end(ctx, z);
   /* the stencil value the next whole-image depth clear keeps: known only after a fill of the whole image */
   if ((mask & 1) && !x && !y && w >= u_minify(z->width0, level) && h >= u_minify(z->height0, level) && !level &&
       !first_layer && z->array_size == 1) {
      pipe_resource_reference(&ctx->stencil_ds, z);
      ctx->stencil_known = true;
      ctx->stencil_value = stencil & 0xFF;
   } else if ((mask & 1) && z == ctx->stencil_ds) {
      ctx->stencil_known = false;
   }
}

static void
fg_clear_render_target(struct pipe_context *pctx, struct pipe_surface *dst, const union pipe_color_union *color,
                       unsigned dstx, unsigned dsty, unsigned width, unsigned height, bool render_condition_enabled)
{
   fg_fill_rect(fg_context(pctx), dst->texture, dst->level, dst->first_layer, dst->last_layer, dstx, dsty, width, height,
                color->f);
}

static void
fg_clear_depth_stencil(struct pipe_context *pctx, struct pipe_surface *dst, unsigned clear_flags, double depth,
                       unsigned stencil, unsigned dstx, unsigned dsty, unsigned width, unsigned height,
                       bool render_condition_enabled)
{
   fg_fill_ds(fg_context(pctx), dst->texture, dst->level, dst->first_layer, dst->last_layer, clear_flags, depth, stencil,
              dstx, dsty, width, height);
}

/* data: one texel in the image's format */
static void
fg_clear_texture(struct pipe_context *pctx, struct pipe_resource *res, unsigned level, const struct pipe_box *box,
                 const void *data)
{
   struct fg_context *ctx = fg_context(pctx);
   const struct fg_format *f = fg_resource(res)->fmt;
   const struct util_format_description *desc = util_format_description(res->format);

   if (!f || res->target == PIPE_BUFFER || box->depth < 1)
      return;
   if (f->depth) {
      float z = 0.0f;
      uint8_t s = 0;
      unsigned flags = 0;

      if (util_format_has_depth(desc)) {
         util_format_unpack_z_float(res->format, &z, data, 1);
         flags |= PIPE_CLEAR_DEPTH;
      }
      if (util_format_has_stencil(desc)) {
         util_format_unpack_s_8uint(res->format, &s, data, 1);
         flags |= PIPE_CLEAR_STENCIL;
      }
      fg_fill_ds(ctx, res, level, box->z, box->z + box->depth - 1, flags, z, s, box->x, box->y, box->width,
                 box->height);
   } else {
      float c[4];

      util_format_unpack_rgba(res->format, c, data, 1);
      fg_fill_rect(ctx, res, level, box->z, box->z + box->depth - 1, box->x, box->y, box->width, box->height, c);
   }
}

static bool
fg_generate_mipmap(struct pipe_context *pctx, struct pipe_resource *prsc, enum pipe_format format,
                   unsigned base_level, unsigned last_level, unsigned first_layer, unsigned last_layer)
{
   return false;
}

static void
fg_flush_resource(struct pipe_context *pctx, struct pipe_resource *prsc)
{
}

static void
fg_resource_release(struct pipe_context *pctx, struct pipe_resource *prsc)
{
   pipe_resource_reference(&prsc, NULL);
}

/* ------------------------------------------------------------------ queries
 * Occlusion: board B counts quads and frames, not samples. A query's result is 1 when a draw ran inside it, else 0:
 * never 0 where samples passed, so a program that culls by it draws too much, never too little (the Windows
 * libraries' rule; GL lets a counter carry no information). GPU_FINISHED: the context's work done. Others: 0 */

struct fg_query {
   unsigned type;
   uint64_t draws0, draws;
   uint64_t fence;
   bool fenced;
};

static struct pipe_query *
fg_create_query(struct pipe_context *pctx, unsigned query_type, unsigned index)
{
   struct fg_query *q = CALLOC_STRUCT(fg_query);

   if (q)
      q->type = query_type;
   return (struct pipe_query *)q;
}

static void
fg_destroy_query(struct pipe_context *pctx, struct pipe_query *q)
{
   FREE(q);
}

static bool
fg_begin_query(struct pipe_context *pctx, struct pipe_query *pq)
{
   struct fg_query *q = (struct fg_query *)pq;

   q->draws0 = fg_context(pctx)->draw_count;
   q->draws = 0;
   return true;
}

static bool
fg_end_query(struct pipe_context *pctx, struct pipe_query *pq)
{
   struct fg_context *ctx = fg_context(pctx);
   struct fg_query *q = (struct fg_query *)pq;

   q->draws = ctx->draw_count - q->draws0;
   if (q->type == PIPE_QUERY_GPU_FINISHED) {
      fg_context_flush_pending(ctx);
      q->fenced = ctx->submitted;
      q->fence = ctx->last_fence;
   }
   return true;
}

static bool
fg_get_query_result(struct pipe_context *pctx, struct pipe_query *pq, bool wait, union pipe_query_result *result)
{
   struct fg_query *q = (struct fg_query *)pq;

   switch (q->type) {
   case PIPE_QUERY_OCCLUSION_COUNTER:
      result->u64 = q->draws ? 1 : 0;
      return true;
   case PIPE_QUERY_OCCLUSION_PREDICATE:
   case PIPE_QUERY_OCCLUSION_PREDICATE_CONSERVATIVE:
      result->b = q->draws != 0;
      return true;
   case PIPE_QUERY_GPU_FINISHED:
      if (q->fenced && fg_wait_fence(fg_screen(pctx->screen), q->fence, wait ? FG_WAIT_NS : 0))
         return false;
      result->b = true;
      return true;
   default:
      memset(result, 0, sizeof(*result));
      return true;
   }
}

/* the blitter's own draws (enable false) are not the application's */
static void
fg_set_active_query_state(struct pipe_context *pctx, bool enable)
{
   fg_context(pctx)->queries_off = !enable;
}

static void
fg_render_condition(struct pipe_context *pctx, struct pipe_query *query, bool condition,
                    enum pipe_render_cond_flag mode)
{
}

/* ------------------------------------------------------------------ state with nothing to send */

static void
fg_set_clip_state(struct pipe_context *pctx, const struct pipe_clip_state *state)
{
}

static void
fg_set_inlinable_constants(struct pipe_context *pctx, mesa_shader_stage shader, uint num_values, uint32_t *values)
{
}

static void
fg_set_polygon_stipple(struct pipe_context *pctx, const struct pipe_poly_stipple *state)
{
}

static void
fg_set_sample_mask(struct pipe_context *pctx, unsigned sample_mask)
{
}

static void
fg_set_window_rectangles(struct pipe_context *pctx, bool include, unsigned num_rectangles,
                         const struct pipe_scissor_state *rects)
{
}

static void
fg_set_shader_buffers(struct pipe_context *pctx, mesa_shader_stage shader, unsigned start_slot, unsigned count,
                      const struct pipe_shader_buffer *buffers, unsigned writable_bitmask)
{
}

static void
fg_set_shader_images(struct pipe_context *pctx, mesa_shader_stage shader, unsigned start_slot, unsigned count,
                     unsigned unbind_num_trailing_slots, const struct pipe_image_view *images)
{
}

static struct pipe_stream_output_target *
fg_create_stream_output_target(struct pipe_context *pctx, struct pipe_resource *res, unsigned buffer_offset,
                               unsigned buffer_size)
{
   struct pipe_stream_output_target *t = CALLOC_STRUCT(pipe_stream_output_target);

   if (!t)
      return NULL;
   pipe_reference_init(&t->reference, 1);
   pipe_resource_reference(&t->buffer, res);
   t->buffer_offset = buffer_offset;
   t->buffer_size = buffer_size;
   return t;
}

static void
fg_stream_output_target_destroy(struct pipe_context *pctx, struct pipe_stream_output_target *t)
{
   pipe_resource_reference(&t->buffer, NULL);
   FREE(t);
}

static void
fg_set_stream_output_targets(struct pipe_context *pctx, unsigned num_targets,
                             struct pipe_stream_output_target **targets, const unsigned *offsets,
                             enum mesa_prim output_prim)
{
}

/* what the draws before it wrote is to be read after it (GL_NV_texture_barrier): board B renders a pass at its FLUSH,
 * so the pass ends here */
static void
fg_texture_barrier(struct pipe_context *pctx, unsigned flags)
{
   fg_end_pass(fg_context(pctx));
}

static void
fg_memory_barrier(struct pipe_context *pctx, unsigned flags)
{
}

static void
fg_set_context_param(struct pipe_context *pctx, enum pipe_context_param param, unsigned value)
{
}

static void
fg_set_frontend_noop(struct pipe_context *pctx, bool enable)
{
}

static enum pipe_reset_status
fg_get_device_reset_status(struct pipe_context *pctx)
{
   return PIPE_NO_RESET;
}

static void
fg_get_sample_position(struct pipe_context *pctx, unsigned sample_count, unsigned sample_index, float *out_value)
{
   out_value[0] = out_value[1] = 0.5f;
}

/* ------------------------------------------------------------------ context */

static void
fg_context_destroy(struct pipe_context *pctx)
{
   struct fg_context *ctx = fg_context(pctx);
   struct fg_screen *screen = fg_screen(pctx->screen);

   fg_context_flush_pending(ctx);
   if (screen->last_ctx == ctx)
      screen->last_ctx = NULL;
   if (ctx->blitter)
      util_blitter_destroy(ctx->blitter);
   fg_draw_fini(ctx);
   util_unreference_framebuffer_state(&ctx->fb);
   fg_target_hold(ctx, NULL);
   pipe_resource_reference(&ctx->discard_color, NULL);
   pipe_resource_reference(&ctx->stencil_ds, NULL);
   if (pctx->stream_uploader)
      u_upload_destroy(pctx->stream_uploader);
   fg_ctx_bo_destroy(ctx, &ctx->prog_a);
   fg_ctx_bo_destroy(ctx, &ctx->prog_b);
   fg_ctx_bo_destroy(ctx, &ctx->ring);
   free(ctx->cmd);
   free(ctx->bos);
   free(ctx->pass_res);
   p_atomic_dec(&pctx->screen->num_contexts);
   FREE(ctx);
}

struct pipe_context *
fg_context_create(struct pipe_screen *pscreen, void *priv, unsigned flags)
{
   struct fg_context *ctx = CALLOC_STRUCT(fg_context);
   struct pipe_context *pctx;
   int64_t prog_kb;

   if (!ctx)
      return NULL;
   pctx = &ctx->base;
   pctx->screen = pscreen;
   pctx->priv = priv;
   ctx->debug = fg_screen(pscreen)->debug;
   ctx->no_region = debug_get_bool_option("FPGAGPU_NOREGION", false);
   ctx->no_copy = debug_get_bool_option("FPGAGPU_NOCOPY", false);
   ctx->debug_copy = debug_get_bool_option("FPGAGPU_DEBUG_COPY", false);
   ctx->flush_depth = MIN2(debug_get_num_option("FPGAGPU_FLUSH_DEPTH", 4), FG_FLUSH_RING - 1);
   ctx->cl_depth = 0xFFFFFF;
   ctx->cl_smask = 0xFF;
   ctx->next_id = 1;

   /* (FPGAGPU_PROG_KB: program buffers of that many KB, to see them fill and start again) */
   prog_kb = debug_get_num_option("FPGAGPU_PROG_KB", 0);
   if (!fg_ctx_bo_create(ctx, &ctx->prog_a, FPGAGPU_BO_A_BUFFER, prog_kb > 0 ? prog_kb * 1024 : FG_PROG_A_BYTES) ||
       !fg_ctx_bo_create(ctx, &ctx->prog_b, FPGAGPU_BO_B_BUFFER, prog_kb > 0 ? prog_kb * 1024 : FG_PROG_B_BYTES) ||
       !fg_ctx_bo_create(ctx, &ctx->ring, FPGAGPU_BO_A_BUFFER, FG_RING_BYTES)) {
      fg_ctx_bo_destroy(ctx, &ctx->prog_a);
      fg_ctx_bo_destroy(ctx, &ctx->prog_b);
      fg_ctx_bo_destroy(ctx, &ctx->ring);
      FREE(ctx);
      return NULL;
   }

   pctx->stream_uploader = u_upload_create_default(pctx);
   if (!pctx->stream_uploader) {
      fg_ctx_bo_destroy(ctx, &ctx->prog_a);
      fg_ctx_bo_destroy(ctx, &ctx->prog_b);
      fg_ctx_bo_destroy(ctx, &ctx->ring);
      FREE(ctx);
      return NULL;
   }
   pctx->const_uploader = pctx->stream_uploader;

   pctx->destroy = fg_context_destroy;
   pctx->flush = fg_flush;
   pctx->clear = fg_clear;
   pctx->clear_render_target = fg_clear_render_target;
   pctx->clear_depth_stencil = fg_clear_depth_stencil;
   pctx->clear_texture = fg_clear_texture;
   pctx->clear_buffer = u_default_clear_buffer;
   pctx->resource_copy_region = fg_resource_copy_region;
   pctx->blit = fg_blit;
   pctx->generate_mipmap = fg_generate_mipmap;
   pctx->flush_resource = fg_flush_resource;
   pctx->invalidate_resource = fg_invalidate_resource;
   pctx->resource_release = fg_resource_release;

   pctx->create_query = fg_create_query;
   pctx->destroy_query = fg_destroy_query;
   pctx->begin_query = fg_begin_query;
   pctx->end_query = fg_end_query;
   pctx->get_query_result = fg_get_query_result;
   pctx->set_active_query_state = fg_set_active_query_state;
   pctx->render_condition = fg_render_condition;

   pctx->set_framebuffer_state = fg_set_framebuffer_state;
   pctx->set_clip_state = fg_set_clip_state;
   pctx->set_inlinable_constants = fg_set_inlinable_constants;
   pctx->set_polygon_stipple = fg_set_polygon_stipple;
   pctx->set_sample_mask = fg_set_sample_mask;
   pctx->set_window_rectangles = fg_set_window_rectangles;
   pctx->set_shader_buffers = fg_set_shader_buffers;
   pctx->set_shader_images = fg_set_shader_images;
   pctx->create_stream_output_target = fg_create_stream_output_target;
   pctx->stream_output_target_destroy = fg_stream_output_target_destroy;
   pctx->set_stream_output_targets = fg_set_stream_output_targets;
   pctx->texture_barrier = fg_texture_barrier;
   pctx->memory_barrier = fg_memory_barrier;
   pctx->set_context_param = fg_set_context_param;
   pctx->set_frontend_noop = fg_set_frontend_noop;
   pctx->get_device_reset_status = fg_get_device_reset_status;
   pctx->get_sample_position = fg_get_sample_position;

   fg_resource_context_init(pctx);
   fg_draw_init(pctx);
   /* (after every entry point is set: the blitter makes its state objects here) */
   ctx->blitter = util_blitter_create(pctx);
   if (!ctx->blitter)
      fprintf(stderr, "fpgagpu: no blitter: blits that are no copy are left out\n");

   p_atomic_inc(&pscreen->num_contexts);
   return pctx;
}
