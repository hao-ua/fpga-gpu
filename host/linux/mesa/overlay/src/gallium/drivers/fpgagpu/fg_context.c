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

/* ------------------------------------------------------------------ passes */

/* FLUSH: {target, flags}, clear colour {R, G} and {B, A} (13 bits each), {depth 24, stencil 8}, the clear's
 * scissor {x0, y0}, {x1, y1}, {depth-stencil image, stencil write mask} */
static void
fg_emit_flush(struct fg_context *ctx, uint32_t flags)
{
   uint8_t b[28];

   fg_put32(b + 0, (uint32_t)ctx->tgt.color_handle | (flags & 0xFFFFu) << 16);
   fg_put32(b + 4, (ctx->cl_c16[0] & 0x1FFFu) | (ctx->cl_c16[1] & 0x1FFFu) << 13);
   fg_put32(b + 8, (ctx->cl_c16[2] & 0x1FFFu) | (ctx->cl_c16[3] & 0x1FFFu) << 13);
   fg_put32(b + 12, (ctx->cl_depth & 0xFFFFFFu) | (ctx->cl_stencil & 0xFFu) << 24);
   if (flags & FG_FL_CLEAR_SCISSOR) {
      fg_put32(b + 16, (ctx->cl_sc[0] & 0x1FFFu) | (ctx->cl_sc[1] & 0x1FFFu) << 16);
      fg_put32(b + 20, (ctx->cl_sc[2] & 0x1FFFu) | (ctx->cl_sc[3] & 0x1FFFu) << 16);
   } else {
      fg_put32(b + 16, 0);
      fg_put32(b + 20, 0);
   }
   fg_put32(b + 24, (uint32_t)ctx->tgt.ds_handle | (ctx->cl_smask & 0xFFu) << 16);
   fg_emit(ctx, FG_T_FLUSH, b, sizeof(b), NULL, 0);
   fg_use_resource(ctx, ctx->tgt.color);
   if (ctx->tgt.ds)
      fg_use_resource(ctx, ctx->tgt.ds);
}

/* the boards' render state for this process until the pass ends (another process's pass would land in the middle of
 * one that takes several SUBMITs); what another process sent since this one last held it replaced its state */
static void
fg_pass_take(struct fg_context *ctx, bool take)
{
   struct drm_fpgagpu_pass p = {
      .op = take ? FPGAGPU_PASS_BEGIN : FPGAGPU_PASS_END,
      .timeout_ns = 5ll * 1000 * 1000 * 1000,
   };

   if (drmIoctl(fg_screen(ctx->base.screen)->fd, DRM_IOCTL_FPGAGPU_PASS, &p))
      return;
   if (take && p.state_lost)
      fg_hw_invalidate(ctx);
}

void
fg_pass_open(struct fg_context *ctx)
{
   uint8_t z[4] = { 0, 0, 0, 0 };

   if (ctx->pass_open)
      return;
   fg_pass_take(ctx, true);
   fg_emit(ctx, FG_T_FRAME_BEGIN, z, sizeof(z), NULL, 0);
   ctx->pass_open = true;
   ctx->pass_draws = 0;
}

void fg_send_target_size(struct fg_context *ctx);

void
fg_end_pass(struct fg_context *ctx)
{
   uint32_t fl;

   if (!ctx->pass_open) {
      if (!ctx->cl_flags || !ctx->have_tgt)
         return;
      fg_pass_open(ctx);
   }
   fg_send_target_size(ctx);
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
   fl = ctx->cl_flags | (ctx->tgt.depth16 ? FG_FL_DEPTH16 : 0);
   fg_emit_flush(ctx, fl);
   if (ctx->tgt.shadow_of) {
      /* rendered in the shadow: back into the image, after the FLUSH that rendered it */
      fg_emit_copy(ctx, ctx->tgt.color, 0, 0, 0, 0, ctx->tgt.shadow_of, 0, 0, 0, 0, ctx->tgt.width, ctx->tgt.height);
      fg_resource(ctx->tgt.shadow_of)->initialized = true;
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
   for (unsigned i = 0; i < ctx->npass_res; i++)
      pipe_resource_reference(&ctx->pass_res[i], NULL);
   ctx->npass_res = 0;
   ctx->nbos = 0;
   ctx->pass_open = false;
   fg_pass_take(ctx, false);
   ctx->pass_draws = 0;
   ctx->cl_flags = 0;
   ctx->cl_depth = 0xFFFFFF;
   ctx->cl_smask = 0xFF;
}

/* the image board B renders into in place of c (pitch a multiple of 32, one level, one layer), made the first time */
static struct pipe_resource *
fg_render_shadow(struct pipe_resource *c)
{
   struct fg_resource *r = fg_resource(c);

   if (!r->shadow) {
      struct pipe_resource templ = *c;

      templ.target = PIPE_TEXTURE_2D;
      templ.bind = PIPE_BIND_RENDER_TARGET;
      templ.last_level = 0;
      templ.array_size = 1;
      r->shadow = c->screen->resource_create(c->screen, &templ);
      r->shadow_stale = true;
   }
   return r->shadow;
}

/* the framebuffer's first colour buffer and depth-stencil buffer as the pass's target; a change ends the pass.
 * false: nothing board B can render into */
bool
fg_set_target(struct fg_context *ctx)
{
   struct pipe_framebuffer_state *fb = &ctx->fb;
   struct pipe_resource *c = fb->nr_cbufs ? fb->cbufs[0].texture : NULL, *rt;
   struct pipe_resource *z = fb->zsbuf.texture;
   struct fg_target t;
   const struct fg_format *f;

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
   if (fb->cbufs[0].level || fb->cbufs[0].first_layer || c->width0 > FG_FLUSH_MAX_WIDTH ||
       (z && (fb->zsbuf.level || fb->zsbuf.first_layer || z->width0 < c->width0 || z->height0 < c->height0))) {
      FG_NOTE_ONCE("rendering into a mip level above 0, a cube face or layer above 0, or a target wider than 1280");
      return false;
   }
   f = fg_resource(c)->fmt;
   if (!fg_format_renderable(f))
      return false;
   rt = c;
   if (fg_resource(c)->pitch % 32) {
      rt = fg_render_shadow(c);
      if (!rt)
         return false;
   }
   memset(&t, 0, sizeof(t));
   t.color = rt;
   t.shadow_of = rt != c ? c : NULL;
   t.ds = z;
   t.color_handle = fg_resource(rt)->board_handle;
   t.ds_handle = z ? fg_resource(z)->board_handle : 0;
   t.width = c->width0;
   t.height = c->height0;
   t.depth16 = z && z->format == PIPE_FORMAT_Z16_UNORM;
   t.has_stencil = z && util_format_has_stencil(util_format_description(z->format));
   memcpy(t.bits, f->bits, sizeof(t.bits));
   t.no_alpha = f->force_alpha;
   t.wmask = (uint8_t)fg_format_wmask(f);
   if (!ctx->have_tgt || memcmp(&ctx->tgt, &t, sizeof(t))) {
      fg_end_pass(ctx);
      ctx->tgt = t;
      ctx->have_tgt = true;
   }
   if (t.shadow_of && fg_resource(c)->shadow_stale) {
      /* the image changed (an upload, a copy, a first use): the shadow takes it before the pass renders. Board B
       * copies at once and renders at the FLUSH, so this lands under the pass's draws whenever it is sent */
      fg_emit_copy(ctx, c, 0, 0, 0, 0, rt, 0, 0, 0, 0, t.width, t.height);
      fg_resource(c)->shadow_stale = false;
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
       (ctx->tgt.color == prsc || ctx->tgt.ds == prsc || ctx->tgt.shadow_of == prsc))
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
   if (ctx->pass_draws || (have && (!same_sc || (have & want))))
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

static void
fg_blit(struct pipe_context *pctx, const struct pipe_blit_info *info)
{
   FG_NOTE_ONCE("blit");
}

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
   /* board B copies at once and renders at a FLUSH: a pass that reads or writes either image ends first */
   if (fg_context_pending_uses(ctx, src) || fg_context_pending_uses(ctx, dst))
      fg_end_pass(ctx);
   for (int z = 0; z < src_box->depth; z++)
      fg_emit_copy(ctx, src, src_level, src_box->z + z, src_box->x, src_box->y, dst, dst_level, dstz + z, dstx, dsty,
                   src_box->width, src_box->height);
   d->initialized = true;
   d->shadow_stale = true;
   if (!ctx->pass_open)
      fg_submit_stream(ctx);
}

/* COPY_IMAGE: src, level, layer, dst, level, layer, source x y w h, destination x y w h, flags, colour key. Both
 * images are held until the stream is submitted: the frontend often lets go of a source right after a copy */
void
fg_emit_copy(struct fg_context *ctx, struct pipe_resource *src, unsigned slevel, unsigned slayer, unsigned sx,
             unsigned sy, struct pipe_resource *dst, unsigned dlevel, unsigned dlayer, unsigned dx, unsigned dy,
             unsigned w, unsigned h)
{
   uint8_t b[36];

   fg_put16(b + 0, fg_resource(src)->board_handle);
   b[2] = (uint8_t)slevel;
   b[3] = (uint8_t)slayer;
   fg_put16(b + 4, fg_resource(dst)->board_handle);
   b[6] = (uint8_t)dlevel;
   b[7] = (uint8_t)dlayer;
   fg_put16(b + 8, sx);
   fg_put16(b + 10, sy);
   fg_put16(b + 12, w);
   fg_put16(b + 14, h);
   fg_put16(b + 16, dx);
   fg_put16(b + 18, dy);
   fg_put16(b + 20, w);
   fg_put16(b + 22, h);
   fg_put32(b + 24, 0);
   fg_put32(b + 28, 0);
   fg_put32(b + 32, 0);
   fg_emit(ctx, FG_T_COPY_IMAGE, b, sizeof(b), NULL, 0);
   fg_use_resource(ctx, src);
   fg_use_resource(ctx, dst);
}

static void
fg_clear_render_target(struct pipe_context *pctx, struct pipe_surface *dst, const union pipe_color_union *color,
                       unsigned dstx, unsigned dsty, unsigned width, unsigned height, bool render_condition_enabled)
{
   FG_NOTE_ONCE("clear_render_target");
}

static void
fg_clear_depth_stencil(struct pipe_context *pctx, struct pipe_surface *dst, unsigned clear_flags, double depth,
                       unsigned stencil, unsigned dstx, unsigned dsty, unsigned width, unsigned height,
                       bool render_condition_enabled)
{
   FG_NOTE_ONCE("clear_depth_stencil");
}

static void
fg_clear_texture(struct pipe_context *pctx, struct pipe_resource *res, unsigned level, const struct pipe_box *box,
                 const void *data)
{
   FG_NOTE_ONCE("clear_texture");
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

/* ------------------------------------------------------------------ queries (none are exposed) */

struct fg_query {
   unsigned type;
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
fg_begin_query(struct pipe_context *pctx, struct pipe_query *q)
{
   return true;
}

static bool
fg_end_query(struct pipe_context *pctx, struct pipe_query *q)
{
   return true;
}

static bool
fg_get_query_result(struct pipe_context *pctx, struct pipe_query *q, bool wait, union pipe_query_result *result)
{
   result->u64 = 0;
   return true;
}

static void
fg_set_active_query_state(struct pipe_context *pctx, bool enable)
{
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

static void
fg_texture_barrier(struct pipe_context *pctx, unsigned flags)
{
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
   fg_draw_fini(ctx);
   util_unreference_framebuffer_state(&ctx->fb);
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

   if (!ctx)
      return NULL;
   pctx = &ctx->base;
   pctx->screen = pscreen;
   pctx->priv = priv;
   ctx->debug = fg_screen(pscreen)->debug;
   ctx->cl_depth = 0xFFFFFF;
   ctx->cl_smask = 0xFF;
   ctx->next_id = 1;

   if (!fg_ctx_bo_create(ctx, &ctx->prog_a, FPGAGPU_BO_A_BUFFER, FG_PROG_A_BYTES) ||
       !fg_ctx_bo_create(ctx, &ctx->prog_b, FPGAGPU_BO_B_BUFFER, FG_PROG_B_BYTES) ||
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

   p_atomic_inc(&pscreen->num_contexts);
   return pctx;
}
