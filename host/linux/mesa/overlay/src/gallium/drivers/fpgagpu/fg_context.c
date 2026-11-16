/* SPDX-License-Identifier: MIT */
/*
 * fpgagpu context: framebuffer state, clears and flushes. State objects are kept but not sent, and draws are not
 * carried out: the first one prints a note on stderr.
 *
 * A frame that only clears is one submission, the way the Windows libraries end a clear-only pass:
 *   FRAME_BEGIN               board A starts a frame (its bins empty);
 *   VS_STATE                  the target's size, by which board A ends the frame's bands;
 *   FLUSH                     board B renders the frame into the colour image with the clear's flags and values,
 *                             keeping depth / stencil in the depth-stencil image.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "util/format/u_format.h"
#include "util/ralloc.h"
#include "util/u_atomic.h"
#include "util/u_framebuffer.h"
#include "util/u_inlines.h"
#include "util/u_memory.h"
#include "util/u_transfer.h"
#include "util/u_upload_mgr.h"

#include "fg_context.h"
#include "fg_resource.h"
#include "fg_screen.h"

/* board A clips x / y at +-2040 pixels: VS_STATE's guard band, in 1/16 pixel */
#define FG_GUARD_BAND 32640.0

/* colour channel mask bits, the same in Gallium's colour masks and FLUSH's: R 1, G 2, B 4, A 8 */
#define FG_MASK_A 0x8u

/* FP24: sign 1, exponent 7 (bias 63), mantissa 16; rounded to nearest even, flushed to zero */
static uint32_t
fg_fp24(double x)
{
   uint32_t s;
   double m, q;
   int e;

   if (x == 0.0)
      return 0;
   if (isinf(x))
      return (0x7Fu << 16) | (x < 0 ? 0x800000u : 0);
   if (isnan(x))
      return (0x7Fu << 16) | 0x8000u;
   s = x < 0 ? 0x800000u : 0;
   m = frexp(fabs(x), &e);		/* |x| = m 2^e, m in [0.5, 1) */
   e -= 1;
   m *= 2;
   q = rint(m * 65536.0);
   if (q == 131072.0) {
      q = 65536.0;
      e += 1;
   }
   if (e + 63 <= 0)
      return s;
   if (e + 63 >= 0x7F)
      return s | (0x7Fu << 16);
   return s | ((uint32_t)(e + 63) << 16) | ((uint32_t)q & 0xFFFFu);
}

/* a colour component in the render target's channel units: round(f 16 (2^bits - 1)), 4080 = 1.0 for 8 bits */
static uint32_t
fg_c16(float f, unsigned bits)
{
   double c = f > 0.0f ? (f < 1.0f ? f : 1.0) : 0.0;

   return bits ? (uint32_t)rint(c * 16.0 * (double)((1u << bits) - 1)) : 0;
}

/* VS_STATE for a frame without draws: the viewport is the whole target, no culling, line width 1, depth range
 * 0 .. 1, fragment shader inputs none, draw id 0, the draw rectangle the whole target */
static void
fg_build_vs_state(uint8_t b[60], uint32_t width, uint32_t height)
{
   const double vp[4] = { width * 8.0, width * 8.0, height * 8.0, height * 8.0 };	/* window = ndc s + o, 1/16 px */

   for (unsigned i = 0; i < 4; i++)
      fg_put32(b + 4 * i, fg_fp24(vp[i]));
   fg_put16(b + 16, 1u << 9);			/* cull 0, front clockwise 0, line width 1 */
   b[18] = 0;					/* fragment shader inputs */
   b[19] = 0;					/* z 0 .. w off, coarse-depth flags none */
   fg_put16(b + 20, width);
   fg_put16(b + 22, height);
   fg_put32(b + 24, 0);				/* draw id */
   fg_put32(b + 28, fg_fp24(0.5));		/* depth range (f - n) / 2 */
   fg_put32(b + 32, fg_fp24(0.5));		/* (n + f) / 2 */
   for (unsigned i = 0; i < 2; i++) {		/* the guard band's NDC bounds (+-G - o) / s, ordered */
      double s = vp[2 * i], o = vp[2 * i + 1], lo = (-FG_GUARD_BAND - o) / s, hi = (FG_GUARD_BAND - o) / s;

      fg_put32(b + 36 + 8 * i, fg_fp24(lo < hi ? lo : hi));
      fg_put32(b + 40 + 8 * i, fg_fp24(lo < hi ? hi : lo));
   }
   fg_put16(b + 52, 0);				/* draw rectangle x0, y0, x1, y1 */
   fg_put16(b + 54, 0);
   fg_put16(b + 56, width);
   fg_put16(b + 58, height);
}

/* FLUSH: {target, flags}, clear colour {R, G} and {B, A} (13 bits each), {depth 24, stencil 8}, the clear's
 * scissor {x0, y0}, {x1, y1}, {depth-stencil image, stencil write mask} */
static void
fg_build_flush(uint8_t b[28], uint16_t target, uint32_t flags, const uint32_t c[4], uint32_t depth,
               uint32_t stencil, const uint32_t sc[4], uint16_t ds, uint32_t stencil_mask)
{
   fg_put32(b + 0, (uint32_t)target | (flags & 0xFFFFu) << 16);
   fg_put32(b + 4, (c[0] & 0x1FFFu) | (c[1] & 0x1FFFu) << 13);
   fg_put32(b + 8, (c[2] & 0x1FFFu) | (c[3] & 0x1FFFu) << 13);
   fg_put32(b + 12, (depth & 0xFFFFFFu) | (stencil & 0xFFu) << 24);
   fg_put32(b + 16, (sc[0] & 0x1FFFu) | (sc[1] & 0x1FFFu) << 16);
   fg_put32(b + 20, (sc[2] & 0x1FFFu) | (sc[3] & 0x1FFFu) << 16);
   fg_put32(b + 24, (uint32_t)ds | (stencil_mask & 0xFFu) << 16);
}

static void
fg_pending_reset(struct fg_pending_clear *p)
{
   memset(p, 0, sizeof(*p));
   p->depth = 0xFFFFFF;
   p->stencil_mask = 0xFF;
}

bool
fg_context_pending_uses(struct fg_context *ctx, struct pipe_resource *prsc)
{
   return ctx->clear.flags && prsc && (ctx->clear.cbuf == prsc || ctx->clear.zsbuf == prsc);
}

void
fg_context_flush_pending(struct fg_context *ctx)
{
   struct fg_pending_clear *p = &ctx->clear;
   struct fg_screen *screen = fg_screen(ctx->base.screen);
   struct fg_resource *cb, *zs;
   uint8_t cmds[(4 + 4) + (4 + 60) + (4 + 28)], frame[4] = { 0 }, vs[60], fl[28];
   uint32_t bos[2], nbos = 0, n = 0, flags;
   uint64_t fence;
   int r;

   if (!p->flags)
      return;
   cb = fg_resource(p->cbuf);
   zs = p->zsbuf ? fg_resource(p->zsbuf) : NULL;
   flags = p->flags | (zs && zs->fmt->storage == FG_FMT_D16 ? FG_FL_DEPTH16 : 0);

   fg_build_vs_state(vs, cb->base.width0, cb->base.height0);
   fg_build_flush(fl, cb->board_handle, flags, p->c16, p->depth, p->stencil, p->sc, zs ? zs->board_handle : 0,
                  p->stencil_mask);
   n += fg_put_cmd(cmds + n, FG_T_FRAME_BEGIN, frame, sizeof(frame), NULL, 0);
   n += fg_put_cmd(cmds + n, FG_T_VS_STATE, vs, sizeof(vs), NULL, 0);
   n += fg_put_cmd(cmds + n, FG_T_FLUSH, fl, sizeof(fl), NULL, 0);
   bos[nbos++] = cb->gem;
   if (zs)
      bos[nbos++] = zs->gem;

   r = fg_submit(screen, cmds, n, bos, nbos, &fence);
   if (r) {
      fprintf(stderr, "fpgagpu: SUBMIT of a clear failed: %s\n", strerror(-r));
   } else {
      fg_context_fenced(ctx, fence);
      if (flags & FG_FL_CLEAR_COLOR)
         cb->initialized = true;
      if (zs && (flags & (FG_FL_CLEAR_DEPTH | FG_FL_CLEAR_STENCIL)))
         zs->initialized = true;
   }
   pipe_resource_reference(&p->cbuf, NULL);
   pipe_resource_reference(&p->zsbuf, NULL);
   fg_pending_reset(p);
}

static void
fg_clear(struct pipe_context *pctx, unsigned buffers, uint32_t color_clear_mask, uint8_t stencil_clear_mask,
         const struct pipe_scissor_state *scissor, const union pipe_color_union *color, double depth,
         unsigned stencil)
{
   struct fg_context *ctx = fg_context(pctx);
   struct pipe_framebuffer_state *fb = &ctx->fb;
   struct fg_pending_clear *p = &ctx->clear;
   struct pipe_resource *cres = fb->nr_cbufs ? fb->cbufs[0].texture : NULL;
   struct pipe_resource *zres = fb->zsbuf.texture;
   uint32_t want = 0, sflag = 0, sc[4] = { 0, 0, 0, 0 };

   if ((buffers & PIPE_CLEAR_COLOR0) && cres)
      want |= FG_FL_CLEAR_COLOR;
   if ((buffers & PIPE_CLEAR_DEPTH) && zres)
      want |= FG_FL_CLEAR_DEPTH;
   if ((buffers & PIPE_CLEAR_STENCIL) && zres && util_format_has_stencil(util_format_description(zres->format)))
      want |= FG_FL_CLEAR_STENCIL;
   if ((buffers & PIPE_CLEAR_COLOR) & ~PIPE_CLEAR_COLOR0)
      FG_NOTE_ONCE("clearing a colour buffer other than the first");
   if (!want)
      return;
   if (!cres) {
      FG_NOTE_ONCE("a depth / stencil clear without a colour buffer");
      return;
   }
   if (fb->cbufs[0].level || fb->cbufs[0].first_layer || cres->width0 > FG_FLUSH_MAX_WIDTH ||
       (zres && (fb->zsbuf.level || fb->zsbuf.first_layer))) {
      FG_NOTE_ONCE("a clear of a mip level above 0, a cube face or array layer above 0, or a target wider than 1280");
      return;
   }

   if (scissor && (scissor->minx > 0 || scissor->miny > 0 || scissor->maxx < cres->width0 ||
                   scissor->maxy < cres->height0)) {
      if (scissor->maxx <= scissor->minx || scissor->maxy <= scissor->miny)
         return;
      sflag = FG_FL_CLEAR_SCISSOR;
      sc[0] = scissor->minx;
      sc[1] = scissor->miny;
      sc[2] = MIN2(scissor->maxx, cres->width0);
      sc[3] = MIN2(scissor->maxy, cres->height0);
   }

   /* clears merge while they name the same buffers, agree on the rectangle and do not overlap */
   if (p->flags && (p->cbuf != cres || p->zsbuf != zres || (p->flags & FG_FL_CLEAR_SCISSOR) != sflag ||
                    (sflag && memcmp(p->sc, sc, sizeof(sc))) || (p->flags & want)))
      fg_context_flush_pending(ctx);

   if (want & FG_FL_CLEAR_COLOR) {
      const struct fg_format *f = fg_resource(cres)->fmt;
      uint32_t mask = color_clear_mask & 0xF;

      for (unsigned i = 0; i < 3; i++)
         p->c16[i] = fg_c16(color->f[i], f->bits[i]);
      if (f->force_alpha) {
         p->c16[3] = fg_c16(1.0f, 8);
         if (mask)
            mask |= FG_MASK_A;
      } else {
         p->c16[3] = fg_c16(color->f[3], f->bits[3]);
         if (!f->bits[3])
            mask &= ~FG_MASK_A;
      }
      if (mask) {
         p->flags |= FG_FL_CMASK(mask);
         if (ctx->blend && ctx->blend->dither)
            p->flags |= FG_FL_DITHER;
      } else {
         want &= ~FG_FL_CLEAR_COLOR;
      }
   }
   if (want & FG_FL_CLEAR_DEPTH) {
      double d = depth > 0.0 ? (depth < 1.0 ? depth : 1.0) : 0.0;

      p->depth = (uint32_t)rint(d * (zres->format == PIPE_FORMAT_Z16_UNORM ? 65535.0 : 16777215.0));
   }
   if (want & FG_FL_CLEAR_STENCIL) {
      p->stencil = stencil & 0xFF;
      p->stencil_mask = stencil_clear_mask;
   }
   if (!want)
      return;

   pipe_resource_reference(&p->cbuf, cres);
   pipe_resource_reference(&p->zsbuf, zres);
   p->flags |= want | sflag;
   memcpy(p->sc, sc, sizeof(sc));
}

static void
fg_flush(struct pipe_context *pctx, struct pipe_fence_handle **fence, unsigned flags)
{
   struct fg_context *ctx = fg_context(pctx);

   fg_context_flush_pending(ctx);
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

static void
fg_draw_vbo(struct pipe_context *pctx, const struct pipe_draw_info *info, unsigned drawid_offset,
            const struct pipe_draw_indirect_info *indirect, const struct pipe_draw_start_count_bias *draws,
            unsigned num_draws)
{
   FG_NOTE_ONCE("drawing (draw_vbo)");
}

static void
fg_draw_vertex_state(struct pipe_context *pctx, struct pipe_vertex_state *state, uint32_t partial_velem_mask,
                     struct pipe_draw_vertex_state_info info, const struct pipe_draw_start_count_bias *draws,
                     unsigned num_draws)
{
   FG_NOTE_ONCE("drawing (draw_vertex_state)");
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
   FG_NOTE_ONCE("resource_copy_region");
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
fg_invalidate_resource(struct pipe_context *pctx, struct pipe_resource *prsc)
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

/* ------------------------------------------------------------------ state objects: kept, not sent */

struct fg_shader {
   void *nir;			/* the nir_shader handed over by the frontend (ralloc'ed), or NULL */
};

static void *
fg_create_shader_state(struct pipe_context *pctx, const struct pipe_shader_state *cso)
{
   struct fg_shader *so = CALLOC_STRUCT(fg_shader);

   if (so && cso->type == PIPE_SHADER_IR_NIR)
      so->nir = cso->ir.nir;
   return so;
}

static void
fg_delete_shader_state(struct pipe_context *pctx, void *hwcso)
{
   struct fg_shader *so = hwcso;

   if (so)
      ralloc_free(so->nir);
   FREE(so);
}

static void *
fg_create_compute_state(struct pipe_context *pctx, const struct pipe_compute_state *cso)
{
   return CALLOC_STRUCT(fg_shader);
}

static void *
fg_create_blend_state(struct pipe_context *pctx, const struct pipe_blend_state *cso)
{
   struct pipe_blend_state *so = MALLOC_STRUCT(pipe_blend_state);

   if (so)
      *so = *cso;
   return so;
}

static void
fg_bind_blend_state(struct pipe_context *pctx, void *hwcso)
{
   fg_context(pctx)->blend = hwcso;
}

static void
fg_delete_blend_state(struct pipe_context *pctx, void *hwcso)
{
   struct fg_context *ctx = fg_context(pctx);

   if (ctx->blend == hwcso)
      ctx->blend = NULL;
   FREE(hwcso);
}

static void *
fg_create_dsa_state(struct pipe_context *pctx, const struct pipe_depth_stencil_alpha_state *cso)
{
   return MALLOC(1);
}

static void *
fg_create_rs_state(struct pipe_context *pctx, const struct pipe_rasterizer_state *cso)
{
   return MALLOC(1);
}

static void *
fg_create_sampler_state(struct pipe_context *pctx, const struct pipe_sampler_state *cso)
{
   return MALLOC(1);
}

static void *
fg_create_vertex_elements(struct pipe_context *pctx, unsigned count, const struct pipe_vertex_element *elements)
{
   return MALLOC(1);
}

static void
fg_bind_state(struct pipe_context *pctx, void *hwcso)
{
}

static void
fg_delete_state(struct pipe_context *pctx, void *hwcso)
{
   FREE(hwcso);
}

static void
fg_bind_sampler_states(struct pipe_context *pctx, mesa_shader_stage shader, unsigned start, unsigned count,
                       void **states)
{
}

static struct pipe_sampler_view *
fg_create_sampler_view(struct pipe_context *pctx, struct pipe_resource *texture,
                       const struct pipe_sampler_view *templ)
{
   struct pipe_sampler_view *view = CALLOC_STRUCT(pipe_sampler_view);

   if (!view)
      return NULL;
   *view = *templ;
   view->texture = NULL;
   pipe_resource_reference(&view->texture, texture);
   pipe_reference_init(&view->reference, 1);
   view->context = pctx;
   return view;
}

static void
fg_sampler_view_destroy(struct pipe_context *pctx, struct pipe_sampler_view *view)
{
   pipe_resource_reference(&view->texture, NULL);
   FREE(view);
}

static void
fg_set_sampler_views(struct pipe_context *pctx, mesa_shader_stage shader, unsigned start, unsigned count,
                     unsigned unbind_num_trailing_slots, struct pipe_sampler_view **views)
{
}

static void
fg_set_blend_color(struct pipe_context *pctx, const struct pipe_blend_color *state)
{
}

static void
fg_set_clip_state(struct pipe_context *pctx, const struct pipe_clip_state *state)
{
}

static void
fg_set_constant_buffer(struct pipe_context *pctx, mesa_shader_stage shader, uint index,
                       const struct pipe_constant_buffer *cb)
{
}

static void
fg_set_inlinable_constants(struct pipe_context *pctx, mesa_shader_stage shader, uint num_values,
                           uint32_t *values)
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
fg_set_scissor_states(struct pipe_context *pctx, unsigned start_slot, unsigned num_scissors,
                      const struct pipe_scissor_state *state)
{
}

static void
fg_set_stencil_ref(struct pipe_context *pctx, const struct pipe_stencil_ref state)
{
}

static void
fg_set_viewport_states(struct pipe_context *pctx, unsigned start_slot, unsigned num_viewports,
                       const struct pipe_viewport_state *state)
{
}

static void
fg_set_vertex_buffers(struct pipe_context *pctx, unsigned count, const struct pipe_vertex_buffer *buffers)
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
fg_render_condition(struct pipe_context *pctx, struct pipe_query *query, bool condition,
                    enum pipe_render_cond_flag mode)
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

   fg_context_flush_pending(ctx);
   util_unreference_framebuffer_state(&ctx->fb);
   if (pctx->stream_uploader)
      u_upload_destroy(pctx->stream_uploader);
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
   fg_pending_reset(&ctx->clear);

   pctx->stream_uploader = u_upload_create_default(pctx);
   if (!pctx->stream_uploader) {
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
   pctx->draw_vbo = fg_draw_vbo;
   pctx->draw_vertex_state = fg_draw_vertex_state;

   pctx->create_query = fg_create_query;
   pctx->destroy_query = fg_destroy_query;
   pctx->begin_query = fg_begin_query;
   pctx->end_query = fg_end_query;
   pctx->get_query_result = fg_get_query_result;
   pctx->set_active_query_state = fg_set_active_query_state;
   pctx->render_condition = fg_render_condition;

   pctx->create_blend_state = fg_create_blend_state;
   pctx->bind_blend_state = fg_bind_blend_state;
   pctx->delete_blend_state = fg_delete_blend_state;
   pctx->create_depth_stencil_alpha_state = fg_create_dsa_state;
   pctx->bind_depth_stencil_alpha_state = fg_bind_state;
   pctx->delete_depth_stencil_alpha_state = fg_delete_state;
   pctx->create_rasterizer_state = fg_create_rs_state;
   pctx->bind_rasterizer_state = fg_bind_state;
   pctx->delete_rasterizer_state = fg_delete_state;
   pctx->create_sampler_state = fg_create_sampler_state;
   pctx->bind_sampler_states = fg_bind_sampler_states;
   pctx->delete_sampler_state = fg_delete_state;
   pctx->create_vertex_elements_state = fg_create_vertex_elements;
   pctx->bind_vertex_elements_state = fg_bind_state;
   pctx->delete_vertex_elements_state = fg_delete_state;
   pctx->create_vs_state = fg_create_shader_state;
   pctx->bind_vs_state = fg_bind_state;
   pctx->delete_vs_state = fg_delete_shader_state;
   pctx->create_fs_state = fg_create_shader_state;
   pctx->bind_fs_state = fg_bind_state;
   pctx->delete_fs_state = fg_delete_shader_state;
   pctx->create_compute_state = fg_create_compute_state;
   pctx->bind_compute_state = fg_bind_state;
   pctx->delete_compute_state = fg_delete_state;

   pctx->create_sampler_view = fg_create_sampler_view;
   pctx->sampler_view_destroy = fg_sampler_view_destroy;
   pctx->sampler_view_release = u_default_sampler_view_release;
   pctx->set_sampler_views = fg_set_sampler_views;
   pctx->set_framebuffer_state = fg_set_framebuffer_state;
   pctx->set_blend_color = fg_set_blend_color;
   pctx->set_clip_state = fg_set_clip_state;
   pctx->set_constant_buffer = fg_set_constant_buffer;
   pctx->set_inlinable_constants = fg_set_inlinable_constants;
   pctx->set_polygon_stipple = fg_set_polygon_stipple;
   pctx->set_sample_mask = fg_set_sample_mask;
   pctx->set_scissor_states = fg_set_scissor_states;
   pctx->set_stencil_ref = fg_set_stencil_ref;
   pctx->set_viewport_states = fg_set_viewport_states;
   pctx->set_vertex_buffers = fg_set_vertex_buffers;
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

   p_atomic_inc(&pscreen->num_contexts);
   return pctx;
}
