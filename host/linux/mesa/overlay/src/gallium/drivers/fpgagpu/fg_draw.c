/* SPDX-License-Identifier: MIT */
/*
 * fpgagpu draws: state objects, shaders, and a draw's messages (host/win/core/g2_render.c g2r_draw):
 *   [WRITE_BUFFER the vertices and indices into the ring]
 *   [VS_PROGRAM] [VERTEX_ATTR ...] [VS_UNIFORMS ...]   board A: what changed since the last draw
 *   [DRAW_STATE id]                                     board B: the draw's state and uniforms, by id
 *   [VS_STATE]                                          board A: viewport, culling, the draw id, the target's size
 *   DRAW
 * A DRAW_STATE stays in board B's bucket until another overwrites it, so a draw with the same words and uniforms
 * takes the id it had before, across passes and frames.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "compiler/nir/nir.h"
#include "compiler/nir/nir_builder.h"
#include "indices/u_primconvert.h"
#include "util/format/u_format.h"
#include "util/u_atomic.h"
#include "util/u_helpers.h"
#include "util/u_inlines.h"
#include "util/u_math.h"
#include "util/u_memory.h"

#include "fg_context.h"
#include "fg_resource.h"
#include "fg_screen.h"

#define FG_T_DRAW_STATE  0x50
#define FG_T_VS_PROGRAM  0x51
#define FG_T_VS_UNIFORMS 0x52
#define FG_T_VERTEX_ATTR 0x53
#define FG_T_DRAW        0x55

#define FG_GUARD_BAND 32640.0 /* board A clips x / y at +-2040 pixels, in 1/16 pixel */

enum { FG_AT_BYTE, FG_AT_UBYTE, FG_AT_SHORT, FG_AT_USHORT, FG_AT_FIXED, FG_AT_FLOAT, FG_AT_FP24 };
enum { FG_FN_NEVER, FG_FN_LESS, FG_FN_EQUAL, FG_FN_LEQUAL, FG_FN_GREATER, FG_FN_NOTEQUAL, FG_FN_GEQUAL, FG_FN_ALWAYS };
enum { FG_ZC_OCC = 1, FG_ZC_CULL = 2, FG_ZC_STRICT = 4, FG_ZC_INV = 8, FG_ZC_LAZY = 16 };

/* what one draw sends: g2r_state */
struct fg_hw_draw {
   const struct fg_vs_state *vs;
   const struct fg_fs_variant *fs;
   double vp[4];              /* window = ndc s + o, 1/16 pixel: s x, o x, s y, o y */
   double dscale, doffset;    /* window z = ndc z dscale + doffset */
   bool z01;                  /* the clip volume is 0 <= z <= w */
   int cull;                  /* 0 none, 1 back, 2 front, 3 both */
   bool front_cw;
   double line_width;
   uint32_t h[23];
   uint32_t attr[FG_MAX_ATTRIBS][8]; /* VERTEX_ATTR bodies */
   const uint32_t *vs_uni;    /* FP24, vs->prog.const_base vectors */
   const uint32_t *fs_uni;    /* FP24, fs->prog.const_base vectors */
};

/* ------------------------------------------------------------------ board A's attribute formats */

static bool
fg_attr_format(enum pipe_format format, uint8_t *type, uint8_t *size, uint8_t *normalized, uint8_t *bytes)
{
   const struct util_format_description *d = util_format_description(format);
   const struct util_format_channel_description *c;

   if (!fg_vertex_format_supported(format))
      return false;
   c = &d->channel[0];
   *size = d->nr_channels;
   *normalized = c->normalized;
   *bytes = d->block.bits / 8;
   switch (c->type) {
   case UTIL_FORMAT_TYPE_FLOAT: *type = FG_AT_FLOAT; break;
   case UTIL_FORMAT_TYPE_FIXED: *type = FG_AT_FIXED; break;
   case UTIL_FORMAT_TYPE_SIGNED: *type = c->size == 8 ? FG_AT_BYTE : FG_AT_SHORT; break;
   default: *type = c->size == 8 ? FG_AT_UBYTE : FG_AT_USHORT; break;
   }
   return true;
}

/* VERTEX_ATTR: {normalized [24], type [22:20], size [18:16], enabled [8], index [3:0]}, {handle, 0}, stride, offset,
 * the constant (FP24) of a disabled attribute */
static void
fg_attr_words(uint32_t w[8], unsigned index, bool enabled, unsigned size, unsigned type, bool normalized,
              uint16_t handle, uint32_t stride, uint32_t offset, const uint32_t c[4])
{
   w[0] = (index & 0xF) | (enabled ? 1u << 8 : 0) | (size & 7) << 16 | (type & 7) << 20 | (normalized ? 1u << 24 : 0);
   w[1] = enabled ? handle : 0;
   w[2] = stride;
   w[3] = offset;
   memcpy(&w[4], c, 16);
}

/* ------------------------------------------------------------------ blend, depth-stencil, rasterizer, samplers */

static uint32_t
fg_blend_factor(enum pipe_blendfactor f)
{
   switch (f) {
   case PIPE_BLENDFACTOR_ZERO: return 0;
   case PIPE_BLENDFACTOR_ONE: return 1;
   case PIPE_BLENDFACTOR_SRC_COLOR: return 2;
   case PIPE_BLENDFACTOR_INV_SRC_COLOR: return 3;
   case PIPE_BLENDFACTOR_DST_COLOR: return 4;
   case PIPE_BLENDFACTOR_INV_DST_COLOR: return 5;
   case PIPE_BLENDFACTOR_SRC_ALPHA: return 6;
   case PIPE_BLENDFACTOR_INV_SRC_ALPHA: return 7;
   case PIPE_BLENDFACTOR_DST_ALPHA: return 8;
   case PIPE_BLENDFACTOR_INV_DST_ALPHA: return 9;
   case PIPE_BLENDFACTOR_CONST_COLOR: return 10;
   case PIPE_BLENDFACTOR_INV_CONST_COLOR: return 11;
   case PIPE_BLENDFACTOR_CONST_ALPHA: return 12;
   case PIPE_BLENDFACTOR_INV_CONST_ALPHA: return 13;
   case PIPE_BLENDFACTOR_SRC_ALPHA_SATURATE: return 14;
   default:
      FG_NOTE_ONCE("a blend factor of a second colour source");
      return 1;
   }
}

static uint32_t
fg_blend_func(enum pipe_blend_func f)
{
   switch (f) {
   case PIPE_BLEND_SUBTRACT: return 1;
   case PIPE_BLEND_REVERSE_SUBTRACT: return 2;
   case PIPE_BLEND_MIN: return 3;
   case PIPE_BLEND_MAX: return 4;
   default: return 0;
   }
}

static void *
fg_create_blend_state(struct pipe_context *pctx, const struct pipe_blend_state *cso)
{
   struct fg_blend_state *so = CALLOC_STRUCT(fg_blend_state);
   const struct pipe_rt_blend_state *rt = &cso->rt[0];

   if (!so)
      return NULL;
   if (cso->logicop_enable)
      FG_NOTE_ONCE("a logic op");
   so->h0 = (rt->blend_enable ? 64u : 0) | (cso->dither ? 128u : 0) | fg_board_cmask(rt->colormask) << 16;
   if (rt->blend_enable)
      so->h4 = fg_blend_factor(rt->rgb_src_factor) | fg_blend_factor(rt->rgb_dst_factor) << 4 |
               fg_blend_factor(rt->alpha_src_factor) << 8 | fg_blend_factor(rt->alpha_dst_factor) << 12 |
               fg_blend_func(rt->rgb_func) << 16 | fg_blend_func(rt->alpha_func) << 20;
   else
      so->h4 = 1u | 1u << 8; /* ONE, ZERO: what the factors of a state without blending read as */
   so->dither = cso->dither;
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

static uint32_t
fg_stencil_op(enum pipe_stencil_op o)
{
   switch (o) {
   case PIPE_STENCIL_OP_ZERO: return 1;
   case PIPE_STENCIL_OP_REPLACE: return 2;
   case PIPE_STENCIL_OP_INCR: return 3;
   case PIPE_STENCIL_OP_DECR: return 4;
   case PIPE_STENCIL_OP_INVERT: return 5;
   case PIPE_STENCIL_OP_INCR_WRAP: return 6;
   case PIPE_STENCIL_OP_DECR_WRAP: return 7;
   default: return 0;
   }
}

static void *
fg_create_dsa_state(struct pipe_context *pctx, const struct pipe_depth_stencil_alpha_state *cso)
{
   struct fg_dsa_state *so = CALLOC_STRUCT(fg_dsa_state);
   const struct pipe_stencil_state *f = &cso->stencil[0], *b = cso->stencil[1].enabled ? &cso->stencil[1] : f;

   if (!so)
      return NULL;
   /* the function codes are Gallium's (PIPE_FUNC_NEVER .. ALWAYS) */
   so->h0 = (cso->depth_enabled ? 1u : 0) | (cso->depth_enabled && cso->depth_writemask ? 2u : 0) |
            (uint32_t)(cso->depth_func & 7) << 2 | (f->enabled ? 32u : 0);
   so->h1 = (uint32_t)f->valuemask << 8 | (uint32_t)f->writemask << 16 | (uint32_t)(f->func & 7) << 24;
   so->h2 = (uint32_t)b->valuemask << 8 | (uint32_t)b->writemask << 16 | (uint32_t)(b->func & 7) << 24;
   so->h3 = fg_stencil_op(f->fail_op) | fg_stencil_op(f->zfail_op) << 3 | fg_stencil_op(f->zpass_op) << 6 |
            fg_stencil_op(b->fail_op) << 16 | fg_stencil_op(b->zfail_op) << 19 | fg_stencil_op(b->zpass_op) << 22;
   if (cso->alpha_enabled)
      FG_NOTE_ONCE("alpha test in the depth-stencil-alpha state");
   return so;
}

static void
fg_bind_dsa_state(struct pipe_context *pctx, void *hwcso)
{
   fg_context(pctx)->dsa = hwcso;
}

static void
fg_delete_dsa_state(struct pipe_context *pctx, void *hwcso)
{
   struct fg_context *ctx = fg_context(pctx);

   if (ctx->dsa == hwcso)
      ctx->dsa = NULL;
   FREE(hwcso);
}

static void *
fg_create_rs_state(struct pipe_context *pctx, const struct pipe_rasterizer_state *cso)
{
   struct pipe_rasterizer_state *so = MALLOC_STRUCT(pipe_rasterizer_state);

   if (so)
      *so = *cso;
   return so;
}

static void
fg_bind_rs_state(struct pipe_context *pctx, void *hwcso)
{
   struct fg_context *ctx = fg_context(pctx);

   ctx->rast = hwcso;
   if (hwcso)
      util_primconvert_save_rasterizer_state(ctx->primconvert, hwcso);
}

static void
fg_delete_rs_state(struct pipe_context *pctx, void *hwcso)
{
   struct fg_context *ctx = fg_context(pctx);

   if (ctx->rast == hwcso)
      ctx->rast = NULL;
   FREE(hwcso);
}

static uint32_t
fg_wrap(enum pipe_tex_wrap w)
{
   switch (w) {
   case PIPE_TEX_WRAP_REPEAT: return 0;
   case PIPE_TEX_WRAP_MIRROR_REPEAT: return 2;
   default: return 1; /* clamp to the edge: the TMU has no border colour */
   }
}

static void *
fg_create_sampler_state(struct pipe_context *pctx, const struct pipe_sampler_state *cso)
{
   struct fg_sampler_state *so = CALLOC_STRUCT(fg_sampler_state);
   uint32_t mag = cso->mag_img_filter == PIPE_TEX_FILTER_LINEAR ? 8 : 0;
   uint32_t min = cso->min_img_filter == PIPE_TEX_FILTER_LINEAR ? 1 : 0;
   uint32_t mip = cso->min_mip_filter == PIPE_TEX_MIPFILTER_NEAREST ? 1 : cso->min_mip_filter == PIPE_TEX_MIPFILTER_LINEAR ? 2 : 0;

   if (!so)
      return NULL;
   so->filter_nomip = mag | min;
   so->filter = mag | min | mip << 1;
   so->wrap = fg_wrap(cso->wrap_s) << 20 | fg_wrap(cso->wrap_t) << 22 | fg_wrap(cso->wrap_r) << 25;
   return so;
}

static void
fg_bind_sampler_states(struct pipe_context *pctx, mesa_shader_stage shader, unsigned start, unsigned count,
                       void **states)
{
   struct fg_context *ctx = fg_context(pctx);

   if (shader != MESA_SHADER_FRAGMENT)
      return;
   for (unsigned i = 0; i < count && start + i < FG_MAX_SAMPLERS; i++)
      ctx->samplers[start + i] = states ? states[i] : NULL;
}

static void
fg_delete_sampler_state(struct pipe_context *pctx, void *hwcso)
{
   struct fg_context *ctx = fg_context(pctx);

   for (unsigned i = 0; i < FG_MAX_SAMPLERS; i++)
      if (ctx->samplers[i] == hwcso)
         ctx->samplers[i] = NULL;
   FREE(hwcso);
}

static struct pipe_sampler_view *
fg_create_sampler_view(struct pipe_context *pctx, struct pipe_resource *texture, const struct pipe_sampler_view *templ)
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
   struct fg_context *ctx = fg_context(pctx);
   unsigned i;

   if (shader != MESA_SHADER_FRAGMENT)
      return;
   for (i = 0; i < count && start + i < FG_MAX_SAMPLERS; i++)
      pipe_sampler_view_reference(&ctx->views[start + i], views ? views[i] : NULL);
   for (unsigned j = 0; j < unbind_num_trailing_slots && start + i + j < FG_MAX_SAMPLERS; j++)
      pipe_sampler_view_reference(&ctx->views[start + i + j], NULL);
}

static void *
fg_create_vertex_elements(struct pipe_context *pctx, unsigned count, const struct pipe_vertex_element *elements)
{
   struct fg_velems_state *so = CALLOC_STRUCT(fg_velems_state);

   if (!so)
      return NULL;
   so->count = MIN2(count, FG_MAX_ATTRIBS);
   for (unsigned i = 0; i < so->count; i++) {
      struct fg_velem *v = &so->v[i];

      v->e = elements[i];
      if (!fg_attr_format(elements[i].src_format, &v->type, &v->size, &v->normalized, &v->bytes)) {
         fprintf(stderr, "fpgagpu: vertex format %s cannot be fetched\n", util_format_name(elements[i].src_format));
         v->bytes = 0;
      }
      if (elements[i].instance_divisor)
         FG_NOTE_ONCE("instanced vertex attributes");
   }
   return so;
}

static void
fg_bind_vertex_elements(struct pipe_context *pctx, void *hwcso)
{
   fg_context(pctx)->velems = hwcso;
}

static void
fg_delete_vertex_elements(struct pipe_context *pctx, void *hwcso)
{
   struct fg_context *ctx = fg_context(pctx);

   if (ctx->velems == hwcso)
      ctx->velems = NULL;
   FREE(hwcso);
}

/* ------------------------------------------------------------------ shaders */

static uint32_t
fg_noops(uint32_t flags)
{
   return flags >> 30 & 3u; /* bit 0 no integer divider, bit 1 no SIN / COS */
}

static void
fg_report(struct fg_screen *screen, const char *what, const struct fg_program *p)
{
   if (screen->debug_shaders)
      fprintf(stderr, "fpgagpu: %s: %u instructions, %u uniform + %u constant vectors, varyings 0x%x\n%s\n", what,
              p->nwords, p->const_base, p->nconsts, p->varyings, p->sasm ? p->sasm : "");
}

static void *
fg_create_vs_state(struct pipe_context *pctx, const struct pipe_shader_state *cso)
{
   struct fg_context *ctx = fg_context(pctx);
   struct fg_screen *screen = fg_screen(pctx->screen);
   struct fg_vs_state *so = CALLOC_STRUCT(fg_vs_state);
   struct fg_compile_key key = { 0 };
   char err[512];

   if (!so)
      return NULL;
   so->off = ~0u;
   so->id = ++ctx->next_vs_id;
   if (cso->type != PIPE_SHADER_IR_NIR) {
      fprintf(stderr, "fpgagpu: a vertex shader that is not NIR\n");
      return so;
   }
   so->nir = cso->ir.nir;
   fg_nir_lower(so->nir);
   so->vmask = fg_varying_mask(so->nir->info.outputs_written);
   key.noops = fg_noops(screen->a_flags);
   if (fg_compile(so->nir, &key, &so->prog, err, sizeof(err)))
      fprintf(stderr, "fpgagpu: vertex shader: %s\n", err);
   else {
      so->ok = true;
      fg_report(screen, "vertex shader", &so->prog);
   }
   return so;
}

static void
fg_bind_vs_state(struct pipe_context *pctx, void *hwcso)
{
   fg_context(pctx)->vs = hwcso;
}

static void
fg_delete_vs_state(struct pipe_context *pctx, void *hwcso)
{
   struct fg_context *ctx = fg_context(pctx);
   struct fg_vs_state *so = hwcso;

   if (ctx->vs == so)
      ctx->vs = NULL;
   if (ctx->hw_prog == so)
      ctx->hw_prog = NULL;
   fg_program_free(&so->prog);
   ralloc_free(so->nir);
   FREE(so);
}

static void *
fg_create_fs_state(struct pipe_context *pctx, const struct pipe_shader_state *cso)
{
   struct fg_fs_state *so = CALLOC_STRUCT(fg_fs_state);

   if (!so)
      return NULL;
   if (cso->type != PIPE_SHADER_IR_NIR) {
      fprintf(stderr, "fpgagpu: a fragment shader that is not NIR\n");
      return so;
   }
   so->nir = cso->ir.nir;
   fg_nir_lower(so->nir);
   return so;
}

static void
fg_bind_fs_state(struct pipe_context *pctx, void *hwcso)
{
   fg_context(pctx)->fs = hwcso;
}

static void
fg_delete_fs_state(struct pipe_context *pctx, void *hwcso)
{
   struct fg_context *ctx = fg_context(pctx);
   struct fg_fs_state *so = hwcso;

   if (ctx->fs == so)
      ctx->fs = NULL;
   while (so->variants) {
      struct fg_fs_variant *v = so->variants;

      so->variants = v->next;
      fg_program_free(&v->prog);
      FREE(v);
   }
   ralloc_free(so->nir);
   FREE(so);
}

/* the fragment program for this vertex stage and these textures */
static struct fg_fs_variant *
fg_fs_variant(struct fg_context *ctx, struct fg_fs_state *fs, const struct fg_vs_state *vs)
{
   struct fg_screen *screen = fg_screen(ctx->base.screen);
   struct fg_compile_key key = { 0 };
   struct fg_fs_variant *v;
   char err[512];

   if (!fs->nir)
      return NULL;
   key.xops = screen->b_flags >> 28 & 3u;
   key.noops = fg_noops(screen->b_flags);
   key.vs_outputs = vs->vmask;
   for (unsigned u = 0; u < FG_MAX_SAMPLERS; u++) {
      struct pipe_sampler_view *view = ctx->views[u];

      if (!view || !view->texture || !(fs->nir->info.textures_used[0] & (1u << u)))
         continue;
      if (util_format_is_depth_or_stencil(view->texture->format))
         key.depth_samplers |= 1u << u;
      unsigned sw = view->swizzle_r | view->swizzle_g << 3 | view->swizzle_b << 6 | view->swizzle_a << 9;
      if (sw != (PIPE_SWIZZLE_X | PIPE_SWIZZLE_Y << 3 | PIPE_SWIZZLE_Z << 6 | PIPE_SWIZZLE_W << 9)) {
         key.swizzled |= 1u << u;
         key.swizzle[u] = sw;
      }
   }
   for (v = fs->variants; v; v = v->next)
      if (!memcmp(&v->key, &key, sizeof(key)))
         return v->ok ? v : NULL;
   v = CALLOC_STRUCT(fg_fs_variant);
   if (!v)
      return NULL;
   v->key = key;
   v->off = ~0u;
   if (fg_compile(fs->nir, &key, &v->prog, err, sizeof(err)))
      fprintf(stderr, "fpgagpu: fragment shader: %s\n", err);
   else {
      v->ok = true;
      fg_report(screen, "fragment shader", &v->prog);
   }
   v->next = fs->variants;
   fs->variants = v;
   return v->ok ? v : NULL;
}

/* a program's words into the context's program buffer the first time it is drawn with (in this placement
 * generation); false: no room */
static bool
fg_place(struct fg_context *ctx, struct fg_ctx_bo *bo, const struct fg_program *p, uint32_t *off, uint32_t *gen)
{
   uint32_t a = (bo->used + 31) & ~31u, bytes = p->nwords * 8;

   if (*off != ~0u && *gen == ctx->prog_gen)
      return true;
   if (a + bytes > bo->size) {
      FG_NOTE_ONCE("reusing program memory (the context's program buffer is full)");
      return false;
   }
   fg_write_buffer(ctx, bo, a, p->words, bytes);
   bo->used = a + bytes;
   *off = a;
   *gen = ctx->prog_gen;
   return true;
}

/* ------------------------------------------------------------------ the clear quad's programs */

static nir_shader *
fg_clear_nir(const nir_shader_compiler_options *o, bool fs)
{
   nir_builder b = nir_builder_init_simple_shader(fs ? MESA_SHADER_FRAGMENT : MESA_SHADER_VERTEX, o, "fpgagpu clear");
   nir_def *zero = nir_imm_int(&b, 0);
   nir_io_semantics sem = { 0 };

   sem.num_slots = 1;
   if (!fs) {
      nir_io_semantics in = { 0 };
      nir_def *p;

      in.location = VERT_ATTRIB_GENERIC0;
      in.num_slots = 1;
      p = nir_load_input(&b, 4, 32, zero, .base = 0, .dest_type = nir_type_float32, .io_semantics = in);
      sem.location = VARYING_SLOT_POS;
      nir_store_output(&b, p, zero, .base = 0, .write_mask = 0xF, .src_type = nir_type_float32, .io_semantics = sem);
   } else {
      nir_def *c = nir_load_uniform(&b, 4, 32, zero, .base = 0, .range = 1, .dest_type = nir_type_float32);

      sem.location = FRAG_RESULT_DATA0;
      nir_store_output(&b, c, zero, .base = 0, .write_mask = 0xF, .src_type = nir_type_float32, .io_semantics = sem);
   }
   return b.shader;
}

static void
fg_clear_programs(struct fg_context *ctx)
{
   struct fg_screen *screen = fg_screen(ctx->base.screen);
   struct fg_compile_key key = { 0 };
   nir_shader *s;
   char err[512];

   ctx->clear_vs.off = ~0u;
   ctx->clear_fs.off = ~0u;
   s = fg_clear_nir(screen->base.nir_options[MESA_SHADER_VERTEX], false);
   key.noops = fg_noops(screen->a_flags);
   if (fg_compile(s, &key, &ctx->clear_vs.prog, err, sizeof(err)))
      fprintf(stderr, "fpgagpu: the clear's vertex program: %s\n", err);
   else
      ctx->clear_vs.ok = true;
   ralloc_free(s);
   s = fg_clear_nir(screen->base.nir_options[MESA_SHADER_FRAGMENT], true);
   key.noops = fg_noops(screen->b_flags);
   key.xops = screen->b_flags >> 28 & 3u;
   if (fg_compile(s, &key, &ctx->clear_fs.prog, err, sizeof(err)))
      fprintf(stderr, "fpgagpu: the clear's fragment program: %s\n", err);
   else
      ctx->clear_fs.ok = true;
   ralloc_free(s);
}

/* ------------------------------------------------------------------ the messages of a draw */

void
fg_hw_invalidate(struct fg_context *ctx)
{
   ctx->hw_prog = NULL;
   ctx->hw_attr_ok = false;
   memset(ctx->hw_uni_ok, 0, sizeof(ctx->hw_uni_ok));
   ctx->hw_vs_state_ok = false;
   memset(ctx->ids, 0, sizeof(ctx->ids));
   ctx->next_id = 1;
}

/* a SUBMIT failed: the boards did not get what it carried, so every program is placed again (from the start of the
 * program buffers) and every cached state is sent again */
void
fg_hw_lost(struct fg_context *ctx)
{
   fg_hw_invalidate(ctx);
   ctx->prog_a.used = 0;
   ctx->prog_b.used = 0;
   ctx->prog_gen++;
}

/* the boards hold one context's state: taking them from another forgets what this one sent before */
static void
fg_claim_boards(struct fg_context *ctx)
{
   struct fg_screen *screen = fg_screen(ctx->base.screen);

   if (screen->last_ctx != ctx) {
      fg_hw_invalidate(ctx);
      screen->last_ctx = ctx;
   }
}

/* VS_STATE: viewport (FP24), {cull, front clockwise, line width}, fragment inputs, {z 0..w, coarse-depth flags},
 * target size, draw id, depth scale and offset, the guard band's NDC bounds, the draw's rectangle */
static void
fg_vs_state_bytes(uint8_t b[60], const double vp[4], int cull, bool front_cw, double line_width, uint32_t fsmask,
                  bool z01, uint32_t a_flags, uint32_t width, uint32_t height, uint32_t draw_id, double dscale,
                  double doffset, const uint32_t rect[4])
{
   int lw = (int)rint(line_width);

   lw = CLAMP(lw, 1, 64);
   for (unsigned i = 0; i < 4; i++)
      fg_put32(b + 4 * i, fg_fp24(vp[i]));
   fg_put16(b + 16, (uint32_t)cull | (front_cw ? 1u << 8 : 0) | (uint32_t)lw << 9);
   b[18] = (uint8_t)fsmask;
   b[19] = (uint8_t)((z01 ? 1u : 0) | (a_flags & 0x1Fu) << 1);
   fg_put16(b + 20, width);
   fg_put16(b + 22, height);
   fg_put32(b + 24, draw_id);
   fg_put32(b + 28, fg_fp24(dscale));
   fg_put32(b + 32, fg_fp24(doffset));
   for (unsigned i = 0; i < 2; i++) {
      double s = vp[2 * i], o = vp[2 * i + 1];
      double lo = s ? (-FG_GUARD_BAND - o) / s : -1e30, hi = s ? (FG_GUARD_BAND - o) / s : 1e30;

      fg_put32(b + 36 + 8 * i, fg_fp24(lo < hi ? lo : hi));
      fg_put32(b + 40 + 8 * i, fg_fp24(lo < hi ? hi : lo));
   }
   fg_put16(b + 52, rect ? rect[0] : 0);
   fg_put16(b + 54, rect ? rect[1] : 0);
   fg_put16(b + 56, rect ? rect[2] : width);
   fg_put16(b + 58, rect ? rect[3] : height);
}

static void
fg_send_vs_state(struct fg_context *ctx, const uint8_t b[60])
{
   if (ctx->hw_vs_state_ok && !memcmp(ctx->hw_vs_state, b, 60))
      return;
   fg_emit(ctx, FG_T_VS_STATE, b, 60, NULL, 0);
   memcpy(ctx->hw_vs_state, b, 60);
   ctx->hw_vs_state_ok = true;
}

/* board A learns the target's size from VS_STATE and ends a frame's bands by it: a pass that is only a clear
 * would cover the last target's rows */
void fg_send_target_size(struct fg_context *ctx);
void
fg_send_target_size(struct fg_context *ctx)
{
   double vp[4] = { ctx->tgt.width * 8.0, ctx->tgt.width * 8.0, ctx->tgt.height * 8.0, ctx->tgt.height * 8.0 };
   uint8_t b[60];

   fg_claim_boards(ctx);
   if (ctx->hw_vs_state_ok && (ctx->hw_vs_state[20] | ctx->hw_vs_state[21] << 8) == ctx->tgt.width &&
       (ctx->hw_vs_state[22] | ctx->hw_vs_state[23] << 8) == ctx->tgt.height)
      return;
   fg_vs_state_bytes(b, vp, 0, false, 1.0, 0, false, FG_ZC_LAZY, ctx->tgt.width, ctx->tgt.height, 0, 0.5, 0.5, NULL);
   fg_send_vs_state(ctx, b);
}

/* board A's coarse depth: what the draw promises (g2_render.c zc_flags). A wrong bit culls visible quads */
static uint32_t
fg_zc_flags(const uint32_t *h, const struct fg_fs_variant *fs)
{
   uint32_t func = h[0] >> 2 & 7, z = 0;
   bool test = h[0] & 1u, write = h[0] & 2u, stencil = h[0] & 32u, offset = h[0] & 512u;
   bool discard = fs->prog.discard, depth = fs->prog.writes_depth;

   if (write && (!test || func >= FG_FN_GREATER))
      return FG_ZC_INV;
   if (!test)
      return 0;
   if ((func == FG_FN_LESS || func == FG_FN_LEQUAL) && write && !(stencil || discard || depth || offset))
      z |= FG_ZC_OCC;
   if (func >= FG_FN_LESS && func <= FG_FN_LEQUAL && !(stencil || depth || offset))
      z |= FG_ZC_CULL;
   if (z && (func == FG_FN_EQUAL || func == FG_FN_LEQUAL))
      z |= FG_ZC_STRICT;
   return z;
}

static uint64_t
fg_fnv(uint64_t h, const void *p, size_t n)
{
   const uint8_t *b = p;

   for (size_t i = 0; i < n; i++) {
      h ^= b[i];
      h *= 0x100000001B3ull;
   }
   return h;
}

/* the id of a DRAW_STATE with these words and uniforms, sent when it is new */
static uint32_t
fg_draw_id(struct fg_context *ctx, const uint32_t h[23], const uint32_t *uni, uint32_t nuni)
{
   struct fg_screen *screen = fg_screen(ctx->base.screen);
   uint32_t gen = p_atomic_read(&screen->img_gen), i, n;
   uint64_t hash;
   uint8_t hdr[4 + 23 * 4];

   /* the generations go through the hash as data (XORed into the seed they would cancel against h0) */
   hash = fg_fnv(fg_fnv(fg_fnv(fg_fnv(0xCBF29CE484222325ull, &ctx->id_gen, 4), &gen, 4), h, 23 * 4), uni,
                 (size_t)nuni * 16);
   if (!hash)
      hash = 1;
   i = (uint32_t)hash & (FG_ID_SLOTS - 1);
   for (n = 0; n < FG_ID_SLOTS; n++) {
      if (ctx->ids[i].hash == hash)
         return ctx->ids[i].id;
      if (!ctx->ids[i].hash)
         break;
      i = (i + 1) & (FG_ID_SLOTS - 1);
   }
   if (ctx->next_id >= FG_MAX_IDS || n == FG_ID_SLOTS) {
      /* every id taken: the open pass may still need them, so it ends first */
      fg_end_pass(ctx);
      fg_pass_open(ctx);
      memset(ctx->ids, 0, sizeof(ctx->ids));
      ctx->next_id = 1;
      i = (uint32_t)hash & (FG_ID_SLOTS - 1);
   }
   ctx->ids[i].hash = hash;
   ctx->ids[i].id = ctx->next_id++;
   fg_put32(hdr, ctx->ids[i].id & 0x1FFF);
   for (unsigned k = 0; k < 23; k++)
      fg_put32(hdr + 4 + 4 * k, h[k]);
   fg_emit(ctx, FG_T_DRAW_STATE, hdr, sizeof(hdr), uni, nuni * 16);
   return ctx->ids[i].id;
}

static void
fg_send_uniforms(struct fg_context *ctx, const uint32_t *u, uint32_t n)
{
   uint32_t i = 0;

   while (i < n) {
      uint32_t k;
      uint8_t hdr[4];

      if (ctx->hw_uni_ok[i] && !memcmp(ctx->hw_uni[i], u + 4 * i, 16)) {
         i++;
         continue;
      }
      k = i;
      while (k < n && !(ctx->hw_uni_ok[k] && !memcmp(ctx->hw_uni[k], u + 4 * k, 16)) && k - i < 80) {
         memcpy(ctx->hw_uni[k], u + 4 * k, 16);
         ctx->hw_uni_ok[k] = 1;
         k++;
      }
      fg_put16(hdr, i);
      fg_put16(hdr + 2, k - i);
      fg_emit(ctx, FG_T_VS_UNIFORMS, hdr, sizeof(hdr), u + 4 * i, (k - i) * 16);
      i = k;
   }
}

/* one draw's messages, everything but the vertices, which are in the ring already (g2r_draw) */
static void
fg_hw_draw(struct fg_context *ctx, const struct fg_hw_draw *d, unsigned mode, uint32_t count, bool indexed,
           uint32_t ioff)
{
   const struct fg_vs_state *vs = d->vs;
   const struct fg_fs_variant *fs = d->fs;
   uint32_t vu[4 * FG_VS_PARAMS], fu[4 * FG_FS_PARAMS], h[23], id, zc, nv, nf, i;
   uint8_t b[60];
   uint32_t vs_off = vs->off, fs_off = fs->off, vs_gen = vs->off_gen, fs_gen = fs->off_gen;

   nv = vs->prog.const_base + vs->prog.nconsts;
   nf = fs->prog.const_base + fs->prog.nconsts;
   if (nv > FG_VS_PARAMS || nf > FG_FS_PARAMS)
      return;
   fg_claim_boards(ctx);
   fg_pass_open(ctx);
   if (!fg_place(ctx, &ctx->prog_a, &vs->prog, &vs_off, &vs_gen) ||
       !fg_place(ctx, &ctx->prog_b, &fs->prog, &fs_off, &fs_gen))
      return;
   ((struct fg_vs_state *)vs)->off = vs_off;
   ((struct fg_vs_state *)vs)->off_gen = vs_gen;
   ((struct fg_fs_variant *)fs)->off = fs_off;
   ((struct fg_fs_variant *)fs)->off_gen = fs_gen;
   if (d->h[0] & 32u)
      ctx->stencil_known = false; /* a draw may write the stencil */

   /* board A: the program, attributes, uniforms */
   if (ctx->hw_prog != vs) {
      uint8_t p[12];

      fg_put16(p, ctx->prog_a.handle);
      fg_put16(p + 2, 0);
      fg_put32(p + 4, vs_off);
      fg_put32(p + 8, vs->prog.nwords);
      fg_emit(ctx, FG_T_VS_PROGRAM, p, sizeof(p), NULL, 0);
      fg_use_bo(ctx, ctx->prog_a.gem);
      ctx->hw_prog = vs;
   }
   for (i = 0; i < FG_MAX_ATTRIBS; i++) {
      if (ctx->hw_attr_ok && !memcmp(ctx->hw_attr[i], d->attr[i], 32))
         continue;
      fg_emit(ctx, FG_T_VERTEX_ATTR, d->attr[i], 32, NULL, 0);
      memcpy(ctx->hw_attr[i], d->attr[i], 32);
   }
   ctx->hw_attr_ok = true;
   if (vs->prog.const_base)
      memcpy(vu, d->vs_uni, vs->prog.const_base * 16);
   memcpy(vu + 4 * vs->prog.const_base, vs->prog.consts, vs->prog.nconsts * 16);
   fg_send_uniforms(ctx, vu, nv);

   /* board B: the draw's state, by id */
   memcpy(h, d->h, sizeof(h));
   h[11] = fs->prog.nwords << 16 | ctx->prog_b.handle;
   h[12] = fs_off;
   h[13] = nf;
   if (fs->prog.const_base)
      memcpy(fu, d->fs_uni, fs->prog.const_base * 16);
   memcpy(fu + 4 * fs->prog.const_base, fs->prog.consts, fs->prog.nconsts * 16);
   fg_use_bo(ctx, ctx->prog_b.gem);
   id = fg_draw_id(ctx, h, fu, nf);

   /* board A: viewport, culling, the draw id, the coarse-depth flags from the state B has under this id; its
    * rectangle is the scissor box of that state when the scissor test is on, so the boards cannot disagree */
   zc = fg_zc_flags(h, fs) | FG_ZC_LAZY;
   {
      uint32_t rect[4] = { h[7] & 0xFFFFu, h[7] >> 16, h[8] & 0xFFFFu, h[8] >> 16 };

      fg_vs_state_bytes(b, d->vp, d->cull, d->front_cw, d->line_width, fs->prog.varyings, d->z01, zc,
                        ctx->tgt.width, ctx->tgt.height, id, d->dscale, d->doffset, (h[0] & 256u) ? rect : NULL);
   }
   fg_send_vs_state(ctx, b);

   {
      uint8_t dr[20];

      dr[0] = (uint8_t)mode;
      dr[1] = indexed ? 1 : 0;
      dr[2] = 0;                     /* 16-bit indices */
      dr[3] = 0;
      fg_put16(dr + 4, indexed ? ctx->ring.handle : 0);
      fg_put16(dr + 6, 0);
      fg_put32(dr + 8, ioff);
      fg_put32(dr + 12, 0);          /* first */
      fg_put32(dr + 16, count);
      fg_emit(ctx, FG_T_DRAW, dr, sizeof(dr), NULL, 0);
   }
   fg_use_bo(ctx, ctx->ring.gem);
   ctx->pass_draws++;
}

/* ------------------------------------------------------------------ clears inside a pass */

void
fg_clear_quad(struct fg_context *ctx, uint32_t flags, const uint32_t c16[4], uint32_t depth, uint32_t stencil,
              uint32_t smask, const uint32_t sc[4])
{
   struct fg_hw_draw d;
   bool de = flags & FG_FL_CLEAR_DEPTH, se = flags & FG_FL_CLEAR_STENCIL, ce = flags & FG_FL_CLEAR_COLOR;
   uint32_t x0 = 0, y0 = 0, x1 = ctx->tgt.width, y1 = ctx->tgt.height, fu[4] = { 0, 0, 0, 0 }, one[4] = { 0, 0, 0, 0 };
   float z = depth ? 1.0f : -1.0f, v[16];
   uint32_t off;

   if (!ctx->clear_vs.ok || !ctx->clear_fs.ok)
      return;
   if (flags & FG_FL_CLEAR_SCISSOR) {
      x0 = sc[0];
      y0 = sc[1];
      x1 = MIN2(sc[2], ctx->tgt.width);
      y1 = MIN2(sc[3], ctx->tgt.height);
   }
   if (x1 <= x0 || y1 <= y0)
      return;
   memset(&d, 0, sizeof(d));
   d.vs = &ctx->clear_vs;
   d.fs = &ctx->clear_fs;
   /* the quad's viewport is the clear's rectangle: no pixel outside it is walked or shaded */
   d.vp[0] = (x1 - x0) * 8.0;
   d.vp[1] = (x0 + (x1 - x0) / 2.0) * 16.0;
   d.vp[2] = (y1 - y0) * 8.0;
   d.vp[3] = (y0 + (y1 - y0) / 2.0) * 16.0;
   d.dscale = 0.5;
   d.doffset = 0.5;
   d.line_width = 1.0;
   d.h[0] = (de ? 1u | 2u : 0) | (uint32_t)FG_FN_ALWAYS << 2 | (se ? 32u : 0) | 256u;
   if (ce) {
      d.h[0] |= (flags >> 2 & 0xFu) << 16 | ((flags & FG_FL_DITHER) ? 128u : 0);
      /* the program's output c16 / ONE (ONE = 16 (2^n - 1)) rounds to the clear's c16 for every channel width */
      for (unsigned i = 0; i < 4; i++)
         if (ctx->tgt.bits[i] || (i == 3 && ctx->tgt.no_alpha))
            fu[i] = fg_fp24((double)c16[i] / (16.0 * (double)((1u << (ctx->tgt.bits[i] ? ctx->tgt.bits[i] : 8)) - 1)));
   }
   d.fs_uni = fu;
   d.h[1] = d.h[2] = (stencil & 255) | 255u << 8 | (smask & 255) << 16 | (uint32_t)FG_FN_ALWAYS << 24;
   d.h[3] = 2u | 2u << 3 | 2u << 6 | 2u << 16 | 2u << 19 | 2u << 22; /* REPLACE */
   d.h[7] = x0 | y0 << 16;
   d.h[8] = x1 | y1 << 16;
   v[0] = -1; v[1] = -1; v[4] = 1; v[5] = -1; v[8] = -1; v[9] = 1; v[12] = 1; v[13] = 1;
   for (unsigned i = 0; i < 4; i++) {
      v[4 * i + 2] = z;
      v[4 * i + 3] = 1.0f;
   }
   one[3] = fg_fp24(1.0);
   for (unsigned i = 0; i < FG_MAX_ATTRIBS; i++)
      fg_attr_words(d.attr[i], i, false, 4, FG_AT_FP24, false, 0, 0, 0, one);
   off = fg_stream(ctx, v, sizeof(v), 4);
   fg_attr_words(d.attr[0], 0, true, 4, FG_AT_FLOAT, false, ctx->ring.handle, 16, off, one);
   fg_hw_draw(ctx, &d, 1, 4, false, 0);
}

/* ------------------------------------------------------------------ draw_vbo */

static const int8_t fg_hw_mode[] = {
   [MESA_PRIM_POINTS] = 3, [MESA_PRIM_LINES] = 4, [MESA_PRIM_LINE_LOOP] = 6, [MESA_PRIM_LINE_STRIP] = 5,
   [MESA_PRIM_TRIANGLES] = 0, [MESA_PRIM_TRIANGLE_STRIP] = 1, [MESA_PRIM_TRIANGLE_FAN] = 2,
};

static void
fg_cb_fp24(uint32_t *out, const uint8_t *cb, uint32_t cb_size, unsigned nvec)
{
   const float *f = (const float *)cb;

   for (unsigned i = 0; i < 4 * nvec; i++)
      out[i] = i * 4 < cb_size ? fg_fp24(f[i]) : 0;
}

/* DRAW_STATE's words from the bound state; the shader parts (h11-h13) are filled in by fg_hw_draw */
static bool
fg_fill_state(struct fg_context *ctx, struct fg_hw_draw *d, const struct fg_fs_variant *fs, unsigned mode)
{
   const struct pipe_rasterizer_state *rs = ctx->rast;
   const struct fg_target *t = &ctx->tgt;
   uint32_t *h = d->h, cmask;
   static const struct fg_dsa_state no_dsa;
   const struct fg_dsa_state *dsa = ctx->dsa ? ctx->dsa : &no_dsa;
   const struct pipe_viewport_state *vp = &ctx->viewport;
   int b[4];

   if (!rs || !ctx->blend)
      return false;
   cmask = ctx->blend->h0 >> 16 & 0xF;
   if (t->no_alpha)
      cmask &= 14; /* the board's order: an X format's alpha stays the 1.0 it was given */
   cmask &= fg_board_cmask(t->wmask); /* A8, R8, RG8: their own channels */
   h[0] = (ctx->blend->h0 & (64u | 128u)) | cmask << 16;
   if (t->ds)
      h[0] |= dsa->h0 & (1u | 2u | 0x1Cu);
   if (t->has_stencil)
      h[0] |= dsa->h0 & 32u;
   if (rs->scissor)
      h[0] |= 256u;
   if (rs->offset_tri)
      h[0] |= 512u;
   h[1] = dsa->h1 | CLAMP(ctx->stencil_ref.ref_value[0], 0, 255);
   h[2] = dsa->h2 | CLAMP(ctx->stencil_ref.ref_value[1], 0, 255);
   h[3] = dsa->h3;
   h[4] = ctx->blend->h4;
   h[5] = fg_c16(ctx->blend_color.color[0], t->bits[0]) | fg_c16(ctx->blend_color.color[1], t->bits[1]) << 13;
   h[6] = fg_c16(ctx->blend_color.color[2], t->bits[2]) | fg_c16(ctx->blend_color.color[3], t->bits[3] ? t->bits[3] : 8) << 13;
   h[14] = fg_c16(ctx->blend_color.color[3], t->bits[0]) | fg_c16(ctx->blend_color.color[3], t->bits[1]) << 13;
   b[0] = 0;
   b[1] = 0;
   b[2] = t->width;
   b[3] = t->height;
   if (rs->scissor) {
      b[0] = MIN2(ctx->scissor.minx, t->width);
      b[1] = MIN2(ctx->scissor.miny, t->height);
      b[2] = MIN2(ctx->scissor.maxx, t->width);
      b[3] = MIN2(ctx->scissor.maxy, t->height);
   }
   h[9] = fg_fp24(rs->offset_scale);
   h[10] = (uint32_t)(int32_t)rs->offset_units;
   for (unsigned u = 0; u < FG_MAX_SAMPLERS; u++) {
      struct pipe_sampler_view *view = ctx->views[u];
      const struct fg_sampler_state *s = ctx->samplers[u];
      struct fg_resource *r;
      uint32_t filter;

      if (!(fs->prog.samplers & (1u << u)))
         continue;
      if (!view || !view->texture || view->texture->target == PIPE_BUFFER || !s) {
         h[15 + u] = 1u << 24; /* incomplete: the TMU reads (0, 0, 0, 1) */
         continue;
      }
      r = fg_resource(view->texture);
      if (r->pitch != view->texture->width0 * fg_storage_bpp(r->fmt) && ctx->debug) {
         static unsigned shown;

         if (shown++ < 16)
            fprintf(stderr, "fpgagpu: sampling a %ux%u %s image whose rows are padded (pitch %u): the TMU reads tight rows\n",
                    view->texture->width0, view->texture->height0, util_format_name(view->texture->format), r->pitch);
      }
      filter = view->u.tex.last_level > view->u.tex.first_level ? s->filter : s->filter_nomip;
      h[15 + u] = r->board_handle | filter << 16 | s->wrap;
      fg_use_resource(ctx, view->texture);
   }

   /* board A clips x and y only at its guard band: the scissor keeps a draw to its viewport (a point goes by near
    * and far only) */
   if (mode != MESA_PRIM_POINTS) {
      double sx = fabs(vp->scale[0]), sy = fabs(vp->scale[1]);
      int x0 = (int)ceil(vp->translate[0] - sx - 0.5), y0 = (int)ceil(vp->translate[1] - sy - 0.5);
      int x1 = (int)ceil(vp->translate[0] + sx - 0.5), y1 = (int)ceil(vp->translate[1] + sy - 0.5);

      if (x0 > 0 || y0 > 0 || x1 < (int)t->width || y1 < (int)t->height) {
         b[0] = MAX2(b[0], x0);
         b[1] = MAX2(b[1], y0);
         b[2] = MIN2(b[2], x1);
         b[3] = MIN2(b[3], y1);
         if (b[2] < b[0])
            b[2] = b[0];
         if (b[3] < b[1])
            b[3] = b[1];
         h[0] |= 256u;
      }
   }
   h[7] = (uint32_t)b[0] | (uint32_t)b[1] << 16;
   h[8] = (uint32_t)b[2] | (uint32_t)b[3] << 16;

   d->vp[0] = vp->scale[0] * 16.0;
   d->vp[1] = vp->translate[0] * 16.0;
   d->vp[2] = vp->scale[1] * 16.0;
   d->vp[3] = vp->translate[1] * 16.0;
   d->dscale = vp->scale[2];
   d->doffset = vp->translate[2];
   d->z01 = rs->clip_halfz;
   d->cull = rs->cull_face == PIPE_FACE_BACK ? 1 : rs->cull_face == PIPE_FACE_FRONT ? 2 :
             rs->cull_face == PIPE_FACE_FRONT_AND_BACK ? 3 : 0;
   /* Gallium gives the winding in window coordinates with y down, board A takes it with y up (GL's, as the Windows
    * libraries send it): Gallium's counter-clockwise is the board's clockwise */
   d->front_cw = rs->front_ccw;
   d->line_width = rs->line_width;
   return true;
}

static const void *
fg_vb_data(const struct pipe_vertex_buffer *vb)
{
   if (vb->is_user_buffer)
      return vb->buffer.user;
   return vb->buffer.resource ? fg_resource(vb->buffer.resource)->data : NULL;
}

/* the vertices lo .. hi of every attribute into the ring; idx, when not NULL, picks n vertices instead (a draw
 * whose indices span more than 16 bits is drawn unindexed) */
static bool
fg_stream_vertices(struct fg_context *ctx, struct fg_hw_draw *d, int lo, int hi, const uint32_t *idx, uint32_t n)
{
   const struct fg_velems_state *ve = ctx->velems;
   uint32_t one[4] = { 0, 0, 0, fg_fp24(1.0) };
   bool done[FG_MAX_ATTRIBS] = { false };

   for (unsigned i = 0; i < FG_MAX_ATTRIBS; i++)
      fg_attr_words(d->attr[i], i, false, 4, FG_AT_FP24, false, 0, 0, 0, one);
   for (unsigned i = 0; i < ve->count; i++) {
      const struct fg_velem *v = &ve->v[i];
      const struct pipe_vertex_buffer *vb = &ctx->vb[v->e.vertex_buffer_index];
      const uint8_t *base = v->e.vertex_buffer_index < ctx->nvb ? fg_vb_data(vb) : NULL;
      uint32_t stride = v->e.src_stride, off, lo_off, hi_end;

      if (done[i] || !v->bytes || !base)
         continue;
      base += vb->buffer_offset;
      if (!stride) {
         /* the same value for every vertex: the attribute's constant */
         float f[4] = { 0, 0, 0, 1 };
         uint32_t c[4];

         util_format_unpack_rgba(v->e.src_format, f, base + v->e.src_offset, 1);
         for (unsigned k = 0; k < 4; k++)
            c[k] = fg_fp24(f[k]);
         fg_attr_words(d->attr[i], i, false, 4, FG_AT_FP24, false, 0, 0, 0, c);
         continue;
      }
      if (idx) {
         uint8_t *t = malloc((size_t)n * v->bytes);

         if (!t)
            return false;
         for (uint32_t k = 0; k < n; k++)
            memcpy(t + (size_t)k * v->bytes, base + (size_t)idx[k] * stride + v->e.src_offset, v->bytes);
         off = fg_stream(ctx, t, n * v->bytes, 4);
         free(t);
         fg_attr_words(d->attr[i], i, true, v->size, v->type, v->normalized, ctx->ring.handle, v->bytes, off, one);
         continue;
      }
      /* the attributes of one buffer with one stride go up together, interleaved as they are */
      lo_off = v->e.src_offset;
      hi_end = v->e.src_offset + v->bytes;
      for (unsigned j = i + 1; j < ve->count; j++) {
         const struct fg_velem *w = &ve->v[j];

         if (w->bytes && w->e.vertex_buffer_index == v->e.vertex_buffer_index && w->e.src_stride == stride) {
            lo_off = MIN2(lo_off, w->e.src_offset);
            hi_end = MAX2(hi_end, w->e.src_offset + w->bytes);
         }
      }
      off = fg_stream(ctx, base + (size_t)lo * stride + lo_off, (uint32_t)(hi - lo) * stride + (hi_end - lo_off), 4);
      for (unsigned j = i; j < ve->count; j++) {
         const struct fg_velem *w = &ve->v[j];

         if (j != i && !(w->bytes && w->e.vertex_buffer_index == v->e.vertex_buffer_index && w->e.src_stride == stride))
            continue;
         fg_attr_words(d->attr[j], j, true, w->size, w->type, w->normalized, ctx->ring.handle, stride,
                       off + (w->e.src_offset - lo_off), one);
         done[j] = true;
      }
   }
   return true;
}

static void
fg_draw_vbo(struct pipe_context *pctx, const struct pipe_draw_info *info, unsigned drawid_offset,
            const struct pipe_draw_indirect_info *indirect, const struct pipe_draw_start_count_bias *draws,
            unsigned num_draws)
{
   struct fg_context *ctx = fg_context(pctx);
   struct fg_vs_state *vs = ctx->vs;
   struct fg_fs_variant *fs;
   struct fg_hw_draw d;
   uint32_t vu[4 * FG_VS_PARAMS], fu[4 * FG_FS_PARAMS];

   if (indirect) {
      FG_NOTE_ONCE("indirect draws");
      return;
   }
   if (info->mode >= ARRAY_SIZE(fg_hw_mode) || (info->mode != MESA_PRIM_POINTS && !fg_hw_mode[info->mode] &&
                                                info->mode != MESA_PRIM_TRIANGLES)) {
      util_primconvert_draw_vbo(ctx->primconvert, info, drawid_offset, indirect, draws, num_draws);
      return;
   }
   if (info->primitive_restart && info->index_size) {
      util_primconvert_draw_vbo(ctx->primconvert, info, drawid_offset, indirect, draws, num_draws);
      return;
   }
   if (info->instance_count != 1)
      FG_NOTE_ONCE("instancing (each draw is drawn once)");
   if (!vs || !vs->ok || !ctx->fs || !ctx->velems || !fg_set_target(ctx))
      return;
   fs = fg_fs_variant(ctx, ctx->fs, vs);
   if (!fs)
      return;

   memset(&d, 0, sizeof(d));
   d.vs = vs;
   d.fs = fs;
   if (!fg_fill_state(ctx, &d, fs, info->mode))
      return;
   fg_cb_fp24(vu, ctx->cb[0], ctx->cb_size[0], vs->prog.const_base);
   fg_cb_fp24(fu, ctx->cb[1], ctx->cb_size[1], fs->prog.const_base);
   d.vs_uni = vu;
   d.fs_uni = fu;

   for (unsigned k = 0; k < num_draws; k++) {
      uint32_t count = draws[k].count, ioff = 0;
      int lo, hi;

      if (!count)
         continue;
      if (!info->index_size) {
         lo = draws[k].start;
         hi = lo + count - 1;
         if (!fg_stream_vertices(ctx, &d, lo, hi, NULL, 0))
            return;
         fg_hw_draw(ctx, &d, fg_hw_mode[info->mode], count, false, 0);
         continue;
      }
      {
         const uint8_t *ib = info->has_user_indices ? info->index.user :
                             (info->index.resource ? fg_resource(info->index.resource)->data : NULL);
         uint32_t *idx = malloc((size_t)count * sizeof(uint32_t));
         uint16_t *i16;

         if (!ib || !idx) {
            free(idx);
            return;
         }
         ib += (size_t)draws[k].start * info->index_size;
         lo = INT32_MAX;
         hi = INT32_MIN;
         for (uint32_t i = 0; i < count; i++) {
            int64_t v = info->index_size == 1 ? ib[i] : info->index_size == 2 ? ((const uint16_t *)ib)[i] :
                        ((const uint32_t *)ib)[i];

            v += draws[k].index_bias;
            if (v < 0)
               v = 0;
            idx[i] = (uint32_t)v;
            lo = MIN2(lo, (int)v);
            hi = MAX2(hi, (int)v);
         }
         if (hi - lo >= 65536) {
            /* the hardware's indices are 16 bits: the draw's vertices one by one */
            bool ok = fg_stream_vertices(ctx, &d, 0, 0, idx, count);

            free(idx);
            if (ok)
               fg_hw_draw(ctx, &d, fg_hw_mode[info->mode], count, false, 0);
            continue;
         }
         if (!fg_stream_vertices(ctx, &d, lo, hi, NULL, 0)) {
            free(idx);
            return;
         }
         i16 = (uint16_t *)idx; /* in place: each 16-bit index is written behind the 32-bit one it comes from */
         for (uint32_t i = 0; i < count; i++)
            i16[i] = (uint16_t)(idx[i] - lo);
         ioff = fg_stream(ctx, i16, count * 2, 4);
         free(idx);
         fg_hw_draw(ctx, &d, fg_hw_mode[info->mode], count, true, ioff);
      }
   }
}

static void
fg_draw_vertex_state(struct pipe_context *pctx, struct pipe_vertex_state *state, uint32_t partial_velem_mask,
                     struct pipe_draw_vertex_state_info info, const struct pipe_draw_start_count_bias *draws,
                     unsigned num_draws)
{
   FG_NOTE_ONCE("drawing with a vertex state object");
}

/* ------------------------------------------------------------------ state setters */

static void
fg_set_vertex_buffers(struct pipe_context *pctx, unsigned count, const struct pipe_vertex_buffer *buffers)
{
   struct fg_context *ctx = fg_context(pctx);

   util_set_vertex_buffers_count(ctx->vb, &ctx->nvb, buffers, MIN2(count, PIPE_MAX_ATTRIBS));
}

static void
fg_set_constant_buffer(struct pipe_context *pctx, mesa_shader_stage shader, uint index,
                       const struct pipe_constant_buffer *cb)
{
   struct fg_context *ctx = fg_context(pctx);
   unsigned s = shader == MESA_SHADER_FRAGMENT ? 1 : 0;
   const uint8_t *src = NULL;
   uint32_t n = 0;

   if (index || (shader != MESA_SHADER_VERTEX && shader != MESA_SHADER_FRAGMENT))
      return;
   if (cb) {
      src = cb->user_buffer ? cb->user_buffer :
            cb->buffer ? (const uint8_t *)fg_resource(cb->buffer)->data + cb->buffer_offset : NULL;
      n = MIN2(cb->buffer_size, sizeof(ctx->cb[s]));
   }
   if (src && n)
      memcpy(ctx->cb[s], src, n);
   ctx->cb_size[s] = src ? n : 0;
}

static void
fg_set_viewport_states(struct pipe_context *pctx, unsigned start_slot, unsigned num_viewports,
                       const struct pipe_viewport_state *state)
{
   if (start_slot == 0 && num_viewports)
      fg_context(pctx)->viewport = state[0];
}

static void
fg_set_scissor_states(struct pipe_context *pctx, unsigned start_slot, unsigned num_scissors,
                      const struct pipe_scissor_state *state)
{
   if (start_slot == 0 && num_scissors)
      fg_context(pctx)->scissor = state[0];
}

static void
fg_set_blend_color(struct pipe_context *pctx, const struct pipe_blend_color *state)
{
   fg_context(pctx)->blend_color = *state;
}

static void
fg_set_stencil_ref(struct pipe_context *pctx, const struct pipe_stencil_ref state)
{
   fg_context(pctx)->stencil_ref = state;
}

static void *
fg_create_compute_state(struct pipe_context *pctx, const struct pipe_compute_state *cso)
{
   return CALLOC_STRUCT(fg_fs_state);
}

static void
fg_bind_compute_state(struct pipe_context *pctx, void *hwcso)
{
}

static void
fg_delete_compute_state(struct pipe_context *pctx, void *hwcso)
{
   FREE(hwcso);
}

/* ------------------------------------------------------------------ init */

void
fg_draw_init(struct pipe_context *pctx)
{
   struct fg_context *ctx = fg_context(pctx);

   ctx->primconvert = util_primconvert_create(pctx, BITFIELD_MASK(MESA_PRIM_TRIANGLE_FAN + 1));
   fg_clear_programs(ctx);

   pctx->draw_vbo = fg_draw_vbo;
   pctx->draw_vertex_state = fg_draw_vertex_state;
   pctx->create_blend_state = fg_create_blend_state;
   pctx->bind_blend_state = fg_bind_blend_state;
   pctx->delete_blend_state = fg_delete_blend_state;
   pctx->create_depth_stencil_alpha_state = fg_create_dsa_state;
   pctx->bind_depth_stencil_alpha_state = fg_bind_dsa_state;
   pctx->delete_depth_stencil_alpha_state = fg_delete_dsa_state;
   pctx->create_rasterizer_state = fg_create_rs_state;
   pctx->bind_rasterizer_state = fg_bind_rs_state;
   pctx->delete_rasterizer_state = fg_delete_rs_state;
   pctx->create_sampler_state = fg_create_sampler_state;
   pctx->bind_sampler_states = fg_bind_sampler_states;
   pctx->delete_sampler_state = fg_delete_sampler_state;
   pctx->create_vertex_elements_state = fg_create_vertex_elements;
   pctx->bind_vertex_elements_state = fg_bind_vertex_elements;
   pctx->delete_vertex_elements_state = fg_delete_vertex_elements;
   pctx->create_vs_state = fg_create_vs_state;
   pctx->bind_vs_state = fg_bind_vs_state;
   pctx->delete_vs_state = fg_delete_vs_state;
   pctx->create_fs_state = fg_create_fs_state;
   pctx->bind_fs_state = fg_bind_fs_state;
   pctx->delete_fs_state = fg_delete_fs_state;
   pctx->create_compute_state = fg_create_compute_state;
   pctx->bind_compute_state = fg_bind_compute_state;
   pctx->delete_compute_state = fg_delete_compute_state;
   pctx->create_sampler_view = fg_create_sampler_view;
   pctx->sampler_view_destroy = fg_sampler_view_destroy;
   pctx->sampler_view_release = u_default_sampler_view_release;
   pctx->set_sampler_views = fg_set_sampler_views;
   pctx->set_vertex_buffers = fg_set_vertex_buffers;
   pctx->set_constant_buffer = fg_set_constant_buffer;
   pctx->set_viewport_states = fg_set_viewport_states;
   pctx->set_scissor_states = fg_set_scissor_states;
   pctx->set_blend_color = fg_set_blend_color;
   pctx->set_stencil_ref = fg_set_stencil_ref;
}

void
fg_draw_fini(struct fg_context *ctx)
{
   for (unsigned i = 0; i < FG_MAX_SAMPLERS; i++)
      pipe_sampler_view_reference(&ctx->views[i], NULL);
   util_set_vertex_buffers_count(ctx->vb, &ctx->nvb, NULL, 0);
   if (ctx->primconvert)
      util_primconvert_destroy(ctx->primconvert);
   fg_program_free(&ctx->clear_vs.prog);
   fg_program_free(&ctx->clear_fs.prog);
}
