/* SPDX-License-Identifier: MIT */
/*
 * fg_compiler: NIR to the programs of the fpgagpu cores.
 *
 * The cores run vec4 programs (isa64) on 32 registers, with floats only (FP24), predicated IF / ELSE / ENDIF and
 * LOOP / BREAK / ENDL, and a parameter file holding the uniforms and then the constants. The NIR is lowered to float
 * code with registers instead of phis (fg_nir_lower); this file turns it into the assembler lines of the compiler the
 * Windows libraries use (host/win/shc), over virtual registers, and that compiler's back end does the rest: copy
 * propagation, CSE, MAD fusion, the register allocation that packs scalars into lanes, and the assembler.
 *
 * Conventions of those lines (shc_gen.c emits the same):
 *   - a value of one component lives in a scalar register (packed into a lane later; any swizzle of it reads that
 *     lane); a vector value in a whole register, written without a mask (lanes past its size are don't-care)
 *   - a source swizzle gives the component each lane reads, 1-3 letters repeat the last
 *   - a constant is "LDP rN @k", k its vector in the pool; a uniform "LDP rN slot"; an input "LDV rN slot"
 *   - booleans are 1.0 / 0.0; SET <cc> d a b compares, SEL d c a b = c != 0 ? a : b
 *   - if: SETP SNE c 0; IF Lelse; ...; Lelse:; ELSE Lend; ...; Lend:; ENDIF. A BREAK / CONT acts in the lanes
 *     the last SETP holds
 *   - outputs are kept in registers and stored once at the end: STO 0 position / colour, STO 1 point size,
 *     STOZ depth, STV n varying n
 */
#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fg_compiler.h"
#include "compiler/nir/nir_builder.h"
#include "util/bitscan.h"

#include "shc_internal.h"

#define IF_DEPTH   8 /* the cores' IF and LOOP nesting */
#define LOOP_DEPTH 4

uint32_t fg_fp24(double x)
{
   uint32_t s;
   int e;
   double m, q;

   if (x == 0.0)
      return 0;
   if (isinf(x))
      return (0x7Fu << 16) | (x < 0 ? 0x800000u : 0);
   if (isnan(x))
      return (0x7Fu << 16) | 0x8000u;
   s = x < 0 ? 0x800000u : 0;
   m = frexp(fabs(x), &e); /* |x| = m 2^e, m in [0.5, 1) */
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

uint64_t fg_varying_mask(uint64_t outputs_written)
{
   return outputs_written & ~(BITFIELD64_BIT(VARYING_SLOT_POS) | BITFIELD64_BIT(VARYING_SLOT_PSIZ) |
                              BITFIELD64_BIT(VARYING_SLOT_CLIP_VERTEX) | BITFIELD64_BIT(VARYING_SLOT_CLIP_DIST0) |
                              BITFIELD64_BIT(VARYING_SLOT_CLIP_DIST1) | BITFIELD64_BIT(VARYING_SLOT_EDGE));
}

/* ------------------------------------------------------------------ lowering */

static unsigned fg_type_size(const struct glsl_type *type, bool bindless)
{
   (void)bindless;
   return glsl_count_attribute_slots(type, false);
}

/* the vector comparisons nir_lower_bool_to_float cannot take */
static bool fg_scalarize_filter(const nir_instr *instr, const void *data)
{
   (void)data;
   if (instr->type != nir_instr_type_alu)
      return false;
   switch (nir_instr_as_alu(instr)->op) {
   case nir_op_ball_fequal2: case nir_op_ball_fequal3: case nir_op_ball_fequal4:
   case nir_op_bany_fnequal2: case nir_op_bany_fnequal3: case nir_op_bany_fnequal4:
   case nir_op_ball_iequal2: case nir_op_ball_iequal3: case nir_op_ball_iequal4:
   case nir_op_bany_inequal2: case nir_op_bany_inequal3: case nir_op_bany_inequal4:
      return true;
   default:
      return false;
   }
}

static bool fg_optimize_once(nir_shader *s)
{
   bool progress = false;
   nir_opt_peephole_select_options ps = {
      .limit = 8,
      .indirect_load_ok = true,
      .expensive_alu_ok = true,
   };

   NIR_PASS(_, s, nir_lower_vars_to_ssa);
   NIR_PASS(progress, s, nir_opt_copy_prop);
   NIR_PASS(progress, s, nir_opt_remove_phis);
   NIR_PASS(progress, s, nir_opt_dce);
   NIR_PASS(progress, s, nir_opt_dead_cf);
   NIR_PASS(progress, s, nir_opt_cse);
   NIR_PASS(progress, s, nir_opt_peephole_select, &ps);
   NIR_PASS(progress, s, nir_opt_algebraic);
   NIR_PASS(progress, s, nir_opt_constant_folding);
   NIR_PASS(progress, s, nir_opt_undef);
   NIR_PASS(progress, s, nir_opt_loop_unroll);
   return progress;
}

/* ------------------------------------------------------------------ outputs on the CPU */

enum { EV_CONST, EV_INPUT, EV_UNIFORM, EV_ALU, EV_OUT };

struct fg_eval_op {
   uint8_t kind;
   uint8_t nc;                  /* components of the value it defines (EV_OUT: of the value it stores) */
   uint8_t nsrc;
   uint8_t mask;                /* EV_OUT: the components it stores */
   uint16_t op;                 /* EV_ALU: the nir_op */
   uint32_t dst;                /* the value it defines (SSA index) */
   uint32_t src[3];
   uint8_t swz[3][4];
   int base;                    /* EV_INPUT: the attribute; EV_UNIFORM: the vec4 slot; EV_OUT: the first component */
   uint8_t comp;                /* EV_INPUT: the attribute's first component */
   float k[4];                  /* EV_CONST */
};

struct fg_eval {
   struct fg_eval_op *ops;
   unsigned nops;
   float (*v)[4];               /* one vec4 per SSA value (scratch) */
   uint32_t inputs;
};

static bool
fg_eval_op_ok(nir_op op)
{
   switch (op) {
   case nir_op_mov: case nir_op_fneg: case nir_op_fabs: case nir_op_fsat: case nir_op_fadd: case nir_op_fsub:
   case nir_op_fmul: case nir_op_ffma: case nir_op_fmin: case nir_op_fmax: case nir_op_frcp: case nir_op_frsq:
   case nir_op_fsqrt: case nir_op_fdiv: case nir_op_ffloor: case nir_op_fceil: case nir_op_ffract:
   case nir_op_ftrunc: case nir_op_fround_even: case nir_op_fsign: case nir_op_fexp2: case nir_op_flog2:
   case nir_op_fpow: case nir_op_fsin: case nir_op_fcos: case nir_op_slt: case nir_op_sge: case nir_op_seq:
   case nir_op_sne: case nir_op_fcsel: case nir_op_flrp: case nir_op_b2f32: case nir_op_fdot2: case nir_op_fdot3:
   case nir_op_fdot4: case nir_op_fdph: case nir_op_vec2: case nir_op_vec3: case nir_op_vec4:
      return true;
   default:
      return false;
   }
}

/* the instructions the output at this location depends on, from the SSA form of a vertex program (one block, after
 * the optimising passes); NULL when it depends on something else: control flow, a uniform at a computed index, an
 * operation above */
struct fg_eval *
fg_eval_create(nir_shader *s, unsigned location)
{
   nir_function_impl *impl = nir_shader_get_entrypoint(s);
   struct fg_eval *e = NULL;
   BITSET_WORD *need = NULL;
   nir_instr **kept = NULL;
   unsigned nkept = 0, n = 0;
   bool pos = false;

   if (!impl || s->info.stage != MESA_SHADER_VERTEX || exec_list_length(&impl->body) != 1)
      return NULL;
   nir_index_ssa_defs(impl);
   nir_foreach_instr(instr, nir_start_block(impl))
      n++;
   need = calloc(BITSET_WORDS(impl->ssa_alloc) + 1, sizeof(BITSET_WORD));
   kept = calloc(n + 1, sizeof(*kept));
   if (!need || !kept)
      goto fail;
   nir_foreach_instr_reverse(instr, nir_start_block(impl)) {
      nir_def *def = nir_instr_def(instr);
      bool needed = def && BITSET_TEST(need, def->index);

      if (instr->type == nir_instr_type_intrinsic) {
         nir_intrinsic_instr *in = nir_instr_as_intrinsic(instr);

         if (in->intrinsic == nir_intrinsic_store_output && nir_intrinsic_io_semantics(in).location == location) {
            if (!nir_src_is_const(in->src[1]) || nir_src_as_uint(in->src[1]) || in->src[0].ssa->bit_size != 32)
               goto fail;
            BITSET_SET(need, in->src[0].ssa->index);
            kept[nkept++] = instr;
            pos = true;
            continue;
         }
         if (!needed)
            continue;
         if (in->intrinsic == nir_intrinsic_load_input) {
            if (!nir_src_is_const(in->src[0]) || nir_src_as_uint(in->src[0]) ||
                nir_intrinsic_base(in) >= 16 || nir_intrinsic_component(in) + in->def.num_components > 4)
               goto fail;
         } else if (in->intrinsic != nir_intrinsic_load_uniform || !nir_src_is_const(in->src[0])) {
            goto fail;
         }
         kept[nkept++] = instr;
         continue;
      }
      if (!needed)
         continue;
      if (def->bit_size != 32 || def->num_components > 4)
         goto fail;
      if (instr->type == nir_instr_type_load_const || instr->type == nir_instr_type_undef) {
         kept[nkept++] = instr;
      } else if (instr->type == nir_instr_type_alu && fg_eval_op_ok(nir_instr_as_alu(instr)->op)) {
         nir_alu_instr *alu = nir_instr_as_alu(instr);

         for (unsigned i = 0; i < nir_op_infos[alu->op].num_inputs; i++)
            BITSET_SET(need, alu->src[i].src.ssa->index);
         kept[nkept++] = instr;
      } else {
         goto fail;
      }
   }
   if (!pos)
      goto fail;
   e = calloc(1, sizeof(*e));
   if (!e || !(e->ops = calloc(nkept, sizeof(*e->ops))) || !(e->v = calloc(impl->ssa_alloc + 1, sizeof(*e->v))))
      goto fail;
   /* in program order */
   for (unsigned i = nkept; i-- > 0;) {
      nir_instr *instr = kept[i];
      struct fg_eval_op *o = &e->ops[e->nops++];
      nir_def *def = nir_instr_def(instr);

      if (def) {
         o->dst = def->index;
         o->nc = def->num_components;
      }
      switch (instr->type) {
      case nir_instr_type_load_const:
         o->kind = EV_CONST;
         for (unsigned c = 0; c < o->nc; c++)
            o->k[c] = nir_instr_as_load_const(instr)->value[c].f32;
         break;
      case nir_instr_type_undef:
         o->kind = EV_CONST;
         break;
      case nir_instr_type_intrinsic: {
         nir_intrinsic_instr *in = nir_instr_as_intrinsic(instr);

         if (in->intrinsic == nir_intrinsic_store_output) {
            o->kind = EV_OUT;
            o->nc = in->src[0].ssa->num_components;
            o->src[0] = in->src[0].ssa->index;
            o->base = nir_intrinsic_component(in);
            o->mask = nir_intrinsic_write_mask(in);
         } else if (in->intrinsic == nir_intrinsic_load_input) {
            o->kind = EV_INPUT;
            o->base = nir_intrinsic_base(in);
            o->comp = nir_intrinsic_component(in);
            e->inputs |= 1u << o->base;
         } else {
            /* integers are floats by now (nir_lower_int_to_float): the offset constant is a float, as the compiler
             * reads it */
            o->kind = EV_UNIFORM;
            o->base = nir_intrinsic_base(in) + (int)nir_src_as_float(in->src[0]);
            if (o->base < 0)
               goto fail;
         }
         break;
      }
      default: {
         nir_alu_instr *alu = nir_instr_as_alu(instr);

         o->kind = EV_ALU;
         o->op = alu->op;
         o->nsrc = nir_op_infos[alu->op].num_inputs;
         for (unsigned j = 0; j < o->nsrc && j < 3; j++) {
            o->src[j] = alu->src[j].src.ssa->index;
            for (unsigned c = 0; c < 4; c++)
               o->swz[j][c] = alu->src[j].swizzle[c] & 3;
         }
         if (alu->op == nir_op_vec4) {
            /* the fourth source (one component) is kept in base / comp */
            o->base = (int)alu->src[3].src.ssa->index;
            o->comp = alu->src[3].swizzle[0] & 3;
         }
         break;
      }
      }
   }
   free(need);
   free(kept);
   return e;

fail:
   free(need);
   free(kept);
   fg_eval_destroy(e);
   return NULL;
}

void
fg_eval_destroy(struct fg_eval *e)
{
   if (!e)
      return;
   free(e->ops);
   free(e->v);
   free(e);
}

uint32_t
fg_eval_inputs(const struct fg_eval *e)
{
   return e ? e->inputs : 0;
}

static float
fg_eval_alu1(nir_op op, float a, float b, float c)
{
   switch (op) {
   case nir_op_mov: case nir_op_b2f32: return a;
   case nir_op_fneg: return -a;
   case nir_op_fabs: return fabsf(a);
   case nir_op_fsat: return a != a ? a : a < 0.0f ? 0.0f : a > 1.0f ? 1.0f : a;
   case nir_op_fadd: return a + b;
   case nir_op_fsub: return a - b;
   case nir_op_fmul: return a * b;
   case nir_op_ffma: return a * b + c;
   case nir_op_fmin: return (a != a || b != b) ? NAN : a < b ? a : b;
   case nir_op_fmax: return (a != a || b != b) ? NAN : a > b ? a : b;
   case nir_op_frcp: return 1.0f / a;
   case nir_op_frsq: return 1.0f / sqrtf(a);
   case nir_op_fsqrt: return sqrtf(a);
   case nir_op_fdiv: return a / b;
   case nir_op_ffloor: return floorf(a);
   case nir_op_fceil: return ceilf(a);
   case nir_op_ffract: return a - floorf(a);
   case nir_op_ftrunc: return truncf(a);
   case nir_op_fround_even: return rintf(a);
   case nir_op_fsign: return a > 0.0f ? 1.0f : a < 0.0f ? -1.0f : a;
   case nir_op_fexp2: return exp2f(a);
   case nir_op_flog2: return log2f(a);
   case nir_op_fpow: return powf(a, b);
   case nir_op_fsin: return sinf(a);
   case nir_op_fcos: return cosf(a);
   case nir_op_slt: return (a != a || b != b) ? NAN : a < b ? 1.0f : 0.0f;
   case nir_op_sge: return (a != a || b != b) ? NAN : a >= b ? 1.0f : 0.0f;
   case nir_op_seq: return (a != a || b != b) ? NAN : a == b ? 1.0f : 0.0f;
   case nir_op_sne: return (a != a || b != b) ? NAN : a != b ? 1.0f : 0.0f;
   case nir_op_fcsel: return a != a ? NAN : a != 0.0f ? b : c;
   case nir_op_flrp: return a * (1.0f - c) + b * c;
   default: return NAN;
   }
}

unsigned
fg_eval_output(struct fg_eval *e, const float (*in)[4], const float *uni, unsigned nvec, float pos[4])
{
   unsigned have = 0;

   for (unsigned i = 0; i < e->nops; i++) {
      const struct fg_eval_op *o = &e->ops[i];
      float *d = e->v[o->dst];

      switch (o->kind) {
      case EV_CONST:
         memcpy(d, o->k, sizeof(o->k));
         break;
      case EV_INPUT:
         for (unsigned c = 0; c < o->nc; c++)
            d[c] = in[o->base][o->comp + c];
         break;
      case EV_UNIFORM:
         if ((unsigned)o->base >= nvec)
            return 0;
         for (unsigned c = 0; c < o->nc; c++)
            d[c] = uni[4 * o->base + c];
         break;
      case EV_OUT:
         for (unsigned c = 0; c < o->nc && o->base + c < 4; c++) {
            if (o->mask & (1u << c)) {
               pos[o->base + c] = e->v[o->src[0]][c];
               have |= 1u << (o->base + c);
            }
         }
         break;
      default: {
         const float *s0 = e->v[o->src[0]], *s1 = e->v[o->src[1]], *s2 = e->v[o->src[2]];
         float r[4];

         switch (o->op) {
         case nir_op_vec2:
         case nir_op_vec3:
            for (unsigned c = 0; c < o->nc; c++)
               r[c] = e->v[o->src[c]][o->swz[c][0]];
            break;
         case nir_op_vec4:
            for (unsigned c = 0; c < 3; c++)
               r[c] = e->v[o->src[c]][o->swz[c][0]];
            r[3] = e->v[o->base][o->comp];
            break;
         case nir_op_fdot2:
         case nir_op_fdot3:
         case nir_op_fdot4:
         case nir_op_fdph: {
            unsigned n = o->op == nir_op_fdot2 ? 2 : o->op == nir_op_fdot4 ? 4 : 3;
            float sum = 0.0f;

            for (unsigned c = 0; c < n; c++)
               sum += s0[o->swz[0][c]] * s1[o->swz[1][c]];
            if (o->op == nir_op_fdph)
               sum += s1[o->swz[1][3]];
            for (unsigned c = 0; c < o->nc; c++)
               r[c] = sum;
            break;
         }
         default:
            for (unsigned c = 0; c < o->nc; c++)
               r[c] = fg_eval_alu1((nir_op)o->op, s0[o->swz[0][c]], o->nsrc > 1 ? s1[o->swz[1][c]] : 0.0f,
                                   o->nsrc > 2 ? s2[o->swz[2][c]] : 0.0f);
            break;
         }
         memcpy(d, r, o->nc * sizeof(float));
         break;
      }
      }
   }
   for (unsigned c = 0; c < 4; c++)
      if ((have & (1u << c)) && !isfinite(pos[c]))
         have &= ~(1u << c);
   return have;
}

bool
fg_eval_position(struct fg_eval *e, const float (*in)[4], const float *uni, unsigned nvec, float pos[4])
{
   return fg_eval_output(e, in, uni, nvec, pos) == 0xF;
}

void
fg_vs_eval_free(struct fg_vs_eval *ev)
{
   fg_eval_destroy(ev->pos);
   for (unsigned i = 0; i < FG_EVAL_VARYINGS; i++)
      fg_eval_destroy(ev->var[i]);
   memset(ev, 0, sizeof(*ev));
}

struct fg_eval *
fg_vs_eval_varying(const struct fg_vs_eval *ev, unsigned location)
{
   for (unsigned i = 0; i < FG_EVAL_VARYINGS; i++)
      if (ev->var[i] && ev->var_loc[i] == location)
         return ev->var[i];
   return NULL;
}

/* the position, and the first generic and texture-coordinate varyings */
static void
fg_vs_eval_create(nir_shader *s, struct fg_vs_eval *ev)
{
   uint64_t outs = s->info.outputs_written;
   unsigned n = 0;

   memset(ev, 0, sizeof(*ev));
   ev->pos = fg_eval_create(s, VARYING_SLOT_POS);
   if (!ev->pos)
      return;
   u_foreach_bit64(loc, outs) {
      if (n == FG_EVAL_VARYINGS)
         break;
      if (loc < VARYING_SLOT_VAR0 && !(loc >= VARYING_SLOT_TEX0 && loc <= VARYING_SLOT_TEX7))
         continue;
      ev->var[n] = fg_eval_create(s, loc);
      if (ev->var[n])
         ev->var_loc[n++] = (uint8_t)loc;
   }
}

/* ------------------------------------------------------------------ fragment programs that copy */

/* a texture coordinate that is a varying's component as it is, or its fract() (lowered: a - floor(a), a + -floor(a)) */
static bool
fg_copy_coord(nir_scalar v, uint8_t *loc, uint8_t *comp, bool *fract)
{
   nir_intrinsic_instr *in;
   nir_src *off;
   unsigned l;

   *fract = false;
   if (nir_scalar_is_alu(v)) {
      nir_op op = nir_scalar_alu_op(v);

      if (op == nir_op_ffract) {
         v = nir_scalar_chase_alu_src(v, 0);
         *fract = true;
      } else if (op == nir_op_fsub || op == nir_op_fadd) {
         for (unsigned i = 0; i < 2 && !*fract; i++) {
            nir_scalar a = nir_scalar_chase_alu_src(v, i), f = nir_scalar_chase_alu_src(v, 1 - i);

            if (op == nir_op_fsub && i)
               break;
            if (op == nir_op_fadd) {
               if (!nir_scalar_is_alu(f) || nir_scalar_alu_op(f) != nir_op_fneg)
                  continue;
               f = nir_scalar_chase_alu_src(f, 0);
            }
            if (nir_scalar_is_alu(f) && nir_scalar_alu_op(f) == nir_op_ffloor &&
                nir_scalar_equal(nir_scalar_chase_alu_src(f, 0), a)) {
               v = a;
               *fract = true;
            }
         }
         if (!*fract)
            return false;
      } else {
         return false;
      }
   }
   if (!nir_scalar_is_intrinsic(v))
      return false;
   in = nir_def_as_intrinsic(v.def);
   if (in->intrinsic != nir_intrinsic_load_input && in->intrinsic != nir_intrinsic_load_interpolated_input)
      return false;
   off = nir_get_io_offset_src(in);
   if (!off || !nir_src_is_const(*off) || nir_src_as_uint(*off))
      return false;
   l = nir_intrinsic_io_semantics(in).location;
   if (l < VARYING_SLOT_VAR0 && !(l >= VARYING_SLOT_TEX0 && l <= VARYING_SLOT_TEX7))
      return false;
   *loc = (uint8_t)l;
   *comp = (uint8_t)(nir_intrinsic_component(in) + v.comp);
   return true;
}

/* one store of the colour, all four components the texel of one 2D texture read (as read: no swizzle), whose
 * coordinates are varyings; nothing else with an effect (no discard, no depth) */
static void
fg_fs_copy_find(nir_shader *s, struct fg_fs_copy *c)
{
   nir_function_impl *impl = nir_shader_get_entrypoint(s);
   nir_intrinsic_instr *st = NULL;
   nir_tex_instr *tex = NULL;
   int ci;

   memset(c, 0, sizeof(*c));
   if (!impl || s->info.stage != MESA_SHADER_FRAGMENT || exec_list_length(&impl->body) != 1)
      return;
   nir_foreach_instr(instr, nir_start_block(impl)) {
      if (instr->type == nir_instr_type_intrinsic) {
         nir_intrinsic_instr *in = nir_instr_as_intrinsic(instr);

         switch (in->intrinsic) {
         case nir_intrinsic_load_input:
         case nir_intrinsic_load_interpolated_input:
         case nir_intrinsic_load_barycentric_pixel:
         case nir_intrinsic_load_barycentric_centroid:
         case nir_intrinsic_load_uniform:
            break;
         case nir_intrinsic_store_output:
            if (st)
               return;
            st = in;
            break;
         default:
            return;
         }
      } else if (instr->type == nir_instr_type_tex) {
         if (tex)
            return;
         tex = nir_instr_as_tex(instr);
      } else if (instr->type != nir_instr_type_alu && instr->type != nir_instr_type_load_const) {
         return;
      }
   }
   if (!st || !tex)
      return;
   {
      unsigned loc = nir_intrinsic_io_semantics(st).location;

      if ((loc != FRAG_RESULT_COLOR && loc != FRAG_RESULT_DATA0) || nir_intrinsic_component(st) ||
          (nir_intrinsic_write_mask(st) & 0xF) != 0xF || st->src[0].ssa->num_components != 4 ||
          !nir_src_is_const(st->src[1]) || nir_src_as_uint(st->src[1]))
         return;
   }
   for (unsigned k = 0; k < 4; k++) {
      nir_scalar v = nir_scalar_resolved(st->src[0].ssa, k);

      if (v.def != &tex->def || v.comp != k)
         return;
   }
   if (tex->op != nir_texop_tex || tex->sampler_dim != GLSL_SAMPLER_DIM_2D || tex->is_array || tex->is_shadow ||
       tex->texture_index != tex->sampler_index || tex->texture_index >= 8 || tex->def.num_components != 4)
      return;
   for (unsigned i = 0; i < tex->num_srcs; i++)
      if (tex->src[i].src_type != nir_tex_src_coord)
         return;
   ci = nir_tex_instr_src_index(tex, nir_tex_src_coord);
   if (ci < 0 || tex->coord_components != 2)
      return;
   for (unsigned k = 0; k < 2; k++)
      if (!fg_copy_coord(nir_scalar_resolved(tex->src[ci].src.ssa, k), &c->loc[k], &c->comp[k], &c->fract[k]))
         return;
   nir_foreach_shader_in_variable(var, s) {
      if ((var->data.location == c->loc[0] || var->data.location == c->loc[1]) &&
          var->data.interpolation == INTERP_MODE_FLAT)
         return;
   }
   c->unit = (uint8_t)tex->texture_index;
   c->ok = true;
}

void fg_nir_lower(nir_shader *s, struct fg_vs_eval *ev, struct fg_fs_copy *copy)
{
   bool progress;

   NIR_PASS(_, s, nir_lower_io, nir_var_shader_in | nir_var_shader_out, fg_type_size, (nir_lower_io_options)0);
   if (s->info.stage == MESA_SHADER_FRAGMENT) {
      nir_lower_tex_options t;

      memset(&t, 0, sizeof(t));
      t.lower_txp = ~0u; /* the TMU has no projection */
      NIR_PASS(_, s, nir_lower_tex, &t);
   }
   NIR_PASS(_, s, nir_lower_alu_to_scalar, fg_scalarize_filter, NULL);
   do {
      progress = fg_optimize_once(s);
   } while (progress);

   NIR_PASS(_, s, nir_lower_int_to_float);
   NIR_PASS(_, s, nir_lower_bool_to_float, true);
   do {
      progress = false;
      NIR_PASS(progress, s, nir_opt_algebraic);
      NIR_PASS(progress, s, nir_opt_copy_prop);
      NIR_PASS(progress, s, nir_opt_constant_folding);
      NIR_PASS(progress, s, nir_opt_dce);
   } while (progress);
   NIR_PASS(_, s, nir_opt_algebraic_late);
   NIR_PASS(_, s, nir_opt_copy_prop);
   NIR_PASS(_, s, nir_opt_dce);

   /* the last SSA form: a vertex program's position and varyings, for working out on the CPU what a draw can cover
    * and whether it is a copy; whether a fragment program is one */
   if (ev)
      fg_vs_eval_create(s, ev);
   if (copy)
      fg_fs_copy_find(s, copy);

   NIR_PASS(_, s, nir_convert_from_ssa, true, false);
   NIR_PASS(_, s, nir_remove_dead_variables, nir_var_function_temp, NULL);
   NIR_PASS(_, s, nir_move_vec_src_uses_to_dest, false);
   NIR_PASS(_, s, nir_lower_vec_to_regs, NULL, NULL);
   NIR_PASS(_, s, nir_opt_dce);
   NIR_PASS(_, s, nir_trivialize_registers);
   nir_sweep(s);
}

/* ------------------------------------------------------------------ the emitter */

typedef struct {
   Shc *h;
   nir_shader *s;
   const struct fg_compile_key *key;
   int fs, remat;
   Lines code;
   int vcount;
   unsigned char *sv;
   int svcap;
   double *pool; /* 4 doubles a vector */
   int npool, poolcap;
   int *pool_vec;            /* the register a pool vector was loaded into as a vector (0 = none) */
   int *pool_sca;            /* 4 a vector: the register a lane was loaded into as a scalar (0 = none) */
   int *pool_lanes;          /* the lanes scalar constants packed into a vector have taken (0: a vector constant) */
   int *defs;                /* nir_def index -> register */
   int ndefs;
   int labels;
   int pos, psize, color, depth; /* output registers, 0 = not written */
   int vary[FG_VARYINGS];
   uint64_t vmask;           /* the varying locations that have slots */
   uint32_t vary_used;
   uint32_t samplers;
   int max_uniform;          /* parameter vectors the uniforms take */
   int ifd, loopd;
   bool discard;
} fgc;

static SHC_NORETURN void fail(fgc *c, const char *what, const char *name)
{
   sh_compile_error(c->h, "%s%s%s", what, name ? ": " : "", name ? name : "");
}

static void emit(fgc *c, const char *fmt, ...)
{
   char buf[256];
   va_list ap;

   va_start(ap, fmt);
   vsnprintf(buf, sizeof(buf), fmt, ap);
   va_end(ap);
   lines_push(c->h, &c->code, line_parse(c->h, sh_strdup(c->h, buf)));
}

static void sv_room(fgc *c, int n)
{
   if (n >= c->svcap) {
      int cap = c->svcap ? c->svcap * 2 : 1024;
      unsigned char *nn;

      while (n >= cap)
         cap *= 2;
      nn = (unsigned char *)sh_zalloc(c->h, (size_t)cap);
      if (c->svcap)
         memcpy(nn, c->sv, (size_t)c->svcap);
      c->sv = nn;
      c->svcap = cap;
   }
}

static int vreg(fgc *c, bool scalar)
{
   int n = ++c->vcount;

   sv_room(c, n);
   c->sv[n] = scalar;
   return n;
}

/* a new pool vector (x, y, z, w); lanes: the lanes packed scalars take of it (0: a vector constant) */
static int pool_add(fgc *c, double x, double y, double z, double w, int lanes)
{
   if (c->npool == c->poolcap) {
      int cap = c->poolcap ? c->poolcap * 2 : 64;
      double *np = (double *)sh_alloc(c->h, (size_t)cap * 4 * sizeof(double));
      int *nv = (int *)sh_zalloc(c->h, (size_t)cap * sizeof(int));
      int *ns = (int *)sh_zalloc(c->h, (size_t)cap * 4 * sizeof(int));
      int *nl = (int *)sh_zalloc(c->h, (size_t)cap * sizeof(int));

      if (c->npool) {
         memcpy(np, c->pool, (size_t)c->npool * 4 * sizeof(double));
         memcpy(nv, c->pool_vec, (size_t)c->npool * sizeof(int));
         memcpy(ns, c->pool_sca, (size_t)c->npool * 4 * sizeof(int));
         memcpy(nl, c->pool_lanes, (size_t)c->npool * sizeof(int));
      }
      c->pool = np;
      c->pool_vec = nv;
      c->pool_sca = ns;
      c->pool_lanes = nl;
      c->poolcap = cap;
   }
   c->pool[4 * c->npool] = x;
   c->pool[4 * c->npool + 1] = y;
   c->pool[4 * c->npool + 2] = z;
   c->pool[4 * c->npool + 3] = w;
   c->pool_lanes[c->npool] = lanes;
   return c->npool++;
}

/* a vector constant: a parameter vector of its own (one holding the same four values shared; not one scalars are
 * packed into, whose free lanes may still change) */
static int kload(fgc *c, double x, double y, double z, double w)
{
   int i, slot = -1, r;

   for (i = 0; i < c->npool; i++) {
      const double *p = &c->pool[4 * i];

      if (!c->pool_lanes[i] && p[0] == x && p[1] == y && p[2] == z && p[3] == w) {
         slot = i;
         break;
      }
   }
   if (slot < 0)
      slot = pool_add(c, x, y, z, w, 0);
   if (!c->remat && c->pool_vec[slot])
      return c->pool_vec[slot];
   r = vreg(c, false);
   emit(c, "LDP r%d @%d", r, slot);
   if (!c->remat)
      c->pool_vec[slot] = r;
   return r;
}

/* a scalar constant: any lane of the pool that holds it, else the next free lane of a vector scalars are packed into
 * (four to a parameter vector: the fragment core has 64, uniforms included), read with that lane broadcast. 0 and 1
 * are vectors of their own, all four lanes, which the optimizer's compare folding recognises */
static int kf(fgc *c, double f)
{
   static const char *bcast[4] = { "xxxx", "yyyy", "zzzz", "wwww" };
   int i, k, slot = -1, lane = 0, r;

   if (f == 0.0 || f == 1.0) {
      for (i = 0; i < c->npool && slot < 0; i++) {
         const double *p = &c->pool[4 * i];

         if (!c->pool_lanes[i] && p[0] == f && p[1] == f && p[2] == f && p[3] == f)
            slot = i;
      }
      if (slot < 0)
         slot = pool_add(c, f, f, f, f, 0);
      if (!c->remat && c->pool_sca[4 * slot])
         return c->pool_sca[4 * slot];
      r = vreg(c, true);
      emit(c, "LDP r%d @%d", r, slot);
      if (!c->remat)
         c->pool_sca[4 * slot] = r;
      return r;
   }
   for (i = 0; i < c->npool && slot < 0; i++) {
      int n = c->pool_lanes[i] ? c->pool_lanes[i] : 4;

      for (k = 0; k < n; k++) {
         if (c->pool[4 * i + k] == f) {
            slot = i;
            lane = k;
            break;
         }
      }
   }
   for (i = 0; i < c->npool && slot < 0; i++) {
      if (c->pool_lanes[i] && c->pool_lanes[i] < 4) {
         slot = i;
         lane = c->pool_lanes[i]++;
         c->pool[4 * i + lane] = f;
      }
   }
   if (slot < 0)
      slot = pool_add(c, f, 0.0, 0.0, 0.0, 1);
   if (!c->remat && c->pool_sca[4 * slot + lane])
      return c->pool_sca[4 * slot + lane];
   r = vreg(c, true);
   emit(c, "LDP r%d @%d.%s", r, slot, bcast[lane]);
   if (!c->remat)
      c->pool_sca[4 * slot + lane] = r;
   return r;
}

static int def_reg(fgc *c, const nir_def *d)
{
   int r = d->index < (unsigned)c->ndefs ? c->defs[d->index] : 0;

   if (!r)
      fail(c, "internal: a value used before it is defined", NULL);
   return r;
}

static void set_def(fgc *c, const nir_def *d, int r)
{
   if (d->index >= (unsigned)c->ndefs)
      fail(c, "internal: value index out of range", NULL);
   c->defs[d->index] = r;
}

static int new_def(fgc *c, const nir_def *d)
{
   int r = vreg(c, d->num_components == 1);

   set_def(c, d, r);
   return r;
}

/* a source operand: mod "", "-", "!" or "-!"; n the components the instruction uses, the swizzle letters */
static void src_text(fgc *c, char *out, size_t size, const nir_def *d, const uint8_t *swizzle, unsigned n, const char *mod)
{
   int r = def_reg(c, d);
   char sw[5];
   unsigned i;
   bool ident = n == 4;

   if (c->sv[r] || d->num_components == 1) {
      snprintf(out, size, "%sr%d", mod, r);
      return;
   }
   for (i = 0; i < n && i < 4; i++) {
      sw[i] = "xyzw"[swizzle ? swizzle[i] & 3 : i];
      if ((swizzle ? swizzle[i] : i) != i)
         ident = false;
   }
   sw[i] = 0;
   if (ident)
      snprintf(out, size, "%sr%d", mod, r);
   else
      snprintf(out, size, "%sr%d.%s", mod, r, sw);
}

static void alu_src(fgc *c, char *out, size_t size, const nir_alu_instr *alu, unsigned i, unsigned n, const char *mod)
{
   src_text(c, out, size, alu->src[i].src.ssa, alu->src[i].swizzle, n, mod);
}

static const char *newlabel(fgc *c, char *buf, size_t size, const char *p)
{
   snprintf(buf, size, "%s%d", p, ++c->labels);
   return buf;
}

/* ------------------------------------------------------------------ ALU */

static void emit_alu(fgc *c, nir_alu_instr *alu)
{
   const nir_op_info *info = &nir_op_infos[alu->op];
   unsigned n = alu->def.num_components, i;
   char a[48], b[48], d3[48];
   const char *mn = NULL, *cc = NULL;
   int r, t;

   if (alu->def.bit_size != 32)
      fail(c, "not a 32-bit operation", info->name);

   switch (alu->op) {
   case nir_op_mov: mn = "MOV"; break;
   case nir_op_frcp: mn = "RCP"; break;
   case nir_op_frsq: mn = "RSQRT"; break;
   case nir_op_fexp2: mn = "EXP2"; break;
   case nir_op_flog2: mn = "LOG2"; break;
   case nir_op_fsin: mn = "SIN"; break;
   case nir_op_fcos: mn = "COS"; break;
   case nir_op_ffloor: mn = "FLR"; break;
   case nir_op_fceil: mn = "CEIL"; break;
   case nir_op_ftrunc: mn = "TRUNC"; break;
   case nir_op_fround_even: mn = "RNDE"; break;
   default: break;
   }
   if (mn) {
      r = new_def(c, &alu->def);
      alu_src(c, a, sizeof(a), alu, 0, n, "");
      emit(c, "%s r%d %s", mn, r, a);
      return;
   }

   switch (alu->op) {
   case nir_op_fneg:
   case nir_op_fabs:
      r = new_def(c, &alu->def);
      alu_src(c, a, sizeof(a), alu, 0, n, alu->op == nir_op_fneg ? "-" : "!");
      emit(c, "MOV r%d %s", r, a);
      return;
   case nir_op_fadd: mn = "ADD"; break;
   case nir_op_fmul: mn = "MUL"; break;
   case nir_op_fmin: mn = "MIN"; break;
   case nir_op_fmax: mn = "MAX"; break;
   case nir_op_slt: cc = "SLT"; break;
   case nir_op_sge: cc = "SGE"; break;
   case nir_op_seq: cc = "SEQ"; break;
   case nir_op_sne: cc = "SNE"; break;
   default: break;
   }
   if (mn || cc) {
      r = new_def(c, &alu->def);
      alu_src(c, a, sizeof(a), alu, 0, n, "");
      alu_src(c, b, sizeof(b), alu, 1, n, "");
      if (mn)
         emit(c, "%s r%d %s %s", mn, r, a, b);
      else
         emit(c, "SET %s r%d %s %s", cc, r, a, b);
      return;
   }

   switch (alu->op) {
   case nir_op_ffma: /* MUL + ADD: the back end fuses them into a MAD */
      t = vreg(c, n == 1);
      alu_src(c, a, sizeof(a), alu, 0, n, "");
      alu_src(c, b, sizeof(b), alu, 1, n, "");
      emit(c, "MUL r%d %s %s", t, a, b);
      r = new_def(c, &alu->def);
      alu_src(c, a, sizeof(a), alu, 2, n, "");
      emit(c, "ADD r%d r%d %s", r, t, a);
      return;
   case nir_op_fcsel:
      r = new_def(c, &alu->def);
      alu_src(c, a, sizeof(a), alu, 0, n, "");
      alu_src(c, b, sizeof(b), alu, 1, n, "");
      alu_src(c, d3, sizeof(d3), alu, 2, n, "");
      emit(c, "SEL r%d %s %s %s", r, a, b, d3);
      return;
   case nir_op_fdot2:
      t = vreg(c, false);
      alu_src(c, a, sizeof(a), alu, 0, 2, "");
      alu_src(c, b, sizeof(b), alu, 1, 2, "");
      emit(c, "MUL r%d %s %s", t, a, b);
      r = new_def(c, &alu->def);
      emit(c, "ADD r%d r%d.x r%d.y", r, t, t);
      return;
   case nir_op_fdot3:
   case nir_op_fdot4:
      r = new_def(c, &alu->def);
      i = alu->op == nir_op_fdot3 ? 3 : 4;
      alu_src(c, a, sizeof(a), alu, 0, i, "");
      alu_src(c, b, sizeof(b), alu, 1, i, "");
      emit(c, "%s r%d %s %s", i == 3 ? "DP3" : "DP4", r, a, b);
      if (n != 1)
         fail(c, "a dot product of more than one component", info->name);
      return;
   case nir_op_fsqrt:
      alu_src(c, a, sizeof(a), alu, 0, n, "");
      if (c->fs && (c->key->xops & 1)) {
         r = new_def(c, &alu->def);
         emit(c, "SQRT r%d %s", r, a);
         return;
      }
      /* x * rsqrt(x), and x itself where it is 0 (glslc's sqrt) */
      t = vreg(c, n == 1);
      emit(c, "RSQRT r%d %s", t, a);
      {
         int p = vreg(c, n == 1);

         emit(c, "MUL r%d %s r%d", p, a, t);
         r = new_def(c, &alu->def);
         emit(c, "SEL r%d %s r%d %s", r, a, p, a);
      }
      return;
   case nir_op_fsat:
      alu_src(c, a, sizeof(a), alu, 0, n, "");
      if (c->fs && (c->key->xops & 2)) {
         int z = kf(c, 0.0), o = kf(c, 1.0);

         r = new_def(c, &alu->def);
         emit(c, "CLAMP r%d %s r%d r%d", r, a, z, o);
         return;
      }
      t = vreg(c, n == 1);
      emit(c, "MAX r%d %s r%d", t, a, kf(c, 0.0));
      r = new_def(c, &alu->def);
      emit(c, "MIN r%d r%d r%d", r, t, kf(c, 1.0));
      return;
   case nir_op_vec2:
   case nir_op_vec3:
   case nir_op_vec4:
      r = new_def(c, &alu->def);
      for (i = 0; i < n; i++) {
         uint8_t sw = alu->src[i].swizzle[0];

         src_text(c, a, sizeof(a), alu->src[i].src.ssa, &sw, 1, "");
         emit(c, "MOV.%c r%d %s", "xyzw"[i], r, a);
      }
      return;
   default:
      fail(c, "an operation the cores do not have", info->name);
   }
}

/* ------------------------------------------------------------------ intrinsics */

/* the swizzle letters of components first .. first + n - 1 */
static void comp_letters(char *out, unsigned first, unsigned n)
{
   unsigned i;

   for (i = 0; i < n && first + i < 4; i++)
      out[i] = "xyzw"[first + i];
   out[i] = 0;
}

static int varying_slot(fgc *c, unsigned loc)
{
   int slot;

   if (loc >= 64 || !(c->vmask & BITFIELD64_BIT(loc)))
      return -1;
   slot = util_bitcount64(c->vmask & BITFIELD64_MASK(loc));
   if (slot >= FG_VARYINGS)
      fail(c, "more varyings than the cores pass on", NULL);
   return slot;
}

/* LDV of input index idx (a slot) into a new value of n components from component first */
static void emit_ldv(fgc *c, const nir_def *d, unsigned idx, unsigned first)
{
   int r = new_def(c, d);
   char sw[5];

   if (first == 0 && d->num_components == 4)
      emit(c, "LDV r%d %u", r, idx);
   else {
      comp_letters(sw, first, d->num_components);
      emit(c, "LDV r%d %u.%s", r, idx, sw);
   }
}

static void emit_zero_def(fgc *c, const nir_def *d)
{
   set_def(c, d, d->num_components == 1 ? kf(c, 0.0) : kload(c, 0, 0, 0, 0));
}

static void emit_fs_input(fgc *c, nir_intrinsic_instr *in, unsigned loc)
{
   unsigned first = nir_intrinsic_component(in);
   int slot;

   switch (loc) {
   case VARYING_SLOT_POS: emit_ldv(c, &in->def, 8, first); return;
   case VARYING_SLOT_FACE: emit_ldv(c, &in->def, 9, 0); return;
   case VARYING_SLOT_PNTC: emit_ldv(c, &in->def, 10, first); return;
   default: break;
   }
   slot = varying_slot(c, loc);
   if (slot < 0) { /* the vertex stage does not write it */
      emit_zero_def(c, &in->def);
      return;
   }
   c->vary_used |= 1u << slot;
   emit_ldv(c, &in->def, (unsigned)slot, first);
}

/* a store into an output register: value components go to lanes first.. under the write mask */
static void store_into(fgc *c, int *reg, bool scalar, nir_intrinsic_instr *st, unsigned first)
{
   const nir_def *v = st->src[0].ssa;
   unsigned wm = nir_intrinsic_write_mask(st) << first, l;
   char m[5], sw[5], a[48];
   int nm = 0, r;

   if (!*reg)
      *reg = vreg(c, scalar);
   r = def_reg(c, v);
   if (scalar) {
      if (c->sv[r] || v->num_components == 1)
         emit(c, "MOV r%d r%d", *reg, r);
      else
         emit(c, "MOV r%d r%d.%c", *reg, r, "xyzw"[0]);
      return;
   }
   for (l = 0; l < 4; l++) {
      sw[l] = 'x';
      if (wm & (1u << l)) {
         m[nm++] = "xyzw"[l];
         sw[l] = "xyzw"[l - first];
      }
   }
   m[nm] = 0;
   sw[4] = 0;
   if (!nm)
      return;
   if (c->sv[r] || v->num_components == 1)
      snprintf(a, sizeof(a), "r%d", r);
   else
      snprintf(a, sizeof(a), "r%d.%s", r, sw);
   if (nm == 4)
      emit(c, "MOV r%d %s", *reg, a);
   else
      emit(c, "MOV.%s r%d %s", m, *reg, a);
}

static void emit_store_output(fgc *c, nir_intrinsic_instr *st)
{
   unsigned loc = nir_intrinsic_io_semantics(st).location, first = nir_intrinsic_component(st);
   int slot;

   if (!nir_src_is_const(st->src[1]) || nir_src_as_uint(st->src[1]) != 0)
      fail(c, "an output at a computed index", NULL);
   if (c->fs) {
      if (loc == FRAG_RESULT_DEPTH)
         store_into(c, &c->depth, true, st, 0);
      else if (loc == FRAG_RESULT_COLOR || loc == FRAG_RESULT_DATA0)
         store_into(c, &c->color, false, st, first);
      else
         fail(c, "a fragment output the cores do not have", gl_frag_result_name(loc));
      return;
   }
   if (loc == VARYING_SLOT_POS) {
      store_into(c, &c->pos, false, st, first);
      return;
   }
   if (loc == VARYING_SLOT_PSIZ) {
      store_into(c, &c->psize, true, st, 0);
      return;
   }
   slot = varying_slot(c, loc);
   if (slot < 0)
      return; /* not passed on: clip distances, edge flags */
   c->vary_used |= 1u << slot;
   store_into(c, &c->vary[slot], false, st, first);
}

static void emit_load_uniform(fgc *c, nir_intrinsic_instr *in)
{
   int base = nir_intrinsic_base(in), r, ix, f;

   if (nir_src_is_const(in->src[0])) {
      /* integers are floats by now (nir_lower_int_to_float): the offset constant is a float */
      int slot = base + (int)nir_src_as_float(in->src[0]);

      if (slot + 1 > c->max_uniform)
         c->max_uniform = slot + 1;
      r = new_def(c, &in->def);
      emit(c, "LDP r%d %d", r, slot);
      return;
   }
   if (c->fs)
      fail(c, "a uniform at a computed index in a fragment program", NULL);
   if (base + (int)nir_intrinsic_range(in) > c->max_uniform)
      c->max_uniform = base + (int)nir_intrinsic_range(in);
   /* LDPX adds an integer register to the slot */
   f = def_reg(c, in->src[0].ssa);
   ix = vreg(c, true);
   if (c->sv[f] || in->src[0].ssa->num_components == 1)
      emit(c, "FTOI r%d r%d", ix, f);
   else
      emit(c, "FTOI r%d r%d.x", ix, f);
   r = new_def(c, &in->def);
   emit(c, "LDPX r%d %d r%d", r, base, ix);
}

static void emit_if_discard(fgc *c, const nir_def *cond)
{
   char le[16], ln[16];

   if (c->ifd + 1 > IF_DEPTH)
      fail(c, "IF nesting deeper than the cores have", NULL);
   emit(c, "SETP SNE r%d r%d", def_reg(c, cond), kf(c, 0.0));
   newlabel(c, le, sizeof(le), "Lelse");
   newlabel(c, ln, sizeof(ln), "Lend");
   emit(c, "IF %s", le);
   emit(c, "DISCARD");
   emit(c, "%s:", le);
   emit(c, "ELSE %s", ln);
   emit(c, "%s:", ln);
   emit(c, "ENDIF");
}

static void emit_intrinsic(fgc *c, nir_intrinsic_instr *in)
{
   switch (in->intrinsic) {
   case nir_intrinsic_decl_reg: {
      int r;

      if (nir_intrinsic_num_array_elems(in))
         fail(c, "an array in registers", NULL);
      r = vreg(c, nir_intrinsic_num_components(in) == 1);
      set_def(c, &in->def, r);
      return;
   }
   case nir_intrinsic_load_reg: {
      int src = def_reg(c, in->src[0].ssa), r;

      if (nir_intrinsic_base(in))
         fail(c, "an array in registers", NULL);
      r = new_def(c, &in->def);
      emit(c, "MOV r%d r%d", r, src);
      return;
   }
   case nir_intrinsic_store_reg: {
      nir_intrinsic_instr *decl = nir_reg_get_decl(in->src[1].ssa);
      int reg = def_reg(c, in->src[1].ssa);
      unsigned nc = nir_intrinsic_num_components(decl), wm = nir_intrinsic_write_mask(in), l;
      const nir_def *v = in->src[0].ssa;
      int r = def_reg(c, v);
      char m[5];
      int nm = 0;

      if (nir_intrinsic_base(in))
         fail(c, "an array in registers", NULL);
      if (nc == 1 || c->sv[reg]) {
         emit(c, "MOV r%d r%d", reg, r);
         return;
      }
      for (l = 0; l < 4; l++)
         if (wm & (1u << l))
            m[nm++] = "xyzw"[l];
      m[nm] = 0;
      if (wm == BITFIELD_MASK(nc))
         emit(c, "MOV r%d r%d", reg, r);
      else
         emit(c, "MOV.%s r%d r%d", m, reg, r);
      return;
   }
   case nir_intrinsic_load_input:
      if (!nir_src_is_const(in->src[0]) || nir_src_as_uint(in->src[0]) != 0)
         fail(c, "an input at a computed index", NULL);
      if (c->fs)
         emit_fs_input(c, in, nir_intrinsic_io_semantics(in).location);
      else
         emit_ldv(c, &in->def, nir_intrinsic_base(in), nir_intrinsic_component(in));
      return;
   case nir_intrinsic_load_interpolated_input:
      if (!nir_src_is_const(in->src[1]) || nir_src_as_uint(in->src[1]) != 0)
         fail(c, "an input at a computed index", NULL);
      emit_fs_input(c, in, nir_intrinsic_io_semantics(in).location);
      return;
   case nir_intrinsic_load_barycentric_pixel:
   case nir_intrinsic_load_barycentric_centroid:
   case nir_intrinsic_load_barycentric_sample:
      return; /* the cores interpolate the inputs themselves: LDV takes no barycentrics */
   case nir_intrinsic_load_frag_coord:
      emit_ldv(c, &in->def, 8, 0);
      return;
   case nir_intrinsic_load_front_face:
      emit_ldv(c, &in->def, 9, 0);
      return;
   case nir_intrinsic_load_point_coord:
      emit_ldv(c, &in->def, 10, 0);
      return;
   case nir_intrinsic_load_uniform:
      emit_load_uniform(c, in);
      return;
   case nir_intrinsic_ddx:
   case nir_intrinsic_ddx_fine:
   case nir_intrinsic_ddx_coarse:
   case nir_intrinsic_ddy:
   case nir_intrinsic_ddy_fine:
   case nir_intrinsic_ddy_coarse: {
      bool x = in->intrinsic == nir_intrinsic_ddx || in->intrinsic == nir_intrinsic_ddx_fine ||
               in->intrinsic == nir_intrinsic_ddx_coarse;
      int r;
      char a[48];

      if (!c->fs)
         fail(c, "a derivative in a vertex program", NULL);
      src_text(c, a, sizeof(a), in->src[0].ssa, NULL, in->def.num_components, "");
      r = new_def(c, &in->def);
      emit(c, "%s r%d %s", x ? "DDX" : "DDY", r, a);
      return;
   }
   case nir_intrinsic_store_output:
      emit_store_output(c, in);
      return;
   case nir_intrinsic_terminate:
   case nir_intrinsic_demote:
      if (!c->fs)
         fail(c, "discard in a vertex program", NULL);
      c->discard = true;
      emit(c, "DISCARD");
      return;
   case nir_intrinsic_terminate_if:
   case nir_intrinsic_demote_if:
      if (!c->fs)
         fail(c, "discard in a vertex program", NULL);
      c->discard = true;
      emit_if_discard(c, in->src[0].ssa);
      return;
   default:
      fail(c, "an intrinsic the driver does not handle", nir_intrinsic_infos[in->intrinsic].name);
   }
}

/* ------------------------------------------------------------------ textures */

static void emit_tex(fgc *c, nir_tex_instr *tex)
{
   int coord = -1, bias = -1, lod = -1, cmp = -1, i, r, t, u = (int)tex->texture_index;
   bool cube = tex->sampler_dim == GLSL_SAMPLER_DIM_CUBE || tex->sampler_dim == GLSL_SAMPLER_DIM_3D;

   if (!c->fs)
      fail(c, "a texture read in a vertex program (the vertex core has no texture unit)", NULL);
   if (u < 0 || u >= 16)
      fail(c, "a sampler unit past 15", NULL);
   if (tex->sampler_dim != GLSL_SAMPLER_DIM_2D && !cube && tex->sampler_dim != GLSL_SAMPLER_DIM_EXTERNAL)
      fail(c, "a texture type the cores do not sample (only 2D, cube and 3D)", NULL);
   for (i = 0; i < (int)tex->num_srcs; i++) {
      int reg = def_reg(c, tex->src[i].src.ssa);

      switch (tex->src[i].src_type) {
      case nir_tex_src_coord: coord = reg; break;
      case nir_tex_src_comparator: cmp = reg; break;
      case nir_tex_src_bias: bias = reg; break;
      case nir_tex_src_lod: lod = reg; break;
      case nir_tex_src_texture_deref:
      case nir_tex_src_sampler_deref:
      case nir_tex_src_texture_offset:
      case nir_tex_src_sampler_offset:
         fail(c, "a sampler chosen at run time", NULL);
      default:
         fail(c, "a texture operand the cores do not have", NULL);
      }
   }
   if (coord < 0)
      fail(c, "a texture read without coordinates", NULL);
   c->samplers |= 1u << u;
   r = vreg(c, false);
   switch (tex->op) {
   case nir_texop_tex:
      emit(c, "TEX r%d r%d #%d %s", r, coord, u, cube ? ".xyz" : ".xy");
      break;
   case nir_texop_txb:
   case nir_texop_txl: {
      int w = tex->op == nir_texop_txb ? bias : lod;

      if (w < 0)
         fail(c, "a texture read without its bias or level", NULL);
      t = vreg(c, false);
      emit(c, "MOV r%d r%d", t, coord);
      emit(c, "MOV.w r%d r%d", t, w);
      emit(c, "TEX r%d r%d #%d %s%s", r, t, u, tex->op == nir_texop_txb ? "BIAS" : "LOD", cube ? "" : " .xyw");
      break;
   }
   default:
      fail(c, "a texture operation the cores do not have", NULL);
   }
   if (c->key->depth_samplers & (1u << u)) {
      /* a depth texture: the TMU reads the D24S8 word as RGBA bytes, R the stencil, then the depth high byte first */
      int g = vreg(c, true), b = vreg(c, true), a = vreg(c, true), z = vreg(c, true), q = vreg(c, true), o = vreg(c, true);
      int d = vreg(c, false);

      emit(c, "MOV r%d r%d.y", g, r);
      emit(c, "MOV r%d r%d.z", b, r);
      emit(c, "MOV r%d r%d.w", a, r);
      if (c->key->depth16_samplers & (1u << u)) {
         /* a 16-bit depth: (B 256 + A) / 65535, the bytes read as / 255: 65280 / 65535 and 255 / 65535 */
         emit(c, "MUL r%d r%d r%d", z, b, kf(c, 0.99610895));
         emit(c, "MUL r%d r%d r%d", q, a, kf(c, 0.0038910506));
         emit(c, "ADD r%d r%d r%d", z, z, q);
      } else {
         emit(c, "MUL r%d r%d r%d", z, g, kf(c, 0.99609375));
         emit(c, "MUL r%d r%d r%d", q, b, kf(c, 0.003891));
         emit(c, "ADD r%d r%d r%d", z, z, q);
         emit(c, "MUL r%d r%d r%d", q, a, kf(c, 0.000016));
         emit(c, "ADD r%d r%d r%d", z, z, q);
      }
      emit(c, "MIN r%d r%d r%d", o, z, kf(c, 1.0));
      emit(c, "MOV r%d r%d", d, o);
      emit(c, "MOV.w r%d r%d", d, kf(c, 1.0));
      r = d;
   }
   if (tex->is_shadow) {
      /* a depth compare (the TMU has none): the reference, clamped to [0, 1] as for a fixed-point depth, against
       * the depth read (.x), by the sampler's function; 1 or 0 in every channel (the view's swizzle follows) */
      unsigned func = u < 8 ? c->key->shadow_func >> (3 * u) & 7 : 0;
      int ref = vreg(c, true), z = vreg(c, true), res = vreg(c, true), d = vreg(c, false);

      if (cmp < 0)
         fail(c, "a shadow read without its reference value", NULL);
      emit(c, "MAX r%d r%d r%d", ref, cmp, kf(c, 0.0));
      emit(c, "MIN r%d r%d r%d", ref, ref, kf(c, 1.0));
      emit(c, "MOV r%d r%d.x", z, r);
      switch (func) {
      case 1: emit(c, "SET SLT r%d r%d r%d", res, ref, z); break;  /* less: ref < depth */
      case 2: emit(c, "SET SEQ r%d r%d r%d", res, ref, z); break;  /* equal */
      case 3: emit(c, "SET SGE r%d r%d r%d", res, z, ref); break;  /* less or equal: depth >= ref */
      case 4: emit(c, "SET SLT r%d r%d r%d", res, z, ref); break;  /* greater: depth < ref */
      case 5: emit(c, "SET SNE r%d r%d r%d", res, ref, z); break;  /* not equal */
      case 6: emit(c, "SET SGE r%d r%d r%d", res, ref, z); break;  /* greater or equal */
      case 7: emit(c, "MOV r%d r%d", res, kf(c, 1.0)); break;      /* always */
      default: emit(c, "MOV r%d r%d", res, kf(c, 0.0)); break;     /* never */
      }
      emit(c, "MOV r%d r%d", d, res);
      r = d;
   }
   if (c->key->swizzled & (1u << u)) {
      /* the sampler view's swizzle: each component from a texel component, zero or one */
      int d = vreg(c, false);

      emit(c, "MOV r%d r%d", d, r);
      for (i = 0; i < 4; i++) {
         unsigned s = c->key->swizzle[u] >> (3 * i) & 7;

         if (s == (unsigned)i)
            continue;
         if (s < 4)
            emit(c, "MOV.%c r%d r%d.%c", "xyzw"[i], d, r, "xyzw"[s]);
         else
            emit(c, "MOV.%c r%d r%d", "xyzw"[i], d, kf(c, s == 4 ? 0.0 : 1.0));
      }
      r = d;
   }
   set_def(c, &tex->def, r);
}

/* ------------------------------------------------------------------ control flow */

static void emit_cf_list(fgc *c, struct exec_list *list);

static void emit_jump(fgc *c, nir_jump_instr *j)
{
   switch (j->type) {
   case nir_jump_break:
   case nir_jump_continue:
      if (!c->loopd)
         fail(c, "break or continue outside a loop", NULL);
      emit(c, "SETP SNE r%d r%d", kf(c, 1.0), kf(c, 0.0));
      emit(c, j->type == nir_jump_break ? "BREAK" : "CONT");
      return;
   default:
      fail(c, "a jump the cores do not have (return, halt)", NULL);
   }
}

static void emit_block(fgc *c, nir_block *block)
{
   nir_foreach_instr(instr, block) {
      switch (instr->type) {
      case nir_instr_type_alu:
         emit_alu(c, nir_instr_as_alu(instr));
         break;
      case nir_instr_type_intrinsic:
         emit_intrinsic(c, nir_instr_as_intrinsic(instr));
         break;
      case nir_instr_type_load_const: {
         nir_load_const_instr *lc = nir_instr_as_load_const(instr);
         double v[4] = { 0, 0, 0, 0 };
         unsigned i, n = lc->def.num_components;

         if (lc->def.bit_size != 32)
            fail(c, "a constant that is not 32-bit", NULL);
         for (i = 0; i < n && i < 4; i++)
            v[i] = lc->value[i].f32;
         set_def(c, &lc->def, n == 1 ? kf(c, v[0]) : kload(c, v[0], v[1], v[2], v[3]));
         break;
      }
      case nir_instr_type_undef:
         emit_zero_def(c, &nir_instr_as_undef(instr)->def);
         break;
      case nir_instr_type_tex:
         emit_tex(c, nir_instr_as_tex(instr));
         break;
      case nir_instr_type_jump:
         emit_jump(c, nir_instr_as_jump(instr));
         break;
      case nir_instr_type_deref:
         break; /* samplers are by index; nothing else is left as a deref */
      default:
         fail(c, "an instruction the driver does not handle", NULL);
      }
   }
}

static void emit_if(fgc *c, nir_if *nif)
{
   char le[16], ln[16];

   if (++c->ifd > IF_DEPTH)
      fail(c, "IF nesting deeper than the cores have", NULL);
   emit(c, "SETP SNE r%d r%d", def_reg(c, nif->condition.ssa), kf(c, 0.0));
   newlabel(c, le, sizeof(le), "Lelse");
   newlabel(c, ln, sizeof(ln), "Lend");
   emit(c, "IF %s", le);
   emit_cf_list(c, &nif->then_list);
   emit(c, "%s:", le);
   emit(c, "ELSE %s", ln);
   emit_cf_list(c, &nif->else_list);
   emit(c, "%s:", ln);
   emit(c, "ENDIF");
   c->ifd--;
}

static void emit_loop(fgc *c, nir_loop *loop)
{
   if (nir_loop_has_continue_construct(loop))
      fail(c, "internal: a loop with a continue construct", NULL);
   if (++c->loopd > LOOP_DEPTH)
      fail(c, "loops nested deeper than the cores have", NULL);
   emit(c, "LOOP 4096");
   emit_cf_list(c, &loop->body);
   emit(c, "ENDL");
   c->loopd--;
}

static void emit_cf_list(fgc *c, struct exec_list *list)
{
   foreach_list_typed(nir_cf_node, node, node, list) {
      switch (node->type) {
      case nir_cf_node_block:
         emit_block(c, nir_cf_node_as_block(node));
         break;
      case nir_cf_node_if:
         emit_if(c, nir_cf_node_as_if(node));
         break;
      case nir_cf_node_loop:
         emit_loop(c, nir_cf_node_as_loop(node));
         break;
      default:
         fail(c, "internal: a control-flow node of an unknown kind", NULL);
      }
   }
}

static void emit_outputs(fgc *c)
{
   int i;

   if (c->fs) {
      emit(c, "STO 0 r%d", c->color ? c->color : kload(c, 0, 0, 0, 0));
      if (c->depth)
         emit(c, "STOZ r%d", c->depth);
      return;
   }
   emit(c, "STO 0 r%d", c->pos ? c->pos : kload(c, 0, 0, 0, 1));
   if (c->psize)
      emit(c, "STO 1 r%d", c->psize);
   for (i = 0; i < FG_VARYINGS; i++)
      if (c->vary[i])
         emit(c, "STV %d r%d", i, c->vary[i]);
}

/* ------------------------------------------------------------------ the driver */

int fg_compile(nir_shader *s, const struct fg_compile_key *key, struct fg_program *out, char *err, size_t errsize)
{
   nir_function_impl *impl = nir_shader_get_entrypoint(s);
   Shc *h;
   fgc *c;
   char *sasm;
   uint64_t *words;
   int nwords, i;
   volatile int attempt = 0;
   int fs = s->info.stage == MESA_SHADER_FRAGMENT;

   memset(out, 0, sizeof(*out));
   if (s->info.stage != MESA_SHADER_VERTEX && !fs) {
      snprintf(err, errsize, "only vertex and fragment programs");
      return -1;
   }
   nir_index_ssa_defs(impl);
   h = (Shc *)calloc(1, sizeof(Shc));
   if (!h) {
      snprintf(err, errsize, "out of memory");
      return -1;
   }
   h->xops = fs ? (key->xops & 3u) : 0;
   h->noops = key->noops;
   sh_init(h);
retry:
   if (setjmp(h->jb)) {
      /* registers run out: again without the loop-invariant hoist, then with the constants loaded where used */
      if (attempt < 2 && h->compile_error && strstr(h->err, "register spill")) {
         attempt++;
         sh_done(h);
         sh_init(h);
         goto retry;
      }
      snprintf(err, errsize, "%s", h->err[0] ? h->err : "compile failed");
      sh_done(h);
      free(h);
      fg_program_free(out);
      return -1;
   }
   h->nohoist = attempt >= 1;
   c = (fgc *)sh_zalloc(h, sizeof(fgc));
   c->h = h;
   c->s = s;
   c->key = key;
   c->fs = fs;
   c->remat = attempt == 2;
   c->ndefs = (int)impl->ssa_alloc + 1;
   c->defs = (int *)sh_zalloc(h, (size_t)c->ndefs * sizeof(int));
   c->vmask = fs ? key->vs_outputs : fg_varying_mask(s->info.outputs_written);

   emit_cf_list(c, &impl->body);
   emit_outputs(c);
   emit(c, "END");
   sv_room(c, c->vcount + 1);

   sasm =sh_optimize(h, &c->code, c->vcount, c->sv, c->remat, c->max_uniform, fs, c->pool, &c->npool);
   if (c->max_uniform + c->npool > (fs ? FG_FS_PARAMS : FG_VS_PARAMS)) {
      char n[64];

      snprintf(n, sizeof(n), "%d uniform + %d constant vectors, %d room", c->max_uniform, c->npool,
               fs ? FG_FS_PARAMS : FG_VS_PARAMS);
      fail(c, "more uniforms and constants than the core's parameter vectors", n);
   }
   words = sh_assemble(h, sasm, !fs, IF_DEPTH, LOOP_DEPTH, &nwords);

   out->words = (uint64_t *)malloc((size_t)(nwords ? nwords : 1) * sizeof(uint64_t));
   out->consts = (uint32_t *)malloc((size_t)(c->npool ? c->npool : 1) * 4 * sizeof(uint32_t));
   out->sasm = strdup(sasm);
   if (!out->words || !out->consts || !out->sasm) {
      snprintf(err, errsize, "out of memory");
      sh_done(h);
      free(h);
      fg_program_free(out);
      return -1;
   }
   memcpy(out->words, words, (size_t)nwords * sizeof(uint64_t));
   out->nwords = (unsigned)nwords;
   for (i = 0; i < 4 * c->npool; i++)
      out->consts[i] = fg_fp24(c->pool[i]);
   out->nconsts = (unsigned)c->npool;
   out->const_base = (unsigned)c->max_uniform;
   out->varyings = c->vary_used;
   out->samplers = c->samplers;
   out->discard = c->discard;
   out->writes_depth = c->depth != 0;
   sh_done(h);
   free(h);
   return 0;
}

void fg_program_free(struct fg_program *p)
{
   free(p->words);
   free(p->consts);
   free(p->sasm);
   memset(p, 0, sizeof(*p));
}
