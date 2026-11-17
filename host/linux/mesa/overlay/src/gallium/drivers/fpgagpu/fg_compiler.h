/* SPDX-License-Identifier: MIT */
/*
 * fg_compiler: NIR to the programs of the fpgagpu cores (board A's vertex core, board B's fragment core).
 */
#ifndef FG_COMPILER_H
#define FG_COMPILER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "compiler/nir/nir.h"

#define FG_VS_PARAMS 256 /* parameter vectors of the vertex core: the uniforms, then the constants */
#define FG_FS_PARAMS 64  /* of the fragment core */
#define FG_VARYINGS  8   /* varying vectors from the vertex to the fragment stage */

/* a 2D unit's border (the TMU clamps to the edge; the program mixes the border colour in) */
#define FG_BORDER_S      0x01 /* the s axis has a border: GL_CLAMP_TO_BORDER, or GL_CLAMP filtered */
#define FG_BORDER_T      0x02
#define FG_BORDER_S_HALF 0x04 /* GL_CLAMP: the coordinate stops at the edge, half a texel of border at most */
#define FG_BORDER_T_HALF 0x08
#define FG_BORDER_LINEAR 0x10 /* filtered: the border's share grows over a texel; else all past half a texel */
#define FG_BORDER_MIP    0x20 /* read with mipmaps: a level's texels are larger */

/* what a program depends on besides its NIR */
struct fg_compile_key {
   uint32_t xops;           /* fragment stage: board B's extra operations, STATUS b_flags bits 28-29: bit 0 SQRT / RCPM / RSQM, bit 1 CLAMP */
   uint32_t noops;          /* the units this stage's core lacks, STATUS flags bits 30-31: bit 0 the integer divider, bit 1 SIN / COS */
   uint64_t vs_outputs;     /* fragment stage: fg_varying_mask() of the vertex stage it runs with */
   uint32_t depth_samplers; /* fragment stage: the units a depth texture is bound to (the TMU reads its D24S8 word) */
   uint32_t depth16_samplers; /* of those, the ones whose word keeps a 16-bit depth (Z16: 0 .. 65535) */
   uint32_t shadow_func;    /* fragment stage: 3 bits a unit (units 0-7), a shadow read's compare function: 0 never,
                             * 1 less, 2 equal, 3 less-or-equal, 4 greater, 5 not equal, 6 greater-or-equal, 7 always */
   uint32_t swizzled;       /* fragment stage: the units whose texel is rearranged, as swizzle[unit] says */
   uint32_t logic;          /* fragment stage, a logic op's colour: 0 the program's, 1 all ones, 2 its complement */
   float lod_bias[8];       /* fragment stage: a unit's sampler LOD bias (GL_TEXTURE_LOD_BIAS), added to every read */
   /* fragment stage, 2D units 0-7, what the TMU has not got: */
   uint32_t lod_clamp;      /* the level worked out here and clamped to [lod_min, lod_max] (GL_TEXTURE_MIN_LOD / MAX_LOD) */
   uint8_t border[8];       /* FG_BORDER_*: the axes that read border_color past the image's edge */
   float lod_min[8], lod_max[8];
   float tex_size[8][2];    /* the width and height of the unit's first level (lod_clamp, the border) */
   float border_color[8][4];
   uint32_t point_flip;     /* fragment stage: gl_PointCoord.t is 1 - the board's (the rasterizer's sprite origin is
                             * the upper left of the surface's rows; the board's t runs from the lower left) */
   uint16_t swizzle[16];    /* 3 bits a component, R at bit 0 .. A at bit 9: 0-3 the texel's R, G, B, A; 4 zero; 5 one */
};

struct fg_program {
   uint64_t *words;
   unsigned nwords;
   uint32_t *consts;    /* FP24 bits, 4 per vector: the compiler's constants, the parameter vectors from const_base on */
   unsigned nconsts;    /* vectors */
   unsigned const_base; /* parameter vectors before the constants: constant buffer 0's first const_base vec4s */
   uint32_t varyings;   /* vertex stage: the varying slots it writes; fragment stage: those it reads (VS_STATE's fsmask) */
   uint32_t samplers;   /* fragment stage: the units it samples */
   bool discard;        /* a fragment stage with DISCARD */
   bool writes_depth;   /* a fragment stage that writes its depth */
   char *sasm;          /* the assembler text */
};

/* the varying locations a vertex stage passes on (VARYING_SLOT_* bits of its outputs_written); slot i is the i-th bit set */
uint64_t fg_varying_mask(uint64_t outputs_written);

/* an output of a vertex program (the clip position, a varying) worked out on the CPU: the instructions of its SSA form
 * the output depends on (inputs, uniforms, constants, arithmetic). fg_eval_create gives NULL when it depends on
 * anything else (control flow, a uniform at a computed index, an operation the evaluator does not do) */
struct fg_eval;
struct fg_eval *fg_eval_create(nir_shader *s, unsigned location);
void fg_eval_destroy(struct fg_eval *e);
uint32_t fg_eval_inputs(const struct fg_eval *e);   /* the attributes it reads */
/* in[i]: attribute i; uni: nvec vec4s (constant buffer 0). fg_eval_output: the mask of the components it wrote with
 * finite values; fg_eval_position: false when the position is not all there and finite */
unsigned fg_eval_output(struct fg_eval *e, const float (*in)[4], const float *uni, unsigned nvec, float out[4]);
bool fg_eval_position(struct fg_eval *e, const float (*in)[4], const float *uni, unsigned nvec, float pos[4]);

#define FG_EVAL_VARYINGS 4

/* what the CPU can work out of a vertex program: its position and its first varyings, each on its own */
struct fg_vs_eval {
   struct fg_eval *pos;                     /* NULL: not to be had */
   struct fg_eval *var[FG_EVAL_VARYINGS];
   uint8_t var_loc[FG_EVAL_VARYINGS];       /* their VARYING_SLOT_* */
};
void fg_vs_eval_free(struct fg_vs_eval *ev);
struct fg_eval *fg_vs_eval_varying(const struct fg_vs_eval *ev, unsigned location);

/* a fragment program that samples one 2D image at a varying and writes what it read as its colour, and does nothing
 * else: drawn with the varying running one to one with the window position, it copies the image */
struct fg_fs_copy {
   bool ok;
   uint8_t unit;              /* the texture and sampler unit */
   uint8_t loc[2], comp[2];   /* the varying location and component of the coordinates s, t */
   bool fract[2];             /* the coordinate is fract() of it (a repeating tile) */
};

/* the passes fg_compile needs first, on the driver's own copy of a shader (once per shader). From the last SSA form:
 * ev, when not NULL, gets a vertex program's evaluators; copy, when not NULL, whether a fragment program is a copy */
void fg_nir_lower(nir_shader *s, struct fg_vs_eval *ev, struct fg_fs_copy *copy);

/* 0, or -1 with the reason in err */
int fg_compile(nir_shader *s, const struct fg_compile_key *key, struct fg_program *out, char *err, size_t errsize);
void fg_program_free(struct fg_program *p);

/* FP24: sign 1, exponent 7 (bias 63), mantissa 16; round to nearest even, flush to zero */
uint32_t fg_fp24(double x);

#endif
