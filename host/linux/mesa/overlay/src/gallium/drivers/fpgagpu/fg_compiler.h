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

/* what a program depends on besides its NIR */
struct fg_compile_key {
   uint32_t xops;           /* fragment stage: board B's extra operations, STATUS b_flags bits 28-29: bit 0 SQRT / RCPM / RSQM, bit 1 CLAMP */
   uint32_t noops;          /* the units this stage's core lacks, STATUS flags bits 30-31: bit 0 the integer divider, bit 1 SIN / COS */
   uint64_t vs_outputs;     /* fragment stage: fg_varying_mask() of the vertex stage it runs with */
   uint32_t depth_samplers; /* fragment stage: the units a depth texture is bound to (the TMU reads its D24S8 word) */
   uint32_t swizzled;       /* fragment stage: the units whose texel is rearranged, as swizzle[unit] says */
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

/* the passes fg_compile needs first, on the driver's own copy of a shader (once per shader) */
void fg_nir_lower(nir_shader *s);

/* 0, or -1 with the reason in err */
int fg_compile(nir_shader *s, const struct fg_compile_key *key, struct fg_program *out, char *err, size_t errsize);
void fg_program_free(struct fg_program *p);

/* FP24: sign 1, exponent 7 (bias 63), mantissa 16; round to nearest even, flush to zero */
uint32_t fg_fp24(double x);

#endif
