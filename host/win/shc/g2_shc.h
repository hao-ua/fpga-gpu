/* g2_shc.h — the gpu2 GLSL ES 1.00 compiler in C: a bit-exact port of tools/glslc.py (the isa64 production path:
 * compile_stage(src, stage, isa64=True), optimizer on, hw sin/cos, MAD fusion, the register-spill fallback to
 * rematerialized constants) and of the assembler half of tools/isa64.py. Python stays the oracle: host/win/shc/test/.
 *
 * Re-entrant: every compile owns its state. Nothing is global, nothing exits or aborts; every failure comes back as
 * text in result.error. Only the C standard library is used.
 *
 * x86 builds must use -msse2 -mfpmath=sse (host/win/shc/build_shc.ps1): x87 double rounding changes folded constants. */
#ifndef G2_SHC_H
#define G2_SHC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* one symbol. attribs: name -> (index, type). uniforms: name -> (slot, type[, count]). samplers: name -> unit.
 * varyings: name -> (index, type). The order is glslc's dict order (first insertion). */
typedef struct g2sc_sym {
    char name[64];              /* truncated to 63 characters: lname is the whole name */
    int  loc;
    char type[16];              /* truncated to 15 characters: ltype is the whole type (a struct name, for a struct) */
    int  array;                 /* element count, 0 = not an array */
    /* additions to the API */
    int  has_array;             /* 1 = glslc's 3-tuple form: `array` is meaningful even when it is 0 or negative */
    const char *lname;          /* NUL-terminated, owned by the result */
    const char *ltype;          /* samplers: the declared type (glslc's table has none) */
} g2sc_sym;

/* the layout of a struct used by a uniform, in uniform slots (glslc's metadata has only the base slot; a front end
 * needs this to resolve "lights[2].ambient": slot = base + index * struct.nslots + member.offset + element) */
typedef struct g2sc_member {
    const char *name;
    const char *type;           /* a struct name for a nested struct */
    int offset;                 /* slots from the start of the struct */
    int nslots;                 /* slots of the whole member (array count included) */
    int array;                  /* element count, 0 = not an array */
    int has_array;
    const char *prec;           /* "lowp" / "mediump" / "highp": its own or the default where the struct is defined; NULL for bool and struct types (static, not freed) */
} g2sc_member;

typedef struct g2sc_struct {
    const char *name;
    int nslots;                 /* one element */
    g2sc_member *members; int nmembers;
} g2sc_struct;

typedef struct g2sc_result {
    uint64_t *words; int nwords;                 /* assembled 64-bit instructions */
    g2sc_sym *attribs;  int nattribs;            /* glslc attribs: name -> (index, type) */
    g2sc_sym *uniforms; int nuniforms;           /* glslc uniforms: name -> (slot, type[, count]) */
    g2sc_sym *samplers; int nsamplers;           /* glslc samplers: name -> unit */
    g2sc_sym *varyings; int nvaryings;           /* glslc varyings: name -> (index, type) */
    int const_base; double (*const_pool)[4]; int nconst;
    char error[512];                             /* empty on success */
    /* additions to the API */
    g2sc_struct *structs; int nstructs;          /* struct types reachable from the uniforms, in definition order */
    char *sasm;                                  /* the assembler text compile_stage returned ('\n' separated) */
    int remat;                                   /* 1 = the register-spill fallback ran (constants rematerialized) */
    int vs_tex;                                  /* a vertex program samples a texture: legal to compile, MAX_VERTEX_TEXTURE_IMAGE_UNITS (0) makes the link refuse it */
    int oversize;                                /* more uniforms and constants, or varyings, than the stage has: tables and sasm are valid, there are no words. A link packs it (g2sc_compile_packed) or refuses it */
} g2sc_result;

enum { G2SC_VS = 0, G2SC_FS = 1 };

/* 0 ok, < 0 error with out->error. out is overwritten (not freed): start from a fresh or g2sc_free'd result. */
int g2sc_compile(const char *src, int stage, g2sc_result *out);
/* a program from an application: every error GLSL ES 1.00 requires. g2sc_compile is for the libraries' own programs: it only reports what it cannot compile. */
int g2sc_compile_strict(const char *src, int stage, g2sc_result *out);
/* a program's stages share varyings by name, the hardware by number: varying_order gives a linked vertex stage's names in index order. A varying of this shader named there takes that index;
 * any other takes the next index after n_order, in declaration order, so a link can still report it. NULL / 0 = g2sc_compile.
 * Arrays and matrices take a run of indices: entry i names the varying that starts at index i, the rest are NULL. */
int g2sc_compile_ex(const char *src, int stage, const char *const *varying_order, int n_order, g2sc_result *out);
/* a vertex stage again, knowing its link: bit i of unread_varyings = the fragment stage never reads varying slot i, so the slot is not written and what only it needed is not computed.
 * Slots, attribute and uniform tables stay those of g2sc_compile; a slot left unwritten must stay out of the program's fsmask. */
int g2sc_compile_vs(const char *src, uint32_t unread_varyings, g2sc_result *out);
/* GLSL ES Appendix A.7: a program whose uniforms or varyings do not fit one row a variable. umap[slot] = row * 4 + lane of a uniform slot and urows the rows they take, the constants start there; vmap[slot] the same of a
 * varying slot, -1 = not passed on, nvmap entries. Either may be NULL: as declared. unread as g2sc_compile_vs, without vmap. Tables stay those of g2sc_compile. */
typedef struct g2sc_pack { const int *umap; int urows; const int *vmap; int nvmap; uint32_t unread; uint32_t depth_units; } g2sc_pack;   /* depth_units: bit per sampler, it reads a depth texture: (z, z, z, 1) from the D24S8 word */
int g2sc_compile_packed(const char *src, int stage, const char *const *varying_order, int n_order, const g2sc_pack *pack, g2sc_result *out);
/* the placement of Appendix A.7. rank: 0 mat4, 1 mat2, 2 vec4, 3 mat3, 4 vec3, 5 vec2, 6 float; rows[i]: its vectors, array included. out[i] = row * 4 + lane of its first vector, the rest follow a row each.
 * Returns the rows in use, counted to the last one, or -1: it does not fit in max_rows */
int g2sc_pack_vars(const int *rank, const int *rows, int n, int max_rows, int *out);
/* a program of two stages, as a library links one: the fragment stage in the vertex stage's varying numbering, what it never reads not computed, Appendix A.7 when one row a variable does not fit.
 * vs and fs are what was compiled. 0 ok; umap_* (malloc'd, NULL = as declared) say where each uniform slot went, row * 4 + lane. < 0: error says why, nothing to free */
typedef struct g2sc_linked {
    g2sc_result vs, fs;
    uint32_t fsmask;
    int *umap_vs, *umap_fs;
    int vpacked, vmap[64];                       /* the varyings by Appendix A.7: vmap[loc of the vertex stage's table] = row * 4 + lane it leaves in, -1 not passed on */
    char error[512];
} g2sc_linked;
int g2sc_link(const char *vs, const char *fs, g2sc_linked *out);
int g2sc_link_depth(const char *vs, const char *fs, uint32_t depth_units, g2sc_linked *out);     /* the same, g2sc_pack's depth_units in the fragment stage */
void g2sc_link_free(g2sc_linked *l);
void g2sc_free(g2sc_result *r);

/* The one setting that is not a compile's own: the fragment-core operations the board B in use has, glslc.compile_stage's xops. Bit 0: SQRT, RCPM and RSQM; bit 1: CLAMP —
 * STATUS b_flags bits 28 / 29 (G2_B_FLAG_XOPS in gpu2core.h), which every board B before them reads as 0 and would run as no-ops. A library sets it when it opens the
 * board, before it compiles; 0 (the default) emits none. Fragment programs only; the code is bit for bit what it is without them. */
void g2sc_set_target(uint32_t xops);
uint32_t g2sc_target(void);

/* The units a stage's core lacks, glslc.compile_stage's noops, one mask per stage: bit 0 the integer divider (int / is lowered to RCP and integer ops), bit 1
 * SIN / COS (lowered to a polynomial) — STATUS flags bits 30 / 31: board A's for the vertex core, board B's b_flags for the fragment core (G2_*_FLAG_ABSENT in
 * gpu2core.h); every build that has the units reads 0. A library sets it when it opens the board, before it compiles; 0 (the default) emits the units' ops. */
void g2sc_set_absent(uint32_t vs, uint32_t fs);
uint32_t g2sc_absent(int stage);

/* isa64.assemble(lines, vs, max_if, max_loop) on its own, for hand-written programs; max_if / max_loop < 0 = no limit
 * (Python's None). The default limits are 8 and 4. *words is malloc'd, release it with free(). 0 ok, < 0 error. */
int g2sc_assemble(const char *sasm, int vs, int max_if, int max_loop, uint64_t **words, int *nwords, char *error, int errsize);

#ifdef __cplusplus
}
#endif
#endif
