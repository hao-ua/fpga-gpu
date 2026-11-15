/* g2_render — the 3D layer every API library shares, on gpu2core: resident programs, the streaming vertex
 * ring, draw state with deduplication, render passes (the hardware's clear belongs to a FLUSH, so a clear starts a pass) and frames in flight.
 * A front end fills a g2r_state per draw; only what differs from the last draw reaches the wire. */
#ifndef G2_RENDER_H
#define G2_RENDER_H
#include "gpu2core.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct g2r g2r;

/* a program pair resident on the boards: VS in A's program buffer, FS in B's */
typedef struct g2r_prog {
    uint32_t id;
    uint32_t vs_off, vs_n, fs_off, fs_n;
    uint32_t fsmask;                    /* the varyings the fragment program reads */
    int fs_discard, fs_depth;           /* the fragment program has a DISCARD / writes its depth (STOZ): board A's coarse depth must not trust its quads' depth */
    uint32_t vs_const_base, vs_nconst, fs_const_base, fs_nconst;
    uint32_t *vs_const, *fs_const;      /* the compilers' constants, FP24 bits, 4 per vec4 */
} g2r_prog;

typedef struct g2r_attr {
    int enabled, size, type, normalized;
    uint16_t handle;
    uint32_t stride, offset;
    uint32_t constant[4];               /* FP24 bits: the value of a disabled attribute */
} g2r_attr;

/* blend factors and ops, stencil ops: the hardware's codes (pixel_blend_b, b_rop) */
enum { G2R_BF_ZERO, G2R_BF_ONE, G2R_BF_SRC_COLOR, G2R_BF_ONE_MINUS_SRC_COLOR, G2R_BF_DST_COLOR, G2R_BF_ONE_MINUS_DST_COLOR, G2R_BF_SRC_ALPHA,
       G2R_BF_ONE_MINUS_SRC_ALPHA, G2R_BF_DST_ALPHA, G2R_BF_ONE_MINUS_DST_ALPHA, G2R_BF_CONSTANT_COLOR, G2R_BF_ONE_MINUS_CONSTANT_COLOR,
       G2R_BF_CONSTANT_ALPHA, G2R_BF_ONE_MINUS_CONSTANT_ALPHA, G2R_BF_SRC_ALPHA_SATURATE };
enum { G2R_BOP_ADD, G2R_BOP_SUB, G2R_BOP_REV_SUB, G2R_BOP_MIN, G2R_BOP_MAX };
enum { G2R_SOP_KEEP, G2R_SOP_ZERO, G2R_SOP_REPLACE, G2R_SOP_INCR, G2R_SOP_DECR, G2R_SOP_INVERT, G2R_SOP_INCR_WRAP, G2R_SOP_DECR_WRAP };
/* sampler words */
enum { G2R_FILT_NEAREST = 0x0, G2R_WRAP_REPEAT = 0, G2R_WRAP_CLAMP = 1, G2R_WRAP_MIRROR = 2 };
#define G2R_FILTER(mag_linear, mip, min_linear) ((uint32_t)((mag_linear) ? 8 : 0) | (uint32_t)(mip) << 1 | (uint32_t)((min_linear) ? 1 : 0))   /* mip: 0 none, 1 nearest, 2 linear */
#define G2R_SAMPLER(handle, filter, wrap_s, wrap_t, incomplete) \
    ((uint32_t)(handle) | (uint32_t)(filter) << 16 | (uint32_t)(wrap_s) << 20 | (uint32_t)(wrap_t) << 22 | (uint32_t)((incomplete) ? 1 : 0) << 24)
#define G2R_SAMPLER3D(handle, filter, wrap_s, wrap_t, wrap_r, incomplete) (G2R_SAMPLER(handle, filter, wrap_s, wrap_t, incomplete) | (uint32_t)(wrap_r) << 25)

typedef struct g2r_state {
    const g2r_prog *prog;
    double vp_x, vp_y, vp_w, vp_h;      /* the viewport in pixels, in the target's own coordinates */
    double depth_near, depth_far;
    int z01;                            /* the program's z runs 0 .. w (not -w .. w): clipped at z >= 0, window z = depth_near + ndc_z (depth_far - depth_near). Board A g2r_a_build() >= G2_A_ZCLIP0 */
    int cull, front_cw;                 /* cull: 0 none, 1 back, 2 front, 3 both */
    double line_width;
    uint32_t h[23];                     /* DRAW_STATE h0..h22; h11, h12, h13 are filled here from prog */
    g2r_attr attr[16];
    const uint32_t *vs_uniforms;        /* slots 0 .. prog->vs_const_base - 1, FP24 bits */
    const uint32_t *fs_uniforms;        /* slots 0 .. prog->fs_const_base - 1 */
} g2r_state;

typedef struct g2r_target {
    uint16_t color, ds;                 /* image handles; ds 0: no depth / stencil attachment */
    uint32_t width, height;
    int depth16;
    int bits_r, bits_g, bits_b, bits_a; /* the colour channels: the clear colour and blend constant's units */
} g2r_target;

g2r *g2r_create(g2_board *bd);
void g2r_destroy(g2r *r);
g2_board *g2r_board(g2r *r);

/* memory */
uint32_t g2r_alloc_b(g2r *r, uint32_t bytes, uint32_t align);
void g2r_free_b(g2r *r, uint32_t base);
uint32_t g2r_alloc_a(g2r *r, uint32_t bytes, uint32_t align);
void g2r_free_a(g2r *r, uint32_t base);
int g2r_image_create(g2r *r, uint16_t *handle, uint32_t *base, uint32_t w, uint32_t h, uint32_t fmt, uint32_t levels, uint32_t layers, int origin, uint32_t usage);
/* tight rows on a single level: what a texture needs, the TMU's descriptor has no pitch. g2r_image_create gives a single level a pitch that is a multiple of 32, for render targets and scanout */
int g2r_image_create_tight(g2r *r, uint16_t *handle, uint32_t *base, uint32_t w, uint32_t h, uint32_t fmt, uint32_t levels, uint32_t layers, int origin, uint32_t usage);
/* 3D textures in the TMU (a board B built with them); GPU2_NO_3D=1 says no whatever the board. Limits G2R_VOL_WH, G2R_VOL_D. Destroy with g2r_image_destroy, an alias with g2r_alias_destroy */
#define G2R_VOL_WH 256
#define G2R_VOL_D 255
int g2r_has_3d(g2r *r);
int g2r_a_build(g2r *r);                             /* board A's build number, read once (ask it outside a pass); < 0: no STATUS */
int g2r_volume_create(g2r *r, uint16_t *handle, uint32_t *base, uint32_t w, uint32_t h, uint32_t d, uint32_t fmt, uint32_t levels, uint32_t usage);
int g2r_volume_alias(g2r *r, uint16_t *handle, uint32_t base, uint32_t w, uint32_t h, uint32_t d, uint32_t fmt, uint32_t levels, uint32_t usage);
void g2r_image_destroy(g2r *r, uint16_t handle, uint32_t base);
/* an image the open pass may still sample: its descriptor goes now, its memory once the pass's FLUSH is sent (draws already given keep their texels) */
void g2r_image_retire(g2r *r, uint16_t handle, uint32_t base);
/* copy-on-write: the image (made by g2r_image_create, or g2r_image_create_tight when tight, with these parameters) copied on board B under a new
   handle, the old one retired — what an application changes after the pass sampled it, without ending the pass. level_mask: bit l = copy level l
   (of every layer); a level the caller rewrites whole needs none. <0: no room */
int g2r_image_rename(g2r *r, uint16_t *handle, uint32_t *base, uint32_t w, uint32_t h, uint32_t fmt, uint32_t levels, uint32_t layers, int origin, uint32_t usage, int tight,
                     uint32_t level_mask);
int g2r_is_target(g2r *r, uint16_t handle);         /* the open pass renders into it (colour or depth-stencil) */
/* a second image over memory that already has one (a tail of a mip chain, the same bytes in another format). It has no memory of its own, and a handle used again is a new image to the state cache */
int g2r_image_alias(g2r *r, uint16_t *handle, uint32_t base, uint32_t w, uint32_t h, uint32_t fmt, uint32_t pitch, int origin, uint32_t levels, uint32_t usage);
void g2r_alias_destroy(g2r *r, uint16_t handle);
int g2r_handle_alloc(g2r *r, uint16_t kind, uint16_t *handle);
void g2r_handle_free(g2r *r, uint16_t handle);

/* programs: words + constants as the compiler gives them */
g2r_prog *g2r_prog_create(g2r *r, const uint64_t *vs, uint32_t vs_n, const double (*vs_const)[4], uint32_t vs_nconst, uint32_t vs_const_base,
                          const uint64_t *fs, uint32_t fs_n, const double (*fs_const)[4], uint32_t fs_nconst, uint32_t fs_const_base, uint32_t fsmask);

/* per-frame data for board A: copies `bytes` into the streaming ring, returns the offset in g2r_stream_handle's buffer */
uint32_t g2r_stream(g2r *r, const void *data, uint32_t bytes, uint32_t align);
uint16_t g2r_stream_handle(g2r *r);
uint32_t g2r_stream_gen(g2r *r);                     /* changes when the ring wraps: what was streamed before may be overwritten from then on */
/* client arrays: `base` = the element of vertex 0 (NULL: skip). Uploads vertices lo .. lo + span - 1 of each; arrays that overlap with the same stride (interleaved) go up once; off = the ring offset of element lo */
typedef struct g2r_vsrc { const uint8_t *base; int stride, bytes; uint32_t off; } g2r_vsrc;
void g2r_gather(g2r *r, g2r_vsrc *in, int n, uint32_t lo, uint32_t span);

/* A buffer of the application's that stops changing is kept on board A, so its draws send no vertices. g2r_keep before a draw that reads it: 1 = it is there, k->off bytes into g2r_kept_handle's buffer.
 * It moves after a few frames drawn without a change. A draw can still be fetching on A when later messages run, so a copy is never written twice: g2r_keep_changed (the application writes the buffer, or frees it)
 * drops it, draws stream again, and the memory comes back when the frames that used it have finished. All zero is the initial state. GPU2_KEEP_MB: the room on A (256), 0 = stream everything. */
typedef struct g2r_kept { struct g2r_kept *next, **pprev; uint8_t *sent; uint32_t off, bytes, last, stable, drops; int watch; } g2r_kept;
int g2r_keep(g2r *r, g2r_kept *k, const void *mem, uint32_t bytes);
void g2r_keep_changed(g2r *r, g2r_kept *k);          /* r may be NULL while nothing is kept */
/* Set k->watch where the application holds a pointer into mem: Direct3D's buffers. A Lock is a promise, not a wall, so a game can write after Unlock or under D3DLOCK_READONLY and no call of ours sees it.
 * Then what was sent is held on the PC, and a draw that g2r_keep says is kept asks g2r_kept_same about the n bytes at off it will read: 0 = they differ, the copy has been dropped, stream this draw. */
int g2r_kept_same(g2r *r, g2r_kept *k, const void *mem, uint32_t off, uint32_t n);
uint16_t g2r_kept_handle(g2r *r);

/* passes */
void g2r_set_target(g2r *r, const g2r_target *t);    /* ends the pass when the target changes */
/* flags: G2_FL_CLEAR_COLOR | G2_FL_CLEAR_DEPTH | G2_FL_CLEAR_STENCIL, optional G2_FL_CLEAR_SCISSOR (sc = x0, y0, x1, y1), G2_FL_DITHER, G2_FL_CMASK */
void g2r_clear(g2r *r, uint32_t flags, const double rgba[4], uint32_t depth, uint32_t stencil, uint32_t stencil_mask, const uint32_t sc[4]);
void g2r_scissor_viewport(g2r_state *s, uint32_t width, uint32_t height);   /* keeps a draw to s's viewport by the scissor: call it after h and vp are filled in */
void g2r_draw(g2r *r, const g2r_state *s, int mode, uint32_t count, uint32_t first, int indexed, uint16_t ihandle, uint32_t ioffset, int index_u8);
void g2r_end_pass(g2r *r);                           /* FLUSH: everything so far is rendered, in order */
/* the next pass into `image` need not keep its colour (a back buffer after a swap: undefined): that pass skips the band load, the most expensive thing a frame can do */
void g2r_discard_color(g2r *r, uint16_t image);
/* a new depth-stencil image: its stencil is undefined, so any value may stand for it */
void g2r_stencil_undefined(g2r *r, uint16_t ds);
void g2r_stencil_written(g2r *r, uint16_t ds);       /* WRITE_IMAGE gave it other stencil values: what a whole-image depth clear assumed no longer holds */
uint32_t g2r_pass_id(g2r *r);                        /* the open pass; B runs WRITE_IMAGE at once but draws at the FLUSH: end a pass before changing what it sampled */
void g2r_finish(g2r *r);                             /* + wait for the board */

/* presentation: ends the pass, PRESENTs, keeps at most two frames in flight */
void g2r_present(g2r *r, uint16_t image, int hold);

/* diagnostics */
typedef struct g2r_stats { uint64_t draws, draw_states, passes, frames, stream_bytes, keep_bytes, kept_binds, kept_unseen; uint32_t kept_now, kept_mb; uint64_t renames; } g2r_stats;
void g2r_get_stats(g2r *r, g2r_stats *s);
/* GPU2_STATS=seconds: a line per period in the log, through the application's own socket (the boards answer whoever spoke last: a second program asking for STATUS steals the game's ACKs) */
void g2r_stats_tick(g2r *r);

#ifdef __cplusplus
}
#endif
#endif
