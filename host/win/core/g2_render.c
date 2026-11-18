/* g2_render: see g2_render.h. The shape of a frame is fb_cube.render_frame generalised:
 *   BATCH { FRAME_BEGIN, [VS_PROGRAM] [VERTEX_ATTR] [VS_UNIFORMS] [DRAW_STATE] [VS_STATE] DRAW ... [WRITE_BUFFER ...] FLUSH } PRESENT
 * The hardware keeps a pass's clear in its FLUSH: a clear before the pass's first draw merges into it, a clear after a draw ends the pass.
 * Draw ids: a DRAW_STATE stays in B's bucket until overwritten, so an id is reused for the same words + uniforms, across frames too. Handles are
 * resolved when a DRAW_STATE is stored, so image create / destroy starts a new generation of ids.
 */
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "g2_env.h"
#include "g2_render.h"
#include "gen/clear_programs.h"

#define PROG_A_BYTES (4u << 20)
#define PROG_B_BYTES (16u << 20)
#define RING_BYTES (128u << 20)
#define MAX_IDS 8192
#define ID_SLOTS 16384
#define MAX_RING_FRAMES 3
#define KEEP_FRAMES 2                   /* frames drawn unchanged after the first before a buffer moves to A, doubled each time it is dropped: uploading a whole buffer is not free */
#define KEEP_DROPS_MAX 6
#define MAX_DEAD 1024
#define QUAR_MAX 256                    /* image handles held back after a rename (g2r_image_retire) before they go back to the table */

typedef struct id_entry { uint64_t hash; uint32_t id; } id_entry;
typedef struct dead_copy { uint32_t off, frame; } dead_copy;    /* frame: the last that may still read it */

struct g2r {
    g2_board *bd;
    g2_heap *heap_a, *heap_b;
    g2_handles h_buf_a, h_buf_b, h_img;
    uint16_t prog_a, prog_b, ring;
    uint32_t prog_a_used, prog_b_used, next_prog;
    uint32_t ring_used, ring_gen;
    uint32_t ring_at_frame[2];          /* ring position when each in-flight frame was presented */

    /* buffers kept on board A: one buffer with its own allocator, so a kept copy costs no handle and no message */
    uint16_t keep;
    int keep_state;                     /* 0 not set up, 1 ready, -1 off */
    uint32_t keep_room;
    g2_heap *keep_heap;
    g2r_kept *keep_list;
    dead_copy dead[MAX_DEAD];
    int ndead;
    uint32_t frame_pending[2], frame_done;
    double wait_s;                      /* how long the last PRESENT took to answer */

    /* the pass */
    int pass_open, pass_draws;
    g2r_target tgt;
    int have_tgt;
    uint32_t cl_flags, cl_depth, cl_stencil, cl_smask, cl_sc[4], cl_c16[4];
    uint16_t discard_image;             /* the next FLUSH into it need not load its colour */
    g2r_prog *clear_prog;
    /* depth and stencil share the D24S8 word: a depth clear alone must load it, unless the stencil's value is known and can be cleared along */
    uint16_t stencil_ds;
    int stencil_known;
    uint32_t stencil_value;
    int has_3d;                         /* 0 not asked yet, 1 yes, -1 no */

    /* frames in flight */
    g2_ref pending[2];
    int npending;
    uint32_t frame;

    /* what board A holds */
    const g2r_prog *cur_prog;
    g2r_attr sh_attr[16];
    int sh_attr_ok;
    uint32_t sh_uni[256][4];
    uint8_t sh_uni_ok[256];
    double sh_vp[4], sh_dn, sh_df, sh_lw;
    int sh_z01, a_build;
    int skip_fl;                        /* G2_FL_SKIP_BANDS or 0 on every FLUSH; -1 not decided yet */
    int sh_cull, sh_cw, sh_vs_ok;
    uint32_t sh_fsmask, sh_w, sh_h, sh_id, sh_zc;

    /* draw ids */
    id_entry ids[ID_SLOTS];
    uint32_t next_id, id_gen;

    /* images retired while the pass may still sample them (g2r_image_retire): their memory goes back once the pass's FLUSH is sent,
       their handles once quar fills up, with a new draw-id generation */
    uint16_t quar[QUAR_MAX];
    int nquar;
    uint32_t *pass_free;
    int npass_free, cap_pass_free;

    g2r_stats st;
};

static uint64_t fnv(uint64_t h, const void *p, size_t n)
{
    const uint8_t *b = (const uint8_t *)p;
    size_t i;
    for (i = 0; i < n; i++) { h ^= b[i]; h *= 0x100000001B3ull; }
    return h;
}

g2r *g2r_create(g2_board *bd)
{
    g2r *r = (g2r *)calloc(1, sizeof *r);
    uint32_t a, b, rn;
    if (!r) return NULL;
    r->bd = bd;
    r->skip_fl = -1;
    r->heap_a = g2_heap_create(G2_HEAP_A, G2_HEAP_A_END);
    r->heap_b = g2_heap_create(G2_HEAP_B, G2_HEAP_B_END);
    r->next_id = 1;
    r->next_prog = 1;
    a = g2_heap_alloc(r->heap_a, PROG_A_BYTES, 4096);
    b = g2_heap_alloc(r->heap_b, PROG_B_BYTES, 4096);
    rn = g2_heap_alloc(r->heap_a, RING_BYTES, 4096);
    if (!a || !b || !rn || g2_handle_alloc(&r->h_buf_a, G2_HK_BUF_A, &r->prog_a) < 0 || g2_handle_alloc(&r->h_buf_b, G2_HK_BUF_B, &r->prog_b) < 0 ||
        g2_handle_alloc(&r->h_buf_a, G2_HK_BUF_A, &r->ring) < 0) {
        free(r);
        return NULL;
    }
    g2_create_buffer(bd, r->prog_a, a, PROG_A_BYTES, 0);
    g2_create_buffer(bd, r->prog_b, b, PROG_B_BYTES, 0);
    g2_create_buffer(bd, r->ring, rn, RING_BYTES, 0);
    r->h_img.limit = G2_HANDLES_MAX;    /* image handles are virtual: gpu2core maps them onto board B's table */
    r->clear_prog = g2r_prog_create(r, clear_vs_words, clear_vs_nwords, clear_vs_const, clear_vs_nconst, clear_vs_const_base, clear_fs_words, clear_fs_nwords,
                                    clear_fs_const, clear_fs_nconst, clear_fs_const_base, 0);
    return r;
}

void g2r_destroy(g2r *r)
{
    if (!r) return;
    g2_heap_destroy(r->heap_a);
    g2_heap_destroy(r->heap_b);
    free(r->pass_free);
    free(r);
}

g2_board *g2r_board(g2r *r) { return r->bd; }
uint32_t g2r_alloc_b(g2r *r, uint32_t bytes, uint32_t align) { return g2_heap_alloc(r->heap_b, bytes, align); }
void g2r_free_b(g2r *r, uint32_t base) { g2_heap_free(r->heap_b, base); }
uint32_t g2r_alloc_a(g2r *r, uint32_t bytes, uint32_t align) { return g2_heap_alloc(r->heap_a, bytes, align); }
void g2r_free_a(g2r *r, uint32_t base) { g2_heap_free(r->heap_a, base); }

int g2r_handle_alloc(g2r *r, uint16_t kind, uint16_t *handle)
{
    g2_handles *t = kind == G2_HK_BUF_A ? &r->h_buf_a : kind == G2_HK_BUF_B ? &r->h_buf_b : &r->h_img;
    return g2_handle_alloc(t, kind, handle);
}

void g2r_handle_free(g2r *r, uint16_t handle)
{
    g2_handles *t = (handle & 0xC000) == 0xC000 ? &r->h_img : (handle & 0x8000) ? &r->h_buf_b : &r->h_buf_a;
    g2_handle_free(t, handle);
}

/* the handles held back after renames go back to the table; a new draw-id generation, as for any destroy, keeps a reused handle from matching a
   draw state that named the image it was before */
static void quar_release(g2r *r)
{
    int i;
    if (!r->nquar) return;
    for (i = 0; i < r->nquar; i++) g2_handle_free(&r->h_img, r->quar[i]);
    r->nquar = 0;
    r->id_gen++;
}

static int handle_take(g2r *r, uint16_t *hd)
{
    if (g2_handle_alloc(&r->h_img, G2_HK_IMG_B, hd) >= 0) return 0;
    quar_release(r);
    return g2_handle_alloc(&r->h_img, G2_HK_IMG_B, hd);
}

/* board B loads and stores a depth-stencil row in as many 32-byte bursts as its colour target's row takes, times the bursts of a 16-pixel group
   (two for a 16-bit colour format), and refuses a FLUSH whose depth-stencil pitch is shorter. A multiple of 64 bytes holds that for a colour
   target of the same width in any format, at the pitch g2r_image_create gives it or at tight rows */
uint32_t g2r_ds_pitch(uint32_t w) { return (w * 4 + 63) & ~63u; }

static int image_alloc(g2r *r, uint16_t *handle, uint32_t *base, uint32_t w, uint32_t h, uint32_t fmt, uint32_t levels, uint32_t layers, int origin, uint32_t usage, int tight, int bump)
{
    uint32_t bpp = g2_storage_bpp(fmt), pitch, bytes, a;
    uint16_t hd;
    /* a mip chain has tight rows; a single level gets a pitch that is a multiple of 32, as render targets and scanout need, and a depth-stencil
       image (or the image whose bytes one aliases: tight 2) the pitch board B loads its rows at */
    if (tight == 2 || (fmt == G2_FMT_D24S8 && !tight && levels == 1 && layers == 1)) pitch = g2r_ds_pitch(w);
    else pitch = tight || levels > 1 || layers > 1 ? w * bpp : (w * bpp + 31) & ~31u;
    bytes = levels > 1 || layers > 1 ? g2_image_bytes(w, h, bpp, levels, layers) : pitch * h;
    if (handle_take(r, &hd) < 0) return -1;
    /* board B renders in bands of up to 8 rows and loads and stores every row of the last band (b_rop), also past an image
     * whose height is no multiple of 8: those rows get memory of the image's own, or they would rewrite the next image's
     * bytes, and zero those that fall in the row padding of a pitch wider than the image */
    a = g2_heap_alloc(r->heap_b, bytes + (((h + 7) & ~7u) - h) * pitch, 32);
    if (!a) { g2_handle_free(&r->h_img, hd); return -1; }
    g2_create_image(r->bd, hd, a, w, h, fmt, pitch, origin, levels, layers, 0, usage);
    if (bump) r->id_gen++;
    *handle = hd;
    *base = a;
    return 0;
}

static int image_create(g2r *r, uint16_t *handle, uint32_t *base, uint32_t w, uint32_t h, uint32_t fmt, uint32_t levels, uint32_t layers, int origin, uint32_t usage, int tight)
{
    return image_alloc(r, handle, base, w, h, fmt, levels, layers, origin, usage, tight, 1);
}

int g2r_image_create(g2r *r, uint16_t *handle, uint32_t *base, uint32_t w, uint32_t h, uint32_t fmt, uint32_t levels, uint32_t layers, int origin, uint32_t usage)
{
    return image_create(r, handle, base, w, h, fmt, levels, layers, origin, usage, 0);
}

int g2r_image_create_tight(g2r *r, uint16_t *handle, uint32_t *base, uint32_t w, uint32_t h, uint32_t fmt, uint32_t levels, uint32_t layers, int origin, uint32_t usage)
{
    return image_create(r, handle, base, w, h, fmt, levels, layers, origin, usage, 1);
}

int g2r_image_create_ds_rows(g2r *r, uint16_t *handle, uint32_t *base, uint32_t w, uint32_t h, uint32_t fmt, int origin, uint32_t usage)
{
    return image_create(r, handle, base, w, h, fmt, 1, 1, origin, usage, 2);
}

int g2r_a_build(g2r *r)
{
    if (!r->a_build) {
        uint32_t st[G2_ST_WORDS];
        r->a_build = g2_status(r->bd, st, NULL) == 0 ? (int)st[G2_ST_BUILD] : -1;
    }
    return r->a_build;
}

/* STATUS has no build number of board B. On a B with 3D textures a depth counts towards the chain: 1 x 1 x 2 with two levels is legal, and a B without them answers ERR_RANGE */
int g2r_has_3d(g2r *r)
{
    if (!r->has_3d) {
        g2_error e[8];
        uint16_t hd;
        uint32_t a;
        r->has_3d = -1;
        if (!g2e_get_int("GPU2_NO_3D", 0) && g2_handle_alloc(&r->h_img, G2_HK_IMG_B, &hd) >= 0) {
            if ((a = g2_heap_alloc(r->heap_b, 32, 32)) != 0) {
                g2_take_errors(r->bd, e, 8);
                g2_create_volume(r->bd, hd, a, 1, 1, 2, G2_FMT_RGBA8, 2, G2_USE_SAMPLED);
                if (g2_fence(r->bd, 5.0) == 0 && !g2_take_errors(r->bd, e, 8)) r->has_3d = 1;
                g2_destroy(r->bd, hd);
                g2_heap_free(r->heap_b, a);
            }
            g2_handle_free(&r->h_img, hd);
        }
    }
    return r->has_3d > 0;
}

int g2r_volume_create(g2r *r, uint16_t *handle, uint32_t *base, uint32_t w, uint32_t h, uint32_t d, uint32_t fmt, uint32_t levels, uint32_t usage)
{
    uint32_t a;
    uint16_t hd;
    if (handle_take(r, &hd) < 0) return -1;
    a = g2_heap_alloc(r->heap_b, (g2_vol_chain_bytes(w, h, d, g2_storage_bpp(fmt), levels) + 31) & ~31u, 32);
    if (!a) { g2_handle_free(&r->h_img, hd); return -1; }
    g2_create_volume(r->bd, hd, a, w, h, d, fmt, levels, usage);
    r->id_gen++;
    *handle = hd;
    *base = a;
    return 0;
}

int g2r_volume_alias(g2r *r, uint16_t *handle, uint32_t base, uint32_t w, uint32_t h, uint32_t d, uint32_t fmt, uint32_t levels, uint32_t usage)
{
    uint16_t hd;
    if (handle_take(r, &hd) < 0) return -1;
    g2_create_volume(r->bd, hd, base, w, h, d, fmt, levels, usage);
    r->id_gen++;
    *handle = hd;
    return 0;
}

void g2r_image_destroy(g2r *r, uint16_t handle, uint32_t base)
{
    if (r->discard_image == handle) r->discard_image = 0;      /* the handle may come back as another image, whose contents count */
    g2_destroy(r->bd, handle);
    g2_handle_free(&r->h_img, handle);
    g2_heap_free(r->heap_b, base);
    r->id_gen++;
}

/* An image the open pass may still sample. Board B resolves a sampler into an address when the DRAW_STATE arrives and renders at the FLUSH, so its
   descriptor can go now, while its memory must stay untouched until the FLUSH is sent: later commands run after it. Its handle is held back:
   draw states of this generation name it, and another image under the same handle would match them. */
void g2r_image_retire(g2r *r, uint16_t handle, uint32_t base)
{
    if (!r->pass_open || !r->pass_draws) { g2r_image_destroy(r, handle, base); return; }
    g2_destroy(r->bd, handle);
    if (r->nquar == QUAR_MAX) quar_release(r);
    r->quar[r->nquar++] = handle;
    if (r->npass_free == r->cap_pass_free) {
        int cap = r->cap_pass_free ? 2 * r->cap_pass_free : 64;
        uint32_t *p = (uint32_t *)realloc(r->pass_free, (size_t)cap * sizeof *p);
        if (!p) { g2r_end_pass(r); g2_heap_free(r->heap_b, base); return; }     /* no room for the list: the pass ends and the memory goes back now */
        r->pass_free = p; r->cap_pass_free = cap;
    }
    r->pass_free[r->npass_free++] = base;
}

/* A copy of the image under a new handle, which the caller then changes: draws of the open pass keep sampling the old one, which is retired.
   Every level in level_mask (bit l: level l; levels the caller rewrites whole need no copy) of every layer is copied on board B. tight: 1 the
   layout of g2r_image_create_tight, 2 that of g2r_image_create_ds_rows. The new handle starts no new draw-id generation: it is one no draw state of this generation names.
   <0: no room (the caller ends the pass instead) */
int g2r_image_rename(g2r *r, uint16_t *handle, uint32_t *base, uint32_t w, uint32_t h, uint32_t fmt, uint32_t levels, uint32_t layers, int origin, uint32_t usage, int tight,
                     uint32_t level_mask)
{
    uint16_t nh;
    uint32_t nb, l, f;
    if (image_alloc(r, &nh, &nb, w, h, fmt, levels, layers, origin, usage, tight, 0) < 0) return -1;
    for (f = 0; f < layers; f++) {
        for (l = 0; l < levels && l < 32; l++) {
            uint32_t lw = w >> l ? w >> l : 1, lh = h >> l ? h >> l : 1;
            if (!(level_mask >> l & 1)) continue;
            g2_copy_image(r->bd, *handle, nh, 0, 0, lw, lh, 0, 0, lw, lh, 0, 0, 0, 0, 0, l, f, l, f);
        }
    }
    g2r_image_retire(r, *handle, *base);
    *handle = nh; *base = nb;
    r->st.renames++;
    return 0;
}

int g2r_is_target(g2r *r, uint16_t handle) { return handle && r->have_tgt && (r->tgt.color == handle || r->tgt.ds == handle); }

int g2r_image_alias(g2r *r, uint16_t *handle, uint32_t base, uint32_t w, uint32_t h, uint32_t fmt, uint32_t pitch, int origin, uint32_t levels, uint32_t usage)
{
    uint16_t hd;
    if (handle_take(r, &hd) < 0) return -1;
    g2_create_image(r->bd, hd, base, w, h, fmt, pitch, origin, levels, 1, 0, usage);
    r->id_gen++;
    *handle = hd;
    return 0;
}

void g2r_alias_destroy(g2r *r, uint16_t handle)
{
    g2_destroy(r->bd, handle);
    g2_handle_free(&r->h_img, handle);
    r->id_gen++;
}

static uint32_t *const_bits(const double (*c)[4], uint32_t n)
{
    uint32_t *o = (uint32_t *)malloc((size_t)(n ? n : 1) * 16), i, k;
    if (!o) return NULL;
    for (i = 0; i < n; i++) for (k = 0; k < 4; k++) o[4 * i + k] = g2_fp24(c[i][k]);
    return o;
}

g2r_prog *g2r_prog_create(g2r *r, const uint64_t *vs, uint32_t vs_n, const double (*vc)[4], uint32_t vs_nconst, uint32_t vs_const_base,
                          const uint64_t *fs, uint32_t fs_n, const double (*fc)[4], uint32_t fs_nconst, uint32_t fs_const_base, uint32_t fsmask)
{
    g2r_prog *p = (g2r_prog *)calloc(1, sizeof *p);
    uint32_t va = (r->prog_a_used + 31) & ~31u, fa = (r->prog_b_used + 31) & ~31u, i;
    if (!p) return NULL;
    if (va + vs_n * 8 > PROG_A_BYTES || fa + fs_n * 8 > PROG_B_BYTES) { free(p); return NULL; }
    g2_write_buffer(r->bd, r->prog_a, va, vs, (size_t)vs_n * 8);
    g2_write_buffer(r->bd, r->prog_b, fa, fs, (size_t)fs_n * 8);
    r->prog_a_used = va + vs_n * 8;
    r->prog_b_used = fa + fs_n * 8;
    p->id = r->next_prog++;
    p->vs_off = va; p->vs_n = vs_n; p->fs_off = fa; p->fs_n = fs_n; p->fsmask = fsmask;
    p->vs_const_base = vs_const_base; p->vs_nconst = vs_nconst; p->fs_const_base = fs_const_base; p->fs_nconst = fs_nconst;
    p->vs_const = const_bits(vc, vs_nconst);
    p->fs_const = const_bits(fc, fs_nconst);
    for (i = 0; i < fs_n; i++) {                                /* the opcode, word [63:57]: 29 DISCARD, 21 STOZ in a fragment program */
        uint32_t op = (uint32_t)(fs[i] >> 57) & 0x7F;
        if (op == 29) p->fs_discard = 1;
        if (op == 21) p->fs_depth = 1;
    }
    return p;
}

/* ---------------------------------------------------------------- frames in flight */
static void wait_one(g2r *r)
{
    g2_event ev;
    LARGE_INTEGER f, a, b;
    if (!r->npending) return;
    /* long enough that an ordinary frame sees no FENCE, which would end its overlap with the next: a FENCE waits for all of A's work */
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&a);
    if (g2_wait_done(r->bd, r->pending[0], &ev, 0.1 + 2.0 * r->wait_s, 5.0) == 0) {
        QueryPerformanceCounter(&b);
        r->wait_s = (double)(b.QuadPart - a.QuadPart) / (double)f.QuadPart;
    }
    r->frame_done = r->frame_pending[0];
    r->pending[0] = r->pending[1];
    r->ring_at_frame[0] = r->ring_at_frame[1];
    r->frame_pending[0] = r->frame_pending[1];
    r->npending--;
}

uint16_t g2r_stream_handle(g2r *r) { return r->ring; }
uint32_t g2r_stream_gen(g2r *r) { return r->ring_gen; }

uint32_t g2r_stream(g2r *r, const void *data, uint32_t bytes, uint32_t align)
{
    uint32_t a = (r->ring_used + align - 1) & ~(align - 1);
    if (a + bytes > RING_BYTES) {                               /* wrap: nothing in flight may still read the start of the ring */
        while (r->npending) wait_one(r);
        a = 0;
        r->ring_gen++;
    }
    g2_write_buffer(r->bd, r->ring, a, data, bytes);
    r->ring_used = a + bytes;
    r->st.stream_bytes += bytes;
    return a;
}

void g2r_gather(g2r *r, g2r_vsrc *in, int n, uint32_t lo, uint32_t span)
{
    struct seg { const uint8_t *a, *b; uint32_t off; int stride; } seg[32];
    int ns = 0, i, k, merged = 1;
    for (i = 0; i < n; i++) {
        g2r_vsrc *s = &in[i];
        if (!s->base) continue;
        seg[ns].a = s->base + (size_t)lo * s->stride;
        seg[ns].b = seg[ns].a + (size_t)(span - 1) * s->stride + s->bytes;
        seg[ns].stride = s->stride;
        ns++;
    }
    while (merged) {
        merged = 0;
        for (i = 0; i < ns && !merged; i++) {
            for (k = i + 1; k < ns; k++) {
                if (seg[i].stride == seg[k].stride && seg[i].a < seg[k].b && seg[k].a < seg[i].b) {
                    if (seg[k].a < seg[i].a) seg[i].a = seg[k].a;
                    if (seg[k].b > seg[i].b) seg[i].b = seg[k].b;
                    seg[k] = seg[--ns];
                    merged = 1;
                    break;
                }
            }
        }
    }
    for (i = 0; i < ns; i++) seg[i].off = g2r_stream(r, seg[i].a, (uint32_t)(seg[i].b - seg[i].a), 4);
    for (i = 0; i < n; i++) {
        g2r_vsrc *s = &in[i];
        const uint8_t *e;
        if (!s->base) continue;
        e = s->base + (size_t)lo * s->stride;
        for (k = 0; k < ns; k++) {
            if (e >= seg[k].a && e < seg[k].b && seg[k].stride == s->stride) { s->off = seg[k].off + (uint32_t)(e - seg[k].a); break; }
        }
    }
}

/* ---------------------------------------------------------------- buffers kept on board A */
static void keep_reclaim(g2r *r)
{
    int i = 0;
    while (i < r->ndead) {
        if (r->dead[i].frame <= r->frame_done) { g2_heap_free(r->keep_heap, r->dead[i].off); r->dead[i] = r->dead[--r->ndead]; }
        else i++;
    }
}

static void keep_drop(g2r *r, g2r_kept *k)
{
    if (r->ndead == MAX_DEAD) {                                 /* a game dropping faster than frames finish */
        while (r->npending) wait_one(r);
        keep_reclaim(r);
    }
    r->dead[r->ndead].off = k->off;
    r->dead[r->ndead++].frame = k->last;
    *k->pprev = k->next;
    if (k->next) k->next->pprev = k->pprev;
    k->off = 0;
    free(k->sent);
    k->sent = NULL;
    r->st.kept_now--;
}

static int keep_open(g2r *r)
{
    uint32_t base;
    if (r->keep_state) return r->keep_state > 0;
    r->keep_state = -1;
    r->keep_room = (uint32_t)g2e_get_int("GPU2_KEEP_MB", 256) << 20;
    if (!r->keep_room) return 0;
    base = g2_heap_alloc(r->heap_a, r->keep_room, 4096);
    r->keep_heap = g2_heap_create(4096, r->keep_room);
    if (!base || !r->keep_heap || g2_handle_alloc(&r->h_buf_a, G2_HK_BUF_A, &r->keep) < 0) return 0;
    g2_create_buffer(r->bd, r->keep, base, r->keep_room, 0);
    r->keep_state = 1;
    return 1;
}

uint16_t g2r_kept_handle(g2r *r) { return r->keep; }

int g2r_keep(g2r *r, g2r_kept *k, const void *mem, uint32_t bytes)
{
    uint32_t now = r->frame + 1, off;
    uint32_t need = KEEP_FRAMES << (k->drops < KEEP_DROPS_MAX ? k->drops : KEEP_DROPS_MAX);
    uint8_t *sent = NULL;
    if (k->off) { k->last = now; r->st.kept_binds++; return 1; }
    if (k->last == now) return 0;                               /* asked once a frame */
    k->last = now;
    if (++k->stable <= need || !bytes || !keep_open(r) || bytes > r->keep_room / 2) return 0;
    if (k->watch && !(sent = (uint8_t *)malloc(bytes))) return 0;
    keep_reclaim(r);
    while (!(off = g2_heap_alloc(r->keep_heap, bytes, 4096))) { /* full: the least recently drawn goes, if no frame in flight still reads it */
        g2r_kept *v = NULL, *e;
        for (e = r->keep_list; e; e = e->next) if (e->last <= r->frame_done && (!v || e->last < v->last)) v = e;
        if (!v) { free(sent); return 0; }
        keep_drop(r, v);
        keep_reclaim(r);
    }
    if (sent) memcpy(sent, mem, bytes);
    g2_write_buffer(r->bd, r->keep, off, mem, bytes);
    k->off = off;
    k->bytes = bytes;
    k->sent = sent;
    k->next = r->keep_list;
    k->pprev = &r->keep_list;
    if (k->next) k->next->pprev = &k->next;
    r->keep_list = k;
    r->st.keep_bytes += bytes;
    r->st.kept_now++;
    r->st.kept_binds++;
    return 1;
}

void g2r_keep_changed(g2r *r, g2r_kept *k)
{
    k->stable = 0;
    if (!k->off) return;
    keep_drop(r, k);
    if (k->drops <= KEEP_DROPS_MAX) k->drops++;
}

int g2r_kept_same(g2r *r, g2r_kept *k, const void *mem, uint32_t off, uint32_t n)
{
    if (off >= k->bytes) return 1;
    if (n > k->bytes - off) n = k->bytes - off;
    if (!memcmp((const uint8_t *)mem + off, k->sent + off, n)) return 1;
    if (!r->st.kept_unseen++) g2e_log("g2r: a kept buffer of %u bytes changed with no Lock", k->bytes);
    g2r_keep_changed(r, k);
    return 0;
}

/* ---------------------------------------------------------------- passes */
static void pass_open(g2r *r)
{
    if (r->pass_open) return;
    g2_batch_begin(r->bd);
    g2_frame_begin(r->bd);
    r->pass_open = 1;
    r->pass_draws = 0;
}

/* board A's TRI items only in bands where the primitive sends a quad (VS_STATE word 4 bit 29; the last VS_STATE before a FLUSH decides
   for its frame, so every VS_STATE carries it). On by default; GPU2_LAZY_TRI=0 turns it off for a comparison */
static uint32_t lazy_tri(void)
{
    static int on = -1;
    if (on < 0) on = g2e_get_int("GPU2_LAZY_TRI", 1);
    return on ? G2_ZC_LAZY : 0;
}

/* board A's coarse depth: VS_STATE word 4 [28:25], a promise about the draw that B's DRAW_STATE (h0: depth test [0], write [1], function
 * [4:2] NEVER..ALWAYS, stencil [5], polygon offset [9]) and the fragment program keep — a wrong bit culls visible quads
 * (host/fb/fbproto.py zc_flags is the same rule).
 *   OCC 1     every pixel it covers ends with a depth <= its own: LESS / LEQUAL with writes, no stencil, DISCARD, gl_FragDepth or polygon
 *             offset (a scissor box is fine: board A's rectangle is the same box, so its coverage never leaves it)
 *   CULL 2    its fragments that fail the depth test change nothing: LESS / LEQUAL / EQUAL, no stencil, gl_FragDepth or polygon offset
 *   STRICT 4  LEQUAL / EQUAL: A culls only quads strictly behind
 *   INV 8     its depth writes may raise the stored depth: writes with GREATER / NOTEQUAL / GEQUAL / ALWAYS (the depth clears by a quad
 *             too), or with the test off
 * On by default; GPU2_COARSE_Z=0 sends none (A then culls nothing). */
static uint32_t zc_flags(const uint32_t *h, const g2r_prog *p)
{
    static int on = -1;
    uint32_t func = h[0] >> 2 & 7, z = 0;
    int test = (h[0] & 1u) != 0, write = (h[0] & 2u) != 0, stencil = (h[0] & 32u) != 0, offset = (h[0] & 512u) != 0;
    if (on < 0) on = g2e_get_int("GPU2_COARSE_Z", 1);
    if (!on) return 0;
    if (write && (!test || func >= 4)) return G2_ZC_INV;
    if (!test) return 0;
    if ((func == 1 || func == 3) && write && !(stencil || p->fs_discard || p->fs_depth || offset)) z |= G2_ZC_OCC;
    if (func >= 1 && func <= 3 && !(stencil || p->fs_depth || offset)) z |= G2_ZC_CULL;
    if (z && (func == 2 || func == 3)) z |= G2_ZC_STRICT;
    return z;
}

/* board A leaves out the bands with nothing to draw that the pass's clear does not reach (FLUSH flags bit 10): board B neither loads nor
   writes them, so a pass that draws into a few bands of the target costs those bands. Used when the last STATUS shows a board A that does it
   (layout 13) and a board B that preloads the band a BAND item names (its flags bit 13). On by default; GPU2_SKIP_BANDS=0 turns it off for a
   comparison */
static uint32_t skip_bands(g2r *r)
{
    if (r->skip_fl < 0) {
        uint32_t st[G2_ST_WORDS];
        int on = g2e_get_int("GPU2_SKIP_BANDS", 1);
        if (g2_last_status(r->bd, st) < 0) return 0;              /* no STATUS yet: decided at a later pass */
        r->skip_fl = (on && st[G2_ST_LAYOUT] >= 13 && G2_B_FLAG_BAND_NEXT(st[G2_ST_B_FLAGS])) ? G2_FL_SKIP_BANDS : 0;
        g2e_log("render: empty bands %s", r->skip_fl ? "left out (FLUSH flags bit 10)" : on ? "walked (the boards do not leave them out)" : "walked (GPU2_SKIP_BANDS=0)");
    }
    return (uint32_t)r->skip_fl;
}

/* A learns the target's size from VS_STATE and ends a frame's bands by it, and draws are what send VS_STATE: a pass that is only a clear would cover the last target's rows, past the end of a smaller image */
static void send_target_size(g2r *r)
{
    double vp[4];
    if (r->sh_vs_ok && r->sh_w == r->tgt.width && r->sh_h == r->tgt.height) return;
    vp[0] = r->tgt.width * 8.0; vp[1] = r->tgt.width * 8.0;
    vp[2] = r->tgt.height * 8.0; vp[3] = r->tgt.height * 8.0;
    g2_vs_state3(r->bd, vp, 0, 0, 0, r->tgt.width, r->tgt.height, 0, 1.0, 0.0, 1.0, 0, lazy_tri(), NULL);
    memcpy(r->sh_vp, vp, sizeof vp);
    r->sh_dn = 0.0; r->sh_df = 1.0; r->sh_z01 = 0; r->sh_cull = 0; r->sh_cw = 0; r->sh_lw = 1.0;
    r->sh_fsmask = 0; r->sh_w = r->tgt.width; r->sh_h = r->tgt.height; r->sh_id = 0; r->sh_zc = lazy_tri();
    r->sh_vs_ok = 1;
}

void g2r_end_pass(g2r *r)
{
    if (!r->pass_open) {
        if (!r->cl_flags) return;
        pass_open(r);
    }
    send_target_size(r);
    {
        uint32_t none[4] = { 0, 0, 0, 0 };
        uint32_t fl;
        if (r->discard_image == r->tgt.color) {
            if (!(r->cl_flags & (G2_FL_CLEAR_COLOR | G2_FL_CLEAR_SCISSOR))) {
                r->cl_flags |= G2_FL_CLEAR_COLOR | G2_FL_CMASK(0xF);
                memset(r->cl_c16, 0, sizeof r->cl_c16);
            }
            r->discard_image = 0;
        }
        /* a whole-image depth clear keeps the stencil by clearing it to what it already is: no band load */
        if ((r->cl_flags & G2_FL_CLEAR_DEPTH) && !(r->cl_flags & (G2_FL_CLEAR_STENCIL | G2_FL_CLEAR_SCISSOR)) && r->stencil_known && r->stencil_ds == r->tgt.ds) {
            r->cl_flags |= G2_FL_CLEAR_STENCIL;
            r->cl_stencil = r->stencil_value;
            r->cl_smask = 0xFF;
        }
        fl = r->cl_flags | (r->tgt.depth16 ? G2_FL_DEPTH16 : 0) | skip_bands(r);
        g2_flush(r->bd, r->tgt.color, fl, r->cl_c16, r->cl_depth, r->cl_stencil, (r->cl_flags & G2_FL_CLEAR_SCISSOR) ? r->cl_sc : none, r->tgt.ds, r->cl_smask);
    }
    g2_batch_end(r->bd);
    r->pass_open = 0;
    r->pass_draws = 0;
    r->cl_flags = 0;
    r->st.passes++;
    {
        int i;
        for (i = 0; i < r->npass_free; i++) g2_heap_free(r->heap_b, r->pass_free[i]);
        r->npass_free = 0;
        if (r->nquar >= QUAR_MAX / 2) quar_release(r);
    }
}

uint32_t g2r_pass_id(g2r *r) { return (uint32_t)r->st.passes; }

void g2r_finish(g2r *r)
{
    g2r_end_pass(r);
    g2_fence(r->bd, 30.0);
}

void g2r_set_target(g2r *r, const g2r_target *t)
{
    if (r->have_tgt && !memcmp(&r->tgt, t, sizeof *t)) return;
    g2r_end_pass(r);
    r->tgt = *t;
    r->have_tgt = 1;
}

void g2r_discard_color(g2r *r, uint16_t image) { r->discard_image = image; }
void g2r_stencil_undefined(g2r *r, uint16_t ds) { r->stencil_ds = ds; r->stencil_known = 1; r->stencil_value = 0; }
void g2r_stencil_written(g2r *r, uint16_t ds) { if (r->stencil_ds == ds) r->stencil_known = 0; }

/* Board A clips x and y only at its guard band, 2 040 pixels out, so what the view volume leaves out is still drawn if it lands in the target. What clipping to the volume leaves of a triangle is the pixels
 * with their centres in the viewport: the scissor takes the rest. A line wider than a pixel loses what would stand out of the viewport beside it, which GL leaves to it */
void g2r_scissor_viewport(g2r_state *s, uint32_t width, uint32_t height)
{
    uint32_t *h = s->h;
    int x0 = (int)ceil(s->vp_x - 0.5), y0 = (int)ceil(s->vp_y - 0.5), x1 = (int)ceil(s->vp_x + s->vp_w - 0.5), y1 = (int)ceil(s->vp_y + s->vp_h - 0.5), b[4];
    if (x0 <= 0 && y0 <= 0 && x1 >= (int)width && y1 >= (int)height) return;
    b[0] = 0; b[1] = 0; b[2] = (int)width; b[3] = (int)height;
    if (h[0] & 256) { b[0] = (int)(h[7] & 0xFFFF); b[1] = (int)(h[7] >> 16); b[2] = (int)(h[8] & 0xFFFF); b[3] = (int)(h[8] >> 16); }
    if (x0 > b[0]) b[0] = x0;
    if (y0 > b[1]) b[1] = y0;
    if (x1 < b[2]) b[2] = x1;
    if (y1 < b[3]) b[3] = y1;
    if (b[2] < b[0]) b[2] = b[0];
    if (b[3] < b[1]) b[3] = b[1];
    h[0] |= 256;
    h[7] = (uint32_t)b[0] | (uint32_t)b[1] << 16;
    h[8] = (uint32_t)b[2] | (uint32_t)b[3] << 16;
}

/* depth / stencil cleared by a quad inside the open pass: ALWAYS, colour off, the clear's rectangle. The same pixels as a FLUSH's clear, without ending the pass (Quake 3 clears depth for every HUD icon it
   draws as a small 3D scene, and each FLUSH reloads and rewrites the whole target).
   The quad covers the clear's rectangle and no more: its viewport IS that rectangle. Board B applies the scissor behind the fragment core (b_render sc_mask) and a board A without the draw
   rectangle walks the whole viewport, so a quad over the whole target would cost a full screen of walked, sent and shaded quads for every HUD icon. The corners are whole pixels, so the pixels are the scissor's */
/* The colour too. Board B stores a cleared pixel and an unblended fragment by one formula, floor((c16 + t) / 16)
   with t the dither threshold of the pixel or 0 (b_rop dclear, pixel_blend_b stage E), c16 the colour in channel units, ONE = 16 (2^n - 1) for n bits. The clear's c16 comes from here; a fragment's is
   round(f ONE) of the program's output f, an exact product rounded (b_render cn_a / c16_b1 / c16_b2). So the quad's program outputs f = c16 / ONE as FP24: the error of that, under c16 2^-17 < 0.04,
   rounds away and the fragment's c16 is the clear's — checked for every c16 of every channel width (1, 4, 5, 6, 8 bits). The clear's colour mask and dither flag are the quad's state */
static void clear_quad(g2r *r, uint32_t flags, const uint32_t c16[4], uint32_t depth, uint32_t stencil, uint32_t smask, const uint32_t sc[4])
{
    g2r_state s;
    float z = depth ? 1.0f : -1.0f, v[16];
    int i, de = (flags & G2_FL_CLEAR_DEPTH) != 0, se = (flags & G2_FL_CLEAR_STENCIL) != 0, ce = (flags & G2_FL_CLEAR_COLOR) != 0;
    const int bits[4] = { r->tgt.bits_r, r->tgt.bits_g, r->tgt.bits_b, r->tgt.bits_a };
    uint32_t fu[4] = { 0, 0, 0, 0 };
    uint32_t x0 = (flags & G2_FL_CLEAR_SCISSOR) ? sc[0] : 0, y0 = (flags & G2_FL_CLEAR_SCISSOR) ? sc[1] : 0, x1 = (flags & G2_FL_CLEAR_SCISSOR) ? sc[2] : r->tgt.width, y1 = (flags & G2_FL_CLEAR_SCISSOR) ? sc[3] : r->tgt.height;
    if (x1 > r->tgt.width) x1 = r->tgt.width;
    if (y1 > r->tgt.height) y1 = r->tgt.height;
    if (x1 <= x0 || y1 <= y0) return;
    memset(&s, 0, sizeof s);
    s.prog = r->clear_prog;
    s.vp_x = x0; s.vp_y = y0; s.vp_w = x1 - x0; s.vp_h = y1 - y0;
    s.depth_far = 1.0;
    s.line_width = 1.0;
    s.h[0] = (de ? 1u | 2u : 0) | (uint32_t)G2_FN_ALWAYS << 2 | (se ? 32u : 0) | 256u;
    if (ce) {
        s.h[0] |= (flags >> 2 & 0xFu) << 16 | ((flags & G2_FL_DITHER) ? 128u : 0);     /* G2_FL_CMASK's four bits are the state's; no blend */
        for (i = 0; i < 4; i++) if (bits[i] > 0) fu[i] = g2_fp24((double)c16[i] / (16.0 * (double)((1u << bits[i]) - 1)));
    }
    s.fs_uniforms = fu;
    s.h[1] = s.h[2] = (stencil & 255) | 255u << 8 | (smask & 255) << 16 | (uint32_t)G2_FN_ALWAYS << 24;
    s.h[3] = 2u | 2u << 3 | 2u << 6 | 2u << 16 | 2u << 19 | 2u << 22;   /* REPLACE */
    s.h[7] = x0 | y0 << 16;
    s.h[8] = x1 | y1 << 16;
    v[0] = -1; v[1] = -1; v[4] = 1; v[5] = -1; v[8] = -1; v[9] = 1; v[12] = 1; v[13] = 1;
    for (i = 0; i < 4; i++) { v[4 * i + 2] = z; v[4 * i + 3] = 1.0f; }
    for (i = 0; i < 16; i++) { s.attr[i].size = 4; s.attr[i].type = G2_AT_FP24; s.attr[i].constant[3] = g2_fp24(1.0); }
    s.attr[0].enabled = 1; s.attr[0].type = G2_AT_FLOAT; s.attr[0].handle = r->ring; s.attr[0].stride = 16;
    s.attr[0].offset = g2r_stream(r, v, sizeof v, 4);
    g2r_draw(r, &s, 1, 4, 0, 0, 0, 0, 0);
}

void g2r_clear(g2r *r, uint32_t flags, const double rgba[4], uint32_t depth, uint32_t stencil, uint32_t stencil_mask, const uint32_t sc[4])
{
    const uint32_t kinds = G2_FL_CLEAR_COLOR | G2_FL_CLEAR_DEPTH | G2_FL_CLEAR_STENCIL;
    uint32_t want = flags & kinds, have = r->cl_flags & kinds;
    int same_sc;
    if (!want) return;
    if ((flags & G2_FL_CLEAR_SCISSOR) && sc[0] == 0 && sc[1] == 0 && sc[2] >= r->tgt.width && sc[3] >= r->tgt.height) flags &= ~(uint32_t)G2_FL_CLEAR_SCISSOR;
    /* what the stencil holds from here on */
    if (want & G2_FL_CLEAR_STENCIL) {
        if (!(flags & G2_FL_CLEAR_SCISSOR) && (stencil_mask & 255) == 255) { r->stencil_ds = r->tgt.ds; r->stencil_known = 1; r->stencil_value = stencil & 255; }
        else if ((stencil_mask & 255) && !(r->stencil_known && (stencil & stencil_mask & 255) == (r->stencil_value & stencil_mask & 255))) r->stencil_known = 0;
    }
    /* depth values a quad reaches exactly: 0 and all ones — of a 16-bit depth too (quad_setup makes a fragment's depth round(z (2^n - 1)) for n = 16 as for 24, so z = 1 is 0xFFFF there) */
    /* A colour clear is a quad too when it is of a part of the target (a scissor box left after the line above): the whole target's colour clear still ends the pass — the next one then starts with the
       fast clear, where a quad would shade every pixel */
    if (r->pass_draws && (!(want & G2_FL_CLEAR_COLOR) || (flags & G2_FL_CLEAR_SCISSOR)) &&
        (!(want & G2_FL_CLEAR_DEPTH) || depth == 0 || depth == (r->tgt.depth16 ? 0xFFFFu : 0xFFFFFFu))) {
        uint32_t c16[4] = { 0, 0, 0, 0 };
        if (want & G2_FL_CLEAR_COLOR) {
            c16[0] = g2_c16(rgba[0], r->tgt.bits_r); c16[1] = g2_c16(rgba[1], r->tgt.bits_g); c16[2] = g2_c16(rgba[2], r->tgt.bits_b);
            c16[3] = r->tgt.bits_a ? g2_c16(rgba[3], r->tgt.bits_a) : 0;
        }
        clear_quad(r, flags, c16, depth, stencil, stencil_mask, sc);
        return;
    }
    /* clears merge into the pass while it has no draw, when the two agree on the rectangle and the mask of what they share */
    same_sc = (r->cl_flags & G2_FL_CLEAR_SCISSOR) == (flags & G2_FL_CLEAR_SCISSOR) &&
              (!(flags & G2_FL_CLEAR_SCISSOR) || !memcmp(r->cl_sc, sc, sizeof r->cl_sc));
    if (r->pass_draws || (have && (!same_sc || (have & want)))) g2r_end_pass(r);
    have = r->cl_flags & kinds;
    if (want & G2_FL_CLEAR_COLOR) {
        r->cl_c16[0] = g2_c16(rgba[0], r->tgt.bits_r);
        r->cl_c16[1] = g2_c16(rgba[1], r->tgt.bits_g);
        r->cl_c16[2] = g2_c16(rgba[2], r->tgt.bits_b);
        r->cl_c16[3] = r->tgt.bits_a ? g2_c16(rgba[3], r->tgt.bits_a) : 0;
    }
    if (want & G2_FL_CLEAR_DEPTH) r->cl_depth = depth;
    if (want & G2_FL_CLEAR_STENCIL) { r->cl_stencil = stencil; r->cl_smask = stencil_mask; }
    r->cl_flags |= flags;
    if (flags & G2_FL_CLEAR_SCISSOR) memcpy(r->cl_sc, sc, sizeof r->cl_sc);
}

/* ---------------------------------------------------------------- draws */
static uint32_t draw_id_of(g2r *r, const g2r_state *s, const uint32_t *h, const uint32_t *uni, uint32_t nuni)
{
    /* the generation goes through the hash as data. XORed into the seed it cancels against h0's low byte, the first thing hashed: a generation 3 later made h0 ...84 and ...87 (depth off / on) the same key, and a draw
       silently took another draw's id and state */
    uint64_t hash = fnv(fnv(fnv(0xCBF29CE484222325ull, &r->id_gen, sizeof r->id_gen), h, 23 * 4), uni, (size_t)nuni * 16);
    uint32_t i = (uint32_t)hash & (ID_SLOTS - 1), n;
    (void)s;
    if (!hash) hash = 1;
    for (n = 0; n < ID_SLOTS; n++) {
        id_entry *e = &r->ids[i];
        if (e->hash == hash) return e->id;
        if (!e->hash) break;
        i = (i + 1) & (ID_SLOTS - 1);
    }
    if (r->next_id >= MAX_IDS) {                                /* all ids taken: this pass may still need them, end it first */
        g2r_end_pass(r);
        pass_open(r);
        memset(r->ids, 0, sizeof r->ids);
        r->next_id = 1;
        i = (uint32_t)hash & (ID_SLOTS - 1);
    }
    r->ids[i].hash = hash;
    r->ids[i].id = r->next_id++;
    g2_draw_state(r->bd, r->ids[i].id, h, uni, nuni);
    r->st.draw_states++;
    return r->ids[i].id;
}

static void send_uniforms(g2r *r, const uint32_t *u, uint32_t n)
{
    uint32_t i = 0;
    while (i < n) {                                             /* runs of changed vec4s */
        uint32_t k;
        if (r->sh_uni_ok[i] && !memcmp(r->sh_uni[i], u + 4 * i, 16)) { i++; continue; }
        k = i;
        while (k < n && !(r->sh_uni_ok[k] && !memcmp(r->sh_uni[k], u + 4 * k, 16)) && k - i < 80) {
            memcpy(r->sh_uni[k], u + 4 * k, 16);
            r->sh_uni_ok[k] = 1;
            k++;
        }
        g2_vs_uniforms(r->bd, i, u + 4 * i, k - i);
        i = k;
    }
}

void g2r_draw(g2r *r, const g2r_state *s, int mode, uint32_t count, uint32_t first, int indexed, uint16_t ihandle, uint32_t ioffset, int index_u8)
{
    const g2r_prog *p = s->prog;
    if ((s->h[0] & 32u) && (s->h[1] & 0xFF0000u || s->h[2] & 0xFF0000u) && s->prog != r->clear_prog) r->stencil_known = 0;  /* a draw may write the stencil */
    uint32_t h[23], id, i;
    uint32_t vu[4 * 256], fu[4 * 64];
    uint32_t nv = p->vs_const_base + p->vs_nconst, nf = p->fs_const_base + p->fs_nconst;
    pass_open(r);
    /* A: the program, attributes, uniforms */
    if (r->cur_prog != p) {
        g2_vs_program(r->bd, r->prog_a, p->vs_off, p->vs_n);
        r->cur_prog = p;
    }
    if (!r->sh_attr_ok) {
        for (i = 0; i < 16; i++) {
            g2_vertex_attr(r->bd, (int)i, s->attr[i].enabled, s->attr[i].size, s->attr[i].handle, s->attr[i].stride, s->attr[i].offset, s->attr[i].constant, s->attr[i].type, s->attr[i].normalized);
            r->sh_attr[i] = s->attr[i];
        }
        r->sh_attr_ok = 1;
    } else {
        for (i = 0; i < 16; i++) {
            if (memcmp(&r->sh_attr[i], &s->attr[i], sizeof s->attr[i])) {
                g2_vertex_attr(r->bd, (int)i, s->attr[i].enabled, s->attr[i].size, s->attr[i].handle, s->attr[i].stride, s->attr[i].offset, s->attr[i].constant, s->attr[i].type, s->attr[i].normalized);
                r->sh_attr[i] = s->attr[i];
            }
        }
    }
    if (p->vs_const_base) memcpy(vu, s->vs_uniforms, (size_t)p->vs_const_base * 16);
    memcpy(vu + 4 * p->vs_const_base, p->vs_const, (size_t)p->vs_nconst * 16);
    send_uniforms(r, vu, nv);
    /* B: the draw's state, by id */
    memcpy(h, s->h, sizeof h);
    h[11] = p->fs_n << 16 | r->prog_b;
    h[12] = p->fs_off;
    h[13] = nf;
    if (p->fs_const_base) memcpy(fu, s->fs_uniforms, (size_t)p->fs_const_base * 16);
    memcpy(fu + 4 * p->fs_const_base, p->fs_const, (size_t)p->fs_nconst * 16);
    id = draw_id_of(r, s, h, fu, nf);
    {
        static int trace = -1;
        if (trace < 0) trace = g2e_get_int("GPU2_TRACE_DRAW", 0);
        if (trace) g2e_log("draw %llu pass %u: id %u prog %u fs_off %u h0 %08x sc %08x-%08x mode %d n %u vs_off %u", (unsigned long long)r->st.draws, (uint32_t)r->st.passes, id, p->id, p->fs_off, h[0], h[7], h[8], mode, count, p->vs_off);
    }
    /* A's state: viewport, cull, the draw id, the coarse-depth flags (from the very state B gets under this id) and the lazy TRI bit */
    {
        double vp[4];
        uint32_t zc = zc_flags(h, p) | lazy_tri();
        vp[0] = s->vp_w * 8.0; vp[1] = (s->vp_x + s->vp_w / 2) * 16.0;
        vp[2] = s->vp_h * 8.0; vp[3] = (s->vp_y + s->vp_h / 2) * 16.0;
        if (!r->sh_vs_ok || memcmp(r->sh_vp, vp, sizeof vp) || r->sh_dn != s->depth_near || r->sh_df != s->depth_far || r->sh_z01 != s->z01 || r->sh_cull != s->cull ||
            r->sh_cw != s->front_cw || r->sh_lw != s->line_width || r->sh_fsmask != p->fsmask || r->sh_w != r->tgt.width || r->sh_h != r->tgt.height || r->sh_id != id ||
            r->sh_zc != zc) {
            /* the draw's rectangle for board A: the scissor box of the state B gets under this id, when its scissor test is on — the same words, so the two boards can never
               disagree; a change of the box is a change of the id, which sends this again */
            uint32_t rect[4] = { h[7] & 0xFFFFu, h[7] >> 16, h[8] & 0xFFFFu, h[8] >> 16 };
            g2_vs_state3(r->bd, vp, s->cull, s->front_cw, p->fsmask, r->tgt.width, r->tgt.height, id, s->line_width, s->depth_near, s->depth_far, s->z01, zc,
                         (h[0] & 256u) ? rect : NULL);
            memcpy(r->sh_vp, vp, sizeof vp);
            r->sh_dn = s->depth_near; r->sh_df = s->depth_far; r->sh_z01 = s->z01; r->sh_cull = s->cull; r->sh_cw = s->front_cw; r->sh_lw = s->line_width;
            r->sh_fsmask = p->fsmask; r->sh_w = r->tgt.width; r->sh_h = r->tgt.height; r->sh_id = id; r->sh_zc = zc;
            r->sh_vs_ok = 1;
        }
    }
    g2_draw(r->bd, mode, count, first, indexed, ihandle, ioffset, index_u8);
    r->pass_draws++;
    r->st.draws++;
}

/* GPU2_FRAMES_AHEAD: unfinished frames before Present waits. A game reads its input as a frame starts, so every frame queued here is a frame of lag: harmless at 60 fps,
   a fifth of a second at the 10-20 fps real games run at. One keeps A binning the next frame while B renders this one, which is where the two boards' parallelism comes from. */
void g2r_present(g2r *r, uint16_t image, int hold)
{
    static int ahead = -1;
    if (ahead < 0) {
        ahead = g2e_get_int("GPU2_FRAMES_AHEAD", 1);
        if (ahead < 1) ahead = 1;
        if (ahead > 2) ahead = 2;                               /* pending[] */
    }
    g2r_end_pass(r);
    while (r->npending >= ahead) wait_one(r);
    r->ring_at_frame[r->npending] = r->ring_used;
    r->frame_pending[r->npending] = r->frame + 1;
    r->pending[r->npending++] = g2_present(r->bd, image, ++r->frame, 0, hold);
    r->st.frames++;
}

void g2r_present_idle(g2r *r)
{
    g2r_end_pass(r);
    while (r->npending) wait_one(r);
}

void g2r_get_stats(g2r *r, g2r_stats *s) { *s = r->st; }

void g2r_stats_tick(g2r *r)
{
    static int period = -1;
    static DWORD t0;
    static g2r_stats s0;
    static g2_counters c0;
    static uint32_t st0[G2_ST_WORDS];
    DWORD now = GetTickCount();
    if (period < 0) period = g2e_get_int("GPU2_STATS", 0);
    if (period <= 0) return;
    if (t0 && now - t0 >= (DWORD)period * 1000) {
        uint32_t st[G2_ST_WORDS];
        g2_counters c;
        double dt = (now - t0) / 1000.0, n = (double)(r->st.frames - s0.frames);
        g2_get_counters(r->bd, &c);
        if (n > 0 && g2_status(r->bd, st, NULL) == 0) {
            /* the boards' cycle counters in ms: both boards run clk_sys at GPU2_CLK_MHZ (default 120) */
            static double ckhz = -1;
            double bc = st[G2_ST_B_CYC_FRAME], f;
            if (ckhz < 0) ckhz = g2e_get_int("GPU2_CLK_MHZ", 120) * 1e3;
            f = bc / ckhz;
#define BPCT(w) (bc > 0 ? 100.0 * st[w] / bc : 0.0)
            g2e_log("stats: %.1f fps | per frame: %.0f draws, %.0f new draw states, %.1f passes, %.1f renamed textures, %.1f image entries reloaded, %.0f KB vertices, %.0f KB to keep, %.0f kept binds, %.0f datagrams | %u buffers kept, %llu changed with no Lock | EVENTs lost %llu, resent %llu, A gap %u | B last frame %.2f ms: flush %.0f%% band %.0f%% draw %.0f%% FS %.0f%% "
                    "setup %.0f%% starved %.0f%% ROP %.0f%%, %u quads, %u tris | A per frame: geometry %.2f ms, walk %.2f ms, %.0f visits", n / dt, (double)(r->st.draws - s0.draws) / n, (double)(r->st.draw_states - s0.draw_states) / n,
                    (double)(r->st.passes - s0.passes) / n, (double)(r->st.renames - s0.renames) / n, (double)(c.img_evictions - c0.img_evictions) / n, (double)(r->st.stream_bytes - s0.stream_bytes) / n / 1024.0, (double)(r->st.keep_bytes - s0.keep_bytes) / n / 1024.0,
                    (double)(r->st.kept_binds - s0.kept_binds) / n, (double)(c.dgrams - c0.dgrams) / n, r->st.kept_now, (unsigned long long)r->st.kept_unseen, (unsigned long long)(c.n_lost_events - c0.n_lost_events), (unsigned long long)(c.n_resent - c0.n_resent),
                    st[G2_ST_GAP] - st0[G2_ST_GAP], f, BPCT(G2_ST_B_CYC_FLUSH), BPCT(G2_ST_B_CYC_BAND), BPCT(G2_ST_B_CYC_DRAW), BPCT(G2_ST_B_CYC_FS),
                    BPCT(G2_ST_B_CYC_BANDLOAD), BPCT(G2_ST_B_CYC_STARVED), BPCT(G2_ST_B_CYC_ROPSTALL), st[G2_ST_RENDER_QUADS], st[G2_ST_GEOM_BUSY_TRIS] & 0xFFFFF,
                    (double)(st[G2_ST_A_CYC_GEOM] - st0[G2_ST_A_CYC_GEOM]) / ckhz / n, (double)(st[G2_ST_A_CYC_RENDER] - st0[G2_ST_A_CYC_RENDER]) / ckhz / n,
                    (double)(st[G2_ST_A_VISITS] - st0[G2_ST_A_VISITS]) / n);
#undef BPCT
            /* STATUS layout 10: the geometry side's profile, each stage's share of the geometry-busy cycles (the
               stages overlap; the shares do not add up), the loads / draws / primitives, the dispatcher's idle and wait */
            if (st[G2_ST_LAYOUT] >= 10 && st[G2_ST_A_CYC_GEOM] != st0[G2_ST_A_CYC_GEOM]) {
                double g = (double)(st[G2_ST_A_CYC_GEOM] - st0[G2_ST_A_CYC_GEOM]);
                uint32_t ld = st[G2_ST_A_PROF_LOADS_DRAWS] - st0[G2_ST_A_PROF_LOADS_DRAWS];
#define APCT(w) (100.0 * (double)(st[w] - st0[w]) / g)
                g2e_log("  A geometry %.0f kcycles/frame: load %.0f%% copy %.0f%% feed %.0f%% fetch-wait %.0f%% core %.0f%% asm %.0f%% vo-stall %.0f%% setup %.0f%% push-stall %.0f%% bin %.0f%% clear %.0f%% | "
                        "per frame: loads %.1f draws %.1f prims %.0f | dispatcher idle %.2f ms, geometry wait %.2f ms",
                        g / 1e3 / n, APCT(G2_ST_A_PROF_LOAD), APCT(G2_ST_A_PROF_COPY), APCT(G2_ST_A_PROF_FEED), APCT(G2_ST_A_PROF_FETCH_WAIT), APCT(G2_ST_A_PROF_CORE),
                        APCT(G2_ST_A_PROF_ASM), APCT(G2_ST_A_PROF_VO_STALL), APCT(G2_ST_A_PROF_SETUP), APCT(G2_ST_A_PROF_PUSH_STALL), APCT(G2_ST_A_PROF_BIN), APCT(G2_ST_A_PROF_CLEAR),
                        (double)((ld >> 16) & 0xFFFF) / n, (double)(ld & 0xFFFF) / n, (double)(st[G2_ST_A_PROF_PRIMS] - st0[G2_ST_A_PROF_PRIMS]) / n,
                        (double)(st[G2_ST_A_DISP_IDLE] - st0[G2_ST_A_DISP_IDLE]) / ckhz / n, (double)(st[G2_ST_A_DISP_GWAIT] - st0[G2_ST_A_DISP_GWAIT]) / ckhz / n);
#undef APCT
            }
            /* STATUS layout 12: board A's coarse depth and TRI items per frame (the quads A walked = culled + sent) */
            if (st[G2_ST_LAYOUT] >= 12) {
#define AD(w) ((double)(st[w] - st0[w]) / n)
                g2e_log("  A per frame: %.0f quads walked, %.0f culled (%.1f%%), %.0f sent | TRI items %.0f sent, %.0f held back, %.0f sent with no quad | %.0f blocks lowered%s%s | %.1f bands left out",
                        AD(G2_ST_A_CZ_CULLED) + AD(G2_ST_A_CZ_QUADS_SENT), AD(G2_ST_A_CZ_CULLED),
                        AD(G2_ST_A_CZ_CULLED) + AD(G2_ST_A_CZ_QUADS_SENT) > 0 ? 100.0 * AD(G2_ST_A_CZ_CULLED) / (AD(G2_ST_A_CZ_CULLED) + AD(G2_ST_A_CZ_QUADS_SENT)) : 0.0,
                        AD(G2_ST_A_CZ_QUADS_SENT), AD(G2_ST_A_TRI_SENT), AD(G2_ST_A_TRI_HELD), AD(G2_ST_A_TRI_NOQUAD), AD(G2_ST_A_CZ_BLOCKS),
                        g2e_get_int("GPU2_COARSE_Z", 1) ? "" : " (GPU2_COARSE_Z=0)", lazy_tri() ? "" : " (GPU2_LAZY_TRI=0)",
                        st[G2_ST_LAYOUT] >= 13 ? AD(G2_ST_A_BANDS_SKIPPED) : 0.0);
#undef AD
            }
            memcpy(st0, st, sizeof st);
        }
        s0 = r->st; c0 = c; t0 = now;
    } else if (!t0) {
        t0 = now;
        s0 = r->st;
        g2_get_counters(r->bd, &c0);
        g2_status(r->bd, st0, NULL);
    }
}
