/* textures, pixel transfers, framebuffer objects and buffer objects.
 * Storage is B's format closest to what the internal format asks (gl_hw_format): RGB565, RGBA4444 or RGBA5551 where those hold every bit asked, else RGBA8. L / A / LA / I / RGB expand to
 * R G B A at upload and base_class keeps what the texture environment must do with them. A 16-bit format is packed on the PC and goes raw. Each texture gets its whole mip chain up front, so levels can arrive in any order.
 *
 * Ordering: board B runs WRITE_IMAGE when it arrives, but a pass's draws only render at its FLUSH. An update to a texture the open pass has used would reach those earlier draws, so it ends
 * the pass first (tex_touch).
 */
#include "gl_internal.h"

#define T_BUCKETS 1024
typedef struct tnode { gl_tex t; struct tnode *next; uint32_t used_pass; } tnode;
static tnode *g_nodes[T_BUCKETS];
static GLuint g_next_tex = 1, g_next_buf = 1, g_next_fbo = 1, g_next_rb = 1;
static tnode g_default[4];                              /* texture object 0 of GL_TEXTURE_2D, of the cube map target, of GL_TEXTURE_1D and of GL_TEXTURE_3D */

static void tex_defaults(gl_tex *t)
{
    t->min_filter = GL_NEAREST_MIPMAP_LINEAR;
    t->mag_filter = GL_LINEAR;
    t->wrap_s = t->wrap_t = t->wrap_r = GL_REPEAT;
    t->max_level = 1000;
    t->min_lod = -1000.0f; t->max_lod = 1000.0f;
    t->compare_mode = GL_NONE; t->compare_func = GL_LEQUAL; t->depth_mode = GL_LUMINANCE;
}

static tnode *node_of(GLuint name, int create)
{
    tnode *n = g_nodes[name % T_BUCKETS];
    for (; n; n = n->next) if (n->t.name == name) return n;
    if (!create) return NULL;
    n = (tnode *)calloc(1, sizeof *n);
    if (!n) return NULL;
    n->t.name = name;
    n->used_pass = 0xFFFFFFFFu;
    tex_defaults(&n->t);
    n->next = g_nodes[name % T_BUCKETS];
    g_nodes[name % T_BUCKETS] = n;
    return n;
}

gl_tex *gl_tex_lookup(GLuint name, GLenum target, int create)
{
    tnode *n;
    if (name == 0) {
        n = &g_default[target == GL_TEXTURE_CUBE_MAP ? 1 : target == GL_TEXTURE_1D ? 2 : target == GL_TEXTURE_3D ? 3 : 0];
        if (!n->t.mag_filter) { tex_defaults(&n->t); n->used_pass = 0xFFFFFFFFu; }
        return &n->t;
    }
    n = node_of(name, create);
    return n ? &n->t : NULL;
}

gl_tex *gl_unit_tex(int unit, int kind)
{
    const gl_unit *u = &C->unit[unit];
    return gl_tex_lookup(kind == 1 ? u->bound_cube : kind == 2 ? u->bound_1d : kind == 3 ? u->bound_3d : u->bound_2d, kind == 1 ? GL_TEXTURE_CUBE_MAP : kind == 2 ? GL_TEXTURE_1D : kind == 3 ? GL_TEXTURE_3D : GL_TEXTURE_2D, 1);
}

static tnode *node_from(gl_tex *t) { return (tnode *)t; }

#define TEX_USAGE (G2_USE_SAMPLED | G2_USE_RT | G2_USE_TRANSFER)

/* a texture whose image can be replaced by a copy under a new handle: nothing else holds the handle or the memory (a view, a border image, a render
   image, a depth alias, a framebuffer of the open pass), and board B copies its format */
static int can_rename(const gl_tex *t)
{
    return t->handle && !t->view && !t->bimg && !t->rt && !t->rt_ds && !t->ds_alias && t->target != GL_TEXTURE_3D && !t->vol3 && t->base_class != BF_DEPTH &&
           !g2r_is_target(D.r, t->handle);
}

/* a texture is about to change on the board. When the open pass has sampled it, its draws must keep the old texels: the texture moves to a copy
   of itself (copy-on-write, the old image is freed after the pass) — an application that updates a texture in the middle of a frame, such as
   lightmaps under a dynamic light, costs a copy on board B instead of a FLUSH and a reload of the whole target. Where that is not possible the pass
   ends first. */
void gl_tex_touch(gl_tex *t)
{
    gl_tex_resolve(t);
    static int cow = -1;                                        /* GPU2_TEX_COW=0: end the pass instead, for a comparison */
    if (cow < 0) cow = g2e_get_int("GPU2_TEX_COW", 1);
    if (node_from(t)->used_pass == g2r_pass_id(D.r)) {
        uint32_t mask = 0;
        int f, l;
        for (f = 0; f < t->layers && f < 6; f++) for (l = 0; l < t->levels && l < MAX_LEVELS; l++) if (t->level[f][l].defined) mask |= 1u << l;
        if (!cow || !can_rename(t) || !mask || g2r_image_rename(D.r, &t->handle, &t->base, (uint32_t)t->w, (uint32_t)t->h, t->hwfmt, (uint32_t)t->levels,
                                                          (uint32_t)t->layers, 1, TEX_USAGE, 0, mask) < 0) g2r_end_pass(D.r);
        node_from(t)->used_pass = 0xFFFFFFFFu;
    }
    t->rev++;
}

/* the image is about to be replaced or destroyed as a whole: a pass that sampled it ends first */
static void tex_touch_flush(gl_tex *t)
{
    gl_tex_resolve(t);
    if (node_from(t)->used_pass == g2r_pass_id(D.r)) g2r_end_pass(D.r);
    t->rev++;
}

/* ---- render images: B renders level 0 of an image of one layer, at a pitch that is a multiple of 32. Any other attachment renders in an image of its own, filled from the texture when a framebuffer takes it up and copied
 * back before anything else uses the texture */
static void rt_copy(gl_tex *t, int to_tex)
{
    uint32_t w = (uint32_t)t->rt_w, h = (uint32_t)t->rt_h, l = (uint32_t)t->rt_level, f = (uint32_t)t->rt_face;
    g2r_end_pass(D.r);
    if (t->target == GL_TEXTURE_3D) {
        if (to_tex) gl_tex3d_rt_store(t); else gl_tex3d_rt_load(t);
    } else if (to_tex) g2_copy_image(D.bd, t->rt, t->handle, 0, 0, w, h, 0, 0, w, h, 0, 0, 0, 0, 0, 0, 0, l, f);
    else g2_copy_image(D.bd, t->handle, t->rt, 0, 0, w, h, 0, 0, w, h, 0, 0, 0, 0, 0, l, f, 0, 0);
}

void gl_tex_resolve(gl_tex *t)
{
    if (!t->rt_dirty) return;
    t->rt_dirty = 0;
    rt_copy(t, 1);
    t->rt_rev = ++t->rev;
}

void gl_tex_rt_drop(gl_tex *t)
{
    if (!t->rt) return;
    g2r_end_pass(D.r);
    if (t->rt_ds) g2r_alias_destroy(D.r, t->rt_ds);
    g2r_image_destroy(D.r, t->rt, t->rt_base);
    t->rt = t->rt_ds = 0;
    t->rt_dirty = 0;
}

/* the image a framebuffer draws in; 0: no room */
static uint16_t rt_bind(gl_tex *t, int level, int face, int w, int h, int ds)
{
    int load = 0;
    if (t->rt && (t->rt_w != w || t->rt_h != h)) { gl_tex_resolve(t); gl_tex_rt_drop(t); }
    if (t->rt && (t->rt_level != level || t->rt_face != face)) { gl_tex_resolve(t); load = 1; }
    if (!t->rt) {
        /* a depth texture's: its D24S8 alias renders depth and stencil, at the rows board B needs */
        if ((ds ? g2r_image_create_ds_rows(D.r, &t->rt, &t->rt_base, (uint32_t)w, (uint32_t)h, t->hwfmt, 1, G2_USE_RT | G2_USE_TRANSFER)
                : g2r_image_create(D.r, &t->rt, &t->rt_base, (uint32_t)w, (uint32_t)h, t->hwfmt, 1, 1, 1, G2_USE_RT | G2_USE_TRANSFER)) < 0) { t->rt = 0; return 0; }
        t->rt_w = w; t->rt_h = h;
        load = 1;
    }
    t->rt_level = level; t->rt_face = face;
    if (load || t->rt_rev != t->rev) {
        rt_copy(t, 0);
        t->rt_rev = t->rev;
    }
    if (ds && !t->rt_ds && g2r_image_alias(D.r, &t->rt_ds, t->rt_base, (uint32_t)w, (uint32_t)h, G2_FMT_D24S8, g2r_ds_pitch((uint32_t)w), 1, 1, G2_USE_RT) < 0) { t->rt_ds = 0; return 0; }
    t->rt_dirty = 1;
    return t->rt;
}
#define tex_touch gl_tex_touch

/* ---------------------------------------------------------------- storage formats
 * §3.8.1: the GL chooses an internal representation that closely approximates the one asked for. B stores RGBA8, RGB565, RGBA4444 and RGBA5551, renders into all four and samples all four, so a
 * texture takes the 16-bit format that holds every bit its internal format asks, with these rules: RGB takes RGB565, whose alpha always reads 1 (also after rendering into it); RGBA takes
 * RGBA5551 for one bit of alpha, else RGBA4444; alpha, luminance and intensity need R, G and B alike and take RGBA4444. What asks for more, an unsized format with 8-bit data, compressed formats
 * and depth (the word of a D24S8 target) take RGBA8. An unsized format asks for what its client type has, so 5_6_5 data stays 5_6_5 */
enum { K_RGB, K_RGBA, K_A, K_L, K_LA, K_I, K_DEPTH };

static int base_class_of(GLenum internal);

static int base_kind(GLenum in)
{
    switch (in) {
    case GL_ALPHA: case GL_ALPHA4: case GL_ALPHA8: case GL_ALPHA12: case GL_ALPHA16: case GL_COMPRESSED_ALPHA: return K_A;
    case GL_LUMINANCE: case 1: case GL_LUMINANCE4: case GL_LUMINANCE8: case GL_LUMINANCE12: case GL_LUMINANCE16: case GL_COMPRESSED_LUMINANCE: return K_L;
    case GL_LUMINANCE_ALPHA: case 2: case GL_LUMINANCE4_ALPHA4: case GL_LUMINANCE6_ALPHA2: case GL_LUMINANCE8_ALPHA8: case GL_LUMINANCE12_ALPHA4: case GL_LUMINANCE12_ALPHA12: case GL_LUMINANCE16_ALPHA16:
    case GL_COMPRESSED_LUMINANCE_ALPHA: return K_LA;
    case GL_INTENSITY: case GL_INTENSITY4: case GL_INTENSITY8: case GL_INTENSITY12: case GL_INTENSITY16: case GL_COMPRESSED_INTENSITY: return K_I;
    default: break;
    }
    return base_class_of(in) == BF_DEPTH ? K_DEPTH : base_class_of(in) == BF_RGB ? K_RGB : K_RGBA;
}

uint32_t gl_hw_format(GLenum in, GLenum type)
{
    static const struct { GLenum f; uint8_t c[4]; } SIZED[] = {
        { GL_ALPHA4, { 0, 0, 0, 4 } }, { GL_LUMINANCE4, { 4, 4, 4, 0 } }, { GL_LUMINANCE4_ALPHA4, { 4, 4, 4, 4 } }, { GL_INTENSITY4, { 4, 4, 4, 4 } }, { GL_R3_G3_B2, { 3, 3, 2, 0 } },
        { GL_RGB4, { 4, 4, 4, 0 } }, { GL_RGB5, { 5, 5, 5, 0 } }, { GL_RGBA2, { 2, 2, 2, 2 } }, { GL_RGBA4, { 4, 4, 4, 4 } }, { GL_RGB5_A1, { 5, 5, 5, 1 } },
    };
    static const struct { GLenum t; uint8_t c[4]; } PACKED[] = {
        { GL_UNSIGNED_SHORT_5_6_5, { 5, 6, 5, 0 } }, { GL_UNSIGNED_SHORT_5_6_5_REV, { 5, 6, 5, 0 } }, { GL_UNSIGNED_SHORT_4_4_4_4, { 4, 4, 4, 4 } }, { GL_UNSIGNED_SHORT_4_4_4_4_REV, { 4, 4, 4, 4 } },
        { GL_UNSIGNED_SHORT_5_5_5_1, { 5, 5, 5, 1 } }, { GL_UNSIGNED_SHORT_1_5_5_5_REV, { 5, 5, 5, 1 } }, { GL_UNSIGNED_BYTE_3_3_2, { 3, 3, 2, 0 } }, { GL_UNSIGNED_BYTE_2_3_3_REV, { 3, 3, 2, 0 } },
    };
    const uint8_t *c = NULL;
    int k = base_kind(in), unsized = in == GL_ALPHA || in == GL_LUMINANCE || in == 1 || in == GL_LUMINANCE_ALPHA || in == 2 || in == GL_INTENSITY || in == GL_RGB || in == 3 || in == GL_RGBA || in == 4, i;
    for (i = 0; i < (int)(sizeof SIZED / sizeof SIZED[0]); i++) if (SIZED[i].f == in) c = SIZED[i].c;
    if (unsized) for (i = 0; i < (int)(sizeof PACKED / sizeof PACKED[0]); i++) if (PACKED[i].t == type) c = PACKED[i].c;
    if (!c) return G2_FMT_RGBA8;
    switch (k) {
    case K_RGB: return c[0] <= 5 && c[1] <= 6 && c[2] <= 5 ? G2_FMT_RGB565 : G2_FMT_RGBA8;
    case K_RGBA: return c[3] <= 1 && c[0] <= 5 && c[1] <= 5 && c[2] <= 5 ? G2_FMT_RGBA5551 : c[0] <= 4 && c[1] <= 4 && c[2] <= 4 && c[3] <= 4 ? G2_FMT_RGBA4444 : G2_FMT_RGBA8;
    case K_A: return c[3] <= 4 ? G2_FMT_RGBA4444 : G2_FMT_RGBA8;
    case K_L: case K_I: return c[0] <= 4 ? G2_FMT_RGBA4444 : G2_FMT_RGBA8;
    case K_LA: return c[0] <= 4 && c[3] <= 4 ? G2_FMT_RGBA4444 : G2_FMT_RGBA8;
    default: return G2_FMT_RGBA8;
    }
}

int gl_fmt_bpp(uint32_t fmt) { return fmt == G2_FMT_RGB565 || fmt == G2_FMT_RGBA4444 || fmt == G2_FMT_RGBA5551 ? 2 : 4; }

int gl_fmt_bits(uint32_t fmt, int c)
{
    static const uint8_t B[4][4] = { { 8, 8, 8, 8 }, { 5, 6, 5, 0 }, { 4, 4, 4, 4 }, { 5, 5, 5, 1 } };
    return B[fmt == G2_FMT_RGB565 ? 1 : fmt == G2_FMT_RGBA4444 ? 2 : fmt == G2_FMT_RGBA5551 ? 3 : 0][c];
}

uint32_t gl_tg_fmt(const g2r_target *t)
{
    if (t->bits_r == 5 && t->bits_g == 6) return G2_FMT_RGB565;
    if (t->bits_r == 4) return G2_FMT_RGBA4444;
    if (t->bits_r == 5) return G2_FMT_RGBA5551;
    return G2_FMT_RGBA8;
}

void gl_tg_bits(g2r_target *t, uint32_t fmt)
{
    t->bits_r = gl_fmt_bits(fmt, 0); t->bits_g = gl_fmt_bits(fmt, 1); t->bits_b = gl_fmt_bits(fmt, 2); t->bits_a = gl_fmt_bits(fmt, 3);
}

int gl_img_write(uint16_t img, uint32_t fmt, const uint8_t *rgba, int pitch, uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t level, uint32_t layer)
{
    uint16_t *p;
    uint32_t r;
    int rc;
    if (gl_fmt_bpp(fmt) == 4) {
        if (pitch == (int)w) return g2_write_image(D.bd, img, rgba, (size_t)w * h * 4, G2_TF_RGBA8888, x, y, w, h, level, layer, 0, fmt);
        return g2_write_image_rect(D.bd, img, rgba, pitch * 4, G2_TF_RGBA8888, x, y, w, h, level, layer, fmt);
    }
    p = (uint16_t *)malloc((size_t)w * h * 2 + 2);
    if (!p) return -1;
    for (r = 0; r < h; r++) g2_px16_pack(fmt, rgba + (size_t)r * pitch * 4, w, p + (size_t)r * w);
    rc = g2_write_image(D.bd, img, p, (size_t)w * h * 2, G2_TF_RAW, x, y, w, h, level, layer, 0, fmt);
    free(p);
    return rc;
}

int gl_img_read(uint16_t img, uint32_t fmt, uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t level, uint32_t layer, uint8_t *rgba)
{
    uint16_t *p;
    int rc;
    if (gl_fmt_bpp(fmt) == 4) return g2_read_image(D.bd, img, G2_TF_RGBA8888, x, y, w, h, level, layer, fmt, 0, 0, rgba);
    p = (uint16_t *)malloc((size_t)w * h * 2 + 2);
    if (!p) return -1;
    rc = g2_read_image(D.bd, img, G2_TF_RAW, x, y, w, h, level, layer, fmt, 0, 0, p);
    g2_px16_unpack(fmt, p, (size_t)w * h, rgba);
    free(p);
    return rc;
}

void gl_quantize(uint32_t fmt, uint8_t *rgba, size_t n)
{
    uint16_t q[256];
    size_t i;
    if (gl_fmt_bpp(fmt) == 4) return;
    for (i = 0; i < n; i += 256) {
        size_t k = n - i < 256 ? n - i : 256;
        g2_px16_pack(fmt, rgba + i * 4, k, q);
        g2_px16_unpack(fmt, q, k, rgba + i * 4);
    }
}

int gl_tex_complete(const gl_tex *t)
{
    int faces = t->target == GL_TEXTURE_CUBE_MAP ? 6 : 1, f, l, b = t->base_level;
    int mip = t->min_filter != GL_NEAREST && t->min_filter != GL_LINEAR;
    if (!t->handle || b >= t->levels || t->max_level < b) return 0;
    for (f = 0; f < faces; f++) {
        if (!t->level[f][b].defined) return 0;
        if (mip) {
            for (l = b + 1; l < t->levels && l <= t->max_level; l++) {
                if (!t->level[f][l].defined) return 0;
            }
        }
    }
    if (t->target == GL_TEXTURE_3D) {
        if (!gl_tex3d_fits(t, b)) return 0;
        if (mip) for (l = b + 1; l < t->levels && l <= t->max_level; l++) if (!gl_tex3d_fits(t, l)) return 0;
    }
    return 1;
}

void gl_tex_view_drop(gl_tex *t)
{
    if (t->ds_alias) { g2r_end_pass(D.r); g2r_alias_destroy(D.r, t->ds_alias); t->ds_alias = 0; }
    if (!t->view) return;
    tex_touch(t);
    if (t->view_copy) g2r_image_destroy(D.r, t->view, t->view_base);
    else g2r_alias_destroy(D.r, t->view);
    t->view = 0; t->view_b = t->view_n = 0;
}

/* BASE_LEVEL and MAX_LEVEL say which part of the chain the TMU sees. A 2D chain is contiguous and level l's rows are tight, so its tail is an image of its own at the same memory. Cube faces lie a
   whole chain apart, which a shorter chain is not: those levels go into an image of their own, once per change */
uint16_t gl_tex_sample_image(gl_tex *t)
{
    int b = t->base_level > 0 ? t->base_level : 0, top = t->max_level < t->levels - 1 ? t->max_level : t->levels - 1, n = top - b + 1, l, f;
    if (t->target == GL_TEXTURE_3D && !t->vol3) return t->handle;
    if (!b && n >= t->levels) { gl_tex_view_drop(t); return t->handle; }
    if (n < 1 || b >= t->levels) return t->handle;
    if (t->view && t->view_b == b && t->view_n == n && t->rev == t->view_rev) return t->view;
    gl_tex_view_drop(t);
    if (t->vol3) {
        uint16_t im = gl_tex3d_view(t, b, n);
        if (t->view) { t->view_b = b; t->view_n = n; t->view_rev = t->rev; }
        return im;
    }
    {
        int w = t->w >> b, h = t->h >> b, faces = t->target == GL_TEXTURE_CUBE_MAP ? 6 : 1, bpp = gl_fmt_bpp(t->hwfmt);
        uint32_t off = b ? g2_mip_chain_bytes((uint32_t)t->w, (uint32_t)t->h, (uint32_t)bpp, (uint32_t)b) : 0;
        w = w ? w : 1; h = h ? h : 1;
        if (faces == 1 && !(off & 31)) {                        /* an image starts on a 32-byte boundary: the small levels of a chain do not */
            uint16_t hd;
            if (g2r_image_alias(D.r, &hd, t->base + off, (uint32_t)w, (uint32_t)h, t->hwfmt, (uint32_t)(w * bpp), 1, (uint32_t)n, G2_USE_SAMPLED) < 0) return t->handle;
            t->view = hd; t->view_base = t->base + off; t->view_copy = 0;
        } else {
            uint8_t *px = (uint8_t *)malloc((size_t)w * h * 4);
            if (!px) return t->handle;
            if (g2r_image_create_tight(D.r, &t->view, &t->view_base, (uint32_t)w, (uint32_t)h, t->hwfmt, (uint32_t)n, (uint32_t)faces, 1, G2_USE_SAMPLED | G2_USE_TRANSFER) < 0) { free(px); t->view = 0; return t->handle; }
            g2r_finish(D.r);
            for (f = 0; f < faces; f++) {
                for (l = 0; l < n; l++) {
                    int lw = w >> l, lh = h >> l;
                    lw = lw ? lw : 1; lh = lh ? lh : 1;
                    g2_read_image(D.bd, t->handle, G2_TF_RAW, 0, 0, (uint32_t)lw, (uint32_t)lh, (uint32_t)(b + l), (uint32_t)f, t->hwfmt, 0, 0, px);
                    g2_write_image(D.bd, t->view, px, (size_t)lw * lh * bpp, G2_TF_RAW, 0, 0, (uint32_t)lw, (uint32_t)lh, (uint32_t)l, (uint32_t)f, 0, t->hwfmt);
                }
            }
            free(px);
            t->view_copy = 1;
        }
        t->view_b = b; t->view_n = n; t->view_rev = t->rev;
    }
    return t->view;
}

/* the draw uses it */
void gl_tex_prepare(gl_tex *t) { gl_tex_resolve(t); node_from(t)->used_pass = g2r_pass_id(D.r); }

static int full_levels(int w, int h)
{
    int n = 1, m = w > h ? w : h;
    while (m > 1) { m >>= 1; n++; }
    return n > MAX_LEVELS ? MAX_LEVELS : n;
}

static int ensure_image(gl_tex *t, int w, int h, int layers, uint32_t fmt)
{
    int levels = full_levels(w, h);
    if (t->handle && t->w == w && t->h == h && t->layers == layers && t->hwfmt == fmt) return 0;
    if (t->handle && t->w == w && t->h == h && t->layers == layers) {     /* the same chain in another format: the levels there go over */
        uint16_t nh;
        uint32_t nb;
        int f, l;
        tex_touch_flush(t);
        gl_tex_view_drop(t);
        gl_tex_rt_drop(t);
        if (t->bimg) { g2r_image_destroy(D.r, t->bimg, t->bimg_base); t->bimg = 0; }
        if (g2r_image_create(D.r, &nh, &nb, (uint32_t)w, (uint32_t)h, fmt, (uint32_t)levels, (uint32_t)layers, 1, TEX_USAGE) < 0) return -1;
        g2r_finish(D.r);
        for (f = 0; f < layers; f++) {
            for (l = 0; l < levels; l++) {
                int lw = w >> l ? w >> l : 1, lh = h >> l ? h >> l : 1;
                uint8_t *px;
                if (!t->level[f][l].defined) continue;
                px = (uint8_t *)malloc((size_t)lw * lh * 4);
                if (!px) continue;
                if (gl_img_read(t->handle, t->hwfmt, 0, 0, (uint32_t)lw, (uint32_t)lh, (uint32_t)l, (uint32_t)f, px) == 0) gl_img_write(nh, fmt, px, lw, 0, 0, (uint32_t)lw, (uint32_t)lh, (uint32_t)l, (uint32_t)f);
                free(px);
            }
        }
        g2r_image_destroy(D.r, t->handle, t->base);
        t->handle = nh; t->base = nb; t->hwfmt = fmt;
        return 0;
    }
    if (t->handle) {
        tex_touch_flush(t);
        gl_tex_view_drop(t);
        gl_tex_rt_drop(t);
        g2r_image_destroy(D.r, t->handle, t->base);
        t->handle = 0;
        memset(t->level, 0, sizeof t->level);
    }
    if (t->bimg) { g2r_image_destroy(D.r, t->bimg, t->bimg_base); t->bimg = 0; }
    if (g2r_image_create(D.r, &t->handle, &t->base, (uint32_t)w, (uint32_t)h, fmt, (uint32_t)levels, (uint32_t)layers, 1, TEX_USAGE) < 0) {
        g2e_log("gl: no room for a %d x %d texture", w, h);
        return -1;
    }
    t->w = w; t->h = h; t->levels = levels; t->layers = layers; t->hwfmt = fmt;
    return 0;
}

/* ---------------------------------------------------------------- pixel unpacking */
static int fmt_comps(GLenum format)
{
    switch (format) {
    case GL_RED: case GL_GREEN: case GL_BLUE: case GL_ALPHA: case GL_LUMINANCE: case GL_INTENSITY: case GL_COLOR_INDEX: case GL_DEPTH_COMPONENT: case GL_STENCIL_INDEX: case 1: return 1;
    case GL_LUMINANCE_ALPHA: case 2: return 2;
    case GL_RGB: case GL_BGR: case 3: return 3;
    default: return 4;
    }
}

static int type_bytes(GLenum type, GLenum format)
{
    switch (type) {
    case GL_UNSIGNED_BYTE: case GL_BYTE: return fmt_comps(format);
    case GL_UNSIGNED_BYTE_3_3_2: case GL_UNSIGNED_BYTE_2_3_3_REV: return 1;
    case GL_UNSIGNED_SHORT: case GL_SHORT: return 2 * fmt_comps(format);
    case GL_UNSIGNED_INT: case GL_INT: case GL_FLOAT: return 4 * fmt_comps(format);
    case GL_UNSIGNED_SHORT_5_6_5: case GL_UNSIGNED_SHORT_5_6_5_REV: case GL_UNSIGNED_SHORT_4_4_4_4: case GL_UNSIGNED_SHORT_4_4_4_4_REV: case GL_UNSIGNED_SHORT_5_5_5_1:
    case GL_UNSIGNED_SHORT_1_5_5_5_REV: return 2;
    default: return 4;
    }
}

/* one pixel to floats-in-bytes r g b a */
static void read_px(const uint8_t *p, GLenum format, GLenum type, uint8_t out[4])
{
    int comp[4] = { 0, 0, 0, 255 }, n = fmt_comps(format), i;
    switch (type) {
    case GL_UNSIGNED_BYTE: for (i = 0; i < n; i++) comp[i] = p[i]; break;
    case GL_BYTE: for (i = 0; i < n; i++) { int v = ((const signed char *)p)[i]; comp[i] = v < 0 ? 0 : v * 255 / 127; } break;
    case GL_UNSIGNED_SHORT: for (i = 0; i < n; i++) comp[i] = ((const uint16_t *)p)[i] >> 8; break;
    case GL_SHORT: for (i = 0; i < n; i++) { int v = ((const int16_t *)p)[i]; comp[i] = v < 0 ? 0 : v >> 7; } break;
    case GL_UNSIGNED_INT: for (i = 0; i < n; i++) comp[i] = (int)(((const uint32_t *)p)[i] >> 24); break;
    case GL_INT: for (i = 0; i < n; i++) { int v = ((const int32_t *)p)[i]; comp[i] = v < 0 ? 0 : v >> 23; } break;
    case GL_FLOAT:
        for (i = 0; i < n; i++) {
            float v = ((const float *)p)[i];
            comp[i] = v <= 0.0f ? 0 : v >= 1.0f ? 255 : (int)(v * 255.0f + 0.5f);
        }
        break;
    case GL_UNSIGNED_SHORT_5_6_5: { uint32_t v = *(const uint16_t *)p; comp[0] = (int)((v >> 11) * 255 / 31); comp[1] = (int)(((v >> 5) & 63) * 255 / 63); comp[2] = (int)((v & 31) * 255 / 31); break; }
    case GL_UNSIGNED_SHORT_5_6_5_REV: { uint32_t v = *(const uint16_t *)p; comp[2] = (int)((v >> 11) * 255 / 31); comp[1] = (int)(((v >> 5) & 63) * 255 / 63); comp[0] = (int)((v & 31) * 255 / 31); break; }
    case GL_UNSIGNED_SHORT_4_4_4_4: { uint32_t v = *(const uint16_t *)p; comp[0] = (int)((v >> 12) * 17); comp[1] = (int)(((v >> 8) & 15) * 17); comp[2] = (int)(((v >> 4) & 15) * 17); comp[3] = (int)((v & 15) * 17); break; }
    case GL_UNSIGNED_SHORT_4_4_4_4_REV: { uint32_t v = *(const uint16_t *)p; comp[3] = (int)((v >> 12) * 17); comp[2] = (int)(((v >> 8) & 15) * 17); comp[1] = (int)(((v >> 4) & 15) * 17); comp[0] = (int)((v & 15) * 17); break; }
    case GL_UNSIGNED_SHORT_5_5_5_1: { uint32_t v = *(const uint16_t *)p; comp[0] = (int)((v >> 11) * 255 / 31); comp[1] = (int)(((v >> 6) & 31) * 255 / 31); comp[2] = (int)(((v >> 1) & 31) * 255 / 31); comp[3] = v & 1 ? 255 : 0; break; }
    case GL_UNSIGNED_SHORT_1_5_5_5_REV: { uint32_t v = *(const uint16_t *)p; comp[3] = v >> 15 ? 255 : 0; comp[2] = (int)(((v >> 10) & 31) * 255 / 31); comp[1] = (int)(((v >> 5) & 31) * 255 / 31); comp[0] = (int)((v & 31) * 255 / 31); break; }
    case GL_UNSIGNED_INT_8_8_8_8: { uint32_t v = *(const uint32_t *)p; comp[0] = (int)(v >> 24); comp[1] = (int)((v >> 16) & 255); comp[2] = (int)((v >> 8) & 255); comp[3] = (int)(v & 255); break; }
    case GL_UNSIGNED_INT_8_8_8_8_REV: { uint32_t v = *(const uint32_t *)p; comp[3] = (int)(v >> 24); comp[2] = (int)((v >> 16) & 255); comp[1] = (int)((v >> 8) & 255); comp[0] = (int)(v & 255); break; }
    case GL_UNSIGNED_BYTE_3_3_2: { uint32_t v = *p; comp[0] = (int)((v >> 5) * 255 / 7); comp[1] = (int)(((v >> 2) & 7) * 255 / 7); comp[2] = (int)((v & 3) * 85); break; }
    case GL_UNSIGNED_BYTE_2_3_3_REV: { uint32_t v = *p; comp[2] = (int)((v >> 6) * 85); comp[1] = (int)(((v >> 3) & 7) * 255 / 7); comp[0] = (int)((v & 7) * 255 / 7); break; }
    case GL_UNSIGNED_INT_10_10_10_2: { uint32_t v = *(const uint32_t *)p; comp[0] = (int)(v >> 24); comp[1] = (int)((v >> 14) & 255); comp[2] = (int)((v >> 4) & 255); comp[3] = (int)((v & 3) * 85); break; }
    case GL_UNSIGNED_INT_2_10_10_10_REV: { uint32_t v = *(const uint32_t *)p; comp[3] = (int)((v >> 30) * 85); comp[2] = (int)((v >> 22) & 255); comp[1] = (int)((v >> 12) & 255); comp[0] = (int)((v >> 2) & 255); break; }
    default: break;
    }
    switch (format) {
    case GL_BGR: case GL_BGRA: if (type != GL_UNSIGNED_SHORT_5_6_5 && type != GL_UNSIGNED_SHORT_5_6_5_REV) { int t = comp[0]; comp[0] = comp[2]; comp[2] = t; } break;
    case GL_ALPHA: comp[3] = comp[0]; comp[0] = comp[1] = comp[2] = 0; break;
    case GL_RED: comp[1] = comp[2] = 0; break;
    case GL_GREEN: comp[1] = comp[0]; comp[0] = comp[2] = 0; break;
    case GL_BLUE: comp[2] = comp[0]; comp[0] = comp[1] = 0; break;
    case GL_LUMINANCE: case 1: comp[1] = comp[2] = comp[0]; break;
    case GL_INTENSITY: comp[1] = comp[2] = comp[3] = comp[0]; break;
    case GL_LUMINANCE_ALPHA: case 2: comp[3] = comp[1]; comp[1] = comp[2] = comp[0]; break;
    default: break;
    }
    out[0] = (uint8_t)comp[0]; out[1] = (uint8_t)comp[1]; out[2] = (uint8_t)comp[2]; out[3] = (uint8_t)comp[3];
}

static int base_class_of(GLenum internal)
{
    switch (internal) {                                         /* what the compressed formats are when they are stored as texels */
    case GL_DEPTH_COMPONENT: case GL_DEPTH_COMPONENT16: case GL_DEPTH_COMPONENT24: case GL_DEPTH_COMPONENT32: case GL_DEPTH_STENCIL_EXT: case GL_DEPTH24_STENCIL8_EXT: return BF_DEPTH;
    case GL_COMPRESSED_RGB: case GL_COMPRESSED_RGB_S3TC_DXT1_EXT: return BF_RGB;
    case GL_COMPRESSED_RGBA: case GL_COMPRESSED_RGBA_S3TC_DXT1_EXT: case GL_COMPRESSED_RGBA_S3TC_DXT3_EXT: case GL_COMPRESSED_RGBA_S3TC_DXT5_EXT: return BF_RGBA;
    case GL_COMPRESSED_ALPHA: return BF_ALPHA;
    case GL_COMPRESSED_LUMINANCE: return BF_RGB;
    case GL_COMPRESSED_LUMINANCE_ALPHA: return BF_RGBA;
    case GL_COMPRESSED_INTENSITY: return BF_INTENSITY;
    default: break;
    }
    switch (internal) {
    case GL_ALPHA: case GL_ALPHA4: case GL_ALPHA8: case GL_ALPHA12: case GL_ALPHA16: return BF_ALPHA;
    case GL_INTENSITY: case GL_INTENSITY4: case GL_INTENSITY8: case GL_INTENSITY12: case GL_INTENSITY16: return BF_INTENSITY;
    case GL_LUMINANCE: case 1: case GL_LUMINANCE4: case GL_LUMINANCE8: case GL_LUMINANCE12: case GL_LUMINANCE16: case GL_RGB: case 3: case GL_R3_G3_B2: case GL_RGB4: case GL_RGB5:
    case GL_RGB8: case GL_RGB10: case GL_RGB12: case GL_RGB16: return BF_RGB;
    default: return BF_RGBA;                                    /* RGBA and LUMINANCE_ALPHA */
    }
}

/* the source pixels to RGBA8, then what the internal format keeps (§3.6.4, table 3.15) */
void gl_unpack(uint8_t *rgba, GLsizei w, GLsizei h, GLenum format, GLenum type, const void *pixels, GLenum internal)
{
    int bpp = type_bytes(type, format), row = (C->un_row ? C->un_row : w) * bpp, x, y;
    int cls = internal ? base_class_of(internal) : -1, lum = internal == GL_LUMINANCE || internal == 1 || (internal >= GL_LUMINANCE4 && internal <= GL_LUMINANCE16);
    int lumalpha = internal == GL_LUMINANCE_ALPHA || internal == 2 || (internal >= GL_LUMINANCE4_ALPHA4 && internal <= GL_LUMINANCE16_ALPHA16);
    const uint8_t *src = (const uint8_t *)pixels;
    row = (row + C->un_align - 1) / C->un_align * C->un_align;
    src += (ptrdiff_t)C->un_skip_row * row + (ptrdiff_t)C->un_skip_px * bpp;
    if (C->bound_array && 0) src = NULL;
    if (type == GL_BITMAP) {                                        /* colour indices of one bit; rows are whole bytes */
        row = ((C->un_row ? C->un_row : w) + 7) / 8;
        row = (row + C->un_align - 1) / C->un_align * C->un_align;
        src = (const uint8_t *)pixels + (ptrdiff_t)C->un_skip_row * row;
    }
    for (y = 0; y < h; y++) {
        const uint8_t *p = src + (ptrdiff_t)y * row;
        uint8_t *o = rgba + (size_t)y * w * 4;
        if (format == GL_COLOR_INDEX) {
            for (x = 0; x < w; x++, o += 4) {
                uint32_t i;
                switch (type) {
                case GL_BITMAP: { int b = x + C->un_skip_px; i = (p[b / 8] >> (7 - b % 8)) & 1; break; }
                case GL_UNSIGNED_BYTE: case GL_BYTE: i = p[x]; break;
                case GL_UNSIGNED_SHORT: case GL_SHORT: i = ((const uint16_t *)p)[x]; break;
                case GL_FLOAT: i = (uint32_t)((const float *)p)[x]; break;
                default: i = ((const uint32_t *)p)[x]; break;
                }
                gl_px_index(i, o);
            }
            continue;
        }
        if (type == GL_UNSIGNED_BYTE && format == GL_RGBA && cls == BF_RGBA && !lumalpha && !C->px_active) {
            memcpy(o, p, (size_t)w * 4);
            continue;
        }
        for (x = 0; x < w; x++, p += bpp, o += 4) {
            read_px(p, format, type, o);
            if (C->px_active) gl_px_one(o);
            if (cls == BF_ALPHA) { o[0] = o[1] = o[2] = 0; }
            else if (cls == BF_INTENSITY) { o[1] = o[2] = o[3] = o[0]; }
            else if (lum) { o[1] = o[2] = o[0]; o[3] = 255; }
            else if (lumalpha) { o[1] = o[2] = o[0]; }
            else if (cls == BF_RGB) { o[3] = 255; }
        }
    }
}

/* depth and stencil pixels: their own conversions, §3.6.4; the rows are found as for colour */
static const uint8_t *unpack_start(const void *pixels, GLsizei w, int bpp, int *row)
{
    int r = (C->un_row ? C->un_row : w) * bpp;
    *row = (r + C->un_align - 1) / C->un_align * C->un_align;
    return (const uint8_t *)pixels + (ptrdiff_t)C->un_skip_row * *row + (ptrdiff_t)C->un_skip_px * bpp;
}

void gl_unpack_depth(float *o, GLsizei w, GLsizei h, GLenum type, const void *pixels)
{
    int bpp = type_bytes(type, GL_DEPTH_COMPONENT), row, x, y;
    const uint8_t *src = unpack_start(pixels, w, bpp, &row);
    for (y = 0; y < h; y++) {
        const uint8_t *p = src + (ptrdiff_t)y * row;
        for (x = 0; x < w; x++, p += bpp) {
            double v;
            switch (type) {
            case GL_UNSIGNED_BYTE: v = *p / 255.0; break;
            case GL_BYTE: v = (2.0 * *(const signed char *)p + 1.0) / 255.0; break;
            case GL_UNSIGNED_SHORT: v = *(const uint16_t *)p / 65535.0; break;
            case GL_SHORT: v = (2.0 * *(const int16_t *)p + 1.0) / 65535.0; break;
            case GL_UNSIGNED_INT: v = *(const uint32_t *)p / 4294967295.0; break;
            case GL_INT: v = (2.0 * *(const int32_t *)p + 1.0) / 4294967295.0; break;
            default: v = *(const float *)p; break;
            }
            *o++ = gl_px_depth_value((float)v);
        }
    }
}

void gl_unpack_ds(float *z, uint32_t *s, GLsizei w, GLsizei h, const void *pixels)
{
    int row, x, y;
    const uint8_t *src = unpack_start(pixels, w, 4, &row);
    for (y = 0; y < h; y++) {
        const uint32_t *p = (const uint32_t *)(src + (ptrdiff_t)y * row);
        for (x = 0; x < w; x++) {
            *z++ = gl_px_depth_value((float)((p[x] >> 8) / 16777215.0));
            *s++ = gl_px_stencil_value(p[x] & 255);
        }
    }
}

void gl_unpack_index(uint32_t *o, GLsizei w, GLsizei h, GLenum type, const void *pixels)
{
    int bpp = type == GL_BITMAP ? 1 : type_bytes(type, GL_STENCIL_INDEX), row, x, y;
    const uint8_t *src = unpack_start(pixels, w, bpp, &row);
    if (type == GL_BITMAP) {
        row = ((C->un_row ? C->un_row : w) + 7) / 8;
        row = (row + C->un_align - 1) / C->un_align * C->un_align;
        src = (const uint8_t *)pixels + (ptrdiff_t)C->un_skip_row * row;
    }
    for (y = 0; y < h; y++) {
        const uint8_t *p = src + (ptrdiff_t)y * row;
        for (x = 0; x < w; x++) {
            uint32_t i;
            switch (type) {
            case GL_BITMAP: { int b = x + C->un_skip_px; i = (p[b / 8] >> (7 - b % 8)) & 1; break; }
            case GL_UNSIGNED_BYTE: case GL_BYTE: i = p[x]; break;
            case GL_UNSIGNED_SHORT: case GL_SHORT: i = ((const uint16_t *)p)[x]; break;
            case GL_FLOAT: i = (uint32_t)((const float *)p)[x]; break;
            default: i = ((const uint32_t *)p)[x]; break;
            }
            *o++ = gl_px_stencil_value(i);
        }
    }
}

/* ---------------------------------------------------------------- pixel packing, §4.3.2 */
static int packed_type(GLenum type)
{
    switch (type) {
    case GL_UNSIGNED_BYTE_3_3_2: case GL_UNSIGNED_BYTE_2_3_3_REV: case GL_UNSIGNED_SHORT_5_6_5: case GL_UNSIGNED_SHORT_5_6_5_REV: case GL_UNSIGNED_SHORT_4_4_4_4: case GL_UNSIGNED_SHORT_4_4_4_4_REV:
    case GL_UNSIGNED_SHORT_5_5_5_1: case GL_UNSIGNED_SHORT_1_5_5_5_REV: case GL_UNSIGNED_INT_8_8_8_8: case GL_UNSIGNED_INT_8_8_8_8_REV: case GL_UNSIGNED_INT_10_10_10_2: case GL_UNSIGNED_INT_2_10_10_10_REV: return 1;
    default: return 0;
    }
}

static int pack_ok(GLenum format, GLenum type)
{
    switch (format) {
    case GL_RED: case GL_GREEN: case GL_BLUE: case GL_ALPHA: case GL_RGB: case GL_BGR: case GL_RGBA: case GL_BGRA: case GL_LUMINANCE: case GL_LUMINANCE_ALPHA: break;
    default: return 0;
    }
    switch (type) {
    case GL_UNSIGNED_BYTE: case GL_BYTE: case GL_UNSIGNED_SHORT: case GL_SHORT: case GL_UNSIGNED_INT: case GL_INT: case GL_FLOAT: return 1;
    default: return packed_type(type);
    }
}

/* 0, or what §3.6.4 says is wrong with the pair: a packed type belongs to the format with its number of components */
static GLenum pack_error(GLenum format, GLenum type)
{
    if (!pack_ok(format, type)) return GL_INVALID_ENUM;
    switch (type) {
    case GL_UNSIGNED_BYTE_3_3_2: case GL_UNSIGNED_BYTE_2_3_3_REV: case GL_UNSIGNED_SHORT_5_6_5: case GL_UNSIGNED_SHORT_5_6_5_REV: return format == GL_RGB ? 0 : GL_INVALID_OPERATION;
    default: return !packed_type(type) || format == GL_RGBA || format == GL_BGRA ? 0 : GL_INVALID_OPERATION;
    }
}

/* DEPTH_COMPONENT and STENCIL_INDEX take the plain types, and only stencil takes GL_BITMAP */
int gl_ds_type(GLenum format, GLenum type) { return (type == GL_BITMAP && format == GL_STENCIL_INDEX) || (format == GL_DEPTH_STENCIL_EXT ? type == GL_UNSIGNED_INT_24_8_EXT : pack_ok(GL_RGBA, type) && !packed_type(type)); }

static uint32_t scale_bits(double v, int bits) { return (uint32_t)(v * (double)((1u << bits) - 1) + 0.5); }

/* where the destination's row r starts: PACK_ROW_LENGTH, the skips, the alignment */
static uint8_t *pack_row(void *pixels, int w, int r, int bpp)
{
    int row = (C->pk_row ? C->pk_row : w) * bpp;
    row = (row + C->pk_align - 1) / C->pk_align * C->pk_align;
    return (uint8_t *)pixels + (ptrdiff_t)(r + C->pk_skip_row) * row + (ptrdiff_t)C->pk_skip_px * bpp;
}

/* one component of a group, v in 0 .. 1 */
static void pack_comp(uint8_t *o, GLenum type, double v)
{
    switch (type) {
    case GL_UNSIGNED_BYTE: *o = (uint8_t)scale_bits(v, 8); break;
    case GL_BYTE: *(signed char *)o = (signed char)(v * 127.0 + 0.5); break;
    case GL_UNSIGNED_SHORT: *(uint16_t *)o = (uint16_t)scale_bits(v, 16); break;
    case GL_SHORT: *(int16_t *)o = (int16_t)(v * 32767.0 + 0.5); break;
    case GL_UNSIGNED_INT: *(uint32_t *)o = (uint32_t)(v * 4294967295.0 + 0.5); break;
    case GL_INT: *(int32_t *)o = (int32_t)(v * 2147483647.0 + 0.5); break;
    default: *(float *)o = (float)v; break;
    }
}

static void pack_packed(uint8_t *o, GLenum type, const double *c)
{
    uint32_t v;
    switch (type) {
    case GL_UNSIGNED_BYTE_3_3_2: *o = (uint8_t)(scale_bits(c[0], 3) << 5 | scale_bits(c[1], 3) << 2 | scale_bits(c[2], 2)); return;
    case GL_UNSIGNED_BYTE_2_3_3_REV: *o = (uint8_t)(scale_bits(c[2], 2) << 6 | scale_bits(c[1], 3) << 3 | scale_bits(c[0], 3)); return;
    case GL_UNSIGNED_SHORT_5_6_5: v = scale_bits(c[0], 5) << 11 | scale_bits(c[1], 6) << 5 | scale_bits(c[2], 5); *(uint16_t *)o = (uint16_t)v; return;
    case GL_UNSIGNED_SHORT_5_6_5_REV: v = scale_bits(c[2], 5) << 11 | scale_bits(c[1], 6) << 5 | scale_bits(c[0], 5); *(uint16_t *)o = (uint16_t)v; return;
    case GL_UNSIGNED_SHORT_4_4_4_4: v = scale_bits(c[0], 4) << 12 | scale_bits(c[1], 4) << 8 | scale_bits(c[2], 4) << 4 | scale_bits(c[3], 4); *(uint16_t *)o = (uint16_t)v; return;
    case GL_UNSIGNED_SHORT_4_4_4_4_REV: v = scale_bits(c[3], 4) << 12 | scale_bits(c[2], 4) << 8 | scale_bits(c[1], 4) << 4 | scale_bits(c[0], 4); *(uint16_t *)o = (uint16_t)v; return;
    case GL_UNSIGNED_SHORT_5_5_5_1: v = scale_bits(c[0], 5) << 11 | scale_bits(c[1], 5) << 6 | scale_bits(c[2], 5) << 1 | scale_bits(c[3], 1); *(uint16_t *)o = (uint16_t)v; return;
    case GL_UNSIGNED_SHORT_1_5_5_5_REV: v = scale_bits(c[3], 1) << 15 | scale_bits(c[2], 5) << 10 | scale_bits(c[1], 5) << 5 | scale_bits(c[0], 5); *(uint16_t *)o = (uint16_t)v; return;
    case GL_UNSIGNED_INT_8_8_8_8: *(uint32_t *)o = scale_bits(c[0], 8) << 24 | scale_bits(c[1], 8) << 16 | scale_bits(c[2], 8) << 8 | scale_bits(c[3], 8); return;
    case GL_UNSIGNED_INT_8_8_8_8_REV: *(uint32_t *)o = scale_bits(c[3], 8) << 24 | scale_bits(c[2], 8) << 16 | scale_bits(c[1], 8) << 8 | scale_bits(c[0], 8); return;
    case GL_UNSIGNED_INT_10_10_10_2: *(uint32_t *)o = scale_bits(c[0], 10) << 22 | scale_bits(c[1], 10) << 12 | scale_bits(c[2], 10) << 2 | scale_bits(c[3], 2); return;
    default: *(uint32_t *)o = scale_bits(c[3], 2) << 30 | scale_bits(c[2], 10) << 20 | scale_bits(c[1], 10) << 10 | scale_bits(c[0], 10); return;
    }
}

/* RGBA bytes into the application's format and type. Luminance is the sum of red, green and blue */
static void gl_pack(const uint8_t *rgba, int w, int h, GLenum format, GLenum type, void *pixels)
{
    int bpp = type_bytes(type, format), n = fmt_comps(format), one = bpp / n, x, y, k;
    for (y = 0; y < h; y++) {
        const uint8_t *s = rgba + (size_t)y * w * 4;
        uint8_t *o = pack_row(pixels, w, y, bpp);
        if (format == GL_RGBA && type == GL_UNSIGNED_BYTE) { memcpy(o, s, (size_t)w * 4); continue; }
        for (x = 0; x < w; x++, s += 4, o += bpp) {
            double c[4], g[4], lum;
            for (k = 0; k < 4; k++) c[k] = s[k] / 255.0;
            lum = c[0] + c[1] + c[2];
            if (lum > 1.0) lum = 1.0;
            switch (format) {
            case GL_RED: g[0] = c[0]; break;
            case GL_GREEN: g[0] = c[1]; break;
            case GL_BLUE: g[0] = c[2]; break;
            case GL_ALPHA: g[0] = c[3]; break;
            case GL_LUMINANCE: g[0] = lum; break;
            case GL_LUMINANCE_ALPHA: g[0] = lum; g[1] = c[3]; break;
            case GL_BGR: g[0] = c[2]; g[1] = c[1]; g[2] = c[0]; g[3] = 1.0; break;
            case GL_BGRA: g[0] = c[2]; g[1] = c[1]; g[2] = c[0]; g[3] = c[3]; break;
            default: memcpy(g, c, sizeof g); break;
            }
            if (packed_type(type)) pack_packed(o, type, g);
            else for (k = 0; k < n; k++) pack_comp(o + k * one, type, g[k]);
        }
    }
}

/* the raw words of depth-stencil pixels as DEPTH_COMPONENT or STENCIL_INDEX */
static void pack_ds(const uint32_t *ds, int w, int h, GLenum format, GLenum type, void *pixels, int d16)
{
    int bpp = type == GL_BITMAP ? 1 : type_bytes(type, format), x, y;
    double dmax = d16 ? 65535.0 : 16777215.0;
    for (y = 0; y < h; y++) {
        const uint32_t *s = ds + (size_t)y * w;
        uint8_t *o = pack_row(pixels, type == GL_BITMAP ? (w + 7) / 8 : w, y, bpp);
        if (type == GL_BITMAP) memset(o, 0, (size_t)(w + 7) / 8);
        for (x = 0; x < w; x++) {
            uint8_t *q = o + (size_t)x * bpp;
            uint32_t d24 = d16 ? (uint32_t)((s[x] & 0xFFFF) * 16777215.0 / 65535.0 + 0.5) : s[x] & 0xFFFFFF;
            if (format == GL_DEPTH_STENCIL_EXT) *(uint32_t *)q = d24 << 8 | gl_px_stencil_value(s[x] >> 24);
            else if (format == GL_DEPTH_COMPONENT) {
                pack_comp(q, type, gl_px_depth_value((float)((s[x] & 0xFFFFFF) / dmax)));
                if (type == GL_UNSIGNED_INT && C->px_depth_scale == 1.0f && C->px_depth_bias == 0.0f)       /* all the bits, which a float does not hold */
                    *(uint32_t *)q = d16 ? (s[x] & 0xFFFF) * 65537u : (s[x] & 0xFFFFFF) << 8 | (s[x] & 0xFFFFFF) >> 16;
            } else {
                uint32_t v = gl_px_stencil_value(s[x] >> 24);
                switch (type) {
                case GL_BITMAP: if (v & 1) o[x / 8] |= (uint8_t)(0x80 >> (x % 8)); break;
                case GL_UNSIGNED_BYTE: case GL_BYTE: *q = (uint8_t)v; break;
                case GL_UNSIGNED_SHORT: case GL_SHORT: *(uint16_t *)q = (uint16_t)v; break;
                case GL_FLOAT: *(float *)q = (float)v; break;
                default: *(uint32_t *)q = v; break;
                }
            }
        }
    }
}

/* A rectangle of a target in window coordinates, 4 bytes a pixel. A window rendered smaller gives the nearest pixel of what exists; what is outside the target reads as zero */
int gl_read_rect(const g2r_target *t, uint16_t image, uint32_t tf, uint32_t fmt, int x, int y, int w, int h, void *out)
{
    double kx, ky;
    int rx0, ry0, rx1, ry1, rw, rh, r, c, rc;
    uint8_t *big;
    gl_scale(&kx, &ky);
    rx0 = (int)floor(x * kx); ry0 = (int)floor(y * ky); rx1 = (int)ceil((x + w) * kx); ry1 = (int)ceil((y + h) * ky);
    if (rx0 < 0) rx0 = 0;
    if (ry0 < 0) ry0 = 0;
    if (rx1 > (int)t->width) rx1 = (int)t->width;
    if (ry1 > (int)t->height) ry1 = (int)t->height;
    rw = rx1 - rx0; rh = ry1 - ry0;
    memset(out, 0, (size_t)w * h * 4);
    if (rw <= 0 || rh <= 0) return 0;
    big = (uint8_t *)malloc((size_t)rw * rh * 4);
    if (!big) return -1;
    if (fmt != G2_FMT_D24S8 && gl_fmt_bpp(fmt) == 2) rc = gl_img_read(image, fmt, (uint32_t)rx0, (uint32_t)ry0, (uint32_t)rw, (uint32_t)rh, 0, 0, big);
    else rc = g2_read_image(D.bd, image, tf, (uint32_t)rx0, (uint32_t)ry0, (uint32_t)rw, (uint32_t)rh, 0, 0, fmt, 0, 0, big);
    for (r = 0; r < h; r++) {
        int sy = (int)floor((y + r + 0.5) * ky) - ry0;
        if (sy < 0 || sy >= rh) continue;
        for (c = 0; c < w; c++) {
            int sx = (int)floor((x + c + 0.5) * kx) - rx0;
            if (sx >= 0 && sx < rw) memcpy((uint8_t *)out + ((size_t)r * w + c) * 4, big + ((size_t)sy * rw + sx) * 4, 4);
        }
    }
    free(big);
    return rc;
}

static int face_of(GLenum target)
{
    return target >= GL_TEXTURE_CUBE_MAP_POSITIVE_X && target <= GL_TEXTURE_CUBE_MAP_NEGATIVE_Z ? (int)(target - GL_TEXTURE_CUBE_MAP_POSITIVE_X) : 0;
}

gl_tex *gl_bound_tex(GLenum target)
{
    gl_unit *u = &C->unit[C->active_unit];
    int cube = target != GL_TEXTURE_2D && target != GL_TEXTURE_1D;
    gl_tex *t;
    if (target == GL_TEXTURE_3D) {
        t = gl_tex_lookup(u->bound_3d, GL_TEXTURE_3D, 1);
        if (t) t->target = GL_TEXTURE_3D;
        return t;
    }
    if (target == GL_TEXTURE_1D) {
        t = gl_tex_lookup(u->bound_1d, GL_TEXTURE_1D, 1);
        if (t) t->target = GL_TEXTURE_1D;
        return t;
    }
    t = gl_tex_lookup(cube ? u->bound_cube : u->bound_2d, cube ? GL_TEXTURE_CUBE_MAP : GL_TEXTURE_2D, 1);
    if (t) t->target = cube ? GL_TEXTURE_CUBE_MAP : GL_TEXTURE_2D;
    return t;
}

#define bound_tex gl_bound_tex

const void *gl_unpack_source(const void *data) { return gl_unpack_ptr(data); }
int gl_base_class(GLenum internal) { return base_class_of(internal); }

/* the upload of what the PC decoded: no pixel store, no pixel transfer */
void gl_tex_store_rgba(GLenum target, GLint level, GLenum internal, int w, int h, const uint8_t *rgba)
{
    int al = C->un_align, ur = C->un_row, us = C->un_skip_px, uz = C->un_skip_row, tr = C->px_active;
    C->un_align = 4; C->un_row = C->un_skip_px = C->un_skip_row = 0; C->px_active = 0;
    GL_FROM_CLIENT(glTexImage2D(target, level, (GLint)internal, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba));
    C->un_align = al; C->un_row = ur; C->un_skip_px = us; C->un_skip_row = uz; C->px_active = tr;
}

void gl_tex_sub_rgba(GLenum target, GLint level, int x, int y, int w, int h, const uint8_t *rgba)
{
    int al = C->un_align, ur = C->un_row, us = C->un_skip_px, uz = C->un_skip_row, tr = C->px_active;
    C->un_align = 4; C->un_row = C->un_skip_px = C->un_skip_row = 0; C->px_active = 0;
    GL_FROM_CLIENT(glTexSubImage2D(target, level, x, y, w, h, GL_RGBA, GL_UNSIGNED_BYTE, rgba));
    C->un_align = al; C->un_row = ur; C->un_skip_px = us; C->un_skip_row = uz; C->px_active = tr;
}

static uint32_t depth_of(const uint8_t *p) { return (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }

/* the mean of depths, not of their bytes */
static void box_depth(uint8_t *dst, const uint8_t *src, int sw, int sh, int dw, int dh)
{
    int x, y;
    for (y = 0; y < dh; y++) {
        int y0 = sh > 1 ? 2 * y : 0, y1 = sh > 1 ? 2 * y + 1 : 0;
        for (x = 0; x < dw; x++) {
            int x0 = sw > 1 ? 2 * x : 0, x1 = sw > 1 ? 2 * x + 1 : 0;
            uint32_t v = (depth_of(src + (y0 * sw + x0) * 4) + depth_of(src + (y0 * sw + x1) * 4) + depth_of(src + (y1 * sw + x0) * 4) + depth_of(src + (y1 * sw + x1) * 4) + 2) >> 2;
            uint8_t *o = dst + (y * dw + x) * 4;
            o[0] = 0; o[1] = (uint8_t)(v >> 16); o[2] = (uint8_t)(v >> 8); o[3] = (uint8_t)v;
        }
    }
}

static void box_mip(uint8_t *dst, const uint8_t *src, int sw, int sh, int dw, int dh)
{
    int x, y, c;
    for (y = 0; y < dh; y++) {
        int y0 = sh > 1 ? 2 * y : 0, y1 = sh > 1 ? 2 * y + 1 : 0;
        for (x = 0; x < dw; x++) {
            int x0 = sw > 1 ? 2 * x : 0, x1 = sw > 1 ? 2 * x + 1 : 0;
            for (c = 0; c < 4; c++) {
                dst[(y * dw + x) * 4 + c] = (uint8_t)((src[(y0 * sw + x0) * 4 + c] + src[(y0 * sw + x1) * 4 + c] + src[(y1 * sw + x0) * 4 + c] + src[(y1 * sw + x1) * 4 + c] + 2) >> 2);
            }
        }
    }
}

static void generate_levels(gl_tex *t, int face, const uint8_t *level0)
{
    int l, w = t->w, h = t->h;
    uint8_t *cur = (uint8_t *)malloc((size_t)w * h * 4);
    if (!cur) return;
    memcpy(cur, level0, (size_t)w * h * 4);
    for (l = 1; l < t->levels; l++) {
        int nw = w > 1 ? w / 2 : 1, nh = h > 1 ? h / 2 : 1;
        uint8_t *nx = (uint8_t *)malloc((size_t)nw * nh * 4);
        if (!nx) break;
        if (t->base_class == BF_DEPTH) box_depth(nx, cur, w, h, nw, nh);
        else box_mip(nx, cur, w, h, nw, nh);
        if (t->base_class == BF_DEPTH) g2_write_image(D.bd, t->handle, nx, (size_t)nw * nh * 4, G2_TF_RGBA8888, 0, 0, (uint32_t)nw, (uint32_t)nh, (uint32_t)l, (uint32_t)face, 0, G2_FMT_RGBA8);
        else gl_img_write(t->handle, t->hwfmt, nx, nw, 0, 0, (uint32_t)nw, (uint32_t)nh, (uint32_t)l, (uint32_t)face);
        t->level[face][l].defined = 1; t->level[face][l].w = nw; t->level[face][l].h = nh;
        free(cur);
        cur = nx; w = nw; h = nh;
    }
    free(cur);
}

/* level 0's border: the hardware has no border texels, so the texture goes with an image of level 0 that has them, and the mean of them for what the TMU's mip chain cannot hold */
static void border_image(gl_tex *t, const uint8_t *rgba, int w, int h)
{
    int x, y, c, n = 0, oned = h == 1;
    double sum[4] = { 0, 0, 0, 0 };
    if (t->bimg && (t->bimg_w != w || t->bimg_h != h)) { tex_touch_flush(t); g2r_image_destroy(D.r, t->bimg, t->bimg_base); t->bimg = 0; }
    if (!t->bimg && g2r_image_create_tight(D.r, &t->bimg, &t->bimg_base, (uint32_t)w, (uint32_t)h, t->hwfmt, 1, 1, 1, G2_USE_SAMPLED | G2_USE_TRANSFER) < 0) { t->bimg = 0; return; }
    t->bimg_w = w; t->bimg_h = h;
    gl_img_write(t->bimg, t->hwfmt, rgba, w, 0, 0, (uint32_t)w, (uint32_t)h, 0, 0);
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            if (x && x != w - 1 && (oned || (y && y != h - 1))) continue;
            if (t->base_class == BF_DEPTH) sum[0] += depth_of(rgba + (y * w + x) * 4) / 16777215.0;
            else for (c = 0; c < 4; c++) sum[c] += rgba[(y * w + x) * 4 + c] / 255.0;
            n++;
        }
    }
    for (c = 0; c < 4; c++) t->ring[c] = n ? (float)(sum[c] / n) : 0.0f;
}

/* the colour outside a texture that has no texels of its own there, as its base format reads it (§3.8.7) */
void gl_tex_border_color(const gl_tex *t, float *o)
{
    const float *c = t->bimg ? t->ring : t->border_color;
    int lum = t->internal == GL_LUMINANCE || t->internal == 1 || (t->internal >= GL_LUMINANCE4 && t->internal <= GL_LUMINANCE16);
    o[0] = c[0]; o[1] = c[1]; o[2] = c[2]; o[3] = c[3];
    if (t->base_class == BF_ALPHA) o[0] = o[1] = o[2] = 0.0f;
    else if (t->base_class == BF_INTENSITY) o[1] = o[2] = o[3] = c[0];
    else if (lum) { o[1] = o[2] = c[0]; o[3] = 1.0f; }
    else if (t->base_class == BF_RGB) o[3] = 1.0f;
}

/* depth as the memory of a D24S8 target, {stencil, depth} in the word of a texel, R the top byte. The program reads it back (gl_ff.c, gen_sw_funcs) */
static void depth_texels(uint8_t *o, int w, int h, GLenum type, const void *pixels)
{
    float *z = (float *)malloc((size_t)w * h * sizeof(float));
    int i;
    if (!z) return;
    gl_unpack_depth(z, w, h, type, pixels);
    for (i = 0; i < w * h; i++) {
        double d = z[i] < 0.0f ? 0.0 : z[i] > 1.0f ? 1.0 : z[i];
        uint32_t v = (uint32_t)(d * 16777215.0 + 0.5);
        o[i * 4] = 0; o[i * 4 + 1] = (uint8_t)(v >> 16); o[i * 4 + 2] = (uint8_t)(v >> 8); o[i * 4 + 3] = (uint8_t)v;
    }
    free(z);
}

/* GL_DEPTH_STENCIL_EXT: the stencil in the top byte, where a D24S8 target keeps it */
static void ds_texels(uint8_t *o, int w, int h, const void *pixels)
{
    float *z = (float *)malloc((size_t)w * h * sizeof(float));
    uint32_t *s = (uint32_t *)malloc((size_t)w * h * 4);
    int i;
    if (z && s) {
        gl_unpack_ds(z, s, w, h, pixels);
        for (i = 0; i < w * h; i++) {
            double d = z[i] < 0.0f ? 0.0 : z[i] > 1.0f ? 1.0 : z[i];
            uint32_t v = (uint32_t)(d * 16777215.0 + 0.5);
            o[i * 4] = (uint8_t)s[i]; o[i * 4 + 1] = (uint8_t)(v >> 16); o[i * 4 + 2] = (uint8_t)(v >> 8); o[i * 4 + 3] = (uint8_t)v;
        }
    }
    free(z); free(s);
}

static int is_ds(GLenum format) { return format == GL_DEPTH_COMPONENT || format == GL_DEPTH_STENCIL_EXT; }

void APIENTRY glTexImage2D(GLenum target, GLint level, GLint internal, GLsizei wb, GLsizei hb, GLint border, GLenum format, GLenum type, const void *pixels)
{
    gl_tex *t;
    int face = face_of(target), bw, bh, oned = target == GL_TEXTURE_1D, w = wb - 2 * border, h = oned ? 1 : hb - 2 * border;
    uint8_t *rgba = NULL;
    if (target == GL_PROXY_TEXTURE_2D || target == GL_PROXY_TEXTURE_CUBE_MAP) { gl_tex_proxy(target, level, w, h, 1, (GLenum)internal); return; }
    FLUSH_VERTICES();
    if (level < 0 || level >= MAX_LEVELS || (border != 0 && border != 1) || w < 0 || h < 0 || w > MAX_TEX_SIZE || h > MAX_TEX_SIZE) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (is_ds(format) != (base_class_of((GLenum)internal) == BF_DEPTH) || (is_ds(format) && !oned && target != GL_TEXTURE_2D)) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (format == GL_DEPTH_STENCIL_EXT && type != GL_UNSIGNED_INT_24_8_EXT) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (!C->live || !(t = bound_tex(target))) return;
    if (C->bound_unpack) {
        if (!gl_pbo_fits(0, pixels, (size_t)wb * (oned ? 1 : hb) * (size_t)type_bytes(type, format))) { SET_ERROR(GL_INVALID_OPERATION); return; }
        pixels = gl_unpack_ptr(pixels);
    }
    bw = w << level; bh = oned ? 1 : h << level;                /* a 1D texture is an image one texel high */
    if (level == 0 || !t->handle) {
        if (bw > MAX_TEX_SIZE) bw = MAX_TEX_SIZE;
        if (bh > MAX_TEX_SIZE) bh = MAX_TEX_SIZE;
        if (ensure_image(t, bw ? bw : 1, bh ? bh : 1, t->target == GL_TEXTURE_CUBE_MAP ? 6 : 1, gl_hw_format((GLenum)internal, type)) < 0) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    }
    if (level == 0) { t->internal = (GLenum)internal; t->base_class = base_class_of((GLenum)internal); }
    tex_touch(t);
    free(t->comp[face][level]); t->comp[face][level] = NULL;
    t->level[face][level].defined = 1; t->level[face][level].w = w; t->level[face][level].h = h; t->level[face][level].border = border;
    if (level == 0 && !border && t->bimg) { g2r_image_destroy(D.r, t->bimg, t->bimg_base); t->bimg = 0; }
    if (!pixels) return;
    if (!w || !h) return;
    rgba = (uint8_t *)malloc((size_t)wb * (oned ? 1 : hb) * 4);
    if (!rgba) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    if (format == GL_DEPTH_STENCIL_EXT) ds_texels(rgba, wb, oned ? 1 : hb, pixels);
    else if (format == GL_DEPTH_COMPONENT) depth_texels(rgba, wb, oned ? 1 : hb, type, pixels);
    else {
        gl_unpack(rgba, wb, oned ? 1 : hb, format, type, pixels, (GLenum)internal);
        gl_quantize(t->hwfmt, rgba, (size_t)wb * (oned ? 1 : hb));
    }
    if (border) {
        int r;
        if (level == 0) border_image(t, rgba, wb, oned ? 1 : hb);
        for (r = 0; r < h; r++) memmove(rgba + (size_t)r * w * 4, rgba + ((size_t)(oned ? 0 : r + 1) * wb + 1) * 4, (size_t)w * 4);     /* the inside: what the mip chain holds */
    }
    if (is_ds(format)) g2_write_image(D.bd, t->handle, rgba, (size_t)w * h * 4, G2_TF_RGBA8888, 0, 0, (uint32_t)w, (uint32_t)h, (uint32_t)level, (uint32_t)face, 0, G2_FMT_RGBA8);
    else gl_img_write(t->handle, t->hwfmt, rgba, w, 0, 0, (uint32_t)w, (uint32_t)h, (uint32_t)level, (uint32_t)face);
    if (t->gen_mipmap && level == 0) {
        generate_levels(t, face, rgba);
        free(t->level0);
        t->level0 = rgba;
        return;
    }
    free(rgba);
}

void APIENTRY glTexImage1D(GLenum target, GLint level, GLint internal, GLsizei w, GLint border, GLenum format, GLenum type, const void *pixels)
{
    if (target == GL_PROXY_TEXTURE_1D) { gl_tex_proxy(target, level, w - 2 * border, 1, 1, (GLenum)internal); return; }
    if (target != GL_TEXTURE_1D) { SET_ERROR(GL_INVALID_ENUM); return; }
    glTexImage2D(GL_TEXTURE_1D, level, internal, w, 1, border, format, type, pixels);
}

void APIENTRY glTexSubImage1D(GLenum target, GLint level, GLint x, GLsizei w, GLenum format, GLenum type, const void *pixels)
{
    if (target != GL_TEXTURE_1D) { SET_ERROR(GL_INVALID_ENUM); return; }
    glTexSubImage2D(GL_TEXTURE_1D, level, x, 0, w, 1, format, type, pixels);
}

void APIENTRY glTexSubImage2D(GLenum target, GLint level, GLint x, GLint y, GLsizei w, GLsizei h, GLenum format, GLenum type, const void *pixels)
{
    gl_tex *t;
    int face = face_of(target), b;
    uint8_t *rgba;
    FLUSH_VERTICES();
    if (!C->live || !(t = bound_tex(target)) || !t->handle || !w || !h) return;
    if (is_ds(format) != (t->base_class == BF_DEPTH)) { SET_ERROR(GL_INVALID_OPERATION); return; }
    pixels = gl_unpack_ptr(pixels);
    tex_touch(t);
    rgba = (uint8_t *)malloc((size_t)w * h * 4);
    if (!rgba) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    if (format == GL_DEPTH_STENCIL_EXT) ds_texels(rgba, w, h, pixels);
    else if (format == GL_DEPTH_COMPONENT) depth_texels(rgba, w, h, type, pixels);
    else {
        gl_unpack(rgba, w, h, format, type, pixels, t->internal);
        gl_quantize(t->hwfmt, rgba, (size_t)w * h);
    }
    b = level >= 0 && level < MAX_LEVELS ? t->level[face][level].border : 0;
    if (b && level == 0 && t->bimg) gl_img_write(t->bimg, t->hwfmt, rgba, w, (uint32_t)(x + b), (uint32_t)(target == GL_TEXTURE_1D ? 0 : y + b), (uint32_t)w, (uint32_t)h, 0, 0);
    if (b && (x < 0 || y < 0 || x + w > t->level[face][level].w || y + h > t->level[face][level].h)) {     /* it touches the border: only the bordered image holds that */
        free(rgba);
        return;
    }
    if (is_ds(format)) g2_write_image(D.bd, t->handle, rgba, (size_t)w * h * 4, G2_TF_RGBA8888, (uint32_t)x, (uint32_t)y, (uint32_t)w, (uint32_t)h, (uint32_t)level, (uint32_t)face, 0, G2_FMT_RGBA8);
    else gl_img_write(t->handle, t->hwfmt, rgba, w, (uint32_t)x, (uint32_t)y, (uint32_t)w, (uint32_t)h, (uint32_t)level, (uint32_t)face);
    if (t->gen_mipmap && level == 0 && t->level0) {
        int r;
        for (r = 0; r < h; r++) memcpy(t->level0 + (size_t)(y + r) * t->w * 4 + (size_t)x * 4, rgba + (size_t)r * w * 4, (size_t)w * 4);
        generate_levels(t, face, t->level0);
    }
    free(rgba);
}

void APIENTRY glCopyTexSubImage2D(GLenum target, GLint level, GLint xo, GLint yo, GLint x, GLint y, GLsizei w, GLsizei h)
{
    gl_tex *t;
    g2r_target tg;
    double kx, ky;
    int sx, sy, sw, sh, rgb_fill;
    FLUSH_VERTICES();
    if (!C->live || !(t = bound_tex(target)) || !t->handle) return;
    gl_bind_target();
    gl_read_target(&tg);
    gl_scale(&kx, &ky);
    sx = (int)floor(x * kx); sy = (int)floor(y * ky); sw = (int)ceil((x + w) * kx) - sx; sh = (int)ceil((y + h) * ky) - sy;
    g2r_end_pass(D.r);
    if (t->base_class == BF_DEPTH) {                            /* a depth texture is the memory of a D24S8 target: the board's copy takes colour formats, so by the PC */
        uint8_t *ds = (uint8_t *)malloc((size_t)w * h * 4);
        if (!ds) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
        if (tg.ds && gl_read_rect(&tg, tg.ds, G2_TF_RAW, G2_FMT_D24S8, x, y, w, h, ds) == 0) {
            if (tg.depth16) { size_t i; uint32_t *v = (uint32_t *)ds; for (i = 0; i < (size_t)w * h; i++) v[i] = (v[i] & 0xFF000000u) | (uint32_t)((v[i] & 0xFFFF) * 16777215.0 / 65535.0 + 0.5); }
            tex_touch(t);
            g2_write_image(D.bd, t->handle, ds, (size_t)w * h * 4, G2_TF_RAW, (uint32_t)xo, (uint32_t)yo, (uint32_t)w, (uint32_t)h, (uint32_t)level, (uint32_t)face_of(target), 0, G2_FMT_RGBA8);
        }
        free(ds);
        return;
    }
    {
        uint32_t sf = gl_tg_fmt(&tg), df = t->hwfmt;
        int k = base_kind(t->internal);
        int board = (sf == df || ((sf == G2_FMT_RGBA8 || sf == G2_FMT_RGB565) && (df == G2_FMT_RGBA8 || df == G2_FMT_RGB565))) && (k == K_RGBA || (k == K_RGB && (df == G2_FMT_RGB565 || df == G2_FMT_RGBA8)));
        rgb_fill = board && k == K_RGB && df == G2_FMT_RGBA8;    /* RGB keeps no alpha: it reads 1 */
        if (!board) {                                           /* table 3.15: what the internal format keeps, in a format the board does not convert to */
            uint8_t *px = (uint8_t *)malloc((size_t)w * h * 4), *o = (uint8_t *)malloc((size_t)w * h * 4);
            int al = C->un_align, ur = C->un_row, us = C->un_skip_px, uz = C->un_skip_row, tr = C->px_active;
            if (!px || !o) { free(px); free(o); SET_ERROR(GL_OUT_OF_MEMORY); return; }
            gl_read_rect(&tg, tg.color, G2_TF_RGBA8888, sf, x, y, w, h, px);
            C->un_align = 4; C->un_row = C->un_skip_px = C->un_skip_row = 0; C->px_active = 0;
            gl_unpack(o, w, h, GL_RGBA, GL_UNSIGNED_BYTE, px, t->internal);
            C->un_align = al; C->un_row = ur; C->un_skip_px = us; C->un_skip_row = uz; C->px_active = tr;
            tex_touch(t);
            gl_img_write(t->handle, df, o, w, (uint32_t)xo, (uint32_t)yo, (uint32_t)w, (uint32_t)h, (uint32_t)level, (uint32_t)face_of(target));
            free(px); free(o);
            return;
        }
    }
    tex_touch(t);
    g2_copy_image(D.bd, tg.color, t->handle, (uint32_t)sx, (uint32_t)sy, (uint32_t)sw, (uint32_t)sh, (uint32_t)xo, (uint32_t)yo, (uint32_t)w, (uint32_t)h, 0, 0, 0, 0, 0, 0, 0, (uint32_t)level,
                  (uint32_t)face_of(target));
    if (rgb_fill) g2_fill_image(D.bd, t->handle, G2_TF_RAW, 0x000000FFu, (uint32_t)xo, (uint32_t)yo, (uint32_t)w, (uint32_t)h, G2_MASK_A, (uint32_t)level, (uint32_t)face_of(target));
}

void APIENTRY glCopyTexImage2D(GLenum target, GLint level, GLenum internal, GLint x, GLint y, GLsizei w, GLsizei h, GLint border)
{
    if (border) {                                               /* the border comes from the framebuffer too: through the path that takes a border */
        uint8_t *px = (uint8_t *)malloc((size_t)w * h * 4);
        int al = C->un_align, pa = C->pk_align, pr = C->pk_row, ps = C->pk_skip_px, pz = C->pk_skip_row, ur = C->un_row, us = C->un_skip_px, uz = C->un_skip_row, tr = C->px_active;
        if (!px) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
        C->un_align = C->pk_align = 4; C->pk_row = C->pk_skip_px = C->pk_skip_row = C->un_row = C->un_skip_px = C->un_skip_row = 0; C->px_active = 0;
        glReadPixels(x, y, w, h, GL_RGBA, GL_UNSIGNED_BYTE, px);
        GL_FROM_CLIENT(glTexImage2D(target, level, (GLint)internal, w, h, border, GL_RGBA, GL_UNSIGNED_BYTE, px));
        C->un_align = al; C->pk_align = pa; C->pk_row = pr; C->pk_skip_px = ps; C->pk_skip_row = pz; C->un_row = ur; C->un_skip_px = us; C->un_skip_row = uz; C->px_active = tr;
        free(px);
        return;
    }
    GL_FROM_CLIENT(glTexImage2D(target, level, (GLint)internal, w, h, border, base_class_of(internal) == BF_DEPTH ? GL_DEPTH_COMPONENT : GL_RGBA, GL_UNSIGNED_BYTE, NULL));
    glCopyTexSubImage2D(target, level, 0, 0, x, y, w, h);
}

void APIENTRY glCopyTexSubImage1D(GLenum target, GLint level, GLint xo, GLint x, GLint y, GLsizei w)
{
    if (target != GL_TEXTURE_1D) { SET_ERROR(GL_INVALID_ENUM); return; }
    glCopyTexSubImage2D(GL_TEXTURE_1D, level, xo, 0, x, y, w, 1);
}

void APIENTRY glCopyTexImage1D(GLenum target, GLint level, GLenum internal, GLint x, GLint y, GLsizei w, GLint border)
{
    if (target != GL_TEXTURE_1D) { SET_ERROR(GL_INVALID_ENUM); return; }
    glCopyTexImage2D(GL_TEXTURE_1D, level, internal, x, y, w, 1, border);
}

void APIENTRY glGenTextures(GLsizei n, GLuint *names)
{
    GLsizei i;
    for (i = 0; i < n; i++) {
        while (node_of(g_next_tex, 0)) g_next_tex++;
        names[i] = g_next_tex++;
    }
}

void APIENTRY glDeleteTextures(GLsizei n, const GLuint *names)
{
    GLsizei i;
    FLUSH_VERTICES();
    for (i = 0; i < n; i++) {
        tnode **pp = &g_nodes[names[i] % T_BUCKETS];
        int u;
        while (*pp) {
            tnode *t = *pp;
            if (t->t.name == names[i]) {
                if (t->t.handle && C->live) {
                    if (t->used_pass == g2r_pass_id(D.r)) g2r_end_pass(D.r);
                    gl_tex_view_drop(&t->t);
                    gl_tex_rt_drop(&t->t);
                    g2r_image_destroy(D.r, t->t.handle, t->t.base);
                    if (t->t.bimg) g2r_image_destroy(D.r, t->t.bimg, t->t.bimg_base);
                }
                {
                    int f, l;
                    for (f = 0; f < 6; f++) for (l = 0; l < MAX_LEVELS; l++) free(t->t.comp[f][l]);
                }
                gl_tex3d_free(&t->t);
                free(t->t.level0);
                *pp = t->next;
                free(t);
                break;
            }
            pp = &t->next;
        }
        for (u = 0; u < MAX_UNITS; u++) {
            if (C->unit[u].bound_2d == names[i]) C->unit[u].bound_2d = 0;
            if (C->unit[u].bound_cube == names[i]) C->unit[u].bound_cube = 0;
            if (C->unit[u].bound_1d == names[i]) C->unit[u].bound_1d = 0;
            if (C->unit[u].bound_3d == names[i]) C->unit[u].bound_3d = 0;
        }
    }
}

GLboolean APIENTRY glIsTexture(GLuint name) { return name && node_of(name, 0) ? GL_TRUE : GL_FALSE; }

void APIENTRY glBindTexture(GLenum target, GLuint name)
{
    gl_unit *u = &C->unit[C->active_unit];
    gl_tex *t;
    LREC_I(LOP_BINDTEX, target, name);
    FLUSH_VERTICES();
    t = gl_tex_lookup(name, target, 1);
    if (!t) return;
    t->target = target;
    if (target == GL_TEXTURE_CUBE_MAP) u->bound_cube = name;
    else if (target == GL_TEXTURE_2D) u->bound_2d = name;
    else if (target == GL_TEXTURE_1D) u->bound_1d = name;
    else if (target == GL_TEXTURE_3D) u->bound_3d = name;
}

static void tex_param(GLenum target, GLenum p, float v, const float *fv)
{
    gl_tex *t = bound_tex(target);
    GLenum e = (GLenum)v;
    FLUSH_VERTICES();
    if (!t) return;
    switch (p) {
    case GL_TEXTURE_BORDER_COLOR: {
        int c;
        if (!fv) { SET_ERROR(GL_INVALID_ENUM); return; }
        for (c = 0; c < 4; c++) t->border_color[c] = fv[c] < 0.0f ? 0.0f : fv[c] > 1.0f ? 1.0f : fv[c];
        break;
    }
    case GL_TEXTURE_MIN_FILTER: t->min_filter = e; break;
    case GL_TEXTURE_MAG_FILTER: t->mag_filter = e; break;
    case GL_TEXTURE_WRAP_S: t->wrap_s = e; break;
    case GL_TEXTURE_WRAP_T: t->wrap_t = e; break;
    case GL_TEXTURE_WRAP_R: t->wrap_r = e; break;
    case GL_TEXTURE_COMPARE_MODE: t->compare_mode = e; break;
    case GL_TEXTURE_COMPARE_FUNC: t->compare_func = e; break;
    case GL_DEPTH_TEXTURE_MODE: t->depth_mode = e; break;
    case GL_TEXTURE_MAX_LEVEL: if (v < 0) { SET_ERROR(GL_INVALID_VALUE); return; } t->max_level = (GLint)v; break;
    case GL_TEXTURE_BASE_LEVEL: if (v < 0) { SET_ERROR(GL_INVALID_VALUE); return; } t->base_level = (GLint)v; break;
    case GL_TEXTURE_MIN_LOD: t->min_lod = v; break;
    case GL_TEXTURE_MAX_LOD: t->max_lod = v; break;
    case GL_TEXTURE_LOD_BIAS: t->lod_bias = v; break;
    case GL_GENERATE_MIPMAP: t->gen_mipmap = v != 0.0f; break;
    case GL_TEXTURE_PRIORITY: t->priority = v; break;
    default: SET_ERROR(GL_INVALID_ENUM); break;
    }
}

void APIENTRY glTexParameteri(GLenum t, GLenum p, GLint v) { tex_param(t, p, (float)v, NULL); }
void APIENTRY glTexParameterf(GLenum t, GLenum p, GLfloat v) { tex_param(t, p, v, NULL); }
void APIENTRY glTexParameteriv(GLenum t, GLenum p, const GLint *v)
{
    if (p == GL_TEXTURE_BORDER_COLOR) {
        float f[4];
        int c;
        for (c = 0; c < 4; c++) f[c] = (float)((2.0 * v[c] + 1.0) / 4294967295.0);
        tex_param(t, p, f[0], f);
        return;
    }
    tex_param(t, p, (float)v[0], NULL);
}
void APIENTRY glTexParameterfv(GLenum t, GLenum p, const GLfloat *v) { tex_param(t, p, v[0], v); }

void APIENTRY glGetTexParameteriv(GLenum target, GLenum p, GLint *v)
{
    gl_tex *t = bound_tex(target);
    if (!t) return;
    switch (p) {
    case GL_TEXTURE_MIN_FILTER: *v = (GLint)t->min_filter; break;
    case GL_TEXTURE_MAG_FILTER: *v = (GLint)t->mag_filter; break;
    case GL_TEXTURE_WRAP_S: *v = (GLint)t->wrap_s; break;
    case GL_TEXTURE_WRAP_T: *v = (GLint)t->wrap_t; break;
    case GL_TEXTURE_WRAP_R: *v = (GLint)t->wrap_r; break;
    case GL_TEXTURE_COMPARE_MODE: *v = (GLint)t->compare_mode; break;
    case GL_TEXTURE_COMPARE_FUNC: *v = (GLint)t->compare_func; break;
    case GL_DEPTH_TEXTURE_MODE: *v = (GLint)t->depth_mode; break;
    case GL_TEXTURE_MAX_LEVEL: *v = t->max_level; break;
    case GL_TEXTURE_BASE_LEVEL: *v = t->base_level; break;
    case GL_TEXTURE_MIN_LOD: *v = (GLint)t->min_lod; break;
    case GL_TEXTURE_MAX_LOD: *v = (GLint)t->max_lod; break;
    case GL_TEXTURE_LOD_BIAS: *v = (GLint)t->lod_bias; break;
    case GL_GENERATE_MIPMAP: *v = t->gen_mipmap; break;
    case GL_TEXTURE_PRIORITY: *v = (GLint)(t->priority * 2147483647.0); break;
    case GL_TEXTURE_RESIDENT: *v = GL_TRUE; break;
    default: *v = 0; SET_ERROR(GL_INVALID_ENUM); break;
    }
}

void APIENTRY glGetTexParameterfv(GLenum target, GLenum p, GLfloat *v)
{
    GLint i = 0;
    gl_tex *t = bound_tex(target);
    if (p == GL_TEXTURE_BORDER_COLOR) { if (t) memcpy(v, t->border_color, 16); return; }
    if (t && p == GL_TEXTURE_MIN_LOD) { *v = t->min_lod; return; }
    if (t && p == GL_TEXTURE_MAX_LOD) { *v = t->max_lod; return; }
    if (t && p == GL_TEXTURE_LOD_BIAS) { *v = t->lod_bias; return; }
    if (t && p == GL_TEXTURE_PRIORITY) { *v = t->priority; return; }
    glGetTexParameteriv(target, p, &i);
    *v = (GLfloat)i;
}

/* §3.8.11: a proxy image is a question: does it fit, and the answer is in the state of the proxy target */
static struct { int w, h, d; GLenum internal; } g_proxy[4][MAX_LEVELS];

static int proxy_of(GLenum t) { return t == GL_PROXY_TEXTURE_2D ? 0 : t == GL_PROXY_TEXTURE_1D ? 1 : t == GL_PROXY_TEXTURE_3D ? 2 : t == GL_PROXY_TEXTURE_CUBE_MAP ? 3 : -1; }

void gl_tex_proxy(GLenum target, int level, int w, int h, int d, GLenum internal)
{
    int i = proxy_of(target), fit;
    if (level < 0 || level >= MAX_LEVELS) { SET_ERROR(GL_INVALID_VALUE); return; }
    fit = i == 2 ? gl_tex3d_fit(level, w, h, d, 0) : w >= 0 && h >= 0 && d >= 0 && w << level <= MAX_TEX_SIZE && h << level <= MAX_TEX_SIZE && d << level <= MAX_TEX_SIZE;
    g_proxy[i][level].w = fit ? w : 0; g_proxy[i][level].h = fit ? h : 0; g_proxy[i][level].d = fit ? d : 0; g_proxy[i][level].internal = fit ? internal : 0;
}

static int proxy_get(GLenum target, int level, GLenum p, GLint *v)
{
    int i = proxy_of(target);
    if (i < 0) return 0;
    if (level < 0 || level >= MAX_LEVELS) { SET_ERROR(GL_INVALID_VALUE); return 1; }
    switch (p) {
    case GL_TEXTURE_WIDTH: *v = g_proxy[i][level].w; break;
    case GL_TEXTURE_HEIGHT: *v = g_proxy[i][level].h; break;
    case GL_TEXTURE_DEPTH: *v = g_proxy[i][level].d; break;
    case GL_TEXTURE_INTERNAL_FORMAT: *v = (GLint)g_proxy[i][level].internal; break;
    default: *v = 0; break;
    }
    return 1;
}

void APIENTRY glGetTexLevelParameteriv(GLenum target, GLint level, GLenum p, GLint *v)
{
    gl_tex *t;
    *v = 0;
    if (proxy_get(target, level, p, v)) return;
    t = bound_tex(target);
    if (!t || level >= MAX_LEVELS) return;
    switch (p) {
    case GL_TEXTURE_WIDTH: *v = t->level[face_of(target)][level].w + 2 * t->level[face_of(target)][level].border; break;
    case GL_TEXTURE_HEIGHT: *v = t->level[face_of(target)][level].h + (target == GL_TEXTURE_1D ? 0 : 2 * t->level[face_of(target)][level].border); break;
    case GL_TEXTURE_DEPTH: *v = target == GL_TEXTURE_3D ? t->level[0][level].d : 1; break;
    case GL_TEXTURE_DEPTH_SIZE: *v = t->base_class == BF_DEPTH ? 24 : 0; break;
    case GL_TEXTURE_BORDER: *v = t->level[face_of(target)][level].border; break;
    case GL_TEXTURE_INTERNAL_FORMAT: *v = (GLint)t->internal; break;
    case GL_TEXTURE_RED_SIZE: case GL_TEXTURE_GREEN_SIZE: case GL_TEXTURE_BLUE_SIZE: case GL_TEXTURE_ALPHA_SIZE: case GL_TEXTURE_LUMINANCE_SIZE: case GL_TEXTURE_INTENSITY_SIZE: {
        int k = base_kind(t->internal), c = p == GL_TEXTURE_GREEN_SIZE ? 1 : p == GL_TEXTURE_BLUE_SIZE ? 2 : p == GL_TEXTURE_ALPHA_SIZE ? 3 : 0;
        int has = k == K_DEPTH ? 0 : p == GL_TEXTURE_LUMINANCE_SIZE ? k == K_L || k == K_LA : p == GL_TEXTURE_INTENSITY_SIZE ? k == K_I : p == GL_TEXTURE_ALPHA_SIZE ? k == K_A || k == K_LA || k == K_RGBA : k == K_RGB || k == K_RGBA;
        *v = has ? gl_fmt_bits(t->hwfmt, c) : 0;
        break;
    }
    case GL_TEXTURE_COMPRESSED: *v = t->comp[face_of(target)][level] != NULL; break;
    case GL_TEXTURE_COMPRESSED_IMAGE_SIZE:
        if (!t->comp[face_of(target)][level]) SET_ERROR(GL_INVALID_OPERATION);
        else *v = t->comp_size[face_of(target)][level];
        break;
    default: break;
    }
}

void APIENTRY glGetTexLevelParameterfv(GLenum target, GLint level, GLenum p, GLfloat *v) { GLint i = 0; glGetTexLevelParameteriv(target, level, p, &i); *v = (GLfloat)i; }

/* table 6.1: what the components of a stored texel are, then packed as glReadPixels without the transfer */
void APIENTRY glGetTexImage(GLenum target, GLint level, GLenum format, GLenum type, void *pixels)
{
    gl_tex *t = bound_tex(target);
    int w, h, b, lum, face = face_of(target), depth = 1, tf, n3 = target == GL_TEXTURE_3D;
    size_t i, n;
    uint8_t *rgba;
    GLenum e = is_ds(format) ? (gl_ds_type(format, type) ? 0 : GL_INVALID_ENUM) : pack_error(format, type);
    FLUSH_VERTICES();
    if (level < 0 || level >= MAX_LEVELS) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (e) { SET_ERROR(e); return; }
    if (!t || !t->handle || !C->live || !t->level[face][level].defined) return;
    if (is_ds(format) != (t->base_class == BF_DEPTH)) { SET_ERROR(GL_INVALID_OPERATION); return; }
    pixels = gl_pack_ptr(pixels);
    gl_tex_resolve(t);
    w = t->level[face][level].w; h = t->level[face][level].h;
    b = t->level[face][level].border;
    if (b && level == 0 && t->bimg) { w = t->bimg_w; h = t->bimg_h; }
    if (n3) { w += 2 * b; h += 2 * b; depth = t->level[0][level].d + 2 * b; }
    n = (size_t)w * h * depth;
    rgba = (uint8_t *)malloc(n * 4);
    if (!rgba) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    tf = t->base_class == BF_DEPTH ? G2_TF_RAW : G2_TF_RGBA8888;    /* {stencil, depth}, as gl_read_rect gives a depth target */
    if (n3) memcpy(rgba, t->px3[level], n * 4);                 /* the PC has it */
    else {
        g2r_finish(D.r);
        if (t->base_class == BF_DEPTH) g2_read_image(D.bd, b && level == 0 && t->bimg ? t->bimg : t->handle, (uint32_t)tf, 0, 0, (uint32_t)w, (uint32_t)h, b && level == 0 && t->bimg ? 0 : (uint32_t)level, b && level == 0 && t->bimg ? 0 : (uint32_t)face, G2_FMT_RGBA8, 0, 0, rgba);
        else if (b && level == 0 && t->bimg) gl_img_read(t->bimg, t->hwfmt, 0, 0, (uint32_t)w, (uint32_t)h, 0, 0, rgba);
        else gl_img_read(t->handle, t->hwfmt, 0, 0, (uint32_t)w, (uint32_t)h, (uint32_t)level, (uint32_t)face, rgba);
    }
    if (t->base_class == BF_DEPTH) {
        pack_ds((const uint32_t *)rgba, w, h, format, type, pixels, 0);
        free(rgba);
        return;
    }
    lum = t->internal == GL_LUMINANCE || t->internal == 1 || t->internal == GL_LUMINANCE_ALPHA || t->internal == 2 || (t->internal >= GL_LUMINANCE4 && t->internal <= GL_LUMINANCE16_ALPHA16) || t->base_class == BF_INTENSITY;
    if (lum) for (i = 0; i < n; i++) rgba[i * 4 + 1] = rgba[i * 4 + 2] = 0;
    if (t->base_class == BF_INTENSITY) for (i = 0; i < n; i++) rgba[i * 4 + 3] = 255;
    if (n3) {                                                   /* a slice at a time, as GL_PACK_IMAGE_HEIGHT and GL_PACK_SKIP_IMAGES say */
        int z, sr = C->pk_skip_row, ih = C->pk_image_h ? C->pk_image_h : h;
        for (z = 0; z < depth; z++) {
            C->pk_skip_row = sr + (C->pk_skip_images + z) * ih;
            gl_pack(rgba + (size_t)z * w * h * 4, w, h, format, type, pixels);
        }
        C->pk_skip_row = sr;
    } else gl_pack(rgba, w, h, format, type, pixels);
    free(rgba);
}

/* ---------------------------------------------------------------- glReadPixels */
void APIENTRY glReadPixels(GLint x, GLint y, GLsizei w, GLsizei h, GLenum format, GLenum type, void *pixels)
{
    g2r_target tg;
    uint8_t *tmp;
    GLenum e;
    FLUSH_VERTICES();
    if (w < 0 || h < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (format == GL_COLOR_INDEX) { SET_ERROR(GL_INVALID_OPERATION); return; }                  /* an RGBA context */
    if (format == GL_DEPTH_COMPONENT || format == GL_STENCIL_INDEX || format == GL_DEPTH_STENCIL_EXT) e = gl_ds_type(format, type) ? 0 : packed_type(type) ? GL_INVALID_OPERATION : GL_INVALID_ENUM;
    else e = pack_error(format, type);
    if (e) { SET_ERROR(e); return; }
    if (!C->live || !w || !h) return;
    if (C->bound_pack) {
        if (!gl_pbo_fits(1, pixels, (size_t)w * h * (size_t)type_bytes(type, format))) { SET_ERROR(GL_INVALID_OPERATION); return; }
        pixels = gl_pack_ptr(pixels);
    }
    gl_bind_target();
    gl_read_target(&tg);
    g2r_finish(D.r);
    tmp = (uint8_t *)malloc((size_t)w * h * 4);
    if (!tmp) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    if (format == GL_DEPTH_COMPONENT || format == GL_STENCIL_INDEX || format == GL_DEPTH_STENCIL_EXT) {
        if (!tg.ds) SET_ERROR(GL_INVALID_OPERATION);
        else {
            gl_read_rect(&tg, tg.ds, G2_TF_RAW, G2_FMT_D24S8, x, y, w, h, tmp);
            pack_ds((const uint32_t *)tmp, w, h, format, type, pixels, tg.depth16);
        }
    } else {
        gl_read_rect(&tg, tg.color, G2_TF_RGBA8888, gl_tg_fmt(&tg), x, y, w, h, tmp);
        gl_px_rows(tmp, w, h);
        gl_pack(tmp, w, h, format, type, pixels);
    }
    free(tmp);
}

/* ---------------------------------------------------------------- buffer objects: PC copies, gathered per draw like client arrays */
#define B_BUCKETS 256
typedef struct bnode { gl_buffer b; struct bnode *next; } bnode;
static bnode *g_bufs[B_BUCKETS];

gl_buffer *gl_buffer_lookup(GLuint name)
{
    bnode *n;
    for (n = g_bufs[name % B_BUCKETS]; n; n = n->next) if (n->b.name == name) return &n->b;
    n = (bnode *)calloc(1, sizeof *n);
    if (!n) return NULL;
    n->b.name = name;
    n->next = g_bufs[name % B_BUCKETS];
    g_bufs[name % B_BUCKETS] = n;
    return &n->b;
}

static gl_buffer *target_buffer(GLenum target)
{
    GLuint n = target == GL_ELEMENT_ARRAY_BUFFER ? C->bound_element : target == GL_PIXEL_PACK_BUFFER ? C->bound_pack : target == GL_PIXEL_UNPACK_BUFFER ? C->bound_unpack : C->bound_array;
    return n ? gl_buffer_lookup(n) : NULL;
}

/* GL_ARB_pixel_buffer_object: a buffer is a PC copy, so a transfer through one is an offset into it */
const void *gl_unpack_ptr(const void *pixels)
{
    gl_buffer *b = C->bound_unpack ? gl_buffer_lookup(C->bound_unpack) : NULL;
    return b ? b->data + (size_t)pixels : pixels;
}

void *gl_pack_ptr(void *pixels)
{
    gl_buffer *b = C->bound_pack ? gl_buffer_lookup(C->bound_pack) : NULL;
    return b ? b->data + (size_t)pixels : pixels;
}

int gl_pbo_fits(int pack, const void *pixels, size_t bytes)
{
    gl_buffer *b = gl_buffer_lookup(pack ? C->bound_pack : C->bound_unpack);
    return b && b->data && !b->mapped && (size_t)pixels + bytes <= (size_t)b->size;
}

void APIENTRY glGenBuffers(GLsizei n, GLuint *b) { GLsizei i; for (i = 0; i < n; i++) b[i] = g_next_buf++; }
void APIENTRY glBindBuffer(GLenum t, GLuint b) { FLUSH_VERTICES(); if (t == GL_ELEMENT_ARRAY_BUFFER) C->bound_element = b; else if (t == GL_PIXEL_PACK_BUFFER) C->bound_pack = b; else if (t == GL_PIXEL_UNPACK_BUFFER) C->bound_unpack = b; else C->bound_array = b; }
void APIENTRY glDeleteBuffers(GLsizei n, const GLuint *b)
{
    GLsizei i;
    for (i = 0; i < n; i++) {
        bnode **pp = &g_bufs[b[i] % B_BUCKETS];
        while (*pp) {
            bnode *o = *pp;
            if (o->b.name == b[i]) { g2r_keep_changed(D.r, &o->b.keep); free(o->b.data); *pp = o->next; free(o); break; }
            pp = &o->next;
        }
        if (C->bound_array == b[i]) C->bound_array = 0;
        if (C->bound_element == b[i]) C->bound_element = 0;
        if (C->bound_pack == b[i]) C->bound_pack = 0;
        if (C->bound_unpack == b[i]) C->bound_unpack = 0;
    }
}
GLboolean APIENTRY glIsBuffer(GLuint b) { bnode *n; for (n = g_bufs[b % B_BUCKETS]; n; n = n->next) if (n->b.name == b) return GL_TRUE; return GL_FALSE; }
void APIENTRY glBufferData(GLenum t, GLsizeiptr size, const void *data, GLenum usage)
{
    gl_buffer *o = target_buffer(t);
    FLUSH_VERTICES();
    if (!o) { SET_ERROR(GL_INVALID_OPERATION); return; }
    g2r_keep_changed(D.r, &o->keep);
    free(o->data);
    o->data = (uint8_t *)malloc(size ? (size_t)size : 1);
    if (!o->data) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    if (data) memcpy(o->data, data, (size_t)size);
    o->size = size; o->usage = usage;
}
void APIENTRY glBufferSubData(GLenum t, GLintptr off, GLsizeiptr size, const void *data)
{
    gl_buffer *o = target_buffer(t);
    FLUSH_VERTICES();
    if (!o || !o->data || off + size > o->size) { SET_ERROR(GL_INVALID_VALUE); return; }
    g2r_keep_changed(D.r, &o->keep);
    memcpy(o->data + off, data, (size_t)size);
}
void APIENTRY glGetBufferSubData(GLenum t, GLintptr off, GLsizeiptr size, void *data)
{
    gl_buffer *o = target_buffer(t);
    if (!o || !o->data || off + size > o->size) { SET_ERROR(GL_INVALID_VALUE); return; }
    memcpy(data, o->data + off, (size_t)size);
}
void *APIENTRY glMapBuffer(GLenum t, GLenum a) { gl_buffer *o = target_buffer(t); FLUSH_VERTICES(); if (!o) return NULL; if (a != GL_READ_ONLY) g2r_keep_changed(D.r, &o->keep); o->mapped = 1; return o->data; }
GLboolean APIENTRY glUnmapBuffer(GLenum t) { gl_buffer *o = target_buffer(t); if (o) o->mapped = 0; return GL_TRUE; }
void APIENTRY glGetBufferParameteriv(GLenum t, GLenum p, GLint *v)
{
    gl_buffer *o = target_buffer(t);
    *v = 0;
    if (!o) return;
    if (p == GL_BUFFER_SIZE) *v = (GLint)o->size;
    else if (p == GL_BUFFER_USAGE) *v = (GLint)o->usage;
    else if (p == GL_BUFFER_MAPPED) *v = o->mapped;
}

/* ---------------------------------------------------------------- framebuffer objects (EXT) */
#define F_MAX 256
static gl_fbo g_fbos[F_MAX];
static gl_renderbuffer g_rbs[F_MAX];

gl_fbo *gl_fbo_lookup(GLuint name) { return name && name < F_MAX ? &g_fbos[name] : NULL; }

static const gl_level *att_level(const gl_tex *x, int face, int level) { return &x->level[x->target == GL_TEXTURE_CUBE_MAP ? face : 0][level]; }

/* B renders here itself */
static int tex_direct(const gl_tex *x, int level) { return !level && x->layers == 1 && x->target != GL_TEXTURE_3D && !((x->w * gl_fmt_bpp(x->hwfmt)) % 32); }

/* the target of a framebuffer object. bind: a draw is coming, render images come into being and the texture they stand for is theirs from now on */
static GLenum fbo_status(gl_fbo *f, g2r_target *t, int bind)
{
    gl_tex *dt = NULL;
    const gl_level *dl = NULL;
    memset(t, 0, sizeof *t);
    t->bits_r = t->bits_g = t->bits_b = t->bits_a = 8;
    if (f->depth_type == GL_TEXTURE || f->stencil_type == GL_TEXTURE) {
        dt = gl_tex_lookup(f->depth_type == GL_TEXTURE ? f->depth : f->stencil, GL_TEXTURE_2D, 0);
        if (!dt || !dt->handle || dt->base_class != BF_DEPTH || dt->target != GL_TEXTURE_2D) return GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT;
        dl = att_level(dt, 0, f->depth_level);
        if (!dl->defined || !dl->w || !dl->h) return GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT;
    }
    if (f->color_type == GL_TEXTURE) {
        gl_tex *x = gl_tex_lookup(f->color, GL_TEXTURE_2D, 0);
        const gl_level *l = x ? att_level(x, f->color_face, f->color_level) : NULL;
        int layer = x && x->target == GL_TEXTURE_3D ? f->color_z : f->color_face;
        if (!x || !x->handle || !l->defined || !l->w || !l->h || x->base_class == BF_DEPTH) return GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT;
        if (x->target == GL_TEXTURE_3D && f->color_z >= l->d) return GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT;
        if (l->w > 1280 || l->border) return GL_FRAMEBUFFER_UNSUPPORTED_EXT;
        t->width = (uint32_t)l->w; t->height = (uint32_t)l->h;
        gl_tg_bits(t, x->hwfmt);
        t->color = tex_direct(x, f->color_level) ? x->handle : !bind ? 1 : rt_bind(x, f->color_level, layer, l->w, l->h, 0);
        if (!t->color) return GL_FRAMEBUFFER_UNSUPPORTED_EXT;
    } else if (f->color_type == GL_RENDERBUFFER_EXT) {
        gl_renderbuffer *b = &g_rbs[f->color];
        if (!b->handle || b->internal == GL_DEPTH_COMPONENT || (b->internal >= GL_DEPTH_COMPONENT16 && b->internal <= GL_DEPTH_COMPONENT32) || b->internal == GL_DEPTH24_STENCIL8_EXT || b->internal == GL_STENCIL_INDEX8_EXT) return GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT;
        t->color = b->handle; t->width = (uint32_t)b->w; t->height = (uint32_t)b->h;
        gl_tg_bits(t, b->hwfmt);
    } else if (f->depth_type || f->stencil_type) {              /* depth alone, a shadow map with GL_DRAW_BUFFER GL_NONE: B needs a colour target, and nothing reads this one */
        int w, h;
        if (dt) { w = dl->w; h = dl->h; }
        else {
            GLuint r = f->depth_type ? f->depth : f->stencil;
            if (!g_rbs[r].handle) return GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT;
            w = g_rbs[r].w; h = g_rbs[r].h;
        }
        if (w > 1280) return GL_FRAMEBUFFER_UNSUPPORTED_EXT;
        if (f->blind && (f->blind_w != w || f->blind_h != h)) { g2r_end_pass(D.r); g2r_image_destroy(D.r, f->blind, f->blind_base); f->blind = 0; }
        if (!f->blind) {
            if (g2r_image_create(D.r, &f->blind, &f->blind_base, (uint32_t)w, (uint32_t)h, G2_FMT_RGBA8, 1, 1, 1, G2_USE_RT) < 0) { f->blind = 0; return GL_FRAMEBUFFER_UNSUPPORTED_EXT; }
            f->blind_w = w; f->blind_h = h;
        }
        t->color = f->blind; t->width = (uint32_t)w; t->height = (uint32_t)h;
    } else {
        return GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT;
    }
    if (f->depth_type && f->stencil_type && (f->depth_type != f->stencil_type || f->depth != f->stencil)) return GL_FRAMEBUFFER_UNSUPPORTED_EXT;     /* one word holds both */
    if (f->depth_type == GL_RENDERBUFFER_EXT || f->stencil_type == GL_RENDERBUFFER_EXT) {
        gl_renderbuffer *b = &g_rbs[f->depth_type ? f->depth : f->stencil];
        if (!b->handle) return GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT;
        if ((uint32_t)b->w != t->width || (uint32_t)b->h != t->height) return GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT;
        t->ds = b->handle;
        t->depth16 = b->depth16;
    } else if (dt) {                                            /* a depth texture is the memory of a D24S8 target: the same bytes under a second name, which needs B's pitch */
        if ((uint32_t)dl->w != t->width || (uint32_t)dl->h != t->height) return GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT;
        if (dl->w > 1280) return GL_FRAMEBUFFER_UNSUPPORTED_EXT;
        if (tex_direct(dt, f->depth_level) && (uint32_t)dt->w * 4 == g2r_ds_pitch((uint32_t)dt->w)) {     /* its tight rows are long enough */
            if (!dt->ds_alias) {
                uint16_t hd;
                if (g2r_image_alias(D.r, &hd, dt->base, (uint32_t)dt->w, (uint32_t)dt->h, G2_FMT_D24S8, (uint32_t)dt->w * 4, 1, 1, G2_USE_RT) < 0) return GL_FRAMEBUFFER_UNSUPPORTED_EXT;
                dt->ds_alias = hd;
            }
            t->ds = dt->ds_alias;
        } else if (bind) {
            if (!rt_bind(dt, f->depth_level, 0, dl->w, dl->h, 1)) return GL_FRAMEBUFFER_UNSUPPORTED_EXT;
            t->ds = dt->rt_ds;
        } else t->ds = 1;
    }
    return GL_FRAMEBUFFER_COMPLETE_EXT;
}

int gl_fbo_target(gl_fbo *f, g2r_target *t) { return fbo_status(f, t, 1) == GL_FRAMEBUFFER_COMPLETE_EXT ? 0 : -1; }

/* the depth and stencil bits of a framebuffer object's attachments */
void gl_fbo_ds_bits(GLuint name, int *depth, int *stencil)
{
    gl_fbo *f = gl_fbo_lookup(name);
    *depth = *stencil = 0;
    if (!f) return;
    if (f->depth_type == GL_RENDERBUFFER_EXT && f->depth < F_MAX && g_rbs[f->depth].handle) {
        const gl_renderbuffer *b = &g_rbs[f->depth];
        *depth = b->internal == GL_STENCIL_INDEX8_EXT ? 0 : b->depth16 ? 16 : 24;
        if (b->internal == GL_DEPTH24_STENCIL8_EXT) *stencil = 8;
    } else if (f->depth_type == GL_TEXTURE) {
        const gl_tex *x = gl_tex_lookup(f->depth, GL_TEXTURE_2D, 0);
        *depth = 24;
        if (x && (x->internal == GL_DEPTH_STENCIL_EXT || x->internal == GL_DEPTH24_STENCIL8_EXT)) *stencil = 8;
    }
    if (f->stencil_type) *stencil = 8;
}

void APIENTRY glGenFramebuffersEXT(GLsizei n, GLuint *o) { GLsizei i; for (i = 0; i < n; i++) o[i] = g_next_fbo < F_MAX ? g_next_fbo++ : 0; }
void APIENTRY glBindFramebufferEXT(GLenum t, GLuint f)
{
    FLUSH_VERTICES();
    if (t != GL_READ_FRAMEBUFFER_EXT) C->fbo = f;
    if (t != GL_DRAW_FRAMEBUFFER_EXT) C->read_fbo = f;
}
void APIENTRY glDeleteFramebuffersEXT(GLsizei n, const GLuint *o)
{
    GLsizei i;
    for (i = 0; i < n; i++) {
        if (o[i] && o[i] < F_MAX) memset(&g_fbos[o[i]], 0, sizeof g_fbos[0]);
        if (C->fbo == o[i]) C->fbo = 0;
        if (C->read_fbo == o[i]) C->read_fbo = 0;
    }
}
GLboolean APIENTRY glIsFramebufferEXT(GLuint f) { return f && f < g_next_fbo; }
GLenum APIENTRY glCheckFramebufferStatusEXT(GLenum t)
{
    gl_fbo *f = gl_fbo_lookup(t == GL_READ_FRAMEBUFFER_EXT ? C->read_fbo : C->fbo);
    g2r_target x;
    return f ? fbo_status(f, &x, 0) : GL_FRAMEBUFFER_COMPLETE_EXT;
}
void APIENTRY glFramebufferTexture2DEXT(GLenum t, GLenum att, GLenum tt, GLuint tex, GLint level)
{
    gl_fbo *f = gl_fbo_lookup(t == GL_READ_FRAMEBUFFER_EXT ? C->read_fbo : C->fbo);
    FLUSH_VERTICES();
    if (!f) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (tex && !gl_tex_lookup(tex, GL_TEXTURE_2D, 0)) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (level < 0 || level >= MAX_LEVELS) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (att == GL_COLOR_ATTACHMENT0_EXT) {
        f->color_type = tex ? GL_TEXTURE : 0; f->color = tex; f->color_level = level; f->color_face = face_of(tt); f->color_z = 0;
    } else if (att == GL_DEPTH_ATTACHMENT_EXT) {
        f->depth_type = tex ? GL_TEXTURE : 0; f->depth = tex; f->depth_level = level;
    } else if (att == GL_STENCIL_ATTACHMENT_EXT) {
        f->stencil_type = tex ? GL_TEXTURE : 0; f->stencil = tex; f->depth_level = level;
    } else SET_ERROR(GL_INVALID_ENUM);
}
void APIENTRY glFramebufferRenderbufferEXT(GLenum t, GLenum att, GLenum rt, GLuint rb)
{
    gl_fbo *f = gl_fbo_lookup(t == GL_READ_FRAMEBUFFER_EXT ? C->read_fbo : C->fbo);
    (void)rt;
    FLUSH_VERTICES();
    if (!f) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (att == GL_COLOR_ATTACHMENT0_EXT) { f->color_type = rb ? GL_RENDERBUFFER_EXT : 0; f->color = rb; }
    else if (att == GL_DEPTH_ATTACHMENT_EXT) { f->depth_type = rb ? GL_RENDERBUFFER_EXT : 0; f->depth = rb; }
    else if (att == GL_STENCIL_ATTACHMENT_EXT) { f->stencil_type = rb ? GL_RENDERBUFFER_EXT : 0; f->stencil = rb; }
}
void APIENTRY glGetFramebufferAttachmentParameterivEXT(GLenum t, GLenum att, GLenum p, GLint *v)
{
    gl_fbo *f = gl_fbo_lookup(t == GL_READ_FRAMEBUFFER_EXT ? C->read_fbo : C->fbo);
    GLenum ty = 0;
    GLuint n = 0;
    int level = 0;
    if (!f) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (att == GL_COLOR_ATTACHMENT0_EXT) { ty = f->color_type; n = f->color; level = f->color_level; }
    else if (att == GL_DEPTH_ATTACHMENT_EXT) { ty = f->depth_type; n = f->depth; level = f->depth_level; }
    else if (att == GL_STENCIL_ATTACHMENT_EXT) { ty = f->stencil_type; n = f->stencil; level = f->depth_level; }
    else { SET_ERROR(GL_INVALID_ENUM); return; }
    switch (p) {
    case GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE_EXT: *v = (GLint)ty; break;
    case GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME_EXT: if (ty) *v = (GLint)n; else SET_ERROR(GL_INVALID_ENUM); break;
    case GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_LEVEL_EXT: if (ty == GL_TEXTURE) *v = level; else SET_ERROR(GL_INVALID_ENUM); break;
    case GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_CUBE_MAP_FACE_EXT: {
        gl_tex *x = ty == GL_TEXTURE ? gl_tex_lookup(n, GL_TEXTURE_2D, 0) : NULL;
        if (!x) SET_ERROR(GL_INVALID_ENUM); else *v = x->target == GL_TEXTURE_CUBE_MAP ? (GLint)GL_TEXTURE_CUBE_MAP_POSITIVE_X + f->color_face : 0;
        break;
    }
    case GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_3D_ZOFFSET_EXT: if (ty == GL_TEXTURE) *v = f->color_z; else SET_ERROR(GL_INVALID_ENUM); break;
    default: SET_ERROR(GL_INVALID_ENUM); break;
    }
}
void APIENTRY glGenRenderbuffersEXT(GLsizei n, GLuint *o) { GLsizei i; for (i = 0; i < n; i++) o[i] = g_next_rb < F_MAX ? g_next_rb++ : 0; }
void APIENTRY glBindRenderbufferEXT(GLenum t, GLuint r) { (void)t; C->renderbuffer = r; }
void APIENTRY glDeleteRenderbuffersEXT(GLsizei n, const GLuint *o)
{
    GLsizei i;
    for (i = 0; i < n; i++) {
        if (o[i] && o[i] < F_MAX) {
            gl_renderbuffer *b = &g_rbs[o[i]];
            if (b->handle && C->live) { g2r_end_pass(D.r); g2r_image_destroy(D.r, b->handle, b->base); }
            memset(b, 0, sizeof *b);
        }
    }
}
GLboolean APIENTRY glIsRenderbufferEXT(GLuint r) { return r && r < g_next_rb; }
void APIENTRY glRenderbufferStorageEXT(GLenum t, GLenum internal, GLsizei w, GLsizei h)
{
    gl_renderbuffer *b = C->renderbuffer && C->renderbuffer < F_MAX ? &g_rbs[C->renderbuffer] : NULL;
    int depth = internal == GL_DEPTH_COMPONENT || internal == GL_DEPTH_COMPONENT16 || internal == GL_DEPTH_COMPONENT24 || internal == GL_DEPTH_COMPONENT32 ||
                internal == GL_STENCIL_INDEX8_EXT || internal == GL_DEPTH24_STENCIL8_EXT;
    (void)t;
    FLUSH_VERTICES();
    if (!b || !C->live) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (b->handle) { g2r_end_pass(D.r); g2r_image_destroy(D.r, b->handle, b->base); b->handle = 0; }
    b->hwfmt = depth ? G2_FMT_D24S8 : gl_hw_format(internal, GL_UNSIGNED_BYTE);
    if (g2r_image_create(D.r, &b->handle, &b->base, (uint32_t)w, (uint32_t)h, b->hwfmt, 1, 1, 1, G2_USE_RT | G2_USE_TRANSFER) < 0) {
        SET_ERROR(GL_OUT_OF_MEMORY);
        return;
    }
    if (depth) g2r_stencil_undefined(D.r, b->handle);
    b->internal = internal; b->w = w; b->h = h; b->depth16 = internal == GL_DEPTH_COMPONENT16;
}
void APIENTRY glGetRenderbufferParameterivEXT(GLenum t, GLenum p, GLint *v)
{
    gl_renderbuffer *b = C->renderbuffer < F_MAX ? &g_rbs[C->renderbuffer] : &g_rbs[0];
    (void)t;
    int depth = b->internal == GL_DEPTH_COMPONENT || b->internal == GL_DEPTH_COMPONENT16 || b->internal == GL_DEPTH_COMPONENT24 || b->internal == GL_DEPTH_COMPONENT32 || b->internal == GL_STENCIL_INDEX8_EXT || b->internal == GL_DEPTH24_STENCIL8_EXT;
    switch (p) {
    case GL_RENDERBUFFER_WIDTH_EXT: *v = b->w; break;
    case GL_RENDERBUFFER_HEIGHT_EXT: *v = b->h; break;
    case GL_RENDERBUFFER_INTERNAL_FORMAT_EXT: *v = (GLint)b->internal; break;
    case GL_RENDERBUFFER_DEPTH_SIZE_EXT: *v = depth ? (b->depth16 ? 16 : 24) : 0; break;
    case GL_RENDERBUFFER_STENCIL_SIZE_EXT: *v = depth ? 8 : 0; break;
    case GL_RENDERBUFFER_RED_SIZE_EXT: *v = depth || !b->internal ? 0 : gl_fmt_bits(b->hwfmt, 0); break;
    case GL_RENDERBUFFER_GREEN_SIZE_EXT: *v = depth || !b->internal ? 0 : gl_fmt_bits(b->hwfmt, 1); break;
    case GL_RENDERBUFFER_BLUE_SIZE_EXT: *v = depth || !b->internal ? 0 : gl_fmt_bits(b->hwfmt, 2); break;
    case GL_RENDERBUFFER_ALPHA_SIZE_EXT: *v = depth || !b->internal || base_kind(b->internal) == K_RGB ? 0 : gl_fmt_bits(b->hwfmt, 3); break;
    default: *v = 0; break;
    }
}
void APIENTRY glFramebufferTexture1DEXT(GLenum t, GLenum att, GLenum tt, GLuint tex, GLint level) { glFramebufferTexture2DEXT(t, att, tt, tex, level); }
void APIENTRY glFramebufferTexture3DEXT(GLenum t, GLenum att, GLenum tt, GLuint tex, GLint level, GLint z)
{
    gl_fbo *f = gl_fbo_lookup(t == GL_READ_FRAMEBUFFER_EXT ? C->read_fbo : C->fbo);
    glFramebufferTexture2DEXT(t, att, tt, tex, level);
    if (f && att == GL_COLOR_ATTACHMENT0_EXT) f->color_z = z;
}

/* GL_EXT_framebuffer_blit. The board's copy scales by the nearest pixel and mirrors; a linear scale and what is not colour go by the PC */
static void blit_depth(const g2r_target *s, const g2r_target *d, const int *a, const int *b, GLbitfield mask)
{
    int dw = abs(b[2] - b[0]), dh = abs(b[3] - b[1]), sw = abs(a[2] - a[0]), sh = abs(a[3] - a[1]), x, y;
    int sx = a[0] < a[2] ? a[0] : a[2], sy = a[1] < a[3] ? a[1] : a[3], dx = b[0] < b[2] ? b[0] : b[2], dy = b[1] < b[3] ? b[1] : b[3];
    uint32_t *from = (uint32_t *)malloc((size_t)sw * sh * 4), *to = (uint32_t *)malloc((size_t)dw * dh * 4);
    uint32_t keep = (mask & GL_DEPTH_BUFFER_BIT ? 0 : 0xFFFFFFu) | (mask & GL_STENCIL_BUFFER_BIT ? 0 : 0xFF000000u);
    if (from && to) {
        g2_read_image(D.bd, s->ds, G2_TF_RAW, (uint32_t)sx, (uint32_t)sy, (uint32_t)sw, (uint32_t)sh, 0, 0, G2_FMT_D24S8, 0, 0, from);
        g2_read_image(D.bd, d->ds, G2_TF_RAW, (uint32_t)dx, (uint32_t)dy, (uint32_t)dw, (uint32_t)dh, 0, 0, G2_FMT_D24S8, 0, 0, to);
        if (s->depth16 != d->depth16) {                         /* 16 and 24 bits of depth: the value, in the destination's units */
            size_t i;
            for (i = 0; i < (size_t)sw * sh; i++) from[i] = (from[i] & 0xFF000000u) | (s->depth16 ? (uint32_t)((from[i] & 0xFFFF) * 16777215.0 / 65535.0 + 0.5) : (uint32_t)((from[i] & 0xFFFFFF) * 65535.0 / 16777215.0 + 0.5));
        }
        for (y = 0; y < dh; y++) {
            int v = (y * 2 + 1) * sh / (2 * dh);
            if ((a[3] < a[1]) != (b[3] < b[1])) v = sh - 1 - v;
            for (x = 0; x < dw; x++) {
                int u = (x * 2 + 1) * sw / (2 * dw);
                if ((a[2] < a[0]) != (b[2] < b[0])) u = sw - 1 - u;
                to[(size_t)y * dw + x] = (to[(size_t)y * dw + x] & keep) | (from[(size_t)v * sw + u] & ~keep);
            }
        }
        g2_write_image(D.bd, d->ds, to, (size_t)dw * dh * 4, G2_TF_RAW, (uint32_t)dx, (uint32_t)dy, (uint32_t)dw, (uint32_t)dh, 0, 0, 0, G2_FMT_D24S8);
        g2r_stencil_written(D.r, d->ds);
    }
    free(from); free(to);
}

static void blit_linear(const g2r_target *s, const g2r_target *d, const int *a, const int *b, int linear)
{
    int dw = abs(b[2] - b[0]), dh = abs(b[3] - b[1]), sw = abs(a[2] - a[0]), sh = abs(a[3] - a[1]), x, y, c;
    int sx = a[0] < a[2] ? a[0] : a[2], sy = a[1] < a[3] ? a[1] : a[3], dx = b[0] < b[2] ? b[0] : b[2], dy = b[1] < b[3] ? b[1] : b[3];
    uint8_t *from = (uint8_t *)malloc((size_t)sw * sh * 4), *to = (uint8_t *)malloc((size_t)dw * dh * 4);
    if (from && to) {
        gl_img_read(s->color, gl_tg_fmt(s), (uint32_t)sx, (uint32_t)sy, (uint32_t)sw, (uint32_t)sh, 0, 0, from);
        for (y = 0; y < dh && !linear; y++) {                   /* GL_NEAREST between formats the board does not convert */
            int v = (y * 2 + 1) * sh / (2 * dh);
            if ((a[3] < a[1]) != (b[3] < b[1])) v = sh - 1 - v;
            for (x = 0; x < dw; x++) {
                int u = (x * 2 + 1) * sw / (2 * dw);
                if ((a[2] < a[0]) != (b[2] < b[0])) u = sw - 1 - u;
                memcpy(to + ((size_t)y * dw + x) * 4, from + ((size_t)v * sw + u) * 4, 4);
            }
        }
        for (y = 0; y < dh && linear; y++) {
            double v = (y + 0.5) * sh / dh - 0.5;
            int v0 = (int)floor(v), v1;
            double fy;
            if ((a[3] < a[1]) != (b[3] < b[1])) { v = sh - 1 - v; v0 = (int)floor(v); }
            fy = v - v0; v1 = v0 + 1;
            if (v0 < 0) v0 = 0;
            if (v1 > sh - 1) v1 = sh - 1;
            if (v0 > sh - 1) v0 = sh - 1;
            for (x = 0; x < dw; x++) {
                double u = (x + 0.5) * sw / dw - 0.5, fx;
                int u0, u1;
                if ((a[2] < a[0]) != (b[2] < b[0])) u = sw - 1 - u;
                u0 = (int)floor(u); fx = u - u0; u1 = u0 + 1;
                if (u0 < 0) u0 = 0;
                if (u1 > sw - 1) u1 = sw - 1;
                if (u0 > sw - 1) u0 = sw - 1;
                for (c = 0; c < 4; c++) {
                    double t0 = from[((size_t)v0 * sw + u0) * 4 + c] * (1.0 - fx) + from[((size_t)v0 * sw + u1) * 4 + c] * fx;
                    double t1 = from[((size_t)v1 * sw + u0) * 4 + c] * (1.0 - fx) + from[((size_t)v1 * sw + u1) * 4 + c] * fx;
                    to[((size_t)y * dw + x) * 4 + c] = (uint8_t)(t0 * (1.0 - fy) + t1 * fy + 0.5);
                }
            }
        }
        gl_img_write(d->color, gl_tg_fmt(d), to, dw, (uint32_t)dx, (uint32_t)dy, (uint32_t)dw, (uint32_t)dh, 0, 0);
    }
    free(from); free(to);
}

/* one rectangle against its image, the other in step, a coordinate at a time */
static int blit_clip(int *a, int *b, int lo, int hi)
{
    if (a[0] == a[1] || b[0] == b[1]) return 0;
    {
        double k = (double)(b[1] - b[0]) / (a[1] - a[0]);
        int i;
        for (i = 0; i < 2; i++) {
            if (a[i] < lo) { b[i] += (int)floor((lo - a[i]) * k + 0.5); a[i] = lo; }
            if (a[i] > hi) { b[i] -= (int)floor((a[i] - hi) * k + 0.5); a[i] = hi; }
        }
    }
    return a[0] != a[1] && b[0] != b[1];
}

void APIENTRY glBlitFramebufferEXT(GLint sx0, GLint sy0, GLint sx1, GLint sy1, GLint dx0, GLint dy0, GLint dx1, GLint dy1, GLbitfield mask, GLenum filter)
{
    g2r_target s, d;
    int a[4] = { sx0, sy0, sx1, sy1 }, b[4] = { dx0, dy0, dx1, dy1 }, ax[2], bx[2], ay[2], by[2], sc[4];
    FLUSH_VERTICES();
    if (mask & ~(GLbitfield)(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT)) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (filter != GL_NEAREST && filter != GL_LINEAR) { SET_ERROR(GL_INVALID_ENUM); return; }
    if ((mask & (GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT)) && filter != GL_NEAREST) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (!C->live) return;
    gl_bind_target();
    gl_cur_target(&d);
    gl_read_target(&s);
    g2r_finish(D.r);
    if (!gl_scissor_px(&d, sc)) return;
    ax[0] = a[0]; ax[1] = a[2]; bx[0] = b[0]; bx[1] = b[2]; ay[0] = a[1]; ay[1] = a[3]; by[0] = b[1]; by[1] = b[3];
    if (!blit_clip(ax, bx, 0, (int)s.width) || !blit_clip(ay, by, 0, (int)s.height)) return;
    if (!blit_clip(bx, ax, sc[0], sc[2]) || !blit_clip(by, ay, sc[1], sc[3])) return;
    a[0] = ax[0]; a[2] = ax[1]; b[0] = bx[0]; b[2] = bx[1]; a[1] = ay[0]; a[3] = ay[1]; b[1] = by[0]; b[3] = by[1];
    if (mask & GL_COLOR_BUFFER_BIT) {
        int sw = abs(a[2] - a[0]), sh = abs(a[3] - a[1]), dw = abs(b[2] - b[0]), dh = abs(b[3] - b[1]);
        uint32_t sf = gl_tg_fmt(&s), df = gl_tg_fmt(&d);
        int board = sf == df || ((sf == G2_FMT_RGBA8 || sf == G2_FMT_RGB565) && (df == G2_FMT_RGBA8 || df == G2_FMT_RGB565));
        if (filter == GL_LINEAR && (sw != dw || sh != dh)) blit_linear(&s, &d, a, b, 1);
        else if (!board) blit_linear(&s, &d, a, b, 0);
        else g2_copy_image(D.bd, s.color, d.color, (uint32_t)(a[0] < a[2] ? a[0] : a[2]), (uint32_t)(a[1] < a[3] ? a[1] : a[3]), (uint32_t)sw, (uint32_t)sh, (uint32_t)(b[0] < b[2] ? b[0] : b[2]), (uint32_t)(b[1] < b[3] ? b[1] : b[3]), (uint32_t)dw, (uint32_t)dh,
                           (a[2] < a[0]) != (b[2] < b[0]), (a[3] < a[1]) != (b[3] < b[1]), 0, 0, 0, 0, 0, 0, 0);
    }
    if ((mask & (GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT)) && s.ds && d.ds) blit_depth(&s, &d, a, b, mask);
}

void APIENTRY glGenerateMipmapEXT(GLenum target)
{
    gl_tex *t = bound_tex(target);
    uint8_t *px;
    FLUSH_VERTICES();
    if (!t || !t->handle || !C->live) return;
    gl_tex_resolve(t);
    if (target == GL_TEXTURE_3D) { gl_tex3d_generate(t); return; }
    px = (uint8_t *)malloc((size_t)t->w * t->h * 4);
    if (!px) return;
    g2r_finish(D.r);
    if (t->base_class == BF_DEPTH) g2_read_image(D.bd, t->handle, G2_TF_RGBA8888, 0, 0, (uint32_t)t->w, (uint32_t)t->h, 0, 0, G2_FMT_RGBA8, 0, 0, px);
    else gl_img_read(t->handle, t->hwfmt, 0, 0, (uint32_t)t->w, (uint32_t)t->h, 0, 0, px);
    generate_levels(t, 0, px);
    free(px);
}
