/* what the OpenGL 1.1 pixel path asks for and the GPU has no part in: pixel transfer and pixel maps (§3.6.3), colour indices, the accumulation buffer (§4.2.4), stipple patterns, the logic op's state and
 * the current colour index. Everything here is the PC's: the patterns reach the board as small images that generated programs read (gl_ff.c), the logic ops run in gl_draw.c. */
#include "gl_internal.h"

void gl_extra_init(gl_ctx *c)
{
    int i;
    c->line_factor = 1;
    c->line_pattern = 0xFFFF;
    memset(c->poly_pattern, 0xFF, sizeof c->poly_pattern);
    c->logic_op = GL_COPY;
    c->cur_index = 1.0f;
    c->render_mode = GL_RENDER;
    c->px_depth_scale = 1.0f;
    for (i = 0; i < 4; i++) c->px_scale[i] = 1.0f;
    c->grid1_n = 1; c->grid1[1] = 1.0f;
    c->grid2_n[0] = c->grid2_n[1] = 1; c->grid2[1] = c->grid2[3] = 1.0f;
    gl_eval_clear(c);
}

/* ---------------------------------------------------------------- pixel transfer */
static void px_changed(void)
{
    int i;
    C->px_active = C->px_map_color;
    for (i = 0; i < 4; i++) if (C->px_scale[i] != 1.0f || C->px_bias[i] != 0.0f) C->px_active = 1;
}

/* one RGBA8 pixel through scale, bias and the colour maps; bytes both ways, as everything is stored */
void gl_px_one(uint8_t *o)
{
    int c;
    for (c = 0; c < 4; c++) {
        float v = (float)o[c] / 255.0f * C->px_scale[c] + C->px_bias[c];
        v = v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v;
        if (C->px_map_color) {
            const gl_pixmap *m = &C->pmap[6 + c];                   /* R_TO_R .. A_TO_A */
            if (m->n && m->v) {
                v = m->v[(int)(v * (float)(m->n - 1) + 0.5f)];
                v = v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v;
            }
        }
        o[c] = (uint8_t)(v * 255.0f + 0.5f);
    }
}

void gl_px_rows(uint8_t *rgba, int w, int h)
{
    size_t i, n = (size_t)w * h;
    if (C->px_active) for (i = 0; i < n; i++) gl_px_one(rgba + i * 4);
}

/* a colour index to RGBA: shift, offset, then always the I_TO_* maps, as this context is RGBA (§3.6.3) */
void gl_px_index(uint32_t i, uint8_t *o)
{
    int s = C->px_shift, c;
    int64_t v = s >= 0 ? (int64_t)i << s : (int64_t)i >> -s;
    v += C->px_offset;
    for (c = 0; c < 4; c++) {
        const gl_pixmap *m = &C->pmap[2 + c];                       /* I_TO_R .. I_TO_A */
        float f = m->n && m->v ? m->v[(uint32_t)v & (uint32_t)(m->n - 1)] : 0.0f;
        f = f < 0.0f ? 0.0f : f > 1.0f ? 1.0f : f;
        o[c] = (uint8_t)(f * 255.0f + 0.5f);
    }
}

/* a stencil index through shift, offset and the S_TO_S map; a depth through scale and bias */
uint32_t gl_px_stencil_value(uint32_t v)
{
    int s = C->px_shift;
    int64_t i = s >= 0 ? (int64_t)v << s : (int64_t)v >> -s;
    i += C->px_offset;
    if (C->px_map_stencil && C->pmap[1].n && C->pmap[1].v) i = (int64_t)C->pmap[1].v[(uint32_t)i & (uint32_t)(C->pmap[1].n - 1)];
    return (uint32_t)i;
}

float gl_px_depth_value(float z)
{
    z = z * C->px_depth_scale + C->px_depth_bias;
    return z < 0.0f ? 0.0f : z > 1.0f ? 1.0f : z;
}

static int cmp_test(GLenum f, uint32_t a, uint32_t b)
{
    switch (f) {
    case GL_NEVER: return 0;
    case GL_LESS: return a < b;
    case GL_EQUAL: return a == b;
    case GL_LEQUAL: return a <= b;
    case GL_GREATER: return a > b;
    case GL_NOTEQUAL: return a != b;
    case GL_GEQUAL: return a >= b;
    default: return 1;
    }
}

static uint32_t stencil_op(GLenum op, uint32_t s, uint32_t ref)
{
    switch (op) {
    case GL_ZERO: return 0;
    case GL_REPLACE: return ref;
    case GL_INCR: return s < 255 ? s + 1 : 255;
    case GL_DECR: return s ? s - 1 : 0;
    case GL_INVERT: return ~s & 255;
    case GL_INCR_WRAP: return (s + 1) & 255;
    case GL_DECR_WRAP: return (s - 1) & 255;
    default: return s;
    }
}

/* Depth and stencil pixels. A depth value is 24 bits and the fragment program's registers hold 17 of them, so these fragments go through the depth and stencil tests here, on the depth-stencil image as it stands,
 * exactly; what passed comes back as a mask of the target's rectangle, rect = x, y, w, h, for gl_api.c to draw the colour through. Stencil pixels pass only the scissor and the write mask (§4.3.1): st gives them.
 * Which fragments: the target pixels whose centres fall in a pixel's zoomed rectangle at the raster position (§3.6.4). NULL: none */
uint8_t *gl_px_fragments(const float *z, const uint32_t *st, int w, int h, float zx, float zy, int *rect)
{
    g2r_target t;
    double kx, ky, xr = C->raster_pos[0], yr = C->raster_pos[1], ex = xr + w * (double)zx, ey = yr + h * (double)zy;
    int sc[4], tx0, ty0, tx1, ty1, tw, th, x, y, wrote = 0;
    uint32_t ref = (uint32_t)C->st_ref[0] & 255, vm = C->st_vmask[0] & 255, wm = C->st_wmask[0] & 255, *ds;
    uint8_t *mask;
    double dmax;
    if (zx == 0.0f || zy == 0.0f) return NULL;
    gl_bind_target();
    gl_cur_target(&t);
    dmax = t.depth16 ? 65535.0 : 16777215.0;                    /* the values the depth buffer holds */
    gl_scale(&kx, &ky);
    if (!gl_scissor_px(&t, sc)) return NULL;
    if (z && C->alpha_test && !cmp_test(C->alpha_func, (uint32_t)(C->raster_color[3] * 16777215.0f + 0.5f), (uint32_t)(C->alpha_ref * 16777215.0f + 0.5f))) return NULL;
    tx0 = (int)floor((xr < ex ? xr : ex) * kx); tx1 = (int)ceil((xr < ex ? ex : xr) * kx);
    ty0 = (int)floor((yr < ey ? yr : ey) * ky); ty1 = (int)ceil((yr < ey ? ey : yr) * ky);
    if (tx0 < sc[0]) tx0 = sc[0];
    if (ty0 < sc[1]) ty0 = sc[1];
    if (tx1 > sc[2]) tx1 = sc[2];
    if (ty1 > sc[3]) ty1 = sc[3];
    tw = tx1 - tx0; th = ty1 - ty0;
    if (tw <= 0 || th <= 0) return NULL;
    ds = (uint32_t *)malloc((size_t)tw * th * 4);
    mask = (uint8_t *)calloc((size_t)tw * th, 4);
    if (!ds || !mask) { free(ds); free(mask); SET_ERROR(GL_OUT_OF_MEMORY); return NULL; }
    g2r_finish(D.r);
    g2_read_image(D.bd, t.ds, G2_TF_RAW, (uint32_t)tx0, (uint32_t)ty0, (uint32_t)tw, (uint32_t)th, 0, 0, G2_FMT_D24S8, 0, 0, ds);
    for (y = 0; y < th; y++) {
        int m = (int)floor(((ty0 + y + 0.5) / ky - yr) / zy);
        if (m < 0 || m >= h) continue;
        for (x = 0; x < tw; x++) {
            int n = (int)floor(((tx0 + x + 0.5) / kx - xr) / zx);
            uint32_t *v = &ds[(size_t)y * tw + x], d = *v & 0xFFFFFF, s = *v >> 24, was = *v;
            if (n < 0 || n >= w) continue;
            if (st) {
                s = (s & ~wm) | (st[(size_t)m * w + n] & wm);
            } else {
                uint32_t zv = (uint32_t)((double)z[(size_t)m * w + n] * dmax + 0.5);
                int zok = !C->depth_test || cmp_test(C->depth_func, zv, d), sok = 1;
                if (C->stencil_test) {
                    sok = cmp_test(C->st_func[0], ref & vm, s & vm);
                    s = (s & ~wm) | (stencil_op(!sok ? C->st_fail[0] : zok ? C->st_zpass[0] : C->st_zfail[0], s, ref) & wm);
                }
                if (zok && sok) {
                    mask[((size_t)y * tw + x) * 4 + 3] = 255;
                    if (C->depth_test && C->depth_mask) d = zv;
                }
            }
            *v = s << 24 | d;
            if (*v != was) wrote = 1;
        }
    }
    if (wrote) {
        g2_write_image(D.bd, t.ds, ds, (size_t)tw * th * 4, G2_TF_RAW, (uint32_t)tx0, (uint32_t)ty0, (uint32_t)tw, (uint32_t)th, 0, 0, 0, G2_FMT_D24S8);
        g2r_stencil_written(D.r, t.ds);
    }
    free(ds);
    if (st) { free(mask); return NULL; }                        /* stencil pixels make no colour */
    rect[0] = tx0; rect[1] = ty0; rect[2] = tw; rect[3] = th;
    return mask;
}

static void pixel_transfer(GLenum p, float v)
{
    FLUSH_VERTICES();
    switch (p) {
    case GL_RED_SCALE: C->px_scale[0] = v; break;
    case GL_GREEN_SCALE: C->px_scale[1] = v; break;
    case GL_BLUE_SCALE: C->px_scale[2] = v; break;
    case GL_ALPHA_SCALE: C->px_scale[3] = v; break;
    case GL_RED_BIAS: C->px_bias[0] = v; break;
    case GL_GREEN_BIAS: C->px_bias[1] = v; break;
    case GL_BLUE_BIAS: C->px_bias[2] = v; break;
    case GL_ALPHA_BIAS: C->px_bias[3] = v; break;
    case GL_INDEX_SHIFT: C->px_shift = (int)v; break;
    case GL_INDEX_OFFSET: C->px_offset = (int)v; break;
    case GL_MAP_COLOR: C->px_map_color = v != 0.0f; break;
    case GL_MAP_STENCIL: C->px_map_stencil = v != 0.0f; break;
    case GL_DEPTH_SCALE: C->px_depth_scale = v; break;
    case GL_DEPTH_BIAS: C->px_depth_bias = v; break;
    default: SET_ERROR(GL_INVALID_ENUM); return;
    }
    px_changed();
}
void APIENTRY glPixelTransferf(GLenum p, GLfloat v) { pixel_transfer(p, v); }
void APIENTRY glPixelTransferi(GLenum p, GLint v) { pixel_transfer(p, (float)v); }

static gl_pixmap *pixmap(GLenum m)
{
    if (m < GL_PIXEL_MAP_I_TO_I || m > GL_PIXEL_MAP_A_TO_A) { SET_ERROR(GL_INVALID_ENUM); return NULL; }
    return &C->pmap[m - GL_PIXEL_MAP_I_TO_I];
}

static float *pixmap_set(GLenum m, GLsizei n)
{
    gl_pixmap *p = pixmap(m);
    float *v;
    if (!p) return NULL;
    /* the tables a colour index goes through are indexed by a mask: their size is a power of two */
    if (n < 1 || n > MAX_PIXEL_MAP || (m <= GL_PIXEL_MAP_I_TO_A && (n & (n - 1)))) { SET_ERROR(GL_INVALID_VALUE); return NULL; }
    FLUSH_VERTICES();
    v = (float *)malloc((size_t)n * sizeof *v);
    if (!v) { SET_ERROR(GL_OUT_OF_MEMORY); return NULL; }
    free(p->v);
    p->v = v; p->n = n;
    return v;
}

void APIENTRY glPixelMapfv(GLenum m, GLsizei n, const GLfloat *x) { float *v = pixmap_set(m, n); if (v) memcpy(v, x, (size_t)n * 4); }
void APIENTRY glPixelMapuiv(GLenum m, GLsizei n, const GLuint *x) { float *v = pixmap_set(m, n); GLsizei i; if (v) for (i = 0; i < n; i++) v[i] = m <= GL_PIXEL_MAP_S_TO_S ? (float)x[i] : (float)(x[i] / 4294967295.0); }
void APIENTRY glPixelMapusv(GLenum m, GLsizei n, const GLushort *x) { float *v = pixmap_set(m, n); GLsizei i; if (v) for (i = 0; i < n; i++) v[i] = m <= GL_PIXEL_MAP_S_TO_S ? (float)x[i] : (float)x[i] / 65535.0f; }

void APIENTRY glGetPixelMapfv(GLenum m, GLfloat *o) { gl_pixmap *p = pixmap(m); if (!p) return; if (p->v) memcpy(o, p->v, (size_t)p->n * 4); else *o = 0.0f; }
void APIENTRY glGetPixelMapuiv(GLenum m, GLuint *o)
{
    gl_pixmap *p = pixmap(m);
    int i;
    if (!p) return;
    if (!p->v) { *o = 0; return; }
    for (i = 0; i < p->n; i++) o[i] = m <= GL_PIXEL_MAP_S_TO_S ? (GLuint)p->v[i] : (GLuint)((double)p->v[i] * 4294967295.0);
}
void APIENTRY glGetPixelMapusv(GLenum m, GLushort *o)
{
    gl_pixmap *p = pixmap(m);
    int i;
    if (!p) return;
    if (!p->v) { *o = 0; return; }
    for (i = 0; i < p->n; i++) o[i] = m <= GL_PIXEL_MAP_S_TO_S ? (GLushort)p->v[i] : (GLushort)(p->v[i] * 65535.0f + 0.5f);
}

/* ---------------------------------------------------------------- the accumulation buffer */
static int accum_ready(void)
{
    g2r_target t;
    gl_bind_target();
    gl_cur_target(&t);
    if (C->accum && C->accum_w == (int)t.width && C->accum_h == (int)t.height) return 1;
    free(C->accum);
    C->accum = (float *)calloc((size_t)t.width * t.height * 4, sizeof(float));
    if (!C->accum) { SET_ERROR(GL_OUT_OF_MEMORY); return 0; }
    C->accum_w = (int)t.width; C->accum_h = (int)t.height;
    return 1;
}

void gl_accum_clear(int x0, int y0, int x1, int y1)
{
    int x, y, c;
    if (!accum_ready()) return;
    if (x1 > C->accum_w) x1 = C->accum_w;
    if (y1 > C->accum_h) y1 = C->accum_h;
    for (y = y0; y < y1; y++) {
        for (x = x0; x < x1; x++) for (c = 0; c < 4; c++) C->accum[((size_t)y * C->accum_w + x) * 4 + c] = C->accum_clear[c];
    }
}

void APIENTRY glClearAccum(GLfloat r, GLfloat g, GLfloat b, GLfloat a)
{
    C->accum_clear[0] = r < -1 ? -1 : r > 1 ? 1 : r; C->accum_clear[1] = g < -1 ? -1 : g > 1 ? 1 : g;
    C->accum_clear[2] = b < -1 ? -1 : b > 1 ? 1 : b; C->accum_clear[3] = a < -1 ? -1 : a > 1 ? 1 : a;
}

void APIENTRY glAccum(GLenum op, GLfloat value)
{
    g2r_target t;
    uint8_t *px;
    int x0, y0, x1, y1, w, h, x, y, c, sc[4], saved = 0;
    FLUSH_VERTICES();
    if (op != GL_ACCUM && op != GL_LOAD && op != GL_RETURN && op != GL_MULT && op != GL_ADD) { SET_ERROR(GL_INVALID_ENUM); return; }
    if (!C->live || !accum_ready()) return;
    gl_cur_target(&t);
    gl_scissor_px(&t, sc);
    x0 = sc[0]; y0 = sc[1]; x1 = sc[2]; y1 = sc[3];
    w = x1 - x0; h = y1 - y0;
    if (w <= 0 || h <= 0) return;
    if (op == GL_MULT || op == GL_ADD) {
        for (y = y0; y < y1; y++) {
            float *a = C->accum + ((size_t)y * C->accum_w + x0) * 4;
            for (x = 0; x < w * 4; x++) a[x] = op == GL_MULT ? a[x] * value : a[x] + value;
        }
        return;
    }
    px = (uint8_t *)malloc((size_t)w * h * 4);
    if (!px) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    saved = C->px_active;
    C->px_active = 0;                                               /* the framebuffer's values as they are, §4.2.4 */
    g2r_finish(D.r);
    gl_img_read(t.color, gl_tg_fmt(&t), (uint32_t)x0, (uint32_t)y0, (uint32_t)w, (uint32_t)h, 0, 0, px);
    for (y = 0; y < h; y++) {
        float *a = C->accum + ((size_t)(y0 + y) * C->accum_w + x0) * 4;
        uint8_t *p = px + (size_t)y * w * 4;
        for (x = 0; x < w * 4; x += 4) {
            for (c = 0; c < 4; c++) {
                if (op == GL_RETURN) {
                    float v = a[x + c] * value;
                    if (C->color_mask >> (3 - c) & 1) p[x + c] = (uint8_t)(v <= 0.0f ? 0 : v >= 1.0f ? 255 : v * 255.0f + 0.5f);
                } else if (op == GL_LOAD) {
                    a[x + c] = (float)p[x + c] / 255.0f * value;
                } else {
                    a[x + c] += (float)p[x + c] / 255.0f * value;
                }
            }
        }
    }
    if (op == GL_RETURN) gl_img_write(t.color, gl_tg_fmt(&t), px, w, (uint32_t)x0, (uint32_t)y0, (uint32_t)w, (uint32_t)h, 0, 0);
    C->px_active = saved;
    free(px);
}

/* ---------------------------------------------------------------- stipple: a pattern is a small image a generated program reads */
void APIENTRY glLineStipple(GLint factor, GLushort pattern)
{
    FLUSH_VERTICES();
    C->line_factor = factor < 1 ? 1 : factor > 256 ? 256 : factor;
    C->line_pattern = pattern;
}

void APIENTRY glPolygonStipple(const GLubyte *m)
{
    m = (const GLubyte *)gl_unpack_ptr(m);
    int row = (4 + C->un_align - 1) / C->un_align * C->un_align, y;
    FLUSH_VERTICES();
    for (y = 0; y < 32; y++) memcpy(C->poly_pattern + y * 4, m + y * row, 4);
}

void APIENTRY glGetPolygonStipple(GLubyte *m)
{
    m = (GLubyte *)gl_pack_ptr(m);
    int row = (4 + C->pk_align - 1) / C->pk_align * C->pk_align, y;
    for (y = 0; y < 32; y++) memcpy(m + y * row, C->poly_pattern + y * 4, 4);
}

uint16_t gl_stipple_image(int line)
{
    static uint16_t handle[2];
    static uint32_t base[2];
    static uint8_t seen[2][128];
    static int have[2];
    uint8_t now[128];
    int bytes = line ? 2 : 128;
    memset(now, 0, sizeof now);
    if (line) { now[0] = (uint8_t)C->line_pattern; now[1] = (uint8_t)(C->line_pattern >> 8); }
    else memcpy(now, C->poly_pattern, 128);
    if (!handle[line] && g2r_image_create_tight(D.r, &handle[line], &base[line], line ? 16 : 32, line ? 1 : 32, G2_FMT_RGBA8, 1, 1, 1, G2_USE_SAMPLED | G2_USE_TRANSFER) < 0) return 0;
    if (!have[line] || memcmp(seen[line], now, (size_t)bytes)) {
        uint8_t px[32 * 32 * 4];
        int w = line ? 16 : 32, h = line ? 1 : 32, x, y;
        for (y = 0; y < h; y++) {
            for (x = 0; x < w; x++) {
                /* the polygon's bytes read from the top bit, §3.5.2; the line's bit 0 comes first, §3.4.2 */
                int on = line ? (C->line_pattern >> x) & 1 : (now[y * 4 + x / 8] >> (7 - x % 8)) & 1;
                memset(px + (size_t)(y * w + x) * 4, on ? 255 : 0, 4);
            }
        }
        g2r_end_pass(D.r);                                          /* the open pass may have sampled the old one */
        g2_write_image(D.bd, handle[line], px, (size_t)w * h * 4, G2_TF_RGBA8888, 0, 0, (uint32_t)w, (uint32_t)h, 0, 0, 0, G2_FMT_RGBA8);
        memcpy(seen[line], now, (size_t)bytes);
        have[line] = 1;
    }
    return handle[line];
}

/* ---------------------------------------------------------------- the rest of the state */
void APIENTRY glLogicOp(GLenum op)
{
    FLUSH_VERTICES();
    if (op < GL_CLEAR || op > GL_SET) { SET_ERROR(GL_INVALID_ENUM); return; }
    C->logic_op = op;
}

void APIENTRY glIndexd(GLdouble c) { C->cur_index = (float)c; }
void APIENTRY glIndexf(GLfloat c) { C->cur_index = c; }
void APIENTRY glIndexi(GLint c) { C->cur_index = (float)c; }
void APIENTRY glIndexs(GLshort c) { C->cur_index = (float)c; }
void APIENTRY glIndexub(GLubyte c) { C->cur_index = (float)c; }
void APIENTRY glIndexdv(const GLdouble *c) { C->cur_index = (float)*c; }
void APIENTRY glIndexfv(const GLfloat *c) { C->cur_index = *c; }
void APIENTRY glIndexiv(const GLint *c) { C->cur_index = (float)*c; }
void APIENTRY glIndexsv(const GLshort *c) { C->cur_index = (float)*c; }
void APIENTRY glIndexubv(const GLubyte *c) { C->cur_index = (float)*c; }

int gl_extra_ints(GLenum p, GLint *v)
{
    switch (p) {
    case GL_LINE_STIPPLE_PATTERN: *v = C->line_pattern; return 1;
    case GL_LINE_STIPPLE_REPEAT: *v = C->line_factor; return 1;
    case GL_LOGIC_OP_MODE: *v = (GLint)C->logic_op; return 1;
    case GL_CURRENT_INDEX: *v = (GLint)C->cur_index; return 1;
    case GL_RENDER_MODE: *v = (GLint)C->render_mode; return 1;
    case GL_NAME_STACK_DEPTH: *v = C->name_sp; return 1;
    case GL_MAX_NAME_STACK_DEPTH: *v = MAX_NAME_DEPTH; return 1;
    case GL_MAX_EVAL_ORDER: *v = MAX_EVAL_ORDER; return 1;
    case GL_MAX_PIXEL_MAP_TABLE: *v = MAX_PIXEL_MAP; return 1;
    case GL_TEXTURE_BINDING_1D: *v = (GLint)C->unit[C->active_unit].bound_1d; return 1;
    case GL_ACCUM_RED_BITS: case GL_ACCUM_GREEN_BITS: case GL_ACCUM_BLUE_BITS: case GL_ACCUM_ALPHA_BITS: *v = 16; return 1;
    case GL_MAP1_GRID_SEGMENTS: *v = C->grid1_n; return 1;
    case GL_MAP2_GRID_SEGMENTS: v[0] = C->grid2_n[0]; v[1] = C->grid2_n[1]; return 2;
    case GL_INDEX_SHIFT: *v = C->px_shift; return 1;
    case GL_INDEX_OFFSET: *v = C->px_offset; return 1;
    case GL_MAP_COLOR: *v = C->px_map_color; return 1;
    case GL_MAP_STENCIL: *v = C->px_map_stencil; return 1;
    case GL_PIXEL_MAP_I_TO_I_SIZE: case GL_PIXEL_MAP_S_TO_S_SIZE: case GL_PIXEL_MAP_I_TO_R_SIZE: case GL_PIXEL_MAP_I_TO_G_SIZE: case GL_PIXEL_MAP_I_TO_B_SIZE: case GL_PIXEL_MAP_I_TO_A_SIZE:
    case GL_PIXEL_MAP_R_TO_R_SIZE: case GL_PIXEL_MAP_G_TO_G_SIZE: case GL_PIXEL_MAP_B_TO_B_SIZE: case GL_PIXEL_MAP_A_TO_A_SIZE: {
        int n = C->pmap[p - GL_PIXEL_MAP_I_TO_I_SIZE].n;
        *v = n ? n : 1;
        return 1;
    }
    default: return 0;
    }
}
