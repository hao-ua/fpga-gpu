/* geometry: immediate mode, client arrays, display of both through the streaming ring.
 * A draw gathers the referenced range of each array (overlapping interleaved arrays go up once), rebases the indices to u16 and hands g2r_draw one DRAW. The hardware converts the
 * array types of VERTEX_ATTR itself, so a gather is a memcpy; GL_DOUBLE / GL_INT / GL_UNSIGNED_INT are converted here. QUADS, QUAD_STRIP and POLYGON become triangles, in an order
 * that keeps GL's provoking vertex as the triangle's last, because GL_FLAT (which the hardware does not have) expands every primitive with that vertex's colour.
 */
#include "gl_internal.h"

static int hw_type(GLenum t, int *bytes)
{
    switch (t) {
    case GL_BYTE: *bytes = 1; return G2_AT_BYTE;
    case GL_UNSIGNED_BYTE: *bytes = 1; return G2_AT_UBYTE;
    case GL_SHORT: *bytes = 2; return G2_AT_SHORT;
    case GL_UNSIGNED_SHORT: *bytes = 2; return G2_AT_USHORT;
    case GL_FLOAT: *bytes = 4; return G2_AT_FLOAT;
    default: *bytes = 0; return -1;
    }
}

/* a client array as the hardware can read it; GL_DOUBLE, GL_INT, GL_UNSIGNED_INT need a conversion, done over the range the draw uses */
static void *resolve(const gl_array *a, src *s, int normalized, uint32_t lo, uint32_t n)
{
    int b, ty = hw_type(a->type, &b);
    const uint8_t *p = (const uint8_t *)a->ptr;
    void *tmp = NULL;
    if (a->buffer) {
        gl_buffer *o = gl_buffer_lookup(a->buffer);
        p = o && o->data ? o->data + (uintptr_t)a->ptr : NULL;
    }
    s->size = a->size;
    s->normalized = normalized;
    if (ty >= 0) {
        s->from = a->buffer ? gl_buffer_lookup(a->buffer) : NULL;
        s->base = p;
        s->type = ty;
        s->bytes = b * a->size;
        s->stride = a->stride ? a->stride : s->bytes;
    } else {
        int es = a->type == GL_DOUBLE ? 8 : 4, stride = a->stride ? a->stride : es * a->size;
        uint32_t v;
        int c;
        float *f = (float *)malloc((size_t)n * a->size * 4);
        tmp = f;
        for (v = 0; v < n && f; v++) {
            const uint8_t *e = p + (size_t)(lo + v) * stride;
            for (c = 0; c < a->size; c++) {
                float x;
                if (a->type == GL_DOUBLE) x = (float)((const double *)e)[c];
                else if (a->type == GL_INT) x = normalized ? (float)(((const int *)e)[c] / 2147483647.0) : (float)((const int *)e)[c];
                else x = normalized ? (float)(((const unsigned *)e)[c] / 4294967295.0) : (float)((const unsigned *)e)[c];
                f[v * a->size + c] = x;
            }
        }
        s->type = G2_AT_FLOAT;
        s->bytes = 4 * a->size;
        s->stride = s->bytes;
        s->base = (const uint8_t *)f - (size_t)lo * s->stride;
        s->normalized = 0;
        s->conv = 1;
    }
    return tmp;
}

static void constant(src *s, const float *v, int n)
{
    int i;
    s->base = NULL;
    for (i = 0; i < 4; i++) s->constant[i] = g2_fp24(i < n ? v[i] : (i == 3 ? 1.0 : 0.0));
}

/* upload the used range of every array */
static void gather(src *in, int n, uint32_t lo, uint32_t span)
{
    g2r_vsrc v[A_COUNT];
    int i;
    for (i = 0; i < n; i++) {
        gl_buffer *o = in[i].from;
        v[i].base = in[i].base; v[i].stride = in[i].stride; v[i].bytes = in[i].bytes;
        in[i].handle = g2r_stream_handle(D.r);
        /* what board A already holds is not sent; vertex lo is what the indices count from. The range check: a copy of a src (emit_expanded) may have its own base */
        if (o && in[i].base >= o->data && in[i].base < o->data + o->size && g2r_keep(D.r, &o->keep, o->data, (uint32_t)o->size)) {
            in[i].handle = g2r_kept_handle(D.r);
            in[i].off = o->keep.off + (uint32_t)(in[i].base - o->data) + lo * (uint32_t)in[i].stride;
            v[i].base = NULL;
        }
    }
    /* EXT_compiled_vertex_array: what was enabled at the lock does not change until the unlock, so its locked range goes up once. Quake 3 locks positions alone and rewrites colours and texture
       coordinates between the passes of a lock, which is why the mask is taken at the lock and nothing else may be reused */
    if (C->cva_locked && lo >= (uint32_t)C->cva_first && lo + span <= (uint32_t)C->cva_first + (uint32_t)C->cva_count) {
        g2r_vsrc w[A_COUNT];
        uint32_t gen = g2r_stream_gen(D.r);
        int need = 0;
        for (i = 0; i < n; i++) {
            w[i] = v[i];
            w[i].base = NULL;
            if (!v[i].base || in[i].conv || i >= 32 || !(C->cva_mask >> i & 1)) continue;
            if (C->cva[i].base == v[i].base && C->cva[i].stride == v[i].stride && C->cva[i].bytes == v[i].bytes && C->cva[i].gen == gen) continue;
            w[i].base = v[i].base;
            need = 1;
        }
        if (need) {
            g2r_gather(D.r, w, n, (uint32_t)C->cva_first, (uint32_t)C->cva_count);
            gen = g2r_stream_gen(D.r);
            for (i = 0; i < n; i++) {
                if (!w[i].base) continue;
                C->cva[i].base = w[i].base; C->cva[i].stride = w[i].stride; C->cva[i].bytes = w[i].bytes; C->cva[i].off = w[i].off; C->cva[i].gen = gen;
            }
        }
        for (i = 0; i < n; i++) {
            if (!v[i].base || in[i].conv || i >= 32 || !(C->cva_mask >> i & 1) || C->cva[i].base != v[i].base) continue;
            in[i].off = C->cva[i].off + (lo - (uint32_t)C->cva_first) * (uint32_t)v[i].stride;
            v[i].base = NULL;
        }
    }
    g2r_gather(D.r, v, n, lo, span);
    for (i = 0; i < n; i++) if (v[i].base) in[i].off = v[i].off;
}

/* an element of an attribute as floats: what is on the PC when the PC has to look at a vertex. o keeps its defaults past s->size */
void gl_src_get(const src *s, uint32_t v, float *o)
{
    const uint8_t *e = s->base + (size_t)v * s->stride;
    int k;
    for (k = 0; k < s->size; k++) {
        switch (s->type) {
        case G2_AT_BYTE: o[k] = s->normalized ? (float)(2 * (double)((const int8_t *)e)[k] + 1) / 255.0f : (float)((const int8_t *)e)[k]; break;
        case G2_AT_UBYTE: o[k] = s->normalized ? (float)e[k] / 255.0f : (float)e[k]; break;
        case G2_AT_SHORT: o[k] = s->normalized ? (float)(2 * (double)((const int16_t *)e)[k] + 1) / 65535.0f : (float)((const int16_t *)e)[k]; break;
        case G2_AT_USHORT: o[k] = s->normalized ? (float)((const uint16_t *)e)[k] / 65535.0f : (float)((const uint16_t *)e)[k]; break;
        default: o[k] = ((const float *)e)[k]; break;
        }
    }
}

void gl_fill_hw_state(g2r_state *s)
{
    g2r_target t;
    int depth, sten, tri = C->drawing_cls == 0;
    uint32_t *h = s->h, i;
    double kx, ky;
    static const uint8_t SOP[] = { 0 };
    (void)SOP;
    gl_cur_target(&t);
    gl_scale(&kx, &ky);
    depth = C->depth_test && t.ds;
    sten = C->stencil_test && t.ds;
    memset(h, 0, 23 * 4);
    h[0] = (depth ? 1u : 0) | (depth && C->depth_mask ? 2u : 0) | (uint32_t)(C->depth_func & 7) << 2 | (sten ? 32u : 0) | (C->blend && !C->color_logic_op ? 64u : 0) | (C->dither ? 128u : 0) |
           (C->scissor_test ? 256u : 0) | (C->poly_offset_fill && tri ? 512u : 0) | (uint32_t)(C->color_logic_op && C->logic_op == GL_NOOP ? 0 : C->color_mask & 15) << 16;
    for (i = 0; i < 2; i++) {                                   /* h1 front, h2 back */
        h[1 + i] = (uint32_t)(C->st_ref[i] & 255) | (C->st_vmask[i] & 255) << 8 | (C->st_wmask[i] & 255) << 16 | (uint32_t)(C->st_func[i] & 7) << 24;
    }
    {
        static const GLenum codes[8] = { GL_KEEP, GL_ZERO, GL_REPLACE, GL_INCR, GL_DECR, GL_INVERT, GL_INCR_WRAP, GL_DECR_WRAP };
        uint32_t o[2][3];
        int f, k, c;
        for (f = 0; f < 2; f++) {
            GLenum e[3] = { C->st_fail[f], C->st_zfail[f], C->st_zpass[f] };
            for (k = 0; k < 3; k++) {
                o[f][k] = 0;
                for (c = 0; c < 8; c++) if (codes[c] == e[k]) o[f][k] = (uint32_t)c;
            }
        }
        h[3] = o[0][0] | o[0][1] << 3 | o[0][2] << 6 | o[1][0] << 16 | o[1][1] << 19 | o[1][2] << 22;
    }
    {
        extern uint32_t gl_blend_code(GLenum f), gl_blend_eq(GLenum e);
        uint32_t cr = g2_c16(C->blend_color[0], t.bits_r), cg = g2_c16(C->blend_color[1], t.bits_g), cb = g2_c16(C->blend_color[2], t.bits_b),
                 ca = g2_c16(C->blend_color[3], t.bits_a ? t.bits_a : 8);
        h[4] = gl_blend_code(C->blend_src) | gl_blend_code(C->blend_dst) << 4 | gl_blend_code(C->blend_src_a) << 8 | gl_blend_code(C->blend_dst_a) << 12 |
               gl_blend_eq(C->blend_eq) << 16 | gl_blend_eq(C->blend_eq_a) << 20;
        if (C->color_logic_op && C->logic_op == GL_INVERT) {    /* the program writes all ones: blending with 1 - dst is the complement of every bit */
            h[0] |= 64u;
            h[4] = gl_blend_code(GL_ONE_MINUS_DST_COLOR) | gl_blend_code(GL_ZERO) << 4 | gl_blend_code(GL_ONE_MINUS_DST_ALPHA) << 8 | gl_blend_code(GL_ZERO) << 12 | gl_blend_eq(GL_FUNC_ADD) << 16 | gl_blend_eq(GL_FUNC_ADD) << 20;
        }
        h[5] = cr | cg << 13;
        h[6] = cb | ca << 13;
        h[14] = g2_c16(C->blend_color[3], t.bits_r) | g2_c16(C->blend_color[3], t.bits_g) << 13;
    }
    {
        int b[4], sw = C->scissor_test;
        C->scissor_test = 1;                                    /* the box goes up whether the test is on or not, the enable is in h0 */
        gl_scissor_px(&t, b);
        C->scissor_test = sw;
        h[7] = (uint32_t)b[0] | (uint32_t)b[1] << 16;
        h[8] = (uint32_t)b[2] | (uint32_t)b[3] << 16;
    }
    h[9] = g2_fp24(C->poly_factor);
    h[10] = (uint32_t)(int32_t)C->poly_units;
    s->vp_x = C->vp[0] * kx; s->vp_y = C->vp[1] * ky; s->vp_w = C->vp[2] * kx; s->vp_h = C->vp[3] * ky;
    if (C->drawing_cls != 2) g2r_scissor_viewport(s, t.width, t.height);        /* a point inside stays whole: draw_prims */
    s->depth_near = C->depth_near; s->depth_far = C->depth_far;
    s->cull = C->cull_face && tri ? (C->cull_mode == GL_BACK ? 1 : C->cull_mode == GL_FRONT ? 2 : 3) : 0;      /* an antialiased line is triangles, which are not polygons */
    s->front_cw = C->front_face == GL_CW;
    s->line_width = C->line_width * ky;
}

/* GL_CLAMP and GL_CLAMP_TO_BORDER clamp to the edge here: the program adds the border (gl_ff.c) */
static uint32_t wrap_code(GLenum w)
{
    switch (w) {
    case GL_REPEAT: return G2R_WRAP_REPEAT;
    case GL_MIRRORED_REPEAT: return G2R_WRAP_MIRROR;
    default: return G2R_WRAP_CLAMP;
    }
}

static void fill_samplers(g2r_state *s, const ff_prog *p)
{
    int u;
    if (p->user_fs) gl_glsl_samplers(s, p);
    for (u = 0; u < MAX_UNITS && !p->user_fs; u++) {
        gl_tex *t;
        uint32_t mip, min_l;
        int oned = p->key.u[u].target == 3;
        if (!p->key.u[u].target || p->sampler[u] < 0) continue;
        t = gl_unit_tex(u, p->key.u[u].target == 2 ? 1 : p->key.u[u].target == 4 ? 3 : oned ? 2 : 0);
        gl_tex_prepare(t);
        if (p->key.u[u].sw) {                                   /* one texel, at the level the program says: the filter and the wrap are in the program */
            uint16_t im = p->key.u[u].sw == 3 ? t->handle : p->key.u[u].swb ? t->bimg : gl_tex_sample_image(t);
            s->h[15 + p->sampler[u]] = G2R_SAMPLER3D(im, G2R_FILTER(0, p->key.u[u].swn > 1 ? 1 : 0, 0), G2R_WRAP_CLAMP, G2R_WRAP_CLAMP, G2R_WRAP_CLAMP, 0);
            continue;
        }
        min_l = t->min_filter == GL_LINEAR || t->min_filter == GL_LINEAR_MIPMAP_NEAREST || t->min_filter == GL_LINEAR_MIPMAP_LINEAR;
        mip = t->min_filter == GL_NEAREST_MIPMAP_NEAREST || t->min_filter == GL_LINEAR_MIPMAP_NEAREST ? 1 :
              t->min_filter == GL_NEAREST_MIPMAP_LINEAR || t->min_filter == GL_LINEAR_MIPMAP_LINEAR ? 2 : 0;
        if (p->key.u[u].target == 4) s->h[15 + p->sampler[u]] = G2R_SAMPLER3D(gl_tex_sample_image(t), G2R_FILTER(t->mag_filter == GL_LINEAR, mip, min_l), wrap_code(t->wrap_s), wrap_code(t->wrap_t), wrap_code(t->wrap_r), 0);
        else if (p->key.u[u].bimg) s->h[15 + p->sampler[u]] = G2R_SAMPLER(t->bimg, G2R_FILTER(t->mag_filter == GL_LINEAR, 0, min_l), G2R_WRAP_CLAMP, G2R_WRAP_CLAMP, 0);     /* the program wraps */
        else s->h[15 + p->sampler[u]] = G2R_SAMPLER(gl_tex_sample_image(t), G2R_FILTER(t->mag_filter == GL_LINEAR, mip, min_l), wrap_code(t->wrap_s), oned ? G2R_WRAP_CLAMP : wrap_code(t->wrap_t), 0);
    }
    if (p->stip_sampler >= 0) s->h[15 + p->stip_sampler] = G2R_SAMPLER(gl_stipple_image(p->key.line_stip), G2R_FILTER(0, 0, 0), G2R_WRAP_REPEAT, G2R_WRAP_REPEAT, 0);
}

/* one DRAW: in[] = the attributes, indexed by A_*; idx NULL: n vertices from `first` */
static void emit_one(const ff_prog *p, src *in, int hwmode, const uint32_t *idx, uint32_t n, uint32_t first)
{
    g2r_state s;
    uint32_t lo = first, hi = first + n - 1, i, vu[4 * 256], fu[4 * 64];
    int a;
    uint16_t *ib = NULL;
    uint32_t ioff = 0;
    if (!n) return;
    if (idx) {
        lo = 0xFFFFFFFFu; hi = 0;
        for (i = 0; i < n; i++) {
            if (idx[i] < lo) lo = idx[i];
            if (idx[i] > hi) hi = idx[i];
        }
        ib = (uint16_t *)malloc((size_t)n * 2);
        if (!ib) return;
        for (i = 0; i < n; i++) ib[i] = (uint16_t)(idx[i] - lo);
    }
    gather(in, A_COUNT, lo, hi - lo + 1);
    if (idx) { ioff = g2r_stream(D.r, ib, n * 2, 2); free(ib); }
    memset(&s, 0, sizeof s);
    gl_fill_hw_state(&s);
    s.prog = p->p;
    fill_samplers(&s, p);
    for (a = 0; a < 16; a++) { s.attr[a].size = 4; s.attr[a].type = G2_AT_FP24; s.attr[a].constant[3] = g2_fp24(1.0); }
    for (a = 0; a < A_COUNT; a++) {
        int l = p->attr_loc[a];
        g2r_attr *d;
        if (l < 0) continue;
        d = &s.attr[l];
        if (in[a].base) {
            d->enabled = 1; d->size = in[a].size; d->type = in[a].type; d->normalized = in[a].normalized;
            d->handle = in[a].handle;
            d->stride = (uint32_t)in[a].stride;
            d->offset = in[a].off;
        } else {
            memcpy(d->constant, in[a].constant, 16);
        }
    }
    memset(vu, 0, (size_t)p->p->vs_const_base * 16);
    memset(fu, 0, (size_t)p->p->fs_const_base * 16);
    gl_ff_uniforms(p, vu, fu);
    if (p->user) gl_glsl_uniforms(p, vu, fu);
    s.vs_uniforms = vu;
    s.fs_uniforms = fu;
    C->draws++;
    g2r_draw(D.r, &s, hwmode, n, idx ? 0 : 0, idx ? 1 : 0, g2r_stream_handle(D.r), ioff, 0);
}

static uint8_t logic8(GLenum op, uint8_t s, uint8_t d)
{
    switch (op) {
    case GL_AND: return (uint8_t)(s & d);
    case GL_AND_REVERSE: return (uint8_t)(s & ~d);
    case GL_AND_INVERTED: return (uint8_t)(~s & d);
    case GL_XOR: return (uint8_t)(s ^ d);
    case GL_OR: return (uint8_t)(s | d);
    case GL_NOR: return (uint8_t)~(s | d);
    case GL_EQUIV: return (uint8_t)~(s ^ d);
    case GL_OR_REVERSE: return (uint8_t)(s | ~d);
    case GL_OR_INVERTED: return (uint8_t)(~s | d);
    case GL_NAND: return (uint8_t)~(s & d);
    default: return s;
    }
}

/* the same on the words of a 16-bit target: §4.1.10 works on the bits the framebuffer holds */
static uint16_t logic16(GLenum op, uint32_t s, uint32_t d)
{
    switch (op) {
    case GL_AND: return (uint16_t)(s & d);
    case GL_AND_REVERSE: return (uint16_t)(s & ~d);
    case GL_AND_INVERTED: return (uint16_t)(~s & d);
    case GL_XOR: return (uint16_t)(s ^ d);
    case GL_OR: return (uint16_t)(s | d);
    case GL_NOR: return (uint16_t)~(s | d);
    case GL_EQUIV: return (uint16_t)~(s ^ d);
    case GL_OR_REVERSE: return (uint16_t)(s | ~d);
    case GL_OR_INVERTED: return (uint16_t)(~s | d);
    case GL_NAND: return (uint16_t)~(s & d);
    default: return (uint16_t)s;
    }
}

/* The logic ops that work bit by bit have no path through B's blender, so the PC does them: slow, a few read-backs a draw, and exact. The draw runs twice over two known backgrounds, so a
   pixel that comes out the same over both is one the draw wrote, whatever colour it has; then the result goes back, and a last pass with the colour masked does the depth and stencil writes.
   A 16-bit target goes by its words, the colour mask by the bits of each channel */
static void emit(const ff_prog *p, src *in, int hwmode, const uint32_t *idx, uint32_t n, uint32_t first)
{
    g2r_target t;
    uint8_t *d, *s0, *s1;
    uint32_t i, px, fmt;
    int cm = C->color_mask, dm = C->depth_mask, bl = C->blend, w16;
    GLuint wm[2];
    if (!C->color_logic_op || C->logic_op == GL_COPY || C->logic_op == GL_NOOP || C->logic_op == GL_CLEAR || C->logic_op == GL_SET || C->logic_op == GL_COPY_INVERTED || C->logic_op == GL_INVERT) {
        emit_one(p, in, hwmode, idx, n, first);
        return;
    }
    gl_cur_target(&t);
    fmt = gl_tg_fmt(&t);
    w16 = gl_fmt_bpp(fmt) == 2;
    px = t.width * t.height;
    d = (uint8_t *)malloc((size_t)px * 4); s0 = (uint8_t *)malloc((size_t)px * 4); s1 = (uint8_t *)malloc((size_t)px * 4);
    if (!d || !s0 || !s1) { free(d); free(s0); free(s1); SET_ERROR(GL_OUT_OF_MEMORY); return; }
    wm[0] = C->st_wmask[0]; wm[1] = C->st_wmask[1];
    C->color_logic_op = 0; C->blend = 0; C->depth_mask = 0; C->st_wmask[0] = C->st_wmask[1] = 0; C->color_mask = 15;
    g2r_finish(D.r);
    g2_read_image(D.bd, t.color, w16 ? G2_TF_RAW : G2_TF_RGBA8888, 0, 0, t.width, t.height, 0, 0, fmt, 0, 0, d);
    g2_fill_image(D.bd, t.color, G2_TF_RAW, 0x00000000u, 0, 0, t.width, t.height, 0xF, 0, 0);
    emit_one(p, in, hwmode, idx, n, first);
    g2r_finish(D.r);
    g2_read_image(D.bd, t.color, w16 ? G2_TF_RAW : G2_TF_RGBA8888, 0, 0, t.width, t.height, 0, 0, fmt, 0, 0, s0);
    g2_fill_image(D.bd, t.color, G2_TF_RAW, 0xFFFFFFFFu, 0, 0, t.width, t.height, 0xF, 0, 0);
    emit_one(p, in, hwmode, idx, n, first);
    g2r_finish(D.r);
    g2_read_image(D.bd, t.color, w16 ? G2_TF_RAW : G2_TF_RGBA8888, 0, 0, t.width, t.height, 0, 0, fmt, 0, 0, s1);
    if (w16) {
        uint16_t *dw = (uint16_t *)d, *a = (uint16_t *)s0, *b = (uint16_t *)s1;
        uint32_t keep = 0;
        int c, sh = 16;
        for (c = 0; c < 4; c++) {                               /* the channels from the top bit down, as B packs them */
            int nb = gl_fmt_bits(fmt, c);
            sh -= nb;
            if (nb && (cm >> (3 - c) & 1)) keep |= ((1u << nb) - 1) << sh;
        }
        for (i = 0; i < px; i++) if (a[i] == b[i]) dw[i] = (uint16_t)((dw[i] & ~keep) | (logic16(C->logic_op, a[i], dw[i]) & keep));
    } else {
        for (i = 0; i < px; i++) {
            int c;
            if (memcmp(s0 + (size_t)i * 4, s1 + (size_t)i * 4, 4)) continue;
            for (c = 0; c < 4; c++) if (cm >> (3 - c) & 1) d[(size_t)i * 4 + c] = logic8(C->logic_op, s0[(size_t)i * 4 + c], d[(size_t)i * 4 + c]);
        }
    }
    g2_write_image(D.bd, t.color, d, (size_t)px * (w16 ? 2 : 4), w16 ? G2_TF_RAW : G2_TF_RGBA8888, 0, 0, t.width, t.height, 0, 0, 0, fmt);
    free(d); free(s0); free(s1);
    C->blend = bl; C->depth_mask = dm; C->st_wmask[0] = wm[0]; C->st_wmask[1] = wm[1];
    if (dm || wm[0] || wm[1]) {
        C->color_mask = 0;
        emit_one(p, in, hwmode, idx, n, first);
    }
    C->color_mask = cm;
    C->color_logic_op = 1;
}

/* expand to n separate vertices (non-indexed): for GL_FLAT, where `prim` = 3 or 2 and colour and normal come from the primitive's last vertex, or from prov[i] for what a polygon became; and for draws
   that cannot be rebased to u16 */
static void emit_expanded(const ff_prog *p, src *in, int hwmode, const uint32_t *idx, uint32_t n, int prim, const uint32_t *prov)
{
    src out[A_COUNT];
    void *tmp[A_COUNT];
    int a;
    uint32_t i;
    memset(tmp, 0, sizeof tmp);
    for (a = 0; a < A_COUNT; a++) {
        out[a] = in[a];
        if (!in[a].base) continue;
        tmp[a] = malloc((size_t)n * in[a].bytes);
        if (!tmp[a]) goto done;
        for (i = 0; i < n; i++) {
            uint32_t v = a == A_STIP || a >= A_FLAT ? i : idx[i];   /* the stipple distance belongs to the list, not to a vertex: a strip's vertex has two ends */
            if (a == A_COLOR || a == A_NORMAL) {
                if (prov) v = prov[i];
                else if (prim > 1) v = idx[i - i % (uint32_t)prim + (uint32_t)prim - 1];
            }
            memcpy((uint8_t *)tmp[a] + (size_t)i * in[a].bytes, in[a].base + (size_t)v * in[a].stride, (size_t)in[a].bytes);
        }
        out[a].base = (const uint8_t *)tmp[a];
        out[a].stride = in[a].bytes;
        out[a].conv = 1;
    }
    emit(p, out, hwmode, NULL, n, 0);
done:
    for (a = 0; a < A_COUNT; a++) free(tmp[a]);
}

/* The list of a primitive mode as triangles / lines / points indices; returns the count.
 * tri, if not NULL, gets per triangle what it says about the polygon it came from, for polygon mode: bits 0..2, which of its edges, vertex k to the next, are the polygon's and not a diagonal; bit 3, the edge
 * flags count (§2.6.2: not for strips and fans). Bits are in the order of the triangle, which starts each of the polygon's edges from the vertex the polygon has it start from */
static uint32_t to_list(GLenum mode, const uint32_t *in, uint32_t first, uint32_t n, uint32_t *out, uint8_t *tri)
{
    uint32_t o = 0, i, t = 0;
#define V(k) (in ? in[k] : first + (k))
#define TRI(m) do { if (tri) tri[t++] = (uint8_t)(m); } while (0)
    switch (mode) {
    case GL_TRIANGLES: for (i = 0; i + 2 < n; i += 3) { out[o++] = V(i); out[o++] = V(i + 1); out[o++] = V(i + 2); TRI(15); } break;
    case GL_TRIANGLE_STRIP:
        for (i = 0; i + 2 < n; i++) {
            if (i & 1) { out[o++] = V(i + 1); out[o++] = V(i); out[o++] = V(i + 2); }
            else { out[o++] = V(i); out[o++] = V(i + 1); out[o++] = V(i + 2); }
            TRI(7);
        }
        break;
    case GL_TRIANGLE_FAN: for (i = 1; i + 1 < n; i++) { out[o++] = V(0); out[o++] = V(i); out[o++] = V(i + 1); TRI(7); } break;
    case GL_POLYGON:                                            /* GL_FLAT: a polygon's first vertex */
        for (i = 1; i + 1 < n; i++) {
            out[o++] = V(i); out[o++] = V(i + 1); out[o++] = V(0);
            TRI(9 | (i + 2 == n ? 2 : 0) | (i == 1 ? 4 : 0));
        }
        break;
    case GL_QUADS:
        for (i = 0; i + 3 < n; i += 4) {
            out[o++] = V(i); out[o++] = V(i + 1); out[o++] = V(i + 3); TRI(13);
            out[o++] = V(i + 1); out[o++] = V(i + 2); out[o++] = V(i + 3); TRI(11);
        }
        break;
    case GL_QUAD_STRIP:
        for (i = 0; i + 3 < n; i += 2) {
            out[o++] = V(i); out[o++] = V(i + 1); out[o++] = V(i + 3); TRI(3);
            out[o++] = V(i); out[o++] = V(i + 3); out[o++] = V(i + 2); TRI(6);
        }
        break;
    case GL_LINES: for (i = 0; i + 1 < n; i += 2) { out[o++] = V(i); out[o++] = V(i + 1); } break;
    case GL_LINE_STRIP: for (i = 0; i + 1 < n; i++) { out[o++] = V(i); out[o++] = V(i + 1); } break;
    case GL_LINE_LOOP:
        for (i = 0; i + 1 < n; i++) { out[o++] = V(i); out[o++] = V(i + 1); }
        if (n > 1) { out[o++] = V(n - 1); out[o++] = V(0); }
        break;
    default: for (i = 0; i < n; i++) out[o++] = V(i); break;
    }
#undef V
#undef TRI
    return o;
}

static int prim_class(GLenum mode)
{
    switch (mode) {
    case GL_POINTS: return 2;
    case GL_LINES: case GL_LINE_STRIP: case GL_LINE_LOOP: return 1;
    default: return 0;
    }
}

static uint32_t list_capacity(uint32_t n) { return n * 6 + 6; }

/* where vertex v lands in clip space: a user's vertex shader says, or the matrices */
static void clip_of(const src *in, const float *mvp, uint32_t v, float *c)
{
    float o[4] = { 0, 0, 0, 1 };
    int r;
    if (gl_vs_clip(in, v, c)) return;
    gl_src_get(&in[A_POS], v, o);
    for (r = 0; r < 4; r++) c[r] = mvp[r] * o[0] + mvp[r + 4] * o[1] + mvp[r + 8] * o[2] + mvp[r + 12] * o[3];
}

/* A stippled line's counter runs in window pixels along the major axis (§3.4.2), through a strip and from 0 at each independent line. The hardware has no stipple: the distance goes up as an attribute
   of each end, and the program looks the bit up. The attribute is per list entry, so every line gets its own two vertices */
static void emit_stippled(const ff_prog *p, src *in, const uint32_t *list, uint32_t n, int hw, int prim, const uint32_t *prov)
{
    float mvp[16], *dist = (float *)malloc((size_t)n * sizeof(float)), s = 0;
    uint32_t i;
    if (!dist) return;
    gl_mul(mvp, C->pr[C->pr_sp], C->mv[C->mv_sp]);
    for (i = 0; i + 1 < n; i += 2) {
        float w[2][2], m;
        int e;
        for (e = 0; e < 2; e++) {
            float c[4];
            clip_of(in, mvp, list[i + (uint32_t)e], c);
            if (c[3] < 1e-6f) c[3] = 1e-6f;
            w[e][0] = (c[0] / c[3] * 0.5f + 0.5f) * (float)C->vp[2];
            w[e][1] = (c[1] / c[3] * 0.5f + 0.5f) * (float)C->vp[3];
        }
        if (i < 2 || list[i] != list[i - 1]) s = 0;
        m = fabsf(w[1][0] - w[0][0]);
        if (fabsf(w[1][1] - w[0][1]) > m) m = fabsf(w[1][1] - w[0][1]);
        dist[i] = s;
        s += m;
        dist[i + 1] = s;
    }
    memset(&in[A_STIP], 0, sizeof in[A_STIP]);
    in[A_STIP].base = (const uint8_t *)dist; in[A_STIP].stride = 4; in[A_STIP].bytes = 4; in[A_STIP].size = 1; in[A_STIP].type = G2_AT_FLOAT; in[A_STIP].conv = 1;
    emit_expanded(p, in, hw, list, n, prim, prov);
    in[A_STIP].base = NULL;
    free(dist);
}


/* An antialiased line is a rectangle as wide as the line, on the segment (§3.4.4). The hardware has only its own lines, so each line becomes four corners a pixel outside that rectangle, pushed out in the vertex
 * program by aLine.xy, in normalized x and y so it stays in the plane of the line under any w. The fragment program takes the cover from the distance to the centre line, aLine.z, and from the ends, aStip.yz.
 * aStip.x is the stipple counter. A strip's vertex has two ends here too, so every line gets its own corners */
#define SMOOTH_LINES 16000
static void emit_smooth(const ff_prog *p, src *in, const uint32_t *list, uint32_t n, const uint32_t *prov, int flat)
{
    float mvp[16], *ln, *st, s = 0.0f, ax = 0.0f, ay = 0.0f, hw;
    double kx, ky;
    uint32_t lines = n / 2, nv, l, i, *vi = NULL, *pv = NULL, *ib = NULL;
    void *tmp[A_COUNT];
    src out[A_COUNT];
    int a;
    if (lines > SMOOTH_LINES) {
        emit_smooth(p, in, list, SMOOTH_LINES * 2, prov, flat);
        emit_smooth(p, in, list + SMOOTH_LINES * 2, n - SMOOTH_LINES * 2, prov ? prov + SMOOTH_LINES * 2 : NULL, flat);
        return;
    }
    nv = lines * 4;
    gl_scale(&kx, &ky);
    gl_mul(mvp, C->pr[C->pr_sp], C->mv[C->mv_sp]);
    hw = (float)(C->line_width * ky) * 0.5f + 1.0f;
    ax = (float)(C->vp[2] * kx); ay = (float)(C->vp[3] * ky);
    ln = (float *)malloc((size_t)nv * 16); st = (float *)malloc((size_t)nv * 12);
    vi = (uint32_t *)malloc((size_t)nv * 4); pv = (uint32_t *)malloc((size_t)nv * 4); ib = (uint32_t *)malloc((size_t)lines * 24);
    memset(tmp, 0, sizeof tmp);
    if (!ln || !st || !vi || !pv || !ib) goto done;
    for (l = 0; l < lines; l++) {
        float c0[4], c1[4], x0, y0, x1, y1, dx, dy, len, m;
        uint32_t a0 = list[2 * l], a1 = list[2 * l + 1], v = 4 * l;
        int e, side;
        clip_of(in, mvp, a0, c0);
        clip_of(in, mvp, a1, c1);
        if (c0[3] < 1e-6f || c1[3] < 1e-6f) {                   /* through the eye: the direction comes from the part in front */
            float t = (1e-6f - c0[3]) / (c1[3] - c0[3]);
            if (c0[3] < 1e-6f && c1[3] < 1e-6f) t = 0.0f;
            for (e = 0; e < 4; e++) { float q = c0[e] + (c1[e] - c0[e]) * t; if (c0[3] < 1e-6f) c0[e] = q; else c1[e] = q; }
            if (c0[3] < 1e-6f) c0[3] = 1e-6f;
            if (c1[3] < 1e-6f) c1[3] = 1e-6f;
        }
        x0 = c0[0] / c0[3] * 0.5f * ax; y0 = c0[1] / c0[3] * 0.5f * ay;
        x1 = c1[0] / c1[3] * 0.5f * ax; y1 = c1[1] / c1[3] * 0.5f * ay;
        dx = x1 - x0; dy = y1 - y0;
        len = sqrtf(dx * dx + dy * dy);
        if (l == 0 || a0 != list[2 * l - 1]) s = 0.0f;
        m = fabsf(dx) > fabsf(dy) ? fabsf(dx) : fabsf(dy);
        if (len > 0.0f) { dx /= len; dy /= len; }
        else { dx = 1.0f; dy = 0.0f; }
        for (e = 0; e < 2; e++) {
            for (side = 0; side < 2; side++, v++) {
                float sg = side ? 1.0f : -1.0f, ex = e ? 1.0f : -1.0f;
                ln[4 * v] = (-dy * sg * hw + dx * ex) * 2.0f / ax;
                ln[4 * v + 1] = (dx * sg * hw + dy * ex) * 2.0f / ay;
                ln[4 * v + 2] = sg * hw;
                ln[4 * v + 3] = 0.0f;
                st[3 * v] = e ? s + m + 1.0f : s - 1.0f;
                st[3 * v + 1] = e ? len + 1.0f : -1.0f;
                st[3 * v + 2] = e ? -1.0f : len + 1.0f;
                vi[v] = e ? a1 : a0;
                pv[v] = prov ? prov[2 * l] : a1;
            }
        }
        s += m;
        v = 4 * l;
        ib[6 * l] = v; ib[6 * l + 1] = v + 1; ib[6 * l + 2] = v + 3; ib[6 * l + 3] = v; ib[6 * l + 4] = v + 3; ib[6 * l + 5] = v + 2;
    }
    for (a = 0; a < A_COUNT; a++) {
        out[a] = in[a];
        if (!in[a].base) continue;
        tmp[a] = malloc((size_t)nv * in[a].bytes);
        if (!tmp[a]) goto done;
        for (i = 0; i < nv; i++) memcpy((uint8_t *)tmp[a] + (size_t)i * in[a].bytes, in[a].base + (size_t)(a >= A_FLAT ? i / 4 * 2 : flat && (a == A_COLOR || a == A_NORMAL) ? pv[i] : vi[i]) * in[a].stride, (size_t)in[a].bytes);
        out[a].base = (const uint8_t *)tmp[a];
        out[a].stride = in[a].bytes;
        out[a].conv = 1;
    }
    memset(&out[A_LINE], 0, sizeof out[A_LINE]);
    out[A_LINE].base = (const uint8_t *)ln; out[A_LINE].stride = 16; out[A_LINE].bytes = 16; out[A_LINE].size = 4; out[A_LINE].type = G2_AT_FLOAT; out[A_LINE].conv = 1;
    memset(&out[A_STIP], 0, sizeof out[A_STIP]);
    out[A_STIP].base = (const uint8_t *)st; out[A_STIP].stride = 12; out[A_STIP].bytes = 12; out[A_STIP].size = 3; out[A_STIP].type = G2_AT_FLOAT; out[A_STIP].conv = 1;
    emit(p, out, 0, ib, lines * 6, 0);
done:
    for (a = 0; a < A_COUNT; a++) free(tmp[a]);
    free(ln); free(st); free(vi); free(pv); free(ib);
}

/* a list of triangles / lines / points through whatever the state asks for. prov: what polygon mode made, each vertex's polygon's provoking vertex */
static void draw_prims(src *in, int cls, uint32_t *list, uint32_t n, uint32_t *prov)
{
    const ff_prog *p;
    int flat = C->shade_model == GL_FLAT && (in[A_COLOR].base || in[A_NORMAL].base || gl_glsl_user_vs());
    uint32_t i, lo = 0xFFFFFFFFu, hi = 0;
    float *fl = NULL;
    int size = cls == 0 ? 3 : cls == 1 ? 2 : 1, hw = cls == 0 ? 0 : cls == 1 ? 4 : 3;
    if (!n) return;
    C->vs_serial++;
    gl_bind_target();
    if (C->render_mode != GL_RENDER) {
        gl_select_prims(in, cls, list, n);
        return;
    }
    if (cls == 2 && !gl_glsl_user_vs()) {                       /* §2.12: a point is whole or gone by its centre, and board A goes by near and far only; a user's vertex shader does this in its wrapped main */
        float mvp[16], c[4];
        uint32_t m = 0;
        gl_mul(mvp, C->pr[C->pr_sp], C->mv[C->mv_sp]);
        for (i = 0; i < n; i++) {
            clip_of(in, mvp, list[i], c);
            if (fabsf(c[0]) > c[3] || fabsf(c[1]) > c[3]) continue;
            list[m] = list[i];
            if (prov) prov[m] = prov[i];
            m++;
        }
        n = m;
        if (!n) return;
    }
    C->drawing_points = cls == 2;
    C->drawing_cls = cls;
    p = C->cur && (!C->internal_draw || gl_glsl_user_fs()) ? gl_glsl_program() : gl_ff_program();
    if (p) {
        const uint32_t *fp = flat ? prov : NULL;
        for (i = 0; i < n; i++) {
            if (list[i] < lo) lo = list[i];
            if (list[i] > hi) hi = list[i];
        }
        if (flat && p->user && p->attr_loc[A_FLAT] >= 0 && (fl = (float *)malloc((size_t)n * 64)) != NULL) {     /* §2.14.7: what the shader said of the provoking vertex: the hardware interpolates all it is given */
            uint32_t last = 0xFFFFFFFFu;
            int a;
            C->no_flat = 1;
            for (i = 0; i < n; i++) {
                uint32_t v = fp ? fp[i] : list[i - i % (uint32_t)size + (uint32_t)size - 1];
                if (v != last) gl_vs_flat(in, v, fl + (size_t)i * 16);
                else memcpy(fl + (size_t)i * 16, fl + ((size_t)i - 1) * 16, 64);
                last = v;
            }
            C->no_flat = 0;
            for (a = 0; a < 4; a++) {
                src *d = &in[A_FLAT + a];
                memset(d, 0, sizeof *d);
                d->base = (const uint8_t *)(fl + a * 4); d->stride = 64; d->bytes = 16; d->size = 4; d->type = G2_AT_FLOAT; d->conv = 1;
            }
        }
        if (p->key.smooth_ln) emit_smooth(p, in, list, n, prov, flat);
        else if (p->key.line_stip) emit_stippled(p, in, list, n, hw, flat ? 2 : 1, fp);
        else if (flat && (size > 1 || fp)) emit_expanded(p, in, hw, list, n, size, fp);
        else if (hi - lo >= 65536) emit_expanded(p, in, hw, list, n, 1, NULL);
        else emit(p, in, hw, list, n, 0);
        if (fl) {
            int a;
            for (a = 0; a < 4; a++) in[A_FLAT + a].base = NULL;
            free(fl);
        }
    }
}

/* what polygon mode needs to know of a list of triangles: to_list's tri, and the edge flags by vertex; ef NULL: the current flag */
typedef struct polyinfo { const uint8_t *tri, *ef; int stride; } polyinfo;

/* Polygon mode comes after culling and front and back have their own (§3.5.4), so which way a polygon faces is worked out here: the sign of the determinant of x, y, w is the winding for any w, so a polygon
 * through the near plane needs no clipping first. Lines and points are the polygon's own edges, the diagonals of the triangles it became are not drawn, and a false edge flag drops the edge from its vertex.
 * Polygons keep their order: a change of mode ends the batch. An offset for lines and points goes per polygon through the vertex program, as the slope is the polygon's and the hardware's offset is for triangles */
static void draw_polygons(src *in, uint32_t *list, uint32_t n, const polyinfo *g)
{
    float mvp[16];
    double kx, ky;
    uint32_t *l = (uint32_t *)malloc(((size_t)n * 2 + 6) * 4), *pv = (uint32_t *)malloc(((size_t)n * 2 + 6) * 4), i, m = 0;
    GLenum run = 0;
    int run_back = 0;
    if (!l || !pv) { free(l); free(pv); return; }
    C->vs_serial++;
    gl_mul(mvp, C->pr[C->pr_sp], C->mv[C->mv_sp]);
    gl_scale(&kx, &ky);
#define RUN_END() do { if (m) { C->drawing_back = run_back; draw_prims(in, run == GL_LINE ? 1 : run == GL_POINT ? 2 : 0, l, m, run == GL_FILL ? NULL : pv); C->drawing_back = 0; C->z_off = 0.0f; m = 0; } } while (0)
    for (i = 0; i + 2 < n; i += 3) {
        float c[3][4], det;
        GLenum mode;
        int e, front, back, off;
        uint8_t mask = g->tri[i / 3];
        for (e = 0; e < 3; e++) clip_of(in, mvp, list[i + (uint32_t)e], c[e]);
        det = c[0][0] * (c[1][1] * c[2][3] - c[2][1] * c[1][3]) - c[0][1] * (c[1][0] * c[2][3] - c[2][0] * c[1][3]) + c[0][3] * (c[1][0] * c[2][1] - c[2][0] * c[1][1]);
        front = (det > 0.0f) == (C->front_face == GL_CCW);
        if (C->cull_face && (C->cull_mode == GL_FRONT_AND_BACK || (C->cull_mode == GL_BACK && !front) || (C->cull_mode == GL_FRONT && front))) continue;
        mode = front ? C->poly_mode_front : C->poly_mode_back;
        back = mode != GL_FILL && !front && C->lighting && C->lm_two_side;
        off = (mode == GL_LINE && C->poly_offset_line) || (mode == GL_POINT && C->poly_offset_point);
        if (m && (mode != run || back != run_back)) RUN_END();
        run = mode; run_back = back;
        if (mode == GL_FILL) {
            for (e = 0; e < 3; e++) l[m++] = list[i + (uint32_t)e];
        } else {
            for (e = 0; e < 3; e++) {
                uint32_t v = list[i + (uint32_t)e], w = list[i + (uint32_t)(e + 1) % 3], prov = list[i + 2];
                if (!(mask >> e & 1)) continue;
                if ((mask & 8) && !(g->ef ? g->ef[(size_t)v * g->stride] : C->edge_flag)) continue;
                l[m] = v; pv[m++] = prov;
                if (mode == GL_LINE) { l[m] = w; pv[m++] = prov; }
            }
        }
        if (off && c[0][3] > 0.0f && c[1][3] > 0.0f && c[2][3] > 0.0f) {                 /* §3.5.5: m the polygon's largest slope in z, in the target's pixels */
            float x[3], y[3], z[3], a, b, d, slope = 0.0f, dz;
            for (e = 0; e < 3; e++) {
                x[e] = c[e][0] / c[e][3] * 0.5f * (float)(C->vp[2] * kx);
                y[e] = c[e][1] / c[e][3] * 0.5f * (float)(C->vp[3] * ky);
                z[e] = (c[e][2] / c[e][3] * 0.5f + 0.5f) * (C->depth_far - C->depth_near);
            }
            d = (x[1] - x[0]) * (y[2] - y[0]) - (y[1] - y[0]) * (x[2] - x[0]);
            if (d != 0.0f) {
                a = ((y[1] - y[0]) * (z[2] - z[0]) - (z[1] - z[0]) * (y[2] - y[0])) / d;
                b = ((z[1] - z[0]) * (x[2] - x[0]) - (x[1] - x[0]) * (z[2] - z[0])) / d;
                slope = fabsf(a) > fabsf(b) ? fabsf(a) : fabsf(b);
            }
            {
                g2r_target tg;
                gl_cur_target(&tg);
                dz = slope * C->poly_factor + C->poly_units / (tg.depth16 ? 65536.0f : 16777216.0f);     /* r, the depth buffer's smallest step */
            }
            C->z_off = C->depth_far != C->depth_near ? 2.0f * dz / (C->depth_far - C->depth_near) : 0.0f;
            RUN_END();
        }
    }
    RUN_END();
#undef RUN_END
    free(l); free(pv);
}

static void draw_list(src *in, int cls, uint32_t *list, uint32_t n, const polyinfo *g)
{
    if (cls == 0 && g && g->tri) draw_polygons(in, list, n, g);
    else draw_prims(in, cls, list, n, NULL);
}

static int poly_filled(void) { return C->poly_mode_front == GL_FILL && C->poly_mode_back == GL_FILL; }

static void arrays_to_src(src *in, void **tmp, uint32_t lo, uint32_t n)
{
    int u;
    if (C->cur && C->a_attr[0].enabled) tmp[A_POS] = resolve(&C->a_attr[0], &in[A_POS], C->a_attr[0].normalized, lo, n);     /* §2.7: generic 0 before gl_Vertex */
    else tmp[A_POS] = resolve(&C->a_vertex, &in[A_POS], 0, lo, n);
    if (C->a_color.enabled) tmp[A_COLOR] = resolve(&C->a_color, &in[A_COLOR], 1, lo, n);
    else constant(&in[A_COLOR], C->color, 4);
    if (C->a_normal.enabled) { C->a_normal.size = 3; tmp[A_NORMAL] = resolve(&C->a_normal, &in[A_NORMAL], 1, lo, n); }
    else constant(&in[A_NORMAL], C->normal, 3);
    if (C->a_fog.enabled) tmp[A_FOGC] = resolve(&C->a_fog, &in[A_FOGC], 0, lo, n);
    else constant(&in[A_FOGC], &C->fog_coord, 1);
    if (C->a_sec.enabled) tmp[A_SEC] = resolve(&C->a_sec, &in[A_SEC], 1, lo, n);
    else constant(&in[A_SEC], C->sec_color, 3);
    for (u = 0; u < MAX_UNITS; u++) {
        if (C->a_tc[u].enabled) tmp[A_TC0 + u] = resolve(&C->a_tc[u], &in[A_TC0 + u], 0, lo, n);
        else constant(&in[A_TC0 + u], C->unit[u].tc, 4);
    }
    for (u = 1; C->cur && u < MAX_ATTRIBS; u++) {
        if (C->a_attr[u].enabled) tmp[A_GEN + u - 1] = resolve(&C->a_attr[u], &in[A_GEN + u - 1], C->a_attr[u].normalized, lo, n);
        else constant(&in[A_GEN + u - 1], C->attr[u], 4);
    }
}

void gl_src_current(src *in)
{
    int u;
    constant(&in[A_COLOR], C->color, 4);
    constant(&in[A_NORMAL], C->normal, 3);
    constant(&in[A_FOGC], &C->fog_coord, 1);
    constant(&in[A_SEC], C->sec_color, 3);
    for (u = 0; u < MAX_UNITS; u++) constant(&in[A_TC0 + u], C->unit[u].tc, 4);
    for (u = 1; u < MAX_ATTRIBS; u++) constant(&in[A_GEN + u - 1], C->attr[u], 4);
}

/* polygon mode's view of a draw from arrays. *tri: room for to_list's bytes, only when polygons are not simply filled */
static void poly_arrays(polyinfo *g, int cls, uint8_t **tri, uint32_t count)
{
    memset(g, 0, sizeof *g);
    if (cls || poly_filled()) return;
    *tri = (uint8_t *)malloc((size_t)list_capacity(count));
    g->tri = *tri;
    if (C->a_edge.enabled) {
        const uint8_t *b = (const uint8_t *)C->a_edge.ptr;
        if (C->a_edge.buffer) { gl_buffer *o = gl_buffer_lookup(C->a_edge.buffer); b = o && o->data ? o->data + (uintptr_t)C->a_edge.ptr : NULL; }
        g->ef = b;
        g->stride = C->a_edge.stride ? C->a_edge.stride : 1;
    }
}

void gl_draw_arrays(GLenum mode, GLint first, GLsizei count)
{
    src in[A_COUNT];
    void *tmp[A_COUNT];
    int a, cls = prim_class(mode);
    if (count < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (!count || !(C->a_vertex.enabled || (C->cur && C->a_attr[0].enabled)) || !C->live) return;
    FLUSH_VERTICES();
    memset(tmp, 0, sizeof tmp); memset(in, 0, sizeof in);
    arrays_to_src(in, tmp, (uint32_t)first, (uint32_t)count);
    if (in[A_POS].base) {
        uint32_t *list = (uint32_t *)malloc((size_t)list_capacity((uint32_t)count) * sizeof *list);
        if (list) {
            polyinfo g;
            uint8_t *tri = NULL;
            uint32_t n;
            poly_arrays(&g, cls, &tri, (uint32_t)count);
            n = to_list(mode, NULL, (uint32_t)first, (uint32_t)count, list, tri);
            draw_list(in, cls, list, n, &g);
            free(list); free(tri);
        }
    }
    for (a = 0; a < A_COUNT; a++) free(tmp[a]);
}

void gl_draw_elements(GLenum mode, GLsizei count, GLenum type, const void *indices, GLuint start, GLuint end, int have_range)
{
    src in[A_COUNT];
    void *tmp[A_COUNT];
    uint32_t *idx, *list, i, lo = 0xFFFFFFFFu, hi = 0;
    const uint8_t *ip = (const uint8_t *)indices;
    int a, cls = prim_class(mode);
    (void)start; (void)end; (void)have_range;
    if (count < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (!count || !(C->a_vertex.enabled || (C->cur && C->a_attr[0].enabled)) || !C->live) return;
    FLUSH_VERTICES();
    if (C->bound_element) {
        gl_buffer *o = gl_buffer_lookup(C->bound_element);
        if (!o || !o->data) return;
        ip = o->data + (uintptr_t)indices;
    }
    idx = (uint32_t *)malloc((size_t)count * sizeof *idx);
    list = (uint32_t *)malloc((size_t)list_capacity((uint32_t)count) * sizeof *list);
    if (!idx || !list) { free(idx); free(list); return; }
    for (i = 0; i < (uint32_t)count; i++) {
        uint32_t v = type == GL_UNSIGNED_BYTE ? ip[i] : type == GL_UNSIGNED_SHORT ? ((const uint16_t *)ip)[i] : ((const uint32_t *)ip)[i];
        idx[i] = v;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    memset(tmp, 0, sizeof tmp); memset(in, 0, sizeof in);
    arrays_to_src(in, tmp, lo, hi - lo + 1);
    if (in[A_POS].base) {
        polyinfo g;
        uint8_t *tri = NULL;
        uint32_t n;
        poly_arrays(&g, cls, &tri, (uint32_t)count);
        n = to_list(mode, idx, 0, (uint32_t)count, list, tri);
        draw_list(in, cls, list, n, &g);
        free(tri);
    }
    for (a = 0; a < A_COUNT; a++) free(tmp[a]);
    free(idx); free(list);
}

/* ---------------------------------------------------------------- immediate mode: the batch */
void gl_im_flush(void)
{
    src in[A_COUNT];
    int u, stride = C->im_vsize * 4;
    if (!C->im_n) return;
    memset(in, 0, sizeof in);
#define IM_ATTR(a, off, n) do { in[a].base = (const uint8_t *)(C->im_v + (off)); in[a].size = (n); in[a].type = G2_AT_FLOAT; in[a].bytes = (n) * 4; in[a].stride = stride; } while (0)
    IM_ATTR(A_POS, 0, 4);
    IM_ATTR(A_COLOR, 4, 4);
    if (C->im_off_normal >= 0) IM_ATTR(A_NORMAL, C->im_off_normal, 3);
    else constant(&in[A_NORMAL], C->normal, 3);
    if (C->im_off_fog >= 0) IM_ATTR(A_FOGC, C->im_off_fog, 1);
    else constant(&in[A_FOGC], &C->fog_coord, 1);
    if (C->im_off_sec >= 0) IM_ATTR(A_SEC, C->im_off_sec, 3);
    else constant(&in[A_SEC], C->sec_color, 3);
    for (u = 0; u < MAX_UNITS; u++) {
        if (C->im_off_tc[u] >= 0) IM_ATTR(A_TC0 + u, C->im_off_tc[u], 4);
        else constant(&in[A_TC0 + u], C->unit[u].tc, 4);
    }
    for (u = 1; C->cur && u < MAX_ATTRIBS; u++) {
        if (C->im_off_attr[u] >= 0) IM_ATTR(A_GEN + u - 1, C->im_off_attr[u], 4);
        else constant(&in[A_GEN + u - 1], C->attr[u], 4);
    }
#undef IM_ATTR
    if (C->live) {
        polyinfo g;
        memset(&g, 0, sizeof g);
        if (!C->im_class && !poly_filled()) { g.tri = C->im_tri; g.ef = C->im_ef; g.stride = 1; }
        draw_list(in, C->im_class, C->im_idx, C->im_ni, &g);
    }
    C->im_n = 0;
    C->im_ni = 0;
}

static void im_layout(void)
{
    int u, off = 8;
    C->im_off_normal = -1;
    if (C->cur || C->lighting || C->unit[0].gen_en[0] || C->unit[1].gen_en[0]) { C->im_off_normal = off; off += 3; }     /* what a shader reads the batch cannot tell: all of it */
    C->im_off_fog = C->cur || (C->fog && C->fog_src == GL_FOG_COORD) ? off : -1;
    if (C->im_off_fog >= 0) off += 1;
    C->im_off_sec = C->cur || (C->color_sum && !C->lighting) ? off : -1;
    if (C->im_off_sec >= 0) off += 3;
    for (u = 0; u < MAX_UNITS; u++) {
        C->im_off_tc[u] = -1;
        if (C->cur || C->unit[u].en_1d || C->unit[u].en_2d || C->unit[u].en_cube || C->unit[u].en_3d) { C->im_off_tc[u] = off; off += 4; }
    }
    for (u = 0; u < MAX_ATTRIBS; u++) {
        C->im_off_attr[u] = -1;
        if (gl_glsl_attribs() >> u & 1) { C->im_off_attr[u] = off; off += 4; }
    }
    C->im_vsize = off;
}

void gl_begin(GLenum mode)
{
    int cls = prim_class(mode);
    if (C->in_begin) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (C->im_n && C->im_class != cls) gl_im_flush();
    if (!C->im_n) { im_layout(); C->im_class = cls; }
    C->prim = mode;
    C->in_begin = 1;
    C->im_first = C->im_n;
}

void gl_end(void)
{
    uint32_t n, need;
    if (!C->in_begin) { SET_ERROR(GL_INVALID_OPERATION); return; }
    C->in_begin = 0;
    n = C->im_n - C->im_first;
    need = C->im_ni + list_capacity(n);
    if (need > C->im_icap) {
        C->im_icap = need * 2;
        C->im_idx = (uint32_t *)realloc(C->im_idx, (size_t)C->im_icap * sizeof *C->im_idx);
        C->im_tri = (uint8_t *)realloc(C->im_tri, (size_t)C->im_icap / 3 + 1);
    }
    C->im_ni += to_list(C->prim, NULL, C->im_first, n, C->im_idx + C->im_ni, C->im_tri + C->im_ni / 3);
    if (C->im_n > 40000) gl_im_flush();
}

void gl_vertex(float x, float y, float z, float w)
{
    float *v;
    int u;
    if (C->compiling) {
        gl_list_sync_attribs();
        LREC_F(LOP_VERTEX, 0, x, y, z, w);
    }
    if (!C->in_begin) return;
    if (C->im_n == C->im_cap) {
        C->im_cap = C->im_cap ? C->im_cap * 2 : 4096;
        C->im_v = (float *)realloc(C->im_v, (size_t)C->im_cap * sizeof(float) * (8 + 3 + 1 + 3 + 4 * MAX_UNITS + 4 * MAX_ATTRIBS));
        C->im_ef = (uint8_t *)realloc(C->im_ef, (size_t)C->im_cap);
    }
    C->im_ef[C->im_n] = (uint8_t)C->edge_flag;
    v = C->im_v + (size_t)C->im_n * C->im_vsize;
    v[0] = x; v[1] = y; v[2] = z; v[3] = w;
    memcpy(v + 4, C->color, 16);
    if (C->im_off_normal >= 0) memcpy(v + C->im_off_normal, C->normal, 12);
    if (C->im_off_fog >= 0) v[C->im_off_fog] = C->fog_coord;
    if (C->im_off_sec >= 0) memcpy(v + C->im_off_sec, C->sec_color, 12);
    for (u = 0; u < MAX_UNITS; u++) if (C->im_off_tc[u] >= 0) memcpy(v + C->im_off_tc[u], C->unit[u].tc, 16);
    for (u = 1; C->cur && u < MAX_ATTRIBS; u++) if (C->im_off_attr[u] >= 0) memcpy(v + C->im_off_attr[u], C->attr[u], 16);
    C->im_n++;
}
