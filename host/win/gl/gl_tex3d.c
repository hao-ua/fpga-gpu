/* 3D textures (GL 1.2 §3.8). On a board B with 3D textures a texture without border texels is a volume and the TMU filters it (sampling in gl_ff.c and gl_glsl.c), up to 256 x 256 x 255. What the TMU has no mode for, the
 * border colour, the program does around single texels of the same volume (gen_sw, sw = 4). Border texels do not halve with a level as the TMU's chain does, and other B builds have no volumes: then the slices of every
 * level lie side by side in one 2D image, read texel by texel with no filter, and the program does what the TMU would: the wrap in each of the three axes, the filter within a level and between levels (sw = 3).
 * Nothing is approximated there, at 8 reads a level for GL_LINEAR. The PC keeps every level: a sub-image, a read and GL_GENERATE_MIPMAP need no trip to the board. A level's texels are kept and stored with its
 * border, so the program reads border texels like any other. The images take the texture's storage format (gl_hw_format), and the PC's copy holds what that format keeps */
#include "gl_internal.h"

#define LV(v, l) ((v) >> (l) ? (v) >> (l) : 1)

/* the shelves: level 0 on top, the rest in a row below it */
static int atlas_layout(gl_tex *t, int w, int h, int d, int b)
{
    int l, x = 0, y = 0, rowh = 0, W = 0;
    t->w3 = w; t->h3 = h; t->d3 = d; t->b3 = b;
    t->levels = 0;
    for (l = 0; l < MAX_LEVELS; l++) {
        int lw = LV(w, l) + 2 * b, lh = LV(h, l) + 2 * b, ld = LV(d, l) + 2 * b, cols = 1, rows;
        while ((cols + 1) * lw <= 2048 && cols * cols < ld) cols++;
        rows = (ld + cols - 1) / cols;
        if (!l) {
            t->lv3[0][0] = 0; t->lv3[0][1] = 0; t->lv3[0][2] = cols;
            W = cols * lw;
            y = rows * lh;
        } else {
            if (x && x + cols * lw > W) { y += rowh; x = 0; rowh = 0; }
            t->lv3[l][0] = x; t->lv3[l][1] = y; t->lv3[l][2] = cols;
            x += cols * lw;
            if (x > W) W = x;
            if (rows * lh > rowh) rowh = rows * lh;
        }
        t->levels = l + 1;
        if (LV(w, l) == 1 && LV(h, l) == 1 && LV(d, l) == 1) break;
    }
    t->aw3 = W; t->ah3 = y + rowh;
    return W > 2048 || t->ah3 > 2048 ? -1 : 0;
}

int gl_tex3d_fits(const gl_tex *t, int l)
{
    const gl_level *v = &t->level[0][l];
    return v->w == LV(t->w3, l) && v->h == LV(t->h3, l) && v->d == LV(t->d3, l) && v->border == t->b3;
}

/* GL_CLAMP is CLAMP_TO_EDGE as long as nothing filters across it */
static int reads_border(GLenum wrap, int linear) { return wrap == GL_CLAMP_TO_BORDER || (wrap == GL_CLAMP && linear); }

int gl_tex3d_tmu(const gl_tex *t)
{
    int linear = t->mag_filter == GL_LINEAR || t->min_filter == GL_LINEAR || t->min_filter == GL_LINEAR_MIPMAP_NEAREST || t->min_filter == GL_LINEAR_MIPMAP_LINEAR;
    return t->vol3 && !reads_border(t->wrap_s, linear) && !reads_border(t->wrap_t, linear) && !reads_border(t->wrap_r, linear);
}

/* a region of a level from its PC copy to the board, slice by slice, in the level's own coordinates (0 is the border's first texel) */
static void region_up(gl_tex *t, int l, int x, int y, int z, int w, int h, int d)
{
    int b = t->level[0][l].border, lw = t->level[0][l].w + 2 * b, lh = t->level[0][l].h + 2 * b, cols = t->lv3[l][2], s;
    const uint8_t *px = t->px3[l];
    if (!px || !gl_tex3d_fits(t, l)) return;
    if (t->vol3) {
        for (s = z; s < z + d; s++) gl_img_write(t->handle, t->hwfmt, px + (((size_t)s * lh + y) * lw + x) * 4, lw, (uint32_t)x, (uint32_t)y, (uint32_t)w, (uint32_t)h, (uint32_t)l, (uint32_t)s);
        return;
    }
    for (s = z; s < z + d; s++) {
        int col = s % cols, row = s / cols;
        gl_img_write(t->handle, t->hwfmt, px + (((size_t)s * lh + y) * lw + x) * 4, lw, (uint32_t)(t->lv3[l][0] + col * lw + x), (uint32_t)(t->lv3[l][1] + row * lh + y), (uint32_t)w, (uint32_t)h, 0, 0);
    }
}

static int full_chain(int w, int h, int d)
{
    int n = 1, m = w > h ? w : h;
    if (d > m) m = d;
    while (m > 1) { m >>= 1; n++; }
    return n;
}

/* the largest a level 0 may be: the TMU's volume without border texels, else what is sure to lie side by side */
static int max_wh(int b) { return !b && g2r_has_3d(D.r) ? G2R_VOL_WH : MAX_3D_SIZE; }
static int max_d(int b) { return !b && g2r_has_3d(D.r) ? G2R_VOL_D : MAX_3D_SIZE; }

int gl_tex3d_fit(int level, int w, int h, int d, int b)
{
    return w >= 0 && h >= 0 && d >= 0 && w << level <= max_wh(b) && h << level <= max_wh(b) && d << level <= max_d(b);
}

static int image_3d(gl_tex *t, int w, int h, int d, int b, uint32_t fmt)
{
    int l, vol = !b && g2r_has_3d(D.r) && w <= G2R_VOL_WH && h <= G2R_VOL_WH && d <= G2R_VOL_D;
    if (t->handle && t->w3 == w && t->h3 == h && t->d3 == d && t->b3 == b && t->vol3 == vol && t->hwfmt == fmt) return 0;
    if (t->handle) { gl_tex_touch(t); gl_tex_view_drop(t); gl_tex_rt_drop(t); g2r_image_destroy(D.r, t->handle, t->base); t->handle = 0; }
    t->vol3 = vol; t->hwfmt = fmt;
    if (vol) {
        t->w3 = w; t->h3 = h; t->d3 = d; t->b3 = b;
        t->levels = full_chain(w, h, d);
        if (g2r_volume_create(D.r, &t->handle, &t->base, (uint32_t)w, (uint32_t)h, (uint32_t)d, fmt, (uint32_t)t->levels, G2_USE_SAMPLED | G2_USE_TRANSFER) < 0) return -1;
        t->w = w; t->h = h; t->layers = 1;
    } else {
        if (atlas_layout(t, w, h, d, b) < 0) return -1;
        if (g2r_image_create_tight(D.r, &t->handle, &t->base, (uint32_t)t->aw3, (uint32_t)t->ah3, fmt, 1, 1, 1, G2_USE_SAMPLED | G2_USE_TRANSFER) < 0) return -1;
        t->w = t->aw3; t->h = t->ah3; t->layers = 1;
    }
    for (l = 0; l < t->levels; l++) {                           /* the levels that were already there, as the format keeps them */
        gl_level *v = &t->level[0][l];
        if (!v->defined) continue;
        if (t->px3[l]) gl_quantize(fmt, t->px3[l], (size_t)(v->w + 2 * v->border) * (v->h + 2 * v->border) * (v->d + 2 * v->border));
        region_up(t, l, 0, 0, 0, v->w + 2 * v->border, v->h + 2 * v->border, v->d + 2 * v->border);
    }
    return 0;
}

/* an image starts on a 32-byte boundary, which the small levels of a chain do not: those go into a volume of their own */
uint16_t gl_tex3d_view(gl_tex *t, int b, int n)
{
    uint32_t off = g2_vol_chain_bytes((uint32_t)t->w3, (uint32_t)t->h3, (uint32_t)t->d3, (uint32_t)gl_fmt_bpp(t->hwfmt), (uint32_t)b);
    int w = LV(t->w3, b), h = LV(t->h3, b), d = LV(t->d3, b), l, s;
    uint16_t hd;
    if (!(off & 31)) {
        if (g2r_volume_alias(D.r, &hd, t->base + off, (uint32_t)w, (uint32_t)h, (uint32_t)d, t->hwfmt, (uint32_t)n, G2_USE_SAMPLED) < 0) return t->handle;
        t->view = hd; t->view_base = t->base + off; t->view_copy = 0;
        return hd;
    }
    if (g2r_volume_create(D.r, &t->view, &t->view_base, (uint32_t)w, (uint32_t)h, (uint32_t)d, t->hwfmt, (uint32_t)n, G2_USE_SAMPLED | G2_USE_TRANSFER) < 0) { t->view = 0; return t->handle; }
    for (l = 0; l < n; l++) {
        int lw = LV(w, l), lh = LV(h, l), ld = LV(d, l);
        for (s = 0; s < ld && t->px3[b + l]; s++) gl_img_write(t->view, t->hwfmt, t->px3[b + l] + (size_t)s * lw * lh * 4, lw, 0, 0, (uint32_t)lw, (uint32_t)lh, (uint32_t)l, (uint32_t)s);
    }
    t->view_copy = 1;
    return t->view;
}

/* a framebuffer renders a slice in an image of its own (gl_tex.c, the texture's rt). The PC's copy is the texture: it fills the image, and what was rendered comes back through it */
void gl_tex3d_rt_load(gl_tex *t)
{
    int l = t->rt_level, lw = t->level[0][l].w, lh = t->level[0][l].h;
    if (t->px3[l]) gl_img_write(t->rt, t->hwfmt, t->px3[l] + (size_t)t->rt_face * lw * lh * 4, lw, 0, 0, (uint32_t)lw, (uint32_t)lh, 0, 0);
}

void gl_tex3d_rt_store(gl_tex *t)
{
    int l = t->rt_level, lw = t->level[0][l].w, lh = t->level[0][l].h;
    if (!t->px3[l]) return;
    g2r_finish(D.r);
    gl_img_read(t->rt, t->hwfmt, 0, 0, (uint32_t)lw, (uint32_t)lh, 0, 0, t->px3[l] + (size_t)t->rt_face * lw * lh * 4);
    region_up(t, l, 0, 0, t->rt_face, lw, lh, 1);
}

/* the mean of the 8 texels under each one, the border as the edge of the level below it goes */
static void box_3d(gl_tex *t, int l)
{
    const gl_level *f = &t->level[0][l - 1];
    int b = t->b3, sw = f->w, sh = f->h, sd = f->d, w = LV(sw, 1), h = LV(sh, 1), d = LV(sd, 1), W = w + 2 * b, H = h + 2 * b, D3 = d + 2 * b, SW = sw + 2 * b, SH = sh + 2 * b, x, y, z, c;
    const uint8_t *s = t->px3[l - 1];
    uint8_t *o;
    free(t->px3[l]);
    o = t->px3[l] = (uint8_t *)malloc((size_t)W * H * D3 * 4);
    if (!o) return;
    t->level[0][l].defined = 1; t->level[0][l].w = w; t->level[0][l].h = h; t->level[0][l].d = d; t->level[0][l].border = b;
    for (z = 0; z < D3; z++) {
        int zi = z - b < 0 ? 0 : z - b >= d ? d - 1 : z - b;
        for (y = 0; y < H; y++) {
            int yi = y - b < 0 ? 0 : y - b >= h ? h - 1 : y - b;
            for (x = 0; x < W; x++) {
                int xi = x - b < 0 ? 0 : x - b >= w ? w - 1 : x - b;
                for (c = 0; c < 4; c++) {
                    int sum = 0, i, j, k;
                    for (k = 0; k < 2; k++) for (j = 0; j < 2; j++) for (i = 0; i < 2; i++) {
                        int ix = (sw > 1 ? 2 * xi + i : 0) + b, iy = (sh > 1 ? 2 * yi + j : 0) + b, iz = (sd > 1 ? 2 * zi + k : 0) + b;
                        sum += s[(((size_t)iz * SH + iy) * SW + ix) * 4 + c];
                    }
                    o[(((size_t)z * H + y) * W + x) * 4 + c] = (uint8_t)((sum + 4) >> 3);
                }
            }
        }
    }
    gl_quantize(t->hwfmt, o, (size_t)W * H * D3);
    region_up(t, l, 0, 0, 0, W, H, D3);
}

void gl_tex3d_generate(gl_tex *t)
{
    int l;
    if (!t->px3[0] || !gl_tex3d_fits(t, 0)) return;
    for (l = 1; l < t->levels; l++) box_3d(t, l);
}

void gl_tex3d_free(gl_tex *t)
{
    int l;
    for (l = 0; l < MAX_LEVELS; l++) { free(t->px3[l]); t->px3[l] = NULL; }
}

static const void *unpack3(GLsizei w, GLsizei h, GLsizei d, GLenum format, GLenum type, const void *pixels)
{
    (void)w; (void)h; (void)d; (void)format; (void)type;
    return gl_unpack_ptr(pixels);
}

static int check3(GLenum target, GLint level)
{
    if (target != GL_TEXTURE_3D) { SET_ERROR(GL_INVALID_ENUM); return 0; }
    if (level < 0 || level >= MAX_LEVELS) { SET_ERROR(GL_INVALID_VALUE); return 0; }
    return C->live;
}

/* the rows of each slice, as GL_UNPACK_IMAGE_HEIGHT and GL_UNPACK_SKIP_IMAGES say */
static void unpack_slices(uint8_t *o, int w, int h, int d, GLenum format, GLenum type, const void *pixels, GLenum internal)
{
    int z, sr = C->un_skip_row, ih = C->un_image_h ? C->un_image_h : h;
    for (z = 0; z < d; z++) {
        C->un_skip_row = sr + (C->un_skip_images + z) * ih;
        gl_unpack(o + (size_t)z * w * h * 4, w, h, format, type, pixels, internal);
    }
    C->un_skip_row = sr;
}

void APIENTRY glTexImage3D(GLenum target, GLint level, GLint internal, GLsizei wb, GLsizei hb, GLsizei db, GLint border, GLenum format, GLenum type, const void *pixels)
{
    gl_tex *t;
    int w = wb - 2 * border, h = hb - 2 * border, d = db - 2 * border;
    uint32_t fmt;
    if (target == GL_PROXY_TEXTURE_3D) { gl_tex_proxy(target, level, w, h, d, (GLenum)internal); return; }
    FLUSH_VERTICES();
    if (!check3(target, level)) return;
    if ((border != 0 && border != 1) || !gl_tex3d_fit(level, w, h, d, border)) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (format == GL_DEPTH_COMPONENT || gl_base_class((GLenum)internal) == BF_DEPTH) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (!(t = gl_bound_tex(target))) return;
    if (pixels || C->bound_unpack) pixels = unpack3(wb, hb, db, format, type, pixels);
    gl_tex_resolve(t);
    free(t->px3[level]);
    t->px3[level] = NULL;
    t->level[0][level].defined = 1; t->level[0][level].w = w; t->level[0][level].h = h; t->level[0][level].d = d; t->level[0][level].border = border;
    if (level == 0) { t->internal = (GLenum)internal; t->base_class = gl_base_class((GLenum)internal); }
    gl_tex_touch(t);
    fmt = level == 0 || !t->handle ? gl_hw_format((GLenum)internal, type) : t->hwfmt;
    if (wb && hb && db) {
        t->px3[level] = (uint8_t *)calloc((size_t)wb * hb * db, 4);
        if (!t->px3[level]) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
        if (pixels) unpack_slices(t->px3[level], wb, hb, db, format, type, pixels, (GLenum)internal);
        gl_quantize(fmt, t->px3[level], (size_t)wb * hb * db);
    }
    if ((level == 0 || !t->handle) && image_3d(t, w ? w << level : 1, h ? h << level : 1, d ? d << level : 1, border, fmt) < 0) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    region_up(t, level, 0, 0, 0, wb, hb, db);
    if (t->gen_mipmap && level == 0) gl_tex3d_generate(t);
}

void APIENTRY glTexSubImage3D(GLenum target, GLint level, GLint x, GLint y, GLint z, GLsizei w, GLsizei h, GLsizei d, GLenum format, GLenum type, const void *pixels)
{
    gl_tex *t;
    gl_level *lv;
    uint8_t *rgba;
    int r, s, b, W, H;
    FLUSH_VERTICES();
    if (!check3(target, level) || !(t = gl_bound_tex(target))) return;
    lv = &t->level[0][level];
    if (!lv->defined || !t->px3[level]) { SET_ERROR(GL_INVALID_OPERATION); return; }
    b = lv->border; W = lv->w + 2 * b; H = lv->h + 2 * b;
    x += b; y += b; z += b;
    if (x < 0 || y < 0 || z < 0 || w < 0 || h < 0 || d < 0 || x + w > W || y + h > H || z + d > lv->d + 2 * b) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (!w || !h || !d || (!pixels && !C->bound_unpack)) return;
    pixels = unpack3(w, h, d, format, type, pixels);
    gl_tex_resolve(t);
    rgba = (uint8_t *)malloc((size_t)w * h * d * 4);
    if (!rgba) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    unpack_slices(rgba, w, h, d, format, type, pixels, t->internal);
    gl_quantize(t->hwfmt, rgba, (size_t)w * h * d);
    gl_tex_touch(t);
    for (s = 0; s < d; s++) for (r = 0; r < h; r++) memcpy(t->px3[level] + (((size_t)(z + s) * H + y + r) * W + x) * 4, rgba + ((size_t)s * h + r) * w * 4, (size_t)w * 4);
    free(rgba);
    region_up(t, level, x, y, z, w, h, d);
    if (t->gen_mipmap && level == 0) gl_tex3d_generate(t);
}

void APIENTRY glCopyTexSubImage3D(GLenum target, GLint level, GLint xo, GLint yo, GLint zo, GLint x, GLint y, GLsizei w, GLsizei h)
{
    gl_tex *t;
    gl_level *lv;
    uint8_t *px;
    int al = C->pk_align, pr = C->pk_row, ps = C->pk_skip_px, pz = C->pk_skip_row, r, b, W, H;
    FLUSH_VERTICES();
    if (!check3(target, level) || !(t = gl_bound_tex(target))) return;
    lv = &t->level[0][level];
    if (!lv->defined || !t->px3[level]) { SET_ERROR(GL_INVALID_OPERATION); return; }
    b = lv->border; W = lv->w + 2 * b; H = lv->h + 2 * b;
    xo += b; yo += b; zo += b;
    if (xo < 0 || yo < 0 || zo < 0 || w < 0 || h < 0 || xo + w > W || yo + h > H || zo >= lv->d + 2 * b) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (!w || !h) return;
    px = (uint8_t *)malloc((size_t)w * h * 4);
    if (!px) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    C->pk_align = 4; C->pk_row = C->pk_skip_px = C->pk_skip_row = 0;
    glReadPixels(x, y, w, h, GL_RGBA, GL_UNSIGNED_BYTE, px);
    C->pk_align = al; C->pk_row = pr; C->pk_skip_px = ps; C->pk_skip_row = pz;
    {                                                           /* table 3.15: what the internal format keeps */
        uint8_t *o = (uint8_t *)malloc((size_t)w * h * 4);
        int ua = C->un_align, ur = C->un_row, us = C->un_skip_px, uz = C->un_skip_row, tr = C->px_active;
        if (o) {
            C->un_align = 4; C->un_row = C->un_skip_px = C->un_skip_row = 0; C->px_active = 0;
            gl_unpack(o, w, h, GL_RGBA, GL_UNSIGNED_BYTE, px, t->internal);
            C->un_align = ua; C->un_row = ur; C->un_skip_px = us; C->un_skip_row = uz; C->px_active = tr;
            free(px);
            px = o;
        }
    }
    gl_quantize(t->hwfmt, px, (size_t)w * h);
    gl_tex_touch(t);
    for (r = 0; r < h; r++) memcpy(t->px3[level] + (((size_t)zo * H + yo + r) * W + xo) * 4, px + (size_t)r * w * 4, (size_t)w * 4);
    free(px);
    region_up(t, level, xo, yo, zo, w, h, 1);
    if (t->gen_mipmap && level == 0) gl_tex3d_generate(t);
}
