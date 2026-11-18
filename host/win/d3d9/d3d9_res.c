/* d3d9_res.c — the resource objects: surfaces, textures, buffers, declarations, shader objects, queries, state blocks. */
#include "d3d9_internal.h"

/* ---------------------------------------------------------------- formats */
int d3_fmt_dtex(D3DFORMAT f);

int d3_fmt_bytes(D3DFORMAT f)
{
    switch (f) {
    case D3DFMT_A8R8G8B8: case D3DFMT_X8R8G8B8: case D3DFMT_A8B8G8R8: case D3DFMT_X8B8G8R8: case D3DFMT_D24S8: case D3DFMT_D24X8: case D3DFMT_D32: case D3DFMT_INDEX32: case D3DFMT_Q8W8V8U8: case D3DFMT_X8L8V8U8: return 4;
    case D3DFMT_R5G6B5: case D3DFMT_A1R5G5B5: case D3DFMT_X1R5G5B5: case D3DFMT_A4R4G4B4: case D3DFMT_X4R4G4B4: case D3DFMT_A8L8: case D3DFMT_D16: case D3DFMT_D16_LOCKABLE: case D3DFMT_INDEX16: case D3DFMT_V8U8: case D3DFMT_L6V5U5: return 2;
    case D3DFMT_L8: case D3DFMT_A8: case D3DFMT_A4L4: case D3DFMT_R3G3B2: return 1;
    case D3DFMT_A8R3G3B2: return 2;
    default: return d3_fmt_dtex(f) ? 4 : 0;
    }
}

int d3_fmt_signed(D3DFORMAT f)
{
    switch (f) {
    case D3DFMT_V8U8: case D3DFMT_L6V5U5: case D3DFMT_X8L8V8U8: return 1;
    case D3DFMT_Q8W8V8U8: return 2;
    default: return 0;
    }
}

int d3_fmt_srgb(D3DFORMAT f) { return f == D3DFMT_A8R8G8B8 || f == D3DFMT_X8R8G8B8 || f == D3DFMT_A8B8G8R8 || f == D3DFMT_X8B8G8R8 || f == D3DFMT_DXT1 || f == D3DFMT_DXT2 || f == D3DFMT_DXT3 || f == D3DFMT_DXT4 || f == D3DFMT_DXT5 || f == D3DFMT_L8 || f == D3DFMT_A8L8; }

static int dxt_of(D3DFORMAT f) { return f == D3DFMT_DXT1 ? G2_DXT1A : f == D3DFMT_DXT2 || f == D3DFMT_DXT3 ? G2_DXT3 : f == D3DFMT_DXT4 || f == D3DFMT_DXT5 ? G2_DXT5 : 0; }

int d3_fmt_g2(D3DFORMAT f, uint32_t *g2fmt)
{
    switch (f) {
    case D3DFMT_A8R8G8B8: case D3DFMT_X8R8G8B8: case D3DFMT_A8B8G8R8: case D3DFMT_X8B8G8R8: case D3DFMT_L8: case D3DFMT_A8: case D3DFMT_A8L8: case D3DFMT_V8U8: case D3DFMT_L6V5U5: case D3DFMT_X8L8V8U8: case D3DFMT_Q8W8V8U8:
    case D3DFMT_DXT1: case D3DFMT_DXT2: case D3DFMT_DXT3: case D3DFMT_DXT4: case D3DFMT_DXT5: case D3DFMT_A8R3G3B2: *g2fmt = G2_FMT_RGBA8; return 1;
    case D3DFMT_R5G6B5: case D3DFMT_R3G3B2: *g2fmt = G2_FMT_RGB565; return 1;           /* R3G3B2: the closest of B's formats that has its bits and no alpha */
    case D3DFMT_A1R5G5B5: case D3DFMT_X1R5G5B5: *g2fmt = G2_FMT_RGBA5551; return 1;
    case D3DFMT_A4R4G4B4: case D3DFMT_X4R4G4B4: case D3DFMT_A4L4: *g2fmt = G2_FMT_RGBA4444; return 1;
    default: return 0;
    }
}

int d3_fmt_rt(D3DFORMAT f) { return f == D3DFMT_A8R8G8B8 || f == D3DFMT_X8R8G8B8 || f == D3DFMT_R5G6B5 || f == D3DFMT_A1R5G5B5 || f == D3DFMT_X1R5G5B5 || f == D3DFMT_A4R4G4B4; }
#define FOURCC_INTZ ((D3DFORMAT)MAKEFOURCC('I', 'N', 'T', 'Z'))
#define FOURCC_DF24 ((D3DFORMAT)MAKEFOURCC('D', 'F', '2', '4'))
#define FOURCC_DF16 ((D3DFORMAT)MAKEFOURCC('D', 'F', '1', '6'))
/* a depth surface that can be sampled: the word of a D24S8 target in the bytes of RGBA8 */
int d3_fmt_dtex(D3DFORMAT f) { return f == FOURCC_INTZ || f == FOURCC_DF24 || f == FOURCC_DF16 || f == D3DFMT_D24S8 || f == D3DFMT_D24X8 || f == D3DFMT_D16; }
int d3_fmt_ds(D3DFORMAT f) { return f == D3DFMT_D24S8 || f == D3DFMT_D24X8 || f == D3DFMT_D16 || f == D3DFMT_D32 || f == D3DFMT_D16_LOCKABLE; }

static int shadow_layout(UINT w, UINT h, D3DFORMAT f, int *pitch)
{
    int b = d3_fmt_bytes(f), d = dxt_of(f);
    if (d) {
        int block = f == D3DFMT_DXT1 ? 8 : 16;
        *pitch = (int)((w + 3) / 4) * block;
        return *pitch * (int)((h + 3) / 4);
    }
    *pitch = ((int)w * b + 3) & ~3;
    return *pitch * (int)h;
}

/* the shadow's rows as the board stores them: RAW for 16-bit storage, RGBA bytes for RGBA8 */
static uint8_t *to_storage(const d3surf *s, uint32_t *tf, int *bpp)
{
    int w = (int)s->w, h = (int)s->h, r, c, d = dxt_of(s->fmt);
    uint8_t *out;
    if (d) {
        out = (uint8_t *)malloc((size_t)w * h * 4);
        if (out) g2_dxt_decode(d, s->mem, w, h, out);
        *tf = G2_TF_RGBA8888; *bpp = 4;
        return out;
    }
    *bpp = (s->tex ? s->tex->g2fmt : s->g2fmt) == G2_FMT_RGBA8 ? 4 : 2;
    *tf = *bpp == 4 ? G2_TF_RGBA8888 : G2_TF_RAW;
    out = (uint8_t *)malloc((size_t)w * h * *bpp);
    if (!out) return NULL;
    for (r = 0; r < h; r++) {
        const uint8_t *p = s->mem + (ptrdiff_t)r * s->pitch;
        uint8_t *q = out + (size_t)r * w * *bpp;
        const uint16_t *p16 = (const uint16_t *)p;
        uint16_t *q16 = (uint16_t *)q;
        switch (s->fmt) {
        case D3DFMT_A8R8G8B8: for (c = 0; c < w; c++) { q[4 * c] = p[4 * c + 2]; q[4 * c + 1] = p[4 * c + 1]; q[4 * c + 2] = p[4 * c]; q[4 * c + 3] = p[4 * c + 3]; } break;
        case D3DFMT_X8R8G8B8: for (c = 0; c < w; c++) { q[4 * c] = p[4 * c + 2]; q[4 * c + 1] = p[4 * c + 1]; q[4 * c + 2] = p[4 * c]; q[4 * c + 3] = 255; } break;
        case D3DFMT_A8B8G8R8: memcpy(q, p, (size_t)w * 4); break;
        case D3DFMT_X8B8G8R8: for (c = 0; c < w; c++) { memcpy(q + 4 * c, p + 4 * c, 3); q[4 * c + 3] = 255; } break;
        case D3DFMT_L8: for (c = 0; c < w; c++) { q[4 * c] = q[4 * c + 1] = q[4 * c + 2] = p[c]; q[4 * c + 3] = 255; } break;
        case D3DFMT_A8: for (c = 0; c < w; c++) { q[4 * c] = q[4 * c + 1] = q[4 * c + 2] = 0; q[4 * c + 3] = p[c]; } break;
        case D3DFMT_A8L8: for (c = 0; c < w; c++) { q[4 * c] = q[4 * c + 1] = q[4 * c + 2] = p[2 * c]; q[4 * c + 3] = p[2 * c + 1]; } break;
        /* bump formats: signed channels stored + 128, G2S<n> in d3d9_ff.c takes it off; what a format does not have reads 1 */
        case D3DFMT_V8U8: for (c = 0; c < w; c++) { q[4 * c] = (uint8_t)(p[2 * c] ^ 0x80); q[4 * c + 1] = (uint8_t)(p[2 * c + 1] ^ 0x80); q[4 * c + 2] = q[4 * c + 3] = 255; } break;
        case D3DFMT_L6V5U5: for (c = 0; c < w; c++) {
            int u = p16[c] & 31, v = p16[c] >> 5 & 31, l = p16[c] >> 10 & 63;
            u = u > 15 ? u - 32 : u; v = v > 15 ? v - 32 : v;
            q[4 * c] = (uint8_t)(128 + u * 127 / 15); q[4 * c + 1] = (uint8_t)(128 + v * 127 / 15); q[4 * c + 2] = (uint8_t)(l * 255 / 63); q[4 * c + 3] = 255;
        } break;
        case D3DFMT_X8L8V8U8: for (c = 0; c < w; c++) { q[4 * c] = (uint8_t)(p[4 * c] ^ 0x80); q[4 * c + 1] = (uint8_t)(p[4 * c + 1] ^ 0x80); q[4 * c + 2] = p[4 * c + 2]; q[4 * c + 3] = 255; } break;
        case D3DFMT_Q8W8V8U8: for (c = 0; c < w; c++) { q[4 * c] = (uint8_t)(p[4 * c] ^ 0x80); q[4 * c + 1] = (uint8_t)(p[4 * c + 1] ^ 0x80); q[4 * c + 2] = (uint8_t)(p[4 * c + 2] ^ 0x80); q[4 * c + 3] = (uint8_t)(p[4 * c + 3] ^ 0x80); } break;
        case D3DFMT_R5G6B5: memcpy(q, p, (size_t)w * 2); break;
        case D3DFMT_A4L4: for (c = 0; c < w; c++) { uint32_t l = p[c] & 15u; q16[c] = (uint16_t)(l << 12 | l << 8 | l << 4 | (uint32_t)p[c] >> 4); } break;
        case D3DFMT_R3G3B2: for (c = 0; c < w; c++) q16[c] = (uint16_t)(((p[c] >> 5) * 62 + 7) / 14 << 11 | ((p[c] >> 2 & 7) * 126 + 7) / 14 << 5 | ((p[c] & 3) * 62 + 3) / 6); break;
        case D3DFMT_A8R3G3B2: for (c = 0; c < w; c++) { uint32_t v = p16[c]; q[4 * c] = (uint8_t)((v >> 5 & 7) * 255 / 7); q[4 * c + 1] = (uint8_t)((v >> 2 & 7) * 255 / 7); q[4 * c + 2] = (uint8_t)((v & 3) * 85); q[4 * c + 3] = (uint8_t)(v >> 8); } break;
        case D3DFMT_A1R5G5B5: case D3DFMT_X1R5G5B5:
            if (*bpp == 4) {                                    /* a back buffer of A1R5G5B5 is RGBA8 */
                for (c = 0; c < w; c++) { uint32_t v = p16[c]; q[4 * c] = (uint8_t)(((v >> 10 & 31) * 510 + 31) / 62); q[4 * c + 1] = (uint8_t)(((v >> 5 & 31) * 510 + 31) / 62); q[4 * c + 2] = (uint8_t)(((v & 31) * 510 + 31) / 62); q[4 * c + 3] = s->fmt == D3DFMT_X1R5G5B5 || v >> 15 ? 255 : 0; }
            } else if ((s->tex ? s->tex->g2fmt : s->g2fmt) == G2_FMT_RGB565) {      /* a back buffer of X1R5G5B5 is 5:6:5: green gains its top bit at the bottom */
                for (c = 0; c < w; c++) { uint32_t v = p16[c], g = v >> 5 & 31; q16[c] = (uint16_t)((v >> 10 & 31) << 11 | (g << 1 | g >> 4) << 5 | (v & 31)); }
            } else if (s->fmt == D3DFMT_A1R5G5B5) for (c = 0; c < w; c++) q16[c] = (uint16_t)(p16[c] << 1 | p16[c] >> 15);
            else for (c = 0; c < w; c++) q16[c] = (uint16_t)(p16[c] << 1 | 1);
            break;
        case D3DFMT_A4R4G4B4: for (c = 0; c < w; c++) q16[c] = (uint16_t)(p16[c] << 4 | p16[c] >> 12); break;
        case D3DFMT_X4R4G4B4: for (c = 0; c < w; c++) q16[c] = (uint16_t)(p16[c] << 4 | 0xF); break;
        default: memset(q, 0, (size_t)w * *bpp); break;
        }
    }
    return out;
}

/* board pixels (RGBA bytes or RAW 16-bit) into the shadow's format */
static void from_storage(d3surf *s, const uint8_t *in)
{
    int w = (int)s->w, h = (int)s->h, r, c;
    for (r = 0; r < h; r++) {
        uint8_t *q = s->mem + (ptrdiff_t)r * s->pitch;
        const uint8_t *p = in + (size_t)r * w * (s->g2fmt == G2_FMT_RGBA8 ? 4 : 2);
        const uint16_t *p16 = (const uint16_t *)p;
        uint16_t *q16 = (uint16_t *)q;
        switch (s->fmt) {
        case D3DFMT_A8R8G8B8: case D3DFMT_X8R8G8B8: for (c = 0; c < w; c++) { q[4 * c] = p[4 * c + 2]; q[4 * c + 1] = p[4 * c + 1]; q[4 * c + 2] = p[4 * c]; q[4 * c + 3] = s->fmt == D3DFMT_X8R8G8B8 ? 255 : p[4 * c + 3]; } break;
        case D3DFMT_R5G6B5: memcpy(q, p, (size_t)w * 2); break;
        case D3DFMT_A1R5G5B5: case D3DFMT_X1R5G5B5:
            if (s->g2fmt == G2_FMT_RGBA8) for (c = 0; c < w; c++) q16[c] = (uint16_t)(((p[4 * c] * 62u + 255) / 510) << 10 | ((p[4 * c + 1] * 62u + 255) / 510) << 5 | ((p[4 * c + 2] * 62u + 255) / 510) | (p[4 * c + 3] >= 128 ? 0x8000u : 0));
            else if (s->g2fmt == G2_FMT_RGB565) for (c = 0; c < w; c++) { uint32_t v = p16[c]; q16[c] = (uint16_t)(0x8000u | (v >> 11) << 10 | ((v >> 5 & 63) * 31 + 31) / 63 << 5 | (v & 31)); }
            else for (c = 0; c < w; c++) q16[c] = (uint16_t)(p16[c] >> 1 | p16[c] << 15);
            break;
        case D3DFMT_A4R4G4B4: for (c = 0; c < w; c++) q16[c] = (uint16_t)(p16[c] >> 4 | p16[c] << 12); break;
        default: break;
        }
    }
}

/* ---------------------------------------------------------------- texture upload, render images */
static void tex_storage(d3tex *t)
{
    d3dev *d = t->dev;
    if (t->handle) return;
    if (t->vol) {
        if (g2r_volume_create(d->r, &t->handle, &t->base, t->w, t->h, t->d, t->g2fmt, (uint32_t)t->levels, G2_USE_SAMPLED | G2_USE_TRANSFER) < 0) g2e_log("d3d9: no room for a %u x %u x %u texture", t->w, t->h, t->d);
        return;
    }
    if (g2r_image_create_tight(d->r, &t->handle, &t->base, t->w, t->h, t->g2fmt, (uint32_t)t->hwl, (uint32_t)t->layers, 0, G2_USE_SAMPLED | G2_USE_TRANSFER | G2_USE_RT) < 0) g2e_log("d3d9: no room for a %u x %u texture", t->w, t->h);
}

/* the slices of a level, each through to_storage as a surface of its own */
static void vol_prepare(d3tex *t)
{
    d3dev *d = t->dev;
    int l;
    for (l = 0; l < t->levels; l++) {
        d3vol *v = t->lv[l];
        UINT z;
        if (!v->dirty) continue;
        if (t->used_pass == g2r_pass_id(d->r)) g2r_end_pass(d->r);
        for (z = 0; z < v->d; z++) {
            d3surf s;
            uint8_t *px;
            uint32_t tf;
            int bpp;
            memset(&s, 0, sizeof s);
            s.w = v->w; s.h = v->h; s.fmt = t->fmt; s.g2fmt = t->g2fmt; s.pitch = v->pitch; s.mem = v->mem + (size_t)z * v->slice;
            px = to_storage(&s, &tf, &bpp);
            if (px) {
                g2_write_image(d->bd, t->handle, px, (size_t)v->w * v->h * bpp, tf, 0, 0, v->w, v->h, (uint32_t)l, z, 0, t->g2fmt);
                free(px);
            }
        }
        v->dirty = 0;
        t->rev++;
    }
}

#define TEX_USAGE (G2_USE_SAMPLED | G2_USE_TRANSFER | G2_USE_RT)

/* a texture whose image can be replaced by a copy under a new handle: nothing else holds the handle or the memory (a view, a render image, a
   depth alias, the open pass's target), and its chain is not regenerated from the board (D3DUSAGE_AUTOGENMIPMAP reads it back, which ends the
   pass anyway) */
static int can_rename(const d3tex *t)
{
    return t->handle && !t->vol && !t->view && !t->rt && !t->rt_ds && !t->ds_alias && !t->depth && !t->autogen && !g2r_is_target(t->dev->r, t->handle);
}

/* Board B writes an image at once but renders a pass's draws at its FLUSH: a texture the open pass sampled must keep its old texels for those
   draws. It moves to a copy of itself (copy-on-write; the old image is freed after the pass), so an application that changes a texture in the
   middle of a frame costs a copy on board B instead of a FLUSH and a reload of the whole target. A level whose every face is uploaded whole needs
   no copy. Where a copy is not possible the pass ends first. */
static void tex_before_write(d3tex *t)
{
    d3dev *d = t->dev;
    static int cow = -1;                                        /* GPU2_TEX_COW=0: end the pass instead, for a comparison */
    uint32_t mask = 0;
    int l, f;
    if (t->used_pass != g2r_pass_id(d->r)) return;
    if (cow < 0) cow = g2e_get_int("GPU2_TEX_COW", 1);
    for (l = 0; l < t->levels && l < 32; l++) {
        for (f = 0; f < t->layers; f++) {
            const d3surf *s = t->surf[f * t->levels + l];
            if (!s->dirty || !s->mem) { mask |= 1u << l; break; }
        }
    }
    if (!cow || !can_rename(t) || g2r_image_rename(d->r, &t->handle, &t->base, t->w, t->h, t->g2fmt, (uint32_t)t->hwl, (uint32_t)t->layers, 0, TEX_USAGE, 1, mask) < 0)
        g2r_end_pass(d->r);
    t->used_pass = 0xFFFFFFFFu;
}

void d3_tex_prepare(d3tex *t)
{
    d3dev *d = t->dev;
    int l, f, any = 0;
    tex_storage(t);
    if (t->vol) { vol_prepare(t); return; }
    for (f = 0; f < t->layers && !any; f++) for (l = 0; l < t->levels; l++) if (t->surf[f * t->levels + l]->dirty && t->surf[f * t->levels + l]->mem) { any = 1; break; }
    if (!any) return;
    d3_tex_resolve(t);
    tex_before_write(t);
    for (f = 0; f < t->layers; f++) {
        for (l = 0; l < t->levels; l++) {
            d3surf *s = t->surf[f * t->levels + l];
            uint8_t *px;
            uint32_t tf;
            int bpp;
            if (!s->dirty || !s->mem) continue;
            px = to_storage(s, &tf, &bpp);
            if (px) {
                g2_write_image(d->bd, t->handle, px, (size_t)s->w * s->h * bpp, tf, 0, 0, s->w, s->h, (uint32_t)l, (uint32_t)f, 0, t->g2fmt);
                free(px);
            }
            s->dirty = 0;
            t->rev++;
            if (t->rt) t->rt_stale = 1;
            if (t->autogen) t->gen_dirty = 1;
        }
    }
}

static void tex_copy_rt(d3tex *t, int to_tex)
{
    d3dev *d = t->dev;
    uint32_t f = (uint32_t)t->rt_face, l = (uint32_t)t->rt_level, w = (uint32_t)t->rt_w, h = (uint32_t)t->rt_h;
    g2r_end_pass(d->r);
    if (to_tex) g2_copy_image(d->bd, t->rt, t->handle, 0, 0, w, h, 0, 0, w, h, 0, 0, 0, 0, 0, 0, 0, l, f);
    else g2_copy_image(d->bd, t->handle, t->rt, 0, 0, w, h, 0, 0, w, h, 0, 0, 0, 0, 0, l, f, 0, 0);
}

void d3_tex_resolve(d3tex *t)
{
    if (t->rt_dirty) { tex_copy_rt(t, 1); t->rev++; }
    t->rt_dirty = 0;
}

/* D3DUSAGE_AUTOGENMIPMAP: B has no GEN_MIPMAP, so the top level comes back and the chain goes up */
static void box16(uint8_t *nx, const uint8_t *cur, int w, int h, int nw, int nh, uint32_t fmt, int point)
{
    static const uint8_t SH[3][4] = { { 11, 5, 0, 0 }, { 12, 8, 4, 0 }, { 11, 6, 1, 0 } }, BI[3][4] = { { 5, 6, 5, 0 }, { 4, 4, 4, 4 }, { 5, 5, 5, 1 } };
    int f = fmt == G2_FMT_RGB565 ? 0 : fmt == G2_FMT_RGBA4444 ? 1 : 2, x, y, c;
    const uint16_t *s = (const uint16_t *)cur;
    for (y = 0; y < nh; y++) {
        int y0 = h > 1 ? 2 * y : 0, y1 = h > 1 ? 2 * y + 1 : 0;
        for (x = 0; x < nw; x++) {
            int x0 = w > 1 ? 2 * x : 0, x1 = w > 1 ? 2 * x + 1 : 0;
            uint32_t v = 0;
            for (c = 0; c < 4; c++) {
                uint32_t m = (1u << BI[f][c]) - 1, a = s[y0 * w + x0] >> SH[f][c] & m;
                if (!point) a = (a + (s[y0 * w + x1] >> SH[f][c] & m) + (s[y1 * w + x0] >> SH[f][c] & m) + (s[y1 * w + x1] >> SH[f][c] & m) + 2) >> 2;
                v |= a << SH[f][c];
            }
            ((uint16_t *)nx)[y * nw + x] = (uint16_t)v;
        }
    }
}

static void box32(uint8_t *nx, const uint8_t *cur, int w, int h, int nw, int nh, int point)
{
    int x, y, c;
    for (y = 0; y < nh; y++) {
        int y0 = h > 1 ? 2 * y : 0, y1 = h > 1 ? 2 * y + 1 : 0;
        for (x = 0; x < nw; x++) {
            int x0 = w > 1 ? 2 * x : 0, x1 = w > 1 ? 2 * x + 1 : 0;
            for (c = 0; c < 4; c++) {
                int a = cur[(y0 * w + x0) * 4 + c];
                nx[(y * nw + x) * 4 + c] = (uint8_t)(point ? a : (a + cur[(y0 * w + x1) * 4 + c] + cur[(y1 * w + x0) * 4 + c] + cur[(y1 * w + x1) * 4 + c] + 2) >> 2);
            }
        }
    }
}

static void gen_levels(d3tex *t)
{
    d3dev *d = t->dev;
    int f, l, bpp = t->g2fmt == G2_FMT_RGBA8 ? 4 : 2, point = t->autogen_filter == D3DTEXF_POINT;
    uint32_t tf = bpp == 4 ? G2_TF_RGBA8888 : G2_TF_RAW;
    g2r_finish(d->r);
    for (f = 0; f < t->layers; f++) {
        int w = (int)t->w, h = (int)t->h;
        uint8_t *cur = (uint8_t *)malloc((size_t)w * h * bpp);
        if (!cur) return;
        g2_read_image(d->bd, t->handle, tf, 0, 0, (uint32_t)w, (uint32_t)h, 0, (uint32_t)f, t->g2fmt, 0, 0, cur);
        for (l = 1; l < t->hwl; l++) {
            int nw = w > 1 ? w / 2 : 1, nh = h > 1 ? h / 2 : 1;
            uint8_t *nx = (uint8_t *)malloc((size_t)nw * nh * bpp);
            if (!nx) break;
            if (bpp == 4) box32(nx, cur, w, h, nw, nh, point);
            else box16(nx, cur, w, h, nw, nh, t->g2fmt, point);
            g2_write_image(d->bd, t->handle, nx, (size_t)nw * nh * bpp, tf, 0, 0, (uint32_t)nw, (uint32_t)nh, (uint32_t)l, (uint32_t)f, 0, t->g2fmt);
            free(cur);
            cur = nx; w = nw; h = nh;
        }
        free(cur);
    }
    t->gen_dirty = 0;
    t->rev++;
}

void d3_tex_ready(d3tex *t)
{
    d3_tex_prepare(t);
    d3_tex_resolve(t);
    if (t->gen_dirty) gen_levels(t);
}

int d3_tex_base(const d3tex *x, const DWORD *ss)
{
    DWORD b = ss[D3DSAMP_MAXMIPLEVEL] > x->lod ? ss[D3DSAMP_MAXMIPLEVEL] : x->lod;
    return b >= (DWORD)x->hwl ? x->hwl - 1 : (int)b;
}

static void view_drop(d3tex *t)
{
    d3dev *d = t->dev;
    if (!t->view) return;
    g2r_end_pass(d->r);
    if (t->view_copy) g2r_image_destroy(d->r, t->view, t->view_base);
    else g2r_alias_destroy(d->r, t->view);
    t->view = 0;
}

/* a volume's chain is contiguous like a 2D one's, the small levels do not start on 32 bytes either */
static uint16_t vol_view(d3tex *t, int b, int n)
{
    d3dev *d = t->dev;
    uint32_t bpp = g2_storage_bpp(t->g2fmt), off = g2_vol_chain_bytes(t->w, t->h, t->d, bpp, (uint32_t)b);
    uint32_t w = t->w >> b, h = t->h >> b, de = t->d >> b;
    w = w ? w : 1; h = h ? h : 1; de = de ? de : 1;
    if (!(off & 31)) {
        uint16_t hd;
        t->view_base = t->base + off;
        if (g2r_volume_alias(d->r, &hd, t->view_base, w, h, de, t->g2fmt, (uint32_t)n, G2_USE_SAMPLED) < 0) return t->handle;
        t->view = hd; t->view_copy = 0;
    } else {
        uint8_t *px = (uint8_t *)malloc((size_t)w * h * bpp);
        int l;
        if (!px) return t->handle;
        if (g2r_volume_create(d->r, &t->view, &t->view_base, w, h, de, t->g2fmt, (uint32_t)n, G2_USE_SAMPLED | G2_USE_TRANSFER) < 0) { free(px); t->view = 0; return t->handle; }
        g2r_finish(d->r);
        for (l = 0; l < n; l++) {
            uint32_t lw = w >> l, lh = h >> l, ld = de >> l, z;
            lw = lw ? lw : 1; lh = lh ? lh : 1; ld = ld ? ld : 1;
            for (z = 0; z < ld; z++) {
                g2_read_image(d->bd, t->handle, G2_TF_RAW, 0, 0, lw, lh, (uint32_t)(b + l), z, t->g2fmt, 0, 0, px);
                g2_write_image(d->bd, t->view, px, (size_t)lw * lh * bpp, G2_TF_RAW, 0, 0, lw, lh, (uint32_t)l, z, 0, t->g2fmt);
            }
        }
        free(px);
        t->view_copy = 1; t->view_rev = t->rev;
    }
    t->view_b = b;
    return t->view;
}

/* a 2D chain is contiguous and its rows tight, so its tail is an image of its own at the same memory. Cube faces lie a whole chain apart, which a shorter chain is not: those levels go into an image of their own, once per change */
uint16_t d3_tex_image(d3tex *t, const DWORD *ss)
{
    d3dev *d = t->dev;
    int b = d3_tex_base(t, ss), n = t->hwl - b, l, f;
    uint32_t w, h, bpp = g2_storage_bpp(t->g2fmt);
    if (!b) return t->handle;
    if (t->view && t->view_b == b && (!t->view_copy || t->view_rev == t->rev)) return t->view;
    view_drop(t);
    if (t->vol) return vol_view(t, b, n);
    w = t->w >> b; h = t->h >> b;
    w = w ? w : 1; h = h ? h : 1;
    if (t->layers == 1 && !(g2_mip_chain_bytes(t->w, t->h, bpp, (uint32_t)b) & 31)) {      /* an image starts on a 32-byte boundary: the small levels of a chain do not */
        uint16_t hd;
        t->view_base = t->base + g2_mip_chain_bytes(t->w, t->h, bpp, (uint32_t)b);
        if (g2r_image_alias(d->r, &hd, t->view_base, w, h, t->g2fmt, w * bpp, 0, (uint32_t)n, G2_USE_SAMPLED) < 0) return t->handle;
        t->view = hd; t->view_copy = 0;
    } else {
        uint8_t *px = (uint8_t *)malloc((size_t)w * h * bpp);
        if (!px) return t->handle;
        if (g2r_image_create_tight(d->r, &t->view, &t->view_base, w, h, t->g2fmt, (uint32_t)n, (uint32_t)t->layers, 0, G2_USE_SAMPLED | G2_USE_TRANSFER) < 0) { free(px); t->view = 0; return t->handle; }
        g2r_finish(d->r);
        for (f = 0; f < t->layers; f++) {
            for (l = 0; l < n; l++) {
                uint32_t lw = w >> l, lh = h >> l;
                lw = lw ? lw : 1; lh = lh ? lh : 1;
                g2_read_image(d->bd, t->handle, G2_TF_RAW, 0, 0, lw, lh, (uint32_t)(b + l), (uint32_t)f, t->g2fmt, 0, 0, px);
                g2_write_image(d->bd, t->view, px, (size_t)lw * lh * bpp, G2_TF_RAW, 0, 0, lw, lh, (uint32_t)l, (uint32_t)f, 0, t->g2fmt);
            }
        }
        free(px);
        t->view_copy = 1; t->view_rev = t->rev;
    }
    t->view_b = b;
    return t->view;
}

static void rt_drop(d3tex *t)
{
    d3dev *d = t->dev;
    if (!t->rt && !t->rt_ds) return;                            /* nothing of its own to drop: the pass goes on */
    g2r_end_pass(d->r);
    if (t->rt_ds) g2r_alias_destroy(d->r, t->rt_ds);
    if (t->rt) g2r_image_destroy(d->r, t->rt, t->rt_base);
    t->rt = t->rt_ds = 0;
}

/* B renders level 0 of an image of one layer, at a pitch that is a multiple of 32. Anything else renders in an image of its own, copied across before the texture is used */
uint16_t d3_tex_target(d3tex *t, int face, int level, UINT w, UINT h)
{
    d3dev *d = t->dev;
    d3_tex_prepare(t);
    if (t->autogen) t->gen_dirty = 1;
    if (!level && t->layers == 1 && !(t->w * g2_storage_bpp(t->g2fmt) % 32)) { t->used_pass = g2r_pass_id(d->r); return t->handle; }
    if (t->rt && (t->rt_w != (int)w || t->rt_h != (int)h)) { d3_tex_resolve(t); rt_drop(t); t->rt_stale = 0; }
    if (!t->rt) {
        if (g2r_image_create(d->r, &t->rt, &t->rt_base, w, h, t->g2fmt, 1, 1, 0, G2_USE_RT | G2_USE_TRANSFER) < 0) { g2e_log("d3d9: no room for a render image"); t->rt = 0; return t->handle; }
        t->rt_stale = 1; t->rt_face = face; t->rt_level = level; t->rt_w = (int)w; t->rt_h = (int)h;
    }
    if (t->rt_face != face || t->rt_level != level) { d3_tex_resolve(t); t->rt_face = face; t->rt_level = level; t->rt_stale = 1; }
    if (t->rt_stale) { tex_copy_rt(t, 0); t->rt_stale = 0; }
    t->rt_dirty = 1;
    return t->rt;
}

uint16_t d3_tex_ds(d3tex *t)
{
    d3dev *d = t->dev;
    tex_storage(t);
    if (t->w * 4 == g2r_ds_pitch(t->w)) {                       /* its tight rows are as long as board B loads a depth row */
        if (!t->ds_alias && g2r_image_alias(d->r, &t->ds_alias, t->base, t->w, t->h, G2_FMT_D24S8, t->w * 4, 0, 1, G2_USE_RT) < 0) t->ds_alias = 0;
        return t->ds_alias;
    }
    if (!t->rt) {
        if (g2r_image_create_ds_rows(d->r, &t->rt, &t->rt_base, t->w, t->h, G2_FMT_RGBA8, 0, G2_USE_RT | G2_USE_TRANSFER) < 0) { t->rt = 0; return 0; }
        t->rt_w = (int)t->w; t->rt_h = (int)t->h; t->rt_level = t->rt_face = 0;
        tex_copy_rt(t, 0);
    }
    if (!t->rt_ds && g2r_image_alias(d->r, &t->rt_ds, t->rt_base, t->w, t->h, G2_FMT_D24S8, g2r_ds_pitch(t->w), 0, 1, G2_USE_RT) < 0) t->rt_ds = 0;
    t->rt_dirty = 1;
    return t->rt_ds;
}

/* where a surface's pixels are now */
static uint16_t surf_image(d3surf *s, uint32_t *level, uint32_t *layer)
{
    *level = *layer = 0;
    if (!s->tex) return s->handle;
    if (s->tex->rt && s->tex->rt_level == s->level && s->tex->rt_face == s->face) return s->tex->rt;
    *level = (uint32_t)s->level; *layer = (uint32_t)s->face;
    return s->tex->handle;
}

void d3_surf_read(d3surf *s)
{
    d3dev *d = s->dev;
    uint8_t *tmp;
    int bpp = s->g2fmt == G2_FMT_RGBA8 ? 4 : 2;
    uint32_t level, layer;
    uint16_t img = surf_image(s, &level, &layer);
    if (!img || !s->mem) return;
    g2r_finish(d->r);
    tmp = (uint8_t *)malloc((size_t)s->w * s->h * bpp);
    if (!tmp) return;
    if (g2_read_image(d->bd, img, bpp == 4 ? G2_TF_RGBA8888 : G2_TF_RAW, 0, 0, s->w, s->h, level, layer, s->g2fmt, 0, 0, tmp) >= 0) from_storage(s, tmp);
    free(tmp);
}

void d3_surf_write(d3surf *s)
{
    d3dev *d = s->dev;
    uint8_t *px;
    uint32_t tf;
    int bpp;
    uint32_t level, layer;
    uint16_t img = surf_image(s, &level, &layer);
    if (!img || !s->mem) return;
    px = to_storage(s, &tf, &bpp);
    g2r_end_pass(d->r);
    if (px) g2_write_image(d->bd, img, px, (size_t)s->w * s->h * bpp, tf, 0, 0, s->w, s->h, level, layer, 0, s->g2fmt);
    free(px);
    s->dirty = 0;
}

/* ---------------------------------------------------------------- COM boilerplate */
static HRESULT qi(void *self, REFIID id, const IID *own, const IID *own2, void **out)
{
    if (!out) return E_POINTER;
    if (IsEqualGUID(id, &IID_IUnknown) || IsEqualGUID(id, own) || (own2 && IsEqualGUID(id, own2))) { ((IUnknown *)self)->lpVtbl->AddRef((IUnknown *)self); *out = self; return S_OK; }
    *out = NULL;
    return E_NOINTERFACE;
}

/* resources share SetPrivateData and friends: nothing here reads them back except the game itself */
typedef struct priv { struct priv *next; const void *owner; GUID id; DWORD size; void *data; } priv;
static priv *g_priv;
static HRESULT priv_set(const void *o, REFGUID id, const void *d, DWORD n)
{
    priv *p = (priv *)malloc(sizeof *p);
    if (!p) return E_OUTOFMEMORY;
    p->data = malloc(n ? n : 1);
    if (!p->data) { free(p); return E_OUTOFMEMORY; }
    memcpy(p->data, d, n);
    p->owner = o; p->id = *id; p->size = n; p->next = g_priv;
    g_priv = p;
    return D3D_OK;
}
static HRESULT priv_get(const void *o, REFGUID id, void *d, DWORD *n)
{
    priv *p;
    for (p = g_priv; p; p = p->next) {
        if (p->owner == o && IsEqualGUID(&p->id, id)) {
            if (!d || *n < p->size) { *n = p->size; return D3DERR_MOREDATA; }
            memcpy(d, p->data, p->size); *n = p->size;
            return D3D_OK;
        }
    }
    return D3DERR_NOTFOUND;
}
static HRESULT priv_free(const void *o, REFGUID id)
{
    priv **q;
    for (q = &g_priv; *q; q = &(*q)->next) {
        if ((*q)->owner == o && IsEqualGUID(&(*q)->id, id)) { priv *p = *q; *q = p->next; free(p->data); free(p); return D3D_OK; }
    }
    return D3DERR_NOTFOUND;
}

/* ---------------------------------------------------------------- surfaces */
d3surf *d3_surface_new(d3dev *d, UINT w, UINT h, D3DFORMAT f, DWORD usage, D3DPOOL pool, int shadow)
{
    d3surf *s = (d3surf *)calloc(1, sizeof *s);
    if (!s) return NULL;
    s->vtbl = d3_vt_surface; s->refs = 1; s->dev = d;
    s->w = w; s->h = h; s->fmt = f; s->usage = usage; s->pool = pool;
    d3_fmt_g2(f, &s->g2fmt);
    if (shadow) {
        int bytes = shadow_layout(w, h, f, &s->pitch);
        s->mem = (uint8_t *)calloc(1, (size_t)bytes + 16);
        if (!s->mem) { free(s); return NULL; }
    }
    return s;
}

#define SURF(t) ((d3surf *)(t))
static HRESULT WINAPI s_QueryInterface(IDirect3DSurface9 *t, REFIID id, void **o) { return qi(t, id, &IID_IDirect3DSurface9, &IID_IDirect3DResource9, o); }
static ULONG WINAPI s_AddRef(IDirect3DSurface9 *t) { d3surf *s = SURF(t); if (s->tex) return ((IUnknown *)s->tex)->lpVtbl->AddRef((IUnknown *)s->tex); return (ULONG)(InterlockedIncrement(&s->refs) - s->binds); }
static ULONG WINAPI s_Release(IDirect3DSurface9 *t)
{
    d3surf *s = SURF(t);
    LONG r;
    if (s->tex) return ((IUnknown *)s->tex)->lpVtbl->Release((IUnknown *)s->tex);
    r = d3_unref(s);
    if (r < 0) return 0;
    if (!r) {
        if (s->handle && s->dev->r && !s->back) { g2r_end_pass(s->dev->r); g2r_image_destroy(s->dev->r, s->handle, s->base); }
        if (s->part && s->dev->r) g2r_image_destroy(s->dev->r, s->part, s->part_base);
        free(s->mem);
        free(s);
        return 0;
    }
    return (ULONG)(r - s->binds);
}
static HRESULT WINAPI s_GetDevice(IDirect3DSurface9 *t, IDirect3DDevice9 **o) { *o = (IDirect3DDevice9 *)SURF(t)->dev; IDirect3DDevice9_AddRef(*o); return D3D_OK; }
static HRESULT WINAPI s_SetPrivateData(IDirect3DSurface9 *t, REFGUID g, const void *d, DWORD n, DWORD f) { (void)f; return priv_set(t, g, d, n); }
static HRESULT WINAPI s_GetPrivateData(IDirect3DSurface9 *t, REFGUID g, void *d, DWORD *n) { return priv_get(t, g, d, n); }
static HRESULT WINAPI s_FreePrivateData(IDirect3DSurface9 *t, REFGUID g) { return priv_free(t, g); }
static DWORD WINAPI s_SetPriority(IDirect3DSurface9 *t, DWORD p) { (void)t; (void)p; return 0; }
static DWORD WINAPI s_GetPriority(IDirect3DSurface9 *t) { (void)t; return 0; }
static void WINAPI s_PreLoad(IDirect3DSurface9 *t) { (void)t; }
static D3DRESOURCETYPE WINAPI s_GetType(IDirect3DSurface9 *t) { (void)t; return D3DRTYPE_SURFACE; }
static HRESULT WINAPI s_GetContainer(IDirect3DSurface9 *t, REFIID id, void **o)
{
    d3surf *s = SURF(t);
    if (s->tex) return ((IUnknown *)s->tex)->lpVtbl->QueryInterface((IUnknown *)s->tex, id, o);
    return IDirect3DDevice9_QueryInterface((IDirect3DDevice9 *)s->dev, id, o);
}
static HRESULT WINAPI s_GetDesc(IDirect3DSurface9 *t, D3DSURFACE_DESC *o)
{
    d3surf *s = SURF(t);
    memset(o, 0, sizeof *o);
    o->Format = s->fmt; o->Type = D3DRTYPE_SURFACE; o->Usage = s->usage; o->Pool = s->pool; o->MultiSampleType = D3DMULTISAMPLE_NONE; o->Width = s->w; o->Height = s->h;
    if (s->tex && !s->level) { o->Width = s->tex->vw; o->Height = s->tex->vh; }
    if (s->back) { o->Width = (UINT)s->dev->vw; o->Height = (UINT)s->dev->vh; }
    if (s->vw) { o->Width = s->vw; o->Height = s->vh; }
    return D3D_OK;
}
static HRESULT WINAPI s_LockRect(IDirect3DSurface9 *t, D3DLOCKED_RECT *o, const RECT *r, DWORD f)
{
    d3surf *s = SURF(t);
    int b = d3_fmt_bytes(s->fmt);
    if (s->locked) return D3DERR_INVALIDCALL;
    if (!s->mem) {
        int bytes = shadow_layout(s->w, s->h, s->fmt, &s->pitch);
        s->mem = (uint8_t *)calloc(1, (size_t)bytes + 16);
        if (!s->mem) return E_OUTOFMEMORY;
    }
    /* a target's shadow lags the board */
    if ((s->usage & D3DUSAGE_RENDERTARGET || s->back) && !(f & D3DLOCK_DISCARD)) d3_surf_read(s);
    s->locked = 1;
    s->lock_flags = f;
    o->Pitch = s->pitch;
    if (r) {
        s->lock = *r;
        if (b) o->pBits = s->mem + (ptrdiff_t)r->top * s->pitch + (ptrdiff_t)r->left * b;
        else o->pBits = s->mem + (ptrdiff_t)(r->top / 4) * s->pitch + (ptrdiff_t)(r->left / 4) * (s->fmt == D3DFMT_DXT1 ? 8 : 16);
    } else {
        SetRect(&s->lock, 0, 0, (int)s->w, (int)s->h);
        o->pBits = s->mem;
    }
    return D3D_OK;
}
static HRESULT WINAPI s_UnlockRect(IDirect3DSurface9 *t)
{
    d3surf *s = SURF(t);
    if (!s->locked) return D3DERR_INVALIDCALL;
    s->locked = 0;
    if (s->lock_flags & D3DLOCK_READONLY) return D3D_OK;
    s->dirty = 1;
    if ((s->usage & D3DUSAGE_RENDERTARGET || s->back) && !s->tex) d3_surf_write(s);
    return D3D_OK;
}
static HRESULT WINAPI s_GetDC(IDirect3DSurface9 *t, HDC *o) { (void)t; (void)o; return D3DERR_INVALIDCALL; }
static HRESULT WINAPI s_ReleaseDC(IDirect3DSurface9 *t, HDC o) { (void)t; (void)o; return D3DERR_INVALIDCALL; }

const IDirect3DSurface9Vtbl d3_surface_vtbl = {
    s_QueryInterface, s_AddRef, s_Release, s_GetDevice, s_SetPrivateData, s_GetPrivateData, s_FreePrivateData, s_SetPriority, s_GetPriority, s_PreLoad, s_GetType,
    s_GetContainer, s_GetDesc, s_LockRect, s_UnlockRect, s_GetDC, s_ReleaseDC
};

/* ---------------------------------------------------------------- textures */
HRESULT d3_texture_new(d3dev *d, UINT w, UINT h, UINT levels, DWORD usage, D3DFORMAT f, D3DPOOL pool, int cube, d3tex **out)
{
    d3tex *t;
    uint32_t g2;
    UINT full = 1, mw, i, vw = w, vh = h;
    int fc, l, depth = 0;
    if (usage & D3DUSAGE_DEPTHSTENCIL) {
        if (!d3_fmt_dtex(f) || cube) return D3DERR_INVALIDCALL;
        g2 = G2_FMT_RGBA8;
        depth = f == D3DFMT_D24S8 || f == D3DFMT_D24X8 || f == D3DFMT_D16 ? 2 : 1;
        levels = 1;
    } else if (!d3_fmt_g2(f, &g2)) { g2e_log("d3d9: texture format %u is not supported", (unsigned)f); return D3DERR_INVALIDCALL; }
    if (!w || !h) return D3DERR_INVALIDCALL;
    if ((usage & D3DUSAGE_RENDERTARGET) && (w > MAX_RT_W || h > MAX_TEX_SIZE)) {      /* a full-screen target on a big desktop, as the back buffer */
        double s = (double)MAX_RT_W / w;
        if ((double)MAX_TEX_SIZE / h < s) s = (double)MAX_TEX_SIZE / h;
        w = (UINT)(w * s + 0.5); h = (UINT)(h * s + 0.5);
        if (w > MAX_RT_W) w = MAX_RT_W;
        if (!h) h = 1;
    }
    if (w > MAX_TEX_SIZE || h > MAX_TEX_SIZE) return D3DERR_INVALIDCALL;
    mw = w > h ? w : h;
    while (mw > 1) { mw >>= 1; full++; }
    if (!levels || levels > full) levels = full;
    if (usage & D3DUSAGE_AUTOGENMIPMAP) levels = 1;
    t = (d3tex *)calloc(1, sizeof *t);
    if (!t) return E_OUTOFMEMORY;
    t->vtbl = cube ? d3_vt_cube : d3_vt_texture;
    t->refs = 1; t->dev = d; t->cube = cube; t->layers = cube ? 6 : 1; t->levels = (int)levels; t->depth = depth;
    t->hwl = usage & D3DUSAGE_AUTOGENMIPMAP ? (int)full : (int)levels;
    t->autogen = (usage & D3DUSAGE_AUTOGENMIPMAP) != 0;
    t->w = w; t->h = h; t->vw = vw; t->vh = vh; t->fmt = f; t->usage = usage; t->pool = pool; t->g2fmt = g2; t->used_pass = 0xFFFFFFFFu;
    t->autogen_filter = D3DTEXF_LINEAR;
    i = (UINT)t->layers * levels;
    t->surf = (d3surf **)calloc(i, sizeof *t->surf);
    if (!t->surf) { free(t); return E_OUTOFMEMORY; }
    for (fc = 0; fc < t->layers; fc++) {
        for (l = 0; l < (int)levels; l++) {
            UINT lw = w >> l, lh = h >> l;
            d3surf *s = d3_surface_new(d, lw ? lw : 1, lh ? lh : 1, f, usage, pool, 1);
            if (!s) return E_OUTOFMEMORY;
            s->tex = t; s->level = l; s->face = fc;
            s->g2fmt = g2;
            t->surf[fc * (int)levels + l] = s;
        }
    }
    *out = t;
    return D3D_OK;
}

static void tex_free(d3tex *t)
{
    d3dev *d = t->dev;
    int i;
    if (d->r) {
        /* the open pass ends only if it used this texture (its draws wait for the FLUSH and still need the texels), or holds its depth alias; rt_drop and view_drop end it for theirs. Ending
           it for every Release would cost a flush of the whole target and its reload for a texture the frame never touched */
        if (t->used_pass == g2r_pass_id(d->r) || t->ds_alias) g2r_end_pass(d->r);
        if (t->ds_alias) g2r_alias_destroy(d->r, t->ds_alias);
        rt_drop(t);
        view_drop(t);
        if (t->handle) g2r_image_destroy(d->r, t->handle, t->base);
    }
    if (t->vol) {
        for (i = 0; i < t->levels; i++) if (t->lv[i]) { free(t->lv[i]->mem); free(t->lv[i]); }
        free(t->lv);
    } else {
        for (i = 0; i < t->layers * t->levels; i++) { free(t->surf[i]->mem); free(t->surf[i]); }
        free(t->surf);
    }
    free(t);
}

#define TEX(p) ((d3tex *)(p))
#define TEX_COMMON(pfx, T, own, rtype) \
static HRESULT WINAPI pfx##QueryInterface(T *t, REFIID id, void **o) { HRESULT h = qi(t, id, own, &IID_IDirect3DBaseTexture9, o); return h == E_NOINTERFACE ? qi(t, id, &IID_IDirect3DResource9, NULL == o ? NULL : own, o) : h; } \
static ULONG WINAPI pfx##AddRef(T *t) { return (ULONG)(InterlockedIncrement(&TEX(t)->refs) - TEX(t)->binds); } \
static ULONG WINAPI pfx##Release(T *t) { LONG r = d3_unref(t); if (r < 0) return 0; if (!r) { tex_free(TEX(t)); return 0; } return (ULONG)(r - TEX(t)->binds); } \
static HRESULT WINAPI pfx##GetDevice(T *t, IDirect3DDevice9 **o) { *o = (IDirect3DDevice9 *)TEX(t)->dev; IDirect3DDevice9_AddRef(*o); return D3D_OK; } \
static HRESULT WINAPI pfx##SetPrivateData(T *t, REFGUID g, const void *d, DWORD n, DWORD f) { (void)f; return priv_set(t, g, d, n); } \
static HRESULT WINAPI pfx##GetPrivateData(T *t, REFGUID g, void *d, DWORD *n) { return priv_get(t, g, d, n); } \
static HRESULT WINAPI pfx##FreePrivateData(T *t, REFGUID g) { return priv_free(t, g); } \
static DWORD WINAPI pfx##SetPriority(T *t, DWORD p) { (void)t; (void)p; return 0; } \
static DWORD WINAPI pfx##GetPriority(T *t) { (void)t; return 0; } \
static void WINAPI pfx##PreLoad(T *t) { d3_tex_prepare(TEX(t)); } \
static D3DRESOURCETYPE WINAPI pfx##GetType(T *t) { (void)t; return rtype; } \
static DWORD WINAPI pfx##SetLOD(T *t, DWORD l) { DWORD o = TEX(t)->lod; TEX(t)->lod = l; return o; } \
static DWORD WINAPI pfx##GetLOD(T *t) { return TEX(t)->lod; } \
static DWORD WINAPI pfx##GetLevelCount(T *t) { return (DWORD)TEX(t)->levels; } \
static HRESULT WINAPI pfx##SetAutoGenFilterType(T *t, D3DTEXTUREFILTERTYPE f) { TEX(t)->autogen_filter = f; return D3D_OK; } \
static D3DTEXTUREFILTERTYPE WINAPI pfx##GetAutoGenFilterType(T *t) { return (D3DTEXTUREFILTERTYPE)TEX(t)->autogen_filter; } \
static void WINAPI pfx##GenerateMipSubLevels(T *t) { d3_tex_ready(TEX(t)); }
#define TEX_DESC(pfx, T) static HRESULT WINAPI pfx##GetLevelDesc(T *t, UINT l, D3DSURFACE_DESC *o) { if ((int)l >= TEX(t)->levels) return D3DERR_INVALIDCALL; return s_GetDesc((IDirect3DSurface9 *)TEX(t)->surf[l], o); }

TEX_COMMON(t2_, IDirect3DTexture9, &IID_IDirect3DTexture9, D3DRTYPE_TEXTURE)
TEX_COMMON(tc_, IDirect3DCubeTexture9, &IID_IDirect3DCubeTexture9, D3DRTYPE_CUBETEXTURE)
TEX_COMMON(tv_, IDirect3DVolumeTexture9, &IID_IDirect3DVolumeTexture9, D3DRTYPE_VOLUMETEXTURE)
TEX_DESC(t2_, IDirect3DTexture9)
TEX_DESC(tc_, IDirect3DCubeTexture9)

static HRESULT WINAPI t2_GetSurfaceLevel(IDirect3DTexture9 *t, UINT l, IDirect3DSurface9 **o)
{
    if ((int)l >= TEX(t)->levels) return D3DERR_INVALIDCALL;
    *o = (IDirect3DSurface9 *)TEX(t)->surf[l];
    IDirect3DSurface9_AddRef(*o);
    return D3D_OK;
}
static HRESULT WINAPI t2_LockRect(IDirect3DTexture9 *t, UINT l, D3DLOCKED_RECT *o, const RECT *r, DWORD f) { if ((int)l >= TEX(t)->levels) return D3DERR_INVALIDCALL; return s_LockRect((IDirect3DSurface9 *)TEX(t)->surf[l], o, r, f); }
static HRESULT WINAPI t2_UnlockRect(IDirect3DTexture9 *t, UINT l) { if ((int)l >= TEX(t)->levels) return D3DERR_INVALIDCALL; return s_UnlockRect((IDirect3DSurface9 *)TEX(t)->surf[l]); }
static HRESULT WINAPI t2_AddDirtyRect(IDirect3DTexture9 *t, const RECT *r) { int l; (void)r; for (l = 0; l < TEX(t)->levels; l++) TEX(t)->surf[l]->dirty = 1; return D3D_OK; }

const IDirect3DTexture9Vtbl d3_texture_vtbl = {
    t2_QueryInterface, t2_AddRef, t2_Release, t2_GetDevice, t2_SetPrivateData, t2_GetPrivateData, t2_FreePrivateData, t2_SetPriority, t2_GetPriority, t2_PreLoad, t2_GetType,
    t2_SetLOD, t2_GetLOD, t2_GetLevelCount, t2_SetAutoGenFilterType, t2_GetAutoGenFilterType, t2_GenerateMipSubLevels,
    t2_GetLevelDesc, t2_GetSurfaceLevel, t2_LockRect, t2_UnlockRect, t2_AddDirtyRect
};

static d3surf *face_surf(d3tex *t, D3DCUBEMAP_FACES f, UINT l) { return (int)l < t->levels && f <= D3DCUBEMAP_FACE_NEGATIVE_Z ? t->surf[(int)f * t->levels + (int)l] : NULL; }
static HRESULT WINAPI tc_GetCubeMapSurface(IDirect3DCubeTexture9 *t, D3DCUBEMAP_FACES f, UINT l, IDirect3DSurface9 **o)
{
    d3surf *s = face_surf(TEX(t), f, l);
    if (!s) return D3DERR_INVALIDCALL;
    *o = (IDirect3DSurface9 *)s;
    IDirect3DSurface9_AddRef(*o);
    return D3D_OK;
}
static HRESULT WINAPI tc_LockRect(IDirect3DCubeTexture9 *t, D3DCUBEMAP_FACES f, UINT l, D3DLOCKED_RECT *o, const RECT *r, DWORD fl) { d3surf *s = face_surf(TEX(t), f, l); return s ? s_LockRect((IDirect3DSurface9 *)s, o, r, fl) : D3DERR_INVALIDCALL; }
static HRESULT WINAPI tc_UnlockRect(IDirect3DCubeTexture9 *t, D3DCUBEMAP_FACES f, UINT l) { d3surf *s = face_surf(TEX(t), f, l); return s ? s_UnlockRect((IDirect3DSurface9 *)s) : D3DERR_INVALIDCALL; }
static HRESULT WINAPI tc_AddDirtyRect(IDirect3DCubeTexture9 *t, D3DCUBEMAP_FACES f, const RECT *r) { d3surf *s = face_surf(TEX(t), f, 0); (void)r; if (s) s->dirty = 1; return D3D_OK; }

const IDirect3DCubeTexture9Vtbl d3_cube_vtbl = {
    tc_QueryInterface, tc_AddRef, tc_Release, tc_GetDevice, tc_SetPrivateData, tc_GetPrivateData, tc_FreePrivateData, tc_SetPriority, tc_GetPriority, tc_PreLoad, tc_GetType,
    tc_SetLOD, tc_GetLOD, tc_GetLevelCount, tc_SetAutoGenFilterType, tc_GetAutoGenFilterType, tc_GenerateMipSubLevels,
    tc_GetLevelDesc, tc_GetCubeMapSurface, tc_LockRect, tc_UnlockRect, tc_AddDirtyRect
};

/* ---------------------------------------------------------------- volume textures: where board B has 3D textures (d3_has_3d). Each level is a shadow of its slices, uploaded when it is drawn with */
#define VOL(p) ((d3vol *)(p))
static HRESULT WINAPI v_QueryInterface(IDirect3DVolume9 *t, REFIID id, void **o) { return qi(t, id, &IID_IDirect3DVolume9, NULL, o); }
static ULONG WINAPI v_AddRef(IDirect3DVolume9 *t) { return ((IUnknown *)VOL(t)->tex)->lpVtbl->AddRef((IUnknown *)VOL(t)->tex); }
static ULONG WINAPI v_Release(IDirect3DVolume9 *t) { return ((IUnknown *)VOL(t)->tex)->lpVtbl->Release((IUnknown *)VOL(t)->tex); }
static HRESULT WINAPI v_GetDevice(IDirect3DVolume9 *t, IDirect3DDevice9 **o) { *o = (IDirect3DDevice9 *)VOL(t)->tex->dev; IDirect3DDevice9_AddRef(*o); return D3D_OK; }
static HRESULT WINAPI v_SetPrivateData(IDirect3DVolume9 *t, REFGUID g, const void *d, DWORD n, DWORD f) { (void)f; return priv_set(t, g, d, n); }
static HRESULT WINAPI v_GetPrivateData(IDirect3DVolume9 *t, REFGUID g, void *d, DWORD *n) { return priv_get(t, g, d, n); }
static HRESULT WINAPI v_FreePrivateData(IDirect3DVolume9 *t, REFGUID g) { return priv_free(t, g); }
static HRESULT WINAPI v_GetContainer(IDirect3DVolume9 *t, REFIID id, void **o) { return ((IUnknown *)VOL(t)->tex)->lpVtbl->QueryInterface((IUnknown *)VOL(t)->tex, id, o); }
static HRESULT WINAPI v_GetDesc(IDirect3DVolume9 *t, D3DVOLUME_DESC *o)
{
    const d3vol *v = VOL(t);
    memset(o, 0, sizeof *o);
    o->Format = v->tex->fmt; o->Type = D3DRTYPE_VOLUME; o->Usage = v->tex->usage; o->Pool = v->tex->pool; o->Width = v->w; o->Height = v->h; o->Depth = v->d;
    return D3D_OK;
}
static HRESULT WINAPI v_LockBox(IDirect3DVolume9 *t, D3DLOCKED_BOX *o, const D3DBOX *b, DWORD f)
{
    d3vol *v = VOL(t);
    int n = d3_fmt_bytes(v->tex->fmt);
    if (v->locked) return D3DERR_INVALIDCALL;
    if (b && (b->Left >= b->Right || b->Top >= b->Bottom || b->Front >= b->Back || b->Right > v->w || b->Bottom > v->h || b->Back > v->d)) return D3DERR_INVALIDCALL;
    v->locked = 1;
    v->lock_flags = f;
    o->RowPitch = v->pitch; o->SlicePitch = v->slice;
    o->pBits = v->mem;
    if (b) {
        if (n) o->pBits = v->mem + (ptrdiff_t)b->Front * v->slice + (ptrdiff_t)b->Top * v->pitch + (ptrdiff_t)b->Left * n;
        else o->pBits = v->mem + (ptrdiff_t)b->Front * v->slice + (ptrdiff_t)(b->Top / 4) * v->pitch + (ptrdiff_t)(b->Left / 4) * (v->tex->fmt == D3DFMT_DXT1 ? 8 : 16);
    }
    return D3D_OK;
}
static HRESULT WINAPI v_UnlockBox(IDirect3DVolume9 *t)
{
    d3vol *v = VOL(t);
    if (!v->locked) return D3DERR_INVALIDCALL;
    v->locked = 0;
    if (!(v->lock_flags & D3DLOCK_READONLY)) v->dirty = 1;
    return D3D_OK;
}

const IDirect3DVolume9Vtbl d3_volume_vtbl = { v_QueryInterface, v_AddRef, v_Release, v_GetDevice, v_SetPrivateData, v_GetPrivateData, v_FreePrivateData, v_GetContainer, v_GetDesc, v_LockBox, v_UnlockBox };

static HRESULT WINAPI tv_GetLevelDesc(IDirect3DVolumeTexture9 *t, UINT l, D3DVOLUME_DESC *o) { if ((int)l >= TEX(t)->levels) return D3DERR_INVALIDCALL; return v_GetDesc((IDirect3DVolume9 *)TEX(t)->lv[l], o); }
static HRESULT WINAPI tv_GetVolumeLevel(IDirect3DVolumeTexture9 *t, UINT l, IDirect3DVolume9 **o)
{
    if ((int)l >= TEX(t)->levels) return D3DERR_INVALIDCALL;
    *o = (IDirect3DVolume9 *)TEX(t)->lv[l];
    IDirect3DVolume9_AddRef(*o);
    return D3D_OK;
}
static HRESULT WINAPI tv_LockBox(IDirect3DVolumeTexture9 *t, UINT l, D3DLOCKED_BOX *o, const D3DBOX *b, DWORD f) { if ((int)l >= TEX(t)->levels) return D3DERR_INVALIDCALL; return v_LockBox((IDirect3DVolume9 *)TEX(t)->lv[l], o, b, f); }
static HRESULT WINAPI tv_UnlockBox(IDirect3DVolumeTexture9 *t, UINT l) { if ((int)l >= TEX(t)->levels) return D3DERR_INVALIDCALL; return v_UnlockBox((IDirect3DVolume9 *)TEX(t)->lv[l]); }
static HRESULT WINAPI tv_AddDirtyBox(IDirect3DVolumeTexture9 *t, const D3DBOX *b) { int l; (void)b; for (l = 0; l < TEX(t)->levels; l++) TEX(t)->lv[l]->dirty = 1; return D3D_OK; }

const IDirect3DVolumeTexture9Vtbl d3_voltex_vtbl = {
    tv_QueryInterface, tv_AddRef, tv_Release, tv_GetDevice, tv_SetPrivateData, tv_GetPrivateData, tv_FreePrivateData, tv_SetPriority, tv_GetPriority, tv_PreLoad, tv_GetType,
    tv_SetLOD, tv_GetLOD, tv_GetLevelCount, tv_SetAutoGenFilterType, tv_GetAutoGenFilterType, tv_GenerateMipSubLevels,
    tv_GetLevelDesc, tv_GetVolumeLevel, tv_LockBox, tv_UnlockBox, tv_AddDirtyBox
};

HRESULT d3_volume_new(d3dev *d, UINT w, UINT h, UINT de, UINT levels, DWORD usage, D3DFORMAT f, D3DPOOL pool, d3tex **out)
{
    d3tex *t;
    uint32_t g2;
    UINT full = 1, m = w > h ? w : h;
    int l;
    if (!d3_has_3d()) { g2e_log("d3d9: no volume textures: board B has no 3D textures (the caps say so)"); return D3DERR_NOTAVAILABLE; }
    if (!d3_fmt_g2(f, &g2)) { g2e_log("d3d9: texture format %u is not supported", (unsigned)f); return D3DERR_INVALIDCALL; }
    if (!w || !h || !de || w > G2R_VOL_WH || h > G2R_VOL_WH || de > G2R_VOL_D || (usage & (D3DUSAGE_RENDERTARGET | D3DUSAGE_DEPTHSTENCIL | D3DUSAGE_AUTOGENMIPMAP))) return D3DERR_INVALIDCALL;
    if (de > m) m = de;
    while (m > 1) { m >>= 1; full++; }
    if (!levels || levels > full) levels = full;
    t = (d3tex *)calloc(1, sizeof *t);
    if (!t) return E_OUTOFMEMORY;
    t->vtbl = d3_vt_voltex;
    t->refs = 1; t->dev = d; t->vol = 1; t->layers = 1; t->levels = t->hwl = (int)levels;
    t->w = t->vw = w; t->h = t->vh = h; t->d = de; t->fmt = f; t->usage = usage; t->pool = pool; t->g2fmt = g2; t->used_pass = 0xFFFFFFFFu;
    t->autogen_filter = D3DTEXF_LINEAR;
    t->lv = (d3vol **)calloc(levels, sizeof *t->lv);
    if (!t->lv) { free(t); return E_OUTOFMEMORY; }
    for (l = 0; l < (int)levels; l++) {
        UINT lw = w >> l, lh = h >> l, ld = de >> l;
        d3vol *v = (d3vol *)calloc(1, sizeof *v);
        if (!v) { tex_free(t); return E_OUTOFMEMORY; }
        t->lv[l] = v;
        v->vtbl = d3_vt_volume; v->tex = t; v->level = l;
        v->w = lw ? lw : 1; v->h = lh ? lh : 1; v->d = ld ? ld : 1;
        v->slice = shadow_layout(v->w, v->h, f, &v->pitch);
        v->mem = (uint8_t *)calloc(1, (size_t)v->slice * v->d + 16);
        if (!v->mem) { tex_free(t); return E_OUTOFMEMORY; }
    }
    *out = t;
    return D3D_OK;
}

/* ---------------------------------------------------------------- vertex and index buffers: memory on the PC */
#define BUF(p) ((d3buf *)(p))
#define BUF_COMMON(pfx, T, own, rtype) \
static HRESULT WINAPI pfx##QueryInterface(T *t, REFIID id, void **o) { return qi(t, id, own, &IID_IDirect3DResource9, o); } \
static ULONG WINAPI pfx##AddRef(T *t) { return (ULONG)(InterlockedIncrement(&BUF(t)->refs) - BUF(t)->binds); } \
static ULONG WINAPI pfx##Release(T *t) \
{ \
    LONG r = d3_unref(t); \
    if (r < 0) return 0; \
    if (!r) { g2r_keep_changed(BUF(t)->dev->r, &BUF(t)->keep); free(BUF(t)->mem); free(t); return 0; } \
    return (ULONG)(r - BUF(t)->binds); \
} \
static HRESULT WINAPI pfx##GetDevice(T *t, IDirect3DDevice9 **o) { *o = (IDirect3DDevice9 *)BUF(t)->dev; IDirect3DDevice9_AddRef(*o); return D3D_OK; } \
static HRESULT WINAPI pfx##SetPrivateData(T *t, REFGUID g, const void *d, DWORD n, DWORD f) { (void)f; return priv_set(t, g, d, n); } \
static HRESULT WINAPI pfx##GetPrivateData(T *t, REFGUID g, void *d, DWORD *n) { return priv_get(t, g, d, n); } \
static HRESULT WINAPI pfx##FreePrivateData(T *t, REFGUID g) { return priv_free(t, g); } \
static DWORD WINAPI pfx##SetPriority(T *t, DWORD p) { (void)t; (void)p; return 0; } \
static DWORD WINAPI pfx##GetPriority(T *t) { (void)t; return 0; } \
static void WINAPI pfx##PreLoad(T *t) { (void)t; } \
static D3DRESOURCETYPE WINAPI pfx##GetType(T *t) { (void)t; return rtype; } \
static HRESULT WINAPI pfx##Lock(T *t, UINT off, UINT n, void **o, DWORD f) { (void)n; if (off > BUF(t)->size) return D3DERR_INVALIDCALL; if (!(f & D3DLOCK_READONLY)) g2r_keep_changed(BUF(t)->dev->r, &BUF(t)->keep); *o = BUF(t)->mem + off; return D3D_OK; } \
static HRESULT WINAPI pfx##Unlock(T *t) { (void)t; return D3D_OK; }

BUF_COMMON(vb_, IDirect3DVertexBuffer9, &IID_IDirect3DVertexBuffer9, D3DRTYPE_VERTEXBUFFER)
BUF_COMMON(ib_, IDirect3DIndexBuffer9, &IID_IDirect3DIndexBuffer9, D3DRTYPE_INDEXBUFFER)

static HRESULT WINAPI vb_GetDesc(IDirect3DVertexBuffer9 *t, D3DVERTEXBUFFER_DESC *o)
{
    memset(o, 0, sizeof *o);
    o->Format = D3DFMT_VERTEXDATA; o->Type = D3DRTYPE_VERTEXBUFFER; o->Usage = BUF(t)->usage; o->Pool = BUF(t)->pool; o->Size = BUF(t)->size; o->FVF = BUF(t)->fvf;
    return D3D_OK;
}
static HRESULT WINAPI ib_GetDesc(IDirect3DIndexBuffer9 *t, D3DINDEXBUFFER_DESC *o)
{
    memset(o, 0, sizeof *o);
    o->Format = BUF(t)->fmt; o->Type = D3DRTYPE_INDEXBUFFER; o->Usage = BUF(t)->usage; o->Pool = BUF(t)->pool; o->Size = BUF(t)->size;
    return D3D_OK;
}

const IDirect3DVertexBuffer9Vtbl d3_vb_vtbl = { vb_QueryInterface, vb_AddRef, vb_Release, vb_GetDevice, vb_SetPrivateData, vb_GetPrivateData, vb_FreePrivateData, vb_SetPriority, vb_GetPriority, vb_PreLoad, vb_GetType, vb_Lock, vb_Unlock, vb_GetDesc };
const IDirect3DIndexBuffer9Vtbl d3_ib_vtbl = { ib_QueryInterface, ib_AddRef, ib_Release, ib_GetDevice, ib_SetPrivateData, ib_GetPrivateData, ib_FreePrivateData, ib_SetPriority, ib_GetPriority, ib_PreLoad, ib_GetType, ib_Lock, ib_Unlock, ib_GetDesc };

const void *d3_vt_surface = &d3_surface_vtbl, *d3_vt_texture = &d3_texture_vtbl, *d3_vt_cube = &d3_cube_vtbl, *d3_vt_volume = &d3_volume_vtbl, *d3_vt_voltex = &d3_voltex_vtbl, *d3_vt_vb = &d3_vb_vtbl, *d3_vt_ib = &d3_ib_vtbl, *d3_vt_device = &d3_dev_vtbl;

/* ---------------------------------------------------------------- declarations, shaders, queries */
static HRESULT WINAPI de_QueryInterface(IDirect3DVertexDeclaration9 *t, REFIID id, void **o) { return qi(t, id, &IID_IDirect3DVertexDeclaration9, NULL, o); }
static ULONG WINAPI de_AddRef(IDirect3DVertexDeclaration9 *t) { d3decl *d = (d3decl *)t; return (ULONG)(InterlockedIncrement(&d->refs) - d->binds); }
static ULONG WINAPI de_Release(IDirect3DVertexDeclaration9 *t) { LONG r = d3_unref(t); if (r < 0) return 0; if (!r) { free(t); return 0; } return (ULONG)(r - ((d3decl *)t)->binds); }
static HRESULT WINAPI de_GetDevice(IDirect3DVertexDeclaration9 *t, IDirect3DDevice9 **o) { *o = (IDirect3DDevice9 *)((d3decl *)t)->dev; IDirect3DDevice9_AddRef(*o); return D3D_OK; }
static HRESULT WINAPI de_GetDeclaration(IDirect3DVertexDeclaration9 *t, D3DVERTEXELEMENT9 *o, UINT *n)
{
    d3decl *d = (d3decl *)t;
    if (o) memcpy(o, d->el, sizeof(D3DVERTEXELEMENT9) * (size_t)(d->n + 1));
    *n = (UINT)d->n + 1;
    return D3D_OK;
}
const IDirect3DVertexDeclaration9Vtbl d3_decl_vtbl = { de_QueryInterface, de_AddRef, de_Release, de_GetDevice, de_GetDeclaration };

#define SHADER_COM(pfx, T, own) \
static HRESULT WINAPI pfx##QueryInterface(T *t, REFIID id, void **o) { return qi(t, id, own, NULL, o); } \
static ULONG WINAPI pfx##AddRef(T *t) { d3shader *s = (d3shader *)t; return (ULONG)(InterlockedIncrement(&s->refs) - s->binds); } \
static ULONG WINAPI pfx##Release(T *t) \
{ \
    d3shader *s = (d3shader *)t; \
    LONG r = d3_unref(s); \
    if (r < 0) return 0; \
    if (!r) { free(s->tok); free(s->body); free(s->decl); free(s->defc); free(s); return 0; } \
    return (ULONG)(r - s->binds); \
} \
static HRESULT WINAPI pfx##GetDevice(T *t, IDirect3DDevice9 **o) { *o = (IDirect3DDevice9 *)((d3shader *)t)->dev; IDirect3DDevice9_AddRef(*o); return D3D_OK; } \
static HRESULT WINAPI pfx##GetFunction(T *t, void *o, UINT *n) { d3shader *s = (d3shader *)t; if (o) memcpy(o, s->tok, (size_t)s->ntok * 4); *n = s->ntok * 4; return D3D_OK; }
SHADER_COM(vs_, IDirect3DVertexShader9, &IID_IDirect3DVertexShader9)
SHADER_COM(ps_, IDirect3DPixelShader9, &IID_IDirect3DPixelShader9)
const IDirect3DVertexShader9Vtbl d3_vs_vtbl = { vs_QueryInterface, vs_AddRef, vs_Release, vs_GetDevice, vs_GetFunction };
const IDirect3DPixelShader9Vtbl d3_ps_vtbl = { ps_QueryInterface, ps_AddRef, ps_Release, ps_GetDevice, ps_GetFunction };

static HRESULT WINAPI q_QueryInterface(IDirect3DQuery9 *t, REFIID id, void **o) { return qi(t, id, &IID_IDirect3DQuery9, NULL, o); }
static ULONG WINAPI q_AddRef(IDirect3DQuery9 *t) { return (ULONG)InterlockedIncrement(&((d3query *)t)->refs); }
static ULONG WINAPI q_Release(IDirect3DQuery9 *t) { LONG r = InterlockedDecrement(&((d3query *)t)->refs); if (!r) free(t); return (ULONG)r; }
static HRESULT WINAPI q_GetDevice(IDirect3DQuery9 *t, IDirect3DDevice9 **o) { *o = (IDirect3DDevice9 *)((d3query *)t)->dev; IDirect3DDevice9_AddRef(*o); return D3D_OK; }
static D3DQUERYTYPE WINAPI q_GetType(IDirect3DQuery9 *t) { return ((d3query *)t)->type; }
static DWORD WINAPI q_GetDataSize(IDirect3DQuery9 *t)
{
    switch (((d3query *)t)->type) {
    case D3DQUERYTYPE_EVENT: case D3DQUERYTYPE_TIMESTAMPDISJOINT: return sizeof(BOOL);
    case D3DQUERYTYPE_TIMESTAMP: case D3DQUERYTYPE_TIMESTAMPFREQ: return sizeof(UINT64);
    case D3DQUERYTYPE_VCACHE: return sizeof(D3DDEVINFO_VCACHE);
    default: return 0;
    }
}
/* an event says the board has done everything before it; a timestamp is read here at that moment, on the PC's clock: it counts what the hardware has no counter for */
static HRESULT WINAPI q_Issue(IDirect3DQuery9 *t, DWORD f)
{
    d3query *q = (d3query *)t;
    if (f & D3DISSUE_END) {
        g2r_finish(q->dev->r);
        q->issued = 1;
        if (q->type == D3DQUERYTYPE_TIMESTAMP) QueryPerformanceCounter(&q->stamp);
    }
    return D3D_OK;
}
static HRESULT WINAPI q_GetData(IDirect3DQuery9 *t, void *o, DWORD n, DWORD f)
{
    d3query *q = (d3query *)t;
    (void)f;
    if (!q->issued && q->type != D3DQUERYTYPE_VCACHE) return S_FALSE;
    if (!o || n < q_GetDataSize(t)) return S_OK;
    switch (q->type) {
    case D3DQUERYTYPE_TIMESTAMP: *(UINT64 *)o = (UINT64)q->stamp.QuadPart; break;
    case D3DQUERYTYPE_TIMESTAMPFREQ: { LARGE_INTEGER h; QueryPerformanceFrequency(&h); *(UINT64 *)o = (UINT64)h.QuadPart; break; }
    case D3DQUERYTYPE_TIMESTAMPDISJOINT: *(BOOL *)o = FALSE; break;
    case D3DQUERYTYPE_VCACHE: {                                 /* A runs every index, no post-transform cache: a mesh gains nothing from an order */
        D3DDEVINFO_VCACHE c;
        c.Pattern = MAKEFOURCC('C', 'A', 'C', 'H'); c.OptMethod = 0; c.CacheSize = 1; c.MagicNumber = 0;
        *(D3DDEVINFO_VCACHE *)o = c;
        break;
    }
    default: *(BOOL *)o = TRUE; break;
    }
    return S_OK;
}
const IDirect3DQuery9Vtbl d3_query_vtbl = { q_QueryInterface, q_AddRef, q_Release, q_GetDevice, q_GetType, q_GetDataSize, q_Issue, q_GetData };

/* ---------------------------------------------------------------- state blocks */
static void mask_all(d3mask *m) { memset(m, 0xFF, sizeof *m); }

/* a texture's level counts in its texture, as its AddRef and Release do */
static d3ref *ref_owner(void *o)
{
    const void *vt = ((d3ref *)o)->vtbl;
    if ((vt == d3_vt_surface || vt == (const void *)&d3_surface_vtbl) && ((d3surf *)o)->tex) return (d3ref *)((d3surf *)o)->tex;
    return (d3ref *)o;
}
LONG d3_unref(void *o)
{
    d3ref *h = (d3ref *)o;
    for (;;) {
        LONG r = h->refs;
        if (r <= h->binds) return -1;
        if (InterlockedCompareExchange(&h->refs, r - 1, r) == r) return r - 1;
    }
}
void d3_bind(void *o) { ((IUnknown *)o)->lpVtbl->AddRef((IUnknown *)o); InterlockedIncrement(&ref_owner(o)->binds); }
void d3_unbind(void *o) { InterlockedDecrement(&ref_owner(o)->binds); ((IUnknown *)o)->lpVtbl->Release((IUnknown *)o); }

void d3_ref_set(void *slot, void *v)
{
    IUnknown **s = (IUnknown **)slot, *o = *s;
    if (o == (IUnknown *)v) return;
    if (v) d3_bind(v);
    *s = (IUnknown *)v;
    if (o) d3_unbind(o);
}

int d3_shadow_bytes(const d3surf *s) { int p; return shadow_layout(s->w, s->h, s->fmt, &p); }

HRESULT d3_block_new(d3dev *d, int all, d3block **out)
{
    d3block *b = (d3block *)calloc(1, sizeof *b);
    if (!b) return E_OUTOFMEMORY;
    b->vtbl = &d3_block_vtbl; b->refs = 1; b->dev = d;
    if (all) { mask_all(&b->m); d3_block_capture(b); }
    *out = b;
    return D3D_OK;
}

/* one direction of Capture / Apply: mask bits say what moves */
static void block_copy(d3state *to, const d3state *from, const d3mask *m)
{
    int i, j;
    for (i = 0; i < MAX_RS; i++) if (m->rs[i / 32] >> (i % 32) & 1) to->rs[i] = from->rs[i];
    for (i = 0; i < MAX_STAGES; i++) for (j = 0; j < MAX_TSS; j++) if (m->tss[i] >> j & 1) to->tss[i][j] = from->tss[i][j];
    for (i = 0; i < MAX_SAMPLERS + 5; i++) for (j = 0; j < MAX_SS; j++) if (m->ss[i] >> j & 1) to->ss[i][j] = from->ss[i][j];
    for (i = 0; i < 32 && i < MAX_SAMPLERS + 5; i++) if (m->tex >> i & 1) d3_ref_set(&to->tex[i], from->tex[i]);
    for (i = 0; i < MAX_STREAMS; i++) if (m->vb >> i & 1) { d3_ref_set(&to->vb[i], from->vb[i]); to->vb_off[i] = from->vb_off[i]; to->vb_stride[i] = from->vb_stride[i]; to->vb_freq[i] = from->vb_freq[i]; }
    if (m->ib) { d3_ref_set(&to->ib, from->ib); to->ib_base = from->ib_base; }
    if (m->decl) d3_ref_set(&to->decl, from->decl);
    if (m->fvf) to->fvf = from->fvf;
    if (m->vs) d3_ref_set(&to->vs, from->vs);
    if (m->ps) d3_ref_set(&to->ps, from->ps);
    if (m->xf) { to->world = from->world; to->view = from->view; to->proj = from->proj; memcpy(to->texm, from->texm, sizeof to->texm); memcpy(to->wm, from->wm, sizeof to->wm); }
    if (m->mat) to->mat = from->mat;
    if (m->light) { memcpy(to->light, from->light, sizeof to->light); memcpy(to->light_on, from->light_on, sizeof to->light_on); }
    if (m->clip) memcpy(to->clip, from->clip, sizeof to->clip);
    if (m->vp) to->vp = from->vp;
    if (m->sc) to->sc = from->sc;
    if (m->vc) { memcpy(to->vc, from->vc, sizeof to->vc); memcpy(to->vi, from->vi, sizeof to->vi); memcpy(to->vb_c, from->vb_c, sizeof to->vb_c); }
    if (m->pc) { memcpy(to->pc, from->pc, sizeof to->pc); memcpy(to->pi, from->pi, sizeof to->pi); memcpy(to->pb_c, from->pb_c, sizeof to->pb_c); }
}

void d3_block_capture(d3block *b) { block_copy(&b->s, &b->dev->st, &b->m); }
void d3_block_apply(d3block *b) { block_copy(&b->dev->st, &b->s, &b->m); }

static HRESULT WINAPI b_QueryInterface(IDirect3DStateBlock9 *t, REFIID id, void **o) { return qi(t, id, &IID_IDirect3DStateBlock9, NULL, o); }
static ULONG WINAPI b_AddRef(IDirect3DStateBlock9 *t) { return (ULONG)InterlockedIncrement(&((d3block *)t)->refs); }
static ULONG WINAPI b_Release(IDirect3DStateBlock9 *t)
{
    d3block *b = (d3block *)t;
    LONG r = InterlockedDecrement(&b->refs);
    if (!r) {
        int i;
        for (i = 0; i < MAX_SAMPLERS + 5; i++) d3_ref_set(&b->s.tex[i], NULL);
        for (i = 0; i < MAX_STREAMS; i++) d3_ref_set(&b->s.vb[i], NULL);
        d3_ref_set(&b->s.ib, NULL); d3_ref_set(&b->s.decl, NULL); d3_ref_set(&b->s.vs, NULL); d3_ref_set(&b->s.ps, NULL);
        free(b);
    }
    return (ULONG)r;
}
static HRESULT WINAPI b_GetDevice(IDirect3DStateBlock9 *t, IDirect3DDevice9 **o) { *o = (IDirect3DDevice9 *)((d3block *)t)->dev; IDirect3DDevice9_AddRef(*o); return D3D_OK; }
static HRESULT WINAPI b_Capture(IDirect3DStateBlock9 *t) { d3_block_capture((d3block *)t); return D3D_OK; }
static HRESULT WINAPI b_Apply(IDirect3DStateBlock9 *t) { d3_block_apply((d3block *)t); return D3D_OK; }
const IDirect3DStateBlock9Vtbl d3_block_vtbl = { b_QueryInterface, b_AddRef, b_Release, b_GetDevice, b_Capture, b_Apply };

/* ---------------------------------------------------------------- the string helper */
void d3sb_printf(d3sb *b, const char *fmt, ...)
{
    va_list a;
    int n;
    if (b->cap - b->n < 2048) {
        size_t cap = b->cap ? b->cap * 2 : 8192;
        char *p = (char *)realloc(b->p, cap);
        if (!p) return;
        b->p = p; b->cap = cap;
    }
    va_start(a, fmt);
    n = vsnprintf(b->p + b->n, b->cap - b->n, fmt, a);
    va_end(a);
    if (n > 0) b->n += (size_t)n < b->cap - b->n ? (size_t)n : b->cap - b->n - 1;
}
