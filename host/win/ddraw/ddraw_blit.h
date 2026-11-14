/* pixel formats and the CPU blitter of the DirectDraw library: every surface has a shadow in PC memory in the game's format, and
 * every blit runs on it, so Lock never has to read a surface back; the GPU repeats the blits it can (ddraw_main.c). The rules match the
 * hardware's (fb_2d.py model_copy): nearest at pixel centres u = ((2i + 1) sw) / (2 dw), mirrors after the stretch. */
#ifndef DDRAW_BLIT_H
#define DDRAW_BLIT_H
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

enum { F_P8, F_565, F_555, F_888, F_X888, F_A888, F_COUNT };

typedef struct fmt_info {
    int bytes;                  /* per pixel in the shadow */
    uint32_t r, g, b, a;        /* channel masks; P8: none */
    int g2_fmt, g2_tf;          /* the image's storage format and the transfer format of the shadow's pixels; -1: no image */
} fmt_info;

static const fmt_info FMT[F_COUNT] = {
    { 1, 0, 0, 0, 0, 0x0C, 0x00 },                                       /* FMT_P8, RAW */
    { 2, 0xF800, 0x07E0, 0x001F, 0, 0x04, 0x00 },                        /* FMT_RGB565, RAW */
    { 2, 0x7C00, 0x03E0, 0x001F, 0, 0x04, 0x00 },                        /* FMT_RGB565, RAW: no 555 storage format, so the PC widens green (ddraw_main.c, w565) */
    { 3, 0xFF0000, 0x00FF00, 0x0000FF, 0, 0x00, 0x0B },                  /* TF_BGR888 */
    { 4, 0xFF0000, 0x00FF00, 0x0000FF, 0, 0x00, 0x03 },                  /* TF_BGRX8888 */
    { 4, 0xFF0000, 0x00FF00, 0x0000FF, 0xFF000000u, 0x00, 0x02 },        /* TF_BGRA8888 */
};

static inline uint32_t bx_read(const uint8_t *p, int bytes)
{
    switch (bytes) {
    case 1: return p[0];
    case 2: return *(const uint16_t *)p;
    case 3: return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16;
    default: return *(const uint32_t *)p;
    }
}

static inline void bx_write(uint8_t *p, int bytes, uint32_t v)
{
    switch (bytes) {
    case 1: p[0] = (uint8_t)v; break;
    case 2: *(uint16_t *)p = (uint16_t)v; break;
    case 3: p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); break;
    default: *(uint32_t *)p = v; break;
    }
}

/* c bits -> 8 bits, exact: round(c * 255 / (2^n - 1)) (fbproto._e) */
static inline uint32_t exp8(uint32_t c, int n)
{
    uint32_t m = (1u << n) - 1;
    return (c * 510 + m) / (2 * m);
}

/* 8 bits -> n bits: round(v * (2^n - 1) / 255) (fbproto._r) */
static inline uint32_t rnd8(uint32_t v, int n) { return (v * ((1u << n) - 1) * 2 + 255) / 510; }

/* one pixel to 0xAARRGGBB; P8 goes through a palette of 0x00RRGGBB */
static inline uint32_t px_to_argb(int fmt, uint32_t v, const uint32_t *pal)
{
    switch (fmt) {
    case F_P8: return 0xFF000000u | (pal ? pal[v & 255] : (v & 255) * 0x010101u);
    case F_565: return 0xFF000000u | exp8(v >> 11, 5) << 16 | exp8((v >> 5) & 63, 6) << 8 | exp8(v & 31, 5);
    case F_555: return 0xFF000000u | exp8((v >> 10) & 31, 5) << 16 | exp8((v >> 5) & 31, 5) << 8 | exp8(v & 31, 5);
    case F_888: case F_X888: return 0xFF000000u | (v & 0xFFFFFF);
    default: return v;
    }
}

static inline uint32_t px_from_argb(int fmt, uint32_t c)
{
    uint32_t r = (c >> 16) & 255, g = (c >> 8) & 255, b = c & 255;
    switch (fmt) {
    case F_P8: return (r * 77 + g * 150 + b * 29) >> 8;          /* no palette search: real blits to P8 are P8 -> P8 */
    case F_565: return rnd8(r, 5) << 11 | rnd8(g, 6) << 5 | rnd8(b, 5);
    case F_555: return rnd8(r, 5) << 10 | rnd8(g, 5) << 5 | rnd8(b, 5);
    case F_888: case F_X888: return c & 0xFFFFFF;
    default: return c;
    }
}

typedef struct bview { uint8_t *pix; int32_t pitch; int fmt, w, h; } bview;

typedef struct blit_op {
    int sx, sy, sw, sh;         /* source rectangle */
    int dx, dy, dw, dh;         /* destination rectangle */
    int cx0, cy0, cx1, cy1;     /* destination clip */
    int mirror_x, mirror_y;
    int use_src_key, use_dst_key;
    uint32_t sk_lo, sk_hi, dk_lo, dk_hi;
    const uint32_t *pal;        /* the source's palette (P8 -> RGB) */
} blit_op;

/* the colour key looks at the colour bits only */
static inline uint32_t key_mask(int fmt) { return fmt == F_A888 ? 0xFFFFFF : 0xFFFFFFFFu; }

static inline void cpu_blit(const bview *dst, const bview *src, const blit_op *o)
{
    const int db = FMT[dst->fmt].bytes, sb = FMT[src->fmt].bytes, conv = dst->fmt != src->fmt;
    const int stretch = o->dw != o->sw || o->dh != o->sh;
    const uint32_t smask = key_mask(src->fmt), dmask = key_mask(dst->fmt);
    uint8_t *tmp = NULL;
    const uint8_t *sp = src->pix + (ptrdiff_t)o->sy * src->pitch + (ptrdiff_t)o->sx * sb;
    int32_t spitch = src->pitch;
    int x, y;
    int y0 = o->dy > o->cy0 ? o->dy : o->cy0, y1 = o->dy + o->dh < o->cy1 ? o->dy + o->dh : o->cy1;
    int x0 = o->dx > o->cx0 ? o->dx : o->cx0, x1 = o->dx + o->dw < o->cx1 ? o->dx + o->dw : o->cx1;
    if (x0 >= x1 || y0 >= y1) return;
    if (dst->pix == src->pix) {                                 /* overlap: read the source as it was */
        int r;
        tmp = (uint8_t *)malloc((size_t)o->sw * o->sh * sb);
        if (!tmp) return;
        for (r = 0; r < o->sh; r++) memcpy(tmp + (size_t)r * o->sw * sb, sp + (ptrdiff_t)r * spitch, (size_t)o->sw * sb);
        sp = tmp;
        spitch = o->sw * sb;
    }
    if (!stretch && !conv && !o->mirror_x && !o->mirror_y && !o->use_src_key && !o->use_dst_key) {
        for (y = y0; y < y1; y++) {
            memcpy(dst->pix + (ptrdiff_t)y * dst->pitch + (ptrdiff_t)x0 * db,
                   sp + (ptrdiff_t)(y - o->dy) * spitch + (ptrdiff_t)(x0 - o->dx) * sb, (size_t)(x1 - x0) * db);
        }
    } else {
        for (y = y0; y < y1; y++) {
            int j = y - o->dy, v = (int)((2 * (int64_t)j + 1) * o->sh / (2 * (int64_t)o->dh));
            const uint8_t *srow;
            uint8_t *drow = dst->pix + (ptrdiff_t)y * dst->pitch;
            if (o->mirror_y) v = o->sh - 1 - v;
            srow = sp + (ptrdiff_t)v * spitch;
            for (x = x0; x < x1; x++) {
                int i = x - o->dx, u = (int)((2 * (int64_t)i + 1) * o->sw / (2 * (int64_t)o->dw));
                uint32_t s, d;
                if (o->mirror_x) u = o->sw - 1 - u;
                s = bx_read(srow + (ptrdiff_t)u * sb, sb);
                if (o->use_src_key) {
                    uint32_t k = s & smask;
                    if (k >= o->sk_lo && k <= o->sk_hi) continue;
                }
                if (o->use_dst_key) {
                    uint32_t k = bx_read(drow + (ptrdiff_t)x * db, db) & dmask;
                    if (k < o->dk_lo || k > o->dk_hi) continue;
                }
                d = conv ? px_from_argb(dst->fmt, px_to_argb(src->fmt, s, o->pal)) : s;
                bx_write(drow + (ptrdiff_t)x * db, db, d);
            }
        }
    }
    free(tmp);
}

static inline void cpu_fill(const bview *dst, int x0, int y0, int x1, int y1, uint32_t value)
{
    const int db = FMT[dst->fmt].bytes;
    int x, y;
    for (y = y0; y < y1; y++) {
        uint8_t *row = dst->pix + (ptrdiff_t)y * dst->pitch + (ptrdiff_t)x0 * db;
        switch (db) {
        case 1: memset(row, (int)value, (size_t)(x1 - x0)); break;
        case 2: for (x = x0; x < x1; x++) ((uint16_t *)row)[x - x0] = (uint16_t)value; break;
        case 4: for (x = x0; x < x1; x++) ((uint32_t *)row)[x - x0] = value; break;
        default: for (x = x0; x < x1; x++) bx_write(row + (ptrdiff_t)(x - x0) * 3, 3, value); break;
        }
    }
}
#endif
