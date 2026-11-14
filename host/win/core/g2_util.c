/* gpu2core: numbers (FP24, channel units), image layout, heaps, descriptor handles */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "gpu2core.h"

/* fbproto.fp24: float -> FP24 bits (s1 e7 m16, bias 63, round to nearest even, flush to zero) */
uint32_t g2_fp24(double x)
{
    uint32_t s;
    int e;
    double m, q;
    if (x == 0.0) return 0;
    if (isinf(x)) return (0x7Fu << 16) | (x < 0 ? 0x800000u : 0);
    if (isnan(x)) return (0x7Fu << 16) | 0x8000u;
    s = x < 0 ? 0x800000u : 0;
    m = frexp(fabs(x), &e);                                     /* abs(x) = m * 2^e, m in [0.5, 1) */
    e -= 1; m *= 2;                                             /* m in [1, 2) */
    q = rint(m * 65536.0);                                      /* round half to even (the default rounding mode) */
    if (q == 131072.0) { q = 65536.0; e += 1; }
    if (e + 63 <= 0) return s;                                  /* FTZ */
    if (e + 63 >= 0x7F) return s | (0x7Fu << 16);
    return s | ((uint32_t)(e + 63) << 16) | ((uint32_t)q & 0xFFFFu);
}

double g2_fp24_to_double(uint32_t bits)
{
    double s = (bits >> 23) & 1 ? -1.0 : 1.0;
    uint32_t e = (bits >> 16) & 0x7F, m = bits & 0xFFFF;
    if (e == 0x7F) return m == 0 ? s * INFINITY : NAN;
    if (e == 0) return 0.0 * s;
    return s * (1.0 + m / 65536.0) * ldexp(1.0, (int)e - 63);
}

uint32_t g2_c16(double f, int bits)
{
    double c = f < 0.0 ? 0.0 : f > 1.0 ? 1.0 : f;
    return (uint32_t)rint(c * 16 * (double)((1u << bits) - 1));
}

uint32_t g2_mip_chain_bytes(uint32_t width, uint32_t height, uint32_t bpp, uint32_t levels)
{
    uint32_t chain = 0, lv;
    for (lv = 0; lv < levels; lv++) {
        uint32_t w = width >> lv ? width >> lv : 1, h = height >> lv ? height >> lv : 1;
        chain += (w * h * bpp + 3) & ~3u;
    }
    return chain;
}

/* img_geom.sv: a level's slices follow each other, a level ends on 4 bytes */
uint32_t g2_vol_chain_bytes(uint32_t width, uint32_t height, uint32_t depth, uint32_t bpp, uint32_t levels)
{
    uint32_t chain = 0, lv;
    for (lv = 0; lv < levels; lv++) {
        uint32_t w = width >> lv ? width >> lv : 1, h = height >> lv ? height >> lv : 1, d = depth >> lv ? depth >> lv : 1;
        chain += (w * h * d * bpp + 3) & ~3u;
    }
    return chain;
}

uint32_t g2_image_bytes(uint32_t width, uint32_t height, uint32_t bpp, uint32_t levels, uint32_t layers)
{
    uint32_t stride = (g2_mip_chain_bytes(width, height, bpp, levels) + 31) & ~31u;
    return stride * layers;
}

/* ---------------------------------------------------------------- heaps: address-ordered blocks, first fit, merge on free */
typedef struct block { uint32_t addr, size; int used; } block;
struct g2_heap { block *b; int n, cap; uint32_t free_bytes; };

g2_heap *g2_heap_create(uint32_t base, uint32_t end)
{
    g2_heap *h = (g2_heap *)calloc(1, sizeof *h);
    if (!h) return NULL;
    h->cap = 64;
    h->b = (block *)malloc((size_t)h->cap * sizeof(block));
    h->b[0].addr = base; h->b[0].size = end - base; h->b[0].used = 0;
    h->n = 1;
    h->free_bytes = end - base;
    return h;
}

void g2_heap_destroy(g2_heap *h)
{
    if (!h) return;
    free(h->b);
    free(h);
}

static void insert_block(g2_heap *h, int at, uint32_t addr, uint32_t size, int used)
{
    if (h->n == h->cap) {
        h->cap *= 2;
        h->b = (block *)realloc(h->b, (size_t)h->cap * sizeof(block));
    }
    memmove(&h->b[at + 1], &h->b[at], (size_t)(h->n - at) * sizeof(block));
    h->b[at].addr = addr; h->b[at].size = size; h->b[at].used = used;
    h->n++;
}

uint32_t g2_heap_alloc(g2_heap *h, uint32_t size, uint32_t align)
{
    int i;
    if (align < 32) align = 32;
    size = (size + 31) & ~31u;
    if (size == 0) size = 32;
    for (i = 0; i < h->n; i++) {
        block *c = &h->b[i];
        uint32_t a, pad;
        if (c->used) continue;
        a = (c->addr + align - 1) & ~(align - 1);
        pad = a - c->addr;
        if (c->size < pad || c->size - pad < size) continue;
        if (pad) {                                              /* the padding stays a free block in front */
            insert_block(h, i, c->addr, pad, 0);
            i++;
            c = &h->b[i];
            c->addr = a; c->size -= pad;
        }
        if (c->size > size) insert_block(h, i + 1, a + size, c->size - size, 0);
        c = &h->b[i];
        c->size = size; c->used = 1;
        h->free_bytes -= size;
        return a;
    }
    return 0;
}

void g2_heap_free(g2_heap *h, uint32_t addr)
{
    int lo = 0, hi = h->n - 1;
    while (lo <= hi) {
        int m = (lo + hi) / 2;
        if (h->b[m].addr == addr) {
            if (!h->b[m].used) return;
            h->b[m].used = 0;
            h->free_bytes += h->b[m].size;
            if (m + 1 < h->n && !h->b[m + 1].used) {
                h->b[m].size += h->b[m + 1].size;
                memmove(&h->b[m + 1], &h->b[m + 2], (size_t)(h->n - m - 2) * sizeof(block));
                h->n--;
            }
            if (m > 0 && !h->b[m - 1].used) {
                h->b[m - 1].size += h->b[m].size;
                memmove(&h->b[m], &h->b[m + 1], (size_t)(h->n - m - 1) * sizeof(block));
                h->n--;
            }
            return;
        }
        if (h->b[m].addr < addr) lo = m + 1; else hi = m - 1;
    }
}

uint32_t g2_heap_free_bytes(const g2_heap *h) { return h->free_bytes; }

/* ---------------------------------------------------------------- handles: one bitmap per table, indices 16 .. limit - 1 */
int g2_handle_alloc(g2_handles *t, uint16_t kind, uint16_t *out)
{
    int n = t->limit > 0 && t->limit <= G2_HANDLES_MAX ? t->limit : 1024, i;
    for (i = 16; i < n; i++) {
        if (t->used[i >> 5] == 0xFFFFFFFFu) { i |= 31; continue; }    /* a full word */
        if (!(t->used[i >> 5] & (1u << (i & 31)))) {
            t->used[i >> 5] |= 1u << (i & 31);
            *out = (uint16_t)(kind | i);
            return 0;
        }
    }
    return -1;
}

void g2_handle_free(g2_handles *t, uint16_t handle)
{
    int i = handle & (G2_HANDLES_MAX - 1);
    t->used[i >> 5] &= ~(1u << (i & 31));
}
