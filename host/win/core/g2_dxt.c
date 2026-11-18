/* DXT block decoding: see g2_dxt.h. Colours expand 5 / 6 bits by rounding, the interpolants are the (2a + b) / 3 and (a + b) / 2 of the S3TC specification. */
#include <string.h>
#include "g2_dxt.h"
#include "gpu2core.h"

static void rgb565(uint32_t c, uint8_t *o)
{
    o[0] = (uint8_t)(((c >> 11) * 510 + 31) / 62);
    o[1] = (uint8_t)((((c >> 5) & 63) * 510 + 63) / 126);
    o[2] = (uint8_t)(((c & 31) * 510 + 31) / 62);
    o[3] = 255;
}

/* the block's 16 colours (RGBA); four_colors: the DXT3 / DXT5 rule, where color0 <= color1 does not select the 3-colour mode */
static void color_block(const uint8_t *b, int four_colors, int alpha_black, uint8_t out[16][4])
{
    uint8_t c[4][4];
    uint32_t c0 = b[0] | (uint32_t)b[1] << 8, c1 = b[2] | (uint32_t)b[3] << 8, bits = b[4] | (uint32_t)b[5] << 8 | (uint32_t)b[6] << 16 | (uint32_t)b[7] << 24;
    int i, k;
    rgb565(c0, c[0]);
    rgb565(c1, c[1]);
    if (four_colors || c0 > c1) {
        for (k = 0; k < 3; k++) {
            c[2][k] = (uint8_t)((2 * c[0][k] + c[1][k]) / 3);
            c[3][k] = (uint8_t)((c[0][k] + 2 * c[1][k]) / 3);
        }
        c[2][3] = c[3][3] = 255;
    } else {
        for (k = 0; k < 3; k++) { c[2][k] = (uint8_t)((c[0][k] + c[1][k]) / 2); c[3][k] = 0; }
        c[2][3] = 255;
        c[3][3] = alpha_black ? 0 : 255;
    }
    for (i = 0; i < 16; i++) memcpy(out[i], c[(bits >> (2 * i)) & 3], 4);
}

static void alpha_block5(const uint8_t *b, uint8_t a[16])
{
    uint32_t a0 = b[0], a1 = b[1], v[8];
    uint64_t bits = 0;
    int i;
    for (i = 0; i < 6; i++) bits |= (uint64_t)b[2 + i] << (8 * i);
    v[0] = a0; v[1] = a1;
    if (a0 > a1) {
        for (i = 1; i <= 6; i++) v[1 + i] = ((7 - (uint32_t)i) * a0 + (uint32_t)i * a1) / 7;
    } else {
        for (i = 1; i <= 4; i++) v[1 + i] = ((5 - (uint32_t)i) * a0 + (uint32_t)i * a1) / 5;
        v[6] = 0; v[7] = 255;
    }
    for (i = 0; i < 16; i++) a[i] = (uint8_t)v[(bits >> (3 * i)) & 7];
}

uint32_t g2_dxt_bytes(int kind, int w, int h)
{
    uint32_t bw = (uint32_t)(w + 3) / 4, bh = (uint32_t)(h + 3) / 4;
    return bw * bh * (kind == G2_DXT1 || kind == G2_DXT1A ? 8u : 16u);
}

void g2_dxt_decode(int kind, const uint8_t *src, int w, int h, uint8_t *rgba)
{
    int bx, by, x, y, step = kind == G2_DXT1 || kind == G2_DXT1A ? 8 : 16;
    for (by = 0; by < (h + 3) / 4; by++) {
        for (bx = 0; bx < (w + 3) / 4; bx++) {
            uint8_t px[16][4], al[16];
            const uint8_t *b = src;
            src += step;
            if (kind == G2_DXT1 || kind == G2_DXT1A) {
                color_block(b, 0, kind == G2_DXT1A, px);
            } else {
                int i;
                color_block(b + 8, 1, 0, px);
                if (kind == G2_DXT3) {
                    for (i = 0; i < 16; i++) { uint32_t n = (b[i / 2] >> (4 * (i & 1))) & 15; px[i][3] = (uint8_t)(n * 17); }
                } else {
                    alpha_block5(b, al);
                    for (i = 0; i < 16; i++) px[i][3] = al[i];
                }
            }
            for (y = 0; y < 4 && by * 4 + y < h; y++) {
                for (x = 0; x < 4 && bx * 4 + x < w; x++) {
                    memcpy(rgba + (((size_t)(by * 4 + y)) * w + (size_t)(bx * 4 + x)) * 4, px[y * 4 + x], 4);
                }
            }
        }
    }
}

/* ---------------------------------------------------------------- ETC1 (OES_compressed_ETC1_RGB8_texture): 64 bits a 4 x 4 block, big-endian, a base colour and a table for each half of the block. A texel's two bits, spread over
   two 16-bit words, pick one of its table's four offsets, added to all three channels and clamped to 0 .. 255 */
static const int ETC1_MOD[8][4] = { { 2, 8, -2, -8 }, { 5, 17, -5, -17 }, { 9, 29, -9, -29 }, { 13, 42, -13, -42 }, { 18, 60, -18, -60 }, { 24, 80, -24, -80 }, { 33, 106, -33, -106 }, { 47, 183, -47, -183 } };

static int etc1_clamp(int v) { return v < 0 ? 0 : v > 255 ? 255 : v; }
static int etc1_5to8(int v) { return (v << 3 | v >> 2) & 255; }
static int etc1_delta(int v) { return v >= 4 ? v - 8 : v; }     /* 3 bits, two's complement */

uint32_t g2_etc1_bytes(int w, int h) { return (uint32_t)((w + 3) / 4) * (uint32_t)((h + 3) / 4) * 8; }

void g2_etc1_decode(const uint8_t *src, int w, int h, uint8_t *rgba)
{
    int bx, by, x, y;
    for (by = 0; by < h; by += 4) {
        for (bx = 0; bx < w; bx += 4, src += 8) {
            int base[2][3], tab[2], c, diff = src[3] >> 1 & 1, flip = src[3] & 1;
            unsigned msb = (unsigned)src[4] << 8 | src[5], lsb = (unsigned)src[6] << 8 | src[7];
            tab[0] = src[3] >> 5; tab[1] = src[3] >> 2 & 7;
            for (c = 0; c < 3; c++) {
                if (diff) {             /* a base + delta outside 0 .. 31 is no valid block; its 8 bits extend as the reference decoder's do */
                    int a = src[c] >> 3;
                    base[0][c] = etc1_5to8(a); base[1][c] = etc1_5to8((a + etc1_delta(src[c] & 7)) & 255);
                } else {
                    base[0][c] = (src[c] >> 4) * 17; base[1][c] = (src[c] & 15) * 17;
                }
            }
            for (y = 0; y < 4 && by + y < h; y++) {
                for (x = 0; x < 4 && bx + x < w; x++) {
                    int k = x * 4 + y, half = flip ? y >= 2 : x >= 2, m = ETC1_MOD[tab[half]][(lsb >> k & 1) | (msb >> k & 1) << 1];
                    uint8_t *o = rgba + ((size_t)(by + y) * w + bx + x) * 4;
                    o[0] = (uint8_t)etc1_clamp(base[half][0] + m); o[1] = (uint8_t)etc1_clamp(base[half][1] + m); o[2] = (uint8_t)etc1_clamp(base[half][2] + m); o[3] = 255;
                }
            }
        }
    }
}

/* ---------------------------------------------------------------- 16-bit storage */
static const struct { uint8_t sh[4], bits[4]; } PX16[3] = { { { 11, 5, 0, 0 }, { 5, 6, 5, 0 } }, { { 12, 8, 4, 0 }, { 4, 4, 4, 4 } }, { { 11, 6, 1, 0 }, { 5, 5, 5, 1 } } };

static int px16_of(uint32_t fmt) { return fmt == G2_FMT_RGB565 ? 0 : fmt == G2_FMT_RGBA4444 ? 1 : 2; }

void g2_px16_unpack(uint32_t fmt, const uint16_t *in, size_t n, uint8_t *rgba)
{
    int f = px16_of(fmt), c;
    size_t i;
    for (i = 0; i < n; i++) {
        for (c = 0; c < 4; c++) {
            uint32_t m = (1u << PX16[f].bits[c]) - 1;
            rgba[i * 4 + c] = m ? (uint8_t)(((in[i] >> PX16[f].sh[c] & m) * 510 + m) / (2 * m)) : 255;
        }
    }
}

void g2_px16_pack(uint32_t fmt, const uint8_t *rgba, size_t n, uint16_t *out)
{
    int f = px16_of(fmt), c;
    size_t i;
    for (i = 0; i < n; i++) {
        uint32_t v = 0;
        for (c = 0; c < 4; c++) {
            uint32_t m = (1u << PX16[f].bits[c]) - 1;
            v |= (((uint32_t)rgba[i * 4 + c] * 2 * m + 255) / 510) << PX16[f].sh[c];
        }
        out[i] = (uint16_t)v;
    }
}
