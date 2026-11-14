/* g2_soak — WRITE_IMAGE at full rate from a native
 * Windows process, A's gap / dup deltas = the host -> A datagram loss with WSL's NAT out of the path; then a read-back of the last
 * frame against what was sent.
 *   g2_soak [--ip A] [--secs S] [--w W --h H] [--fmt rgb565|bgrx]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include "gpu2core.h"

#define H_IMG 0xC060
#define B_BASE 0x34000000u

static double now_s(void)
{
    LARGE_INTEGER t, f;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&t);
    return (double)t.QuadPart / (double)f.QuadPart;
}

int main(int argc, char **argv)
{
    const char *ip = "192.168.1.10";
    double secs = 10.0, t0, el;
    uint32_t W = 1280, H = 720, bpp = 4, tf = G2_TF_BGRX8888, fmt = G2_FMT_RGBA8, st0[G2_ST_WORDS], st1[G2_ST_WORDS], x, y, frames = 0;
    uint8_t *pix, *back;
    size_t bytes;
    g2_board *bd;
    g2_counters c0, c1;
    int i, bad = 0, trace = 0;
    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--ip") && i + 1 < argc) ip = argv[++i];
        else if (!strcmp(argv[i], "--trace")) trace = 1;
        else if (!strcmp(argv[i], "--secs") && i + 1 < argc) secs = atof(argv[++i]);
        else if (!strcmp(argv[i], "--w") && i + 1 < argc) W = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--h") && i + 1 < argc) H = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--fmt") && i + 1 < argc) {
            i++;
            if (!strcmp(argv[i], "rgb565")) { bpp = 2; tf = G2_TF_RGB565; fmt = G2_FMT_RGB565; }
        }
    }
    bytes = (size_t)W * H * bpp;
    pix = (uint8_t *)malloc(bytes);
    back = (uint8_t *)malloc(bytes);
    bd = g2_open(ip, 5000, NULL);
    if (!bd || g2_hello(bd, 1) < 0 || g2_check_boards(bd, 57) < 0) { fprintf(stderr, "boards: %s\n", bd ? g2_error_text(bd) : "no socket"); return 1; }
    g2_create_image(bd, H_IMG, B_BASE, W, H, fmt, (W * bpp + 31) & ~31u, 0, 1, 1, 0, G2_USE_SCANOUT | G2_USE_TRANSFER);
    g2_fence(bd, 10.0);
    g2_status(bd, st0, NULL);
    g2_get_counters(bd, &c0);
    t0 = now_s();
    while (now_s() - t0 < secs) {
        static double t_tr;
        static uint32_t gap_tr, sec;
        if (trace && now_s() - t_tr >= 1.0) {                  /* when do the gaps happen? */
            uint32_t st[G2_ST_WORDS];
            g2_status(bd, st, NULL);
            printf("  t %2u s: gaps +%u\n", sec++, st[G2_ST_GAP] - st0[G2_ST_GAP] - gap_tr);
            gap_tr = st[G2_ST_GAP] - st0[G2_ST_GAP];
            t_tr = now_s();
        }
        for (y = 0; y < H; y++) {
            uint8_t *row = pix + (size_t)y * W * bpp;
            for (x = 0; x < W; x++) {
                uint32_t v = (x + y + frames * 7) * 0x01030507u;
                memcpy(row + x * bpp, &v, bpp);
            }
        }
        if (g2_write_image(bd, H_IMG, pix, bytes, tf, 0, 0, W, H, 0, 0, 0, fmt) < 0) { fprintf(stderr, "write: %s\n", g2_error_text(bd)); return 1; }
        frames++;
    }
    if (g2_fence(bd, 30.0) < 0) { fprintf(stderr, "fence: %s\n", g2_error_text(bd)); return 1; }
    el = now_s() - t0;
    g2_status(bd, st1, NULL);
    g2_get_counters(bd, &c1);
    {
        uint64_t dg = c1.dgrams - c0.dgrams;
        uint32_t gap = st1[G2_ST_GAP] - st0[G2_ST_GAP], dup = st1[G2_ST_DUP] - st0[G2_ST_DUP];
        printf("g2_soak: %u frames %u x %u x %u in %.2f s: %.1f MB/s of pixels, %.1f k datagrams/s; resent %llu (heartbeat rule %llu); A gap %u dup %u full %u = %.4f %% gaps per datagram\n",
               frames, W, H, bpp, el, (double)frames * (double)bytes / el / 1e6, (double)dg / el / 1e3, (unsigned long long)(c1.n_resent - c0.n_resent),
               (unsigned long long)(c1.n_hb_resend - c0.n_hb_resend), gap, dup, st1[G2_ST_FULL] - st0[G2_ST_FULL], dg ? 100.0 * gap / (double)dg : 0.0);
    }
    /* the last frame back, as READ_IMAGE RAW (RGBA8: 0xRRGGBBAA little-endian) against what was sent */
    if (g2_read_image(bd, H_IMG, tf, 0, 0, W, H, 0, 0, fmt, 0, 0, back) < 0) { fprintf(stderr, "read: %s\n", g2_error_text(bd)); return 1; }
    for (y = 0; y < H; y++) {
        for (x = 0; x < W; x++) {
            size_t o = ((size_t)y * W + x) * bpp;
            if (tf == G2_TF_BGRX8888 ? memcmp(pix + o, back + o, 3) != 0 : memcmp(pix + o, back + o, bpp) != 0) {
                if (++bad <= 4) printf("  pixel (%u, %u) differs\n", x, y);
            }
        }
    }
    printf("read back %u x %u: %d pixels differ\n", W, H, bad);
    g2_close(bd);
    free(pix); free(back);
    return bad ? 1 : 0;
}
