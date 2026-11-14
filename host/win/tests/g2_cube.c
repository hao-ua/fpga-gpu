/* g2_cube — fb_cube.py through gpu2core on the boards: same scene, same --check, same report.
 *
 *   g2_cube [--ip A] [--mode M] [--frames N | --secs S] [--speed DEG] [--cubes N] [--draws] [--nohold] [--scale S] [--check]
 */
#include <stdio.h>
#include <windows.h>
#include "g2_cube_scene.h"

#define RENDER_TIMEOUT_CYCLES 300000000u

static double now_s(void)
{
    LARGE_INTEGER t, f;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&t);
    return (double)t.QuadPart / (double)f.QuadPart;
}

/* ---------------------------------------------------------------- --check (fb_cube.check_frame) */
typedef struct front_t { int k; const face_t *face; double win[4][3], area; uint32_t col; } front_t;

static void face_colour(const mat4 *R, const face_t *f, double out[3])
{
    double n4[4] = { f->n[0], f->n[1], f->n[2], 0 }, n[4], l[3], d;
    int c;
    mat_vec(R, n4, n);
    normalize3(LIGHT, l);
    d = n[0] * l[0] + n[1] * l[1] + n[2] * l[2];
    if (d < 0.0) d = 0.0;
    for (c = 0; c < 3; c++) out[c] = f->col[c] * (AMBIENT + (1 - AMBIENT) * d);
}

static uint32_t rgba8(const double c[3])
{
    uint32_t r = g2_c16(c[0], 8) >> 4, g = g2_c16(c[1], 8) >> 4, b = g2_c16(c[2], 8) >> 4, a = g2_c16(1.0, 8) >> 4;
    return r << 24 | g << 16 | b << 8 | a;
}

static int tri_z(const double *p0, const double *p1, const double *p2, double x, double y, double *z)
{
    double d = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]), l1, l2, l0, m;
    if (d == 0) return 0;
    l1 = ((x - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (y - p0[1])) / d;
    l2 = ((p1[0] - p0[0]) * (y - p0[1]) - (x - p0[0]) * (p1[1] - p0[1])) / d;
    l0 = 1 - l1 - l2;
    m = l0 < l1 ? l0 : l1;
    if (l2 < m) m = l2;
    if (m < -1e-9) return 0;
    *z = l0 * p0[2] + l1 * p1[2] + l2 * p2[2];
    return 1;
}

static int close8(uint32_t a, uint32_t b)
{
    int s;
    for (s = 0; s < 32; s += 8) {
        int d = (int)((a >> s) & 255) - (int)((b >> s) & 255);
        if (d < -2 || d > 2) return 0;
    }
    return 1;
}

static int check_frame(g2_board *bd, int i, uint32_t W, uint32_t H, const scene_t *s)
{
    double a = scene_angle(i, 1.0);
    uint8_t *data = (uint8_t *)malloc((size_t)W * H * 4);
    front_t *front = (front_t *)malloc((size_t)s->n * 6 * sizeof(front_t));
    int nf = 0, k, f, bad = 0, faces = 0, occluded = 0, j;
    double bg[3] = { CLEAR_RGBA[0], CLEAR_RGBA[1], CLEAR_RGBA[2] };
    uint32_t exp_bg;
    if (g2_mem_read(bd, B_IMG[i & 1], data, (size_t)W * H * 4, 1) < 0) { printf("check: read failed: %s\n", g2_error_text(bd)); return 0; }
#define PX(x, y) ((uint32_t)data[((size_t)(y) * W + (x)) * 4] | (uint32_t)data[((size_t)(y) * W + (x)) * 4 + 1] << 8 | \
                  (uint32_t)data[((size_t)(y) * W + (x)) * 4 + 2] << 16 | (uint32_t)data[((size_t)(y) * W + (x)) * 4 + 3] << 24)
    for (k = 0; k < s->n; k++) {
        mat4 R = scene_rotation(s, a, k), mvp = scene_mvp(s, &R, k);
        for (f = 0; f < 6; f++) {
            front_t *t = &front[nf];
            double area = 0;
            for (j = 0; j < 4; j++) {
                double p[4] = { FACES[f].corner[j][0], FACES[f].corner[j][1], FACES[f].corner[j][2], 1.0 }, c[4];
                mat_vec(&mvp, p, c);
                t->win[j][0] = (c[0] / c[3]) * W / 2 + (double)W / 2;
                t->win[j][1] = (c[1] / c[3]) * H / 2 + (double)H / 2;
                t->win[j][2] = c[2] / c[3];
            }
            for (j = 0; j < 4; j++) area += t->win[j][0] * t->win[(j + 1) % 4][1] - t->win[(j + 1) % 4][0] * t->win[j][1];
            area /= 2;
            if (area > 0) {
                double col[3];
                face_colour(&R, &FACES[f], col);
                t->k = k; t->face = &FACES[f]; t->area = area; t->col = rgba8(col);
                nf++;
            }
        }
    }
    for (f = 0; f < nf; f++) {
        front_t *t = &front[f];
        double cx = 0, cy = 0, pts[6][2];
        if (t->area <= 400) continue;
        faces++;
        for (j = 0; j < 4; j++) { cx += t->win[j][0] / 4; cy += t->win[j][1] / 4; }
        pts[0][0] = cx; pts[0][1] = cy;
        for (j = 0; j < 4; j++) { pts[1 + j][0] = (cx + t->win[j][0]) / 2; pts[1 + j][1] = (cy + t->win[j][1]) / 2; }
        pts[5][0] = (3 * cx + t->win[0][0]) / 4; pts[5][1] = (3 * cy + t->win[0][1]) / 4;
        for (j = 0; j < 6; j++) {
            int x = (int)pts[j][0], y = (int)pts[j][1], best = -1, q;
            double bz = 0, z;
            uint32_t got = PX(x, y), e;
            for (q = 0; q < nf; q++) {
                front_t *u = &front[q];
                if (!tri_z(u->win[0], u->win[1], u->win[2], x + 0.5, y + 0.5, &z) && !tri_z(u->win[0], u->win[2], u->win[3], x + 0.5, y + 0.5, &z)) continue;
                if (best < 0 || z < bz) { best = q; bz = z; }
            }
            e = best >= 0 ? front[best].col : t->col;
            if (best >= 0 && front[best].k != t->k) occluded++;
            if (!close8(got, e)) {
                if (++bad <= 10) printf("  check: cube %d pixel (%d, %d): %08x, expected %08x\n", t->k, x, y, got, e);
            }
        }
    }
    exp_bg = rgba8(bg);
    {
        uint32_t cx[4] = { 0, W - 1, 0, W - 1 }, cy[4] = { 0, 0, H - 1, H - 1 };
        for (j = 0; j < 4; j++) {
            if (PX(cx[j], cy[j]) != exp_bg) { bad++; printf("  check: corner (%u, %u): %08x, expected %08x\n", cx[j], cy[j], PX(cx[j], cy[j]), exp_bg); }
        }
    }
    printf("check: frame %d: %d cubes, %d front faces x 6 points (%d points behind another cube) + 4 corners, %d mismatches\n", i, s->n, faces, occluded, bad);
    free(data); free(front);
    return bad == 0;
}

int main(int argc, char **argv)
{
    const char *ip = "192.168.1.10";
    int mode = -1, frames = -1, ncubes = 32, instanced = 1, hold = 1, check = 0, i, c, np = 0, retrains = 0, ok = 1;
    double secs = -1, speed = 1.0, size = 1.0, t0, t_rep, el;
    g2_board *bd;
    uint32_t st[G2_ST_WORDS], st0[G2_ST_WORDS], W, H, nidx, shown = 0, dropped = 0, bm;
    g2_ref pending[2];
    g2_counters c0, c1;
    scene_t s;
    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--ip") && i + 1 < argc) ip = argv[++i];
        else if (!strcmp(argv[i], "--mode") && i + 1 < argc) mode = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--frames") && i + 1 < argc) frames = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--secs") && i + 1 < argc) secs = atof(argv[++i]);
        else if (!strcmp(argv[i], "--speed") && i + 1 < argc) speed = atof(argv[++i]);
        else if (!strcmp(argv[i], "--cubes") && i + 1 < argc) ncubes = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--scale") && i + 1 < argc) size = atof(argv[++i]);
        else if (!strcmp(argv[i], "--draws")) instanced = 0;
        else if (!strcmp(argv[i], "--nohold")) hold = 0;
        else if (!strcmp(argv[i], "--check")) check = 1;
        else { fprintf(stderr, "unknown argument %s\n", argv[i]); return 2; }
    }
    if (ncubes < 1 || ncubes > 2000) { fprintf(stderr, "--cubes 1..2000\n"); return 2; }
    if (mode >= G2_NMODES) { fprintf(stderr, "--mode 0..%d\n", G2_NMODES - 1); return 2; }
    bd = g2_open(ip, 5000, NULL);
    if (!bd) { fprintf(stderr, "cannot open the socket\n"); return 1; }
    if (g2_hello(bd, 1) < 0 || g2_check_boards(bd, 57) < 0) { fprintf(stderr, "boards: %s\n", g2_error_text(bd)); return 1; }
    if (g2_status(bd, st, NULL) < 0) return 1;
    bm = (uint32_t)g2_mode_of_flags(st[G2_ST_B_FLAGS]);
    if (mode >= 0) { if (g2_set_mode(bd, (uint32_t)mode, 5.0) < 0) { fprintf(stderr, "%s\n", g2_error_text(bd)); return 1; } }
    else mode = (int)bm;
    W = (uint32_t)G2_MODE_W[mode]; H = (uint32_t)G2_MODE_H[mode];
    scene_init(&s, ncubes, (double)W / (double)H, size);
    printf("board A build %u, B mode %d (%u x %u); %d cube(s), %s, PRESENT %s\n", st[G2_ST_BUILD], mode, W, H, ncubes,
           instanced ? "one draw (cubes.vert)" : "a draw per cube (cube.vert)", hold ? "hold" : "without hold");
    nidx = scene_setup(bd, W, H, &s, instanced);
    {
        g2_error e[8];
        int n = g2_take_errors(bd, e, 8), k;
        for (k = 0; k < n; k++) printf("  EV_ERROR seq %u: %s (type %02X, detail %08X)\n", e[k].seq, g2_error_name(e[k].code), e[k].type, e[k].detail);
        if (n) return 1;
    }
    g2_status(bd, st0, NULL);
    g2_get_counters(bd, &c0);
    t0 = t_rep = now_s();
    i = -1;
    while (1) {
        g2_ref r;
        g2_error e[8];
        int n, k;
        if (frames >= 0 && i + 1 >= frames) break;
        if (secs >= 0 && now_s() - t0 >= secs) break;
        i++;
        while (np >= 2) {
            uint32_t drp = 0;
            if (g2_present_wait(bd, pending[0], NULL, NULL, &drp, 5.0) < 0) printf("  frame: %s\n", g2_error_text(bd));
            else { shown++; dropped += drp; }
            pending[0] = pending[1]; np--;
        }
        r = scene_frame(bd, i, W, H, &s, instanced, speed, nidx, hold);
        pending[np++] = r;
        if (i == 0 && check) {
            uint32_t drp = 0;
            if (g2_present_wait(bd, r, NULL, NULL, &drp, 10.0) == 0) { shown++; dropped += drp; }
            np = 0;
            ok = check_frame(bd, 0, W, H, &s);
        }
        n = g2_take_errors(bd, e, 8);
        for (k = 0; k < n; k++) {
            printf("  EV_ERROR seq %u: %s (type %02X, detail %08X)\n", e[k].seq, g2_error_name(e[k].code), e[k].type, e[k].detail);
            if (e[k].code == 9 && e[k].detail < RENDER_TIMEOUT_CYCLES) retrains++;
            else ok = 0;
        }
        if (!ok) break;
        if (now_s() - t_rep >= 2.0) {
            printf("%6.1f s  frame %6d  shown %6u\n", now_s() - t0, i, shown);
            fflush(stdout);
            t_rep = now_s();
        }
    }
    for (c = 0; c < np; c++) {
        uint32_t drp = 0;
        if (g2_present_wait(bd, pending[c], NULL, NULL, &drp, 5.0) == 0) { shown++; dropped += drp; }
    }
    g2_fence(bd, 30.0);
    el = now_s() - t0;
    g2_status(bd, st, NULL);
    g2_get_counters(bd, &c1);
    {
        uint32_t rs = st[G2_ST_RS_PKTS_INFLIGHT];
        int n = i + 1;
        printf("g2_cube: %d cube(s) %s: %d frames in %.1f s = %.1f fps (%.1f k triangles/s), shown %u, replaced before shown %u; B underruns %u, A errs %u, "
               "link retrains %d, render beats in flight %u; %s\n", ncubes, instanced ? "one draw" : "per-cube draws", n, el, el > 0 ? n / el : 0.0,
               (el > 0 ? n / el : 0.0) * 12 * ncubes / 1e3, shown, dropped, st[G2_ST_B_UNDERRUNS_MODERR] & 0xFFFF, st[G2_ST_ERRS] - st0[G2_ST_ERRS], retrains,
               rs >> 16, ok && (rs >> 16) == 0 ? "PASS" : "FAIL");
        printf("  host: %.1f datagrams per frame (%llu BATCH datagrams carrying %llu commands), resent %llu (heartbeat rule %llu), A gap %u dup %u\n",
               (double)(c1.dgrams - c0.dgrams) / n, (unsigned long long)(c1.n_batches - c0.n_batches), (unsigned long long)(c1.n_batched - c0.n_batched),
               (unsigned long long)(c1.n_resent - c0.n_resent), (unsigned long long)(c1.n_hb_resend - c0.n_hb_resend), st[G2_ST_GAP] - st0[G2_ST_GAP],
               st[G2_ST_DUP] - st0[G2_ST_DUP]);
        ok = ok && (rs >> 16) == 0;
    }
    scene_free(&s);
    g2_close(bd);
    return ok ? 0 : 1;
}
