/* the fb_cube.py scene in C: maths, buffers, setup and one frame through gpu2core, the same doubles in the same order, so a
 * recording is byte-identical to Python's (g2_parity) and the hardware picture is the same (g2_cube) */
#ifndef G2_CUBE_SCENE_H
#define G2_CUBE_SCENE_H
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "gpu2core.h"
#include "gen/cube_programs.h"

#define H_VBO 0x0021
#define H_IBO 0x0022
#define H_VSPROG 0x0023
#define A_VBO 0x1A000000u
#define A_IBO 0x1A400000u
#define A_VSPROG 0x1A500000u
#define H_FSPROG 0x8021                 /* at the start of B's heap (g2_heaps), as fb_cube's b_fsprog */
#define DRAW_ID 1
static const uint16_t H_IMG[2] = { 0xC030, 0xC031 };
static const uint32_t B_IMG[2] = { 0x30000000u, 0x30400000u };

#define FOV_DEG 45.0
#define NEAR_Z 1.0
#define FAR_Z 40.0
#define CAM_Z 4.0
#define START_DEG 35.0
#define X_RATE 0.7
#define PHASE_STEP 0.37
#define AMBIENT 0.25
static const double LIGHT[3] = { 0.4, 0.6, 0.7 };
static const double CLEAR_RGBA[4] = { 0.05, 0.05, 0.12, 1.0 };

typedef struct face_t { int n[3]; double col[3]; int corner[4][3]; } face_t;
static const face_t FACES[6] = {
    { { 0, 0, 1 },  { 0.25, 0.45, 1.0 }, { { -1, -1, 1 }, { 1, -1, 1 }, { 1, 1, 1 }, { -1, 1, 1 } } },
    { { 0, 0, -1 }, { 1.0, 0.9, 0.2 },   { { 1, -1, -1 }, { -1, -1, -1 }, { -1, 1, -1 }, { 1, 1, -1 } } },
    { { 1, 0, 0 },  { 1.0, 0.25, 0.25 }, { { 1, -1, 1 }, { 1, -1, -1 }, { 1, 1, -1 }, { 1, 1, 1 } } },
    { { -1, 0, 0 }, { 0.2, 0.9, 0.9 },   { { -1, -1, -1 }, { -1, -1, 1 }, { -1, 1, 1 }, { -1, 1, -1 } } },
    { { 0, 1, 0 },  { 0.3, 0.9, 0.3 },   { { -1, 1, 1 }, { 1, 1, 1 }, { 1, 1, -1 }, { -1, 1, -1 } } },
    { { 0, -1, 0 }, { 0.9, 0.3, 0.9 },   { { -1, -1, -1 }, { 1, -1, -1 }, { 1, -1, 1 }, { -1, -1, 1 } } },
};

typedef struct mat4 { double m[4][4]; } mat4;

static mat4 mat_mul(const mat4 *a, const mat4 *b)
{
    mat4 r;
    int i, j, k;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            double s = 0.0;
            for (k = 0; k < 4; k++) s += a->m[i][k] * b->m[k][j];
            r.m[i][j] = s;
        }
    }
    return r;
}

static void mat_vec(const mat4 *m, const double v[4], double out[4])
{
    int i, k;
    for (i = 0; i < 4; i++) {
        double s = 0.0;
        for (k = 0; k < 4; k++) s += m->m[i][k] * v[k];
        out[i] = s;
    }
}

static double radians_(double d) { return d * (3.141592653589793 / 180.0); }

static mat4 rot_x(double a)
{
    double c = cos(a), s = sin(a);
    mat4 r = { { { 1, 0, 0, 0 }, { 0, c, -s, 0 }, { 0, s, c, 0 }, { 0, 0, 0, 1 } } };
    return r;
}

static mat4 rot_y(double a)
{
    double c = cos(a), s = sin(a);
    mat4 r = { { { c, 0, s, 0 }, { 0, 1, 0, 0 }, { -s, 0, c, 0 }, { 0, 0, 0, 1 } } };
    return r;
}

static mat4 perspective(double fov_deg, double aspect, double n, double f)
{
    double t = 1.0 / tan(radians_(fov_deg) / 2);
    mat4 r = { { { t / aspect, 0, 0, 0 }, { 0, t, 0, 0 }, { 0, 0, (f + n) / (n - f), 2 * f * n / (n - f) }, { 0, 0, -1, 0 } } };
    return r;
}

static void normalize3(const double v[3], double out[3])
{
    double l = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    out[0] = v[0] / l; out[1] = v[1] / l; out[2] = v[2] / l;
}

typedef struct scene_t {
    int n, g;
    double scale;
    double *centres, *phases;
    mat4 vp;
} scene_t;

static void scene_init(scene_t *s, int n, double aspect, double size)
{
    int k;
    mat4 p, V = { { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, -CAM_Z }, { 0, 0, 0, 1 } } };
    s->n = n;
    s->g = (int)ceil(sqrt((double)n));
    if (s->g < 1) s->g = 1;
    s->scale = size / s->g;
    s->centres = (double *)malloc((size_t)n * 3 * sizeof(double));
    s->phases = (double *)malloc((size_t)n * sizeof(double));
    for (k = 0; k < n; k++) {
        double half = (double)(s->g - 1) / 2;
        s->centres[3 * k] = ((double)(k % s->g) - half) * 2.6 * s->scale;
        s->centres[3 * k + 1] = (half - (double)(k / s->g)) * 2.6 * s->scale;
        s->centres[3 * k + 2] = 0.0;
        s->phases[k] = k * PHASE_STEP;
    }
    p = perspective(FOV_DEG, aspect, NEAR_Z, FAR_Z);
    s->vp = mat_mul(&p, &V);
}

static void scene_free(scene_t *s) { free(s->centres); free(s->phases); }

static double scene_angle(int frame, double speed_deg) { return radians_(START_DEG + frame * speed_deg); }

static mat4 scene_rotation(const scene_t *s, double a, int k)
{
    double t = a + s->phases[k];
    mat4 y = rot_y(t), x = rot_x(X_RATE * t);
    return mat_mul(&y, &x);
}

static mat4 scene_mvp(const scene_t *s, const mat4 *R, int k)
{
    mat4 m;
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) m.m[i][j] = R->m[i][j] * s->scale;
        m.m[i][3] = s->centres[3 * k + i];
    }
    m.m[3][0] = 0; m.m[3][1] = 0; m.m[3][2] = 0; m.m[3][3] = 1;
    return mat_mul(&s->vp, &m);
}

static void put_vec4(uint32_t *d, double a, double b, double c, double e)
{
    d[0] = g2_fp24(a); d[1] = g2_fp24(b); d[2] = g2_fp24(c); d[3] = g2_fp24(e);
}

/* -> the vec4 count; the vertex program's uniforms for angle a followed by the compiler's constants */
static int scene_vs_uniforms(const scene_t *s, int instanced, double a, int k, uint32_t *u)
{
    double l[3];
    int i, n = 0;
    normalize3(LIGHT, l);
    if (instanced) {
        for (i = 0; i < 4; i++) put_vec4(u + 4 * n++, s->vp.m[i][0], s->vp.m[i][1], s->vp.m[i][2], s->vp.m[i][3]);
        put_vec4(u + 4 * n++, l[0], l[1], l[2], AMBIENT);
        put_vec4(u + 4 * n++, a, s->scale, X_RATE, 0.0);
        for (i = 0; i < cubes_vs_nconst; i++) put_vec4(u + 4 * n++, cubes_vs_const[i][0], cubes_vs_const[i][1], cubes_vs_const[i][2], cubes_vs_const[i][3]);
    } else {
        mat4 R = scene_rotation(s, a, k), mvp = scene_mvp(s, &R, k);
        for (i = 0; i < 4; i++) put_vec4(u + 4 * n++, mvp.m[i][0], mvp.m[i][1], mvp.m[i][2], mvp.m[i][3]);
        for (i = 0; i < 3; i++) put_vec4(u + 4 * n++, R.m[i][0], R.m[i][1], R.m[i][2], 0.0);
        put_vec4(u + 4 * n++, l[0], l[1], l[2], AMBIENT);
        for (i = 0; i < cube_vs_nconst; i++) put_vec4(u + 4 * n++, cube_vs_const[i][0], cube_vs_const[i][1], cube_vs_const[i][2], cube_vs_const[i][3]);
    }
    return n;
}

static uint32_t scene_setup(g2_board *bd, uint32_t W, uint32_t H, const scene_t *s, int instanced)
{
    int ncubes = instanced ? s->n : 1, stride = instanced ? 13 : 9, nv = 0, ni = 0, k, f, i, c, nattr;
    uint32_t *vw = (uint32_t *)malloc((size_t)ncubes * 24 * (size_t)stride * 4);
    uint16_t *idx = (uint16_t *)malloc((size_t)ncubes * 36 * 2);
    const uint32_t off4[4] = { 0, 0, 0, 0 };
    uint32_t const1[4], h[23], uni[4 * 64], nuni;
    const uint64_t *vsw = instanced ? cubes_vs_words : cube_vs_words;
    uint32_t vsn = instanced ? cubes_vs_nwords : cube_vs_nwords;
    double vp[4];
    (void)off4;
    for (k = 0; k < ncubes; k++) {
        uint32_t inst[4] = { g2_fp24(s->centres[3 * k]), g2_fp24(s->centres[3 * k + 1]), g2_fp24(s->centres[3 * k + 2]), g2_fp24(s->phases[k]) };
        for (f = 0; f < 6; f++) {
            const face_t *fc = &FACES[f];
            int b = 24 * k + 4 * f;
            for (i = 0; i < 4; i++) {
                for (c = 0; c < 3; c++) vw[nv++] = g2_fp24((double)fc->corner[i][c]);
                for (c = 0; c < 3; c++) vw[nv++] = g2_fp24((double)fc->n[c]);
                for (c = 0; c < 3; c++) vw[nv++] = g2_fp24(fc->col[c]);
                if (instanced) for (c = 0; c < 4; c++) vw[nv++] = inst[c];
            }
            idx[ni++] = (uint16_t)b; idx[ni++] = (uint16_t)(b + 1); idx[ni++] = (uint16_t)(b + 2);
            idx[ni++] = (uint16_t)b; idx[ni++] = (uint16_t)(b + 2); idx[ni++] = (uint16_t)(b + 3);
        }
    }
    g2_create_buffer(bd, H_VBO, A_VBO, 4u << 20, 0);
    g2_write_buffer(bd, H_VBO, 0, vw, (size_t)nv * 4);
    g2_create_buffer(bd, H_IBO, A_IBO, 1u << 20, 0);
    g2_write_buffer(bd, H_IBO, 0, idx, (size_t)ni * 2);
    g2_create_buffer(bd, H_VSPROG, A_VSPROG, 4096, 0);
    g2_write_buffer(bd, H_VSPROG, 0, vsw, (size_t)vsn * 8);
    g2_vs_program(bd, H_VSPROG, 0, vsn);
    nattr = instanced ? 4 : 3;
    const1[0] = 0; const1[1] = 0; const1[2] = 0; const1[3] = g2_fp24(1.0);
    for (i = 0; i < 16; i++) {
        if (i < nattr) g2_vertex_attr(bd, i, 1, i == 3 ? 4 : 3, H_VBO, 4u * (uint32_t)stride, 12u * (uint32_t)i, const1, G2_AT_FP24, 0);
        else g2_vertex_attr(bd, i, 0, 4, 0, 0, 0, const1, G2_AT_FP24, 0);
    }
    vp[0] = W * 8.0; vp[1] = W * 8.0; vp[2] = H * 8.0; vp[3] = H * 8.0;
    g2_vs_state(bd, vp, 1, 0, 0x01, W, H, DRAW_ID, 1, 0.0, 1.0, NULL);
    g2_create_buffer(bd, H_FSPROG, g2_heaps(bd).b, 4096, 0);
    g2_write_buffer(bd, H_FSPROG, 0, cube_fs_words, (size_t)cube_fs_nwords * 8);
    for (i = 0; i < 2; i++)
        g2_create_image(bd, H_IMG[i], B_IMG[i], W, H, G2_FMT_RGBA8, 0, 1, 1, 1, 0, G2_USE_RT | G2_USE_SCANOUT | G2_USE_TRANSFER);
    memset(h, 0, sizeof h);
    h[0] = 1u | 1u << 1 | (uint32_t)G2_FN_LESS << 2 | 0xFu << 16;
    nuni = cube_fs_const_base + cube_fs_nconst;
    h[11] = (uint32_t)cube_fs_nwords << 16 | H_FSPROG; h[12] = 0; h[13] = nuni;
    memset(uni, 0, sizeof uni);
    for (i = 0; i < cube_fs_nconst; i++)
        put_vec4(uni + 4 * (cube_fs_const_base + i), cube_fs_const[i][0], cube_fs_const[i][1], cube_fs_const[i][2], cube_fs_const[i][3]);
    g2_draw_state(bd, DRAW_ID, h, uni, nuni);
    g2_fence(bd, 10.0);
    free(vw); free(idx);
    return (uint32_t)ni;
}

static g2_ref scene_frame(g2_board *bd, int i, uint32_t W, uint32_t H, const scene_t *s, int instanced, double speed, uint32_t nidx, int hold)
{
    double a = scene_angle(i, speed);
    uint32_t u[4 * 256], cc[4], sc[4];
    int k, c;
    g2_batch_begin(bd);
    g2_frame_begin(bd);
    if (instanced) {
        int n = scene_vs_uniforms(s, 1, a, 0, u);
        g2_vs_uniforms(bd, 0, u, (uint32_t)n);
        g2_draw(bd, 0, nidx, 0, 1, H_IBO, 0, 0);
    } else {
        for (k = 0; k < s->n; k++) {
            int n = scene_vs_uniforms(s, 0, a, k, u);
            g2_vs_uniforms(bd, 0, u, (uint32_t)n);
            g2_draw(bd, 0, 36, 0, 1, H_IBO, 0, 0);
        }
    }
    for (c = 0; c < 4; c++) cc[c] = g2_c16(CLEAR_RGBA[c], 8);
    sc[0] = 0; sc[1] = 0; sc[2] = W; sc[3] = H;
    g2_flush(bd, H_IMG[i & 1], G2_FL_CLEAR_COLOR | G2_FL_CMASK(0xF) | G2_FL_CLEAR_DEPTH | G2_FL_CLEAR_STENCIL, cc, 0xFFFFFF, 0, sc, 0, 0xFF);
    g2_batch_end(bd);
    return g2_present(bd, H_IMG[i & 1], (uint32_t)i, 0, hold);
}

#endif
