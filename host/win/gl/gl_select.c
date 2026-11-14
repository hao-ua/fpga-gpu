/* selection and feedback (OpenGL 1.3 §5.2, §5.3): nothing reaches the board, so primitives are transformed, lit, clipped and culled here, the way the hardware's stages would have, and reported.
 * draw_list hands over triangles, lines and points after polygon mode: a quad reports as its two triangles, which §5.3 allows, as a polygon is whatever the implementation decomposed it to. */
#include "gl_internal.h"

/* a vertex and what clipping has to carry: clip position, eye position, front and back colour, texture coordinate, the distance to each user clip plane */
typedef struct fv { float clip[4], eye[4], col[4], colb[4], tc[4], cd[MAX_CLIP]; } fv;
#define FV_FLOATS (20 + MAX_CLIP)
#define MAX_CLIPPED (3 + 6 + MAX_CLIP)

static void xf(const float *m, const float *v, float *o)
{
    int r;
    for (r = 0; r < 4; r++) o[r] = m[r] * v[0] + m[r + 4] * v[1] + m[r + 8] * v[2] + m[r + 12] * v[3];
}

/* §2.13.1 for one colour, the same equations as gl_ff.c's gen_light */
static void lit(const float *eye, const float *n, const float *in, int back, float *out)
{
    const gl_material *m = &C->mat[back];
    int cm = C->color_material, i, c;
    const float *dif = cm && (C->cm_mode == GL_DIFFUSE || C->cm_mode == GL_AMBIENT_AND_DIFFUSE) ? in : m->diffuse;
    const float *amb = cm && (C->cm_mode == GL_AMBIENT || C->cm_mode == GL_AMBIENT_AND_DIFFUSE) ? in : m->ambient;
    const float *spc = cm && C->cm_mode == GL_SPECULAR ? in : m->specular;
    const float *emi = cm && C->cm_mode == GL_EMISSION ? in : m->emission;
    float r[3];
    for (c = 0; c < 3; c++) r[c] = emi[c] + amb[c] * C->lm_ambient[c];
    for (i = 0; i < MAX_LIGHTS; i++) {
        const gl_light *l = &C->light[i];
        float vp[3], att = 1.0f, nl, h[3], hn, s = 0.0f;
        if (!l->enabled) continue;
        if (l->position[3] != 0.0f) {
            float d2, d;
            for (c = 0; c < 3; c++) vp[c] = l->position[c] - eye[c];
            d2 = vp[0] * vp[0] + vp[1] * vp[1] + vp[2] * vp[2];
            d = sqrtf(d2);
            if (d > 0.0f) for (c = 0; c < 3; c++) vp[c] /= d;
            att = 1.0f / (l->atten[0] + l->atten[1] * d + l->atten[2] * d2);
            if (l->spot_cut != 180.0f) {
                float cs = -(vp[0] * l->spot_dir[0] + vp[1] * l->spot_dir[1] + vp[2] * l->spot_dir[2]);
                att = cs < cosf(l->spot_cut * 3.14159265358979f / 180.0f) ? 0.0f : att * powf(cs > 0.0f ? cs : 0.0f, l->spot_exp);
            }
        } else {
            float m2 = sqrtf(l->position[0] * l->position[0] + l->position[1] * l->position[1] + l->position[2] * l->position[2]);
            for (c = 0; c < 3; c++) vp[c] = m2 > 0.0f ? l->position[c] / m2 : 0.0f;
        }
        nl = n[0] * vp[0] + n[1] * vp[1] + n[2] * vp[2];
        if (nl < 0.0f) nl = 0.0f;
        if (nl > 0.0f) {
            if (C->lm_local_viewer) {
                float e = sqrtf(eye[0] * eye[0] + eye[1] * eye[1] + eye[2] * eye[2]);
                for (c = 0; c < 3; c++) h[c] = vp[c] - (e > 0.0f ? eye[c] / e : 0.0f);
            } else {
                h[0] = vp[0]; h[1] = vp[1]; h[2] = vp[2] + 1.0f;
            }
            hn = sqrtf(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
            if (hn > 0.0f) {
                float ns = (n[0] * h[0] + n[1] * h[1] + n[2] * h[2]) / hn;
                s = ns > 0.0f ? powf(ns, m->shininess) : 0.0f;
            }
        }
        for (c = 0; c < 3; c++) r[c] += att * (amb[c] * l->ambient[c] + nl * dif[c] * l->diffuse[c] + s * spc[c] * l->specular[c]);
    }
    for (c = 0; c < 3; c++) out[c] = r[c] < 0.0f ? 0.0f : r[c] > 1.0f ? 1.0f : r[c];
    out[3] = dif[3] < 0.0f ? 0.0f : dif[3] > 1.0f ? 1.0f : dif[3];
}

static void make_vertex(const src *in, uint32_t i, fv *o)
{
    const float *mv = C->mv[C->mv_sp], *pr = C->pr[C->pr_sp];
    const gl_unit *u0 = &C->unit[0];
    float p[4] = { 0, 0, 0, 1 }, col[4], tc[4];
    int c;
    if (gl_vs_outputs(in, i, o->clip, o->col, o->tc, o->cd)) {  /* what a user's vertex shader wrote; the planes by its gl_ClipVertex */
        memcpy(o->colb, o->col, 16);
        memcpy(o->eye, o->clip, 16);
        return;
    }
    gl_src_get(&in[A_POS], i, p);
    xf(mv, p, o->eye);
    xf(pr, o->eye, o->clip);
    memcpy(col, C->color, 16);
    if (in[A_COLOR].base) { col[3] = 1.0f; gl_src_get(&in[A_COLOR], i, col); }
    memcpy(o->colb, col, 16);
    if (C->lighting) {
        float n[3], t[3], inv[16], l;
        memcpy(n, C->normal, 12);
        if (in[A_NORMAL].base) gl_src_get(&in[A_NORMAL], i, n);
        memcpy(t, n, 12);
        if (gl_invert(inv, mv)) for (c = 0; c < 3; c++) t[c] = inv[c * 4] * n[0] + inv[c * 4 + 1] * n[1] + inv[c * 4 + 2] * n[2];
        l = sqrtf(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
        if (l > 0.0f && (C->normalize || C->rescale_normal)) for (c = 0; c < 3; c++) t[c] /= l;
        lit(o->eye, t, col, 0, o->col);
        if (C->lm_two_side) {
            float b[3];
            b[0] = -t[0]; b[1] = -t[1]; b[2] = -t[2];
            lit(o->eye, b, col, 1, o->colb);
        } else {
            memcpy(o->colb, o->col, 16);
        }
    } else {
        for (c = 0; c < 4; c++) o->col[c] = o->colb[c] = col[c] < 0.0f ? 0.0f : col[c] > 1.0f ? 1.0f : col[c];
    }
    memcpy(tc, u0->tc, 16);
    if (in[A_TC0].base) { tc[0] = tc[1] = tc[2] = 0.0f; tc[3] = 1.0f; gl_src_get(&in[A_TC0], i, tc); }
    for (c = 0; c < 4; c++) {
        if (!u0->gen_en[c]) continue;
        if (u0->gen_mode[c] == GL_OBJECT_LINEAR) tc[c] = u0->gen_obj[c][0] * p[0] + u0->gen_obj[c][1] * p[1] + u0->gen_obj[c][2] * p[2] + u0->gen_obj[c][3] * p[3];
        else if (u0->gen_mode[c] == GL_EYE_LINEAR) tc[c] = u0->gen_eye[c][0] * o->eye[0] + u0->gen_eye[c][1] * o->eye[1] + u0->gen_eye[c][2] * o->eye[2] + u0->gen_eye[c][3] * o->eye[3];
    }
    xf(u0->mat[u0->mat_sp], tc, o->tc);
    for (c = 0; c < MAX_CLIP; c++) { const float *n = C->clip_plane[c]; o->cd[c] = n[0] * o->eye[0] + n[1] * o->eye[1] + n[2] * o->eye[2] + n[3] * o->eye[3]; }
}

/* a plane of the view volume (0..5) or a user plane (6..): >= 0 inside */
static float plane_d(const fv *v, int p)
{
    const float *c = v->clip;
    switch (p) {
    case 0: return c[3] + c[0];
    case 1: return c[3] - c[0];
    case 2: return c[3] + c[1];
    case 3: return c[3] - c[1];
    case 4: return c[3] + c[2];
    case 5: return c[3] - c[2];
    default: return v->cd[p - 6];
    }
}

static int plane_on(int p) { return p < 6 || C->clip_en[p - 6]; }

static void mix(fv *o, const fv *a, const fv *b, float t)
{
    const float *x = (const float *)a, *y = (const float *)b;
    float *r = (float *)o;
    int i;
    for (i = 0; i < FV_FLOATS; i++) r[i] = x[i] + (y[i] - x[i]) * t;
}

static int clip_polygon(fv *v, int n)
{
    fv tmp[MAX_CLIPPED];
    int p, i;
    for (p = 0; p < 6 + MAX_CLIP && n; p++) {
        int m = 0;
        if (!plane_on(p)) continue;
        for (i = 0; i < n; i++) {
            const fv *a = &v[i], *b = &v[(i + 1) % n];
            float da = plane_d(a, p), db = plane_d(b, p);
            if (da >= 0.0f) tmp[m++] = *a;
            if ((da < 0.0f) != (db < 0.0f)) mix(&tmp[m++], a, b, da / (da - db));
        }
        memcpy(v, tmp, (size_t)m * sizeof *v);
        n = m;
    }
    return n;
}

static int clip_line(fv *a, fv *b)
{
    fv s = *a, e = *b;
    float t0 = 0.0f, t1 = 1.0f;
    int p;
    for (p = 0; p < 6 + MAX_CLIP; p++) {
        float da, db, t;
        if (!plane_on(p)) continue;
        da = plane_d(&s, p); db = plane_d(&e, p);
        if (da < 0.0f && db < 0.0f) return 0;
        if (da < 0.0f) { t = da / (da - db); if (t > t0) t0 = t; }
        else if (db < 0.0f) { t = da / (da - db); if (t < t1) t1 = t; }
    }
    if (t0 > t1) return 0;
    mix(a, &s, &e, t0);
    mix(b, &s, &e, t1);
    return 1;
}

static void window(const fv *v, float *o)
{
    float w = v->clip[3];
    o[0] = (v->clip[0] / w * 0.5f + 0.5f) * (float)C->vp[2] + (float)C->vp[0];
    o[1] = (v->clip[1] / w * 0.5f + 0.5f) * (float)C->vp[3] + (float)C->vp[1];
    o[2] = (v->clip[2] / w * 0.5f + 0.5f) * (C->depth_far - C->depth_near) + C->depth_near;
}

/* ---------------------------------------------------------------- selection */
static void sel_put(GLuint v)
{
    if (C->sel_n < (GLuint)C->sel_size) C->sel_buf[C->sel_n] = v;
    else C->sel_over = 1;
    C->sel_n++;
}

static void sel_flush(void)
{
    int i;
    if (!C->sel_hit) return;
    sel_put((GLuint)C->name_sp);
    sel_put((GLuint)((double)C->sel_min * 4294967295.0));
    sel_put((GLuint)((double)C->sel_max * 4294967295.0));
    for (i = 0; i < C->name_sp; i++) sel_put(C->names[i]);
    C->sel_hits++;
    C->sel_hit = 0;
}

static void sel_z(float z)
{
    if (!C->sel_hit || z < C->sel_min) C->sel_min = z;
    if (!C->sel_hit || z > C->sel_max) C->sel_max = z;
    C->sel_hit = 1;
}

/* ---------------------------------------------------------------- feedback */
static void fb_put(float v)
{
    if (C->fb_n < (GLuint)C->fb_size) C->fb_buf[C->fb_n] = v;
    else C->fb_over = 1;
    C->fb_n++;
}

static void fb_vertex(const fv *v, const float *col)
{
    float w[3];
    int c, colour = C->fb_type >= GL_3D_COLOR, tex = C->fb_type >= GL_3D_COLOR_TEXTURE;
    window(v, w);
    fb_put(w[0]); fb_put(w[1]);
    if (C->fb_type != GL_2D) fb_put(w[2]);
    if (C->fb_type == GL_4D_COLOR_TEXTURE) fb_put(v->clip[3]);
    if (colour) for (c = 0; c < 4; c++) fb_put(col[c]);
    if (tex) {
        for (c = 0; c < 4; c++) fb_put(v->tc[c]);
    }
}

static void do_point(const fv *a)
{
    fv v = *a;
    float w[3];
    int p;
    for (p = 0; p < 6 + MAX_CLIP; p++) if (plane_on(p) && plane_d(&v, p) < 0.0f) return;
    if (C->render_mode == GL_SELECT) { window(&v, w); sel_z(w[2]); return; }
    fb_put((float)GL_POINT_TOKEN);
    fb_vertex(&v, C->drawing_back ? v.colb : v.col);
}

static void do_line(const fv *a, const fv *b, int reset)
{
    fv s = *a, e = *b;
    float w[3];
    if (!clip_line(&s, &e)) return;
    if (C->render_mode == GL_SELECT) {
        window(&s, w); sel_z(w[2]);
        window(&e, w); sel_z(w[2]);
        return;
    }
    fb_put((float)(reset ? GL_LINE_RESET_TOKEN : GL_LINE_TOKEN));
    fb_vertex(&s, C->drawing_back ? s.colb : s.col);
    fb_vertex(&e, C->drawing_back ? e.colb : e.col);
}

static void do_tri(const fv *a, const fv *b, const fv *c)
{
    fv v[MAX_CLIPPED];
    float w[MAX_CLIPPED][3], area = 0.0f;
    int n, i, front;
    v[0] = *a; v[1] = *b; v[2] = *c;
    n = clip_polygon(v, 3);
    if (n < 3) return;
    for (i = 0; i < n; i++) window(&v[i], w[i]);
    for (i = 0; i < n; i++) area += w[i][0] * w[(i + 1) % n][1] - w[(i + 1) % n][0] * w[i][1];
    front = (area > 0.0f) == (C->front_face == GL_CCW);
    if (C->cull_face && (C->cull_mode == GL_FRONT_AND_BACK || (C->cull_mode == GL_BACK && !front) || (C->cull_mode == GL_FRONT && front))) return;
    if (C->render_mode == GL_SELECT) {
        for (i = 0; i < n; i++) sel_z(w[i][2]);
        return;
    }
    fb_put((float)GL_POLYGON_TOKEN);
    fb_put((float)n);
    for (i = 0; i < n; i++) fb_vertex(&v[i], front ? v[i].col : v[i].colb);
}

void gl_select_prims(const src *in, int cls, const uint32_t *list, uint32_t n)
{
    uint32_t i;
    fv a, b, c;
    if (!in[A_POS].base) return;
    if (cls == 2) {
        for (i = 0; i < n; i++) { make_vertex(in, list[i], &a); do_point(&a); }
    } else if (cls == 1) {
        for (i = 0; i + 1 < n; i += 2) {
            make_vertex(in, list[i], &a);
            make_vertex(in, list[i + 1], &b);
            do_line(&a, &b, i < 2 || list[i] != list[i - 1]);
        }
    } else {
        for (i = 0; i + 2 < n; i += 3) {
            make_vertex(in, list[i], &a);
            make_vertex(in, list[i + 1], &b);
            make_vertex(in, list[i + 2], &c);
            do_tri(&a, &b, &c);
        }
    }
}

/* The current raster position: a vertex through everything a vertex goes through, lighting and texture coordinates too, and clipped as a point: outside the view volume or a clip plane it is not valid (§2.12) */
void gl_raster_set(const float *obj)
{
    src in[A_COUNT];
    fv v;
    float w[3];
    int p;
    memset(in, 0, sizeof in);
    gl_src_current(in);                                         /* §2.13: a shader reads them */
    in[A_POS].base = (const uint8_t *)obj; in[A_POS].size = 4; in[A_POS].type = G2_AT_FLOAT; in[A_POS].bytes = 16; in[A_POS].stride = 16;
    C->vs_serial++;
    make_vertex(in, 0, &v);
    C->raster_valid = v.clip[3] > 0.0f;
    for (p = 0; p < 6 + MAX_CLIP; p++) if (plane_on(p) && plane_d(&v, p) < 0.0f) C->raster_valid = 0;
    if (!C->raster_valid) return;
    window(&v, w);
    C->raster_pos[0] = w[0]; C->raster_pos[1] = w[1]; C->raster_pos[2] = w[2]; C->raster_pos[3] = v.clip[3];
    memcpy(C->raster_color, v.col, 16);
    memcpy(C->raster_tc, v.tc, 16);
    C->raster_dist = sqrtf(v.eye[0] * v.eye[0] + v.eye[1] * v.eye[1] + v.eye[2] * v.eye[2]);
}

/* glBitmap, glDrawPixels, glCopyPixels while no fragments are made */
void gl_select_raster(GLenum token)
{
    fv v;
    if (!C->raster_valid) return;
    if (C->render_mode == GL_SELECT) { sel_z(C->raster_pos[2]); return; }
    memset(&v, 0, sizeof v);
    /* the window position back to what window() undoes */
    v.clip[0] = (C->raster_pos[0] - (float)C->vp[0]) / (float)C->vp[2] * 2.0f - 1.0f;
    v.clip[1] = (C->raster_pos[1] - (float)C->vp[1]) / (float)C->vp[3] * 2.0f - 1.0f;
    v.clip[2] = C->depth_far != C->depth_near ? (C->raster_pos[2] - C->depth_near) / (C->depth_far - C->depth_near) * 2.0f - 1.0f : 0.0f;
    v.clip[0] *= C->raster_pos[3]; v.clip[1] *= C->raster_pos[3]; v.clip[2] *= C->raster_pos[3];
    v.clip[3] = C->raster_pos[3];
    memcpy(v.tc, C->raster_tc, 16);
    fb_put((float)token);
    fb_vertex(&v, C->raster_color);
}

/* ---------------------------------------------------------------- the API */
void APIENTRY glSelectBuffer(GLsizei n, GLuint *b)
{
    if (n < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (C->render_mode == GL_SELECT) { SET_ERROR(GL_INVALID_OPERATION); return; }
    C->sel_buf = b; C->sel_size = n;
}

void APIENTRY glFeedbackBuffer(GLsizei n, GLenum type, GLfloat *b)
{
    if (n < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (type != GL_2D && type != GL_3D && type != GL_3D_COLOR && type != GL_3D_COLOR_TEXTURE && type != GL_4D_COLOR_TEXTURE) { SET_ERROR(GL_INVALID_ENUM); return; }
    if (C->render_mode == GL_FEEDBACK) { SET_ERROR(GL_INVALID_OPERATION); return; }
    C->fb_buf = b; C->fb_size = n; C->fb_type = type;
}

GLint APIENTRY glRenderMode(GLenum mode)
{
    GLint r = 0;
    FLUSH_VERTICES();
    if (mode != GL_RENDER && mode != GL_SELECT && mode != GL_FEEDBACK) { SET_ERROR(GL_INVALID_ENUM); return 0; }
    if ((mode == GL_SELECT && !C->sel_buf) || (mode == GL_FEEDBACK && !C->fb_buf)) { SET_ERROR(GL_INVALID_OPERATION); return 0; }
    if (C->render_mode == GL_SELECT) {
        sel_flush();
        r = C->sel_over ? -1 : C->sel_hits;
    } else if (C->render_mode == GL_FEEDBACK) {
        r = C->fb_over ? -1 : (GLint)C->fb_n;
    }
    C->render_mode = mode;
    C->sel_n = 0; C->sel_hits = 0; C->sel_over = 0; C->sel_hit = 0; C->name_sp = 0;
    C->fb_n = 0; C->fb_over = 0;
    return r;
}

void APIENTRY glInitNames(void) { FLUSH_VERTICES(); if (C->render_mode != GL_SELECT) return; sel_flush(); C->name_sp = 0; }
void APIENTRY glPushName(GLuint n)
{
    FLUSH_VERTICES();
    if (C->render_mode != GL_SELECT) return;
    sel_flush();
    if (C->name_sp >= MAX_NAME_DEPTH) { SET_ERROR(GL_STACK_OVERFLOW); return; }
    C->names[C->name_sp++] = n;
}
void APIENTRY glPopName(void)
{
    FLUSH_VERTICES();
    if (C->render_mode != GL_SELECT) return;
    sel_flush();
    if (C->name_sp <= 0) { SET_ERROR(GL_STACK_UNDERFLOW); return; }
    C->name_sp--;
}
void APIENTRY glLoadName(GLuint n)
{
    FLUSH_VERTICES();
    if (C->render_mode != GL_SELECT) return;
    if (C->name_sp <= 0) { SET_ERROR(GL_INVALID_OPERATION); return; }
    sel_flush();
    C->names[C->name_sp - 1] = n;
}
void APIENTRY glPassThrough(GLfloat t)
{
    FLUSH_VERTICES();
    if (C->render_mode != GL_FEEDBACK) return;
    fb_put((float)GL_PASS_THROUGH_TOKEN);
    fb_put(t);
}
