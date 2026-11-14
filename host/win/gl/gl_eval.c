/* evaluators (OpenGL 1.3 §5.1): Bezier maps of one and two variables, on the PC. glEvalCoord works out the vertex and hands it to the immediate-mode path as glVertex would, and leaves the current
 * colour, normal and texture coordinate as it found them. */
#include "gl_internal.h"

void gl_begin(GLenum mode);
void gl_end(void);
void gl_vertex(float x, float y, float z, float w);

/* index in a map array of the target; -1: not one. COLOR_4 INDEX NORMAL TEXTURE_COORD_1 2 3 4 VERTEX_3 VERTEX_4, in the order of their enum values */
static const int COMPS[9] = { 4, 1, 3, 1, 2, 3, 4, 3, 4 };
static int map_of(GLenum t, int dim) { GLenum b = dim == 1 ? GL_MAP1_COLOR_4 : GL_MAP2_COLOR_4; return t >= b && t < b + 9 ? (int)(t - b) : -1; }
enum { M_COLOR, M_INDEX, M_NORMAL, M_TC1, M_TC2, M_TC3, M_TC4, M_V3, M_V4 };

void gl_eval_clear(gl_ctx *c)
{
    static const float zero[4], one[4] = { 1, 1, 1, 1 }, nz[3] = { 0, 0, 1 }, tc[4] = { 0, 0, 0, 1 };
    /* table 5.1: an unset map is order 1 over [0, 1], and its one point is the initial current value */
    const float *def[9] = { one, one, nz, tc, tc, tc, tc, zero, tc };
    int d, m;
    for (d = 0; d < 2; d++) {
        for (m = 0; m < 9; m++) {
            gl_map *e = &c->map[d][m];
            e->order[0] = e->order[1] = 1; e->comps = COMPS[m];
            e->d[0] = 0; e->d[1] = 1; e->d[2] = 0; e->d[3] = 1;
            e->pts = (float *)malloc((size_t)COMPS[m] * sizeof(float));
            if (e->pts) memcpy(e->pts, def[m], (size_t)COMPS[m] * sizeof(float));
        }
    }
}

/* de Casteljau over `order` points of n components, `step` floats apart: the value and, if d, its derivative in t */
static void casteljau(const float *p, int step, int order, int n, float t, float *o, float *d)
{
    float w[MAX_EVAL_ORDER][4];
    int i, k, c;
    for (i = 0; i < order; i++) for (c = 0; c < n; c++) w[i][c] = p[i * step + c];
    for (k = order - 1; k > 0; k--) {
        if (k == 1 && d) for (c = 0; c < n; c++) d[c] = (w[1][c] - w[0][c]) * (float)(order - 1);
        for (i = 0; i < k; i++) for (c = 0; c < n; c++) w[i][c] = w[i][c] * (1.0f - t) + w[i + 1][c] * t;
    }
    if (order == 1 && d) for (c = 0; c < n; c++) d[c] = 0.0f;
    for (c = 0; c < n; c++) o[c] = w[0][c];
}

static void eval1(const gl_map *e, float u, float *o)
{
    float t = e->d[1] != e->d[0] ? (u - e->d[0]) / (e->d[1] - e->d[0]) : 0.0f;
    casteljau(e->pts, e->comps, e->order[0], e->comps, t, o, NULL);
}

/* du, dv: the partial derivatives in the map's own u and v, for GL_AUTO_NORMAL */
static void eval2(const gl_map *e, float u, float v, float *o, float *du, float *dv)
{
    float s = e->d[1] != e->d[0] ? (u - e->d[0]) / (e->d[1] - e->d[0]) : 0.0f, t = e->d[3] != e->d[2] ? (v - e->d[2]) / (e->d[3] - e->d[2]) : 0.0f;
    float rows[MAX_EVAL_ORDER][4], cols[MAX_EVAL_ORDER][4];
    int n = e->comps, i, j, c;
    for (i = 0; i < e->order[0]; i++) casteljau(e->pts + (size_t)i * e->order[1] * n, n, e->order[1], n, t, rows[i], NULL);
    casteljau(&rows[0][0], 4, e->order[0], n, s, o, du);
    if (dv) {
        for (j = 0; j < e->order[1]; j++) {
            float col[MAX_EVAL_ORDER][4];
            for (i = 0; i < e->order[0]; i++) for (c = 0; c < n; c++) col[i][c] = e->pts[((size_t)i * e->order[1] + j) * n + c];
            casteljau(&col[0][0], 4, e->order[0], n, s, cols[j], NULL);
        }
        { float ig[4]; casteljau(&cols[0][0], 4, e->order[1], n, t, ig, dv); }
    }
}

/* the vertex of the enabled maps, §5.1. dim 1 or 2 */
static void eval_vertex(int dim, float u, float v)
{
    const int d = dim - 1, *en = C->map_en[d];
    int vm = en[M_V4] ? M_V4 : en[M_V3] ? M_V3 : -1, tm = en[M_TC4] ? M_TC4 : en[M_TC3] ? M_TC3 : en[M_TC2] ? M_TC2 : en[M_TC1] ? M_TC1 : -1;
    float pos[4] = { 0, 0, 0, 1 }, du[4], dv[4], col[4], nrm[3], tc[4], saved_c[4], saved_n[3], saved_t[4], x[4];
    if (vm < 0) return;                                             /* no vertex map: no vertex */
    memcpy(saved_c, C->color, 16); memcpy(saved_n, C->normal, 12); memcpy(saved_t, C->unit[0].tc, 16);
    memcpy(col, C->color, 16); memcpy(nrm, C->normal, 12); memcpy(tc, C->unit[0].tc, 16);
    if (dim == 1) eval1(&C->map[0][vm], u, pos);
    else eval2(&C->map[1][vm], u, v, pos, du, dv);
    if (en[M_COLOR]) { if (dim == 1) eval1(&C->map[0][M_COLOR], u, col); else eval2(&C->map[1][M_COLOR], u, v, col, NULL, NULL); }
    if (tm >= 0) {
        float t[4] = { 0, 0, 0, 1 };
        if (dim == 1) eval1(&C->map[0][tm], u, t); else eval2(&C->map[1][tm], u, v, t, NULL, NULL);
        memcpy(tc, t, 16);
    }
    if (dim == 2 && C->auto_normal) {
        float m;
        /* §5.1: for VERTEX_4 the derivatives of x / w */
        if (vm == M_V4) {
            int c;
            for (c = 0; c < 3; c++) { du[c] = (du[c] * pos[3] - pos[c] * du[3]) / (pos[3] * pos[3]); dv[c] = (dv[c] * pos[3] - pos[c] * dv[3]) / (pos[3] * pos[3]); }
        }
        x[0] = du[1] * dv[2] - du[2] * dv[1]; x[1] = du[2] * dv[0] - du[0] * dv[2]; x[2] = du[0] * dv[1] - du[1] * dv[0];
        m = sqrtf(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
        if (m > 0.0f) { nrm[0] = x[0] / m; nrm[1] = x[1] / m; nrm[2] = x[2] / m; }
    } else if (en[M_NORMAL]) {
        if (dim == 1) eval1(&C->map[0][M_NORMAL], u, nrm); else eval2(&C->map[1][M_NORMAL], u, v, nrm, NULL, NULL);
    }
    memcpy(C->color, col, 16); memcpy(C->normal, nrm, 12); memcpy(C->unit[0].tc, tc, 16);
    C->cur_dirty |= 7;
    gl_vertex(pos[0], pos[1], pos[2], vm == M_V4 ? pos[3] : 1.0f);
    memcpy(C->color, saved_c, 16); memcpy(C->normal, saved_n, 12); memcpy(C->unit[0].tc, saved_t, 16);
    C->cur_dirty |= 7;
}

void APIENTRY glEvalCoord1f(GLfloat u) { eval_vertex(1, u, 0); }
void APIENTRY glEvalCoord1d(GLdouble u) { eval_vertex(1, (float)u, 0); }
void APIENTRY glEvalCoord1fv(const GLfloat *u) { eval_vertex(1, u[0], 0); }
void APIENTRY glEvalCoord1dv(const GLdouble *u) { eval_vertex(1, (float)u[0], 0); }
void APIENTRY glEvalCoord2f(GLfloat u, GLfloat v) { eval_vertex(2, u, v); }
void APIENTRY glEvalCoord2d(GLdouble u, GLdouble v) { eval_vertex(2, (float)u, (float)v); }
void APIENTRY glEvalCoord2fv(const GLfloat *u) { eval_vertex(2, u[0], u[1]); }
void APIENTRY glEvalCoord2dv(const GLdouble *u) { eval_vertex(2, (float)u[0], (float)u[1]); }

/* stored tightly, whatever stride they came with */
static gl_map *map_set(GLenum target, int dim, int uo, int vo, const float *d)
{
    int m = map_of(target, dim);
    gl_map *e;
    float *pts;
    if (m < 0) { SET_ERROR(GL_INVALID_ENUM); return NULL; }
    if (uo < 1 || uo > MAX_EVAL_ORDER || vo < 1 || vo > MAX_EVAL_ORDER || d[0] == d[1] || (dim == 2 && d[2] == d[3])) { SET_ERROR(GL_INVALID_VALUE); return NULL; }
    FLUSH_VERTICES();
    e = &C->map[dim - 1][m];
    pts = (float *)malloc((size_t)uo * vo * e->comps * sizeof(float));
    if (!pts) { SET_ERROR(GL_OUT_OF_MEMORY); return NULL; }
    free(e->pts);
    e->pts = pts; e->order[0] = uo; e->order[1] = vo;
    memcpy(e->d, d, 16);
    return e;
}

#define MAP_FUNCS(sfx, T) \
    void APIENTRY glMap1##sfx(GLenum t, T u1, T u2, GLint st, GLint o, const T *p) \
    { \
        float d[4]; gl_map *e; int i, c; \
        d[0] = (float)u1; d[1] = (float)u2; d[2] = 0; d[3] = 1; \
        e = map_set(t, 1, o, 1, d); \
        if (!e) return; \
        if (st < e->comps) { SET_ERROR(GL_INVALID_VALUE); return; } \
        for (i = 0; i < o; i++) for (c = 0; c < e->comps; c++) e->pts[i * e->comps + c] = (float)p[i * st + c]; \
    } \
    void APIENTRY glMap2##sfx(GLenum t, T u1, T u2, GLint us, GLint uo, T v1, T v2, GLint vs, GLint vo, const T *p) \
    { \
        float d[4]; gl_map *e; int i, j, c; \
        d[0] = (float)u1; d[1] = (float)u2; d[2] = (float)v1; d[3] = (float)v2; \
        e = map_set(t, 2, uo, vo, d); \
        if (!e) return; \
        if (us < e->comps || vs < e->comps) { SET_ERROR(GL_INVALID_VALUE); return; } \
        for (i = 0; i < uo; i++) for (j = 0; j < vo; j++) for (c = 0; c < e->comps; c++) e->pts[((size_t)i * vo + j) * e->comps + c] = (float)p[i * us + j * vs + c]; \
    }
MAP_FUNCS(f, GLfloat)
MAP_FUNCS(d, GLdouble)

void APIENTRY glMapGrid1f(GLint n, GLfloat u1, GLfloat u2) { if (n <= 0) { SET_ERROR(GL_INVALID_VALUE); return; } C->grid1_n = n; C->grid1[0] = u1; C->grid1[1] = u2; }
void APIENTRY glMapGrid1d(GLint n, GLdouble u1, GLdouble u2) { glMapGrid1f(n, (float)u1, (float)u2); }
void APIENTRY glMapGrid2f(GLint un, GLfloat u1, GLfloat u2, GLint vn, GLfloat v1, GLfloat v2)
{
    if (un <= 0 || vn <= 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    C->grid2_n[0] = un; C->grid2_n[1] = vn; C->grid2[0] = u1; C->grid2[1] = u2; C->grid2[2] = v1; C->grid2[3] = v2;
}
void APIENTRY glMapGrid2d(GLint un, GLdouble u1, GLdouble u2, GLint vn, GLdouble v1, GLdouble v2) { glMapGrid2f(un, (float)u1, (float)u2, vn, (float)v1, (float)v2); }

/* the grid's point i: its last is exactly the far end, §5.1 */
static float grid_u(int i) { return i == C->grid1_n ? C->grid1[1] : C->grid1[0] + (float)i * (C->grid1[1] - C->grid1[0]) / (float)C->grid1_n; }
static float grid_a(int i) { return i == C->grid2_n[0] ? C->grid2[1] : C->grid2[0] + (float)i * (C->grid2[1] - C->grid2[0]) / (float)C->grid2_n[0]; }
static float grid_b(int j) { return j == C->grid2_n[1] ? C->grid2[3] : C->grid2[2] + (float)j * (C->grid2[3] - C->grid2[2]) / (float)C->grid2_n[1]; }

void APIENTRY glEvalPoint1(GLint i) { eval_vertex(1, grid_u(i), 0); }
void APIENTRY glEvalPoint2(GLint i, GLint j) { eval_vertex(2, grid_a(i), grid_b(j)); }

void APIENTRY glEvalMesh1(GLenum mode, GLint i1, GLint i2)
{
    int i;
    if (mode != GL_POINT && mode != GL_LINE) { SET_ERROR(GL_INVALID_ENUM); return; }
    gl_begin(mode == GL_POINT ? GL_POINTS : GL_LINE_STRIP);
    for (i = i1; i <= i2; i++) eval_vertex(1, grid_u(i), 0);
    gl_end();
}

void APIENTRY glEvalMesh2(GLenum mode, GLint i1, GLint i2, GLint j1, GLint j2)
{
    int i, j;
    switch (mode) {
    case GL_POINT:
        gl_begin(GL_POINTS);
        for (j = j1; j <= j2; j++) for (i = i1; i <= i2; i++) eval_vertex(2, grid_a(i), grid_b(j));
        gl_end();
        break;
    case GL_LINE:
        for (j = j1; j <= j2; j++) { gl_begin(GL_LINE_STRIP); for (i = i1; i <= i2; i++) eval_vertex(2, grid_a(i), grid_b(j)); gl_end(); }
        for (i = i1; i <= i2; i++) { gl_begin(GL_LINE_STRIP); for (j = j1; j <= j2; j++) eval_vertex(2, grid_a(i), grid_b(j)); gl_end(); }
        break;
    case GL_FILL:
        for (j = j1; j < j2; j++) {
            gl_begin(GL_QUAD_STRIP);
            for (i = i1; i <= i2; i++) { eval_vertex(2, grid_a(i), grid_b(j)); eval_vertex(2, grid_a(i), grid_b(j + 1)); }
            gl_end();
        }
        break;
    default: SET_ERROR(GL_INVALID_ENUM); break;
    }
}

static gl_map *map_get(GLenum t, int *dim)
{
    int m = map_of(t, 1);
    *dim = 1;
    if (m < 0) { m = map_of(t, 2); *dim = 2; }
    if (m < 0) { SET_ERROR(GL_INVALID_ENUM); return NULL; }
    return &C->map[*dim - 1][m];
}

#define GETMAP(sfx, T) \
    void APIENTRY glGetMap##sfx##v(GLenum t, GLenum q, T *v) \
    { \
        int dim, i, n; \
        gl_map *e = map_get(t, &dim); \
        if (!e) return; \
        switch (q) { \
        case GL_COEFF: n = e->order[0] * (dim == 2 ? e->order[1] : 1) * e->comps; for (i = 0; i < n; i++) v[i] = (T)e->pts[i]; break; \
        case GL_ORDER: v[0] = (T)e->order[0]; if (dim == 2) v[1] = (T)e->order[1]; break; \
        case GL_DOMAIN: for (i = 0; i < dim * 2; i++) v[i] = (T)e->d[i]; break; \
        default: SET_ERROR(GL_INVALID_ENUM); break; \
        } \
    }
GETMAP(f, GLfloat)
GETMAP(d, GLdouble)
GETMAP(i, GLint)
