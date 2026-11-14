/* OpenGL entry points of the vertex path: every type variant of glVertex / glColor / glNormal / glTexCoord, client arrays, draws, raster position, DrawPixels / Bitmap. */
#include "gl_internal.h"

void gl_begin(GLenum mode);
void gl_end(void);
void gl_vertex(float x, float y, float z, float w);

/* §2.7 conversions of integer colours and normals */
#define N_b(c) ((float)(2 * (double)(c) + 1) / 255.0f)
#define N_ub(c) ((float)(c) / 255.0f)
#define N_s(c) ((float)(2 * (double)(c) + 1) / 65535.0f)
#define N_us(c) ((float)(c) / 65535.0f)
#define N_i(c) ((float)((2 * (double)(c) + 1) / 4294967295.0))
#define N_ui(c) ((float)((double)(c) / 4294967295.0))
#define N_f(c) ((float)(c))
#define N_d(c) ((float)(c))

void APIENTRY glBegin(GLenum m) { if (C->compiling) gl_list_sync_attribs(); LREC_I(LOP_BEGIN, m, 0); gl_begin(m); }
void APIENTRY glEnd(void) { LREC_I(LOP_END, 0, 0); gl_end(); }

#define V_FUNCS(sfx, T) \
    void APIENTRY glVertex2##sfx(T x, T y) { gl_vertex((float)x, (float)y, 0, 1); } \
    void APIENTRY glVertex3##sfx(T x, T y, T z) { gl_vertex((float)x, (float)y, (float)z, 1); } \
    void APIENTRY glVertex4##sfx(T x, T y, T z, T w) { gl_vertex((float)x, (float)y, (float)z, (float)w); } \
    void APIENTRY glVertex2##sfx##v(const T *v) { gl_vertex((float)v[0], (float)v[1], 0, 1); } \
    void APIENTRY glVertex3##sfx##v(const T *v) { gl_vertex((float)v[0], (float)v[1], (float)v[2], 1); } \
    void APIENTRY glVertex4##sfx##v(const T *v) { gl_vertex((float)v[0], (float)v[1], (float)v[2], (float)v[3]); } \
    void APIENTRY glTexCoord1##sfx(T s) { float *t = C->unit[0].tc; C->cur_dirty |= 4; t[0] = (float)s; t[1] = 0; t[2] = 0; t[3] = 1; } \
    void APIENTRY glTexCoord2##sfx(T s, T u) { float *t = C->unit[0].tc; C->cur_dirty |= 4; t[0] = (float)s; t[1] = (float)u; t[2] = 0; t[3] = 1; } \
    void APIENTRY glTexCoord3##sfx(T s, T u, T r) { float *t = C->unit[0].tc; C->cur_dirty |= 4; t[0] = (float)s; t[1] = (float)u; t[2] = (float)r; t[3] = 1; } \
    void APIENTRY glTexCoord4##sfx(T s, T u, T r, T q) { float *t = C->unit[0].tc; C->cur_dirty |= 4; if ((float)q != 1.0f) C->tc4 |= 1; t[0] = (float)s; t[1] = (float)u; t[2] = (float)r; t[3] = (float)q; } \
    void APIENTRY glTexCoord1##sfx##v(const T *v) { glTexCoord1##sfx(v[0]); } \
    void APIENTRY glTexCoord2##sfx##v(const T *v) { glTexCoord2##sfx(v[0], v[1]); } \
    void APIENTRY glTexCoord3##sfx##v(const T *v) { glTexCoord3##sfx(v[0], v[1], v[2]); } \
    void APIENTRY glTexCoord4##sfx##v(const T *v) { glTexCoord4##sfx(v[0], v[1], v[2], v[3]); } \
    void APIENTRY glMultiTexCoord1##sfx(GLenum e, T s) { float *t = C->unit[(e - GL_TEXTURE0) % MAX_UNITS].tc; C->cur_dirty |= 4u << ((e - GL_TEXTURE0) % MAX_UNITS); t[0] = (float)s; t[1] = 0; t[2] = 0; t[3] = 1; } \
    void APIENTRY glMultiTexCoord2##sfx(GLenum e, T s, T u) { float *t = C->unit[(e - GL_TEXTURE0) % MAX_UNITS].tc; C->cur_dirty |= 4u << ((e - GL_TEXTURE0) % MAX_UNITS); t[0] = (float)s; t[1] = (float)u; t[2] = 0; t[3] = 1; } \
    void APIENTRY glMultiTexCoord3##sfx(GLenum e, T s, T u, T r) { float *t = C->unit[(e - GL_TEXTURE0) % MAX_UNITS].tc; C->cur_dirty |= 4u << ((e - GL_TEXTURE0) % MAX_UNITS); t[0] = (float)s; t[1] = (float)u; t[2] = (float)r; t[3] = 1; } \
    void APIENTRY glMultiTexCoord4##sfx(GLenum e, T s, T u, T r, T q) { float *t = C->unit[(e - GL_TEXTURE0) % MAX_UNITS].tc; C->cur_dirty |= 4u << ((e - GL_TEXTURE0) % MAX_UNITS); if ((float)q != 1.0f) C->tc4 |= 1u << ((e - GL_TEXTURE0) % MAX_UNITS); t[0] = (float)s; t[1] = (float)u; t[2] = (float)r; t[3] = (float)q; } \
    void APIENTRY glMultiTexCoord1##sfx##v(GLenum e, const T *v) { glMultiTexCoord1##sfx(e, v[0]); } \
    void APIENTRY glMultiTexCoord2##sfx##v(GLenum e, const T *v) { glMultiTexCoord2##sfx(e, v[0], v[1]); } \
    void APIENTRY glMultiTexCoord3##sfx##v(GLenum e, const T *v) { glMultiTexCoord3##sfx(e, v[0], v[1], v[2]); } \
    void APIENTRY glMultiTexCoord4##sfx##v(GLenum e, const T *v) { glMultiTexCoord4##sfx(e, v[0], v[1], v[2], v[3]); }
V_FUNCS(d, GLdouble)
V_FUNCS(f, GLfloat)
V_FUNCS(i, GLint)
V_FUNCS(s, GLshort)

#define COL_FUNCS(sfx, T, N) \
    void APIENTRY glColor3##sfx(T r, T g, T b) { C->color[0] = N(r); C->color[1] = N(g); C->color[2] = N(b); C->color[3] = 1.0f; C->cur_dirty |= 1; } \
    void APIENTRY glColor4##sfx(T r, T g, T b, T a) { C->color[0] = N(r); C->color[1] = N(g); C->color[2] = N(b); C->color[3] = N(a); C->cur_dirty |= 1; } \
    void APIENTRY glColor3##sfx##v(const T *v) { glColor3##sfx(v[0], v[1], v[2]); } \
    void APIENTRY glColor4##sfx##v(const T *v) { glColor4##sfx(v[0], v[1], v[2], v[3]); }
COL_FUNCS(b, GLbyte, N_b)
COL_FUNCS(d, GLdouble, N_d)
COL_FUNCS(f, GLfloat, N_f)
COL_FUNCS(i, GLint, N_i)
COL_FUNCS(s, GLshort, N_s)
COL_FUNCS(ub, GLubyte, N_ub)
COL_FUNCS(ui, GLuint, N_ui)
COL_FUNCS(us, GLushort, N_us)

#define NRM_FUNCS(sfx, T, N) \
    void APIENTRY glNormal3##sfx(T x, T y, T z) { C->normal[0] = N(x); C->normal[1] = N(y); C->normal[2] = N(z); C->cur_dirty |= 2; } \
    void APIENTRY glNormal3##sfx##v(const T *v) { glNormal3##sfx(v[0], v[1], v[2]); }
NRM_FUNCS(b, GLbyte, N_b)
NRM_FUNCS(d, GLdouble, N_d)
NRM_FUNCS(f, GLfloat, N_f)
NRM_FUNCS(i, GLint, N_i)
NRM_FUNCS(s, GLshort, N_s)

void APIENTRY glEdgeFlag(GLboolean f) { LREC_I(LOP_EDGEFLAG, f, 0); C->edge_flag = f != 0; }
void APIENTRY glEdgeFlagv(const GLboolean *f) { glEdgeFlag(*f); }

#define RECT_FUNCS(sfx, T) \
    void APIENTRY glRect##sfx(T x1, T y1, T x2, T y2) { gl_begin(GL_POLYGON); gl_vertex((float)x1, (float)y1, 0, 1); gl_vertex((float)x2, (float)y1, 0, 1); gl_vertex((float)x2, (float)y2, 0, 1); gl_vertex((float)x1, (float)y2, 0, 1); gl_end(); } \
    void APIENTRY glRect##sfx##v(const T *a, const T *b) { glRect##sfx(a[0], a[1], b[0], b[1]); }
RECT_FUNCS(d, GLdouble)
RECT_FUNCS(f, GLfloat)
RECT_FUNCS(i, GLint)
RECT_FUNCS(s, GLshort)

/* ---------------------------------------------------------------- arrays */
void gl_set_array(gl_array *a, GLint size, GLenum type, GLsizei stride, const void *p)
{
    FLUSH_VERTICES();
    /* Quake 3's fog pass rewrites colours and texture coordinates under the lock and only says so by setting the same pointers again: a pointer set under a lock unlocks that array */
    if (C->cva_locked) C->cva_mask &= ~(1u << (a == &C->a_vertex ? A_POS : a == &C->a_color ? A_COLOR : a == &C->a_normal ? A_NORMAL : a == &C->a_fog ? A_FOGC : a == &C->a_sec ? A_SEC : A_TC0 + (int)(a - C->a_tc)));
    a->size = size; a->type = type; a->stride = stride; a->ptr = p; a->buffer = C->bound_array;
}
void APIENTRY glVertexPointer(GLint s, GLenum t, GLsizei st, const void *p) { gl_set_array(&C->a_vertex, s, t, st, p); }
void APIENTRY glColorPointer(GLint s, GLenum t, GLsizei st, const void *p) { gl_set_array(&C->a_color, s, t, st, p); }
void APIENTRY glNormalPointer(GLenum t, GLsizei st, const void *p) { gl_set_array(&C->a_normal, 3, t, st, p); }
void APIENTRY glTexCoordPointer(GLint s, GLenum t, GLsizei st, const void *p) { gl_set_array(&C->a_tc[C->client_unit], s, t, st, p); }
void APIENTRY glIndexPointer(GLenum t, GLsizei st, const void *p) { (void)t; (void)st; (void)p; }
void APIENTRY glEdgeFlagPointer(GLsizei st, const void *p) { FLUSH_VERTICES(); C->a_edge.stride = st; C->a_edge.ptr = p; C->a_edge.buffer = C->bound_array; }

static gl_array *client_array(GLenum c)
{
    switch (c) {
    case GL_VERTEX_ARRAY: return &C->a_vertex;
    case GL_COLOR_ARRAY: return &C->a_color;
    case GL_NORMAL_ARRAY: return &C->a_normal;
    case GL_TEXTURE_COORD_ARRAY: return &C->a_tc[C->client_unit];
    case GL_EDGE_FLAG_ARRAY: return &C->a_edge;
    case GL_FOG_COORD_ARRAY: return &C->a_fog;
    case GL_SECONDARY_COLOR_ARRAY: return &C->a_sec;
    default: return NULL;
    }
}
void APIENTRY glEnableClientState(GLenum c) { gl_array *a = client_array(c); FLUSH_VERTICES(); if (a) a->enabled = 1; }
void APIENTRY glDisableClientState(GLenum c) { gl_array *a = client_array(c); FLUSH_VERTICES(); if (a) a->enabled = 0; }
void APIENTRY glDrawArrays(GLenum m, GLint f, GLsizei n) { gl_draw_arrays(m, f, n); }
void APIENTRY glDrawElements(GLenum m, GLsizei n, GLenum t, const void *i) { gl_draw_elements(m, n, t, i, 0, 0, 0); }
void APIENTRY glDrawRangeElements(GLenum m, GLuint s, GLuint e, GLsizei n, GLenum t, const void *i) { gl_draw_elements(m, n, t, i, s, e, 1); }
void APIENTRY glLockArraysEXT(GLint f, GLsizei n)
{
    int u;
    FLUSH_VERTICES();
    if (f < 0 || n <= 0 || C->cva_locked) { SET_ERROR(C->cva_locked ? GL_INVALID_OPERATION : GL_INVALID_VALUE); return; }
    C->cva_first = f; C->cva_count = n; C->cva_locked = 1;
    C->cva_mask = (C->a_vertex.enabled ? 1u << A_POS : 0) | (C->a_color.enabled ? 1u << A_COLOR : 0) | (C->a_normal.enabled ? 1u << A_NORMAL : 0) | (C->a_fog.enabled ? 1u << A_FOGC : 0) | (C->a_sec.enabled ? 1u << A_SEC : 0);
    for (u = 0; u < MAX_UNITS; u++) if (C->a_tc[u].enabled) C->cva_mask |= 1u << (A_TC0 + u);
    memset(C->cva, 0, sizeof C->cva);
}
void APIENTRY glUnlockArraysEXT(void) { FLUSH_VERTICES(); C->cva_locked = 0; }
void APIENTRY glActiveTexture(GLenum t) { FLUSH_VERTICES(); C->active_unit = (int)(t - GL_TEXTURE0) & (MAX_UNITS - 1); }
void APIENTRY glClientActiveTexture(GLenum t) { C->client_unit = (int)(t - GL_TEXTURE0) & (MAX_UNITS - 1); }

static void fetch(const gl_array *a, GLint i, float *o, int norm)
{
    const uint8_t *b = (const uint8_t *)a->ptr, *e;
    int es, k;
    if (a->buffer) { gl_buffer *x = gl_buffer_lookup(a->buffer); b = x && x->data ? x->data + (uintptr_t)a->ptr : NULL; }
    if (!b) return;
    switch (a->type) { case GL_BYTE: case GL_UNSIGNED_BYTE: es = 1; break; case GL_SHORT: case GL_UNSIGNED_SHORT: es = 2; break; case GL_DOUBLE: es = 8; break; default: es = 4; break; }
    e = b + (ptrdiff_t)i * (a->stride ? a->stride : es * a->size);
    for (k = 0; k < a->size; k++) {
        switch (a->type) {
        case GL_BYTE: o[k] = norm ? N_b(((const GLbyte *)e)[k]) : (float)((const GLbyte *)e)[k]; break;
        case GL_UNSIGNED_BYTE: o[k] = norm ? N_ub(e[k]) : (float)e[k]; break;
        case GL_SHORT: o[k] = norm ? N_s(((const GLshort *)e)[k]) : (float)((const GLshort *)e)[k]; break;
        case GL_UNSIGNED_SHORT: o[k] = norm ? N_us(((const GLushort *)e)[k]) : (float)((const GLushort *)e)[k]; break;
        case GL_INT: o[k] = norm ? N_i(((const GLint *)e)[k]) : (float)((const GLint *)e)[k]; break;
        case GL_UNSIGNED_INT: o[k] = norm ? N_ui(((const GLuint *)e)[k]) : (float)((const GLuint *)e)[k]; break;
        case GL_DOUBLE: o[k] = (float)((const GLdouble *)e)[k]; break;
        default: o[k] = ((const GLfloat *)e)[k]; break;
        }
    }
}

void APIENTRY glArrayElement(GLint i)
{
    float v[4] = { 0, 0, 0, 1 };
    int u;
    if (C->a_color.enabled) { float c[4] = { 0, 0, 0, 1 }; fetch(&C->a_color, i, c, 1); memcpy(C->color, c, 16); }
    if (C->a_normal.enabled) fetch(&C->a_normal, i, C->normal, 1);
    if (C->a_fog.enabled) fetch(&C->a_fog, i, &C->fog_coord, 0);
    if (C->a_sec.enabled) fetch(&C->a_sec, i, C->sec_color, 1);
    if (C->a_edge.enabled && C->a_edge.ptr) C->edge_flag = ((const GLboolean *)C->a_edge.ptr)[(ptrdiff_t)i * (C->a_edge.stride ? C->a_edge.stride : 1)] != 0;
    for (u = 0; u < MAX_UNITS; u++) {
        if (C->a_tc[u].enabled) { float t[4] = { 0, 0, 0, 1 }; fetch(&C->a_tc[u], i, t, 0); memcpy(C->unit[u].tc, t, 16); }
    }
    if (C->a_vertex.enabled) { fetch(&C->a_vertex, i, v, 0); gl_vertex(v[0], v[1], v[2], v[3]); }
}

void APIENTRY glInterleavedArrays(GLenum f, GLsizei stride, const void *p)
{
    int tc = 0, cs = 0, ns = 0, vs = 3, cf = 0;
    GLenum ct = GL_FLOAT;
    int o = 0, s;
    const uint8_t *b = (const uint8_t *)p;
    switch (f) {
    case GL_V2F: vs = 2; break;
    case GL_V3F: break;
    case GL_C4UB_V2F: cs = 4; ct = GL_UNSIGNED_BYTE; vs = 2; cf = 4; break;
    case GL_C4UB_V3F: cs = 4; ct = GL_UNSIGNED_BYTE; cf = 4; break;
    case GL_C3F_V3F: cs = 3; cf = 12; break;
    case GL_N3F_V3F: ns = 12; break;
    case GL_C4F_N3F_V3F: cs = 4; cf = 16; ns = 12; break;
    case GL_T2F_V3F: tc = 2; break;
    case GL_T4F_V4F: tc = 4; vs = 4; break;
    case GL_T2F_C4UB_V3F: tc = 2; cs = 4; ct = GL_UNSIGNED_BYTE; cf = 4; break;
    case GL_T2F_C3F_V3F: tc = 2; cs = 3; cf = 12; break;
    case GL_T2F_N3F_V3F: tc = 2; ns = 12; break;
    case GL_T2F_C4F_N3F_V3F: tc = 2; cs = 4; cf = 16; ns = 12; break;
    case GL_T4F_C4F_N3F_V4F: tc = 4; cs = 4; cf = 16; ns = 12; vs = 4; break;
    default: SET_ERROR(GL_INVALID_ENUM); return;
    }
    s = stride ? stride : tc * 4 + cf + ns + vs * 4;
    C->a_tc[C->client_unit].enabled = tc != 0;
    C->a_edge.enabled = 0;
    C->a_color.enabled = cs != 0;
    C->a_normal.enabled = ns != 0;
    C->a_vertex.enabled = 1;
    if (tc) { glTexCoordPointer(tc, GL_FLOAT, s, b); o += tc * 4; }
    if (cs) { glColorPointer(cs, ct, s, b + o); o += cf; }
    if (ns) { glNormalPointer(GL_FLOAT, s, b + o); o += ns; }
    glVertexPointer(vs, GL_FLOAT, s, b + o);
}

/* ---------------------------------------------------------------- raster position, pixels through the fragment pipeline */
#define RP_FUNCS(sfx, T) \
    void APIENTRY glRasterPos2##sfx(T x, T y) { gl_raster((float)x, (float)y, 0, 1); } \
    void APIENTRY glRasterPos3##sfx(T x, T y, T z) { gl_raster((float)x, (float)y, (float)z, 1); } \
    void APIENTRY glRasterPos4##sfx(T x, T y, T z, T w) { gl_raster((float)x, (float)y, (float)z, (float)w); } \
    void APIENTRY glRasterPos2##sfx##v(const T *v) { gl_raster((float)v[0], (float)v[1], 0, 1); } \
    void APIENTRY glRasterPos3##sfx##v(const T *v) { gl_raster((float)v[0], (float)v[1], (float)v[2], 1); } \
    void APIENTRY glRasterPos4##sfx##v(const T *v) { gl_raster((float)v[0], (float)v[1], (float)v[2], (float)v[3]); }

static void gl_raster(float x, float y, float z, float w)
{
    float in[4];
    in[0] = x; in[1] = y; in[2] = z; in[3] = w;
    gl_raster_set(in);
}
RP_FUNCS(d, GLdouble)
RP_FUNCS(f, GLfloat)
RP_FUNCS(i, GLint)
RP_FUNCS(s, GLshort)

/* an RGBA rectangle at window (x, y) as a textured quad, so it passes the fragment operations as §3.6 asks. mask: the image only says which fragments exist, by alpha, and they have the raster colour */
static void draw_rgba(const uint8_t *rgba, int w, int h, float x, float y, float z, float zx, float zy, int mask)
{
    static GLuint tex;
    float xform[16], px, py, dn = C->depth_near, df = C->depth_far;
    GLenum saved_mode = C->matrix_mode, pf = C->poly_mode_front, pb = C->poly_mode_back;
    int au = C->active_unit, vp[4], transfer = C->px_active;
    g2r_target t;
    double kx, ky;
    GLuint bound;
    if (!tex) glGenTextures(1, &tex);
    /* pixels are in window coordinates: no viewport to be clipped by, no depth range to go through again */
    memcpy(vp, C->vp, sizeof vp);
    gl_cur_target(&t);
    gl_scale(&kx, &ky);
    C->vp[0] = 0; C->vp[1] = 0; C->vp[2] = (int)(t.width / kx + 0.5); C->vp[3] = (int)(t.height / ky + 0.5);
    C->depth_near = 0.0f; C->depth_far = 1.0f;
    px = 2.0f / (float)C->vp[2]; py = 2.0f / (float)C->vp[3];
    glPushAttrib(GL_ENABLE_BIT | GL_TEXTURE_BIT | GL_CURRENT_BIT);
    glDisable(GL_POLYGON_STIPPLE);                                  /* pixels are not polygons, §3.6 */
    glDisable(GL_POLYGON_OFFSET_FILL);
    C->poly_mode_front = C->poly_mode_back = GL_FILL;
    C->px_active = 0;                                               /* these have been through the transfer */
    C->internal_draw = 1;
    if (mask) { memcpy(C->color, C->raster_color, 16); C->drawing_mask = 1; }
    glActiveTexture(GL_TEXTURE0);
    bound = C->unit[0].bound_2d;
    glBindTexture(GL_TEXTURE_2D, tex);
    {
        int ra = C->un_row, sr = C->un_skip_row, sp = C->un_skip_px, al = C->un_align;
        C->un_row = 0; C->un_skip_row = 0; C->un_skip_px = 0; C->un_align = 4;
        GL_FROM_CLIENT(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba));
        C->un_row = ra; C->un_skip_row = sr; C->un_skip_px = sp; C->un_align = al;
    }
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    if (gl_glsl_user_fs()) { C->pic_tex = tex; C->unit[0].bound_2d = bound; }   /* a shader's samplers read what the application bound */
    /* the depth is the raster position's, and so is the distance fog goes by: the projection gives the one, the modelview's z is the other */
    glMatrixMode(GL_PROJECTION); glPushMatrix();
    gl_ident(xform);
    xform[10] = 0.0f; xform[14] = 2.0f * z - 1.0f;
    glLoadMatrixf(xform);
    glMatrixMode(GL_MODELVIEW); glPushMatrix();
    gl_ident(xform);
    xform[0] = px; xform[5] = py;
    xform[12] = -1.0f; xform[13] = -1.0f; xform[14] = -C->raster_dist;
    glLoadMatrixf(xform);
    gl_begin(GL_QUADS);
    C->unit[0].tc[0] = 0; C->unit[0].tc[1] = 0; C->unit[0].tc[2] = 0; C->unit[0].tc[3] = 1; gl_vertex(x, y, 0, 1);
    C->unit[0].tc[0] = 1; gl_vertex(x + (float)w * zx, y, 0, 1);
    C->unit[0].tc[1] = 1; gl_vertex(x + (float)w * zx, y + (float)h * zy, 0, 1);
    C->unit[0].tc[0] = 0; gl_vertex(x, y + (float)h * zy, 0, 1);
    gl_end();
    gl_im_flush();
    C->drawing_mask = 0;
    C->internal_draw = 0;
    glPopMatrix();
    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(saved_mode);
    glPopAttrib();
    C->active_unit = au;
    C->px_active = transfer;
    C->poly_mode_front = pf; C->poly_mode_back = pb;
    memcpy(C->vp, vp, sizeof vp);
    C->depth_near = dn; C->depth_far = df;
}

/* depth or stencil pixels at the raster position: gl_pixel.c does what they do to the depth-stencil image, and the fragments that passed are drawn in the raster colour with the tests behind them off */
static void fragments(const float *z, const uint32_t *st, int w, int h)
{
    int rect[4], dt = C->depth_test, stn = C->stencil_test;
    uint8_t *mask = gl_px_fragments(z, st, w, h, C->pixel_zoom[0], C->pixel_zoom[1], rect);
    double kx, ky;
    if (!mask) return;
    gl_scale(&kx, &ky);
    if (C->color_mask) {
        C->depth_test = 0; C->stencil_test = 0;
        draw_rgba(mask, rect[2], rect[3], (float)(rect[0] / kx), (float)(rect[1] / ky), C->raster_pos[2], (float)(1.0 / kx), (float)(1.0 / ky), 1);
        C->depth_test = dt; C->stencil_test = stn;
    }
    free(mask);
}

static int have_ds(void)
{
    g2r_target t;
    gl_cur_target(&t);
    return t.ds != 0;
}

void APIENTRY glDrawPixels(GLsizei w, GLsizei h, GLenum format, GLenum type, const void *pixels)
{
    uint8_t *rgba;
    FLUSH_VERTICES();
    if (w < 0 || h < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (C->render_mode != GL_RENDER) { gl_select_raster(GL_DRAW_PIXEL_TOKEN); return; }
    if (!C->raster_valid || !w || !h || !C->live) return;
    if (w > MAX_TEX_SIZE || h > MAX_TEX_SIZE) { SET_ERROR(GL_INVALID_VALUE); return; }
    pixels = gl_unpack_ptr(pixels);
    if (format == GL_DEPTH_STENCIL_EXT) {                       /* EXT_packed_depth_stencil: both, as the two formats alone would */
        float *z = (float *)malloc((size_t)w * h * 4);
        uint32_t *s = (uint32_t *)malloc((size_t)w * h * 4);
        if (type != GL_UNSIGNED_INT_24_8_EXT) { free(z); free(s); SET_ERROR(GL_INVALID_ENUM); return; }
        if (!have_ds()) { free(z); free(s); SET_ERROR(GL_INVALID_OPERATION); return; }
        if (z && s) {
            gl_unpack_ds(z, s, w, h, pixels);
            fragments(NULL, s, w, h);
            fragments(z, NULL, w, h);
        }
        free(z); free(s);
        return;
    }
    if (format == GL_DEPTH_COMPONENT || format == GL_STENCIL_INDEX) {
        if (!gl_ds_type(format, type)) { SET_ERROR(GL_INVALID_ENUM); return; }
        if (!have_ds()) { SET_ERROR(GL_INVALID_OPERATION); return; }
        if (format == GL_DEPTH_COMPONENT) {
            float *z = (float *)malloc((size_t)w * h * 4);
            if (!z) return;
            gl_unpack_depth(z, w, h, type, pixels);
            fragments(z, NULL, w, h);
            free(z);
        } else {
            uint32_t *s = (uint32_t *)malloc((size_t)w * h * 4);
            if (!s) return;
            gl_unpack_index(s, w, h, type, pixels);
            fragments(NULL, s, w, h);
            free(s);
        }
        return;
    }
    rgba = (uint8_t *)malloc((size_t)w * h * 4);
    if (!rgba) return;
    gl_unpack(rgba, w, h, format, type, pixels, 0);
    draw_rgba(rgba, w, h, C->raster_pos[0], C->raster_pos[1], C->raster_pos[2], C->pixel_zoom[0], C->pixel_zoom[1], 0);
    free(rgba);
}

void APIENTRY glCopyPixels(GLint x, GLint y, GLsizei w, GLsizei h, GLenum type)
{
    g2r_target t;
    uint8_t *rgba;
    FLUSH_VERTICES();
    if (type != GL_COLOR && type != GL_DEPTH && type != GL_STENCIL && type != GL_DEPTH_STENCIL_EXT) { SET_ERROR(GL_INVALID_ENUM); return; }
    if (w < 0 || h < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (C->render_mode != GL_RENDER) { gl_select_raster(GL_COPY_PIXEL_TOKEN); return; }
    if (!C->raster_valid || !w || !h || !C->live) return;
    gl_bind_target();
    gl_read_target(&t);
    if (type != GL_COLOR && !t.ds) { SET_ERROR(GL_INVALID_OPERATION); return; }
    rgba = (uint8_t *)malloc((size_t)w * h * 4);
    if (!rgba) return;
    g2r_finish(D.r);
    if (type == GL_COLOR) {
        gl_read_rect(&t, t.color, G2_TF_RGBA8888, gl_tg_fmt(&t), x, y, w, h, rgba);
        gl_px_rows(rgba, w, h);
        draw_rgba(rgba, w, h, C->raster_pos[0], C->raster_pos[1], C->raster_pos[2], C->pixel_zoom[0], C->pixel_zoom[1], 0);
    } else if (type == GL_DEPTH_STENCIL_EXT) {
        uint32_t *v = (uint32_t *)rgba, *s = (uint32_t *)malloc((size_t)w * h * 4);
        float *z = (float *)rgba;
        size_t i, n = (size_t)w * h;
        gl_read_rect(&t, t.ds, G2_TF_RAW, G2_FMT_D24S8, x, y, w, h, rgba);
        if (s) {
            for (i = 0; i < n; i++) { uint32_t q = v[i]; s[i] = gl_px_stencil_value(q >> 24); z[i] = gl_px_depth_value((float)((q & 0xFFFFFF) / (t.depth16 ? 65535.0 : 16777215.0))); }
            fragments(NULL, s, w, h);
            fragments(z, NULL, w, h);
            free(s);
        }
    } else {
        uint32_t *v = (uint32_t *)rgba;
        size_t i, n = (size_t)w * h;
        gl_read_rect(&t, t.ds, G2_TF_RAW, G2_FMT_D24S8, x, y, w, h, rgba);
        if (type == GL_DEPTH) {
            float *z = (float *)rgba;
            for (i = 0; i < n; i++) z[i] = gl_px_depth_value((float)((v[i] & 0xFFFFFF) / (t.depth16 ? 65535.0 : 16777215.0)));
            fragments(z, NULL, w, h);
        } else {
            for (i = 0; i < n; i++) v[i] = gl_px_stencil_value(v[i] >> 24);
            fragments(NULL, v, w, h);
        }
    }
    free(rgba);
}
void APIENTRY glBitmap(GLsizei w, GLsizei h, GLfloat xo, GLfloat yo, GLfloat xm, GLfloat ym, const GLubyte *bits)
{
    bits = (const GLubyte *)gl_unpack_ptr(bits);
    if (C->compiling) {
        float f[4];
        f[0] = xo; f[1] = yo; f[2] = xm; f[3] = ym;
        gl_list_bitmap(w, h, f, bits, bits ? (size_t)((w + 7) / 8) * h : 0);
        if (C->compile_mode == GL_COMPILE) return;
    }
    FLUSH_VERTICES();
    if (C->render_mode != GL_RENDER) {
        gl_select_raster(GL_BITMAP_TOKEN);
        C->raster_pos[0] += xm;
        C->raster_pos[1] += ym;
        return;
    }
    if (bits && w > 0 && h > 0 && C->raster_valid && C->live) {
        uint8_t *rgba = (uint8_t *)malloc((size_t)w * h * 4);
        int row = ((w + 7) / 8 + C->un_align - 1) / C->un_align * C->un_align, x, y;
        if (rgba) {
            for (y = 0; y < h; y++) {
                for (x = 0; x < w; x++) {
                    int on = (bits[y * row + x / 8] >> (7 - x % 8)) & 1;
                    uint8_t *o = rgba + (size_t)(y * w + x) * 4;
                    memset(o, 0, 3);
                    o[3] = on ? 255 : 0;
                }
            }
            draw_rgba(rgba, w, h, C->raster_pos[0] - xo, C->raster_pos[1] - yo, C->raster_pos[2], 1.0f, 1.0f, 1);
            free(rgba);
        }
    }
    C->raster_pos[0] += xm;
    C->raster_pos[1] += ym;
}
