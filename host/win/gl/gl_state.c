/* OpenGL state: enables, per-fragment state, lights, materials, fog, texture environment, matrices, attribute stack, queries. Each call that changes what a draw would do ends the
 * immediate-mode batch first (FLUSH_VERTICES). */
#include "gl_internal.h"

/* ---------------------------------------------------------------- blend codes */
uint32_t gl_blend_code(GLenum f)
{
    switch (f) {
    case GL_ZERO: return G2R_BF_ZERO;
    case GL_ONE: return G2R_BF_ONE;
    case GL_SRC_COLOR: return G2R_BF_SRC_COLOR;
    case GL_ONE_MINUS_SRC_COLOR: return G2R_BF_ONE_MINUS_SRC_COLOR;
    case GL_DST_COLOR: return G2R_BF_DST_COLOR;
    case GL_ONE_MINUS_DST_COLOR: return G2R_BF_ONE_MINUS_DST_COLOR;
    case GL_SRC_ALPHA: return G2R_BF_SRC_ALPHA;
    case GL_ONE_MINUS_SRC_ALPHA: return G2R_BF_ONE_MINUS_SRC_ALPHA;
    case GL_DST_ALPHA: return G2R_BF_DST_ALPHA;
    case GL_ONE_MINUS_DST_ALPHA: return G2R_BF_ONE_MINUS_DST_ALPHA;
    case GL_CONSTANT_COLOR: return G2R_BF_CONSTANT_COLOR;
    case GL_ONE_MINUS_CONSTANT_COLOR: return G2R_BF_ONE_MINUS_CONSTANT_COLOR;
    case GL_CONSTANT_ALPHA: return G2R_BF_CONSTANT_ALPHA;
    case GL_ONE_MINUS_CONSTANT_ALPHA: return G2R_BF_ONE_MINUS_CONSTANT_ALPHA;
    case GL_SRC_ALPHA_SATURATE: return G2R_BF_SRC_ALPHA_SATURATE;
    default: return G2R_BF_ONE;
    }
}

uint32_t gl_blend_eq(GLenum e)
{
    switch (e) {
    case GL_FUNC_SUBTRACT: return G2R_BOP_SUB;
    case GL_FUNC_REVERSE_SUBTRACT: return G2R_BOP_REV_SUB;
    case GL_MIN: return G2R_BOP_MIN;
    case GL_MAX: return G2R_BOP_MAX;
    default: return G2R_BOP_ADD;
    }
}

/* ---------------------------------------------------------------- enables */
static int *enable_flag(GLenum cap)
{
    if (cap >= GL_CLIP_PLANE0 && cap < GL_CLIP_PLANE0 + MAX_CLIP) return &C->clip_en[cap - GL_CLIP_PLANE0];
    if (cap >= GL_LIGHT0 && cap < GL_LIGHT0 + MAX_LIGHTS) return &C->light[cap - GL_LIGHT0].enabled;
    if (cap >= GL_MAP1_COLOR_4 && cap <= GL_MAP1_VERTEX_4) return &C->map_en[0][cap - GL_MAP1_COLOR_4];
    if (cap >= GL_MAP2_COLOR_4 && cap <= GL_MAP2_VERTEX_4) return &C->map_en[1][cap - GL_MAP2_COLOR_4];
    switch (cap) {
    case GL_DEPTH_TEST: return &C->depth_test;
    case GL_CULL_FACE: return &C->cull_face;
    case GL_BLEND: return &C->blend;
    case GL_SCISSOR_TEST: return &C->scissor_test;
    case GL_ALPHA_TEST: return &C->alpha_test;
    case GL_STENCIL_TEST: return &C->stencil_test;
    case GL_FOG: return &C->fog;
    case GL_LIGHTING: return &C->lighting;
    case GL_COLOR_MATERIAL: return &C->color_material;
    case GL_NORMALIZE: return &C->normalize;
    case GL_RESCALE_NORMAL: return &C->rescale_normal;
    case GL_DITHER: return &C->dither;
    case GL_POLYGON_OFFSET_FILL: return &C->poly_offset_fill;
    case GL_POLYGON_OFFSET_LINE: return &C->poly_offset_line;
    case GL_POLYGON_OFFSET_POINT: return &C->poly_offset_point;
    case GL_COLOR_LOGIC_OP: return &C->color_logic_op;
    case GL_LINE_SMOOTH: return &C->line_smooth;
    case GL_POINT_SMOOTH: return &C->point_smooth;
    case GL_POLYGON_SMOOTH: return &C->polygon_smooth;
    case GL_LINE_STIPPLE: return &C->line_stipple;
    case GL_POLYGON_STIPPLE: return &C->polygon_stipple;
    case GL_AUTO_NORMAL: return &C->auto_normal;
    case GL_COLOR_SUM: return &C->color_sum;
    case GL_MULTISAMPLE: return &C->multisample;
    case GL_SAMPLE_ALPHA_TO_COVERAGE: return &C->sample_a2c;
    case GL_SAMPLE_ALPHA_TO_ONE: return &C->sample_a2o;
    case GL_SAMPLE_COVERAGE: return &C->sample_cov;
    case GL_TEXTURE_1D: return &C->unit[C->active_unit].en_1d;
    case GL_TEXTURE_2D: return &C->unit[C->active_unit].en_2d;
    case GL_TEXTURE_CUBE_MAP: return &C->unit[C->active_unit].en_cube;
    case GL_TEXTURE_3D: return &C->unit[C->active_unit].en_3d;
    case GL_VERTEX_PROGRAM_POINT_SIZE: return &C->vp_point_size;
    case GL_VERTEX_PROGRAM_TWO_SIDE: return &C->vp_two_side;
    case GL_POINT_SPRITE: return &C->point_sprite;
    case GL_TEXTURE_GEN_S: return &C->unit[C->active_unit].gen_en[0];
    case GL_TEXTURE_GEN_T: return &C->unit[C->active_unit].gen_en[1];
    case GL_TEXTURE_GEN_R: return &C->unit[C->active_unit].gen_en[2];
    case GL_TEXTURE_GEN_Q: return &C->unit[C->active_unit].gen_en[3];
    default: return NULL;
    }
}

void APIENTRY glEnable(GLenum cap)
{
    int *f;
    LREC_I(LOP_ENABLE, cap, 0);
    FLUSH_VERTICES();
    f = enable_flag(cap);
    if (f) *f = 1;
    else if (cap != GL_INDEX_LOGIC_OP) g2e_log("gl: glEnable(%04x) ignored", cap);
}

void APIENTRY glDisable(GLenum cap)
{
    int *f;
    LREC_I(LOP_DISABLE, cap, 0);
    FLUSH_VERTICES();
    f = enable_flag(cap);
    if (f) *f = 0;
}

GLboolean APIENTRY glIsEnabled(GLenum cap)
{
    int *f = enable_flag(cap);
    if (f) return *f ? GL_TRUE : GL_FALSE;
    switch (cap) {
    case GL_VERTEX_ARRAY: return (GLboolean)C->a_vertex.enabled;
    case GL_COLOR_ARRAY: return (GLboolean)C->a_color.enabled;
    case GL_NORMAL_ARRAY: return (GLboolean)C->a_normal.enabled;
    case GL_TEXTURE_COORD_ARRAY: return (GLboolean)C->a_tc[C->client_unit].enabled;
    case GL_EDGE_FLAG_ARRAY: return (GLboolean)C->a_edge.enabled;
    case GL_FOG_COORD_ARRAY: return (GLboolean)C->a_fog.enabled;
    case GL_SECONDARY_COLOR_ARRAY: return (GLboolean)C->a_sec.enabled;
    default: return GL_FALSE;
    }
}

/* ---------------------------------------------------------------- per-fragment state */
void APIENTRY glBlendFunc(GLenum s, GLenum d) { LREC_I(LOP_BLENDFUNC, s, d); FLUSH_VERTICES(); C->blend_src = C->blend_src_a = s; C->blend_dst = C->blend_dst_a = d; }
void APIENTRY glBlendFuncSeparate(GLenum s, GLenum d, GLenum sa, GLenum da) { FLUSH_VERTICES(); C->blend_src = s; C->blend_dst = d; C->blend_src_a = sa; C->blend_dst_a = da; }
void APIENTRY glBlendEquation(GLenum e) { FLUSH_VERTICES(); C->blend_eq = C->blend_eq_a = e; }
void APIENTRY glBlendEquationSeparate(GLenum e, GLenum a) { FLUSH_VERTICES(); C->blend_eq = e; C->blend_eq_a = a; }
void APIENTRY glBlendColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a) { FLUSH_VERTICES(); C->blend_color[0] = r; C->blend_color[1] = g; C->blend_color[2] = b; C->blend_color[3] = a; }
void APIENTRY glDepthFunc(GLenum f) { LREC_I(LOP_DEPTHFUNC, f, 0); FLUSH_VERTICES(); C->depth_func = f; }
void APIENTRY glDepthMask(GLboolean m) { LREC_I(LOP_DEPTHMASK, m, 0); FLUSH_VERTICES(); C->depth_mask = m ? 1 : 0; }
void APIENTRY glDepthRange(GLclampd n, GLclampd f)
{
    FLUSH_VERTICES();
    C->depth_near = (float)(n < 0 ? 0 : n > 1 ? 1 : n);
    C->depth_far = (float)(f < 0 ? 0 : f > 1 ? 1 : f);
}
void APIENTRY glAlphaFunc(GLenum f, GLclampf r) { LREC_F(LOP_ALPHAFUNC, f, r, 0, 0, 0); FLUSH_VERTICES(); C->alpha_func = f; C->alpha_ref = r < 0 ? 0 : r > 1 ? 1 : r; }
void APIENTRY glColorMask(GLboolean r, GLboolean g, GLboolean b, GLboolean a) { LREC_I(LOP_COLORMASK, (r ? 8 : 0) | (g ? 4 : 0) | (b ? 2 : 0) | (a ? 1 : 0), 0); FLUSH_VERTICES(); C->color_mask = (r ? 8 : 0) | (g ? 4 : 0) | (b ? 2 : 0) | (a ? 1 : 0); }
void APIENTRY glCullFace(GLenum m) { LREC_I(LOP_CULLFACE, m, 0); FLUSH_VERTICES(); C->cull_mode = m; }
void APIENTRY glFrontFace(GLenum m) { LREC_I(LOP_FRONTFACE, m, 0); FLUSH_VERTICES(); C->front_face = m; }
void APIENTRY glShadeModel(GLenum m) { LREC_I(LOP_SHADEMODEL, m, 0); FLUSH_VERTICES(); C->shade_model = m; }
void APIENTRY glPolygonMode(GLenum face, GLenum m)
{
    LREC_I(LOP_POLYGONMODE, face, m);
    FLUSH_VERTICES();
    if (m != GL_POINT && m != GL_LINE && m != GL_FILL) { SET_ERROR(GL_INVALID_ENUM); return; }
    if (face != GL_FRONT && face != GL_BACK && face != GL_FRONT_AND_BACK) { SET_ERROR(GL_INVALID_ENUM); return; }
    if (face == GL_FRONT || face == GL_FRONT_AND_BACK) C->poly_mode_front = m;
    if (face == GL_BACK || face == GL_FRONT_AND_BACK) C->poly_mode_back = m;
}
void APIENTRY glPolygonOffset(GLfloat f, GLfloat u) { LREC_F(LOP_POLYOFFSET, 0, f, u, 0, 0); FLUSH_VERTICES(); C->poly_factor = f; C->poly_units = u; }
void APIENTRY glLineWidth(GLfloat w) { LREC_F(LOP_LINEWIDTH, 0, w, 0, 0, 0); FLUSH_VERTICES(); if (w <= 0) { SET_ERROR(GL_INVALID_VALUE); return; } C->line_width = w; }
void APIENTRY glPointSize(GLfloat s) { LREC_F(LOP_POINTSIZE, 0, s, 0, 0, 0); FLUSH_VERTICES(); if (s <= 0) { SET_ERROR(GL_INVALID_VALUE); return; } C->point_size = s; }
void APIENTRY glScissor(GLint x, GLint y, GLsizei w, GLsizei h) { FLUSH_VERTICES(); C->sc[0] = x; C->sc[1] = y; C->sc[2] = w; C->sc[3] = h; }
void APIENTRY glViewport(GLint x, GLint y, GLsizei w, GLsizei h) { FLUSH_VERTICES(); C->vp[0] = x; C->vp[1] = y; C->vp[2] = w; C->vp[3] = h; }

static void stencil_func(int f, GLenum fn, GLint ref, GLuint mask) { C->st_func[f] = fn; C->st_ref[f] = ref; C->st_vmask[f] = mask; }
static void stencil_op(int f, GLenum a, GLenum b, GLenum c) { C->st_fail[f] = a; C->st_zfail[f] = b; C->st_zpass[f] = c; }
void APIENTRY glStencilFunc(GLenum fn, GLint ref, GLuint mask) { FLUSH_VERTICES(); stencil_func(0, fn, ref, mask); stencil_func(1, fn, ref, mask); }
void APIENTRY glStencilOp(GLenum a, GLenum b, GLenum c) { FLUSH_VERTICES(); stencil_op(0, a, b, c); stencil_op(1, a, b, c); }
void APIENTRY glStencilMask(GLuint m) { FLUSH_VERTICES(); C->st_wmask[0] = C->st_wmask[1] = m; }
void APIENTRY glStencilFuncSeparate(GLenum face, GLenum fn, GLint ref, GLuint mask)
{
    FLUSH_VERTICES();
    if (face != GL_BACK) stencil_func(0, fn, ref, mask);
    if (face != GL_FRONT) stencil_func(1, fn, ref, mask);
}
void APIENTRY glStencilOpSeparate(GLenum face, GLenum a, GLenum b, GLenum c)
{
    FLUSH_VERTICES();
    if (face != GL_BACK) stencil_op(0, a, b, c);
    if (face != GL_FRONT) stencil_op(1, a, b, c);
}
void APIENTRY glStencilMaskSeparate(GLenum face, GLuint m)
{
    FLUSH_VERTICES();
    if (face != GL_BACK) C->st_wmask[0] = m;
    if (face != GL_FRONT) C->st_wmask[1] = m;
}

void APIENTRY glClearColor(GLclampf r, GLclampf g, GLclampf b, GLclampf a) { C->clear_color[0] = r; C->clear_color[1] = g; C->clear_color[2] = b; C->clear_color[3] = a; }
void APIENTRY glClearDepth(GLclampd d) { C->clear_depth = (float)d; }
void APIENTRY glClearStencil(GLint s) { C->clear_stencil = s; }

void APIENTRY glClear(GLbitfield mask)
{
    uint32_t fl = 0, sc[4];
    double c[4];
    g2r_target t;
    int i;
    FLUSH_VERTICES();
    if (!C->live) return;
    gl_bind_target();
    gl_cur_target(&t);
    if (mask & GL_ACCUM_BUFFER_BIT) {
        int b[4];
        gl_scissor_px(&t, b);
        gl_accum_clear(b[0], b[1], b[2], b[3]);
    }
    if ((mask & GL_COLOR_BUFFER_BIT) && C->color_mask) fl |= G2_FL_CLEAR_COLOR | G2_FL_CMASK(C->color_mask);
    if ((mask & GL_DEPTH_BUFFER_BIT) && C->depth_mask && t.ds) fl |= G2_FL_CLEAR_DEPTH;
    if ((mask & GL_STENCIL_BUFFER_BIT) && t.ds) fl |= G2_FL_CLEAR_STENCIL;
    if (!fl) return;
    if (C->dither) fl |= G2_FL_DITHER;
    if (C->scissor_test) {
        int b[4];
        if (!gl_scissor_px(&t, b)) return;
        sc[0] = (uint32_t)b[0]; sc[1] = (uint32_t)b[1]; sc[2] = (uint32_t)b[2]; sc[3] = (uint32_t)b[3];
        fl |= G2_FL_CLEAR_SCISSOR;
    } else {
        sc[0] = 0; sc[1] = 0; sc[2] = t.width; sc[3] = t.height;
    }
    for (i = 0; i < 4; i++) c[i] = C->clear_color[i];
    {
        double d = C->clear_depth < 0 ? 0 : C->clear_depth > 1 ? 1 : C->clear_depth;
        g2r_clear(D.r, fl, c, (uint32_t)(d * (t.depth16 ? 65535.0 : 16777215.0) + 0.5), (uint32_t)C->clear_stencil & 255, C->st_wmask[0] & 255, sc);
    }
}

void APIENTRY glFlush(void)
{
    FLUSH_VERTICES();
    /* what is drawn to the front buffer has to show: the pass ends and goes out. With a back buffer nothing can be seen before the swap, and a read ends the pass itself; ending it here costs the boards
       a flush of the whole target and its reload for the next draw */
    if (C->live && (!D.doublebuffer || C->draw_buffer == GL_FRONT)) {
        g2r_end_pass(D.r);
        gl_swap();
    }
}
void APIENTRY glFinish(void) { FLUSH_VERTICES(); if (C->live) g2r_finish(D.r); }
void APIENTRY glHint(GLenum t, GLenum m) { (void)t; (void)m; }
void APIENTRY glDrawBuffer(GLenum b) { FLUSH_VERTICES(); C->draw_buffer = b; }
void APIENTRY glReadBuffer(GLenum b) { C->read_buffer = b; }

/* ---------------------------------------------------------------- matrices */
float *gl_top_matrix(GLenum mode)
{
    switch (mode) {
    case GL_PROJECTION: return C->pr[C->pr_sp];
    case GL_TEXTURE: { gl_unit *u = &C->unit[C->active_unit]; return u->mat[u->mat_sp]; }
    default: return C->mv[C->mv_sp];
    }
}
const float *gl_cur_matrix(void) { return gl_top_matrix(C->matrix_mode); }

void gl_mult(const float *t) { float *m = gl_top_matrix(C->matrix_mode); FLUSH_VERTICES(); gl_mul(m, m, t); }

void APIENTRY glMatrixMode(GLenum m) { LREC_I(LOP_MATRIXMODE, m, 0); FLUSH_VERTICES(); C->matrix_mode = m; }
void APIENTRY glLoadIdentity(void) { LREC_I(LOP_LOADI, 0, 0); FLUSH_VERTICES(); gl_ident(gl_top_matrix(C->matrix_mode)); }
void APIENTRY glLoadMatrixf(const GLfloat *m) { if (C->compiling) { gl_list_big(LOP_LOADM, 0, m, 64); if (C->compile_mode == GL_COMPILE) return; } FLUSH_VERTICES(); memcpy(gl_top_matrix(C->matrix_mode), m, 64); }
void APIENTRY glLoadMatrixd(const GLdouble *m) { float f[16]; int i; for (i = 0; i < 16; i++) f[i] = (float)m[i]; glLoadMatrixf(f); }
void APIENTRY glMultMatrixf(const GLfloat *m) { if (C->compiling) { gl_list_big(LOP_MULTM, 0, m, 64); if (C->compile_mode == GL_COMPILE) return; } gl_mult(m); }
void APIENTRY glMultMatrixd(const GLdouble *m) { float f[16]; int i; for (i = 0; i < 16; i++) f[i] = (float)m[i]; gl_mult(f); }
void APIENTRY glLoadTransposeMatrixf(const GLfloat *m) { float f[16]; int r, c; for (r = 0; r < 4; r++) for (c = 0; c < 4; c++) f[c * 4 + r] = m[r * 4 + c]; glLoadMatrixf(f); }
void APIENTRY glMultTransposeMatrixf(const GLfloat *m) { float f[16]; int r, c; for (r = 0; r < 4; r++) for (c = 0; c < 4; c++) f[c * 4 + r] = m[r * 4 + c]; gl_mult(f); }

void APIENTRY glPushMatrix(void)
{
    int *sp;
    float (*st)[16];
    gl_unit *u = &C->unit[C->active_unit];
    LREC_I(LOP_PUSHM, 0, 0);
    FLUSH_VERTICES();
    if (C->matrix_mode == GL_PROJECTION) { sp = &C->pr_sp; st = C->pr; }
    else if (C->matrix_mode == GL_TEXTURE) { sp = &u->mat_sp; st = u->mat; }
    else { sp = &C->mv_sp; st = C->mv; }
    if (*sp >= MS_DEPTH - 1) { SET_ERROR(GL_STACK_OVERFLOW); return; }
    memcpy(st[*sp + 1], st[*sp], 64);
    (*sp)++;
}

void APIENTRY glPopMatrix(void)
{
    int *sp;
    gl_unit *u = &C->unit[C->active_unit];
    LREC_I(LOP_POPM, 0, 0);
    FLUSH_VERTICES();
    sp = C->matrix_mode == GL_PROJECTION ? &C->pr_sp : C->matrix_mode == GL_TEXTURE ? &u->mat_sp : &C->mv_sp;
    if (*sp <= 0) { SET_ERROR(GL_STACK_UNDERFLOW); return; }
    (*sp)--;
}

void APIENTRY glTranslatef(GLfloat x, GLfloat y, GLfloat z) { LREC_F(LOP_TRANSLATE, 0, x, y, z, 0); float t[16]; gl_ident(t); t[12] = x; t[13] = y; t[14] = z; gl_mult(t); }
void APIENTRY glTranslated(GLdouble x, GLdouble y, GLdouble z) { glTranslatef((float)x, (float)y, (float)z); }
void APIENTRY glScalef(GLfloat x, GLfloat y, GLfloat z) { LREC_F(LOP_SCALE, 0, x, y, z, 0); float t[16]; gl_ident(t); t[0] = x; t[5] = y; t[10] = z; gl_mult(t); }
void APIENTRY glScaled(GLdouble x, GLdouble y, GLdouble z) { glScalef((float)x, (float)y, (float)z); }
void APIENTRY glRotatef(GLfloat ang, GLfloat x, GLfloat y, GLfloat z)
{
    float r = ang * 3.14159265358979f / 180.0f, c = cosf(r), s = sinf(r), l = sqrtf(x * x + y * y + z * z), t[16];
    LREC_F(LOP_ROTATE, 0, ang, x, y, z);
    if (l > 0) { x /= l; y /= l; z /= l; }
    gl_ident(t);
    t[0] = x * x * (1 - c) + c; t[1] = y * x * (1 - c) + z * s; t[2] = x * z * (1 - c) - y * s;
    t[4] = x * y * (1 - c) - z * s; t[5] = y * y * (1 - c) + c; t[6] = y * z * (1 - c) + x * s;
    t[8] = x * z * (1 - c) + y * s; t[9] = y * z * (1 - c) - x * s; t[10] = z * z * (1 - c) + c;
    gl_mult(t);
}
void APIENTRY glRotated(GLdouble a, GLdouble x, GLdouble y, GLdouble z) { glRotatef((float)a, (float)x, (float)y, (float)z); }
void APIENTRY glOrtho(GLdouble l, GLdouble r, GLdouble b, GLdouble t, GLdouble n, GLdouble f)
{
    float m[16];
    if (l == r || b == t || n == f) { SET_ERROR(GL_INVALID_VALUE); return; }
    gl_ident(m);
    m[0] = (float)(2 / (r - l)); m[5] = (float)(2 / (t - b)); m[10] = (float)(-2 / (f - n));
    m[12] = (float)(-(r + l) / (r - l)); m[13] = (float)(-(t + b) / (t - b)); m[14] = (float)(-(f + n) / (f - n));
    gl_mult(m);
}
void APIENTRY glFrustum(GLdouble l, GLdouble r, GLdouble b, GLdouble t, GLdouble n, GLdouble f)
{
    float m[16];
    if (n <= 0 || f <= 0 || l == r || b == t || n == f) { SET_ERROR(GL_INVALID_VALUE); return; }
    memset(m, 0, sizeof m);
    m[0] = (float)(2 * n / (r - l)); m[5] = (float)(2 * n / (t - b)); m[8] = (float)((r + l) / (r - l)); m[9] = (float)((t + b) / (t - b));
    m[10] = (float)(-(f + n) / (f - n)); m[11] = -1; m[14] = (float)(-2 * f * n / (f - n));
    gl_mult(m);
}

/* ---------------------------------------------------------------- lights, materials */
static void xform4(const float *m, const float *v, float *o)
{
    float t[4];
    int r;
    for (r = 0; r < 4; r++) t[r] = m[r] * v[0] + m[r + 4] * v[1] + m[r + 8] * v[2] + m[r + 12] * v[3];
    memcpy(o, t, sizeof t);
}

static void light_param(GLenum l, GLenum p, const float *v)
{
    gl_light *o;
    FLUSH_VERTICES();
    if (l < GL_LIGHT0 || l >= GL_LIGHT0 + MAX_LIGHTS) { SET_ERROR(GL_INVALID_ENUM); return; }
    o = &C->light[l - GL_LIGHT0];
    switch (p) {
    case GL_AMBIENT: memcpy(o->ambient, v, 16); break;
    case GL_DIFFUSE: memcpy(o->diffuse, v, 16); break;
    case GL_SPECULAR: memcpy(o->specular, v, 16); break;
    case GL_POSITION: xform4(C->mv[C->mv_sp], v, o->position); break;
    case GL_SPOT_DIRECTION: {
        const float *m = C->mv[C->mv_sp];
        float d[3];
        int r;
        for (r = 0; r < 3; r++) d[r] = m[r] * v[0] + m[r + 4] * v[1] + m[r + 8] * v[2];
        {
            float n = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            if (n > 0) { d[0] /= n; d[1] /= n; d[2] /= n; }
        }
        memcpy(o->spot_dir, d, 12);
        break;
    }
    case GL_SPOT_EXPONENT: o->spot_exp = v[0]; break;
    case GL_SPOT_CUTOFF: o->spot_cut = v[0]; break;
    case GL_CONSTANT_ATTENUATION: o->atten[0] = v[0]; break;
    case GL_LINEAR_ATTENUATION: o->atten[1] = v[0]; break;
    case GL_QUADRATIC_ATTENUATION: o->atten[2] = v[0]; break;
    default: SET_ERROR(GL_INVALID_ENUM); break;
    }
}
void APIENTRY glLightfv(GLenum l, GLenum p, const GLfloat *v) { if (C->compiling) { int big = p == GL_AMBIENT || p == GL_DIFFUSE || p == GL_SPECULAR || p == GL_POSITION || p == GL_SPOT_DIRECTION; gl_list_f4(LOP_LIGHT, l, v[0], big ? v[1] : 0, big ? v[2] : 0, big && p != GL_SPOT_DIRECTION ? v[3] : 0); C->compiling->ops[C->compiling->n - 1].b = p; if (C->compile_mode == GL_COMPILE) return; } light_param(l, p, v); }
void APIENTRY glLightf(GLenum l, GLenum p, GLfloat v) { light_param(l, p, &v); }
void APIENTRY glLighti(GLenum l, GLenum p, GLint v) { float f = (float)v; light_param(l, p, &f); }
void APIENTRY glLightiv(GLenum l, GLenum p, const GLint *v)
{
    float f[4];
    int i, n = p == GL_POSITION || p == GL_SPOT_DIRECTION ? 4 : p == GL_AMBIENT || p == GL_DIFFUSE || p == GL_SPECULAR ? -4 : 1;
    if (n < 0) { for (i = 0; i < 4; i++) f[i] = (float)(v[i] / 2147483647.0); }
    else for (i = 0; i < n; i++) f[i] = (float)v[i];
    light_param(l, p, f);
}
void APIENTRY glGetLightfv(GLenum l, GLenum p, GLfloat *v)
{
    gl_light *o = &C->light[(l - GL_LIGHT0) & 7];
    switch (p) {
    case GL_AMBIENT: memcpy(v, o->ambient, 16); break;
    case GL_DIFFUSE: memcpy(v, o->diffuse, 16); break;
    case GL_SPECULAR: memcpy(v, o->specular, 16); break;
    case GL_POSITION: memcpy(v, o->position, 16); break;
    case GL_SPOT_DIRECTION: memcpy(v, o->spot_dir, 12); break;
    case GL_SPOT_EXPONENT: v[0] = o->spot_exp; break;
    case GL_SPOT_CUTOFF: v[0] = o->spot_cut; break;
    default: break;
    }
}

static void lightmodel(GLenum p, const float *v)
{
    FLUSH_VERTICES();
    switch (p) {
    case GL_LIGHT_MODEL_AMBIENT: memcpy(C->lm_ambient, v, 16); break;
    case GL_LIGHT_MODEL_TWO_SIDE: C->lm_two_side = v[0] != 0.0f; break;
    case GL_LIGHT_MODEL_LOCAL_VIEWER: C->lm_local_viewer = v[0] != 0.0f; break;
    case GL_LIGHT_MODEL_COLOR_CONTROL: C->lm_color_control = (GLenum)v[0]; break;
    default: SET_ERROR(GL_INVALID_ENUM); break;
    }
}
void APIENTRY glLightModelfv(GLenum p, const GLfloat *v) { lightmodel(p, v); }
void APIENTRY glLightModelf(GLenum p, GLfloat v) { lightmodel(p, &v); }
void APIENTRY glLightModeli(GLenum p, GLint v) { float f = (float)v; lightmodel(p, &f); }
void APIENTRY glLightModeliv(GLenum p, const GLint *v)
{
    float f[4];
    int i;
    if (p == GL_LIGHT_MODEL_AMBIENT) for (i = 0; i < 4; i++) f[i] = (float)(v[i] / 2147483647.0);
    else f[0] = (float)v[0];
    lightmodel(p, f);
}

static void material(GLenum face, GLenum p, const float *v)
{
    int f;
    FLUSH_VERTICES();
    for (f = 0; f < 2; f++) {
        gl_material *m = &C->mat[f];
        if ((f == 0 && face == GL_BACK) || (f == 1 && face == GL_FRONT)) continue;
        switch (p) {
        case GL_AMBIENT: memcpy(m->ambient, v, 16); break;
        case GL_DIFFUSE: memcpy(m->diffuse, v, 16); break;
        case GL_AMBIENT_AND_DIFFUSE: memcpy(m->ambient, v, 16); memcpy(m->diffuse, v, 16); break;
        case GL_SPECULAR: memcpy(m->specular, v, 16); break;
        case GL_EMISSION: memcpy(m->emission, v, 16); break;
        case GL_SHININESS: m->shininess = v[0]; break;
        default: SET_ERROR(GL_INVALID_ENUM); break;
        }
    }
}
void APIENTRY glMaterialfv(GLenum f, GLenum p, const GLfloat *v) { if (C->compiling) { gl_list_f4(LOP_MATERIAL, f, v[0], p == GL_SHININESS ? 0 : v[1], p == GL_SHININESS ? 0 : v[2], p == GL_SHININESS ? 0 : v[3]); C->compiling->ops[C->compiling->n - 1].b = p; if (C->compile_mode == GL_COMPILE) return; } material(f, p, v); }
void APIENTRY glMaterialf(GLenum f, GLenum p, GLfloat v) { material(f, p, &v); }
void APIENTRY glMateriali(GLenum f, GLenum p, GLint v) { float x = (float)v; material(f, p, &x); }
void APIENTRY glMaterialiv(GLenum f, GLenum p, const GLint *v)
{
    float x[4];
    int i;
    if (p == GL_SHININESS) x[0] = (float)v[0];
    else for (i = 0; i < 4; i++) x[i] = (float)(v[i] / 2147483647.0);
    material(f, p, x);
}
void APIENTRY glColorMaterial(GLenum f, GLenum m) { FLUSH_VERTICES(); C->cm_face = f; C->cm_mode = m; }

/* ---------------------------------------------------------------- fog, clip planes */
static void fog(GLenum p, const float *v)
{
    FLUSH_VERTICES();
    switch (p) {
    case GL_FOG_MODE: C->fog_mode = (GLenum)v[0]; break;
    case GL_FOG_DENSITY: C->fog_density = v[0]; break;
    case GL_FOG_START: C->fog_start = v[0]; break;
    case GL_FOG_END: C->fog_end = v[0]; break;
    case GL_FOG_COLOR: memcpy(C->fog_color, v, 16); break;
    case GL_FOG_COORD_SRC: C->fog_src = (GLenum)v[0]; break;
    default: break;
    }
}
void APIENTRY glFogfv(GLenum p, const GLfloat *v) { fog(p, v); }
void APIENTRY glFogf(GLenum p, GLfloat v) { fog(p, &v); }
void APIENTRY glFogi(GLenum p, GLint v) { float f = (float)v; fog(p, &f); }
void APIENTRY glFogiv(GLenum p, const GLint *v)
{
    float f[4];
    int i;
    if (p == GL_FOG_COLOR) for (i = 0; i < 4; i++) f[i] = (float)(v[i] / 2147483647.0);
    else f[0] = (float)v[0];
    fog(p, f);
}

void APIENTRY glClipPlane(GLenum p, const GLdouble *e)
{
    float inv[16], tr[16], v[4], o[4];
    int r, c;
    FLUSH_VERTICES();
    if (p < GL_CLIP_PLANE0 || p >= GL_CLIP_PLANE0 + MAX_CLIP) { SET_ERROR(GL_INVALID_ENUM); return; }
    for (r = 0; r < 4; r++) v[r] = (float)e[r];
    if (!gl_invert(inv, C->mv[C->mv_sp])) return;
    for (r = 0; r < 4; r++) for (c = 0; c < 4; c++) tr[c * 4 + r] = inv[r * 4 + c];    /* a plane transforms by the inverse transpose */
    xform4(tr, v, o);
    memcpy(C->clip_plane[p - GL_CLIP_PLANE0], o, 16);
}
void APIENTRY glGetClipPlane(GLenum p, GLdouble *e) { int i; for (i = 0; i < 4; i++) e[i] = C->clip_plane[(p - GL_CLIP_PLANE0) % MAX_CLIP][i]; }

/* ---------------------------------------------------------------- texture environment, texgen */
static void tex_env(GLenum target, GLenum p, const float *v)
{
    gl_unit *u = &C->unit[C->active_unit];
    FLUSH_VERTICES();
    if (target == GL_TEXTURE_FILTER_CONTROL) { if (p == GL_TEXTURE_LOD_BIAS) u->lod_bias = v[0]; return; }
    if (target == GL_POINT_SPRITE) { if (p == GL_COORD_REPLACE) u->coord_replace = v[0] != 0.0f; return; }
    switch (p) {
    case GL_TEXTURE_ENV_MODE: u->env_mode = (GLenum)v[0]; break;
    case GL_TEXTURE_ENV_COLOR: memcpy(u->env_color, v, 16); break;
    case GL_COMBINE_RGB: u->comb_rgb = (GLenum)v[0]; break;
    case GL_COMBINE_ALPHA: u->comb_a = (GLenum)v[0]; break;
    case GL_SRC0_RGB: case GL_SRC1_RGB: case GL_SRC2_RGB: u->src_rgb[p - GL_SRC0_RGB] = (GLenum)v[0]; break;
    case GL_SRC0_ALPHA: case GL_SRC1_ALPHA: case GL_SRC2_ALPHA: u->src_a[p - GL_SRC0_ALPHA] = (GLenum)v[0]; break;
    case GL_OPERAND0_RGB: case GL_OPERAND1_RGB: case GL_OPERAND2_RGB: u->op_rgb[p - GL_OPERAND0_RGB] = (GLenum)v[0]; break;
    case GL_OPERAND0_ALPHA: case GL_OPERAND1_ALPHA: case GL_OPERAND2_ALPHA: u->op_a[p - GL_OPERAND0_ALPHA] = (GLenum)v[0]; break;
    case GL_RGB_SCALE: u->scale_rgb = v[0]; break;
    case GL_ALPHA_SCALE: u->scale_a = v[0]; break;
    default: break;
    }
}
void APIENTRY glTexEnvfv(GLenum t, GLenum p, const GLfloat *v) { tex_env(t, p, v); }
void APIENTRY glTexEnvf(GLenum t, GLenum p, GLfloat v) { if (C->compiling) { gl_list_f4(LOP_TEXENVF, t, v, 0, 0, 0); C->compiling->ops[C->compiling->n - 1].b = p; if (C->compile_mode == GL_COMPILE) return; } tex_env(t, p, &v); }
void APIENTRY glTexEnvi(GLenum t, GLenum p, GLint v) { float f = (float)v; tex_env(t, p, &f); }
void APIENTRY glTexEnviv(GLenum t, GLenum p, const GLint *v)
{
    float f[4];
    int i;
    if (p == GL_TEXTURE_ENV_COLOR) for (i = 0; i < 4; i++) f[i] = (float)(v[i] / 2147483647.0);
    else f[0] = (float)v[0];
    tex_env(t, p, f);
}
void APIENTRY glGetTexEnviv(GLenum t, GLenum p, GLint *v) { if (t == GL_POINT_SPRITE && p == GL_COORD_REPLACE) *v = C->unit[C->active_unit].coord_replace; else if (p == GL_TEXTURE_ENV_MODE) *v = (GLint)C->unit[C->active_unit].env_mode; }
void APIENTRY glGetTexEnvfv(GLenum t, GLenum p, GLfloat *v) { (void)t; if (p == GL_TEXTURE_ENV_COLOR) memcpy(v, C->unit[C->active_unit].env_color, 16); else *v = (GLfloat)C->unit[C->active_unit].env_mode; }

static void tex_gen(GLenum coord, GLenum p, const float *v)
{
    gl_unit *u = &C->unit[C->active_unit];
    int c = coord == GL_S ? 0 : coord == GL_T ? 1 : coord == GL_R ? 2 : 3;
    FLUSH_VERTICES();
    switch (p) {
    case GL_TEXTURE_GEN_MODE: u->gen_mode[c] = (GLenum)v[0]; break;
    case GL_OBJECT_PLANE: memcpy(u->gen_obj[c], v, 16); break;
    case GL_EYE_PLANE: {
        float inv[16], tr[16];
        int r, k;
        if (gl_invert(inv, C->mv[C->mv_sp])) {
            for (r = 0; r < 4; r++) for (k = 0; k < 4; k++) tr[k * 4 + r] = inv[r * 4 + k];
            xform4(tr, v, u->gen_eye[c]);
        }
        break;
    }
    default: break;
    }
}
void APIENTRY glTexGenfv(GLenum c, GLenum p, const GLfloat *v) { tex_gen(c, p, v); }
void APIENTRY glTexGenf(GLenum c, GLenum p, GLfloat v) { tex_gen(c, p, &v); }
void APIENTRY glTexGeni(GLenum c, GLenum p, GLint v) { float f = (float)v; tex_gen(c, p, &f); }
void APIENTRY glTexGend(GLenum c, GLenum p, GLdouble v) { float f = (float)v; tex_gen(c, p, &f); }
void APIENTRY glTexGeniv(GLenum c, GLenum p, const GLint *v) { float f[4]; int i; for (i = 0; i < (p == GL_TEXTURE_GEN_MODE ? 1 : 4); i++) f[i] = (float)v[i]; tex_gen(c, p, f); }
void APIENTRY glTexGendv(GLenum c, GLenum p, const GLdouble *v) { float f[4]; int i; for (i = 0; i < (p == GL_TEXTURE_GEN_MODE ? 1 : 4); i++) f[i] = (float)v[i]; tex_gen(c, p, f); }

/* ---------------------------------------------------------------- pixel store */
void APIENTRY glPixelStorei(GLenum p, GLint v)
{
    switch (p) {
    case GL_UNPACK_ALIGNMENT: C->un_align = v; break;
    case GL_UNPACK_ROW_LENGTH: C->un_row = v; break;
    case GL_UNPACK_IMAGE_HEIGHT: C->un_image_h = v; break;
    case GL_UNPACK_SKIP_IMAGES: C->un_skip_images = v; break;
    case GL_PACK_IMAGE_HEIGHT: C->pk_image_h = v; break;
    case GL_PACK_SKIP_IMAGES: C->pk_skip_images = v; break;
    case GL_UNPACK_SKIP_PIXELS: C->un_skip_px = v; break;
    case GL_UNPACK_SKIP_ROWS: C->un_skip_row = v; break;
    case GL_PACK_ALIGNMENT: C->pk_align = v; break;
    case GL_PACK_ROW_LENGTH: C->pk_row = v; break;
    case GL_PACK_SKIP_PIXELS: C->pk_skip_px = v; break;
    case GL_PACK_SKIP_ROWS: C->pk_skip_row = v; break;
    default: break;
    }
}
void APIENTRY glPixelStoref(GLenum p, GLfloat v) { glPixelStorei(p, (GLint)v); }
void APIENTRY glPixelZoom(GLfloat x, GLfloat y) { C->pixel_zoom[0] = x; C->pixel_zoom[1] = y; }

/* ---------------------------------------------------------------- the attribute stack: a snapshot per glPushAttrib, restored group by group */
#define ATTR_DEPTH 16
typedef struct attr_entry { GLbitfield mask; gl_ctx s; } attr_entry;
static attr_entry *g_attr[ATTR_DEPTH];
static int g_attr_sp;

void APIENTRY glPushAttrib(GLbitfield mask)
{
    attr_entry *e;
    FLUSH_VERTICES();
    if (g_attr_sp >= ATTR_DEPTH) { SET_ERROR(GL_STACK_OVERFLOW); return; }
    e = (attr_entry *)malloc(sizeof *e);
    if (!e) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    e->mask = mask;
    e->s = *C;
    g_attr[g_attr_sp++] = e;
}

#define R(f) memcpy(&C->f, &s->f, sizeof C->f)
void APIENTRY glPopAttrib(void)
{
    attr_entry *e;
    const gl_ctx *s;
    GLbitfield m;
    int u;
    FLUSH_VERTICES();
    if (g_attr_sp <= 0) { SET_ERROR(GL_STACK_UNDERFLOW); return; }
    e = g_attr[--g_attr_sp];
    s = &e->s; m = e->mask;
    if (m & GL_CURRENT_BIT) { R(attr); R(color); R(normal); R(raster_pos); R(raster_valid); R(raster_color); R(raster_tc); R(raster_dist); R(edge_flag); for (u = 0; u < MAX_UNITS; u++) memcpy(C->unit[u].tc, s->unit[u].tc, 16); }
    if (m & GL_ENABLE_BIT) {
        R(vp_point_size); R(vp_two_side); R(point_sprite);
        R(depth_test); R(cull_face); R(blend); R(scissor_test); R(alpha_test); R(stencil_test); R(fog); R(lighting); R(color_material); R(normalize); R(rescale_normal); R(dither);
        R(poly_offset_fill); R(poly_offset_line); R(poly_offset_point); R(clip_en); R(color_logic_op); R(line_smooth); R(point_smooth); R(polygon_smooth); R(line_stipple); R(polygon_stipple);
        for (u = 0; u < MAX_LIGHTS; u++) C->light[u].enabled = s->light[u].enabled;
        R(map_en); R(auto_normal);
        for (u = 0; u < MAX_UNITS; u++) { C->unit[u].en_1d = s->unit[u].en_1d; C->unit[u].en_2d = s->unit[u].en_2d; C->unit[u].en_cube = s->unit[u].en_cube; C->unit[u].en_3d = s->unit[u].en_3d; memcpy(C->unit[u].gen_en, s->unit[u].gen_en, sizeof C->unit[u].gen_en); }
    }
    if (m & GL_COLOR_BUFFER_BIT) {
        R(alpha_test); R(alpha_func); R(alpha_ref); R(blend); R(blend_src); R(blend_dst); R(blend_src_a); R(blend_dst_a); R(blend_eq); R(blend_eq_a); R(blend_color); R(dither); R(color_mask);
        R(clear_color); R(draw_buffer); R(color_logic_op); R(logic_op);
    }
    if (m & GL_ACCUM_BUFFER_BIT) R(accum_clear);
    if (m & GL_PIXEL_MODE_BIT) { R(px_scale); R(px_bias); R(px_shift); R(px_offset); R(px_map_color); R(px_active); R(px_map_stencil); R(px_depth_scale); R(px_depth_bias); R(pixel_zoom); R(read_buffer); }
    if (m & GL_EVAL_BIT) { R(map_en); R(auto_normal); R(grid1_n); R(grid2_n); R(grid1); R(grid2); }
    if (m & GL_POLYGON_STIPPLE_BIT) R(poly_pattern);
    if (m & GL_DEPTH_BUFFER_BIT) { R(depth_test); R(depth_func); R(depth_mask); R(clear_depth); }
    if (m & GL_STENCIL_BUFFER_BIT) { R(stencil_test); R(st_func); R(st_fail); R(st_zfail); R(st_zpass); R(st_ref); R(st_vmask); R(st_wmask); R(clear_stencil); }
    if (m & GL_FOG_BIT) { R(fog); R(fog_mode); R(fog_density); R(fog_start); R(fog_end); R(fog_color); }
    if (m & GL_LIGHTING_BIT) { R(lighting); R(light); R(mat); R(lm_ambient); R(lm_two_side); R(lm_local_viewer); R(lm_color_control); R(cm_face); R(cm_mode); R(color_material); R(shade_model); }
    if (m & GL_POLYGON_BIT) { R(cull_face); R(cull_mode); R(front_face); R(poly_mode_front); R(poly_mode_back); R(poly_factor); R(poly_units); R(poly_offset_fill); R(poly_offset_line); R(poly_offset_point); R(polygon_stipple); }
    if (m & GL_LINE_BIT) { R(line_width); R(line_smooth); R(line_stipple); R(line_factor); R(line_pattern); }
    if (m & GL_POINT_BIT) { R(point_size); R(point_smooth); R(point_sprite); R(sprite_origin); R(pt_min); R(pt_max); R(pt_fade); R(pt_att); for (u = 0; u < MAX_UNITS; u++) C->unit[u].coord_replace = s->unit[u].coord_replace; }
    if (m & GL_SCISSOR_BIT) { R(scissor_test); R(sc); }
    if (m & GL_VIEWPORT_BIT) { R(vp); R(depth_near); R(depth_far); }
    if (m & GL_TRANSFORM_BIT) { R(matrix_mode); R(normalize); R(rescale_normal); R(clip_en); R(clip_plane); }
    if (m & GL_TEXTURE_BIT) {
        R(active_unit);
        for (u = 0; u < MAX_UNITS; u++) {
            gl_unit *d = &C->unit[u];
            const gl_unit *o = &s->unit[u];
            d->en_1d = o->en_1d; d->en_2d = o->en_2d; d->en_cube = o->en_cube; d->en_3d = o->en_3d; d->bound_1d = o->bound_1d; d->bound_2d = o->bound_2d; d->bound_cube = o->bound_cube; d->bound_3d = o->bound_3d; d->env_mode = o->env_mode; memcpy(d->env_color, o->env_color, 16);
            d->comb_rgb = o->comb_rgb; d->comb_a = o->comb_a; memcpy(d->src_rgb, o->src_rgb, sizeof d->src_rgb); memcpy(d->src_a, o->src_a, sizeof d->src_a); memcpy(d->op_rgb, o->op_rgb, sizeof d->op_rgb);
            memcpy(d->op_a, o->op_a, sizeof d->op_a); d->scale_rgb = o->scale_rgb; d->scale_a = o->scale_a; memcpy(d->gen_en, o->gen_en, sizeof d->gen_en); memcpy(d->gen_mode, o->gen_mode, sizeof d->gen_mode);
            memcpy(d->gen_obj, o->gen_obj, sizeof d->gen_obj); memcpy(d->gen_eye, o->gen_eye, sizeof d->gen_eye);
        }
    }
    free(e);
}
#undef R


/* ---------------------------------------------------------------- queries */
GLenum APIENTRY glGetError(void) { GLenum e = C->error; C->error = GL_NO_ERROR; return e; }

const GLubyte *APIENTRY glGetString(GLenum n)
{
    switch (n) {
    case GL_VENDOR: return (const GLubyte *)"gpu2";
    case GL_RENDERER: return (const GLubyte *)"GPU2 two-board FPGA GPU";
    case GL_VERSION: return (const GLubyte *)"2.0.0 gpu2";
    case GL_SHADING_LANGUAGE_VERSION: return (const GLubyte *)"1.10 gpu2";
    case GL_EXTENSIONS:
        return (const GLubyte *)"GL_ARB_multitexture GL_ARB_texture_env_add GL_ARB_texture_env_combine GL_ARB_texture_env_crossbar GL_ARB_texture_env_dot3 GL_ARB_texture_cube_map GL_ARB_vertex_buffer_object "
                                "GL_ARB_transpose_matrix GL_ARB_texture_mirrored_repeat GL_EXT_texture_env_add GL_EXT_texture_env_combine GL_EXT_texture_edge_clamp GL_SGIS_texture_edge_clamp GL_EXT_compiled_vertex_array "
                                "GL_EXT_draw_range_elements GL_EXT_framebuffer_object GL_EXT_bgra GL_EXT_blend_color GL_EXT_blend_minmax GL_EXT_blend_subtract GL_EXT_blend_func_separate GL_EXT_rescale_normal "
                                "GL_EXT_separate_specular_color GL_SGIS_generate_mipmap GL_EXT_stencil_wrap GL_ARB_texture_border_clamp GL_ARB_texture_compression GL_EXT_texture_compression_s3tc GL_ARB_multisample "
                                "GL_ARB_point_parameters GL_EXT_point_parameters GL_EXT_fog_coord GL_EXT_secondary_color GL_EXT_multi_draw_arrays GL_ARB_window_pos GL_SGIS_texture_lod GL_EXT_texture_lod_bias GL_ARB_depth_texture GL_ARB_shadow GL_EXT_shadow_funcs "
                                "GL_EXT_texture3D GL_ARB_texture_non_power_of_two GL_ARB_shader_objects GL_ARB_vertex_shader GL_ARB_fragment_shader GL_ARB_shading_language_100 GL_ARB_point_sprite GL_ARB_draw_buffers "
                                "GL_EXT_framebuffer_blit GL_EXT_packed_depth_stencil GL_ARB_pixel_buffer_object GL_EXT_pixel_buffer_object GL_EXT_blend_equation_separate GL_ARB_shader_texture_lod WGL_EXT_swap_control";
    default: SET_ERROR(GL_INVALID_ENUM); return NULL;
    }
}

/* integer-valued state; returns the number of values, 0: not one of these */
static int get_ints(GLenum p, GLint *v)
{
    switch (p) {
    case GL_MAX_TEXTURE_SIZE: case GL_MAX_CUBE_MAP_TEXTURE_SIZE: *v = MAX_TEX_SIZE; return 1;
    case GL_MAX_TEXTURE_UNITS: *v = MAX_UNITS; return 1;
    case GL_MAX_LIGHTS: *v = MAX_LIGHTS; return 1;
    case GL_MAX_CLIP_PLANES: *v = MAX_CLIP; return 1;
    case GL_MAX_MODELVIEW_STACK_DEPTH: case GL_MAX_PROJECTION_STACK_DEPTH: case GL_MAX_TEXTURE_STACK_DEPTH: *v = MS_DEPTH; return 1;
    case GL_MAX_ATTRIB_STACK_DEPTH: *v = ATTR_DEPTH; return 1;
    case GL_MAX_VIEWPORT_DIMS: v[0] = 2048; v[1] = 2048; return 2;
    case GL_MAX_ELEMENTS_VERTICES: case GL_MAX_ELEMENTS_INDICES: *v = 65536; return 1;
    case GL_MAX_RENDERBUFFER_SIZE_EXT: *v = 1280; return 1;
    case GL_MAX_COLOR_ATTACHMENTS_EXT: *v = 1; return 1;
    case GL_VIEWPORT: memcpy(v, C->vp, 16); return 4;
    case GL_SCISSOR_BOX: memcpy(v, C->sc, 16); return 4;
    case GL_MATRIX_MODE: *v = (GLint)C->matrix_mode; return 1;
    case GL_MODELVIEW_STACK_DEPTH: *v = C->mv_sp + 1; return 1;
    case GL_PROJECTION_STACK_DEPTH: *v = C->pr_sp + 1; return 1;
    case GL_TEXTURE_STACK_DEPTH: *v = C->unit[C->active_unit].mat_sp + 1; return 1;
    case GL_ACTIVE_TEXTURE: *v = GL_TEXTURE0 + C->active_unit; return 1;
    case GL_CLIENT_ACTIVE_TEXTURE: *v = GL_TEXTURE0 + C->client_unit; return 1;
    case GL_TEXTURE_BINDING_2D: *v = (GLint)C->unit[C->active_unit].bound_2d; return 1;
    case GL_TEXTURE_BINDING_CUBE_MAP: *v = (GLint)C->unit[C->active_unit].bound_cube; return 1;
    case GL_TEXTURE_BINDING_3D: *v = (GLint)C->unit[C->active_unit].bound_3d; return 1;
    case GL_MAX_3D_TEXTURE_SIZE: *v = MAX_3D_SIZE; return 1;
    case GL_CURRENT_PROGRAM: *v = (GLint)C->program; return 1;
    case GL_BLEND_EQUATION_ALPHA: *v = (GLint)C->blend_eq_a; return 1;
    case GL_STENCIL_BACK_FUNC: *v = (GLint)C->st_func[1]; return 1;
    case GL_STENCIL_BACK_FAIL: *v = (GLint)C->st_fail[1]; return 1;
    case GL_STENCIL_BACK_PASS_DEPTH_FAIL: *v = (GLint)C->st_zfail[1]; return 1;
    case GL_STENCIL_BACK_PASS_DEPTH_PASS: *v = (GLint)C->st_zpass[1]; return 1;
    case GL_STENCIL_BACK_REF: *v = C->st_ref[1]; return 1;
    case GL_STENCIL_BACK_VALUE_MASK: *v = (GLint)C->st_vmask[1]; return 1;
    case GL_STENCIL_BACK_WRITEMASK: *v = (GLint)C->st_wmask[1]; return 1;
    case GL_MAX_VERTEX_ATTRIBS: *v = MAX_ATTRIBS; return 1;
    case GL_MAX_VERTEX_UNIFORM_COMPONENTS: *v = 896; return 1;
    case GL_MAX_FRAGMENT_UNIFORM_COMPONENTS: *v = 192; return 1;
    case GL_MAX_VARYING_FLOATS: *v = 32; return 1;
    case GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS: *v = 0; return 1;
    case GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS: case GL_MAX_TEXTURE_IMAGE_UNITS: case GL_MAX_TEXTURE_COORDS: *v = MAX_UNITS; return 1;
    case GL_MAX_DRAW_BUFFERS: *v = 1; return 1;
    case GL_DRAW_BUFFER0: *v = (GLint)C->draw_buffer; return 1;
    case GL_POINT_SPRITE_COORD_ORIGIN: *v = (GLint)C->sprite_origin; return 1;
    case GL_UNPACK_IMAGE_HEIGHT: *v = C->un_image_h; return 1;
    case GL_UNPACK_SKIP_IMAGES: *v = C->un_skip_images; return 1;
    case GL_PACK_IMAGE_HEIGHT: *v = C->pk_image_h; return 1;
    case GL_PACK_SKIP_IMAGES: *v = C->pk_skip_images; return 1;
    case GL_RED_BITS: case GL_GREEN_BITS: case GL_BLUE_BITS: case GL_ALPHA_BITS: case GL_DEPTH_BITS: case GL_STENCIL_BITS: {      /* of the framebuffer drawn to */
        g2r_target t;
        int dd = D.has_depth ? (D.depth16 ? 16 : 24) : 0, ss = D.has_stencil ? 8 : 0;
        gl_cur_target(&t);
        if (C->fbo) gl_fbo_ds_bits(C->fbo, &dd, &ss);
        *v = p == GL_RED_BITS ? t.bits_r : p == GL_GREEN_BITS ? t.bits_g : p == GL_BLUE_BITS ? t.bits_b : p == GL_ALPHA_BITS ? t.bits_a : p == GL_DEPTH_BITS ? dd : ss;
        return 1;
    }
    case GL_DOUBLEBUFFER: *v = D.doublebuffer; return 1;
    case GL_DEPTH_FUNC: *v = (GLint)C->depth_func; return 1;
    case GL_DEPTH_WRITEMASK: *v = C->depth_mask; return 1;
    case GL_BLEND_SRC: *v = (GLint)C->blend_src; return 1;
    case GL_BLEND_DST: *v = (GLint)C->blend_dst; return 1;
    case GL_ALPHA_TEST_FUNC: *v = (GLint)C->alpha_func; return 1;
    case GL_CULL_FACE_MODE: *v = (GLint)C->cull_mode; return 1;
    case GL_FRONT_FACE: *v = (GLint)C->front_face; return 1;
    case GL_SHADE_MODEL: *v = (GLint)C->shade_model; return 1;
    case GL_ARRAY_BUFFER_BINDING: *v = (GLint)C->bound_array; return 1;
    case GL_PIXEL_PACK_BUFFER_BINDING: *v = (GLint)C->bound_pack; return 1;
    case GL_PIXEL_UNPACK_BUFFER_BINDING: *v = (GLint)C->bound_unpack; return 1;
    case GL_READ_FRAMEBUFFER_BINDING_EXT: *v = (GLint)C->read_fbo; return 1;
    case GL_ELEMENT_ARRAY_BUFFER_BINDING: *v = (GLint)C->bound_element; return 1;
    case GL_FRAMEBUFFER_BINDING_EXT: *v = (GLint)C->fbo; return 1;
    case GL_RENDERBUFFER_BINDING_EXT: *v = (GLint)C->renderbuffer; return 1;
    case GL_UNPACK_ALIGNMENT: *v = C->un_align; return 1;
    case GL_UNPACK_ROW_LENGTH: *v = C->un_row; return 1;
    case GL_UNPACK_SKIP_ROWS: *v = C->un_skip_row; return 1;
    case GL_UNPACK_SKIP_PIXELS: *v = C->un_skip_px; return 1;
    case GL_PACK_ALIGNMENT: *v = C->pk_align; return 1;
    case GL_PACK_ROW_LENGTH: *v = C->pk_row; return 1;
    case GL_PACK_SKIP_ROWS: *v = C->pk_skip_row; return 1;
    case GL_PACK_SKIP_PIXELS: *v = C->pk_skip_px; return 1;
    case GL_POLYGON_MODE: v[0] = (GLint)C->poly_mode_front; v[1] = (GLint)C->poly_mode_back; return 2;
    case GL_EDGE_FLAG: *v = C->edge_flag; return 1;
    case GL_EDGE_FLAG_ARRAY_STRIDE: *v = C->a_edge.stride; return 1;
    case GL_CURRENT_RASTER_POSITION_VALID: *v = C->raster_valid; return 1;
    case GL_DRAW_BUFFER: *v = (GLint)C->draw_buffer; return 1;
    case GL_READ_BUFFER: *v = (GLint)C->read_buffer; return 1;
    case GL_LIST_BASE: *v = (GLint)C->list_base; return 1;
    case GL_NUM_COMPRESSED_TEXTURE_FORMATS: *v = 4; return 1;
    case GL_COMPRESSED_TEXTURE_FORMATS: v[0] = GL_COMPRESSED_RGB_S3TC_DXT1_EXT; v[1] = GL_COMPRESSED_RGBA_S3TC_DXT1_EXT; v[2] = GL_COMPRESSED_RGBA_S3TC_DXT3_EXT; v[3] = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT; return 4;
    case GL_SAMPLE_BUFFERS: case GL_SAMPLES: *v = 0; return 1;
    case GL_FOG_COORD_SRC: *v = (GLint)C->fog_src; return 1;
    case GL_FOG_COORD_ARRAY_TYPE: *v = (GLint)C->a_fog.type; return 1;
    case GL_FOG_COORD_ARRAY_STRIDE: *v = C->a_fog.stride; return 1;
    case GL_SECONDARY_COLOR_ARRAY_SIZE: *v = C->a_sec.size; return 1;
    case GL_SECONDARY_COLOR_ARRAY_TYPE: *v = (GLint)C->a_sec.type; return 1;
    case GL_SECONDARY_COLOR_ARRAY_STRIDE: *v = C->a_sec.stride; return 1;
    case GL_MAX_TEXTURE_LOD_BIAS: *v = 15; return 1;
    default: return gl_extra_ints(p, v);
    }
}

static int get_floats(GLenum p, float *v)
{
    switch (p) {
    case GL_MODELVIEW_MATRIX: memcpy(v, C->mv[C->mv_sp], 64); return 16;
    case GL_PROJECTION_MATRIX: memcpy(v, C->pr[C->pr_sp], 64); return 16;
    case GL_TEXTURE_MATRIX: memcpy(v, C->unit[C->active_unit].mat[C->unit[C->active_unit].mat_sp], 64); return 16;
    case GL_CURRENT_COLOR: memcpy(v, C->color, 16); return 4;
    case GL_CURRENT_RASTER_POSITION: memcpy(v, C->raster_pos, 16); return 4;
    case GL_CURRENT_RASTER_COLOR: memcpy(v, C->raster_color, 16); return 4;
    case GL_CURRENT_RASTER_TEXTURE_COORDS: memcpy(v, C->raster_tc, 16); return 4;
    case GL_CURRENT_RASTER_DISTANCE: *v = C->raster_dist; return 1;
    case GL_DEPTH_SCALE: *v = C->px_depth_scale; return 1;
    case GL_DEPTH_BIAS: *v = C->px_depth_bias; return 1;
    case GL_CURRENT_NORMAL: memcpy(v, C->normal, 12); return 3;
    case GL_CURRENT_SECONDARY_COLOR: memcpy(v, C->sec_color, 16); return 4;
    case GL_CURRENT_FOG_COORD: *v = C->fog_coord; return 1;
    case GL_SAMPLE_COVERAGE_VALUE: *v = C->sample_val; return 1;
    case GL_SAMPLE_COVERAGE_INVERT: *v = (float)C->sample_inv; return 1;
    case GL_POINT_SIZE_MIN: *v = C->pt_min; return 1;
    case GL_POINT_SIZE_MAX: *v = C->pt_max; return 1;
    case GL_POINT_FADE_THRESHOLD_SIZE: *v = C->pt_fade; return 1;
    case GL_POINT_DISTANCE_ATTENUATION: memcpy(v, C->pt_att, 12); return 3;
    case GL_CURRENT_TEXTURE_COORDS: memcpy(v, C->unit[C->active_unit].tc, 16); return 4;
    case GL_COLOR_CLEAR_VALUE: memcpy(v, C->clear_color, 16); return 4;
    case GL_DEPTH_CLEAR_VALUE: *v = C->clear_depth; return 1;
    case GL_DEPTH_RANGE: v[0] = C->depth_near; v[1] = C->depth_far; return 2;
    case GL_ALPHA_TEST_REF: *v = C->alpha_ref; return 1;
    case GL_FOG_COLOR: memcpy(v, C->fog_color, 16); return 4;
    case GL_FOG_DENSITY: *v = C->fog_density; return 1;
    case GL_FOG_START: *v = C->fog_start; return 1;
    case GL_FOG_END: *v = C->fog_end; return 1;
    case GL_LIGHT_MODEL_AMBIENT: memcpy(v, C->lm_ambient, 16); return 4;
    case GL_LINE_WIDTH: *v = C->line_width; return 1;
    case GL_POINT_SIZE: *v = C->point_size; return 1;
    case GL_ALIASED_LINE_WIDTH_RANGE: case GL_ALIASED_POINT_SIZE_RANGE: case GL_LINE_WIDTH_RANGE: case GL_POINT_SIZE_RANGE: v[0] = 1; v[1] = 64; return 2;
    case GL_POLYGON_OFFSET_FACTOR: *v = C->poly_factor; return 1;
    case GL_POLYGON_OFFSET_UNITS: *v = C->poly_units; return 1;
    case GL_CURRENT_INDEX: *v = C->cur_index; return 1;
    case GL_ACCUM_CLEAR_VALUE: memcpy(v, C->accum_clear, 16); return 4;
    case GL_MAP1_GRID_DOMAIN: memcpy(v, C->grid1, 8); return 2;
    case GL_MAP2_GRID_DOMAIN: memcpy(v, C->grid2, 16); return 4;
    default: return 0;
    }
}

void APIENTRY glGetIntegerv(GLenum p, GLint *v)
{
    float f[16];
    int n, i;
    if (get_ints(p, v)) return;
    if ((n = get_floats(p, f))) { for (i = 0; i < n; i++) v[i] = (GLint)f[i]; return; }
    *v = glIsEnabled(p);
}

void APIENTRY glGetFloatv(GLenum p, GLfloat *v)
{
    GLint i[8];
    int n, k;
    if (get_floats(p, v)) return;
    if ((n = get_ints(p, i))) { for (k = 0; k < n; k++) v[k] = (GLfloat)i[k]; return; }
    *v = glIsEnabled(p);
}

void APIENTRY glGetDoublev(GLenum p, GLdouble *v)
{
    float f[16];
    GLint i[8];
    int n, k;
    if ((n = get_floats(p, f))) { for (k = 0; k < n; k++) v[k] = f[k]; return; }
    if ((n = get_ints(p, i))) { for (k = 0; k < n; k++) v[k] = i[k]; return; }
    *v = glIsEnabled(p);
}

void APIENTRY glGetBooleanv(GLenum p, GLboolean *v)
{
    GLint i[8];
    int n, k;
    if ((n = get_ints(p, i))) { for (k = 0; k < n; k++) v[k] = i[k] != 0; return; }
    *v = glIsEnabled(p);
}
