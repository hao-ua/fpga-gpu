/* display lists: the calls are recorded and replayed. Immediate-mode attributes are recorded as they are set, ahead of the vertex that uses them, so a list that sets no colour
 * still uses the colour current when it is called. Also here: small queries, and wglUseFontBitmaps, which is display lists + glBitmap. */
#include "gl_internal.h"

void gl_begin(GLenum mode);
void gl_end(void);
void gl_vertex(float x, float y, float z, float w);

#define L_BUCKETS 1024
typedef struct lnode { gl_list l; struct lnode *next; } lnode;
static lnode *g_lists[L_BUCKETS];
static GLuint g_next_list = 1;
static float g_saved_color[4], g_saved_normal[3], g_saved_tc[MAX_UNITS][4];

static gl_list *list_of(GLuint name, int create)
{
    lnode *n;
    for (n = g_lists[name % L_BUCKETS]; n; n = n->next) if (n->l.name == name) return &n->l;
    if (!create) return NULL;
    n = (lnode *)calloc(1, sizeof *n);
    if (!n) return NULL;
    n->l.name = name;
    n->next = g_lists[name % L_BUCKETS];
    g_lists[name % L_BUCKETS] = n;
    return &n->l;
}

static gl_listop *add(int kind)
{
    gl_list *l = C->compiling;
    gl_listop *o;
    if (l->n == l->cap) {
        l->cap = l->cap ? l->cap * 2 : 64;
        l->ops = (gl_listop *)realloc(l->ops, (size_t)l->cap * sizeof *l->ops);
    }
    o = &l->ops[l->n++];
    memset(o, 0, sizeof *o);
    o->kind = kind;
    return o;
}

void gl_list_i2(int kind, uint32_t a, uint32_t b) { gl_listop *o = add(kind); o->a = a; o->b = b; }
void gl_list_f4(int kind, uint32_t a, float x, float y, float z, float w) { gl_listop *o = add(kind); o->a = a; o->f[0] = x; o->f[1] = y; o->f[2] = z; o->f[3] = w; }
void gl_list_big(int kind, uint32_t a, const void *data, size_t bytes)
{
    gl_listop *o = add(kind);
    o->a = a;
    o->big = malloc(bytes ? bytes : 1);
    if (o->big) memcpy(o->big, data, bytes);
}
void gl_list_bitmap(int w, int h, const float *f4, const void *bits, size_t bytes)
{
    gl_listop *o = add(LOP_BITMAP);
    o->w = w; o->h = h;
    memcpy(o->f, f4, 16);
    o->big = malloc(bytes ? bytes : 1);
    if (o->big) memcpy(o->big, bits, bytes);
}

void gl_list_uniform(GLint loc, int kind, int comps, int n, const void *data, size_t bytes)
{
    gl_listop *o = add(LOP_UNIFORM);
    o->a = (uint32_t)loc; o->w = kind << 8 | comps; o->h = n;
    o->big = malloc(bytes ? bytes : 1);
    if (o->big) memcpy(o->big, data, bytes);
}

void gl_list_sync_attribs(void)
{
    int u;
    if (C->cur_dirty & 1) gl_list_f4(LOP_COLOR, 0, C->color[0], C->color[1], C->color[2], C->color[3]);
    if (C->cur_dirty & 2) gl_list_f4(LOP_NORMAL, 0, C->normal[0], C->normal[1], C->normal[2], 0);
    if (C->cur_dirty & (1u << 8)) gl_list_f4(LOP_FOGC, 0, C->fog_coord, 0, 0, 0);
    if (C->cur_dirty & (1u << 9)) gl_list_f4(LOP_SEC, 0, C->sec_color[0], C->sec_color[1], C->sec_color[2], 1.0f);
    for (u = 0; u < MAX_UNITS; u++) {
        if (C->cur_dirty & (4u << u)) gl_list_f4(LOP_TEXCOORD, (uint32_t)u, C->unit[u].tc[0], C->unit[u].tc[1], C->unit[u].tc[2], C->unit[u].tc[3]);
    }
    C->cur_dirty = 0;
}

static void replay(const gl_list *l)
{
    int i;
    for (i = 0; i < l->n; i++) {
        const gl_listop *o = &l->ops[i];
        const float *f = o->f;
        switch (o->kind) {
        case LOP_BEGIN: gl_begin(o->a); break;
        case LOP_END: gl_end(); break;
        case LOP_VERTEX: gl_vertex(f[0], f[1], f[2], f[3]); break;
        case LOP_COLOR: memcpy(C->color, f, 16); break;
        case LOP_NORMAL: memcpy(C->normal, f, 12); break;
        case LOP_FOGC: C->fog_coord = f[0]; break;
        case LOP_SEC: memcpy(C->sec_color, f, 12); break;
        case LOP_TEXCOORD: memcpy(C->unit[o->a].tc, f, 16); break;
        case LOP_ENABLE: glEnable(o->a); break;
        case LOP_DISABLE: glDisable(o->a); break;
        case LOP_BINDTEX: glBindTexture(o->a, o->b); break;
        case LOP_MATRIXMODE: glMatrixMode(o->a); break;
        case LOP_PUSHM: glPushMatrix(); break;
        case LOP_POPM: glPopMatrix(); break;
        case LOP_LOADI: glLoadIdentity(); break;
        case LOP_LOADM: glLoadMatrixf((const float *)o->big); break;
        case LOP_MULTM: glMultMatrixf((const float *)o->big); break;
        case LOP_TRANSLATE: glTranslatef(f[0], f[1], f[2]); break;
        case LOP_SCALE: glScalef(f[0], f[1], f[2]); break;
        case LOP_ROTATE: glRotatef(f[0], f[1], f[2], f[3]); break;
        case LOP_BLENDFUNC: glBlendFunc(o->a, o->b); break;
        case LOP_DEPTHFUNC: glDepthFunc(o->a); break;
        case LOP_DEPTHMASK: glDepthMask((GLboolean)o->a); break;
        case LOP_ALPHAFUNC: glAlphaFunc(o->a, f[0]); break;
        case LOP_COLORMASK: glColorMask((GLboolean)(o->a >> 3 & 1), (GLboolean)(o->a >> 2 & 1), (GLboolean)(o->a >> 1 & 1), (GLboolean)(o->a & 1)); break;
        case LOP_CULLFACE: glCullFace(o->a); break;
        case LOP_FRONTFACE: glFrontFace(o->a); break;
        case LOP_SHADEMODEL: glShadeModel(o->a); break;
        case LOP_LINEWIDTH: glLineWidth(f[0]); break;
        case LOP_POINTSIZE: glPointSize(f[0]); break;
        case LOP_POLYGONMODE: glPolygonMode(o->a, o->b); break;
        case LOP_EDGEFLAG: glEdgeFlag((GLboolean)o->a); break;
        case LOP_POLYOFFSET: glPolygonOffset(f[0], f[1]); break;
        case LOP_MATERIAL: glMaterialfv(o->a & 0xFFFF, o->b, f); break;
        case LOP_LIGHT: glLightfv(o->a, o->b, f); break;
        case LOP_TEXENVF: glTexEnvf(o->a, o->b, f[0]); break;
        case LOP_CALL: glCallList(o->a); break;
        case LOP_BITMAP: glBitmap(o->w, o->h, f[0], f[1], f[2], f[3], (const GLubyte *)o->big); break;
        case LOP_ATTRIB: glVertexAttrib4f(o->a, f[0], f[1], f[2], f[3]); break;
        case LOP_USEPROG: glUseProgram(o->a); break;
        case LOP_UNIFORM: gl_glsl_uniform_replay(o); break;
        default: break;
        }
    }
}

GLuint APIENTRY glGenLists(GLsizei n) { GLuint f = g_next_list; g_next_list += (GLuint)n; return f; }
GLboolean APIENTRY glIsList(GLuint n) { return list_of(n, 0) ? GL_TRUE : GL_FALSE; }
void APIENTRY glListBase(GLuint b) { C->list_base = b; }

void APIENTRY glDeleteLists(GLuint first, GLsizei n)
{
    GLuint k;
    for (k = first; k < first + (GLuint)n; k++) {
        gl_list *l = list_of(k, 0);
        int i;
        if (!l) continue;
        for (i = 0; i < l->n; i++) free(l->ops[i].big);
        free(l->ops);
        l->ops = NULL; l->n = l->cap = 0;
    }
}

void APIENTRY glNewList(GLuint name, GLenum mode)
{
    int u;
    FLUSH_VERTICES();
    if (C->compiling) { SET_ERROR(GL_INVALID_OPERATION); return; }
    glDeleteLists(name, 1);
    C->compiling = list_of(name, 1);
    C->compile_mode = mode;
    C->cur_dirty = 0;
    memcpy(g_saved_color, C->color, 16); memcpy(g_saved_normal, C->normal, 12);
    for (u = 0; u < MAX_UNITS; u++) memcpy(g_saved_tc[u], C->unit[u].tc, 16);
}

void APIENTRY glEndList(void)
{
    int u;
    if (!C->compiling) { SET_ERROR(GL_INVALID_OPERATION); return; }
    gl_list_sync_attribs();
    C->compiling = NULL;
    if (C->compile_mode == GL_COMPILE) {                        /* compiling changes no current value */
        memcpy(C->color, g_saved_color, 16); memcpy(C->normal, g_saved_normal, 12);
        for (u = 0; u < MAX_UNITS; u++) memcpy(C->unit[u].tc, g_saved_tc[u], 16);
    }
}

void APIENTRY glCallList(GLuint name)
{
    gl_list *l;
    if (C->compiling) {
        gl_list_i2(LOP_CALL, name, 0);
        if (C->compile_mode == GL_COMPILE) return;
    }
    l = list_of(name, 0);
    if (l && C->in_list_exec < 64) {
        C->in_list_exec++;
        replay(l);
        C->in_list_exec--;
    }
}

void APIENTRY glCallLists(GLsizei n, GLenum type, const void *lists)
{
    GLsizei i;
    const uint8_t *p = (const uint8_t *)lists;
    for (i = 0; i < n; i++) {
        GLuint v;
        switch (type) {
        case GL_BYTE: v = (GLuint)(int)((const GLbyte *)p)[i]; break;
        case GL_UNSIGNED_BYTE: v = p[i]; break;
        case GL_SHORT: v = (GLuint)(int)((const GLshort *)p)[i]; break;
        case GL_UNSIGNED_SHORT: v = ((const GLushort *)p)[i]; break;
        case GL_INT: case GL_UNSIGNED_INT: v = ((const GLuint *)p)[i]; break;
        case GL_FLOAT: v = (GLuint)((const GLfloat *)p)[i]; break;
        case GL_2_BYTES: v = (GLuint)p[2 * i] << 8 | p[2 * i + 1]; break;
        default: SET_ERROR(GL_INVALID_ENUM); return;
        }
        glCallList(C->list_base + v);
    }
}

/* ---------------------------------------------------------------- wglUseFontBitmaps: a list per glyph */
static BOOL font_bitmaps(HDC dc, DWORD first, DWORD count, DWORD base, int wide)
{
    static const MAT2 id = { { 0, 1 }, { 0, 0 }, { 0, 0 }, { 0, 1 } };
    DWORD i;
    for (i = 0; i < count; i++) {
        GLYPHMETRICS gm;
        DWORD n = wide ? GetGlyphOutlineW(dc, first + i, GGO_BITMAP, &gm, 0, NULL, &id) : GetGlyphOutlineA(dc, first + i, GGO_BITMAP, &gm, 0, NULL, &id);
        uint8_t *src, *dst;
        int rows, row_in, row_out, y;
        float f[4];
        if (n == GDI_ERROR) {
            glNewList(base + i, GL_COMPILE);
            glEndList();
            continue;
        }
        rows = (int)gm.gmBlackBoxY;
        row_in = (((int)gm.gmBlackBoxX + 31) / 32) * 4;
        row_out = ((int)gm.gmBlackBoxX + 7) / 8;
        src = (uint8_t *)calloc(1, n ? n : 1);
        dst = (uint8_t *)calloc(1, (size_t)(rows ? rows : 1) * (row_out ? row_out : 1));
        if (!src || !dst) { free(src); free(dst); return FALSE; }
        if (n) {
            if (wide) GetGlyphOutlineW(dc, first + i, GGO_BITMAP, &gm, n, src, &id);
            else GetGlyphOutlineA(dc, first + i, GGO_BITMAP, &gm, n, src, &id);
        }
        for (y = 0; y < rows; y++) memcpy(dst + (size_t)(rows - 1 - y) * row_out, src + (size_t)y * row_in, (size_t)row_out);   /* GDI: top-down, glBitmap: bottom-up */
        f[0] = (float)-gm.gmptGlyphOrigin.x; f[1] = (float)(rows - gm.gmptGlyphOrigin.y); f[2] = (float)gm.gmCellIncX; f[3] = (float)gm.gmCellIncY;
        glNewList(base + i, GL_COMPILE);
        gl_list_bitmap((int)gm.gmBlackBoxX, rows, f, dst, (size_t)rows * row_out);
        glEndList();
        free(src); free(dst);
    }
    return TRUE;
}

BOOL WINAPI wglUseFontBitmapsA(HDC dc, DWORD first, DWORD count, DWORD base) { return font_bitmaps(dc, first, count, base, 0); }
BOOL WINAPI wglUseFontBitmapsW(HDC dc, DWORD first, DWORD count, DWORD base) { return font_bitmaps(dc, first, count, base, 1); }

/* ---------------------------------------------------------------- small queries and accepted no-ops */
GLboolean APIENTRY glAreTexturesResident(GLsizei n, const GLuint *t, GLboolean *r) { GLsizei i; (void)t; for (i = 0; i < n; i++) r[i] = GL_TRUE; return GL_TRUE; }
void APIENTRY glPrioritizeTextures(GLsizei n, const GLuint *t, const GLclampf *p) { (void)n; (void)t; (void)p; }
void APIENTRY glGetPointerv(GLenum p, void **v)
{
    switch (p) {
    case GL_VERTEX_ARRAY_POINTER: *v = (void *)C->a_vertex.ptr; break;
    case GL_COLOR_ARRAY_POINTER: *v = (void *)C->a_color.ptr; break;
    case GL_NORMAL_ARRAY_POINTER: *v = (void *)C->a_normal.ptr; break;
    case GL_FOG_COORD_ARRAY_POINTER: *v = (void *)C->a_fog.ptr; break;
    case GL_SECONDARY_COLOR_ARRAY_POINTER: *v = (void *)C->a_sec.ptr; break;
    case GL_TEXTURE_COORD_ARRAY_POINTER: *v = (void *)C->a_tc[C->client_unit].ptr; break;
    default: *v = NULL; break;
    }
}
void APIENTRY glGetMaterialfv(GLenum face, GLenum p, GLfloat *v)
{
    const gl_material *m = &C->mat[face == GL_BACK];
    switch (p) {
    case GL_AMBIENT: memcpy(v, m->ambient, 16); break;
    case GL_DIFFUSE: memcpy(v, m->diffuse, 16); break;
    case GL_SPECULAR: memcpy(v, m->specular, 16); break;
    case GL_EMISSION: memcpy(v, m->emission, 16); break;
    case GL_SHININESS: v[0] = m->shininess; break;
    default: SET_ERROR(GL_INVALID_ENUM); break;
    }
}
void APIENTRY glGetMaterialiv(GLenum face, GLenum p, GLint *v)
{
    float f[4] = { 0, 0, 0, 0 };
    int i;
    glGetMaterialfv(face, p, f);
    if (p == GL_SHININESS) v[0] = (GLint)f[0];
    else for (i = 0; i < 4; i++) v[i] = (GLint)(f[i] * 2147483647.0);
}
void APIENTRY glGetLightiv(GLenum l, GLenum p, GLint *v) { float f[4] = { 0, 0, 0, 0 }; int i; glGetLightfv(l, p, f); for (i = 0; i < 4; i++) v[i] = (GLint)f[i]; }
void APIENTRY glGetTexGenfv(GLenum c, GLenum p, GLfloat *v)
{
    const gl_unit *u = &C->unit[C->active_unit];
    int k = c == GL_S ? 0 : c == GL_T ? 1 : c == GL_R ? 2 : 3;
    if (p == GL_TEXTURE_GEN_MODE) v[0] = (GLfloat)u->gen_mode[k];
    else memcpy(v, p == GL_OBJECT_PLANE ? u->gen_obj[k] : u->gen_eye[k], 16);
}
void APIENTRY glGetTexGeniv(GLenum c, GLenum p, GLint *v) { float f[4]; int i; glGetTexGenfv(c, p, f); for (i = 0; i < (p == GL_TEXTURE_GEN_MODE ? 1 : 4); i++) v[i] = (GLint)f[i]; }
void APIENTRY glGetTexGendv(GLenum c, GLenum p, GLdouble *v) { float f[4]; int i; glGetTexGenfv(c, p, f); for (i = 0; i < (p == GL_TEXTURE_GEN_MODE ? 1 : 4); i++) v[i] = f[i]; }
void APIENTRY glIndexMask(GLuint m) { (void)m; }
void APIENTRY glClearIndex(GLfloat c) { (void)c; }
