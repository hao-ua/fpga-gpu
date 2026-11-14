/* what OpenGL 1.2 to 1.4 added and the library lacked, none of it in the hardware's way: multi-draw, window positions, compressed uploads (the PC decodes them: the TMU samples no compressed
 * format), the multisample state of a context with SAMPLE_BUFFERS 0 (GL_ARB_multisample allows that, and then none of it has an effect), point parameters, fog coordinates, secondary colours,
 * the client attribute stack. */
#include "g2_dxt.h"
#include "gl_internal.h"

/* ---------------------------------------------------------------- multi-draw */
void APIENTRY glMultiDrawArrays(GLenum mode, const GLint *first, const GLsizei *count, GLsizei n)
{
    GLsizei i;
    if (n < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    for (i = 0; i < n; i++) if (count[i] > 0) gl_draw_arrays(mode, first[i], count[i]);
}

void APIENTRY glMultiDrawElements(GLenum mode, const GLsizei *count, GLenum type, const void *const *indices, GLsizei n)
{
    GLsizei i;
    if (n < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    for (i = 0; i < n; i++) if (count[i] > 0) gl_draw_elements(mode, count[i], type, indices[i], 0, 0, 0);
}

/* ---------------------------------------------------------------- the raster position in window coordinates, §2.13 */
static void window_pos(float x, float y, float z)
{
    FLUSH_VERTICES();
    C->raster_pos[0] = x; C->raster_pos[1] = y;
    C->raster_pos[2] = z <= 0.0f ? C->depth_near : z >= 1.0f ? C->depth_far : C->depth_near + z * (C->depth_far - C->depth_near);
    C->raster_pos[3] = 1.0f;
    C->raster_valid = 1;
    C->raster_dist = C->fog_src == GL_FOG_COORD ? C->fog_coord : 0.0f;
    memcpy(C->raster_color, C->color, 16);
    memcpy(C->raster_tc, C->unit[0].tc, 16);
}

#define WP(sfx, T) \
    void APIENTRY glWindowPos2##sfx(T x, T y) { window_pos((float)x, (float)y, 0.0f); } \
    void APIENTRY glWindowPos3##sfx(T x, T y, T z) { window_pos((float)x, (float)y, (float)z); } \
    void APIENTRY glWindowPos2##sfx##v(const T *v) { window_pos((float)v[0], (float)v[1], 0.0f); } \
    void APIENTRY glWindowPos3##sfx##v(const T *v) { window_pos((float)v[0], (float)v[1], (float)v[2]); }
WP(d, GLdouble)
WP(f, GLfloat)
WP(i, GLint)
WP(s, GLshort)

/* ---------------------------------------------------------------- multisample state: SAMPLE_BUFFERS is 0 */
void APIENTRY glSampleCoverage(GLclampf v, GLboolean inv)
{
    FLUSH_VERTICES();
    C->sample_val = v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v;
    C->sample_inv = inv != 0;
}

/* ---------------------------------------------------------------- compressed textures */
static int dxt_kind(GLenum f)
{
    switch (f) {
    case GL_COMPRESSED_RGB_S3TC_DXT1_EXT: return G2_DXT1;
    case GL_COMPRESSED_RGBA_S3TC_DXT1_EXT: return G2_DXT1A;
    case GL_COMPRESSED_RGBA_S3TC_DXT3_EXT: return G2_DXT3;
    case GL_COMPRESSED_RGBA_S3TC_DXT5_EXT: return G2_DXT5;
    default: return 0;
    }
}

void APIENTRY glCompressedTexImage2D(GLenum target, GLint level, GLenum internal, GLsizei w, GLsizei h, GLint border, GLsizei size, const void *data)
{
    int kind = dxt_kind(internal), face = target >= GL_TEXTURE_CUBE_MAP_POSITIVE_X && target <= GL_TEXTURE_CUBE_MAP_NEGATIVE_Z ? (int)(target - GL_TEXTURE_CUBE_MAP_POSITIVE_X) : 0;
    uint8_t *rgba;
    gl_tex *t;
    const void *src = gl_unpack_source(data);
    if (target == GL_PROXY_TEXTURE_2D || target == GL_PROXY_TEXTURE_CUBE_MAP) { gl_tex_proxy(target, level, w, h, 1, internal); return; }
    if (!kind || (target != GL_TEXTURE_2D && !face && target != GL_TEXTURE_CUBE_MAP_POSITIVE_X)) { SET_ERROR(GL_INVALID_ENUM); return; }
    if (border || level < 0 || level >= MAX_LEVELS || w < 0 || h < 0 || size != (GLsizei)g2_dxt_bytes(kind, w, h)) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (!src && size) { SET_ERROR(GL_INVALID_VALUE); return; }
    rgba = (uint8_t *)malloc((size_t)w * h * 4 + 4);
    if (!rgba) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    g2_dxt_decode(kind, (const uint8_t *)src, w, h, rgba);
    gl_tex_store_rgba(target, level, kind == G2_DXT1 ? GL_RGB : GL_RGBA, w, h, rgba);
    free(rgba);
    t = gl_bound_tex(target);
    if (!t) return;
    if (level == 0) t->internal = internal;
    free(t->comp[face][level]);
    t->comp[face][level] = malloc((size_t)size + 1);
    t->comp_size[face][level] = size;
    if (t->comp[face][level]) memcpy(t->comp[face][level], src, (size_t)size);
}

/* the region is whole 4 x 4 blocks, or reaches the level's edge: §3.8.3 of the extension */
void APIENTRY glCompressedTexSubImage2D(GLenum target, GLint level, GLint x, GLint y, GLsizei w, GLsizei h, GLenum format, GLsizei size, const void *data)
{
    int kind = dxt_kind(format), face = target >= GL_TEXTURE_CUBE_MAP_POSITIVE_X && target <= GL_TEXTURE_CUBE_MAP_NEGATIVE_Z ? (int)(target - GL_TEXTURE_CUBE_MAP_POSITIVE_X) : 0;
    gl_tex *t = gl_bound_tex(target);
    int lw, lh, bs, bx, by, bw, bh, r;
    uint8_t *rgba;
    const uint8_t *src = (const uint8_t *)gl_unpack_source(data);
    if (!kind) { SET_ERROR(GL_INVALID_ENUM); return; }
    if (!t || level < 0 || level >= MAX_LEVELS || !t->comp[face][level] || t->internal != format) { SET_ERROR(GL_INVALID_OPERATION); return; }
    lw = t->level[face][level].w; lh = t->level[face][level].h;
    if (x < 0 || y < 0 || w < 0 || h < 0 || x + w > lw || y + h > lh) { SET_ERROR(GL_INVALID_VALUE); return; }
    if ((x & 3) || (y & 3) || ((w & 3) && x + w != lw) || ((h & 3) && y + h != lh)) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (size != (GLsizei)g2_dxt_bytes(kind, w, h)) { SET_ERROR(GL_INVALID_VALUE); return; }
    bs = kind == G2_DXT1 || kind == G2_DXT1A ? 8 : 16;
    bx = x / 4; by = y / 4; bw = (w + 3) / 4; bh = (h + 3) / 4;
    {
        int stride = (lw + 3) / 4 * bs;
        for (r = 0; r < bh; r++) memcpy((uint8_t *)t->comp[face][level] + (size_t)(by + r) * stride + (size_t)bx * bs, src + (size_t)r * bw * bs, (size_t)bw * bs);
    }
    rgba = (uint8_t *)malloc((size_t)w * h * 4 + 4);
    if (!rgba) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    g2_dxt_decode(kind, src, w, h, rgba);
    gl_tex_sub_rgba(target, level, x, y, w, h, rgba);
    free(rgba);
}

void APIENTRY glCompressedTexImage1D(GLenum target, GLint level, GLenum internal, GLsizei w, GLint border, GLsizei size, const void *data) { (void)target; (void)level; (void)internal; (void)w; (void)border; (void)size; (void)data; SET_ERROR(GL_INVALID_ENUM); }
void APIENTRY glCompressedTexSubImage1D(GLenum target, GLint level, GLint x, GLsizei w, GLenum format, GLsizei size, const void *data) { (void)target; (void)level; (void)x; (void)w; (void)format; (void)size; (void)data; SET_ERROR(GL_INVALID_ENUM); }

void APIENTRY glGetCompressedTexImage(GLenum target, GLint level, void *out)
{
    out = gl_pack_ptr(out);
    int face = target >= GL_TEXTURE_CUBE_MAP_POSITIVE_X && target <= GL_TEXTURE_CUBE_MAP_NEGATIVE_Z ? (int)(target - GL_TEXTURE_CUBE_MAP_POSITIVE_X) : 0;
    gl_tex *t = gl_bound_tex(target);
    if (level < 0 || level >= MAX_LEVELS) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (!t || !t->comp[face][level]) { SET_ERROR(GL_INVALID_OPERATION); return; }
    memcpy(out, t->comp[face][level], (size_t)t->comp_size[face][level]);
}

/* ---------------------------------------------------------------- point parameters, §3.3 */
static void point_param(GLenum p, const float *v)
{
    FLUSH_VERTICES();
    switch (p) {
    case GL_POINT_SIZE_MIN: if (v[0] < 0) { SET_ERROR(GL_INVALID_VALUE); return; } C->pt_min = v[0]; break;
    case GL_POINT_SIZE_MAX: if (v[0] < 0) { SET_ERROR(GL_INVALID_VALUE); return; } C->pt_max = v[0]; break;
    case GL_POINT_FADE_THRESHOLD_SIZE: if (v[0] < 0) { SET_ERROR(GL_INVALID_VALUE); return; } C->pt_fade = v[0]; break;
    case GL_POINT_SPRITE_COORD_ORIGIN: if ((GLenum)v[0] != GL_LOWER_LEFT && (GLenum)v[0] != GL_UPPER_LEFT) { SET_ERROR(GL_INVALID_ENUM); return; } C->sprite_origin = (GLenum)v[0]; break;
    case GL_POINT_DISTANCE_ATTENUATION: memcpy(C->pt_att, v, 12); break;
    default: SET_ERROR(GL_INVALID_ENUM); break;
    }
}

void APIENTRY glPointParameterf(GLenum p, GLfloat v) { if (p == GL_POINT_DISTANCE_ATTENUATION) { SET_ERROR(GL_INVALID_ENUM); return; } point_param(p, &v); }
void APIENTRY glPointParameterfv(GLenum p, const GLfloat *v) { point_param(p, v); }
void APIENTRY glPointParameteri(GLenum p, GLint v) { float f = (float)v; if (p == GL_POINT_DISTANCE_ATTENUATION) { SET_ERROR(GL_INVALID_ENUM); return; } point_param(p, &f); }
void APIENTRY glPointParameteriv(GLenum p, const GLint *v)
{
    float f[3];
    int i;
    f[0] = (float)v[0];
    if (p == GL_POINT_DISTANCE_ATTENUATION) for (i = 1; i < 3; i++) f[i] = (float)v[i];
    point_param(p, f);
}

/* ---------------------------------------------------------------- fog coordinates, secondary colours */
void APIENTRY glFogCoordf(GLfloat c) { C->fog_coord = c; C->cur_dirty |= 1u << 8; }
void APIENTRY glFogCoordfv(const GLfloat *c) { glFogCoordf(c[0]); }
void APIENTRY glFogCoordd(GLdouble c) { glFogCoordf((GLfloat)c); }
void APIENTRY glFogCoorddv(const GLdouble *c) { glFogCoordf((GLfloat)c[0]); }
void APIENTRY glFogCoordPointer(GLenum t, GLsizei st, const void *p) { gl_set_array(&C->a_fog, 1, t, st, p); }

#define N_D(x) ((float)(x))
#define SC(sfx, T, N) \
    void APIENTRY glSecondaryColor3##sfx(T r, T g, T b) { C->sec_color[0] = N(r); C->sec_color[1] = N(g); C->sec_color[2] = N(b); C->cur_dirty |= 1u << 9; } \
    void APIENTRY glSecondaryColor3##sfx##v(const T *v) { glSecondaryColor3##sfx(v[0], v[1], v[2]); }
#define NB(x) ((float)((2.0 * (x) + 1.0) / 255.0))
#define NS(x) ((float)((2.0 * (x) + 1.0) / 65535.0))
#define NI(x) ((float)((2.0 * (x) + 1.0) / 4294967295.0))
#define NUB(x) ((float)((x) / 255.0))
#define NUS(x) ((float)((x) / 65535.0))
#define NUI(x) ((float)((x) / 4294967295.0))
SC(b, GLbyte, NB)
SC(s, GLshort, NS)
SC(i, GLint, NI)
SC(ub, GLubyte, NUB)
SC(us, GLushort, NUS)
SC(ui, GLuint, NUI)
SC(f, GLfloat, N_D)
SC(d, GLdouble, N_D)
void APIENTRY glSecondaryColorPointer(GLint size, GLenum t, GLsizei st, const void *p)
{
    if (size != 3) { SET_ERROR(GL_INVALID_VALUE); return; }
    gl_set_array(&C->a_sec, size, t, st, p);
}

/* ---------------------------------------------------------------- glPushClientAttrib, §6.1.14 */
#define CLIENT_DEPTH 16
typedef struct cli_frame {
    GLbitfield mask;
    int un[6], pk[6];
    gl_array v, c, n, t[MAX_UNITS], e, f, s;
    int client_unit;
    GLuint array, element;
} cli_frame;
static cli_frame g_cli[CLIENT_DEPTH];
static int g_cli_sp;

void APIENTRY glPushClientAttrib(GLbitfield m)
{
    cli_frame *f;
    if (g_cli_sp == CLIENT_DEPTH) { SET_ERROR(GL_STACK_OVERFLOW); return; }
    f = &g_cli[g_cli_sp++];
    f->mask = m;
    if (m & GL_CLIENT_PIXEL_STORE_BIT) {
        f->un[0] = C->un_align; f->un[1] = C->un_row; f->un[2] = C->un_skip_px; f->un[3] = C->un_skip_row; f->un[4] = C->un_image_h; f->un[5] = C->un_skip_images;
        f->pk[0] = C->pk_align; f->pk[1] = C->pk_row; f->pk[2] = C->pk_skip_px; f->pk[3] = C->pk_skip_row; f->pk[4] = C->pk_image_h; f->pk[5] = C->pk_skip_images;
    }
    if (m & GL_CLIENT_VERTEX_ARRAY_BIT) {
        f->v = C->a_vertex; f->c = C->a_color; f->n = C->a_normal; memcpy(f->t, C->a_tc, sizeof f->t); f->e = C->a_edge; f->f = C->a_fog; f->s = C->a_sec;
        f->client_unit = C->client_unit; f->array = C->bound_array; f->element = C->bound_element;
    }
}

void APIENTRY glPopClientAttrib(void)
{
    cli_frame *f;
    if (!g_cli_sp) { SET_ERROR(GL_STACK_UNDERFLOW); return; }
    FLUSH_VERTICES();
    f = &g_cli[--g_cli_sp];
    if (f->mask & GL_CLIENT_PIXEL_STORE_BIT) {
        C->un_align = f->un[0]; C->un_row = f->un[1]; C->un_skip_px = f->un[2]; C->un_skip_row = f->un[3]; C->un_image_h = f->un[4]; C->un_skip_images = f->un[5];
        C->pk_align = f->pk[0]; C->pk_row = f->pk[1]; C->pk_skip_px = f->pk[2]; C->pk_skip_row = f->pk[3]; C->pk_image_h = f->pk[4]; C->pk_skip_images = f->pk[5];
    }
    if (f->mask & GL_CLIENT_VERTEX_ARRAY_BIT) {
        C->a_vertex = f->v; C->a_color = f->c; C->a_normal = f->n; memcpy(C->a_tc, f->t, sizeof f->t); C->a_edge = f->e; C->a_fog = f->f; C->a_sec = f->s;
        C->client_unit = f->client_unit; C->bound_array = f->array; C->bound_element = f->element;
    }
}

/* ---------------------------------------------------------------- occlusion queries, GL 1.5 §4.1.7. The board counts quads, triangles, early rejects and frames, not samples, so GL_QUERY_COUNTER_BITS is 0, which
 * §4.1.7 allows: the counter then holds no useful information. A query still works as an object and always ends: its result is 1 if a draw ran inside it (never 0 when samples passed), so a program that
 * culls by it draws too much, never too little. GL_ARB_occlusion_query stays out of the extension string: an application that asks for that is asking for the count */
#define Q_BUCKETS 64
typedef struct qnode { GLuint name; GLuint result; struct qnode *next; } qnode;
static qnode *g_q[Q_BUCKETS];
static GLuint g_q_next = 1;

static qnode **q_find(GLuint id)
{
    qnode **p;
    for (p = &g_q[id % Q_BUCKETS]; *p; p = &(*p)->next) if ((*p)->name == id) return p;
    return p;
}

void APIENTRY glGenQueries(GLsizei n, GLuint *ids)
{
    GLsizei i;
    if (n < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    for (i = 0; i < n; i++) ids[i] = g_q_next++;
}

void APIENTRY glDeleteQueries(GLsizei n, const GLuint *ids)
{
    GLsizei i;
    if (n < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    for (i = 0; i < n; i++) {
        qnode **p = q_find(ids[i]), *q = *p;
        if (ids[i] && ids[i] == C->query) { SET_ERROR(GL_INVALID_OPERATION); continue; }
        if (q) { *p = q->next; free(q); }
    }
}

GLboolean APIENTRY glIsQuery(GLuint id) { return id && *q_find(id) ? GL_TRUE : GL_FALSE; }

void APIENTRY glBeginQuery(GLenum target, GLuint id)
{
    qnode **p;
    FLUSH_VERTICES();
    if (target != GL_SAMPLES_PASSED) { SET_ERROR(GL_INVALID_ENUM); return; }
    if (!id || C->query) { SET_ERROR(GL_INVALID_OPERATION); return; }
    p = q_find(id);
    if (!*p) {
        qnode *q = (qnode *)calloc(1, sizeof *q);
        if (!q) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
        q->name = id;
        *p = q;
    }
    C->query = id;
    C->query_from = C->draws;
}

void APIENTRY glEndQuery(GLenum target)
{
    qnode *q;
    FLUSH_VERTICES();
    if (target != GL_SAMPLES_PASSED) { SET_ERROR(GL_INVALID_ENUM); return; }
    if (!C->query) { SET_ERROR(GL_INVALID_OPERATION); return; }
    q = *q_find(C->query);
    if (q) q->result = C->draws != C->query_from;
    C->query = 0;
}

void APIENTRY glGetQueryiv(GLenum target, GLenum p, GLint *v)
{
    if (target != GL_SAMPLES_PASSED) { SET_ERROR(GL_INVALID_ENUM); return; }
    if (p == GL_QUERY_COUNTER_BITS) *v = 0;
    else if (p == GL_CURRENT_QUERY) *v = (GLint)C->query;
    else SET_ERROR(GL_INVALID_ENUM);
}

void APIENTRY glGetQueryObjectuiv(GLuint id, GLenum p, GLuint *v)
{
    qnode *q = *q_find(id);
    if (!q || id == C->query) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (p == GL_QUERY_RESULT) *v = q->result;
    else if (p == GL_QUERY_RESULT_AVAILABLE) *v = GL_TRUE;
    else SET_ERROR(GL_INVALID_ENUM);
}

void APIENTRY glGetQueryObjectiv(GLuint id, GLenum p, GLint *v) { glGetQueryObjectuiv(id, p, (GLuint *)v); }
