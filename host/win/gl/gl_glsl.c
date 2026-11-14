/* OpenGL 2.0: shader and program objects and GLSL 1.10, on the GLSL ES 1.00 compiler of host/win/shc.
 *
 * What GLSL 1.10 has beyond ES 1.00 is the state of the fixed pipeline and a few types and functions, and all of that is text. A shader is translated when a program needs it (xl_*): built-in attributes and state become
 * ordinary attributes and hidden uniforms, declared only if used; a built-in structure goes member by member, so gl_LightSource[0].diffuse costs one vector and not the whole array; varyings of the fixed pipeline are
 * globals that a wrapped main() copies from and to g2v* varyings, the names the generated programs of gl_ff.c use, so either stage may be left to them.
 *
 * Much of the fixed pipeline lives in generated programs here, and the hardware has none of it: alpha test, stipples, antialiasing cover, logic op, clip planes. So a user's program goes on doing those, by what the wrapped
 * main() adds around the user's, and what a sampler can do depends on what is bound to it (a depth texture, a border). A program is therefore a family of variants, one per state key, as gl_ff.c's programs are, each
 * compiled when first drawn. A link compiles the first, for the errors and for the tables a program answers from. */
#include <ctype.h>
#include <stdarg.h>
#include "gl_internal.h"

enum { ST_1D, ST_2D, ST_3D, ST_CUBE, ST_1DS, ST_2DS };

typedef struct tb { char *p; size_t n, cap; } tb;

static void tb_put(tb *t, const char *s, size_t n)
{
    if (t->n + n + 1 > t->cap) {
        t->cap = (t->n + n + 1) * 2 + 4096;
        t->p = (char *)realloc(t->p, t->cap);
    }
    memcpy(t->p + t->n, s, n);
    t->n += n;
    t->p[t->n] = 0;
}

static void tb_f(tb *t, const char *fmt, ...)
{
    char b[1024];
    va_list ap;
    int k;
    va_start(ap, fmt);
    k = vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    if (k > 0) tb_put(t, b, (size_t)k < sizeof b ? (size_t)k : sizeof b - 1);
}

/* ---------------------------------------------------------------- objects */
typedef struct gs_shader { GLuint name; GLenum type; char *src; int compiled, del, refs; char log[2048]; struct gs_shader *next; } gs_shader;
typedef struct gs_samp { char name[64]; int type, count, first; } gs_samp;
typedef struct gs_uni {
    char name[96];
    GLenum type;
    int size, stride, comps;    /* elements, rows of one, floats a row */
    int base;                   /* 0 float, 1 int, 2 bool */
    int sampler;                /* a sampler's first index, -1 */
    float *val;                 /* size * stride * comps */
    uint32_t *bits;             /* the same as the hardware's numbers */
} gs_uni;
typedef struct gs_info { uint32_t tc; int color, sec, fogf, front, back, fsec, bsec, clipv, psize, has_main, lodx; } gs_info;
typedef struct gs_key { ff_key ff, sk; uint8_t vpps, two, flat, pix, lbias[MAX_UNITS]; } gs_key;     /* pix: a picture through a fragment shader: 1 its colour, 2 its alpha says which fragments exist */
typedef struct gs_bind { int uni, vs, fs, vc, fc; } gs_bind;
typedef struct gs_hid { uint8_t kind, idx, fs, rows; uint16_t slot; } gs_hid;
typedef struct gs_variant {
    ff_prog ff;
    gs_key key;
    gs_bind *bind;
    int nbind;
    gs_hid *hid;
    int nhid;
    int samp_slot[MAX_UNITS];   /* sampler index -> the hardware's */
    int pix_slot;
    char *sasm;                 /* the vertex program, for a position the PC must know */
    void *vm;
    double (*cpool)[4];
    int ncpool, cbase, vary_color, vary_tc0, vary_flat[4], vary_clip[MAX_CLIP];
    struct gs_variant *next;
} gs_variant;
typedef struct gs_attr { char name[64]; GLenum type; int rows, loc, a; } gs_attr;
typedef struct gs_prog {
    GLuint name;
    int linked, validated, del, has_vs, has_fs;
    GLuint att[16];
    int natt;
    char *src[2];               /* each stage's shaders, one after the other */
    gs_info info[2];
    gs_samp samp[MAX_UNITS];
    int nsamp, nel;             /* declarations, sampler indices */
    int samp_unit[MAX_UNITS];
    gs_uni *uni;
    int nuni;
    gs_attr attr[MAX_ATTRIBS];
    int nattr, attr_used[MAX_ATTRIBS];
    uint32_t gen_mask;
    int vertex_alias;           /* gl_Vertex is read: generic 0 is taken */
    struct { char name[64]; GLuint loc; } bind[64];
    int nbind;
    gs_variant *variants, *last;
    char log[2048];
    struct gs_prog *next;
} gs_prog;

#define G_BUCKETS 256
static gs_shader *g_sh[G_BUCKETS];
static gs_prog *g_pr[G_BUCKETS];
static GLuint g_next = 1;

static gs_shader *sh_of(GLuint n) { gs_shader *s; for (s = g_sh[n % G_BUCKETS]; s; s = s->next) if (s->name == n) return s; return NULL; }
static gs_prog *pr_of(GLuint n) { gs_prog *p; for (p = g_pr[n % G_BUCKETS]; p; p = p->next) if (p->name == n) return p; return NULL; }

static void sh_free(gs_shader *s)
{
    gs_shader **q;
    for (q = &g_sh[s->name % G_BUCKETS]; *q; q = &(*q)->next) if (*q == s) { *q = s->next; break; }
    free(s->src);
    free(s);
}

/* ---------------------------------------------------------------- built-in state */
enum { B_MAT = 0, B_NM = 16, B_NS, B_CLIP, B_ENV, B_EYE, B_OBJ = B_EYE + 4, B_DR = B_OBJ + 4, B_PT = B_DR + 3, B_MF = B_PT + 7, B_MB = B_MF + 5, B_LS = B_MB + 5, B_LM = B_LS + 12, B_LMPF, B_LMPB, B_LPF, B_LPB = B_LPF + 3, B_FOG = B_LPB + 3 };

/* the matrices: { MV, PR, MVP, TM } x { as it is, inverse, transpose, inverse transpose } */
static const struct bi_uni { const char *name, *ty; int count, kind; } BI_UNI[] = {
    { "gl_ModelViewMatrix", "mat4", 0, 0 }, { "gl_ProjectionMatrix", "mat4", 0, 1 }, { "gl_ModelViewProjectionMatrix", "mat4", 0, 2 }, { "gl_TextureMatrix", "mat4", MAX_UNITS, 3 },
    { "gl_ModelViewMatrixInverse", "mat4", 0, 4 }, { "gl_ProjectionMatrixInverse", "mat4", 0, 5 }, { "gl_ModelViewProjectionMatrixInverse", "mat4", 0, 6 }, { "gl_TextureMatrixInverse", "mat4", MAX_UNITS, 7 },
    { "gl_ModelViewMatrixTranspose", "mat4", 0, 8 }, { "gl_ProjectionMatrixTranspose", "mat4", 0, 9 }, { "gl_ModelViewProjectionMatrixTranspose", "mat4", 0, 10 }, { "gl_TextureMatrixTranspose", "mat4", MAX_UNITS, 11 },
    { "gl_ModelViewMatrixInverseTranspose", "mat4", 0, 12 }, { "gl_ProjectionMatrixInverseTranspose", "mat4", 0, 13 }, { "gl_ModelViewProjectionMatrixInverseTranspose", "mat4", 0, 14 }, { "gl_TextureMatrixInverseTranspose", "mat4", MAX_UNITS, 15 },
    { "gl_NormalMatrix", "mat3", 0, B_NM }, { "gl_NormalScale", "float", 0, B_NS }, { "gl_ClipPlane", "vec4", MAX_CLIP, B_CLIP }, { "gl_TextureEnvColor", "vec4", MAX_UNITS, B_ENV },
    { "gl_EyePlaneS", "vec4", MAX_UNITS, B_EYE }, { "gl_EyePlaneT", "vec4", MAX_UNITS, B_EYE + 1 }, { "gl_EyePlaneR", "vec4", MAX_UNITS, B_EYE + 2 }, { "gl_EyePlaneQ", "vec4", MAX_UNITS, B_EYE + 3 },
    { "gl_ObjectPlaneS", "vec4", MAX_UNITS, B_OBJ }, { "gl_ObjectPlaneT", "vec4", MAX_UNITS, B_OBJ + 1 }, { "gl_ObjectPlaneR", "vec4", MAX_UNITS, B_OBJ + 2 }, { "gl_ObjectPlaneQ", "vec4", MAX_UNITS, B_OBJ + 3 },
};
#define N_BI_UNI ((int)(sizeof BI_UNI / sizeof BI_UNI[0]))

/* the structures, member by member */
static const struct bi_mem { const char *var, *mem, *ty; int count, kind; } BI_MEM[] = {
    { "gl_DepthRange", "near", "float", 0, B_DR }, { "gl_DepthRange", "far", "float", 0, B_DR + 1 }, { "gl_DepthRange", "diff", "float", 0, B_DR + 2 },
    { "gl_Point", "size", "float", 0, B_PT }, { "gl_Point", "sizeMin", "float", 0, B_PT + 1 }, { "gl_Point", "sizeMax", "float", 0, B_PT + 2 }, { "gl_Point", "fadeThresholdSize", "float", 0, B_PT + 3 },
    { "gl_Point", "distanceConstantAttenuation", "float", 0, B_PT + 4 }, { "gl_Point", "distanceLinearAttenuation", "float", 0, B_PT + 5 }, { "gl_Point", "distanceQuadraticAttenuation", "float", 0, B_PT + 6 },
    { "gl_FrontMaterial", "emission", "vec4", 0, B_MF }, { "gl_FrontMaterial", "ambient", "vec4", 0, B_MF + 1 }, { "gl_FrontMaterial", "diffuse", "vec4", 0, B_MF + 2 }, { "gl_FrontMaterial", "specular", "vec4", 0, B_MF + 3 }, { "gl_FrontMaterial", "shininess", "float", 0, B_MF + 4 },
    { "gl_BackMaterial", "emission", "vec4", 0, B_MB }, { "gl_BackMaterial", "ambient", "vec4", 0, B_MB + 1 }, { "gl_BackMaterial", "diffuse", "vec4", 0, B_MB + 2 }, { "gl_BackMaterial", "specular", "vec4", 0, B_MB + 3 }, { "gl_BackMaterial", "shininess", "float", 0, B_MB + 4 },
    { "gl_LightSource", "ambient", "vec4", MAX_LIGHTS, B_LS }, { "gl_LightSource", "diffuse", "vec4", MAX_LIGHTS, B_LS + 1 }, { "gl_LightSource", "specular", "vec4", MAX_LIGHTS, B_LS + 2 }, { "gl_LightSource", "position", "vec4", MAX_LIGHTS, B_LS + 3 },
    { "gl_LightSource", "halfVector", "vec4", MAX_LIGHTS, B_LS + 4 }, { "gl_LightSource", "spotDirection", "vec3", MAX_LIGHTS, B_LS + 5 }, { "gl_LightSource", "spotExponent", "float", MAX_LIGHTS, B_LS + 6 }, { "gl_LightSource", "spotCutoff", "float", MAX_LIGHTS, B_LS + 7 },
    { "gl_LightSource", "spotCosCutoff", "float", MAX_LIGHTS, B_LS + 8 }, { "gl_LightSource", "constantAttenuation", "float", MAX_LIGHTS, B_LS + 9 }, { "gl_LightSource", "linearAttenuation", "float", MAX_LIGHTS, B_LS + 10 }, { "gl_LightSource", "quadraticAttenuation", "float", MAX_LIGHTS, B_LS + 11 },
    { "gl_LightModel", "ambient", "vec4", 0, B_LM }, { "gl_FrontLightModelProduct", "sceneColor", "vec4", 0, B_LMPF }, { "gl_BackLightModelProduct", "sceneColor", "vec4", 0, B_LMPB },
    { "gl_FrontLightProduct", "ambient", "vec4", MAX_LIGHTS, B_LPF }, { "gl_FrontLightProduct", "diffuse", "vec4", MAX_LIGHTS, B_LPF + 1 }, { "gl_FrontLightProduct", "specular", "vec4", MAX_LIGHTS, B_LPF + 2 },
    { "gl_BackLightProduct", "ambient", "vec4", MAX_LIGHTS, B_LPB }, { "gl_BackLightProduct", "diffuse", "vec4", MAX_LIGHTS, B_LPB + 1 }, { "gl_BackLightProduct", "specular", "vec4", MAX_LIGHTS, B_LPB + 2 },
    { "gl_Fog", "color", "vec4", 0, B_FOG }, { "gl_Fog", "density", "float", 0, B_FOG + 1 }, { "gl_Fog", "start", "float", 0, B_FOG + 2 }, { "gl_Fog", "end", "float", 0, B_FOG + 3 }, { "gl_Fog", "scale", "float", 0, B_FOG + 4 },
};
#define N_BI_MEM ((int)(sizeof BI_MEM / sizeof BI_MEM[0]))

/* the types of those structures, for a shader that takes one whole */
static const struct bi_struct { const char *type, *var; } BI_STRUCT[] = {
    { "gl_DepthRangeParameters", "gl_DepthRange" }, { "gl_PointParameters", "gl_Point" }, { "gl_MaterialParameters", "gl_FrontMaterial" }, { "gl_MaterialParameters", "gl_BackMaterial" },
    { "gl_LightSourceParameters", "gl_LightSource" }, { "gl_LightModelParameters", "gl_LightModel" }, { "gl_LightModelProducts", "gl_FrontLightModelProduct" }, { "gl_LightModelProducts", "gl_BackLightModelProduct" },
    { "gl_LightProducts", "gl_FrontLightProduct" }, { "gl_LightProducts", "gl_BackLightProduct" }, { "gl_FogParameters", "gl_Fog" },
};
#define N_BI_STRUCT ((int)(sizeof BI_STRUCT / sizeof BI_STRUCT[0]))

static const struct bi_const { const char *name; int v; } BI_CONST[] = {
    { "gl_MaxLights", MAX_LIGHTS }, { "gl_MaxClipPlanes", MAX_CLIP }, { "gl_MaxTextureUnits", MAX_UNITS }, { "gl_MaxTextureCoords", MAX_UNITS }, { "gl_MaxVertexAttribs", MAX_ATTRIBS }, { "gl_MaxVertexUniformComponents", 896 },
    { "gl_MaxVaryingFloats", 32 }, { "gl_MaxVertexTextureImageUnits", 0 }, { "gl_MaxCombinedTextureImageUnits", MAX_UNITS }, { "gl_MaxTextureImageUnits", MAX_UNITS }, { "gl_MaxFragmentUniformComponents", 192 }, { "gl_MaxDrawBuffers", 1 },
};

/* a vertex attribute of the pipeline; in a fragment shader gl_Color and gl_SecondaryColor are varyings */
static const struct bi_attr { const char *name, *ty, *as; int a; } BI_ATTR[] = {
    { "gl_Vertex", "vec4", "g2a_Vertex", A_POS }, { "gl_Normal", "vec3", "g2a_Normal", A_NORMAL }, { "gl_Color", "vec4", "g2a_Color", A_COLOR }, { "gl_SecondaryColor", "vec4", "g2a_SecondaryColor", A_SEC }, { "gl_FogCoord", "float", "g2a_FogCoord", A_FOGC },
    { "gl_MultiTexCoord0", "vec4", "g2a_Tc0", A_TC0 }, { "gl_MultiTexCoord1", "vec4", "g2a_Tc1", A_TC0 + 1 }, { "gl_MultiTexCoord2", "vec4", "g2a_Tc2", A_TC0 + 2 }, { "gl_MultiTexCoord3", "vec4", "g2a_Tc3", A_TC0 + 3 },
    { NULL, "vec4", "g2aStip", A_STIP }, { NULL, "vec4", "g2aLine", A_LINE },
    { NULL, "vec4", "g2aFlat0", A_FLAT }, { NULL, "vec4", "g2aFlat1", A_FLAT + 1 }, { NULL, "vec4", "g2aFlat2", A_FLAT + 2 }, { NULL, "vec4", "g2aFlat3", A_FLAT + 3 },
    { "gl_MultiTexCoord4", "vec4", "g2a_Tc4", A_TC0 + 4 }, { "gl_MultiTexCoord5", "vec4", "g2a_Tc5", A_TC0 + 5 }, { "gl_MultiTexCoord6", "vec4", "g2a_Tc6", A_TC0 + 6 }, { "gl_MultiTexCoord7", "vec4", "g2a_Tc7", A_TC0 + 7 },
};
#define N_BI_ATTR ((int)(sizeof BI_ATTR / sizeof BI_ATTR[0]))

/* the texture functions, each by what it hands the sampler: st from the coordinate c */
static const struct bi_tex { const char *name, *ty, *st; int sampler; } BI_TEX[] = {
    { "texture1D", "float", "vec3(c, 0.0, 0.0)", ST_1D }, { "texture1DProj", "vec2", "vec3(c.x / c.y, 0.0, 0.0)", ST_1D }, { "texture1DProj", "vec4", "vec3(c.x / c.w, 0.0, 0.0)", ST_1D },
    { "texture2D", "vec2", "vec3(c, 0.0)", ST_2D }, { "texture2DProj", "vec3", "vec3(c.xy / c.z, 0.0)", ST_2D }, { "texture2DProj", "vec4", "vec3(c.xy / c.w, 0.0)", ST_2D },
    { "texture3D", "vec3", "c", ST_3D }, { "texture3DProj", "vec4", "c.xyz / c.w", ST_3D },
    { "shadow1D", "vec3", "vec3(c.x, 0.0, c.z)", ST_1DS }, { "shadow1DProj", "vec4", "vec3(c.x / c.w, 0.0, c.z / c.w)", ST_1DS }, { "shadow2D", "vec3", "c", ST_2DS }, { "shadow2DProj", "vec4", "c.xyz / c.w", ST_2DS },
};
#define N_BI_TEX ((int)(sizeof BI_TEX / sizeof BI_TEX[0]))
static int bi_tex_dim(int i) { int t = BI_TEX[i].sampler; return t == ST_1D || t == ST_1DS ? 1 : t == ST_3D ? 3 : 2; }

static void mat_transpose(float *o, const float *m) { int r, c; for (c = 0; c < 4; c++) for (r = 0; r < 4; r++) o[c * 4 + r] = m[r * 4 + c]; }

static void bi_matrix(int base, int idx, float *o)
{
    const float *mv = C->mv[C->mv_sp], *pr = C->pr[C->pr_sp];
    switch (base) {
    case 0: memcpy(o, mv, 64); break;
    case 1: memcpy(o, pr, 64); break;
    case 2: gl_mul(o, pr, mv); break;
    default: memcpy(o, C->unit[idx].mat[C->unit[idx].mat_sp], 64); break;
    }
}

/* the value of a built-in; 4 floats a row */
static void bi_fill(int k, int i, float *o)
{
    float t[16];
    memset(o, 0, 64);
    if (k < B_NM) {
        float m[16];
        int form = k >> 2;
        bi_matrix(k & 3, i, m);
        if (form & 1) { memcpy(t, m, 64); if (!gl_invert(m, t)) gl_ident(m); }
        if (form & 2) { memcpy(t, m, 64); mat_transpose(m, t); }
        memcpy(o, m, 64);
        return;
    }
    if (k == B_NM || k == B_NS) {                               /* the inverse transpose of the modelview's upper 3 x 3, as the fixed pipeline's */
        float inv[16];
        int r, c;
        gl_ident(inv);
        gl_invert(inv, C->mv[C->mv_sp]);
        if (k == B_NS) { o[0] = 1.0f / sqrtf(inv[2] * inv[2] + inv[6] * inv[6] + inv[10] * inv[10]); return; }
        for (c = 0; c < 3; c++) for (r = 0; r < 3; r++) o[c * 4 + r] = inv[r * 4 + c];
        return;
    }
    if (k == B_CLIP) { memcpy(o, C->clip_plane[i], 16); return; }
    if (k == B_ENV) { memcpy(o, C->unit[i].env_color, 16); return; }
    if (k >= B_EYE && k < B_OBJ) { memcpy(o, C->unit[i].gen_eye[k - B_EYE], 16); return; }
    if (k >= B_OBJ && k < B_DR) { memcpy(o, C->unit[i].gen_obj[k - B_OBJ], 16); return; }
    if (k >= B_DR && k < B_PT) { o[0] = k == B_DR ? C->depth_near : k == B_DR + 1 ? C->depth_far : C->depth_far - C->depth_near; return; }
    if (k >= B_PT && k < B_MF) {
        const float v[7] = { C->point_size, C->pt_min, C->pt_max, C->pt_fade, C->pt_att[0], C->pt_att[1], C->pt_att[2] };
        o[0] = v[k - B_PT];
        return;
    }
    if (k >= B_MF && k < B_LS) {
        const gl_material *m = &C->mat[k >= B_MB];
        int w = (k - B_MF) % 5;
        if (w == 4) o[0] = m->shininess;
        else memcpy(o, w == 0 ? m->emission : w == 1 ? m->ambient : w == 2 ? m->diffuse : m->specular, 16);
        return;
    }
    if (k >= B_LS && k < B_LM) {
        const gl_light *l = &C->light[i];
        switch (k - B_LS) {
        case 0: memcpy(o, l->ambient, 16); break;
        case 1: memcpy(o, l->diffuse, 16); break;
        case 2: memcpy(o, l->specular, 16); break;
        case 3: memcpy(o, l->position, 16); break;
        case 4: {                                               /* §2.14.1: the direction to the light and to the viewer at infinity, halfway */
            float n = sqrtf(l->position[0] * l->position[0] + l->position[1] * l->position[1] + l->position[2] * l->position[2]), h[3], m;
            if (n == 0.0f) n = 1.0f;
            h[0] = l->position[0] / n; h[1] = l->position[1] / n; h[2] = l->position[2] / n + 1.0f;
            m = sqrtf(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
            if (m == 0.0f) m = 1.0f;
            o[0] = h[0] / m; o[1] = h[1] / m; o[2] = h[2] / m; o[3] = 1.0f;
            break;
        }
        case 5: memcpy(o, l->spot_dir, 12); break;
        case 6: o[0] = l->spot_exp; break;
        case 7: o[0] = l->spot_cut; break;
        case 8: o[0] = cosf(l->spot_cut * 3.14159265358979f / 180.0f); break;
        case 9: o[0] = l->atten[0]; break;
        case 10: o[0] = l->atten[1]; break;
        default: o[0] = l->atten[2]; break;
        }
        return;
    }
    if (k == B_LM) { memcpy(o, C->lm_ambient, 16); return; }
    if (k == B_LMPF || k == B_LMPB) {
        const gl_material *m = &C->mat[k == B_LMPB];
        int c;
        for (c = 0; c < 3; c++) o[c] = m->emission[c] + m->ambient[c] * C->lm_ambient[c];
        o[3] = m->diffuse[3];
        return;
    }
    if (k >= B_LPF && k < B_FOG) {
        int w = (k - B_LPF) % 3, c;
        const gl_material *m = &C->mat[k >= B_LPB];
        const gl_light *l = &C->light[i];
        const float *a = w == 0 ? m->ambient : w == 1 ? m->diffuse : m->specular, *b = w == 0 ? l->ambient : w == 1 ? l->diffuse : l->specular;
        for (c = 0; c < 4; c++) o[c] = a[c] * b[c];
        return;
    }
    switch (k - B_FOG) {
    case 0: memcpy(o, C->fog_color, 16); break;
    case 1: o[0] = C->fog_density; break;
    case 2: o[0] = C->fog_start; break;
    case 3: o[0] = C->fog_end; break;
    default: o[0] = C->fog_end != C->fog_start ? 1.0f / (C->fog_end - C->fog_start) : 0.0f; break;
    }
}

/* ---------------------------------------------------------------- reading a shader */
static int id0(int c) { return isalpha(c) || c == '_'; }
static int idc(int c) { return isalnum(c) || c == '_'; }

/* past white space and comments */
static const char *skip(const char *p)
{
    for (;;) {
        while (*p && isspace((unsigned char)*p)) p++;
        if (p[0] == '/' && p[1] == '/') { while (*p && *p != '\n') p++; continue; }
        if (p[0] == '/' && p[1] == '*') { p += 2; while (*p && !(p[0] == '*' && p[1] == '/')) p++; if (*p) p += 2; continue; }
        return p;
    }
}

static int is_word(const char *p, size_t n, const char *w) { return strlen(w) == n && !memcmp(p, w, n); }

static int samp_type(const char *p, size_t n)
{
    if (is_word(p, n, "sampler1D")) return ST_1D;
    if (is_word(p, n, "sampler2D")) return ST_2D;
    if (is_word(p, n, "sampler3D")) return ST_3D;
    if (is_word(p, n, "samplerCube")) return ST_CUBE;
    if (is_word(p, n, "sampler1DShadow")) return ST_1DS;
    if (is_word(p, n, "sampler2DShadow")) return ST_2DS;
    return -1;
}

/* gl_X [ n ] with a plain number: the number, else -1 */
static int literal_index(const char *p)
{
    int v = 0, any = 0;
    p = skip(p);
    if (*p != '[') return -1;
    p = skip(p + 1);
    while (isdigit((unsigned char)*p)) { v = v * 10 + *p++ - '0'; any = 1; }
    p = skip(p);
    return any && *p == ']' ? v : -1;
}

/* what a stage reads and writes of the fixed pipeline, and the samplers it declares */
static void scan(const char *s, gs_info *o, gs_prog *p)
{
    const char *q = s;
    memset(o, 0, sizeof *o);
    for (;;) {
        const char *e;
        size_t n;
        q = skip(q);
        if (!*q) break;
        if (isdigit((unsigned char)*q)) { while (idc(*q) || *q == '.') q++; continue; }
        if (!id0(*q)) { q++; continue; }
        for (e = q; idc(*e); e++) {}
        n = (size_t)(e - q);
        if (is_word(q, n, "main")) o->has_main = 1;
        else if (is_word(q, n, "gl_TexCoord")) { int i = literal_index(e); o->tc |= i >= 0 && i < MAX_UNITS ? 1u << i : (1u << MAX_UNITS) - 1; }
        else if (is_word(q, n, "gl_Color")) o->color = 1;
        else if (is_word(q, n, "gl_SecondaryColor")) o->sec = 1;
        else if (is_word(q, n, "gl_FogFragCoord")) o->fogf = 1;
        else if (is_word(q, n, "gl_FrontColor")) o->front = 1;
        else if (is_word(q, n, "gl_BackColor")) o->back = 1;
        else if (is_word(q, n, "gl_FrontSecondaryColor")) o->fsec = 1;
        else if (is_word(q, n, "gl_BackSecondaryColor")) o->bsec = 1;
        else if (is_word(q, n, "gl_ClipVertex")) o->clipv = 1;
        else if (is_word(q, n, "gl_PointSize")) o->psize = 1;
        else if (is_word(q, n, "GL_ARB_shader_texture_lod")) o->lodx = 1;
        else if (is_word(q, n, "uniform") && p) {
            const char *t = skip(e), *te = t;
            int ty;
            while (idc(*te)) te++;
            ty = samp_type(t, (size_t)(te - t));
            if (ty >= 0) {
                const char *w = te;
                for (;;) {
                    const char *ne;
                    int count = 0;
                    w = skip(w);
                    for (ne = w; idc(*ne); ne++) {}
                    if (ne == w) break;
                    {
                        const char *b = skip(ne);
                        if (*b == '[') { count = atoi(skip(b + 1)); while (*b && *b != ']') b++; if (*b) b++; }
                        if (p->nsamp < MAX_UNITS) {
                            gs_samp *m = &p->samp[p->nsamp];
                            int k, dup = 0;
                            for (k = 0; k < p->nsamp; k++) if (is_word(w, (size_t)(ne - w), p->samp[k].name)) dup = 1;
                            if (!dup) {
                                size_t l = (size_t)(ne - w) < sizeof m->name - 1 ? (size_t)(ne - w) : sizeof m->name - 1;
                                memcpy(m->name, w, l); m->name[l] = 0;
                                m->type = ty; m->count = count; m->first = p->nel;
                                p->nel += count ? count : 1;
                                p->nsamp++;
                            }
                        }
                        ne = skip(b);
                    }
                    if (*ne != ',') break;
                    w = ne + 1;
                }
                e = te;
            }
        }
        q = e;
    }
}

/* ---------------------------------------------------------------- translation */
typedef struct xl {
    gs_prog *p;
    const gs_key *k;
    int fs;
    tb head, body;
    uint8_t attr[N_BI_ATTR], uni[N_BI_UNI], mem_max[N_BI_MEM];  /* 1 + the highest index of a member, or its count; 0 unused */
    uint8_t mem_used[N_BI_MEM], st_type[N_BI_STRUCT], st_var[N_BI_STRUCT];
    const char *src;                                            /* the stage's text, where an unsized array finds its size */
    uint32_t tex_fn[MAX_UNITS];                                 /* which of BI_TEX each sampler is asked for */
    uint32_t tex_lod[MAX_UNITS], tex_grad[MAX_UNITS];           /* and in the forms of GL_ARB_shader_texture_lod */
    uint32_t cube_fn, tsz;                                      /* tsz: the samplers that want their size */
    int ftransform, noise, t1, t1p, t1l, grad;
    uint32_t el_mode, el_decl;                                  /* el_mode: arrays (gs_samp index) read element by element; el_decl: the elements the TMU reads, each as g2Tex<s> */
    uint32_t arr_fn[MAX_UNITS], arr_lod[MAX_UNITS], arr_grad[MAX_UNITS];   /* an element chosen at run time: g2a<m>_<function>, by BI_TEX */
    uint8_t arr_cube[MAX_UNITS];                                /* the same for textureCube: 1 plain, 2 Lod, 4 GradARB */
    uint32_t shadow_tmu;                                        /* shadow functions read as colour, BI_TEX bits, Lod from bit 16: a texture that is not a depth texture, which §3.8.14 leaves undefined */
    int spec;                                                   /* a variant: functions that take a sampler are copied per sampler (sp_run); glCompileShader checks the text as written */
    gl_ureq ur[64];
    int nur;
    char err[256];
} xl;

static gs_samp *samp_of(const gs_prog *p, const char *q, size_t n)
{
    int i;
    for (i = 0; i < p->nsamp; i++) if (is_word(q, n, p->samp[i].name)) return (gs_samp *)&p->samp[i];
    return NULL;
}

static void xl_run(xl *x, const char *s);

static int fn_is_tex(const char *q, size_t n)
{
    int i;
    for (i = 0; i < N_BI_TEX; i++) if (is_word(q, n, BI_TEX[i].name)) return 1;
    return is_word(q, n, "shadow1DLod") || is_word(q, n, "shadow2DLod");
}

/* GL_ARB_shader_texture_lod. 1: base + "Lod", 2: base + "GradARB", 0: neither; base is texture2D, textureCube and so on */
static int lod_form(const char *q, size_t n, char *base, size_t cap)
{
    static const char *const SUF[2] = { "Lod", "GradARB" };
    int f, i;
    for (f = 0; f < 2; f++) {
        size_t l = strlen(SUF[f]);
        if (n <= l || n - l >= cap || memcmp(q + n - l, SUF[f], l)) continue;
        memcpy(base, q, n - l);
        base[n - l] = 0;
        if (!strcmp(base, "textureCube")) return f + 1;
        for (i = 0; i < N_BI_TEX; i++) if (!strcmp(base, BI_TEX[i].name)) return f + 1;
    }
    return 0;
}

/* gradients to a level on the TMU: g2_g<n>(sampler, its size, coordinate, dx, dy) */
static const struct { const char *base, *fn; } GRAD[] = { { "texture1D", "g2_g1" }, { "texture1DProj", "g2_g1p" }, { "texture2D", "g2_g2" }, { "texture2DProj", "g2_g2p" }, { "texture3D", "g2_g3" }, { "texture3DProj", "g2_g3p" }, { "textureCube", "g2_gc" } };
#define N_GRAD ((int)(sizeof GRAD / sizeof GRAD[0]))

static void mark_fn(uint32_t *m, const char *base)              /* every overload of a name */
{
    int i;
    for (i = 0; i < N_BI_TEX; i++) if (!strcmp(base, BI_TEX[i].name)) *m |= 1u << i;
}

static void shadow_mark(xl *x, const char *base, int lod)
{
    int i;
    for (i = 0; i < N_BI_TEX; i++) if (!strcmp(base, BI_TEX[i].name)) x->shadow_tmu |= 1u << (i + (lod ? 16 : 0));
}

/* a call of sampler index s on its own, up to its first argument: q the function as written, base without the Lod or GradARB of form 1 or 2. An element of an array the TMU reads is g2Tex<s>, so the array
 * is not one to the compiler, which takes no index but a constant */
static void elem_head(xl *x, tb *b, int s, const char *q, size_t n, int form, const char *base)
{
    int cube = !strcmp(base, "textureCube"), i;
    if (!x->k->sk.u[s].target) { tb_f(b, "g2c%d_", s); tb_put(b, q, n); tb_f(b, "("); x->cube_fn |= 1u << s; return; }
    if (x->k->sk.u[s].sw && !cube) {
        mark_fn(form == 0 ? &x->tex_fn[s] : form == 1 ? &x->tex_lod[s] : &x->tex_grad[s], base);
        if (form == 2) x->grad = 1;
        tb_f(b, "g2t%d_", s); tb_put(b, q, n); tb_f(b, "(");
        return;
    }
    x->el_decl |= 1u << s;
    if (form == 2) {
        for (i = 0; i < N_GRAD; i++) if (!strcmp(base, GRAD[i].base)) break;
        if (i == N_GRAD) { snprintf(x->err, sizeof x->err, "%.*s of a depth texture needs a complete one", (int)n, q); return; }
        x->grad = 1; x->tsz |= 1u << s;
        tb_f(b, "%s(g2Tex%d, g2Tsz%d, ", GRAD[i].fn, s, s);
        if (cube) { if (x->k->lbias[s]) tb_f(b, "g2Lod%d.x, ", s); else tb_f(b, "0.0, "); }
        return;
    }
    if (cube && x->k->lbias[s]) { tb_f(b, "g2c%d_", s); tb_put(b, q, n); tb_f(b, "("); x->cube_fn |= 1u << s; return; }
    if (!strncmp(base, "shadow", 6)) { shadow_mark(x, base, form); tb_f(b, "g2_"); }
    else if (!strncmp(base, "texture1D", 9)) { if (form) x->t1l = 1; else if (!strcmp(base, "texture1D")) x->t1 = 1; else x->t1p = 1; tb_f(b, "g2_"); }
    tb_put(b, q, n); tb_f(b, "(g2Tex%d, ", s);
}

/* a call on an element of an array read element by element: a number for an index reads that element, anything else g2a<m>_<function>(index, ...). ne is past the array's name, *pp at the call's parenthesis */
static int xl_elem(xl *x, const char *q, size_t n, const gs_samp *m, const char *ne, int form, const char *base, const char **pp)
{
    tb *b = &x->body;
    const char *ib = skip(ne), *ie, *c;
    int depth = 1, lit, mi = (int)(m - x->p->samp);
    if (*ib != '[') return 0;
    for (ie = ib + 1; *ie && depth; ie++) { if (*ie == '[') depth++; else if (*ie == ']') depth--; }
    if (depth) return 0;
    c = skip(ie);
    if (*c != ',') return 0;
    lit = literal_index(ne);
    if (lit >= 0 && lit < m->count) { elem_head(x, b, m->first + lit, q, n, form, base); *pp = c + 1; return 1; }
    if (!strcmp(base, "textureCube")) x->arr_cube[mi] |= (uint8_t)(1 << form);
    else mark_fn(form == 0 ? &x->arr_fn[mi] : form == 1 ? &x->arr_lod[mi] : &x->arr_grad[mi], base);
    tb_f(b, "g2a%d_", mi); tb_put(b, q, n); tb_f(b, "(");
    {
        size_t l = (size_t)(ie - 1 - (ib + 1));
        char *e = (char *)malloc(l + 1);
        if (e) { memcpy(e, ib + 1, l); e[l] = 0; xl_run(x, e); free(e); }
    }
    tb_f(b, ", ");
    *pp = c + 1;
    return 1;
}

/* a level or gradients in a fragment shader: the TMU takes a level from the program. A sampler the program filters takes it in g2sl<s>, gradients come to a level here. *pp is at the call's parenthesis */
static int xl_lod(xl *x, const char *q, size_t n, const char **pp)
{
    tb *b = &x->body;
    char base[32];
    const char *a = skip(*pp), *ne = a, *c;
    gs_samp *m = NULL;
    int form = lod_form(q, n, base, sizeof base), s = 0, i, cube;
    if (!form) return 0;
    if (!x->p->info[1].lodx) { snprintf(x->err, sizeof x->err, "%.*s needs GL_ARB_shader_texture_lod", (int)n, q); return 1; }
    if (*a == '(') {
        a = skip(a + 1);
        for (ne = a; idc(*ne); ne++) {}
        m = ne > a ? samp_of(x->p, a, (size_t)(ne - a)) : NULL;
    }
    if (!m) {                                                   /* a sampler handed to a function: only the compiler sees it; one that needs the library is copied in (sp_run) */
        if (form == 2) {
            if (x->spec) snprintf(x->err, sizeof x->err, "%.*s takes a uniform sampler", (int)n, q);
            else tb_put(b, q, n);                               /* glCompileShader: a variant copies the function */
            return 1;
        }
        if (!strncmp(base, "shadow", 6)) { shadow_mark(x, base, 1); tb_f(b, "g2_"); }
        else if (!strncmp(base, "texture1D", 9)) { x->t1l = 1; tb_f(b, "g2_"); }
        tb_put(b, q, n);
        return 1;
    }
    if (m->count && (x->el_mode >> (m - x->p->samp) & 1) && xl_elem(x, q, n, m, ne, form, base, pp)) return 1;
    c = skip(ne);
    if (*c == '[') {
        int l = literal_index(ne);
        s = l > 0 ? l : 0;
        while (*c && *c != ']') c++;
        if (*c) c++;
        c = skip(c);
    }
    s += m->first;
    cube = !strcmp(base, "textureCube");
    if (!m->count && !x->k->sk.u[s].target) {                   /* an incomplete texture: a function that says so; an array's element, the TMU's sampler that reads (0, 0, 0, 1) */
        tb_f(b, "g2c%d_", s); tb_put(b, q, n); tb_f(b, "(");
        x->cube_fn |= 1u << s;
        *pp = c + 1;
        return 1;
    }
    if (x->k->sk.u[s].sw && !m->count) {
        for (i = 0; i < N_BI_TEX; i++) if (!strcmp(base, BI_TEX[i].name)) { if (form == 1) x->tex_lod[s] |= 1u << i; else x->tex_grad[s] |= 1u << i; }
        if (form == 2) x->grad = 1;
        tb_f(b, "g2t%d_", s); tb_put(b, q, n); tb_f(b, "(");
        *pp = c + 1;
        return 1;
    }
    if (form == 1) {
        if (cube && x->k->lbias[s]) {                           /* the level, then the bias of the texture */
            tb_f(b, "g2c%d_textureCubeLod(", s);
            x->cube_fn |= 1u << s;
            *pp = c + 1;
            return 1;
        }
        if (!strncmp(base, "texture1D", 9)) { x->t1l = 1; tb_f(b, "g2_"); }
        else if (!strncmp(base, "shadow", 6)) { shadow_mark(x, base, 1); tb_f(b, "g2_"); }
        tb_put(b, q, n);
        return 1;
    }
    for (i = 0; i < N_GRAD; i++) if (!strcmp(base, GRAD[i].base)) break;
    if (i == N_GRAD) { snprintf(x->err, sizeof x->err, "%.*s of a depth texture needs a complete one", (int)n, q); return 1; }
    x->grad = 1;
    x->tsz |= 1u << s;
    tb_f(b, "%s(", GRAD[i].fn);
    tb_put(b, a, (size_t)(c - a));
    tb_f(b, ", g2Tsz%d", s);
    if (cube) { if (x->k->lbias[s]) tb_f(b, ", g2Lod%d.x", s); else tb_f(b, ", 0.0"); }
    *pp = c;
    return 1;
}

/* an identifier of the shader. *pp is past it; a rewrite may take more */
static void xl_ident(xl *x, const char *q, size_t n, const char **pp)
{
    tb *b = &x->body;
    int i;
    if (n > 3 && !memcmp(q, "gl_", 3)) {
        for (i = 0; i < (int)(sizeof BI_CONST / sizeof BI_CONST[0]); i++) if (is_word(q, n, BI_CONST[i].name)) { tb_f(b, "%d", BI_CONST[i].v); return; }
        if (!x->fs) {
            for (i = 0; i < N_BI_ATTR; i++) if (BI_ATTR[i].name && is_word(q, n, BI_ATTR[i].name)) { x->attr[i] = 1; tb_f(b, "%s", BI_ATTR[i].as); return; }
            if (is_word(q, n, "gl_MultiTexCoord4") || is_word(q, n, "gl_MultiTexCoord5") || is_word(q, n, "gl_MultiTexCoord6") || is_word(q, n, "gl_MultiTexCoord7")) { tb_f(b, "vec4(0.0, 0.0, 0.0, 1.0)"); return; }
            if (is_word(q, n, "gl_FrontColor")) { tb_f(b, "g2o_FC"); return; }
            if (is_word(q, n, "gl_BackColor")) { tb_f(b, "g2o_BC"); return; }
            if (is_word(q, n, "gl_FrontSecondaryColor")) { tb_f(b, "g2o_FS"); return; }
            if (is_word(q, n, "gl_BackSecondaryColor")) { tb_f(b, "g2o_BS"); return; }
            if (is_word(q, n, "gl_TexCoord")) { tb_f(b, "g2o_Tc"); return; }
            if (is_word(q, n, "gl_FogFragCoord")) { tb_f(b, "g2o_Fog"); return; }
            if (is_word(q, n, "gl_ClipVertex")) { tb_f(b, "g2o_Clip"); return; }
            if (is_word(q, n, "gl_PointSize")) { tb_f(b, "g2o_PS"); return; }
        } else {
            if (is_word(q, n, "gl_Color")) { tb_f(b, "g2i_Color"); return; }
            if (is_word(q, n, "gl_SecondaryColor")) { tb_f(b, "g2i_Sec"); return; }
            if (is_word(q, n, "gl_TexCoord")) { tb_f(b, "g2i_Tc"); return; }
            if (is_word(q, n, "gl_FogFragCoord")) { tb_f(b, "g2i_Fog"); return; }
        }
        for (i = 0; i < N_BI_UNI; i++) {
            if (is_word(q, n, BI_UNI[i].name)) {
                x->uni[i] = 1;
                tb_f(b, "g2u_%s", BI_UNI[i].name + 3);
                return;
            }
        }
        for (i = 0; i < N_BI_STRUCT; i++) if (is_word(q, n, BI_STRUCT[i].type)) { x->st_type[i] = 1; tb_f(b, "g2s_"); tb_put(b, q + 3, n - 3); return; }
        for (i = 0; i < N_BI_MEM; i++) {
            if (is_word(q, n, BI_MEM[i].var)) {                 /* gl_LightSource[expr].diffuse */
                const char *p = skip(*pp), *ix = NULL, *ie = NULL, *m, *me;
                int depth, k;
                if (*p == '[') {
                    ix = p + 1;
                    for (depth = 1, p++; *p && depth; p++) { if (*p == '[') depth++; else if (*p == ']') depth--; }
                    ie = p - 1;
                    p = skip(p);
                }
                if (*p != '.') {                                /* the structure whole: a function that puts it together */
                    int t, j;
                    for (t = 0; t < N_BI_STRUCT; t++) if (is_word(q, n, BI_STRUCT[t].var)) break;
                    x->st_var[t] = 1; x->st_type[t] = 1;
                    for (j = 0; j < N_BI_STRUCT; j++) if (!strcmp(BI_STRUCT[j].type, BI_STRUCT[t].type)) x->st_type[j] = 1;
                    for (j = 0; j < N_BI_MEM; j++) if (!strcmp(BI_MEM[j].var, BI_STRUCT[t].var)) { x->mem_used[j] = 1; x->mem_max[j] = (uint8_t)BI_MEM[j].count; }
                    tb_f(b, "g2b_%s(", BI_STRUCT[t].var + 3);
                    if (ix) {
                        char *e = (char *)malloc((size_t)(ie - ix) + 1);
                        if (e) { memcpy(e, ix, (size_t)(ie - ix)); e[ie - ix] = 0; xl_run(x, e); free(e); }
                    }
                    tb_f(b, ")");
                    *pp = p;
                    return;
                }
                m = skip(p + 1);
                for (me = m; idc(*me); me++) {}
                for (k = 0; k < N_BI_MEM; k++) {
                    if (is_word(q, n, BI_MEM[k].var) && is_word(m, (size_t)(me - m), BI_MEM[k].mem)) break;
                }
                if (k == N_BI_MEM) { snprintf(x->err, sizeof x->err, "%.*s has no member %.*s", (int)n, q, (int)(me - m), m); return; }
                x->mem_used[k] = 1;
                tb_f(b, "g2u_%s_%s", BI_MEM[k].var + 3, BI_MEM[k].mem);
                if (ix) {
                    int lit = literal_index(ix - 1);
                    char *e = (char *)malloc((size_t)(ie - ix) + 1);
                    if (e) {
                        memcpy(e, ix, (size_t)(ie - ix)); e[ie - ix] = 0;
                        tb_f(b, "[");
                        xl_run(x, e);
                        tb_f(b, "]");
                        free(e);
                    }
                    k = k < N_BI_MEM ? k : 0;
                    if (lit < 0 || lit >= BI_MEM[k].count) x->mem_max[k] = (uint8_t)BI_MEM[k].count;
                    else if (lit + 1 > x->mem_max[k]) x->mem_max[k] = (uint8_t)(lit + 1);
                }
                *pp = me;
                return;
            }
        }
        tb_put(b, q, n);                                        /* gl_Position, gl_FragColor and the like: the compiler's */
        return;
    }
    if (is_word(q, n, "main")) { tb_f(b, "g2_main"); return; }
    if (is_word(q, n, "ftransform")) { x->ftransform = 1; tb_f(b, "g2_ftransform"); return; }
    if (n == 6 && !memcmp(q, "noise", 5) && q[5] >= '1' && q[5] <= '4') { x->noise = 1; tb_f(b, "g2_noise%c", q[5]); return; }
    if (samp_type(q, n) >= 0) {
        tb_f(b, is_word(q, n, "samplerCube") ? "samplerCube" : "sampler2D");
        return;
    }
    if (is_word(q, n, "varying")) {                             /* varying vec4 gl_TexCoord[3]; says how many are used: nothing here needs to know */
        const char *t = skip(*pp), *te = t, *w, *we;
        while (idc(*te)) te++;
        w = skip(te);
        for (we = w; idc(*we); we++) {}
        if (we - w > 3 && !memcmp(w, "gl_", 3)) {
            while (*we && *we != ';') we++;
            *pp = *we ? we + 1 : we;
            return;
        }
        tb_put(b, q, n);
        return;
    }
    if (is_word(q, n, "uniform")) {
        const char *t = skip(*pp), *te = t;
        while (idc(*te)) te++;
        if (x->fs && samp_type(t, (size_t)(te - t)) >= 0 && samp_type(t, (size_t)(te - t)) != ST_CUBE) {   /* each name by what it is now: what the program filters itself is declared by gl_ff.c */
            const char *w = te;
            for (;;) {
                const char *ne, *be;
                gs_samp *m;
                w = skip(w);
                for (ne = w; idc(*ne); ne++) {}
                be = skip(ne);
                if (*be == '[') { while (*be && *be != ']') be++; if (*be) be++; }
                m = samp_of(x->p, w, (size_t)(ne - w));
                if (m && (m->count ? !(x->el_mode >> (m - x->p->samp) & 1) : (x->k->sk.u[m->first].sw == 0 || !x->spec) && x->k->sk.u[m->first].target)) {
                    tb_f(b, m->type == ST_3D ? "uniform sampler3D " : "uniform sampler2D ");
                    tb_put(b, w, (size_t)(be - w));
                    tb_f(b, ";");
                } else if (m && !m->count && !x->k->sk.u[m->first].target) tb_f(b, " ");
                w = skip(be);
                if (*w != ',') break;
                w++;
            }
            while (*w && *w != ';') w++;
            if (*w) w++;
            *pp = w;
            return;
        }
        tb_put(b, q, n);
        return;
    }
    if (x->fs && xl_lod(x, q, n, pp)) return;
    if (fn_is_tex(q, n) || is_word(q, n, "textureCube")) {
        const char *a = skip(*pp), *ne;
        if (*a == '(') {
            a = skip(a + 1);
            for (ne = a; idc(*ne); ne++) {}
            {
                gs_samp *m = ne > a ? samp_of(x->p, a, (size_t)(ne - a)) : NULL;
                const char *c = skip(ne);
                if (m && m->count && *c == '[' && (x->el_mode >> (m - x->p->samp) & 1)) {
                    char base[32];
                    snprintf(base, sizeof base, "%.*s", (int)n, q);
                    if (xl_elem(x, q, n, m, ne, 0, base, pp)) return;
                }
                if (m && !m->count && *c == ',') {
                    int s = m->first, sw = x->k->sk.u[s].sw, target = x->k->sk.u[s].target;
                    if (!target || (is_word(q, n, "textureCube") && x->k->lbias[s])) {   /* an incomplete texture: a function that says so; a cube's bias */
                        tb_f(b, "g2c%d_", s);
                        tb_put(b, q, n);
                        tb_f(b, "(");
                        x->cube_fn |= 1u << s;
                        *pp = c + 1;
                        return;
                    }
                    if (sw) {
                        for (i = 0; i < N_BI_TEX; i++) if (is_word(q, n, BI_TEX[i].name)) x->tex_fn[s] |= 1u << i;
                        tb_f(b, "g2t%d_", s);
                        tb_put(b, q, n);
                        tb_f(b, "(");
                        *pp = c + 1;
                        return;
                    }
                }
            }
        }
        if (is_word(q, n, "texture1D")) { x->t1 = 1; tb_f(b, "g2_texture1D"); return; }
        if (is_word(q, n, "texture1DProj")) { x->t1p = 1; tb_f(b, "g2_texture1DProj"); return; }
        if (n > 6 && !memcmp(q, "shadow", 6)) {                 /* what the TMU reads as colour */
            char base[32];
            snprintf(base, sizeof base, "%.*s", (int)n, q);
            shadow_mark(x, base, 0);
            tb_f(b, "g2_");
        }
    }
    tb_put(b, q, n);
}

/* a[] : as large as its largest constant index, §4.1.9 */
static int unsized_len(const char *src, const char *name, size_t n)
{
    const char *p = src;
    int most = 0;
    while ((p = strstr(p, name)) != NULL) {
        const char *e = p + n;
        if ((p == src || !idc(p[-1])) && !idc(*e)) {
            int i = literal_index(e);
            if (i + 1 > most) most = i + 1;
        }
        p = e;
    }
    return most ? most : 1;
}

static void xl_run(xl *x, const char *s)
{
    const char *p = s, *last = NULL;
    size_t last_n = 0;
    tb *b = &x->body;
    int start = 1;
    while (*p) {
        if (p[0] == '/' && p[1] == '/') { const char *e = p; while (*e && *e != '\n') e++; tb_put(b, p, (size_t)(e - p)); p = e; continue; }
        if (p[0] == '/' && p[1] == '*') { const char *e = p + 2; while (*e && !(e[0] == '*' && e[1] == '/')) e++; if (*e) e += 2; tb_put(b, p, (size_t)(e - p)); p = e; continue; }
        if (*p == '#' && start) {
            const char *q = p + 1;
            while (*q == ' ' || *q == '\t') q++;
            if (!strncmp(q, "version", 7) || !strncmp(q, "extension", 9)) { while (*p && *p != '\n') p++; continue; }
        }
        if (isdigit((unsigned char)*p) || (*p == '.' && isdigit((unsigned char)p[1]))) {
            const char *e = p;
            while (idc(*e) || *e == '.' || ((*e == '+' || *e == '-') && (e[-1] == 'e' || e[-1] == 'E'))) e++;
            tb_put(b, p, (size_t)(e - p));
            p = e;
            start = 0;
            continue;
        }
        if (id0(*p)) {
            const char *e = p;
            while (idc(*e)) e++;
            {
                const char *after = e;
                xl_ident(x, p, (size_t)(e - p), &after);
                last = p; last_n = (size_t)(e - p);
                p = after;
            }
            start = 0;
            continue;
        }
        if (*p == '[' && last && *skip(p + 1) == ']' && x->src) {
            char nm[96];
            snprintf(nm, sizeof nm, "%.*s", (int)last_n, last);
            tb_f(b, "[%d]", unsized_len(x->src, nm, last_n));
            p = skip(p + 1) + 1;
            continue;
        }
        if (*p == '\n') start = 1; else if (!isspace((unsigned char)*p)) start = 0;
        tb_put(b, p, 1);
        p++;
    }
}

/* the structures a shader takes whole: their types, then what makes one from its members, which the uniforms above hold */
static void xl_structs(xl *x, tb *h)
{
    int i, j, k;
    for (i = 0; i < N_BI_STRUCT; i++) {
        int done = 0;
        if (!x->st_type[i]) continue;
        for (k = 0; k < i; k++) if (x->st_type[k] && !strcmp(BI_STRUCT[k].type, BI_STRUCT[i].type)) done = 1;
        if (done) continue;
        tb_f(h, "struct g2s_%s {\n", BI_STRUCT[i].type + 3);
        for (j = 0; j < N_BI_MEM; j++) if (!strcmp(BI_MEM[j].var, BI_STRUCT[i].var)) tb_f(h, "  %s %s;\n", BI_MEM[j].ty, BI_MEM[j].mem);
        tb_f(h, "};\n");
    }
    for (i = 0; i < N_BI_STRUCT; i++) {
        int first = 1, arr = 0;
        if (!x->st_var[i]) continue;
        for (j = 0; j < N_BI_MEM; j++) if (!strcmp(BI_MEM[j].var, BI_STRUCT[i].var) && BI_MEM[j].count) arr = 1;
        tb_f(h, "g2s_%s g2b_%s(%s) { return g2s_%s(", BI_STRUCT[i].type + 3, BI_STRUCT[i].var + 3, arr ? "int i" : "", BI_STRUCT[i].type + 3);
        for (j = 0; j < N_BI_MEM; j++) {
            if (strcmp(BI_MEM[j].var, BI_STRUCT[i].var)) continue;
            tb_f(h, "%sg2u_%s_%s%s", first ? "" : ", ", BI_MEM[j].var + 3, BI_MEM[j].mem, arr ? "[i]" : "");
            first = 0;
        }
        tb_f(h, "); }\n");
    }
}

static void need_uni(xl *x, tb *h, const char *ty, const char *name, int kind, int idx)
{
    gl_ureq *q = &x->ur[x->nur++];
    snprintf(q->name, sizeof q->name, "%s", name);
    q->kind = (uint8_t)kind; q->idx = (uint8_t)idx;
    tb_f(h, "uniform %s %s;\n", ty, name);
}

/* what the fragment side takes from the vertex side */
typedef struct gs_need { int color, colorb, spec, specb, fogf, fog; uint32_t tc; } gs_need;

static void need_of(const gs_prog *p, const gs_key *k, gs_need *n)
{
    int u;
    memset(n, 0, sizeof *n);
    if (p->has_fs) {
        const gs_info *f = &p->info[1];
        n->color = f->color; n->colorb = f->color && k->two; n->spec = f->sec; n->specb = f->sec && k->two; n->fogf = f->fogf; n->tc = f->tc;
    } else {
        n->color = 1;
        n->colorb = k->ff.light && k->ff.two_side;
        n->spec = (k->ff.sep_spec && k->ff.light) || k->ff.sec;
        n->fog = k->ff.fog != 0;
        for (u = 0; u < MAX_UNITS; u++) if (k->ff.u[u].target) n->tc |= 1u << u;
    }
}

/* declarations the wrapped programs need, then the shader, then main */
static void xl_vs(xl *x, tb *out)
{
    const gs_prog *p = x->p;
    const gs_key *k = x->k;
    const ff_key *f = &k->ff;
    gs_need n;
    tb h = { 0 }, t = { 0 };
    int i;
    need_of(p, k, &n);
    tb_f(&h, "#extension GL_OES_texture_3D : enable\nvec4 g2o_FC, g2o_BC, g2o_FS, g2o_BS, g2o_Clip;\nvec4 g2o_Tc[%d];\nfloat g2o_Fog, g2o_PS;\n", MAX_UNITS);
    if (n.color) tb_f(&h, "varying vec4 g2vColor;\n");
    if (n.colorb) tb_f(&h, "varying vec4 g2vColorB;\n");
    if (n.spec) tb_f(&h, "varying vec4 g2vSpec;\n");
    if (n.specb) tb_f(&h, "varying vec4 g2vSpecB;\n");
    if (n.fogf) tb_f(&h, "varying vec4 g2vFogF;\n");
    if (n.fog) tb_f(&h, "varying vec4 g2vFog;\n");
    for (i = 0; i < MAX_UNITS; i++) if (n.tc >> i & 1) tb_f(&h, "varying vec4 g2vTc%d;\n", i);
    for (i = 0; i < MAX_CLIP; i++) if (f->clip_mask >> i & 1) { tb_f(&h, "varying vec4 g2vClip%d;\n", i); }
    if (f->line_stip) tb_f(&h, "varying vec4 g2vStip;\n");
    if (f->smooth_ln) tb_f(&h, "varying vec4 g2vLine;\n");
    x->src = p->src[0];
    xl_run(x, p->src[0]);
    if (x->ftransform) { x->attr[0] = 1; x->uni[2] = 1; }
    for (i = 0; i < N_BI_UNI; i++) {
        if (!x->uni[i]) continue;
        if (BI_UNI[i].count) tb_f(&h, "uniform %s g2u_%s[%d];\n", BI_UNI[i].ty, BI_UNI[i].name + 3, BI_UNI[i].count);
        else tb_f(&h, "uniform %s g2u_%s;\n", BI_UNI[i].ty, BI_UNI[i].name + 3);
    }
    for (i = 0; i < N_BI_MEM; i++) {
        if (!x->mem_used[i]) continue;
        if (BI_MEM[i].count) tb_f(&h, "uniform %s g2u_%s_%s[%d];\n", BI_MEM[i].ty, BI_MEM[i].var + 3, BI_MEM[i].mem, x->mem_max[i] ? x->mem_max[i] : BI_MEM[i].count);
        else tb_f(&h, "uniform %s g2u_%s_%s;\n", BI_MEM[i].ty, BI_MEM[i].var + 3, BI_MEM[i].mem);
    }
    xl_structs(x, &h);
    if (f->line_stip || f->smooth_ln) x->attr[9] = 1;
    if (f->smooth_ln) x->attr[10] = 1;
    if (k->flat) { if (n.color) x->attr[11] = 1; if (n.colorb) x->attr[12] = 1; if (n.spec) x->attr[13] = 1; if (n.specb) x->attr[14] = 1; }
    for (i = 0; i < N_BI_ATTR; i++) if (x->attr[i]) tb_f(&h, "attribute %s %s;\n", BI_ATTR[i].ty, BI_ATTR[i].as);
    if (x->ftransform) tb_f(&h, "vec4 g2_ftransform() { return g2u_ModelViewProjectionMatrix * g2a_Vertex; }\n");
    if (x->noise) tb_f(&h, "float g2_noise1(float x) { return 0.0; }\nfloat g2_noise1(vec2 x) { return 0.0; }\nfloat g2_noise1(vec3 x) { return 0.0; }\nfloat g2_noise1(vec4 x) { return 0.0; }\n"
                           "vec2 g2_noise2(float x) { return vec2(0.0); }\nvec2 g2_noise2(vec2 x) { return vec2(0.0); }\nvec2 g2_noise2(vec3 x) { return vec2(0.0); }\nvec2 g2_noise2(vec4 x) { return vec2(0.0); }\n"
                           "vec3 g2_noise3(float x) { return vec3(0.0); }\nvec3 g2_noise3(vec2 x) { return vec3(0.0); }\nvec3 g2_noise3(vec3 x) { return vec3(0.0); }\nvec3 g2_noise3(vec4 x) { return vec3(0.0); }\n"
                           "vec4 g2_noise4(float x) { return vec4(0.0); }\nvec4 g2_noise4(vec2 x) { return vec4(0.0); }\nvec4 g2_noise4(vec3 x) { return vec4(0.0); }\nvec4 g2_noise4(vec4 x) { return vec4(0.0); }\n");
    if (n.fog) need_uni(x, &h, "vec4", "g2_fog", U_FOG, 0);
    if (f->points) need_uni(x, &h, "vec4", "g2_pt", U_POINT, 0);
    if (f->zoff) need_uni(x, &h, "vec4", "g2_zo", U_Z, 0);
    for (i = 0; i < MAX_CLIP; i++) {
        if (f->clip_mask >> i & 1) {
            char nm[16];
            snprintf(nm, sizeof nm, "g2_cl%d", i);
            need_uni(x, &h, "vec4", nm, U_CLIP, i);
        }
    }
    tb_f(&t, "\nvoid main() {\n  g2o_FC = g2o_BC = g2o_FS = g2o_BS = vec4(0.0);\n  g2o_Fog = 0.0;\n  g2o_PS = 1.0;\n");
    for (i = 0; i < MAX_UNITS; i++) if (n.tc >> i & 1) tb_f(&t, "  g2o_Tc[%d] = vec4(0.0, 0.0, 0.0, 1.0);\n", i);
    tb_f(&t, "  g2_main();\n");
    if (n.color) tb_f(&t, k->flat ? "  g2vColor = g2aFlat0;\n" : "  g2vColor = clamp(g2o_FC, 0.0, 1.0);\n");
    if (n.colorb) tb_f(&t, k->flat ? "  g2vColorB = g2aFlat1;\n" : "  g2vColorB = clamp(g2o_BC, 0.0, 1.0);\n");
    if (n.spec) tb_f(&t, k->flat ? "  g2vSpec = g2aFlat2;\n" : "  g2vSpec = clamp(g2o_FS, 0.0, 1.0);\n");
    if (n.specb) tb_f(&t, k->flat ? "  g2vSpecB = g2aFlat3;\n" : "  g2vSpecB = clamp(g2o_BS, 0.0, 1.0);\n");
    if (n.fogf) tb_f(&t, "  g2vFogF = vec4(g2o_Fog);\n");
    if (n.fog) {
        tb_f(&t, "  float g2fz = abs(g2o_Fog);\n");
        if (f->fog == 1) tb_f(&t, "  g2vFog = vec4(clamp((g2_fog.z - g2fz) * g2_fog.w, 0.0, 1.0));\n");
        else if (f->fog == 2) tb_f(&t, "  g2vFog = vec4(clamp(exp(-g2_fog.x * g2fz), 0.0, 1.0));\n");
        else tb_f(&t, "  float g2dz = g2_fog.x * g2fz;\n  g2vFog = vec4(clamp(exp(-g2dz * g2dz), 0.0, 1.0));\n");
    }
    for (i = 0; i < MAX_UNITS; i++) if (n.tc >> i & 1) tb_f(&t, "  g2vTc%d = g2o_Tc[%d];\n", i, i);
    for (i = 0; i < MAX_CLIP; i++) {
        if (f->clip_mask >> i & 1) {
            if (p->info[0].clipv) tb_f(&t, "  g2vClip%d = vec4(dot(g2_cl%d, g2o_Clip));\n", i, i);
            else tb_f(&t, "  g2vClip%d = vec4(1.0);\n", i);                  /* §2.12: no gl_ClipVertex, nothing to clip by */
        }
    }
    if (f->points) {
        if (k->vpps && p->info[0].psize) tb_f(&t, "  gl_PointSize = g2o_PS;\n");
        else tb_f(&t, "  gl_PointSize = g2_pt.x;\n");
        tb_f(&t, "  if (abs(gl_Position.x) > gl_Position.w || abs(gl_Position.y) > gl_Position.w) gl_Position.z = 2.0 * abs(gl_Position.w) + 1.0;\n");    /* §2.12: a point is whole or gone by its centre, and board A goes by near and far only */
    }
    if (f->zoff) tb_f(&t, "  gl_Position.z = gl_Position.z + g2_zo.x * gl_Position.w;\n");
    if (f->smooth_ln) {
        tb_f(&t, "  gl_Position.xy = gl_Position.xy + g2aLine.xy * gl_Position.w;\n");
        tb_f(&t, "  g2vLine = vec4(g2aLine.z * gl_Position.w, g2aStip.y * gl_Position.w, g2aStip.z * gl_Position.w, gl_Position.w);\n");
    }
    if (f->line_stip) tb_f(&t, "  g2vStip = vec4(g2aStip.x * gl_Position.w, gl_Position.w, 0.0, 0.0);\n");
    tb_f(&t, "}\n");
    tb_put(out, h.p, h.n);
    tb_f(out, "#line 1\n");
    tb_put(out, x->body.p ? x->body.p : "", x->body.n);
    tb_put(out, t.p, t.n);
    free(h.p); free(t.p);
}

/* ---------------------------------------------------------------- functions that take a sampler
 * What the library does for a sampler (the program's filter, a cube map's bias, an array's element on its own) needs the sampler's name, which a parameter hides. So in a variant whose samplers need that, a
 * function with sampler parameters becomes a copy per sampler it is called with, the parameter replaced by the uniform: f(s, c) -> g2F1_f(c). An element of an array with an index that is not a number keeps the
 * index as a parameter: f(a[i], c) -> g2F2_f(c, i), and the copy reads a[g2x0]. A sampler that needs nothing stays a parameter, which the compiler takes. Every call must resolve, else the text stays as written.
 * #line keeps the shader's lines for the errors. */
#define SP_PARAMS 16
#define SP_FNS 32
#define SP_COPIES 64
typedef struct sp_fn {
    const char *name, *start, *end, *body;                      /* start: the return type; end: past its ';' or '}'; body: its '{', NULL for a prototype */
    size_t nlen;
    int np, pst[SP_PARAMS];                                     /* a parameter's sampler type, -1 for another */
    const char *ps[SP_PARAMS], *pe[SP_PARAMS], *pn[SP_PARAMS];
    size_t pnl[SP_PARAMS];
} sp_fn;
typedef struct sp_copy { int def, id; char form[SP_PARAMS][72]; char *head, *text; } sp_copy;   /* form: "" kept, else "s", "a[3]", or "a[]" with the index a parameter */
typedef struct sp {
    const gs_prog *p;
    const char *src, *end;
    uint32_t special;                                           /* the gs_samp indices to copy for */
    sp_fn fn[SP_FNS];
    sp_copy cp[SP_COPIES];
    const sp_fn *cf;                                            /* the copy being written: its kept parameters are samplers too */
    const sp_copy *cc;
    int nfn, ncp, ids, fail;
} sp;

static int sp_line(const sp *S, const char *at)
{
    const char *p;
    int l = 1;
    for (p = S->src; p < at; p++) if (*p == '\n') l++;
    return l;
}

/* past the bracket that closes the one at p; NULL if it is not closed before e */
static const char *sp_close(const char *p, const char *e)
{
    int d = 0;
    while (p < e) {
        if (p[0] == '/' && p + 1 < e && p[1] == '/') { while (p < e && *p != '\n') p++; continue; }
        if (p[0] == '/' && p + 1 < e && p[1] == '*') { p += 2; while (p + 1 < e && !(p[0] == '*' && p[1] == '/')) p++; p += 2; continue; }
        if (*p == '(' || *p == '[' || *p == '{') d++;
        else if ((*p == ')' || *p == ']' || *p == '}') && --d == 0) return p + 1;
        p++;
    }
    return NULL;
}

static void sp_param(sp *S, sp_fn *f, const char *s, const char *e)
{
    const char *p = s;
    int k = f->np, arr = 0, ty = 0;
    if (k == SP_PARAMS) { S->fail = 1; return; }
    f->ps[k] = skip(s); f->pe[k] = e; f->pst[k] = -1; f->pn[k] = NULL; f->pnl[k] = 0;
    while (f->pe[k] > f->ps[k] && isspace((unsigned char)f->pe[k][-1])) f->pe[k]--;
    while (p < e) {
        p = skip(p);
        if (p >= e) break;
        if (id0(*p)) {
            const char *ie = p;
            int t;
            while (idc(*ie)) ie++;
            t = samp_type(p, (size_t)(ie - p));
            if (t >= 0) { f->pst[k] = t; ty = 1; }
            else if (ty && !f->pn[k]) { f->pn[k] = p; f->pnl[k] = (size_t)(ie - p); }
            p = ie;
            continue;
        }
        if (*p == '[') arr = 1;
        p++;
    }
    if (f->pst[k] >= 0 && arr) S->fail = 1;                     /* an array of samplers as a parameter */
    if (f->ps[k] == f->pe[k] || (f->pe[k] - f->ps[k] == 4 && !memcmp(f->ps[k], "void", 4))) return;
    f->np++;
}

/* the declarations of functions that take a sampler, in the text's order */
static void sp_scan(sp *S)
{
    const char *p = S->src, *stmt = p;
    int depth = 0, after_id = 0;
    while (*p && !S->fail) {
        const char *q = skip(p);
        if (q != p) { p = q; continue; }
        if (*p == '#') { while (*p && *p != '\n') { if (*p == '\\' && p[1] == '\n') p++; p++; } stmt = p; after_id = 0; continue; }
        if (isdigit((unsigned char)*p) || (*p == '.' && isdigit((unsigned char)p[1]))) { while (idc(*p) || *p == '.') p++; after_id = 0; continue; }
        if (id0(*p)) {
            const char *e = p, *a;
            while (idc(*e)) e++;
            a = skip(e);
            if (!depth && after_id && *a == '(') {
                sp_fn f;
                const char *close = sp_close(a, S->end), *t, *u, *b;
                int dep = 0, j, samp = 0;
                if (!close) { S->fail = 1; return; }
                memset(&f, 0, sizeof f);
                f.name = p; f.nlen = (size_t)(e - p); f.start = skip(stmt);
                for (t = u = a + 1; u < close - 1; u++) {
                    if (*u == '(' || *u == '[') dep++;
                    else if (*u == ')' || *u == ']') dep--;
                    else if (*u == ',' && !dep) { sp_param(S, &f, t, u); t = u + 1; }
                }
                sp_param(S, &f, t, close - 1);
                b = skip(close);
                if (*b == '{') { f.body = b; f.end = sp_close(b, S->end); if (!f.end) { S->fail = 1; return; } }
                else if (*b == ';') f.end = b + 1;
                else { p = e; after_id = 0; continue; }
                for (j = 0; j < f.np; j++) if (f.pst[j] >= 0) samp = 1;
                if (samp) { if (S->nfn == SP_FNS) { S->fail = 1; return; } S->fn[S->nfn++] = f; }
                p = stmt = f.end; after_id = 0;
                continue;
            }
            p = e; after_id = !depth;
            continue;
        }
        if (*p == '{') depth++;
        else if (*p == '}') { if (depth) depth--; if (!depth) stmt = p + 1; }
        else if (*p == ';' && !depth) stmt = p + 1;
        after_id = 0;
        p++;
    }
}

static int sp_is(const sp *S, const char *q, size_t n)
{
    int i;
    for (i = 0; i < S->nfn; i++) if (S->fn[i].nlen == n && !memcmp(S->fn[i].name, q, n)) return 1;
    return 0;
}

/* an argument that is a sampler: 1 with its form (empty when it stays a sampler: need 0) and type; 0 for something else; -1 for a sampler used in a way not handled */
static int sp_arg(sp *S, const char *s, const char *e, char *form, size_t cap, const char **ix, const char **ixe, int *type, int *need)
{
    const char *ne, *q;
    const gs_samp *m;
    int lit;
    for (;;) {
        s = skip(s);
        while (e > s && isspace((unsigned char)e[-1])) e--;
        if (s < e && *s == '(' && sp_close(s, e) == e) { s++; e--; continue; }
        break;
    }
    *ix = *ixe = NULL; form[0] = 0; *need = 0;
    if (s >= e || !id0(*s)) return 0;
    for (ne = s; ne < e && idc(*ne); ne++) {}
    q = ne < e ? skip(ne) : ne;
    if (S->cf) {                                                /* a parameter the copy keeps */
        int j;
        for (j = 0; j < S->cf->np; j++)
            if (S->cf->pst[j] >= 0 && !S->cc->form[j][0] && S->cf->pn[j] && S->cf->pnl[j] == (size_t)(ne - s) && !memcmp(S->cf->pn[j], s, S->cf->pnl[j])) {
                if (q < e) return -1;
                *type = S->cf->pst[j];
                return 1;
            }
    }
    m = samp_of(S->p, s, (size_t)(ne - s));
    if (!m) return 0;
    *type = m->type;
    *need = (int)(S->special >> (m - S->p->samp) & 1);
    if (q >= e) { if (m->count) return -1; snprintf(form, cap, "%.*s", (int)(ne - s), s); return 1; }
    if (*q != '[' || !m->count || sp_close(q, e) != e) return -1;
    lit = literal_index(ne);
    if (lit >= 0) snprintf(form, cap, "%.*s[%d]", (int)(ne - s), s, lit);
    else { snprintf(form, cap, "%.*s[]", (int)(ne - s), s); *ix = q + 1; *ixe = e - 1; }
    return 1;
}

/* the copy of declaration d for these forms, made if new; a copy shares its name with the copies of the other overloads for the same forms */
static int sp_copy_of(sp *S, int d, char (*form)[72], int id)
{
    const sp_fn *f = &S->fn[d];
    int c, j;
    for (c = 0; c < S->ncp; c++) {
        const sp_copy *k = &S->cp[c];
        const sp_fn *g = &S->fn[k->def];
        int same = g->nlen == f->nlen && !memcmp(g->name, f->name, f->nlen) && g->np == f->np;
        for (j = 0; same && j < f->np; j++) same = !strcmp(k->form[j], form[j]);
        if (!same) continue;
        if (k->def == d) return k->id;
        if (id < 0) id = k->id;
    }
    if (S->ncp == SP_COPIES) { S->fail = 1; return 0; }
    {
        sp_copy *k = &S->cp[S->ncp++];
        memset(k, 0, sizeof *k);
        k->def = d; k->id = id >= 0 ? id : S->ids++;
        for (j = 0; j < f->np; j++) snprintf(k->form[j], sizeof k->form[j], "%s", form[j]);
        return k->id;
    }
}

static void sp_rw(sp *S, const char *s, const char *e, tb *o);

/* a call of a function that takes a sampler: q its name, a at its parenthesis. Returns past the call */
static const char *sp_call(sp *S, const char *q, size_t n, const char *a, const char *e, tb *o)
{
    const char *close = sp_close(a, e), *as[SP_PARAMS], *ae[SP_PARAMS], *ix[SP_PARAMS], *ixe[SP_PARAMS], *t, *u;
    char form[SP_PARAMS][72];
    int na = 0, typ[SP_PARAMS], need[SP_PARAMS], is[SP_PARAMS], d, j, id = -1, any = 0, dep = 0, nl = 0;
    if (!close) { S->fail = 1; return e; }
    for (t = u = a + 1; u < close - 1; u++) {
        if (*u == '(' || *u == '[') dep++;
        else if (*u == ')' || *u == ']') dep--;
        else if (*u == ',' && !dep) { if (na == SP_PARAMS) { S->fail = 1; return close; } as[na] = t; ae[na++] = u; t = u + 1; }
    }
    if (na || skip(t) < close - 1) { if (na == SP_PARAMS) { S->fail = 1; return close; } as[na] = t; ae[na++] = close - 1; }
    for (j = 0; j < na; j++) {
        typ[j] = -1;
        is[j] = sp_arg(S, as[j], ae[j], form[j], sizeof form[j], &ix[j], &ixe[j], &typ[j], &need[j]);
        if (is[j] < 0) { S->fail = 1; return close; }
        if (!need[j]) { form[j][0] = 0; ix[j] = NULL; }
        any |= is[j];
    }
    if (!any) {                                                 /* an overload without samplers */
        tb_put(o, q, n); tb_f(o, "(");
        for (j = 0; j < na; j++) { if (j) tb_f(o, ","); sp_rw(S, as[j], ae[j], o); }
        tb_f(o, ")");
        return close;
    }
    any = 0;
    for (d = 0; d < S->nfn; d++) {
        const sp_fn *f = &S->fn[d];
        int ok = f->body && f->nlen == n && !memcmp(f->name, q, n) && f->np == na;
        for (j = 0; ok && j < na; j++) ok = is[j] ? f->pst[j] == typ[j] : f->pst[j] < 0;
        if (!ok) continue;
        id = sp_copy_of(S, d, form, id);
        any = 1;
    }
    if (!any || S->fail) { S->fail = 1; return close; }
    tb_f(o, "g2F%d_%.*s(", id, (int)n, q);
    for (j = 0, d = 0; j < na; j++) {
        if (form[j][0]) { for (t = as[j]; t < ae[j]; t++) nl += *t == '\n'; continue; }
        if (d++) tb_f(o, ",");
        sp_rw(S, as[j], ae[j], o);
    }
    for (j = 0; j < na; j++) if (ix[j]) { if (d++) tb_f(o, ", "); sp_rw(S, ix[j], ixe[j], o); }
    tb_f(o, ")");
    while (nl-- > 0) tb_f(o, "\n");
    return close;
}

/* text with every call of a function that takes a sampler made a call of its copy */
static void sp_rw(sp *S, const char *s, const char *e, tb *o)
{
    const char *p = s;
    while (p < e && !S->fail) {
        if (p[0] == '/' && p + 1 < e && p[1] == '/') { const char *q = p; while (q < e && *q != '\n') q++; tb_put(o, p, (size_t)(q - p)); p = q; continue; }
        if (p[0] == '/' && p + 1 < e && p[1] == '*') { const char *q = p + 2; while (q + 1 < e && !(q[0] == '*' && q[1] == '/')) q++; q = q + 2 <= e ? q + 2 : e; tb_put(o, p, (size_t)(q - p)); p = q; continue; }
        if (isdigit((unsigned char)*p)) { const char *q = p; while (q < e && (idc(*q) || *q == '.')) q++; tb_put(o, p, (size_t)(q - p)); p = q; continue; }
        if (id0(*p)) {
            const char *ie = p, *b = p, *a;
            while (ie < e && idc(*ie)) ie++;
            while (b > s && isspace((unsigned char)b[-1])) b--;
            a = ie < e ? skip(ie) : ie;
            if (a < e && *a == '(' && (b == s || b[-1] != '.') && sp_is(S, p, (size_t)(ie - p))) { p = sp_call(S, p, (size_t)(ie - p), a, e, o); continue; }
            tb_put(o, p, (size_t)(ie - p));
            p = ie;
            continue;
        }
        tb_put(o, p, 1);
        p++;
    }
}

/* copy c: its header, and its body with the parameters it does not keep replaced, its own calls rewritten (which may make more copies) */
static void sp_body(sp *S, int c)
{
    sp_copy *k = &S->cp[c];
    const sp_fn *f = &S->fn[k->def], *cf = S->cf;
    const sp_copy *cc = S->cc;
    tb sub = { 0 }, h = { 0 }, o = { 0 };
    const char *p = f->body, *e = f->end;
    int dyn[SP_PARAMS], nd = 0, j, first = 1;
    for (j = 0; j < f->np; j++) { size_t l = strlen(k->form[j]); dyn[j] = l > 2 && !strcmp(k->form[j] + l - 2, "[]") ? nd++ : -1; }
    while (p < e) {
        if (p[0] == '/' && p[1] == '/') { const char *q = p; while (q < e && *q != '\n') q++; tb_put(&sub, p, (size_t)(q - p)); p = q; continue; }
        if (p[0] == '/' && p[1] == '*') { const char *q = p + 2; while (q + 1 < e && !(q[0] == '*' && q[1] == '/')) q++; q = q + 2 <= e ? q + 2 : e; tb_put(&sub, p, (size_t)(q - p)); p = q; continue; }
        if (isdigit((unsigned char)*p)) { const char *q = p; while (q < e && (idc(*q) || *q == '.')) q++; tb_put(&sub, p, (size_t)(q - p)); p = q; continue; }
        if (id0(*p)) {
            const char *ie = p, *b = p;
            while (ie < e && idc(*ie)) ie++;
            while (b > f->body && isspace((unsigned char)b[-1])) b--;
            for (j = 0; j < f->np; j++) if (k->form[j][0] && f->pn[j] && f->pnl[j] == (size_t)(ie - p) && !memcmp(f->pn[j], p, f->pnl[j])) break;
            if (j < f->np && b[-1] != '.') {
                if (dyn[j] >= 0) tb_f(&sub, "%.*sg2x%d]", (int)strlen(k->form[j]) - 1, k->form[j], dyn[j]);
                else tb_f(&sub, "%s", k->form[j]);
            } else tb_put(&sub, p, (size_t)(ie - p));
            p = ie;
            continue;
        }
        tb_put(&sub, p, 1);
        p++;
    }
    tb_put(&h, f->start, (size_t)(f->name - f->start));
    tb_f(&h, "g2F%d_%.*s(", k->id, (int)f->nlen, f->name);
    for (j = 0; j < f->np; j++) if (!k->form[j][0]) { if (!first) tb_f(&h, ", "); tb_put(&h, f->ps[j], (size_t)(f->pe[j] - f->ps[j])); first = 0; }
    for (j = 0; j < f->np; j++) if (dyn[j] >= 0) { if (!first) tb_f(&h, ", "); tb_f(&h, "int g2x%d", dyn[j]); first = 0; }
    tb_f(&h, ")");
    k->head = h.p;
    tb_f(&o, "\n#line %d\n", sp_line(S, f->body));
    S->cf = f; S->cc = k;
    sp_rw(S, sub.p ? sub.p : "", sub.p ? sub.p + sub.n : NULL, &o);
    S->cf = cf; S->cc = cc;
    k->text = o.p;
    free(sub.p);
}

/* the stage's text with a copy of every function that takes a sampler per call; NULL when it has none or a call does not resolve */
static char *sp_run(const gs_prog *p, const char *src, uint32_t special)
{
    sp *S = (sp *)calloc(1, sizeof *S);
    tb seg[SP_FNS + 1], o = { 0 };
    const char *at = src;
    char *out = NULL;
    int i, c, j;
    if (!S) return NULL;
    memset(seg, 0, sizeof seg);
    S->p = p; S->src = src; S->end = src + strlen(src); S->special = special;
    sp_scan(S);
    if (S->fail || !S->nfn) goto done;
    for (i = 0; i <= S->nfn && !S->fail; i++) {                 /* the text between the declarations */
        sp_rw(S, at, i < S->nfn ? S->fn[i].start : S->end, &seg[i]);
        if (i < S->nfn) at = S->fn[i].end;
    }
    for (c = 0; c < S->ncp && !S->fail; c++) sp_body(S, c);
    if (S->fail) goto done;
    for (i = 0; i <= S->nfn; i++) {
        const sp_fn *f = &S->fn[i];
        int first = 1;
        if (seg[i].p) tb_put(&o, seg[i].p, seg[i].n);
        if (i == S->nfn) break;
        for (j = 0; j < i; j++) if (S->fn[j].nlen == f->nlen && !memcmp(S->fn[j].name, f->name, f->nlen)) first = 0;
        if (first && !f->body)                                  /* a prototype: one for every copy of the name */
            for (c = 0; c < S->ncp; c++) {
                const sp_fn *g = &S->fn[S->cp[c].def];
                if (g->nlen == f->nlen && !memcmp(g->name, f->name, f->nlen)) { tb_f(&o, "\n"); tb_put(&o, S->cp[c].head, strlen(S->cp[c].head)); tb_f(&o, ";"); }
            }
        if (f->body)
            for (c = 0; c < S->ncp; c++) if (S->cp[c].def == i) { tb_f(&o, "\n"); tb_put(&o, S->cp[c].head, strlen(S->cp[c].head)); tb_put(&o, S->cp[c].text, strlen(S->cp[c].text)); }
        tb_f(&o, "\n#line %d\n", sp_line(S, f->end));
    }
    out = o.p; o.p = NULL;
done:
    for (i = 0; i <= SP_FNS; i++) free(seg[i].p);
    for (c = 0; c < S->ncp; c++) { free(S->cp[c].head); free(S->cp[c].text); }
    free(o.p);
    free(S);
    return out;
}

/* g2a<m>_<function>(int g2i, ...): an array's element chosen at run time, each read as elem_head reads it. Both sides of an IF run on the SIMT core, so this costs a read per element */
static void xl_dispatch(xl *x, tb *h)
{
    const gs_prog *p = x->p;
    int mi, i, f, o, e;
    for (mi = 0; mi < p->nsamp; mi++) {
        const gs_samp *m = &p->samp[mi];
        for (i = 0; i <= N_BI_TEX; i++) {
            const char *nm = i < N_BI_TEX ? BI_TEX[i].name : "textureCube", *ty = i < N_BI_TEX ? BI_TEX[i].ty : "vec3";
            int d = i < N_BI_TEX ? bi_tex_dim(i) : 3;
            const char *dt = d == 1 ? "float" : d == 2 ? "vec2" : "vec3";
            for (f = 0; f < 3; f++) {
                uint32_t w = i < N_BI_TEX ? (f == 0 ? x->arr_fn[mi] : f == 1 ? x->arr_lod[mi] : x->arr_grad[mi]) >> i & 1 : (uint32_t)(x->arr_cube[mi] >> f & 1);
                char fn[40];
                if (!w) continue;
                snprintf(fn, sizeof fn, "%s%s", nm, f == 1 ? "Lod" : f == 2 ? "GradARB" : "");
                for (o = 0; o < (f == 0 ? 2 : 1); o++) {
                    tb_f(h, "vec4 g2a%d_%s(int g2i, %s c%s", mi, fn, ty, f == 0 && o ? ", float b" : f == 1 ? ", float l" : "");
                    if (f == 2) tb_f(h, ", %s dx, %s dy", dt, dt);
                    tb_f(h, ") {\n  vec4 r;\n");
                    for (e = 0; e < m->count; e++) {
                        tb_f(h, e ? "  else " : "  ");
                        if (e + 1 < m->count) tb_f(h, "if (g2i == %d) ", e);
                        tb_f(h, "r = ");
                        elem_head(x, h, m->first + e, fn, strlen(fn), f, nm);
                        tb_f(h, "c%s);\n", f == 0 && o ? ", b" : f == 1 ? ", l" : f == 2 ? ", dx, dy" : "");
                    }
                    tb_f(h, "  return r;\n}\n");
                }
            }
        }
    }
}

static void xl_fs(xl *x, tb *out)
{
    const gs_prog *p = x->p;
    const gs_key *k = x->k;
    const ff_key *f = &k->ff;
    const gs_info *in = &p->info[1];
    tb h = { 0 }, t = { 0 }, dsp = { 0 };
    char *spec = NULL;
    uint32_t special = 0;
    int i, s, all = strstr(p->src[1], "GradARB") != NULL;      /* gradients need a sampler's size, so its name */
    tb_f(&h, "#extension GL_OES_texture_3D : enable\n#define GL_ARB_shader_texture_lod 1\nvec4 g2i_Color, g2i_Sec;\nvec4 g2i_Tc[%d];\nfloat g2i_Fog;\n", MAX_UNITS);
    if (in->color || k->pix) tb_f(&h, "varying vec4 g2vColor;\n");
    if (in->color && k->two) tb_f(&h, "varying vec4 g2vColorB;\n");
    if (k->pix) { tb_f(&h, "uniform sampler2D g2Pix;\n"); need_uni(x, &h, "vec4", "g2_rtc", U_F_RTC, 0); if (!(in->tc & 1)) tb_f(&h, "varying vec4 g2vTc0;\n"); }
    if (in->sec) tb_f(&h, "varying vec4 g2vSpec;\n");
    if (in->sec && k->two) tb_f(&h, "varying vec4 g2vSpecB;\n");
    if (in->fogf) tb_f(&h, "varying vec4 g2vFogF;\n");
    for (i = 0; i < MAX_UNITS; i++) if (in->tc >> i & 1) tb_f(&h, "varying vec4 g2vTc%d;\n", i);
    for (i = 0; i < MAX_CLIP; i++) if (f->clip_mask >> i & 1) tb_f(&h, "varying vec4 g2vClip%d;\n", i);
    if (f->line_stip) tb_f(&h, "varying vec4 g2vStip;\n");
    if (f->smooth_ln) tb_f(&h, "varying vec4 g2vLine;\n");
    if (f->line_stip || f->poly_stip) tb_f(&h, "uniform sampler2D g2Stip;\n");
    if (f->alpha_fn != 7) need_uni(x, &h, "vec4", "g2_alpha", U_F_ALPHA, 0);
    if (f->line_stip) need_uni(x, &h, "vec4", "g2_stipF", U_F_STIP, 0);
    if (f->smooth_ln) need_uni(x, &h, "vec4", "g2_lineF", U_F_LINE, 0);
    if (f->smooth_pt) need_uni(x, &h, "vec4", "g2_ptF", U_F_POINT, 0);
    for (s = 0; s < p->nel; s++) {
        if (k->sk.u[s].sw && k->sk.u[s].target) {
            char *txt = NULL;
            gl_ureq fu[16];
            int nf = 0, q;
            gl_ff_sw_text(&k->sk, s, &txt, fu, &nf);
            if (txt) { tb_put(&h, txt, strlen(txt)); free(txt); }
            for (q = 0; q < nf; q++) x->ur[x->nur++] = fu[q];
        }
        if (k->lbias[s]) { char nm[16]; snprintf(nm, sizeof nm, "g2Lod%d", s); need_uni(x, &h, "vec4", nm, U_F_LOD, s); }
    }
    for (i = 0; i < p->nsamp; i++) {                            /* what only the library does for a sampler: the program's filter, a cube map's bias, (0, 0, 0, 1) of an incomplete one */
        const gs_samp *m = &p->samp[i];
        int e;
        for (e = 0; e < (m->count ? m->count : 1); e++) {
            int q = m->first + e;
            if (all || (k->sk.u[q].target ? k->sk.u[q].sw || k->lbias[q] : !m->count)) special |= 1u << i;
        }
        if (m->count && (special >> i & 1)) x->el_mode |= 1u << i;
    }
    if (x->spec && special) spec = sp_run(p, p->src[1], special);
    x->src = spec ? spec : p->src[1];
    xl_run(x, x->src);
    xl_dispatch(x, &dsp);
    for (s = 0; s < p->nel; s++) if (x->tsz >> s & 1) { char nm[16]; snprintf(nm, sizeof nm, "g2Tsz%d", s); need_uni(x, &h, "vec4", nm, U_F_TSZ, s); }
    for (s = 0; s < p->nel; s++) {                              /* an array's element the TMU reads on its own */
        const gs_samp *m = NULL;
        if (!(x->el_decl >> s & 1)) continue;
        for (i = 0; i < p->nsamp; i++) if (s >= p->samp[i].first && s < p->samp[i].first + (p->samp[i].count ? p->samp[i].count : 1)) m = &p->samp[i];
        tb_f(&h, "uniform %s g2Tex%d;\n", m && m->type == ST_CUBE ? "samplerCube" : m && m->type == ST_3D ? "sampler3D" : "sampler2D", s);
    }
    for (i = 0; i < N_BI_UNI; i++) {
        if (!x->uni[i]) continue;
        if (BI_UNI[i].count) tb_f(&h, "uniform %s g2u_%s[%d];\n", BI_UNI[i].ty, BI_UNI[i].name + 3, BI_UNI[i].count);
        else tb_f(&h, "uniform %s g2u_%s;\n", BI_UNI[i].ty, BI_UNI[i].name + 3);
    }
    for (i = 0; i < N_BI_MEM; i++) {
        if (!x->mem_used[i]) continue;
        if (BI_MEM[i].count) tb_f(&h, "uniform %s g2u_%s_%s[%d];\n", BI_MEM[i].ty, BI_MEM[i].var + 3, BI_MEM[i].mem, x->mem_max[i] ? x->mem_max[i] : BI_MEM[i].count);
        else tb_f(&h, "uniform %s g2u_%s_%s;\n", BI_MEM[i].ty, BI_MEM[i].var + 3, BI_MEM[i].mem);
    }
    xl_structs(x, &h);
    if (x->noise) tb_f(&h, "float g2_noise1(float x) { return 0.0; }\nfloat g2_noise1(vec2 x) { return 0.0; }\nfloat g2_noise1(vec3 x) { return 0.0; }\nfloat g2_noise1(vec4 x) { return 0.0; }\n"
                           "vec2 g2_noise2(float x) { return vec2(0.0); }\nvec2 g2_noise2(vec2 x) { return vec2(0.0); }\nvec2 g2_noise2(vec3 x) { return vec2(0.0); }\nvec2 g2_noise2(vec4 x) { return vec2(0.0); }\n"
                           "vec3 g2_noise3(float x) { return vec3(0.0); }\nvec3 g2_noise3(vec2 x) { return vec3(0.0); }\nvec3 g2_noise3(vec3 x) { return vec3(0.0); }\nvec3 g2_noise3(vec4 x) { return vec3(0.0); }\n"
                           "vec4 g2_noise4(float x) { return vec4(0.0); }\nvec4 g2_noise4(vec2 x) { return vec4(0.0); }\nvec4 g2_noise4(vec3 x) { return vec4(0.0); }\nvec4 g2_noise4(vec4 x) { return vec4(0.0); }\n");
    if (x->t1) tb_f(&h, "vec4 g2_texture1D(sampler2D s, float c) { return texture2D(s, vec2(c, 0.5)); }\nvec4 g2_texture1D(sampler2D s, float c, float b) { return texture2D(s, vec2(c, 0.5), b); }\n");
    if (x->t1p) tb_f(&h, "vec4 g2_texture1DProj(sampler2D s, vec2 c) { return texture2D(s, vec2(c.x / c.y, 0.5)); }\nvec4 g2_texture1DProj(sampler2D s, vec2 c, float b) { return texture2D(s, vec2(c.x / c.y, 0.5), b); }\n"
                        "vec4 g2_texture1DProj(sampler2D s, vec4 c) { return texture2D(s, vec2(c.x / c.w, 0.5)); }\nvec4 g2_texture1DProj(sampler2D s, vec4 c, float b) { return texture2D(s, vec2(c.x / c.w, 0.5), b); }\n");
    if (x->t1l) tb_f(&h, "vec4 g2_texture1DLod(sampler2D s, float c, float l) { return texture2DLod(s, vec2(c, 0.5), l); }\nvec4 g2_texture1DProjLod(sampler2D s, vec2 c, float l) { return texture2DLod(s, vec2(c.x / c.y, 0.5), l); }\n"
                         "vec4 g2_texture1DProjLod(sampler2D s, vec4 c, float l) { return texture2DLod(s, vec2(c.x / c.w, 0.5), l); }\n");
    for (i = 0; i < N_BI_TEX; i++) {                            /* a shadow function on what the TMU reads as colour */
        char co[96];
        if (!(x->shadow_tmu >> i & 1) && !(x->shadow_tmu >> (i + 16) & 1)) continue;
        if (bi_tex_dim(i) == 1) snprintf(co, sizeof co, "vec2((%s).x, 0.5)", BI_TEX[i].st);
        else snprintf(co, sizeof co, "(%s).xy", BI_TEX[i].st);
        if (x->shadow_tmu >> i & 1) tb_f(&h, "vec4 g2_%s(sampler2D s, %s c) { return texture2D(s, %s); }\nvec4 g2_%s(sampler2D s, %s c, float b) { return texture2D(s, %s, b); }\n", BI_TEX[i].name, BI_TEX[i].ty, co, BI_TEX[i].name, BI_TEX[i].ty, co);
        if (x->shadow_tmu >> (i + 16) & 1) tb_f(&h, "vec4 g2_%sLod(sampler2D s, %s c, float l) { return texture2DLod(s, %s, l); }\n", BI_TEX[i].name, BI_TEX[i].ty, co);
    }
    if (x->grad) {                                              /* λ from the gradients in texels, as §3.8.8 has it from the derivatives */
        tb_f(&h, "float g2_gl(vec3 a, vec3 b) { return 0.5 * log2(max(max(dot(a, a), dot(b, b)), 1.0e-12)); }\n");
        tb_f(&h, "vec4 g2_g1(sampler2D s, vec4 n, float c, float dx, float dy) { return texture2DLod(s, vec2(c, 0.5), g2_gl(vec3(dx * n.x, 0.0, 0.0), vec3(dy * n.x, 0.0, 0.0))); }\n");
        tb_f(&h, "vec4 g2_g1p(sampler2D s, vec4 n, vec2 c, float dx, float dy) { return g2_g1(s, n, c.x / c.y, dx, dy); }\nvec4 g2_g1p(sampler2D s, vec4 n, vec4 c, float dx, float dy) { return g2_g1(s, n, c.x / c.w, dx, dy); }\n");
        tb_f(&h, "vec4 g2_g2(sampler2D s, vec4 n, vec2 c, vec2 dx, vec2 dy) { return texture2DLod(s, c, g2_gl(vec3(dx * n.xy, 0.0), vec3(dy * n.xy, 0.0))); }\n");
        tb_f(&h, "vec4 g2_g2p(sampler2D s, vec4 n, vec3 c, vec2 dx, vec2 dy) { return g2_g2(s, n, c.xy / c.z, dx, dy); }\nvec4 g2_g2p(sampler2D s, vec4 n, vec4 c, vec2 dx, vec2 dy) { return g2_g2(s, n, c.xy / c.w, dx, dy); }\n");
        tb_f(&h, "vec4 g2_g3(sampler3D s, vec4 n, vec3 c, vec3 dx, vec3 dy) { return texture3DLod(s, c, g2_gl(dx * n.xyz, dy * n.xyz)); }\nvec4 g2_g3p(sampler3D s, vec4 n, vec4 c, vec3 dx, vec3 dy) { return g2_g3(s, n, c.xyz / c.w, dx, dy); }\n");
        tb_f(&h, "vec4 g2_gc(samplerCube s, vec4 n, float b, vec3 c, vec3 dx, vec3 dy) { vec3 m = abs(c); float k = n.x * 0.5 / max(m.x, max(m.y, m.z)); return textureCubeLod(s, c, g2_gl(dx * k, dy * k) + b); }\n");
    }
    for (s = 0; s < p->nel; s++) {                              /* the calls of a sampler the program filters: every overload of the names it was called by */
        for (i = 0; i < N_BI_TEX; i++) {
            if (x->tex_fn[s] >> i & 1) {
                tb_f(&h, "vec4 g2t%d_%s(%s c) { return g2s%d(%s, 0.0); }\n", s, BI_TEX[i].name, BI_TEX[i].ty, s, BI_TEX[i].st);
                tb_f(&h, "vec4 g2t%d_%s(%s c, float b) { return g2s%d(%s, b); }\n", s, BI_TEX[i].name, BI_TEX[i].ty, s, BI_TEX[i].st);
                if (i + 1 < N_BI_TEX && !strcmp(BI_TEX[i].name, BI_TEX[i + 1].name)) x->tex_fn[s] |= 1u << (i + 1);
            }
            if (x->tex_lod[s] >> i & 1) tb_f(&h, "vec4 g2t%d_%sLod(%s c, float l) { return g2sl%d(%s, l); }\n", s, BI_TEX[i].name, BI_TEX[i].ty, s, BI_TEX[i].st);
            if (x->tex_grad[s] >> i & 1) {
                int d = bi_tex_dim(i);
                const char *ty = d == 1 ? "float" : d == 2 ? "vec2" : "vec3";
                if (d == 1) tb_f(&h, "vec4 g2t%d_%sGradARB(%s c, float dx, float dy) { return g2sl%d(%s, g2_gl(vec3(dx * g2Sz%d.x, 0.0, 0.0), vec3(dy * g2Sz%d.x, 0.0, 0.0))); }\n", s, BI_TEX[i].name, BI_TEX[i].ty, s, BI_TEX[i].st, s, s);
                else if (d == 2) tb_f(&h, "vec4 g2t%d_%sGradARB(%s c, %s dx, %s dy) { return g2sl%d(%s, g2_gl(vec3(dx * g2Sz%d.xy, 0.0), vec3(dy * g2Sz%d.xy, 0.0))); }\n", s, BI_TEX[i].name, BI_TEX[i].ty, ty, ty, s, BI_TEX[i].st, s, s);
                else tb_f(&h, "vec4 g2t%d_%sGradARB(%s c, %s dx, %s dy) { return g2sl%d(%s, g2_gl(dx * g2Sz%d.xyz, dy * g2Sz%d.xyz)); }\n", s, BI_TEX[i].name, BI_TEX[i].ty, ty, ty, s, BI_TEX[i].st, s, s);
            }
        }
    }
    for (s = 0; s < p->nel; s++) {                              /* an incomplete texture reads (0, 0, 0, 1); a cube map's bias */
        if (x->cube_fn >> s & 1) {
            const gs_samp *m = NULL;
            int q;
            for (q = 0; q < p->nsamp; q++) if (p->samp[q].first == s) m = &p->samp[q];
            if (!k->sk.u[s].target) {
                for (i = 0; i < N_BI_TEX; i++) {
                    tb_f(&h, "vec4 g2c%d_%s(%s c) { return vec4(0.0, 0.0, 0.0, 1.0); }\n", s, BI_TEX[i].name, BI_TEX[i].ty);
                    tb_f(&h, "vec4 g2c%d_%s(%s c, float b) { return vec4(0.0, 0.0, 0.0, 1.0); }\n", s, BI_TEX[i].name, BI_TEX[i].ty);
                    tb_f(&h, "vec4 g2c%d_%sLod(%s c, float l) { return vec4(0.0, 0.0, 0.0, 1.0); }\n", s, BI_TEX[i].name, BI_TEX[i].ty);
                    {
                        const char *ty = bi_tex_dim(i) == 1 ? "float" : bi_tex_dim(i) == 2 ? "vec2" : "vec3";
                        tb_f(&h, "vec4 g2c%d_%sGradARB(%s c, %s dx, %s dy) { return vec4(0.0, 0.0, 0.0, 1.0); }\n", s, BI_TEX[i].name, BI_TEX[i].ty, ty, ty);
                    }
                }
                tb_f(&h, "vec4 g2c%d_textureCube(vec3 c) { return vec4(0.0, 0.0, 0.0, 1.0); }\nvec4 g2c%d_textureCube(vec3 c, float b) { return vec4(0.0, 0.0, 0.0, 1.0); }\n", s, s);
                tb_f(&h, "vec4 g2c%d_textureCubeLod(vec3 c, float l) { return vec4(0.0, 0.0, 0.0, 1.0); }\nvec4 g2c%d_textureCubeGradARB(vec3 c, vec3 dx, vec3 dy) { return vec4(0.0, 0.0, 0.0, 1.0); }\n", s, s);
            } else {                                            /* a sampler by its name, an array's element as g2Tex<s> */
                char nm[72];
                if (m && !m->count) { snprintf(nm, sizeof nm, "%s", m->name); tb_f(&h, "uniform samplerCube %s;\n", nm); }
                else snprintf(nm, sizeof nm, "g2Tex%d", s);
                tb_f(&h, "vec4 g2c%d_textureCube(vec3 c) { return textureCube(%s, c, g2Lod%d.x); }\nvec4 g2c%d_textureCube(vec3 c, float b) { return textureCube(%s, c, g2Lod%d.x + b); }\n", s, nm, s, s, nm, s);
                tb_f(&h, "vec4 g2c%d_textureCubeLod(vec3 c, float l) { return textureCubeLod(%s, c, l + g2Lod%d.x); }\n", s, nm, s);
            }
        }
    }
    if (dsp.p) tb_put(&h, dsp.p, dsp.n);
    free(dsp.p);
    tb_f(&t, "\nvoid main() {\n");
    for (i = 0; i < MAX_CLIP; i++) if (f->clip_mask >> i & 1) tb_f(&t, "  if (g2vClip%d.x < 0.0) discard;\n", i);
    if (f->poly_stip) tb_f(&t, "  if (texture2D(g2Stip, gl_FragCoord.xy / 32.0).a < 0.5) discard;\n");
    if (f->line_stip) tb_f(&t, "  if (texture2D(g2Stip, vec2((mod(floor(g2vStip.x / g2vStip.y / g2_stipF.x), 16.0) + 0.5) / 16.0, 0.5)).a < 0.5) discard;\n");
    if (k->pix == 1) tb_f(&t, "  g2i_Color = texture2D(g2Pix, g2vTc0.st);\n");
    else if (k->pix == 2) tb_f(&t, "  if (texture2D(g2Pix, g2vTc0.st).a < 0.5) discard;\n  g2i_Color = g2vColor;\n");
    else if (in->color) {
        if (k->two) tb_f(&t, "  g2i_Color = gl_FrontFacing ? g2vColor : g2vColorB;\n");
        else tb_f(&t, "  g2i_Color = g2vColor;\n");
    }
    if (in->sec) {
        if (k->two) tb_f(&t, "  g2i_Sec = gl_FrontFacing ? g2vSpec : g2vSpecB;\n");
        else tb_f(&t, "  g2i_Sec = g2vSpec;\n");
    }
    if (in->fogf) tb_f(&t, "  g2i_Fog = g2vFogF.x;\n");
    for (i = 0; i < MAX_UNITS; i++) {
        if (!(in->tc >> i & 1)) continue;
        if (k->pix && !i) tb_f(&t, "  g2i_Tc[0] = g2_rtc;\n");
        else if (f->crep >> i & 1) tb_f(&t, "  g2i_Tc[%d] = vec4(gl_PointCoord.x, %sgl_PointCoord.y, 0.0, 1.0);\n", i, f->sprite_low ? "1.0 - " : "");
        else tb_f(&t, "  g2i_Tc[%d] = g2vTc%d;\n", i, i);
    }
    tb_f(&t, "  g2_main();\n");
    if (f->smooth_pt) tb_f(&t, "  {\n    vec2 pc = (gl_PointCoord - vec2(0.5)) * g2_ptF.y;\n    float cov = clamp(g2_ptF.x + 0.5 - sqrt(dot(pc, pc)), 0.0, 1.0);\n    if (cov <= 0.0) discard;\n    gl_FragColor.a = gl_FragColor.a * cov;\n  }\n");
    if (f->smooth_ln) {
        tb_f(&t, "  {\n    float ld = g2vLine.x / g2vLine.w;\n    float cov = clamp(min(ld + 0.5, g2_lineF.x) - max(ld - 0.5, -g2_lineF.x), 0.0, 1.0);\n");
        tb_f(&t, "    cov = cov * clamp(min(g2vLine.y / g2vLine.w, 0.5) + min(g2vLine.z / g2vLine.w, 0.5), 0.0, 1.0);\n    if (cov <= 0.0) discard;\n    gl_FragColor.a = gl_FragColor.a * cov;\n  }\n");
    }
    {
        static const char *const OP[7] = { NULL, "<", "==", "<=", ">", "!=", ">=" };
        if (f->alpha_fn == 0) tb_f(&t, "  discard;\n");
        else if (f->alpha_fn < 7) tb_f(&t, "  if (!(gl_FragColor.a %s g2_alpha.x)) discard;\n", OP[f->alpha_fn]);
    }
    if (f->logic == 1) tb_f(&t, "  gl_FragColor = vec4(0.0);\n");
    else if (f->logic == 2) tb_f(&t, "  gl_FragColor = vec4(1.0);\n");
    else if (f->logic == 3) tb_f(&t, "  gl_FragColor = vec4(1.0) - gl_FragColor;\n");
    tb_f(&t, "}\n");
    tb_put(out, h.p, h.n);
    tb_f(out, "#line 1\n");
    tb_put(out, x->body.p ? x->body.p : "", x->body.n);
    tb_put(out, t.p, t.n);
    free(h.p); free(t.p); free(spec);
}

/* ---------------------------------------------------------------- the key of the current state */
/* what the TMU cannot do for a colour texture a shader reads */
static int needs_filter(const gl_tex *x, const gl_unit *u, int linear)
{
    int b1 = x->wrap_s == GL_CLAMP_TO_BORDER || (x->wrap_s == GL_CLAMP && linear), b2 = x->wrap_t == GL_CLAMP_TO_BORDER || (x->wrap_t == GL_CLAMP && linear);
    return b1 || b2 || x->bimg || x->min_lod > -1000.0f || x->max_lod < 1000.0f || x->lod_bias != 0.0f || u->lod_bias != 0.0f;
}

static int samp_kind(int t) { return t == ST_1D || t == ST_1DS ? 2 : t == ST_3D ? 3 : t == ST_CUBE ? 1 : 0; }

static void key_samplers(const gs_prog *p, gs_key *k)
{
    int m, e;
    for (m = 0; m < p->nsamp; m++) {
        const gs_samp *d = &p->samp[m];
        for (e = 0; e < (d->count ? d->count : 1); e++) {
            int s = d->first + e, unit = p->samp_unit[s];
            const gl_unit *u = &C->unit[unit];
            gl_tex *x = gl_unit_tex(unit, samp_kind(d->type));
            int target = d->type == ST_1D || d->type == ST_1DS ? 3 : d->type == ST_3D ? 4 : d->type == ST_CUBE ? 2 : 1;
            if (!x || !gl_tex_complete(x)) continue;
            k->sk.u[s].target = (uint8_t)target;
            if (target == 2) { if (x->lod_bias != 0.0f || u->lod_bias != 0.0f) k->lbias[s] = 1; continue; }
            {
                int linear = x->mag_filter == GL_LINEAR || x->min_filter == GL_LINEAR || x->min_filter == GL_LINEAR_MIPMAP_NEAREST || x->min_filter == GL_LINEAR_MIPMAP_LINEAR;
                if ((target == 4 && !gl_tex3d_tmu(x)) || x->base_class == BF_DEPTH || needs_filter(x, u, linear)) gl_ff_sw_key(&k->sk, s, x);
            }
        }
    }
}

static void key_of(const gs_prog *p, gs_key *k)
{
    ff_key *f = &k->ff;
    int u, hv;
    memset(k, 0, sizeof *k);
    gl_ff_key(f);
    if (C->internal_draw && p->has_fs) { k->pix = C->drawing_mask ? 2 : 1; f->mask0 = 0; }
    hv = p->has_vs && !k->pix;                                  /* a picture has no vertices to shade */
    k->vpps = (uint8_t)C->vp_point_size;
    k->flat = (uint8_t)(hv && C->shade_model == GL_FLAT && !C->drawing_points && !C->no_flat);
    k->two = (uint8_t)(hv ? C->vp_two_side : !k->pix && C->lighting && C->lm_two_side);
    if (hv) {                                                   /* the fragment side stays: what is left of the vertex side is what the generated fragment program reads */
        f->color_mat = f->local_viewer = f->normalize = f->rescale = f->cm_mode = 0;
        f->light_on = f->light_pos = f->light_spot = f->pt_att = f->fogc = 0;
        f->light = k->two; f->two_side = k->two;
        f->sep_spec = 0;
        f->sec = (uint8_t)C->color_sum;
        for (u = 0; u < MAX_UNITS; u++) {
            memset(f->u[u].gen, 0, sizeof f->u[u].gen);
            f->u[u].texmat = 0;
            if (f->u[u].target) f->u[u].proj = 1;
        }
    }
    if (p->has_fs) {
        ff_key keep = *f;
        const gs_info *in = &p->info[1];
        memset(f->u, 0, sizeof f->u);
        f->tex_units = 0; f->mask0 = 0;
        f->fog = 0; f->sec = 0;
        if (!hv) {                                              /* a generated vertex program writes what the shader reads */
            for (u = 0; u < MAX_UNITS; u++) {
                if ((in->tc >> u & 1) || (k->pix && !u)) {
                    int i, id = 1;
                    const gl_unit *t = &C->unit[u];
                    static const float I4[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
                    f->u[u].target = 1;
                    id = !memcmp(t->mat[t->mat_sp], I4, sizeof I4);
                    f->u[u].texmat = !id;
                    for (i = 0; i < 4; i++) {
                        if (!t->gen_en[i]) continue;
                        f->u[u].gen[i] = t->gen_mode[i] == GL_OBJECT_LINEAR ? 1 : t->gen_mode[i] == GL_EYE_LINEAR ? 2 : t->gen_mode[i] == GL_SPHERE_MAP ? 3 : t->gen_mode[i] == GL_NORMAL_MAP ? 4 : 5;
                    }
                }
            }
            f->need_spec = (uint8_t)in->sec;
            f->fogfrag = in->fogf ? (C->fog_src == GL_FOG_COORD ? 2 : 1) : 0;
            f->sec = keep.sec;
        }
    }
    key_samplers(p, k);
    k->sk.smooth_pt = f->smooth_pt; k->sk.smooth_ln = f->smooth_ln; k->sk.line_stip = f->line_stip;
}

/* ---------------------------------------------------------------- a variant */
static uint8_t attr_of_builtin(const char *n)
{
    int i;
    for (i = 0; i < N_BI_ATTR; i++) if (!strcmp(BI_ATTR[i].as, n)) return (uint8_t)BI_ATTR[i].a;
    return 255;
}

static const char *sym_n(const g2sc_sym *y) { return y->lname ? y->lname : y->name; }
static const char *sym_t(const g2sc_sym *y) { return y->ltype ? y->ltype : y->type; }

static GLenum type_enum(const char *t)
{
    static const struct { const char *n; GLenum e; } T[] = { { "float", GL_FLOAT }, { "vec2", GL_FLOAT_VEC2 }, { "vec3", GL_FLOAT_VEC3 }, { "vec4", GL_FLOAT_VEC4 }, { "int", GL_INT }, { "ivec2", GL_INT_VEC2 }, { "ivec3", GL_INT_VEC3 },
        { "ivec4", GL_INT_VEC4 }, { "bool", GL_BOOL }, { "bvec2", GL_BOOL_VEC2 }, { "bvec3", GL_BOOL_VEC3 }, { "bvec4", GL_BOOL_VEC4 }, { "mat2", GL_FLOAT_MAT2 }, { "mat3", GL_FLOAT_MAT3 }, { "mat4", GL_FLOAT_MAT4 } };
    size_t i;
    for (i = 0; i < sizeof T / sizeof T[0]; i++) if (!strcmp(T[i].n, t)) return T[i].e;
    return GL_FLOAT;
}

static void type_shape(GLenum e, int *rows, int *comps, int *base)
{
    *base = 0;
    switch (e) {
    case GL_FLOAT_MAT2: *rows = 2; *comps = 2; return;
    case GL_FLOAT_MAT3: *rows = 3; *comps = 3; return;
    case GL_FLOAT_MAT4: *rows = 4; *comps = 4; return;
    case GL_FLOAT_VEC2: *comps = 2; break;
    case GL_FLOAT_VEC3: *comps = 3; break;
    case GL_FLOAT_VEC4: *comps = 4; break;
    case GL_INT: *comps = 1; *base = 1; break;
    case GL_INT_VEC2: *comps = 2; *base = 1; break;
    case GL_INT_VEC3: *comps = 3; *base = 1; break;
    case GL_INT_VEC4: *comps = 4; *base = 1; break;
    case GL_BOOL: *comps = 1; *base = 2; break;
    case GL_BOOL_VEC2: *comps = 2; *base = 2; break;
    case GL_BOOL_VEC3: *comps = 3; *base = 2; break;
    case GL_BOOL_VEC4: *comps = 4; *base = 2; break;
    default: *comps = 1; break;
    }
    *rows = 1;
}

static int uni_find(const gs_prog *p, const char *name)
{
    int i;
    for (i = 0; i < p->nuni; i++) if (!strcmp(p->uni[i].name, name)) return i;
    return -1;
}

static int uni_add(gs_prog *p, const char *name, const char *ty, int count)
{
    gs_uni *u;
    int i = uni_find(p, name);
    size_t n;
    if (i >= 0) return i;
    p->uni = (gs_uni *)realloc(p->uni, ((size_t)p->nuni + 1) * sizeof *p->uni);
    u = &p->uni[p->nuni];
    memset(u, 0, sizeof *u);
    snprintf(u->name, sizeof u->name, "%s", name);
    u->type = type_enum(ty);
    u->size = count ? count : 1;
    u->sampler = -1;
    type_shape(u->type, &u->stride, &u->comps, &u->base);
    n = (size_t)u->size * u->stride * u->comps;
    u->val = (float *)calloc(n, sizeof(float));
    u->bits = (uint32_t *)calloc(n, sizeof(uint32_t));
    return p->nuni++;
}

static void bind_add(gs_variant *v, int uni, int stage, int slot)
{
    int i;
    for (i = 0; i < v->nbind; i++) if (v->bind[i].uni == uni) break;
    if (i == v->nbind) {
        v->bind = (gs_bind *)realloc(v->bind, ((size_t)v->nbind + 1) * sizeof *v->bind);
        v->bind[i].uni = uni; v->bind[i].vs = v->bind[i].fs = -1; v->bind[i].vc = v->bind[i].fc = 0;
        v->nbind++;
    }
    if (stage) v->bind[i].fs = slot; else v->bind[i].vs = slot;
}

/* a user's uniform, a struct by "s.member", "a[1].member": the names glGetUniformLocation is asked for */
static void member(gs_prog *p, gs_variant *v, const g2sc_result *r, const char *prefix, const char *type, int count, int base, int stage, int add)
{
    int st, k, m;
    for (st = 0; st < r->nstructs; st++) if (!strcmp(r->structs[st].name, type)) break;
    if (st == r->nstructs) {
        int i = add ? uni_add(p, prefix, type, count) : uni_find(p, prefix);
        if (i >= 0) bind_add(v, i, stage, base);
        return;
    }
    for (k = 0; k < (count ? count : 1); k++) {
        const g2sc_struct *s = &r->structs[st];
        char name[96];
        int b = base + k * s->nslots;
        if (count) snprintf(name, sizeof name, "%s[%d]", prefix, k); else snprintf(name, sizeof name, "%s", prefix);
        for (m = 0; m < s->nmembers; m++) {
            char full[128];
            snprintf(full, sizeof full, "%s.%s", name, s->members[m].name);
            member(p, v, r, full, s->members[m].type, s->members[m].array, b + s->members[m].offset, stage, add);
        }
    }
}

static int hid_kind(const char *n, int *rows, int *array)
{
    int i;
    char b[96];
    for (i = 0; i < N_BI_UNI; i++) {
        snprintf(b, sizeof b, "g2u_%s", BI_UNI[i].name + 3);
        if (!strcmp(b, n)) { *rows = !strcmp(BI_UNI[i].ty, "mat4") ? 4 : !strcmp(BI_UNI[i].ty, "mat3") ? 3 : 1; *array = BI_UNI[i].count; return BI_UNI[i].kind; }
    }
    for (i = 0; i < N_BI_MEM; i++) {
        snprintf(b, sizeof b, "g2u_%s_%s", BI_MEM[i].var + 3, BI_MEM[i].mem);
        if (!strcmp(b, n)) { *rows = 1; *array = BI_MEM[i].count; return BI_MEM[i].kind; }
    }
    return -1;
}

static void hidden(gs_variant *v, const g2sc_result *r, const int *umap, int fs)
{
    int i, e;
    for (i = 0; i < r->nuniforms; i++) {
        int rows, array, kind = hid_kind(sym_n(&r->uniforms[i]), &rows, &array), n = r->uniforms[i].has_array ? r->uniforms[i].array : 1;
        (void)array;
        if (kind < 0) continue;
        for (e = 0; e < n; e++) {
            gs_hid *h;
            int slot = r->uniforms[i].loc + e * rows;
            v->hid = (gs_hid *)realloc(v->hid, ((size_t)v->nhid + 1) * sizeof *v->hid);
            h = &v->hid[v->nhid++];
            h->kind = (uint8_t)kind; h->idx = (uint8_t)e; h->fs = (uint8_t)fs; h->rows = (uint8_t)rows;
            h->slot = (uint16_t)(umap ? umap[slot] >> 2 : slot);
        }
    }
}

static int is_hidden(const char *n) { return !strncmp(n, "g2", 2) || !strncmp(n, "gl_", 3) || !strncmp(n, "_g2_", 4); }

/* §2.15.3: a built-in that is used is an active uniform too, and has no location */
static void reflect_builtins(gs_prog *p, const g2sc_result *r)
{
    int i, k;
    for (i = 0; i < r->nuniforms; i++) {
        const char *n = sym_n(&r->uniforms[i]);
        char gl[96] = "";
        int count = r->uniforms[i].has_array ? r->uniforms[i].array : 0;
        if (strncmp(n, "g2u_", 4)) continue;
        for (k = 0; k < N_BI_UNI && !gl[0]; k++) if (!strcmp(n + 4, BI_UNI[k].name + 3)) snprintf(gl, sizeof gl, "%s", BI_UNI[k].name);
        for (k = 0; k < N_BI_MEM && !gl[0]; k++) {
            char b[96];
            snprintf(b, sizeof b, "%s_%s", BI_MEM[k].var + 3, BI_MEM[k].mem);
            if (!strcmp(n + 4, b)) { snprintf(gl, sizeof gl, BI_MEM[k].count ? "%s[0].%s" : "%s.%s", BI_MEM[k].var, BI_MEM[k].mem); if (BI_MEM[k].count) count = 0; }
        }
        if (gl[0]) uni_add(p, gl, sym_t(&r->uniforms[i]), count);
    }
}

static void reqs(ff_uni *o, int *n, int max, const gl_ureq *q, int nq, const g2sc_result *r, const int *umap)
{
    int i;
    for (i = 0; i < nq; i++) {
        int s = gl_ff_find(r->uniforms, r->nuniforms, q[i].name);
        if (s >= 0 && *n < max) { ff_uni *u = &o[(*n)++]; u->kind = q[i].kind; u->idx = q[i].idx; u->slot = (uint16_t)(umap ? umap[s] >> 2 : s); }
    }
}

/* the texts of a variant. A stage the program does not have is a generated one */
static int texts(gs_prog *p, const gs_key *k, tb *vs, tb *fs, xl *xv, xl *xf, gl_ureq *ffvu, int *nvu, gl_ureq *fffu, int *nfu, char *err, size_t nerr)
{
    char *gv = NULL, *gf = NULL;
    int hv = p->has_vs && !k->pix;
    *nvu = *nfu = 0;
    if (!hv || !p->has_fs) gl_ff_text(&k->ff, &gv, &gf, ffvu, nvu, fffu, nfu);
    memset(xv, 0, sizeof *xv); memset(xf, 0, sizeof *xf);
    xv->p = xf->p = p; xv->k = xf->k = k; xf->fs = 1; xf->spec = 1;
    if (hv) xl_vs(xv, vs); else tb_put(vs, gv, strlen(gv));
    if (p->has_fs) xl_fs(xf, fs); else tb_put(fs, gf, strlen(gf));
    free(gv); free(gf);
    if (hv) *nvu = 0;
    if (p->has_fs) *nfu = 0;
    free(xv->body.p); free(xv->head.p); free(xf->body.p); free(xf->head.p);
    xv->body.p = xf->body.p = NULL;
    if (xv->err[0] || xf->err[0]) { snprintf(err, nerr, "%s", xv->err[0] ? xv->err : xf->err); return -1; }
    return 0;
}

/* attribute locations: bound names first, the rest take free ones; a matrix takes consecutive ones */
static int locate(gs_prog *p, const g2sc_result *rv, char *err, size_t nerr)
{
    int i, k, used[MAX_ATTRIBS];
    memset(used, 0, sizeof used);
    p->nattr = 0;
    p->gen_mask = 0;
    p->vertex_alias = gl_ff_find(rv->attribs, rv->nattribs, "g2a_Vertex") >= 0;
    if (p->vertex_alias) used[0] = 1;
    for (i = 0; i < rv->nattribs; i++) {
        const char *n = sym_n(&rv->attribs[i]);
        gs_attr *a;
        int b;
        if (p->nattr == MAX_ATTRIBS) { snprintf(err, nerr, "too many attributes"); return -1; }
        a = &p->attr[p->nattr++];
        memset(a, 0, sizeof *a);
        a->type = type_enum(sym_t(&rv->attribs[i]));
        { int c, bs; type_shape(a->type, &a->rows, &c, &bs); }
        a->loc = -1; a->a = -1;
        if (!strncmp(n, "g2a", 3)) {
            int q;
            a->a = attr_of_builtin(n);
            for (q = 0; q < N_BI_ATTR; q++) if (!strcmp(BI_ATTR[q].as, n) && BI_ATTR[q].name) snprintf(a->name, sizeof a->name, "%s", BI_ATTR[q].name);
            continue;
        }
        snprintf(a->name, sizeof a->name, "%s", n);
        for (k = 0; k < p->nbind; k++) {
            if (!strcmp(p->bind[k].name, n)) {
                a->loc = (int)p->bind[k].loc;
                if (a->loc + a->rows > MAX_ATTRIBS) { snprintf(err, nerr, "attribute %s does not fit at its location", n); return -1; }
                for (b = 0; b < a->rows; b++) used[a->loc + b] = 1;
            }
        }
    }
    for (i = 0; i < p->nattr; i++) {
        gs_attr *a = &p->attr[i];
        int next;
        if (a->loc >= 0 || a->a >= 0) continue;
        for (next = 0; next + a->rows <= MAX_ATTRIBS; next++) {
            for (k = 0; k < a->rows; k++) if (used[next + k]) break;
            if (k == a->rows) break;
        }
        if (next + a->rows > MAX_ATTRIBS) { snprintf(err, nerr, "too many attributes"); return -1; }
        a->loc = next;
        for (k = 0; k < a->rows; k++) used[next + k] = 1;
    }
    for (i = 0; i < p->nattr; i++) {
        const gs_attr *a = &p->attr[i];
        if (a->loc > 0) for (k = 0; k < a->rows; k++) p->gen_mask |= 1u << (a->loc + k);
        else if (a->loc == 0) for (k = 1; k < a->rows; k++) p->gen_mask |= 1u << k;
    }
    return 0;
}

/* the row a vec4 varying leaves the vertex program in, as gl_vs_exec gives it; -1 when it is not passed on */
static int vary_row(const g2sc_linked *l, const char *name)
{
    int s = gl_ff_find(l->vs.varyings, l->vs.nvaryings, name);
    if (s < 0 || !l->vpacked) return s;
    return s < (int)(sizeof l->vmap / sizeof l->vmap[0]) && l->vmap[s] >= 0 ? l->vmap[s] >> 2 : -1;
}

static gs_variant *build(gs_prog *p, const gs_key *k, char *err, size_t nerr, int first)
{
    gs_variant *v;
    tb vs = { 0 }, fs = { 0 };
    xl *xv = (xl *)malloc(sizeof *xv), *xf = (xl *)malloc(sizeof *xf);
    gl_ureq ffvu[FF_MAX_UNI], fffu[FF_MAX_UNI];
    int nvu = 0, nfu = 0, i, s, rc, hv = p->has_vs && !k->pix;
    g2sc_linked l;
    g2r_prog *hw;
    if (!xv || !xf) { free(xv); free(xf); snprintf(err, nerr, "out of memory"); return NULL; }
    rc = texts(p, k, &vs, &fs, xv, xf, ffvu, &nvu, fffu, &nfu, err, nerr);
    if (rc < 0) { free(xv); free(xf); free(vs.p); free(fs.p); return NULL; }
    if (g2sc_link(vs.p, fs.p, &l) < 0) {
        snprintf(err, nerr, "%s", l.error);
        g2e_log("gl: a program does not link: %s\n--- vs\n%s\n--- fs\n%s", l.error, vs.p, fs.p);
        free(xv); free(xf); free(vs.p); free(fs.p);
        return NULL;
    }
    free(vs.p); free(fs.p);
    v = (gs_variant *)calloc(1, sizeof *v);
    if (l.vs.vs_tex) { snprintf(err, nerr, "the vertex stage has no texture units (MAX_VERTEX_TEXTURE_IMAGE_UNITS is 0)"); goto fail; }
    if (first && locate(p, &l.vs, err, nerr) < 0) goto fail;
    hw = g2r_prog_create(D.r, l.vs.words, (uint32_t)l.vs.nwords, (const double (*)[4])l.vs.const_pool, (uint32_t)l.vs.nconst, (uint32_t)l.vs.const_base, l.fs.words, (uint32_t)l.fs.nwords,
                         (const double (*)[4])l.fs.const_pool, (uint32_t)l.fs.nconst, (uint32_t)l.fs.const_base, l.fsmask);
    if (!hw) { snprintf(err, nerr, "no room for the programs on the boards"); goto fail; }
    v->key = *k;
    v->ff.p = hw;
    v->ff.key = p->has_fs ? k->sk : k->ff;
    v->ff.user = v;
    v->ff.user_fs = p->has_fs;
    for (i = 0; i < A_COUNT; i++) v->ff.attr_loc[i] = -1;
    for (i = 0; i < l.vs.nattribs; i++) {
        const char *n = sym_n(&l.vs.attribs[i]);
        int a = hv ? attr_of_builtin(n) : gl_ff_attr_of(n);
        if (a != 255 && a >= 0) { v->ff.attr_loc[a] = l.vs.attribs[i].loc; continue; }
        for (s = 0; s < p->nattr; s++) {
            if (!strcmp(p->attr[s].name, n)) {
                int r;
                for (r = 0; r < p->attr[s].rows; r++) {
                    int lc = p->attr[s].loc + r;
                    v->ff.attr_loc[lc ? A_GEN + lc - 1 : A_POS] = l.vs.attribs[i].loc + r;
                }
            }
        }
    }
    reqs(v->ff.vu, &v->ff.nvu, FF_MAX_UNI, ffvu, nvu, &l.vs, l.umap_vs);
    reqs(v->ff.vu, &v->ff.nvu, FF_MAX_UNI, xv->ur, xv->nur, &l.vs, l.umap_vs);
    reqs(v->ff.fu, &v->ff.nfu, FF_MAX_UNI / 2, fffu, nfu, &l.fs, l.umap_fs);
    reqs(v->ff.fu, &v->ff.nfu, FF_MAX_UNI / 2, xf->ur, xf->nur, &l.fs, l.umap_fs);
    for (i = 0; i < MAX_UNITS; i++) {
        char nm[16];
        snprintf(nm, sizeof nm, "g2Tex%d", i);
        v->ff.sampler[i] = gl_ff_find(l.fs.samplers, l.fs.nsamplers, nm);
        v->samp_slot[i] = -1;
    }
    v->ff.stip_sampler = gl_ff_find(l.fs.samplers, l.fs.nsamplers, p->has_fs ? "g2Stip" : "uStip");
    v->pix_slot = gl_ff_find(l.fs.samplers, l.fs.nsamplers, "g2Pix");
    for (i = 0; i < p->nsamp; i++) {
        const gs_samp *m = &p->samp[i];
        int e;
        if (!p->has_fs) break;
        for (e = 0; e < (m->count ? m->count : 1); e++) {
            int sl = v->ff.sampler[m->first + e] >= 0 ? v->ff.sampler[m->first + e] : gl_ff_find(l.fs.samplers, l.fs.nsamplers, m->name);
            if (sl >= 0) v->samp_slot[m->first + e] = sl + (v->ff.sampler[m->first + e] >= 0 ? 0 : e);
        }
    }
    /* uniforms: the first variant's tables are the program's */
    for (s = 0; s < 2; s++) {
        const g2sc_result *r = s ? &l.fs : &l.vs;
        const int *umap = s ? l.umap_fs : l.umap_vs;
        if (s ? !p->has_fs : !hv) continue;
        for (i = 0; i < r->nuniforms; i++) {
            const char *n = sym_n(&r->uniforms[i]);
            if (!is_hidden(n)) member(p, v, r, n, sym_t(&r->uniforms[i]), r->uniforms[i].has_array ? r->uniforms[i].array : 0, r->uniforms[i].loc, s, first);
        }
        hidden(v, r, umap, s);
        if (first) reflect_builtins(p, r);
        for (i = 0; i < v->nbind; i++) {
            int *sl = s ? &v->bind[i].fs : &v->bind[i].vs, *c = s ? &v->bind[i].fc : &v->bind[i].vc;
            if (umap && *sl >= 0 && !*c) { int at = umap[*sl]; *sl = at >> 2; *c = at & 3 ? at & 3 : 8; }
        }
        for (i = 0; i < v->nbind; i++) { if (s) { if (v->bind[i].fc == 8) v->bind[i].fc = 0; } else if (v->bind[i].vc == 8) v->bind[i].vc = 0; }
    }
    if (hv) {                                                   /* a position the PC must know: lines stippled or antialiased, polygon mode, selection and feedback */
        v->sasm = _strdup(l.vs.sasm);
        v->ncpool = l.vs.nconst; v->cbase = l.vs.const_base;
        if (l.vs.nconst) { v->cpool = (double (*)[4])malloc((size_t)l.vs.nconst * sizeof *v->cpool); if (v->cpool) memcpy(v->cpool, l.vs.const_pool, (size_t)l.vs.nconst * sizeof *v->cpool); }
        v->vary_color = vary_row(&l, "g2vColor");
        v->vary_tc0 = vary_row(&l, "g2vTc0");
        for (i = 0; i < MAX_CLIP; i++) { char nm[16]; snprintf(nm, sizeof nm, "g2vClip%d", i); v->vary_clip[i] = vary_row(&l, nm); }
        v->vary_flat[0] = vary_row(&l, "g2vColor");
        v->vary_flat[1] = vary_row(&l, "g2vColorB");
        v->vary_flat[2] = vary_row(&l, "g2vSpec");
        v->vary_flat[3] = vary_row(&l, "g2vSpecB");
    }
    g2e_log("gl: program %u, variant %u: VS %d, FS %d instructions", p->name, hw->id, l.vs.nwords, l.fs.nwords);
    v->next = p->variants;
    p->variants = v;
    p->last = v;
    g2sc_link_free(&l);
    free(xv); free(xf);
    return v;
fail:
    g2sc_link_free(&l);
    free(v); free(xv); free(xf);
    return NULL;
}

ff_prog *gl_glsl_program(void)
{
    gs_prog *p = C->cur;
    gs_key k;
    gs_variant *v;
    char err[512];
    key_of(p, &k);
    if (p->last && !memcmp(&p->last->key, &k, sizeof k)) return &p->last->ff;
    for (v = p->variants; v; v = v->next) if (!memcmp(&v->key, &k, sizeof k)) { p->last = v; return &v->ff; }
    err[0] = 0;
    v = build(p, &k, err, sizeof err, 0);
    if (!v) { g2e_log("gl: program %u: a variant does not build: %s", p->name, err); return NULL; }
    return &v->ff;
}

/* the values of a draw */
void gl_glsl_uniforms(const ff_prog *ff, uint32_t *vs, uint32_t *fs)
{
    const gs_variant *v = ff->user;
    const gs_prog *p = C->cur;
    int i, r, c;
    for (i = 0; i < v->nbind; i++) {
        const gs_bind *b = &v->bind[i];
        const gs_uni *u = &p->uni[b->uni];
        int rows = u->size * u->stride;
        if (b->vs >= 0) for (r = 0; r < rows && b->vs + r < 256; r++) for (c = 0; c < u->comps; c++) vs[4 * (b->vs + r) + b->vc + c] = u->bits[r * u->comps + c];
        if (b->fs >= 0) for (r = 0; r < rows && b->fs + r < 64; r++) for (c = 0; c < u->comps; c++) fs[4 * (b->fs + r) + b->fc + c] = u->bits[r * u->comps + c];
    }
    for (i = 0; i < v->nhid; i++) {
        const gs_hid *h = &v->hid[i];
        float o[16];
        uint32_t *d = (h->fs ? fs : vs) + 4 * h->slot;
        bi_fill(h->kind, h->idx, o);
        for (r = 0; r < h->rows * 4; r++) d[r] = g2_fp24(o[r]);
    }
}

const gl_tex *gl_glsl_tex(const ff_prog *ff, int s)
{
    const gs_prog *p = C->cur;
    int i;
    (void)ff;
    for (i = 0; i < p->nsamp; i++) {
        const gs_samp *d = &p->samp[i];
        if (s >= d->first && s < d->first + (d->count ? d->count : 1)) return gl_unit_tex(p->samp_unit[s], samp_kind(d->type));
    }
    return gl_unit_tex(0, 0);
}

static uint32_t wrap_hw(GLenum w) { return w == GL_REPEAT ? G2R_WRAP_REPEAT : w == GL_MIRRORED_REPEAT ? G2R_WRAP_MIRROR : G2R_WRAP_CLAMP; }

void gl_glsl_samplers(g2r_state *st, const ff_prog *ff)
{
    const gs_variant *v = ff->user;
    const gs_prog *p = C->cur;
    int m, e;
    if (v->pix_slot >= 0) {
        gl_tex *t = gl_tex_lookup(C->pic_tex, GL_TEXTURE_2D, 0);
        if (t) { gl_tex_prepare(t); st->h[15 + v->pix_slot] = G2R_SAMPLER(t->handle, G2R_FILTER(0, 0, 0), G2R_WRAP_CLAMP, G2R_WRAP_CLAMP, 0); }
    }
    for (m = 0; m < p->nsamp; m++) {
        const gs_samp *d = &p->samp[m];
        for (e = 0; e < (d->count ? d->count : 1); e++) {
            int s = d->first + e, slot = v->samp_slot[s], oned = d->type == ST_1D || d->type == ST_1DS;
            gl_tex *t;
            uint32_t mip, min_l;
            if (slot < 0) continue;
            if (!ff->key.u[s].target) { st->h[15 + slot] = G2R_SAMPLER(0, 0, 0, 0, 1); continue; }  /* it reads (0, 0, 0, 1) */
            t = gl_unit_tex(p->samp_unit[s], samp_kind(d->type));
            gl_tex_prepare(t);
            if (ff->key.u[s].sw) {
                uint16_t im = ff->key.u[s].sw == 3 ? t->handle : ff->key.u[s].swb ? t->bimg : gl_tex_sample_image(t);
                st->h[15 + slot] = G2R_SAMPLER3D(im, G2R_FILTER(0, ff->key.u[s].swn > 1 ? 1 : 0, 0), G2R_WRAP_CLAMP, G2R_WRAP_CLAMP, G2R_WRAP_CLAMP, 0);
                continue;
            }
            min_l = t->min_filter == GL_LINEAR || t->min_filter == GL_LINEAR_MIPMAP_NEAREST || t->min_filter == GL_LINEAR_MIPMAP_LINEAR;
            mip = t->min_filter == GL_NEAREST_MIPMAP_NEAREST || t->min_filter == GL_LINEAR_MIPMAP_NEAREST ? 1 : t->min_filter == GL_NEAREST_MIPMAP_LINEAR || t->min_filter == GL_LINEAR_MIPMAP_LINEAR ? 2 : 0;
            st->h[15 + slot] = G2R_SAMPLER3D(gl_tex_sample_image(t), G2R_FILTER(t->mag_filter == GL_LINEAR, mip, min_l), wrap_hw(t->wrap_s), oned ? G2R_WRAP_CLAMP : wrap_hw(t->wrap_t), d->type == ST_3D ? wrap_hw(t->wrap_r) : 0, 0);
        }
    }
}

uint32_t gl_glsl_attribs(void) { return C->cur ? C->cur->gen_mask : 0; }
int gl_glsl_user_vs(void) { return C->cur && C->cur->has_vs && !C->internal_draw; }
int gl_glsl_user_fs(void) { return C->cur && C->cur->has_fs; }

/* ---------------------------------------------------------------- shader objects */
GLuint APIENTRY glCreateShader(GLenum type)
{
    gs_shader *s;
    if (type != GL_VERTEX_SHADER && type != GL_FRAGMENT_SHADER) { SET_ERROR(GL_INVALID_ENUM); return 0; }
    s = (gs_shader *)calloc(1, sizeof *s);
    if (!s) return 0;
    s->name = g_next++; s->type = type;
    s->next = g_sh[s->name % G_BUCKETS];
    g_sh[s->name % G_BUCKETS] = s;
    return s->name;
}

void APIENTRY glDeleteShader(GLuint n)
{
    gs_shader *s = sh_of(n);
    if (!n) return;
    if (!s) { SET_ERROR(GL_INVALID_VALUE); return; }
    s->del = 1;
    if (!s->refs) sh_free(s);
}

GLboolean APIENTRY glIsShader(GLuint n) { return n && sh_of(n) ? GL_TRUE : GL_FALSE; }

void APIENTRY glShaderSource(GLuint n, GLsizei count, const GLchar *const *str, const GLint *len)
{
    gs_shader *s = sh_of(n);
    size_t total = 0, o = 0;
    GLsizei i;
    if (!s || count < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    for (i = 0; i < count; i++) total += len && len[i] >= 0 ? (size_t)len[i] : strlen(str[i]);
    free(s->src);
    s->src = (char *)malloc(total + 1);
    if (!s->src) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    for (i = 0; i < count; i++) {
        size_t l = len && len[i] >= 0 ? (size_t)len[i] : strlen(str[i]);
        memcpy(s->src + o, str[i], l);
        o += l;
    }
    s->src[o] = 0;
}

/* a program of one shader, for its errors: the stage alone, with the samplers that are always the program's own to read */
static void default_key(gs_prog *p, gs_key *k)
{
    int m, e;
    memset(k, 0, sizeof *k);
    k->ff.alpha_fn = 7;
    for (m = 0; m < p->nsamp; m++) {
        const gs_samp *d = &p->samp[m];
        for (e = 0; e < (d->count ? d->count : 1); e++) {
            int s = d->first + e;
            k->sk.u[s].target = (uint8_t)(d->type == ST_1D || d->type == ST_1DS ? 3 : d->type == ST_3D ? 4 : d->type == ST_CUBE ? 2 : 1);
            if (!d->count && d->type >= ST_3D && d->type != ST_CUBE && !(d->type == ST_3D && D.r && g2r_has_3d(D.r))) {
                k->sk.u[s].sw = d->type == ST_3D ? 3 : 2;
                k->sk.u[s].swn = 1;
                k->sk.u[s].dmode = d->type == ST_3D ? 0 : 1;
                k->sk.u[s].sww[0] = k->sk.u[s].sww[1] = k->sk.u[s].sww[2] = 3;
            }
        }
    }
}

void APIENTRY glCompileShader(GLuint n)
{
    gs_shader *s = sh_of(n);
    gs_prog *p;
    gs_key k;
    xl *x;
    tb t = { 0 }, src = { 0 };
    g2sc_result r;
    int fs;
    if (!s) { SET_ERROR(GL_INVALID_VALUE); return; }
    s->compiled = 0; s->log[0] = 0;
    if (!s->src) { snprintf(s->log, sizeof s->log, "no source"); return; }
    fs = s->type == GL_FRAGMENT_SHADER;
    p = (gs_prog *)calloc(1, sizeof *p);
    x = (xl *)calloc(1, sizeof *x);
    if (!p || !x) { free(p); free(x); return; }
    p->has_vs = !fs; p->has_fs = fs;
    scan(s->src, &p->info[fs], fs ? p : NULL);
    tb_put(&src, s->src, strlen(s->src));
    if (!p->info[fs].has_main) tb_f(&src, "\nvoid main() { }\n");   /* a shader of functions for another: main is elsewhere */
    p->src[fs] = src.p;
    default_key(p, &k);
    x->p = p; x->k = &k; x->fs = fs;
    if (fs) xl_fs(x, &t); else xl_vs(x, &t);
    memset(&r, 0, sizeof r);
    if (x->err[0]) snprintf(s->log, sizeof s->log, "%s", x->err);
    else if (g2sc_compile_ex(t.p, fs ? G2SC_FS : G2SC_VS, NULL, 0, &r) == 0) s->compiled = 1;
    else if (strstr(r.error, "unknown function/builtin")) s->compiled = 1;      /* §2.15.2: it may be in another shader of the program: the link says */
    else { snprintf(s->log, sizeof s->log, "%s", r.error); g2e_log("gl: a shader does not compile: %s\n%s", r.error, t.p); }
    g2sc_free(&r);
    free(t.p); free(src.p); free(x->body.p); free(x); free(p);
}

void APIENTRY glGetShaderiv(GLuint n, GLenum q, GLint *v)
{
    gs_shader *s = sh_of(n);
    if (!s) { SET_ERROR(GL_INVALID_VALUE); return; }
    switch (q) {
    case GL_SHADER_TYPE: *v = (GLint)s->type; break;
    case GL_DELETE_STATUS: *v = s->del; break;
    case GL_COMPILE_STATUS: *v = s->compiled; break;
    case GL_INFO_LOG_LENGTH: *v = s->log[0] ? (GLint)strlen(s->log) + 1 : 0; break;
    case GL_SHADER_SOURCE_LENGTH: *v = s->src ? (GLint)strlen(s->src) + 1 : 0; break;
    default: SET_ERROR(GL_INVALID_ENUM); break;
    }
}

static void give(const char *text, GLsizei max, GLsizei *len, GLchar *o)
{
    int l;
    if (max < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (!max) { if (len) *len = 0; return; }
    l = snprintf(o, (size_t)max, "%s", text);
    if (len) *len = l < max ? l : max - 1;
}

void APIENTRY glGetShaderInfoLog(GLuint n, GLsizei max, GLsizei *len, GLchar *log) { gs_shader *s = sh_of(n); if (!s) { SET_ERROR(GL_INVALID_VALUE); return; } give(s->log, max, len, log); }
void APIENTRY glGetShaderSource(GLuint n, GLsizei max, GLsizei *len, GLchar *src) { gs_shader *s = sh_of(n); if (!s) { SET_ERROR(GL_INVALID_VALUE); return; } give(s->src ? s->src : "", max, len, src); }

/* ---------------------------------------------------------------- programs */
GLuint APIENTRY glCreateProgram(void)
{
    gs_prog *p = (gs_prog *)calloc(1, sizeof *p);
    if (!p) return 0;
    p->name = g_next++;
    p->next = g_pr[p->name % G_BUCKETS];
    g_pr[p->name % G_BUCKETS] = p;
    return p->name;
}

GLboolean APIENTRY glIsProgram(GLuint n) { return n && pr_of(n) ? GL_TRUE : GL_FALSE; }

static void uni_clear(gs_prog *p)
{
    int i;
    for (i = 0; i < p->nuni; i++) { free(p->uni[i].val); free(p->uni[i].bits); }
    free(p->uni);
    p->uni = NULL; p->nuni = 0;
}

/* a hardware program stays on the board: the ring of programs is the renderer's */
static void variants_clear(gs_prog *p)
{
    gs_variant *v = p->variants;
    while (v) {
        gs_variant *n = v->next;
        free(v->bind); free(v->hid); free(v->sasm); free(v->cpool); gl_vs_free(v->vm); free(v);
        v = n;
    }
    p->variants = p->last = NULL;
}

static void pr_free(gs_prog *p)
{
    gs_prog **q;
    int i;
    for (i = 0; i < p->natt; i++) {
        gs_shader *s = sh_of(p->att[i]);
        if (s && !--s->refs && s->del) sh_free(s);
    }
    uni_clear(p); variants_clear(p);
    free(p->src[0]); free(p->src[1]);
    for (q = &g_pr[p->name % G_BUCKETS]; *q; q = &(*q)->next) if (*q == p) { *q = p->next; break; }
    free(p);
}

void APIENTRY glDeleteProgram(GLuint n)
{
    gs_prog *p = pr_of(n);
    if (!n) return;
    if (!p) { SET_ERROR(GL_INVALID_VALUE); return; }
    p->del = 1;
    if (C->cur != p) pr_free(p);                                /* else when it stops being in use */
}

void APIENTRY glAttachShader(GLuint pn, GLuint sn)
{
    gs_prog *p = pr_of(pn);
    gs_shader *s = sh_of(sn);
    int i;
    if (!p || !s) { SET_ERROR(GL_INVALID_VALUE); return; }
    for (i = 0; i < p->natt; i++) if (p->att[i] == sn) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (p->natt == (int)(sizeof p->att / sizeof p->att[0])) { SET_ERROR(GL_OUT_OF_MEMORY); return; }
    p->att[p->natt++] = sn;
    s->refs++;
}

void APIENTRY glDetachShader(GLuint pn, GLuint sn)
{
    gs_prog *p = pr_of(pn);
    gs_shader *s = sh_of(sn);
    int i;
    if (!p || !s) { SET_ERROR(GL_INVALID_VALUE); return; }
    for (i = 0; i < p->natt; i++) if (p->att[i] == sn) break;
    if (i == p->natt) { SET_ERROR(GL_INVALID_OPERATION); return; }
    memmove(p->att + i, p->att + i + 1, (size_t)(p->natt - i - 1) * sizeof p->att[0]);
    p->natt--;
    if (!--s->refs && s->del) sh_free(s);
}

void APIENTRY glGetAttachedShaders(GLuint pn, GLsizei max, GLsizei *n, GLuint *out)
{
    gs_prog *p = pr_of(pn);
    int c = 0, i;
    if (!p || max < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    for (i = 0; i < p->natt && c < max; i++) out[c++] = p->att[i];
    if (n) *n = c;
}

void APIENTRY glBindAttribLocation(GLuint pn, GLuint loc, const GLchar *name)
{
    gs_prog *p = pr_of(pn);
    int i;
    if (!p || loc >= MAX_ATTRIBS) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (!strncmp(name, "gl_", 3)) { SET_ERROR(GL_INVALID_OPERATION); return; }
    for (i = 0; i < p->nbind; i++) if (!strcmp(p->bind[i].name, name)) break;
    if (i == p->nbind) {
        if (p->nbind == (int)(sizeof p->bind / sizeof p->bind[0])) return;
        p->nbind++;
    }
    snprintf(p->bind[i].name, sizeof p->bind[i].name, "%s", name);
    p->bind[i].loc = loc;
}

/* what a program runs, apart from what the application set up */
static void exec_move(gs_prog *to, const gs_prog *from)
{
    to->has_vs = from->has_vs; to->has_fs = from->has_fs;
    to->src[0] = from->src[0]; to->src[1] = from->src[1];
    memcpy(to->info, from->info, sizeof to->info);
    memcpy(to->samp, from->samp, sizeof to->samp); to->nsamp = from->nsamp; to->nel = from->nel;
    memcpy(to->samp_unit, from->samp_unit, sizeof to->samp_unit);
    to->uni = from->uni; to->nuni = from->nuni;
    memcpy(to->attr, from->attr, sizeof to->attr); to->nattr = from->nattr;
    to->gen_mask = from->gen_mask; to->vertex_alias = from->vertex_alias;
    to->variants = from->variants; to->last = from->last;
}

void APIENTRY glLinkProgram(GLuint pn)
{
    gs_prog *p = pr_of(pn), *keep;
    tb src[2] = { { 0 }, { 0 } };
    gs_key k;
    char err[sizeof p->log];
    int i, st, mains[2] = { 0, 0 }, has[2] = { 0, 0 };
    if (!p) { SET_ERROR(GL_INVALID_VALUE); return; }
    FLUSH_VERTICES();
    p->linked = 0; p->validated = 0; p->log[0] = 0; err[0] = 0;
    if (!p->natt) { snprintf(p->log, sizeof p->log, "no shaders"); return; }
    for (i = 0; i < p->natt; i++) {
        gs_shader *s = sh_of(p->att[i]);
        gs_info o;
        if (!s) continue;
        st = s->type == GL_FRAGMENT_SHADER;
        if (!s->compiled) { snprintf(p->log, sizeof p->log, "a shader did not compile"); free(src[0].p); free(src[1].p); return; }
        scan(s->src, &o, NULL);
        tb_put(&src[st], s->src, strlen(s->src));
        tb_f(&src[st], "\n");
        mains[st] += o.has_main;
        has[st] = 1;
    }
    if ((has[0] && mains[0] != 1) || (has[1] && mains[1] != 1)) {
        snprintf(p->log, sizeof p->log, "a stage needs one main()");
        free(src[0].p); free(src[1].p);
        return;
    }
    if (!D.r) { snprintf(p->log, sizeof p->log, "no current context"); free(src[0].p); free(src[1].p); return; }
    /* §2.15.2: a link that fails leaves what the program was running, which a program in use goes on running */
    keep = (gs_prog *)calloc(1, sizeof *keep);
    if (!keep) { free(src[0].p); free(src[1].p); return; }
    exec_move(keep, p);
    p->uni = NULL; p->nuni = 0; p->variants = p->last = NULL;
    p->has_vs = has[0]; p->has_fs = has[1];
    p->nsamp = p->nel = 0;
    p->src[0] = src[0].p; p->src[1] = src[1].p;
    if (p->has_vs) scan(p->src[0], &p->info[0], NULL);
    if (p->has_fs) scan(p->src[1], &p->info[1], p);
    memset(p->samp_unit, 0, sizeof p->samp_unit);
    key_of(p, &k);
    if (!build(p, &k, err, sizeof err, 1)) {
        uni_clear(p); variants_clear(p);
        free(p->src[0]); free(p->src[1]);
        exec_move(p, keep);
        free(keep);
        snprintf(p->log, sizeof p->log, "%s", err);
        g2e_log("gl: link failed: %s", p->log);
        return;
    }
    uni_clear(keep); variants_clear(keep);
    free(keep->src[0]); free(keep->src[1]);
    free(keep);
    for (i = 0; i < p->nsamp; i++) {                            /* a sampler answers as the uniform it is */
        static const GLenum T[6] = { GL_SAMPLER_1D, GL_SAMPLER_2D, GL_SAMPLER_3D, GL_SAMPLER_CUBE, GL_SAMPLER_1D_SHADOW, GL_SAMPLER_2D_SHADOW };
        const gs_samp *m = &p->samp[i];
        gs_uni *u;
        p->uni = (gs_uni *)realloc(p->uni, ((size_t)p->nuni + 1) * sizeof *p->uni);
        u = &p->uni[p->nuni++];
        memset(u, 0, sizeof *u);
        snprintf(u->name, sizeof u->name, "%s", m->name);
        u->type = T[m->type]; u->size = m->count ? m->count : 1; u->stride = 1; u->comps = 1; u->base = 1; u->sampler = m->first;
    }
    p->linked = 1;
}

void APIENTRY glUseProgram(GLuint n)
{
    gs_prog *p = n ? pr_of(n) : NULL, *old = C->cur;
    if (C->compiling) { gl_list_i2(LOP_USEPROG, n, 0); if (C->compile_mode == GL_COMPILE) return; }
    FLUSH_VERTICES();
    if (n && !p) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (p && !p->linked) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (C->in_begin) { SET_ERROR(GL_INVALID_OPERATION); return; }
    C->cur = p;
    C->program = n;
    if (old && old != p && old->del) pr_free(old);
}

/* §2.15.4: samplers of different kinds cannot share a unit */
void APIENTRY glValidateProgram(GLuint n)
{
    gs_prog *p = pr_of(n);
    int a, b, x, y;
    if (!p) { SET_ERROR(GL_INVALID_VALUE); return; }
    p->validated = p->linked;
    p->log[0] = 0;
    if (!p->linked) return;
    for (a = 0; a < p->nsamp; a++) {
        for (b = a + 1; b < p->nsamp; b++) {
            const gs_samp *s = &p->samp[a], *t = &p->samp[b];
            if (samp_kind(s->type) == samp_kind(t->type)) continue;
            for (x = 0; x < (s->count ? s->count : 1); x++) {
                for (y = 0; y < (t->count ? t->count : 1); y++) {
                    if (p->samp_unit[s->first + x] == p->samp_unit[t->first + y]) {
                        p->validated = 0;
                        snprintf(p->log, sizeof p->log, "samplers %s and %s are of different kinds on one texture unit", s->name, t->name);
                    }
                }
            }
        }
    }
}

void APIENTRY glGetProgramiv(GLuint n, GLenum q, GLint *v)
{
    gs_prog *p = pr_of(n);
    int i, m = 0;
    if (!p) { SET_ERROR(GL_INVALID_VALUE); return; }
    switch (q) {
    case GL_DELETE_STATUS: *v = p->del; break;
    case GL_LINK_STATUS: *v = p->linked; break;
    case GL_VALIDATE_STATUS: *v = p->validated; break;
    case GL_INFO_LOG_LENGTH: *v = p->log[0] ? (GLint)strlen(p->log) + 1 : 0; break;
    case GL_ATTACHED_SHADERS: *v = p->natt; break;
    case GL_ACTIVE_ATTRIBUTES: *v = p->nattr; break;
    case GL_ACTIVE_ATTRIBUTE_MAX_LENGTH: for (i = 0; i < p->nattr; i++) { int l = (int)strlen(p->attr[i].name) + 1; if (l > m) m = l; } *v = m; break;
    case GL_ACTIVE_UNIFORMS: *v = p->nuni; break;
    case GL_ACTIVE_UNIFORM_MAX_LENGTH: for (i = 0; i < p->nuni; i++) { int l = (int)strlen(p->uni[i].name) + 4; if (l > m) m = l; } *v = m; break;
    default: SET_ERROR(GL_INVALID_ENUM); break;
    }
}

void APIENTRY glGetProgramInfoLog(GLuint n, GLsizei max, GLsizei *len, GLchar *log) { gs_prog *p = pr_of(n); if (!p) { SET_ERROR(GL_INVALID_VALUE); return; } give(p->log, max, len, log); }

void APIENTRY glGetActiveAttrib(GLuint n, GLuint i, GLsizei max, GLsizei *len, GLint *size, GLenum *type, GLchar *name)
{
    gs_prog *p = pr_of(n);
    if (!p || (int)i >= p->nattr) { SET_ERROR(GL_INVALID_VALUE); return; }
    give(p->attr[i].name, max, len, name);
    *size = 1; *type = p->attr[i].type;
}

GLint APIENTRY glGetAttribLocation(GLuint n, const GLchar *name)
{
    gs_prog *p = pr_of(n);
    int i;
    if (!p || !p->linked) { SET_ERROR(GL_INVALID_OPERATION); return -1; }
    if (!strncmp(name, "gl_", 3)) return -1;
    for (i = 0; i < p->nattr; i++) if (!strcmp(p->attr[i].name, name)) return p->attr[i].loc;
    return -1;
}

void APIENTRY glGetActiveUniform(GLuint n, GLuint i, GLsizei max, GLsizei *len, GLint *size, GLenum *type, GLchar *name)
{
    gs_prog *p = pr_of(n);
    char b[110];
    if (!p || (int)i >= p->nuni) { SET_ERROR(GL_INVALID_VALUE); return; }
    snprintf(b, sizeof b, "%s%s", p->uni[i].name, p->uni[i].size > 1 ? "[0]" : "");
    give(b, max, len, name);
    *size = p->uni[i].size; *type = p->uni[i].type;
}

/* a location: uniform index << 12 | array element */
GLint APIENTRY glGetUniformLocation(GLuint n, const GLchar *name)
{
    gs_prog *p = pr_of(n);
    char base[96];
    char *br;
    int el, i;
    if (!p || !p->linked) { SET_ERROR(GL_INVALID_OPERATION); return -1; }
    if (!strncmp(name, "gl_", 3)) return -1;
    snprintf(base, sizeof base, "%s", name);
    i = uni_find(p, base);
    if (i >= 0) return i << 12;
    br = strrchr(base, '[');
    if (br && br[strlen(br) - 1] == ']') {
        el = atoi(br + 1);
        *br = 0;
        i = uni_find(p, base);
        if (i >= 0 && el >= 0 && el < p->uni[i].size) return i << 12 | el;
    }
    return -1;
}

/* §2.15.3. kind: 'f', 'i', 'm' (a matrix of comps columns). Ints go to the hardware as floats: the compiler keeps them that way, FTOI on load */
static void uni_set(GLint loc, GLsizei n, int kind, int comps, const float *v, const int *iv)
{
    gs_prog *p = C->cur;
    gs_uni *u;
    int e, c, idx, el, per, ismat;
    if (C->compiling) {
        gl_list_uniform(loc, kind, comps, n, kind == 'i' ? (const void *)iv : (const void *)v, (size_t)n * (size_t)(kind == 'm' ? comps * comps : comps) * 4);
        if (C->compile_mode == GL_COMPILE) return;
    }
    FLUSH_VERTICES();
    if (!p) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (n < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (loc == -1) return;
    idx = loc >> 12; el = loc & 4095;
    if (idx < 0 || idx >= p->nuni || el >= p->uni[idx].size) { SET_ERROR(GL_INVALID_OPERATION); return; }
    u = &p->uni[idx];
    if (n > 1 && u->size == 1) { SET_ERROR(GL_INVALID_OPERATION); return; }
    if (u->sampler >= 0) {
        if (kind != 'i' || comps != 1) { SET_ERROR(GL_INVALID_OPERATION); return; }
        for (e = 0; e < n; e++) if (iv[e] < 0 || iv[e] >= MAX_UNITS) { SET_ERROR(GL_INVALID_VALUE); return; }
        for (e = 0; e < n && el + e < u->size; e++) p->samp_unit[u->sampler + el + e] = iv[e];
        return;
    }
    ismat = u->stride > 1;
    if (kind == 'm' ? !ismat || u->stride != comps : ismat || u->comps != comps || (kind == 'f' && u->base == 1) || (kind == 'i' && u->base == 0)) { SET_ERROR(GL_INVALID_OPERATION); return; }
    per = kind == 'm' ? comps * comps : comps;
    for (e = 0; e < n && el + e < u->size; e++) {
        for (c = 0; c < per; c++) {
            size_t at = (size_t)(el + e) * u->stride * u->comps + (size_t)c;
            double d = kind == 'i' ? (double)iv[e * per + c] : (double)v[e * per + c];
            if (u->base == 2) d = d != 0.0;
            u->val[at] = (float)d;
            u->bits[at] = g2_fp24(d);
        }
    }
}

void gl_glsl_uniform_replay(const gl_listop *o) { uni_set((GLint)o->a, o->h, o->w >> 8, o->w & 255, (const float *)o->big, (const int *)o->big); }

#define UF(n, p, ...) void APIENTRY n { float x[] = { __VA_ARGS__ }; uni_set(l, 1, 'f', p, x, NULL); }
#define UI(n, p, ...) void APIENTRY n { int x[] = { __VA_ARGS__ }; uni_set(l, 1, 'i', p, NULL, x); }
UF(glUniform1f(GLint l, GLfloat a), 1, a)
UF(glUniform2f(GLint l, GLfloat a, GLfloat b), 2, a, b)
UF(glUniform3f(GLint l, GLfloat a, GLfloat b, GLfloat c), 3, a, b, c)
UF(glUniform4f(GLint l, GLfloat a, GLfloat b, GLfloat c, GLfloat d), 4, a, b, c, d)
UI(glUniform1i(GLint l, GLint a), 1, a)
UI(glUniform2i(GLint l, GLint a, GLint b), 2, a, b)
UI(glUniform3i(GLint l, GLint a, GLint b, GLint c), 3, a, b, c)
UI(glUniform4i(GLint l, GLint a, GLint b, GLint c, GLint d), 4, a, b, c, d)
void APIENTRY glUniform1fv(GLint l, GLsizei n, const GLfloat *v) { uni_set(l, n, 'f', 1, v, NULL); }
void APIENTRY glUniform2fv(GLint l, GLsizei n, const GLfloat *v) { uni_set(l, n, 'f', 2, v, NULL); }
void APIENTRY glUniform3fv(GLint l, GLsizei n, const GLfloat *v) { uni_set(l, n, 'f', 3, v, NULL); }
void APIENTRY glUniform4fv(GLint l, GLsizei n, const GLfloat *v) { uni_set(l, n, 'f', 4, v, NULL); }
void APIENTRY glUniform1iv(GLint l, GLsizei n, const GLint *v) { uni_set(l, n, 'i', 1, NULL, v); }
void APIENTRY glUniform2iv(GLint l, GLsizei n, const GLint *v) { uni_set(l, n, 'i', 2, NULL, v); }
void APIENTRY glUniform3iv(GLint l, GLsizei n, const GLint *v) { uni_set(l, n, 'i', 3, NULL, v); }
void APIENTRY glUniform4iv(GLint l, GLsizei n, const GLint *v) { uni_set(l, n, 'i', 4, NULL, v); }

static void uni_matrix(GLint l, GLsizei n, GLboolean tr, int c, const GLfloat *v)
{
    float *t = NULL;
    GLsizei e;
    int a, b;
    if (tr) {
        t = (float *)malloc((size_t)(n > 0 ? n : 1) * (size_t)(c * c) * sizeof(float));
        if (!t) return;
        for (e = 0; e < n; e++) for (a = 0; a < c; a++) for (b = 0; b < c; b++) t[e * c * c + a * c + b] = v[e * c * c + b * c + a];
        v = t;
    }
    uni_set(l, n, 'm', c, v, NULL);
    free(t);
}
void APIENTRY glUniformMatrix2fv(GLint l, GLsizei n, GLboolean t, const GLfloat *v) { uni_matrix(l, n, t, 2, v); }
void APIENTRY glUniformMatrix3fv(GLint l, GLsizei n, GLboolean t, const GLfloat *v) { uni_matrix(l, n, t, 3, v); }
void APIENTRY glUniformMatrix4fv(GLint l, GLsizei n, GLboolean t, const GLfloat *v) { uni_matrix(l, n, t, 4, v); }

static gs_uni *uni_get(GLuint n, GLint loc, int *el)
{
    gs_prog *p = pr_of(n);
    int i;
    if (!p) { SET_ERROR(GL_INVALID_VALUE); return NULL; }
    if (!p->linked) { SET_ERROR(GL_INVALID_OPERATION); return NULL; }
    i = loc >> 12; *el = loc & 4095;
    if (loc < 0 || i >= p->nuni || *el >= p->uni[i].size) { SET_ERROR(GL_INVALID_OPERATION); return NULL; }
    return &p->uni[i];
}

void APIENTRY glGetUniformfv(GLuint n, GLint loc, GLfloat *v)
{
    int el, c;
    gs_uni *u = uni_get(n, loc, &el);
    if (!u) return;
    if (u->sampler >= 0) { v[0] = (float)pr_of(n)->samp_unit[u->sampler + el]; return; }
    for (c = 0; c < u->stride * u->comps; c++) v[c] = u->val[(size_t)el * u->stride * u->comps + c];
}

void APIENTRY glGetUniformiv(GLuint n, GLint loc, GLint *v)
{
    int el, c;
    gs_uni *u = uni_get(n, loc, &el);
    if (!u) return;
    if (u->sampler >= 0) { v[0] = pr_of(n)->samp_unit[u->sampler + el]; return; }
    for (c = 0; c < u->stride * u->comps; c++) v[c] = (GLint)u->val[(size_t)el * u->stride * u->comps + c];
}

/* ---------------------------------------------------------------- generic attributes */
void gl_vertex(float x, float y, float z, float w);

void APIENTRY glVertexAttrib4f(GLuint i, GLfloat x, GLfloat y, GLfloat z, GLfloat w)
{
    if (i >= MAX_ATTRIBS) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (!i && (C->in_begin || C->compiling)) { gl_vertex(x, y, z, w); return; }   /* §2.7: generic 0 is the vertex: inside glBegin it makes one */
    LREC_F(LOP_ATTRIB, i, x, y, z, w);
    C->attr[i][0] = x; C->attr[i][1] = y; C->attr[i][2] = z; C->attr[i][3] = w;
}

#define NB(v) ((float)(((v) * 2.0 + 1.0) / 255.0))
#define NS(v) ((float)(((v) * 2.0 + 1.0) / 65535.0))
#define NI(v) ((float)(((v) * 2.0 + 1.0) / 4294967295.0))
#define UB(v) ((float)((v) / 255.0))
#define US(v) ((float)((v) / 65535.0))
#define UI_(v) ((float)((v) / 4294967295.0))
void APIENTRY glVertexAttrib1f(GLuint i, GLfloat x) { glVertexAttrib4f(i, x, 0, 0, 1); }
void APIENTRY glVertexAttrib2f(GLuint i, GLfloat x, GLfloat y) { glVertexAttrib4f(i, x, y, 0, 1); }
void APIENTRY glVertexAttrib3f(GLuint i, GLfloat x, GLfloat y, GLfloat z) { glVertexAttrib4f(i, x, y, z, 1); }
void APIENTRY glVertexAttrib1s(GLuint i, GLshort x) { glVertexAttrib4f(i, x, 0, 0, 1); }
void APIENTRY glVertexAttrib2s(GLuint i, GLshort x, GLshort y) { glVertexAttrib4f(i, x, y, 0, 1); }
void APIENTRY glVertexAttrib3s(GLuint i, GLshort x, GLshort y, GLshort z) { glVertexAttrib4f(i, x, y, z, 1); }
void APIENTRY glVertexAttrib4s(GLuint i, GLshort x, GLshort y, GLshort z, GLshort w) { glVertexAttrib4f(i, x, y, z, w); }
void APIENTRY glVertexAttrib1d(GLuint i, GLdouble x) { glVertexAttrib4f(i, (float)x, 0, 0, 1); }
void APIENTRY glVertexAttrib2d(GLuint i, GLdouble x, GLdouble y) { glVertexAttrib4f(i, (float)x, (float)y, 0, 1); }
void APIENTRY glVertexAttrib3d(GLuint i, GLdouble x, GLdouble y, GLdouble z) { glVertexAttrib4f(i, (float)x, (float)y, (float)z, 1); }
void APIENTRY glVertexAttrib4d(GLuint i, GLdouble x, GLdouble y, GLdouble z, GLdouble w) { glVertexAttrib4f(i, (float)x, (float)y, (float)z, (float)w); }
void APIENTRY glVertexAttrib1fv(GLuint i, const GLfloat *v) { glVertexAttrib4f(i, v[0], 0, 0, 1); }
void APIENTRY glVertexAttrib2fv(GLuint i, const GLfloat *v) { glVertexAttrib4f(i, v[0], v[1], 0, 1); }
void APIENTRY glVertexAttrib3fv(GLuint i, const GLfloat *v) { glVertexAttrib4f(i, v[0], v[1], v[2], 1); }
void APIENTRY glVertexAttrib4fv(GLuint i, const GLfloat *v) { glVertexAttrib4f(i, v[0], v[1], v[2], v[3]); }
void APIENTRY glVertexAttrib1sv(GLuint i, const GLshort *v) { glVertexAttrib4f(i, v[0], 0, 0, 1); }
void APIENTRY glVertexAttrib2sv(GLuint i, const GLshort *v) { glVertexAttrib4f(i, v[0], v[1], 0, 1); }
void APIENTRY glVertexAttrib3sv(GLuint i, const GLshort *v) { glVertexAttrib4f(i, v[0], v[1], v[2], 1); }
void APIENTRY glVertexAttrib4sv(GLuint i, const GLshort *v) { glVertexAttrib4f(i, v[0], v[1], v[2], v[3]); }
void APIENTRY glVertexAttrib1dv(GLuint i, const GLdouble *v) { glVertexAttrib4f(i, (float)v[0], 0, 0, 1); }
void APIENTRY glVertexAttrib2dv(GLuint i, const GLdouble *v) { glVertexAttrib4f(i, (float)v[0], (float)v[1], 0, 1); }
void APIENTRY glVertexAttrib3dv(GLuint i, const GLdouble *v) { glVertexAttrib4f(i, (float)v[0], (float)v[1], (float)v[2], 1); }
void APIENTRY glVertexAttrib4dv(GLuint i, const GLdouble *v) { glVertexAttrib4f(i, (float)v[0], (float)v[1], (float)v[2], (float)v[3]); }
void APIENTRY glVertexAttrib4bv(GLuint i, const GLbyte *v) { glVertexAttrib4f(i, v[0], v[1], v[2], v[3]); }
void APIENTRY glVertexAttrib4iv(GLuint i, const GLint *v) { glVertexAttrib4f(i, (float)v[0], (float)v[1], (float)v[2], (float)v[3]); }
void APIENTRY glVertexAttrib4ubv(GLuint i, const GLubyte *v) { glVertexAttrib4f(i, v[0], v[1], v[2], v[3]); }
void APIENTRY glVertexAttrib4usv(GLuint i, const GLushort *v) { glVertexAttrib4f(i, v[0], v[1], v[2], v[3]); }
void APIENTRY glVertexAttrib4uiv(GLuint i, const GLuint *v) { glVertexAttrib4f(i, (float)v[0], (float)v[1], (float)v[2], (float)v[3]); }
void APIENTRY glVertexAttrib4Nbv(GLuint i, const GLbyte *v) { glVertexAttrib4f(i, NB(v[0]), NB(v[1]), NB(v[2]), NB(v[3])); }
void APIENTRY glVertexAttrib4Nsv(GLuint i, const GLshort *v) { glVertexAttrib4f(i, NS(v[0]), NS(v[1]), NS(v[2]), NS(v[3])); }
void APIENTRY glVertexAttrib4Niv(GLuint i, const GLint *v) { glVertexAttrib4f(i, NI((double)v[0]), NI((double)v[1]), NI((double)v[2]), NI((double)v[3])); }
void APIENTRY glVertexAttrib4Nub(GLuint i, GLubyte x, GLubyte y, GLubyte z, GLubyte w) { glVertexAttrib4f(i, UB(x), UB(y), UB(z), UB(w)); }
void APIENTRY glVertexAttrib4Nubv(GLuint i, const GLubyte *v) { glVertexAttrib4f(i, UB(v[0]), UB(v[1]), UB(v[2]), UB(v[3])); }
void APIENTRY glVertexAttrib4Nusv(GLuint i, const GLushort *v) { glVertexAttrib4f(i, US(v[0]), US(v[1]), US(v[2]), US(v[3])); }
void APIENTRY glVertexAttrib4Nuiv(GLuint i, const GLuint *v) { glVertexAttrib4f(i, UI_((double)v[0]), UI_((double)v[1]), UI_((double)v[2]), UI_((double)v[3])); }

void APIENTRY glVertexAttribPointer(GLuint i, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void *p)
{
    if (i >= MAX_ATTRIBS || size < 1 || size > 4 || stride < 0) { SET_ERROR(GL_INVALID_VALUE); return; }
    gl_set_array(&C->a_attr[i], size, type, stride, p);
    C->a_attr[i].normalized = normalized;
}

void APIENTRY glEnableVertexAttribArray(GLuint i) { if (i >= MAX_ATTRIBS) { SET_ERROR(GL_INVALID_VALUE); return; } FLUSH_VERTICES(); C->a_attr[i].enabled = 1; }
void APIENTRY glDisableVertexAttribArray(GLuint i) { if (i >= MAX_ATTRIBS) { SET_ERROR(GL_INVALID_VALUE); return; } FLUSH_VERTICES(); C->a_attr[i].enabled = 0; }

void APIENTRY glGetVertexAttribfv(GLuint i, GLenum q, GLfloat *v)
{
    const gl_array *a;
    if (i >= MAX_ATTRIBS) { SET_ERROR(GL_INVALID_VALUE); return; }
    a = &C->a_attr[i];
    switch (q) {
    case GL_CURRENT_VERTEX_ATTRIB: if (!i) { SET_ERROR(GL_INVALID_OPERATION); return; } memcpy(v, C->attr[i], 16); break;
    case GL_VERTEX_ATTRIB_ARRAY_ENABLED: v[0] = (float)a->enabled; break;
    case GL_VERTEX_ATTRIB_ARRAY_SIZE: v[0] = (float)(a->size ? a->size : 4); break;
    case GL_VERTEX_ATTRIB_ARRAY_STRIDE: v[0] = (float)a->stride; break;
    case GL_VERTEX_ATTRIB_ARRAY_TYPE: v[0] = (float)(a->type ? a->type : GL_FLOAT); break;
    case GL_VERTEX_ATTRIB_ARRAY_NORMALIZED: v[0] = (float)a->normalized; break;
    case GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING: v[0] = (float)a->buffer; break;
    default: SET_ERROR(GL_INVALID_ENUM); break;
    }
}
void APIENTRY glGetVertexAttribiv(GLuint i, GLenum q, GLint *v)
{
    float f[4] = { 0, 0, 0, 0 };
    int c;
    glGetVertexAttribfv(i, q, f);
    for (c = 0; c < (q == GL_CURRENT_VERTEX_ATTRIB ? 4 : 1); c++) v[c] = (GLint)f[c];
}
void APIENTRY glGetVertexAttribdv(GLuint i, GLenum q, GLdouble *v)
{
    float f[4] = { 0, 0, 0, 0 };
    int c;
    glGetVertexAttribfv(i, q, f);
    for (c = 0; c < (q == GL_CURRENT_VERTEX_ATTRIB ? 4 : 1); c++) v[c] = f[c];
}
void APIENTRY glGetVertexAttribPointerv(GLuint i, GLenum q, void **v)
{
    if (i >= MAX_ATTRIBS) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (q != GL_VERTEX_ATTRIB_ARRAY_POINTER) { SET_ERROR(GL_INVALID_ENUM); return; }
    *v = (void *)C->a_attr[i].ptr;
}

/* §4.2.1: one colour output, so one draw buffer */
void APIENTRY glDrawBuffers(GLsizei n, const GLenum *b)
{
    if (n < 0 || n > 1) { SET_ERROR(GL_INVALID_VALUE); return; }
    if (n == 1) glDrawBuffer(b[0]);
}

/* ---------------------------------------------------------------- GL_ARB_shader_objects: the same objects under the older names */
void APIENTRY glDeleteObjectARB(GLhandleARB o) { if (sh_of(o)) glDeleteShader(o); else glDeleteProgram(o); }

void APIENTRY glGetObjectParameterivARB(GLhandleARB o, GLenum q, GLint *v)
{
    if (q == GL_OBJECT_TYPE_ARB) { *v = sh_of(o) ? GL_SHADER_OBJECT_ARB : GL_PROGRAM_OBJECT_ARB; return; }
    if (sh_of(o)) {
        gs_shader *s = sh_of(o);
        if (q == GL_OBJECT_SUBTYPE_ARB) *v = (GLint)s->type;
        else glGetShaderiv(o, q, v);
    } else glGetProgramiv(o, q, v);
}

void APIENTRY glGetObjectParameterfvARB(GLhandleARB o, GLenum q, GLfloat *v) { GLint i = 0; glGetObjectParameterivARB(o, q, &i); *v = (GLfloat)i; }

void APIENTRY glGetInfoLogARB(GLhandleARB o, GLsizei max, GLsizei *len, GLcharARB *log)
{
    if (sh_of(o)) glGetShaderInfoLog(o, max, len, log);
    else glGetProgramInfoLog(o, max, len, log);
}

GLhandleARB APIENTRY glGetHandleARB(GLenum q)
{
    if (q != GL_PROGRAM_OBJECT_ARB) { SET_ERROR(GL_INVALID_ENUM); return 0; }
    return C->program;
}

/* what the PC must know of a vertex a user's shader moves: gl_vsrun.c runs the program the board runs */
static int vs_run(const src *in, uint32_t idx, double *pos, double (*vary)[4], gs_variant **vo)
{
    static double par[256][4];
    static const gs_variant *pv;
    static uint32_t ps;
    double attr[16][4];
    ff_prog *ff;
    gs_variant *v;
    int a, i, c;
    if (!gl_glsl_user_vs()) return 0;
    ff = gl_glsl_program();
    if (!ff) return 0;
    v = ff->user;
    if (!v->sasm) return 0;
    if (!v->vm) v->vm = gl_vs_parse(v->sasm);
    if (pv != v || ps != C->vs_serial) {
        static uint32_t vu[4 * 256], fu[4 * 64];
        memset(vu, 0, sizeof vu); memset(fu, 0, sizeof fu);
        gl_ff_uniforms(ff, vu, fu);
        gl_glsl_uniforms(ff, vu, fu);
        for (i = 0; i < 256; i++) for (c = 0; c < 4; c++) par[i][c] = g2_fp24_to_double(vu[4 * i + c]);
        for (i = 0; i < v->ncpool && v->cbase + i < 256; i++) memcpy(par[v->cbase + i], v->cpool[i], sizeof par[0]);
        pv = v; ps = C->vs_serial;
    }
    memset(attr, 0, sizeof attr);
    for (i = 0; i < 16; i++) attr[i][3] = 1.0;
    for (a = 0; a < A_COUNT; a++) {
        int l = ff->attr_loc[a];
        float o[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
        if (l < 0 || l > 15) continue;
        if (in[a].base) { gl_src_get(&in[a], idx, o); for (c = 0; c < 4; c++) attr[l][c] = o[c]; }
        else for (c = 0; c < 4; c++) attr[l][c] = g2_fp24_to_double(in[a].constant[c]);
    }
    gl_vs_exec(v->vm, (const double (*)[4])attr, (const double (*)[4])par, pos, vary);
    *vo = v;
    return 1;
}

int gl_vs_clip(const src *in, uint32_t idx, float *c)
{
    double pos[4] = { 0, 0, 0, 1 };
    gs_variant *v;
    int i;
    if (!vs_run(in, idx, pos, NULL, &v)) return 0;
    for (i = 0; i < 4; i++) c[i] = (float)pos[i];
    return 1;
}

void gl_vs_flat(const src *in, uint32_t idx, float *o)
{
    double pos[4] = { 0, 0, 0, 1 }, vary[16][4];
    gs_variant *v;
    int a, c;
    memset(vary, 0, sizeof vary);
    memset(o, 0, 64);
    if (!vs_run(in, idx, pos, vary, &v)) return;
    for (a = 0; a < 4; a++) if (v->vary_flat[a] >= 0) for (c = 0; c < 4; c++) o[a * 4 + c] = (float)vary[v->vary_flat[a]][c];
}

int gl_vs_outputs(const src *in, uint32_t idx, float *clip, float *color, float *tc0, float *cd)
{
    double pos[4] = { 0, 0, 0, 1 }, vary[16][4];
    gs_variant *v;
    int i;
    memset(vary, 0, sizeof vary);
    if (!vs_run(in, idx, pos, vary, &v)) return 0;
    for (i = 0; i < 4; i++) {
        clip[i] = (float)pos[i];
        color[i] = v->vary_color >= 0 ? (float)vary[v->vary_color][i] : (i == 3 ? 1.0f : 0.0f);
        tc0[i] = v->vary_tc0 >= 0 ? (float)vary[v->vary_tc0][i] : (i == 3 ? 1.0f : 0.0f);
    }
    for (i = 0; i < MAX_CLIP; i++) cd[i] = v->vary_clip[i] >= 0 ? (float)vary[v->vary_clip[i]][0] : 1.0f;   /* §2.12: a plane against gl_ClipVertex, none when it is not written */
    return 1;
}
