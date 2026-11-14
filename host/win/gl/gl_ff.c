/* the fixed-function pipeline as generated GLSL ES programs: the program of a state key, the uniforms of a draw.
 * The equations are the OpenGL 1.x specification's: lighting §2.13.1 per vertex, texgen §2.10.4, texture environment table 3.12 (+ ARB_texture_env_combine), fog §3.10
 * per vertex (GL_FOG_HINT leaves that open), alpha test as `discard` (identical: both act before blending), user clip planes as a varying + `discard` (fragments dropped
 * instead of geometry clipped: the same coverage, interpolation unchanged). */
#include <stdarg.h>
#include "gl_internal.h"


typedef struct sb { char *p; size_t n, cap; } sb;

static void add(sb *s, const char *fmt, ...)
{
    va_list ap;
    int k;
    if (s->cap - s->n < 512) {
        s->cap *= 2;
        s->p = (char *)realloc(s->p, s->cap);
    }
    va_start(ap, fmt);
    k = vsnprintf(s->p + s->n, s->cap - s->n, fmt, ap);
    va_end(ap);
    if (k > 0) s->n += (size_t)k;
}

typedef gl_ureq ureq;
typedef struct gen { sb vs, fs; ureq vu[FF_MAX_UNI], fu[FF_MAX_UNI]; int nvu, nfu; } gen;

static void vu_(gen *g, const char *ty, const char *name, int kind, int idx)
{
    ureq *q = &g->vu[g->nvu++];
    snprintf(q->name, sizeof q->name, "%s", name);
    q->kind = (uint8_t)kind; q->idx = (uint8_t)idx;
    add(&g->vs, "uniform %s %s;\n", ty, name);
}

static void fu_(gen *g, const char *ty, const char *name, int kind, int idx)
{
    ureq *q = &g->fu[g->nfu++];
    snprintf(q->name, sizeof q->name, "%s", name);
    q->kind = (uint8_t)kind; q->idx = (uint8_t)idx;
    add(&g->fs, "uniform %s %s;\n", ty, name);
}

static void fua_(gen *g, const char *name, int n, int kind, int idx)
{
    ureq *q = &g->fu[g->nfu++];
    snprintf(q->name, sizeof q->name, "%s", name);
    q->kind = (uint8_t)kind; q->idx = (uint8_t)idx;
    add(&g->fs, "uniform vec4 %s[%d];\n", name, n);
}

static const char *const SWZ = "stpq";

static void gen_light(gen *g, const ff_key *k, int i, const char *mat, const char *n, const char *out, const char *spec)
{
    sb *v = &g->vs;
    int pos = (k->light_pos >> i) & 1, spot = (k->light_spot >> i) & 1;
    add(v, "  {\n");
    if (pos) {
        add(v, "    vec3 VP = uL%dPos.xyz - eye.xyz;\n    float d2 = dot(VP, VP);\n    float d = sqrt(d2);\n    VP = VP / d;\n", i);
        add(v, "    float att = 1.0 / (uL%dAtt.x + uL%dAtt.y * d + uL%dAtt.z * d2);\n", i, i, i);
        if (spot) {
            add(v, "    float cs = dot(-VP, uL%dDir.xyz);\n", i);
            add(v, "    if (cs < uL%dSpot.y) att = 0.0; else att = att * pow(max(cs, 0.0), uL%dSpot.x);\n", i, i);
        }
    } else {
        add(v, "    vec3 VP = uL%dPos.xyz;\n    float att = 1.0;\n", i);
    }
    add(v, "    float nl = max(dot(%s, VP), 0.0);\n", n);
    add(v, "    vec4 c = %sAmb * uL%dAmb + nl * %sDif * uL%dDif;\n", mat, i, mat, i);
    add(v, "    %s = %s + att * c.rgb;\n", out, out);
    add(v, "    if (nl > 0.0) {\n");
    if (k->local_viewer) add(v, "      vec3 h = normalize(VP - normalize(eye.xyz));\n");
    else add(v, "      vec3 h = normalize(VP + vec3(0.0, 0.0, 1.0));\n");
    add(v, "      float ns = max(dot(%s, h), 0.0);\n", n);
    add(v, "      if (ns > 0.0) %s = %s + att * pow(ns, %sShi.x) * %sSpc.rgb * uL%dSpc.rgb;\n", spec, spec, mat, mat, i);
    add(v, "    }\n  }\n");
}

static void gen_texcoord(gen *g, const ff_key *k, int u)
{
    sb *v = &g->vs;
    const char *tc = "aTc";
    int c, any = k->u[u].gen[0] | k->u[u].gen[1] | k->u[u].gen[2] | k->u[u].gen[3];
    if (any) {
        add(v, "  vec4 tc%d = aTc%d;\n", u, u);
        for (c = 0; c < 4; c++) {
            switch (k->u[u].gen[c]) {
            case 1: add(v, "  tc%d.%c = dot(aPos, uGen%c%d);\n", u, SWZ[c], "STRQ"[c], u); break;
            case 2: add(v, "  tc%d.%c = dot(eye, uGen%c%d);\n", u, SWZ[c], "STRQ"[c], u); break;
            case 3: add(v, "  tc%d.%c = sm.%c;\n", u, SWZ[c], "xyz"[c < 3 ? c : 2]); break;
            case 4: add(v, "  tc%d.%c = nrm.%c;\n", u, SWZ[c], "xyz"[c < 3 ? c : 2]); break;
            case 5: add(v, "  tc%d.%c = rfl.%c;\n", u, SWZ[c], "xyz"[c < 3 ? c : 2]); break;
            default: break;
            }
        }
    }
    (void)tc;
    if (k->u[u].texmat) add(v, "  g2vTc%d = uTm%d * %s%d;\n", u, u, any ? "tc" : "aTc", u);
    else add(v, "  g2vTc%d = %s%d;\n", u, any ? "tc" : "aTc", u);
}

static const char *comb_src(int src, int op, int unit, int alpha, char *buf)
{
    /* src: 0 texture, 1 constant, 2 primary, 3 previous, 4.. texture unit n */
    char base[16];
    if (src == 0) snprintf(base, sizeof base, "t%d", unit);
    else if (src == 1) snprintf(base, sizeof base, "uEnv%d", unit);
    else if (src == 2) snprintf(base, sizeof base, "g2vColor");
    else if (src == 3) snprintf(base, sizeof base, "c");
    else snprintf(base, sizeof base, "t%d", src - 4);
    if (alpha) {
        snprintf(buf, 32, op == 3 ? "(1.0 - %s.a)" : "%s.a", base);
    } else {
        switch (op) {
        case 0: snprintf(buf, 32, "%s.rgb", base); break;
        case 1: snprintf(buf, 32, "(vec3(1.0) - %s.rgb)", base); break;
        case 2: snprintf(buf, 32, "vec3(%s.a)", base); break;
        default: snprintf(buf, 32, "vec3(1.0 - %s.a)", base); break;
        }
    }
    return buf;
}

/* a unit that leaves alpha as it is (table 3.12: RGB, and DECAL of RGBA) */
static int rgb_only(const ff_key *k, int u)
{
    return k->u[u].env != 5 && (k->u[u].base == BF_RGB || (k->u[u].base == BF_RGBA && k->u[u].env == 1));
}

/* full: c.a is kept aside (generate), so the unit writes all four components, each by the same expression as its c.rgb: the compiler moves a partial write through a register, 7 micro-ops more */
static void gen_env(gen *g, const ff_key *k, int u, int full)
{
    sb *f = &g->fs;
    int base = k->u[u].base, e = k->u[u].env;
    char a[32], b[32], d[32];
    add(f, "  {\n");
    if (full) {
        if (base == BF_RGBA) add(f, "    c = c * (1.0 - t%d.a) + t%d * t%d.a;\n", u, u, u);           /* DECAL */
        else switch (e) {
        case 1: case 2: add(f, "    c = t%d;\n", u); break;
        case 3: add(f, "    c = c * (vec4(1.0) - t%d) + uEnv%d * t%d;\n", u, u, u); break;
        case 4: add(f, "    c = clamp(c + t%d, 0.0, 1.0);\n", u); break;
        default: add(f, "    c = c * t%d;\n", u); break;
        }
        add(f, "  }\n");
        return;
    }
    if (e == 5) {
        const int r = k->u[u].comb_rgb, al = k->u[u].comb_a;
        comb_src(k->u[u].src_rgb[0], k->u[u].op_rgb[0], u, 0, a);
        comb_src(k->u[u].src_rgb[1], k->u[u].op_rgb[1], u, 0, b);
        comb_src(k->u[u].src_rgb[2], k->u[u].op_rgb[2], u, 0, d);
        switch (r) {
        case 0: add(f, "    vec3 cr = %s;\n", a); break;
        case 1: add(f, "    vec3 cr = %s * %s;\n", a, b); break;
        case 2: add(f, "    vec3 cr = %s + %s;\n", a, b); break;
        case 3: add(f, "    vec3 cr = %s + %s - vec3(0.5);\n", a, b); break;
        case 4: add(f, "    vec3 cr = %s * %s + %s * (vec3(1.0) - %s);\n", a, d, b, d); break;
        case 5: add(f, "    vec3 cr = %s - %s;\n", a, b); break;
        default: add(f, "    vec3 cr = vec3(4.0 * dot(%s - vec3(0.5), %s - vec3(0.5)));\n", a, b); break;
        }
        comb_src(k->u[u].src_a[0], k->u[u].op_a[0], u, 1, a);
        comb_src(k->u[u].src_a[1], k->u[u].op_a[1], u, 1, b);
        comb_src(k->u[u].src_a[2], k->u[u].op_a[2], u, 1, d);
        switch (al) {
        case 0: add(f, "    float ca = %s;\n", a); break;
        case 1: add(f, "    float ca = %s * %s;\n", a, b); break;
        case 2: add(f, "    float ca = %s + %s;\n", a, b); break;
        case 3: add(f, "    float ca = %s + %s - 0.5;\n", a, b); break;
        case 4: add(f, "    float ca = %s * %s + %s * (1.0 - %s);\n", a, d, b, d); break;
        default: add(f, "    float ca = %s - %s;\n", a, b); break;
        }
        if (r == 6 || r == 7) {                                 /* DOT3_RGBA: the result goes to alpha too */
            if (r == 7) add(f, "    ca = cr.x;\n");
        }
        add(f, "    c = clamp(vec4(cr * %d.0, ca * %d.0), 0.0, 1.0);\n", k->u[u].scale_rgb, k->u[u].scale_a);
        add(f, "  }\n");
        return;
    }
    /* table 3.12: Cp = c, Cs = t, Cc = uEnv */
    switch (base) {
    case BF_ALPHA:
        if (e == 2) add(f, "    c.a = t%d.a;\n", u);
        else add(f, "    c.a = c.a * t%d.a;\n", u);
        break;
    case BF_INTENSITY:
        switch (e) {
        case 2: add(f, "    c = t%d;\n", u); break;
        case 3: add(f, "    c = c * (vec4(1.0) - t%d) + uEnv%d * t%d;\n", u, u, u); break;
        case 4: add(f, "    c = clamp(c + t%d, 0.0, 1.0);\n", u); break;
        default: add(f, "    c = c * t%d;\n", u); break;
        }
        break;
    case BF_RGB:
        switch (e) {
        case 1: case 2: add(f, "    c.rgb = t%d.rgb;\n", u); break;
        case 3: add(f, "    c.rgb = c.rgb * (vec3(1.0) - t%d.rgb) + uEnv%d.rgb * t%d.rgb;\n", u, u, u); break;
        case 4: add(f, "    c.rgb = clamp(c.rgb + t%d.rgb, 0.0, 1.0);\n", u); break;
        default: add(f, "    c.rgb = c.rgb * t%d.rgb;\n", u); break;
        }
        break;
    default:                                                    /* BF_RGBA */
        switch (e) {
        case 1: add(f, "    c.rgb = c.rgb * (1.0 - t%d.a) + t%d.rgb * t%d.a;\n", u, u, u); break;
        case 2: add(f, "    c = t%d;\n", u); break;
        case 3:
            add(f, "    c.rgb = c.rgb * (vec3(1.0) - t%d.rgb) + uEnv%d.rgb * t%d.rgb;\n    c.a = c.a * t%d.a;\n", u, u, u, u);
            break;
        case 4: add(f, "    c.rgb = clamp(c.rgb + t%d.rgb, 0.0, 1.0);\n    c.a = c.a * t%d.a;\n", u, u); break;
        default: add(f, "    c = c * t%d;\n", u); break;
        }
        break;
    }
    add(f, "  }\n");
}

/* a texture coordinate, wrapped by the program */
static void wrap_text(sb *f, const char *c, const char *n, int mode)
{
    switch (mode) {
    case 0: add(f, "    %s = fract(%s);\n", c, c); break;
    case 1: add(f, "    %s = 1.0 - abs(mod(%s, 2.0) - 1.0);\n", c, c); break;
    case 2: add(f, "    %s = clamp(%s, 0.0, 1.0);\n", c, c); break;
    case 3: add(f, "    %s = clamp(%s, 0.5 / %s, 1.0 - 0.5 / %s);\n", c, c, n, n); break;
    default: add(f, "    %s = clamp(%s, -0.5 / %s, 1.0 + 0.5 / %s);\n", c, c, n, n); break;
    }
}

/* how far a coordinate is past the middle of the outer texel, in texels, as cover of what is inside: the TMU has clamped to the edge, so what it read is the inside of the filter */
static void border_text(sb *f, const char *c, const char *n, int mode)
{
    add(f, "    {\n      float o = max(0.5 - %s * %s, %s * %s - %s + 0.5);\n", c, n, c, n, n);
    if (mode == 1) add(f, "      inside = inside * (1.0 - clamp(o, 0.0, 0.5));\n");    /* GL_CLAMP: s stops at 0 and 1, half a texel out */
    else if (mode == 2) add(f, "      inside = inside * (1.0 - clamp(o, 0.0, 1.0));\n");
    else add(f, "      if (o > 0.5) inside = 0.0;\n");
    add(f, "    }\n");
}

/* the last argument of a fetch: nothing, the bias, or the level gen_lod worked out */
static void lod_arg(char *e, size_t n, const ff_key *k, int u)
{
    if (k->u[u].lod == 1) snprintf(e, n, ", uLod%d.x", u);
    else if (k->u[u].lod == 2) snprintf(e, n, ", lod%d", u);
    else e[0] = 0;
}
static const char *tex_fn(const ff_key *k, int u) { return k->u[u].lod == 2 ? "texture2DLod" : "texture2D"; }

/* §3.8.8: the level, with GL_TEXTURE_MIN_LOD and GL_TEXTURE_MAX_LOD around it: the TMU works out its own level and clamps it to the chain and nothing else. uLod: bias, min, max */
static void gen_lod(gen *g, const ff_key *k, int u)
{
    sb *f = &g->fs;
    if (k->u[u].target == 4) {
        add(f, k->u[u].proj ? "  vec3 lc%d = g2vTc%d.xyz / g2vTc%d.w;\n" : "  vec3 lc%d = g2vTc%d.xyz;\n", u, u, u);
        add(f, "  float lod%d;\n  {\n    vec3 dx = dFdx(lc%d) * uTsz%d.xyz;\n    vec3 dy = dFdy(lc%d) * uTsz%d.xyz;\n", u, u, u, u, u);
    } else if (k->u[u].target == 2) {                           /* on a face, the coordinate over its major axis is what moves */
        add(f, "  float lod%d;\n  {\n    vec3 a = abs(g2vTc%d.xyz);\n    vec3 p = g2vTc%d.xyz / max(a.x, max(a.y, a.z));\n    vec3 dx = dFdx(p) * uTsz%d.x * 0.5;\n    vec3 dy = dFdy(p) * uTsz%d.x * 0.5;\n", u, u, u, u, u);
    } else {
        if (k->u[u].target == 3) add(f, k->u[u].proj ? "  vec2 lc%d = vec2(g2vTc%d.x / g2vTc%d.w, 0.0);\n" : "  vec2 lc%d = vec2(g2vTc%d.x, 0.0);\n", u, u, u);
        else add(f, k->u[u].proj ? "  vec2 lc%d = g2vTc%d.xy / g2vTc%d.w;\n" : "  vec2 lc%d = g2vTc%d.xy;\n", u, u, u);
        add(f, "  float lod%d;\n  {\n    vec2 dx = dFdx(lc%d) * uTsz%d.xy;\n    vec2 dy = dFdy(lc%d) * uTsz%d.xy;\n", u, u, u, u, u);
    }
    add(f, "    lod%d = clamp(0.5 * log2(max(max(dot(dx, dx), dot(dy, dy)), 1.0e-12)) + uLod%d.x, uLod%d.y, uLod%d.z);\n  }\n", u, u, u, u);
}

/* the texture fetch of a 2D or 1D unit that needs a border the hardware has not got */
static void gen_fetch(gen *g, const ff_key *k, int u)
{
    sb *f = &g->fs;
    int oned = k->u[u].target == 3;
    char c[16], n[16], e[40];
    lod_arg(e, sizeof e, k, u);
    if (oned) add(f, k->u[u].proj ? "  vec2 st%d = vec2(g2vTc%d.x / g2vTc%d.w, 0.5);\n" : "  vec2 st%d = vec2(g2vTc%d.x, 0.5);\n", u, u, u);
    else add(f, k->u[u].proj ? "  vec2 st%d = g2vTc%d.xy / g2vTc%d.w;\n" : "  vec2 st%d = g2vTc%d.xy;\n", u, u, u);
    if (k->u[u].bimg) {                                         /* the image has its border: the inside starts a texel in */
        add(f, "  {\n");
        snprintf(c, sizeof c, "st%d.x", u); snprintf(n, sizeof n, "uTsz%d.x", u);
        wrap_text(f, c, n, k->u[u].ws);
        add(f, "    %s = (%s * %s + 1.0) / (%s + 2.0);\n", c, c, n, n);
        if (!oned) {
            snprintf(c, sizeof c, "st%d.y", u); snprintf(n, sizeof n, "uTsz%d.y", u);
            wrap_text(f, c, n, k->u[u].wt);
            add(f, "    %s = (%s * %s + 1.0) / (%s + 2.0);\n", c, c, n, n);
        }
        add(f, "  }\n  vec4 t%d = %s(g2Tex%d, st%d%s);\n", u, tex_fn(k, u), u, u, e);
        return;
    }
    add(f, "  vec4 t%d = %s(g2Tex%d, st%d%s);\n  {\n    vec2 n = uTsz%d.xy;\n    float inside = 1.0;\n", u, tex_fn(k, u), u, u, e, u);
    if (k->u[u].blod) add(f, "    vec2 dx = dFdx(st%d) * n;\n    vec2 dy = dFdy(st%d) * n;\n    n = n * exp2(-max(0.5 * log2(max(max(dot(dx, dx), dot(dy, dy)), 1.0e-6)), 0.0));\n", u, u);    /* a mip level's texels */
    snprintf(c, sizeof c, "st%d.x", u);
    if (k->u[u].bs) border_text(f, c, "n.x", k->u[u].bs);
    snprintf(c, sizeof c, "st%d.y", u);
    if (k->u[u].bt) border_text(f, c, "n.y", k->u[u].bt);
    add(f, "    t%d = mix(uBrd%d, t%d, inside);\n  }\n", u, u, u);
}

/* ---- what the TMU cannot filter. The slices of a 3D texture with border texels, or on a B before 69, lie side by side in one image (sw 3) and a depth texel is 24 bits in the bytes of an RGBA8 one (sw 2), so the TMU
 * gives the texel the program asks for and the rest of §3.8.8 is here, the wrap of each axis and the filter within and between levels. A user's program comes here too for what the TMU has not got on a colour texture
 * (sw 1) or a volume (sw 4): the border, MIN_LOD and MAX_LOD. Entries of the level table: x, y, slices a row of the level in the image (else: the level), its size against the base level */
static int sw_lam(const ff_key *k, int u) { return k->u[u].swmin >= 2 || k->u[u].swmag != (k->u[u].swmin & 1); }

static void gen_sw_funcs(gen *g, const ff_key *k, int u)
{
    sb *f = &g->fs;
    int sw = k->u[u].sw, dim = sw >= 3 ? 3 : 2, B = k->u[u].swb, a, bord = 0;
    const uint8_t *wr = k->u[u].sww;
    const char *lo = B ? "-1.0" : "0.0", *hi = B ? "0.0" : "1.0", *off = B ? "1.5" : "0.5", *pad = B ? "2.0" : "0.0";
    char lvl[32], nm[16];
    static const char *const CMP[9] = { "", "false", "i.z < z", "i.z == z", "i.z <= z", "i.z > z", "i.z != z", "i.z >= z", "true" };
    for (a = 0; a < dim; a++) if ((wr[a] == 2 || wr[a] == 4) && !B) bord = 1;
    if (sw == 3) {
        snprintf(nm, sizeof nm, "g2V%d", u);
        fua_(g, nm, k->u[u].swn, U_F_SWL, u);
    } else {                                                    /* the same for every texture: one table, or four of them run out of uniforms */
        int v, first = 1, most = 1;
        for (v = 0; v < MAX_UNITS; v++) {
            if (k->u[v].sw != 1 && k->u[v].sw != 2 && k->u[v].sw != 4) continue;
            if (v < u) first = 0;
            if (k->u[v].swn > most) most = k->u[v].swn;
        }
        snprintf(nm, sizeof nm, "g2Vd");
        if (first) fua_(g, nm, most, U_F_SWD, most);
    }
    snprintf(nm, sizeof nm, "g2Sz%d", u); fu_(g, "vec4", nm, U_F_SWS, u);
    if (sw == 3) { snprintf(nm, sizeof nm, "g2At%d", u); fu_(g, "vec4", nm, U_F_SWA, u); }
    if (bord) { snprintf(nm, sizeof nm, "g2Brd%d", u); fu_(g, "vec4", nm, U_F_BRD, u); }
    if (sw_lam(k, u)) { snprintf(nm, sizeof nm, "g2Lod%d", u); fu_(g, "vec4", nm, U_F_LOD, u); }
    if (sw != 3) snprintf(lvl, sizeof lvl, k->u[u].swn > 1 ? "g2Vd[int(l)]" : "g2Vd[0]");
    else if (k->u[u].swn > 1) snprintf(lvl, sizeof lvl, "g2V%d[int(l)]", u);
    else snprintf(lvl, sizeof lvl, "g2V%d[0]", u);

    add(f, "vec4 g2tap%d(vec3 i, vec4 o, vec3 n) {\n", u);
    if (sw == 3) {
        add(f, "  vec3 q = clamp(i, vec3(%s), n - vec3(%s));\n  float k = q.z + %s;\n  float r = floor((k + 0.5) / o.z);\n", lo, hi, B ? "1.0" : "0.0");
        add(f, "  vec4 c = texture2DLod(g2Tex%d, (o.xy + vec2(k - r * o.z, r) * (n.xy + vec2(%s)) + q.xy + vec2(%s)) * g2At%d.xy, 0.0);\n", u, pad, off, u);
        if (bord) add(f, "  if (q.x != i.x || q.y != i.y || q.z != i.z) c = g2Brd%d;\n", u);
        add(f, "  return c;\n}\n");
    } else if (sw == 4) {
        add(f, "  vec3 q = clamp(i, vec3(0.0), n - vec3(1.0));\n  vec4 c = textureCubeLod(g2Tex%d, (q + vec3(0.5)) / n, o.x);\n", u);
        if (bord) add(f, "  if (q.x != i.x || q.y != i.y || q.z != i.z) c = g2Brd%d;\n", u);
        add(f, "  return c;\n}\n");
    } else if (sw == 1) {
        add(f, "  vec2 q = clamp(i.xy, vec2(%s), n.xy - vec2(%s));\n  vec4 c = texture2DLod(g2Tex%d, (q + vec2(%s)) / (n.xy + vec2(%s)), o.x);\n", lo, hi, u, off, pad);
        if (bord) add(f, "  if (q.x != i.x || q.y != i.y) c = g2Brd%d;\n", u);
        add(f, "  return c;\n}\n");
    } else {
        /* {stencil, depth}: R the stencil, G the high byte. All ones must come to 1 whatever the additions round */
        add(f, "  vec2 q = clamp(i.xy, vec2(%s), n.xy - vec2(%s));\n", lo, hi);
        add(f, "  float z = min(dot(texture2DLod(g2Tex%d, (q + vec2(%s)) / (n.xy + vec2(%s)), o.x).gba, vec3(0.99609375, 0.003891, 0.000016)), 1.0);\n", u, off, pad);
        if (bord) add(f, "  if (q.x != i.x || q.y != i.y) z = g2Brd%d.x;\n", u);
        if (k->u[u].dcmp) add(f, "  z = (%s) ? 1.0 : 0.0;\n", CMP[k->u[u].dcmp]);
        add(f, "  return vec4(z);\n}\n");
    }

    add(f, "vec3 g2wr%d(vec3 st, vec3 n) {\n", u);
    for (a = 0; a < dim; a++) {
        char c[8], m[8];
        snprintf(c, sizeof c, "st.%c", "xyz"[a]); snprintf(m, sizeof m, "n.%c", "xyz"[a]);
        wrap_text(f, c, m, wr[a]);
        if (wr[a] == 1) add(f, "    %s = clamp(%s, 0.5 / %s, 1.0 - 0.5 / %s);\n", c, c, m, m);     /* mirrored, then as CLAMP_TO_EDGE */
    }
    add(f, "  return st;\n}\n");

    add(f, "vec4 g2near%d(vec3 st, float l) {\n  vec4 o = %s;\n  vec3 n = max(floor(g2Sz%d.xyz * o.w), vec3(1.0));\n  vec3 i = floor(g2wr%d(st, n) * n);\n", u, lvl, u, u);
    for (a = 0; a < dim; a++) if (wr[a] != 4) add(f, "  i.%c = min(i.%c, n.%c - 1.0);\n", "xyz"[a], "xyz"[a], "xyz"[a]);
    if (dim == 2) add(f, "  i.z = clamp(st.z, 0.0, 1.0);\n");
    add(f, "  return g2tap%d(i, o, n);\n}\n", u);

    add(f, "vec4 g2lin%d(vec3 st, float l) {\n  vec4 o = %s;\n  vec3 n = max(floor(g2Sz%d.xyz * o.w), vec3(1.0));\n  vec3 p = g2wr%d(st, n) * n - vec3(0.5);\n  vec3 i0 = floor(p);\n  vec3 a = p - i0;\n  vec3 i1 = i0 + vec3(1.0);\n", u, lvl, u, u);
    for (a = 0; a < dim; a++) {
        char c = "xyz"[a];
        if (wr[a] == 0) add(f, "  if (i0.%c < 0.0) i0.%c = n.%c - 1.0;\n  if (i1.%c >= n.%c) i1.%c = 0.0;\n", c, c, c, c, c, c);
        else if (wr[a] == 1 || wr[a] == 3) add(f, "  i0.%c = max(i0.%c, 0.0);\n  i1.%c = min(i1.%c, n.%c - 1.0);\n", c, c, c, c, c);
    }
    if (dim == 2) add(f, "  i0.z = clamp(st.z, 0.0, 1.0);\n  i1.z = i0.z;\n");
    add(f, "  return %smix(mix(g2tap%d(i0, o, n), g2tap%d(vec3(i1.x, i0.y, i0.z), o, n), a.x), mix(g2tap%d(vec3(i0.x, i1.y, i0.z), o, n), g2tap%d(vec3(i1.x, i1.y, i0.z), o, n), a.x), a.y)", dim == 3 ? "mix(" : "", u, u, u, u);
    if (dim == 3) add(f, ", mix(mix(g2tap%d(vec3(i0.x, i0.y, i1.z), o, n), g2tap%d(vec3(i1.x, i0.y, i1.z), o, n), a.x), mix(g2tap%d(vec3(i0.x, i1.y, i1.z), o, n), g2tap%d(i1, o, n), a.x), a.y), a.z)", u, u, u, u);
    add(f, ";\n}\n");
}

/* §3.8.8: λ, then magnification or minification, then the level or the two levels. st is there; bias: a call's own; lod: the level a call gives, for λ base */
static void gen_sw_body(sb *f, const ff_key *k, int u, const char *bias, const char *lod)
{
    int dim = k->u[u].sw >= 3 ? 3 : 2, n = k->u[u].swn, mn = k->u[u].swmin, mg = k->u[u].swmag;
    const char *mf = mg ? "g2lin" : "g2near", *nf = mn & 1 ? "g2lin" : "g2near";
    if (k->u[u].target == 3) add(f, "    st.y = 0.5;\n");
    if (sw_lam(k, u)) {
        if (lod) add(f, "    float lam = clamp(%s + g2Lod%d.x, g2Lod%d.y, g2Lod%d.z);\n", lod, u, u, u);
        else {
            if (dim == 3) add(f, "    vec3 dx = dFdx(st) * g2Sz%d.xyz;\n    vec3 dy = dFdy(st) * g2Sz%d.xyz;\n", u, u);
            else add(f, "    vec2 dx = dFdx(st.xy) * g2Sz%d.xy;\n    vec2 dy = dFdy(st.xy) * g2Sz%d.xy;\n", u, u);
            add(f, "    float lam = clamp(0.5 * log2(max(max(dot(dx, dx), dot(dy, dy)), 1.0e-12)) + g2Lod%d.x%s%s, g2Lod%d.y, g2Lod%d.z);\n", u, bias ? " + " : "", bias ? bias : "", u, u);
        }
        add(f, "    if (lam <= %s) t%d = %s%d(st, 0.0);\n    else ", mg && (mn == 2 || mn == 4) ? "0.5" : "0.0", u, mf, u);
        if (mn < 2) add(f, "t%d = %s%d(st, 0.0);\n", u, nf, u);
        else if (mn < 4) add(f, "t%d = %s%d(st, min(ceil(lam + 0.5) - 1.0, %d.0));\n", u, nf, u, n - 1);
        else {
            add(f, "{\n      float d1 = min(floor(lam), %d.0);\n      float d2 = min(d1 + 1.0, %d.0);\n", n - 1, n - 1);
            add(f, "      t%d = mix(%s%d(st, d1), %s%d(st, d2), lam - floor(lam));\n    }\n", u, nf, u, nf, u);
        }
    } else add(f, "    t%d = %s%d(st, 0.0);\n", u, nf, u);
    if (k->u[u].sw == 2 && k->u[u].dmode == 1) add(f, "    t%d.a = 1.0;\n", u);
    else if (k->u[u].sw == 2 && k->u[u].dmode == 3) add(f, "    t%d = vec4(0.0, 0.0, 0.0, t%d.x);\n", u, u);
}

static void gen_sw(gen *g, const ff_key *k, int u)
{
    sb *f = &g->fs;
    add(f, "  vec4 t%d;\n  {\n    vec3 st = g2vTc%d.xyz", u, u);
    if (k->u[u].proj) add(f, " / g2vTc%d.w", u);
    add(f, ";\n");
    gen_sw_body(f, k, u, NULL, NULL);
    add(f, "  }\n");
}

static void generate(gen *g, const ff_key *k)
{
    sb *v = &g->vs, *f = &g->fs;
    char nm[32];
    int u, i, needs_eye, needs_n = k->light, any_tex = 0;
    int need_sm = 0, need_rfl = 0, need_nrm = 0;
    for (u = 0; u < MAX_UNITS; u++) {
        if (!k->u[u].target) continue;
        any_tex = 1;
        for (i = 0; i < 4; i++) {
            if (k->u[u].gen[i] == 3) need_sm = 1;
            if (k->u[u].gen[i] == 4) need_nrm = 1;
            if (k->u[u].gen[i] == 5) need_rfl = 1;
        }
    }
    needs_n = needs_n || need_sm || need_nrm || need_rfl;
    needs_eye = k->light || (k->fog && !k->fogc) || k->clip_mask || need_sm || need_rfl || k->pt_att || k->fogfrag == 1;
    for (u = 0; u < MAX_UNITS && !needs_eye; u++) for (i = 0; i < 4; i++) if (k->u[u].gen[i] == 2) needs_eye = 1;

    add(v, "attribute vec4 aPos;\nattribute vec4 aColor;\n");
    if (needs_n) add(v, "attribute vec3 aNormal;\n");
    for (u = 0; u < MAX_UNITS; u++) if (k->u[u].target) add(v, "attribute vec4 aTc%d;\n", u);
    if (k->line_stip || k->smooth_ln) add(v, "attribute vec4 aStip;\n");
    if (k->smooth_ln) add(v, "attribute vec4 aLine;\n");
    if ((k->fog && k->fogc) || k->fogfrag == 2) add(v, "attribute vec4 aFogC;\n");
    if (k->fogfrag) { add(v, "varying vec4 g2vFogF;\n"); add(f, "varying vec4 g2vFogF;\n"); }
    if (k->sec) add(v, "attribute vec4 aSec;\n");
    vu_(g, "mat4", "uMVP", U_MVP, 0);
    if (needs_eye) vu_(g, "mat4", "uMV", U_MV, 0);
    if (needs_n) vu_(g, "mat4", "uNM", U_NM, 0);
    add(v, "varying vec4 g2vColor;\n");
    add(f, "precision mediump float;\nvarying vec4 g2vColor;\n");
    if (k->light && k->two_side) { add(v, "varying vec4 g2vColorB;\n"); add(f, "varying vec4 g2vColorB;\n"); }
    if ((k->sep_spec && k->light) || k->sec || k->need_spec) { add(v, "varying vec4 g2vSpec;\n"); add(f, "varying vec4 g2vSpec;\n"); }
    if (k->line_stip) { add(v, "varying vec4 g2vStip;\n"); add(f, "varying vec4 g2vStip;\n"); fu_(g, "vec4", "uStipF", U_F_STIP, 0); }
    if (k->smooth_ln) { add(v, "varying vec4 g2vLine;\n"); add(f, "varying vec4 g2vLine;\n"); fu_(g, "vec4", "uLineF", U_F_LINE, 0); }
    if (k->smooth_pt) fu_(g, "vec4", "uPointF", U_F_POINT, 0);
    if (k->zoff) vu_(g, "vec4", "uZOff", U_Z, 0);
    if (k->line_stip || k->poly_stip) add(f, "uniform sampler2D uStip;\n");
    if (k->fog) { add(v, "varying vec4 g2vFog;\n"); add(f, "varying vec4 g2vFog;\n"); vu_(g, "vec4", "uFog", U_FOG, 0); fu_(g, "vec4", "uFogC", U_F_FOGC, 0); }
    for (u = 0; u < MAX_UNITS; u++) {
        if (!k->u[u].target) continue;
        add(v, "varying vec4 g2vTc%d;\n", u);
        add(f, "varying vec4 g2vTc%d;\n", u);
        if (k->u[u].texmat) { snprintf(nm, sizeof nm, "uTm%d", u); vu_(g, "mat4", nm, U_TM, u); }
        for (i = 0; i < 4; i++) {
            if (k->u[u].gen[i] == 1 || k->u[u].gen[i] == 2) {
                snprintf(nm, sizeof nm, "uGen%c%d", "STRQ"[i], u);
                vu_(g, "vec4", nm, U_GEN_S + i, u);
            }
        }
        add(f, "uniform %s g2Tex%d;\n", k->u[u].target == 2 || (k->u[u].target == 4 && k->u[u].sw != 3) ? "samplerCube" : "sampler2D", u);        /* a volume reads like a cube map: s, t, r, and the image says which */
        if (k->u[u].env == 3 || k->u[u].env == 5) { snprintf(nm, sizeof nm, "uEnv%d", u); fu_(g, "vec4", nm, U_F_ENV, u); }
        if (k->u[u].sw) { gen_sw_funcs(g, k, u); continue; }
        if (k->u[u].bimg || k->u[u].bs || k->u[u].bt) { snprintf(nm, sizeof nm, "uTsz%d", u); fu_(g, "vec4", nm, U_F_TSZ, u); }
        if (k->u[u].bs || k->u[u].bt) { snprintf(nm, sizeof nm, "uBrd%d", u); fu_(g, "vec4", nm, U_F_BRD, u); }
        if (k->u[u].lod) { snprintf(nm, sizeof nm, "uLod%d", u); fu_(g, "vec4", nm, U_F_LOD, u); }
        if (k->u[u].lod == 2 && !(k->u[u].bimg || k->u[u].bs || k->u[u].bt)) { snprintf(nm, sizeof nm, "uTsz%d", u); fu_(g, "vec4", nm, U_F_TSZ, u); }
    }
    for (i = 0; i < MAX_CLIP; i++) {
        if ((k->clip_mask >> i) & 1) {
            add(v, "varying vec4 g2vClip%d;\n", i);
            add(f, "varying vec4 g2vClip%d;\n", i);
            snprintf(nm, sizeof nm, "uClip%d", i);
            vu_(g, "vec4", nm, U_CLIP, i);
        }
    }
    if (k->alpha_fn != 7) fu_(g, "float", "uAlpha", U_F_ALPHA, 0);     /* x of its slot: a vec4 would load four components */
    if (k->light) {
        vu_(g, "vec4", "uScene", U_COLOR_SCENE, 0);
        vu_(g, "vec4", "uMEmi", U_M_EMI, 0); vu_(g, "vec4", "uMAmb", U_M_AMB, 0); vu_(g, "vec4", "uMDif", U_M_DIF, 0); vu_(g, "vec4", "uMSpc", U_M_SPC, 0);
        vu_(g, "vec4", "uMShi", U_M_SHI, 0);
        if (k->two_side) {
            vu_(g, "vec4", "uBEmi", U_BM_EMI, 0); vu_(g, "vec4", "uBAmb", U_BM_AMB, 0); vu_(g, "vec4", "uBDif", U_BM_DIF, 0); vu_(g, "vec4", "uBSpc", U_BM_SPC, 0);
            vu_(g, "vec4", "uBShi", U_BM_SHI, 0);
        }
        for (i = 0; i < MAX_LIGHTS; i++) {
            if (!((k->light_on >> i) & 1)) continue;
            snprintf(nm, sizeof nm, "uL%dPos", i); vu_(g, "vec4", nm, U_L_POS, i);
            snprintf(nm, sizeof nm, "uL%dAmb", i); vu_(g, "vec4", nm, U_L_AMB, i);
            snprintf(nm, sizeof nm, "uL%dDif", i); vu_(g, "vec4", nm, U_L_DIF, i);
            snprintf(nm, sizeof nm, "uL%dSpc", i); vu_(g, "vec4", nm, U_L_SPC, i);
            if ((k->light_pos >> i) & 1) {
                snprintf(nm, sizeof nm, "uL%dAtt", i); vu_(g, "vec4", nm, U_L_ATT, i);
                if ((k->light_spot >> i) & 1) {
                    snprintf(nm, sizeof nm, "uL%dDir", i); vu_(g, "vec4", nm, U_L_DIR, i);
                    snprintf(nm, sizeof nm, "uL%dSpot", i); vu_(g, "vec4", nm, U_L_SPOT, i);
                }
            }
        }
    }

    /* ---- vertex program ---- */
    if (k->points) vu_(g, "vec4", "uPoint", U_POINT, 0);
    if (k->points && k->pt_att) vu_(g, "vec4", "uPointA", U_POINTA, 0);
    add(v, "void main() {\n  gl_Position = uMVP * aPos;\n");
    if (k->smooth_ln) {                                         /* in normalized x and y, so a corner stays in the plane of the line under any w; the distances over w as the stipple's */
        add(v, "  gl_Position.xy = gl_Position.xy + aLine.xy * gl_Position.w;\n");
        add(v, "  g2vLine = vec4(aLine.z * gl_Position.w, aStip.y * gl_Position.w, aStip.z * gl_Position.w, gl_Position.w);\n");
    }
    if (k->zoff) add(v, "  gl_Position.z = gl_Position.z + uZOff.x * gl_Position.w;\n");
    /* the counter is linear in the window, a varying in the eye: s * w over w undoes the perspective */
    if (k->line_stip) add(v, "  g2vStip = vec4(aStip.x * gl_Position.w, gl_Position.w, 0.0, 0.0);\n");
    if (needs_eye) add(v, "  vec4 eye = uMV * aPos;\n");
    if (k->points && k->pt_att) add(v, "  float pd = length(eye.xyz);\n  gl_PointSize = clamp(uPoint.x * sqrt(1.0 / (uPointA.x + uPointA.y * pd + uPointA.z * pd * pd)), uPoint.y, uPoint.z);\n");   /* §3.3, derived size */
    else if (k->points) add(v, "  gl_PointSize = uPoint.x;\n");
    if (needs_n) {
        add(v, "  vec3 nrm = (uNM * vec4(aNormal, 0.0)).xyz;\n");
        if (k->normalize) add(v, "  nrm = normalize(nrm);\n");
        else if (k->rescale) add(v, "  nrm = nrm * uNM[3].w;\n");
    }
    if (k->light) {
        const int cm = k->color_mat ? k->cm_mode : 7;           /* 0 emission 1 ambient 2 diffuse 3 specular 4 ambient + diffuse */
        add(v, "  vec4 mDif = %s;\n  vec4 mAmb = %s;\n  vec4 mSpc = %s;\n  vec4 mEmi = %s;\n  vec4 mShi = uMShi;\n", cm == 2 || cm == 4 ? "aColor" : "uMDif",
            cm == 1 || cm == 4 ? "aColor" : "uMAmb", cm == 3 ? "aColor" : "uMSpc", cm == 0 ? "aColor" : "uMEmi");
        add(v, "  vec3 lc = mEmi.rgb + mAmb.rgb * uScene.rgb;\n  vec3 ls = vec3(0.0);\n");
        for (i = 0; i < MAX_LIGHTS; i++) if ((k->light_on >> i) & 1) gen_light(g, k, i, "m", "nrm", "lc", "ls");
        if (k->sep_spec) {
            add(v, "  g2vColor = vec4(clamp(lc, 0.0, 1.0), clamp(mDif.a, 0.0, 1.0));\n  g2vSpec = vec4(clamp(ls, 0.0, 1.0), 0.0);\n");
        } else {
            add(v, "  g2vColor = vec4(clamp(lc + ls, 0.0, 1.0), clamp(mDif.a, 0.0, 1.0));\n");
        }
        if (k->two_side) {
            add(v, "  vec3 nb = -nrm;\n  vec4 bDif = %s;\n  vec4 bAmb = %s;\n  vec4 bSpc = %s;\n  vec4 bEmi = %s;\n  vec4 bShi = uBShi;\n", cm == 2 || cm == 4 ? "aColor" : "uBDif",
                cm == 1 || cm == 4 ? "aColor" : "uBAmb", cm == 3 ? "aColor" : "uBSpc", cm == 0 ? "aColor" : "uBEmi");
            add(v, "  vec3 lb = bEmi.rgb + bAmb.rgb * uScene.rgb;\n  vec3 lt = vec3(0.0);\n");
            for (i = 0; i < MAX_LIGHTS; i++) if ((k->light_on >> i) & 1) gen_light(g, k, i, "b", "nb", "lb", "lt");
            add(v, "  g2vColorB = vec4(clamp(lb + lt, 0.0, 1.0), clamp(bDif.a, 0.0, 1.0));\n");
        }
    } else {
        add(v, "  g2vColor = clamp(aColor, 0.0, 1.0);\n");
        if (k->sec) add(v, "  g2vSpec = vec4(clamp(aSec.rgb, 0.0, 1.0), 0.0);\n");
    }
    if (k->need_spec && !(k->sep_spec && k->light) && !(k->sec && !k->light)) add(v, "  g2vSpec = vec4(0.0);\n");
    if (k->fogfrag) add(v, k->fogfrag == 2 ? "  g2vFogF = vec4(aFogC.x);\n" : "  g2vFogF = vec4(abs(eye.z));\n");
    if (need_sm || need_rfl) add(v, "  vec3 ue = normalize(eye.xyz);\n  vec3 rfl = ue - 2.0 * nrm * dot(nrm, ue);\n");
    if (need_sm) add(v, "  vec3 rz = rfl + vec3(0.0, 0.0, 1.0);\n  float mm = 2.0 * sqrt(dot(rz, rz));\n  vec3 sm = vec3(rfl.x / mm + 0.5, rfl.y / mm + 0.5, 0.0);\n");
    for (u = 0; u < MAX_UNITS; u++) if (k->u[u].target) gen_texcoord(g, k, u);
    if (k->fog) {
        if (k->fogc) add(v, "  float fz = aFogC.x;\n");     /* §3.10: the coordinate as it is, the eye distance by its size */
        else add(v, "  float fz = abs(eye.z);\n");
        switch (k->fog) {
        case 1: add(v, "  float ff = (uFog.z - fz) * uFog.w;\n"); break;           /* LINEAR: (e - z) / (e - s) */
        case 2: add(v, "  float ff = exp(-uFog.x * fz);\n"); break;                 /* EXP */
        default: add(v, "  float dz = uFog.x * fz;\n  float ff = exp(-dz * dz);\n"); break;   /* EXP2 */
        }
        add(v, "  g2vFog = vec4(clamp(ff, 0.0, 1.0));\n");
    }
    for (i = 0; i < MAX_CLIP; i++) if ((k->clip_mask >> i) & 1) add(v, "  g2vClip%d = vec4(dot(uClip%d, eye));\n", i, i);
    add(v, "}\n");

    /* ---- fragment program ---- */
    for (u = 0; u < MAX_UNITS; u++) if ((k->crep >> u & 1) && k->u[u].target) add(f, "#define g2vTc%d g2pc%d\n", u, u);
    add(f, "void main() {\n");
    for (u = 0; u < MAX_UNITS; u++) if ((k->crep >> u & 1) && k->u[u].target) add(f, "  vec4 g2pc%d = vec4(gl_PointCoord.x, %sgl_PointCoord.y, 0.0, 1.0);\n", u, k->sprite_low ? "1.0 - " : "");
    for (i = 0; i < MAX_CLIP; i++) if ((k->clip_mask >> i) & 1) add(f, "  if (g2vClip%d.x < 0.0) discard;\n", i);
    if (k->poly_stip) add(f, "  if (texture2D(uStip, gl_FragCoord.xy / 32.0).a < 0.5) discard;\n");
    if (k->line_stip) add(f, "  if (texture2D(uStip, vec2((mod(floor(g2vStip.x / g2vStip.y / uStipF.x), 16.0) + 0.5) / 16.0, 0.5)).a < 0.5) discard;\n");
    if (k->light && k->two_side && k->back_only) add(f, "  vec4 c = g2vColorB;\n");
    else if (k->light && k->two_side) add(f, "  vec4 c = g2vColor;\n  if (!gl_FrontFacing) c = g2vColorB;\n");
    else add(f, "  vec4 c = g2vColor;\n");
    for (u = 0; u < MAX_UNITS; u++) {
        char e[40];
        if (!k->u[u].target) continue;
        if (k->u[u].sw) { gen_sw(g, k, u); continue; }
        lod_arg(e, sizeof e, k, u);
        if (k->u[u].lod == 2) gen_lod(g, k, u);
        if (k->u[u].target == 2) add(f, k->u[u].lod == 2 ? "  vec4 t%d = textureCubeLod(g2Tex%d, g2vTc%d.xyz%s);\n" : "  vec4 t%d = textureCube(g2Tex%d, g2vTc%d.xyz%s);\n", u, u, u, e);
        else if (k->u[u].target == 4 && k->u[u].proj) add(f, "  vec4 t%d = %s(g2Tex%d, g2vTc%d.xyz / g2vTc%d.w%s);\n", u, k->u[u].lod == 2 ? "textureCubeLod" : "textureCube", u, u, u, e);
        else if (k->u[u].target == 4) add(f, "  vec4 t%d = %s(g2Tex%d, g2vTc%d.xyz%s);\n", u, k->u[u].lod == 2 ? "textureCubeLod" : "textureCube", u, u, e);
        else if (k->u[u].bimg || k->u[u].bs || k->u[u].bt) gen_fetch(g, k, u);
        else if (k->u[u].target == 3 && k->u[u].proj) add(f, "  vec4 t%d = %s(g2Tex%d, vec2(g2vTc%d.x / g2vTc%d.w, 0.5)%s);\n", u, tex_fn(k, u), u, u, u, e);
        else if (k->u[u].target == 3) add(f, "  vec4 t%d = %s(g2Tex%d, vec2(g2vTc%d.x, 0.5)%s);\n", u, tex_fn(k, u), u, u, e);
        else if (k->u[u].proj) add(f, "  vec4 t%d = %s(g2Tex%d, g2vTc%d.xy / g2vTc%d.w%s);\n", u, tex_fn(k, u), u, u, u, e);
        else add(f, "  vec4 t%d = %s(g2Tex%d, g2vTc%d.xy%s);\n", u, tex_fn(k, u), u, u, e);
    }
    (void)any_tex;
    if (k->mask0) add(f, "  if (t0.a < 0.5) discard;\n");
    /* what changes only c.rgb writes all of c while c.a waits in ca<kept>: each of r, g, b is GL's expression as it was, alpha comes back when something reads it.
     * The compiler moves a partial write through a register (7 micro-ops), a restore costs 4 */
    {
        int kept = -1;
        for (u = 0; u < MAX_UNITS; u++) {
            if (!k->u[u].target || (k->mask0 && !u)) continue;
            if (rgb_only(k, u)) {
                if (kept < 0) { add(f, "  float ca%d = c.a;\n", u); kept = u; }
                gen_env(g, k, u, 1);
            } else if (kept >= 0 && k->u[u].base == BF_ALPHA && k->u[u].env != 5) {     /* table 3.12, ALPHA: the alpha it makes from the one kept */
                if (k->u[u].env == 2) add(f, "  c = vec4(c.rgb, t%d.a);\n", u);
                else add(f, "  c = vec4(c.rgb, ca%d * t%d.a);\n", kept, u);
                kept = -1;
            } else {
                if (kept >= 0) { add(f, "  c = vec4(c.rgb, ca%d);\n", kept); kept = -1; }
                gen_env(g, k, u, 0);
            }
        }
        if ((k->sep_spec && k->light) || k->sec || k->fog) {
            if (kept < 0) { add(f, "  float ca%d = c.a;\n", MAX_UNITS); kept = MAX_UNITS; }
            if ((k->sep_spec && k->light) || k->sec) add(f, "  c = clamp(c + vec4(g2vSpec.rgb, 0.0), 0.0, 1.0);\n");
            if (k->fog) add(f, "  c = mix(uFogC, c, g2vFog.x);\n");
        }
        if (kept >= 0) add(f, "  c = vec4(c.rgb, ca%d);\n", kept);
    }
    /* antialiasing: the fragment's cover goes into alpha, §3.12 (coverage application), and a fragment with none does not exist */
    if (k->smooth_pt) {
        add(f, "  {\n    vec2 pc = (gl_PointCoord - vec2(0.5)) * uPointF.y;\n    float cov = clamp(uPointF.x + 0.5 - sqrt(dot(pc, pc)), 0.0, 1.0);\n    if (cov <= 0.0) discard;\n    c.a = c.a * cov;\n  }\n");
    }
    if (k->smooth_ln) {
        add(f, "  {\n    float ld = g2vLine.x / g2vLine.w;\n    float cov = clamp(min(ld + 0.5, uLineF.x) - max(ld - 0.5, -uLineF.x), 0.0, 1.0);\n");
        add(f, "    cov = cov * clamp(min(g2vLine.y / g2vLine.w, 0.5) + min(g2vLine.z / g2vLine.w, 0.5), 0.0, 1.0);\n    if (cov <= 0.0) discard;\n    c.a = c.a * cov;\n  }\n");
    }
    /* the alpha test discards on the comparison that fails it: c.a and the reference are numbers (§2.1.1, a NaN in gives what is unspecified), so that is the test's complement. Written after the colour,
     * which the compiler then stores without a copy, unless a logic op still changes it */
    {
        static const char *const FAIL[7] = { "", ">=", "!=", ">", "<=", "==", "<" };
        int late = k->alpha_fn >= 1 && k->alpha_fn <= 6 && !k->logic;
        if (k->alpha_fn == 0) add(f, "  discard;\n");           /* NEVER */
        else if (k->alpha_fn != 7 && !late) add(f, "  if (c.a %s uAlpha) discard;\n", FAIL[k->alpha_fn]);
        /* what the blender can do for a logic op: the fragment's value is decided here, and INVERT blends all ones with 1 - dst (gl_fill_hw_state) */
        if (k->logic == 1) add(f, "  c = vec4(0.0);\n");
        else if (k->logic == 2) add(f, "  c = vec4(1.0);\n");
        else if (k->logic == 3) add(f, "  c = vec4(1.0) - c;\n");
        add(f, "  gl_FragColor = c;\n");
        if (late) add(f, "  if (c.a %s uAlpha) discard;\n", FAIL[k->alpha_fn]);
        add(f, "}\n");
    }
}

/* ---------------------------------------------------------------- the key of the current state */
static uint8_t comb_fn(GLenum f)
{
    switch (f) {
    case GL_REPLACE: return 0;
    case GL_MODULATE: return 1;
    case GL_ADD: return 2;
    case GL_ADD_SIGNED: return 3;
    case GL_INTERPOLATE: return 4;
    case GL_SUBTRACT: return 5;
    case GL_DOT3_RGB: return 6;
    case GL_DOT3_RGBA: return 7;
    default: return 1;
    }
}

static uint8_t comb_srcc(GLenum s)
{
    if (s >= GL_TEXTURE0 && s < GL_TEXTURE0 + MAX_UNITS) return (uint8_t)(4 + s - GL_TEXTURE0);
    switch (s) {
    case GL_TEXTURE: return 0;
    case GL_CONSTANT: return 1;
    case GL_PRIMARY_COLOR: return 2;
    default: return 3;
    }
}

static uint8_t comb_op(GLenum o)
{
    switch (o) {
    case GL_SRC_COLOR: return 0;
    case GL_ONE_MINUS_SRC_COLOR: return 1;
    case GL_SRC_ALPHA: return 2;
    default: return 3;
    }
}

static int is_ident(const float *m)
{
    static const float id[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    return !memcmp(m, id, sizeof id);
}

void gl_ff_sw_key(ff_key *k, int u, const gl_tex *x)
{
    static const GLenum W[5] = { GL_REPEAT, GL_MIRRORED_REPEAT, GL_CLAMP, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_BORDER };
    static const GLenum F[6] = { GL_NEAREST, GL_LINEAR, GL_NEAREST_MIPMAP_NEAREST, GL_LINEAR_MIPMAP_NEAREST, GL_NEAREST_MIPMAP_LINEAR, GL_LINEAR_MIPMAP_LINEAR };
    int top = x->max_level < x->levels - 1 ? x->max_level : x->levels - 1, i;
    k->u[u].sw = k->u[u].target == 4 ? (x->vol3 ? 4 : 3) : x->base_class == BF_DEPTH ? 2 : 1;
    k->u[u].swmag = x->mag_filter == GL_LINEAR;
    for (i = 0; i < 6; i++) if (x->min_filter == F[i]) k->u[u].swmin = (uint8_t)i;
    k->u[u].swn = k->u[u].swmin >= 2 ? (uint8_t)(top - x->base_level + 1) : 1;
    k->u[u].swb = k->u[u].sw >= 3 ? (uint8_t)x->b3 : (uint8_t)(x->bimg && k->u[u].swmin < 2);
    for (i = 0; i < 5; i++) {
        if (x->wrap_s == W[i]) k->u[u].sww[0] = (uint8_t)i;
        if (x->wrap_t == W[i]) k->u[u].sww[1] = (uint8_t)i;
        if (x->wrap_r == W[i]) k->u[u].sww[2] = (uint8_t)i;
    }
    if (k->u[u].target == 3) k->u[u].sww[1] = 3;
    if (k->u[u].sw == 2) {
        int c = x->depth_mode == GL_INTENSITY ? 2 : x->depth_mode == GL_ALPHA ? 3 : 1;
        k->u[u].dmode = (uint8_t)c;
        k->u[u].base = c == 2 ? BF_INTENSITY : c == 3 ? BF_ALPHA : BF_RGB;      /* §3.8.13: the texture is what the mode makes it */
        if (x->compare_mode != GL_NONE) k->u[u].dcmp = (uint8_t)(x->compare_func - GL_NEVER + 1);
    }
}

void gl_ff_key(ff_key *k)
{
    int i, u;
    memset(k, 0, sizeof *k);
    k->light = C->lighting ? 1 : 0;
    k->points = C->drawing_points ? 1 : 0;
    k->poly_stip = C->polygon_stipple && C->drawing_cls == 0;
    k->line_stip = C->line_stipple && C->drawing_cls == 1;
    k->smooth_pt = C->point_smooth && C->drawing_points;
    k->smooth_ln = C->line_smooth && C->drawing_cls == 1;
    k->back_only = (uint8_t)C->drawing_back;
    k->zoff = C->z_off != 0.0f;
    k->mask0 = (uint8_t)C->drawing_mask;
    k->fogc = C->fog && C->fog_src == GL_FOG_COORD;
    k->sec = C->color_sum && !C->lighting;
    if (C->point_sprite && C->drawing_points) {                 /* §3.3.1: a sprite is a square, and what it carries in its coordinates */
        k->smooth_pt = 0;
        for (u = 0; u < MAX_UNITS; u++) if (C->unit[u].coord_replace) k->crep |= (uint8_t)(1u << u);
        k->sprite_low = C->sprite_origin == GL_LOWER_LEFT;
    }
    k->pt_att = C->drawing_points && (C->pt_att[0] != 1.0f || C->pt_att[1] != 0.0f || C->pt_att[2] != 0.0f || C->pt_min != 0.0f || C->pt_max != 64.0f);
    if (C->color_logic_op) k->logic = C->logic_op == GL_CLEAR ? 1 : C->logic_op == GL_SET || C->logic_op == GL_INVERT ? 2 : C->logic_op == GL_COPY_INVERTED ? 3 : 0;
    k->alpha_fn = C->alpha_test ? (C->alpha_func & 7) : 7;
    if (C->fog) k->fog = C->fog_mode == GL_LINEAR ? 1 : C->fog_mode == GL_EXP ? 2 : 3;
    for (i = 0; i < MAX_CLIP; i++) if (C->clip_en[i]) k->clip_mask |= 1u << i;
    if (C->lighting) {
        k->color_mat = C->color_material ? 1 : 0;
        k->two_side = C->lm_two_side ? 1 : 0;
        k->local_viewer = C->lm_local_viewer ? 1 : 0;
        k->sep_spec = C->lm_color_control == GL_SEPARATE_SPECULAR_COLOR;
        k->normalize = C->normalize ? 1 : 0;
        k->rescale = C->rescale_normal ? 1 : 0;
        switch (C->cm_mode) {
        case GL_EMISSION: k->cm_mode = 0; break;
        case GL_AMBIENT: k->cm_mode = 1; break;
        case GL_DIFFUSE: k->cm_mode = 2; break;
        case GL_SPECULAR: k->cm_mode = 3; break;
        default: k->cm_mode = 4; break;
        }
        for (i = 0; i < MAX_LIGHTS; i++) {
            const gl_light *l = &C->light[i];
            if (!l->enabled) continue;
            k->light_on |= (uint8_t)(1u << i);
            if (l->position[3] != 0.0f) {
                k->light_pos |= (uint8_t)(1u << i);
                if (l->spot_cut != 180.0f) k->light_spot |= (uint8_t)(1u << i);
            }
        }
    }
    for (u = 0; u < MAX_UNITS; u++) {
        gl_unit *t = &C->unit[u];
        gl_tex *x = gl_unit_tex(u, t->en_cube ? 1 : t->en_3d ? 3 : t->en_2d ? 0 : 2);   /* cube over 3D over 2D over 1D, §3.8.15 */
        if (!(t->en_1d || t->en_2d || t->en_cube || t->en_3d) || !x || !gl_tex_complete(x)) continue;   /* an incomplete texture: as if disabled */
        k->tex_units++;
        k->u[u].target = t->en_cube ? 2 : t->en_3d ? 4 : t->en_2d ? 1 : 3;
        k->u[u].base = (uint8_t)x->base_class;
        switch (t->env_mode) {
        case GL_DECAL: k->u[u].env = 1; break;
        case GL_REPLACE: k->u[u].env = 2; break;
        case GL_BLEND: k->u[u].env = 3; break;
        case GL_ADD: k->u[u].env = 4; break;
        case GL_COMBINE: k->u[u].env = 5; break;
        default: k->u[u].env = 0; break;
        }
        k->u[u].texmat = !is_ident(t->mat[t->mat_sp]);
        for (i = 0; i < 4; i++) {
            if (!t->gen_en[i]) continue;
            switch (t->gen_mode[i]) {
            case GL_OBJECT_LINEAR: k->u[u].gen[i] = 1; break;
            case GL_EYE_LINEAR: k->u[u].gen[i] = 2; break;
            case GL_SPHERE_MAP: k->u[u].gen[i] = 3; break;
            case GL_NORMAL_MAP: k->u[u].gen[i] = 4; break;
            case GL_REFLECTION_MAP: k->u[u].gen[i] = 5; break;
            default: break;
            }
        }
        k->u[u].proj = k->u[u].texmat || k->u[u].gen[3] || C->a_tc[u].size == 4;
        if (k->u[u].target == 4 ? !gl_tex3d_tmu(x) : x->base_class == BF_DEPTH) {
            gl_ff_sw_key(k, u, x);
        } else if (x->min_lod > -1000.0f || x->max_lod < 1000.0f) k->u[u].lod = 2;
        else if (x->lod_bias != 0.0f || t->lod_bias != 0.0f) k->u[u].lod = 1;
        if (k->u[u].target != 2 && k->u[u].target != 4 && !k->u[u].sw) {
            static const GLenum W[5] = { GL_REPEAT, GL_MIRRORED_REPEAT, GL_CLAMP, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_BORDER };
            int mip = x->min_filter != GL_NEAREST && x->min_filter != GL_LINEAR, oned = k->u[u].target == 3, ws = 0, wt = 0;
            int linear = x->mag_filter == GL_LINEAR || x->min_filter == GL_LINEAR || x->min_filter == GL_LINEAR_MIPMAP_NEAREST || x->min_filter == GL_LINEAR_MIPMAP_LINEAR;
            for (i = 0; i < 5; i++) { if (x->wrap_s == W[i]) ws = i; if (x->wrap_t == W[i]) wt = i; }
            if (x->bimg && !mip && (ws >= 2 || (!oned && wt >= 2))) {
                k->u[u].bimg = 1; k->u[u].ws = (uint8_t)ws; k->u[u].wt = (uint8_t)wt;
            } else {
                k->u[u].bs = ws == 2 ? (linear ? 1 : 0) : ws == 4 ? (linear ? 2 : 3) : 0;
                if (!oned) k->u[u].bt = wt == 2 ? (linear ? 1 : 0) : wt == 4 ? (linear ? 2 : 3) : 0;
                k->u[u].blod = (k->u[u].bs || k->u[u].bt) && mip;
            }
        }
        if (k->u[u].env == 5) {
            k->u[u].comb_rgb = comb_fn(t->comb_rgb); k->u[u].comb_a = comb_fn(t->comb_a);
            k->u[u].scale_rgb = (uint8_t)t->scale_rgb; k->u[u].scale_a = (uint8_t)t->scale_a;
            for (i = 0; i < 3; i++) {
                k->u[u].src_rgb[i] = comb_srcc(t->src_rgb[i]); k->u[u].src_a[i] = comb_srcc(t->src_a[i]);
                k->u[u].op_rgb[i] = comb_op(t->op_rgb[i]); k->u[u].op_a[i] = comb_op(t->op_a[i]);
            }
        }
    }
}

#define FF_BUCKETS 257
static ff_prog *g_cache[FF_BUCKETS];

static const char *const ATTR_NAME[A_COUNT] = { "aPos", "aColor", "aNormal", "aTc0", "aTc1", "aTc2", "aTc3", "aTc4", "aTc5", "aTc6", "aTc7", "aStip", "aLine", "aFogC", "aSec" };

int gl_ff_attr_of(const char *name)
{
    int i;
    for (i = 0; i < A_COUNT; i++) if (ATTR_NAME[i] && !strcmp(ATTR_NAME[i], name)) return i;
    return -1;
}

static int find_sym(const g2sc_sym *s, int n, const char *name)
{
    int i;
    for (i = 0; i < n; i++) if (!strcmp(s[i].name, name)) return s[i].loc;
    return -1;
}

int gl_ff_find(const g2sc_sym *s, int n, const char *name) { return find_sym(s, n, name); }

/* what a user's program takes of the generated ones */
void gl_ff_text(const ff_key *k, char **vs, char **fs, gl_ureq *vu, int *nvu, gl_ureq *fu, int *nfu)
{
    gen g;
    memset(&g, 0, sizeof g);
    g.vs.cap = g.fs.cap = 16384;
    g.vs.p = (char *)malloc(g.vs.cap); g.fs.p = (char *)malloc(g.fs.cap);
    g.vs.p[0] = g.fs.p[0] = 0;
    generate(&g, k);
    *vs = g.vs.p; *fs = g.fs.p;
    memcpy(vu, g.vu, (size_t)g.nvu * sizeof *vu); *nvu = g.nvu;
    memcpy(fu, g.fu, (size_t)g.nfu * sizeof *fu); *nfu = g.nfu;
}

/* sampler u of a user's fragment shader as functions g2s<u>(vec3 st, float bias) and g2sl<u>(vec3 st, float level): k->u[] holds them all, as the level table is shared */
void gl_ff_sw_text(const ff_key *k, int u, char **text, gl_ureq *fu, int *nfu)
{
    gen g;
    memset(&g, 0, sizeof g);
    g.vs.cap = 64; g.fs.cap = 16384;
    g.vs.p = (char *)malloc(g.vs.cap); g.fs.p = (char *)malloc(g.fs.cap);
    g.vs.p[0] = g.fs.p[0] = 0;
    add(&g.fs, "uniform %s g2Tex%d;\n", k->u[u].sw == 4 ? "samplerCube" : "sampler2D", u);
    gen_sw_funcs(&g, k, u);
    add(&g.fs, "vec4 g2s%d(vec3 st, float g2b) {\n  vec4 t%d;\n", u, u);
    gen_sw_body(&g.fs, k, u, "g2b", NULL);
    add(&g.fs, "  return t%d;\n}\nvec4 g2sl%d(vec3 st, float g2l) {\n  vec4 t%d;\n", u, u, u);
    gen_sw_body(&g.fs, k, u, NULL, "g2l");
    add(&g.fs, "  return t%d;\n}\n", u);
    *text = g.fs.p;
    free(g.vs.p);
    memcpy(fu, g.fu, (size_t)g.nfu * sizeof *fu); *nfu = g.nfu;
}

/* ---- more varyings than the hardware's 8 rows, 8 units with colours and clip planes. The text stays as it is: a varying becomes a global written at the end of main, a smaller varying takes what it needs of it, and the
 * fragment program sees the vec4 again through a macro. g2sc_link then packs them by Appendix A.7 */
static void sb_sub(sb *s, const char *a, const char *b)
{
    char *q = strstr(s->p, a);
    size_t la = strlen(a), lb = strlen(b);
    if (!q) return;
    while (s->n + lb + 1 > s->cap) { size_t at = (size_t)(q - s->p); s->cap *= 2; s->p = (char *)realloc(s->p, s->cap); q = s->p + at; }
    memmove(q + lb, q + la, s->n - (size_t)(q - s->p) - la + 1);
    memcpy(q, b, lb);
    s->n += lb - la;
}

static void tighten(gen *g, const char *name, int n)
{
    static const char *const TY[5] = { "", "float", "vec2", "vec3", "vec4" }, *const SW[5] = { "", ".x", ".xy", ".xyz", "" };
    char a[48], b[96], c[128];
    snprintf(a, sizeof a, "varying vec4 %s;\n", name);
    if (!strstr(g->vs.p, a)) return;
    snprintf(b, sizeof b, "vec4 %s;\nvarying %s g2c_%s;\n", name, TY[n], name);
    sb_sub(&g->vs, a, b);
    g->vs.n -= 2; g->vs.p[g->vs.n] = 0;                         /* the closing brace of main, and back */
    add(&g->vs, "  g2c_%s = %s%s;\n}\n", name, name, SW[n]);
    snprintf(c, sizeof c, "varying %s g2c_%s;\n#define %s vec4(g2c_%s%s)\n", TY[n], name, name, name, n == 4 ? "" : n == 3 ? ", 1.0" : n == 2 ? ", 0.0, 1.0" : ", 0.0, 0.0, 1.0");
    sb_sub(&g->fs, a, c);
}

/* the varyings and what each needs of its vec4 */
typedef struct ff_vary { char name[16]; int n; } ff_vary;
#define FF_NVARY (7 + MAX_CLIP + MAX_UNITS)

static int ff_varys(const ff_key *k, ff_vary *o)
{
    static const ff_vary FIX[7] = { { "g2vColor", 4 }, { "g2vColorB", 4 }, { "g2vSpec", 3 }, { "g2vFog", 1 }, { "g2vFogF", 1 }, { "g2vStip", 2 }, { "g2vLine", 4 } };
    int c = 0, u;
    for (u = 0; u < 7; u++) o[c++] = FIX[u];
    for (u = 0; u < MAX_CLIP; u++) { snprintf(o[c].name, sizeof o[c].name, "g2vClip%d", u); o[c++].n = 1; }
    for (u = 0; u < MAX_UNITS; u++) {
        if (k->crep >> u & 1) continue;                         /* a sprite's coordinate is gl_PointCoord's macro: its vec4 stays, nothing reads it */
        snprintf(o[c].name, sizeof o[c].name, "g2vTc%d", u);
        o[c++].n = k->u[u].proj ? 4 : k->u[u].target == 2 || k->u[u].target == 4 || k->u[u].sw ? 3 : 2;
    }
    return c;
}

static void tighten_all(gen *g, const ff_key *k)
{
    ff_vary v[FF_NVARY];
    int i, n = ff_varys(k, v);
    for (i = 0; i < n; i++) tighten(g, v[i].name, v[i].n);
}

/* do the tightened varyings of both programs take fewer rows by Appendix A.7 than a row each: two 2D coordinates in one row are one LDV on board B */
static int ff_packs(const gen *g, const ff_key *k)
{
    static const int RANK[5] = { 6, 6, 5, 4, 2 };              /* float, vec2, vec3, vec4: g2sc_pack_vars' ranks */
    ff_vary v[FF_NVARY];
    int rank[FF_NVARY], rows[FF_NVARY], at[FF_NVARY], i, m = 0, r, n = ff_varys(k, v);
    char a[48];
    for (i = 0; i < n; i++) {
        snprintf(a, sizeof a, "varying vec4 %s;\n", v[i].name);
        if (!strstr(g->vs.p, a) || !strstr(g->fs.p, a)) continue;
        rank[m] = RANK[v[i].n]; rows[m] = 1; m++;
    }
    for (r = 1; r < m; r++) if (g2sc_pack_vars(rank, rows, m, r, at) >= 0) return 1;
    return 0;
}

/* gl_ff_program's texts of a key, for tests/ff_dump.c: 1 = tightened, g2sc_link packs them (past 8 rows without drop_q, which reads the draw's matrices) */
int gl_ff_text_tight(const ff_key *k, char **vs, char **fs)
{
    gen g;
    const char *q;
    int rows = 0, t;
    memset(&g, 0, sizeof g);
    g.vs.cap = g.fs.cap = 16384;
    g.vs.p = (char *)malloc(g.vs.cap); g.fs.p = (char *)malloc(g.fs.cap);
    g.vs.p[0] = g.fs.p[0] = 0;
    generate(&g, k);
    for (q = g.vs.p; (q = strstr(q, "varying vec4 ")) != NULL; q++) rows++;
    t = rows > 8 || ff_packs(&g, k);
    if (t) tighten_all(&g, k);
    *vs = g.vs.p; *fs = g.fs.p;
    return t;
}

/* a q of 1 needs no fourth coordinate: what a unit's coordinate can be, for the case where every row counts */
static void drop_q(ff_key *k)
{
    int u;
    for (u = 0; u < MAX_UNITS; u++) {
        const gl_unit *t = &C->unit[u];
        const float *m = t->mat[t->mat_sp];
        if (!k->u[u].target || !k->u[u].proj || k->u[u].gen[3]) continue;
        if (k->u[u].texmat && (m[3] != 0.0f || m[7] != 0.0f || m[11] != 0.0f || m[15] != 1.0f)) continue;
        if (C->a_tc[u].enabled ? C->a_tc[u].size == 4 : (C->tc4 >> u & 1) != 0) continue;
        k->u[u].proj = 0;
    }
}

ff_prog *gl_ff_program(void)
{
    static ff_prog *last;
    ff_key k;
    uint32_t h = 2166136261u, i, b;
    const uint8_t *kb = (const uint8_t *)&k;
    ff_prog *p;
    gen g;
    g2sc_result rv, rf;
    uint32_t fsmask = 0;
    int rows = 0, tight;
    const char *q;
    g2sc_linked lk;
    gl_ff_key(&k);
    if (last && !memcmp(&last->key, &k, sizeof k)) return last;
    for (i = 0; i < sizeof k; i++) { h ^= kb[i]; h *= 16777619u; }
    b = h % FF_BUCKETS;
    for (p = g_cache[b]; p; p = p->next) {
        if (!memcmp(&p->key, &k, sizeof k)) { last = p; return p; }
    }
    memset(&g, 0, sizeof g);
    g.vs.cap = g.fs.cap = 16384;
    g.vs.p = (char *)malloc(g.vs.cap); g.fs.p = (char *)malloc(g.fs.cap);
    g.vs.p[0] = g.fs.p[0] = 0;
    generate(&g, &k);
    for (q = g.vs.p; (q = strstr(q, "varying vec4 ")) != NULL; q++) rows++;
    tight = rows > 8 || ff_packs(&g, &k);
    memset(&rv, 0, sizeof rv); memset(&rf, 0, sizeof rf);
    if (tight) {
        ff_key k2 = k;
        if (rows > 8) {                                         /* drop_q reads the matrices, not the key: only where every row counts */
            drop_q(&k2);
            g.vs.n = g.fs.n = 0; g.vs.p[0] = g.fs.p[0] = 0; g.nvu = g.nfu = 0;
            generate(&g, &k2);
        }
        tighten_all(&g, &k2);
        if (g2sc_link(g.vs.p, g.fs.p, &lk) < 0 || lk.umap_vs || lk.umap_fs) {
            g2e_log("gl: %d varyings of the fixed pipeline do not fit: %s", rows, lk.error);
            free(g.vs.p); free(g.fs.p);
            return NULL;
        }
        rv = lk.vs; rf = lk.fs;
        fsmask = lk.fsmask;
    } else if (g2sc_compile(g.vs.p, G2SC_VS, &rv) < 0 || g2sc_compile(g.fs.p, G2SC_FS, &rf) < 0) {
        g2e_log("gl: program does not compile: %s %s\n--- vs\n%s\n--- fs\n%s", rv.error, rf.error, g.vs.p, g.fs.p);
        g2sc_free(&rv); g2sc_free(&rf);
        free(g.vs.p); free(g.fs.p);
        return NULL;
    }
    p = (ff_prog *)calloc(1, sizeof *p);
    p->key = k;
    if (!tight) for (i = 0; i < (uint32_t)rf.nvaryings; i++) fsmask |= 1u << rf.varyings[i].loc;
    p->p = g2r_prog_create(D.r, rv.words, (uint32_t)rv.nwords, (const double (*)[4])rv.const_pool, (uint32_t)rv.nconst, (uint32_t)rv.const_base, rf.words, (uint32_t)rf.nwords,
                           (const double (*)[4])rf.const_pool, (uint32_t)rf.nconst, (uint32_t)rf.const_base, fsmask);
    for (i = 0; i < A_COUNT; i++) p->attr_loc[i] = ATTR_NAME[i] ? find_sym(rv.attribs, rv.nattribs, ATTR_NAME[i]) : -1;
    for (i = 0; i < (uint32_t)g.nvu; i++) {
        int s = find_sym(rv.uniforms, rv.nuniforms, g.vu[i].name);
        if (s >= 0) { ff_uni *q = &p->vu[p->nvu++]; q->kind = g.vu[i].kind; q->idx = g.vu[i].idx; q->slot = (uint16_t)s; }
    }
    for (i = 0; i < (uint32_t)g.nfu; i++) {
        int s = find_sym(rf.uniforms, rf.nuniforms, g.fu[i].name);
        if (s >= 0) { ff_uni *q = &p->fu[p->nfu++]; q->kind = g.fu[i].kind; q->idx = g.fu[i].idx; q->slot = (uint16_t)s; }
    }
    for (i = 0; i < MAX_UNITS; i++) {
        char nm[16];
        snprintf(nm, sizeof nm, "g2Tex%u", i);
        p->sampler[i] = find_sym(rf.samplers, rf.nsamplers, nm);
    }
    p->stip_sampler = find_sym(rf.samplers, rf.nsamplers, "uStip");
    if (rv.nvaryings != rf.nvaryings && !tight) g2e_log("gl: %d varyings written, %d read", rv.nvaryings, rf.nvaryings);
    g2e_log("gl: program %u: VS %d, FS %d instructions", p->p ? p->p->id : 0, rv.nwords, rf.nwords);
    if (tight) g2sc_link_free(&lk);
    else { g2sc_free(&rv); g2sc_free(&rf); }
    free(g.vs.p); free(g.fs.p);
    if (!p->p) { free(p); return NULL; }
    p->next = g_cache[b];
    g_cache[b] = p;
    last = p;
    return p;
}

/* ---------------------------------------------------------------- uniforms */
void gl_ident(float *m) { memset(m, 0, 64); m[0] = m[5] = m[10] = m[15] = 1.0f; }

void gl_mul(float *r, const float *a, const float *b)
{
    float t[16];
    int c, w, k;
    for (c = 0; c < 4; c++) {
        for (w = 0; w < 4; w++) {
            float s = 0;
            for (k = 0; k < 4; k++) s += a[k * 4 + w] * b[c * 4 + k];
            t[c * 4 + w] = s;
        }
    }
    memcpy(r, t, sizeof t);
}

int gl_invert(float *out, const float *m)
{
    double a[4][8];
    int i, j, k;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) { a[i][j] = m[j * 4 + i]; a[i][j + 4] = i == j; }
    }
    for (i = 0; i < 4; i++) {
        int p = i;
        double d;
        for (k = i + 1; k < 4; k++) if (fabs(a[k][i]) > fabs(a[p][i])) p = k;
        if (a[p][i] == 0.0) return 0;
        if (p != i) for (j = 0; j < 8; j++) { double t = a[p][j]; a[p][j] = a[i][j]; a[i][j] = t; }
        d = a[i][i];
        for (j = 0; j < 8; j++) a[i][j] /= d;
        for (k = 0; k < 4; k++) {
            if (k != i) {
                double f = a[k][i];
                for (j = 0; j < 8; j++) a[k][j] -= f * a[i][j];
            }
        }
    }
    for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) out[j * 4 + i] = (float)a[i][j + 4];
    return 1;
}

static void put4(uint32_t *d, const float *v)
{
    d[0] = g2_fp24(v[0]); d[1] = g2_fp24(v[1]); d[2] = g2_fp24(v[2]); d[3] = g2_fp24(v[3]);
}

static void put4f(uint32_t *d, float a, float b, float c, float e)
{
    d[0] = g2_fp24(a); d[1] = g2_fp24(b); d[2] = g2_fp24(c); d[3] = g2_fp24(e);
}

static void put_mat(uint32_t *d, const float *m)
{
    int i;
    for (i = 0; i < 16; i++) d[i] = g2_fp24(m[i]);
}

static const gl_tex *uni_tex(const ff_prog *p, int u)
{
    int t = p->key.u[u].target;
    if (p->user_fs) return gl_glsl_tex(p, u);
    return gl_unit_tex(u, t == 3 ? 2 : t == 2 ? 1 : t == 4 ? 3 : 0);
}

void gl_ff_uniforms(const ff_prog *p, uint32_t *vs, uint32_t *fs)
{
    const float *mv = C->mv[C->mv_sp], *pr = C->pr[C->pr_sp];
    float mvp[16], nm[16];
    int i, have_mvp = 0, have_nm = 0;
    double kx, ky;
    float psize;
    gl_scale(&kx, &ky);
    psize = C->point_size * (float)ky;
    for (i = 0; i < p->nvu; i++) {
        const ff_uni *q = &p->vu[i];
        uint32_t *d = vs + 4 * q->slot;
        const gl_light *l = &C->light[q->idx];
        const gl_material *f0 = &C->mat[0], *b0 = &C->mat[1];
        switch (q->kind) {
        case U_MVP:
            if (!have_mvp) { gl_mul(mvp, pr, mv); have_mvp = 1; }
            put_mat(d, mvp);
            break;
        case U_MV: put_mat(d, mv); break;
        case U_NM:
            if (!have_nm) {                                     /* the inverse transpose of the modelview's upper 3 x 3 */
                float inv[16];
                int r, c;
                gl_ident(nm);
                if (gl_invert(inv, mv)) {
                    for (r = 0; r < 3; r++) for (c = 0; c < 3; c++) nm[c * 4 + r] = inv[r * 4 + c];
                }
                /* GL_RESCALE_NORMAL's factor (§2.10.3) in the unused corner */
                nm[15] = 1.0f / sqrtf(inv[2] * inv[2] + inv[6] * inv[6] + inv[10] * inv[10]);
                have_nm = 1;
            }
            put_mat(d, nm);
            break;
        case U_TM: put_mat(d, C->unit[q->idx].mat[C->unit[q->idx].mat_sp]); break;
        case U_COLOR_SCENE: put4(d, C->lm_ambient); break;
        case U_L_POS:
            if (l->position[3] != 0.0f) {
                put4(d, l->position);
            } else {
                float n = sqrtf(l->position[0] * l->position[0] + l->position[1] * l->position[1] + l->position[2] * l->position[2]);
                if (n == 0.0f) n = 1.0f;
                put4f(d, l->position[0] / n, l->position[1] / n, l->position[2] / n, 0.0f);
            }
            break;
        case U_L_AMB: put4(d, l->ambient); break;
        case U_L_DIF: put4(d, l->diffuse); break;
        case U_L_SPC: put4(d, l->specular); break;
        case U_L_DIR: put4f(d, l->spot_dir[0], l->spot_dir[1], l->spot_dir[2], 0.0f); break;
        case U_L_ATT: put4f(d, l->atten[0], l->atten[1], l->atten[2], 0.0f); break;
        case U_L_SPOT: put4f(d, l->spot_exp, cosf(l->spot_cut * 3.14159265358979f / 180.0f), 0.0f, 0.0f); break;
        case U_M_EMI: put4(d, f0->emission); break;
        case U_M_AMB: put4(d, f0->ambient); break;
        case U_M_DIF: put4(d, f0->diffuse); break;
        case U_M_SPC: put4(d, f0->specular); break;
        case U_M_SHI: put4f(d, f0->shininess, 0.0f, 0.0f, 0.0f); break;
        case U_BM_EMI: put4(d, b0->emission); break;
        case U_BM_AMB: put4(d, b0->ambient); break;
        case U_BM_DIF: put4(d, b0->diffuse); break;
        case U_BM_SPC: put4(d, b0->specular); break;
        case U_BM_SHI: put4f(d, b0->shininess, 0.0f, 0.0f, 0.0f); break;
        case U_GEN_S: case U_GEN_T: case U_GEN_R: case U_GEN_Q: {
            const gl_unit *t = &C->unit[q->idx];
            int c = q->kind - U_GEN_S;
            put4(d, t->gen_mode[c] == GL_OBJECT_LINEAR ? t->gen_obj[c] : t->gen_eye[c]);
            break;
        }
        case U_FOG: {
            float e = C->fog_end - C->fog_start;
            put4f(d, C->fog_density, C->fog_start, C->fog_end, e != 0.0f ? 1.0f / e : 0.0f);
            break;
        }
        case U_CLIP: put4(d, C->clip_plane[q->idx]); break;
        case U_POINT: put4f(d, p->key.smooth_pt ? psize + 2.0f : psize, C->pt_min * (float)ky, C->pt_max * (float)ky, 0.0f); break;
        case U_POINTA: put4f(d, C->pt_att[0], C->pt_att[1], C->pt_att[2], 0.0f); break;
        case U_Z: put4f(d, C->z_off, 0.0f, 0.0f, 0.0f); break;
        default: break;
        }
    }
    for (i = 0; i < p->nfu; i++) {
        const ff_uni *q = &p->fu[i];
        uint32_t *d = fs + 4 * q->slot;
        switch (q->kind) {
        case U_F_FOGC: put4(d, C->fog_color); break;
        case U_F_ALPHA: put4f(d, C->alpha_ref, 0.0f, 0.0f, 0.0f); break;
        case U_F_ENV: put4(d, C->unit[q->idx].env_color); break;
        case U_F_STIP: put4f(d, (float)C->line_factor, 0.0f, 0.0f, 0.0f); break;
        case U_F_LINE: put4f(d, C->line_width * (float)ky * 0.5f, 0.0f, 0.0f, 0.0f); break;
        case U_F_POINT: put4f(d, psize * 0.5f, psize + 2.0f, 0.0f, 0.0f); break;
        case U_F_TSZ: {
            const gl_tex *x = uni_tex(p, q->idx);
            int b = x->base_level > 0 ? x->base_level : 0, w = x->w >> b, h = x->h >> b, e = x->d3 >> b;     /* the texels of the base level, which is level 0 of what the TMU is given */
            put4f(d, (float)(w ? w : 1), (float)(h ? h : 1), (float)(e ? e : 1), 0.0f);
            break;
        }
        case U_F_LOD: {
            const gl_tex *x = uni_tex(p, q->idx);
            float b = x->lod_bias + C->unit[q->idx].lod_bias;
            b = b < -15.0f ? -15.0f : b > 15.0f ? 15.0f : b;    /* MAX_TEXTURE_LOD_BIAS */
            put4f(d, b, x->min_lod, x->max_lod, 0.0f);
            break;
        }
        case U_F_BRD: { float b[4]; gl_tex_border_color(uni_tex(p, q->idx), b); put4(d, b); break; }
        case U_F_SWL: {
            const gl_tex *x = uni_tex(p, q->idx);
            int b = x->base_level > 0 ? x->base_level : 0, l;
            for (l = 0; l < p->key.u[q->idx].swn; l++) {
                float sc = 1.0f / (float)(1 << l);
                put4f(d + 4 * l, (float)x->lv3[b + l][0], (float)x->lv3[b + l][1], (float)x->lv3[b + l][2], sc);
            }
            break;
        }
        case U_F_SWD: {
            int l;
            for (l = 0; l < q->idx; l++) put4f(d + 4 * l, (float)l, 0.0f, 0.0f, 1.0f / (float)(1 << l));
            break;
        }
        case U_F_SWS: {
            const gl_tex *x = uni_tex(p, q->idx);
            int b = x->base_level > 0 ? x->base_level : 0, s3 = p->key.u[q->idx].sw >= 3;
            int w = (s3 ? x->w3 : x->w) >> b, h = (s3 ? x->h3 : x->h) >> b, e = s3 ? x->d3 >> b : 1;
            put4f(d, (float)(w ? w : 1), (float)(h ? h : 1), (float)(e ? e : 1), 0.0f);
            break;
        }
        case U_F_RTC: put4(d, C->raster_tc); break;
        case U_F_SWA: { const gl_tex *x = uni_tex(p, q->idx); put4f(d, 1.0f / (float)x->aw3, 1.0f / (float)x->ah3, 0.0f, 0.0f); break; }
        default: break;
        }
    }
}
