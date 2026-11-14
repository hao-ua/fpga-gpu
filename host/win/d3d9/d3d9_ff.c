/* d3d9_ff.c — programs: fixed function from a state key, shaders from d3d9_sm.c's text, each pair linked once and cached.
 * Inputs are named a_u<usage>_<index> and outputs by what they carry (v_c0, v_c1, v_t<n>, v_fog), so any vertex stage links with any pixel stage. Fog and alpha test are the pipeline's last steps
 * in Direct3D 9 whatever the pixel stage is, so they go on the end of every fragment program. */
#include "d3d9_internal.h"
#include <math.h>

typedef struct tc_key { uint8_t on, ci, gen, ttf, proj, dim, isz; } tc_key;     /* proj: the coordinate to divide by, 1 .. 4; isz: the coordinates the vertex has */
typedef struct st_key { uint8_t cop, ca0, ca1, ca2, aop, aa0, aa1, aa2, res, tex; } st_key;
typedef struct pkey {
    uint32_t vs, ps;            /* shader ids, 0: fixed function */
    uint32_t rb;                /* inputs that arrive as D3DCOLOR: bytes B G R A */
    uint8_t xyzrhw, normal, c0, c1, psize, lighting, normalize, local, spec, cvertex, dsrc, asrc, ssrc, esrc;
    uint8_t light[MAX_LIGHTS];
    uint8_t fog, vfog, tfog, range, wfog, sprite, clip, afunc;
    uint8_t points, pscale, flat, blend, ixblend, ixcolor, tween;       /* blend: the matrices of each vertex */
    tc_key tc[MAX_STAGES];
    st_key st[MAX_STAGES];
    uint8_t stex[8], sproj[8], ssign[8];                                /* per sampler: a ps 1.x's cube or 2D and projection, the format's signed channels */
    uint8_t sdepth[8];                                                  /* a depth texture: 1 its value, 2 a comparison */
    uint8_t sau[8], sav[8], saw[8], svol[8], sbias[8], ssrgb[8], slin[8], smip[8];   /* address U, V, W done in the program: 1 mirror once, 2 border; read with three coordinates; a LOD bias; sRGB read; linear, mipmapped: how a border reads */
    uint8_t srgbw;                                                      /* D3DRS_SRGBWRITEENABLE */
    uint8_t zrev;                                                       /* the depth surface keeps depth reversed (d3_zrev): the vertex stage gives z' = w - z */
    short li[2][16][3];                                                 /* what rep and loop read from the integer constants: vertex, pixel */
} pkey;

static const char *const U_NAMES[] = { "u_wvp", "u_wv", "u_proj", "u_norm", "u_vp", "u_rt", "u_gamb", "u_matd", "u_mata", "u_mats", "u_mate", "u_matp", "u_fog", "u_psize", "u_tf", "u_acolor", "u_fogcol", "u_aref", "u_pscale", "u_tween" };

static int decl_has(const d3decl *d, int usage, int index)
{
    int i;
    for (i = 0; i < d->n; i++) if (d->el[i].Usage == usage && d->el[i].UsageIndex == index) return 1 + i;
    return 0;
}

static uint8_t mcs(const d3dev *d, DWORD rs, int have_c0, int have_c1)
{
    DWORD v = d->st.rs[rs];
    if (!d->st.rs[D3DRS_COLORVERTEX]) return D3DMCS_MATERIAL;
    if (v == D3DMCS_COLOR1 && !have_c0) return D3DMCS_MATERIAL;
    if (v == D3DMCS_COLOR2 && !have_c1) return D3DMCS_MATERIAL;
    return (uint8_t)v;
}

static int elem_comps(BYTE t)
{
    switch (t) {
    case D3DDECLTYPE_FLOAT1: return 1;
    case D3DDECLTYPE_FLOAT2: case D3DDECLTYPE_SHORT2: case D3DDECLTYPE_SHORT2N: case D3DDECLTYPE_USHORT2N: return 2;
    case D3DDECLTYPE_FLOAT3: return 3;
    default: return 4;
    }
}

static void key_fill(d3dev *d, pkey *k, const d3decl *de, d3shader *vs, d3shader *ps, int mode)
{
    const d3state *s = &d->st;
    int i;
    memset(k, 0, sizeof *k);
    k->vs = vs ? vs->id : 0; k->ps = ps ? ps->id : 0;
    k->points = mode == 3;
    if (k->points) k->sprite = s->rs[D3DRS_POINTSPRITEENABLE] != 0;
    k->flat = s->rs[D3DRS_SHADEMODE] == D3DSHADE_FLAT && !k->points;
    if (s->rs[D3DRS_ALPHATESTENABLE]) k->afunc = (uint8_t)s->rs[D3DRS_ALPHAFUNC];
    if (s->rs[D3DRS_FOGENABLE]) { k->fog = 1; k->tfog = (uint8_t)s->rs[D3DRS_FOGTABLEMODE]; k->vfog = (uint8_t)s->rs[D3DRS_FOGVERTEXMODE]; k->range = s->rs[D3DRS_RANGEFOGENABLE] != 0; k->wfog = (s->proj.m[3][3] == 0.0f); }
    for (i = 0; i < MAX_CLIP; i++) if (s->rs[D3DRS_CLIPPLANEENABLE] >> i & 1) k->clip |= (uint8_t)(1 << i);
    if (!vs) {
        DWORD vb = s->rs[D3DRS_VERTEXBLEND];
        k->xyzrhw = decl_has(de, D3DDECLUSAGE_POSITIONT, 0) != 0;
        k->normal = decl_has(de, D3DDECLUSAGE_NORMAL, 0) != 0;
        k->c0 = decl_has(de, D3DDECLUSAGE_COLOR, 0) != 0;
        k->c1 = decl_has(de, D3DDECLUSAGE_COLOR, 1) != 0;
        k->psize = decl_has(de, D3DDECLUSAGE_PSIZE, 0) != 0;
        if (k->points && !k->xyzrhw) k->pscale = s->rs[D3DRS_POINTSCALEENABLE] != 0;
        if (!k->xyzrhw) {
            int ix = s->rs[D3DRS_INDEXEDVERTEXBLENDENABLE] ? decl_has(de, D3DDECLUSAGE_BLENDINDICES, 0) : 0, w = decl_has(de, D3DDECLUSAGE_BLENDWEIGHT, 0);
            k->ixblend = ix != 0;
            if (ix) k->ixcolor = de->el[ix - 1].Type == D3DDECLTYPE_D3DCOLOR;
            if (vb == D3DVBF_TWEENING) k->tween = decl_has(de, D3DDECLUSAGE_POSITION, 1) != 0;
            else if (vb >= D3DVBF_1WEIGHTS && vb <= D3DVBF_3WEIGHTS && w) k->blend = (uint8_t)(vb + 1);
            else if (vb == D3DVBF_0WEIGHTS && ix) k->blend = 1;
            if (!k->blend) k->ixblend = k->ixcolor = 0;
        }
        if (s->rs[D3DRS_LIGHTING] && !k->xyzrhw) {
            k->lighting = 1;
            k->normalize = s->rs[D3DRS_NORMALIZENORMALS] != 0;
            k->local = s->rs[D3DRS_LOCALVIEWER] != 0;
            k->spec = s->rs[D3DRS_SPECULARENABLE] != 0;
            k->cvertex = s->rs[D3DRS_COLORVERTEX] != 0;
            k->dsrc = mcs(d, D3DRS_DIFFUSEMATERIALSOURCE, k->c0, k->c1);
            k->asrc = mcs(d, D3DRS_AMBIENTMATERIALSOURCE, k->c0, k->c1);
            k->ssrc = mcs(d, D3DRS_SPECULARMATERIALSOURCE, k->c0, k->c1);
            k->esrc = mcs(d, D3DRS_EMISSIVEMATERIALSOURCE, k->c0, k->c1);
            for (i = 0; i < MAX_LIGHTS; i++) if (s->light_on[i]) k->light[i] = (uint8_t)s->light[i].Type;
        }
        k->spec = s->rs[D3DRS_SPECULARENABLE] != 0;
        if (k->xyzrhw) k->clip = 0;
        if (k->c0 && de->el[decl_has(de, D3DDECLUSAGE_COLOR, 0) - 1].Type == D3DDECLTYPE_D3DCOLOR) k->rb |= 1;
        if (k->c1 && de->el[decl_has(de, D3DDECLUSAGE_COLOR, 1) - 1].Type == D3DDECLTYPE_D3DCOLOR) k->rb |= 2;
        if (!k->c0 && !k->c1 && !k->lighting) k->flat = 0;      /* nothing that varies is a colour */
    } else {
        for (i = 0; i < vs->nin && i < 32; i++) {
            int e = decl_has(de, vs->in[i].usage, vs->in[i].index);
            if (e && de->el[e - 1].Type == D3DDECLTYPE_D3DCOLOR) k->rb |= 1u << i;
        }
        if (!strstr(vs->decl, "varying vec4 v_c")) k->flat = 0;
        for (i = 0; i < 16; i++) if (vs->loop_i >> i & 1) { k->li[0][i][0] = (short)s->vi[i][0]; k->li[0][i][1] = (short)s->vi[i][1]; k->li[0][i][2] = (short)s->vi[i][2]; }
    }
    if (ps) for (i = 0; i < 16; i++) if (ps->loop_i >> i & 1) { k->li[1][i][0] = (short)s->pi[i][0]; k->li[1][i][1] = (short)s->pi[i][1]; k->li[1][i][2] = (short)s->pi[i][2]; }
    k->srgbw = s->rs[D3DRS_SRGBWRITEENABLE] != 0;
    k->zrev = (uint8_t)d3_zrev(d);
    for (i = 0; i < 8; i++) {
        const d3tex *x = s->tex[i];
        if (x) {
            const DWORD *ss = s->ss[i];
            k->ssign[i] = (uint8_t)d3_fmt_signed(x->fmt);
            k->sdepth[i] = (uint8_t)x->depth;
            if (x->depth == 2) k->slin[i] = ss[D3DSAMP_MAGFILTER] != D3DTEXF_POINT || ss[D3DSAMP_MINFILTER] != D3DTEXF_POINT;
            k->ssrgb[i] = ss[D3DSAMP_SRGBTEXTURE] != 0;
            k->sbias[i] = ss[D3DSAMP_MIPMAPLODBIAS] != 0;
            if (!x->cube) {                                     /* cube maps: the coordinates never leave a face */
                k->sau[i] = ss[D3DSAMP_ADDRESSU] == D3DTADDRESS_MIRRORONCE ? 1 : ss[D3DSAMP_ADDRESSU] == D3DTADDRESS_BORDER ? 2 : 0;
                k->sav[i] = ss[D3DSAMP_ADDRESSV] == D3DTADDRESS_MIRRORONCE ? 1 : ss[D3DSAMP_ADDRESSV] == D3DTADDRESS_BORDER ? 2 : 0;
                k->svol[i] = (uint8_t)(ps ? ps->samp_kind[i] == 4 || (ps->samp_kind[i] == 1 && x->vol) : x->vol);     /* the coordinate the program has, whatever is bound */
                if (k->svol[i]) k->saw[i] = ss[D3DSAMP_ADDRESSW] == D3DTADDRESS_MIRRORONCE ? 1 : ss[D3DSAMP_ADDRESSW] == D3DTADDRESS_BORDER ? 2 : 0;
                if (k->sau[i] == 2 || k->sav[i] == 2 || k->saw[i] == 2) {
                    k->slin[i] = ss[D3DSAMP_MAGFILTER] != D3DTEXF_POINT || ss[D3DSAMP_MINFILTER] != D3DTEXF_POINT;
                    k->smip[i] = x->hwl > 1 && ss[D3DSAMP_MIPFILTER] != D3DTEXF_NONE;
                }
            }
        }
        if (ps && ps->samp_kind[i] == 1) {
            DWORD ttf = s->tss[i][D3DTSS_TEXTURETRANSFORMFLAGS];
            k->stex[i] = x && x->cube ? 3 : x && x->vol ? 4 : 2;
            if (ttf & D3DTTFF_PROJECTED) k->sproj[i] = ttf & 7 ? (uint8_t)(ttf & 7) : 4;
        }
    }
    for (i = 0; i < MAX_STAGES; i++) {
        DWORD cop = s->tss[i][D3DTSS_COLOROP];
        DWORD tci = s->tss[i][D3DTSS_TEXCOORDINDEX], ttf = s->tss[i][D3DTSS_TEXTURETRANSFORMFLAGS];
        int use = 0, dim = 0;
        if (!ps) {
            if (cop == D3DTOP_DISABLE) break;
            k->st[i].cop = (uint8_t)cop; k->st[i].ca0 = (uint8_t)s->tss[i][D3DTSS_COLORARG0]; k->st[i].ca1 = (uint8_t)s->tss[i][D3DTSS_COLORARG1]; k->st[i].ca2 = (uint8_t)s->tss[i][D3DTSS_COLORARG2];
            k->st[i].aop = (uint8_t)s->tss[i][D3DTSS_ALPHAOP]; k->st[i].aa0 = (uint8_t)s->tss[i][D3DTSS_ALPHAARG0]; k->st[i].aa1 = (uint8_t)s->tss[i][D3DTSS_ALPHAARG1]; k->st[i].aa2 = (uint8_t)s->tss[i][D3DTSS_ALPHAARG2];
            k->st[i].res = (uint8_t)s->tss[i][D3DTSS_RESULTARG];
            if (s->tex[i]) { k->st[i].tex = s->tex[i]->cube ? 3 : s->tex[i]->vol ? 4 : 2; use = 1; dim = k->st[i].tex; }
        } else {
            int j;
            for (j = 0; j < ps->nin; j++) if (ps->in[j].usage == D3DDECLUSAGE_TEXCOORD && ps->in[j].index == i) use = 1;
        }
        if (use && !vs) {
            tc_key *t = &k->tc[i];
            int ci = (int)(tci & 7), e = decl_has(de, D3DDECLUSAGE_TEXCOORD, ci);
            t->on = 1; t->ci = (uint8_t)ci; t->gen = (uint8_t)(tci >> 16); t->dim = (uint8_t)dim;
            t->ttf = (uint8_t)(ttf & 7);
            if (t->ttf == 1 || k->xyzrhw) t->ttf = 0;               /* one coordinate, or a vertex already transformed: no matrix */
            t->isz = e ? (uint8_t)elem_comps(de->el[e - 1].Type) : 4;
            if (!e && !t->gen) t->gen = 0xFF;                       /* no such coordinates: zero */
        }
        if (use && (ttf & D3DTTFF_PROJECTED)) k->tc[i].proj = (ttf & 7) == 2 || (ttf & 7) == 3 ? (uint8_t)(ttf & 7) : 4;    /* the last coordinate the matrix gave */
    }
}

/* ---------------------------------------------------------------- the vertex stage */
static const char *g_tail = "    gl_Position = vec4(g2p.x + u_vp.x * g2p.w, u_vp.y * g2p.w - g2p.y, 2.0 * g2p.z - g2p.w, g2p.w);\n";
/* Depth kept reversed (d3_zrev): g2z = z' = w - z, 0 at the far plane and w at the near one, goes to board A as it is — VS_STATE's z01 says the range is 0 .. w. It is never the
   difference of the program's own z and w where that can be avoided: fixed function gets a matrix whose z column is already w - z, a shader of the usual form a row c[w] - c[z] (G2_ZR) */
static const char *g_tail_rev = "    gl_Position = vec4(g2p.x + u_vp.x * g2p.w, u_vp.y * g2p.w - g2p.y, g2z, g2p.w);\n";

/* the position and normal of a vertex in eye space, from attributes a_* or a primitive's first vertex f_*: ep, the eye position, and N. The position before any matrix is ep_p */
static void ff_xform(d3sb *b, const pkey *k, const char *pre, const char *ep, const char *nv, int eye)
{
    int i;
    if (k->tween) d3sb_printf(b, "    vec4 %s_p = mix(%s_u0_0, %s_u0_1, u_tween.x);\n", ep, pre, pre);
    else d3sb_printf(b, "    vec4 %s_p = %s_u0_0;\n", ep, pre);
    if (k->normal) {
        if (k->tween) d3sb_printf(b, "    vec3 %s_n = mix(%s_u3_0, %s_u3_1, u_tween.x);\n", nv, pre, pre);
        else d3sb_printf(b, "    vec3 %s_n = %s_u3_0;\n", nv, pre);
    }
    if (!eye && !k->blend) return;
    if (!k->blend) {
        d3sb_printf(b, "    vec4 %s = u_wv * %s_p;\n", ep, ep);
        if (k->normal) d3sb_printf(b, "    vec3 %s = u_norm * %s_n;\n", nv, nv);
        else d3sb_printf(b, "    vec3 %s = vec3(0.0);\n", nv);
    } else {
        d3sb_printf(b, "    vec4 %s = vec4(0.0);\n    vec3 %s = vec3(0.0);\n", ep, nv);
        d3sb_printf(b, "    {\n        float wl = 1.0;\n");
        if (k->ixblend) d3sb_printf(b, "        vec4 ix = %s_u2_0%s;\n", pre, k->ixcolor ? ".zyxw * 255.001" : "");
        for (i = 0; i < k->blend; i++) {
            char at[16];
            if (k->ixblend) snprintf(at, sizeof at, "int(ix.%c)", "xyzw"[i]);
            else snprintf(at, sizeof at, "%d", i);
            if (i + 1 < k->blend) d3sb_printf(b, "        float w%d = %s_u1_0.%c;\n        wl -= w%d;\n", i, pre, "xyzw"[i], i);
            else d3sb_printf(b, "        float w%d = wl;\n", i);
            d3sb_printf(b, "        %s += w%d * (u_wm[%s] * %s_p);\n", ep, i, at, ep);
            if (k->normal) d3sb_printf(b, "        %s += w%d * (u_nm[%s] * %s_n);\n", nv, i, at, nv);
        }
        d3sb_printf(b, "    }\n");
    }
    if (k->normalize) d3sb_printf(b, "    %s = normalize(%s);\n", nv, nv);
}

static const char *src_of(int mcs_v, int field)
{
    static const char *const M[4] = { "u_matd", "u_mata", "u_mats", "u_mate" };
    return mcs_v == D3DMCS_COLOR1 ? "col0" : mcs_v == D3DMCS_COLOR2 ? "col1" : M[field];
}

static void ff_vs(d3sb *b, const pkey *k)
{
    int i, any_light = 0, need_eye, lit_first;
    for (i = 0; i < MAX_LIGHTS; i++) if (k->light[i]) any_light = 1;
    need_eye = any_light || (k->fog && k->vfog && !k->tfog) || k->clip || k->pscale;
    for (i = 0; i < MAX_STAGES; i++) if (k->tc[i].on && k->tc[i].gen >= 1 && k->tc[i].gen <= 4) need_eye = 1;
    lit_first = k->flat && any_light;                           /* flat shading lights the primitive's first vertex, on every vertex of the primitive */
    d3sb_printf(b, "precision highp float;\nattribute vec4 a_u0_0;\nuniform vec4 u_vp;\nvarying vec4 v_c0;\nvarying vec4 v_c1;\n");
    if (k->normal) d3sb_printf(b, "attribute vec3 a_u3_0;\n");
    if (k->c0) d3sb_printf(b, "attribute vec4 a_u10_0;\n");
    if (k->c1) d3sb_printf(b, "attribute vec4 a_u10_1;\n");
    if (k->flat && k->c0) d3sb_printf(b, "attribute vec4 f_u10_0;\n");
    if (k->flat && k->c1) d3sb_printf(b, "attribute vec4 f_u10_1;\n");
    if (k->blend > 1) d3sb_printf(b, "attribute vec4 a_u1_0;\n");
    if (k->ixblend) d3sb_printf(b, "attribute vec4 a_u2_0;\n");
    if (k->tween) d3sb_printf(b, "attribute vec4 a_u0_1;\nuniform vec4 u_tween;\n");
    if (k->tween && k->normal) d3sb_printf(b, "attribute vec3 a_u3_1;\n");
    if (lit_first) {
        d3sb_printf(b, "attribute vec4 f_u0_0;\n");
        if (k->normal) d3sb_printf(b, "attribute vec3 f_u3_0;\n");
        if (k->blend > 1) d3sb_printf(b, "attribute vec4 f_u1_0;\n");
        if (k->ixblend) d3sb_printf(b, "attribute vec4 f_u2_0;\n");
        if (k->tween) d3sb_printf(b, "attribute vec4 f_u0_1;\n");
        if (k->tween && k->normal) d3sb_printf(b, "attribute vec3 f_u3_1;\n");
    }
    if (k->psize && k->points) d3sb_printf(b, "attribute float a_u4_0;\n");
    if (k->xyzrhw) d3sb_printf(b, "uniform vec4 u_rt;\n");
    else if (k->blend) d3sb_printf(b, "uniform mat4 u_proj;\n");
    else d3sb_printf(b, "uniform mat4 u_wvp;\n");
    if (need_eye && !k->blend) d3sb_printf(b, "uniform mat4 u_wv;\n");
    if (need_eye && !k->blend && (k->normal || any_light)) d3sb_printf(b, "uniform mat3 u_norm;\n");
    if (k->blend) {
        d3sb_printf(b, "uniform mat4 u_wm[%d];\n", k->ixblend ? MAX_WM : k->blend);
        if (k->normal) d3sb_printf(b, "uniform mat3 u_nm[%d];\n", k->ixblend ? MAX_WM : k->blend);
    }
    if (k->points) d3sb_printf(b, "uniform vec4 u_psize;\nuniform vec4 u_pscale;\n");
    if (k->fog) d3sb_printf(b, "uniform vec4 u_fog;\nvarying float v_fog;\n");
    if (any_light) d3sb_printf(b, "uniform vec4 u_gamb;\nuniform vec4 u_matd;\nuniform vec4 u_mata;\nuniform vec4 u_mats;\nuniform vec4 u_mate;\nuniform vec4 u_matp;\n");
    for (i = 0; i < MAX_LIGHTS; i++) if (k->light[i]) d3sb_printf(b, "uniform vec4 l%d_pos;\nuniform vec4 l%d_dir;\nuniform vec4 l%d_diff;\nuniform vec4 l%d_spec;\nuniform vec4 l%d_amb;\nuniform vec4 l%d_att;\nuniform vec4 l%d_spot;\n", i, i, i, i, i, i, i);
    for (i = 0; i < MAX_STAGES; i++) {
        const tc_key *t = &k->tc[i];
        if (!t->on) continue;
        d3sb_printf(b, "varying vec4 v_t%d;\n", i);
        if (t->ttf) d3sb_printf(b, "uniform mat4 u_tm%d;\n", i);
        if (!t->gen) d3sb_printf(b, "attribute vec4 a_u5_%d;\n", t->ci);
    }
    for (i = 0; i < MAX_CLIP; i++) if (k->clip >> i & 1) d3sb_printf(b, "uniform vec4 u_clip%d;\nvarying float v_cd%d;\n", i, i);
    d3sb_printf(b, "void main() {\n    vec4 g2p;\n");
    d3sb_printf(b, "    vec4 col0 = %s%s;\n", !k->c0 ? "vec4(1.0)" : k->flat ? "f_u10_0" : "a_u10_0", k->c0 && k->rb & 1 ? ".zyxw" : "");
    d3sb_printf(b, "    vec4 col1 = %s%s;\n", !k->c1 ? "vec4(0.0)" : k->flat ? "f_u10_1" : "a_u10_1", k->c1 && k->rb & 2 ? ".zyxw" : "");
    d3sb_printf(b, "    float spa = %s;\n", k->c1 ? "a_u10_1.w" : "0.0");   /* specular alpha, the vertex fog of pretransformed vertices, is interpolated when colours are flat */
    if (k->xyzrhw) {
        d3sb_printf(b, "    float w = 1.0 / a_u0_0.w;\n    g2p = vec4((2.0 * a_u0_0.x * u_rt.x - 1.0) * w, (1.0 - 2.0 * a_u0_0.y * u_rt.y) * w, %s * w, w);\n", k->zrev ? "(1.0 - a_u0_0.z)" : "a_u0_0.z");
        if (need_eye) d3sb_printf(b, "    vec4 ep = vec4(0.0, 0.0, 0.0, 1.0);\n    vec3 N = vec3(0.0);\n");
    } else {
        ff_xform(b, k, "a", "ep", "N", need_eye);
        if (k->blend) d3sb_printf(b, "    g2p = u_proj * ep;\n");
        else d3sb_printf(b, "    g2p = u_wvp * ep_p;\n");
    }
    if (lit_first) ff_xform(b, k, "f", "lep", "lN", 1);
    else if (any_light) d3sb_printf(b, "    vec4 lep = ep;\n    vec3 lN = N;\n");
    if (any_light) {
        d3sb_printf(b, "    vec3 V = %s;\n    vec3 amb = vec3(0.0), dif = vec3(0.0), spe = vec3(0.0);\n", k->local ? "normalize(-lep.xyz)" : "vec3(0.0, 0.0, -1.0)");
        for (i = 0; i < MAX_LIGHTS; i++) {
            if (!k->light[i]) continue;
            d3sb_printf(b, "    {\n        vec3 L; float at = 1.0;\n");
            if (k->light[i] == D3DLIGHT_DIRECTIONAL) {
                d3sb_printf(b, "        L = l%d_dir.xyz;\n", i);
            } else {
                d3sb_printf(b, "        vec3 D = l%d_pos.xyz - lep.xyz;\n        float dist = length(D);\n        L = D / dist;\n", i);
                d3sb_printf(b, "        at = dist > l%d_pos.w ? 0.0 : 1.0 / (l%d_att.x + l%d_att.y * dist + l%d_att.z * dist * dist);\n", i, i, i, i);
                if (k->light[i] == D3DLIGHT_SPOT) {
                    d3sb_printf(b, "        float rho = dot(L, l%d_dir.xyz);\n", i);
                    d3sb_printf(b, "        if (rho <= l%d_spot.y) at = 0.0;\n        else if (rho < l%d_spot.x) at = at * pow((rho - l%d_spot.y) / (l%d_spot.x - l%d_spot.y), l%d_spot.z);\n", i, i, i, i, i, i);
                }
            }
            d3sb_printf(b, "        float nl = max(dot(lN, L), 0.0);\n        amb += at * l%d_amb.rgb;\n        dif += at * nl * l%d_diff.rgb;\n", i, i);
            if (k->spec) d3sb_printf(b, "        if (nl > 0.0) spe += at * pow(max(dot(lN, normalize(L + V)), 0.0), u_matp.x) * l%d_spec.rgb;\n", i);
            d3sb_printf(b, "    }\n");
        }
        d3sb_printf(b, "    v_c0 = clamp(vec4(%s.rgb + %s.rgb * (u_gamb.rgb + amb) + %s.rgb * dif, %s.a), 0.0, 1.0);\n", src_of(k->esrc, 3), src_of(k->asrc, 1), src_of(k->dsrc, 0), src_of(k->dsrc, 0));
        d3sb_printf(b, "    v_c1 = clamp(vec4(%s.rgb * spe, spa), 0.0, 1.0);\n", src_of(k->ssrc, 2));
    } else {
        d3sb_printf(b, "    v_c0 = col0;\n    v_c1 = vec4(col1.rgb, spa);\n");
    }
    if (k->fog && !k->tfog) {
        if (!k->vfog || k->xyzrhw) {
            d3sb_printf(b, "    v_fog = spa;\n");
        } else {
            d3sb_printf(b, "    float fz = %s;\n", k->range ? "length(ep.xyz)" : "abs(ep.z)");
            if (k->vfog == D3DFOG_LINEAR) d3sb_printf(b, "    v_fog = clamp((u_fog.y - fz) * u_fog.w, 0.0, 1.0);\n");
            else if (k->vfog == D3DFOG_EXP) d3sb_printf(b, "    v_fog = clamp(exp(-u_fog.z * fz), 0.0, 1.0);\n");
            else d3sb_printf(b, "    v_fog = clamp(exp(-(u_fog.z * fz) * (u_fog.z * fz)), 0.0, 1.0);\n");
        }
    }
    for (i = 0; i < MAX_STAGES; i++) {
        const tc_key *t = &k->tc[i];
        if (!t->on) continue;
        switch (t->gen) {
        case 0xFF: d3sb_printf(b, "    vec4 tc%d = vec4(0.0, 0.0, 0.0, 1.0);\n", i); break;
        case 0:                                                 /* Direct3D's matrix sees 1 in the first coordinate the vertex does not have, not in the fourth: _31, _32 are a 2D translation */
            if (t->ttf && t->isz == 1) d3sb_printf(b, "    vec4 tc%d = vec4(a_u5_%d.x, 1.0, 0.0, 0.0);\n", i, t->ci);
            else if (t->ttf && t->isz == 2) d3sb_printf(b, "    vec4 tc%d = vec4(a_u5_%d.xy, 1.0, 0.0);\n", i, t->ci);
            else d3sb_printf(b, "    vec4 tc%d = a_u5_%d;\n", i, t->ci);
            break;
        case 1: d3sb_printf(b, "    vec4 tc%d = vec4(N, 1.0);\n", i); break;                                         /* D3DTSS_TCI_CAMERASPACENORMAL */
        case 2: d3sb_printf(b, "    vec4 tc%d = vec4(ep.xyz, 1.0);\n", i); break;                                    /* ...POSITION */
        case 3: d3sb_printf(b, "    vec4 tc%d = vec4(reflect(normalize(ep.xyz), N), 1.0);\n", i); break;             /* ...REFLECTIONVECTOR */
        default:                                                /* ...SPHEREMAP: the eye looks along +z, v runs down the map */
            d3sb_printf(b, "    vec3 r%d = reflect(normalize(ep.xyz), N);\n    float m%d = 2.0 * sqrt(r%d.x * r%d.x + r%d.y * r%d.y + (r%d.z - 1.0) * (r%d.z - 1.0));\n", i, i, i, i, i, i, i, i);
            d3sb_printf(b, "    vec4 tc%d = vec4(r%d.x / m%d + 0.5, 0.5 - r%d.y / m%d, 0.0, 1.0);\n", i, i, i, i, i);
            break;
        }
        if (t->ttf) d3sb_printf(b, "    v_t%d = u_tm%d * tc%d;\n", i, i, i);
        else d3sb_printf(b, "    v_t%d = tc%d;\n", i, i);
    }
    for (i = 0; i < MAX_CLIP; i++) if (k->clip >> i & 1) d3sb_printf(b, "    v_cd%d = dot(u_clip%d, ep);\n", i, i);
    if (k->points) {
        d3sb_printf(b, "    float g2s = %s;\n", k->psize ? "a_u4_0" : "u_psize.x");
        if (k->pscale) d3sb_printf(b, "    {\n        float de = length(ep.xyz);\n        g2s = g2s * u_psize.w * sqrt(1.0 / (u_pscale.x + u_pscale.y * de + u_pscale.z * de * de));\n    }\n");
        d3sb_printf(b, "    gl_PointSize = clamp(g2s, u_psize.y, u_psize.z) * u_pscale.w;\n");
    }
    if (k->zrev) d3sb_printf(b, "    float g2z = g2p.z;\n%s}\n", g_tail_rev);      /* u_wvp / u_proj come with the z column already w - z (d3_prog_uniforms) */
    else d3sb_printf(b, "%s}\n", g_tail);
}

/* ---------------------------------------------------------------- the pixel stage */
static void ta(d3sb *b, int stage, unsigned a, int alpha)
{
    char x[16];
    const char *s;
    switch (a & D3DTA_SELECTMASK) {
    case D3DTA_DIFFUSE: s = "v_c0"; break;
    case D3DTA_CURRENT: s = "cur"; break;
    case D3DTA_TEXTURE: snprintf(x, sizeof x, "t%d", stage); s = x; break;
    case D3DTA_TFACTOR: s = "u_tf"; break;
    case D3DTA_SPECULAR: s = "v_c1"; break;
    case D3DTA_TEMP: s = "tmp"; break;
    case D3DTA_CONSTANT: snprintf(x, sizeof x, "u_tk%d", stage); s = x; break;
    default: s = "vec4(1.0)"; break;
    }
    (void)alpha;
    if ((a & D3DTA_COMPLEMENT) && (a & D3DTA_ALPHAREPLICATE)) d3sb_printf(b, "vec4(1.0 - %s.a)", s);
    else if (a & D3DTA_COMPLEMENT) d3sb_printf(b, "(vec4(1.0) - %s)", s);
    else if (a & D3DTA_ALPHAREPLICATE) d3sb_printf(b, "vec4(%s.a)", s);
    else d3sb_printf(b, "%s", s);
}

/* cur op: `swz` is .rgb or .a; a1, a2, a0 have been declared as vec4 */
static void op_text(d3sb *b, int stage, int op, const char *swz, const char *res)
{
    const char *cl = "clamp(", *ce = ", 0.0, 1.0)";
    d3sb_printf(b, "        %s%s = %s", res, swz, cl);
#define A(s) "a" #s
    switch (op) {
    case D3DTOP_SELECTARG1: d3sb_printf(b, "a1%s", swz); break;
    case D3DTOP_SELECTARG2: d3sb_printf(b, "a2%s", swz); break;
    case D3DTOP_MODULATE: d3sb_printf(b, "a1%s * a2%s", swz, swz); break;
    case D3DTOP_MODULATE2X: d3sb_printf(b, "2.0 * a1%s * a2%s", swz, swz); break;
    case D3DTOP_MODULATE4X: d3sb_printf(b, "4.0 * a1%s * a2%s", swz, swz); break;
    case D3DTOP_ADD: d3sb_printf(b, "a1%s + a2%s", swz, swz); break;
    case D3DTOP_ADDSIGNED: d3sb_printf(b, "a1%s + a2%s - %s", swz, swz, swz[1] == 'a' ? "0.5" : "vec3(0.5)"); break;
    case D3DTOP_ADDSIGNED2X: d3sb_printf(b, "2.0 * (a1%s + a2%s - %s)", swz, swz, swz[1] == 'a' ? "0.5" : "vec3(0.5)"); break;
    case D3DTOP_SUBTRACT: d3sb_printf(b, "a1%s - a2%s", swz, swz); break;
    case D3DTOP_ADDSMOOTH: d3sb_printf(b, "a1%s + a2%s - a1%s * a2%s", swz, swz, swz, swz); break;
    case D3DTOP_BLENDDIFFUSEALPHA: d3sb_printf(b, "a1%s * v_c0.a + a2%s * (1.0 - v_c0.a)", swz, swz); break;
    case D3DTOP_BLENDTEXTUREALPHA: d3sb_printf(b, "a1%s * t%d.a + a2%s * (1.0 - t%d.a)", swz, stage, swz, stage); break;
    case D3DTOP_BLENDFACTORALPHA: d3sb_printf(b, "a1%s * u_tf.a + a2%s * (1.0 - u_tf.a)", swz, swz); break;
    case D3DTOP_BLENDCURRENTALPHA: d3sb_printf(b, "a1%s * cur.a + a2%s * (1.0 - cur.a)", swz, swz); break;
    case D3DTOP_BLENDTEXTUREALPHAPM: d3sb_printf(b, "a1%s + a2%s * (1.0 - t%d.a)", swz, swz, stage); break;
    case D3DTOP_MODULATEALPHA_ADDCOLOR: d3sb_printf(b, "a1.rgb + a1.a * a2.rgb"); break;
    case D3DTOP_MODULATECOLOR_ADDALPHA: d3sb_printf(b, "a1.rgb * a2.rgb + vec3(a1.a)"); break;
    case D3DTOP_MODULATEINVALPHA_ADDCOLOR: d3sb_printf(b, "(1.0 - a1.a) * a2.rgb + a1.rgb"); break;
    case D3DTOP_MODULATEINVCOLOR_ADDALPHA: d3sb_printf(b, "(vec3(1.0) - a1.rgb) * a2.rgb + vec3(a1.a)"); break;
    case D3DTOP_DOTPRODUCT3: d3sb_printf(b, "%s(4.0 * dot(a1.rgb - vec3(0.5), a2.rgb - vec3(0.5)))", swz[1] == 'a' ? "" : "vec3"); break;
    case D3DTOP_MULTIPLYADD: d3sb_printf(b, "a0%s + a1%s * a2%s", swz, swz, swz); break;
    case D3DTOP_LERP: d3sb_printf(b, "a0%s * a1%s + (%s - a0%s) * a2%s", swz, swz, swz[1] == 'a' ? "1.0" : "vec3(1.0)", swz, swz); break;
    case D3DTOP_PREMODULATE: d3sb_printf(b, "a1%s * t%d%s", swz, stage + 1, swz); break;        /* this stage with the next stage's texture, which ff_fs reads early */
    case D3DTOP_BUMPENVMAP: case D3DTOP_BUMPENVMAPLUMINANCE: d3sb_printf(b, "cur%s", swz); break;       /* what it does is to the next stage's coordinates */
    default: d3sb_printf(b, "a1%s", swz); break;
    }
    d3sb_printf(b, "%s;\n", ce);
}

static void fs_end(d3sb *b, const pkey *k)
{
    if (k->fog) {
        if (k->tfog) {
            d3sb_printf(b, "    float fz = %s;\n", k->wfog ? "1.0 / gl_FragCoord.w" : k->zrev ? "1.0 - gl_FragCoord.z" : "gl_FragCoord.z");
            if (k->tfog == D3DFOG_LINEAR) d3sb_printf(b, "    float ff = clamp((u_fog.y - fz) * u_fog.w, 0.0, 1.0);\n");
            else if (k->tfog == D3DFOG_EXP) d3sb_printf(b, "    float ff = clamp(exp(-u_fog.z * fz), 0.0, 1.0);\n");
            else d3sb_printf(b, "    float ff = clamp(exp(-(u_fog.z * fz) * (u_fog.z * fz)), 0.0, 1.0);\n");
        } else {
            d3sb_printf(b, "    float ff = clamp(v_fog, 0.0, 1.0);\n");
        }
        d3sb_printf(b, "    cur.rgb = mix(u_fogcol.rgb, cur.rgb, ff);\n");
    }
    if (k->afunc && k->afunc != D3DCMP_ALWAYS) {
        static const char *const CMP[9] = { "", "false", "<", "==", "<=", ">", "!=", ">=", "true" };
        if (k->afunc == D3DCMP_NEVER) d3sb_printf(b, "    discard;\n");
        else d3sb_printf(b, "    if (!(cur.a %s u_aref.x)) discard;\n", CMP[k->afunc]);
    }
    d3sb_printf(b, k->srgbw ? "    gl_FragColor = vec4(g2_tosrgb(cur.rgb), cur.a);\n}\n" : "    gl_FragColor = cur;\n}\n");    /* before blending: D3DPMISCCAPS_POSTBLENDSRGBCONVERT is not in the caps */
}

/* how far a coordinate has left the texture, as how much of a filter's footprint is still on it. The TMU has clamped to the edge, so what it read is the part inside */
static void border_axis(d3sb *b, const char *c, const char *n, int lin)
{
    d3sb_printf(b, "    {\n        float o = max(0.5 - %s * %s, %s * %s - %s + 0.5);\n", c, n, c, n, n);
    if (lin) d3sb_printf(b, "        inside = inside * (1.0 - clamp(o, 0.0, 1.0));\n");
    else d3sb_printf(b, "        if (o > 0.5) inside = 0.0;\n");
    d3sb_printf(b, "    }\n");
}

/* per sampler, what the TMU has no state for. G2S<n>: a sample as the program reads it: signed channels are stored biased by 128, sRGB becomes linear after the filter (Direct3D 9 leaves the order open).
   G2C<n>: the coordinate: MIRRORONCE folds it and the TMU clamps. G2E<n>: BORDER: the TMU clamps and the program mixes in the colour. G2B<n>, G2A<n>: the bias as a last argument, as a term of .w */
static void fs_macros(d3sb *b, const pkey *k)
{
    int i, rg = 0, all = 0, lin = 0;
    for (i = 0; i < 8; i++) { if (k->ssign[i] == 1) rg = 1; if (k->ssign[i] == 2) all = 1; if (k->ssrgb[i]) lin = 1; }
    if (rg) d3sb_printf(b, "vec4 g2_srg(vec4 x) { return vec4((x.xy * 255.0 - vec2(128.0)) / 127.0, x.zw); }\n");
    if (all) d3sb_printf(b, "vec4 g2_srgba(vec4 x) { return (x * 255.0 - vec4(128.0)) / 127.0; }\n");
    if (lin) d3sb_printf(b, "vec4 g2_lin(vec4 x) { vec3 c = clamp(x.rgb, 0.0, 1.0); return vec4(mix(c / 12.92, pow((c + vec3(0.055)) / 1.055, vec3(2.4)), step(vec3(0.04045), c)), x.a); }\n");
    if (k->srgbw) d3sb_printf(b, "vec3 g2_tosrgb(vec3 x) { vec3 c = clamp(x, 0.0, 1.0); return mix(c * 12.92, 1.055 * pow(c, vec3(1.0 / 2.4)) - vec3(0.055), step(vec3(0.0031308), c)); }\n");
    for (i = 0; i < 8; i++) if (k->sdepth[i]) { d3sb_printf(b, "float g2_dz(vec4 x) { return min(dot(x.gba, vec3(0.99609375, 0.003891, 0.000016)), 1.0); }\n"); break; }
    for (i = 0; i < 8; i++) {
        const char *ty = k->svol[i] ? "vec3" : "vec2";
        d3sb_printf(b, "#define G2S%d(x) %s\n", i, k->sdepth[i] == 1 ? "vec4(g2_dz(x))" : k->ssign[i] == 1 ? "g2_srg(x)" : k->ssign[i] == 2 ? "g2_srgba(x)" : k->ssrgb[i] ? "g2_lin(x)" : "x");
        /* D24S8 under a sampler: the comparison with the coordinate's z, and four of them weighed for a linear filter, as hardware shadow maps do */
        if (k->sdepth[i] == 2) {
            if (!(k->sau[i] == 2 || k->sav[i] == 2 || k->saw[i] == 2)) d3sb_printf(b, "uniform vec4 u_ts%d;\n", i);
            d3sb_printf(b, "float g2_sc%d(sampler2D s, vec2 at, float z) { return z <= g2_dz(texture2DLod(s, at / u_ts%d.xy, 0.0)) ? 1.0 : 0.0; }\n", i, i);
            if (k->slin[i]) d3sb_printf(b, "vec4 g2_sh%d(sampler2D s, vec3 p) {\n    vec2 t = p.xy * u_ts%d.xy - vec2(0.5);\n    vec2 i = floor(t);\n    vec2 f = t - i;\n    vec2 c = i + vec2(0.5);\n    return vec4(mix(mix(g2_sc%d(s, c, p.z), g2_sc%d(s, c + vec2(1.0, 0.0), p.z), f.x), mix(g2_sc%d(s, c + vec2(0.0, 1.0), p.z), g2_sc%d(s, c + vec2(1.0, 1.0), p.z), f.x), f.y));\n}\n", i, i, i, i, i, i);
            else d3sb_printf(b, "vec4 g2_sh%d(sampler2D s, vec3 p) { return vec4(g2_sc%d(s, (floor(p.xy * u_ts%d.xy) + vec2(0.5)), p.z)); }\n", i, i, i);
            d3sb_printf(b, "#define G2X%d(t, c) g2_sh%d(s%d, c)\n", i, i, i);
        }
        else d3sb_printf(b, "#define G2X%d(t, c) t\n", i);
        if (k->sbias[i]) d3sb_printf(b, "uniform vec4 u_lb%d;\n#define G2B%d , u_lb%d.x\n#define G2A%d + u_lb%d.x\n", i, i, i, i, i);
        else d3sb_printf(b, "#define G2B%d\n#define G2A%d\n", i, i);
        if (k->sau[i] == 1 || k->sav[i] == 1 || k->saw[i] == 1) {
            d3sb_printf(b, "%s g2_c%d(%s c) {\n", ty, i, ty);
            if (k->sau[i] == 1) d3sb_printf(b, "    c.x = min(abs(c.x), 1.0);\n");
            if (k->sav[i] == 1) d3sb_printf(b, "    c.y = min(abs(c.y), 1.0);\n");
            if (k->saw[i] == 1) d3sb_printf(b, "    c.z = min(abs(c.z), 1.0);\n");
            d3sb_printf(b, "    return c;\n}\n#define G2C%d(c) g2_c%d(c)\n", i, i);
        }
        else d3sb_printf(b, "#define G2C%d(c) (c)\n", i);
        if (k->sau[i] == 2 || k->sav[i] == 2 || k->saw[i] == 2) {
            d3sb_printf(b, "uniform vec4 u_bc%d;\nuniform vec4 u_ts%d;\nvec4 g2_e%d(vec4 t, %s c) {\n    %s n = u_ts%d.%s;\n    float inside = 1.0;\n", i, i, i, ty, ty, i, k->svol[i] ? "xyz" : "xy");
            if (k->smip[i]) d3sb_printf(b, "    %s dx = dFdx(c) * n;\n    %s dy = dFdy(c) * n;\n    n = n * exp2(-max(0.5 * log2(max(max(dot(dx, dx), dot(dy, dy)), 1.0e-6)), 0.0));\n", ty, ty);     /* the texels of the level in use */
            if (k->sau[i] == 2) border_axis(b, "c.x", "n.x", k->slin[i]);
            if (k->sav[i] == 2) border_axis(b, "c.y", "n.y", k->slin[i]);
            if (k->saw[i] == 2) border_axis(b, "c.z", "n.z", k->slin[i]);
            d3sb_printf(b, "    return mix(u_bc%d, t, inside);\n}\n#define G2E%d(x, c) g2_e%d(x, c)\n", i, i, i);
        }
        else d3sb_printf(b, "#define G2E%d(x, c) x\n", i);
    }
}

static int stage_constant(const st_key *s)
{
    const unsigned a[6] = { s->ca0, s->ca1, s->ca2, s->aa0, s->aa1, s->aa2 };
    int i;
    for (i = 0; i < 6; i++) if ((a[i] & D3DTA_SELECTMASK) == D3DTA_CONSTANT) return 1;
    return 0;
}

static int op_bump(int op) { return op == D3DTOP_BUMPENVMAP || op == D3DTOP_BUMPENVMAPLUMINANCE; }

/* stage i's texture read: its coordinate, the sample as t<i> */
static void stage_sample(d3sb *b, const pkey *k, int i)
{
    const st_key *s = &k->st[i];
    int bumped = i && op_bump(k->st[i - 1].cop);
    if (s->tex) {
        if (k->sprite) d3sb_printf(b, "    vec4 c%d = vec4(gl_PointCoord.x, 1.0 - gl_PointCoord.y, 0.0, 1.0);\n", i);     /* window rows run the other way here: d3d9_internal.h */
        else d3sb_printf(b, "    vec4 c%d = v_t%d;\n", i, i);
        if (k->tc[i].proj > 1 && s->tex == 4) d3sb_printf(b, "    c%d.xyz = c%d.xyz / c%d.%c;\n", i, i, i, "xyzw"[k->tc[i].proj - 1]);
        else if (k->tc[i].proj > 1 && k->sdepth[i] == 2) d3sb_printf(b, "    c%d.xyz = c%d.xyz / c%d.%c;\n", i, i, i, "xyzw"[k->tc[i].proj - 1]);
        else if (k->tc[i].proj > 1 && s->tex != 3) d3sb_printf(b, "    c%d.xy = c%d.xy / c%d.%c;\n", i, i, i, "xyzw"[k->tc[i].proj - 1]);
        if (bumped) d3sb_printf(b, "    c%d.xy += vec2(pm%d.x * t%d.x + pm%d.z * t%d.y, pm%d.y * t%d.x + pm%d.w * t%d.y);\n", i, i - 1, i - 1, i - 1, i - 1, i - 1, i - 1, i - 1, i - 1);
    }
    if (s->tex == 3) d3sb_printf(b, "    vec4 t%d = G2S%d(textureCube(s%d, c%d.xyz G2B%d));\n", i, i, i, i, i);
    else if (s->tex == 4) d3sb_printf(b, "    vec4 t%d = G2S%d(G2E%d(texture3D(s%d, G2C%d(c%d.xyz) G2B%d), c%d.xyz));\n", i, i, i, i, i, i, i, i);
    else if (s->tex) d3sb_printf(b, "    vec4 t%d = G2S%d(G2X%d(G2E%d(texture2D(s%d, G2C%d(c%d.xy) G2B%d), c%d.xy), c%d.xyz));\n", i, i, i, i, i, i, i, i, i, i);
    else d3sb_printf(b, "    vec4 t%d = vec4(1.0);\n", i);
}

static void ff_fs(d3sb *b, const pkey *k)
{
    int i, n = 0;
    d3sb_printf(b, "precision highp float;\nvarying vec4 v_c0;\nvarying vec4 v_c1;\nuniform vec4 u_tf;\nuniform vec4 u_aref;\nuniform vec4 u_fog;\nuniform vec4 u_fogcol;\n");
    fs_macros(b, k);
    if (k->fog && !k->tfog) d3sb_printf(b, "varying float v_fog;\n");
    for (i = 0; i < MAX_STAGES && k->st[i].cop; i++) {
        if (stage_constant(&k->st[i])) d3sb_printf(b, "uniform vec4 u_tk%d;\n", i);
        if (op_bump(k->st[i].cop)) d3sb_printf(b, "uniform vec4 pm%d;\n", i);
        if (k->st[i].cop == D3DTOP_BUMPENVMAPLUMINANCE) d3sb_printf(b, "uniform vec4 pl%d;\n", i);
        if (!k->st[i].tex) continue;
        d3sb_printf(b, "varying vec4 v_t%d;\nuniform %s s%d;\n", i, k->st[i].tex == 3 ? "samplerCube" : k->st[i].tex == 4 ? "sampler3D" : "sampler2D", i);
    }
    for (i = 0; i < MAX_CLIP; i++) if (k->clip >> i & 1) d3sb_printf(b, "varying float v_cd%d;\n", i);
    d3sb_printf(b, "void main() {\n    vec4 cur = v_c0;\n    vec4 tmp = vec4(0.0);\n");
    for (i = 0; i < MAX_CLIP; i++) if (k->clip >> i & 1) d3sb_printf(b, "    if (v_cd%d < 0.0) discard;\n", i);
    for (i = 0; i < MAX_STAGES && k->st[i].cop; i++) {
        const st_key *s = &k->st[i];
        const char *res = (s->res & D3DTA_SELECTMASK) == D3DTA_TEMP ? "tmp" : "cur";
        int bumped = i && op_bump(k->st[i - 1].cop);
        n++;
        if (!i || k->st[i - 1].cop != D3DTOP_PREMODULATE) stage_sample(b, k, i);
        if (s->cop == D3DTOP_PREMODULATE) {                     /* the next stage's texture is an argument here */
            if (i + 1 < MAX_STAGES && k->st[i + 1].cop) stage_sample(b, k, i + 1);
            else d3sb_printf(b, "    vec4 t%d = vec4(1.0);\n", i + 1);
        }
        if (bumped && k->st[i - 1].cop == D3DTOP_BUMPENVMAPLUMINANCE) d3sb_printf(b, "    t%d.rgb = t%d.rgb * clamp(t%d.z * pl%d.x + pl%d.y, 0.0, 1.0);\n", i, i, i - 1, i - 1, i - 1);
        d3sb_printf(b, "    {\n        vec4 a0 = ");
        ta(b, i, s->ca0, 0);
        d3sb_printf(b, ";\n        vec4 a1 = ");
        ta(b, i, s->ca1, 0);
        d3sb_printf(b, ";\n        vec4 a2 = ");
        ta(b, i, s->ca2, 0);
        d3sb_printf(b, ";\n        vec3 rgb;\n");
        op_text(b, i, s->cop, ".rgb", "vec4 r_; r_");
        d3sb_printf(b, "        vec4 b0 = ");
        ta(b, i, s->aa0, 1);
        d3sb_printf(b, ";\n        vec4 b1 = ");
        ta(b, i, s->aa1, 1);
        d3sb_printf(b, ";\n        vec4 b2 = ");
        ta(b, i, s->aa2, 1);
        d3sb_printf(b, ";\n        a0 = b0; a1 = b1; a2 = b2;\n");
        if (s->aop && s->aop != D3DTOP_DISABLE) op_text(b, i, s->aop, ".a", "r_");
        else d3sb_printf(b, "        r_.a = cur.a;\n");
        d3sb_printf(b, "        %s = r_;\n    }\n", res);
    }
    (void)n;
    if (k->spec) d3sb_printf(b, "    cur.rgb = clamp(cur.rgb + v_c1.rgb, 0.0, 1.0);\n");
    fs_end(b, k);
}

/* ---------------------------------------------------------------- link */
static void add_slot(short *tab, int n, const char *s, char pre, const g2sc_sym *y)
{
    if (s[0] == pre && s[1] >= '0' && s[1] <= '9') { int i = atoi(s + 1); if (i < n) tab[i] = (short)y->loc; }
}

static void uni_table(d3prog *p, const g2sc_result *r, int fs)
{
    short *u = fs ? p->fu : p->vu;
    int i, k;
    for (i = 0; i < r->nuniforms; i++) {
        const g2sc_sym *y = &r->uniforms[i];
        const char *n = y->lname ? y->lname : y->name;
        for (k = 0; k < (int)(sizeof U_NAMES / sizeof U_NAMES[0]); k++) if (!strcmp(n, U_NAMES[k])) u[k] = (short)y->loc;
        if (n[0] == 'l' && n[1] >= '0' && n[1] <= '7' && n[2] == '_') {
            static const char *const F[7] = { "pos", "dir", "diff", "spec", "amb", "att", "spot" };
            for (k = 0; k < 7; k++) if (!strcmp(n + 3, F[k])) u[U_L0 + 8 * (n[1] - '0') + k] = (short)y->loc;
        }
        if (!strncmp(n, "u_tm", 4)) u[U_TM0 + atoi(n + 4)] = (short)y->loc;
        if (!strncmp(n, "u_tk", 4)) u[U_TK0 + atoi(n + 4)] = (short)y->loc;
        if (!strncmp(n, "u_clip", 6)) u[U_CLIP0 + atoi(n + 6)] = (short)y->loc;
        if (!strncmp(n, "u_lb", 4) && fs) p->lb_slot[atoi(n + 4)] = (short)y->loc;
        if (!strncmp(n, "u_bc", 4) && fs) p->bc_slot[atoi(n + 4)] = (short)y->loc;
        if (!strncmp(n, "u_ts", 4) && fs) p->ts_slot[atoi(n + 4)] = (short)y->loc;
        if (!strcmp(n, "u_wm")) u[U_WM] = (short)y->loc;
        if (!strcmp(n, "u_nm")) u[U_NM] = (short)y->loc;
        if (!fs && !strcmp(n, "u_zr")) p->zr_slot = (short)y->loc;
        if (fs) {
            if (n[0] == 'p') {
                if (n[1] == 'c') add_slot(p->pc_slot, PS_CONSTS, n + 1, 'c', y);
                if (n[1] == 'i') add_slot(p->pi_slot, 16, n + 1, 'i', y);
                if (n[1] == 'b') add_slot(p->pb_slot, 16, n + 1, 'b', y);
                if (n[1] == 'm') add_slot(p->pm_slot, MAX_STAGES, n + 1, 'm', y);
                if (n[1] == 'l') add_slot(p->pl_slot, MAX_STAGES, n + 1, 'l', y);
                if (n[1] == 'z') add_slot(p->pz_slot, MAX_STAGES, n + 1, 'z', y);
            }
        } else {
            if (n[0] == 'v') {
                if (n[1] == 'c') add_slot(p->vc_slot, VS_CONSTS, n + 1, 'c', y);
                if (n[1] == 'i') add_slot(p->vi_slot, 16, n + 1, 'i', y);
                if (n[1] == 'b') add_slot(p->vb_slot, 16, n + 1, 'b', y);
                if (n[1] == 'r') add_slot(p->vr_slot, MAX_REL, n + 1, 'r', y);
                if (!strcmp(n, "vca")) p->vca_slot = (short)y->loc;
            }
        }
    }
}

static int fs_reads(const char *sasm, int idx)
{
    const char *p = sasm;
    while ((p = strstr(p, "LDV")) != NULL) {                    /* LDV r1 3, or LDV.xy r1 3 */
        const char *q = strchr(p + 3, ' ');
        if (q) q = strchr(q + 1, ' ');
        if (q && atoi(q + 1) == idx) return 1;
        p += 3;
    }
    return 0;
}

static const char *sname(const g2sc_sym *y) { return y->lname ? y->lname : y->name; }

/* what rep and loop read: a macro per value, the compiler unrolls them */
static void loop_macros(d3sb *b, const d3shader *s, const short (*li)[3])
{
    int i;
    for (i = 0; i < 16; i++) if (s->loop_i >> i & 1) d3sb_printf(b, "#define G2I%dX %d\n#define G2I%dY %d\n#define G2I%dZ %d\n", i, li[i][0], i, li[i][1], i, li[i][2]);
}

/* a vertex shader's body reading the first vertex's inputs: a_u.. is f_u.. */
static char *first_vertex_text(const char *body)
{
    char *o = _strdup(body), *p = o;
    while (o && (p = strstr(p, "a_u")) != NULL) {
        if (p == o || !(p[-1] == '_' || (p[-1] >= '0' && p[-1] <= '9') || (p[-1] >= 'A' && p[-1] <= 'Z') || (p[-1] >= 'a' && p[-1] <= 'z'))) *p = 'f';
        p += 3;
    }
    return o;
}

/* ---- more than the hardware's 8 rows, 8 texture coordinates with two colours. A varying becomes a global written at the end of main, a smaller varying takes what is used of it, the pixel stage sees the vec4 again through a
 * macro; g2sc_link packs them by Appendix A.7. Only the coordinates shrink: Direct3D's colours and fog are as they are */
static void sb_sub(d3sb *s, const char *a, const char *b)
{
    char *q = strstr(s->p, a);
    size_t la = strlen(a), lb = strlen(b), n = strlen(s->p), at;
    char *o;
    if (!q) return;
    at = (size_t)(q - s->p);
    o = (char *)malloc(n + lb - la + 1);
    if (!o) return;
    memcpy(o, s->p, at); memcpy(o + at, b, lb); memcpy(o + at + lb, q + la, n - at - la + 1);
    free(s->p);
    s->p = o;
    s->n = n + lb - la; s->cap = s->n + 1;
}

static void tight_tc(d3sb *vb, d3sb *fb, int i, int n)
{
    static const char *const TY[4] = { "", "float", "vec2", "vec3" }, *const SW[4] = { "", ".x", ".xy", ".xyz" };
    char a[40], b[80], c[120], e[64], nm[8];
    if (n < 1 || n > 3) return;
    snprintf(nm, sizeof nm, "v_t%d", i);
    snprintf(a, sizeof a, "varying vec4 %s;\n", nm);
    if (!strstr(vb->p, a) || !strstr(fb->p, a)) return;
    snprintf(b, sizeof b, "vec4 %s;\nvarying %s g2c_%s;\n", nm, TY[n], nm);
    sb_sub(vb, a, b);
    vb->n = strlen(vb->p) - 2; vb->p[vb->n] = 0;                /* the closing brace of main, and back */
    snprintf(e, sizeof e, "    g2c_%s = %s%s;\n}\n", nm, nm, SW[n]);
    d3sb_printf(vb, "%s", e);
    snprintf(c, sizeof c, "varying %s g2c_%s;\n#define %s vec4(g2c_%s%s)\n", TY[n], nm, nm, nm, n == 3 ? ", 1.0" : n == 2 ? ", 0.0, 1.0" : ", 0.0, 0.0, 1.0");
    sb_sub(fb, a, c);
}

/* what coordinate i needs of its vec4 */
static int tc_n(const pkey *k, const d3shader *vs, const d3shader *ps, int i)
{
    int n = 4;
    if (ps && ps->tcomps[i]) n = ps->tcomps[i];
    else if (!ps) n = k->tc[i].proj ? 4 : k->st[i].tex == 2 ? 2 : k->st[i].tex ? 3 : 4;
    if (vs && vs->tcomps[i] && vs->tcomps[i] < n) n = vs->tcomps[i];
    return n;
}

static void tight_all(d3sb *vb, d3sb *fb, const pkey *k, const d3shader *vs, const d3shader *ps)
{
    int i;
    for (i = 0; i < 8; i++) tight_tc(vb, fb, i, tc_n(k, vs, ps, i));
}

/* do the varyings both stages declare, the coordinates tightened, take fewer rows by Appendix A.7 than a row each: two 2D coordinates in one row are one LDV on board B */
static int d3_packs(const d3sb *vb, const d3sb *fb, const pkey *k, const d3shader *vs, const d3shader *ps)
{
    static const char *const TY[4] = { "float", "vec2", "vec3", "vec4" };
    static const int RANK[4] = { 6, 5, 4, 2 };                 /* g2sc_pack_vars' ranks */
    int rank[32], rows[32], at[32], m = 0, r;
    const char *p = fb->p;
    if (k->sprite) return 0;                                    /* a sprite's coordinates are gl_PointCoord's macro */
    while (m < 32 && (p = strstr(p, "varying ")) != NULL) {
        char ty[8], nm[40], decl[64];
        int t, n, idx, len = 0;
        p += 8;
        if (sscanf(p, "%7s %39[A-Za-z0-9_]", ty, nm) != 2) continue;
        for (t = 0; t < 4 && strcmp(ty, TY[t]); t++) ;
        snprintf(decl, sizeof decl, "varying %s %s;", ty, nm);
        if (t == 4 || !strstr(vb->p, decl)) continue;
        n = t + 1;
        if (n == 4 && sscanf(nm, "v_t%d%n", &idx, &len) == 1 && !nm[len] && idx >= 0 && idx < 8) n = tc_n(k, vs, ps, idx);
        rank[m] = RANK[n - 1]; rows[m] = 1; m++;
    }
    for (r = 1; r < m; r++) if (g2sc_pack_vars(rank, rows, m, r, at) >= 0) return 1;
    return 0;
}

static d3prog *link_prog(d3dev *d, const pkey *k, const d3shader *vs, const d3shader *ps)
{
    d3sb vb, fb, ex, exw;
    g2sc_result rv, rf;
    const char *order[32];
    uint32_t fsmask = 0;
    int i, j, nslots = 0;
    d3prog *p = NULL;
    memset(&vb, 0, sizeof vb); memset(&fb, 0, sizeof fb); memset(&ex, 0, sizeof ex); memset(&exw, 0, sizeof exw);
    memset(&rv, 0, sizeof rv); memset(&rf, 0, sizeof rf);
    d3sb_printf(&fb, "#extension GL_OES_texture_3D : enable\n");
    if (ps) {
        d3sb_printf(&fb, "precision highp float;\nuniform vec4 u_aref;\nuniform vec4 u_fog;\nuniform vec4 u_fogcol;\n#define G2_FD(x) %s\n", k->zrev ? "(1.0 - (x))" : "(x)");
        fs_macros(&fb, k);
        loop_macros(&fb, ps, k->li[1]);
        for (i = 0; i < 8; i++) {                               /* ps 1.x says nothing of its samplers: the texture on the stage does */
            if (ps->samp_kind[i] != 1) continue;
            d3sb_printf(&fb, "uniform %s s%d;\n", k->stex[i] == 3 ? "samplerCube" : k->stex[i] == 4 ? "sampler3D" : "sampler2D", i);
            if (k->stex[i] == 3) d3sb_printf(&fb, "#define G2T%d(c) textureCube(s%d, (c).xyz G2B%d)\n", i, i, i);
            else if (k->stex[i] == 4 && k->sproj[i]) d3sb_printf(&fb, "#define G2T%d(c) G2E%d(texture3D(s%d, G2C%d((c).xyz / (c).%c) G2B%d), (c).xyz / (c).%c)\n", i, i, i, i, "xyzw"[k->sproj[i] - 1], i, "xyzw"[k->sproj[i] - 1]);
            else if (k->stex[i] == 4) d3sb_printf(&fb, "#define G2T%d(c) G2E%d(texture3D(s%d, G2C%d((c).xyz) G2B%d), (c).xyz)\n", i, i, i, i, i);
            else if (k->sproj[i]) d3sb_printf(&fb, "#define G2T%d(c) G2X%d(G2E%d(texture2D(s%d, G2C%d((c).xy / (c).%c) G2B%d), (c).xy / (c).%c), (c).xyz / (c).%c)\n", i, i, i, i, i, "xyzw"[k->sproj[i] - 1], i, "xyzw"[k->sproj[i] - 1], "xyzw"[k->sproj[i] - 1]);
            else d3sb_printf(&fb, "#define G2T%d(c) G2X%d(G2E%d(texture2D(s%d, G2C%d((c).xy) G2B%d), (c).xy), (c).xyz)\n", i, i, i, i, i, i);
        }
        d3sb_printf(&fb, "%s", ps->decl);
        if (k->fog && !k->tfog) d3sb_printf(&fb, "varying float v_fog;\n");
        for (i = 0; i < MAX_CLIP; i++) if (k->clip >> i & 1) d3sb_printf(&fb, "varying float v_cd%d;\n", i);
        if (k->sprite) for (i = 0; i < 8; i++) d3sb_printf(&fb, "#define v_t%d g2_pc\n", i);     /* after the declarations: every texture coordinate of a point sprite */
        d3sb_printf(&fb, "void main() {\n");
        if (k->sprite) d3sb_printf(&fb, "    vec4 g2_pc = vec4(gl_PointCoord.x, 1.0 - gl_PointCoord.y, 0.0, 1.0);\n");
        for (i = 0; i < MAX_CLIP; i++) if (k->clip >> i & 1) d3sb_printf(&fb, "    if (v_cd%d < 0.0) discard;\n", i);
        d3sb_printf(&fb, "%s    vec4 cur = g2_out;\n", ps->body);
        fs_end(&fb, k);
    } else {
        ff_fs(&fb, k);
    }
    /* what the pixel stage reads and no vertex stage writes takes Direct3D's default */
    if (g2sc_compile(fb.p, G2SC_FS, &rf) < 0) { g2e_log("d3d9: pixel stage does not compile: %s\n%s", rf.error, fb.p); goto done; }
    if (vs) {
        for (i = 0; i < rf.nvaryings; i++) {
            const char *n = sname(&rf.varyings[i]);
            int fog = !strcmp(n, "v_fog");
            char pat[48];
            snprintf(pat, sizeof pat, "varying %s %s;", fog ? "float" : "vec4", n);
            if (fs_reads(rf.sasm, rf.varyings[i].loc) && !strstr(vs->decl, pat)) {
                d3sb_printf(&ex, "%s\n", pat);
                d3sb_printf(&exw, fog ? "    %s = 1.0;\n" : "    %s = vec4(0.0, 0.0, 0.0, 1.0);\n", n);
            }
        }
        d3sb_printf(&vb, "precision highp float;\nuniform vec4 u_vp;\n");
        if (k->zrev && vs->pos_zk >= 0) d3sb_printf(&vb, "uniform vec4 u_zr;\n#define G2_ZR(x) g2z = dot(x, u_zr);\n");
        else d3sb_printf(&vb, "#define G2_ZR(x)\n");
        for (i = 0; i < vs->nin; i++) d3sb_printf(&vb, "#define SW_u%d_%d %s\n", vs->in[i].usage, vs->in[i].index, k->rb >> i & 1 ? ".zyxw" : "");
        loop_macros(&vb, vs, k->li[0]);
        if (k->points) d3sb_printf(&vb, "uniform vec4 u_psize;\nuniform vec4 u_pscale;\n#define G2_PSIZE u_psize.x\n#define G2_POINT(x) gl_PointSize = clamp(x, u_psize.y, u_psize.z) * u_pscale.w\n");
        else d3sb_printf(&vb, "#define G2_PSIZE 1.0\n#define G2_POINT(x)\n");
        for (i = 0; i < MAX_CLIP; i++) if (k->clip >> i & 1) d3sb_printf(&vb, "uniform vec4 u_clip%d;\nvarying float v_cd%d;\n", i, i);
        d3sb_printf(&vb, "%s%s", vs->decl, ex.p ? ex.p : "");
        if (k->flat) {                                          /* the shader runs on the primitive's first vertex for the colours, then on this vertex for the rest */
            int c0 = strstr(vs->decl, "varying vec4 v_c0;") != NULL, c1 = strstr(vs->decl, "varying vec4 v_c1;") != NULL;
            char *first = first_vertex_text(vs->body);
            if (!first) goto done;
            for (i = 0; i < vs->nin; i++) d3sb_printf(&vb, "attribute vec4 f_u%d_%d;\n", vs->in[i].usage, vs->in[i].index);
            d3sb_printf(&vb, "vec4 g2p;\n%svoid g2_first() {\n%s}\nvoid main() {\n    g2p = vec4(0.0, 0.0, 0.0, 1.0);\n%s    g2_first();\n", k->zrev ? "float g2z;\n" : "", first, exw.p ? exw.p : "");
            free(first);
            if (c0) d3sb_printf(&vb, "    vec4 g2_f0 = v_c0;\n");
            if (c1) d3sb_printf(&vb, "    vec4 g2_f1 = v_c1;\n");
            d3sb_printf(&vb, "    g2p = vec4(0.0, 0.0, 0.0, 1.0);\n%s", vs->body);
            if (c0) d3sb_printf(&vb, "    v_c0 = g2_f0;\n");
            if (c1) d3sb_printf(&vb, "    v_c1 = vec4(g2_f1.rgb, v_c1.a);\n");
        } else {
            d3sb_printf(&vb, "void main() {\n    vec4 g2p = vec4(0.0, 0.0, 0.0, 1.0);\n%s%s%s", k->zrev ? "    float g2z = 0.0;\n" : "", exw.p ? exw.p : "", vs->body);
        }
        for (i = 0; i < MAX_CLIP; i++) if (k->clip >> i & 1) d3sb_printf(&vb, "    v_cd%d = dot(u_clip%d, g2p);\n", i, i);
        if (k->zrev && vs->pos_zk < 0) d3sb_printf(&vb, "    g2z = g2p.w - g2p.z;\n");       /* a position this translator cannot follow: the difference of the two FP24 numbers, no finer than Direct3D's own count */
        d3sb_printf(&vb, "%s}\n", k->zrev ? g_tail_rev : g_tail);
    } else {
        ff_vs(&vb, k);
    }
    g2sc_free(&rf);
    memset(&rf, 0, sizeof rf);
    if (g2sc_compile(vb.p, G2SC_VS, &rv) < 0) { g2e_log("d3d9: vertex stage does not compile: %s\n%s", rv.error, vb.p); goto done; }
    memset(order, 0, sizeof order);
    for (i = 0; i < rv.nvaryings; i++) {
        if (rv.varyings[i].loc < 32) order[rv.varyings[i].loc] = sname(&rv.varyings[i]);
        if (rv.varyings[i].loc + 1 > nslots) nslots = rv.varyings[i].loc + 1;
    }
    if (nslots > 8 || d3_packs(&vb, &fb, k, vs, ps)) {
        g2sc_linked l;
        tight_all(&vb, &fb, k, vs, ps);
        g2sc_free(&rv);
        memset(&rv, 0, sizeof rv);
        if (g2sc_link(vb.p, fb.p, &l) < 0 || l.umap_vs || l.umap_fs) { g2e_log("d3d9: %d varyings, the hardware has 8 rows: %s", nslots, l.error); goto done; }
        rv = l.vs; rf = l.fs; fsmask = l.fsmask;
    } else {
        if (g2sc_compile_ex(fb.p, G2SC_FS, order, nslots, &rf) < 0) { g2e_log("d3d9: pixel stage does not compile: %s\n%s", rf.error, fb.p); goto done; }
        for (i = 0; i < rf.nvaryings; i++) if (rf.varyings[i].loc < 8 && fs_reads(rf.sasm, rf.varyings[i].loc)) fsmask |= 1u << rf.varyings[i].loc;
        if (((1u << nslots) - 1) & ~fsmask) {   /* what no pixel stage reads is not worth computing: a game's oFog with fog off */
            g2sc_result r2;
            memset(&r2, 0, sizeof r2);
            if (g2sc_compile_vs(vb.p, ((1u << nslots) - 1) & ~fsmask, &r2) == 0) { g2sc_free(&rv); rv = r2; }
            else g2sc_free(&r2);
        }
    }
    if (d3_trace() > 1) g2e_log("d3d9: program vs %u ps %u:\n%s\n%s", k->vs, k->ps, vb.p, fb.p);
    p = (d3prog *)calloc(1, sizeof *p);
    if (!p) goto done;
    memset(p->vu, 0xFF, sizeof p->vu); memset(p->fu, 0xFF, sizeof p->fu);
    memset(p->vc_slot, 0xFF, sizeof p->vc_slot); memset(p->pc_slot, 0xFF, sizeof p->pc_slot);
    memset(p->vi_slot, 0xFF, sizeof p->vi_slot); memset(p->pi_slot, 0xFF, sizeof p->pi_slot);
    memset(p->vb_slot, 0xFF, sizeof p->vb_slot); memset(p->pb_slot, 0xFF, sizeof p->pb_slot);
    memset(p->vr_slot, 0xFF, sizeof p->vr_slot);
    memset(p->pm_slot, 0xFF, sizeof p->pm_slot); memset(p->pl_slot, 0xFF, sizeof p->pl_slot); memset(p->pz_slot, 0xFF, sizeof p->pz_slot);
    memset(p->lb_slot, 0xFF, sizeof p->lb_slot); memset(p->bc_slot, 0xFF, sizeof p->bc_slot); memset(p->ts_slot, 0xFF, sizeof p->ts_slot);
    p->vca_slot = -1;
    p->zr_slot = -1;
    p->zrev = k->zrev;
    p->flat = k->flat;
    p->wm_n = k->blend ? (k->ixblend ? MAX_WM : k->blend) : 0;
    uni_table(p, &rv, 0);
    uni_table(p, &rf, 1);
    for (i = 0; i < rv.nattribs && p->nattr < 16; i++) {
        const char *n = sname(&rv.attribs[i]);
        d3attr *a = &p->attr[p->nattr];
        a->first = n[0] == 'f';
        if (sscanf(n + 1, "_u%d_%d", &a->usage, &a->index) == 2) { a->idx = rv.attribs[i].loc; p->nattr++; }
    }
    for (i = 0; i < rf.nsamplers && i < 8; i++) {
        const char *n = sname(&rf.samplers[i]);
        j = rf.samplers[i].loc;
        if (j < 8) {
            p->stage_of[j] = n[0] == 's' ? atoi(n + 1) : 0;
            p->cube_of[j] = !strcmp(rf.samplers[i].ltype ? rf.samplers[i].ltype : rf.samplers[i].type, "samplerCube");
            if (j + 1 > p->nsamp) p->nsamp = j + 1;
        }
    }
    p->vbase = rv.const_base; p->fbase = rf.const_base;
    p->p = g2r_prog_create(d->r, rv.words, (uint32_t)rv.nwords, (const double (*)[4])rv.const_pool, (uint32_t)rv.nconst, (uint32_t)rv.const_base, rf.words, (uint32_t)rf.nwords,
                           (const double (*)[4])rf.const_pool, (uint32_t)rf.nconst, (uint32_t)rf.const_base, fsmask);
    if (!p->p) { g2e_log("d3d9: no room for a program on the boards"); free(p); p = NULL; }
done:
    g2sc_free(&rv); g2sc_free(&rf);
    free(vb.p); free(fb.p); free(ex.p); free(exw.p);
    return p;
}

static uint64_t fnv(const void *d, size_t n)
{
    const uint8_t *p = (const uint8_t *)d;
    uint64_t h = 0xCBF29CE484222325ull;
    size_t i;
    for (i = 0; i < n; i++) h = (h ^ p[i]) * 0x100000001B3ull;
    return h;
}

d3prog *d3_ff_program(d3dev *d, const d3decl *decl, d3shader *vs, d3shader *ps, int mode)
{
    pkey k;
    uint64_t h;
    d3prog *p, *np;
    key_fill(d, &k, decl, vs, ps, mode);
    h = fnv(&k, sizeof k);
    for (p = d->progs[h & 1023]; p; p = p->next) if (p->hash == h && !memcmp(p->key, &k, sizeof k)) return p;
    np = link_prog(d, &k, vs, ps);
    if (!np) {          /* a program that cannot link is not retried each draw */
        np = (d3prog *)calloc(1, sizeof *np);
        if (!np) return NULL;
    }
    np->hash = h;
    np->key = malloc(sizeof k);
    if (np->key) memcpy(np->key, &k, sizeof k);
    np->key_size = sizeof k;
    np->next = d->progs[h & 1023];
    d->progs[h & 1023] = np;
    return np;
}

/* ---------------------------------------------------------------- what a draw fills in */
static void put(uint32_t *u, int slot, double a, double b, double c, double e)
{
    if (slot < 0) return;
    u[4 * slot] = g2_fp24(a); u[4 * slot + 1] = g2_fp24(b); u[4 * slot + 2] = g2_fp24(c); u[4 * slot + 3] = g2_fp24(e);
}
static void put_mat(uint32_t *u, int slot, const D3DMATRIX *m)
{
    int i;
    if (slot < 0) return;
    for (i = 0; i < 4; i++) put(u, slot + i, m->m[i][0], m->m[i][1], m->m[i][2], m->m[i][3]);       /* row-vector D3D, column vectors in GLSL: its rows are the columns */
}
static void put_color(uint32_t *u, int slot, const D3DCOLORVALUE *c) { put(u, slot, c->r, c->g, c->b, c->a); }

static void mul(D3DMATRIX *o, const D3DMATRIX *a, const D3DMATRIX *b)
{
    D3DMATRIX r;
    int i, j, k;
    for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) { float s = 0; for (k = 0; k < 4; k++) s += a->m[i][k] * b->m[k][j]; r.m[i][j] = s; }
    *o = r;
}

int d3_invert3(const D3DMATRIX *a, float o[3][3])
{
    float c00 = a->m[1][1] * a->m[2][2] - a->m[1][2] * a->m[2][1], c01 = a->m[1][2] * a->m[2][0] - a->m[1][0] * a->m[2][2], c02 = a->m[1][0] * a->m[2][1] - a->m[1][1] * a->m[2][0];
    float det = a->m[0][0] * c00 + a->m[0][1] * c01 + a->m[0][2] * c02, s;
    if (det == 0.0f) return -1;
    s = 1.0f / det;
    o[0][0] = c00 * s; o[1][0] = c01 * s; o[2][0] = c02 * s;
    o[0][1] = (a->m[0][2] * a->m[2][1] - a->m[0][1] * a->m[2][2]) * s; o[1][1] = (a->m[0][0] * a->m[2][2] - a->m[0][2] * a->m[2][0]) * s; o[2][1] = (a->m[0][1] * a->m[2][0] - a->m[0][0] * a->m[2][1]) * s;
    o[0][2] = (a->m[0][1] * a->m[1][2] - a->m[0][2] * a->m[1][1]) * s; o[1][2] = (a->m[0][2] * a->m[1][0] - a->m[0][0] * a->m[1][2]) * s; o[2][2] = (a->m[0][0] * a->m[1][1] - a->m[0][1] * a->m[1][0]) * s;
    return 0;
}

static void xform(const D3DMATRIX *m, const D3DVECTOR *v, float w, float *o)
{
    int i;
    for (i = 0; i < 3; i++) o[i] = v->x * m->m[0][i] + v->y * m->m[1][i] + v->z * m->m[2][i] + w * m->m[3][i];
}

/* a constant register as the shader sees it: what def put there wins over SetVertexShaderConstant */
static const float *vs_const(const d3state *s, int r)
{
    static const float zero[4];
    if (r < 0 || r >= VS_CONSTS) return zero;
    if (s->vs->def_mask[r / 32] >> (r % 32) & 1) return s->vs->defc + 4 * r;
    return s->vc[r];
}

static float rs_float(const d3state *s, D3DRENDERSTATETYPE r) { float f; memcpy(&f, &s->rs[r], 4); return f; }

/* the inverse transpose of a matrix's 3 x 3: what takes normals into eye space */
static void put_normal(uint32_t *u, int slot, const D3DMATRIX *wv)
{
    float n[3][3];
    int i;
    if (slot >= 0 && !d3_invert3(wv, n)) for (i = 0; i < 3; i++) put(u, slot + i, n[0][i], n[1][i], n[2][i], 0);
}

void d3_prog_uniforms(d3dev *d, d3prog *p, int w, int h, double ky)
{
    const d3state *s = &d->st;
    uint32_t *vu = d->vu, *fu = d->fu;
    D3DMATRIX wv, wvp;
    int i;
    memset(vu, 0, (size_t)p->vbase * 16);
    memset(fu, 0, (size_t)p->fbase * 16);
    mul(&wv, &s->world, &s->view);
    mul(&wvp, &wv, &s->proj);
    put_mat(vu, p->vu[U_WVP], &wvp);
    put_mat(vu, p->vu[U_WV], &wv);
    put_mat(vu, p->vu[U_PROJ], &s->proj);
    if (p->zrev) {      /* depth kept reversed: the z a program hands on is w - z. Worked out here in double from the matrices as the application gave them, so it is exact to their own rounding;
                           in the program it would be the difference of two FP24 numbers that agree in all but their last bits for anything far */
        if (p->vu[U_WVP] >= 0) {
            double a[4][4], zr;
            int j, k;
            for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) { double t = 0; for (k = 0; k < 4; k++) t += (double)s->world.m[i][k] * (double)s->view.m[k][j]; a[i][j] = t; }
            for (i = 0; i < 4; i++) {
                zr = 0;
                for (k = 0; k < 4; k++) zr += a[i][k] * ((double)s->proj.m[k][3] - (double)s->proj.m[k][2]);
                put(vu, p->vu[U_WVP] + i, wvp.m[i][0], wvp.m[i][1], zr, wvp.m[i][3]);
            }
        }
        if (p->vu[U_PROJ] >= 0) for (i = 0; i < 4; i++) put(vu, p->vu[U_PROJ] + i, s->proj.m[i][0], s->proj.m[i][1], (double)s->proj.m[i][3] - (double)s->proj.m[i][2], s->proj.m[i][3]);
    }
    put_normal(vu, p->vu[U_NORM], &wv);
    for (i = 0; i < p->wm_n; i++) {                             /* vertex blending: the first is D3DTS_WORLD */
        D3DMATRIX m;
        mul(&m, i ? &s->wm[i] : &s->world, &s->view);
        if (p->vu[U_WM] >= 0) put_mat(vu, p->vu[U_WM] + 4 * i, &m);
        if (p->vu[U_NM] >= 0) put_normal(vu, p->vu[U_NM] + 3 * i, &m);
    }
    put(vu, p->vu[U_TWEEN], rs_float(s, D3DRS_TWEENFACTOR), 0, 0, 0);
    if (p->vu[U_RT] >= 0) put(vu, p->vu[U_VP], 1.0 / w, 1.0 / h, 0, 0);    /* pretransformed: the whole target */
    else put(vu, p->vu[U_VP], s->vp.Width ? 1.0 / s->vp.Width : 0, s->vp.Height ? 1.0 / s->vp.Height : 0, 0, 0);
    put(vu, p->vu[U_RT], 1.0 / w, 1.0 / h, 0, 0);
    {
        DWORD g = s->rs[D3DRS_AMBIENT];
        put(vu, p->vu[U_GLOBAL_AMB], (g >> 16 & 255) / 255.0, (g >> 8 & 255) / 255.0, (g & 255) / 255.0, (g >> 24) / 255.0);
    }
    put_color(vu, p->vu[U_MAT_D], &s->mat.Diffuse);
    put_color(vu, p->vu[U_MAT_A], &s->mat.Ambient);
    put_color(vu, p->vu[U_MAT_S], &s->mat.Specular);
    put_color(vu, p->vu[U_MAT_E], &s->mat.Emissive);
    put(vu, p->vu[U_MAT_P], s->mat.Power, 0, 0, 0);
    {
        float st = rs_float(s, D3DRS_FOGSTART), en = rs_float(s, D3DRS_FOGEND), de = rs_float(s, D3DRS_FOGDENSITY), big = rs_float(s, D3DRS_POINTSIZE_MAX);
        double r = en != st ? 1.0 / (en - st) : 0.0;
        put(vu, p->vu[U_FOG], st, en, de, r);
        put(fu, p->fu[U_FOG], st, en, de, r);
        put(vu, p->vu[U_PSIZE], rs_float(s, D3DRS_POINTSIZE), rs_float(s, D3DRS_POINTSIZE_MIN), big < MAX_POINT ? big : MAX_POINT, s->vp.Height);
        put(vu, p->vu[U_PSCALE], rs_float(s, D3DRS_POINTSCALE_A), rs_float(s, D3DRS_POINTSCALE_B), rs_float(s, D3DRS_POINTSCALE_C), ky);
    }
    {
        DWORD t = s->rs[D3DRS_TEXTUREFACTOR], c = s->rs[D3DRS_FOGCOLOR];
        put(fu, p->fu[U_TF], (t >> 16 & 255) / 255.0, (t >> 8 & 255) / 255.0, (t & 255) / 255.0, (t >> 24) / 255.0);
        put(fu, p->fu[U_FOGCOL], (c >> 16 & 255) / 255.0, (c >> 8 & 255) / 255.0, (c & 255) / 255.0, 1.0);
        put(fu, p->fu[U_AREF], (s->rs[D3DRS_ALPHAREF] & 255) / 255.0, 0, 0, 0);
    }
    for (i = 0; i < MAX_LIGHTS; i++) {
        const D3DLIGHT9 *l = &s->light[i];
        int b = U_L0 + 8 * i;
        float v[3];
        if (p->vu[b + L_DIFF] < 0 && p->vu[b + L_POS] < 0 && p->vu[b + L_DIR] < 0) continue;
        xform(&s->view, &l->Position, 1.0f, v);
        put(vu, p->vu[b + L_POS], v[0], v[1], v[2], l->Range);
        xform(&s->view, &l->Direction, 0.0f, v);
        {
            double n = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            if (n == 0.0) n = 1.0;
            put(vu, p->vu[b + L_DIR], -v[0] / n, -v[1] / n, -v[2] / n, 0);
        }
        put_color(vu, p->vu[b + L_DIFF], &l->Diffuse);
        put_color(vu, p->vu[b + L_SPEC], &l->Specular);
        put_color(vu, p->vu[b + L_AMB], &l->Ambient);
        put(vu, p->vu[b + L_ATT], l->Attenuation0, l->Attenuation1, l->Attenuation2, 0);
        put(vu, p->vu[b + L_SPOT], cos(l->Theta / 2.0), cos(l->Phi / 2.0), l->Falloff, 0);
    }
    for (i = 0; i < MAX_STAGES; i++) put_mat(vu, p->vu[U_TM0 + i], &s->texm[i]);
    for (i = 0; i < MAX_STAGES; i++) {
        DWORD c = s->tss[i][D3DTSS_CONSTANT];
        float m[4], l[2];
        memcpy(m, &s->tss[i][D3DTSS_BUMPENVMAT00], 16);
        memcpy(&l[0], &s->tss[i][D3DTSS_BUMPENVLSCALE], 4); memcpy(&l[1], &s->tss[i][D3DTSS_BUMPENVLOFFSET], 4);
        put(fu, p->pm_slot[i], m[0], m[1], m[2], m[3]);
        put(fu, p->pl_slot[i], l[0], l[1], 0, 0);
        put(fu, p->fu[U_TK0 + i], (c >> 16 & 255) / 255.0, (c >> 8 & 255) / 255.0, (c & 255) / 255.0, (c >> 24) / 255.0);
        if (p->pz_slot[i] >= 0 && s->tex[i]) put(fu, p->pz_slot[i], s->tex[i]->w, s->tex[i]->h, s->tex[i]->d, 0);
        if (i < 8) {
            const DWORD *ss = s->ss[i];
            if (p->lb_slot[i] >= 0) { float lb; memcpy(&lb, &ss[D3DSAMP_MIPMAPLODBIAS], 4); put(fu, p->lb_slot[i], lb, 0, 0, 0); }
            if (p->bc_slot[i] >= 0) {
                DWORD bc = ss[D3DSAMP_BORDERCOLOR];
                put(fu, p->bc_slot[i], (bc >> 16 & 255) / 255.0, (bc >> 8 & 255) / 255.0, (bc & 255) / 255.0, (bc >> 24) / 255.0);
            }
            if (p->ts_slot[i] >= 0 && s->tex[i]) { int b = d3_tex_base(s->tex[i], ss); int w = s->tex[i]->w >> b, h = s->tex[i]->h >> b, e = s->tex[i]->d >> b; put(fu, p->ts_slot[i], w ? w : 1, h ? h : 1, e ? e : 1, 0); }
        }
    }
    for (i = 0; i < MAX_CLIP; i++) {                            /* a plane in world space, the varying in eye space; with a vertex shader Direct3D's plane is in clip space */
        if (p->vu[U_CLIP0 + i] >= 0 && s->vs) put(vu, p->vu[U_CLIP0 + i], s->clip[i][0], s->clip[i][1], s->clip[i][2], s->clip[i][3]);
        else if (p->vu[U_CLIP0 + i] >= 0) {
            const float *c = s->clip[i];
            D3DVECTOR n, q;
            float nn = c[0] * c[0] + c[1] * c[1] + c[2] * c[2], a[3], b[3];
            n.x = c[0]; n.y = c[1]; n.z = c[2];
            q.x = -c[3] * c[0] / nn; q.y = -c[3] * c[1] / nn; q.z = -c[3] * c[2] / nn;
            xform(&s->view, &n, 0.0f, a);
            xform(&s->view, &q, 1.0f, b);
            put(vu, p->vu[U_CLIP0 + i], a[0], a[1], a[2], -(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]));
        }
    }
    for (i = 0; i < VS_CONSTS; i++) if (p->vc_slot[i] >= 0) put(vu, p->vc_slot[i], s->vc[i][0], s->vc[i][1], s->vc[i][2], s->vc[i][3]);
    if (s->vs) {
        if (p->zr_slot >= 0) {                                  /* the row that gives z' = w - z of the shader's position: c[w] - c[z], in double */
            const float *cz = vs_const(s, s->vs->pos_zk), *cw = vs_const(s, s->vs->pos_wk);
            put(vu, p->zr_slot, (double)cw[0] - (double)cz[0], (double)cw[1] - (double)cz[1], (double)cw[2] - (double)cz[2], (double)cw[3] - (double)cz[3]);
        }
        for (i = 0; i < s->vs->nrel; i++) {
            const d3rel *r = &s->vs->rel[i];
            if (p->vr_slot[i] >= 0) {
                float a = vs_const(s, r->reg)[r->comp];
                const float *c = vs_const(s, (int)floor(r->round ? a + 0.5 : a) + r->base);
                put(vu, p->vr_slot[i], c[0], c[1], c[2], c[3]);
            }
        }
        if (p->vca_slot >= 0) {
            for (i = 0; i < VCA_N; i++) {
                const float *c = vs_const(s, i);
                put(vu, p->vca_slot + i, c[0], c[1], c[2], c[3]);
            }
        }
    }
    for (i = 0; i < PS_CONSTS; i++) if (p->pc_slot[i] >= 0) put(fu, p->pc_slot[i], s->pc[i][0], s->pc[i][1], s->pc[i][2], s->pc[i][3]);
    for (i = 0; i < 16; i++) {
        if (p->vi_slot[i] >= 0) put(vu, p->vi_slot[i], s->vi[i][0], s->vi[i][1], s->vi[i][2], s->vi[i][3]);
        if (p->pi_slot[i] >= 0) put(fu, p->pi_slot[i], s->pi[i][0], s->pi[i][1], s->pi[i][2], s->pi[i][3]);
        if (p->vb_slot[i] >= 0) put(vu, p->vb_slot[i], s->vb_c[i] != 0, 0, 0, 0);
        if (p->pb_slot[i] >= 0) put(fu, p->pb_slot[i], s->pb_c[i] != 0, 0, 0, 0);
    }
}
