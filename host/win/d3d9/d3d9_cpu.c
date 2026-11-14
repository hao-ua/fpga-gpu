/* d3d9_cpu.c — vertex processing on the PC, for what the boards cannot hand back: ProcessVertices writes its results into a buffer, and wireframe and point fill must know which triangles culling drops before they become lines.
 * Fixed function here says what d3d9_ff.c's programs say, in the same order. A vertex shader runs in d3d9_sm.c's interpreter. */
#include "d3d9_internal.h"
#include <math.h>

void d3_elem_read(const uint8_t *p, BYTE t, float *o)
{
    int i;
    o[0] = o[1] = o[2] = 0.0f; o[3] = 1.0f;
    switch (t) {
    case D3DDECLTYPE_FLOAT1: memcpy(o, p, 4); break;
    case D3DDECLTYPE_FLOAT2: memcpy(o, p, 8); break;
    case D3DDECLTYPE_FLOAT3: memcpy(o, p, 12); break;
    case D3DDECLTYPE_FLOAT4: memcpy(o, p, 16); break;
    case D3DDECLTYPE_D3DCOLOR: case D3DDECLTYPE_UBYTE4N: for (i = 0; i < 4; i++) o[i] = p[i] / 255.0f; break;
    case D3DDECLTYPE_UBYTE4: for (i = 0; i < 4; i++) o[i] = p[i]; break;
    case D3DDECLTYPE_SHORT2: for (i = 0; i < 2; i++) o[i] = ((const short *)p)[i]; break;
    case D3DDECLTYPE_SHORT4: for (i = 0; i < 4; i++) o[i] = ((const short *)p)[i]; break;
    case D3DDECLTYPE_SHORT2N: for (i = 0; i < 2; i++) o[i] = ((const short *)p)[i] / 32767.0f; break;
    case D3DDECLTYPE_SHORT4N: for (i = 0; i < 4; i++) o[i] = ((const short *)p)[i] / 32767.0f; break;
    case D3DDECLTYPE_USHORT2N: for (i = 0; i < 2; i++) o[i] = ((const WORD *)p)[i] / 65535.0f; break;
    case D3DDECLTYPE_USHORT4N: for (i = 0; i < 4; i++) o[i] = ((const WORD *)p)[i] / 65535.0f; break;
    default: break;
    }
}

const uint8_t *d3_stream_at(const d3dev *d, const D3DVERTEXELEMENT9 *e, const uint8_t *up, UINT up_stride, uint32_t v, int inst)
{
    const d3state *st = &d->st;
    const d3buf *b;
    if (up) return up + (size_t)v * up_stride + e->Offset;
    b = st->vb[e->Stream];
    if (!b) return NULL;
    if (inst >= 0 && (st->vb_freq[e->Stream] & D3DSTREAMSOURCE_INSTANCEDATA)) {
        UINT div = st->vb_freq[e->Stream] & 0x3FFFFFFFu;
        v = (uint32_t)inst / (div ? div : 1);
    }
    return b->mem + st->vb_off[e->Stream] + (size_t)v * st->vb_stride[e->Stream] + e->Offset;
}

static const D3DVERTEXELEMENT9 *elem(const d3decl *d, int usage, int index)
{
    int i;
    for (i = 0; i < d->n; i++) if (d->el[i].Usage == usage && d->el[i].UsageIndex == index) return &d->el[i];
    return NULL;
}

/* the source of one vertex */
typedef struct vsrc { const d3dev *d; const d3decl *de; const uint8_t *up; UINT stride; uint32_t v; int inst; } vsrc;

/* 0: the vertex has no such element, o has the default (0, 0, 0, 1). A colour comes back r, g, b, a */
static int load(const vsrc *s, int usage, int index, float *o)
{
    const D3DVERTEXELEMENT9 *e = elem(s->de, usage, index);
    const uint8_t *p = e ? d3_stream_at(s->d, e, s->up, s->stride, s->v, s->inst) : NULL;
    o[0] = o[1] = o[2] = 0.0f; o[3] = 1.0f;
    if (!p) return 0;
    d3_elem_read(p, e->Type, o);
    if (e->Type == D3DDECLTYPE_D3DCOLOR) { float t = o[0]; o[0] = o[2]; o[2] = t; }
    return 1;
}

static void row(const D3DMATRIX *m, const float *v, float *o)
{
    float t[4];
    int i;
    for (i = 0; i < 4; i++) t[i] = v[0] * m->m[0][i] + v[1] * m->m[1][i] + v[2] * m->m[2][i] + v[3] * m->m[3][i];
    memcpy(o, t, sizeof t);
}

static void mat_mul(D3DMATRIX *o, const D3DMATRIX *a, const D3DMATRIX *b)
{
    D3DMATRIX r;
    int i;
    for (i = 0; i < 4; i++) row(b, a->m[i], r.m[i]);
    *o = r;
}

static int blend_of(const d3state *s, const d3decl *de, int *indexed)
{
    DWORD vb = s->rs[D3DRS_VERTEXBLEND];
    *indexed = s->rs[D3DRS_INDEXEDVERTEXBLENDENABLE] && elem(de, D3DDECLUSAGE_BLENDINDICES, 0);
    if (vb >= D3DVBF_1WEIGHTS && vb <= D3DVBF_3WEIGHTS && elem(de, D3DDECLUSAGE_BLENDWEIGHT, 0)) return (int)vb + 1;
    if (vb == D3DVBF_0WEIGHTS && *indexed) return 1;
    return 0;
}

/* the position (and normal: 0 when the vertex has none) in eye space */
static int ff_eye(const vsrc *s, float *ep, float *nv)
{
    const d3state *st = &s->d->st;
    float pos[4], n[4], w[4], ix[4], sum = 0.0f;
    int has_n = load(s, D3DDECLUSAGE_NORMAL, 0, n), indexed, nm, i, j;
    load(s, D3DDECLUSAGE_POSITION, 0, pos);
    if (st->rs[D3DRS_VERTEXBLEND] == D3DVBF_TWEENING && elem(s->de, D3DDECLUSAGE_POSITION, 1)) {
        float tw, p2[4], n2[4];
        memcpy(&tw, &st->rs[D3DRS_TWEENFACTOR], 4);
        load(s, D3DDECLUSAGE_POSITION, 1, p2);
        for (i = 0; i < 4; i++) pos[i] += (p2[i] - pos[i]) * tw;
        if (has_n && load(s, D3DDECLUSAGE_NORMAL, 1, n2)) for (i = 0; i < 3; i++) n[i] += (n2[i] - n[i]) * tw;
    }
    nm = blend_of(st, s->de, &indexed);
    load(s, D3DDECLUSAGE_BLENDWEIGHT, 0, w);
    load(s, D3DDECLUSAGE_BLENDINDICES, 0, ix);
    if (indexed && elem(s->de, D3DDECLUSAGE_BLENDINDICES, 0)->Type == D3DDECLTYPE_D3DCOLOR) for (i = 0; i < 4; i++) ix[i] = ix[i] * 255.001f;
    memset(ep, 0, 16);
    if (nv) memset(nv, 0, 12);
    for (i = 0; i < (nm ? nm : 1); i++) {
        int k = !nm ? 0 : indexed ? (int)ix[i] : i;
        float a = !nm ? 1.0f : i + 1 < nm ? w[i] : 1.0f - sum, t[4], inv[3][3];
        D3DMATRIX wv;
        sum += a;
        k = k >= 0 && k < MAX_WM ? k : 0;
        mat_mul(&wv, k ? &st->wm[k] : &st->world, &st->view);
        row(&wv, pos, t);
        for (j = 0; j < 4; j++) ep[j] += a * t[j];
        if (nv && has_n && !d3_invert3(&wv, inv)) for (j = 0; j < 3; j++) nv[j] += a * (inv[j][0] * n[0] + inv[j][1] * n[1] + inv[j][2] * n[2]);
    }
    if (nv && has_n && st->rs[D3DRS_NORMALIZENORMALS]) {
        float l = sqrtf(nv[0] * nv[0] + nv[1] * nv[1] + nv[2] * nv[2]);
        if (l > 0.0f) for (j = 0; j < 3; j++) nv[j] /= l;
    }
    return has_n;
}

/* a vertex shader's inputs by register */
static void shader_in(const vsrc *s, const d3shader *sh, float (*in)[4])
{
    int i;
    for (i = 0; i < 32; i++) { in[i][0] = in[i][1] = in[i][2] = 0.0f; in[i][3] = 1.0f; }
    for (i = 0; i < sh->nin; i++) {
        int r = atoi(sh->in[i].name);
        if (r >= 0 && r < 32) load(s, sh->in[i].usage, sh->in[i].index, in[r]);
    }
}

int d3_cpu_position(d3dev *d, const uint8_t *up, UINT up_stride, uint32_t v, int inst, float *o)
{
    const d3state *st = &d->st;
    vsrc s;
    float t[4];
    s.d = d; s.de = st->decl; s.up = up; s.stride = up_stride; s.v = v; s.inst = inst;
    if (st->vs) {
        float in[32][4];
        d3vsout out;
        shader_in(&s, st->vs, in);
        if (d3_vs_cpu(st, st->vs, (const float (*)[4])in, &out) < 0) return -1;
        memcpy(o, out.pos, 16);
        return 0;
    }
    if (elem(st->decl, D3DDECLUSAGE_POSITIONT, 0)) {
        load(&s, D3DDECLUSAGE_POSITIONT, 0, t);
        o[0] = t[0]; o[1] = -t[1]; o[2] = 0.0f; o[3] = 1.0f;
        return 0;
    }
    ff_eye(&s, t, NULL);
    row(&st->proj, t, o);
    return 0;
}

static const float *mat_src(const d3state *st, DWORD rs, const float *m, const float *c0, const float *c1, int has0, int has1)
{
    DWORD v = st->rs[rs];
    if (!st->rs[D3DRS_COLORVERTEX]) return m;
    if (v == D3DMCS_COLOR1 && has0) return c0;
    if (v == D3DMCS_COLOR2 && has1) return c1;
    return m;
}

/* the colours fixed function lights a vertex with: ff_vs in d3d9_ff.c */
static void ff_light(const d3state *st, const float *ep, const float *nv, const float *c0, const float *c1, int has0, int has1, float *o0, float *o1)
{
    const D3DMATERIAL9 *m = &st->mat;
    const float *dm = mat_src(st, D3DRS_DIFFUSEMATERIALSOURCE, &m->Diffuse.r, c0, c1, has0, has1), *am = mat_src(st, D3DRS_AMBIENTMATERIALSOURCE, &m->Ambient.r, c0, c1, has0, has1);
    const float *sm = mat_src(st, D3DRS_SPECULARMATERIALSOURCE, &m->Specular.r, c0, c1, has0, has1), *em = mat_src(st, D3DRS_EMISSIVEMATERIALSOURCE, &m->Emissive.r, c0, c1, has0, has1);
    float amb[3] = { 0, 0, 0 }, dif[3] = { 0, 0, 0 }, spe[3] = { 0, 0, 0 }, v[3], gl[3];
    DWORD g = st->rs[D3DRS_AMBIENT];
    int i, j;
    gl[0] = (g >> 16 & 255) / 255.0f; gl[1] = (g >> 8 & 255) / 255.0f; gl[2] = (g & 255) / 255.0f;
    if (st->rs[D3DRS_LOCALVIEWER]) {
        float l = sqrtf(ep[0] * ep[0] + ep[1] * ep[1] + ep[2] * ep[2]);
        for (j = 0; j < 3; j++) v[j] = l > 0.0f ? -ep[j] / l : 0.0f;
    } else { v[0] = v[1] = 0.0f; v[2] = -1.0f; }
    for (i = 0; i < MAX_LIGHTS; i++) {
        const D3DLIGHT9 *l = &st->light[i];
        float dir[4], lv[3], at = 1.0f, nl, h[3], len;
        if (!st->light_on[i]) continue;
        dir[0] = l->Direction.x; dir[1] = l->Direction.y; dir[2] = l->Direction.z; dir[3] = 0.0f;
        row(&st->view, dir, dir);
        len = sqrtf(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
        if (len == 0.0f) len = 1.0f;
        for (j = 0; j < 3; j++) dir[j] = -dir[j] / len;
        if (l->Type == D3DLIGHT_DIRECTIONAL) memcpy(lv, dir, sizeof lv);
        else {
            float p[4], dist;
            p[0] = l->Position.x; p[1] = l->Position.y; p[2] = l->Position.z; p[3] = 1.0f;
            row(&st->view, p, p);
            for (j = 0; j < 3; j++) p[j] -= ep[j];
            dist = sqrtf(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
            for (j = 0; j < 3; j++) lv[j] = p[j] / dist;
            at = dist > l->Range ? 0.0f : 1.0f / (l->Attenuation0 + l->Attenuation1 * dist + l->Attenuation2 * dist * dist);
            if (l->Type == D3DLIGHT_SPOT) {
                float rho = lv[0] * dir[0] + lv[1] * dir[1] + lv[2] * dir[2], ct = cosf(l->Theta / 2.0f), cp = cosf(l->Phi / 2.0f);
                if (rho <= cp) at = 0.0f;
                else if (rho < ct) at *= powf((rho - cp) / (ct - cp), l->Falloff);
            }
        }
        nl = nv[0] * lv[0] + nv[1] * lv[1] + nv[2] * lv[2];
        if (nl < 0.0f) nl = 0.0f;
        for (j = 0; j < 3; j++) { amb[j] += at * (&l->Ambient.r)[j]; dif[j] += at * nl * (&l->Diffuse.r)[j]; h[j] = lv[j] + v[j]; }
        if (nl > 0.0f && st->rs[D3DRS_SPECULARENABLE]) {
            float hl = sqrtf(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]), nh = hl > 0.0f ? (nv[0] * h[0] + nv[1] * h[1] + nv[2] * h[2]) / hl : 0.0f, sp;
            sp = nh > 0.0f ? at * powf(nh, m->Power) : 0.0f;
            for (j = 0; j < 3; j++) spe[j] += sp * (&l->Specular.r)[j];
        }
    }
    for (j = 0; j < 3; j++) {
        o0[j] = em[j] + am[j] * (gl[j] + amb[j]) + dm[j] * dif[j];
        o1[j] = sm[j] * spe[j];
    }
    o0[3] = dm[3];
}

static uint32_t pack(const float *c)
{
    uint32_t o = 0;
    int i;
    for (i = 0; i < 4; i++) {
        float v = c[i] < 0.0f ? 0.0f : c[i] > 1.0f ? 1.0f : c[i];
        o |= (uint32_t)(v * 255.0f + 0.5f) << (i == 3 ? 24 : 16 - 8 * i);
    }
    return o;
}

static float fog_of(const d3state *st, const float *ep)
{
    float st0, en, de, z = fabsf(ep[2]), f;
    memcpy(&st0, &st->rs[D3DRS_FOGSTART], 4); memcpy(&en, &st->rs[D3DRS_FOGEND], 4); memcpy(&de, &st->rs[D3DRS_FOGDENSITY], 4);
    if (st->rs[D3DRS_RANGEFOGENABLE]) z = sqrtf(ep[0] * ep[0] + ep[1] * ep[1] + ep[2] * ep[2]);
    switch (st->rs[D3DRS_FOGVERTEXMODE]) {
    case D3DFOG_LINEAR: f = en != st0 ? (en - z) / (en - st0) : 1.0f; break;
    case D3DFOG_EXP: f = expf(-de * z); break;
    default: f = expf(-(de * z) * (de * z)); break;
    }
    return f < 0.0f ? 0.0f : f > 1.0f ? 1.0f : f;
}

static int elem_bytes(BYTE t)
{
    switch (t) {
    case D3DDECLTYPE_FLOAT1: case D3DDECLTYPE_D3DCOLOR: case D3DDECLTYPE_UBYTE4: case D3DDECLTYPE_UBYTE4N: case D3DDECLTYPE_SHORT2: case D3DDECLTYPE_SHORT2N: case D3DDECLTYPE_USHORT2N: return 4;
    case D3DDECLTYPE_FLOAT2: case D3DDECLTYPE_SHORT4: case D3DDECLTYPE_SHORT4N: case D3DDECLTYPE_USHORT4N: return 8;
    case D3DDECLTYPE_FLOAT3: return 12;
    default: return 16;
    }
}

/* ProcessVertices: n vertices from src go to the output buffer from vertex dst, pretransformed. out's layout is `de` */
HRESULT d3_process_vertices(d3dev *d, UINT src, UINT dst, UINT n, d3buf *out, const d3decl *de, DWORD flags)
{
    const d3state *st = &d->st;
    const D3DVIEWPORT9 *vp = &st->vp;
    const d3decl *in = st->decl;
    vsrc s;
    UINT i, stride = 0;
    int j;
    if (!in || !de) return D3DERR_INVALIDCALL;
    for (j = 0; j < de->n; j++) {
        UINT e = de->el[j].Offset + (UINT)elem_bytes(de->el[j].Type);
        if (de->el[j].Stream == 0 && e > stride) stride = e;
    }
    if (!stride || (size_t)(dst + n) * stride > out->size) return D3DERR_INVALIDCALL;
    g2r_keep_changed(d->r, &out->keep);
    s.d = d; s.de = in; s.up = NULL; s.stride = 0; s.inst = -1;
    for (i = 0; i < n; i++) {
        uint8_t *o = out->mem + (size_t)(dst + i) * stride;
        float pos[4], col[2][4] = { { 1, 1, 1, 1 }, { 0, 0, 0, 0 } }, tex[8][4], psize = 1.0f, fog = -1.0f, rhw;
        int have_col = 1;
        s.v = src + i;
        memcpy(&psize, &st->rs[D3DRS_POINTSIZE], 4);
        if (st->vs) {
            float vin[32][4];
            d3vsout v;
            shader_in(&s, st->vs, vin);
            if (d3_vs_cpu(st, st->vs, (const float (*)[4])vin, &v) < 0) return D3DERR_INVALIDCALL;
            memcpy(pos, v.pos, sizeof pos);
            memcpy(col, v.col, sizeof col);
            for (j = 0; j < 8; j++) memcpy(tex[j], v.tex[j], 16);
            if (v.wrote_psize) psize = v.psize[0];
            if (v.wrote_fog) fog = v.fog[0];
        } else if (elem(in, D3DDECLUSAGE_POSITIONT, 0)) {
            int a = load(&s, D3DDECLUSAGE_COLOR, 0, col[0]), b = load(&s, D3DDECLUSAGE_COLOR, 1, col[1]);
            if (!a) col[0][0] = col[0][1] = col[0][2] = col[0][3] = 1.0f;
            if (!b) col[1][0] = col[1][1] = col[1][2] = col[1][3] = 0.0f;
            load(&s, D3DDECLUSAGE_POSITIONT, 0, pos);
            rhw = pos[3];
            pos[3] = rhw != 0.0f ? 1.0f / rhw : 1.0f;
            pos[0] = (2.0f * (pos[0] - vp->X) / vp->Width - 1.0f) * pos[3];
            pos[1] = (1.0f - 2.0f * (pos[1] - vp->Y) / vp->Height) * pos[3];
            pos[2] = (pos[2] - vp->MinZ) / (vp->MaxZ - vp->MinZ) * pos[3];
            for (j = 0; j < 8; j++) load(&s, D3DDECLUSAGE_TEXCOORD, j, tex[j]);
        } else {
            float ep[4], nv[3];
            int a, b;
            ff_eye(&s, ep, nv);
            row(&st->proj, ep, pos);
            a = load(&s, D3DDECLUSAGE_COLOR, 0, col[0]); b = load(&s, D3DDECLUSAGE_COLOR, 1, col[1]);
            if (!a) col[0][0] = col[0][1] = col[0][2] = col[0][3] = 1.0f;
            if (!b) col[1][0] = col[1][1] = col[1][2] = col[1][3] = 0.0f;
            if (st->rs[D3DRS_LIGHTING]) { float c1a = col[1][3]; ff_light(st, ep, nv, col[0], col[1], a, b, col[0], col[1]); col[1][3] = c1a; }
            else have_col = a || b;
            if (st->rs[D3DRS_FOGENABLE] && st->rs[D3DRS_FOGVERTEXMODE] != D3DFOG_NONE && st->rs[D3DRS_FOGTABLEMODE] == D3DFOG_NONE) fog = fog_of(st, ep);
            for (j = 0; j < 8; j++) {
                DWORD ttf = st->tss[j][D3DTSS_TEXTURETRANSFORMFLAGS] & 7;
                load(&s, D3DDECLUSAGE_TEXCOORD, (int)(st->tss[j][D3DTSS_TEXCOORDINDEX] & 7), tex[j]);
                if (ttf) row(&st->texm[j], tex[j], tex[j]);
            }
        }
        rhw = pos[3] != 0.0f ? 1.0f / pos[3] : 0.0f;
        if (fog >= 0.0f) col[1][3] = fog;
        for (j = 0; j < de->n; j++) {
            const D3DVERTEXELEMENT9 *e = &de->el[j];
            uint8_t *q = o + e->Offset;
            if (e->Stream) continue;
            if (e->Usage == D3DDECLUSAGE_POSITIONT || e->Usage == D3DDECLUSAGE_POSITION) {
                float f[4];
                f[0] = vp->X + (pos[0] * rhw + 1.0f) * vp->Width * 0.5f;
                f[1] = vp->Y + (1.0f - pos[1] * rhw) * vp->Height * 0.5f;
                f[2] = vp->MinZ + pos[2] * rhw * (vp->MaxZ - vp->MinZ);
                f[3] = rhw;
                memcpy(q, f, (size_t)elem_bytes(e->Type));
            }
            else if (e->Usage == D3DDECLUSAGE_COLOR && e->UsageIndex < 2 && (have_col || !(flags & D3DPV_DONOTCOPYDATA))) {
                if (e->Type == D3DDECLTYPE_D3DCOLOR) { uint32_t c = pack(col[e->UsageIndex]); memcpy(q, &c, 4); }
                else memcpy(q, col[e->UsageIndex], (size_t)elem_bytes(e->Type));
            }
            else if (e->Usage == D3DDECLUSAGE_TEXCOORD && e->UsageIndex < 8 && !(flags & D3DPV_DONOTCOPYDATA)) memcpy(q, tex[e->UsageIndex], (size_t)elem_bytes(e->Type));
            else if (e->Usage == D3DDECLUSAGE_PSIZE) memcpy(q, &psize, 4);
        }
    }
    return D3D_OK;
}
