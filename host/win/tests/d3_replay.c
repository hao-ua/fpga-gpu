/* d3_replay — plays a file of d3d9_rec.c on the boards through d3d9.dll's public API, with no game: every recorded operation with the whole state it ran under, each frame saved as replay_NNN.bmp.
 *   d3_replay file.g2rc outdir [first_frame last_frame [first_op last_op]]
 * Frames count from the file's oldest; the op range limits draws within the frame, for narrowing a fault down to one draw (clears always run). Recorded with d3d8.dll too: by then its shaders are Direct3D 9's form. */
#include "d3d9_internal.h"
#include "d3d9_rec.h"

static IDirect3DDevice9 *g_d;
static const uint8_t *g_p;

typedef struct map { uint32_t id; void *o; } map;
static void *find(const map *m, int n, uint32_t id)
{
    int i;
    for (i = 0; i < n; i++) if (m[i].id == id) return m[i].o;
    return NULL;
}

static const void *take(size_t n) { const uint8_t *q = g_p; g_p += n; return q; }

static void save(IDirect3DSurface9 *sys, const char *dir, int frame, int w, int h)
{
    char path[MAX_PATH];
    BITMAPFILEHEADER fh;
    BITMAPINFOHEADER ih;
    D3DLOCKED_RECT l;
    FILE *f;
    int y;
    snprintf(path, sizeof path, "%s\\replay_%03d.bmp", dir, frame);
    f = fopen(path, "wb");
    if (!f) return;
    memset(&fh, 0, sizeof fh); memset(&ih, 0, sizeof ih);
    fh.bfType = 0x4D42; fh.bfOffBits = sizeof fh + sizeof ih; fh.bfSize = fh.bfOffBits + (DWORD)w * h * 4;
    ih.biSize = sizeof ih; ih.biWidth = w; ih.biHeight = -h; ih.biPlanes = 1; ih.biBitCount = 32;
    fwrite(&fh, sizeof fh, 1, f); fwrite(&ih, sizeof ih, 1, f);
    IDirect3DSurface9_LockRect(sys, &l, NULL, D3DLOCK_READONLY);
    for (y = 0; y < h; y++) fwrite((const uint8_t *)l.pBits + (ptrdiff_t)y * l.Pitch, 4, (size_t)w, f);
    IDirect3DSurface9_UnlockRect(sys);
    fclose(f);
}

static void apply_state(const d3state *s)
{
    int i, j;
    for (i = 0; i < MAX_RS; i++) IDirect3DDevice9_SetRenderState(g_d, (D3DRENDERSTATETYPE)i, s->rs[i]);
    for (i = 0; i < MAX_STAGES; i++) for (j = 1; j < MAX_TSS; j++) IDirect3DDevice9_SetTextureStageState(g_d, (DWORD)i, (D3DTEXTURESTAGESTATETYPE)j, s->tss[i][j]);
    for (i = 0; i < MAX_SAMPLERS; i++) for (j = 1; j < MAX_SS; j++) IDirect3DDevice9_SetSamplerState(g_d, (DWORD)i, (D3DSAMPLERSTATETYPE)j, s->ss[i][j]);
    IDirect3DDevice9_SetTransform(g_d, D3DTS_WORLD, &s->world);
    IDirect3DDevice9_SetTransform(g_d, D3DTS_VIEW, &s->view);
    IDirect3DDevice9_SetTransform(g_d, D3DTS_PROJECTION, &s->proj);
    for (i = 0; i < MAX_STAGES; i++) IDirect3DDevice9_SetTransform(g_d, (D3DTRANSFORMSTATETYPE)(D3DTS_TEXTURE0 + i), &s->texm[i]);
    IDirect3DDevice9_SetMaterial(g_d, &s->mat);
    for (i = 0; i < MAX_LIGHTS; i++) { IDirect3DDevice9_SetLight(g_d, (DWORD)i, &s->light[i]); IDirect3DDevice9_LightEnable(g_d, (DWORD)i, s->light_on[i]); }
    for (i = 0; i < 6; i++) IDirect3DDevice9_SetClipPlane(g_d, (DWORD)i, s->clip[i]);
    IDirect3DDevice9_SetVertexShaderConstantF(g_d, 0, s->vc[0], VS_CONSTS);
    IDirect3DDevice9_SetPixelShaderConstantF(g_d, 0, s->pc[0], PS_CONSTS);
    IDirect3DDevice9_SetVertexShaderConstantI(g_d, 0, s->vi[0], 16);
    IDirect3DDevice9_SetPixelShaderConstantI(g_d, 0, s->pi[0], 16);
    IDirect3DDevice9_SetVertexShaderConstantB(g_d, 0, s->vb_c, 16);
    IDirect3DDevice9_SetPixelShaderConstantB(g_d, 0, s->pb_c, 16);
}

static D3DPRIMITIVETYPE prim_of(uint32_t hw, uint32_t n, UINT *count)
{
    switch (hw) {
    case 3: *count = n; return D3DPT_POINTLIST;
    case 4: *count = n / 2; return D3DPT_LINELIST;
    case 5: *count = n - 1; return D3DPT_LINESTRIP;
    case 1: *count = n - 2; return D3DPT_TRIANGLESTRIP;
    case 2: *count = n - 2; return D3DPT_TRIANGLEFAN;
    default: *count = n / 3; return D3DPT_TRIANGLELIST;
    }
}

int main(int argc, char **argv)
{
    static IDirect3DVertexBuffer9 *vb[MAX_STREAMS];
    static UINT vb_cap[MAX_STREAMS];
    IDirect3DIndexBuffer9 *ib = NULL;
    UINT ib_cap = 0;
    const char *dir = argc > 2 ? argv[2] : ".";
    int f0 = argc > 3 ? atoi(argv[3]) : 0, f1 = argc > 4 ? atoi(argv[4]) : 1 << 30, o0 = argc > 5 ? atoi(argv[5]) : 0, o1 = argc > 6 ? atoi(argv[6]) : 1 << 30, frame = -1, op = 0, i, shown = 0;
    uint32_t last_recorded = 0xFFFFFFFFu, n, cur_rt = 0, cur_ds = 0xFFFFFFFFu;
    const rec_head *h;
    FILE *fp;
    long long size;
    uint8_t *file;
    WNDCLASSA wc;
    HWND w;
    IDirect3D9 *o = Direct3DCreate9(D3D_SDK_VERSION);
    D3DPRESENT_PARAMETERS p;
    map *sh, *de, *tx, *sf;
    IDirect3DSurface9 *back = NULL, *auto_ds = NULL, *sys = NULL, *cap = NULL;
    if (argc < 2 || !o) { printf("d3_replay file.g2rc outdir [first_frame last_frame [first_op last_op]]\n"); return 1; }
    fp = fopen(argv[1], "rb");
    if (!fp) { printf("cannot open %s\n", argv[1]); return 1; }
    _fseeki64(fp, 0, SEEK_END); size = _ftelli64(fp); _fseeki64(fp, 0, SEEK_SET);
    file = (uint8_t *)malloc((size_t)size);
    if (!file || fread(file, 1, (size_t)size, fp) != (size_t)size) { printf("cannot read the file\n"); return 1; }
    fclose(fp);
    g_p = file;
    h = (const rec_head *)take(sizeof *h);
    if (h->magic != REC_MAGIC || h->version != REC_VERSION) { printf("not a recording of this version\n"); return 1; }
    if (h->state_size != sizeof(d3state)) { printf("the recording's state is %u bytes, here %u: rebuild this tool with the library that recorded it\n", h->state_size, (unsigned)sizeof(d3state)); return 1; }
    printf("%u frames, %u operations, %u shaders, %u textures, %u x %u\n", h->nframes, h->nops, h->nshaders, h->ntex, h->vw, h->vh);
    memset(&wc, 0, sizeof wc);
    wc.lpfnWndProc = DefWindowProcA; wc.lpszClassName = "d3_replay"; wc.hInstance = GetModuleHandleA(NULL);
    RegisterClassA(&wc);
    w = CreateWindowA("d3_replay", "d3_replay", WS_OVERLAPPEDWINDOW, 0, 0, (int)h->vw, (int)h->vh, NULL, NULL, wc.hInstance, NULL);
    memset(&p, 0, sizeof p);
    p.BackBufferWidth = h->vw; p.BackBufferHeight = h->vh; p.BackBufferFormat = (D3DFORMAT)h->back_fmt; p.BackBufferCount = 1; p.SwapEffect = D3DSWAPEFFECT_DISCARD; p.hDeviceWindow = w; p.Windowed = TRUE;
    p.EnableAutoDepthStencil = h->ds_fmt != 0; p.AutoDepthStencilFormat = (D3DFORMAT)h->ds_fmt;
    if (IDirect3D9_CreateDevice(o, 0, D3DDEVTYPE_HAL, w, D3DCREATE_HARDWARE_VERTEXPROCESSING, &p, &g_d) != D3D_OK) { printf("no device\n"); return 1; }
    IDirect3DDevice9_GetBackBuffer(g_d, 0, 0, D3DBACKBUFFER_TYPE_MONO, &back);
    IDirect3DDevice9_GetDepthStencilSurface(g_d, &auto_ds);
    IDirect3DDevice9_CreateOffscreenPlainSurface(g_d, h->vw, h->vh, (D3DFORMAT)h->back_fmt, D3DPOOL_SYSTEMMEM, &cap, NULL);
    (void)sys;
    sh = (map *)calloc(h->nshaders + 1, sizeof *sh); de = (map *)calloc(h->ndecls + 1, sizeof *de); tx = (map *)calloc(h->ntex + 1, sizeof *tx); sf = (map *)calloc(h->nsurf + 1, sizeof *sf);
    for (n = 0; n < h->nshaders; n++) {
        const rec_shader *r = (const rec_shader *)take(sizeof *r);
        const DWORD *t = (const DWORD *)take((size_t)r->ntok * 4);
        void *x = NULL;
        if (r->ps) IDirect3DDevice9_CreatePixelShader(g_d, t, (IDirect3DPixelShader9 **)&x);
        else IDirect3DDevice9_CreateVertexShader(g_d, t, (IDirect3DVertexShader9 **)&x);
        sh[n].id = r->id; sh[n].o = x;
    }
    for (n = 0; n < h->ndecls; n++) {
        const rec_decl *r = (const rec_decl *)take(sizeof *r);
        const D3DVERTEXELEMENT9 *e = (const D3DVERTEXELEMENT9 *)take(((size_t)r->n + 1) * sizeof *e);
        IDirect3DVertexDeclaration9 *x = NULL;
        IDirect3DDevice9_CreateVertexDeclaration(g_d, e, &x);
        de[n].id = r->id; de[n].o = x;
    }
    for (n = 0; n < h->ntex; n++) {
        const rec_tex *r = (const rec_tex *)take(sizeof *r);
        int cube = r->layers > 1, f2, l;
        D3DPOOL pool = (r->usage & D3DUSAGE_RENDERTARGET) ? D3DPOOL_DEFAULT : (D3DPOOL)r->pool;
        void *x = NULL;
        if (cube) IDirect3DDevice9_CreateCubeTexture(g_d, r->w, r->levels, r->usage, (D3DFORMAT)r->fmt, pool, (IDirect3DCubeTexture9 **)&x, NULL);
        else IDirect3DDevice9_CreateTexture(g_d, r->w, r->h, r->levels, r->usage, (D3DFORMAT)r->fmt, pool, (IDirect3DTexture9 **)&x, NULL);
        for (f2 = 0; f2 < (int)r->layers; f2++) {
            for (l = 0; l < (int)r->levels; l++) {
                const rec_level *q = (const rec_level *)take(sizeof *q);
                const uint8_t *px = (const uint8_t *)take(q->bytes);
                D3DLOCKED_RECT k;
                if (!q->bytes || !x) continue;
                if (cube ? IDirect3DCubeTexture9_LockRect((IDirect3DCubeTexture9 *)x, (D3DCUBEMAP_FACES)f2, (UINT)l, &k, NULL, 0) != D3D_OK : IDirect3DTexture9_LockRect((IDirect3DTexture9 *)x, (UINT)l, &k, NULL, 0) != D3D_OK) continue;
                if (k.Pitch == (INT)q->pitch) memcpy(k.pBits, px, q->bytes);
                if (cube) IDirect3DCubeTexture9_UnlockRect((IDirect3DCubeTexture9 *)x, (D3DCUBEMAP_FACES)f2, (UINT)l); else IDirect3DTexture9_UnlockRect((IDirect3DTexture9 *)x, (UINT)l);
            }
        }
        tx[n].id = r->id; tx[n].o = x;
    }
    for (n = 0; n < h->nsurf; n++) {
        const rec_surf *r = (const rec_surf *)take(sizeof *r);
        IDirect3DSurface9 *x = NULL;
        if (r->back) x = back;
        else if (r->auto_ds) x = auto_ds;
        else if (r->tex) {
            void *t = find(tx, (int)h->ntex, r->tex);
            if (t) IDirect3DTexture9_GetSurfaceLevel((IDirect3DTexture9 *)t, r->level, &x);
        }
        else if (r->usage & D3DUSAGE_DEPTHSTENCIL) IDirect3DDevice9_CreateDepthStencilSurface(g_d, r->w, r->h, (D3DFORMAT)r->fmt, D3DMULTISAMPLE_NONE, 0, FALSE, &x, NULL);
        else IDirect3DDevice9_CreateRenderTarget(g_d, r->w, r->h, (D3DFORMAT)r->fmt, D3DMULTISAMPLE_NONE, 0, FALSE, &x, NULL);
        sf[n].id = r->id; sf[n].o = x;
    }
    cur_rt = 0;
    for (n = 0; n < h->nops; n++) {
        const rec_ophead *q = (const rec_ophead *)take(sizeof *q);
        const d3state *st = (const d3state *)take(sizeof(d3state));
        const D3DRECT *rects = (const D3DRECT *)take((size_t)q->cn * sizeof(D3DRECT));
        const uint32_t *idx = (const uint32_t *)take((size_t)q->nidx * 4);
        const uint8_t *sdata[MAX_STREAMS];
        rec_stream sinfo[MAX_STREAMS];
        int run;
        memset(sdata, 0, sizeof sdata);
        for (i = 0; i < MAX_STREAMS; i++) {
            if (q->streams >> i & 1) {
                sinfo[i] = *(const rec_stream *)take(sizeof(rec_stream));
                sdata[i] = (const uint8_t *)take(sinfo[i].bytes);
            }
        }
        if (q->frame != last_recorded) { last_recorded = q->frame; frame++; op = 0; }
        run = frame >= f0 && frame <= f1;
        if (q->kind == REC_PRESENT) {
            if (run) {
                IDirect3DDevice9_SetRenderTarget(g_d, 0, back);
                cur_rt = 0;
                IDirect3DDevice9_GetRenderTargetData(g_d, back, cap);
                save(cap, dir, frame, (int)h->vw, (int)h->vh);
                IDirect3DDevice9_Present(g_d, NULL, NULL, NULL, NULL);
                shown++;
            }
            continue;
        }
        if (!run) continue;
        if (q->rt != cur_rt) { void *x = find(sf, (int)h->nsurf, q->rt); if (x) IDirect3DDevice9_SetRenderTarget(g_d, 0, (IDirect3DSurface9 *)x); cur_rt = q->rt; }
        if (q->ds != cur_ds) { IDirect3DDevice9_SetDepthStencilSurface(g_d, q->ds ? (IDirect3DSurface9 *)find(sf, (int)h->nsurf, q->ds) : NULL); cur_ds = q->ds; }
        apply_state(st);
        IDirect3DDevice9_SetViewport(g_d, &st->vp);
        IDirect3DDevice9_SetScissorRect(g_d, &st->sc);
        if (q->kind == REC_CLEAR) { IDirect3DDevice9_Clear(g_d, q->cn, q->cn ? rects : NULL, q->cflags, q->ccolor, q->cz, q->cstencil); continue; }
        if (op >= o0 && op <= o1) {
            D3DPRIMITIVETYPE pt;
            UINT count, nv = q->hi - q->lo + 1;
            IDirect3DDevice9_SetVertexDeclaration(g_d, (IDirect3DVertexDeclaration9 *)find(de, (int)h->ndecls, (uint32_t)(uintptr_t)st->decl));
            IDirect3DDevice9_SetVertexShader(g_d, st->vs ? (IDirect3DVertexShader9 *)find(sh, (int)h->nshaders, (uint32_t)(uintptr_t)st->vs) : NULL);
            IDirect3DDevice9_SetPixelShader(g_d, st->ps ? (IDirect3DPixelShader9 *)find(sh, (int)h->nshaders, (uint32_t)(uintptr_t)st->ps) : NULL);
            for (i = 0; i < 8; i++) IDirect3DDevice9_SetTexture(g_d, (DWORD)i, st->tex[i] ? (IDirect3DBaseTexture9 *)find(tx, (int)h->ntex, (uint32_t)(uintptr_t)st->tex[i]) : NULL);
            for (i = 0; i < MAX_STREAMS; i++) {
                void *m;
                if (!sdata[i]) continue;
                if (vb_cap[i] < sinfo[i].bytes) {
                    if (vb[i]) IDirect3DVertexBuffer9_Release(vb[i]);
                    vb_cap[i] = sinfo[i].bytes * 2;
                    IDirect3DDevice9_CreateVertexBuffer(g_d, vb_cap[i], 0, 0, D3DPOOL_MANAGED, &vb[i], NULL);
                }
                IDirect3DVertexBuffer9_Lock(vb[i], 0, 0, &m, 0);
                memcpy(m, sdata[i], sinfo[i].bytes);
                IDirect3DVertexBuffer9_Unlock(vb[i]);
                IDirect3DDevice9_SetStreamSource(g_d, (UINT)i, vb[i], 0, sinfo[i].stride);
            }
            pt = prim_of(q->mode, q->n, &count);
            if (q->nidx) {
                uint32_t *ix = NULL;
                UINT bytes = q->nidx * 4, k;
                if (ib_cap < bytes) {
                    if (ib) IDirect3DIndexBuffer9_Release(ib);
                    ib_cap = bytes * 2;
                    IDirect3DDevice9_CreateIndexBuffer(g_d, ib_cap, 0, D3DFMT_INDEX32, D3DPOOL_MANAGED, &ib, NULL);
                }
                IDirect3DIndexBuffer9_Lock(ib, 0, 0, (void **)&ix, 0);
                for (k = 0; k < q->nidx; k++) ix[k] = idx[k] - q->lo;
                IDirect3DIndexBuffer9_Unlock(ib);
                IDirect3DDevice9_SetIndices(g_d, ib);
                IDirect3DDevice9_DrawIndexedPrimitive(g_d, pt, 0, 0, nv, 0, count);
            } else IDirect3DDevice9_DrawPrimitive(g_d, pt, 0, count);
        }
        op++;
    }
    printf("%d frames played\n", shown);
    return 0;
}
