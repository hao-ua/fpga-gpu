/* d3d8_test — d3d8.dll through the SDK's d3d8.h on the boards: what Direct3D 8 does differently from the d3d9.dll underneath, read back with CopyRects as a game would.
 * The draws themselves are d3d_test's business. Shaders are tokens written out: nothing current assembles vs.1.1 without dcl. Tolerances are +-2 of 255. */
#define CINTERFACE
#define COBJMACROS
#include <windows.h>
#include <initguid.h>
#include <d3d8.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 640
#define H 480

static int g_bad, g_checks;
static IDirect3D8 *g_o;
static IDirect3DDevice8 *g_d;

static void ok(int c, const char *what) { g_checks++; if (!c) { g_bad++; printf("FAIL %s\n", what); } }
#define HR(x) do { HRESULT h_ = (x); g_checks++; if (FAILED(h_)) { g_bad++; printf("FAIL %s = %08lx\n", #x, (unsigned long)h_); } } while (0)

/* a pixel of a surface, through a system-memory copy */
static void surf_px(IDirect3DSurface8 *s, int x, int y, const char *what, int r, int g, int b)
{
    IDirect3DSurface8 *m = NULL;
    D3DSURFACE_DESC de;
    D3DLOCKED_RECT l;
    uint8_t *p;
    g_checks++;
    IDirect3DSurface8_GetDesc(s, &de);
    if (FAILED(IDirect3DDevice8_CreateImageSurface(g_d, de.Width, de.Height, de.Format, &m)) || FAILED(IDirect3DDevice8_CopyRects(g_d, s, NULL, 0, m, NULL))) { g_bad++; printf("FAIL %s: no copy\n", what); if (m) IDirect3DSurface8_Release(m); return; }
    IDirect3DSurface8_LockRect(m, &l, NULL, D3DLOCK_READONLY);
    p = (uint8_t *)l.pBits + (ptrdiff_t)y * l.Pitch + (ptrdiff_t)x * 4;
    if (abs(p[2] - r) > 2 || abs(p[1] - g) > 2 || abs(p[0] - b) > 2) { g_bad++; printf("FAIL %s: pixel (%d, %d) = %d %d %d, expected %d %d %d\n", what, x, y, p[2], p[1], p[0], r, g, b); }
    IDirect3DSurface8_UnlockRect(m);
    IDirect3DSurface8_Release(m);
}
static void px(int x, int y, const char *what, int r, int g, int b)
{
    IDirect3DSurface8 *rt = NULL;
    IDirect3DDevice8_GetRenderTarget(g_d, &rt);
    surf_px(rt, x, y, what, r, g, b);
    IDirect3DSurface8_Release(rt);
}

typedef struct { float x, y, z, w; DWORD c; float u, v; } tlv;
#define FVF_TL (D3DFVF_XYZRHW | D3DFVF_DIFFUSE | D3DFVF_TEX1)

static void quad_uv(float x0, float y0, float x1, float y1, float z, DWORD c, float u1, float v1)
{
    tlv v[4] = { { x0, y0, z, 1, c, 0, 0 }, { x1, y0, z, 1, c, u1, 0 }, { x0, y1, z, 1, c, 0, v1 }, { x1, y1, z, 1, c, u1, v1 } };
    IDirect3DDevice8_SetVertexShader(g_d, FVF_TL);
    HR(IDirect3DDevice8_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, v, sizeof v[0]));
}
static void quad(float x0, float y0, float x1, float y1, float z, DWORD c) { quad_uv(x0, y0, x1, y1, z, c, 1, 1); }

static void clear(D3DCOLOR c) { HR(IDirect3DDevice8_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, c, 1.0f, 0)); }

static void flat_state(void)
{
    IDirect3DDevice8_SetRenderState(g_d, D3DRS_ZENABLE, FALSE);
    IDirect3DDevice8_SetRenderState(g_d, D3DRS_LIGHTING, FALSE);
    IDirect3DDevice8_SetRenderState(g_d, D3DRS_CULLMODE, D3DCULL_NONE);
    IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
    IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_ALPHAARG1, D3DTA_DIFFUSE);
    IDirect3DDevice8_SetTexture(g_d, 0, NULL);
}
static void use_texture(IDirect3DTexture8 *t)
{
    IDirect3DDevice8_SetTexture(g_d, 0, (IDirect3DBaseTexture8 *)t);
    IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
}

static IDirect3DTexture8 *tex_new(UINT w, UINT h, UINT levels, const DWORD *colors)
{
    IDirect3DTexture8 *t = NULL;
    UINT l;
    HR(IDirect3DDevice8_CreateTexture(g_d, w, h, levels, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &t));
    if (!t) return NULL;
    for (l = 0; l < levels; l++) {
        D3DLOCKED_RECT k;
        UINT lw = w >> l ? w >> l : 1, lh = h >> l ? h >> l : 1, x, y;
        HR(IDirect3DTexture8_LockRect(t, l, &k, NULL, 0));
        for (y = 0; y < lh; y++) for (x = 0; x < lw; x++) ((DWORD *)((uint8_t *)k.pBits + (ptrdiff_t)y * k.Pitch))[x] = levels > 1 ? colors[l] : colors[y * lw + x];
        IDirect3DTexture8_UnlockRect(t, l);
    }
    return t;
}

/* ---------------------------------------------------------------- IDirect3D8 */
static void test_d3d(void)
{
    D3DADAPTER_IDENTIFIER8 id;
    D3DCAPS8 c;
    D3DDISPLAYMODE m;
    UINT n, i, big = 0, f565 = 0;
    ok(IDirect3D8_GetAdapterCount(g_o) == 1, "one adapter");
    HR(IDirect3D8_GetAdapterIdentifier(g_o, 0, 0, &id));
    ok(!strcmp(id.Driver, "d3d8.dll") && strstr(id.Description, "GPU2") != NULL && id.VendorId == 0x1D0F, "adapter identifier");
    n = IDirect3D8_GetAdapterModeCount(g_o, 0);
    ok(n == 10, "sizes times formats");
    for (i = 0; i < n; i++) {
        HR(IDirect3D8_EnumAdapterModes(g_o, 0, i, &m));
        if (m.Width == 1280 && m.Height == 800) big++;
        if (m.Format == D3DFMT_R5G6B5) f565++;
    }
    ok(big == 2 && f565 == 5, "every size in both formats");
    ok(IDirect3D8_EnumAdapterModes(g_o, 0, n, &m) == D3DERR_INVALIDCALL, "past the last mode");
    HR(IDirect3D8_GetAdapterDisplayMode(g_o, 0, &m));
    HR(IDirect3D8_CheckDeviceType(g_o, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, D3DFMT_X8R8G8B8, TRUE));
    HR(IDirect3D8_CheckDeviceFormat(g_o, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, 0, D3DRTYPE_TEXTURE, D3DFMT_DXT1));
    HR(IDirect3D8_CheckDeviceFormat(g_o, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, D3DUSAGE_DEPTHSTENCIL, D3DRTYPE_SURFACE, D3DFMT_D16));
    HR(IDirect3D8_CheckDeviceMultiSampleType(g_o, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, TRUE, D3DMULTISAMPLE_NONE));
    ok(IDirect3D8_CheckDeviceMultiSampleType(g_o, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, TRUE, D3DMULTISAMPLE_4_SAMPLES) != D3D_OK, "no multisampling");
    HR(IDirect3D8_CheckDepthStencilMatch(g_o, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, D3DFMT_X8R8G8B8, D3DFMT_D24S8));
    HR(IDirect3D8_GetDeviceCaps(g_o, 0, D3DDEVTYPE_HAL, &c));
    ok(c.VertexShaderVersion == D3DVS_VERSION(1, 1) && c.PixelShaderVersion == D3DPS_VERSION(1, 1) && c.MaxVertexShaderConst == 96, "shader versions of Direct3D 8");
    ok((IDirect3D8_CheckDeviceFormat(g_o, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, 0, D3DRTYPE_VOLUMETEXTURE, D3DFMT_A8R8G8B8) == D3D_OK) == ((c.TextureCaps & D3DPTEXTURECAPS_VOLUMEMAP) != 0), "a volume format where the caps have volumes");   /* board B's 3D textures */
    ok((c.RasterCaps & D3DPRASTERCAPS_ZBIAS) != 0, "ZBIAS in the caps");
    ok(c.MaxTextureWidth == 2048 && c.MaxTextureBlendStages == 8 && c.MaxStreams == 16, "the fields after Direct3D 9's additions stay in place");
    ok(c.MaxPixelShaderValue == 8.0f, "the last field of D3DCAPS8");
    ok(IDirect3D8_GetAdapterMonitor(g_o, 0) != NULL, "a monitor");
}

/* ---------------------------------------------------------------- the device's small methods */
static void test_device(void)
{
    D3DCAPS8 c;
    D3DDISPLAYMODE m;
    D3DRASTER_STATUS rs;
    D3DGAMMARAMP g;
    D3DDEVICE_CREATION_PARAMETERS cp;
    IDirect3D8 *o = NULL;
    IDirect3DSurface8 *bb = NULL;
    IDirect3DDevice8 *x = NULL;
    DWORD n = 0, info[4];
    HR(IDirect3DDevice8_TestCooperativeLevel(g_d));
    ok(IDirect3DDevice8_GetAvailableTextureMem(g_d) > 0, "texture memory");
    HR(IDirect3DDevice8_ResourceManagerDiscardBytes(g_d, 0));
    HR(IDirect3DDevice8_GetDirect3D(g_d, &o));
    ok(o == g_o, "GetDirect3D");
    if (o) IDirect3D8_Release(o);
    HR(IDirect3DDevice8_GetDeviceCaps(g_d, &c));
    ok(c.VertexShaderVersion == D3DVS_VERSION(1, 1), "device caps");
    HR(IDirect3DDevice8_GetDisplayMode(g_d, &m));
    HR(IDirect3DDevice8_GetCreationParameters(g_d, &cp));
    ok(cp.DeviceType == D3DDEVTYPE_HAL, "creation parameters");
    HR(IDirect3DDevice8_GetBackBuffer(g_d, 0, D3DBACKBUFFER_TYPE_MONO, &bb));
    if (bb) {
        D3DSURFACE_DESC d;
        IDirect3DSurface8_GetDesc(bb, &d);
        ok(d.Width == W && d.Height == H && d.Format == D3DFMT_X8R8G8B8 && d.Size == W * H * 4 && d.Usage == D3DUSAGE_RENDERTARGET, "the back buffer's description");
        IDirect3DSurface8_Release(bb);
    }
    HR(IDirect3DDevice8_GetRasterStatus(g_d, &rs));
    IDirect3DDevice8_GetGammaRamp(g_d, &g);
    ok(g.red[255] == 65535 && g.green[0] == 0, "gamma ramp");
    IDirect3DDevice8_SetGammaRamp(g_d, 0, &g);
    HR(IDirect3DDevice8_ValidateDevice(g_d, &n));
    ok(IDirect3DDevice8_GetInfo(g_d, 1, info, sizeof info) == S_FALSE, "GetInfo has nothing");
    HR(IDirect3DDevice8_QueryInterface(g_d, &IID_IDirect3DDevice8, (void **)&x));
    ok(x == g_d, "QueryInterface of the device");
    if (x) IDirect3DDevice8_Release(x);
}

/* ---------------------------------------------------------------- states Direct3D 9 moved to the samplers */
static void test_samplers(void)
{
    static const DWORD two[2] = { 0xFFFF0000, 0xFF0000FF }, mips[3] = { 0xFFFF0000, 0xFF00FF00, 0xFF0000FF };
    IDirect3DTexture8 *t = tex_new(2, 1, 1, two), *m = tex_new(4, 4, 3, mips);
    DWORD v = 0;
    flat_state();
    use_texture(t);
    clear(0);
    HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_MAGFILTER, D3DTEXF_POINT));
    HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_MINFILTER, D3DTEXF_POINT));
    HR(IDirect3DDevice8_GetTextureStageState(g_d, 0, D3DTSS_MAGFILTER, &v));
    ok(v == D3DTEXF_POINT, "GetTextureStageState reads the filter");
    quad(0, 0, 200, 40, 0.5f, 0xFFFFFFFF);
    px(50, 20, "point: the first texel", 255, 0, 0);
    px(150, 20, "point: the second texel", 0, 0, 255);
    HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_MAGFILTER, D3DTEXF_LINEAR));
    quad(0, 0, 200, 40, 0.5f, 0xFFFFFFFF);
    px(100, 20, "linear: halfway between the texels", 127, 0, 128);
    HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_MAGFILTER, D3DTEXF_POINT));
    HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_ADDRESSU, D3DTADDRESS_WRAP));
    quad_uv(0, 0, 400, 40, 0.5f, 0xFFFFFFFF, 2, 1);
    px(250, 20, "wrap: u = 1.25 is the first texel", 255, 0, 0);
    HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_ADDRESSU, D3DTADDRESS_CLAMP));
    HR(IDirect3DDevice8_GetTextureStageState(g_d, 0, D3DTSS_ADDRESSU, &v));
    ok(v == D3DTADDRESS_CLAMP, "GetTextureStageState reads the address mode");
    quad_uv(0, 0, 400, 40, 0.5f, 0xFFFFFFFF, 2, 1);
    px(250, 20, "clamp: u = 1.25 is the last texel", 0, 0, 255);
    HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_ADDRESSU, D3DTADDRESS_MIRROR));
    quad_uv(0, 0, 400, 40, 0.5f, 0xFFFFFFFF, 2, 1);
    px(250, 20, "mirror: u = 1.25 is the second texel", 0, 0, 255);
    px(350, 20, "mirror: u = 1.75 is the first texel", 255, 0, 0);
    HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_ADDRESSU, D3DTADDRESS_WRAP));
    /* a 4 x 4 texture on 2 x 2 pixels: level 1 if mipmaps are on */
    use_texture(m);
    HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_MIPFILTER, D3DTEXF_NONE));
    quad(100, 100, 102, 102, 0.5f, 0xFFFFFFFF);
    px(100, 100, "no mip filter: level 0", 255, 0, 0);
    HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_MIPFILTER, D3DTEXF_POINT));
    quad(100, 100, 102, 102, 0.5f, 0xFFFFFFFF);
    px(100, 100, "point mip filter: level 1", 0, 255, 0);
    HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_MIPFILTER, D3DTEXF_NONE));
    flat_state();
    if (t) IDirect3DTexture8_Release(t);
    if (m) IDirect3DTexture8_Release(m);
}

/* ---------------------------------------------------------------- SetIndices carries the base vertex */
static IDirect3DVertexBuffer8 *g_vb;
static IDirect3DIndexBuffer8 *g_ib;

static void test_indices(void)
{
    static const WORD idx[4] = { 0, 1, 2, 3 };
    tlv v[8];
    uint8_t *p = NULL;
    IDirect3DIndexBuffer8 *b = NULL;
    IDirect3DVertexBuffer8 *vb = NULL;
    D3DVERTEXBUFFER_DESC vd;
    D3DINDEXBUFFER_DESC id;
    UINT base = 99, stride = 0;
    int i;
    for (i = 0; i < 8; i++) {
        float x = (i & 1 ? 100.0f : 0.0f) + (i >= 4 ? 200.0f : 0.0f), y = i & 2 ? 100.0f : 0.0f;
        tlv one = { x, y, 0.5f, 1, i >= 4 ? 0xFF00FF00 : 0xFFFF0000, 0, 0 };
        v[i] = one;
    }
    HR(IDirect3DDevice8_CreateVertexBuffer(g_d, sizeof v, D3DUSAGE_WRITEONLY, FVF_TL, D3DPOOL_MANAGED, &g_vb));
    HR(IDirect3DDevice8_CreateIndexBuffer(g_d, sizeof idx, D3DUSAGE_WRITEONLY, D3DFMT_INDEX16, D3DPOOL_MANAGED, &g_ib));
    HR(IDirect3DVertexBuffer8_Lock(g_vb, 0, 0, &p, 0));
    memcpy(p, v, sizeof v);
    IDirect3DVertexBuffer8_Unlock(g_vb);
    HR(IDirect3DIndexBuffer8_Lock(g_ib, 0, 0, &p, 0));
    memcpy(p, idx, sizeof idx);
    IDirect3DIndexBuffer8_Unlock(g_ib);
    HR(IDirect3DVertexBuffer8_GetDesc(g_vb, &vd));
    ok(vd.Size == sizeof v && vd.FVF == FVF_TL && vd.Type == D3DRTYPE_VERTEXBUFFER, "vertex buffer description");
    HR(IDirect3DIndexBuffer8_GetDesc(g_ib, &id));
    ok(id.Size == sizeof idx && id.Format == D3DFMT_INDEX16, "index buffer description");
    flat_state();
    clear(0);
    IDirect3DDevice8_SetVertexShader(g_d, FVF_TL);
    HR(IDirect3DDevice8_SetStreamSource(g_d, 0, g_vb, sizeof v[0]));
    HR(IDirect3DDevice8_GetStreamSource(g_d, 0, &vb, &stride));
    ok(vb == g_vb && stride == sizeof v[0], "GetStreamSource");
    if (vb) IDirect3DVertexBuffer8_Release(vb);
    HR(IDirect3DDevice8_DrawPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 2));
    px(50, 50, "a stream without an offset", 255, 0, 0);
    px(250, 50, "only the first four", 0, 0, 0);
    HR(IDirect3DDevice8_SetIndices(g_d, g_ib, 4));
    HR(IDirect3DDevice8_GetIndices(g_d, &b, &base));
    ok(b == g_ib && base == 4, "GetIndices gives the base vertex");
    if (b) IDirect3DIndexBuffer8_Release(b);
    clear(0);
    HR(IDirect3DDevice8_DrawIndexedPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 4, 0, 2));
    px(250, 50, "base vertex 4: the second quad", 0, 255, 0);
    px(50, 50, "base vertex 4: not the first", 0, 0, 0);
    HR(IDirect3DDevice8_SetIndices(g_d, g_ib, 0));
    clear(0);
    HR(IDirect3DDevice8_DrawIndexedPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 4, 0, 2));
    px(50, 50, "base vertex 0: the first quad", 255, 0, 0);
    px(250, 50, "base vertex 0: not the second", 0, 0, 0);
}

/* a buffer that stops changing moves to board A: over several frames, so the move and the drops happen inside the test */
static void tlv_quad(tlv *o, float x0, float x1, DWORD c)
{
    int i;
    for (i = 0; i < 4; i++) { o[i].x = i & 1 ? x1 : x0; o[i].y = i & 2 ? (float)H : 0.0f; o[i].z = 0.5f; o[i].w = 1; o[i].c = c; o[i].u = o[i].v = 0; }
}

/* the memory stays the game's: a write no call of ours sees still has to reach the next draw */
static void kept_quad(const char *what, int r, int b)
{
    clear(0);
    HR(IDirect3DDevice8_DrawPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 2));
    px(W / 4, H / 2, what, r, 0, 0);
    px(W * 3 / 4, H / 2, what, 0, 0, b);
    HR(IDirect3DDevice8_Present(g_d, NULL, NULL, NULL, NULL));
}

static void test_kept_unseen(void)
{
    tlv v[4], w[4];
    uint8_t *held = NULL, *p = NULL;
    IDirect3DVertexBuffer8 *vb = NULL;
    int f;
    tlv_quad(v, 0, W / 2, 0xFFFF0000);
    tlv_quad(w, W / 2, W, 0xFF0000FF);
    HR(IDirect3DDevice8_CreateVertexBuffer(g_d, sizeof v, D3DUSAGE_WRITEONLY, FVF_TL, D3DPOOL_MANAGED, &vb));
    HR(IDirect3DVertexBuffer8_Lock(vb, 0, 0, &held, 0));
    memcpy(held, v, sizeof v);
    IDirect3DVertexBuffer8_Unlock(vb);
    flat_state();
    IDirect3DDevice8_SetVertexShader(g_d, FVF_TL);
    HR(IDirect3DDevice8_SetStreamSource(g_d, 0, vb, sizeof v[0]));
    for (f = 0; f < 6; f++) kept_quad("kept, unseen: before", 255, 0);
    memcpy(held, w, sizeof w);
    kept_quad("kept, unseen: a pointer held past Unlock", 0, 255);
    memcpy(held, v, sizeof v);
    for (f = 0; f < 14; f++) kept_quad("kept, unseen: written back", 255, 0);
    HR(IDirect3DVertexBuffer8_Lock(vb, 0, 0, &p, D3DLOCK_READONLY));
    memcpy(p, w, sizeof w);
    IDirect3DVertexBuffer8_Unlock(vb);
    kept_quad("kept, unseen: under READONLY", 0, 255);
    memcpy(held, v, sizeof v);
    for (f = 0; f < 30; f++) kept_quad("kept, unseen: written back again", 255, 0);
    ((tlv *)held)[3].y = 0;                                     /* the last vertex alone: the second triangle collapses */
    clear(0);
    HR(IDirect3DDevice8_DrawPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 2));
    px(W / 8, H / 8, "kept, unseen: the last vertex alone, the first triangle", 255, 0, 0);
    px(W * 3 / 8, H * 7 / 8, "kept, unseen: the last vertex alone, the second triangle", 0, 0, 0);
    HR(IDirect3DDevice8_Present(g_d, NULL, NULL, NULL, NULL));
    HR(IDirect3DDevice8_SetStreamSource(g_d, 0, NULL, 0));
    IDirect3DVertexBuffer8_Release(vb);
}

static void test_kept(void)
{
    tlv v[8], w[4];
    uint8_t *p = NULL;
    IDirect3DVertexBuffer8 *vb = NULL;
    int f;
    tlv_quad(v, 0, W / 2, 0xFFFF0000);
    tlv_quad(v + 4, W / 2, W, 0xFF00FF00);
    HR(IDirect3DDevice8_CreateVertexBuffer(g_d, sizeof v, D3DUSAGE_WRITEONLY, FVF_TL, D3DPOOL_MANAGED, &vb));
    HR(IDirect3DVertexBuffer8_Lock(vb, 0, 0, &p, 0));
    memcpy(p, v, sizeof v);
    IDirect3DVertexBuffer8_Unlock(vb);
    flat_state();
    IDirect3DDevice8_SetVertexShader(g_d, FVF_TL);
    HR(IDirect3DDevice8_SetStreamSource(g_d, 0, vb, sizeof v[0]));
    HR(IDirect3DDevice8_SetIndices(g_d, g_ib, 4));
    for (f = 0; f < 8; f++) {
        clear(0);
        HR(IDirect3DDevice8_DrawPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 2));
        px(W / 4, H / 2, "kept: from vertex 0", 255, 0, 0);
        clear(0);
        HR(IDirect3DDevice8_DrawIndexedPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 4, 0, 2));
        px(W * 3 / 4, H / 2, "kept: base vertex 4", 0, 255, 0);
        px(W / 4, H / 2, "kept: base vertex 4, not the first quad", 0, 0, 0);
        HR(IDirect3DDevice8_Present(g_d, NULL, NULL, NULL, NULL));
    }
    /* changed between two draws of a frame, through Direct3D 8's own Lock: the first keeps what it read */
    clear(0);
    HR(IDirect3DDevice8_DrawPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 2));
    tlv_quad(w, W / 2, W, 0xFF0000FF);
    HR(IDirect3DVertexBuffer8_Lock(vb, 0, sizeof w, &p, 0));
    memcpy(p, w, sizeof w);
    IDirect3DVertexBuffer8_Unlock(vb);
    HR(IDirect3DDevice8_DrawPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 2));
    px(W / 4, H / 2, "kept: a draw before the Lock", 255, 0, 0);
    px(W * 3 / 4, H / 2, "kept: a draw after the Lock", 0, 0, 255);
    HR(IDirect3DDevice8_Present(g_d, NULL, NULL, NULL, NULL));
    for (f = 0; f < 48; f++) {
        int c = 64 + (f / 7) * 16;
        if (f % 7 == 0) {
            tlv_quad(w, 0, W, D3DCOLOR_XRGB(c, 0, 0));
            HR(IDirect3DVertexBuffer8_Lock(vb, 0, sizeof w, &p, 0));
            memcpy(p, w, sizeof w);
            IDirect3DVertexBuffer8_Unlock(vb);
        }
        clear(0);
        HR(IDirect3DDevice8_DrawPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 2));
        px(W / 2, H / 2, "kept: changed now and then", c, 0, 0);
        HR(IDirect3DDevice8_Present(g_d, NULL, NULL, NULL, NULL));
    }
    IDirect3DDevice8_SetStreamSource(g_d, 0, NULL, 0);
    IDirect3DDevice8_SetIndices(g_d, NULL, 0);
    IDirect3DVertexBuffer8_Release(vb);
}

/* ---------------------------------------------------------------- ZBIAS */
static void zbias_on(const char *what)
{
    char s[80];
    DWORD v = 99;
    clear(0);
    IDirect3DDevice8_SetRenderState(g_d, D3DRS_ZENABLE, D3DZB_TRUE);
    IDirect3DDevice8_SetRenderState(g_d, D3DRS_ZFUNC, D3DCMP_LESS);
    IDirect3DDevice8_SetRenderState(g_d, D3DRS_ZBIAS, 0);
    quad(0, 0, 100, 100, 0.5f, 0xFFFF0000);
    quad(0, 0, 100, 100, 0.5f, 0xFF00FF00);
    snprintf(s, sizeof s, "%s: the same depth loses without a bias", what);
    px(50, 50, s, 255, 0, 0);
    HR(IDirect3DDevice8_SetRenderState(g_d, D3DRS_ZBIAS, 1));
    HR(IDirect3DDevice8_GetRenderState(g_d, D3DRS_ZBIAS, &v));
    ok(v == 1, "GetRenderState reads ZBIAS as it was set");
    quad(0, 0, 100, 100, 0.5f, 0xFF00FF00);
    snprintf(s, sizeof s, "%s: bias 1 comes forward", what);
    px(50, 50, s, 0, 255, 0);
    HR(IDirect3DDevice8_SetRenderState(g_d, D3DRS_ZBIAS, 0));
    quad(0, 0, 100, 100, 0.5f, 0xFF0000FF);
    snprintf(s, sizeof s, "%s: bias 0 again stays behind", what);
    px(50, 50, s, 0, 255, 0);
    IDirect3DDevice8_SetRenderState(g_d, D3DRS_ZENABLE, D3DZB_FALSE);
    IDirect3DDevice8_SetRenderState(g_d, D3DRS_ZFUNC, D3DCMP_LESSEQUAL);
}

static void test_targets(void)
{
    static const DWORD one[1] = { 0xFF102030 };
    IDirect3DSurface8 *bb = NULL, *ds = NULL, *rt = NULL, *d16 = NULL, *got = NULL, *lv = NULL;
    IDirect3DTexture8 *rtt = NULL;
    D3DVIEWPORT8 vp;
    flat_state();
    zbias_on("D24S8");
    HR(IDirect3DDevice8_GetRenderTarget(g_d, &bb));
    HR(IDirect3DDevice8_GetDepthStencilSurface(g_d, &ds));
    HR(IDirect3DDevice8_CreateRenderTarget(g_d, 256, 128, D3DFMT_X8R8G8B8, D3DMULTISAMPLE_NONE, TRUE, &rt));
    HR(IDirect3DDevice8_CreateDepthStencilSurface(g_d, 256, 128, D3DFMT_D16, D3DMULTISAMPLE_NONE, &d16));
    HR(IDirect3DDevice8_SetRenderTarget(g_d, rt, d16));
    HR(IDirect3DDevice8_GetViewport(g_d, &vp));
    ok(vp.Width == 256 && vp.Height == 128, "the viewport follows the target");
    HR(IDirect3DDevice8_GetRenderTarget(g_d, &got));
    ok(got == rt, "GetRenderTarget gives the same surface");
    if (got) IDirect3DSurface8_Release(got);
    zbias_on("D16");
    HR(IDirect3DDevice8_SetRenderTarget(g_d, NULL, NULL));
    HR(IDirect3DDevice8_GetRenderTarget(g_d, &got));
    ok(got == rt, "a NULL target keeps the target");
    if (got) IDirect3DSurface8_Release(got);
    got = (IDirect3DSurface8 *)1;
    ok(IDirect3DDevice8_GetDepthStencilSurface(g_d, &got) == D3DERR_NOTFOUND && !got, "a NULL depth surface takes it away");
    /* a texture as the target, then sampled */
    HR(IDirect3DDevice8_CreateTexture(g_d, 64, 64, 1, D3DUSAGE_RENDERTARGET, D3DFMT_X8R8G8B8, D3DPOOL_DEFAULT, &rtt));
    HR(IDirect3DTexture8_GetSurfaceLevel(rtt, 0, &lv));
    HR(IDirect3DDevice8_SetRenderTarget(g_d, lv, NULL));
    HR(IDirect3DDevice8_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0xFF204060, 1.0f, 0));
    HR(IDirect3DDevice8_SetRenderTarget(g_d, bb, ds));
    (void)one;
    clear(0);
    use_texture(rtt);
    quad(0, 0, 100, 100, 0.5f, 0xFFFFFFFF);
    px(50, 50, "a texture that was a target", 0x20, 0x40, 0x60);
    flat_state();
    {   /* a level that is not the first */
        IDirect3DTexture8 *m = NULL;
        IDirect3DSurface8 *l0 = NULL, *l1 = NULL;
        HR(IDirect3DDevice8_CreateTexture(g_d, 16, 16, 2, D3DUSAGE_RENDERTARGET, D3DFMT_X8R8G8B8, D3DPOOL_DEFAULT, &m));
        HR(IDirect3DTexture8_GetSurfaceLevel(m, 0, &l0));
        HR(IDirect3DTexture8_GetSurfaceLevel(m, 1, &l1));
        HR(IDirect3DDevice8_SetRenderTarget(g_d, l0, NULL));
        HR(IDirect3DDevice8_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0xFFFF0000, 1.0f, 0));
        HR(IDirect3DDevice8_SetRenderTarget(g_d, l1, NULL));
        HR(IDirect3DDevice8_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0xFF00FF00, 1.0f, 0));
        HR(IDirect3DDevice8_SetRenderTarget(g_d, bb, ds));
        clear(0);
        use_texture(m);
        IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_MIPFILTER, D3DTEXF_POINT);
        quad(0, 0, 16, 16, 0.5f, 0xFFFFFFFF);
        quad(100, 100, 108, 108, 0.5f, 0xFFFFFFFF);
        px(8, 8, "level 0 as a target", 255, 0, 0);
        px(104, 104, "level 1 as a target", 0, 255, 0);
        IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_MIPFILTER, D3DTEXF_NONE);
        flat_state();
        if (l0) IDirect3DSurface8_Release(l0);
        if (l1) IDirect3DSurface8_Release(l1);
        if (m) IDirect3DTexture8_Release(m);
    }
    if (lv) IDirect3DSurface8_Release(lv);
    if (rtt) IDirect3DTexture8_Release(rtt);
    if (rt) IDirect3DSurface8_Release(rt);
    if (d16) IDirect3DSurface8_Release(d16);
    if (bb) IDirect3DSurface8_Release(bb);
    if (ds) IDirect3DSurface8_Release(ds);
}

/* ---------------------------------------------------------------- vertex and pixel shaders */
#define VS11 0xFFFE0101u
#define PS11 0xFFFF0101u
#define END 0x0000FFFFu
#define OP_MOV 1
#define OP_MUL 5
#define OP_M4X4 20
#define OP_TEX 66
#define DST(ty, n) (0x80000000u | (DWORD)(ty) << 28 | 15u << 16 | (DWORD)(n))
#define SRC(ty, n) (0x80000000u | (DWORD)(ty) << 28 | 0xE4u << 16 | (DWORD)(n))
enum { R_TEMP, R_INPUT, R_CONST, R_ADDR, R_RASTOUT, R_ATTROUT, R_OUTPUT };

typedef struct { float x, y, z, w; DWORD c; } cv;
static float fl(float f) { return f; }

static void cv_quad(const cv *v, UINT stride) { HR(IDirect3DDevice8_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, v, stride)); }

static void test_shaders(void)
{
    static const DWORD fa[] = { VS11, OP_MOV, DST(R_RASTOUT, 0), SRC(R_INPUT, 0), OP_MOV, DST(R_ATTROUT, 0), SRC(R_INPUT, 5), END };
    DWORD da[] = { D3DVSD_STREAM(0), D3DVSD_REG(0, D3DVSDT_FLOAT4), D3DVSD_REG(5, D3DVSDT_D3DCOLOR), D3DVSD_END() };
    static const DWORD fb[] = { VS11, OP_M4X4, DST(R_RASTOUT, 0), SRC(R_INPUT, 0), SRC(R_CONST, 0), OP_MOV, DST(R_ATTROUT, 0), SRC(R_CONST, 4), END };
    DWORD db[12];
    static const DWORD fc[] = { VS11, OP_MOV, DST(R_RASTOUT, 0), SRC(R_INPUT, 0), OP_MOV, DST(R_ATTROUT, 0), SRC(R_INPUT, 5), OP_MOV, DST(R_OUTPUT, 0), SRC(R_INPUT, 7), END };
    DWORD dc[] = { D3DVSD_STREAM(0), D3DVSD_REG(0, D3DVSDT_FLOAT4), D3DVSD_SKIP(1), D3DVSD_REG(7, D3DVSDT_FLOAT2), D3DVSD_STREAM(1), D3DVSD_REG(5, D3DVSDT_D3DCOLOR), D3DVSD_END() };
    static const DWORD pc[] = { PS11, OP_TEX, DST(R_ADDR, 0), OP_MUL, DST(R_TEMP, 0), SRC(R_ADDR, 0), SRC(R_INPUT, 0), END };
    DWORD dd[] = { D3DVSD_STREAM(0), D3DVSD_REG(D3DVSDE_POSITION, D3DVSDT_FLOAT3), D3DVSD_REG(D3DVSDE_DIFFUSE, D3DVSDT_D3DCOLOR), D3DVSD_END() };
    static const DWORD tc[2] = { 0xFF000000, 0xFF80FF40 };
    static const float m[16] = { 1, 0, 0, 0.5f, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }, blue[4] = { 0, 0, 1, 1 };
    float fgreen[4] = { 0, 1, 0, 1 };
    cv q[4] = { { -0.5f, 0.5f, 0.5f, 1, 0xFFFF0000 }, { 0.5f, 0.5f, 0.5f, 1, 0xFFFF0000 }, { -0.5f, -0.5f, 0.5f, 1, 0xFFFF0000 }, { 0.5f, -0.5f, 0.5f, 1, 0xFFFF0000 } };
    struct { float x, y, z; DWORD c; } qd[4] = { { -0.5f, 0.5f, 0.5f, 0xFF0000FF }, { 0.5f, 0.5f, 0.5f, 0xFF0000FF }, { -0.5f, -0.5f, 0.5f, 0xFF0000FF }, { 0.5f, -0.5f, 0.5f, 0xFF0000FF } };
    /* the skipped word reads as u = 0.25, the first texel: only the right offset reaches the second */
    struct { float x, y, z, w, pad, u, v; } s0[4] = { { -0.5f, 0.5f, 0.5f, 1, 0.25f, 0.75f, 0.5f }, { 0.5f, 0.5f, 0.5f, 1, 0.25f, 0.75f, 0.5f }, { -0.5f, -0.5f, 0.5f, 1, 0.25f, 0.75f, 0.5f }, { 0.5f, -0.5f, 0.5f, 1, 0.25f, 0.75f, 0.5f } };
    DWORD s1[4] = { 0xFFFF80FF, 0xFFFF80FF, 0xFFFF80FF, 0xFFFF80FF }, ha = 0, hb = 0, hc = 0, hd = 0, hp = 0, got = 0, bytes = 0;
    DWORD back[16];
    IDirect3DVertexBuffer8 *v0 = NULL, *v1 = NULL;
    IDirect3DTexture8 *t = tex_new(2, 1, 1, tc);
    uint8_t *p = NULL;
    int k = 0;
    flat_state();
    IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_MAGFILTER, D3DTEXF_POINT);
    IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_MINFILTER, D3DTEXF_POINT);

    /* inputs named by the declaration alone, D3DCOLOR swapped as a game's vertex has it */
    HR(IDirect3DDevice8_CreateVertexShader(g_d, da, fa, &ha, 0));
    ok((ha & 1) != 0, "a shader handle is not an FVF");
    HR(IDirect3DDevice8_SetVertexShader(g_d, ha));
    HR(IDirect3DDevice8_GetVertexShader(g_d, &got));
    ok(got == ha, "GetVertexShader gives the handle");
    clear(0);
    cv_quad(q, sizeof q[0]);
    px(320, 240, "vs.1.1 with a declaration", 255, 0, 0);
    px(100, 100, "vs.1.1: outside the quad", 0, 0, 0);
    bytes = 0;
    HR(IDirect3DDevice8_GetVertexShaderFunction(g_d, ha, NULL, &bytes));
    ok(bytes == sizeof fa, "GetVertexShaderFunction: the size");
    memset(back, 0, sizeof back);
    HR(IDirect3DDevice8_GetVertexShaderFunction(g_d, ha, back, &bytes));
    ok(!memcmp(back, fa, sizeof fa), "GetVertexShaderFunction: the application's tokens, no dcl");
    bytes = 0;
    HR(IDirect3DDevice8_GetVertexShaderDeclaration(g_d, ha, NULL, &bytes));
    ok(bytes == sizeof da, "GetVertexShaderDeclaration: the size");
    HR(IDirect3DDevice8_GetVertexShaderDeclaration(g_d, ha, back, &bytes));
    ok(!memcmp(back, da, sizeof da), "GetVertexShaderDeclaration: the tokens");
    bytes = 1;
    ok(IDirect3DDevice8_GetVertexShaderDeclaration(g_d, ha, back, &bytes) == D3DERR_MOREDATA && bytes == sizeof da, "a buffer too small");

    /* a matrix in constants, and constants in the declaration */
    db[k++] = D3DVSD_CONST(4, 1);
    memcpy(&db[k], fgreen, 16); k += 4;
    db[k++] = D3DVSD_STREAM(0); db[k++] = D3DVSD_REG(0, D3DVSDT_FLOAT4); db[k++] = D3DVSD_END();
    HR(IDirect3DDevice8_CreateVertexShader(g_d, db, fb, &hb, 0));
    HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, 0, m, 4));
    memset(back, 0, sizeof back);
    HR(IDirect3DDevice8_GetVertexShaderConstant(g_d, 0, back, 4));
    ok(!memcmp(back, m, sizeof m), "GetVertexShaderConstant");
    HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, 4, blue, 1));
    HR(IDirect3DDevice8_SetVertexShader(g_d, hb));
    clear(0);
    cv_quad(q, sizeof q[0]);
    px(480, 240, "m4x4: moved right by half the clip space, in the declaration's colour", 0, 255, 0);
    px(100, 240, "m4x4: not where it was", 0, 0, 0);
    HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, 4, blue, 1));
    clear(0);
    cv_quad(q, sizeof q[0]);
    px(480, 240, "a constant set after the shader wins", 0, 0, 255);
    HR(IDirect3DDevice8_SetVertexShader(g_d, hb));
    clear(0);
    cv_quad(q, sizeof q[0]);
    px(480, 240, "setting the shader loads the declaration's constants again", 0, 255, 0);

    /* two streams, a skip, registers of the shader's choosing, and ps.1.1 */
    HR(IDirect3DDevice8_CreateVertexShader(g_d, dc, fc, &hc, 0));
    HR(IDirect3DDevice8_CreatePixelShader(g_d, pc, &hp));
    HR(IDirect3DDevice8_CreateVertexBuffer(g_d, sizeof s0, 0, 0, D3DPOOL_MANAGED, &v0));
    HR(IDirect3DDevice8_CreateVertexBuffer(g_d, sizeof s1, 0, 0, D3DPOOL_MANAGED, &v1));
    IDirect3DVertexBuffer8_Lock(v0, 0, 0, &p, 0); memcpy(p, s0, sizeof s0); IDirect3DVertexBuffer8_Unlock(v0);
    IDirect3DVertexBuffer8_Lock(v1, 0, 0, &p, 0); memcpy(p, s1, sizeof s1); IDirect3DVertexBuffer8_Unlock(v1);
    HR(IDirect3DDevice8_SetVertexShader(g_d, hc));
    HR(IDirect3DDevice8_SetPixelShader(g_d, hp));
    HR(IDirect3DDevice8_GetPixelShader(g_d, &got));
    ok(got == hp, "GetPixelShader gives the handle");
    HR(IDirect3DDevice8_SetStreamSource(g_d, 0, v0, sizeof s0[0]));
    HR(IDirect3DDevice8_SetStreamSource(g_d, 1, v1, 4));
    IDirect3DDevice8_SetTexture(g_d, 0, (IDirect3DBaseTexture8 *)t);
    clear(0);
    HR(IDirect3DDevice8_DrawPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 2));
    px(320, 240, "ps.1.1: texture past the skipped word times the colour of the second stream", 128, 128, 64);
    bytes = sizeof back;
    HR(IDirect3DDevice8_GetPixelShaderFunction(g_d, hp, back, &bytes));
    ok(bytes == sizeof pc && !memcmp(back, pc, sizeof pc), "GetPixelShaderFunction");
    HR(IDirect3DDevice8_SetPixelShader(g_d, 0));
    IDirect3DDevice8_SetTexture(g_d, 0, NULL);
    IDirect3DDevice8_SetStreamSource(g_d, 0, NULL, 0);
    IDirect3DDevice8_SetStreamSource(g_d, 1, NULL, 0);

    /* a declaration with no function: the fixed function reads it by the registers' meanings */
    HR(IDirect3DDevice8_CreateVertexShader(g_d, dd, NULL, &hd, 0));
    HR(IDirect3DDevice8_SetVertexShader(g_d, hd));
    clear(0);
    HR(IDirect3DDevice8_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, qd, sizeof qd[0]));
    px(320, 240, "fixed function through a declaration", 0, 0, 255);
    bytes = 7;
    HR(IDirect3DDevice8_GetVertexShaderFunction(g_d, hd, NULL, &bytes));
    ok(bytes == 0, "a declaration has no function");

    /* an FVF is a handle too */
    HR(IDirect3DDevice8_SetVertexShader(g_d, FVF_TL));
    HR(IDirect3DDevice8_GetVertexShader(g_d, &got));
    ok(got == FVF_TL, "GetVertexShader gives an FVF back");
    quad(0, 0, 10, 10, 0.5f, 0xFFFFFFFF);
    px(5, 5, "an FVF after a shader", 255, 255, 255);

    HR(IDirect3DDevice8_DeleteVertexShader(g_d, ha));
    HR(IDirect3DDevice8_DeleteVertexShader(g_d, hb));
    HR(IDirect3DDevice8_DeleteVertexShader(g_d, hc));
    HR(IDirect3DDevice8_DeleteVertexShader(g_d, hd));
    HR(IDirect3DDevice8_DeletePixelShader(g_d, hp));
    ok(IDirect3DDevice8_DeleteVertexShader(g_d, FVF_TL) == D3DERR_INVALIDCALL, "an FVF cannot be deleted");
    if (v0) IDirect3DVertexBuffer8_Release(v0);
    if (v1) IDirect3DVertexBuffer8_Release(v1);
    if (t) IDirect3DTexture8_Release(t);
    (void)fl;
}

/* ---------------------------------------------------------------- a0 loaded from a constant: one register per draw */
#define OP_DEF 81
#define DST_A0 (0x80000000u | (DWORD)R_ADDR << 28 | 1u << 16)
#define SRC_X(ty, n) (0x80000000u | (DWORD)(ty) << 28 | (DWORD)(n))
#define SRC_REL(n) (SRC(R_CONST, n) | 1u << 13)
#define FLOAT_1 0x3F800000u

static void test_relative(void)
{
    /* the game's pattern: the last lines of its 255-token car shader */
    static const DWORD fa[] = { VS11, OP_MOV, DST(R_RASTOUT, 0), SRC(R_INPUT, 0), OP_MOV, DST_A0, SRC_X(R_CONST, 8), OP_MOV, DST(R_ATTROUT, 0), SRC_REL(10), END };
    /* a matrix through a0, a0 loaded twice, a register def wrote */
    static const DWORD fb[] = { VS11, OP_DEF, DST(R_CONST, 42), 0, 0, FLOAT_1, FLOAT_1, OP_MOV, DST_A0, SRC_X(R_CONST, 9), OP_M4X4, DST(R_RASTOUT, 0), SRC(R_INPUT, 0), SRC_REL(20),
                                OP_MOV, DST_A0, SRC_X(R_CONST, 8), OP_MOV, DST(R_ATTROUT, 0), SRC_REL(40), END };
    /* from a vertex, the shape of skinning: every quad of one draw its own matrix and colour, a run-time index (LDPX). c64 is a def, the fifth quad's colour */
    static const DWORD fc[] = { VS11, OP_DEF, DST(R_CONST, 64), 0, FLOAT_1, FLOAT_1, FLOAT_1, OP_MOV, DST_A0, SRC_X(R_INPUT, 7), OP_M4X4, DST(R_RASTOUT, 0), SRC(R_INPUT, 0), SRC_REL(40), OP_MOV, DST(R_ATTROUT, 0), SRC_REL(44), END };
    DWORD de[] = { D3DVSD_STREAM(0), D3DVSD_REG(0, D3DVSDT_FLOAT4), D3DVSD_REG(5, D3DVSDT_D3DCOLOR), D3DVSD_END() };
    DWORD dv[] = { D3DVSD_STREAM(0), D3DVSD_REG(0, D3DVSDT_FLOAT4), D3DVSD_REG(7, D3DVSDT_FLOAT1), D3DVSD_END() };
    static const float green[4] = { 0, 1, 0, 1 }, red[4] = { 1, 0, 0, 1 }, white[4] = { 1, 1, 1, 1 }, mv[16] = { 1, 0, 0, -0.5f, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    cv q[4] = { { -0.5f, 0.5f, 0.5f, 1, 0xFF808080 }, { 0.5f, 0.5f, 0.5f, 1, 0xFF808080 }, { -0.5f, -0.5f, 0.5f, 1, 0xFF808080 }, { 0.5f, -0.5f, 0.5f, 1, 0xFF808080 } };
    float at[4] = { 2.7f, 0, 0, 0 };
    DWORD ha = 0, hb = 0, hc = 0;
    flat_state();
    HR(IDirect3DDevice8_CreateVertexShader(g_d, de, fa, &ha, 0));
    HR(IDirect3DDevice8_SetVertexShader(g_d, ha));
    HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, 8, at, 1));
    HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, 12, green, 1));
    HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, 9, red, 1));
    clear(0);
    cv_quad(q, sizeof q[0]);
    px(320, 240, "a0 = floor(2.7): c[12]", 0, 255, 0);
    at[0] = -1.0f;
    HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, 8, at, 1));
    clear(0);
    cv_quad(q, sizeof q[0]);
    px(320, 240, "the same shader, another index: c[9]", 255, 0, 0);
    at[0] = -0.5f;
    HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, 8, at, 1));
    HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, 9, green, 1));
    clear(0);
    cv_quad(q, sizeof q[0]);
    px(320, 240, "a0 = floor(-0.5) is -1, not 0", 0, 255, 0);

    HR(IDirect3DDevice8_CreateVertexShader(g_d, de, fb, &hb, 0));
    HR(IDirect3DDevice8_SetVertexShader(g_d, hb));
    at[0] = 3.0f;
    HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, 9, at, 1));
    at[0] = 2.0f;
    HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, 8, at, 1));
    HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, 23, mv, 4));
    HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, 42, white, 1));
    clear(0);
    cv_quad(q, sizeof q[0]);
    px(160, 240, "m4x4 through a0: four registers from c[23], moved left", 0, 0, 255);
    px(400, 240, "m4x4 through a0: not where it was", 0, 0, 0);

    /* a per-vertex a0: the indices floor, so 5.7 is 5 and -4.5 is -5, and the -5 quad reads registers below the base */
    {
        static const float idx[5] = { 0.0f, 5.7f, 10.0f, -4.5f, 20.0f }, tx[5] = { -0.8f, -0.4f, 0.0f, 0.4f, 0.8f };
        static const float bone[5][4] = { { 1, 0, 0, 1 }, { 0, 1, 0, 1 }, { 0, 0, 1, 1 }, { 1, 1, 0, 1 }, { 1, 0, 0, 1 } };
        static const int at_reg[5] = { 40, 45, 50, 35, 60 };
        static const float corner[6][2] = { { -0.1f, -0.1f }, { 0.1f, -0.1f }, { -0.1f, 0.1f }, { -0.1f, 0.1f }, { 0.1f, -0.1f }, { 0.1f, 0.1f } };
        struct { float x, y, z, w, i; } v[30];
        int b, k;
        HR(IDirect3DDevice8_CreateVertexShader(g_d, dv, fc, &hc, 0));
        HR(IDirect3DDevice8_SetVertexShader(g_d, hc));
        for (b = 0; b < 5; b++) {
            float mat[16];
            memset(mat, 0, sizeof mat);
            mat[0] = mat[5] = mat[10] = mat[15] = 1; mat[3] = tx[b];
            HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, (DWORD)at_reg[b], mat, 4));
            HR(IDirect3DDevice8_SetVertexShaderConstant(g_d, (DWORD)at_reg[b] + 4, bone[b], 1));
            for (k = 0; k < 6; k++) { v[b * 6 + k].x = corner[k][0]; v[b * 6 + k].y = corner[k][1]; v[b * 6 + k].z = 0.5f; v[b * 6 + k].w = 1; v[b * 6 + k].i = idx[b]; }
        }
        clear(0);
        HR(IDirect3DDevice8_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 10, v, sizeof v[0]));
        px(64, 240, "a0 from a vertex: index 0", 255, 0, 0);
        px(192, 240, "a0 from a vertex: 5.7 floors to 5", 0, 255, 0);
        px(320, 240, "a0 from a vertex: index 10", 0, 0, 255);
        px(448, 240, "a0 from a vertex: -4.5 floors to -5", 255, 255, 0);
        px(576, 240, "a0 from a vertex: a register def wrote", 0, 255, 255);
        px(128, 240, "a0 from a vertex: between the quads", 0, 0, 0);
    }

    HR(IDirect3DDevice8_SetVertexShader(g_d, FVF_TL));
    HR(IDirect3DDevice8_DeleteVertexShader(g_d, ha));
    HR(IDirect3DDevice8_DeleteVertexShader(g_d, hb));
    HR(IDirect3DDevice8_DeleteVertexShader(g_d, hc));
}

/* ---------------------------------------------------------------- state blocks are handles */
static void test_blocks(void)
{
    static const DWORD f[] = { VS11, OP_MOV, DST(R_RASTOUT, 0), SRC(R_INPUT, 0), OP_MOV, DST(R_ATTROUT, 0), SRC(R_INPUT, 5), END };
    DWORD de[] = { D3DVSD_STREAM(0), D3DVSD_REG(0, D3DVSDT_FLOAT4), D3DVSD_REG(5, D3DVSDT_D3DCOLOR), D3DVSD_END() };
    DWORD h = 0, tok = 0, all = 0, got = 0, v = 0;
    UINT base = 0;
    IDirect3DIndexBuffer8 *b = NULL;
    HR(IDirect3DDevice8_CreateVertexShader(g_d, de, f, &h, 0));
    HR(IDirect3DDevice8_SetIndices(g_d, g_ib, 0));
    HR(IDirect3DDevice8_SetVertexShader(g_d, FVF_TL));
    HR(IDirect3DDevice8_BeginStateBlock(g_d));
    HR(IDirect3DDevice8_SetIndices(g_d, g_ib, 4));
    HR(IDirect3DDevice8_SetVertexShader(g_d, h));
    HR(IDirect3DDevice8_SetRenderState(g_d, D3DRS_ZBIAS, 3));
    HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_ADDRESSU, D3DTADDRESS_CLAMP));
    HR(IDirect3DDevice8_EndStateBlock(g_d, &tok));
    ok(tok != 0, "EndStateBlock gives a handle");
    HR(IDirect3DDevice8_SetIndices(g_d, g_ib, 0));
    HR(IDirect3DDevice8_SetVertexShader(g_d, FVF_TL));
    HR(IDirect3DDevice8_SetRenderState(g_d, D3DRS_ZBIAS, 0));
    HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_ADDRESSU, D3DTADDRESS_WRAP));
    HR(IDirect3DDevice8_ApplyStateBlock(g_d, tok));
    HR(IDirect3DDevice8_GetIndices(g_d, &b, &base));
    ok(base == 4, "a state block carries the base vertex");
    if (b) IDirect3DIndexBuffer8_Release(b);
    HR(IDirect3DDevice8_GetVertexShader(g_d, &got));
    ok(got == h, "a state block carries the shader's handle");
    HR(IDirect3DDevice8_GetRenderState(g_d, D3DRS_ZBIAS, &v));
    ok(v == 3, "a state block carries ZBIAS");
    HR(IDirect3DDevice8_GetTextureStageState(g_d, 0, D3DTSS_ADDRESSU, &v));
    ok(v == D3DTADDRESS_CLAMP, "a state block carries a stage's address mode");
    /* an FVF the block has seen */
    HR(IDirect3DDevice8_SetVertexShader(g_d, FVF_TL));
    HR(IDirect3DDevice8_CaptureStateBlock(g_d, tok));
    HR(IDirect3DDevice8_SetVertexShader(g_d, h));
    HR(IDirect3DDevice8_ApplyStateBlock(g_d, tok));
    HR(IDirect3DDevice8_GetVertexShader(g_d, &got));
    ok(got == FVF_TL, "CaptureStateBlock took the FVF");
    HR(IDirect3DDevice8_CreateStateBlock(g_d, D3DSBT_ALL, &all));
    ok(all != 0, "CreateStateBlock gives a handle");
    HR(IDirect3DDevice8_SetRenderState(g_d, D3DRS_ZBIAS, 7));
    HR(IDirect3DDevice8_ApplyStateBlock(g_d, all));
    HR(IDirect3DDevice8_GetRenderState(g_d, D3DRS_ZBIAS, &v));
    ok(v == 3, "CreateStateBlock took everything");
    HR(IDirect3DDevice8_DeleteStateBlock(g_d, tok));
    HR(IDirect3DDevice8_DeleteStateBlock(g_d, all));
    HR(IDirect3DDevice8_SetRenderState(g_d, D3DRS_ZBIAS, 0));
    HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_ADDRESSU, D3DTADDRESS_WRAP));
    HR(IDirect3DDevice8_SetVertexShader(g_d, FVF_TL));
    HR(IDirect3DDevice8_DeleteVertexShader(g_d, h));
}

/* ---------------------------------------------------------------- the objects are Direct3D 8's to a game */
static void test_objects(void)
{
    static const DWORD c[1] = { 0xFFFFFFFF };
    IDirect3DTexture8 *t = tex_new(1, 1, 1, c), *q = NULL;
    IDirect3DCubeTexture8 *cu = NULL, *qc = NULL;
    IDirect3DBaseTexture8 *b = NULL;
    IDirect3DResource8 *r = NULL;
    IDirect3DSurface8 *s = NULL, *cs = NULL;
    IDirect3DDevice8 *d = NULL;
    IUnknown *u = NULL;
    D3DSURFACE_DESC de;
    D3DLOCKED_RECT l;
    D3DVERTEXBUFFER_DESC vd;
    ULONG n;
    if (!t) return;
    HR(IDirect3DDevice8_SetTexture(g_d, 0, (IDirect3DBaseTexture8 *)t));
    HR(IDirect3DDevice8_GetTexture(g_d, 0, &b));
    ok((void *)b == (void *)t, "GetTexture gives the same object");
    HR(IDirect3DBaseTexture8_QueryInterface(b, &IID_IDirect3DTexture8, (void **)&q));
    ok(q == t, "a base texture is a texture");
    if (q) IDirect3DTexture8_Release(q);
    HR(IDirect3DBaseTexture8_QueryInterface(b, &IID_IDirect3DResource8, (void **)&r));
    if (r) IDirect3DResource8_Release(r);
    ok(IDirect3DBaseTexture8_GetType(b) == D3DRTYPE_TEXTURE, "GetType");
    ok(IDirect3DBaseTexture8_GetLevelCount(b) == 1, "GetLevelCount");
    if (b) IDirect3DBaseTexture8_Release(b);
    IDirect3DDevice8_SetTexture(g_d, 0, NULL);
    ok(IDirect3DTexture8_QueryInterface(t, &IID_IDirect3DCubeTexture8, (void **)&qc) == E_NOINTERFACE, "a texture is no cube");
    HR(IDirect3DTexture8_GetSurfaceLevel(t, 0, &s));
    HR(IDirect3DSurface8_GetContainer(s, &IID_IDirect3DTexture8, (void **)&q));
    ok(q == t, "GetContainer");
    if (q) IDirect3DTexture8_Release(q);
    HR(IDirect3DSurface8_GetDesc(s, &de));
    ok(de.Size == 4 && de.Width == 1 && de.Format == D3DFMT_A8R8G8B8 && de.Type == D3DRTYPE_SURFACE && de.Pool == D3DPOOL_MANAGED, "a surface's description has Size");
    HR(IDirect3DSurface8_GetDevice(s, &d));
    ok(d == g_d, "GetDevice of a surface");
    if (d) IDirect3DDevice8_Release(d);
    HR(IDirect3DSurface8_QueryInterface(s, &IID_IDirect3DSurface8, (void **)&u));
    if (u) IUnknown_Release(u);
    HR(IDirect3DSurface8_LockRect(s, &l, NULL, 0));
    HR(IDirect3DSurface8_UnlockRect(s));
    if (s) IDirect3DSurface8_Release(s);
    HR(IDirect3DTexture8_GetLevelDesc(t, 0, &de));
    ok(de.Size == 4, "GetLevelDesc");
    ok(IDirect3DTexture8_GetLevelDesc(t, 1, &de) == D3DERR_INVALIDCALL, "GetLevelDesc past the last level");
    HR(IDirect3DTexture8_GetDevice(t, &d));
    ok(d == g_d, "GetDevice of a texture");
    if (d) IDirect3DDevice8_Release(d);
    HR(IDirect3DTexture8_AddDirtyRect(t, NULL));
    n = IDirect3DTexture8_Release(t);
    ok(n == 0, "the last reference of a texture");

    HR(IDirect3DDevice8_CreateCubeTexture(g_d, 8, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &cu));
    if (cu) {
        HR(IDirect3DCubeTexture8_GetCubeMapSurface(cu, D3DCUBEMAP_FACE_POSITIVE_Y, 0, &cs));
        HR(IDirect3DCubeTexture8_GetLevelDesc(cu, 0, &de));
        ok(de.Size == 256 && de.Width == 8, "a cube's level description");
        HR(IDirect3DCubeTexture8_LockRect(cu, D3DCUBEMAP_FACE_NEGATIVE_X, 0, &l, NULL, 0));
        HR(IDirect3DCubeTexture8_UnlockRect(cu, D3DCUBEMAP_FACE_NEGATIVE_X, 0));
        HR(IDirect3DCubeTexture8_QueryInterface(cu, &IID_IDirect3DCubeTexture8, (void **)&qc));
        if (qc) IDirect3DCubeTexture8_Release(qc);
        if (cs) {
            HR(IDirect3DSurface8_GetContainer(cs, &IID_IDirect3DCubeTexture8, (void **)&qc));
            ok(qc == cu, "GetContainer of a cube face");
            if (qc) IDirect3DCubeTexture8_Release(qc);
            IDirect3DSurface8_Release(cs);
        }
        IDirect3DCubeTexture8_Release(cu);
    }
    HR(IDirect3DVertexBuffer8_GetDevice(g_vb, &d));
    ok(d == g_d, "GetDevice of a buffer");
    if (d) IDirect3DDevice8_Release(d);
    HR(IDirect3DVertexBuffer8_QueryInterface(g_vb, &IID_IDirect3DVertexBuffer8, (void **)&u));
    if (u) IUnknown_Release(u);
    HR(IDirect3DIndexBuffer8_QueryInterface(g_ib, &IID_IDirect3DIndexBuffer8, (void **)&u));
    if (u) IUnknown_Release(u);
    HR(IDirect3DVertexBuffer8_GetDesc(g_vb, &vd));
    {   /* the same object behind Direct3D 8's tables: GetLevelDesc has a Size */
        IDirect3DVolumeTexture8 *vt = NULL;
        IDirect3DVolume8 *vl = NULL;
        D3DVOLUME_DESC vde;
        D3DLOCKED_BOX bx;
        D3DCAPS8 cp;
        void *q = NULL;
        typedef struct { float x, y, z, w, u, v, r; } v3;
        v3 a[4];
        int i, x, y, k;
        HR(IDirect3DDevice8_GetDeviceCaps(g_d, &cp));
        if (!(cp.TextureCaps & D3DPTEXTURECAPS_VOLUMEMAP))      /* a board B without 3D textures (or GPU2_NO_3D=1) */
            ok(!cp.MaxVolumeExtent && IDirect3DDevice8_CreateVolumeTexture(g_d, 4, 4, 2, 2, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &vt) == D3DERR_NOTAVAILABLE && !vt, "no volumes");
        else {
            ok(cp.MaxVolumeExtent == 255, "volume caps");
            HR(IDirect3DDevice8_CreateVolumeTexture(g_d, 4, 4, 2, 2, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &vt));
        }
        if (vt) {
            ok(IDirect3DVolumeTexture8_GetType(vt) == D3DRTYPE_VOLUMETEXTURE, "GetType of a volume");
            ok(IDirect3DVolumeTexture8_GetLevelCount(vt) == 2, "a volume's levels");
            HR(IDirect3DVolumeTexture8_GetLevelDesc(vt, 0, &vde));
            ok(vde.Size == 4 * 4 * 2 * 4 && vde.Width == 4 && vde.Height == 4 && vde.Depth == 2 && vde.Type == D3DRTYPE_VOLUME, "a volume's level description");
            HR(IDirect3DVolumeTexture8_GetVolumeLevel(vt, 1, &vl));
            if (vl) {
                HR(IDirect3DVolume8_GetDesc(vl, &vde));
                ok(vde.Size == 2 * 2 * 1 * 4 && vde.Depth == 1, "IDirect3DVolume8::GetDesc");
                HR(IDirect3DVolume8_GetContainer(vl, &IID_IDirect3DVolumeTexture8, &q));
                ok(q == vt, "GetContainer of a volume");
                if (q) IDirect3DVolumeTexture8_Release((IDirect3DVolumeTexture8 *)q);
                HR(IDirect3DVolume8_QueryInterface(vl, &IID_IDirect3DVolume8, (void **)&u));
                if (u) IUnknown_Release(u);
                IDirect3DVolume8_Release(vl);
            }
            HR(IDirect3DVolumeTexture8_QueryInterface(vt, &IID_IDirect3DVolumeTexture8, (void **)&u));
            if (u) IUnknown_Release(u);
            HR(IDirect3DVolumeTexture8_LockBox(vt, 0, &bx, NULL, 0));
            for (k = 0; k < 2; k++) for (y = 0; y < 4; y++) for (x = 0; x < 4; x++) *(DWORD *)((uint8_t *)bx.pBits + k * bx.SlicePitch + y * bx.RowPitch + x * 4) = k ? 0xFF00FF00 : 0xFFFF0000;
            HR(IDirect3DVolumeTexture8_UnlockBox(vt, 0));
            HR(IDirect3DDevice8_SetTexture(g_d, 0, (IDirect3DBaseTexture8 *)vt));
            HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_MAGFILTER, D3DTEXF_POINT));
            HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_MINFILTER, D3DTEXF_POINT));
            HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_MIPFILTER, D3DTEXF_NONE));
            HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG1));
            HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_TEXTURE));
            for (k = 0; k < 2; k++) {
                for (i = 0; i < 4; i++) { a[i].x = i & 1 ? (float)W : 0.0f; a[i].y = i & 2 ? (float)H : 0.0f; a[i].z = 0.5f; a[i].w = 1.0f; a[i].u = a[i].v = 0.5f; a[i].r = k ? 0.75f : 0.25f; }
                HR(IDirect3DDevice8_SetVertexShader(g_d, D3DFVF_XYZRHW | D3DFVF_TEX1 | D3DFVF_TEXCOORDSIZE3(0)));
                HR(IDirect3DDevice8_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, a, sizeof a[0]));
                px(W / 2, H / 2, "a volume through Direct3D 8", k ? 0 : 255, k ? 255 : 0, 0);
            }
            HR(IDirect3DDevice8_SetTexture(g_d, 0, NULL));
            HR(IDirect3DDevice8_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_MODULATE));
            n = IDirect3DVolumeTexture8_Release(vt);
            ok(n == 0, "the last reference of a volume");
        }
    }
}

/* ---------------------------------------------------------------- the hardware cursor: it is after the frame buffer, so no read-back sees it, only the return values and B's counters (scratch tool) */
static void test_cursor(void)
{
    D3DCAPS8 c;
    IDirect3DSurface8 *s = NULL, *x = NULL;
    D3DLOCKED_RECT l;
    int i, j;
    HR(IDirect3DDevice8_GetDeviceCaps(g_d, &c));
    ok((c.CursorCaps & D3DCURSORCAPS_COLOR) && (c.CursorCaps & D3DCURSORCAPS_LOWRES), "a hardware colour cursor in the caps");
    HR(IDirect3DDevice8_CreateImageSurface(g_d, 32, 32, D3DFMT_A8R8G8B8, &s));
    HR(IDirect3DDevice8_CreateImageSurface(g_d, 32, 32, D3DFMT_R5G6B5, &x));
    IDirect3DSurface8_LockRect(s, &l, NULL, 0);
    for (j = 0; j < 32; j++) for (i = 0; i < 32; i++) ((DWORD *)((uint8_t *)l.pBits + (ptrdiff_t)j * l.Pitch))[i] = i + j < 32 ? 0xFFFFFF00 : 0;
    IDirect3DSurface8_UnlockRect(s);
    ok(IDirect3DDevice8_SetCursorProperties(g_d, 0, 0, x) == D3DERR_INVALIDCALL, "a cursor must be A8R8G8B8");
    ok(IDirect3DDevice8_SetCursorProperties(g_d, 32, 0, s) == D3DERR_INVALIDCALL, "the hot spot must be inside it");
    HR(IDirect3DDevice8_SetCursorProperties(g_d, 2, 3, s));
    ok(IDirect3DDevice8_ShowCursor(g_d, TRUE) == FALSE, "ShowCursor gives the previous state: hidden");
    IDirect3DDevice8_SetCursorPosition(g_d, 300, 200, 0);
    ok(IDirect3DDevice8_ShowCursor(g_d, FALSE) == TRUE, "ShowCursor gives the previous state: shown");
    if (x) IDirect3DSurface8_Release(x);
    if (s) IDirect3DSurface8_Release(s);
}

/* ---------------------------------------------------------------- CopyRects, every way */
static void fill(IDirect3DSurface8 *s, DWORD c)
{
    D3DSURFACE_DESC d;
    D3DLOCKED_RECT l;
    UINT x, y;
    IDirect3DSurface8_GetDesc(s, &d);
    IDirect3DSurface8_LockRect(s, &l, NULL, 0);
    for (y = 0; y < d.Height; y++) for (x = 0; x < d.Width; x++) ((DWORD *)((uint8_t *)l.pBits + (ptrdiff_t)y * l.Pitch))[x] = c;
    IDirect3DSurface8_UnlockRect(s);
}

static void test_copy(void)
{
    IDirect3DSurface8 *bb = NULL, *a = NULL, *b = NULL, *rt = NULL, *x = NULL, *ls = NULL;
    IDirect3DTexture8 *t = NULL, *dx = NULL, *dd = NULL;
    D3DLOCKED_RECT l;
    RECT r = { 10, 10, 30, 30 }, r2[2] = { { 0, 0, 10, 10 }, { 20, 20, 30, 30 } };
    POINT p = { 100, 50 }, p2[2] = { { 200, 200 }, { 300, 300 } };
    flat_state();
    HR(IDirect3DDevice8_GetBackBuffer(g_d, 0, D3DBACKBUFFER_TYPE_MONO, &bb));
    HR(IDirect3DDevice8_CreateImageSurface(g_d, 64, 64, D3DFMT_X8R8G8B8, &a));
    HR(IDirect3DDevice8_CreateImageSurface(g_d, 64, 64, D3DFMT_X8R8G8B8, &b));
    HR(IDirect3DDevice8_CreateImageSurface(g_d, 64, 64, D3DFMT_R5G6B5, &x));
    ok(IDirect3DDevice8_CopyRects(g_d, a, NULL, 0, x, NULL) == D3DERR_INVALIDCALL, "different formats");
    if (x) IDirect3DSurface8_Release(x);
    x = (IDirect3DSurface8 *)1;
    ok(IDirect3DDevice8_CreateImageSurface(g_d, 8, 8, D3DFMT_V16U16, &x) == D3DERR_INVALIDCALL, "an image in a format that has no pixels here");
    fill(a, 0xFF00FF00);
    fill(b, 0xFF000000);

    /* memory to memory */
    HR(IDirect3DDevice8_CopyRects(g_d, a, &r, 1, b, &p == NULL ? NULL : NULL));
    surf_px(b, 15, 15, "memory to memory, no point: the same place", 0, 255, 0);
    surf_px(b, 5, 5, "memory to memory: nothing else", 0, 0, 0);
    /* memory to the back buffer */
    clear(0);
    HR(IDirect3DDevice8_CopyRects(g_d, a, &r, 1, bb, &p));
    px(100, 50, "memory to the back buffer: the first pixel", 0, 255, 0);
    px(119, 69, "memory to the back buffer: the last pixel", 0, 255, 0);
    px(99, 50, "memory to the back buffer: one left", 0, 0, 0);
    px(120, 69, "memory to the back buffer: one right", 0, 0, 0);
    /* several rectangles, each with its point */
    clear(0);
    HR(IDirect3DDevice8_CopyRects(g_d, a, r2, 2, bb, p2));
    px(205, 205, "two rectangles: the first", 0, 255, 0);
    px(305, 305, "two rectangles: the second", 0, 255, 0);
    px(250, 250, "two rectangles: between them", 0, 0, 0);
    /* the whole surface, at a point */
    clear(0);
    HR(IDirect3DDevice8_CopyRects(g_d, a, NULL, 0, bb, &p));
    px(100 + 63, 50 + 63, "the whole surface at a point", 0, 255, 0);
    /* the back buffer to memory, a part of it */
    clear(0xFF0000FF);
    fill(b, 0xFF000000);
    {
        RECT rb = { 300, 200, 320, 220 };
        POINT pb = { 4, 4 };
        HR(IDirect3DDevice8_CopyRects(g_d, bb, &rb, 1, b, &pb));
    }
    surf_px(b, 10, 10, "the back buffer to memory", 0, 0, 255);
    surf_px(b, 30, 30, "the back buffer to memory: nothing else", 0, 0, 0);
    /* target to target on the board */
    HR(IDirect3DDevice8_CreateRenderTarget(g_d, 128, 128, D3DFMT_X8R8G8B8, D3DMULTISAMPLE_NONE, FALSE, &rt));
    {
        RECT rb = { 0, 0, 16, 16 };
        POINT pb = { 8, 8 };
        HR(IDirect3DDevice8_SetRenderTarget(g_d, rt, NULL));
        HR(IDirect3DDevice8_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0xFFFF0000, 1.0f, 0));
        HR(IDirect3DDevice8_SetRenderTarget(g_d, bb, NULL));
        HR(IDirect3DDevice8_CopyRects(g_d, bb, &rb, 1, rt, &pb));
    }
    surf_px(rt, 12, 12, "target to target: copied", 0, 0, 255);
    surf_px(rt, 40, 40, "target to target: the rest stays", 255, 0, 0);
    /* memory to a texture's level, then sampled */
    HR(IDirect3DDevice8_CreateTexture(g_d, 64, 64, 1, 0, D3DFMT_X8R8G8B8, D3DPOOL_MANAGED, &t));
    HR(IDirect3DTexture8_GetSurfaceLevel(t, 0, &ls));
    HR(IDirect3DDevice8_CopyRects(g_d, a, NULL, 0, ls, NULL));
    clear(0);
    use_texture(t);
    quad(0, 0, 64, 64, 0.5f, 0xFFFFFFFF);
    px(32, 32, "memory to a texture", 0, 255, 0);
    flat_state();
    /* a compressed one: one 4 x 4 block of red, by whole blocks */
    HR(IDirect3DDevice8_CreateTexture(g_d, 4, 4, 1, 0, D3DFMT_DXT1, D3DPOOL_SYSTEMMEM, &dx));
    HR(IDirect3DDevice8_CreateTexture(g_d, 4, 4, 1, 0, D3DFMT_DXT1, D3DPOOL_MANAGED, &dd));
    if (dx && dd) {
        IDirect3DSurface8 *s1 = NULL, *s2 = NULL;
        static const WORD block[4] = { 0xF800, 0xF800, 0, 0 };
        IDirect3DTexture8_LockRect(dx, 0, &l, NULL, 0);
        memcpy(l.pBits, block, 8);
        IDirect3DTexture8_UnlockRect(dx, 0);
        IDirect3DTexture8_GetSurfaceLevel(dx, 0, &s1);
        IDirect3DTexture8_GetSurfaceLevel(dd, 0, &s2);
        HR(IDirect3DDevice8_CopyRects(g_d, s1, NULL, 0, s2, NULL));
        clear(0);
        use_texture(dd);
        quad(0, 0, 16, 16, 0.5f, 0xFFFFFFFF);
        px(8, 8, "a compressed block", 255, 0, 0);
        flat_state();
        IDirect3DSurface8_Release(s1);
        IDirect3DSurface8_Release(s2);
    }
    /* UpdateTexture stays Direct3D 9's */
    if (dx && dd) {
        HR(IDirect3DDevice8_UpdateTexture(g_d, (IDirect3DBaseTexture8 *)dx, (IDirect3DBaseTexture8 *)dd));
    }
    if (dx) IDirect3DTexture8_Release(dx);
    if (dd) IDirect3DTexture8_Release(dd);
    if (ls) IDirect3DSurface8_Release(ls);
    if (t) IDirect3DTexture8_Release(t);
    if (rt) IDirect3DSurface8_Release(rt);
    if (a) IDirect3DSurface8_Release(a);
    if (b) IDirect3DSurface8_Release(b);
    if (bb) IDirect3DSurface8_Release(bb);
    {
        IDirect3DSurface8 *ds = NULL;
        (void)ds;
    }
}

/* ---------------------------------------------------------------- a mesh as a game's strips against the same triangles as a list: every pixel the same.
 * Long indexed strips joined by degenerate triangles, the vertices in shuffled order (the vertex cache), through a vertex shader, perspective, some of it past the clip planes */
static uint32_t rnd(uint32_t *s) { *s = *s * 1664525u + 1013904223u; return *s >> 8; }

static uint8_t *snap(void)
{
    IDirect3DSurface8 *rt = NULL, *m = NULL;
    D3DSURFACE_DESC de;
    D3DLOCKED_RECT l;
    uint8_t *o = (uint8_t *)malloc(W * H * 4);
    int y;
    IDirect3DDevice8_GetRenderTarget(g_d, &rt);
    IDirect3DSurface8_GetDesc(rt, &de);
    IDirect3DDevice8_CreateImageSurface(g_d, de.Width, de.Height, de.Format, &m);
    IDirect3DDevice8_CopyRects(g_d, rt, NULL, 0, m, NULL);
    IDirect3DSurface8_LockRect(m, &l, NULL, D3DLOCK_READONLY);
    for (y = 0; y < H; y++) memcpy(o + (size_t)y * W * 4, (uint8_t *)l.pBits + (ptrdiff_t)y * l.Pitch, W * 4);
    IDirect3DSurface8_UnlockRect(m);
    IDirect3DSurface8_Release(m);
    IDirect3DSurface8_Release(rt);
    return o;
}

static void strip_mesh(int cols, int rows, int shuffle, int deep, uint32_t seed, const char *what)
{
    int nv = (cols + 1) * (rows + 1), i, r, c, ns = 0, nl = 0, bad = 0, base = 7, *perm = (int *)malloc((size_t)nv * sizeof(int));
    cv *v = (cv *)calloc((size_t)nv + (size_t)base, sizeof(cv));
    WORD *is = (WORD *)malloc(((size_t)rows * (2 * ((size_t)cols + 1) + 2)) * 2), *il = (WORD *)malloc((size_t)rows * cols * 6 * 2);
    IDirect3DVertexBuffer8 *vb = NULL;
    IDirect3DIndexBuffer8 *ib = NULL, *ibl = NULL;
    uint8_t *p = NULL, *a, *b;
    char s[120];
    for (i = 0; i < nv; i++) perm[i] = i;
    if (shuffle) for (i = nv - 1; i > 0; i--) { int j = (int)(rnd(&seed) % (uint32_t)(i + 1)), t = perm[i]; perm[i] = perm[j]; perm[j] = t; }
    for (r = 0; r <= rows; r++) {
        for (c = 0; c <= cols; c++) {
            cv *o = &v[base + perm[r * (cols + 1) + c]];
            float w = deep ? 0.6f + (float)(rnd(&seed) % 2000) / 1000.0f : 1.0f, z = deep ? -0.3f + (float)(rnd(&seed) % 1500) / 1000.0f : 0.5f;
            o->x = (-1.15f + 2.3f * (float)c / (float)cols + (float)(rnd(&seed) % 100) * 0.0002f - 0.01f) * w;
            o->y = (-1.15f + 2.3f * (float)r / (float)rows + (float)(rnd(&seed) % 100) * 0.0002f - 0.01f) * w;
            o->z = z * w; o->w = w;
            o->c = 0xFF000000u | (rnd(&seed) & 0xFFFFFF);
        }
    }
#define G(ro, co) ((WORD)perm[(ro) * (cols + 1) + (co)])
    for (r = 0; r < rows; r++) {
        if (r) { is[ns] = is[ns - 1]; ns++; is[ns++] = G(r, 0); }       /* the joint: two repeated indices, an even number of degenerate triangles */
        for (c = 0; c <= cols; c++) { is[ns++] = G(r, c); is[ns++] = G(r + 1, c); }
        for (c = 0; c < cols; c++) { il[nl++] = G(r, c); il[nl++] = G(r + 1, c); il[nl++] = G(r, c + 1); il[nl++] = G(r, c + 1); il[nl++] = G(r + 1, c); il[nl++] = G(r + 1, c + 1); }
    }
#undef G
    HR(IDirect3DDevice8_CreateVertexBuffer(g_d, (UINT)(nv + base) * sizeof(cv), 0, 0, D3DPOOL_MANAGED, &vb));
    IDirect3DVertexBuffer8_Lock(vb, 0, 0, &p, 0); memcpy(p, v, (size_t)(nv + base) * sizeof(cv)); IDirect3DVertexBuffer8_Unlock(vb);
    HR(IDirect3DDevice8_CreateIndexBuffer(g_d, (UINT)ns * 2, 0, D3DFMT_INDEX16, D3DPOOL_MANAGED, &ib));
    IDirect3DIndexBuffer8_Lock(ib, 0, 0, &p, 0); memcpy(p, is, (size_t)ns * 2); IDirect3DIndexBuffer8_Unlock(ib);
    HR(IDirect3DDevice8_CreateIndexBuffer(g_d, (UINT)nl * 2, 0, D3DFMT_INDEX16, D3DPOOL_MANAGED, &ibl));
    IDirect3DIndexBuffer8_Lock(ibl, 0, 0, &p, 0); memcpy(p, il, (size_t)nl * 2); IDirect3DIndexBuffer8_Unlock(ibl);
    IDirect3DDevice8_SetStreamSource(g_d, 0, vb, sizeof(cv));
    clear(0);
    IDirect3DDevice8_SetIndices(g_d, ibl, (UINT)base);
    HR(IDirect3DDevice8_DrawIndexedPrimitive(g_d, D3DPT_TRIANGLELIST, 0, (UINT)nv, 0, (UINT)nl / 3));
    a = snap();
    clear(0);
    IDirect3DDevice8_SetIndices(g_d, ib, (UINT)base);
    HR(IDirect3DDevice8_DrawIndexedPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, (UINT)nv, 0, (UINT)ns - 2));
    b = snap();
    for (i = 0; i < W * H * 4; i += 4) if (abs(a[i] - b[i]) > 4 || abs(a[i + 1] - b[i + 1]) > 4 || abs(a[i + 2] - b[i + 2]) > 4) bad++;
    snprintf(s, sizeof s, "%s: %d indices as a strip = as a list (%d pixels differ)", what, ns, bad);
    ok(bad == 0, s);
    if (bad) { printf("  first check of the list frame: centre pixel %d %d %d\n", a[(H / 2 * W + W / 2) * 4 + 2], a[(H / 2 * W + W / 2) * 4 + 1], a[(H / 2 * W + W / 2) * 4]); }
    free(a); free(b); free(perm); free(v); free(is); free(il);
    IDirect3DDevice8_SetStreamSource(g_d, 0, NULL, 0);
    IDirect3DDevice8_SetIndices(g_d, NULL, 0);
    if (vb) IDirect3DVertexBuffer8_Release(vb);
    if (ib) IDirect3DIndexBuffer8_Release(ib);
    if (ibl) IDirect3DIndexBuffer8_Release(ibl);
}

static void test_strips(void)
{
    static const DWORD f[] = { VS11, OP_MOV, DST(R_RASTOUT, 0), SRC(R_INPUT, 0), OP_MOV, DST(R_ATTROUT, 0), SRC(R_INPUT, 5), END };
    DWORD de[] = { D3DVSD_STREAM(0), D3DVSD_REG(0, D3DVSDT_FLOAT4), D3DVSD_REG(5, D3DVSDT_D3DCOLOR), D3DVSD_END() };
    DWORD h = 0;
    flat_state();
    HR(IDirect3DDevice8_CreateVertexShader(g_d, de, f, &h, 0));
    HR(IDirect3DDevice8_SetVertexShader(g_d, h));
    strip_mesh(30, 60, 0, 0, 1, "in order, flat");
    strip_mesh(30, 60, 1, 0, 2, "shuffled, flat");
    strip_mesh(30, 60, 1, 1, 3, "shuffled, perspective, clipped");
    strip_mesh(200, 8, 1, 1, 4, "wide");
    strip_mesh(3, 300, 1, 1, 5, "narrow, a joint every 8 indices");
    strip_mesh(1, 400, 1, 1, 6, "one quad wide, a joint every 4");
    strip_mesh(100, 100, 1, 1, 7, "large");
    HR(IDirect3DDevice8_SetVertexShader(g_d, FVF_TL));
    HR(IDirect3DDevice8_DeleteVertexShader(g_d, h));
}

/* ---------------------------------------------------------------- what Direct3D 8 gets from d3d9.dll's code through its own tables and structures */
static void test_extra(HWND w)
{
    typedef struct { float x, y, z, b; DWORD c; } vb1;
    typedef struct { float x, y, z; DWORD c; } vc;
    typedef struct { float x, y, z, rhw; DWORD c; } vt;
    static const D3DMATRIX id = { { { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } } };
    D3DMATRIX a = id, b = id;
    D3DPRESENT_PARAMETERS q;
    IDirect3DSwapChain8 *ch = NULL;
    IDirect3DSurface8 *rt = NULL, *ds = NULL, *bb = NULL;
    IDirect3DVertexBuffer8 *in = NULL, *out = NULL;
    vb1 v[4];
    vt t[3] = { { 0, 0, 0.5f, 1, 0xFFFF0000 }, { 200, 0, 0.5f, 1, 0xFF00FF00 }, { 0, 200, 0.5f, 1, 0xFF0000FF } };
    vc s[1] = { { 0.5f, 0, 0.5f, 0xFF102030 } };
    BYTE *p;
    int i;
    flat_state();
    /* the size of a vertex with weights: XYZB1 is one float, the colour must be found after it */
    for (i = 0; i < 4; i++) { v[i].x = i & 1 ? 0.25f : -0.25f; v[i].y = i & 2 ? 0.25f : -0.25f; v[i].z = 0.5f; v[i].b = 1.0f; v[i].c = 0xFF00FF00; }
    a.m[3][0] = -0.5f; b.m[3][0] = 0.5f;
    IDirect3DDevice8_SetTransform(g_d, D3DTS_WORLD, &a);
    IDirect3DDevice8_SetTransform(g_d, D3DTS_WORLDMATRIX(1), &b);
    IDirect3DDevice8_SetTransform(g_d, D3DTS_VIEW, &id);
    IDirect3DDevice8_SetTransform(g_d, D3DTS_PROJECTION, &id);
    IDirect3DDevice8_SetRenderState(g_d, D3DRS_VERTEXBLEND, D3DVBF_1WEIGHTS);
    IDirect3DDevice8_SetVertexShader(g_d, D3DFVF_XYZB1 | D3DFVF_DIFFUSE);
    clear(0);
    HR(IDirect3DDevice8_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, v, sizeof v[0]));
    px(W / 4, H / 2, "vertex blend, Direct3D 8: world 0", 0, 255, 0);
    px(W * 3 / 4, H / 2, "vertex blend, Direct3D 8: not world 1", 0, 0, 0);
    IDirect3DDevice8_SetRenderState(g_d, D3DRS_VERTEXBLEND, D3DVBF_DISABLE);
    IDirect3DDevice8_SetTransform(g_d, D3DTS_WORLD, &id);
    /* flat */
    IDirect3DDevice8_SetRenderState(g_d, D3DRS_SHADEMODE, D3DSHADE_FLAT);
    IDirect3DDevice8_SetVertexShader(g_d, D3DFVF_XYZRHW | D3DFVF_DIFFUSE);
    clear(0);
    HR(IDirect3DDevice8_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, t, sizeof t[0]));
    px(180, 10, "flat, Direct3D 8", 255, 0, 0);
    IDirect3DDevice8_SetRenderState(g_d, D3DRS_SHADEMODE, D3DSHADE_GOURAUD);
    /* ProcessVertices: the output's layout is its FVF, there is no declaration */
    HR(IDirect3DDevice8_CreateVertexBuffer(g_d, sizeof s, 0, 0, D3DPOOL_MANAGED, &in));
    HR(IDirect3DDevice8_CreateVertexBuffer(g_d, sizeof(vt) * 2, 0, D3DFVF_XYZRHW | D3DFVF_DIFFUSE, D3DPOOL_MANAGED, &out));
    IDirect3DVertexBuffer8_Lock(in, 0, 0, &p, 0); memcpy(p, s, sizeof s); IDirect3DVertexBuffer8_Unlock(in);
    IDirect3DDevice8_SetVertexShader(g_d, D3DFVF_XYZ | D3DFVF_DIFFUSE);
    IDirect3DDevice8_SetStreamSource(g_d, 0, in, sizeof s[0]);
    HR(g_d->lpVtbl->ProcessVertices(g_d, 0, 1, 1, out, 0));     /* d3d8.h's macro misspells it */
    IDirect3DVertexBuffer8_Lock(out, 0, 0, &p, D3DLOCK_READONLY);
    ok(((vt *)p)[1].x == W * 0.75f && ((vt *)p)[1].y == H * 0.5f, "ProcessVertices, Direct3D 8: the position");
    ok(((vt *)p)[1].c == 0xFF102030, "ProcessVertices, Direct3D 8: the colour");
    IDirect3DVertexBuffer8_Unlock(out);
    IDirect3DDevice8_SetStreamSource(g_d, 0, NULL, 0);
    IDirect3DVertexBuffer8_Release(in);
    IDirect3DVertexBuffer8_Release(out);
    /* an additional swap chain behind IDirect3DSwapChain8, whose Present has no flags: a wrong table would corrupt the stack */
    IDirect3DDevice8_GetRenderTarget(g_d, &rt);
    IDirect3DDevice8_GetDepthStencilSurface(g_d, &ds);
    memset(&q, 0, sizeof q);
    q.BackBufferWidth = 320; q.BackBufferHeight = 200; q.BackBufferFormat = D3DFMT_A8R8G8B8; q.SwapEffect = D3DSWAPEFFECT_COPY; q.Windowed = TRUE; q.hDeviceWindow = w;
    HR(IDirect3DDevice8_CreateAdditionalSwapChain(g_d, &q, &ch));
    if (ch) {
        HR(IDirect3DSwapChain8_GetBackBuffer(ch, 0, D3DBACKBUFFER_TYPE_MONO, &bb));
        HR(IDirect3DDevice8_SetRenderTarget(g_d, bb, NULL));
        IDirect3DDevice8_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(0, 0, 40), 1.0f, 0);
        quad(10, 10, 50, 50, 0.5f, D3DCOLOR_XRGB(0, 200, 0));
        surf_px(bb, 20, 20, "additional swap chain, Direct3D 8", 0, 200, 0);
        HR(IDirect3DSwapChain8_Present(ch, NULL, NULL, NULL, NULL));
        HR(IDirect3DDevice8_SetRenderTarget(g_d, rt, ds));
        IDirect3DSurface8_Release(bb);
        IDirect3DSwapChain8_Release(ch);
    }
    IDirect3DSurface8_Release(rt);
    if (ds) IDirect3DSurface8_Release(ds);
    HR(IDirect3DDevice8_SetVertexShader(g_d, FVF_TL));
}

/* ---------------------------------------------------------------- present, front buffer, reset */
static void present_params(D3DPRESENT_PARAMETERS *p, HWND w, D3DFORMAT f, D3DSWAPEFFECT sw)
{
    memset(p, 0, sizeof *p);
    p->BackBufferWidth = W; p->BackBufferHeight = H; p->BackBufferFormat = f; p->BackBufferCount = 1;
    p->SwapEffect = sw; p->hDeviceWindow = w; p->Windowed = TRUE;
    p->EnableAutoDepthStencil = TRUE; p->AutoDepthStencilFormat = D3DFMT_D24S8;
}

static void front(const char *what, int r, int g, int b)
{
    IDirect3DSurface8 *s = NULL;
    D3DLOCKED_RECT l;
    uint8_t *p;
    HR(IDirect3DDevice8_CreateImageSurface(g_d, W, H, D3DFMT_A8R8G8B8, &s));
    HR(IDirect3DDevice8_GetFrontBuffer(g_d, s));
    IDirect3DSurface8_LockRect(s, &l, NULL, D3DLOCK_READONLY);
    p = (uint8_t *)l.pBits + 100 * (ptrdiff_t)l.Pitch + 400;
    g_checks++;
    if (abs(p[2] - r) > 8 || abs(p[1] - g) > 8 || abs(p[0] - b) > 8) { g_bad++; printf("FAIL %s: %d %d %d, expected %d %d %d\n", what, p[2], p[1], p[0], r, g, b); }
    IDirect3DSurface8_UnlockRect(s);
    IDirect3DSurface8_Release(s);
}

static void test_present(HWND w)
{
    D3DPRESENT_PARAMETERS p;
    UINT base = 9;
    IDirect3DIndexBuffer8 *b = NULL;
    DWORD v = 9;
    clear(D3DCOLOR_XRGB(10, 200, 30));
    HR(IDirect3DDevice8_Present(g_d, NULL, NULL, NULL, NULL));
    clear(D3DCOLOR_XRGB(200, 10, 30));
    front("the front buffer is the frame shown, not the one being drawn", 10, 200, 30);
    HR(IDirect3DDevice8_SetIndices(g_d, g_ib, 4));
    HR(IDirect3DDevice8_SetRenderState(g_d, D3DRS_ZBIAS, 5));
    present_params(&p, w, D3DFMT_X8R8G8B8, D3DSWAPEFFECT_COPY_VSYNC);
    HR(IDirect3DDevice8_Reset(g_d, &p));
    HR(IDirect3DDevice8_GetIndices(g_d, &b, &base));
    ok(!b && base == 0, "Reset clears the indices and their base vertex");
    if (b) IDirect3DIndexBuffer8_Release(b);
    HR(IDirect3DDevice8_GetRenderState(g_d, D3DRS_ZBIAS, &v));
    ok(v == 0, "Reset clears ZBIAS");
    flat_state();
    IDirect3DDevice8_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(30, 60, 200), 1.0f, 0);
    HR(IDirect3DDevice8_Present(g_d, NULL, NULL, NULL, NULL));
    front("COPY_VSYNC: the scan-out image", 30, 60, 200);
    present_params(&p, w, D3DFMT_R5G6B5, D3DSWAPEFFECT_DISCARD);
    HR(IDirect3DDevice8_Reset(g_d, &p));
    IDirect3DDevice8_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(255, 255, 0), 1.0f, 0);
    HR(IDirect3DDevice8_Present(g_d, NULL, NULL, NULL, NULL));
    IDirect3DDevice8_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    front("5:6:5: the front buffer comes as A8R8G8B8", 255, 255, 0);
}

static void test_exports(void)
{
    typedef HRESULT (WINAPI *vv_t)(const DWORD *, const DWORD *, const D3DCAPS8 *, BOOL, char **);
    typedef HRESULT (WINAPI *vp_t)(const DWORD *, const D3DCAPS8 *, BOOL, char **);
    HMODULE m = GetModuleHandleA("d3d8.dll");
    vv_t vv = (vv_t)GetProcAddress(m, "ValidateVertexShader");
    vp_t vp = (vp_t)GetProcAddress(m, "ValidatePixelShader");
    static const DWORD good[] = { VS11, END }, bad[] = { 0xFFFE0300, END }, pgood[] = { PS11, END };
    char *e = NULL;
    ok(vv && vp && GetProcAddress(m, "DebugSetMute") && GetProcAddress(m, "Direct3D8EnableMaximizedWindowedModeShim"), "the exports of the system's d3d8.dll");
    if (!vv || !vp) return;
    ok(vv(good, NULL, NULL, FALSE, NULL) == S_OK, "ValidateVertexShader takes vs.1.1");
    ok(vv(bad, NULL, NULL, TRUE, &e) == E_FAIL && e && e[0], "ValidateVertexShader refuses vs.3.0");
    if (e) HeapFree(GetProcessHeap(), 0, e);
    ok(vp(pgood, NULL, FALSE, NULL) == S_OK, "ValidatePixelShader takes ps.1.1");
}

int main(void)
{
    WNDCLASSA c;
    HWND w;
    D3DPRESENT_PARAMETERS p;
    memset(&c, 0, sizeof c);
    c.lpfnWndProc = DefWindowProcA; c.lpszClassName = "d3d8_test"; c.hInstance = GetModuleHandleA(NULL);
    RegisterClassA(&c);
    w = CreateWindowA("d3d8_test", "d3d8_test", WS_OVERLAPPEDWINDOW, 0, 0, W, H, NULL, NULL, c.hInstance, NULL);
    g_o = Direct3DCreate8(D3D_SDK_VERSION);
    if (!g_o) { printf("FAIL Direct3DCreate8\n"); return 1; }
    test_d3d();
    present_params(&p, w, D3DFMT_X8R8G8B8, D3DSWAPEFFECT_DISCARD);
    if (IDirect3D8_CreateDevice(g_o, 0, D3DDEVTYPE_HAL, w, D3DCREATE_HARDWARE_VERTEXPROCESSING, &p, &g_d) != D3D_OK) { printf("FAIL CreateDevice\n"); return 1; }
    test_device();
    test_samplers();
    test_indices();
    test_kept();
    test_kept_unseen();
    test_targets();
    test_shaders();
    test_relative();
    test_blocks();
    test_objects();
    test_cursor();
    test_copy();
    test_strips();
    test_extra(w);
    test_present(w);
    test_exports();
    IDirect3DVertexBuffer8_Release(g_vb);
    IDirect3DIndexBuffer8_Release(g_ib);
    IDirect3DDevice8_Release(g_d);
    IDirect3D8_Release(g_o);
    printf("%d checks, %d failed\n", g_checks, g_bad);
    return g_bad != 0;
}
