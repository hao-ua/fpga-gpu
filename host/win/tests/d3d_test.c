/* d3d_test — d3d9.dll through the real API on the boards: scenes with known pixels, read back through GetRenderTargetData as a game would.
 * Shaders are real bytecode: D3DXAssembleShader of the system's d3dx9_43.dll, loaded at run time.
 * Tolerances are +-2 of 255: the shaders compute in FP24. */
#define CINTERFACE
#define COBJMACROS
#include <windows.h>
#include <d3d9.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 640
#define H 480

static int g_bad, g_checks;
static IDirect3DDevice9 *g_d;
static IDirect3DSurface9 *g_sys;

typedef struct ID3DXBuffer ID3DXBuffer;
typedef struct ID3DXBufferVtbl { HRESULT (WINAPI *QI)(ID3DXBuffer *, REFIID, void **); ULONG (WINAPI *AddRef)(ID3DXBuffer *); ULONG (WINAPI *Release)(ID3DXBuffer *); void *(WINAPI *GetBufferPointer)(ID3DXBuffer *); DWORD (WINAPI *GetBufferSize)(ID3DXBuffer *); } ID3DXBufferVtbl;
struct ID3DXBuffer { const ID3DXBufferVtbl *lpVtbl; };
typedef HRESULT (WINAPI *assemble_t)(const char *, UINT, const void *, void *, DWORD, ID3DXBuffer **, ID3DXBuffer **);
static assemble_t g_asm;

static void ok(int c, const char *what) { g_checks++; if (!c) { g_bad++; printf("FAIL %s\n", what); } }
#define HR(x) do { HRESULT h_ = (x); g_checks++; if (FAILED(h_)) { g_bad++; printf("FAIL %s = %08lx\n", #x, (unsigned long)h_); } } while (0)

static void px(int x, int y, const char *what, int r, int g, int b)
{
    IDirect3DSurface9 *bb = NULL, *rt = NULL;
    D3DLOCKED_RECT l;
    uint8_t *p;
    IDirect3DDevice9_GetRenderTarget(g_d, 0, &rt);
    bb = rt;
    g_checks++;
    if (FAILED(IDirect3DDevice9_GetRenderTargetData(g_d, bb, g_sys))) { g_bad++; printf("FAIL %s: GetRenderTargetData\n", what); IDirect3DSurface9_Release(rt); return; }
    IDirect3DSurface9_LockRect(g_sys, &l, NULL, D3DLOCK_READONLY);
    p = (uint8_t *)l.pBits + (ptrdiff_t)y * l.Pitch + (ptrdiff_t)x * 4;
    if (abs(p[2] - r) > 2 || abs(p[1] - g) > 2 || abs(p[0] - b) > 2) { g_bad++; printf("FAIL %s: pixel (%d, %d) = %d %d %d, expected %d %d %d\n", what, x, y, p[2], p[1], p[0], r, g, b); }
    IDirect3DSurface9_UnlockRect(g_sys);
    IDirect3DSurface9_Release(rt);
}

typedef struct { float x, y, z, w; DWORD c; float u, v; } tlv;
#define FVF_TL (D3DFVF_XYZRHW | D3DFVF_DIFFUSE | D3DFVF_TEX1)

static void quad(float x0, float y0, float x1, float y1, float z, DWORD c)
{
    tlv v[4] = { { x0, y0, z, 1, c, 0, 0 }, { x1, y0, z, 1, c, 1, 0 }, { x0, y1, z, 1, c, 0, 1 }, { x1, y1, z, 1, c, 1, 1 } };
    IDirect3DDevice9_SetFVF(g_d, FVF_TL);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, v, sizeof v[0]));
}

static IDirect3DTexture9 *tex1(D3DFORMAT f, const void *data, int bytes)
{
    IDirect3DTexture9 *t = NULL;
    D3DLOCKED_RECT l;
    HR(IDirect3DDevice9_CreateTexture(g_d, 1, 1, 1, 0, f, D3DPOOL_MANAGED, &t, NULL));
    if (!t) return NULL;
    IDirect3DTexture9_LockRect(t, 0, &l, NULL, 0);
    memcpy(l.pBits, data, (size_t)bytes);
    IDirect3DTexture9_UnlockRect(t, 0);
    return t;
}

static void show(IDirect3DTexture9 *t, const char *what, int r, int g, int b)
{
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)t);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
    quad(0, 0, W, H, 0.5f, 0xFFFFFFFF);
    px(100, 100, what, r, g, b);
    IDirect3DDevice9_SetTexture(g_d, 0, NULL);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_MODULATE);
    if (t) IDirect3DTexture9_Release(t);
}

static void test_clear(void)
{
    D3DRECT r = { 100, 100, 200, 200 };
    HR(IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, D3DCOLOR_XRGB(51, 102, 153), 1.0f, 0));
    px(10, 10, "clear", 51, 102, 153);
    HR(IDirect3DDevice9_Clear(g_d, 1, &r, D3DCLEAR_TARGET, D3DCOLOR_XRGB(255, 0, 0), 1.0f, 0));
    px(150, 150, "clear rectangle: inside", 255, 0, 0);
    px(50, 50, "clear rectangle: outside", 51, 102, 153);
}

/* row 0 is the top, pixel centres are at integer coordinates */
static void test_tl(void)
{
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ZENABLE, FALSE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    quad(10.0f, 20.0f, 110.0f, 70.0f, 0.5f, D3DCOLOR_XRGB(0, 255, 0));
    px(10, 20, "pretransformed: the first pixel", 0, 255, 0);
    px(109, 69, "pretransformed: the last pixel", 0, 255, 0);
    px(9, 20, "pretransformed: one left", 0, 0, 0);
    px(110, 69, "pretransformed: one right", 0, 0, 0);
    px(50, 19, "pretransformed: one above", 0, 0, 0);
    px(50, 70, "pretransformed: one below", 0, 0, 0);
    px(50, H - 40, "row 0 is the top", 0, 0, 0);
    /* Direct3D 9 samples at integer coordinates, GL at + 0.5: an edge at 10.25 covers pixel 11 first here, pixel 10 there. Whole-number edges cannot tell the two apart */
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    quad(10.25f, 20.25f, 110.25f, 70.25f, 0.5f, D3DCOLOR_XRGB(0, 255, 0));
    px(10, 40, "half pixel: left edge at 10.25 leaves pixel 10 out", 0, 0, 0);
    px(11, 40, "half pixel: pixel 11 is in", 0, 255, 0);
    px(110, 40, "half pixel: right edge at 110.25 takes pixel 110", 0, 255, 0);
    px(111, 40, "half pixel: pixel 111 is out", 0, 0, 0);
    px(50, 20, "half pixel: top edge at 20.25 leaves row 20 out", 0, 0, 0);
    px(50, 21, "half pixel: row 21 is in", 0, 255, 0);
    px(50, 70, "half pixel: bottom edge at 70.25 takes row 70", 0, 255, 0);
    px(50, 71, "half pixel: row 71 is out", 0, 0, 0);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_MODULATE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ZENABLE, TRUE);
}

static void test_formats(void)
{
    static const uint8_t argb[4] = { 30, 20, 10, 40 };
    static const WORD c565 = 16 << 11 | 32 << 5 | 8, c1555 = 1u << 15 | 31 << 10 | 15, c4444 = 0x4123;
    static const uint8_t l8[1] = { 100 }, dxt1[8] = { 0x00, 0xF8, 0x1F, 0x00, 0, 0, 0, 0 };
    IDirect3DTexture9 *t = NULL;
    D3DLOCKED_RECT lr;
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ZENABLE, FALSE);
    show(tex1(D3DFMT_A8R8G8B8, argb, 4), "A8R8G8B8", 10, 20, 30);
    show(tex1(D3DFMT_X8R8G8B8, argb, 4), "X8R8G8B8", 10, 20, 30);
    show(tex1(D3DFMT_R5G6B5, &c565, 2), "R5G6B5", 132, 130, 66);
    show(tex1(D3DFMT_A1R5G5B5, &c1555, 2), "A1R5G5B5", 255, 0, 123);
    show(tex1(D3DFMT_A4R4G4B4, &c4444, 2), "A4R4G4B4", 17, 34, 51);
    show(tex1(D3DFMT_L8, l8, 1), "L8", 100, 100, 100);
    {   /* what B has no format for, in the closest it has: A4L4 in RGBA4444, R3G3B2 in RGB565, A8R3G3B2 in RGBA8 */
        static const uint8_t a4l4[1] = { 0x9C }, r332[1] = { 0xAD };
        static const WORD a8r332 = 0x80E0;
        show(tex1(D3DFMT_A4L4, a4l4, 1), "A4L4 in RGBA4444", 204, 204, 204);
        show(tex1(D3DFMT_R3G3B2, r332, 1), "R3G3B2 in RGB565", 181, 109, 82);
        show(tex1(D3DFMT_A8R3G3B2, &a8r332, 2), "A8R3G3B2 in RGBA8", 255, 0, 0);
    }
    {   /* ColorFill of a 16-bit target */
        IDirect3DSurface9 *s = NULL;
        HR(IDirect3DDevice9_CreateTexture(g_d, 1, 1, 1, D3DUSAGE_RENDERTARGET, D3DFMT_R5G6B5, D3DPOOL_DEFAULT, &t, NULL));
        if (t) {
            IDirect3DTexture9_GetSurfaceLevel(t, 0, &s);
            HR(IDirect3DDevice9_ColorFill(g_d, s, NULL, D3DCOLOR_XRGB(255, 0, 255)));
            IDirect3DSurface9_Release(s);
        }
        show(t, "ColorFill of an R5G6B5 target", 255, 0, 255);
    }
    {
        IDirect3D9 *d9 = NULL;
        IDirect3DDevice9_GetDirect3D(g_d, &d9);
        if (d9) {
            ok(IDirect3D9_CheckDeviceType(d9, 0, D3DDEVTYPE_HAL, D3DFMT_X1R5G5B5, D3DFMT_X1R5G5B5, TRUE) == D3D_OK, "a 5:5:5 display and back buffer, stored 5:6:5");
            ok(IDirect3D9_CheckDeviceFormat(d9, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, 0, D3DRTYPE_TEXTURE, D3DFMT_A4L4) == D3D_OK, "A4L4 textures");
            IDirect3D9_Release(d9);
        }
    }
    HR(IDirect3DDevice9_CreateTexture(g_d, 4, 4, 1, 0, D3DFMT_DXT1, D3DPOOL_MANAGED, &t, NULL));
    if (t) {
        IDirect3DTexture9_LockRect(t, 0, &lr, NULL, 0);
        memcpy(lr.pBits, dxt1, 8);
        IDirect3DTexture9_UnlockRect(t, 0);
    }
    show(t, "DXT1", 255, 0, 0);
    {   /* rows must be tight: no pitch in the TMU */
        static const DWORD grid[3][5] = { { 0xFF0A0000, 0xFF140000, 0xFF1E0000, 0xFF280000, 0xFF320000 }, { 0xFF003C00, 0xFF004600, 0xFF005000, 0xFF005A00, 0xFF006400 }, { 0xFF00006E, 0xFF000078, 0xFF000082, 0xFF00008C, 0xFF000096 } };
        int y;
        HR(IDirect3DDevice9_CreateTexture(g_d, 5, 3, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &t, NULL));
        IDirect3DTexture9_LockRect(t, 0, &lr, NULL, 0);
        for (y = 0; y < 3; y++) memcpy((uint8_t *)lr.pBits + y * lr.Pitch, grid[y], 20);
        IDirect3DTexture9_UnlockRect(t, 0);
        IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)t);
        IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
        IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP);
        IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);
        quad(0, 0, W, H, 0.5f, 0xFFFFFFFF);
        px(W / 10, H / 6, "texture rows: first row, first texel", 10, 0, 0);
        px(W * 9 / 10, H * 5 / 6, "texture rows: last row, last texel", 0, 0, 150);
        px(W / 2, H / 2, "texture rows: the middle", 0, 80, 0);
        IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_ADDRESSU, D3DTADDRESS_WRAP);
        IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_ADDRESSV, D3DTADDRESS_WRAP);
        IDirect3DDevice9_SetTexture(g_d, 0, NULL);
        IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_MODULATE);
        IDirect3DTexture9_Release(t);
    }
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ZENABLE, TRUE);
}

static void test_stages(void)
{
    static const uint8_t half[4] = { 128, 128, 128, 128 };
    IDirect3DTexture9 *t = tex1(D3DFMT_A8R8G8B8, half, 4);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ZENABLE, FALSE);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(0, 0, 0), 1.0f, 0);
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)t);
    quad(0, 0, W, H, 0.5f, D3DCOLOR_XRGB(255, 128, 0));
    px(100, 100, "default stage: modulate", 128, 64, 0);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_ADD);
    quad(0, 0, W, H, 0.5f, D3DCOLOR_XRGB(64, 128, 0));
    px(100, 100, "add", 192, 255, 128);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_MODULATE2X);
    quad(0, 0, W, H, 0.5f, D3DCOLOR_XRGB(100, 200, 0));
    px(100, 100, "modulate 2x", 100, 200, 0);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG2);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG2, D3DTA_TFACTOR);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_TEXTUREFACTOR, D3DCOLOR_XRGB(10, 20, 30));
    quad(0, 0, W, H, 0.5f, 0xFFFFFFFF);
    px(100, 100, "texture factor", 10, 20, 30);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG2, D3DTA_CURRENT);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_MODULATE);
    IDirect3DDevice9_SetTexture(g_d, 0, NULL);
    IDirect3DTexture9_Release(t);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ZENABLE, TRUE);
}

static void test_ops(void)
{
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_LIGHTING, FALSE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    /* depth */
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
    quad(0, 0, 200, 200, 0.25f, D3DCOLOR_XRGB(255, 0, 0));
    quad(100, 100, 300, 300, 0.75f, D3DCOLOR_XRGB(0, 255, 0));
    px(150, 150, "depth: nearer stays", 255, 0, 0);
    px(250, 250, "depth: behind nothing", 0, 255, 0);
    /* blend */
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, D3DCOLOR_XRGB(128, 128, 128), 1.0f, 0);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ALPHABLENDENABLE, TRUE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_ALPHAARG1, D3DTA_DIFFUSE);
    quad(0, 0, W, H, 0.5f, D3DCOLOR_ARGB(128, 255, 0, 0));
    px(100, 100, "alpha blend", 191, 64, 64);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ALPHABLENDENABLE, FALSE);
    /* alpha test */
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ALPHATESTENABLE, TRUE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ALPHAFUNC, D3DCMP_GREATER);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ALPHAREF, 100);
    quad(0, 0, 100, 100, 0.5f, D3DCOLOR_ARGB(50, 255, 255, 255));
    quad(100, 0, 200, 100, 0.5f, D3DCOLOR_ARGB(200, 255, 255, 255));
    px(50, 50, "alpha test: fails", 0, 0, 0);
    px(150, 50, "alpha test: passes", 255, 255, 255);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ALPHATESTENABLE, FALSE);
    /* scissor */
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
    {
        RECT r = { 50, 60, 100, 110 };
        IDirect3DDevice9_SetScissorRect(g_d, &r);
        IDirect3DDevice9_SetRenderState(g_d, D3DRS_SCISSORTESTENABLE, TRUE);
        quad(0, 0, W, H, 0.5f, D3DCOLOR_XRGB(0, 0, 255));
        IDirect3DDevice9_SetRenderState(g_d, D3DRS_SCISSORTESTENABLE, FALSE);
        px(75, 80, "scissor: inside", 0, 0, 255);
        px(75, 30, "scissor: above, y is down", 0, 0, 0);
    }
    /* stencil: mark, then draw where marked */
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER | D3DCLEAR_STENCIL, 0, 1.0f, 0);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_STENCILENABLE, TRUE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_STENCILFUNC, D3DCMP_ALWAYS);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_STENCILREF, 1);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_STENCILPASS, D3DSTENCILOP_REPLACE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_COLORWRITEENABLE, 0);
    quad(0, 0, 100, H, 0.5f, 0xFFFFFFFF);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_COLORWRITEENABLE, 0xF);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_STENCILFUNC, D3DCMP_EQUAL);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_STENCILPASS, D3DSTENCILOP_KEEP);
    quad(0, 0, W, H, 0.25f, D3DCOLOR_XRGB(255, 255, 0));
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_STENCILENABLE, FALSE);
    px(50, 50, "stencil: inside", 255, 255, 0);
    px(300, 50, "stencil: outside", 0, 0, 0);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_MODULATE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
}

/* world, view, projection, and the cull convention: a clockwise triangle on the screen is the front */
static void test_3d(void)
{
    typedef struct { float x, y, z; DWORD c; } v3;
    static const D3DMATRIX id = { { { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } } };
    v3 cw[3] = { { -0.5f, -0.5f, 0.5f, 0xFFFF0000 }, { 0.0f, 0.5f, 0.5f, 0xFFFF0000 }, { 0.5f, -0.5f, 0.5f, 0xFFFF0000 } };
    v3 ccw[3] = { { -0.5f, -0.5f, 0.5f, 0xFF00FF00 }, { 0.5f, -0.5f, 0.5f, 0xFF00FF00 }, { 0.0f, 0.5f, 0.5f, 0xFF00FF00 } };
    D3DMATRIX t = id;
    IDirect3DDevice9_SetTransform(g_d, D3DTS_WORLD, &id);
    IDirect3DDevice9_SetTransform(g_d, D3DTS_VIEW, &id);
    IDirect3DDevice9_SetTransform(g_d, D3DTS_PROJECTION, &id);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_LIGHTING, FALSE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZ | D3DFVF_DIFFUSE);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, cw, sizeof cw[0]));
    px(W / 2, H / 2, "clockwise, default cull: drawn", 255, 0, 0);
    px(W / 2, H / 8, "y up in clip space: above the triangle", 0, 0, 0);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, ccw, sizeof ccw[0]));
    px(W / 2, H / 2, "counter-clockwise, default cull: culled", 0, 0, 0);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_CULLMODE, D3DCULL_CW);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, ccw, sizeof ccw[0]));
    px(W / 2, H / 2, "counter-clockwise, D3DCULL_CW: drawn", 0, 255, 0);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_CULLMODE, D3DCULL_NONE);
    /* a translation through the world matrix: row-vector convention */
    t.m[3][0] = 0.5f;
    IDirect3DDevice9_SetTransform(g_d, D3DTS_WORLD, &t);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, cw, sizeof cw[0]));
    px(W * 3 / 4, H / 2, "world translation", 255, 0, 0);
    px(W / 4, H / 2, "world translation: not here", 0, 0, 0);
    IDirect3DDevice9_SetTransform(g_d, D3DTS_WORLD, &id);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_CULLMODE, D3DCULL_CCW);
    /* viewport: its rectangle, and z through MinZ .. MaxZ */
    {
        D3DVIEWPORT9 v = { 100, 50, 200, 100, 0.0f, 1.0f }, all = { 0, 0, W, H, 0.0f, 1.0f };
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
        IDirect3DDevice9_SetViewport(g_d, &v);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, cw, sizeof cw[0]));
        IDirect3DDevice9_SetViewport(g_d, &all);
        px(200, 100, "viewport: inside", 255, 0, 0);
        px(200, 300, "viewport: outside", 0, 0, 0);
        /* board A clips x and y only far outside the target: everything stops at the viewport all the same */
        {
            v3 big[3] = { { -5.0f, -5.0f, 0.5f, 0xFF00FF00 }, { 0.0f, 5.0f, 0.5f, 0xFF00FF00 }, { 5.0f, -5.0f, 0.5f, 0xFF00FF00 } }, across[2] = { { -3.0f, 0.0f, 0.5f, 0xFF00FF00 }, { 3.0f, 0.0f, 0.5f, 0xFF00FF00 } };
            IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
            IDirect3DDevice9_SetViewport(g_d, &v);
            HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, big, sizeof big[0]));
            IDirect3DDevice9_SetViewport(g_d, &all);
            px(200, 100, "viewport: a large triangle, inside", 0, 255, 0);
            px(100, 50, "viewport: its corner", 0, 255, 0);
            px(299, 149, "viewport: its other corner", 0, 255, 0);
            px(99, 100, "viewport: left of it", 0, 0, 0);
            px(300, 100, "viewport: right of it", 0, 0, 0);
            px(200, 49, "viewport: above", 0, 0, 0);
            px(200, 150, "viewport: below", 0, 0, 0);
            IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
            IDirect3DDevice9_SetViewport(g_d, &v);
            HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_LINELIST, 1, across, sizeof across[0]));
            IDirect3DDevice9_SetViewport(g_d, &all);
            px(99, 100, "viewport: a line, left of it", 0, 0, 0);
            px(300, 100, "viewport: a line, right of it", 0, 0, 0);
            IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
            IDirect3DDevice9_SetViewport(g_d, &v);
            quad(0, 0, W, H, 0.5f, 0xFF0000FF);
            IDirect3DDevice9_SetViewport(g_d, &all);
            IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZ | D3DFVF_DIFFUSE);
            px(200, 100, "viewport: pretransformed vertices, inside", 0, 0, 255);
            px(99, 100, "viewport: pretransformed vertices, outside", 0, 0, 0);
        }
    }
    /* z behind the near plane is clipped: 0 .. w, not -w .. w */
    {
        v3 near_t[3] = { { -0.5f, -0.5f, -0.5f, 0xFFFFFFFF }, { 0.0f, 0.5f, -0.5f, 0xFFFFFFFF }, { 0.5f, -0.5f, -0.5f, 0xFFFFFFFF } };
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, near_t, sizeof near_t[0]));
        px(W / 2, H / 2, "z below 0 is clipped", 0, 0, 0);
    }
    /* and beyond the far plane (the library keeps depth reversed and asks board A for the plane z' >= 0 of that count, VS_STATE's z01; a board without it would draw what lies
       beyond, at the far depth). The second triangle reaches z = 1 a third of the way to its tip: x = -0.083 in clip space, pixel 293 */
    {
        v3 far_t[3] = { { -0.5f, -0.5f, 1.5f, 0xFFFFFFFF }, { 0.0f, 0.5f, 1.5f, 0xFFFFFFFF }, { 0.5f, -0.5f, 1.5f, 0xFFFFFFFF } };
        v3 thru[3] = { { -0.5f, -0.5f, 0.5f, 0xFFFFFFFF }, { -0.5f, 0.5f, 0.5f, 0xFFFFFFFF }, { 0.75f, 0.0f, 2.0f, 0xFFFFFFFF } };
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, far_t, sizeof far_t[0]));
        px(W / 2, H / 2, "z above 1 is clipped", 0, 0, 0);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, thru, sizeof thru[0]));
        px(200, H / 2, "a triangle through the far plane: its near part", 255, 255, 255);
        px(285, H / 2, "a triangle through the far plane: just before the plane", 255, 255, 255);
        px(300, H / 2, "a triangle through the far plane: just beyond is cut", 0, 0, 0);
        px(400, H / 2, "a triangle through the far plane: beyond is cut", 0, 0, 0);
    }
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_MODULATE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
}

static void test_light(void)
{
    typedef struct { float x, y, z, nx, ny, nz; } vn;
    vn q[4] = { { -1, -1, 0.5f, 0, 0, -1 }, { -1, 1, 0.5f, 0, 0, -1 }, { 1, -1, 0.5f, 0, 0, -1 }, { 1, 1, 0.5f, 0, 0, -1 } };
    D3DLIGHT9 l;
    D3DMATERIAL9 m;
    memset(&l, 0, sizeof l); memset(&m, 0, sizeof m);
    l.Type = D3DLIGHT_DIRECTIONAL; l.Diffuse.r = 1; l.Diffuse.g = 0.5f; l.Diffuse.b = 0.25f; l.Direction.z = 1;
    m.Diffuse.r = m.Diffuse.g = m.Diffuse.b = m.Diffuse.a = 1; m.Ambient.b = 0.5f;
    IDirect3DDevice9_SetLight(g_d, 0, &l);
    IDirect3DDevice9_LightEnable(g_d, 0, TRUE);
    IDirect3DDevice9_SetMaterial(g_d, &m);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_LIGHTING, TRUE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_AMBIENT, D3DCOLOR_XRGB(0, 0, 255));
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_CULLMODE, D3DCULL_NONE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZ | D3DFVF_NORMAL);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
    px(W / 2, H / 2, "directional light + ambient", 255, 128, 191);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_LIGHTING, FALSE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_CULLMODE, D3DCULL_CCW);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_MODULATE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
}

/* a render target, its pixels read back and sampled upright */
static void test_target(void)
{
    IDirect3DTexture9 *t = NULL, *n = NULL;
    IDirect3DSurface9 *s = NULL, *back = NULL, *ds = NULL;
    IDirect3DDevice9_GetRenderTarget(g_d, 0, &back);
    IDirect3DDevice9_GetDepthStencilSurface(g_d, &ds);
    HR(IDirect3DDevice9_CreateTexture(g_d, 53, 31, 1, D3DUSAGE_RENDERTARGET, D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &t, NULL));
    IDirect3DTexture9_GetSurfaceLevel(t, 0, &s);
    HR(IDirect3DDevice9_SetRenderTarget(g_d, 0, s));
    IDirect3DDevice9_SetDepthStencilSurface(g_d, NULL);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ZENABLE, FALSE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(0, 0, 255), 1.0f, 0);
    quad(0, 0, 53, 10, 0.5f, D3DCOLOR_XRGB(255, 0, 0));          /* the top rows */
    IDirect3DDevice9_SetRenderTarget(g_d, 0, back);
    IDirect3DDevice9_SetDepthStencilSurface(g_d, ds);
    IDirect3DSurface9_Release(ds);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)t);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);
    quad(0, 0, W, H, 0.5f, 0xFFFFFFFF);
    px(W / 2, 20, "render target sampled: its top is the top", 255, 0, 0);
    px(W / 2, H - 20, "render target sampled: the rest", 0, 0, 255);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_ADDRESSU, D3DTADDRESS_WRAP);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_ADDRESSV, D3DTADDRESS_WRAP);
    IDirect3DDevice9_SetTexture(g_d, 0, NULL);
    IDirect3DSurface9_Release(s);
    IDirect3DTexture9_Release(t);
    (void)n;
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_MODULATE);
    IDirect3DSurface9_Release(back);
}

static DWORD *assemble(const char *text)
{
    ID3DXBuffer *code = NULL, *errs = NULL;
    HRESULT h;
    if (!g_asm) return NULL;
    h = g_asm(text, (UINT)strlen(text), NULL, NULL, 0, &code, &errs);
    if (FAILED(h)) { g_bad++; printf("FAIL assemble: %s\n", errs ? (char *)errs->lpVtbl->GetBufferPointer(errs) : "?"); return NULL; }
    return (DWORD *)code->lpVtbl->GetBufferPointer(code);
}

static void test_shaders(void)
{
    typedef struct { float x, y, z; DWORD c; float u, v; } vc;
    vc q[4] = { { -1, -1, 0.5f, 0xFF204060, 0, 1 }, { -1, 1, 0.5f, 0xFF204060, 0, 0 }, { 1, -1, 0.5f, 0xFF204060, 1, 1 }, { 1, 1, 0.5f, 0xFF204060, 1, 0 } };
    IDirect3DVertexShader9 *vs = NULL, *vs2 = NULL;
    IDirect3DPixelShader9 *ps = NULL, *ps1 = NULL;
    static const float one[4] = { 1, 1, 1, 1 }, half[4] = { 0.5f, 0.25f, 0.75f, 1.0f };
    static const uint8_t tx[4] = { 200, 100, 50, 255 };
    IDirect3DTexture9 *t;
    DWORD *code;
    if (!g_asm) { printf("note: no d3dx9_43.dll, shaders not tested\n"); return; }
    code = assemble("vs_2_0\ndcl_position v0\ndcl_color v1\ndcl_texcoord v2\ndef c8, 0.5, 0.5, 0.5, 1\nmov oPos, v0\nmov oD0, v1\nmov oT0, v2\n");
    if (code) HR(IDirect3DDevice9_CreateVertexShader(g_d, code, &vs));
    code = assemble("ps_2_0\ndcl v0\ndcl t0\ndcl_2d s0\ndef c7, 2, 2, 2, 1\ntexld r0, t0, s0\nmul r0, r0, c0\nadd r0, r0, v0\nmov oC0, r0\n");
    if (code) HR(IDirect3DDevice9_CreatePixelShader(g_d, code, &ps));
    code = assemble("vs_1_1\ndcl_position v0\ndcl_color v1\nm4x4 oPos, v0, c0\nmul oD0, v1, c4\n");
    if (code) HR(IDirect3DDevice9_CreateVertexShader(g_d, code, &vs2));
    code = assemble("ps_1_1\nmov r0, v0\n");
    if (code) HR(IDirect3DDevice9_CreatePixelShader(g_d, code, &ps1));
    if (!vs || !ps || !vs2 || !ps1) { ok(0, "shaders created"); return; }
    t = tex1(D3DFMT_A8R8G8B8, tx, 4);                            /* B 200, G 100, R 50 */
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_CULLMODE, D3DCULL_NONE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ZENABLE, FALSE);
    IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZ | D3DFVF_DIFFUSE | D3DFVF_TEX1);
    IDirect3DDevice9_SetVertexShader(g_d, vs);
    IDirect3DDevice9_SetPixelShader(g_d, ps);
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)t);
    IDirect3DDevice9_SetPixelShaderConstantF(g_d, 0, half, 1);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
    /* texture (50, 100, 200) * (0.5, 0.25, 0.75) + vertex colour (0x20, 0x40, 0x60) */
    px(W / 2, H / 2, "vs_2_0 + ps_2_0: texld, constant, colour", 25 + 32, 25 + 64, 150 + 96);
    IDirect3DDevice9_SetTexture(g_d, 0, NULL);
    IDirect3DDevice9_SetVertexShader(g_d, vs2);
    IDirect3DDevice9_SetPixelShader(g_d, ps1);
    {
        static const float idm[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
        IDirect3DDevice9_SetVertexShaderConstantF(g_d, 0, idm, 4);
        IDirect3DDevice9_SetVertexShaderConstantF(g_d, 4, one, 1);
    }
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
    px(W / 2, H / 2, "vs_1_1 + ps_1_1: m4x4, a D3DCOLOR input", 0x20, 0x40, 0x60);
    /* a0 loaded from a constant is one register per draw, from a vertex a run-time index (LDPX): either way vs 1.x floors it and mova rounds */
    {
        static const float at[4] = { 2.7f, 0, 0, 0 }, c12[4] = { 0, 1, 0, 1 }, c13[4] = { 0, 0, 1, 1 }, c7[4] = { 38.0f, 0, 0, 0 }, c14[4] = { 1, 1, 0, 1 }, c15[4] = { 0, 1, 1, 1 };
        IDirect3DVertexShader9 *r1 = NULL, *r2 = NULL, *r3 = NULL, *r4 = NULL;
        IDirect3DDevice9_SetPixelShader(g_d, NULL);
        IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
        IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
        IDirect3DDevice9_SetVertexShaderConstantF(g_d, 8, at, 1);
        IDirect3DDevice9_SetVertexShaderConstantF(g_d, 12, c12, 1);
        IDirect3DDevice9_SetVertexShaderConstantF(g_d, 13, c13, 1);
        code = assemble("vs_1_1\ndcl_position v0\nmov oPos, v0\nmov a0.x, c8.x\nmov oD0, c[a0.x + 10]\n");
        if (code) HR(IDirect3DDevice9_CreateVertexShader(g_d, code, &r1));
        code = assemble("vs_2_0\ndcl_position v0\nmov oPos, v0\nmova a0.x, c8.x\nmov oD0, c[a0.x + 10]\n");
        if (code) HR(IDirect3DDevice9_CreateVertexShader(g_d, code, &r2));
        /* the vertex's red is 0x20: 0.1255 x 38 = 4.77, so 4 floored and 5 rounded */
        IDirect3DDevice9_SetVertexShaderConstantF(g_d, 7, c7, 1);
        IDirect3DDevice9_SetVertexShaderConstantF(g_d, 14, c14, 1);
        IDirect3DDevice9_SetVertexShaderConstantF(g_d, 15, c15, 1);
        code = assemble("vs_1_1\ndcl_position v0\ndcl_color v1\nmov oPos, v0\nmul r0, v1, c7\nmov a0.x, r0.x\nmov oD0, c[a0.x + 10]\n");
        if (code) HR(IDirect3DDevice9_CreateVertexShader(g_d, code, &r3));
        code = assemble("vs_2_0\ndcl_position v0\ndcl_color v1\nmov oPos, v0\nmul r0, v1, c7\nmova a0.x, r0.x\nmov oD0, c[a0.x + 10]\n");
        if (code) HR(IDirect3DDevice9_CreateVertexShader(g_d, code, &r4));
        if (r1 && r2 && r3 && r4) {
            IDirect3DDevice9_SetVertexShader(g_d, r1);
            IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
            HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
            px(W / 2, H / 2, "vs_1_1: a0 from a constant, floored", 0, 255, 0);
            IDirect3DDevice9_SetVertexShader(g_d, r2);
            IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
            HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
            px(W / 2, H / 2, "vs_2_0: mova from a constant, rounded", 0, 0, 255);
            IDirect3DDevice9_SetVertexShader(g_d, r3);
            IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
            HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
            px(W / 2, H / 2, "vs_1_1: a0 from a vertex, floored", 255, 255, 0);
            IDirect3DDevice9_SetVertexShader(g_d, r4);
            IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
            HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
            px(W / 2, H / 2, "vs_2_0: mova from a vertex, rounded", 0, 255, 255);
        } else ok(0, "relative shaders created");
        IDirect3DDevice9_SetVertexShader(g_d, NULL);
        if (r1) IDirect3DVertexShader9_Release(r1);
        if (r2) IDirect3DVertexShader9_Release(r2);
        if (r3) IDirect3DVertexShader9_Release(r3);
        if (r4) IDirect3DVertexShader9_Release(r4);
        IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_MODULATE);
        IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
    }
    IDirect3DDevice9_SetVertexShader(g_d, NULL);
    IDirect3DDevice9_SetPixelShader(g_d, NULL);
    IDirect3DTexture9_Release(t);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_CULLMODE, D3DCULL_CCW);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ZENABLE, TRUE);
}

static void test_present(void)
{
    int i;
    for (i = 0; i < 6; i++) {
        IDirect3DDevice9_BeginScene(g_d);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, D3DCOLOR_XRGB(i * 40, 0, 255), 1.0f, 0);
        IDirect3DDevice9_EndScene(g_d);
        px(20, 20, "a frame after Present", i * 40, 0, 255);
        HR(IDirect3DDevice9_Present(g_d, NULL, NULL, NULL, NULL));
    }
}

/* a buffer that stops changing moves to board A: over several frames, so the move and the drops happen inside the test */
static void tlv_quad(tlv *o, float x0, float x1, DWORD c)
{
    int i;
    for (i = 0; i < 4; i++) { o[i].x = i & 1 ? x1 : x0; o[i].y = i & 2 ? (float)H : 0.0f; o[i].z = 0.5f; o[i].w = 1; o[i].c = c; o[i].u = o[i].v = 0; }
}

static void kept_frame(int a, int b)
{
    IDirect3DDevice9_BeginScene(g_d);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
    if (a >= 0) HR(IDirect3DDevice9_DrawPrimitive(g_d, D3DPT_TRIANGLESTRIP, (UINT)a, 2));
    if (b >= 0) HR(IDirect3DDevice9_DrawIndexedPrimitive(g_d, D3DPT_TRIANGLESTRIP, b, 0, 4, 0, 2));
    IDirect3DDevice9_EndScene(g_d);
}

/* the memory stays the game's: a write no call of ours sees still has to reach the next draw */
static void kept_quad(const char *what, int r, int b)
{
    kept_frame(0, -1);
    px(W / 4, H / 2, what, r, 0, 0);
    px(W * 3 / 4, H / 2, what, 0, 0, b);
    HR(IDirect3DDevice9_Present(g_d, NULL, NULL, NULL, NULL));
}

static void test_kept_unseen(void)
{
    tlv v[4], w[4];
    void *held = NULL, *p = NULL;
    IDirect3DVertexBuffer9 *vb = NULL;
    int f;
    tlv_quad(v, 0, W / 2, 0xFFFF0000);
    tlv_quad(w, W / 2, W, 0xFF0000FF);
    HR(IDirect3DDevice9_CreateVertexBuffer(g_d, sizeof v, D3DUSAGE_WRITEONLY, FVF_TL, D3DPOOL_MANAGED, &vb, NULL));
    HR(IDirect3DVertexBuffer9_Lock(vb, 0, 0, &held, 0));
    memcpy(held, v, sizeof v);
    IDirect3DVertexBuffer9_Unlock(vb);
    IDirect3DDevice9_SetFVF(g_d, FVF_TL);
    HR(IDirect3DDevice9_SetStreamSource(g_d, 0, vb, 0, sizeof(tlv)));
    for (f = 0; f < 6; f++) kept_quad("kept, unseen: before", 255, 0);
    memcpy(held, w, sizeof w);
    kept_quad("kept, unseen: a pointer held past Unlock", 0, 255);
    memcpy(held, v, sizeof v);
    for (f = 0; f < 14; f++) kept_quad("kept, unseen: written back", 255, 0);
    HR(IDirect3DVertexBuffer9_Lock(vb, 0, 0, &p, D3DLOCK_READONLY));
    memcpy(p, w, sizeof w);
    IDirect3DVertexBuffer9_Unlock(vb);
    kept_quad("kept, unseen: under READONLY", 0, 255);
    memcpy(held, v, sizeof v);
    for (f = 0; f < 30; f++) kept_quad("kept, unseen: written back again", 255, 0);
    ((tlv *)held)[3].y = 0;                                     /* the last vertex alone: the second triangle collapses */
    kept_frame(0, -1);
    px(W / 8, H / 8, "kept, unseen: the last vertex alone, the first triangle", 255, 0, 0);
    px(W * 3 / 8, H * 7 / 8, "kept, unseen: the last vertex alone, the second triangle", 0, 0, 0);
    HR(IDirect3DDevice9_Present(g_d, NULL, NULL, NULL, NULL));
    HR(IDirect3DDevice9_SetStreamSource(g_d, 0, NULL, 0, 0));
    IDirect3DVertexBuffer9_Release(vb);
}

static void test_kept(void)
{
    static const WORD idx[4] = { 0, 1, 2, 3 };
    static const D3DVERTEXELEMENT9 two[] = { { 0, 0, D3DDECLTYPE_FLOAT4, 0, D3DDECLUSAGE_POSITIONT, 0 }, { 1, 0, D3DDECLTYPE_D3DCOLOR, 0, D3DDECLUSAGE_COLOR, 0 }, D3DDECL_END() };
    IDirect3DVertexBuffer9 *vb = NULL, *col = NULL;
    IDirect3DIndexBuffer9 *ib = NULL;
    IDirect3DVertexDeclaration9 *de = NULL;
    tlv v[9], w[4];
    void *p = NULL;
    int f, i;
    memset(v, 0x7F, sizeof v[0]);                               /* a stream offset skips this vertex */
    tlv_quad(v + 1, 0, W / 2, 0xFFFF0000);
    tlv_quad(v + 5, W / 2, W, 0xFF00FF00);
    HR(IDirect3DDevice9_CreateVertexBuffer(g_d, sizeof v, D3DUSAGE_WRITEONLY, FVF_TL, D3DPOOL_MANAGED, &vb, NULL));
    HR(IDirect3DDevice9_CreateIndexBuffer(g_d, sizeof idx, D3DUSAGE_WRITEONLY, D3DFMT_INDEX16, D3DPOOL_MANAGED, &ib, NULL));
    HR(IDirect3DVertexBuffer9_Lock(vb, 0, 0, &p, 0));
    memcpy(p, v, sizeof v);
    IDirect3DVertexBuffer9_Unlock(vb);
    HR(IDirect3DIndexBuffer9_Lock(ib, 0, 0, &p, 0));
    memcpy(p, idx, sizeof idx);
    IDirect3DIndexBuffer9_Unlock(ib);
    IDirect3DDevice9_SetVertexShader(g_d, NULL);
    IDirect3DDevice9_SetPixelShader(g_d, NULL);
    IDirect3DDevice9_SetTexture(g_d, 0, NULL);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_ALPHAARG1, D3DTA_DIFFUSE);
    IDirect3DDevice9_SetTextureStageState(g_d, 1, D3DTSS_COLOROP, D3DTOP_DISABLE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ZENABLE, FALSE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ALPHABLENDENABLE, FALSE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ALPHATESTENABLE, FALSE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_CULLMODE, D3DCULL_NONE);
    IDirect3DDevice9_SetFVF(g_d, FVF_TL);
    HR(IDirect3DDevice9_SetStreamSource(g_d, 0, vb, sizeof(tlv), sizeof(tlv)));
    HR(IDirect3DDevice9_SetIndices(g_d, ib));
    for (f = 0; f < 8; f++) {
        kept_frame(0, -1);
        px(W / 4, H / 2, "kept: from vertex 0", 255, 0, 0);
        kept_frame(4, -1);
        px(W * 3 / 4, H / 2, "kept: from vertex 4", 0, 255, 0);
        px(W / 4, H / 2, "kept: from vertex 4, not the first quad", 0, 0, 0);
        kept_frame(-1, 4);
        px(W * 3 / 4, H / 2, "kept: base vertex 4", 0, 255, 0);
        px(W / 4, H / 2, "kept: base vertex 4, not the first quad", 0, 0, 0);
        HR(IDirect3DDevice9_Present(g_d, NULL, NULL, NULL, NULL));
    }
    /* changed between two draws of a frame: the first keeps what it read. A read-only lock changes nothing */
    HR(IDirect3DVertexBuffer9_Lock(vb, 0, 0, &p, D3DLOCK_READONLY));
    IDirect3DVertexBuffer9_Unlock(vb);
    IDirect3DDevice9_BeginScene(g_d);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
    HR(IDirect3DDevice9_DrawPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 2));
    tlv_quad(w, W / 2, W, 0xFF0000FF);
    HR(IDirect3DVertexBuffer9_Lock(vb, sizeof(tlv), sizeof w, &p, 0));
    memcpy(p, w, sizeof w);
    IDirect3DVertexBuffer9_Unlock(vb);
    HR(IDirect3DDevice9_DrawPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 2));
    IDirect3DDevice9_EndScene(g_d);
    px(W / 4, H / 2, "kept: a draw before the Lock", 255, 0, 0);
    px(W * 3 / 4, H / 2, "kept: a draw after the Lock", 0, 0, 255);
    HR(IDirect3DDevice9_Present(g_d, NULL, NULL, NULL, NULL));
    /* a buffer that changes now and then: dropped, kept again, dropped */
    for (f = 0; f < 48; f++) {
        int c = 64 + (f / 7) * 16;
        if (f % 7 == 0) {
            tlv_quad(w, 0, W, D3DCOLOR_XRGB(c, 0, 0));
            HR(IDirect3DVertexBuffer9_Lock(vb, sizeof(tlv), sizeof w, &p, 0));
            memcpy(p, w, sizeof w);
            IDirect3DVertexBuffer9_Unlock(vb);
        }
        kept_frame(0, -1);
        px(W / 2, H / 2, "kept: changed now and then", c, 0, 0);
        HR(IDirect3DDevice9_Present(g_d, NULL, NULL, NULL, NULL));
    }
    /* one stream kept, one refilled every frame; a release while bound by nothing */
    HR(IDirect3DDevice9_CreateVertexBuffer(g_d, 4 * 4, D3DUSAGE_DYNAMIC | D3DUSAGE_WRITEONLY, 0, D3DPOOL_DEFAULT, &col, NULL));
    HR(IDirect3DDevice9_CreateVertexDeclaration(g_d, two, &de));
    IDirect3DDevice9_SetVertexDeclaration(g_d, de);
    HR(IDirect3DDevice9_SetStreamSource(g_d, 0, vb, sizeof(tlv), sizeof(tlv)));
    HR(IDirect3DDevice9_SetStreamSource(g_d, 1, col, 0, 4));
    for (f = 0; f < 10; f++) {
        HR(IDirect3DVertexBuffer9_Lock(col, 0, 0, &p, D3DLOCK_DISCARD));
        for (i = 0; i < 4; i++) ((DWORD *)p)[i] = D3DCOLOR_XRGB(0, 40 + f * 20, 0);
        IDirect3DVertexBuffer9_Unlock(col);
        kept_frame(0, -1);
        px(W / 2, H / 2, "kept: beside a stream refilled every frame", 0, 40 + f * 20, 0);
        HR(IDirect3DDevice9_Present(g_d, NULL, NULL, NULL, NULL));
    }
    IDirect3DDevice9_SetStreamSource(g_d, 0, NULL, 0, 0);
    IDirect3DDevice9_SetStreamSource(g_d, 1, NULL, 0, 0);
    IDirect3DDevice9_SetIndices(g_d, NULL);
    IDirect3DDevice9_SetVertexDeclaration(g_d, NULL);
    IDirect3DVertexDeclaration9_Release(de);
    IDirect3DVertexBuffer9_Release(col);
    IDirect3DVertexBuffer9_Release(vb);
    IDirect3DIndexBuffer9_Release(ib);
}

/* ---------------------------------------------------------------- what games ask for rarely: every scene has a pixel that comes out otherwise if the feature is ignored */
static DWORD fbits(float f) { DWORD v; memcpy(&v, &f, 4); return v; }
static void stage(int s, D3DTEXTURESTAGESTATETYPE ty, DWORD v) { IDirect3DDevice9_SetTextureStageState(g_d, (DWORD)s, ty, v); }
static void rs(D3DRENDERSTATETYPE r, DWORD v) { IDirect3DDevice9_SetRenderState(g_d, r, v); }

static void plain_state(void)
{
    int i;
    IDirect3DDevice9_SetVertexShader(g_d, NULL);
    IDirect3DDevice9_SetPixelShader(g_d, NULL);
    for (i = 0; i < 8; i++) {
        IDirect3DDevice9_SetTexture(g_d, (DWORD)i, NULL);
        stage(i, D3DTSS_COLOROP, i ? D3DTOP_DISABLE : D3DTOP_SELECTARG1);
        stage(i, D3DTSS_ALPHAOP, i ? D3DTOP_DISABLE : D3DTOP_SELECTARG1);
        stage(i, D3DTSS_COLORARG1, i ? D3DTA_TEXTURE : D3DTA_DIFFUSE);
        stage(i, D3DTSS_ALPHAARG1, i ? D3DTA_TEXTURE : D3DTA_DIFFUSE);
        stage(i, D3DTSS_TEXCOORDINDEX, (DWORD)i);
        stage(i, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
        IDirect3DDevice9_SetSamplerState(g_d, (DWORD)i, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP);
        IDirect3DDevice9_SetSamplerState(g_d, (DWORD)i, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);
        IDirect3DDevice9_SetSamplerState(g_d, (DWORD)i, D3DSAMP_MINFILTER, D3DTEXF_POINT);
        IDirect3DDevice9_SetSamplerState(g_d, (DWORD)i, D3DSAMP_MAGFILTER, D3DTEXF_POINT);
    }
    rs(D3DRS_ZENABLE, FALSE); rs(D3DRS_LIGHTING, FALSE); rs(D3DRS_CULLMODE, D3DCULL_NONE); rs(D3DRS_SHADEMODE, D3DSHADE_GOURAUD); rs(D3DRS_FILLMODE, D3DFILL_SOLID);
    rs(D3DRS_ALPHABLENDENABLE, FALSE); rs(D3DRS_ALPHATESTENABLE, FALSE); rs(D3DRS_STENCILENABLE, FALSE); rs(D3DRS_FOGENABLE, FALSE); rs(D3DRS_SPECULARENABLE, FALSE);
    rs(D3DRS_POINTSPRITEENABLE, FALSE); rs(D3DRS_POINTSCALEENABLE, FALSE); rs(D3DRS_VERTEXBLEND, D3DVBF_DISABLE); rs(D3DRS_INDEXEDVERTEXBLENDENABLE, FALSE); rs(D3DRS_CLIPPLANEENABLE, 0);
}

static IDirect3DTexture9 *tex_px(int w, int h, D3DFORMAT f, const void *data, int bytes)
{
    IDirect3DTexture9 *t = NULL;
    D3DLOCKED_RECT l;
    int y;
    HR(IDirect3DDevice9_CreateTexture(g_d, (UINT)w, (UINT)h, 1, 0, f, D3DPOOL_MANAGED, &t, NULL));
    if (!t) return NULL;
    IDirect3DTexture9_LockRect(t, 0, &l, NULL, 0);
    for (y = 0; y < h; y++) memcpy((uint8_t *)l.pBits + y * l.Pitch, (const uint8_t *)data + y * w * bytes, (size_t)(w * bytes));
    IDirect3DTexture9_UnlockRect(t, 0);
    return t;
}

/* (0, 0) red, (1, 0) green, (0, 1) blue, (1, 1) white, D3DFMT_A8R8G8B8 */
static IDirect3DTexture9 *tex_map(void) { static const DWORD c[4] = { 0xFFFF0000, 0xFF00FF00, 0xFF0000FF, 0xFFFFFFFF }; return tex_px(2, 2, D3DFMT_A8R8G8B8, c, 4); }
static IDirect3DTexture9 *tex_one(DWORD c) { return tex_px(1, 1, D3DFMT_A8R8G8B8, &c, 4); }

/* +X red, -X green, +Y blue, -Y yellow, +Z cyan, -Z magenta */
static IDirect3DCubeTexture9 *tex_cube(void)
{
    static const DWORD c[6] = { 0xFFFF0000, 0xFF00FF00, 0xFF0000FF, 0xFFFFFF00, 0xFF00FFFF, 0xFFFF00FF };
    IDirect3DCubeTexture9 *t = NULL;
    D3DLOCKED_RECT l;
    int f;
    HR(IDirect3DDevice9_CreateCubeTexture(g_d, 1, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &t, NULL));
    if (!t) return NULL;
    for (f = 0; f < 6; f++) { IDirect3DCubeTexture9_LockRect(t, (D3DCUBEMAP_FACES)f, 0, &l, NULL, 0); *(DWORD *)l.pBits = c[f]; IDirect3DCubeTexture9_UnlockRect(t, (D3DCUBEMAP_FACES)f, 0); }
    return t;
}

typedef struct { float x, y, z, w, t[4][4]; } v4t;
#define FVF_4T (D3DFVF_XYZRHW | D3DFVF_TEX4 | D3DFVF_TEXCOORDSIZE4(0) | D3DFVF_TEXCOORDSIZE4(1) | D3DFVF_TEXCOORDSIZE4(2) | D3DFVF_TEXCOORDSIZE4(3))
/* the whole target with the same four coordinates everywhere */
static void quad_4t(const float t[4][4])
{
    v4t v[4];
    int i;
    for (i = 0; i < 4; i++) { v[i].x = i & 1 ? (float)W : 0.0f; v[i].y = i & 2 ? (float)H : 0.0f; v[i].z = 0.5f; v[i].w = 1.0f; memcpy(v[i].t, t, sizeof v[i].t); }
    IDirect3DDevice9_SetFVF(g_d, FVF_4T);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, v, sizeof v[0]));
}

static IDirect3DPixelShader9 *ps_of(const char *text)
{
    IDirect3DPixelShader9 *p = NULL;
    DWORD *code = assemble(text);
    if (code) HR(IDirect3DDevice9_CreatePixelShader(g_d, code, &p));
    return p;
}
static IDirect3DVertexShader9 *vs_of(const char *text)
{
    IDirect3DVertexShader9 *p = NULL;
    DWORD *code = assemble(text);
    if (code) HR(IDirect3DDevice9_CreateVertexShader(g_d, code, &p));
    return p;
}

/* a pixel shader over the whole target */
static void ps_scene(const char *what, const char *text, const float t[4][4], int r, int g, int b)
{
    IDirect3DPixelShader9 *p = ps_of(text);
    if (!p) { ok(0, what); return; }
    IDirect3DDevice9_SetPixelShader(g_d, p);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(9, 9, 9), 1.0f, 0);
    quad_4t(t);
    px(W / 2, H / 2, what, r, g, b);
    IDirect3DDevice9_SetPixelShader(g_d, NULL);
    IDirect3DPixelShader9_Release(p);
}

static void test_ps1(void)
{
    static const signed char bump[2] = { 64, 0 };              /* du about +0.5 */
    static const uint8_t bumpl[4] = { 64, 0, 128, 0 };         /* X8L8V8U8: U, V, L */
    IDirect3DTexture9 *map = tex_map(), *b = tex_px(1, 1, D3DFMT_V8U8, bump, 2), *bl = tex_px(1, 1, D3DFMT_X8L8V8U8, bumpl, 4), *n, *x;
    IDirect3DCubeTexture9 *cube = tex_cube();
    float t[4][4];
    static const float c0[4] = { 0.5f, 0, 1, 0 };
    if (!g_asm || !map || !b || !bl || !cube) { ok(0, "test_ps1 textures"); return; }
    plain_state();
    memset(t, 0, sizeof t);
    /* texbem: (0.25, 0.25) moved right by the bump texel through stage 1's matrix */
    t[1][0] = t[1][1] = 0.25f;
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)b);
    IDirect3DDevice9_SetTexture(g_d, 1, (IDirect3DBaseTexture9 *)map);
    stage(1, D3DTSS_BUMPENVMAT00, fbits(1.0f)); stage(1, D3DTSS_BUMPENVMAT01, 0); stage(1, D3DTSS_BUMPENVMAT10, 0); stage(1, D3DTSS_BUMPENVMAT11, fbits(1.0f));
    ps_scene("texbem: moved", "ps_1_1\ntex t0\ntexbem t1, t0\nmov r0, t1\n", t, 0, 255, 0);
    stage(1, D3DTSS_BUMPENVMAT00, 0); stage(1, D3DTSS_BUMPENVMAT11, 0);
    ps_scene("texbem: a zero matrix leaves it", "ps_1_1\ntex t0\ntexbem t1, t0\nmov r0, t1\n", t, 255, 0, 0);
    stage(1, D3DTSS_BUMPENVMAT01, fbits(1.0f));
    ps_scene("texbem: BUMPENVMAT01 moves v", "ps_1_1\ntex t0\ntexbem t1, t0\nmov r0, t1\n", t, 0, 0, 255);
    stage(1, D3DTSS_BUMPENVMAT01, 0); stage(1, D3DTSS_BUMPENVMAT00, fbits(1.0f)); stage(1, D3DTSS_BUMPENVMAT11, fbits(1.0f));
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)bl);
    stage(1, D3DTSS_BUMPENVLSCALE, fbits(1.0f)); stage(1, D3DTSS_BUMPENVLOFFSET, 0);
    ps_scene("texbeml: luminance half", "ps_1_1\ntex t0\ntexbeml t1, t0\nmov r0, t1\n", t, 0, 128, 0);
    stage(1, D3DTSS_BUMPENVLOFFSET, fbits(0.25f));
    ps_scene("texbeml: luminance offset", "ps_1_1\ntex t0\ntexbeml t1, t0\nmov r0, t1\n", t, 0, 192, 0);
    /* a signed texel read as it is: r and g below zero stay below zero */
    {
        static const signed char neg[2] = { -128 + 1, 127 };
        IDirect3DTexture9 *s = tex_px(1, 1, D3DFMT_V8U8, neg, 2);
        static const float half[4] = { 0.5f, 0.5f, 0.5f, 0.5f };
        IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)s);
        IDirect3DDevice9_SetPixelShaderConstantF(g_d, 0, half, 1);
        ps_scene("V8U8: signed, and b reads 1", "ps_2_0\ndcl t0\ndcl_2d s0\ntexld r0, t0, s0\nmad r0, r0, c0, c0\nmov oC0, r0\n", t, 0, 255, 255);
        IDirect3DTexture9_Release(s);
    }
    /* texreg2ar, texreg2gb, texreg2rgb: a texel's channels are the next stage's coordinates */
    x = tex_one(D3DCOLOR_ARGB(191, 64, 0, 0));
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)x);
    ps_scene("texreg2ar", "ps_1_1\ntex t0\ntexreg2ar t1, t0\nmov r0, t1\n", t, 0, 255, 0);
    IDirect3DTexture9_Release(x);
    x = tex_one(D3DCOLOR_ARGB(0, 0, 191, 191));
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)x);
    ps_scene("texreg2gb", "ps_1_1\ntex t0\ntexreg2gb t1, t0\nmov r0, t1\n", t, 255, 255, 255);
    IDirect3DTexture9_Release(x);
    x = tex_one(D3DCOLOR_ARGB(0, 64, 191, 0));
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)x);
    ps_scene("texreg2rgb", "ps_1_2\ntex t0\ntexreg2rgb t1, t0\nmov r0, t1\n", t, 0, 0, 255);
    IDirect3DTexture9_Release(x);
    /* texm3x2: two rows of a matrix on a normal from stage 0 */
    n = tex_one(D3DCOLOR_ARGB(255, 191, 64, 128));              /* _bx2: about (0.5, -0.5, 0) */
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)n);
    IDirect3DDevice9_SetTexture(g_d, 1, NULL);
    IDirect3DDevice9_SetTexture(g_d, 2, (IDirect3DBaseTexture9 *)map);
    memset(t, 0, sizeof t);
    t[1][0] = 1.5f; t[2][1] = -0.5f;
    ps_scene("texm3x2tex", "ps_1_1\ntex t0\ntexm3x2pad t1, t0_bx2\ntexm3x2tex t2, t0_bx2\nmov r0, t2\n", t, 0, 255, 0);
    IDirect3DTexture9_Release(n);
    /* texm3x3: three rows, a cube map on stage 3 chosen by the texture, since ps 1.x declares nothing */
    n = tex_one(D3DCOLOR_ARGB(255, 255, 128, 128));             /* +X */
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)n);
    IDirect3DDevice9_SetTexture(g_d, 2, NULL);
    IDirect3DDevice9_SetTexture(g_d, 3, (IDirect3DBaseTexture9 *)cube);
    memset(t, 0, sizeof t);
    t[1][0] = t[2][1] = t[3][2] = 1.0f;
    ps_scene("texm3x3tex", "ps_1_1\ntex t0\ntexm3x3pad t1, t0_bx2\ntexm3x3pad t2, t0_bx2\ntexm3x3tex t3, t0_bx2\nmov r0, t3\n", t, 255, 0, 0);
    ps_scene("texm3x3: the normal itself", "ps_1_2\ntex t0\ntexm3x3pad t1, t0_bx2\ntexm3x3pad t2, t0_bx2\ntexm3x3 t3, t0_bx2\nmov r0, t3\n", t, 255, 1, 1);
    IDirect3DTexture9_Release(n);
    n = tex_one(D3DCOLOR_ARGB(255, 128, 255, 128));             /* +Y: a cube's own face, which no 2D read of its first image would give */
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)n);
    ps_scene("texm3x3tex: another face", "ps_1_1\ntex t0\ntexm3x3pad t1, t0_bx2\ntexm3x3pad t2, t0_bx2\ntexm3x3tex t3, t0_bx2\nmov r0, t3\n", t, 0, 0, 255);
    IDirect3DTexture9_Release(n);
    n = tex_one(D3DCOLOR_ARGB(255, 128, 128, 255));             /* +Z. The eye (0.5, 0, 1) reflects to (-0.5, 0, 1), +Z; half the reflection would give -X */
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)n);
    IDirect3DDevice9_SetPixelShaderConstantF(g_d, 0, c0, 1);
    ps_scene("texm3x3spec: the eye in a constant", "ps_1_1\ntex t0\ntexm3x3pad t1, t0_bx2\ntexm3x3pad t2, t0_bx2\ntexm3x3spec t3, t0_bx2, c0\nmov r0, t3\n", t, 0, 255, 255);
    t[1][3] = 0.5f; t[3][3] = 1.0f;
    ps_scene("texm3x3vspec: the eye in the coordinates", "ps_1_1\ntex t0\ntexm3x3pad t1, t0_bx2\ntexm3x3pad t2, t0_bx2\ntexm3x3vspec t3, t0_bx2\nmov r0, t3\n", t, 0, 255, 255);
    IDirect3DTexture9_Release(n);
    IDirect3DDevice9_SetTexture(g_d, 3, NULL);
    /* texdp3, texdp3tex */
    n = tex_one(D3DCOLOR_ARGB(255, 255, 128, 128));
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)n);
    IDirect3DDevice9_SetTexture(g_d, 1, (IDirect3DBaseTexture9 *)map);
    memset(t, 0, sizeof t);
    t[1][0] = 0.5f;
    ps_scene("texdp3", "ps_1_2\ntex t0\ntexdp3 t1, t0_bx2\nmov r0, t1\n", t, 128, 128, 128);
    t[1][0] = 0.75f;
    ps_scene("texdp3tex", "ps_1_2\ntex t0\ntexdp3tex t1, t0_bx2\nmov r0, t1\n", t, 0, 255, 0);
    IDirect3DTexture9_Release(n);
    /* ps_1_4: bem, phase, dependent read */
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)map);
    IDirect3DDevice9_SetTexture(g_d, 1, (IDirect3DBaseTexture9 *)b);
    stage(0, D3DTSS_BUMPENVMAT00, fbits(1.0f)); stage(0, D3DTSS_BUMPENVMAT01, 0); stage(0, D3DTSS_BUMPENVMAT10, 0); stage(0, D3DTSS_BUMPENVMAT11, fbits(1.0f));
    memset(t, 0, sizeof t);
    t[0][0] = t[0][1] = 0.25f;
    ps_scene("bem", "ps_1_4\ntexld r1, t1\ntexcrd r0.rgb, t0\nbem r0.rg, r0, r1\nphase\ntexld r0, r0\n", t, 0, 255, 0);
    /* projected through the stage's flags: only ps 1.1 to 1.3 do that */
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)map);
    IDirect3DDevice9_SetTexture(g_d, 1, NULL);
    memset(t, 0, sizeof t);
    t[0][0] = 1.5f; t[0][1] = 0.5f; t[0][3] = 2.0f;
    stage(0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_PROJECTED);
    ps_scene("ps 1.1: tex projected by the stage's flag", "ps_1_1\ntex t0\nmov r0, t0\n", t, 0, 255, 0);
    stage(0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
    ps_scene("ps 1.1: not projected without it", "ps_1_1\ntex t0\nmov r0, t0\n", t, 255, 255, 255);
    /* depth from a shader */
    {
        IDirect3DPixelShader9 *p = ps_of("ps_1_4\ndef c0, 0, 1, 0, 1\ntexcrd r5.rgb, t0\nphase\ntexdepth r5\nmov r0, c0\n");
        static const float one[4] = { 0, 0, 0, 0 };
        (void)one;
        memset(t, 0, sizeof t);
        t[0][0] = 0.5f; t[0][1] = 1.0f;
        rs(D3DRS_ZENABLE, TRUE); rs(D3DRS_ZFUNC, D3DCMP_ALWAYS); rs(D3DRS_ZWRITEENABLE, TRUE);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
        IDirect3DDevice9_SetPixelShader(g_d, p);
        if (p) quad_4t(t);
        IDirect3DDevice9_SetPixelShader(g_d, NULL);
        rs(D3DRS_ZFUNC, D3DCMP_LESS);
        IDirect3DDevice9_SetTexture(g_d, 0, NULL);
        quad(0, 0, W, H / 2, 0.75f, D3DCOLOR_XRGB(255, 0, 0));
        quad(0, H / 2, W, H, 0.25f, D3DCOLOR_XRGB(0, 0, 255));
        px(W / 2, H / 4, "texdepth: nearer than 0.75", 0, 255, 0);
        px(W / 2, H * 3 / 4, "texdepth: further than 0.25", 0, 0, 255);
        rs(D3DRS_ZFUNC, D3DCMP_LESSEQUAL);
        if (p) IDirect3DPixelShader9_Release(p);
    }
    IDirect3DTexture9_Release(map);
    IDirect3DTexture9_Release(b);
    IDirect3DTexture9_Release(bl);
    IDirect3DCubeTexture9_Release(cube);
    plain_state();
}

/* a vertex shader's colour over the whole target */
static void vs_scene(const char *what, IDirect3DVertexShader9 *s, int r, int g, int b)
{
    static const float q[4][3] = { { -1, -1, 0.5f }, { -1, 1, 0.5f }, { 1, -1, 0.5f }, { 1, 1, 0.5f } };
    IDirect3DDevice9_SetVertexShader(g_d, s);
    IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZ);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(9, 9, 9), 1.0f, 0);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
    px(W / 2, H / 2, what, r, g, b);
    IDirect3DDevice9_SetVertexShader(g_d, NULL);
}

static void test_flow(void)
{
    static const float c[4] = { 0.25f, 0.5f, 0.75f, 1 }, one[4] = { 1, 1, 1, 1 }, c11[4] = { 0.125f, 0, 0, 0 }, c12[4] = { 0, 0.25f, 0, 0 }, c13[4] = { 0, 0, 0.5f, 0 };
    static const BOOL off = FALSE, on = TRUE;
    static const int two[4] = { 2, 12, 1, 0 }, one_i[4] = { 1, 11, 1, 0 };
    IDirect3DVertexShader9 *s;
    IDirect3DPixelShader9 *p;
    IDirect3DTexture9 *t;
    float tc[4][4];
    if (!g_asm) return;
    plain_state();
    IDirect3DDevice9_SetVertexShaderConstantF(g_d, 20, c, 1);
    IDirect3DDevice9_SetVertexShaderConstantF(g_d, 21, one, 1);
    IDirect3DDevice9_SetVertexShaderConstantF(g_d, 11, c11, 1);
    IDirect3DDevice9_SetVertexShaderConstantF(g_d, 12, c12, 1);
    IDirect3DDevice9_SetVertexShaderConstantF(g_d, 13, c13, 1);
    IDirect3DDevice9_SetVertexShaderConstantB(g_d, 0, &off, 1);
    /* subroutines written out at their calls; callnz on a constant bool and on one defined in the shader */
    s = vs_of("vs_2_0\ndcl_position v0\ndefb b1, true\nmov oPos, v0\nmov r0, c20\ncall l0\ncallnz l1, b1\ncallnz l2, b0\nmov oD0, r0\nret\n"
              "label l0\nadd r0.x, r0.x, r0.x\nret\nlabel l1\nmov r0.y, c20.x\nret\nlabel l2\nmov r0.z, c20.x\nret\n");
    if (s) {
        vs_scene("call, callnz, ret: b0 off", s, 128, 64, 191);
        IDirect3DDevice9_SetVertexShaderConstantB(g_d, 0, &on, 1);
        vs_scene("callnz: b0 on", s, 128, 64, 64);
        IDirect3DDevice9_SetVertexShaderConstantB(g_d, 0, &off, 1);
        IDirect3DVertexShader9_Release(s);
    } else ok(0, "subroutine shader");
    /* a subroutine called twice runs twice, and one calling another */
    s = vs_of("vs_2_0\ndcl_position v0\nmov oPos, v0\nmov r0, c20\ncall l0\ncall l0\nmov oD0, r0\nret\nlabel l0\ncall l1\nret\nlabel l1\nadd r0.x, r0.x, c20.x\nret\n");
    if (s) { vs_scene("a call in a call, twice", s, 191, 128, 191); IDirect3DVertexShader9_Release(s); }
    else ok(0, "nested call shader");
    /* loop from defi: aL addresses the constants */
    s = vs_of("vs_2_0\ndcl_position v0\ndefi i0, 3, 11, 1, 0\ndef c30, 0, 0, 0, 1\nmov oPos, v0\nmov r0, c30\nloop aL, i0\nadd r0, r0, c[aL]\nendloop\nmov oD0, r0\n");
    if (s) { vs_scene("loop aL, c[aL]", s, 32, 64, 128); IDirect3DVertexShader9_Release(s); }
    else ok(0, "loop shader");
    /* the count from SetVertexShaderConstantI: the program follows the value */
    s = vs_of("vs_2_0\ndcl_position v0\ndef c30, 0, 0, 0, 1\nmov oPos, v0\nmov r0, c30\nloop aL, i1\nadd r0, r0, c[aL]\nendloop\nmov oD0, r0\n");
    if (s) {
        IDirect3DDevice9_SetVertexShaderConstantI(g_d, 1, two, 1);
        vs_scene("loop: count and start from i1", s, 0, 64, 128);
        IDirect3DDevice9_SetVertexShaderConstantI(g_d, 1, one_i, 1);
        vs_scene("loop: another i1", s, 32, 0, 0);
        IDirect3DVertexShader9_Release(s);
    } else ok(0, "loop i1 shader");
    s = vs_of("vs_2_0\ndcl_position v0\ndef c30, 0, 0, 0, 1\ndef c31, 0.125, 0, 0, 0\nmov oPos, v0\nmov r0, c30\nrep i1\nadd r0, r0, c31\nendrep\nmov oD0, r0\n");
    if (s) {
        IDirect3DDevice9_SetVertexShaderConstantI(g_d, 1, two, 1);
        vs_scene("rep: twice", s, 64, 0, 0);
        IDirect3DVertexShader9_Release(s);
    } else ok(0, "rep shader");
    /* predicates */
    s = vs_of("vs_2_x\ndcl_position v0\ndef c30, 1, 2, 3, 4\ndef c31, 2, 2, 2, 2\ndef c32, 1, 1, 1, 1\ndef c33, 0, 0, 0, 0\nmov oPos, v0\nsetp_lt p0, c30, c31\nmov r0, c33\n(p0) mov r0, c32\nmov oD0, r0\n");
    if (s) { vs_scene("setp_lt, (p0)", s, 255, 0, 0); IDirect3DVertexShader9_Release(s); }
    else ok(0, "predicate shader");
    s = vs_of("vs_2_x\ndcl_position v0\ndef c30, 1, 2, 3, 4\ndef c31, 2, 2, 2, 2\ndef c32, 1, 1, 1, 1\ndef c33, 0, 0, 0, 0\nmov oPos, v0\nsetp_ge p0, c30, c31\nmov r0, c33\n(!p0) mov r0, c32\nmov oD0, r0\n");
    if (s) { vs_scene("setp_ge, (!p0)", s, 255, 0, 0); IDirect3DVertexShader9_Release(s); }
    else ok(0, "not predicate shader");
    s = vs_of("vs_2_x\ndcl_position v0\ndef c30, 1, 2, 3, 4\ndef c31, 2, 2, 2, 2\ndef c32, 1, 1, 1, 1\ndef c33, 0, 0, 0, 0\nmov oPos, v0\nsetp_eq p0, c30, c31\nmov r0, c33\nif p0.y\nmov r0.g, c32.x\nelse\nmov r0.b, c32.x\nendif\nmov oD0, r0\n");
    if (s) { vs_scene("setp_eq, if p0.y", s, 0, 255, 0); IDirect3DVertexShader9_Release(s); }
    else ok(0, "if p0 shader");
    s = vs_of("vs_2_x\ndcl_position v0\ndefi i0, 8, 0, 1, 0\ndef c30, 0, 0, 0, 1\ndef c31, 0.125, 0, 0, 0\ndef c32, 0.3, 0, 0, 0\nmov oPos, v0\nmov r0, c30\nrep i0\nsetp_gt p0.x, r0.x, c32.x\nbreakp p0.x\nadd r0, r0, c31\nendrep\nmov oD0, r0\n");
    if (s) { vs_scene("breakp ends the rep", s, 96, 0, 0); IDirect3DVertexShader9_Release(s); }
    else ok(0, "breakp shader");
    /* ps: predicates, texldd, texldl */
    memset(tc, 0, sizeof tc);
    tc[0][0] = tc[0][1] = 0.25f;
    ps_scene("ps_2_x: setp, (p0)", "ps_2_x\ndef c1, 1, 0, 0, 0\ndef c2, 0, 1, 0, 1\ndef c3, 0, 0, 1, 1\nsetp_gt p0, c1, c2\nmov r0, c3\n(p0.x) mov r0.r, c2.g\nmov oC0, r0\n", tc, 255, 0, 255);
    {   /* four levels, each its own colour */
        static const DWORD lv[4] = { 0xFFFF0000, 0xFF00FF00, 0xFF0000FF, 0xFFFFFFFF };
        D3DLOCKED_RECT l;
        UINT i, x, y;
        t = NULL;
        HR(IDirect3DDevice9_CreateTexture(g_d, 8, 8, 4, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &t, NULL));
        for (i = 0; t && i < 4; i++) {
            IDirect3DTexture9_LockRect(t, i, &l, NULL, 0);
            for (y = 0; y < 8u >> i; y++) for (x = 0; x < 8u >> i; x++) *(DWORD *)((uint8_t *)l.pBits + y * (UINT)l.Pitch + x * 4) = lv[i];
            IDirect3DTexture9_UnlockRect(t, i);
        }
        IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)t);
        IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_MIPFILTER, D3DTEXF_POINT);
        ps_scene("texldd: gradients of two texels, level 1", "ps_2_x\ndcl t0\ndcl_2d s0\ndef c1, 0.25, 0, 0, 0\ndef c2, 0, 0.25, 0, 0\ntexldd r0, t0, s0, c1, c2\nmov oC0, r0\n", tc, 0, 255, 0);
        ps_scene("texldd: gradients of eight texels, level 3", "ps_2_x\ndcl t0\ndcl_2d s0\ndef c1, 1, 0, 0, 0\ndef c2, 0, 0, 0, 0\ntexldd r0, t0, s0, c1, c2\nmov oC0, r0\n", tc, 255, 255, 255);
        /* texldl needs ps_3_0 and so a vs_3_0 */
        s = vs_of("vs_3_0\ndcl_position v0\ndcl_position o0\nmov o0, v0\n");
        if (s) {
            IDirect3DPixelShader9 *a = ps_of("ps_3_0\ndcl_2d s0\ndef c0, 0.25, 0.25, 0, 2\ntexldl r0, c0, s0\nmov oC0, r0\n");
            static const float q[4][3] = { { -1, -1, 0.5f }, { -1, 1, 0.5f }, { 1, -1, 0.5f }, { 1, 1, 0.5f } };
            IDirect3DDevice9_SetVertexShader(g_d, s);
            IDirect3DDevice9_SetPixelShader(g_d, a);
            IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZ);
            IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
            HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
            px(W / 2, H / 2, "texldl: level 2", 0, 0, 255);
            IDirect3DDevice9_SetPixelShader(g_d, NULL);
            IDirect3DDevice9_SetVertexShader(g_d, NULL);
            if (a) IDirect3DPixelShader9_Release(a);
            IDirect3DVertexShader9_Release(s);
        } else ok(0, "vs_3_0 shader");
        IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
        IDirect3DDevice9_SetTexture(g_d, 0, NULL);
        if (t) IDirect3DTexture9_Release(t);
    }
    (void)p;
    plain_state();
}

static void test_ff_extra(void)
{
    typedef struct { float x, y, z, w; DWORD c; float u, v; } v1;
    static const D3DMATRIX id = { { { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } } };
    IDirect3DTexture9 *map = tex_map();
    D3DMATRIX m = id;
    plain_state();
    IDirect3DDevice9_SetTransform(g_d, D3DTS_WORLD, &id);
    IDirect3DDevice9_SetTransform(g_d, D3DTS_VIEW, &id);
    IDirect3DDevice9_SetTransform(g_d, D3DTS_PROJECTION, &id);
    /* D3DTA_CONSTANT: each stage has its own */
    stage(0, D3DTSS_COLORARG1, D3DTA_CONSTANT);
    stage(0, D3DTSS_CONSTANT, D3DCOLOR_XRGB(10, 200, 30));
    stage(1, D3DTSS_COLOROP, D3DTOP_ADD); stage(1, D3DTSS_COLORARG1, D3DTA_CURRENT); stage(1, D3DTSS_COLORARG2, D3DTA_CONSTANT);
    stage(1, D3DTSS_CONSTANT, D3DCOLOR_XRGB(40, 0, 60));
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    quad(0, 0, W, H, 0.5f, 0xFFFFFFFF);
    px(W / 2, H / 2, "D3DTA_CONSTANT, per stage", 50, 200, 90);
    stage(1, D3DTSS_COLOROP, D3DTOP_DISABLE);
    stage(0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    /* BUMPENVMAP: the stage moves the next stage's coordinates */
    {
        static const signed char bump[2] = { 64, 0 };
        static const uint8_t bumpl[4] = { 64, 0, 128, 0 };
        IDirect3DTexture9 *b = tex_px(1, 1, D3DFMT_V8U8, bump, 2), *bl = tex_px(1, 1, D3DFMT_X8L8V8U8, bumpl, 4);
        typedef struct { float x, y, z, w, u0, v0, u1, v1; } v2;
        v2 v[4];
        int i;
        for (i = 0; i < 4; i++) { v[i].x = i & 1 ? (float)W : 0.0f; v[i].y = i & 2 ? (float)H : 0.0f; v[i].z = 0.5f; v[i].w = 1; v[i].u0 = v[i].v0 = 0; v[i].u1 = v[i].v1 = 0.25f; }
        IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)b);
        IDirect3DDevice9_SetTexture(g_d, 1, (IDirect3DBaseTexture9 *)map);
        stage(0, D3DTSS_COLOROP, D3DTOP_BUMPENVMAP); stage(0, D3DTSS_COLORARG1, D3DTA_TEXTURE); stage(0, D3DTSS_COLORARG2, D3DTA_CURRENT);
        stage(0, D3DTSS_BUMPENVMAT00, fbits(1.0f)); stage(0, D3DTSS_BUMPENVMAT01, 0); stage(0, D3DTSS_BUMPENVMAT10, 0); stage(0, D3DTSS_BUMPENVMAT11, fbits(1.0f));
        stage(1, D3DTSS_COLOROP, D3DTOP_SELECTARG1); stage(1, D3DTSS_COLORARG1, D3DTA_TEXTURE); stage(1, D3DTSS_TEXCOORDINDEX, 1);
        IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZRHW | D3DFVF_TEX2);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, v, sizeof v[0]));
        px(W / 2, H / 2, "D3DTOP_BUMPENVMAP", 0, 255, 0);
        IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)bl);
        stage(0, D3DTSS_COLOROP, D3DTOP_BUMPENVMAPLUMINANCE);
        stage(0, D3DTSS_BUMPENVLSCALE, fbits(1.0f)); stage(0, D3DTSS_BUMPENVLOFFSET, 0);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, v, sizeof v[0]));
        px(W / 2, H / 2, "D3DTOP_BUMPENVMAPLUMINANCE", 0, 128, 0);
        IDirect3DTexture9_Release(b);
        IDirect3DTexture9_Release(bl);
        plain_state();
    }
    /* a texture matrix on two coordinates: its third row is the translation */
    {
        v1 v[4] = { { 0, 0, 0.5f, 1, 0, 0.25f, 0.25f }, { W, 0, 0.5f, 1, 0, 0.25f, 0.25f }, { 0, H, 0.5f, 1, 0, 0.25f, 0.25f }, { W, H, 0.5f, 1, 0, 0.25f, 0.25f } };
        typedef struct { float x, y, z, u, v; } v3;
        v3 q[4] = { { -1, -1, 0.5f, 0.25f, 0.25f }, { -1, 1, 0.5f, 0.25f, 0.25f }, { 1, -1, 0.5f, 0.25f, 0.25f }, { 1, 1, 0.5f, 0.25f, 0.25f } };
        IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)map);
        stage(0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
        m.m[2][0] = 0.5f;
        IDirect3DDevice9_SetTransform(g_d, D3DTS_TEXTURE0, &m);
        stage(0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_COUNT2);
        IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZ | D3DFVF_TEX1);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
        px(W / 2, H / 2, "texture matrix: _31 moves two coordinates", 0, 255, 0);
        /* a pretransformed vertex has no texture matrix */
        IDirect3DDevice9_SetFVF(g_d, FVF_TL);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, v, sizeof v[0]));
        px(W / 2, H / 2, "texture matrix: not for pretransformed vertices", 255, 0, 0);
        m = id;
        IDirect3DDevice9_SetTransform(g_d, D3DTS_TEXTURE0, &m);
        stage(0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
    }
    /* projected, three coordinates: the third divides */
    {
        typedef struct { float x, y, z, w, u, v, q; } v3;
        v3 v[4] = { { 0, 0, 0.5f, 1, 1.5f, 0.5f, 2 }, { W, 0, 0.5f, 1, 1.5f, 0.5f, 2 }, { 0, H, 0.5f, 1, 1.5f, 0.5f, 2 }, { W, H, 0.5f, 1, 1.5f, 0.5f, 2 } };
        stage(0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_COUNT3 | D3DTTFF_PROJECTED);
        IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZRHW | D3DFVF_TEX1 | D3DFVF_TEXCOORDSIZE3(0));
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, v, sizeof v[0]));
        px(W / 2, H / 2, "projected: COUNT3 divides by the third", 0, 255, 0);
        stage(0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
    }
    /* sphere map: a normal up and to the right reflects to the top right of the map */
    {
        typedef struct { float x, y, z, nx, ny, nz; } v3;
        v3 q[4] = { { -0.02f, -0.02f, 0.5f, 0.5f, 0.5f, -0.7071f }, { -0.02f, 0.02f, 0.5f, 0.5f, 0.5f, -0.7071f }, { 0.02f, -0.02f, 0.5f, 0.5f, 0.5f, -0.7071f }, { 0.02f, 0.02f, 0.5f, 0.5f, 0.5f, -0.7071f } };
        stage(0, D3DTSS_TEXCOORDINDEX, D3DTSS_TCI_SPHEREMAP);
        IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZ | D3DFVF_NORMAL);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
        px(W / 2, H / 2, "D3DTSS_TCI_SPHEREMAP", 0, 255, 0);
        stage(0, D3DTSS_TEXCOORDINDEX, 0);
    }
    /* point sprites: the texture over the point, row 0 at the top; scaled by the distance */
    {
        typedef struct { float x, y, z, w; DWORD c; } pt;
        typedef struct { float x, y, z; DWORD c; } p3;
        pt a = { 100, 100, 0.5f, 1, 0xFFFFFFFF };
        p3 b = { 0, 0, 0.5f, 0xFFFFFFFF };
        rs(D3DRS_POINTSPRITEENABLE, TRUE);
        rs(D3DRS_POINTSIZE, fbits(32.0f));
        IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZRHW | D3DFVF_DIFFUSE);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_POINTLIST, 1, &a, sizeof a));
        px(92, 92, "point sprite: top left", 255, 0, 0);
        px(108, 92, "point sprite: top right", 0, 255, 0);
        px(92, 108, "point sprite: bottom left", 0, 0, 255);
        px(108, 108, "point sprite: bottom right", 255, 255, 255);
        px(100, 125, "point sprite: outside", 0, 0, 0);
        rs(D3DRS_POINTSIZE_MAX, fbits(16.0f));
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_POINTLIST, 1, &a, sizeof a));
        px(100, 112, "D3DRS_POINTSIZE_MAX", 0, 0, 0);
        rs(D3DRS_POINTSIZE_MAX, fbits(64.0f));
        rs(D3DRS_POINTSCALEENABLE, TRUE);
        rs(D3DRS_POINTSIZE, fbits(0.1f));                       /* 48 pixels at a viewport 480 high, with A = 1 */
        rs(D3DRS_POINTSCALE_A, fbits(1.0f)); rs(D3DRS_POINTSCALE_B, 0); rs(D3DRS_POINTSCALE_C, 0);
        IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZ | D3DFVF_DIFFUSE);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_POINTLIST, 1, &b, sizeof b));
        px(W / 2 - 20, H / 2 - 20, "D3DRS_POINTSCALEENABLE: 48 pixels", 255, 0, 0);
        rs(D3DRS_POINTSCALE_A, 0); rs(D3DRS_POINTSCALE_C, fbits(16.0f));      /* eye distance 0.5: size x 0.5 */
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_POINTLIST, 1, &b, sizeof b));
        px(W / 2 - 20, H / 2 - 20, "POINTSCALE_C: 24 pixels", 0, 0, 0);
        px(W / 2 - 8, H / 2 - 8, "POINTSCALE_C: inside", 255, 0, 0);
        rs(D3DRS_POINTSCALE_A, fbits(1.0f)); rs(D3DRS_POINTSCALE_C, 0);
        plain_state();
    }
    /* vertex blending: world 0 at the left, world 1 at the right */
    {
        typedef struct { float x, y, z, b; DWORD c; } v3;
        typedef struct { float x, y, z, b; uint8_t ix[4]; DWORD c; } v4;
        v3 q[4];
        v4 r[4];
        D3DMATRIX l = id, n = id, a = id, b = id;
        int i;
        l.m[3][0] = -0.5f; n.m[3][0] = 0.5f; a.m[3][1] = 0.5f; b.m[3][1] = -0.5f;
        for (i = 0; i < 4; i++) {
            q[i].x = r[i].x = i & 1 ? 0.25f : -0.25f; q[i].y = r[i].y = i & 2 ? 0.25f : -0.25f; q[i].z = r[i].z = 0.5f; q[i].b = 1.0f; q[i].c = 0xFF00FF00;
            r[i].b = 0.0f; r[i].ix[0] = 3; r[i].ix[1] = 7; r[i].ix[2] = r[i].ix[3] = 0; r[i].c = 0xFFFF0000;
        }
        IDirect3DDevice9_SetTransform(g_d, D3DTS_WORLD, &l);
        IDirect3DDevice9_SetTransform(g_d, D3DTS_WORLDMATRIX(1), &n);
        IDirect3DDevice9_SetTransform(g_d, D3DTS_WORLDMATRIX(3), &a);
        IDirect3DDevice9_SetTransform(g_d, D3DTS_WORLDMATRIX(7), &b);
        rs(D3DRS_VERTEXBLEND, D3DVBF_1WEIGHTS);
        IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZB1 | D3DFVF_DIFFUSE);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
        px(W / 4, H / 2, "vertex blend: all of world 0", 0, 255, 0);
        px(W * 3 / 4, H / 2, "vertex blend: none of world 1", 0, 0, 0);
        for (i = 0; i < 4; i++) q[i].b = 0.0f;
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
        px(W * 3 / 4, H / 2, "vertex blend: the last weight is what is left", 0, 255, 0);
        for (i = 0; i < 4; i++) q[i].b = 0.5f;
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
        px(W / 2, H / 2, "vertex blend: half of each", 0, 255, 0);
        rs(D3DRS_INDEXEDVERTEXBLENDENABLE, TRUE);
        IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZB2 | D3DFVF_LASTBETA_UBYTE4 | D3DFVF_DIFFUSE);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, r, sizeof r[0]));
        px(W / 2, H * 3 / 4, "indexed vertex blend: matrix 7", 255, 0, 0);
        px(W / 2, H / 4, "indexed vertex blend: not matrix 3", 0, 0, 0);
        IDirect3DDevice9_SetTransform(g_d, D3DTS_WORLD, &id);
        plain_state();
    }
    /* tweening: two positions, a factor */
    {
        static const D3DVERTEXELEMENT9 de[] = { { 0, 0, D3DDECLTYPE_FLOAT3, 0, D3DDECLUSAGE_POSITION, 0 }, { 1, 0, D3DDECLTYPE_FLOAT3, 0, D3DDECLUSAGE_POSITION, 1 }, D3DDECL_END() };
        float a[4][3], b[4][3];
        IDirect3DVertexDeclaration9 *decl = NULL;
        IDirect3DVertexBuffer9 *va = NULL, *vb = NULL;
        void *p;
        int i;
        for (i = 0; i < 4; i++) { a[i][0] = (i & 1 ? 0.2f : -0.2f) - 0.5f; b[i][0] = a[i][0] + 1.0f; a[i][1] = b[i][1] = i & 2 ? 0.2f : -0.2f; a[i][2] = b[i][2] = 0.5f; }
        HR(IDirect3DDevice9_CreateVertexDeclaration(g_d, de, &decl));
        HR(IDirect3DDevice9_CreateVertexBuffer(g_d, sizeof a, 0, 0, D3DPOOL_MANAGED, &va, NULL));
        HR(IDirect3DDevice9_CreateVertexBuffer(g_d, sizeof b, 0, 0, D3DPOOL_MANAGED, &vb, NULL));
        IDirect3DVertexBuffer9_Lock(va, 0, 0, &p, 0); memcpy(p, a, sizeof a); IDirect3DVertexBuffer9_Unlock(va);
        IDirect3DVertexBuffer9_Lock(vb, 0, 0, &p, 0); memcpy(p, b, sizeof b); IDirect3DVertexBuffer9_Unlock(vb);
        IDirect3DDevice9_SetVertexDeclaration(g_d, decl);
        IDirect3DDevice9_SetStreamSource(g_d, 0, va, 0, 12);
        IDirect3DDevice9_SetStreamSource(g_d, 1, vb, 0, 12);
        rs(D3DRS_VERTEXBLEND, D3DVBF_TWEENING);
        rs(D3DRS_TWEENFACTOR, fbits(0.5f));
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 2));
        px(W / 2, H / 2, "tweening: half way", 255, 255, 255);
        px(W / 4, H / 2, "tweening: not at the first position", 0, 0, 0);
        rs(D3DRS_TWEENFACTOR, fbits(1.0f));
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 2));
        px(W * 3 / 4, H / 2, "tweening: all the way", 255, 255, 255);
        IDirect3DDevice9_SetStreamSource(g_d, 0, NULL, 0, 0);
        IDirect3DDevice9_SetStreamSource(g_d, 1, NULL, 0, 0);
        IDirect3DDevice9_SetVertexDeclaration(g_d, NULL);
        IDirect3DVertexDeclaration9_Release(decl);
        IDirect3DVertexBuffer9_Release(va);
        IDirect3DVertexBuffer9_Release(vb);
        plain_state();
    }
    /* clip planes with a pixel shader, and with a vertex shader: there the plane is in clip space */
    {
        static const float q[4][3] = { { -1, -1, 0.5f }, { -1, 1, 0.5f }, { 1, -1, 0.5f }, { 1, 1, 0.5f } }, plane[4] = { 1, 0, 0, 0 };
        IDirect3DPixelShader9 *p = g_asm ? ps_of("ps_1_1\ndef c0, 0, 1, 0, 1\nmov r0, c0\n") : NULL;
        IDirect3DVertexShader9 *s = g_asm ? vs_of("vs_2_0\ndcl_position v0\ndef c0, 1, 0, 0, 1\nmov oPos, v0\nmov oD0, c0\n") : NULL;
        IDirect3DDevice9_SetClipPlane(g_d, 0, plane);
        rs(D3DRS_CLIPPLANEENABLE, 1);
        IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZ);
        IDirect3DDevice9_SetPixelShader(g_d, p);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
        IDirect3DDevice9_SetPixelShader(g_d, NULL);
        px(W * 3 / 4, H / 2, "clip plane with a pixel shader: kept", 0, 255, 0);
        px(W / 4, H / 2, "clip plane with a pixel shader: cut", 0, 0, 0);
        IDirect3DDevice9_SetVertexShader(g_d, s);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]));
        IDirect3DDevice9_SetVertexShader(g_d, NULL);
        px(W * 3 / 4, H / 2, "clip plane with a vertex shader: kept", 255, 0, 0);
        px(W / 4, H / 2, "clip plane with a vertex shader: cut", 0, 0, 0);
        rs(D3DRS_CLIPPLANEENABLE, 0);
        if (p) IDirect3DPixelShader9_Release(p);
        if (s) IDirect3DVertexShader9_Release(s);
    }
    IDirect3DTexture9_Release(map);
    plain_state();
}

typedef struct { float x, y, z, w; DWORD c; } tc;
#define FVF_TC (D3DFVF_XYZRHW | D3DFVF_DIFFUSE)

static int lit_near(int x, int y)
{
    IDirect3DSurface9 *rt = NULL;
    D3DLOCKED_RECT l;
    int i, j, found = 0;
    IDirect3DDevice9_GetRenderTarget(g_d, 0, &rt);
    IDirect3DDevice9_GetRenderTargetData(g_d, rt, g_sys);
    IDirect3DSurface9_LockRect(g_sys, &l, NULL, D3DLOCK_READONLY);
    for (j = y - 1; j <= y + 1; j++) for (i = x - 1; i <= x + 1; i++) { const uint8_t *p = (const uint8_t *)l.pBits + (ptrdiff_t)j * l.Pitch + (ptrdiff_t)i * 4; if (p[0] || p[1] || p[2]) found = 1; }
    IDirect3DSurface9_UnlockRect(g_sys);
    IDirect3DSurface9_Release(rt);
    return found;
}

static void test_shade_fill(void)
{
    tc t[3] = { { 0, 0, 0.5f, 1, 0xFFFF0000 }, { 200, 0, 0.5f, 1, 0xFF00FF00 }, { 0, 200, 0.5f, 1, 0xFF0000FF } };
    tc s[4] = { { 0, 0, 0.5f, 1, 0xFFFF0000 }, { 200, 0, 0.5f, 1, 0xFF00FF00 }, { 0, 200, 0.5f, 1, 0xFF0000FF }, { 200, 200, 0.5f, 1, 0xFFFFFF00 } };
    tc f[4] = { { 0, 0, 0.5f, 1, 0xFFFF0000 }, { 200, 0, 0.5f, 1, 0xFF00FF00 }, { 200, 200, 0.5f, 1, 0xFF0000FF }, { 0, 200, 0.5f, 1, 0xFFFFFF00 } };
    static const WORD ix[3] = { 2, 0, 1 };
    plain_state();
    IDirect3DDevice9_SetFVF(g_d, FVF_TC);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, t, sizeof t[0]));
    px(180, 10, "Gouraud: near the second vertex", 12, 229, 12);
    rs(D3DRS_SHADEMODE, D3DSHADE_FLAT);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, t, sizeof t[0]));
    px(180, 10, "flat: the first vertex", 255, 0, 0);
    px(10, 180, "flat: the first vertex, the other corner", 255, 0, 0);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    HR(IDirect3DDevice9_DrawIndexedPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 0, 3, 1, ix, D3DFMT_INDEX16, t, sizeof t[0]));
    px(50, 50, "flat, indexed: the first index", 0, 0, 255);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, s, sizeof s[0]));
    px(40, 40, "flat strip: the first triangle", 255, 0, 0);
    px(160, 160, "flat strip: the second, its first vertex", 0, 255, 0);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLEFAN, 2, f, sizeof f[0]));
    px(160, 40, "flat fan: the one after the hub", 0, 255, 0);
    px(40, 160, "flat fan: the next", 0, 0, 255);
    {   /* a line takes its first vertex */
        tc l[2] = { { 10, 20, 0.5f, 1, 0xFFFF0000 }, { 210, 20, 0.5f, 1, 0xFF00FF00 } };
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_LINELIST, 1, l, sizeof l[0]));
        px(200, 20, "flat line", 255, 0, 0);
    }
    /* lit: the first vertex's normal and position light the whole triangle */
    {
        typedef struct { float x, y, z, nx, ny, nz; } vn;
        vn v[3] = { { -1, 1, 0.5f, 0, 0, -1 }, { 1, 1, 0.5f, 0, 0, 1 }, { -1, -1, 0.5f, 0, 0, 1 } };
        D3DLIGHT9 l;
        D3DMATERIAL9 m;
        static const D3DMATRIX id = { { { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } } };
        memset(&l, 0, sizeof l); memset(&m, 0, sizeof m);
        l.Type = D3DLIGHT_DIRECTIONAL; l.Diffuse.r = 1; l.Diffuse.g = 1; l.Diffuse.b = 1; l.Direction.z = 1;
        m.Diffuse.r = m.Diffuse.g = m.Diffuse.b = m.Diffuse.a = 1;
        IDirect3DDevice9_SetTransform(g_d, D3DTS_WORLD, &id); IDirect3DDevice9_SetTransform(g_d, D3DTS_VIEW, &id); IDirect3DDevice9_SetTransform(g_d, D3DTS_PROJECTION, &id);
        IDirect3DDevice9_SetLight(g_d, 0, &l);
        IDirect3DDevice9_LightEnable(g_d, 0, TRUE);
        IDirect3DDevice9_SetMaterial(g_d, &m);
        rs(D3DRS_LIGHTING, TRUE); rs(D3DRS_AMBIENT, 0);
        IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZ | D3DFVF_NORMAL);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, v, sizeof v[0]));
        px(W - 40, 20, "flat and lit: the first vertex's normal", 255, 255, 255);
        rs(D3DRS_LIGHTING, FALSE);
        IDirect3DDevice9_LightEnable(g_d, 0, FALSE);
    }
    /* a vertex shader: its colour on the first vertex, its position on each */
    if (g_asm) {
        static const float q[3][4] = { { -1, 1, 0.5f, 1 }, { 1, 1, 0.5f, 0 }, { -1, -1, 0.5f, 0 } };
        IDirect3DVertexShader9 *v = vs_of("vs_2_0\ndcl_position v0\ndef c0, 0, 1, 0, 0\nmov r0, v0\nmov r0.w, c0.y\nmov oPos, r0\nmul oD0, v0.w, c0.yxyy\n");
        if (v) {
            IDirect3DDevice9_SetVertexShader(g_d, v);
            IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZW);
            IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
            HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, q, sizeof q[0]));
            IDirect3DDevice9_SetVertexShader(g_d, NULL);
            px(W - 40, 20, "flat with a vertex shader", 255, 0, 255);
            IDirect3DVertexShader9_Release(v);
        } else ok(0, "flat shader");
    }
    rs(D3DRS_SHADEMODE, D3DSHADE_GOURAUD);
    /* wireframe: edges, no inside, culling as for the solid triangle */
    IDirect3DDevice9_SetFVF(g_d, FVF_TC);
    rs(D3DRS_FILLMODE, D3DFILL_WIREFRAME);
    {
        tc w[4] = { { 100, 100, 0.5f, 1, 0xFF00FF00 }, { 300, 100, 0.5f, 1, 0xFF00FF00 }, { 100, 300, 0.5f, 1, 0xFF00FF00 }, { 300, 300, 0.5f, 1, 0xFF00FF00 } };
        tc cw[3] = { { 100, 100, 0.5f, 1, 0xFF00FF00 }, { 300, 100, 0.5f, 1, 0xFF00FF00 }, { 100, 300, 0.5f, 1, 0xFF00FF00 } };
        tc ccw[3] = { { 100, 100, 0.5f, 1, 0xFF00FF00 }, { 100, 300, 0.5f, 1, 0xFF00FF00 }, { 300, 100, 0.5f, 1, 0xFF00FF00 } };
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, w, sizeof w[0]));
        ok(lit_near(200, 100), "wireframe: the top edge");
        ok(lit_near(100, 200), "wireframe: the left edge");
        ok(lit_near(200, 200), "wireframe: the diagonal");
        ok(lit_near(300, 200), "wireframe: the right edge");
        ok(!lit_near(140, 140), "wireframe: not inside");
        rs(D3DRS_CULLMODE, D3DCULL_CCW);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, cw, sizeof cw[0]));
        ok(lit_near(200, 100), "wireframe, D3DCULL_CCW: clockwise drawn");
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, ccw, sizeof ccw[0]));
        ok(!lit_near(200, 100), "wireframe, D3DCULL_CCW: counter-clockwise culled");
        rs(D3DRS_FILLMODE, D3DFILL_POINT);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, cw, sizeof cw[0]));
        ok(lit_near(100, 100) && lit_near(300, 100) && lit_near(100, 300), "point fill: the vertices");
        ok(!lit_near(200, 100), "point fill: no edge");
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
        HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, ccw, sizeof ccw[0]));
        ok(!lit_near(300, 100), "point fill, D3DCULL_CCW: counter-clockwise culled");
        rs(D3DRS_CULLMODE, D3DCULL_NONE);
    }
    /* wireframe culled by what a vertex shader does: it mirrors x, so what looks clockwise in the buffer is counter-clockwise on screen */
    if (g_asm) {
        static const float a[3][3] = { { -0.5f, 0.5f, 0.5f }, { 0.5f, 0.5f, 0.5f }, { -0.5f, -0.5f, 0.5f } };
        IDirect3DVertexShader9 *v = vs_of("vs_2_0\ndcl_position v0\ndef c0, -1, 1, 1, 1\ndef c1, 0, 1, 0, 1\nmul oPos, v0, c0\nmov oD0, c1\n");
        if (v) {
            rs(D3DRS_FILLMODE, D3DFILL_WIREFRAME);
            rs(D3DRS_CULLMODE, D3DCULL_CCW);
            IDirect3DDevice9_SetVertexShader(g_d, v);
            IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZ);
            IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
            HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, a, sizeof a[0]));
            ok(!lit_near(W / 2, H / 4), "wireframe with a vertex shader: mirrored, culled");
            rs(D3DRS_CULLMODE, D3DCULL_CW);
            IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
            HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, 1, a, sizeof a[0]));
            ok(lit_near(W / 2, H / 4), "wireframe with a vertex shader: mirrored, drawn");
            IDirect3DDevice9_SetVertexShader(g_d, NULL);
            rs(D3DRS_CULLMODE, D3DCULL_NONE);
            IDirect3DVertexShader9_Release(v);
        } else ok(0, "mirror shader");
    }
    plain_state();
}

static void test_instancing(void)
{
    static const D3DVERTEXELEMENT9 de[] = { { 0, 0, D3DDECLTYPE_FLOAT3, 0, D3DDECLUSAGE_POSITION, 0 }, { 1, 0, D3DDECLTYPE_FLOAT2, 0, D3DDECLUSAGE_TEXCOORD, 0 }, { 1, 8, D3DDECLTYPE_D3DCOLOR, 0, D3DDECLUSAGE_COLOR, 0 }, D3DDECL_END() };
    typedef struct { float dx, dy; DWORD c; } in1;
    static const float q[4][3] = { { -0.1f, -0.1f, 0.5f }, { -0.1f, 0.1f, 0.5f }, { 0.1f, -0.1f, 0.5f }, { 0.1f, 0.1f, 0.5f } };
    static const in1 ins[4] = { { -0.5f, 0, 0xFFFF0000 }, { 0, 0, 0xFF00FF00 }, { 0.5f, 0, 0xFF0000FF }, { 0, 0.5f, 0xFFFFFFFF } };
    static const WORD ix[4] = { 0, 1, 2, 3 };
    IDirect3DVertexDeclaration9 *decl = NULL;
    IDirect3DVertexBuffer9 *a = NULL, *b = NULL;
    IDirect3DIndexBuffer9 *ib = NULL;
    IDirect3DVertexShader9 *s;
    UINT f = 0;
    void *p;
    if (!g_asm) return;
    plain_state();
    s = vs_of("vs_2_0\ndcl_position v0\ndcl_texcoord v1\ndcl_color v2\nmov r0, v0\nadd r0.xy, v0, v1\nmov oPos, r0\nmov oD0, v2\n");
    HR(IDirect3DDevice9_CreateVertexDeclaration(g_d, de, &decl));
    HR(IDirect3DDevice9_CreateVertexBuffer(g_d, sizeof q, 0, 0, D3DPOOL_MANAGED, &a, NULL));
    HR(IDirect3DDevice9_CreateVertexBuffer(g_d, sizeof ins, 0, 0, D3DPOOL_MANAGED, &b, NULL));
    HR(IDirect3DDevice9_CreateIndexBuffer(g_d, sizeof ix, 0, D3DFMT_INDEX16, D3DPOOL_MANAGED, &ib, NULL));
    if (!s || !decl || !a || !b || !ib) { ok(0, "instancing objects"); return; }
    IDirect3DVertexBuffer9_Lock(a, 0, 0, &p, 0); memcpy(p, q, sizeof q); IDirect3DVertexBuffer9_Unlock(a);
    IDirect3DVertexBuffer9_Lock(b, 0, 0, &p, 0); memcpy(p, ins, sizeof ins); IDirect3DVertexBuffer9_Unlock(b);
    IDirect3DIndexBuffer9_Lock(ib, 0, 0, &p, 0); memcpy(p, ix, sizeof ix); IDirect3DIndexBuffer9_Unlock(ib);
    IDirect3DDevice9_SetVertexDeclaration(g_d, decl);
    IDirect3DDevice9_SetVertexShader(g_d, s);
    IDirect3DDevice9_SetStreamSource(g_d, 0, a, 0, 12);
    IDirect3DDevice9_SetStreamSource(g_d, 1, b, 0, sizeof(in1));
    IDirect3DDevice9_SetIndices(g_d, ib);
    HR(IDirect3DDevice9_SetStreamSourceFreq(g_d, 0, D3DSTREAMSOURCE_INDEXEDDATA | 3));
    HR(IDirect3DDevice9_SetStreamSourceFreq(g_d, 1, D3DSTREAMSOURCE_INSTANCEDATA | 1));
    HR(IDirect3DDevice9_GetStreamSourceFreq(g_d, 0, &f));
    ok(f == (D3DSTREAMSOURCE_INDEXEDDATA | 3), "GetStreamSourceFreq");
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    HR(IDirect3DDevice9_DrawIndexedPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 0, 4, 0, 2));
    px(W / 4, H / 2, "instancing: the first", 255, 0, 0);
    px(W / 2, H / 2, "instancing: the second", 0, 255, 0);
    px(W * 3 / 4, H / 2, "instancing: the third", 0, 0, 255);
    px(W / 2, H / 4, "instancing: no fourth", 0, 0, 0);
    /* data that steps every second instance */
    HR(IDirect3DDevice9_SetStreamSourceFreq(g_d, 0, D3DSTREAMSOURCE_INDEXEDDATA | 4));
    HR(IDirect3DDevice9_SetStreamSourceFreq(g_d, 1, D3DSTREAMSOURCE_INSTANCEDATA | 2));
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    HR(IDirect3DDevice9_DrawIndexedPrimitive(g_d, D3DPT_TRIANGLESTRIP, 0, 0, 4, 0, 2));
    px(W / 4, H / 2, "instancing, every second: 0 and 1 at the first", 255, 0, 0);
    px(W / 2, H / 2, "instancing, every second: 2 and 3 at the second", 0, 255, 0);
    px(W * 3 / 4, H / 2, "instancing, every second: not the third", 0, 0, 0);
    IDirect3DDevice9_SetStreamSourceFreq(g_d, 0, 1);
    IDirect3DDevice9_SetStreamSourceFreq(g_d, 1, 1);
    IDirect3DDevice9_SetStreamSource(g_d, 0, NULL, 0, 0);
    IDirect3DDevice9_SetStreamSource(g_d, 1, NULL, 0, 0);
    IDirect3DDevice9_SetIndices(g_d, NULL);
    IDirect3DDevice9_SetVertexDeclaration(g_d, NULL);
    IDirect3DDevice9_SetVertexShader(g_d, NULL);
    IDirect3DVertexDeclaration9_Release(decl);
    IDirect3DVertexBuffer9_Release(a);
    IDirect3DVertexBuffer9_Release(b);
    IDirect3DIndexBuffer9_Release(ib);
    IDirect3DVertexShader9_Release(s);
    plain_state();
}

static void test_device_extra(void)
{
    IDirect3DSurface9 *back = NULL, *ds = NULL, *small = NULL, *bb = NULL;
    IDirect3DSwapChain9 *ch = NULL, *ch0 = NULL;
    D3DPRESENT_PARAMETERS pp;
    D3DSURFACE_DESC de;
    plain_state();
    IDirect3DDevice9_GetRenderTarget(g_d, 0, &back);
    IDirect3DDevice9_GetDepthStencilSurface(g_d, &ds);
    /* a depth surface larger than its target: its top-left, both ways */
    HR(IDirect3DDevice9_CreateRenderTarget(g_d, 128, 128, D3DFMT_X8R8G8B8, D3DMULTISAMPLE_NONE, 0, FALSE, &small, NULL));
    rs(D3DRS_ZENABLE, TRUE); rs(D3DRS_ZFUNC, D3DCMP_LESS); rs(D3DRS_ZWRITEENABLE, TRUE);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
    quad(0, 0, 64, 128, 0.25f, D3DCOLOR_XRGB(0, 255, 0));
    HR(IDirect3DDevice9_SetRenderTarget(g_d, 0, small));
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    quad(0, 0, 128, 128, 0.5f, D3DCOLOR_XRGB(255, 0, 0));
    {
        IDirect3DSurface9 *m = NULL;
        D3DLOCKED_RECT l;
        HR(IDirect3DDevice9_CreateOffscreenPlainSurface(g_d, 128, 128, D3DFMT_X8R8G8B8, D3DPOOL_SYSTEMMEM, &m, NULL));
        HR(IDirect3DDevice9_GetRenderTargetData(g_d, small, m));
        IDirect3DSurface9_LockRect(m, &l, NULL, D3DLOCK_READONLY);
        ok(((const uint8_t *)l.pBits)[20 * l.Pitch + 20 * 4 + 2] == 0, "larger depth surface: what the back buffer left there holds");
        ok(((const uint8_t *)l.pBits)[20 * l.Pitch + 100 * 4 + 2] == 255, "larger depth surface: the rest passes");
        IDirect3DSurface9_UnlockRect(m);
        IDirect3DSurface9_Release(m);
    }
    quad(100, 0, 128, 128, 0.125f, D3DCOLOR_XRGB(255, 0, 0));    /* into the depth surface through the smaller target */
    HR(IDirect3DDevice9_SetRenderTarget(g_d, 0, back));
    quad(0, 0, W, H, 0.25f, D3DCOLOR_XRGB(0, 0, 255));
    px(110, 20, "larger depth surface: what the smaller target wrote is there", 0, 0, 0);
    px(300, 20, "larger depth surface: the rest is as it was", 0, 0, 255);
    rs(D3DRS_ZENABLE, FALSE); rs(D3DRS_ZFUNC, D3DCMP_LESSEQUAL);
    IDirect3DSurface9_Release(small);
    /* an additional swap chain: its own back buffer */
    memset(&pp, 0, sizeof pp);
    pp.BackBufferWidth = 320; pp.BackBufferHeight = 200; pp.BackBufferFormat = D3DFMT_X8R8G8B8; pp.SwapEffect = D3DSWAPEFFECT_COPY; pp.Windowed = TRUE;
    HR(IDirect3DDevice9_CreateAdditionalSwapChain(g_d, &pp, &ch));
    if (ch) {
        HR(IDirect3DSwapChain9_GetBackBuffer(ch, 0, D3DBACKBUFFER_TYPE_MONO, &bb));
        IDirect3DSurface9_GetDesc(bb, &de);
        ok(de.Width == 320 && de.Height == 200, "additional swap chain: its size");
        HR(IDirect3DDevice9_SetRenderTarget(g_d, 0, bb));
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(0, 0, 40), 1.0f, 0);
        quad(10, 10, 50, 50, 0.5f, D3DCOLOR_XRGB(0, 200, 0));
        {
            IDirect3DSurface9 *m = NULL;
            D3DLOCKED_RECT l;
            HR(IDirect3DDevice9_CreateOffscreenPlainSurface(g_d, 320, 200, D3DFMT_X8R8G8B8, D3DPOOL_SYSTEMMEM, &m, NULL));
            HR(IDirect3DDevice9_GetRenderTargetData(g_d, bb, m));
            IDirect3DSurface9_LockRect(m, &l, NULL, D3DLOCK_READONLY);
            ok(((const uint8_t *)l.pBits)[20 * l.Pitch + 20 * 4 + 1] == 200, "additional swap chain: renders into its back buffer");
            IDirect3DSurface9_UnlockRect(m);
            IDirect3DSurface9_Release(m);
        }
        HR(IDirect3DSwapChain9_Present(ch, NULL, NULL, NULL, NULL, 0));
        HR(IDirect3DDevice9_SetRenderTarget(g_d, 0, back));
        IDirect3DSurface9_Release(bb);
        IDirect3DSwapChain9_Release(ch);
    }
    HR(IDirect3DDevice9_GetSwapChain(g_d, 0, &ch0));
    if (ch0) { HR(IDirect3DSwapChain9_Present(ch0, NULL, NULL, NULL, NULL, 0)); IDirect3DSwapChain9_Release(ch0); }
    ok(IDirect3DDevice9_GetNumberOfSwapChains(g_d) == 1, "one implicit swap chain");
    /* ProcessVertices: fixed function, lit, and a vertex shader */
    {
        typedef struct { float x, y, z, nx, ny, nz; DWORD c; float u, v; } in1;
        typedef struct { float x, y, z, w; DWORD c, s; float u, v; } out1;
        static const D3DMATRIX id = { { { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } } };
        in1 v[2] = { { 0, 0, 0.5f, 0, 0, -1, 0xFF804020, 0.25f, 0.75f }, { 0.5f, 0.5f, 0.25f, 0, 0, -1, 0xFF804020, 1, 0 } };
        IDirect3DVertexBuffer9 *a = NULL, *o = NULL;
        D3DMATRIX tr = id;
        void *p;
        out1 *r;
        HR(IDirect3DDevice9_CreateVertexBuffer(g_d, sizeof v, 0, 0, D3DPOOL_MANAGED, &a, NULL));
        HR(IDirect3DDevice9_CreateVertexBuffer(g_d, sizeof(out1) * 4, 0, D3DFVF_XYZRHW | D3DFVF_DIFFUSE | D3DFVF_SPECULAR | D3DFVF_TEX1, D3DPOOL_MANAGED, &o, NULL));
        IDirect3DVertexBuffer9_Lock(a, 0, 0, &p, 0); memcpy(p, v, sizeof v); IDirect3DVertexBuffer9_Unlock(a);
        IDirect3DDevice9_SetFVF(g_d, D3DFVF_XYZ | D3DFVF_NORMAL | D3DFVF_DIFFUSE | D3DFVF_TEX1);
        IDirect3DDevice9_SetStreamSource(g_d, 0, a, 0, sizeof v[0]);
        IDirect3DDevice9_SetTransform(g_d, D3DTS_VIEW, &id); IDirect3DDevice9_SetTransform(g_d, D3DTS_PROJECTION, &id);
        tr.m[3][0] = 0.5f;
        IDirect3DDevice9_SetTransform(g_d, D3DTS_WORLD, &tr);
        HR(IDirect3DDevice9_ProcessVertices(g_d, 0, 1, 2, o, NULL, 0));
        IDirect3DVertexBuffer9_Lock(o, 0, 0, &p, D3DLOCK_READONLY);
        r = (out1 *)p + 1;
        ok(fabs(r[0].x - W * 0.75) < 0.01 && fabs(r[0].y - H * 0.5) < 0.01 && fabs(r[0].z - 0.5) < 0.001 && fabs(r[0].w - 1.0) < 0.001, "ProcessVertices: the position");
        ok(fabs(r[1].x - W) < 0.01 && fabs(r[1].y - H * 0.25) < 0.01, "ProcessVertices: the next vertex");
        ok(r[0].c == 0xFF804020, "ProcessVertices: the colour copied");
        ok(r[0].u == 0.25f && r[0].v == 0.75f, "ProcessVertices: the coordinates copied");
        IDirect3DVertexBuffer9_Unlock(o);
        {
            D3DLIGHT9 l;
            D3DMATERIAL9 m;
            memset(&l, 0, sizeof l); memset(&m, 0, sizeof m);
            l.Type = D3DLIGHT_DIRECTIONAL; l.Diffuse.r = 1; l.Diffuse.g = 0.5f; l.Diffuse.b = 0.25f; l.Direction.z = 1;
            m.Diffuse.r = m.Diffuse.g = m.Diffuse.b = m.Diffuse.a = 1;
            IDirect3DDevice9_SetLight(g_d, 0, &l); IDirect3DDevice9_LightEnable(g_d, 0, TRUE); IDirect3DDevice9_SetMaterial(g_d, &m);
            rs(D3DRS_LIGHTING, TRUE); rs(D3DRS_AMBIENT, 0); rs(D3DRS_COLORVERTEX, FALSE);
            HR(IDirect3DDevice9_ProcessVertices(g_d, 0, 0, 1, o, NULL, 0));
            rs(D3DRS_LIGHTING, FALSE); rs(D3DRS_COLORVERTEX, TRUE); IDirect3DDevice9_LightEnable(g_d, 0, FALSE);
            IDirect3DVertexBuffer9_Lock(o, 0, 0, &p, D3DLOCK_READONLY);
            ok(((out1 *)p)[0].c == 0xFFFF8040, "ProcessVertices: lit");
            IDirect3DVertexBuffer9_Unlock(o);
        }
        if (g_asm) {
            IDirect3DVertexShader9 *s = vs_of("vs_2_0\ndcl_position v0\ndefi i0, 2, 0, 0, 0\ndef c0, 0.25, 0, 0, 0\ndef c1, 0, 1, 0, 0.5\nmov r0, v0\nrep i0\nadd r0, r0, c0\nendrep\nmov oPos, r0\nmov oD0, c1\n");
            if (s) {
                IDirect3DDevice9_SetVertexShader(g_d, s);
                HR(IDirect3DDevice9_ProcessVertices(g_d, 0, 0, 1, o, NULL, 0));
                IDirect3DDevice9_SetVertexShader(g_d, NULL);
                IDirect3DVertexBuffer9_Lock(o, 0, 0, &p, D3DLOCK_READONLY);
                ok(fabs(((out1 *)p)[0].x - W * 0.75) < 0.01, "ProcessVertices with a vertex shader: a rep in it");
                ok(((out1 *)p)[0].c == 0x8000FF00, "ProcessVertices with a vertex shader: its colour");
                IDirect3DVertexBuffer9_Unlock(o);
                IDirect3DVertexShader9_Release(s);
            } else ok(0, "ProcessVertices shader");
        }
        IDirect3DDevice9_SetTransform(g_d, D3DTS_WORLD, &id);
        IDirect3DDevice9_SetStreamSource(g_d, 0, NULL, 0, 0);
        IDirect3DVertexBuffer9_Release(a);
        IDirect3DVertexBuffer9_Release(o);
    }
    /* queries: time on the PC's clock when the board has done */
    {
        IDirect3DQuery9 *q1 = NULL, *q2 = NULL, *f = NULL, *dj = NULL, *vc = NULL;
        UINT64 a = 0, b = 0, hz = 0;
        BOOL dis = TRUE;
        D3DDEVINFO_VCACHE c;
        ok(IDirect3DDevice9_CreateQuery(g_d, D3DQUERYTYPE_TIMESTAMP, NULL) == D3D_OK, "CreateQuery(NULL): timestamps are there");
        ok(IDirect3DDevice9_CreateQuery(g_d, D3DQUERYTYPE_OCCLUSION, NULL) != D3D_OK, "CreateQuery(NULL): occlusion is not");
        HR(IDirect3DDevice9_CreateQuery(g_d, D3DQUERYTYPE_TIMESTAMP, &q1));
        HR(IDirect3DDevice9_CreateQuery(g_d, D3DQUERYTYPE_TIMESTAMP, &q2));
        HR(IDirect3DDevice9_CreateQuery(g_d, D3DQUERYTYPE_TIMESTAMPFREQ, &f));
        HR(IDirect3DDevice9_CreateQuery(g_d, D3DQUERYTYPE_TIMESTAMPDISJOINT, &dj));
        HR(IDirect3DDevice9_CreateQuery(g_d, D3DQUERYTYPE_VCACHE, &vc));
        ok(IDirect3DQuery9_GetDataSize(q1) == 8 && IDirect3DQuery9_GetDataSize(dj) == sizeof(BOOL) && IDirect3DQuery9_GetDataSize(vc) == sizeof c, "query sizes");
        ok(IDirect3DQuery9_GetData(q1, &a, 8, 0) == S_FALSE, "a timestamp not issued has nothing");
        IDirect3DQuery9_Issue(dj, D3DISSUE_BEGIN);
        IDirect3DQuery9_Issue(q1, D3DISSUE_END);
        Sleep(20);
        IDirect3DQuery9_Issue(q2, D3DISSUE_END);
        IDirect3DQuery9_Issue(dj, D3DISSUE_END);
        IDirect3DQuery9_Issue(f, D3DISSUE_END);
        HR(IDirect3DQuery9_GetData(q1, &a, 8, D3DGETDATA_FLUSH));
        HR(IDirect3DQuery9_GetData(q2, &b, 8, D3DGETDATA_FLUSH));
        HR(IDirect3DQuery9_GetData(f, &hz, 8, D3DGETDATA_FLUSH));
        HR(IDirect3DQuery9_GetData(dj, &dis, sizeof dis, D3DGETDATA_FLUSH));
        HR(IDirect3DQuery9_GetData(vc, &c, sizeof c, D3DGETDATA_FLUSH));
        ok(hz && b > a && (double)(b - a) / (double)hz > 0.015, "timestamps: 20 ms apart");
        ok(!dis, "timestamps: not disjoint");
        ok(c.Pattern == MAKEFOURCC('C', 'A', 'C', 'H'), "VCACHE");
        IDirect3DQuery9_Release(q1); IDirect3DQuery9_Release(q2); IDirect3DQuery9_Release(f); IDirect3DQuery9_Release(dj); IDirect3DQuery9_Release(vc);
    }
    IDirect3DSurface9_Release(back);
    if (ds) IDirect3DSurface9_Release(ds);
    plain_state();
}

static LRESULT CALLBACK wnd_proc(HWND w, UINT m, WPARAM a, LPARAM b) { return DefWindowProcA(w, m, a, b); }

static void quad_uv(float x0, float y0, float x1, float y1, float u0, float v0, float u1, float v1)
{
    tlv v[4] = { { x0, y0, 0.5f, 1, 0xFFFFFFFF, u0, v0 }, { x1, y0, 0.5f, 1, 0xFFFFFFFF, u1, v0 }, { x0, y1, 0.5f, 1, 0xFFFFFFFF, u0, v1 }, { x1, y1, 0.5f, 1, 0xFFFFFFFF, u1, v1 } };
    IDirect3DDevice9_SetFVF(g_d, FVF_TL);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, v, sizeof v[0]));
}

static void samp(int s, D3DSAMPLERSTATETYPE ty, DWORD v) { IDirect3DDevice9_SetSamplerState(g_d, (DWORD)s, ty, v); }
static void stage_tex(int s, IDirect3DBaseTexture9 *t)
{
    IDirect3DDevice9_SetTexture(g_d, (DWORD)s, t);
    stage(s, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    stage(s, D3DTSS_COLORARG1, D3DTA_TEXTURE);
    stage(s, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
    stage(s, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
}

/* 4 x 4 red, 2 x 2 green, 1 x 1 blue */
static IDirect3DTexture9 *tex_chain(void)
{
    static const DWORD c[3] = { 0xFFFF0000, 0xFF00FF00, 0xFF0000FF };
    IDirect3DTexture9 *t = NULL;
    D3DLOCKED_RECT l;
    int m, x, y;
    HR(IDirect3DDevice9_CreateTexture(g_d, 4, 4, 3, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &t, NULL));
    if (!t) return NULL;
    for (m = 0; m < 3; m++) {
        IDirect3DTexture9_LockRect(t, (UINT)m, &l, NULL, 0);
        for (y = 0; y < 4 >> m; y++) for (x = 0; x < 4 >> m; x++) ((DWORD *)((uint8_t *)l.pBits + y * l.Pitch))[x] = c[m];
        IDirect3DTexture9_UnlockRect(t, (UINT)m);
    }
    return t;
}

/* a 4 x 4 texture on 4 x 4 pixels: level 0 before anything moves it */
static void level_scene(const char *what, int r, int g, int b)
{
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    quad_uv(100, 100, 104, 104, 0, 0, 1, 1);
    px(101, 101, what, r, g, b);
}

/* the sampler states the TMU has nothing for: border, mirror once, LOD bias, the largest level, sRGB; and the last texture op */
static void test_sampler_extra(IDirect3D9 *d3d)
{
    D3DCAPS9 c;
    IDirect3DTexture9 *t;
    plain_state();
    HR(IDirect3DDevice9_GetDeviceCaps(g_d, &c));
    ok(c.RasterCaps & D3DPRASTERCAPS_MIPMAPLODBIAS, "D3DPRASTERCAPS_MIPMAPLODBIAS");
    ok((c.TextureAddressCaps & (D3DPTADDRESSCAPS_BORDER | D3DPTADDRESSCAPS_MIRRORONCE)) == (D3DPTADDRESSCAPS_BORDER | D3DPTADDRESSCAPS_MIRRORONCE), "border and mirror once are in the caps");
    ok(c.TextureOpCaps & D3DTEXOPCAPS_PREMODULATE, "D3DTEXOPCAPS_PREMODULATE");
    ok(IDirect3D9_CheckDeviceFormat(d3d, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, D3DUSAGE_QUERY_SRGBREAD, D3DRTYPE_TEXTURE, D3DFMT_A8R8G8B8) == D3D_OK, "sRGB read");
    ok(IDirect3D9_CheckDeviceFormat(d3d, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, D3DUSAGE_QUERY_SRGBWRITE | D3DUSAGE_RENDERTARGET, D3DRTYPE_TEXTURE, D3DFMT_A8R8G8B8) == D3D_OK, "sRGB write");
    ok(IDirect3D9_CheckDeviceFormat(d3d, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, D3DUSAGE_QUERY_SRGBREAD, D3DRTYPE_TEXTURE, D3DFMT_R5G6B5) != D3D_OK, "no sRGB in 16 bits");
    /* border, point and linear */
    t = tex_one(0xFF00FF00);
    stage_tex(0, (IDirect3DBaseTexture9 *)t);
    samp(0, D3DSAMP_ADDRESSU, D3DTADDRESS_BORDER); samp(0, D3DSAMP_ADDRESSV, D3DTADDRESS_BORDER);
    samp(0, D3DSAMP_BORDERCOLOR, 0xFFFF0000);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    quad_uv(0, 0, 300, 300, -1, -1, 2, 2);
    px(50, 150, "border: left of the texture", 255, 0, 0);
    px(150, 150, "border: on the texture", 0, 255, 0);
    px(250, 150, "border: right of it", 255, 0, 0);
    px(150, 50, "border: above it", 255, 0, 0);
    samp(0, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);
    quad_uv(0, 0, 300, 300, -1, -1, 2, 2);
    px(150, 50, "border on U alone: V clamps", 0, 255, 0);
    px(50, 150, "border on U alone: U still borders", 255, 0, 0);
    samp(0, D3DSAMP_ADDRESSV, D3DTADDRESS_BORDER);
    samp(0, D3DSAMP_MINFILTER, D3DTEXF_LINEAR); samp(0, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR);
    quad_uv(0, 0, 300, 300, 0.5f, 0.5f, 1.5f, 1.5f);                    /* the edge of a 1 x 1 texture, linear: half a texel of the footprint past the middle of the texel */
    px(5, 5, "border, linear: near the middle of the texel", 8, 246, 0);
    px(298, 150, "border, linear: past the texture", 255, 0, 0);
    px(150, 5, "border, linear: half and half at the edge", 129, 126, 0);
    px(150, 150, "border, linear: a quarter at the corner", 191, 64, 0);
    samp(0, D3DSAMP_MINFILTER, D3DTEXF_POINT); samp(0, D3DSAMP_MAGFILTER, D3DTEXF_POINT);
    {   /* through a pixel shader */
        static const float tc[4][4] = { { -0.5f, 0.5f, 0, 1 } };
        ps_scene("border from a pixel shader", "ps_2_0\ndcl t0\ndcl_2d s0\ntexld r0, t0, s0\nmov oC0, r0\n", tc, 255, 0, 0);
    }
    IDirect3DDevice9_SetTexture(g_d, 0, NULL);
    IDirect3DTexture9_Release(t);
    /* mirror once: the coordinate's size, then a clamp */
    t = tex_map();
    stage_tex(0, (IDirect3DBaseTexture9 *)t);
    samp(0, D3DSAMP_ADDRESSU, D3DTADDRESS_MIRRORONCE); samp(0, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    quad_uv(0, 0, 400, 100, -2, 0, 2, 0.25f);                           /* u = -2 + x / 100 */
    px(10, 10, "mirror once: past -1 it clamps", 0, 255, 0);
    px(125, 10, "mirror once: -0.75", 0, 255, 0);
    px(175, 10, "mirror once: -0.25", 255, 0, 0);
    px(225, 10, "mirror once: 0.25", 255, 0, 0);
    px(275, 10, "mirror once: 0.75", 0, 255, 0);
    px(390, 10, "mirror once: past 1 it clamps", 0, 255, 0);
    samp(0, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP);
    IDirect3DDevice9_SetTexture(g_d, 0, NULL);
    IDirect3DTexture9_Release(t);
    /* the level */
    t = tex_chain();
    stage_tex(0, (IDirect3DBaseTexture9 *)t);
    samp(0, D3DSAMP_MIPFILTER, D3DTEXF_POINT);
    level_scene("no bias: level 0", 255, 0, 0);
    samp(0, D3DSAMP_MIPMAPLODBIAS, fbits(1.0f));
    level_scene("D3DSAMP_MIPMAPLODBIAS 1", 0, 255, 0);
    samp(0, D3DSAMP_MIPMAPLODBIAS, fbits(2.0f));
    level_scene("D3DSAMP_MIPMAPLODBIAS 2", 0, 0, 255);
    {   /* in a shader: texld and texldb both */
        IDirect3DPixelShader9 *p = ps_of("ps_2_0\ndcl t0\ndcl_2d s0\ntexld r0, t0, s0\nmov oC0, r0\n");
        if (p) {
            IDirect3DDevice9_SetPixelShader(g_d, p);
            samp(0, D3DSAMP_MIPMAPLODBIAS, fbits(1.0f));
            level_scene("a shader's texld takes the sampler's bias", 0, 255, 0);
            IDirect3DDevice9_SetPixelShader(g_d, NULL);
            IDirect3DPixelShader9_Release(p);
        }
        p = ps_of("ps_2_0\ndef c0, 0.0, 0.0, 0.0, 1.0\ndcl t0\ndcl_2d s0\nmov r1, t0\nmov r1.w, c0.w\ntexldb r0, r1, s0\nmov oC0, r0\n");
        if (p) {
            IDirect3DDevice9_SetPixelShader(g_d, p);
            level_scene("texldb adds to the sampler's bias", 0, 0, 255);
            IDirect3DDevice9_SetPixelShader(g_d, NULL);
            IDirect3DPixelShader9_Release(p);
        }
    }
    samp(0, D3DSAMP_MIPMAPLODBIAS, 0);
    samp(0, D3DSAMP_MAXMIPLEVEL, 1);
    level_scene("D3DSAMP_MAXMIPLEVEL 1", 0, 255, 0);
    samp(0, D3DSAMP_MAXMIPLEVEL, 2);
    level_scene("D3DSAMP_MAXMIPLEVEL 2", 0, 0, 255);
    samp(0, D3DSAMP_MAXMIPLEVEL, 0);
    level_scene("D3DSAMP_MAXMIPLEVEL back to 0", 255, 0, 0);
    ok(IDirect3DTexture9_SetLOD(t, 1) == 0, "SetLOD: the old value");
    ok(IDirect3DTexture9_GetLOD(t) == 1, "GetLOD");
    level_scene("SetLOD 1", 0, 255, 0);
    IDirect3DTexture9_SetLOD(t, 0);
    level_scene("SetLOD 0", 255, 0, 0);
    samp(0, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
    IDirect3DDevice9_SetTexture(g_d, 0, NULL);
    IDirect3DTexture9_Release(t);
    {   /* the largest level of a cube map: its chain cannot be shared, the levels are copied, and a write brings a new copy */
        static const DWORD c[2] = { 0xFFFF0000, 0xFF00FF00 };
        IDirect3DCubeTexture9 *q = NULL;
        D3DLOCKED_RECT l;
        int f, m;
        static const float dir[4][4] = { { 1, 0, 0, 1 } };
        HR(IDirect3DDevice9_CreateCubeTexture(g_d, 2, 2, 0 + 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &q, NULL));
        if (q) {
            for (f = 0; f < 6; f++) for (m = 0; m < 2; m++) {
                int x, n = 2 >> m;
                IDirect3DCubeTexture9_LockRect(q, (D3DCUBEMAP_FACES)f, (UINT)m, &l, NULL, 0);
                for (x = 0; x < n * n; x++) ((DWORD *)((uint8_t *)l.pBits + (x / n) * l.Pitch))[x % n] = c[m];
                IDirect3DCubeTexture9_UnlockRect(q, (D3DCUBEMAP_FACES)f, (UINT)m);
            }
            stage_tex(0, (IDirect3DBaseTexture9 *)q);
            samp(0, D3DSAMP_MIPFILTER, D3DTEXF_POINT);
            samp(0, D3DSAMP_MAXMIPLEVEL, 1);
            stage(0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
            IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
            quad_4t(dir);
            px(W / 2, H / 2, "a cube map's D3DSAMP_MAXMIPLEVEL", 0, 255, 0);
            IDirect3DCubeTexture9_LockRect(q, D3DCUBEMAP_FACE_POSITIVE_X, 1, &l, NULL, 0);
            *(DWORD *)l.pBits = 0xFF0000FF;
            IDirect3DCubeTexture9_UnlockRect(q, D3DCUBEMAP_FACE_POSITIVE_X, 1);
            quad_4t(dir);
            px(W / 2, H / 2, "a write reaches the copy", 0, 0, 255);
            samp(0, D3DSAMP_MAXMIPLEVEL, 0); samp(0, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
            IDirect3DDevice9_SetTexture(g_d, 0, NULL);
            IDirect3DCubeTexture9_Release(q);
        }
    }
    /* sRGB */
    t = tex_one(0xFF808080);
    stage_tex(0, (IDirect3DBaseTexture9 *)t);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    samp(0, D3DSAMP_SRGBTEXTURE, TRUE);
    quad(0, 0, 100, 100, 0.5f, 0xFFFFFFFF);
    px(50, 50, "D3DSAMP_SRGBTEXTURE: 128 is 0.2158", 55, 55, 55);
    rs(D3DRS_SRGBWRITEENABLE, TRUE);
    quad(0, 0, 100, 100, 0.5f, 0xFFFFFFFF);
    px(50, 50, "and D3DRS_SRGBWRITEENABLE brings it back", 128, 128, 128);
    samp(0, D3DSAMP_SRGBTEXTURE, FALSE);
    stage(0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    IDirect3DDevice9_SetTexture(g_d, 0, NULL);
    quad(0, 0, 100, 100, 0.5f, D3DCOLOR_XRGB(55, 55, 55));
    px(50, 50, "D3DRS_SRGBWRITEENABLE on a plain colour", 128, 128, 128);
    rs(D3DRS_SRGBWRITEENABLE, FALSE);
    IDirect3DTexture9_Release(t);
    /* PREMODULATE: this stage's argument with the next stage's texture */
    t = tex_one(0xFF8040FF);
    plain_state();
    IDirect3DDevice9_SetTexture(g_d, 1, (IDirect3DBaseTexture9 *)t);
    stage(0, D3DTSS_COLOROP, D3DTOP_PREMODULATE);
    stage(0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    stage(1, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    stage(1, D3DTSS_COLORARG1, D3DTA_CURRENT);
    stage(1, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
    stage(1, D3DTSS_ALPHAARG1, D3DTA_CURRENT);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    quad(0, 0, 100, 100, 0.5f, D3DCOLOR_XRGB(255, 128, 0));
    px(50, 50, "D3DTOP_PREMODULATE", 128, 32, 0);
    IDirect3DDevice9_SetTexture(g_d, 1, NULL);
    IDirect3DTexture9_Release(t);
    plain_state();
}

/* volume textures: the TMU's (a board B built with them) */
typedef struct { float x, y, z, w, u, v, r, q; } v3t;
#define FVF_3T (D3DFVF_XYZRHW | D3DFVF_TEX1 | D3DFVF_TEXCOORDSIZE4(0))
static void quad_3t(float r, float q)
{
    v3t v[4];
    int i;
    for (i = 0; i < 4; i++) { v[i].x = i & 1 ? (float)W : 0.0f; v[i].y = i & 2 ? (float)H : 0.0f; v[i].z = 0.5f; v[i].w = 1.0f; v[i].u = (i & 1 ? 1.0f : 0.0f) * q; v[i].v = (i & 2 ? 1.0f : 0.0f) * q; v[i].r = r * q; v[i].q = q; }
    IDirect3DDevice9_SetFVF(g_d, FVF_3T);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, v, sizeof v[0]));
}

/* red = x, green = y, blue = z of 4 x 4 x 4 */
static IDirect3DVolumeTexture9 *vol_grad(UINT levels, D3DPOOL pool)
{
    IDirect3DVolumeTexture9 *t = NULL;
    D3DLOCKED_BOX b;
    int x, y, z;
    HR(IDirect3DDevice9_CreateVolumeTexture(g_d, 4, 4, 4, levels, 0, D3DFMT_A8R8G8B8, pool, &t, NULL));
    if (!t) return NULL;
    HR(IDirect3DVolumeTexture9_LockBox(t, 0, &b, NULL, 0));
    for (z = 0; z < 4; z++) for (y = 0; y < 4; y++) for (x = 0; x < 4; x++) *(DWORD *)((uint8_t *)b.pBits + z * b.SlicePitch + y * b.RowPitch + x * 4) = D3DCOLOR_ARGB(255, x * 85, y * 85, z * 85);
    HR(IDirect3DVolumeTexture9_UnlockBox(t, 0));
    return t;
}

static void vol_stage(IDirect3DVolumeTexture9 *t, DWORD filter, DWORD addr)
{
    IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)t);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_MAGFILTER, filter);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_MINFILTER, filter);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_ADDRESSW, addr);
}

static void test_volume(IDirect3D9 *d3d)
{
    static const float mid[4][4] = { { 0.125f, 0.125f, 0.625f, 1.0f } };
    static const float proj[4][4] = { { 0.25f, 0.25f, 1.25f, 2.0f } };
    IDirect3DVolumeTexture9 *t, *s, *o = NULL;
    IDirect3DVolume9 *lv = NULL;
    IDirect3DBaseTexture9 *got = NULL;
    D3DVOLUME_DESC de;
    D3DLOCKED_BOX b;
    D3DBOX box = { 1, 2, 3, 3, 1, 2 };
    D3DCAPS9 c;
    IUnknown *u = NULL;
    void *q = NULL;
    int z;
    HR(IDirect3DDevice9_GetDeviceCaps(g_d, &c));
    if (!(c.TextureCaps & D3DPTEXTURECAPS_VOLUMEMAP)) {          /* a board B without 3D textures (or GPU2_NO_3D=1): no volumes, and every way of asking says so */
        D3DCAPS9 c0;
        HR(IDirect3D9_GetDeviceCaps(d3d, 0, D3DDEVTYPE_HAL, &c0));
        ok(!(c0.TextureCaps & (D3DPTEXTURECAPS_VOLUMEMAP | D3DPTEXTURECAPS_MIPVOLUMEMAP)) && !(c.TextureCaps & D3DPTEXTURECAPS_MIPVOLUMEMAP) && !c.MaxVolumeExtent && !c.VolumeTextureFilterCaps && !c.VolumeTextureAddressCaps, "no volume caps");
        ok(IDirect3D9_CheckDeviceFormat(d3d, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, 0, D3DRTYPE_VOLUMETEXTURE, D3DFMT_A8R8G8B8) == D3DERR_NOTAVAILABLE, "no volume format");
        ok(IDirect3DDevice9_CreateVolumeTexture(g_d, 2, 2, 2, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &o, NULL) == D3DERR_NOTAVAILABLE, "no volume texture");
        return;
    }
    HR(IDirect3D9_CheckDeviceFormat(d3d, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, 0, D3DRTYPE_VOLUMETEXTURE, D3DFMT_A8R8G8B8));
    ok(IDirect3D9_CheckDeviceFormat(d3d, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, D3DUSAGE_RENDERTARGET, D3DRTYPE_VOLUMETEXTURE, D3DFMT_A8R8G8B8) != D3D_OK, "a volume is no target");
    ok((c.TextureCaps & D3DPTEXTURECAPS_VOLUMEMAP) && (c.TextureCaps & D3DPTEXTURECAPS_MIPVOLUMEMAP) && !(c.TextureCaps & D3DPTEXTURECAPS_VOLUMEMAP_POW2), "volume caps");
    ok(c.MaxVolumeExtent == 255 && c.VolumeTextureFilterCaps == c.TextureFilterCaps && c.VolumeTextureAddressCaps == c.TextureAddressCaps, "volume limits");
    ok(IDirect3DDevice9_CreateVolumeTexture(g_d, 257, 2, 2, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &o, NULL) == D3DERR_INVALIDCALL, "257 wide");
    ok(IDirect3DDevice9_CreateVolumeTexture(g_d, 2, 2, 256, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &o, NULL) == D3DERR_INVALIDCALL, "256 deep");
    ok(IDirect3DDevice9_CreateVolumeTexture(g_d, 2, 2, 2, 1, D3DUSAGE_RENDERTARGET, D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &o, NULL) == D3DERR_INVALIDCALL, "not a target");
    t = vol_grad(0, D3DPOOL_MANAGED);
    if (!t) return;
    ok(IDirect3DVolumeTexture9_GetLevelCount(t) == 3, "a volume's level count");
    ok(IDirect3DVolumeTexture9_GetType(t) == D3DRTYPE_VOLUMETEXTURE, "GetType");
    HR(IDirect3DVolumeTexture9_GetLevelDesc(t, 1, &de));
    ok(de.Width == 2 && de.Height == 2 && de.Depth == 2 && de.Format == D3DFMT_A8R8G8B8 && de.Type == D3DRTYPE_VOLUME, "GetLevelDesc");
    ok(IDirect3DVolumeTexture9_GetLevelDesc(t, 3, &de) == D3DERR_INVALIDCALL, "GetLevelDesc past the last");
    HR(IDirect3DVolumeTexture9_GetVolumeLevel(t, 0, &lv));
    if (lv) {
        HR(IDirect3DVolume9_GetDesc(lv, &de));
        ok(de.Depth == 4, "the level's GetDesc");
        HR(IDirect3DVolume9_GetContainer(lv, &IID_IDirect3DVolumeTexture9, &q));
        ok(q == t, "GetContainer");
        if (q) IDirect3DVolumeTexture9_Release((IDirect3DVolumeTexture9 *)q);
        HR(IDirect3DVolume9_QueryInterface(lv, &IID_IDirect3DVolume9, (void **)&u));
        if (u) IUnknown_Release(u);
        HR(IDirect3DVolume9_LockBox(lv, &b, &box, D3DLOCK_READONLY));
        ok(*(DWORD *)b.pBits == D3DCOLOR_ARGB(255, 1 * 85, 2 * 85, 1 * 85), "LockBox: a box starts at its corner");
        ok(IDirect3DVolume9_LockBox(lv, &b, NULL, 0) == D3DERR_INVALIDCALL, "locked twice");
        HR(IDirect3DVolume9_UnlockBox(lv));
        IDirect3DVolume9_Release(lv);
    }
    /* the fixed stages: three coordinates */
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
    vol_stage(t, D3DTEXF_POINT, D3DTADDRESS_CLAMP);
    HR(IDirect3DDevice9_GetTexture(g_d, 0, &got));
    ok(got == (IDirect3DBaseTexture9 *)t, "GetTexture");
    if (got) IDirect3DBaseTexture9_Release(got);
    for (z = 0; z < 4; z++) {
        quad_3t((z + 0.5f) / 4.0f, 1.0f);
        px(W / 8, H / 8, "volume, a slice", 0, 0, z * 85);
        px(W * 5 / 8, H * 3 / 8, "volume, a slice", 170, 85, z * 85);
    }
    vol_stage(t, D3DTEXF_LINEAR, D3DTADDRESS_CLAMP);
    quad_3t(0.5f, 1.0f);
    px(W / 8, H / 8, "volume linear", 0, 0, 128);
    vol_stage(t, D3DTEXF_POINT, D3DTADDRESS_WRAP);
    quad_3t(1.125f, 1.0f);
    px(W / 8, H / 8, "volume WRAP in w", 0, 0, 0);
    vol_stage(t, D3DTEXF_POINT, D3DTADDRESS_MIRROR);
    quad_3t(1.125f, 1.0f);
    px(W / 8, H / 8, "volume MIRROR in w", 0, 0, 255);
    vol_stage(t, D3DTEXF_POINT, D3DTADDRESS_MIRRORONCE);
    quad_3t(-0.125f, 1.0f);
    px(W / 8, H / 8, "volume MIRRORONCE in w", 0, 0, 0);
    quad_3t(-0.375f, 1.0f);
    px(W / 8, H / 8, "volume MIRRORONCE in w, slice 1", 0, 0, 85);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_BORDERCOLOR, D3DCOLOR_ARGB(255, 40, 80, 120));
    vol_stage(t, D3DTEXF_POINT, D3DTADDRESS_BORDER);
    quad_3t(1.125f, 1.0f);
    px(W / 8, H / 8, "volume BORDER in w", 40, 80, 120);
    quad_3t(0.625f, 1.0f);
    px(W / 8, H / 8, "volume BORDER in w, inside", 0, 0, 170);
    /* the matrix's coordinates, projected */
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_COUNT4 | D3DTTFF_PROJECTED);
    vol_stage(t, D3DTEXF_POINT, D3DTADDRESS_CLAMP);
    quad_3t(0.625f, 2.0f);
    px(W / 8, H / 8, "volume, projected", 0, 0, 170);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
    /* shaders */
    if (g_asm) {
        ps_scene("dcl_volume, texld", "ps_2_0\ndcl t0\ndcl_volume s0\ntexld r0, t0, s0\nmov oC0, r0\n", mid, 0, 0, 170);
        ps_scene("dcl_volume, texldp", "ps_2_0\ndcl t0\ndcl_volume s0\ntexldp r0, t0, s0\nmov oC0, r0\n", proj, 0, 0, 170);
        ps_scene("ps_1_1: tex takes what the stage holds", "ps_1_1\ntex t0\nmov r0, t0\n", mid, 0, 0, 170);
    }
    IDirect3DDevice9_SetTexture(g_d, 0, NULL);
    IDirect3DVolumeTexture9_Release(t);
    /* levels: the largest in use. 8 x 8 x 8: its tail starts on 32 bytes; 5 x 5 x 5: on 500, an image of its own */
    {
        static const int SZ[2] = { 8, 5 };
        static const DWORD C[3] = { 0xFFFF0000, 0xFF00FF00, 0xFF0000FF };
        int k, l;
        for (k = 0; k < 2; k++) {
            UINT n = SZ[k];
            t = NULL;
            HR(IDirect3DDevice9_CreateVolumeTexture(g_d, n, n, n, 3, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &t, NULL));
            if (!t) continue;
            for (l = 0; l < 3; l++) {
                int m = n >> l, i;
                HR(IDirect3DVolumeTexture9_LockBox(t, (UINT)l, &b, NULL, 0));
                for (i = 0; i < m * m * m; i++) ((DWORD *)b.pBits)[i] = C[l];
                HR(IDirect3DVolumeTexture9_UnlockBox(t, (UINT)l));
            }
            vol_stage(t, D3DTEXF_POINT, D3DTADDRESS_CLAMP);
            IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_MIPFILTER, D3DTEXF_POINT);
            quad_3t(0.5f, 1.0f);
            px(W / 2, H / 2, "volume MAXMIPLEVEL 0", 255, 0, 0);
            IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_MAXMIPLEVEL, 1);
            quad_3t(0.5f, 1.0f);
            px(W / 2, H / 2, "volume MAXMIPLEVEL 1", 0, 255, 0);
            IDirect3DVolumeTexture9_SetLOD(t, 2);
            quad_3t(0.5f, 1.0f);
            px(W / 2, H / 2, "volume SetLOD 2", 0, 0, 255);
            IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_MAXMIPLEVEL, 0);
            IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
            IDirect3DDevice9_SetTexture(g_d, 0, NULL);
            IDirect3DVolumeTexture9_Release(t);
        }
    }
    /* the formats, the system pool */
    {
        static const uint8_t dxt[16] = { 0x00, 0xF8, 0x1F, 0x00, 0, 0, 0, 0, 0xE0, 0x07, 0x1F, 0x00, 0, 0, 0, 0 };   /* red, then green: DXT1, 1 x 1 x 2 */
        s = NULL; t = NULL;
        HR(IDirect3DDevice9_CreateVolumeTexture(g_d, 2, 1, 2, 1, 0, D3DFMT_R5G6B5, D3DPOOL_SYSTEMMEM, &s, NULL));
        HR(IDirect3DDevice9_CreateVolumeTexture(g_d, 2, 1, 2, 1, 0, D3DFMT_R5G6B5, D3DPOOL_DEFAULT, &t, NULL));
        if (s && t) {
            HR(IDirect3DVolumeTexture9_LockBox(s, 0, &b, NULL, 0));
            ((WORD *)b.pBits)[0] = 0xF800; ((WORD *)b.pBits)[1] = 0x07E0;
            *(WORD *)((uint8_t *)b.pBits + b.SlicePitch) = 0x001F; *(WORD *)((uint8_t *)b.pBits + b.SlicePitch + 2) = 0xFFFF;
            HR(IDirect3DVolumeTexture9_UnlockBox(s, 0));
            HR(IDirect3DDevice9_UpdateTexture(g_d, (IDirect3DBaseTexture9 *)s, (IDirect3DBaseTexture9 *)t));
            vol_stage(t, D3DTEXF_POINT, D3DTADDRESS_CLAMP);
            quad_3t(0.75f, 1.0f);
            px(W / 4, H / 2, "volume R5G6B5, UpdateTexture", 0, 0, 255);
            px(W * 3 / 4, H / 2, "volume R5G6B5, UpdateTexture", 255, 255, 255);
        }
        IDirect3DDevice9_SetTexture(g_d, 0, NULL);
        if (s) IDirect3DVolumeTexture9_Release(s);
        if (t) IDirect3DVolumeTexture9_Release(t);
        t = NULL;
        HR(IDirect3DDevice9_CreateVolumeTexture(g_d, 4, 4, 2, 1, 0, D3DFMT_DXT1, D3DPOOL_MANAGED, &t, NULL));
        if (t) {
            HR(IDirect3DVolumeTexture9_LockBox(t, 0, &b, NULL, 0));
            ok(b.RowPitch == 8 && b.SlicePitch == 8, "a DXT1 volume's pitches");
            memcpy(b.pBits, dxt, 16);
            HR(IDirect3DVolumeTexture9_UnlockBox(t, 0));
            vol_stage(t, D3DTEXF_POINT, D3DTADDRESS_CLAMP);
            quad_3t(0.75f, 1.0f);
            px(W / 2, H / 2, "a DXT1 volume, its second slice", 0, 255, 0);
            IDirect3DDevice9_SetTexture(g_d, 0, NULL);
            IDirect3DVolumeTexture9_Release(t);
        }
    }
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_MODULATE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_ADDRESSU, D3DTADDRESS_WRAP);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_ADDRESSV, D3DTADDRESS_WRAP);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_ADDRESSW, D3DTADDRESS_WRAP);
}

typedef struct { float x, y, z, w, t[8][2]; } v8t;
#define FVF_8T (D3DFVF_XYZRHW | D3DFVF_TEX8)
#define FOURCC_INTZ ((D3DFORMAT)MAKEFOURCC('I', 'N', 'T', 'Z'))
#define FOURCC_DF24 ((D3DFORMAT)MAKEFOURCC('D', 'F', '2', '4'))

/* a stage per bit: which texel, which bit says the stage and its coordinates were each its own */
static void test_eight_stages(void)
{
    IDirect3DTexture9 *t[8];
    v8t v[4];
    int i, s;
    plain_state();
    for (s = 0; s < 8; s++) {
        DWORD c[2];
        c[0] = 0xFF000000u | (1u << (16 + s)); c[1] = 0xFF000000u | (1u << (8 + s));
        t[s] = tex_px(2, 1, D3DFMT_A8R8G8B8, c, 4);
        IDirect3DDevice9_SetTexture(g_d, (DWORD)s, (IDirect3DBaseTexture9 *)t[s]);
        stage(s, D3DTSS_COLOROP, s ? D3DTOP_ADD : D3DTOP_SELECTARG1);
        stage(s, D3DTSS_COLORARG1, D3DTA_TEXTURE);
        stage(s, D3DTSS_COLORARG2, D3DTA_CURRENT);
        stage(s, D3DTSS_ALPHAOP, s ? D3DTOP_SELECTARG2 : D3DTOP_SELECTARG1);
        stage(s, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
        stage(s, D3DTSS_ALPHAARG2, D3DTA_CURRENT);
    }
    for (i = 0; i < 4; i++) {
        v[i].x = i & 1 ? (float)W : 0.0f; v[i].y = i & 2 ? (float)H : 0.0f; v[i].z = 0.5f; v[i].w = 1.0f;
        for (s = 0; s < 8; s++) { v[i].t[s][0] = s & 1 ? 0.75f : 0.25f; v[i].t[s][1] = 0.5f; }
    }
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    IDirect3DDevice9_SetFVF(g_d, FVF_8T);
    HR(IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, v, sizeof v[0]));
    px(W / 2, H / 2, "8 texture stages", 0x55, 0xAA, 0);
    plain_state();
    for (s = 0; s < 8; s++) IDirect3DTexture9_Release(t[s]);
}

/* what depends on nothing but the caps */
static void test_true_caps(IDirect3D9 *d3d)
{
    D3DCAPS9 c;
    HR(IDirect3D9_GetDeviceCaps(d3d, 0, D3DDEVTYPE_HAL, &c));
    ok(c.MaxSimultaneousTextures == 8, "8 textures at once");
    ok((c.Caps2 & D3DCAPS2_CANAUTOGENMIPMAP) != 0, "D3DCAPS2_CANAUTOGENMIPMAP");
    ok(!(c.TextureCaps & (D3DPTEXTURECAPS_POW2 | D3DPTEXTURECAPS_NONPOW2CONDITIONAL)), "any size is a texture size");
    ok(c.VS20Caps.NumTemps == 32 && c.PS20Caps.NumTemps == 32, "32 temporaries");
    ok(c.PS20Caps.NumInstructionSlots == 512, "512 pixel shader slots");
    ok(IDirect3D9_CheckDeviceFormat(d3d, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, D3DUSAGE_AUTOGENMIPMAP, D3DRTYPE_TEXTURE, D3DFMT_A8R8G8B8) == D3D_OK, "autogen of A8R8G8B8");
    ok(IDirect3D9_CheckDeviceFormat(d3d, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, D3DUSAGE_DEPTHSTENCIL, D3DRTYPE_TEXTURE, FOURCC_INTZ) == D3D_OK, "INTZ is a depth texture");
    ok(IDirect3D9_CheckDeviceFormat(d3d, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, D3DUSAGE_DEPTHSTENCIL, D3DRTYPE_TEXTURE, D3DFMT_D24S8) == D3D_OK, "D24S8 is a depth texture");
}

/* the levels from level 0: half red, half blue comes to their mean at the small end */
static void test_autogen(void)
{
    IDirect3DTexture9 *t = NULL;
    D3DLOCKED_RECT l;
    int x, y;
    plain_state();
    HR(IDirect3DDevice9_CreateTexture(g_d, 4, 4, 0, D3DUSAGE_AUTOGENMIPMAP, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &t, NULL));
    if (!t) return;
    ok(IDirect3DTexture9_GetLevelCount(t) == 1, "an autogen texture shows one level");
    IDirect3DTexture9_LockRect(t, 0, &l, NULL, 0);
    for (y = 0; y < 4; y++) for (x = 0; x < 4; x++) ((DWORD *)((uint8_t *)l.pBits + y * l.Pitch))[x] = x < 2 ? 0xFFFF0000 : 0xFF0000FF;
    IDirect3DTexture9_UnlockRect(t, 0);
    IDirect3DTexture9_SetAutoGenFilterType(t, D3DTEXF_LINEAR);
    IDirect3DTexture9_GenerateMipSubLevels(t);
    ok(IDirect3DTexture9_GetAutoGenFilterType(t) == D3DTEXF_LINEAR, "the autogen filter");
    stage_tex(0, (IDirect3DBaseTexture9 *)t);
    samp(0, D3DSAMP_MIPFILTER, D3DTEXF_POINT);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    quad_uv(100, 100, 101, 101, 0, 0, 1, 1);
    px(100, 100, "autogen: the last level is the mean", 128, 0, 128);
    samp(0, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
    plain_state();
    IDirect3DTexture9_Release(t);
}

/* a level of a texture as the target */
static void test_target_level(void)
{
    IDirect3DTexture9 *t = NULL;
    IDirect3DSurface9 *s0 = NULL, *s1 = NULL, *back = NULL, *ds = NULL;
    plain_state();
    IDirect3DDevice9_GetRenderTarget(g_d, 0, &back);
    IDirect3DDevice9_GetDepthStencilSurface(g_d, &ds);
    HR(IDirect3DDevice9_CreateTexture(g_d, 16, 16, 2, D3DUSAGE_RENDERTARGET, D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &t, NULL));
    if (!t) return;
    IDirect3DTexture9_GetSurfaceLevel(t, 0, &s0);
    IDirect3DTexture9_GetSurfaceLevel(t, 1, &s1);
    IDirect3DDevice9_SetDepthStencilSurface(g_d, NULL);
    HR(IDirect3DDevice9_SetRenderTarget(g_d, 0, s0));
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(255, 0, 0), 1.0f, 0);
    HR(IDirect3DDevice9_SetRenderTarget(g_d, 0, s1));
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, D3DCOLOR_XRGB(0, 255, 0), 1.0f, 0);
    HR(IDirect3DDevice9_SetRenderTarget(g_d, 0, back));
    IDirect3DDevice9_SetDepthStencilSurface(g_d, ds);
    stage_tex(0, (IDirect3DBaseTexture9 *)t);
    samp(0, D3DSAMP_MIPFILTER, D3DTEXF_POINT);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0, 1.0f, 0);
    quad_uv(0, 0, 16, 16, 0, 0, 1, 1);
    quad_uv(100, 100, 108, 108, 0, 0, 1, 1);
    px(8, 8, "level 0 as a target", 255, 0, 0);
    px(104, 104, "level 1 as a target", 0, 255, 0);
    samp(0, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
    plain_state();
    IDirect3DSurface9_Release(s0); IDirect3DSurface9_Release(s1);
    IDirect3DSurface9_Release(back);
    if (ds) IDirect3DSurface9_Release(ds);
    IDirect3DTexture9_Release(t);
}

/* depth rendered through a texture, for what is sampled from it: the value (INTZ) or the comparison (D24S8) */
static IDirect3DTexture9 *depth_scene(D3DFORMAT f)
{
    IDirect3DTexture9 *t = NULL;
    IDirect3DSurface9 *s = NULL, *c = NULL, *back = NULL, *ds = NULL;
    HR(IDirect3DDevice9_CreateTexture(g_d, 64, 64, 1, D3DUSAGE_DEPTHSTENCIL, f, D3DPOOL_DEFAULT, &t, NULL));
    if (!t) return NULL;
    IDirect3DTexture9_GetSurfaceLevel(t, 0, &s);
    HR(IDirect3DDevice9_CreateRenderTarget(g_d, 64, 64, D3DFMT_A8R8G8B8, D3DMULTISAMPLE_NONE, 0, FALSE, &c, NULL));
    IDirect3DDevice9_GetRenderTarget(g_d, 0, &back);
    IDirect3DDevice9_GetDepthStencilSurface(g_d, &ds);
    HR(IDirect3DDevice9_SetRenderTarget(g_d, 0, c));
    HR(IDirect3DDevice9_SetDepthStencilSurface(g_d, s));
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0, 1.0f, 0);
    rs(D3DRS_ZENABLE, TRUE); rs(D3DRS_ZWRITEENABLE, TRUE); rs(D3DRS_ZFUNC, D3DCMP_LESSEQUAL);
    quad(0, 0, 32, 64, 0.5f, 0xFFFFFFFF);
    rs(D3DRS_ZENABLE, FALSE);
    IDirect3DDevice9_SetRenderTarget(g_d, 0, back);
    IDirect3DDevice9_SetDepthStencilSurface(g_d, ds);
    IDirect3DSurface9_Release(s); IDirect3DSurface9_Release(c); IDirect3DSurface9_Release(back);
    if (ds) IDirect3DSurface9_Release(ds);
    return t;
}

static void test_depth_tex(void)
{
    static const float near_[4][4] = { { 0.25f, 0.5f, 0.25f, 1 }, { 0.25f, 0.5f, 0.25f, 1 }, { 0.25f, 0.5f, 0.25f, 1 }, { 0.25f, 0.5f, 0.25f, 1 } };
    static const float far_[4][4] = { { 0.25f, 0.5f, 0.75f, 1 }, { 0.25f, 0.5f, 0.75f, 1 }, { 0.25f, 0.5f, 0.75f, 1 }, { 0.25f, 0.5f, 0.75f, 1 } };
    static const char PS[] = "ps_2_0\ndcl t0\ndcl_2d s0\ntexld r0, t0, s0\nmov oC0, r0\n";
    IDirect3DTexture9 *t;
    plain_state();
    t = depth_scene(FOURCC_INTZ);
    if (t) {
        stage_tex(0, (IDirect3DBaseTexture9 *)t);
        quad_uv(0, 0, W, H, 0, 0, 1, 1);
        px(W / 4, H / 2, "INTZ: the depth drawn", 128, 128, 128);
        px(3 * W / 4, H / 2, "INTZ: the depth cleared", 255, 255, 255);
        plain_state();
        IDirect3DTexture9_Release(t);
    }
    t = depth_scene(D3DFMT_D24S8);
    if (t) {
        IDirect3DDevice9_SetTexture(g_d, 0, (IDirect3DBaseTexture9 *)t);
        ps_scene("D24S8: nearer than the depth drawn", PS, near_, 255, 255, 255);
        ps_scene("D24S8: farther than the depth drawn", PS, far_, 0, 0, 0);
        plain_state();
        IDirect3DTexture9_Release(t);
    }
}

/* after a Present: what the window shows */
static void test_front_data(void)
{
    IDirect3DSurface9 *s = NULL;
    D3DDISPLAYMODE m;
    D3DLOCKED_RECT l;
    HRESULT h;
    IDirect3DDevice9_BeginScene(g_d);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, D3DCOLOR_XRGB(10, 200, 30), 1.0f, 0);
    IDirect3DDevice9_EndScene(g_d);
    HR(IDirect3DDevice9_Present(g_d, NULL, NULL, NULL, NULL));
    HR(IDirect3DDevice9_GetDisplayMode(g_d, 0, &m));
    HR(IDirect3DDevice9_CreateOffscreenPlainSurface(g_d, m.Width, m.Height, D3DFMT_A8R8G8B8, D3DPOOL_SYSTEMMEM, &s, NULL));
    if (!s) return;
    h = IDirect3DDevice9_GetFrontBufferData(g_d, 0, s);
    ok(h == D3D_OK, "GetFrontBufferData");
    if (SUCCEEDED(h)) {
        const uint8_t *p;
        IDirect3DSurface9_LockRect(s, &l, NULL, D3DLOCK_READONLY);
        p = (const uint8_t *)l.pBits + (ptrdiff_t)20 * l.Pitch + 20 * 4;
        g_checks++;
        if (abs(p[2] - 10) > 2 || abs(p[1] - 200) > 2 || abs(p[0] - 30) > 2) { g_bad++; printf("FAIL GetFrontBufferData: %d %d %d\n", p[2], p[1], p[0]); }
        IDirect3DSurface9_UnlockRect(s);
    }
    IDirect3DSurface9_Release(s);
}

int main(void)
{
    WNDCLASSA wc;
    RECT r = { 0, 0, W, H };
    HWND wnd;
    IDirect3D9 *d3d;
    D3DPRESENT_PARAMETERS pp;
    D3DCAPS9 c;
    HMODULE x;
    setvbuf(stdout, NULL, _IONBF, 0);
    memset(&wc, 0, sizeof wc);
    wc.lpfnWndProc = wnd_proc; wc.hInstance = GetModuleHandleA(NULL); wc.lpszClassName = "d3d_test";
    RegisterClassA(&wc);
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    wnd = CreateWindowA("d3d_test", "d3d_test", WS_OVERLAPPEDWINDOW, 40, 40, r.right - r.left, r.bottom - r.top, NULL, NULL, wc.hInstance, NULL);
    ShowWindow(wnd, SW_SHOWNOACTIVATE);
    x = LoadLibraryA("d3dx9_43.dll");
    if (x) g_asm = (assemble_t)GetProcAddress(x, "D3DXAssembleShader");
    d3d = Direct3DCreate9(D3D_SDK_VERSION);
    ok(d3d != NULL, "Direct3DCreate9");
    ok(IDirect3D9_CheckDeviceFormat(d3d, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, 0, D3DRTYPE_TEXTURE, D3DFMT_DXT5) == D3D_OK, "DXT5 is a format");
    ok(IDirect3D9_CheckDeviceFormat(d3d, 0, D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, 0, D3DRTYPE_TEXTURE, D3DFMT_A16B16G16R16F) != D3D_OK, "float textures are not");
    HR(IDirect3D9_GetDeviceCaps(d3d, 0, D3DDEVTYPE_HAL, &c));
    ok(c.VertexShaderVersion >= D3DVS_VERSION(2, 0) && c.MaxTextureBlendStages == 8, "caps");
    memset(&pp, 0, sizeof pp);
    pp.Windowed = TRUE; pp.SwapEffect = D3DSWAPEFFECT_DISCARD; pp.BackBufferFormat = D3DFMT_X8R8G8B8; pp.EnableAutoDepthStencil = TRUE; pp.AutoDepthStencilFormat = D3DFMT_D24S8;
    pp.BackBufferWidth = W; pp.BackBufferHeight = H; pp.hDeviceWindow = wnd;
    HR(IDirect3D9_CreateDevice(d3d, 0, D3DDEVTYPE_HAL, wnd, D3DCREATE_HARDWARE_VERTEXPROCESSING, &pp, &g_d));
    if (!g_d) return 1;
    HR(IDirect3DDevice9_CreateOffscreenPlainSurface(g_d, W, H, D3DFMT_X8R8G8B8, D3DPOOL_SYSTEMMEM, &g_sys, NULL));
    IDirect3DDevice9_BeginScene(g_d);
    test_clear();
    test_tl();
    test_formats();
    test_stages();
    test_ops();
    test_3d();
    test_light();
    {   /* a surface of the size of the render target for reads above and below: the target test has its own size */
        test_target();
    }
    test_shaders();
    test_ps1();
    test_flow();
    test_ff_extra();
    test_shade_fill();
    test_instancing();
    test_device_extra();
    test_sampler_extra(d3d);
    test_volume(d3d);
    test_true_caps(d3d);
    test_eight_stages();
    test_autogen();
    test_target_level();
    test_depth_tex();
    IDirect3DDevice9_EndScene(g_d);
    test_present();
    test_front_data();
    test_kept();
    test_kept_unseen();
    printf("%d checks, %d failed\n", g_checks, g_bad);
    if (!g_bad) printf("OK d3d_test\n");
    return g_bad ? 1 : 0;
}
