/* d3d_tex_cow_test — textures changed in the middle of a scene, after draws that sampled them (dynamic lightmaps, video frames, a font cache):
 * every draw must show the texels its texture held when it was drawn. The library keeps them by copying the texture on board B (copy-on-write)
 * instead of ending the render pass; the result is the same either way, only the cost differs, which `perf` measures.
 *   d3d_tex_cow_test [perf]
 * (Not named "...update...": Windows treats such a program as an installer and asks for elevation.)
 * A plain Direct3D 9 program: beside our d3d9.dll it runs on the boards, anywhere else on the PC's adapter (the reference). */
#define CINTERFACE
#define COBJMACROS
#include <windows.h>
#include <d3d9.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 256
#define H 128

static IDirect3DDevice9 *g_d;
static IDirect3DSurface9 *g_sys;
static uint32_t g_px[W * H];            /* the last frame read back, 0xRRGGBB */
static int g_checks, g_bad;

typedef struct { float x, y, z, rhw, u, v; } tv;
#define FVF (D3DFVF_XYZRHW | D3DFVF_TEX1)

static void ok(int c, const char *what) { g_checks++; if (!c) { g_bad++; printf("FAIL %s\n", what); } }

/* a quad in pixels [x0, x1) x [y0, y1) (y down) with the whole texture on it */
static void quad(IDirect3DBaseTexture9 *t, int x0, int y0, int x1, int y1)
{
    tv v[4] = { { x0 - 0.5f, y0 - 0.5f, 0.5f, 1.0f, 0, 0 }, { x1 - 0.5f, y0 - 0.5f, 0.5f, 1.0f, 1, 0 },
                { x0 - 0.5f, y1 - 0.5f, 0.5f, 1.0f, 0, 1 }, { x1 - 0.5f, y1 - 0.5f, 0.5f, 1.0f, 1, 1 } };
    IDirect3DDevice9_SetTexture(g_d, 0, t);
    IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLESTRIP, 2, v, sizeof v[0]);
}

static void begin(void)
{
    IDirect3DDevice9_BeginScene(g_d);
    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0xFF000000, 1.0f, 0);
}

/* the scene ends and the target comes back */
static void end(void)
{
    IDirect3DSurface9 *rt = NULL;
    D3DLOCKED_RECT l;
    int y, x;
    IDirect3DDevice9_EndScene(g_d);
    IDirect3DDevice9_GetRenderTarget(g_d, 0, &rt);
    if (FAILED(IDirect3DDevice9_GetRenderTargetData(g_d, rt, g_sys))) { printf("GetRenderTargetData failed\n"); exit(1); }
    IDirect3DSurface9_LockRect(g_sys, &l, NULL, D3DLOCK_READONLY);
    for (y = 0; y < H; y++) for (x = 0; x < W; x++) g_px[y * W + x] = ((const uint32_t *)((const uint8_t *)l.pBits + (ptrdiff_t)y * l.Pitch))[x] & 0xFFFFFF;
    IDirect3DSurface9_UnlockRect(g_sys);
    IDirect3DSurface9_Release(rt);
    IDirect3DDevice9_Present(g_d, NULL, NULL, NULL, NULL);
}

static void expect(int x, int y, uint32_t want, const char *what)
{
    uint32_t got = g_px[y * W + x];
    char b[160];
    int d = 0, i;
    want &= 0xFFFFFF;
    for (i = 0; i < 3; i++) { int a = (int)(got >> (8 * i) & 255), e = (int)(want >> (8 * i) & 255); if (abs(a - e) > d) d = abs(a - e); }
    snprintf(b, sizeof b, "%s: pixel (%d, %d) = %06x, expected %06x", what, x, y, got, want);
    ok(d <= 2, b);
}

/* level `level` of t, or the rectangle r of it, set to one colour (0xAARRGGBB) */
static void fill_level(IDirect3DTexture9 *t, UINT level, const RECT *r, DWORD flags, uint32_t argb)
{
    D3DSURFACE_DESC sd;
    D3DLOCKED_RECT l;
    int x, y, w, h;
    IDirect3DTexture9_GetLevelDesc(t, level, &sd);
    if (FAILED(IDirect3DTexture9_LockRect(t, level, &l, r, flags))) { ok(0, "LockRect"); return; }
    w = r ? r->right - r->left : (int)sd.Width; h = r ? r->bottom - r->top : (int)sd.Height;
    for (y = 0; y < h; y++) for (x = 0; x < w; x++) ((uint32_t *)((uint8_t *)l.pBits + (ptrdiff_t)y * l.Pitch))[x] = argb;
    IDirect3DTexture9_UnlockRect(t, level);
}

static IDirect3DTexture9 *tex_of(UINT w, UINT h, UINT levels, DWORD usage, D3DPOOL pool, uint32_t argb)
{
    IDirect3DTexture9 *t = NULL;
    UINT l;
    if (FAILED(IDirect3DDevice9_CreateTexture(g_d, w, h, levels, usage, D3DFMT_A8R8G8B8, pool, &t, NULL)) || !t) { printf("CreateTexture failed\n"); exit(1); }
    if (pool != D3DPOOL_DEFAULT || (usage & D3DUSAGE_DYNAMIC))
        for (l = 0; l < IDirect3DTexture9_GetLevelCount(t); l++) fill_level(t, l, NULL, !l && (usage & D3DUSAGE_DYNAMIC) ? D3DLOCK_DISCARD : 0, argb);   /* DISCARD: level 0 only */
    return t;
}

static void tests(void)
{
    IDirect3DTexture9 *a, *b, *m, *md, *dy, *sys, *dst;
    RECT corner = { 0, 0, 8, 8 }, mid = { 4, 4, 12, 12 };
    int i;
    const uint32_t RED = 0xFFFF0000, GREEN = 0xFF00FF00, BLUE = 0xFF0000FF, WHITE = 0xFFFFFFFF, YELLOW = 0xFFFFFF00;
    /* 1: the whole texture replaced between two draws */
    a = tex_of(16, 16, 1, 0, D3DPOOL_MANAGED, RED);
    begin();
    quad((IDirect3DBaseTexture9 *)a, 0, 0, 64, 64);
    fill_level(a, 0, NULL, 0, GREEN);
    quad((IDirect3DBaseTexture9 *)a, 64, 0, 128, 64);
    end();
    expect(32, 32, RED, "whole update: the draw before it");
    expect(96, 32, GREEN, "whole update: the draw after it");
    /* 2: a part of it: the rest stays */
    b = tex_of(16, 16, 1, 0, D3DPOOL_MANAGED, RED);
    begin();
    quad((IDirect3DBaseTexture9 *)b, 0, 0, 64, 64);
    fill_level(b, 0, &corner, 0, BLUE);                         /* texels (0..7, 0..7): the quad's top-left quarter */
    quad((IDirect3DBaseTexture9 *)b, 64, 0, 128, 64);
    end();
    expect(16, 16, RED, "part: the draw before it, the changed corner");
    expect(80, 16, BLUE, "part: the draw after it, the changed corner");
    expect(112, 48, RED, "part: the draw after it, the rest");
    /* 3: one texture changed ten times in one scene, a draw after each change */
    begin();
    for (i = 0; i < 10; i++) {
        fill_level(a, 0, NULL, 0, 0xFF000000u | 0x100000u * (uint32_t)(i + 1) | 0x8060u);
        quad((IDirect3DBaseTexture9 *)a, i * 24, 64, i * 24 + 24, 128);
    }
    end();
    for (i = 0; i < 10; i++) {
        char w[64];
        snprintf(w, sizeof w, "ten updates: draw %d", i);
        expect(i * 24 + 12, 96, 0xFF000000u | 0x100000u * (uint32_t)(i + 1) | 0x8060u, w);
    }
    /* 4: a mipmapped texture: levels 0 and 1 changed apart, each draw at the level it samples. Then level 1 alone, in a dynamic texture: level 0
       must survive the copy. (A managed texture takes its dirty region from level 0 only: there a lock of level 1 alone need not reach the GPU.) */
    m = tex_of(32, 32, 0, 0, D3DPOOL_MANAGED, WHITE);
    md = tex_of(32, 32, 0, D3DUSAGE_DYNAMIC, D3DPOOL_DEFAULT, WHITE);
    fill_level(md, 0, NULL, 0, RED);
    fill_level(md, 1, NULL, 0, GREEN);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_MIPFILTER, D3DTEXF_POINT);
    begin();
    quad((IDirect3DBaseTexture9 *)m, 0, 0, 32, 32);             /* 32 texels on 32 pixels: level 0 */
    quad((IDirect3DBaseTexture9 *)m, 32, 0, 48, 16);            /* 32 on 16: level 1 */
    fill_level(m, 0, NULL, 0, RED);
    fill_level(m, 1, NULL, 0, GREEN);
    quad((IDirect3DBaseTexture9 *)m, 64, 0, 96, 32);
    quad((IDirect3DBaseTexture9 *)m, 96, 0, 112, 16);
    quad((IDirect3DBaseTexture9 *)md, 0, 64, 32, 96);
    quad((IDirect3DBaseTexture9 *)md, 32, 64, 48, 80);
    fill_level(md, 1, NULL, 0, BLUE);
    quad((IDirect3DBaseTexture9 *)md, 64, 64, 96, 96);
    quad((IDirect3DBaseTexture9 *)md, 96, 64, 112, 80);
    end();
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
    expect(16, 16, WHITE, "mipmaps: level 0 before");
    expect(40, 8, WHITE, "mipmaps: level 1 before");
    expect(80, 16, RED, "mipmaps: level 0 after");
    expect(104, 8, GREEN, "mipmaps: level 1 after");
    expect(16, 80, RED, "dynamic mipmaps: level 0 before");
    expect(40, 72, GREEN, "dynamic mipmaps: level 1 before");
    expect(80, 80, RED, "dynamic mipmaps: level 0 kept when level 1 alone changes");
    expect(104, 72, BLUE, "dynamic mipmaps: level 1 changed alone");
    /* 5: a dynamic texture rewritten with D3DLOCK_DISCARD, as video or a font cache */
    dy = tex_of(16, 16, 1, D3DUSAGE_DYNAMIC, D3DPOOL_DEFAULT, YELLOW);
    begin();
    quad((IDirect3DBaseTexture9 *)dy, 0, 0, 32, 32);
    fill_level(dy, 0, NULL, D3DLOCK_DISCARD, BLUE);
    quad((IDirect3DBaseTexture9 *)dy, 32, 0, 64, 32);
    fill_level(dy, 0, &mid, 0, RED);                             /* no DISCARD: a part, the rest kept */
    quad((IDirect3DBaseTexture9 *)dy, 64, 0, 96, 32);
    end();
    expect(16, 16, YELLOW, "dynamic: the draw before DISCARD");
    expect(48, 16, BLUE, "dynamic: the draw after DISCARD");
    expect(80, 16, RED, "dynamic: a part changed, its middle");
    expect(66, 2, BLUE, "dynamic: a part changed, the rest");
    /* 6: UpdateTexture from system memory into a default-pool texture, between draws */
    sys = tex_of(16, 16, 1, 0, D3DPOOL_SYSTEMMEM, RED);
    dst = tex_of(16, 16, 1, 0, D3DPOOL_DEFAULT, 0);
    IDirect3DDevice9_UpdateTexture(g_d, (IDirect3DBaseTexture9 *)sys, (IDirect3DBaseTexture9 *)dst);
    begin();
    quad((IDirect3DBaseTexture9 *)dst, 0, 0, 32, 32);
    fill_level(sys, 0, NULL, 0, GREEN);
    IDirect3DDevice9_UpdateTexture(g_d, (IDirect3DBaseTexture9 *)sys, (IDirect3DBaseTexture9 *)dst);
    quad((IDirect3DBaseTexture9 *)dst, 32, 0, 64, 32);
    end();
    expect(16, 16, RED, "UpdateTexture: the draw before it");
    expect(48, 16, GREEN, "UpdateTexture: the draw after it");
    /* 7: a texture released after a draw that sampled it, in the same scene */
    begin();
    quad((IDirect3DBaseTexture9 *)b, 0, 0, 64, 64);
    IDirect3DDevice9_SetTexture(g_d, 0, NULL);
    IDirect3DTexture9_Release(b);
    end();
    expect(48, 48, RED, "released after its draw");
    IDirect3DTexture9_Release(a);
    IDirect3DTexture9_Release(m);
    IDirect3DTexture9_Release(md);
    IDirect3DTexture9_Release(dy);
    IDirect3DTexture9_Release(sys);
    IDirect3DTexture9_Release(dst);
}

/* frames of 16 textures, each drawn, then changed, then drawn again before the next one: what dynamic lights do to a game's lightmaps.
   Without copy-on-write every change ends the pass: 17 passes a frame */
static void perf(void)
{
    IDirect3DTexture9 *t[16];
    RECT r = { 32, 32, 96, 96 };
    int i, f, n = 240;
    LARGE_INTEGER q0, q1, fr;
    for (i = 0; i < 16; i++) t[i] = tex_of(128, 128, 1, 0, D3DPOOL_MANAGED, 0xFF808080);
    QueryPerformanceFrequency(&fr);
    QueryPerformanceCounter(&q0);
    for (f = 0; f < n; f++) {
        IDirect3DDevice9_BeginScene(g_d);
        IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET, 0xFF000000, 1.0f, 0);
        for (i = 0; i < 16; i++) {
            quad((IDirect3DBaseTexture9 *)t[i], (i % 8) * 32, (i / 8) * 64, (i % 8) * 32 + 32, (i / 8) * 64 + 32);
            fill_level(t[i], 0, &r, 0, 0xFF000000u | 0x101010u * (uint32_t)(f % 15));
            quad((IDirect3DBaseTexture9 *)t[i], (i % 8) * 32, (i / 8) * 64 + 32, (i % 8) * 32 + 32, (i / 8) * 64 + 64);
        }
        IDirect3DDevice9_EndScene(g_d);
        IDirect3DDevice9_Present(g_d, NULL, NULL, NULL, NULL);
    }
    begin();
    end();                                                      /* waits for the last frame */
    QueryPerformanceCounter(&q1);
    printf("perf: %d frames of 16 textures updated after use: %.2f ms a frame\n", n, 1000.0 * (double)(q1.QuadPart - q0.QuadPart) / (double)fr.QuadPart / n);
    for (i = 0; i < 16; i++) IDirect3DTexture9_Release(t[i]);
}

int main(int argc, char **argv)
{
    WNDCLASSA wc;
    RECT r = { 0, 0, W, H };
    HWND wnd;
    IDirect3D9 *d3d;
    D3DPRESENT_PARAMETERS pp;
    D3DADAPTER_IDENTIFIER9 ai;
    int do_perf = argc > 1 && !strcmp(argv[1], "perf");
    setvbuf(stdout, NULL, _IONBF, 0);
    memset(&wc, 0, sizeof wc);
    wc.lpfnWndProc = DefWindowProcA; wc.hInstance = GetModuleHandleA(NULL); wc.lpszClassName = "d3d_tex_cow_test";
    RegisterClassA(&wc);
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    wnd = CreateWindowA("d3d_tex_cow_test", "d3d_tex_cow_test", WS_OVERLAPPEDWINDOW, 40, 40, r.right - r.left, r.bottom - r.top, NULL, NULL, wc.hInstance, NULL);
    ShowWindow(wnd, SW_SHOWNOACTIVATE);
    d3d = Direct3DCreate9(D3D_SDK_VERSION);
    if (!d3d) { printf("no Direct3D 9\n"); return 1; }
    memset(&ai, 0, sizeof ai);
    IDirect3D9_GetAdapterIdentifier(d3d, 0, 0, &ai);
    memset(&pp, 0, sizeof pp);
    pp.Windowed = TRUE; pp.SwapEffect = D3DSWAPEFFECT_DISCARD; pp.BackBufferFormat = D3DFMT_X8R8G8B8;
    pp.BackBufferWidth = W; pp.BackBufferHeight = H; pp.hDeviceWindow = wnd; pp.PresentationInterval = D3DPRESENT_INTERVAL_IMMEDIATE;
    if (FAILED(IDirect3D9_CreateDevice(d3d, 0, D3DDEVTYPE_HAL, wnd, D3DCREATE_HARDWARE_VERTEXPROCESSING, &pp, &g_d)) || !g_d) { printf("no device\n"); return 1; }
    IDirect3DDevice9_CreateOffscreenPlainSurface(g_d, W, H, D3DFMT_X8R8G8B8, D3DPOOL_SYSTEMMEM, &g_sys, NULL);
    printf("adapter: %s\n", ai.Description);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_LIGHTING, FALSE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_CULLMODE, D3DCULL_NONE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ZENABLE, FALSE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_MINFILTER, D3DTEXF_POINT);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_MAGFILTER, D3DTEXF_POINT);
    IDirect3DDevice9_SetSamplerState(g_d, 0, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
    IDirect3DDevice9_SetFVF(g_d, FVF);
    tests();
    if (do_perf) perf();
    printf("%d checks, %d failed\n", g_checks, g_bad);
    if (!g_bad) printf("OK d3d_tex_cow_test\n");
    IDirect3DSurface9_Release(g_sys);
    IDirect3DDevice9_Release(g_d);
    IDirect3D9_Release(d3d);
    return g_bad ? 1 : 0;
}
