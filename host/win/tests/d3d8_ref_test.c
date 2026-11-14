/* d3d8_ref_test — what Release counts, against fake_board (no boards): the application's references alone, the device's own apart, as
 * Direct3D's runtime counts them. A game's Direct3D 8 renderer may take the back buffer and the depth surface after CreateDevice and
 * release them at once, then before Reset release them and its bound 2.6 MB vertex buffer again until Release says 0.
 * If the device's references counted too, that would free the surfaces under the device and Reset would crash on the depth surface. This plays
 * that sequence, then checks the counts a game can see for surfaces, textures and buffers.
 * The libraries read GPU2_IP with msvcrt's getenv, whose copy of the environment SetEnvironmentVariable does not change: set that way, the
 * test would talk to the real board A. So _putenv, a getenv check before anything connects, d3d8.dll
 * loaded only then, and a device fake_board never heard from stops the test at once. */
#define CINTERFACE
#define COBJMACROS
#include <windows.h>
#include <d3d8.h>
#include <stdio.h>
#include <string.h>
#include "fake_board.h"

#define PORT 5603

static int g_bad, g_checks;
static void ok(int c, const char *what) { g_checks++; if (!c) { g_bad++; printf("FAIL %s\n", what); } }
#define EQ(x, want) do { long v_ = (long)(x); g_checks++; if (v_ != (long)(want)) { g_bad++; printf("FAIL %s = %ld, want %ld\n", #x, v_, (long)(want)); } } while (0)
#define HR(x) do { HRESULT h_ = (x); g_checks++; if (FAILED(h_)) { g_bad++; printf("FAIL %s = %08lx\n", #x, (unsigned long)h_); } } while (0)

/* the driver's loop: Release until it says 0. Returns how many said more */
static int release_all(void *o)
{
    IUnknown *u = (IUnknown *)o;
    int n = 0;
    while (u->lpVtbl->Release(u) && n < 100) n++;
    return n;
}

int main(void)
{
    fake_board *f = fake_start(PORT, 0.0, 0.0, 7);
    fake_stats fs;
    IDirect3D8 *(WINAPI *create8)(UINT);
    HMODULE lib;
    char path[MAX_PATH], *slash, env[64];
    const char *ip, *pt;
    IDirect3D8 *o;
    IDirect3DDevice8 *d = NULL;
    D3DPRESENT_PARAMETERS pp;
    IDirect3DSurface8 *bb = NULL, *ds = NULL, *bb2 = NULL, *ds2 = NULL;
    IDirect3DVertexBuffer8 *vb = NULL, *vb2 = NULL;
    IDirect3DTexture8 *t = NULL;
    IDirect3DBaseTexture8 *got = NULL;
    D3DSURFACE_DESC de;
    UINT stride = 0;
    HWND w;
    char port[16];
    if (!f) { printf("FAIL fake_board on port %d\n", PORT); return 1; }
    sprintf(port, "%d", PORT);
    _putenv("GPU2_IP=127.0.0.1");
    sprintf(env, "GPU2_PORT=%d", PORT);
    _putenv(env);
    _putenv("GPU2_MIN_A=0");                                    /* fake_board's build number is below the default minimum */
    ip = getenv("GPU2_IP"); pt = getenv("GPU2_PORT");
    if (!ip || strcmp(ip, "127.0.0.1") || !pt || strcmp(pt, port)) { printf("FAIL getenv does not see fake_board's address: nothing is sent\n"); return 1; }
    if (GetModuleHandleA("d3d8.dll")) { printf("FAIL d3d8.dll was loaded before the environment was set: it would talk to the real board\n"); return 1; }
    GetModuleFileNameA(NULL, path, sizeof path);
    slash = strrchr(path, '\\');
    if (!slash || (size_t)(slash - path) + 10 >= sizeof path) { printf("FAIL path\n"); return 1; }
    strcpy(slash + 1, "d3d8.dll");                              /* ours, beside this .exe: never the system's */
    lib = LoadLibraryA(path);
    create8 = lib ? (IDirect3D8 *(WINAPI *)(UINT))(void *)GetProcAddress(lib, "Direct3DCreate8") : NULL;
    if (!create8) { printf("FAIL no Direct3DCreate8 in %s\n", path); return 1; }
    w = CreateWindowA("STATIC", "d3d8_ref_test", WS_POPUP, 0, 0, 640, 480, NULL, NULL, NULL, NULL);
    o = create8(D3D_SDK_VERSION);
    memset(&pp, 0, sizeof pp);
    pp.BackBufferWidth = 640; pp.BackBufferHeight = 480; pp.BackBufferFormat = D3DFMT_X8R8G8B8; pp.BackBufferCount = 1;
    pp.SwapEffect = D3DSWAPEFFECT_FLIP; pp.hDeviceWindow = w; pp.Windowed = TRUE;
    pp.EnableAutoDepthStencil = TRUE; pp.AutoDepthStencilFormat = D3DFMT_D16;
    if (!o || FAILED(IDirect3D8_CreateDevice(o, 0, D3DDEVTYPE_HAL, w, D3DCREATE_SOFTWARE_VERTEXPROCESSING | D3DCREATE_MULTITHREADED, &pp, &d))) { printf("FAIL CreateDevice\n"); return 1; }
    fake_get_stats(f, &fs);
    if (!fs.accepted) { printf("FAIL the device did not talk to fake_board: stopped before anything more reaches a board\n"); ExitProcess(1); }

    /* after CreateDevice: the pointers kept, the references given back at once. The device's own are not the application's: 0 */
    HR(IDirect3DDevice8_GetBackBuffer(d, 0, D3DBACKBUFFER_TYPE_MONO, &bb));
    EQ(IDirect3DSurface8_Release(bb), 0);
    HR(IDirect3DDevice8_GetDepthStencilSurface(d, &ds));
    EQ(IDirect3DSurface8_Release(ds), 0);
    HR(IDirect3DDevice8_CreateVertexBuffer(d, 0x27ffd8, D3DUSAGE_WRITEONLY | D3DUSAGE_DYNAMIC, 0, D3DPOOL_DEFAULT, &vb));
    HR(IDirect3DDevice8_SetStreamSource(d, 0, vb, 32));
    /* before Reset: until Release says 0. The surfaces have no reference of the application's left, the buffer one */
    EQ(release_all(bb), 0);
    EQ(release_all(ds), 0);
    EQ(release_all(vb), 0);
    HR(IDirect3DDevice8_GetStreamSource(d, 0, &vb2, &stride));  /* still bound, alive */
    ok(vb2 == vb, "the released vertex buffer is still the stream's");
    EQ(stride, 32);
    if (vb2) EQ(IDirect3DVertexBuffer8_Release(vb2), 0);
    HR(IDirect3DDevice8_Reset(d, &pp));                         /* crashes here if the depth surface was freed */

    /* after Reset: the same back buffer, a new depth surface, counted from 0 again, and one Release too many changes nothing */
    HR(IDirect3DDevice8_GetBackBuffer(d, 0, D3DBACKBUFFER_TYPE_MONO, &bb2));
    ok(bb2 == bb, "the back buffer is the same object after Reset");
    EQ(IDirect3DSurface8_AddRef(bb2), 2);
    EQ(IDirect3DSurface8_Release(bb2), 1);
    EQ(IDirect3DSurface8_Release(bb2), 0);
    EQ(IDirect3DSurface8_Release(bb2), 0);
    HR(IDirect3DDevice8_GetDepthStencilSurface(d, &ds2));
    if (ds2) {
        HR(IDirect3DSurface8_GetDesc(ds2, &de));
        EQ(de.Format, D3DFMT_D16);
        EQ(de.Width, 640);
        EQ(IDirect3DSurface8_Release(ds2), 0);
    }

    /* a bound texture: the binding is the device's. The application's last Release says 0, the texture stays until it is unbound */
    HR(IDirect3DDevice8_CreateTexture(d, 64, 64, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &t));
    EQ(IDirect3DTexture8_AddRef(t), 2);
    EQ(IDirect3DTexture8_Release(t), 1);
    HR(IDirect3DDevice8_SetTexture(d, 0, (IDirect3DBaseTexture8 *)t));
    EQ(IDirect3DTexture8_AddRef(t), 2);
    EQ(IDirect3DTexture8_Release(t), 1);
    EQ(IDirect3DTexture8_Release(t), 0);
    HR(IDirect3DDevice8_GetTexture(d, 0, &got));
    ok(got == (IDirect3DBaseTexture8 *)t, "the released texture is still stage 0's");
    HR(IDirect3DTexture8_GetLevelDesc(t, 0, &de));
    EQ(de.Width, 64);
    if (got) EQ(IDirect3DBaseTexture8_Release(got), 0);
    HR(IDirect3DDevice8_SetTexture(d, 0, NULL));                /* the last reference: it goes */
    HR(IDirect3DDevice8_SetStreamSource(d, 0, NULL, 0));        /* and the buffer */

    /* a frame after all that */
    HR(IDirect3DDevice8_Clear(d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0xFF336699, 1.0f, 0));
    HR(IDirect3DDevice8_Present(d, NULL, NULL, NULL, NULL));
    IDirect3DDevice8_Release(d);
    IDirect3D8_Release(o);
    DestroyWindow(w);
    fake_stop(f);
    printf("%s d3d8_ref_test: %d checks, %d failed\n", g_bad ? "FAIL" : "OK", g_checks, g_bad);
    return g_bad != 0;
}
