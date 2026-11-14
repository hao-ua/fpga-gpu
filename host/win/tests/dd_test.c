/* dd_test — the DirectDraw library through the real API on the boards: linked against ddraw.dll
 * like a game, one surface version per test. After every group the GPU's copy of each surface is read back and compared with what Lock
 * shows: the two execute the same operations on different machines, so a difference is a bug in one of them.
 *   dd_test [--secs S]      (the last test runs a WinQuake-style frame loop for S seconds and prints fps) */
#define CINTERFACE
#include <windows.h>
#include <initguid.h>
#include <ddraw.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

HRESULT WINAPI GPU2DD_ReadBack(LPDIRECTDRAWSURFACE7 s, void *out);

#define CHECK(hr) do { HRESULT h_ = (hr); if (FAILED(h_)) { printf("FAIL line %d: %s = %08lx\n", __LINE__, #hr, (unsigned long)h_); exit(1); } } while (0)
#define EXPECT(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

static double now_s(void)
{
    LARGE_INTEGER t, f;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&t);
    return (double)t.QuadPart / (double)f.QuadPart;
}

static int bytes_of(LPDIRECTDRAWSURFACE7 s)
{
    DDPIXELFORMAT pf;
    memset(&pf, 0, sizeof pf); pf.dwSize = sizeof pf;
    CHECK(IDirectDrawSurface7_GetPixelFormat(s, &pf));
    return (int)pf.dwRGBBitCount / 8;
}

/* the board's image against the shadow, pixel by pixel */
static int compare(LPDIRECTDRAWSURFACE7 s, const char *what)
{
    DDSURFACEDESC2 d;
    int bytes = bytes_of(s), x, y, bad = 0;
    uint8_t *img;
    memset(&d, 0, sizeof d); d.dwSize = sizeof d;
    CHECK(IDirectDrawSurface7_GetSurfaceDesc(s, &d));
    img = (uint8_t *)malloc((size_t)d.dwWidth * d.dwHeight * bytes);
    CHECK(GPU2DD_ReadBack(s, img));
    CHECK(IDirectDrawSurface7_Lock(s, NULL, &d, DDLOCK_WAIT | DDLOCK_READONLY, NULL));
    for (y = 0; y < (int)d.dwHeight; y++) {
        const uint8_t *row = (const uint8_t *)d.lpSurface + (ptrdiff_t)y * d.lPitch, *gpu = img + (size_t)y * d.dwWidth * bytes;
        for (x = 0; x < (int)d.dwWidth; x++) {
            /* 32-bit: BGRX read-back forces X = 255, compare colour only */
            int n = bytes == 4 ? 3 : bytes;
            if (memcmp(row + x * bytes, gpu + x * bytes, n)) {
                if (++bad <= 3) printf("  %s: pixel (%d, %d) differs: shadow %02x%02x gpu %02x%02x\n", what, x, y, row[x * bytes], row[x * bytes + bytes - 1], gpu[x * bytes], gpu[x * bytes + bytes - 1]);
            }
        }
    }
    CHECK(IDirectDrawSurface7_Unlock(s, NULL));
    free(img);
    printf("%s %s: %lu x %lu x %d, %d pixels differ between the shadow and the board\n", bad ? "FAIL" : "ok:", what, d.dwWidth, d.dwHeight, bytes * 8, bad);
    return bad;
}

static void paint(LPDIRECTDRAWSURFACE7 s, uint32_t seed)
{
    DDSURFACEDESC2 d;
    int bytes = bytes_of(s), x, y;
    uint32_t v = seed;
    memset(&d, 0, sizeof d); d.dwSize = sizeof d;
    CHECK(IDirectDrawSurface7_Lock(s, NULL, &d, DDLOCK_WAIT | DDLOCK_WRITEONLY, NULL));
    for (y = 0; y < (int)d.dwHeight; y++) {
        uint8_t *row = (uint8_t *)d.lpSurface + (ptrdiff_t)y * d.lPitch;
        for (x = 0; x < (int)d.dwWidth; x++) {
            v = v * 1664525u + 1013904223u;
            memcpy(row + x * bytes, &v, bytes);
        }
    }
    CHECK(IDirectDrawSurface7_Unlock(s, NULL));
}

static LPDIRECTDRAWSURFACE7 offscreen(LPDIRECTDRAW7 dd, int w, int h, DWORD caps, const DDPIXELFORMAT *pf)
{
    DDSURFACEDESC2 d;
    LPDIRECTDRAWSURFACE7 s;
    memset(&d, 0, sizeof d); d.dwSize = sizeof d;
    d.dwFlags = DDSD_CAPS | DDSD_WIDTH | DDSD_HEIGHT | (pf ? DDSD_PIXELFORMAT : 0);
    d.dwWidth = (DWORD)w; d.dwHeight = (DWORD)h;
    d.ddsCaps.dwCaps = DDSCAPS_OFFSCREENPLAIN | caps;
    if (pf) d.ddpfPixelFormat = *pf;
    CHECK(IDirectDraw7_CreateSurface(dd, &d, &s, NULL));
    return s;
}

static void blit_suite(LPDIRECTDRAW7 dd, LPDIRECTDRAWSURFACE7 prim, LPDIRECTDRAWSURFACE7 back, const char *name)
{
    LPDIRECTDRAWSURFACE7 a = offscreen(dd, 256, 160, DDSCAPS_VIDEOMEMORY, NULL), b = offscreen(dd, 200, 120, DDSCAPS_VIDEOMEMORY, NULL);
    LPDIRECTDRAWSURFACE7 sys = offscreen(dd, 64, 64, DDSCAPS_SYSTEMMEMORY, NULL);
    DDCOLORKEY key;
    DDBLTFX fx;
    RECT s, d;
    int bad = 0;
    char label[64];
    paint(a, 1); paint(b, 2); paint(sys, 3); paint(back, 4);
    /* copy, overlap, stretch both ways, mirrors */
    SetRect(&s, 0, 0, 256, 160); SetRect(&d, 0, 0, 200, 120);
    CHECK(IDirectDrawSurface7_Blt(b, &d, a, &s, DDBLT_WAIT, NULL));
    SetRect(&s, 20, 20, 50, 40); SetRect(&d, 100, 50, 190, 110);
    memset(&fx, 0, sizeof fx); fx.dwSize = sizeof fx; fx.dwDDFX = DDBLTFX_MIRRORLEFTRIGHT;
    CHECK(IDirectDrawSurface7_Blt(b, &d, a, &s, DDBLT_WAIT | DDBLT_DDFX, &fx));
    SetRect(&s, 0, 0, 100, 60); SetRect(&d, 7, 5, 107, 65);
    CHECK(IDirectDrawSurface7_Blt(a, &d, a, &s, DDBLT_WAIT, NULL));
    SetRect(&s, 10, 10, 110, 70);
    CHECK(IDirectDrawSurface7_BltFast(a, 2, 3, a, &s, DDBLTFAST_WAIT));
    /* source colour key: a value that occurs */
    {
        DDSURFACEDESC2 l;
        uint32_t v = 0;
        int by = bytes_of(a);
        memset(&l, 0, sizeof l); l.dwSize = sizeof l;
        CHECK(IDirectDrawSurface7_Lock(b, NULL, &l, DDLOCK_WAIT | DDLOCK_READONLY, NULL));
        memcpy(&v, (uint8_t *)l.lpSurface + 3 * l.lPitch + 5 * by, by);
        CHECK(IDirectDrawSurface7_Unlock(b, NULL));
        if (by == 4) v &= 0xFFFFFF;
        key.dwColorSpaceLowValue = key.dwColorSpaceHighValue = v;
        CHECK(IDirectDrawSurface7_SetColorKey(b, DDCKEY_SRCBLT, &key));
        CHECK(IDirectDrawSurface7_BltFast(back, 11, 13, b, NULL, DDBLTFAST_WAIT | DDBLTFAST_SRCCOLORKEY));
        SetRect(&d, 300, 200, 450, 290);
        CHECK(IDirectDrawSurface7_Blt(back, &d, b, NULL, DDBLT_WAIT | DDBLT_KEYSRC, NULL));
    }
    /* colour fill, from system memory (WRITE_IMAGE path), destination key (CPU path), a clipper cutting a blit */
    memset(&fx, 0, sizeof fx); fx.dwSize = sizeof fx; fx.dwFillColor = 0x0000F81F & (bytes_of(a) == 1 ? 0x7F : 0xFFFFFFFFu);
    SetRect(&d, 40, 40, 140, 90);
    CHECK(IDirectDrawSurface7_Blt(a, &d, NULL, NULL, DDBLT_WAIT | DDBLT_COLORFILL, &fx));
    CHECK(IDirectDrawSurface7_BltFast(back, 100, 300, sys, NULL, DDBLTFAST_WAIT));
    SetRect(&d, 200, 300, 328, 428);
    CHECK(IDirectDrawSurface7_Blt(back, &d, sys, NULL, DDBLT_WAIT, NULL));
    key.dwColorSpaceLowValue = key.dwColorSpaceHighValue = fx.dwFillColor;
    CHECK(IDirectDrawSurface7_SetColorKey(a, DDCKEY_DESTBLT, &key));
    SetRect(&d, 30, 30, 150, 100); SetRect(&s, 0, 0, 120, 70);
    CHECK(IDirectDrawSurface7_Blt(a, &d, b, &s, DDBLT_WAIT | DDBLT_KEYDEST, NULL));
    {
        LPDIRECTDRAWCLIPPER c;
        struct { RGNDATAHEADER h; RECT r[2]; } rgn;
        CHECK(IDirectDraw7_CreateClipper(dd, 0, &c, NULL));
        memset(&rgn, 0, sizeof rgn);
        rgn.h.dwSize = sizeof rgn.h; rgn.h.iType = RDH_RECTANGLES; rgn.h.nCount = 2; rgn.h.nRgnSize = sizeof rgn.r;
        SetRect(&rgn.r[0], 0, 0, 60, 60); SetRect(&rgn.r[1], 100, 80, 200, 120);
        SetRect(&rgn.h.rcBound, 0, 0, 200, 120);
        CHECK(IDirectDrawClipper_SetClipList(c, (LPRGNDATA)&rgn, 0));
        CHECK(IDirectDrawSurface7_SetClipper(b, c));
        SetRect(&d, -10, -10, 190, 110);
        CHECK(IDirectDrawSurface7_Blt(b, &d, a, NULL, DDBLT_WAIT, NULL));
        CHECK(IDirectDrawSurface7_SetClipper(b, NULL));
        IDirectDrawClipper_Release(c);
    }
    /* GDI into a surface */
    {
        HDC dc;
        CHECK(IDirectDrawSurface7_GetDC(a, &dc));
        TextOutA(dc, 5, 5, "gpu2", 4);
        Rectangle(dc, 150, 100, 220, 150);
        CHECK(IDirectDrawSurface7_ReleaseDC(a, dc));
    }
    snprintf(label, sizeof label, "%s: offscreen a", name); bad += compare(a, label);
    snprintf(label, sizeof label, "%s: offscreen b", name); bad += compare(b, label);
    snprintf(label, sizeof label, "%s: back buffer", name); bad += compare(back, label);
    IDirectDrawSurface7_Release(a); IDirectDrawSurface7_Release(b); IDirectDrawSurface7_Release(sys);
    if (bad) exit(1);
    (void)prim;
}

static LRESULT CALLBACK wndproc(HWND h, UINT m, WPARAM w, LPARAM l) { return DefWindowProcA(h, m, w, l); }

int main(int argc, char **argv)
{
    WNDCLASSA wc;
    HWND hwnd;
    LPDIRECTDRAW7 dd;
    double secs = 3.0;
    static const struct { int w, h, bpp; } modes[] = { { 640, 480, 8 }, { 800, 600, 16 }, { 640, 480, 32 } };
    int m;
    if (argc == 3 && !strcmp(argv[1], "--secs")) secs = atof(argv[2]);
    memset(&wc, 0, sizeof wc);
    wc.lpfnWndProc = wndproc; wc.lpszClassName = "dd_test"; wc.hInstance = GetModuleHandleA(NULL);
    RegisterClassA(&wc);
    hwnd = CreateWindowA("dd_test", "dd_test", WS_POPUP | WS_VISIBLE, 0, 0, 200, 200, NULL, NULL, wc.hInstance, NULL);
    CHECK(DirectDrawCreateEx(NULL, (void **)&dd, &IID_IDirectDraw7, NULL));
    CHECK(IDirectDraw7_SetCooperativeLevel(dd, hwnd, DDSCL_EXCLUSIVE | DDSCL_FULLSCREEN));
    {   /* version facets: the same object through every interface */
        LPDIRECTDRAW d1;
        LPDIRECTDRAW2 d2;
        LPDIRECTDRAW4 d4;
        CHECK(IDirectDraw7_QueryInterface(dd, &IID_IDirectDraw, (void **)&d1));
        CHECK(IDirectDraw7_QueryInterface(dd, &IID_IDirectDraw2, (void **)&d2));
        CHECK(IDirectDraw7_QueryInterface(dd, &IID_IDirectDraw4, (void **)&d4));
        CHECK(IDirectDraw_SetDisplayMode(d1, 640, 480, 8));     /* the 3-argument slot */
        IDirectDraw_Release(d1); IDirectDraw2_Release(d2); IDirectDraw4_Release(d4);
        printf("ok: QueryInterface IDirectDraw 1 / 2 / 4, IDirectDraw::SetDisplayMode\n");
    }
    for (m = 0; m < 3; m++) {
        DDSURFACEDESC2 d;
        LPDIRECTDRAWSURFACE7 prim, back;
        DDSCAPS2 caps;
        char name[40];
        CHECK(IDirectDraw7_SetDisplayMode(dd, modes[m].w, modes[m].h, modes[m].bpp, 0, 0));
        memset(&d, 0, sizeof d); d.dwSize = sizeof d;
        d.dwFlags = DDSD_CAPS | DDSD_BACKBUFFERCOUNT;
        d.ddsCaps.dwCaps = DDSCAPS_PRIMARYSURFACE | DDSCAPS_FLIP | DDSCAPS_COMPLEX;
        d.dwBackBufferCount = 1;
        CHECK(IDirectDraw7_CreateSurface(dd, &d, &prim, NULL));
        memset(&caps, 0, sizeof caps); caps.dwCaps = DDSCAPS_BACKBUFFER;
        CHECK(IDirectDrawSurface7_GetAttachedSurface(prim, &caps, &back));
        if (modes[m].bpp == 8) {
            LPDIRECTDRAWPALETTE p;
            PALETTEENTRY e[256];
            int i;
            for (i = 0; i < 256; i++) { e[i].peRed = (BYTE)i; e[i].peGreen = (BYTE)(255 - i); e[i].peBlue = (BYTE)(i * 3); e[i].peFlags = 0; }
            CHECK(IDirectDraw7_CreatePalette(dd, DDPCAPS_8BIT | DDPCAPS_ALLOW256, e, &p, NULL));
            CHECK(IDirectDrawSurface7_SetPalette(prim, p));
            IDirectDrawPalette_Release(p);
        }
        snprintf(name, sizeof name, "%d x %d x %d", modes[m].w, modes[m].h, modes[m].bpp);
        blit_suite(dd, prim, back, name);
        CHECK(IDirectDrawSurface7_Flip(prim, NULL, DDFLIP_WAIT));
        if (compare(prim, "front after Flip")) return 1;
        /* WinQuake's loop: full Lock, the game writes every pixel, Unlock, Flip */
        if (modes[m].bpp == 8 || modes[m].bpp == 16) {
            double t0 = now_s();
            int frames = 0;
            while (now_s() - t0 < secs) {
                DDSURFACEDESC2 l;
                LPDIRECTDRAWSURFACE7 bk;
                int y;
                memset(&caps, 0, sizeof caps); caps.dwCaps = DDSCAPS_BACKBUFFER;
                CHECK(IDirectDrawSurface7_GetAttachedSurface(prim, &caps, &bk));
                memset(&l, 0, sizeof l); l.dwSize = sizeof l;
                CHECK(IDirectDrawSurface7_Lock(bk, NULL, &l, DDLOCK_WAIT | DDLOCK_WRITEONLY, NULL));
                for (y = 0; y < (int)l.dwHeight; y++) memset((uint8_t *)l.lpSurface + (ptrdiff_t)y * l.lPitch, (y + frames) & 255, (size_t)l.dwWidth * modes[m].bpp / 8);
                CHECK(IDirectDrawSurface7_Unlock(bk, NULL));
                CHECK(IDirectDrawSurface7_Flip(prim, NULL, DDFLIP_WAIT));
                IDirectDrawSurface7_Release(bk);
                frames++;
            }
            printf("ok: %s Lock + Flip loop: %d frames in %.1f s = %.1f fps\n", name, frames, now_s() - t0, frames / (now_s() - t0));
        }
        CHECK(IDirectDraw7_WaitForVerticalBlank(dd, DDWAITVB_BLOCKBEGIN, NULL));
        IDirectDrawSurface7_Release(back);
        IDirectDrawSurface7_Release(prim);
    }
    {   /* 320 x 200 x 8: not an HDMI mode, scaled to fit mode 0 */
        DDSURFACEDESC2 d;
        LPDIRECTDRAWSURFACE7 prim, back;
        DDSCAPS2 caps;
        CHECK(IDirectDraw7_SetDisplayMode(dd, 320, 200, 8, 0, 0));
        memset(&d, 0, sizeof d); d.dwSize = sizeof d;
        d.dwFlags = DDSD_CAPS | DDSD_BACKBUFFERCOUNT;
        d.ddsCaps.dwCaps = DDSCAPS_PRIMARYSURFACE | DDSCAPS_FLIP | DDSCAPS_COMPLEX;
        d.dwBackBufferCount = 1;
        CHECK(IDirectDraw7_CreateSurface(dd, &d, &prim, NULL));
        memset(&caps, 0, sizeof caps); caps.dwCaps = DDSCAPS_BACKBUFFER;
        CHECK(IDirectDrawSurface7_GetAttachedSurface(prim, &caps, &back));
        paint(back, 9);
        CHECK(IDirectDrawSurface7_Flip(prim, NULL, DDFLIP_WAIT));
        if (compare(prim, "320 x 200 x 8 front")) return 1;
        IDirectDrawSurface7_Release(back);
        IDirectDrawSurface7_Release(prim);
    }
    {   /* 5:5:5 surfaces: stored as RGB565, which holds every 5:5:5 value; a board copy with a source key between two of them */
        DDPIXELFORMAT pf;
        LPDIRECTDRAWSURFACE7 s[2];
        DDCOLORKEY key;
        DDSURFACEDESC2 l;
        uint16_t v = 0;
        int i, y, x;
        memset(&pf, 0, sizeof pf); pf.dwSize = sizeof pf; pf.dwFlags = DDPF_RGB; pf.dwRGBBitCount = 16;
        pf.dwRBitMask = 0x7C00; pf.dwGBitMask = 0x03E0; pf.dwBBitMask = 0x001F;
        for (i = 0; i < 2; i++) {
            s[i] = offscreen(dd, 64, 32, DDSCAPS_VIDEOMEMORY, &pf);
            paint(s[i], 5u + (uint32_t)i);
            memset(&l, 0, sizeof l); l.dwSize = sizeof l;
            CHECK(IDirectDrawSurface7_Lock(s[i], NULL, &l, DDLOCK_WAIT, NULL));       /* bit 15 is no colour, and the board keeps none */
            for (y = 0; y < 32; y++) for (x = 0; x < 64; x++) ((uint16_t *)((uint8_t *)l.lpSurface + (ptrdiff_t)y * l.lPitch))[x] &= 0x7FFF;
            CHECK(IDirectDrawSurface7_Unlock(s[i], NULL));
        }
        memset(&l, 0, sizeof l); l.dwSize = sizeof l;
        CHECK(IDirectDrawSurface7_Lock(s[1], NULL, &l, DDLOCK_WAIT | DDLOCK_READONLY, NULL));
        v = ((uint16_t *)((uint8_t *)l.lpSurface + 3 * l.lPitch))[5];
        CHECK(IDirectDrawSurface7_Unlock(s[1], NULL));
        key.dwColorSpaceLowValue = key.dwColorSpaceHighValue = v;
        CHECK(IDirectDrawSurface7_SetColorKey(s[1], DDCKEY_SRCBLT, &key));
        CHECK(IDirectDrawSurface7_BltFast(s[0], 0, 0, s[1], NULL, DDBLTFAST_WAIT | DDBLTFAST_SRCCOLORKEY));
        if (compare(s[0], "5:5:5 offscreen after a keyed copy")) return 1;
        if (compare(s[1], "5:5:5 offscreen, the source")) return 1;
        IDirectDrawSurface7_Release(s[0]); IDirectDrawSurface7_Release(s[1]);
    }
    CHECK(IDirectDraw7_RestoreDisplayMode(dd));
    IDirectDraw7_Release(dd);
    printf("OK dd_test\n");
    return 0;
}
