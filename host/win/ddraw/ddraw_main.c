/* ddraw.dll — DirectDraw 1–7 on the two-board GPU.
 *
 * A surface = a buffer: shadow pixels in PC memory (the game's format) + an optional board-B image + dirty rectangles. Every blit runs on
 * the shadow (ddraw_blit.h); blits and fills the hardware has also run there as COPY_IMAGE / FILL_IMAGE, the others leave a dirty rectangle
 * that goes out as WRITE_IMAGE before the buffer is next needed. The shadow is therefore always current and Lock never reads back (a
 * read-back costs a pipeline drain + 44 MB/s). Flip rotates buffers and PRESENTs.
 *
 * Interfaces: IDirectDrawSurface 1, 2, 3, 4, 7 only ever append vtable slots, so one vtable serves them all and each interface pointer is a
 * facet {vtbl, surface, version}; the same for IDirectDraw 2, 4, 7, with IDirectDraw 1's own vtable for its 3-argument SetDisplayMode.
 * Differences by version (DDSURFACEDESC vs ...2, DDSCAPS vs ...2) are decided by the facet's version or the struct's dwSize.
 */
#define _WIN32_WINNT 0x0601
#define WIN32_LEAN_AND_MEAN
#define CINTERFACE
#include <windows.h>
#include <objbase.h>
#include <initguid.h>
#include <ddraw.h>
#include <math.h>
#include <stdio.h>
#include "g2_env.h"
#include "ddraw_blit.h"

#define MAX_DIRTY 8
#define MAX_LOCKS 4

typedef struct ddbuf {
    LONG refs;
    uint8_t *pix;
    int own_pix;
    int w, h, fmt;
    int32_t pitch;
    uint16_t handle;                        /* board-B image, 0: shadow only */
    uint32_t base;
    RECT dirty[MAX_DIRTY];
    int ndirty;
} ddbuf;

struct surf;
struct ddobj;
typedef struct iface_s { const IDirectDrawSurface7Vtbl *vtbl; struct surf *s; int ver; } iface_s;
typedef struct iface_g { const IDirectDrawGammaControlVtbl *vtbl; struct surf *s; } iface_g;
typedef struct iface_d { const void *vtbl; struct ddobj *o; int ver; } iface_d;

typedef struct pal {
    const IDirectDrawPaletteVtbl *vtbl;
    LONG refs;
    DWORD caps;
    PALETTEENTRY e[256];
    uint32_t rgb[256];                      /* 0x00RRGGBB */
} pal;

typedef struct clip {
    const IDirectDrawClipperVtbl *vtbl;
    LONG refs;
    HWND hwnd;
    RGNDATA *list;
} clip;

typedef struct privdata { struct privdata *next; GUID tag; DWORD size; void *data; } privdata;

typedef struct surf {
    iface_s f[5];
    iface_g gamma;
    LONG refs;
    struct ddobj *dd;
    struct surf *next, *prev;               /* the object's list */
    ddbuf *buf;
    DWORD caps;
    int primary;
    struct surf *root, *next_flip;          /* the flip chain: root = front buffer, ring through next_flip */
    int nlocks;
    RECT locks[MAX_LOCKS];
    pal *palette;
    clip *clipper;
    DDCOLORKEY ck_src, ck_dst;
    int has_ck_src, has_ck_dst;
    HDC dc;
    HBITMAP bmp, oldbmp;
    void *dib;
    DWORD unique, priority, lod;
    privdata *priv;
    DDGAMMARAMP ramp;
    int has_ramp;
} surf;

typedef struct ddobj {
    iface_d i1, i2, i4, i7;
    LONG refs;
    surf *surfaces;
} ddobj;

static struct {
    CRITICAL_SECTION cs;
    g2_board *bd;
    g2_heap *heap;
    g2_handles imgs;
    int hold, stretch;
    int mode;                               /* the HDMI mode */
    int mode_w, mode_h;
    int disp_w, disp_h, disp_fmt, have_mode;
    HWND hwnd;
    DWORD coop;
    int windowed;
    surf *primary;
    pal *primary_pal;
    uint16_t scan[2];                       /* mode-sized images of a game mode that is not an HDMI mode */
    uint32_t scan_base[2];
    int scan_fmt, scan_idx, scan_mode;
    uint16_t win_img;                       /* windowed: the mode-sized image the client area goes to */
    uint32_t win_base;
    int win_mode;
    g2_ref pending[2];
    int npending;
    uint32_t frame;
    uint16_t shown;
    double vb_t0;
    uint64_t n_hw_copy, n_hw_fill, n_cpu_only, n_uploads, n_upload_bytes, n_flips;
} g;

static const IDirectDrawSurface7Vtbl surf_vtbl;
static const IDirectDrawPaletteVtbl pal_vtbl;
static const IDirectDrawClipperVtbl clip_vtbl;
static const IDirectDrawGammaControlVtbl gamma_vtbl;
static const IDirectDraw7Vtbl dd_vtbl;
static const IDirectDrawVtbl dd1_vtbl;

#define LOCK() EnterCriticalSection(&g.cs)
#define UNLOCK() LeaveCriticalSection(&g.cs)
#define RET(x) do { HRESULT r_ = (x); UNLOCK(); return r_; } while (0)

static void log_guid(const char *what, REFIID r)
{
    g2e_log("ddraw: %s: unknown interface {%08lx-%04x-%04x-%02x%02x-%02x%02x%02x%02x%02x%02x}", what, r->Data1, r->Data2, r->Data3, r->Data4[0], r->Data4[1],
            r->Data4[2], r->Data4[3], r->Data4[4], r->Data4[5], r->Data4[6], r->Data4[7]);
}

static int ver_index(int ver) { return ver == 7 ? 4 : ver - 1; }
static void *surf_iface(surf *s, int ver) { return &s->f[ver_index(ver)]; }
static surf *to_surf(void *p) { return p ? ((iface_s *)p)->s : NULL; }
static double now_s(void)
{
    LARGE_INTEGER t, f;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&t);
    return (double)t.QuadPart / (double)f.QuadPart;
}

/* ---------------------------------------------------------------- formats */
static int bpp_to_fmt(int bpp)
{
    switch (bpp) {
    case 8: return F_P8;
    case 16: return F_565;
    case 24: return F_888;
    case 32: return F_X888;
    default: return -1;
    }
}

static int fmt_bits(int fmt) { return FMT[fmt].bytes * 8; }

static int pf_to_fmt(const DDPIXELFORMAT *pf)
{
    if (pf->dwFlags & DDPF_PALETTEINDEXED8) return F_P8;
    if (!(pf->dwFlags & DDPF_RGB)) return -1;
    switch (pf->dwRGBBitCount) {
    case 16:
        if (pf->dwRBitMask == 0xF800 && pf->dwGBitMask == 0x07E0 && pf->dwBBitMask == 0x001F) return F_565;
        if (pf->dwRBitMask == 0x7C00 && pf->dwGBitMask == 0x03E0 && pf->dwBBitMask == 0x001F) return F_555;
        return -1;
    case 24:
        return pf->dwRBitMask == 0xFF0000 && pf->dwBBitMask == 0xFF ? F_888 : -1;
    case 32:
        if (pf->dwRBitMask != 0xFF0000 || pf->dwGBitMask != 0xFF00 || pf->dwBBitMask != 0xFF) return -1;
        return (pf->dwFlags & DDPF_ALPHAPIXELS) && pf->dwRGBAlphaBitMask == 0xFF000000u ? F_A888 : F_X888;
    default:
        return -1;
    }
}

static void fill_pf(int fmt, DDPIXELFORMAT *pf)
{
    memset(pf, 0, sizeof *pf);
    pf->dwSize = sizeof *pf;
    pf->dwRGBBitCount = (DWORD)fmt_bits(fmt);
    if (fmt == F_P8) { pf->dwFlags = DDPF_RGB | DDPF_PALETTEINDEXED8; return; }
    pf->dwFlags = DDPF_RGB;
    pf->dwRBitMask = FMT[fmt].r; pf->dwGBitMask = FMT[fmt].g; pf->dwBBitMask = FMT[fmt].b;
    if (FMT[fmt].a) { pf->dwFlags |= DDPF_ALPHAPIXELS; pf->dwRGBAlphaBitMask = FMT[fmt].a; }
}

/* ---------------------------------------------------------------- the boards */
static int board_up(void)
{
    uint32_t st[G2_ST_WORDS];
    if (g.bd) return 0;
    g2e_crash_handler();
    g.bd = g2e_connect("ddraw");
    if (!g.bd) return -1;
    g.heap = g2_heap_create(G2_HEAP_B + 0x10000000u, G2_HEAP_B_END);
    memset(&g.imgs, 0, sizeof g.imgs);
    g.imgs.limit = G2_HANDLES_MAX;          /* image handles are virtual: gpu2core maps them onto board B's table */
    g.hold = g2e_get_int("GPU2_HOLD", 1);
    g.stretch = !strcmp(g2e_get("GPU2_SCALE", "fit"), "stretch");
    g.scan_fmt = -1; g.win_mode = -1;
    g.vb_t0 = now_s();
    if (g2_status(g.bd, st, NULL) == 0) {
        g.mode = g2_mode_of_flags(st[G2_ST_B_FLAGS]);
        g.mode_w = G2_MODE_W[g.mode]; g.mode_h = G2_MODE_H[g.mode];
    }
    return 0;
}

/* the smallest HDMI mode that holds w x h (g2_mode_select: 1 024 x 768 on a B that has it, else 1:1 in 1 280 x 800) */
static int set_hdmi_mode(int w, int h)
{
    if (g2_mode_select(g.bd, w, h, &g.mode) < 0) { g2e_log("ddraw: SET_MODE for %d x %d: %s", w, h, g2_error_text(g.bd)); return -1; }
    g.mode_w = G2_MODE_W[g.mode]; g.mode_h = G2_MODE_H[g.mode];
    return 0;
}

static uint32_t image_pitch(int w, int fmt)
{
    uint32_t bytes = (uint32_t)(FMT[fmt].g2_fmt == G2_FMT_RGBA8 ? 4 : FMT[fmt].bytes);
    return ((uint32_t)w * bytes + 31) & ~31u;
}

/* ---------------------------------------------------------------- buffers */
static uint32_t clear_value(int fmt) { return FMT[fmt].g2_fmt == G2_FMT_RGBA8 ? 0x000000FFu : 0; }

static uint16_t image_new(int w, int h, int fmt, uint32_t *base)
{
    uint32_t pitch = image_pitch(w, fmt), a;
    uint16_t handle;
    if (g2_handle_alloc(&g.imgs, G2_HK_IMG_B, &handle) < 0) return 0;
    a = g2_heap_alloc(g.heap, pitch * (uint32_t)h, 32);
    if (!a) { g2_handle_free(&g.imgs, handle); return 0; }
    g2_create_image(g.bd, handle, a, (uint32_t)w, (uint32_t)h, (uint32_t)FMT[fmt].g2_fmt, pitch, 0, 1, 1, 0,
                    G2_USE_SAMPLED | G2_USE_SCANOUT | G2_USE_TRANSFER);
    g2_fill_image(g.bd, handle, G2_TF_RAW, clear_value(fmt), 0, 0, (uint32_t)w, (uint32_t)h, 0xF, 0, 0);
    *base = a;
    return handle;
}

static void image_free(uint16_t handle, uint32_t base)
{
    g2_destroy(g.bd, handle);
    g2_handle_free(&g.imgs, handle);
    g2_heap_free(g.heap, base);
}

static ddbuf *buf_new(int w, int h, int fmt, int want_image, void *client, int32_t client_pitch)
{
    ddbuf *b = (ddbuf *)calloc(1, sizeof *b);
    if (!b) return NULL;
    b->refs = 1; b->w = w; b->h = h; b->fmt = fmt;
    if (client) {
        b->pix = (uint8_t *)client; b->pitch = client_pitch;
    } else {
        b->pitch = (w * FMT[fmt].bytes + 3) & ~3;               /* DWORD aligned: the GetDC DIB has the same rows */
        b->pix = (uint8_t *)calloc((size_t)b->pitch, (size_t)h);
        b->own_pix = 1;
        if (!b->pix) { free(b); return NULL; }
    }
    if (want_image) b->handle = image_new(w, h, fmt, &b->base);
    return b;
}

static void buf_release(ddbuf *b)
{
    if (--b->refs) return;
    if (b->handle) image_free(b->handle, b->base);
    if (b->own_pix) free(b->pix);
    free(b);
}

static int rects_touch(const RECT *a, const RECT *b)
{
    return a->left <= b->right && b->left <= a->right && a->top <= b->bottom && b->top <= a->bottom;
}

static void rect_union(RECT *a, const RECT *b)
{
    if (b->left < a->left) a->left = b->left;
    if (b->top < a->top) a->top = b->top;
    if (b->right > a->right) a->right = b->right;
    if (b->bottom > a->bottom) a->bottom = b->bottom;
}

static void dirty_add(ddbuf *b, const RECT *r)
{
    int i;
    RECT n = *r;
    if (n.left >= n.right || n.top >= n.bottom) return;
    for (i = 0; i < b->ndirty;) {                               /* touching rectangles merge */
        if (rects_touch(&b->dirty[i], &n)) {
            rect_union(&n, &b->dirty[i]);
            b->dirty[i] = b->dirty[--b->ndirty];
            i = 0;
        } else {
            i++;
        }
    }
    if (b->ndirty == MAX_DIRTY) {
        for (i = 0; i < b->ndirty; i++) rect_union(&n, &b->dirty[i]);
        b->ndirty = 0;
    }
    b->dirty[b->ndirty++] = n;
}

static int dirty_overlaps(const ddbuf *b, const RECT *r)
{
    int i;
    for (i = 0; i < b->ndirty; i++) {
        const RECT *d = &b->dirty[i];
        if (d->left < r->right && r->left < d->right && d->top < r->bottom && r->top < d->bottom) return 1;
    }
    return 0;
}

/* a 5:5:5 word as the 5:6:5 one that holds it: green's top bit repeats at the bottom, so every 5:5:5 value comes back by dropping that bit */
static uint16_t w565(uint32_t v) { uint32_t g = v >> 5 & 31; return (uint16_t)((v >> 10 & 31) << 11 | (g << 1 | g >> 4) << 5 | (v & 31)); }

static void write_rect(uint16_t handle, const ddbuf *b, int x, int y, int w, int h, int dx, int dy)
{
    const fmt_info *fi = &FMT[b->fmt];
    const uint8_t *src = b->pix + (ptrdiff_t)y * b->pitch + (ptrdiff_t)x * fi->bytes;
    if (b->fmt == F_555) {                                      /* no such storage format: 555 goes into RGB565, which holds it */
        uint16_t *row = (uint16_t *)malloc((size_t)w * h * 2);
        int i, j;
        if (!row) return;
        for (j = 0; j < h; j++) {
            const uint16_t *s = (const uint16_t *)(src + (ptrdiff_t)j * b->pitch);
            for (i = 0; i < w; i++) row[(size_t)j * w + i] = w565(s[i]);
        }
        g2_write_image_rect(g.bd, handle, row, w * 2, G2_TF_RAW, (uint32_t)dx, (uint32_t)dy, (uint32_t)w, (uint32_t)h, 0, 0, G2_FMT_RGB565);
        free(row);
        return;
    }
    g2_write_image_rect(g.bd, handle, src, b->pitch, (uint32_t)fi->g2_tf, (uint32_t)dx, (uint32_t)dy, (uint32_t)w, (uint32_t)h, 0, 0,
                        (uint32_t)fi->g2_fmt);
}

static void flush_dirty(ddbuf *b)
{
    int i;
    if (!b->handle) { b->ndirty = 0; return; }
    for (i = 0; i < b->ndirty; i++) {
        const RECT *r = &b->dirty[i];
        g.n_uploads++;
        g.n_upload_bytes += (uint64_t)(r->right - r->left) * (r->bottom - r->top) * FMT[b->fmt].bytes;
        write_rect(b->handle, b, r->left, r->top, r->right - r->left, r->bottom - r->top, r->left, r->top);
    }
    b->ndirty = 0;
}

/* ---------------------------------------------------------------- presentation */
static void pending_poll(void)
{
    while (g.npending) {
        g2_event ev;
        int r = g2_poll_event(g.bd, g.pending[0], &ev);
        if (r == 0) break;
        g.pending[0] = g.pending[1];
        g.npending--;
    }
}

static void pending_wait_one(void)
{
    g2_event ev;
    if (!g.npending) return;
    if (g2_wait_done(g.bd, g.pending[0], &ev, 0.25, 2.0) < 0) g2e_log("ddraw: PRESENT: %s", g2_error_text(g.bd));
    g.pending[0] = g.pending[1];
    g.npending--;
}

static void present_handle(uint16_t handle, int hold)
{
    pending_poll();
    while (g.npending >= 2) pending_wait_one();
    g.pending[g.npending++] = g2_present(g.bd, handle, ++g.frame, 0, hold);
    g.shown = handle;
}

static void scan_images(int fmt)
{
    int i;
    if (g.scan_fmt == fmt && g.scan_mode == g.mode) return;
    for (i = 0; i < 2; i++) {
        if (g.scan[i]) image_free(g.scan[i], g.scan_base[i]);
        g.scan[i] = image_new(g.mode_w, g.mode_h, fmt, &g.scan_base[i]);
    }
    g.scan_fmt = fmt; g.scan_mode = g.mode;
}

/* show a buffer: directly when it is mode-sized, else through a mode-sized image (fit: the largest integer scale, centred; stretch: the whole
   mode, GPU2_SCALE). 1 024 x 768 is an HDMI mode on a board B that has mode 4; one without it shows it 1:1 centred on 1 280 x 800. */
static void show_buffer(ddbuf *b, int hold)
{
    if (b->w == g.mode_w && b->h == g.mode_h) {
        present_handle(b->handle, hold);
    } else {
        int dw = g.mode_w, dh = g.mode_h, dx = 0, dy = 0;
        scan_images(b->fmt);
        if (!g.stretch) {
            int sx = g.mode_w / b->w, sy = g.mode_h / b->h, s = sx < sy ? sx : sy;
            if (s < 1) s = 1;
            dw = b->w * s; dh = b->h * s;
            dx = (g.mode_w - dw) / 2; dy = (g.mode_h - dh) / 2;
        }
        g.scan_idx ^= 1;
        g2_copy_image(g.bd, b->handle, g.scan[g.scan_idx], 0, 0, (uint32_t)b->w, (uint32_t)b->h, (uint32_t)dx, (uint32_t)dy, (uint32_t)dw,
                      (uint32_t)dh, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        present_handle(g.scan[g.scan_idx], hold);
    }
}

/* windowed (DDSCL_NORMAL): the primary is the PC desktop, which the HDMI monitor does not show; only the game's client area goes to a mode-sized
   image */
static void client_rect(RECT *r)
{
    HWND w = g.hwnd;
    if (g.primary && g.primary->clipper && g.primary->clipper->hwnd) w = g.primary->clipper->hwnd;
    if (w && GetClientRect(w, r)) {
        POINT p = { 0, 0 };
        ClientToScreen(w, &p);
        OffsetRect(r, p.x, p.y);
    } else {
        SetRect(r, 0, 0, g.mode_w, g.mode_h);
    }
}

static void flush_windowed(ddbuf *b)
{
    RECT c;
    int i;
    client_rect(&c);
    if (g.win_mode != g.mode || !g.win_img) {
        set_hdmi_mode(c.right - c.left, c.bottom - c.top);
        if (g.win_img) image_free(g.win_img, g.win_base);
        g.win_img = image_new(g.mode_w, g.mode_h, F_X888, &g.win_base);
        g.win_mode = g.mode;
        present_handle(g.win_img, 0);
    }
    for (i = 0; i < b->ndirty; i++) {
        RECT r;
        if (IntersectRect(&r, &b->dirty[i], &c)) {
            int dx = r.left - c.left, dy = r.top - c.top, w = r.right - r.left, h = r.bottom - r.top;
            if (dx < g.mode_w && dy < g.mode_h) {
                if (dx + w > g.mode_w) w = g.mode_w - dx;
                if (dy + h > g.mode_h) h = g.mode_h - dy;
                write_rect(g.win_img, b, r.left, r.top, w, h, dx, dy);
            }
        }
    }
    b->ndirty = 0;
}

/* GPU2_DUMP = a directory: every GPU2_DUMP_EVERY-th frame as a BMP, to check frames without watching the monitor */
static void dump_frame(const ddbuf *b)
{
    static int every = -1;
    static uint32_t seq;
    const char *dir = g2e_get("GPU2_DUMP", NULL);
    char path[MAX_PATH];
    BITMAPFILEHEADER fh;
    BITMAPINFOHEADER ih;
    FILE *f;
    int x, y;
    uint32_t *row;
    const uint32_t *pal = g.primary_pal ? g.primary_pal->rgb : NULL;
    if (!dir) return;
    if (every < 0) every = g2e_get_int("GPU2_DUMP_EVERY", 120);
    if (seq++ % (uint32_t)every) return;
    snprintf(path, sizeof path, "%s\\frame_%05u.bmp", dir, seq / (uint32_t)every);
    f = fopen(path, "wb");
    if (!f) return;
    memset(&fh, 0, sizeof fh); memset(&ih, 0, sizeof ih);
    fh.bfType = 0x4D42;
    fh.bfOffBits = sizeof fh + sizeof ih;
    fh.bfSize = fh.bfOffBits + (DWORD)b->w * b->h * 4;
    ih.biSize = sizeof ih; ih.biWidth = b->w; ih.biHeight = -b->h; ih.biPlanes = 1; ih.biBitCount = 32; ih.biCompression = BI_RGB;
    fwrite(&fh, sizeof fh, 1, f); fwrite(&ih, sizeof ih, 1, f);
    row = (uint32_t *)malloc((size_t)b->w * 4);
    for (y = 0; y < b->h && row; y++) {
        const uint8_t *s = b->pix + (ptrdiff_t)y * b->pitch;
        for (x = 0; x < b->w; x++) row[x] = px_to_argb(b->fmt, bx_read(s + (ptrdiff_t)x * FMT[b->fmt].bytes, FMT[b->fmt].bytes), pal);
        fwrite(row, 4, (size_t)b->w, f);
    }
    free(row);
    fclose(f);
}

/* a write reached the buffer on display */
static void primary_touched(ddbuf *b)
{
    dump_frame(b);
    if (g.windowed) { flush_windowed(b); return; }
    flush_dirty(b);
    if (b->w != g.mode_w || b->h != g.mode_h) show_buffer(b, 0);
}

static int on_display(const surf *s) { return g.primary && s->buf == g.primary->buf; }

/* ---------------------------------------------------------------- IDirectDrawPalette */
static uint32_t ramped(const surf *s, const PALETTEENTRY *e)
{
    if (s && s->has_ramp) {
        return (uint32_t)(s->ramp.red[e->peRed] >> 8) << 16 | (uint32_t)(s->ramp.green[e->peGreen] >> 8) << 8 | (s->ramp.blue[e->peBlue] >> 8);
    }
    return (uint32_t)e->peRed << 16 | (uint32_t)e->peGreen << 8 | e->peBlue;
}

static void palette_send(pal *p, DWORD first, DWORD n)
{
    uint32_t c[256];
    DWORD i;
    for (i = 0; i < n; i++) c[i] = ramped(g.primary, &p->e[first + i]);
    g2_set_palette(g.bd, first, c, n);
}

static HRESULT WINAPI p_QI(IDirectDrawPalette *t, REFIID r, void **o)
{
    if (IsEqualGUID(r, &IID_IUnknown) || IsEqualGUID(r, &IID_IDirectDrawPalette)) { *o = t; t->lpVtbl->AddRef(t); return S_OK; }
    *o = NULL;
    return E_NOINTERFACE;
}
static ULONG WINAPI p_AddRef(IDirectDrawPalette *t) { return (ULONG)InterlockedIncrement(&((pal *)t)->refs); }
static ULONG WINAPI p_Release(IDirectDrawPalette *t)
{
    pal *p = (pal *)t;
    LONG n = InterlockedDecrement(&p->refs);
    if (n == 0) { LOCK(); if (g.primary_pal == p) g.primary_pal = NULL; UNLOCK(); free(p); }
    return (ULONG)n;
}
static HRESULT WINAPI p_GetCaps(IDirectDrawPalette *t, LPDWORD c) { *c = ((pal *)t)->caps; return DD_OK; }
static HRESULT WINAPI p_GetEntries(IDirectDrawPalette *t, DWORD f, DWORD first, DWORD n, LPPALETTEENTRY e)
{
    (void)f;
    if (first + n > 256) return DDERR_INVALIDPARAMS;
    memcpy(e, ((pal *)t)->e + first, n * sizeof *e);
    return DD_OK;
}
static HRESULT WINAPI p_Initialize(IDirectDrawPalette *t, LPDIRECTDRAW d, DWORD f, LPPALETTEENTRY e) { (void)t; (void)d; (void)f; (void)e; return DDERR_ALREADYINITIALIZED; }
static HRESULT WINAPI p_SetEntries(IDirectDrawPalette *t, DWORD f, DWORD first, DWORD n, LPPALETTEENTRY e)
{
    pal *p = (pal *)t;
    DWORD i;
    (void)f;
    if (first + n > 256) return DDERR_INVALIDPARAMS;
    LOCK();
    memcpy(p->e + first, e, n * sizeof *e);
    for (i = 0; i < n; i++) p->rgb[first + i] = (uint32_t)e[i].peRed << 16 | (uint32_t)e[i].peGreen << 8 | e[i].peBlue;
    if (g.primary_pal == p) palette_send(p, first, n);
    RET(DD_OK);
}
static const IDirectDrawPaletteVtbl pal_vtbl = { p_QI, p_AddRef, p_Release, p_GetCaps, p_GetEntries, p_Initialize, p_SetEntries };

/* ---------------------------------------------------------------- IDirectDrawClipper */
static HRESULT WINAPI c_QI(IDirectDrawClipper *t, REFIID r, void **o)
{
    if (IsEqualGUID(r, &IID_IUnknown) || IsEqualGUID(r, &IID_IDirectDrawClipper)) { *o = t; t->lpVtbl->AddRef(t); return S_OK; }
    *o = NULL;
    return E_NOINTERFACE;
}
static ULONG WINAPI c_AddRef(IDirectDrawClipper *t) { return (ULONG)InterlockedIncrement(&((clip *)t)->refs); }
static ULONG WINAPI c_Release(IDirectDrawClipper *t)
{
    clip *c = (clip *)t;
    LONG n = InterlockedDecrement(&c->refs);
    if (n == 0) { free(c->list); free(c); }
    return (ULONG)n;
}
static HRESULT WINAPI c_GetClipList(IDirectDrawClipper *t, LPRECT r, LPRGNDATA d, LPDWORD sz)
{
    clip *c = (clip *)t;
    DWORD need;
    RECT w;
    (void)r;
    if (c->hwnd) {
        if (!GetClientRect(c->hwnd, &w)) return DDERR_INVALIDPARAMS;
        { POINT p = { 0, 0 }; ClientToScreen(c->hwnd, &p); OffsetRect(&w, p.x, p.y); }
        need = sizeof(RGNDATAHEADER) + sizeof(RECT);
        if (d) {
            if (*sz < need) { *sz = need; return DDERR_REGIONTOOSMALL; }
            d->rdh.dwSize = sizeof d->rdh; d->rdh.iType = RDH_RECTANGLES; d->rdh.nCount = 1;
            d->rdh.nRgnSize = sizeof(RECT); d->rdh.rcBound = w;
            memcpy(d->Buffer, &w, sizeof w);
        }
        *sz = need;
        return DD_OK;
    }
    if (!c->list) return DDERR_NOCLIPLIST;
    need = sizeof(RGNDATAHEADER) + c->list->rdh.nCount * sizeof(RECT);
    if (d) {
        if (*sz < need) { *sz = need; return DDERR_REGIONTOOSMALL; }
        memcpy(d, c->list, need);
    }
    *sz = need;
    return DD_OK;
}
static HRESULT WINAPI c_GetHWnd(IDirectDrawClipper *t, HWND *h) { *h = ((clip *)t)->hwnd; return DD_OK; }
static HRESULT WINAPI c_Initialize(IDirectDrawClipper *t, LPDIRECTDRAW d, DWORD f) { (void)t; (void)d; (void)f; return DDERR_ALREADYINITIALIZED; }
static HRESULT WINAPI c_IsClipListChanged(IDirectDrawClipper *t, BOOL *b) { (void)t; *b = FALSE; return DD_OK; }
static HRESULT WINAPI c_SetClipList(IDirectDrawClipper *t, LPRGNDATA d, DWORD f)
{
    clip *c = (clip *)t;
    (void)f;
    if (c->hwnd) return DDERR_CLIPPERISUSINGHWND;
    free(c->list);
    c->list = NULL;
    if (d) {
        size_t n = sizeof(RGNDATAHEADER) + d->rdh.nCount * sizeof(RECT);
        c->list = (RGNDATA *)malloc(n);
        if (!c->list) return DDERR_OUTOFMEMORY;
        memcpy(c->list, d, n);
    }
    return DD_OK;
}
static HRESULT WINAPI c_SetHWnd(IDirectDrawClipper *t, DWORD f, HWND h) { (void)f; ((clip *)t)->hwnd = h; return DD_OK; }
static const IDirectDrawClipperVtbl clip_vtbl = { c_QI, c_AddRef, c_Release, c_GetClipList, c_GetHWnd, c_Initialize, c_IsClipListChanged, c_SetClipList, c_SetHWnd };

static HRESULT new_clipper(LPDIRECTDRAWCLIPPER *out)
{
    clip *c = (clip *)calloc(1, sizeof *c);
    if (!c) return DDERR_OUTOFMEMORY;
    c->vtbl = &clip_vtbl;
    c->refs = 1;
    *out = (LPDIRECTDRAWCLIPPER)c;
    return DD_OK;
}

/* ---------------------------------------------------------------- blits */
/* destination clip rectangles of a surface; returns the count (0: everything clipped), NULL list: the whole surface */
static int clip_rects(surf *d, const RECT **list, RECT *one)
{
    clip *c = d->clipper;
    SetRect(one, 0, 0, d->buf->w, d->buf->h);
    *list = one;
    if (!c) return 1;
    if (c->list) {
        *list = (const RECT *)c->list->Buffer;
        return (int)c->list->rdh.nCount;
    }
    if (c->hwnd) {
        RECT w, full = *one;
        if (GetClientRect(c->hwnd, &w)) {
            POINT p = { 0, 0 };
            ClientToScreen(c->hwnd, &p);
            OffsetRect(&w, p.x, p.y);
            if (!IntersectRect(one, &w, &full)) return 0;
        }
    }
    return 1;
}

static uint32_t gpu_key(int fmt, uint32_t v)
{
    if (fmt == F_555) return w565(v);
    return FMT[fmt].g2_fmt == G2_FMT_RGBA8 ? (v & 0xFFFFFF) << 8 : v;
}

static uint32_t gpu_fill(int fmt, uint32_t v)
{
    switch (fmt) {
    case F_P8: case F_565: return v;
    case F_555: return w565(v);
    case F_A888: return v << 8 | v >> 24;
    default: return (v & 0xFFFFFF) << 8 | 0xFF;
    }
}

static const uint32_t *source_palette(surf *s)
{
    if (s->palette) return s->palette->rgb;
    if (g.primary_pal) return g.primary_pal->rgb;
    return NULL;
}

typedef struct blit_req {
    int use_sk, use_dk;
    uint32_t sk_lo, sk_hi, dk_lo, dk_hi;
    int mirror_x, mirror_y;
} blit_req;

static HRESULT do_blit(surf *ds, const RECT *dr, surf *ss, const RECT *sr, const blit_req *q)
{
    ddbuf *db = ds->buf, *sb = ss->buf;
    bview dv = { db->pix, db->pitch, db->fmt, db->w, db->h }, sv = { sb->pix, sb->pitch, sb->fmt, sb->w, sb->h };
    RECT one;
    const RECT *cl;
    int n = clip_rects(ds, &cl, &one), i, whole = 0;
    blit_op o;
    int hw;
    memset(&o, 0, sizeof o);
    o.sx = sr->left; o.sy = sr->top; o.sw = sr->right - sr->left; o.sh = sr->bottom - sr->top;
    o.dx = dr->left; o.dy = dr->top; o.dw = dr->right - dr->left; o.dh = dr->bottom - dr->top;
    o.mirror_x = q->mirror_x; o.mirror_y = q->mirror_y;
    o.use_src_key = q->use_sk; o.use_dst_key = q->use_dk;
    o.sk_lo = q->sk_lo; o.sk_hi = q->sk_hi; o.dk_lo = q->dk_lo; o.dk_hi = q->dk_hi;
    o.pal = source_palette(ss);
    if (o.sw <= 0 || o.sh <= 0 || o.dw <= 0 || o.dh <= 0) return DDERR_INVALIDRECT;
    if (sr->left < 0 || sr->top < 0 || sr->right > sb->w || sr->bottom > sb->h) return DDERR_INVALIDRECT;
    if (!ds->clipper && (dr->left < 0 || dr->top < 0 || dr->right > db->w || dr->bottom > db->h)) return DDERR_INVALIDRECT;
    for (i = 0; i < n; i++) {
        o.cx0 = cl[i].left; o.cy0 = cl[i].top; o.cx1 = cl[i].right; o.cy1 = cl[i].bottom;
        if (o.cx0 < 0) o.cx0 = 0;
        if (o.cy0 < 0) o.cy0 = 0;
        if (o.cx1 > db->w) o.cx1 = db->w;
        if (o.cy1 > db->h) o.cy1 = db->h;
        cpu_blit(&dv, &sv, &o);
        if (o.cx0 <= dr->left && o.cy0 <= dr->top && o.cx1 >= dr->right && o.cy1 >= dr->bottom) whole = 1;
    }
    ds->unique++;
    /* what the hardware can repeat: same format, source colour key by exact value, the whole rectangle (no clip cut it) */
    hw = db->handle && sb->handle && db->fmt == sb->fmt && !q->use_dk && whole &&
         (!q->use_sk || q->sk_lo == q->sk_hi);
    if (hw) {
        g.n_hw_copy++;
        if (dirty_overlaps(sb, sr)) flush_dirty(sb);
        g2_copy_image(g.bd, sb->handle, db->handle, (uint32_t)o.sx, (uint32_t)o.sy, (uint32_t)o.sw, (uint32_t)o.sh, (uint32_t)o.dx, (uint32_t)o.dy,
                      (uint32_t)o.dw, (uint32_t)o.dh, q->mirror_x, q->mirror_y, q->use_sk, gpu_key(sb->fmt, q->sk_lo), gpu_key(sb->fmt, q->sk_hi), 0, 0, 0, 0);
    } else {
        RECT c = *dr;
        RECT bounds = { 0, 0, db->w, db->h };
        g.n_cpu_only++;
        if (IntersectRect(&c, dr, &bounds)) dirty_add(db, &c);
    }
    if (on_display(ds)) primary_touched(db);
    return DD_OK;
}

static HRESULT do_fill(surf *ds, const RECT *dr, uint32_t value)
{
    ddbuf *db = ds->buf;
    bview dv = { db->pix, db->pitch, db->fmt, db->w, db->h };
    RECT one, full = { 0, 0, db->w, db->h }, want = dr ? *dr : full;
    const RECT *cl;
    int n = clip_rects(ds, &cl, &one), i;
    if (!ds->clipper && (want.left < 0 || want.top < 0 || want.right > db->w || want.bottom > db->h)) return DDERR_INVALIDRECT;
    for (i = 0; i < n; i++) {
        RECT r, c = cl[i];
        if (!IntersectRect(&r, &c, &want) || !IntersectRect(&r, &r, &full)) continue;
        cpu_fill(&dv, r.left, r.top, r.right, r.bottom, value);
        if (db->handle) {
            g.n_hw_fill++;
            g2_fill_image(g.bd, db->handle, G2_TF_RAW, gpu_fill(db->fmt, value), (uint32_t)r.left, (uint32_t)r.top, (uint32_t)(r.right - r.left),
                          (uint32_t)(r.bottom - r.top), 0xF, 0, 0);
        } else {
            dirty_add(db, &r);
        }
    }
    ds->unique++;
    if (on_display(ds)) primary_touched(db);
    return DD_OK;
}

/* ---------------------------------------------------------------- IDirectDrawSurface 1..7 */
static void surf_desc(surf *s, DDSURFACEDESC2 *out)
{
    DDSURFACEDESC2 d;
    DWORD size = out->dwSize;
    ddbuf *b = s->buf;
    memset(&d, 0, sizeof d);
    d.dwFlags = DDSD_CAPS | DDSD_WIDTH | DDSD_HEIGHT | DDSD_PITCH | DDSD_PIXELFORMAT;
    d.dwWidth = (DWORD)b->w; d.dwHeight = (DWORD)b->h;
    d.lPitch = b->pitch;
    fill_pf(b->fmt, &d.ddpfPixelFormat);
    d.ddsCaps.dwCaps = s->caps;
    if (s->next_flip) {
        int n = 0;
        surf *q = s->root->next_flip;
        while (q != s->root) { n++; q = q->next_flip; }
        d.dwFlags |= DDSD_BACKBUFFERCOUNT;
        d.dwBackBufferCount = (DWORD)n;
    }
    if (s->has_ck_src) { d.dwFlags |= DDSD_CKSRCBLT; d.ddckCKSrcBlt = s->ck_src; }
    if (s->has_ck_dst) { d.dwFlags |= DDSD_CKDESTBLT; d.ddckCKDestBlt = s->ck_dst; }
    if (size > sizeof d) size = sizeof d;
    memcpy(out, &d, size);
    out->dwSize = size;
}

static void surf_addref_all(surf *s, LONG n) { InterlockedExchangeAdd(&s->refs, n); }

static HRESULT WINAPI s_QueryInterface(IDirectDrawSurface7 *t, REFIID r, void **o)
{
    surf *s = to_surf(t);
    int ver = 0;
    if (IsEqualGUID(r, &IID_IUnknown)) ver = ((iface_s *)t)->ver;
    else if (IsEqualGUID(r, &IID_IDirectDrawSurface)) ver = 1;
    else if (IsEqualGUID(r, &IID_IDirectDrawSurface2)) ver = 2;
    else if (IsEqualGUID(r, &IID_IDirectDrawSurface3)) ver = 3;
    else if (IsEqualGUID(r, &IID_IDirectDrawSurface4)) ver = 4;
    else if (IsEqualGUID(r, &IID_IDirectDrawSurface7)) ver = 7;
    else if (IsEqualGUID(r, &IID_IDirectDrawGammaControl) && s->primary) { *o = &s->gamma; surf_addref_all(s, 1); return S_OK; }
    if (!ver) { log_guid("surface QueryInterface", r); *o = NULL; return E_NOINTERFACE; }
    *o = surf_iface(s, ver);
    surf_addref_all(s, 1);
    return S_OK;
}

static ULONG WINAPI s_AddRef(IDirectDrawSurface7 *t) { return (ULONG)InterlockedIncrement(&to_surf(t)->refs); }

static void surf_free_one(surf *s)
{
    ddobj *o = s->dd;
    privdata *p = s->priv;
    if (s->dc) { SelectObject(s->dc, s->oldbmp); DeleteObject(s->bmp); DeleteDC(s->dc); }
    while (p) { privdata *n = p->next; free(p->data); free(p); p = n; }
    if (s->palette) s->palette->vtbl->Release((IDirectDrawPalette *)s->palette);
    if (s->clipper) s->clipper->vtbl->Release((IDirectDrawClipper *)s->clipper);
    if (s->prev) s->prev->next = s->next; else if (o) o->surfaces = s->next;
    if (s->next) s->next->prev = s->prev;
    buf_release(s->buf);
    free(s);
}

static ULONG WINAPI s_Release(IDirectDrawSurface7 *t)
{
    surf *s = to_surf(t);
    LONG n;
    LOCK();
    n = s->refs > 0 ? InterlockedDecrement(&s->refs) : 0;
    /* a complex surface dies with its root: back buffers have no references of their own to lose */
    if (n == 0 && (!s->root || s->root == s)) {
        if (g.primary == s) { g.primary = NULL; }
        if (s->root) {
            surf *q = s->next_flip;
            while (q != s) { surf *nx = q->next_flip; surf_free_one(q); q = nx; }
        }
        surf_free_one(s);
    }
    UNLOCK();
    return (ULONG)n;
}

static HRESULT WINAPI s_AddAttachedSurface(IDirectDrawSurface7 *t, LPDIRECTDRAWSURFACE7 a) { (void)t; (void)a; return DDERR_CANNOTATTACHSURFACE; }
static HRESULT WINAPI s_AddOverlayDirtyRect(IDirectDrawSurface7 *t, LPRECT r) { (void)t; (void)r; return DDERR_NOTAOVERLAYSURFACE; }

static HRESULT WINAPI s_Blt(IDirectDrawSurface7 *t, LPRECT dr, LPDIRECTDRAWSURFACE7 src, LPRECT sr, DWORD flags, LPDDBLTFX fx)
{
    surf *ds = to_surf(t), *ss = to_surf(src);
    RECT d, s;
    blit_req q;
    HRESULT hr;
    LOCK();
    if (flags & DDBLT_COLORFILL) {
        if (!fx) RET(DDERR_INVALIDPARAMS);
        RET(do_fill(ds, dr, fx->dwFillColor));
    }
    if (flags & DDBLT_DEPTHFILL) RET(DDERR_UNSUPPORTED);
    if (flags & DDBLT_ROP) {
        if (!fx) RET(DDERR_INVALIDPARAMS);
        if (fx->dwROP == BLACKNESS) RET(do_fill(ds, dr, 0));
        if (fx->dwROP == WHITENESS) RET(do_fill(ds, dr, ds->buf->fmt == F_P8 ? 255 : 0xFFFFFFFFu));
        if (fx->dwROP != SRCCOPY) RET(DDERR_NORASTEROPHW);
    }
    if (!ss) RET(DDERR_INVALIDPARAMS);
    memset(&q, 0, sizeof q);
    if (flags & DDBLT_KEYSRC) {
        if (!ss->has_ck_src) RET(DDERR_NOCOLORKEY);
        q.use_sk = 1; q.sk_lo = ss->ck_src.dwColorSpaceLowValue; q.sk_hi = ss->ck_src.dwColorSpaceHighValue;
    }
    if (flags & DDBLT_KEYSRCOVERRIDE) {
        if (!fx) RET(DDERR_INVALIDPARAMS);
        q.use_sk = 1; q.sk_lo = fx->ddckSrcColorkey.dwColorSpaceLowValue; q.sk_hi = fx->ddckSrcColorkey.dwColorSpaceHighValue;
    }
    if (flags & DDBLT_KEYDEST) {
        if (!ds->has_ck_dst) RET(DDERR_NOCOLORKEY);
        q.use_dk = 1; q.dk_lo = ds->ck_dst.dwColorSpaceLowValue; q.dk_hi = ds->ck_dst.dwColorSpaceHighValue;
    }
    if (flags & DDBLT_KEYDESTOVERRIDE) {
        if (!fx) RET(DDERR_INVALIDPARAMS);
        q.use_dk = 1; q.dk_lo = fx->ddckDestColorkey.dwColorSpaceLowValue; q.dk_hi = fx->ddckDestColorkey.dwColorSpaceHighValue;
    }
    if ((flags & DDBLT_DDFX) && fx) {
        if (fx->dwDDFX & DDBLTFX_MIRRORLEFTRIGHT) q.mirror_x = 1;
        if (fx->dwDDFX & DDBLTFX_MIRRORUPDOWN) q.mirror_y = 1;
    }
    if (q.sk_hi < q.sk_lo) q.sk_hi = q.sk_lo;                   /* DirectDraw: a high value below the low one means the single value */
    if (q.dk_hi < q.dk_lo) q.dk_hi = q.dk_lo;
    if (dr) d = *dr; else SetRect(&d, 0, 0, ds->buf->w, ds->buf->h);
    if (sr) s = *sr; else SetRect(&s, 0, 0, ss->buf->w, ss->buf->h);
    hr = do_blit(ds, &d, ss, &s, &q);
    if (FAILED(hr)) g2e_log("ddraw: Blt flags %08lx dst (%ld,%ld)-(%ld,%ld) of %dx%d src (%ld,%ld)-(%ld,%ld) of %dx%d: %08lx", flags, d.left, d.top, d.right, d.bottom,
                            ds->buf->w, ds->buf->h, s.left, s.top, s.right, s.bottom, ss->buf->w, ss->buf->h, (unsigned long)hr);
    RET(hr);
}

static HRESULT WINAPI s_BltBatch(IDirectDrawSurface7 *t, LPDDBLTBATCH b, DWORD n, DWORD f) { (void)t; (void)b; (void)n; (void)f; return DDERR_UNSUPPORTED; }

static HRESULT WINAPI s_BltFast(IDirectDrawSurface7 *t, DWORD x, DWORD y, LPDIRECTDRAWSURFACE7 src, LPRECT sr, DWORD trans)
{
    surf *ds = to_surf(t), *ss = to_surf(src);
    RECT d, s;
    blit_req q;
    if (!ss) return DDERR_INVALIDPARAMS;
    LOCK();
    memset(&q, 0, sizeof q);
    if (sr) s = *sr; else SetRect(&s, 0, 0, ss->buf->w, ss->buf->h);
    SetRect(&d, (int)x, (int)y, (int)x + s.right - s.left, (int)y + s.bottom - s.top);
    if (trans & DDBLTFAST_SRCCOLORKEY) {
        if (!ss->has_ck_src) RET(DDERR_NOCOLORKEY);
        q.use_sk = 1; q.sk_lo = ss->ck_src.dwColorSpaceLowValue; q.sk_hi = ss->ck_src.dwColorSpaceHighValue;
        if (q.sk_hi < q.sk_lo) q.sk_hi = q.sk_lo;
    }
    if (trans & DDBLTFAST_DESTCOLORKEY) {
        if (!ds->has_ck_dst) RET(DDERR_NOCOLORKEY);
        q.use_dk = 1; q.dk_lo = ds->ck_dst.dwColorSpaceLowValue; q.dk_hi = ds->ck_dst.dwColorSpaceHighValue;
        if (q.dk_hi < q.dk_lo) q.dk_hi = q.dk_lo;
    }
    RET(do_blit(ds, &d, ss, &s, &q));
}

static HRESULT WINAPI s_DeleteAttachedSurface(IDirectDrawSurface7 *t, DWORD f, LPDIRECTDRAWSURFACE7 a) { (void)t; (void)f; (void)a; return DDERR_CANNOTDETACHSURFACE; }

static HRESULT WINAPI s_EnumAttachedSurfaces(IDirectDrawSurface7 *t, LPVOID ctx, LPDDENUMSURFACESCALLBACK7 cb)
{
    surf *s = to_surf(t);
    int ver = ((iface_s *)t)->ver;
    if (s->next_flip && s->next_flip != s->root) {
        DDSURFACEDESC2 d;
        memset(&d, 0, sizeof d);
        d.dwSize = ver >= 4 ? sizeof(DDSURFACEDESC2) : sizeof(DDSURFACEDESC);
        surf_desc(s->next_flip, &d);
        surf_addref_all(s->next_flip, 1);
        cb((LPDIRECTDRAWSURFACE7)surf_iface(s->next_flip, ver), &d, ctx);
    }
    return DD_OK;
}

static HRESULT WINAPI s_EnumOverlayZOrders(IDirectDrawSurface7 *t, DWORD f, LPVOID c, LPDDENUMSURFACESCALLBACK7 cb) { (void)t; (void)f; (void)c; (void)cb; return DDERR_NOTAOVERLAYSURFACE; }

static HRESULT WINAPI s_Flip(IDirectDrawSurface7 *t, LPDIRECTDRAWSURFACE7 target, DWORD flags)
{
    surf *s = to_surf(t), *q;
    ddbuf *first;
    LOCK();
    if (!s->next_flip) RET(DDERR_NOTFLIPPABLE);
    pending_poll();
    if (g.npending >= 2 && (flags & DDFLIP_DONOTWAIT)) RET(DDERR_WASSTILLDRAWING);
    if (target) {                                               /* swap with that surface of the chain */
        surf *u = to_surf(target);
        ddbuf *tmp = s->buf;
        if (!u->next_flip) RET(DDERR_NOTFLIPPABLE);
        s->buf = u->buf;
        u->buf = tmp;
    } else {                                                    /* the buffers rotate through the chain, the surfaces stay */
        first = s->buf;
        for (q = s; q->next_flip != s; q = q->next_flip) q->buf = q->next_flip->buf;
        q->buf = first;
    }
    s->unique++;
    g.n_flips++;
    if (s->primary) {
        flush_dirty(s->buf);
        if (!s->buf->handle) RET(DDERR_GENERIC);
        dump_frame(s->buf);
        show_buffer(s->buf, g.hold && !(flags & DDFLIP_NOVSYNC));
    }
    RET(DD_OK);
}

static HRESULT WINAPI s_GetAttachedSurface(IDirectDrawSurface7 *t, LPDDSCAPS2 caps, LPDIRECTDRAWSURFACE7 *out)
{
    surf *s = to_surf(t);
    int ver = ((iface_s *)t)->ver;
    DWORD want = caps->dwCaps;
    if (s->next_flip && (want & (DDSCAPS_BACKBUFFER | DDSCAPS_FLIP))) {
        surf *n = s->next_flip;
        if (n == s->root && (want & DDSCAPS_BACKBUFFER)) return DDERR_NOTFOUND;
        surf_addref_all(n, 1);
        *out = (LPDIRECTDRAWSURFACE7)surf_iface(n, ver);
        return DD_OK;
    }
    return DDERR_NOTFOUND;
}

static HRESULT WINAPI s_GetBltStatus(IDirectDrawSurface7 *t, DWORD f) { (void)t; (void)f; return DD_OK; }

static HRESULT WINAPI s_GetCaps(IDirectDrawSurface7 *t, LPDDSCAPS2 c)
{
    if (((iface_s *)t)->ver >= 4) memset(c, 0, sizeof *c);
    c->dwCaps = to_surf(t)->caps;
    return DD_OK;
}

static HRESULT WINAPI s_GetClipper(IDirectDrawSurface7 *t, LPDIRECTDRAWCLIPPER *c)
{
    surf *s = to_surf(t);
    if (!s->clipper) return DDERR_NOCLIPPERATTACHED;
    s->clipper->vtbl->AddRef((IDirectDrawClipper *)s->clipper);
    *c = (LPDIRECTDRAWCLIPPER)s->clipper;
    return DD_OK;
}

static HRESULT WINAPI s_GetColorKey(IDirectDrawSurface7 *t, DWORD f, LPDDCOLORKEY k)
{
    surf *s = to_surf(t);
    if (f & DDCKEY_SRCBLT) { if (!s->has_ck_src) return DDERR_NOCOLORKEY; *k = s->ck_src; return DD_OK; }
    if (f & DDCKEY_DESTBLT) { if (!s->has_ck_dst) return DDERR_NOCOLORKEY; *k = s->ck_dst; return DD_OK; }
    return DDERR_UNSUPPORTED;
}

/* GetDC: a DIB section, filled from the shadow; ReleaseDC brings what GDI drew back */
static HRESULT WINAPI s_GetDC(IDirectDrawSurface7 *t, HDC *out)
{
    surf *s = to_surf(t);
    ddbuf *b;
    LOCK();
    b = s->buf;
    if (!s->dc) {
        struct { BITMAPINFOHEADER h; DWORD masks[256]; } bi;
        memset(&bi, 0, sizeof bi);
        bi.h.biSize = sizeof bi.h;
        bi.h.biWidth = b->w; bi.h.biHeight = -b->h; bi.h.biPlanes = 1;
        bi.h.biBitCount = (WORD)fmt_bits(b->fmt);
        bi.h.biCompression = BI_RGB;
        if (b->fmt == F_565) {
            bi.h.biCompression = BI_BITFIELDS;
            bi.masks[0] = 0xF800; bi.masks[1] = 0x07E0; bi.masks[2] = 0x001F;
        }
        s->dc = CreateCompatibleDC(NULL);
        s->bmp = CreateDIBSection(s->dc, (BITMAPINFO *)&bi, DIB_RGB_COLORS, &s->dib, NULL, 0);
        if (!s->dc || !s->bmp) RET(DDERR_GENERIC);
        s->oldbmp = (HBITMAP)SelectObject(s->dc, s->bmp);
    }
    if (b->fmt == F_P8) {
        const uint32_t *p = source_palette(s);
        RGBQUAD q[256];
        int i;
        for (i = 0; i < 256; i++) {
            uint32_t c = p ? p[i] : (uint32_t)i * 0x010101u;
            q[i].rgbRed = (BYTE)(c >> 16); q[i].rgbGreen = (BYTE)(c >> 8); q[i].rgbBlue = (BYTE)c; q[i].rgbReserved = 0;
        }
        SetDIBColorTable(s->dc, 0, 256, q);
    }
    GdiFlush();
    memcpy(s->dib, b->pix, (size_t)b->pitch * b->h);
    *out = s->dc;
    RET(DD_OK);
}

static HRESULT WINAPI s_GetFlipStatus(IDirectDrawSurface7 *t, DWORD f)
{
    (void)t;
    LOCK();
    pending_poll();
    if (f & DDGFS_ISFLIPDONE) RET(g.npending ? DDERR_WASSTILLDRAWING : DD_OK);
    RET(g.npending >= 2 ? DDERR_WASSTILLDRAWING : DD_OK);
}

static HRESULT WINAPI s_GetOverlayPosition(IDirectDrawSurface7 *t, LPLONG x, LPLONG y) { (void)t; (void)x; (void)y; return DDERR_NOTAOVERLAYSURFACE; }

static HRESULT WINAPI s_GetPalette(IDirectDrawSurface7 *t, LPDIRECTDRAWPALETTE *p)
{
    surf *s = to_surf(t);
    if (!s->palette) return DDERR_NOPALETTEATTACHED;
    s->palette->vtbl->AddRef((IDirectDrawPalette *)s->palette);
    *p = (LPDIRECTDRAWPALETTE)s->palette;
    return DD_OK;
}

static HRESULT WINAPI s_GetPixelFormat(IDirectDrawSurface7 *t, LPDDPIXELFORMAT pf) { fill_pf(to_surf(t)->buf->fmt, pf); return DD_OK; }
static HRESULT WINAPI s_GetSurfaceDesc(IDirectDrawSurface7 *t, LPDDSURFACEDESC2 d) { surf_desc(to_surf(t), d); return DD_OK; }
static HRESULT WINAPI s_Initialize(IDirectDrawSurface7 *t, LPDIRECTDRAW d, LPDDSURFACEDESC2 sd) { (void)t; (void)d; (void)sd; return DDERR_ALREADYINITIALIZED; }
static HRESULT WINAPI s_IsLost(IDirectDrawSurface7 *t) { (void)t; return DD_OK; }

static HRESULT WINAPI s_Lock(IDirectDrawSurface7 *t, LPRECT r, LPDDSURFACEDESC2 d, DWORD flags, HANDLE ev)
{
    surf *s = to_surf(t);
    ddbuf *b;
    RECT lr;
    (void)ev;
    LOCK();
    b = s->buf;
    if (s->nlocks == MAX_LOCKS) RET(DDERR_SURFACEBUSY);
    if (r) {
        if (r->left < 0 || r->top < 0 || r->right > b->w || r->bottom > b->h || r->left >= r->right || r->top >= r->bottom) RET(DDERR_INVALIDRECT);
        lr = *r;
    } else {
        SetRect(&lr, 0, 0, b->w, b->h);
    }
    surf_desc(s, d);
    d->dwFlags |= DDSD_LPSURFACE;
    d->lpSurface = b->pix + (ptrdiff_t)lr.top * b->pitch + (ptrdiff_t)lr.left * FMT[b->fmt].bytes;
    /* a read-only lock leaves nothing to upload */
    if (flags & DDLOCK_READONLY) SetRect(&lr, 0, 0, 0, 0);
    s->locks[s->nlocks++] = lr;
    RET(DD_OK);
}

static HRESULT WINAPI s_ReleaseDC(IDirectDrawSurface7 *t, HDC dc)
{
    surf *s = to_surf(t);
    ddbuf *b;
    RECT all;
    (void)dc;
    LOCK();
    if (!s->dc) RET(DDERR_NODC);
    b = s->buf;
    GdiFlush();
    memcpy(b->pix, s->dib, (size_t)b->pitch * b->h);
    SetRect(&all, 0, 0, b->w, b->h);
    dirty_add(b, &all);
    s->unique++;
    if (on_display(s)) primary_touched(b);
    RET(DD_OK);
}

static HRESULT WINAPI s_Restore(IDirectDrawSurface7 *t) { (void)t; return DD_OK; }

static HRESULT WINAPI s_SetClipper(IDirectDrawSurface7 *t, LPDIRECTDRAWCLIPPER c)
{
    surf *s = to_surf(t);
    LOCK();
    if (c) IDirectDrawClipper_AddRef(c);
    if (s->clipper) s->clipper->vtbl->Release((IDirectDrawClipper *)s->clipper);
    s->clipper = (clip *)c;
    RET(DD_OK);
}

static HRESULT WINAPI s_SetColorKey(IDirectDrawSurface7 *t, DWORD f, LPDDCOLORKEY k)
{
    surf *s = to_surf(t);
    LOCK();
    if (f & DDCKEY_SRCBLT) {
        s->has_ck_src = k != NULL;
        if (k) { s->ck_src = *k; if (!(f & DDCKEY_COLORSPACE)) s->ck_src.dwColorSpaceHighValue = k->dwColorSpaceLowValue; }
    }
    if (f & DDCKEY_DESTBLT) {
        s->has_ck_dst = k != NULL;
        if (k) { s->ck_dst = *k; if (!(f & DDCKEY_COLORSPACE)) s->ck_dst.dwColorSpaceHighValue = k->dwColorSpaceLowValue; }
    }
    RET(DD_OK);
}

static HRESULT WINAPI s_SetOverlayPosition(IDirectDrawSurface7 *t, LONG x, LONG y) { (void)t; (void)x; (void)y; return DDERR_NOTAOVERLAYSURFACE; }

static HRESULT WINAPI s_SetPalette(IDirectDrawSurface7 *t, LPDIRECTDRAWPALETTE p)
{
    surf *s = to_surf(t);
    LOCK();
    if (p) IDirectDrawPalette_AddRef(p);
    if (s->palette) s->palette->vtbl->Release((IDirectDrawPalette *)s->palette);
    s->palette = (pal *)p;
    if (on_display(s) || s->primary) {
        g.primary_pal = (pal *)p;
        if (p) palette_send((pal *)p, 0, 256);
    }
    RET(DD_OK);
}

static HRESULT WINAPI s_Unlock(IDirectDrawSurface7 *t, LPRECT r)
{
    surf *s = to_surf(t);
    (void)r;
    LOCK();
    if (!s->nlocks) RET(DDERR_NOTLOCKED);
    dirty_add(s->buf, &s->locks[--s->nlocks]);
    s->unique++;
    if (!s->nlocks && on_display(s)) primary_touched(s->buf);
    RET(DD_OK);
}

static HRESULT WINAPI s_UpdateOverlay(IDirectDrawSurface7 *t, LPRECT a, LPDIRECTDRAWSURFACE7 b, LPRECT c, DWORD d, LPDDOVERLAYFX e) { (void)t; (void)a; (void)b; (void)c; (void)d; (void)e; return DDERR_NOTAOVERLAYSURFACE; }
static HRESULT WINAPI s_UpdateOverlayDisplay(IDirectDrawSurface7 *t, DWORD f) { (void)t; (void)f; return DDERR_NOTAOVERLAYSURFACE; }
static HRESULT WINAPI s_UpdateOverlayZOrder(IDirectDrawSurface7 *t, DWORD f, LPDIRECTDRAWSURFACE7 s) { (void)t; (void)f; (void)s; return DDERR_NOTAOVERLAYSURFACE; }

/* IDirectDrawSurface2 */
static HRESULT WINAPI s_GetDDInterface(IDirectDrawSurface7 *t, LPVOID *d)
{
    ddobj *o = to_surf(t)->dd;
    InterlockedIncrement(&o->refs);
    *d = &o->i1;
    return DD_OK;
}
static HRESULT WINAPI s_PageLock(IDirectDrawSurface7 *t, DWORD f) { (void)t; (void)f; return DD_OK; }
static HRESULT WINAPI s_PageUnlock(IDirectDrawSurface7 *t, DWORD f) { (void)t; (void)f; return DD_OK; }

/* IDirectDrawSurface3: a system-memory surface takes over the application's pixels */
static HRESULT WINAPI s_SetSurfaceDesc(IDirectDrawSurface7 *t, LPDDSURFACEDESC2 d, DWORD f)
{
    surf *s = to_surf(t);
    ddbuf *b;
    int fmt;
    (void)f;
    LOCK();
    b = s->buf;
    if (b->handle || s->primary) RET(DDERR_INVALIDOBJECT);
    if (!(d->dwFlags & DDSD_LPSURFACE)) RET(DDERR_INVALIDPARAMS);
    fmt = d->dwFlags & DDSD_PIXELFORMAT ? pf_to_fmt(&d->ddpfPixelFormat) : b->fmt;
    if (fmt < 0) RET(DDERR_INVALIDPIXELFORMAT);
    if (b->own_pix) free(b->pix);
    b->pix = (uint8_t *)d->lpSurface; b->own_pix = 0; b->fmt = fmt;
    if (d->dwFlags & DDSD_WIDTH) b->w = (int)d->dwWidth;
    if (d->dwFlags & DDSD_HEIGHT) b->h = (int)d->dwHeight;
    if (d->dwFlags & DDSD_PITCH) b->pitch = d->lPitch;
    RET(DD_OK);
}

/* IDirectDrawSurface4 */
static HRESULT WINAPI s_SetPrivateData(IDirectDrawSurface7 *t, REFGUID tag, LPVOID data, DWORD size, DWORD f)
{
    surf *s = to_surf(t);
    privdata *p;
    (void)f;
    for (p = s->priv; p; p = p->next) if (IsEqualGUID(&p->tag, tag)) break;
    if (!p) {
        p = (privdata *)calloc(1, sizeof *p);
        if (!p) return DDERR_OUTOFMEMORY;
        p->tag = *tag; p->next = s->priv; s->priv = p;
    }
    free(p->data);
    p->data = malloc(size ? size : 1);
    if (!p->data) return DDERR_OUTOFMEMORY;
    memcpy(p->data, data, size);
    p->size = size;
    return DD_OK;
}
static HRESULT WINAPI s_GetPrivateData(IDirectDrawSurface7 *t, REFGUID tag, LPVOID data, LPDWORD size)
{
    privdata *p;
    for (p = to_surf(t)->priv; p; p = p->next) {
        if (IsEqualGUID(&p->tag, tag)) {
            if (*size < p->size) { *size = p->size; return DDERR_MOREDATA; }
            memcpy(data, p->data, p->size);
            *size = p->size;
            return DD_OK;
        }
    }
    return DDERR_NOTFOUND;
}
static HRESULT WINAPI s_FreePrivateData(IDirectDrawSurface7 *t, REFGUID tag)
{
    surf *s = to_surf(t);
    privdata **pp = &s->priv;
    while (*pp) {
        privdata *p = *pp;
        if (IsEqualGUID(&p->tag, tag)) { *pp = p->next; free(p->data); free(p); return DD_OK; }
        pp = &p->next;
    }
    return DDERR_NOTFOUND;
}
static HRESULT WINAPI s_GetUniquenessValue(IDirectDrawSurface7 *t, LPDWORD v) { *v = to_surf(t)->unique; return DD_OK; }
static HRESULT WINAPI s_ChangeUniquenessValue(IDirectDrawSurface7 *t) { to_surf(t)->unique++; return DD_OK; }

/* IDirectDrawSurface7 */
static HRESULT WINAPI s_SetPriority(IDirectDrawSurface7 *t, DWORD p) { to_surf(t)->priority = p; return DD_OK; }
static HRESULT WINAPI s_GetPriority(IDirectDrawSurface7 *t, LPDWORD p) { *p = to_surf(t)->priority; return DD_OK; }
static HRESULT WINAPI s_SetLOD(IDirectDrawSurface7 *t, DWORD l) { to_surf(t)->lod = l; return DD_OK; }
static HRESULT WINAPI s_GetLOD(IDirectDrawSurface7 *t, LPDWORD l) { *l = to_surf(t)->lod; return DD_OK; }

static const IDirectDrawSurface7Vtbl surf_vtbl = {
    s_QueryInterface, s_AddRef, s_Release, s_AddAttachedSurface, s_AddOverlayDirtyRect, s_Blt, s_BltBatch, s_BltFast, s_DeleteAttachedSurface,
    s_EnumAttachedSurfaces, s_EnumOverlayZOrders, s_Flip, s_GetAttachedSurface, s_GetBltStatus, s_GetCaps, s_GetClipper, s_GetColorKey, s_GetDC,
    s_GetFlipStatus, s_GetOverlayPosition, s_GetPalette, s_GetPixelFormat, s_GetSurfaceDesc, s_Initialize, s_IsLost, s_Lock, s_ReleaseDC, s_Restore,
    s_SetClipper, s_SetColorKey, s_SetOverlayPosition, s_SetPalette, s_Unlock, s_UpdateOverlay, s_UpdateOverlayDisplay, s_UpdateOverlayZOrder,
    s_GetDDInterface, s_PageLock, s_PageUnlock, s_SetSurfaceDesc, s_SetPrivateData, s_GetPrivateData, s_FreePrivateData, s_GetUniquenessValue,
    s_ChangeUniquenessValue, s_SetPriority, s_GetPriority, s_SetLOD, s_GetLOD
};

/* ---------------------------------------------------------------- IDirectDrawGammaControl: folded into the palette in 8-bit modes; the
   scanout has no gamma table, so other modes keep the ramp and show no effect */
static HRESULT WINAPI g_QI(IDirectDrawGammaControl *t, REFIID r, void **o) { iface_g *f = (iface_g *)t; return s_QueryInterface((IDirectDrawSurface7 *)&f->s->f[4], r, o); }
static ULONG WINAPI g_AddRef(IDirectDrawGammaControl *t) { return (ULONG)InterlockedIncrement(&((iface_g *)t)->s->refs); }
static ULONG WINAPI g_Release(IDirectDrawGammaControl *t) { return s_Release((IDirectDrawSurface7 *)&((iface_g *)t)->s->f[4]); }
static HRESULT WINAPI g_GetGammaRamp(IDirectDrawGammaControl *t, DWORD f, LPDDGAMMARAMP r)
{
    surf *s = ((iface_g *)t)->s;
    int i;
    (void)f;
    if (s->has_ramp) { *r = s->ramp; return DD_OK; }
    for (i = 0; i < 256; i++) r->red[i] = r->green[i] = r->blue[i] = (WORD)(i * 257);
    return DD_OK;
}
static HRESULT WINAPI g_SetGammaRamp(IDirectDrawGammaControl *t, DWORD f, LPDDGAMMARAMP r)
{
    surf *s = ((iface_g *)t)->s;
    (void)f;
    LOCK();
    s->ramp = *r;
    s->has_ramp = 1;
    if (s->buf->fmt == F_P8 && g.primary_pal) palette_send(g.primary_pal, 0, 256);
    RET(DD_OK);
}
static const IDirectDrawGammaControlVtbl gamma_vtbl = { g_QI, g_AddRef, g_Release, g_GetGammaRamp, g_SetGammaRamp };

/* ---------------------------------------------------------------- IDirectDraw 1, 2, 4, 7 */
static ddobj *to_dd(void *p) { return ((iface_d *)p)->o; }
static void *dd_iface(ddobj *o, int ver) { return ver == 1 ? (void *)&o->i1 : ver == 2 ? (void *)&o->i2 : ver == 4 ? (void *)&o->i4 : (void *)&o->i7; }

static HRESULT WINAPI d_QueryInterface(void *t, REFIID r, void **out)
{
    ddobj *o = to_dd(t);
    int ver = 0;
    if (IsEqualGUID(r, &IID_IUnknown)) ver = ((iface_d *)t)->ver;
    else if (IsEqualGUID(r, &IID_IDirectDraw)) ver = 1;
    else if (IsEqualGUID(r, &IID_IDirectDraw2)) ver = 2;
    else if (IsEqualGUID(r, &IID_IDirectDraw4)) ver = 4;
    else if (IsEqualGUID(r, &IID_IDirectDraw7)) ver = 7;
    if (!ver) { log_guid("IDirectDraw QueryInterface", r); *out = NULL; return E_NOINTERFACE; }
    InterlockedIncrement(&o->refs);
    *out = dd_iface(o, ver);
    return S_OK;
}

static ULONG WINAPI d_AddRef(void *t) { return (ULONG)InterlockedIncrement(&to_dd(t)->refs); }

static ULONG WINAPI d_Release(void *t)
{
    ddobj *o = to_dd(t);
    LONG n = InterlockedDecrement(&o->refs);
    if (n == 0) {
        LOCK();
        while (o->surfaces) {                                   /* DirectDraw frees what the application left */
            surf *s = o->surfaces;
            surf *root = s->root ? s->root : s;
            if (g.primary == root) g.primary = NULL;
            if (root->root) {
                surf *q = root->next_flip;
                while (q != root) { surf *nx = q->next_flip; surf_free_one(q); q = nx; }
            }
            surf_free_one(root);
        }
        g2e_log("ddraw: hardware copies %llu, hardware fills %llu, CPU-only blits %llu, flips %llu, uploads %llu (%.1f MB)", (unsigned long long)g.n_hw_copy,
                (unsigned long long)g.n_hw_fill, (unsigned long long)g.n_cpu_only, (unsigned long long)g.n_flips, (unsigned long long)g.n_uploads, (double)g.n_upload_bytes / 1e6);
        UNLOCK();
        free(o);
    }
    return (ULONG)n;
}

static HRESULT WINAPI d_Compact(void *t) { (void)t; return DD_OK; }
static HRESULT WINAPI d_CreateClipper(void *t, DWORD f, LPDIRECTDRAWCLIPPER *c, IUnknown *u) { (void)t; (void)f; (void)u; return new_clipper(c); }

static HRESULT WINAPI d_CreatePalette(void *t, DWORD flags, LPPALETTEENTRY e, LPDIRECTDRAWPALETTE *out, IUnknown *u)
{
    pal *p;
    int i;
    (void)t; (void)u;
    if (!(flags & (DDPCAPS_8BIT | DDPCAPS_4BIT | DDPCAPS_2BIT | DDPCAPS_1BIT))) return DDERR_INVALIDPARAMS;
    if (!(flags & DDPCAPS_8BIT)) return DDERR_UNSUPPORTED;
    p = (pal *)calloc(1, sizeof *p);
    if (!p) return DDERR_OUTOFMEMORY;
    p->vtbl = &pal_vtbl; p->refs = 1; p->caps = flags;
    if (e) {
        memcpy(p->e, e, sizeof p->e);
        for (i = 0; i < 256; i++) p->rgb[i] = (uint32_t)e[i].peRed << 16 | (uint32_t)e[i].peGreen << 8 | e[i].peBlue;
    }
    *out = (LPDIRECTDRAWPALETTE)p;
    return DD_OK;
}

static surf *surf_new(ddobj *o, ddbuf *b, DWORD caps)
{
    surf *s = (surf *)calloc(1, sizeof *s);
    static const int vers[5] = { 1, 2, 3, 4, 7 };
    int i;
    if (!s) return NULL;
    for (i = 0; i < 5; i++) { s->f[i].vtbl = &surf_vtbl; s->f[i].s = s; s->f[i].ver = vers[i]; }
    s->gamma.vtbl = &gamma_vtbl; s->gamma.s = s;
    s->dd = o; s->buf = b; s->caps = caps; s->refs = 1;
    s->next = o->surfaces;
    if (o->surfaces) o->surfaces->prev = s;
    o->surfaces = s;
    return s;
}

static void desktop_mode(int *w, int *h, int *fmt)
{
    HDC dc = GetDC(NULL);
    int bpp = GetDeviceCaps(dc, BITSPIXEL);
    ReleaseDC(NULL, dc);
    *w = GetSystemMetrics(SM_CXSCREEN); *h = GetSystemMetrics(SM_CYSCREEN);
    *fmt = bpp_to_fmt(bpp);
    if (*fmt < 0) *fmt = F_X888;
}

static HRESULT create_primary(ddobj *o, const DDSURFACEDESC2 *d, surf **out)
{
    DWORD caps = d->ddsCaps.dwCaps;
    int n = (d->dwFlags & DDSD_BACKBUFFERCOUNT) && (caps & DDSCAPS_FLIP) ? (int)d->dwBackBufferCount : 0, i;
    int w, h, fmt;
    surf *front, *prev;
    ddbuf *b;
    if (g.primary) return DDERR_PRIMARYSURFACEALREADYEXISTS;
    g.windowed = !(g.coop & DDSCL_EXCLUSIVE);
    if (g.windowed) {
        desktop_mode(&w, &h, &fmt);
        if (n) return DDERR_INVALIDCAPS;
    } else {
        if (!g.have_mode) {                                     /* exclusive without SetDisplayMode: the HDMI mode as it is */
            g.disp_w = g.mode_w; g.disp_h = g.mode_h; g.disp_fmt = F_X888; g.have_mode = 1;
        }
        w = g.disp_w; h = g.disp_h; fmt = g.disp_fmt;
    }
    b = buf_new(w, h, fmt, !g.windowed, NULL, 0);
    if (!b) return DDERR_OUTOFMEMORY;
    front = surf_new(o, b, DDSCAPS_PRIMARYSURFACE | DDSCAPS_VIDEOMEMORY | DDSCAPS_LOCALVIDMEM | DDSCAPS_VISIBLE | (n ? DDSCAPS_FRONTBUFFER | DDSCAPS_FLIP | DDSCAPS_COMPLEX : 0));
    if (!front) return DDERR_OUTOFMEMORY;
    front->primary = 1;
    g.primary = front;
    prev = front;
    for (i = 0; i < n; i++) {
        surf *q;
        b = buf_new(w, h, fmt, 1, NULL, 0);
        if (!b) return DDERR_OUTOFVIDEOMEMORY;
        q = surf_new(o, b, DDSCAPS_VIDEOMEMORY | DDSCAPS_LOCALVIDMEM | DDSCAPS_FLIP | DDSCAPS_COMPLEX | (i == 0 ? DDSCAPS_BACKBUFFER : 0));
        if (!q) return DDERR_OUTOFMEMORY;
        q->refs = 0;
        q->root = front;
        prev->next_flip = q;
        prev = q;
    }
    if (n) { prev->next_flip = front; front->root = front; }
    if (!g.windowed) show_buffer(front->buf, 0);
    *out = front;
    return DD_OK;
}

static HRESULT create_offscreen(ddobj *o, const DDSURFACEDESC2 *d, surf **out)
{
    DWORD caps = d->ddsCaps.dwCaps, flags = d->dwFlags;
    int fmt = g.have_mode ? g.disp_fmt : F_X888, n = (flags & DDSD_BACKBUFFERCOUNT) && (caps & DDSCAPS_FLIP) ? (int)d->dwBackBufferCount : 0, i;
    int w = (int)d->dwWidth, h = (int)d->dwHeight, want;
    void *client = flags & DDSD_LPSURFACE ? d->lpSurface : NULL;
    surf *s, *prev;
    ddbuf *b;
    if (!(flags & DDSD_WIDTH) || !(flags & DDSD_HEIGHT) || w <= 0 || h <= 0) return DDERR_INVALIDPARAMS;
    if (flags & DDSD_PIXELFORMAT) {
        fmt = pf_to_fmt(&d->ddpfPixelFormat);
        if (fmt < 0) return DDERR_INVALIDPIXELFORMAT;
    }
    want = !(caps & DDSCAPS_SYSTEMMEMORY) && !client && w <= 2048 && h <= 2048;
    b = buf_new(w, h, fmt, want, client, client ? d->lPitch : 0);
    if (!b) return DDERR_OUTOFMEMORY;
    s = surf_new(o, b, (caps & ~DDSCAPS_SYSTEMMEMORY & ~DDSCAPS_VIDEOMEMORY) | (b->handle ? DDSCAPS_VIDEOMEMORY | DDSCAPS_LOCALVIDMEM : DDSCAPS_SYSTEMMEMORY));
    if (!s) return DDERR_OUTOFMEMORY;
    if (flags & DDSD_CKSRCBLT) { s->has_ck_src = 1; s->ck_src = d->ddckCKSrcBlt; }
    if (flags & DDSD_CKDESTBLT) { s->has_ck_dst = 1; s->ck_dst = d->ddckCKDestBlt; }
    prev = s;
    for (i = 0; i < n; i++) {
        surf *q;
        b = buf_new(w, h, fmt, want, NULL, 0);
        if (!b) return DDERR_OUTOFMEMORY;
        q = surf_new(o, b, caps | (i == 0 ? DDSCAPS_BACKBUFFER : 0));
        if (!q) return DDERR_OUTOFMEMORY;
        q->refs = 0;
        q->root = s;
        prev->next_flip = q;
        prev = q;
    }
    if (n) { prev->next_flip = s; s->root = s; }
    *out = s;
    return DD_OK;
}

static HRESULT WINAPI d_CreateSurface(void *t, LPDDSURFACEDESC2 d, void **out, IUnknown *u)
{
    ddobj *o = to_dd(t);
    int ver = ((iface_d *)t)->ver;
    surf *s = NULL;
    HRESULT hr;
    DWORD caps;
    DDSURFACEDESC2 local;
    (void)u;
    if (!d || !out) return DDERR_INVALIDPARAMS;
    memset(&local, 0, sizeof local);
    memcpy(&local, d, d->dwSize < sizeof local ? d->dwSize : sizeof local);
    caps = local.dwFlags & DDSD_CAPS ? local.ddsCaps.dwCaps : DDSCAPS_OFFSCREENPLAIN;
    local.ddsCaps.dwCaps = caps;
    LOCK();
    if (caps & DDSCAPS_ZBUFFER) RET(DDERR_UNSUPPORTED);         /* no Direct3D in this library */
    if (caps & DDSCAPS_3DDEVICE) RET(DDERR_INVALIDCAPS);
    if (caps & DDSCAPS_OVERLAY) RET(DDERR_NOOVERLAYHW);
    hr = caps & DDSCAPS_PRIMARYSURFACE ? create_primary(o, &local, &s) : create_offscreen(o, &local, &s);
    if (hr == DD_OK) *out = surf_iface(s, ver);
    else g2e_log("ddraw: CreateSurface caps %08lx %lux%lu: %08lx", caps, local.dwWidth, local.dwHeight, (unsigned long)hr);
    RET(hr);
}

static HRESULT WINAPI d_DuplicateSurface(void *t, void *src, void **out)
{
    surf *s = to_surf(src), *q;
    (void)t;
    LOCK();
    if (s->primary) RET(DDERR_CANTDUPLICATE);
    s->buf->refs++;
    q = surf_new(s->dd, s->buf, s->caps);
    if (!q) RET(DDERR_OUTOFMEMORY);
    *out = surf_iface(q, ((iface_s *)src)->ver);
    RET(DD_OK);
}

/* the modes offered: the HDMI modes (1 024 x 768 1:1 centred on 1 280 x 800 on a board B without mode 4), the small ones scaled up */
static const int ENUM_W[] = { 320, 320, 512, 640, 640, 800, 1024, 1280, 1280 }, ENUM_H[] = { 200, 240, 384, 400, 480, 600, 768, 720, 800 };

static HRESULT WINAPI d_EnumDisplayModes(void *t, DWORD f, LPDDSURFACEDESC2 want, LPVOID ctx, LPDDENUMMODESCALLBACK2 cb)
{
    int ver = ((iface_d *)t)->ver, i, b;
    static const int bpps[4] = { 8, 16, 24, 32 };
    (void)f;
    for (b = 0; b < 4; b++) {
        for (i = 0; i < (int)(sizeof ENUM_W / sizeof ENUM_W[0]); i++) {
            DDSURFACEDESC2 d;
            int fmt = bpp_to_fmt(bpps[b]);
            if (want && (want->dwFlags & DDSD_WIDTH) && want->dwWidth != (DWORD)ENUM_W[i]) continue;
            if (want && (want->dwFlags & DDSD_HEIGHT) && want->dwHeight != (DWORD)ENUM_H[i]) continue;
            if (want && (want->dwFlags & DDSD_PIXELFORMAT) && want->ddpfPixelFormat.dwRGBBitCount != (DWORD)bpps[b]) continue;
            memset(&d, 0, sizeof d);
            d.dwSize = ver >= 4 ? sizeof(DDSURFACEDESC2) : sizeof(DDSURFACEDESC);
            d.dwFlags = DDSD_WIDTH | DDSD_HEIGHT | DDSD_PITCH | DDSD_PIXELFORMAT | DDSD_REFRESHRATE;
            d.dwWidth = (DWORD)ENUM_W[i]; d.dwHeight = (DWORD)ENUM_H[i];
            d.lPitch = ((ENUM_W[i] * FMT[fmt].bytes) + 3) & ~3;
            d.dwRefreshRate = 60;
            fill_pf(fmt, &d.ddpfPixelFormat);
            if (cb(&d, ctx) == DDENUMRET_CANCEL) return DD_OK;
        }
    }
    return DD_OK;
}

static HRESULT WINAPI d_EnumSurfaces(void *t, DWORD f, LPDDSURFACEDESC2 want, LPVOID ctx, LPDDENUMSURFACESCALLBACK7 cb)
{
    ddobj *o = to_dd(t);
    int ver = ((iface_d *)t)->ver;
    surf *s;
    (void)want;
    if (!(f & DDENUMSURFACES_DOESEXIST)) return DDERR_INVALIDPARAMS;
    for (s = o->surfaces; s; s = s->next) {
        DDSURFACEDESC2 d;
        memset(&d, 0, sizeof d);
        d.dwSize = ver >= 4 ? sizeof(DDSURFACEDESC2) : sizeof(DDSURFACEDESC);
        surf_desc(s, &d);
        surf_addref_all(s, 1);
        if (cb((LPDIRECTDRAWSURFACE7)surf_iface(s, ver), &d, ctx) == DDENUMRET_CANCEL) break;
    }
    return DD_OK;
}

static HRESULT WINAPI d_FlipToGDISurface(void *t) { (void)t; return DD_OK; }

static HRESULT WINAPI d_GetCaps(void *t, LPDDCAPS drv, LPDDCAPS hel)
{
    DDCAPS c;
    (void)t;
    memset(&c, 0, sizeof c);
    c.dwCaps = DDCAPS_BLT | DDCAPS_BLTCOLORFILL | DDCAPS_BLTSTRETCH | DDCAPS_COLORKEY | DDCAPS_CANBLTSYSMEM | DDCAPS_PALETTE | DDCAPS_PALETTEVSYNC;
    c.dwCaps2 = DDCAPS2_WIDESURFACES | DDCAPS2_CANRENDERWINDOWED;
    c.dwCKeyCaps = DDCKEYCAPS_SRCBLT;
    c.dwFXCaps = DDFXCAPS_BLTMIRRORLEFTRIGHT | DDFXCAPS_BLTMIRRORUPDOWN | DDFXCAPS_BLTSTRETCHX | DDFXCAPS_BLTSTRETCHXN | DDFXCAPS_BLTSTRETCHY |
                 DDFXCAPS_BLTSTRETCHYN | DDFXCAPS_BLTSHRINKX | DDFXCAPS_BLTSHRINKXN | DDFXCAPS_BLTSHRINKY | DDFXCAPS_BLTSHRINKYN;
    c.dwPalCaps = DDPCAPS_8BIT | DDPCAPS_PRIMARYSURFACE;
    c.dwVidMemTotal = c.dwVidMemFree = 256u << 20;
    c.ddsCaps.dwCaps = DDSCAPS_FLIP | DDSCAPS_COMPLEX | DDSCAPS_OFFSCREENPLAIN | DDSCAPS_PRIMARYSURFACE | DDSCAPS_VIDEOMEMORY | DDSCAPS_LOCALVIDMEM |
                       DDSCAPS_SYSTEMMEMORY | DDSCAPS_FRONTBUFFER | DDSCAPS_BACKBUFFER;
    c.dwMaxVisibleOverlays = 0;
    c.dwSVBCaps = c.dwCaps; c.dwSVBCKeyCaps = c.dwCKeyCaps; c.dwSVBFXCaps = c.dwFXCaps;
    if (drv) { DWORD n = drv->dwSize < sizeof c ? drv->dwSize : sizeof c; c.dwSize = n; memcpy(drv, &c, n); }
    if (hel) {                                                  /* the CPU path does the same and more */
        DDCAPS h = c;
        DWORD n = hel->dwSize < sizeof h ? hel->dwSize : sizeof h;
        h.dwCKeyCaps |= DDCKEYCAPS_DESTBLT;
        h.dwSize = n;
        memcpy(hel, &h, n);
    }
    return DD_OK;
}

static HRESULT WINAPI d_GetDisplayMode(void *t, LPDDSURFACEDESC2 d)
{
    DDSURFACEDESC2 m;
    DWORD size = d->dwSize;
    int w, h, fmt;
    (void)t;
    LOCK();
    if (g.have_mode) { w = g.disp_w; h = g.disp_h; fmt = g.disp_fmt; }
    else desktop_mode(&w, &h, &fmt);
    memset(&m, 0, sizeof m);
    m.dwFlags = DDSD_WIDTH | DDSD_HEIGHT | DDSD_PITCH | DDSD_PIXELFORMAT | DDSD_REFRESHRATE;
    m.dwWidth = (DWORD)w; m.dwHeight = (DWORD)h;
    m.lPitch = ((w * FMT[fmt].bytes) + 3) & ~3;
    m.dwRefreshRate = 60;
    fill_pf(fmt, &m.ddpfPixelFormat);
    if (size > sizeof m) size = sizeof m;
    memcpy(d, &m, size);
    d->dwSize = size;
    RET(DD_OK);
}

static HRESULT WINAPI d_GetFourCCCodes(void *t, LPDWORD n, LPDWORD c) { (void)t; (void)c; *n = 0; return DD_OK; }

static HRESULT WINAPI d_GetGDISurface(void *t, void **out)
{
    int ver = ((iface_d *)t)->ver;
    if (!g.primary) return DDERR_NOTFOUND;
    surf_addref_all(g.primary, 1);
    *out = surf_iface(g.primary, ver);
    return DD_OK;
}

static HRESULT WINAPI d_GetMonitorFrequency(void *t, LPDWORD f) { (void)t; *f = (DWORD)(G2_MODE_HZ[g.mode] + 0.5); return DD_OK; }

/* no scanline position from the board: an estimate from the clock, monotonic within a frame */
static double frame_phase(void)
{
    double p = 1.0 / G2_MODE_HZ[g.mode], t = fmod(now_s() - g.vb_t0, p);
    return t / p;
}

static HRESULT WINAPI d_GetScanLine(void *t, LPDWORD line)
{
    double ph = frame_phase();
    int total = g.mode_h + g.mode_h / 20;
    int l = (int)(ph * total);
    (void)t;
    if (l >= g.mode_h) { *line = (DWORD)(g.mode_h - 1); return DDERR_VERTICALBLANKINPROGRESS; }
    *line = (DWORD)l;
    return DD_OK;
}

static HRESULT WINAPI d_GetVerticalBlankStatus(void *t, LPBOOL b) { (void)t; *b = frame_phase() > (double)g.mode_h / (g.mode_h + g.mode_h / 20); return DD_OK; }
static HRESULT WINAPI d_Initialize(void *t, GUID *g_) { (void)t; (void)g_; return DD_OK; }

static HRESULT WINAPI d_RestoreDisplayMode(void *t)
{
    (void)t;
    LOCK();
    ClipCursor(NULL);
    g.have_mode = 0;
    RET(DD_OK);
}

static HRESULT WINAPI d_SetCooperativeLevel(void *t, HWND hwnd, DWORD f)
{
    (void)t;
    LOCK();
    g.hwnd = hwnd;
    g.coop = f;
    g2e_log("ddraw: SetCooperativeLevel hwnd %p flags %08lx", (void *)hwnd, f);
    RET(DD_OK);
}

/* output goes to the HDMI monitor, input stays with the game's window: it gets exactly the requested size at the PC's top-left, and the cursor
   stays inside it as it would on a real full screen of that size. The PC's own display mode is never changed. */
static void fit_window(int w, int h)
{
    if (!g.hwnd || !(g.coop & DDSCL_EXCLUSIVE)) return;
    SetWindowPos(g.hwnd, HWND_TOP, 0, 0, w, h, SWP_SHOWWINDOW | SWP_NOOWNERZORDER);
    { RECT r = { 0, 0, w, h }; ClipCursor(&r); }
}

static HRESULT set_display_mode(int w, int h, int bpp)
{
    int fmt = bpp_to_fmt(bpp);
    if (fmt < 0 || w <= 0 || h <= 0 || w > 1280 || h > 800) return DDERR_INVALIDMODE;
    if (set_hdmi_mode(w, h) < 0) return DDERR_GENERIC;
    g.disp_w = w; g.disp_h = h; g.disp_fmt = fmt; g.have_mode = 1;
    fit_window(w, h);
    g2e_log("ddraw: SetDisplayMode %d x %d x %d -> HDMI mode %d", w, h, bpp, g.mode);
    return DD_OK;
}

static HRESULT WINAPI d1_SetDisplayMode(void *t, DWORD w, DWORD h, DWORD bpp)
{
    (void)t;
    LOCK();
    RET(set_display_mode((int)w, (int)h, (int)bpp));
}

static HRESULT WINAPI d_SetDisplayMode(void *t, DWORD w, DWORD h, DWORD bpp, DWORD refresh, DWORD f)
{
    (void)t; (void)refresh; (void)f;
    LOCK();
    RET(set_display_mode((int)w, (int)h, (int)bpp));
}

/* the next vsync: a PRESENT of what is on display, held, answers at it */
static HRESULT WINAPI d_WaitForVerticalBlank(void *t, DWORD f, HANDLE h)
{
    (void)t; (void)h;
    if (f & DDWAITVB_BLOCKBEGINEVENT) return DDERR_UNSUPPORTED;
    LOCK();
    if (g.shown) {
        g2_event ev;
        g2_ref r;
        while (g.npending) pending_wait_one();
        r = g2_present(g.bd, g.shown, ++g.frame, 0, 1);
        if (g2_wait_event(g.bd, r, &ev, 0.2) == 0) RET(DD_OK);
    }
    Sleep(16);
    RET(DD_OK);
}

static HRESULT WINAPI d_GetAvailableVidMem(void *t, LPDDSCAPS2 c, LPDWORD total, LPDWORD fr)
{
    (void)t; (void)c;
    if (total) *total = 256u << 20;
    if (fr) { uint32_t n = g2_heap_free_bytes(g.heap); *fr = n > (256u << 20) ? 256u << 20 : n; }
    return DD_OK;
}

static HRESULT WINAPI d_GetSurfaceFromDC(void *t, HDC dc, void **out)
{
    ddobj *o = to_dd(t);
    int ver = ((iface_d *)t)->ver;
    surf *s;
    for (s = o->surfaces; s; s = s->next) {
        if (s->dc == dc) {
            surf_addref_all(s, 1);
            *out = surf_iface(s, ver);
            return DD_OK;
        }
    }
    return DDERR_NOTFOUND;
}
static HRESULT WINAPI d_RestoreAllSurfaces(void *t) { (void)t; return DD_OK; }
static HRESULT WINAPI d_TestCooperativeLevel(void *t) { (void)t; return DD_OK; }

static HRESULT WINAPI d_GetDeviceIdentifier(void *t, LPDDDEVICEIDENTIFIER2 id, DWORD f)
{
    int ver = ((iface_d *)t)->ver;
    size_t n = ver >= 7 ? sizeof(DDDEVICEIDENTIFIER2) : sizeof(DDDEVICEIDENTIFIER);
    (void)f;
    memset(id, 0, n);
    strcpy(id->szDriver, "gpu2-ddraw");
    strcpy(id->szDescription, "GPU2 two-board FPGA GPU");
    return DD_OK;
}
static HRESULT WINAPI d_StartModeTest(void *t, LPSIZE s, DWORD n, DWORD f) { (void)t; (void)s; (void)n; (void)f; return DDERR_TESTFINISHED; }
static HRESULT WINAPI d_EvaluateMode(void *t, DWORD f, DWORD *s) { (void)t; (void)f; (void)s; return DDERR_TESTFINISHED; }

#define V(fn) ((void *)(fn))
#define DD_SLOTS(set_mode) \
    .QueryInterface = V(d_QueryInterface), .AddRef = V(d_AddRef), .Release = V(d_Release), .Compact = V(d_Compact), .CreateClipper = V(d_CreateClipper), \
    .CreatePalette = V(d_CreatePalette), .CreateSurface = V(d_CreateSurface), .DuplicateSurface = V(d_DuplicateSurface), \
    .EnumDisplayModes = V(d_EnumDisplayModes), .EnumSurfaces = V(d_EnumSurfaces), .FlipToGDISurface = V(d_FlipToGDISurface), .GetCaps = V(d_GetCaps), \
    .GetDisplayMode = V(d_GetDisplayMode), .GetFourCCCodes = V(d_GetFourCCCodes), .GetGDISurface = V(d_GetGDISurface), \
    .GetMonitorFrequency = V(d_GetMonitorFrequency), .GetScanLine = V(d_GetScanLine), .GetVerticalBlankStatus = V(d_GetVerticalBlankStatus), \
    .Initialize = V(d_Initialize), .RestoreDisplayMode = V(d_RestoreDisplayMode), .SetCooperativeLevel = V(d_SetCooperativeLevel), \
    .SetDisplayMode = V(set_mode), .WaitForVerticalBlank = V(d_WaitForVerticalBlank)

static const IDirectDrawVtbl dd1_vtbl = { DD_SLOTS(d1_SetDisplayMode) };
static const IDirectDraw7Vtbl dd_vtbl = {
    DD_SLOTS(d_SetDisplayMode), .GetAvailableVidMem = V(d_GetAvailableVidMem), .GetSurfaceFromDC = V(d_GetSurfaceFromDC),
    .RestoreAllSurfaces = V(d_RestoreAllSurfaces), .TestCooperativeLevel = V(d_TestCooperativeLevel), .GetDeviceIdentifier = V(d_GetDeviceIdentifier),
    .StartModeTest = V(d_StartModeTest), .EvaluateMode = V(d_EvaluateMode)
};

static ddobj *dd_new(void)
{
    ddobj *o = (ddobj *)calloc(1, sizeof *o);
    if (!o) return NULL;
    o->i1.vtbl = &dd1_vtbl; o->i1.o = o; o->i1.ver = 1;
    o->i2.vtbl = &dd_vtbl; o->i2.o = o; o->i2.ver = 2;
    o->i4.vtbl = &dd_vtbl; o->i4.o = o; o->i4.ver = 4;
    o->i7.vtbl = &dd_vtbl; o->i7.o = o; o->i7.ver = 7;
    o->refs = 1;
    return o;
}

/* ---------------------------------------------------------------- exports */
HRESULT WINAPI DirectDrawCreate(GUID *guid, LPDIRECTDRAW *out, IUnknown *outer)
{
    ddobj *o;
    (void)guid;
    if (outer) return CLASS_E_NOAGGREGATION;
    LOCK();
    if (board_up() < 0) RET(DDERR_NODIRECTDRAWHW);
    o = dd_new();
    if (!o) RET(DDERR_OUTOFMEMORY);
    *out = (LPDIRECTDRAW)&o->i1;
    RET(DD_OK);
}

HRESULT WINAPI DirectDrawCreateEx(GUID *guid, LPVOID *out, REFIID r, IUnknown *outer)
{
    ddobj *o;
    HRESULT hr;
    (void)guid;
    if (outer) return CLASS_E_NOAGGREGATION;
    LOCK();
    if (board_up() < 0) RET(DDERR_NODIRECTDRAWHW);
    o = dd_new();
    if (!o) RET(DDERR_OUTOFMEMORY);
    hr = d_QueryInterface(&o->i7, r, out);
    d_Release(&o->i7);
    RET(hr);
}

HRESULT WINAPI DirectDrawCreateClipper(DWORD f, LPDIRECTDRAWCLIPPER *out, IUnknown *outer) { (void)f; (void)outer; return new_clipper(out); }

HRESULT WINAPI DirectDrawEnumerateA(LPDDENUMCALLBACKA cb, LPVOID ctx) { cb(NULL, (LPSTR)"GPU2 two-board FPGA GPU", (LPSTR)"display", ctx); return DD_OK; }
HRESULT WINAPI DirectDrawEnumerateW(LPDDENUMCALLBACKW cb, LPVOID ctx) { cb(NULL, (LPWSTR)L"GPU2 two-board FPGA GPU", (LPWSTR)L"display", ctx); return DD_OK; }
HRESULT WINAPI DirectDrawEnumerateExA(LPDDENUMCALLBACKEXA cb, LPVOID ctx, DWORD f) { (void)f; cb(NULL, (LPSTR)"GPU2 two-board FPGA GPU", (LPSTR)"display", ctx, NULL); return DD_OK; }
HRESULT WINAPI DirectDrawEnumerateExW(LPDDENUMCALLBACKEXW cb, LPVOID ctx, DWORD f) { (void)f; cb(NULL, (LPWSTR)L"GPU2 two-board FPGA GPU", (LPWSTR)L"display", ctx, NULL); return DD_OK; }

void WINAPI AcquireDDThreadLock(void) { LOCK(); }
void WINAPI ReleaseDDThreadLock(void) { UNLOCK(); }
HRESULT WINAPI SetAppCompatData(DWORD i, DWORD v) { (void)i; (void)v; return DD_OK; }
HRESULT WINAPI GetSurfaceFromDC(HDC dc, LPVOID *out, HDC *ddc) { (void)dc; (void)out; (void)ddc; return DDERR_NOTFOUND; }

/* the image of a surface from the board, in the surface's own row layout (w x h, tight): the test's check of the GPU copy against the shadow */
HRESULT WINAPI GPU2DD_ReadBack(LPDIRECTDRAWSURFACE7 t, void *out)
{
    surf *s = to_surf(t);
    ddbuf *b;
    int r;
    LOCK();
    b = s->buf;
    if (!b->handle) RET(DDERR_INVALIDOBJECT);
    flush_dirty(b);
    if (g2_fence(g.bd, 10.0) < 0) RET(DDERR_GENERIC);
    r = g2_read_image(g.bd, b->handle, (uint32_t)FMT[b->fmt].g2_tf, 0, 0, (uint32_t)b->w, (uint32_t)b->h, 0, 0, (uint32_t)FMT[b->fmt].g2_fmt, 0, 0, out);
    if (b->fmt == F_555) {                                      /* back to 5:5:5 */
        uint16_t *p = (uint16_t *)out;
        size_t i;
        for (i = 0; i < (size_t)b->w * b->h; i++) p[i] = (uint16_t)((p[i] >> 11) << 10 | (p[i] >> 6 & 31) << 5 | (p[i] & 31));
    }
    RET(r < 0 ? DDERR_GENERIC : DD_OK);
}

/* class factory: CoCreateInstance(CLSID_DirectDraw, ...) then IDirectDraw::Initialize */
typedef struct cf { const IClassFactoryVtbl *vtbl; int clipper; } cf;
static HRESULT WINAPI f_QI(IClassFactory *t, REFIID r, void **o)
{
    if (IsEqualGUID(r, &IID_IUnknown) || IsEqualGUID(r, &IID_IClassFactory)) { *o = t; return S_OK; }
    *o = NULL;
    return E_NOINTERFACE;
}
static ULONG WINAPI f_AddRef(IClassFactory *t) { (void)t; return 2; }
static ULONG WINAPI f_Release(IClassFactory *t) { (void)t; return 1; }
static HRESULT WINAPI f_Lock(IClassFactory *t, BOOL b) { (void)t; (void)b; return S_OK; }
static HRESULT WINAPI f_Create(IClassFactory *t, IUnknown *outer, REFIID r, void **out)
{
    cf *c = (cf *)t;
    HRESULT hr;
    if (outer) return CLASS_E_NOAGGREGATION;
    if (c->clipper) {
        LPDIRECTDRAWCLIPPER p;
        hr = new_clipper(&p);
        if (hr != DD_OK) return hr;
        hr = c_QI((IDirectDrawClipper *)p, r, out);
        IDirectDrawClipper_Release(p);
        return hr;
    }
    return DirectDrawCreateEx(NULL, out, r, NULL);
}
static const IClassFactoryVtbl cf_vtbl = { f_QI, f_AddRef, f_Release, f_Create, f_Lock };
static cf cf_dd = { &cf_vtbl, 0 }, cf_clip = { &cf_vtbl, 1 };

HRESULT WINAPI DllGetClassObject(REFCLSID cls, REFIID r, void **out)
{
    if (IsEqualGUID(cls, &CLSID_DirectDraw) || IsEqualGUID(cls, &CLSID_DirectDraw7)) return f_QI((IClassFactory *)&cf_dd, r, out);
    if (IsEqualGUID(cls, &CLSID_DirectDrawClipper)) return f_QI((IClassFactory *)&cf_clip, r, out);
    *out = NULL;
    return CLASS_E_CLASSNOTAVAILABLE;
}

HRESULT WINAPI DllCanUnloadNow(void) { return S_FALSE; }

BOOL WINAPI DllMain(HINSTANCE h, DWORD why, LPVOID r)
{
    (void)r;
    if (why == DLL_PROCESS_ATTACH) {
        DisableThreadLibraryCalls(h);
        InitializeCriticalSection(&g.cs);
    }
    return TRUE;
}
