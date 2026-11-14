/* d3d9_dev.c — IDirect3DDevice9: the swap chain, render targets, state, draws. */
#include "d3d9_internal.h"
#include <math.h>

#define DEV(t) ((d3dev *)(t))
#define MARK_BIT(f, i) do { if (d->rec) d->rec->m.f |= 1u << (i); } while (0)
#define MARK(f) do { if (d->rec) d->rec->m.f = 1; } while (0)

static int g_mode;                                              /* the HDMI mode B runs: one board session per process, its devices share it */

/* ---------------------------------------------------------------- swap chain */
static void swap_free(d3dev *d)
{
    int i;
    g2r_end_pass(d->r);
    for (i = 0; i < 3; i++) {
        if (d->color[i]) g2r_image_destroy(d->r, d->color[i], d->color_base[i]);
        if (d->scan[i]) g2r_image_destroy(d->r, d->scan[i], d->scan_base[i]);
        d->color[i] = d->scan[i] = 0;
    }
}

/* B renders at most 1 280 wide and scans out at most 1 280 x 800: a larger back buffer renders smaller, viewport and scissor scale (as in libGLESv2) */
static int swap_create(d3dev *d, D3DPRESENT_PARAMETERS *pp)
{
    RECT r;
    int w = (int)pp->BackBufferWidth, h = (int)pp->BackBufferHeight, i, maxw = g2e_get_int("GPU2_MAX_W", MAX_RT_W), scan;
    uint32_t fmt;
    if ((!w || !h) && GetClientRect(d->hwnd, &r)) { if (!w) w = r.right; if (!h) h = r.bottom; }
    if (w < 1) w = 640;
    if (h < 1) h = 480;
    pp->BackBufferWidth = (UINT)w; pp->BackBufferHeight = (UINT)h;
    if (pp->BackBufferFormat == D3DFMT_UNKNOWN) pp->BackBufferFormat = D3DFMT_X8R8G8B8;
    if (!pp->BackBufferCount) pp->BackBufferCount = 1;
    d->vw = w; d->vh = h;
    if (maxw > MAX_RT_W || maxw < 1) maxw = MAX_RT_W;
    if (w > maxw || h > MAX_SCAN_H) {
        double s = (double)maxw / w;
        if ((double)MAX_SCAN_H / h < s) s = (double)MAX_SCAN_H / h;
        w = (int)(w * s + 0.5); h = (int)(h * s + 0.5);
        if (w > maxw) w = maxw;
        if (h > MAX_SCAN_H) h = MAX_SCAN_H;
    }
    d->mode = g_mode;                                           /* another device may have changed it */
    if (g2_mode_select(d->bd, w, h, &d->mode) < 0) g2e_log("d3d9: SET_MODE for %d x %d: %s", w, h, g2_error_text(d->bd));
    g_mode = d->mode;
    d->mode_w = G2_MODE_W[d->mode]; d->mode_h = G2_MODE_H[d->mode];
    fmt = pp->BackBufferFormat == D3DFMT_R5G6B5 || pp->BackBufferFormat == D3DFMT_X1R5G5B5 ? G2_FMT_RGB565 : G2_FMT_RGBA8;    /* what B scans out: A1R5G5B5 keeps its alpha in RGBA8 */
    for (i = 0; i < 3; i++) {
        if (g2r_image_create(d->r, &d->color[i], &d->color_base[i], (uint32_t)w, (uint32_t)h, fmt, 1, 1, 0, G2_USE_RT | G2_USE_SCANOUT | G2_USE_TRANSFER | G2_USE_SAMPLED) < 0) { g2e_log("d3d9: no room for the back buffer"); return -1; }
        g2_fill_image(d->bd, d->color[i], G2_TF_RAW, fmt == G2_FMT_RGBA8 ? 0x000000FFu : 0, 0, 0, (uint32_t)w, (uint32_t)h, 0xF, 0, 0);
    }
    scan = w != d->mode_w || h != d->mode_h || pp->SwapEffect == D3DSWAPEFFECT_COPY;
    if (scan) {
        for (i = 0; i < 3; i++) {
            g2r_image_create(d->r, &d->scan[i], &d->scan_base[i], (uint32_t)d->mode_w, (uint32_t)d->mode_h, G2_FMT_RGBA8, 1, 1, 0, G2_USE_SCANOUT | G2_USE_TRANSFER);
            g2_fill_image(d->bd, d->scan[i], G2_TF_RAW, 0x000000FFu, 0, 0, (uint32_t)d->mode_w, (uint32_t)d->mode_h, 0xF, 0, 0);
        }
    }
    if (!d->backbuf) {
        d->backbuf = d3_surface_new(d, (UINT)w, (UINT)h, pp->BackBufferFormat, D3DUSAGE_RENDERTARGET, D3DPOOL_DEFAULT, 0);
        if (!d->backbuf) return -1;
        d->backbuf->back = 1;
        d->backbuf->binds = 1;                                  /* the device's own reference: the application's count starts at 0 */
    }
    d->backbuf->w = (UINT)w; d->backbuf->h = (UINT)h; d->backbuf->fmt = pp->BackBufferFormat;
    d->backbuf->g2fmt = fmt;
    d->back_i = 0;
    d->backbuf->handle = d->color[0];
    g2e_log("d3d9: back buffer %d x %d, rendered %d x %d, HDMI mode %d", d->vw, d->vh, w, h, d->mode);
    return 0;
}

static void ds_alloc(d3dev *d, d3surf *s)
{
    if (g2r_image_create(d->r, &s->handle, &s->base, s->w, s->h, G2_FMT_D24S8, 1, 1, 0, G2_USE_RT | G2_USE_TRANSFER) < 0) g2e_log("d3d9: no room for a depth-stencil surface");
    else g2r_stencil_undefined(d->r, s->handle);
}

static void dump_frame(d3dev *d)
{
    static int every = -1;
    char path[MAX_PATH];
    uint8_t *px;
    FILE *f;
    uint32_t w = d->backbuf->w, h = d->backbuf->h;
    if (!g2e_get("GPU2_DUMP", NULL) || d->backbuf->g2fmt != G2_FMT_RGBA8) return;
    if (every < 0) every = g2e_get_int("GPU2_DUMP_EVERY", 120);
    if (d->frame % (uint32_t)every != 1) return;
    snprintf(path, sizeof path, "%s\\frame_%05u.bmp", g2e_get("GPU2_DUMP", ""), d->frame);
    px = (uint8_t *)malloc((size_t)w * h * 4);
    f = fopen(path, "wb");
    if (px && f) {
        BITMAPFILEHEADER fh;
        BITMAPINFOHEADER ih;
        g2_fence(d->bd, 5.0);
        g2_read_image(d->bd, d->backbuf->handle, G2_TF_BGRA8888, 0, 0, w, h, 0, 0, G2_FMT_RGBA8, 0, 0, px);
        memset(&fh, 0, sizeof fh); memset(&ih, 0, sizeof ih);
        fh.bfType = 0x4D42; fh.bfOffBits = sizeof fh + sizeof ih; fh.bfSize = fh.bfOffBits + w * h * 4;
        ih.biSize = sizeof ih; ih.biWidth = (LONG)w; ih.biHeight = -(LONG)h; ih.biPlanes = 1; ih.biBitCount = 32; ih.biCompression = BI_RGB;     /* row 0 is the top */
        fwrite(&fh, sizeof fh, 1, f); fwrite(&ih, sizeof ih, 1, f); fwrite(px, 4, (size_t)w * h, f);
    }
    if (f) fclose(f);
    free(px);
}

/* the part of the HDMI screen a w x h image fills */
static void fit_rect(const d3dev *d, int w, int h, int *dx, int *dy, int *dw, int *dh)
{
    *dw = d->mode_w; *dh = d->mode_h; *dx = *dy = 0;
    if (!d->stretch) {
        int sx = d->mode_w / w, sy = d->mode_h / h, s = sx < sy ? sx : sy;
        if (s < 1) s = 1;
        *dw = w * s; *dh = h * s;
        if (*dw > d->mode_w) *dw = d->mode_w;
        if (*dh > d->mode_h) *dh = d->mode_h;
        *dx = (d->mode_w - *dw) / 2; *dy = (d->mode_h - *dh) / 2;
    }
}
static void scan_rect(const d3dev *d, int *dx, int *dy, int *dw, int *dh) { fit_rect(d, (int)d->backbuf->w, (int)d->backbuf->h, dx, dy, dw, dh); }

/* the hardware cursor, 64 x 64 colour on B: the application's position is in back buffer pixels, B's in screen pixels */
static void cursor_place(d3dev *d, int x, int y)
{
    int dx, dy, dw, dh;
    if (x == d->cursor_x && y == d->cursor_y) return;
    d->cursor_x = x; d->cursor_y = y;
    scan_rect(d, &dx, &dy, &dw, &dh);
    g2_cursor_pos(d->bd, dx + x * dw / d->vw, dy + y * dh / d->vh);
}
static void cursor_show(d3dev *d)
{
    if (!d->cursor_have) return;
    g2_cursor_show(d->bd, G2_CUR_COLOR, d->cursor_hx, d->cursor_hy, 0, 0, 0, d->cursor_new);
    d->cursor_new = 0;
}
/* the pointer in back buffer pixels; 0: it is not over the window */
static int pointer_at(d3dev *d, int *x, int *y)
{
    POINT p;
    RECT r;
    if (!GetCursorPos(&p) || !ScreenToClient(d->hwnd, &p) || !GetClientRect(d->hwnd, &r) || !PtInRect(&r, p)) return 0;
    *x = p.x * d->vw / r.right; *y = p.y * d->vh / r.bottom;
    return 1;
}

/* the cursor moves at the pointer's rate, not the frame rate: CURSOR_POS is the unsequenced fast path made for that, and a game at 5 fps would otherwise drag it */
static DWORD WINAPI cursor_mover(void *a)
{
    d3dev *d = (d3dev *)a;
    while (!d->cursor_stop) {
        int x, y;
        if (d->cursor_track && pointer_at(d, &x, &y)) cursor_place(d, x, y);
        Sleep(8);
    }
    return 0;
}
static void cursor_track(d3dev *d, int on)
{
    d->cursor_track = on;
    if (on && !d->cursor_thread) d->cursor_thread = CreateThread(NULL, 0, cursor_mover, d, 0, NULL);
}

/* a game that leaves its pointer to Windows: B's cursor shows the same shape, while the game is in front and the pointer over it (d3d9_cursor.c) */
static void cursor_mirror(d3dev *d)
{
    CURSORINFO ci;
    int x, y, take = 0, on;
    if (d->cursor_have) return;                                 /* the application uses Direct3D's cursor */
    ci.cbSize = sizeof ci;
    on = GetCursorInfo(&ci) && (ci.flags & CURSOR_SHOWING) && ci.hCursor && GetForegroundWindow() == GetAncestor(d->hwnd, GA_ROOT) && pointer_at(d, &x, &y);
    if (on && ci.hCursor != d->mirror_cur) {
        uint32_t *w = (uint32_t *)malloc(64 * 64 * 4);
        if (w && d3_cursor_words(ci.hCursor, w, &d->mirror_mode, &d->mirror_hx, &d->mirror_hy) == 0) {
            g2r_end_pass(d->r);
            g2_cursor_shape(d->bd, w, 64 * 64);
            D3T("d3d9: the Windows pointer on HDMI: mode %d, hot (%d, %d)", d->mirror_mode, d->mirror_hx, d->mirror_hy);
            d->mirror_cur = ci.hCursor;
            take = 1;
        } else on = 0;
        free(w);
    }
    if (on && (take || !d->mirror_on)) { g2_cursor_show(d->bd, d->mirror_mode, d->mirror_hx, d->mirror_hy, 0, 0, 0, take); d->mirror_on = 1; d->cursor_x = -1; }
    if (!on && d->mirror_on) { g2_cursor_hide(d->bd); d->mirror_on = 0; }
    if (on) cursor_place(d, x, y);
    cursor_track(d, on);
}

/* output goes to the HDMI monitor, input stays with the game's window: as in ddraw.dll it gets exactly the requested size at the PC's top-left, and the pointer stays inside it as on a real full screen of that size */
static void fit_window(d3dev *d)
{
    RECT r;
    if (d->pp.Windowed || !d->hwnd) return;
    SetWindowPos(d->hwnd, HWND_TOP, 0, 0, d->vw, d->vh, SWP_SHOWWINDOW | SWP_NOOWNERZORDER);
    SetRect(&r, 0, 0, d->vw, d->vh);
    ClipCursor(&r);
}

static HRESULT present(d3dev *d)
{
    uint16_t image = d->color[d->back_i];
    int hold = d->hold && d->pp.PresentationInterval != D3DPRESENT_INTERVAL_IMMEDIATE, w = (int)d->backbuf->w, h = (int)d->backbuf->h;
    g2r_end_pass(d->r);
    dump_frame(d);
    cursor_mirror(d);
    if (d3_rec_on()) d3_rec_present(d);
    if (d->scan[0]) {
        int dw, dh, dx, dy;
        scan_rect(d, &dx, &dy, &dw, &dh);
        d->scan_i = (d->scan_i + 1) % 3;
        g2_copy_image(d->bd, image, d->scan[d->scan_i], 0, 0, (uint32_t)w, (uint32_t)h, (uint32_t)dx, (uint32_t)dy, (uint32_t)dw, (uint32_t)dh, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        g2r_present(d->r, d->scan[d->scan_i], hold);
    } else {
        g2r_present(d->r, image, hold);
    }
    d->frame++;
    g2r_stats_tick(d->r);
    if (d->pp.SwapEffect != D3DSWAPEFFECT_COPY) {
        d->back_i = (d->back_i + 1) % 3;
        d->backbuf->handle = d->color[d->back_i];
        if (d->pp.SwapEffect == D3DSWAPEFFECT_DISCARD) g2r_discard_color(d->r, d->backbuf->handle);      /* undefined after a discard: the next frame needs no band load */
    }
    return D3D_OK;
}

/* ---------------------------------------------------------------- default state */
static DWORD fbits(float f) { DWORD v; memcpy(&v, &f, 4); return v; }

static void state_defaults(d3dev *d)
{
    d3state *s = &d->st;
    int i, j;
    static const D3DMATRIX id = { { { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } } };
    for (i = 0; i < MAX_SAMPLERS + 5; i++) d3_ref_set(&s->tex[i], NULL);
    for (i = 0; i < MAX_STREAMS; i++) d3_ref_set(&s->vb[i], NULL);
    d3_ref_set(&s->ib, NULL); d3_ref_set(&s->decl, NULL); d3_ref_set(&s->vs, NULL); d3_ref_set(&s->ps, NULL);
    memset(s, 0, sizeof *s);
    s->rs[D3DRS_ZENABLE] = d->pp.EnableAutoDepthStencil ? D3DZB_TRUE : D3DZB_FALSE;
    s->rs[D3DRS_FILLMODE] = D3DFILL_SOLID; s->rs[D3DRS_SHADEMODE] = D3DSHADE_GOURAUD; s->rs[D3DRS_ZWRITEENABLE] = TRUE; s->rs[D3DRS_LASTPIXEL] = TRUE;
    s->rs[D3DRS_SRCBLEND] = D3DBLEND_ONE; s->rs[D3DRS_DESTBLEND] = D3DBLEND_ZERO; s->rs[D3DRS_CULLMODE] = D3DCULL_CCW; s->rs[D3DRS_ZFUNC] = D3DCMP_LESSEQUAL; s->rs[D3DRS_ALPHAFUNC] = D3DCMP_ALWAYS;
    s->rs[D3DRS_FOGEND] = fbits(1.0f); s->rs[D3DRS_FOGDENSITY] = fbits(1.0f);
    s->rs[D3DRS_STENCILFAIL] = s->rs[D3DRS_STENCILZFAIL] = s->rs[D3DRS_STENCILPASS] = D3DSTENCILOP_KEEP; s->rs[D3DRS_STENCILFUNC] = D3DCMP_ALWAYS;
    s->rs[D3DRS_STENCILMASK] = s->rs[D3DRS_STENCILWRITEMASK] = 0xFFFFFFFFu; s->rs[D3DRS_TEXTUREFACTOR] = 0xFFFFFFFFu;
    s->rs[D3DRS_CLIPPING] = TRUE; s->rs[D3DRS_LIGHTING] = TRUE; s->rs[D3DRS_COLORVERTEX] = TRUE; s->rs[D3DRS_LOCALVIEWER] = TRUE;
    s->rs[D3DRS_DIFFUSEMATERIALSOURCE] = D3DMCS_COLOR1; s->rs[D3DRS_SPECULARMATERIALSOURCE] = D3DMCS_COLOR2; s->rs[D3DRS_AMBIENTMATERIALSOURCE] = D3DMCS_MATERIAL; s->rs[D3DRS_EMISSIVEMATERIALSOURCE] = D3DMCS_MATERIAL;
    s->rs[D3DRS_POINTSIZE] = fbits(1.0f); s->rs[D3DRS_POINTSIZE_MIN] = fbits(1.0f); s->rs[D3DRS_POINTSIZE_MAX] = fbits(64.0f); s->rs[D3DRS_POINTSCALE_A] = fbits(1.0f);
    s->rs[D3DRS_MULTISAMPLEANTIALIAS] = TRUE; s->rs[D3DRS_MULTISAMPLEMASK] = 0xFFFFFFFFu; s->rs[D3DRS_COLORWRITEENABLE] = 0xF; s->rs[D3DRS_BLENDOP] = D3DBLENDOP_ADD;
    s->rs[D3DRS_CCW_STENCILFAIL] = s->rs[D3DRS_CCW_STENCILZFAIL] = s->rs[D3DRS_CCW_STENCILPASS] = D3DSTENCILOP_KEEP; s->rs[D3DRS_CCW_STENCILFUNC] = D3DCMP_ALWAYS;
    s->rs[D3DRS_COLORWRITEENABLE1] = s->rs[D3DRS_COLORWRITEENABLE2] = s->rs[D3DRS_COLORWRITEENABLE3] = 0xF; s->rs[D3DRS_BLENDFACTOR] = 0xFFFFFFFFu;
    s->rs[D3DRS_SRCBLENDALPHA] = D3DBLEND_ONE; s->rs[D3DRS_DESTBLENDALPHA] = D3DBLEND_ZERO; s->rs[D3DRS_BLENDOPALPHA] = D3DBLENDOP_ADD;
    for (i = 0; i < MAX_STAGES; i++) {
        s->tss[i][D3DTSS_COLOROP] = i ? D3DTOP_DISABLE : D3DTOP_MODULATE;
        s->tss[i][D3DTSS_ALPHAOP] = i ? D3DTOP_DISABLE : D3DTOP_SELECTARG1;
        s->tss[i][D3DTSS_COLORARG1] = s->tss[i][D3DTSS_ALPHAARG1] = D3DTA_TEXTURE;
        s->tss[i][D3DTSS_COLORARG2] = s->tss[i][D3DTSS_ALPHAARG2] = D3DTA_CURRENT;
        s->tss[i][D3DTSS_COLORARG0] = s->tss[i][D3DTSS_ALPHAARG0] = D3DTA_CURRENT;
        s->tss[i][D3DTSS_RESULTARG] = D3DTA_CURRENT;
        s->tss[i][D3DTSS_TEXCOORDINDEX] = (DWORD)i;
        s->texm[i] = id;
    }
    for (i = 0; i < MAX_WM; i++) s->wm[i] = id;
    for (i = 0; i < MAX_STREAMS; i++) s->vb_freq[i] = 1;
    for (i = 0; i < MAX_SAMPLERS + 5; i++) {
        for (j = D3DSAMP_ADDRESSU; j <= D3DSAMP_ADDRESSW; j++) s->ss[i][j] = D3DTADDRESS_WRAP;
        s->ss[i][D3DSAMP_MAGFILTER] = s->ss[i][D3DSAMP_MINFILTER] = D3DTEXF_POINT; s->ss[i][D3DSAMP_MIPFILTER] = D3DTEXF_NONE; s->ss[i][D3DSAMP_MAXANISOTROPY] = 1;
    }
    s->world = s->view = s->proj = id;
    s->vp.Width = d->pp.BackBufferWidth; s->vp.Height = d->pp.BackBufferHeight; s->vp.MaxZ = 1.0f;
    SetRect(&s->sc, 0, 0, (int)d->pp.BackBufferWidth, (int)d->pp.BackBufferHeight);
}

/* ---------------------------------------------------------------- IUnknown and friends */
static HRESULT WINAPI d_QueryInterface(IDirect3DDevice9 *t, REFIID id, void **o)
{
    if (!o) return E_POINTER;
    if (IsEqualGUID(id, &IID_IUnknown) || IsEqualGUID(id, &IID_IDirect3DDevice9)) { IDirect3DDevice9_AddRef(t); *o = t; return S_OK; }
    *o = NULL;
    return E_NOINTERFACE;
}
static ULONG WINAPI d_AddRef(IDirect3DDevice9 *t) { return (ULONG)InterlockedIncrement(&DEV(t)->refs); }
static ULONG WINAPI d_Release(IDirect3DDevice9 *t)
{
    d3dev *d = DEV(t);
    LONG r = InterlockedDecrement(&d->refs);
    if (!r && d->r) { g2r_finish(d->r); swap_free(d); }
    if (!r && d->cursor_thread) { d->cursor_stop = 1; WaitForSingleObject(d->cursor_thread, 1000); CloseHandle(d->cursor_thread); d->cursor_thread = NULL; }
    if (!r && d->bd && (d->cursor_on || d->mirror_on)) g2_cursor_hide(d->bd);
    if (!r && !d->pp.Windowed) ClipCursor(NULL);
    return (ULONG)r;
}
static HRESULT WINAPI d_TestCooperativeLevel(IDirect3DDevice9 *t) { (void)t; return D3D_OK; }
static UINT WINAPI d_GetAvailableTextureMem(IDirect3DDevice9 *t) { (void)t; return 256u << 20; }
static HRESULT WINAPI d_EvictManagedResources(IDirect3DDevice9 *t) { (void)t; return D3D_OK; }
static HRESULT WINAPI d_GetDirect3D(IDirect3DDevice9 *t, IDirect3D9 **o) { *o = DEV(t)->d3d; IDirect3D9_AddRef(*o); return D3D_OK; }
static HRESULT WINAPI d_GetDeviceCaps(IDirect3DDevice9 *t, D3DCAPS9 *c) { (void)t; d3_caps(c); return D3D_OK; }
static HRESULT WINAPI d_GetDisplayMode(IDirect3DDevice9 *t, UINT sw, D3DDISPLAYMODE *m)
{
    d3dev *d = DEV(t);
    (void)sw;
    m->Width = d->pp.Windowed ? (UINT)GetSystemMetrics(SM_CXSCREEN) : d->pp.BackBufferWidth;
    m->Height = d->pp.Windowed ? (UINT)GetSystemMetrics(SM_CYSCREEN) : d->pp.BackBufferHeight;
    m->RefreshRate = 60; m->Format = D3DFMT_X8R8G8B8;
    return D3D_OK;
}
static HRESULT WINAPI d_GetCreationParameters(IDirect3DDevice9 *t, D3DDEVICE_CREATION_PARAMETERS *o)
{
    d3dev *d = DEV(t);
    o->AdapterOrdinal = 0; o->DeviceType = D3DDEVTYPE_HAL; o->hFocusWindow = d->hwnd; o->BehaviorFlags = d->flags;
    return D3D_OK;
}
static HRESULT WINAPI d_SetCursorProperties(IDirect3DDevice9 *t, UINT x, UINT y, IDirect3DSurface9 *s)
{
    d3dev *d = DEV(t);
    d3surf *c = (d3surf *)s;
    uint32_t *w;
    UINT r;
    D3T("d3d9: SetCursorProperties hot (%u, %u), %u x %u format %d", x, y, c ? c->w : 0, c ? c->h : 0, c ? (int)c->fmt : 0);
    if (!c || c->fmt != D3DFMT_A8R8G8B8 || !c->mem || c->w > 64 || c->h > 64 || x >= c->w || y >= c->h) return D3DERR_INVALIDCALL;
    w = (uint32_t *)calloc(64 * 64, 4);
    if (!w) return E_OUTOFMEMORY;
    for (r = 0; r < c->h; r++) memcpy(w + (size_t)r * 64, c->mem + (ptrdiff_t)r * c->pitch, (size_t)c->w * 4);       /* colour mode's words are 0xAARRGGBB, as the surface */
    g2r_end_pass(d->r);
    g2_cursor_shape(d->bd, w, 64 * 64);
    free(w);
    d->cursor_hx = (int)x; d->cursor_hy = (int)y; d->cursor_have = 1; d->cursor_new = 1;
    if (d->mirror_on) d->mirror_on = 0;                         /* the application's cursor replaces the Windows one */
    cursor_track(d, d->cursor_on && !d->cursor_app);
    if (d->cursor_on) cursor_show(d);
    return D3D_OK;
}
static void WINAPI d_SetCursorPosition(IDirect3DDevice9 *t, int x, int y, DWORD f)
{
    d3dev *d = DEV(t);
    (void)f;
    if (d->pp.Windowed) { POINT p; p.x = x; p.y = y; ScreenToClient(d->hwnd, &p); x = p.x; y = p.y; }       /* a window's cursor is in desktop pixels */
    D3T("d3d9: SetCursorPosition (%d, %d)", x, y);
    d->cursor_app = 1;
    cursor_track(d, 0);
    cursor_place(d, x, y);
}
static BOOL WINAPI d_ShowCursor(IDirect3DDevice9 *t, BOOL s)
{
    d3dev *d = DEV(t);
    BOOL o = d->cursor_on;
    D3T("d3d9: ShowCursor %d", s);
    d->cursor_on = s;
    if (s && !o) cursor_show(d);
    if (!s && o && d->cursor_have) g2_cursor_hide(d->bd);
    if (d->cursor_have) cursor_track(d, s && !d->cursor_app);
    return o;
}
static UINT WINAPI d_GetNumberOfSwapChains(IDirect3DDevice9 *t) { (void)t; return 1; }
static HRESULT WINAPI d_Reset(IDirect3DDevice9 *t, D3DPRESENT_PARAMETERS *p)
{
    d3dev *d = DEV(t);
    if (p->MultiSampleType > D3DMULTISAMPLE_NONMASKABLE) return D3DERR_INVALIDCALL;
    d->pp = *p;
    swap_free(d);
    if (d->auto_ds) { d3_unbind(d->auto_ds); d->auto_ds = NULL; }
    if (swap_create(d, &d->pp) < 0) return D3DERR_OUTOFVIDEOMEMORY;
    *p = d->pp;
    if (d->pp.Windowed) ClipCursor(NULL);
    fit_window(d);
    d->cursor_x = d->cursor_y = -1;
    state_defaults(d);
    d3_ref_set(&d->rt, d->backbuf);
    d3_ref_set(&d->ds, NULL);
    if (d->pp.EnableAutoDepthStencil) {
        d->auto_ds = d3_surface_new(d, d->backbuf->w, d->backbuf->h, d->pp.AutoDepthStencilFormat, D3DUSAGE_DEPTHSTENCIL, D3DPOOL_DEFAULT, 0);
        if (d->auto_ds) { d->auto_ds->binds = 1; ds_alloc(d, d->auto_ds); }   /* the device's own, as the back buffer */
        d3_ref_set(&d->ds, d->auto_ds);
    }
    return D3D_OK;
}
static HRESULT WINAPI d_Present(IDirect3DDevice9 *t, const RECT *s, const RECT *o, HWND w, const RGNDATA *r) { (void)s; (void)o; (void)w; (void)r; return present(DEV(t)); }
static HRESULT WINAPI d_GetBackBuffer(IDirect3DDevice9 *t, UINT sw, UINT i, D3DBACKBUFFER_TYPE ty, IDirect3DSurface9 **o)
{
    (void)sw; (void)i; (void)ty;
    *o = (IDirect3DSurface9 *)DEV(t)->backbuf;
    IDirect3DSurface9_AddRef(*o);
    return D3D_OK;
}
static HRESULT WINAPI d_GetRasterStatus(IDirect3DDevice9 *t, UINT sw, D3DRASTER_STATUS *o) { (void)t; (void)sw; o->InVBlank = TRUE; o->ScanLine = 0; return D3D_OK; }
static HRESULT WINAPI d_SetDialogBoxMode(IDirect3DDevice9 *t, BOOL b) { (void)t; (void)b; return D3D_OK; }
static void WINAPI d_SetGammaRamp(IDirect3DDevice9 *t, UINT sw, DWORD f, const D3DGAMMARAMP *r) { (void)t; (void)sw; (void)f; (void)r; }
static void WINAPI d_GetGammaRamp(IDirect3DDevice9 *t, UINT sw, D3DGAMMARAMP *r)
{
    int i;
    (void)t; (void)sw;
    for (i = 0; i < 256; i++) r->red[i] = r->green[i] = r->blue[i] = (WORD)(i * 257);
}

/* ---------------------------------------------------------------- creation */
static HRESULT WINAPI d_CreateTexture(IDirect3DDevice9 *t, UINT w, UINT h, UINT l, DWORD u, D3DFORMAT f, D3DPOOL p, IDirect3DTexture9 **o, HANDLE *sh)
{
    d3tex *x;
    HRESULT r;
    (void)sh;
    r = d3_texture_new(DEV(t), w, h, l, u, f, p, 0, &x);
    D3T("d3d9: CreateTexture %u x %u levels %u usage %08lx format %d pool %d: %08lx", w, h, l, (unsigned long)u, (int)f, (int)p, (unsigned long)r);
    if (r == D3D_OK) *o = (IDirect3DTexture9 *)x;
    return r;
}
static HRESULT WINAPI d_CreateCubeTexture(IDirect3DDevice9 *t, UINT e, UINT l, DWORD u, D3DFORMAT f, D3DPOOL p, IDirect3DCubeTexture9 **o, HANDLE *sh)
{
    d3tex *x;
    HRESULT r;
    (void)sh;
    r = d3_texture_new(DEV(t), e, e, l, u, f, p, 1, &x);
    if (r == D3D_OK) *o = (IDirect3DCubeTexture9 *)x;
    return r;
}
static HRESULT WINAPI d_CreateVolumeTexture(IDirect3DDevice9 *t, UINT w, UINT h, UINT de, UINT l, DWORD u, D3DFORMAT f, D3DPOOL p, IDirect3DVolumeTexture9 **o, HANDLE *sh)
{
    d3tex *x;
    HRESULT r;
    (void)sh;
    r = d3_volume_new(DEV(t), w, h, de, l, u, f, p, &x);
    D3T("d3d9: CreateVolumeTexture %u x %u x %u levels %u usage %08lx format %d pool %d: %08lx", w, h, de, l, (unsigned long)u, (int)f, (int)p, (unsigned long)r);
    if (r == D3D_OK) *o = (IDirect3DVolumeTexture9 *)x;
    return r;
}
static HRESULT new_buf(d3dev *d, UINT n, DWORD u, DWORD fvf, D3DFORMAT f, D3DPOOL p, const void *vtbl, void **o)
{
    d3buf *b = (d3buf *)calloc(1, sizeof *b);
    if (!b) return E_OUTOFMEMORY;
    b->mem = (uint8_t *)calloc(1, (size_t)n + 16);
    if (!b->mem) { free(b); return E_OUTOFMEMORY; }
    b->vtbl = vtbl; b->refs = 1; b->dev = d; b->size = n; b->usage = u; b->fvf = fvf; b->fmt = f; b->pool = p;
    b->keep.watch = 1;
    *o = b;
    return D3D_OK;
}
static HRESULT WINAPI d_CreateVertexBuffer(IDirect3DDevice9 *t, UINT n, DWORD u, DWORD fvf, D3DPOOL p, IDirect3DVertexBuffer9 **o, HANDLE *sh) { (void)sh; return new_buf(DEV(t), n, u, fvf, D3DFMT_VERTEXDATA, p, d3_vt_vb, (void **)o); }
static HRESULT WINAPI d_CreateIndexBuffer(IDirect3DDevice9 *t, UINT n, DWORD u, D3DFORMAT f, D3DPOOL p, IDirect3DIndexBuffer9 **o, HANDLE *sh) { (void)sh; return new_buf(DEV(t), n, u, 0, f, p, d3_vt_ib, (void **)o); }
static HRESULT WINAPI d_CreateRenderTarget(IDirect3DDevice9 *t, UINT w, UINT h, D3DFORMAT f, D3DMULTISAMPLE_TYPE ms, DWORD q, BOOL lk, IDirect3DSurface9 **o, HANDLE *sh)
{
    d3dev *d = DEV(t);
    d3surf *s;
    (void)ms; (void)q; (void)lk; (void)sh;
    D3T("d3d9: CreateRenderTarget %u x %u format %d", w, h, (int)f);
    if (!d3_fmt_rt(f) || w > MAX_RT_W) return D3DERR_INVALIDCALL;
    s = d3_surface_new(d, w, h, f, D3DUSAGE_RENDERTARGET, D3DPOOL_DEFAULT, 0);
    if (!s) return E_OUTOFMEMORY;
    if (g2r_image_create(d->r, &s->handle, &s->base, w, h, s->g2fmt, 1, 1, 0, G2_USE_RT | G2_USE_TRANSFER | G2_USE_SAMPLED) < 0) { IDirect3DSurface9_Release((IDirect3DSurface9 *)s); return D3DERR_OUTOFVIDEOMEMORY; }
    *o = (IDirect3DSurface9 *)s;
    return D3D_OK;
}
static HRESULT WINAPI d_CreateDepthStencilSurface(IDirect3DDevice9 *t, UINT w, UINT h, D3DFORMAT f, D3DMULTISAMPLE_TYPE ms, DWORD q, BOOL dc, IDirect3DSurface9 **o, HANDLE *sh)
{
    d3dev *d = DEV(t);
    d3surf *s;
    (void)ms; (void)q; (void)dc; (void)sh;
    D3T("d3d9: CreateDepthStencilSurface %u x %u format %d", w, h, (int)f);
    if (!d3_fmt_ds(f)) return D3DERR_INVALIDCALL;
    s = d3_surface_new(d, w, h, f, D3DUSAGE_DEPTHSTENCIL, D3DPOOL_DEFAULT, 0);
    if (!s) return E_OUTOFMEMORY;
    ds_alloc(d, s);
    *o = (IDirect3DSurface9 *)s;
    return D3D_OK;
}
static HRESULT WINAPI d_CreateOffscreenPlainSurface(IDirect3DDevice9 *t, UINT w, UINT h, D3DFORMAT f, D3DPOOL p, IDirect3DSurface9 **o, HANDLE *sh)
{
    d3surf *s = d3_surface_new(DEV(t), w, h, f, 0, p, 1);
    (void)sh;
    if (!s) return E_OUTOFMEMORY;
    *o = (IDirect3DSurface9 *)s;
    return D3D_OK;
}

/* ---------------------------------------------------------------- copies between surfaces */
static uint16_t surf_handle(d3surf *s)
{
    if (s->tex) { d3_tex_prepare(s->tex); d3_tex_resolve(s->tex); return s->tex->handle; }
    return s->handle;
}

static HRESULT WINAPI d_UpdateSurface(IDirect3DDevice9 *t, IDirect3DSurface9 *si, const RECT *sr, IDirect3DSurface9 *di, const POINT *dp)
{
    d3surf *s = (d3surf *)si, *o = (d3surf *)di;
    RECT r;
    int b = d3_fmt_bytes(s->fmt), y, x0 = dp ? dp->x : 0, y0 = dp ? dp->y : 0;
    (void)t;
    if (s->fmt != o->fmt || !s->mem || !o->mem) return D3DERR_INVALIDCALL;
    if (sr) r = *sr; else SetRect(&r, 0, 0, (int)s->w, (int)s->h);
    if (!b) {                                                   /* compressed: whole blocks */
        int rows = (r.bottom - r.top + 3) / 4, bytes = (r.right - r.left + 3) / 4 * (s->fmt == D3DFMT_DXT1 ? 8 : 16);
        for (y = 0; y < rows; y++) memcpy(o->mem + (ptrdiff_t)(y0 / 4 + y) * o->pitch + (ptrdiff_t)(x0 / 4) * (s->fmt == D3DFMT_DXT1 ? 8 : 16),
                                          s->mem + (ptrdiff_t)(r.top / 4 + y) * s->pitch + (ptrdiff_t)(r.left / 4) * (s->fmt == D3DFMT_DXT1 ? 8 : 16), (size_t)bytes);
    } else {
        for (y = 0; y < r.bottom - r.top; y++) memcpy(o->mem + (ptrdiff_t)(y0 + y) * o->pitch + (ptrdiff_t)x0 * b, s->mem + (ptrdiff_t)(r.top + y) * s->pitch + (ptrdiff_t)r.left * b, (size_t)(r.right - r.left) * b);
    }
    o->dirty = 1;
    return D3D_OK;
}
static HRESULT WINAPI d_UpdateTexture(IDirect3DDevice9 *t, IDirect3DBaseTexture9 *si, IDirect3DBaseTexture9 *di)
{
    d3tex *s = (d3tex *)si, *o = (d3tex *)di;
    int i;
    (void)t;
    if (s->vol != o->vol || s->layers != o->layers || s->levels < o->levels) return D3DERR_INVALIDCALL;
    if (s->vol) {
        for (i = 0; i < o->levels; i++) {
            d3vol *a = s->lv[i], *b = o->lv[i];
            if (a->slice == b->slice && a->d == b->d) { memcpy(b->mem, a->mem, (size_t)a->slice * a->d); b->dirty = 1; }
        }
        return D3D_OK;
    }
    for (i = 0; i < o->layers * o->levels; i++) {
        d3surf *a = s->surf[i], *b = o->surf[i];
        if (a->pitch == b->pitch && a->h == b->h) { memcpy(b->mem, a->mem, (size_t)d3_shadow_bytes(a)); b->dirty = 1; }
    }
    return D3D_OK;
}
static HRESULT WINAPI d_GetRenderTargetData(IDirect3DDevice9 *t, IDirect3DSurface9 *si, IDirect3DSurface9 *di)
{
    d3surf *s = (d3surf *)si, *o = (d3surf *)di;
    int y;
    D3DLOCKED_RECT l;
    (void)t;
    if (s->w != o->w || s->h != o->h || s->fmt != o->fmt) return D3DERR_INVALIDCALL;
    if (d3_surface_vtbl.LockRect(si, &l, NULL, D3DLOCK_READONLY) != D3D_OK) return D3DERR_INVALIDCALL;
    for (y = 0; y < (int)s->h; y++) memcpy(o->mem + (ptrdiff_t)y * o->pitch, (uint8_t *)l.pBits + (ptrdiff_t)y * l.Pitch, (size_t)s->w * d3_fmt_bytes(s->fmt));
    d3_surface_vtbl.UnlockRect(si);
    o->dirty = 1;
    return D3D_OK;
}
/* what the screen shows, as A8R8G8B8: the scan-out image if the back buffer had to be stretched, else the frame before the back buffer */
HRESULT d3_front_read(d3dev *d, d3surf *o)
{
    int scan = d->scan[0] != 0, y;
    uint16_t img = scan ? d->scan[d->scan_i] : d->color[(d->back_i + 2) % 3];
    uint32_t g = scan ? G2_FMT_RGBA8 : d->backbuf->g2fmt, iw = scan ? (uint32_t)d->mode_w : d->backbuf->w, ih = scan ? (uint32_t)d->mode_h : d->backbuf->h, w = o->w < iw ? o->w : iw, h = o->h < ih ? o->h : ih, i;
    uint8_t *px;
    if (o->fmt != D3DFMT_A8R8G8B8 || !o->mem) return D3DERR_INVALIDCALL;
    px = (uint8_t *)malloc((size_t)w * h * 4);
    if (!px) return E_OUTOFMEMORY;
    g2r_finish(d->r);
    g2_read_image(d->bd, img, g == G2_FMT_RGBA8 ? G2_TF_BGRA8888 : G2_TF_RAW, 0, 0, w, h, 0, 0, g, 0, 0, px);
    if (g != G2_FMT_RGBA8) {                                    /* 5:6:5, from the end: the words are smaller than the result */
        for (i = w * h; i-- > 0; ) {
            uint16_t c = ((uint16_t *)px)[i];
            px[4 * i] = (uint8_t)((c & 31) * 255 / 31); px[4 * i + 1] = (uint8_t)((c >> 5 & 63) * 255 / 63); px[4 * i + 2] = (uint8_t)((c >> 11) * 255 / 31); px[4 * i + 3] = 255;
        }
    }
    for (y = 0; y < (int)h; y++) memcpy(o->mem + (ptrdiff_t)y * o->pitch, px + (size_t)y * w * 4, (size_t)w * 4);
    free(px);
    return D3D_OK;
}

static HRESULT WINAPI d_GetFrontBufferData(IDirect3DDevice9 *t, UINT sw, IDirect3DSurface9 *o) { (void)sw; return d3_front_read(DEV(t), (d3surf *)o); }
static HRESULT WINAPI d_StretchRect(IDirect3DDevice9 *t, IDirect3DSurface9 *si, const RECT *sr, IDirect3DSurface9 *di, const RECT *dr, D3DTEXTUREFILTERTYPE f)
{
    d3dev *d = DEV(t);
    d3surf *s = (d3surf *)si, *o = (d3surf *)di;
    RECT a, b;
    uint16_t hs, hd;
    (void)f;
    if (sr) a = *sr; else SetRect(&a, 0, 0, (int)s->w, (int)s->h);
    if (dr) b = *dr; else SetRect(&b, 0, 0, (int)o->w, (int)o->h);
    if (s->g2fmt != o->g2fmt) { g2e_log("d3d9: StretchRect between formats %u and %u", (unsigned)s->fmt, (unsigned)o->fmt); return D3DERR_INVALIDCALL; }
    if (s->fmt == D3DFMT_D24S8 || d3_fmt_ds(s->fmt)) return D3D_OK;
    g2r_end_pass(d->r);
    hs = surf_handle(s);
    if (o->tex) { d3_tex_prepare(o->tex); d3_tex_resolve(o->tex); }
    hd = o->tex ? o->tex->handle : o->handle;
    g2_copy_image(d->bd, hs, hd, (uint32_t)a.left, (uint32_t)a.top, (uint32_t)(a.right - a.left), (uint32_t)(a.bottom - a.top), (uint32_t)b.left, (uint32_t)b.top, (uint32_t)(b.right - b.left), (uint32_t)(b.bottom - b.top),
                  0, 0, 0, 0, 0, s->tex ? (uint32_t)s->level : 0, s->tex ? (uint32_t)s->face : 0, o->tex ? (uint32_t)o->level : 0, o->tex ? (uint32_t)o->face : 0);
    if (o->tex && o->tex->rt) o->tex->rt_stale = 1;
    return D3D_OK;
}
static HRESULT WINAPI d_ColorFill(IDirect3DDevice9 *t, IDirect3DSurface9 *si, const RECT *r, D3DCOLOR c)
{
    d3dev *d = DEV(t);
    d3surf *s = (d3surf *)si;
    RECT a;
    uint16_t h = s->tex ? surf_handle(s) : s->handle;
    if (r) a = *r; else SetRect(&a, 0, 0, (int)s->w, (int)s->h);
    if (!h) { g2e_log("d3d9: ColorFill of a surface without an image"); return D3D_OK; }
    g2r_end_pass(d->r);
    if (s->g2fmt == G2_FMT_RGBA8) g2_fill_image(d->bd, h, G2_TF_RAW, (c >> 16 & 255) << 24 | (c >> 8 & 255) << 16 | (c & 255) << 8 | c >> 24, (uint32_t)a.left, (uint32_t)a.top, (uint32_t)(a.right - a.left), (uint32_t)(a.bottom - a.top), 0xF, 0, 0);
    else {                                                      /* a 16-bit image: the colour as its word */
        uint8_t rgba[4];
        uint16_t w;
        rgba[0] = (uint8_t)(c >> 16); rgba[1] = (uint8_t)(c >> 8); rgba[2] = (uint8_t)c; rgba[3] = s->fmt == D3DFMT_X1R5G5B5 || s->fmt == D3DFMT_X4R4G4B4 ? 255 : (uint8_t)(c >> 24);
        g2_px16_pack(s->g2fmt, rgba, 1, &w);
        g2_fill_image(d->bd, h, G2_TF_RAW, w, (uint32_t)a.left, (uint32_t)a.top, (uint32_t)(a.right - a.left), (uint32_t)(a.bottom - a.top), 0xF, s->tex ? (uint32_t)s->level : 0, s->tex ? (uint32_t)s->face : 0);
    }
    return D3D_OK;
}

/* ---------------------------------------------------------------- targets */
static HRESULT WINAPI d_SetRenderTarget(IDirect3DDevice9 *t, DWORD i, IDirect3DSurface9 *s)
{
    d3dev *d = DEV(t);
    d3surf *x = (d3surf *)s;
    if (i) return s ? D3DERR_INVALIDCALL : D3D_OK;              /* NumSimultaneousRTs is 1 */
    if (!x) return D3DERR_INVALIDCALL;
    d3_ref_set(&d->rt, x);
    d->st.vp.X = d->st.vp.Y = 0; d->st.vp.MinZ = 0; d->st.vp.MaxZ = 1;
    d->st.vp.Width = x->back ? (DWORD)d->vw : x->vw ? x->vw : x->tex ? x->tex->vw >> x->level : x->w; d->st.vp.Height = x->back ? (DWORD)d->vh : x->vh ? x->vh : x->tex ? x->tex->vh >> x->level : x->h;
    SetRect(&d->st.sc, 0, 0, (int)d->st.vp.Width, (int)d->st.vp.Height);
    return D3D_OK;
}
static HRESULT WINAPI d_GetRenderTarget(IDirect3DDevice9 *t, DWORD i, IDirect3DSurface9 **o)
{
    d3dev *d = DEV(t);
    if (i || !d->rt) return D3DERR_NOTFOUND;
    *o = (IDirect3DSurface9 *)d->rt;
    IDirect3DSurface9_AddRef(*o);
    return D3D_OK;
}
static HRESULT WINAPI d_SetDepthStencilSurface(IDirect3DDevice9 *t, IDirect3DSurface9 *s) { d3_ref_set(&DEV(t)->ds, s); return D3D_OK; }
static HRESULT WINAPI d_GetDepthStencilSurface(IDirect3DDevice9 *t, IDirect3DSurface9 **o)
{
    d3dev *d = DEV(t);
    if (!d->ds) { *o = NULL; return D3DERR_NOTFOUND; }
    *o = (IDirect3DSurface9 *)d->ds;
    IDirect3DSurface9_AddRef(*o);
    return D3D_OK;
}
static HRESULT WINAPI d_BeginScene(IDirect3DDevice9 *t) { DEV(t)->in_scene = 1; return D3D_OK; }
static HRESULT WINAPI d_EndScene(IDirect3DDevice9 *t) { DEV(t)->in_scene = 0; return D3D_OK; }

typedef struct tgt { g2r_target t; int no_alpha; double kx, ky; int vw, vh; } tgt;

int d3_zrev(const d3dev *d)
{
    const d3surf *z = d->ds;
    return d->zrev_ok && z && !z->tex && z->fmt != D3DFMT_D16_LOCKABLE;
}

/* COPY_IMAGE takes colour formats: depth goes by way of the PC, the top-left w x h of one image into the other */
static void ds_copy(d3dev *d, uint16_t from, uint16_t to, uint32_t w, uint32_t h)
{
    uint32_t *px = (uint32_t *)malloc((size_t)w * h * 4);
    if (!px) return;
    g2r_finish(d->r);
    if (g2_read_image(d->bd, from, G2_TF_RAW, 0, 0, w, h, 0, 0, G2_FMT_D24S8, 0, 0, px) >= 0) g2_write_image(d->bd, to, px, (size_t)w * h * 4, G2_TF_RAW, 0, 0, w, h, 0, 0, 0, G2_FMT_D24S8);
    g2r_stencil_written(d->r, to);
    free(px);
}

/* What was drawn through a part goes back to the whole depth surface */
static void ds_settle(d3dev *d, d3surf *z)
{
    if (!z->part_live) return;
    ds_copy(d, z->part, z->handle, z->part_w, z->part_h);
    z->part_live = 0;
}

/* Direct3D lets a target use a larger depth surface, its top-left. B's FLUSH takes one size for both: the rectangle moves to an image of the target's size and back */
static uint16_t ds_part(d3dev *d, d3surf *z, UINT w, UINT h)
{
    if (z->part && (z->part_w != w || z->part_h != h)) {
        ds_settle(d, z);
        g2r_end_pass(d->r);
        g2r_image_destroy(d->r, z->part, z->part_base);
        z->part = 0;
    }
    if (!z->part) {
        if (g2r_image_create(d->r, &z->part, &z->part_base, w, h, G2_FMT_D24S8, 1, 1, 0, G2_USE_RT | G2_USE_TRANSFER) < 0) { g2e_log("d3d9: no room for a depth image of a smaller target"); z->part = 0; return 0; }
        z->part_w = w; z->part_h = h;
    }
    if (!z->part_live) {
        ds_copy(d, z->handle, z->part, w, h);
        z->part_live = 1;
    }
    return z->part;
}

static int bind_target(d3dev *d, tgt *g)
{
    d3surf *c = d->rt, *z = d->ds;
    memset(g, 0, sizeof *g);
    if (!c) return -1;
    g->t.width = c->w; g->t.height = c->h;
    g->vw = (int)c->w; g->vh = (int)c->h;
    g->kx = g->ky = 1.0;
    if (c->back) { g->vw = d->vw; g->vh = d->vh; g->kx = (double)c->w / d->vw; g->ky = (double)c->h / d->vh; }
    if (c->vw) { g->vw = (int)c->vw; g->vh = (int)c->vh; g->kx = (double)c->w / c->vw; g->ky = (double)c->h / c->vh; }
    if (c->tex && !c->level && c->tex->vw != c->w) { g->vw = (int)c->tex->vw; g->vh = (int)c->tex->vh; g->kx = (double)c->w / c->tex->vw; g->ky = (double)c->h / c->tex->vh; }
    if (c->tex) {
        g->t.color = d3_tex_target(c->tex, c->face, c->level, c->w, c->h);
    } else {
        g->t.color = c->handle;
    }
    {
        static const uint8_t bits[4][4] = { { 8, 8, 8, 8 }, { 5, 6, 5, 0 }, { 4, 4, 4, 4 }, { 5, 5, 5, 1 } };
        int f = c->g2fmt == G2_FMT_RGB565 ? 1 : c->g2fmt == G2_FMT_RGBA4444 ? 2 : c->g2fmt == G2_FMT_RGBA5551 ? 3 : 0;
        g->t.bits_r = bits[f][0]; g->t.bits_g = bits[f][1]; g->t.bits_b = bits[f][2]; g->t.bits_a = bits[f][3];
    }
    g->no_alpha = c->fmt == D3DFMT_X8R8G8B8;
    if (z) {
        int d16 = !z->tex && (z->fmt == D3DFMT_D16 || z->fmt == D3DFMT_D16_LOCKABLE);      /* a depth texture holds 24 bits whatever its format says */
        if (z->tex) z->handle = d3_tex_ds(z->tex);
        if (z->w == c->w && z->h == c->h) { ds_settle(d, z); g->t.ds = z->handle; g->t.depth16 = d16; }
        else if (z->w >= c->w && z->h >= c->h) { g->t.ds = ds_part(d, z, c->w, c->h); g->t.depth16 = d16; }
        else { static int said; if (!said++) g2e_log("d3d9: a %u x %u depth surface is smaller than its %u x %u target, which Direct3D does not allow: this draw has no depth", z->w, z->h, c->w, c->h); }
    }
    g2r_set_target(d->r, &g->t);
    return 0;
}

static void clip_rect(const tgt *g, const RECT *r, int *b)
{
    b[0] = r->left < 0 ? 0 : (int)floor(r->left * g->kx);
    b[1] = r->top < 0 ? 0 : (int)floor(r->top * g->ky);
    b[2] = (int)ceil(r->right * g->kx);
    b[3] = (int)ceil(r->bottom * g->ky);
    if (b[2] > (int)g->t.width) b[2] = (int)g->t.width;
    if (b[3] > (int)g->t.height) b[3] = (int)g->t.height;
    if (b[2] < b[0]) b[2] = b[0];
    if (b[3] < b[1]) b[3] = b[1];
}

static HRESULT WINAPI d_Clear(IDirect3DDevice9 *t, DWORD n, const D3DRECT *rects, DWORD flags, D3DCOLOR c, float z, DWORD st)
{
    d3dev *d = DEV(t);
    tgt g;
    double col[4];
    uint32_t fl = 0;
    DWORD i;
    int vb[4];
    RECT vr;
    if (bind_target(d, &g) < 0) return D3DERR_INVALIDCALL;
    if (d3_rec_on()) d3_rec_clear(d, n, rects, flags, c, z, st);
    if (flags & D3DCLEAR_TARGET) fl |= G2_FL_CLEAR_COLOR | G2_FL_CMASK(0xF);
    if ((flags & D3DCLEAR_ZBUFFER) && g.t.ds) fl |= G2_FL_CLEAR_DEPTH;
    if ((flags & D3DCLEAR_STENCIL) && g.t.ds) fl |= G2_FL_CLEAR_STENCIL;
    if (!fl) return D3D_OK;
    col[0] = (c >> 16 & 255) / 255.0; col[1] = (c >> 8 & 255) / 255.0; col[2] = (c & 255) / 255.0; col[3] = g.no_alpha ? 1.0 : (c >> 24) / 255.0;
    SetRect(&vr, (int)d->st.vp.X, (int)d->st.vp.Y, (int)(d->st.vp.X + d->st.vp.Width), (int)(d->st.vp.Y + d->st.vp.Height));    /* Clear stops at the viewport */
    clip_rect(&g, &vr, vb);
    for (i = 0; i < (rects ? n : 1); i++) {
        int b[4];
        uint32_t sc[4];
        if (rects) {
            RECT r;
            SetRect(&r, rects[i].x1, rects[i].y1, rects[i].x2, rects[i].y2);
            clip_rect(&g, &r, b);
            if (b[0] < vb[0]) b[0] = vb[0];
            if (b[1] < vb[1]) b[1] = vb[1];
            if (b[2] > vb[2]) b[2] = vb[2];
            if (b[3] > vb[3]) b[3] = vb[3];
        } else memcpy(b, vb, sizeof b);
        if (d->st.rs[D3DRS_SCISSORTESTENABLE]) {
            int s[4];
            clip_rect(&g, &d->st.sc, s);
            if (b[0] < s[0]) b[0] = s[0];
            if (b[1] < s[1]) b[1] = s[1];
            if (b[2] > s[2]) b[2] = s[2];
            if (b[3] > s[3]) b[3] = s[3];
        }
        if (b[2] <= b[0] || b[3] <= b[1]) continue;
        sc[0] = (uint32_t)b[0]; sc[1] = (uint32_t)b[1]; sc[2] = (uint32_t)b[2]; sc[3] = (uint32_t)b[3];
        {
            uint32_t top = g.t.depth16 ? 0xFFFFu : 0xFFFFFFu, zu = g.t.depth16 ? (uint32_t)(z * 65535.0f + 0.5f) : (uint32_t)((double)z * 16777215.0 + 0.5);
            if (zu > top) zu = top;
            g2r_clear(d->r, fl | G2_FL_CLEAR_SCISSOR, col, d3_zrev(d) ? top - zu : zu, st & 255, 0xFF, sc);    /* reversed depth: the far value is 0 */
        }
    }
    return D3D_OK;
}

/* ---------------------------------------------------------------- state */
static HRESULT WINAPI d_SetTransform(IDirect3DDevice9 *t, D3DTRANSFORMSTATETYPE s, const D3DMATRIX *m)
{
    d3dev *d = DEV(t);
    MARK(xf);
    if (s == D3DTS_VIEW) d->st.view = *m;
    else if (s == D3DTS_PROJECTION) d->st.proj = *m;
    else if (s == D3DTS_WORLD) d->st.world = *m;
    else if (s >= D3DTS_TEXTURE0 && s <= D3DTS_TEXTURE7) d->st.texm[s - D3DTS_TEXTURE0] = *m;
    else if (s > D3DTS_WORLD && s < D3DTS_WORLD + MAX_WM) d->st.wm[s - D3DTS_WORLD] = *m;
    return D3D_OK;
}
static HRESULT WINAPI d_GetTransform(IDirect3DDevice9 *t, D3DTRANSFORMSTATETYPE s, D3DMATRIX *m)
{
    d3dev *d = DEV(t);
    if (s == D3DTS_VIEW) *m = d->st.view;
    else if (s == D3DTS_PROJECTION) *m = d->st.proj;
    else if (s == D3DTS_WORLD) *m = d->st.world;
    else if (s >= D3DTS_TEXTURE0 && s <= D3DTS_TEXTURE7) *m = d->st.texm[s - D3DTS_TEXTURE0];
    else if (s > D3DTS_WORLD && s < D3DTS_WORLD + MAX_WM) *m = d->st.wm[s - D3DTS_WORLD];
    return D3D_OK;
}
static HRESULT WINAPI d_MultiplyTransform(IDirect3DDevice9 *t, D3DTRANSFORMSTATETYPE s, const D3DMATRIX *a)
{
    D3DMATRIX c, r;
    int i, j, k;
    d_GetTransform(t, s, &c);
    for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) { float v = 0; for (k = 0; k < 4; k++) v += a->m[i][k] * c.m[k][j]; r.m[i][j] = v; }
    return d_SetTransform(t, s, &r);
}
static HRESULT WINAPI d_SetViewport(IDirect3DDevice9 *t, const D3DVIEWPORT9 *v) { d3dev *d = DEV(t); MARK(vp); d->st.vp = *v; return D3D_OK; }
static HRESULT WINAPI d_GetViewport(IDirect3DDevice9 *t, D3DVIEWPORT9 *v) { *v = DEV(t)->st.vp; return D3D_OK; }
static HRESULT WINAPI d_SetMaterial(IDirect3DDevice9 *t, const D3DMATERIAL9 *m) { d3dev *d = DEV(t); MARK(mat); d->st.mat = *m; return D3D_OK; }
static HRESULT WINAPI d_GetMaterial(IDirect3DDevice9 *t, D3DMATERIAL9 *m) { *m = DEV(t)->st.mat; return D3D_OK; }
static HRESULT WINAPI d_SetLight(IDirect3DDevice9 *t, DWORD i, const D3DLIGHT9 *l) { d3dev *d = DEV(t); MARK(light); if (i >= MAX_LIGHTS) return D3D_OK; d->st.light[i] = *l; return D3D_OK; }
static HRESULT WINAPI d_GetLight(IDirect3DDevice9 *t, DWORD i, D3DLIGHT9 *l) { if (i >= MAX_LIGHTS) return D3DERR_INVALIDCALL; *l = DEV(t)->st.light[i]; return D3D_OK; }
static HRESULT WINAPI d_LightEnable(IDirect3DDevice9 *t, DWORD i, BOOL b) { d3dev *d = DEV(t); MARK(light); if (i < MAX_LIGHTS) d->st.light_on[i] = b; return D3D_OK; }
static HRESULT WINAPI d_GetLightEnable(IDirect3DDevice9 *t, DWORD i, BOOL *b) { if (i >= MAX_LIGHTS) return D3DERR_INVALIDCALL; *b = DEV(t)->st.light_on[i]; return D3D_OK; }
static HRESULT WINAPI d_SetClipPlane(IDirect3DDevice9 *t, DWORD i, const float *p) { d3dev *d = DEV(t); MARK(clip); if (i >= 6) return D3DERR_INVALIDCALL; memcpy(d->st.clip[i], p, 16); return D3D_OK; }
static HRESULT WINAPI d_GetClipPlane(IDirect3DDevice9 *t, DWORD i, float *p) { if (i >= 6) return D3DERR_INVALIDCALL; memcpy(p, DEV(t)->st.clip[i], 16); return D3D_OK; }
static HRESULT WINAPI d_SetRenderState(IDirect3DDevice9 *t, D3DRENDERSTATETYPE s, DWORD v)
{
    d3dev *d = DEV(t);
    if ((unsigned)s >= MAX_RS) return D3DERR_INVALIDCALL;
    d->st.rs[s] = v;
    if (d->rec) d->rec->m.rs[s / 32] |= 1u << (s % 32);
    return D3D_OK;
}
static HRESULT WINAPI d_GetRenderState(IDirect3DDevice9 *t, D3DRENDERSTATETYPE s, DWORD *v) { if ((unsigned)s >= MAX_RS) return D3DERR_INVALIDCALL; *v = DEV(t)->st.rs[s]; return D3D_OK; }
static HRESULT WINAPI d_CreateStateBlock(IDirect3DDevice9 *t, D3DSTATEBLOCKTYPE ty, IDirect3DStateBlock9 **o) { d3block *b; HRESULT r; (void)ty; r = d3_block_new(DEV(t), 1, &b); if (r == D3D_OK) *o = (IDirect3DStateBlock9 *)b; return r; }
static HRESULT WINAPI d_BeginStateBlock(IDirect3DDevice9 *t) { d3dev *d = DEV(t); if (d->rec) return D3DERR_INVALIDCALL; return d3_block_new(d, 0, &d->rec); }
static HRESULT WINAPI d_EndStateBlock(IDirect3DDevice9 *t, IDirect3DStateBlock9 **o)
{
    d3dev *d = DEV(t);
    if (!d->rec) return D3DERR_INVALIDCALL;
    d3_block_capture(d->rec);
    *o = (IDirect3DStateBlock9 *)d->rec;
    d->rec = NULL;
    return D3D_OK;
}
static HRESULT WINAPI d_SetClipStatus(IDirect3DDevice9 *t, const D3DCLIPSTATUS9 *s) { (void)t; (void)s; return D3D_OK; }
static HRESULT WINAPI d_GetClipStatus(IDirect3DDevice9 *t, D3DCLIPSTATUS9 *s) { (void)t; memset(s, 0, sizeof *s); return D3D_OK; }
static int sampler_i(DWORD s) { return s >= D3DDMAPSAMPLER ? MAX_SAMPLERS + (int)(s - D3DDMAPSAMPLER) : (int)s; }
static HRESULT WINAPI d_GetTexture(IDirect3DDevice9 *t, DWORD s, IDirect3DBaseTexture9 **o)
{
    d3tex *x = DEV(t)->st.tex[sampler_i(s)];
    *o = (IDirect3DBaseTexture9 *)x;
    if (x) IDirect3DBaseTexture9_AddRef(*o);
    return D3D_OK;
}
static HRESULT WINAPI d_SetTexture(IDirect3DDevice9 *t, DWORD s, IDirect3DBaseTexture9 *x)
{
    d3dev *d = DEV(t);
    int i = sampler_i(s);
    if (i >= MAX_SAMPLERS + 5) return D3DERR_INVALIDCALL;
    MARK_BIT(tex, i);
    d3_ref_set(&d->st.tex[i], x);
    return D3D_OK;
}
static HRESULT WINAPI d_GetTextureStageState(IDirect3DDevice9 *t, DWORD st, D3DTEXTURESTAGESTATETYPE ty, DWORD *v) { if (st >= MAX_STAGES || (unsigned)ty >= MAX_TSS) return D3DERR_INVALIDCALL; *v = DEV(t)->st.tss[st][ty]; return D3D_OK; }
static HRESULT WINAPI d_SetTextureStageState(IDirect3DDevice9 *t, DWORD st, D3DTEXTURESTAGESTATETYPE ty, DWORD v)
{
    d3dev *d = DEV(t);
    if (st >= MAX_STAGES || (unsigned)ty >= MAX_TSS) return D3D_OK;
    d->st.tss[st][ty] = v;
    if (d->rec) d->rec->m.tss[st] |= 1u << ty;
    return D3D_OK;
}
static HRESULT WINAPI d_GetSamplerState(IDirect3DDevice9 *t, DWORD s, D3DSAMPLERSTATETYPE ty, DWORD *v) { *v = DEV(t)->st.ss[sampler_i(s)][ty]; return D3D_OK; }
static HRESULT WINAPI d_SetSamplerState(IDirect3DDevice9 *t, DWORD s, D3DSAMPLERSTATETYPE ty, DWORD v)
{
    d3dev *d = DEV(t);
    int i = sampler_i(s);
    if (i >= MAX_SAMPLERS + 5 || (unsigned)ty >= MAX_SS) return D3D_OK;
    d->st.ss[i][ty] = v;
    if (d->rec) d->rec->m.ss[i] |= 1u << ty;
    return D3D_OK;
}
static HRESULT WINAPI d_ValidateDevice(IDirect3DDevice9 *t, DWORD *n) { (void)t; *n = 1; return D3D_OK; }
static HRESULT WINAPI d_SetPaletteEntries(IDirect3DDevice9 *t, UINT n, const PALETTEENTRY *e) { (void)t; (void)n; (void)e; return D3D_OK; }
static HRESULT WINAPI d_GetPaletteEntries(IDirect3DDevice9 *t, UINT n, PALETTEENTRY *e) { (void)t; (void)n; (void)e; return D3DERR_INVALIDCALL; }
static HRESULT WINAPI d_SetCurrentTexturePalette(IDirect3DDevice9 *t, UINT n) { (void)t; (void)n; return D3D_OK; }
static HRESULT WINAPI d_GetCurrentTexturePalette(IDirect3DDevice9 *t, UINT *n) { (void)t; *n = 0; return D3D_OK; }
static HRESULT WINAPI d_SetScissorRect(IDirect3DDevice9 *t, const RECT *r) { d3dev *d = DEV(t); MARK(sc); d->st.sc = *r; return D3D_OK; }
static HRESULT WINAPI d_GetScissorRect(IDirect3DDevice9 *t, RECT *r) { *r = DEV(t)->st.sc; return D3D_OK; }
static HRESULT WINAPI d_SetSoftwareVertexProcessing(IDirect3DDevice9 *t, BOOL b) { (void)t; (void)b; return D3D_OK; }
static BOOL WINAPI d_GetSoftwareVertexProcessing(IDirect3DDevice9 *t) { (void)t; return FALSE; }
static HRESULT WINAPI d_SetNPatchMode(IDirect3DDevice9 *t, float n) { (void)t; (void)n; return D3D_OK; }
static float WINAPI d_GetNPatchMode(IDirect3DDevice9 *t) { (void)t; return 0.0f; }

/* ---------------------------------------------------------------- vertex formats */
static void el_add(d3decl *d, WORD off, BYTE type, BYTE usage, BYTE index)
{
    D3DVERTEXELEMENT9 *e = &d->el[d->n++];
    e->Stream = 0; e->Offset = off; e->Type = type; e->Method = D3DDECLMETHOD_DEFAULT; e->Usage = usage; e->UsageIndex = index;
}

static d3decl *decl_end(d3dev *dv, d3decl *d)
{
    static const D3DVERTEXELEMENT9 end = D3DDECL_END();
    d->vtbl = &d3_decl_vtbl; d->refs = 1; d->dev = dv;
    d->el[d->n] = end;
    return d;
}

static d3decl *fvf_decl(d3dev *dv, DWORD fvf)
{
    int i, n, h = (int)((fvf ^ fvf >> 6 ^ fvf >> 16) & 63);
    d3decl *d = dv->fvf_decl[h];
    WORD o = 0;
    DWORD pos = fvf & D3DFVF_POSITION_MASK;
    if (d && d->fvf == fvf) return d;
    d = (d3decl *)calloc(1, sizeof *d);
    if (!d) return NULL;
    d->fvf = fvf;
    switch (pos) {
    case D3DFVF_XYZRHW: el_add(d, 0, D3DDECLTYPE_FLOAT4, D3DDECLUSAGE_POSITIONT, 0); o = 16; break;
    case D3DFVF_XYZW: el_add(d, 0, D3DDECLTYPE_FLOAT4, D3DDECLUSAGE_POSITION, 0); o = 16; break;
    default:
        el_add(d, 0, D3DDECLTYPE_FLOAT3, D3DDECLUSAGE_POSITION, 0);
        o = 12;
        if (pos >= D3DFVF_XYZB1 && pos <= D3DFVF_XYZB5) {       /* 1 .. 5 floats, the last may be the matrix indices */
            int betas = (int)(pos - D3DFVF_XYZRHW) / 2, w = fvf & (D3DFVF_LASTBETA_UBYTE4 | D3DFVF_LASTBETA_D3DCOLOR) ? betas - 1 : betas;
            if (w) el_add(d, o, (BYTE)(D3DDECLTYPE_FLOAT1 + (w > 4 ? 3 : w - 1)), D3DDECLUSAGE_BLENDWEIGHT, 0);
            o = (WORD)(o + 4 * w);
            if (w != betas) { el_add(d, o, fvf & D3DFVF_LASTBETA_UBYTE4 ? D3DDECLTYPE_UBYTE4 : D3DDECLTYPE_D3DCOLOR, D3DDECLUSAGE_BLENDINDICES, 0); o = (WORD)(o + 4); }
        }
        break;
    }
    if (fvf & D3DFVF_NORMAL) { el_add(d, o, D3DDECLTYPE_FLOAT3, D3DDECLUSAGE_NORMAL, 0); o = (WORD)(o + 12); }
    if (fvf & D3DFVF_PSIZE) { el_add(d, o, D3DDECLTYPE_FLOAT1, D3DDECLUSAGE_PSIZE, 0); o = (WORD)(o + 4); }
    if (fvf & D3DFVF_DIFFUSE) { el_add(d, o, D3DDECLTYPE_D3DCOLOR, D3DDECLUSAGE_COLOR, 0); o = (WORD)(o + 4); }
    if (fvf & D3DFVF_SPECULAR) { el_add(d, o, D3DDECLTYPE_D3DCOLOR, D3DDECLUSAGE_COLOR, 1); o = (WORD)(o + 4); }
    n = (int)((fvf & D3DFVF_TEXCOUNT_MASK) >> D3DFVF_TEXCOUNT_SHIFT);
    for (i = 0; i < n; i++) {
        static const BYTE T[4] = { D3DDECLTYPE_FLOAT2, D3DDECLTYPE_FLOAT3, D3DDECLTYPE_FLOAT4, D3DDECLTYPE_FLOAT1 };
        static const BYTE S[4] = { 8, 12, 16, 4 };
        int f = (int)(fvf >> (16 + 2 * i) & 3);
        el_add(d, o, T[f], D3DDECLUSAGE_TEXCOORD, (BYTE)i);
        o = (WORD)(o + S[f]);
    }
    decl_end(dv, d);
    if (dv->fvf_decl[h]) d3_unbind(dv->fvf_decl[h]);
    d->binds = 1;                                               /* the cache's own */
    dv->fvf_decl[h] = d;
    return d;
}

static HRESULT WINAPI d_CreateVertexDeclaration(IDirect3DDevice9 *t, const D3DVERTEXELEMENT9 *e, IDirect3DVertexDeclaration9 **o)
{
    D3T("d3d9: CreateVertexDeclaration");
    d3decl *d = (d3decl *)calloc(1, sizeof *d);
    if (!d) return E_OUTOFMEMORY;
    while (e[d->n].Stream != 0xFF && d->n < 64) { d->el[d->n] = e[d->n]; d->n++; }
    decl_end(DEV(t), d);
    *o = (IDirect3DVertexDeclaration9 *)d;
    return D3D_OK;
}
static HRESULT WINAPI d_SetVertexDeclaration(IDirect3DDevice9 *t, IDirect3DVertexDeclaration9 *de) { d3dev *d = DEV(t); MARK(decl); MARK(fvf); d3_ref_set(&d->st.decl, de); d->st.fvf = 0; return D3D_OK; }
static HRESULT WINAPI d_GetVertexDeclaration(IDirect3DDevice9 *t, IDirect3DVertexDeclaration9 **o) { d3dev *d = DEV(t); *o = (IDirect3DVertexDeclaration9 *)d->st.decl; if (*o) IDirect3DVertexDeclaration9_AddRef(*o); return D3D_OK; }
static HRESULT WINAPI d_SetFVF(IDirect3DDevice9 *t, DWORD f) { d3dev *d = DEV(t); MARK(decl); MARK(fvf); d->st.fvf = f; if (f) d3_ref_set(&d->st.decl, fvf_decl(d, f)); return D3D_OK; }
static HRESULT WINAPI d_GetFVF(IDirect3DDevice9 *t, DWORD *f) { *f = DEV(t)->st.fvf; return D3D_OK; }

/* ---------------------------------------------------------------- shaders */
static HRESULT new_shader(d3dev *d, const DWORD *tok, int ps, const void *vtbl, void **o)
{
    d3shader *s = (d3shader *)calloc(1, sizeof *s);
    UINT n = 1;
    if (!s) return E_OUTOFMEMORY;
    while (tok[n] != 0x0000FFFF) {                              /* the end token; comments carry their length, other tokens are never equal to it */
        if ((tok[n] & 0xFFFF) == 0xFFFE) n += tok[n] >> 16;
        n++;
    }
    n++;
    s->tok = (DWORD *)malloc((size_t)n * 4);
    if (!s->tok) { free(s); return E_OUTOFMEMORY; }
    memcpy(s->tok, tok, (size_t)n * 4);
    {
        static LONG next_id;
        s->id = (uint32_t)InterlockedIncrement(&next_id);
    }
    s->ntok = n; s->vtbl = vtbl; s->refs = 1; s->dev = d; s->ps = ps;
    if (d3_sm_parse(s) < 0) {
        UINT i;
        g2e_log("d3d9: %s shader not translated: %s", ps ? "pixel" : "vertex", s->error);
        for (i = 0; d3_trace() && i < n; i += 8) g2e_log("d3d9: tokens %u: %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx", i, (unsigned long)tok[i], (unsigned long)(i + 1 < n ? tok[i + 1] : 0), (unsigned long)(i + 2 < n ? tok[i + 2] : 0), (unsigned long)(i + 3 < n ? tok[i + 3] : 0),
                                                         (unsigned long)(i + 4 < n ? tok[i + 4] : 0), (unsigned long)(i + 5 < n ? tok[i + 5] : 0), (unsigned long)(i + 6 < n ? tok[i + 6] : 0), (unsigned long)(i + 7 < n ? tok[i + 7] : 0));
    }
    /* a vertex shader's position: whether its z and w were followed to two rows of constants (reversed depth gets z' = w - z exactly then, d3d9_sm.c pos_scan) */
    if (ps || s->error[0]) D3T("d3d9: Create%sShader %d.%d, %u tokens, %s", ps ? "Pixel" : "Vertex", s->major, s->minor, n, s->error[0] ? s->error : "translated");
    else if (s->pos_zk >= 0) D3T("d3d9: CreateVertexShader %d.%d, %u tokens, translated, position z = c%d, w = c%d", s->major, s->minor, n, s->pos_zk, s->pos_wk);
    else D3T("d3d9: CreateVertexShader %d.%d, %u tokens, translated, position not followed", s->major, s->minor, n);
    *o = s;
    return D3D_OK;
}
static HRESULT WINAPI d_CreateVertexShader(IDirect3DDevice9 *t, const DWORD *f, IDirect3DVertexShader9 **o) { return new_shader(DEV(t), f, 0, &d3_vs_vtbl, (void **)o); }
static HRESULT WINAPI d_CreatePixelShader(IDirect3DDevice9 *t, const DWORD *f, IDirect3DPixelShader9 **o) { return new_shader(DEV(t), f, 1, &d3_ps_vtbl, (void **)o); }
static HRESULT WINAPI d_SetVertexShader(IDirect3DDevice9 *t, IDirect3DVertexShader9 *s) { d3dev *d = DEV(t); MARK(vs); d3_ref_set(&d->st.vs, s); return D3D_OK; }
static HRESULT WINAPI d_GetVertexShader(IDirect3DDevice9 *t, IDirect3DVertexShader9 **o) { *o = (IDirect3DVertexShader9 *)DEV(t)->st.vs; if (*o) IDirect3DVertexShader9_AddRef(*o); return D3D_OK; }
static HRESULT WINAPI d_SetPixelShader(IDirect3DDevice9 *t, IDirect3DPixelShader9 *s) { d3dev *d = DEV(t); MARK(ps); d3_ref_set(&d->st.ps, s); return D3D_OK; }
static HRESULT WINAPI d_GetPixelShader(IDirect3DDevice9 *t, IDirect3DPixelShader9 **o) { *o = (IDirect3DPixelShader9 *)DEV(t)->st.ps; if (*o) IDirect3DPixelShader9_AddRef(*o); return D3D_OK; }
#define CONSTS(name, field, T, per, limit, mk) \
static HRESULT WINAPI d_Set##name(IDirect3DDevice9 *t, UINT r, const T *v, UINT n) { d3dev *d = DEV(t); MARK(mk); if (r + n > limit) return D3DERR_INVALIDCALL; memcpy(&d->st.field[r], v, (size_t)n * per * sizeof(T)); return D3D_OK; } \
static HRESULT WINAPI d_Get##name(IDirect3DDevice9 *t, UINT r, T *v, UINT n) { d3dev *d = DEV(t); if (r + n > limit) return D3DERR_INVALIDCALL; memcpy(v, &d->st.field[r], (size_t)n * per * sizeof(T)); return D3D_OK; }
CONSTS(VertexShaderConstantF, vc, float, 4, VS_CONSTS, vc)
CONSTS(VertexShaderConstantI, vi, int, 4, 16, vc)
CONSTS(VertexShaderConstantB, vb_c, BOOL, 1, 16, vc)
CONSTS(PixelShaderConstantF, pc, float, 4, PS_CONSTS, pc)
CONSTS(PixelShaderConstantI, pi, int, 4, 16, pc)
CONSTS(PixelShaderConstantB, pb_c, BOOL, 1, 16, pc)

/* ---------------------------------------------------------------- streams */
static HRESULT WINAPI d_SetStreamSource(IDirect3DDevice9 *t, UINT i, IDirect3DVertexBuffer9 *b, UINT off, UINT stride)
{
    d3dev *d = DEV(t);
    if (i >= MAX_STREAMS) return D3DERR_INVALIDCALL;
    MARK_BIT(vb, i);
    d3_ref_set(&d->st.vb[i], b); d->st.vb_off[i] = off; d->st.vb_stride[i] = stride;
    return D3D_OK;
}
static HRESULT WINAPI d_GetStreamSource(IDirect3DDevice9 *t, UINT i, IDirect3DVertexBuffer9 **b, UINT *off, UINT *stride)
{
    d3dev *d = DEV(t);
    if (i >= MAX_STREAMS) return D3DERR_INVALIDCALL;
    *b = (IDirect3DVertexBuffer9 *)d->st.vb[i]; *off = d->st.vb_off[i]; *stride = d->st.vb_stride[i];
    if (*b) IDirect3DVertexBuffer9_AddRef(*b);
    return D3D_OK;
}
static HRESULT WINAPI d_SetStreamSourceFreq(IDirect3DDevice9 *t, UINT i, UINT f) { d3dev *d = DEV(t); if (i >= MAX_STREAMS) return D3DERR_INVALIDCALL; MARK_BIT(vb, i); d->st.vb_freq[i] = f; return D3D_OK; }
static HRESULT WINAPI d_GetStreamSourceFreq(IDirect3DDevice9 *t, UINT i, UINT *f) { d3dev *d = DEV(t); if (i >= MAX_STREAMS) return D3DERR_INVALIDCALL; *f = d->st.vb_freq[i]; return D3D_OK; }
static HRESULT WINAPI d_SetIndices(IDirect3DDevice9 *t, IDirect3DIndexBuffer9 *b) { d3dev *d = DEV(t); MARK(ib); d3_ref_set(&d->st.ib, b); return D3D_OK; }
static HRESULT WINAPI d_GetIndices(IDirect3DDevice9 *t, IDirect3DIndexBuffer9 **b) { *b = (IDirect3DIndexBuffer9 *)DEV(t)->st.ib; if (*b) IDirect3DIndexBuffer9_AddRef(*b); return D3D_OK; }

/* ---------------------------------------------------------------- draws */
static int prim_vertices(D3DPRIMITIVETYPE p, UINT c, int *hw)
{
    switch (p) {
    case D3DPT_POINTLIST: *hw = 3; return (int)c;
    case D3DPT_LINELIST: *hw = 4; return (int)c * 2;
    case D3DPT_LINESTRIP: *hw = 5; return (int)c + 1;
    case D3DPT_TRIANGLELIST: *hw = 0; return (int)c * 3;
    case D3DPT_TRIANGLESTRIP: *hw = 1; return (int)c + 2;
    case D3DPT_TRIANGLEFAN: *hw = 2; return (int)c + 2;
    default: return 0;
    }
}

static int attr_of(BYTE type, int *size, int *norm, int *bytes)
{
    *norm = 0;
    switch (type) {
    case D3DDECLTYPE_FLOAT1: *size = 1; *bytes = 4; return G2_AT_FLOAT;
    case D3DDECLTYPE_FLOAT2: *size = 2; *bytes = 8; return G2_AT_FLOAT;
    case D3DDECLTYPE_FLOAT3: *size = 3; *bytes = 12; return G2_AT_FLOAT;
    case D3DDECLTYPE_FLOAT4: *size = 4; *bytes = 16; return G2_AT_FLOAT;
    case D3DDECLTYPE_D3DCOLOR: *size = 4; *norm = 1; *bytes = 4; return G2_AT_UBYTE;
    case D3DDECLTYPE_UBYTE4: *size = 4; *bytes = 4; return G2_AT_UBYTE;
    case D3DDECLTYPE_UBYTE4N: *size = 4; *norm = 1; *bytes = 4; return G2_AT_UBYTE;
    case D3DDECLTYPE_SHORT2: *size = 2; *bytes = 4; return G2_AT_SHORT;
    case D3DDECLTYPE_SHORT4: *size = 4; *bytes = 8; return G2_AT_SHORT;
    case D3DDECLTYPE_SHORT2N: *size = 2; *norm = 1; *bytes = 4; return G2_AT_SHORT;
    case D3DDECLTYPE_SHORT4N: *size = 4; *norm = 1; *bytes = 8; return G2_AT_SHORT;
    case D3DDECLTYPE_USHORT2N: *size = 2; *norm = 1; *bytes = 4; return G2_AT_USHORT;
    case D3DDECLTYPE_USHORT4N: *size = 4; *norm = 1; *bytes = 8; return G2_AT_USHORT;
    default: return -1;
    }
}

static const D3DVERTEXELEMENT9 *find_el(const d3decl *d, int usage, int index)
{
    int i;
    for (i = 0; i < d->n; i++) if (d->el[i].Usage == usage && d->el[i].UsageIndex == index) return &d->el[i];
    if (usage == D3DDECLUSAGE_POSITION) for (i = 0; i < d->n; i++) if (d->el[i].Usage == D3DDECLUSAGE_POSITIONT && d->el[i].UsageIndex == index) return &d->el[i];
    return NULL;
}

static void fill_state(d3dev *d, g2r_state *s, d3prog *p, const tgt *g)
{
    static const uint8_t BL[16] = { 0, G2R_BF_ZERO, G2R_BF_ONE, G2R_BF_SRC_COLOR, G2R_BF_ONE_MINUS_SRC_COLOR, G2R_BF_SRC_ALPHA, G2R_BF_ONE_MINUS_SRC_ALPHA, G2R_BF_DST_ALPHA, G2R_BF_ONE_MINUS_DST_ALPHA,
                                    G2R_BF_DST_COLOR, G2R_BF_ONE_MINUS_DST_COLOR, G2R_BF_SRC_ALPHA_SATURATE, G2R_BF_SRC_ALPHA, G2R_BF_ONE_MINUS_SRC_ALPHA, G2R_BF_CONSTANT_COLOR, G2R_BF_ONE_MINUS_CONSTANT_COLOR };
    const d3state *st = &d->st;
    const DWORD *rs = st->rs;
    uint32_t *h = s->h, cw = rs[D3DRS_COLORWRITEENABLE], cmask, i;
    uint32_t sb, db, sa, da, bo, boa;
    int depth = rs[D3DRS_ZENABLE] && g->t.ds, sten = rs[D3DRS_STENCILENABLE] && g->t.ds, twoside = rs[D3DRS_TWOSIDEDSTENCILMODE] != 0, b[4], blend = rs[D3DRS_ALPHABLENDENABLE] != 0;
    static const uint8_t ZREV[8] = { 0, 4, 2, 6, 1, 5, 3, 7 };    /* the comparison of 1 - a with 1 - b: LESS <-> GREATER, LESSEQUAL <-> GREATEREQUAL */
    int zrev = d3_zrev(d);
    uint32_t zf = (rs[D3DRS_ZFUNC] - 1) & 7;
    float bias, slope;
    cmask = (cw & 1 ? 8u : 0) | (cw & 2 ? 4u : 0) | (cw & 4 ? 2u : 0) | (cw & 8 ? 1u : 0);
    if (g->no_alpha) cmask &= 14;
    h[0] = (depth ? 1u : 0) | (depth && rs[D3DRS_ZWRITEENABLE] ? 2u : 0) | (zrev ? ZREV[zf] : zf) << 2 | (sten ? 32u : 0) | (blend ? 64u : 0) | (rs[D3DRS_DITHERENABLE] ? 128u : 0) |
           (rs[D3DRS_SCISSORTESTENABLE] ? 256u : 0) | cmask << 16;
    memcpy(&bias, &rs[D3DRS_DEPTHBIAS], 4);
    memcpy(&slope, &rs[D3DRS_SLOPESCALEDEPTHBIAS], 4);
    if (bias != 0.0f || slope != 0.0f) {
        h[0] |= 512u;
        if (zrev) { slope = -slope; bias = -bias; }             /* towards the far end is down */
        h[9] = g2_fp24(slope);
        h[10] = (uint32_t)(int32_t)(bias * (g->t.depth16 ? 65536.0f : 16777216.0f));
    }
    h[1] = (rs[D3DRS_STENCILREF] & 255) | (rs[D3DRS_STENCILMASK] & 255) << 8 | (rs[D3DRS_STENCILWRITEMASK] & 255) << 16 | ((rs[D3DRS_STENCILFUNC] - 1) & 7) << 24;
    h[2] = (rs[D3DRS_STENCILREF] & 255) | (rs[D3DRS_STENCILMASK] & 255) << 8 | (rs[D3DRS_STENCILWRITEMASK] & 255) << 16 | (((twoside ? rs[D3DRS_CCW_STENCILFUNC] : rs[D3DRS_STENCILFUNC]) - 1) & 7) << 24;
    h[3] = ((rs[D3DRS_STENCILFAIL] - 1) & 7) | ((rs[D3DRS_STENCILZFAIL] - 1) & 7) << 3 | ((rs[D3DRS_STENCILPASS] - 1) & 7) << 6;
    h[3] |= twoside ? ((rs[D3DRS_CCW_STENCILFAIL] - 1) & 7) << 16 | ((rs[D3DRS_CCW_STENCILZFAIL] - 1) & 7) << 19 | ((rs[D3DRS_CCW_STENCILPASS] - 1) & 7) << 22 : (h[3] & 511) << 16;
    sb = rs[D3DRS_SRCBLEND]; db = rs[D3DRS_DESTBLEND]; bo = rs[D3DRS_BLENDOP];
    if (sb == D3DBLEND_BOTHSRCALPHA) { sb = D3DBLEND_SRCALPHA; db = D3DBLEND_INVSRCALPHA; }
    else if (sb == D3DBLEND_BOTHINVSRCALPHA) { sb = D3DBLEND_INVSRCALPHA; db = D3DBLEND_SRCALPHA; }
    sa = sb; da = db; boa = bo;
    if (rs[D3DRS_SEPARATEALPHABLENDENABLE]) { sa = rs[D3DRS_SRCBLENDALPHA]; da = rs[D3DRS_DESTBLENDALPHA]; boa = rs[D3DRS_BLENDOPALPHA]; }
    h[4] = (uint32_t)BL[sb & 15] | (uint32_t)BL[db & 15] << 4 | (uint32_t)BL[sa & 15] << 8 | (uint32_t)BL[da & 15] << 12 | ((bo - 1) & 7) << 16 | ((boa - 1) & 7) << 20;
    {
        DWORD c = rs[D3DRS_BLENDFACTOR];
        double r = (c >> 16 & 255) / 255.0, gr = (c >> 8 & 255) / 255.0, bl = (c & 255) / 255.0, a = (c >> 24) / 255.0;
        h[5] = g2_c16(r, g->t.bits_r) | g2_c16(gr, g->t.bits_g) << 13;
        h[6] = g2_c16(bl, g->t.bits_b) | g2_c16(a, g->t.bits_a ? g->t.bits_a : 8) << 13;
        h[14] = g2_c16(a, g->t.bits_r) | g2_c16(a, g->t.bits_g) << 13;
    }
    clip_rect(g, &st->sc, b);
    h[7] = (uint32_t)b[0] | (uint32_t)b[1] << 16; h[8] = (uint32_t)b[2] | (uint32_t)b[3] << 16;
    for (i = 0; i < (uint32_t)p->nsamp; i++) {
        int stage = p->stage_of[i];
        d3tex *x = st->tex[stage];
        const DWORD *ss = st->ss[stage];
        DWORD mf = ss[D3DSAMP_MIPFILTER];
#define ADDR(a) ((a) == D3DTADDRESS_WRAP ? G2R_WRAP_REPEAT : (a) == D3DTADDRESS_MIRROR ? G2R_WRAP_MIRROR : G2R_WRAP_CLAMP)    /* MIRRORONCE and BORDER clamp here: d3d9_ff.c, fs_macros */
        if (x) {
            d3_tex_ready(x);
            x->used_pass = g2r_pass_id(d->r);
            h[15 + i] = G2R_SAMPLER3D(d3_tex_image(x, ss), x->depth ? 0 : G2R_FILTER(ss[D3DSAMP_MAGFILTER] != D3DTEXF_POINT, x->hwl == 1 ? 0 : mf == D3DTEXF_POINT ? 1 : mf == D3DTEXF_NONE ? 0 : 2, ss[D3DSAMP_MINFILTER] != D3DTEXF_POINT), ADDR(ss[D3DSAMP_ADDRESSU]), ADDR(ss[D3DSAMP_ADDRESSV]), x->vol ? ADDR(ss[D3DSAMP_ADDRESSW]) : 0, 0);
        } else {
            h[15 + i] = G2R_SAMPLER(0, 0, 0, 0, 1);             /* no texture: (0, 0, 0, 1) */
        }
    }
    s->vp_x = st->vp.X * g->kx; s->vp_y = st->vp.Y * g->ky; s->vp_w = st->vp.Width * g->kx; s->vp_h = st->vp.Height * g->ky;
    g2r_scissor_viewport(s, g->t.width, g->t.height);           /* everything stops at the viewport, pretransformed vertices too */
    s->depth_near = st->vp.MinZ; s->depth_far = st->vp.MaxZ;
    if (zrev) { s->z01 = 1; s->depth_near = 1.0 - (double)st->vp.MaxZ; s->depth_far = 1.0 - (double)st->vp.MinZ; }     /* z' / w runs 0 (far) .. 1 (near): 1 - MaxZ .. 1 - MinZ in the surface */
    s->cull = rs[D3DRS_CULLMODE] == D3DCULL_CCW ? 1 : rs[D3DRS_CULLMODE] == D3DCULL_CW ? 2 : 0;
    s->front_cw = 0;
    s->line_width = 1.0;
}

/* a triangle the cull mode drops: the sign of the determinant of x, y, w is the winding on screen for any w, so a triangle through the near plane needs no clipping first */
static int culled(d3dev *d, const uint8_t *up, UINT up_stride, int inst, uint32_t a, uint32_t b, uint32_t c)
{
    float p[3][4], det;
    DWORD m = d->st.rs[D3DRS_CULLMODE];
    if (m == D3DCULL_NONE) return 0;
    if (d3_cpu_position(d, up, up_stride, a, inst, p[0]) < 0 || d3_cpu_position(d, up, up_stride, b, inst, p[1]) < 0 || d3_cpu_position(d, up, up_stride, c, inst, p[2]) < 0) return 0;
    det = p[0][0] * (p[1][1] * p[2][3] - p[2][1] * p[1][3]) - p[0][1] * (p[1][0] * p[2][3] - p[2][0] * p[1][3]) + p[0][3] * (p[1][0] * p[2][1] - p[2][0] * p[1][1]);
    return m == D3DCULL_CCW ? det > 0.0f : det < 0.0f;          /* y is up here: counter-clockwise is positive */
}

/* The primitives as separate vertices. Flat shading takes a primitive's first vertex, which an indexed strip cannot say; wireframe and point fill are the lines and points of the triangles culling leaves.
 * *list: the vertices, *prov: each one's primitive's first vertex. A strip's is its first in order of the strip, a fan's the one after the hub. Returns how many */
static uint32_t expand_prims(d3dev *d, int mode, int to, const uint32_t *idx, uint32_t n, uint32_t first, const uint8_t *up, UINT up_stride, int inst, uint32_t **list, uint32_t **prov)
{
    uint32_t cap = (mode <= 2 ? (mode ? n : n / 3) * 6 : n * 2) + 6, m = 0, t, *l, *f;
#define V(k) (idx ? idx[k] : first + (k))
#define PUT(x, pv) do { l[m] = (x); f[m++] = (pv); } while (0)
    l = (uint32_t *)malloc((size_t)cap * 4);
    f = (uint32_t *)malloc((size_t)cap * 4);
    if (!l || !f) { free(l); free(f); return 0; }
    if (mode >= 4) {
        uint32_t step = mode == 4 ? 2 : 1;
        for (t = 0; t + 1 < n; t += step) { uint32_t a = V(t), b = V(t + 1); PUT(a, a); PUT(b, a); }
    } else {
        uint32_t tris = mode ? (n >= 2 ? n - 2 : 0) : n / 3;
        for (t = 0; t < tris; t++) {
            uint32_t a, b, c, pv;
            if (!mode) { a = V(3 * t); b = V(3 * t + 1); c = V(3 * t + 2); pv = a; }
            else if (mode == 1) { a = V(t & 1 ? t + 1 : t); b = V(t & 1 ? t : t + 1); c = V(t + 2); pv = V(t); }
            else { a = V(0); b = V(t + 1); c = V(t + 2); pv = b; }
            if (to != mode && to && culled(d, up, up_stride, inst, a, b, c)) continue;
            if (to == 4) { PUT(a, pv); PUT(b, pv); PUT(b, pv); PUT(c, pv); PUT(c, pv); PUT(a, pv); }
            else { PUT(a, pv); PUT(b, pv); PUT(c, pv); }
        }
    }
#undef V
#undef PUT
    *list = l; *prov = f;
    return m;
}

/* n vertices from `first` (idx NULL), or n indices; up: DrawPrimitiveUP's stream 0; inst: the instance, -1: not instanced */
static HRESULT draw_one(d3dev *d, int mode, const uint32_t *idx, uint32_t n, uint32_t first, const uint8_t *up, UINT up_stride, int inst)
{
    d3state *st = &d->st;
    d3decl *de = st->decl;
    d3prog *p;
    tgt g;
    g2r_state s;
    g2r_vsrc v[16];
    void *tmp[16];
    d3buf *from[16];
    uint32_t lo = first, hi = first + n - 1, i, ioff = 0, *list = NULL, *prov = NULL;
    uint8_t of_first[16];
    int a, to = mode;
    if (!n || !de) { D3T("d3d9: draw skipped: %u vertices, declaration %p", n, (void *)de); return D3D_OK; }
    if ((st->vs && st->vs->error[0]) || (st->ps && st->ps->error[0])) { D3T("d3d9: draw skipped: an untranslated shader"); return D3D_OK; }     /* said once, at creation */
    if (mode <= 2 && st->rs[D3DRS_FILLMODE] == D3DFILL_WIREFRAME) to = 4;
    else if (mode <= 2 && st->rs[D3DRS_FILLMODE] == D3DFILL_POINT) to = 3;
    p = d3_ff_program(d, de, st->vs, st->ps, to);
    if (!p || !p->p) { D3T("d3d9: draw skipped: no program"); return D3D_OK; }
    if (bind_target(d, &g) < 0) return D3DERR_INVALIDCALL;
    if (d3_rec_on()) d3_rec_draw(d, mode, idx, n, first, up, up_stride);
    if (to != mode || (p->flat && mode != 3)) {
        n = expand_prims(d, mode, to, idx, n, first, up, up_stride, inst, &list, &prov);
        if (!n) { free(list); free(prov); return D3D_OK; }      /* all culled */
        idx = list;
        mode = to == mode ? (mode <= 2 ? 0 : 4) : to;
    }
    if (idx) {
        lo = 0xFFFFFFFFu; hi = 0;
        for (i = 0; i < n; i++) { if (idx[i] < lo) lo = idx[i]; if (idx[i] > hi) hi = idx[i]; }
    }
    memset(&s, 0, sizeof s);
    fill_state(d, &s, p, &g);
    s.prog = p->p;
    for (a = 0; a < 16; a++) { s.attr[a].size = 4; s.attr[a].type = G2_AT_FP24; s.attr[a].constant[3] = g2_fp24(1.0); }
    memset(v, 0, sizeof v);
    memset(tmp, 0, sizeof tmp);
    memset(from, 0, sizeof from);
    memset(of_first, 0, sizeof of_first);
    for (a = 0; a < p->nattr; a++) {
        const D3DVERTEXELEMENT9 *e = find_el(de, p->attr[a].usage, p->attr[a].index);
        int k = p->attr[a].idx, size, norm, bytes, ty;
        const uint8_t *base;
        UINT stride;
        g2r_attr *o = &s.attr[k];
        if (!e) continue;
        ty = attr_of(e->Type, &size, &norm, &bytes);
        if (ty < 0) { static int said; if (!said++) g2e_log("d3d9: vertex element type %d is not supported", e->Type); continue; }
        of_first[k] = (uint8_t)p->attr[a].first;
        if (inst >= 0 && !up && (st->vb_freq[e->Stream] & D3DSTREAMSOURCE_INSTANCEDATA)) {      /* one value per instance: the attribute stays off and holds it */
            const uint8_t *q = d3_stream_at(d, e, NULL, 0, 0, inst);
            float f[4];
            int c;
            if (q) { d3_elem_read(q, e->Type, f); for (c = 0; c < 4; c++) o->constant[c] = g2_fp24(f[c]); }
            continue;
        }
        if (up) { base = up; stride = up_stride; }
        else {
            d3buf *b = st->vb[e->Stream];
            if (!b) continue;
            base = b->mem + st->vb_off[e->Stream]; stride = st->vb_stride[e->Stream];
            from[k] = b;
        }
        v[k].base = base + e->Offset; v[k].bytes = bytes; v[k].stride = (int)stride;
        o->enabled = 1; o->size = size; o->type = ty; o->normalized = norm;
        o->handle = g2r_stream_handle(d->r);
        o->stride = stride;
    }
    if (prov || (idx && hi - lo >= 65536)) {                    /* the hardware's indices are 16 bits: separate vertices */
        for (a = 0; a < 16; a++) {
            uint8_t *t;
            const uint32_t *at = of_first[a] ? prov : idx;
            if (!v[a].base) continue;
            t = (uint8_t *)malloc((size_t)n * v[a].bytes);
            if (!t) goto out;
            for (i = 0; i < n; i++) memcpy(t + (size_t)i * v[a].bytes, v[a].base + (size_t)at[i] * v[a].stride, (size_t)v[a].bytes);
            tmp[a] = t;
            v[a].base = t; v[a].stride = v[a].bytes;
            s.attr[a].stride = (uint32_t)v[a].bytes;
        }
        idx = NULL; lo = 0; hi = n - 1;
    }
    for (a = 0; a < 16; a++) {                                  /* what board A already holds is not sent; vertex lo is what the indices below count from */
        d3buf *b = from[a];
        if (b && !tmp[a] && g2r_keep(d->r, &b->keep, b->mem, b->size)) {
            uint32_t at = (uint32_t)(v[a].base - b->mem) + lo * (uint32_t)v[a].stride;
            if (!g2r_kept_same(d->r, &b->keep, b->mem, at, (hi - lo) * (uint32_t)v[a].stride + (uint32_t)v[a].bytes)) continue;
            s.attr[a].handle = g2r_kept_handle(d->r);
            s.attr[a].offset = b->keep.off + at;
            v[a].base = NULL;
        }
    }
    g2r_gather(d->r, v, 16, lo, hi - lo + 1);
    for (a = 0; a < 16; a++) if (v[a].base) s.attr[a].offset = v[a].off;
    if (idx) {
        uint16_t *ib = (uint16_t *)malloc((size_t)n * 2);
        if (!ib) goto out;
        for (i = 0; i < n; i++) ib[i] = (uint16_t)(idx[i] - lo);
        ioff = g2r_stream(d->r, ib, n * 2, 2);
        free(ib);
    }
    d3_prog_uniforms(d, p, g.vw, g.vh, g.ky);
    s.vs_uniforms = d->vu;
    s.fs_uniforms = d->fu;
    /* pretransformed vertices ignore the viewport */
    if (p->vu[U_RT] >= 0) { s.vp_x = 0; s.vp_y = 0; s.vp_w = g.t.width; s.vp_h = g.t.height; s.depth_near = 0; s.depth_far = 1; }       /* (the range 0 .. 1 either way round) */
    if (d3_trace() > 1) g2e_log("d3d9: draw mode %d, %u vertices, target %u x %u, vs %u ps %u, frame %u", mode, n, g.t.width, g.t.height, st->vs ? st->vs->id : 0, st->ps ? st->ps->id : 0, d->frame);
    g2r_draw(d->r, &s, mode, n, 0, idx != NULL, g2r_stream_handle(d->r), ioff, 0);
out:
    for (a = 0; a < 16; a++) free(tmp[a]);
    free(list); free(prov);
    return D3D_OK;
}

/* instancing: SetStreamSourceFreq's stream 0 holds the count. The hardware draws one instance at a time: a draw for each, the instance streams' elements as constant attributes */
static HRESULT draw(d3dev *d, int mode, const uint32_t *idx, uint32_t n, uint32_t first, const uint8_t *up, UINT up_stride)
{
    UINT f0 = d->st.vb_freq[0];
    if (idx && !up && (f0 & D3DSTREAMSOURCE_INDEXEDDATA)) {
        UINT i, count = f0 & 0x3FFFFFFFu;
        for (i = 0; i < count; i++) draw_one(d, mode, idx, n, first, up, up_stride, (int)i);
        return D3D_OK;
    }
    return draw_one(d, mode, idx, n, first, up, up_stride, -1);
}

static HRESULT WINAPI d_DrawPrimitive(IDirect3DDevice9 *t, D3DPRIMITIVETYPE p, UINT start, UINT c)
{
    int hw = 0, n = prim_vertices(p, c, &hw);
    if (!n) return D3DERR_INVALIDCALL;
    return draw(DEV(t), hw, NULL, (uint32_t)n, start, NULL, 0);
}

static uint32_t *index_list(const void *src, int bits, uint32_t n, int base)
{
    uint32_t *o = (uint32_t *)malloc((size_t)n * 4), i;
    if (!o) return NULL;
    for (i = 0; i < n; i++) o[i] = (uint32_t)(base + (int)(bits == 32 ? ((const uint32_t *)src)[i] : ((const uint16_t *)src)[i]));
    return o;
}

static HRESULT WINAPI d_DrawIndexedPrimitive(IDirect3DDevice9 *t, D3DPRIMITIVETYPE p, INT base, UINT mi, UINT nv, UINT start, UINT c)
{
    d3dev *d = DEV(t);
    d3buf *b = d->st.ib;
    int hw = 0, n = prim_vertices(p, c, &hw), bits;
    uint32_t *o;
    HRESULT r;
    (void)mi; (void)nv;
    if (!n || !b) return D3DERR_INVALIDCALL;
    bits = b->fmt == D3DFMT_INDEX32 ? 32 : 16;
    o = index_list(b->mem + start * (UINT)(bits / 8), bits, (uint32_t)n, base);
    if (!o) return E_OUTOFMEMORY;
    r = draw(d, hw, o, (uint32_t)n, 0, NULL, 0);
    free(o);
    return r;
}
static HRESULT WINAPI d_DrawPrimitiveUP(IDirect3DDevice9 *t, D3DPRIMITIVETYPE p, UINT c, const void *data, UINT stride)
{
    int hw = 0, n = prim_vertices(p, c, &hw);
    if (!n) return D3DERR_INVALIDCALL;
    return draw(DEV(t), hw, NULL, (uint32_t)n, 0, (const uint8_t *)data, stride);
}
static HRESULT WINAPI d_DrawIndexedPrimitiveUP(IDirect3DDevice9 *t, D3DPRIMITIVETYPE p, UINT mi, UINT nv, UINT c, const void *ind, D3DFORMAT f, const void *data, UINT stride)
{
    int hw = 0, n = prim_vertices(p, c, &hw);
    uint32_t *o;
    HRESULT r;
    (void)mi; (void)nv;
    if (!n) return D3DERR_INVALIDCALL;
    o = index_list(ind, f == D3DFMT_INDEX32 ? 32 : 16, (uint32_t)n, 0);
    if (!o) return E_OUTOFMEMORY;
    r = draw(DEV(t), hw, o, (uint32_t)n, 0, (const uint8_t *)data, stride);
    free(o);
    return r;
}
static HRESULT WINAPI d_ProcessVertices(IDirect3DDevice9 *t, UINT a, UINT b, UINT c, IDirect3DVertexBuffer9 *o, IDirect3DVertexDeclaration9 *de, DWORD f)
{
    d3dev *d = DEV(t);
    d3buf *out = (d3buf *)o;
    if (!out) return D3DERR_INVALIDCALL;
    return d3_process_vertices(d, a, b, c, out, de ? (const d3decl *)de : out->fvf ? fvf_decl(d, out->fvf) : NULL, f);
}
static HRESULT WINAPI d_DrawRectPatch(IDirect3DDevice9 *t, UINT h, const float *s, const D3DRECTPATCH_INFO *i) { (void)t; (void)h; (void)s; (void)i; return D3DERR_NOTAVAILABLE; }
static HRESULT WINAPI d_DrawTriPatch(IDirect3DDevice9 *t, UINT h, const float *s, const D3DTRIPATCH_INFO *i) { (void)t; (void)h; (void)s; (void)i; return D3DERR_NOTAVAILABLE; }
static HRESULT WINAPI d_DeletePatch(IDirect3DDevice9 *t, UINT h) { (void)t; (void)h; return D3D_OK; }
static HRESULT WINAPI d_CreateQuery(IDirect3DDevice9 *t, D3DQUERYTYPE ty, IDirect3DQuery9 **o)
{
    d3query *q;
    D3T("d3d9: CreateQuery %d", (int)ty);
    if (ty != D3DQUERYTYPE_EVENT && ty != D3DQUERYTYPE_TIMESTAMP && ty != D3DQUERYTYPE_TIMESTAMPDISJOINT && ty != D3DQUERYTYPE_TIMESTAMPFREQ && ty != D3DQUERYTYPE_VCACHE) return D3DERR_NOTAVAILABLE;     /* B counts no samples per query */
    if (!o) return D3D_OK;
    q = (d3query *)calloc(1, sizeof *q);
    if (!q) return E_OUTOFMEMORY;
    q->vtbl = &d3_query_vtbl; q->refs = 1; q->dev = DEV(t); q->type = ty;
    *o = (IDirect3DQuery9 *)q;
    return D3D_OK;
}

/* ---------------------------------------------------------------- swap chains. There is one screen: the implicit chain flips, an additional one is its own target, copied to fit the screen when it presents */
typedef struct d3chain { const void *vtbl; LONG refs; d3dev *dev; d3surf *back; D3DPRESENT_PARAMETERS pp; } d3chain;
#define CH(t) ((d3chain *)(t))

static HRESULT chain_present(d3chain *c)
{
    d3dev *d = c->dev;
    d3surf *b = c->back;
    int i, dx, dy, dw, dh, hold = d->hold && c->pp.PresentationInterval != D3DPRESENT_INTERVAL_IMMEDIATE;
    g2r_end_pass(d->r);
    if (!d->scan[0]) {                                          /* the implicit chain makes these only if its size needs them */
        for (i = 0; i < 3; i++) g2r_image_create(d->r, &d->scan[i], &d->scan_base[i], (uint32_t)d->mode_w, (uint32_t)d->mode_h, G2_FMT_RGBA8, 1, 1, 0, G2_USE_SCANOUT | G2_USE_TRANSFER);
    }
    fit_rect(d, (int)b->w, (int)b->h, &dx, &dy, &dw, &dh);
    d->scan_i = (d->scan_i + 1) % 3;
    g2_fill_image(d->bd, d->scan[d->scan_i], G2_TF_RAW, 0x000000FFu, 0, 0, (uint32_t)d->mode_w, (uint32_t)d->mode_h, 0xF, 0, 0);
    g2_copy_image(d->bd, b->handle, d->scan[d->scan_i], 0, 0, b->w, b->h, (uint32_t)dx, (uint32_t)dy, (uint32_t)dw, (uint32_t)dh, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    g2r_present(d->r, d->scan[d->scan_i], hold);
    d->frame++;
    return D3D_OK;
}

static HRESULT WINAPI c_QueryInterface(IDirect3DSwapChain9 *t, REFIID id, void **o)
{
    if (IsEqualGUID(id, &IID_IUnknown) || IsEqualGUID(id, &IID_IDirect3DSwapChain9)) { IDirect3DSwapChain9_AddRef(t); *o = t; return S_OK; }
    *o = NULL;
    return E_NOINTERFACE;
}
static ULONG WINAPI c_AddRef(IDirect3DSwapChain9 *t) { return (ULONG)InterlockedIncrement(&CH(t)->refs); }
static ULONG WINAPI c_Release(IDirect3DSwapChain9 *t)
{
    d3chain *c = CH(t);
    LONG r = InterlockedDecrement(&c->refs);
    if (!r && c->back) { d3_unbind(c->back); free(c); }        /* the implicit chain lives with the device */
    return (ULONG)r;
}
static HRESULT WINAPI c_Present(IDirect3DSwapChain9 *t, const RECT *s, const RECT *o, HWND w, const RGNDATA *r, DWORD f) { (void)s; (void)o; (void)w; (void)r; (void)f; return CH(t)->back ? chain_present(CH(t)) : present(CH(t)->dev); }
static HRESULT WINAPI c_GetFrontBufferData(IDirect3DSwapChain9 *t, IDirect3DSurface9 *o) { return d3_front_read(((d3chain *)t)->dev, (d3surf *)o); }
static HRESULT WINAPI c_GetBackBuffer(IDirect3DSwapChain9 *t, UINT i, D3DBACKBUFFER_TYPE ty, IDirect3DSurface9 **o)
{
    if (!CH(t)->back) return d_GetBackBuffer((IDirect3DDevice9 *)CH(t)->dev, 0, i, ty, o);
    *o = (IDirect3DSurface9 *)CH(t)->back;
    IDirect3DSurface9_AddRef(*o);
    return D3D_OK;
}
static HRESULT WINAPI c_GetRasterStatus(IDirect3DSwapChain9 *t, D3DRASTER_STATUS *o) { return d_GetRasterStatus((IDirect3DDevice9 *)CH(t)->dev, 0, o); }
static HRESULT WINAPI c_GetDisplayMode(IDirect3DSwapChain9 *t, D3DDISPLAYMODE *m) { return d_GetDisplayMode((IDirect3DDevice9 *)CH(t)->dev, 0, m); }
static HRESULT WINAPI c_GetDevice(IDirect3DSwapChain9 *t, IDirect3DDevice9 **o) { *o = (IDirect3DDevice9 *)CH(t)->dev; IDirect3DDevice9_AddRef(*o); return D3D_OK; }
static HRESULT WINAPI c_GetPresentParameters(IDirect3DSwapChain9 *t, D3DPRESENT_PARAMETERS *o) { *o = CH(t)->back ? CH(t)->pp : CH(t)->dev->pp; return D3D_OK; }
const IDirect3DSwapChain9Vtbl d3_chain_vtbl = { c_QueryInterface, c_AddRef, c_Release, c_Present, c_GetFrontBufferData, c_GetBackBuffer, c_GetRasterStatus, c_GetDisplayMode, c_GetDevice, c_GetPresentParameters };
const void *d3_vt_chain = &d3_chain_vtbl;

static HRESULT WINAPI d_CreateAdditionalSwapChain(IDirect3DDevice9 *t, D3DPRESENT_PARAMETERS *p, IDirect3DSwapChain9 **o)
{
    d3dev *d = DEV(t);
    d3chain *c;
    d3surf *b;
    RECT r;
    HWND w = p->hDeviceWindow ? p->hDeviceWindow : d->hwnd;
    UINT vw = p->BackBufferWidth, vh = p->BackBufferHeight, rw, rh;
    D3DFORMAT f = p->BackBufferFormat == D3DFMT_UNKNOWN ? D3DFMT_X8R8G8B8 : p->BackBufferFormat;
    if (p->MultiSampleType != D3DMULTISAMPLE_NONE || !d3_fmt_rt(f)) return D3DERR_INVALIDCALL;
    if ((!vw || !vh) && GetClientRect(w, &r)) { if (!vw) vw = (UINT)r.right; if (!vh) vh = (UINT)r.bottom; }
    if (!vw) vw = 640;
    if (!vh) vh = 480;
    rw = vw; rh = vh;
    if (rw > MAX_RT_W) { rh = (UINT)((double)rh * MAX_RT_W / rw + 0.5); rw = MAX_RT_W; }        /* renders smaller, viewport and scissor scale: as the back buffer */
    c = (d3chain *)calloc(1, sizeof *c);
    b = d3_surface_new(d, rw, rh, f, D3DUSAGE_RENDERTARGET, D3DPOOL_DEFAULT, 0);
    if (!c || !b) { free(c); if (b) IDirect3DSurface9_Release((IDirect3DSurface9 *)b); return E_OUTOFMEMORY; }
    b->vw = vw; b->vh = vh;
    if (g2r_image_create(d->r, &b->handle, &b->base, rw, rh, b->g2fmt, 1, 1, 0, G2_USE_RT | G2_USE_TRANSFER | G2_USE_SAMPLED) < 0) { IDirect3DSurface9_Release((IDirect3DSurface9 *)b); free(c); return D3DERR_OUTOFVIDEOMEMORY; }
    p->BackBufferWidth = vw; p->BackBufferHeight = vh; p->BackBufferFormat = f;
    if (!p->BackBufferCount) p->BackBufferCount = 1;
    c->vtbl = d3_vt_chain; c->refs = 1; c->dev = d; c->back = b; c->pp = *p;
    b->binds = 1;                                               /* the chain's own, as the device's back buffer */
    *o = (IDirect3DSwapChain9 *)c;
    return D3D_OK;
}

static HRESULT WINAPI d_GetSwapChain(IDirect3DDevice9 *t, UINT i, IDirect3DSwapChain9 **o)
{
    d3dev *d = DEV(t);
    if (i) return D3DERR_INVALIDCALL;
    if (!d->swapchain) {
        d3chain *c = (d3chain *)calloc(1, sizeof *c);
        if (!c) return E_OUTOFMEMORY;
        c->vtbl = &d3_chain_vtbl; c->refs = 1; c->dev = d;
        d->swapchain = c;
    }
    *o = (IDirect3DSwapChain9 *)d->swapchain;
    IDirect3DSwapChain9_AddRef(*o);
    return D3D_OK;
}

#define S(n) .n = d_##n
const IDirect3DDevice9Vtbl d3_dev_vtbl = {
    S(QueryInterface), S(AddRef), S(Release), S(TestCooperativeLevel), S(GetAvailableTextureMem), S(EvictManagedResources), S(GetDirect3D), S(GetDeviceCaps), S(GetDisplayMode), S(GetCreationParameters),
    S(SetCursorProperties), S(SetCursorPosition), S(ShowCursor), S(CreateAdditionalSwapChain), S(GetSwapChain), S(GetNumberOfSwapChains), S(Reset), S(Present), S(GetBackBuffer), S(GetRasterStatus),
    S(SetDialogBoxMode), S(SetGammaRamp), S(GetGammaRamp), S(CreateTexture), S(CreateVolumeTexture), S(CreateCubeTexture), S(CreateVertexBuffer), S(CreateIndexBuffer), S(CreateRenderTarget), S(CreateDepthStencilSurface),
    S(UpdateSurface), S(UpdateTexture), S(GetRenderTargetData), S(GetFrontBufferData), S(StretchRect), S(ColorFill), S(CreateOffscreenPlainSurface), S(SetRenderTarget), S(GetRenderTarget), S(SetDepthStencilSurface), S(GetDepthStencilSurface),
    S(BeginScene), S(EndScene), S(Clear), S(SetTransform), S(GetTransform), S(MultiplyTransform), S(SetViewport), S(GetViewport), S(SetMaterial), S(GetMaterial), S(SetLight), S(GetLight), S(LightEnable), S(GetLightEnable),
    S(SetClipPlane), S(GetClipPlane), S(SetRenderState), S(GetRenderState), S(CreateStateBlock), S(BeginStateBlock), S(EndStateBlock), S(SetClipStatus), S(GetClipStatus), S(GetTexture), S(SetTexture),
    S(GetTextureStageState), S(SetTextureStageState), S(GetSamplerState), S(SetSamplerState), S(ValidateDevice), S(SetPaletteEntries), S(GetPaletteEntries), S(SetCurrentTexturePalette), S(GetCurrentTexturePalette),
    S(SetScissorRect), S(GetScissorRect), S(SetSoftwareVertexProcessing), S(GetSoftwareVertexProcessing), S(SetNPatchMode), S(GetNPatchMode), S(DrawPrimitive), S(DrawIndexedPrimitive), S(DrawPrimitiveUP), S(DrawIndexedPrimitiveUP),
    S(ProcessVertices), S(CreateVertexDeclaration), S(SetVertexDeclaration), S(GetVertexDeclaration), S(SetFVF), S(GetFVF), S(CreateVertexShader), S(SetVertexShader), S(GetVertexShader),
    S(SetVertexShaderConstantF), S(GetVertexShaderConstantF), S(SetVertexShaderConstantI), S(GetVertexShaderConstantI), S(SetVertexShaderConstantB), S(GetVertexShaderConstantB),
    S(SetStreamSource), S(GetStreamSource), S(SetStreamSourceFreq), S(GetStreamSourceFreq), S(SetIndices), S(GetIndices), S(CreatePixelShader), S(SetPixelShader), S(GetPixelShader),
    S(SetPixelShaderConstantF), S(GetPixelShaderConstantF), S(SetPixelShaderConstantI), S(GetPixelShaderConstantI), S(SetPixelShaderConstantB), S(GetPixelShaderConstantB),
    S(DrawRectPatch), S(DrawTriPatch), S(DeletePatch), S(CreateQuery)
};

/* the boards take one session a process: the first device opens it, and every device after (a game's capability probe) shares it */
static g2_board *g_bd;
static g2r *g_r;
static int g_asked;

static int session(void)
{
    uint32_t st[G2_ST_WORDS];
    int n = 0;
    if (g_r) return 0;
    g2e_crash_handler();
    if (!g_bd && !(g_bd = g2e_connect("d3d9"))) return -1;
    if (!(g_r = g2r_create(g_bd))) return -1;
    if (g2_status(g_bd, st, &n) >= 0) {
        g_mode = g2_mode_of_flags(st[G2_ST_B_FLAGS]);
        g2sc_set_target(G2_B_FLAG_XOPS(st[G2_ST_B_FLAGS]));    /* the fragment-core operations this board B has: the compiler emits them */
        if (g2sc_target()) g2e_log("d3d9: board B runs SQRT / RCPM / RSQM %s, CLAMP %s", g2sc_target() & 1 ? "yes" : "no", g2sc_target() & 2 ? "yes" : "no");
    }
    g2_cursor_hide(g_bd);                                       /* what a killed program left on the screen */
    return 0;
}

/* volume textures are board B's 3D textures (on a board B built with them; GPU2_NO_3D=1 says no): the caps a game reads before it has a device ask the boards too, once */
int d3_has_3d(void)
{
    if (g2e_get_int("GPU2_NO_3D", 0)) return 0;
    if (!g_r && !g_asked) { g_asked = 1; session(); }
    return g_r && g2r_has_3d(g_r);
}

HRESULT d3_device_new(IDirect3D9 *d3d, HWND focus, DWORD flags, D3DPRESENT_PARAMETERS *pp, IDirect3DDevice9 **out)
{
    d3dev *d = (d3dev *)calloc(1, sizeof *d);
    if (!d) return E_OUTOFMEMORY;
    d->vtbl = d3_vt_device; d->refs = 1; d->d3d = d3d; d->pp = *pp; d->flags = flags;
    d->hwnd = pp->hDeviceWindow ? pp->hDeviceWindow : focus;
    IDirect3D9_AddRef(d3d);
    if (session() < 0) { free(d); return D3DERR_NOTAVAILABLE; }
    d->bd = g_bd; d->r = g_r; d->mode = g_mode;
    d->hold = g2e_get_int("GPU2_HOLD", 1);
    d->stretch = !strcmp(g2e_get("GPU2_SCALE", "fit"), "stretch");
    /* depth kept reversed (d3_zrev) needs board A's clip at z >= 0; GPU2_REVZ=0 keeps Direct3D's own count, for a comparison; 2 reverses whatever board A is there (one without
       that clip then draws what lies beyond the far plane instead of cutting it: for trying the library on such a board) */
    d->zrev_ok = g2e_get_int("GPU2_REVZ", 1) == 2 || (g2e_get_int("GPU2_REVZ", 1) && g2r_a_build(d->r) >= G2_A_ZCLIP0);
    { static int said; if (!said++) g2e_log("d3d9: depth is kept %s (board A build %d%s)", d->zrev_ok ? "reversed, far at 0" : "as Direct3D counts it", g2r_a_build(d->r), g2e_get_int("GPU2_REVZ", 1) ? "" : ", GPU2_REVZ=0"); }
    if (swap_create(d, &d->pp) < 0) { free(d); return D3DERR_OUTOFVIDEOMEMORY; }
    *pp = d->pp;
    d->cursor_x = d->cursor_y = -1;
    fit_window(d);
    state_defaults(d);
    d3_ref_set(&d->rt, d->backbuf);
    if (d->pp.EnableAutoDepthStencil) {
        d->auto_ds = d3_surface_new(d, d->backbuf->w, d->backbuf->h, d->pp.AutoDepthStencilFormat, D3DUSAGE_DEPTHSTENCIL, D3DPOOL_DEFAULT, 0);
        if (d->auto_ds) { d->auto_ds->binds = 1; ds_alloc(d, d->auto_ds); }   /* the device's own, as the back buffer */
        d3_ref_set(&d->ds, d->auto_ds);
    }
    *out = (IDirect3DDevice9 *)d;
    return D3D_OK;
}
