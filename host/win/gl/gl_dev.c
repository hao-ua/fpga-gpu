/* the Windows side of opengl32.dll: wgl, pixel formats, the default framebuffer, presentation.
 * gdi32's ChoosePixelFormat / SetPixelFormat / DescribePixelFormat / SwapBuffers call the wgl* functions of the opengl32.dll in the process by name, so these exports are how a game
 * reaches this library at all. The game's window stays on the PC for input; the picture goes to the HDMI monitor. */
#include "gl_internal.h"

gl_ctx *C;
gl_dev D;
static gl_ctx g_null;                                   /* current when no context is: calls before wglMakeCurrent must not crash, and draw nothing */
static gl_ctx *g_current;
static HDC g_cur_dc;

gl_ctx *gl_null_ctx(void) { return &g_null; }

/* ---------------------------------------------------------------- pixel formats
 * What B renders and scans out: colour RGBA8 or RGB565, depth of 24 or 16 bits (B's DEPTH_COMPONENT16 mode), stencil 8 in the same word */
typedef struct pf { int color, depth, stencil, dbl; } pf;
static const pf FORMATS[] = {
    { 32, 24, 8, 1 }, { 32, 24, 0, 1 }, { 32, 0, 0, 1 }, { 32, 24, 8, 0 }, { 32, 24, 0, 0 }, { 32, 0, 0, 0 },
    { 32, 16, 0, 1 }, { 32, 16, 8, 1 }, { 32, 16, 0, 0 }, { 32, 16, 8, 0 },
    { 16, 16, 0, 1 }, { 16, 16, 8, 1 }, { 16, 24, 8, 1 }, { 16, 24, 0, 1 }, { 16, 0, 0, 1 },
    { 16, 16, 0, 0 }, { 16, 16, 8, 0 }, { 16, 24, 8, 0 }, { 16, 24, 0, 0 }, { 16, 0, 0, 0 },
};
#define NFORMATS ((int)(sizeof FORMATS / sizeof FORMATS[0]))

int WINAPI wglDescribePixelFormat(HDC dc, int i, UINT size, LPPIXELFORMATDESCRIPTOR d)
{
    (void)dc;
    if (!d) return NFORMATS;
    if (i < 1 || i > NFORMATS || size < sizeof *d) return 0;
    memset(d, 0, sizeof *d);
    d->nSize = sizeof *d;
    d->nVersion = 1;
    d->dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | (FORMATS[i - 1].dbl ? PFD_DOUBLEBUFFER : 0) | PFD_SWAP_EXCHANGE;
    d->iPixelType = PFD_TYPE_RGBA;
    if (FORMATS[i - 1].color == 16) { d->cColorBits = 16; d->cRedBits = 5; d->cRedShift = 11; d->cGreenBits = 6; d->cGreenShift = 5; d->cBlueBits = 5; d->cBlueShift = 0; }
    else { d->cColorBits = 32; d->cRedBits = 8; d->cRedShift = 16; d->cGreenBits = 8; d->cGreenShift = 8; d->cBlueBits = 8; d->cBlueShift = 0; d->cAlphaBits = 8; d->cAlphaShift = 24; }
    d->cAccumBits = 64; d->cAccumRedBits = d->cAccumGreenBits = d->cAccumBlueBits = d->cAccumAlphaBits = 16;
    d->cDepthBits = (BYTE)FORMATS[i - 1].depth;
    d->cStencilBits = (BYTE)FORMATS[i - 1].stencil;
    d->iLayerType = PFD_MAIN_PLANE;
    return NFORMATS;
}

/* the closest: what is asked for of colour, then of depth (at least that many bits, the fewest above), stencil and alpha */
int WINAPI wglChoosePixelFormat(HDC dc, const PIXELFORMATDESCRIPTOR *w)
{
    int i, best = 1, score = -1, want16 = w->cColorBits && w->cColorBits <= 16 && !w->cAlphaBits;
    (void)dc;
    for (i = 1; i <= NFORMATS; i++) {
        const pf *f = &FORMATS[i - 1];
        int s = 0;
        if (!(w->dwFlags & PFD_DOUBLEBUFFER_DONTCARE) && !!(w->dwFlags & PFD_DOUBLEBUFFER) != f->dbl) continue;
        if (w->cDepthBits && !f->depth) continue;
        if (w->cStencilBits && !f->stencil) continue;
        s += (f->color == 16) == want16 ? 16 : 0;
        if (w->cDepthBits) s += f->depth >= w->cDepthBits ? (f->depth == w->cDepthBits || (w->cDepthBits > 16 && f->depth == 24) ? 8 : 6) : 0;
        else s += !f->depth ? 8 : 0;
        s += (w->cStencilBits ? 2 : !f->stencil);
        if (s > score) { score = s; best = i; }
    }
    return best;
}

BOOL WINAPI wglSetPixelFormat(HDC dc, int i, const PIXELFORMATDESCRIPTOR *d)
{
    (void)dc; (void)d;
    if (i < 1 || i > NFORMATS) return FALSE;
    D.pixel_format = i;
    g2e_log("gl: pixel format %d", i);
    return TRUE;
}

int WINAPI wglGetPixelFormat(HDC dc) { (void)dc; return D.pixel_format; }

/* ---------------------------------------------------------------- device */
int gl_dev_up(void)
{
    if (D.r) return 0;
    g2e_crash_handler();
    D.bd = g2e_connect("gl");
    if (!D.bd) return -1;
    D.r = g2r_create(D.bd);
    if (!D.r) { g2e_log("gl: cannot set up the render layer"); return -1; }
    D.hold = g2e_get_int("GPU2_HOLD", 1);
    D.stretch = !strcmp(g2e_get("GPU2_SCALE", "fit"), "stretch");
    D.swap_interval = 1;
    {
        uint32_t st[G2_ST_WORDS];
        if (g2_status(D.bd, st, NULL) == 0) {
            D.mode = g2_mode_of_flags(st[G2_ST_B_FLAGS]);
            g2sc_set_target(G2_B_FLAG_XOPS(st[G2_ST_B_FLAGS]));    /* the fragment-core operations this board B has: the compiler emits them */
            if (g2sc_target()) g2e_log("gl: board B runs SQRT / RCPM / RSQM %s, CLAMP %s", g2sc_target() & 1 ? "yes" : "no", g2sc_target() & 2 ? "yes" : "no");
        }
        D.mode_w = G2_MODE_W[D.mode]; D.mode_h = G2_MODE_H[D.mode];
    }
    return 0;
}

/* B renders at most 1 280 wide and scans out at most 1 280 x 800: a larger window renders smaller, and viewport, scissor, reads and copies scale, so the application keeps its coordinates (as in libGLESv2) */
void gl_dev_framebuffer(int vw, int vh)
{
    const pf *f = &FORMATS[(D.pixel_format ? D.pixel_format : 1) - 1];
    int i, w, h, maxw = g2e_get_int("GPU2_MAX_W", 1280);
    if (vw < 1) vw = 640;
    if (vh < 1) vh = 480;
    if (vw == D.vw && vh == D.vh && D.color[0]) return;
    if (maxw > 1280 || maxw < 1) maxw = 1280;
    w = vw; h = vh;
    if (w > maxw || h > 800) {
        double s = (double)maxw / w;
        if (800.0 / h < s) s = 800.0 / h;
        w = (int)(w * s + 0.5); h = (int)(h * s + 0.5);
        if (w > maxw) w = maxw;
        if (h > 800) h = 800;
    }
    g2r_end_pass(D.r);
    for (i = 0; i < 3; i++) {
        if (D.color[i]) g2r_image_destroy(D.r, D.color[i], D.color_base[i]);
        D.color[i] = 0;
        if (D.scan[i]) g2r_image_destroy(D.r, D.scan[i], D.scan_base[i]);
        D.scan[i] = 0;
    }
    if (D.ds) g2r_image_destroy(D.r, D.ds, D.ds_base);
    D.ds = 0;
    D.has_depth = f->depth != 0; D.has_stencil = f->stencil != 0; D.doublebuffer = f->dbl;
    D.color_fmt = f->color == 16 ? G2_FMT_RGB565 : G2_FMT_RGBA8; D.depth16 = f->depth == 16;
    if (g2_mode_select(D.bd, w, h, &D.mode) < 0) g2e_log("gl: SET_MODE for %d x %d: %s", w, h, g2_error_text(D.bd));
    D.mode_w = G2_MODE_W[D.mode]; D.mode_h = G2_MODE_H[D.mode];
    for (i = 0; i < 3; i++) {
        if (g2r_image_create(D.r, &D.color[i], &D.color_base[i], (uint32_t)w, (uint32_t)h, D.color_fmt, 1, 1, 1, G2_USE_RT | G2_USE_SCANOUT | G2_USE_TRANSFER | G2_USE_SAMPLED) < 0)
            g2e_log("gl: no room for the framebuffer");
    }
    if (f->depth || f->stencil) {
        g2r_image_create(D.r, &D.ds, &D.ds_base, (uint32_t)w, (uint32_t)h, G2_FMT_D24S8, 1, 1, 1, G2_USE_RT | G2_USE_TRANSFER);
        g2r_stencil_undefined(D.r, D.ds);
    }
    if (w != D.mode_w || h != D.mode_h) {
        for (i = 0; i < 3; i++) {
            g2r_image_create(D.r, &D.scan[i], &D.scan_base[i], (uint32_t)D.mode_w, (uint32_t)D.mode_h, G2_FMT_RGBA8, 1, 1, 1, G2_USE_SCANOUT | G2_USE_TRANSFER);
            g2_fill_image(D.bd, D.scan[i], G2_TF_RAW, 0x000000FFu, 0, 0, (uint32_t)D.mode_w, (uint32_t)D.mode_h, 0xF, 0, 0);
        }
    }
    D.fb_w = w; D.fb_h = h; D.vw = vw; D.vh = vh; D.back = 0; D.front = 0;
    g2e_log("gl: window %d x %d, framebuffer %d x %d, colour %d depth %d stencil %d double %d, HDMI mode %d", vw, vh, w, h, f->color, f->depth, f->stencil, f->dbl, D.mode);
}

static void window_size(HDC dc, int *w, int *h)
{
    HWND wnd = WindowFromDC(dc);
    RECT r;
    *w = 640; *h = 480;
    if (wnd && GetClientRect(wnd, &r) && r.right > 0 && r.bottom > 0) { *w = r.right; *h = r.bottom; }
}

void gl_cur_target(g2r_target *t)
{
    gl_fbo *f = C->fbo ? gl_fbo_lookup(C->fbo) : NULL;
    if (f && gl_fbo_target(f, t) == 0) return;
    memset(t, 0, sizeof *t);
    t->color = D.color[C->draw_buffer == GL_FRONT && D.doublebuffer ? D.front : D.back];
    t->ds = D.ds;
    t->width = (uint32_t)D.fb_w; t->height = (uint32_t)D.fb_h;
    gl_tg_bits(t, D.color_fmt);
    t->depth16 = D.depth16;
}

/* what reads read: GL_EXT_framebuffer_blit gives them a framebuffer of their own */
void gl_read_target(g2r_target *t)
{
    GLuint keep = C->fbo;
    C->fbo = C->read_fbo;
    gl_cur_target(t);
    C->fbo = keep;
}

void gl_scale(double *kx, double *ky)
{
    gl_fbo *f = C->fbo ? gl_fbo_lookup(C->fbo) : NULL;
    g2r_target t;
    *kx = *ky = 1.0;
    if ((f && gl_fbo_target(f, &t) == 0) || !D.vw || !D.vh) return;
    *kx = (double)D.fb_w / D.vw; *ky = (double)D.fb_h / D.vh;
}

int gl_scissor_px(const g2r_target *t, int *b)
{
    double kx, ky;
    gl_scale(&kx, &ky);
    b[0] = 0; b[1] = 0; b[2] = (int)t->width; b[3] = (int)t->height;
    if (C->scissor_test) {
        int x0 = (int)floor(C->sc[0] * kx), y0 = (int)floor(C->sc[1] * ky), x1 = (int)ceil((C->sc[0] + (double)C->sc[2]) * kx), y1 = (int)ceil((C->sc[1] + (double)C->sc[3]) * ky);
        if (x0 > b[0]) b[0] = x0;
        if (y0 > b[1]) b[1] = y0;
        if (x1 < b[2]) b[2] = x1;
        if (y1 < b[3]) b[3] = y1;
    }
    if (b[2] < b[0]) b[2] = b[0];
    if (b[3] < b[1]) b[3] = b[1];
    return b[2] > b[0] && b[3] > b[1];
}

void gl_bind_target(void)
{
    g2r_target t;
    gl_cur_target(&t);
    g2r_set_target(D.r, &t);
}

void gl_swap(void)
{
    uint16_t image;
    int hold = D.hold && D.swap_interval;
    int w, h;
    if (!C->live) return;
    gl_im_flush();
    gl_bind_target();
    g2r_end_pass(D.r);
    image = D.color[D.back];
    if (D.fb_w == D.mode_w && D.fb_h == D.mode_h) {
        g2r_present(D.r, image, hold);
    } else {
        int dw = D.mode_w, dh = D.mode_h, dx = 0, dy = 0;
        if (!D.stretch) {
            int sx = D.mode_w / D.fb_w, sy = D.mode_h / D.fb_h, s = sx < sy ? sx : sy;
            if (s < 1) s = 1;
            dw = D.fb_w * s; dh = D.fb_h * s;
            if (dw > D.mode_w) dw = D.mode_w;
            if (dh > D.mode_h) dh = D.mode_h;
            dx = (D.mode_w - dw) / 2; dy = (D.mode_h - dh) / 2;
        }
        D.scan_idx = (D.scan_idx + 1) % 3;
        g2_copy_image(D.bd, image, D.scan[D.scan_idx], 0, 0, (uint32_t)D.fb_w, (uint32_t)D.fb_h, (uint32_t)dx, (uint32_t)dy, (uint32_t)dw, (uint32_t)dh, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        g2r_present(D.r, D.scan[D.scan_idx], hold);
    }
    D.frame++;
    g2r_stats_tick(D.r);
    if (D.doublebuffer) {
        D.front = D.back;
        D.back = (D.back + 1) % 3;
        /* PFD_SWAP_EXCHANGE: the back buffer is undefined after a swap, and games that repaint everything (Quake 3 clears no colour) would pay a full band load for it each frame */
        if (!g2e_get_int("GPU2_KEEP_BACK", 0)) g2r_discard_color(D.r, D.color[D.back]);
    }
    window_size(g_cur_dc, &w, &h);
    if (w != D.vw || h != D.vh) gl_dev_framebuffer(w, h);
    if (g2e_get("GPU2_DUMP", NULL)) {                           /* frames to files, to check them without the monitor */
        static int every = -1;
        if (every < 0) every = g2e_get_int("GPU2_DUMP_EVERY", 120);
        if (D.frame % (uint32_t)every == 1) {
            char path[MAX_PATH];
            uint8_t *px = (uint8_t *)malloc((size_t)D.fb_w * D.fb_h * 4);
            FILE *f;
            snprintf(path, sizeof path, "%s\\frame_%05u.bmp", g2e_get("GPU2_DUMP", ""), D.frame);
            f = fopen(path, "wb");
            if (px && f) {
                BITMAPFILEHEADER fh;
                BITMAPINFOHEADER ih;
                g2_fence(D.bd, 5.0);
                g2_read_image(D.bd, image, G2_TF_BGRA8888, 0, 0, (uint32_t)D.fb_w, (uint32_t)D.fb_h, 0, 0, G2_FMT_RGBA8, 0, 0, px);
                memset(&fh, 0, sizeof fh); memset(&ih, 0, sizeof ih);
                fh.bfType = 0x4D42; fh.bfOffBits = sizeof fh + sizeof ih; fh.bfSize = fh.bfOffBits + (DWORD)D.fb_w * D.fb_h * 4;
                ih.biSize = sizeof ih; ih.biWidth = D.fb_w; ih.biHeight = D.fb_h; ih.biPlanes = 1; ih.biBitCount = 32; ih.biCompression = BI_RGB;    /* bottom-up, as GL */
                fwrite(&fh, sizeof fh, 1, f); fwrite(&ih, sizeof ih, 1, f); fwrite(px, 4, (size_t)D.fb_w * D.fb_h, f);
            }
            if (f) fclose(f);
            free(px);
        }
    }
}

/* ---------------------------------------------------------------- contexts */
void gl_ctx_init(gl_ctx *c)
{
    int i, u, f;
    memset(c, 0, sizeof *c);
    gl_extra_init(c);
    c->matrix_mode = GL_MODELVIEW;
    gl_ident(c->mv[0]); gl_ident(c->pr[0]);
    c->color[0] = c->color[1] = c->color[2] = c->color[3] = 1.0f;
    c->normal[2] = 1.0f;
    c->raster_pos[3] = 1.0f; c->raster_valid = 1;
    c->raster_color[0] = c->raster_color[1] = c->raster_color[2] = c->raster_color[3] = 1.0f;
    c->dither = 1;
    c->edge_flag = 1;
    c->cull_mode = GL_BACK; c->front_face = GL_CCW; c->shade_model = GL_SMOOTH; c->depth_func = GL_LESS; c->alpha_func = GL_ALWAYS;
    c->poly_mode_front = c->poly_mode_back = GL_FILL;
    c->blend_src = c->blend_src_a = GL_ONE; c->blend_dst = c->blend_dst_a = GL_ZERO; c->blend_eq = c->blend_eq_a = GL_FUNC_ADD;
    c->depth_mask = 1; c->color_mask = 15; c->clear_depth = 1.0f;
    c->depth_near = 0.0f; c->depth_far = 1.0f; c->line_width = 1.0f; c->point_size = 1.0f;
    for (f = 0; f < 2; f++) {
        c->st_func[f] = GL_ALWAYS; c->st_fail[f] = c->st_zfail[f] = c->st_zpass[f] = GL_KEEP;
        c->st_vmask[f] = c->st_wmask[f] = 0xFFFFFFFFu;
        c->mat[f].ambient[0] = c->mat[f].ambient[1] = c->mat[f].ambient[2] = 0.2f; c->mat[f].ambient[3] = 1.0f;
        c->mat[f].diffuse[0] = c->mat[f].diffuse[1] = c->mat[f].diffuse[2] = 0.8f; c->mat[f].diffuse[3] = 1.0f;
        c->mat[f].specular[3] = 1.0f; c->mat[f].emission[3] = 1.0f;
    }
    c->fog_mode = GL_EXP; c->fog_density = 1.0f; c->fog_end = 1.0f;
    for (i = 0; i < MAX_LIGHTS; i++) {
        gl_light *l = &c->light[i];
        l->ambient[3] = l->diffuse[3] = l->specular[3] = 1.0f;
        l->position[2] = 1.0f; l->spot_dir[2] = -1.0f; l->spot_cut = 180.0f; l->atten[0] = 1.0f;
    }
    c->light[0].diffuse[0] = c->light[0].diffuse[1] = c->light[0].diffuse[2] = 1.0f;
    c->light[0].specular[0] = c->light[0].specular[1] = c->light[0].specular[2] = 1.0f;
    c->lm_ambient[0] = c->lm_ambient[1] = c->lm_ambient[2] = 0.2f; c->lm_ambient[3] = 1.0f;
    c->lm_color_control = GL_SINGLE_COLOR;
    c->cm_face = GL_FRONT_AND_BACK; c->cm_mode = GL_AMBIENT_AND_DIFFUSE;
    for (u = 0; u < MAX_UNITS; u++) {
        gl_unit *t = &c->unit[u];
        t->env_mode = GL_MODULATE; t->tc[3] = 1.0f;
        t->comb_rgb = t->comb_a = GL_MODULATE; t->scale_rgb = t->scale_a = 1.0f;
        t->src_rgb[0] = t->src_a[0] = GL_TEXTURE; t->src_rgb[1] = t->src_a[1] = GL_PREVIOUS; t->src_rgb[2] = t->src_a[2] = GL_CONSTANT;
        t->op_rgb[0] = t->op_rgb[1] = GL_SRC_COLOR; t->op_rgb[2] = GL_SRC_ALPHA; t->op_a[0] = t->op_a[1] = t->op_a[2] = GL_SRC_ALPHA;
        for (f = 0; f < 4; f++) t->gen_mode[f] = GL_EYE_LINEAR;
        t->gen_obj[0][0] = t->gen_eye[0][0] = 1.0f; t->gen_obj[1][1] = t->gen_eye[1][1] = 1.0f;
        gl_ident(t->mat[0]);
        c->a_tc[u].size = 4; c->a_tc[u].type = GL_FLOAT;
    }
    c->a_vertex.size = 4; c->a_vertex.type = GL_FLOAT; c->a_color.size = 4; c->a_color.type = GL_FLOAT; c->a_normal.size = 3; c->a_normal.type = GL_FLOAT;
    c->a_fog.size = 1; c->a_fog.type = GL_FLOAT; c->a_sec.size = 3; c->a_sec.type = GL_FLOAT;
    c->fog_src = GL_FRAGMENT_DEPTH;
    c->multisample = 1; c->sample_val = 1.0f;
    c->pt_max = 64.0f; c->pt_fade = 1.0f; c->pt_att[0] = 1.0f;
    c->sec_color[3] = 1.0f;
    c->un_align = c->pk_align = 4;
    c->draw_buffer = GL_BACK; c->read_buffer = GL_BACK;
    for (u = 0; u < MAX_ATTRIBS; u++) { c->attr[u][3] = 1.0f; c->a_attr[u].size = 4; c->a_attr[u].type = GL_FLOAT; }
    c->sprite_origin = GL_UPPER_LEFT;
    c->pixel_zoom[0] = c->pixel_zoom[1] = 1.0f;
}

HGLRC WINAPI wglCreateContext(HDC dc)
{
    gl_ctx *c = (gl_ctx *)malloc(sizeof *c);
    (void)dc;
    if (!c) return NULL;
    gl_ctx_init(c);
    g2e_log("gl: wglCreateContext -> %p", (void *)c);
    return (HGLRC)c;
}

BOOL WINAPI wglDeleteContext(HGLRC h)
{
    gl_ctx *c = (gl_ctx *)h;
    if (!c) return FALSE;
    if (c == g_current) { C = &g_null; g_current = NULL; }
    free(c->im_v); free(c->im_idx); free(c->im_ef); free(c->im_tri); free(c);
    return TRUE;
}

BOOL WINAPI wglMakeCurrent(HDC dc, HGLRC h)
{
    gl_ctx *c = (gl_ctx *)h;
    int w, h2, first;
    if (g_current && g_current->im_n) gl_im_flush();
    if (!c) { C = &g_null; g_current = NULL; g_cur_dc = NULL; return TRUE; }
    if (gl_dev_up() < 0) return FALSE;
    first = !c->live;
    C = g_current = c;
    g_cur_dc = dc;
    c->live = 1;
    window_size(dc, &w, &h2);
    gl_dev_framebuffer(w, h2);
    if (first) { c->vp[2] = w; c->vp[3] = h2; c->sc[2] = w; c->sc[3] = h2; }
    gl_bind_target();
    g2e_log("gl: wglMakeCurrent %p, window %d x %d", (void *)c, w, h2);
    return TRUE;
}

HGLRC WINAPI wglGetCurrentContext(void) { return (HGLRC)g_current; }
HDC WINAPI wglGetCurrentDC(void) { return g_cur_dc; }
BOOL WINAPI wglShareLists(HGLRC a, HGLRC b) { (void)a; (void)b; return TRUE; }             /* objects are process-wide here */
BOOL WINAPI wglCopyContext(HGLRC a, HGLRC b, UINT m) { (void)a; (void)b; (void)m; return FALSE; }
HGLRC WINAPI wglCreateLayerContext(HDC dc, int l) { return l == 0 ? wglCreateContext(dc) : NULL; }
BOOL WINAPI wglSwapBuffers(HDC dc) { (void)dc; if (C == &g_null) return FALSE; gl_swap(); return TRUE; }
BOOL WINAPI wglSwapLayerBuffers(HDC dc, UINT p) { (void)p; return wglSwapBuffers(dc); }
DWORD WINAPI wglSwapMultipleBuffers(UINT n, const WGLSWAP *s) { UINT i; for (i = 0; i < n; i++) wglSwapBuffers(s[i].hdc); return n; }
BOOL WINAPI wglDescribeLayerPlane(HDC a, int b, int c, UINT d, LPLAYERPLANEDESCRIPTOR e) { (void)a; (void)b; (void)c; (void)d; (void)e; return FALSE; }
int WINAPI wglGetLayerPaletteEntries(HDC a, int b, int c, int d, COLORREF *e) { (void)a; (void)b; (void)c; (void)d; (void)e; return 0; }
int WINAPI wglSetLayerPaletteEntries(HDC a, int b, int c, int d, const COLORREF *e) { (void)a; (void)b; (void)c; (void)d; (void)e; return 0; }
BOOL WINAPI wglRealizeLayerPalette(HDC a, int b, BOOL c) { (void)a; (void)b; (void)c; return FALSE; }
BOOL WINAPI wglUseFontOutlinesA(HDC a, DWORD b, DWORD c, DWORD d, FLOAT e, FLOAT f, int g, LPGLYPHMETRICSFLOAT h) { (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; (void)g; (void)h; return FALSE; }
BOOL WINAPI wglUseFontOutlinesW(HDC a, DWORD b, DWORD c, DWORD d, FLOAT e, FLOAT f, int g, LPGLYPHMETRICSFLOAT h) { (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; (void)g; (void)h; return FALSE; }

/* wgl extensions */
static const char *WINAPI gl_wgl_ext_arb(HDC dc) { (void)dc; return "WGL_ARB_extensions_string WGL_EXT_extensions_string WGL_EXT_swap_control"; }
static const char *WINAPI gl_wgl_ext(void) { return "WGL_ARB_extensions_string WGL_EXT_extensions_string WGL_EXT_swap_control"; }
static BOOL WINAPI gl_swap_interval(int i) { D.swap_interval = i; g2e_log("gl: wglSwapIntervalEXT(%d)", i); return TRUE; }
static int WINAPI gl_get_swap_interval(void) { return D.swap_interval; }

PROC WINAPI wglGetProcAddress(LPCSTR name)
{
    static const struct { const char *n; PROC p; } T[] = {
#define EXT(n, s) { s, (PROC)n },
#include "gl_ext_list.h"
#undef EXT
        { "wglGetExtensionsStringARB", (PROC)gl_wgl_ext_arb }, { "wglGetExtensionsStringEXT", (PROC)gl_wgl_ext },
        { "wglSwapIntervalEXT", (PROC)gl_swap_interval }, { "wglGetSwapIntervalEXT", (PROC)gl_get_swap_interval },
    };
    size_t i;
    for (i = 0; i < sizeof T / sizeof T[0]; i++) if (!strcmp(T[i].n, name)) return T[i].p;
    g2e_log("gl: wglGetProcAddress(%s): not provided", name);
    return NULL;
}

PROC WINAPI wglGetDefaultProcAddress(LPCSTR name) { return wglGetProcAddress(name); }

BOOL WINAPI DllMain(HINSTANCE h, DWORD why, LPVOID r)
{
    (void)r;
    if (why == DLL_PROCESS_ATTACH) {
        DisableThreadLibraryCalls(h);
        gl_ctx_init(&g_null);
        C = &g_null;
        /* before the game's first window. Unaware, a game on a scaled desktop gets virtual coordinates: SDL 1.2's windowed mouse (warp to the centre, read the difference) then drifts */
        SetProcessDPIAware();
    }
    return TRUE;
}
