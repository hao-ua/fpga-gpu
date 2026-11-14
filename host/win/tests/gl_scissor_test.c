/* gl_scissor_test — does the scissor apply to depth writes? Plain GL, no clears inside a pass: a near quad is drawn through a scissor, then a far quad over the whole area. Where the near quad wrote depth the far
 * one loses. Variants: colour writes on / off, depth func ALWAYS / LESS. Each one ends the pass before the far quad, and in a second set stays in one pass. */
#include <windows.h>
#include <GL/gl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 640
#define H 480
static int g_bad;
static HDC g_dc;

static void quad(float x0, float y0, float x1, float y1, float z)
{
    glBegin(GL_QUADS);
    glVertex3f(x0, y0, z); glVertex3f(x1, y0, z); glVertex3f(x1, y1, z); glVertex3f(x0, y1, z);
    glEnd();
}

static void px(int x, int y, const char *what, int r, int g, int b)
{
    uint8_t c[4];
    glReadPixels(x, y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, c);
    if (abs(c[0] - r) > 2 || abs(c[1] - g) > 2 || abs(c[2] - b) > 2) { g_bad++; printf("FAIL %s: (%d, %d) = %d %d %d, expected %d %d %d\n", what, x, y, c[0], c[1], c[2], r, g, b); }
    else printf("ok   %s\n", what);
}

static void scene(int colormask, GLenum func, int pass_break, const char *name)
{
    char a[96], b[96];
    glClearColor(0.2f, 0.4f, 0.6f, 1.0f);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(func);
    if (!colormask) glColorMask(0, 0, 0, 0);
    glColor3f(1, 0, 0);
    glEnable(GL_SCISSOR_TEST);
    glScissor(100, 100, 100, 100);
    quad(100, 100, 300, 200, 0.8f);                             /* depth writes reach x < 200 only */
    glDisable(GL_SCISSOR_TEST);
    glColorMask(1, 1, 1, 1);
    glDepthFunc(GL_LESS);
    if (pass_break) glFinish();
    glColor3f(0, 1, 0);
    quad(100, 100, 300, 200, 0.2f);                             /* far */
    glDisable(GL_DEPTH_TEST);
    snprintf(a, sizeof a, "%s: inside the scissor the near quad wrote depth", name);
    snprintf(b, sizeof b, "%s: outside the scissor it wrote none", name);
    if (colormask) px(150, 150, a, 255, 0, 0);
    else px(150, 150, a, 51, 102, 153);
    px(250, 150, b, 0, 255, 0);
    SwapBuffers(g_dc);
}

/* a depth clear by hand, then by glClear: near quad over the whole area, depth raised back to 1 in the scissor, far quad over the whole area */
static void clear_scene(int by_hand, const char *name, GLbitfield first)
{
    char a[96], b[96];
    glClearColor(0.2f, 0.4f, 0.6f, 1.0f);
    glClearDepth(1.0);
    glClear(first);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glColor3f(1, 0, 0);
    quad(100, 100, 300, 200, 0.8f);
    glEnable(GL_SCISSOR_TEST);
    glScissor(100, 100, 100, 100);
    if (by_hand) {
        glDepthFunc(GL_ALWAYS);
        glColorMask(0, 0, 0, 0);
        quad(0, 0, W, H, -1.0f);
        glColorMask(1, 1, 1, 1);
        glDepthFunc(GL_LESS);
    } else {
        glClear(GL_DEPTH_BUFFER_BIT);
    }
    glDisable(GL_SCISSOR_TEST);
    glColor3f(0, 1, 0);
    quad(100, 100, 300, 200, 0.2f);
    glDisable(GL_DEPTH_TEST);
    snprintf(a, sizeof a, "%s: inside the scissor depth was cleared", name);
    snprintf(b, sizeof b, "%s: outside it was kept", name);
    px(150, 150, a, 0, 255, 0);
    px(250, 150, b, 255, 0, 0);
    SwapBuffers(g_dc);
}

/* the edges of a depth clear inside a pass (the clear's quad covers the scissor box alone, not the target): the near quad lies all around the box, the far quad shows through where depth was
   cleared — the box's corner pixels inside, the pixels next to each side outside */
static void clear_edges(int x, int y, int w, int h, const char *name)
{
    char a[96];
    int i;
    const int in[4][2] = { {x, y}, {x + w - 1, y}, {x, y + h - 1}, {x + w - 1, y + h - 1} };
    const int out[4][2] = { {x - 1, y + h / 2}, {x + w, y + h / 2}, {x + w / 2, y - 1}, {x + w / 2, y + h} };
    glClearColor(0.2f, 0.4f, 0.6f, 1.0f);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glColor3f(1, 0, 0);
    quad(50, 50, 350, 250, 0.8f);
    glFlush();                                                  /* into the back buffer: the pass goes on, the picture is the same either way */
    glEnable(GL_SCISSOR_TEST);
    glScissor(x, y, w, h);
    glClear(GL_DEPTH_BUFFER_BIT);
    glDisable(GL_SCISSOR_TEST);
    glColor3f(0, 1, 0);
    quad(50, 50, 350, 250, 0.2f);
    glDisable(GL_DEPTH_TEST);
    for (i = 0; i < 4; i++) { snprintf(a, sizeof a, "%s: corner %d of the box cleared", name, i); px(in[i][0], in[i][1], a, 0, 255, 0); }
    for (i = 0; i < 4; i++) { snprintf(a, sizeof a, "%s: side %d, the pixel outside kept", name, i); px(out[i][0], out[i][1], a, 255, 0, 0); }
    SwapBuffers(g_dc);
}

/* a colour clear of a scissor box inside a pass (a quad of the clear program, no pass end) against the same clear as a pass's own (the FLUSH's clear): a 16 x 16 block of the box must be the
   same bytes, with and without dither, under any colour mask; the pixel left of the box keeps the background, the quad that opened the pass keeps its colour */
static void color_clear(const char *name, float r, float g, float b, float a, int dither, int mask)
{
    static uint8_t blk[2][16 * 16 * 4];
    uint8_t out[2][4], far_[4];
    char w[128];
    int pass, i, diff = 0;
    for (pass = 0; pass < 2; pass++) {
        glDisable(GL_SCISSOR_TEST); glDisable(GL_DEPTH_TEST);
        if (dither) glEnable(GL_DITHER); else glDisable(GL_DITHER);
        glColorMask(1, 1, 1, 1);
        glClearColor(0.25f, 0.5f, 0.75f, 1.0f); glClearDepth(1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (pass) { glColor3f(1, 0, 0); quad(400, 300, 420, 320, 0.5f); }      /* the pass has a draw: the clear below is a quad */
        glColorMask((GLboolean)(mask & 1), (GLboolean)(mask >> 1 & 1), (GLboolean)(mask >> 2 & 1), (GLboolean)(mask >> 3 & 1));
        glEnable(GL_SCISSOR_TEST);
        glScissor(101, 67, 53, 41);
        glClearColor(r, g, b, a);
        glClear(GL_COLOR_BUFFER_BIT);
        glDisable(GL_SCISSOR_TEST);
        glColorMask(1, 1, 1, 1);
        glReadPixels(120, 80, 16, 16, GL_RGBA, GL_UNSIGNED_BYTE, blk[pass]);
        glReadPixels(100, 80, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, out[pass]);
        if (pass) glReadPixels(410, 310, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, far_);
        SwapBuffers(g_dc);
    }
    for (i = 0; i < 16 * 16 * 4; i++) if (blk[0][i] != blk[1][i]) diff++;
    snprintf(w, sizeof w, "%s: the box as a quad = the box as the pass's clear", name);
    if (diff) { g_bad++; printf("FAIL %s: %d of 1024 bytes differ, first pixel %d %d %d %d against %d %d %d %d\n", w, diff, blk[1][0], blk[1][1], blk[1][2], blk[1][3], blk[0][0], blk[0][1], blk[0][2], blk[0][3]); }
    else printf("ok   %s (%d %d %d %d)\n", w, blk[1][0], blk[1][1], blk[1][2], blk[1][3]);
    snprintf(w, sizeof w, "%s: outside the box", name);
    if (memcmp(out[0], out[1], 4)) { g_bad++; printf("FAIL %s: %d %d %d %d against %d %d %d %d\n", w, out[1][0], out[1][1], out[1][2], out[1][3], out[0][0], out[0][1], out[0][2], out[0][3]); }
    else printf("ok   %s\n", w);
    snprintf(w, sizeof w, "%s: the quad that opened the pass", name);
    if (far_[0] < 250 || far_[1] > 5 || far_[2] > 5) { g_bad++; printf("FAIL %s: %d %d %d\n", w, far_[0], far_[1], far_[2]); }
    else printf("ok   %s\n", w);
}

/* the draw's rectangle on board A (board A sets up, bins and walks a draw inside its scissor box). A draw through a box must leave exactly the picture the same draw leaves without it,
   cut to the box: the box and an 8-pixel frame around it are read back from both. kind 0 smooth triangles, 1 lines of width 3, 2 points of size 5, all blended (a quad sent twice would show); the vertices
   come from a fixed generator and fall up to m pixels around the box, so most primitives cross its sides. vp = the box is the VIEWPORT instead (the driver cuts a draw to its viewport with the same
   rectangle): the projection maps the same coordinates to the same pixels, the geometry reaches far beyond the viewport */
static unsigned g_rng;
static float rnd(float lo, float hi) { g_rng = g_rng * 1664525u + 1013904223u; return lo + (hi - lo) * (float)(g_rng >> 8 & 0xFFFF) / 65535.0f; }

static void rect_draw(int kind, int x, int y, int w, int h)
{
    int i, n = kind == 0 ? 3 * 24 : kind == 1 ? 2 * 40 : 120;
    float m = kind == 0 ? 120.0f : 40.0f;
    g_rng = 12345u + (unsigned)kind;
    if (kind == 1) glLineWidth(3.0f);
    if (kind == 2) glPointSize(5.0f);
    glBegin(kind == 0 ? GL_TRIANGLES : kind == 1 ? GL_LINES : GL_POINTS);
    for (i = 0; i < n; i++) {
        float r = rnd(0.1f, 1.0f), g = rnd(0.1f, 1.0f), b = rnd(0.1f, 1.0f);
        float vx = rnd((float)x - m, (float)(x + w) + m), vy = rnd((float)y - m, (float)(y + h) + m);
        glColor4f(r, g, b, 0.6f);
        glVertex2f(vx, vy);
    }
    glEnd();
    glLineWidth(1.0f); glPointSize(1.0f);
}

static void rect_same(int kind, int vp, int x, int y, int w, int h, const char *name)
{
    enum { M = 8, BMAX = 128 + 2 * M };
    static uint8_t blk[2][BMAX * BMAX * 4];
    const int bx0 = x < 0 ? 0 : x, by0 = y < 0 ? 0 : y, bx1 = x + w > W ? W : x + w, by1 = y + h > H ? H : y + h;     /* the box cut to the window */
    const int rx0 = bx0 - M < 0 ? 0 : bx0 - M, ry0 = by0 - M < 0 ? 0 : by0 - M, rx1 = bx1 + M > W ? W : bx1 + M, ry1 = by1 + M > H ? H : by1 + M;
    const int rw = rx1 - rx0, rh = ry1 - ry0;
    int pass, i, j, c, bad_in = 0, bad_out = 0, lit_in = 0, lit_out = 0, npx_in = 0;
    char a[160];
    for (pass = 0; pass < 2; pass++) {
        glDisable(GL_DEPTH_TEST); glDisable(GL_SCISSOR_TEST);
        glClearColor(0.2f, 0.4f, 0.6f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        if (pass && vp) {
            glViewport(x, y, w, h);
            glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(x, x + w, y, y + h, -1, 1); glMatrixMode(GL_MODELVIEW);
        } else if (pass) { glEnable(GL_SCISSOR_TEST); glScissor(x, y, w, h); }
        rect_draw(kind, x, y, w, h);
        glDisable(GL_SCISSOR_TEST); glDisable(GL_BLEND);
        if (pass && vp) {
            glViewport(0, 0, W, H);
            glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, W, 0, H, -1, 1); glMatrixMode(GL_MODELVIEW);
        }
        glReadPixels(rx0, ry0, rw, rh, GL_RGBA, GL_UNSIGNED_BYTE, blk[pass]);
        SwapBuffers(g_dc);
    }
    for (j = 0; j < rh; j++) for (i = 0; i < rw; i++) {
        const uint8_t *p0 = &blk[0][(j * rw + i) * 4], *p1 = &blk[1][(j * rw + i) * 4];
        const int in = rx0 + i >= bx0 && rx0 + i < bx1 && ry0 + j >= by0 && ry0 + j < by1;
        const int bg0 = abs(p0[0] - 51) <= 2 && abs(p0[1] - 102) <= 2 && abs(p0[2] - 153) <= 2;
        const int bg1 = abs(p1[0] - 51) <= 2 && abs(p1[1] - 102) <= 2 && abs(p1[2] - 153) <= 2;
        int same = 1;
        for (c = 0; c < 3; c++) if (p0[c] != p1[c]) same = 0;
        if (in) { npx_in++; lit_in += !bg0; bad_in += !same; }
        else    { lit_out += !bg0; bad_out += !bg1; }
    }
    /* a viewport's picture comes through another projection: a vertex may land one sub-pixel step aside, so a few edge pixels may differ; a scissor's must be the same bytes */
    snprintf(a, sizeof a, "%s: inside the box the picture of the whole draw (%d of %d pixels drawn)", name, lit_in, npx_in);
    if (bad_in > (vp ? npx_in / 32 : 0) || (lit_in == 0 && npx_in > 4)) { g_bad++; printf("FAIL %s: %d pixels differ\n", a, bad_in); }
    else if (bad_in) printf("ok   %s, %d differ by the projection\n", a, bad_in);
    else printf("ok   %s\n", a);
    snprintf(a, sizeof a, "%s: nothing outside the box (%d pixels of the frame around it drawn without it)", name, lit_out);
    if (bad_out) { g_bad++; printf("FAIL %s: %d pixels drawn\n", a, bad_out); }
    else printf("ok   %s\n", a);
}

static double now_ms(void) { LARGE_INTEGER f, c; QueryPerformanceFrequency(&f); QueryPerformanceCounter(&c); return 1000.0 * (double)c.QuadPart / (double)f.QuadPart; }

/* gl_scissor_test perf: what a box saves. 60 frames of 24 blended whole-window quads (vp: a viewport of that box, the quads reaching 4 times beyond it); a board A that does not bound a draw by its box
   walks, sends and shades every one of them whole. The times come in steps of the display's frame (a swap waits for it) */
static void perf(const char *name, int sx, int sy, int sw, int sh, int vp)
{
    int f, i;
    double t0 = 0;
    for (f = -4; f < 60; f++) {
        if (f == 0) { glFinish(); t0 = now_ms(); }
        glDisable(GL_SCISSOR_TEST);
        glClearColor(0.2f, 0.4f, 0.6f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (vp) glViewport(sx, sy, sw, sh);
        else if (sw) { glEnable(GL_SCISSOR_TEST); glScissor(sx, sy, sw, sh); }
        glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        for (i = 0; i < 24; i++) {
            glColor4f(0.1f * (float)(i % 8 + 1), 1.0f - 0.1f * (float)(i % 8), 0.5f, 0.3f);
            if (vp) quad(-1.5f * W, -1.5f * H, 2.5f * W, 2.5f * H, 0.0f); else quad(0, 0, W, H, 0.0f);
        }
        glDisable(GL_BLEND); glDisable(GL_SCISSOR_TEST);
        if (vp) glViewport(0, 0, W, H);
        SwapBuffers(g_dc);
    }
    glFinish();
    printf("perf %s: %.2f ms a frame\n", name, (now_ms() - t0) / 60.0);
}

static LRESULT CALLBACK wndproc(HWND h, UINT m, WPARAM w, LPARAM l) { return DefWindowProcA(h, m, w, l); }

/* gl_scissor_test [d16] [perf]: d16 = a 16-bit depth buffer without stencil (a depth clear inside a pass is a quad there too); perf = the frame times of draws through a box, after the checks */
int main(int argc, char **argv)
{
    int d16 = 0, do_perf = 0, ai;
    for (ai = 1; ai < argc; ai++) { if (!strcmp(argv[ai], "d16")) d16 = 1; if (!strcmp(argv[ai], "perf")) do_perf = 1; }
    WNDCLASSA wc;
    HWND hwnd;
    HGLRC rc;
    PIXELFORMATDESCRIPTOR pfd;
    int pf;
    RECT r = { 0, 0, W, H };
    memset(&wc, 0, sizeof wc);
    wc.lpfnWndProc = wndproc; wc.lpszClassName = "gl_sc"; wc.hInstance = GetModuleHandleA(NULL); wc.style = CS_OWNDC;
    RegisterClassA(&wc);
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    hwnd = CreateWindowA("gl_sc", "gl_sc", WS_OVERLAPPEDWINDOW | WS_VISIBLE, 40, 40, r.right - r.left, r.bottom - r.top, NULL, NULL, wc.hInstance, NULL);
    g_dc = GetDC(hwnd);
    memset(&pfd, 0, sizeof pfd);
    pfd.nSize = sizeof pfd; pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA; pfd.cColorBits = 32; pfd.cDepthBits = d16 ? 16 : 24; pfd.cStencilBits = d16 ? 0 : 8;
    pf = ChoosePixelFormat(g_dc, &pfd);
    SetPixelFormat(g_dc, pf, &pfd);
    rc = wglCreateContext(g_dc);
    wglMakeCurrent(g_dc, rc);
    glViewport(0, 0, W, H);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, W, 0, H, -1, 1);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    scene(1, GL_LESS, 0, "colour on, LESS, one pass");
    scene(0, GL_LESS, 0, "colour off, LESS, one pass");
    scene(0, GL_ALWAYS, 0, "colour off, ALWAYS, one pass");
    scene(1, GL_ALWAYS, 0, "colour on, ALWAYS, one pass");
    scene(1, GL_LESS, 1, "colour on, LESS, two passes");
    scene(0, GL_ALWAYS, 1, "colour off, ALWAYS, two passes");
    clear_scene(1, "clear by hand", GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    clear_scene(0, "glClear", GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    clear_scene(0, "glClear, first clear with stencil", GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glFinish();
    clear_scene(0, "glClear after glFinish", GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    {   /* no swap in between: the reads of the previous scene ended its pass */
        int k;
        for (k = 0; k < 3; k++) {
            glClearColor(0.2f, 0.4f, 0.6f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glColor3f(1, 0, 0); quad(100, 100, 300, 200, 0.8f);
            glEnable(GL_SCISSOR_TEST);
            glScissor(100, 100, 100, 100);
            glClear(GL_DEPTH_BUFFER_BIT);
            glDisable(GL_SCISSOR_TEST);
            glColor3f(0, 1, 0); quad(100, 100, 300, 200, 0.2f);
            glDisable(GL_DEPTH_TEST);
            px(150, 150, "repeat without a swap: inside", 0, 255, 0);
            px(250, 150, "repeat without a swap: outside", 255, 0, 0);
        }
    }
    clear_edges(100, 100, 100, 100, "clear edges, 100 x 100");
    clear_edges(101, 103, 37, 5, "clear edges, 37 x 5");
    clear_edges(233, 181, 1, 1, "clear edges, one pixel");
    clear_edges(64, 64, 256, 128, "clear edges, 256 x 128");
    {
        static const float col[5][4] = { { 0.2f, 0.4f, 0.6f, 1.0f }, { 0.3333333f, 0.1234f, 0.999f, 0.5f }, { 0.0f, 1.0f, 0.5f, 0.0f }, { 0.5019f, 0.25f, 0.75f, 0.3f }, { 0.0627f, 0.9373f, 0.0039f, 0.9961f } };
        static const int masks[3] = { 15, 5, 10 };
        char nm[64];
        int c, d, m;
        for (c = 0; c < 5; c++) for (d = 0; d < 2; d++) for (m = 0; m < (c < 2 ? 3 : 1); m++) {
            snprintf(nm, sizeof nm, "colour clear %d, dither %d, mask %d", c, d, masks[m]);
            color_clear(nm, col[c][0], col[c][1], col[c][2], col[c][3], d, masks[m]);
        }
        glEnable(GL_DITHER);
    }
    {
        static const int box[7][4] = { { 100, 100, 100, 100 }, { 101, 103, 37, 5 }, { 233, 181, 1, 1 }, { 64, 64, 128, 96 }, { 0, 0, 50, 40 }, { W - 60, H - 30, 60, 30 }, { 600, 440, 200, 200 } };
        static const char *kn[3] = { "triangles", "lines", "points" };
        char nm[96];
        int k, b;
        for (k = 0; k < 3; k++) for (b = 0; b < 7; b++) {
            snprintf(nm, sizeof nm, "%s through the box %d %d %d x %d", kn[k], box[b][0], box[b][1], box[b][2], box[b][3]);
            rect_same(k, 0, box[b][0], box[b][1], box[b][2], box[b][3], nm);
        }
        for (b = 0; b < 4; b++) {                                /* a viewport: boxes inside the window */
            snprintf(nm, sizeof nm, "triangles beyond the viewport %d %d %d x %d", box[b][0], box[b][1], box[b][2], box[b][3]);
            rect_same(0, 1, box[b][0], box[b][1], box[b][2], box[b][3], nm);
        }
    }
    if (do_perf) {
        perf("no box", 0, 0, 0, 0, 0);
        perf("scissor 320 x 240", 160, 120, 320, 240, 0);
        perf("scissor 64 x 64", 288, 208, 64, 64, 0);
        perf("scissor 8 x 8", 316, 236, 8, 8, 0);
        perf("viewport 64 x 64, quads 4 x beyond", 288, 208, 64, 64, 1);
    }
    wglMakeCurrent(NULL, NULL);
    printf(g_bad ? "%d failed\n" : "OK\n", g_bad);
    return g_bad ? 1 : 0;
}
