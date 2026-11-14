/* gl_tex_cow_test — textures changed in the middle of a frame, after draws that sampled them (what a game does with lightmaps under a
 * dynamic light): every draw must show the texels its texture held when it was drawn. The library keeps them by copying the texture on board B
 * (copy-on-write) instead of ending the render pass; the result is the same either way, only the cost differs, which `perf` measures.
 *   gl_tex_cow_test [perf]
 * (Not named "...update...": Windows treats such a program as an installer and asks for elevation.)
 * Linked against the system's import library: beside our opengl32.dll it runs on the boards, anywhere else on the PC's adapter. */
#include <windows.h>
#include <GL/gl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 256
#define H 128

static HDC g_dc;
static int g_checks, g_bad;

static void ok(int c, const char *what) { g_checks++; if (!c) { g_bad++; printf("FAIL %s\n", what); } }

static void fill(uint8_t *p, int n, uint32_t rgba)
{
    int i;
    for (i = 0; i < n; i++) { p[4 * i] = (uint8_t)(rgba >> 24); p[4 * i + 1] = (uint8_t)(rgba >> 16); p[4 * i + 2] = (uint8_t)(rgba >> 8); p[4 * i + 3] = (uint8_t)rgba; }
}

/* a quad in window pixels [x0, x1) x [y0, y1) with the whole texture on it */
static void quad(int x0, int y0, int x1, int y1)
{
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0); glVertex2i(x0, y0);
    glTexCoord2f(1, 0); glVertex2i(x1, y0);
    glTexCoord2f(1, 1); glVertex2i(x1, y1);
    glTexCoord2f(0, 1); glVertex2i(x0, y1);
    glEnd();
}

static uint32_t px(int x, int y)
{
    uint8_t p[4];
    glReadPixels(x, y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, p);
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

static void expect(int x, int y, uint32_t want, const char *what)
{
    uint32_t got = px(x, y);
    char b[160];
    int d = 0, i;
    for (i = 0; i < 4; i++) { int a = (int)(got >> (8 * i) & 255), e = (int)(want >> (8 * i) & 255); if (abs(a - e) > d) d = abs(a - e); }
    snprintf(b, sizeof b, "%s: pixel (%d, %d) = %08x, expected %08x", what, x, y, got, want);
    ok(d <= 2, b);
}

static GLuint tex_of(int w, int h, uint32_t rgba, int mip)
{
    GLuint t;
    uint8_t *p = (uint8_t *)malloc((size_t)w * h * 4);
    int l = 0;
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, mip ? GL_NEAREST_MIPMAP_NEAREST : GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    fill(p, w * h, rgba);
    for (;;) {
        glTexImage2D(GL_TEXTURE_2D, l, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, p);
        if (!mip || (w == 1 && h == 1)) break;
        w = w > 1 ? w / 2 : 1; h = h > 1 ? h / 2 : 1; l++;
    }
    free(p);
    return t;
}

static void sub(GLuint t, int level, int x, int y, int w, int h, uint32_t rgba)
{
    uint8_t *p = (uint8_t *)malloc((size_t)w * h * 4);
    fill(p, w * h, rgba);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexSubImage2D(GL_TEXTURE_2D, level, x, y, w, h, GL_RGBA, GL_UNSIGNED_BYTE, p);
    free(p);
}

static void tests(void)
{
    GLuint a, b, m;
    int i;
    const uint32_t RED = 0xFF0000FF, GREEN = 0x00FF00FF, BLUE = 0x0000FFFF, WHITE = 0xFFFFFFFF;
    glClearColor(0, 0, 0, 1);
    /* 1: the whole texture replaced between two draws */
    a = tex_of(16, 16, RED, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    glBindTexture(GL_TEXTURE_2D, a); quad(0, 0, 64, 64);
    sub(a, 0, 0, 0, 16, 16, GREEN);
    glBindTexture(GL_TEXTURE_2D, a); quad(64, 0, 128, 64);
    glFinish();
    expect(32, 32, RED, "whole update: the draw before it");
    expect(96, 32, GREEN, "whole update: the draw after it");
    /* 2: a part of it: the rest stays */
    b = tex_of(16, 16, RED, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    glBindTexture(GL_TEXTURE_2D, b); quad(0, 0, 64, 64);
    sub(b, 0, 0, 0, 8, 8, BLUE);                               /* texels (0..7, 0..7): the quad's bottom-left quarter */
    glBindTexture(GL_TEXTURE_2D, b); quad(64, 0, 128, 64);
    glFinish();
    expect(16, 16, RED, "part: the draw before it, the changed corner");
    expect(80, 16, BLUE, "part: the draw after it, the changed corner");
    expect(112, 48, RED, "part: the draw after it, the rest");
    /* 3: one texture changed ten times in one frame, a draw after each change */
    glClear(GL_COLOR_BUFFER_BIT);
    for (i = 0; i < 10; i++) {
        sub(a, 0, 0, 0, 16, 16, 0x10000000u * (uint32_t)(i + 1) | 0x00806000u | 0xFF);
        glBindTexture(GL_TEXTURE_2D, a); quad(i * 24, 64, i * 24 + 24, 128);
    }
    glFinish();
    for (i = 0; i < 10; i++) {
        char w[64];
        snprintf(w, sizeof w, "ten updates: draw %d", i);
        expect(i * 24 + 12, 96, 0x10000000u * (uint32_t)(i + 1) | 0x00806000u | 0xFF, w);
    }
    /* 4: a mipmapped texture: levels 0 and 1 changed apart, each draw at the level it samples */
    m = tex_of(32, 32, WHITE, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glBindTexture(GL_TEXTURE_2D, m); quad(0, 0, 32, 32);         /* 32 texels on 32 pixels: level 0 */
    quad(32, 0, 48, 16);                                          /* 32 on 16: level 1 */
    sub(m, 0, 0, 0, 32, 32, RED);
    sub(m, 1, 0, 0, 16, 16, GREEN);
    glBindTexture(GL_TEXTURE_2D, m); quad(64, 0, 96, 32); quad(96, 0, 112, 16);
    glFinish();
    expect(16, 16, WHITE, "mipmaps: level 0 before");
    expect(40, 8, WHITE, "mipmaps: level 1 before");
    expect(80, 16, RED, "mipmaps: level 0 after");
    expect(104, 8, GREEN, "mipmaps: level 1 after");
    /* 5: a texture deleted after a draw that sampled it, in the same frame */
    glClear(GL_COLOR_BUFFER_BIT);
    glBindTexture(GL_TEXTURE_2D, b); quad(0, 0, 64, 64);
    glDeleteTextures(1, &b);
    glFinish();
    expect(80 - 64, 48, RED, "deleted after its draw");
    glDeleteTextures(1, &a);
    glDeleteTextures(1, &m);
}

/* frames of 16 textures, each drawn, then changed, then drawn again before the next one: what dynamic lights do to a game's lightmaps.
   Without copy-on-write every change ends the pass: 17 passes a frame */
static void perf(void)
{
    GLuint t[16];
    int i, f, n = 240;
    LARGE_INTEGER q0, q1, fr;
    for (i = 0; i < 16; i++) t[i] = tex_of(128, 128, 0x808080FFu, 0);
    QueryPerformanceFrequency(&fr);
    QueryPerformanceCounter(&q0);
    for (f = 0; f < n; f++) {
        glClear(GL_COLOR_BUFFER_BIT);
        for (i = 0; i < 16; i++) {
            glBindTexture(GL_TEXTURE_2D, t[i]); quad((i % 8) * 32, (i / 8) * 64, (i % 8) * 32 + 32, (i / 8) * 64 + 32);
            sub(t[i], 0, 32, 32, 64, 64, 0x10101000u * (uint32_t)(f % 15) | 0xFF);
            glBindTexture(GL_TEXTURE_2D, t[i]); quad((i % 8) * 32, (i / 8) * 64 + 32, (i % 8) * 32 + 32, (i / 8) * 64 + 64);
        }
        SwapBuffers(g_dc);
    }
    glFinish();
    QueryPerformanceCounter(&q1);
    printf("perf: %d frames of 16 textures updated after use: %.2f ms a frame\n", n, 1000.0 * (double)(q1.QuadPart - q0.QuadPart) / (double)fr.QuadPart / n);
    glDeleteTextures(16, t);
}

static LRESULT CALLBACK wndproc(HWND h, UINT m, WPARAM w, LPARAM l) { return DefWindowProcA(h, m, w, l); }

int main(int argc, char **argv)
{
    WNDCLASSA wc;
    HWND hwnd;
    HGLRC rc;
    PIXELFORMATDESCRIPTOR pfd;
    RECT r = { 0, 0, W, H };
    int pf, do_perf = argc > 1 && !strcmp(argv[1], "perf");
    setvbuf(stdout, NULL, _IONBF, 0);
    memset(&wc, 0, sizeof wc);
    wc.lpfnWndProc = wndproc; wc.lpszClassName = "gl_cow"; wc.hInstance = GetModuleHandleA(NULL); wc.style = CS_OWNDC;
    RegisterClassA(&wc);
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    hwnd = CreateWindowA("gl_cow", "gl_cow", WS_OVERLAPPEDWINDOW | WS_VISIBLE, 40, 40, r.right - r.left, r.bottom - r.top, NULL, NULL, wc.hInstance, NULL);
    g_dc = GetDC(hwnd);
    memset(&pfd, 0, sizeof pfd);
    pfd.nSize = sizeof pfd; pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA; pfd.cColorBits = 32; pfd.cDepthBits = 24; pfd.cStencilBits = 8;
    pf = ChoosePixelFormat(g_dc, &pfd);
    SetPixelFormat(g_dc, pf, &pfd);
    rc = wglCreateContext(g_dc);
    if (!rc || !wglMakeCurrent(g_dc, rc)) { printf("no OpenGL context\n"); return 1; }
    printf("renderer: %s\n", (const char *)glGetString(GL_RENDERER));
    glViewport(0, 0, W, H);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, W, 0, H, -1, 1);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    glEnable(GL_TEXTURE_2D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    tests();
    if (do_perf) perf();
    printf("%d checks, %d failed\n", g_checks, g_bad);
    if (!g_bad) printf("OK gl_tex_cow_test\n");
    wglMakeCurrent(NULL, NULL);
    return g_bad ? 1 : 0;
}
