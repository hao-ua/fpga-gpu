/* gl_many_tex_test — more textures than board B's descriptor table holds (1 024 entries): the library gives them table entries as they
 * are drawn and takes back the ones drawn longest ago. Every texture is a distinct colour, drawn as a small block; every block is read back.
 *   gl_many_tex_test [count]          default 6000
 * Linked against the system's import library: beside our opengl32.dll it runs on the boards, anywhere else on the PC's adapter. */
#include <windows.h>
#include <GL/gl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 512
#define H 512
#define BW 4                        /* a block: 4 x 4 pixels, 128 x 128 of them */
#define PER_ROW (W / BW)

static int g_checks, g_bad;

static uint32_t colour_of(int i) { return (uint32_t)((i * 37 + 11) & 255) << 24 | (uint32_t)((i * 101 + 7) & 255) << 16 | (uint32_t)((i / 7 * 53 + 3) & 255) << 8 | 0xFF; }

static void block(int slot)
{
    int x = (slot % PER_ROW) * BW, y = (slot / PER_ROW) * BW;
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0); glVertex2i(x, y);
    glTexCoord2f(1, 0); glVertex2i(x + BW, y);
    glTexCoord2f(1, 1); glVertex2i(x + BW, y + BW);
    glTexCoord2f(0, 1); glVertex2i(x, y + BW);
    glEnd();
}

/* draw texture tex[i] into block slot[i] for every i, then read the whole frame back and check every block's centre */
static void round_of(const GLuint *tex, const int *colour_id, const int *slot, int n, const char *what)
{
    static uint8_t px[W * H * 4];
    int i, bad = 0;
    glClear(GL_COLOR_BUFFER_BIT);
    for (i = 0; i < n; i++) { glBindTexture(GL_TEXTURE_2D, tex[i]); block(slot[i]); }
    glFinish();
    glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
    for (i = 0; i < n; i++) {
        int x = (slot[i] % PER_ROW) * BW + BW / 2, y = (slot[i] / PER_ROW) * BW + BW / 2, c, d = 0;
        const uint8_t *p = px + ((size_t)y * W + x) * 4;
        uint32_t want = colour_of(colour_id[i]);
        for (c = 0; c < 4; c++) { int e = (int)(want >> (24 - 8 * c) & 255); if (abs(p[c] - e) > d) d = abs(p[c] - e); }
        g_checks++;
        if (d > 2) { g_bad++; if (bad++ < 8) printf("FAIL %s: texture %d in block %d reads %02x%02x%02x%02x, expected %08x\n", what, colour_id[i], slot[i], p[0], p[1], p[2], p[3], want); }
    }
    printf("%s: %d textures drawn, %d wrong\n", what, n, bad);
}

static LRESULT CALLBACK wndproc(HWND h, UINT m, WPARAM w, LPARAM l) { return DefWindowProcA(h, m, w, l); }

int main(int argc, char **argv)
{
    WNDCLASSA wc;
    HWND hwnd;
    HDC dc;
    HGLRC rc;
    PIXELFORMATDESCRIPTOR pfd;
    RECT r = { 0, 0, W, H };
    int n = argc > 1 ? atoi(argv[1]) : 6000, i, pf, half;
    GLuint *tex;
    int *cid, *slot;
    uint8_t texel[4 * 4 * 4];
    setvbuf(stdout, NULL, _IONBF, 0);
    if (n < 1) n = 1;
    if (n > (W / BW) * (H / BW)) n = (W / BW) * (H / BW);
    memset(&wc, 0, sizeof wc);
    wc.lpfnWndProc = wndproc; wc.lpszClassName = "gl_many"; wc.hInstance = GetModuleHandleA(NULL); wc.style = CS_OWNDC;
    RegisterClassA(&wc);
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    hwnd = CreateWindowA("gl_many", "gl_many", WS_OVERLAPPEDWINDOW | WS_VISIBLE, 40, 40, r.right - r.left, r.bottom - r.top, NULL, NULL, wc.hInstance, NULL);
    dc = GetDC(hwnd);
    memset(&pfd, 0, sizeof pfd);
    pfd.nSize = sizeof pfd; pfd.nVersion = 1; pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA; pfd.cColorBits = 32; pfd.cDepthBits = 24; pfd.cStencilBits = 8;
    pf = ChoosePixelFormat(dc, &pfd);
    SetPixelFormat(dc, pf, &pfd);
    rc = wglCreateContext(dc);
    if (!rc || !wglMakeCurrent(dc, rc)) { printf("no OpenGL context\n"); return 1; }
    printf("renderer: %s, %d textures\n", (const char *)glGetString(GL_RENDERER), n);
    glViewport(0, 0, W, H);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, W, 0, H, -1, 1);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    glEnable(GL_TEXTURE_2D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glClearColor(0, 0, 0, 1);
    tex = (GLuint *)malloc((size_t)n * sizeof *tex); cid = (int *)malloc((size_t)n * sizeof *cid); slot = (int *)malloc((size_t)n * sizeof *slot);
    glGenTextures(n, tex);
    for (i = 0; i < n; i++) {
        uint32_t c = colour_of(i);
        int t;
        for (t = 0; t < 16; t++) { texel[4 * t] = (uint8_t)(c >> 24); texel[4 * t + 1] = (uint8_t)(c >> 16); texel[4 * t + 2] = (uint8_t)(c >> 8); texel[4 * t + 3] = (uint8_t)c; }
        glBindTexture(GL_TEXTURE_2D, tex[i]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, texel);
        if (glGetError() != GL_NO_ERROR) { printf("FAIL texture %d: glTexImage2D error\n", i); g_bad++; }
        cid[i] = i; slot[i] = i;
    }
    round_of(tex, cid, slot, n, "in the order made");
    for (i = 0; i < n; i++) slot[i] = n - 1 - i;                      /* the other way round: the ones made first were taken back first */
    round_of(tex, cid, slot, n, "the other way round");
    for (i = 0; i < n; i++) { int j = (int)((i * 2654435761u) % (unsigned)n); GLuint t = tex[i]; int c = cid[i]; tex[i] = tex[j]; cid[i] = cid[j]; tex[j] = t; cid[j] = c; slot[i] = i; }
    round_of(tex, cid, slot, n, "shuffled");
    /* half deleted, as many made anew: the new ones take the freed handles */
    half = n / 2;
    glDeleteTextures(half, tex);
    glGenTextures(half, tex);
    for (i = 0; i < half; i++) {
        uint32_t c = colour_of(n + i);
        int t;
        for (t = 0; t < 16; t++) { texel[4 * t] = (uint8_t)(c >> 24); texel[4 * t + 1] = (uint8_t)(c >> 16); texel[4 * t + 2] = (uint8_t)(c >> 8); texel[4 * t + 3] = (uint8_t)c; }
        glBindTexture(GL_TEXTURE_2D, tex[i]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, texel);
        cid[i] = n + i;
    }
    for (i = 0; i < n; i++) slot[i] = i;
    round_of(tex, cid, slot, n, "half made anew");
    glDeleteTextures(n, tex);
    printf("%d checks, %d failed\n", g_checks, g_bad);
    if (!g_bad) printf("OK gl_many_tex_test\n");
    wglMakeCurrent(NULL, NULL);
    return g_bad ? 1 : 0;
}
