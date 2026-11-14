/* gl_far_test — d3d_far_test's measurement through OpenGL's fixed function: the depth resolution of far geometry. Two parallel walls, the red one `gap`
 * behind the green one and inside its outline, so a red pixel is a depth error whichever is drawn first; per distance and gap the red pixels over a few
 * camera positions, as a share of the red wall's own area.
 *   gl_far_test [near far [offset [frames]]]      defaults 0.5 5000 3000 4
 * Linked against the system's import library: beside our opengl32.dll it measures the boards, anywhere else the PC's own adapter (the reference). */
#include <windows.h>
#include <GL/gl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 640
#define H 480

typedef struct { float x, y, z; uint8_t r, g, b, a; } vtx;
typedef struct { double x, y, z; } v3;
static v3 add(v3 a, v3 b) { v3 r = { a.x + b.x, a.y + b.y, a.z + b.z }; return r; }
static v3 mul(v3 a, double s) { v3 r = { a.x * s, a.y * s, a.z * s }; return r; }
static v3 cross(v3 a, v3 b) { v3 r = { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; return r; }
static double dot(v3 a, v3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static v3 norm(v3 a) { return mul(a, 1.0 / sqrt(dot(a, a))); }

static int wall(vtx *o, v3 c, v3 u, v3 v, double hu, double hv, int nx, int ny, int red)
{
    int i, j, k, n = 0;
    static const int q[6][2] = { { 0, 0 }, { 1, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 }, { 0, 1 } };
    for (j = 0; j < ny; j++) for (i = 0; i < nx; i++) for (k = 0; k < 6; k++) {
        double a = -1.0 + 2.0 * (i + q[k][0]) / nx, b = -1.0 + 2.0 * (j + q[k][1]) / ny;
        v3 p = add(c, add(mul(u, a * hu), mul(v, b * hv)));
        o[n].x = (float)p.x; o[n].y = (float)p.y; o[n].z = (float)p.z; o[n].r = red ? 255 : 0; o[n].g = red ? 0 : 255; o[n].b = 0; o[n].a = 255; n++;
    }
    return n;
}

static void look(v3 eye, v3 dir)
{
    GLfloat m[16];
    v3 up = { 0, 1, 0 }, f = norm(dir), s = norm(cross(f, up)), u = cross(s, f);
    memset(m, 0, sizeof m);
    m[0] = (GLfloat)s.x; m[4] = (GLfloat)s.y; m[8] = (GLfloat)s.z; m[12] = (GLfloat)-dot(s, eye);
    m[1] = (GLfloat)u.x; m[5] = (GLfloat)u.y; m[9] = (GLfloat)u.z; m[13] = (GLfloat)-dot(u, eye);
    m[2] = (GLfloat)-f.x; m[6] = (GLfloat)-f.y; m[10] = (GLfloat)-f.z; m[14] = (GLfloat)dot(f, eye);
    m[15] = 1.0f;
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(m);
}

static void draw(const vtx *v, int n)
{
    glVertexPointer(3, GL_FLOAT, sizeof *v, &v->x);
    glColorPointer(4, GL_UNSIGNED_BYTE, sizeof *v, &v->r);
    glDrawArrays(GL_TRIANGLES, 0, n);
}

static int count_red(void)
{
    static uint8_t px[W * H * 4];
    int i, n = 0;
    glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
    for (i = 0; i < W * H; i++) if (px[i * 4] > 128 && px[i * 4 + 1] < 128) n++;
    return n;
}

int main(int argc, char **argv)
{
    static const double dist[] = { 25, 50, 100, 200, 400, 800, 1600 }, gaps[] = { 0.01, 0.03, 0.1, 0.3, 1, 3, 10, 30, 100 };
    static vtx vn[48 * 36 * 6], vf[16 * 12 * 6];
    double zn = argc > 2 ? atof(argv[1]) : 0.5, zf = argc > 2 ? atof(argv[2]) : 5000.0, off = argc > 3 ? atof(argv[3]) : 3000.0, t;
    int frames = argc > 4 ? atoi(argv[4]) : 4, di, gi, fr, order, pf;
    WNDCLASSA wc;
    RECT r = { 0, 0, W, H };
    HWND wnd;
    HDC dc;
    HGLRC rc;
    PIXELFORMATDESCRIPTOR pfd;
    GLfloat p[16];
    setvbuf(stdout, NULL, _IONBF, 0);
    memset(&wc, 0, sizeof wc);
    wc.lpfnWndProc = DefWindowProcA; wc.hInstance = GetModuleHandleA(NULL); wc.lpszClassName = "gl_far_test"; wc.style = CS_OWNDC;
    RegisterClassA(&wc);
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    wnd = CreateWindowA("gl_far_test", "gl_far_test", WS_OVERLAPPEDWINDOW | WS_VISIBLE, 40, 40, r.right - r.left, r.bottom - r.top, NULL, NULL, wc.hInstance, NULL);
    dc = GetDC(wnd);
    memset(&pfd, 0, sizeof pfd);
    pfd.nSize = sizeof pfd; pfd.nVersion = 1; pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA; pfd.cColorBits = 32; pfd.cDepthBits = 24; pfd.cStencilBits = 8;
    pf = ChoosePixelFormat(dc, &pfd);
    SetPixelFormat(dc, pf, &pfd);
    rc = wglCreateContext(dc);
    if (!rc || !wglMakeCurrent(dc, rc)) { printf("no OpenGL context\n"); return 1; }
    printf("renderer: %s\nnear %g far %g, the scene %g from the origin, %d camera positions per cell, depth 24 bits\n", (const char *)glGetString(GL_RENDERER), zn, zf, off, frames);
    printf("red pixels showing, %% of the red wall's area (the worse of the two draw orders); a 24-bit buffer resolves d*d/(near*2^24)\n");
    glViewport(0, 0, W, H);
    t = 1.0 / tan(0.5 * 50.0 * 3.14159265358979 / 180.0);
    memset(p, 0, sizeof p);
    p[0] = (GLfloat)(t * H / W); p[5] = (GLfloat)t; p[10] = (GLfloat)((zf + zn) / (zn - zf)); p[11] = -1.0f; p[14] = (GLfloat)(2.0 * zf * zn / (zn - zf));
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(p);
    glDisable(GL_LIGHTING); glDisable(GL_CULL_FACE); glEnable(GL_DEPTH_TEST); glDepthMask(GL_TRUE); glDepthFunc(GL_LEQUAL);
    glEnableClientState(GL_VERTEX_ARRAY); glEnableClientState(GL_COLOR_ARRAY);
    glClearColor(0.0f, 0.0f, 0.25f, 1.0f); glClearDepth(1.0);
    printf("%8s %9s |", "distance", "24 bits");
    for (gi = 0; gi < (int)(sizeof gaps / sizeof gaps[0]); gi++) printf(" %6g", gaps[gi]);
    printf("   (the gap)\n");
    for (di = 0; di < (int)(sizeof dist / sizeof dist[0]); di++) {
        double d = dist[di];
        printf("%8g %9.4f |", d, d * d / (zn * 16777216.0));
        for (gi = 0; gi < (int)(sizeof gaps / sizeof gaps[0]); gi++) {
            double gap = gaps[gi], worst = 0.0;
            if (gap >= d || d + gap > 0.8 * zf) { printf(" %6s", "-"); continue; }
            for (order = 0; order < 2; order++) {
                long red = 0, area = 0;
                for (fr = 0; fr < frames; fr++) {
                    double yaw = 0.35 + 0.0052 * fr, tilt = 0.33;
                    v3 eye = { off + 0.37 * fr, 20.0 + 0.11 * fr, 0.66 * off + 0.53 * fr }, dir = { sin(yaw), 0.0, cos(yaw) }, up = { 0, 1, 0 };
                    v3 u = { cos(yaw + tilt), 0.0, -sin(yaw + tilt) };
                    int nn, nf;
                    nn = wall(vn, add(eye, mul(dir, d)), u, up, 0.30 * d, 0.24 * d, 48, 36, 0);
                    nf = wall(vf, add(eye, mul(dir, d + gap)), u, up, 0.25 * d, 0.20 * d, 16, 12, 1);
                    look(eye, dir);
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                    draw(vf, nf);
                    area += count_red();
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                    if (order) draw(vn, nn);
                    draw(vf, nf);
                    if (!order) draw(vn, nn);
                    red += count_red();
                    SwapBuffers(dc);
                }
                if (area && 100.0 * red / area > worst) worst = 100.0 * red / area;
            }
            printf(" %6.1f", worst);
        }
        printf("\n");
    }
    wglMakeCurrent(NULL, NULL);
    return 0;
}
