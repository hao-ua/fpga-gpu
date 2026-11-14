/* d3d_far_test — the depth resolution of far geometry, measured through the real API: two parallel walls, the red one `gap` behind the green one and
 * inside its outline, so a red pixel is a depth error whichever is drawn first. Per distance and gap: the red pixels over a few camera positions,
 * as a share of the red wall's own area. The walls are tilted and tessellated differently, the scene sits far from the world's origin as a game's does.
 *   d3d_far_test [near far [offset [frames [rev | vs | vs2]]]]      defaults 0.5 5000 3000 4
 *   vs = the same scene through a vertex shader as games write it (vs_1_1: m4x4 r0, v0, c0; mov oPos, r0; the matrix the application's own float product);
 *   vs2 = a position the library cannot follow (the matrix rows scaled by a constant in the shader): its depth is the difference of the program's own z and w;
 *   rev = the application reverses the depth itself (far at 0):
 *       on a library that keeps Direct3D's count (GPU2_REVZ=0, or board A before 147) it shows that the boards resolve far depth once it is counted
 *       from the far end. Our d3d9.dll does that inside, so there `rev` undoes it and measures the resolution of Direct3D's own count
 * It is a plain Direct3D 9 program: beside our d3d9.dll it measures the boards, anywhere else the PC's own adapter (the reference).
 * A 24-bit depth buffer resolves about d * d / (near * 2^24): 1 mm at 100, 8 cm at 800 (near 0.5). */
#define CINTERFACE
#define COBJMACROS
#include <windows.h>
#include <d3d9.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 640
#define H 480

static IDirect3DDevice9 *g_d;
static IDirect3DSurface9 *g_sys;
static D3DMATRIX g_pr;
static int g_vs;

typedef struct { float x, y, z; DWORD c; } vtx;
#define FVF (D3DFVF_XYZ | D3DFVF_DIFFUSE)

typedef struct { double x, y, z; } v3;
static v3 add(v3 a, v3 b) { v3 r = { a.x + b.x, a.y + b.y, a.z + b.z }; return r; }
static v3 mul(v3 a, double s) { v3 r = { a.x * s, a.y * s, a.z * s }; return r; }
static v3 cross(v3 a, v3 b) { v3 r = { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; return r; }
static double dot(v3 a, v3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static v3 norm(v3 a) { return mul(a, 1.0 / sqrt(dot(a, a))); }

/* a wall of nx x ny cells: centre c, half extents hu along u and hv along v, as a triangle list */
static int wall(vtx *o, v3 c, v3 u, v3 v, double hu, double hv, int nx, int ny, DWORD col)
{
    int i, j, k, n = 0;
    static const int q[6][2] = { { 0, 0 }, { 1, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 }, { 0, 1 } };
    for (j = 0; j < ny; j++) for (i = 0; i < nx; i++) for (k = 0; k < 6; k++) {
        double a = -1.0 + 2.0 * (i + q[k][0]) / nx, b = -1.0 + 2.0 * (j + q[k][1]) / ny;
        v3 p = add(c, add(mul(u, a * hu), mul(v, b * hv)));
        o[n].x = (float)p.x; o[n].y = (float)p.y; o[n].z = (float)p.z; o[n].c = col; n++;
    }
    return n;
}

static void look(v3 eye, v3 dir)
{
    D3DMATRIX m;
    v3 up = { 0, 1, 0 }, z = norm(dir), x = norm(cross(up, z)), y = cross(z, x);
    memset(&m, 0, sizeof m);
    m.m[0][0] = (float)x.x; m.m[0][1] = (float)y.x; m.m[0][2] = (float)z.x;
    m.m[1][0] = (float)x.y; m.m[1][1] = (float)y.y; m.m[1][2] = (float)z.y;
    m.m[2][0] = (float)x.z; m.m[2][1] = (float)y.z; m.m[2][2] = (float)z.z;
    m.m[3][0] = (float)-dot(x, eye); m.m[3][1] = (float)-dot(y, eye); m.m[3][2] = (float)-dot(z, eye); m.m[3][3] = 1.0f;
    IDirect3DDevice9_SetTransform(g_d, D3DTS_VIEW, &m);
    if (g_vs) {         /* c0 .. c3: the rows the shader's m4x4 takes, view x projection as a game multiplies them, in float */
        float c[4][4];
        int i, j, k;
        for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) { float t = 0; for (k = 0; k < 4; k++) t += m.m[j][k] * g_pr.m[k][i]; c[i][j] = t; }
        IDirect3DDevice9_SetVertexShaderConstantF(g_d, 0, c[0], 4);
    }
}

static int count_red(void)
{
    IDirect3DSurface9 *rt = NULL;
    D3DLOCKED_RECT l;
    int x, y, n = 0;
    IDirect3DDevice9_GetRenderTarget(g_d, 0, &rt);
    if (FAILED(IDirect3DDevice9_GetRenderTargetData(g_d, rt, g_sys))) { printf("GetRenderTargetData failed\n"); exit(1); }
    IDirect3DSurface9_LockRect(g_sys, &l, NULL, D3DLOCK_READONLY);
    for (y = 0; y < H; y++) {
        const uint8_t *p = (const uint8_t *)l.pBits + (ptrdiff_t)y * l.Pitch;
        for (x = 0; x < W; x++) if (p[x * 4 + 2] > 128 && p[x * 4 + 1] < 128) n++;
    }
    IDirect3DSurface9_UnlockRect(g_sys);
    IDirect3DSurface9_Release(rt);
    return n;
}

int main(int argc, char **argv)
{
    static const double dist[] = { 25, 50, 100, 200, 400, 800, 1600 }, gaps[] = { 0.01, 0.03, 0.1, 0.3, 1, 3, 10, 30, 100 };
    static vtx vn[48 * 36 * 6], vf[16 * 12 * 6];
    double zn = argc > 2 ? atof(argv[1]) : 0.5, zf = argc > 2 ? atof(argv[2]) : 5000.0, off = argc > 3 ? atof(argv[3]) : 3000.0;
    int frames = argc > 4 ? atoi(argv[4]) : 4, rev = argc > 5 && !strcmp(argv[5], "rev"), di, gi, fr, order;
    /* vs_1_1; dcl_position v0; dcl_color v1; m4x4 r0, v0, c0; mov oPos, r0; mov oD0, v1 */
    static const DWORD vs1[] = { 0xFFFE0101, 0x0000001F, 0x80000000, 0x900F0000, 0x0000001F, 0x8000000A, 0x900F0001,
                                 0x00000014, 0x800F0000, 0x90E40000, 0xA0E40000, 0x00000001, 0xC00F0000, 0x80E40000, 0x00000001, 0xD00F0000, 0x90E40001, 0x0000FFFF };
    /* the same with the rows doubled and halved again in the shader: m4x4 r0, v0, c0; mul r0, r0, c4.x; mov oPos, r0 (c4.x = 1: the value is the same, the form is not a dot product) */
    static const DWORD vs2[] = { 0xFFFE0101, 0x0000001F, 0x80000000, 0x900F0000, 0x0000001F, 0x8000000A, 0x900F0001,
                                 0x00000014, 0x800F0000, 0x90E40000, 0xA0E40000, 0x00000005, 0x800F0000, 0x80E40000, 0xA0000004,
                                 0x00000001, 0xC00F0000, 0x80E40000, 0x00000001, 0xD00F0000, 0x90E40001, 0x0000FFFF };
    static const float one[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
    IDirect3DVertexShader9 *sh = NULL;
    float zclear = rev ? 0.0f : 1.0f;
    WNDCLASSA wc;
    RECT r = { 0, 0, W, H };
    HWND wnd;
    IDirect3D9 *d3d;
    D3DPRESENT_PARAMETERS pp;
    D3DMATRIX id, pr;
    D3DADAPTER_IDENTIFIER9 ai;
    setvbuf(stdout, NULL, _IONBF, 0);
    memset(&wc, 0, sizeof wc);
    wc.lpfnWndProc = DefWindowProcA; wc.hInstance = GetModuleHandleA(NULL); wc.lpszClassName = "d3d_far_test";
    RegisterClassA(&wc);
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    wnd = CreateWindowA("d3d_far_test", "d3d_far_test", WS_OVERLAPPEDWINDOW, 40, 40, r.right - r.left, r.bottom - r.top, NULL, NULL, wc.hInstance, NULL);
    ShowWindow(wnd, SW_SHOWNOACTIVATE);
    d3d = Direct3DCreate9(D3D_SDK_VERSION);
    if (!d3d) { printf("no Direct3D 9\n"); return 1; }
    memset(&ai, 0, sizeof ai);
    IDirect3D9_GetAdapterIdentifier(d3d, 0, 0, &ai);
    memset(&pp, 0, sizeof pp);
    pp.Windowed = TRUE; pp.SwapEffect = D3DSWAPEFFECT_DISCARD; pp.BackBufferFormat = D3DFMT_X8R8G8B8; pp.EnableAutoDepthStencil = TRUE; pp.AutoDepthStencilFormat = D3DFMT_D24S8;
    pp.BackBufferWidth = W; pp.BackBufferHeight = H; pp.hDeviceWindow = wnd;
    if (FAILED(IDirect3D9_CreateDevice(d3d, 0, D3DDEVTYPE_HAL, wnd, D3DCREATE_HARDWARE_VERTEXPROCESSING, &pp, &g_d)) || !g_d) { printf("no device\n"); return 1; }
    IDirect3DDevice9_CreateOffscreenPlainSurface(g_d, W, H, D3DFMT_X8R8G8B8, D3DPOOL_SYSTEMMEM, &g_sys, NULL);
    printf("adapter: %s\nnear %g far %g, the scene %g from the origin, %d camera positions per cell, depth D24S8\n", ai.Description, zn, zf, off, frames);
    printf("red pixels showing, %% of the red wall's area (the worse of the two draw orders); a 24-bit buffer resolves d*d/(near*2^24)\n");
    memset(&id, 0, sizeof id); id.m[0][0] = id.m[1][1] = id.m[2][2] = id.m[3][3] = 1.0f;
    memset(&pr, 0, sizeof pr);
    pr.m[1][1] = (float)(1.0 / tan(0.5 * 50.0 * 3.14159265358979 / 180.0)); pr.m[0][0] = pr.m[1][1] * (float)H / (float)W;
    pr.m[2][2] = (float)(zf / (zf - zn)); pr.m[2][3] = 1.0f; pr.m[3][2] = (float)(-zn * zf / (zf - zn));
    if (rev) {      /* the application's own reversed depth: z' = w - z, far at 0, the test the other way round */
        pr.m[2][2] = (float)(-zn / (zf - zn)); pr.m[3][2] = (float)(zn * zf / (zf - zn));
        printf("reversed depth: the projection gives w - z, GREATEREQUAL, cleared to 0\n");
    }
    IDirect3DDevice9_SetTransform(g_d, D3DTS_WORLD, &id);
    IDirect3DDevice9_SetTransform(g_d, D3DTS_PROJECTION, &pr);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_LIGHTING, FALSE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_CULLMODE, D3DCULL_NONE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ZENABLE, TRUE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ZWRITEENABLE, TRUE);
    IDirect3DDevice9_SetRenderState(g_d, D3DRS_ZFUNC, rev ? D3DCMP_GREATEREQUAL : D3DCMP_LESSEQUAL);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    IDirect3DDevice9_SetTextureStageState(g_d, 0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    IDirect3DDevice9_SetFVF(g_d, FVF);
    g_pr = pr;
    if (argc > 5 && !strncmp(argv[5], "vs", 2)) {
        int two = !strcmp(argv[5], "vs2");
        if (FAILED(IDirect3DDevice9_CreateVertexShader(g_d, two ? vs2 : vs1, &sh)) || !sh) { printf("no vertex shader\n"); return 1; }
        IDirect3DDevice9_SetVertexShader(g_d, sh);
        IDirect3DDevice9_SetVertexShaderConstantF(g_d, 4, one, 1);
        g_vs = 1;
        printf("vertex shader: %s\n", two ? "m4x4 r0, v0, c0; mul r0, r0, c4.x; mov oPos, r0" : "m4x4 r0, v0, c0; mov oPos, r0");
    }
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
                    v3 u = { cos(yaw + tilt), 0.0, -sin(yaw + tilt) };        /* the walls' horizontal axis: turned from facing the camera */
                    int nn, nf;
                    nn = wall(vn, add(eye, mul(dir, d)), u, up, 0.30 * d, 0.24 * d, 48, 36, 0xFF00FF00);
                    nf = wall(vf, add(eye, mul(dir, d + gap)), u, up, 0.25 * d, 0.20 * d, 16, 12, 0xFFFF0000);
                    look(eye, dir);
                    IDirect3DDevice9_BeginScene(g_d);
                    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0xFF000040, zclear, 0);
                    IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, (UINT)nf / 3, vf, sizeof vf[0]);
                    IDirect3DDevice9_EndScene(g_d);
                    area += count_red();
                    IDirect3DDevice9_BeginScene(g_d);
                    IDirect3DDevice9_Clear(g_d, 0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0xFF000040, zclear, 0);
                    if (order) IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, (UINT)nn / 3, vn, sizeof vn[0]);
                    IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, (UINT)nf / 3, vf, sizeof vf[0]);
                    if (!order) IDirect3DDevice9_DrawPrimitiveUP(g_d, D3DPT_TRIANGLELIST, (UINT)nn / 3, vn, sizeof vn[0]);
                    IDirect3DDevice9_EndScene(g_d);
                    red += count_red();
                    IDirect3DDevice9_Present(g_d, NULL, NULL, NULL, NULL);
                }
                if (area && 100.0 * red / area > worst) worst = 100.0 * red / area;
            }
            printf(" %6.1f", worst);
        }
        printf("\n");
    }
    return 0;
}
