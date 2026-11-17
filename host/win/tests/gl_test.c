/* gl_test — opengl32.dll through the real API on the boards: a window, ChoosePixelFormat / SetPixelFormat / SwapBuffers through gdi32 exactly as a game reaches
 * the library, then scenes with known pixels, read back with glReadPixels:
 *   clear, flat triangle, depth test, blend, scissor, stencil, alpha test, texture modulate / replace, multitexture, fog, a lit quad, display list, quads through arrays, FBO render-to-texture.
 * Tolerances are +-2 of 255: the shaders compute in FP24. */
#include <windows.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_bad, g_checks;
static HDC g_dc;
#define W 640
#define H 480

static void px(int x, int y, const char *what, int r, int g, int b, int a)
{
    uint8_t c[4];
    glReadPixels(x, y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, c);
    g_checks++;
    if (abs(c[0] - r) > 2 || abs(c[1] - g) > 2 || abs(c[2] - b) > 2 || (a >= 0 && abs(c[3] - a) > 2)) {
        g_bad++;
        printf("FAIL %s: pixel (%d, %d) = %d %d %d %d, expected %d %d %d %d\n", what, x, y, c[0], c[1], c[2], c[3], r, g, b, a);
    }
}

static void quad(float x0, float y0, float x1, float y1, float z)
{
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0); glVertex3f(x0, y0, z);
    glTexCoord2f(1, 0); glVertex3f(x1, y0, z);
    glTexCoord2f(1, 1); glVertex3f(x1, y1, z);
    glTexCoord2f(0, 1); glVertex3f(x0, y1, z);
    glEnd();
}

static void ortho(void)
{
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, W, 0, H, -1, 1);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
}

static void tests(void)
{
    static uint8_t tex[4 * 4 * 4];
    GLuint t[2], fb, rb, list;
    int i;
    printf("GL_RENDERER: %s\nGL_VERSION: %s\n", glGetString(GL_RENDERER), glGetString(GL_VERSION));
    glViewport(0, 0, W, H);
    ortho();

    /* clear */
    glClearColor(0.2f, 0.4f, 0.6f, 1.0f);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    px(320, 240, "clear", 51, 102, 153, 255);

    /* a flat triangle */
    glColor3f(1, 0, 0);
    glBegin(GL_TRIANGLES); glVertex2f(100, 100); glVertex2f(300, 100); glVertex2f(100, 300); glEnd();
    px(150, 150, "triangle inside", 255, 0, 0, 255);
    px(280, 280, "triangle outside", 51, 102, 153, 255);

    /* interpolation */
    glBegin(GL_TRIANGLE_STRIP);
    glColor3f(0, 0, 0); glVertex2f(400, 100); glVertex2f(400, 200);
    glColor3f(0, 1, 0); glVertex2f(600, 100); glVertex2f(600, 200);
    glEnd();
    px(500, 150, "gouraud", 0, 128, 0, 255);

    /* depth: the farther quad, drawn second, loses */
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glColor3f(0, 1, 0); quad(50, 350, 150, 450, 0.5f);
    glColor3f(1, 1, 0); quad(50, 350, 150, 450, -0.5f + 0.9f);
    px(100, 400, "depth nearer second", 0, 255, 0, 255);
    glColor3f(1, 0, 1); quad(50, 350, 150, 450, 0.8f);          /* glOrtho(.., -1, 1): larger z is nearer */
    px(100, 400, "depth nearest", 255, 0, 255, 255);
    glDisable(GL_DEPTH_TEST);

    /* blend */
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(1, 1, 1, 0.5f); quad(200, 350, 300, 450, 0);
    px(250, 400, "blend", 153, 179, 204, -1);
    glDisable(GL_BLEND);

    /* scissor */
    glEnable(GL_SCISSOR_TEST);
    glScissor(350, 350, 50, 50);
    glColor3f(1, 1, 1); quad(320, 320, 450, 450, 0);
    px(375, 375, "scissor inside", 255, 255, 255, 255);
    px(420, 420, "scissor outside", 51, 102, 153, 255);
    glDisable(GL_SCISSOR_TEST);

    /* stencil: draw where a shape marked it */
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glColorMask(0, 0, 0, 0);
    quad(460, 350, 520, 450, 0);
    glColorMask(1, 1, 1, 1);
    glStencilFunc(GL_EQUAL, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    glColor3f(0, 1, 1); quad(460, 350, 620, 450, 0);
    px(490, 400, "stencil inside", 0, 255, 255, 255);
    px(580, 400, "stencil outside", 51, 102, 153, 255);
    glDisable(GL_STENCIL_TEST);

    /* textures */
    for (i = 0; i < 16; i++) {
        uint8_t *p = tex + 4 * i;
        int on = ((i & 1) ^ ((i >> 2) & 1));
        p[0] = on ? 255 : 0; p[1] = 128; p[2] = 0; p[3] = 255;
    }
    glGenTextures(2, t);
    glBindTexture(GL_TEXTURE_2D, t[0]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glEnable(GL_TEXTURE_2D);
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(1, 1, 1);
    quad(0, 0, 80, 80, 0);                                      /* 20 pixels per texel */
    px(10, 10, "texture texel 0", 0, 128, 0, 255);
    px(30, 10, "texture texel 1", 255, 128, 0, 255);
    glColor3f(0.5f, 1, 1);
    quad(100, 0, 180, 80, 0);
    px(130, 10, "texture modulate", 128, 128, 0, 255);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    quad(200, 0, 280, 80, 0);
    px(230, 10, "texture replace", 255, 128, 0, 255);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

    /* alpha test */
    glEnable(GL_ALPHA_TEST);
    glAlphaFunc(GL_GREATER, 0.5f);
    glDisable(GL_TEXTURE_2D);
    glColor4f(1, 0, 0, 0.4f); quad(300, 0, 380, 80, 0);
    px(340, 40, "alpha test rejected", 51, 102, 153, 255);
    glColor4f(1, 0, 0, 0.6f); quad(300, 0, 380, 80, 0);
    px(340, 40, "alpha test passed", 255, 0, 0, -1);
    glDisable(GL_ALPHA_TEST);

    /* fog: linear, half way */
    glEnable(GL_FOG);
    glFogi(GL_FOG_MODE, GL_LINEAR);
    glFogf(GL_FOG_START, 0.0f);
    glFogf(GL_FOG_END, 1.0f);
    {
        float fc[4] = { 0, 0, 1, 1 };
        glFogfv(GL_FOG_COLOR, fc);
    }
    glColor3f(1, 0, 0);
    quad(400, 0, 480, 80, -0.5f);
    px(440, 40, "fog", 128, 0, 128, 255);
    glDisable(GL_FOG);

    /* lighting: a directional light straight onto the quad */
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    {
        float pos[4] = { 0, 0, 1, 0 }, dif[4] = { 0.5f, 0.5f, 0.5f, 1 }, amb[4] = { 0, 0, 0, 1 }, mat[4] = { 1, 0.5f, 0, 1 }, none[4] = { 0, 0, 0, 1 };
        glLightfv(GL_LIGHT0, GL_POSITION, pos);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, dif);
        glLightfv(GL_LIGHT0, GL_AMBIENT, amb);
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, none);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, mat);
        glMaterialfv(GL_FRONT, GL_AMBIENT, none);
        glMaterialfv(GL_FRONT, GL_SPECULAR, none);
    }
    glNormal3f(0, 0, 1);
    quad(500, 0, 580, 80, 0);
    px(540, 40, "lighting", 128, 64, 0, 255);
    glDisable(GL_LIGHTING);

    /* display list, played twice at different places */
    list = glGenLists(1);
    glNewList(list, GL_COMPILE);
    glColor3f(0, 1, 0);
    glBegin(GL_TRIANGLES); glVertex2f(0, 0); glVertex2f(40, 0); glVertex2f(0, 40); glEnd();
    glEndList();
    glPushMatrix(); glTranslatef(100, 200, 0); glCallList(list); glPopMatrix();
    glPushMatrix(); glTranslatef(200, 200, 0); glCallList(list); glPopMatrix();
    px(110, 210, "display list 1", 0, 255, 0, 255);
    px(210, 210, "display list 2", 0, 255, 0, 255);

    /* client arrays: QUADS, unsigned byte colours, 32-bit indices */
    {
        static const float v[8] = { 300, 200, 380, 200, 380, 280, 300, 280 };
        static const uint8_t c[16] = { 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255 };
        static const uint32_t idx[4] = { 0, 1, 2, 3 };
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glVertexPointer(2, GL_FLOAT, 0, v);
        glColorPointer(4, GL_UNSIGNED_BYTE, 0, c);
        glDrawElements(GL_QUADS, 4, GL_UNSIGNED_INT, idx);
        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
        px(340, 240, "client arrays", 0, 0, 255, 255);
    }

    /* a texture update between two draws must not reach the first (the pass ordering rule of gl_tex.c) */
    {
        uint8_t green[4] = { 0, 255, 0, 255 };
        glEnable(GL_TEXTURE_2D);
        glColor3f(1, 1, 1);
        quad(400, 200, 440, 240, 0);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, green);
        quad(450, 200, 490, 240, 0);
        glDisable(GL_TEXTURE_2D);
        px(405, 205, "texture before its update", 0, 128, 0, 255);
        px(455, 205, "texture after its update", 0, 255, 0, 255);
    }

    /* render to texture, then draw it */
    {
        typedef void (APIENTRY *gen_t)(GLsizei, GLuint *);
        typedef void (APIENTRY *bind_t)(GLenum, GLuint);
        typedef void (APIENTRY *ftex_t)(GLenum, GLenum, GLenum, GLuint, GLint);
        typedef GLenum (APIENTRY *check_t)(GLenum);
        gen_t gen_fb = (gen_t)wglGetProcAddress("glGenFramebuffersEXT");
        bind_t bind_fb = (bind_t)wglGetProcAddress("glBindFramebufferEXT");
        ftex_t ftex = (ftex_t)wglGetProcAddress("glFramebufferTexture2DEXT");
        check_t check = (check_t)wglGetProcAddress("glCheckFramebufferStatusEXT");
        (void)rb;
        glBindTexture(GL_TEXTURE_2D, t[1]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 64, 64, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        gen_fb(1, &fb);
        bind_fb(GL_FRAMEBUFFER_EXT, fb);
        ftex(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t[1], 0);
        if (check(GL_FRAMEBUFFER_EXT) != GL_FRAMEBUFFER_COMPLETE_EXT) { printf("FAIL framebuffer incomplete\n"); g_bad++; }
        glViewport(0, 0, 64, 64);
        glClearColor(1, 1, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT);
        bind_fb(GL_FRAMEBUFFER_EXT, 0);
        glViewport(0, 0, W, H);
        glEnable(GL_TEXTURE_2D);
        glColor3f(1, 1, 1);
        quad(500, 200, 564, 264, 0);
        glDisable(GL_TEXTURE_2D);
        px(530, 230, "render to texture", 255, 255, 0, 255);
    }

    /* clears after draws (Quake 3 clears depth for each HUD icon): colour stays, depth changes only inside the scissor, and a stencil clear likewise */
    glClearColor(0.2f, 0.4f, 0.6f, 1.0f);
    glClearDepth(1.0);
    glClearStencil(0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glColor3f(1, 0, 0); quad(100, 100, 300, 200, 0.8f);         /* near */
    glEnable(GL_SCISSOR_TEST);
    glScissor(100, 100, 100, 100);
    glClear(GL_DEPTH_BUFFER_BIT);                               /* the left half forgets it */
    glDisable(GL_SCISSOR_TEST);
    glColor3f(0, 1, 0); quad(100, 100, 300, 200, 0.2f);         /* far: passes only where depth was cleared */
    glDisable(GL_DEPTH_TEST);
    px(150, 150, "depth clear mid-pass, inside the scissor", 0, 255, 0, 255);
    px(250, 150, "depth clear mid-pass, outside the scissor", 255, 0, 0, 255);
    px(350, 150, "depth clear mid-pass keeps the colour", 51, 102, 153, 255);
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 5, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glColor3f(1, 1, 0); quad(400, 100, 600, 200, 0);
    glEnable(GL_SCISSOR_TEST);
    glScissor(400, 100, 100, 100);
    glClearStencil(9);
    glClear(GL_STENCIL_BUFFER_BIT);
    glDisable(GL_SCISSOR_TEST);
    glClearStencil(0);
    glStencilFunc(GL_EQUAL, 9, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    glColor3f(0, 1, 1); quad(400, 100, 600, 200, 0);
    glDisable(GL_STENCIL_TEST);
    px(450, 150, "stencil clear mid-pass, inside the scissor", 0, 255, 255, 255);
    px(550, 150, "stencil clear mid-pass, outside the scissor", 255, 255, 0, 255);
    SwapBuffers(g_dc);

    /* depth survives a swap (only colour is undefined): a farther quad next frame must still lose to it */
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glColor3f(1, 0, 1); quad(0, 0, 640, 480, 0.8f);
    SwapBuffers(g_dc);
    glColor3f(1, 1, 1); quad(0, 0, 640, 480, -0.5f);            /* colour after the swap is undefined, the depth test is not */
    glDisable(GL_DEPTH_TEST);
    {
        uint8_t c[4];
        glReadPixels(320, 240, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, c);
        g_checks++;
        if (c[0] > 200 && c[1] > 200 && c[2] > 200) { g_bad++; printf("FAIL depth across a swap: the farther quad passed\n"); }
    }
    SwapBuffers(g_dc);
}

/* a buffer that stops changing moves to board A: over several frames, so the move and the drops happen inside the test */
typedef struct { float x, y; uint8_t c[4]; } vtx;
static void vquad(vtx *o, float x0, float x1, uint8_t r, uint8_t g, uint8_t b)
{
    int i;
    for (i = 0; i < 4; i++) { o[i].x = i == 1 || i == 2 ? x1 : x0; o[i].y = i >= 2 ? 480.0f : 0.0f; o[i].c[0] = r; o[i].c[1] = g; o[i].c[2] = b; o[i].c[3] = 255; }
}

static void test_kept(void)
{
    PFNGLGENBUFFERSPROC gen = (PFNGLGENBUFFERSPROC)wglGetProcAddress("glGenBuffers");
    PFNGLBINDBUFFERPROC bind = (PFNGLBINDBUFFERPROC)wglGetProcAddress("glBindBuffer");
    PFNGLBUFFERDATAPROC data = (PFNGLBUFFERDATAPROC)wglGetProcAddress("glBufferData");
    PFNGLBUFFERSUBDATAPROC sub = (PFNGLBUFFERSUBDATAPROC)wglGetProcAddress("glBufferSubData");
    PFNGLMAPBUFFERPROC map = (PFNGLMAPBUFFERPROC)wglGetProcAddress("glMapBuffer");
    PFNGLUNMAPBUFFERPROC unmap = (PFNGLUNMAPBUFFERPROC)wglGetProcAddress("glUnmapBuffer");
    PFNGLDELETEBUFFERSPROC del = (PFNGLDELETEBUFFERSPROC)wglGetProcAddress("glDeleteBuffers");
    static const GLushort ix[4] = { 4, 5, 6, 7 };
    vtx v[8], w[4];
    GLuint b = 0;
    int f;
    g_checks++;
    if (!gen || !bind || !data || !sub || !map || !unmap || !del) { g_bad++; printf("FAIL no buffer entry points\n"); return; }
    ortho();
    glDisable(GL_TEXTURE_2D); glDisable(GL_DEPTH_TEST); glDisable(GL_BLEND); glDisable(GL_LIGHTING); glDisable(GL_FOG); glDisable(GL_ALPHA_TEST); glDisable(GL_SCISSOR_TEST); glDisable(GL_STENCIL_TEST);
    vquad(v, 0, 320, 255, 0, 0);
    vquad(v + 4, 320, 640, 0, 255, 0);
    gen(1, &b);
    bind(GL_ARRAY_BUFFER, b);
    data(GL_ARRAY_BUFFER, sizeof v, v, GL_STATIC_DRAW);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glVertexPointer(2, GL_FLOAT, sizeof(vtx), (void *)0);
    glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(vtx), (void *)8);
    for (f = 0; f < 8; f++) {
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_QUADS, 0, 4);
        glDrawElements(GL_QUADS, 4, GL_UNSIGNED_SHORT, ix);
        px(160, 240, "kept: from vertex 0", 255, 0, 0, 255);
        px(480, 240, "kept: indices from 4", 0, 255, 0, 255);
        SwapBuffers(g_dc);
    }
    /* changed between two draws of a frame: the first keeps what it read */
    glClear(GL_COLOR_BUFFER_BIT);
    glDrawArrays(GL_QUADS, 0, 4);
    vquad(w, 320, 640, 0, 0, 255);
    sub(GL_ARRAY_BUFFER, 0, sizeof w, w);
    glDrawArrays(GL_QUADS, 0, 4);
    px(160, 240, "kept: a draw before the change", 255, 0, 0, 255);
    px(480, 240, "kept: a draw after the change", 0, 0, 255, 255);
    SwapBuffers(g_dc);
    /* a buffer that changes now and then, through glMapBuffer */
    for (f = 0; f < 48; f++) {
        int c = 64 + (f / 7) * 16;
        if (f % 7 == 0) {
            void *p = map(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
            vquad(w, 0, 640, (uint8_t)c, 0, 0);
            memcpy(p, w, sizeof w);
            unmap(GL_ARRAY_BUFFER);
        }
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_QUADS, 0, 4);
        px(320, 240, "kept: changed now and then", c, 0, 0, 255);
        SwapBuffers(g_dc);
    }
    /* a new buffer, on A by the end of the loop and never changed: glBufferData replaces it */
    del(1, &b);
    gen(1, &b);
    bind(GL_ARRAY_BUFFER, b);
    vquad(w, 0, 640, 255, 255, 0);
    data(GL_ARRAY_BUFFER, sizeof w, w, GL_STATIC_DRAW);
    glVertexPointer(2, GL_FLOAT, sizeof(vtx), (void *)0);
    glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(vtx), (void *)8);
    for (f = 0; f < 6; f++) {
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_QUADS, 0, 4);
        px(320, 240, "kept: a new buffer", 255, 255, 0, 255);
        SwapBuffers(g_dc);
    }
    vquad(w, 0, 640, 255, 0, 255);
    data(GL_ARRAY_BUFFER, sizeof w, w, GL_STATIC_DRAW);
    glClear(GL_COLOR_BUFFER_BIT);
    glDrawArrays(GL_QUADS, 0, 4);
    px(320, 240, "kept: replaced by glBufferData", 255, 0, 255, 255);
    SwapBuffers(g_dc);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    bind(GL_ARRAY_BUFFER, 0);
    del(1, &b);
}

/* what OpenGL 1.1 asks for and the hardware has no part of: each scene below can fail if the feature is not there */
#define GL_CLEAN(what) do { GLenum e_ = glGetError(); g_checks++; if (e_) { g_bad++; printf("FAIL %s: GL error %04x\n", what, e_); } } while (0)
static void ieq2(const char *what, int got, int want) { g_checks++; if (got != want) { g_bad++; printf("FAIL %s: %d, expected %d\n", what, got, want); } }
static void val(const char *what, double got, double want, double tol)
{
    g_checks++;
    if (fabs(got - want) > tol) { g_bad++; printf("FAIL %s: %g, expected %g\n", what, got, want); }
}

static void plain(void)
{
    ortho();
    glDisable(GL_TEXTURE_1D); glDisable(GL_TEXTURE_2D); glDisable(GL_DEPTH_TEST); glDisable(GL_BLEND); glDisable(GL_LIGHTING); glDisable(GL_FOG); glDisable(GL_ALPHA_TEST); glDisable(GL_SCISSOR_TEST);
    glDisable(GL_STENCIL_TEST); glDisable(GL_CULL_FACE); glDisable(GL_COLOR_LOGIC_OP); glDisable(GL_LINE_STIPPLE); glDisable(GL_POLYGON_STIPPLE);
    glColor4f(1, 1, 1, 1);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

static void test_locked_arrays(void)
{
    PFNGLLOCKARRAYSEXTPROC lock = (PFNGLLOCKARRAYSEXTPROC)wglGetProcAddress("glLockArraysEXT");
    PFNGLUNLOCKARRAYSEXTPROC unlock = (PFNGLUNLOCKARRAYSEXTPROC)wglGetProcAddress("glUnlockArraysEXT");
    float pos[16] = { 0, 0, 320, 0, 320, 480, 0, 480, 320, 0, 640, 0, 640, 480, 320, 480 };
    uint8_t col[8 * 4];
    int i;
    g_checks++;
    if (!lock || !unlock) { g_bad++; printf("FAIL no glLockArraysEXT\n"); return; }
    plain();
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 0, pos);
    glColorPointer(4, GL_UNSIGNED_BYTE, 0, col);
    lock(0, 8);
    /* as Quake 3: colours enabled after the lock are rewritten between the passes of it, and each pass must see them */
    glEnableClientState(GL_COLOR_ARRAY);
    for (i = 0; i < 8; i++) { col[i * 4] = 255; col[i * 4 + 1] = 0; col[i * 4 + 2] = 0; col[i * 4 + 3] = 255; }
    glDrawArrays(GL_QUADS, 0, 4);
    px(160, 240, "locked arrays: the first pass", 255, 0, 0, 255);
    for (i = 0; i < 8; i++) { col[i * 4] = 0; col[i * 4 + 1] = 255; }
    glDrawArrays(GL_QUADS, 0, 4);
    px(160, 240, "locked arrays: a colour array enabled after the lock is not locked", 0, 255, 0, 255);
    /* what was locked goes up once: a position changed under the lock is not read again. Undefined for an application, the proof for a test that the second upload did not happen */
    pos[2] = pos[4] = 100;
    glClear(GL_COLOR_BUFFER_BIT);
    glDrawArrays(GL_QUADS, 0, 4);
    px(200, 240, "locked arrays: positions go up once", 0, 255, 0, 255);
    glDrawArrays(GL_QUADS, 4, 4);
    px(480, 240, "locked arrays: a part of the locked range", 0, 255, 0, 255);
    unlock();
    pos[2] = pos[4] = 320;
    /* enabled before the lock, and rewritten with the pointer set again, as Quake 3's fog pass does: the new contents count */
    lock(0, 8);
    for (i = 0; i < 8; i++) { col[i * 4] = 0; col[i * 4 + 1] = 0; col[i * 4 + 2] = 255; }
    glDrawArrays(GL_QUADS, 0, 4);
    px(160, 240, "locked arrays: a locked colour array", 0, 0, 255, 255);
    for (i = 0; i < 8; i++) { col[i * 4] = 255; col[i * 4 + 2] = 0; }
    glColorPointer(4, GL_UNSIGNED_BYTE, 0, col);
    glDrawArrays(GL_QUADS, 0, 4);
    px(160, 240, "locked arrays: a pointer set under the lock unlocks its array", 255, 0, 0, 255);
    unlock();
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    GL_CLEAN("locked arrays");
}

static void test_tex1d(void)
{
    static const uint8_t t[16] = { 255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255 }, black[4] = { 0, 0, 0, 255 };
    GLuint n;
    GLint b = 0;
    plain();
    glGenTextures(1, &n);
    glBindTexture(GL_TEXTURE_1D, n);
    glGetIntegerv(GL_TEXTURE_BINDING_1D, &b);
    val("1D: the binding", b, n, 0);
    glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, t);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glEnable(GL_TEXTURE_1D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    quad(0, 0, 640, 480, 0);
    px(80, 240, "1D: texel 0", 255, 0, 0, 255);
    px(240, 240, "1D: texel 1", 0, 255, 0, 255);
    px(400, 240, "1D: texel 2", 0, 0, 255, 255);
    px(560, 240, "1D: texel 3", 255, 255, 255, 255);
    px(80, 420, "1D: t does not matter", 255, 0, 0, 255);
    glTexSubImage1D(GL_TEXTURE_1D, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, black);
    quad(0, 0, 640, 480, 0);
    px(240, 240, "1D: glTexSubImage1D", 0, 0, 0, 255);
    px(400, 240, "1D: its neighbour", 0, 0, 255, 255);
    glDisable(GL_TEXTURE_1D);
    glClearColor(0.5f, 0.25f, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glCopyTexSubImage1D(GL_TEXTURE_1D, 0, 2, 10, 10, 1);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_TEXTURE_1D);
    quad(0, 0, 640, 480, 0);
    px(400, 240, "1D: glCopyTexSubImage1D", 128, 64, 0, 255);
    px(80, 240, "1D: outside the copy", 255, 0, 0, 255);
    glDisable(GL_TEXTURE_1D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glDeleteTextures(1, &n);
    GL_CLEAN("1D textures");
}

static void test_pixel_transfer(void)
{
    uint8_t d[16], rd[4];
    static const float r_to_r[2] = { 1, 0 }, i_r[2] = { 0, 1 }, i_g[2] = { 1, 0 }, i_b[2] = { 0, 0 }, i_a[2] = { 1, 1 };
    static const uint8_t idx[2] = { 0, 1 };
    float got[2] = { 0, 0 };
    GLint size = 0;
    int i;
    plain();
    for (i = 0; i < 4; i++) { d[i * 4] = 200; d[i * 4 + 1] = 100; d[i * 4 + 2] = 50; d[i * 4 + 3] = 255; }
    glRasterPos2i(10, 10);
    glPixelTransferf(GL_RED_SCALE, 0.5f);
    glPixelTransferf(GL_GREEN_BIAS, 0.25f);
    glDrawPixels(2, 2, GL_RGBA, GL_UNSIGNED_BYTE, d);
    glPixelTransferf(GL_RED_SCALE, 1.0f);
    glPixelTransferf(GL_GREEN_BIAS, 0.0f);
    px(10, 10, "pixel transfer: scale and bias", 100, 164, 50, 255);
    glPixelMapfv(GL_PIXEL_MAP_R_TO_R, 2, r_to_r);
    glPixelTransferi(GL_MAP_COLOR, GL_TRUE);
    for (i = 0; i < 4; i++) { d[i * 4] = 255; d[i * 4 + 1] = 255; d[i * 4 + 2] = 255; }
    glRasterPos2i(20, 10);
    glDrawPixels(2, 2, GL_RGBA, GL_UNSIGNED_BYTE, d);
    glPixelTransferi(GL_MAP_COLOR, GL_FALSE);
    px(20, 10, "pixel map: R_TO_R turns 1 to 0", 0, 255, 255, 255);
    glGetPixelMapfv(GL_PIXEL_MAP_R_TO_R, got);
    val("pixel map: read back 0", got[0], 1, 0);
    val("pixel map: read back 1", got[1], 0, 0);
    glGetIntegerv(GL_PIXEL_MAP_R_TO_R_SIZE, &size);
    val("pixel map: size", size, 2, 0);
    glPixelMapfv(GL_PIXEL_MAP_I_TO_R, 2, i_r);
    glPixelMapfv(GL_PIXEL_MAP_I_TO_G, 2, i_g);
    glPixelMapfv(GL_PIXEL_MAP_I_TO_B, 2, i_b);
    glPixelMapfv(GL_PIXEL_MAP_I_TO_A, 2, i_a);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glRasterPos2i(30, 10);
    glDrawPixels(2, 1, GL_COLOR_INDEX, GL_UNSIGNED_BYTE, idx);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    px(30, 10, "colour index: index 0 through the maps", 0, 255, 0, 255);
    px(31, 10, "colour index: index 1 through the maps", 255, 0, 0, 255);
    glPixelTransferf(GL_BLUE_SCALE, 0.5f);
    glClearColor(0, 0, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);
    glReadPixels(100, 100, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, rd);
    glPixelTransferf(GL_BLUE_SCALE, 1.0f);
    val("pixel transfer: glReadPixels", rd[2], 128, 1);
    GL_CLEAN("pixel transfer");
}

static void test_stipple(void)
{
    uint8_t m[128], back[128];
    GLint v = 0;
    plain();
    /* x: bits 0..3 of each byte on, from the top bit */
    memset(m, 0xF0, sizeof m);
    glPolygonStipple(m);
    glGetPolygonStipple(back);
    g_checks++;
    if (memcmp(m, back, sizeof m)) { g_bad++; printf("FAIL glGetPolygonStipple\n"); }
    glEnable(GL_POLYGON_STIPPLE);
    quad(0, 0, 640, 480, 0);
    px(1, 1, "polygon stipple: x 1 is on", 255, 255, 255, 255);
    px(5, 1, "polygon stipple: x 5 is off", 0, 0, 0, 255);
    px(9, 1, "polygon stipple: x 9 is on", 255, 255, 255, 255);
    px(33, 1, "polygon stipple: repeats at 32", 255, 255, 255, 255);
    px(37, 1, "polygon stipple: off at 37", 0, 0, 0, 255);
    /* y: the first 16 rows on: window y, counted from the bottom */
    memset(m, 0xFF, 64); memset(m + 64, 0x00, 64);
    glPolygonStipple(m);
    glClear(GL_COLOR_BUFFER_BIT);
    quad(0, 0, 640, 480, 0);
    px(3, 3, "polygon stipple: row 3 is on", 255, 255, 255, 255);
    px(3, 20, "polygon stipple: row 20 is off", 0, 0, 0, 255);
    px(3, 35, "polygon stipple: row 35 is on", 255, 255, 255, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    {
        static const uint8_t solid[16] = { 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255 };
        glRasterPos2i(3, 20);
        glDrawPixels(2, 2, GL_RGBA, GL_UNSIGNED_BYTE, solid);
        px(3, 20, "polygon stipple leaves glDrawPixels alone", 255, 255, 255, 255);
    }
    glDisable(GL_POLYGON_STIPPLE);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(1, 0x00FF);
    glGetIntegerv(GL_LINE_STIPPLE_PATTERN, &v);
    val("line stipple: the pattern", v, 0x00FF, 0);
    glBegin(GL_LINES); glVertex2f(100, 100.5f); glVertex2f(300, 100.5f); glEnd();
    px(103, 100, "line stipple: bit 3 is on", 255, 255, 255, 255);
    px(112, 100, "line stipple: bit 12 is off", 0, 0, 0, 255);
    px(120, 100, "line stipple: the pattern repeats", 255, 255, 255, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glLineStipple(2, 0x00FF);
    glBegin(GL_LINES); glVertex2f(100, 100.5f); glVertex2f(300, 100.5f); glEnd();
    px(112, 100, "line stipple: factor 2, bit 6 is on", 255, 255, 255, 255);
    px(120, 100, "line stipple: factor 2, bit 10 is off", 0, 0, 0, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glLineStipple(1, 0x00FF);
    /* a strip goes on counting through its corner, independent lines start over */
    glBegin(GL_LINE_STRIP); glVertex2f(100, 100.5f); glVertex2f(105, 100.5f); glVertex2f(130, 100.5f); glEnd();
    px(107, 100, "line stipple: a strip counts on, bit 7", 255, 255, 255, 255);
    px(110, 100, "line stipple: a strip counts on, bit 10", 0, 0, 0, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_LINES); glVertex2f(100, 100.5f); glVertex2f(110, 100.5f); glVertex2f(110, 100.5f); glVertex2f(130, 100.5f); glEnd();
    px(112, 100, "line stipple: independent lines start over", 255, 255, 255, 255);
    glDisable(GL_LINE_STIPPLE);
    GL_CLEAN("stipple");
}

/* lighting with known numbers: earlier scenes leave their own material and light behind */
static void known_light(void)
{
    static const float pos[4] = { 0, 0, 1, 0 }, white[4] = { 1, 1, 1, 1 }, none[4] = { 0, 0, 0, 1 }, amb[4] = { 0.2f, 0.2f, 0.2f, 1 }, dif[4] = { 0.8f, 0.8f, 0.8f, 1 };
    glEnable(GL_LIGHTING); glEnable(GL_LIGHT0); glDisable(GL_COLOR_MATERIAL);
    glLightfv(GL_LIGHT0, GL_POSITION, pos); glLightfv(GL_LIGHT0, GL_AMBIENT, none); glLightfv(GL_LIGHT0, GL_DIFFUSE, white); glLightfv(GL_LIGHT0, GL_SPECULAR, none);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb); glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif); glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, none); glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, none);
}

static void test_eval(void)
{
    static const float line[3][3] = { { 100, 100, 0 }, { 200, 300, 0 }, { 300, 100, 0 } };
    static const float patch[2][2][3] = { { { 0, 0, 0 }, { 0, 480, 0 } }, { { 640, 0, 0 }, { 640, 480, 0 } } };
    static const float colours[2][2][4] = { { { 1, 0, 0, 1 }, { 1, 0, 0, 1 } }, { { 0, 0, 1, 1 }, { 0, 0, 1, 1 } } };
    GLint order[2] = { 0, 0 };
    float cur[4], dom[4], one[3];
    plain();
    glMap1f(GL_MAP1_VERTEX_3, 0, 1, 3, 3, &line[0][0]);
    glEnable(GL_MAP1_VERTEX_3);
    glPointSize(3);
    glBegin(GL_POINTS); glEvalCoord1f(0.5f); glEnd();
    glPointSize(1);
    px(200, 200, "evaluator: the middle of a quadratic", 255, 255, 255, 255);
    px(200, 300, "evaluator: not its control point", 0, 0, 0, 255);
    glGetMapiv(GL_MAP1_VERTEX_3, GL_ORDER, order);
    val("evaluator: glGetMapiv ORDER", order[0], 3, 0);
    glGetMapfv(GL_MAP1_VERTEX_3, GL_COEFF, &one[0]);
    glGetMapfv(GL_MAP1_VERTEX_3, GL_DOMAIN, dom);
    val("evaluator: glGetMapfv DOMAIN", dom[1], 1, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    glMapGrid1f(100, 0, 1);
    glEvalMesh1(GL_LINE, 0, 100);
    px(150, 175, "evaluator: a mesh of lines", 255, 255, 255, 255);
    glDisable(GL_MAP1_VERTEX_3);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_POINTS); glEvalCoord1f(0.5f); glEnd();
    px(200, 200, "evaluator: no vertex map, no vertex", 0, 0, 0, 255);
    glMap2f(GL_MAP2_VERTEX_3, 0, 1, 6, 2, 0, 1, 3, 2, &patch[0][0][0]);
    glMap2f(GL_MAP2_COLOR_4, 0, 1, 8, 2, 0, 1, 4, 2, &colours[0][0][0]);
    glEnable(GL_MAP2_VERTEX_3); glEnable(GL_MAP2_COLOR_4);
    glMapGrid2f(8, 0, 1, 8, 0, 1);
    glColor3f(0, 1, 0);
    glEvalMesh2(GL_FILL, 0, 8, 0, 8);
    px(10, 240, "evaluator: the surface's near edge", 252, 0, 3, 255);
    px(320, 240, "evaluator: the surface's middle", 128, 0, 128, 255);
    px(630, 240, "evaluator: the surface's far edge", 3, 0, 252, 255);
    glGetFloatv(GL_CURRENT_COLOR, cur);
    val("evaluator: the current colour is left alone", cur[1], 1, 0);
    glDisable(GL_MAP2_COLOR_4);
    glClear(GL_COLOR_BUFFER_BIT);
    /* du x dv = +z, toward a light along +z: mostly diffuse. The other way it would be ambient alone */
    glEnable(GL_AUTO_NORMAL);
    known_light();
    glEvalMesh2(GL_FILL, 0, 8, 0, 8);
    px(320, 240, "evaluator: GL_AUTO_NORMAL", 214, 214, 214, 255);
    glDisable(GL_LIGHTING); glDisable(GL_LIGHT0); glDisable(GL_AUTO_NORMAL); glDisable(GL_MAP2_VERTEX_3);
    GL_CLEAN("evaluators");
    (void)one;
}

static void test_select(void)
{
    GLuint b[64];
    GLint hits;
    plain();
    glSelectBuffer(64, b);
    glRenderMode(GL_SELECT);
    glInitNames();
    glPushName(0);
    glLoadName(1); quad(10, 10, 100, 100, 0.5f);
    glLoadName(2); quad(700, 10, 800, 100, 0);                  /* outside the view */
    glLoadName(3); quad(200, 10, 300, 100, -0.5f);
    glEnable(GL_CULL_FACE);
    glLoadName(4);
    glBegin(GL_QUADS); glVertex2f(10, 200); glVertex2f(10, 300); glVertex2f(100, 300); glVertex2f(100, 200); glEnd();   /* clockwise: culled */
    glDisable(GL_CULL_FACE);
    hits = glRenderMode(GL_RENDER);
    val("selection: two hits", hits, 2, 0);
    val("selection: first record's names", b[0], 1, 0);
    val("selection: its name", b[3], 1, 0);
    val("selection: its depth", (double)b[1] / 4294967295.0, 0.25, 0.001);      /* z 0.5 in glOrtho(.., -1, 1) */
    val("selection: second record's name", b[7], 3, 0);
    val("selection: its depth", (double)b[6] / 4294967295.0, 0.75, 0.001);
    px(50, 50, "selection draws nothing", 0, 0, 0, 255);
    glSelectBuffer(2, b);
    glRenderMode(GL_SELECT);
    glInitNames();
    glPushName(1);
    quad(10, 10, 100, 100, 0);
    hits = glRenderMode(GL_RENDER);
    val("selection: overflow", hits, -1, 0);
    glPopName();
    GL_CLEAN("selection");
}

static void test_feedback(void)
{
    GLfloat f[128];
    GLint n;
    int i = 0;
    plain();
    glFeedbackBuffer(128, GL_3D_COLOR, f);
    glRenderMode(GL_FEEDBACK);
    glColor4f(1, 0, 0, 1);
    glBegin(GL_TRIANGLES); glVertex2f(10, 20); glVertex2f(30, 20); glVertex2f(10, 40); glEnd();
    glPassThrough(7);
    glBegin(GL_LINE_STRIP); glVertex2f(0, 5); glVertex2f(10, 5); glVertex2f(20, 5); glEnd();
    glBegin(GL_POINTS); glVertex2f(5, 6); glEnd();
    n = glRenderMode(GL_RENDER);
    val("feedback: the count", n, 2 + 3 * 7 + 2 + 2 * (1 + 2 * 7) + 1 + 7, 0);
    val("feedback: polygon token", f[i++], GL_POLYGON_TOKEN, 0);
    val("feedback: its vertices", f[i++], 3, 0);
    val("feedback: x", f[i], 10, 0.01);
    val("feedback: y", f[i + 1], 20, 0.01);
    val("feedback: z", f[i + 2], 0.5, 0.001);
    val("feedback: red", f[i + 3], 1, 0);
    val("feedback: green", f[i + 4], 0, 0);
    i += 21;
    val("feedback: pass-through token", f[i++], GL_PASS_THROUGH_TOKEN, 0);
    val("feedback: its value", f[i++], 7, 0);
    val("feedback: a strip starts with a reset", f[i], GL_LINE_RESET_TOKEN, 0);
    i += 15;
    val("feedback: and goes on", f[i], GL_LINE_TOKEN, 0);
    i += 15;
    val("feedback: point token", f[i++], GL_POINT_TOKEN, 0);
    val("feedback: its x", f[i], 5, 0.01);
    /* clipped: one corner out of the view turns a triangle into four vertices */
    glFeedbackBuffer(64, GL_2D, f);
    glRenderMode(GL_FEEDBACK);
    glBegin(GL_TRIANGLES); glVertex2f(100, 100); glVertex2f(800, 100); glVertex2f(100, 200); glEnd();
    n = glRenderMode(GL_RENDER);
    val("feedback: clipped, the vertices", f[1], 4, 0);
    val("feedback: clipped, the values", n, 2 + 4 * 2, 0);
    val("feedback: clipped at the window's edge", f[4], 640, 0.01);
    /* lit, as the hardware would have: a white light on the diffuse default */
    {
        known_light();
        glFeedbackBuffer(64, GL_3D_COLOR, f);
        glRenderMode(GL_FEEDBACK);
        glNormal3f(0, 0, 1);
        glBegin(GL_TRIANGLES); glVertex2f(10, 20); glVertex2f(30, 20); glVertex2f(10, 40); glEnd();
        glRenderMode(GL_RENDER);
        glDisable(GL_LIGHTING); glDisable(GL_LIGHT0);
        val("feedback: lit", f[5], 0.84, 0.01);
    }
    GL_CLEAN("feedback");
}

static void test_accum(void)
{
    GLint bits = 0;
    plain();
    glGetIntegerv(GL_ACCUM_RED_BITS, &bits);
    val("accumulation: its bits", bits, 16, 0);
    glClearAccum(0, 0, 0, 0);
    glClear(GL_ACCUM_BUFFER_BIT);
    glClearColor(1, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glAccum(GL_LOAD, 0.5f);
    glClearColor(0, 0, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glAccum(GL_ACCUM, 0.5f);
    glAccum(GL_RETURN, 1.0f);
    px(320, 240, "accumulation: the mean of two", 128, 0, 128, 255);
    glAccum(GL_MULT, 2.0f);
    glAccum(GL_RETURN, 1.0f);
    px(320, 240, "accumulation: GL_MULT", 255, 0, 255, 255);
    glAccum(GL_ADD, -0.5f);
    glAccum(GL_RETURN, 1.0f);
    px(320, 240, "accumulation: GL_ADD", 128, 0, 128, 255);
    glClearAccum(0.25f, 0, 0, 1);
    glClear(GL_ACCUM_BUFFER_BIT);
    glAccum(GL_RETURN, 2.0f);
    px(320, 240, "accumulation: cleared, and a return value", 128, 0, 0, 255);
    glClearColor(0, 0, 0, 1);
    GL_CLEAN("accumulation");
}

static void logic_case(GLenum op, const char *what, int sr, int sg, int sb, int dr, int dg, int db, int r, int g, int b)
{
    glDisable(GL_COLOR_LOGIC_OP);
    glClearColor((float)dr / 255.0f, (float)dg / 255.0f, (float)db / 255.0f, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_COLOR_LOGIC_OP);
    glLogicOp(op);
    glColor4ub((GLubyte)sr, (GLubyte)sg, (GLubyte)sb, 255);
    quad(100, 100, 300, 300, 0);
    px(200, 200, what, r, g, b, -1);
    px(400, 200, "logic op: outside the draw", dr, dg, db, -1);
    glDisable(GL_COLOR_LOGIC_OP);
}

static void test_logic(void)
{
    plain();
    logic_case(GL_XOR, "logic op: XOR", 0xFF, 0x0F, 0x55, 0x0F, 0xF0, 0xAA, 0xF0, 0xFF, 0xFF);
    /* the draw's colour is the background's: only coverage tells them apart */
    logic_case(GL_XOR, "logic op: XOR with the same colour", 0x40, 0x80, 0xC0, 0x40, 0x80, 0xC0, 0, 0, 0);
    logic_case(GL_AND, "logic op: AND", 0xF0, 0x3C, 0xFF, 0xCC, 0xFF, 0x0F, 0xC0, 0x3C, 0x0F);
    logic_case(GL_OR, "logic op: OR", 0x81, 0x18, 0x00, 0x42, 0x24, 0x00, 0xC3, 0x3C, 0x00);
    logic_case(GL_EQUIV, "logic op: EQUIV", 0xF0, 0x00, 0xFF, 0xF0, 0xFF, 0x00, 0xFF, 0x00, 0x00);
    logic_case(GL_NAND, "logic op: NAND", 0xFF, 0x0F, 0x00, 0xFF, 0x3C, 0x12, 0x00, 0xF3, 0xFF);
    logic_case(GL_INVERT, "logic op: INVERT", 0x12, 0x34, 0x56, 0x0F, 0xF0, 0x55, 0xF0, 0x0F, 0xAA);
    logic_case(GL_COPY_INVERTED, "logic op: COPY_INVERTED", 0x0F, 0xF0, 0x55, 0x12, 0x34, 0x56, 0xF0, 0x0F, 0xAA);
    logic_case(GL_CLEAR, "logic op: CLEAR", 0x12, 0x34, 0x56, 0x80, 0x80, 0x80, 0, 0, 0);
    logic_case(GL_SET, "logic op: SET", 0x12, 0x34, 0x56, 0x80, 0x80, 0x80, 255, 255, 255);
    logic_case(GL_NOOP, "logic op: NOOP", 0x12, 0x34, 0x56, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80);
    logic_case(GL_COPY, "logic op: COPY", 0x12, 0x34, 0x56, 0x80, 0x80, 0x80, 0x12, 0x34, 0x56);
    /* with the colour masked: green stays; and the depth it wrote still counts */
    glClearDepth(1.0);
    glClearColor(0x0F / 255.0f, 0xF0 / 255.0f, 0xAA / 255.0f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glColorMask(GL_TRUE, GL_FALSE, GL_TRUE, GL_TRUE);
    glEnable(GL_COLOR_LOGIC_OP);
    glLogicOp(GL_XOR);
    glColor4ub(0xFF, 0xFF, 0x00, 255);
    quad(100, 100, 300, 300, 0);
    glDisable(GL_COLOR_LOGIC_OP);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    px(200, 200, "logic op: under a colour mask", 0xF0, 0xF0, 0xAA, -1);
    glColor4f(0, 1, 0, 1);
    quad(100, 100, 300, 300, 0);
    px(200, 200, "logic op: its depth was written", 0xF0, 0xF0, 0xAA, -1);
    glDisable(GL_DEPTH_TEST);
    glClearColor(0, 0, 0, 1);
    GL_CLEAN("logic op");
}

/* the fixed-function fragment program: units that change only RGB (table 3.12) keep the colour's alpha, as do the colour sum and fog, whatever the
 * program does with it in between (gl_ff.c keeps it aside); and the alpha test, each function, on the comparison that fails it */
static void ffa_tex(GLuint t, GLenum fmt, const uint8_t *texel)
{
    glBindTexture(GL_TEXTURE_2D, t);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, (GLint)fmt, 1, 1, 0, fmt, GL_UNSIGNED_BYTE, texel);
}

static void ffa_quad(float x, float y, PFNGLMULTITEXCOORD2FPROC mt, PFNGLFOGCOORDFPROC fog)
{
    static const float C[4][2] = { { 0, 0 }, { 20, 0 }, { 20, 20 }, { 0, 20 } };
    int i;
    glBegin(GL_QUADS);
    for (i = 0; i < 4; i++) {
        glTexCoord2f(0.5f, 0.5f);
        if (mt) mt(GL_TEXTURE1, 0.5f, 0.5f);
        if (fog) fog(0.5f);
        glVertex2f(x + C[i][0], y + C[i][1]);
    }
    glEnd();
}

static void ffa_env(GLenum mode, float r, float g, float b, float a, float x, const char *what, int er, int eg, int eb, int ea)
{
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, (GLint)mode);
    glColor4f(r, g, b, a);
    ffa_quad(x, 10, NULL, NULL);
    px((int)x + 10, 20, what, er, eg, eb, ea);
}

/* alpha a against the reference: drawn (the texture's colour, alpha a) or not (the background) */
static void ffa_test(GLenum fn, float ref, float a, int x, int y, int drawn, const char *what)
{
    glAlphaFunc(fn, ref);
    glColor4f(1, 1, 1, a);
    ffa_quad((float)x, (float)y, NULL, NULL);
    if (drawn) px(x + 10, y + 10, what, 200, 100, 50, (int)(a * 255.0f + 0.5f));
    else px(x + 10, y + 10, what, 0, 0, 0, 255);
}

static void test_ff_alpha(void)
{
    PFNGLACTIVETEXTUREPROC at = (PFNGLACTIVETEXTUREPROC)wglGetProcAddress("glActiveTexture");
    PFNGLMULTITEXCOORD2FPROC mt = (PFNGLMULTITEXCOORD2FPROC)wglGetProcAddress("glMultiTexCoord2f");
    PFNGLSECONDARYCOLOR3FPROC sec = (PFNGLSECONDARYCOLOR3FPROC)wglGetProcAddress("glSecondaryColor3f");
    PFNGLFOGCOORDFPROC fogc = (PFNGLFOGCOORDFPROC)wglGetProcAddress("glFogCoordf");
    static const uint8_t rgb[3] = { 200, 100, 50 }, rgb2[3] = { 128, 255, 64 }, rgba[4] = { 200, 100, 50, 128 }, alpha[1] = { 102 };
    static const float red[4] = { 1, 0, 0, 1 }, blue0[4] = { 0, 0, 1, 0 };
    GLuint t[4];
    g_checks++;
    if (!at || !mt || !sec || !fogc) { g_bad++; printf("FAIL fixed-function alpha: entry points missing\n"); return; }
    plain();
    glGenTextures(4, t);
    ffa_tex(t[1], GL_RGB, rgb2);
    ffa_tex(t[2], GL_RGBA, rgba);
    ffa_tex(t[3], GL_ALPHA, alpha);
    ffa_tex(t[0], GL_RGB, rgb);
    glEnable(GL_TEXTURE_2D);
    glTexEnvfv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_COLOR, red);
    /* one unit: the colour's alpha stays */
    ffa_env(GL_MODULATE, 1, 1, 1, 0.5f, 10, "fixed-function alpha: RGB MODULATE", 200, 100, 50, 128);
    ffa_env(GL_REPLACE, 0, 0, 0, 0.75f, 40, "fixed-function alpha: RGB REPLACE", 200, 100, 50, 191);
    ffa_env(GL_DECAL, 0, 0, 0, 0.25f, 70, "fixed-function alpha: RGB DECAL", 200, 100, 50, 64);
    ffa_env(GL_ADD, 0.5f, 0.5f, 0.5f, 0.25f, 100, "fixed-function alpha: RGB ADD, clamped", 255, 228, 178, 64);
    ffa_env(GL_BLEND, 1, 1, 1, 0.5f, 130, "fixed-function alpha: RGB BLEND", 255, 155, 205, 128);
    glBindTexture(GL_TEXTURE_2D, t[2]);
    ffa_env(GL_DECAL, 0, 0, 0, 0.25f, 160, "fixed-function alpha: RGBA DECAL", 100, 50, 25, 64);
    /* two units */
    glBindTexture(GL_TEXTURE_2D, t[0]); glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    at(GL_TEXTURE1); glEnable(GL_TEXTURE_2D); glBindTexture(GL_TEXTURE_2D, t[1]); glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); at(GL_TEXTURE0);
    glColor4f(1, 1, 1, 0.5f); ffa_quad(190, 10, mt, NULL);
    px(200, 20, "fixed-function alpha: RGB x RGB lightmap", 100, 100, 13, 128);
    at(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, t[3]); at(GL_TEXTURE0);
    ffa_quad(220, 10, mt, NULL);
    px(230, 20, "fixed-function alpha: RGB, then ALPHA MODULATE", 200, 100, 50, 51);
    at(GL_TEXTURE1); glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); at(GL_TEXTURE0);
    ffa_quad(250, 10, mt, NULL);
    px(260, 20, "fixed-function alpha: RGB, then ALPHA REPLACE", 200, 100, 50, 102);
    at(GL_TEXTURE1); glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); glDisable(GL_TEXTURE_2D); at(GL_TEXTURE0);
    /* the colour sum and fog after an RGB unit; fog's colour has alpha 0 */
    sec(0, 0.25f, 0.5f); glEnable(GL_COLOR_SUM);
    glColor4f(1, 1, 1, 0.5f); ffa_quad(280, 10, NULL, NULL);
    glDisable(GL_COLOR_SUM); sec(0, 0, 0);
    px(290, 20, "fixed-function alpha: RGB, colour sum", 200, 164, 178, 128);
    glFogi(GL_FOG_MODE, GL_LINEAR); glFogf(GL_FOG_START, 0); glFogf(GL_FOG_END, 1); glFogfv(GL_FOG_COLOR, blue0); glFogi(GL_FOG_COORD_SRC, GL_FOG_COORD);
    glEnable(GL_FOG);
    ffa_quad(310, 10, NULL, fogc);
    glBindTexture(GL_TEXTURE_2D, t[2]);
    ffa_quad(340, 10, NULL, fogc);
    glDisable(GL_FOG); glFogi(GL_FOG_COORD_SRC, GL_FRAGMENT_DEPTH);
    px(320, 20, "fixed-function alpha: RGB, fog", 100, 50, 153, 128);
    px(350, 20, "fixed-function alpha: RGBA, fog", 100, 50, 153, 64);
    /* the alpha test: 0.25 and 0.75 against 0.5; 0 against 0 for the equal cases */
    glBindTexture(GL_TEXTURE_2D, t[0]);
    glEnable(GL_ALPHA_TEST);
    ffa_test(GL_NEVER, 0.5f, 0.75f, 10, 60, 0, "alpha test NEVER");
    ffa_test(GL_LESS, 0.5f, 0.25f, 40, 60, 1, "alpha test LESS, below");
    ffa_test(GL_LESS, 0.5f, 0.75f, 70, 60, 0, "alpha test LESS, above");
    ffa_test(GL_LESS, 0.0f, 0.0f, 100, 60, 0, "alpha test LESS, equal");
    ffa_test(GL_EQUAL, 0.0f, 0.0f, 130, 60, 1, "alpha test EQUAL, equal");
    ffa_test(GL_EQUAL, 0.0f, 0.25f, 160, 60, 0, "alpha test EQUAL, above");
    ffa_test(GL_LEQUAL, 0.5f, 0.25f, 190, 60, 1, "alpha test LEQUAL, below");
    ffa_test(GL_LEQUAL, 0.0f, 0.0f, 220, 60, 1, "alpha test LEQUAL, equal");
    ffa_test(GL_LEQUAL, 0.5f, 0.75f, 250, 60, 0, "alpha test LEQUAL, above");
    ffa_test(GL_GREATER, 0.5f, 0.25f, 280, 60, 0, "alpha test GREATER, below");
    ffa_test(GL_GREATER, 0.0f, 0.0f, 310, 60, 0, "alpha test GREATER, equal");
    ffa_test(GL_GREATER, 0.5f, 0.75f, 340, 60, 1, "alpha test GREATER, above");
    ffa_test(GL_NOTEQUAL, 0.0f, 0.0f, 370, 60, 0, "alpha test NOTEQUAL, equal");
    ffa_test(GL_NOTEQUAL, 0.0f, 0.25f, 400, 60, 1, "alpha test NOTEQUAL, above");
    ffa_test(GL_GEQUAL, 0.5f, 0.25f, 430, 60, 0, "alpha test GEQUAL, below");
    ffa_test(GL_GEQUAL, 0.0f, 0.0f, 460, 60, 1, "alpha test GEQUAL, equal");
    ffa_test(GL_GEQUAL, 0.5f, 0.75f, 490, 60, 1, "alpha test GEQUAL, above");
    ffa_test(GL_ALWAYS, 0.5f, 0.25f, 520, 60, 1, "alpha test ALWAYS");
    /* an RGBA texture's alpha meets the test (Quake III's cut-out surfaces); a logic op after it */
    glBindTexture(GL_TEXTURE_2D, t[2]);
    glAlphaFunc(GL_GEQUAL, 0.5f); glColor4f(1, 1, 1, 1); ffa_quad(10, 100, NULL, NULL);
    glAlphaFunc(GL_GREATER, 0.75f); ffa_quad(40, 100, NULL, NULL);
    px(20, 110, "alpha test of an RGBA texture: passes", 200, 100, 50, 128);
    px(50, 110, "alpha test of an RGBA texture: fails", 0, 0, 0, 255);
    glEnable(GL_COLOR_LOGIC_OP); glLogicOp(GL_COPY_INVERTED);
    glAlphaFunc(GL_GEQUAL, 0.5f); ffa_quad(70, 100, NULL, NULL);
    glAlphaFunc(GL_GREATER, 0.75f); ffa_quad(100, 100, NULL, NULL);
    glDisable(GL_COLOR_LOGIC_OP);
    px(80, 110, "alpha test, then COPY_INVERTED: passes", 55, 155, 205, -1);
    px(110, 110, "alpha test, then COPY_INVERTED: fails", 0, 0, 0, 255);
    glDisable(GL_ALPHA_TEST); glDisable(GL_TEXTURE_2D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glDeleteTextures(4, t);
    glColor4f(1, 1, 1, 1);
    GL_CLEAN("fixed-function alpha");
}

static void test_index_state(void)
{
    GLint i = 0;
    glIndexi(5);
    glGetIntegerv(GL_CURRENT_INDEX, &i);
    val("glIndex: the current index", i, 5, 0);
    GL_CLEAN("glIndex");
}

/* how many pixels of a rectangle have anything on them */
static int lit(int x0, int y0, int w, int h)
{
    uint8_t *c = (uint8_t *)malloc((size_t)w * h * 4);
    int i, n = 0;
    glReadPixels(x0, y0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, c);
    for (i = 0; i < w * h; i++) if (c[i * 4] > 128 || c[i * 4 + 1] > 128 || c[i * 4 + 2] > 128) n++;
    free(c);
    return n;
}

static void pv(float x, float y) { glVertex2f(x + 0.5f, y + 0.5f); }
static float depth_at(int x, int y) { float z = -1.0f; glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z); return z; }
static int stencil_at(int x, int y) { uint8_t s = 0xAA; glReadPixels(x, y, 1, 1, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, &s); return s; }
static void raster_is(const char *what, int want) { GLint v = -1; glGetIntegerv(GL_CURRENT_RASTER_POSITION_VALID, &v); val(what, v, want, 0); }

/* the raster position is a vertex that goes through everything a vertex does, and clipped as a point, §2.12 */
static void test_raster(void)
{
    static const uint8_t green[4] = { 0, 255, 0, 255 }, bits[1] = { 0xF0 };
    static const double right_half[4] = { 1, 0, 0, -320 };
    float v[4];
    plain();
    glRasterPos2f(100, 50);
    raster_is("raster position: inside", 1);
    glGetFloatv(GL_CURRENT_RASTER_POSITION, v);
    val("raster position: x", v[0], 100, 0.01); val("raster position: y", v[1], 50, 0.01); val("raster position: z", v[2], 0.5, 0.01); val("raster position: w", v[3], 1, 0.01);
    glRasterPos2f(-1, 50);
    raster_is("raster position: left of the view", 0);
    glDrawPixels(1, 1, GL_RGBA, GL_UNSIGNED_BYTE, green);
    px(100, 50, "raster position: an invalid one draws nothing, at the old place", 0, 0, 0, 255);
    px(0, 50, "raster position: an invalid one draws nothing, at the edge", 0, 0, 0, 255);
    glBitmap(0, 0, 0, 0, 10, 0, NULL);
    raster_is("raster position: glBitmap leaves an invalid one", 0);
    glRasterPos2f(W + 1.0f, 50);
    raster_is("raster position: right of the view", 0);
    glRasterPos3f(100, 50, 2);
    raster_is("raster position: past near", 0);
    glRasterPos3f(100, 50, -2);
    raster_is("raster position: past far", 0);
    glClipPlane(GL_CLIP_PLANE0, right_half); glEnable(GL_CLIP_PLANE0);
    glRasterPos2f(100, 50);
    raster_is("raster position: behind a clip plane", 0);
    glRasterPos2f(400, 50);
    raster_is("raster position: before a clip plane", 1);
    glDisable(GL_CLIP_PLANE0);
    /* what it carries */
    glColor4f(1, 0.5f, 0, 1);
    glTexCoord4f(0.25f, 0.5f, 0.75f, 1);
    glMatrixMode(GL_TEXTURE); glLoadIdentity(); glScalef(2, 2, 2); glMatrixMode(GL_MODELVIEW);
    glRasterPos2f(100, 50);
    glMatrixMode(GL_TEXTURE); glLoadIdentity(); glMatrixMode(GL_MODELVIEW);
    glGetFloatv(GL_CURRENT_RASTER_COLOR, v);
    val("raster colour: r", v[0], 1, 0.01); val("raster colour: g", v[1], 0.5, 0.01); val("raster colour: b", v[2], 0, 0.01);
    glGetFloatv(GL_CURRENT_RASTER_TEXTURE_COORDS, v);
    val("raster coordinates: through the texture matrix, s", v[0], 0.5, 0.01); val("raster coordinates: t", v[1], 1, 0.01); val("raster coordinates: r", v[2], 1.5, 0.01);
    glGetFloatv(GL_CURRENT_RASTER_DISTANCE, v);
    val("raster distance", v[0], 111.803, 0.01);
    known_light();
    glNormal3f(0, 0, 1);
    glRasterPos2f(100, 50);
    glGetFloatv(GL_CURRENT_RASTER_COLOR, v);
    val("raster colour: lit", v[0], 0.84, 0.01);
    glDisable(GL_LIGHTING);
    /* w is the clip w */
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glFrustum(-1, 1, -1, 1, 1, 10); glMatrixMode(GL_MODELVIEW);
    glRasterPos3f(0, 0, -2);
    raster_is("raster position: under a perspective", 1);
    glGetFloatv(GL_CURRENT_RASTER_POSITION, v);
    val("raster position: perspective x", v[0], W / 2, 0.01); val("raster position: perspective y", v[1], H / 2, 0.01); val("raster position: clip w", v[3], 2, 0.01);
    glRasterPos3f(0, 0, 2);
    raster_is("raster position: behind the eye", 0);
    ortho();
    /* a bitmap has the raster colour, whatever the colour is by then */
    glColor3f(1, 0, 0);
    glRasterPos2i(200, 100);
    glColor3f(0, 0, 1);
    glBitmap(8, 1, 0, 0, 8, 0, bits);
    px(201, 100, "glBitmap: the raster colour", 255, 0, 0, 255);
    px(206, 100, "glBitmap: its zeros", 0, 0, 0, 255);
    glGetFloatv(GL_CURRENT_RASTER_POSITION, v);
    val("glBitmap: moves the position", v[0], 208, 0.01);
    /* pixels through fog have the raster distance, not one of their own */
    {
        static const float fc[4] = { 1, 0, 0, 1 };
        static const uint8_t white[4] = { 255, 255, 255, 255 };
        glFogi(GL_FOG_MODE, GL_LINEAR); glFogf(GL_FOG_START, 0); glFogf(GL_FOG_END, 200); glFogfv(GL_FOG_COLOR, fc);
        glEnable(GL_FOG);
        glRasterPos2i(100, 0);
        glDrawPixels(1, 1, GL_RGBA, GL_UNSIGNED_BYTE, white);
        glDisable(GL_FOG);
        px(100, 0, "fog on pixels: the raster distance", 255, 128, 128, 255);
    }
    /* the depth range is taken once */
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_ALWAYS);
    glDepthRange(0.25, 0.75);
    glRasterPos3f(300, 100, 0.5f);
    glDrawPixels(1, 1, GL_RGBA, GL_UNSIGNED_BYTE, green);
    glDepthRange(0, 1);
    val("glDrawPixels: depth at the raster position", depth_at(300, 100), 0.375, 0.001);
    glDepthFunc(GL_LESS); glDisable(GL_DEPTH_TEST);
    GL_CLEAN("raster position");
}

static void quad_ccw(float x0, float y0, float x1, float y1) { pv(x0, y0); pv(x1, y0); pv(x1, y1); pv(x0, y1); }
static void quad_cw(float x0, float y0, float x1, float y1) { pv(x0, y0); pv(x0, y1); pv(x1, y1); pv(x1, y0); }
static void hollow(const char *what, int x0, int y0, int x1, int y1)
{
    char s[120];
    snprintf(s, sizeof s, "%s: the bottom edge", what); val(s, lit(x0 + 20, y0, x1 - x0 - 40, 1), x1 - x0 - 40, 0);
    snprintf(s, sizeof s, "%s: the left edge", what); val(s, lit(x0, y0 + 20, 1, y1 - y0 - 40), y1 - y0 - 40, 0);
    snprintf(s, sizeof s, "%s: the inside", what); val(s, lit(x0 + 10, y0 + 10, x1 - x0 - 20, y1 - y0 - 20), 0, 0);
}

static void test_polygon_mode(void)
{
    static const GLboolean ef[3] = { 1, 0, 1 };
    static const uint16_t rot[3] = { 1, 2, 0 };
    float tri[6] = { 100.5f, 100.5f, 300.5f, 100.5f, 100.5f, 300.5f };
    static const float red[4] = { 1, 0, 0, 1 }, green[4] = { 0, 1, 0, 1 }, black[4] = { 0, 0, 0, 1 };
    static const uint8_t white[4] = { 255, 255, 255, 255 };
    GLint m[2], b = -1;
    GLuint l;
    plain();
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glGetIntegerv(GL_POLYGON_MODE, m);
    val("polygon mode: front", m[0], GL_LINE, 0); val("polygon mode: back", m[1], GL_LINE, 0);
    glBegin(GL_QUADS); quad_ccw(100, 100, 300, 300); glEnd();
    hollow("a quad as lines", 100, 100, 300, 300);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_POLYGON); pv(400, 100); pv(600, 100); pv(600, 300); pv(500, 400); pv(400, 300); glEnd();
    val("a polygon as lines: no inner edge", lit(430, 130, 140, 150), 0, 0);
    val("a polygon as lines: its first edge", lit(450, 100, 100, 1), 100, 0);
    g_checks++;
    if (lit(440, 340, 20, 20) < 15) { g_bad++; printf("FAIL a polygon as lines: a slanted edge\n"); }
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_QUAD_STRIP); pv(100, 100); pv(100, 300); pv(200, 100); pv(200, 300); pv(300, 100); pv(300, 300); glEnd();
    val("a quad strip as lines: the rung between", lit(200, 120, 1, 160), 160, 0);
    val("a quad strip as lines: no diagonal", lit(110, 110, 80, 180) + lit(210, 110, 80, 180), 0, 0);
    /* culled as polygons */
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_CULL_FACE); glCullFace(GL_BACK);
    glBegin(GL_QUADS); quad_cw(100, 100, 300, 300); glEnd();
    val("polygon mode: a back face is culled", lit(90, 90, 220, 220), 0, 0);
    glBegin(GL_QUADS); quad_ccw(100, 100, 300, 300); glEnd();
    hollow("polygon mode: a front face under culling", 100, 100, 300, 300);
    glDisable(GL_CULL_FACE);
    /* each face its own mode, in one glBegin */
    glClear(GL_COLOR_BUFFER_BIT);
    glPolygonMode(GL_FRONT, GL_LINE); glPolygonMode(GL_BACK, GL_FILL);
    glGetIntegerv(GL_POLYGON_MODE, m);
    val("polygon mode: front apart", m[0], GL_LINE, 0); val("polygon mode: back apart", m[1], GL_FILL, 0);
    glBegin(GL_QUADS); quad_ccw(100, 100, 300, 300); quad_cw(400, 100, 600, 300); glEnd();
    hollow("polygon mode: the front face", 100, 100, 300, 300);
    px(500, 200, "polygon mode: the back face filled", 255, 255, 255, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glPolygonMode(GL_FRONT, GL_FILL); glPolygonMode(GL_BACK, GL_LINE);
    glBegin(GL_QUADS); quad_ccw(100, 100, 300, 300); quad_cw(400, 100, 600, 300); glEnd();
    px(200, 200, "polygon mode: the front face filled", 255, 255, 255, 255);
    hollow("polygon mode: the back face", 400, 100, 600, 300);
    glFrontFace(GL_CW);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_QUADS); quad_ccw(100, 100, 300, 300); glEnd();
    hollow("polygon mode: which face is front follows glFrontFace", 100, 100, 300, 300);
    glFrontFace(GL_CCW);
    /* points */
    glClear(GL_COLOR_BUFFER_BIT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
    glPointSize(3);
    glBegin(GL_QUADS); quad_ccw(100, 100, 300, 300); glEnd();
    glPointSize(1);
    val("a quad as points: its corners", lit(99, 99, 3, 3) + lit(299, 99, 3, 3) + lit(299, 299, 3, 3) + lit(99, 299, 3, 3), 36, 0);
    val("a quad as points: nothing else", lit(110, 90, 180, 220), 0, 0);
    /* edge flags */
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_TRIANGLES); glEdgeFlag(GL_TRUE); pv(100, 100); glEdgeFlag(GL_FALSE); pv(300, 100); glEdgeFlag(GL_TRUE); pv(100, 300); glEnd();
    val("edge flag: the edge after a true", lit(120, 100, 160, 1), 160, 0);
    val("edge flag: the edge before the first", lit(100, 120, 1, 160), 160, 0);
    val("edge flag: the edge after a false", lit(190, 190, 20, 20), 0, 0);
    glGetIntegerv(GL_EDGE_FLAG, &b);
    val("edge flag: the current one", b, 1, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    glEdgeFlag(GL_FALSE);
    glBegin(GL_TRIANGLES); pv(100, 100); pv(300, 100); pv(100, 300); glEnd();
    glEdgeFlag(GL_TRUE);
    val("edge flag: all of them false", lit(90, 90, 220, 220), 0, 0);
    /* a triangle fan with its inner edges off, as a tessellator gives */
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_TRIANGLES);
    glEdgeFlag(GL_TRUE); pv(100, 100); pv(300, 100); glEdgeFlag(GL_FALSE); pv(300, 300);
    glEdgeFlag(GL_FALSE); pv(100, 100); glEdgeFlag(GL_TRUE); pv(300, 300); pv(100, 300);
    glEnd();
    hollow("edge flag: two triangles as one quad", 100, 100, 300, 300);
    val("edge flag: a quad's edges in a list of triangles", lit(120, 300, 160, 1) + lit(300, 120, 1, 160), 320, 0);
    /* filled, they mean nothing */
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_TRIANGLES); glEdgeFlag(GL_FALSE); pv(100, 100); pv(300, 100); pv(100, 300); glEnd();
    glEdgeFlag(GL_TRUE);
    px(150, 150, "edge flag: nothing to a filled polygon", 255, 255, 255, 255);
    /* through an array, drawn and by elements */
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 0, tri);
    glEdgeFlagPointer(0, ef);
    glEnableClientState(GL_EDGE_FLAG_ARRAY);
    val("edge flag array: enabled", glIsEnabled(GL_EDGE_FLAG_ARRAY), 1, 0);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    val("edge flag array: glDrawArrays, kept", lit(120, 100, 160, 1), 160, 0);
    val("edge flag array: glDrawArrays, left out", lit(190, 190, 20, 20), 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_SHORT, rot);
    val("edge flag array: glDrawElements, kept", lit(100, 120, 1, 160), 160, 0);
    val("edge flag array: glDrawElements, left out", lit(190, 190, 20, 20), 0, 0);
    glDisableClientState(GL_EDGE_FLAG_ARRAY);
    glClear(GL_COLOR_BUFFER_BIT);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    val("edge flag array: off again", lit(190, 190, 20, 20), 19, 0);
    glEnableClientState(GL_EDGE_FLAG_ARRAY);
    glInterleavedArrays(GL_V2F, 0, tri);
    val("edge flag array: glInterleavedArrays turns it off", glIsEnabled(GL_EDGE_FLAG_ARRAY), 0, 0);
    glDisableClientState(GL_VERTEX_ARRAY);
    /* through a display list */
    glClear(GL_COLOR_BUFFER_BIT);
    l = glGenLists(1);
    glNewList(l, GL_COMPILE);
    glBegin(GL_TRIANGLES); glEdgeFlag(GL_TRUE); pv(100, 100); glEdgeFlag(GL_FALSE); pv(300, 100); glEdgeFlag(GL_TRUE); pv(100, 300); glEnd();
    glEndList();
    glGetIntegerv(GL_EDGE_FLAG, &b);
    val("edge flag: compiling does not change it", b, 1, 0);
    glCallList(l);
    glDeleteLists(l, 1);
    val("edge flag: in a list, kept", lit(120, 100, 160, 1), 160, 0);
    val("edge flag: in a list, left out", lit(190, 190, 20, 20), 0, 0);
    /* which vertex's colour */
    glClear(GL_COLOR_BUFFER_BIT);
    glShadeModel(GL_FLAT);
    glBegin(GL_TRIANGLES); glColor3f(1, 0, 0); pv(100, 100); glColor3f(0, 1, 0); pv(300, 100); glColor3f(0, 0, 1); pv(100, 300); glEnd();
    px(200, 100, "polygon mode flat: a triangle has its last vertex's colour", 0, 0, 255, 255);
    px(100, 200, "polygon mode flat: on every edge", 0, 0, 255, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_POLYGON); glColor3f(1, 0, 0); pv(100, 100); glColor3f(0, 1, 0); pv(300, 100); glColor3f(0, 0, 1); pv(300, 300); pv(100, 300); glEnd();
    px(200, 300, "polygon mode flat: a polygon has its first vertex's colour", 255, 0, 0, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_QUADS); glColor3f(1, 0, 0); pv(100, 100); glColor3f(0, 1, 0); pv(300, 100); glColor3f(0, 0, 1); pv(300, 300); glColor3f(1, 1, 0); pv(100, 300); glEnd();
    px(200, 100, "polygon mode flat: a quad has its last vertex's colour", 255, 255, 0, 255);
    glShadeModel(GL_SMOOTH);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_TRIANGLES); glColor3f(1, 0, 0); pv(100, 100); glColor3f(0, 1, 0); pv(300, 100); glColor3f(0, 0, 1); pv(100, 300); glEnd();
    px(200, 100, "polygon mode: smooth along an edge", 128, 128, 0, 255);
    glColor3f(1, 1, 1);
    /* two-sided lighting takes the back colour for an edge of a back face */
    known_light();
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, red); glMaterialfv(GL_BACK, GL_DIFFUSE, green);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, black);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_QUADS); glNormal3f(0, 0, 1); quad_ccw(100, 100, 300, 300); glNormal3f(0, 0, -1); quad_cw(400, 100, 600, 300); glEnd();
    px(200, 100, "polygon mode two-sided: a front face", 255, 0, 0, 255);
    px(500, 100, "polygon mode two-sided: a back face", 0, 255, 0, 255);
    glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
    glPointSize(3);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_QUADS); glNormal3f(0, 0, -1); quad_cw(400, 100, 600, 300); glEnd();
    glPointSize(1);
    px(400, 100, "polygon mode two-sided: points of a back face", 0, 255, 0, 255);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);
    glDisable(GL_LIGHTING);
    /* polygon offset per mode: the lines lie behind a filled quad, 0.001 of depth: 16777 of the smallest step */
    plain();
    glEnable(GL_DEPTH_TEST);
    glColor3f(1, 0, 0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_QUADS); glVertex3f(100, 100, 0); glVertex3f(300, 100, 0); glVertex3f(300, 300, 0); glVertex3f(100, 300, 0); glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3f(0, 1, 0);
    glPolygonOffset(0, -20000);
#define BEHIND() do { glBegin(GL_QUADS); glVertex3f(150.5f, 150.5f, -0.002f); glVertex3f(250.5f, 150.5f, -0.002f); glVertex3f(250.5f, 250.5f, -0.002f); glVertex3f(150.5f, 250.5f, -0.002f); glEnd(); } while (0)
    BEHIND();
    px(200, 150, "polygon offset: lines without it are behind", 255, 0, 0, 255);
    glEnable(GL_POLYGON_OFFSET_FILL);
    BEHIND();
    px(200, 150, "polygon offset: the fill enable is not for lines", 255, 0, 0, 255);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glEnable(GL_POLYGON_OFFSET_LINE);
    BEHIND();
    px(200, 150, "polygon offset: the line enable", 0, 255, 0, 255);
    glDisable(GL_POLYGON_OFFSET_LINE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
    glPointSize(3);
    glColor3f(0, 0, 1);
    glPolygonOffset(0, -30000);                                 /* nearer than the line just drawn there */
    BEHIND();
    px(250, 250, "polygon offset: points without it are behind", 0, 255, 0, 255);
    glEnable(GL_POLYGON_OFFSET_POINT);
    BEHIND();
    px(250, 250, "polygon offset: the point enable", 0, 0, 255, 255);
    glDisable(GL_POLYGON_OFFSET_POINT);
    glPointSize(1);
#undef BEHIND
    /* the slope: the same slanted plane, once filled and once as lines: 1 of depth over 200 pixels, so the factor moves it by 0.0025 */
    plain();
    glEnable(GL_DEPTH_TEST);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor3f(1, 0, 0);
#define SLANT() do { glBegin(GL_QUADS); glVertex3f(100.5f, 100.5f, 0.5f); glVertex3f(300.5f, 100.5f, -0.5f); glVertex3f(300.5f, 300.5f, -0.5f); glVertex3f(100.5f, 300.5f, 0.5f); glEnd(); } while (0)
    SLANT();
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glPolygonOffset(-2, 0);
    glColor3f(0, 1, 0);
    glEnable(GL_POLYGON_OFFSET_LINE);
    SLANT();
    glDisable(GL_POLYGON_OFFSET_LINE);
    px(200, 100, "polygon offset: the slope of a line's polygon", 0, 255, 0, 255);
#undef SLANT
    glPolygonOffset(0, 0);
    /* pixels are not polygons */
    plain();
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glPixelZoom(100, 100);
    glRasterPos2i(100, 100);
    glDrawPixels(1, 1, GL_RGBA, GL_UNSIGNED_BYTE, white);
    glPixelZoom(1, 1);
    px(150, 150, "glDrawPixels under GL_LINE: solid", 255, 255, 255, 255);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    /* nor do they stop at the viewport */
    glClear(GL_COLOR_BUFFER_BIT);
    glViewport(0, 0, 10, 10);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glMatrixMode(GL_MODELVIEW);
    glRasterPos2f(0, 0);
    glPixelZoom(100, 100);
    glDrawPixels(1, 1, GL_RGBA, GL_UNSIGNED_BYTE, white);
    glPixelZoom(1, 1);
    glViewport(0, 0, W, H);
    ortho();
    px(50, 50, "glDrawPixels: not clipped by a small viewport", 255, 255, 255, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glViewport(100, 100, 50, 50);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    glColor3f(1, 1, 1);
    glBegin(GL_QUADS); glVertex2f(-3, -3); glVertex2f(3, -3); glVertex2f(3, 3); glVertex2f(-3, 3); glEnd();
    glLineWidth(3);
    glBegin(GL_LINES); glVertex2f(-3, 0); glVertex2f(3, 0); glEnd();
    glLineWidth(1);
    glViewport(0, 0, W, H);
    ortho();
    px(125, 125, "the view volume: inside", 255, 255, 255, 255);
    px(100, 100, "the view volume: its corner", 255, 255, 255, 255);
    px(149, 149, "the view volume: its other corner", 255, 255, 255, 255);
    px(99, 125, "the view volume: left of it", 0, 0, 0, 255);
    px(150, 125, "the view volume: right of it", 0, 0, 0, 255);
    px(125, 99, "the view volume: below", 0, 0, 0, 255);
    px(125, 150, "the view volume: above", 0, 0, 0, 255);
    /* a point is whole or gone by its centre */
    glClear(GL_COLOR_BUFFER_BIT);
    glViewport(100, 100, 50, 50);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    glPointSize(11);
    glBegin(GL_POINTS); glVertex2f(0.96f, 0); glVertex2f(1.04f, 0.5f); glEnd();
    glPointSize(1);
    glViewport(0, 0, W, H);
    ortho();
    px(153, 125, "the view volume: a point inside stands out of it", 255, 255, 255, 255);
    px(147, 137, "the view volume: a point outside is gone", 0, 0, 0, 255);
    px(3, 3, "glDrawPixels: from the raster position", 0, 0, 0, 255);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    GL_CLEAN("polygon mode");
    glPolygonMode(GL_FRONT, 0x1234);
    val("glPolygonMode: a bad mode", glGetError(), GL_INVALID_ENUM, 0);
    glPolygonMode(0x1234, GL_LINE);
    val("glPolygonMode: a bad face", glGetError(), GL_INVALID_ENUM, 0);
}

/* depth and stencil pixels, §3.6.4, §4.3.1, and glCopyPixels of them: exact to the last of the 24 bits */
static void test_ds_pixels(void)
{
    static const GLuint zin[4] = { 0x12345612u, 0x89ABCD89u, 0xFEDCBAFEu, 0x00000100u };
    static const float z2[2] = { 0.25f, 0.75f };
    static const uint8_t st[4] = { 1, 2, 3, 0xFF }, bits[1] = { 0xA0 };
    static const GLuint smap[4] = { 9, 8, 7, 6 };
    GLuint zout[4];
    float one = 1.0f, e = 0.125f;
    int i;
    plain();
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glClearStencil(0);
    glClear(GL_STENCIL_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_ALWAYS);
    glRasterPos2i(10, 10);
    glDrawPixels(2, 2, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, zin);
    memset(zout, 0, sizeof zout);
    glReadPixels(10, 10, 2, 2, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, zout);
    for (i = 0; i < 4; i++) val("depth pixels: all 24 bits, there and back", zout[i], zin[i], 0);
    val("depth pixels: as a float", depth_at(11, 10), zin[1] / 4294967295.0, 1e-6);
    {
        static const GLushort us[1] = { 0x8000 };
        static const GLubyte ub[1] = { 0x80 };
        glRasterPos2i(20, 10);
        glDrawPixels(1, 1, GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT, us);
        val("depth pixels: unsigned short", depth_at(20, 10), 32768.0 / 65535.0, 1e-6);
        glDrawPixels(1, 1, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, ub);
        val("depth pixels: unsigned byte", depth_at(20, 10), 128.0 / 255.0, 1e-6);
    }
    /* no test, no depth written; and the mask */
    glDisable(GL_DEPTH_TEST);
    glRasterPos2i(10, 10);
    glDrawPixels(1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &e);
    glReadPixels(10, 10, 1, 1, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, zout);
    val("depth pixels: no writes without the test", zout[0], zin[0], 0);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDrawPixels(1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &e);
    glDepthMask(GL_TRUE);
    glReadPixels(10, 10, 1, 1, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, zout);
    val("depth pixels: the depth mask", zout[0], zin[0], 0);
    /* the test, and the fragments it lets by */
    plain();
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_ALWAYS);
    glColor3f(0, 0, 1); quad(20, 20, 60, 60, 0);
    glDepthFunc(GL_LESS);
    glColor3f(1, 0, 0); glRasterPos2i(20, 20); glColor3f(0, 1, 0);
    glDrawPixels(2, 1, GL_DEPTH_COMPONENT, GL_FLOAT, z2);
    px(20, 20, "depth pixels: passed, in the raster colour", 255, 0, 0, 255);
    px(21, 20, "depth pixels: failed", 0, 0, 255, 255);
    px(22, 20, "depth pixels: only those", 0, 0, 255, 255);
    val("depth pixels: written", depth_at(20, 20), 0.25, 1e-5);
    val("depth pixels: kept", depth_at(21, 20), 0.5, 1e-5);
    /* the stencil test sees them too */
    glClearStencil(3); glClear(GL_STENCIL_BUFFER_BIT); glClearStencil(0);
    glEnable(GL_STENCIL_TEST); glStencilFunc(GL_ALWAYS, 7, 255); glStencilOp(GL_KEEP, GL_INCR, GL_REPLACE);
    glRasterPos2i(20, 30);
    glDrawPixels(2, 1, GL_DEPTH_COMPONENT, GL_FLOAT, z2);
    val("depth pixels: stencil, depth passed", stencil_at(20, 30), 7, 0);
    val("depth pixels: stencil, depth failed", stencil_at(21, 30), 4, 0);
    val("depth pixels: stencil, beside them", stencil_at(22, 30), 3, 0);
    glStencilFunc(GL_EQUAL, 5, 255); glStencilOp(GL_ZERO, GL_KEEP, GL_KEEP);
    glRasterPos2i(20, 40);
    glDrawPixels(1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, z2);
    val("depth pixels: stencil failed", stencil_at(20, 40), 0, 0);
    val("depth pixels: stencil failed, no depth", depth_at(20, 40), 0.5, 1e-5);
    px(20, 40, "depth pixels: stencil failed, no colour", 0, 0, 255, 255);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP); glDisable(GL_STENCIL_TEST);
    /* zoom */
    glDepthFunc(GL_ALWAYS);
    glPixelZoom(3, 2);
    glRasterPos2i(100, 100);
    glDrawPixels(2, 1, GL_DEPTH_COMPONENT, GL_FLOAT, z2);
    val("depth pixels: zoom, the first", depth_at(102, 101), 0.25, 1e-5);
    val("depth pixels: zoom, the second", depth_at(103, 100), 0.75, 1e-5);
    val("depth pixels: zoom, the end", depth_at(105, 101), 0.75, 1e-5);
    val("depth pixels: zoom, past it", depth_at(106, 100), 1, 1e-5);
    val("depth pixels: zoom, above", depth_at(100, 102), 1, 1e-5);
    glPixelZoom(2, -2);
    glRasterPos2i(200, 200);
    glDrawPixels(2, 1, GL_DEPTH_COMPONENT, GL_FLOAT, z2);
    glPixelZoom(1, 1);
    val("depth pixels: downwards", depth_at(201, 198), 0.25, 1e-5);
    val("depth pixels: downwards, the second", depth_at(202, 199), 0.75, 1e-5);
    val("depth pixels: not upwards", depth_at(201, 200), 1, 1e-5);
    /* the raster colour's alpha meets the alpha test */
    glEnable(GL_ALPHA_TEST); glAlphaFunc(GL_GREATER, 0.5f);
    glColor4f(1, 1, 1, 0.25f); glRasterPos2i(60, 100); glColor4f(1, 1, 1, 1);
    glDrawPixels(1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, z2);
    val("depth pixels: the alpha test stops them", depth_at(60, 100), 1, 1e-5);
    glRasterPos2i(61, 100);
    glDrawPixels(1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, z2);
    val("depth pixels: the alpha test lets them by", depth_at(61, 100), 0.25, 1e-5);
    glDisable(GL_ALPHA_TEST);
    /* scale and bias, both ways */
    glPixelTransferf(GL_DEPTH_SCALE, 0.5f); glPixelTransferf(GL_DEPTH_BIAS, 0.25f);
    glRasterPos2i(120, 100);
    glDrawPixels(1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &one);
    val("glReadPixels: scale and bias", depth_at(120, 100), 0.625, 1e-5);
    glPixelTransferf(GL_DEPTH_SCALE, 1); glPixelTransferf(GL_DEPTH_BIAS, 0);
    val("depth pixels: scale and bias", depth_at(120, 100), 0.75, 1e-5);
    /* the scissor stops them */
    glEnable(GL_SCISSOR_TEST); glScissor(141, 100, 10, 10);
    glRasterPos2i(140, 100);
    glDrawPixels(2, 1, GL_DEPTH_COMPONENT, GL_FLOAT, z2);
    glDisable(GL_SCISSOR_TEST);
    val("depth pixels: scissored out", depth_at(140, 100), 1, 1e-5);
    val("depth pixels: scissored in", depth_at(141, 100), 0.75, 1e-5);
    /* stencil pixels: only the scissor and the write mask */
    glClearStencil(0xF0); glClear(GL_STENCIL_BUFFER_BIT); glClearStencil(0);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_STENCIL_TEST); glStencilFunc(GL_NEVER, 0, 255); glStencilOp(GL_ZERO, GL_ZERO, GL_ZERO);
    glDepthFunc(GL_NEVER);
    glStencilMask(0x0F);
    glRasterPos2i(300, 100);
    glDrawPixels(2, 2, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, st);
    glStencilMask(0xFF);
    glDepthFunc(GL_ALWAYS); glDisable(GL_STENCIL_TEST); glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP); glStencilFunc(GL_ALWAYS, 0, 255);
    glClearStencil(0); glClear(GL_STENCIL_BUFFER_BIT);
    glStencilMask(0x0F);
    glRasterPos2i(300, 110);
    glDrawPixels(1, 1, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, st + 3);
    glStencilMask(0xFF);
    val("stencil pixels: the mask keeps a bit out", stencil_at(300, 110), 0x0F, 0);
    glClearStencil(0xF0); glClear(GL_STENCIL_BUFFER_BIT); glClearStencil(0);
    glRasterPos2i(300, 100);
    { static const uint8_t back[4] = { 0xF1, 0xF2, 0xF3, 0xFF }; glDrawPixels(2, 2, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, back); }
    val("stencil pixels: the first, through the mask", stencil_at(300, 100), 0xF1, 0);
    val("stencil pixels: the second", stencil_at(301, 100), 0xF2, 0);
    val("stencil pixels: the next row", stencil_at(300, 101), 0xF3, 0);
    val("stencil pixels: the last", stencil_at(301, 101), 0xFF, 0);
    val("stencil pixels: past them", stencil_at(302, 100), 0xF0, 0);
    val("stencil pixels: no colour", lit(300, 100, 2, 2), 0, 0);
    glEnable(GL_SCISSOR_TEST); glScissor(311, 100, 10, 10);
    glRasterPos2i(310, 100);
    glDrawPixels(2, 1, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, st);
    glDisable(GL_SCISSOR_TEST);
    val("stencil pixels: scissored out", stencil_at(310, 100), 0xF0, 0);
    val("stencil pixels: scissored in", stencil_at(311, 100), 2, 0);
    glPixelTransferi(GL_INDEX_SHIFT, 1); glPixelTransferi(GL_INDEX_OFFSET, 3);
    glRasterPos2i(320, 100);
    glDrawPixels(1, 1, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, st + 1);
    val("glReadPixels: shift and offset of a stencil index", stencil_at(320, 100), 17, 0);
    glPixelMapuiv(GL_PIXEL_MAP_S_TO_S, 4, smap);
    glPixelTransferi(GL_INDEX_SHIFT, 0); glPixelTransferi(GL_INDEX_OFFSET, 0);
    val("stencil pixels: shift and offset", stencil_at(320, 100), 7, 0);
    glPixelTransferi(GL_INDEX_SHIFT, 1); glPixelTransferi(GL_INDEX_OFFSET, 3);
    glPixelTransferi(GL_MAP_STENCIL, 1);
    glDrawPixels(1, 1, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, st + 1);
    glPixelTransferi(GL_INDEX_SHIFT, 0); glPixelTransferi(GL_INDEX_OFFSET, 0);
    glPixelTransferi(GL_MAP_STENCIL, 0);
    val("stencil pixels: the map (7 & 3 is 3)", stencil_at(320, 100), 6, 0);
    glPixelZoom(2, 2);
    glRasterPos2i(330, 100);
    glDrawPixels(1, 1, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, st + 1);
    glPixelZoom(1, 1);
    val("stencil pixels: zoom", stencil_at(331, 101), 2, 0);
    val("stencil pixels: zoom, past it", stencil_at(332, 100), 0xF0, 0);
    glRasterPos2i(340, 100);
    glDrawPixels(4, 1, GL_STENCIL_INDEX, GL_BITMAP, bits);
    val("stencil pixels: a bit", stencil_at(340, 100), 1, 0);
    val("stencil pixels: a zero bit", stencil_at(341, 100), 0, 0);
    val("stencil pixels: the next bit", stencil_at(342, 100), 1, 0);
    /* glCopyPixels */
    glDepthFunc(GL_ALWAYS);
    glRasterPos2i(400, 100);
    glDrawPixels(2, 2, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, zin);
    glColor3f(0, 1, 0); glRasterPos2i(410, 100); glColor3f(1, 0, 0);
    glCopyPixels(400, 100, 2, 2, GL_DEPTH);
    glReadPixels(410, 100, 2, 2, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, zout);
    for (i = 0; i < 4; i++) val("glCopyPixels GL_DEPTH: every bit", zout[i], zin[i], 0);
    px(411, 101, "glCopyPixels GL_DEPTH: the raster colour", 0, 255, 0, 255);
    glDepthFunc(GL_NEVER);
    glRasterPos2i(420, 100);
    glCopyPixels(400, 100, 1, 1, GL_DEPTH);
    glDepthFunc(GL_ALWAYS);
    val("glCopyPixels GL_DEPTH: the test", depth_at(420, 100), 1, 1e-5);
    glRasterPos2i(430, 100);
    glDrawPixels(2, 2, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, st);
    glRasterPos2i(440, 100);
    glStencilMask(0x0F);
    glCopyPixels(430, 100, 2, 2, GL_STENCIL);
    glStencilMask(0xFF);
    val("glCopyPixels GL_STENCIL", stencil_at(441, 100), 0xF2, 0);
    val("glCopyPixels GL_STENCIL: the next row", stencil_at(440, 101), 0xF3, 0);
    /* what stencil the hardware's own tests see afterwards is what was written */
    glRasterPos2i(450, 100);
    { static const uint8_t mark[4] = { 5, 5, 5, 5 }; glDrawPixels(2, 2, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, mark); }
    glEnable(GL_STENCIL_TEST); glStencilFunc(GL_EQUAL, 5, 255);
    glDepthFunc(GL_ALWAYS);
    glColor3f(0, 1, 1); quad(445, 95, 460, 110, 0);
    glDisable(GL_STENCIL_TEST); glStencilFunc(GL_ALWAYS, 0, 255);
    px(451, 101, "stencil pixels: a later draw tests against them", 0, 255, 255, 255);
    px(453, 101, "stencil pixels: and not past them", 0, 0, 0, 255);
    glClear(GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glRasterPos2i(450, 100);
    { static const uint8_t mark[4] = { 5, 5, 5, 5 }; glDrawPixels(2, 2, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, mark); }
    glClear(GL_DEPTH_BUFFER_BIT);
    val("stencil pixels: a clear of depth leaves them", stencil_at(450, 100), 5, 0);
    glCopyPixels(0, 0, 1, 1, 0x1234);
    val("glCopyPixels: a bad type", glGetError(), GL_INVALID_ENUM, 0);
    glDepthFunc(GL_LESS); glDisable(GL_DEPTH_TEST);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glClearStencil(0); glClear(GL_STENCIL_BUFFER_BIT);
    GL_CLEAN("depth and stencil pixels");
}

static void test_readback(void)
{
    static const uint8_t tx[4 * 4] = { 20, 100, 200, 250, 40, 80, 120, 160, 1, 2, 3, 4, 5, 6, 7, 8 };
    uint8_t b[64], *q;
    uint16_t s[4];
    uint32_t u[4];
    int16_t ss[4];
    int8_t sb[4];
    float f[4];
    GLint i;
    GLuint t;
    plain();
    glClearColor(20 / 255.0f, 100 / 255.0f, 200 / 255.0f, 250 / 255.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(10, 10, 1, 1, GL_RGBA, GL_FLOAT, f);
    val("glReadPixels GL_FLOAT: r", f[0], 20 / 255.0, 1e-4); val("glReadPixels GL_FLOAT: g", f[1], 100 / 255.0, 1e-4); val("glReadPixels GL_FLOAT: b", f[2], 200 / 255.0, 1e-4); val("glReadPixels GL_FLOAT: a", f[3], 250 / 255.0, 1e-4);
    glReadPixels(10, 10, 1, 1, GL_RGBA, GL_UNSIGNED_SHORT, s);
    val("glReadPixels GL_UNSIGNED_SHORT", s[1], 100 * 257, 1);
    glReadPixels(10, 10, 1, 1, GL_RGBA, GL_UNSIGNED_INT, u);
    val("glReadPixels GL_UNSIGNED_INT", u[1], 100.0 * 16843009.0, 256);
    glReadPixels(10, 10, 1, 1, GL_RGBA, GL_BYTE, sb);
    val("glReadPixels GL_BYTE", sb[2], 100, 0);
    glReadPixels(10, 10, 1, 1, GL_RGBA, GL_SHORT, ss);
    val("glReadPixels GL_SHORT", ss[0], 2570, 1);
    glReadPixels(10, 10, 1, 1, GL_RED, GL_UNSIGNED_BYTE, b);
    val("glReadPixels GL_RED", b[0], 20, 0);
    glReadPixels(10, 10, 1, 1, GL_GREEN, GL_UNSIGNED_BYTE, b);
    val("glReadPixels GL_GREEN", b[0], 100, 0);
    glReadPixels(10, 10, 1, 1, GL_BLUE, GL_UNSIGNED_BYTE, b);
    val("glReadPixels GL_BLUE", b[0], 200, 0);
    glReadPixels(10, 10, 1, 1, GL_ALPHA, GL_UNSIGNED_BYTE, b);
    val("glReadPixels GL_ALPHA", b[0], 250, 0);
    glReadPixels(10, 10, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, b);
    val("glReadPixels GL_RGB", b[0] << 16 | b[1] << 8 | b[2], 20 << 16 | 100 << 8 | 200, 0);
    glReadPixels(10, 10, 1, 1, GL_BGR, GL_UNSIGNED_BYTE, b);
    val("glReadPixels GL_BGR", b[0] << 16 | b[1] << 8 | b[2], 200 << 16 | 100 << 8 | 20, 0);
    glReadPixels(10, 10, 1, 1, GL_BGRA, GL_UNSIGNED_BYTE, b);
    val("glReadPixels GL_BGRA", b[0] << 16 | b[1] << 8 | b[2], 200 << 16 | 100 << 8 | 20, 0);
    val("glReadPixels GL_BGRA: alpha", b[3], 250, 0);
    /* luminance is the sum, and it clamps */
    glReadPixels(10, 10, 1, 1, GL_LUMINANCE_ALPHA, GL_FLOAT, f);
    val("glReadPixels GL_LUMINANCE clamps", f[0], 1, 1e-4);
    val("glReadPixels GL_LUMINANCE_ALPHA", f[1], 250 / 255.0, 1e-4);
    glClearColor(10 / 255.0f, 20 / 255.0f, 30 / 255.0f, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(20 / 255.0f, 100 / 255.0f, 200 / 255.0f, 250 / 255.0f);
    glReadPixels(10, 10, 1, 1, GL_LUMINANCE, GL_UNSIGNED_BYTE, b);
    val("glReadPixels GL_LUMINANCE: the sum", b[0], 60, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);
    /* packed */
    glReadPixels(10, 10, 1, 1, GL_RGB, GL_UNSIGNED_SHORT_5_6_5, s);
    val("5_6_5", s[0], 2 << 11 | 25 << 5 | 24, 0);
    glReadPixels(10, 10, 1, 1, GL_RGB, GL_UNSIGNED_SHORT_5_6_5_REV, s);
    val("5_6_5_REV", s[0], 24 << 11 | 25 << 5 | 2, 0);
    glReadPixels(10, 10, 1, 1, GL_RGBA, GL_UNSIGNED_SHORT_4_4_4_4, s);
    val("4_4_4_4", s[0], 0x16CF, 0);
    glReadPixels(10, 10, 1, 1, GL_RGBA, GL_UNSIGNED_SHORT_4_4_4_4_REV, s);
    val("4_4_4_4_REV", s[0], 0xFC61, 0);
    glReadPixels(10, 10, 1, 1, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, s);
    val("5_5_5_1", s[0], 2 << 11 | 12 << 6 | 24 << 1 | 1, 0);
    glReadPixels(10, 10, 1, 1, GL_RGBA, GL_UNSIGNED_SHORT_1_5_5_5_REV, s);
    val("1_5_5_5_REV", s[0], 1 << 15 | 24 << 10 | 12 << 5 | 2, 0);
    glReadPixels(10, 10, 1, 1, GL_RGBA, GL_UNSIGNED_INT_8_8_8_8, u);
    val("8_8_8_8", u[0], 0x1464C8FAu, 0);
    glReadPixels(10, 10, 1, 1, GL_RGBA, GL_UNSIGNED_INT_8_8_8_8_REV, u);
    val("8_8_8_8_REV", u[0], 0xFAC86414u, 0);
    glReadPixels(10, 10, 1, 1, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, u);
    val("GL_BGRA 8_8_8_8_REV", u[0], 0xFA1464C8u, 0);
    glReadPixels(10, 10, 1, 1, GL_RGBA, GL_UNSIGNED_INT_10_10_10_2, u);
    val("10_10_10_2", u[0], 80u << 22 | 401u << 12 | 802u << 2 | 3u, 0);
    glReadPixels(10, 10, 1, 1, GL_RGBA, GL_UNSIGNED_INT_2_10_10_10_REV, u);
    val("2_10_10_10_REV", u[0], 3u << 30 | 802u << 20 | 401u << 10 | 80u, 0);
    glReadPixels(10, 10, 1, 1, GL_RGB, GL_UNSIGNED_BYTE_3_3_2, b);
    val("3_3_2", b[0], 0x2E, 0);
    glReadPixels(10, 10, 1, 1, GL_RGB, GL_UNSIGNED_BYTE_2_3_3_REV, b);
    val("2_3_3_REV", b[0], 0x99, 0);
    /* and the other way */
    {
        static const uint8_t p1 = 0x2E, p2 = 0x99;
        static const uint32_t p3 = 80u << 22 | 401u << 12 | 802u << 2 | 3u, p4 = 3u << 30 | 802u << 20 | 401u << 10 | 80u;
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glRasterPos2i(20, 20);
        glDrawPixels(1, 1, GL_RGB, GL_UNSIGNED_BYTE_3_3_2, &p1);
        px(20, 20, "glDrawPixels 3_3_2", 36, 109, 170, 255);
        glRasterPos2i(21, 20);
        glDrawPixels(1, 1, GL_RGB, GL_UNSIGNED_BYTE_2_3_3_REV, &p2);
        px(21, 20, "glDrawPixels 2_3_3_REV", 36, 109, 170, 255);
        glRasterPos2i(22, 20);
        glDrawPixels(1, 1, GL_RGBA, GL_UNSIGNED_INT_10_10_10_2, &p3);
        px(22, 20, "glDrawPixels 10_10_10_2", 20, 100, 200, 255);
        glRasterPos2i(23, 20);
        glDrawPixels(1, 1, GL_RGBA, GL_UNSIGNED_INT_2_10_10_10_REV, &p4);
        px(23, 20, "glDrawPixels 2_10_10_10_REV", 20, 100, 200, 255);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    }
    /* what glPixelStore says about the destination */
    memset(b, 0xAA, sizeof b);
    glPixelStorei(GL_PACK_ALIGNMENT, 8);
    glReadPixels(10, 10, 3, 2, GL_RGB, GL_UNSIGNED_BYTE, b);
    val("PACK_ALIGNMENT: the second row starts on 16", b[16], 20, 0);
    val("PACK_ALIGNMENT: the pad is left alone", b[9], 0xAA, 0);
    memset(b, 0xAA, sizeof b);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ROW_LENGTH, 5); glPixelStorei(GL_PACK_SKIP_PIXELS, 1); glPixelStorei(GL_PACK_SKIP_ROWS, 1);
    glReadPixels(10, 10, 2, 2, GL_RGB, GL_UNSIGNED_BYTE, b);
    glGetIntegerv(GL_PACK_ROW_LENGTH, &i);
    val("PACK_ROW_LENGTH: the query", i, 5, 0);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0); glPixelStorei(GL_PACK_SKIP_PIXELS, 0); glPixelStorei(GL_PACK_SKIP_ROWS, 0);
    q = b + 15 + 3;
    val("PACK_SKIP: where the first pixel lands", q[0], 20, 0);
    val("PACK_SKIP: before it", q[-1], 0xAA, 0);
    val("PACK_ROW_LENGTH: the second row", q[15 + 1], 100, 0);
    val("PACK_ROW_LENGTH: past the row", q[6], 0xAA, 0);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    /* partly outside: what is inside lands in its place */
    memset(b, 0xAA, sizeof b);
    glReadPixels(W - 1, 10, 2, 1, GL_RGBA, GL_UNSIGNED_BYTE, b);
    val("glReadPixels: a part outside, the part inside", b[1], 100, 0);
    GL_CLEAN("glReadPixels");
    glReadPixels(0, 0, 1, 1, GL_COLOR_INDEX, GL_UNSIGNED_BYTE, b);
    val("glReadPixels: no colour index in RGBA", glGetError(), GL_INVALID_OPERATION, 0);
    glReadPixels(0, 0, 1, 1, GL_RGBA, 0x1234, b);
    val("glReadPixels: a bad type", glGetError(), GL_INVALID_ENUM, 0);
    glReadPixels(0, 0, 1, 1, 0x1234, GL_UNSIGNED_BYTE, b);
    val("glReadPixels: a bad format", glGetError(), GL_INVALID_ENUM, 0);
    glReadPixels(0, 0, 1, 1, GL_RED, GL_UNSIGNED_SHORT_5_6_5, b);
    val("glReadPixels: 5_6_5 is for RGB", glGetError(), GL_INVALID_OPERATION, 0);
    glReadPixels(0, 0, 1, 1, GL_RGB, GL_UNSIGNED_SHORT_4_4_4_4, b);
    val("glReadPixels: 4_4_4_4 is for RGBA", glGetError(), GL_INVALID_OPERATION, 0);
    glReadPixels(0, 0, 1, 1, GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT_5_6_5, b);
    val("glReadPixels: no packed depth", glGetError(), GL_INVALID_OPERATION, 0);
    glReadPixels(0, 0, 1, 1, GL_DEPTH_COMPONENT, GL_BITMAP, b);
    val("glReadPixels: no depth in bits", glGetError(), GL_INVALID_ENUM, 0);
    glDrawPixels(1, 1, GL_DEPTH_COMPONENT, GL_BITMAP, b);
    val("glDrawPixels: no depth in bits", glGetError(), GL_INVALID_ENUM, 0);
    glDrawPixels(1, 1, GL_STENCIL_INDEX, 0x1234, b);
    val("glDrawPixels: a bad type for a stencil index", glGetError(), GL_INVALID_ENUM, 0);
    /* glGetTexImage */
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, tx);
    memset(b, 0, sizeof b);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, b);
    val("glGetTexImage GL_RGBA", memcmp(b, tx, 16), 0, 0);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_BGRA, GL_UNSIGNED_BYTE, b);
    val("glGetTexImage GL_BGRA", b[0], 200, 0);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, b);
    val("glGetTexImage GL_RGB: the pixel after", b[5], 120, 0);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_UNSIGNED_BYTE, b);
    val("glGetTexImage GL_RED: the pixel after", b[1], 40, 0);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_ALPHA, GL_UNSIGNED_BYTE, b);
    val("glGetTexImage GL_ALPHA", b[0], 250, 0);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, b);
    memcpy(f, b, 16);
    val("glGetTexImage GL_FLOAT", f[2], 200 / 255.0, 1e-4);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_SHORT_4_4_4_4, s);
    val("glGetTexImage 4_4_4_4", s[0], 0x16CF, 0);
    {
        static const uint8_t l[8] = { 100, 200, 0, 0, 10, 20, 0, 0 };           /* rows on 4 */
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, 2, 2, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, l);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, b);
        val("glGetTexImage of luminance: red", b[0], 100, 0); val("glGetTexImage of luminance: no green", b[1], 0, 0); val("glGetTexImage of luminance: alpha", b[3], 255, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA, 1, 1, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, l);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, b);
        val("glGetTexImage of luminance-alpha: red", b[0], 100, 0); val("glGetTexImage of luminance-alpha: no blue", b[2], 0, 0); val("glGetTexImage of luminance-alpha: alpha", b[3], 200, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_INTENSITY, 1, 1, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, l);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, b);
        val("glGetTexImage of intensity: red", b[0], 100, 0); val("glGetTexImage of intensity: alpha", b[3], 255, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, tx);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, b);
        val("glGetTexImage of RGB: alpha", b[3], 255, 0);
    }
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, tx);
    glTexImage2D(GL_TEXTURE_2D, 1, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, tx + 4);
    glGetTexImage(GL_TEXTURE_2D, 1, GL_RGBA, GL_UNSIGNED_BYTE, b);
    val("glGetTexImage: a mip level", b[0], 40, 0);
    GL_CLEAN("glGetTexImage");
    glGetTexImage(GL_TEXTURE_2D, -1, GL_RGBA, GL_UNSIGNED_BYTE, b);
    val("glGetTexImage: a bad level", glGetError(), GL_INVALID_VALUE, 0);
    glGetTexImage(GL_TEXTURE_2D, 0, 0x1234, GL_UNSIGNED_BYTE, b);
    val("glGetTexImage: a bad format", glGetError(), GL_INVALID_ENUM, 0);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_6_5, b);
    val("glGetTexImage: 5_6_5 is for RGB", glGetError(), GL_INVALID_OPERATION, 0);
    glDeleteTextures(1, &t);
}

/* a 450 x 450 quad with coordinates -1 .. 2: inside the texture from 150 to 300 */
static void wide_quad(void)
{
    glBegin(GL_QUADS);
    glTexCoord2f(-1, -1); glVertex2f(0, 0);
    glTexCoord2f(2, -1); glVertex2f(450, 0);
    glTexCoord2f(2, 2); glVertex2f(450, 450);
    glTexCoord2f(-1, 2); glVertex2f(0, 450);
    glEnd();
}

static void wrap(GLenum target, GLint s, GLint t, GLint filter)
{
    glTexParameteri(target, GL_TEXTURE_WRAP_S, s);
    glTexParameteri(target, GL_TEXTURE_WRAP_T, t);
    glTexParameteri(target, GL_TEXTURE_MIN_FILTER, filter);
    glTexParameteri(target, GL_TEXTURE_MAG_FILTER, filter);
}

static void test_border(void)
{
    static const float red[4] = { 1, 0, 0, 1 };
    static const uint8_t white[16] = { 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255 };
    static const uint8_t two[16] = { 255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255 };
    uint8_t ring[4 * 4 * 4], big[64 * 64 * 4], px1[4];
    float f[4];
    GLint v = -1;
    GLuint t;
    int x, y, lv;
    g_checks++;
    if (!strstr((const char *)glGetString(GL_EXTENSIONS), "GL_ARB_texture_border_clamp")) { g_bad++; printf("FAIL no GL_ARB_texture_border_clamp in the extensions\n"); }
    plain();
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glEnable(GL_TEXTURE_2D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, white);
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, red);
    glGetTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, f);
    val("border colour: the query", f[0] + f[1] * 2 + f[3] * 4, 5, 1e-4);
    /* GL_CLAMP stops at the texture's edge, half a texel past the last texel's middle: half of the border in the filter there */
    wrap(GL_TEXTURE_2D, GL_CLAMP, GL_CLAMP, GL_LINEAR);
    wide_quad();
    px(225, 225, "GL_CLAMP: inside", 255, 255, 255, 255);
    px(50, 225, "GL_CLAMP: to the left, half the border", 255, 128, 128, 255);
    px(400, 225, "GL_CLAMP: to the right", 255, 128, 128, 255);
    px(225, 50, "GL_CLAMP: below", 255, 128, 128, 255);
    px(225, 400, "GL_CLAMP: above", 255, 128, 128, 255);
    px(50, 50, "GL_CLAMP: a corner, three quarters", 255, 64, 64, 255);
    px(168, 225, "GL_CLAMP: a quarter texel in", 255, 190, 190, 255);
    px(281, 225, "GL_CLAMP: a quarter texel in, from the other side", 255, 190, 190, 255);
    wrap(GL_TEXTURE_2D, GL_CLAMP, GL_CLAMP, GL_NEAREST);
    wide_quad();
    px(50, 225, "GL_CLAMP nearest: no border in the filter", 255, 255, 255, 255);
    wrap(GL_TEXTURE_2D, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE, GL_LINEAR);
    wide_quad();
    px(50, 225, "GL_CLAMP_TO_EDGE: no border", 255, 255, 255, 255);
    /* to the border */
    wrap(GL_TEXTURE_2D, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER, GL_LINEAR);
    wide_quad();
    px(225, 225, "GL_CLAMP_TO_BORDER: inside", 255, 255, 255, 255);
    px(50, 225, "GL_CLAMP_TO_BORDER: outside", 255, 0, 0, 255);
    px(150, 225, "GL_CLAMP_TO_BORDER: at the edge, half", 255, 129, 129, 255);
    px(130, 225, "GL_CLAMP_TO_BORDER: on the way out", 255, 61, 61, 255);
    px(112, 225, "GL_CLAMP_TO_BORDER: past half a texel, none of the texture", 255, 2, 2, 255);
    wrap(GL_TEXTURE_2D, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER, GL_NEAREST);
    wide_quad();
    px(148, 225, "GL_CLAMP_TO_BORDER nearest: outside by a little", 255, 0, 0, 255);
    px(152, 225, "GL_CLAMP_TO_BORDER nearest: inside by a little", 255, 255, 255, 255);
    px(225, 148, "GL_CLAMP_TO_BORDER nearest: t", 255, 0, 0, 255);
    px(225, 302, "GL_CLAMP_TO_BORDER nearest: t, above", 255, 0, 0, 255);
    wrap(GL_TEXTURE_2D, GL_REPEAT, GL_CLAMP_TO_BORDER, GL_NEAREST);
    wide_quad();
    px(50, 225, "the axes are apart: s repeats", 255, 255, 255, 255);
    px(225, 50, "the axes are apart: t has the border", 255, 0, 0, 255);
    /* a texture with texels of its own: what the filter meets at the edge is that texel */
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, two);
    wrap(GL_TEXTURE_2D, GL_CLAMP, GL_CLAMP, GL_LINEAR);
    wide_quad();
    px(50, 187, "GL_CLAMP: the border with the edge's first texel", 255, 0, 0, 255);
    px(50, 262, "GL_CLAMP: the border with the edge's second texel", 128, 0, 128, 255);
    px(400, 187, "GL_CLAMP: the other edge", 128, 128, 0, 255);
    /* as the base format reads it */
    {
        static const float c[4] = { 0.5f, 0.25f, 0.125f, 0.75f };
        static const uint8_t l[4] = { 255, 255, 255, 255 };
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, c);
        wrap(GL_TEXTURE_2D, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, 2, 2, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, l);
        wide_quad();
        px(50, 225, "the border of luminance", 128, 128, 128, 255);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, white);
        wide_quad();
        px(50, 225, "the border of RGB", 128, 64, 32, 255);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE);
        glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_REPLACE); glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_ALPHA, GL_REPLACE);
        glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_RGB, GL_TEXTURE); glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_ALPHA, GL_TEXTURE);
        glColor4f(1, 1, 1, 0.5f);
        wide_quad();
        glColor4f(1, 1, 1, 1);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        px(50, 225, "the border of RGB: its alpha is 1", 128, 64, 32, 255);
    }
    {
        static const GLint i[4] = { 0x7FFFFFFF, 0, 0, 0x7FFFFFFF };
        static const float over[4] = { 2, -1, 0.5f, 1 };
        glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, i);
        glGetTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, f);
        val("border colour: as integers", f[0], 1, 1e-4);
        {
            static const GLint half[4] = { 0x3FFFFFFF, 0, 0, 0x7FFFFFFF };
            glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, half);
            glGetTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, f);
            val("border colour: as integers, the middle", f[0], 0.5, 1e-4);
        }
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, over);
        glGetTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, f);
        val("border colour: clamped above", f[0], 1, 1e-4); val("border colour: clamped below", f[1], 0, 1e-4);
    }
    /* a mip chain: the filter's edge is that level's, a texel at the level is 1 pixel here, and the quad starts a quarter of it off a pixel's middle */
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, red);
    wrap(GL_TEXTURE_2D, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
    memset(big, 255, sizeof big);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 64, 64, 0, GL_RGBA, GL_UNSIGNED_BYTE, big);
    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_FALSE);
    glBegin(GL_QUADS);
    glTexCoord2f(-0.5f, -0.5f); glVertex2f(0.25f, 0.25f);
    glTexCoord2f(1.5f, -0.5f); glVertex2f(24.25f, 0.25f);
    glTexCoord2f(1.5f, 1.5f); glVertex2f(24.25f, 24.25f);
    glTexCoord2f(-0.5f, 1.5f); glVertex2f(0.25f, 24.25f);
    glEnd();
    px(12, 12, "border with mipmaps: inside", 255, 255, 255, 255);
    px(2, 12, "border with mipmaps: outside", 255, 0, 0, 255);
    g_checks++;
    {
        uint8_t c[4];
        glReadPixels(6, 12, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, c);
        if (c[1] < 100 || c[1] > 245) { g_bad++; printf("FAIL border with mipmaps: the edge of the level, not of level 0: green %d\n", c[1]); }
    }
    /* level 0 with a border of its own: a ring */
    for (y = 0; y < 4; y++) {
        for (x = 0; x < 4; x++) {
            static const uint8_t K[6][4] = { { 255, 255, 255, 255 }, { 255, 0, 0, 255 }, { 0, 255, 0, 255 }, { 0, 0, 255, 255 }, { 255, 255, 0, 255 } };
            int k = y == 0 ? 3 : y == 3 ? 4 : x == 0 ? 1 : x == 3 ? 2 : 0;
            memcpy(&ring[(y * 4 + x) * 4], K[k], 4);
        }
    }
    glDeleteTextures(1, &t);
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 4, 4, 1, GL_RGBA, GL_UNSIGNED_BYTE, ring);
    GL_CLEAN("a texture with a border");
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_BORDER, &v);
    val("a texture's border: the query", v, 1, 0);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &v);
    val("a texture's border: the width has it", v, 4, 0);
    wrap(GL_TEXTURE_2D, GL_CLAMP, GL_CLAMP, GL_LINEAR);
    wide_quad();
    px(225, 225, "bordered GL_CLAMP: inside", 255, 255, 255, 255);
    px(50, 225, "bordered GL_CLAMP: the left of the ring, half", 255, 128, 128, 255);
    px(400, 225, "bordered GL_CLAMP: the right", 128, 255, 128, 255);
    px(225, 50, "bordered GL_CLAMP: the bottom", 128, 128, 255, 255);
    px(225, 400, "bordered GL_CLAMP: the top", 255, 255, 128, 255);
    wrap(GL_TEXTURE_2D, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER, GL_LINEAR);
    wide_quad();
    px(50, 225, "bordered GL_CLAMP_TO_BORDER: the left", 255, 0, 0, 255);
    px(400, 225, "bordered GL_CLAMP_TO_BORDER: the right", 0, 255, 0, 255);
    px(225, 50, "bordered GL_CLAMP_TO_BORDER: the bottom", 0, 0, 255, 255);
    px(225, 400, "bordered GL_CLAMP_TO_BORDER: the top", 255, 255, 0, 255);
    wrap(GL_TEXTURE_2D, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER, GL_NEAREST);
    wide_quad();
    px(148, 225, "bordered nearest: the ring at once", 255, 0, 0, 255);
    px(152, 225, "bordered nearest: then the inside", 255, 255, 255, 255);
    wrap(GL_TEXTURE_2D, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE, GL_LINEAR);
    wide_quad();
    px(50, 225, "bordered GL_CLAMP_TO_EDGE: not the ring", 255, 255, 255, 255);
    /* what writes to it */
    px1[0] = 0; px1[1] = 0; px1[2] = 0; px1[3] = 255;
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, px1);
    wrap(GL_TEXTURE_2D, GL_CLAMP, GL_CLAMP, GL_NEAREST);
    wide_quad();
    px(187, 187, "bordered: glTexSubImage2D inside", 0, 0, 0, 255);
    px(262, 187, "bordered: glTexSubImage2D, beside it", 255, 255, 255, 255);
    glTexSubImage2D(GL_TEXTURE_2D, 0, -1, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, px1);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, ring);
    val("bordered: glTexSubImage2D on the ring", ring[(1 * 4 + 0) * 4], 0, 0);
    val("bordered: glGetTexImage has the ring", ring[(2 * 4 + 0) * 4], 255, 0);
    val("bordered: glGetTexImage, its right side", ring[(2 * 4 + 3) * 4 + 1], 255, 0);
    GL_CLEAN("texture border, writes");
    /* mipmapped, with a ring of one colour */
    glDeleteTextures(1, &t);
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    for (lv = 0; lv < 3; lv++) {
        int n = (4 >> lv) + 2;
        for (y = 0; y < n; y++) {
            for (x = 0; x < n; x++) {
                int e = !x || !y || x == n - 1 || y == n - 1;
                big[(y * n + x) * 4] = 255; big[(y * n + x) * 4 + 1] = e ? 0 : 255; big[(y * n + x) * 4 + 2] = e ? 0 : 255; big[(y * n + x) * 4 + 3] = 255;
            }
        }
        glTexImage2D(GL_TEXTURE_2D, lv, GL_RGBA, n, n, 1, GL_RGBA, GL_UNSIGNED_BYTE, big);
    }
    wrap(GL_TEXTURE_2D, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
    glClear(GL_COLOR_BUFFER_BIT);
    wide_quad();
    px(50, 225, "a ring of one colour with mipmaps", 255, 0, 0, 255);
    glDeleteTextures(1, &t);
    /* glCopyTexImage2D takes the ring from the framebuffer */
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glDisable(GL_TEXTURE_2D);
    for (y = 0; y < 4; y++) {
        for (x = 0; x < 4; x++) {
            int e = x == 0;
            ring[(y * 4 + x) * 4] = 255; ring[(y * 4 + x) * 4 + 1] = e ? 0 : 255; ring[(y * 4 + x) * 4 + 2] = e ? 0 : 255; ring[(y * 4 + x) * 4 + 3] = 255;
        }
    }
    glRasterPos2i(500, 10);
    glDrawPixels(4, 4, GL_RGBA, GL_UNSIGNED_BYTE, ring);
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 500, 10, 4, 4, 1);
    GL_CLEAN("glCopyTexImage2D with a border");
    glEnable(GL_TEXTURE_2D);
    wrap(GL_TEXTURE_2D, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER, GL_NEAREST);
    glClear(GL_COLOR_BUFFER_BIT);
    wide_quad();
    px(148, 225, "glCopyTexImage2D with a border: the ring", 255, 0, 0, 255);
    px(225, 225, "glCopyTexImage2D with a border: the inside", 255, 255, 255, 255);
    glDeleteTextures(1, &t);
    glDisable(GL_TEXTURE_2D);
    /* 1D */
    {
        static const uint8_t r1[16] = { 255, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 255, 0, 255 };
        glGenTextures(1, &t);
        glBindTexture(GL_TEXTURE_1D, t);
        glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA, 4, 1, GL_RGBA, GL_UNSIGNED_BYTE, r1);
        wrap(GL_TEXTURE_1D, GL_CLAMP, GL_CLAMP, GL_LINEAR);
        glEnable(GL_TEXTURE_1D);
        wide_quad();
        glDisable(GL_TEXTURE_1D);
        glBindTexture(GL_TEXTURE_1D, 0);
        glDeleteTextures(1, &t);
        px(50, 225, "1D with a border: the left", 255, 128, 128, 255);
        px(400, 225, "1D with a border: the right", 128, 255, 128, 255);
        px(225, 225, "1D with a border: inside", 255, 255, 255, 255);
    }
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 4, 4, 2, GL_RGBA, GL_UNSIGNED_BYTE, ring);
    val("a border of 2", glGetError(), GL_INVALID_VALUE, 0);
    glDeleteTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, 0);
    GL_CLEAN("texture border");
}

/* antialiasing: the cover of the pixel goes into alpha, and blending does the rest */
static void test_smooth(void)
{
    static const float pts[4] = { 100.5f, 300.5f, 300.5f, 300.5f };
    plain();
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SMOOTH);
    glPointSize(8);
    glBegin(GL_POINTS); glVertex2f(100.5f, 100.5f); glEnd();
    px(100, 100, "smooth point: the middle", 255, 255, 255, 255);
    px(103, 100, "smooth point: inside", 255, 255, 255, 255);
    px(104, 100, "smooth point: on the rim, half", 128, 128, 128, -1);
    px(100, 96, "smooth point: on the rim below", 128, 128, 128, -1);
    px(105, 100, "smooth point: outside", 0, 0, 0, 255);
    px(103, 103, "smooth point: round", 66, 66, 66, -1);
    px(104, 104, "smooth point: not a square's corner", 0, 0, 0, 255);
    glDisable(GL_BLEND);
    glBegin(GL_POINTS); glColor4f(1, 0, 0, 1); glVertex2f(200.5f, 100.5f); glEnd();
    px(204, 100, "smooth point: no blending, the cover in alpha", 255, 0, 0, 128);
    glEnable(GL_ALPHA_TEST); glAlphaFunc(GL_GREATER, 0.75f);
    glBegin(GL_POINTS); glColor4f(0, 1, 0, 1); glVertex2f(300.5f, 100.5f); glEnd();
    glDisable(GL_ALPHA_TEST);
    px(304, 100, "smooth point: the alpha test sees the cover", 0, 0, 0, 255);
    px(300, 100, "smooth point: the alpha test passes the middle", 0, 255, 0, 255);
    glColor4f(1, 1, 1, 1);
    glEnable(GL_BLEND);
    glPointSize(1);
    glDisable(GL_POINT_SMOOTH);
    /* lines */
    glEnable(GL_LINE_SMOOTH);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_LINES); glVertex2f(100.5f, 200.5f); glVertex2f(300.5f, 200.5f); glEnd();
    px(200, 200, "smooth line: on a row", 255, 255, 255, 255);
    px(200, 199, "smooth line: the row below", 0, 0, 0, 255);
    px(200, 201, "smooth line: the row above", 0, 0, 0, 255);
    px(299, 200, "smooth line: before the end", 255, 255, 255, 255);
    px(300, 200, "smooth line: the end, half", 128, 128, 128, -1);
    px(301, 200, "smooth line: past the end", 0, 0, 0, 255);
    px(100, 200, "smooth line: the start, half", 128, 128, 128, -1);
    px(99, 200, "smooth line: before the start", 0, 0, 0, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_LINES); glVertex2f(100, 200); glVertex2f(300, 200); glEnd();
    px(200, 200, "smooth line: between rows, half on one", 128, 128, 128, -1);
    px(200, 199, "smooth line: between rows, half on the other", 128, 128, 128, -1);
    px(200, 201, "smooth line: between rows, none above", 0, 0, 0, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glLineWidth(3);
    glBegin(GL_LINES); glVertex2f(100, 200.5f); glVertex2f(300, 200.5f); glEnd();
    px(200, 199, "smooth line 3: the row below", 255, 255, 255, 255);
    px(200, 201, "smooth line 3: the row above", 255, 255, 255, 255);
    px(200, 198, "smooth line 3: past it", 0, 0, 0, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glLineWidth(2);
    glBegin(GL_LINES); glVertex2f(100, 200.5f); glVertex2f(300, 200.5f); glEnd();
    glLineWidth(1);
    px(200, 200, "smooth line 2: the row on it", 255, 255, 255, 255);
    px(200, 201, "smooth line 2: half the row above", 128, 128, 128, -1);
    px(200, 198, "smooth line 2: none past it", 0, 0, 0, 255);
    /* a slanted one has the same width across */
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_LINES); glVertex2f(100.5f, 100.5f); glVertex2f(300.5f, 300.5f); glEnd();
    px(200, 200, "smooth line: on the diagonal", 255, 255, 255, 255);
    g_checks++;
    {
        uint8_t c[4];
        glReadPixels(201, 200, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, c);
        if (c[0] > 90) { g_bad++; printf("FAIL smooth line: a slanted line is as wide as a straight one: %d\n", c[0]); }
    }
    /* strip, loop, colours */
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_LINE_STRIP); glVertex2f(100, 100.5f); glVertex2f(200.5f, 100.5f); glVertex2f(200.5f, 200); glEnd();
    px(150, 100, "smooth strip: the first segment", 255, 255, 255, 255);
    px(200, 150, "smooth strip: the second segment", 255, 255, 255, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_LINE_LOOP); glVertex2f(100, 100.5f); glVertex2f(300, 100.5f); glVertex2f(300, 300.5f); glVertex2f(100, 300.5f); glEnd();
    px(100, 200, "smooth loop: the closing segment", 128, 128, 128, -1);
    px(200, 300, "smooth loop: the top", 255, 255, 255, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_LINES); glColor3f(1, 0, 0); glVertex2f(100, 200.5f); glColor3f(0, 0, 1); glVertex2f(300, 200.5f); glEnd();
    px(200, 200, "smooth line: smooth shading", 128, 0, 128, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glShadeModel(GL_FLAT);
    glBegin(GL_LINES); glColor3f(1, 0, 0); glVertex2f(100, 200.5f); glColor3f(0, 0, 1); glVertex2f(300, 200.5f); glEnd();
    glShadeModel(GL_SMOOTH);
    px(120, 200, "smooth line: flat, the last vertex", 0, 0, 255, 255);
    glColor4f(1, 1, 1, 1);
    /* stipple */
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_LINE_STIPPLE); glLineStipple(1, 0x00FF);
    glBegin(GL_LINES); glVertex2f(100.5f, 200.5f); glVertex2f(300.5f, 200.5f); glEnd();
    glDisable(GL_LINE_STIPPLE);
    px(104, 200, "smooth stipple: on", 255, 255, 255, 255);
    px(112, 200, "smooth stipple: off", 0, 0, 0, 255);
    /* clipped: where the view cuts it, there is no end to be half covered */
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_LINES); glVertex2f(-100, 200.5f); glVertex2f(100, 200.5f); glEnd();
    px(0, 200, "smooth line: cut by the view", 255, 255, 255, 255);
    /* arrays */
    glClear(GL_COLOR_BUFFER_BIT);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 0, pts);
    glDrawArrays(GL_LINES, 0, 2);
    glDisableClientState(GL_VERTEX_ARRAY);
    px(200, 300, "smooth line: from an array", 255, 255, 255, 255);
    px(300, 300, "smooth line: from an array, its end", 128, 128, 128, -1);
    /* its corners are triangles, and they are not culled */
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_CULL_FACE); glCullFace(GL_FRONT_AND_BACK);
    glBegin(GL_LINES); glVertex2f(100.5f, 200.5f); glVertex2f(300.5f, 200.5f); glVertex2f(300.5f, 220.5f); glVertex2f(100.5f, 220.5f); glEnd();
    glDisable(GL_CULL_FACE); glCullFace(GL_BACK);
    px(200, 200, "smooth line: not culled, one way", 255, 255, 255, 255);
    px(200, 220, "smooth line: not culled, the other way", 255, 255, 255, 255);
    /* under a strong perspective: the ends are where they are on the screen, the far one ten times as far */
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glFrustum(-1, 1, -1, 1, 1, 100); glMatrixMode(GL_MODELVIEW);
    glBegin(GL_LINES);
    glVertex3f((2.0f * 100.5f / W - 1.0f) * 1.25f, 1.25f / H, -1.25f);
    glVertex3f((2.0f * 500.5f / W - 1.0f) * 12.5f, 12.5f / H, -12.5f);
    glEnd();
    ortho();
    px(300, H / 2, "smooth line under a perspective: the middle", 255, 255, 255, 255);
    px(300, H / 2 + 1, "smooth line under a perspective: its width", 0, 0, 0, 255);
    px(100, H / 2, "smooth line under a perspective: the near end, half", 128, 128, 128, -1);
    px(500, H / 2, "smooth line under a perspective: the far end, half", 128, 128, 128, -1);
    px(499, H / 2, "smooth line under a perspective: before the far end", 255, 255, 255, 255);
    px(501, H / 2, "smooth line under a perspective: past the far end", 0, 0, 0, 255);
    /* polygon edges are lines */
    glClear(GL_COLOR_BUFFER_BIT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_TRIANGLES); glVertex2f(100, 100); glVertex2f(300, 100); glVertex2f(100, 300); glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    px(200, 100, "smooth line: a polygon's edge, half", 128, 128, 128, -1);
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_BLEND);
    GL_CLEAN("smooth");
}

/* a window larger than the hardware: everything the application says is in window pixels */
static void test_wide(HWND hwnd, int vw, int vh)
{
    RECT r;
    uint8_t *rgba;
    float *z;
    GLuint t;
    GLint n, vp[4];
    int x = vw * 3 / 4, y = vh * 3 / 4, i;
    r.left = r.top = 0; r.right = vw; r.bottom = vh;
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    SetWindowPos(hwnd, NULL, 0, 0, r.right - r.left, r.bottom - r.top, SWP_NOMOVE | SWP_NOZORDER);
    GetClientRect(hwnd, &r);
    val("a wide window: its width", r.right, vw, 0); val("a wide window: its height", r.bottom, vh, 0);
    SwapBuffers(g_dc);
    glViewport(0, 0, vw, vh);
    plain();
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, vw, 0, vh, -1, 1); glMatrixMode(GL_MODELVIEW);
    glGetIntegerv(GL_VIEWPORT, vp);
    val("a wide window: the viewport as set", vp[2], vw, 0);
    glColor3f(0, 1, 0);
    quad((float)x, (float)y, (float)vw, (float)vh, 0);
    px(x + 4, y + 4, "a wide window: a quad out of the old size", 0, 255, 0, 255);
    px(vw - 1, vh - 1, "a wide window: to the corner", 0, 255, 0, 255);
    px(x - 4, y - 4, "a wide window: not before it", 0, 0, 0, 255);
    glEnable(GL_SCISSOR_TEST);
    glScissor(vw / 4, vh / 4, 40, 40);
    glClearColor(1, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);
    px(vw / 4 + 20, vh / 4 + 20, "a wide window: a scissored clear", 255, 0, 0, 255);
    px(vw / 4 + 60, vh / 4 + 20, "a wide window: a scissored clear, outside", 0, 0, 0, 255);
    glColor3f(0, 0, 1);
    glScissor(vw / 2, vh / 4, 40, 40);
    quad(0, 0, (float)vw, (float)vh, 0);
    glDisable(GL_SCISSOR_TEST);
    px(vw / 2 + 20, vh / 4 + 20, "a wide window: a scissored draw", 0, 0, 255, 255);
    px(vw / 2 + 60, vh / 4 + 20, "a wide window: a scissored draw, outside", 0, 0, 0, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glViewport(vw / 2, 0, vw / 2, vh / 2);
    glColor3f(1, 1, 0);
    quad(-(float)vw, -(float)vh, 2.0f * vw, 2.0f * vh, 0);
    glViewport(0, 0, vw, vh);
    px(x, vh / 4, "a wide window: a viewport", 255, 255, 0, 255);
    px(vw / 4, vh / 4, "a wide window: left of the viewport", 0, 0, 0, 255);
    px(x, y, "a wide window: above the viewport", 0, 0, 0, 255);
    /* pixels */
    rgba = (uint8_t *)malloc(16 * 16 * 4);
    for (i = 0; i < 16 * 16 * 4; i += 4) { rgba[i] = 255; rgba[i + 1] = 0; rgba[i + 2] = 255; rgba[i + 3] = 255; }
    glRasterPos2i(x, y / 2);
    glDrawPixels(16, 16, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    px(x + 8, y / 2 + 8, "a wide window: glDrawPixels", 255, 0, 255, 255);
    px(x + 20, y / 2 + 8, "a wide window: glDrawPixels, past it", 255, 255, 0, 255);
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    wrap(GL_TEXTURE_2D, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE, GL_NEAREST);
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, x, y / 2, 16, 16, 0);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    val("a wide window: glCopyTexImage2D, red", rgba[8 * 16 * 4 + 32], 255, 2);
    val("a wide window: glCopyTexImage2D, green", rgba[8 * 16 * 4 + 33], 0, 2);
    glEnable(GL_TEXTURE_2D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    quad(100, 100, 200, 200, 0);
    glDisable(GL_TEXTURE_2D);
    px(150, 150, "a wide window: and drawn", 255, 0, 255, 255);
    glBindTexture(GL_TEXTURE_2D, 0);
    glDeleteTextures(1, &t);
    /* a block read in window pixels */
    rgba = (uint8_t *)realloc(rgba, 40 * 40 * 4);
    glReadPixels(x - 20, y / 2 - 20, 40, 40, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    val("a wide window: a block read, inside", rgba[(28 * 40 + 28) * 4 + 1], 0, 2);
    val("a wide window: a block read, past it", rgba[(28 * 40 + 39) * 4 + 1], 255, 2);
    free(rgba);
    /* depth and stencil in window pixels too */
    z = (float *)malloc(16 * 16 * 4);
    for (i = 0; i < 16 * 16; i++) z[i] = 0.25f;
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_ALWAYS);
    glRasterPos2i(x, y / 2);
    glDrawPixels(16, 16, GL_DEPTH_COMPONENT, GL_FLOAT, z);
    free(z);
    glDepthFunc(GL_LESS); glDisable(GL_DEPTH_TEST);
    val("a wide window: depth pixels", depth_at(x + 8, y / 2 + 8), 0.25, 1e-5);
    val("a wide window: depth pixels, past them", depth_at(x + 24, y / 2 + 8), 1, 1e-5);
    /* widths: 10 window pixels are 10 of the window's, however few of the hardware's */
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(1, 1, 1);
    glLineWidth(20);
    glBegin(GL_LINES); glVertex2f(0, vh / 2 + 0.5f); glVertex2f((float)vw, vh / 2 + 0.5f); glEnd();
    glLineWidth(1);
    px(vw / 2, vh / 2 + 7, "a wide window: a line's width, inside", 255, 255, 255, 255);
    px(vw / 2, vh / 2 + 12, "a wide window: a line's width, outside", 0, 0, 0, 255);
    glPointSize(20);
    glBegin(GL_POINTS); glVertex2f(vw / 4 + 0.5f, vh / 4 + 0.5f); glEnd();
    glPointSize(1);
    px(vw / 4 + 7, vh / 4, "a wide window: a point's size, inside", 255, 255, 255, 255);
    px(vw / 4 + 12, vh / 4, "a wide window: a point's size, outside", 0, 0, 0, 255);
    glGetIntegerv(GL_MAX_VIEWPORT_DIMS, &n);
    GL_CLEAN("a wide window");
    SwapBuffers(g_dc);
}

static void window_back(HWND hwnd)
{
    RECT r = { 0, 0, W, H };
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    SetWindowPos(hwnd, NULL, 0, 0, r.right - r.left, r.bottom - r.top, SWP_NOMOVE | SWP_NOZORDER);
    SwapBuffers(g_dc);
    glViewport(0, 0, W, H);
    plain();
    px(10, 10, "the window back at its size", 0, 0, 0, 255);
}

static void chain_tex(GLenum filter)
{
    static const uint8_t r4[4 * 16] = { 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255 };
    static const uint8_t g2[4 * 4] = { 0,255,0,255, 0,255,0,255, 0,255,0,255, 0,255,0,255 };
    static const uint8_t b1[4] = { 0, 0, 255, 255 };
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, r4);
    glTexImage2D(GL_TEXTURE_2D, 1, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, g2);
    glTexImage2D(GL_TEXTURE_2D, 2, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, b1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, (GLint)filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
}

/* a 4 x 4 texture on 16 x 16 pixels is magnified, level 0; on 4 x 4 pixels of a 64 x 64 window quad: quad(0, 0, 400, 400) with s, t up to 400 is 100 texels... the test only needs a known level */
static void lod_scene(const char *what, float size, int r, int g, int b)
{
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_TEXTURE_2D);
    quad(100, 100, 100 + size, 100 + size, 0);
    glDisable(GL_TEXTURE_2D);
    px(100, 100, what, r, g, b, 255);
}

/* what OpenGL 1.2 to 1.4 added */
/* depth textures, GL_ARB_shadow, 3D textures, queries: GL 1.2 to 1.5 where the filter is in the fragment program */
static PFNGLTEXIMAGE3DPROC p_TexImage3D;
static PFNGLTEXSUBIMAGE3DPROC p_TexSubImage3D;
static PFNGLGENQUERIESPROC p_GenQueries;
static PFNGLDELETEQUERIESPROC p_DeleteQueries;
static PFNGLISQUERYPROC p_IsQuery;
static PFNGLBEGINQUERYPROC p_BeginQuery;
static PFNGLENDQUERYPROC p_EndQuery;
static PFNGLGETQUERYIVPROC p_GetQueryiv;
static PFNGLGETQUERYOBJECTUIVPROC p_GetQueryObjectuiv;
static PFNGLGENFRAMEBUFFERSEXTPROC p_GenFramebuffers;
static PFNGLDELETEFRAMEBUFFERSEXTPROC p_DeleteFramebuffers;
static PFNGLBINDFRAMEBUFFEREXTPROC p_BindFramebuffer;
static PFNGLFRAMEBUFFERTEXTURE2DEXTPROC p_FramebufferTexture2D;
static PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC p_CheckFramebufferStatus;
#ifdef glTexImage3D
#undef glTexImage3D
#endif
#define glTexImage3D p_TexImage3D
#define glTexSubImage3D p_TexSubImage3D
#define glGenQueries p_GenQueries
#define glDeleteQueries p_DeleteQueries
#define glIsQuery p_IsQuery
#define glBeginQuery p_BeginQuery
#define glEndQuery p_EndQuery
#define glGetQueryiv p_GetQueryiv
#define glGetQueryObjectuiv p_GetQueryObjectuiv
#define glGenFramebuffersEXT p_GenFramebuffers
#define glDeleteFramebuffersEXT p_DeleteFramebuffers
#define glBindFramebufferEXT p_BindFramebuffer
#define glFramebufferTexture2DEXT p_FramebufferTexture2D
#define glCheckFramebufferStatusEXT p_CheckFramebufferStatus

static int gl15_procs(void)
{
    #define GET(T, n) ((p_##n = (T)wglGetProcAddress("gl" #n)) != NULL)
    int ok = GET(PFNGLTEXIMAGE3DPROC, TexImage3D) & GET(PFNGLTEXSUBIMAGE3DPROC, TexSubImage3D) & GET(PFNGLGENQUERIESPROC, GenQueries) & GET(PFNGLDELETEQUERIESPROC, DeleteQueries) & GET(PFNGLISQUERYPROC, IsQuery)
           & GET(PFNGLBEGINQUERYPROC, BeginQuery) & GET(PFNGLENDQUERYPROC, EndQuery) & GET(PFNGLGETQUERYIVPROC, GetQueryiv) & GET(PFNGLGETQUERYOBJECTUIVPROC, GetQueryObjectuiv);
    #undef GET
    p_GenFramebuffers = (PFNGLGENFRAMEBUFFERSEXTPROC)wglGetProcAddress("glGenFramebuffersEXT");
    p_DeleteFramebuffers = (PFNGLDELETEFRAMEBUFFERSEXTPROC)wglGetProcAddress("glDeleteFramebuffersEXT");
    p_BindFramebuffer = (PFNGLBINDFRAMEBUFFEREXTPROC)wglGetProcAddress("glBindFramebufferEXT");
    p_FramebufferTexture2D = (PFNGLFRAMEBUFFERTEXTURE2DEXTPROC)wglGetProcAddress("glFramebufferTexture2DEXT");
    p_CheckFramebufferStatus = (PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC)wglGetProcAddress("glCheckFramebufferStatusEXT");
    g_checks++;
    if (!ok || !p_GenFramebuffers || !p_DeleteFramebuffers || !p_BindFramebuffer || !p_FramebufferTexture2D || !p_CheckFramebufferStatus) { g_bad++; printf("FAIL an entry point of GL 1.5 is missing\n"); return 0; }
    return 1;
}
static void u24(uint32_t *o, int w, int h, float (*f)(int, int))
{
    int x, y;
    for (y = 0; y < h; y++) for (x = 0; x < w; x++) o[y * w + x] = (uint32_t)(f(x, y) * 16777215.0f + 0.5f);
}
static float z_steps(int x, int y) { (void)y; return (float)x / 8.0f; }

static void shadow_scene(float r)
{
    glBegin(GL_QUADS);
    glTexCoord3f(0, 0, r); glVertex2f(0, 0);
    glTexCoord3f(1, 0, r); glVertex2f(64, 0);
    glTexCoord3f(1, 1, r); glVertex2f(64, 64);
    glTexCoord3f(0, 1, r); glVertex2f(0, 64);
    glEnd();
}

static void test_shadow(void)
{
    GLuint t, fbo;
    uint32_t z[8 * 8], got[8 * 8];
    GLint v = -1;
    int i;
    if (!gl15_procs()) return;
    plain();
    ortho();
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    u24(z, 8, 8, z_steps);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, 8, 8, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, NULL);
    {                                                           /* UNSIGNED_INT: the 24 bits at the top */
        uint32_t up[8 * 8];
        for (i = 0; i < 64; i++) up[i] = z[i] << 8;
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 8, 8, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, up);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, got);
        for (i = 0; i < 8; i++) val("depth texel read back", (double)(got[i] >> 8), z[i], 1.0);
    }
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_DEPTH_SIZE, &v);
    ieq2("GL_TEXTURE_DEPTH_SIZE", v, 24);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glEnable(GL_TEXTURE_2D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    /* the depth itself, as luminance */
    shadow_scene(0.0f);
    glFinish();
    for (i = 0; i < 8; i++) { char m[32]; sprintf(m, "depth as luminance %d", i); px(i * 8 + 4, 32, m, i * 255 / 8, i * 255 / 8, i * 255 / 8, 255); }
    glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_ALPHA);
    glClear(GL_COLOR_BUFFER_BIT);
    shadow_scene(0.0f);
    glFinish();
    px(5 * 8 + 4, 32, "depth as alpha", 255, 255, 255, 5 * 255 / 8);
    glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY);
    glClear(GL_COLOR_BUFFER_BIT);
    shadow_scene(0.0f);
    glFinish();
    px(5 * 8 + 4, 32, "depth as intensity", 5 * 255 / 8, 5 * 255 / 8, 5 * 255 / 8, 5 * 255 / 8);
    glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE);
    /* R to texture: every function at r = 0.5, against 0.25 and 0.75 */
    {
        static const struct { GLenum f; int low, high; } F[8] = {
            { GL_LEQUAL, 0, 255 }, { GL_GEQUAL, 255, 0 }, { GL_LESS, 0, 255 }, { GL_GREATER, 255, 0 }, { GL_EQUAL, 0, 0 }, { GL_NOTEQUAL, 255, 255 }, { GL_ALWAYS, 255, 255 }, { GL_NEVER, 0, 0 } };
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
        for (i = 0; i < 8; i++) {
            char m[32];
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, (GLint)F[i].f);
            glClear(GL_COLOR_BUFFER_BIT);
            shadow_scene(0.5f);
            glFinish();
            sprintf(m, "compare %d, depth 0.25", i);
            px(2 * 8 + 4, 32, m, F[i].low, F[i].low, F[i].low, 255);
            sprintf(m, "compare %d, depth 0.75", i);
            px(6 * 8 + 4, 32, m, F[i].high, F[i].high, F[i].high, 255);
        }
    }
    /* filtered: the mean of the two comparisons either side of the step */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glClear(GL_COLOR_BUFFER_BIT);
    shadow_scene(0.3f);                                         /* texels at 2/8 fail, at 3/8 pass: the boundary is at x = 24 */
    glFinish();
    px(23, 32, "PCF near the step, fail side", 112, 112, 112, 255);
    px(24, 32, "PCF near the step, pass side", 143, 143, 143, 255);
    px(10, 32, "PCF well inside the fail side", 0, 0, 0, 255);
    px(40, 32, "PCF well inside the pass side", 255, 255, 255, 255);
    /* the depth of a real render, in a depth-only framebuffer, sampled afterwards */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, 64, 64, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, NULL);
    glGenFramebuffersEXT(1, &fbo);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, t, 0);
    ieq2("depth-only framebuffer complete", (int)glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT), GL_FRAMEBUFFER_COMPLETE_EXT);
    glDrawBuffer(GL_NONE);
    glViewport(0, 0, 64, 64);
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glClearDepth(1.0);
    glClear(GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, 64, 0, 64, 0, -1); glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    glBegin(GL_QUADS);                                          /* z 0.25 on the left half: window depth 0.25 */
    glVertex3f(0, 0, 0.25f); glVertex3f(32, 0, 0.25f); glVertex3f(32, 64, 0.25f); glVertex3f(0, 64, 0.25f);
    glEnd();
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
    glDrawBuffer(GL_BACK);
    glDisable(GL_DEPTH_TEST);
    glViewport(0, 0, W, H);
    ortho();
    glEnable(GL_TEXTURE_2D);
    glClear(GL_COLOR_BUFFER_BIT);
    shadow_scene(0.0f);
    glFinish();
    px(16, 32, "rendered depth sampled", 64, 64, 64, 255);
    px(48, 32, "cleared depth sampled", 255, 255, 255, 255);
    glDisable(GL_TEXTURE_2D);
    glDeleteFramebuffersEXT(1, &fbo);
    glDeleteTextures(1, &t);
    plain();
}

static void tex3d_set(GLuint t, GLint mag, GLint min, GLint wrap)
{
    glBindTexture(GL_TEXTURE_3D, t);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, mag);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, min);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, wrap);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, wrap);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, wrap);
}

static void tex3d_scene(float r)
{
    glBegin(GL_QUADS);
    glTexCoord3f(0, 0, r); glVertex2f(0, 0);
    glTexCoord3f(1, 0, r); glVertex2f(64, 0);
    glTexCoord3f(1, 1, r); glVertex2f(64, 64);
    glTexCoord3f(0, 1, r); glVertex2f(0, 64);
    glEnd();
    glFinish();
}

static void test_tex3d(void)
{
    GLuint t;
    uint8_t px4[4 * 4 * 4 * 4], back[4 * 4 * 4 * 4];
    GLint v = -1;
    int x, y, z, ok = 1;
    plain();
    ortho();
    glClearColor(0, 0, 0, 1);
    glGetIntegerv(GL_MAX_3D_TEXTURE_SIZE, &v);
    ieq2("GL_MAX_3D_TEXTURE_SIZE", v, 128);
    glTexImage3D(GL_PROXY_TEXTURE_3D, 0, GL_RGBA8, 64, 32, 16, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glGetTexLevelParameteriv(GL_PROXY_TEXTURE_3D, 0, GL_TEXTURE_DEPTH, &v);
    ieq2("a proxy that fits", v, 16);
    glTexImage3D(GL_PROXY_TEXTURE_3D, 0, GL_RGBA8, 256, 256, 256, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glGetTexLevelParameteriv(GL_PROXY_TEXTURE_3D, 0, GL_TEXTURE_WIDTH, &v);
    ieq2("a proxy that does not", v, 0);
    /* red = x, green = y, blue = z */
    for (z = 0; z < 4; z++) for (y = 0; y < 4; y++) for (x = 0; x < 4; x++) {
        uint8_t *p = px4 + ((z * 4 + y) * 4 + x) * 4;
        p[0] = (uint8_t)(x * 85); p[1] = (uint8_t)(y * 85); p[2] = (uint8_t)(z * 85); p[3] = 255;
    }
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_3D, t);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, 4, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, px4);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glGetTexLevelParameteriv(GL_TEXTURE_3D, 0, GL_TEXTURE_DEPTH, &v);
    ieq2("GL_TEXTURE_DEPTH", v, 4);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glGetTexImage(GL_TEXTURE_3D, 0, GL_RGBA, GL_UNSIGNED_BYTE, back);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    for (x = 0; x < (int)sizeof back; x++) if (back[x] != px4[x]) ok = 0;
    ieq2("3D texels read back", ok, 1);
    glEnable(GL_TEXTURE_3D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    tex3d_set(t, GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE);
    for (z = 0; z < 4; z++) {
        char m[32];
        tex3d_scene((z + 0.5f) / 4.0f);
        sprintf(m, "3D nearest, slice %d", z);
        px(8, 8, m, 0, 0, z * 85, 255);
        px(56, 8, m, 255, 0, z * 85, 255);
        px(8, 56, m, 0, 255, z * 85, 255);
        px(40, 24, m, 170, 85, z * 85, 255);
    }
    /* linear in r: half way between slices 1 and 2 */
    tex3d_set(t, GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE);
    tex3d_scene(0.5f);
    px(3, 3, "3D linear in r", 0, 0, 128, 255);
    px(32, 32, "3D linear in s, t, r", 130, 130, 128, 255);
    /* wrap in r: r = 1.125 is slice 0 again under REPEAT, the last slice clamped or mirrored, the border past it */
    tex3d_set(t, GL_NEAREST, GL_NEAREST, GL_REPEAT);
    tex3d_scene(1.125f);
    px(8, 8, "3D GL_REPEAT in r", 0, 0, 0, 255);
    tex3d_set(t, GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE);
    tex3d_scene(1.125f);
    px(8, 8, "3D GL_CLAMP_TO_EDGE in r", 0, 0, 255, 255);
    tex3d_set(t, GL_NEAREST, GL_NEAREST, GL_MIRRORED_REPEAT);
    tex3d_scene(1.125f);
    px(8, 8, "3D GL_MIRRORED_REPEAT in r", 0, 0, 255, 255);
    {
        static const GLfloat brd[4] = { 0.2f, 0.4f, 0.6f, 1.0f };
        glTexParameterfv(GL_TEXTURE_3D, GL_TEXTURE_BORDER_COLOR, brd);
        tex3d_set(t, GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_BORDER);
        tex3d_scene(1.125f);
        px(8, 8, "3D GL_CLAMP_TO_BORDER in r", 51, 102, 153, 255);
    }
    /* a sub-image, then the mip chain: the mean of 8 texels, and the level the size of the quad asks for */
    {
        static const uint8_t white[4] = { 255, 255, 255, 255 };
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexSubImage3D(GL_TEXTURE_3D, 0, 1, 2, 3, 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, white);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        tex3d_set(t, GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE);
        tex3d_scene(0.875f);
        px(24, 40, "3D sub-image", 255, 255, 255, 255);
        glTexSubImage3D(GL_TEXTURE_3D, 0, 1, 2, 3, 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, px4 + ((3 * 4 + 2) * 4 + 1) * 4);
    }
    glTexParameteri(GL_TEXTURE_3D, GL_GENERATE_MIPMAP, GL_TRUE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, 4, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, px4);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    tex3d_set(t, GL_NEAREST, GL_NEAREST_MIPMAP_NEAREST, GL_CLAMP_TO_EDGE);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_QUADS);                                          /* 4 texels over 2 pixels: level 1, whose texel 0 is the mean of 0 and 85 */
    glTexCoord3f(0, 0, 0.1f); glVertex2f(0, 0);
    glTexCoord3f(1, 0, 0.1f); glVertex2f(2, 0);
    glTexCoord3f(1, 1, 0.1f); glVertex2f(2, 2);
    glTexCoord3f(0, 1, 0.1f); glVertex2f(0, 2);
    glEnd();
    glFinish();
    px(0, 0, "3D level 1", 43, 43, 43, 255);
    glDisable(GL_TEXTURE_3D);
    glDeleteTextures(1, &t);
    plain();
}

static void test_queries(void)
{
    GLuint q[2], r = 77;
    GLint v = -1;
    plain();
    ortho();
    glGenQueries(2, q);
    glGetQueryiv(GL_SAMPLES_PASSED, GL_QUERY_COUNTER_BITS, &v);
    ieq2("GL_QUERY_COUNTER_BITS", v, 0);
    ieq2("a name is not a query before it is used", glIsQuery(q[0]), GL_FALSE);
    glBeginQuery(GL_SAMPLES_PASSED, q[0]);
    ieq2("a query is one after it", glIsQuery(q[0]), GL_TRUE);
    glGetQueryiv(GL_SAMPLES_PASSED, GL_CURRENT_QUERY, &v);
    ieq2("GL_CURRENT_QUERY", v, (GLint)q[0]);
    glBeginQuery(GL_SAMPLES_PASSED, q[1]);
    ieq2("a second query at once", (int)glGetError(), GL_INVALID_OPERATION);
    glGetQueryObjectuiv(q[0], GL_QUERY_RESULT, &r);
    ieq2("the result of a running query", (int)glGetError(), GL_INVALID_OPERATION);
    quad(0, 0, 8, 8, 0);
    glEndQuery(GL_SAMPLES_PASSED);
    glBeginQuery(GL_SAMPLES_PASSED, q[1]);
    glEndQuery(GL_SAMPLES_PASSED);
    glGetQueryObjectuiv(q[0], GL_QUERY_RESULT_AVAILABLE, &r);
    ieq2("always available", (int)r, GL_TRUE);
    glGetQueryObjectuiv(q[0], GL_QUERY_RESULT, &r);
    ieq2("a query with a draw in it", r != 0, 1);
    glGetQueryObjectuiv(q[1], GL_QUERY_RESULT, &r);
    ieq2("a query with none", (int)r, 0);
    glDeleteQueries(2, q);
    ieq2("deleted", glIsQuery(q[0]), GL_FALSE);
}

static void test_gl14(void)
{
    PFNGLMULTIDRAWARRAYSPROC multi = (PFNGLMULTIDRAWARRAYSPROC)wglGetProcAddress("glMultiDrawArrays");
    PFNGLMULTIDRAWELEMENTSPROC multie = (PFNGLMULTIDRAWELEMENTSPROC)wglGetProcAddress("glMultiDrawElementsEXT");
    PFNGLWINDOWPOS2IPROC wpos = (PFNGLWINDOWPOS2IPROC)wglGetProcAddress("glWindowPos2i");
    PFNGLWINDOWPOS3FPROC wpos3 = (PFNGLWINDOWPOS3FPROC)wglGetProcAddress("glWindowPos3fARB");
    PFNGLCOMPRESSEDTEXIMAGE2DPROC cimg = (PFNGLCOMPRESSEDTEXIMAGE2DPROC)wglGetProcAddress("glCompressedTexImage2D");
    PFNGLCOMPRESSEDTEXSUBIMAGE2DPROC csub = (PFNGLCOMPRESSEDTEXSUBIMAGE2DPROC)wglGetProcAddress("glCompressedTexSubImage2DARB");
    PFNGLGETCOMPRESSEDTEXIMAGEPROC cget = (PFNGLGETCOMPRESSEDTEXIMAGEPROC)wglGetProcAddress("glGetCompressedTexImage");
    PFNGLSAMPLECOVERAGEPROC scov = (PFNGLSAMPLECOVERAGEPROC)wglGetProcAddress("glSampleCoverage");
    PFNGLPOINTPARAMETERFPROC ptf = (PFNGLPOINTPARAMETERFPROC)wglGetProcAddress("glPointParameterf");
    PFNGLPOINTPARAMETERFVPROC ptfv = (PFNGLPOINTPARAMETERFVPROC)wglGetProcAddress("glPointParameterfv");
    PFNGLFOGCOORDFPROC fogc = (PFNGLFOGCOORDFPROC)wglGetProcAddress("glFogCoordf");
    PFNGLFOGCOORDPOINTERPROC fogp = (PFNGLFOGCOORDPOINTERPROC)wglGetProcAddress("glFogCoordPointer");
    PFNGLSECONDARYCOLOR3FPROC sec = (PFNGLSECONDARYCOLOR3FPROC)wglGetProcAddress("glSecondaryColor3f");
    PFNGLSECONDARYCOLORPOINTERPROC secp = (PFNGLSECONDARYCOLORPOINTERPROC)wglGetProcAddress("glSecondaryColorPointer");
    GLuint t;
    GLint v = 0;
    float f;
    plain();
    ortho();
    g_checks++;
    if (!multi || !multie || !wpos || !wpos3 || !cimg || !csub || !cget || !scov || !ptf || !ptfv || !fogc || !fogp || !sec || !secp) { g_bad++; printf("FAIL an entry point of GL 1.4 is missing\n"); return; }
    /* multi-draw */
    {
        static const GLfloat q[] = { 10, 10, 30, 10, 10, 30, 30, 30,  50, 10, 70, 10, 50, 30, 70, 30 };
        static const GLushort i0[] = { 0, 1, 2, 3 }, i1[] = { 4, 5, 6, 7 };
        const void *ix[2] = { i0, i1 };
        GLint first[3] = { 0, 0, 4 };
        GLsizei count[3] = { 4, 0, 4 }, c2[2] = { 4, 4 };
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT);
        glColor3f(1, 0, 0);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(2, GL_FLOAT, 0, q);
        multi(GL_TRIANGLE_STRIP, first, count, 3);
        px(20, 20, "glMultiDrawArrays, first", 255, 0, 0, 255);
        px(60, 20, "glMultiDrawArrays, third", 255, 0, 0, 255);
        px(40, 20, "glMultiDrawArrays, between", 0, 0, 0, 255);
        glClear(GL_COLOR_BUFFER_BIT);
        glColor3f(0, 1, 0);
        multie(GL_TRIANGLE_STRIP, c2, GL_UNSIGNED_SHORT, ix, 2);
        px(60, 20, "glMultiDrawElements", 0, 255, 0, 255);
        glDisableClientState(GL_VERTEX_ARRAY);
    }
    /* window positions: no matrix between the numbers and the pixels */
    {
        static const uint8_t c[4] = { 10, 200, 30, 255 };
        glMatrixMode(GL_MODELVIEW); glPushMatrix(); glTranslatef(100, 100, 0);
        glClear(GL_COLOR_BUFFER_BIT);
        wpos(50, 60);
        glDrawPixels(1, 1, GL_RGBA, GL_UNSIGNED_BYTE, c);
        px(50, 60, "glWindowPos2i, glDrawPixels", 10, 200, 30, 255);
        glPopMatrix();
        wpos3(7.0f, 8.0f, 0.25f);
        glGetFloatv(GL_CURRENT_RASTER_POSITION, &f);
        val("glWindowPos3f x", f, 7.0, 0.01);
        {
            GLfloat rp[4];
            glGetFloatv(GL_CURRENT_RASTER_POSITION, rp);
            val("glWindowPos3f z", rp[2], 0.25, 0.001);
        }
        glGetIntegerv(GL_CURRENT_RASTER_POSITION_VALID, &v);
        ieq2("glWindowPos is valid", v, 1);
    }
    /* multisample state, of no effect with SAMPLE_BUFFERS 0 */
    glGetIntegerv(GL_SAMPLE_BUFFERS, &v);
    ieq2("GL_SAMPLE_BUFFERS", v, 0);
    scov(0.5f, GL_TRUE);
    glGetFloatv(GL_SAMPLE_COVERAGE_VALUE, &f);
    val("glSampleCoverage value", f, 0.5, 0.001);
    glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    ieq2("GL_SAMPLE_ALPHA_TO_COVERAGE enable", glIsEnabled(GL_SAMPLE_ALPHA_TO_COVERAGE), 1);
    glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    scov(1.0f, GL_FALSE);
    /* compressed textures */
    {
        static const uint8_t red[8] = { 0x00, 0xF8, 0, 0, 0, 0, 0, 0 }, blue[8] = { 0x1F, 0x00, 0, 0, 0, 0, 0, 0 };
        uint8_t data[32], back[32];
        int i;
        for (i = 0; i < 4; i++) memcpy(data + 8 * i, red, 8);
        glGenTextures(1, &t);
        glBindTexture(GL_TEXTURE_2D, t);
        cimg(GL_TEXTURE_2D, 0, GL_COMPRESSED_RGB_S3TC_DXT1_EXT, 8, 8, 0, 32, data);
        GL_CLEAN("glCompressedTexImage2D");
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        lod_scene("a DXT1 texture", 100, 255, 0, 0);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_COMPRESSED, &v);
        ieq2("GL_TEXTURE_COMPRESSED", v, 1);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_COMPRESSED_IMAGE_SIZE, &v);
        ieq2("GL_TEXTURE_COMPRESSED_IMAGE_SIZE", v, 32);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_INTERNAL_FORMAT, &v);
        ieq2("GL_TEXTURE_INTERNAL_FORMAT", v, GL_COMPRESSED_RGB_S3TC_DXT1_EXT);
        csub(GL_TEXTURE_2D, 0, 0, 0, 4, 4, GL_COMPRESSED_RGB_S3TC_DXT1_EXT, 8, blue);
        GL_CLEAN("glCompressedTexSubImage2D");
        lod_scene("a block replaced", 100, 0, 0, 255);
        px(190, 190, "the next block stays", 255, 0, 0, 255);
        cget(GL_TEXTURE_2D, 0, back);
        ieq2("glGetCompressedTexImage: the block written", memcmp(back, blue, 8), 0);
        ieq2("glGetCompressedTexImage: the next", memcmp(back + 8, red, 8), 0);
        csub(GL_TEXTURE_2D, 0, 1, 0, 4, 4, GL_COMPRESSED_RGB_S3TC_DXT1_EXT, 8, blue);
        ieq2("a region off the block grid", glGetError(), GL_INVALID_OPERATION);
        cimg(GL_TEXTURE_2D, 0, GL_COMPRESSED_RGB_S3TC_DXT1_EXT, 8, 8, 0, 31, data);
        ieq2("the wrong size", glGetError(), GL_INVALID_VALUE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, red);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_COMPRESSED, &v);
        ieq2("glTexImage2D ends it", v, 0);
        cget(GL_TEXTURE_2D, 0, back);
        ieq2("glGetCompressedTexImage of an uncompressed level", glGetError(), GL_INVALID_OPERATION);
        glDeleteTextures(1, &t);
    }
    /* secondary colour, unlit */
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(0.5f, 0, 0);
    sec(0, 0.25f, 0);
    glEnable(GL_COLOR_SUM);
    quad(10, 10, 30, 30, 0);
    px(20, 20, "glSecondaryColor3f adds", 128, 64, 0, 255);
    glDisable(GL_COLOR_SUM);
    quad(10, 10, 30, 30, 0);
    px(20, 20, "without GL_COLOR_SUM it does not", 128, 0, 0, 255);
    /* fog coordinates */
    glClear(GL_COLOR_BUFFER_BIT);
    {
        static const float fc[4] = { 0, 0, 1, 1 };
        glEnable(GL_FOG);
        glFogi(GL_FOG_MODE, GL_LINEAR);
        glFogf(GL_FOG_START, 0.0f); glFogf(GL_FOG_END, 1.0f);
        glFogfv(GL_FOG_COLOR, fc);
        glFogi(GL_FOG_COORD_SRC, GL_FOG_COORD);
        glGetIntegerv(GL_FOG_COORD_SRC, &v);
        ieq2("GL_FOG_COORD_SRC", v, GL_FOG_COORD);
        glColor3f(1, 0, 0);
        fogc(0.25f);
        quad(10, 10, 30, 30, 0);
        px(20, 20, "glFogCoordf: a quarter fog", 191, 0, 64, 255);
        {
            static const GLfloat q[] = { 40, 10, 60, 10, 40, 30, 60, 30 }, sc[] = { 0.75f, 0.75f, 0.75f, 0.75f };
            glEnableClientState(GL_VERTEX_ARRAY); glVertexPointer(2, GL_FLOAT, 0, q);
            glEnableClientState(GL_FOG_COORD_ARRAY); fogp(GL_FLOAT, 0, sc);
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            glDisableClientState(GL_FOG_COORD_ARRAY); glDisableClientState(GL_VERTEX_ARRAY);
            px(50, 20, "a fog coordinate array", 64, 0, 191, 255);
        }
        {
            GLuint l = glGenLists(1);
            glNewList(l, GL_COMPILE);
            glBegin(GL_QUADS);
            fogc(0.5f);
            glVertex2f(70, 10); glVertex2f(90, 10); glVertex2f(90, 30); glVertex2f(70, 30);
            glEnd();
            glEndList();
            fogc(0.0f);
            glCallList(l);
            glDeleteLists(l, 1);
            px(80, 20, "a fog coordinate in a display list", 128, 0, 128, 255);
        }
        glFogi(GL_FOG_COORD_SRC, GL_FRAGMENT_DEPTH);
        glDisable(GL_FOG);
    }
    /* a secondary colour array */
    glClear(GL_COLOR_BUFFER_BIT);
    {
        static const GLfloat q[] = { 10, 10, 30, 10, 10, 30, 30, 30 }, sc[] = { 0, 0, 0.5f, 0, 0, 0.5f, 0, 0, 0.5f, 0, 0, 0.5f };
        glColor3f(0.25f, 0, 0);
        glEnable(GL_COLOR_SUM);
        glEnableClientState(GL_VERTEX_ARRAY); glVertexPointer(2, GL_FLOAT, 0, q);
        glEnableClientState(GL_SECONDARY_COLOR_ARRAY); secp(3, GL_FLOAT, 0, sc);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glDisableClientState(GL_SECONDARY_COLOR_ARRAY); glDisableClientState(GL_VERTEX_ARRAY);
        glDisable(GL_COLOR_SUM);
        px(20, 20, "a secondary colour array", 64, 0, 128, 255);
    }
    /* point parameters: size / sqrt(a + b d + c d^2) at eye distance d */
    {
        static const float at[3] = { 0, 0, 1 }, one[3] = { 1, 0, 0 };
        glClear(GL_COLOR_BUFFER_BIT);
        glColor3f(1, 1, 1);
        glPointSize(16.0f);
        ptfv(GL_POINT_DISTANCE_ATTENUATION, at);
        glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(-100.5, W - 100.5, -100.5, H - 100.5, 0, 10);      /* the eye over the pixel: d is the length of the eye position, not its z */
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        glBegin(GL_POINTS); glVertex3f(0.0f, 0.0f, -2.0f); glEnd();         /* 16 / 2 = 8 pixels */
        px(100, 100, "an attenuated point: its middle", 255, 255, 255, 255);
        px(103, 100, "inside 8 pixels", 255, 255, 255, 255);
        px(106, 100, "outside 8, inside 16", 0, 0, 0, 255);
        glClear(GL_COLOR_BUFFER_BIT);
        ptf(GL_POINT_SIZE_MIN, 12.0f);
        glBegin(GL_POINTS); glVertex3f(0.0f, 0.0f, -2.0f); glEnd();
        px(105, 100, "GL_POINT_SIZE_MIN lifts it to 12", 255, 255, 255, 255);
        px(108, 100, "and no further", 0, 0, 0, 255);
        ptf(GL_POINT_SIZE_MIN, 0.0f);
        ptfv(GL_POINT_DISTANCE_ATTENUATION, one);
        glPointSize(1.0f);
        ortho();
    }
    /* the level: BASE_LEVEL, MAX_LEVEL, MIN_LOD, MAX_LOD, LOD_BIAS on red / green / blue levels */
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    chain_tex(GL_NEAREST_MIPMAP_NEAREST);
    lod_scene("level 0 magnified", 100, 255, 0, 0);
    lod_scene("level 1 at a texel of 2 pixels... minified by 2", 2, 0, 255, 0);
    lod_scene("level 2 minified by 4", 1, 0, 0, 255);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 1);
    lod_scene("GL_TEXTURE_BASE_LEVEL 1", 100, 0, 255, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 2);
    lod_scene("GL_TEXTURE_BASE_LEVEL 2", 100, 0, 0, 255);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    lod_scene("GL_TEXTURE_MAX_LEVEL 0 keeps the top level", 1, 255, 0, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 1);
    lod_scene("GL_TEXTURE_MAX_LEVEL 1", 1, 0, 255, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 1000);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_LOD, 2.0f);
    lod_scene("GL_TEXTURE_MIN_LOD 2", 100, 0, 0, 255);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_LOD, -1000.0f);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_LOD, 1.0f);
    lod_scene("GL_TEXTURE_MAX_LOD 1", 1, 0, 255, 0);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_LOD, 1000.0f);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_LOD_BIAS, 2.0f);
    lod_scene("GL_TEXTURE_LOD_BIAS 2", 4, 0, 0, 255);
    glGetTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_LOD_BIAS, &f);
    val("GL_TEXTURE_LOD_BIAS back", f, 2.0, 0.001);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_LOD_BIAS, 0.0f);
    glTexEnvf(GL_TEXTURE_FILTER_CONTROL, GL_TEXTURE_LOD_BIAS, 1.0f);
    lod_scene("the unit's GL_TEXTURE_LOD_BIAS", 4, 0, 255, 0);
    glTexEnvf(GL_TEXTURE_FILTER_CONTROL, GL_TEXTURE_LOD_BIAS, 0.0f);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 5);
    lod_scene("a base level not defined: as if disabled", 100, 255, 255, 255);
    glDeleteTextures(1, &t);
    /* the client attribute stack */
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPushClientAttrib(GL_CLIENT_PIXEL_STORE_BIT);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 8);
    glGetIntegerv(GL_UNPACK_ALIGNMENT, &v);
    ieq2("inside the push", v, 8);
    glPopClientAttrib();
    glGetIntegerv(GL_UNPACK_ALIGNMENT, &v);
    ieq2("glPopClientAttrib: pixel store", v, 1);
    {
        static const GLfloat q[2] = { 0, 0 };
        GLvoid *pt = NULL;
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT);
        glEnableClientState(GL_COLOR_ARRAY);
        glVertexPointer(2, GL_FLOAT, 0, q);
        glPopClientAttrib();
        ieq2("glPopClientAttrib: an enable", glIsEnabled(GL_COLOR_ARRAY), 0);
        glGetPointerv(GL_VERTEX_ARRAY_POINTER, &pt);
        ieq2("glPopClientAttrib: a pointer", pt != (GLvoid *)q, 1);
    }
    GL_CLEAN("the GL 1.4 scenes");
    plain();
}

#define SCENE(call) do { printf("%s\n", #call); call; } while (0)

/* OpenGL 2.0: shaders through the entry points a program gets from wglGetProcAddress */
#define P20(X) \
    X(PFNGLCREATESHADERPROC, CreateShader) X(PFNGLSHADERSOURCEPROC, ShaderSource) X(PFNGLCOMPILESHADERPROC, CompileShader) X(PFNGLGETSHADERIVPROC, GetShaderiv) X(PFNGLGETSHADERINFOLOGPROC, GetShaderInfoLog) \
    X(PFNGLCREATEPROGRAMPROC, CreateProgram) X(PFNGLATTACHSHADERPROC, AttachShader) X(PFNGLDETACHSHADERPROC, DetachShader) X(PFNGLLINKPROGRAMPROC, LinkProgram) X(PFNGLGETPROGRAMIVPROC, GetProgramiv) X(PFNGLGETPROGRAMINFOLOGPROC, GetProgramInfoLog) \
    X(PFNGLUSEPROGRAMPROC, UseProgram) X(PFNGLDELETEPROGRAMPROC, DeleteProgram) X(PFNGLDELETESHADERPROC, DeleteShader) X(PFNGLISPROGRAMPROC, IsProgram) X(PFNGLISSHADERPROC, IsShader) X(PFNGLVALIDATEPROGRAMPROC, ValidateProgram) \
    X(PFNGLGETUNIFORMLOCATIONPROC, GetUniformLocation) X(PFNGLUNIFORM1FPROC, Uniform1f) X(PFNGLUNIFORM2FPROC, Uniform2f) X(PFNGLUNIFORM4FPROC, Uniform4f) X(PFNGLUNIFORM1IPROC, Uniform1i) X(PFNGLUNIFORM2IPROC, Uniform2i) X(PFNGLUNIFORM4FVPROC, Uniform4fv) X(PFNGLUNIFORM1FVPROC, Uniform1fv) X(PFNGLUNIFORM1IVPROC, Uniform1iv) \
    X(PFNGLUNIFORMMATRIX2FVPROC, UniformMatrix2fv) X(PFNGLUNIFORMMATRIX4FVPROC, UniformMatrix4fv) X(PFNGLGETUNIFORMFVPROC, GetUniformfv) X(PFNGLGETUNIFORMIVPROC, GetUniformiv) X(PFNGLGETACTIVEUNIFORMPROC, GetActiveUniform) X(PFNGLGETACTIVEATTRIBPROC, GetActiveAttrib) \
    X(PFNGLGETATTRIBLOCATIONPROC, GetAttribLocation) X(PFNGLBINDATTRIBLOCATIONPROC, BindAttribLocation) X(PFNGLVERTEXATTRIB4FPROC, VertexAttrib4f) X(PFNGLVERTEXATTRIB2FPROC, VertexAttrib2f) X(PFNGLVERTEXATTRIB4NUBPROC, VertexAttrib4Nub) X(PFNGLVERTEXATTRIBPOINTERPROC, VertexAttribPointer) \
    X(PFNGLENABLEVERTEXATTRIBARRAYPROC, EnableVertexAttribArray) X(PFNGLDISABLEVERTEXATTRIBARRAYPROC, DisableVertexAttribArray) X(PFNGLGETVERTEXATTRIBFVPROC, GetVertexAttribfv) X(PFNGLDRAWBUFFERSPROC, DrawBuffers) X(PFNGLACTIVETEXTUREPROC, ActiveTexture) X(PFNGLMULTITEXCOORD2FPROC, MultiTexCoord2f)
#define X(T, n) static T p_##n;
P20(X)
#undef X
#define glCreateShader p_CreateShader
#define glShaderSource p_ShaderSource
#define glCompileShader p_CompileShader
#define glGetShaderiv p_GetShaderiv
#define glGetShaderInfoLog p_GetShaderInfoLog
#define glCreateProgram p_CreateProgram
#define glAttachShader p_AttachShader
#define glDetachShader p_DetachShader
#define glLinkProgram p_LinkProgram
#define glGetProgramiv p_GetProgramiv
#define glGetProgramInfoLog p_GetProgramInfoLog
#define glUseProgram p_UseProgram
#define glDeleteProgram p_DeleteProgram
#define glDeleteShader p_DeleteShader
#define glIsProgram p_IsProgram
#define glIsShader p_IsShader
#define glValidateProgram p_ValidateProgram
#define glGetUniformLocation p_GetUniformLocation
#define glUniform1f p_Uniform1f
#define glUniform2f p_Uniform2f
#define glUniform4f p_Uniform4f
#define glUniform1i p_Uniform1i
#define glUniform2i p_Uniform2i
#define glUniform4fv p_Uniform4fv
#define glUniform1fv p_Uniform1fv
#define glUniform1iv p_Uniform1iv
#define glUniformMatrix2fv p_UniformMatrix2fv
#define glUniformMatrix4fv p_UniformMatrix4fv
#define glGetUniformfv p_GetUniformfv
#define glGetUniformiv p_GetUniformiv
#define glGetActiveUniform p_GetActiveUniform
#define glGetActiveAttrib p_GetActiveAttrib
#define glGetAttribLocation p_GetAttribLocation
#define glBindAttribLocation p_BindAttribLocation
#define glVertexAttrib4f p_VertexAttrib4f
#define glVertexAttrib2f p_VertexAttrib2f
#define glVertexAttrib4Nub p_VertexAttrib4Nub
#define glVertexAttribPointer p_VertexAttribPointer
#define glEnableVertexAttribArray p_EnableVertexAttribArray
#define glDisableVertexAttribArray p_DisableVertexAttribArray
#define glGetVertexAttribfv p_GetVertexAttribfv
#define glDrawBuffers p_DrawBuffers
#ifndef glActiveTexture
#define glActiveTexture p_ActiveTexture
#endif

static GLuint sh(GLenum type, const char *src)
{
    GLuint s = glCreateShader(type);
    GLint ok = 0;
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    g_checks++;
    if (!ok) { char log[512] = ""; g_bad++; glGetShaderInfoLog(s, sizeof log, NULL, log); printf("FAIL a shader did not compile: %s\n%s\n", log, src); }
    return s;
}

/* either stage may be NULL */
static GLuint prog2(const char *vs, const char *fs)
{
    GLuint p = glCreateProgram(), a = 0, b = 0;
    GLint ok = 0;
    if (vs) { a = sh(GL_VERTEX_SHADER, vs); glAttachShader(p, a); }
    if (fs) { b = sh(GL_FRAGMENT_SHADER, fs); glAttachShader(p, b); }
    glLinkProgram(p);
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    g_checks++;
    if (!ok) { char log[512] = ""; g_bad++; glGetProgramInfoLog(p, sizeof log, NULL, log); printf("FAIL a program did not link: %s\n", log); }
    if (a) glDeleteShader(a);
    if (b) glDeleteShader(b);
    glUseProgram(p);
    return p;
}

#define PLAIN_VS "void main() { gl_Position = ftransform(); gl_FrontColor = gl_Color; gl_TexCoord[0] = gl_MultiTexCoord0; }\n"
#define COLOR_FS "void main() { gl_FragColor = gl_Color; }\n"

static void q64(void) { quad(0, 0, 64, 64, 0); }
static void done(GLuint p) { glUseProgram(0); glDeleteProgram(p); GL_CLEAN("a GLSL scene"); plain(); }

static uint32_t tex2d(int w, int h, const uint8_t *px, GLint filter, GLint wrap)
{
    GLuint t;
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, px);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
    return t;
}

/* the language and the fixed pipeline's state */
static void test_glsl_basic(void)
{
    GLuint p;
    GLint v = -1;
    const char *lang = (const char *)glGetString(GL_SHADING_LANGUAGE_VERSION);
    plain();
    ieq2("GL_SHADING_LANGUAGE_VERSION is there", lang != NULL && !strncmp(lang, "1.10", 4), 1);
    glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &v); ieq2("GL_MAX_VERTEX_ATTRIBS", v, 16);
    glGetIntegerv(GL_MAX_VARYING_FLOATS, &v); ieq2("GL_MAX_VARYING_FLOATS", v, 32);
    glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &v); ieq2("GL_MAX_TEXTURE_IMAGE_UNITS", v, 8);
    glGetIntegerv(GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS, &v); ieq2("GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS", v, 0);
    glGetIntegerv(GL_MAX_DRAW_BUFFERS, &v); ieq2("GL_MAX_DRAW_BUFFERS", v, 1);
    ieq2("GL_VERSION", !strncmp((const char *)glGetString(GL_VERSION), "2.0", 3), 1);
    glGetIntegerv(GL_STENCIL_BACK_FUNC, &v); ieq2("GL_STENCIL_BACK_FUNC", v, GL_ALWAYS);
    glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &v); ieq2("GL_BLEND_EQUATION_ALPHA", v, GL_FUNC_ADD);
    {   /* §2.0's textures have no power of two to be: repeat and a chain */
        static const uint8_t c3[12] = { 255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255 };
        GLuint t = tex2d(3, 1, c3, GL_NEAREST, GL_REPEAT);
        glEnable(GL_TEXTURE_2D);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glBegin(GL_QUADS); glTexCoord2f(0, 0); glVertex2f(0, 0); glTexCoord2f(2, 0); glVertex2f(60, 0); glTexCoord2f(2, 1); glVertex2f(60, 10); glTexCoord2f(0, 1); glVertex2f(0, 10); glEnd();
        px(15, 5, "a texture of 3 texels, the middle one", 0, 255, 0, 255);
        px(45, 5, "repeated", 0, 255, 0, 255);
        glDisable(GL_TEXTURE_2D);
        glDeleteTextures(1, &t);
        glClear(GL_COLOR_BUFFER_BIT);
    }
    /* the plainest program there is: matrices and colour from the pipeline */
    p = prog2(PLAIN_VS, COLOR_FS);
    glGetIntegerv(GL_CURRENT_PROGRAM, &v); ieq2("GL_CURRENT_PROGRAM", v, (int)p);
    glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE); glEnable(GL_POINT_SPRITE);
    glPopAttrib();
    ieq2("glPopAttrib: GL_VERTEX_PROGRAM_POINT_SIZE", glIsEnabled(GL_VERTEX_PROGRAM_POINT_SIZE), 0);
    ieq2("glPopAttrib: GL_POINT_SPRITE", glIsEnabled(GL_POINT_SPRITE), 0);
    ieq2("glIsProgram", glIsProgram(p), GL_TRUE);
    glColor3f(1.0f, 0.5f, 0.25f);
    q64();
    px(32, 32, "ftransform, gl_Color", 255, 128, 64, 255);
    glMatrixMode(GL_MODELVIEW); glTranslatef(100, 0, 0);
    glColor3f(0, 1, 0);
    q64();
    px(132, 32, "the modelview reaches it", 0, 255, 0, 255);
    glLoadIdentity();
    done(p);
    /* a #version line, comments with built-in names, a macro over one */
    p = prog2("#version 110\n// gl_Vertex in a comment\n#define POS gl_Vertex\nvoid main() { /* gl_Color */ gl_Position = gl_ModelViewProjectionMatrix * POS; }\n", "#version 110\nvoid main() { gl_FragColor = vec4(0.25, 0.5, 0.75, 1.0); }\n");
    q64();
    px(32, 32, "#version 110", 64, 128, 191, 255);
    done(p);
    /* uniforms of every shape */
    p = prog2(PLAIN_VS, "uniform vec4 c; uniform float k[3]; uniform int i; uniform bool b; uniform mat2 m; uniform ivec2 w;\nvoid main() { vec2 r = m * vec2(1.0, 0.0); gl_FragColor = vec4(c.x * k[2], b ? r.y : 0.0, float(i) * 0.125, float(w.y) * 0.25); }\n");
    {
        static const GLfloat ks[3] = { 0, 0, 0.5f }, mt[4] = { 0, 0.75f, 0, 0 };
        GLfloat g[4] = { 0, 0, 0, 0 };
        GLint gi = 0;
        glUniform4f(glGetUniformLocation(p, "c"), 1, 0, 0, 0);
        glUniform1fv(glGetUniformLocation(p, "k"), 3, ks);
        glUniform1i(glGetUniformLocation(p, "i"), 4);
        glUniform1i(glGetUniformLocation(p, "b"), 1);
        glUniformMatrix2fv(glGetUniformLocation(p, "m"), 1, GL_FALSE, mt);
        glUniform2i(glGetUniformLocation(p, "w"), 0, 3);
        q64();
        px(32, 32, "uniforms: vec4, an array, int, bool, mat2, ivec2", 128, 191, 128, 191);
        glGetUniformfv(p, glGetUniformLocation(p, "k[2]"), g); val("glGetUniformfv of an element", g[0], 0.5, 0.0);
        glGetUniformiv(p, glGetUniformLocation(p, "i"), &gi); ieq2("glGetUniformiv", gi, 4);
        ieq2("no location for a name that is not there", glGetUniformLocation(p, "nothing"), -1);
        ieq2("no location for gl_", glGetUniformLocation(p, "gl_ModelViewMatrix"), -1);
        glUniform1f(glGetUniformLocation(p, "i"), 1.0f);
        ieq2("glUniform1f on an int", (int)glGetError(), GL_INVALID_OPERATION);
        glGetProgramiv(p, GL_ACTIVE_UNIFORMS, &v); ieq2("GL_ACTIVE_UNIFORMS: the six and what ftransform reads", v, 7);
    }
    done(p);
    /* a matrix uniform moves the vertex as the pipeline would */
    p = prog2("uniform mat4 mvp;\nvoid main() { gl_Position = mvp * gl_Vertex; gl_FrontColor = vec4(1.0, 0.0, 0.0, 1.0); }\n", COLOR_FS);
    {
        GLfloat m[16];
        glMatrixMode(GL_MODELVIEW); glPushMatrix(); glTranslatef(200, 100, 0);
        glGetFloatv(GL_MODELVIEW_MATRIX, m);
        glPopMatrix();
        {
            GLfloat pr[16], o[16];
            int r, c, k;
            glGetFloatv(GL_PROJECTION_MATRIX, pr);
            for (c = 0; c < 4; c++) for (r = 0; r < 4; r++) { float s = 0; for (k = 0; k < 4; k++) s += pr[k * 4 + r] * m[c * 4 + k]; o[c * 4 + r] = s; }
            glUniformMatrix4fv(glGetUniformLocation(p, "mvp"), 1, GL_FALSE, o);
        }
        q64();
        px(232, 132, "a mat4 uniform", 255, 0, 0, 255);
        px(32, 32, "and not where it was", 0, 0, 0, 255);
    }
    done(p);
    /* state as uniforms */
    glEnable(GL_LIGHT0);
    {
        static const GLfloat dif[4] = { 0.25f, 0.5f, 0.75f, 1.0f }, md[4] = { 1.0f, 0.5f, 1.0f, 1.0f }, fc[4] = { 0.5f, 0.0f, 0.25f, 1.0f };
        glLightfv(GL_LIGHT0, GL_DIFFUSE, dif);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, md);
        glFogfv(GL_FOG_COLOR, fc);
        glFogf(GL_FOG_DENSITY, 0.5f);
        p = prog2("void main() { gl_Position = ftransform(); gl_FrontColor = gl_FrontLightProduct[0].diffuse; gl_BackColor = gl_LightSource[0].diffuse * gl_FrontMaterial.diffuse; }\n", COLOR_FS);
        q64();
        px(32, 32, "gl_FrontLightProduct in a vertex shader", 64, 64, 191, 255);
        done(p);
        p = prog2(PLAIN_VS, "void main() { gl_FragColor = gl_LightSource[0].diffuse * gl_FrontMaterial.diffuse + gl_Fog.color * gl_Fog.density; }\n");
        q64();
        px(32, 32, "the light, the material and the fog in a fragment shader", 128, 64, 223, 255);
        done(p);
        glDisable(GL_LIGHT0);
    }
    /* a vertex shader that does the whole lighting from state, and the pipeline: the same picture */
    {
        static const GLfloat pos[4] = { 0.0f, 0.0f, 1.0f, 0.0f }, dif[4] = { 0.8f, 0.6f, 0.4f, 1.0f }, md[4] = { 1, 1, 1, 1 }, zero[4] = { 0, 0, 0, 1 };
        uint8_t a[4], b[4];
        glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); glOrtho(0, W, 0, H, -100, 100); glMatrixMode(GL_MODELVIEW);
        glEnable(GL_LIGHT0); glEnable(GL_LIGHTING);
        glLightfv(GL_LIGHT0, GL_POSITION, pos); glLightfv(GL_LIGHT0, GL_DIFFUSE, dif);
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, zero);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, md); glMaterialfv(GL_FRONT, GL_AMBIENT, zero);
        glNormal3f(0.6f, 0.0f, 0.8f);
        q64();
        glFinish();
        glReadPixels(32, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, a);
        glClear(GL_COLOR_BUFFER_BIT);
        glDisable(GL_LIGHTING);
        p = prog2("void main() { vec3 n = gl_NormalMatrix * gl_Normal; vec3 l = normalize(gl_LightSource[0].position.xyz); float d = max(dot(n, l), 0.0);\n"
                  "  gl_FrontColor = vec4(gl_FrontLightProduct[0].diffuse.rgb * d, gl_FrontMaterial.diffuse.a); gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; }\n", COLOR_FS);
        q64();
        glFinish();
        glReadPixels(32, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, b);
        val("the shader's light against the pipeline's, red", b[0], a[0], 2);
        val("green", b[1], a[1], 2);
        val("blue", b[2], a[2], 2);
        glNormal3f(0, 0, 1);
        done(p);
        glDisable(GL_LIGHT0);
        glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW);
    }
    /* a compile error is a log, not a crash; a link error too */
    {
        static const char *bad = "void main() { gl_Position = nothing; }\n";
        GLuint s = glCreateShader(GL_VERTEX_SHADER);
        GLint st = 1;
        char log[256] = "";
        glShaderSource(s, 1, &bad, NULL);
        glCompileShader(s);
        glGetShaderiv(s, GL_COMPILE_STATUS, &st);
        ieq2("an undeclared name does not compile", st, 0);
        glGetShaderInfoLog(s, sizeof log, NULL, log);
        ieq2("and the log says something", log[0] != 0, 1);
        p = glCreateProgram();
        glAttachShader(p, s);
        glLinkProgram(p);
        glGetProgramiv(p, GL_LINK_STATUS, &st);
        ieq2("a program of it does not link", st, 0);
        glUseProgram(p);
        ieq2("nor can it be used", (int)glGetError(), GL_INVALID_OPERATION);
        glDeleteProgram(p);
        glDeleteShader(s);
        glUseProgram(77777);
        ieq2("a name that is no program", (int)glGetError(), GL_INVALID_VALUE);
    }
    /* several shaders of a stage */
    {
        GLuint a = sh(GL_FRAGMENT_SHADER, "vec4 shade();\nvoid main() { gl_FragColor = shade(); }\n"), b = sh(GL_FRAGMENT_SHADER, "vec4 shade() { return vec4(0.5, 0.25, 1.0, 1.0); }\n"), c = sh(GL_VERTEX_SHADER, PLAIN_VS);
        GLint st = 0;
        p = glCreateProgram();
        glAttachShader(p, a); glAttachShader(p, b); glAttachShader(p, c);
        glLinkProgram(p);
        glGetProgramiv(p, GL_LINK_STATUS, &st);
        ieq2("two fragment shaders link", st, 1);
        glUseProgram(p);
        q64();
        px(32, 32, "a function from the other shader", 128, 64, 255, 255);
        done(p);
        glDeleteShader(a); glDeleteShader(b); glDeleteShader(c);
    }
    GL_CLEAN("GLSL");
}

/* attributes: generic, current, by array, inside glBegin */
static void test_glsl_attribs(void)
{
    GLuint p;
    GLint l;
    float g[4];
    static const GLfloat verts[8] = { 0, 0, 64, 0, 0, 64, 64, 64 }, cols[16] = { 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1 };
    static const GLubyte ub[16] = { 0, 128, 0, 255, 0, 128, 0, 255, 0, 128, 0, 255, 0, 128, 0, 255 };
    plain();
    p = prog2("attribute vec2 pos; attribute vec4 col; varying vec4 c;\nuniform mat4 m;\nvoid main() { gl_Position = gl_ModelViewProjectionMatrix * vec4(pos, 0.0, 1.0); c = col; }\n", "varying vec4 c;\nvoid main() { gl_FragColor = c; }\n");
    ieq2("the first attribute of a program without gl_Vertex can be generic 0", glGetAttribLocation(p, "pos") == 0 || glGetAttribLocation(p, "col") == 0, 1);
    l = glGetAttribLocation(p, "col");
    glVertexAttrib4f(l, 0.25f, 0.5f, 0.75f, 1.0f);
    glGetVertexAttribfv(l, GL_CURRENT_VERTEX_ATTRIB, g);
    val("GL_CURRENT_VERTEX_ATTRIB", g[1], 0.5, 0);
    glEnableVertexAttribArray(glGetAttribLocation(p, "pos"));
    glVertexAttribPointer(glGetAttribLocation(p, "pos"), 2, GL_FLOAT, GL_FALSE, 0, verts);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    px(32, 32, "a generic array and a current attribute", 64, 128, 191, 255);
    glEnableVertexAttribArray(l);
    glVertexAttribPointer(l, 4, GL_FLOAT, GL_FALSE, 0, cols);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    px(32, 32, "two generic arrays", 255, 0, 0, 255);
    glVertexAttribPointer(l, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, ub);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    px(32, 32, "a normalized one", 0, 128, 0, 255);
    glDisableVertexAttribArray(l);
    glDisableVertexAttribArray(glGetAttribLocation(p, "pos"));
    done(p);
    /* glBindAttribLocation, inside glBegin, generic 0 makes the vertex */
    {
        GLuint a = sh(GL_VERTEX_SHADER, "attribute vec4 at; attribute vec4 p0;\nvarying vec4 c;\nvoid main() { gl_Position = gl_ModelViewProjectionMatrix * p0; c = at; }\n"), b = sh(GL_FRAGMENT_SHADER, "varying vec4 c;\nvoid main() { gl_FragColor = c; }\n");
        p = glCreateProgram();
        glAttachShader(p, a); glAttachShader(p, b);
        glBindAttribLocation(p, 0, "p0");
        glBindAttribLocation(p, 5, "at");
        glLinkProgram(p);
        ieq2("glBindAttribLocation", glGetAttribLocation(p, "at"), 5);
        glUseProgram(p);
        glBegin(GL_QUADS);
        glVertexAttrib4f(5, 1, 0, 0, 1); glVertexAttrib2f(0, 0, 0);
        glVertexAttrib4f(5, 1, 0, 0, 1); glVertexAttrib2f(0, 32, 0);
        glVertexAttrib4f(5, 1, 0, 0, 1); glVertexAttrib2f(0, 32, 64);
        glVertexAttrib4f(5, 1, 0, 0, 1); glVertexAttrib2f(0, 0, 64);
        glVertexAttrib4f(5, 0, 1, 0, 1); glVertexAttrib2f(0, 32, 0);
        glVertexAttrib4f(5, 0, 1, 0, 1); glVertexAttrib2f(0, 64, 0);
        glVertexAttrib4f(5, 0, 0, 1, 1); glVertexAttrib2f(0, 64, 64);
        glVertexAttrib4f(5, 0, 0, 1, 1); glVertexAttrib2f(0, 32, 64);
        glEnd();
        px(16, 32, "generic attributes inside glBegin", 255, 0, 0, 255);
        px(48, 1, "per vertex", 0, 250, 5, 255);
        px(48, 62, "per vertex, the other end", 0, 5, 250, 255);
        done(p);
        glDeleteShader(a); glDeleteShader(b);
    }
    /* a matrix attribute takes four locations */
    p = prog2("attribute mat4 m;\nvoid main() { gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; gl_FrontColor = vec4(m[0].x, m[1].y, m[3].z, 1.0); }\n", COLOR_FS);
    l = glGetAttribLocation(p, "m");
    ieq2("gl_Vertex keeps generic 0 from an attribute", l > 0, 1);
    glVertexAttrib4f(l, 0.25f, 0, 0, 0); glVertexAttrib4f(l + 1, 0, 0.5f, 0, 0); glVertexAttrib4f(l + 3, 0, 0, 0.75f, 0);
    q64();
    px(32, 32, "a matrix attribute", 64, 128, 191, 255);
    done(p);
    /* the arrays of the pipeline under a shader */
    p = prog2(PLAIN_VS, "void main() { gl_FragColor = gl_TexCoord[0] + gl_Color; }\n");
    {
        static const GLfloat tc[8] = { 0.5f, 0, 0.5f, 0, 0.5f, 0, 0.5f, 0 }, cc[12] = { 0, 0.25f, 0, 0, 0.25f, 0, 0, 0.25f, 0, 0, 0.25f, 0 };
        glEnableClientState(GL_VERTEX_ARRAY); glEnableClientState(GL_TEXTURE_COORD_ARRAY); glEnableClientState(GL_COLOR_ARRAY);
        glVertexPointer(2, GL_FLOAT, 0, verts); glTexCoordPointer(2, GL_FLOAT, 0, tc); glColorPointer(3, GL_FLOAT, 0, cc);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        px(32, 32, "the pipeline's arrays", 128, 64, 0, 255);
        glDisableClientState(GL_VERTEX_ARRAY); glDisableClientState(GL_TEXTURE_COORD_ARRAY); glDisableClientState(GL_COLOR_ARRAY);
    }
    done(p);
    GL_CLEAN("attributes");
}

/* samplers: which unit, which kind, and what the TMU has not got */
static void test_glsl_tex(void)
{
    static const uint8_t two[8] = { 255, 0, 0, 255, 0, 255, 0, 255 }, blue[4] = { 0, 0, 255, 255 };
    GLuint p, a, b, t3;
    plain();
    a = tex2d(2, 1, two, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glActiveTexture(GL_TEXTURE1);
    b = tex2d(1, 1, blue, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glActiveTexture(GL_TEXTURE0);
    p = prog2(PLAIN_VS, "uniform sampler2D s, u;\nvoid main() { gl_FragColor = texture2D(s, gl_TexCoord[0].st) + texture2D(u, gl_TexCoord[0].st); }\n");
    glUniform1i(glGetUniformLocation(p, "s"), 0);
    glUniform1i(glGetUniformLocation(p, "u"), 1);
    q64();
    px(16, 32, "two samplers on two units", 255, 0, 255, 255);
    glUniform1i(glGetUniformLocation(p, "s"), 1);
    q64();
    px(16, 32, "the unit is a uniform", 0, 0, 255, 255);
    done(p);
    /* no glEnable(GL_TEXTURE_2D) needed: a shader reads what is bound */
    p = prog2(PLAIN_VS, "uniform sampler2D s;\nvec4 f(sampler2D q, vec2 c) { return texture2D(q, c); }\nvoid main() { gl_FragColor = f(s, gl_TexCoord[0].st); }\n");
    q64();
    px(48, 32, "a sampler through a function", 0, 255, 0, 255);
    done(p);
    /* incomplete: (0, 0, 0, 1) */
    {
        GLuint in;
        glActiveTexture(GL_TEXTURE2);
        glGenTextures(1, &in);
        glBindTexture(GL_TEXTURE_2D, in);
        glActiveTexture(GL_TEXTURE0);
        p = prog2(PLAIN_VS, "uniform sampler2D s;\nvoid main() { gl_FragColor = texture2D(s, gl_TexCoord[0].st) + vec4(0.5, 0.0, 0.0, 0.0); }\n");
        glUniform1i(glGetUniformLocation(p, "s"), 2);
        q64();
        px(16, 32, "an incomplete texture reads (0, 0, 0, 1)", 128, 0, 0, 255);
        done(p);
        p = prog2(PLAIN_VS, "uniform sampler2D s;\nvec4 f(sampler2D q, vec2 c) { return texture2D(q, c); }\nvoid main() { gl_FragColor = f(s, gl_TexCoord[0].st) + vec4(0.5, 0.0, 0.0, 0.0); }\n");
        glUniform1i(glGetUniformLocation(p, "s"), 2);
        q64();
        px(16, 32, "an incomplete texture through a function", 128, 0, 0, 255);
        done(p);
        glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, 0); glActiveTexture(GL_TEXTURE0);
        glDeleteTextures(1, &in);
    }
    /* sampler1D */
    {
        GLuint t;
        glGenTextures(1, &t);
        glBindTexture(GL_TEXTURE_1D, t);
        glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, two);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        p = prog2(PLAIN_VS, "uniform sampler1D s;\nvoid main() { gl_FragColor = texture1D(s, gl_TexCoord[0].s); }\n");
        q64();
        px(16, 32, "sampler1D, left", 255, 0, 0, 255);
        px(48, 32, "sampler1D, right", 0, 255, 0, 255);
        done(p);
        glDeleteTextures(1, &t);
    }
    /* a border, which the TMU has not got: the program filters */
    {
        static const GLfloat brd[4] = { 0.0f, 0.0f, 1.0f, 1.0f };
        glBindTexture(GL_TEXTURE_2D, a);
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, brd);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        p = prog2("void main() { gl_Position = ftransform(); gl_TexCoord[0] = gl_MultiTexCoord0 * 2.0 - 0.5; }\n", "uniform sampler2D s;\nvoid main() { gl_FragColor = texture2D(s, gl_TexCoord[0].st); }\n");
        q64();
        px(4, 32, "GL_CLAMP_TO_BORDER under a shader: outside", 0, 0, 255, 255);
        px(24, 32, "inside, left texel", 255, 0, 0, 255);
        px(40, 32, "inside, right texel", 0, 255, 0, 255);
        done(p);
        p = prog2("void main() { gl_Position = ftransform(); gl_TexCoord[0] = gl_MultiTexCoord0 * 2.0 - 0.5; }\n",
                  "uniform sampler2D s;\nvec4 f(sampler2D q, vec2 c) { return texture2D(q, c); }\nvec4 g(sampler2D q, vec2 c) { return f(q, c); }\nvoid main() { gl_FragColor = g(s, gl_TexCoord[0].st); }\n");
        q64();
        px(4, 32, "GL_CLAMP_TO_BORDER through two functions: outside", 0, 0, 255, 255);
        px(24, 32, "through two functions: inside, left texel", 255, 0, 0, 255);
        done(p);
        /* §3.8.14: a shadow function on a texture that is not a depth texture is undefined, but the program builds */
        p = prog2(PLAIN_VS, "uniform sampler2DShadow s;\nvoid main() { gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0) + 0.0 * shadow2D(s, vec3(gl_TexCoord[0].st, 0.5)); }\n");
        glUniform1i(glGetUniformLocation(p, "s"), 1);
        q64();
        px(32, 32, "shadow2D on a colour texture draws", 255, 0, 0, 255);
        done(p);
    }
    /* sampler3D and a shadow sampler go by the program's own filter */
    {
        uint8_t v[2 * 2 * 2 * 4];
        int x, y, z;
        for (z = 0; z < 2; z++) for (y = 0; y < 2; y++) for (x = 0; x < 2; x++) { uint8_t *o = v + ((z * 2 + y) * 2 + x) * 4; o[0] = (uint8_t)(x * 255); o[1] = (uint8_t)(y * 255); o[2] = (uint8_t)(z * 255); o[3] = 255; }
        glGenTextures(1, &t3);
        glBindTexture(GL_TEXTURE_3D, t3);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        p_TexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, 2, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, v);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        tex3d_set(t3, GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE);
        p = prog2(PLAIN_VS, "uniform sampler3D s; uniform float r;\nvoid main() { gl_FragColor = texture3D(s, vec3(gl_TexCoord[0].st, r)); }\n");
        glUniform1f(glGetUniformLocation(p, "r"), 0.75f);
        q64();
        px(48, 16, "sampler3D", 255, 0, 255, 255);
        tex3d_set(t3, GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE);
        glUniform1f(glGetUniformLocation(p, "r"), 0.5f);
        q64();
        px(60, 2, "a state change is a new variant: linear in r", 255, 0, 128, 255);
        done(p);
        glDeleteTextures(1, &t3);
    }
    {
        uint32_t z[4] = { 0x40000000u, 0x40000000u, 0xC0000000u, 0xC0000000u };
        GLuint d;
        glGenTextures(1, &d);
        glBindTexture(GL_TEXTURE_2D, d);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, 2, 2, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, z);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
        p = prog2(PLAIN_VS, "uniform sampler2DShadow s;\nvoid main() { gl_FragColor = shadow2D(s, vec3(gl_TexCoord[0].st, 0.5)); }\n");
        q64();
        px(32, 16, "shadow2D: nearer than the reference", 0, 0, 0, 255);
        px(32, 48, "shadow2D: farther", 255, 255, 255, 255);
        done(p);
        p = prog2(PLAIN_VS, "uniform sampler2DShadow s;\nvec4 lit(sampler2DShadow m, vec3 c) { return shadow2D(m, c); }\nvoid main() { gl_FragColor = lit(s, vec3(gl_TexCoord[0].st, 0.5)); }\n");
        q64();
        px(32, 16, "shadow2D through a function: nearer", 0, 0, 0, 255);
        px(32, 48, "shadow2D through a function: farther", 255, 255, 255, 255);
        done(p);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
        p = prog2(PLAIN_VS, "uniform sampler2D s;\nvoid main() { gl_FragColor = texture2D(s, gl_TexCoord[0].st); }\n");
        q64();
        px(32, 48, "a depth texture on a sampler2D", 191, 191, 191, 255);
        done(p);
        /* an array: an element the program filters, the others the TMU reads; unit 1 holds blue */
        {
            static const GLint u01[2] = { 0, 1 }, u03[2] = { 0, 3 };
            GLuint in;
            p = prog2(PLAIN_VS, "uniform sampler2D a[2];\nvec4 rd(sampler2D q, vec2 c) { return texture2D(q, c); }\nvec4 hlf(sampler2D q, vec2 c) { return rd(q, c) * 0.5; }\n"
                                "void main() { vec4 r = vec4(0.0); for (int i = 0; i < 2; i++) r += hlf(a[i], gl_TexCoord[0].st); gl_FragColor = r; }\n");
            glUniform1iv(glGetUniformLocation(p, "a"), 2, u01);
            q64();
            px(32, 16, "a depth texture in an array, by a loop's index, through two functions: 0.25", 32, 32, 159, 255);
            px(32, 48, "the same: 0.75", 96, 96, 223, 255);
            done(p);
            p = prog2(PLAIN_VS, "uniform sampler2D a[2];\nvoid main() { gl_FragColor = texture2D(a[0], gl_TexCoord[0].st) * 0.5 + texture2D(a[1], gl_TexCoord[0].st) * 0.5; }\n");
            glUniform1iv(glGetUniformLocation(p, "a"), 2, u01);
            q64();
            px(32, 48, "a depth texture in an array, a number for an index", 96, 96, 223, 255);
            glActiveTexture(GL_TEXTURE3); glGenTextures(1, &in); glBindTexture(GL_TEXTURE_2D, in); glActiveTexture(GL_TEXTURE0);
            glUniform1iv(glGetUniformLocation(p, "a"), 2, u03);
            q64();
            px(32, 48, "an incomplete element of that array reads (0, 0, 0, 1)", 96, 96, 96, 255);
            done(p);
            glActiveTexture(GL_TEXTURE3); glBindTexture(GL_TEXTURE_2D, 0); glActiveTexture(GL_TEXTURE0);
            glDeleteTextures(1, &in);
            glBindTexture(GL_TEXTURE_2D, d);
        }
        glDeleteTextures(1, &d);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    glDeleteTextures(1, &a);
    glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, 0); glActiveTexture(GL_TEXTURE0);
    glDeleteTextures(1, &b);
    GL_CLEAN("samplers");
}

/* one stage of the program, the pipeline for the other; and what the pipeline does around a shader */
static void test_glsl_pipeline(void)
{
    static const uint8_t white[4] = { 255, 255, 255, 255 }, half[4] = { 128, 128, 128, 255 };
    GLuint p, t;
    plain();
    /* a fragment shader on the pipeline's vertices */
    p = prog2(NULL, "uniform sampler2D s;\nvoid main() { gl_FragColor = texture2D(s, gl_TexCoord[0].st) * gl_Color; }\n");
    t = tex2d(1, 1, half, GL_NEAREST, GL_REPEAT);
    glColor3f(1, 0, 0);
    q64();
    px(32, 32, "a fragment shader alone", 128, 0, 0, 255);
    done(p);
    p = prog2(NULL, "void main() { gl_FragColor = vec4(gl_TexCoord[1].st, gl_FogFragCoord * 0.25, 1.0); }\n");
    p_MultiTexCoord2f(GL_TEXTURE1, 0.25f, 0.75f);
    glBegin(GL_QUADS); glVertex3f(0, 0, -0.5f); glVertex3f(64, 0, -0.5f); glVertex3f(64, 64, -0.5f); glVertex3f(0, 64, -0.5f); glEnd();
    px(32, 32, "a generated vertex program feeds the shader", 64, 191, 32, 255);
    p_MultiTexCoord2f(GL_TEXTURE1, 0, 0);
    done(p);
    /* a vertex shader on the pipeline's fragments */
    p = prog2("void main() { gl_Position = ftransform(); gl_FrontColor = gl_Color.bgra; gl_TexCoord[0] = gl_MultiTexCoord0; }\n", NULL);
    glEnable(GL_TEXTURE_2D);
    glColor3f(1, 0.5f, 0);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    q64();
    px(32, 32, "a vertex shader alone, the texture environment after it", 0, 64, 128, 255);
    glDisable(GL_TEXTURE_2D);
    done(p);
    glEnable(GL_FOG); glFogi(GL_FOG_MODE, GL_LINEAR); glFogf(GL_FOG_START, 0); glFogf(GL_FOG_END, 1);
    {
        static const GLfloat fc[4] = { 0, 1, 0, 1 };
        glFogfv(GL_FOG_COLOR, fc);
    }
    p = prog2("void main() { gl_Position = ftransform(); gl_FrontColor = vec4(1.0, 0.0, 0.0, 1.0); gl_FogFragCoord = 0.5; }\n", NULL);
    q64();
    px(32, 32, "fog from gl_FogFragCoord", 128, 128, 0, 255);
    done(p);
    glDisable(GL_FOG);
    glBindTexture(GL_TEXTURE_2D, 0);
    glDeleteTextures(1, &t);
    /* the alpha test, which here is in the program */
    p = prog2(PLAIN_VS, "uniform float a;\nvoid main() { gl_FragColor = vec4(1.0, 1.0, 1.0, a); }\n");
    glEnable(GL_ALPHA_TEST);
    glAlphaFunc(GL_GREATER, 0.5f);
    glUniform1f(glGetUniformLocation(p, "a"), 0.25f);
    q64();
    px(32, 32, "the alpha test stops it", 0, 0, 0, 255);
    glUniform1f(glGetUniformLocation(p, "a"), 0.75f);
    q64();
    px(32, 32, "the alpha test passes it", 255, 255, 255, 191);
    glDisable(GL_ALPHA_TEST);
    done(p);
    /* a clip plane, by gl_ClipVertex */
    {
        static const GLdouble eq[4] = { -1, 0, 0, 32 };
        glClipPlane(GL_CLIP_PLANE0, eq);
        glEnable(GL_CLIP_PLANE0);
        p = prog2("void main() { gl_Position = ftransform(); gl_ClipVertex = gl_ModelViewMatrix * gl_Vertex; gl_FrontColor = vec4(1.0); }\n", COLOR_FS);
        q64();
        px(16, 32, "inside the plane", 255, 255, 255, 255);
        px(48, 32, "outside it", 0, 0, 0, 255);
        glDisable(GL_CLIP_PLANE0);
        done(p);
    }
    /* two-sided colour is the shader's to switch on */
    glEnable(GL_VERTEX_PROGRAM_TWO_SIDE);
    p = prog2("void main() { gl_Position = ftransform(); gl_FrontColor = vec4(1.0, 0.0, 0.0, 1.0); gl_BackColor = vec4(0.0, 1.0, 0.0, 1.0); }\n", COLOR_FS);
    q64();
    px(32, 32, "the front colour", 255, 0, 0, 255);
    glBegin(GL_QUADS); glVertex2f(100, 0); glVertex2f(100, 64); glVertex2f(164, 64); glVertex2f(164, 0); glEnd();
    px(132, 32, "the back colour", 0, 255, 0, 255);
    glDisable(GL_VERTEX_PROGRAM_TWO_SIDE);
    q64();
    done(p);
    /* gl_PointSize, and sprites */
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    p = prog2("void main() { gl_Position = ftransform(); gl_PointSize = 9.0; gl_FrontColor = vec4(1.0); }\n", COLOR_FS);
    glBegin(GL_POINTS); glVertex2f(100.5f, 100.5f); glEnd();
    px(104, 100, "gl_PointSize", 255, 255, 255, 255);
    px(106, 100, "and no more", 0, 0, 0, 255);
    glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_POINTS); glVertex2f(100.5f, 100.5f); glEnd();
    px(100, 100, "glPointSize again", 255, 255, 255, 255);
    px(102, 100, "one pixel", 0, 0, 0, 255);
    done(p);
    glEnable(GL_POINT_SPRITE);
    glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
    glPointSize(16.0f);
    p = prog2(NULL, "void main() { gl_FragColor = vec4(gl_TexCoord[0].st, 0.0, 1.0); }\n");
    glBegin(GL_POINTS); glVertex2f(100.5f, 100.5f); glEnd();
    px(104, 100, "a sprite's coordinate in a shader", 191, 128, 0, 255);
    done(p);
    {
        GLuint w = tex2d(1, 1, white, GL_NEAREST, GL_REPEAT);
        static const uint8_t grad[16] = { 255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255 };
        GLuint g = tex2d(2, 2, grad, GL_NEAREST, GL_CLAMP_TO_EDGE);
        glEnable(GL_TEXTURE_2D);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glBegin(GL_POINTS); glVertex2f(100.5f, 100.5f); glEnd();
        px(96, 96, "a sprite in the pipeline: the top row of the image is at the top by default, the bottom left is image (0, 1)", 0, 0, 255, 255);
        glDisable(GL_TEXTURE_2D);
        glDeleteTextures(1, &w); glDeleteTextures(1, &g);
    }
    glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_FALSE);
    glDisable(GL_POINT_SPRITE);
    glPointSize(1.0f);
    /* one draw buffer */
    {
        GLenum b = GL_BACK;
        glDrawBuffers(1, &b);
        GL_CLEAN("glDrawBuffers of one");
        glDrawBuffers(2, &b);
        ieq2("two draw buffers", (int)glGetError(), GL_INVALID_VALUE);
    }
    /* §3.6: pictures have no vertices, but their fragments go through the fragment shader */
    p = prog2("void main() { gl_Position = vec4(0.0, 0.0, 0.0, 1.0); gl_FrontColor = vec4(0.0); gl_PointSize = 1.0; }\n", "uniform float k;\nvoid main() { gl_FragColor = gl_Color.bgra * k; }\n");
    {
        static const uint8_t r[4] = { 255, 0, 0, 255 };
        static const GLubyte bits[1] = { 0x80 };
        glUniform1f(glGetUniformLocation(p, "k"), 0.5f);
        glUseProgram(0); glRasterPos2f(10, 10); glUseProgram(p);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glDrawPixels(1, 1, GL_RGBA, GL_UNSIGNED_BYTE, r);
        px(10, 10, "glDrawPixels through a fragment shader", 0, 0, 128, 128);
        glUseProgram(0); glColor3f(0, 1, 0); glRasterPos2f(20, 20); glUseProgram(p);
        glBitmap(1, 1, 0, 0, 0, 0, bits);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        px(20, 20, "glBitmap through a fragment shader", 0, 128, 0, 128);
        px(21, 20, "and only where its bits are", 0, 0, 0, 255);
    }
    done(p);
    p = prog2("void main() { gl_Position = ftransform(); gl_FrontColor = vec4(1.0, 0.0, 0.0, 1.0); }\n", NULL);
    {
        static const uint8_t g[4] = { 0, 255, 0, 255 };
        glUseProgram(0); glRasterPos2f(30, 30); glUseProgram(p);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glDrawPixels(1, 1, GL_RGBA, GL_UNSIGNED_BYTE, g);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        px(30, 30, "a vertex shader alone: a picture as it is", 0, 255, 0, 255);
    }
    done(p);
    /* §2.13: the raster position goes through the vertex shader */
    p = prog2("attribute vec4 at;\nvoid main() { gl_Position = gl_ModelViewProjectionMatrix * (gl_Vertex + vec4(100.0, 0.0, 0.0, 0.0)); gl_FrontColor = at; }\n", COLOR_FS);
    {
        GLfloat rp[4] = { 0, 0, 0, 0 }, rc[4] = { 0, 0, 0, 0 };
        glVertexAttrib4f(glGetAttribLocation(p, "at"), 0.25f, 0.5f, 0.75f, 1.0f);
        glRasterPos2f(10, 20);
        glGetFloatv(GL_CURRENT_RASTER_POSITION, rp);
        glGetFloatv(GL_CURRENT_RASTER_COLOR, rc);
        val("glRasterPos through a shader: x", rp[0], 110, 0.01);
        val("y", rp[1], 20, 0.01);
        val("the colour it made", rc[1], 0.5, 0.01);
    }
    done(p);
    /* §2.12: under a vertex shader a user clip plane takes gl_ClipVertex, here the eye position mirrored in x: the raster position, selection */
    p = prog2("void main() { gl_Position = ftransform(); gl_ClipVertex = gl_ModelViewMatrix * gl_Vertex * vec4(-1.0, 1.0, 1.0, 1.0); gl_FrontColor = gl_Color; }\n", COLOR_FS);
    {
        static const GLdouble keep_left[4] = { -1.0, 0.0, 0.0, 30.0 }, keep_right[4] = { 1.0, 0.0, 0.0, -30.0 };
        GLuint sel[16];
        GLint hits;
        glClipPlane(GL_CLIP_PLANE0, keep_left);
        glEnable(GL_CLIP_PLANE0);
        glRasterPos2f(50, 20);
        raster_is("glRasterPos through a shader: a clip plane takes gl_ClipVertex, inside", 1);
        glClipPlane(GL_CLIP_PLANE0, keep_right);
        glRasterPos2f(50, 20);
        raster_is("glRasterPos through a shader: a clip plane takes gl_ClipVertex, outside", 0);
        glSelectBuffer(16, sel);
        glRenderMode(GL_SELECT);
        glInitNames(); glPushName(7);
        glBegin(GL_TRIANGLES); glVertex2f(40, 10); glVertex2f(60, 10); glVertex2f(50, 30); glEnd();
        hits = glRenderMode(GL_RENDER);
        ieq2("selection through a shader: clipped away by gl_ClipVertex", hits, 0);
        glClipPlane(GL_CLIP_PLANE0, keep_left);
        glRenderMode(GL_SELECT);
        glInitNames(); glPushName(7);
        glBegin(GL_TRIANGLES); glVertex2f(40, 10); glVertex2f(60, 10); glVertex2f(50, 30); glEnd();
        hits = glRenderMode(GL_RENDER);
        ieq2("selection through a shader: kept by gl_ClipVertex", hits, 1);
        glDisable(GL_CLIP_PLANE0);
    }
    done(p);
    /* the same in a program Appendix A.7 packs: a mat4 goes before every vec4, so the clip plane's varying leaves in row 4, not its declared 0 */
    p = prog2("varying mat4 m; varying vec2 a0, a1, a2, a3;\nvoid main() { gl_Position = ftransform(); gl_ClipVertex = gl_ModelViewMatrix * gl_Vertex * vec4(-1.0, 1.0, 1.0, 1.0);\n"
              "  m = mat4(1.0); a0 = vec2(0.0); a1 = a0; a2 = a0; a3 = a0; }\n",
              "varying mat4 m; varying vec2 a0, a1, a2, a3;\nvoid main() { gl_FragColor = vec4(a0 + a1 + a2 + a3, 0.0, 1.0) + (m[0] + m[1] + m[2] + m[3]) * 0.001; }\n");
    {
        static const GLdouble keep_left[4] = { -1.0, 0.0, 0.0, 30.0 }, keep_right[4] = { 1.0, 0.0, 0.0, -30.0 };
        glClipPlane(GL_CLIP_PLANE0, keep_right);
        glEnable(GL_CLIP_PLANE0);
        glRasterPos2f(50, 20);
        raster_is("a packed program: the clip plane's row, outside", 0);
        glClipPlane(GL_CLIP_PLANE0, keep_left);
        glRasterPos2f(50, 20);
        raster_is("a packed program: the clip plane's row, inside", 1);
        glDisable(GL_CLIP_PLANE0);
    }
    done(p);
    /* display lists hold the calls */
    {
        GLuint l = glGenLists(1);
        GLint loc;
        p = prog2("attribute vec4 at; void main() { gl_Position = ftransform(); gl_FrontColor = at; }\n", "uniform vec4 k;\nvoid main() { gl_FragColor = gl_Color + k; }\n");
        loc = glGetUniformLocation(p, "k");
        glUseProgram(0);
        glNewList(l, GL_COMPILE);
        glUseProgram(p);
        glUniform4f(loc, 0.5f, 0, 0, 0);
        glVertexAttrib4f(glGetAttribLocation(p, "at"), 0, 0.25f, 0, 1);
        glBegin(GL_QUADS); glVertex2f(0, 0); glVertex2f(64, 0); glVertex2f(64, 64); glVertex2f(0, 64); glEnd();
        glUseProgram(0);
        glEndList();
        glGetIntegerv(GL_CURRENT_PROGRAM, &loc);
        ieq2("glUseProgram in GL_COMPILE does nothing yet", loc, 0);
        glCallList(l);
        px(32, 32, "program, uniform and attribute from a list", 128, 64, 0, 255);
        glDeleteLists(l, 1);
        done(p);
    }
    /* more varying floats than varyings of one row each: packed */
    p = prog2("varying vec2 a, b, c, d, e, f, g, h, i, j, k, l;\nvoid main() { gl_Position = ftransform(); a = vec2(0.1, 0.0); b = vec2(0.0, 0.2); c = vec2(0.1, 0.1); d = e = f = g = h = i = j = vec2(0.0); k = vec2(0.5, 0.0); l = vec2(0.0, 0.25); }\n",
              "varying vec2 a, b, c, d, e, f, g, h, i, j, k, l;\nvoid main() { gl_FragColor = vec4(a.x + c.x + l.y + d.x + e.x + f.x + g.x + h.x, b.y + c.y + i.x + j.x, k.x, 1.0); }\n");
    q64();
    px(32, 32, "12 vec2 varyings", 115, 76, 128, 255);
    done(p);
    GL_CLEAN("the pipeline around a shader");
}

/* what the PC has to know of a vertex: each shader here moves it where the matrices alone would not */
static void test_glsl_host(void)
{
    GLuint p;
    GLfloat fb[32];
    GLuint sel[16];
    GLint hits;
    plain();
    /* polygon mode comes after culling: the swap of x and y turns a front face into a back one */
    p = prog2("void main() { gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.y, gl_Vertex.x, 0.0, 1.0); gl_FrontColor = vec4(1.0); }\n", COLOR_FS);
    glEnable(GL_CULL_FACE); glCullFace(GL_BACK);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_QUADS); glVertex2f(10, 10); glVertex2f(50, 10); glVertex2f(50, 30); glVertex2f(10, 30); glEnd();
    px(10, 20, "polygon mode: a back face by the shader is culled", 0, 0, 0, 255);
    glCullFace(GL_FRONT);
    glBegin(GL_QUADS); glVertex2f(10, 10); glVertex2f(50, 10); glVertex2f(50, 30); glVertex2f(10, 30); glEnd();
    px(10, 20, "polygon mode: and with the front culled it is drawn, at the shader's place", 255, 255, 255, 255);
    glDisable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    done(p);
    /* the stipple runs in pixels of where the shader put the line */
    p = prog2("void main() { gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.x * 2.0, gl_Vertex.yzw); gl_FrontColor = vec4(1.0); }\n", COLOR_FS);
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(1, 0x00FF);
    glBegin(GL_LINES); glVertex2f(10, 10.5f); glVertex2f(26, 10.5f); glEnd();
    px(24, 10, "a stipple under a shader: on", 255, 255, 255, 255);
    px(32, 10, "off", 0, 0, 0, 255);
    px(40, 10, "on again", 255, 255, 255, 255);
    glDisable(GL_LINE_STIPPLE);
    done(p);
    /* an antialiased line's rectangle follows the shader */
    p = prog2("void main() { gl_Position = gl_ModelViewProjectionMatrix * (gl_Vertex + vec4(100.0, 0.0, 0.0, 0.0)); gl_FrontColor = vec4(1.0); }\n", COLOR_FS);
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(3.0f);
    glBegin(GL_LINES); glVertex2f(10, 50.5f); glVertex2f(40, 50.5f); glEnd();
    px(125, 50, "an antialiased line under a shader", 255, 255, 255, 255);
    px(125, 51, "and its width", 255, 255, 255, 255);
    px(25, 50, "not where the matrices alone put it", 0, 0, 0, 255);
    glLineWidth(1.0f);
    glDisable(GL_LINE_SMOOTH);
    done(p);
    /* feedback and selection see the shader's vertex */
    p = prog2("uniform float dx;\nvoid main() { gl_Position = gl_ModelViewProjectionMatrix * (gl_Vertex + vec4(dx, 0.0, 0.0, 0.0)); gl_FrontColor = vec4(0.25, 0.5, 0.75, 1.0); gl_PointSize = 1.0; }\n", COLOR_FS);
    glUniform1f(glGetUniformLocation(p, "dx"), 100.0f);
    glFeedbackBuffer(32, GL_2D, fb);
    glRenderMode(GL_FEEDBACK);
    glBegin(GL_POINTS); glVertex2f(10, 20); glEnd();
    ieq2("feedback: a point and its two numbers", glRenderMode(GL_RENDER), 3);
    val("feedback: x from the shader", fb[1], 110, 0.01);
    val("feedback: y", fb[2], 20, 0.01);
    glFeedbackBuffer(32, GL_3D_COLOR, fb);
    glRenderMode(GL_FEEDBACK);
    glBegin(GL_POINTS); glVertex2f(10, 20); glEnd();
    glRenderMode(GL_RENDER);
    val("feedback: the shader's colour", fb[5], 0.5, 0.01);
    glSelectBuffer(16, sel);
    glRenderMode(GL_SELECT);
    glInitNames(); glPushName(7);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(105, 115, 15, 25, -1, 1); glMatrixMode(GL_MODELVIEW);
    glBegin(GL_POINTS); glVertex2f(10, 20); glEnd();
    hits = glRenderMode(GL_RENDER);
    ieq2("selection: a hit where the shader put it", hits, 1);
    glRenderMode(GL_SELECT);
    glInitNames(); glPushName(7);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(5, 15, 15, 25, -1, 1); glMatrixMode(GL_MODELVIEW);
    glBegin(GL_POINTS); glVertex2f(10, 20); glEnd();
    hits = glRenderMode(GL_RENDER);
    ieq2("selection: none where the matrices alone put it", hits, 0);
    done(p);
    /* GL_FLAT: the provoking vertex's colours as the shader made them, not as its attributes were */
    p = prog2("void main() { gl_Position = ftransform(); gl_FrontColor = vec4(gl_Vertex.x / 64.0, gl_Vertex.y / 64.0, 0.0, 1.0); gl_FrontSecondaryColor = vec4(0.0, 0.0, gl_Vertex.y / 128.0, 0.0); }\n", "void main() { gl_FragColor = gl_Color + gl_SecondaryColor; }\n");
    glShadeModel(GL_FLAT);
    glBegin(GL_TRIANGLES); glVertex2f(0, 0); glVertex2f(64, 0); glVertex2f(0, 64); glEnd();
    px(10, 10, "GL_FLAT of a shader's colour", 0, 255, 128, 255);
    px(40, 10, "the whole triangle", 0, 255, 128, 255);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_QUADS); glVertex2f(0, 0); glVertex2f(64, 0); glVertex2f(64, 32); glVertex2f(0, 32); glEnd();
    px(10, 10, "a quad's provoking vertex is its last", 0, 128, 64, 255);
    glBegin(GL_LINES); glVertex2f(0, 40.5f); glVertex2f(64, 40.5f); glEnd();
    px(10, 40, "a line's", 255, 161, 80, 255);
    glShadeModel(GL_SMOOTH);
    done(p);
    /* a built-in structure whole, by its type; an array of no stated size */
    {
        static const GLfloat md[4] = { 0.25f, 0.5f, 0.75f, 1.0f };
        glMaterialfv(GL_FRONT, GL_DIFFUSE, md);
        p = prog2("void main() { gl_Position = ftransform(); gl_MaterialParameters m = gl_FrontMaterial; gl_FrontColor = m.diffuse; }\n", "uniform float k[];\nvoid main() { gl_FragColor = gl_Color + vec4(k[2], 0.0, 0.0, 0.0); }\n");
        glUniform1f(glGetUniformLocation(p, "k[2]"), 0.25f);
        q64();
        px(32, 32, "a structure whole, an unsized array", 128, 128, 191, 255);
        {
            GLint n = 0, size = 0, i, found = 0;
            GLenum type = 0;
            char name[64];
            glGetProgramiv(p, GL_ACTIVE_UNIFORMS, &n);
            for (i = 0; i < n; i++) { glGetActiveUniform(p, (GLuint)i, sizeof name, NULL, &size, &type, name); if (!strcmp(name, "gl_FrontMaterial.diffuse") && type == GL_FLOAT_VEC4) found = 1; }
            ieq2("a built-in in use is an active uniform", found, 1);
        }
        done(p);
    }
    /* a link that fails leaves the program running what it ran */
    {
        GLuint bad = glCreateShader(GL_FRAGMENT_SHADER), v2;
        static const char *nomain = "vec4 g() { return vec4(1.0); }\n";
        GLint st = 1;
        p = prog2(PLAIN_VS, "void main() { gl_FragColor = vec4(0.0, 1.0, 0.0, 1.0); }\n");
        glShaderSource(bad, 1, &nomain, NULL);
        glCompileShader(bad);
        glGetShaderiv(bad, GL_COMPILE_STATUS, &st);
        ieq2("a shader of functions alone compiles", st, 1);
        v2 = sh(GL_FRAGMENT_SHADER, "void main() { gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0); }\n");
        glAttachShader(p, bad); glAttachShader(p, v2);
        glLinkProgram(p);                                       /* two mains: one from prog2's deleted shader stays attached */
        glGetProgramiv(p, GL_LINK_STATUS, &st);
        ieq2("the link fails", st, 0);
        q64();
        px(32, 32, "and the program in use still draws", 0, 255, 0, 255);
        glDeleteShader(bad); glDeleteShader(v2);
        done(p);
    }
    /* samplers of two kinds on a unit do not validate */
    {
        GLint st = 1;
        p = prog2(PLAIN_VS, "uniform sampler2D a; uniform samplerCube b;\nvoid main() { gl_FragColor = texture2D(a, gl_TexCoord[0].st) + textureCube(b, gl_TexCoord[0].xyz); }\n");
        glValidateProgram(p);
        glGetProgramiv(p, GL_VALIDATE_STATUS, &st);
        ieq2("two kinds of sampler on unit 0", st, 0);
        glUniform1i(glGetUniformLocation(p, "b"), 1);
        glValidateProgram(p);
        glGetProgramiv(p, GL_VALIDATE_STATUS, &st);
        ieq2("each on its own", st, 1);
        done(p);
    }
    /* gl_TexCoord redeclared with its size */
    p = prog2("varying vec4 gl_TexCoord[2];\nvoid main() { gl_Position = ftransform(); gl_TexCoord[1] = vec4(0.5, 0.25, 0.0, 1.0); }\n", "varying vec4 gl_TexCoord[2];\nvoid main() { gl_FragColor = gl_TexCoord[1]; }\n");
    q64();
    px(32, 32, "gl_TexCoord redeclared", 128, 64, 0, 255);
    done(p);
    /* the interpreter against the board: control flow and functions, the same vertex both ways */
    p = prog2("uniform float k[3]; uniform int n; attribute vec2 a;\nvoid main() { float s = 0.0; for (int i = 0; i < n; i++) { if (k[i] > 0.5) { s += sqrt(k[i]) + sin(k[i]); if (s > 2.5) break; } else s -= pow(k[i], 2.0) * exp(a.x); }\n"
              "  vec4 q = gl_Vertex + vec4(floor(s * 16.0), mod(s * 8.0, 7.0) + fract(a.y), 0.0, 0.0); if (q.x > 0.0 && a.x > 0.0) q.y = q.y + 2.0; gl_Position = gl_ModelViewProjectionMatrix * q; gl_FrontColor = vec4(1.0); }\n", COLOR_FS);
    {
        static const GLfloat ks[3] = { 1.0f, 0.25f, 4.0f };
        int l = glGetAttribLocation(p, "a");
        glUniform1fv(glGetUniformLocation(p, "k"), 3, ks);
        glUniform1i(glGetUniformLocation(p, "n"), 3);
        glVertexAttrib4f(l, 0.5f, 0.25f, 0, 1);
        glFeedbackBuffer(32, GL_2D, fb);
        glRenderMode(GL_FEEDBACK);
        glBegin(GL_POINTS); glVertex2f(10.5f, 10); glEnd();
        glRenderMode(GL_RENDER);
        glBegin(GL_POINTS); glVertex2f(10.5f, 10); glEnd();
        px((int)fb[1], (int)fb[2], "the PC's vertex is the board's", 255, 255, 255, 255);
        {
            double s = 1.0 + sin(1.0);
            s -= 0.0625 * exp(0.5);
            s += 2.0 + sin(4.0);
            val("and the number by hand, x", fb[1], 10.5 + floor(s * 16.0), 0.02);
        }
    }
    done(p);
    GL_CLEAN("what the PC knows of a shader's vertex");
    plain();
}

/* the older names of the same thing */
static void test_glsl_arb(void)
{
    PFNGLCREATESHADEROBJECTARBPROC cs = (PFNGLCREATESHADEROBJECTARBPROC)wglGetProcAddress("glCreateShaderObjectARB");
    PFNGLSHADERSOURCEARBPROC ss = (PFNGLSHADERSOURCEARBPROC)wglGetProcAddress("glShaderSourceARB");
    PFNGLCOMPILESHADERARBPROC co = (PFNGLCOMPILESHADERARBPROC)wglGetProcAddress("glCompileShaderARB");
    PFNGLCREATEPROGRAMOBJECTARBPROC cp = (PFNGLCREATEPROGRAMOBJECTARBPROC)wglGetProcAddress("glCreateProgramObjectARB");
    PFNGLATTACHOBJECTARBPROC at = (PFNGLATTACHOBJECTARBPROC)wglGetProcAddress("glAttachObjectARB");
    PFNGLLINKPROGRAMARBPROC ln = (PFNGLLINKPROGRAMARBPROC)wglGetProcAddress("glLinkProgramARB");
    PFNGLUSEPROGRAMOBJECTARBPROC up = (PFNGLUSEPROGRAMOBJECTARBPROC)wglGetProcAddress("glUseProgramObjectARB");
    PFNGLGETOBJECTPARAMETERIVARBPROC gp = (PFNGLGETOBJECTPARAMETERIVARBPROC)wglGetProcAddress("glGetObjectParameterivARB");
    PFNGLGETHANDLEARBPROC gh = (PFNGLGETHANDLEARBPROC)wglGetProcAddress("glGetHandleARB");
    PFNGLDELETEOBJECTARBPROC del = (PFNGLDELETEOBJECTARBPROC)wglGetProcAddress("glDeleteObjectARB");
    PFNGLUNIFORM4FARBPROC u4 = (PFNGLUNIFORM4FARBPROC)wglGetProcAddress("glUniform4fARB");
    PFNGLGETUNIFORMLOCATIONARBPROC gu = (PFNGLGETUNIFORMLOCATIONARBPROC)wglGetProcAddress("glGetUniformLocationARB");
    static const GLcharARB *vs = PLAIN_VS, *fs = "uniform vec4 c;\nvoid main() { gl_FragColor = c; }\n";
    GLhandleARB v, f, p;
    GLint st = 0;
    plain();
    g_checks++;
    if (!cs || !ss || !co || !cp || !at || !ln || !up || !gp || !gh || !del || !u4 || !gu) { g_bad++; printf("FAIL an ARB_shader_objects entry point is missing\n"); return; }
    v = cs(GL_VERTEX_SHADER_ARB); ss(v, 1, &vs, NULL); co(v);
    gp(v, GL_OBJECT_COMPILE_STATUS_ARB, &st); ieq2("GL_OBJECT_COMPILE_STATUS_ARB", st, 1);
    gp(v, GL_OBJECT_TYPE_ARB, &st); ieq2("GL_OBJECT_TYPE_ARB", st, GL_SHADER_OBJECT_ARB);
    f = cs(GL_FRAGMENT_SHADER_ARB); ss(f, 1, &fs, NULL); co(f);
    p = cp(); at(p, v); at(p, f); ln(p);
    gp(p, GL_OBJECT_LINK_STATUS_ARB, &st); ieq2("GL_OBJECT_LINK_STATUS_ARB", st, 1);
    up(p);
    ieq2("glGetHandleARB", (int)gh(GL_PROGRAM_OBJECT_ARB), (int)p);
    u4(gu(p, "c"), 0, 1, 0, 1);
    q64();
    px(32, 32, "the ARB names", 0, 255, 0, 255);
    up(0);
    del(p); del(v); del(f);
    GL_CLEAN("ARB_shader_objects");
    plain();
}

/* every level a colour of its own: red, green, blue, white, yellow, cyan */
static GLuint tex3d_levels(int w, int h, int d)
{
    static const uint8_t C[6][4] = { { 255, 0, 0, 255 }, { 0, 255, 0, 255 }, { 0, 0, 255, 255 }, { 255, 255, 255, 255 }, { 255, 255, 0, 255 }, { 0, 255, 255, 255 } };
    GLuint t;
    int l;
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_3D, t);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    for (l = 0; ; l++) {
        int lw = w >> l ? w >> l : 1, lh = h >> l ? h >> l : 1, ld = d >> l ? d >> l : 1, i, n = lw * lh * ld;
        uint8_t *o = (uint8_t *)malloc((size_t)n * 4);
        for (i = 0; i < n; i++) memcpy(o + 4 * i, C[l], 4);
        glTexImage3D(GL_TEXTURE_3D, l, GL_RGBA8, lw, lh, ld, 0, GL_RGBA, GL_UNSIGNED_BYTE, o);
        free(o);
        if (lw == 1 && lh == 1 && ld == 1) break;
    }
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    return t;
}

static GLuint tex3d_grad(void)
{
    uint8_t v[4 * 4 * 4 * 4];
    GLuint t;
    int x, y, z;
    for (z = 0; z < 4; z++) for (y = 0; y < 4; y++) for (x = 0; x < 4; x++) { uint8_t *o = v + ((z * 4 + y) * 4 + x) * 4; o[0] = (uint8_t)(x * 85); o[1] = (uint8_t)(y * 85); o[2] = (uint8_t)(z * 85); o[3] = 255; }
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_3D, t);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, 4, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, v);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    return t;
}

static void tex3d_small(float r)                                /* 8 texels over 2 pixels: level 2 */
{
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_QUADS);
    glTexCoord3f(0, 0, r); glVertex2f(0, 0);
    glTexCoord3f(1, 0, r); glVertex2f(2, 0);
    glTexCoord3f(1, 1, r); glVertex2f(2, 2);
    glTexCoord3f(0, 1, r); glVertex2f(0, 2);
    glEnd();
    glFinish();
}

/* the pictures of a volume in the TMU. GPU2_NO_3D=1 sends them through the slices side by side instead; what only a volume reaches then has its own expected answer */
static void test_tex3d_hw(void)
{
    static const GLfloat brd[4] = { 0.2f, 0.4f, 0.6f, 1.0f };
    int hw = !getenv("GPU2_NO_3D");
    GLuint t, p;
    GLint v = -1;
    plain();
    ortho();
    glClearColor(0, 0, 0, 1);
    glEnable(GL_TEXTURE_3D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    /* the largest: more than lies side by side, and the depth stops a texel short of 256 */
    glTexImage3D(GL_PROXY_TEXTURE_3D, 0, GL_RGBA8, 256, 256, 255, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glGetTexLevelParameteriv(GL_PROXY_TEXTURE_3D, 0, GL_TEXTURE_WIDTH, &v);
    /* a board whose TMU has no volumes (VOL3D = 0) answers as GPU2_NO_3D does, and the proxy is how a program asks: the expectations follow the board, as d3d_test's follow its caps */
    if (hw && v == 0) { hw = 0; printf("note: this board's TMU has no volumes: 3D textures go through the slices side by side\n"); }
    ieq2("a proxy of the TMU's largest volume", v, hw ? 256 : 0);
    {
        uint8_t *o = (uint8_t *)calloc(200 * 2 * 3, 4);
        int z;
        for (z = 0; z < 3; z++) { int i; for (i = 140; i < 160; i++) { o[(z * 400 + i) * 4] = (uint8_t)(50 + 100 * z); o[(z * 400 + i) * 4 + 3] = 255; } }
        glGenTextures(1, &t);
        glBindTexture(GL_TEXTURE_3D, t);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, 200, 2, 3, 0, GL_RGBA, GL_UNSIGNED_BYTE, o);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        free(o);
        v = (GLint)glGetError();
        ieq2("a volume 200 wide", v, hw ? GL_NO_ERROR : GL_INVALID_VALUE);
        if (hw) {
            tex3d_set(t, GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE);
            tex3d_scene(0.5f);
            px(48, 8, "a volume 200 wide: its middle slice", 150, 0, 0, 255);
        }
        glDeleteTextures(1, &t);
    }
    /* BASE_LEVEL and MAX_LEVEL: a tail on 32 bytes shares the memory */
    t = tex3d_levels(8, 8, 8);
    tex3d_set(t, GL_NEAREST, GL_NEAREST_MIPMAP_NEAREST, GL_CLAMP_TO_EDGE);
    tex3d_scene(0.5f);
    px(32, 32, "3D, the whole chain", 255, 0, 0, 255);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_BASE_LEVEL, 1);
    tex3d_scene(0.5f);
    px(32, 32, "3D BASE_LEVEL 1", 0, 255, 0, 255);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_BASE_LEVEL, 3);
    tex3d_scene(0.5f);
    px(32, 32, "3D BASE_LEVEL 3", 255, 255, 255, 255);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_BASE_LEVEL, 0);
    tex3d_small(0.5f);
    px(1, 1, "3D level 2 of the size", 0, 0, 255, 255);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAX_LEVEL, 1);
    tex3d_small(0.5f);
    px(1, 1, "3D MAX_LEVEL 1", 0, 255, 0, 255);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAX_LEVEL, 1000);
    /* the level, as the program works it out or adds to the TMU's */
    glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_LOD_BIAS, 4.0f);
    tex3d_scene(0.5f);
    px(32, 32, "3D LOD bias: magnified by 8, level 1", 0, 255, 0, 255);
    glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_LOD_BIAS, 0.0f);
    glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_MIN_LOD, 2.0f);
    tex3d_scene(0.5f);
    px(32, 32, "3D MIN_LOD 2", 0, 0, 255, 255);
    glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_MIN_LOD, -1000.0f);
    glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_MAX_LOD, 1.0f);
    tex3d_small(0.5f);
    px(1, 1, "3D MAX_LOD 1", 0, 255, 0, 255);
    glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_MAX_LOD, 1000.0f);
    glDeleteTextures(1, &t);
    /* a tail that does not start on 32 bytes: 5 x 5 x 5 x 4 = 500 */
    t = tex3d_levels(5, 5, 5);
    tex3d_set(t, GL_NEAREST, GL_NEAREST_MIPMAP_NEAREST, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_BASE_LEVEL, 1);
    tex3d_scene(0.5f);
    px(32, 32, "3D BASE_LEVEL 1, its own image", 0, 255, 0, 255);
    {
        static const uint8_t mag[4] = { 255, 0, 255, 255 };
        int x, y, z;
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        for (z = 0; z < 2; z++) for (y = 0; y < 2; y++) for (x = 0; x < 2; x++) glTexSubImage3D(GL_TEXTURE_3D, 1, x, y, z, 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, mag);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    }
    tex3d_scene(0.5f);
    px(32, 32, "3D BASE_LEVEL 1 after a write", 255, 0, 255, 255);
    glDeleteTextures(1, &t);
    /* the wrap of r in the filter, and what the TMU has no mode for */
    t = tex3d_grad();
    tex3d_set(t, GL_LINEAR, GL_LINEAR, GL_REPEAT);
    tex3d_scene(0.0f);
    px(8, 8, "3D LINEAR REPEAT: r = 0 between the last slice and the first", 0, 0, 128, 255);
    tex3d_set(t, GL_LINEAR, GL_LINEAR, GL_MIRRORED_REPEAT);
    tex3d_scene(1.125f);
    px(8, 8, "3D LINEAR MIRRORED_REPEAT", 0, 0, 255, 255);
    tex3d_set(t, GL_NEAREST, GL_NEAREST, GL_CLAMP);
    tex3d_scene(1.125f);
    px(8, 8, "3D NEAREST GL_CLAMP: the last slice", 0, 0, 255, 255);
    glTexParameterfv(GL_TEXTURE_3D, GL_TEXTURE_BORDER_COLOR, brd);
    tex3d_set(t, GL_LINEAR, GL_LINEAR, GL_CLAMP);
    tex3d_scene(1.125f);
    px(8, 8, "3D LINEAR GL_CLAMP: half the border", 26, 51, 204, 255);
    tex3d_set(t, GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_BORDER);
    tex3d_scene(1.0f);
    px(8, 8, "3D LINEAR CLAMP_TO_BORDER", 26, 51, 204, 255);
    tex3d_scene(1.5f);
    px(8, 8, "3D LINEAR CLAMP_TO_BORDER past it", 51, 102, 153, 255);
    /* border texels do not halve with a level: side by side, and a volume again without them */
    {
        uint8_t bv[6 * 6 * 6 * 4];
        int i, x, y, z;
        for (z = 0; z < 6; z++) for (y = 0; y < 6; y++) for (x = 0; x < 6; x++) {
            int in = x && x < 5 && y && y < 5 && z && z < 5;
            i = ((z * 6 + y) * 6 + x) * 4;
            bv[i] = in ? 0 : 255; bv[i + 1] = in ? 255 : 0; bv[i + 2] = 0; bv[i + 3] = 255;
        }
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, 6, 6, 6, 1, GL_RGBA, GL_UNSIGNED_BYTE, bv);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        tex3d_set(t, GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_BORDER);
        tex3d_scene(0.5f);
        px(32, 32, "3D with border texels, inside", 0, 255, 0, 255);
        tex3d_scene(1.5f);
        px(32, 32, "3D with border texels, in them", 255, 0, 0, 255);
    }
    {
        uint8_t o[4] = { 10, 20, 30, 255 };
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, 1, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, o);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        tex3d_set(t, GL_NEAREST, GL_NEAREST, GL_REPEAT);
        tex3d_scene(0.5f);
        px(32, 32, "3D redefined without a border", 10, 20, 30, 255);
    }
    glDeleteTextures(1, &t);
    /* a size that is no power of two, and depth > width > height */
    {
        uint8_t nv[3 * 2 * 7 * 4];
        int x, y, z;
        for (z = 0; z < 7; z++) for (y = 0; y < 2; y++) for (x = 0; x < 3; x++) { uint8_t *o = nv + ((z * 2 + y) * 3 + x) * 4; o[0] = (uint8_t)(x * 100); o[1] = (uint8_t)(y * 200); o[2] = (uint8_t)(z * 40); o[3] = 255; }
        glGenTextures(1, &t);
        glBindTexture(GL_TEXTURE_3D, t);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexParameteri(GL_TEXTURE_3D, GL_GENERATE_MIPMAP, GL_TRUE);
        glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, 3, 2, 7, 0, GL_RGBA, GL_UNSIGNED_BYTE, nv);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        tex3d_set(t, GL_NEAREST, GL_NEAREST_MIPMAP_NEAREST, GL_CLAMP_TO_EDGE);
        tex3d_scene(5.5f / 7.0f);
        px(53, 48, "3D 3 x 2 x 7", 200, 200, 200, 255);
        glDeleteTextures(1, &t);
    }
    glDisable(GL_TEXTURE_3D);
    /* a shader's sampler3D */
    t = tex3d_levels(8, 8, 8);
    tex3d_set(t, GL_NEAREST, GL_NEAREST_MIPMAP_NEAREST, GL_CLAMP_TO_EDGE);
    p = prog2(PLAIN_VS, "uniform sampler3D s;\nvoid main() { gl_FragColor = texture3DProj(s, vec4(gl_TexCoord[0].st, 0.5, 1.0) * 2.0); }\n");
    q64();
    px(32, 32, "texture3DProj", 255, 0, 0, 255);
    done(p);
    p = prog2(PLAIN_VS, "uniform sampler3D s;\nvoid main() { gl_FragColor = texture3D(s, vec3(gl_TexCoord[0].st, 0.5), 4.0); }\n");
    q64();
    px(32, 32, "texture3D with a bias: level 1", 0, 255, 0, 255);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_BASE_LEVEL, 1);
    q64();
    px(32, 32, "and BASE_LEVEL 1, whose size starts the level: level 1", 0, 255, 0, 255);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_BASE_LEVEL, 0);
    glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_LOD_BIAS, 1.0f);
    q64();
    px(32, 32, "and the object's bias: level 2", 0, 0, 255, 255);
    glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_LOD_BIAS, 0.0f);
    done(p);
    if (hw) {
        p = prog2(PLAIN_VS, "uniform sampler3D a[2];\nvoid main() { gl_FragColor = texture3D(a[1], vec3(gl_TexCoord[0].st, 0.5)); }\n");
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_3D, t);
        glActiveTexture(GL_TEXTURE0);
        glUniform1i(glGetUniformLocation(p, "a[1]"), 1);
        q64();
        px(32, 32, "an array of sampler3D", 255, 0, 0, 255);
        done(p);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_3D, 0);
        glActiveTexture(GL_TEXTURE0);
        p = prog2(PLAIN_VS, "uniform sampler3D s;\nvec4 f(sampler3D q, vec3 c) { return texture3D(q, c); }\nvoid main() { gl_FragColor = f(s, vec3(gl_TexCoord[0].st, 0.5)); }\n");
        q64();
        px(32, 32, "a sampler3D as a parameter", 255, 0, 0, 255);
        done(p);
    }
    glDeleteTextures(1, &t);
    t = tex3d_grad();
    glTexParameterfv(GL_TEXTURE_3D, GL_TEXTURE_BORDER_COLOR, brd);
    tex3d_set(t, GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_BORDER);
    p = prog2(PLAIN_VS, "uniform sampler3D s;\nvoid main() { gl_FragColor = texture3D(s, vec3(gl_TexCoord[0].st, 1.0)); }\n");
    q64();
    px(8, 8, "sampler3D CLAMP_TO_BORDER: half the border", 26, 51, 204, 255);
    tex3d_set(t, GL_LINEAR, GL_LINEAR, GL_MIRRORED_REPEAT);
    q64();
    px(8, 8, "sampler3D MIRRORED_REPEAT in r", 0, 0, 255, 255);
    done(p);
    glDeleteTextures(1, &t);
    plain();
}

/* what the hardware has that the library reaches since: 8 units, all a framebuffer can render in, blits, packed depth and stencil, pixel buffers, a level from a fragment program */
#ifndef GL_READ_FRAMEBUFFER_EXT
#define GL_READ_FRAMEBUFFER_EXT 0x8CA8
#define GL_DRAW_FRAMEBUFFER_EXT 0x8CA9
#define GL_READ_FRAMEBUFFER_BINDING_EXT 0x8CAA
#endif
#ifndef GL_DEPTH_STENCIL_EXT
#define GL_DEPTH_STENCIL_EXT 0x84F9
#define GL_UNSIGNED_INT_24_8_EXT 0x84FA
#define GL_DEPTH24_STENCIL8_EXT 0x88F0
#endif
typedef void (APIENTRY *fb_blit_t)(GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLbitfield, GLenum);
typedef void (APIENTRY *fb_att_t)(GLenum, GLenum, GLenum, GLint *);
typedef void (APIENTRY *fb_rbq_t)(GLenum, GLenum, GLint *);
typedef void (APIENTRY *fb_grb_t)(GLsizei, GLuint *);
typedef void (APIENTRY *fb_brb_t)(GLenum, GLuint);
typedef void (APIENTRY *fb_rbs_t)(GLenum, GLenum, GLsizei, GLsizei);
typedef void (APIENTRY *fb_frb_t)(GLenum, GLenum, GLenum, GLuint);
typedef void (APIENTRY *fb_ft3_t)(GLenum, GLenum, GLenum, GLuint, GLint, GLint);
static fb_blit_t p_Blit;
static fb_att_t p_AttQ;
static fb_rbq_t p_RbQ;
static fb_grb_t p_GenRb;
static fb_brb_t p_BindRb;
static fb_rbs_t p_RbStore;
static fb_frb_t p_FbRb;
static fb_ft3_t p_FbTex3;

static void fb_view(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, w, 0, h, -1, 1);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
}

static void fb_back(void)
{
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
    fb_view(W, H);
}

static void fb_done(const char *what) { val(what, (double)glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT), GL_FRAMEBUFFER_COMPLETE_EXT, 0); }

static GLuint fb_tex(GLenum target, GLenum filter)
{
    GLuint t;
    glGenTextures(1, &t);
    glBindTexture(target, t);
    glTexParameteri(target, GL_TEXTURE_MIN_FILTER, (GLint)filter);
    glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    return t;
}

static void rgba_is(const char *what, const uint8_t *c, int r, int g, int b, int a)
{
    g_checks++;
    if (abs(c[0] - r) > 2 || abs(c[1] - g) > 2 || abs(c[2] - b) > 2 || abs(c[3] - a) > 2) {
        g_bad++;
        printf("FAIL %s: %d %d %d %d, expected %d %d %d %d\n", what, c[0], c[1], c[2], c[3], r, g, b, a);
    }
}

static void test_units(void)
{
    GLuint t[8], p;
    GLint n = 0;
    int u, i;
    static const float xy[4][2] = { { 0, 0 }, { 64, 0 }, { 64, 64 }, { 0, 64 } };
    static const char *const NM[8] = { "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" };
    glGetIntegerv(GL_MAX_TEXTURE_UNITS, &n);
    val("GL_MAX_TEXTURE_UNITS", n, 8, 0);
    glGenTextures(8, t);
    for (u = 0; u < 8; u++) {                                   /* a bit a unit: which texel, which bit says the unit and its coordinates were each its own */
        uint8_t c[8] = { 0, 0, 0, 255, 0, 0, 0, 255 };
        c[0] = (uint8_t)(1u << u); c[5] = (uint8_t)(1u << u);
        glActiveTexture(GL_TEXTURE0 + (GLenum)u);
        glBindTexture(GL_TEXTURE_2D, t[u]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, c);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_ADD);
        glEnable(GL_TEXTURE_2D);
    }
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glColor4f(0, 0, 0.25f, 1);
    glBegin(GL_QUADS);
    for (i = 0; i < 4; i++) {
        for (u = 0; u < 8; u++) p_MultiTexCoord2f(GL_TEXTURE0 + (GLenum)u, u & 1 ? 0.75f : 0.25f, 0.5f);
        glVertex2f(xy[i][0], xy[i][1]);
    }
    glEnd();
    px(32, 32, "8 units of the fixed pipeline, more varyings than rows", 0x55, 0xAA, 64, 255);
    for (u = 7; u >= 0; u--) {
        glActiveTexture(GL_TEXTURE0 + (GLenum)u);
        glDisable(GL_TEXTURE_2D);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    }
    p = prog2(PLAIN_VS, "uniform sampler2D s0, s1, s2, s3, s4, s5, s6, s7;\nvoid main() {\n"
                        "  vec4 c = texture2D(s0, vec2(0.25)) + texture2D(s1, vec2(0.75)) + texture2D(s2, vec2(0.25)) + texture2D(s3, vec2(0.75));\n"
                        "  c += texture2D(s4, vec2(0.25)) + texture2D(s5, vec2(0.75)) + texture2D(s6, vec2(0.25)) + texture2D(s7, vec2(0.75));\n"
                        "  gl_FragColor = vec4(c.rgb, 1.0);\n}\n");
    for (u = 0; u < 8; u++) glUniform1i(glGetUniformLocation(p, NM[u]), u);
    glClear(GL_COLOR_BUFFER_BIT);
    q64();
    px(32, 32, "8 samplers in a fragment shader", 0x55, 0xAA, 0, 255);
    done(p);
    glDeleteTextures(8, t);
    glActiveTexture(GL_TEXTURE0);
    plain();
}

static void test_fbo_targets(void)
{
    GLuint t, d, fbo, rb;
    GLint v = 0;
    uint8_t buf[4 * 64 * 64 * 4];
    float z[64 * 64];
    uint32_t w[64 * 64];
    int i;
    glGenFramebuffersEXT(1, &fbo);

    /* a width B cannot render at: the target is an image of its own, copied back */
    t = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 33, 17, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t, 0);
    fb_done("a target 33 wide");
    fb_view(33, 17);
    glClearColor(1, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(0, 1, 0);
    quad(11, 0, 33, 17, 0);
    px(5, 8, "33 wide: cleared", 255, 0, 0, 255);
    px(20, 8, "33 wide: drawn", 0, 255, 0, 255);
    fb_back();
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_TEXTURE_2D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    quad(0, 0, 66, 34, 0);
    glDisable(GL_TEXTURE_2D);
    px(10, 16, "33 wide as a texture: cleared", 255, 0, 0, 255);
    px(40, 16, "33 wide as a texture: drawn", 0, 255, 0, 255);
    glDeleteTextures(1, &t);

    /* a level that is not the first */
    t = fb_tex(GL_TEXTURE_2D, GL_NEAREST_MIPMAP_NEAREST);
    memset(buf, 0x40, sizeof buf);
    for (i = 0; i <= 6; i++) glTexImage2D(GL_TEXTURE_2D, i, GL_RGBA, 64 >> i, 64 >> i, 0, GL_RGBA, GL_UNSIGNED_BYTE, buf);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t, 1);
    fb_done("level 1 of a texture");
    p_AttQ(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE_EXT, &v);
    val("attachment type", v, GL_TEXTURE, 0);
    p_AttQ(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME_EXT, &v);
    val("attachment name", v, t, 0);
    p_AttQ(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_LEVEL_EXT, &v);
    val("attachment level", v, 1, 0);
    fb_view(32, 32);
    glClearColor(0, 0, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    fb_back();
    glBindTexture(GL_TEXTURE_2D, t);
    glGetTexImage(GL_TEXTURE_2D, 1, GL_RGBA, GL_UNSIGNED_BYTE, buf);
    rgba_is("level 1 rendered", buf, 0, 0, 255, 255);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, buf);
    rgba_is("level 0 left alone", buf, 0x40, 0x40, 0x40, 0x40);
    glDeleteTextures(1, &t);

    /* a face of a cube */
    t = fb_tex(GL_TEXTURE_CUBE_MAP, GL_NEAREST);
    memset(buf, 0xFF, sizeof buf);
    for (i = 0; i < 6; i++) glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + (GLenum)i, 0, GL_RGBA, 16, 16, 0, GL_RGBA, GL_UNSIGNED_BYTE, buf);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_CUBE_MAP_POSITIVE_Y, t, 0);
    fb_done("a face of a cube");
    p_AttQ(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_CUBE_MAP_FACE_EXT, &v);
    val("attachment face", v, GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0);
    fb_view(16, 16);
    glClearColor(0, 1, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    fb_back();
    glBindTexture(GL_TEXTURE_CUBE_MAP, t);
    glGetTexImage(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, GL_RGBA, GL_UNSIGNED_BYTE, buf);
    rgba_is("+Y rendered", buf, 0, 255, 0, 255);
    glGetTexImage(GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, GL_RGBA, GL_UNSIGNED_BYTE, buf);
    rgba_is("+X left alone", buf, 255, 255, 255, 255);
    glDeleteTextures(1, &t);

    /* a slice of a volume */
    t = fb_tex(GL_TEXTURE_3D, GL_NEAREST);
    memset(buf, 0xFF, sizeof buf);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA, 8, 8, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, buf);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    p_FbTex3(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_3D, t, 0, 2);
    fb_done("a slice of a volume");
    p_AttQ(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_3D_ZOFFSET_EXT, &v);
    val("attachment slice", v, 2, 0);
    fb_view(8, 8);
    glClearColor(1, 0, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    fb_back();
    glBindTexture(GL_TEXTURE_3D, t);
    glGetTexImage(GL_TEXTURE_3D, 0, GL_RGBA, GL_UNSIGNED_BYTE, buf);
    rgba_is("slice 2 rendered", buf + 4 * 8 * 8 * 2, 255, 0, 255, 255);
    rgba_is("slice 1 left alone", buf + 4 * 8 * 8, 255, 255, 255, 255);
    glDeleteTextures(1, &t);

    /* a depth texture of a width B cannot render at, depth-only */
    d = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, 33, 17, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, 0, 0);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, d, 0);
    glDrawBuffer(GL_NONE); glReadBuffer(GL_NONE);
    fb_done("a depth target 33 wide");
    fb_view(33, 17);
    glEnable(GL_DEPTH_TEST);
    glClearDepth(1.0);
    glClear(GL_DEPTH_BUFFER_BIT);
    quad(11, 0, 33, 17, 0);
    glDisable(GL_DEPTH_TEST);
    glDrawBuffer(GL_BACK); glReadBuffer(GL_BACK);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, 0, 0);
    fb_back();
    glBindTexture(GL_TEXTURE_2D, d);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, z);
    val("depth 33 wide: cleared", z[8 * 33 + 5], 1.0, 0.002);
    val("depth 33 wide: drawn", z[8 * 33 + 20], 0.5, 0.002);
    glDeleteTextures(1, &d);

    /* depth and stencil in one texture */
    t = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 32, 16, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    d = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH24_STENCIL8_EXT, 32, 16, 0, GL_DEPTH_STENCIL_EXT, GL_UNSIGNED_INT_24_8_EXT, NULL);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t, 0);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, d, 0);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT, GL_TEXTURE_2D, d, 0);
    fb_done("a depth-stencil texture");
    fb_view(32, 16);
    glClearStencil(0); glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST); glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 5, 255); glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
    quad(16, 0, 32, 16, 0);
    val("a stencil value drawn", stencil_at(24, 8), 5, 0);
    val("a stencil value cleared", stencil_at(4, 8), 0, 0);
    glDisable(GL_STENCIL_TEST);
    glDepthFunc(GL_ALWAYS);
    w[0] = (0x400000u << 8) | 9u;
    glRasterPos2i(2, 2);
    glDrawPixels(1, 1, GL_DEPTH_STENCIL_EXT, GL_UNSIGNED_INT_24_8_EXT, w);
    val("glDrawPixels of both: stencil", stencil_at(2, 2), 9, 0);
    val("glDrawPixels of both: depth", depth_at(2, 2), 0.25, 0.002);
    glDepthFunc(GL_LESS);
    glDisable(GL_DEPTH_TEST);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, 0, 0);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT, GL_TEXTURE_2D, 0, 0);
    fb_back();
    glBindTexture(GL_TEXTURE_2D, d);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_STENCIL_EXT, GL_UNSIGNED_INT_24_8_EXT, w);
    val("the stencil of a depth-stencil texture", w[8 * 32 + 24] & 255, 5, 0);
    val("the depth of a depth-stencil texture", (double)(w[8 * 32 + 24] >> 8) / 16777215.0, 0.5, 0.002);
    glDeleteTextures(1, &t);
    glDeleteTextures(1, &d);

    /* a renderbuffer answers for itself */
    p_GenRb(1, &rb);
    p_BindRb(GL_RENDERBUFFER_EXT, rb);
    p_RbStore(GL_RENDERBUFFER_EXT, GL_DEPTH24_STENCIL8_EXT, 40, 24);
    p_RbQ(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_WIDTH_EXT, &v);
    val("renderbuffer width", v, 40, 0);
    p_RbQ(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_HEIGHT_EXT, &v);
    val("renderbuffer height", v, 24, 0);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    p_FbRb(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, rb);
    p_AttQ(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE_EXT, &v);
    val("attachment of a renderbuffer", v, GL_RENDERBUFFER_EXT, 0);
    p_FbRb(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, 0);
    fb_back();
    glDeleteFramebuffersEXT(1, &fbo);
    GL_CLEAN("framebuffer targets");
    plain();
}

/* targets whose height is no multiple of 8 (board B loads and stores whole bands of 8 rows) leave the next image intact: 32 textures 21 x
 * 29 (rows of 96 bytes) rendered into once, each followed by a 64 x 8 texture with a pattern; every target rendered again, then every
 * pattern read back. Without the bands' rows in the targets' memory, a pattern after a target had bytes zeroed where its 96-byte rows are
 * padding */
static void test_band_tail(void)
{
    enum { N = 32, TW = 21, TH = 29, VW = 64, VH = 8 };
    static uint8_t pat[VW * VH * 4], got[VW * VH * 4];
    GLuint t[N], v[N], fbo;
    int i, k, imgs = 0, bytes = 0;
    for (k = 0; k < VW * VH; k++) {
        pat[4 * k] = (uint8_t)(k * 7 + 1);
        pat[4 * k + 1] = (uint8_t)(k * 13 + 2);
        pat[4 * k + 2] = (uint8_t)(k * 29 + 3);
        pat[4 * k + 3] = 255;
    }
    glGenFramebuffersEXT(1, &fbo);
    for (i = 0; i < N; i++) {
        t[i] = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, TW, TH, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t[i], 0);
        fb_view(TW, TH);
        glColor3f(1, 0, 0);
        quad(0, 0, TW, TH, 0);
        glFinish();
        fb_back();
        v[i] = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, VW, VH, 0, GL_RGBA, GL_UNSIGNED_BYTE, pat);
    }
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    for (i = 0; i < N; i++) {
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t[i], 0);
        fb_view(TW, TH);
        glColor3f(0, 1, 0);
        quad(0, 0, TW, TH, 0);
        glFlush();
    }
    for (i = 0; i < N; i++) {
        int n = 0;
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, v[i], 0);
        memset(got, 0xA5, sizeof got);
        glReadPixels(0, 0, VW, VH, GL_RGBA, GL_UNSIGNED_BYTE, got);
        for (k = 0; k < VW * VH * 4; k++) n += got[k] != pat[k];
        imgs += n != 0;
        bytes += n;
    }
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t[N - 1], 0);
    fb_view(TW, TH);
    px(10, 14, "the last 21 x 29 target: green", 0, 255, 0, 255);
    px(20, 28, "its top-right pixel: green", 0, 255, 0, 255);
    fb_back();
    g_checks++;
    if (imgs) {
        g_bad++;
        printf("FAIL %d of %d textures after 21 x 29 targets changed (%d bytes)\n", imgs, N, bytes);
    }
    glDeleteFramebuffersEXT(1, &fbo);
    glDeleteTextures(N, t);
    glDeleteTextures(N, v);
    plain();
}

static void test_blit(void)
{
    GLuint src, dst, fbo[2], ds, dd;
    uint8_t c[4];
    GLint v = -1;
    float z = 0.0f;
    glGenFramebuffersEXT(2, fbo);
    src = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 64, 64, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    ds = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, 64, 64, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    dst = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 64, 64, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    dd = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, 64, 64, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo[0]);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, src, 0);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, ds, 0);
    fb_done("the source of a blit");
    fb_view(64, 64);
    glClearColor(1, 0, 0, 1); glClearDepth(0.25);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearDepth(1.0);
    glColor3f(0, 1, 0);
    quad(32, 0, 64, 64, 0);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo[1]);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, dst, 0);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, dd, 0);
    fb_done("the destination of a blit");
    fb_view(64, 64);
    glClearColor(0, 0, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* the two bindings */
    glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, fbo[0]);
    glBindFramebufferEXT(GL_DRAW_FRAMEBUFFER_EXT, fbo[1]);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING_EXT, &v);
    val("read binding", v, fbo[0], 0);
    glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, &v);
    val("draw binding", v, fbo[1], 0);
    px(8, 8, "a read comes from the read framebuffer", 255, 0, 0, 255);

    p_Blit(0, 0, 64, 64, 0, 0, 64, 64, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, fbo[1]);
    px(8, 8, "blit, 1:1: left", 255, 0, 0, 255);
    px(50, 8, "blit, 1:1: right", 0, 255, 0, 255);

    glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, fbo[0]);
    p_Blit(0, 0, 64, 64, 64, 0, 0, 64, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, fbo[1]);
    px(8, 8, "blit, mirrored: left", 0, 255, 0, 255);
    px(50, 8, "blit, mirrored: right", 255, 0, 0, 255);

    glClear(GL_COLOR_BUFFER_BIT);
    glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, fbo[0]);
    p_Blit(0, 0, 32, 32, 0, 0, 16, 16, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, fbo[1]);
    px(8, 8, "blit, scaled: inside", 255, 0, 0, 255);
    px(20, 8, "blit, scaled: outside", 0, 0, 255, 255);

    /* a source that hangs out of the framebuffer: only what is in it is written */
    glClear(GL_COLOR_BUFFER_BIT);
    glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, fbo[0]);
    p_Blit(-32, 0, 32, 64, 0, 0, 64, 64, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, fbo[1]);
    px(8, 8, "blit, clipped: out of the source", 0, 0, 255, 255);
    px(50, 8, "blit, clipped: in the source", 255, 0, 0, 255);

    /* linear: the middle of black and white */
    {
        GLuint t = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
        static const uint8_t bw[8] = { 0, 0, 0, 255, 255, 255, 255, 255 };
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, bw);
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo[0]);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t, 0);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, 0, 0);
        glBindFramebufferEXT(GL_DRAW_FRAMEBUFFER_EXT, fbo[1]);
        p_Blit(0, 0, 2, 1, 0, 0, 64, 64, GL_COLOR_BUFFER_BIT, GL_LINEAR);
        glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, fbo[1]);
        glReadPixels(32, 8, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, c);
        val("blit, linear: the middle", c[0], 131, 8);
        glReadPixels(1, 8, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, c);
        val("blit, linear: the edge", c[0], 0, 2);
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo[0]);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, src, 0);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, ds, 0);
        glDeleteTextures(1, &t);
    }

    /* depth, which goes by NEAREST alone */
    glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, fbo[0]);
    glBindFramebufferEXT(GL_DRAW_FRAMEBUFFER_EXT, fbo[1]);
    p_Blit(0, 0, 64, 64, 0, 0, 64, 64, GL_DEPTH_BUFFER_BIT, GL_LINEAR);
    val("a blit of depth by LINEAR", (double)glGetError(), GL_INVALID_OPERATION, 0);
    p_Blit(0, 0, 64, 64, 0, 0, 64, 64, GL_DEPTH_BUFFER_BIT, GL_NEAREST);
    glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, fbo[1]);
    glReadPixels(8, 8, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);
    val("a blit of depth", z, 0.25, 0.002);

    fb_back();
    glDeleteFramebuffersEXT(2, fbo);
    glDeleteTextures(1, &src); glDeleteTextures(1, &dst); glDeleteTextures(1, &ds); glDeleteTextures(1, &dd);
    GL_CLEAN("blits");
    plain();
}

static void test_pbo(void)
{
    PFNGLGENBUFFERSPROC gen = (PFNGLGENBUFFERSPROC)wglGetProcAddress("glGenBuffers");
    PFNGLBINDBUFFERPROC bind = (PFNGLBINDBUFFERPROC)wglGetProcAddress("glBindBuffer");
    PFNGLBUFFERDATAPROC data = (PFNGLBUFFERDATAPROC)wglGetProcAddress("glBufferData");
    PFNGLMAPBUFFERPROC map = (PFNGLMAPBUFFERPROC)wglGetProcAddress("glMapBuffer");
    PFNGLUNMAPBUFFERPROC unmap = (PFNGLUNMAPBUFFERPROC)wglGetProcAddress("glUnmapBuffer");
    PFNGLDELETEBUFFERSPROC del = (PFNGLDELETEBUFFERSPROC)wglGetProcAddress("glDeleteBuffers");
    static const uint8_t px2[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 255, 0, 255, 0, 255 };
    GLuint b, t;
    GLint v = -1;
    uint8_t *m;
    gen(1, &b);

    /* pack */
    bind(GL_PIXEL_PACK_BUFFER, b);
    data(GL_PIXEL_PACK_BUFFER, 16, NULL, GL_STREAM_READ);
    glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &v);
    val("pack binding", v, b, 0);
    glClearColor(10 / 255.0f, 20 / 255.0f, 30 / 255.0f, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glReadPixels(5, 5, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, (void *)(intptr_t)8);
    m = (uint8_t *)map(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
    g_checks++;
    if (!m) { g_bad++; printf("FAIL a pack buffer does not map\n"); } else { rgba_is("glReadPixels into a buffer", m + 8, 10, 20, 30, 255); unmap(GL_PIXEL_PACK_BUFFER); }
    glReadPixels(0, 0, 4, 4, GL_RGBA, GL_UNSIGNED_BYTE, (void *)(intptr_t)8);
    val("glReadPixels past the end of a buffer", (double)glGetError(), GL_INVALID_OPERATION, 0);
    bind(GL_PIXEL_PACK_BUFFER, 0);

    /* unpack */
    bind(GL_PIXEL_UNPACK_BUFFER, b);
    data(GL_PIXEL_UNPACK_BUFFER, 16, px2, GL_STATIC_DRAW);
    glGetIntegerv(GL_PIXEL_UNPACK_BUFFER_BINDING, &v);
    val("unpack binding", v, b, 0);
    t = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const void *)(intptr_t)8);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_TEXTURE_2D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    quad(0, 0, 64, 64, 0);
    glDisable(GL_TEXTURE_2D);
    px(16, 16, "glTexImage2D from a buffer: first texel", 255, 0, 0, 255);
    px(48, 16, "glTexImage2D from a buffer: second texel", 0, 255, 0, 255);
    glRasterPos2i(100, 100);
    glDrawPixels(2, 1, GL_RGBA, GL_UNSIGNED_BYTE, (const void *)(intptr_t)8);
    px(100, 100, "glDrawPixels from a buffer", 255, 0, 0, 255);
    px(101, 100, "glDrawPixels from a buffer", 0, 255, 0, 255);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const void *)(intptr_t)8);
    val("glTexImage2D past the end of a buffer", (double)glGetError(), GL_INVALID_OPERATION, 0);
    {   /* a compressed image from a buffer: one DXT1 block, all red */
        static const uint8_t block[8] = { 0x00, 0xF8, 0x00, 0xF8, 0, 0, 0, 0 };
        PFNGLCOMPRESSEDTEXIMAGE2DPROC cimg = (PFNGLCOMPRESSEDTEXIMAGE2DPROC)wglGetProcAddress("glCompressedTexImage2D");
        data(GL_PIXEL_UNPACK_BUFFER, 8, block, GL_STATIC_DRAW);
        cimg(GL_TEXTURE_2D, 0, GL_COMPRESSED_RGB_S3TC_DXT1_EXT, 4, 4, 0, 8, (const void *)(intptr_t)0);
        glClear(GL_COLOR_BUFFER_BIT);
        glEnable(GL_TEXTURE_2D);
        quad(0, 0, 64, 64, 0);
        glDisable(GL_TEXTURE_2D);
        px(16, 16, "a compressed image from a buffer", 255, 0, 0, 255);
    }
    bind(GL_PIXEL_UNPACK_BUFFER, 0);
    glDeleteTextures(1, &t);
    del(1, &b);
    GL_CLEAN("pixel buffers");
    plain();
}

#define LODV "#extension GL_ARB_shader_texture_lod : require\n"

static void test_lod_fs(void)
{
    GLuint t, p, s;
    GLint ok = 1;
    int i, l;
    static const uint8_t COL[3][4] = { { 255, 0, 0, 255 }, { 0, 255, 0, 255 }, { 0, 0, 255, 255 } };
    uint8_t c[4 * 16 * 4];
    const char *src = "uniform sampler2D s; void main() { gl_FragColor = texture2DLod(s, vec2(0.5), 0.0); }\n";
    /* §8.7: for vertex shaders, unless the extension is enabled */
    s = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    val("a level in a fragment shader without the extension", ok, 0, 0);
    glDeleteShader(s);

    t = fb_tex(GL_TEXTURE_2D, GL_NEAREST_MIPMAP_NEAREST);
    chain_tex(GL_NEAREST_MIPMAP_NEAREST);
    p = prog2(PLAIN_VS, LODV "uniform sampler2D s; uniform float l; void main() { gl_FragColor = texture2DLod(s, gl_TexCoord[0].st, l); }\n");
    for (i = 0; i < 3; i++) {
        glUniform1f(glGetUniformLocation(p, "l"), (float)i);
        q64();
        px(32, 32, "texture2DLod", COL[i][0], COL[i][1], COL[i][2], 255);
    }
    done(p);
    p = prog2(PLAIN_VS, LODV "uniform sampler2D s; void main() { gl_FragColor = texture2DProjLod(s, vec4(gl_TexCoord[0].st * 2.0, 0.0, 2.0), 1.0); }\n");
    q64();
    px(32, 32, "texture2DProjLod", 0, 255, 0, 255);
    done(p);
    p = prog2(PLAIN_VS, LODV "uniform sampler2D s; void main() { gl_FragColor = texture2DGradARB(s, gl_TexCoord[0].st, vec2(0.5, 0.0), vec2(0.0)); }\n");
    q64();
    px(32, 32, "texture2DGradARB: 2 texels a pixel is level 1", 0, 255, 0, 255);
    done(p);
    p = prog2(PLAIN_VS, LODV "uniform sampler2D s; void main() { gl_FragColor = texture2DProjGradARB(s, vec3(gl_TexCoord[0].st, 1.0), vec2(1.0, 0.0), vec2(0.0)); }\n");
    q64();
    px(32, 32, "texture2DProjGradARB: 4 texels a pixel is level 2", 0, 0, 255, 255);
    done(p);
    p = prog2(PLAIN_VS, LODV "uniform sampler2D s; vec4 g(sampler2D q, vec2 c, vec2 d) { return texture2DGradARB(q, c, d, vec2(0.0)); }\nvoid main() { gl_FragColor = g(s, gl_TexCoord[0].st, vec2(0.5, 0.0)); }\n");
    q64();
    px(32, 32, "texture2DGradARB through a function", 0, 255, 0, 255);
    done(p);
    {
        static const GLint u00[2] = { 0, 0 };
        p = prog2(PLAIN_VS, LODV "uniform sampler2D a[2]; void main() { vec4 r = vec4(0.0); for (int i = 0; i < 2; i++) r += 0.5 * texture2DGradARB(a[i], gl_TexCoord[0].st, vec2(0.5, 0.0), vec2(0.0)); gl_FragColor = r; }\n");
        glUniform1iv(glGetUniformLocation(p, "a"), 2, u00);
        q64();
        px(32, 32, "texture2DGradARB on an array, by a loop's index", 0, 255, 0, 255);
        done(p);
    }

    /* the same through the filter in the program: a border */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    p = prog2(PLAIN_VS, LODV "uniform sampler2D s; void main() { gl_FragColor = texture2DLod(s, gl_TexCoord[0].st, 1.0); }\n");
    q64();
    px(32, 32, "texture2DLod, a sampler the program filters", 0, 255, 0, 255);
    done(p);
    p = prog2(PLAIN_VS, LODV "uniform sampler2D s; void main() { gl_FragColor = texture2DGradARB(s, gl_TexCoord[0].st, vec2(1.0, 0.0), vec2(0.0)); }\n");
    q64();
    px(32, 32, "texture2DGradARB, a sampler the program filters", 0, 0, 255, 255);
    done(p);
    p = prog2(PLAIN_VS, LODV "uniform sampler2D s; vec4 g(sampler2D q, vec2 c) { return texture2DLod(q, c, 1.0); }\nvoid main() { gl_FragColor = g(s, gl_TexCoord[0].st); }\n");
    q64();
    px(32, 32, "texture2DLod through a function, a sampler the program filters", 0, 255, 0, 255);
    done(p);
    glDeleteTextures(1, &t);

    /* an incomplete texture reads (0, 0, 0, 1) */
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    p = prog2(PLAIN_VS, LODV "uniform sampler2D s; void main() { gl_FragColor = texture2DLod(s, vec2(0.5), 0.0) + vec4(0.0, 0.5, 0.0, 0.0); }\n");
    q64();
    px(32, 32, "texture2DLod of an incomplete texture", 0, 128, 0, 255);
    done(p);
    glDeleteTextures(1, &t);

    /* a cube */
    t = fb_tex(GL_TEXTURE_CUBE_MAP, GL_NEAREST_MIPMAP_NEAREST);
    for (l = 0; l < 3; l++) {
        for (i = 0; i < 4 * 16; i += 4) memcpy(c + i, COL[l], 4);
        for (i = 0; i < 6; i++) glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + (GLenum)i, l, GL_RGBA, 4 >> l, 4 >> l, 0, GL_RGBA, GL_UNSIGNED_BYTE, c);
    }
    p = prog2(PLAIN_VS, LODV "uniform samplerCube s; void main() { gl_FragColor = textureCubeLod(s, vec3(1.0, gl_TexCoord[0].st - 0.5), 2.0); }\n");
    q64();
    px(32, 32, "textureCubeLod", 0, 0, 255, 255);
    done(p);
    p = prog2(PLAIN_VS, LODV "uniform samplerCube s; void main() { gl_FragColor = textureCubeGradARB(s, vec3(1.0, gl_TexCoord[0].st - 0.5), vec3(0.0, 1.0, 0.0), vec3(0.0)); }\n");
    q64();
    px(32, 32, "textureCubeGradARB: 2 texels a pixel is level 1", 0, 255, 0, 255);
    done(p);
    glDeleteTextures(1, &t);

    /* a volume */
    t = fb_tex(GL_TEXTURE_3D, GL_NEAREST_MIPMAP_NEAREST);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    for (l = 0; l < 3; l++) {
        for (i = 0; i < 4 * 16 * 4; i += 4) memcpy(c + i, COL[l], 4);
        glTexImage3D(GL_TEXTURE_3D, l, GL_RGBA, 4 >> l, 4 >> l, 4 >> l, 0, GL_RGBA, GL_UNSIGNED_BYTE, c);
    }
    p = prog2(PLAIN_VS, LODV "uniform sampler3D s; void main() { gl_FragColor = texture3DLod(s, vec3(gl_TexCoord[0].st, 0.5), 1.0); }\n");
    q64();
    px(32, 32, "texture3DLod", 0, 255, 0, 255);
    done(p);
    p = prog2(PLAIN_VS, LODV "uniform sampler3D s; void main() { gl_FragColor = texture3DGradARB(s, vec3(gl_TexCoord[0].st, 0.5), vec3(1.0, 0.0, 0.0), vec3(0.0)); }\n");
    q64();
    px(32, 32, "texture3DGradARB: 4 texels a pixel is level 2", 0, 0, 255, 255);
    done(p);
    glDeleteTextures(1, &t);
    plain();
}

/* storage: B's format closest to the internal format (RGB565, RGBA4444, RGBA5551, RGBA8), what that format keeps of a texel, 16-bit framebuffers and depth */
static void test_storage(void)
{
    static const struct { GLenum f; int fill, r, g, b, a, bits[6]; } FMT[] = {      /* bits: red green blue alpha luminance intensity */
        { GL_RGBA4, 25, 17, 17, 17, 17, { 4, 4, 4, 4, 0, 0 } }, { GL_RGB5_A1, 4, 0, 0, 0, 0, { 5, 5, 5, 1, 0, 0 } }, { GL_RGB5, 4, 0, 4, 0, 255, { 5, 6, 5, 0, 0, 0 } },
        { GL_R3_G3_B2, 4, 0, 4, 0, 255, { 5, 6, 5, 0, 0, 0 } }, { GL_RGBA8, 25, 25, 25, 25, 25, { 8, 8, 8, 8, 0, 0 } }, { GL_RGB10_A2, 25, 25, 25, 25, 25, { 8, 8, 8, 8, 0, 0 } },
        { GL_LUMINANCE4, 25, 17, 0, 0, 255, { 0, 0, 0, 0, 4, 0 } }, { GL_ALPHA4, 25, 0, 0, 0, 17, { 0, 0, 0, 4, 0, 0 } }, { GL_INTENSITY4, 25, 17, 0, 0, 255, { 0, 0, 0, 0, 0, 4 } },
        { GL_LUMINANCE8, 25, 25, 0, 0, 255, { 0, 0, 0, 0, 8, 0 } },
    };
    static const GLenum Q[6] = { GL_TEXTURE_RED_SIZE, GL_TEXTURE_GREEN_SIZE, GL_TEXTURE_BLUE_SIZE, GL_TEXTURE_ALPHA_SIZE, GL_TEXTURE_LUMINANCE_SIZE, GL_TEXTURE_INTENSITY_SIZE };
    uint8_t tx[16 * 16 * 4], out[16 * 16 * 4];
    GLuint t, t4, fbo, rb, rbd, p;
    GLint v;
    int i, q;
    float z = 0.0f;
    PIXELFORMATDESCRIPTOR pfd;
    char what[128];
    for (i = 0; i < (int)(sizeof FMT / sizeof FMT[0]); i++) {
        memset(tx, FMT[i].fill, sizeof tx);
        t = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, (GLint)FMT[i].f, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, tx);
        for (q = 0; q < 6; q++) {
            glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, Q[q], &v);
            snprintf(what, sizeof what, "internal format 0x%04X: bits of size query 0x%04X", FMT[i].f, Q[q]);
            val(what, v, FMT[i].bits[q], 0);
        }
        memset(out, 0xEE, sizeof out);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, out);
        snprintf(what, sizeof what, "internal format 0x%04X: what its storage keeps of %d", FMT[i].f, FMT[i].fill);
        rgba_is(what, out, FMT[i].r, FMT[i].g, FMT[i].b, FMT[i].a);
        glDeleteTextures(1, &t);
    }
    /* the TMU samples RGBA4444 */
    memset(tx, 25, sizeof tx);
    t = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA4, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, tx);
    glEnable(GL_TEXTURE_2D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    quad(0, 0, 16, 16, 0);
    glDisable(GL_TEXTURE_2D);
    px(8, 8, "an RGBA4 texture drawn", 17, 17, 17, 17);
    glDeleteTextures(1, &t);
    /* GL_UNSIGNED_SHORT_5_6_5 data in an unsized RGB texture stays 5_6_5 */
    {
        static const uint16_t w565[16] = { 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234 };
        uint16_t back[16];
        t = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 4, 4, 0, GL_RGB, GL_UNSIGNED_SHORT_5_6_5, w565);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_GREEN_SIZE, &v);
        val("unsized RGB from 5_6_5 data: 6 bits of green", v, 6, 0);
        glPixelStorei(GL_PACK_ALIGNMENT, 2);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_SHORT_5_6_5, back);
        glPixelStorei(GL_PACK_ALIGNMENT, 4);
        val("and the 5_6_5 word comes back as it went", back[5], 0x1234, 0);
        glDeleteTextures(1, &t);
    }
    /* a 3D texture in RGBA4444 */
    t = fb_tex(GL_TEXTURE_3D, GL_NEAREST);
    p_TexImage3D(GL_TEXTURE_3D, 0, GL_RGBA4, 2, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, tx);
    memset(out, 0xEE, sizeof out);
    glGetTexImage(GL_TEXTURE_3D, 0, GL_RGBA, GL_UNSIGNED_BYTE, out);
    rgba_is("a 3D RGBA4 texture keeps 4 bits", out, 17, 17, 17, 17);
    glEnable(GL_TEXTURE_3D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    quad(0, 0, 16, 16, 0);
    glDisable(GL_TEXTURE_3D);
    px(8, 8, "a 3D RGBA4 texture drawn", 17, 17, 17, 17);
    glDeleteTextures(1, &t);

    /* a 565 renderbuffer, a logic op on its words, a 16-bit depth buffer */
    glDisable(GL_DITHER);
    glGenFramebuffersEXT(1, &fbo);
    p_GenRb(1, &rb);
    p_BindRb(GL_RENDERBUFFER_EXT, rb);
    p_RbStore(GL_RENDERBUFFER_EXT, GL_RGB5, 32, 32);
    p_RbQ(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_GREEN_SIZE_EXT, &v);
    val("an RGB5 renderbuffer: 6 bits of green", v, 6, 0);
    p_RbQ(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_ALPHA_SIZE_EXT, &v);
    val("an RGB5 renderbuffer: no alpha", v, 0, 0);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    p_FbRb(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, rb);
    fb_done("an RGB5 renderbuffer");
    glGetIntegerv(GL_GREEN_BITS, &v);
    val("GL_GREEN_BITS of it", v, 6, 0);
    fb_view(32, 32);
    glClearColor(4 / 255.0f, 4 / 255.0f, 4 / 255.0f, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    px(5, 5, "a clear of a 565 renderbuffer keeps 5 6 5 bits", 0, 4, 0, 255);
    glClearColor(16 / 31.0f, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_COLOR_LOGIC_OP);
    glLogicOp(GL_XOR);
    glColor3f(8 / 31.0f, 0, 0);
    quad(0, 0, 32, 32, 0);
    glDisable(GL_COLOR_LOGIC_OP);
    glLogicOp(GL_COPY);
    px(5, 5, "GL_XOR on a 565 target: 16 ^ 8 = 24 of 31", 197, 0, 0, 255);
    p_GenRb(1, &rbd);
    p_BindRb(GL_RENDERBUFFER_EXT, rbd);
    p_RbStore(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT16, 32, 32);
    p_RbQ(GL_RENDERBUFFER_EXT, GL_RENDERBUFFER_DEPTH_SIZE_EXT, &v);
    val("a DEPTH_COMPONENT16 renderbuffer: 16 bits", v, 16, 0);
    p_FbRb(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, rbd);
    fb_done("RGB5 and DEPTH_COMPONENT16");
    glGetIntegerv(GL_DEPTH_BITS, &v);
    val("GL_DEPTH_BITS of it", v, 16, 0);
    glClearDepth(0.25);
    glClear(GL_DEPTH_BUFFER_BIT);
    glReadPixels(5, 5, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);
    val("a depth cleared at 16 bits", z, 16384.0 / 65535.0, 1e-7);
    glClearDepth(0.50001);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glColor3f(1, 0, 0);
    quad(0, 0, 16, 32, -0.00001f);                              /* window z 0.500005: 32768 at 16 bits, as the clear */
    quad(16, 0, 32, 32, 0.0002f);                               /* 0.4999: nearer at any precision */
    glDisable(GL_DEPTH_TEST);
    glClearDepth(1.0);
    px(8, 16, "a 16-bit depth buffer: equal at 16 bits, so LESS fails", 0, 0, 0, 255);
    px(24, 16, "a 16-bit depth buffer: nearer passes", 255, 0, 0, 255);
    fb_back();

    /* render into an RGBA4444 texture, blit it to the window's RGBA8 */
    t4 = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA4, 16, 16, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    p_FbRb(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, 0);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t4, 0);
    fb_done("an RGBA4 texture as a target");
    fb_view(16, 16);
    glClearColor(5 / 15.0f, 10 / 15.0f, 1, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    fb_back();
    memset(out, 0xEE, sizeof out);
    glBindTexture(GL_TEXTURE_2D, t4);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, out);
    rgba_is("rendered into RGBA4444", out, 85, 170, 255, 0);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, fbo);
    glBindFramebufferEXT(GL_DRAW_FRAMEBUFFER_EXT, 0);
    p_Blit(0, 0, 16, 16, 0, 0, 16, 16, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
    px(8, 8, "blit from RGBA4444 to RGBA8", 85, 170, 255, 0);
    glDeleteTextures(1, &t4);
    glDeleteFramebuffersEXT(1, &fbo);
    {
        PFNGLDELETERENDERBUFFERSEXTPROC del = (PFNGLDELETERENDERBUFFERSEXTPROC)wglGetProcAddress("glDeleteRenderbuffersEXT");
        if (del) { del(1, &rb); del(1, &rbd); }
    }
    glEnable(GL_DITHER);

    /* copies take what the internal format keeps (table 3.15): red as luminance, alpha 1 for RGB */
    glClearColor(1, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    t = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, 0, 0, 16, 16, 0);
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, 0, 0, 16, 16, 0);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_TEXTURE_2D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    quad(0, 0, 16, 16, 0);
    glDisable(GL_TEXTURE_2D);
    px(8, 8, "glCopyTexImage2D into LUMINANCE: red is the luminance", 255, 255, 255, 255);
    glClearColor(0, 1, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 0, 0, 16, 16, 0);
    p = prog2(PLAIN_VS, "uniform sampler2D s;\nvoid main() { gl_FragColor = vec4(texture2D(s, gl_TexCoord[0].st).ag, 0.0, 1.0); }\n");
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    q64();
    px(8, 8, "an RGB texture copied from alpha 0 reads alpha 1", 255, 255, 0, 255);
    done(p);
    glDeleteTextures(1, &t);

    /* a pixel format of 16 bits of colour and of depth */
    memset(&pfd, 0, sizeof pfd);
    pfd.nSize = sizeof pfd; pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA; pfd.cColorBits = 16; pfd.cDepthBits = 16;
    i = ChoosePixelFormat(g_dc, &pfd);
    DescribePixelFormat(g_dc, i, sizeof pfd, &pfd);
    val("ChoosePixelFormat, 16 bits of colour: 5 6 5", pfd.cRedBits * 100 + pfd.cGreenBits * 10 + pfd.cBlueBits, 565, 0);
    val("16 bits of depth", pfd.cDepthBits, 16, 0);
    memset(&pfd, 0, sizeof pfd);
    pfd.nSize = sizeof pfd; pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA; pfd.cColorBits = 32; pfd.cDepthBits = 24; pfd.cStencilBits = 8;
    i = ChoosePixelFormat(g_dc, &pfd);
    DescribePixelFormat(g_dc, i, sizeof pfd, &pfd);
    val("32 bits, depth 24, stencil 8: what it was", pfd.cColorBits * 10000 + pfd.cDepthBits * 100 + pfd.cStencilBits, 322408, 0);
}

/* depth beside an RGB565 target whose rows are no multiple of 16 pixels: board B loads a depth row in twice the colour row's bursts (200
   pixels: 13 bursts of colour, 26 of depth = 832 bytes, more than 200 x 4) and refuses a shorter pitch. A depth-stencil renderbuffer beside
   an RGB5 renderbuffer, then a depth texture (its own rows are 800 bytes) beside an RGB5 texture, its depth read back after */
static void test_ds_rows(void)
{
    static uint8_t c[200 * 40 * 4];
    static float z[200 * 40];
    GLuint fbo, rb, rbd, t, d;
    int k, i, bad;
    glDisable(GL_DITHER);
    glGenFramebuffersEXT(1, &fbo);
    p_GenRb(1, &rb);
    p_BindRb(GL_RENDERBUFFER_EXT, rb);
    p_RbStore(GL_RENDERBUFFER_EXT, GL_RGB5, 200, 40);
    p_GenRb(1, &rbd);
    p_BindRb(GL_RENDERBUFFER_EXT, rbd);
    p_RbStore(GL_RENDERBUFFER_EXT, GL_DEPTH24_STENCIL8_EXT, 200, 40);
    t = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB5, 200, 40, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    d = fb_tex(GL_TEXTURE_2D, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, 200, 40, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, NULL);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
    for (k = 0; k < 2; k++) {
        if (!k) {
            p_FbRb(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, rb);
            p_FbRb(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, rbd);
            p_FbRb(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, rbd);
        } else {
            p_FbRb(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, 0);
            p_FbRb(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, 0);
            p_FbRb(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, 0);
            glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t, 0);
            glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, d, 0);
        }
        fb_done(k ? "an RGB5 texture 200 x 40 with a depth texture" : "RGB5 200 x 40 with DEPTH24_STENCIL8");
        fb_view(200, 40);
        glClearColor(0, 0, 0, 1); glClearDepth(1.0); glClearStencil(0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS);
        if (!k) { glEnable(GL_STENCIL_TEST); glStencilFunc(GL_ALWAYS, 1, 255); glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE); }
        glColor3f(1, 0, 0);
        quad(0, 0, 200, 40, 0);                                 /* window z 0.5 (stencil 1) everywhere */
        glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
        glColor3f(0, 1, 0);
        quad(0, 0, 200, 40, -0.5f);                             /* 0.75, behind: nowhere */
        glStencilFunc(GL_EQUAL, 1, 255);
        glColor3f(0, 0, 1);
        quad(0, 0, 100, 40, 0.5f);                              /* 0.25, in front (where the stencil is 1): the left half */
        glDisable(GL_DEPTH_TEST); glDisable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS, 0, 255);
        glColor3f(1, 1, 1);
        memset(c, 0x5A, sizeof c);
        glReadPixels(0, 0, 200, 40, GL_RGBA, GL_UNSIGNED_BYTE, c);
        for (i = bad = 0; i < 200 * 40; i++) {
            const uint8_t *q = c + 4 * i;
            int left = i % 200 < 100;
            if (abs(q[0] - (left ? 0 : 255)) > 2 || q[1] > 2 || abs(q[2] - (left ? 255 : 0)) > 2) {
                if (!bad) printf("  the first wrong pixel (%d, %d): %d %d %d\n", i % 200, i / 200, q[0], q[1], q[2]);
                bad++;
            }
        }
        val(k ? "RGB5 texture 200 wide, depth texture: pixels wrong" : "RGB5 200 wide, depth and stencil renderbuffer: pixels wrong", bad, 0, 0);
    }
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, 0, 0);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, 0, 0);
    fb_back();
    glBindTexture(GL_TEXTURE_2D, d);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, z);
    val("a depth texture 200 wide: the depth in front", z[20 * 200 + 50], 0.25, 0.002);
    val("a depth texture 200 wide: the depth behind", z[20 * 200 + 150], 0.5, 0.002);
    val("a depth texture 200 wide: its last column", z[20 * 200 + 199], 0.5, 0.002);
    glBindTexture(GL_TEXTURE_2D, 0);
    glDeleteTextures(1, &t);
    glDeleteTextures(1, &d);
    glDeleteFramebuffersEXT(1, &fbo);
    {
        PFNGLDELETERENDERBUFFERSEXTPROC del = (PFNGLDELETERENDERBUFFERSEXTPROC)wglGetProcAddress("glDeleteRenderbuffersEXT");
        if (del) { del(1, &rb); del(1, &rbd); }
    }
}

static void test_gaps(void)
{
    const char *ext = (const char *)glGetString(GL_EXTENSIONS);
    static const char *const NAMES[] = { "GL_EXT_framebuffer_blit", "GL_EXT_packed_depth_stencil", "GL_ARB_pixel_buffer_object", "GL_ARB_shader_texture_lod", NULL };
    int i;
    for (i = 0; NAMES[i]; i++) {
        g_checks++;
        if (!strstr(ext, NAMES[i])) { g_bad++; printf("FAIL %s is not in the extension string\n", NAMES[i]); }
    }
    val("no sRGB textures: the TMU has no such decode", strstr(ext, "texture_sRGB") != NULL, 0, 0);
    g_checks++;
    if (strncmp((const char *)glGetString(GL_VERSION), "2.0", 3)) { g_bad++; printf("FAIL GL_VERSION %s\n", glGetString(GL_VERSION)); }
    p_Blit = (fb_blit_t)wglGetProcAddress("glBlitFramebufferEXT");
    p_AttQ = (fb_att_t)wglGetProcAddress("glGetFramebufferAttachmentParameterivEXT");
    p_RbQ = (fb_rbq_t)wglGetProcAddress("glGetRenderbufferParameterivEXT");
    p_GenRb = (fb_grb_t)wglGetProcAddress("glGenRenderbuffersEXT");
    p_BindRb = (fb_brb_t)wglGetProcAddress("glBindRenderbufferEXT");
    p_RbStore = (fb_rbs_t)wglGetProcAddress("glRenderbufferStorageEXT");
    p_FbRb = (fb_frb_t)wglGetProcAddress("glFramebufferRenderbufferEXT");
    p_FbTex3 = (fb_ft3_t)wglGetProcAddress("glFramebufferTexture3DEXT");
    g_checks++;
    if (!p_Blit || !p_AttQ || !p_RbQ || !p_GenRb || !p_BindRb || !p_RbStore || !p_FbRb || !p_FbTex3) { g_bad++; printf("FAIL a framebuffer entry point is missing\n"); return; }
    SCENE(test_units());
    SCENE(test_fbo_targets());
    SCENE(test_band_tail());
    SCENE(test_blit());
    SCENE(test_pbo());
    SCENE(test_lod_fs());
    SCENE(test_storage());
    SCENE(test_ds_rows());
}

static void test_gl20(void)
{
#define X(T, n) p_##n = (T)wglGetProcAddress("gl" #n);
    P20(X)
#undef X
#define X(T, n) if (!p_##n) { printf("FAIL gl%s is missing\n", #n); g_bad++; return; }
    P20(X)
#undef X
    if (!gl15_procs()) return;
    SCENE(test_glsl_basic());
    SCENE(test_glsl_attribs());
    SCENE(test_glsl_tex());
    SCENE(test_glsl_pipeline());
    SCENE(test_glsl_host());
    SCENE(test_glsl_arb());
    SCENE(test_gaps());
}

static LRESULT CALLBACK wndproc(HWND h, UINT m, WPARAM w, LPARAM l)
{
    if (m == WM_GETMINMAXINFO) {                                /* a window can be larger than this PC's desktop */
        ((MINMAXINFO *)l)->ptMaxTrackSize.x = 4000;
        ((MINMAXINFO *)l)->ptMaxTrackSize.y = 4000;
        return 0;
    }
    return DefWindowProcA(h, m, w, l);
}

int main(int argc, char **argv)
{
    WNDCLASSA wc;
    HWND hwnd;
    HGLRC rc;
    PIXELFORMATDESCRIPTOR pfd;
    int pf, spin = 0;
    RECT r = { 0, 0, W, H };
    if (argc > 2 && !strcmp(argv[1], "--spin")) spin = atoi(argv[2]);
    setvbuf(stdout, NULL, _IONBF, 0);                           /* what was printed stays if a scene crashes */
    memset(&wc, 0, sizeof wc);
    wc.lpfnWndProc = wndproc; wc.lpszClassName = "gl_test"; wc.hInstance = GetModuleHandleA(NULL); wc.style = CS_OWNDC;
    RegisterClassA(&wc);
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    hwnd = CreateWindowA("gl_test", "gl_test", WS_OVERLAPPEDWINDOW | WS_VISIBLE, 40, 40, r.right - r.left, r.bottom - r.top, NULL, NULL, wc.hInstance, NULL);
    g_dc = GetDC(hwnd);
    memset(&pfd, 0, sizeof pfd);
    pfd.nSize = sizeof pfd; pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA; pfd.cColorBits = 32; pfd.cDepthBits = 24; pfd.cStencilBits = 8;
    pf = ChoosePixelFormat(g_dc, &pfd);
    printf("ChoosePixelFormat -> %d, DescribePixelFormat count %d\n", pf, DescribePixelFormat(g_dc, pf, sizeof pfd, &pfd));
    if (!pf || !SetPixelFormat(g_dc, pf, &pfd)) { printf("FAIL SetPixelFormat: %lu\n", GetLastError()); return 1; }
    rc = wglCreateContext(g_dc);
    if (!rc || !wglMakeCurrent(g_dc, rc)) { printf("FAIL context: %lu\n", GetLastError()); return 1; }
    SCENE(tests());
    SCENE(test_kept());
    SCENE(test_locked_arrays());
    SCENE(test_tex1d());
    SCENE(test_pixel_transfer());
    SCENE(test_stipple());
    SCENE(test_eval());
    SCENE(test_select());
    SCENE(test_feedback());
    SCENE(test_accum());
    SCENE(test_logic());
    SCENE(test_ff_alpha());
    SCENE(test_index_state());
    SCENE(test_raster());
    SCENE(test_polygon_mode());
    SCENE(test_ds_pixels());
    SCENE(test_readback());
    SCENE(test_border());
    SCENE(test_smooth());
    SCENE(test_gl14());
    SCENE(test_shadow());
    SCENE(test_tex3d());
    SCENE(test_queries());
    SCENE(test_gl20());
    SCENE(test_tex3d_hw());
    SCENE(test_wide(hwnd, 1600, 1000));
    SCENE(test_wide(hwnd, 1000, 1200));
    SCENE(window_back(hwnd));
    printf("%d checks, %d failed\n", g_checks, g_bad);
    if (spin) {                                                 /* a moving scene for the monitor: fps of the whole GL path */
        DWORD t0 = GetTickCount();
        int f = 0;
        glDisable(GL_TEXTURE_2D);
        while (GetTickCount() - t0 < (DWORD)spin * 1000) {
            int i;
            glClearColor(0.1f, 0.1f, 0.2f, 1);
            glClear(GL_COLOR_BUFFER_BIT);
            glMatrixMode(GL_MODELVIEW); glLoadIdentity();
            glTranslatef(320, 240, 0); glRotatef((float)f, 0, 0, 1);
            for (i = 0; i < 12; i++) {
                glRotatef(30, 0, 0, 1);
                glColor3f((i & 1) ? 1.0f : 0.2f, (i & 2) ? 1.0f : 0.2f, (i & 4) ? 1.0f : 0.2f);
                quad(60, -20, 200, 20, 0);
            }
            SwapBuffers(g_dc);
            f++;
        }
        printf("spin: %d frames in %d s = %.1f fps\n", f, spin, (double)f / spin);
    }
    wglMakeCurrent(NULL, NULL);
    wglDeleteContext(rc);
    if (!g_bad) printf("OK gl_test\n");
    return g_bad ? 1 : 0;
}
