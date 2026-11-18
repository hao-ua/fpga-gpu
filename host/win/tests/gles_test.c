/* gles_test — libEGL.dll + libGLESv2.dll through the real API on the boards: scenes with known pixels, read back with glReadPixels, plus the errors the spec requires.
 * Varyings declared in another order than the vertex stage's, attribute binding, every uniform kind, every texture format, completeness, mipmaps (generated, and uploaded bottom-up), cube maps, framebuffer objects,
 * depth, stencil, blending, all primitive modes, indices of all three sizes and beyond 16 bits, copies, reads, two contexts.
 * Tolerances are +-2 of 255: the shaders compute in FP24. */
#define _GDI32_
#define GL_GLEXT_PROTOTYPES
#include <windows.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GL_VERTEX_ATTRIB_ARRAY_DIVISOR_ANGLE_T 0x88FE
#ifndef GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS
#define GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS 0x8CD9
#endif

typedef void *EGLDisplay, *EGLConfig, *EGLSurface, *EGLContext;
typedef int EGLint;
#define EGLAPI __stdcall
EGLDisplay EGLAPI eglGetDisplay(void *);
int EGLAPI eglInitialize(EGLDisplay, EGLint *, EGLint *);
int EGLAPI eglChooseConfig(EGLDisplay, const EGLint *, EGLConfig *, EGLint, EGLint *);
EGLSurface EGLAPI eglCreateWindowSurface(EGLDisplay, EGLConfig, HWND, const EGLint *);
EGLSurface EGLAPI eglCreatePbufferSurface(EGLDisplay, EGLConfig, const EGLint *);
int EGLAPI eglDestroySurface(EGLDisplay, EGLSurface);
EGLContext EGLAPI eglCreateContext(EGLDisplay, EGLConfig, EGLContext, const EGLint *);
int EGLAPI eglMakeCurrent(EGLDisplay, EGLSurface, EGLSurface, EGLContext);
int EGLAPI eglSwapBuffers(EGLDisplay, EGLSurface);
int EGLAPI eglSwapInterval(EGLDisplay, EGLint);
int EGLAPI eglQuerySurface(EGLDisplay, EGLSurface, EGLint, EGLint *);
int EGLAPI eglGetConfigAttrib(EGLDisplay, EGLConfig, EGLint, EGLint *);
EGLint EGLAPI eglGetError(void);
void *EGLAPI eglGetProcAddress(const char *);

#define W 640
#define H 480

static int g_bad, g_checks;
static EGLDisplay g_dpy;
static EGLSurface g_surf;
static EGLConfig g_cfg;
static EGLContext g_ctx;

static void fail(const char *fmt, const char *what, int a, int b) { g_bad++; printf("FAIL %s: ", what); printf(fmt, a, b); printf("\n"); }
static void ok(int cond, const char *what) { g_checks++; if (!cond) { g_bad++; printf("FAIL %s\n", what); } }
static void err(GLenum want, const char *what) { GLenum e = glGetError(); g_checks++; if (e != want) fail("error 0x%x, expected 0x%x", what, (int)e, (int)want); }
static void ieq(int got, int want, const char *what) { g_checks++; if (got != want) fail("%d, expected %d", what, got, want); }

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

static GLuint shader(GLenum type, const char *src)
{
    GLuint s = glCreateShader(type);
    GLint okk = 0;
    char log[600];
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    glGetShaderiv(s, GL_COMPILE_STATUS, &okk);
    g_checks++;
    if (!okk) { glGetShaderInfoLog(s, sizeof log, NULL, log); g_bad++; printf("FAIL compile: %s\n%s\n", log, src); }
    return s;
}

static GLuint link_prog(GLuint p, GLuint v, GLuint f)
{
    GLint okk = 0;
    char log[800];
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glGetProgramiv(p, GL_LINK_STATUS, &okk);
    g_checks++;
    if (!okk) { glGetProgramInfoLog(p, sizeof log, NULL, log); g_bad++; printf("FAIL link: %s\n", log); }
    glDeleteShader(v);
    glDeleteShader(f);
    return p;
}

static GLuint prog(const char *vs, const char *fs) { return link_prog(glCreateProgram(), shader(GL_VERTEX_SHADER, vs), shader(GL_FRAGMENT_SHADER, fs)); }

static const char VS[] = "attribute vec2 a_pos; attribute vec2 a_uv; varying vec2 v_uv; void main() { v_uv = a_uv; gl_Position = vec4(a_pos, 0.0, 1.0); }\n";
static const char VS_Z[] = "attribute vec2 a_pos; uniform float z; void main() { gl_Position = vec4(a_pos, z, 1.0); }\n";
static const char FS_TEX[] = "precision mediump float; varying vec2 v_uv; uniform sampler2D t; void main() { gl_FragColor = texture2D(t, v_uv); }\n";
static const char FS_COL[] = "precision mediump float; uniform vec4 c; void main() { gl_FragColor = c; }\n";
static GLuint g_tex_prog, g_col_prog, g_zcol_prog;

/* a rectangle in NDC with client arrays, through whatever attribute names the program has */
static void rect(GLuint p, float x0, float y0, float x1, float y1)
{
    GLfloat v[16] = { x0, y0, 0, 0,  x1, y0, 1, 0,  x0, y1, 0, 1,  x1, y1, 1, 1 };
    GLint a = glGetAttribLocation(p, "a_pos"), u = glGetAttribLocation(p, "a_uv");
    glUseProgram(p);
    if (a >= 0) { glVertexAttribPointer((GLuint)a, 2, GL_FLOAT, GL_FALSE, 16, v); glEnableVertexAttribArray((GLuint)a); }
    if (u >= 0) { glVertexAttribPointer((GLuint)u, 2, GL_FLOAT, GL_FALSE, 16, v + 2); glEnableVertexAttribArray((GLuint)u); }
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    if (a >= 0) glDisableVertexAttribArray((GLuint)a);
    if (u >= 0) glDisableVertexAttribArray((GLuint)u);
}
static void full(GLuint p) { rect(p, -1, -1, 1, 1); }

static void color(float r, float g, float b, float a) { glUseProgram(g_col_prog); glUniform4f(glGetUniformLocation(g_col_prog, "c"), r, g, b, a); }
static void zcolor(float z, float r, float g, float b, float a)
{
    glUseProgram(g_zcol_prog);
    glUniform4f(glGetUniformLocation(g_zcol_prog, "c"), r, g, b, a);
    glUniform1f(glGetUniformLocation(g_zcol_prog, "z"), z);
}

static void clear_to(float r, float g, float b, float a) { glClearColor(r, g, b, a); glClear(GL_COLOR_BUFFER_BIT); }

static GLuint make_tex(GLenum format, GLenum type, int w, int h, const void *data, GLenum filter, GLenum wrap)
{
    GLuint t;
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, (GLint)format, w, h, 0, format, type, data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, (GLint)filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, (GLint)wrap);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, (GLint)wrap);
    return t;
}

static void show_tex(GLuint t, const char *what, int r, int g, int b, int a)
{
    glBindTexture(GL_TEXTURE_2D, t);
    full(g_tex_prog);
    px(W / 2, H / 2, what, r, g, b, a);
    glDeleteTextures(1, &t);
}

static void test_varyings(void)
{
    static const char vs[] = "attribute vec2 a_pos; varying vec4 v_a; varying vec4 v_b; void main() { v_a = vec4(1.0, 0.0, 0.0, 1.0); v_b = vec4(0.0, 1.0, 0.0, 1.0); gl_Position = vec4(a_pos, 0.0, 1.0); }\n";
    GLuint p = prog(vs, "precision mediump float; varying vec4 v_b; varying vec4 v_a; void main() { gl_FragColor = v_a; }\n");
    full(p);
    px(100, 100, "varyings: other order, v_a", 255, 0, 0, 255);
    glDeleteProgram(p);
    p = prog(vs, "precision mediump float; varying vec4 v_b; void main() { gl_FragColor = v_b; }\n");
    full(p);
    px(100, 100, "varyings: only the second one", 0, 255, 0, 255);
    glDeleteProgram(p);
    p = glCreateProgram();
    link_prog(p, shader(GL_VERTEX_SHADER, VS), shader(GL_FRAGMENT_SHADER, FS_COL));
    glDeleteProgram(p);
    /* what only an unread varying needed goes with it: its attribute, uniform and constants; the slots and uniforms of the rest stay */
    p = prog("attribute vec2 a_pos; attribute vec2 a_uv; uniform vec4 u_only; uniform vec4 u_kept; varying vec4 v_a; varying vec4 v_b; varying vec4 v_c;"
             " void main() { v_a = u_only * vec4(a_uv, 3.0, 5.0); v_b = u_kept * 0.5; v_c = vec4(0.25, 0.75, 1.0, 1.0); gl_Position = vec4(a_pos, 0.0, 1.0); }\n",
             "precision mediump float; varying vec4 v_c; varying vec4 v_b; void main() { gl_FragColor = vec4(v_b.x, v_c.y, v_b.z, 1.0); }\n");
    glUseProgram(p);
    glUniform4f(glGetUniformLocation(p, "u_only"), 9.0f, 9.0f, 9.0f, 9.0f);
    glUniform4f(glGetUniformLocation(p, "u_kept"), 1.0f, 0.0f, 0.5f, 1.0f);
    err(GL_NO_ERROR, "the uniform of an unread varying");
    full(p);
    px(100, 100, "an unread varying and what only it needed", 128, 191, 64, 255);
    glDeleteProgram(p);
    /* declared and unused needs no partner in the vertex stage (Cocos2d's u_color shaders) */
    p = prog("attribute vec2 a_pos; void main() { gl_Position = vec4(a_pos, 0.0, 1.0); }\n", "precision mediump float; varying vec4 v_unused; void main() { gl_FragColor = vec4(0.0, 0.0, 1.0, 1.0); }\n");
    full(p);
    px(100, 100, "an unused varying without a partner", 0, 0, 255, 255);
    glDeleteProgram(p);
    {   /* a varying nobody writes must not link */
        GLint st = 1;
        p = glCreateProgram();
        glAttachShader(p, shader(GL_VERTEX_SHADER, VS));
        glAttachShader(p, shader(GL_FRAGMENT_SHADER, "precision mediump float; varying vec4 v_nothing; void main() { gl_FragColor = v_nothing; }\n"));
        glLinkProgram(p);
        glGetProgramiv(p, GL_LINK_STATUS, &st);
        ieq(st, 0, "link: unwritten varying fails");
        glUseProgram(p);
        err(GL_INVALID_OPERATION, "glUseProgram of an unlinked program");
        glDeleteProgram(p);
    }
}

static void test_attributes(void)
{
    static const GLfloat q[8] = { -1, -1, 1, -1, -1, 1, 1, 1 };
    GLuint p = glCreateProgram();
    glBindAttribLocation(p, 5, "a_pos");
    glBindAttribLocation(p, 9, "a_col");
    link_prog(p, shader(GL_VERTEX_SHADER, "attribute vec2 a_pos; attribute vec4 a_col; varying vec4 v; void main() { v = a_col; gl_Position = vec4(a_pos, 0.0, 1.0); }\n"),
              shader(GL_FRAGMENT_SHADER, "precision mediump float; varying vec4 v; void main() { gl_FragColor = v; }\n"));
    ieq(glGetAttribLocation(p, "a_pos"), 5, "bound attribute location");
    ieq(glGetAttribLocation(p, "a_col"), 9, "bound attribute location");
    ieq(glGetAttribLocation(p, "nope"), -1, "no such attribute");
    glUseProgram(p);
    glVertexAttribPointer(5, 2, GL_FLOAT, GL_FALSE, 0, q);
    glEnableVertexAttribArray(5);
    glVertexAttrib4f(9, 0.0f, 0.5f, 1.0f, 1.0f);                /* a disabled array: the current value */
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    px(100, 100, "constant attribute", 0, 128, 255, 255);
    {
        static const GLubyte col[16] = { 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255 };
        glVertexAttribPointer(9, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, col);
        glEnableVertexAttribArray(9);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        px(100, 100, "normalized byte attribute", 255, 0, 0, 255);
        glDisableVertexAttribArray(9);
    }
    glDisableVertexAttribArray(5);
    glBindAttribLocation(p, 1, "gl_Vertex");
    err(GL_INVALID_OPERATION, "glBindAttribLocation gl_");
    glDeleteProgram(p);
}

static void test_uniforms(void)
{
    static const GLfloat ident[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }, m2[4] = { 0, 0, 0.75f, 0 };
    GLuint p = prog("attribute vec2 a_pos; uniform mat4 m; uniform vec4 arr[3]; struct S { vec2 a; float b; mat2 c; }; uniform S s[2]; uniform vec4 shared_v; varying vec4 v;\n"
                    "void main() { v = arr[2] + vec4(s[1].a, s[1].b, s[1].c[1][0]) + shared_v; gl_Position = m * vec4(a_pos, 0.0, 1.0); }\n",
                    "precision mediump float; varying vec4 v; uniform bool f; uniform int n; uniform vec4 shared_v; void main() { vec4 c = v; if (f) c.r += 0.25; c.g += float(n) * 0.125; gl_FragColor = c + shared_v * 0.0; }\n");
    GLfloat g[4];
    GLint i[4], loc;
    glUseProgram(p);
    glUniformMatrix4fv(glGetUniformLocation(p, "m"), 1, GL_FALSE, ident);
    glUniform4f(glGetUniformLocation(p, "arr[2]"), 0.25f, 0, 0, 0);
    glUniform2f(glGetUniformLocation(p, "s[1].a"), 0.25f, 0.25f);
    glUniform1f(glGetUniformLocation(p, "s[1].b"), 0.5f);
    glUniformMatrix2fv(glGetUniformLocation(p, "s[1].c"), 1, GL_FALSE, m2);
    glUniform4f(glGetUniformLocation(p, "shared_v"), 0, 0, 0, 0.25f);
    glUniform1i(glGetUniformLocation(p, "f"), 7);               /* any non-zero value is true */
    glUniform1i(glGetUniformLocation(p, "n"), 2);
    full(p);
    px(100, 100, "uniforms of every kind", 191, 128, 128, 255);
    ok(glGetUniformLocation(p, "nope") == -1, "no such uniform");
    ok(glGetUniformLocation(p, "arr[3]") == -1, "array index out of range");
    ok(glGetUniformLocation(p, "arr") == glGetUniformLocation(p, "arr[0]"), "name[0]");
    glGetUniformfv(p, glGetUniformLocation(p, "s[1].a"), g);
    ok(g[0] == 0.25f && g[1] == 0.25f, "glGetUniformfv");
    glGetUniformiv(p, glGetUniformLocation(p, "n"), i);
    ieq(i[0], 2, "glGetUniformiv");
    glGetUniformiv(p, glGetUniformLocation(p, "f"), i);
    ieq(i[0], 1, "a bool reads 1");
    err(GL_NO_ERROR, "uniforms");
    loc = glGetUniformLocation(p, "n");
    glUniform1f(loc, 1.0f); err(GL_INVALID_OPERATION, "glUniform1f on an int");
    loc = glGetUniformLocation(p, "arr[2]");
    glUniform1i(loc, 1); err(GL_INVALID_OPERATION, "glUniform1i on a vec4");
    glUniform2f(loc, 1, 1); err(GL_INVALID_OPERATION, "glUniform2f on a vec4");
    loc = glGetUniformLocation(p, "s[1].c");
    glUniform4f(loc, 1, 2, 3, 4); err(GL_INVALID_OPERATION, "glUniform4f on a mat2");
    glUniform1f(-1, 1.0f); err(GL_NO_ERROR, "location -1 is ignored");
    glUniformMatrix4fv(glGetUniformLocation(p, "m"), 1, GL_TRUE, ident); err(GL_INVALID_VALUE, "transpose");
    {
        GLint size = 0, n = 0, found = 0, k;
        GLenum type = 0;
        char name[100];
        glGetProgramiv(p, GL_ACTIVE_UNIFORMS, &n);
        for (k = 0; k < n; k++) {
            glGetActiveUniform(p, (GLuint)k, sizeof name, NULL, &size, &type, name);
            if (!strcmp(name, "arr[0]") && size == 3 && type == GL_FLOAT_VEC4) found++;
            if (!strcmp(name, "s[1].c") && size == 1 && type == GL_FLOAT_MAT2) found++;
        }
        ieq(found, 2, "glGetActiveUniform");
    }
    glDeleteProgram(p);
}

static void test_formats(void)
{
    static const uint8_t rgba[4] = { 10, 20, 30, 40 }, rgb[3] = { 10, 20, 30 }, bgra[4] = { 30, 20, 10, 40 }, lum[1] = { 100 }, la[2] = { 100, 200 };
    static const uint16_t c565 = 16 << 11 | 32 << 5 | 8, c4444 = 0x1234, c5551 = 31 << 11 | 15 << 1 | 1;
    static const uint8_t dxt1[8] = { 0x00, 0xF8, 0x1F, 0x00, 0, 0, 0, 0 };   /* colour 0 red, every index 0 */
    static const uint8_t dxt5[16] = { 200, 200, 0, 0, 0, 0, 0, 0, 0xE0, 0x07, 0x1F, 0x00, 0, 0, 0, 0 };
    GLuint t;
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    show_tex(make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 1, 1, rgba, GL_NEAREST, GL_CLAMP_TO_EDGE), "RGBA8", 10, 20, 30, 40);
    show_tex(make_tex(GL_RGB, GL_UNSIGNED_BYTE, 1, 1, rgb, GL_NEAREST, GL_CLAMP_TO_EDGE), "RGB: alpha 1", 10, 20, 30, 255);
    show_tex(make_tex(GL_BGRA_EXT, GL_UNSIGNED_BYTE, 1, 1, bgra, GL_NEAREST, GL_CLAMP_TO_EDGE), "BGRA", 10, 20, 30, 40);
    show_tex(make_tex(GL_LUMINANCE, GL_UNSIGNED_BYTE, 1, 1, lum, GL_NEAREST, GL_CLAMP_TO_EDGE), "LUMINANCE", 100, 100, 100, 255);
    show_tex(make_tex(GL_ALPHA, GL_UNSIGNED_BYTE, 1, 1, lum, GL_NEAREST, GL_CLAMP_TO_EDGE), "ALPHA", 0, 0, 0, 100);
    show_tex(make_tex(GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, 1, 1, la, GL_NEAREST, GL_CLAMP_TO_EDGE), "LUMINANCE_ALPHA", 100, 100, 100, 200);
    show_tex(make_tex(GL_RGB, GL_UNSIGNED_SHORT_5_6_5, 1, 1, &c565, GL_NEAREST, GL_CLAMP_TO_EDGE), "RGB565", 132, 130, 66, 255);
    show_tex(make_tex(GL_RGBA, GL_UNSIGNED_SHORT_4_4_4_4, 1, 1, &c4444, GL_NEAREST, GL_CLAMP_TO_EDGE), "RGBA4444", 17, 34, 51, 68);
    show_tex(make_tex(GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, 1, 1, &c5551, GL_NEAREST, GL_CLAMP_TO_EDGE), "RGBA5551", 255, 0, 123, 255);
    /* a sub-image of another type converts to the storage */
    t = make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 1, 1, rgba, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_SHORT_4_4_4_4, &c4444);
    err(GL_NO_ERROR, "sub-image, other type");
    show_tex(t, "4444 sub-image into RGBA8", 17, 34, 51, 68);
    t = make_tex(GL_RGB, GL_UNSIGNED_SHORT_5_6_5, 1, 1, &c565, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, rgb);
    err(GL_NO_ERROR, "sub-image, bytes into 565");
    show_tex(t, "bytes into RGB565", 8, 20, 33, 255);
    /* DXT */
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glCompressedTexImage2D(GL_TEXTURE_2D, 0, GL_COMPRESSED_RGB_S3TC_DXT1_EXT, 4, 4, 0, sizeof dxt1, dxt1);
    err(GL_NO_ERROR, "DXT1");
    full(g_tex_prog);
    px(W / 2, H / 2, "DXT1", 255, 0, 0, 255);
    glCompressedTexImage2D(GL_TEXTURE_2D, 0, GL_COMPRESSED_RGBA_S3TC_DXT5_EXT, 4, 4, 0, sizeof dxt5, dxt5);
    err(GL_NO_ERROR, "DXT5");
    full(g_tex_prog);
    px(W / 2, H / 2, "DXT5", 0, 255, 0, 200);
    glCompressedTexImage2D(GL_TEXTURE_2D, 0, GL_COMPRESSED_RGBA_S3TC_DXT5_EXT, 4, 4, 0, 3, dxt5);
    err(GL_INVALID_VALUE, "DXT size");
    glDeleteTextures(1, &t);
    /* unpack alignment: rows of 3 + 1 pad bytes, the second texel's row */
    {
        static const uint8_t pad[8] = { 1, 2, 3, 9, 40, 50, 60, 9 };
        t = make_tex(GL_RGB, GL_UNSIGNED_BYTE, 1, 2, NULL, GL_NEAREST, GL_CLAMP_TO_EDGE);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 1, 2, GL_RGB, GL_UNSIGNED_BYTE, pad);
        glBindTexture(GL_TEXTURE_2D, t);
        rect(g_tex_prog, -1, 0, 1, 1);                          /* t = 0 .. 1 over the top half: both rows, row 0 first */
        px(W / 2, H * 3 / 4 + 40, "unpack alignment, row 1", 40, 50, 60, 255);
        px(W / 2, H * 3 / 4 - 40, "unpack alignment, row 0", 1, 2, 3, 255);
        glDeleteTextures(1, &t);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    }
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    err(GL_INVALID_OPERATION, "internalformat != format");
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    err(GL_INVALID_VALUE, "border");
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_FLOAT, rgba);
    err(GL_INVALID_ENUM, "type");
}

static void test_complete(void)
{
    static const uint8_t c[4 * 9] = { 200, 100, 50, 255, 200, 100, 50, 255, 200, 100, 50, 255, 200, 100, 50, 255, 200, 100, 50, 255, 200, 100, 50, 255, 200, 100, 50, 255, 200, 100, 50, 255, 200, 100, 50, 255 };
    GLuint t;
    t = make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 3, 3, c, GL_NEAREST, GL_CLAMP_TO_EDGE);
    full(g_tex_prog);
    px(100, 100, "NPOT, clamp, no mipmap filter", 200, 100, 50, 255);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    full(g_tex_prog);
    px(100, 100, "NPOT, repeat (GL_OES_texture_npot)", 200, 100, 50, 255);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    full(g_tex_prog);
    px(100, 100, "NPOT, mipmap filter: incomplete", 0, 0, 0, 255);
    glGenerateMipmap(GL_TEXTURE_2D);
    err(GL_NO_ERROR, "glGenerateMipmap of NPOT");
    full(g_tex_prog);
    px(100, 100, "NPOT with its generated chain", 200, 100, 50, 255);
    glDeleteTextures(1, &t);
    /* rows must be tight: the TMU has no pitch, and a padded row shears the image */
    {
        static const uint8_t grid[4 * 15] = { 10, 0, 0, 255, 20, 0, 0, 255, 30, 0, 0, 255, 40, 0, 0, 255, 50, 0, 0, 255,
                                              0, 60, 0, 255, 0, 70, 0, 255, 0, 80, 0, 255, 0, 90, 0, 255, 0, 100, 0, 255,
                                              0, 0, 110, 255, 0, 0, 120, 255, 0, 0, 130, 255, 0, 0, 140, 255, 0, 0, 150, 255 };
        t = make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 5, 3, grid, GL_NEAREST, GL_CLAMP_TO_EDGE);
        full(g_tex_prog);
        px(W / 10, H / 6, "NPOT rows: (0, 0)", 10, 0, 0, 255);
        px(W * 9 / 10, H / 6, "NPOT rows: (4, 0)", 50, 0, 0, 255);
        px(W / 2, H / 2, "NPOT rows: (2, 1)", 0, 80, 0, 255);
        px(W / 10, H * 5 / 6, "NPOT rows: (0, 2)", 0, 0, 110, 255);
        px(W * 9 / 10, H * 5 / 6, "NPOT rows: (4, 2)", 0, 0, 150, 255);
        glDeleteTextures(1, &t);
    }
    /* the default state wants mipmaps: one level alone is incomplete */
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, c);
    full(g_tex_prog);
    px(100, 100, "no chain with a mipmap filter: incomplete", 0, 0, 0, 255);
    glDeleteTextures(1, &t);
    /* no texture at all */
    glBindTexture(GL_TEXTURE_2D, 0);
    full(g_tex_prog);
    px(100, 100, "texture 0 without an image", 0, 0, 0, 255);
}

static void test_mipmaps(void)
{
    static const uint8_t checker[4 * 16] = {
        255, 0, 0, 255, 0, 0, 0, 255, 255, 0, 0, 255, 0, 0, 0, 255,  255, 0, 0, 255, 0, 0, 0, 255, 255, 0, 0, 255, 0, 0, 0, 255,
        255, 0, 0, 255, 0, 0, 0, 255, 255, 0, 0, 255, 0, 0, 0, 255,  255, 0, 0, 255, 0, 0, 0, 255, 255, 0, 0, 255, 0, 0, 0, 255 };
    static const uint16_t c565[16] = { 0xF800, 0, 0xF800, 0, 0xF800, 0, 0xF800, 0, 0xF800, 0, 0xF800, 0, 0xF800, 0, 0xF800, 0 };
    static const uint8_t red[4 * 4] = { 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255 }, green[4] = { 0, 255, 0, 255 };
    GLuint t;
    /* one pixel for the whole texture: lambda picks the last level */
    t = make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 4, 4, checker, GL_NEAREST_MIPMAP_NEAREST, GL_CLAMP_TO_EDGE);
    glGenerateMipmap(GL_TEXTURE_2D);
    err(GL_NO_ERROR, "glGenerateMipmap");
    glViewport(10, 10, 1, 1);
    full(g_tex_prog);
    glViewport(0, 0, W, H);
    px(10, 10, "generated mipmap, RGBA8", 128, 0, 0, 255);
    glDeleteTextures(1, &t);
    t = make_tex(GL_RGB, GL_UNSIGNED_SHORT_5_6_5, 4, 4, c565, GL_NEAREST_MIPMAP_NEAREST, GL_CLAMP_TO_EDGE);
    glGenerateMipmap(GL_TEXTURE_2D);
    err(GL_NO_ERROR, "glGenerateMipmap 565");
    glViewport(10, 10, 1, 1);
    full(g_tex_prog);
    glViewport(0, 0, W, H);
    px(10, 10, "generated mipmap, RGB565", 132, 0, 0, 255);
    glDeleteTextures(1, &t);
    /* levels uploaded bottom-up: nothing is known about the chain until level 0 */
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 1, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, green);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, red);
    err(GL_NO_ERROR, "bottom-up upload");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glViewport(10, 10, 1, 1);
    full(g_tex_prog);
    glViewport(0, 0, W, H);
    px(10, 10, "level 1 uploaded before level 0", 0, 255, 0, 255);
    /* a level of the wrong size: incomplete, not a corrupted chain */
    glTexImage2D(GL_TEXTURE_2D, 1, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, red);
    err(GL_NO_ERROR, "level of another size");
    glViewport(10, 10, 1, 1);
    full(g_tex_prog);
    glViewport(0, 0, W, H);
    px(10, 10, "inconsistent level: incomplete", 0, 0, 0, 255);
    glDeleteTextures(1, &t);
}

static void test_cube(void)
{
    static const uint8_t f[6][4] = { { 255, 0, 0, 255 }, { 0, 255, 0, 255 }, { 0, 0, 255, 255 }, { 255, 255, 0, 255 }, { 255, 0, 255, 255 }, { 0, 255, 255, 255 } };
    static const float dir[6][3] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
    GLuint p = prog("attribute vec2 a_pos; uniform vec3 d; varying vec3 v; void main() { v = d; gl_Position = vec4(a_pos, 0.0, 1.0); }\n",
                    "precision mediump float; varying vec3 v; uniform samplerCube c; void main() { gl_FragColor = textureCube(c, v); }\n"), t;
    int i;
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_CUBE_MAP, t);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    for (i = 0; i < 5; i++) glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + (GLenum)i, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, f[i]);
    glUseProgram(p);
    glUniform3fv(glGetUniformLocation(p, "d"), 1, dir[0]);
    full(p);
    px(100, 100, "cube with a face missing: incomplete", 0, 0, 0, 255);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, f[5]);
    for (i = 0; i < 6; i++) {
        glUniform3fv(glGetUniformLocation(p, "d"), 1, dir[i]);
        full(p);
        px(100, 100, "cube face", f[i][0], f[i][1], f[i][2], 255);
    }
    glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, GL_RGBA, 2, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, f[0]);
    err(GL_INVALID_VALUE, "a cube face must be square");
    glTexImage2D(GL_TEXTURE_CUBE_MAP, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, f[0]);
    err(GL_INVALID_ENUM, "GL_TEXTURE_CUBE_MAP is not an image target");
    glDeleteTextures(1, &t);
    glDeleteProgram(p);
}

/* a colour clear of a scissor box inside a pass (a quad of g2_render's clear program, no pass end) against the same clear as a pass's own (the FLUSH's clear), in the bound 64 x 64
   framebuffer: a 16 x 16 block of the box must be the same bytes, with dither and without. Run for each 16-bit format, whose channels have their own ONE */
static void sub_clear_same(const char *fmt)
{
    static const float col[3][4] = { { 0.2f, 0.4f, 0.6f, 1.0f }, { 0.3333333f, 0.1234f, 0.999f, 0.5f }, { 0.5019f, 0.25f, 0.75f, 0.3f } };
    static uint8_t blk[2][16 * 16 * 4];
    char w[112];
    int c, d, pass, i, diff;
    for (c = 0; c < 3; c++) for (d = 0; d < 2; d++) {
        for (pass = 0; pass < 2; pass++) {
            glDisable(GL_SCISSOR_TEST);
            if (d) glEnable(GL_DITHER); else glDisable(GL_DITHER);
            clear_to(0.25f, 0.5f, 0.75f, 1.0f);
            if (pass) { color(1, 0, 0, 1); rect(g_col_prog, 0.5f, 0.5f, 0.75f, 0.75f); }      /* the pass has a draw: the clear below is a quad */
            glEnable(GL_SCISSOR_TEST);
            glScissor(5, 7, 37, 29);
            clear_to(col[c][0], col[c][1], col[c][2], col[c][3]);
            glDisable(GL_SCISSOR_TEST);
            glReadPixels(12, 12, 16, 16, GL_RGBA, GL_UNSIGNED_BYTE, blk[pass]);
        }
        diff = 0;
        for (i = 0; i < 16 * 16 * 4; i++) if (blk[0][i] != blk[1][i]) diff++;
        snprintf(w, sizeof w, "%s: a box cleared inside a pass = the pass's own clear (colour %d, dither %d: %d bytes differ)", fmt, c, d, diff);
        ok(diff == 0, w);
    }
    glEnable(GL_DITHER);
}

static void test_fbo(void)
{
    static const uint8_t lum[1] = { 9 };
    GLuint fb, t, rb, rb2;
    glGenFramebuffers(1, &fb);
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT, "empty framebuffer");
    glClear(GL_COLOR_BUFFER_BIT);
    err(GL_INVALID_FRAMEBUFFER_OPERATION, "clear of an incomplete framebuffer");

    /* colour texture + packed depth-stencil */
    t = make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 64, 64, NULL, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t, 64);
    err(GL_INVALID_VALUE, "an attachment level past the last");     /* OES_fbo_render_mipmap: any other level is legal */
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t, 0);
    glGenRenderbuffers(1, &rb);
    glBindRenderbuffer(GL_RENDERBUFFER, rb);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, 64, 64);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rb);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rb);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "colour + packed depth-stencil");
    glViewport(0, 0, 64, 64);
    glClearColor(0, 0, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    zcolor(0.5f, 1, 0, 0, 1);
    rect(g_zcol_prog, -1, -1, 1, 1);
    zcolor(0.75f, 0, 1, 0, 1);
    rect(g_zcol_prog, -1, -1, 0, 1);                            /* behind */
    glDisable(GL_DEPTH_TEST);
    px(10, 32, "render to texture, depth", 255, 0, 0, 255);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, W, H);
    glBindTexture(GL_TEXTURE_2D, t);
    full(g_tex_prog);
    px(100, 100, "the render target sampled", 255, 0, 0, 255);

    /* deleting the attached texture detaches it from the bound framebuffer */
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    glDeleteTextures(1, &t);
    {
        GLint ty = 1;
        glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE, &ty);
        ieq(ty, GL_NONE, "deleted texture detached");
    }

    /* NPOT colour: one level, a pitch for rendering */
    t = make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 53, 31, NULL, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t, 0);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "NPOT render target");
    glViewport(0, 0, 53, 31);
    clear_to(0, 1, 0, 1);
    color(1, 0, 0, 1);
    rect(g_col_prog, -1, -1, 0, 1);
    px(50, 30, "NPOT render target", 0, 255, 0, 255);
    px(10, 10, "NPOT render target, drawn", 255, 0, 0, 255);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, W, H);
    glBindTexture(GL_TEXTURE_2D, t);
    full(g_tex_prog);
    px(W / 4, H / 2, "NPOT render target sampled, left", 255, 0, 0, 255);
    px(W * 3 / 4, H / 2, "NPOT render target sampled, right", 0, 255, 0, 255);
    {   /* a sub-image into the texture shows when rendering goes on */
        static const uint8_t blue[4] = { 0, 0, 255, 255 };
        glTexSubImage2D(GL_TEXTURE_2D, 0, 40, 10, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, blue);
        glBindFramebuffer(GL_FRAMEBUFFER, fb);
        px(40, 10, "a sub-image reaches the render image", 0, 0, 255, 255);
        px(41, 10, "the rest of the render image is kept", 0, 255, 0, 255);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    glViewport(0, 0, 53, 31);
    {   /* a power of two below 8 wide: a full chain, tight, not renderable in place */
        GLuint s4 = make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 4, 4, NULL, GL_NEAREST, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, s4, 0);
        ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "4 x 4 render target");
        glViewport(0, 0, 4, 4);
        clear_to(1, 1, 0, 1);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, W, H);
        glBindTexture(GL_TEXTURE_2D, s4);
        full(g_tex_prog);
        px(100, 100, "4 x 4 render target sampled", 255, 255, 0, 255);
        glDeleteTextures(1, &s4);
        glBindFramebuffer(GL_FRAMEBUFFER, fb);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t, 0);
        glViewport(0, 0, 53, 31);
    }
    {   /* each face of a cube map is a render target of its own */
        static const char fs[] = "precision mediump float; varying vec3 v; uniform samplerCube c; void main() { gl_FragColor = textureCube(c, v); }\n";
        GLuint cp = prog("attribute vec2 a_pos; uniform vec3 d; varying vec3 v; void main() { v = d; gl_Position = vec4(a_pos, 0.0, 1.0); }\n", fs), ct;
        int i;
        glGenTextures(1, &ct);
        glBindTexture(GL_TEXTURE_CUBE_MAP, ct);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        for (i = 0; i < 6; i++) glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + (GLenum)i, 0, GL_RGBA, 8, 8, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
        glViewport(0, 0, 8, 8);
        for (i = 0; i < 6; i++) {
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_CUBE_MAP_POSITIVE_X + (GLenum)i, ct, 0);
            ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "cube face render target");
            clear_to(i == 0 ? 1.0f : 0.0f, i == 3 ? 1.0f : 0.0f, i == 5 ? 1.0f : 0.25f, 1);
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, W, H);
        glBindTexture(GL_TEXTURE_CUBE_MAP, ct);
        glUseProgram(cp);
        glUniform3f(glGetUniformLocation(cp, "d"), 1, 0, 0);
        full(cp);
        px(100, 100, "cube face +X rendered", 255, 0, 64, 255);
        glUniform3f(glGetUniformLocation(cp, "d"), 0, -1, 0);
        full(cp);
        px(100, 100, "cube face -Y rendered", 0, 255, 64, 255);
        glUniform3f(glGetUniformLocation(cp, "d"), 0, 0, -1);
        full(cp);
        px(100, 100, "cube face -Z rendered", 0, 0, 255, 255);
        glBindFramebuffer(GL_FRAMEBUFFER, fb);
        glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t, 0);
        glDeleteTextures(1, &ct);
        glDeleteProgram(cp);
        glViewport(0, 0, 53, 31);
    }

    /* an RGB texture has no alpha: writes leave it 1, reads say 1, ALPHA_BITS says 0 */
    glDeleteTextures(1, &t);
    t = make_tex(GL_RGB, GL_UNSIGNED_BYTE, 64, 64, NULL, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t, 0);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "RGB render target");
    glViewport(0, 0, 64, 64);
    clear_to(1, 0, 0, 0.25f);
    color(0, 1, 0, 0.25f);
    rect(g_col_prog, -1, -1, 0, 1);
    px(10, 10, "RGB target, draw: alpha 1", 0, 255, 0, 255);
    px(50, 10, "RGB target, clear: alpha 1", 255, 0, 0, 255);
    {
        GLint bits = 9;
        glGetIntegerv(GL_ALPHA_BITS, &bits);
        ieq(bits, 0, "RGB target ALPHA_BITS");
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, W, H);
    glBindTexture(GL_TEXTURE_2D, t);
    clear_to(0, 0, 0, 0);
    full(g_tex_prog);
    px(100, 100, "RGB target sampled: alpha 1", 0, 255, 0, 255);
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    glDeleteTextures(1, &t);

    /* 16-bit renderbuffers: ReadPixels converts on the host */
    glGenRenderbuffers(1, &rb2);
    glBindRenderbuffer(GL_RENDERBUFFER, rb2);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB565, 64, 64);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb2);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "RGB565 renderbuffer");
    clear_to(1, 0, 1, 1);
    px(10, 10, "RGB565 renderbuffer", 255, 0, 255, 255);
    sub_clear_same("RGB565");
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA4, 64, 64);
    clear_to(1, 1, 0, 0);
    px(10, 10, "RGBA4 renderbuffer", 255, 255, 0, 0);
    sub_clear_same("RGBA4");
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB5_A1, 64, 64);
    clear_to(0, 1, 1, 1);
    px(10, 10, "RGB5_A1 renderbuffer", 0, 255, 255, 255);
    sub_clear_same("RGB5_A1");
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, 64, 64);
    sub_clear_same("RGBA8");

    /* depth-only: DEPTH_COMPONENT16 */
    glBindRenderbuffer(GL_RENDERBUFFER, rb);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, 64, 64);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rb);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "DEPTH_COMPONENT16");
    {
        GLint bits = 0;
        glGetIntegerv(GL_DEPTH_BITS, &bits); ieq(bits, 16, "DEPTH_BITS 16");
        glGetIntegerv(GL_STENCIL_BITS, &bits); ieq(bits, 0, "no stencil attached");
        glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_DEPTH_SIZE, &bits); ieq(bits, 16, "RENDERBUFFER_DEPTH_SIZE");
    }
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_STENCIL_TEST);                                  /* no stencil buffer: the test always passes */
    glStencilFunc(GL_NEVER, 0, 255);
    zcolor(0.0f, 1, 1, 1, 1);
    rect(g_zcol_prog, -1, -1, 1, 1);
    glDisable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 0, 255);
    zcolor(0.5f, 1, 0, 0, 1);
    rect(g_zcol_prog, -1, -1, 1, 1);
    glDisable(GL_DEPTH_TEST);
    px(10, 10, "16-bit depth, stencil test without stencil", 255, 255, 255, 255);

    /* stencil-only: depth test without a depth buffer always passes */
    glRenderbufferStorage(GL_RENDERBUFFER, GL_STENCIL_INDEX8, 64, 64);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT, "stencil format on the depth attachment");
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rb);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "stencil only");
    glClearColor(0, 0, 0, 1);
    glClearStencil(0);
    glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_NEVER);
    color(0, 1, 0, 1);
    rect(g_col_prog, -1, -1, 1, 1);
    glDisable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    px(10, 10, "depth test without depth", 0, 255, 0, 255);

    /* formats and sizes */
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, 32, 32);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rb);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS, "attachments of other sizes");
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, 64, 64);
    {
        GLuint rs;
        glGenRenderbuffers(1, &rs);
        glBindRenderbuffer(GL_RENDERBUFFER, rs);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_STENCIL_INDEX8, 64, 64);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rs);
        ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_UNSUPPORTED, "separate depth and stencil images");
        glDeleteRenderbuffers(1, &rs);
    }
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
    t = make_tex(GL_LUMINANCE, GL_UNSIGNED_BYTE, 64, 64, NULL, GL_NEAREST, GL_CLAMP_TO_EDGE);
    (void)lum;
    glBindTexture(GL_TEXTURE_2D, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t, 0);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT, "LUMINANCE is not colour-renderable");
    glDeleteTextures(1, &t);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fb);
    glDeleteRenderbuffers(1, &rb);
    glDeleteRenderbuffers(1, &rb2);
    glViewport(0, 0, W, H);
    err(GL_NO_ERROR, "framebuffer objects");
}

static void test_fragment_ops(void)
{
    glClearColor(0, 0, 0, 1);
    glClearStencil(0);
    glClearDepthf(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    /* depth */
    glEnable(GL_DEPTH_TEST);
    zcolor(0.0f, 1, 0, 0, 1);
    rect(g_zcol_prog, -1, -1, 1, 1);
    zcolor(0.5f, 0, 1, 0, 1);
    rect(g_zcol_prog, -1, -1, 1, 1);
    px(100, 100, "depth: behind", 255, 0, 0, 255);
    zcolor(-0.5f, 0, 0, 1, 1);
    rect(g_zcol_prog, -1, -1, 1, 1);
    px(100, 100, "depth: in front", 0, 0, 255, 255);
    glDepthMask(GL_FALSE);
    zcolor(-0.75f, 1, 1, 1, 1);
    rect(g_zcol_prog, -1, -1, 1, 1);
    glDepthMask(GL_TRUE);
    zcolor(-0.625f, 0, 1, 0, 1);
    rect(g_zcol_prog, -1, -1, 1, 1);
    px(100, 100, "depth mask off", 0, 255, 0, 255);
    glDisable(GL_DEPTH_TEST);
    /* stencil: mark the left half, draw where marked */
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 1, 255);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    rect(g_zcol_prog, -1, -1, 0, 1);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glStencilFunc(GL_EQUAL, 1, 255);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    color(1, 1, 0, 1);
    rect(g_col_prog, -1, -1, 1, 1);
    px(100, 100, "stencil: inside", 255, 255, 0, 255);
    px(500, 100, "stencil: outside", 0, 255, 0, 255);
    /* two-sided */
    glClear(GL_STENCIL_BUFFER_BIT);
    glStencilFuncSeparate(GL_FRONT, GL_ALWAYS, 0, 255);
    glStencilFuncSeparate(GL_BACK, GL_NEVER, 0, 255);
    color(1, 0, 1, 1);
    rect(g_col_prog, -1, -1, 1, 1);                             /* counter-clockwise in the strip's first triangle: front */
    px(100, 100, "two-sided stencil: front passes", 255, 0, 255, 255);
    glDisable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 0, 255);
    /* blending */
    clear_to(0.5f, 0.5f, 0.5f, 0.5f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    color(1, 0, 0, 0.5f);
    rect(g_col_prog, -1, -1, 1, 1);
    px(100, 100, "blend: alpha", 191, 64, 64, -1);
    clear_to(0, 0, 0, 0.25f);
    glBlendFunc(GL_DST_ALPHA, GL_ZERO);
    color(1, 1, 1, 1);
    rect(g_col_prog, -1, -1, 1, 1);
    px(100, 100, "blend: destination alpha", 64, 64, 64, -1);
    clear_to(0, 0, 0, 1);
    glBlendColor(0.25f, 0.5f, 0.75f, 1.0f);
    glBlendFunc(GL_CONSTANT_COLOR, GL_ZERO);
    rect(g_col_prog, -1, -1, 1, 1);
    px(100, 100, "blend: constant colour", 64, 128, 191, -1);
    clear_to(0.5f, 0.5f, 0.5f, 1);
    glBlendFunc(GL_ONE, GL_ONE);
    glBlendEquation(GL_FUNC_REVERSE_SUBTRACT);
    color(0.25f, 0.25f, 0.25f, 0);
    rect(g_col_prog, -1, -1, 1, 1);
    px(100, 100, "blend: reverse subtract", 64, 64, 64, -1);
    glBlendEquation(GL_MAX);
    color(0.75f, 0, 0.75f, 0);
    rect(g_col_prog, -1, -1, 1, 1);
    px(100, 100, "blend: max", 191, 64, 191, -1);
    glBlendEquation(GL_FUNC_ADD);
    glBlendFunc(GL_ONE, GL_ZERO);
    glDisable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_SRC_ALPHA_SATURATE);
    err(GL_INVALID_ENUM, "SRC_ALPHA_SATURATE as a destination");
    /* scissor */
    clear_to(0, 0, 0, 1);
    glEnable(GL_SCISSOR_TEST);
    glScissor(100, 100, 50, 50);
    clear_to(1, 0, 0, 1);
    px(120, 120, "scissored clear: inside", 255, 0, 0, 255);
    px(200, 120, "scissored clear: outside", 0, 0, 0, 255);
    color(0, 1, 0, 1);
    glScissor(300, 100, 50, 50);
    full(g_col_prog);
    px(320, 120, "scissored draw: inside", 0, 255, 0, 255);
    px(120, 120, "scissored draw: outside", 255, 0, 0, 255);
    glDisable(GL_SCISSOR_TEST);
    /* colour mask */
    clear_to(0, 0, 0, 1);
    glColorMask(GL_TRUE, GL_FALSE, GL_TRUE, GL_FALSE);
    clear_to(1, 1, 1, 0);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    px(100, 100, "colour mask on a clear", 255, 0, 255, 255);
    /* cull */
    clear_to(0, 0, 0, 1);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);
    color(1, 0, 0, 1);
    rect(g_col_prog, -1, -1, 1, 1);
    px(100, 100, "cull front", 0, 0, 0, 255);
    glCullFace(GL_BACK);
    rect(g_col_prog, -1, -1, 1, 1);
    px(100, 100, "cull back", 255, 0, 0, 255);
    glDisable(GL_CULL_FACE);
    err(GL_NO_ERROR, "fragment operations");
}

static void test_primitives(void)
{
    GLuint p = prog("attribute vec2 a_pos; void main() { gl_PointSize = 16.0; gl_Position = vec4(a_pos, 0.0, 1.0); }\n", "precision mediump float; void main() { gl_FragColor = vec4(gl_PointCoord, 0.0, 1.0); }\n");
    GLint a = glGetAttribLocation(g_col_prog, "a_pos"), a2 = glGetAttribLocation(p, "a_pos");
    static const GLfloat pt[2] = { 0, 0 }, tri_fan[10] = { 0, 0, -0.5f, -0.5f, 0.5f, -0.5f, 0.5f, 0.5f, -0.5f, 0.5f }, loop[8] = { -0.5f, -0.5f, 0.5f, -0.5f, 0.5f, 0.5f, -0.5f, 0.5f };
    clear_to(0, 0, 0, 1);
    glUseProgram(p);
    glVertexAttribPointer((GLuint)a2, 2, GL_FLOAT, GL_FALSE, 0, pt);
    glEnableVertexAttribArray((GLuint)a2);
    glDrawArrays(GL_POINTS, 0, 1);
    glDisableVertexAttribArray((GLuint)a2);
    {
        uint8_t c[4];
        glReadPixels(W / 2 + 4, H / 2 + 4, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, c);
        g_checks++;
        if (c[0] < 150 || c[0] > 210 || c[1] > 90) { g_bad++; printf("FAIL point sprite: %d %d %d, gl_PointCoord right of centre, t grows downward\n", c[0], c[1], c[2]); }
    }
    /* §2.13: a point goes by near and far only, whatever the viewport */
    {
        static const GLfloat past[2] = { 1.2f, 0 };
        uint8_t c[4];
        clear_to(0, 0, 0, 1);
        glViewport(100, 100, 50, 50);
        glUseProgram(p);
        glVertexAttribPointer((GLuint)a2, 2, GL_FLOAT, GL_FALSE, 0, past);
        glEnableVertexAttribArray((GLuint)a2);
        glDrawArrays(GL_POINTS, 0, 1);
        glDisableVertexAttribArray((GLuint)a2);
        glViewport(0, 0, W, H);
        glReadPixels(150, 125, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, c);
        g_checks++;
        if (c[1] < 60) { g_bad++; printf("FAIL a point is not clipped by x and y: %d %d %d\n", c[0], c[1], c[2]); }
    }
    glDeleteProgram(p);
    color(0, 1, 0, 1);
    glVertexAttribPointer((GLuint)a, 2, GL_FLOAT, GL_FALSE, 0, tri_fan);
    glEnableVertexAttribArray((GLuint)a);
    clear_to(0, 0, 0, 1);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 5);
    px(W / 2, H / 2 + 60, "fan: inside", 0, 255, 0, 255);
    px(W / 2 - 240, H / 2, "fan: outside", 0, 0, 0, 255);
    glVertexAttribPointer((GLuint)a, 2, GL_FLOAT, GL_FALSE, 0, loop);
    clear_to(0, 0, 0, 1);
    glLineWidth(1.0f);
    glDrawArrays(GL_LINE_LOOP, 0, 4);
    px(W / 4, H / 2, "loop: the closing edge", 0, 255, 0, 255);
    px(W / 2, H / 2, "loop: centre", 0, 0, 0, 255);
    clear_to(0, 0, 0, 1);
    glDrawArrays(GL_LINE_STRIP, 0, 4);
    px(W / 2, H * 3 / 4, "strip: third segment", 0, 255, 0, 255);
    px(W / 4, H / 2, "strip: not closed", 0, 0, 0, 255);
    clear_to(0, 0, 0, 1);
    glDrawArrays(GL_LINES, 0, 4);
    px(W / 2, H / 4, "lines: first", 0, 255, 0, 255);
    px(W * 3 / 4, H / 2, "lines: not the gap", 0, 0, 0, 255);
    /* the view volume: board A clips x and y only far outside the target, so a smaller viewport is the library's to keep */
    {
        static const GLfloat across[4] = { -3, 0.02f, 3, 0.02f }, big[8] = { -3, -3, 3, -3, 3, 3, -3, 3 };
        clear_to(0, 0, 0, 1);
        glViewport(100, 100, 50, 50);
        glVertexAttribPointer((GLuint)a, 2, GL_FLOAT, GL_FALSE, 0, across);
        glDrawArrays(GL_LINES, 0, 2);
        px(125, 125, "view volume: a line inside", 0, 255, 0, 255);
        px(99, 125, "view volume: a line left of it", 0, 0, 0, 255);
        px(150, 125, "view volume: a line right of it", 0, 0, 0, 255);
        clear_to(0, 0, 0, 1);
        glVertexAttribPointer((GLuint)a, 2, GL_FLOAT, GL_FALSE, 0, big);
        glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
        px(125, 125, "view volume: inside", 0, 255, 0, 255);
        px(100, 100, "view volume: its corner", 0, 255, 0, 255);
        px(149, 149, "view volume: its other corner", 0, 255, 0, 255);
        px(99, 125, "view volume: left of it", 0, 0, 0, 255);
        px(150, 125, "view volume: right of it", 0, 0, 0, 255);
        px(125, 99, "view volume: below", 0, 0, 0, 255);
        px(125, 150, "view volume: above", 0, 0, 0, 255);
        clear_to(0, 0, 0, 1);
        glEnable(GL_SCISSOR_TEST);
        glScissor(120, 120, 100, 100);
        glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
        glDisable(GL_SCISSOR_TEST);
        px(130, 130, "view volume and scissor: inside both", 0, 255, 0, 255);
        px(115, 130, "view volume and scissor: inside the view only", 0, 0, 0, 255);
        px(160, 130, "view volume and scissor: inside the scissor only", 0, 0, 0, 255);
        glViewport(0, 0, W, H);
    }
    glDisableVertexAttribArray((GLuint)a);
    err(GL_NO_ERROR, "primitives");
}

static void test_indices(void)
{
    static const GLfloat v[8] = { -1, -1, 1, -1, -1, 1, 1, 1 };
    static const GLubyte i8[4] = { 0, 1, 2, 3 };
    static const GLushort i16[4] = { 0, 1, 2, 3 };
    static const GLuint i32[4] = { 0, 1, 2, 3 };
    GLint a = glGetAttribLocation(g_col_prog, "a_pos");
    GLuint vb, ib;
    GLfloat *big;
    GLuint bi[4] = { 0, 70000, 70001, 70002 };
    color(1, 0, 0, 1);
    glVertexAttribPointer((GLuint)a, 2, GL_FLOAT, GL_FALSE, 0, v);
    glEnableVertexAttribArray((GLuint)a);
    clear_to(0, 0, 0, 1);
    glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_BYTE, i8);
    px(100, 100, "u8 indices", 255, 0, 0, 255);
    clear_to(0, 0, 0, 1);
    glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, i16);
    px(100, 100, "u16 indices", 255, 0, 0, 255);
    clear_to(0, 0, 0, 1);
    glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_INT, i32);
    px(100, 100, "u32 indices", 255, 0, 0, 255);
    /* more than 16 bits between the lowest and highest: the hardware's indices are 16 bits */
    big = (GLfloat *)calloc(70004 * 2, sizeof(GLfloat));
    memcpy(big, v, 8);
    memcpy(big + 2 * 70000, v + 2, 24);
    glVertexAttribPointer((GLuint)a, 2, GL_FLOAT, GL_FALSE, 0, big);
    clear_to(0, 0, 0, 1);
    glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_INT, bi);
    px(100, 100, "indices beyond 16 bits", 255, 0, 0, 255);
    free(big);
    /* buffer objects, and an offset into each */
    glGenBuffers(1, &vb); glGenBuffers(1, &ib);
    glBindBuffer(GL_ARRAY_BUFFER, vb);
    glBufferData(GL_ARRAY_BUFFER, 64, NULL, GL_STATIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 16, 32, v);
    glVertexAttribPointer((GLuint)a, 2, GL_FLOAT, GL_FALSE, 0, (const void *)16);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ib);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 12, NULL, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 4, 8, i16);
    color(0, 1, 0, 1);
    clear_to(0, 0, 0, 1);
    glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, (const void *)4);
    px(100, 100, "buffer objects with offsets", 0, 255, 0, 255);
    err(GL_NO_ERROR, "indices");
    glDrawElements(GL_TRIANGLE_STRIP, 400, GL_UNSIGNED_SHORT, (const void *)4);
    err(GL_INVALID_OPERATION, "indices past the end of the buffer");
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glDeleteBuffers(1, &vb);
    {
        GLint b = 7;
        glGetVertexAttribiv((GLuint)a, GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING, &b);
        ieq(b, 0, "deleted buffer leaves the attribute binding");
    }
    glDeleteBuffers(1, &ib);
    glDisableVertexAttribArray((GLuint)a);
}

static void test_copies(void)
{
    uint8_t c[8 * 4];
    GLuint t;
    glPixelStorei(GL_PACK_ALIGNMENT, 8);
    clear_to(0.25f, 0.5f, 0.75f, 0.25f);
    /* RGB: alpha reads 1 although the framebuffer holds 0.25 */
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 10, 10, 16, 16, 0);
    err(GL_NO_ERROR, "glCopyTexImage2D RGB");
    clear_to(0, 0, 0, 0);
    full(g_tex_prog);
    px(100, 100, "copy to RGB: alpha 1", 64, 128, 191, 255);
    /* LUMINANCE takes red */
    clear_to(0.25f, 0.5f, 0.75f, 0.25f);
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, 10, 10, 16, 16, 0);
    clear_to(0, 0, 0, 0);
    full(g_tex_prog);
    px(100, 100, "copy to LUMINANCE", 64, 64, 64, 255);
    /* ALPHA takes alpha, into a texture that was specified before */
    clear_to(0.25f, 0.5f, 0.75f, 0.25f);
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, 10, 10, 16, 16, 0);
    clear_to(1, 1, 1, 1);
    full(g_tex_prog);
    px(100, 100, "copy to ALPHA", 0, 0, 0, 64);
    /* a sub-rectangle into RGB565: the format differs, the host converts */
    {
        static const uint16_t blue[1] = { 0x001F };
        uint16_t *m = (uint16_t *)malloc(16 * 16 * 2);
        int i;
        for (i = 0; i < 16 * 16; i++) m[i] = blue[0];
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 16, 16, 0, GL_RGB, GL_UNSIGNED_SHORT_5_6_5, m);
        free(m);
        clear_to(1, 0, 0, 1);
        glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, 8, 16);
        err(GL_NO_ERROR, "glCopyTexSubImage2D into 565");
        clear_to(0, 0, 0, 0);
        full(g_tex_prog);
        px(W / 4, H / 2, "copied half of the 565 texture", 255, 0, 0, 255);
        px(W * 3 / 4, H / 2, "the half not copied", 0, 0, 255, 255);
    }
    glDeleteTextures(1, &t);
    /* reads: alignment, BGRA, the rectangle partly outside the framebuffer */
    clear_to(0.25f, 0.5f, 0.75f, 1.0f);
    memset(c, 0, sizeof c);
    glReadPixels(100, 100, 2, 1, GL_BGRA_EXT, GL_UNSIGNED_BYTE, c);
    ok(abs(c[0] - 191) < 3 && abs(c[1] - 128) < 3 && abs(c[2] - 64) < 3 && abs(c[6] - 64) < 3, "glReadPixels BGRA");
    memset(c, 77, sizeof c);
    glReadPixels(W - 1, 0, 2, 1, GL_RGBA, GL_UNSIGNED_BYTE, c);
    ok(abs(c[0] - 64) < 3 && c[4] == 77, "glReadPixels past the right edge: inside read, outside untouched");
    glReadPixels(0, 0, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, c);
    err(GL_INVALID_OPERATION, "glReadPixels RGB is not the implementation's pair");
    {
        GLint f = 0, ty = 0;
        glGetIntegerv(GL_IMPLEMENTATION_COLOR_READ_FORMAT, &f);
        glGetIntegerv(GL_IMPLEMENTATION_COLOR_READ_TYPE, &ty);
        ok(f == GL_RGBA && ty == GL_UNSIGNED_BYTE, "implementation read pair");
    }
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
}

static void test_queries(void)
{
    GLint v[4];
    GLfloat f[4];
    GLboolean b[4];
    glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, v); ieq(v[0], 8, "MAX_TEXTURE_IMAGE_UNITS");
    glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, v); ok(v[0] >= 8, "MAX_VERTEX_ATTRIBS");
    glGetIntegerv(GL_MAX_VARYING_VECTORS, v); ok(v[0] >= 8, "MAX_VARYING_VECTORS");
    glGetIntegerv(GL_MAX_VERTEX_UNIFORM_VECTORS, v); ok(v[0] >= 128, "MAX_VERTEX_UNIFORM_VECTORS");
    glGetIntegerv(GL_MAX_FRAGMENT_UNIFORM_VECTORS, v); ok(v[0] >= 16, "MAX_FRAGMENT_UNIFORM_VECTORS");
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE, GL_ZERO, GL_DST_COLOR);
    glGetIntegerv(GL_BLEND_DST_ALPHA, v); ieq(v[0], GL_DST_COLOR, "BLEND_DST_ALPHA");
    glBlendFunc(GL_ONE, GL_ZERO);
    glStencilOpSeparate(GL_BACK, GL_INCR_WRAP, GL_DECR, GL_INVERT);
    glGetIntegerv(GL_STENCIL_BACK_PASS_DEPTH_PASS, v); ieq(v[0], GL_INVERT, "STENCIL_BACK_PASS_DEPTH_PASS");
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    glColorMask(GL_TRUE, GL_FALSE, GL_TRUE, GL_FALSE);
    glGetBooleanv(GL_COLOR_WRITEMASK, b); ok(b[0] && !b[1] && b[2] && !b[3], "COLOR_WRITEMASK");
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glClearColor(0.25f, 0.5f, 0.75f, 1);
    glGetFloatv(GL_COLOR_CLEAR_VALUE, f); ok(f[1] == 0.5f, "COLOR_CLEAR_VALUE");
    glGetBooleanv(GL_SHADER_COMPILER, b); ok(b[0], "SHADER_COMPILER");
    ok(glIsEnabled(GL_DITHER), "dither is on by default");
    ok(strstr((const char *)glGetString(GL_VERSION), "OpenGL ES 2.0") != NULL, "VERSION string");
    ok(strstr((const char *)glGetString(GL_SHADING_LANGUAGE_VERSION), "OpenGL ES GLSL ES 1.00") != NULL, "SHADING_LANGUAGE_VERSION string");
    err(GL_NO_ERROR, "queries");
    glEnable(0x1234); err(GL_INVALID_ENUM, "glEnable of nothing");
    glViewport(0, 0, -1, 1); err(GL_INVALID_VALUE, "glViewport negative");
    glPixelStorei(GL_PACK_ALIGNMENT, 3); err(GL_INVALID_VALUE, "alignment 3");
    glDrawArrays(GL_TRIANGLES, 0, -1); err(GL_INVALID_VALUE, "glDrawArrays negative");
    glDrawArrays(0x77, 0, 1); err(GL_INVALID_ENUM, "glDrawArrays mode");
    glDrawElements(GL_TRIANGLES, 3, GL_FLOAT, NULL); err(GL_INVALID_ENUM, "glDrawElements type");
    ok(eglGetProcAddress("glCreateProgram") != NULL && eglGetProcAddress("eglSwapBuffers") != NULL, "eglGetProcAddress");
}

static void test_contexts(void)
{
    static const EGLint ca[] = { 0x3098, 2, 0x3038 };
    EGLContext c2 = eglCreateContext(g_dpy, g_cfg, NULL, ca);
    GLfloat f[4];
    GLint v = 0;
    ok(c2 != NULL, "second context");
    glClearColor(0.25f, 0, 0, 1);
    glEnable(GL_BLEND);
    eglMakeCurrent(g_dpy, g_surf, g_surf, c2);
    glGetFloatv(GL_COLOR_CLEAR_VALUE, f);
    ok(f[0] == 0.0f, "a new context has its own state");
    ok(!glIsEnabled(GL_BLEND), "a new context has its own state");
    glGetIntegerv(GL_VIEWPORT, &v);
    glClearColor(0, 0.5f, 0, 1);
    eglMakeCurrent(g_dpy, g_surf, g_surf, g_ctx);
    glGetFloatv(GL_COLOR_CLEAR_VALUE, f);
    ok(f[0] == 0.25f && f[1] == 0.0f, "state comes back with its context");
    ok(glIsEnabled(GL_BLEND), "state comes back with its context");
    glDisable(GL_BLEND);
}

/* a window larger than the hardware renders 1 280 wide: the application goes on using its own coordinates */
static void test_scaled(void)
{
    enum { BW = 1920, BH = 1080 };
    HWND big = CreateWindowA("gles_test", "big", WS_POPUP, 0, 0, BW, BH, NULL, NULL, GetModuleHandleA(NULL), NULL);
    EGLSurface s = eglCreateWindowSurface(g_dpy, g_cfg, big, NULL);
    EGLint w = 0;
    GLint d[2];
    ok(eglMakeCurrent(g_dpy, s, s, g_ctx), "make the large window current");
    eglQuerySurface(g_dpy, s, 0x3057, &w);
    ieq(w, BW, "the surface keeps the window's size");
    glGetIntegerv(GL_MAX_VIEWPORT_DIMS, d);
    ok(d[0] >= BW && d[1] >= BH, "MAX_VIEWPORT_DIMS holds the window");
    glViewport(0, 0, BW, BH);
    clear_to(0, 0, 1, 1);
    px(BW - 1, BH - 1, "scaled window: the far corner", 0, 0, 255, 255);
    glViewport(0, 0, BW / 2, BH);
    color(1, 0, 0, 1);
    full(g_col_prog);
    glViewport(0, 0, BW, BH);
    px(BW / 4, BH / 2, "scaled window: viewport, inside", 255, 0, 0, 255);
    px(BW * 3 / 4, BH / 2, "scaled window: viewport, outside", 0, 0, 255, 255);
    glEnable(GL_SCISSOR_TEST);
    glScissor(BW / 2, 0, BW / 2, BH / 2);
    clear_to(0, 1, 0, 1);
    glDisable(GL_SCISSOR_TEST);
    px(BW * 3 / 4, BH / 4, "scaled window: scissor, inside", 0, 255, 0, 255);
    px(BW * 3 / 4, BH * 3 / 4, "scaled window: scissor, outside", 0, 0, 255, 255);
    {
        GLuint t;
        glGenTextures(1, &t);
        glBindTexture(GL_TEXTURE_2D, t);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, BW * 3 / 4 - 8, BH / 4 - 8, 16, 16, 0);     /* the green rectangle, by window coordinates */
        err(GL_NO_ERROR, "scaled window: copy");
        clear_to(0, 0, 0, 1);
        full(g_tex_prog);
        px(BW / 2, BH / 2, "scaled window: copied", 0, 255, 0, 255);
        glDeleteTextures(1, &t);
    }
    ok(eglSwapBuffers(g_dpy, s), "scaled window: swap");
    ok(eglMakeCurrent(g_dpy, g_surf, g_surf, g_ctx), "back to the small window");
    glViewport(0, 0, W, H);
    DestroyWindow(big);
    clear_to(1, 0, 1, 1);
    px(W - 1, H - 1, "back in the small window", 255, 0, 255, 255);
}

/* EGL configs of what B renders: RGB565 and a 16-bit depth buffer, sorted as §3.4.1.2 says */
static void test_configs(void)
{
    static const EGLint want[] = { 0x3024, 5, 0x3023, 6, 0x3022, 5, 0x3025, 16, 0x3038 };
    EGLConfig c[8], c565 = NULL;
    EGLint n = 0, v = 0, i;
    ok(eglChooseConfig(g_dpy, want, c, 8, &n), "eglChooseConfig for 5 6 5 and depth 16");
    ieq(n, 4, "four configs hold 5 6 5 and depth 16");
    eglGetConfigAttrib(g_dpy, c[0], 0x3024, &v);
    ieq(v, 8, "more colour bits come first");
    for (i = 0; i < n; i++) {
        EGLint r = 0, d = 0;
        eglGetConfigAttrib(g_dpy, c[i], 0x3024, &r);
        eglGetConfigAttrib(g_dpy, c[i], 0x3025, &d);
        if (r == 5 && d == 16 && !c565) c565 = c[i];
    }
    ok(c565 != NULL, "an RGB565 config with depth 16");
    if (c565) {
        HWND w = CreateWindowA("gles_test", "565", WS_POPUP, 0, 0, W, H, NULL, NULL, GetModuleHandleA(NULL), NULL);
        EGLSurface s = eglCreateWindowSurface(g_dpy, c565, w, NULL);
        GLint b[3];
        GLfloat z = 0;
        ok(eglMakeCurrent(g_dpy, s, s, g_ctx), "make the 565 window current");
        glViewport(0, 0, W, H);
        glGetIntegerv(GL_GREEN_BITS, &b[0]);
        glGetIntegerv(GL_DEPTH_BITS, &b[1]);
        glGetIntegerv(GL_STENCIL_BITS, &b[2]);
        ieq(b[0] * 100 + b[1], 616, "its green and depth bits");
        ieq(b[2], 0, "no stencil");
        glDisable(GL_DITHER);
        clear_to(4 / 255.0f, 4 / 255.0f, 4 / 255.0f, 1);
        px(W / 2, H / 2, "a 565 window keeps 5 6 5 bits", 0, 4, 0, 255);
        glEnable(GL_DITHER);
        (void)z;
        ok(eglSwapBuffers(g_dpy, s), "565 window: swap");
        ok(eglMakeCurrent(g_dpy, g_surf, g_surf, g_ctx), "back to the RGBA8 window");
        glViewport(0, 0, W, H);
        DestroyWindow(w);
        clear_to(1, 0, 1, 1);
        px(W - 1, H - 1, "back in the RGBA8 window", 255, 0, 255, 255);
    }
}

/* what real shaders use: Cocos2d declares samplers in vertex shaders, void parameter lists, gl_FragData, ^^, sampler arrays, matrix attributes */
static void test_language(void)
{
    static const uint8_t r[4] = { 255, 0, 0, 255 }, g[4] = { 0, 255, 0, 255 };
    GLuint p, t[2];
    GLint loc, units[2] = { 3, 5 };
    p = prog("uniform sampler2D CC_Texture0; attribute vec2 a_pos; float half_of(void) { return 0.5; } void main(void) { gl_Position = vec4(a_pos * half_of(), 0.0, 1.0); }\n",
             "precision mediump float; uniform sampler2D CC_Texture0; void main(void) { gl_FragData[0] = vec4(0.0, 1.0, 0.0, 1.0); }\n");
    clear_to(0, 0, 0, 1);
    full(p);
    px(W / 2, H / 2, "an unused sampler in the vertex stage, f(void), gl_FragData", 0, 255, 0, 255);
    px(10, 10, "f(void) halves the quad", 0, 0, 0, 255);
    glDeleteProgram(p);
    p = prog("attribute vec2 a_pos; varying vec2 v; void main() { v = a_pos; gl_Position = vec4(a_pos, 0.0, 1.0); }\n",
             "precision mediump float; varying vec2 v; void main() { bool a = v.x > 0.0, b = v.y > 0.0; gl_FragColor = (a ^^ b) ? vec4(1.0, 1.0, 0.0, 1.0) : vec4(0.0, 0.0, 1.0, 1.0); }\n");
    full(p);
    px(W * 3 / 4, H * 3 / 4, "^^: both true", 0, 0, 255, 255);
    px(W / 4, H / 4, "^^: both false", 0, 0, 255, 255);
    px(W * 3 / 4, H / 4, "^^: one true", 255, 255, 0, 255);
    glDeleteProgram(p);
    /* a vertex shader reads back a varying and writes part of it: Cocos2d's TextureCoordOut.y = 1.0 - TextureCoordOut.y */
    p = prog("attribute vec2 a_pos; attribute vec2 a_uv; varying vec2 t; varying vec4 w; void main() { t = a_uv; t.y = 1.0 - t.y; w = vec4(0.25, 0.5, 0.0, 1.0); w.b = w.r + w.r; gl_Position = vec4(a_pos, 0.0, 1.0); }\n",
             "precision mediump float; varying vec2 t; varying vec4 w; void main() { gl_FragColor = vec4(t.x, t.y, w.b, w.a); }\n");
    full(p);
    px(W / 4, H * 3 / 4, "varying read back and written in part: t.x, 1 - t.y", 64, 64, 128, 255);
    px(W * 3 / 4, H / 4, "varying read back and written in part, other corner", 191, 191, 128, 255);
    glDeleteProgram(p);
    /* varying arrays and matrices take a run of indices; the fragment stage declares them in another order */
    p = prog("attribute vec2 a_pos; varying vec2 after; varying vec3 d[3]; varying mat2 m; void main() { for (int i = 0; i < 3; ++i) { d[i] = vec3(0.25) * float(i); } m[0] = vec2(0.25, 0.0); m[1] = vec2(0.0, 0.5); after = vec2(0.75, 1.0); gl_Position = vec4(a_pos, 0.0, 1.0); }\n",
             "precision mediump float; varying mat2 m; varying vec3 d[3]; varying vec2 after; void main() { gl_FragColor = vec4(d[2].r, m[0][0] + m[1][1], after.x, after.y); }\n");
    full(p);
    px(100, 100, "varying array, matrix, and what follows them", 128, 191, 191, 255);
    glDeleteProgram(p);
    /* ++i and --i */
    p = prog("attribute vec2 a_pos; varying float v; void main() { float s = 0.0; int k = 0; for (int i = 0; i < 3; ++i) { s += 0.125; } int a = ++k; float b = 2.0; float c = --b; v = s + float(a) * 0.125 + c * 0.25; gl_Position = vec4(a_pos, 0.0, 1.0); }\n",
             "precision mediump float; varying float v; void main() { gl_FragColor = vec4(v, 0.0, 0.0, 1.0); }\n");
    full(p);
    px(100, 100, "prefix operators", 191, 0, 0, 255);
    glDeleteProgram(p);
    /* a sampler array: each element has its own unit */
    p = prog(VS, "precision mediump float; varying vec2 v_uv; uniform sampler2D s[2]; void main() { gl_FragColor = v_uv.x < 0.5 ? texture2D(s[0], v_uv) : texture2D(s[1], v_uv); }\n");
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glActiveTexture(GL_TEXTURE3);
    t[0] = make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 1, 1, r, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glActiveTexture(GL_TEXTURE5);
    t[1] = make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 1, 1, g, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glActiveTexture(GL_TEXTURE0);
    glUseProgram(p);
    loc = glGetUniformLocation(p, "s");
    glUniform1iv(loc, 2, units);
    err(GL_NO_ERROR, "sampler array");
    full(p);
    px(W / 4, H / 2, "sampler array, element 0", 255, 0, 0, 255);
    px(W * 3 / 4, H / 2, "sampler array, element 1", 0, 255, 0, 255);
    glUniform1i(glGetUniformLocation(p, "s[1]"), 3);
    full(p);
    px(W * 3 / 4, H / 2, "sampler array, element set alone", 255, 0, 0, 255);
    glUniform1i(loc, 99);
    err(GL_INVALID_VALUE, "sampler unit out of range");
    glDeleteProgram(p);
    glActiveTexture(GL_TEXTURE3); glDeleteTextures(1, &t[0]);
    glActiveTexture(GL_TEXTURE5); glDeleteTextures(1, &t[1]);
    glActiveTexture(GL_TEXTURE0);
    /* a matrix attribute takes one location per column */
    p = prog("attribute vec2 a_pos; attribute mat2 m; void main() { gl_Position = vec4(m * a_pos, 0.0, 1.0); }\n", "precision mediump float; void main() { gl_FragColor = vec4(1.0, 0.0, 1.0, 1.0); }\n");
    loc = glGetAttribLocation(p, "m");
    ok(loc >= 0 && loc + 1 < 16, "a matrix attribute's location");
    {
        GLint other = glGetAttribLocation(p, "a_pos");
        ok(other != loc && other != loc + 1, "the next attribute does not share a column's location");
    }
    glVertexAttrib2f((GLuint)loc, 0.5f, 0.0f);
    glVertexAttrib2f((GLuint)loc + 1, 0.0f, 0.25f);
    clear_to(0, 0, 0, 1);
    full(p);
    px(W / 2 + 100, H / 2 + 40, "matrix attribute: inside", 255, 0, 255, 255);
    px(W / 2 + 100, H / 2 + 100, "matrix attribute: the second column scales y", 0, 0, 0, 255);
    px(W / 2 + 200, H / 2, "matrix attribute: the first column scales x", 0, 0, 0, 255);
    glDeleteProgram(p);
    /* out arguments with no register of their own, copied back past the function's return; a macro call over two lines */
    p = prog(VS, "precision mediump float;\n#define PICK(a, b) vec4(a, b, 0.5, 1.0)\n"
                 "void paint(out vec4 c, vec4 v) { if (v.x > 0.5) { c = v.wzyx; return; } c = v; }\nvoid half_it(inout vec2 q) { q = q.yx * 0.5; }\n"
                 "void main() { vec4 a = PICK(1.0,\n                  0.25); half_it(a.xy); paint(gl_FragColor, a.wzyx); }\n");
    full(p);
    px(100, 100, "out arguments, a macro call over two lines", 32, 128, 128, 255);
    glDeleteProgram(p);
    /* names: a parameter hides the caller's constant, a function's uniform is not the caller's local, a block's x is not the outer one */
    p = prog(VS, "precision mediump float; uniform float k;\nfloat f(float x) { return x; }\nfloat g() { return k; }\n"
                 "void main() { float x = 0.25; float k = 0.75; float y = f(0.5); { float x = 1.0; } gl_FragColor = vec4(x, y, g(), 1.0) * (k + 0.25); }\n");
    glUseProgram(p);
    glUniform1f(glGetUniformLocation(p, "k"), 1.0f);
    full(p);
    px(100, 100, "names in their own scope", 64, 128, 255, 255);
    glDeleteProgram(p);
    err(GL_NO_ERROR, "language");
}

/* lanes that disagree: the odd columns take one way and the even ones the other, so every quad of the row is mixed. A jump out of an IF passes any test where a warp agrees, and loses lanes here */
static void lanes(const char *what, const char *body)
{
    static uint8_t c[W * 4];
    char fs[1024];
    GLuint p;
    int x, wrong = 0;
    snprintf(fs, sizeof fs, "precision mediump float; uniform float n; struct S { vec4 a; float b; };\nfloat odd() { return mod(floor(gl_FragCoord.x), 2.0); }\n%s\n", body);
    p = prog("attribute vec2 a_pos; void main() { gl_Position = vec4(a_pos, 0.0, 1.0); }\n", fs);
    glUseProgram(p);
    glUniform1f(glGetUniformLocation(p, "n"), 4.0f);
    full(p);
    glReadPixels(0, 100, W, 1, GL_RGBA, GL_UNSIGNED_BYTE, c);
    g_checks++;
    for (x = 0; x < W; x++) if (abs(c[x * 4] - (x & 1 ? 64 : 191)) > 2) wrong++;
    if (wrong) fail("%d of %d columns wrong", what, wrong, W);
    glDeleteProgram(p);
}

static void test_lanes(void)
{
    GLuint p;
    lanes("return in some lanes", "float f(float o) { if (o > 0.5) return 0.25; return 0.75; }\nvoid main() { gl_FragColor = vec4(f(odd()), 0.0, 0.0, 1.0); }");
    lanes("return in both branches", "float f(float o) { if (o > 0.5) { return 0.25; } else { return 0.75; } }\nvoid main() { gl_FragColor = vec4(f(odd()), 0.0, 0.0, 1.0); }");
    lanes("return with code after the if", "float f(float o) { float r = 0.5; if (o > 0.5) { r = 0.25; return r; } r = 0.75; return r; }\nvoid main() { gl_FragColor = vec4(f(odd()), 0.0, 0.0, 1.0); }");
    lanes("return in a loop", "float f(float o) { for (float i = 0.0; i < n; i += 1.0) { if (o > 0.5) return 0.25; } return 0.75; }\nvoid main() { gl_FragColor = vec4(f(odd()), 0.0, 0.0, 1.0); }");
    lanes("return past two loops", "float f(float o) { float s = 0.0; for (float i = 0.0; i < n; i += 1.0) { for (float j = 0.0; j < n; j += 1.0) { if (o > 0.5) return 0.25; } s = 0.75; } return s; }\nvoid main() { gl_FragColor = vec4(f(odd()), 0.0, 0.0, 1.0); }");
    lanes("return of a matrix", "mat2 f(float o) { if (o > 0.5) return mat2(0.25, 0.0, 0.0, 1.0); return mat2(0.75, 0.0, 0.0, 1.0); }\nvoid main() { mat2 m = f(odd()); gl_FragColor = vec4(m[0].x, 0.0, 0.0, 1.0); }");
    lanes("return of a struct", "S f(float o) { S a = S(vec4(0.25), 1.0); S b = S(vec4(0.75), 2.0); if (o > 0.5) return a; return b; }\nvoid main() { S s = f(odd()); gl_FragColor = vec4(s.a.x, 0.0, 0.0, 1.0); }");
    lanes("return from main", "void main() { gl_FragColor = vec4(0.25, 0.0, 0.0, 1.0); if (odd() > 0.5) return; gl_FragColor = vec4(0.75, 0.0, 0.0, 1.0); }");
    lanes("return from main, written in part", "void main() { gl_FragColor.gba = vec3(0.0, 0.0, 1.0); gl_FragColor.r = 0.25; if (odd() > 0.5) return; gl_FragColor.r = 0.75; }");
    lanes("gl_FragColor written in part under an if", "void main() { gl_FragColor = vec4(0.75, 0.0, 0.0, 1.0); if (odd() > 0.5) gl_FragColor.r = 0.25; }");
    lanes("gl_FragColor read back", "void main() { gl_FragColor = vec4(0.75, 0.0, 0.0, 1.0); if (odd() > 0.5) gl_FragColor.r = gl_FragColor.r / 3.0; }");
    lanes("a matrix assigned under an if", "void main() { mat2 m = mat2(0.75, 0.0, 0.0, 1.0); mat2 k = mat2(0.25, 0.0, 0.0, 1.0); if (odd() > 0.5) m = k; gl_FragColor = vec4(m[0].x, 0.0, 0.0, 1.0); }");
    lanes("a struct assigned under an if", "void main() { S s = S(vec4(0.75), 1.0); S t = S(vec4(0.25), 2.0); if (odd() > 0.5) s = t; gl_FragColor = vec4(s.a.x, 0.0, 0.0, 1.0); }");
    lanes("?: of matrices", "void main() { mat2 m = odd() > 0.5 ? mat2(0.25, 0.0, 0.0, 1.0) : mat2(0.75, 0.0, 0.0, 1.0); gl_FragColor = vec4(m[0].x, 0.0, 0.0, 1.0); }");
    /* gl_Position written in part and read back: the rectangle of the upper half lands in the lower one */
    p = prog("attribute vec2 a_pos; void main() { gl_Position = vec4(a_pos, 0.0, 1.0); gl_Position.y = -gl_Position.y; }\n", "precision mediump float; void main() { gl_FragColor = vec4(0.0, 1.0, 0.0, 1.0); }\n");
    clear_to(0, 0, 0, 1);
    rect(p, -1, 0, 1, 1);
    px(W / 2, H / 4, "gl_Position.y written back: where it lands", 0, 255, 0, 255);
    px(W / 2, H * 3 / 4, "gl_Position.y written back: where it came from", 0, 0, 0, 255);
    glDeleteProgram(p);
    err(GL_NO_ERROR, "lanes");
}

/* A buffer that stops changing moves to board A, and nothing a program can see may depend on whether it has yet. Each scene runs over several frames, so the move, the drops and the moves back happen inside the test. GPU2_KEEP_MB=1 runs
 * the same with eviction, =0 without the move at all */
typedef struct { float x, y, c[4]; } vtx;
static void vquad(vtx *o, float x0, float x1, float r, float g, float b)
{
    int i;
    for (i = 0; i < 4; i++) { o[i].x = i & 1 ? x1 : x0; o[i].y = i & 2 ? 1.0f : -1.0f; o[i].c[0] = r; o[i].c[1] = g; o[i].c[2] = b; o[i].c[3] = 1.0f; }
}

static void test_kept(void)
{
    static const uint16_t ix[6] = { 4, 5, 6, 6, 5, 7 };
    static const uint8_t blue[32] = { 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 255, 255,  0, 0, 255, 255,  0, 0, 255, 255,  0, 0, 255, 255 };
    enum { LEAD = 12, BIG = 300 * 1024, HUGE_B = 700 * 1024 };
    vtx v[8], w[4];
    uint8_t store[LEAD + sizeof v], *big;
    GLuint p = prog("attribute vec2 a_pos; attribute vec4 a_c; varying vec4 v; void main() { gl_Position = vec4(a_pos, 0.0, 1.0); v = a_c; }\n", "precision mediump float; varying vec4 v; void main() { gl_FragColor = v; }\n"), b[6];
    GLint ap = glGetAttribLocation(p, "a_pos"), ac = glGetAttribLocation(p, "a_c");
    int f, i;
#define KEPT_ATTRS(lead) do { glVertexAttribPointer((GLuint)ap, 2, GL_FLOAT, GL_FALSE, sizeof(vtx), (void *)(intptr_t)(lead)); glVertexAttribPointer((GLuint)ac, 4, GL_FLOAT, GL_FALSE, sizeof(vtx), (void *)(intptr_t)((lead) + 8)); } while (0)
    glUseProgram(p);
    glEnableVertexAttribArray((GLuint)ap);
    glEnableVertexAttribArray((GLuint)ac);
    glGenBuffers(6, b);
    vquad(v, -1, 0, 1, 0, 0);
    vquad(v + 4, 0, 1, 0, 1, 0);
    memset(store, 0xEE, LEAD);
    memcpy(store + LEAD, v, sizeof v);
    glBindBuffer(GL_ARRAY_BUFFER, b[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof store, store, GL_STATIC_DRAW);
    KEPT_ATTRS(LEAD);
    for (f = 0; f < 8; f++) {
        clear_to(0, 0, 0, 1);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        px(W / 4, H / 2, "kept: from vertex 0", 255, 0, 0, 255);
        clear_to(0, 0, 0, 1);
        glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);
        px(W * 3 / 4, H / 2, "kept: from vertex 4", 0, 255, 0, 255);
        px(W / 4, H / 2, "kept: from vertex 4, not the first quad", 0, 0, 0, 255);
        clear_to(0, 0, 0, 1);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, ix);
        px(W * 3 / 4, H / 2, "kept: indices from 4", 0, 255, 0, 255);
        eglSwapBuffers(g_dpy, g_surf);
    }
    /* changed between two draws of a frame: the first keeps what it read */
    clear_to(0, 0, 0, 1);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    vquad(w, 0, 1, 0, 0, 1);
    glBufferSubData(GL_ARRAY_BUFFER, LEAD, sizeof w, w);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    px(W / 4, H / 2, "kept: a draw before the change", 255, 0, 0, 255);
    px(W * 3 / 4, H / 2, "kept: a draw after the change", 0, 0, 255, 255);
    eglSwapBuffers(g_dpy, g_surf);
    /* a buffer that changes now and then: dropped, kept again, dropped */
    for (f = 0; f < 48; f++) {
        int c = 64 + (f / 7) * 16;
        if (f % 7 == 0) {
            vquad(w, -1, 1, (float)c / 255.0f, 0, 0);
            glBufferSubData(GL_ARRAY_BUFFER, LEAD, sizeof w, w);
        }
        clear_to(0, 0, 0, 1);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        px(W / 2, H / 2, "kept: changed now and then", c, 0, 0, 255);
        eglSwapBuffers(g_dpy, g_surf);
    }
    /* one attribute kept, one from client memory; indices that start at 4 reach both */
    vquad(v + 4, -1, 1, 0, 1, 0);
    memcpy(store + LEAD, v, sizeof v);
    glBufferData(GL_ARRAY_BUFFER, sizeof store, store, GL_STATIC_DRAW);
    KEPT_ATTRS(LEAD);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glVertexAttribPointer((GLuint)ac, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, blue);
    for (f = 0; f < 6; f++) {
        clear_to(0, 0, 0, 1);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, ix);
        px(W / 2, H / 2, "kept: beside a client array", 0, 0, 255, 255);
        px(W / 4, H / 2, "kept: glBufferData replaced it", 0, 0, 255, 255);
        eglSwapBuffers(g_dpy, g_surf);
    }
    /* deleted, and the name again with other data */
    glDeleteBuffers(1, &b[0]);
    vquad(w, -1, 1, 1, 1, 0);
    glBindBuffer(GL_ARRAY_BUFFER, b[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof w, w, GL_STATIC_DRAW);
    KEPT_ATTRS(0);
    for (f = 0; f < 6; f++) {
        clear_to(0, 0, 0, 1);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        px(W / 2, H / 2, "kept: the name again, other data", 255, 255, 0, 255);
        eglSwapBuffers(g_dpy, g_surf);
    }
    /* by now on A, never changed: glBufferData replaces it */
    vquad(w, -1, 1, 1, 0, 1);
    glBufferData(GL_ARRAY_BUFFER, sizeof w, w, GL_STATIC_DRAW);
    KEPT_ATTRS(0);
    clear_to(0, 0, 0, 1);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    px(W / 2, H / 2, "kept: replaced by glBufferData", 255, 0, 255, 255);
    eglSwapBuffers(g_dpy, g_surf);
    /* more than GPU2_KEEP_MB=1 holds: each drawn in turn, the oldest evicted; and one too big to be kept at all */
    big = (uint8_t *)calloc(1, HUGE_B);
    for (i = 0; i < 4; i++) {
        vquad(w, -1, 1, (float)(i + 1) * 0.2f, 0, 1);
        memcpy(big, w, sizeof w);
        glBindBuffer(GL_ARRAY_BUFFER, b[1 + i]);
        glBufferData(GL_ARRAY_BUFFER, BIG, big, GL_STATIC_DRAW);
    }
    vquad(w, -1, 1, 0, 1, 1);
    memcpy(big, w, sizeof w);
    glBindBuffer(GL_ARRAY_BUFFER, b[5]);
    glBufferData(GL_ARRAY_BUFFER, HUGE_B, big, GL_STATIC_DRAW);
    free(big);
    for (f = 0; f < 60; f++) {
        i = f % 5;
        glBindBuffer(GL_ARRAY_BUFFER, b[1 + i]);
        KEPT_ATTRS(0);
        clear_to(0, 0, 0, 1);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        if (i < 4) px(W / 2, H / 2, "kept: drawn in turn", (i + 1) * 51, 0, 255, 255);
        else px(W / 2, H / 2, "kept: too big to keep", 0, 255, 255, 255);
        eglSwapBuffers(g_dpy, g_surf);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDisableVertexAttribArray((GLuint)ap);
    glDisableVertexAttribArray((GLuint)ac);
    glDeleteBuffers(6, b);
    glDeleteProgram(p);
#undef KEPT_ATTRS
    err(GL_NO_ERROR, "kept");
}

/* a file of tests/ at the repository's root, three folders above this source */
static char *repo_test_file(const char *name)
{
    char path[1024];
    const char *f = __FILE__;
    size_t n = strlen(f);
    int up = 0;
    FILE *fp;
    long sz;
    char *s;
    while (n > 0 && up < 4) { n--; if (f[n] == '\\' || f[n] == '/') up++; }
    snprintf(path, sizeof path, "%.*s\\tests\\%s", (int)n, f, name);
    fp = fopen(path, "rb");
    g_checks++;
    if (!fp) { g_bad++; printf("FAIL cannot read %s\n", path); return NULL; }
    fseek(fp, 0, SEEK_END);
    sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    s = (char *)malloc((size_t)sz + 1);
    sz = (long)fread(s, 1, (size_t)sz, fp);
    s[sz] = 0;
    fclose(fp);
    return s;
}

/* the mean blue along a row of the left half: where the tunnel's seam runs */
static double row_blue(int y)
{
    static uint8_t c[W * 4];
    int x, n = 0, sum = 0;
    glReadPixels(0, y, W, 1, GL_RGBA, GL_UNSIGNED_BYTE, c);
    for (x = 24; x < W / 2 - 24; x += 4, n++) sum += c[x * 4 + 2];
    return (double)sum / n;
}

/* Shadertoy shaders, tests/fs_heart.frag and fs_tunnel.frag, through the library rather than host/fb's harness. The heart against host/fb/fb_shader.py's float
 * maths; the tunnel's texture stores the mip level in blue, so a seam of wrong LOD shows along the middle row, and only its naive form may have one */
static const char VS_TOY[] = "attribute vec2 a_pos; uniform vec4 uParam; varying vec4 vUV; void main() { gl_Position = vec4(a_pos, 0.0, 1.0); vUV = vec4(a_pos.x * uParam.x, a_pos.y, uParam.y, 1.0); }\n";
static void test_shadertoy(void)
{
    enum { TS = 256, TLEVELS = 9 };
    char *fs = repo_test_file("fs_heart.frag");
    GLuint p, t;
    int lv, x, y, pass;
    if (fs) {
        p = prog(VS_TOY, fs);
        glUseProgram(p);
        glUniform4f(glGetUniformLocation(p, "uParam"), (float)W / H, 0.2f, 0.0f, 0.0f);
        glUniform3f(glGetUniformLocation(p, "iResolution"), (float)W, (float)H, 1.0f);
        full(p);
        px(320, 240, "heart: the middle", 195, 19, 58, 255);
        px(280, 300, "heart: the upper left", 177, 12, 53, 255);
        px(360, 200, "heart: the lower right", 198, 35, 59, 255);
        px(40, 40, "heart: the background, bottom left", 164, 131, 124, 255);
        px(600, 440, "heart: the background, top right", 163, 131, 105, 255);
        glDeleteProgram(p);
        free(fs);
    }
    fs = repo_test_file("fs_tunnel.frag");
    if (fs) {                                                   /* derivatives and a level in a fragment shader: the page around a Shadertoy shader asks for them */
        char *e = (char *)malloc(strlen(fs) + 128);
        sprintf(e, "#extension GL_OES_standard_derivatives : enable\n#extension GL_EXT_shader_texture_lod : enable\n#define texture2DLod texture2DLodEXT\n%s", fs);
        free(fs);
        fs = e;
    }
    if (fs) {
        uint8_t *img = (uint8_t *)malloc(TS * TS * 4);
        char *m = strstr(fs, "#define IMPLEMENTATION ");
        glGenTextures(1, &t);
        glBindTexture(GL_TEXTURE_2D, t);
        for (lv = 0; lv < TLEVELS; lv++) {
            int s = TS >> lv;
            for (y = 0; y < s; y++) {
                for (x = 0; x < s; x++) {
                    uint8_t *c = img + (y * s + x) * 4;
                    c[0] = (uint8_t)(128 + 127 * cos(2.0 * 3.14159265358979 * 4.0 * (x + 0.5) / s));
                    c[1] = (uint8_t)(128 + 127 * cos(2.0 * 3.14159265358979 * 4.0 * (y + 0.5) / s));
                    c[2] = (uint8_t)(lv * 24);
                    c[3] = 255;
                }
            }
            glTexImage2D(GL_TEXTURE_2D, lv, GL_RGBA, s, s, 0, GL_RGBA, GL_UNSIGNED_BYTE, img);
        }
        free(img);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        for (pass = 0; pass < 2; pass++) {
            double seam, beside;
            m[23] = pass ? '0' : '1';                           /* IMPLEMENTATION: 1 explicit derivatives, 0 naive */
            p = prog(VS_TOY, fs);
            glUseProgram(p);
            glUniform4f(glGetUniformLocation(p, "uParam"), (float)W / H, 0.0f, 0.0f, 0.0f);
            glUniform3f(glGetUniformLocation(p, "iResolution"), (float)W, (float)(H + 2), 1.0f);     /* H / 2 even: H + 2 puts the centre inside a 2 x 2 quad, the seam exists */
            glUniform2f(glGetUniformLocation(p, "iChannelSize"), (float)TS, (float)TS);
            glUniform1i(glGetUniformLocation(p, "iChannel0"), 0);
            full(p);
            seam = (row_blue(H / 2) + row_blue(H / 2 + 1)) / 2;
            beside = row_blue(H / 2 + 8);
            g_checks++;
            if (pass ? seam < beside + 48 : fabs(seam - beside) > 12) fail("mean blue %d on the seam, %d beside it", pass ? "tunnel naive: no seam shown" : "tunnel: a seam", (int)seam, (int)beside);
            glDeleteProgram(p);
        }
        glDeleteTextures(1, &t);
        free(fs);
    }
    err(GL_NO_ERROR, "shadertoy");
}

/* ES 2.0 Appendix A.5: a vertex shader indexes uniform arrays with any integer expression (LDPX). Three quads in one draw, each vertex naming its own bone */
static void test_runtime_index(void)
{
    static const char FS_V[] = "precision mediump float; varying vec4 v; void main() { gl_FragColor = v; }\n";
    GLuint p = prog("attribute vec2 a_pos; attribute float a_i; uniform mat4 u_bone[3]; uniform vec4 u_col[3]; varying vec4 v;\n"
                    "void main() { int i = int(a_i); v = u_col[i]; gl_Position = u_bone[i] * vec4(a_pos, 0.0, 1.0); }\n", FS_V);
    GLuint s = prog("attribute vec2 a_pos; attribute float a_i; struct L { vec4 c[2]; float k; }; uniform L u_l[3]; varying vec4 v;\n"
                    "void main() { int i = int(a_i); v = (u_l[i].c[1] + u_l[2 - i].c[0]) * u_l[i].k; gl_Position = vec4(a_pos.x * 8.0, a_pos.y * 8.0, 0.0, 1.0); }\n", FS_V);
    GLfloat m[48], v[18 * 3];
    static const GLfloat cols[12] = { 1, 0, 0, 1,  0, 1, 0, 1,  0, 0, 1, 1 };
    static const GLfloat q[12] = { -0.1f, -0.1f,  0.1f, -0.1f,  -0.1f, 0.1f,  -0.1f, 0.1f,  0.1f, -0.1f,  0.1f, 0.1f };
    static const float idx[3] = { 0.0f, 1.9f, 2.0f };           /* int() truncates */
    GLint a, i_;
    GLuint fsh;
    GLint st = 1;
    int b, k;
    memset(m, 0, sizeof m);
    for (b = 0; b < 3; b++) {
        m[b * 16 + 0] = m[b * 16 + 5] = m[b * 16 + 10] = m[b * 16 + 15] = 1;
        m[b * 16 + 12] = -0.6f + 0.6f * (float)b;               /* column major: translate x */
        for (k = 0; k < 6; k++) { v[(b * 6 + k) * 3] = q[k * 2]; v[(b * 6 + k) * 3 + 1] = q[k * 2 + 1]; v[(b * 6 + k) * 3 + 2] = idx[b]; }
    }
    glUseProgram(p);
    glUniformMatrix4fv(glGetUniformLocation(p, "u_bone"), 3, GL_FALSE, m);
    glUniform4fv(glGetUniformLocation(p, "u_col"), 3, cols);
    a = glGetAttribLocation(p, "a_pos"); i_ = glGetAttribLocation(p, "a_i");
    glVertexAttribPointer((GLuint)a, 2, GL_FLOAT, GL_FALSE, 12, v); glEnableVertexAttribArray((GLuint)a);
    glVertexAttribPointer((GLuint)i_, 1, GL_FLOAT, GL_FALSE, 12, v + 2); glEnableVertexAttribArray((GLuint)i_);
    clear_to(0, 0, 0, 1);
    glDrawArrays(GL_TRIANGLES, 0, 18);
    px(128, 240, "run-time index: bone 0", 255, 0, 0, 255);
    px(320, 240, "run-time index: bone 1, from 1.9", 0, 255, 0, 255);
    px(512, 240, "run-time index: bone 2", 0, 0, 255, 255);
    px(224, 240, "run-time index: between the bones", 0, 0, 0, 255);
    glDisableVertexAttribArray((GLuint)a); glDisableVertexAttribArray((GLuint)i_);
    /* an array in a struct in an array: a run-time index, a constant one under it, one worked out from the first */
    glUseProgram(s);
    for (b = 0; b < 3; b++) {
        char n[32];
        snprintf(n, sizeof n, "u_l[%d].c[0]", b); glUniform4f(glGetUniformLocation(s, n), b == 0 ? 0.5f : 0, b == 1 ? 0.5f : 0, b == 2 ? 0.5f : 0, 0.5f);
        snprintf(n, sizeof n, "u_l[%d].c[1]", b); glUniform4f(glGetUniformLocation(s, n), b == 2 ? 0.5f : 0, b == 0 ? 0.25f : 0, b == 1 ? 0.25f : 0, 0.5f);
        snprintf(n, sizeof n, "u_l[%d].k", b); glUniform1f(glGetUniformLocation(s, n), b == 1 ? 2.0f : 1.0f);
    }
    a = glGetAttribLocation(s, "a_pos"); i_ = glGetAttribLocation(s, "a_i");
    glVertexAttribPointer((GLuint)a, 2, GL_FLOAT, GL_FALSE, 12, v); glEnableVertexAttribArray((GLuint)a);
    glVertexAttribPointer((GLuint)i_, 1, GL_FLOAT, GL_FALSE, 12, v + 2); glEnableVertexAttribArray((GLuint)i_);
    clear_to(0, 0, 0, 1);
    glDrawArrays(GL_TRIANGLES, 6, 6);                           /* index 1: (c[1] of 1 + c[0] of 1) x 2 */
    px(320, 240, "run-time index: a struct's array", 0, 255, 128, 255);
    glDisableVertexAttribArray((GLuint)a); glDisableVertexAttribArray((GLuint)i_);
    glDeleteProgram(p);
    glDeleteProgram(s);
    /* the fragment shader keeps the constant-index rule */
    fsh = glCreateShader(GL_FRAGMENT_SHADER);
    {
        static const char *const src = "precision mediump float; uniform vec4 u[4]; varying float k; void main() { gl_FragColor = u[int(k)]; }\n";
        glShaderSource(fsh, 1, &src, NULL);
        glCompileShader(fsh);
        glGetShaderiv(fsh, GL_COMPILE_STATUS, &st);
        ok(st != 0, "a run-time index in a fragment shader compiles");
    }
    glDeleteShader(fsh);
    err(GL_NO_ERROR, "run-time index");
}

static void test_swap(void)
{
    int i;
    eglSwapInterval(g_dpy, 1);
    for (i = 0; i < 6; i++) {
        clear_to(i & 1 ? 1.0f : 0.0f, 0, 1.0f, 1);
        color(0, 1, 0, 1);
        rect(g_col_prog, -0.5f, -0.5f, 0.5f, 0.5f);
        px(W / 2, H / 2, "after a swap: rendering continues", 0, 255, 0, 255);
        px(10, 10, "after a swap: clear", i & 1 ? 255 : 0, 0, 255, 255);
        ok(eglSwapBuffers(g_dpy, g_surf), "eglSwapBuffers");
    }
    eglSwapInterval(g_dpy, 0);
    for (i = 0; i < 6; i++) {
        clear_to(0, 1, i & 1 ? 1.0f : 0.0f, 1);
        eglSwapBuffers(g_dpy, g_surf);
    }
    clear_to(0.25f, 0.25f, 0.25f, 1);
    px(10, 10, "no vsync", 64, 64, 64, 255);
    eglSwapBuffers(g_dpy, g_surf);
    eglSwapInterval(g_dpy, 1);
}

static LRESULT CALLBACK wnd_proc(HWND w, UINT m, WPARAM a, LPARAM b) { return DefWindowProcA(w, m, a, b); }

/* ---------------------------------------------------------------- what a build of ANGLE exports beyond ES 2.0, and language forms the compiler must accept */
typedef void (__stdcall *divisor_t)(GLuint, GLuint);
typedef void (__stdcall *draw_arrays_i_t)(GLenum, GLint, GLsizei, GLsizei);
typedef void (__stdcall *draw_elems_i_t)(GLenum, GLsizei, GLenum, const void *, GLsizei);
typedef void (__stdcall *gen_t)(GLsizei, GLuint *);
typedef void (__stdcall *del_t)(GLsizei, const GLuint *);
typedef GLboolean (__stdcall *isf_t)(GLuint);
typedef void (__stdcall *setf_t)(GLuint, GLenum);
typedef void (__stdcall *finf_t)(GLuint);
typedef void (__stdcall *getf_t)(GLuint, GLenum, GLint *);
typedef void (__stdcall *storage_t)(GLenum, GLsizei, GLenum, GLsizei, GLsizei);
typedef void (__stdcall *blit_t)(GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLbitfield, GLenum);
typedef void (__stdcall *readn_t)(GLint, GLint, GLsizei, GLsizei, GLenum, GLenum, GLsizei, void *);
typedef void (__stdcall *getn_t)(GLuint, GLint, GLsizei, GLfloat *);
typedef GLenum (__stdcall *reset_t)(void);
typedef void (__stdcall *progbin_get_t)(GLuint, GLsizei, GLsizei *, GLenum *, void *);
typedef void (__stdcall *progbin_set_t)(GLuint, GLenum, const void *, GLsizei);
typedef void (__stdcall *trans_t)(GLuint, GLsizei, GLsizei *, GLchar *);
typedef void (__stdcall *dbufs_t)(GLsizei, const GLenum *);
typedef void (__stdcall *msaa_t)(GLenum, GLsizei, GLenum, GLsizei, GLsizei);
typedef void (__stdcall *qbegin_t)(GLenum, GLuint);
typedef void (__stdcall *qend_t)(GLenum);
typedef void (__stdcall *qget_t)(GLenum, GLenum, GLint *);
typedef void (__stdcall *tex3d_t)(GLenum, GLint, GLenum, GLsizei, GLsizei, GLsizei, GLint, GLenum, GLenum, const void *);
#define GETP(ty, n) ty n = (ty)eglGetProcAddress(#n); ok(n != NULL, "export " #n)

static void shader_scene(const char *what, const char *vs, const char *fs, int r, int g, int b, int a)
{
    GLuint p = prog(vs, fs);
    clear_to(0, 0, 0, 1);
    full(p);
    px(W / 2, H / 2, what, r, g, b, a);
    glDeleteProgram(p);
}

static void test_compiler_gaps(void)
{
    static const char V[] = "attribute vec2 a_pos; void main() { gl_Position = vec4(a_pos, 0.0, 1.0); }\n";
    GLuint p;
    shader_scene("an array of matrix varyings",
                 "attribute vec2 a_pos; varying mat2 m[2]; void main() { m[0] = mat2(0.25, 0.5, 0.75, 1.0); m[1] = mat2(0.1, 0.2, 0.3, 0.4); gl_Position = vec4(a_pos, 0.0, 1.0); }\n",
                 "precision mediump float; varying mat2 m[2]; void main() { gl_FragColor = vec4(m[0][0].x, m[0][1].x, m[1][1].y, 1.0); }\n", 64, 191, 102, 255);
    shader_scene("a local array of matrices", V, "precision mediump float; void main() { mat2 r[2]; r[0] = mat2(0.5, 0.0, 0.0, 0.5); r[1] = r[0] * r[0]; r[1][1].y = 0.75; gl_FragColor = vec4(r[1][0][0], r[0][1][1], r[1][1].y, 1.0); }\n", 64, 128, 191, 255);
    shader_scene("a local array of structs", V, "precision mediump float; struct L { vec2 v; float k; }; void main() { L l[2]; l[1].v = vec2(0.25, 0.75); l[1].k = 0.5; l[0] = l[1]; l[1].k = 0.0; gl_FragColor = vec4(l[0].v, l[0].k, 1.0); }\n", 64, 191, 128, 255);
    p = prog(V, "precision mediump float; struct S { vec4 a[2]; float f; }; uniform S u; void main() { S s; s = u; s.a[1] = s.a[1] * 0.5; gl_FragColor = s.a[0] + s.a[1] * s.f + u.a[1] * 0.25; }\n");
    glUseProgram(p);
    glUniform4f(glGetUniformLocation(p, "u.a[0]"), 0.5f, 0.0f, 0.0f, 0.0f);
    glUniform4f(glGetUniformLocation(p, "u.a[1]"), 0.0f, 1.0f, 0.0f, 1.0f);
    glUniform1f(glGetUniformLocation(p, "u.f"), 1.0f);
    clear_to(0, 0, 0, 1);
    full(p);
    px(W / 2, H / 2, "a uniform array copied in a struct, written, and the uniform left alone", 128, 191, 0, 191);
    glDeleteProgram(p);
    err(GL_NO_ERROR, "the compiler scenes");
}

static void test_instanced(void)
{
    static const char IV[] = "attribute vec2 a_pos; attribute vec2 a_off; attribute vec4 a_col; varying vec4 v; void main() { v = a_col; gl_Position = vec4(a_pos + a_off, 0.0, 1.0); }\n";
    static const char IF[] = "precision mediump float; varying vec4 v; void main() { gl_FragColor = v; }\n";
    static const GLfloat quad[8] = { -0.2f, -0.2f, 0.2f, -0.2f, -0.2f, 0.2f, 0.2f, 0.2f }, offs[8] = { -0.75f, 0, -0.25f, 0, 0.25f, 0, 0.75f, 0 };
    static const GLubyte cols[8] = { 255, 0, 0, 255, 0, 255, 0, 255 };
    static const GLushort strip[4] = { 0, 1, 2, 3 };
    GETP(divisor_t, glVertexAttribDivisorANGLE); GETP(draw_arrays_i_t, glDrawArraysInstancedANGLE); GETP(draw_elems_i_t, glDrawElementsInstancedANGLE);
    GLuint p = prog(IV, IF);
    GLint ap = glGetAttribLocation(p, "a_pos"), ao = glGetAttribLocation(p, "a_off"), ac = glGetAttribLocation(p, "a_col"), d = -1;
    if (!glVertexAttribDivisorANGLE || !glDrawArraysInstancedANGLE || !glDrawElementsInstancedANGLE) return;
    glUseProgram(p);
    glVertexAttribPointer((GLuint)ap, 2, GL_FLOAT, GL_FALSE, 0, quad);
    glVertexAttribPointer((GLuint)ao, 2, GL_FLOAT, GL_FALSE, 0, offs);
    glVertexAttribPointer((GLuint)ac, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, cols);
    glEnableVertexAttribArray((GLuint)ap); glEnableVertexAttribArray((GLuint)ao); glEnableVertexAttribArray((GLuint)ac);
    glVertexAttribDivisorANGLE((GLuint)ao, 1);
    glVertexAttribDivisorANGLE((GLuint)ac, 2);
    glGetVertexAttribiv((GLuint)ac, GL_VERTEX_ATTRIB_ARRAY_DIVISOR_ANGLE_T, &d);
    ieq(d, 2, "instancing: the divisor reads back");
    clear_to(0, 0, 0, 1);
    glDrawArraysInstancedANGLE(GL_TRIANGLE_STRIP, 0, 4, 4);
    px(W / 8, H / 2, "instancing: instance 0, colour 0", 255, 0, 0, 255);
    px(W * 3 / 8, H / 2, "instancing: instance 1, the colour's divisor is 2", 255, 0, 0, 255);
    px(W * 5 / 8, H / 2, "instancing: instance 2, colour 1", 0, 255, 0, 255);
    px(W * 7 / 8, H / 2, "instancing: instance 3", 0, 255, 0, 255);
    px(W / 4, H / 2, "instancing: between them", 0, 0, 0, 255);
    clear_to(0, 0, 0, 1);
    glDrawElementsInstancedANGLE(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, strip, 3);
    px(W * 5 / 8, H / 2, "instancing: indexed", 0, 255, 0, 255);
    px(W * 7 / 8, H / 2, "instancing: three instances, not four", 0, 0, 0, 255);
    clear_to(0, 0, 0, 1);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);                      /* a plain draw is instance 0 */
    px(W / 8, H / 2, "instancing: a plain draw is instance 0", 255, 0, 0, 255);
    px(W * 3 / 8, H / 2, "instancing: and only that", 0, 0, 0, 255);
    err(GL_NO_ERROR, "instancing");
    glVertexAttribDivisorANGLE((GLuint)ap, 1);
    glDrawArraysInstancedANGLE(GL_TRIANGLE_STRIP, 0, 4, 1);
    err(GL_INVALID_OPERATION, "instancing: no array with divisor 0");
    glVertexAttribDivisorANGLE((GLuint)ap, 0); glVertexAttribDivisorANGLE((GLuint)ao, 0); glVertexAttribDivisorANGLE((GLuint)ac, 0);
    glDisableVertexAttribArray((GLuint)ao); glDisableVertexAttribArray((GLuint)ac); glDisableVertexAttribArray((GLuint)ap);
    glDeleteProgram(p);
}

static void test_fence_robust(void)
{
    GETP(gen_t, glGenFencesNV); GETP(del_t, glDeleteFencesNV); GETP(isf_t, glIsFenceNV); GETP(setf_t, glSetFenceNV); GETP(isf_t, glTestFenceNV); GETP(finf_t, glFinishFenceNV); GETP(getf_t, glGetFenceivNV);
    GETP(readn_t, glReadnPixelsEXT); GETP(getn_t, glGetnUniformfvEXT); GETP(reset_t, glGetGraphicsResetStatusEXT);
    GLuint f = 0;
    GLint v = 0;
    uint8_t c[8];
    GLfloat u[4];
    GLint loc;
    glGenFencesNV(1, &f);
    ok(!glIsFenceNV(f), "fence: not one until it is set");
    glTestFenceNV(f);
    err(GL_INVALID_OPERATION, "fence: testing one never set");
    color(1, 0, 0, 1);
    clear_to(0, 0, 0, 1);
    full(g_col_prog);
    glSetFenceNV(f, 0x84F2);
    ok(glIsFenceNV(f), "fence: a fence once set");
    glFinishFenceNV(f);
    ok(glTestFenceNV(f), "fence: done after a finish");
    glGetFenceivNV(f, 0x84F3, &v);
    ieq(v, GL_TRUE, "fence: its status");
    glGetFenceivNV(f, 0x84F4, &v);
    ieq(v, 0x84F2, "fence: its condition");
    glSetFenceNV(f, 0x1234);
    err(GL_INVALID_ENUM, "fence: a wrong condition");
    glDeleteFencesNV(1, &f);
    ok(!glIsFenceNV(f), "fence: gone");
    err(GL_NO_ERROR, "fence");
    ieq((int)glGetGraphicsResetStatusEXT(), GL_NO_ERROR, "robustness: no reset");
    glReadnPixelsEXT(W / 2, H / 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, 3, c);
    err(GL_INVALID_OPERATION, "robustness: a read into too little room");
    glReadnPixelsEXT(W / 2, H / 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, 4, c);
    ieq(c[0], 255, "robustness: a read that fits");
    loc = glGetUniformLocation(g_col_prog, "c");
    glGetnUniformfvEXT(g_col_prog, loc, 8, u);
    err(GL_INVALID_OPERATION, "robustness: a uniform into too little room");
    glGetnUniformfvEXT(g_col_prog, loc, 16, u);
    ok(u[0] == 1.0f && u[1] == 0.0f, "robustness: a uniform that fits");
    err(GL_NO_ERROR, "robustness");
}

static void test_tex_storage(void)
{
    static const uint8_t red[4] = { 255, 0, 0, 255 }, green[4] = { 0, 255, 0, 255 };
    GETP(storage_t, glTexStorage2DEXT);
    GLuint t;
    GLint im = 0;
    glGenTextures(1, &t);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexStorage2DEXT(GL_TEXTURE_2D, 5, GL_RGBA8, 2, 2);
    err(GL_INVALID_OPERATION, "texture storage: more levels than the size has");
    glTexStorage2DEXT(GL_TEXTURE_2D, 2, 0x1234, 2, 2);
    err(GL_INVALID_ENUM, "texture storage: a format it does not know");
    glTexStorage2DEXT(GL_TEXTURE_2D, 2, GL_RGBA8, 2, 2);
    err(GL_NO_ERROR, "texture storage");
    glGetTexParameteriv(GL_TEXTURE_2D, 0x912F, &im);
    ieq(im, 1, "texture storage: immutable");
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, red);
    err(GL_INVALID_OPERATION, "texture storage: glTexImage2D on it");
    glTexSubImage2D(GL_TEXTURE_2D, 1, 0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, green);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, red);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 1, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, red);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, red);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 1, 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, red);
    err(GL_NO_ERROR, "texture storage: glTexSubImage2D on it");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glUseProgram(g_tex_prog);
    glUniform1i(glGetUniformLocation(g_tex_prog, "t"), 0);
    clear_to(0, 0, 0, 1);
    full(g_tex_prog);
    px(W / 2, H / 2, "texture storage: sampled, level 0 under magnification", 255, 0, 0, 255);
    clear_to(0, 0, 0, 1);
    rect(g_tex_prog, 0.0f, 0.0f, 2.0f / 640.0f, 2.0f / 480.0f);
    px(W / 2, H / 2, "texture storage: sampled, level 1 under minification", 0, 255, 0, 255);
    glDeleteTextures(1, &t);
    err(GL_NO_ERROR, "texture storage: sampling");
    /* two levels of the three a 4 x 4 has: complete all the same */
    {
        static const uint8_t blue[16 * 4] = { 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255,
                                              0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255 };
        glGenTextures(1, &t);
        glBindTexture(GL_TEXTURE_2D, t);
        glTexStorage2DEXT(GL_TEXTURE_2D, 2, GL_RGBA8, 4, 4);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 4, 4, GL_RGBA, GL_UNSIGNED_BYTE, blue);
        glTexSubImage2D(GL_TEXTURE_2D, 1, 0, 0, 2, 2, GL_RGBA, GL_UNSIGNED_BYTE, blue);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
        clear_to(0, 0, 0, 1);
        full(g_tex_prog);
        px(W / 2, H / 2, "texture storage: complete with fewer levels than its size has", 0, 0, 255, 255);
        glDeleteTextures(1, &t);
    }
}

static void test_blit(void)
{
    GETP(blit_t, glBlitFramebufferANGLE); GETP(msaa_t, glRenderbufferStorageMultisampleANGLE); GETP(dbufs_t, glDrawBuffersEXT);
    GLuint fb, rb;
    GLenum none = GL_NONE, back = GL_BACK;
    GLint read = -1;
    glGenFramebuffers(1, &fb);
    glGenRenderbuffers(1, &rb);
    glBindRenderbuffer(GL_RENDERBUFFER, rb);
    glRenderbufferStorageMultisampleANGLE(GL_RENDERBUFFER, 4, GL_RGBA8, W, H);
    err(GL_INVALID_VALUE, "multisampled storage: there are no samples");
    glRenderbufferStorageMultisampleANGLE(GL_RENDERBUFFER, 0, GL_RGBA8, W, H);
    err(GL_NO_ERROR, "multisampled storage of 0 samples");
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb);
    clear_to(1, 0, 0, 1);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    clear_to(0, 0, 1, 1);
    glBindFramebuffer(0x8CA8, fb);                              /* READ_FRAMEBUFFER_ANGLE */
    glBindFramebuffer(0x8CA9, 0);                               /* DRAW_FRAMEBUFFER_ANGLE */
    glGetIntegerv(0x8CAA, &read);
    ieq(read, (int)fb, "blit: the read binding");
    glBlitFramebufferANGLE(0, 0, W / 2, H / 2, 0, 0, W / 2, H / 2, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    err(GL_NO_ERROR, "blit");
    glBlitFramebufferANGLE(0, 0, W / 2, H / 2, 0, 0, W, H, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    err(GL_INVALID_OPERATION, "blit: no scaling");
    glBlitFramebufferANGLE(0, 0, 4, 4, 0, 0, 4, 4, GL_COLOR_BUFFER_BIT, GL_LINEAR);
    err(GL_INVALID_ENUM, "blit: nearest only");
    glBindFramebuffer(0x8CA8, 0);
    px(W / 4, H / 4, "blit: the copied corner", 255, 0, 0, 255);
    px(W * 3 / 4, H * 3 / 4, "blit: the rest", 0, 0, 255, 255);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fb);
    glDeleteRenderbuffers(1, &rb);
    color(0, 1, 0, 1);
    clear_to(0, 0, 0, 1);
    glDrawBuffersEXT(1, &none);
    full(g_col_prog);
    px(W / 2, H / 2, "draw buffers: GL_NONE writes nothing", 0, 0, 0, 255);
    glDrawBuffersEXT(1, &back);
    full(g_col_prog);
    px(W / 2, H / 2, "draw buffers: back again", 0, 255, 0, 255);
    glDrawBuffersEXT(2, &back);
    err(GL_INVALID_VALUE, "draw buffers: one output");
    err(GL_NO_ERROR, "draw buffers");
}

static void test_program_binary(void)
{
    GETP(progbin_get_t, glGetProgramBinaryOES); GETP(progbin_set_t, glProgramBinaryOES); GETP(trans_t, glGetTranslatedShaderSourceANGLE);
    static uint8_t bin[4096];
    GLint len = 0, n = 0, linked = 1, fmtv = 0;
    GLsizei got = 0;
    GLenum fmt = 0;
    GLuint p = prog("attribute vec2 a_pos; void main() { gl_Position = vec4(a_pos, 0.0, 1.0); }\n", "precision mediump float; uniform vec4 c; void main() { gl_FragColor = c.bgra; }\n"), q;
    GLuint s = shader(GL_FRAGMENT_SHADER, FS_COL);
    char text[512];
    glGetIntegerv(0x87FE, &n);
    ieq(n, 1, "program binary: one format");
    glGetIntegerv(0x87FF, &fmtv);
    glGetProgramiv(p, 0x8741, &len);
    ok(len > 16 && len < (GLint)sizeof bin, "program binary: its length");
    glGetProgramBinaryOES(p, 4, &got, &fmt, bin);
    err(GL_INVALID_OPERATION, "program binary: too little room");
    glGetProgramBinaryOES(p, (GLsizei)sizeof bin, &got, &fmt, bin);
    err(GL_NO_ERROR, "program binary: get");
    ieq(got, len, "program binary: the length it gave");
    ieq((int)fmt, fmtv, "program binary: the format it gave");
    glDeleteProgram(p);
    q = glCreateProgram();
    glProgramBinaryOES(q, fmt, bin, got);
    err(GL_NO_ERROR, "program binary: set");
    glGetProgramiv(q, GL_LINK_STATUS, &linked);
    ieq(linked, 1, "program binary: linked");
    glUseProgram(q);
    glUniform4f(glGetUniformLocation(q, "c"), 0, 0, 1, 1);
    clear_to(0, 0, 0, 1);
    full(q);
    px(W / 2, H / 2, "program binary: it draws, and it is that shader", 255, 0, 0, 255);
    glProgramBinaryOES(q, 0x1234, bin, got);
    err(GL_INVALID_ENUM, "program binary: a wrong format");
    bin[0] ^= 0xFF;
    glProgramBinaryOES(q, fmt, bin, got);
    glGetProgramiv(q, GL_LINK_STATUS, &linked);
    ieq(linked, 0, "program binary: a corrupt one does not link");
    glDeleteProgram(q);
    glGetShaderiv(s, 0x93A0, &len);
    ok(len > 8, "translated source: its length");
    glGetTranslatedShaderSourceANGLE(s, sizeof text, &got, text);
    ok(got > 8 && strstr(text, "gl_FragColor") != NULL, "translated source: the text");
    glDeleteShader(s);
    err(GL_NO_ERROR, "translated source");
}

/* these take arguments: called through a stub that takes none, stdcall leaves them on the stack and the caller crashes on the way out */
static void test_queries_and_stack(void)
{
    GETP(gen_t, glGenQueriesEXT); GETP(del_t, glDeleteQueriesEXT); GETP(isf_t, glIsQueryEXT); GETP(qbegin_t, glBeginQueryEXT); GETP(qend_t, glEndQueryEXT); GETP(qget_t, glGetQueryivEXT); GETP(getf_t, glGetQueryObjectuivEXT);
    GETP(tex3d_t, glTexImage3DOES);
    GLuint q = 0;
    GLint v = 7;
    volatile int guard = 0x12345678;
    glGenQueriesEXT(1, &q);
    ok(q != 0, "queries: a name");
    glBeginQueryEXT(0x8C2F, q);
    err(GL_INVALID_ENUM, "queries: no occlusion counter to begin");
    glEndQueryEXT(0x8C2F);
    err(GL_INVALID_ENUM, "queries: nothing to end");
    glGetQueryivEXT(0x8C2F, 0x8865, &v);
    err(GL_INVALID_ENUM, "queries: glGetQueryivEXT");
    glGetQueryObjectuivEXT(q, 0x8866, &v);
    err(GL_INVALID_OPERATION, "queries: glGetQueryObjectuivEXT");
    ok(!glIsQueryEXT(q), "queries: never begun");
    glDeleteQueriesEXT(1, &q);
    glTexImage3DOES(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    err(GL_INVALID_ENUM, "glTexImage3DOES: the target");
    ieq(guard, 0x12345678, "the stack survives");
}

#define GL_ETC1_RGB8_OES 0x8D64
#define T3 0x806F
#define WRAP_R 0x8072
typedef void (__stdcall *sub3d_t)(GLenum, GLint, GLint, GLint, GLint, GLsizei, GLsizei, GLsizei, GLenum, GLenum, const void *);
typedef void (__stdcall *copy3d_t)(GLenum, GLint, GLint, GLint, GLint, GLint, GLint, GLsizei, GLsizei);
typedef void (__stdcall *comp3d_t)(GLenum, GLint, GLenum, GLsizei, GLsizei, GLsizei, GLint, GLsizei, const void *);
typedef void (__stdcall *fbt3d_t)(GLenum, GLenum, GLenum, GLuint, GLint, GLint);
typedef void (__stdcall *stor3d_t)(GLenum, GLsizei, GLenum, GLsizei, GLsizei, GLsizei);

/* the texel at (x, y) of a w x h texture drawn over the whole window: ETC1's 8 bits a channel, alpha 1 */
static void texel(int x, int y, int w, int h, const char *what, int r, int g, int b)
{
    px((x * 2 + 1) * W / (2 * w), (y * 2 + 1) * H / (2 * h), what, r, g, b, 255);
}

static void no_compile(GLenum type, const char *src, const char *what);

static void test_etc1(void)
{
    /* two halves side by side, of 4 bits a channel: table 0, every texel index 0, +2 */
    static const uint8_t plain[8] = { 0xF0, 0x0F, 0x00, 0x00, 0, 0, 0, 0 };
    /* differential, the halves one above the other. Base (16, 8, 24) and + (3, -4, 0): (132, 66, 198) and (156, 33, 198), tables 2 and 7.
       (0, 0) index 1, +29; (1, 3) index 2, -47; (3, 1) index 3, -29; every other texel index 0: +9 on top, +47 below */
    static const uint8_t diff[8] = { 0x83, 0x44, 0xC0, 0x5F, 0x20, 0x80, 0x20, 0x01 };
    /* a level below: 2 x 2 is one block */
    static const uint8_t lv1[8] = { 0x00, 0x00, 0xFF, 0x00, 0, 0, 0, 0 };
    uint8_t two[16];
    GLint n = 0, f[8];
    GLuint t;
    int i, seen = 0;
    glGetIntegerv(GL_NUM_COMPRESSED_TEXTURE_FORMATS, &n);
    ieq(n, 5, "compressed formats");
    glGetIntegerv(GL_COMPRESSED_TEXTURE_FORMATS, f);
    for (i = 0; i < n && i < 8; i++) if (f[i] == GL_ETC1_RGB8_OES) seen = 1;
    ieq(seen, 1, "ETC1 among them");
    clear_to(0, 0, 0, 0);
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glCompressedTexImage2D(GL_TEXTURE_2D, 0, GL_ETC1_RGB8_OES, 4, 4, 0, sizeof plain, plain);
    err(GL_NO_ERROR, "ETC1");
    full(g_tex_prog);
    texel(0, 2, 4, 4, "ETC1 left half", 255, 2, 2);
    texel(3, 1, 4, 4, "ETC1 right half", 2, 255, 2);
    glCompressedTexImage2D(GL_TEXTURE_2D, 0, GL_ETC1_RGB8_OES, 4, 4, 0, sizeof diff, diff);
    err(GL_NO_ERROR, "ETC1 differential");
    full(g_tex_prog);
    texel(0, 0, 4, 4, "ETC1 index 1", 161, 95, 227);
    texel(1, 3, 4, 4, "ETC1 index 2, clamped below", 109, 0, 151);
    texel(3, 1, 4, 4, "ETC1 index 3", 103, 37, 169);
    texel(2, 1, 4, 4, "ETC1 index 0, top half", 141, 75, 207);
    texel(2, 2, 4, 4, "ETC1 index 0, bottom half", 203, 80, 245);
    /* 6 x 2: two blocks, of which the second shows two columns */
    memcpy(two, plain, 8); memcpy(two + 8, diff, 8);
    glCompressedTexImage2D(GL_TEXTURE_2D, 0, GL_ETC1_RGB8_OES, 6, 2, 0, sizeof two, two);
    err(GL_NO_ERROR, "ETC1 6 x 2");
    full(g_tex_prog);
    texel(1, 0, 6, 2, "ETC1 6 x 2, first block", 255, 2, 2);
    texel(5, 1, 6, 2, "ETC1 6 x 2, second block", 141, 75, 207);
    glCompressedTexImage2D(GL_TEXTURE_2D, 0, GL_ETC1_RGB8_OES, 6, 2, 0, 8, two);
    err(GL_INVALID_VALUE, "ETC1 size");
    /* no sub-images of any kind, no generated levels */
    glCompressedTexImage2D(GL_TEXTURE_2D, 0, GL_ETC1_RGB8_OES, 4, 4, 0, sizeof diff, diff);
    glCompressedTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 4, 4, GL_ETC1_RGB8_OES, sizeof diff, diff);
    err(GL_INVALID_OPERATION, "ETC1 glCompressedTexSubImage2D");
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, diff);
    err(GL_INVALID_OPERATION, "ETC1 glTexSubImage2D");
    glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, 1, 1);
    err(GL_INVALID_OPERATION, "ETC1 glCopyTexSubImage2D");
    glGenerateMipmap(GL_TEXTURE_2D);
    err(GL_INVALID_OPERATION, "ETC1 glGenerateMipmap");
    /* a chain of it, and a cube face */
    glCompressedTexImage2D(GL_TEXTURE_2D, 1, GL_ETC1_RGB8_OES, 2, 2, 0, sizeof lv1, lv1);
    glCompressedTexImage2D(GL_TEXTURE_2D, 2, GL_ETC1_RGB8_OES, 1, 1, 0, sizeof lv1, lv1);
    err(GL_NO_ERROR, "ETC1 levels");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
    rect(g_tex_prog, -1, -1, -1 + 4.0f / W, -1 + 4.0f / H);     /* 4 texels over 2 pixels */
    px(0, 0, "ETC1 level 1", 0, 0, 255, 255);
    glDeleteTextures(1, &t);
}

#define FS3 "#extension GL_OES_texture_3D : require\nprecision mediump float; precision mediump sampler3D; varying vec2 v_uv; uniform sampler3D t; uniform float r;\n"

static void show3(GLuint p, float r)
{
    glUseProgram(p);
    glUniform1f(glGetUniformLocation(p, "r"), r);
    clear_to(0, 0, 0, 0);
    full(p);
}

static void params3(GLint filter, GLint wrap)
{
    glTexParameteri(T3, GL_TEXTURE_MIN_FILTER, filter);
    glTexParameteri(T3, GL_TEXTURE_MAG_FILTER, filter);
    glTexParameteri(T3, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(T3, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(T3, WRAP_R, wrap);
}

static void test_tex3d(void)
{
    GETP(tex3d_t, glTexImage3DOES); GETP(sub3d_t, glTexSubImage3DOES); GETP(copy3d_t, glCopyTexSubImage3DOES); GETP(comp3d_t, glCompressedTexImage3DOES); GETP(fbt3d_t, glFramebufferTexture3DOES); GETP(stor3d_t, glTexStorage3DEXT);
    uint8_t v[4 * 4 * 4 * 4];
    GLuint t, p, pb, fb;
    GLint n = -1, sz;
    GLenum ty = 0;
    char name[8];
    int x, y, z;
    if (!strstr((const char *)glGetString(GL_EXTENSIONS), "GL_OES_texture_3D")) { printf("note: test_tex3d skipped, the board offers no GL_OES_texture_3D\n"); return; }
    for (z = 0; z < 4; z++) for (y = 0; y < 4; y++) for (x = 0; x < 4; x++) { uint8_t *o = v + ((z * 4 + y) * 4 + x) * 4; o[0] = (uint8_t)(x * 85); o[1] = (uint8_t)(y * 85); o[2] = (uint8_t)(z * 85);o[3] = 255; }
    glGetIntegerv(0x8073, &n);
    ieq(n, 128, "GL_MAX_3D_TEXTURE_SIZE_OES");
    p = prog(VS, FS3 "void main() { gl_FragColor = texture3D(t, vec3(v_uv, r)); }\n");
    pb = prog(VS, FS3 "void main() { gl_FragColor = texture3DProj(t, vec4(v_uv, r, 1.0) * 2.0, 10.0); }\n");     /* the bias reaches the last level */
    glGetActiveUniform(p, 0, sizeof name, NULL, &sz, &ty, name);
    if (ty != 0x8B5F) glGetActiveUniform(p, 1, sizeof name, NULL, &sz, &ty, name);
    ieq((int)ty, 0x8B5F, "GL_SAMPLER_3D_OES");
    glGenTextures(1, &t);
    glBindTexture(T3, t);
    glGetIntegerv(0x806A, &n);
    ieq(n, (int)t, "GL_TEXTURE_BINDING_3D_OES");
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage3DOES(T3, 0, GL_RGBA, 4, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, v);
    err(GL_NO_ERROR, "glTexImage3DOES");
    show3(p, 0.5f);
    px(W / 2, H / 2, "3D with its default filter and no levels: incomplete", 0, 0, 0, 255);
    params3(GL_NEAREST, GL_CLAMP_TO_EDGE);
    glGetTexParameteriv(T3, WRAP_R, &n);
    ieq(n, GL_CLAMP_TO_EDGE, "TEXTURE_WRAP_R");
    for (z = 0; z < 4; z++) {
        show3(p, (z + 0.5f) / 4.0f);
        px(W / 8, H / 8, "3D nearest, a slice", 0, 0, z * 85, 255);
        px(W * 5 / 8, H * 7 / 8, "3D nearest, a slice", 170, 255, z * 85, 255);
    }
    params3(GL_LINEAR, GL_CLAMP_TO_EDGE);
    show3(p, 0.5f);
    px(W / 8, H / 8, "3D linear in r", 0, 0, 128, 255);
    params3(GL_NEAREST, GL_REPEAT);
    show3(p, 1.125f);
    px(W / 8, H / 8, "3D REPEAT in r", 0, 0, 0, 255);
    params3(GL_LINEAR, GL_REPEAT);
    show3(p, 0.0f);
    px(W / 8, H / 8, "3D LINEAR REPEAT: between the last slice and the first", 0, 0, 128, 255);
    params3(GL_NEAREST, GL_MIRRORED_REPEAT);
    show3(p, 1.125f);
    px(W / 8, H / 8, "3D MIRRORED_REPEAT in r", 0, 0, 255, 255);
    params3(GL_NEAREST, GL_CLAMP_TO_EDGE);
    show3(p, 1.125f);
    px(W / 8, H / 8, "3D CLAMP_TO_EDGE in r", 0, 0, 255, 255);
    /* a sub-image */
    {
        static const uint8_t white[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };
        glTexSubImage3DOES(T3, 0, 1, 2, 2, 1, 1, 2, GL_RGBA, GL_UNSIGNED_BYTE, white);
        err(GL_NO_ERROR, "glTexSubImage3DOES");
        show3(p, 3.5f / 4.0f);
        px(W * 3 / 8, H * 5 / 8, "3D sub-image, its second slice", 255, 255, 255, 255);
        glTexSubImage3DOES(T3, 0, 3, 3, 3, 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, white);
        err(GL_INVALID_VALUE, "3D sub-image past the edge");
    }
    /* a copy from the framebuffer into a slice */
    clear_to(0.5f, 0.25f, 0.75f, 1);
    glCopyTexSubImage3DOES(T3, 0, 0, 0, 1, 0, 0, 2, 2);
    err(GL_NO_ERROR, "glCopyTexSubImage3DOES");
    show3(p, 1.5f / 4.0f);
    px(W / 8, H / 8, "3D copy into a slice", 128, 64, 191, 255);
    px(W * 5 / 8, H / 8, "3D copy: beside it", 170, 0, 85, 255);
    /* into a slice as a target: a volume is no target of B's, the image in between goes in at the first read */
    {
        static const uint8_t none[8 * 8 * 4 * 4] = { 0 };
        GLuint vt;
        glGenFramebuffers(1, &fb);
        glBindFramebuffer(GL_FRAMEBUFFER, fb);
        glGenTextures(1, &vt);
        glBindTexture(T3, vt);
        glTexImage3DOES(T3, 0, GL_RGBA, 8, 8, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, none);
        params3(GL_NEAREST, GL_CLAMP_TO_EDGE);
        glFramebufferTexture3DOES(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, T3, vt, 0, 2);
        err(GL_NO_ERROR, "glFramebufferTexture3DOES");
        ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "a slice as a target");
        glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 0x8CD4, &n);
        ieq(n, 2, "ZOFFSET");
        glFramebufferTexture3DOES(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, T3, vt, 0, 4);
        ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT, "a slice past the depth");
        glFramebufferTexture3DOES(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, T3, vt, 0, 2);
        glViewport(0, 0, 8, 8);
        clear_to(0, 1, 0, 1);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDeleteFramebuffers(1, &fb);
        glViewport(0, 0, W, H);
        show3(p, 2.5f / 4.0f);
        px(W / 2, H / 2, "3D rendered into a slice", 0, 255, 0, 255);
        show3(p, 1.5f / 4.0f);
        px(W / 2, H / 2, "3D: the slice before it", 0, 0, 0, 0);
        glDeleteTextures(1, &vt);
        glBindTexture(T3, t);
    }
    glDeleteTextures(1, &t);
    /* the storage formats */
    {
        static const uint16_t c565[2] = { 31 << 11, 63 << 5 }, c4444[2] = { 0xF00F, 0x0F0F };
        static const uint8_t lum[2] = { 40, 200 };
        glGenTextures(1, &t);
        glBindTexture(T3, t);
        glTexImage3DOES(T3, 0, GL_RGB, 1, 1, 2, 0, GL_RGB, GL_UNSIGNED_SHORT_5_6_5, c565);
        params3(GL_NEAREST, GL_CLAMP_TO_EDGE);
        show3(p, 0.25f);
        px(W / 2, H / 2, "3D RGB565, slice 0", 255, 0, 0, 255);
        show3(p, 0.75f);
        px(W / 2, H / 2, "3D RGB565, slice 1", 0, 255, 0, 255);
        glTexImage3DOES(T3, 0, GL_RGBA, 1, 1, 2, 0, GL_RGBA, GL_UNSIGNED_SHORT_4_4_4_4, c4444);
        show3(p, 0.75f);
        px(W / 2, H / 2, "3D RGBA4444", 0, 255, 0, 255);
        glTexImage3DOES(T3, 0, GL_LUMINANCE, 1, 1, 2, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, lum);
        show3(p, 0.75f);
        px(W / 2, H / 2, "3D LUMINANCE", 200, 200, 200, 255);
        glDeleteTextures(1, &t);
    }
    /* rows of a slice and the alignment: 1 x 2 x 2 RGB, 4 bytes a row */
    {
        static const uint8_t pad[16] = { 1, 2, 3, 9, 4, 5, 6, 9, 7, 8, 9, 9, 40, 50, 60, 9 };
        glGenTextures(1, &t);
        glBindTexture(T3, t);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glTexImage3DOES(T3, 0, GL_RGB, 1, 2, 2, 0, GL_RGB, GL_UNSIGNED_BYTE, pad);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        params3(GL_NEAREST, GL_CLAMP_TO_EDGE);
        show3(p, 0.75f);
        px(W / 2, H * 3 / 4, "3D unpack alignment", 40, 50, 60, 255);
        glDeleteTextures(1, &t);
    }
    /* levels, bottom up: complete with the last one; a generated chain; storage for it */
    {
        static const uint8_t a8[3 * 2 * 5 * 4] = { 0 };
        static const uint8_t c1[4] = { 10, 20, 30, 255 };
        uint8_t hi[2 * 2 * 2 * 4];
        int l;
        glGenTextures(1, &t);
        glBindTexture(T3, t);
        glTexImage3DOES(T3, 2, GL_RGBA, 1, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, c1);
        glTexImage3DOES(T3, 0, GL_RGBA, 4, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, v);
        glTexParameteri(T3, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
        glTexParameteri(T3, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        show3(pb, 0.5f);
        px(W / 2, H / 2, "3D with a level missing: incomplete", 0, 0, 0, 255);
        glTexImage3DOES(T3, 1, GL_RGBA, 2, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, v);
        show3(pb, 0.5f);
        px(W / 2, H / 2, "3D level 2, sent before level 0", 10, 20, 30, 255);
        for (l = 0; l < 8 * 4; l += 4) { hi[l] = 16; hi[l + 1] = (uint8_t)(l * 8); hi[l + 2] = 200; hi[l + 3] = 255; }     /* green 0 .. 224, mean 112 */
        glTexImage3DOES(T3, 0, GL_RGBA, 2, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, hi);
        glGenerateMipmap(T3);
        err(GL_NO_ERROR, "glGenerateMipmap of a volume");
        show3(pb, 0.5f);
        px(W / 2, H / 2, "3D generated: the mean of 8", 16, 112, 200, 255);
        glDeleteTextures(1, &t);
        glGenTextures(1, &t);
        glBindTexture(T3, t);
        glTexStorage3DEXT(T3, 3, GL_RGBA8, 4, 2, 8);
        err(GL_NO_ERROR, "glTexStorage3DEXT");
        glTexImage3DOES(T3, 0, GL_RGBA, 4, 2, 8, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        err(GL_INVALID_OPERATION, "glTexStorage3DEXT is immutable");
        glTexSubImage3DOES(T3, 2, 0, 0, 1, 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, c1);
        err(GL_NO_ERROR, "a sub-image of its level 2");
        glTexParameteri(T3, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
        glTexParameteri(T3, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        show3(pb, 0.75f);
        px(W / 2, H / 2, "3D storage, level 2, slice 1", 10, 20, 30, 255);
        glDeleteTextures(1, &t);
        /* depth beyond the others, sizes that are no powers of two */
        glGenTextures(1, &t);
        glBindTexture(T3, t);
        glTexImage3DOES(T3, 0, GL_RGBA, 3, 2, 5, 0, GL_RGBA, GL_UNSIGNED_BYTE, a8);
        glTexSubImage3DOES(T3, 0, 2, 1, 4, 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, c1);
        params3(GL_NEAREST, GL_CLAMP_TO_EDGE);
        show3(p, 0.9f);
        px(W * 5 / 6, H * 3 / 4, "3D 3 x 2 x 5", 10, 20, 30, 255);
        glDeleteTextures(1, &t);
    }
    /* the largest, and what is refused */
    glGenTextures(1, &t);
    glBindTexture(T3, t);
    glTexImage3DOES(T3, 0, GL_RGBA, 256, 2, 255, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    err(GL_NO_ERROR, "the TMU's largest");
    glTexImage3DOES(T3, 0, GL_RGBA, 257, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    err(GL_INVALID_VALUE, "257 wide");
    glTexImage3DOES(T3, 0, GL_RGBA, 2, 2, 256, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    err(GL_INVALID_VALUE, "256 deep");
    glTexImage3DOES(T3, 0, GL_RGBA, 2, 2, 2, 1, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    err(GL_INVALID_VALUE, "a border");
    glTexImage3DOES(T3, 0, GL_RGB, 2, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    err(GL_INVALID_OPERATION, "internalformat != format");
    glCompressedTexImage3DOES(T3, 0, GL_ETC1_RGB8_OES, 4, 4, 4, 0, 32, v);
    err(GL_INVALID_ENUM, "no compressed volumes");
    glTexParameteri(GL_TEXTURE_2D, WRAP_R, GL_REPEAT);
    err(GL_INVALID_ENUM, "TEXTURE_WRAP_R of a 2D texture");
    glDeleteTextures(1, &t);
    glDeleteProgram(p); glDeleteProgram(pb);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    no_compile(GL_FRAGMENT_SHADER, "precision mediump float; uniform sampler3D t; void main() { }\n", "sampler3D with no #extension");
}

static void no_compile(GLenum type, const char *src, const char *what)
{
    GLuint s = glCreateShader(type);
    GLint okk = 1;
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    glGetShaderiv(s, GL_COMPILE_STATUS, &okk);
    ieq(okk, 0, what);
    glDeleteShader(s);
}

static const char VSUV[] = "attribute vec2 a_pos; attribute vec2 a_uv; varying vec2 v_uv; void main() { v_uv = a_uv; gl_Position = vec4(a_pos, 0.0, 1.0); }\n";

/* GLSL ES 1.00 on the real cores: what the semantic pass made legal, and what it refuses */
static void test_language_es100(void)
{
    static const char V[] = "attribute vec2 a_pos; void main() { gl_Position = vec4(a_pos, 0.0, 1.0); }\n";
#define FSH "precision mediump float;\n"
    GLuint p, f, v;
    GLint okk = 1;
    shader_scene("mat + mat, mat * float, in the fragment core", V, FSH "void main() { mat2 a = mat2(0.25); mat2 b = a + a; mat2 c = b * 0.5; gl_FragColor = vec4(b[0][0], c[0][0], c[0][1], 1.0); }\n", 128, 64, 0, 255);
    shader_scene("mat3 of a mat2, mat / mat", V, FSH "void main() { mat3 m = mat3(mat2(0.5)); mat2 d = mat2(0.5, 1.0, 1.0, 1.0) / mat2(1.0, 2.0, 2.0, 2.0); gl_FragColor = vec4(m[2][2], m[0][0], d[0][0], 1.0); }\n", 255, 128, 128, 255);
    shader_scene("mat + mat in the vertex core", "attribute vec2 a_pos; varying vec4 c; void main() { mat2 a = mat2(0.25); a = a + a; c = vec4(a[0][0], (a * 0.5)[0][0], 0.0, 1.0); gl_Position = vec4(a_pos, 0.0, 1.0); }\n", FSH "varying vec4 c; void main() { gl_FragColor = c; }\n", 128, 64, 0, 255);
    shader_scene("overloads, a prototype", V, FSH "float g(float);\nfloat f(float a) { return a * 0.5; }\nvec2 f(vec2 a) { return a * 0.25; }\nvoid main() { gl_FragColor = vec4(f(1.0), f(vec2(1.0)), g(0.25)); }\nfloat g(float a) { return a + 0.75; }\n", 128, 64, 64, 255);
    shader_scene("ints: * then / in order, ==, hex", V, FSH "uniform int z;\nvoid main() { int i = 4 + z; float a = float(7 * i / 2 * 2) / 56.0; gl_FragColor = vec4(a, i == 4 ? 1.0 : 0.0, float(0x10) / 32.0, 1.0); }\n", 128, 255, 128, 255);
    shader_scene("ivec / truncates, ivec of floats", V, FSH "void main() { ivec2 w = ivec2(9, 7) / ivec2(4, 2); ivec2 u = ivec2(vec2(1.75, 2.75)); gl_FragColor = vec4(float(w.x) * 0.25, float(w.y) * 0.25, float(u.x + u.y) * 0.25, 1.0); }\n", 128, 191, 191, 255);
    shader_scene("bool of a float, bvec of floats", V, FSH "void main() { float a = 0.25; gl_FragColor = vec4(float(bool(a)), vec3(bvec3(0.5, 0.0, -1.0)).yz, 1.0); }\n", 255, 0, 255, 255);
    shader_scene("struct ==, a struct in a block, two consts", V, FSH "const float x = 0.25, y = 0.5;\nstruct S { vec2 a; float b; };\nvoid main() { S p = S(vec2(x), y); S q = S(vec2(x), y); struct L { float k; }; L l = L(y); gl_FragColor = vec4(p == q ? 1.0 : 0.0, l.k, p != q ? 1.0 : 0.0, 1.0); }\n", 255, 128, 0, 255);
    shader_scene("?: runs one side, x++ gives the old", V, FSH "void main() { float a = 0.25; float q = gl_FragCoord.x > 0.0 ? (a = 0.5) : 0.75; float o = gl_FragCoord.y * 0.0 + 0.25; float s = o++; gl_FragColor = vec4(a, q, s, o - 1.0); }\n", 128, 128, 64, 64);
    shader_scene("a vector written and read at a run-time index", V, FSH "uniform float u;\nvoid main() { int k = 1 + int(u); vec4 c = vec4(0.0, 0.0, 0.0, 1.0); c[k] = 0.5; vec3 w = vec3(0.25, 0.5, 0.75); c.r = w[k + 1]; gl_FragColor = c; }\n", 191, 128, 0, 255);
    shader_scene("a loop on a local bound", V, FSH "uniform float u;\nvoid main() { int m = 3 + int(u); int i = 0; float s = 0.0; while (i < m) { s += 0.25; i++; } do { s += 0.125; } while (s < 0.0); gl_FragColor = vec4(s, 0.0, 0.0, 1.0); }\n", 223, 0, 0, 255);
    shader_scene("built-in constants, literals, #line", V, "#line 40\n" FSH "#if __LINE__ != 41\n#error line\n#endif\nvoid main() { gl_FragColor = vec4(float(gl_MaxVaryingVectors) * 0.125, float(gl_MaxDrawBuffers) * 0.5, float(010) * 0.0625, 1.0); }\n", 255, 128, 128, 255);
    shader_scene("gl_FragData[constant]", V, FSH "const int z = 0;\nvoid main() { gl_FragData[z] = vec4(0.5, 0.0, 0.0, 1.0); gl_FragData[z].g = 0.25; }\n", 128, 64, 0, 255);
    no_compile(GL_FRAGMENT_SHADER, "#version 300 es\nprecision mediump float;\nvoid main() { }\n", "#version 300 es is refused");
    no_compile(GL_FRAGMENT_SHADER, "void main() { float a = 1.0; }\n", "no precision for float");
    no_compile(GL_FRAGMENT_SHADER, FSH "void main() { float a = 1; }\n", "an int for a float");
    no_compile(GL_FRAGMENT_SHADER, FSH "float g(float a) { return g(a); }\nvoid main() { }\n", "recursion");
    no_compile(GL_FRAGMENT_SHADER, FSH "uniform float u;\nvoid main() { u = 1.0; }\n", "assignment to a uniform");
    no_compile(GL_FRAGMENT_SHADER, FSH "void main() { int switch = 1; }\n", "a reserved word");
    no_compile(GL_FRAGMENT_SHADER, "#extension GL_FOO : require\nvoid main() { }\n", "an unknown extension required");
    no_compile(GL_VERTEX_SHADER, "void main() { gl_FragColor = vec4(1.0); }\n", "gl_FragColor in a vertex shader");
    /* a sampler a vertex shader uses compiles, and the link refuses it: MAX_VERTEX_TEXTURE_IMAGE_UNITS is 0 */
    v = glCreateShader(GL_VERTEX_SHADER);
    {
        const char *src = "attribute vec2 a_pos; uniform sampler2D s; void main() { gl_Position = texture2DLod(s, a_pos, 0.0); }\n";
        glShaderSource(v, 1, &src, NULL);
    }
    glCompileShader(v);
    glGetShaderiv(v, GL_COMPILE_STATUS, &okk);
    ieq(okk, 1, "a vertex shader with a sampler compiles");
    f = shader(GL_FRAGMENT_SHADER, FS_COL);
    p = glCreateProgram();
    glAttachShader(p, v); glAttachShader(p, f);
    glLinkProgram(p);
    glGetProgramiv(p, GL_LINK_STATUS, &okk);
    ieq(okk, 0, "and does not link");
    glDeleteProgram(p); glDeleteShader(v); glDeleteShader(f);
    err(GL_NO_ERROR, "the language scenes");
}

typedef void (__stdcall *PGENVA)(GLsizei, GLuint *);
typedef void (__stdcall *PBINDVA)(GLuint);
typedef void (__stdcall *PDELVA)(GLsizei, const GLuint *);
typedef GLboolean (__stdcall *PISVA)(GLuint);
typedef void *(__stdcall *PMAP)(GLenum, GLenum);
typedef GLboolean (__stdcall *PUNMAP)(GLenum);
typedef void (__stdcall *PDISCARD)(GLenum, GLsizei, const GLenum *);

/* the extensions that were only host code: the TMU wraps any size and takes a level, the cores have DDX, DDY and STOZ */
/* Appendix A.7: what fits when packed must link, and every value must still arrive */
static void test_packing(void)
{
    static const char v12[] = "attribute vec2 a_pos; varying vec2 a, b, c, d, e, f, g, h, i, j, k, l;\n"
        "void main() { gl_Position = vec4(a_pos, 0.0, 1.0); a = vec2(0.1, 0.0); b = vec2(0.0, 0.2); c = vec2(0.1, 0.1); d = vec2(0.0); e = vec2(0.0); f = vec2(0.0); g = vec2(0.0); h = vec2(0.0); i = vec2(0.0); j = vec2(0.0); k = vec2(0.5, 0.0); l = vec2(0.0, 0.25); }\n";
    static const char f12[] = "precision mediump float; varying vec2 a, b, c, d, e, f, g, h, i, j, k, l;\n"
        "void main() { gl_FragColor = vec4(a.x + c.x + l.y + d.x + e.x + f.x + g.x + h.x, b.y + c.y + i.x + j.x, k.x, 1.0); }\n";
    static const char f33[] = "precision mediump float; varying float a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, d0, d1, d2;\n"
        "void main() { gl_FragColor = vec4(a0 + a1 + a2 + a3 + a4 + a5 + a6 + a7 + a8 + a9 + b0 + b1 + b2 + b3 + b4 + b5 + b6 + b7 + b8 + b9 + c0 + c1 + c2 + c3 + c4 + c5 + c6 + c7 + c8 + c9 + d0 + d1 + d2); }\n";
    static const char v33[] = "attribute vec2 a_pos; varying float a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, d0, d1, d2;\n"
        "void main() { gl_Position = vec4(a_pos, 0.0, 1.0); a0 = a1 = a2 = a3 = a4 = a5 = a6 = a7 = a8 = a9 = b0 = b1 = b2 = b3 = b4 = b5 = b6 = b7 = b8 = b9 = c0 = c1 = c2 = c3 = c4 = c5 = c6 = c7 = c8 = c9 = d0 = d1 = d2 = 0.01; }\n";
    static const char fu[] = "precision mediump float; uniform float u[40]; uniform float q[40]; uniform vec3 w[4];\n"
        "void main() { gl_FragColor = vec4(u[39], q[1], w[3].z, 1.0); }\n";
    static const char fp[] = "precision mediump float; uniform float u[70];\nvoid main() { gl_FragColor = vec4(u[69]); }\n";
    static const char vu[] = "attribute vec2 a_pos; uniform float s[120]; uniform float t[120]; uniform float m[50]; varying vec4 c;\n"
        "void main() { int i = int(a_pos.x * 0.0) + 119; gl_Position = vec4(a_pos, 0.0, 1.0); c = vec4(s[i], t[i - 1], m[49], 1.0); }\n";
    static const char fc[] = "precision mediump float; varying vec4 c; void main() { gl_FragColor = c; }\n";
    GLfloat v[80];
    GLuint p;
    int i, st = 0;
    shader_scene("12 vec2 varyings", v12, f12, 115, 76, 128, 255);
    p = glCreateProgram();
    glAttachShader(p, shader(GL_VERTEX_SHADER, v33));
    glAttachShader(p, shader(GL_FRAGMENT_SHADER, f33));
    glLinkProgram(p);
    glGetProgramiv(p, GL_LINK_STATUS, &st);
    ieq(st, 0, "33 floats do not pack into 8 vectors");
    glDeleteProgram(p);
    p = prog(VS, fu);
    for (i = 0; i < 40; i++) v[i] = i == 39 ? 0.25f : 0.0f;
    glUseProgram(p);
    glUniform1fv(glGetUniformLocation(p, "u"), 40, v);
    glUniform1f(glGetUniformLocation(p, "q[1]"), 0.5f);
    glUniform3f(glGetUniformLocation(p, "w[3]"), 0.0f, 0.0f, 0.75f);
    glGetUniformfv(p, glGetUniformLocation(p, "u[39]"), v);
    ok(v[0] == 0.25f, "a packed uniform reads back");
    clear_to(0, 0, 0, 1);
    full(p);
    px(32, 32, "84 uniform rows, packed", 64, 128, 191, 255);
    glDeleteProgram(p);
    p = glCreateProgram();
    glAttachShader(p, shader(GL_VERTEX_SHADER, VS));
    glAttachShader(p, shader(GL_FRAGMENT_SHADER, fp));
    glLinkProgram(p);
    glGetProgramiv(p, GL_LINK_STATUS, &st);
    ieq(st, 0, "an array longer than the fragment stage's rows does not pack");
    glDeleteProgram(p);
    p = prog(vu, fc);
    glUseProgram(p);
    glUniform1f(glGetUniformLocation(p, "s[119]"), 0.25f);
    glUniform1f(glGetUniformLocation(p, "t[118]"), 0.5f);
    glUniform1f(glGetUniformLocation(p, "m[49]"), 0.75f);
    clear_to(0, 0, 0, 1);
    full(p);
    px(32, 32, "290 vertex uniform rows, packed, a run-time index", 64, 128, 191, 255);
    glDeleteProgram(p);
    err(GL_NO_ERROR, "packing");
}

static void test_extensions(void)
{
    static const uint8_t rgb[12] = { 255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255 };
    static const uint8_t lv0[4 * 16] = { 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255, 255,0,0,255 };
    static const uint8_t lv1[4 * 4] = { 0,255,0,255, 0,255,0,255, 0,255,0,255, 0,255,0,255 };
    static const uint8_t lv2[4] = { 0, 0, 255, 255 };
    static const char *const names[] = { "GL_OES_texture_npot", "GL_OES_standard_derivatives", "GL_EXT_shader_texture_lod", "GL_EXT_frag_depth", "GL_OES_vertex_array_object", "GL_OES_mapbuffer", "GL_EXT_discard_framebuffer", NULL };
    const char *x = (const char *)glGetString(GL_EXTENSIONS);
    PGENVA genva = (PGENVA)eglGetProcAddress("glGenVertexArraysOES");
    PBINDVA bindva = (PBINDVA)eglGetProcAddress("glBindVertexArrayOES");
    PDELVA delva = (PDELVA)eglGetProcAddress("glDeleteVertexArraysOES");
    PISVA isva = (PISVA)eglGetProcAddress("glIsVertexArrayOES");
    PMAP mapb = (PMAP)eglGetProcAddress("glMapBufferOES");
    PUNMAP unmapb = (PUNMAP)eglGetProcAddress("glUnmapBufferOES");
    PDISCARD disc = (PDISCARD)eglGetProcAddress("glDiscardFramebufferEXT");
    GLuint t, p, vao, vbo, ebo;
    int i, f;
#define LODX "#extension GL_EXT_shader_texture_lod : enable\nprecision mediump float; varying vec2 v_uv;\n"
    for (i = 0; names[i]; i++) ok(strstr(x, names[i]) != NULL, names[i]);
    ok(strstr(x, "GL_OES_compressed_ETC1_RGB8_texture") != NULL, "GL_OES_compressed_ETC1_RGB8_texture");
    /* advertised only on a board whose TMU has volumes (gles_main.c has_3d): with VOL3D = 0 there are none, so its absence is the board's answer, not a failure */
    if (!strstr(x, "GL_OES_texture_3D")) printf("note: no GL_OES_texture_3D on this board (a TMU without volumes)\n");
    ok(genva && bindva && delva && isva && mapb && unmapb && disc, "the extension entry points");
    /* a size that is no power of two: REPEAT and MIRRORED_REPEAT */
    t = make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 3, 1, rgb, GL_NEAREST, GL_REPEAT);
    p = prog(VSUV, "precision mediump float; varying vec2 v_uv; uniform sampler2D t; void main() { gl_FragColor = texture2D(t, vec2(v_uv.x + 1.0, 0.5)); }\n");
    clear_to(0, 0, 0, 1);
    full(p);
    px(W / 6, H / 2, "NPOT repeat, first texel", 255, 0, 0, 255);
    px(W * 5 / 6, H / 2, "NPOT repeat, last texel", 0, 0, 255, 255);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
    full(p);
    px(W / 6, H / 2, "NPOT mirrored repeat: the last texel first", 0, 0, 255, 255);
    glDeleteProgram(p); glDeleteTextures(1, &t);
    /* an NPOT chain: 5 x 3, 2 x 1, 1 x 1 */
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    {
        uint8_t l0[4 * 15];
        for (i = 0; i < 15; i++) memcpy(l0 + 4 * i, lv0, 4);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 5, 3, 0, GL_RGBA, GL_UNSIGNED_BYTE, l0);
    }
    glTexImage2D(GL_TEXTURE_2D, 1, GL_RGBA, 2, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, lv1);
    glTexImage2D(GL_TEXTURE_2D, 2, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, lv2);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    p = prog(VSUV, LODX "uniform sampler2D t; uniform float l; void main() { gl_FragColor = texture2DLodEXT(t, v_uv, l); }\n");
    for (i = 0; i < 3; i++) {
        glUseProgram(p);
        glUniform1f(glGetUniformLocation(p, "l"), (float)i);
        full(p);
        px(W / 2, H / 2, "an NPOT chain, by texture2DLodEXT", i == 0 ? 255 : 0, i == 1 ? 255 : 0, i == 2 ? 255 : 0, 255);
    }
    glDeleteProgram(p); glDeleteTextures(1, &t);
    /* texture*LodEXT, *GradEXT on a chain of 4 x 4, 2 x 2, 1 x 1 */
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, lv0);
    glTexImage2D(GL_TEXTURE_2D, 1, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, lv1);
    glTexImage2D(GL_TEXTURE_2D, 2, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, lv2);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    p = prog(VSUV, LODX "uniform sampler2D t; void main() { gl_FragColor = texture2DProjLodEXT(t, vec3(v_uv * 2.0, 2.0), 1.0); }\n");
    full(p);
    px(W / 2, H / 2, "texture2DProjLodEXT", 0, 255, 0, 255);
    glDeleteProgram(p);
    p = prog(VSUV, LODX "uniform sampler2D t; void main() { gl_FragColor = texture2DGradEXT(t, v_uv, vec2(0.5, 0.0), vec2(0.0, 0.0)); }\n");
    full(p);
    px(W / 2, H / 2, "texture2DGradEXT: 2 texels a pixel is level 1", 0, 255, 0, 255);
    glDeleteProgram(p);
    p = prog(VSUV, LODX "uniform sampler2D t; void main() { gl_FragColor = texture2DProjGradEXT(t, vec4(v_uv, 0.0, 1.0), vec2(0.0, 1.0), vec2(0.0, 0.25)); }\n");
    full(p);
    px(W / 2, H / 2, "texture2DProjGradEXT: 4 texels a pixel is level 2", 0, 0, 255, 255);
    glDeleteProgram(p); glDeleteTextures(1, &t);
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_CUBE_MAP, t);
    for (f = 0; f < 6; f++) {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + (GLenum)f, 0, GL_RGBA, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, lv0);
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + (GLenum)f, 1, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, lv1);
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + (GLenum)f, 2, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, lv2);
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    p = prog(VSUV, LODX "uniform samplerCube t; void main() { gl_FragColor = textureCubeLodEXT(t, vec3(1.0, v_uv - 0.5), 2.0); }\n");
    full(p);
    px(W / 2, H / 2, "textureCubeLodEXT", 0, 0, 255, 255);
    glDeleteProgram(p);
    p = prog(VSUV, LODX "uniform samplerCube t; void main() { gl_FragColor = textureCubeGradEXT(t, vec3(1.0, v_uv - 0.5), vec3(0.0, 1.0, 0.0), vec3(0.0)); }\n");
    full(p);
    px(W / 2, H / 2, "textureCubeGradEXT: 2 texels a pixel is level 1", 0, 255, 0, 255);
    glDeleteProgram(p); glDeleteTextures(1, &t);
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
    /* derivatives */
    shader_scene("dFdx, dFdy, fwidth", VSUV, "#extension GL_OES_standard_derivatives : enable\nprecision highp float; varying vec2 v_uv; void main() { gl_FragColor = vec4(dFdx(v_uv.x) * 320.0, abs(dFdy(v_uv.y)) * 240.0, fwidth(v_uv.x) * 160.0, 1.0); }\n", 128, 128, 64, 255);
    /* gl_FragDepthEXT against what is drawn after it */
    p = prog("attribute vec2 a_pos; void main() { gl_Position = vec4(a_pos, 0.5, 1.0); }\n", "#extension GL_EXT_frag_depth : enable\nprecision mediump float; void main() { gl_FragDepthEXT = 0.25; gl_FragColor = vec4(0.0, 1.0, 0.0, 1.0); }\n");
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glClearDepthf(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    full(p);
    px(W / 2, H / 2, "gl_FragDepthEXT drew", 0, 255, 0, 255);
    zcolor(0.0f, 1, 0, 0, 1);
    full(g_zcol_prog);
    px(W / 2, H / 2, "depth 0.5 behind gl_FragDepthEXT 0.25", 0, 255, 0, 255);
    zcolor(-0.75f, 0, 0, 1, 1);
    full(g_zcol_prog);
    px(W / 2, H / 2, "depth 0.125 in front of it", 0, 0, 255, 255);
    glDisable(GL_DEPTH_TEST);
    glDeleteProgram(p);
    /* gl_DepthRange in both stages */
    glDepthRangef(0.25f, 0.75f);
    p = prog("attribute vec2 a_pos; varying float d; void main() { d = gl_DepthRange.diff; gl_Position = vec4(a_pos, 0.0, 1.0); }\n", "precision mediump float; varying float d; void main() { gl_FragColor = vec4(gl_DepthRange.near, gl_DepthRange.far, d, 1.0); }\n");
    full(p);
    px(W / 2, H / 2, "gl_DepthRange", 64, 191, 128, 255);
    ieq(glGetUniformLocation(p, "gl_DepthRange.near"), -1, "a gl_ name has no location");
    glDepthRangef(0.0f, 1.0f);
    full(p);
    px(W / 2, H / 2, "gl_DepthRange follows glDepthRangef", 0, 255, 255, 255);
    glDeleteProgram(p);
    /* vertex array objects: state goes with the object, the default object comes back */
    {
        static const GLfloat v[8] = { -1, -1, 1, -1, -1, 1, 1, 1 };
        static const GLushort e[4] = { 0, 1, 2, 3 };
        GLint a = glGetAttribLocation(g_col_prog, "a_pos"), cur = 7;
        genva(1, &vao);
        ok(!isva(vao) == 0, "glIsVertexArrayOES of a new object");
        bindva(vao);
        glGetIntegerv(0x85B5, &cur);
        ieq(cur, (int)vao, "GL_VERTEX_ARRAY_BINDING_OES");
        glGenBuffers(1, &vbo); glGenBuffers(1, &ebo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof v, v, GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof e, e, GL_STATIC_DRAW);
        glVertexAttribPointer((GLuint)a, 2, GL_FLOAT, GL_FALSE, 0, NULL);
        glEnableVertexAttribArray((GLuint)a);
        bindva(0);
        glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &cur);
        ieq(cur, 0, "the element buffer goes with the object");
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        clear_to(0, 0, 0, 1);
        color(0.5f, 0.25f, 0, 1);
        bindva(vao);
        glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, NULL);
        px(W / 2, H / 2, "a draw from a vertex array object", 128, 64, 0, 255);
        /* a mapped buffer: the bytes written reach the next draw */
        {
            GLfloat *m;
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            m = (GLfloat *)mapb(GL_ARRAY_BUFFER, 0x88B9);
            ok(m != NULL, "glMapBufferOES");
            if (m) {
                m[0] = -1; m[1] = -1; m[2] = 0; m[3] = -1; m[4] = -1; m[5] = 1; m[6] = 0; m[7] = 1;
                glBufferSubData(GL_ARRAY_BUFFER, 0, 4, v);
                err(GL_INVALID_OPERATION, "glBufferSubData of a mapped buffer");
                ok(unmapb(GL_ARRAY_BUFFER) == GL_TRUE, "glUnmapBufferOES");
                clear_to(0, 0, 0, 1);
                glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, NULL);
                px(W / 4, H / 2, "the left half, as mapped", 128, 64, 0, 255);
                px(W * 3 / 4, H / 2, "not the right half", 0, 0, 0, 255);
            }
            unmapb(GL_ARRAY_BUFFER);
            err(GL_INVALID_OPERATION, "glUnmapBufferOES of a buffer not mapped");
        }
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        bindva(0);
        delva(1, &vao);
        ok(!isva(vao), "a deleted object");
        glDeleteBuffers(1, &vbo); glDeleteBuffers(1, &ebo);
    }
    {
        static const GLenum good[2] = { 0x1800, 0x1801 }, bad[1] = { GL_COLOR_ATTACHMENT0 };
        disc(GL_FRAMEBUFFER, 2, good);
        err(GL_NO_ERROR, "glDiscardFramebufferEXT");
        disc(GL_FRAMEBUFFER, 1, bad);
        err(GL_INVALID_ENUM, "glDiscardFramebufferEXT of an attachment on the default framebuffer");
    }
    err(GL_NO_ERROR, "the extension scenes");
}

#ifndef GL_DEPTH_STENCIL_OES
#define GL_DEPTH_STENCIL_OES 0x84F9
#define GL_UNSIGNED_INT_24_8_OES 0x84FA
#endif

/* OES_depth_texture, OES_packed_depth_stencil, a framebuffer of depth alone, OES_fbo_render_mipmap */
static void test_depth_textures(void)
{
    static const char FS_R[] = "precision mediump float; varying vec2 v_uv; uniform sampler2D t; void main() { gl_FragColor = vec4(texture2D(t, v_uv).rrr, texture2D(t, v_uv).a); }\n";
    static const char FS_L[] = "#extension GL_EXT_shader_texture_lod : enable\nprecision mediump float; varying vec2 v_uv; uniform sampler2D t; uniform float l; void main() { gl_FragColor = texture2DLodEXT(t, v_uv, l); }\n";
    static const char *const NAMES[] = { "GL_OES_depth_texture", "GL_OES_packed_depth_stencil", "GL_OES_fbo_render_mipmap", NULL };
    const char *ext = (const char *)glGetString(GL_EXTENSIONS);
    GLuint fb, c, d, p, lp;
    uint8_t white[4 * 64 * 64];
    int i;
    for (i = 0; NAMES[i]; i++) ok(strstr(ext, NAMES[i]) != NULL, NAMES[i]);
    p = prog(VS, FS_R);
    glGenFramebuffers(1, &fb);

    /* with a colour target */
    d = make_tex(GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, 64, 64, NULL, GL_NEAREST, GL_CLAMP_TO_EDGE);
    c = make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 64, 64, NULL, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, c, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, d, 0);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "a depth texture with colour");
    glViewport(0, 0, 64, 64);
    glEnable(GL_DEPTH_TEST);
    glClearDepthf(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    zcolor(0.0f, 1, 0, 0, 1);
    rect(g_zcol_prog, -1, -1, 0, 1);
    glDisable(GL_DEPTH_TEST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, W, H);
    glBindTexture(GL_TEXTURE_2D, d);
    full(p);
    px(W / 4, H / 2, "a depth texture sampled: drawn", 128, 128, 128, 255);
    px(3 * W / 4, H / 2, "a depth texture sampled: cleared", 255, 255, 255, 255);
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
    glDeleteTextures(1, &d);

    /* depth alone */
    d = make_tex(GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, 64, 64, NULL, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, d, 0);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "a framebuffer of depth alone");
    glViewport(0, 0, 64, 64);
    glEnable(GL_DEPTH_TEST);
    glClear(GL_DEPTH_BUFFER_BIT);
    zcolor(-0.5f, 1, 0, 0, 1);
    rect(g_zcol_prog, 0, -1, 1, 1);
    glDisable(GL_DEPTH_TEST);
    err(GL_NO_ERROR, "a draw with no colour");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, W, H);
    glBindTexture(GL_TEXTURE_2D, d);
    full(p);
    px(W / 4, H / 2, "depth alone: cleared", 255, 255, 255, 255);
    px(3 * W / 4, H / 2, "depth alone: drawn", 64, 64, 64, 255);
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
    glDeleteTextures(1, &d);

    /* depth and stencil in one texture */
    d = make_tex(GL_DEPTH_STENCIL_OES, GL_UNSIGNED_INT_24_8_OES, 64, 64, NULL, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, c, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, d, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_TEXTURE_2D, d, 0);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "a depth-stencil texture");
    glViewport(0, 0, 64, 64);
    glClearStencil(0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST); glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 3, 255); glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    zcolor(0.0f, 0, 0, 0, 1);
    rect(g_zcol_prog, -1, -1, 0, 1);
    glDisable(GL_DEPTH_TEST);
    glStencilFunc(GL_EQUAL, 3, 255); glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    color(0, 1, 0, 1);
    full(g_col_prog);
    glDisable(GL_STENCIL_TEST);
    glReadPixels(16, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, white);
    ieq(white[1], 255, "the stencil of a depth-stencil texture: drawn");
    glReadPixels(48, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, white);
    ieq(white[1], 0, "the stencil of a depth-stencil texture: cleared");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, W, H);
    glBindTexture(GL_TEXTURE_2D, d);
    full(p);
    px(W / 4, H / 2, "the depth of a depth-stencil texture", 128, 128, 128, 255);
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
    glDeleteTextures(1, &d);
    glDeleteTextures(1, &c);

    /* a level of a texture */
    memset(white, 255, sizeof white);
    c = make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 64, 64, white, GL_NEAREST_MIPMAP_NEAREST, GL_CLAMP_TO_EDGE);
    glGenerateMipmap(GL_TEXTURE_2D);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, c, 1);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "level 1 of a texture");
    glViewport(0, 0, 32, 32);
    clear_to(0, 0, 1, 1);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, W, H);
    lp = prog(VS, FS_L);
    glBindTexture(GL_TEXTURE_2D, c);
    glUseProgram(lp);
    glUniform1f(glGetUniformLocation(lp, "l"), 1.0f);
    full(lp);
    px(W / 2, H / 2, "level 1 rendered", 0, 0, 255, 255);
    glUniform1f(glGetUniformLocation(lp, "l"), 0.0f);
    full(lp);
    px(W / 2, H / 2, "level 0 left alone", 255, 255, 255, 255);
    glDeleteTextures(1, &c);
    glDeleteProgram(lp);
    glDeleteProgram(p);
    glDeleteFramebuffers(1, &fb);
    err(GL_NO_ERROR, "depth textures");
}

/* every pixel of a 200 x 40 target: the left half (x < 100) one colour, the right half another */
static void halves(const char *what, int r0, int g0, int b0, int r1, int g1, int b1)
{
    static uint8_t c[4 * 200 * 40];
    int i, bad = 0, first = -1;
    memset(c, 0x5A, sizeof c);
    glReadPixels(0, 0, 200, 40, GL_RGBA, GL_UNSIGNED_BYTE, c);
    for (i = 0; i < 200 * 40; i++) {
        const uint8_t *q = c + 4 * i;
        int l = i % 200 < 100, r = l ? r0 : r1, g = l ? g0 : g1, b = l ? b0 : b1;
        if (abs(q[0] - r) > 2 || abs(q[1] - g) > 2 || abs(q[2] - b) > 2) { if (first < 0) first = i; bad++; }
    }
    g_checks++;
    if (bad) {
        g_bad++;
        printf("FAIL %s: %d of 8000 pixels wrong, the first (%d, %d) = %d %d %d\n", what, bad, first % 200, first / 200, c[4 * first], c[4 * first + 1], c[4 * first + 2]);
    }
}

/* a depth-stencil image beside an RGB565 target whose rows are no multiple of 16 pixels: board B loads a depth row in twice the colour row's
   bursts (200 pixels: 13 bursts of colour, 26 of depth = 832 bytes, more than 200 x 4) and refuses a shorter pitch */
static void test_ds_rows(void)
{
    static const char FS_R[] = "precision mediump float; varying vec2 v_uv; uniform sampler2D t; void main() { gl_FragColor = vec4(texture2D(t, v_uv).rrr, 1.0); }\n";
    GLuint fb, rc, rd, c, d, p;
    glGenFramebuffers(1, &fb);
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    glViewport(0, 0, 200, 40);

    /* renderbuffers: RGB565 and DEPTH24_STENCIL8 */
    glGenRenderbuffers(1, &rc);
    glBindRenderbuffer(GL_RENDERBUFFER, rc);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB565, 200, 40);
    glGenRenderbuffers(1, &rd);
    glBindRenderbuffer(GL_RENDERBUFFER, rd);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, 200, 40);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rc);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rd);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rd);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "RGB565 200 x 40 with DEPTH24_STENCIL8");
    glClearColor(0, 0, 0, 1); glClearDepthf(1.0f); glClearStencil(0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST); glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 1, 255); glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    zcolor(0.0f, 1, 0, 0, 1);                                   /* depth 0.5, stencil 1 everywhere */
    full(g_zcol_prog);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    zcolor(0.5f, 0, 1, 0, 1);                                   /* behind: nowhere */
    full(g_zcol_prog);
    glStencilFunc(GL_EQUAL, 1, 255);
    zcolor(-0.5f, 0, 0, 1, 1);                                  /* in front, where the stencil is 1: the left half */
    rect(g_zcol_prog, -1, -1, 0, 1);
    glDisable(GL_DEPTH_TEST); glDisable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 0, 255);
    halves("RGB565 200 wide, depth and stencil renderbuffer", 0, 0, 255, 255, 0, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, 0);

    /* textures: RGB565 colour (rendered in an image of its own: 400-byte rows) and a depth texture whose own rows (800 bytes) are short */
    c = make_tex(GL_RGB, GL_UNSIGNED_SHORT_5_6_5, 200, 40, NULL, GL_NEAREST, GL_CLAMP_TO_EDGE);
    d = make_tex(GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, 200, 40, NULL, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, c, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, d, 0);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "RGB565 texture 200 x 40 with a depth texture");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    zcolor(0.0f, 1, 0, 0, 1);
    full(g_zcol_prog);
    zcolor(0.5f, 0, 1, 0, 1);
    full(g_zcol_prog);
    zcolor(-0.5f, 0, 0, 1, 1);
    rect(g_zcol_prog, -1, -1, 0, 1);
    glDisable(GL_DEPTH_TEST);
    halves("RGB565 texture 200 wide, depth texture", 0, 0, 255, 255, 0, 0);
    /* the depth texture as rendered: 0.25 on the left, 0.5 on the right */
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, rc);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, 200, 40);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rc);
    p = prog(VS, FS_R);
    glBindTexture(GL_TEXTURE_2D, d);
    full(p);
    halves("the depth texture 200 wide sampled", 64, 64, 64, 128, 128, 128);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, W, H);
    glDeleteProgram(p);
    glDeleteTextures(1, &c);
    glDeleteTextures(1, &d);
    glDeleteRenderbuffers(1, &rc);
    glDeleteRenderbuffers(1, &rd);
    glDeleteFramebuffers(1, &fb);
    err(GL_NO_ERROR, "depth rows beside RGB565");
}

/* ---------------------------------------------------------------- the API as dEQP-GLES2 checks it: one name space for shaders and programs, uniforms as given, format errors, framebuffer objects */
#ifndef GL_BGRA8_EXT
#define GL_BGRA8_EXT 0x93A1
#endif

static void test_name_space(void)
{
    GLuint sh = glCreateShader(GL_VERTEX_SHADER), pr = glCreateProgram(), none = 0x7FFFFFF0u, out[2];
    GLint v = 0, iv[4];
    GLfloat fv[4];
    GLsizei len = 0;
    char buf[64];
    /* a name of the other kind is INVALID_OPERATION, a name of neither INVALID_VALUE */
    glShaderSource(pr, 0, NULL, NULL); err(GL_INVALID_OPERATION, "glShaderSource of a program");
    glCompileShader(pr); err(GL_INVALID_OPERATION, "glCompileShader of a program");
    glCompileShader(none); err(GL_INVALID_VALUE, "glCompileShader of no object");
    glGetShaderiv(pr, GL_SHADER_TYPE, &v); err(GL_INVALID_OPERATION, "glGetShaderiv of a program");
    glGetShaderInfoLog(pr, sizeof buf, &len, buf); err(GL_INVALID_OPERATION, "glGetShaderInfoLog of a program");
    glGetShaderSource(pr, sizeof buf, &len, buf); err(GL_INVALID_OPERATION, "glGetShaderSource of a program");
    glAttachShader(sh, sh); err(GL_INVALID_OPERATION, "glAttachShader, a shader for the program");
    glAttachShader(pr, pr); err(GL_INVALID_OPERATION, "glAttachShader, a program for the shader");
    glAttachShader(pr, none); err(GL_INVALID_VALUE, "glAttachShader of no shader");
    glDetachShader(sh, sh); err(GL_INVALID_OPERATION, "glDetachShader, a shader for the program");
    glLinkProgram(sh); err(GL_INVALID_OPERATION, "glLinkProgram of a shader");
    glUseProgram(sh); err(GL_INVALID_OPERATION, "glUseProgram of a shader");
    glValidateProgram(sh); err(GL_INVALID_OPERATION, "glValidateProgram of a shader");
    glGetProgramiv(sh, GL_LINK_STATUS, &v); err(GL_INVALID_OPERATION, "glGetProgramiv of a shader");
    glGetProgramInfoLog(sh, sizeof buf, &len, buf); err(GL_INVALID_OPERATION, "glGetProgramInfoLog of a shader");
    glGetAttachedShaders(sh, 2, &len, out); err(GL_INVALID_OPERATION, "glGetAttachedShaders of a shader");
    glBindAttribLocation(sh, 0, "a"); err(GL_INVALID_OPERATION, "glBindAttribLocation of a shader");
    glGetAttribLocation(sh, "a"); err(GL_INVALID_OPERATION, "glGetAttribLocation of a shader");
    glGetAttribLocation(none, "a"); err(GL_INVALID_VALUE, "glGetAttribLocation of no object");
    glGetUniformLocation(none, "a"); err(GL_INVALID_VALUE, "glGetUniformLocation of no object");
    glGetUniformfv(none, 0, fv); err(GL_INVALID_VALUE, "glGetUniformfv of no object");
    glGetUniformiv(sh, 0, iv); err(GL_INVALID_OPERATION, "glGetUniformiv of a shader");
    glDeleteShader(pr); err(GL_INVALID_OPERATION, "glDeleteShader of a program");
    glDeleteProgram(sh); err(GL_INVALID_OPERATION, "glDeleteProgram of a shader");
    glDeleteShader(sh);
    glDeleteProgram(pr);
    err(GL_NO_ERROR, "name space");
}

static void test_uniform_values(void)
{
    static const GLfloat m3[9] = { -1, 0.1f, 4, 800, 13, 55, 12, 91, -55.1f }, two[2] = { 0.25f, 0.5f };
    GLuint p = prog(VS, "precision mediump float; uniform float one[1]; uniform int big; uniform bool b; uniform mat3 m; uniform vec4 single;\n"
                        "struct T { sampler2D s; float k; }; uniform T st;\n"
                        "void main() { gl_FragColor = vec4(one[0], float(big) * 0.0, b ? 1.0 : 0.0, m[2][2]) + single * st.k + texture2D(st.s, vec2(0.5)) * 0.0; }\n");
    GLint n = 0, mx = 0, size = 0, k, loc, iv[4];
    GLfloat fv[9];
    GLenum type = 0;
    char name[64];
    int arr = 0;
    glUseProgram(p);
    /* an array of one element is an array: its name reads "[0]", and the longest name with its NUL is the maximum length */
    glGetProgramiv(p, GL_ACTIVE_UNIFORMS, &n);
    for (k = 0; k < n; k++) {
        glGetActiveUniform(p, (GLuint)k, sizeof name, NULL, &size, &type, name);
        if (!strcmp(name, "one[0]") && size == 1 && type == GL_FLOAT) arr = 1;
    }
    ieq(arr, 1, "an array of one reads one[0]");
    glGetProgramiv(p, GL_ACTIVE_UNIFORM_MAX_LENGTH, &mx);
    ieq(mx, 7, "ACTIVE_UNIFORM_MAX_LENGTH: one[0] and its NUL");
    ok(glGetUniformLocation(p, "one") != -1 && glGetUniformLocation(p, "one") == glGetUniformLocation(p, "one[0]"), "one and one[0]");
    ieq(glGetUniformLocation(p, "big[0]"), -1, "no [0] on a uniform that is no array");
    glUniform1fv(glGetUniformLocation(p, "one"), 2, two); err(GL_NO_ERROR, "a count past an array's end is no error");
    glGetUniformfv(p, glGetUniformLocation(p, "one"), fv); ok(fv[0] == 0.25f, "an array of one reads back");
    glGetUniformfv(p, glGetUniformLocation(p, "one") + 1, fv); err(GL_INVALID_OPERATION, "glGetUniformfv past the array");
    /* values read back as given, not as the boards hold them */
    glUniform4f(glGetUniformLocation(p, "single"), 0.1f, -55.1f, 12.2f, 1.1f);
    glGetUniformfv(p, glGetUniformLocation(p, "single"), fv);
    ok(fv[0] == 0.1f && fv[1] == -55.1f && fv[2] == 12.2f && fv[3] == 1.1f, "a vec4 reads back exactly");
    glUniformMatrix3fv(glGetUniformLocation(p, "m"), 1, GL_FALSE, m3);
    glGetUniformfv(p, glGetUniformLocation(p, "m"), fv);
    ok(!memcmp(fv, m3, sizeof m3), "a mat3 reads back exactly");
    glUniform1i(glGetUniformLocation(p, "big"), 123456789);
    glGetUniformiv(p, glGetUniformLocation(p, "big"), iv); ieq(iv[0], 123456789, "an int reads back exactly");
    glUniform1f(glGetUniformLocation(p, "b"), 0.5f);
    glGetUniformiv(p, glGetUniformLocation(p, "b"), iv); ieq(iv[0], 1, "a bool set by a float reads 1");
    glGetUniformfv(p, glGetUniformLocation(p, "b"), fv); ok(fv[0] == 1.0f, "a bool reads 1.0");
    /* a sampler inside a struct takes Uniform1i{v} alone, in range */
    loc = glGetUniformLocation(p, "st.s");
    ok(loc != -1, "st.s has a location");
    glUniform1i(loc, 3); err(GL_NO_ERROR, "glUniform1i of a sampler in a struct");
    glGetUniformiv(p, loc, iv); ieq(iv[0], 3, "a sampler in a struct reads back");
    glUniform1i(loc, 8); err(GL_INVALID_VALUE, "a sampler in a struct, unit 8");
    glUniform1f(loc, 1.0f); err(GL_INVALID_OPERATION, "glUniform1f of a sampler in a struct");
    glDeleteProgram(p);
}

static void test_format_errors(void)
{
    static const uint8_t bgra[4] = { 30, 20, 10, 40 };
    GLuint t;
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexImage2D(GL_TEXTURE_2D, 0, 0, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL); err(GL_INVALID_VALUE, "internalformat 0");
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_SHORT_4_4_4_4, NULL); err(GL_INVALID_OPERATION, "RGB with UNSIGNED_SHORT_4_4_4_4");
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_SHORT_5_5_5_1, NULL); err(GL_INVALID_OPERATION, "RGB with UNSIGNED_SHORT_5_5_5_1");
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 16, 16, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_6_5, NULL); err(GL_INVALID_OPERATION, "a sub-image, RGBA with UNSIGNED_SHORT_5_6_5");
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, 0, GL_UNSIGNED_BYTE, NULL); err(GL_INVALID_ENUM, "a sub-image of format 0");
    /* EXT_texture_format_BGRA8888: BGRA8_EXT is the sized BGRA_EXT */
    glTexImage2D(GL_TEXTURE_2D, 0, GL_BGRA8_EXT, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL); err(GL_INVALID_OPERATION, "BGRA8_EXT with RGBA");
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_BGRA8_EXT, 1, 1, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, bgra); err(GL_NO_ERROR, "BGRA8_EXT");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    clear_to(0, 0, 0, 0);
    show_tex(t, "BGRA8_EXT", 10, 20, 30, 40);
}

static void test_fbo_objects(void)
{
    static const uint8_t green[4 * 16] = { 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255,
                                           0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255 };
    GLuint fb, fb2, t, cube, rb;
    GLint v = 0;
    glGenFramebuffers(1, &fb);
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    glViewport(0, 0, 4, 4);
    /* each attachment point keeps its own face and level */
    glGenTextures(1, &cube);
    glBindTexture(GL_TEXTURE_CUBE_MAP, cube);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, cube, 0);
    glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_CUBE_MAP_FACE, &v);
    ieq(v, GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, "the depth attachment's cube face");
    /* deleted while the bound framebuffer holds it, at any attachment point: detached */
    glDeleteTextures(1, &cube);
    glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE, &v);
    ieq(v, GL_NONE, "a texture deleted at the bound framebuffer's depth attachment");
    /* deleted while a framebuffer that is not bound holds it: that framebuffer keeps the object, and a new texture under the old name is another one */
    t = make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 4, 4, NULL, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t, 0);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "4 x 4 texture");
    clear_to(1, 0, 0, 1);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 4, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, green);
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    px(1, 1, "a deleted texture stays the framebuffer's image", 255, 0, 0, 255);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t, 0);
    px(1, 1, "the new texture under the old name", 0, 255, 0, 255);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
    glDeleteTextures(1, &t);
    /* the same for a renderbuffer */
    glGenRenderbuffers(1, &rb);
    glBindRenderbuffer(GL_RENDERBUFFER, rb);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, 4, 4);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb);
    clear_to(0, 0, 1, 1);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteRenderbuffers(1, &rb);
    glBindRenderbuffer(GL_RENDERBUFFER, rb);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, 4, 4);
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    px(1, 1, "a deleted renderbuffer stays the framebuffer's image", 0, 0, 255, 255);
    /* EXT_texture_format_BGRA8888: BGRA_EXT and BGRA8_EXT are colour-renderable renderbuffer formats */
    glRenderbufferStorage(GL_RENDERBUFFER, GL_BGRA_EXT, 4, 4); err(GL_NO_ERROR, "BGRA_EXT renderbuffer");
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_COMPLETE, "BGRA_EXT renderbuffer");
    clear_to(0.25f, 0.5f, 0.75f, 1);
    px(1, 1, "BGRA_EXT renderbuffer", 64, 128, 191, 255);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_BGRA8_EXT, 4, 4); err(GL_NO_ERROR, "BGRA8_EXT renderbuffer");
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_INTERNAL_FORMAT, &v); ieq(v, GL_BGRA8_EXT, "BGRA8_EXT internal format");
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_ALPHA_SIZE, &v); ieq(v, 8, "BGRA8_EXT alpha bits");
    clear_to(0.75f, 0.5f, 0.25f, 0.5f);
    px(1, 1, "BGRA8_EXT renderbuffer", 191, 128, 64, 128);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_BGRA8_EXT, 16, 16);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rb);
    ieq((int)glCheckFramebufferStatus(GL_FRAMEBUFFER), GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT, "BGRA8_EXT at the depth attachment");
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA4, 1, 1281); err(GL_INVALID_VALUE, "renderbuffer height above MAX_RENDERBUFFER_SIZE");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteRenderbuffers(1, &rb);
    glDeleteFramebuffers(1, &fb);
    /* copies from a framebuffer that is not complete: INVALID_FRAMEBUFFER_OPERATION, of no pixels too, and the texture is left as it was */
    t = make_tex(GL_RGBA, GL_UNSIGNED_BYTE, 4, 4, green, GL_NEAREST, GL_CLAMP_TO_EDGE);
    glGenFramebuffers(1, &fb2);
    glBindFramebuffer(GL_FRAMEBUFFER, fb2);
    glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, 0, 0); err(GL_INVALID_FRAMEBUFFER_OPERATION, "glCopyTexSubImage2D of no pixels from an incomplete framebuffer");
    glCopyTexImage2D(GL_TEXTURE_2D, 1, GL_RGBA, 0, 0, 2, 2, 0); err(GL_INVALID_FRAMEBUFFER_OPERATION, "glCopyTexImage2D from an incomplete framebuffer");
    glTexSubImage2D(GL_TEXTURE_2D, 1, 0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, green); err(GL_INVALID_OPERATION, "the failed copy defined no level");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fb2);
    glDeleteTextures(1, &t);
    glViewport(0, 0, W, H);
    err(GL_NO_ERROR, "framebuffer objects");
}

static void test_state_conversions(void)
{
    GLfloat f = 0;
    GLint v = 0;
    /* §6.1.2: an unsigned mask read as a float stays unsigned; a float read as an integer rounds to the nearest, unless it is a colour or a depth value */
    glGetFloatv(GL_STENCIL_VALUE_MASK, &f); ok(f >= 255.0f, "STENCIL_VALUE_MASK as a float");
    glGetFloatv(GL_STENCIL_BACK_WRITEMASK, &f); ok(f >= 255.0f, "STENCIL_BACK_WRITEMASK as a float");
    glLineWidth(20.6f);
    glGetIntegerv(GL_LINE_WIDTH, &v); ieq(v, 21, "LINE_WIDTH as an integer");
    glLineWidth(1.0f);
    glPolygonOffset(-4770.93f, 39542.6f);
    glGetIntegerv(GL_POLYGON_OFFSET_FACTOR, &v); ieq(v, -4771, "POLYGON_OFFSET_FACTOR as an integer");
    glGetIntegerv(GL_POLYGON_OFFSET_UNITS, &v); ieq(v, 39543, "POLYGON_OFFSET_UNITS as an integer");
    glPolygonOffset(0, 0);
    glSampleCoverage(0.75f, GL_FALSE);
    glGetIntegerv(GL_SAMPLE_COVERAGE_VALUE, &v); ieq(v, 1, "SAMPLE_COVERAGE_VALUE rounds, it is no colour");
    glSampleCoverage(1.0f, GL_FALSE);
    err(GL_NO_ERROR, "state conversions");
}

static void test_read_draw_errors(void)
{
    static const uint8_t idx[1] = { 0 };
    uint8_t c[16];
    GLuint fb;
    glReadPixels(0, 0, 1, 1, GL_LUMINANCE_ALPHA, GL_UNSIGNED_SHORT_4_4_4_4, c); err(GL_INVALID_OPERATION, "glReadPixels LUMINANCE_ALPHA, UNSIGNED_SHORT_4_4_4_4");
    glReadPixels(0, 0, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, c); err(GL_INVALID_OPERATION, "glReadPixels RGB, UNSIGNED_BYTE");
    glReadPixels(0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_SHORT_5_6_5, c); err(GL_INVALID_OPERATION, "glReadPixels RGBA, UNSIGNED_SHORT_5_6_5");
    glReadPixels(0, 0, 1, 1, 0x1234, GL_UNSIGNED_BYTE, c); err(GL_INVALID_ENUM, "glReadPixels of format 0x1234");
    glReadPixels(0, 0, 1, 1, GL_RGBA, GL_FLOAT, c); err(GL_INVALID_ENUM, "glReadPixels of type FLOAT");
    glGenFramebuffers(1, &fb);
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    glReadPixels(0, 0, 0, 0, GL_RGBA, GL_UNSIGNED_BYTE, c); err(GL_INVALID_FRAMEBUFFER_OPERATION, "glReadPixels of no pixels, incomplete framebuffer");
    /* a draw into a framebuffer that is not complete: an error also with no program, and of no vertices */
    glUseProgram(0);
    glDrawArrays(GL_POINTS, 0, 1); err(GL_INVALID_FRAMEBUFFER_OPERATION, "glDrawArrays, incomplete framebuffer, no program");
    glDrawArrays(GL_POINTS, 0, 0); err(GL_INVALID_FRAMEBUFFER_OPERATION, "glDrawArrays of nothing, incomplete framebuffer");
    glDrawElements(GL_POINTS, 1, GL_UNSIGNED_BYTE, idx); err(GL_INVALID_FRAMEBUFFER_OPERATION, "glDrawElements, incomplete framebuffer, no program");
    glDrawElements(GL_POINTS, 0, GL_UNSIGNED_BYTE, idx); err(GL_INVALID_FRAMEBUFFER_OPERATION, "glDrawElements of nothing, incomplete framebuffer");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fb);
    glDrawElements(0x77, 0, GL_UNSIGNED_BYTE, idx); err(GL_INVALID_ENUM, "glDrawElements of nothing, mode 0x77");
    glDrawArrays(GL_POINTS, 0, 1); err(GL_NO_ERROR, "glDrawArrays with no program");
}

/* wide points against a viewport smaller than the window: the scissor keeps them to it, and a point whose centre is just outside draws its part inside */
static void test_point_viewport(void)
{
    static const GLfloat pts[12] = { 0.0f, 0.0f, 0.0f, 1.0f,  1.01f, 0.0f, 0.0f, 1.0f,  1.3f, 0.0f, 0.0f, 1.0f };
    GLuint p = prog("attribute vec4 a_pos; void main() { gl_Position = a_pos; gl_PointSize = 5.0; }\n", "precision mediump float; void main() { gl_FragColor = vec4(1.0); }\n");
    GLint a = glGetAttribLocation(p, "a_pos");
    clear_to(0, 0, 0, 1);
    glViewport(100, 100, 100, 100);
    glUseProgram(p);
    glVertexAttribPointer((GLuint)a, 4, GL_FLOAT, GL_FALSE, 0, pts);
    glEnableVertexAttribArray((GLuint)a);
    glDrawArrays(GL_POINTS, 0, 3);
    glDisableVertexAttribArray((GLuint)a);
    glViewport(0, 0, W, H);
    px(150, 150, "a wide point inside the viewport", 255, 255, 255, 255);
    px(199, 150, "a wide point whose centre is outside: its part inside the viewport", 255, 255, 255, 255);
    px(201, 150, "nothing beside the viewport", 0, 0, 0, 255);
    px(215, 150, "a point far outside the view volume", 0, 0, 0, 255);
    glDeleteProgram(p);
    err(GL_NO_ERROR, "points and the viewport");
}

/* EGL 1.4 §3.9.3: eglSwapBuffers on a pbuffer has no effect: its colour buffer keeps what was drawn */
static void test_pbuffer_swap(void)
{
    static const EGLint pa[] = { 0x3057, 64, 0x3056, 64, 0x3038 };
    EGLSurface pb = eglCreatePbufferSurface(g_dpy, g_cfg, pa);
    ok(pb != NULL, "eglCreatePbufferSurface");
    if (!pb) return;
    ok(eglMakeCurrent(g_dpy, pb, pb, g_ctx), "eglMakeCurrent on a pbuffer");
    clear_to(0.25f, 0.5f, 0.75f, 1);
    ok(eglSwapBuffers(g_dpy, pb), "eglSwapBuffers on a pbuffer");
    px(1, 1, "a pbuffer after eglSwapBuffers", 64, 128, 191, 255);
    ok(eglSwapBuffers(g_dpy, pb), "eglSwapBuffers on a pbuffer, again");
    px(1, 1, "a pbuffer after two eglSwapBuffers", 64, 128, 191, 255);
    eglMakeCurrent(g_dpy, g_surf, g_surf, g_ctx);
    ok(eglDestroySurface(g_dpy, pb), "eglDestroySurface");
    err(GL_NO_ERROR, "pbuffer swap");
}

int main(void)
{
    static const EGLint want[] = { 0x3024, 8, 0x3023, 8, 0x3022, 8, 0x3021, 8, 0x3025, 24, 0x3026, 8, 0x3033, 4, 0x3040, 4, 0x3038 };
    static const EGLint ca[] = { 0x3098, 2, 0x3038 };
    WNDCLASSA wc;
    RECT r = { 0, 0, W, H };
    HWND wnd;
    EGLint n = 0, maj = 0, min = 0, sz = 0;
    setvbuf(stdout, NULL, _IONBF, 0);
    memset(&wc, 0, sizeof wc);
    wc.lpfnWndProc = wnd_proc; wc.hInstance = GetModuleHandleA(NULL); wc.lpszClassName = "gles_test";
    RegisterClassA(&wc);
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    wnd = CreateWindowA("gles_test", "gles_test", WS_OVERLAPPEDWINDOW, 40, 40, r.right - r.left, r.bottom - r.top, NULL, NULL, wc.hInstance, NULL);
    ShowWindow(wnd, SW_SHOWNOACTIVATE);
    g_dpy = eglGetDisplay(NULL);
    ok(g_dpy != NULL, "eglGetDisplay");
    ok(eglInitialize(g_dpy, &maj, &min) && maj == 1, "eglInitialize");
    ok(eglChooseConfig(g_dpy, want, &g_cfg, 1, &n) && n == 1, "eglChooseConfig");
    {
        static const EGLint msaa[] = { 0x3032, 1, 0x3031, 4, 0x3038 };
        EGLConfig x;
        EGLint k = 9;
        eglChooseConfig(g_dpy, msaa, &x, 1, &k);
        ieq(k, 0, "no multisample configs");
    }
    eglGetConfigAttrib(g_dpy, g_cfg, 0x3025, &sz);
    ieq(sz, 24, "config depth");
    g_surf = eglCreateWindowSurface(g_dpy, g_cfg, wnd, NULL);
    ok(g_surf != NULL, "eglCreateWindowSurface");
    g_ctx = eglCreateContext(g_dpy, g_cfg, NULL, ca);
    ok(g_ctx != NULL, "eglCreateContext");
    ok(eglMakeCurrent(g_dpy, g_surf, g_surf, g_ctx), "eglMakeCurrent");
    if (g_bad) { printf("no context, error 0x%x\n", eglGetError()); return 1; }
    eglQuerySurface(g_dpy, g_surf, 0x3057, &sz);
    ieq(sz, W, "surface width");
    printf("GL_RENDERER: %s\nGL_VERSION: %s\n", glGetString(GL_RENDERER), glGetString(GL_VERSION));
    {
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        ieq(vp[2], W, "the viewport starts as the surface");
    }
    g_tex_prog = prog(VS, FS_TEX);
    g_col_prog = prog("attribute vec2 a_pos; void main() { gl_Position = vec4(a_pos, 0.0, 1.0); }\n", FS_COL);
    g_zcol_prog = prog(VS_Z, FS_COL);
    glClearColor(0.2f, 0.4f, 0.6f, 1.0f);
    glClearDepthf(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    px(W / 2, H / 2, "clear", 51, 102, 153, 255);
    err(GL_NO_ERROR, "set-up");
    test_varyings();
    test_attributes();
    test_uniforms();
    test_formats();
    test_complete();
    test_mipmaps();
    test_cube();
    test_fbo();
    test_fragment_ops();
    test_primitives();
    test_indices();
    test_copies();
    test_queries();
    test_contexts();
    test_language();
    test_runtime_index();
    test_lanes();
    test_shadertoy();
    test_kept();
    test_compiler_gaps();
    test_language_es100();
    test_packing();
    test_extensions();
    test_instanced();
    test_fence_robust();
    test_tex_storage();
    test_etc1();
    test_tex3d();
    test_depth_textures();
    test_ds_rows();
    test_blit();
    test_program_binary();
    test_queries_and_stack();
    test_name_space();
    test_uniform_values();
    test_format_errors();
    test_fbo_objects();
    test_state_conversions();
    test_read_draw_errors();
    test_point_viewport();
    test_scaled();
    test_configs();
    test_pbuffer_swap();
    test_swap();
    printf("%d checks, %d failed\n", g_checks, g_bad);
    if (!g_bad) printf("OK gles_test\n");
    return g_bad ? 1 : 0;
}
