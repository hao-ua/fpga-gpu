/* gles_shader — a fragment shader file drawn over the whole window through libEGL.dll + libGLESv2.dll, the way an application would: the source goes to glShaderSource as it is.
 *   gles_shader <file.frag> [-w W] [-h H] [-frames N] [-time T] [-dt D] [-shot out.ppm]
 * The shader may declare `uniform vec2 u_Resolution` (the window, pixels) and `uniform float u_Time` (T + D per frame; defaults 0 and 0.05). Prints the compile / link log, each frame's time and the
 * rate over the frames after the first. -shot: the first frame read back with glReadPixels and written as a binary PPM (top row first).
 *
 * What a source leaves out, this program puts in front of it and says so — the LIBRARY stays exact GLSL ES 1.00 and refuses such a source, as the language says it must:
 *   `precision mediump float;`                        if the source states no float precision (the fragment language has no default);
 *   uniform vec3 iResolution, float iTime, float iTimeDelta, int iFrame, vec4 iMouse (= 0)      each one the source names and does not declare (the Shadertoy inputs);
 *   tanh / sinh / cosh                                if it calls them and does not define them (GLSL ES 3.00 functions, written out in ES 1.00);
 *   `void main() { vec4 c = vec4(0.0); mainImage(c, gl_FragCoord.xy); gl_FragColor = c; }`      if it defines mainImage and no main (a Shadertoy source).
 * The source's own lines are not touched: what it leaves undefined in GLSL (a local read before it is written — WebGL zeroes those, this compiler does not) stays undefined. */
#define _GDI32_
#define GL_GLEXT_PROTOTYPES
#include <windows.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef void *EGLDisplay, *EGLConfig, *EGLSurface, *EGLContext;
typedef int EGLint;
#define EGLAPI __stdcall
EGLDisplay EGLAPI eglGetDisplay(void *);
int EGLAPI eglInitialize(EGLDisplay, EGLint *, EGLint *);
int EGLAPI eglChooseConfig(EGLDisplay, const EGLint *, EGLConfig *, EGLint, EGLint *);
EGLSurface EGLAPI eglCreateWindowSurface(EGLDisplay, EGLConfig, HWND, const EGLint *);
EGLContext EGLAPI eglCreateContext(EGLDisplay, EGLConfig, EGLContext, const EGLint *);
int EGLAPI eglMakeCurrent(EGLDisplay, EGLSurface, EGLSurface, EGLContext);
int EGLAPI eglSwapBuffers(EGLDisplay, EGLSurface);
EGLint EGLAPI eglGetError(void);

static const char VS[] = "attribute vec2 a_Pos;\nvoid main() { gl_Position = vec4(a_Pos, 0.0, 1.0); }\n";

static LRESULT CALLBACK wnd_proc(HWND h, UINT m, WPARAM w, LPARAM l) { return DefWindowProcA(h, m, w, l); }
static double now_ms(void) { LARGE_INTEGER f, c; QueryPerformanceFrequency(&f); QueryPerformanceCounter(&c); return 1000.0 * (double)c.QuadPart / (double)f.QuadPart; }

static char *slurp(const char *path)
{
    FILE *f = fopen(path, "rb");
    long n;
    char *s;
    if (!f) return NULL;
    fseek(f, 0, SEEK_END); n = ftell(f); fseek(f, 0, SEEK_SET);
    s = (char *)malloc((size_t)n + 1);
    if (fread(s, 1, (size_t)n, f) != (size_t)n) { fclose(f); free(s); return NULL; }
    s[n] = 0;
    fclose(f);
    return s;
}

static int word_ch(char ch) { return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '_'; }

/* the next whole-word `name` in [s, end) (end NULL: to the end of the text), or NULL */
static const char *word_in(const char *s, const char *end, const char *name)
{
    size_t n = strlen(name);
    const char *p = s;
    while ((p = strstr(p, name)) != NULL && (!end || p + n <= end)) {
        if ((p == s || !word_ch(p[-1])) && !word_ch(p[n])) return p;
        p += n;
    }
    return NULL;
}

/* does a statement that starts with the word `lead` name `name` before its ';' ?  (uniform ... iTime;   precision mediump float;) */
static int stated(const char *src, const char *lead, const char *name)
{
    const char *p = src, *e;
    while ((p = word_in(p, NULL, lead)) != NULL) {
        e = strchr(p, ';');
        if (!e) e = p + strlen(p);
        if (word_in(p, e, name)) return 1;
        p = e;
    }
    return 0;
}

/* does the source define the function itself: `name(` straight after a float type */
static int defines_fn(const char *src, const char *name)
{
    static const char *const ty[4] = { "float", "vec2", "vec3", "vec4" };
    const char *p = src, *q;
    int t;
    while ((p = word_in(p, NULL, name)) != NULL) {
        q = p;
        while (q > src && (q[-1] == ' ' || q[-1] == '\t' || q[-1] == '\n' || q[-1] == '\r')) q--;
        for (t = 0; t < 4; t++) { size_t n = strlen(ty[t]); if ((size_t)(q - src) >= n && !strncmp(q - n, ty[t], n) && (q - n == src || !word_ch(q[-n - 1]))) return 1; }
        p += strlen(name);
    }
    return 0;
}

/* is the word `name` somewhere followed by '(' : a function of that name (the word alone may sit in a comment: "// main") */
static int has_fn(const char *src, const char *name)
{
    const char *p = src, *q;
    while ((p = word_in(p, NULL, name)) != NULL) {
        q = p + strlen(name);
        while (*q == ' ' || *q == '\t' || *q == '\n' || *q == '\r') q++;
        if (*q == '(') return 1;
        p += strlen(name);
    }
    return 0;
}

/* what a source leaves out, put in front of it (the header comment): the derivatives extension Shadertoy's page enables, a float precision, the Shadertoy inputs it names and
 * does not declare, tanh / sinh / cosh it calls and does not define, and main() around mainImage. NULL if nothing was missing: the source then goes to the library as it is */
static char *complete_source(const char *src)
{
    static const char *const fn[3][2] = {
        { "tanh", "{ return 1.0 - 2.0 / (exp(2.0 * x) + 1.0); }" },        /* the limits are right: exp -> inf gives 1, exp -> 0 gives -1 */
        { "sinh", "{ return 0.5 * (exp(x) - exp(-x)); }" },
        { "cosh", "{ return 0.5 * (exp(x) + exp(-x)); }" } };
    static const char *const ty[4] = { "float", "vec2", "vec3", "vec4" };
    static const char *const in[5][2] = { { "iResolution", "vec3" }, { "iTime", "float" }, { "iTimeDelta", "float" }, { "iFrame", "int" }, { "iMouse", "vec4" } };
    size_t cap = strlen(src) + 4096;
    char *o, said[512];
    int i, t, toy = has_fn(src, "mainImage") && !has_fn(src, "main");
    int deriv = (word_in(src, NULL, "dFdx") || word_in(src, NULL, "dFdy") || word_in(src, NULL, "fwidth")) && !word_in(src, NULL, "GL_OES_standard_derivatives");
    o = (char *)malloc(cap);
    said[0] = o[0] = 0;
    /* GLSL ES 1.00 has dFdx / dFdy / fwidth only with the extension, which Shadertoy's page enables; the directive goes before every other line */
    if (deriv) {
        strcpy(o, "#extension GL_OES_standard_derivatives : enable\n");
        strcat(said, ", #extension GL_OES_standard_derivatives : enable");
    }
    /* always added: the added lines use floats, and the source's own statement would come after them. A second statement in the source is legal and wins from there on */
    strcat(o, "precision mediump float;\n");
    if (!stated(src, "precision", "float")) strcat(said, ", precision mediump float");
    for (i = 0; i < 5; i++)
        if (word_in(src, NULL, in[i][0]) && !stated(src, "uniform", in[i][0])) {
            sprintf(o + strlen(o), "uniform %s %s;\n", in[i][1], in[i][0]);
            sprintf(said + strlen(said), ", uniform %s %s", in[i][1], in[i][0]);
        }
    for (i = 0; i < 3; i++) {
        if (!word_in(src, NULL, fn[i][0]) || defines_fn(src, fn[i][0])) continue;
        for (t = 0; t < 4; t++) sprintf(o + strlen(o), "%s %s(%s x) %s\n", ty[t], fn[i][0], ty[t], fn[i][1]);
        sprintf(said + strlen(said), ", %s() written in ES 1.00", fn[i][0]);
    }
    if (toy) strcat(said, ", main() around mainImage");
    if (!said[0]) { free(o); return NULL; }
    printf("added to the source: %s\n", said + 2);
    strcat(o, src);
    if (toy) strcat(o, "\nvoid main() { vec4 c = vec4(0.0); mainImage(c, gl_FragCoord.xy); gl_FragColor = c; }\n");
    return o;
}

static GLuint shader(GLenum type, const char *src, const char *what)
{
    GLuint s = glCreateShader(type);
    GLint ok = 0;
    char log[4096];
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    log[0] = 0;
    glGetShaderInfoLog(s, sizeof log, NULL, log);
    printf("%s shader: %s%s%s\n", what, ok ? "compiled" : "DID NOT COMPILE", log[0] ? ": " : "", log);
    return ok ? s : 0;
}

int main(int argc, char **argv)
{
    static const EGLint want[] = { 0x3024, 8, 0x3023, 8, 0x3022, 8, 0x3021, 8, 0x3025, 24, 0x3026, 8, 0x3033, 4, 0x3040, 4, 0x3038 };
    static const EGLint ca[] = { 0x3098, 2, 0x3038 };
    static const GLfloat quad[8] = { -1, -1, 1, -1, -1, 1, 1, 1 };
    int W = 640, H = 480, frames = 10, a, f;
    double t0 = 0.0, dt = 0.05, first = 0.0, start = 0.0, slowest = 0.0, fastest = 1e30;
    const char *path = NULL, *shot = NULL;
    char *src;
    WNDCLASSA wc;
    RECT r;
    HWND wnd;
    EGLDisplay dpy;
    EGLConfig cfg;
    EGLSurface surf;
    EGLContext ctx;
    EGLint n = 0, maj = 0, min = 0;
    GLuint vs, fs, p;
    GLint ok = 0, ures, utime, ires, itime, idt, ifr, imouse;
    char log[4096], *wrapped;
    setvbuf(stdout, NULL, _IONBF, 0);
    for (a = 1; a < argc; a++) {
        if (!strcmp(argv[a], "-w") && a + 1 < argc) W = atoi(argv[++a]);
        else if (!strcmp(argv[a], "-h") && a + 1 < argc) H = atoi(argv[++a]);
        else if (!strcmp(argv[a], "-frames") && a + 1 < argc) frames = atoi(argv[++a]);
        else if (!strcmp(argv[a], "-time") && a + 1 < argc) t0 = atof(argv[++a]);
        else if (!strcmp(argv[a], "-dt") && a + 1 < argc) dt = atof(argv[++a]);
        else if (!strcmp(argv[a], "-shot") && a + 1 < argc) shot = argv[++a];
        else path = argv[a];
    }
    if (!path || !(src = slurp(path))) { printf("usage: gles_shader <file.frag> [-w W] [-h H] [-frames N] [-time T] [-dt D] [-shot out.ppm]\n"); return 2; }
    memset(&wc, 0, sizeof wc);
    wc.lpfnWndProc = wnd_proc; wc.hInstance = GetModuleHandleA(NULL); wc.lpszClassName = "gles_shader";
    RegisterClassA(&wc);
    r.left = 0; r.top = 0; r.right = W; r.bottom = H;
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    wnd = CreateWindowA("gles_shader", "gles_shader", WS_OVERLAPPEDWINDOW, 40, 40, r.right - r.left, r.bottom - r.top, NULL, NULL, wc.hInstance, NULL);
    ShowWindow(wnd, SW_SHOWNOACTIVATE);
    dpy = eglGetDisplay(NULL);
    if (!dpy || !eglInitialize(dpy, &maj, &min) || !eglChooseConfig(dpy, want, &cfg, 1, &n) || n != 1) { printf("no EGL display / config, error 0x%x\n", eglGetError()); return 1; }
    surf = eglCreateWindowSurface(dpy, cfg, wnd, NULL);
    ctx = eglCreateContext(dpy, cfg, NULL, ca);
    if (!surf || !ctx || !eglMakeCurrent(dpy, surf, surf, ctx)) { printf("no context, error 0x%x\n", eglGetError()); return 1; }
    printf("GL_RENDERER: %s\nGL_VERSION: %s\nwindow %d x %d, %s\n", glGetString(GL_RENDERER), glGetString(GL_VERSION), W, H, path);
    wrapped = complete_source(src);
    vs = shader(GL_VERTEX_SHADER, VS, "vertex");
    fs = shader(GL_FRAGMENT_SHADER, wrapped ? wrapped : src, "fragment");
    if (!vs || !fs) return 1;
    p = glCreateProgram();
    glAttachShader(p, vs); glAttachShader(p, fs);
    glBindAttribLocation(p, 0, "a_Pos");
    glLinkProgram(p);
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    log[0] = 0;
    glGetProgramInfoLog(p, sizeof log, NULL, log);
    printf("program: %s%s%s\n", ok ? "linked" : "DID NOT LINK", log[0] ? ": " : "", log);
    if (!ok) return 1;
    glUseProgram(p);
    ures = glGetUniformLocation(p, "u_Resolution");
    utime = glGetUniformLocation(p, "u_Time");
    ires = glGetUniformLocation(p, "iResolution"); itime = glGetUniformLocation(p, "iTime"); idt = glGetUniformLocation(p, "iTimeDelta");
    ifr = glGetUniformLocation(p, "iFrame"); imouse = glGetUniformLocation(p, "iMouse");
    printf("uniforms: u_Resolution %s, u_Time %s%s%s\n", ures >= 0 ? "set" : "not in the program", utime >= 0 ? "set" : "not in the program",
           ires >= 0 ? ", iResolution set" : "", itime >= 0 ? ", iTime set" : "");
    if (ires >= 0) glUniform3f(ires, (GLfloat)W, (GLfloat)H, 1.0f);
    if (idt >= 0) glUniform1f(idt, (GLfloat)dt);
    if (imouse >= 0) glUniform4f(imouse, 0.0f, 0.0f, 0.0f, 0.0f);
    glViewport(0, 0, W, H);
    glDisable(GL_DEPTH_TEST);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, quad);
    glEnableVertexAttribArray(0);
    if (ures >= 0) glUniform2f(ures, (GLfloat)W, (GLfloat)H);
    for (f = 0; f < frames; f++) {
        MSG m;
        double ta = now_ms(), tb;
        while (PeekMessageA(&m, NULL, 0, 0, PM_REMOVE)) { TranslateMessage(&m); DispatchMessageA(&m); }
        if (utime >= 0) glUniform1f(utime, (GLfloat)(t0 + dt * f));
        if (itime >= 0) glUniform1f(itime, (GLfloat)(t0 + dt * f));
        if (ifr >= 0) glUniform1i(ifr, f);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        if (f == 0 && shot) {
            uint8_t *px = (uint8_t *)malloc((size_t)W * H * 4);
            FILE *o = fopen(shot, "wb");
            int x, y;
            glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
            if (o) {
                fprintf(o, "P6\n%d %d\n255\n", W, H);
                for (y = H - 1; y >= 0; y--) for (x = 0; x < W; x++) fwrite(px + ((size_t)y * W + x) * 4, 1, 3, o);
                fclose(o);
                printf("frame 0 read back: %s\n", shot);
            }
            else printf("cannot write %s\n", shot);
            free(px);
        }
        eglSwapBuffers(dpy, surf);
        glFinish();
        tb = now_ms();
        printf("frame %d (u_Time %.3f): %.1f ms%s\n", f, t0 + dt * f, tb - ta, glGetError() ? " GL ERROR" : "");
        if (f == 0) { first = tb - ta; start = tb; }
        else { if (tb - ta > slowest) slowest = tb - ta; if (tb - ta < fastest) fastest = tb - ta; }
    }
    if (frames > 1) {
        double per = (now_ms() - start) / (frames - 1);
        printf("first frame %.1f ms (compile, uploads%s); then %d frames: %.1f ms a frame = %.2f fps (fastest %.1f, slowest %.1f ms)\n", first, shot ? ", read-back" : "", frames - 1, per, 1000.0 / per, fastest, slowest);
    }
    eglMakeCurrent(dpy, NULL, NULL, NULL);
    return 0;
}
