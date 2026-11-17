// SPDX-License-Identifier: MIT
/*
 * fg_egl_draw: OpenGL ES 2.0 draws through the fpgagpu Mesa driver to the boards, read back and checked.
 *   The same context as fg_egl_clear (GBM on the render node, no surface, a 64 x 64 RGBA8 framebuffer object), then:
 *   1. a red triangle over the lower left half (GL's window coordinates), a map of the result printed;
 *   2. a triangle with red, green and blue corners: the colour near each corner;
 *   3. a 2 x 2 texture (red, green, blue, white; nearest) on a quad over the target: each quarter;
 *   4. a white quad of alpha 0.5 blended (SRC_ALPHA, ONE_MINUS_SRC_ALPHA) over black: 128 +-2;
 *   5. a 2 x 2 R8 texture with GL_TEXTURE_SWIZZLE_A = GL_RED (X's glamor reads glyphs so): alpha = red;
 *   6. a 16-pixel point coloured by gl_PointCoord in the framebuffer object: s grows right, t down;
 *   7. a function whose local variable has its parameter's name (GLSL ES 1.00 4.2.2: the body is a scope of its own);
 *   8. texture2DProj on a mipmapped texture whose levels are one colour each: every pixel's level against the levels
 *      the coordinate's derivatives at that pixel and its four neighbours allow (ES 2.0 3.7.7).
 *   cc -O2 -Wall -I<prefix>/include fg_egl_draw.c -o fg_egl_draw -L<libdir> -Wl,-rpath-link,<libdir> \
 *      -lEGL -lGLESv2 -lgbm -lm $(pkg-config --cflags --libs libdrm)
 *   fg_egl_draw [render node, default /dev/dri/renderD129]
 */
#include <execinfo.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <gbm.h>
#include <xf86drm.h>

#ifndef GL_TEXTURE_SWIZZLE_A
#define GL_TEXTURE_SWIZZLE_A 0x8E45
#endif

#define W 64
#define H 64

static uint8_t px[W * H * 4];

/* a crash prints where it happened (resolve the library offsets with addr2line) */
static void on_fault(int sig)
{
	void *f[48];
	int n = backtrace(f, 48);

	fprintf(stderr, "signal %d, backtrace:\n", sig);
	backtrace_symbols_fd(f, n, 2);
	_exit(128 + sig);
}

static const char *vs_src =
	"attribute vec4 pos;\n"
	"attribute vec4 col;\n"
	"varying vec4 v;\n"
	"void main() { gl_Position = pos; v = col; }\n";
static const char *fs_col =
	"precision mediump float;\n"
	"varying vec4 v;\n"
	"void main() { gl_FragColor = v; }\n";
static const char *fs_tex =
	"precision mediump float;\n"
	"varying vec4 v;\n"
	"uniform sampler2D t;\n"
	"void main() { gl_FragColor = texture2D(t, v.xy); }\n";

static GLuint program(const char *fs)
{
	GLuint p = glCreateProgram(), s[2];
	const char *src[2] = { vs_src, fs };
	GLint ok;
	char log[2048];

	for (int i = 0; i < 2; i++) {
		s[i] = glCreateShader(i ? GL_FRAGMENT_SHADER : GL_VERTEX_SHADER);
		glShaderSource(s[i], 1, &src[i], NULL);
		glCompileShader(s[i]);
		glGetShaderiv(s[i], GL_COMPILE_STATUS, &ok);
		if (!ok) {
			glGetShaderInfoLog(s[i], sizeof(log), NULL, log);
			printf("  shader compile: %s\n", log);
		}
		glAttachShader(p, s[i]);
	}
	glBindAttribLocation(p, 0, "pos");
	glBindAttribLocation(p, 1, "col");
	glLinkProgram(p);
	glGetProgramiv(p, GL_LINK_STATUS, &ok);
	if (!ok) {
		glGetProgramInfoLog(p, sizeof(log), NULL, log);
		printf("  link: %s\n", log);
	}
	return p;
}

static void draw(GLenum mode, const float *pos, const float *col, int n)
{
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, pos);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, col);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glDrawArrays(mode, 0, n);
}

static void read_back(void)
{
	memset(px, 0xA5, sizeof(px));
	glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
}

static const uint8_t *at(int x, int y)
{
	return px + 4 * (y * W + x);
}

static int near(const uint8_t *p, int r, int g, int b, int a, int tol)
{
	return abs(p[0] - r) <= tol && abs(p[1] - g) <= tol && abs(p[2] - b) <= tol && abs(p[3] - a) <= tol;
}

/* GL's window coordinates, y up: the top row printed first */
static void print_map(void)
{
	for (int y = H - 4; y >= 0; y -= 8) {
		printf("    ");
		for (int x = 4; x < W; x += 8) {
			const uint8_t *p = at(x, y);

			putchar(p[0] > 128 && p[1] < 64 ? 'R' : p[0] < 64 && p[1] < 64 && p[2] < 64 ? '.' : '?');
		}
		putchar('\n');
	}
}

static void check(int *fails, int ok, const char *what, const uint8_t *p)
{
	printf("  %-46s %s (%u, %u, %u, %u)\n", what, ok ? "ok  " : "BAD ", p[0], p[1], p[2], p[3]);
	*fails += !ok;
}

int main(int argc, char **argv)
{
	const char *node = argc > 1 ? argv[1] : "/dev/dri/renderD129";
	static const EGLint ctx_attr[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
	static const EGLint cfg_attr[] = { EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT, EGL_NONE };
	const float tri[6] = { -1, -1, 1, -1, -1, 1 };
	const float red3[12] = { 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1 };
	const float rgb3[12] = { 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1 };
	const float quad[8] = { -1, -1, 1, -1, -1, 1, 1, 1 };
	const float uv4[16] = { 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1 };
	const float half4[16] = { 1, 1, 1, 0.5f, 1, 1, 1, 0.5f, 1, 1, 1, 0.5f, 1, 1, 1, 0.5f };
	static const uint8_t texels[16] = { 255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255 };
	struct gbm_device *gbm;
	drmVersionPtr v;
	EGLDisplay dpy;
	EGLContext ctx;
	EGLConfig cfg = NULL;
	EGLint major, minor, n = 0;
	GLuint rb, fbo, pc, pt, tex;
	const char *renderer;
	int fd, fails = 0;

	signal(SIGSEGV, on_fault);
	signal(SIGABRT, on_fault);
	fd = open(node, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		perror(node);
		return 1;
	}
	v = drmGetVersion(fd);
	if (!v || !v->name || strcmp(v->name, "fpgagpu")) {
		printf("FAIL: %s is not the fpgagpu device\n", node);
		return 1;
	}
	drmFreeVersion(v);
	gbm = gbm_create_device(fd);
	dpy = gbm ? eglGetPlatformDisplay(EGL_PLATFORM_GBM_KHR, gbm, NULL) : EGL_NO_DISPLAY;
	if (dpy == EGL_NO_DISPLAY || !eglInitialize(dpy, &major, &minor) || !eglBindAPI(EGL_OPENGL_ES_API)) {
		printf("FAIL: EGL on GBM (0x%04x)\n", eglGetError());
		return 1;
	}
	eglChooseConfig(dpy, cfg_attr, &cfg, 1, &n);
	ctx = eglCreateContext(dpy, n ? cfg : EGL_NO_CONFIG_KHR, EGL_NO_CONTEXT, ctx_attr);
	if (ctx == EGL_NO_CONTEXT || !eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) {
		printf("FAIL: an ES 2.0 context without a surface (0x%04x)\n", eglGetError());
		return 1;
	}
	renderer = (const char *)glGetString(GL_RENDERER);
	printf("GL_RENDERER = %s, GL_VERSION = %s\n", renderer, (const char *)glGetString(GL_VERSION));
	if (!renderer || !strstr(renderer, "fpgagpu")) {
		printf("FAIL: the context is not on the fpgagpu driver\n");
		return 1;
	}

	glGenRenderbuffers(1, &rb);
	glBindRenderbuffer(GL_RENDERBUFFER, rb);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8_OES, W, H);
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb);
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		printf("FAIL: framebuffer incomplete\n");
		return 1;
	}
	glViewport(0, 0, W, H);
	pc = program(fs_col);
	pt = program(fs_tex);

	printf("1. a red triangle over the lower left half:\n");
	glUseProgram(pc);
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT);
	draw(GL_TRIANGLES, tri, red3, 3);
	read_back();
	print_map();
	check(&fails, near(at(8, 8), 255, 0, 0, 255, 1), "(8, 8), inside: red", at(8, 8));
	check(&fails, near(at(40, 8), 255, 0, 0, 255, 1), "(40, 8), inside near the bottom edge: red", at(40, 8));
	check(&fails, near(at(56, 56), 0, 0, 0, 255, 1), "(56, 56), outside: the clear's black", at(56, 56));
	check(&fails, near(at(8, 56), 0, 0, 0, 255, 1) || at(8, 56)[0] > 128, "(8, 56), near the top left corner", at(8, 56));

	printf("2. a triangle with red, green and blue corners:\n");
	glClear(GL_COLOR_BUFFER_BIT);
	draw(GL_TRIANGLES, tri, rgb3, 3);
	read_back();
	check(&fails, at(2, 2)[0] > 200 && at(2, 2)[1] < 40 && at(2, 2)[2] < 40, "(2, 2), by the red corner", at(2, 2));
	check(&fails, at(59, 2)[1] > 200 && at(59, 2)[0] < 40, "(59, 2), by the green corner", at(59, 2));
	check(&fails, at(2, 59)[2] > 200 && at(2, 59)[0] < 40, "(2, 59), by the blue corner", at(2, 59));

	printf("3. a 2 x 2 texture over the target:\n");
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, texels);
	glUseProgram(pt);
	glUniform1i(glGetUniformLocation(pt, "t"), 0);
	glClear(GL_COLOR_BUFFER_BIT);
	draw(GL_TRIANGLE_STRIP, quad, uv4, 4);
	read_back();
	check(&fails, near(at(16, 16), 255, 0, 0, 255, 1), "(16, 16): texel (0, 0) red", at(16, 16));
	check(&fails, near(at(48, 16), 0, 255, 0, 255, 1), "(48, 16): texel (1, 0) green", at(48, 16));
	check(&fails, near(at(16, 48), 0, 0, 255, 255, 1), "(16, 48): texel (0, 1) blue", at(16, 48));
	check(&fails, near(at(48, 48), 255, 255, 255, 255, 1), "(48, 48): texel (1, 1) white", at(48, 48));

	printf("4. white at alpha 0.5 blended over black:\n");
	glUseProgram(pc);
	glClear(GL_COLOR_BUFFER_BIT);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	draw(GL_TRIANGLE_STRIP, quad, half4, 4);
	glDisable(GL_BLEND);
	read_back();
	check(&fails, abs(at(32, 32)[0] - 128) <= 2 && abs(at(32, 32)[1] - 128) <= 2, "(32, 32): about 128", at(32, 32));

	printf("5. a 2 x 2 R8 texture with its alpha swizzled from red (X's glamor reads glyphs so):\n");
	{
		static const uint8_t r8[4] = { 64, 128, 192, 255 };
		static const int xy[4][2] = { { 16, 16 }, { 48, 16 }, { 16, 48 }, { 48, 48 } };
		GLuint t8;
		GLenum e;

		glGetError();
		glGenTextures(1, &t8);
		glBindTexture(GL_TEXTURE_2D, t8);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_A, GL_RED_EXT);
		e = glGetError();
		printf("  glTexParameteri(GL_TEXTURE_SWIZZLE_A, GL_RED): GL error 0x%04x %s\n", e, e ? "BAD" : "ok");
		fails += e != 0;
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_R8_EXT, 2, 2, 0, GL_RED_EXT, GL_UNSIGNED_BYTE, r8);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
		glUseProgram(pt);
		glClear(GL_COLOR_BUFFER_BIT);
		draw(GL_TRIANGLE_STRIP, quad, uv4, 4);
		read_back();
		for (int i = 0; i < 4; i++) {
			const uint8_t *p = at(xy[i][0], xy[i][1]);
			char what[64];

			snprintf(what, sizeof(what), "(%d, %d): red %d, green 0, alpha %d", xy[i][0], xy[i][1], r8[i], r8[i]);
			check(&fails, near(p, r8[i], 0, 0, r8[i], 1), what, p);
		}
		glDeleteTextures(1, &t8);
	}

	printf("6. a 16-pixel point coloured by gl_PointCoord in a framebuffer object (ES: s grows right, t down):\n");
	{
		static const char *pvs = "attribute vec4 pos;\nvoid main() { gl_PointSize = 16.0; gl_Position = pos; }\n";
		static const char *pfs = "precision mediump float;\nvoid main() { gl_FragColor = vec4(gl_PointCoord, 0.0, 1.0); }\n";
		static const float centre[2] = { 0, 0 };
		GLuint pp = glCreateProgram(), sh[2];
		const uint8_t *ul, *lr;

		for (int k = 0; k < 2; k++) {
			sh[k] = glCreateShader(k ? GL_FRAGMENT_SHADER : GL_VERTEX_SHADER);
			glShaderSource(sh[k], 1, k ? &pfs : &pvs, NULL);
			glCompileShader(sh[k]);
			glAttachShader(pp, sh[k]);
		}
		glBindAttribLocation(pp, 0, "pos");
		glLinkProgram(pp);
		glUseProgram(pp);
		glClear(GL_COLOR_BUFFER_BIT);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, centre);
		glEnableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glDrawArrays(GL_POINTS, 0, 1);
		read_back();
		ul = at(W / 2 - 7, H / 2 + 6); /* GL's y up: the point's upper left */
		lr = at(W / 2 + 6, H / 2 - 7);
		check(&fails, ul[0] < 64 && ul[1] < 64, "upper left: s and t near 0", ul);
		check(&fails, lr[0] > 191 && lr[1] > 191, "lower right: s and t near 1", lr);
		glDeleteProgram(pp);
		glDeleteShader(sh[0]);
		glDeleteShader(sh[1]);
	}

	printf("7. a local variable with its function's parameter's name (it hides the parameter):\n");
	{
		static const char *fs_scope =
			"precision mediump float;\n"
			"varying vec4 v;\n"
			"float f(float x) { float x = 0.25; return x; }\n"
			"float g(float y) { { float y = y * 0.5; return y; } }\n"
			"void main() { gl_FragColor = vec4(f(1.0), g(1.0), v.x * 0.0, 1.0); }\n";
		GLuint ps = program(fs_scope);
		GLint ok = 0;

		glGetProgramiv(ps, GL_LINK_STATUS, &ok);
		printf("  the program compiles and links: %s\n", ok ? "ok" : "BAD");
		fails += !ok;
		glUseProgram(ps);
		glClear(GL_COLOR_BUFFER_BIT);
		draw(GL_TRIANGLE_STRIP, quad, uv4, 4);
		read_back();
		check(&fails, near(at(32, 32), 64, 128, 0, 255, 2), "(32, 32): the local's 0.25, the inner block's 0.5", at(32, 32));
		glDeleteProgram(ps);
	}

	printf("8. texture2DProj on a mipmapped texture, the level of every pixel:\n");
	{
		static const char *fs_proj =
			"precision mediump float;\n"
			"varying vec4 v;\n"
			"uniform sampler2D t;\n"
			"void main() { gl_FragColor = texture2DProj(t, v.xyw); }\n";
		/* s q, t q and q are affine over the target (X, Y in 0 .. 1): c + cx X + cy Y each. q grows 21 times from
		 * the lower left corner to the upper right: by the first the level changes by a quarter with every pixel */
		static const double sq[3] = { 0.05, 3.0, 0.4 }, tq[3] = { 0.1, 0.5, 3.5 }, qq[3] = { 0.25, 3.0, 2.0 };
		static const double offs[5][2] = { { 0, 0 }, { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } };
		static uint8_t lvl[64 * 64 * 4];
		const double tol = 1.0 / 64, size = 64.0;
		float col[16];
		GLuint pp = program(fs_proj), tm;
		int bad = 0, seen = 0, fx = -1, fy = -1, flev = 0, flo = 0, fhi = 0;

		for (int i = 0; i < 4; i++) {
			double X = i & 1, Y = i >> 1;

			col[4 * i + 0] = (float)(sq[0] + sq[1] * X + sq[2] * Y);
			col[4 * i + 1] = (float)(tq[0] + tq[1] * X + tq[2] * Y);
			col[4 * i + 2] = 0;
			col[4 * i + 3] = (float)(qq[0] + qq[1] * X + qq[2] * Y);
		}
		glGenTextures(1, &tm);
		glBindTexture(GL_TEXTURE_2D, tm);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		for (int l = 0; l < 7; l++) {              /* level l: red 36 l, green 255 */
			for (int k = 0; k < (64 >> l) * (64 >> l); k++) {
				lvl[4 * k] = (uint8_t)(36 * l);
				lvl[4 * k + 1] = 255;
				lvl[4 * k + 2] = 0;
				lvl[4 * k + 3] = 255;
			}
			glTexImage2D(GL_TEXTURE_2D, l, GL_RGBA, 64 >> l, 64 >> l, 0, GL_RGBA, GL_UNSIGNED_BYTE, lvl);
		}
		glUseProgram(pp);
		glUniform1i(glGetUniformLocation(pp, "t"), 0);
		glClear(GL_COLOR_BUFFER_BIT);
		draw(GL_TRIANGLE_STRIP, quad, col, 4);
		read_back();
		for (int y = 0; y < H; y++) {
			for (int x = 0; x < W; x++) {
				const uint8_t *p = at(x, y);
				int lev = (p[0] + 18) / 36, lo, hi;
				double llo = 1e9, lhi = -1e9;

				for (int o = 0; o < 5; o++) {
					double X = (x + 0.5 + offs[o][0]) / W, Y = (y + 0.5 + offs[o][1]) / H;
					double S = sq[0] + sq[1] * X + sq[2] * Y, T = tq[0] + tq[1] * X + tq[2] * Y;
					double Q = qq[0] + qq[1] * X + qq[2] * Y;
					/* the texel coordinates' steps to the next pixel in x and in y */
					double dudx = size / W * (sq[1] * Q - S * qq[1]) / (Q * Q), dudy = size / H * (sq[2] * Q - S * qq[2]) / (Q * Q);
					double dvdx = size / W * (tq[1] * Q - T * qq[1]) / (Q * Q), dvdy = size / H * (tq[2] * Q - T * qq[2]) / (Q * Q);
					double mu = fmax(fabs(dudx), fabs(dudy)), mv = fmax(fabs(dvdx), fabs(dvdy));

					llo = fmin(llo, log2(fmax(mu, mv)));
					lhi = fmax(lhi, log2(mu + mv));
				}
				llo -= tol;
				lhi += tol;
				/* GL_NEAREST_MIPMAP_NEAREST: level 0 up to lambda 1/2, then ceil(lambda + 1/2) - 1 */
				lo = llo <= 0.5 ? 0 : (int)ceil(llo + 0.5) - 1;
				hi = lhi <= 0.5 ? 0 : (int)ceil(lhi + 0.5) - 1;
				lo = lo > 6 ? 6 : lo;
				hi = hi > 6 ? 6 : hi;
				seen |= 1 << lev;
				if (p[1] != 255 || lev < lo || lev > hi) {
					if (!bad) {
						fx = x;
						fy = y;
						flev = lev;
						flo = lo;
						fhi = hi;
					}
					bad++;
				}
			}
		}
		printf("  levels drawn: mask 0x%02x; pixels at a level their derivatives do not allow: %d %s\n", seen, bad,
		       bad || __builtin_popcount(seen) < 3 ? "BAD" : "ok");
		if (bad)
			printf("  the first: (%d, %d) level %d, allowed %d .. %d\n", fx, fy, flev, flo, fhi);
		fails += bad != 0 || __builtin_popcount(seen) < 3;
		glDeleteTextures(1, &tm);
		glDeleteProgram(pp);
	}

	printf("GL error 0x%04x\n", glGetError());
	eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
	eglDestroyContext(dpy, ctx);
	eglTerminate(dpy);
	gbm_device_destroy(gbm);
	close(fd);
	printf("%s\n", fails ? "FAIL" : "PASS");
	return fails != 0;
}
