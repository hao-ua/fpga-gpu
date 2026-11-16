// SPDX-License-Identifier: MIT
/*
 * fg_egl_gl: desktop OpenGL (compatibility profile, fixed function) through the fpgagpu Mesa driver, the way Quake 3
 * engines draw, read back and checked. EGL on GBM on the render node, no surface, a 64 x 64 RGBA8 framebuffer object
 * with a 24-bit depth / 8-bit stencil renderbuffer, an orthographic projection of the target (one unit a pixel):
 *   1. glBegin(GL_QUADS) of a coloured quad (quads go through Gallium's primitive converter);
 *   2. client arrays (glVertexPointer / glColorPointer / glTexCoordPointer) drawn with glDrawElements(GL_TRIANGLES,
 *      GL_UNSIGNED_INT): an RGB texture modulated by a second texture on unit 1 (multitexture), indices far from 0;
 *   3. alpha test: a texture of alpha 0 / 1 halves, GL_GREATER 0.5, over a cleared target;
 *   4. depth test: a near quad drawn first, a far one after it with GL_LEQUAL, then glDepthRange(0, 0.3) for a
 *      quad that must win over the near one;
 *   5. a mipmapped texture (each level its own colour, uploaded level by level as Quake 3 does) drawn small enough
 *      to take level 2 (nearest mip), and magnified (level 0, which survives the chain's reallocation);
 *   6. the state Quake 3 draws its sky with (glDepthRange(1, 1), GL_LEQUAL, no depth writes, back faces culled), and
 *      culling: a clockwise quad with back faces culled and a counter-clockwise one with front faces culled vanish;
 *   7. rendering into a 50 x 30 texture (rows no multiple of 32 bytes: a render shadow), read back and sampled;
 *   8. glBlitFramebuffer from a texture: a copy, a mirror in x, a 2x stretch;
 *   9. a depth texture with ARB_shadow's compare (GL_LEQUAL against r) through fixed function; GL_VERSION printed;
 *  10. a GLSL 1.10 program (OpenGL 2.0 with GLSL 1.10, the hardware's) drawing a quad; #version 120 refused;
 *  11. images with padded rows sampled (the TMU reads tight rows): a 50 x 30 scan-out GBM buffer imported as a dma-buf
 *      EGLImage, rendered red | green and drawn from as a texture (the way X draws a client's window from its DRI3
 *      buffer); a depth texture with a step half way across, compared: 40 wide (rows as rendered) and 36 wide
 *      (rows of 144 bytes, padded to 160);
 *  12. 8400 textured quads with blend constants of their own: the draw ids (8191) run out and start again, the
 *      quad that starts them alone in its pass (its texture must be on the next SUBMIT's list).
 *   cc -O2 -Wall -I<prefix>/include fg_egl_gl.c -o fg_egl_gl -L<libdir> -Wl,-rpath-link,<libdir> \
 *      -lEGL -lGL -lgbm $(pkg-config --cflags --libs libdrm)
 *   fg_egl_gl [render node, default /dev/dri/renderD129]
 */
#define GL_GLEXT_PROTOTYPES 1
#include <execinfo.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <gbm.h>
#include <xf86drm.h>

#define W 64
#define H 64

static uint8_t px[W * H * 4];

static void on_fault(int sig)
{
	void *f[48];
	int n = backtrace(f, 48);

	fprintf(stderr, "signal %d, backtrace:\n", sig);
	backtrace_symbols_fd(f, n, 2);
	_exit(128 + sig);
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

static int near(const uint8_t *p, int r, int g, int b, int tol)
{
	return abs(p[0] - r) <= tol && abs(p[1] - g) <= tol && abs(p[2] - b) <= tol;
}

static void check(int *fails, int ok, const char *what, const uint8_t *p)
{
	printf("  %-52s %s (%u, %u, %u, %u)\n", what, ok ? "ok  " : "BAD ", p[0], p[1], p[2], p[3]);
	*fails += !ok;
}

static GLuint texture(int w, int h, GLenum fmt, const uint8_t *data, GLenum filter)
{
	GLuint t;

	glGenTextures(1, &t);
	glBindTexture(GL_TEXTURE_2D, t);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexImage2D(GL_TEXTURE_2D, 0, fmt, w, h, 0, fmt, GL_UNSIGNED_BYTE, data);
	return t;
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

int main(int argc, char **argv)
{
	const char *node = argc > 1 ? argv[1] : "/dev/dri/renderD129";
	static const EGLint cfg_attr[] = { EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE };
	struct gbm_device *gbm;
	drmVersionPtr v;
	EGLDisplay dpy;
	EGLContext ctx;
	EGLConfig cfg = NULL;
	EGLint major, minor, n = 0;
	GLuint rb[2], fbo, t_rgb, t_mod, t_alpha, t_mip;
	const char *renderer, *ext;
	int fd, fails = 0;

	signal(SIGSEGV, on_fault);
	signal(SIGABRT, on_fault);
	fd = open(node, O_RDWR | O_CLOEXEC);
	v = fd >= 0 ? drmGetVersion(fd) : NULL;
	if (!v || !v->name || strcmp(v->name, "fpgagpu")) {
		printf("FAIL: %s is not the fpgagpu device\n", node);
		return 1;
	}
	drmFreeVersion(v);
	gbm = gbm_create_device(fd);
	dpy = gbm ? eglGetPlatformDisplay(EGL_PLATFORM_GBM_KHR, gbm, NULL) : EGL_NO_DISPLAY;
	if (dpy == EGL_NO_DISPLAY || !eglInitialize(dpy, &major, &minor) || !eglBindAPI(EGL_OPENGL_API)) {
		printf("FAIL: EGL with desktop OpenGL on GBM (0x%04x)\n", eglGetError());
		return 1;
	}
	eglChooseConfig(dpy, cfg_attr, &cfg, 1, &n);
	ctx = eglCreateContext(dpy, n ? cfg : EGL_NO_CONFIG_KHR, EGL_NO_CONTEXT, NULL);
	if (ctx == EGL_NO_CONTEXT || !eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) {
		printf("FAIL: a desktop OpenGL context without a surface (0x%04x)\n", eglGetError());
		return 1;
	}
	renderer = (const char *)glGetString(GL_RENDERER);
	ext = (const char *)glGetString(GL_EXTENSIONS);
	printf("GL_RENDERER = %s, GL_VERSION = %s\n", renderer, (const char *)glGetString(GL_VERSION));
	printf("  multitexture %s, framebuffer objects %s, texture_env_combine %s, compiled vertex arrays %s\n",
	       strstr(ext, "GL_ARB_multitexture") ? "yes" : "no", strstr(ext, "GL_EXT_framebuffer_object") ? "yes" : "no",
	       strstr(ext, "GL_ARB_texture_env_combine") ? "yes" : "no", strstr(ext, "GL_EXT_compiled_vertex_array") ? "yes" : "no");
	if (!renderer || !strstr(renderer, "fpgagpu")) {
		printf("FAIL: the context is not on the fpgagpu driver\n");
		return 1;
	}

	glGenRenderbuffersEXT(2, rb);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, rb[0]);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGBA8, W, H);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, rb[1]);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH24_STENCIL8_EXT, W, H);
	glGenFramebuffersEXT(1, &fbo);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, rb[0]);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, rb[1]);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, rb[1]);
	if (glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT) != GL_FRAMEBUFFER_COMPLETE_EXT) {
		printf("FAIL: framebuffer incomplete (0x%04x)\n", glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT));
		return 1;
	}
	glViewport(0, 0, W, H);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, W, 0, H, -1, 1);   /* z: -1 near (depth 0) .. 1 far: glVertex z = -z_eye */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glClearColor(0, 0, 0, 1);

	printf("1. glBegin(GL_QUADS):\n");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor3f(0, 1, 0);
	quad(16, 16, 48, 48, 0);
	read_back();
	check(&fails, near(at(32, 32), 0, 255, 0, 1), "(32, 32) inside: green", at(32, 32));
	check(&fails, near(at(8, 8), 0, 0, 0, 1), "(8, 8) outside: black", at(8, 8));

	printf("2. client arrays, glDrawElements(GL_UNSIGNED_INT), multitexture:\n");
	{
		static uint8_t rgb[4 * 4 * 3], mod[4];
		float vx[2000 * 3], tc[2000 * 2], tc1[2000 * 2];
		uint8_t col[2000 * 4];
		GLuint idx[6];
		const int b = 1500;         /* the vertices sit far into the arrays, as a batch's do */

		for (int i = 0; i < 16; i++) {
			rgb[3 * i] = 200;
			rgb[3 * i + 1] = 100;
			rgb[3 * i + 2] = 50;
		}
		mod[0] = mod[1] = mod[2] = mod[3] = 128;
		t_rgb = texture(4, 4, GL_RGB, rgb, GL_NEAREST);
		t_mod = texture(1, 1, GL_RGBA, mod, GL_NEAREST);
		memset(vx, 0, sizeof(vx));
		memset(tc, 0, sizeof(tc));
		memset(tc1, 0, sizeof(tc1));
		memset(col, 255, sizeof(col));
		vx[3 * b + 0] = 8;  vx[3 * b + 1] = 8;
		vx[3 * (b + 1) + 0] = 56; vx[3 * (b + 1) + 1] = 8;
		vx[3 * (b + 2) + 0] = 56; vx[3 * (b + 2) + 1] = 56;
		vx[3 * (b + 3) + 0] = 8;  vx[3 * (b + 3) + 1] = 56;
		tc[2 * (b + 1)] = 1; tc[2 * (b + 2)] = 1; tc[2 * (b + 2) + 1] = 1; tc[2 * (b + 3) + 1] = 1;
		idx[0] = b; idx[1] = b + 1; idx[2] = b + 2; idx[3] = b; idx[4] = b + 2; idx[5] = b + 3;
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glColor3f(1, 1, 1);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, t_rgb);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, t_mod);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, vx);
		glColorPointer(4, GL_UNSIGNED_BYTE, 0, col);
		glClientActiveTextureARB(GL_TEXTURE0_ARB);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(2, GL_FLOAT, 0, tc);
		glClientActiveTextureARB(GL_TEXTURE1_ARB);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(2, GL_FLOAT, 0, tc1);
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, idx);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glDisable(GL_TEXTURE_2D);
		glClientActiveTextureARB(GL_TEXTURE0_ARB);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);
		read_back();
		check(&fails, near(at(32, 32), 100, 50, 25, 3), "(32, 32): (200, 100, 50) x 0.5 = (100, 50, 25)", at(32, 32));
		check(&fails, near(at(4, 4), 0, 0, 0, 1), "(4, 4) outside: black", at(4, 4));
	}

	printf("3. alpha test GL_GREATER 0.5:\n");
	{
		static uint8_t a[2 * 4];
		a[0] = a[1] = a[2] = 255; a[3] = 0;      /* texel 0: transparent */
		a[4] = a[5] = a[6] = 255; a[7] = 255;    /* texel 1: opaque white */
		t_alpha = texture(2, 1, GL_RGBA, a, GL_NEAREST);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_TEXTURE_2D);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glEnable(GL_ALPHA_TEST);
		glAlphaFunc(GL_GREATER, 0.5f);
		quad(0, 0, 64, 64, 0);
		glDisable(GL_ALPHA_TEST);
		glDisable(GL_TEXTURE_2D);
		read_back();
		check(&fails, near(at(16, 32), 0, 0, 0, 1), "(16, 32) transparent half: discarded, black", at(16, 32));
		check(&fails, near(at(48, 32), 255, 255, 255, 1), "(48, 32) opaque half: white", at(48, 32));
	}

	printf("4. depth test:\n");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glColor3f(1, 0, 0);
	quad(8, 8, 40, 40, 0.5f);        /* near: depth 0.25 */
	glColor3f(0, 0, 1);
	quad(24, 24, 56, 56, -0.5f);     /* far: depth 0.75, behind the red where they overlap */
	glDepthRange(0, 0.3);
	glColor3f(1, 1, 0);
	quad(4, 30, 20, 36, -0.5f);      /* depth 0.225 with the range: in front of the red */
	glDepthRange(0, 1);
	glDisable(GL_DEPTH_TEST);
	read_back();
	check(&fails, near(at(30, 30), 255, 0, 0, 1), "(30, 30) overlap: the near red", at(30, 30));
	check(&fails, near(at(50, 50), 0, 0, 255, 1), "(50, 50) far only: blue", at(50, 50));
	check(&fails, near(at(12, 33), 255, 255, 0, 1), "(12, 33) glDepthRange(0, 0.3) quad over red: yellow", at(12, 33));

	printf("5. a mipmapped texture, level 2 taken:\n");
	{
		static uint8_t l0[16 * 16 * 4], l1[8 * 8 * 4], l2[4 * 4 * 4], l3[2 * 2 * 4], l4[4];
		uint8_t *lv[5] = { l0, l1, l2, l3, l4 };
		static const uint8_t c[5][3] = { { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 }, { 255, 255, 0 }, { 255, 0, 255 } };

		for (int l = 0; l < 5; l++)
			for (int i = 0; i < (16 >> l) * (16 >> l); i++) {
				memcpy(lv[l] + 4 * i, c[l], 3);
				lv[l][4 * i + 3] = 255;
			}
		glGenTextures(1, &t_mip);
		glBindTexture(GL_TEXTURE_2D, t_mip);
		for (int l = 0; l < 5; l++)
			glTexImage2D(GL_TEXTURE_2D, l, GL_RGBA, 16 >> l, 16 >> l, 0, GL_RGBA, GL_UNSIGNED_BYTE, lv[l]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_TEXTURE_2D);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		quad(28, 28, 32, 32, 0);         /* 16 texels over 4 pixels: level 2 */
		quad(40, 40, 64, 64, 0);         /* over 24 pixels: magnified, level 0 (kept when the chain grew) */
		glDisable(GL_TEXTURE_2D);
		read_back();
		check(&fails, near(at(30, 30), 0, 0, 255, 1), "(30, 30): level 2's blue", at(30, 30));
		check(&fails, near(at(52, 52), 255, 0, 0, 1), "(52, 52): level 0's red", at(52, 52));
	}

	printf("6. the sky's state: glDepthRange(1, 1), GL_LEQUAL, no depth writes, back faces culled:\n");
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDepthMask(GL_FALSE);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glDepthRange(1, 1);
	glColor3f(0, 1, 1);
	quad(8, 8, 56, 56, 0.9f);        /* counter-clockwise: front facing, drawn at depth 1 (= the clear) */
	glDepthRange(0, 1);
	glDepthMask(GL_TRUE);
	read_back();
	check(&fails, near(at(32, 32), 0, 255, 255, 1), "(32, 32) depth range (1, 1) over a cleared depth: cyan", at(32, 32));
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor3f(1, 1, 0);
	glBegin(GL_QUADS);               /* clockwise: a back face, culled */
	glVertex3f(8, 8, 0); glVertex3f(8, 56, 0); glVertex3f(56, 56, 0); glVertex3f(56, 8, 0);
	glEnd();
	glCullFace(GL_FRONT);
	glColor3f(1, 0, 1);
	quad(20, 20, 44, 44, 0);         /* counter-clockwise with front faces culled: culled */
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	read_back();
	check(&fails, near(at(12, 12), 0, 0, 0, 1), "(12, 12) a clockwise quad, back faces culled: black", at(12, 12));
	check(&fails, near(at(32, 32), 0, 0, 0, 1), "(32, 32) a counter-clockwise quad, front culled: black", at(32, 32));

	printf("7. rendering into a 50 x 30 texture (rows of 200 bytes, no multiple of 32), then sampling it:\n");
	{
		static uint8_t small[50 * 30 * 4];
		GLuint t_rt, fbo2;

		glGenTextures(1, &t_rt);
		glBindTexture(GL_TEXTURE_2D, t_rt);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 50, 30, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
		glGenFramebuffersEXT(1, &fbo2);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo2);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t_rt, 0);
		glViewport(0, 0, 50, 30);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0, 50, 0, 30, -1, 1);
		glClearColor(1, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		glColor3f(0, 1, 0);
		quad(0, 0, 25, 30, 0);           /* left half green, right half the clear's red */
		memset(small, 0xA5, sizeof(small));
		glReadPixels(0, 0, 50, 30, GL_RGBA, GL_UNSIGNED_BYTE, small);
		check(&fails, small[4 * (15 * 50 + 10) + 1] == 255 && small[4 * (15 * 50 + 10)] == 0,
		      "texture (10, 15) read back: green", small + 4 * (15 * 50 + 10));
		check(&fails, small[4 * (15 * 50 + 40)] == 255 && small[4 * (15 * 50 + 40) + 1] == 0,
		      "texture (40, 15) read back: red", small + 4 * (15 * 50 + 40));
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
		glViewport(0, 0, W, H);
		glLoadIdentity();
		glOrtho(0, W, 0, H, -1, 1);
		glClearColor(0, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		glEnable(GL_TEXTURE_2D);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		quad(0, 0, 64, 64, 0);
		glDisable(GL_TEXTURE_2D);
		read_back();
		check(&fails, near(at(16, 32), 0, 255, 0, 1), "(16, 32) the texture sampled: its green half", at(16, 32));
		check(&fails, near(at(48, 32), 255, 0, 0, 1), "(48, 32) the texture sampled: its red half", at(48, 32));
		glDeleteFramebuffersEXT(1, &fbo2);
		glDeleteTextures(1, &t_rt);
	}

	printf("8. glBlitFramebuffer from a 32 x 32 texture (left half green, right half red): a copy, a mirror, a stretch:\n");
	{
		PFNGLBLITFRAMEBUFFEREXTPROC glBlitFramebufferEXT =
			(PFNGLBLITFRAMEBUFFEREXTPROC)eglGetProcAddress("glBlitFramebufferEXT");
		GLuint t_src, fbo3;

		if (!glBlitFramebufferEXT) {
			printf("FAIL: no glBlitFramebufferEXT\n");
			return 1;
		}
		glGenTextures(1, &t_src);
		glBindTexture(GL_TEXTURE_2D, t_src);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 32, 32, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
		glGenFramebuffersEXT(1, &fbo3);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo3);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t_src, 0);
		glViewport(0, 0, 32, 32);
		glLoadIdentity();
		glOrtho(0, 32, 0, 32, -1, 1);
		glClearColor(1, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		glColor3f(0, 1, 0);
		quad(0, 0, 16, 32, 0);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
		glViewport(0, 0, W, H);
		glLoadIdentity();
		glOrtho(0, W, 0, H, -1, 1);
		glClearColor(0, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, fbo3);
		glBindFramebufferEXT(GL_DRAW_FRAMEBUFFER_EXT, fbo);
		glBlitFramebufferEXT(0, 0, 32, 32, 0, 0, 32, 32, GL_COLOR_BUFFER_BIT, GL_NEAREST);    /* a copy */
		glBlitFramebufferEXT(0, 0, 32, 32, 64, 32, 32, 64, GL_COLOR_BUFFER_BIT, GL_NEAREST);  /* mirrored in x */
		glBlitFramebufferEXT(0, 0, 16, 16, 32, 0, 64, 32, GL_COLOR_BUFFER_BIT, GL_NEAREST);   /* green, stretched 2x */
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
		read_back();
		check(&fails, near(at(8, 16), 0, 255, 0, 1), "(8, 16) the copy's left: green", at(8, 16));
		check(&fails, near(at(24, 16), 255, 0, 0, 1), "(24, 16) the copy's right: red", at(24, 16));
		check(&fails, near(at(40, 48), 255, 0, 0, 1), "(40, 48) mirrored: red on the left", at(40, 48));
		check(&fails, near(at(56, 48), 0, 255, 0, 1), "(56, 48) mirrored: green on the right", at(56, 48));
		check(&fails, near(at(48, 16), 0, 255, 0, 1) && near(at(62, 30), 0, 255, 0, 1),
		      "(48, 16), (62, 30) the stretch: green", at(62, 30));
		glDeleteFramebuffersEXT(1, &fbo3);
		glDeleteTextures(1, &t_src);
	}

	printf("9. a depth texture compared (ARB_shadow, GL_LEQUAL) at depth 0.5: r 0.25 on the left, 0.75 on the right:\n");
	{
		GLuint t_z, rb4, fbo4;

		printf("  GL_VERSION %s, GLSL %s\n", (const char *)glGetString(GL_VERSION),
		       (const char *)glGetString(GL_SHADING_LANGUAGE_VERSION));
		glGenTextures(1, &t_z);
		glBindTexture(GL_TEXTURE_2D, t_z);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, 32, 32, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, NULL);
		glGenRenderbuffersEXT(1, &rb4);
		glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, rb4);
		glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGBA8, 32, 32);
		glGenFramebuffersEXT(1, &fbo4);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo4);
		glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, rb4);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, t_z, 0);
		if (glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT) != GL_FRAMEBUFFER_COMPLETE_EXT)
			printf("  the depth framebuffer is incomplete (0x%04x)\n", glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT));
		glViewport(0, 0, 32, 32);
		glClearDepth(0.5);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearDepth(1.0);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
		glViewport(0, 0, W, H);
		glClearColor(0.5f, 0, 0.5f, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		glBindTexture(GL_TEXTURE_2D, t_z);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE_ARB, GL_COMPARE_R_TO_TEXTURE_ARB);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC_ARB, GL_LEQUAL);
		glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE_ARB, GL_LUMINANCE);
		glEnable(GL_TEXTURE_2D);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glDisable(GL_DEPTH_TEST);
		glBegin(GL_QUADS);
		glTexCoord3f(0, 0, 0.25f); glVertex2f(0, 0);
		glTexCoord3f(0.5f, 0, 0.25f); glVertex2f(32, 0);
		glTexCoord3f(0.5f, 1, 0.25f); glVertex2f(32, 64);
		glTexCoord3f(0, 1, 0.25f); glVertex2f(0, 64);
		glTexCoord3f(0.5f, 0, 0.75f); glVertex2f(32, 0);
		glTexCoord3f(1, 0, 0.75f); glVertex2f(64, 0);
		glTexCoord3f(1, 1, 0.75f); glVertex2f(64, 64);
		glTexCoord3f(0.5f, 1, 0.75f); glVertex2f(32, 64);
		glEnd();
		glDisable(GL_TEXTURE_2D);
		read_back();
		check(&fails, near(at(16, 32), 255, 255, 255, 1), "(16, 32) 0.25 <= 0.5: white", at(16, 32));
		check(&fails, near(at(48, 32), 0, 0, 0, 1), "(48, 32) 0.75 > 0.5: black", at(48, 32));
		glClearColor(0, 0, 0, 1);
		glDeleteFramebuffersEXT(1, &fbo4);
		glDeleteRenderbuffersEXT(1, &rb4);
		glDeleteTextures(1, &t_z);
	}

	printf("10. a GLSL 1.10 program (built-in matrix, varying colour, a sampler); #version 120 refused:\n");
	{
		static const char *vs110 =
			"#version 110\n"
			"varying vec4 c;\n"
			"void main() { gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; c = gl_Color;\n"
			"  gl_TexCoord[0] = gl_MultiTexCoord0; }\n";
		static const char *fs110 =
			"#version 110\n"
			"uniform sampler2D t;\n"
			"varying vec4 c;\n"
			"void main() { gl_FragColor = c * texture2D(t, gl_TexCoord[0].st); }\n";
		static const char *fs120 = "#version 120\nvoid main() { gl_FragColor = vec4(1.0); }\n";
		static const uint8_t white[4] = { 255, 255, 255, 255 };
		GLuint vs = glCreateShader(GL_VERTEX_SHADER), fs = glCreateShader(GL_FRAGMENT_SHADER);
		GLuint f2 = glCreateShader(GL_FRAGMENT_SHADER), prog = glCreateProgram(), tw;
		GLint ok = 0, ok120 = 1;

		glShaderSource(vs, 1, &vs110, NULL);
		glCompileShader(vs);
		glShaderSource(fs, 1, &fs110, NULL);
		glCompileShader(fs);
		glAttachShader(prog, vs);
		glAttachShader(prog, fs);
		glLinkProgram(prog);
		glGetProgramiv(prog, GL_LINK_STATUS, &ok);
		check(&fails, ok, "the 1.10 program links", (const uint8_t *)"\0\0\0\0");
		glShaderSource(f2, 1, &fs120, NULL);
		glCompileShader(f2);
		glGetShaderiv(f2, GL_COMPILE_STATUS, &ok120);
		check(&fails, !ok120, "#version 120 is refused", (const uint8_t *)"\0\0\0\0");
		glGenTextures(1, &tw);
		glBindTexture(GL_TEXTURE_2D, tw);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, white);
		glClear(GL_COLOR_BUFFER_BIT);
		glUseProgram(prog);
		glUniform1i(glGetUniformLocation(prog, "t"), 0);
		glColor3f(0, 0, 1);
		quad(16, 16, 48, 48, 0);
		glUseProgram(0);
		read_back();
		check(&fails, near(at(32, 32), 0, 0, 255, 1), "(32, 32) the quad: blue", at(32, 32));
		check(&fails, near(at(4, 4), 0, 0, 0, 1), "(4, 4) outside: black", at(4, 4));
		glDeleteTextures(1, &tw);
		glDeleteProgram(prog);
	}

	printf("11. padded rows sampled: a 50 x 30 scan-out buffer rendered red | green and drawn from; depth textures\n"
	       "    40 and 36 wide with a step half way across, compared:\n");
	{
		typedef void (*image_target_fn)(GLenum target, void *image);
		PFNEGLCREATEIMAGEKHRPROC create_image = (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
		PFNEGLDESTROYIMAGEKHRPROC destroy_image = (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
		image_target_fn tex_image = (image_target_fn)eglGetProcAddress("glEGLImageTargetTexture2DOES");
		image_target_fn rb_image = (image_target_fn)eglGetProcAddress("glEGLImageTargetRenderbufferStorageOES");
		struct gbm_bo *bo = gbm_bo_create(gbm, 50, 30, GBM_FORMAT_ARGB8888, GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING);
		int bfd = bo ? gbm_bo_get_fd(bo) : -1;
		EGLImageKHR img = EGL_NO_IMAGE_KHR;
		GLuint rb5, fbo5, t_img, t_z, rb6, fbo6;

		if (bfd >= 0 && create_image) {
			EGLint attr[] = {
				EGL_WIDTH, 50, EGL_HEIGHT, 30, EGL_LINUX_DRM_FOURCC_EXT, GBM_FORMAT_ARGB8888,
				EGL_DMA_BUF_PLANE0_FD_EXT, bfd, EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
				EGL_DMA_BUF_PLANE0_PITCH_EXT, (EGLint)gbm_bo_get_stride(bo), EGL_NONE
			};

			img = create_image(dpy, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, NULL, attr);
		}
		printf("  buffer pitch %u bytes (its tight rows: 200)\n", bo ? gbm_bo_get_stride(bo) : 0);
		check(&fails, img != EGL_NO_IMAGE_KHR && tex_image && rb_image, "the buffer imported as an EGLImage",
		      (const uint8_t *)"\0\0\0\0");
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_CULL_FACE);
		glDisable(GL_ALPHA_TEST);
		if (img != EGL_NO_IMAGE_KHR && tex_image && rb_image) {
			glGenRenderbuffersEXT(1, &rb5);
			glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, rb5);
			rb_image(GL_RENDERBUFFER_EXT, img);
			glGenFramebuffersEXT(1, &fbo5);
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo5);
			glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, rb5);
			if (glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT) != GL_FRAMEBUFFER_COMPLETE_EXT)
				printf("  the buffer's framebuffer is incomplete (0x%04x)\n", glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT));
			glViewport(0, 0, 50, 30);
			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			glOrtho(0, 50, 0, 30, -1, 1);
			glMatrixMode(GL_MODELVIEW);
			glClearColor(1, 0, 0, 1);
			glClear(GL_COLOR_BUFFER_BIT);
			glColor3f(0, 1, 0);
			quad(25, 0, 50, 30, 0);
			glColor3f(1, 1, 1);
			glMatrixMode(GL_PROJECTION);
			glPopMatrix();
			glMatrixMode(GL_MODELVIEW);
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
			glViewport(0, 0, W, H);
			glClearColor(0, 0, 0, 1);
			glClear(GL_COLOR_BUFFER_BIT);
			glGenTextures(1, &t_img);
			glBindTexture(GL_TEXTURE_2D, t_img);
			tex_image(GL_TEXTURE_2D, img);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glEnable(GL_TEXTURE_2D);
			glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
			quad(0, 0, 50, 30, 0);
			glDisable(GL_TEXTURE_2D);
			read_back();
			check(&fails, near(at(20, 2), 255, 0, 0, 1), "(20, 2) the buffer's left: red", at(20, 2));
			check(&fails, near(at(30, 2), 0, 255, 0, 1), "(30, 2) the buffer's right: green", at(30, 2));
			check(&fails, near(at(20, 27), 255, 0, 0, 1), "(20, 27) the buffer's left: red", at(20, 27));
			check(&fails, near(at(30, 27), 0, 255, 0, 1), "(30, 27) the buffer's right: green", at(30, 27));
			glDeleteTextures(1, &t_img);
			glDeleteFramebuffersEXT(1, &fbo5);
			glDeleteRenderbuffersEXT(1, &rb5);
		}
		if (img != EGL_NO_IMAGE_KHR && destroy_image)
			destroy_image(dpy, img);
		if (bfd >= 0)
			close(bfd);
		if (bo)
			gbm_bo_destroy(bo);

		/* 40 wide: rows of 160 bytes, as rendered; 36 wide: rows of 144 bytes, padded to 160 */
		for (int zw = 40; zw >= 36; zw -= 4) {
			char what[64];

			glGenTextures(1, &t_z);
			glBindTexture(GL_TEXTURE_2D, t_z);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, zw, zw, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, NULL);
			glGenRenderbuffersEXT(1, &rb6);
			glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, rb6);
			glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGBA8, zw, zw);
			glGenFramebuffersEXT(1, &fbo6);
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo6);
			glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, rb6);
			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, t_z, 0);
			if (glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT) != GL_FRAMEBUFFER_COMPLETE_EXT)
				printf("  the depth framebuffer is incomplete (0x%04x)\n", glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT));
			glViewport(0, 0, zw, zw);
			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			glOrtho(0, zw, 0, zw, -1, 1);
			glMatrixMode(GL_MODELVIEW);
			glDepthRange(0, 1);
			glDepthMask(GL_TRUE);
			glClearDepth(1.0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glEnable(GL_DEPTH_TEST);
			glDepthFunc(GL_ALWAYS);
			quad(0, 0, zw / 2, zw, 0.5f);     /* z_ndc -0.5, depth 0.25 on the left; 1.0 (the clear) on the right */
			glDepthFunc(GL_LESS);
			glDisable(GL_DEPTH_TEST);
			glMatrixMode(GL_PROJECTION);
			glPopMatrix();
			glMatrixMode(GL_MODELVIEW);
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
			glViewport(0, 0, W, H);
			glClearColor(0.5f, 0, 0.5f, 1);
			glClear(GL_COLOR_BUFFER_BIT);
			glBindTexture(GL_TEXTURE_2D, t_z);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE_ARB, GL_COMPARE_R_TO_TEXTURE_ARB);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC_ARB, GL_LEQUAL);
			glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE_ARB, GL_LUMINANCE);
			glEnable(GL_TEXTURE_2D);
			glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
			glBegin(GL_QUADS);
			glTexCoord3f(0, 0, 0.5f); glVertex2f(0, 0);
			glTexCoord3f(1, 0, 0.5f); glVertex2f(zw, 0);
			glTexCoord3f(1, 1, 0.5f); glVertex2f(zw, zw);
			glTexCoord3f(0, 1, 0.5f); glVertex2f(0, zw);
			glEnd();
			glDisable(GL_TEXTURE_2D);
			read_back();
			snprintf(what, sizeof(what), "%d wide: (8, 2) 0.5 > 0.25: black", zw);
			check(&fails, near(at(8, 2), 0, 0, 0, 1), what, at(8, 2));
			snprintf(what, sizeof(what), "%d wide: (%d, 2) 0.5 <= 1.0: white", zw, zw - 8);
			check(&fails, near(at(zw - 8, 2), 255, 255, 255, 1), what, at(zw - 8, 2));
			snprintf(what, sizeof(what), "%d wide: (8, %d) 0.5 > 0.25: black", zw, zw - 3);
			check(&fails, near(at(8, zw - 3), 0, 0, 0, 1), what, at(8, zw - 3));
			snprintf(what, sizeof(what), "%d wide: (%d, %d) 0.5 <= 1.0: white", zw, zw - 8, zw - 3);
			check(&fails, near(at(zw - 8, zw - 3), 255, 255, 255, 1), what, at(zw - 8, zw - 3));
			glClearColor(0, 0, 0, 1);
			glDeleteFramebuffersEXT(1, &fbo6);
			glDeleteRenderbuffersEXT(1, &rb6);
			glDeleteTextures(1, &t_z);
		}
	}

	printf("12. 8400 one-pixel textured quads, each with its own blend constant (a new draw state each: the draw ids\n"
	       "    run out and start again); from the 7600th on each quad is flushed on its own:\n");
	{
		static const uint8_t white[4] = { 255, 255, 255, 255 };
		GLuint tw = texture(1, 1, GL_RGBA, white, GL_NEAREST);
		const int nq = 8400;
		int bad = 0, first_bad = 0;

		glClear(GL_COLOR_BUFFER_BIT);
		glEnable(GL_TEXTURE_2D);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glEnable(GL_BLEND);
		glBlendFunc(GL_CONSTANT_COLOR, GL_ZERO);
		for (int i = 1; i <= nq; i++) {
			int p = (i - 1) % (W * H);

			glBlendColor((i & 255) / 255.0f, (i >> 8) / 255.0f, 0.5f, 1);
			quad(p % W, p / W, p % W + 1, p / W + 1, 0);
			if (i >= 7600)
				glFlush();
		}
		glDisable(GL_BLEND);
		glDisable(GL_TEXTURE_2D);
		read_back();
		/* each pixel holds the last quad drawn on it: those of quads nq - W * H + 1 .. nq */
		for (int i = nq - W * H + 1; i <= nq; i++) {
			int p = (i - 1) % (W * H);

			if (!near(at(p % W, p / W), i & 255, i >> 8, 128, 1) && !bad++)
				first_bad = i;
		}
		if (bad)
			printf("  %d quads lost, the first quad %d\n", bad, first_bad);
		check(&fails, !bad, "the last 4096 quads, each on its pixel", at((nq - 1) % W, (nq - 1) % (W * H) / W));
		glDeleteTextures(1, &tw);
	}

	printf("GL error 0x%04x\n", glGetError());
	{
		GLuint t[4] = { t_rgb, t_mod, t_alpha, t_mip };

		glDeleteTextures(4, t);
	}
	eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
	eglDestroyContext(dpy, ctx);
	eglTerminate(dpy);
	gbm_device_destroy(gbm);
	close(fd);
	printf("%s\n", fails ? "FAIL" : "PASS");
	return fails != 0;
}
