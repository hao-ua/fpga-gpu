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
 *      quad that starts them alone in its pass (its texture must be on the next SUBMIT's list);
 *  13. region passes (a pass renders only the rectangle its draws and clears reach): quads that grow a region and one
 *      above-left of it (a new pass), a quad in the corner (the rectangle ends at the image's end), scissored clears
 *      alone and inside a region pass, a textured quad, a 250-wide texture (a shadow); then 200 one-quad passes on an
 *      800 x 600 target timed (the whole target would cost about 16 ms a pass);
 *  14. draws that copy (the X server's way: a program that samples one image at a varying running one to one with
 *      the window position, quads of 16-bit positions), done as COPY_IMAGE: a rectangle, two in one draw, one cut by
 *      the scissor, a repeating tile (fract) that wraps on both axes, an image with alpha into an RGB one and back
 *      (alpha 1.0 survives), an 8 x 8 tile over 200 x 150, a 2-row run wrapping in x, 20 rectangles in one draw
 *      (tiles and a list where board B has the copy extensions), the same program magnified (drawn, not copied);
 *      the 64 x 64 tile over 800 x 600 timed and checked; with GL_NV_texture_barrier, two
 *      rectangles moved over themselves inside one image (copies in the move's order) and a draw magnifying rows
 *      of its own target into rows it reads (drawn from a copy taken before its pass); a copy into a whole-target
 *      pass with a draw (drawn in it); 40 moves of a 374 x 298 window (into a temporary image and back) timed;
 *  15. 32 render targets 21 x 29 (rendered in shadows of 96-byte rows; board B stores whole bands of 8 rows), each
 *      followed by an image with a pattern: the targets rendered again, every image must be unchanged;
 *  16. an 800 x 600 target with depth (no region passes) filled with rows of their own colour, then 100 passes of an
 *      8 x 8 quad: every row around the quads kept, the passes timed (with band skipping a pass costs its bands).
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
#include <time.h>
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

	printf("13. region passes (a pass renders only the rectangle its draws and clears reach), 256 x 256 targets:\n");
	{
		static const struct { int x, y, r, g, b; const char *what; } pt[] = {
			{ 210, 160, 255, 0, 0, "(210, 160) the first quad: red" },
			{ 235, 205, 255, 0, 0, "(235, 205) the second, below-right: red" },
			{ 15, 15, 255, 0, 0, "(15, 15) the third, above-left (a new pass): red" },
			{ 199, 160, 0, 0, 255, "(199, 160) left of the first: blue" },
			{ 225, 165, 0, 0, 255, "(225, 165) between them: blue" },
			{ 100, 100, 0, 0, 255, "(100, 100) untouched: blue" },
			{ 250, 250, 0, 255, 0, "(250, 250) the corner quad: green" },
			{ 239, 245, 0, 0, 255, "(239, 245) left of the corner quad: blue" },
			{ 110, 70, 255, 255, 0, "(110, 70) a scissored clear alone: yellow" },
			{ 120, 70, 0, 0, 255, "(120, 70) right of it: blue" },
			{ 205, 185, 255, 0, 255, "(205, 185) a clear inside a region pass: magenta" },
			{ 64, 186, 255, 255, 255, "(64, 186) a textured quad: white" },
			{ 72, 186, 0, 0, 255, "(72, 186) right of it: blue" },
		};
		static const uint8_t white[4] = { 255, 255, 255, 255 };
		uint8_t *p = malloc(256 * 256 * 4);
		GLuint rb7, fbo7, t_sh, fbo8, tw = texture(1, 1, GL_RGBA, white, GL_NEAREST);
		double t0, t1;
		struct timespec ts;

		glGenRenderbuffersEXT(1, &rb7);
		glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, rb7);
		glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGBA8, 256, 256);
		glGenFramebuffersEXT(1, &fbo7);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo7);
		glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, rb7);
		glViewport(0, 0, 256, 256);
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0, 256, 0, 256, -1, 1);
		glMatrixMode(GL_MODELVIEW);
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_BLEND);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_SCISSOR_TEST);
		glClearColor(0, 0, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		glFlush();
		glColor3f(1, 0, 0);
		quad(200, 150, 220, 170, 0);
		quad(230, 200, 240, 210, 0);
		quad(10, 10, 20, 20, 0);
		glFlush();
		glColor3f(0, 1, 0);
		quad(240, 240, 256, 256, 0);
		glFlush();
		glEnable(GL_SCISSOR_TEST);
		glScissor(100, 60, 20, 20);
		glClearColor(1, 1, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		glDisable(GL_SCISSOR_TEST);
		glFlush();
		glColor3f(0, 0, 0);
		quad(196, 176, 200, 180, 0);       /* a region pass, then a clear inside it (a quad) */
		glEnable(GL_SCISSOR_TEST);
		glScissor(200, 180, 10, 10);
		glClearColor(1, 0, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		glDisable(GL_SCISSOR_TEST);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, tw);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		quad(60, 180, 68, 192, 0);
		glDisable(GL_TEXTURE_2D);
		glClearColor(0, 0, 0, 1);
		memset(p, 0xA5, 256 * 256 * 4);
		glReadPixels(0, 0, 256, 256, GL_RGBA, GL_UNSIGNED_BYTE, p);
		for (unsigned i = 0; i < sizeof(pt) / sizeof(pt[0]); i++) {
			const uint8_t *q = p + 4 * (pt[i].y * 256 + pt[i].x);

			check(&fails, near(q, pt[i].r, pt[i].g, pt[i].b, 1), pt[i].what, q);
		}

		/* a 250-wide texture: rows of 1000 bytes, rendered in an aligned shadow and copied back */
		glGenTextures(1, &t_sh);
		glBindTexture(GL_TEXTURE_2D, t_sh);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 250, 250, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
		glGenFramebuffersEXT(1, &fbo8);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo8);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t_sh, 0);
		glViewport(0, 0, 250, 250);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0, 250, 0, 250, -1, 1);
		glMatrixMode(GL_MODELVIEW);
		glClearColor(0, 0, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		glFlush();
		glColor3f(1, 0, 0);
		quad(150, 100, 160, 110, 0);
		glFlush();
		glClearColor(0, 0, 0, 1);
		memset(p, 0xA5, 250 * 250 * 4);
		glReadPixels(0, 0, 250, 250, GL_RGBA, GL_UNSIGNED_BYTE, p);
		check(&fails, near(p + 4 * (105 * 250 + 155), 255, 0, 0, 1), "250 wide (a shadow): (155, 105) the quad: red",
		      p + 4 * (105 * 250 + 155));
		check(&fails, near(p + 4 * (105 * 250 + 145), 0, 0, 255, 1), "250 wide: (145, 105) left of it: blue",
		      p + 4 * (105 * 250 + 145));
		check(&fails, near(p + 4 * (200 * 250 + 200), 0, 0, 255, 1), "250 wide: (200, 200): blue",
		      p + 4 * (200 * 250 + 200));

		/* what a pass of one small quad costs on an 800 x 600 target (a whole-target pass: about 16 ms) */
		glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, rb7);
		glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGBA8, 800, 600);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo7);
		glViewport(0, 0, 800, 600);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0, 800, 0, 600, -1, 1);
		glMatrixMode(GL_MODELVIEW);
		glClear(GL_COLOR_BUFFER_BIT);
		glFinish();
		clock_gettime(CLOCK_MONOTONIC, &ts);
		t0 = ts.tv_sec + ts.tv_nsec * 1e-9;
		for (int i = 0; i < 200; i++) {
			glColor3f((i & 7) / 7.0f, 1, 0);
			quad(100 + (i * 13) % 600, 100 + (i * 7) % 400, 108 + (i * 13) % 600, 108 + (i * 7) % 400, 0);
			glFlush();
		}
		glFinish();
		clock_gettime(CLOCK_MONOTONIC, &ts);
		t1 = ts.tv_sec + ts.tv_nsec * 1e-9;
		printf("  200 passes of an 8 x 8 quad on 800 x 600: %.2f ms a pass\n", (t1 - t0) * 1e3 / 200);
		check(&fails, (t1 - t0) * 1e3 / 200 < 3.0, "a small quad's pass costs its rectangle", (const uint8_t *)"\0\0\0\0");

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glLoadIdentity();
		glOrtho(0, W, 0, H, -1, 1);
		glMatrixMode(GL_MODELVIEW);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
		glViewport(0, 0, W, H);
		glDeleteFramebuffersEXT(1, &fbo7);
		glDeleteFramebuffersEXT(1, &fbo8);
		glDeleteRenderbuffersEXT(1, &rb7);
		glDeleteTextures(1, &t_sh);
		glDeleteTextures(1, &tw);
		free(p);
	}

	printf("14. draws that copy (as the X server copies), done as COPY_IMAGE, on a 256 x 256 texture:\n");
	{
		static const char *vs_src =
			"#version 110\n"
			"attribute vec2 primitive;\n"
			"uniform vec4 v_matrix;\n"
			"uniform vec2 fill_offset;\n"
			"uniform vec2 fill_size_inv;\n"
			"varying vec2 fill_pos;\n"
			"void main() {\n"
			"  gl_Position = vec4(primitive * v_matrix.xz + v_matrix.yw, 0.0, 1.0);\n"
			"  fill_pos = (fill_offset + primitive) * fill_size_inv;\n"
			"}\n";
		static const char *fs_copy =
			"#version 110\n"
			"uniform sampler2D sampler;\n"
			"varying vec2 fill_pos;\n"
			"void main() { gl_FragColor = texture2D(sampler, fill_pos); }\n";
		static const char *fs_tile =
			"#version 110\n"
			"uniform sampler2D sampler;\n"
			"varying vec2 fill_pos;\n"
			"void main() { gl_FragColor = texture2D(sampler, fract(fill_pos)); }\n";
		const char *fs_src[2] = { fs_copy, fs_tile };
		GLuint prog[2], t_src, t_dst, t_rgb3, fbo9, fbo10;
		uint8_t *pat = malloc(64 * 64 * 4), *p = malloc(800 * 600 * 4);
		int bad;
		double t0, t1;
		struct timespec ts;

		for (int k = 0; k < 2; k++) {
			GLuint vs = glCreateShader(GL_VERTEX_SHADER), fs = glCreateShader(GL_FRAGMENT_SHADER);
			GLint ok = 0;

			glShaderSource(vs, 1, &vs_src, NULL);
			glCompileShader(vs);
			glShaderSource(fs, 1, &fs_src[k], NULL);
			glCompileShader(fs);
			prog[k] = glCreateProgram();
			glAttachShader(prog[k], vs);
			glAttachShader(prog[k], fs);
			glBindAttribLocation(prog[k], 0, "primitive");
			glLinkProgram(prog[k]);
			glGetProgramiv(prog[k], GL_LINK_STATUS, &ok);
			check(&fails, ok, k ? "the tile program links" : "the copy program links", (const uint8_t *)"\0\0\0\0");
		}
		/* the source: every texel its own (x * 4, y * 4, x ^ y), alpha 0 (an image with alpha; the RGB case shows it) */
		for (int y = 0; y < 64; y++) {
			for (int x = 0; x < 64; x++) {
				uint8_t *q = pat + 4 * (y * 64 + x);

				q[0] = x * 4;
				q[1] = y * 4;
				q[2] = (x ^ y) * 4 + 1;
				q[3] = 0;
			}
		}
		t_src = texture(64, 64, GL_RGBA, pat, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		t_dst = texture(256, 256, GL_RGBA, NULL, GL_NEAREST);
		glGenFramebuffersEXT(1, &fbo9);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo9);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t_dst, 0);
		glViewport(0, 0, 256, 256);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_SCISSOR_TEST);
		glClearColor(0, 0, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		glEnableVertexAttribArray(0);

/* quads of (x0, y0, x1, y1) boxes (up to 32), the source at box + (ox, oy) (modulo the image for the tile), into a
 * target tw x th */
#define COPY_DRAW(k, boxes, nbox, ox, oy, sw, sh, tw, th)                                                       \
		do {                                                                                                \
			GLshort v_[8 * 32];                                                                             \
			for (int b_ = 0; b_ < (nbox); b_++) {                                                           \
				const int *q_ = (boxes) + 4 * b_;                                                           \
				GLshort *o_ = v_ + 8 * b_;                                                                  \
				o_[0] = q_[0]; o_[1] = q_[1]; o_[2] = q_[0]; o_[3] = q_[3];                                 \
				o_[4] = q_[2]; o_[5] = q_[3]; o_[6] = q_[2]; o_[7] = q_[1];                                 \
			}                                                                                               \
			glUseProgram(prog[k]);                                                                          \
			glUniform4f(glGetUniformLocation(prog[k], "v_matrix"), 2.0f / (tw), -1.0f, 2.0f / (th), -1.0f); \
			glUniform2f(glGetUniformLocation(prog[k], "fill_offset"), (ox), (oy));                          \
			glUniform2f(glGetUniformLocation(prog[k], "fill_size_inv"), 1.0f / (sw), 1.0f / (sh));          \
			glUniform1i(glGetUniformLocation(prog[k], "sampler"), 0);                                       \
			glVertexAttribPointer(0, 2, GL_SHORT, GL_FALSE, 0, v_);                                         \
			glDrawArrays(GL_QUADS, 0, 4 * (nbox));                                                          \
			glUseProgram(0);                                                                                \
		} while (0)

		glBindTexture(GL_TEXTURE_2D, t_src);
		{
			static const int one[4] = { 100, 50, 140, 80 };              /* source (5, 7) */
			static const int two[8] = { 10, 10, 30, 20, 40, 10, 50, 60 }; /* both read box + (2, 3) */
			static const int sc[4] = { 20, 100, 80, 140 };               /* scissored to x < 50 */
			static const int tile[4] = { 100, 140, 250, 250 };           /* the tile from (20, 10) on, wrapping */

			COPY_DRAW(0, one, 1, 5 - 100, 7 - 50, 64, 64, 256, 256);
			COPY_DRAW(0, two, 2, 2, 3, 64, 64, 256, 256);
			glEnable(GL_SCISSOR_TEST);
			glScissor(0, 0, 50, 256);
			COPY_DRAW(0, sc, 1, 1 - 20, 2 - 100, 64, 64, 256, 256);
			glDisable(GL_SCISSOR_TEST);
			COPY_DRAW(1, tile, 1, 20 - 100, 10 - 140, 64, 64, 256, 256);
		}
		memset(p, 0xA5, 256 * 256 * 4);
		glReadPixels(0, 0, 256, 256, GL_RGBA, GL_UNSIGNED_BYTE, p);
		{
			/* every pixel of the target against what it must hold */
			int bad_in = 0, bad_out = 0, fx = -1, fy = -1;

			for (int y = 0; y < 256; y++) {
				for (int x = 0; x < 256; x++) {
					const uint8_t *q = p + 4 * (y * 256 + x);
					int sx = -1, sy = -1;

					if (x >= 100 && x < 140 && y >= 50 && y < 80) { sx = x - 95; sy = y - 43; }
					else if (((x >= 10 && x < 30 && y >= 10 && y < 20) || (x >= 40 && x < 50 && y >= 10 && y < 60))) { sx = x + 2; sy = y + 3; }
					else if (x >= 20 && x < 50 && y >= 100 && y < 140) { sx = x - 19; sy = y - 98; }
					else if (x >= 100 && x < 250 && y >= 140 && y < 250) { sx = (x - 80) % 64; sy = (y - 130) % 64; }
					if (sx >= 0) {
						const uint8_t *e = pat + 4 * (sy * 64 + sx);

						if (memcmp(q, e, 4) && !bad_in++) { fx = x; fy = y; }
					} else if (!(q[0] == 0 && q[1] == 0 && q[2] == 255 && q[3] == 255) && !bad_out++) {
						if (fx < 0) { fx = x; fy = y; }
					}
				}
			}
			if (bad_in || bad_out)
				printf("  %d copied pixels wrong, %d outside changed; the first at (%d, %d)\n", bad_in, bad_out, fx, fy);
			check(&fails, !bad_in, "a rectangle, two in one draw, a scissored one, a tile: exact", p + 4 * (60 * 256 + 110));
			check(&fails, !bad_out, "the pixels around them: blue", p + 4 * (5 * 256 + 5));
		}

		/* an RGB image (alpha kept at 1.0) takes the alpha-0 source; drawn back into the RGBA target its alpha is 1.0 */
		t_rgb3 = texture(64, 64, GL_RGB, NULL, GL_NEAREST);
		glGenFramebuffersEXT(1, &fbo10);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo10);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t_rgb3, 0);
		glViewport(0, 0, 64, 64);
		glBindTexture(GL_TEXTURE_2D, t_src);
		{
			static const int all[4] = { 0, 0, 64, 64 };
			static const int back[4] = { 160, 20, 224, 84 };

			COPY_DRAW(0, all, 1, 0, 0, 64, 64, 64, 64);
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo9);
			glViewport(0, 0, 256, 256);
			glBindTexture(GL_TEXTURE_2D, t_rgb3);
			COPY_DRAW(0, back, 1, -160, -20, 64, 64, 256, 256);
		}
		memset(p, 0xA5, 256 * 256 * 4);
		glReadPixels(0, 0, 256, 256, GL_RGBA, GL_UNSIGNED_BYTE, p);
		bad = 0;
		for (int y = 20; y < 84; y++) {
			for (int x = 160; x < 224; x++) {
				const uint8_t *q = p + 4 * (y * 256 + x), *e = pat + 4 * ((y - 20) * 64 + x - 160);

				bad += q[0] != e[0] || q[1] != e[1] || q[2] != e[2] || q[3] != 255;
			}
		}
		if (bad)
			printf("  %d pixels wrong\n", bad);
		check(&fails, !bad, "RGBA (alpha 0) -> RGB -> RGBA: colours, alpha 1.0", p + 4 * (30 * 256 + 170));

		/* the copy extensions' cases (on a board B without them: pieces, or drawn): an 8 x 8 tile over 200 x 150 (26 x
		 * 19 pieces: a tile), a 2-row run across the repeating 64 x 64 image (5 pieces of 512 pixels: a tile that
		 * repeats in x alone), twenty rectangles in one draw at one offset (one command with a list) */
		{
			static const int wide[4] = { 20, 20, 220, 170 };   /* the 8 x 8 tile from (3, 5) on */
			static const int thin[4] = { 0, 200, 256, 202 };   /* the 64 x 64 image from (60, 9) on, wrapping in x */
			uint8_t *t8 = malloc(8 * 8 * 4);
			int many[4 * 20], bad_in = 0, bad_out = 0, fx = -1, fy = -1;
			GLuint t_small;

			for (int y = 0; y < 8; y++) {
				for (int x = 0; x < 8; x++) {
					uint8_t *q = t8 + 4 * (y * 8 + x);

					q[0] = x * 30 + 7;
					q[1] = y * 30 + 11;
					q[2] = 200 - x * y;
					q[3] = 77;
				}
			}
			for (int i = 0; i < 20; i++) {          /* 10 x 4 boxes in 4 columns, 5 rows: rows 176..199 */
				many[4 * i] = 2 + 15 * (i % 4);
				many[4 * i + 1] = 176 + 5 * (i / 4);
				many[4 * i + 2] = many[4 * i] + 10;
				many[4 * i + 3] = many[4 * i + 1] + 4;
			}
			t_small = texture(8, 8, GL_RGBA, t8, GL_NEAREST);
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo9);
			glViewport(0, 0, 256, 256);
			glClear(GL_COLOR_BUFFER_BIT);
			COPY_DRAW(1, wide, 1, 3 - 20, 5 - 20, 8, 8, 256, 256);
			glBindTexture(GL_TEXTURE_2D, t_src);
			COPY_DRAW(0, thin, 1, 60, 9 - 200, 64, 64, 256, 256);
			COPY_DRAW(0, many, 20, 0, 3 - 176, 64, 64, 256, 256);
			memset(p, 0xA5, 256 * 256 * 4);
			glReadPixels(0, 0, 256, 256, GL_RGBA, GL_UNSIGNED_BYTE, p);
			for (int y = 0; y < 256; y++) {
				for (int x = 0; x < 256; x++) {
					const uint8_t *q = p + 4 * (y * 256 + x), *e = NULL;
					int in_many = 0;

					for (int i = 0; i < 20; i++)
						in_many |= x >= many[4 * i] && x < many[4 * i + 2] && y >= many[4 * i + 1] && y < many[4 * i + 3];
					if (x >= 20 && x < 220 && y >= 20 && y < 170)
						e = t8 + 4 * (((y - 15) & 7) * 8 + ((x - 17) & 7));
					else if (y >= 200 && y < 202)
						e = pat + 4 * ((y - 191) * 64 + (x + 60) % 64);
					else if (in_many)
						e = pat + 4 * ((y - 173) * 64 + x);
					if (e ? memcmp(q, e, 4) != 0 : !(q[0] == 0 && q[1] == 0 && q[2] == 255 && q[3] == 255)) {
						if (fx < 0) {
							fx = x;
							fy = y;
						}
						if (e)
							bad_in++;
						else
							bad_out++;
					}
				}
			}
			if (bad_in || bad_out)
				printf("  %d copied pixels wrong, %d outside changed; the first at (%d, %d)\n", bad_in, bad_out, fx, fy);
			check(&fails, !bad_in, "a small tile over a wide rectangle, a run wrapping in x, 20 rectangles in a draw: exact",
			      p + 4 * (100 * 256 + 100));
			check(&fails, !bad_out, "the pixels around them: blue", p + 4 * (250 * 256 + 250));
			glDeleteTextures(1, &t_small);
			free(t8);
		}

		/* the same program magnified 2x: not a copy, drawn */
		glBindTexture(GL_TEXTURE_2D, t_src);
		{
			static const int mag[4] = { 0, 160, 64, 224 };

			COPY_DRAW(0, mag, 1, 0, -160, 128, 128, 256, 256);   /* texel = (box) / 2 */
		}
		memset(p, 0xA5, 256 * 256 * 4);
		glReadPixels(0, 0, 256, 256, GL_RGBA, GL_UNSIGNED_BYTE, p);
		bad = 0;
		for (int y = 160; y < 224; y++)
			for (int x = 0; x < 64; x++)
				bad += !near(p + 4 * (y * 256 + x), pat[4 * (((y - 160) / 2) * 64 + x / 2)],
				             pat[4 * (((y - 160) / 2) * 64 + x / 2) + 1], pat[4 * (((y - 160) / 2) * 64 + x / 2) + 2], 1);
		if (bad)
			printf("  %d pixels wrong\n", bad);
		check(&fails, !bad, "magnified 2x (drawn): each texel on 2 x 2 pixels", p + 4 * (170 * 256 + 10));

		/* reading the image a draw renders into, as the X server moves a window with GL_NV_texture_barrier and
		 * GL_MESA_tile_raster_order: two rectangles moved 3 right and 5 down over themselves (a copy, in the order
		 * of the move), then a draw magnifying rows above it into rows it reads (drawn: from a copy of the image
		 * taken before its pass) */
		{
			void (*barrier)(void) = (void (*)(void))eglGetProcAddress("glTextureBarrierNV");
			const char *exts = (const char *)glGetString(GL_EXTENSIONS);
			uint8_t *img = malloc(256 * 256 * 4), *want = malloc(256 * 256 * 4);
			static const int moved[8] = { 40, 40, 140, 50, 30, 50, 150, 120 };
			static const int mag2[4] = { 150, 150, 200, 200 };
			GLuint t_self, fbo12;

			check(&fails, exts && strstr(exts, "GL_NV_texture_barrier") && strstr(exts, "GL_MESA_tile_raster_order") &&
			      barrier, "GL_NV_texture_barrier and GL_MESA_tile_raster_order offered", (const uint8_t *)"\0\0\0\0");
			for (int y = 0; y < 256; y++) {
				for (int x = 0; x < 256; x++) {
					uint8_t *q = img + 4 * (y * 256 + x);

					q[0] = x;
					q[1] = y;
					q[2] = (x ^ y) | 1;
					q[3] = 255;
				}
			}
			t_self = texture(256, 256, GL_RGBA, img, GL_NEAREST);
			glGenFramebuffersEXT(1, &fbo12);
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo12);
			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t_self, 0);
			glViewport(0, 0, 256, 256);
			if (barrier)
				barrier();
			COPY_DRAW(0, moved, 2, -3, -5, 256, 256, 256, 256);
			if (barrier)
				barrier();
			COPY_DRAW(0, mag2, 1, 130, 130, 512, 512, 256, 256);   /* texel = (pixel + 130) / 2: rows 140..164 */
			/* what GL gives: the move reads the image as it was, the magnification the image after the move */
			memcpy(want, img, 256 * 256 * 4);
			for (int b = 0; b < 2; b++)
				for (int y = moved[4 * b + 1]; y < moved[4 * b + 3]; y++)
					for (int x = moved[4 * b]; x < moved[4 * b + 2]; x++)
						memcpy(want + 4 * (y * 256 + x), img + 4 * ((y - 5) * 256 + x - 3), 4);
			memcpy(img, want, 256 * 256 * 4);
			for (int y = 150; y < 200; y++)
				for (int x = 150; x < 200; x++)
					memcpy(want + 4 * (y * 256 + x), img + 4 * (((y + 130) / 2) * 256 + (x + 130) / 2), 4);
			memset(p, 0xA5, 256 * 256 * 4);
			glReadPixels(0, 0, 256, 256, GL_RGBA, GL_UNSIGNED_BYTE, p);
			{
				int bad_mv = 0, bad_mag = 0, bad_rest = 0, fx = -1, fy = -1;

				for (int y = 0; y < 256; y++) {
					for (int x = 0; x < 256; x++) {
						const uint8_t *q = p + 4 * (y * 256 + x), *e = want + 4 * (y * 256 + x);
						int in_mv = (x >= 40 && x < 140 && y >= 40 && y < 50) || (x >= 30 && x < 150 && y >= 50 && y < 120);
						int in_mag = x >= 150 && x < 200 && y >= 150 && y < 200;
						int ok = in_mag ? near(q, e[0], e[1], e[2], 1) : !memcmp(q, e, 4);

						if (!ok && fx < 0) {
							fx = x;
							fy = y;
						}
						if (!ok && in_mv)
							bad_mv++;
						else if (!ok && in_mag)
							bad_mag++;
						else if (!ok)
							bad_rest++;
					}
				}
				if (bad_mv || bad_mag || bad_rest)
					printf("  wrong: %d moved, %d magnified, %d elsewhere; the first at (%d, %d)\n", bad_mv, bad_mag,
					       bad_rest, fx, fy);
				check(&fails, !bad_mv, "two rectangles moved over themselves: exact", p + 4 * (60 * 256 + 60));
				check(&fails, !bad_mag, "a draw reading rows of its own target: the image before", p + 4 * (180 * 256 + 180));
				check(&fails, !bad_rest, "the rest of the image unchanged", p + 4 * (220 * 256 + 20));
			}
			glDeleteFramebuffersEXT(1, &fbo12);
			glDeleteTextures(1, &t_self);
			free(img);
			free(want);
		}

		/* moves of an xterm-sized window inside an 800 x 600 image */
		glBindTexture(GL_TEXTURE_2D, t_dst);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 800, 600, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo9);
		glViewport(0, 0, 800, 600);
		glClear(GL_COLOR_BUFFER_BIT);
		glBindTexture(GL_TEXTURE_2D, t_src);
		{
			static const int fill[4] = { 0, 0, 800, 600 };

			glFinish();
			clock_gettime(CLOCK_MONOTONIC, &ts);
			t0 = ts.tv_sec + ts.tv_nsec * 1e-9;
			COPY_DRAW(1, fill, 1, 0, 0, 64, 64, 800, 600);    /* the tile over all of it */
			glFinish();
			clock_gettime(CLOCK_MONOTONIC, &ts);
			t1 = ts.tv_sec + ts.tv_nsec * 1e-9;
			printf("  the 64 x 64 tile over 800 x 600: %.2f ms\n", (t1 - t0) * 1e3);
			memset(p, 0xA5, 800 * 600 * 4);
			glReadPixels(0, 0, 800, 600, GL_RGBA, GL_UNSIGNED_BYTE, p);
			bad = 0;
			for (int y = 0; y < 600; y++)
				for (int x = 0; x < 800; x++)
					bad += memcmp(p + 4 * (y * 800 + x), pat + 4 * ((y % 64) * 64 + x % 64), 4) != 0;
			if (bad)
				printf("  %d pixels of the tile wrong\n", bad);
			check(&fails, !bad, "the 64 x 64 tile over 800 x 600: exact", p + 4 * (300 * 800 + 400));
		}
		{
			GLuint t_tmp = texture(374, 298, GL_RGBA, NULL, GL_NEAREST);
			GLuint fbo11;

			glGenFramebuffersEXT(1, &fbo11);
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo11);
			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, t_tmp, 0);
			glFinish();
			clock_gettime(CLOCK_MONOTONIC, &ts);
			t0 = ts.tv_sec + ts.tv_nsec * 1e-9;
			for (int i = 0; i < 40; i++) {
				/* the X server's way: the window into a temporary image, then back at its new place, 8 right and 5
				 * down; it starts at (40, 60) */
				static const int tmp_box[4] = { 0, 0, 374, 298 };
				int x = 40 + i * 8, y = 60 + i * 5;
				int to[4] = { x + 8, y + 5, x + 8 + 374, y + 5 + 298 };

				glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo11);
				glViewport(0, 0, 374, 298);
				glBindTexture(GL_TEXTURE_2D, t_dst);
				COPY_DRAW(0, tmp_box, 1, x, y, 800, 600, 374, 298);
				glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo9);
				glViewport(0, 0, 800, 600);
				glBindTexture(GL_TEXTURE_2D, t_tmp);
				COPY_DRAW(0, to, 1, -to[0], -to[1], 374, 298, 800, 600);
				glFlush();
			}
			glFinish();
			clock_gettime(CLOCK_MONOTONIC, &ts);
			t1 = ts.tv_sec + ts.tv_nsec * 1e-9;
			printf("  40 moves of a 374 x 298 window (two copies each): %.2f ms a move\n", (t1 - t0) * 1e3 / 40);
			/* two copies of 111 452 pixels at ~9 ns: ~2 ms; drawn (a pass over each target's rows): ~5 ms */
			check(&fails, (t1 - t0) * 1e3 / 40 < 3.5, "a move costs its copies, not passes", (const uint8_t *)"\0\0\0\0");
			/* the window at (360, 260) holds what was at (40, 60): the tile from there */
			memset(p, 0xA5, 800 * 600 * 4);
			glReadPixels(0, 0, 800, 600, GL_RGBA, GL_UNSIGNED_BYTE, p);
			bad = 0;
			for (int y = 0; y < 298; y++)
				for (int x = 0; x < 374; x++)
					bad += memcmp(p + 4 * ((260 + y) * 800 + 360 + x), pat + 4 * (((60 + y) % 64) * 64 + (40 + x) % 64), 4) != 0;
			if (bad)
				printf("  %d pixels of the moved window wrong\n", bad);
			check(&fails, !bad, "the moved window holds what it held at the start", p + 4 * (300 * 800 + 400));
			glDeleteFramebuffersEXT(1, &fbo11);
			glDeleteTextures(1, &t_tmp);
		}
		/* into a pass of the whole target (64 x 64 with depth) that has a draw: drawn in that pass, not copied */
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
		glViewport(0, 0, W, H);
		glClearColor(0, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		glColor3f(1, 1, 0);
		quad(0, 0, 32, 64, 0);
		glBindTexture(GL_TEXTURE_2D, t_src);
		{
			static const int b[4] = { 40, 8, 56, 24 };

			COPY_DRAW(0, b, 1, 3 - 40, 5 - 8, 64, 64, W, H);
		}
		read_back();
		check(&fails, near(at(16, 16), 255, 255, 0, 1), "a whole-target pass: its quad (yellow)", at(16, 16));
		check(&fails, near(at(45, 10), pat[4 * (7 * 64 + 8)], pat[4 * (7 * 64 + 8) + 1], pat[4 * (7 * 64 + 8) + 2], 1),
		      "and the copy drawn in it after the quad", at(45, 10));
#undef COPY_DRAW
		glDisableVertexAttribArray(0);
		glClearColor(0, 0, 0, 1);
		glDeleteFramebuffersEXT(1, &fbo9);
		glDeleteFramebuffersEXT(1, &fbo10);
		glDeleteTextures(1, &t_src);
		glDeleteTextures(1, &t_dst);
		glDeleteTextures(1, &t_rgb3);
		glDeleteProgram(prog[0]);
		glDeleteProgram(prog[1]);
		free(pat);
		free(p);
	}

	printf("15. images rendered with a height no multiple of 8 (board B stores whole bands of 8 rows) leave the next\n"
	       "    object intact: 32 targets 21 x 29 (rendered in shadows of 96-byte rows), each followed by an image:\n");
	{
		enum { N = 32, TW = 21, TH = 29 };
		GLuint tgt[N], vic[N], fbo15;
		uint8_t *pat = malloc(TW * TH * 4), *got = malloc(TW * TH * 4);
		int bad_imgs = 0, bad_px = 0, bad_tgt = 0;

		for (int i = 0; i < TW * TH; i++) {
			pat[4 * i] = (uint8_t)(i * 7 + 1);
			pat[4 * i + 1] = (uint8_t)(i * 13 + 2);
			pat[4 * i + 2] = (uint8_t)(i * 29 + 3);
			pat[4 * i + 3] = 0xFF;
		}
		glGenFramebuffersEXT(1, &fbo15);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo15);
		glViewport(0, 0, TW, TH);
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0, TW, 0, TH, -1, 1);
		glMatrixMode(GL_MODELVIEW);
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_BLEND);
		glDisable(GL_SCISSOR_TEST);
		glDisable(GL_DEPTH_TEST);
		/* each target rendered once (its render shadow is made then), the next image right after it */
		for (int i = 0; i < N; i++) {
			tgt[i] = texture(TW, TH, GL_RGBA, NULL, GL_NEAREST);
			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, tgt[i], 0);
			glColor3f(1, 0, 0);
			quad(0, 0, TW, TH, 0);
			glFinish();
			vic[i] = texture(TW, TH, GL_RGBA, pat, GL_NEAREST);
		}
		/* every target rendered again: green */
		for (int i = 0; i < N; i++) {
			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, tgt[i], 0);
			glColor3f(0, 1, 0);
			quad(0, 0, TW, TH, 0);
			glFlush();
		}
		for (int i = 0; i < N; i++) {
			int n = 0;

			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, vic[i], 0);
			memset(got, 0xA5, TW * TH * 4);
			glReadPixels(0, 0, TW, TH, GL_RGBA, GL_UNSIGNED_BYTE, got);
			for (int k = 0; k < TW * TH * 4; k++)
				n += got[k] != pat[k];
			bad_px += n;
			bad_imgs += n != 0;
			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, tgt[i], 0);
			glReadPixels(0, 0, TW, TH, GL_RGBA, GL_UNSIGNED_BYTE, got);
			for (int k = 0; k < TW * TH; k++)
				bad_tgt += !(got[4 * k] == 0 && got[4 * k + 1] == 255 && got[4 * k + 2] == 0);
		}
		if (bad_imgs || bad_tgt)
			printf("  %d of %d images changed (%d bytes), %d target pixels wrong\n", bad_imgs, N, bad_px, bad_tgt);
		check(&fails, !bad_imgs, "the images after the targets unchanged", (const uint8_t *)"\0\0\0\0");
		check(&fails, !bad_tgt, "the targets green", (const uint8_t *)"\0\0\0\0");
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
		glViewport(0, 0, W, H);
		glDeleteFramebuffersEXT(1, &fbo15);
		glDeleteTextures(N, tgt);
		glDeleteTextures(N, vic);
		free(pat);
		free(got);
		glColor3f(1, 1, 1);
	}

	printf("16. passes that draw into few bands of an 800 x 600 target with depth (no region passes: the whole target),\n"
	       "    bands no draw reaches left alone: 100 passes of an 8 x 8 quad timed, every other row kept:\n");
	{
		GLuint rb16[2], fbo16;
		uint8_t *p = malloc(800 * 600 * 4);
		int bad = 0, bad_q = 0;
		double t0, t1;
		struct timespec ts;

		glGenRenderbuffersEXT(2, rb16);
		glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, rb16[0]);
		glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGBA8, 800, 600);
		glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, rb16[1]);
		glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH24_STENCIL8_EXT, 800, 600);
		glGenFramebuffersEXT(1, &fbo16);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo16);
		glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, rb16[0]);
		glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, rb16[1]);
		glViewport(0, 0, 800, 600);
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0, 800, 0, 600, -1, 1);
		glMatrixMode(GL_MODELVIEW);
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_BLEND);
		glDisable(GL_SCISSOR_TEST);
		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);
		glDepthMask(GL_TRUE);
		/* every row its own colour, drawn: what the bands left alone must still hold */
		glClearColor(0, 0, 0, 1);
		glClearDepth(1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glBegin(GL_QUADS);
		for (int y = 0; y < 600; y += 4) {
			glColor3ub((GLubyte)(y & 255), (GLubyte)(y >> 2), 200);
			glVertex2f(0, y); glVertex2f(800, y); glVertex2f(800, y + 4); glVertex2f(0, y + 4);
		}
		glEnd();
		glFinish();
		clock_gettime(CLOCK_MONOTONIC, &ts);
		t0 = ts.tv_sec + ts.tv_nsec * 1e-9;
		glColor3f(1, 1, 1);
		for (int i = 0; i < 100; i++) {
			int x = 40 + (i * 37) % 720, y = 24 + (i * 7) % 560;

			quad(x, y, x + 8, y + 8, 0);
			glFlush();
		}
		glFinish();
		clock_gettime(CLOCK_MONOTONIC, &ts);
		t1 = ts.tv_sec + ts.tv_nsec * 1e-9;
		memset(p, 0xA5, 800 * 600 * 4);
		glReadPixels(0, 0, 800, 600, GL_RGBA, GL_UNSIGNED_BYTE, p);
		for (int y = 0; y < 600; y++) {
			for (int x = 0; x < 800; x++) {
				const uint8_t *q = p + 4 * (y * 800 + x);
				int in = 0;

				for (int i = 0; i < 100 && !in; i++) {
					int qx = 40 + (i * 37) % 720, qy = 24 + (i * 7) % 560;

					in = x >= qx && x < qx + 8 && y >= qy && y < qy + 8;
				}
				if (in)
					bad_q += !near(q, 255, 255, 255, 1);
				else
					bad += !near(q, (y & ~3) & 255, (y & ~3) >> 2, 200, 1);
			}
		}
		printf("  100 passes of an 8 x 8 quad on 800 x 600 with depth: %.2f ms a pass\n", (t1 - t0) * 1e3 / 100);
		if (bad || bad_q)
			printf("  %d pixels outside the quads wrong, %d inside\n", bad, bad_q);
		check(&fails, !bad, "the rows around the quads kept", p + 4 * (300 * 800 + 5));
		check(&fails, !bad_q, "the quads white", (const uint8_t *)"\377\377\377\377");
		check(&fails, (t1 - t0) * 1e3 / 100 < 4.0, "a pass costs the bands it draws into", (const uint8_t *)"\0\0\0\0");
		glDisable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
		glViewport(0, 0, W, H);
		glDeleteFramebuffersEXT(1, &fbo16);
		glDeleteRenderbuffersEXT(2, rb16);
		free(p);
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
