// SPDX-License-Identifier: MIT
/*
 * fg_egl_clear: OpenGL ES 2.0 clears through the fpgagpu Mesa driver to the boards, read back and checked.
 *   The fpgagpu render node, a GBM device on it, EGL on that (EGL_PLATFORM_GBM_KHR), an ES 2.0 context without a
 *   surface (EGL_KHR_surfaceless_context), a framebuffer object with a 64 x 64 RGBA8 renderbuffer;
 *   1. glClearColor(0.2, 0.4, 0.6, 1.0), glClear, glReadPixels: every pixel (51, 102, 153, 255) +-1;
 *   2. glClearColor(1.0, 0.0, 0.8, 0.0), glClear, glReadPixels: every pixel (255, 0, 204, 0) +-1 (the image does not
 *      just hold an earlier run's colour);
 *   3. glTexImage2D of a pattern into a 64 x 64 RGBA texture, the texture as a framebuffer object's colour,
 *      glReadPixels: every pixel the pattern (the write path).
 *   The context must be the fpgagpu driver's (GL_RENDERER), not a software fallback.
 *   cc -O2 -Wall -I<prefix>/include fg_egl_clear.c -o fg_egl_clear -L<libdir> -Wl,-rpath-link,<libdir> \
 *      -lEGL -lGLESv2 -lgbm $(pkg-config --cflags --libs libdrm)
 *   fg_egl_clear [render node, default /dev/dri/renderD129]
 */
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <gbm.h>
#include <xf86drm.h>

#define W 64
#define H 64

static double now_ms(void)
{
	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec * 1e3 + t.tv_nsec / 1e6;
}

static int has_ext(const char *list, const char *name)
{
	size_t n = strlen(name);
	const char *p = list;

	while (p && (p = strstr(p, name))) {
		if ((p == list || p[-1] == ' ') && (p[n] == ' ' || p[n] == '\0'))
			return 1;
		p += n;
	}
	return 0;
}

/* clear the bound framebuffer to c, read it back, compare every pixel with want (+-1); returns the bad pixels */
static int clear_and_check(const float c[4], const uint8_t want[4])
{
	static uint8_t px[W * H * 4];
	double t0, t1, t2;
	int bad = 0, shown = 0, i, k;
	GLenum e;

	memset(px, 0xA5, sizeof(px));
	glClearColor(c[0], c[1], c[2], c[3]);
	t0 = now_ms();
	glClear(GL_COLOR_BUFFER_BIT);
	t1 = now_ms();
	glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
	t2 = now_ms();
	e = glGetError();
	if (e != GL_NO_ERROR) {
		printf("  GL error 0x%04x\n", e);
		return W * H;
	}
	for (i = 0; i < W * H; i++) {
		int off = 0;

		for (k = 0; k < 4; k++)
			off |= abs((int)px[4 * i + k] - (int)want[k]) > 1;
		if (off) {
			if (shown++ < 4)
				printf("  pixel (%d, %d) = (%u, %u, %u, %u)\n", i % W, i / W, px[4 * i], px[4 * i + 1],
				       px[4 * i + 2], px[4 * i + 3]);
			bad++;
		}
	}
	printf("  clear (%.1f, %.1f, %.1f, %.1f): want (%u, %u, %u, %u), pixel (0, 0) = (%u, %u, %u, %u), %d of %d pixels "
	       "off; glClear %.1f ms, glReadPixels %.1f ms\n", c[0], c[1], c[2], c[3], want[0], want[1], want[2], want[3],
	       px[0], px[1], px[2], px[3], bad, W * H, t1 - t0, t2 - t1);
	return bad;
}

/* a pattern uploaded with glTexImage2D, read back through a framebuffer object; returns the bad pixels */
static int upload_and_check(void)
{
	static uint8_t src[W * H * 4], px[W * H * 4];
	GLuint tex, fbo;
	GLenum st, e;
	double t0, t1, t2;
	int bad = 0, shown = 0, i;

	for (i = 0; i < W * H; i++) {
		int x = i % W, y = i / W;

		src[4 * i + 0] = (uint8_t)(x * 4);
		src[4 * i + 1] = (uint8_t)(y * 4);
		src[4 * i + 2] = (uint8_t)((x + y) * 2);
		src[4 * i + 3] = (uint8_t)(255 - x);
	}
	memset(px, 0xA5, sizeof(px));
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	t0 = now_ms();
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, W, H, 0, GL_RGBA, GL_UNSIGNED_BYTE, src);
	t1 = now_ms();
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
	st = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (st != GL_FRAMEBUFFER_COMPLETE) {
		printf("  texture framebuffer status 0x%04x\n", st);
		bad = W * H;
	} else {
		glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
		t2 = now_ms();
		e = glGetError();
		if (e != GL_NO_ERROR) {
			printf("  GL error 0x%04x\n", e);
			bad = W * H;
		}
		for (i = 0; !bad && i < W * H; i++) {
			if (memcmp(px + 4 * i, src + 4 * i, 4)) {
				if (shown++ < 4)
					printf("  pixel (%d, %d) = (%u, %u, %u, %u), want (%u, %u, %u, %u)\n", i % W, i / W,
					       px[4 * i], px[4 * i + 1], px[4 * i + 2], px[4 * i + 3], src[4 * i],
					       src[4 * i + 1], src[4 * i + 2], src[4 * i + 3]);
				bad++;
			}
		}
		printf("  glTexImage2D pattern: %d of %d pixels off; glTexImage2D %.1f ms, glReadPixels %.1f ms\n", bad,
		       W * H, t1 - t0, t2 - t1);
	}
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDeleteFramebuffers(1, &fbo);
	glDeleteTextures(1, &tex);
	return bad;
}

int main(int argc, char **argv)
{
	const char *node = argc > 1 ? argv[1] : "/dev/dri/renderD129";
	static const float c1[4] = { 0.2f, 0.4f, 0.6f, 1.0f }, c2[4] = { 1.0f, 0.0f, 0.8f, 0.0f };
	static const uint8_t w1[4] = { 51, 102, 153, 255 }, w2[4] = { 255, 0, 204, 0 };
	static const EGLint ctx_attr[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
	const char *cext, *dext, *glext, *renderer;
	struct gbm_device *gbm;
	drmVersionPtr v;
	EGLDisplay dpy;
	EGLContext ctx;
	EGLConfig cfg = NULL;
	EGLint major, minor;
	GLuint rb, fbo;
	GLenum st;
	int fd, ours, fails = 0;

	fd = open(node, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		perror(node);
		return 1;
	}
	v = drmGetVersion(fd);
	ours = v && v->name && strcmp(v->name, "fpgagpu") == 0;
	printf("%s: kernel driver %s\n", node, v && v->name ? v->name : "?");
	drmFreeVersion(v);
	if (!ours) {
		printf("FAIL: %s is not the fpgagpu device\n", node);
		return 1;
	}

	gbm = gbm_create_device(fd);
	if (!gbm) {
		printf("FAIL: gbm_create_device\n");
		return 1;
	}
	printf("GBM backend: %s\n", gbm_device_get_backend_name(gbm));

	cext = eglQueryString(EGL_NO_DISPLAY, EGL_EXTENSIONS);
	if (!cext || (!has_ext(cext, "EGL_KHR_platform_gbm") && !has_ext(cext, "EGL_MESA_platform_gbm"))) {
		printf("FAIL: no EGL_KHR_platform_gbm\n");
		return 1;
	}
	dpy = eglGetPlatformDisplay(EGL_PLATFORM_GBM_KHR, gbm, NULL);
	if (dpy == EGL_NO_DISPLAY || !eglInitialize(dpy, &major, &minor)) {
		printf("FAIL: EGL display on GBM (0x%04x)\n", eglGetError());
		return 1;
	}
	dext = eglQueryString(dpy, EGL_EXTENSIONS);
	printf("EGL %d.%d, %s\n", major, minor, eglQueryString(dpy, EGL_VENDOR));
	if (!has_ext(dext, "EGL_KHR_surfaceless_context")) {
		printf("FAIL: no EGL_KHR_surfaceless_context\n");
		return 1;
	}
	if (!eglBindAPI(EGL_OPENGL_ES_API)) {
		printf("FAIL: eglBindAPI(EGL_OPENGL_ES_API)\n");
		return 1;
	}
	if (!has_ext(dext, "EGL_KHR_no_config_context") && !has_ext(dext, "EGL_MESA_configless_context")) {
		static const EGLint cfg_attr[] = { EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT, EGL_NONE };
		EGLint n = 0;

		if (!eglChooseConfig(dpy, cfg_attr, &cfg, 1, &n) || n < 1) {
			printf("FAIL: no ES 2.0 EGL config\n");
			return 1;
		}
	}
	ctx = eglCreateContext(dpy, cfg ? cfg : EGL_NO_CONFIG_KHR, EGL_NO_CONTEXT, ctx_attr);
	if (ctx == EGL_NO_CONTEXT) {
		printf("FAIL: eglCreateContext ES 2.0 (0x%04x)\n", eglGetError());
		return 1;
	}
	if (!eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) {
		printf("FAIL: eglMakeCurrent without a surface (0x%04x)\n", eglGetError());
		return 1;
	}

	renderer = (const char *)glGetString(GL_RENDERER);
	printf("GL_VENDOR   = %s\n", (const char *)glGetString(GL_VENDOR));
	printf("GL_RENDERER = %s\n", renderer);
	printf("GL_VERSION  = %s\n", (const char *)glGetString(GL_VERSION));
	if (!renderer || !strstr(renderer, "fpgagpu")) {
		printf("FAIL: the context is not on the fpgagpu driver\n");
		return 1;
	}
	glext = (const char *)glGetString(GL_EXTENSIONS);
	if (!glext || !has_ext(glext, "GL_OES_rgb8_rgba8")) {
		printf("FAIL: no GL_OES_rgb8_rgba8 (RGBA8 renderbuffers)\n");
		return 1;
	}

	glGenRenderbuffers(1, &rb);
	glBindRenderbuffer(GL_RENDERBUFFER, rb);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8_OES, W, H);
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb);
	st = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (st != GL_FRAMEBUFFER_COMPLETE) {
		printf("FAIL: framebuffer status 0x%04x\n", st);
		return 1;
	}
	glViewport(0, 0, W, H);

	fails += clear_and_check(c1, w1) != 0;
	fails += clear_and_check(c2, w2) != 0;
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDeleteFramebuffers(1, &fbo);
	glDeleteRenderbuffers(1, &rb);
	fails += upload_and_check() != 0;

	eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
	eglDestroyContext(dpy, ctx);
	eglTerminate(dpy);
	gbm_device_destroy(gbm);
	close(fd);
	printf("%s\n", fails ? "FAIL" : "PASS");
	return fails != 0;
}
