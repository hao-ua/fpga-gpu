// SPDX-License-Identifier: MIT
/*
 * fg_kms_flip: OpenGL ES 2.0 into a GBM surface on the fpgagpu card, its buffers flipped onto board B's HDMI with KMS,
 * the way SDL's kmsdrm video driver runs a game: GBM surface, EGL window surface, eglSwapBuffers,
 * gbm_surface_lock_front_buffer, drmModeAddFB2, drmModeSetCrtc, drmModePageFlip. Needs DRM master on the card (no X
 * on it). Each frame clears to its own colour; the last one is read back before the swap.
 *   cc -O2 -Wall -I<prefix>/include fg_kms_flip.c -o fg_kms_flip -L<libdir> -Wl,-rpath-link,<libdir> \
 *      -lEGL -lGLESv2 -lgbm $(pkg-config --cflags --libs libdrm)
 *   fg_kms_flip [card, default /dev/dri/card1] [frames, default 60] [width height, default 640 480]
 */
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <drm_fourcc.h>
#include <gbm.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

static int flip_pending;

static void on_flip(int fd, unsigned int seq, unsigned int sec, unsigned int usec, void *data)
{
	flip_pending = 0;
}

static double now_ms(void)
{
	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec * 1e3 + t.tv_nsec / 1e6;
}

static uint32_t fb_of(int fd, struct gbm_bo *bo)
{
	uint32_t *fb = gbm_bo_get_user_data(bo), handles[4] = { 0 }, pitches[4] = { 0 }, offsets[4] = { 0 };

	if (fb)
		return *fb;
	fb = calloc(1, sizeof(*fb));
	handles[0] = gbm_bo_get_handle(bo).u32;
	pitches[0] = gbm_bo_get_stride(bo);
	if (drmModeAddFB2(fd, gbm_bo_get_width(bo), gbm_bo_get_height(bo), gbm_bo_get_format(bo), handles, pitches,
			  offsets, fb, 0)) {
		printf("  drmModeAddFB2 (handle %u, pitch %u, %ux%u) failed: %s\n", handles[0], pitches[0],
		       gbm_bo_get_width(bo), gbm_bo_get_height(bo), strerror(errno));
		free(fb);
		return 0;
	}
	gbm_bo_set_user_data(bo, fb, NULL);
	return *fb;
}

int main(int argc, char **argv)
{
	const char *card = argc > 1 ? argv[1] : "/dev/dri/card1";
	int frames = argc > 2 ? atoi(argv[2]) : 60, w = argc > 4 ? atoi(argv[3]) : 640, h = argc > 4 ? atoi(argv[4]) : 480;
	static const EGLint ctx_attr[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
	static const EGLint cfg_attr[] = { EGL_SURFACE_TYPE, EGL_WINDOW_BIT, EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8,
					   EGL_BLUE_SIZE, 8, EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT, EGL_NONE };
	drmEventContext ev = { .version = 2, .page_flip_handler = on_flip };
	drmModeRes *res;
	drmModeConnector *conn = NULL;
	drmModeModeInfo *mode = NULL;
	uint32_t crtc;
	struct gbm_device *gbm;
	struct gbm_surface *gs;
	struct gbm_bo *prev = NULL;
	EGLDisplay dpy;
	EGLConfig cfgs[64], cfg = NULL;
	EGLContext ctx;
	EGLSurface surf;
	EGLint major, minor, n = 0;
	uint8_t px[4];
	double t0;
	int fd, i, fails = 0;

	fd = open(card, O_RDWR | O_CLOEXEC);
	if (fd < 0 || drmSetMaster(fd)) {
		printf("FAIL: %s: %s (DRM master needed)\n", card, strerror(errno));
		return 1;
	}
	res = drmModeGetResources(fd);
	for (i = 0; res && i < res->count_connectors && !conn; i++) {
		conn = drmModeGetConnector(fd, res->connectors[i]);
		if (conn && conn->connection != DRM_MODE_CONNECTED) {
			drmModeFreeConnector(conn);
			conn = NULL;
		}
	}
	if (!conn || !res->count_crtcs) {
		printf("FAIL: no connected connector\n");
		return 1;
	}
	for (i = 0; i < conn->count_modes; i++)
		if (conn->modes[i].hdisplay == w && conn->modes[i].vdisplay == h)
			mode = &conn->modes[i];
	if (!mode) {
		printf("FAIL: no %dx%d mode\n", w, h);
		return 1;
	}
	crtc = res->crtcs[0];

	gbm = gbm_create_device(fd);
	gs = gbm ? gbm_surface_create(gbm, w, h, GBM_FORMAT_XRGB8888, GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING) : NULL;
	if (!gs) {
		printf("FAIL: gbm_surface_create %dx%d XRGB8888\n", w, h);
		return 1;
	}
	dpy = eglGetPlatformDisplay(EGL_PLATFORM_GBM_KHR, gbm, NULL);
	if (dpy == EGL_NO_DISPLAY || !eglInitialize(dpy, &major, &minor) || !eglBindAPI(EGL_OPENGL_ES_API)) {
		printf("FAIL: EGL on GBM (0x%04x)\n", eglGetError());
		return 1;
	}
	eglChooseConfig(dpy, cfg_attr, cfgs, 64, &n);
	for (i = 0; i < n && !cfg; i++) {
		EGLint id = 0;

		eglGetConfigAttrib(dpy, cfgs[i], EGL_NATIVE_VISUAL_ID, &id);
		if ((uint32_t)id == GBM_FORMAT_XRGB8888)
			cfg = cfgs[i];
	}
	printf("%d EGL configs, XRGB8888 one %s\n", n, cfg ? "found" : "NOT found");
	if (!cfg)
		return 1;
	ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctx_attr);
	surf = eglCreateWindowSurface(dpy, cfg, (EGLNativeWindowType)gs, NULL);
	if (ctx == EGL_NO_CONTEXT || surf == EGL_NO_SURFACE || !eglMakeCurrent(dpy, surf, surf, ctx)) {
		printf("FAIL: context / window surface (0x%04x)\n", eglGetError());
		return 1;
	}
	printf("GL_RENDERER = %s\n", (const char *)glGetString(GL_RENDERER));

	t0 = now_ms();
	for (int f = 0; f < frames; f++) {
		struct gbm_bo *bo;
		uint32_t fb;
		float k = (float)(f % 30) / 29.0f;

		glViewport(0, 0, w, h);
		glClearColor(k, 0.5f, 1.0f - k, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		if (f == frames - 1) {
			glReadPixels(w / 2, h / 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, px);
			printf("  last frame's centre pixel (%u, %u, %u, %u), want (%d, 128, %d, 255)\n", px[0], px[1], px[2],
			       px[3], (int)(k * 255 + 0.5f), (int)((1 - k) * 255 + 0.5f));
			fails += abs(px[1] - 128) > 1;
		}
		if (!eglSwapBuffers(dpy, surf)) {
			printf("FAIL: eglSwapBuffers (0x%04x)\n", eglGetError());
			return 1;
		}
		bo = gbm_surface_lock_front_buffer(gs);
		fb = bo ? fb_of(fd, bo) : 0;
		if (!fb) {
			printf("FAIL: no framebuffer for frame %d\n", f);
			return 1;
		}
		if (f == 0) {
			if (drmModeSetCrtc(fd, crtc, fb, 0, 0, &conn->connector_id, 1, mode)) {
				printf("FAIL: drmModeSetCrtc: %s\n", strerror(errno));
				return 1;
			}
			printf("  frame 0 shown: buffer %ux%u, pitch %u, KMS handle %u\n", gbm_bo_get_width(bo),
			       gbm_bo_get_height(bo), gbm_bo_get_stride(bo), gbm_bo_get_handle(bo).u32);
		} else {
			flip_pending = 1;
			if (drmModePageFlip(fd, crtc, fb, DRM_MODE_PAGE_FLIP_EVENT, NULL)) {
				printf("FAIL: drmModePageFlip: %s\n", strerror(errno));
				return 1;
			}
			while (flip_pending) {
				struct pollfd p = { .fd = fd, .events = POLLIN };

				if (poll(&p, 1, 2000) <= 0) {
					printf("FAIL: no flip event\n");
					return 1;
				}
				drmHandleEvent(fd, &ev);
			}
		}
		if (prev)
			gbm_surface_release_buffer(gs, prev);
		prev = bo;
	}
	printf("%d frames in %.1f s: %.1f frames a second\n", frames, (now_ms() - t0) / 1e3,
	       frames * 1e3 / (now_ms() - t0));
	sleep(2);
	printf("%s\n", fails ? "FAIL" : "PASS");
	return fails != 0;
}
