// SPDX-License-Identifier: MIT
/*
 * fg_egl_threads: two threads of one process draw at the same time, each in a context of its own (OpenGL ES 2.0, GBM
 *   on the render node, no surface) into a 64 x 64 RGBA8 framebuffer object of its own: 300 frames each of a clear and
 *   a quad of the frame's colour, every 10th read back and checked whole. The contexts share the process's file, so the
 *   kernel does not keep their passes apart; the driver does (a context opening a pass waits for the other's to end).
 *   cc -O2 -Wall -I<prefix>/include fg_egl_threads.c -o fg_egl_threads -L<libdir> -Wl,-rpath-link,<libdir> \
 *      -lEGL -lGLESv2 -lgbm -lpthread $(pkg-config --cflags --libs libdrm)
 *   fg_egl_threads [render node, default /dev/dri/renderD129]
 */
#include <fcntl.h>
#include <pthread.h>
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
#define FRAMES 300

static EGLDisplay dpy;
static EGLConfig cfg;

struct job {
	int id;
	int checked, bad_frames, first_bad;
	char err[128];
};

/* the two threads' programs and quads differ, so one drawn with the other's program or state shows */
static const char *vs_src[2] = {
	"attribute vec4 pos;\n"
	"void main() { gl_Position = pos; }\n",
	"attribute vec4 pos;\n"
	"void main() { gl_Position = pos * vec4(-1.0, 1.0, 1.0, 1.0); }\n",
};
static const char *fs_src[2] = {
	"precision mediump float;\n"
	"uniform vec4 c;\n"
	"void main() { gl_FragColor = c; }\n",
	"precision mediump float;\n"
	"uniform vec4 c;\n"
	"void main() { gl_FragColor = c.zyxw; }\n",
};

/* a frame's quad colour: its own for each thread and frame, in steps of 1/255 */
static void color(int id, int f, uint8_t c[4])
{
	c[0] = (uint8_t)(id ? 200 : 20);
	c[1] = (uint8_t)(f & 255);
	c[2] = (uint8_t)((f * 7 + id * 100) & 255);
	c[3] = 255;
}

static void *worker(void *arg)
{
	struct job *j = arg;
	static const EGLint ctx_attr[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
	/* thread 0: the middle quarter; thread 1: the left half, mirrored by its vertex program to the right half */
	static const float quads[2][8] = { { -0.5f, -0.5f, 0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f },
	                                   { -1, -1, 0, -1, -1, 1, 0, 1 } };
	EGLContext ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctx_attr);
	uint8_t *px = malloc(W * H * 4);
	GLuint rb, fbo, p, s[2];
	const char *src[2] = { vs_src[j->id], fs_src[j->id] };
	GLint loc;

	if (ctx == EGL_NO_CONTEXT || !eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) {
		snprintf(j->err, sizeof(j->err), "no context (0x%04x)", eglGetError());
		return NULL;
	}
	glGenRenderbuffers(1, &rb);
	glBindRenderbuffer(GL_RENDERBUFFER, rb);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8_OES, W, H);
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb);
	glViewport(0, 0, W, H);
	p = glCreateProgram();
	for (int i = 0; i < 2; i++) {
		s[i] = glCreateShader(i ? GL_FRAGMENT_SHADER : GL_VERTEX_SHADER);
		glShaderSource(s[i], 1, &src[i], NULL);
		glCompileShader(s[i]);
		glAttachShader(p, s[i]);
	}
	glBindAttribLocation(p, 0, "pos");
	glLinkProgram(p);
	glUseProgram(p);
	loc = glGetUniformLocation(p, "c");
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, quads[j->id]);
	glEnableVertexAttribArray(0);
	j->first_bad = -1;
	for (int f = 0; f < FRAMES; f++) {
		uint8_t c[4];

		color(j->id, f, c);
		glClearColor(j->id ? 0.0f : 1.0f, 0.0f, j->id ? 1.0f : 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glUniform4f(loc, c[0] / 255.0f, c[1] / 255.0f, c[2] / 255.0f, 1.0f);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		glFlush();
		if (f % 10 == 9) {
			int bad = 0;

			memset(px, 0xA5, W * H * 4);
			glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
			for (int y = 0; y < H; y++) {
				for (int x = 0; x < W; x++) {
					const uint8_t *q = px + 4 * (y * W + x);
					int in = j->id ? x >= W / 2 : x >= W / 4 && x < 3 * W / 4 && y >= H / 4 && y < 3 * H / 4;
					uint8_t e[4] = { (uint8_t)(j->id ? 0 : 255), 0, (uint8_t)(j->id ? 255 : 0), 255 };
					uint8_t qc[4] = { j->id ? c[2] : c[0], c[1], j->id ? c[0] : c[2], c[3] };

					if (memcmp(q, in ? qc : e, 4) && !bad++ && j->first_bad < 0)
						printf("  thread %d frame %d: (%d, %d) = (%u, %u, %u, %u), want (%u, %u, %u, %u)\n", j->id, f,
						       x, y, q[0], q[1], q[2], q[3], (in ? qc : e)[0], (in ? qc : e)[1], (in ? qc : e)[2],
						       (in ? qc : e)[3]);
				}
			}
			j->checked++;
			if (bad) {
				if (j->first_bad < 0)
					printf("  thread %d frame %d: %d pixels wrong\n", j->id, f, bad);
				j->bad_frames++;
				if (j->first_bad < 0)
					j->first_bad = f;
			}
		}
	}
	eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
	eglDestroyContext(dpy, ctx);
	free(px);
	return NULL;
}

int main(int argc, char **argv)
{
	const char *node = argc > 1 ? argv[1] : "/dev/dri/renderD129";
	static const EGLint cfg_attr[] = { EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT, EGL_NONE };
	struct job jobs[2] = { { .id = 0 }, { .id = 1 } };
	pthread_t th[2];
	struct gbm_device *gbm;
	drmVersionPtr v;
	EGLint major, minor, n = 0;
	struct timespec t0, t1;
	int fd, fails = 0;

	fd = open(node, O_RDWR | O_CLOEXEC);
	v = fd >= 0 ? drmGetVersion(fd) : NULL;
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
	if (!n)
		cfg = EGL_NO_CONFIG_KHR;
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (int i = 0; i < 2; i++)
		pthread_create(&th[i], NULL, worker, &jobs[i]);
	for (int i = 0; i < 2; i++)
		pthread_join(th[i], NULL);
	clock_gettime(CLOCK_MONOTONIC, &t1);
	printf("two threads, %d frames each: %.0f ms\n", FRAMES,
	       (t1.tv_sec - t0.tv_sec) * 1e3 + (t1.tv_nsec - t0.tv_nsec) / 1e6);
	for (int i = 0; i < 2; i++) {
		struct job *j = &jobs[i];
		int ok = !j->err[0] && j->checked == FRAMES / 10 && !j->bad_frames;

		printf("  thread %d: %s%d frames checked, %d wrong (the first: frame %d)  %s\n", i, j->err, j->checked,
		       j->bad_frames, j->first_bad, ok ? "ok" : "BAD");
		fails += !ok;
	}
	eglTerminate(dpy);
	gbm_device_destroy(gbm);
	close(fd);
	printf("%s\n", fails ? "FAIL" : "PASS");
	return fails != 0;
}
