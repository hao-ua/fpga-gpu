// SPDX-License-Identifier: MIT
/*
 * fg_egl_info: what the fpgagpu Mesa driver offers: the EGL extensions on GBM, then for an OpenGL ES 2.0 context and a
 * desktop OpenGL context the version, the shading language version and the extensions (one per line, sorted by the
 * caller if wanted).
 *   cc -O2 -Wall -I<prefix>/include fg_egl_info.c -o fg_egl_info -L<libdir> -Wl,-rpath-link,<libdir> \
 *      -lEGL -lGL -lgbm $(pkg-config --cflags --libs libdrm)
 *   fg_egl_info [render node, default /dev/dri/renderD129]
 */
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/gl.h>
#include <gbm.h>

static void one_per_line(const char *what, const char *list)
{
	char word[128];
	int n = 0;

	printf("%s:\n", what);
	while (list && *list) {
		while (*list == ' ')
			list++;
		if (!*list)
			break;
		n = 0;
		while (*list && *list != ' ' && n < (int)sizeof(word) - 1)
			word[n++] = *list++;
		word[n] = 0;
		printf("  %s\n", word);
	}
}

static void context(EGLDisplay dpy, EGLenum api, const EGLint *attr, const char *name)
{
	EGLContext ctx;

	if (!eglBindAPI(api)) {
		printf("%s: eglBindAPI failed\n", name);
		return;
	}
	ctx = eglCreateContext(dpy, EGL_NO_CONFIG_KHR, EGL_NO_CONTEXT, attr);
	if (ctx == EGL_NO_CONTEXT || !eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) {
		printf("%s: no context (0x%04x)\n", name, eglGetError());
		return;
	}
	printf("== %s: %s, %s, shading language %s\n", name, (const char *)glGetString(GL_RENDERER),
	       (const char *)glGetString(GL_VERSION), (const char *)glGetString(0x8B8C /* GL_SHADING_LANGUAGE_VERSION */));
	one_per_line("extensions", (const char *)glGetString(GL_EXTENSIONS));
	eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
	eglDestroyContext(dpy, ctx);
}

int main(int argc, char **argv)
{
	const char *node = argc > 1 ? argv[1] : "/dev/dri/renderD129";
	static const EGLint es2[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
	int fd = open(node, O_RDWR | O_CLOEXEC);
	struct gbm_device *gbm = fd >= 0 ? gbm_create_device(fd) : NULL;
	EGLDisplay dpy = gbm ? eglGetPlatformDisplay(EGL_PLATFORM_GBM_KHR, gbm, NULL) : EGL_NO_DISPLAY;
	EGLint major, minor;

	if (dpy == EGL_NO_DISPLAY || !eglInitialize(dpy, &major, &minor)) {
		printf("no EGL display on %s\n", node);
		return 1;
	}
	printf("EGL %d.%d\n", major, minor);
	one_per_line("EGL extensions", eglQueryString(dpy, EGL_EXTENSIONS));
	context(dpy, EGL_OPENGL_ES_API, es2, "OpenGL ES 2.0");
	context(dpy, EGL_OPENGL_API, NULL, "desktop OpenGL");
	eglTerminate(dpy);
	return 0;
}
