// SPDX-License-Identifier: MIT
/*
 * fpgagpu_selftest: the fpgagpu driver's buffer objects, SUBMIT, WAIT and READ, end to end through the boards.
 *   1. a 64 x 64 RGBA8 image on board B: FILL_IMAGE with a colour, read back in RGBA8888, every pixel compared;
 *   2. a 4 KB buffer on board A: WRITE_BUFFER of a pattern, read back, compared;
 *   3. SUBMIT of a command the kernel must refuse (MEM_WRITE) and of a handle that is not in the BO list.
 *   cc -O2 -Wall -I/usr/include/libdrm -I../kernel/include/uapi/drm fpgagpu_selftest.c -o fpgagpu_selftest
 */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <drm.h>
#include "fpgagpu_drm.h"

#define T_MEM_WRITE	0x04
#define T_WRITE_BUFFER	0x20
#define T_FILL_IMAGE	0x24

static int fails;

static int open_fpgagpu(void)
{
	char path[32];
	int i;

	for (i = 128; i < 192; i++) {
		char name[32] = { 0 };
		struct drm_version v = { 0 };
		int fd;

		snprintf(path, sizeof(path), "/dev/dri/renderD%d", i);
		fd = open(path, O_RDWR | O_CLOEXEC);
		if (fd < 0)
			continue;
		v.name = name;
		v.name_len = sizeof(name) - 1;
		if (ioctl(fd, DRM_IOCTL_VERSION, &v) == 0 && strcmp(name, "fpgagpu") == 0)
			return fd;
		close(fd);
	}
	return -1;
}

static void check(int ok, const char *what)
{
	printf("  %-60s %s\n", what, ok ? "ok" : "FAILED");
	if (!ok)
		fails++;
}

/* one command in the BATCH sub-message format */
static size_t put_cmd(uint8_t *p, uint8_t type, const void *body, uint16_t len)
{
	p[0] = type;
	p[1] = 0;
	memcpy(p + 2, &len, 2);
	memcpy(p + 4, body, len);
	memset(p + 4 + len, 0, (4 - len % 4) % 4);
	return 4 + ((len + 3u) & ~3u);
}

static int submit(int fd, const uint8_t *cmds, size_t n, const uint32_t *bos, uint32_t nbos, uint64_t *fence)
{
	struct drm_fpgagpu_submit s = { .cmds = (uintptr_t)cmds, .cmds_bytes = (uint32_t)n,
					.bos = (uintptr_t)bos, .nbos = nbos };
	int r = ioctl(fd, DRM_IOCTL_FPGAGPU_SUBMIT, &s) ? -errno : 0;

	if (!r && fence)
		*fence = s.fence;
	return r;
}

static int wait_fence(int fd, uint64_t fence)
{
	struct drm_fpgagpu_wait w = { .fence = fence, .timeout_ns = 5000000000ll };

	return ioctl(fd, DRM_IOCTL_FPGAGPU_WAIT, &w) ? -errno : 0;
}

int main(void)
{
	int fd = open_fpgagpu();
	struct drm_fpgagpu_bo_create img = { .kind = FPGAGPU_BO_B_IMAGE, .usage = 0x9, .pitch = 256, .width = 64,
					     .height = 64, .format = 0x00, .levels = 1, .layers = 1 };
	struct drm_fpgagpu_bo_create buf = { .kind = FPGAGPU_BO_A_BUFFER, .size = 4096 };
	static uint8_t cmds[8192];
	static uint32_t pix[64 * 64];
	static uint8_t data[4096], back[4096];
	uint64_t fence = 0;
	size_t n;
	int i, bad, r;

	if (fd < 0) {
		fprintf(stderr, "no fpgagpu render node (is the module loaded?)\n");
		return 1;
	}

	printf("image: FILL_IMAGE + READ\n");
	r = ioctl(fd, DRM_IOCTL_FPGAGPU_BO_CREATE, &img) ? -errno : 0;
	check(!r, "BO_CREATE 64x64 RGBA8 image");
	if (!r) {
		uint8_t fill[20] = { 0 };
		uint32_t value = 0xFF336699u;			/* R 0x99, G 0x66, B 0x33, A 0xFF in RGBA8888 */
		struct drm_fpgagpu_read rd = { .handle = img.handle, .data = (uintptr_t)pix, .size = sizeof(pix),
					       .w = 64, .h = 64, .tformat = 0x01 };
		uint16_t h = (uint16_t)img.board_handle, w64 = 64;

		printf("    board handle 0x%04x at 0x%08llx, %llu bytes\n", img.board_handle,
		       (unsigned long long)img.board_addr, (unsigned long long)img.size);
		memcpy(fill, &h, 2);
		fill[4] = 0x01;					/* RGBA8888 */
		fill[5] = 0xF;					/* every channel */
		memcpy(fill + 12, &w64, 2);
		memcpy(fill + 14, &w64, 2);
		memcpy(fill + 16, &value, 4);
		n = put_cmd(cmds, T_FILL_IMAGE, fill, sizeof(fill));
		r = submit(fd, cmds, n, &img.handle, 1, &fence);
		check(!r, "SUBMIT FILL_IMAGE");
		check(!r && wait_fence(fd, fence) == 0, "WAIT on its fence");
		r = ioctl(fd, DRM_IOCTL_FPGAGPU_READ, &rd) ? -errno : 0;
		check(!r, "READ the image back (RGBA8888)");
		for (i = 0, bad = 0; !r && i < 64 * 64; i++)
			bad += pix[i] != value;
		check(!r && !bad, "every pixel is the fill colour");
	}

	printf("buffer: WRITE_BUFFER + READ\n");
	r = ioctl(fd, DRM_IOCTL_FPGAGPU_BO_CREATE, &buf) ? -errno : 0;
	check(!r, "BO_CREATE 4 KB buffer on board A");
	if (!r) {
		struct drm_fpgagpu_read rd = { .handle = buf.handle, .data = (uintptr_t)back, .size = sizeof(back) };
		uint8_t body[8 + 1024];
		uint16_t h = (uint16_t)buf.board_handle;
		uint32_t off;

		for (i = 0; i < (int)sizeof(data); i++)
			data[i] = (uint8_t)(i * 7 + 3);
		for (off = 0, n = 0; off < sizeof(data); off += 1024) {
			memset(body, 0, 8);
			memcpy(body, &h, 2);
			memcpy(body + 4, &off, 4);
			memcpy(body + 8, data + off, 1024);
			n += put_cmd(cmds + n, T_WRITE_BUFFER, body, sizeof(body));
		}
		r = submit(fd, cmds, n, &buf.handle, 1, &fence);
		check(!r, "SUBMIT 4 x WRITE_BUFFER");
		check(!r && wait_fence(fd, fence) == 0, "WAIT on its fence");
		r = ioctl(fd, DRM_IOCTL_FPGAGPU_READ, &rd) ? -errno : 0;
		check(!r, "READ the buffer back");
		check(!r && memcmp(data, back, sizeof(data)) == 0, "the bytes match");
	}

	printf("refusals\n");
	{
		uint8_t mw[16] = { 0 };

		n = put_cmd(cmds, T_MEM_WRITE, mw, sizeof(mw));
		check(submit(fd, cmds, n, NULL, 0, NULL) == -EPERM, "MEM_WRITE is refused (EPERM)");
	}
	if (img.handle) {
		uint8_t fill[20] = { 0 };
		uint16_t h = (uint16_t)img.board_handle;

		memcpy(fill, &h, 2);
		n = put_cmd(cmds, T_FILL_IMAGE, fill, sizeof(fill));
		check(submit(fd, cmds, n, NULL, 0, NULL) == -EACCES, "a handle missing from the BO list is refused (EACCES)");
	}
	close(fd);					/* closing the handles sends DESTROY */
	printf("%s\n", fails ? "SELFTEST FAILED" : "SELFTEST PASSED");
	return fails != 0;
}
