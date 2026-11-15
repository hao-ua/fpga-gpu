// SPDX-License-Identifier: MIT
/*
 * fpgagpu_status: the fpgagpu DRM driver's parameters and a fresh STATUS reply, through its render node.
 *   cc -O2 -Wall -I/usr/include/libdrm -I../kernel/include/uapi/drm fpgagpu_status.c -o fpgagpu_status
 *   ./fpgagpu_status [-w]        (-w: every STATUS word)
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

static int open_fpgagpu(char *path, size_t room)
{
	int i;

	for (i = 128; i < 192; i++) {
		char name[32] = { 0 };
		struct drm_version v;
		int fd;

		snprintf(path, room, "/dev/dri/renderD%d", i);
		fd = open(path, O_RDWR | O_CLOEXEC);
		if (fd < 0)
			continue;
		memset(&v, 0, sizeof(v));
		v.name = name;
		v.name_len = sizeof(name) - 1;
		if (ioctl(fd, DRM_IOCTL_VERSION, &v) == 0 && strcmp(name, "fpgagpu") == 0)
			return fd;
		close(fd);
	}
	return -1;
}

static int get_param(int fd, uint32_t param, uint64_t *value)
{
	struct drm_fpgagpu_get_param a = { .param = param };

	if (ioctl(fd, DRM_IOCTL_FPGAGPU_GET_PARAM, &a))
		return -errno;
	*value = a.value;
	return 0;
}

int main(int argc, char **argv)
{
	static const struct { uint32_t id; const char *name; } params[] = {
		{ FPGAGPU_PARAM_STATUS_LAYOUT, "STATUS layout" },
		{ FPGAGPU_PARAM_A_BUILD, "board A build" },
		{ FPGAGPU_PARAM_A_FLAGS, "board A flags" },
		{ FPGAGPU_PARAM_B_FLAGS, "board B flags" },
		{ FPGAGPU_PARAM_LINK_FLAGS, "link flags" },
		{ FPGAGPU_PARAM_RTT_US, "round trip us" },
	};
	uint32_t words[FPGAGPU_STATUS_MAX_WORDS];
	struct drm_fpgagpu_get_status st = { 0 };
	int all = argc > 1 && strcmp(argv[1], "-w") == 0;
	char path[64];
	unsigned int i;
	int fd = open_fpgagpu(path, sizeof(path));

	if (fd < 0) {
		fprintf(stderr, "no fpgagpu render node under /dev/dri (is the module loaded?)\n");
		return 1;
	}
	printf("%s\n", path);
	for (i = 0; i < sizeof(params) / sizeof(params[0]); i++) {
		uint64_t v = 0;
		int r = get_param(fd, params[i].id, &v);

		if (r)
			printf("  %-15s error %d (%s)\n", params[i].name, r, strerror(-r));
		else
			printf("  %-15s %llu (0x%08llx)\n", params[i].name, (unsigned long long)v, (unsigned long long)v);
	}
	st.words = (uintptr_t)words;
	st.count = FPGAGPU_STATUS_MAX_WORDS;
	if (ioctl(fd, DRM_IOCTL_FPGAGPU_GET_STATUS, &st)) {
		printf("GET_STATUS: %s\n", strerror(errno));
		return 1;
	}
	printf("GET_STATUS: %u words, reply #%llu\n", st.count, (unsigned long long)st.serial);
	if (st.count > 42)
		printf("  build %u, uptime %.1f s, link %s, B %s\n", words[1], words[2] / 1000.0,
		       (words[42] >> 2) & 1 ? "trained" : "NOT trained", words[42] & 1 ? "alive" : "NOT alive");
	for (i = 0; all && i < st.count; i++)
		printf("  word %3u: %10u  0x%08x\n", i, words[i], words[i]);
	close(fd);
	return 0;
}
