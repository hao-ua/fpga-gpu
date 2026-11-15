/* SPDX-License-Identifier: (GPL-2.0 WITH Linux-syscall-note) OR MIT */
/*
 * fpgagpu: the userspace interface of the FPGA GPU's DRM driver. The GPU is two FPGA boards on Ethernet; the driver
 * talks to board A over UDP (the protocol of host/fb/fbproto.py), and board A forwards board B's share.
 */
#ifndef _UAPI_FPGAGPU_DRM_H_
#define _UAPI_FPGAGPU_DRM_H_

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define DRM_FPGAGPU_GET_PARAM		0x00
#define DRM_FPGAGPU_GET_STATUS		0x01

#define DRM_IOCTL_FPGAGPU_GET_PARAM	DRM_IOWR(DRM_COMMAND_BASE + DRM_FPGAGPU_GET_PARAM, struct drm_fpgagpu_get_param)
#define DRM_IOCTL_FPGAGPU_GET_STATUS	DRM_IOWR(DRM_COMMAND_BASE + DRM_FPGAGPU_GET_STATUS, struct drm_fpgagpu_get_status)

/* GET_PARAM: one value, from the latest STATUS the boards sent or from the transport */
enum drm_fpgagpu_param {
	FPGAGPU_PARAM_STATUS_LAYOUT	= 0,	/* STATUS word 0: the layout of the words that follow */
	FPGAGPU_PARAM_A_BUILD		= 1,	/* STATUS word 1: board A's build number */
	FPGAGPU_PARAM_A_FLAGS		= 2,	/* STATUS word 3: board A's flags; bits 31 / 30: the vertex core has no
						 * SIN / COS / no integer divider */
	FPGAGPU_PARAM_B_FLAGS		= 3,	/* STATUS word 37: board B's flags; bits 31 / 30 the same for the
						 * fragment core, bits 29 / 28 CLAMP / SQRT-RCPM-RSQM */
	FPGAGPU_PARAM_LINK_FLAGS	= 4,	/* STATUS word 42: bit 0 board B alive, bit 2 the board link trained */
	FPGAGPU_PARAM_RTT_US		= 5,	/* the transport's smoothed round trip to board A, microseconds */
};

struct drm_fpgagpu_get_param {
	__u32 param;		/* in: enum drm_fpgagpu_param */
	__u32 pad;		/* must be 0 */
	__u64 value;		/* out */
};

/* GET_STATUS: a fresh STATUS reply's words (the driver asks board A and waits for the answer) */
#define FPGAGPU_STATUS_MAX_WORDS	256

struct drm_fpgagpu_get_status {
	__u64 words;		/* in: user pointer to __u32[count] */
	__u32 count;		/* in: room, in words; out: words written */
	__u32 flags;		/* must be 0 */
	__u64 serial;		/* out: STATUS replies received since the driver loaded */
};

#if defined(__cplusplus)
}
#endif

#endif /* _UAPI_FPGAGPU_DRM_H_ */
