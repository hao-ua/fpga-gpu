/* SPDX-License-Identifier: (GPL-2.0 WITH Linux-syscall-note) OR MIT */
/* Copyright (c) 2026 Dmytro Ivanysh */
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
#define DRM_FPGAGPU_BO_CREATE		0x02
#define DRM_FPGAGPU_SUBMIT		0x03
#define DRM_FPGAGPU_WAIT		0x04
#define DRM_FPGAGPU_READ		0x05
#define DRM_FPGAGPU_BO_INFO		0x06
#define DRM_FPGAGPU_PASS		0x07

#define DRM_IOCTL_FPGAGPU_GET_PARAM	DRM_IOWR(DRM_COMMAND_BASE + DRM_FPGAGPU_GET_PARAM, struct drm_fpgagpu_get_param)
#define DRM_IOCTL_FPGAGPU_GET_STATUS	DRM_IOWR(DRM_COMMAND_BASE + DRM_FPGAGPU_GET_STATUS, struct drm_fpgagpu_get_status)
#define DRM_IOCTL_FPGAGPU_BO_CREATE	DRM_IOWR(DRM_COMMAND_BASE + DRM_FPGAGPU_BO_CREATE, struct drm_fpgagpu_bo_create)
#define DRM_IOCTL_FPGAGPU_SUBMIT	DRM_IOWR(DRM_COMMAND_BASE + DRM_FPGAGPU_SUBMIT, struct drm_fpgagpu_submit)
#define DRM_IOCTL_FPGAGPU_WAIT		DRM_IOW(DRM_COMMAND_BASE + DRM_FPGAGPU_WAIT, struct drm_fpgagpu_wait)
#define DRM_IOCTL_FPGAGPU_READ		DRM_IOW(DRM_COMMAND_BASE + DRM_FPGAGPU_READ, struct drm_fpgagpu_read)
#define DRM_IOCTL_FPGAGPU_BO_INFO	DRM_IOWR(DRM_COMMAND_BASE + DRM_FPGAGPU_BO_INFO, struct drm_fpgagpu_bo_info)
#define DRM_IOCTL_FPGAGPU_PASS		DRM_IOWR(DRM_COMMAND_BASE + DRM_FPGAGPU_PASS, struct drm_fpgagpu_pass)

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

/*
 * BO_CREATE: a buffer object in the boards' memory. The kernel allocates its memory in the board's host heap and a
 * board handle, and sends CREATE_BUFFER / CREATE_IMAGE. An image's size is computed from its descriptor, the way board
 * B computes it, so it cannot reach into another object's memory. Closing the GEM handle sends DESTROY.
 */
enum drm_fpgagpu_bo_kind {
	FPGAGPU_BO_A_BUFFER	= 0,	/* board A: vertex / index buffers, vertex programs */
	FPGAGPU_BO_B_BUFFER	= 1,	/* board B: fragment programs */
	FPGAGPU_BO_B_IMAGE	= 2,	/* board B: textures, render targets, depth-stencil, scanout */
};

/* BO_CREATE flags */
#define FPGAGPU_BO_CREATE_ALIAS	(1u << 0)	/* an image over the memory of the image `alias` (no memory of its own):
						 * another descriptor of the same bytes, e.g. a D24S8 image read by the
						 * TMU as RGBA8, or a rectangle of it (alias_offset). It keeps that
						 * image's memory while it exists */

struct drm_fpgagpu_bo_create {
	__u32 kind;		/* in: enum drm_fpgagpu_bo_kind */
	__u32 flags;		/* in: FPGAGPU_BO_CREATE_* */
	__u64 size;		/* in: buffers: bytes; images: ignored. out: the bytes allocated */
	/* images (FPGAGPU_BO_B_IMAGE): CREATE_IMAGE's descriptor (host/fb/fbproto.py create_image) */
	__u16 usage;		/* bits: 0 sampled, 1 render target, 2 scanout, 3 transfer */
	__u16 pitch;		/* level 0's row pitch in bytes */
	__u16 width, height;
	__u8 format;		/* storage format (0x00 RGBA8, 0x04 RGB565, 0x05 RGBA4444, 0x06 RGBA5551, 0x0C P8,
				 * 0x10 D24S8, 0x11 D16, ...); compressed formats are refused */
	__u8 img_flags;		/* bit 0 origin, bits 4:3 log2 samples */
	__u8 levels, layers;
	__u16 vol;		/* 1: a 3D texture of depth = layers */
	__u16 pad;		/* must be 0 */
	/* out */
	__u32 handle;		/* the GEM handle */
	__u32 board_handle;	/* the 16-bit handle that commands name it by */
	__u64 board_addr;	/* its byte address in the board's DDR */
	/* in, with FPGAGPU_BO_CREATE_ALIAS */
	__u32 alias;		/* the GEM handle of the image whose memory the new one describes */
	__u32 alias_offset;	/* where in that memory it starts, a multiple of 32 bytes (0 without the flag): a
				 * rectangle of the image (one level, one layer) whose last row ends inside it */
};

/*
 * SUBMIT: a stream of commands, each {__u8 type, __u8 flags (0), __u16 body bytes, body padded to 4 bytes} (the
 * BATCH sub-message format), sent in order and in one piece: no other submission's commands come between them, so a
 * submission may hold whole frames (FRAME_BEGIN .. FLUSH). Allowed types: WRITE_BUFFER, WRITE_IMAGE, FILL_IMAGE,
 * COPY_IMAGE, DRAW_STATE, VS_PROGRAM, VS_UNIFORMS, VERTEX_ATTR, VS_STATE, DRAW, FRAME_BEGIN, FLUSH. Every board
 * handle a command names must belong to a BO in `bos`. The kernel appends a FENCE; `fence` is waited on with WAIT.
 */
struct drm_fpgagpu_submit {
	__u64 cmds;		/* in: user pointer to the command stream */
	__u32 cmds_bytes;	/* in: its length, at most 4 MB */
	__u32 flags;		/* in: must be 0 */
	__u64 bos;		/* in: user pointer to __u32 GEM handles of the BOs the stream names */
	__u32 nbos;		/* in */
	__u32 pad;		/* must be 0 */
	__u64 fence;		/* out */
};

/* WAIT: until a fence has passed, or a BO is idle (its last submission's fence has passed) */
struct drm_fpgagpu_wait {
	__u64 fence;		/* in: a SUBMIT's fence; used when handle is 0 */
	__u32 handle;		/* in: a BO's GEM handle, or 0 */
	__u32 pad;		/* must be 0 */
	__s64 timeout_ns;	/* in: relative */
};

/* READ: a BO's contents into user memory (READ_BUFFER, or READ_IMAGE of a rectangle in a transfer format) */
struct drm_fpgagpu_read {
	__u32 handle;		/* in: the BO's GEM handle */
	__u32 flags;		/* in: must be 0 */
	__u64 data;		/* in: user pointer */
	__u64 size;		/* in: bytes to read */
	__u64 offset;		/* in: buffers: the byte offset */
	/* images */
	__u16 x, y, w, h;	/* the rectangle */
	__u8 level, layer;
	__u8 tformat;		/* transfer format (0 raw, 1 RGBA8888, 2 BGRA8888, 3 BGRX8888, 4 RGB565, 0x0A RGB888, ...) */
	__u8 pad;		/* must be 0 */
	__u32 first;		/* the first pixel of the rectangle to read (row-major) */
	__u32 pad2;		/* must be 0 */
};

/* BO_INFO: what a BO is; the way to learn it for one imported through PRIME from another process */
struct drm_fpgagpu_bo_info {
	__u32 handle;		/* in: the GEM handle */
	__u32 kind;		/* out: enum drm_fpgagpu_bo_kind */
	__u64 size;		/* out: the bytes the board object covers */
	__u32 board_handle;	/* out */
	__u16 width, height;	/* out, images: level 0 */
	__u16 pitch;		/* out, images: level 0's row pitch */
	__u8 format;		/* out, images: the storage format */
	__u8 levels, layers;	/* out, images */
	__u8 pad;		/* 0 */
	__u16 pad2;		/* 0 */
};

/*
 * PASS: the boards hold one render state: board A's program, attributes, uniforms and frame, board B's draw states. A
 * process takes them for a render pass (FRAME_BEGIN .. FLUSH, which may take several SUBMITs) and gives them back
 * after it; while one process holds them, another's BEGIN waits, at most timeout_ns, after which it takes them anyway.
 * Closing the file gives them back too. state_lost 1: another process held them since this one last did, so nothing it
 * sent before (programs, attributes, uniforms, draw states) is on the boards any more. state_lost 2: a board restarted
 * since this file's last BEGIN. The driver made a new session and described every buffer object again, but what they
 * held (program words, vertex data, images) is gone too and must be written again.
 */
enum drm_fpgagpu_pass_op {
	FPGAGPU_PASS_BEGIN	= 0,
	FPGAGPU_PASS_END	= 1,
};

struct drm_fpgagpu_pass {
	__u32 op;		/* in: enum drm_fpgagpu_pass_op */
	__u32 state_lost;	/* out, BEGIN: 0, 1 or 2 */
	__s64 timeout_ns;	/* in, BEGIN */
};

#if defined(__cplusplus)
}
#endif

#endif /* _UAPI_FPGAGPU_DRM_H_ */
