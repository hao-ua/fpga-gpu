/* SPDX-License-Identifier: MIT */
/*
 * fpgagpu: the Gallium screen of the FPGA GPU, two boards on Ethernet driven through the fpgagpu DRM driver
 * (include/drm-uapi/fpgagpu_drm.h). Board A runs the vertex stage, board B the fragment stage and holds the images.
 */
#ifndef FG_SCREEN_H
#define FG_SCREEN_H

#include <stdbool.h>
#include <stdint.h>

#include "pipe/p_screen.h"
#include "pipe/p_state.h"

struct renderonly;
struct pipe_screen_config;

/* command types of a SUBMIT stream (BATCH sub-messages: {u8 type, u8 flags = 0, u16 body bytes, body padded to 4}) */
#define FG_T_WRITE_IMAGE	0x22
#define FG_T_FLUSH		0x41
#define FG_T_VS_STATE		0x54
#define FG_T_FRAME_BEGIN	0x56

/* image storage formats (CREATE_IMAGE's descriptor) */
#define FG_FMT_RGBA8		0x00
#define FG_FMT_RGB565		0x04
#define FG_FMT_D24S8		0x10
#define FG_FMT_D16		0x11

/* transfer formats of READ_IMAGE / WRITE_IMAGE: the pixel layout in host memory */
#define FG_TF_RAW		0x00	/* the stored bytes */
#define FG_TF_RGBA8888		0x01	/* bytes R, G, B, A */
#define FG_TF_BGRA8888		0x02	/* bytes B, G, R, A */
#define FG_TF_BGRX8888		0x03	/* bytes B, G, R, X: X reads as 255, alpha is written as 255 */
#define FG_TF_RGB565		0x04	/* little-endian {R5, G6, B5} */

/* image usage bits */
#define FG_USE_SAMPLED		0x1
#define FG_USE_RT		0x2
#define FG_USE_SCANOUT		0x4
#define FG_USE_TRANSFER		0x8

/* FLUSH flags */
#define FG_FL_CLEAR_COLOR	0x001
#define FG_FL_CLEAR_SCISSOR	0x002
#define FG_FL_CMASK(m)		(((m) & 0xFu) << 2)	/* colour channels the clear writes: R 8, G 4, B 2, A 1 */
#define FG_FL_DITHER		0x040
#define FG_FL_CLEAR_DEPTH	0x080
#define FG_FL_CLEAR_STENCIL	0x100
#define FG_FL_DEPTH16		0x200

#define FG_MAX_DATA		1440u		/* data bytes of one WRITE_IMAGE */
#define FG_SUBMIT_MAX		(4u << 20)	/* the kernel's limit on one SUBMIT's stream */
#define FG_FLUSH_MAX_WIDTH	1280u		/* the widest colour target board B's FLUSH renders */
#define FG_WAIT_NS		(10ll * 1000 * 1000 * 1000)

struct fg_screen {
   struct pipe_screen base;
   int fd;

   /* GET_PARAM at creation */
   uint32_t a_build;
   uint32_t a_flags;		/* bit 31 the vertex core has no SIN / COS, bit 30 no integer divider */
   uint32_t b_flags;		/* the same for the fragment core */
   uint32_t link_flags;		/* bit 0 board B alive, bit 2 the board link trained */

   bool debug;			/* FPGAGPU_DEBUG: flags at creation, the time each read map spends on stderr */
   unsigned debug_shaders;	/* FPGAGPU_SHADERS=1: each program's assembler text on stderr */

   /* the boards hold one context's state at a time: its programs, attributes, uniforms and draw states */
   void *last_ctx;
   /* image BOs created or destroyed: a draw state names images by handle, and board B turns a handle into an
    * address when the state arrives, so a handle given again starts a new generation of draw ids */
   uint32_t img_gen;
};

static inline struct fg_screen *
fg_screen(struct pipe_screen *pscreen)
{
   return (struct fg_screen *)pscreen;
}

/* a kernel fence: the sequence number of a submission's FENCE */
struct pipe_fence_handle {
   struct pipe_reference reference;
   bool submitted;		/* false: nothing was submitted before it, it has passed */
   uint64_t seqno;
};

struct pipe_screen *fg_screen_create(int fd, const struct pipe_screen_config *config, struct renderonly *ro);

struct pipe_fence_handle *fg_fence_create(bool submitted, uint64_t seqno);

/* one command in the BATCH sub-message format at p: the header, the body (head then tail) and its padding;
 * returns the bytes written */
uint32_t fg_put_cmd(uint8_t *p, uint8_t type, const void *head, uint32_t head_len, const void *tail,
                    uint32_t tail_len);

/* the kernel's SUBMIT / WAIT; 0 or -errno */
int fg_submit(struct fg_screen *screen, const void *cmds, uint32_t bytes, const uint32_t *bos, uint32_t nbos,
              uint64_t *fence);
int fg_wait_fence(struct fg_screen *screen, uint64_t fence, int64_t timeout_ns);
int fg_wait_bo(struct fg_screen *screen, uint32_t gem, int64_t timeout_ns);

static inline void
fg_put32(uint8_t *p, uint32_t v)
{
   p[0] = v & 0xFF;
   p[1] = (v >> 8) & 0xFF;
   p[2] = (v >> 16) & 0xFF;
   p[3] = v >> 24;
}

static inline void
fg_put16(uint8_t *p, uint32_t v)
{
   p[0] = v & 0xFF;
   p[1] = (v >> 8) & 0xFF;
}

#endif /* FG_SCREEN_H */
