/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * fpgagpu: the FPGA GPU's DRM driver. Internal declarations.
 */
#ifndef FPGAGPU_H
#define FPGAGPU_H

#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/in.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_encoder.h>
#include <drm/drm_gem.h>
#include <drm/drm_mm.h>
#include <drm/drm_plane.h>
#include <linux/bitmap.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

/* protocol (host/fb/fbproto.py, src/rtl/proto/fb_pkg.sv) */
#define FG_MAGIC		0x4246
#define FG_VERSION		1
#define FG_HDR_BYTES		8
#define FG_DGRAM_MAX		1472		/* header + body: one UDP payload in a 1500-byte frame */

enum {
	FG_T_HELLO = 0x01, FG_T_STATUS_REQ = 0x02,
	FG_T_FENCE = 0x40, FG_T_FLUSH = 0x41,
	FG_T_BATCH = 0x60,
};
enum { FG_R_ACK = 0x81, FG_R_STATUS = 0x82, FG_R_EVENT = 0x83, FG_R_READ_DATA = 0x84, FG_R_PHY_REG = 0x85, FG_R_REG = 0x86 };
enum { FG_EV_FENCE = 0x01, FG_EV_PRESENT = 0x02, FG_EV_MODE = 0x03, FG_EV_MEMTEST = 0x04, FG_EV_ERROR = 0x10 };
enum { FG_ACKF_NAK = 1, FG_ACKF_FULL = 2 };

/* STATUS words the driver reads itself */
#define FG_ST_LAYOUT		0
#define FG_ST_BUILD		1
#define FG_ST_FLAGS		3
#define FG_ST_B_FLAGS		37
#define FG_ST_LINK_FLAGS	42
#define FG_LINK_B_ALIVE(f)	((f) & 1u)
#define FG_LINK_TRAINED(f)	(((f) >> 2) & 1u)

#define FG_STATUS_WORDS		256
#define FG_WINDOW		64		/* sequenced datagrams in flight, as fbproto */
#define FG_EVENTS		64		/* recent events kept for waiters */

struct fg_pkt {
	u32 seq;
	u16 len;
	bool resent;				/* sent more than once: no round-trip sample from it (Karn) */
	ktime_t sent;
	u8 data[FG_DGRAM_MAX];
};

struct fg_event {
	u32 seq;
	u8 idx, code;
	bool valid;
	u32 a0, a1, a2;
};

/* one READ in progress (fpgagpu_submit.c): READ_DATA chunks of every request it sent land in buf */
#define FG_RD_CHUNK		1440		/* READ_DATA chunk size the board uses */
#define FG_RD_TAGS		64
struct fg_read {
	u8 *buf;
	u32 total;				/* bytes */
	bool rel;				/* chunk offsets count from the request's start (READ_BUFFER), else from
						 * the rectangle's pixel 0 (READ_IMAGE), base = first pixel x bpp */
	u32 base;
	unsigned long *got;			/* a bit per chunk */
	u32 nchunks, remaining;
	ktime_t last_data;
	unsigned int ntags;
	struct { u32 tag, start; } tags[FG_RD_TAGS];
};

struct fg_net {
	struct device *dev;
	struct socket *sock;
	struct sockaddr_in dst;
	struct task_struct *rx_task, *mt_task;
	u8 *rxbuf;				/* the receive thread's datagram buffer */
	struct mutex lock;			/* everything below; senders, the receive and the maintenance thread */
	wait_queue_head_t wq;

	/* replies */
	u32 status[FG_STATUS_WORDS];
	u32 status_n;
	u64 status_count;
	ktime_t status_at;
	u64 ack_count;
	u32 ack_exp, credit;
	struct fg_event events[FG_EVENTS];
	unsigned int ev_next;
	u32 fence_done;				/* the newest FENCE datagram's seq that board B answered */
	bool fence_valid;
	struct fg_read *rd;			/* the READ in progress, or NULL */

	/* the send window (Go-Back-N, as fbproto.Board._send_seq / _maint_loop) */
	u32 next_seq;
	struct fg_pkt *win;			/* FG_WINDOW slots, a ring in sequence order */
	unsigned int win_head, win_n;
	u32 inflight;				/* the credit the unacknowledged datagrams take */
	ktime_t last_progress, last_resend;
	int resend_req;				/* 1: NAK / heartbeat, 2: FULL */
	bool hb_suspect;
	ktime_t hb_at;

	/* round trip (RFC 6298 style), from datagrams acknowledged without a resend */
	s64 srtt_ns, rttvar_ns;

	/* counters */
	u64 n_sent, n_resent, n_resends, n_rx, n_rx_bad, n_nak, n_full;
};

int fg_net_init(struct fg_net *n, struct device *dev, const char *ip, u16 port);
void fg_net_fini(struct fg_net *n);
int fg_net_hello(struct fg_net *n, unsigned int timeout_ms);
int fg_net_status(struct fg_net *n, u32 *words, u32 room, u32 *got, u64 *serial, unsigned int timeout_ms);
int fg_net_send_seq(struct fg_net *n, u8 type, const void *body, u32 len, u32 *seq, unsigned int timeout_ms);
int fg_net_wait_event(struct fg_net *n, u32 seq, u8 idx, struct fg_event *ev, unsigned int timeout_ms);
int fg_net_wait_fence(struct fg_net *n, u32 seq, long timeout_jiffies);
bool fg_net_fence_passed(struct fg_net *n, u32 seq);
u32 fg_net_rtt_us(struct fg_net *n);
s64 fg_net_srtt_ns(struct fg_net *n);
void fg_net_read_attach(struct fg_net *n, struct fg_read *rd);

/* the atomic helpers' state type, by the kernel built against (Kbuild sets FG_HAVE_ATOMIC_COMMIT) */
#ifdef FG_HAVE_ATOMIC_COMMIT
typedef struct drm_atomic_commit fg_atomic_t;
#else
typedef struct drm_atomic_state fg_atomic_t;
#endif

/* the display (fpgagpu_kms.c): board B's HDMI output */
struct fg_kms {
	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
	u8 *upbuf;				/* one WRITE_IMAGE body */
	u16 shown;				/* the image board B was last told to show: the scanout image, a BO's, 0 */
	bool img_valid;				/* the scanout image's descriptor is on board B */
	bool need_present;			/* after a modeset: send the whole image and PRESENT it */
	u32 img_fourcc, img_w, img_h;
	u32 frame_id;
	int mode_id;
};

/* the boards' memory and handle tables (fpgagpu_mem.c) */
#define FG_HANDLES		1024		/* entries in board A's / B's buffer table and B's image table */
struct fg_mem {
	struct mutex lock;
	struct drm_mm heap_a, heap_b;		/* byte addresses in each board's DDR */
	DECLARE_BITMAP(a_buf, FG_HANDLES);	/* table entries in use */
	DECLARE_BITMAP(b_buf, FG_HANDLES);
	DECLARE_BITMAP(b_img, FG_HANDLES);
};

/* a buffer object in board memory (fpgagpu_gem.c) */
struct fg_bo {
	struct drm_gem_object base;
	u32 kind;				/* enum drm_fpgagpu_bo_kind */
	u16 board_handle;
	u8 format;				/* images: the storage format */
	u16 width, height, pitch;		/* images: level 0 */
	u8 levels, layers;			/* images */
	bool created;				/* CREATE_* was sent: DESTROY it when freed */
	u64 size;				/* the bytes the board object covers */
	struct drm_mm_node node;		/* its range in the board heap */
	u32 last_fence;				/* the newest SUBMIT fence that named it */
	bool fenced;
	struct list_head free_link;		/* on fpgagpu_device.bo_free while DESTROY is pending */
};

struct fpgagpu_device {
	struct drm_device drm;
	struct fg_net net;
	struct fg_kms kms;
	struct fg_mem mem;
	struct mutex submit_lock;		/* one command stream at a time: SUBMIT, READ, the display's uploads */
	struct mutex pass_lock;			/* the two below */
	struct drm_file *pass_owner;		/* the file holding the boards' render state for a pass (PASS), or NULL */
	struct drm_file *last_owner;		/* the file that held it last */
	wait_queue_head_t pass_wq;
	u32 fence_id;				/* the id in the next FENCE body */
	u32 read_tag;				/* the tag of the last READ request */
	spinlock_t free_lock;
	struct list_head bo_free;		/* BOs whose memory goes back once DESTROY is sent */
	struct work_struct free_work;
};

int fg_kms_init(struct fpgagpu_device *fdev);

int fg_mem_init(struct fpgagpu_device *fdev);
void fg_mem_fini(struct fpgagpu_device *fdev);
int fg_mem_alloc(struct fpgagpu_device *fdev, struct fg_bo *bo, u64 size);
void fg_mem_free(struct fpgagpu_device *fdev, struct fg_bo *bo);
u64 fg_image_bytes(u8 format, u16 width, u16 height, u16 pitch, u8 levels, u8 layers, bool vol, u8 samples_log2);
int fg_format_bpp(u8 format);

extern struct workqueue_struct *fg_wq;	/* the module's own: flushed before the module goes */
extern const struct drm_gem_object_funcs fg_bo_funcs;
static inline struct fg_bo *to_fg_bo(struct drm_gem_object *obj)
{
	return obj->funcs == &fg_bo_funcs ? container_of(obj, struct fg_bo, base) : NULL;
}
void fg_bo_free_work(struct work_struct *work);
int fpgagpu_ioctl_bo_create(struct drm_device *drm, void *data, struct drm_file *file);
int fpgagpu_ioctl_submit(struct drm_device *drm, void *data, struct drm_file *file);
int fpgagpu_ioctl_wait(struct drm_device *drm, void *data, struct drm_file *file);
int fpgagpu_ioctl_read(struct drm_device *drm, void *data, struct drm_file *file);
int fpgagpu_ioctl_bo_info(struct drm_device *drm, void *data, struct drm_file *file);
int fpgagpu_ioctl_pass(struct drm_device *drm, void *data, struct drm_file *file);
void fpgagpu_postclose(struct drm_device *drm, struct drm_file *file);

static inline struct fpgagpu_device *to_fpgagpu(struct drm_device *drm)
{
	return container_of(drm, struct fpgagpu_device, drm);
}

#endif /* FPGAGPU_H */
