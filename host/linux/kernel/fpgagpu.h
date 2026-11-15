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
#include <drm/drm_device.h>

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
u32 fg_net_rtt_us(struct fg_net *n);

struct fpgagpu_device {
	struct drm_device drm;
	struct fg_net net;
};

static inline struct fpgagpu_device *to_fpgagpu(struct drm_device *drm)
{
	return container_of(drm, struct fpgagpu_device, drm);
}

#endif /* FPGAGPU_H */
