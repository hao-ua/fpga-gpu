// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * fpgagpu_net: the board protocol's transport inside the kernel. A port of fbproto.Board (host/fb/fbproto.py) and its
 * C port g2_core.c, with the same rules:
 *   - one UDP socket to board A; unsequenced HELLO / STATUS_REQ; sequenced datagrams under Go-Back-N in a 64-entry
 *     window, limited by the credit (free ingress-queue bytes) the ACKs carry;
 *   - resend everything unacknowledged on a NAK, on FULL (after a wait), by the heartbeat rule (an ACK that still
 *     expects the oldest datagram although nothing progressed), and by the stall timer.
 * On board A's own network the timers are fbproto's (2, 4, 5 and 10 ms). Over a VPN to a remote host
 * (host/linux/tools/g2_relay.py) the round trip is tens of ms, so every timer is at least the smoothed round trip:
 * a NAK or a heartbeat ACK that left the board before a resend arrived must not cause another one.
 */
#include <linux/delay.h>
#include <linux/inet.h>
#include <linux/kthread.h>
#include <linux/net.h>
#include <linux/slab.h>
#include <linux/uio.h>
#include <linux/version.h>
#include <net/sock.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#include "fpgagpu.h"

#define RX_BYTES	2048
#define RTT_FLOOR_NS	(200 * NSEC_PER_USEC)

/* ------------------------------------------------------------------ timers, scaled to the round trip */

static s64 rtt_margin_ns(const struct fg_net *n)
{
	return n->srtt_ns + 2 * n->rttvar_ns;
}

/* no second resend sooner: the ACKs and NAKs until then left the board before the first resend reached it */
static s64 guard_ns(const struct fg_net *n)
{
	return max_t(s64, 4 * NSEC_PER_MSEC, rtt_margin_ns(n));
}

/* the heartbeat rule's "nothing progressed for": fbproto's 2 ms on the LAN */
static s64 hb_ns(const struct fg_net *n)
{
	return max_t(s64, 2 * NSEC_PER_MSEC, rtt_margin_ns(n));
}

/* the wait after FULL: fbproto's 5 ms */
static s64 full_wait_ns(const struct fg_net *n)
{
	return max_t(s64, 5 * NSEC_PER_MSEC, rtt_margin_ns(n));
}

/* the stall timer: fbproto's 10 ms */
static s64 stall_ns(const struct fg_net *n)
{
	return max_t(s64, 10 * NSEC_PER_MSEC, 2 * n->srtt_ns + 4 * n->rttvar_ns);
}

static void rtt_sample(struct fg_net *n, s64 s)
{
	if (s < RTT_FLOOR_NS)
		s = RTT_FLOOR_NS;
	if (!n->srtt_ns) {
		n->srtt_ns = s;
		n->rttvar_ns = s / 2;
		return;
	}
	n->rttvar_ns += (abs(n->srtt_ns - s) - n->rttvar_ns) / 4;
	n->srtt_ns += (s - n->srtt_ns) / 8;
}

u32 fg_net_rtt_us(struct fg_net *n)
{
	s64 v;

	mutex_lock(&n->lock);
	v = n->srtt_ns;
	mutex_unlock(&n->lock);
	return (u32)div_s64(v, NSEC_PER_USEC);
}

/* the credit a datagram takes in board A's ingress queue (fbproto: 32 + the body rounded up to 32 bytes) */
static u32 credit_need(u32 body)
{
	return 32 + ALIGN(body, 32);
}

/* ------------------------------------------------------------------ sending */

static void fg_hdr(u8 *d, u8 type, u32 seq)
{
	put_unaligned_le16(FG_MAGIC, d);
	d[2] = FG_VERSION;
	d[3] = type;
	put_unaligned_le32(seq, d + 4);
}

static int fg_sendto(struct fg_net *n, const void *d, size_t len)
{
	struct msghdr msg = { .msg_name = &n->dst, .msg_namelen = sizeof(n->dst) };
	struct kvec v = { .iov_base = (void *)d, .iov_len = len };
	int r = kernel_sendmsg(n->sock, &msg, &v, 1, len);

	return r < 0 ? r : 0;
}

static int fg_send_unseq(struct fg_net *n, u8 type, const void *body, u32 len)
{
	u8 d[FG_HDR_BYTES + 16];

	if (len > sizeof(d) - FG_HDR_BYTES)
		return -EINVAL;
	fg_hdr(d, type, 0);
	if (len)
		memcpy(d + FG_HDR_BYTES, body, len);
	return fg_sendto(n, d, FG_HDR_BYTES + len);
}

static bool fg_room(struct fg_net *n, u32 need)
{
	unsigned int w = READ_ONCE(n->win_n);

	return w < FG_WINDOW && (!w || READ_ONCE(n->inflight) + need <= READ_ONCE(n->credit));
}

int fg_net_send_seq(struct fg_net *n, u8 type, const void *body, u32 len, u32 *seqp, unsigned int timeout_ms)
{
	u32 need = credit_need(len);
	long left = msecs_to_jiffies(timeout_ms);
	struct fg_pkt *p;
	int r;

	if (len > FG_DGRAM_MAX - FG_HDR_BYTES)
		return -EINVAL;
	mutex_lock(&n->lock);
	while (n->win_n >= FG_WINDOW || (n->win_n && n->inflight + need > n->credit)) {
		mutex_unlock(&n->lock);
		left = wait_event_interruptible_timeout(n->wq, fg_room(n, need), left);
		if (left < 0)
			return (int)left;
		if (left == 0)
			return -ETIMEDOUT;
		mutex_lock(&n->lock);
	}
	p = &n->win[(n->win_head + n->win_n) % FG_WINDOW];
	p->seq = n->next_seq;
	n->next_seq = n->next_seq + 1 ? n->next_seq + 1 : 1;		/* 0 is "unsequenced" */
	fg_hdr(p->data, type, p->seq);
	if (len)
		memcpy(p->data + FG_HDR_BYTES, body, len);
	p->len = FG_HDR_BYTES + len;
	p->resent = false;
	p->sent = ktime_get();
	if (!n->win_n)
		n->last_progress = p->sent;
	n->win_n++;
	n->inflight += need;
	r = fg_sendto(n, p->data, p->len);
	n->n_sent++;
	if (seqp)
		*seqp = p->seq;
	mutex_unlock(&n->lock);
	return r;
}

/* send everything unacknowledged again, oldest first; called with the lock held */
static void fg_resend_all(struct fg_net *n, ktime_t now)
{
	unsigned int i;

	for (i = 0; i < n->win_n; i++) {
		struct fg_pkt *p = &n->win[(n->win_head + i) % FG_WINDOW];

		p->resent = true;
		fg_sendto(n, p->data, p->len);
	}
	n->n_resent += n->win_n;
	n->n_resends++;
	n->resend_req = 0;
	n->last_resend = now;
}

/* ------------------------------------------------------------------ receiving */

static void fg_on_ack(struct fg_net *n, u32 exp, u32 flags, u32 free_bytes)
{
	ktime_t now = ktime_get();
	bool progressed = false;

	n->ack_count++;
	while (n->win_n) {
		struct fg_pkt *p = &n->win[n->win_head];

		if ((s32)(p->seq - exp) >= 0)			/* not yet acknowledged (wrap-safe) */
			break;
		if (!p->resent)
			rtt_sample(n, ktime_to_ns(ktime_sub(now, p->sent)));
		n->inflight -= credit_need(p->len - FG_HDR_BYTES);
		n->win_head = (n->win_head + 1) % FG_WINDOW;
		n->win_n--;
		progressed = true;
	}
	if (progressed) {
		n->last_progress = now;
		n->hb_suspect = false;
	}
	n->ack_exp = exp;
	n->credit = free_bytes;
	if (n->win_n && (flags & FG_ACKF_NAK)) {
		n->resend_req = max(n->resend_req, 1);
		n->n_nak++;
	}
	if (n->win_n && (flags & FG_ACKF_FULL)) {
		n->resend_req = 2;
		n->n_full++;
	}
	/* the heartbeat rule: the ACK may be stale, so it only marks a suspect; the maintenance thread resends if it
	 * still holds after a while (fbproto._on_ack / _hb_check) */
	if (!progressed && n->win_n && !n->hb_suspect && exp == n->win[n->win_head].seq &&
	    ktime_to_ns(ktime_sub(now, n->last_progress)) > hb_ns(n)) {
		n->hb_suspect = true;
		n->hb_at = now;
	}
}

static void fg_rx(struct fg_net *n, const u8 *d, int len)
{
	const u8 *b = d + FG_HDR_BYTES;
	int bl = len - FG_HDR_BYTES;
	struct fg_event *ev;
	u32 seq, cw;
	u8 type;
	int i, nw;

	if (len < FG_HDR_BYTES || get_unaligned_le16(d) != FG_MAGIC || d[2] != FG_VERSION) {
		mutex_lock(&n->lock);
		n->n_rx_bad++;
		mutex_unlock(&n->lock);
		return;
	}
	type = d[3];
	seq = get_unaligned_le32(d + 4);
	mutex_lock(&n->lock);
	n->n_rx++;
	switch (type) {
	case FG_R_ACK:
		if (bl >= 16)
			fg_on_ack(n, get_unaligned_le32(b), get_unaligned_le32(b + 4), get_unaligned_le32(b + 8));
		break;
	case FG_R_STATUS:
		nw = min(bl / 4, FG_STATUS_WORDS);
		for (i = 0; i < nw; i++)
			n->status[i] = get_unaligned_le32(b + 4 * i);
		n->status_n = nw;
		n->status_count++;
		n->status_at = ktime_get();
		break;
	case FG_R_EVENT:
		if (bl < 16)
			break;
		cw = get_unaligned_le32(b);
		ev = &n->events[n->ev_next % FG_EVENTS];
		ev->seq = seq;
		ev->code = cw & 0xff;
		ev->idx = (cw >> 8) & 0xff;
		ev->a0 = get_unaligned_le32(b + 4);
		ev->a1 = get_unaligned_le32(b + 8);
		ev->a2 = get_unaligned_le32(b + 12);
		ev->valid = true;
		n->ev_next++;
		if (ev->code == FG_EV_ERROR)
			dev_warn_ratelimited(n->dev, "board error event: seq %u sub-message %u, error %u, type %02x, detail %08x\n",
					     seq, ev->idx, ev->a0, ev->a1, ev->a2);
		break;
	default:						/* READ_DATA and register replies: later phases */
		break;
	}
	mutex_unlock(&n->lock);
	wake_up_all(&n->wq);
}

static int fg_rx_thread(void *arg)
{
	struct fg_net *n = arg;

	while (!kthread_should_stop()) {
		struct sockaddr_in src;
		struct msghdr msg = { .msg_name = &src, .msg_namelen = sizeof(src) };
		struct kvec v = { .iov_base = n->rxbuf, .iov_len = RX_BYTES };
		int len = kernel_recvmsg(n->sock, &msg, &v, 1, RX_BYTES, 0);

		if (len < 0)
			continue;				/* the receive timeout, or kthread_stop() waking us */
		if (src.sin_addr.s_addr != n->dst.sin_addr.s_addr || src.sin_port != n->dst.sin_port) {
			mutex_lock(&n->lock);
			n->n_rx_bad++;
			mutex_unlock(&n->lock);
			continue;
		}
		fg_rx(n, n->rxbuf, len);
	}
	return 0;
}

static int fg_mt_thread(void *arg)
{
	struct fg_net *n = arg;

	while (!kthread_should_stop()) {
		ktime_t now;
		s64 since_resend, since_progress, wait;

		usleep_range(1000, 1500);
		mutex_lock(&n->lock);
		if (!n->win_n) {
			n->resend_req = 0;
			n->hb_suspect = false;
			mutex_unlock(&n->lock);
			continue;
		}
		now = ktime_get();
		since_resend = ktime_to_ns(ktime_sub(now, n->last_resend));
		since_progress = ktime_to_ns(ktime_sub(now, n->last_progress));
		if (n->hb_suspect && ktime_to_ns(ktime_sub(now, n->hb_at)) >= 2 * NSEC_PER_MSEC &&
		    since_resend > guard_ns(n)) {
			n->hb_suspect = false;
			n->resend_req = max(n->resend_req, 1);
		}
		wait = n->resend_req == 2 ? full_wait_ns(n) : guard_ns(n);
		if ((n->resend_req && since_resend > wait) ||
		    (since_progress > stall_ns(n) && since_resend > stall_ns(n)))
			fg_resend_all(n, now);
		mutex_unlock(&n->lock);
	}
	return 0;
}

/* ------------------------------------------------------------------ requests */

int fg_net_hello(struct fg_net *n, unsigned int timeout_ms)
{
	ktime_t t_end = ktime_add_ms(ktime_get(), timeout_ms);
	u8 body[4];
	u64 c0;
	long r;

	put_unaligned_le32(1, body);				/* a new session: the board expects seq 1 next */
	mutex_lock(&n->lock);
	n->win_head = 0;
	n->win_n = 0;
	n->inflight = 0;
	n->next_seq = 1;
	n->resend_req = 0;
	n->hb_suspect = false;
	memset(n->events, 0, sizeof(n->events));
	c0 = n->ack_count;
	mutex_unlock(&n->lock);
	do {
		fg_send_unseq(n, FG_T_HELLO, body, sizeof(body));
		r = wait_event_interruptible_timeout(n->wq, READ_ONCE(n->ack_count) > c0, msecs_to_jiffies(500));
		if (r < 0)
			return (int)r;
		if (r > 0)
			return 0;
	} while (ktime_before(ktime_get(), t_end));
	return -ETIMEDOUT;
}

int fg_net_status(struct fg_net *n, u32 *words, u32 room, u32 *got, u64 *serial, unsigned int timeout_ms)
{
	unsigned int each = max(timeout_ms / 3, 1u);
	ktime_t t0;
	u64 c0;
	long r;
	int tries;

	mutex_lock(&n->lock);
	c0 = n->status_count;
	mutex_unlock(&n->lock);
	for (tries = 0; tries < 3; tries++) {
		t0 = ktime_get();
		fg_send_unseq(n, FG_T_STATUS_REQ, NULL, 0);
		r = wait_event_interruptible_timeout(n->wq, READ_ONCE(n->status_count) > c0, msecs_to_jiffies(each));
		if (r < 0)
			return (int)r;
		if (r > 0) {
			u32 nw;

			mutex_lock(&n->lock);
			if (!n->srtt_ns)				/* the first round-trip estimate, before any ACK */
				rtt_sample(n, ktime_to_ns(ktime_sub(ktime_get(), t0)));
			nw = min(room, n->status_n);
			memcpy(words, n->status, nw * sizeof(u32));
			*got = nw;
			if (serial)
				*serial = n->status_count;
			mutex_unlock(&n->lock);
			return 0;
		}
	}
	return -ETIMEDOUT;
}

/* the event of a sequenced command: (seq, sub-message index); -EIO for an error event (its words in *ev) */
int fg_net_wait_event(struct fg_net *n, u32 seq, u8 idx, struct fg_event *ev, unsigned int timeout_ms)
{
	long left = msecs_to_jiffies(timeout_ms);

	for (;;) {
		unsigned int i, seen;

		mutex_lock(&n->lock);
		for (i = 0; i < FG_EVENTS; i++) {
			struct fg_event *e = &n->events[i];

			if (e->valid && e->seq == seq && e->idx == idx) {
				*ev = *e;
				e->valid = false;
				mutex_unlock(&n->lock);
				return ev->code == FG_EV_ERROR ? -EIO : 0;
			}
		}
		seen = n->ev_next;
		mutex_unlock(&n->lock);
		left = wait_event_interruptible_timeout(n->wq, READ_ONCE(n->ev_next) != seen, left);
		if (left < 0)
			return (int)left;
		if (left == 0)
			return -ETIMEDOUT;
	}
}

/* ------------------------------------------------------------------ set-up */

int fg_net_init(struct fg_net *n, struct device *dev, const char *ip, u16 port)
{
	struct task_struct *t;
	int r;

	n->dev = dev;
	mutex_init(&n->lock);
	init_waitqueue_head(&n->wq);
	n->next_seq = 1;
	n->dst.sin_family = AF_INET;
	n->dst.sin_port = htons(port);
	if (!in4_pton(ip, -1, (u8 *)&n->dst.sin_addr.s_addr, -1, NULL)) {
		dev_err(dev, "ip=%s is not an IPv4 address\n", ip);
		return -EINVAL;
	}
	n->win = kvcalloc(FG_WINDOW, sizeof(*n->win), GFP_KERNEL);
	n->rxbuf = kmalloc(RX_BYTES, GFP_KERNEL);
	if (!n->win || !n->rxbuf) {
		r = -ENOMEM;
		goto fail;
	}
	r = sock_create_kern(&init_net, AF_INET, SOCK_DGRAM, IPPROTO_UDP, &n->sock);
	if (r)
		goto fail;
	lock_sock(n->sock->sk);
	n->sock->sk->sk_rcvtimeo = msecs_to_jiffies(100);
	n->sock->sk->sk_userlocks |= SOCK_RCVBUF_LOCK | SOCK_SNDBUF_LOCK;
	n->sock->sk->sk_rcvbuf = 2 << 20;			/* board A's heartbeat ACKs and READ_DATA bursts */
	n->sock->sk->sk_sndbuf = 2 << 20;
	release_sock(n->sock->sk);

	t = kthread_run(fg_rx_thread, n, "fpgagpu-rx");
	if (IS_ERR(t)) {
		r = PTR_ERR(t);
		goto fail;
	}
	n->rx_task = t;
	t = kthread_run(fg_mt_thread, n, "fpgagpu-mt");
	if (IS_ERR(t)) {
		r = PTR_ERR(t);
		goto fail;
	}
	n->mt_task = t;
	return 0;
fail:
	fg_net_fini(n);
	return r;
}

void fg_net_fini(struct fg_net *n)
{
	if (n->mt_task)
		kthread_stop(n->mt_task);
	if (n->rx_task)
		kthread_stop(n->rx_task);
	n->mt_task = NULL;
	n->rx_task = NULL;
	if (n->sock)
		sock_release(n->sock);
	n->sock = NULL;
	kvfree(n->win);
	n->win = NULL;
	kfree(n->rxbuf);
	n->rxbuf = NULL;
}
