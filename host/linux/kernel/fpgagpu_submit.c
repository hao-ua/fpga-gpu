// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * fpgagpu_submit: SUBMIT, WAIT and READ.
 *
 * SUBMIT takes a stream of commands in the BATCH sub-message format ({u8 type, u8 flags, u16 bytes, body padded to 4}),
 * checks every one (an exact list of types, and every board handle a command names must belong to one of the
 * submission's BOs, of the right kind), packs them into BATCH datagrams in order (a command too long for a BATCH goes
 * alone), and appends a FENCE. The board bounds-checks everything it reaches through a handle, and only the kernel
 * creates and destroys handles, so a process reaches only its own memory. The whole stream goes under one lock: the
 * hardware has one render context, and a frame's commands must not be interleaved with another process's.
 *
 * A fence is the FENCE datagram's sequence number; the boards answer in order, so "passed" is a monotonic comparison
 * with the newest FENCE event (fpgagpu_net.c).
 *
 * READ follows fbproto.Board._read: requests of up to 256 KB, READ_DATA chunks of 1440 bytes placed by offset, and the
 * chunks still missing after a quiet spell asked for again, with the quiet spell scaled to the round trip.
 */
#include <linux/bitmap.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>

#include "fpgagpu.h"
#include "fpgagpu_drm.h"

#define T_WRITE_BUFFER	0x20
#define T_READ_BUFFER	0x21
#define T_WRITE_IMAGE	0x22
#define T_READ_IMAGE	0x23
#define T_FILL_IMAGE	0x24
#define T_COPY_IMAGE	0x25
#define T_DRAW_STATE	0x50
#define T_VS_PROGRAM	0x51
#define T_VS_UNIFORMS	0x52
#define T_VERTEX_ATTR	0x53
#define T_VS_STATE	0x54
#define T_DRAW		0x55
#define T_FRAME_BEGIN	0x56

#define BATCH_MAX	1464u		/* body bytes of a BATCH datagram */
#define SUBMIT_MAX	(4u << 20)
#define SUBMIT_BOS_MAX	4096u
#define READ_MAX	(64u << 20)
#define READ_REQ_MAX	(256u * 1024 / FG_RD_CHUNK * FG_RD_CHUNK)
#define SEND_TIMEOUT_MS	10000
#define READ_IDLE_MS	10000

enum { WANT_A_BUFFER, WANT_B_BUFFER, WANT_IMAGE, WANT_BUFFER };

struct fg_allow {
	u16 *handle;
	u8 *kind;
	unsigned int n;
};

static int fg_handle_ok(const struct fg_allow *al, u16 h, int want)
{
	unsigned int i;

	for (i = 0; i < al->n; i++) {
		if (al->handle[i] != h)
			continue;
		switch (want) {
		case WANT_A_BUFFER:
			return al->kind[i] == FPGAGPU_BO_A_BUFFER ? 0 : -EACCES;
		case WANT_B_BUFFER:
			return al->kind[i] == FPGAGPU_BO_B_BUFFER ? 0 : -EACCES;
		case WANT_IMAGE:
			return al->kind[i] == FPGAGPU_BO_B_IMAGE ? 0 : -EACCES;
		default:
			return al->kind[i] != FPGAGPU_BO_B_IMAGE ? 0 : -EACCES;
		}
	}
	return -EACCES;
}

/* one command's type, length and handles; the layouts are host/fb/fbproto.py's packers */
static int fg_check_cmd(const struct fg_allow *al, u8 type, const u8 *b, u32 len)
{
	unsigned int i;
	int r;

	switch (type) {
	case T_WRITE_BUFFER:			/* handle, 0, offset, data */
		return len < 8 ? -EINVAL : fg_handle_ok(al, get_unaligned_le16(b), WANT_BUFFER);
	case T_WRITE_IMAGE:			/* handle, level, layer, tformat, ..., x, y, w, h, first, 0, data */
		return len < 24 ? -EINVAL : fg_handle_ok(al, get_unaligned_le16(b), WANT_IMAGE);
	case T_FILL_IMAGE:			/* handle, level, layer, tformat, mask, 0, x, y, w, h, value */
		return len < 20 ? -EINVAL : fg_handle_ok(al, get_unaligned_le16(b), WANT_IMAGE);
	case T_COPY_IMAGE:			/* src, level, layer, dst, level, layer, rectangles, flags, key */
		if (len < 36)
			return -EINVAL;
		r = fg_handle_ok(al, get_unaligned_le16(b), WANT_IMAGE);
		return r ? r : fg_handle_ok(al, get_unaligned_le16(b + 4), WANT_IMAGE);
	case T_DRAW_STATE:			/* draw id, h0..h22, uniforms: h11 [15:0] the program buffer, h15..h22 samplers */
		if (len < 4 + 23 * 4)
			return -EINVAL;
		if ((get_unaligned_le32(b + 4 + 11 * 4) & 0xFFFF) &&
		    (r = fg_handle_ok(al, get_unaligned_le32(b + 4 + 11 * 4) & 0xFFFF, WANT_B_BUFFER)))
			return r;
		for (i = 15; i <= 22; i++) {
			u16 h = get_unaligned_le32(b + 4 + 4 * i) & 0xFFFF;

			if (h && (r = fg_handle_ok(al, h, WANT_IMAGE)))
				return r;
		}
		return 0;
	case T_VS_PROGRAM:			/* handle, 0, offset, instructions */
		return len < 12 ? -EINVAL : fg_handle_ok(al, get_unaligned_le16(b), WANT_A_BUFFER);
	case T_VS_UNIFORMS:			/* first, count, vec4s */
		return len < 4 ? -EINVAL : 0;
	case T_VERTEX_ATTR:			/* {flags, index}, handle, 0, stride, offset, constant */
		if (len < 16)
			return -EINVAL;
		return (get_unaligned_le32(b) & 0x100) ? fg_handle_ok(al, get_unaligned_le16(b + 4), WANT_A_BUFFER) : 0;
	case T_VS_STATE:			/* 13 words (15 with the draw's rectangle) */
		return len < 13 * 4 ? -EINVAL : 0;
	case T_DRAW:				/* mode, indexed, u8, 0, index handle, 0, offset, first, count */
		if (len < 20)
			return -EINVAL;
		return b[1] ? fg_handle_ok(al, get_unaligned_le16(b + 4), WANT_A_BUFFER) : 0;
	case T_FRAME_BEGIN:
		return len < 4 ? -EINVAL : 0;
	case FG_T_FLUSH:			/* {flags, target}, clear, ..., {stencil mask, depth-stencil} */
		if (len < 28)
			return -EINVAL;
		r = fg_handle_ok(al, get_unaligned_le32(b) & 0xFFFF, WANT_IMAGE);
		if (!r && (get_unaligned_le32(b + 24) & 0xFFFF))
			r = fg_handle_ok(al, get_unaligned_le32(b + 24) & 0xFFFF, WANT_IMAGE);
		return r;
	default:				/* everything else is the kernel's, or not offered */
		return -EPERM;
	}
}

static int fg_check_stream(const struct fg_allow *al, const u8 *c, u32 bytes)
{
	u32 p = 0;

	while (p < bytes) {
		u32 blen, padded;
		int r;

		if (bytes - p < 4)
			return -EINVAL;
		blen = get_unaligned_le16(c + p + 2);
		padded = ALIGN(blen, 4);
		if (c[p + 1] || bytes - p - 4 < padded || blen > FG_DGRAM_MAX - FG_HDR_BYTES)
			return -EINVAL;
		r = fg_check_cmd(al, c[p], c + p + 4, blen);
		if (r)
			return r;
		p += 4 + padded;
	}
	return 0;
}

static int fg_send_stream(struct fpgagpu_device *fdev, const u8 *c, u32 bytes)
{
	u8 *batch = kmalloc(BATCH_MAX, GFP_KERNEL);
	u32 used = 0, p = 0;
	int r = 0;

	if (!batch)
		return -ENOMEM;
	while (p < bytes && !r) {
		u8 type = c[p];
		u32 blen = get_unaligned_le16(c + p + 2), padded = ALIGN(blen, 4), n = 4 + padded;
		const u8 *body = c + p + 4;

		if (n > BATCH_MAX) {				/* too long for a BATCH: alone, after what is collected */
			if (used)
				r = fg_net_send_seq(&fdev->net, FG_T_BATCH, batch, used, NULL, SEND_TIMEOUT_MS);
			used = 0;
			if (!r)
				r = fg_net_send_seq(&fdev->net, type, body, blen, NULL, SEND_TIMEOUT_MS);
		} else {
			if (used + n > BATCH_MAX) {
				r = fg_net_send_seq(&fdev->net, FG_T_BATCH, batch, used, NULL, SEND_TIMEOUT_MS);
				used = 0;
			}
			batch[used] = type;
			batch[used + 1] = 0;
			put_unaligned_le16(blen, batch + used + 2);
			memcpy(batch + used + 4, body, blen);
			memset(batch + used + 4 + blen, 0, padded - blen);
			used += n;
		}
		p += 4 + padded;
	}
	if (!r && used)
		r = fg_net_send_seq(&fdev->net, FG_T_BATCH, batch, used, NULL, SEND_TIMEOUT_MS);
	kfree(batch);
	return r;
}

/* a FENCE of its own: its seq is the fence */
static int fg_send_fence(struct fpgagpu_device *fdev, u32 *fence)
{
	u8 b[4];

	put_unaligned_le32(++fdev->fence_id, b);
	return fg_net_send_seq(&fdev->net, FG_T_FENCE, b, sizeof(b), fence, SEND_TIMEOUT_MS);
}

int fpgagpu_ioctl_submit(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct fpgagpu_device *fdev = to_fpgagpu(drm);
	struct drm_fpgagpu_submit *a = data;
	struct drm_gem_object **objs = NULL;
	struct fg_allow al = { 0 };
	u8 *cmds = NULL;
	u32 fence = 0;
	unsigned int i;
	int r, idx;

	if (a->flags || a->pad || !a->cmds_bytes || a->cmds_bytes > SUBMIT_MAX || a->nbos > SUBMIT_BOS_MAX)
		return -EINVAL;
	if (a->nbos) {
		r = drm_gem_objects_lookup(file, u64_to_user_ptr(a->bos), a->nbos, &objs);
		if (r)
			return r;
	}
	al.handle = kmalloc_array(max(a->nbos, 1u), sizeof(*al.handle), GFP_KERNEL);
	al.kind = kmalloc_array(max(a->nbos, 1u), sizeof(*al.kind), GFP_KERNEL);
	cmds = kvmalloc(a->cmds_bytes, GFP_KERNEL);
	if (!al.handle || !al.kind || !cmds) {
		r = -ENOMEM;
		goto out;
	}
	for (i = 0; i < a->nbos; i++) {
		struct fg_bo *bo = to_fg_bo(objs[i]);

		if (!bo) {				/* a host-memory (dumb) object: commands cannot name it */
			r = -EINVAL;
			goto out;
		}
		al.handle[i] = bo->board_handle;
		al.kind[i] = bo->kind;
	}
	al.n = a->nbos;
	if (copy_from_user(cmds, u64_to_user_ptr(a->cmds), a->cmds_bytes)) {
		r = -EFAULT;
		goto out;
	}
	r = fg_check_stream(&al, cmds, a->cmds_bytes);
	if (r)
		goto out;
	if (!drm_dev_enter(drm, &idx)) {
		r = -ENODEV;
		goto out;
	}
	mutex_lock(&fdev->submit_lock);
	r = fg_send_stream(fdev, cmds, a->cmds_bytes);
	if (!r)
		r = fg_send_fence(fdev, &fence);
	mutex_unlock(&fdev->submit_lock);
	drm_dev_exit(idx);
	if (!r) {
		a->fence = fence;
		for (i = 0; i < a->nbos; i++) {
			struct fg_bo *bo = to_fg_bo(objs[i]);

			WRITE_ONCE(bo->last_fence, fence);
			WRITE_ONCE(bo->fenced, true);
		}
	}
out:
	kvfree(cmds);
	kfree(al.handle);
	kfree(al.kind);
	if (objs) {
		for (i = 0; i < a->nbos; i++)
			drm_gem_object_put(objs[i]);
		kvfree(objs);
	}
	return r;
}

int fpgagpu_ioctl_wait(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct fpgagpu_device *fdev = to_fpgagpu(drm);
	struct drm_fpgagpu_wait *a = data;
	long tmo;
	u32 fence;

	if (a->pad)
		return -EINVAL;
	if (a->handle) {
		struct drm_gem_object *obj = drm_gem_object_lookup(file, a->handle);
		struct fg_bo *bo = obj ? to_fg_bo(obj) : NULL;
		bool fenced;

		if (!bo) {
			if (obj)
				drm_gem_object_put(obj);
			return -EINVAL;
		}
		fenced = READ_ONCE(bo->fenced);
		fence = READ_ONCE(bo->last_fence);
		drm_gem_object_put(obj);
		if (!fenced)
			return 0;
	} else {
		fence = (u32)a->fence;
	}
	if (fg_net_fence_passed(&fdev->net, fence))
		return 0;
	tmo = a->timeout_ns > 0 ? (long)min_t(u64, nsecs_to_jiffies(a->timeout_ns), MAX_SCHEDULE_TIMEOUT - 1) : 0;
	if (!tmo)
		return -EBUSY;
	return fg_net_wait_fence(&fdev->net, fence, tmo);
}

/* ------------------------------------------------------------------ READ */

static u32 fg_tformat_bpp(u8 tf, u8 storage)
{
	switch (tf) {
	case 0x00:				/* raw: the storage bytes */
		return fg_format_bpp(storage);
	case 0x04:				/* RGB565 */
		return 2;
	case 0x0A:				/* RGB888 */
	case 0x0B:				/* BGR888 */
		return 3;
	default:
		return 4;
	}
}

struct fg_read_req {
	struct fg_bo *bo;
	bool image;
	u64 offset;
	u16 x, y, w, h;
	u8 level, layer, tf;
	u32 first, bpp;
};

/* ask for bytes [s, s + n) of the read */
static int fg_read_request(struct fpgagpu_device *fdev, struct fg_read *rd, const struct fg_read_req *q, u32 s, u32 n)
{
	u8 b[32];
	u32 tag;

	if (rd->ntags >= FG_RD_TAGS)
		return -EIO;
	tag = ++fdev->read_tag ? fdev->read_tag : ++fdev->read_tag;
	mutex_lock(&fdev->net.lock);
	rd->tags[rd->ntags].tag = tag;
	rd->tags[rd->ntags].start = s;
	rd->ntags++;
	mutex_unlock(&fdev->net.lock);
	if (!q->image) {
		put_unaligned_le16(q->bo->board_handle, b);
		put_unaligned_le16(0, b + 2);
		put_unaligned_le32((u32)(q->offset + s), b + 4);
		put_unaligned_le32(n, b + 8);
		put_unaligned_le32(tag, b + 12);
		return fg_net_send_seq(&fdev->net, T_READ_BUFFER, b, 16, NULL, SEND_TIMEOUT_MS);
	}
	put_unaligned_le16(q->bo->board_handle, b);
	b[2] = q->level;
	b[3] = q->layer;
	b[4] = q->tf;
	b[5] = 0;
	put_unaligned_le16(0, b + 6);
	put_unaligned_le16(q->x, b + 8);
	put_unaligned_le16(q->y, b + 10);
	put_unaligned_le16(q->w, b + 12);
	put_unaligned_le16(q->h, b + 14);
	put_unaligned_le32(q->first + s / q->bpp, b + 16);
	put_unaligned_le32(n / q->bpp, b + 20);
	put_unaligned_le32(tag, b + 24);
	return fg_net_send_seq(&fdev->net, T_READ_IMAGE, b, 28, NULL, SEND_TIMEOUT_MS);
}

static bool fg_read_range_done(struct fg_read *rd, u32 c0, u32 c1)
{
	return find_next_zero_bit(rd->got, c1, c0) >= c1;
}

/* bytes [s, s + n) complete, re-asking for what stays missing after a quiet spell */
static int fg_read_range(struct fpgagpu_device *fdev, struct fg_read *rd, const struct fg_read_req *q, u32 s, u32 n)
{
	u32 c0 = s / FG_RD_CHUNK, c1 = DIV_ROUND_UP(s + n, FG_RD_CHUNK);
	long quiet = msecs_to_jiffies(max_t(s64, 1000, div_s64(4 * fg_net_srtt_ns(&fdev->net), NSEC_PER_MSEC)));
	ktime_t t_data = ktime_get();
	u32 seen = rd->remaining;
	int r = fg_read_request(fdev, rd, q, s, n);

	while (!r && !fg_read_range_done(rd, c0, c1)) {
		long w = wait_event_interruptible_timeout(fdev->net.wq, fg_read_range_done(rd, c0, c1), quiet);
		u32 c;

		if (w < 0)
			return (int)w;
		if (w > 0)
			break;
		if (READ_ONCE(rd->remaining) != seen) {
			seen = READ_ONCE(rd->remaining);
			t_data = ktime_get();
		} else if (ktime_ms_delta(ktime_get(), t_data) > READ_IDLE_MS) {
			return -ETIMEDOUT;
		}
		for (c = c0; c < c1 && !r; ) {			/* the missing runs, each asked for again */
			u32 e;

			c = find_next_zero_bit(rd->got, c1, c);
			if (c >= c1)
				break;
			e = find_next_bit(rd->got, c1, c);
			e = min(e, c + READ_REQ_MAX / FG_RD_CHUNK);
			r = fg_read_request(fdev, rd, q, c * FG_RD_CHUNK,
					    min(e * FG_RD_CHUNK, s + n) - c * FG_RD_CHUNK);
			c = e;
		}
	}
	return r;
}

int fpgagpu_ioctl_read(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct fpgagpu_device *fdev = to_fpgagpu(drm);
	struct drm_fpgagpu_read *a = data;
	struct drm_gem_object *obj;
	struct fg_read_req q = { 0 };
	struct fg_read *rd = NULL;
	u32 s;
	int r, idx;

	if (a->flags || a->pad || a->pad2 || !a->size || a->size > READ_MAX)
		return -EINVAL;
	obj = drm_gem_object_lookup(file, a->handle);
	if (!obj)
		return -ENOENT;
	q.bo = to_fg_bo(obj);
	if (!q.bo) {
		r = -EINVAL;
		goto put;
	}
	q.image = q.bo->kind == FPGAGPU_BO_B_IMAGE;
	if (q.image) {
		q.x = a->x;
		q.y = a->y;
		q.w = a->w;
		q.h = a->h;
		q.level = a->level;
		q.layer = a->layer;
		q.tf = a->tformat;
		q.first = a->first;
		q.bpp = fg_tformat_bpp(a->tformat, q.bo->format);
		if (!q.bpp || !q.w || !q.h || a->size % q.bpp ||
		    (u64)a->first + a->size / q.bpp > (u64)q.w * q.h) {
			r = -EINVAL;
			goto put;
		}
	} else {
		q.offset = a->offset;
		if (a->offset > q.bo->size || a->size > q.bo->size - a->offset) {
			r = -EINVAL;
			goto put;
		}
	}
	rd = kzalloc(sizeof(*rd), GFP_KERNEL);
	if (!rd) {
		r = -ENOMEM;
		goto put;
	}
	rd->total = a->size;
	rd->rel = !q.image;
	rd->base = q.first * q.bpp;
	rd->nchunks = DIV_ROUND_UP(rd->total, FG_RD_CHUNK);
	rd->remaining = rd->nchunks;
	rd->buf = kvmalloc(rd->total, GFP_KERNEL);
	rd->got = bitmap_zalloc(rd->nchunks, GFP_KERNEL);
	if (!rd->buf || !rd->got) {
		r = -ENOMEM;
		goto free;
	}
	if (!drm_dev_enter(drm, &idx)) {
		r = -ENODEV;
		goto free;
	}
	mutex_lock(&fdev->submit_lock);
	fg_net_read_attach(&fdev->net, rd);
	for (s = 0, r = 0; s < rd->total && !r; s += READ_REQ_MAX)
		r = fg_read_range(fdev, rd, &q, s, min(READ_REQ_MAX, rd->total - s));
	fg_net_read_attach(&fdev->net, NULL);
	mutex_unlock(&fdev->submit_lock);
	drm_dev_exit(idx);
	if (!r && copy_to_user(u64_to_user_ptr(a->data), rd->buf, rd->total))
		r = -EFAULT;
free:
	if (rd) {
		kvfree(rd->buf);
		bitmap_free(rd->got);
		kfree(rd);
	}
put:
	drm_gem_object_put(obj);
	return r;
}
