// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * fpgagpu_kms: board B's HDMI output as a KMS display.
 *
 * One CRTC, one primary plane, one encoder and one HDMI-A connector, always connected, with board B's five video
 * modes. A framebuffer in host memory (shmem GEM: dumb buffers, the fbdev console) is copied: a plane update sends the
 * damaged rectangle with WRITE_IMAGE into one board-B image, which board B scans out from its DDR every frame, so
 * an update shows on the next frame without another PRESENT. The image is (re)created and PRESENTed when the
 * framebuffer's size or format changes; a modeset sends SET_MODE first.
 *
 * A framebuffer over a board-B image BO (what Mesa renders into, through GBM) is shown as it is: the plane update
 * PRESENTs that image, nothing is copied. The commands that drew it went to the boards before, in the same stream,
 * so board B has rendered it by the time it takes the PRESENT. Such an image must be the mode's size.
 *
 * Over Gigabit Ethernet a whole 800 x 600 RGBA8 frame is 1.9 MB (about 55 frames a second at best); through the
 * relay to a remote host it takes seconds. Consoles and desktops mostly update small rectangles.
 *
 * The cursor plane is board B's hardware cursor: a 64 x 64 premultiplied ARGB shape over the scanout. A new cursor
 * framebuffer sends its shape (sequenced, after what was drawn before); a move sends only the position on the
 * unsequenced fast path, which board A passes to board B ahead of every queued command, so the pointer does not wait
 * behind rendering. A fast-path message is not resent: the last position goes again a while after the last move.
 */
#include <linux/slab.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modes.h>
#include <drm/drm_print.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "fpgagpu.h"
#include "fpgagpu_drm.h"

/* protocol: board B's images and display (host/fb/fbproto.py) */
#define T_CREATE_IMAGE		0x11
#define T_DESTROY		0x12
#define T_WRITE_IMAGE		0x22
#define T_SET_MODE		0x30
#define T_PRESENT		0x31
#define T_CURSOR		0x32
#define T_CURSOR_POS		0x33		/* unsequenced: the cursor's position, ahead of the queue */
#define CUR_SHAPE		1
#define CUR_SHOW		2
#define CUR_HIDE		3
#define CUR_PREMUL		2		/* CUR_SHOW mode: premultiplied 0xAARRGGBB words */
#define CUR_DIM			64
#define CUR_WORDS		340		/* shape words per CURSOR datagram */
#define CUR_HEAL_MS		50
#define FMT_RGBA8		0x00
#define FMT_RGB565		0x04
#define TF_RGBA8888		0x01
#define TF_BGRX8888		0x03
#define TF_RGB565		0x04
#define USE_SAMPLED		1
#define USE_SCANOUT		4
#define USE_TRANSFER		8
#define MAX_DATA		1440		/* pixel bytes per WRITE_IMAGE datagram */
#define WRITE_HDR		24

/* the scanout image: the start of board B's host heap, room for the largest mode (1280 x 800 x 4 bytes) */
#define SCANOUT_BASE		0x10000000u
#define SCANOUT_INDEX		1000
#define SCANOUT_HANDLE		(0xC000 | SCANOUT_INDEX)	/* bit 15 board B, bit 14 image, bits 9..0 the index */

#define MODE_TIMEOUT_MS		5000
#define PRESENT_TIMEOUT_MS	2000
#define SEND_TIMEOUT_MS		10000

static const struct { u16 w, h; u8 id; } fg_modes[] = {
	{ 640, 480, 0 }, { 800, 600, 1 }, { 1024, 768, 4 }, { 1280, 720, 2 }, { 1280, 800, 3 },
};

/* a copied framebuffer's alpha is not shown; an image BO stores its pixels as board B's RGBA8 or RGB565 whatever the
 * byte order the fourcc names (the driver that rendered it converts on every CPU transfer) */
static const u32 fg_formats[] = {
	DRM_FORMAT_XRGB8888, DRM_FORMAT_ARGB8888, DRM_FORMAT_XBGR8888, DRM_FORMAT_ABGR8888, DRM_FORMAT_RGB565,
};

static const u32 fg_cursor_formats[] = { DRM_FORMAT_ARGB8888 };

static bool fg_fourcc_bgr(u32 fourcc)
{
	return fourcc == DRM_FORMAT_XBGR8888 || fourcc == DRM_FORMAT_ABGR8888;
}

static u8 fg_fourcc_storage(u32 fourcc)
{
	return fourcc == DRM_FORMAT_RGB565 ? FMT_RGB565 : FMT_RGBA8;
}

/* the board-B image a framebuffer is made of, or NULL for one in host memory */
static struct fg_bo *fg_fb_image(const struct drm_framebuffer *fb)
{
	struct fg_bo *bo = fb ? to_fg_bo(fb->obj[0]) : NULL;

	return bo && bo->kind == FPGAGPU_BO_B_IMAGE ? bo : NULL;
}

static int fg_mode_id(int w, int h)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(fg_modes); i++)
		if (fg_modes[i].w == w && fg_modes[i].h == h)
			return fg_modes[i].id;
	return -1;
}

static struct fpgagpu_device *kms_to_fdev(struct drm_device *drm)
{
	return to_fpgagpu(drm);
}

/* ------------------------------------------------------------------ board messages */

static int fg_set_mode(struct fpgagpu_device *fdev, int id)
{
	struct fg_event ev;
	u8 body[4];
	u32 seq;
	int r;

	put_unaligned_le32(id, body);
	r = fg_net_send_seq(&fdev->net, T_SET_MODE, body, sizeof(body), &seq, SEND_TIMEOUT_MS);
	if (r)
		return r;
	/* inside an atomic commit, which cannot be restarted: a signal must not end the wait */
	r = fg_net_wait_event_uninterruptible(&fdev->net, seq, 0, &ev, MODE_TIMEOUT_MS);
	if (!r && (ev.code != FG_EV_MODE || ev.a0 != id))
		r = -EIO;
	return r;
}

static int fg_create_scanout(struct fpgagpu_device *fdev, u32 fourcc, u32 w, u32 h)
{
	u8 fmt = fg_fourcc_storage(fourcc);
	u32 pitch = w * (fmt == FMT_RGB565 ? 2 : 4);
	u8 b[20];

	if (fdev->kms.img_valid) {			/* the old descriptor goes first */
		u8 d[4];

		put_unaligned_le16(SCANOUT_HANDLE, d);
		put_unaligned_le16(0, d + 2);
		fg_net_send_seq(&fdev->net, T_DESTROY, d, sizeof(d), NULL, SEND_TIMEOUT_MS);
		fdev->kms.img_valid = false;
	}
	put_unaligned_le16(SCANOUT_HANDLE, b);
	put_unaligned_le16(USE_SAMPLED | USE_SCANOUT | USE_TRANSFER, b + 2);
	put_unaligned_le32(SCANOUT_BASE, b + 4);
	put_unaligned_le16(pitch, b + 8);
	b[10] = fmt;
	b[11] = 0;					/* origin top-left, one sample */
	put_unaligned_le16(w, b + 12);
	put_unaligned_le16(h, b + 14);
	b[16] = 1;					/* levels */
	b[17] = 1;					/* layers */
	put_unaligned_le16(0, b + 18);			/* not a 3D texture */
	if (fg_net_send_seq(&fdev->net, T_CREATE_IMAGE, b, sizeof(b), NULL, SEND_TIMEOUT_MS))
		return -EIO;
	fdev->kms.img_valid = true;
	if (fdev->kms.shown == SCANOUT_HANDLE)
		fdev->kms.shown = 0;			/* a new descriptor: board B must be told again */
	fdev->kms.img_fourcc = fourcc;
	fdev->kms.img_w = w;
	fdev->kms.img_h = h;
	return 0;
}

/* board B shows `handle` from its next frame on. wait: until it answers that it has (one round trip) */
static void fg_present(struct fpgagpu_device *fdev, u16 handle, bool wait)
{
	struct fg_event ev;
	u8 b[8];
	u32 seq;

	put_unaligned_le16(handle, b);
	put_unaligned_le16(0, b + 2);
	put_unaligned_le32(++fdev->kms.frame_id, b + 4);
	if (fg_net_send_seq(&fdev->net, T_PRESENT, b, sizeof(b), &seq, SEND_TIMEOUT_MS)) {
		drm_warn(&fdev->drm, "PRESENT of image 0x%04x was not sent\n", handle);
		return;
	}
	fdev->kms.shown = handle;
	if (wait && (fg_net_wait_event_uninterruptible(&fdev->net, seq, 0, &ev, PRESENT_TIMEOUT_MS) ||
		     ev.code != FG_EV_PRESENT))
		drm_warn(&fdev->drm, "PRESENT of image 0x%04x got no PRESENT event\n", handle);
}

/* the plane lets go of an image BO, which may be destroyed next: board B goes back to the scanout image (created at
 * the mode's size if there is none, showing whatever its memory holds until the next copied framebuffer) */
static void fg_unshow_bo(struct fpgagpu_device *fdev)
{
	u16 s = fdev->kms.shown;
	unsigned int i;

	if (!s || s == SCANOUT_HANDLE)
		return;
	if (!fdev->kms.img_valid) {
		for (i = 0; i < ARRAY_SIZE(fg_modes); i++)
			if (fg_modes[i].id == fdev->kms.mode_id)
				break;
		if (i == ARRAY_SIZE(fg_modes) ||
		    fg_create_scanout(fdev, DRM_FORMAT_XRGB8888, fg_modes[i].w, fg_modes[i].h))
			return;
	}
	fg_present(fdev, SCANOUT_HANDLE, false);
}

/* the rectangle r of the framebuffer (system memory at vaddr) into the scanout image, row-major, whole pixels per
 * datagram (fbproto.Board.write_image) */
static int fg_upload(struct fpgagpu_device *fdev, struct drm_framebuffer *fb, const u8 *vaddr, const struct drm_rect *r)
{
	u32 bpp = fb->format->cpp[0];
	u32 fourcc = fb->format->format;
	u8 tf = fourcc == DRM_FORMAT_RGB565 ? TF_RGB565 : fg_fourcc_bgr(fourcc) ? TF_RGBA8888 : TF_BGRX8888;
	u32 w = drm_rect_width(r), h = drm_rect_height(r);
	u32 total = w * h, per = MAX_DATA / bpp, p;
	u8 *d = fdev->kms.upbuf;
	int ret = 0;

	for (p = 0; p < total && !ret; p += per) {
		u32 n = min(per, total - p), k = 0;

		put_unaligned_le16(SCANOUT_HANDLE, d);
		d[2] = 0;				/* level */
		d[3] = 0;				/* layer */
		d[4] = tf;
		d[5] = 0;
		put_unaligned_le16(0, d + 6);
		put_unaligned_le16(r->x1, d + 8);
		put_unaligned_le16(r->y1, d + 10);
		put_unaligned_le16(w, d + 12);
		put_unaligned_le16(h, d + 14);
		put_unaligned_le32(p, d + 16);		/* first pixel of this datagram, in the rectangle */
		put_unaligned_le32(0, d + 20);
		while (k < n) {				/* the pixels p + k .. may span rows of the rectangle */
			u32 q = p + k, row = q / w, col = q % w, run = min(n - k, w - col);
			const u8 *src = vaddr + fb->offsets[0] + (r->y1 + row) * fb->pitches[0] + (r->x1 + col) * bpp;

			memcpy(d + WRITE_HDR + k * bpp, src, run * bpp);
			k += run;
		}
		ret = fg_net_send_seq(&fdev->net, T_WRITE_IMAGE, d, WRITE_HDR + n * bpp, NULL, SEND_TIMEOUT_MS);
	}
	return ret;
}

/* ------------------------------------------------------------------ plane */

static int fg_plane_atomic_check(struct drm_plane *plane, fg_atomic_t *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *cs = NULL;
	struct drm_framebuffer *fb = ps->fb;
	struct fg_bo *bo;
	int r;

	if (ps->crtc)
		cs = drm_atomic_get_new_crtc_state(state, ps->crtc);
	r = drm_atomic_helper_check_plane_state(ps, cs, 1 << 16, 1 << 16, false, false);	/* no scaling */
	if (r || !ps->visible)
		return r;
	bo = fg_fb_image(fb);
	if (bo && (bo->format != fg_fourcc_storage(fb->format->format) || fb->offsets[0] || fb->pitches[0] != bo->pitch ||
		   fb->width != bo->width || fb->height != bo->height ||
		   fb->width != cs->mode.hdisplay || fb->height != cs->mode.vdisplay))
		return -EINVAL;				/* board B scans an image out whole, from its first pixel */
	return 0;
}

static void fg_plane_atomic_update(struct drm_plane *plane, fg_atomic_t *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_plane_state *old = drm_atomic_get_old_plane_state(state, plane);
	struct drm_shadow_plane_state *sh = to_drm_shadow_plane_state(ps);
	struct fpgagpu_device *fdev = kms_to_fdev(plane->dev);
	struct drm_framebuffer *fb = ps->fb;
	struct fg_bo *bo = fg_fb_image(fb);
	struct drm_rect r;
	bool whole = false;
	int idx;

	if (!fb || !ps->visible)
		return;
	if (!drm_dev_enter(plane->dev, &idx))
		return;
	mutex_lock(&fdev->submit_lock);			/* not in the middle of a SUBMIT's frame */
	if (bo) {
		/* a flip: no wait for board B's answer, which comes with its next frame */
		if (fdev->kms.shown != bo->board_handle || fdev->kms.need_present)
			fg_present(fdev, bo->board_handle, false);
		fdev->kms.need_present = false;
		goto out;
	}
	if (!fdev->kms.img_valid || fdev->kms.img_fourcc != fb->format->format ||
	    fdev->kms.img_w != fb->width || fdev->kms.img_h != fb->height) {
		if (fg_create_scanout(fdev, fb->format->format, fb->width, fb->height)) {
			drm_warn(plane->dev, "CREATE_IMAGE of the %ux%u scanout image failed\n", fb->width, fb->height);
			goto out;
		}
		whole = true;
	}
	if (fdev->kms.shown != SCANOUT_HANDLE)		/* an image BO was shown: the copy may be behind this framebuffer */
		fdev->kms.need_present = true;
	if (whole || fdev->kms.need_present) {
		r.x1 = 0;
		r.y1 = 0;
		r.x2 = fb->width;
		r.y2 = fb->height;
	} else if (!drm_atomic_helper_damage_merged(old, ps, &r)) {
		goto out;
	}
	if (fg_upload(fdev, fb, sh->data[0].vaddr, &r))
		dev_warn_ratelimited(plane->dev->dev, "WRITE_IMAGE of the damage failed\n");
	if (whole || fdev->kms.need_present) {
		fg_present(fdev, SCANOUT_HANDLE, true);
		fdev->kms.need_present = false;
	}
out:
	mutex_unlock(&fdev->submit_lock);
	drm_dev_exit(idx);
}

static void fg_plane_atomic_disable(struct drm_plane *plane, fg_atomic_t *state)
{
	struct fpgagpu_device *fdev = kms_to_fdev(plane->dev);
	int idx;

	if (!drm_dev_enter(plane->dev, &idx))
		return;
	mutex_lock(&fdev->submit_lock);
	fg_unshow_bo(fdev);
	mutex_unlock(&fdev->submit_lock);
	drm_dev_exit(idx);
}

/* the CPU maps a copied framebuffer for the update; an image BO has nothing for it to map (Kbuild sets
 * FG_HAVE_BEGIN_FB_ACCESS where the shadow-plane helpers map in begin_fb_access instead of prepare_fb) */
#ifdef FG_HAVE_BEGIN_FB_ACCESS
static int fg_plane_begin_fb_access(struct drm_plane *plane, struct drm_plane_state *ps)
{
	return fg_fb_image(ps->fb) ? 0 : drm_gem_begin_shadow_fb_access(plane, ps);
}

static void fg_plane_end_fb_access(struct drm_plane *plane, struct drm_plane_state *ps)
{
	if (!fg_fb_image(ps->fb))
		drm_gem_end_shadow_fb_access(plane, ps);
}
#define FG_PLANE_FB_ACCESS .begin_fb_access = fg_plane_begin_fb_access, .end_fb_access = fg_plane_end_fb_access
#else
static int fg_plane_prepare_fb(struct drm_plane *plane, struct drm_plane_state *ps)
{
	return fg_fb_image(ps->fb) ? 0 : drm_gem_prepare_shadow_fb(plane, ps);
}

static void fg_plane_cleanup_fb(struct drm_plane *plane, struct drm_plane_state *ps)
{
	if (!fg_fb_image(ps->fb))
		drm_gem_cleanup_shadow_fb(plane, ps);
}
#define FG_PLANE_FB_ACCESS .prepare_fb = fg_plane_prepare_fb, .cleanup_fb = fg_plane_cleanup_fb
#endif

static const struct drm_plane_helper_funcs fg_plane_helper_funcs = {
	FG_PLANE_FB_ACCESS,
	.atomic_check	= fg_plane_atomic_check,
	.atomic_update	= fg_plane_atomic_update,
	.atomic_disable	= fg_plane_atomic_disable,
};

static const struct drm_plane_funcs fg_plane_funcs = {
	.update_plane	= drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.destroy	= drm_plane_cleanup,
	DRM_GEM_SHADOW_PLANE_FUNCS,
};

/* ------------------------------------------------------------------ cursor plane */

/* the shape: 64 x 64 words, the framebuffer's pixels from the top-left corner, transparent past them; called with
 * submit_lock held (upbuf) */
static int fg_cursor_shape(struct fpgagpu_device *fdev, const struct drm_framebuffer *fb, const u8 *vaddr)
{
	u32 w = min_t(u32, fb->width, CUR_DIM), h = min_t(u32, fb->height, CUR_DIM);
	u8 *d = fdev->kms.upbuf;
	u32 k, i;
	int r;

	for (k = 0; k < CUR_DIM * CUR_DIM; k += CUR_WORDS) {
		u32 n = min_t(u32, CUR_WORDS, CUR_DIM * CUR_DIM - k);

		d[0] = CUR_SHAPE;
		d[1] = 0;
		put_unaligned_le16(0, d + 2);
		put_unaligned_le16(k, d + 4);		/* the first word's index */
		put_unaligned_le16(0, d + 6);
		for (i = 0; i < n; i++) {
			u32 x = (k + i) % CUR_DIM, y = (k + i) / CUR_DIM, v = 0;

			if (x < w && y < h)
				v = get_unaligned_le32(vaddr + fb->offsets[0] + y * fb->pitches[0] + x * 4);
			put_unaligned_le32(v, d + 8 + 4 * i);
		}
		r = fg_net_send_seq(&fdev->net, T_CURSOR, d, 8 + 4 * n, NULL, SEND_TIMEOUT_MS);
		if (r)
			return r;
	}
	return 0;
}

/* show the shape just sent, its top-left corner at (x, y) */
static int fg_cursor_show(struct fpgagpu_device *fdev, s16 x, s16 y)
{
	u8 b[12];

	b[0] = CUR_SHOW;
	b[1] = CUR_PREMUL;
	b[2] = 1 | 2;					/* take the new shape, at this position */
	b[3] = 0;
	b[4] = 0;					/* hot spot: the position is the corner already */
	b[5] = 0;
	put_unaligned_le16(0, b + 6);
	put_unaligned_le16((u16)x, b + 8);
	put_unaligned_le16((u16)y, b + 10);
	return fg_net_send_seq(&fdev->net, T_CURSOR, b, sizeof(b), NULL, SEND_TIMEOUT_MS);
}

static void fg_cursor_pos(struct fpgagpu_device *fdev, s16 x, s16 y)
{
	u8 b[4];

	put_unaligned_le16((u16)x, b);
	put_unaligned_le16((u16)y, b + 2);
	fg_net_send_unseq(&fdev->net, T_CURSOR_POS, b, sizeof(b));
}

static void fg_cursor_hide(struct fpgagpu_device *fdev)
{
	u8 b[4] = { CUR_HIDE, 0, 0, 0 };

	if (!fdev->kms.cur_shown)
		return;
	mutex_lock(&fdev->submit_lock);
	fg_net_send_seq(&fdev->net, T_CURSOR, b, sizeof(b), NULL, SEND_TIMEOUT_MS);
	mutex_unlock(&fdev->submit_lock);
	WRITE_ONCE(fdev->kms.cur_shown, false);
	cancel_delayed_work(&fdev->kms.cur_work);
}

/* the last position once more: a fast-path message lost on the way is healed when the pointer rests */
static void fg_cursor_heal(struct work_struct *w)
{
	struct fg_kms *k = container_of(to_delayed_work(w), struct fg_kms, cur_work);
	struct fpgagpu_device *fdev = container_of(k, struct fpgagpu_device, kms);
	int idx;

	if (!drm_dev_enter(&fdev->drm, &idx))
		return;
	if (READ_ONCE(k->cur_shown))
		fg_cursor_pos(fdev, READ_ONCE(k->cur_x), READ_ONCE(k->cur_y));
	drm_dev_exit(idx);
}

static int fg_cursor_atomic_check(struct drm_plane *plane, fg_atomic_t *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *cs = NULL;
	int r;

	if (ps->crtc)
		cs = drm_atomic_get_new_crtc_state(state, ps->crtc);
	r = drm_atomic_helper_check_plane_state(ps, cs, 1 << 16, 1 << 16, true, true);
	if (r || !ps->visible)
		return r;
	if (fg_fb_image(ps->fb) || ps->fb->width > CUR_DIM || ps->fb->height > CUR_DIM)
		return -EINVAL;				/* a shape in host memory, at most 64 x 64 */
	return 0;
}

static void fg_cursor_atomic_update(struct drm_plane *plane, fg_atomic_t *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_plane_state *old = drm_atomic_get_old_plane_state(state, plane);
	struct drm_shadow_plane_state *sh = to_drm_shadow_plane_state(ps);
	struct fpgagpu_device *fdev = kms_to_fdev(plane->dev);
	struct fg_kms *k = &fdev->kms;
	s16 x = clamp_t(int, ps->crtc_x, S16_MIN, S16_MAX), y = clamp_t(int, ps->crtc_y, S16_MIN, S16_MAX);
	int idx;

	if (!drm_dev_enter(plane->dev, &idx))
		return;
	if (!ps->fb || !ps->visible) {
		fg_cursor_hide(fdev);
	} else if (ps->fb != old->fb || !k->cur_shown) {
		mutex_lock(&fdev->submit_lock);
		if (!fg_cursor_shape(fdev, ps->fb, sh->data[0].vaddr) && !fg_cursor_show(fdev, x, y))
			WRITE_ONCE(k->cur_shown, true);
		else
			dev_warn_ratelimited(plane->dev->dev, "the cursor's shape was not sent\n");
		mutex_unlock(&fdev->submit_lock);
	} else if (x != k->cur_x || y != k->cur_y) {
		fg_cursor_pos(fdev, x, y);
		mod_delayed_work(system_wq, &k->cur_work, msecs_to_jiffies(CUR_HEAL_MS));
	}
	WRITE_ONCE(k->cur_x, x);
	WRITE_ONCE(k->cur_y, y);
	drm_dev_exit(idx);
}

static void fg_cursor_atomic_disable(struct drm_plane *plane, fg_atomic_t *state)
{
	struct fpgagpu_device *fdev = kms_to_fdev(plane->dev);
	int idx;

	if (!drm_dev_enter(plane->dev, &idx))
		return;
	fg_cursor_hide(fdev);
	drm_dev_exit(idx);
}

static const struct drm_plane_helper_funcs fg_cursor_helper_funcs = {
	FG_PLANE_FB_ACCESS,
	.atomic_check	= fg_cursor_atomic_check,
	.atomic_update	= fg_cursor_atomic_update,
	.atomic_disable	= fg_cursor_atomic_disable,
};

/* ------------------------------------------------------------------ CRTC */

static int fg_crtc_atomic_check(struct drm_crtc *crtc, fg_atomic_t *state)
{
	struct drm_crtc_state *cs = drm_atomic_get_new_crtc_state(state, crtc);

	if (cs->enable && fg_mode_id(cs->mode.hdisplay, cs->mode.vdisplay) < 0)
		return -EINVAL;				/* board B has no such mode */
	return 0;
}

static void fg_crtc_atomic_enable(struct drm_crtc *crtc, fg_atomic_t *state)
{
	struct drm_crtc_state *cs = drm_atomic_get_new_crtc_state(state, crtc);
	struct fpgagpu_device *fdev = kms_to_fdev(crtc->dev);
	int id = fg_mode_id(cs->mode.hdisplay, cs->mode.vdisplay);
	int idx, r;

	if (id < 0 || !drm_dev_enter(crtc->dev, &idx))
		return;
	mutex_lock(&fdev->submit_lock);
	r = fg_set_mode(fdev, id);
	mutex_unlock(&fdev->submit_lock);
	if (r)
		drm_warn(crtc->dev, "SET_MODE %d (%dx%d) failed: %d\n", id, cs->mode.hdisplay, cs->mode.vdisplay, r);
	fdev->kms.mode_id = id;
	fdev->kms.need_present = true;			/* the plane update after this shows the image again */
	drm_dev_exit(idx);
}

static void fg_crtc_atomic_disable(struct drm_crtc *crtc, fg_atomic_t *state)
{
	struct fpgagpu_device *fdev = kms_to_fdev(crtc->dev);
	int idx;

	if (!drm_dev_enter(crtc->dev, &idx))
		return;
	mutex_lock(&fdev->submit_lock);
	fg_unshow_bo(fdev);
	mutex_unlock(&fdev->submit_lock);
	drm_dev_exit(idx);
}

/* the boards send no vblank interrupt: a flip completes once its upload is sent */
static void fg_crtc_atomic_flush(struct drm_crtc *crtc, fg_atomic_t *state)
{
	struct drm_pending_vblank_event *event = crtc->state->event;

	if (event) {
		crtc->state->event = NULL;
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static const struct drm_crtc_helper_funcs fg_crtc_helper_funcs = {
	.atomic_check	= fg_crtc_atomic_check,
	.atomic_enable	= fg_crtc_atomic_enable,
	.atomic_disable	= fg_crtc_atomic_disable,
	.atomic_flush	= fg_crtc_atomic_flush,
};

static const struct drm_crtc_funcs fg_crtc_funcs = {
	.reset			= drm_atomic_helper_crtc_reset,
	.destroy		= drm_crtc_cleanup,
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
};

/* ------------------------------------------------------------------ encoder, connector */

static const struct drm_encoder_funcs fg_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int fg_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *drm = connector->dev;
	unsigned int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(fg_modes); i++) {
		struct drm_display_mode *m = drm_mode_find_dmt(drm, fg_modes[i].w, fg_modes[i].h, 60, false);

		if (!m)
			m = drm_cvt_mode(drm, fg_modes[i].w, fg_modes[i].h, 60, false, false, false);
		if (!m)
			continue;
		if (fg_modes[i].w == 800)
			m->type |= DRM_MODE_TYPE_PREFERRED;
		drm_mode_probed_add(connector, m);
		n++;
	}
	return n;
}

static const struct drm_connector_helper_funcs fg_connector_helper_funcs = {
	.get_modes = fg_connector_get_modes,
};

static const struct drm_connector_funcs fg_connector_funcs = {
	.reset			= drm_atomic_helper_connector_reset,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= drm_connector_cleanup,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
};

/* ------------------------------------------------------------------ mode config */

static const struct drm_mode_config_funcs fg_mode_config_funcs = {
	.fb_create	= drm_gem_fb_create_with_dirty,
	.atomic_check	= drm_atomic_helper_check,
	.atomic_commit	= drm_atomic_helper_commit,
};

/* modeset enables before the planes, so SET_MODE goes ahead of the image the plane presents */
static const struct drm_mode_config_helper_funcs fg_mode_config_helper_funcs = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

int fg_kms_init(struct fpgagpu_device *fdev)
{
	struct drm_device *drm = &fdev->drm;
	struct fg_kms *k = &fdev->kms;
	int r;

	k->upbuf = drmm_kmalloc(drm, WRITE_HDR + MAX_DATA, GFP_KERNEL);
	if (!k->upbuf)
		return -ENOMEM;
	r = drmm_mode_config_init(drm);
	if (r)
		return r;
	/* the limits of every framebuffer, the cursor's 64 x 64 one included; the primary plane's check keeps its
	 * framebuffer covering the mode */
	drm->mode_config.min_width = 1;
	drm->mode_config.min_height = 1;
	drm->mode_config.max_width = 1280;
	drm->mode_config.max_height = 800;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.cursor_width = CUR_DIM;
	drm->mode_config.cursor_height = CUR_DIM;
	drm->mode_config.funcs = &fg_mode_config_funcs;
	drm->mode_config.helper_private = &fg_mode_config_helper_funcs;
	INIT_DELAYED_WORK(&k->cur_work, fg_cursor_heal);
	k->cur_work_ready = true;

	r = drm_universal_plane_init(drm, &k->plane, 0, &fg_plane_funcs, fg_formats, ARRAY_SIZE(fg_formats),
				     NULL, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (r)
		return r;
	drm_plane_helper_add(&k->plane, &fg_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(&k->plane);

	r = drm_universal_plane_init(drm, &k->cursor, 0, &fg_plane_funcs, fg_cursor_formats,
				     ARRAY_SIZE(fg_cursor_formats), NULL, DRM_PLANE_TYPE_CURSOR, NULL);
	if (r)
		return r;
	drm_plane_helper_add(&k->cursor, &fg_cursor_helper_funcs);

	r = drm_crtc_init_with_planes(drm, &k->crtc, &k->plane, &k->cursor, &fg_crtc_funcs, NULL);
	if (r)
		return r;
	drm_crtc_helper_add(&k->crtc, &fg_crtc_helper_funcs);

	r = drm_encoder_init(drm, &k->encoder, &fg_encoder_funcs, DRM_MODE_ENCODER_TMDS, NULL);
	if (r)
		return r;
	k->encoder.possible_crtcs = drm_crtc_mask(&k->crtc);

	r = drm_connector_init(drm, &k->connector, &fg_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);
	if (r)
		return r;
	drm_connector_helper_add(&k->connector, &fg_connector_helper_funcs);
	r = drm_connector_attach_encoder(&k->connector, &k->encoder);
	if (r)
		return r;

	drm_mode_config_reset(drm);
	return 0;
}

/* before the transport goes: nothing may send the cursor's position after it */
void fg_kms_fini(struct fpgagpu_device *fdev)
{
	if (fdev->kms.cur_work_ready)
		cancel_delayed_work_sync(&fdev->kms.cur_work);
}
