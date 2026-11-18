// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2026 Dmytro Ivanysh */
/*
 * fpgagpu_gem: buffer objects in the boards' memory. A board BO is a range in a board heap plus a board handle; the
 * CPU cannot map it. BO_CREATE sends CREATE_BUFFER / CREATE_IMAGE; the last reference going away queues the object, and
 * a worker sends DESTROY and returns its range and handle (the free callback may run where sending would block).
 */
#include <linux/slab.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#include <drm/drm_drv.h>
#include <drm/drm_file.h>

#include "fpgagpu.h"
#include "fpgagpu_drm.h"

#define T_CREATE_BUFFER	0x10
#define T_CREATE_IMAGE	0x11
#define T_DESTROY	0x12
#define SEND_TIMEOUT_MS	10000
#define BUFFER_MAX	(256u << 20)
#define BAND_ROWS	8		/* the most rows of a band board B renders (b_rop's NLMAX) */

static void fg_bo_gem_free(struct drm_gem_object *obj)
{
	struct fg_bo *bo = container_of(obj, struct fg_bo, base);
	struct fpgagpu_device *fdev = to_fpgagpu(obj->dev);

	spin_lock(&fdev->free_lock);
	list_add_tail(&bo->free_link, &fdev->bo_free);
	spin_unlock(&fdev->free_lock);
	queue_work(fg_wq, &fdev->free_work);
}

const struct drm_gem_object_funcs fg_bo_funcs = {
	.free = fg_bo_gem_free,
};

void fg_bo_free_work(struct work_struct *work)
{
	struct fpgagpu_device *fdev = container_of(work, struct fpgagpu_device, free_work);

	for (;;) {
		struct fg_bo *bo = NULL;
		int idx;

		spin_lock(&fdev->free_lock);
		if (!list_empty(&fdev->bo_free)) {
			bo = list_first_entry(&fdev->bo_free, struct fg_bo, free_link);
			list_del(&bo->free_link);
		}
		spin_unlock(&fdev->free_lock);
		if (!bo)
			break;
		if (bo->created) {
			mutex_lock(&fdev->bo_lock);
			list_del(&bo->bo_link);
			mutex_unlock(&fdev->bo_lock);
		}
		if (bo->created && drm_dev_enter(&fdev->drm, &idx)) {	/* no messages once the device is unplugged */
			u8 d[4];

			put_unaligned_le16(bo->board_handle, d);
			put_unaligned_le16(0, d + 2);
			mutex_lock(&fdev->submit_lock);
			fg_net_send_seq(&fdev->net, T_DESTROY, d, sizeof(d), NULL, SEND_TIMEOUT_MS);
			mutex_unlock(&fdev->submit_lock);
			drm_dev_exit(idx);
		}
		fg_mem_free(fdev, bo);
		if (bo->alias_of)			/* the image whose memory it described may go now */
			drm_gem_object_put(&bo->alias_of->base);
		drm_gem_object_release(&bo->base);
		kfree(bo);
	}
}

/* the boards in `lost` restarted and their tables are empty: the descriptor of every BO on them again, in the order
 * they were made (an alias after its image). What the memory held is gone. Called with submit_lock held */
void fg_bo_recreate(struct fpgagpu_device *fdev, unsigned int lost, unsigned int *done, unsigned int *failed)
{
	struct fg_bo *bo;

	*done = 0;
	*failed = 0;
	mutex_lock(&fdev->bo_lock);
	list_for_each_entry(bo, &fdev->bo_list, bo_link) {
		if (!(lost & (bo->kind == FPGAGPU_BO_A_BUFFER ? FG_LOST_A : FG_LOST_B)))
			continue;
		if (fg_net_send_seq(&fdev->net, bo->create_type, bo->create_body, bo->create_len, NULL, SEND_TIMEOUT_MS))
			(*failed)++;
		else
			(*done)++;
	}
	mutex_unlock(&fdev->bo_lock);
}

int fpgagpu_ioctl_bo_create(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct fpgagpu_device *fdev = to_fpgagpu(drm);
	struct drm_fpgagpu_bo_create *a = data;
	struct fg_bo *bo;
	u8 body[20];
	struct fg_bo *orig = NULL;
	u32 blen;
	u8 type;
	u64 size, addr;
	int r, idx;

	if ((a->flags & ~FPGAGPU_BO_CREATE_ALIAS) || a->pad || (a->alias_offset && !(a->flags & FPGAGPU_BO_CREATE_ALIAS)))
		return -EINVAL;
	switch (a->kind) {
	case FPGAGPU_BO_A_BUFFER:
	case FPGAGPU_BO_B_BUFFER:
		if (!a->size || a->size > BUFFER_MAX)
			return -EINVAL;
		size = a->size;
		break;
	case FPGAGPU_BO_B_IMAGE:
		if (a->width > 2048 || a->height > 2048 || a->vol > 1 || (a->img_flags & ~0x19))
			return -EINVAL;
		size = fg_image_bytes(a->format, a->width, a->height, a->pitch, a->levels, a->layers, a->vol,
				      (a->img_flags >> 3) & 3);
		if (!size)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	if (a->flags & FPGAGPU_BO_CREATE_ALIAS) {
		/* another descriptor of an image's bytes: no memory of its own, at most the image's size. One at an offset is
		 * a rectangle (one level, one layer): its last row ends at offset + (height - 1) x pitch + width x bpp, inside
		 * the image. Board B checks base + height x pitch against its heap's end: the heap's top guard covers the
		 * rest of that last pitch */
		struct drm_gem_object *o = a->kind == FPGAGPU_BO_B_IMAGE ? drm_gem_object_lookup(file, a->alias) : NULL;
		u64 end = a->alias_offset ? a->alias_offset + (u64)(a->height - 1) * a->pitch +
					    (u64)a->width * fg_format_bpp(a->format) : size;

		orig = o ? to_fg_bo(o) : NULL;
		if (!orig || orig->kind != FPGAGPU_BO_B_IMAGE || orig->alias_of || end > orig->size ||
		    (a->alias_offset && ((a->alias_offset & 31) || a->levels != 1 || a->layers != 1 || a->vol ||
					 a->img_flags))) {
			if (o)
				drm_gem_object_put(o);
			return -EINVAL;
		}
	}

	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo) {
		if (orig)
			drm_gem_object_put(&orig->base);
		return -ENOMEM;
	}
	bo->alias_of = orig;				/* keeps the lookup's reference until the alias is freed */
	drm_gem_private_object_init(drm, &bo->base, PAGE_ALIGN(size));
	bo->base.funcs = &fg_bo_funcs;
	bo->kind = a->kind;
	bo->format = a->format;
	if (a->kind == FPGAGPU_BO_B_IMAGE) {
		bo->width = a->width;
		bo->height = a->height;
		bo->pitch = a->pitch;
		bo->levels = a->levels;
		bo->layers = a->layers;
	}
	bo->size = size;
	/* board B renders in bands of up to 8 rows and loads and stores every row of the last band, past an image whose
	 * height is no multiple of 8: those rows get memory of the image's own, or they would rewrite the next object's
	 * bytes (and zero those that fall in the row padding of a pitch wider than the image) */
	r = fg_mem_alloc(fdev, bo, orig ? 0 : size + (a->kind == FPGAGPU_BO_B_IMAGE ?
						     (u64)(ALIGN(a->height, BAND_ROWS) - a->height) * a->pitch : 0));
	if (r) {
		if (orig)
			drm_gem_object_put(&orig->base);
		drm_gem_object_release(&bo->base);
		kfree(bo);
		return r;
	}
	addr = orig ? orig->node.start + a->alias_offset : bo->node.start;

	if (a->kind == FPGAGPU_BO_B_IMAGE) {
		type = T_CREATE_IMAGE;
		put_unaligned_le16(bo->board_handle, body);
		put_unaligned_le16(a->usage, body + 2);
		put_unaligned_le32((u32)addr, body + 4);
		put_unaligned_le16(a->pitch, body + 8);
		body[10] = a->format;
		body[11] = a->img_flags;
		put_unaligned_le16(a->width, body + 12);
		put_unaligned_le16(a->height, body + 14);
		body[16] = a->levels;
		body[17] = a->layers;
		put_unaligned_le16(a->vol, body + 18);
		blen = 20;
	} else {
		type = T_CREATE_BUFFER;
		put_unaligned_le16(bo->board_handle, body);
		put_unaligned_le16(0, body + 2);			/* usage */
		put_unaligned_le32((u32)bo->node.start, body + 4);
		put_unaligned_le32((u32)size, body + 8);
		blen = 12;
	}
	bo->create_type = type;
	bo->create_len = (u8)blen;
	memcpy(bo->create_body, body, blen);
	if (!drm_dev_enter(drm, &idx)) {
		r = -ENODEV;
		goto put;
	}
	fg_net_wait_ok(&fdev->net, SEND_TIMEOUT_MS);
	mutex_lock(&fdev->submit_lock);
	r = fg_net_send_seq(&fdev->net, type, body, blen, NULL, SEND_TIMEOUT_MS);
	if (!r) {
		bo->created = true;
		mutex_lock(&fdev->bo_lock);
		list_add_tail(&bo->bo_link, &fdev->bo_list);
		mutex_unlock(&fdev->bo_lock);
	}
	mutex_unlock(&fdev->submit_lock);
	drm_dev_exit(idx);
	if (r)
		goto put;
	r = drm_gem_handle_create(file, &bo->base, &a->handle);
	if (!r) {
		a->board_handle = bo->board_handle;
		a->board_addr = addr;
		a->size = size;
	}
put:
	drm_gem_object_put(&bo->base);		/* the handle holds its own reference */
	return r;
}

int fpgagpu_ioctl_bo_info(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct drm_fpgagpu_bo_info *a = data;
	struct drm_gem_object *obj = drm_gem_object_lookup(file, a->handle);
	struct fg_bo *bo = obj ? to_fg_bo(obj) : NULL;

	if (!bo) {
		if (obj)
			drm_gem_object_put(obj);
		return obj ? -EINVAL : -ENOENT;		/* a host-memory (dumb) object is no board BO */
	}
	a->kind = bo->kind;
	a->size = bo->size;
	a->board_handle = bo->board_handle;
	a->width = bo->width;
	a->height = bo->height;
	a->pitch = bo->pitch;
	a->format = bo->format;
	a->levels = bo->levels;
	a->layers = bo->layers;
	a->pad = 0;
	a->pad2 = 0;
	drm_gem_object_put(obj);
	return 0;
}
