// SPDX-License-Identifier: GPL-2.0 OR MIT
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
		drm_gem_object_release(&bo->base);
		kfree(bo);
	}
}

int fpgagpu_ioctl_bo_create(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct fpgagpu_device *fdev = to_fpgagpu(drm);
	struct drm_fpgagpu_bo_create *a = data;
	struct fg_bo *bo;
	u8 body[20];
	u32 blen;
	u8 type;
	u64 size;
	int r, idx;

	if (a->flags || a->pad)
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

	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo)
		return -ENOMEM;
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
	r = fg_mem_alloc(fdev, bo, size);
	if (r) {
		drm_gem_object_release(&bo->base);
		kfree(bo);
		return r;
	}

	if (a->kind == FPGAGPU_BO_B_IMAGE) {
		type = T_CREATE_IMAGE;
		put_unaligned_le16(bo->board_handle, body);
		put_unaligned_le16(a->usage, body + 2);
		put_unaligned_le32((u32)bo->node.start, body + 4);
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
	if (!drm_dev_enter(drm, &idx)) {
		r = -ENODEV;
		goto put;
	}
	mutex_lock(&fdev->submit_lock);
	r = fg_net_send_seq(&fdev->net, type, body, blen, NULL, SEND_TIMEOUT_MS);
	mutex_unlock(&fdev->submit_lock);
	drm_dev_exit(idx);
	if (r)
		goto put;
	bo->created = true;
	r = drm_gem_handle_create(file, &bo->base, &a->handle);
	if (!r) {
		a->board_handle = bo->board_handle;
		a->board_addr = bo->node.start;
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
