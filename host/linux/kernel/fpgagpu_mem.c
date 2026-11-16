// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * fpgagpu_mem: the boards' host heaps and handle tables, shared by every process.
 *
 * Heaps (the two boards' map): board A's buffers in [384 MB, 1 GB) of board A's DDR, board B's buffers and images in
 * [256 MB, 1 GB) of board B's, less its first 8 MB, which hold the display's scanout image (fpgagpu_kms.c).
 * Handles: 1024 entries each in board A's buffer table, board B's buffer table and board B's image table. Entry 0 of
 * the buffer tables stays unused (handle 0 reads as "none" in commands), image entries 0..15 are board B's built-in
 * test images and entry 1000 is the scanout image.
 *
 * The boards run every command in order, so a range can be handed out again as soon as its DESTROY is sent: what a
 * later object writes there comes after everything that read the old one.
 */
#include <linux/slab.h>

#include <drm/drm_managed.h>

#include "fpgagpu.h"
#include "fpgagpu_drm.h"

#define HEAP_A_START	0x18000000ull
#define HEAP_A_END	0x40000000ull
#define HEAP_B_START	0x10800000ull
#define HEAP_B_END	0x40000000ull
#define B_IMG_FIRST	16
#define B_IMG_SCANOUT	1000
#define BO_ALIGN	256			/* the boards want 32-byte aligned bases; 256 keeps bursts aligned */

int fg_format_bpp(u8 format)
{
	switch (format) {
	case 0x00:				/* RGBA8 */
	case 0x10:				/* D24S8 */
		return 4;
	case 0x04:				/* RGB565 */
	case 0x05:				/* RGBA4444 */
	case 0x06:				/* RGBA5551 */
	case 0x09:				/* LA88 */
	case 0x11:				/* D16 */
		return 2;
	case 0x07:				/* L8 */
	case 0x08:				/* A8 */
	case 0x0C:				/* P8 */
	case 0x12:				/* S8 */
		return 1;
	default:				/* compressed (decoded on the PC) or unknown */
		return 0;
	}
}

/*
 * The bytes an image occupies, as board B's img_geom computes them: level k is max(1, w >> k) x max(1, h >> k), its
 * pitch the descriptor's for level 0 and w_k x bpp above, its size pitch x h rounded up to 4 bytes; a layer (stride)
 * is the chain rounded up to 32 bytes; total = layers x stride x samples. A 3D image (vol): a level is pitch x h x d
 * with the depth halving too, unpadded, and the total is the chain rounded up to 32 bytes. 0 = not a valid descriptor.
 */
u64 fg_image_bytes(u8 format, u16 width, u16 height, u16 pitch, u8 levels, u8 layers, bool vol, u8 samples_log2)
{
	int bpp = fg_format_bpp(format);
	u32 wk = width, hk = height, dk = vol ? layers : 1;
	u64 chain = 0;
	unsigned int k;

	if (!bpp || !width || !height || !levels || !layers || levels > 12 || (u32)pitch < (u32)width * bpp)
		return 0;
	for (k = 0; k < levels; k++) {
		u64 pk = k ? (u64)wk * bpp : pitch;

		chain += vol ? pk * hk * dk : ALIGN(pk * hk, 4);
		wk = max(wk >> 1, 1u);
		hk = max(hk >> 1, 1u);
		dk = max(dk >> 1, 1u);
	}
	if (vol)
		return ALIGN(chain, 32);
	return ((u64)layers * ALIGN(chain, 32)) << samples_log2;
}

int fg_mem_alloc(struct fpgagpu_device *fdev, struct fg_bo *bo, u64 size)
{
	struct fg_mem *m = &fdev->mem;
	struct drm_mm *mm;
	unsigned long *map;
	unsigned int first, idx;
	u16 hbase;
	int r;

	switch (bo->kind) {
	case FPGAGPU_BO_A_BUFFER:
		mm = &m->heap_a;
		map = m->a_buf;
		first = 1;
		hbase = 0x0000;			/* bit 15 board A, bit 14 buffer */
		break;
	case FPGAGPU_BO_B_BUFFER:
		mm = &m->heap_b;
		map = m->b_buf;
		first = 1;
		hbase = 0x8000;			/* bit 15 board B, bit 14 buffer */
		break;
	case FPGAGPU_BO_B_IMAGE:
		mm = &m->heap_b;
		map = m->b_img;
		first = B_IMG_FIRST;
		hbase = 0xC000;			/* bit 15 board B, bit 14 image */
		break;
	default:
		return -EINVAL;
	}
	mutex_lock(&m->lock);
	idx = find_next_zero_bit(map, FG_HANDLES, first);
	if (idx >= FG_HANDLES) {
		r = -ENOSPC;
		goto out;
	}
	r = drm_mm_insert_node_generic(mm, &bo->node, ALIGN(size, BO_ALIGN), BO_ALIGN, 0, DRM_MM_INSERT_BEST);
	if (r)
		goto out;
	set_bit(idx, map);
	bo->board_handle = hbase | idx;
out:
	mutex_unlock(&m->lock);
	return r;
}

void fg_mem_free(struct fpgagpu_device *fdev, struct fg_bo *bo)
{
	struct fg_mem *m = &fdev->mem;
	unsigned long *map = bo->kind == FPGAGPU_BO_A_BUFFER ? m->a_buf : bo->kind == FPGAGPU_BO_B_BUFFER ? m->b_buf : m->b_img;

	mutex_lock(&m->lock);
	if (drm_mm_node_allocated(&bo->node)) {
		drm_mm_remove_node(&bo->node);
		clear_bit(bo->board_handle & (FG_HANDLES - 1), map);
	}
	mutex_unlock(&m->lock);
}

/* when the DRM device goes (every file closed): the heaps are empty by then */
static void fg_mem_release(struct drm_device *drm, void *arg)
{
	struct fg_mem *m = arg;

	drm_mm_takedown(&m->heap_a);
	drm_mm_takedown(&m->heap_b);
}

int fg_mem_init(struct fpgagpu_device *fdev)
{
	struct fg_mem *m = &fdev->mem;

	mutex_init(&m->lock);
	drm_mm_init(&m->heap_a, HEAP_A_START, HEAP_A_END - HEAP_A_START);
	drm_mm_init(&m->heap_b, HEAP_B_START, HEAP_B_END - HEAP_B_START);
	set_bit(B_IMG_SCANOUT, m->b_img);
	return drmm_add_action_or_reset(&fdev->drm, fg_mem_release, m);
}
