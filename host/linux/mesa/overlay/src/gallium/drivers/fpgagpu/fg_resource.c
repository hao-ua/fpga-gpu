/* SPDX-License-Identifier: MIT */
/*
 * fpgagpu resources and transfers.
 *
 * An image is mapped through a staging copy in host memory: a read map flushes what is pending on it, waits for the
 * image to be idle and reads the box with READ_IMAGE in the resource's transfer format; a write map sends the staged
 * pixels on unmap as WRITE_IMAGE commands (whole pixels, at most 1440 data bytes each) in SUBMITs.
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "xf86drm.h"
#include "drm-uapi/fpgagpu_drm.h"
#include "frontend/winsys_handle.h"

#include "util/format/u_format.h"
#include "util/os_time.h"
#include "util/u_atomic.h"
#include "util/u_inlines.h"
#include "util/u_memory.h"
#include "util/u_transfer.h"

#include "fg_context.h"
#include "fg_resource.h"
#include "fg_screen.h"

static const struct fg_format fg_formats[] = {
   { PIPE_FORMAT_R8G8B8A8_UNORM, FG_FMT_RGBA8, FG_TF_RGBA8888, 4, { 8, 8, 8, 8 }, false, false },
   { PIPE_FORMAT_R8G8B8X8_UNORM, FG_FMT_RGBA8, FG_TF_RGBA8888, 4, { 8, 8, 8, 0 }, true, false },
   { PIPE_FORMAT_B8G8R8A8_UNORM, FG_FMT_RGBA8, FG_TF_BGRA8888, 4, { 8, 8, 8, 8 }, false, false },
   { PIPE_FORMAT_B8G8R8X8_UNORM, FG_FMT_RGBA8, FG_TF_BGRX8888, 4, { 8, 8, 8, 0 }, true, false },
   { PIPE_FORMAT_B5G6R5_UNORM, FG_FMT_RGB565, FG_TF_RGB565, 2, { 5, 6, 5, 0 }, false, false },
   { PIPE_FORMAT_Z24_UNORM_S8_UINT, FG_FMT_D24S8, FG_TF_RAW, 4, { 0, 0, 0, 0 }, false, true },
   { PIPE_FORMAT_Z16_UNORM, FG_FMT_D16, FG_TF_RAW, 2, { 0, 0, 0, 0 }, false, true },
};

const struct fg_format *
fg_format_info(enum pipe_format format)
{
   for (unsigned i = 0; i < ARRAY_SIZE(fg_formats); i++) {
      if (fg_formats[i].pformat == format)
         return &fg_formats[i];
   }
   return NULL;
}

/* VERTEX_ATTR's client formats: 1 to 4 components of byte / short (normalized or not), fixed or float, in order */
bool
fg_vertex_format_supported(enum pipe_format format)
{
   const struct util_format_description *desc = util_format_description(format);
   const struct util_format_channel_description *c0;

   if (!desc || desc->layout != UTIL_FORMAT_LAYOUT_PLAIN || desc->nr_channels < 1 || desc->nr_channels > 4)
      return false;
   c0 = &desc->channel[0];
   for (unsigned i = 0; i < desc->nr_channels; i++) {
      const struct util_format_channel_description *c = &desc->channel[i];

      if (c->type != c0->type || c->size != c0->size || c->normalized != c0->normalized || c->pure_integer ||
          desc->swizzle[i] != PIPE_SWIZZLE_X + i)
         return false;
   }
   switch (c0->type) {
   case UTIL_FORMAT_TYPE_FLOAT:
   case UTIL_FORMAT_TYPE_FIXED:
      return c0->size == 32;
   case UTIL_FORMAT_TYPE_UNSIGNED:
   case UTIL_FORMAT_TYPE_SIGNED:
      return c0->size == 8 || c0->size == 16;
   default:
      return false;
   }
}

/* ------------------------------------------------------------------ resources */

static struct pipe_resource *
fg_resource_create(struct pipe_screen *pscreen, const struct pipe_resource *templ)
{
   struct fg_screen *screen = fg_screen(pscreen);
   struct fg_resource *rsc = CALLOC_STRUCT(fg_resource);
   struct drm_fpgagpu_bo_create c = { 0 };
   const struct fg_format *f;
   unsigned levels = templ->last_level + 1, layers = templ->array_size;
   uint32_t row, pitch;

   if (!rsc)
      return NULL;
   rsc->base = *templ;
   rsc->base.screen = pscreen;
   pipe_reference_init(&rsc->base.reference, 1);

   if (templ->target == PIPE_BUFFER) {
      rsc->data = align_malloc(MAX2(templ->width0, 1), 64);
      if (!rsc->data)
         goto fail;
      return &rsc->base;
   }

   f = fg_format_info(templ->format);
   if (!f || templ->width0 < 1 || templ->width0 > 2048 || templ->height0 < 1 || templ->height0 > 2048 ||
       templ->depth0 != 1 || levels > 12 || layers < 1 || layers > 255 || templ->nr_samples > 1)
      goto fail;
   switch (templ->target) {
   case PIPE_TEXTURE_1D:
   case PIPE_TEXTURE_2D:
   case PIPE_TEXTURE_RECT:
   case PIPE_TEXTURE_CUBE:
      break;
   default:
      goto fail;
   }

   /* the TMU's descriptor has no pitch: what is sampled has tight rows, and so has a mip chain or a layered image.
    * A single level that is only rendered into or scanned out gets a pitch that is a multiple of 32 bytes. A texture
    * that is also a render target keeps tight rows (the two agree when a row is a multiple of 32 bytes anyway) */
   row = templ->width0 * f->bpp;
   pitch = (levels > 1 || layers > 1 ||
            ((templ->bind & PIPE_BIND_SAMPLER_VIEW) &&
             !(templ->bind & (PIPE_BIND_SCANOUT | PIPE_BIND_DISPLAY_TARGET | PIPE_BIND_DEPTH_STENCIL | PIPE_BIND_SHARED))))
              ? row : align(row, 32);

   c.kind = FPGAGPU_BO_B_IMAGE;
   c.usage = FG_USE_SAMPLED | FG_USE_RT | FG_USE_TRANSFER;
   c.pitch = (uint16_t)pitch;
   c.width = (uint16_t)templ->width0;
   c.height = (uint16_t)templ->height0;
   c.format = f->storage;
   c.levels = (uint8_t)levels;
   c.layers = (uint8_t)layers;
   if (drmIoctl(screen->fd, DRM_IOCTL_FPGAGPU_BO_CREATE, &c)) {
      fprintf(stderr, "fpgagpu: BO_CREATE of a %ux%u image (format 0x%02x, %u levels, %u layers) failed: %s\n",
              templ->width0, templ->height0, f->storage, levels, layers, strerror(errno));
      goto fail;
   }
   rsc->fmt = f;
   rsc->gem = c.handle;
   rsc->board_handle = (uint16_t)c.board_handle;
   rsc->pitch = pitch;
   rsc->bo_size = c.size;
   p_atomic_inc(&screen->img_gen);	/* the handle may be one a cached draw state names */
   return &rsc->base;

fail:
   FREE(rsc);
   return NULL;
}

static void
fg_resource_destroy(struct pipe_screen *pscreen, struct pipe_resource *prsc)
{
   struct fg_resource *rsc = fg_resource(prsc);

   if (rsc->gem) {
      struct drm_gem_close c = { .handle = rsc->gem };

      drmIoctl(fg_screen(pscreen)->fd, DRM_IOCTL_GEM_CLOSE, &c);
   }
   align_free(rsc->data);
   FREE(rsc);
}

static struct pipe_resource *
fg_resource_from_handle(struct pipe_screen *pscreen, const struct pipe_resource *templ,
                        struct winsys_handle *whandle, unsigned usage)
{
   FG_NOTE_ONCE("importing a resource from a handle");
   return NULL;
}

/* a board-B image is named by its GEM handle on this device; it has no host pages to export as a dma-buf */
static bool
fg_resource_get_handle(struct pipe_screen *pscreen, struct pipe_context *pctx, struct pipe_resource *prsc,
                       struct winsys_handle *whandle, unsigned usage)
{
   struct fg_resource *rsc = fg_resource(prsc);

   if (!rsc->gem || whandle->type != WINSYS_HANDLE_TYPE_KMS)
      return false;
   whandle->handle = rsc->gem;
   whandle->stride = rsc->pitch;
   whandle->offset = 0;
   whandle->size = rsc->bo_size;
   return true;
}

void
fg_resource_screen_init(struct pipe_screen *pscreen)
{
   pscreen->resource_create = fg_resource_create;
   pscreen->resource_destroy = fg_resource_destroy;
   pscreen->resource_from_handle = fg_resource_from_handle;
   pscreen->resource_get_handle = fg_resource_get_handle;
}

/* ------------------------------------------------------------------ transfers */

static void *
fg_buffer_map(struct pipe_context *pctx, struct pipe_resource *prsc, unsigned level, unsigned usage,
              const struct pipe_box *box, struct pipe_transfer **ptransfer)
{
   struct fg_resource *rsc = fg_resource(prsc);
   struct fg_transfer *t = CALLOC_STRUCT(fg_transfer);

   if (!t)
      return NULL;
   pipe_resource_reference(&t->base.resource, prsc);
   t->base.level = level;
   t->base.usage = usage;
   t->base.box = *box;
   *ptransfer = &t->base;
   return (uint8_t *)rsc->data + box->x;
}

static void
fg_buffer_unmap(struct pipe_context *pctx, struct pipe_transfer *ptransfer)
{
   pipe_resource_reference(&ptransfer->resource, NULL);
   FREE(ptransfer);
}

/* the staged pixels of one map as WRITE_IMAGE commands, as few SUBMITs as the kernel's stream limit allows */
static bool
fg_write_image(struct fg_context *ctx, struct fg_resource *rsc, unsigned level, const struct pipe_box *box,
               uint8_t *data, uintptr_t layer_stride)
{
   struct fg_screen *screen = fg_screen(ctx->base.screen);
   const struct fg_format *f = rsc->fmt;
   const uint32_t npix = box->width * box->height, per = FG_MAX_DATA / f->bpp;
   const uint32_t cmd_max = 4 + 24 + per * f->bpp, cmds_per_submit = FG_SUBMIT_MAX / cmd_max;
   const uint32_t total = DIV_ROUND_UP(npix, per) * box->depth;
   uint32_t ncmds = 0, pos = 0;
   uint8_t *cmds = malloc((size_t)MIN2(total, cmds_per_submit) * cmd_max);
   bool ok = true;

   if (!cmds)
      return false;
   for (int z = 0; z < box->depth && ok; z++) {
      uint8_t *src = data + z * layer_stride;

      if (f->force_alpha && f->bpp == 4) {
         for (uint32_t i = 0; i < npix; i++)
            src[4 * i + 3] = 0xFF;
      }
      for (uint32_t p = 0; p < npix && ok; p += per) {
         uint32_t n = MIN2(per, npix - p);
         uint8_t hdr[24] = { 0 };
         uint64_t fence;
         int r;

         /* handle, level, layer, transfer format, 0, 0, x, y, w, h, first pixel of the rectangle, 0 */
         fg_put16(hdr + 0, rsc->board_handle);
         hdr[2] = (uint8_t)level;
         hdr[3] = (uint8_t)(box->z + z);
         hdr[4] = f->tformat;
         fg_put16(hdr + 8, box->x);
         fg_put16(hdr + 10, box->y);
         fg_put16(hdr + 12, box->width);
         fg_put16(hdr + 14, box->height);
         fg_put32(hdr + 16, p);
         pos += fg_put_cmd(cmds + pos, FG_T_WRITE_IMAGE, hdr, sizeof(hdr), src + (size_t)p * f->bpp, n * f->bpp);
         if (++ncmds < cmds_per_submit && !(z == box->depth - 1 && p + n >= npix))
            continue;
         r = fg_submit(screen, cmds, pos, &rsc->gem, 1, &fence);
         if (r) {
            fprintf(stderr, "fpgagpu: SUBMIT of WRITE_IMAGE failed: %s\n", strerror(-r));
            ok = false;
         } else {
            fg_context_fenced(ctx, fence);
         }
         ncmds = 0;
         pos = 0;
      }
   }
   free(cmds);
   if (ok)
      rsc->initialized = true;
   return ok;
}

static void *
fg_texture_map(struct pipe_context *pctx, struct pipe_resource *prsc, unsigned level, unsigned usage,
               const struct pipe_box *box, struct pipe_transfer **ptransfer)
{
   struct fg_context *ctx = fg_context(pctx);
   struct fg_screen *screen = fg_screen(pctx->screen);
   struct fg_resource *rsc = fg_resource(prsc);
   const struct fg_format *f = rsc->fmt;
   bool read = (usage & PIPE_MAP_READ) ||
               ((usage & PIPE_MAP_WRITE) && rsc->initialized &&
                !(usage & (PIPE_MAP_DISCARD_RANGE | PIPE_MAP_DISCARD_WHOLE_RESOURCE)));
   struct fg_transfer *t;
   int64_t t0 = screen->debug ? os_time_get_nano() : 0, t1 = t0, t2 = t0;
   int r;

   if (!f || box->width <= 0 || box->height <= 0 || box->depth <= 0)
      return NULL;

   /* what is pending on the image goes first, reads or writes */
   if (fg_context_pending_uses(ctx, prsc))
      fg_context_flush_pending(ctx);
   if (screen->debug)
      t1 = os_time_get_nano();

   if (read && !(usage & PIPE_MAP_UNSYNCHRONIZED)) {
      r = fg_wait_bo(screen, rsc->gem, (usage & PIPE_MAP_DONTBLOCK) ? 0 : FG_WAIT_NS);
      if (r) {
         if (!(usage & PIPE_MAP_DONTBLOCK))
            fprintf(stderr, "fpgagpu: WAIT for an image failed: %s\n", strerror(-r));
         return NULL;
      }
   }
   if (screen->debug)
      t2 = os_time_get_nano();

   t = CALLOC_STRUCT(fg_transfer);
   if (!t)
      return NULL;
   t->base.level = level;
   t->base.usage = usage;
   t->base.box = *box;
   t->base.stride = box->width * f->bpp;
   t->base.layer_stride = (uintptr_t)t->base.stride * box->height;
   t->staging = malloc(t->base.layer_stride * box->depth);
   if (!t->staging) {
      FREE(t);
      return NULL;
   }

   if (read) {
      for (int z = 0; z < box->depth; z++) {
         struct drm_fpgagpu_read rd = {
            .handle = rsc->gem,
            .data = (uintptr_t)((uint8_t *)t->staging + z * t->base.layer_stride),
            .size = t->base.layer_stride,
            .x = (uint16_t)box->x,
            .y = (uint16_t)box->y,
            .w = (uint16_t)box->width,
            .h = (uint16_t)box->height,
            .level = (uint8_t)level,
            .layer = (uint8_t)(box->z + z),
            .tformat = f->tformat,
         };

         if (drmIoctl(screen->fd, DRM_IOCTL_FPGAGPU_READ, &rd)) {
            fprintf(stderr, "fpgagpu: READ of a %dx%d box failed: %s\n", box->width, box->height, strerror(errno));
            free(t->staging);
            FREE(t);
            return NULL;
         }
      }
      if (screen->debug)
         fprintf(stderr, "fpgagpu: read map %dx%dx%d: submit %.1f ms, wait %.1f ms, READ %.1f ms\n", box->width,
                 box->height, box->depth, (t1 - t0) / 1e6, (t2 - t1) / 1e6, (os_time_get_nano() - t2) / 1e6);
   }

   pipe_resource_reference(&t->base.resource, prsc);
   *ptransfer = &t->base;
   return t->staging;
}

static void
fg_texture_unmap(struct pipe_context *pctx, struct pipe_transfer *ptransfer)
{
   struct fg_transfer *t = (struct fg_transfer *)ptransfer;

   if (ptransfer->usage & PIPE_MAP_WRITE) {
      struct fg_context *ctx = fg_context(pctx);
      struct fg_resource *rsc = fg_resource(ptransfer->resource);

      if (fg_context_pending_uses(ctx, ptransfer->resource))
         fg_context_flush_pending(ctx);
      fg_write_image(ctx, rsc, ptransfer->level, &ptransfer->box, t->staging, ptransfer->layer_stride);
   }
   free(t->staging);
   pipe_resource_reference(&ptransfer->resource, NULL);
   FREE(t);
}

void
fg_resource_context_init(struct pipe_context *pctx)
{
   pctx->buffer_map = fg_buffer_map;
   pctx->buffer_unmap = fg_buffer_unmap;
   pctx->texture_map = fg_texture_map;
   pctx->texture_unmap = fg_texture_unmap;
   pctx->transfer_flush_region = u_default_transfer_flush_region;
   pctx->buffer_subdata = u_default_buffer_subdata;
   pctx->texture_subdata = u_default_texture_subdata;
}
