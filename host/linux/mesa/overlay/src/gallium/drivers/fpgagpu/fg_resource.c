/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026 Dmytro Ivanysh */
/*
 * fpgagpu resources and transfers.
 *
 * An image is mapped through a staging copy in host memory: a read map flushes what is pending on it, waits for the
 * image to be idle and reads the box with READ_IMAGE in the resource's transfer format; a write map sends the staged
 * pixels on unmap as WRITE_IMAGE commands (whole pixels, at most 1440 data bytes each) in SUBMITs.
 */
#include <errno.h>
#include <fcntl.h>
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
#include "util/u_math.h"
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
   /* sampled only, stored as RGBA8 */
   { PIPE_FORMAT_L8_UNORM, FG_FMT_RGBA8, FG_TF_RGBA8888, 1, { 0, 0, 0, 0 }, false, false, FG_EXPAND_L },
   { PIPE_FORMAT_A8_UNORM, FG_FMT_RGBA8, FG_TF_RGBA8888, 1, { 0, 0, 0, 8 }, false, false, FG_EXPAND_A },
   { PIPE_FORMAT_L8A8_UNORM, FG_FMT_RGBA8, FG_TF_RGBA8888, 2, { 0, 0, 0, 0 }, false, false, FG_EXPAND_LA },
   { PIPE_FORMAT_R8_UNORM, FG_FMT_RGBA8, FG_TF_RGBA8888, 1, { 8, 0, 0, 0 }, false, false, FG_EXPAND_R },
   { PIPE_FORMAT_R8G8_UNORM, FG_FMT_RGBA8, FG_TF_RGBA8888, 2, { 8, 8, 0, 0 }, false, false, FG_EXPAND_RG },
   { PIPE_FORMAT_Z24_UNORM_S8_UINT, FG_FMT_D24S8, FG_TF_RAW, 4, { 0, 0, 0, 0 }, false, true },
   { PIPE_FORMAT_Z16_UNORM, FG_FMT_D24S8, FG_TF_RAW, 2, { 0, 0, 0, 0 }, false, true, FG_EXPAND_Z16 },
};

/* what an expanded format travels as: RGBA8, or (Z16) D24S8 words */
static const struct fg_format fg_rgba8_wire = {
   PIPE_FORMAT_R8G8B8A8_UNORM, FG_FMT_RGBA8, FG_TF_RGBA8888, 4, { 8, 8, 8, 8 }, false, false, FG_EXPAND_NONE,
};
static const struct fg_format fg_d24s8_wire = {
   PIPE_FORMAT_Z24_UNORM_S8_UINT, FG_FMT_D24S8, FG_TF_RAW, 4, { 0, 0, 0, 0 }, false, true, FG_EXPAND_NONE,
};

static const struct fg_format *
fg_wire(const struct fg_format *f)
{
   return f->expand == FG_EXPAND_Z16 ? &fg_d24s8_wire : &fg_rgba8_wire;
}

/* RGBA8 and RGB565; and A8, R8, RG8, rendered with their own channels' writes into their RGBA8 storage, whose other
 * channels keep the values the format reads as */
bool
fg_format_renderable(const struct fg_format *f)
{
   return f && !f->depth &&
          (f->expand == FG_EXPAND_NONE || f->expand == FG_EXPAND_A || f->expand == FG_EXPAND_R ||
           f->expand == FG_EXPAND_RG) &&
          (f->storage == FG_FMT_RGBA8 || f->storage == FG_FMT_RGB565);
}

/* n pixels of an expanded format to RGBA8 (Z16: to D24S8 words, the depth 0 .. 65535, stencil 0), and back */
static void
fg_expand(const struct fg_format *f, const uint8_t *src, uint8_t *dst, size_t n)
{
   for (size_t i = 0; i < n; i++) {
      const uint8_t *s = src + i * f->bpp;
      uint8_t *d = dst + 4 * i;

      switch (f->expand) {
      case FG_EXPAND_Z16: d[0] = s[0]; d[1] = s[1]; d[2] = d[3] = 0; break;
      case FG_EXPAND_A: d[0] = d[1] = d[2] = 0; d[3] = s[0]; break;
      case FG_EXPAND_L: d[0] = d[1] = d[2] = s[0]; d[3] = 255; break;
      case FG_EXPAND_LA: d[0] = d[1] = d[2] = s[0]; d[3] = s[1]; break;
      case FG_EXPAND_R: d[0] = s[0]; d[1] = d[2] = 0; d[3] = 255; break;
      default: d[0] = s[0]; d[1] = s[1]; d[2] = 0; d[3] = 255; break;
      }
   }
}

static void
fg_pack(const struct fg_format *f, const uint8_t *src, uint8_t *dst, size_t n)
{
   for (size_t i = 0; i < n; i++) {
      const uint8_t *s = src + 4 * i;
      uint8_t *d = dst + i * f->bpp;

      switch (f->expand) {
      case FG_EXPAND_A: d[0] = s[3]; break;
      case FG_EXPAND_L: case FG_EXPAND_R: d[0] = s[0]; break;
      case FG_EXPAND_LA: d[0] = s[0]; d[1] = s[3]; break;
      default: d[0] = s[0]; d[1] = s[1]; break; /* RG8; Z16: the word's low 16 bits */
      }
   }
}

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
   fg_shadows_stale(rsc);

   if (templ->target == PIPE_BUFFER) {
      rsc->data = align_malloc(MAX2(templ->width0, 1), 64);
      if (!rsc->data)
         goto fail;
      return &rsc->base;
   }

   f = fg_format_info(templ->format);
   if (!f || templ->width0 < 1 || templ->width0 > 2048 || templ->height0 < 1 || templ->height0 > 2048 ||
       templ->depth0 != 1 || levels > 12 || layers < 1 || layers > 255 || templ->nr_samples > 1 ||
       (templ->target != PIPE_TEXTURE_1D && templ->target != PIPE_TEXTURE_2D && templ->target != PIPE_TEXTURE_RECT &&
        templ->target != PIPE_TEXTURE_CUBE)) {
      if (screen->debug)
         fprintf(stderr, "fpgagpu: no image for %s %ux%ux%u, %u levels, %u layers, %u samples, target %d, bind 0x%x\n",
                 util_format_name(templ->format), templ->width0, templ->height0, templ->depth0, levels, layers,
                 templ->nr_samples, templ->target, templ->bind);
      goto fail;
   }

   /* the TMU's descriptor has no pitch: what is sampled has tight rows, and so has a mip chain or a layered image.
    * A single level that is only rendered into or scanned out gets a pitch that is a multiple of 32 bytes. A texture
    * that is also a render target keeps tight rows (the two agree when a row is a multiple of 32 bytes anyway); an
    * image with padded rows that is sampled after all (shared, scanned out, depth) is read from a tight-row copy */
   /* (board B loads and stores a depth-stencil row in as many bursts as the colour target's row takes, times the bursts
    * of a group: twice an RGB565 row's. A depth pitch that is a multiple of 64 holds that for a colour target of its
    * width in either format) */
   row = templ->width0 * fg_storage_bpp(f);
   pitch = (levels > 1 || layers > 1 ||
            ((templ->bind & PIPE_BIND_SAMPLER_VIEW) &&
             !(templ->bind & (PIPE_BIND_SCANOUT | PIPE_BIND_DISPLAY_TARGET | PIPE_BIND_DEPTH_STENCIL | PIPE_BIND_SHARED))))
              ? row : align(row, f->depth ? 64 : 32);

   c.kind = FPGAGPU_BO_B_IMAGE;
   c.usage = FG_USE_SAMPLED | FG_USE_RT | FG_USE_TRANSFER;
   if (templ->bind & (PIPE_BIND_SCANOUT | PIPE_BIND_DISPLAY_TARGET | PIPE_BIND_SHARED))
      c.usage |= FG_USE_SCANOUT;	/* board B shows it (a KMS framebuffer, PRESENT) */
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
   fg_gem_ref(screen, c.handle);
   rsc->board_handle = (uint16_t)c.board_handle;
   rsc->pitch = pitch;
   rsc->bo_size = c.size;
   p_atomic_inc(&screen->img_gen);	/* the handle may be one a cached draw state names */
   if (f->storage == FG_FMT_D24S8 && (templ->bind & PIPE_BIND_SAMPLER_VIEW)) {
      /* the TMU has no D24S8 format: samplers name an RGBA8 descriptor of the same bytes (the fragment program turns
       * the word into the depth) */
      struct drm_fpgagpu_bo_create a = c;

      a.flags = FPGAGPU_BO_CREATE_ALIAS;
      a.alias = c.handle;
      a.format = FG_FMT_RGBA8;
      a.usage = FG_USE_SAMPLED;
      a.handle = a.board_handle = 0;
      if (!drmIoctl(screen->fd, DRM_IOCTL_FPGAGPU_BO_CREATE, &a)) {
         rsc->sample_gem = a.handle;
         rsc->sample_handle = (uint16_t)a.board_handle;
         fg_gem_ref(screen, a.handle);
      } else {
         fprintf(stderr, "fpgagpu: no RGBA8 alias of a %ux%u depth image: %s\n", templ->width0, templ->height0,
                 strerror(errno));
      }
   }
   if (fg_format_wmask(f) != 0xF && (templ->bind & PIPE_BIND_RENDER_TARGET)) {
      /* the channels rendering leaves alone must read as the format says: A8 (0, 0, 0, .), R8 / RG8 (., ., 0, 1).
       * FILL_IMAGE: handle, level, layer, transfer format, channel mask, 0, x, y, w, h, value (bytes R, G, B, A) */
      const uint32_t value = f->expand == FG_EXPAND_A ? 0 : 0xFF000000u;
      uint8_t cmd[4 + 20], b[20] = { 0 };
      uint64_t fence;

      for (unsigned l = 0; l < levels; l++) {
         for (unsigned z = 0; z < layers; z++) {
            fg_put16(b, rsc->board_handle);
            b[2] = (uint8_t)l;
            b[3] = (uint8_t)z;
            b[4] = FG_TF_RGBA8888;
            b[5] = 0xF;
            fg_put16(b + 12, u_minify(templ->width0, l));
            fg_put16(b + 14, u_minify(templ->height0, l));
            fg_put32(b + 16, value);
            fg_put_cmd(cmd, FG_T_FILL_IMAGE, b, sizeof(b), NULL, 0);
            fg_submit(screen, cmd, sizeof(cmd), &rsc->gem, 1, &fence);
         }
      }
   }
   if (screen->debug && (templ->bind & (PIPE_BIND_SCANOUT | PIPE_BIND_DISPLAY_TARGET | PIPE_BIND_SHARED)))
      fprintf(stderr, "fpgagpu: shown image %s %ux%u pitch %u, bind 0x%x: GEM %u, board 0x%04x\n",
              util_format_name(templ->format), templ->width0, templ->height0, pitch, templ->bind, c.handle,
              c.board_handle);
   return &rsc->base;

fail:
   FREE(rsc);
   return NULL;
}

static void
fg_resource_destroy(struct pipe_screen *pscreen, struct pipe_resource *prsc)
{
   struct fg_resource *rsc = fg_resource(prsc);

   for (unsigned l = 0; l < FG_MAX_LEVELS; l++)
      pipe_resource_reference(&rsc->shadow[l], NULL);
   pipe_resource_reference(&rsc->tight, NULL);
   pipe_resource_reference(&rsc->lview, NULL);
   for (unsigned i = 0; i < ARRAY_SIZE(rsc->ralias); i++)
      if (rsc->ralias[i].gem)
         fg_gem_unref(fg_screen(pscreen), rsc->ralias[i].gem);
   if (rsc->sample_gem)
      fg_gem_unref(fg_screen(pscreen), rsc->sample_gem);
   if (rsc->gem)
      fg_gem_unref(fg_screen(pscreen), rsc->gem);
   align_free(rsc->data);
   FREE(rsc);
}

uint16_t
fg_rect_alias(struct pipe_screen *pscreen, struct fg_resource *rsc, unsigned x, unsigned y, unsigned width,
              unsigned height, uint32_t *gem)
{
   struct fg_screen *screen = fg_screen(pscreen);
   uint32_t off = y * rsc->pitch + x * fg_storage_bpp(rsc->fmt);
   struct drm_fpgagpu_bo_create a = { 0 };
   struct fg_rect_alias *e;

   for (unsigned i = 0; i < ARRAY_SIZE(rsc->ralias); i++) {
      e = &rsc->ralias[i];
      if (e->gem && e->offset == off && e->width == width && e->height == height) {
         *gem = e->gem;
         return e->handle;
      }
   }
   a.kind = FPGAGPU_BO_B_IMAGE;
   a.flags = FPGAGPU_BO_CREATE_ALIAS;
   a.alias = rsc->gem;
   a.alias_offset = off;
   a.usage = FG_USE_RT;
   a.pitch = (uint16_t)rsc->pitch;
   a.width = (uint16_t)width;
   a.height = (uint16_t)height;
   a.format = rsc->fmt->storage;
   a.levels = 1;
   a.layers = 1;
   if (drmIoctl(screen->fd, DRM_IOCTL_FPGAGPU_BO_CREATE, &a)) {
      if (screen->debug)
         fprintf(stderr, "fpgagpu: no %ux%u alias at %u, %u of a %ux%u image: %s\n", width, height, x, y,
                 rsc->base.width0, rsc->base.height0, strerror(errno));
      return 0;
   }
   /* the oldest goes: every pass that named it was submitted when it ended */
   e = &rsc->ralias[rsc->ralias_next++ % ARRAY_SIZE(rsc->ralias)];
   if (e->gem)
      fg_gem_unref(screen, e->gem);
   e->gem = a.handle;
   e->handle = (uint16_t)a.board_handle;
   e->offset = off;
   e->width = (uint16_t)width;
   e->height = (uint16_t)height;
   fg_gem_ref(screen, a.handle);
   *gem = e->gem;
   return e->handle;
}

/* a board-B image from a dma-buf (another process's, through PRIME) or a GEM handle of this fd: the kernel says what
 * it is (BO_INFO), the template what it is used as */
static struct pipe_resource *
fg_resource_from_handle(struct pipe_screen *pscreen, const struct pipe_resource *templ,
                        struct winsys_handle *whandle, unsigned usage)
{
   struct fg_screen *screen = fg_screen(pscreen);
   struct drm_fpgagpu_bo_info info = { 0 };
   const struct fg_format *f = fg_format_info(templ->format);
   struct fg_resource *rsc;
   uint32_t gem;

   if (whandle->type == WINSYS_HANDLE_TYPE_FD) {
      if (drmPrimeFDToHandle(screen->fd, (int)whandle->handle, &gem))
         return NULL;
   } else if (whandle->type == WINSYS_HANDLE_TYPE_KMS) {
      gem = whandle->handle;
   } else {
      return NULL;
   }
   info.handle = gem;
   if (drmIoctl(screen->fd, DRM_IOCTL_FPGAGPU_BO_INFO, &info) || info.kind != FPGAGPU_BO_B_IMAGE || !f ||
       f->storage != info.format || templ->width0 > info.width || templ->height0 > info.height || whandle->offset ||
       (whandle->stride && whandle->stride != info.pitch)) {
      if (screen->debug)
         fprintf(stderr, "fpgagpu: no import of GEM %u as %s %ux%u (pitch %u): kind %u format 0x%02x %ux%u pitch %u\n",
                 gem, util_format_name(templ->format), templ->width0, templ->height0, whandle->stride, info.kind,
                 info.format, info.width, info.height, info.pitch);
      return NULL;
   }
   rsc = CALLOC_STRUCT(fg_resource);
   if (!rsc)
      return NULL;
   rsc->base = *templ;
   rsc->base.screen = pscreen;
   pipe_reference_init(&rsc->base.reference, 1);
   fg_shadows_stale(rsc);
   rsc->fmt = f;
   rsc->gem = gem;
   rsc->board_handle = (uint16_t)info.board_handle;
   rsc->pitch = info.pitch;
   rsc->bo_size = info.size;
   rsc->initialized = true;
   rsc->shared = true;
   fg_gem_ref(screen, gem);
   return &rsc->base;
}

/* a board-B image by its GEM handle on this device, or as a dma-buf for another user of the device (it has no host
 * pages: only this device can take it) */
static bool
fg_resource_get_handle(struct pipe_screen *pscreen, struct pipe_context *pctx, struct pipe_resource *prsc,
                       struct winsys_handle *whandle, unsigned usage)
{
   struct fg_resource *rsc = fg_resource(prsc);

   if (!rsc->gem)
      return false;
   if (whandle->type == WINSYS_HANDLE_TYPE_FD) {
      int prime;

      if (drmPrimeHandleToFD(fg_screen(pscreen)->fd, rsc->gem, DRM_CLOEXEC | DRM_RDWR, &prime))
         return false;
      whandle->handle = (unsigned)prime;
   } else if (whandle->type == WINSYS_HANDLE_TYPE_KMS) {
      whandle->handle = rsc->gem;
   } else {
      return false;
   }
   whandle->stride = rsc->pitch;
   whandle->offset = 0;
   whandle->size = rsc->bo_size;
   rsc->shared = true;
   return true;
}

/* what GBM and the DRI frontend ask of a buffer: one linear plane, its pitch, its GEM handle */
static bool
fg_resource_get_param(struct pipe_screen *pscreen, struct pipe_context *pctx, struct pipe_resource *prsc,
                      unsigned plane, unsigned layer, unsigned level, enum pipe_resource_param param,
                      unsigned handle_usage, uint64_t *value)
{
   struct fg_resource *rsc = fg_resource(prsc);

   if (!rsc->gem)
      return false;
   switch (param) {
   case PIPE_RESOURCE_PARAM_NPLANES:
      *value = 1;
      return true;
   case PIPE_RESOURCE_PARAM_STRIDE:
      *value = rsc->pitch;
      return true;
   case PIPE_RESOURCE_PARAM_OFFSET:
      *value = 0;
      return true;
   case PIPE_RESOURCE_PARAM_MODIFIER:
      *value = 0; /* DRM_FORMAT_MOD_LINEAR */
      return true;
   case PIPE_RESOURCE_PARAM_HANDLE_TYPE_KMS:
      *value = rsc->gem;
      return true;
   default:
      return false;
   }
}

void
fg_resource_screen_init(struct pipe_screen *pscreen)
{
   pscreen->resource_create = fg_resource_create;
   pscreen->resource_destroy = fg_resource_destroy;
   pscreen->resource_from_handle = fg_resource_from_handle;
   pscreen->resource_get_handle = fg_resource_get_handle;
   pscreen->resource_get_param = fg_resource_get_param;
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

/* the staged pixels of one map, in format f, as WRITE_IMAGE commands, as few SUBMITs as the kernel's stream limit
 * allows */
static bool
fg_write_image(struct fg_context *ctx, struct fg_resource *rsc, const struct fg_format *f, unsigned level,
               const struct pipe_box *box, uint8_t *data, uintptr_t layer_stride)
{
   struct fg_screen *screen = fg_screen(ctx->base.screen);
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
   fg_resource_written(rsc);
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

   /* what is pending on the image goes first, reads or writes; and what the stream holds (a copy into it) */
   if (fg_context_pending_uses(ctx, prsc))
      fg_context_flush_pending(ctx);
   fg_submit_stream(ctx);
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
      size_t npix = (size_t)box->width * box->height;
      uint8_t *wide = f->expand ? malloc(npix * 4) : NULL;

      if (f->expand && !wide) {
         free(t->staging);
         FREE(t);
         return NULL;
      }
      for (int z = 0; z < box->depth; z++) {
         uint8_t *dst = (uint8_t *)t->staging + z * t->base.layer_stride;
         struct drm_fpgagpu_read rd = {
            .handle = rsc->gem,
            .data = (uintptr_t)(wide ? wide : dst),
            .size = wide ? npix * 4 : t->base.layer_stride,
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
            free(wide);
            free(t->staging);
            FREE(t);
            return NULL;
         }
         if (wide)
            fg_pack(f, wide, dst, npix);
      }
      free(wide);
      if (screen->debug)
         fprintf(stderr, "fpgagpu: read map %dx%dx%d: submit %.1f ms, wait %.1f ms, READ %.1f ms\n", box->width,
                 box->height, box->depth, (t1 - t0) / 1e6, (t2 - t1) / 1e6, (os_time_get_nano() - t2) / 1e6);
   }

   pipe_resource_reference(&t->base.resource, prsc);
   *ptransfer = &t->base;
   return t->staging;
}

bool
fg_image_io(struct fg_context *ctx, struct pipe_resource *prsc, bool write, unsigned w, unsigned h, void *data)
{
   struct fg_screen *screen = fg_screen(ctx->base.screen);
   struct fg_resource *rsc = fg_resource(prsc);
   const struct fg_format *f = rsc->fmt->storage == FG_FMT_RGB565 ? fg_format_info(PIPE_FORMAT_B5G6R5_UNORM)
                                                                   : &fg_rgba8_wire;

   if (write) {
      struct pipe_box box = { .width = (int)w, .height = (int)h, .depth = 1 };

      return fg_write_image(ctx, rsc, f, 0, &box, data, (uintptr_t)w * h * f->bpp);
   }
   fg_submit_stream(ctx);
   if (fg_wait_bo(screen, rsc->gem, FG_WAIT_NS))
      return false;
   {
      struct drm_fpgagpu_read rd = {
         .handle = rsc->gem,
         .data = (uintptr_t)data,
         .size = (uint64_t)w * h * f->bpp,
         .w = (uint16_t)w,
         .h = (uint16_t)h,
         .tformat = f->tformat,
      };

      return !drmIoctl(screen->fd, DRM_IOCTL_FPGAGPU_READ, &rd);
   }
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
      fg_submit_stream(ctx);
      fg_shadows_stale(rsc);
      if (rsc->fmt->expand) {
         const struct pipe_box *b = &ptransfer->box;
         size_t n = (size_t)b->width * b->height * b->depth;
         uint8_t *wide = malloc(n * 4);

         if (wide) {
            fg_expand(rsc->fmt, t->staging, wide, n);
            fg_write_image(ctx, rsc, fg_wire(rsc->fmt), ptransfer->level, b, wide, (uintptr_t)b->width * b->height * 4);
            free(wide);
         }
      } else {
         fg_write_image(ctx, rsc, rsc->fmt, ptransfer->level, &ptransfer->box, t->staging, ptransfer->layer_stride);
      }
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
