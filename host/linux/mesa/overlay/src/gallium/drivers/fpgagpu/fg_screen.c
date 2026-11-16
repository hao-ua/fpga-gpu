/* SPDX-License-Identifier: MIT */
/*
 * fpgagpu screen: capabilities, supported formats, fences and the kernel interface.
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "xf86drm.h"
#include "drm-uapi/fpgagpu_drm.h"

#include "compiler/nir/nir.h"
#include "util/format/u_format.h"
#include "util/u_debug.h"
#include "util/u_inlines.h"
#include "util/u_memory.h"
#include "util/u_screen.h"

#include "fg_context.h"
#include "fg_resource.h"
#include "fg_screen.h"

/* ------------------------------------------------------------------ kernel interface */

uint32_t
fg_put_cmd(uint8_t *p, uint8_t type, const void *head, uint32_t head_len, const void *tail, uint32_t tail_len)
{
   uint32_t len = head_len + tail_len, padded = (len + 3u) & ~3u;

   p[0] = type;
   p[1] = 0;
   fg_put16(p + 2, len);
   if (head_len)
      memcpy(p + 4, head, head_len);
   if (tail_len)
      memcpy(p + 4 + head_len, tail, tail_len);
   memset(p + 4 + len, 0, padded - len);
   return 4 + padded;
}

int
fg_submit(struct fg_screen *screen, const void *cmds, uint32_t bytes, const uint32_t *bos, uint32_t nbos,
          uint64_t *fence)
{
   struct drm_fpgagpu_submit s = {
      .cmds = (uintptr_t)cmds,
      .cmds_bytes = bytes,
      .bos = (uintptr_t)bos,
      .nbos = nbos,
   };

   if (drmIoctl(screen->fd, DRM_IOCTL_FPGAGPU_SUBMIT, &s))
      return -errno;
   if (fence)
      *fence = s.fence;
   return 0;
}

int
fg_wait_fence(struct fg_screen *screen, uint64_t fence, int64_t timeout_ns)
{
   struct drm_fpgagpu_wait w = { .fence = fence, .timeout_ns = timeout_ns };

   return drmIoctl(screen->fd, DRM_IOCTL_FPGAGPU_WAIT, &w) ? -errno : 0;
}

int
fg_wait_bo(struct fg_screen *screen, uint32_t gem, int64_t timeout_ns)
{
   struct drm_fpgagpu_wait w = { .handle = gem, .timeout_ns = timeout_ns };

   return drmIoctl(screen->fd, DRM_IOCTL_FPGAGPU_WAIT, &w) ? -errno : 0;
}

void
fg_gem_ref(struct fg_screen *screen, uint32_t gem)
{
   unsigned i;

   simple_mtx_lock(&screen->gem_lock);
   for (i = 0; i < screen->ngems && screen->gems[i].gem != gem; i++)
      ;
   if (i < screen->ngems) {
      screen->gems[i].refs++;
   } else {
      if (screen->ngems == screen->gemcap) {
         unsigned cap = screen->gemcap ? 2 * screen->gemcap : 256;
         void *n = realloc(screen->gems, cap * sizeof(*screen->gems));

         if (!n) {
            simple_mtx_unlock(&screen->gem_lock);
            return;
         }
         screen->gems = n;
         screen->gemcap = cap;
      }
      screen->gems[screen->ngems].gem = gem;
      screen->gems[screen->ngems].refs = 1;
      screen->ngems++;
   }
   simple_mtx_unlock(&screen->gem_lock);
}

void
fg_gem_unref(struct fg_screen *screen, uint32_t gem)
{
   unsigned i;
   bool close = true;

   simple_mtx_lock(&screen->gem_lock);
   for (i = 0; i < screen->ngems && screen->gems[i].gem != gem; i++)
      ;
   if (i < screen->ngems) {
      close = --screen->gems[i].refs == 0;
      if (close)
         screen->gems[i] = screen->gems[--screen->ngems];
   }
   simple_mtx_unlock(&screen->gem_lock);
   if (close) {
      struct drm_gem_close c = { .handle = gem };

      drmIoctl(screen->fd, DRM_IOCTL_GEM_CLOSE, &c);
   }
}

static int
fg_get_param(int fd, uint32_t param, uint32_t *value)
{
   struct drm_fpgagpu_get_param p = { .param = param };

   if (drmIoctl(fd, DRM_IOCTL_FPGAGPU_GET_PARAM, &p))
      return -errno;
   *value = (uint32_t)p.value;
   return 0;
}

/* ------------------------------------------------------------------ fences */

struct pipe_fence_handle *
fg_fence_create(bool submitted, uint64_t seqno)
{
   struct pipe_fence_handle *f = CALLOC_STRUCT(pipe_fence_handle);

   if (!f)
      return NULL;
   pipe_reference_init(&f->reference, 1);
   f->submitted = submitted;
   f->seqno = seqno;
   return f;
}

static void
fg_fence_reference(struct pipe_screen *pscreen, struct pipe_fence_handle **ptr, struct pipe_fence_handle *fence)
{
   if (pipe_reference(*ptr ? &(*ptr)->reference : NULL, fence ? &fence->reference : NULL))
      FREE(*ptr);
   *ptr = fence;
}

static bool
fg_fence_finish(struct pipe_screen *pscreen, struct pipe_context *pctx, struct pipe_fence_handle *fence,
                uint64_t timeout)
{
   int64_t t = timeout > (uint64_t)INT64_MAX ? INT64_MAX : (int64_t)timeout;
   int r;

   if (!fence->submitted)
      return true;
   r = fg_wait_fence(fg_screen(pscreen), fence->seqno, t);
   if (r && r != -EBUSY && r != -ETIMEDOUT)
      fprintf(stderr, "fpgagpu: WAIT on fence %llu failed: %s\n", (unsigned long long)fence->seqno, strerror(-r));
   return r == 0;
}

/* ------------------------------------------------------------------ screen */

static void
fg_screen_destroy(struct pipe_screen *pscreen)
{
   struct fg_screen *screen = fg_screen(pscreen);

   close(screen->fd);
   free(screen->gems);
   simple_mtx_destroy(&screen->gem_lock);
   FREE(screen);
}

static int
fg_screen_get_fd(struct pipe_screen *pscreen)
{
   return fg_screen(pscreen)->fd;
}

static const char *
fg_screen_get_name(struct pipe_screen *pscreen)
{
   return "fpgagpu";
}

static const char *
fg_screen_get_vendor(struct pipe_screen *pscreen)
{
   return "fpgagpu";
}

static const char *
fg_screen_get_device_vendor(struct pipe_screen *pscreen)
{
   return "fpgagpu";
}

static struct disk_cache *
fg_screen_get_disk_shader_cache(struct pipe_screen *pscreen)
{
   return NULL;
}

static void
fg_screen_flush_frontbuffer(struct pipe_screen *pscreen, struct pipe_context *pctx, struct pipe_resource *prsc,
                            unsigned level, unsigned layer, void *winsys_drawable_handle, unsigned nboxes,
                            struct pipe_box *box)
{
}

static bool
fg_screen_is_format_supported(struct pipe_screen *pscreen, enum pipe_format format, enum pipe_texture_target target,
                              unsigned sample_count, unsigned storage_sample_count, unsigned usage)
{
   const unsigned image_binds = PIPE_BIND_RENDER_TARGET | PIPE_BIND_DEPTH_STENCIL | PIPE_BIND_SAMPLER_VIEW |
                                PIPE_BIND_SAMPLER_VIEW_SUBOPTIMAL | PIPE_BIND_BLENDABLE | PIPE_BIND_DISPLAY_TARGET |
                                PIPE_BIND_SCANOUT | PIPE_BIND_SHARED | PIPE_BIND_LINEAR;
   const unsigned colour_binds = PIPE_BIND_RENDER_TARGET | PIPE_BIND_SAMPLER_VIEW | PIPE_BIND_BLENDABLE |
                                 PIPE_BIND_DISPLAY_TARGET | PIPE_BIND_SCANOUT | PIPE_BIND_SHARED;
   const struct fg_format *f;

   if (MAX2(1, sample_count) != MAX2(1, storage_sample_count) || sample_count > 1)
      return false;

   if (target == PIPE_BUFFER) {
      if (usage & ~(PIPE_BIND_VERTEX_BUFFER | PIPE_BIND_INDEX_BUFFER | PIPE_BIND_CONSTANT_BUFFER))
         return false;
      if ((usage & PIPE_BIND_VERTEX_BUFFER) && !fg_vertex_format_supported(format))
         return false;
      if ((usage & PIPE_BIND_INDEX_BUFFER) && format != PIPE_FORMAT_R8_UINT && format != PIPE_FORMAT_R16_UINT)
         return false;
      return true;
   }

   switch (target) {
   case PIPE_TEXTURE_1D:
   case PIPE_TEXTURE_2D:
   case PIPE_TEXTURE_RECT:
   case PIPE_TEXTURE_CUBE:
      break;
   default:
      return false;
   }

   f = fg_format_info(format);
   if (!f || (usage & ~image_binds))
      return false;
   if (f->depth)
      return !(usage & colour_binds);
   if (!fg_format_renderable(f) &&
       (usage & (PIPE_BIND_RENDER_TARGET | PIPE_BIND_BLENDABLE | PIPE_BIND_DISPLAY_TARGET | PIPE_BIND_SCANOUT)))
      return false; /* board B renders into RGBA8 and RGB565 only */
   return !(usage & PIPE_BIND_DEPTH_STENCIL);
}

/* NIR options: the cores compute in floating point (no integer unit); SIN / COS are lowered on a core whose STATUS
 * flags say it has none. The vertex core reads a uniform at a computed index (LDPX); nothing else is indexed at
 * run time */
#define FG_NIR_COMMON                                                                       \
   .lower_fpow = true,                                                                      \
   .lower_fdiv = true,                                                                      \
   .lower_fmod = true,                                                                      \
   .lower_flrp32 = true,                                                                    \
   .lower_flrp64 = true,                                                                    \
   .lower_fdph = true,                                                                      \
   .lower_fsign = true,                                                                     \
   .lower_ffract = true,                                                                    \
   .max_varying_expression_cost = 2,                                                        \
   .lower_bitops = true,                                                                    \
   .lower_insert_byte = true,                                                               \
   .lower_insert_word = true,                                                               \
   .force_indirect_unrolling_sampler = true,                                                \
   .max_unroll_iterations = 32,                                                             \
   .no_integers = true

#define FG_NIR_VS FG_NIR_COMMON, .force_indirect_unrolling = (nir_var_shader_in | nir_var_shader_out | nir_var_function_temp)
#define FG_NIR_FS FG_NIR_COMMON, .force_indirect_unrolling = nir_var_all

static const nir_shader_compiler_options fg_vs_options = { FG_NIR_VS };
static const nir_shader_compiler_options fg_vs_options_nosincos = { FG_NIR_VS, .lower_sincos = true };
static const nir_shader_compiler_options fg_fs_options = { FG_NIR_FS };
static const nir_shader_compiler_options fg_fs_options_nosincos = { FG_NIR_FS, .lower_sincos = true };

#define FG_FLAG_NO_SINCOS (1u << 31)

static void
fg_init_shader_caps(struct fg_screen *screen)
{
   struct pipe_shader_caps *vs = (struct pipe_shader_caps *)&screen->base.shader_caps[MESA_SHADER_VERTEX];
   struct pipe_shader_caps *fs = (struct pipe_shader_caps *)&screen->base.shader_caps[MESA_SHADER_FRAGMENT];

   vs->max_instructions = vs->max_alu_instructions = 4096;
   vs->max_control_flow_depth = 16;
   vs->max_inputs = 16;
   vs->max_outputs = 16;
   vs->max_const_buffer0_size = 256 * 16;
   vs->max_const_buffers = 1;
   vs->max_temps = 32;
   vs->indirect_const_addr = true;
   vs->supported_irs = 1 << PIPE_SHADER_IR_NIR;

   fs->max_instructions = fs->max_alu_instructions = fs->max_tex_instructions = fs->max_tex_indirections = 4096;
   fs->max_control_flow_depth = 16;
   fs->max_inputs = 8;
   fs->max_outputs = 1;
   fs->max_const_buffer0_size = 64 * 16;
   fs->max_const_buffers = 1;
   fs->max_temps = 32;
   fs->max_texture_samplers = fs->max_sampler_views = 8;
   fs->supported_irs = 1 << PIPE_SHADER_IR_NIR;

   screen->base.nir_options[MESA_SHADER_VERTEX] =
      (screen->a_flags & FG_FLAG_NO_SINCOS) ? &fg_vs_options_nosincos : &fg_vs_options;
   screen->base.nir_options[MESA_SHADER_FRAGMENT] =
      (screen->b_flags & FG_FLAG_NO_SINCOS) ? &fg_fs_options_nosincos : &fg_fs_options;
}

static void
fg_init_screen_caps(struct fg_screen *screen)
{
   struct pipe_caps *caps = (struct pipe_caps *)&screen->base.caps;

   u_init_pipe_screen_caps(&screen->base, 1);

   /* OpenGL ES 2.0 */
   caps->npot_textures = true;
   caps->blend_equation_separate = true;
   caps->glsl_feature_level = 110;
   caps->glsl_feature_level_compatibility = 110;
   caps->max_render_targets = 1;
   caps->max_dual_source_render_targets = 0;
   caps->max_texture_2d_size = 2048;
   caps->max_texture_cube_levels = 12;
   caps->max_texture_3d_levels = 0;
   caps->max_texture_array_layers = 0;
   caps->max_varyings = 8;
   caps->max_viewports = 1;

   caps->fs_coord_origin_upper_left = true;
   caps->fs_coord_pixel_center_half_integer = true;

   /* left to the frontend: fixed-function state ES 2.0 does not have */
   caps->flatshade = false;
   caps->alpha_test = false;
   caps->two_sided_color = false;
   caps->clip_planes = 0;
   caps->texture_shadow_map = false;
   caps->native_fp32_depth = false;

   /* texture transfers map the image (READ_IMAGE / WRITE_IMAGE), they are not blits */
   caps->texture_transfer_modes = 0;

   /* board B's memory is not host memory. Its BOs travel between users of this device as dma-bufs (DRM PRIME): without
    * the export bit GBM gives every surface a host-memory dumb buffer instead of an image; imports are how X and its
    * clients share buffers (DRI3) */
   caps->uma = false;
   caps->dmabuf = DRM_PRIME_CAP_IMPORT | DRM_PRIME_CAP_EXPORT;

   caps->min_line_width = caps->min_line_width_aa = 1.0f;
   caps->max_line_width = caps->max_line_width_aa = 64.0f;
   caps->line_width_granularity = 1.0f;
   caps->min_point_size = caps->min_point_size_aa = 1.0f;
   caps->max_point_size = caps->max_point_size_aa = 1.0f;
   caps->point_size_granularity = 1.0f;

   caps->device_type = PIPE_DEVICE_TYPE_DISCRETE_GPU;
   caps->video_memory = 0;
}

struct pipe_screen *
fg_screen_create(int fd, const struct pipe_screen_config *config, struct renderonly *ro)
{
   struct fg_screen *screen = CALLOC_STRUCT(fg_screen);
   drmVersionPtr version;
   bool ours;
   int r;

   if (!screen) {
      close(fd);
      return NULL;
   }
   screen->fd = fd;
   simple_mtx_init(&screen->gem_lock, mtx_plain);

   version = drmGetVersion(fd);
   ours = version && version->name && strcmp(version->name, "fpgagpu") == 0;
   drmFreeVersion(version);
   if (!ours)
      goto fail;

   if ((r = fg_get_param(fd, FPGAGPU_PARAM_A_FLAGS, &screen->a_flags)) ||
       (r = fg_get_param(fd, FPGAGPU_PARAM_B_FLAGS, &screen->b_flags)) ||
       (r = fg_get_param(fd, FPGAGPU_PARAM_LINK_FLAGS, &screen->link_flags)) ||
       (r = fg_get_param(fd, FPGAGPU_PARAM_A_BUILD, &screen->a_build))) {
      fprintf(stderr, "fpgagpu: GET_PARAM failed: %s\n", strerror(-r));
      goto fail;
   }
   if (!(screen->link_flags & 1))
      fprintf(stderr, "fpgagpu: board B is not alive (link flags 0x%x)\n", screen->link_flags);
   screen->debug = debug_get_bool_option("FPGAGPU_DEBUG", false);
   screen->debug_shaders = debug_get_num_option("FPGAGPU_SHADERS", 0);
   if (screen->debug)
      fprintf(stderr, "fpgagpu: board A build %u, A flags 0x%08x, B flags 0x%08x, link flags 0x%x\n",
              screen->a_build, screen->a_flags, screen->b_flags, screen->link_flags);

   screen->base.destroy = fg_screen_destroy;
   screen->base.get_screen_fd = fg_screen_get_fd;
   screen->base.get_name = fg_screen_get_name;
   screen->base.get_vendor = fg_screen_get_vendor;
   screen->base.get_device_vendor = fg_screen_get_device_vendor;
   screen->base.get_timestamp = u_default_get_timestamp;
   screen->base.get_disk_shader_cache = fg_screen_get_disk_shader_cache;
   screen->base.context_create = fg_context_create;
   screen->base.is_format_supported = fg_screen_is_format_supported;
   screen->base.flush_frontbuffer = fg_screen_flush_frontbuffer;
   screen->base.fence_reference = fg_fence_reference;
   screen->base.fence_finish = fg_fence_finish;
   fg_resource_screen_init(&screen->base);

   /* shader caps first: the screen caps derive the constant buffer size from them */
   fg_init_shader_caps(screen);
   fg_init_screen_caps(screen);

   return &screen->base;

fail:
   close(fd);
   FREE(screen);
   return NULL;
}
