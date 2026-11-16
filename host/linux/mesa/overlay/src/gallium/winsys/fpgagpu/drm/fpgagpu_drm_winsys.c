/* SPDX-License-Identifier: MIT */
/*
 * fpgagpu: one screen per DRM file description; the screen keeps its own duplicate of the fd.
 */
#include "util/os_file.h"
#include "util/u_screen.h"

#include "fpgagpu_drm_public.h"

#include "fpgagpu/fg_screen.h"

struct pipe_screen *
fpgagpu_drm_screen_create(int fd, const struct pipe_screen_config *config)
{
   return u_pipe_screen_lookup_or_create(os_dupfd_cloexec(fd), config, NULL, fg_screen_create);
}
