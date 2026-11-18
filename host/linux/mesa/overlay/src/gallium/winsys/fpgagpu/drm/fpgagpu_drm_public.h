/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026 Dmytro Ivanysh */
/*
 * fpgagpu: the screen of a DRM device whose kernel driver is fpgagpu.
 */
#ifndef FPGAGPU_DRM_PUBLIC_H
#define FPGAGPU_DRM_PUBLIC_H

struct pipe_screen;
struct pipe_screen_config;

struct pipe_screen *fpgagpu_drm_screen_create(int fd, const struct pipe_screen_config *config);

#endif /* FPGAGPU_DRM_PUBLIC_H */
