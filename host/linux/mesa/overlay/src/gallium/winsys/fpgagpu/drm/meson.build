# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Dmytro Ivanysh

libfpgagpuwinsys = static_library(
  'fpgagpuwinsys',
  files('fpgagpu_drm_winsys.c'),
  include_directories : [
    inc_src, inc_include,
    inc_gallium, inc_gallium_aux, inc_gallium_drivers,
  ],
  dependencies : [dep_libdrm, idep_mesautil],
)
