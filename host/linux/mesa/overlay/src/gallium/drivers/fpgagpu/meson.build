# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Dmytro Ivanysh

files_fpgagpu = files(
  'fg_compiler.c',
  'fg_compiler.h',
  'fg_context.c',
  'fg_context.h',
  'fg_draw.c',
  'fg_resource.c',
  'fg_resource.h',
  'fg_screen.c',
  'fg_screen.h',
  # the back end of the Windows libraries' compiler (host/win/shc), copied in by apply.sh
  'shc/shc_asm.c',
  'shc/shc_opt.c',
  'shc/shc_util.c',
)

libfpgagpu = static_library(
  'fpgagpu',
  files_fpgagpu,
  include_directories : [
    inc_src, inc_include, inc_gallium, inc_gallium_aux, inc_gallium_drivers, include_directories('shc'),
  ],
  gnu_symbol_visibility : 'hidden',
  dependencies : [dep_libdrm, idep_nir_headers, idep_mesautil],
)

driver_fpgagpu = declare_dependency(
  compile_args : '-DGALLIUM_FPGAGPU',
  link_with : [libfpgagpu, libfpgagpuwinsys],
  dependencies : idep_nir,
)
