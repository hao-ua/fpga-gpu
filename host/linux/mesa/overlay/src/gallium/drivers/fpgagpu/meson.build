# SPDX-License-Identifier: MIT

files_fpgagpu = files(
  'fg_context.c',
  'fg_context.h',
  'fg_resource.c',
  'fg_resource.h',
  'fg_screen.c',
  'fg_screen.h',
)

libfpgagpu = static_library(
  'fpgagpu',
  files_fpgagpu,
  include_directories : [
    inc_src, inc_include, inc_gallium, inc_gallium_aux, inc_gallium_drivers,
  ],
  gnu_symbol_visibility : 'hidden',
  dependencies : [dep_libdrm, idep_nir_headers, idep_mesautil],
)

driver_fpgagpu = declare_dependency(
  compile_args : '-DGALLIUM_FPGAGPU',
  link_with : [libfpgagpu, libfpgagpuwinsys],
  dependencies : idep_nir,
)
