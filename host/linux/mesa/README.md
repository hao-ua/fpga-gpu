# fpgagpu Mesa driver (overlay)

`overlay/` is a Gallium driver, `fpgagpu`, for Mesa 26.3 (written against the tag `mesa-26.3.0`). It runs on the
`fpgagpu` DRM kernel driver (`../kernel`) and reaches the boards only through its ioctls.

- `overlay/src/gallium/drivers/fpgagpu/`: screen (caps, formats, fences), context (the command stream, render
  passes, clears, flushes), draws (state objects, shaders, a draw's messages), the shader compiler (`fg_compiler`),
  resources (board-B images through `BO_CREATE`, transfers through `READ` / `WRITE_IMAGE`).
- The compiler turns NIR into the assembler lines of the Windows libraries' compiler and runs that compiler's back
  end (optimizer, register allocator, assembler: `host/win/shc/shc_opt.c`, `shc_asm.c`, `shc_util.c`), which
  `apply.sh` copies into the driver's `shc/` directory.
- `overlay/src/gallium/winsys/fpgagpu/drm/`: one screen per DRM file description.
- `overlay/include/drm-uapi/fpgagpu_drm.h`: a copy of `../kernel/include/uapi/drm/fpgagpu_drm.h`; keep the two
  identical.
- `apply.sh <mesa-tree>`: copies the overlay into a Mesa tree and registers the driver: the `gallium-drivers`
  choice, `with_gallium_fpgagpu`, the `src/gallium` subdirectories, the DRI target's dependency, the
  `drm_helper.h` descriptor (before kmsro), the pipe loader's table (before kmsro, else the device falls back to
  kmsro), the dril entry point. Running it again changes nothing; it stops if an anchor line is missing.

Tested on the boards (`../tools/fg_egl_clear.c`, `../tools/fg_egl_draw.c`): an OpenGL ES 2.0 context on the device,
chosen by the loader from the kernel driver's name (no `MESA_LOADER_DRIVER_OVERRIDE`); vertex and fragment shaders
compiled for the two cores; triangles with vertices streamed into a ring on board A, interpolated varyings, a
nearest-filtered 2D texture, alpha blending; render passes ending in a FLUSH with the hardware's clear; reads of
images (`glReadPixels`) through `READ`, writes (`glTexImage2D`) through `WRITE_IMAGE`.

Written, not tested yet: indexed draws (16-bit indices in the ring), points and lines, quads and polygons through
Gallium's primitive converter, depth, stencil, scissor, polygon offset, culling, cube maps, mipmaps, linear filtering,
clears inside a pass drawn as quads.

Not yet: blits, image copies, mipmap generation, queries, instancing, importing buffers, rendering into a level or
layer above 0 or a target wider than 1280, more than one context drawing at a time.

## Apply and build

```sh
sh host/linux/mesa/apply.sh /root/fpgagpu/src/mesa
cd /root/fpgagpu/src/mesa
meson setup /root/fpgagpu/build/mesa-fg -Dprefix=/opt/fpgagpu-mesa -Dbuildtype=debugoptimized \
  -Dgallium-drivers=fpgagpu,softpipe -Dvulkan-drivers= -Dllvm=disabled -Dplatforms=x11,wayland -Degl=enabled \
  -Dgbm=enabled -Dglx=dri -Dgles2=enabled -Dvideo-codecs= -Dtools=
nice -n 19 ionice -c3 ninja -C /root/fpgagpu/build/mesa-fg -j2
ninja -C /root/fpgagpu/build/mesa-fg install
```

## Test

`host/linux/tools/fg_egl_clear.c`: GBM + EGL (no surface) on `/dev/dri/renderD129`, an ES 2.0 context, a 64 x 64
RGBA8 framebuffer object, two clears read back and checked pixel by pixel, then a texture uploaded with
`glTexImage2D` and read back through a framebuffer object; it fails if the context is not on the fpgagpu driver.

```sh
L=/opt/fpgagpu-mesa/lib/x86_64-linux-gnu
cc -O2 -Wall -I/opt/fpgagpu-mesa/include host/linux/tools/fg_egl_clear.c -o fg_egl_clear \
  -L$L -Wl,-rpath-link,$L -lEGL -lGLESv2 -lgbm $(pkg-config --cflags --libs libdrm)
LD_LIBRARY_PATH=$L ./fg_egl_clear
```

`host/linux/tools/fg_egl_draw.c` (built the same way): draws in the same context, read back and checked: a solid
triangle (with a map of the target), a triangle with interpolated colours, a 2 x 2 texture over the target, and a
blend of white at alpha 0.5 over black.

`FPGAGPU_DEBUG=1` prints the boards' flags when the screen is created and, for each read map, the time spent on the
submission, the wait and the `READ`. `FPGAGPU_SHADERS=1` prints each compiled program's assembler text.
