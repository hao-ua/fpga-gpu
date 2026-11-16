# fpgagpu Mesa driver (overlay)

`overlay/` is a Gallium driver, `fpgagpu`, for Mesa 26.3 (written against the tag `mesa-26.3.0`). It runs on the
`fpgagpu` DRM kernel driver (`../kernel`) and reaches the boards only through its ioctls.

- `overlay/src/gallium/drivers/fpgagpu/`: screen (caps, formats, fences), context (state, clears, flushes),
  resources (board-B images through `BO_CREATE`, transfers through `READ` / `WRITE_IMAGE`).
- `overlay/src/gallium/winsys/fpgagpu/drm/`: one screen per DRM file description.
- `overlay/include/drm-uapi/fpgagpu_drm.h`: a copy of `../kernel/include/uapi/drm/fpgagpu_drm.h`; keep the two
  identical.
- `apply.sh <mesa-tree>`: copies the overlay into a Mesa tree and registers the driver: the `gallium-drivers`
  choice, `with_gallium_fpgagpu`, the `src/gallium` subdirectories, the DRI target's dependency, the
  `drm_helper.h` descriptor (before kmsro), the pipe loader's table (before kmsro, else the device falls back to
  kmsro), the dril entry point. Running it again changes nothing; it stops if an anchor line is missing.

What works: an OpenGL ES 2.0 context on the device, chosen by the loader from the kernel driver's name (no
`MESA_LOADER_DRIVER_OVERRIDE`); `glClear` of the bound framebuffer (colour, depth, stencil) sent as one submission
`FRAME_BEGIN` + `VS_STATE` (the target's size) + `FLUSH` with the clear; reads of images (`glReadPixels`) through
`READ`, and writes (`glTexImage2D` / `glTexSubImage2D`) through `WRITE_IMAGE`. Not yet: draws (dropped, one note on stderr), blits, copies, mipmap generation, shader
compilation (shaders are only kept), importing / exporting buffers, clears of a level or layer above 0 or of a target
wider than 1280.

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

`FPGAGPU_DEBUG=1` prints the boards' flags when the screen is created and, for each read map, the time spent on the
submission, the wait and the `READ`.
