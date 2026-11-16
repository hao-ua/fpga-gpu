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

Tested on the boards (`../tools/fg_egl_clear.c`, `fg_egl_draw.c`, `fg_egl_gl.c`): an OpenGL ES 2.0 context and a
desktop OpenGL 1.3 compatibility context on the device, chosen by the loader from the kernel driver's name (no
`MESA_LOADER_DRIVER_OVERRIDE`); vertex and fragment shaders compiled for the two cores, fixed function included;
triangles with vertices streamed into a ring on board A, `glDrawElements` with 32-bit indices far from 0 (rebased to
16 bits), quads through Gallium's primitive converter, interpolated varyings, textures (RGBA, RGB; nearest; mip
levels uploaded one by one, the chain grown with `COPY_IMAGE`), multitexture, alpha blending, alpha test, depth test
with `glDepthRange`; render passes ending in a FLUSH with the hardware's clear; reads of images (`glReadPixels`)
through `READ`, writes (`glTexImage2D`) through `WRITE_IMAGE`. L8, A8 and LA88 are stored as RGBA8 and converted on
the PC (board B has no such storage).

Also tested: face culling, Quake 3's sky state (`glDepthRange(1, 1)`), rendering into a texture whose tight rows are
no multiple of 32 bytes (board B renders into a shadow image with an aligned pitch, copied back after the pass), X's
glamor (OpenGL ES 2.0: A8, R8 and RG8 render targets kept as RGBA8 with masked writes; text, whose glyphs glamor
keeps in R8 textures read through `GL_TEXTURE_SWIZZLE_A = GL_RED`), texture swizzle (the driver applies a sampler
view's swizzle after the TEX; `apply.sh` lets GLES 2.0 contexts set it), and dma-buf import for DRI3 (BO_INFO). Every render pass holds the boards' render state for its process (the kernel's PASS ioctl), so X and a
game can share the boards.

Written, not tested yet: points and lines, stencil, scissor, polygon offset, cube maps, linear filtering, clears
inside a pass drawn as quads, draws whose indices span more than 16 bits.

OpenGL 2.0 with GLSL 1.10, the version the hardware supports (as the Windows `opengl32`): `apply.sh` keeps a
2.0 context's GLSL at the driver's 1.10 (Mesa would raise it to 1.20, which is compiler work beyond the hardware) and
gives the version string for it. ARB_shadow, the one thing Mesa missed for 2.0, is done in the fragment program: the
TMU reads a D24S8 image through an RGBA8 alias of its bytes (BO_CREATE with FPGAGPU_BO_CREATE_ALIAS; the TMU has no
D24S8 format) and the program decodes the depth and compares it with the clamped reference by the sampler's function
(`fg_egl_gl` tests 9 and 10). Such a depth texture is sampled right only when its rows are a multiple of 32 bytes
(a width a multiple of 8): a depth target's pitch is aligned, and the TMU has none. ioquake3's `opengl2` renderer
writes GLSL 1.20 and is refused; its `opengl1` renderer is the one for these boards.

Blits (`glBlitFramebuffer`) are board B's COPY_IMAGE: images of the same storage, a nearest stretch, mirrors, the
scissor when nothing stretches (`fg_egl_gl` test 8); blended blits and blits between storage formats are not done.

Not yet: copies between buffers and images, mipmap generation, queries, instancing, importing buffers,
rendering into a level or layer above 0 or a target wider than 1280, more than one context drawing at a time.

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

On Debian 13 use `-Dplatforms=x11`: its `wayland-scanner` rejects the Wayland protocols Mesa 26.3 takes as a
subproject. Stop an X server that runs on `/opt/fpgagpu-mesa` before `ninja install`, which rewrites the libraries it
has mapped.

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
triangle (with a map of the target), a triangle with interpolated colours, a 2 x 2 texture over the target, a
blend of white at alpha 0.5 over black, and a 2 x 2 R8 texture whose alpha is swizzled from red (as glamor reads
glyphs). `host/linux/tools/fg_egl_gl.c` (linked with `-lGL` instead of
`-lGLESv2`): desktop OpenGL with fixed function the way Quake 3 engines draw: `glBegin(GL_QUADS)`, client arrays
with `glDrawElements(GL_UNSIGNED_INT)` and two texture units, alpha test, depth test with `glDepthRange`, a mip chain
uploaded level by level.

`host/linux/tools/fg_kms_flip.c`: a GBM surface on the card (DRM master needed, no X on it), an EGL window surface,
frames cleared and flipped onto board B's HDMI with KMS the way SDL's kmsdrm driver runs a game; the buffers are
board-B images, shown without a copy.

`FPGAGPU_DEBUG=1` prints the boards' flags when the screen is created, every image created to be shown, any image
refused, what a framebuffer without a colour buffer holds and, for each read map, the time spent on the submission,
the wait and the `READ`. `FPGAGPU_SHADERS=1` prints each compiled program's assembler text.
