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
desktop OpenGL 2.0 compatibility context on the device, chosen by the loader from the kernel driver's name (no
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
(`fg_egl_gl` tests 9 and 10). ioquake3's `opengl2` renderer writes GLSL 1.20 and is refused; its `opengl1`
renderer is the one for these boards.

Pitches: the TMU's descriptor has none (it reads tight rows), and board B renders into and shows images whose pitch
is a multiple of 32 bytes. A texture that is also rendered into keeps tight rows and is rendered in an aligned shadow,
copied back after the pass. The other way round, an image with padded rows that is sampled (a DRI3 or scan-out buffer
X draws a window from, a depth texture whose width is no multiple of 8) is read from a copy with tight rows, taken
before a pass that samples it when the image changed, and in every such pass for an image shared with another process
(`fg_egl_gl` test 11). Without it glxgears' 300-pixel window came out smeared sideways, each row 4 pixels further on.

Region passes: a pass costs board B about 33 ns for every pixel of the image its FLUSH names, whatever it draws
(board A walks every band, board B loads and stores each): 16 ms on an 800 x 600 screen. A pass into a target without
a depth-stencil image (and of at least 128 x 128) therefore renders into the rows it reaches only: the FLUSH names an
alias of those rows (BO_CREATE's alias at `alias_offset`, cached per image), board A gets their count as the target
size, and every coordinate of the pass (viewport, scissor, the draw's rectangle, the clear's scissor) is relative to
the first row. Whole rows, because board B writes each line of a FLUSH's image to its pitch. What a draw reaches is
its viewport and scissor, narrowed to its vertices' window positions where the vertex program's position can be
worked out on the CPU (`fg_eval`: the position's slice of the SSA form, kept at compile time; up to 2048 vertices a
draw). The first draw fixes the first row (a multiple of 8); a draw or clear above it ends the pass; after four such
passes in a row the next covers the whole target. `FPGAGPU_NOREGION=1` turns region passes off. A one-quad pass on
800 x 600 costs 0.35 ms instead of 16 (`fg_egl_gl` test 13).

Draw states: board B keeps 8191 of a context's draw states by id. When they run out the pass ends and the ids start
again, before the next draw sends anything, with the images the draw reads on the next pass's list (`fg_egl_gl` test
12).

Blits (`glBlitFramebuffer`) are board B's COPY_IMAGE: images of the same storage, a nearest stretch, mirrors, the
scissor when nothing stretches (`fg_egl_gl` test 8); blended blits and blits between storage formats are not done.

Copies: a draw whose fragment program samples one 2D image at a varying and writes the texel as its colour, and does
nothing else (`fract()` of the coordinate allowed: a repeating tile), is board B's COPY_IMAGE when the varying runs one
to one with the window position. The program is recognised once (`fg_fs_copy`, from its last SSA form). Per draw, the
vertex program's position and that varying are worked out on the CPU for every vertex (`fg_eval`, up to 4096
vertices); the texel must sit at the same integer offset from the window position at every vertex, and the triangles
must pair into axis-aligned rectangles. Each rectangle, clipped to the scissor, viewport and target, is one COPY_IMAGE
(a tile that wraps: one per piece, at most 64 a draw), which gives what GL defines: the texel under each pixel's
centre. A copy into an image whose alpha the board keeps at 1.0 (an X format), or of a texel read with alpha 1, has
its alpha set by a FILL_IMAGE of the alpha channel. Drawn as before: blending, a colour mask without one of the image's
channels, depth or stencil tests, culling, clip planes, a compare or mip filtering in the sampler, swizzles other
than alpha 1, images of different storage, texels past the edge of a clamped image, and a draw into the target of an
open pass of the whole target that has draws (ending it for the copy would cost a frame with depth a second pass).
This is how the X server copies (glamor's copy and tile programs): 30 moves of a 604 x 394 xterm took 279 ms until
the boards were done instead of 546, of a 364 x 264 one 151 ms instead of 264 (`fg_egl_gl` test 14).
`FPGAGPU_NOCOPY=1` turns it off; `FPGAGPU_DEBUG_COPY=1` prints each copy, and why a copying draw was drawn.

Reading the target: GL_NV_texture_barrier and GL_MESA_tile_raster_order are offered, so the X server copies a window
inside the screen image directly instead of through a temporary image. A barrier ends the pass. A copy inside one
image is exact where its rectangles overlap (board B goes bottom-up for a copy down and reads a row before writing
it), and the rectangles of one draw go in the move's direction, the far rows first. A draw that is no copy and samples
its own target reads a copy of it taken once a pass, before the pass renders (the tight-row copy's mechanism), since
board B rewrites the target band by band while the pass's fragments read it; so any raster order gives what the
extension asks (`fg_egl_gl` test 14).

Flushes: a flush that submitted work waits for the one four flushes back (`FPGAGPU_FLUSH_DEPTH`, 0 = no wait), so a
process that makes work faster than the boards do it does not queue it ever deeper (a dragged window trailed the
pointer by ~130 ms; now ~15 ms). A flush at the end of a frame also waits for the frame before (one frame in flight).

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
