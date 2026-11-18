# Linux driver

The GPU under Linux:
- **Kernel driver:** a DRM driver, `fpgagpu`, that talks to the boards over Ethernet and shows board B's HDMI output as
  a KMS display.
- **OpenGL:** a Mesa Gallium driver, also `fpgagpu`, that gives OpenGL ES 2.0, OpenGL 2.0, EGL, GLX and GBM through
  Mesa's front ends.

It drives the **two-board** design. On boards A 166 / B 162 dEQP-GLES2 passes 17 275 of its 17 775 cases, with 8
failures (section 6).

| directory | contents |
|---|---|
| `host/linux/kernel/` | the DRM / KMS kernel module, its userspace header, the DKMS files |
| `host/linux/mesa/` | the Gallium driver, an overlay on Mesa 26.3 with `apply.sh`. Its [README](host/linux/mesa/README.md) describes it in full and says how to build it. |
| `host/linux/x11/` | X server, display manager and udev configuration for X on the boards. Each file's header says where it goes. |
| `host/linux/tools/` | test programs (section 5) |

## 1. Decisions

| question | choice |
|---|---|
| kernel driver | a DRM driver, `fpgagpu`, with the board protocol's UDP transport inside the kernel. Packaged for DKMS; Linux 6.1 and later. |
| OpenGL | a Mesa Gallium driver, `fpgagpu`. Shaders go from NIR into the Windows libraries' compiler back end. |
| the board's HDMI | a KMS connector: the Linux console, X and other KMS clients (page flips) show on the board's monitor |
| licence | the kernel module's files are `GPL-2.0 OR MIT` (`MODULE_LICENSE("Dual MIT/GPL")`), so it can use DRM's GPL-only helpers. Each of its source files has the SPDX identifier on line 1 (the kernel's rule) and the copyright line after it. Its uapi header is `(GPL-2.0 WITH Linux-syscall-note) OR MIT`, and the Mesa driver's copy of it is identical. The Mesa driver is MIT, like the rest of the repository. Each of its files starts with the SPDX identifier and the copyright line, as Mesa's sources do. So do the compiler files `apply.sh` copies into it (`host/linux/mesa/README.md`, Licence). |

## 2. Layers

```
 GL / GLES / EGL / GLX / GBM applications          console (fbdev) · X · KMS clients
                  │                                                │ KMS atomic commits, page flips
 Mesa: st/mesa → gallium driver fpgagpu (NIR → isa64, state, batches)
                  │ libdrm: ioctls on the render node              │
 ─────────────────┼────────────────────────────────────────────────┼──────────────────────────── kernel
 fpgagpu.ko   GEM buffer objects (host memory, board memory) · SUBMIT · dma_fence / syncobj · KMS pipe
                  │ one UDP socket to 192.168.1.10:5000 (the protocol of host/fb/fbproto.py)
 board A ── board link ── board B ── HDMI
```

The kernel owns the only connection to the boards, and every process goes through it. That also settles the boards'
rule that they answer whoever spoke last: several GL programs and the desktop share one sender. While the module is
loaded it is that sender. The Windows libraries and the `host/fb` tools must not use the boards at the same time.

## 3. Kernel driver (`host/linux/kernel/`)

### 3.1 Device and parameters
- The boards are not on a bus the kernel probes. The module registers a platform device `fpgagpu` at load, and the DRM
  device sits on it: `/dev/dri/card*` (KMS, master) and `/dev/dri/renderD*` (rendering).
- Module parameters: `ip` (board A's address, 192.168.1.10), `port` (5000) and `min_a_build` (the oldest board A build
  accepted).
- `restarted` is a test parameter. Writing 1 or 3 to it makes the module act as if board A, or both boards, had
  restarted (section 3.8).
- At load the module sends HELLO and STATUS requests. It registers the DRM device once both boards answer (STATUS
  layout, link trained, B alive).
- Until then it stays loaded, greets board A again every 5 s and logs why. So it can load at boot, before the boards
  are on or the network is up.

### 3.2 Transport (`fpgagpu_net.c`)
A kernel port of `host/win/core/g2_core.c`, itself a port of `fbproto.Board`. It keeps the same structure, so
datagrams can be compared byte by byte with the Python reference.
- One UDP socket (`sock_create_kern`, `kernel_sendmsg` / `kernel_recvmsg`), a receive thread, and a 1 ms maintenance
  timer.
- Sequenced datagrams in a 64-entry window, credit from the ACKs' free queue space, NAK and the heartbeat rule, resend
  on timeout, BATCH packing.
- Replies: ACK, STATUS (the latest kept for userspace), EVENT (FENCE, PRESENT, MODE, ERROR) and READ_DATA reassembly.
- **Timers scaled to the round trip.** On board A's own network the timers are fbproto's:
  - the 2 ms heartbeat rule;
  - the 4 ms gap between resends;
  - the 5 ms wait after FULL;
  - the 10 ms stall timer.

  A longer path, such as `host/linux/tools/g2_relay.py` between two hosts, has a round trip of tens of milliseconds.
  So each timer is at least the smoothed round trip plus twice its variation, measured RFC 6298-style from datagrams
  acknowledged without a resend. A NAK or a heartbeat ACK that left the board before a resend arrived then causes no
  second one.
- The waits for room in the send window (in SUBMIT and in READ) end only on a fatal signal. An interrupted SUBMIT that
  started again would send its whole stream a second time.

### 3.3 Board memory and handles
- Two heaps with `drm_mm`: board A's [384 MB, 1 GB) and board B's [256 MB, 1 GB), the two boards' map.
- Handle tables: board A's 1024 buffers, board B's 1024 buffers and 1024 images. The kernel hands out the 16-bit
  handles, so processes never collide.
- Every image gets the memory of whole bands, its height rounded up to 8 rows. Board B loads and stores every row of
  a band, even past an image's last row. The size the kernel reports, and checks aliases against, stays the image's
  own.

### 3.4 Buffer objects (GEM)
- **Host-memory BOs** (`drm_gem_shmem`): CPU-visible. Dumb buffers, the fbdev console and upload staging.
- **Board BOs:** an allocation in one board heap plus a board handle.
  - On board A: a buffer (vertices, indices, programs).
  - On board B: a buffer or an image (textures, render targets, depth-stencil, programs).
  - The CPU cannot map them; their contents move with WRITE_* / READ_* through the kernel.
- Implicit sync through each BO's `dma_resv`; explicit sync through `drm_syncobj`.

### 3.5 Userspace interface (`include/uapi/drm/fpgagpu_drm.h`)
| ioctl | does |
|---|---|
| `GET_PARAM` | board builds, STATUS layout, the flags (absent units, extensions), the latest STATUS words |
| `GET_STATUS` | a fresh STATUS reply from the boards |
| `BO_CREATE` | a board BO: heap, size, kind (buffer / image) and the image descriptor (format, size, pitch, levels). Or an alias: another descriptor of an image's bytes, from an offset. |
| `BO_INFO` | a BO's board handle, address and size |
| `SUBMIT` | a stream of protocol messages that names BOs by GEM handle. The kernel checks every message's type, sizes and ranges, writes in the board handles and addresses, sends it in order and returns a syncobj signalled by the stream's FENCE. |
| `WRITE` / `READ` | copy between user memory and a board BO (WRITE_BUFFER / WRITE_IMAGE, READ_BUFFER / READ_IMAGE / MEM_READ) |
| `WAIT` | wait for a BO to be idle, with a timeout |
| `PASS` | BEGIN / END a render pass. The boards hold one render state (programs, attributes, uniforms, the frame, draw states), and one process holds it at a time. Another process's BEGIN waits, up to its timeout. BEGIN returns `state_lost`: 1 when another process held the boards since, 2 when a board restarted (BO contents gone too). |

Raw MEM_WRITE, MEMTEST, PHY and register messages are refused from userspace. A debugfs file shows the transport's
counters and the last STATUS.

### 3.6 Display (KMS)
- One CRTC, one primary plane, a cursor plane (the board's 64 × 64 cursor), one encoder and one HDMI-A connector,
  always connected.
- **Modes:** board B's, 640×480, 800×600 (preferred), 1024×768, 1280×720 and 1280×800 at 60 Hz. A modeset sends
  SET_MODE and completes on the MODE event.
- **Formats:** XRGB8888 (stored as RGBA8) and RGB565.
- **Scanout from a host-memory framebuffer** (dumb buffers, fbdev):
  - The plane's damage rectangles are uploaded with WRITE_IMAGE into a board-side scanout image, double-buffered, then
    PRESENTed.
  - Gigabit Ethernet moves about 110 MB/s: a whole 800×600 RGBA8 frame (1.9 MB) at about 55 frames a second at best,
    so damage tracking matters. Consoles and desktops mostly update small areas.
- **Scanout from a board BO** (GL rendering through GBM): PRESENT the BO's image directly, with no copy. The flip
  completes on the PRESENT event.
- **Cursor plane** (ARGB8888, up to 64 × 64): board B's hardware cursor.
  - A new cursor framebuffer sends its shape (CURSOR SHAPE, sequenced, premultiplied) and CURSOR SHOW.
  - A move sends only CURSOR_POS, unsequenced. Board A passes it to board B ahead of every queued command, so the
    pointer never waits behind rendering.
  - The last position goes once more 50 ms after the last move, in case one was lost.
  - The mode config's minimum framebuffer size is 1 × 1. DRM checks it for every framebuffer, and the 64 × 64 cursor's
    must pass too.
- **Vblank:** the boards send no vblank interrupt, so vblank is emulated with a 60 Hz hrtimer. Flips complete on the
  PRESENT event.
- **fbdev emulation** puts the Linux console on the board's monitor.

### 3.7 Kernel versions
The module builds for Linux 6.1 and for current kernels (compiled against 6.1, 6.12 and 7.3). Version checks in the
code cover the differences:
- `platform_driver.remove` returns void from 6.11;
- `<linux/unaligned.h>` arrived in 6.12;
- `drm_driver.date` was dropped in 6.14;
- fbdev emulation is `drm_fbdev_generic_setup` (with its header from 6.2), then `drm_fbdev_shmem_setup` (6.11), then
  `drm_client_setup` (6.13).

Newer kernels renamed the atomic helpers' `struct drm_atomic_state` to `struct drm_atomic_commit`. Kbuild asks the
kernel being built against (`FG_HAVE_ATOMIC_COMMIT`), so DKMS builds for any version.

### 3.8 Board restarts
**Detection:** the module asks for STATUS once a second.
- Board A restarted when its uptime is short of the last one plus the time since, or when the sequence number it
  expects is far behind the module's.
- Board B restarted when its frame count went back.

**Recovery** starts at once:
- a new session with board A (the wire's sequence numbers start at 1; the driver's fences go on);
- every buffer object on the restarted boards is created again (its contents are lost);
- after board B's restart, the display's mode is set again;
- a hotplug event (the console redraws) and a change uevent with `FPGAGPU_RESET=1`.

Until recovery is done, a request that is sending fails with EIO, new ones wait for it, and fences count as passed.
After a restart of board A alone, X keeps running: PASS's `state_lost` 2 tells Mesa to place its programs again.
After board B's restart, `x11/99-fpgagpu-display.rules` restarts a running display manager.

### 3.9 Files that close mid-frame
A file closed in the middle of a frame (a process killed while drawing) has its frame ended by the kernel. The kernel
sends a FLUSH to an image that does not exist: board A refuses it as a target, and board B drops the frame. Otherwise
board A would keep waiting for the rest of that frame and take no other work.

### 3.10 Build and install with DKMS
- Kbuild files plus `dkms.conf` (`PACKAGE_NAME=fpgagpu`, `AUTOINSTALL=yes`). DKMS builds the module against the
  running kernel and rebuilds it after every kernel update:
  ```
  sudo dkms add host/linux/kernel
  sudo dkms install fpgagpu/<version>
  sudo modprobe fpgagpu ip=192.168.1.10
  ```
- The module is not signed: with Secure Boot on, the kernel refuses it.
- To load it at boot:
  - `kernel/fpgagpu-load.conf` goes in `/etc/modules-load.d/fpgagpu.conf`;
  - its options go in `/etc/modprobe.d/fpgagpu.conf` (`options fpgagpu ip=192.168.1.10 port=5000`).
- The UAPI header is installed for libdrm and Mesa to build against.

## 4. Mesa Gallium driver (`host/linux/mesa/`)

- **How it is kept:** an overlay of new files in this repository. `apply.sh` copies it into a Mesa 26.3.0 tree,
  registers the driver with Mesa's build and loader, and patches a few of Mesa's own files (listed in its README).
  Mesa then builds with `-Dgallium-drivers=fpgagpu`.
- **Installing:** install it to its own prefix (`/opt/fpgagpu-mesa` in the README). To make it the system's GL, add a
  file in `/etc/ld.so.conf.d/` that names its library directory ahead of the system's, then run `ldconfig`. Without
  that the distribution's Mesa loads and answers "failed to load driver: fpgagpu".
- `src/gallium/winsys/fpgagpu/drm`: one screen per DRM file description, over the ioctls above.
- `src/gallium/drivers/fpgagpu`:
  - **screen:** capabilities from `GET_PARAM` (absent units, sizes, extensions), formats, fences;
  - **context:** the command stream, render passes (FRAME_BEGIN … DRAW … FLUSH, each held with PASS), clears and
    flushes;
  - **draws:** state objects mapped onto DRAW_STATE / VS_STATE / VERTEX_ATTR, as `host/win/core/g2_render.c` does for
    the Windows libraries;
  - **resources:** textures, render targets and depth-stencil as board-B images, transfers through `READ` /
    `WRITE_IMAGE`.
- The loader matches the DRM driver name `fpgagpu`, so EGL, GBM and GLX find it on the device node.

### 4.1 Shader compiler: NIR → isa64
- `fg_compiler` turns Mesa's NIR for both stages into the assembler lines of the Windows libraries' compiler.
- That compiler's back end makes the board's 64-bit instruction words: the optimizer, register allocator and
  assembler in `host/win/shc/shc_opt.c`, `shc_asm.c` and `shc_util.c`, which `apply.sh` copies into the driver.
- Where a core reports no unit in the STATUS flags, NIR lowers the missing operations, as the other compilers do:
  - bit 31: SIN / COS;
  - bit 30: integer division.

### 4.2 What it reports
- **OpenGL ES 2.0** with GLSL ES 1.00.
- **OpenGL 2.0** (compatibility profile) with GLSL 1.10, the version the hardware supports, as the Windows `opengl32`
  does:
  - `apply.sh` keeps a 2.0 context's GLSL at 1.10;
  - ARB_shadow is done in the fragment program;
  - occlusion queries answer whether a draw ran (the board counts no samples).
- **EGL** (GBM, X11, surfaceless), **GLX** (DRI3) and **GBM**; buffers shared between processes by dma-buf.
- Nothing is faked: only what the hardware or the PC can do exactly is exposed.
- ioquake3 games need `cl_renderer opengl1`. Their default renderer, `opengl2`, writes GLSL 1.20.

## 5. Test programs (`host/linux/tools/`)

| program | checks |
|---|---|
| `fpgagpu_status` | every `GET_PARAM` and a fresh `GET_STATUS` through the render node |
| `fpgagpu_selftest` | buffer objects, `SUBMIT`, `WAIT` and `READ` end to end. A board-B image filled and read back pixel by pixel, a board-A buffer written and read back byte-exact. `MEM_WRITE` refused with EPERM, a handle missing from the BO list refused with EACCES. |
| `fg_egl_clear` | an ES 2.0 context on GBM: clears and a texture upload, read back and checked |
| `fg_egl_draw` | ES 2.0 draws read back and checked: solid and interpolated triangles, a texture, blending, a swizzled R8 texture |
| `fg_egl_gl` | desktop OpenGL the way Quake 3 engines draw (fixed function, client arrays, two texture units, alpha and depth tests), and the driver's special paths. Each numbered test is described in the Mesa README. |
| `fg_egl_threads` | two threads drawing at once, each in a context of its own |
| `fg_kms_flip` | an EGL window surface on a GBM surface, flipped onto board B's HDMI with KMS, the way SDL's kmsdrm driver runs a game |
| `fg_egl_info` | the EGL extensions, and each context's version and extensions |

The Mesa README says how to build them.

## 6. Status

- **Conformance:** dEQP-GLES2 (VK-GL-CTS) on boards A 166 / B 162, 17 775 cases of info, capability, functional and
  accuracy. It ran on the x11_egl target, with pbuffers through X's fpgagpu device. The surfaceless target finds no
  device, since libdrm lists no platform device without device-tree data.
  - **17 275 pass.**
  - **492 not supported:** half-float textures; samplers in vertex programs (board A has no TMU); separate 16-bit
    depth and 8-bit stencil renderbuffers (board B keeps them in one D24S8 word).
  - **8 fail:** the projected mipmap cases. The TMU takes one level of detail per 2×2 quad, so a pixel or two at a
    level boundary come out a level off.
  - **No crash.**
- **Desktops:** X with glamor (OpenGL ES 2.0) runs on the boards in two ways:
  - as the display manager's server, through `x11/20-fpgagpu.conf` (tested with XFCE through LightDM, and GNOME on
    Xorg through GDM);
  - as a second server beside the machine's own, through `x11/start-x-fpgagpu.sh`.

  OpenArena runs as an X client (GLX through DRI3) and through SDL's kmsdrm.
- **Limits:**
  - **No vblank pacing:** the boards send no vblank, so X's Present does not pace windowed clients. glxgears presents
    as fast as the boards take it.
  - **GTK 4:** its programs draw in software (their GL renderer needs OpenGL 3.3 or GLES 3.0) and hand X whole
    windows to upload.
  - **XFCE:** turn its compositor off (`x11/xfwm4-no-compositing.xml`), or it repaints the screen through the boards
    for every change of any window.
  - **GDM:** it needs `WaylandEnable=false`, since the configuration here is X's.
  - **Wayland:** not tried. The driver was tested in builds with `-Dplatforms=x11`, which the Mesa README gives for
    systems whose `wayland-scanner` rejects the Wayland protocols Mesa 26.3 takes as a subproject.
  - **Board B restarting alone:** only exercised through the `restarted` parameter. Board A's restart was also tested
    for real, with a JTAG load while X ran.
- **Not supported:** the one-board build (its memory map and STATUS flags bit 29).
