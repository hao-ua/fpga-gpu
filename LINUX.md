# Linux driver: design

The GPU under Linux: a DRM kernel driver that talks to the boards over Ethernet and shows the board's HDMI output as a
KMS display, and a Mesa Gallium driver that gives OpenGL ES 2.0, OpenGL, EGL, GLX and GBM through Mesa's front ends.
This branch (`linux-driver`) is cut from master, so it drives the **two-board** design. Nothing here is built yet; this
document is the plan.

## 1. Decisions

| question | choice |
|---|---|
| where Linux runs | a Hyper-V VM on the PC (Debian 13, Hyper-V's Default Switch, section 6). Before it: the Linux host 192.168.53.1, a VPS reached over WireGuard, which reached the boards through a UDP relay on the PC. |
| kernel driver | a DRM driver, `fpgagpu`, with the board protocol's UDP transport inside the kernel; packaged for DKMS |
| OpenGL | a Mesa Gallium driver, `fpgagpu`, with a new shader-compiler back end from NIR to the board's ISA |
| the board's HDMI | a KMS connector: the Linux console, X and Wayland desktops and page flips show on the board's monitor |
| licence | the kernel module's files are `GPL-2.0 OR MIT` (`MODULE_LICENSE("Dual MIT/GPL")`), so it can use DRM's GPL-only helpers; the Mesa driver is MIT, like the rest of the repository |

## 2. Layers

```
 GL / GLES / EGL / GLX / GBM applications          console (fbdev) · X · Wayland compositors
                  │                                                │ KMS atomic commits, page flips
 Mesa: st/mesa → gallium driver fpgagpu (NIR → isa64, state, batches)
                  │ libdrm: ioctls on the render node              │
 ─────────────────┼────────────────────────────────────────────────┼──────────────────────────── kernel
 fpgagpu.ko   GEM buffer objects (host memory, board memory) · SUBMIT · dma_fence / syncobj · KMS pipe
                  │ one UDP socket to 192.168.1.10:5000 (the protocol of host/fb/fbproto.py)
 board A ── board link ── board B ── HDMI
```

The kernel owns the only connection to the boards. Every process goes through it, which also settles the boards' rule
that they answer whoever spoke last: several GL programs and the desktop share one sender.

## 3. Kernel driver (`host/linux/kernel/`)

### 3.1 Device and parameters
- The boards are not on a bus the kernel probes. The module registers a platform device `fpgagpu` at load, and the DRM
  device sits on it: `/dev/dri/card*` (KMS, master) and `/dev/dri/renderD*` (rendering).
- Module parameters: `ip` (192.168.1.10), `port` (5000), `min_a_build` (the oldest board A build accepted).
- At load it sends HELLO and STATUS requests. The DRM device is registered only once both boards answer (STATUS layout,
  link trained, B alive); otherwise the module retries in the background and logs why.

### 3.2 Transport (`fpgagpu_net.c`)
A kernel port of `host/win/core/g2_core.c`, itself a port of `fbproto.Board`; it keeps the same structure so datagrams
can be compared byte by byte with the Python reference.
- One UDP socket (`sock_create_kern`, `kernel_sendmsg` / `kernel_recvmsg`), a receive thread, and a 1 ms maintenance
  timer.
- Sequenced datagrams in a 64-entry window, credit from the ACKs' free queue space, NAK and the heartbeat rule, resend
  on timeout, BATCH packing.
- Replies: ACK, STATUS (the latest kept for userspace), EVENT (FENCE, PRESENT, MODE, ERROR) and READ_DATA reassembly.
- **Timers scaled to the round trip.** On board A's own network the timers are fbproto's: the 2 ms heartbeat rule,
  the 4 ms gap between resends, the 5 ms wait after FULL and the 10 ms stall timer. Through the relay the round trip
  is about 70 ms, so each timer is at least the smoothed round trip plus twice its variation, measured RFC 6298-style
  from datagrams acknowledged without a resend. A NAK or a heartbeat ACK that left the board before a resend arrived
  then causes no second one.

### 3.3 Board memory and handles
- Two heaps with `drm_mm`: board A's [384 MB, 1 GB) and board B's [256 MB, 1 GB), the two boards' map.
- Handle tables: board A's 1024 buffers, board B's 1024 buffers and 1024 images. The kernel hands out the 16-bit
  handles, so processes never collide.

### 3.4 Buffer objects (GEM)
- **Host-memory BOs** (`drm_gem_shmem`): CPU-visible. Dumb buffers, the fbdev console and upload staging.
- **Board BOs:** an allocation in one board heap plus a board handle. A buffer on board A (vertices, indices, programs),
  or a buffer or an image on board B (textures, render targets, depth-stencil, programs). They cannot be mapped by the
  CPU; their contents move with WRITE_* / READ_* through the kernel.
- Implicit sync through each BO's `dma_resv`; explicit sync through `drm_syncobj`.

### 3.5 Userspace interface (`include/uapi/drm/fpgagpu_drm.h`)
| ioctl | does |
|---|---|
| `GET_PARAM` | board builds, STATUS layout, the flags (absent units, extensions), the latest STATUS words |
| `BO_CREATE` | a board BO: heap, size, kind (buffer / image) and the image descriptor (format, size, pitch, levels) |
| `BO_INFO` | a BO's board handle, address and size |
| `SUBMIT` | a stream of protocol messages that names BOs by GEM handle. The kernel checks every message's type, sizes and ranges, writes in the board handles and addresses, sends it in order and returns a syncobj signalled by the stream's FENCE. |
| `WRITE` / `READ` | copy between user memory and a board BO (WRITE_BUFFER / WRITE_IMAGE, READ_BUFFER / READ_IMAGE / MEM_READ) |
| `WAIT` | wait for a BO to be idle, with a timeout |

Raw MEM_WRITE, MEMTEST, PHY and register messages are refused from userspace; a debugfs file shows the transport's
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
- **Cursor plane** (ARGB8888, up to 64 × 64): board B's hardware cursor. A new cursor framebuffer sends its shape
  (CURSOR SHAPE, sequenced, premultiplied) and CURSOR SHOW. A move sends only CURSOR_POS, unsequenced: board A passes
  it to board B ahead of every queued command, so the pointer never waits behind rendering. The last position goes
  once more 50 ms after the last move, in case one was lost. The mode config's minimum framebuffer size is 1 × 1:
  DRM checks it for every framebuffer, and a 640 × 480 minimum made it refuse X's 64 × 64 cursor, which then fell
  back to a software cursor drawn on the boards for every move.
- **Vblank:** the boards send no vblank interrupt, so vblank is emulated with a 60 Hz hrtimer. Flips complete on the
  PRESENT event.
- **fbdev emulation** (`drm_fbdev_shmem`) puts the Linux console on the board's monitor.

### 3.7 Build and install with DKMS
- Kbuild files plus `dkms.conf` (`PACKAGE_NAME=fpgagpu`, `AUTOINSTALL=yes`). DKMS builds the module against the
  running kernel and rebuilds it after every kernel update:
  ```
  sudo dkms add host/linux/kernel
  sudo dkms install fpgagpu/<version>
  sudo modprobe fpgagpu ip=192.168.1.10
  ```
- The UAPI header is installed for libdrm and Mesa to build against.

## 4. Mesa Gallium driver (`host/linux/mesa/`)

- Kept in this repository as an overlay of new files plus a short patch for Mesa's build and loader. A script applies it
  to a pinned Mesa release (the one the VM's distribution ships), and Mesa builds with `-Dgallium-drivers=fpgagpu`.
- `src/gallium/winsys/fpgagpu/drm`: BOs, SUBMIT, fences, over the ioctls above.
- `src/gallium/drivers/fpgagpu`:
  - **screen:** caps from `GET_PARAM` (absent units, sizes);
  - **context:** state objects mapped onto DRAW_STATE / VS_STATE / VERTEX_ATTR, as `host/win/core/g2_render.c` does
    for the Windows libraries;
  - **resources:** textures, render targets and depth-stencil as board BOs, with staging transfers;
  - **draws:** batched per framebuffer into FRAME_BEGIN … DRAW … FLUSH passes. Clears merge into the FLUSH, and a
    framebuffer change, read-back or `flush()` ends the pass;
  - **queries** and the rest.
- The loader matches the DRM driver name `fpgagpu`, so EGL, GBM and GLX find it on the device node.

### 4.1 Shader compiler: NIR → isa64
- A new back end inside the Mesa driver. It takes Mesa's NIR for both stages and produces the board's 64-bit
  instruction words.
- **Lowering in NIR:** scalarize where the micro-ops are scalar, and map inputs / outputs onto the board's attributes,
  varyings and outputs.
  - If the cores report no unit, SIN / COS and integer division are lowered as the existing compilers do, by STATUS
    flags bits 30 / 31.
  - Texture coordinates and LOD go to the TEX forms the board has.
- **Instruction selection, register allocation (32 vec4 registers a lane) and control flow** (IF / LOOP / BRA, per-lane
  BREAK) follow `tools/glslc.py`, the ISA's specification, and its C port `host/win/shc`. Its emitter and encoder are
  reused.
- Every shader is checked against the ISA model (`tools/isa64.py`) the way the GLSL compilers are.

### 4.2 What Mesa will report
- **OpenGL ES 2.0** is the firm target: everything it needs is in the hardware.
- **Desktop OpenGL 2.0** needs two features the hardware lacks, and Mesa requires both before it reports even
  GL 1.2 / 1.5:
  - **3D textures:** the Windows library samples them in shader code; the Gallium driver would do the same in its NIR
    back end.
  - **Occlusion queries:** the hardware has no counter. This needs a decision: count on the PC (not possible for board
    work), or a board feature.
  - Until both are settled, desktop GL comes out at whatever version Mesa computes from the caps, possibly 1.1.
- Nothing is faked: per the project's rule, only what the hardware or the PC can do exactly is exposed.

## 5. Phases and what proves each

| phase | content | proof |
|---|---|---|
| 1 | transport + DRM device + `GET_PARAM`, DKMS packaging | the module loads, both boards' STATUS readable through the ioctl and debugfs |
| 2 | KMS: modes, host-memory scanout with damage, fbdev, cursor | the Linux console on the board's monitor; `modetest`; X with the modesetting driver |
| 3 | board BOs, `SUBMIT`, fences, `WRITE` / `READ` | a libdrm test that clears a target, presents it and reads it back exactly (`fb_3d_test`'s frames as the reference) |
| 4 | Mesa winsys + Gallium skeleton | EGL / GBM: a clear and a present (`kmscube` reaching its first frame) |
| 5 | NIR back end, state, textures, draws | `kmscube`, `es2gears`, `glmark2-es2`, the piglit / dEQP GLES 2.0 subsets |
| 6 | desktops | Weston with its GLES 2.0 renderer; X with glamor if its GL needs are met |

## 6. Environment

- **The Linux host:** 192.168.53.1, a VPS reached over WireGuard.
  - Debian 12, kernel 6.1.0-45 (Debian 6.1.170-1). Its exact headers came from snapshot.debian.org, since the archive
    has moved on.
  - Build tools: DKMS 3.0, meson 1.7 and libdrm 2.4.123 (backports), gcc 12.
  - Sources in `/root/fpgagpu/src`: Mesa 26.3.0 (a shallow clone of the release tag) and Linux 7.3.4, which checks the
    driver against current DRM interfaces.
  - The driver must therefore work on 6.1 and on current kernels. Version checks in the code cover the differences:
    - `platform_driver.remove` returns void from 6.11;
    - `<linux/unaligned.h>` arrived in 6.12;
    - `drm_driver.date` was dropped in 6.14;
    - fbdev emulation is `drm_fbdev_generic_setup` (with its header from 6.2), then `drm_fbdev_shmem_setup` (6.11),
      then `drm_client_setup` (6.13).
  - Newer kernels renamed the atomic helpers' `struct drm_atomic_state` to `struct drm_atomic_commit`. Kbuild asks the
    kernel being built against (`FG_HAVE_ATOMIC_COMMIT`), so DKMS builds for any version.
  - Every change compiles against both 6.1.0-45 and 7.3.4.
- **The relay** (`host/linux/tools/g2_relay.py`). The VPS is not on board A's network, so two relay ends carry the
  protocol's datagrams unchanged:
  ```
  VPS:  python3 g2_relay.py remote --listen 192.168.53.1:5100 --allow 192.168.73.30 --local 127.0.0.1:5000
  PC:   python3 g2_relay.py pc --remote 192.168.53.1:5100 --board 192.168.1.10:5000      (in WSL)
  VPS:  modprobe fpgagpu ip=127.0.0.1
  ```
  - The PC end opens the flow with a keepalive every 2 s, so no inbound firewall rule is needed, and NAT on the way
    keeps the flow open.
  - Between the ends every datagram travels in pieces of at most 1200 bytes, put together again on the other side.
    The protocol's datagrams are up to 1472 bytes (WRITE_* data, READ_DATA chunks); the WireGuard path's MTU is 1420,
    and its IP fragments were all lost. The first load showed 39 000 large datagrams sent and none reaching the board.
  - The VPS end accepts that flow only from the PC's address, and serves programs on 127.0.0.1.
  - The round trip is 62–131 ms (about 70 ms), so the boards work but slowly: 64 datagrams per round trip is about
    1–2 MB/s.
  - Proven 2026-11-15: `fb_status.py --ip 127.0.0.1` on the VPS read both boards' STATUS (A 154, link trained, B alive).
- **The VM (since 2026-11-16):** Hyper-V generation 2 on the PC cabled to board A, Debian 13 (kernel 6.12) from
  Debian's generic cloud image, set up by cloud-init (the WSL ssh key, build tools, headers, DKMS), Secure Boot off
  for the unsigned module. It sits on Hyper-V's Default Switch (NAT): it reaches board A through Windows the way WSL
  does, round trip about 0.3 ms, with no change to the PC's network. `/etc/modprobe.d/fpgagpu.conf` holds
  `ip=192.168.1.10 port=5000`. WSL does not reach the Default Switch, Windows does: ssh from WSL goes through a
  Windows-side TCP pipe as its ProxyCommand.
  - Keyboard and mouse: Deskflow. The server runs on Windows, the client (`deskflow-client`, Debian's 1.22) in the VM's
    X, started by `start-x-fpgagpu.sh` with `DESKFLOW_SERVER=172.20.96.1` (the Default Switch's Windows address); the
    pointer crosses from the Windows screen onto board B's monitor and back. X itself opens no input devices.
- **One sender:** while the module or the relay is in use it is the boards' sender. The Windows libraries and the
  `host/fb` tools on the PC must not run at the same time; stop the relay first.
- **Not in scope yet:** the one-board build (its memory map and STATUS flags bit 29); a different heap layout.

## 7. Status

**On hardware, 2026-11-16** (VPS kernel 6.1.0-55 after an update; DKMS rebuilt the module for it by itself):
- `modprobe fpgagpu ip=127.0.0.1` through the relay: board A build 154, STATUS layout 12, link trained, B alive,
  round trip about 60 ms.
- `fpgagpu_status`: every `GET_PARAM` and `GET_STATUS` works.
- `fpgagpu_selftest` **passes**:
  - a board-B image created, filled with `SUBMIT`, fenced and read back (every pixel);
  - a board-A buffer written and read back byte-exact;
  - `MEM_WRITE` refused with EPERM, and a handle missing from the BO list refused with EACCES.
- KMS registers card1 with HDMI-A-1 connected and the five modes.
  - The VPS's X server (lightdm's Xorg) takes card1 as a GPU screen and holds DRM master.
  - X cannot be extended onto it: X there runs without glamor (llvmpipe), so its main screen cannot feed a second GPU
    (both providers are "Sink Output" only).
  - With lightdm stopped for a minute, `modetest -M fpgagpu -s 35:800x600` set the mode and held its pattern 30 s.
    Board B's STATUS then showed mode 1 (800×600) presenting image 1000 (the driver's scanout image), 0 underruns,
    0 mode errors, 0 command errors.
  - About 6 full screens (12.5 MB) crossed the relay in that minute: the pattern, and fbdev restoring its console
    whenever a client let go.

| phase | state |
|---|---|
| 1 | written: transport (HELLO, STATUS, the sequenced window, the resend rules, events), the DRM device with a render node, `GET_PARAM` / `GET_STATUS`, `fpgagpu_status` (the check), Kbuild and `dkms.conf`. It compiles without warnings against 6.1.0-45, and `dkms add / build / install` work on the VPS. It has not been loaded yet. |
| 2 | written (`fpgagpu_kms.c`):<br>- the KMS pipe (CRTC, primary plane, encoder, HDMI-A connector with board B's five modes, 800×600 preferred);<br>- shmem dumb buffers;<br>- damage → WRITE_IMAGE into one scanout image at board B's heap start (handle 0xC3E8), PRESENT after a modeset or a new size or format;<br>- SET_MODE on modeset, flips completed at once (no vblank);<br>- fbdev for the console.<br>Compiles against 6.1.0-45 and 7.3.4, DKMS-installed. Not loaded. Still to do: double buffering (an update now draws into the image being scanned out). The cursor plane came later (section 3.6). |
| 3 | written:<br>- `fpgagpu_mem.c`: heaps (`drm_mm`) and handle tables; an image's footprint computed by board B's own formula;<br>- `fpgagpu_gem.c`: board BOs, CREATE_* on `BO_CREATE`, DESTROY from a worker when the last reference goes;<br>- `fpgagpu_submit.c`: `SUBMIT` with an exact type list and a handle-ownership check per command, BATCH packing, one stream at a time, a FENCE per submission; `WAIT` on a fence or a BO; `READ` with chunk reassembly and re-requests.<br>The display's uploads take the same submit lock. `fpgagpu_selftest` checks it end to end (fill + read an image, write + read a buffer, two refusals). Compiles against 6.1.0-45 and 7.3.4, DKMS-installed. Not loaded: the VPS has `kernel.modules_disabled=1` until its next reboot. |
| 4 | the Mesa Gallium driver and winsys (`host/linux/mesa`, an overlay on Mesa 26.3.0 with `apply.sh`): an ES 2.0 context on the render node clears on board B and reads back exactly (`fg_egl_clear`). GBM surfaces are board-B images (the screen exports dma-bufs), flipped with KMS without a copy (`fg_kms_flip`, 23 frames a second through the relay). |
| 5 | shaders (NIR to the Windows libraries' compiler's back end), state, draws, textures, passes and clears (`host/linux/mesa/README.md`): `fg_egl_draw` (ES 2.0) and `fg_egl_gl` (desktop GL 1.3, fixed function the way Quake 3 engines draw) pass on the boards. OpenArena runs through SDL's kmsdrm on card1 and renders on board B, the whole scene and HUD with the sky, at about 0.7 frames a second through the relay (about 1 MB/s to the boards; every draw's vertices are streamed). It is not started from X: X has no glamor yet, so a GL program in X would render in software. |
| 6 | X on board B: lightdm's X kept off card1 (`x11/90-no-autoadd-gpu.conf`). X :1 on card1 (`x11/start-x-fpgagpu.sh`) either with software rendering shown through the copied-framebuffer path, or with glamor on the fpgagpu Mesa driver (`start glamor`, OpenGL ES 2.0): X renders on the boards into a GBM buffer shown without a copy. OpenArena started as a client of that X renders on the boards through DRI3 (buffers shared by dma-buf, BO_INFO; passes kept whole between processes by PASS). Glamor draws core-font text and logic ops in software, each a read back from the board: through the relay that makes such clients (xterm, xclock) take minutes. |

**2026-11-16, in the Hyper-V VM** (module and relay on the VPS stopped):
- The module from DKMS for 6.12.116 and 6.12.117: probe round trip about 0.3 ms; `fpgagpu_selftest` passes in 21 ms.
- Mesa 26.3.0 built in the VM (`-Dplatforms=x11`: Debian 13's `wayland-scanner` rejects the Wayland protocols Mesa
  takes as a subproject). `fg_egl_clear`, `fg_egl_draw`, `fg_egl_gl` and `fg_kms_flip` pass, each in about 40 ms.
- X :1 with glamor, Openbox and an xterm on board B, Deskflow for keyboard and mouse.
  - Text was solid blocks: glamor keeps glyphs (alpha-only pictures) in GL_R8 textures and reads their alpha through
    texture swizzle, which Mesa allowed only on desktop GL and GLES 3. `apply.sh` now enables EXT_texture_swizzle in
    GLES 2.0 contexts too, and the driver applies a sampler view's swizzle in the fragment program
    (`fg_egl_draw` test 5).
  - The pointer lagged and looked half transparent while moving: X drew it in software, each move a few render passes
    on the boards into the image being shown. The cursor plane above fixed it.
- `xwd -root` reads the whole 800×600 screen back from board B in about 0.4 s, colours exact.

**2026-11-16, later:** the module was reloaded with the zero-copy flips (1d67aa3). During a Mesa rebuild the VPS's kernel
oopsed in ext4 (`ext4_mb_load_buddy_gfp`, a NULL dereference while meson deleted a file); filesystem operations hung
and the VPS was rebooted. The module's receive, READ and SUBMIT paths were reviewed for memory safety and nothing was
found; the cause is not known. After the reboot the filesystem was clean and the module was loaded again.
