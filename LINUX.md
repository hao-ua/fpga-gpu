# Linux driver: design

The GPU under Linux: a DRM kernel driver that talks to the boards over Ethernet and shows the board's HDMI output as a
KMS display, and a Mesa Gallium driver that gives OpenGL ES 2.0, OpenGL, EGL, GLX and GBM through Mesa's front ends.
This branch (`linux-driver`) is cut from master, so it drives the **two-board** design. Nothing here is built yet; this
document is the plan.

## 1. Decisions

| question | choice |
|---|---|
| where Linux runs | the Linux host 192.168.53.1, a VPS reached over WireGuard, which builds everything and reaches the boards through a UDP relay on the PC (section 6). A Hyper-V VM bridged onto board A's network may follow. |
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
- **Cursor plane:** CURSOR (shape) and CURSOR_POS (position) messages.
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
  - The VPS end accepts that flow only from the PC's address, and serves programs on 127.0.0.1.
  - The round trip is 62–131 ms (about 70 ms), so the boards work but slowly: 64 datagrams per round trip is about
    1–2 MB/s.
  - Proven 2026-11-15: `fb_status.py --ip 127.0.0.1` on the VPS read both boards' STATUS (A 154, link trained, B alive).
- **A VM, if one comes:** Hyper-V generation 2, an external virtual switch on the NIC cabled to board A (shared with
  the management OS so Windows keeps 192.168.1.112), the VM at 192.168.1.113/24, `modprobe fpgagpu` with no relay.
- **One sender:** while the module or the relay is in use it is the boards' sender. The Windows libraries and the
  `host/fb` tools on the PC must not run at the same time; stop the relay first.
- **Not in scope yet:** the one-board build (its memory map and STATUS flags bit 29); a different heap layout.

## 7. Status

| phase | state |
|---|---|
| 1 | written: transport (HELLO, STATUS, the sequenced window, the resend rules, events), the DRM device with a render node, `GET_PARAM` / `GET_STATUS`, `fpgagpu_status` (the check), Kbuild and `dkms.conf`. It compiles without warnings against 6.1.0-45, and `dkms add / build / install` work on the VPS. It has not been loaded yet. |
| 2 | written (`fpgagpu_kms.c`):<br>- the KMS pipe (CRTC, primary plane, encoder, HDMI-A connector with board B's five modes, 800×600 preferred);<br>- shmem dumb buffers;<br>- damage → WRITE_IMAGE into one scanout image at board B's heap start (handle 0xC3E8), PRESENT after a modeset or a new size or format;<br>- SET_MODE on modeset, flips completed at once (no vblank);<br>- fbdev for the console.<br>Compiles against 6.1.0-45 and 7.3.4, DKMS-installed. Not loaded. Still to do: double buffering (an update now draws into the image being scanned out) and the cursor plane. |
| 3–6 | not started |
