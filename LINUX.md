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
| `BO_CREATE` | a board BO: heap, size, kind (buffer / image) and the image descriptor (format, size, pitch, levels); or an alias: another descriptor of an image's bytes, from an offset |
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
  `ip=192.168.1.10 port=5000`. The fpgagpu Mesa is the VM's system GL: `/etc/ld.so.conf.d/00-fpgagpu-mesa.conf`
  names `/opt/fpgagpu-mesa/lib/x86_64-linux-gnu` ahead of the system's directory (`ldconfig`), so every program loads
  it with no `LD_LIBRARY_PATH`; Debian's own Mesa answers "failed to load driver: fpgagpu".
  - ioquake3 games need `cl_renderer opengl1`: their default, `opengl2`, writes GLSL 1.20 (see the OpenGL 2.0 note
    below).
  - WSL does not reach the Default Switch, Windows does: ssh from WSL goes through a Windows-side TCP pipe as its
    ProxyCommand.
  - Keyboard and mouse: Deskflow. The server runs on Windows, the client (`deskflow-client`, Debian's 1.22) in the VM's
    X, started by `start-x-fpgagpu.sh` with `DESKFLOW_SERVER=172.20.96.1` (the Default Switch's Windows address); the
    pointer crosses from the Windows screen onto board B's monitor and back. X itself opens no input devices.
  - **The desktop now: XFCE through LightDM** (GDM and GNOME Shell removed; GNOME's other packages stay). LightDM
    logs `debian` into XFCE (`x11/lightdm-fpgagpu.conf` in `/etc/lightdm/lightdm.conf.d/`); X runs on the boards by
    `x11/20-fpgagpu.conf`, which also turns X's blanking off (each blank is a modeset of board B) and **MIT-SHM off**:
    glamor keeps a shared-memory pixmap in host memory and uploads it again for every operation that reads it, so
    with the extension XFCE's desktop kept X uploading a screen-sized image (about 2 MB) over and over and X stopped
    answering clients (Deskflow among them); without it clients send ordinary pixmaps, uploaded once. XFCE's
    compositor is off for `debian` (`x11/xfwm4-no-compositing.xml` as `~/.config/xfce4/xfconf/xfce-perchannel-xml/
    xfwm4.xml`). The Deskflow client runs in a loop with `--no-restart` (`x11/fpgagpu-deskflow-client` in
    `/usr/local/bin`, started by `x11/deskflow-client.desktop`): its own reconnecting hung after the Windows server
    restarted. The loop ends when its X stops answering, and first stops (by the PIDs it recorded) a loop or client an
    earlier session left behind: Debian keeps a user's processes after the session ends, and two clients of one name
    took turns at the server.
  - OpenGL 2.0 with GLSL 1.10 (2026-11-16): ARB_shadow (depth compare) in the fragment program, the D24S8 image read
    by the TMU through an RGBA8 alias (the kernel's BO_CREATE alias flag); Mesa's OpenGL 2.0 context kept at the
    driver's GLSL 1.10 (`apply.sh` patches `version.c` and `getstring.c`). ioquake3's default `opengl2` renderer needs
    GLSL 1.20 and stays out; `opengl1` is set for `debian` (profile and menu launcher).
  - glxgears' gears came out smeared sideways (2026-11-16): its 300-pixel back buffer (DRI3, scan-out) has rows of
    1216 bytes (a pitch a multiple of 32), and X draws the window from it through the TMU, which reads tight rows
    (1200 bytes): each row started 4 pixels further on. Mesa now samples such an image from a copy with tight rows
    (see `host/linux/mesa/README.md`), a depth texture of a width no multiple of 8 likewise. Fixed with it: when a
    context's 8191 draw ids ran out, the pass restarted after the draw had put its images on the old pass's list, and
    the kernel refused the draw's state (EACCES, once in the VM's log).
  - Resizing and moving windows lagged (2026-11-16): a render pass costs board B ~33 ns per pixel of its target
    whatever it draws (16 ms on the 800 x 600 screen), and X ends ~10 screen passes per resize step (xfwm4's frame
    pieces drawn in small pixmaps in between), ~3 per move step. Mesa now renders a pass that reaches few rows into an
    alias of those rows only (region passes, `host/linux/mesa/README.md`; the kernel's alias takes an offset, and
    heap B keeps an 8 KB guard at its top). Measured with the same 30 steps: moves 1.5 s -> 0.7 s of board time;
    during resizes glxgears kept 181 fps (4 fps before). The boards give no vblank, so X's Present does not pace
    windowed clients: glxgears presents as fast as the board takes it.
  - Moves still lagged with bigger windows. Since 2026-11-17 Mesa does a draw that copies an image one to one as
    board B's COPY_IMAGE (~9 ns a copied pixel; `host/linux/mesa/README.md`, Copies): X's window copies and tiles all
    became copies (190 in 10 moves, none drawn). 30 moves, until the boards were done: a 604 x 394 xterm 546 -> 279
    ms, a 364 x 264 one 264 -> 151 ms (the moves themselves take 140 ms). What remained a move: X copied the window
    into a temporary image and back (no GL_NV_texture_barrier: two copies), and the strip a move uncovers is
    composited into a temporary of its bounds (glamor's composite program, a pass of ~613 x 428) before it is copied.
  - Big windows still trailed the pointer while dragged: X does not wait for the boards, so moves queued behind
    each other (a 604 x 394 xterm dragged 60 steps at ~110 a second: the boards finished 122-135 ms after the
    pointer stopped). Since 2026-11-17 a flush that submitted work waits for the one four flushes back
    (`FPGAGPU_FLUSH_DEPTH`, 0 = no wait), and Mesa offers GL_NV_texture_barrier and GL_MESA_tile_raster_order, so
    glamor copies a window inside the screen image directly (one COPY_IMAGE instead of two; README, Copies). The same
    drag: the boards done 14-16 ms after the pointer stopped; 30 moves of that xterm one after another 303 ms (279
    before, with the backlog; without the wait 208). Depth 1 / 2 / 4 measured: 513 / 383 / 302 ms (the X server
    flushes a few times a move). Left a move: the uncovered strip's pass of ~613 x 428.
  - Small black dashes on every window's title bar, at the top of each button (seen 2026-11-16 already; found
    2026-11-17): board B loads and stores every row of a band (up to 8, b_rop), also past the last row of an image
    whose height is no multiple of 8. xfwm4 makes its 21 x 29 button images by drawing into fresh pixmaps, rendered
    in shadows of 96-byte rows: rows 29-31 of a shadow went into the next object, a button image, whose bytes came
    back as they were except the three padding pixels of each 96-byte row, now zero: three black runs stepping
    right by three pixels a row (a software X in Xephyr showed none). The kernel now gives every image the memory of
    whole bands (its height rounded up to 8 rows, `fpgagpu_gem.c`); the size it reports and checks aliases against
    stays the image's. `fg_egl_gl` test 15: 32 targets 21 x 29, each followed by an image: on the old module all 32
    images changed (36 bytes each), now none. Padded images taller than a multiple of 8 (a 300 x 300 DRI3 back
    buffer of rows of 1216 bytes) did the same to their neighbours.
  - The boards restarted while the VM was paused (the PC asleep; 2026-11-17, board A's uptime 26 min): every
    sequenced command then timed out (BO_CREATE: ETIMEDOUT after 10 s; X drew nothing) while STATUS still answered,
    because the module's session (sequence numbers) was the old one. Reloading the module (a new HELLO) fixed it.
    The module now recovers by itself. It asks for STATUS once a second: board A restarted when its uptime is short
    of the last one plus the time since, or the sequence number it expects is far behind ours; board B when its
    frame count went back. The recovery starts at once; until it is done a request that is sending fails (EIO), new
    ones wait for it, and fences count as passed. Recovery:
    a new session with board A (the wire's sequence numbers start at 1, the driver's fences go on), the CREATE of
    every buffer object on the restarted boards again (contents lost), after board B's restart the display's mode,
    a hotplug event (the console redraws) and a change uevent `FPGAGPU_RESET=1`, on which
    `x11/99-fpgagpu-display.rules` restarts a running display manager. After a restart of board A alone X keeps
    running: PASS's `state_lost` 2 tells Mesa to place its programs again. Checked with the test parameter
    (`echo 1` / `3` `> /sys/module/fpgagpu/parameters/restarted`: as if board A / both had restarted): A alone, X the
    same process and drawing, 2 buffer objects described again; both, 171 described again, X restarted by udev;
    `fpgagpu_selftest` and `fg_egl_gl` pass after each without a reload. And with board A really restarted, twice
    (A 154 loaded again over JTAG while X ran): board A answers on the network again ~67 s after the load, and the
    module had its new session and 2 buffer objects described again 0.5 ms after seeing it; X stayed the same process
    and drew a new xterm correctly, both tests passed without a reload. (The first time recovery ran up to a second
    after the restart was seen and 102 of X's BO_CREATE / SUBMIT calls failed in between: hence "at once" and the
    waiting above; the second time none failed, X being idle.) Board B restarting alone has not been seen for real.
  - X hung in one SUBMIT for minutes (2026-11-17, a nested X server putting 640 x 480 images: 1.25 MB of WRITE_IMAGE
    a call): the wait for room in the send window was interruptible, X's timer signal ended it, the ioctl started
    again and sent the whole stream once more, each time, while board A took ~80 000 datagrams a second. A restarted
    SUBMIT also sent draws twice. The waits in sending and in READ now end only on a fatal signal; with the same
    load X answers in ~10 ms.
  - fpgagpu loads at boot (`kernel/fpgagpu-load.conf` as `/etc/modules-load.d/fpgagpu.conf`). While board A does not
    answer (the boards off; at boot, the network not up yet) the module stays loaded and greets it again every 5 s;
    the DRM device appears once it answers. LightDM waits up to 10 s for the card at its start
    (`x11/lightdm-wait-fpgagpu.conf` as `/etc/systemd/system/lightdm.service.d/fpgagpu.conf`), so X starts on the
    boards when they are on; otherwise on Hyper-V's display, and `x11/99-fpgagpu-display.rules` (udev) restarts LightDM
    when the card appears later (only once it is running: a restart in the middle of its wait left it failed - seen at
    the first reboot). Checked 2026-11-16: two VM reboots (attached on the 2nd try, X on the boards ~4 s after LightDM
    started), and board A made unreachable (a blackhole route) then reachable again (X moved from card0 to the boards).
  - Board B once stopped taking commands (board A waiting on B's credit, its queue still) after X servers were killed
    in the middle of frames while display managers stopped. Resetting B alone was not enough: board A kept waiting for
    the old frame. Both boards had to be power-cycled, and the module reloaded for a new session (HELLO). The kernel
    now ends a frame a closing file left open (a FLUSH to an image that does not exist: board A refuses the target,
    board B drops the frame); seen working when LightDM restarted X.
  - Before XFCE: GNOME through GDM (the display manager; Debian 13's GNOME 48): `x11/20-fpgagpu.conf` in
    `/etc/X11/xorg.conf.d` makes the fpgagpu card the primary GPU with glamor whenever the module is loaded (Hyper-V's
    display is not added); `/etc/gdm3/daemon.conf` has `WaylandEnable=false`, `AutomaticLoginEnable=true`,
    `AutomaticLogin=debian` (the VM's user, whose password is locked); its session starts the Deskflow client from
    `~/.config/autostart` (`x11/deskflow-client.desktop`; the client's certificate and the server's fingerprint in
    `~/.config/Deskflow/tls`). GNOME's screen lock and idle blank are off for it (gsettings
    `org.gnome.desktop.screensaver lock-enabled false`, `org.gnome.desktop.session idle-delay 0`,
    `org.gnome.desktop.lockdown disable-lock-screen true`).
  - GTK 4 programs (GNOME Settings, Software) draw in software here (their GL renderer needs OpenGL 3.3 or GLES 3.0)
    and hand X whole windows to upload. Settings' Displays panel kept X busy uploading and reading back until it was
    closed; Deskflow could not connect meanwhile.
  - Reloading the module under GDM: logind keeps the session's DRM devices open (in PID 1's descriptor store) after
    GDM stops, so `rmmod` finds the module in use; a reboot of the VM (or ending those sessions) frees it.
  - Games (OpenArena): turn on the server's relative mouse moves and lock the pointer to the VM's screen with Scroll
    Lock. Otherwise the absolute pointer reaches the screen's edge and crosses to Windows, and on every crossing the
    Deskflow client takes X's keyboard focus, so the game loses its focus and its mouse grab.
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
- OpenArena as a client of that X (SDL x11, GLX through DRI3): the timedemo `demo088-test1` (640×480 windowed,
  `cl_renderer opengl1`) runs at 15.4 frames a second (3398 frames in 220 s; through the relay it was 0.7, the Windows
  libraries do about 31). Full screen at 800×600 it is playable with Deskflow's relative moves and Scroll Lock.
- After a match, X's text came out as pieces of the wrong glyphs. The game's log showed 20 SUBMITs refused by the
  kernel (EACCES; the timedemo had 13): the driver cut its stream at 3 MB and emptied the SUBMIT's BO list, but a
  draw names its textures and program buffer before it emits the DRAW_STATE that uses them, so a cut in between left
  them off the next list. The list now stays for the whole render pass (the pass holds every image on it). A refused
  SUBMIT also no longer leaves the driver believing its programs and state reached the boards: it places every program
  again and resends its state. The kernel logs which command it refused. The timedemo: 0 refused, 15.3 frames a
  second, the worst frame 149 ms (305 ms before), X's text right after it.
- GNOME on Xorg under GDM renders on the boards (its desktop on board B, Deskflow for input). On the way:
  - the pass did not hold its target: GNOME's compositor frees an offscreen texture right after rendering into it,
    so the pass's FLUSH and copy back named a freed image (SUBMITs refused with EACCES, or with ENOENT for a GEM handle
    already closed). The target's images are now held while they are the target;
  - `glBlitFramebuffer` was ignored (GNOME on X11 presents with it): now COPY_IMAGE;
  - a uniform at a constant offset was read as an integer although integers are floats by then: GNOME's vertex
    shaders were refused as having a billion uniform vectors;
  - a SET_MODE in an atomic commit waited interruptibly, and X's signals cut it short (-512): modeset and present
    waits now end only by their timeout (compiled, loaded at the next module load);
  - GNOME started with its session locked and `debian` has no password: the lock is turned off (above).

**2026-11-16, later:** the module was reloaded with the zero-copy flips (1d67aa3). During a Mesa rebuild the VPS's kernel
oopsed in ext4 (`ext4_mb_load_buddy_gfp`, a NULL dereference while meson deleted a file); filesystem operations hung
and the VPS was rebooted. The module's receive, READ and SUBMIT paths were reviewed for memory safety and nothing was
found; the cause is not known. After the reboot the filesystem was clean and the module was loaded again.
