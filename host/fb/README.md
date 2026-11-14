# host/fb — remote display / GPU resource protocol tools

Protocol: [ARCHITECTURE.md](../../ARCHITECTURE.md) §5; performance tests: [PERF_TESTS.md](../../PERF_TESTS.md).
Python 3, standard library only.

| file | purpose |
|---|---|
| `fbproto.py` | protocol library: Go-Back-N sender with credit and resend, reply decoder, buffers, memory, registers, PHY, fences |
| `fb_status.py` | print the board's STATUS (`--loop 1` to repeat, `--raw` for every word) |
| `fb_a_test.py` | board A acceptance test: registers, PHY ID, MEMTEST, error events, byte-exact buffer and raw memory round trips, throughput |
| `eth_rx_sweep.py` | 1000 Mbit/s RGMII receive timing sweep over the 32 IDELAY taps (`--rxdly 0|1`, `--set` applies the centre of the error-free window) |
| `phy_opt.py` | change RXDLY / 1000BASE-T advertisement / tap safely (PHY option trial: reverts after 8 s if the host loses contact) |
| `udp_blast.ps1` | native Windows paced UDP blast for line-rate receive tests (no Python needed) |
| `fb_mode.py`, `fb_image.py` | board B over the link: video mode / present; show a picture (CREATE_IMAGE + WRITE_IMAGE + PRESENT, `--verify` reads it back) |
| `fb_video.py` | play video on board B's HDMI: a file through ffmpeg, a generated test animation (`--test`) or raw frames (`--raw`); double-buffered, presents locked to B's vsync (see "Video" below) |
| `fb_geom_test.py`, `fb_3d_test.py` | 3D on the two boards: A's geometry records against the model; a whole frame (A draw → render stream → B) read back from B and compared bit for bit (`--abort`: a rejected FLUSH) |
| `fb_cube.py` | **3D on screen**: a rotating, lit cube (tests/cube.vert / cube.frag through `glslc --isa64`), double-buffered render-then-PRESENT at vsync; `--check` reads the first frame back and compares the lit faces with the host, `--ppm` saves it, `--selftest` runs the shaders through the core simulator without a board. **Performance scene: `--cubes N`** (one draw with the rotation in the vertex shader, tests/cubes.vert; `--draws` = a draw per cube), `--nohold` = frames as fast as rendered; `fb_cube_sweep.sh` runs the sweep |
| `fb_2d.py` | hardware test + demo of the DirectDraw-class 2D features: READ / FILL / COPY_IMAGE (stretch, mirrors, colour key, overlap) checked by read-back against Python models, 8-bit palette animation, hardware cursor (colour, AND / XOR) moved over the fast path |
| `link_err.py`, `link_log.py`, `link_beat.py` | board-link error rate per load phase, counter log over time, timer-beat analysis |
| `link_soak.py` | board-link soak under image streaming: alternating formats, FENCE per frame, every counter movement of both receivers, per-word rates, forwarded / received invariant |

## Network setup (one time)

Board A takes UDP port **5000**, MAC 00:0a:35:01:02:03. It asks DHCP for its address each time its link comes up, and
uses **192.168.1.10** when no lease is bound within 10 s. Cabled directly to a PC it is therefore at 192.168.1.10, and
the PC's Ethernet port needs an address in 192.168.1.0/24, for example from an administrator PowerShell (use the
adapter name from `Get-NetAdapter`):

```
New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 192.168.1.112 -PrefixLength 24
```

On a network with a DHCP server, set `GPU2_IP` to the leased address or a name for it: every tool takes it as its
default `--ip`, and `fb_status.py` prints the address in use and whether it is a lease.

Replies go back to the address and port the last valid datagram came from, so the tools may run in
Windows or in WSL (WSL2's NAT forwards the replies).

## Typical bring-up

```
python3 fb_status.py
python3 fb_a_test.py
python3 eth_rx_sweep.py --rxdly 1    # optional: re-measure the 1000 Mbit/s receive window
powershell -File udp_blast.ps1 -Count 1000000 -Mbps 990
```

Board A powers up with RXDLY on and tap 19 (the measured centre of the receive window).

To test 100 Mbit/s without touching the PC's NIC settings: `python3 phy_opt.py --adv1000 0`
(back: `--adv1000 1`). Never change RXDLY or the tap with plain register writes: use `phy_opt.py` /
`eth_rx_sweep.py`, which use the trial modes.

## Video (`fb_video.py`)

Video files need `ffmpeg` and `ffprobe` (in WSL: `sudo apt install ffmpeg`; installed on 2026-10-25). The test
animation needs nothing. Board B has no audio output, so the sound is not played.

```
python3 fb_video.py --test                        # scrolling colours, 8 vertical lines, bouncing box, frame number
python3 fb_video.py movie.mp4                     # fit to the current mode (aspect kept, centred), RGB565
python3 fb_video.py movie.mp4 --mode 2 --loop     # switch to 1280x720 first, repeat endlessly
python3 fb_video.py movie.mp4 --start 60 --secs 30 --size 640x360
python3 fb_video.py movie.mp4 --xfer rgb --storage rgba8          # 24-bit colour, 1.5 x the data
ffmpeg -i x.mkv -vf scale=800:450 -pix_fmt rgb565le -f rawvideo - | python3 fb_video.py --raw - --size 800x450 --fps 25
```

How it works: two full-mode images in B's DDR (default handles 0xC028 / 0xC029 at 0x2800_0000) are used in turn;
each frame is written into the one not on screen (WRITE_IMAGE of the video rectangle) and presented with the hold
flag, so B's command processor runs the next frame's writes only after this frame became current: no tearing,
no host round trip per frame. B's refresh is the clock: frame i owns the vsyncs [round(i r / f), round((i+1) r / f))
for refresh r and frame rate f, and is presented once per vsync of that slot (30 fps at 60 Hz: 2 each, 25 fps:
2-3-2-3-2), so the cadence does not depend on network jitter. The host keeps `--depth` (3) frames in flight and skips
a frame whose slot is already over (bandwidth too low); `--verify` reads the last frame back from B and compares it.

Bandwidth = width x height x bytes per pixel (RGB565 2, RGB 3, others 4) x fps; the host -> A path sustains about
52 MB/s: 800x450 RGB565 at 30 fps = 21.6 MB/s, 800x600 = 28.8 MB/s, 1280x720 = 55 MB/s (a few skips).

The summary line "vsyncs on screen per frame ... N of M as planned" checks the cadence from B's EV_PRESENT vsync
counts; "B underruns" and "B command errors" must stay 0.

## 2D, palette, cursor (`fbproto` calls, docs §20)

```
b.read_image(h, fp.TF_RGBA8888, x, y, w, h_, storage_fmt=fp.FMT_RGB565)      # glReadPixels / DirectDraw Lock
b.fill_image(h, fp.TF_RAW, value, x, y, w, h_, mask=fp.MASK_R | fp.MASK_G)   # Blt colour fill / glClear
b.copy_image(src, dst, sx, sy, sw, sh, dx, dy, dw, dh, mirror_x=True, key=(lo, hi))   # Blt: stretch, mirror, key
b.set_palette(0, [0xRRGGBB] * 256)          # P8 images (fp.FMT_P8) are shown through it, from the next frame on
b.cursor_shape(words); b.cursor_show(fp.CUR_COLOR, hot=(0, 0), pos=(x, y)); b.cursor_pos(x, y)   # fast path
```

Reads (`read_image`, `mem_read`, `read_buffer`) keep two 256 KB requests in flight and re-request a lost
READ_DATA chunk as soon as a later one arrives, so a lost datagram costs about one round trip, not a timeout.
