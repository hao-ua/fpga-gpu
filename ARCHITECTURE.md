# Architecture

**On this branch the GPU is the one-board build:** both halves of the design in one XC7A200T with one DDR3.
[Section 10](#10-the-one-board-build-board_ab_top) describes how they are joined, the shared memory map, and how the
cores and sizes differ. Sections 1–9 describe the two halves as the two-board design has them. The one-board build
keeps all of that except what section 10 lists; there, "board A" and "board B" mean the two halves on the one chip.

Two Artix-7 boards (XC7A200T-2, 1 GB DDR3 each) form one GPU. The host is a Windows PC running user-mode drop-in
libraries; there is no kernel driver. The exact RTL each board is built from is listed in `tools/build_board.tcl`.

## 1. Overview

```
 Windows PC ── UDP over 1 Gbit/s Ethernet ──▶ BOARD A ── board link (40-pin headers JM1/JM2) ──▶ BOARD B ──▶ HDMI
 (ddraw / opengl32 / GLES / d3d8 / d3d9          protocol engine          A→B: commands, render stream     fragment shading
  on one core library, host/win/core)            dispatcher               B→A: status, events, read-back,  texturing, blending
                                                 vertex shading           credits                          band flush, scanout
                                                 clip · setup · bins
                                                 band walk + raster
```

1. **Host to A.** The PC sends commands to board A at 192.168.1.10:5000. Board A's protocol engine writes them into an
   ingress ring in its DDR, and its dispatcher runs them in order:
   - A's own commands: buffers, vertex programs, uniforms, attributes, draws;
   - B's commands: forwarded unchanged over the link.
2. **Geometry on A.** Vertex shading, then clipping and setup, then triangle records and per-band bin lists in A's DDR.
   - At a FLUSH, A walks the bands and rasterizes them into 2×2-pixel quads.
   - The quads go to B as the render stream.
   - Records and bins are frame-banked, so frame *N + 1*'s geometry overlaps frame *N*'s stream.
3. **Fragments on B.** Quad setup and early depth, then the fragment core with the texture unit, then the ROP into an
   on-chip band buffer.
   - Each finished band is flushed to the render-target image in B's DDR.
   - A PRESENT shows the image on HDMI.
4. **Back from B.** B sends STATUS every millisecond, plus events, read-back data and credits. A merges them into its
   replies to the PC.

The board line runs **after the rasterizer**: setup, binning and rasterization are on A, and attribute interpolation is
fragment-shader work on B. Each board has one shader core: A has the vertex core, B has the fragment core and one
texture unit.

## 2. Board A — host link and geometry

- **Network and protocol:**
  - `eth_stack`: RGMII receive, MAC with FCS check and a 32 KB store-and-forward buffer, ARP / IPv4 / ICMP / UDP,
    transmit.
  - `mdio_mgr`: auto-negotiation and PHY status polling, with energy-efficient Ethernet off.
  - `fb_proto_a`: Go-Back-N receive, ACKs, STATUS.
  - `ddr_fifo`: the ingress ring.
- **`fb_dispatch_a`:** in-order execution and BATCH sub-messages, with a 100 ms wait for B and a 3 s render timeout.
  - `xfer_lin` does buffer transfers and memory tests, `buf_table` holds 1024 buffer descriptors, `fb_reply` builds the
    replies.
  - A write into A's memory waits until no draw is fetching from it.
- **`link_a_port`:** command credit and render-stream packing: a 128-beat FIFO, packets of up to 64 beats, sent under
  B's render credit.
- **`a_vertex`:** the vertex core `shader_core_vs2`, 4 warps of 16 vertices, plus its front end.
  - `vtx_feed`: index prefetch, a 32-entry vertex cache, a 16-entry burst cache and `attr_conv`, which converts BYTE
    through FLOAT and FP24 attributes.
  - `vtx_rbuf`: a lane-banked result buffer with 16 slots.
  - `vtx_asm`: assembles triangles, strips, fans, points and lines.
  - Draws overlap: uniforms and descriptors use two banks, chosen by the draw's parity.
- **`a_setup`:**
  - `clip_near` clips (Sutherland–Hodgman) against 7 planes: w ≥ 2⁻⁷, near, far and the ±2040-pixel x/y guard band.
    Triangles that need no clipping take a fast path.
  - `vtx_setup2` computes window x/y in 1/16 pixel.
  - Then area and culling, depth gradients, integer edge steps, and conservative depth bounds per triangle (`zq_tri`).
  - Each primitive becomes a 512-byte record plus a bin push.
  - A draw is bounded by its scissor / viewport rectangle.
  - A VS_STATE flag selects the 0 ≤ z ≤ w clip volume (reversed depth in the Direct3D libraries).
- **`tri_binner2`:** on-chip head tables and linked bin lists in DDR, in submission order, written 4 entries per burst.
- **`a_render`:** the band walk and rasterizer. It covers one quad position per cycle, uses the top-left fill rule, and
  handles points and lines up to 64 pixels.
- **`mem_arb`:** 5 read and 4 write ports onto the DDR3 controller.

## 3. Board B — fragments, images and display

- **`link_b_port`:** a 4096-word command buffer (credit `L_CREDIT`) and a 2048-beat render buffer (credit `L_RCREDIT`).
- **`fb_cmd_b`:** the in-order command processor for images, buffers, DRAW_STATE buckets, FLUSH, PRESENT and FENCE. A
  FLUSH blocks it until its frame is written.
- **Images:**
  - `img_table` holds 1024 image descriptors.
  - `img_write` does WRITE_IMAGE with format conversion.
  - `img_xfer` does READ / FILL / COPY, with colour key and stretch.
  - `img_geom` checks the rectangles.
- **Display:** `disp_b`, `scan_reader`, `disp_post` (8-bit palette, 64×64 cursor), `disp_out_b`, `video_mmcm` (mode
  switching by DRP) and the TMDS encoder and serializer.
  - Modes: 640×480, 800×600 (the default), 1280×720, 1280×800 and 1024×768, all at 60 Hz.
- **`b_render`:**
  - `quad_setup` handles 1 pixel per cycle.
  - An early-depth queue (16 entries, using the depth bounds A computed) comes next.
  - Then the fragment core `shader_core_fs2`, with `tmu24` + `tex_cache` + `cube_rcp`.
  - Then `b_rop` + `pixel_blend_b` at 2 pixels per cycle, with ES 2.0 blend, dither, stencil and depth.
  - B keeps no triangle store: A re-sends a triangle's data with its quads.
- **`mem_arb`:** 6 read and 4 write ports; scanout has the must-run read port.

## 4. The board link

- **Physical:**
  - Single-ended LVCMOS33 at double data rate, with a forwarded clock, over the two 40-pin headers.
  - A → B uses 64 bits per clock (JM1 plus part of JM2); B → A uses 32 bits.
  - At the 100 MHz link clock that is 800 MB/s forward and 400 MB/s back.
  - At 150 MHz the eye shrinks to 0.7 ns, so this physical layer has no faster rate.
- **Training:** each receiver scans its MMCM's fine phase shift on a pseudo-random stream and centres on the longest
  error-free window. A dedicated wire carries the handshake.
- **Packets:** a header {type, sequence, length}, the payload and a CRC-32, with parity on every half-word.
  - Fill words keep the lanes busy between packets.
  - The payload is scrambled with an xorshift32 keystream.
- **Error recovery:** Go-Back-N per direction, with cumulative ACK / NAK control packets.
  - The transmit ring holds 4096 beats, with at most 16 packets unacknowledged and 8-bit sequence numbers.
  - The receive side buffers 2048 beats and forwards only complete packets.
  - The retransmit timeout is 200 µs.
- **Measured error rate:** forward about 1.0 × 10⁻¹⁰ per word (2 events in 20.3 G words); back 0 errors in 154 M real
  words. All of them were corrected by retransmission.
- **Upper-layer types:** CMD, STATUS, EVENT, READ_DATA, CURSOR, CREDIT, SYNC, RSTREAM, RCREDIT. Credits carry an
  epoch, and SYNC handles a board restart.

## 5. The host protocol

- **Datagrams:** UDP with an 8-byte little-endian header: magic 0x4246, version 1, type and a 32-bit sequence number
  (0 = unsequenced).
- **Ordering and flow control:**
  - Board A takes a datagram only at the expected sequence number and only if its queue has room.
  - It ACKs every 16 datagrams, after 100 µs, at once with a NAK on a gap, and as a 1 ms heartbeat. Each ACK carries
    the free queue space as credit.
  - The board replies to the most recent valid sender.
- **Message groups:**
  - session and debug (0x01–0x07);
  - resources (0x10–0x12);
  - transfers (0x20–0x27);
  - display: SET_MODE, PRESENT, CURSOR, SET_PALETTE (0x30–0x34);
  - FENCE and FLUSH (0x40–0x41);
  - 3D: DRAW_STATE for B (0x50), and VS_PROGRAM, VS_UNIFORMS, VERTEX_ATTR, VS_STATE, DRAW and FRAME_BEGIN for A
    (0x51–0x56);
  - BATCH (0x60);
  - replies (0x81–0x86).
- **Handles** are 16 bits: bit 15 picks the board, bit 14 buffer or image, bits 9:0 the index. The host allocates them.
- **Implementations:** `host/fb/fbproto.py` is the reference; `host/win/core` is the C one the libraries use.

## 6. Number formats and shader cores

- **FP24** for every shader value: sign 1, exponent 7 (bias 63), mantissa 16.
  - Rounding is round-to-nearest-even, with denormals flushed to zero.
  - Relative precision is 2⁻¹⁷ and the range about 2^±62, which meets GLSL `highp`.
  - There is no FP16 path.
- **Integers** are 24-bit two's complement in a lane.
- **Positions:** window x / y are integers in 1/16 pixel, and edge values are exact integers.
- **Depth** is unorm 24 or 16 bits; **colour** is in channel units.
- **ISA:** 64-bit instruction words.
  - A vec4 instruction runs as one scalar micro-op per written component, across 16 lanes.
  - All three sources take swizzles and modifiers.
  - DP3, DP4 and varying loads run as accumulator chains.
- **Fragment core** (`shader_core_fs2`, board B):
  - A warp is 16 lanes, which is 4 quads; there are 8 warps.
  - 32 vec4 registers per lane, 64 uniforms, 8 varyings (perspective-correct), 8 samplers, 4096 instructions.
  - 16 multiply-add lanes.
  - Quarter-rate units (4 lanes) for the SFU functions (RCP, RSQRT, LOG2, EXP2, SIN, COS, SQRT, RCPM, RSQM),
    conversions, and integer multiply and divide.
  - Up to 4 quarter-rate micro-ops in flight per unit class.
  - Up to 8 texture requests in flight.
  - Branches, loops, DISCARD, DDX / DDY.
- **Vertex core** (`shader_core_vs2`, board A): the same design with vertices in the lanes, 4 warps of 16.
  - 16 attributes, 256 uniforms with run-time indexing, outputs gl_Position + gl_PointSize + 16 varyings.
  - No texture, derivative or discard instructions.

## 7. Texture unit (`tmu24` + `tex_cache`, board B)

- **Targets and filtering:**
  - 2D and cube maps.
  - Level of detail with bias or an explicit LOD.
  - NEAREST, LINEAR, nearest-mipmap and trilinear filtering.
  - REPEAT, CLAMP_TO_EDGE and MIRRORED_REPEAT wrap modes.
- **Formats:** RGBA8, RGBA4444, RGBA5551 and RGB565, converted exactly as c / (2ⁿ − 1).
- **Limits:** up to 2048 texels per side and 12 mip levels.
- **Incomplete textures** sample as (0, 0, 0, 1), as OpenGL ES 2.0 requires (for example, a mipmap filter with levels
  missing).
- **Rate:**
  - One texel job per pixel and level per cycle: a bilinear quad takes about 4 cycles, trilinear about 8.
  - Pixels a quad does not cover fetch nothing.
- **Cache:** 64 KB (512 sets × 4 lines × 32 bytes).
- **Not in hardware:** 3D textures, compressed formats, anisotropic filtering and MSAA.

## 8. Memory map and frame organisation

- **Board A's DDR:**

  | range | use |
  |---|---|
  | 0–64 MB | ingress ring |
  | 64–320 MB | triangle records: two frame banks of 2¹⁸ records × 512 B |
  | 320–384 MB | bin lists: two pools of 2²² entries × 8 B |
  | 384 MB–1 GB | host heap |

- **Board B's DDR:**

  | range | use |
  |---|---|
  | 128–144 MB | per-draw state (8192 draw ids × 2 KB) |
  | 256 MB–1 GB | host heap: images, programs, buffers |

- **Descriptors:** 1024 buffer descriptors per board, and 1024 image descriptors on B.
- **Bands:** 8 pixel rows for targets up to 1024 wide, 4 rows above that, up to 1280 wide.
- **B's band buffer:**
  - Three banks of 4096 cells. A cell is 2 pixels of colour, depth, stencil and an epoch.
  - The next band is loaded while the current one renders.
  - A D24S8 target is loaded and flushed per band, so depth and stencil persist between passes.
- **Render stream:** 64-bit beats.
  - QUAD items take 1 beat.
  - TRI items take 13 + 5 per varying beats.
  - BAND, FRAME_END and NOP mark the structure.

## 9. Clocks and resources

- **Clocks:**
  - The system clock is 120 MHz on both boards: the DDR3 controller's user clock. The DDR3 runs at 960 MT/s (a 480 MHz
    memory clock).
    - The MIG's PHASER FIFO clocks run 20 % over the speed grade (WPWS −0.833 ns in every 120 MHz build).
    - Calibration and traffic at this rate were checked error-free on both boards.
  - The board link has its own 100 MHz clock.
  - Ethernet receive runs at 125 or 25 MHz; video uses a per-mode pixel clock.
- **Utilisation of the builds now in the boards' flash:**

  | build | timing (setup / hold) | LUT | FF | slices | BRAM tiles | DSP |
  |---|---|---|---|---|---|---|
  | A 147 | +0.007 / +0.026 ns | 86 536 (64.7 %) | 74 823 | 30 755 (91.9 %) | 230.5 (63.2 %) | 209 |
  | B 142 | +0.016 / +0.030 ns | 103 771 (77.6 %) | 74 633 | 32 010 (95.7 %) | 291 (79.7 %) | 311 |

  At these slice counts about three quarters of a failing path's delay is routing, and board B has almost no timing
  margin left.
- **Throughput limits:**
  - B fills at most 1 pixel per cycle (118.5 Mpix/s at 120 MHz).
  - In games, B's frame is set by fragment warp slots and the texture round trip (about 91 cycles), not by instruction
    issue.
  - See [PERF_TESTS.md](PERF_TESTS.md).
- **Pairing:** A and B builds must use the same render-stream format; builds from the same source tree always do.

## 10. The one-board build (`board_ab_top`)

Both halves in one XC7A200T on one board, built with `tools/build_board.tcl S`. The PC sees the same protocol: it
talks to board A's side at 192.168.1.10, and the HDMI output is the same board's.

```
 Windows PC ── UDP ──▶ board A's half (board_a_core) ── link_loop ──▶ board B's half (board_b_core) ──▶ HDMI
                              │                                              │
                              └──────────── one mem_arb, one DDR3 ───────────┘
```

- **`board_ab_core`:** it instantiates `board_a_core` and `board_b_core` with `EXT_ARB` and `EXT_LINK` set.
  - Their memory ports go out to one shared `mem_arb`.
  - `link_a_port` and `link_b_port` are joined by `link_loop` instead of the ribbon-cable PHYs, which stay in the
    netlist idle.
  - Everything above the link is unchanged: commands for B, the render stream, credits, B's STATUS, events and read-back.
- **The shared arbiter:** 11 read and 8 write ports, highest priority first.
  - Reads: B scanout (must-run), A ingress ring, A transfer engine, B READ / COPY, B `xfer_lin`, B band load, B texture
    unit, B buckets and programs, then A's three geometry ports.
  - Writes: A ring, A transfer engine, A's two geometry ports, B WRITE_IMAGE, B `xfer_lin`, B FILL / COPY, B band flush.
- **One address space:** the two boards' DDR maps, packed into one 1 GB.

  | range | use |
  |---|---|
  | 0–64 MB | A's ingress ring |
  | 64–192 MB | A's triangle records: two frame banks of 2¹⁷ records × 512 B (half the two boards' banks) |
  | 192–256 MB | A's bin lists: two pools of 2²² entries × 8 B |
  | 256–448 MB | host heap for board A's buffers |
  | 448–464 MB | B's per-draw state (8192 draw ids × 2 KB) |
  | 464 MB–1 GB | host heap for board B's images, programs and buffers |

  - Board A's STATUS flags **bit 29** says "one address space" (0 on the two boards). The host then uses these two heaps.
  - From eb90593 on, board A refuses a CREATE_BUFFER outside 256–448 MB and board B an image, program or transfer
    outside 464 MB–1 GB (ERR_HEAP, 4). S 158 predates this and keeps the two boards' bounds (A from 384 MB, B from
    256 MB).
  - Board A's MEM_READ, MEM_WRITE and MEMTEST are not bounded, so debug tools can read A's records. On one board they
    reach every region, B's included.
  - In simulation `board_ab_core` stops if board B writes below its draw buckets, or if A's geometry or transfer writes
    leave their regions.
- **The cores, against the two boards:**

  | | one board | two boards |
  |---|---|---|
  | vertex core | 4 warps of 4 vertices | 4 warps of 16 |
  | fragment core | 4 warps of 4 lanes (one quad) | 8 warps of 16 lanes (4 quads) |
  | quarter-rate units per core | 1 | 4 |
  | SIN / COS, integer divide | none | yes |
  | SQRT / RCPM / RSQM micro-ops | none | yes |
  | quarter-rate micro-ops in flight / TEX live / triangle slots | 2 / 4 / 8 | 4 / 8 / 16 |
  | blend units in the ROP | one per pixel pair | one per pixel |

  - The quarter-rate units scale with the lanes, one per 4 lanes. Lane 4k + i goes through unit k in cycle i, so a
    quarter-rate micro-op takes 4 cycles at every lane count. A 16-lane core is cycle-identical to the two boards' one.
  - Without the units, SIN, COS, IDIV, IMOD, UDIV and UMOD stop the warp, as IOP does. STATUS flags bits 31 (no SIN / COS)
    and 30 (no integer divider) say so, in board A's flags for the vertex core and board B's for the fragment core. The
    compilers then lower those operations to float code.
  - The ROP's shared blend unit serves a pair's two pixels in consecutive cycles. B still fills at most one pixel a cycle.
- **Sizes:**
  - Render targets and video modes up to 800 pixels wide: 640×480 and 800×600. A wider mode is refused (a mode error), and
    so is a FLUSH to a wider target.
  - The band buffer has 2048 cells a bank: 8-row bands up to 512 pixels wide, 4 rows above that.
  - The scanout line buffer holds 8 lines.
- **Debug:**
  - Board A's geometry profile counters are off: STATUS words 101–113 read 0.
  - JTAG has board A's status register only (USER1), read out bit by bit from a snapshot. Board B's JTAG registers are
    off; A's STATUS carries B's words as before.
- **Clocks:** 100 MHz for everything on the system clock (the MIG's user clock, DDR3 at 800 MT/s), the link loop included.
  Ethernet and video clocks are as on the two boards.
- **Resources** (S 158, the first one-board build):

  | build | timing (setup / hold) | LUT | FF | slices | BRAM tiles | DSP |
  |---|---|---|---|---|---|---|
  | S 158 | +0.001 / +0.035 ns | 116 936 (87.4 %) | 96 340 | 33 134 (99.1 %) | 261 (71.5 %) | 262 |

  The two boards' builds (A 147 + B 142) need about 190 k LUT together; the lane, unit and queue cuts above bring that
  under the part's 134.6 k.
- **Throughput:** the fragment side sets the pace. In OpenArena at 640×480 board B's frame is about four times the two
  boards' (7.2 against 29.2 fps); see [PERF_TESTS.md](PERF_TESTS.md).
