# Architecture: the one-board build

One Artix-7 board (XC7A200T-2, 1 GB DDR3) is the whole GPU. Its FPGA holds both halves of master's two-board design:
the **geometry half** (board A's design: Ethernet, protocol, vertex shading, setup, binning, rasterization) and the
**fragment half** (board B's design: fragment shading, texturing, blending, scanout). They share one DDR3 through one
arbiter and talk through FIFOs inside the chip instead of a ribbon cable. The host is a Windows PC running user-mode
drop-in libraries; there is no kernel driver.

- The top level is `board_ab_top` (the MIG and clocks) around `board_ab_core` (the two halves, the arbiter and the
  link loop).
- `tools/build_board.tcl S` lists the exact RTL it is built from.
- Every size and unit below is a parameter, set at build time (section 10).

## 1. Overview

```
 Windows PC ── UDP over 1 Gbit/s Ethernet ──▶ GEOMETRY HALF ── link_loop (FIFOs in the chip) ──▶ FRAGMENT HALF ──▶ HDMI
 (ddraw / opengl32 / GLES / d3d8 / d3d9         protocol engine     A→B: commands, render stream     fragment shading
  on one core library, host/win/core)           dispatcher          B→A: status, events, read-back,  texturing, blending
                                                vertex shading           credits                     band flush, scanout
                                                clip · setup · bins
                                                band walk + raster
                                                       │                                                   │
                                                       └──────────── one mem_arb, one DDR3 (1 GB) ─────────┘
```

1. **Host to the board.** The PC sends commands to 192.168.1.10:5000. The protocol engine writes them into an ingress
   ring in DDR, and the dispatcher runs them in order:
   - the geometry half's own commands: buffers, vertex programs, uniforms, attributes, draws;
   - the fragment half's commands: passed unchanged through the link loop.
2. **Geometry.** Vertex shading, then clipping and setup, then triangle records and per-band bin lists in DDR.
   - At a FLUSH the band walk rasterizes the bands into 2×2-pixel quads.
   - The quads go to the fragment half as the render stream.
   - Records and bins are frame-banked, so frame *N + 1*'s geometry overlaps frame *N*'s stream.
3. **Fragments.** Quad setup and early depth, then the fragment core with the texture unit, then the ROP into an
   on-chip band buffer.
   - Each finished band is flushed to the render-target image in DDR.
   - A PRESENT shows the image on HDMI.
4. **Back to the host.** The fragment half sends STATUS every millisecond, plus events, read-back data and credits. The
   geometry half merges them into its replies to the PC.

The line between the halves runs **after the rasterizer**: setup, binning and rasterization are in the geometry half,
and attribute interpolation is fragment-shader work. Each half has one shader core: the vertex core, and the fragment
core with one texture unit.

The text below calls the halves "A" and "B", after the two boards they come from.

## 2. The geometry half (A)

- **Network and protocol:**
  - `eth_stack`: RGMII receive, MAC with FCS check and a 32 KB store-and-forward buffer, ARP / IPv4 / ICMP / UDP,
    transmit.
  - `mdio_mgr`: auto-negotiation and PHY status polling, with energy-efficient Ethernet off.
  - `fb_proto_a`: Go-Back-N receive, ACKs, STATUS.
  - `ddr_fifo`: the ingress ring.
  - The address is fixed at 192.168.1.10.
- **`fb_dispatch_a`:** in-order execution and BATCH sub-messages, with a 100 ms wait for B and a 3 s render timeout.
  - `xfer_lin` does buffer transfers and memory tests, `buf_table` holds 1024 buffer descriptors, `fb_reply` builds the
    replies.
  - A buffer must lie in A's heap, 256–448 MB (ERR_HEAP otherwise, section 5).
  - A write into A's memory waits until no draw is fetching from it.
- **`link_a_port`:** command credit and render-stream packing: a 128-beat FIFO, packets of up to 64 beats, sent under
  B's render credit.
- **`a_vertex`:** the vertex core `shader_core_vs2`, 4 warps of 4 vertices (section 7), plus its front end.
  - `vtx_feed`: index prefetch, a 32-entry vertex cache, a 16-entry burst cache and `attr_conv`, which converts BYTE
    through FLOAT and FP24 attributes.
  - `vtx_rbuf`: a lane-banked result buffer.
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
- **Debug:** the geometry profile counters are off (STATUS words 101–113 read 0). JTAG has this half's status register
  only (USER1), read out bit by bit from a snapshot; it carries B's words too.
- Its 5 read and 4 write memory ports go to the shared arbiter (section 5).

## 3. The fragment half (B)

- **`link_b_port`:** a 4096-word command buffer (credit `L_CREDIT`) and a 2048-beat render buffer (credit `L_RCREDIT`).
- **`fb_cmd_b`:** the in-order command processor for images, buffers, DRAW_STATE buckets, FLUSH, PRESENT and FENCE.
  - A FLUSH blocks it until its frame is written.
  - A FLUSH to a target wider than 800 pixels is refused.
  - Images, programs and transfers must lie in B's heap, from 464 MB (ERR_HEAP otherwise).
- **Images:**
  - `img_table` holds 1024 image descriptors.
  - `img_write` does WRITE_IMAGE with format conversion.
  - `img_xfer` does READ / FILL / COPY, with colour key and stretch.
  - `img_geom` checks the rectangles.
- **Display:** `disp_b`, `scan_reader`, `disp_post` (8-bit palette, 64×64 cursor), `disp_out_b`, `video_mmcm` (mode
  switching by DRP) and the TMDS encoder and serializer.
  - Modes: 640×480 and 800×600 (the default), at 60 Hz. Wider modes are refused (a mode error).
  - The line buffer holds 8 lines.
- **`b_render`:**
  - `quad_setup` handles 1 pixel per cycle.
  - An early-depth queue (16 entries, using the depth bounds A computed) comes next.
  - Then the fragment core `shader_core_fs2` (section 7), with `tmu24` + `tex_cache` + `cube_rcp`, and 8 triangle slots.
  - Then `b_rop` + `pixel_blend_b`, with ES 2.0 blend, dither, stencil and depth. One blend unit serves a pixel pair:
    the pair's two pixels go through it in consecutive cycles.
  - B keeps no triangle store: A re-sends a triangle's data with its quads.
- **Debug:** B's own JTAG registers are off; its STATUS words reach the host through A's.
- Its 6 read and 4 write memory ports go to the shared arbiter; scanout has the must-run read port.

## 4. The link inside the chip

- **`link_loop`** joins `link_a_port` and `link_b_port` with one FIFO each way:
  - A → B: 513 beats of 64 bits;
  - B → A: 513 words of 32 bits.

  At 100 MHz that is 800 MB/s forward and 400 MB/s back, the ribbon link's rates.
- Packets arrive whole and in order, and a beat that the sender has not handed over yet is a stall, so the ribbon link's
  physical layer is not used: no training, parity, CRC, scrambling or retransmission.
- **Above it nothing changes.** The packet types are the two boards' (CMD, STATUS, EVENT, READ_DATA, CURSOR, CREDIT,
  SYNC, RSTREAM, RCREDIT), with the same credits and the same SYNC after a restart.
- `fb_status.py` shows the loop as `link trained 1 … B alive 1`.

## 5. Memory: one arbiter, one address space

- **`mem_arb`:** 11 read and 8 write ports onto the DDR3 controller, highest priority first.
  - Reads: B scanout (must-run), A ingress ring, A transfer engine, B READ / COPY, B `xfer_lin`, B band load, B texture
    unit, B buckets and programs, then A's three geometry ports.
  - Writes: A ring, A transfer engine, A's two geometry ports, B WRITE_IMAGE, B `xfer_lin`, B FILL / COPY, B band flush.
  - Up to 64 reads in flight; the last 4 slots are kept for scanout.
- **One address space** (1 GB):

  | range | use |
  |---|---|
  | 0–64 MB | A's ingress ring |
  | 64–192 MB | A's triangle records: two frame banks of 2¹⁷ records × 512 B |
  | 192–256 MB | A's bin lists: two pools of 2²² entries × 8 B |
  | 256–448 MB | host heap for A's buffers (vertex and index buffers, programs, the vertex ring, kept buffers) |
  | 448–464 MB | B's per-draw state (8192 draw ids × 2 KB) |
  | 464 MB–1 GB | host heap for B's images, programs and buffers |

- **STATUS flags bit 29** (A's flags) says "one address space". The host then uses these two heaps.
- **Bounds:** from eb90593 on, A refuses a CREATE_BUFFER outside 256–448 MB, and B an image, program or transfer outside
  464 MB–1 GB, with ERR_HEAP (4). S 158 predates this: it accepts A's buffers from 384 MB and B's from 256 MB.
- A's MEM_READ, MEM_WRITE and MEMTEST are not bounded, so debug tools can read A's records; they reach every region.
- **In simulation** `board_ab_core` stops if B writes below its draw buckets, if A's geometry writes leave the record
  banks and bin pools, or if A's transfer engine writes outside A's heap.

## 6. The host protocol

- **Datagrams:** UDP with an 8-byte little-endian header: magic 0x4246, version 1, type and a 32-bit sequence number
  (0 = unsequenced).
- **Ordering and flow control:**
  - The board takes a datagram only at the expected sequence number and only if its queue has room.
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
- **Handles** are 16 bits: bit 15 picks the half (A or B), bit 14 buffer or image, bits 9:0 the index. The host
  allocates them.
- **STATUS:** layout 10. The flags that describe this build:
  - A's flags: bit 29 one address space, bit 30 no integer divider, bit 31 no SIN / COS (the vertex core);
  - B's flags: bit 30 no integer divider, bit 31 no SIN / COS (the fragment core), bits 28 / 29 clear (no SQRT / RCPM /
    RSQM, no CLAMP).
- **Implementations:** `host/fb/fbproto.py` is the reference; `host/win/core` is the C one the libraries use.

## 7. Number formats and shader cores

- **FP24** for every shader value: sign 1, exponent 7 (bias 63), mantissa 16.
  - Rounding is round-to-nearest-even, with denormals flushed to zero.
  - Relative precision is 2⁻¹⁷ and the range about 2^±62, which meets GLSL `highp`.
  - There is no FP16 path.
- **Integers** are 24-bit two's complement in a lane.
- **Positions:** window x / y are integers in 1/16 pixel, and edge values are exact integers.
- **Depth** is unorm 24 or 16 bits; **colour** is in channel units.
- **ISA:** 64-bit instruction words.
  - A vec4 instruction runs as one scalar micro-op per written component, across the warp's 4 lanes.
  - All three sources take swizzles and modifiers.
  - DP3, DP4 and varying loads run as accumulator chains.
- **Fragment core** (`shader_core_fs2`):
  - A warp is 4 lanes, one quad; there are 4 warps.
  - 32 vec4 registers per lane, 64 uniforms, 8 varyings (perspective-correct), 8 samplers, 4096 instructions.
  - 4 multiply-add lanes.
  - One quarter-rate unit of each kind: the SFU (RCP, RSQRT, LOG2, EXP2), conversions, and integer multiply.
    - Lane *i* goes through it in cycle *i*, so a quarter-rate micro-op takes 4 cycles.
    - Up to 2 such micro-ops in flight per unit class.
  - Up to 4 TEX instructions live.
  - Branches, loops, DISCARD, DDX / DDY.
- **Vertex core** (`shader_core_vs2`): the same design with vertices in the lanes, 4 warps of 4.
  - 16 attributes, 256 uniforms with run-time indexing, outputs gl_Position + gl_PointSize + 16 varyings.
  - No texture, derivative or discard instructions.
- **No SIN / COS units and no integer divider** in either core:
  - SIN, COS, IDIV, IMOD, UDIV and UMOD stop the warp, as IOP does. STATUS says so (section 6), and the compilers lower
    them to float code: a range reduction and a polynomial for SIN / COS, float operations for integer division.
  - There are no SQRT / RCPM / RSQM micro-ops either; the compilers use RSQRT or RCP with a multiply.

## 8. Texture unit (`tmu24` + `tex_cache`)

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
  - With 4 one-quad warps, too few quads are in flight to hide the round trip to DDR: a textured quad costs about
    46 cycles in practice (see [PERF_TESTS.md](PERF_TESTS.md)).
- **Cache:** 64 KB (512 sets × 4 lines × 32 bytes).
- **Not in hardware:** 3D textures, compressed formats, anisotropic filtering and MSAA.

## 9. Frame organisation

- **Descriptors:** 1024 buffer descriptors per half, and 1024 image descriptors in B.
- **Bands:** 8 pixel rows for targets up to 512 wide, 4 rows above that, up to 800 wide.
- **B's band buffer:**
  - Three banks of 2048 cells. A cell is 2 pixels of colour, depth, stencil and an epoch.
  - The next band is loaded while the current one renders.
  - A D24S8 target is loaded and flushed per band, so depth and stencil persist between passes.
- **Triangle records:** up to 2¹⁷ a frame (each primitive, and each piece of a clipped one, is a record).
- **Render stream:** 64-bit beats.
  - QUAD items take 1 beat.
  - TRI items take 13 + 5 per varying beats.
  - BAND, FRAME_END and NOP mark the structure.

## 10. Configuration

Every cut that fits the design into one XC7A200T is a parameter of `board_ab_top`, set from the environment when
building (the table in [README.md](README.md#the-one-board-build)).
- **Lanes:** 16, 8 or 4 per core (`VS_NL`, `B3D_NL`). The quarter-rate units scale with them, one per 4 lanes. Lane
  4k + i goes through unit k in cycle i, so a 16-lane core is cycle-identical to the two boards' one.
- **Warps:** `B3D_NWARP`, `VS_NWARP`.
- **Optional units:** `INT_DIV`, `SINCOS`, `B3D_XSFU`.
- **Queues:** `B3D_QRINF`, `B3D_TEXQ`, `B3D_NTS`.
- **The ROP's shared blend unit:** `B3D_ROP_BLEND1`.
- **Shared SFU tables:** `SFU_TAB2`.
- **Profile counters:** `GEOM_PROF`.
- **Sizes:** `B3D_MAXW`, `DISP_MAXW`, `B3D_CELLS`, `BAND8_MAXW`, `DISP_LB_LINES`.

A larger FPGA can take the two boards' values back one by one. The heap bounds and the draw-bucket base are
`board_ab_core` parameters (`A_HEAP_LO`, `A_HEAP_HI`, `B_HEAP_LO`, `B3D_DS_BASE`); in simulation they are checked
against each other and against A's regions.

## 11. Clocks and resources

- **Clocks:**
  - 100 MHz system clock for everything: the DDR3 controller's user clock (the stock MIG, DDR3 at 800 MT/s). The link
    loop runs on it too.
  - Ethernet receive runs at 125 or 25 MHz; video uses a per-mode pixel clock.
- **Utilisation:**

  | build | timing (setup / hold) | LUT | FF | slices | BRAM tiles | DSP |
  |---|---|---|---|---|---|---|
  | S 158 | +0.001 / +0.035 ns | 116 936 (87.4 %) | 96 340 | 33 134 (99.1 %) | 261 (71.5 %) | 262 |

  At 99 % of the slices most of a failing path's delay is routing, and there is almost no timing margin.
- **Throughput limits:**
  - B fills at most 1 pixel per cycle (100 Mpixels/s); the fragment core holds flat fill to about 66 Mpixels/s.
  - In games the fragment side sets the pace. In OpenArena at 640×480 board B's frame is about 135 ms, with the
    fragment core busy 93 % of it; see [PERF_TESTS.md](PERF_TESTS.md).
