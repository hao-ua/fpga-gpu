# Performance tests

How to measure the GPU, what each test reports, and the latest results. Builds are named by board and build number,
for example A 147 / B 142. Both boards run at 120 MHz.

## Rules for every measurement

- **One program at a time.** Board A replies to whoever spoke last, so a second program on the boards (a test, a
  status poll, a game) steals the first one's acknowledgements and both slow down. Check that no game or test is
  running first.
- **Check correctness before speed.** Run `fb_cube.py --check` and `fb_3d_test.py` before any sweep. Repeat a run whose
  wall time overshoots its `--secs`.
- **Keep the PC quiet.** No simulations or builds during frame-rate runs; they skew the transfer numbers.
- **Clock scaling.**
  - The tools convert cycles to ms with `GPU2_CLK_MHZ` (default 120).
  - Logs written before 2026-11-08 at 120 MHz show ms 1.2× too large; runs at 110 MHz (2026-11-04 to 11-07) need ×100/110.
  - `tools/b_tmu_counters.ps1` still divides by a fixed 100 MHz.
- **Counters are 32-bit.** Take deltas modulo 2³², with a polling interval under 30 s.
- **OpenArena timedemos** run at the game's default settings, with the driver build named in the results.
- **Simulation overstates gains:** board B's gains in simulation reached the board at about a third of their size,
  because the bench's lock-step warps overstate contention.

## Tests on the boards

The Python tools run from `host/fb` (Python 3, standard library).

| test | command | what it measures |
|---|---|---|
| **fb_bench** | `python3 fb_bench.py --mode M --secs 3 [--only a,b]` | 18 rates, each printed as a RESULT line and a PROFILE line (see below). About 1 min per mode. |
| **fb_cube** | `python3 fb_cube.py --check`; `--cubes N --nohold --secs 6` | exactness against the model (32 cubes, held to vsync); throughput |
| **fb_texperf** | `python3 fb_texperf.py --all --mode M [--check]` | 21 texture cases, cycles per quad; 6 s per case |
| **fb_shader** | `python3 fb_shader.py --shader raymarch\|spiral\|heart --mode M` | fragment-shader-bound full-screen scenes |
| **fb_3d_test** | `python3 fb_3d_test.py 11` (also 13, 15) | exactness of the whole 3D path against the reference models (not speed) |
| **OpenArena timedemo** | `powershell -File tools/oa_timedemo.ps1 -Dll host/win/build/x86/gl/opengl32.dll -Tag <pair> -Mode 3\|4` | demo088-test1 frame rate (from `qconsole.log`), plus the library's statistics log at `GPU2_STATS=3` |
| **timedemo statistics** | `powershell -File tools/oa_td_stats.ps1 -Tags a,b` | medians of the statistics lines of the runs named |
| **JTAG status watch** | `vivado.bat -mode batch -nojournal -nolog -source tools/jtag_watch.tcl -tclargs <log> [interval_s]` | every STATUS word of both boards, and their temperatures, into `<log>.raw`. JTAG only, so it can run beside a game. |
| **texture counters** | `powershell -File tools/b_tmu_counters.ps1 -Raw <log>.raw [-From -To]` | from a JTAG watch: texture jobs, stalls, cache hits and misses, quads, early-depth drops, triangles, frames |
| **B frame profile** | `powershell -File tools/b_prof.ps1 -Raw <log>.raw [-Mhz 120]` | B's frame split: warps, texture round trip. Needs a board B built with its profile counters on (`PROF=1` in `b_render`); the standard builds have them off and read zeros. |
| **library statistics** | set `GPU2_STATS=<seconds>` and `GPU2_LOG=<file>` before starting a game | fps and both boards' profile, every N seconds (see below) |
| **replays** | `host/win/build/x86/d3d9/d3_replay <file.g2rc> <outdir>` | replays a Direct3D recording (`GPU2_RECORD`) on the boards, ms per frame |

`fb_bench --mode` and `fb_texperf --mode` take: 0 = 640×480, 1 = 800×600 (the default), 2 = 1280×720, 3 = 1280×800,
4 = 1024×768. `oa_timedemo -Mode 3 | 4` is the game's own mode number: 640×480 / 800×600.

### fb_bench's rates
- **fill:** fill rate, depth pass, depth reject, blend;
- **textures:** tex1 nearest / bilinear / trilinear, tex2, tex4, tex8;
- **ALU:** alu_16, alu_64;
- **triangles:** tri_culled, tri_8px, tri_32px, tri_128px, tri_512px;
- **host transfer:** upload and read-back.

Each test prints a **RESULT** line with fps, the rate, B's frame ms, A's geometry ms and the walk's band visits. Its
**PROFILE** line has A's geometry split.

## Tests in simulation

| bench | command | reports |
|---|---|---|
| `tb_b_render` | `tools/sim_gpu.ps1 -Test tb_b_render -Extra src/rtl/mem/mem_arb.sv,sim/mig_app_model.sv -Defines TB_NWARP8,TB_BOARD -Plusargs NOGAP,TMUSTAT`; frame sets: `tools/run_b_frames.ps1 -Frames hid3,vis3,fill3,qmod,oa,g1` | cycles per frame of B's render engine, texture statistics. The frames come from `python3 tools/b_ref.py frame …` in WSL. |
| `tb_a_setup_cyc` | `tools/sim_gpu.ps1 -Test tb_a_setup_cyc -Plusargs MEMSLOW,CULL,CLIP,FSMASK0` | cycles per triangle in A's setup: drawn, culled, clipped, by varying count |
| `tb_a_render` | `tools/run_a_seeds.ps1 -Test tb_a_render -Seeds 41 -Plusargs FASTSINK,RNPROF` | the band walk's cycles per visit |
| `tb_tmu24` | `tools/sim_gpu.ps1 -Test tb_tmu24` or `tools/run_tmu24_seeds.ps1` | cold and warm cycles per quad of the texture unit |

Plusargs must not contain `=`, because `xsim.bat` splits there.

## Reading the library statistics (`GPU2_STATS`)

Every *N* seconds the library logs two lines. "Per frame" means divided by the frames presented in that window.

**Line 1** (`stats:`):
- **fps**, then per frame:
  - **draws** and **new draw states**;
  - **passes**: more than 1.0 means the library splits frames, which costs a band pass each time;
  - **KB vertices** (streamed), **KB to keep** and **kept binds** (vertex buffers held on board A), **datagrams**.
- **EVENTs lost / resent** and **A gap**: protocol health, should be 0.
- **B last frame ms** with its shares: flush, band change, draw change, FS busy, band set-up, starved (the render stream
  was empty, so A or the link is the bound) and ROP stall.
  - This is a single frame sampled in the window, not an average.
  - The shares overlap and do not add up to 100 %.
- **quads / tris:** mid-frame snapshots. Don't quote them as frame totals.
- **A per frame:** geometry ms, walk ms (the walk includes waiting for B's credit) and band visits.

**Line 2** (`A geometry … kcycles/frame`): A's geometry-busy cycles per frame and the share of
them each stage was busy. The shares overlap.
- **load:** program loads;
- **copy:** uniform and descriptor copies;
- **feed:** the vertex feed;
- **fetch-wait:** the feed's refused DDR requests;
- **core:** the vertex core;
- **asm:** the primitive assembler;
- **vo-stall:** a vertex waiting for the setup;
- **setup:** primitive setup;
- **push-stall:** the setup waiting for the binner;
- **bin:** the binner;
- **clear:** the frame's bin clear.

Then loads / draws / primitives per frame, the dispatcher's idle ms (no command waiting: host-bound) and its geometry
wait ms.

## Latest results

Only results measured on the current hardware are kept; each names the builds it was measured on. The board tests above
are to be measured on the current pair and recorded here.

### OpenArena timedemo (demo088-test1, the game's default settings)

A 148 / B 142, 2026-11-14, `opengl32.dll` built from f205f6f. A board build was running on the same PC during both runs.

| | 640×480 | 800×600 |
|---|---|---|
| frame rate | **28.6 fps** (3 398 frames in 119.0 s) | **21.1 fps** (3 398 frames in 161.1 s) |
| frame time min / mean / max / std dev | 12.0 / 35.0 / 314.0 / 11.6 ms | 11.0 / 47.4 / 147.0 / 11.5 ms |
| board B frame (median) | 31.7 ms | 47.7 ms |
| fragment core busy / stream starved | 76 % / 3 % | 77 % / 3 % |
| board A geometry / walk per frame | 20.5 / 32.0 ms | 20.4 / 47.0 ms |
| events lost | 0 | 0 |

Board B paces the game at both sizes. A's geometry takes about 20 ms a frame whatever the size, and its walk follows B's
frame.

### Other games
- **NFS Hot Pursuit 2** (Direct3D 8, 800×600, A 147 / B 142): 22–36 fps in races.
- **Half-Life** (OpenGL, 640×480, A 139 / B 142): about 60 fps.

### Board tests
- **Shadertoy shaders through `gles_shader.exe`** (A 139 / B 142, 640×480): tunnel 1 064–1 080 ms per frame, Seascape
  1 467–1 484 ms, ray marcher 364–378 ms.
- **fb_bench, fb_texperf, fb_cube, fb_shader:** to be measured on the current pair.

### Simulation (the current board B options)
- **`tb_b_render` frame cycles:**

  | frame | cycles |
  |---|---|
  | hid3 | 41 689 |
  | vis3 | 67 128 |
  | fill3 | 65 674 |
  | qmod | 52 073 |
  | cm | 51 076 |
  | smod | 143 155 |
  | oa | 171 146 |
  | g1 | 112 333 |

## One-board build (board_ab_top)

Board A's FPGA running both halves, one DDR3 and no board link: build S 158 (`tools/build_board.tcl S`, this branch at
415abdf), measured 2026-11-15 on board A over JTAG, board B idle.

- **Configuration:** 100 MHz; both shader cores 4 lanes (16 on the two boards); 4 fragment warps; no SIN / COS units and no
  integer divider in either core (the compilers lower them, STATUS flags bits 30 / 31); no SQRT / RCPM / RSQM micro-ops;
  one blend unit per pixel pair; targets and video modes up to 800 px wide; no coarse depth or lazy TRI items on board A.
- **Utilisation:** 116.9 k LUT (87.4 %), slices 99.1 %, 261 block RAM tiles, 262 DSP, 96.3 k flip-flops; timing met at
  100 MHz (WNS +0.001 ns, WHS +0.035 ns).
- **How to measure it:**
  - Set `GPU2_CLK_MHZ=100` for the tools' ms.
  - The `host/fb` tools must lower for the cores; they do so from STATUS since 31a0cf2.
  - The libraries need the one-board heaps: board A's buffers in 256-448 MB, board B's images from 464 MB. STATUS flags
    bit 29 says so from eb90593 on, and this branch's libraries follow it from 0297942. S 158 predates bit 29, so the
    timedemo below used a test build of `opengl32.dll`: 31a0cf2 with board A's heap at 384-448 MB, board B's from 464 MB
    and a 32 MB vertex ring.
  - With the two boards' heaps, the vertex ring runs over board B's draw buckets at 448 MB: OpenArena then stops after a
    minute, board B stuck at draw changes.

### Correctness
`fb_cube --check`: 0 mismatches (32 cubes at 60 fps, vsync). `fb_3d_test` 11, 13, 15: bit-exact. `fb_texperf`: 21 / 21
checks ok. `fb_shader`: raymarch, spiral, heart pass.

### OpenArena timedemo (demo088-test1, 640×480, the game's default settings)

The two-board column is A 153 / B 142 with board A's coarse depth and lazy TRI items off (S 158 has neither); with both on,
A 154 / B 142 reaches 31.4 fps. A simulation was running on the PC during the first minutes of the S 158 run.

| | S 158 (one board, 100 MHz) | A 153 / B 142 (two boards, 120 MHz) |
|---|---|---|
| frame rate | **7.2 fps** (3 398 frames in 469.3 s) | 29.2 fps |
| frame time min / mean / max / std dev | 11.0 / 138.1 / 284.0 / 25.6 ms | |
| board B frame (median) | 135.1 ms | 33.4 ms |
| fragment core busy / stream starved | 93 % / 0 % | |
| board A geometry per frame | 24.8 ms | 20.6 ms (A 154) |
| events lost | 0 | 0 |

The fragment side paces the game: its frame is four times the two boards', while board A's geometry costs only a fifth more.

### fb_bench (800×600, `--secs 3`)

Two-board column: A 153 / B 142 with board A's features on, the same measurement as the two-board results above. Clock
alone would give 0.83×.

| test | S 158 | A 153 / B 142 | ratio |
|---|---|---|---|
| fill_flat | 66.3 Mpixels/s | 118.9 | 0.56× |
| fill_varying | 39.2 Mpixels/s | | |
| fill_depth_pass | 66.2 Mpixels/s | 118.7 | 0.56× |
| fill_depth_reject | 170.3 Mpixels/s | 310.6 (219.2 without culling) | 0.78× of 219.2 |
| fill_blend | 66.1 Mpixels/s | 116.8 | 0.57× |
| tex1 nearest / bilinear / trilinear | 17.4 / 17.3 / 17.6 Msamples/s | 109.5 / 104.7 / 54.0 | 0.16 / 0.17 / 0.33× |
| tex2_bilinear | 25.8 Msamples/s | 108.6 | 0.24× |
| tex4_bilinear | 27.3 Msamples/s | 101.0 | 0.27× |
| tex8_bilinear | 18.4 Msamples/s | 86.1 | 0.21× |
| alu_16 / alu_64 | 260 / 301 Mscalar ops/s | 1 279 / 1 454 | 0.20 / 0.21× |
| tri_culled | 3.51 M triangles/s | 3.60 | 0.97× |
| tri_8px / 32px / 128px / 512px | 2.06 / 1.12 / 0.39 / 0.11 M triangles/s | 2.67 / 1.80 / 0.62 / 0.19 | 0.77 / 0.62 / 0.63 / 0.58× |
| upload / read-back | 61.2 / 39.6 MB/s | 60.0 / 34.4 | 1.02 / 1.15× |

At 640×480 the rates are the same within 3 %, except tri_culled and tri_8px (2.12 / 1.71 M triangles/s: those tests draw
fewer triangles a frame there) and upload / read-back (56.3 / 36.5 MB/s).

- The ALU rate is the lane count times the clock: 4 lanes / 16 × 100 / 120 = 0.21×.
- Texturing loses most: with 4 warps of one quad each, too few quads are in flight to hide the texture latency.
- Board A's triangle rate barely changes.

### fb_texperf (800×600, cycles per quad)

Every case is about 4.7× its ideal (the cycles its instructions need); no two-board run of it is recorded yet.

| case | fps | cycles / quad | ideal |
|---|---|---|---|
| t1_256_bilinear / nearest / 565_bilinear | 17.9 / 17.9 / 18.0 | 46.2 / 46.3 / 46.0 | 9.8 |
| t1_2048_mag1 / tri_mag1 | 17.7 / 17.7 | 46.8 / 46.8 | 9.8 |
| t1_2048_min8 / tri_min8 / tri_min6 | 16.6 / 17.6 / 17.0 | 50.1 / 47.2 / 48.9 | 9.8 |
| t1_2048_mipn_min8 / 565_tri_min8 | 18.0 / 17.8 | 46.1 / 46.6 | 9.8 |
| t2_2048tri_256 / _min6 | 13.4 / 13.3 | 62.1 / 62.4 | 13.8 |
| t4_2x2048tri_2x256 / _min6 / t4_4x256_bilinear | 9.0 / 8.9 / 9.0 | 92.2 / 92.7 / 91.7 | 22.2 |
| cube_256 bilinear / trilinear / tri_lv7 / tri_lv8 | 4.6 | 180.2 | 29.0 |
| tunnel / tunnel_naive | 5.0 / 7.0 | 165.6 / 118.9 | 27.2 / 18.8 |

### fb_shader (800×600, 120 400 quads a frame)

| scene | fps | cycles / quad | fragment program |
|---|---|---|---|
| raymarch (32 steps) | 0.55 | 1 498 | 340 instructions |
| spiral | 3.70 | 223 | 94 instructions |
| heart | 2.28 | 364 | 198 instructions |
