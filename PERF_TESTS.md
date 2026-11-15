# Performance tests: the one-board build

How to measure the one-board GPU (both halves in one board's FPGA, `board_ab_top`), what each test reports, and the
latest results. Builds are named S and their build number, for example S 158. The two-board design's tests and results
are in master's PERF_TESTS.md.

## The build measured

S 158 (`tools/build_board.tcl S`, this branch at 415abdf), loaded over JTAG, measured 2026-11-15.

- **Configuration:**
  - 100 MHz, DDR3 at 800 MT/s;
  - both shader cores at 4 lanes; 4 vertex warps; 4 fragment warps of one quad;
  - no SIN / COS units and no integer divider in either core (the compilers lower them, STATUS flags bits 30 / 31);
  - no SQRT / RCPM / RSQM micro-ops;
  - one blend unit per pixel pair;
  - render targets and video modes up to 800 pixels wide.
- **Utilisation:** 116.9 k LUT (87.4 %), slices 99.1 %, 261 block RAM tiles, 262 DSP, 96.3 k flip-flops.
- **Timing:** met at 100 MHz, WNS +0.001 ns, WHS +0.035 ns.

## Rules for every measurement

- **One program at a time.** The board replies to whoever spoke last, so a second program on it (a test, a status poll,
  a game) steals the first one's acknowledgements and both slow down. Check that no game or test is running first.
- **Check correctness before speed.** Run `fb_cube.py --check` and `fb_3d_test.py` before any sweep. Repeat a run whose
  wall time overshoots its `--secs`.
- **Keep the PC quiet.** No simulations or builds during frame-rate runs; they skew the transfer numbers.
- **Clock:** set `GPU2_CLK_MHZ=100`. The tools and the libraries' statistics convert cycles to ms with it, and default to
  120.
- **Counters are 32-bit.** Take deltas modulo 2³², with a polling interval under 30 s.
- **Host software from this branch:**
  - The `host/fb` tools read STATUS before compiling and lower SIN / COS and integer division for the cores.
  - The libraries need the one-board heaps: board A's buffers in 256–448 MB, board B's images from 464 MB. They take
    them from STATUS flags bit 29, which builds from eb90593 on report.
  - S 158 predates bit 29, so its timedemo below used a test build of `opengl32.dll` with the heaps set by hand.
  - With master's libraries the vertex ring runs over board B's draw buckets at 448 MB, and OpenArena stops after about
    a minute, board B stuck at draw changes.
- **Sizes:** 640×480 and 800×600 only; wider modes and targets are refused.
- **OpenArena timedemos** run at the game's default settings, with the library build named in the results.

## Tests on the board

The Python tools run from `host/fb` (Python 3, standard library).

| test | command | what it measures |
|---|---|---|
| **fb_bench** | `python3 fb_bench.py --mode M --secs 3 [--only a,b]` | 18 rates, each printed as a RESULT line. About 1 min per mode. |
| **fb_cube** | `python3 fb_cube.py --check`; `--cubes N --nohold --secs 6` | exactness against the model (32 cubes, held to vsync); throughput |
| **fb_texperf** | `python3 fb_texperf.py --all --mode M [--check]` | 21 texture cases, cycles per quad; 6 s per case |
| **fb_shader** | `python3 fb_shader.py --shader raymarch\|spiral\|heart --mode M` | fragment-shader-bound full-screen scenes |
| **fb_3d_test** | `python3 fb_3d_test.py 11` (also 13, 15) | exactness of the whole 3D path against the reference models (not speed) |
| **OpenArena timedemo** | `powershell -File tools/oa_timedemo.ps1 -Dll host/win/build/x86/gl/opengl32.dll -Tag <name> -Mode 3\|4` | demo088-test1 frame rate (from `qconsole.log`), plus the library's statistics log at `GPU2_STATS=3` |
| **timedemo statistics** | `powershell -File tools/oa_td_stats.ps1 -Tags a,b -Root <repository>\tools\gen\oa_timedemo` | medians of the statistics lines of the runs named |
| **library statistics** | set `GPU2_STATS=<seconds>` and `GPU2_LOG=<file>` before starting a game | fps and the profile, every N seconds (see below) |

- `fb_bench --mode` and `fb_texperf --mode` take 0 = 640×480 and 1 = 800×600 (the default).
- `oa_timedemo -Mode 3 | 4` is the game's own mode number: 640×480 / 800×600.
- Build the `opengl32.dll` for the timedemo from this branch: `powershell -File host/win/build.ps1 -Arch x86 -Target gl`.
- `oa_td_stats.ps1` needs `-Root` when started with `powershell -File`.
- Not yet tried on one board: `tools/jtag_watch.tcl`, `tools/b_tmu_counters.ps1`, `tools/b_prof.ps1` and the
  Direct3D replays. The one-board build has board A's JTAG status register only.

### fb_bench's rates
- **fill:** fill rate, varying fill, depth pass, depth reject, blend;
- **textures:** tex1 nearest / bilinear / trilinear, tex2, tex4, tex8;
- **ALU:** alu_16, alu_64;
- **triangles:** tri_culled, tri_8px, tri_32px, tri_128px, tri_512px;
- **host transfer:** upload and read-back.

Each test prints a **RESULT** line with fps, the rate, B's frame ms, A's geometry ms and the walk's band visits. Its
**PROFILE** line (A's geometry split) reads 0 % on the one-board build: its geometry profile counters are off.

## Tests in simulation

| bench | command | reports |
|---|---|---|
| `tb_one_board3d` | `powershell -File tools/sim_a.ps1 -Test one3d -Plusargs +SEED_11` | the whole chain on `board_ab_core`, host to B's memory, bit-exact against the models; about 6 min |
| `tb_b_render` | `tools/sim_gpu.ps1 -Test tb_b_render -Extra src/rtl/mem/mem_arb.sv,sim/mig_app_model.sv -Defines TB_ONEB,TB_LANES4,TB_BOARD -Plusargs NOGAP,TMUSTAT` | cycles per frame of the one-board render engine, texture statistics; the frames come from `python3 tools/b_ref.py frame …` in WSL |
| fragment benches | `tb_board_b3d`, `tb_fs2` with `-Defines TB_ONEB,TB_LANES4` | the one-board fragment core: 4 warps, 4 lanes, 4 TEX live, 2 quarter-rate micro-ops in flight, 8 triangle slots, no SQRT / RCPM / RSQM, SIN / COS or divider |
| vertex benches | `tb_a_vertex`, `tb_a_geom`, `tb_a_cmd`, `tb_a_stress` with `-Defines VS_ONEB,VS_NL4` | the one-board vertex core: 4 warps of 4 lanes, no SIN / COS or divider |

Plusargs must not contain `=`, because `xsim.bat` splits there.

## Reading the library statistics (`GPU2_STATS`)

Every *N* seconds the library logs two lines. "Per frame" means divided by the frames presented in that window.

**Line 1** (`stats:`):
- **fps**, then per frame:
  - **draws** and **new draw states**;
  - **passes**: more than 1.0 means the library splits frames, which costs a band pass each time;
  - **KB vertices** (streamed), **KB to keep** and **kept binds** (vertex buffers held in board A's heap),
    **datagrams**.
- **EVENTs lost / resent** and **A gap**: protocol health. Events lost should be 0.
- **B last frame ms** with its shares: flush, band change, draw change, FS busy, band set-up, starved (the render stream
  was empty, so the geometry side is the bound) and ROP stall.
  - This is a single frame sampled in the window, not an average.
  - The shares overlap and do not add up to 100 %.
- **quads / tris:** mid-frame snapshots. Don't quote them as frame totals.
- **A per frame:** geometry ms, walk ms (the walk includes waiting for B's credit) and band visits.

**Line 2** (`A geometry … kcycles/frame`): the geometry stage's split. It reads 0 % on the one-board build (profile
counters off), except the dispatcher's idle and geometry-wait times.

## Results (S 158, 2026-11-15)

### Correctness
- `fb_cube --check`: 0 mismatches (32 cubes at 60 fps, vsync).
- `fb_3d_test` 11, 13, 15: bit-exact.
- `fb_texperf`: 21 / 21 checks ok.
- `fb_shader`: raymarch, spiral and heart pass.
- After the runs STATUS showed 0 command errors and no geometry error.

### OpenArena timedemo (demo088-test1, 640×480, the game's default settings)

`opengl32.dll`: this branch's library at 31a0cf2 with the one-board heaps set by hand (board A's heap 384–448 MB, board
B's from 464 MB, a 32 MB vertex ring), since S 158 has no STATUS bit 29. A simulation was running on the PC during the
first minutes of the run.

| | 640×480 |
|---|---|
| frame rate | **7.2 fps** (3 398 frames in 469.3 s) |
| frame time min / mean / max / std dev | 11.0 / 138.1 / 284.0 / 25.6 ms |
| board B frame (median) | 135.1 ms |
| fragment core busy / stream starved | 93 % / 0 % |
| board A geometry per frame | 24.8 ms |
| events lost | 0 |

The fragment side paces the game: the fragment core is busy 93 % of the frame and never starved, while the geometry takes
a fifth of the frame. 800×600 has not been measured yet.

### fb_bench (`--secs 3`)

| test | 800×600 | 640×480 |
|---|---|---|
| fill_flat | 66.3 Mpixels/s | 66.1 |
| fill_varying | 39.2 Mpixels/s | 39.2 |
| fill_depth_pass | 66.2 Mpixels/s | 66.2 |
| fill_depth_reject | 170.3 Mpixels/s | 170.7 |
| fill_blend | 66.1 Mpixels/s | 66.0 |
| tex1 nearest / bilinear / trilinear | 17.4 / 17.3 / 17.6 Msamples/s | 17.2 / 17.1 / 17.5 |
| tex2_bilinear | 25.8 Msamples/s | 25.6 |
| tex4_bilinear | 27.3 Msamples/s | 26.8 |
| tex8_bilinear | 18.4 Msamples/s | 18.4 |
| alu_16 / alu_64 | 260 / 301 Mscalar ops/s | 260 / 301 |
| tri_culled | 3.51 M triangles/s | 2.12 |
| tri_8px / 32px / 128px / 512px | 2.06 / 1.12 / 0.39 / 0.11 M triangles/s | 1.71 / 1.13 / 0.38 / 0.11 |
| upload / read-back | 61.2 / 39.6 MB/s | 56.3 / 36.5 |

- tri_culled and tri_8px draw fewer triangles a frame at 640×480, so their rates there are lower.
- Fill runs at about 0.66 pixels per cycle, with the fragment core busy the whole frame.
- The ALU tests reach 65–75 % of the 4 lanes' 400 M operations a second.
- Texturing is latency-bound: 4 warps of one quad each are too few quads in flight to hide the texture round trip.

### fb_texperf (800×600, cycles per quad)

Every case takes about 4.7 times its ideal (the cycles its instructions need), for the same reason.

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
