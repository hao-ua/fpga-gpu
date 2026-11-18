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
| **fb_bench** | `python3 fb_bench.py --mode M --secs 3 [--only a,b]` | 21 rates, each printed as a RESULT line and a PROFILE line (see below). About 1 min per mode. |
| **fb_cube** | `python3 fb_cube.py --check`; `--cubes N --nohold --secs 6` | exactness against the model (32 cubes, held to vsync); throughput |
| **fb_texperf** | `python3 fb_texperf.py --all --mode M [--check]` | 21 texture cases, cycles per quad; 6 s per case |
| **fb_shader** | `python3 fb_shader.py --shader raymarch\|spiral\|heart --mode M` | fragment-shader-bound full-screen scenes |
| **fb_3d_test** | `python3 fb_3d_test.py 11` (also 13, 15) | exactness of the whole 3D path against the reference models (not speed) |
| **fb_skip_test** | `python3 fb_skip_test.py` | band skipping: partial draws with and without it read back identical, ms per FLUSH |
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
- **partial redraws:** partial_2bands, partial_8bands (a full-width strip of 16 / 64 rows into a target that is not
  cleared), flush_empty (no draw, no clear): ms per pass with band skipping, `--no-skip` for every band walked;
- **host transfer:** upload and read-back.

Each test prints a **RESULT** line with fps, the rate, B's frame ms, A's geometry ms and the walk's band visits. Its
**PROFILE** line has A's geometry split, and its **CZ** line board A's coarse depth and TRI items (see below). Both board-A
features are on, as in the libraries; `--no-zc` sends no coarse-depth flags, `--no-lazy` sends every TRI item.

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

**Line 3** (`A per frame: … quads walked`, STATUS layout 12): board A's coarse depth and TRI items per frame.
- **quads walked / culled / sent:** A's walk found them; it did not send the culled ones, because board B's depth test
  would have dropped them anyway (the draws' flags allow it, see `docs/OVERDRAW.md` §7);
- **TRI items sent / held back / sent with no quad:** a triangle's setup goes to B only in a band where it sends a quad;
  the held-back ones are what that saves; "sent with no quad" is the waste when lazy TRI items are off;
- **blocks lowered:** the coarse-depth bounds that opaque draws lowered.

The two board-A features have switches in the libraries, both on by default: `GPU2_COARSE_Z=0` (no culling) and
`GPU2_LAZY_TRI=0` (every TRI item sent); the line names the one that is off. The line ends with the **bands left out**
per frame (STATUS layout 13; `GPU2_SKIP_BANDS=0` walks every band).

## Latest results

Only results measured on the current hardware are kept; each names the builds it was measured on. The board tests above
are to be measured on the current pair and recorded here.

### OpenArena timedemo (demo088-test1, the game's default settings)

A 166 / B 162 (the flashed pair), 2026-11-18, `opengl32.dll` built from 5cacc4b. Board A's coarse depth and lazy TRI items on
(the libraries' default):

| | 640×480 | 800×600 |
|---|---|---|
| frame rate | **30.5 fps** (3 398 frames in 111.5 s) | **23.8 fps** (3 398 frames in 142.7 s) |
| frame time min / mean / max / std dev | 9.0 / 32.8 / 80.0 / 10.3 ms | 9.0 / 42.0 / 101.0 / 10.5 ms |
| board B frame (median) | 28.3 ms | 41.1 ms |
| fragment core busy / stream starved | 87 % / 9 % | 86 % / 11 % |
| board A geometry / walk per frame | 21.3 / 27.6 ms | 21.3 / 41.0 ms |
| A per frame: quads walked / culled | 801 k / 314 k (39 %) | 1.21 M / 495 k (41 %) |
| A per frame: TRI items sent / held back | 20.4 k / 11.8 k | 24.3 k / 13.9 k |
| events lost | 0 | 0 |

On A 154 / B 142 the same demo ran at 31.4 / 23.9 fps (board B's frame 25.6 / 40.2 ms, A's geometry 20.6 ms). Board A's
wider clipper (A 163 and later) adds about 0.7 ms of geometry a frame.

The four combinations of the two switches (frame rate, B frame median), measured on A 153 / B 142 (A 154's design sends
the same stream; a board build was running on the same PC):

| | both on | coarse depth only | lazy TRI only | both off |
|---|---|---|---|---|
| 640×480 | **31.3 fps**, 27.9 ms | 30.5 fps, 28.5 ms | 29.7 fps, 31.2 ms | 29.2 fps, 33.4 ms |
| 800×600 | **24.0 fps**, 40.5 ms | 23.2 fps, 41.1 ms | 22.0 fps, 44.9 ms | 21.6 fps, 45.5 ms |

Culling takes about 40 % of the quads off board B, and lazy TRI items another third of the TRI items. Together they cut B's
frame by 16 % at 640×480 and 11 % at 800×600. At 640×480 board B is no longer the only bound: its frame (26–28 ms) is
shorter than the game's (32 ms), and the stream is starved 10 % of the time.

### Other games
- **NFS Hot Pursuit 2** (Direct3D 8, 800×600, A 147 / B 142): 22–36 fps in races.
- **Half-Life** (OpenGL, 640×480, A 139 / B 142): about 60 fps.

### Board tests
- **Shadertoy shaders through `gles_shader.exe`** (A 166 / B 162, 640×480): tunnel 1 069 ms per frame, Seascape
  1 453–1 469 ms, ray marcher (`tests/fs_raymarch_soft.frag`) 351 ms. On A 139 / B 142 they took 1 064–1 080, 1 467–1 484
  and 364–378 ms.
- **fb_bench** (A 166 / B 162, 800×600, `--secs 3`, both board-A features on):

  | test | rate | | test | rate |
  |---|---|---|---|---|
  | fill_flat | 118.8 Mpixels/s | | tex4_bilinear | 101.1 Msamples/s |
  | fill_depth_pass | 118.7 Mpixels/s | | tex8_bilinear | 86.1 Msamples/s |
  | fill_depth_reject | **310.8 Mpixels/s** | | alu_16 / alu_64 | 1 279 / 1 454 Mscalar ops/s |
  | fill_blend | 118.7 Mpixels/s | | tri_culled | 3.61 M triangles/s |
  | tex1 nearest / bilinear / trilinear | 110.4 / 105.1 / 54.0 Msamples/s | | tri_8px / 32px / 128px / 512px | 2.69 / 1.80 / 0.63 / 0.19 M triangles/s |
  | tex2_bilinear | 108.6 Msamples/s | | upload / read-back | 51.3 / 29.9 MB/s |
  | copy 1:1 / stretched | 344.3 / 112.9 Mpixels/s | | copy of rectangles, a command each / one list | 92.3 / 94.2 Mpixels/s |

  fill_depth_reject draws 12 layers with 11 hidden: board A culls 91 % of its quads. The rates match A 153 / B 142's
  within 2 %, except host transfer: upload and read-back were 60.0 / 34.4 MB/s then.
- **fb_cube --check** (A 166 / B 162): 0 mismatches; 32 cubes at 59.9 fps (vsync) for 5 minutes, 0 underruns, no errors.
  **fb_3d_test** 11, 13, 15: bit-exact.
- **Band skipping** (A 166 / B 162): `fb_skip_test.py` — the same partial draws with and without FLUSH flags bit 10 read
  back byte-identical, the bands not drawn untouched (800×600 and 1280×720). FLUSH to FENCE for a redraw of one band 0.82
  ms against 4.33 ms (800×600), 0.66 against 7.53 ms (1280×720). fb_bench at 800×600:

  | test | with skipping | every band walked |
  |---|---|---|
  | partial_2bands | 1 154 fps (B's frame 0.38 ms) | 237 fps (3.83 ms) |
  | partial_8bands | 894 fps (0.70 ms) | 235 fps (3.85 ms) |
  | flush_empty | 2 885 fps (0.01 ms) | 238 fps (3.82 ms) |
- **fb_texperf** (A 166 / B 162, 800×600, `--all`, every case checked against the model):

  | case | fps | cycles per quad | | case | fps | cycles per quad |
  |---|---|---|---|---|---|---|
  | t1_256_bilinear | 97.3 | 10.0 | | t2_2048tri_256 | 68.8 | 14.4 |
  | t1_256_nearest | 96.8 | 10.0 | | t2_2048tri_256_min6 | 68.1 | 14.5 |
  | t1_256_565_bilinear | 96.7 | 10.1 | | t4_2x2048tri_2x256 | 38.3 | 25.9 |
  | t1_2048_mag1 | 96.9 | 10.0 | | t4_2x2048tri_2x256_min6 | 34.0 | 29.2 |
  | t1_2048_min8 | 52.3 | 18.9 | | t4_4x256_bilinear | 45.0 | 22.1 |
  | t1_2048_tri_mag1 | 97.1 | 10.1 | | cube_256_bilinear | 94.2 | 10.4 |
  | t1_2048_tri_min8 | 94.5 | 10.3 | | cube_256_trilinear | 94.1 | 10.4 |
  | t1_2048_tri_min6 | 94.4 | 10.3 | | cube_256_tri_lv7 / lv8 | 94.1 / 93.9 | 10.4 |
  | t1_2048_mipn_min8 | 96.7 | 10.1 | | tunnel | 30.3 | 32.8 |
  | t1_2048_565_tri_min8 | 96.0 | 10.1 | | tunnel_naive | 45.8 | 21.7 |
- **fb_shader** (A 166 / B 162, 800×600): raymarch 4.44 fps (227.3 cycles per quad), spiral 34.7 (28.6), heart 14.5
  (68.3).
- **Board link during these tests** (A 166 / B 162): board B's receiver corrected 6 CRC errors in about 3.5 G words.

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
