# fpga-gpu — a two-board FPGA GPU

An OpenGL ES 2.0 / OpenGL 2.0 / Direct3D 9 (shader model 2.0) class GPU built from two Artix-7 boards. A Windows PC
sends commands over gigabit Ethernet to **board A**, which runs the vertex shaders, clips, sets up and bins the
primitives. Board A streams them over a ribbon-cable link to **board B**, which runs the fragment shaders, textures,
blends and shows the picture on HDMI. Drop-in Windows libraries (`ddraw.dll`, `opengl32.dll`, `libGLESv2` / `libEGL`,
`d3d9.dll`, `d3d8.dll`) let unmodified games render on it.

**One-board variant:** the [`single-board`](../../tree/single-board) branch fits both halves into one board's FPGA and
DDR3, with no ribbon cables. It keeps the same host protocol and the OpenGL ES 2.0 feature set, at a lower speed:
- 4 shader lanes instead of 16, at 100 MHz;
- no SIN / COS or integer-divide units (the compilers lower them);
- targets and video modes up to 800×600.

OpenArena's timedemo runs at 7.2 fps at 640×480 there, against 29 fps on the two boards. How to build and run it:
[that branch's README](../../blob/single-board/README.md#the-one-board-build). Build its libraries from that branch too:
only they know the one-board memory map.

| document | contents |
|---|---|
| this file | what you need, and how to build, load, simulate and run it |
| [ARCHITECTURE.md](ARCHITECTURE.md) | how the two boards, the link and the host protocol work |
| [FEATURES.md](FEATURES.md) | what the hardware and each library support, and what they refuse |
| [PERF_TESTS.md](PERF_TESTS.md) | the performance tests, how to run them, the latest results |

## Repository layout

| path | contents |
|---|---|
| `src/rtl/`, `src/constrs/` | the RTL of both boards and their constraints. `tools/build_board.tcl` lists exactly which files each board reads. |
| `ip/mig_7series_0/` | the DDR3 memory controller project (`mig_a.prj`), generated into `tools/gen/ip` by `tools/gen_mig.tcl` |
| `tools/` | build scripts, simulation runners, reference models and golden generators (Python), JTAG status tools, performance scripts |
| `bringup/` | load a bitstream over JTAG, write the QSPI flash, boot from the flash |
| `sim/` | simulation benches and models |
| `tests/` | shader test programs: assembly (`.sasm`) and GLSL |
| `host/win/` | the Windows libraries, the shader compiler (`shc`), test programs and game launchers |
| `host/fb/` | Python host tools: the protocol library, board status, acceptance tests, benchmarks |

Everything generated goes to `tools/gen/`: the MIG IP, tables, goldens, simulation work directories, bitstreams and
reports. `tools/gen/` is not in git.

## What you need

**Hardware**
- Two Puzhi PA200T-Starlite boards. Each has an XC7A200T-FBG484-2, 1 GB DDR3, an RTL8211F gigabit PHY, HDMI, FT232H
  JTAG and a W25Q128 QSPI flash.
  - **Board A** is the Ethernet side; **board B** is the HDMI side.
  - The JTAG scripts select a board by its JTAG target serial (`<serial>` below). Vivado's hardware manager lists them
    (`get_hw_targets`). Keep yours in `bringup/boards.local.tcl` (`set SER_A …` / `set SER_B …`), which git ignores;
    `tools/jtag_watch.tcl` reads it, or `GPU2_JTAG_A` / `GPU2_JTAG_B`.
- Two straight 40-pin 2.54 mm ribbon cables between the boards: JM1 to JM1 and JM2 to JM2, pin *k* to pin *k*, 3.3 V.
  A sends on JM1 and receives on JM2; B the other way round.
- A PC with a gigabit NIC and a 1000 Mbit/s full-duplex link to board A's RJ45: a direct cable, or a switch or router
  on the PC's network.
- An HDMI monitor that takes 640×480, 800×600, 1024×768, 1280×720 and 1280×800 at 60 Hz.
- One USB JTAG cable per board.
- Air cooling: without it the boards reached 100–105 °C, at or above the part's 100 °C limit.

**Software (Windows 11)**
- **AMD Vivado 2026.1** at `C:\AMDDesignTools\2026.1\Vivado`, with Artix-7 devices.
  - The licence must cover the xc7a200t (the free Basic tier does).
  - Call `vivado.bat` / `xvlog.bat` by their full path; they are not on PATH.
  - Windows Smart App Control had to be turned off before xsim would run.
- **WSL2** (Ubuntu) with **Python 3**, standard library only (no numpy). It runs the golden generators and the table
  generator, and git.
- **MinGW-w64 GCC 8.3.0** for the Windows libraries. The copy that ships inside Vivado is used:
  `C:\AMDDesignTools\2026.1\Model_Composer\tps\mingw\8.3.0\win64.o\nt`. It has multilib (`-m32`) and the
  DirectDraw / Direct3D / GL headers. `GPU2_MINGW` points `host/win/build.ps1` at another copy.
- Python 3.8 or later on the PC or in WSL for `host/fb`, standard library only.

## One-time setup

1. **Generate the DDR3 controller (MIG IP)**, about 5 min:
   ```
   C:\AMDDesignTools\2026.1\Vivado\bin\vivado.bat -mode batch -nojournal -source tools/gen_mig.tcl
   ```
   It creates the IP from `ip/mig_7series_0/mig_a.prj` (DDR3L, 32-bit, 4:1) into `tools/gen/ip/mig_7series_0/`.
2. **Generate the function tables** that both boards' shader units load at synthesis
   (`tools/gen/fp24_{rcp_seed,rsqrt_seed,log2_lut,exp2_lut,sin_lut}.mem`), plus the arithmetic golden vectors. From WSL,
   in the repository root:
   ```
   bash tools/gen_gpu_artifacts.sh
   ```
   `tools/build_board.tcl` stops before taking a build number when the MIG IP or any of the five tables is missing, for
   either board (a missing table is no synthesis error, only an all-zero ROM).
3. **Video modes.** `src/rtl/disp/video_modes.svh` is in git. Regenerate it only after changing the mode list, with
   `python3 tools/mmcm_drp_calc.py`; `tools/sim_a.ps1 -Test video_mmcm` checks the result.
4. **Network.** Board A takes UDP port **5000** (MAC 00:0a:35:01:02:03). Its IPv4 address:
   - Each time its link comes up it asks DHCP for one, under the host name `fpga-gpu`. It checks a leased address with
     ARP probes before using it, and renews the lease at T1 / T2 as RFC 2131 describes.
   - When no lease is bound within 10 s of the link coming up, it uses **192.168.1.10**. It goes on asking; a lease
     bound later replaces that address.
   - Cabled directly to the PC, board A is at 192.168.1.10. Give the PC's NIC a static address in 192.168.1.0/24, for
     example as administrator:
     `New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 192.168.1.112 -PrefixLength 24`.
   - On a network with a DHCP server, set `GPU2_IP` to the leased address or a name for it; many routers put DHCP host
     names in their DNS (`fpga-gpu`, `fpga-gpu.lan`, …).
     - The router's lease list shows the address.
     - So do `python3 host/fb/fb_status.py` (once it can reach the board) and `tools/a_status_jtag.tcl` (over JTAG).
   - The MTU stays at 1500; no jumbo frames are needed.
   - Energy-efficient Ethernet needs no PC change: board A stops its PHY from advertising it.
   - Board A replies to whoever sent last, so only one program may talk to the boards at a time.

## Building the bitstreams

From the repository root, in PowerShell. Both boards run at 120 MHz with DDR3 at 960 MT/s (`OC_MHZ=120`). Without
`OC_MHZ` the build uses the stock 100 MHz / 800 MT/s MIG.

```
$env:OC_MHZ = '120'
C:\AMDDesignTools\2026.1\Vivado\bin\vivado.bat -mode batch -nojournal -source tools/build_board.tcl `
    -log tools/gen/build_board_A_oc120_<yyyy-mm-dd_HHMM>.log -tclargs A
C:\AMDDesignTools\2026.1\Vivado\bin\vivado.bat -mode batch -nojournal -source tools/build_board.tcl `
    -log tools/gen/build_board_B_oc120_<yyyy-mm-dd_HHMM>.log -tclargs B
```

- **Build numbers** come from a counter both boards share, `tools/gen/board_build_id.txt`. Start a second build at
  least 30 s after the first so they don't read the same number.
- **Outputs** go to `tools/gen/`:
  - `board_<A|B>_b<id>_oc120.bit` (about 7 MB);
  - `_synth.dcp` and `_routed.dcp` checkpoints;
  - timing (`_timing.txt`, `_failing*.txt`), utilisation, IO, clock and congestion reports.
- **Timing:** the log ends with `TIMING WNS=… WHS=…` and `BIT_DONE`. The bitstream is written even when timing fails,
  so read the WNS first.
- **Time and memory:**
  - A board build takes about 50–85 min on a 12-core / 24-thread PC with 24 GB of RAM.
  - Synthesis with 8 threads peaks at about 12 GB. `$env:SYNTH_THREADS='2'` keeps it near 7.6 GB, so it fits beside
    another build's place and route (one 4–6 GB process).
- **Disk:** keep several GB free. Simulation work directories (`tools/gen/simg_*`, `sim_*`, about 0.3 GB each) add up
  and once filled the disk.
- **Optional environment settings:** `SYNTH_ONLY`, `PLACE_DIRECTIVE` (default `AltSpreadLogic_high`),
  `ROUTE_DIRECTIVE`, `PHYS_OPT`, `OPT_DIRECTIVE`, `PHYS_OPT_FANOUT`, `B_PBLOCK=0` (board B's floorplan block off).
- **Bring-up designs:** `tools/build_link.tcl` builds a board-link test design and `tools/build_mig_test.tcl` a DDR3
  calibration test.

## Loading and flashing

- **Load over JTAG** (lost at power-off):
  ```
  vivado.bat -mode batch -nojournal -source bringup/program.tcl -tclargs <serial> tools/gen/board_A_b<id>_oc120.bit
  ```
- **Write the QSPI flash** so the board boots the design by itself:
  1. Rewrite the routed design for SPI x4 at 33 MHz, compressed, plus a 16 MB `.mcs` (1–2 min). The plain build `.bit`
     would take about 25 s to boot.
     ```
     vivado.bat -mode batch -nojournal -nolog -source tools/write_bit_spi.tcl -tclargs tools/gen/board_A_b<id>_oc120_routed.dcp tools/gen/board_A_b<id>_oc120_spi
     ```
  2. Erase, program and verify the flash (about 2 min 40 s):
     ```
     vivado.bat -mode batch -nojournal -nolog -source bringup/flash_qspi.tcl -tclargs <serial> tools/gen/board_A_b<id>_oc120_spi.mcs
     ```
  3. Load the same design again with `program.tcl`: the flash programmer replaces the running design.
- **Return to the flashed design** without a power cycle:
  `vivado.bat -mode batch -nojournal -source bringup/boot_flash.tcl -tclargs <serial>`.
- The two boards must run builds of the same link and stream versions. Pair them as built together.

## Simulation (xsim, from the same Vivado)

- **Board-level and system benches:**
  `powershell -File tools/sim_a.ps1 -Test <name> [-Fast] [-Tag _x] [-Plusargs A,B]`.
  - Names: `mem`, `eth`, `dhcp`, `board_a`, `board_b`, `b3d`, `two`, `two3d`, `img_write`, `img_xfer`, `scan`,
    `video_mmcm`, `disp_post`, `credit`.
  - Example: `-Test two3d -Plusargs SEED_11 -Fast`, the whole chain from the host to the HDMI pixels, about 25 min.
- **Block benches:** `powershell -File tools/sim_gpu.ps1 -Test tb_<name> [-Extra …] [-Plusargs …]`.
  - Seed lists: `tools/run_a_seeds.ps1 -Test tb_a_cmd -Seeds 1,2,3` (also `tb_a_vertex`, `tb_a_geom`, `tb_a_render`).
  - The shader cores over every test program: `tools/run_vs2.ps1 -All`, `tools/run_fs2.ps1 -All`.
  - The compilers' replacements for a core without the integer divider or SIN / COS: `python3 tools/lower_check.py`
    (`--rtl` writes the `tb_vs2` / `tb_fs2` golden sets of `tests/lower_*`).
  - Other runners: `tools/run_tmu24_seeds.ps1`, `tools/run_b_frames.ps1`, `tools/sim_link.ps1` (one link bench at a time).
  - `tools/run_gpu_suite.ps1 -Benches a,b` runs a list of benches.
- **Long-stream stress of board A's geometry:** `tools/a_stress_gen.py` (streams) with `tb_a_stress` (see its header).
- **Goldens,** from WSL: `bash tools/gen_a_goldens.sh` (board A seeds 1–60), `python3 tools/b_ref.py aframe <seed>`
  (two3d) and `python3 tools/b_ref.py frame <seed> <ntri>` (`tb_b_render`, `tb_board_b3d`).
- Every bench prints `TB_PASS` or `TB_FAIL`. The runners also check the compile and elaboration steps.

## Windows libraries

```
powershell -File host/win/build.ps1 [-Arch x64,x86] [-Target ddraw,gl,gles,d3d9,d3d8,tests] [-Clean]
```

- **Outputs** go to `host/win/build/<arch>/`: `ddraw/ddraw.dll`, `gl/opengl32.dll`,
  `gles/libGLESv2.dll` + `libEGL.dll`, `d3d9/d3d9.dll`, `d3d8/d3d8.dll` (x86 only), and the test programs (`gl_test`,
  `gles_test`, `d3d_test`, `d3d8_test`, …).
- **Running a game:** `host\win\run_game.cmd "C:\path\game.exe" [-Info] [-Check] [-Api ddraw|opengl|gles|d3d8|d3d9] [-GameArgs "…"]`.
  - It reads the game's imports to choose the library.
  - It copies the game to `%LOCALAPPDATA%\gpu2\<name>`, places the DLLs beside the copy and logs to
    `<copy>\fpga_gpu.log`.
  - The picture appears on board B's monitor.
  - `-Info` only reports which library the game would use.
- **Per-game launchers:** `run_halflife.cmd` (also `run_halflife.sh` from WSL), `run_nfshp2.cmd`, `run_homm3.cmd`,
  `run_homm4.cmd`.
  - Run from `host\win`, they use the libraries built there.
  - A copy placed beside the game needs `GPU2_REPO` set to the repository's folder.
  - Half-Life and NFS HP2 take the game the launcher sits beside, else `HL_DIR` / `NFS_DIR`, else the default install
    folder. The HoMM launchers use GOG's default folders.
- **Settings:** environment variables, or a `gpu2.ini` of `key=value` lines beside the `.exe` (`host/win/core/g2_env.c`):
  - `GPU2_IP` (board A's address or host name, default 192.168.1.10; the launchers ping it, and the `host/fb` tools
    take it as their default `--ip`), `GPU2_PORT` (5000);
  - `GPU2_LOG`, `GPU2_STATS` (seconds between statistics lines);
  - `GPU2_KEEP_MB`, `GPU2_FRAMES_AHEAD`, `GPU2_DUMP`, `GPU2_RECORD`, `GPU2_TRACE_DRAW`, `GPU2_CLK_MHZ` (120).

## Python host tools

Run them from `host/fb`; Python 3, standard library only (see `host/fb/README.md`).

- `fb_status.py` prints the boards' status. A healthy pair shows `calib 1 link 1 1000M` and `link trained 1 … B alive 1`.
- `fb_a_test.py` is board A's acceptance test.
- `fb_3d_test.py`, `fb_cube.py --check` and `fb_bench.py` run the 3D path (see [PERF_TESTS.md](PERF_TESTS.md)).
- `fb_mode.py`, `fb_image.py`, `fb_video.py` and `fb_2d.py` exercise the display and the 2D path.
- `eth_rx_sweep.py` and `phy_opt.py` tune the Ethernet receive timing.
