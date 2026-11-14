# run_b_frames.ps1 — tb_b_render at the board's options on STORED study frames (tools/gen/<frame>_params.hex, _stream.hex,
# _ddr.hex, _expect.hex, _expect_ds.hex: a frame of tools/b_ref.py kept under its own prefix), with the bench's profiles;
# one compile per tag, every further frame on the same snapshot. Bit-exact compare + the frame's cycles per run.
#   powershell -File tools/run_b_frames.ps1 -Tag _x -Frames hid3,vis3,qmod [-Defines TB_NWARP8,TB_BOARD,PROF] [-Plus NOGAP,WFILL,TMUSTAT] [-NoCompile]
# The full xsim output of each run: tools/gen/b_frames_<frame><tag>.log. Reuse a tag (a run directory is 0.36 GB).
# The stored frames, 256 x 64 unless named by a width:
#   hid3 / vis3   three full layers of b_fs_oa_mod front to back (1 shaded, 2 hidden) / back to front (all shaded)
#   fill3         three layers of b_fs_flat back to front
#   w800 w1024 w1056 w1280   60 random triangles at those target widths (1056 and 1280: 4-line bands)
#   qmod cm (one textured layer), oa smod (sparse), g1 (small triangles)
# A stored frame is made with, e.g. (WSL, repo root; then copy br_{params,stream,ddr,expect,expect_ds}.hex to <name>_*):
#   BR_LAYERS=3 BR_ORDER=front BR_PROG0=b_fs_oa_mod BR_TEX2D=1 BR_WID=256 BR_HGT=64 BR_FMT=0 BR_D16=0 BR_DS=1 python3 tools/b_ref.py frame 7 6
param([string]$Tag = "_fr", [string]$Defines = "TB_NWARP8,TB_BOARD,PROF", [string]$Frames = "hid3,vis3,fill3,qmod,oa,g1",
      [string]$Plus = "NOGAP,WFILL,TMUSTAT", [switch]$NoCompile)
$root = (Resolve-Path "$PSScriptRoot\..").Path
$first = -not $NoCompile
foreach ($f in @($Frames -split ',')) {
  if (-not (Test-Path (Join-Path $root "tools\gen\${f}_stream.hex"))) { "=== $f : no stored frame tools/gen/${f}_*.hex"; continue }
  $ren = "${f}_params.hex:br_params.hex,${f}_stream.hex:br_stream.hex,${f}_ddr.hex:br_ddr.hex,${f}_expect.hex:br_expect.hex,${f}_expect_ds.hex:br_expect_ds.hex"
  $log = Join-Path $root "tools\gen\b_frames_${f}${Tag}.log"
  $a = @('-NoProfile', '-File', "$root\tools\sim_gpu.ps1", '-Test', 'tb_b_render', '-Tag', $Tag, '-Extra', 'src/rtl/mem/mem_arb.sv,sim/mig_app_model.sv',
         '-Defines', $Defines, '-Plusargs', $Plus, '-Rename', $ren, '-All')
  if (-not $first) { $a += '-NoCompile' }
  $first = $false
  & powershell @a *> $log
  "=== $f $Tag"
  Select-String -Path $log -Pattern "COMPILE|ELAB|TB_PASS|TB_FAIL|frame done|perf frame|PROF|WFILL warp life|WFILL warps started|TMUSTAT run|errors =|fatal|ERROR" | ForEach-Object { $_.Line }
}
"RUN_B_FRAMES_DONE $Tag"
