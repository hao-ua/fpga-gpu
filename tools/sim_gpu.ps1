# sim_gpu.ps1 — compile + elaborate + run one bench of the GPU blocks in xsim (Windows): every src/rtl/*.sv (top
# directory only) plus sim/<bench>.sv (the board-level benches run through tools/sim_a.ps1).
#   powershell -File tools/sim_gpu.ps1 -Test tb_fs2 [-Tag _x] [-Plusargs FOO] [-NoCompile] [-All]
# The RTL and the benches load tables, programs and golden vectors by paths relative to the run directory
# (tools/gen/...), so the run directory tools/gen/simg_<bench><tag> gets a copy of tools/gen's top-level .mem / .hex
# files (regenerate them with tools/gen_gpu_artifacts.sh). Checks compile and elaboration errors (exit code and
# ERROR lines) and prints the bench's result lines; -All prints the whole xsim output.
#   -Defines SCENE,FOO: xvlog -d for each. Verilog files under src/rtl/eth/*.v, if any, are compiled as well.
#   -Extra src/rtl/mem/mem_arb.sv,sim/mig_app_model.sv: further files (repository-relative), compiled before the bench.
#   -Rename ab_13_params.hex:br_params.hex,as_13_stream.hex:br_stream.hex,...: after the copy, tools/gen/<src> is placed
#          in the run directory under <dst> — a bench reads another generator's files under the names it expects (e.g.
#          tb_b_render on board A's frame of b_ref.py aframe <seed>: tools/aframe_on_b.ps1).
#   -Mutant C:\...\some_dir\vtx_feed.sv: this file (ABSOLUTE path, the SAME file name as the src/rtl file it replaces,
#          in another directory) is compiled instead of src/rtl/<name> (a bench's sensitivity check: an older /
#          deliberately broken copy must make it fail).
param([Parameter(Mandatory = $true)][string]$Test, [string]$Tag = "", [string]$Plusargs = "", [switch]$NoCompile,
      [switch]$All, [string]$Defines = "", [string]$Extra = "", [string]$Rename = "", [string]$Mutant = "")
$ErrorActionPreference = 'Continue'
$viv  = "C:\AMDDesignTools\2026.1\Vivado\bin"
$root = (Resolve-Path "$PSScriptRoot\..").Path
$work = Join-Path $root "tools\gen\simg_$Test$Tag"
$wgen = Join-Path $work "tools\gen"
New-Item -ItemType Directory -Force $wgen | Out-Null
Copy-Item (Join-Path $root "tools\gen\*") -Include *.mem, *.hex -Destination $wgen -Force
foreach ($pair in @($Rename -split ',' | Where-Object { $_ -ne '' })) {
  $sd = $pair -split ':'
  Copy-Item (Join-Path $root "tools\gen\$($sd[0])") -Destination (Join-Path $wgen $sd[1]) -Force
}
Set-Location $work

$rtl = @(Get-ChildItem (Join-Path $root "src\rtl\*.sv") | Sort-Object Name | ForEach-Object { $_.FullName })
if ($Mutant -ne "") {
  $leaf = Split-Path $Mutant -Leaf
  $rtl = @($rtl | ForEach-Object { if ((Split-Path $_ -Leaf) -eq $leaf) { $Mutant } else { $_ } })
}
$tbf = Join-Path $root "sim\$Test.sv"
$ext = @($Extra -split ',' | Where-Object { $_ -ne '' } | ForEach-Object { Join-Path $root $_ })
if (-not (Test-Path $tbf)) { Write-Output "no bench $tbf"; exit 1 }
$eth = @(Get-ChildItem (Join-Path $root "src\rtl\eth\*.v") -ErrorAction SilentlyContinue | Sort-Object Name | ForEach-Object { $_.FullName })
$defs = @($Defines -split ',' | Where-Object { $_ -ne '' } | ForEach-Object { "-d"; $_ })
if (-not $NoCompile) {
  $out = @(); $rc = 0
  if ($eth.Count -gt 0) { $out = @(& "$viv\xvlog.bat" @defs @eth 2>&1); $rc = $LASTEXITCODE }
  $out += @(& "$viv\xvlog.bat" -sv @defs --include (Join-Path $root "sim") @rtl @ext $tbf 2>&1)
  if ($LASTEXITCODE -ne 0) { $rc = $LASTEXITCODE }
  $err = $out | Select-String -Pattern "ERROR|error:"
  if ($err -or $rc -ne 0) { $err | ForEach-Object { $_.Line }; Write-Output "COMPILE_FAIL (exit $rc)"; exit 1 }
  $out = & "$viv\xelab.bat" $Test -s "snap_$Test" -timescale 1ns/1ps -debug off 2>&1
  $rc = $LASTEXITCODE
  $err = $out | Select-String -Pattern "ERROR|error:"
  if ($err -or $rc -ne 0) { $err | ForEach-Object { $_.Line }; Write-Output "ELAB_FAIL (exit $rc)"; exit 1 }
  Write-Output "COMPILE_ELAB_OK $Test"
}
$xargs = @("snap_$Test", "-R")
foreach ($p in @($Plusargs -split ',' | Where-Object { $_ -ne '' })) { $xargs += @("-testplusarg", $p) }   # no "=" (cmd splits there)
if ($All) { & "$viv\xsim.bat" @xargs 2>&1 | ForEach-Object { "$_" } }
else {
  & "$viv\xsim.bat" @xargs 2>&1 | Select-String -Pattern "PASS|FAIL|matched|covered|TIMEOUT|fatal|ERROR|mismatch|errors|err=|DONE|ok:|STALL|still busy" |
    ForEach-Object { $_.Line }
}
