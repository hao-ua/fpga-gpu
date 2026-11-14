# run_vs2_progs.ps1 — tb_vs2 over several programs: for each the golden set is regenerated (tools/isa64.py vs2 <sasm> in
# WSL, into tools/gen; the .sasm is tests/<name>.sasm, or tools/gen/<name>.sasm for a compiled shader's _g64 program),
# then the bench runs it with WARPS_4 (one compile, the rest -NoCompile, in its own work directory -Tag).
#   powershell -File tools/run_vs2_progs.ps1 -Progs vs2_ldpx,vs_cube,test_vsint [-Extra TOL_8,OUTONLY] [-Tag _x] [-NoCompile]
# Prints the bench's summary line and the verdict per program; exit code 1 on any failure.
param([string[]]$Progs = @("vs2_ldpx", "vs_cube", "vs_mv", "vs_pass", "test_vsint", "test_vsuidiv", "test_vsround", "test_wmask"),
      [string]$Extra = "", [string]$Tag = "_progs", [switch]$NoCompile)
$root = (Resolve-Path "$PSScriptRoot\..").Path
Set-Location $root
$wroot = "$(& wsl -e wslpath -a $root)".Trim()                 # the repository as WSL sees it
$Progs = @($Progs | ForEach-Object { $_ -split ',' } | Where-Object { $_ -ne '' })
$first = -not $NoCompile; $fail = 0
foreach ($p in $Progs) {
  $sasm = if (Test-Path "tests\$p.sasm") { "tests/$p.sasm" } else { "tools/gen/$p.sasm" }
  $gen = & wsl -e bash -lc "cd '$wroot' && python3 tools/isa64.py vs2 $sasm --exact-rcp 2>&1 | tail -1"
  Write-Output "=== $p : $gen"
  $plus = "PROG_$p,WARPS_4" + $(if ($Extra) { ",$Extra" } else { "" })
  $args = @("-Test", "tb_vs2", "-Tag", $Tag, "-Plusargs", $plus, "-All")
  if (-not $first) { $args += "-NoCompile" }
  $lines = @(& powershell -NoProfile -File (Join-Path $root "tools\sim_gpu.ps1") @args 2>&1 | ForEach-Object { "$_" })
  $first = $false
  $lines | Select-String -Pattern "TB_PASS|TB_FAIL|^VS2 |COMPILE_FAIL|ELAB_FAIL|warp .* got|Fatal|fatal|FAIL:" | Select-Object -First 12 | ForEach-Object { $_.Line }
  if (-not ($lines -cmatch 'TB_PASS')) { $fail++ }
  if ($lines -match 'COMPILE_FAIL|ELAB_FAIL') { $lines | ForEach-Object { Write-Output $_ }; break }
}
Write-Output "run_vs2_progs: $($Progs.Count) programs, $fail failed"
exit ($(if ($fail) { 1 } else { 0 }))
