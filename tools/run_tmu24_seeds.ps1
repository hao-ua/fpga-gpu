# run_tmu24_seeds.ps1 — tb_tmu24 over several seeds: for each seed the goldens are regenerated (tools/tmu_ref.py fp24 <seed>
# in WSL, into tools/gen), then the bench runs (one compile, the rest -NoCompile, in its own work directory -Tag).
#   powershell -File tools/run_tmu24_seeds.ps1 [-Seeds 1,2,3,4] [-Tag _x] [-Vol3d]
# -Vol3d: the 3D texture path (tmu24 VOL3D = 1): goldens with volumes (tmu_ref.py --vol3d) and the bench
# compiled with TB_VOL3D. Without it the DUT is the board's (VOL3D 0) and the descriptors hold no volumes.
# Prints the bench's throughput / thrash lines and the verdict per seed; exit code 1 on any failure.
param([string[]]$Seeds = @("1", "2", "3", "4"), [string]$Tag = "_seeds", [switch]$Vol3d)
$root = (Resolve-Path "$PSScriptRoot\..").Path
Set-Location $root
$wroot = "$(& wsl -e wslpath -a $root)".Trim()                 # the repository as WSL sees it
$Seeds = @($Seeds | ForEach-Object { $_ -split ',' } | Where-Object { $_ -ne '' })
$first = $true; $fail = 0
$vflag = if ($Vol3d) { " --vol3d" } else { "" }
foreach ($s in $Seeds) {
  $gen = & wsl -e bash -lc "cd '$wroot' && python3 -u tools/tmu_ref.py fp24 $s$vflag 2>&1 | tail -2 | head -1"
  Write-Output "=== seed $s : $gen"
  $args = @("-Test", "tb_tmu24", "-Tag", $Tag, "-All")
  if ($Vol3d) { $args += @("-Defines", "TB_VOL3D") }
  if (-not $first) { $args += "-NoCompile" }
  $lines = @(& powershell -NoProfile -File (Join-Path $root "tools\sim_gpu.ps1") @args 2>&1 | ForEach-Object { "$_" })
  $first = $false
  $lines | Select-String -Pattern "TB_PASS|TB_FAIL|TMU24|thrash|THRASH|isolated|trilinear|COMPILE_FAIL|ELAB_FAIL|quad .* got|Fatal|fatal|finish called" | ForEach-Object { $_.Line }
  if (-not ($lines -cmatch 'TB_PASS')) { $fail++ }
  if ($lines -match 'COMPILE_FAIL|ELAB_FAIL') { $lines | ForEach-Object { Write-Output $_ }; break }
}
Write-Output "run_tmu24_seeds: $($Seeds.Count) seeds, $fail failed"
exit ($(if ($fail) { 1 } else { 0 }))
