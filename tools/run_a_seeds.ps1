# run_a_seeds.ps1 — one board-A bench over a list of seeds: compile + elaborate once (tools/sim_gpu.ps1), then run every
# seed with -NoCompile in the same work directory, one verdict line per seed.
#   powershell -File tools/run_a_seeds.ps1 -Test tb_a_vertex -Seeds 1,2,3 [-Plusargs STALL] [-Tag _x] [-Out file]
# The bench's extra files (mem_arb, the dispatcher, the MIG model) are chosen by the bench name. Benches of several seeds
# run in parallel when started with different -Tag values.
param([Parameter(Mandatory = $true)][string]$Test, [Parameter(Mandatory = $true)][string[]]$Seeds, [string]$Plusargs = "",
      [string]$Tag = "", [string]$Out = "")
$root = (Resolve-Path "$PSScriptRoot\..").Path
$Seeds = @($Seeds | ForEach-Object { $_ -split ',' } | Where-Object { $_ -ne '' })
$extra = switch ($Test) {
  "tb_a_vertex" { "sim/mig_app_model.sv" }
  "tb_a_geom"   { "sim/mig_app_model.sv,src/rtl/mem/mem_arb.sv" }
  default       { "src/rtl/proto/fb_pkg.sv,src/rtl/res/buf_table.sv,src/rtl/proto/fb_dispatch_a.sv,src/rtl/mem/mem_arb.sv,sim/mig_app_model.sv" }
}
if ($Out -ne "" -and (Test-Path $Out)) { Remove-Item $Out }
$first = $true
foreach ($s in $Seeds) {
  $t0 = Get-Date
  $pa = "SEED_$s"; if ($Plusargs -ne "") { $pa = "$pa,$Plusargs" }
  $args = @("-Test", $Test, "-Tag", $Tag, "-Extra", $extra, "-Plusargs", $pa)
  if (-not $first) { $args += "-NoCompile" }
  $lines = @(& powershell -NoProfile -File (Join-Path $root "tools\sim_gpu.ps1") @args 2>&1 | ForEach-Object { "$_" })
  $first = $false
  $res = @($lines | Where-Object { $_ -notmatch 'Data truncated' })
  $v = "CHECK"
  if ($res -match 'COMPILE_FAIL') { $v = "COMPILE_FAIL" } elseif ($res -match 'ELAB_FAIL') { $v = "ELAB_FAIL" }
  elseif (($res -cmatch 'TB_FAIL|fatal|TIMEOUT|mismatch|ERROR') -or ($res -match 'errors\s*=\s*[1-9]')) { $v = "FAIL" }
  elseif ($res -cmatch 'TB_PASS') { $v = "PASS" }
  $line = "{0,-12} seed {1,-4} {2,-12} {3,5:N0} s  {4}" -f $Test, $s, $v, ((Get-Date) - $t0).TotalSeconds, (($res | Where-Object { $_ -match 'seed|FAIL' } | Select-Object -Last 1))
  Write-Output $line
  if ($Out -ne "") { Add-Content -Path $Out -Value $line -Encoding utf8 }
  if ($v -eq "COMPILE_FAIL" -or $v -eq "ELAB_FAIL") { $res | ForEach-Object { Write-Output $_ }; break }
}
