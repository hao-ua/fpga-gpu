# run_gpu_suite.ps1 — run a list of single-board GPU benches one after the other with tools/sim_gpu.ps1 and print
# one verdict line per bench (the full xsim output stays in tools/gen/simg_<bench>/xsim.log).
#   powershell -File tools/run_gpu_suite.ps1 -Benches tb_shader_core,tb_brp [-Out <summary file>]
# Verdict: COMPILE_FAIL / ELAB_FAIL as reported; FAIL if a result line shows FAIL, fatal, TIMEOUT, mismatch, ERROR
# or a nonzero err= / errors= count; PASS if a line shows PASS (or "TB DONE" with error counts all zero);
# otherwise CHECK (read the log). Data-truncation warnings of the LUT files are not results.
param([Parameter(Mandatory = $true)][string[]]$Benches, [string]$Out = "")
$root = (Resolve-Path "$PSScriptRoot\..").Path
$Benches = @($Benches | ForEach-Object { $_ -split ',' } | Where-Object { $_ -ne '' })   # -File passes "a,b" as one string
if ($Out -ne "" -and (Test-Path $Out)) { Remove-Item $Out }
foreach ($b in $Benches) {
  $t0 = Get-Date
  $lines = @(& powershell -NoProfile -File (Join-Path $root "tools\sim_gpu.ps1") -Test $b 2>&1 | ForEach-Object { "$_" })
  $res = @($lines | Where-Object { $_ -notmatch 'Data truncated' })
  $v = "CHECK"
  if ($res -match 'COMPILE_FAIL') { $v = "COMPILE_FAIL" }
  elseif ($res -match 'ELAB_FAIL') { $v = "ELAB_FAIL" }
  elseif (($res -cmatch 'FAIL|fatal|TIMEOUT|mismatch|ERROR') -or ($res -match 'err(ors|s)?\s*=\s*[1-9]')) { $v = "FAIL" }
  elseif (($res -cmatch 'PASS') -or (($res -cmatch 'DONE') -and ($res -match 'err(ors|s)?\s*=\s*0'))) { $v = "PASS" }
  $s = "{0,-28} {1,-12} {2,5:N0} s  {3}" -f $b, $v, ((Get-Date) - $t0).TotalSeconds, (($res | Select-Object -Last 2) -join ' | ')
  Write-Output $s
  if ($Out -ne "") { Add-Content -Path $Out -Value $s -Encoding utf8 }
}
