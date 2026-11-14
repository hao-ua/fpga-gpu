# run_vs2.ps1 — goldens + tb_vs2 for the vertex core on the scalar core's pattern. For each program:
# the reference set from tools/isa64.py vs2 (WSL python3), then sim_gpu.ps1 -Test tb_vs2 (one compile, the rest -NoCompile).
#   powershell -File tools/run_vs2.ps1 -All [-Warps 4]          every tests/*.sasm without fragment-only ops (TEX DDX DDY
#                                                               DISCARD STOZ) or the 32-bit vertex core's BRP — the
#                                                               fragment programs included (their LDV are attributes here)
#   powershell -File tools/run_vs2.ps1 -List vs_cube,test_fs     named programs
# Per program: +TOL_8 when it uses RSQRT / LOG2 / EXP2 / RCP (the reference approximates them; RCP is exact with
# --exact-rcp, applied when RCP is present), --deep for the stack-violation programs (*stk*). -Active HEX runs a partial
# warp (golden --active). Prints one line per program and a summary; exit code 1 on any failure.
param([switch]$All, [string]$List = "", [int]$Warps = 4, [string]$Active = "", [switch]$Verbose)
$ErrorActionPreference = 'Continue'
$root = (Resolve-Path "$PSScriptRoot\..").Path
Set-Location $root
if ($All) {
    $progs = @(Get-ChildItem "$root\tests\*.sasm" | Sort-Object Name | Where-Object {
        -not ((Get-Content $_.FullName) -cmatch '^\s*(TEX|DDX|DDY|DISCARD|STOZ|BRP|SQRT|RCPM|RSQM|DIV|CLAMP)\b') } | ForEach-Object { $_.BaseName })
} else {
    $progs = @($List -split ',' | Where-Object { $_ -ne '' })
}
if ($progs.Count -eq 0) { Write-Output "no programs (use -All or -List a,b)"; exit 1 }
$wroot = "/mnt/c" + ($root.Substring(2) -replace '\\', '/')
$fail = 0; $first = $true
foreach ($p in $progs) {
    $src = Get-Content "$root\tests\$p.sasm"
    $opt = ""; $tol = 0
    if ($src -cmatch '^\s*RCP\b') { $opt += " --exact-rcp" }
    if ($src -cmatch '^\s*(RSQRT|LOG2|EXP2|RCP)\b') { $tol = 8 }
    if ($p -match 'stk') { $opt += " --deep" }
    if ($Active -ne "") { $opt += " --active $Active" }
    $g = wsl -e bash -lc "cd $wroot && python3 tools/isa64.py vs2 tests/$p.sasm$opt" 2>&1
    if ($LASTEXITCODE -ne 0) { Write-Output "GOLDEN_FAIL $p : $g"; $fail++; continue }
    $pa = "PROG_$p,WARPS_$Warps,TOL_$tol"
    if ($Active -ne "") { $pa += ",ACTIVE_$Active" }
    $args = @('-Test', 'tb_vs2', '-Plusargs', $pa)
    if (-not $first) { $args += '-NoCompile' }
    $first = $false
    $out = powershell -ExecutionPolicy Bypass -File "$root\tools\sim_gpu.ps1" @args
    $line = $out | Where-Object { $_ -match '^VS2 ' }
    $res  = $out | Where-Object { $_ -match 'TB_PASS|TB_FAIL|COMPILE_FAIL|ELAB_FAIL|Fatal|fatal' }
    if ($Verbose) { $out | ForEach-Object { "    $_" } }
    if ($res -match 'TB_PASS') { Write-Output "ok   $line" } else { Write-Output "FAIL $p : $res"; $fail++ }
}
Write-Output "run_vs2: $($progs.Count) programs, $fail failed"
exit $(if ($fail -eq 0) { 0 } else { 1 })
