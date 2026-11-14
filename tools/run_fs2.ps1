# run_fs2.ps1 — goldens + tb_fs2 for the scalar fragment core. For each program: the reference
# set from tools/isa64.py (WSL python3), then sim_gpu.ps1 -Test tb_fs2 (one compile, the rest -NoCompile).
#   powershell -File tools/run_fs2.ps1 -All [-Warps 4]          every tests/*.sasm fragment program (TEX included:
#                                                               the bench's texture model = isa64.tex_model)
#   powershell -File tools/run_fs2.ps1 -List fs2_dp,test_fs      named programs
# Per program: +TOL_8 when it uses RSQRT / LOG2 / EXP2 / RCP (the reference approximates them; RCP is exact with
# --exact-rcp, applied when RCP is present), --deep for the stack-violation programs (*stk*). -Active HEX runs a partial
# warp (golden --active). Prints one line per program and a summary; exit code 1 on any failure.
param([switch]$All, [string]$List = "", [int]$Warps = 4, [string]$Active = "", [switch]$Verbose, [int]$Pre = 0, [string]$Tag = "", [string]$Defines = "")
$ErrorActionPreference = 'Continue'
$root = (Resolve-Path "$PSScriptRoot\..").Path
Set-Location $root
if ($All) {
    $progs = @(Get-ChildItem "$root\tests\*.sasm" | Sort-Object Name | ForEach-Object { $_.BaseName } |
        Where-Object { ($_ -match '^(fs_|test_fs|b_fs|fs2_)' -or $_ -match '_fs$') })
} else {
    $progs = @($List -split ',' | Where-Object { $_ -ne '' })
}
if ($progs.Count -eq 0) { Write-Output "no programs (use -All or -List a,b)"; exit 1 }
$wroot = "/mnt/c" + ($root.Substring(2) -replace '\\', '/')
$fail = 0; $first = $true
foreach ($p in $progs) {
    $src = Get-Content "$root\tests\$p.sasm"
    $opt = ""; $tol = 0
    if ($src -match '^\s*(RCP|RCPM|DIV)\b') { $opt += " --exact-rcp" }
    if ($src -match '^\s*(RSQRT|LOG2|EXP2|RCP|SQRT|RCPM|RSQM|DIV)\b') { $tol = 8 }
    if ($p -match 'stk') { $opt += " --deep" }
    if ($Active -ne "") { $opt += " --active $Active" }
    $g = wsl -e bash -lc "cd $wroot && python3 tools/isa64.py fs2 tests/$p.sasm$opt" 2>&1
    if ($LASTEXITCODE -ne 0) { Write-Output "GOLDEN_FAIL $p : $g"; $fail++; continue }
    $pa = "PROG_$p,WARPS_$Warps,TOL_$tol"
    if ($Active -ne "") { $pa += ",ACTIVE_$Active" }
    if ($Pre -ne 0) { $pa += ",PRE_$Pre" }                      # the program fetch $Pre cycles ahead of the launch (1..16)
    $args = @('-Test', 'tb_fs2', '-Plusargs', $pa)
    if ($Tag -ne "") { $args += @('-Tag', $Tag) }
    if ($Defines -ne "") { $args += @('-Defines', $Defines) }   # e.g. TB_LDV2 (pair it with its own -Tag)
    if (-not $first) { $args += '-NoCompile' }
    $first = $false
    $out = powershell -ExecutionPolicy Bypass -File "$root\tools\sim_gpu.ps1" @args
    $line = $out | Where-Object { $_ -match '^FS2 ' }
    $res  = $out | Where-Object { $_ -match 'TB_PASS|TB_FAIL|COMPILE_FAIL|ELAB_FAIL|Fatal|fatal' }
    if ($Verbose) { $out | ForEach-Object { "    $_" } }
    if ($res -match 'TB_PASS') { Write-Output "ok   $line" } else { Write-Output "FAIL $p : $res"; $fail++ }
}
Write-Output "run_fs2: $($progs.Count) programs, $fail failed"
exit $(if ($fail -eq 0) { 0 } else { 1 })
