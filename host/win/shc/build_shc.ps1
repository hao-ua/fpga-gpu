# build_shc.ps1 — the GLSL ES compiler in C and its oracle test program, x64 and x86 (host/win/shc/test/compare.ps1 runs them against the Python compiler).
#   powershell -File host/win/shc/build_shc.ps1 [-Arch x64,x86]
param([string[]]$Arch = @('x64', 'x86'))
$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$mingw = 'C:\AMDDesignTools\2026.1\Model_Composer\tps\mingw\8.3.0\win64.o\nt'
if ($env:GPU2_MINGW) { $mingw = $env:GPU2_MINGW }
$env:Path = "$mingw\bin;$env:Path"
$Arch = @($Arch | ForEach-Object { $_ -split ',' })
foreach ($a in $Arch) {
    $b = "$root\..\build\$a\shc"
    New-Item -ItemType Directory -Force $b | Out-Null
    $m = @(if ($a -eq 'x86') { '-m32', '-msse2', '-mfpmath=sse' } else { '-m64' })    # x87 double rounding would change folded constants
    $o = @()
    foreach ($f in (Get-ChildItem $root -Filter *.c -File)) {
        $out = "$b\$($f.BaseName).o"
        if (-not (Test-Path $out) -or (Get-Item $out).LastWriteTime -lt (Get-ChildItem $root -File | Sort-Object LastWriteTime -Descending | Select-Object -First 1).LastWriteTime) {
            & "$mingw\bin\gcc.exe" @m -std=gnu11 -O2 -g -Wall -Wno-unused-function -Wno-unused-variable -Wno-maybe-uninitialized -D__USE_MINGW_ANSI_STDIO=1 "-I$root" -c $f.FullName -o $out
            if ($LASTEXITCODE -ne 0) { throw "compile failed: $($f.Name)" }
        }
        $o += $out
    }
    & "$mingw\bin\gcc.exe" @m -std=gnu11 -O2 -g -Wall -D__USE_MINGW_ANSI_STDIO=1 "-I$root" "$root\test\shc_dump.c" @o -o "$b\shc_dump.exe" -static
    if ($LASTEXITCODE -ne 0) { throw 'link failed' }
    Write-Output "built $a"
}
