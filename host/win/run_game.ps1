# run_game.ps1 - any game on the FPGA GPU, from a copy: the install stays as it is. run_game.cmd beside this file is what you start.
#   run_game.cmd "C:\path\game.exe"                      play; the picture is on board B's HDMI monitor, the log is fpga_gpu.log in the copy
#   run_game.cmd "C:\path\game.exe" -Info                only say what the game renders with: starts nothing, copies nothing, needs no boards
#   run_game.cmd "C:\path\game.exe" -Check               everything but starting the game
#   run_game.cmd "C:\path\game.exe" -Root "C:\path"      copy from this folder: for a game that reads ..\data
#   run_game.cmd "C:\path\game.exe" -Api d3d9            for a game that loads its renderer at run time: ddraw, opengl, gles, d3d8 or d3d9
#   run_game.cmd "C:\path\game.exe" -GameArgs "-window"  passed on to the game
#   -Name homm      the copy's folder under %LOCALAPPDATA%\gpu2, default the name of the .exe      -Recopy   copy again, the game has been updated
#   -Dump           every 4th frame as a .bmp in the copy's frames folder      -Record   Direct3D: F12 saves the last 48 frames of draws in the copy's rec folder
#   -NoKeep         every vertex is streamed: tells a fault of buffers kept on board A from any other
# A library is chosen from the import tables of the .exe and of the DLLs beside it that it imports, not from strings in the file: a name in a binary proves nothing (The Sims' Sims.exe holds "ddraw" and does not render with it).
# An empty log after the game exits means it never called our library: it ran on the PC's GPU, whatever it looked like.
param(
    [Parameter(Mandatory = $true, Position = 0)][string]$Exe,
    [string]$Root, [string]$Name, [string]$Api, [string]$GameArgs,
    [switch]$Info, [switch]$Check, [switch]$Recopy, [switch]$Dump, [switch]$Record, [switch]$NoKeep
)
$ErrorActionPreference = 'Stop'
$Board = if ($env:GPU2_IP) { $env:GPU2_IP } else { '192.168.1.10' }   # GPU2_IP: an address or a host name

# what we build, by the DLL a game imports
$Ours = @{
    'ddraw.dll'     = @{ Api = 'ddraw';  Dir = 'ddraw'; Files = @('ddraw.dll') }
    'opengl32.dll'  = @{ Api = 'opengl'; Dir = 'gl';    Files = @('opengl32.dll') }
    'd3d8.dll'      = @{ Api = 'd3d8';   Dir = 'd3d8';  Files = @('d3d8.dll') }
    'd3d9.dll'      = @{ Api = 'd3d9';   Dir = 'd3d9';  Files = @('d3d9.dll') }
    'libglesv2.dll' = @{ Api = 'gles';   Dir = 'gles';  Files = @('libGLESv2.dll', 'libEGL.dll') }
    'libegl.dll'    = @{ Api = 'gles';   Dir = 'gles';  Files = @('libGLESv2.dll', 'libEGL.dll') }
}
$Others = @('d3d10.dll', 'd3d10_1.dll', 'd3d10core.dll', 'd3d11.dll', 'd3d12.dll', 'dxgi.dll', 'vulkan-1.dll')

function Stop-With($text) { Write-Host $text -ForegroundColor Red; exit 1 }

# machine and imported DLLs of a PE file, delay-loaded ones too; $null: not one
function Read-Pe($path) {
    $b = [IO.File]::ReadAllBytes($path)
    if ($b.Length -lt 64 -or $b[0] -ne 0x4D -or $b[1] -ne 0x5A) { return $null }
    $pe = [BitConverter]::ToInt32($b, 0x3C)
    if ($pe -lt 0 -or $pe + 248 -ge $b.Length -or [BitConverter]::ToUInt32($b, $pe) -ne 0x4550) { return $null }
    $machine = [BitConverter]::ToUInt16($b, $pe + 4)
    $nsec = [BitConverter]::ToUInt16($b, $pe + 6)
    $opt = $pe + 24
    $wide = [BitConverter]::ToUInt16($b, $opt) -eq 0x20B
    $image = if ($wide) { [BitConverter]::ToInt64($b, $opt + 24) } else { [int64][BitConverter]::ToUInt32($b, $opt + 28) }
    $dirs = $opt + $(if ($wide) { 112 } else { 96 })
    $sec = $opt + [BitConverter]::ToUInt16($b, $pe + 20)
    $map = @()
    for ($i = 0; $i -lt $nsec; $i++) {
        $s = $sec + 40 * $i
        $map += , @([BitConverter]::ToUInt32($b, $s + 12), [Math]::Max([BitConverter]::ToUInt32($b, $s + 8), [BitConverter]::ToUInt32($b, $s + 16)), [BitConverter]::ToUInt32($b, $s + 20))
    }
    function Off($rva) { foreach ($m in $map) { if ($rva -ge $m[0] -and $rva -lt $m[0] + $m[1]) { return [int64]($rva - $m[0] + $m[2]) } }; return -1 }
    function Str($rva) {
        $o = Off $rva
        if ($o -lt 0) { return $null }
        $e = $o
        while ($e -lt $b.Length -and $b[$e] -ne 0 -and $e - $o -lt 260) { $e++ }
        return [Text.Encoding]::ASCII.GetString($b, $o, $e - $o)
    }
    $names = New-Object System.Collections.ArrayList
    $o = Off ([BitConverter]::ToUInt32($b, $dirs + 8))
    while ($o -ge 0 -and $o + 20 -le $b.Length) {
        $n = [BitConverter]::ToUInt32($b, $o + 12)
        if (-not $n) { break }
        $t = Str $n
        if ($t) { [void]$names.Add($t.ToLower()) }
        $o += 20
    }
    $o = Off ([BitConverter]::ToUInt32($b, $dirs + 104))
    while ($o -ge 0 -and $o + 32 -le $b.Length) {
        $n = [int64][BitConverter]::ToUInt32($b, $o + 4)
        if (-not $n) { break }
        if (-not ([BitConverter]::ToUInt32($b, $o) -band 1)) { $n -= $image }     # the old form holds addresses
        $t = Str $n
        if ($t) { [void]$names.Add($t.ToLower()) }
        $o += 32
    }
    $arch = switch ($machine) { 0x14C { 'x86' } 0x8664 { 'x64' } default { 'other' } }
    return [pscustomobject]@{ Arch = $arch; Imports = @($names | Sort-Object -Unique) }
}

# the .exe and every DLL beside it that it reaches through imports: an engine DLL is often what imports the renderer. name -> who imports it
function Read-Closure($path) {
    $dir = Split-Path $path
    $seen = @{}
    $hits = @{}
    $todo = New-Object System.Collections.Queue
    $todo.Enqueue($path)
    while ($todo.Count) {
        $f = $todo.Dequeue()
        $leaf = (Split-Path $f -Leaf).ToLower()
        if ($seen[$leaf]) { continue }
        $seen[$leaf] = $true
        $pe = Read-Pe $f
        if (-not $pe) { continue }
        foreach ($n in $pe.Imports) {
            if ($Ours.ContainsKey($n) -or $Others -contains $n) { if (-not $hits[$n]) { $hits[$n] = (Split-Path $f -Leaf) } }
            $near = Join-Path $dir $n
            if (-not $Ours.ContainsKey($n) -and (Test-Path -LiteralPath $near)) { $todo.Enqueue($near) }      # one of our names is replaced, what it imports does not count: GOG's ddraw.dll is a wrapper over d3d9
        }
    }
    return $hits
}

# names that appear in the file: where a game loads its renderer itself. Not proof of anything, said so where it is shown
function Find-Names($path) {
    $found = @()
    $text = [Text.Encoding]::ASCII.GetString([IO.File]::ReadAllBytes($path)).ToLower()
    foreach ($n in (@($Ours.Keys) + $Others)) { if ($text.Contains($n)) { $found += $n } }
    return $found
}

if (-not (Test-Path -LiteralPath $Exe -PathType Leaf)) { Stop-With "no such file: $Exe" }
$Exe = (Resolve-Path -LiteralPath $Exe).Path
$exeDir = Split-Path $Exe
$exeName = Split-Path $Exe -Leaf
$pe = Read-Pe $Exe
if (-not $pe) { Stop-With "$exeName is not a Windows program" }
$hits = Read-Closure $Exe
Write-Host "$exeName is $($pe.Arch)"
$use = @{}
foreach ($k in $hits.Keys) {
    if ($Ours.ContainsKey($k)) { Write-Host "  imports $k (through $($hits[$k]))"; $use[$Ours[$k].Dir] = $Ours[$k] }
    else { Write-Host "  imports $k (through $($hits[$k])): not something this GPU has" -ForegroundColor Yellow }
}
if ($Api) {
    $pick = $Ours.Values | Where-Object { $_.Api -eq $Api } | Select-Object -First 1
    if (-not $pick) { Stop-With "-Api takes ddraw, opengl, gles, d3d8 or d3d9" }
    $use = @{ $pick.Dir = $pick }
    Write-Host "  -Api: $Api"
}
$blocked = @($hits.Keys | Where-Object { $Others -contains $_ })
if (-not $use.Count) {
    Write-Host "  no import of anything this GPU runs: copying our DLLs beside it would change nothing" -ForegroundColor Yellow
    $maybe = @(Find-Names $Exe)
    foreach ($f in (Get-ChildItem -LiteralPath $exeDir -Filter *.dll -File -ErrorAction SilentlyContinue)) { if (-not $Ours.ContainsKey($f.Name.ToLower())) { $maybe += Find-Names $f.FullName } }
    $maybe = @($maybe | Sort-Object -Unique)
    if ($maybe.Count) { Write-Host "  names in its files, which prove nothing but -Api can try: $($maybe -join ', ')" }
    foreach ($f in (Get-ChildItem -LiteralPath $exeDir -Filter *.exe -File -ErrorAction SilentlyContinue)) {
        if ($f.FullName -eq $Exe) { continue }
        $h = Read-Closure $f.FullName
        $y = @($h.Keys | Where-Object { $Ours.ContainsKey($_) })
        if ($y.Count) { Write-Host "  $($f.Name) beside it imports $($y -join ', '): try that one" }
    }
}
elseif ($blocked.Count -and -not $Api) { Write-Host "  it also imports a newer API: if that is what it renders with, the log will stay empty" -ForegroundColor Yellow }
if ($Info) { exit 0 }
if (-not $use.Count) { Stop-With "nothing to run it with" }
if ($pe.Arch -eq 'other') { Stop-With "only x86 and x64 games" }

$files = @()
foreach ($u in $use.Values) {
    foreach ($f in $u.Files) {
        $p = "$PSScriptRoot\build\$($pe.Arch)\$($u.Dir)\$f"
        if ($u.Api -eq 'd3d8' -and $pe.Arch -ne 'x86') { Stop-With "Direct3D 8 is 32-bit" }
        if (-not (Test-Path -LiteralPath $p)) { Stop-With "no $f at $p`: build it with host\win\build.ps1 -Arch $($pe.Arch) -Target $($u.Dir)" }
        $files += $p
    }
}

$known = @('NFSHP2', 'Warlords_og', 'heroes4', 'openarena')
$copies = Join-Path $env:LOCALAPPDATA 'gpu2'
$busy = @(Get-Process -ErrorAction SilentlyContinue | Where-Object { $known -contains $_.ProcessName -or ($_.Path -and $_.Path.StartsWith($copies, [StringComparison]::OrdinalIgnoreCase)) })
if ($busy.Count) { Stop-With "$($busy[0].ProcessName) is running and only one program can use the boards: close it first" }
if (-not ((ping -n 1 -w 1000 $Board) -match 'TTL=')) { Stop-With "board A does not answer at $Board`: are the boards on?" }

if (-not $Root) { $Root = $exeDir }
$Root = (Resolve-Path -LiteralPath $Root).Path.TrimEnd('\')
if (-not $exeDir.StartsWith($Root, [StringComparison]::OrdinalIgnoreCase)) { Stop-With "-Root must be a folder above the game" }
if (-not $Name) { $Name = [IO.Path]::GetFileNameWithoutExtension($exeName).ToLower() }
$copy = Join-Path $copies $Name
$runDir = $copy + $exeDir.Substring($Root.Length)
$mark = Join-Path $copy 'copied.ok'
if ($Recopy -or -not (Test-Path -LiteralPath $mark)) {
    $bytes = (Get-ChildItem -LiteralPath $Root -Recurse -File -ErrorAction SilentlyContinue | Measure-Object Length -Sum).Sum
    $free = (Get-PSDrive ($copies.Substring(0, 1))).Free
    if ($bytes + 1GB -gt $free) { Stop-With ("the copy needs {0:N1} GB and the disk has {1:N1} GB free" -f ($bytes / 1GB), ($free / 1GB)) }
    Write-Host ("copying the game to {0}, {1:N2} GB" -f $copy, ($bytes / 1GB))
    # no copy of a wrapper the game shipped under one of our names: GOG's ddraw.dll is one
    & robocopy $Root $copy /E /XF ddraw.dll d3d8.dll d3d9.dll opengl32.dll libEGL.dll libGLESv2.dll /NFL /NDL /NJH /NJS /NP | Out-Null
    if ($LASTEXITCODE -ge 8) { Stop-With "the copy failed, robocopy says $LASTEXITCODE" }
    Set-Content -LiteralPath $mark -Value 'copied' -Encoding ascii
}
# Steam starts a game again from its install unless the folder says which game this is
if (Test-Path -Path (Join-Path $exeDir 'steam_api*.dll')) {
    if ($Exe -match '^(.*\\steamapps)\\common\\([^\\]+)\\') {
        $apps = $Matches[1]; $install = $Matches[2]
        foreach ($m in (Get-ChildItem -LiteralPath $apps -Filter 'appmanifest_*.acf')) {
            if ((Get-Content -LiteralPath $m.FullName -Raw) -match ('"installdir"\s+"' + [regex]::Escape($install) + '"')) { Set-Content -LiteralPath (Join-Path $runDir 'steam_appid.txt') -Value ($m.BaseName -replace 'appmanifest_', '') -Encoding ascii }
        }
    }
}

# read-only: a game may delete a DLL of these names at its start. Half-Life's hl.exe _unlinks opengl32.dll in its folder (the old 3dfx MiniGL),
# and its hw.dll then loads Windows' own: the game would run on the PC's GPU with an empty log. Remove-Item -Force still takes it away
$placed = @()
foreach ($p in $files) {
    $to = Join-Path $runDir (Split-Path $p -Leaf)
    Copy-Item -LiteralPath $p -Destination $to -Force
    Set-ItemProperty -LiteralPath $to -Name IsReadOnly -Value $true
    $placed += $to
}
function Remove-Placed { foreach ($p in $placed) { Remove-Item -LiteralPath $p -Force -ErrorAction SilentlyContinue } }
if ($Check) {
    Remove-Placed
    Write-Host "ready: the boards answer, and the copy of the game and our DLLs are in place" -ForegroundColor Green
    exit 0
}

$log = Join-Path $runDir 'fpga_gpu.log'
Remove-Item -LiteralPath $log -Force -ErrorAction SilentlyContinue
$env:GPU2_LOG = $log
$env:GPU2_STATS = '3'
$env:GPU2_TRACE_D3D = '1'
if ($NoKeep) { $env:GPU2_KEEP_MB = '0' }
if ($Dump) {
    $f = Join-Path $copy 'frames'
    New-Item -ItemType Directory -Force $f | Out-Null
    Remove-Item -Path "$f\*.bmp" -Force -ErrorAction SilentlyContinue
    $env:GPU2_DUMP = $f; $env:GPU2_DUMP_EVERY = '4'
}
if ($Record) {
    $f = Join-Path $copy 'rec'
    New-Item -ItemType Directory -Force $f | Out-Null
    $env:GPU2_RECORD = $f
}
Write-Host "$exeName on the FPGA GPU: look at the HDMI monitor. Keep this window open, it cleans up when the game exits."
$a = @{ FilePath = (Join-Path $runDir $exeName); WorkingDirectory = $runDir; Wait = $true }
if ($GameArgs) { $a.ArgumentList = $GameArgs }
try { Start-Process @a } finally { Remove-Placed }
if (-not (Test-Path -LiteralPath $log) -or -not (Get-Item -LiteralPath $log).Length) { Stop-With "our library logged nothing: the game never called it, so it did not run on the FPGA GPU" }
if (Select-String -LiteralPath $log -Pattern 'failed|unusable|exception' -Quiet) { Stop-With "the GPU library reported a problem, see $log" }
Write-Host "the log is $log"
