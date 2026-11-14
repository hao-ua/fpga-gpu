# oa_timedemo.ps1 — the OpenArena timedemo on the two boards: a real game number, and a long run that
# shows hangs. Stops by itself when the demo ends, when the library's log stalls, or at the limit.
#   powershell -File tools/oa_timedemo.ps1 -Dll <opengl32.dll> [-Tag clean] [-Mode 3] [-Limit 900]
# -Dll: x86 opengl32.dll; build a clean one from an export of a commit, host/win's working tree may hold unfinished
#       changes (git archive <commit> host/win tools, then build.ps1 -Arch x86 -Target gl in the export).
# -Mode: r_mode, 3 = 640 x 480, 4 = 800 x 600.
# Only demo088-test1 plays on this OpenArena 0.8.8: the .dm_70 demos stop at "Client/Server game mismatch", a dialog that waits.
# fs_homepath keeps the player's q3config.cfg out of it (+set values would otherwise be saved there). run_on_fpga.cmd is not
# used because it copies host/win's working-tree DLL. Results in tools/gen/oa_timedemo/<tag>: gpu2.log (the library's stats
# every 3 s), baseoa/qconsole.log (the timedemo line). Never run this while someone plays or another client uses the
# boards: board A answers whoever spoke last.
#   -Extra: further +set cvars appended to the command line, e.g. '+set r_colorbits 16 +set r_texturebits 16 +set r_depthbits 16'
#          (the 16-bit tier of the 1998 cards' benchmarks) or '+set r_picmip 0 +set r_texturemode GL_LINEAR_MIPMAP_LINEAR' (high quality).
param([Parameter(Mandatory = $true)][string]$Dll, [string]$Tag = 'clean', [int]$Mode = 3, [int]$Limit = 900, [string]$Extra = '')
$root = (Resolve-Path "$PSScriptRoot\..").Path
$game = 'C:\openarena'
if (-not (Test-Path $Dll)) { Write-Output "no DLL at $Dll"; exit 1 }
if (Get-Process openarena -ErrorAction SilentlyContinue) { Write-Output 'OpenArena is running: the user may be playing'; exit 1 }
$s = Join-Path $root "tools\gen\oa_timedemo\$Tag"
if ($s -match ' ') { Write-Output "fs_homepath must have no spaces: $s"; exit 1 }
New-Item -ItemType Directory -Force $s | Out-Null
$log = "$s\gpu2.log"
Remove-Item $log, "$s\baseoa\qconsole.log" -ErrorAction SilentlyContinue
Remove-Item "$env:TEMP\ioq3+oa.pid" -ErrorAction SilentlyContinue
Copy-Item $Dll "$game\opengl32.dll" -Force
$env:GPU2_LOG = $log; $env:GPU2_STATS = '3'
$t0 = Get-Date
$p = Start-Process -FilePath "$game\openarena.exe" -WorkingDirectory $game -PassThru -ArgumentList `
    "+set r_fullscreen 0 +set r_mode $Mode +set in_mouse 1 +set cg_drawFPS 1 +set fs_homepath $s +set s_initsound 0 +set logfile 2 $Extra +set nextdemo quit +timedemo 1 +demo demo088-test1"
$why = 'exited'; $lastLen = -1; $lastChange = Get-Date
try {
    while (-not $p.HasExited) {
        Start-Sleep -Seconds 5
        $len = 0
        if (Test-Path $log) { $len = (Get-Item $log).Length }
        if ($len -ne $lastLen) { $lastLen = $len; $lastChange = Get-Date }
        if (((Get-Date) - $t0).TotalSeconds -gt $Limit) { $why = "the $Limit s limit"; break }
        if (($len -gt 0) -and (((Get-Date) - $lastChange).TotalSeconds -gt 60)) { $why = 'the library log stopped for 60 s'; break }
    }
    if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force -Confirm:$false }
    Start-Sleep -Seconds 2
} finally {
    Remove-Item "$game\opengl32.dll" -ErrorAction SilentlyContinue    # left behind, a normal start of the game would use it
}
Write-Output "ended by: $why after $([int]((Get-Date) - $t0).TotalSeconds) s"
Select-String -Path "$s\baseoa\qconsole.log" -Pattern 'frames.*seconds|ERROR' | Select-Object -Last 3 | ForEach-Object { $_.Line }
Select-String -Path $log -Pattern 'no EVENT' | Select-Object -Last 4 | ForEach-Object { $_.Line.Substring(0, [Math]::Min(120, $_.Line.Length)) }
