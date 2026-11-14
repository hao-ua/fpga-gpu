# oa_td_stats.ps1 — summary of an oa_timedemo run: the timedemo line, the median "B last frame", mean FS / starved / flush / ROP,
# EVENT resends, and the gpu2.log stall check.
param([Parameter(Mandatory = $true)][string[]]$Tags, [string]$Root = (Join-Path $PSScriptRoot 'gen\oa_timedemo'))
function Median($v) { $s = $v | Sort-Object; $n = $s.Count; if ($n -eq 0) { return 0 }; if ($n % 2) { $s[[int](($n - 1) / 2)] } else { ($s[$n / 2 - 1] + $s[$n / 2]) / 2 } }
foreach ($t in ($Tags -join ',') -split ',') {
    $d = Join-Path $Root $t
    $q = Join-Path $d 'baseoa\qconsole.log'
    $g = Join-Path $d 'gpu2.log'
    $td = if (Test-Path $q) { (Select-String -Path $q -Pattern 'frames.*seconds' | Select-Object -Last 1).Line } else { 'no qconsole.log' }
    $bf = @(); $fs = @(); $st = @(); $fl = @(); $rop = @(); $fps = @(); $res = 0; $lost = 0; $geo = @(); $walk = @()
    foreach ($l in Get-Content $g) {
        if ($l -match 'stats: ([\d.]+) fps .*EVENTs lost (\d+), resent (\d+).*B last frame ([\d.]+) ms: flush (\d+)% band \d+% draw \d+% FS (\d+)% setup \d+% starved (\d+)% ROP (\d+)%.*geometry ([\d.]+) ms, walk ([\d.]+) ms') {
            $fps += [double]$Matches[1]; $lost += [int]$Matches[2]; $res += [int]$Matches[3]; $bf += [double]$Matches[4]
            $fl += [int]$Matches[5]; $fs += [int]$Matches[6]; $st += [int]$Matches[7]; $rop += [int]$Matches[8]; $geo += [double]$Matches[9]; $walk += [double]$Matches[10]
        }
    }
    $n = $bf.Count
    "== $t : $td"
    if ($n -gt 0) {
        "   {0} samples | fps mean {1:N1} median {2:N1} | B last frame median {3:N1} ms (mean {4:N1}) | FS {5:N0}% starved {6:N0}% flush {7:N0}% ROP {8:N0}% | A geometry {9:N1} ms walk {10:N1} ms | EVENTs lost {11} resent {12}" -f `
            $n, ($fps | Measure-Object -Average).Average, (Median $fps), (Median $bf), ($bf | Measure-Object -Average).Average, ($fs | Measure-Object -Average).Average, ($st | Measure-Object -Average).Average, ($fl | Measure-Object -Average).Average, ($rop | Measure-Object -Average).Average, ($geo | Measure-Object -Average).Average, ($walk | Measure-Object -Average).Average, $lost, $res
    }
    $err = Select-String -Path $q -Pattern 'ERROR' -ErrorAction SilentlyContinue | Select-Object -Last 2
    if ($err) { $err | ForEach-Object { "   qconsole: " + $_.Line } }
}
