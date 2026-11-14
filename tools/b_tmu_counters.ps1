# b_tmu_counters.ps1 — B's TMU counters (status words 18-21: jobs, A stalls, cache hits, misses; cumulative since reset) from a
# jtag_watch .raw log: the first and last B line, the deltas, the miss percentage, and the temperature range from the .log.
param([Parameter(Mandatory = $true)][string]$Raw, [int]$Skip = 0, [string]$From = '00:00:00', [string]$To = '23:59:59')
$lines = @(Get-Content $Raw | Where-Object { $_ -match '^(\S+) B ' -and $Matches[1] -ge $From -and $Matches[1] -le $To })
if ($lines.Count -lt 2) { "only $($lines.Count) B lines"; exit 1 }
function W($l) { $p = $l -split ' '; $w = @(); for ($i = 2; $i -lt $p.Count; $i++) { $w += [Convert]::ToUInt64($p[$i], 16) }; return @{ t = $p[0]; w = $w } }
$a = W $lines[$Skip]; $b = W $lines[$lines.Count - 1]
# the counters are 32-bit and wrap within a minute at full rate: sum the per-sample deltas modulo 2^32
$MOD = [uint64]4294967296     # not $M: PowerShell variables are case-insensitive and $m is used below
$dj = [uint64]0; $ds = [uint64]0; $dh = [uint64]0; $dm = [uint64]0; $p = $a
$dq = [uint64]0; $dz = [uint64]0; $dz2 = [uint64]0; $dtr = [uint64]0; $dfr = [uint64]0   # words 13-17: quads, early-depth drops (2), tris, frames
for ($i = $Skip + 1; $i -lt $lines.Count; $i++) {
    $c = W $lines[$i]
    $dj += ($c.w[18] + $MOD - $p.w[18]) % $MOD; $ds += ($c.w[19] + $MOD - $p.w[19]) % $MOD
    $dh += ($c.w[20] + $MOD - $p.w[20]) % $MOD; $dm += ($c.w[21] + $MOD - $p.w[21]) % $MOD
    $dq += ($c.w[13] + $MOD - $p.w[13]) % $MOD; $dz += ($c.w[14] + $MOD - $p.w[14]) % $MOD; $dz2 += ($c.w[15] + $MOD - $p.w[15]) % $MOD
    $dtr += ($c.w[16] + $MOD - $p.w[16]) % $MOD; $dfr += ($c.w[17] + $MOD - $p.w[17]) % $MOD
    $p = $c
}
$dt = ($b.w[2] - $a.w[2]) / 1000.0
$shaded = [double]$dq - [double]$dz
"frames {0:N0}  quads {1:N0}  early-depth drops {2:N0} / {3:N0}  tris {4:N0}  -> per frame: {5:N0} quads, {6:N0} shaded, {7:N0} tris, {8:N2} M jobs = {9:N2} jobs per shaded quad" -f `
    $dfr, $dq, $dz, $dz2, $dtr, ($(if ($dfr) { $dq / $dfr } else { 0 })), ($(if ($dfr) { $shaded / $dfr } else { 0 })), ($(if ($dfr) { $dtr / $dfr } else { 0 })), ($(if ($dfr) { $dj / $dfr / 1e6 } else { 0 })), ($(if ($shaded) { $dj / $shaded } else { 0 }))
"{0} -> {1} ({2:N0} s): jobs {3:N0}  A stalls {4:N0} ({5:N1} % of jobs)  hits {6:N0}  misses {7:N0}  -> miss {8:N2} %  | {9:N2} M jobs/s, {10:N3} jobs per 100 MHz cycle (wall, idle included)" -f `
    $a.t, $b.t, $dt, $dj, $ds, ($(if ($dj) { 100.0 * $ds / $dj } else { 0 })), $dh, $dm, ($(if ($dh + $dm) { 100.0 * $dm / ($dh + $dm) } else { 0 })), ($dj / $dt / 1e6), ($dj / $dt / 1e8)
# per-interval miss % along the run (every 20th sample) to see the spread
$prev = $a; $k = 0
foreach ($l in $lines) {
    $k++; if ($k % 20) { continue }
    $c = W $l
    $h = ($c.w[20] + $MOD - $prev.w[20]) % $MOD; $m = ($c.w[21] + $MOD - $prev.w[21]) % $MOD; $j = ($c.w[18] + $MOD - $prev.w[18]) % $MOD; $s = ($c.w[19] + $MOD - $prev.w[19]) % $MOD
    if ($h + $m -gt 0) { "  {0}: miss {1,5:N1} %  astall/jobs {2,5:N1} %  jobs {3:N0}" -f $c.t, (100.0 * $m / ($h + $m)), ($(if ($j) { 100.0 * $s / $j } else { 0 })), $j }
    $prev = $c
}
$log = $Raw -replace '\.raw$', ''
if (Test-Path $log) {
    $ta = @(); $tb = @()
    foreach ($l in Get-Content $log) { if ($l -match '^(\S+) T:([\d.]+)/[\d.]+,([\d.]+)/' -and $Matches[1] -ge $From -and $Matches[1] -le $To) { $ta += [double]$Matches[2]; $tb += [double]$Matches[3] } }
    if ($ta.Count) { "temperatures: A {0:N1}-{1:N1} C, B {2:N1}-{3:N1} C over {4} samples" -f ($ta | Measure-Object -Minimum).Minimum, ($ta | Measure-Object -Maximum).Maximum, ($tb | Measure-Object -Minimum).Minimum, ($tb | Measure-Object -Maximum).Maximum, $ta.Count }
}
