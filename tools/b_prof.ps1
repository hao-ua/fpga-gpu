# b_prof.ps1 — where board B's frame goes: its cumulative profile counters (STATUS layout 4: JTAG words 24-47 =
# b_render hwprof; with words 13-21, the render and texture counters) from a tools/jtag_watch.tcl .raw log, summed over the
# samples of a time window (32-bit counters: the deltas are taken modulo 2^32, so the watch interval must stay under 30 s).
#   powershell -File tools/b_prof.ps1 -Raw tools/gen/<log>.raw [-From HH:MM:SS] [-To HH:MM:SS] [-Mhz 120]
# Words: 24 cycles (a frame in progress), 25 flush busy, 26 band change, 27 draw change, 28 FS busy, 29 band set-up, 30 stream
# empty, 31 ROP stall, 32 cycles with an issue, 33 / 34 / 35 sums of warps waiting with a texture lookup outstanding / with no
# head micro-op loaded / waiting otherwise, 36 sum of busy warps, 37 sum of live TEXes, 38 warps launched, 39 pixel-issuer
# cycles, 40 a QUAD waits for the early queue, 41 a quad waits for a warp slot, 42 the texture unit holds work, 43 its filter
# waits for data, 44 texture requests (quads), 45 sum of quads inside the texture unit, 46 texture DDR reads, 47 sum of those
# outstanding.
param([Parameter(Mandatory = $true)][string]$Raw, [string]$From = '00:00:00', [string]$To = '23:59:59', [double]$Mhz = 120)
$lines = @(Get-Content $Raw | Where-Object { $_ -match '^(\S+) B ' -and $Matches[1] -ge $From -and $Matches[1] -le $To })
if ($lines.Count -lt 2) { "only $($lines.Count) B lines in the window"; exit 1 }
function W($l) { $p = $l -split ' '; $w = @(); for ($i = 2; $i -lt $p.Count; $i++) { $w += [Convert]::ToUInt64($p[$i], 16) }; return @{ t = $p[0]; w = $w } }
$a = W $lines[0]
if ($a.w.Count -lt 48 -or $a.w[0] -lt [uint64]2952790020) { "the log's B lines hold $($a.w.Count) words, layout $('{0:X8}' -f $a.w[0]): no profile counters (needs B >= 136)"; exit 1 }
$MOD = [uint64]4294967296
$d = New-Object 'uint64[]' 48
$p = $a
for ($i = 1; $i -lt $lines.Count; $i++) {
    $c = W $lines[$i]
    foreach ($k in (13..21) + (24..47)) { $d[$k] += ($c.w[$k] + $MOD - $p.w[$k]) % $MOD }
    $p = $c
}
$b = $p
function Pct([double]$x, [double]$of) { if ($of -le 0) { return 0.0 }; return 100.0 * $x / $of }
function Div([double]$x, [double]$by) { if ($by -le 0) { return 0.0 }; return $x / $by }
$run = [double]$d[24]; $fs = [double]$d[28]; $frames = [double]$d[17]
$shaded = [double]$d[13]                                      # quads that joined a warp (word 13); word 14 = dropped after quad_setup, 15 = ahead of it
$jobs = [double]$d[18]; $busyw = [double]$d[36]
$fin = $busyw - [double]$d[32] - [double]$d[33] - [double]$d[34] - [double]$d[35]   # busy warps that are none of: issuing, waiting (2), fetching
"B profile {0} -> {1}: build {2}, {3:N0} frames, {4:N0} cycles of frame time ({5:N1} % of the wall time at {6} MHz)" -f `
    $a.t, $b.t, $b.w[1], $frames, $run, (Pct $run (($b.w[2] - $a.w[2]) / 1000.0 * $Mhz * 1e6)), $Mhz
"per frame: {0:N0} cycles = {1:N2} ms; {2:N0} quads through quad set-up, {3:N0} shaded, {4:N0} dropped ahead of it; {5:N0} triangle visits" -f `
    (Div $run $frames), ((Div $run $frames) / ($Mhz * 1000.0)), (Div ($d[13] + $d[14]) $frames), (Div $shaded $frames), (Div $d[15] $frames), (Div $d[16] $frames)
""
"the frame's cycles (the parts overlap):"
"  fragment core busy            {0,5:N1} %" -f (Pct $fs $run)
"  flush busy                    {0,5:N1} %" -f (Pct $d[25] $run)
"  band set-up                   {0,5:N1} %" -f (Pct $d[29] $run)
"  band change / draw change     {0,5:N1} % / {1:N1} %" -f (Pct $d[26] $run), (Pct $d[27] $run)
"  stream empty (A or the link)  {0,5:N1} %" -f (Pct $d[30] $run)
"  back end stalled by the ROP   {0,5:N1} %" -f (Pct $d[31] $run)
""
"the fragment core while busy:"
"  cycles with an issue          {0,5:N1} %" -f (Pct $d[32] $fs)
"  warps in flight               {0,5:N2} on average: {1:N2} issuing, {2:N2} waiting with a texture lookup outstanding, {3:N2} waiting otherwise, {4:N2} without a head micro-op (fetch), {5:N2} finished" -f `
    (Div $busyw $fs), (Div $d[32] $fs), (Div $d[33] $fs), (Div $d[35] $fs), (Div $d[34] $fs), (Div $fin $fs)
"  live TEXes                    {0,5:N2} on average; {1:N2} quads per warp ({2:N0} warps a frame)" -f (Div $d[37] $fs), (Div $shaded $d[38]), (Div $d[38] $frames)
""
"per shaded quad: {0:N2} cycles of frame, {1:N2} of them with the core busy; {2:N2} micro-ops issued; {3:N2} pixel-issuer cycles; {4:N2} texture jobs in {5:N2} requests" -f `
    (Div $run $shaded), (Div $fs $shaded), (Div $d[32] $shaded), (Div $d[39] $shaded), (Div $jobs $shaded), (Div $d[44] $shaded)
""
"the front end:"
"  pixel issuer runs             {0,5:N1} % of the frame" -f (Pct $d[39] $run)
"  a QUAD waits for the early queue {0,5:N1} %   (the path behind it is full)" -f (Pct $d[40] $run)
"  a quad waits for a warp slot  {0,5:N1} %   (the core is full)" -f (Pct $d[41] $run)
""
"the texture unit:"
"  holds work                    {0,5:N1} % of the frame; {1:N3} jobs per cycle while it does" -f (Pct $d[42] $run), (Div $jobs $d[42])
"  the filter waits for data     {0,5:N1} % of the frame ({1:N1} % of the time it holds work)" -f (Pct $d[43] $run), (Pct $d[43] $d[42])
"  the A stage stalled           {0,5:N1} % of the frame ({1:N2} cycles per job)" -f (Pct $d[19] $run), (Div $d[19] $jobs)
"  round trip                    {0,5:N1} cycles per request on average; {1:N2} quads inside it on average" -f (Div $d[45] $d[44]), (Div $d[45] $run)
"  cache                         {0,5:N1} % of the lookups miss ({1:N0} hits, {2:N0} misses a frame)" -f (Pct $d[21] ($d[20] + $d[21])), (Div $d[20] $frames), (Div $d[21] $frames)
"  texture DDR reads             {0,5:N0} a frame, {1:N1} cycles each, {2:N2} outstanding on average" -f (Div $d[46] $frames), (Div $d[47] $d[46]), (Div $d[47] $run)
