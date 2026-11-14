# parity.ps1 — byte parity of gpu2core with fbproto.py, no board and no network:
# the same scenes through both, recorded, compared datagram by datagram. Needs WSL's python3 for the Python half.
param([string]$Arch = 'x64')
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path "$PSScriptRoot\..\..\..").Path
$out = "$PSScriptRoot\..\build\$Arch"
$wrepo = (wsl -e wslpath -a ($repo -replace '\\', '/')).Trim()
wsl -e bash -c "cd '$wrepo' && python3 -u host/win/tests/record_py.py host/win/build/$Arch/py.dgram"
if ($LASTEXITCODE -ne 0) { throw 'record_py failed' }
& "$out\g2_parity.exe" "$out\c.dgram"
if ($LASTEXITCODE -ne 0) { throw 'g2_parity failed' }
$a = [IO.File]::ReadAllBytes("$out\py.dgram")
$b = [IO.File]::ReadAllBytes("$out\c.dgram")
$pa = 0; $pb = 0; $n = 0; $bad = 0
while ($pa -lt $a.Length -and $pb -lt $b.Length) {
    $la = [BitConverter]::ToUInt32($a, $pa); $lb = [BitConverter]::ToUInt32($b, $pb)
    $same = $la -eq $lb
    if ($same) { for ($i = 0; $i -lt $la; $i++) { if ($a[$pa + 4 + $i] -ne $b[$pb + 4 + $i]) { $same = $false; $at = $i; break } } } else { $at = -1 }
    if (-not $same) {
        $bad++
        if ($bad -le 10) {
            $t = '{0:X2}' -f $a[$pa + 7]
            $s = [BitConverter]::ToUInt32($a, $pa + 8)
            $ctx = ''
            if ($at -ge 0) { $e = [Math]::Min($at + 16, $la); $ctx = (($a[($pa + 4 + $at)..($pa + 3 + $e)] | ForEach-Object { '{0:X2}' -f $_ }) -join ' ') + ' | ' + (($b[($pb + 4 + $at)..($pb + 3 + $e)] | ForEach-Object { '{0:X2}' -f $_ }) -join ' ') }
            "datagram $n type $t seq $s lengths $la / $lb first difference at byte $at : $ctx"
        }
    }
    $pa += 4 + $la; $pb += 4 + $lb; $n++
}
$restA = $a.Length - $pa; $restB = $b.Length - $pb
"$n datagrams compared, $($a.Length) / $($b.Length) bytes, $bad differ, left over $restA / $restB"
if ($bad -or $restA -or $restB) { exit 1 }
'PARITY OK'
