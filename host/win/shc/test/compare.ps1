# compare.ps1 — the C compiler against the Python one (tools/glslc.py, the specification): every shader, every word, x64 and x86.
#   powershell -File host/win/shc/test/compare.ps1 [-Extra dir,dir] [-Arch x64,x86] [-Unread mask] [-Strict] [-Xops n]
# Extra directories add shaders (.vert / .frag), e.g. the fixed-function programs dumped by the GL library. Unread compiles the vertex shaders as g2sc_compile_vs does.
# Xops: board B's fragment-core operations (g2sc_set_target / glslc xops): 1 SQRT / RCPM / RSQM, 2 CLAMP, 3 both.
param([string[]]$Extra = @(), [string[]]$Arch = @('x64', 'x86'), [int]$Unread = 0, [switch]$Strict, [int]$Xops = 0)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$Arch = @($Arch | ForEach-Object { $_ -split ',' })
$Extra = @($Extra | ForEach-Object { $_ -split ',' })
$files = @(Get-ChildItem "$repo\tests", "$repo\driver\gallium\tools", "$repo\host\win\core", "$PSScriptRoot\cases" -File -Include *.vert, *.frag -Recurse -ErrorAction SilentlyContinue)
foreach ($d in $Extra) { $files += @(Get-ChildItem $d -File -Include *.vert, *.frag -Recurse) }
$rel = @($files | ForEach-Object { $_.FullName })
$out = "$env:TEMP\g2shc_cmp"
New-Item -ItemType Directory -Force $out | Out-Null

function ToWsl($p) { '/mnt/' + $p.Substring(0, 1).ToLower() + ($p.Substring(2) -replace '\\', '/') }

# a list file, not arguments: a command line has a length limit
$list = @($rel | ForEach-Object { ToWsl $_ })
[IO.File]::WriteAllText("$out\files.txt", ($list -join "`n") + "`n")
$wlist = ToWsl "$out\files.txt"
$wrepo = ToWsl $repo
$opt = @(if ($Xops) { '--xops', "$Xops" }) + @(if ($Unread) { '--unread', "$Unread" }) + @(if ($Strict) { '--strict' })
wsl -e bash -lc "cd '$wrepo' && xargs -d '\n' -a '$wlist' python3 -I host/win/shc/test/oracle.py $opt > '$(ToWsl "$out\py.txt")'"
if ($LASTEXITCODE -ne 0) { throw 'the Python oracle failed' }

function Blocks($path) {
    $h = @{}; $k = $null; $b = New-Object Collections.Generic.List[string]
    foreach ($l in [IO.File]::ReadLines($path)) {
        if ($l.StartsWith('FILE ')) {
            $k = $l.Substring(5); $n = 2
            while ($h.ContainsKey($k)) { $k = "$($l.Substring(5))#$n"; $n++ }      # two directories, one file name
            $b = New-Object Collections.Generic.List[string]; continue
        }
        if ($l -eq 'END') { $h[$k] = $b; continue }
        if ($k) { $b.Add($l) }
    }
    $h
}
$py = Blocks "$out\py.txt"
$bad = 0
foreach ($a in $Arch) {
    $exe = "$repo\host\win\build\$a\shc\shc_dump.exe"
    $o = "$out\c_$a.txt"
    Remove-Item $o -ErrorAction SilentlyContinue
    for ($i = 0; $i -lt $rel.Count; $i += 40) {
        $n = [Math]::Min(40, $rel.Count - $i)
        & $exe @opt @($rel[$i..($i + $n - 1)]) | Out-File -Append -Encoding ascii $o
    }
    $c = Blocks $o
    $same = 0; $words = 0; $errs = 0; $difft = 0
    foreach ($k in $py.Keys) {
        $p = $py[$k]; $x = $c[$k]
        if (-not $x) { Write-Output "$a $k : missing from the C output"; $bad++; continue }
        $pe = $p.Count -and $p[0].StartsWith('ERROR')
        $xe = $x.Count -and $x[0].StartsWith('ERROR')
        if ($pe -and $xe) { $errs++; if ($p[0] -ne $x[0]) { $difft++; if (-not $Strict) { Write-Output "$a $k : error text differs`n  py: $($p[0])`n  c : $($x[0])" } }; continue }
        $d = -1
        for ($j = 0; $j -lt [Math]::Max($p.Count, $x.Count); $j++) {
            if ($j -ge $p.Count -or $j -ge $x.Count -or $p[$j] -cne $x[$j]) { $d = $j; break }
        }
        if ($d -ge 0) {
            $bad++
            $pl = if ($d -lt $p.Count) { $p[$d] } else { '(end)' }
            $xl = if ($d -lt $x.Count) { $x[$d] } else { '(end)' }
            Write-Output "$a $k : line $d of $($p.Count) differs`n  py: $pl`n  c : $xl"
        } else { $same++; $words += [int]($p[0] -replace 'WORDS ', '') }
    }
    Write-Output "$a : $same of $($py.Count) identical, $words words; $errs rejected by both ($difft with another text)"
}
Write-Output "mismatches: $bad"
if ($bad) { exit 1 }
