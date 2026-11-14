# sim_link.ps1 — compile + run tb_board_link in xsim (Windows). Run from the repo root.
#   powershell -File tools/sim_link.ps1            # main test
#   powershell -File tools/sim_link.ps1 -Guard     # contention-guard test
param([switch]$Guard, [switch]$NoCompile)
$ErrorActionPreference = 'Continue'
$viv = "C:\AMDDesignTools\2026.1\Vivado\bin"
$root = (Resolve-Path "$PSScriptRoot\..").Path
$work = Join-Path $root "tools\gen\sim_link"
New-Item -ItemType Directory -Force $work | Out-Null
Set-Location $work
if (-not $NoCompile) {
    $srcs = @("$root\src\rtl\common\uart_tx.sv") + (Get-ChildItem "$root\src\rtl\link\*.sv" | ForEach-Object { $_.FullName }) + @("$root\sim\link_rx_shifter_sim.sv", "$root\sim\tb_board_link.sv", "$viv\..\data\verilog\src\glbl.v")
    & "$viv\xvlog.bat" -sv -i "$root\src\rtl\link" @srcs | Select-String -Pattern "ERROR|error:" | ForEach-Object { $_.Line }
    & "$viv\xelab.bat" tb_board_link glbl -L unisims_ver -s link_sim -timescale 1ns/1ps | Select-String -Pattern "ERROR|Built" | ForEach-Object { $_.Line }
}
$args = @("link_sim", "-R")
if ($Guard) { $args += @("-testplusarg", "GUARD_TEST") }
& "$viv\xsim.bat" @args | Select-String -Pattern "ERROR|TB_PASS|TB_FAIL|\] |UART:|aux_ok|Failed" | ForEach-Object { $_.Line }
