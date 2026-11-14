# aframe_on_b.ps1 — render board A's frame <seed> (tools/gen/ab_<seed>_* of `python3 tools/b_ref.py aframe <seed>`, the
# stream as_<seed>_stream.hex of a_raster_ref.py) on board B's render engine alone: tb_b_render with the files placed under
# the br_* names it reads. Localizes a tb_two_boards3d mismatch to B's engine (fails here too) or to the boards' link /
# A's streaming (passes here). Run from the repository root:
#   powershell -File tools/aframe_on_b.ps1 -Seed 13 [-NoCompile] [-All]
param([Parameter(Mandatory = $true)][int]$Seed, [switch]$NoCompile, [switch]$All)
$ren = "ab_${Seed}_params.hex:br_params.hex,ab_${Seed}_ddr.hex:br_ddr.hex,as_${Seed}_stream.hex:br_stream.hex," +
       "ab_${Seed}_expect.hex:br_expect.hex,ab_${Seed}_expect_ds.hex:br_expect_ds.hex"
$args_ = @("-Test", "tb_b_render", "-Tag", "_af$Seed", "-Extra", "src/rtl/mem/mem_arb.sv,sim/mig_app_model.sv", "-Rename", $ren)
if ($NoCompile) { $args_ += "-NoCompile" }
if ($All) { $args_ += "-All" }
& powershell -File (Join-Path $PSScriptRoot "sim_gpu.ps1") @args_
