# sim_a.ps1 — compile + elaborate + run the board-A benches in xsim (Windows). Run from the repo root.
#   powershell -File tools/sim_a.ps1 -Test mem        # tb_mem   (arbiter, DDR FIFO, transfer engine)
#   powershell -File tools/sim_a.ps1 -Test eth        # tb_eth   (RGMII 100/1000, MAC, ARP/ICMP/UDP, MDIO)
#   powershell -File tools/sim_a.ps1 -Test dhcp       # tb_dhcp  (DHCP client: lease, renewal, fallback address, conflicts)
#   powershell -File tools/sim_a.ps1 -Test board_a    # tb_board_a (whole board A core, host protocol)
#   powershell -File tools/sim_a.ps1 -Test video_mmcm # tb_video_mmcm (board B video-mode DRP ROM + MMCM reconfiguration)
#   powershell -File tools/sim_a.ps1 -Test scan       # tb_scan_reader (board B scanout reader, random images)
#   powershell -File tools/sim_a.ps1 -Test board_b    # tb_board_b (whole board B core, TMDS decoded back to frames)
#   powershell -File tools/sim_a.ps1 -Test two        # tb_two_boards (host -> A -> board link -> B -> HDMI: mode switch, present, fence)
#   powershell -File tools/sim_a.ps1 -Test img_write  # tb_img_write (board B WRITE_IMAGE engine: formats, rectangles, masks)
#   powershell -File tools/sim_a.ps1 -Test credit     # tb_link_credit (board link credit across board restarts)
#   powershell -File tools/sim_a.ps1 -Test b3d        # tb_board_b3d (board B's 3D path from a bench-side A link)
#   powershell -File tools/sim_a.ps1 -Test two3d -Plusargs SEED_11   # tb_two_boards3d (host -> A geometry + render -> link -> B)
# Checks compile and elaboration errors and prints the bench's TB_PASS / TB_FAIL line.
#   -Fast: word-level HDMI (TMDS_WORD_SIM: serializer stubs, tmds_capture decodes the encoder words at the pixel
#          clock) for board_b / two, in its own work dir (suffix _w). -Plusargs FULL implies it: the full-mode runs
#          skip the OSERDES / bit-level decoding, which the default (short) runs keep covering. -BitLevel forces
#          the serializers into a FULL run as well.
#   -Mutant <file>: compile <file> in place of the RTL file with the same name (mutation check of a bench: a
#          deliberately broken copy must make it fail); use with its own -Tag.
param([Parameter(Mandatory = $true)][string]$Test, [switch]$NoCompile, [string]$Plusargs = "", [string]$Tag = "",
      [switch]$Fast, [string]$Mutant = "", [switch]$BitLevel)
$ErrorActionPreference = 'Continue'
$viv  = "C:\AMDDesignTools\2026.1\Vivado\bin"
$root = (Resolve-Path "$PSScriptRoot\..").Path
if ($Plusargs -match 'FULL' -and -not $BitLevel -and ($Test -eq 'board_b' -or $Test -eq 'two')) { $Fast = $true }
if ($Fast) { $Tag = "_w$Tag" }
$work = Join-Path $root "tools\gen\sim_$Test$Tag"     # -Tag: separate work dir (run beside a running sim)
New-Item -ItemType Directory -Force $work | Out-Null
# the 3D blocks read their FP24 tables from tools/gen (relative to the run directory)
New-Item -ItemType Directory -Force (Join-Path $work "tools\gen") | Out-Null
Copy-Item (Join-Path $root "tools\gen\fp24_*.mem") -Destination (Join-Path $work "tools\gen") -Force
Copy-Item (Join-Path $root "tools\gen\*") -Include br_*.hex, bl_*.hex -Destination (Join-Path $work "tools\gen") -Force   # tb_board_b3d
Copy-Item (Join-Path $root "tools\gen\*") -Include av_*.hex, av_*.mem, as_*.hex, as_*.mem, ab_*.hex -Destination (Join-Path $work "tools\gen") -Force   # tb_board_a's 3D draw, tb_two_boards3d
Set-Location $work

$rtl = @(
  "src\rtl\net\net_pkg.sv", "src\rtl\proto\fb_pkg.sv",
  "src\rtl\net\rst_sync.sv", "src\rtl\net\afifo_bram.sv", "src\rtl\mem\sfifo.sv",
  "src\rtl\net\eth_rgmii_rx.sv", "src\rtl\net\eth_mac_rx.sv", "src\rtl\net\eth_net_rx.sv",
  "src\rtl\net\eth_tx.sv", "src\rtl\net\mdio_mgr.sv", "src\rtl\net\dhcp_client.sv", "src\rtl\net\eth_stack.sv",
  "src\rtl\rgmii_tx.sv", "src\rtl\rgmii_tx_adapt.sv",
  "src\rtl\mem\mem_arb.sv", "src\rtl\mem\ddr_fifo.sv", "src\rtl\res\xfer_lin.sv",
  "src\rtl\res\buf_table.sv", "src\rtl\proto\fb_reply.sv", "src\rtl\proto\fb_proto_a.sv",
  "src\rtl\proto\fb_dispatch_a.sv",
  # board A 3D: the vertex core and its front end, setup / records, bins
  "src\rtl\shader_core_vs2.sv", "src\rtl\attr_conv.sv", "src\rtl\vtx_feed.sv", "src\rtl\vtx_rbuf.sv", "src\rtl\vtx_asm.sv", "src\rtl\a_vertex.sv",
  "src\rtl\clip_near.sv", "src\rtl\float_addsub_w.sv", "src\rtl\float_mul_w.sv", "src\rtl\vtx_setup2.sv", "src\rtl\zq_tri.sv", "src\rtl\zq_lane.sv", "src\rtl\a_setup.sv", "src\rtl\tri_binner2.sv", "src\rtl\a_render.sv", "src\rtl\a_geom.sv",
  "src\rtl\top\board_a_core.sv", "src\rtl\link\link_status_jtag.sv",
  # board B
  "src\rtl\res\img_pkg.sv", "src\rtl\res\img_table.sv", "src\rtl\res\img_write.sv", "src\rtl\res\img_xfer.sv", "src\rtl\res\img_geom.sv", "src\rtl\common\udiv.sv", "src\rtl\common\jtag_ctrl_reg.sv",
  "src\rtl\video_timing.sv", "src\rtl\tmds_encoder.sv", "src\rtl\tmds_serializer.sv",
  "src\rtl\display_line_buffer.sv", "src\rtl\display_out.sv",
  "src\rtl\disp\video_mmcm.sv", "src\rtl\disp\scan_reader.sv", "src\rtl\disp\disp_post.sv", "src\rtl\disp\disp_out_b.sv",
  "src\rtl\disp\disp_ctl_b.sv", "src\rtl\disp\disp_b.sv",
  "src\rtl\top\b_bringup.sv", "src\rtl\top\board_b_core.sv",
  # board link
  "src\rtl\proto\link_a_port.sv", "src\rtl\proto\link_b_port.sv", "src\rtl\proto\fb_cmd_b.sv",
  "src\rtl\common\uart_tx.sv", "sim\link_rx_shifter_sim.sv",
  # board B 3D: render engine, fragment core, TMU, ROP
  "src\rtl\b_render.sv", "src\rtl\b_rop.sv", "src\rtl\pixel_blend_b.sv", "src\rtl\quad_setup.sv", "src\rtl\shader_core_fs2.sv",
  "src\rtl\alu_cluster.sv", "src\rtl\float_addsub.sv", "src\rtl\float_cmp.sv", "src\rtl\float_cmp3.sv", "src\rtl\float_cvt_f2i.sv",
  "src\rtl\float_cvt_i2f.sv", "src\rtl\float_dot4_sum.sv", "src\rtl\float_madd.sv", "src\rtl\float_mul.sv",
  "src\rtl\float_round.sv", "src\rtl\float_sfu.sv", "src\rtl\int_unit.sv", "src\rtl\sincos_sfu.sv", "src\rtl\vec4_sfu.sv",
  "src\rtl\vec4_sincos.sv", "src\rtl\tmu24.sv", "src\rtl\cube_rcp.sv", "src\rtl\tex_cache.sv"
) | ForEach-Object { Join-Path $root $_ } | Where-Object { Test-Path $_ }
$rtl = @(@($rtl) + @(Get-ChildItem (Join-Path $root "src\rtl\link\*.sv") | ForEach-Object { $_.FullName }) | Select-Object -Unique)
if ($Mutant -ne "") {
  $mf = (Resolve-Path $Mutant).Path; $mn = (Split-Path $mf -Leaf) -replace '_m[0-9]+\.sv$', '.sv'
  $rtl = @($rtl | ForEach-Object { if ((Split-Path $_ -Leaf) -eq $mn) { $mf } else { $_ } })
  Write-Output "MUTANT $mf replaces $mn"
}

switch ($Test) {
  "mem"     { $top = "tb_mem";     $tb = @("sim\mig_app_model.sv", "sim\tb_mem.sv") }
  "eth"     { $top = "tb_eth";     $tb = @("sim\rgmii_phy_model.sv", "sim\tb_eth.sv") }
  "dhcp"    { $top = "tb_dhcp";    $tb = @("sim\rgmii_phy_model.sv", "sim\tb_dhcp.sv") }
  "board_a" { $top = "tb_board_a"; $tb = @("sim\mig_app_model.sv", "sim\rgmii_phy_model.sv", "sim\tb_board_a.sv") }
  "video_mmcm" { $top = "tb_video_mmcm"; $tb = @("sim\tb_video_mmcm.sv") }
  "scan"    { $top = "tb_scan_reader"; $tb = @("sim\mig_app_model.sv", "sim\tb_scan_reader.sv") }
  "board_b" { $top = "tb_board_b"; $tb = @("sim\mig_app_model.sv", "sim\tmds_capture.sv", "sim\tb_board_b.sv") }
  "two"     { $top = "tb_two_boards"; $tb = @("sim\mig_app_model.sv", "sim\rgmii_phy_model.sv", "sim\tmds_capture.sv", "sim\tb_two_boards.sv") }
  "img_write" { $top = "tb_img_write"; $tb = @("sim\mig_app_model.sv", "sim\tb_img_write.sv") }
  "img_xfer"  { $top = "tb_img_xfer";  $tb = @("sim\mig_app_model.sv", "sim\tb_img_xfer.sv") }
  "disp_post" { $top = "tb_disp_post"; $tb = @("sim\tb_disp_post.sv") }
  "credit"    { $top = "tb_link_credit"; $tb = @("sim\tb_link_credit.sv") }
  "b3d"       { $top = "tb_board_b3d"; $tb = @("sim\mig_app_model.sv", "sim\tmds_capture.sv", "sim\tb_board_b3d.sv") }
  "two3d"     { $top = "tb_two_boards3d"; $tb = @("sim\mig_app_model.sv", "sim\rgmii_phy_model.sv", "sim\tmds_capture.sv", "sim\tb_two_boards3d.sv") }
  default   { Write-Output "unknown test $Test"; exit 1 }
}
$tbf = @($tb | ForEach-Object { Join-Path $root $_ })
$glbl = "$viv\..\data\verilog\src\glbl.v"
$inc = Join-Path $root "src\rtl\disp"; $inc2 = Join-Path $root "src\rtl\link"; $inc3 = Join-Path $root "sim"

$defs = @()
if ($Fast) { $defs = @("-d", "TMDS_WORD_SIM") }
if (-not $NoCompile) {
  $out = & "$viv\xvlog.bat" -sv @defs --include $inc --include $inc2 --include $inc3 @rtl @tbf $glbl 2>&1
  # the exit code also catches a killed or crashed tool, whose output has no ERROR line
  $rc = $LASTEXITCODE
  $err = $out | Select-String -Pattern "ERROR|error:"
  if ($err -or $rc -ne 0) { $err | ForEach-Object { $_.Line }; Write-Output "COMPILE_FAIL (exit $rc)"; exit 1 }
  $out = & "$viv\xelab.bat" $top glbl -L unisims_ver -L secureip -s "${top}_snap" -timescale 1ns/1ps -debug off 2>&1
  $rc = $LASTEXITCODE
  $err = $out | Select-String -Pattern "ERROR|error:"
  if ($err -or $rc -ne 0) { $err | ForEach-Object { $_.Line }; Write-Output "ELAB_FAIL (exit $rc)"; exit 1 }
  Write-Output "COMPILE_ELAB_OK $top"
}
$xargs = @("${top}_snap", "-R")
if ($Plusargs -ne "") { foreach ($p in $Plusargs -split ',') { $xargs += @("-testplusarg", $p) } }   # "A,B" -> +A +B
& "$viv\xsim.bat" @xargs 2>&1 | Select-String -Pattern "ERROR|WARNING: .*MIG|TB_PASS|TB_FAIL|FAIL|ok:|: PASS|INFO_TB|DIAG|TRACE|MON @|tb_two_boards3d|Fatal|failed" | ForEach-Object { $_.Line }
