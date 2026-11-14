# board_b.xdc — PA200T-Starlite board B (DDR3 + HDMI scanout in five modes). DDR3 pins and MIG timing come from the MIG IP
# (tools/gen/ip/mig_7series_0, generated from ip/mig_7series_0/mig_a.prj). Video clock groups are in
# board_b_video.xdc (read after synthesis by tools/build_board.tcl).
# Pin sources: the board's pin list (clock, reset, LEDs, HDMI), link_pins.xdc (KEY1), the Puzhi RGMII example (PHY reset).

# ---- 200 MHz system clock, bank 34 (feeds the MIG and the video MMCM) ----
set_property PACKAGE_PIN R4           [get_ports sys_clk_p]
set_property PACKAGE_PIN T4           [get_ports sys_clk_n]
set_property IOSTANDARD  DIFF_SSTL135 [get_ports sys_clk_p]
set_property IOSTANDARD  DIFF_SSTL135 [get_ports sys_clk_n]
create_clock -name sys_clk -period 5.000 [get_ports sys_clk_p]

# ---- reset button, KEY1, LEDs (3.3 V) ----
set_property -dict {PACKAGE_PIN R14 IOSTANDARD LVCMOS33} [get_ports rst_btn_n]
set_property -dict {PACKAGE_PIN W21 IOSTANDARD LVCMOS33} [get_ports key1_n]
set_property -dict {PACKAGE_PIN W22 IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN Y22 IOSTANDARD LVCMOS33} [get_ports {led[1]}]

# ---- HDMI (TMDS_33, OSERDESE2 10:1 DDR) ----
set_property PACKAGE_PIN Y18  [get_ports hdmi_clk_p]
set_property PACKAGE_PIN Y19  [get_ports hdmi_clk_n]
set_property PACKAGE_PIN V18  [get_ports {hdmi_data_p[0]}]
set_property PACKAGE_PIN V19  [get_ports {hdmi_data_n[0]}]
set_property PACKAGE_PIN AA19 [get_ports {hdmi_data_p[1]}]
set_property PACKAGE_PIN AB20 [get_ports {hdmi_data_n[1]}]
set_property PACKAGE_PIN V17  [get_ports {hdmi_data_p[2]}]
set_property PACKAGE_PIN W17  [get_ports {hdmi_data_n[2]}]
set_property IOSTANDARD TMDS_33 [get_ports {hdmi_clk_p hdmi_clk_n hdmi_data_p[*] hdmi_data_n[*]}]

# ---- Ethernet PHY reset: held low on board B ----
set_property -dict {PACKAGE_PIN T20 IOSTANDARD LVCMOS33} [get_ports phy_rstn]

# ---- slow / static pins ----
set_false_path -from [get_ports {rst_btn_n key1_n}]
set_false_path -to   [get_ports {led[*] phy_rstn}]
# TMDS lanes leave OSERDESE2 -> OBUFDS clocked by the serial clock; there is no external timing reference.

# ---- bitstream (QSPI boot) ----
set_property CONFIG_VOLTAGE 3.3              [current_design]
set_property CFGBVS         VCCO             [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH    4   [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE      33  [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE   Yes [current_design]
set_property CONFIG_MODE SPIx4                   [current_design]
