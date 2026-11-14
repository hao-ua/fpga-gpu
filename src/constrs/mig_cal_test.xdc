# mig_cal_test.xdc — PA200T-Starlite, the MIG-only PHY test (src/rtl/top/mig_cal_test_top.sv).
# DDR3 pins and the MIG's own timing come from the MIG's generated XDC, read by tools/build_mig_test.tcl.
# Pins as src/constrs/board_a.xdc: clock, reset button, LEDs.

# ---- 200 MHz system clock, bank 34 (feeds the MIG) ----
set_property PACKAGE_PIN R4           [get_ports sys_clk_p]
set_property PACKAGE_PIN T4           [get_ports sys_clk_n]
set_property IOSTANDARD  DIFF_SSTL135 [get_ports sys_clk_p]
set_property IOSTANDARD  DIFF_SSTL135 [get_ports sys_clk_n]
create_clock -name sys_clk -period 5.000 [get_ports sys_clk_p]

# ---- reset button, LEDs (bank 13/14, 3.3 V) ----
set_property -dict {PACKAGE_PIN R14 IOSTANDARD LVCMOS33} [get_ports rst_btn_n]
set_property -dict {PACKAGE_PIN W22 IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN Y22 IOSTANDARD LVCMOS33} [get_ports {led[1]}]
set_false_path -to   [get_ports {led[*]}]
set_false_path -from [get_ports rst_btn_n]

# ---- bitstream (QSPI boot options as the board images; this one is only ever loaded over JTAG) ----
set_property CONFIG_VOLTAGE 3.3              [current_design]
set_property CFGBVS         VCCO             [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH    4   [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE      33  [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE   Yes [current_design]
set_property CONFIG_MODE SPIx4                   [current_design]
