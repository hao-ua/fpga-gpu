# board_ab.xdc — one PA200T-Starlite board running both halves (board_ab_top): board A's Ethernet pins and timing plus
# board B's HDMI and KEY1; the DDR3 pins and MIG timing come from the MIG IP. The video and JTAG clocks are declared after
# synthesis (board_ab_clocks.xdc).

# 200 MHz oscillator (MIG system clock)
set_property PACKAGE_PIN R4           [get_ports sys_clk_p]
set_property PACKAGE_PIN T4           [get_ports sys_clk_n]
set_property IOSTANDARD  DIFF_SSTL135 [get_ports sys_clk_p]
set_property IOSTANDARD  DIFF_SSTL135 [get_ports sys_clk_n]
create_clock -name sys_clk -period 5.000 [get_ports sys_clk_p]

# reset button, KEY1, LEDs
set_property -dict {PACKAGE_PIN R14 IOSTANDARD LVCMOS33} [get_ports rst_btn_n]
set_property -dict {PACKAGE_PIN W21 IOSTANDARD LVCMOS33} [get_ports key1_n]
set_property -dict {PACKAGE_PIN W22 IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN Y22 IOSTANDARD LVCMOS33} [get_ports {led[1]}]

# Ethernet PHY #1 (RTL8211F, RGMII)
set_property -dict {PACKAGE_PIN W19 IOSTANDARD LVCMOS33} [get_ports rgmii_rxc]
set_property -dict {PACKAGE_PIN W20 IOSTANDARD LVCMOS33} [get_ports rgmii_rx_ctl]
set_property -dict {PACKAGE_PIN U18 IOSTANDARD LVCMOS33} [get_ports {rgmii_rxd[0]}]
set_property -dict {PACKAGE_PIN R19 IOSTANDARD LVCMOS33} [get_ports {rgmii_rxd[1]}]
set_property -dict {PACKAGE_PIN R18 IOSTANDARD LVCMOS33} [get_ports {rgmii_rxd[2]}]
set_property -dict {PACKAGE_PIN P20 IOSTANDARD LVCMOS33} [get_ports {rgmii_rxd[3]}]
set_property -dict {PACKAGE_PIN P17 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports rgmii_txc]
set_property -dict {PACKAGE_PIN R16 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports rgmii_tx_ctl]
set_property -dict {PACKAGE_PIN P19 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {rgmii_txd[0]}]
set_property -dict {PACKAGE_PIN P16 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {rgmii_txd[1]}]
set_property -dict {PACKAGE_PIN N17 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {rgmii_txd[2]}]
set_property -dict {PACKAGE_PIN R17 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {rgmii_txd[3]}]
set_property -dict {PACKAGE_PIN T20 IOSTANDARD LVCMOS33} [get_ports phy_rstn]
set_property -dict {PACKAGE_PIN T18 IOSTANDARD LVCMOS33} [get_ports eth_mdc]
set_property -dict {PACKAGE_PIN N15 IOSTANDARD LVCMOS33} [get_ports eth_mdio]

# RGMII receive: a calibrated path (IDELAY tap, PHY RX delay), as on board A
create_clock -name eth_rxc -period 8.000 [get_ports rgmii_rxc]
set rgmii_rx_in [get_ports {rgmii_rx_ctl rgmii_rxd[*]}]
set_input_delay -clock eth_rxc -max  0.5 $rgmii_rx_in
set_input_delay -clock eth_rxc -min -0.5 $rgmii_rx_in
set_input_delay -clock eth_rxc -max  0.5 $rgmii_rx_in -clock_fall -add_delay
set_input_delay -clock eth_rxc -min -0.5 $rgmii_rx_in -clock_fall -add_delay
set_false_path -from $rgmii_rx_in
set_false_path -to [get_ports {rgmii_txc rgmii_tx_ctl rgmii_txd[*]}]
set_false_path -to   [get_ports {phy_rstn eth_mdc eth_mdio led[*]}]
set_false_path -from [get_ports {eth_mdio rst_btn_n key1_n}]
set_clock_groups -asynchronous -group [get_clocks eth_rxc] -group [get_clocks -include_generated_clocks sys_clk]

# HDMI (TMDS_33)
set_property PACKAGE_PIN Y18  [get_ports hdmi_clk_p]
set_property PACKAGE_PIN Y19  [get_ports hdmi_clk_n]
set_property PACKAGE_PIN V18  [get_ports {hdmi_data_p[0]}]
set_property PACKAGE_PIN V19  [get_ports {hdmi_data_n[0]}]
set_property PACKAGE_PIN AA19 [get_ports {hdmi_data_p[1]}]
set_property PACKAGE_PIN AB20 [get_ports {hdmi_data_n[1]}]
set_property PACKAGE_PIN V17  [get_ports {hdmi_data_p[2]}]
set_property PACKAGE_PIN W17  [get_ports {hdmi_data_n[2]}]
set_property IOSTANDARD TMDS_33 [get_ports {hdmi_clk_p hdmi_clk_n hdmi_data_p[*] hdmi_data_n[*]}]

# configuration: QSPI x4, as both boards
set_property CONFIG_VOLTAGE 3.3              [current_design]
set_property CFGBVS         VCCO             [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH    4   [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE      33  [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE   Yes [current_design]
set_property CONFIG_MODE SPIx4                   [current_design]
