# board_a.xdc — PA200T-Starlite board A (Ethernet 100/1000 + DDR3). DDR3 pins and MIG timing come from the MIG IP
# (tools/gen/ip/mig_7series_0, generated from ip/mig_7series_0/mig_a.prj).
# Pin sources: the board's pin list (clock, reset, LEDs) and the Puzhi RGMII example (PHY #1).

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

# ---- RGMII (RTL8211F) ----
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

# Receive clock: constrained for 1000 Mbit/s (125 MHz); at 100 Mbit/s it runs at 25 MHz.
create_clock -name eth_rxc -period 8.000 [get_ports rgmii_rxc]
# RGMII receive, PHY RX delay off: data edge-aligned with RXC, skew +-0.5 ns (RGMII v1.3 TskewT),
# DDR (both edges). The input delays below document that assumption.
set rgmii_rx_in [get_ports {rgmii_rx_ctl rgmii_rxd[*]}]
set_input_delay -clock eth_rxc -max  0.5 $rgmii_rx_in
set_input_delay -clock eth_rxc -min -0.5 $rgmii_rx_in
set_input_delay -clock eth_rxc -max  0.5 $rgmii_rx_in -clock_fall -add_delay
set_input_delay -clock eth_rxc -min -0.5 $rgmii_rx_in -clock_fall -add_delay
# The capture is a calibrated path, like the board link's: the RXC->BUFG->IDDR
# clock delay (1.6 ns fast / ~2.9 ns slow corner) exceeds the pad->IBUF->IDELAY data delay
# (0.8 / ~1.6 ns at tap 0), so each edge captures the data launched by that same edge, 0.4-0.8 ns after
# its latest transition and 2.2-2.6 ns before the next one. Vivado's default edge pairing (next edge)
# does not describe this and reported a meaningless -3.6 ns hold. The working point (PHY RXDLY bit and
# IDELAY tap) is chosen on hardware by host/fb/eth_rx_sweep.py and set at run time (REG message).
# Measured on board A, 1000 Mbit/s: RXDLY off -> error-free taps 0..11 (fails from 12);
# RXDLY on -> taps 7..31 (fails at 5..6). Power-up default: RXDLY on, tap 19 (>= 12 taps = 0.94 ns
# measured clean on both sides). 100 Mbit/s works at the same setting.
set_false_path -from $rgmii_rx_in
# Transmit: TXC and data leave through identical ODDRs from the same clock; the PHY adds the 2 ns TX
# clock delay (TXDLY set over MDIO). Source-synchronous by structure, not timed at the pins.
set_false_path -to [get_ports {rgmii_txc rgmii_tx_ctl rgmii_txd[*]}]
# Slow / static pins
set_false_path -to   [get_ports {phy_rstn eth_mdc eth_mdio led[*]}]
set_false_path -from [get_ports {eth_mdio rst_btn_n}]

# ---- clock domain crossings ----
# eth_rxc <-> MIG clocks: gray-pointer FIFOs (afifo_bram), toggle-handshake snapshot, 2-FF synchronisers.
set_clock_groups -asynchronous -group [get_clocks eth_rxc] -group [get_clocks -include_generated_clocks sys_clk]
# The IDELAY control port is clocked by clk_sys; IDELAY data delay is a static setting between loads.

# ---- bitstream (QSPI boot) ----
set_property CONFIG_VOLTAGE 3.3              [current_design]
set_property CFGBVS         VCCO             [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH    4   [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE      33  [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE   Yes [current_design]
set_property CONFIG_MODE SPIx4                   [current_design]
