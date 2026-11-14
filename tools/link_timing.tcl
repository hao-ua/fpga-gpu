# link_timing.tcl — board-link clock and IO timing for the board designs, sourced by tools/build_board.tcl
# after synth_design with $role = A or B (from tools/build_link.tcl, the hardware-verified link test build).
# Lane pin map: forward lane A->B = JM1[31:0] + JM2[29:22] (clock JM1[20]),
# backward lane B->A = JM2[21:0] (clock JM2[20]).
set lmhz   100
set period [expr {1000.0 / $lmhz}]
proc lane_ports {spec} {
    set l {}
    foreach {hdr bits} $spec { foreach b $bits { lappend l "${hdr}\[$b\]" } }
    return [get_ports $l]
}
set fwd_data [list jm1 {0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 23 24 25 26 27 28 29 30 31} jm2 {23 24 25 26 27 28 29}]
set bwd_data [list jm2 {0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 17}]
if {$role eq "A"} {
    set tx_clk_port {jm1[20]}; set tx_data_ports [lane_ports $fwd_data]
    set rx_clk_port {jm2[20]}; set rx_data_ports [lane_ports $bwd_data]
} else {
    set tx_clk_port {jm2[20]}; set tx_data_ports [lane_ports $bwd_data]
    set rx_clk_port {jm1[20]}; set rx_data_ports [lane_ports $fwd_data]
}
# The link transmit MMCM (link_clk, fed by the BUFG'd 200 MHz) is derived from sys_clk: give its outputs names
create_generated_clock -name link_l0   [get_pins -hierarchical -filter {NAME =~ *u_lclk/u_mmcm/CLKOUT0}]
create_generated_clock -name link_l90  [get_pins -hierarchical -filter {NAME =~ *u_lclk/u_mmcm/CLKOUT1}]
create_generated_clock -name link_l270 [get_pins -hierarchical -filter {NAME =~ *u_lclk/u_mmcm/CLKOUT3}]
create_generated_clock -name link_lfb  [get_pins -hierarchical -filter {NAME =~ *u_lclk/u_mmcm/CLKFBOUT}]
# forwarded clock out of the TX lane LCLK pin
create_generated_clock -name lclk_fwd -source [get_pins -hierarchical -filter {NAME =~ *u_phy_tx/u_oddr_clk/C}] \
    -divide_by 1 [get_ports $tx_clk_port]
# The receiver trains its capture clock to the eye it sees, so TX data is not timed against lclk_fwd
# (build_link.tcl explains why such a promise is not achievable above 100 MHz).
set_false_path -to $tx_data_ports
# recovered clock into the RX lane: centre-aligned DDR, data valid +-period/4 around each edge (documents the
# eye assumption; the capture phase is set at run time by the trainer, so the path is a calibrated path)
create_clock -name lclk_rx -period $period [get_ports $rx_clk_port]
set dv [expr {$period / 4.0 - 1.0}]
set_input_delay -clock lclk_rx -max [expr {$period/2.0 - $dv}] $rx_data_ports
set_input_delay -clock lclk_rx -min $dv $rx_data_ports
set_input_delay -clock lclk_rx -max [expr {$period/2.0 - $dv}] $rx_data_ports -clock_fall -add_delay
set_input_delay -clock lclk_rx -min $dv $rx_data_ports -clock_fall -add_delay
set_false_path -from [get_clocks lclk_rx] -to [get_clocks lclk_rx_u]
# The other link pins: control lines are launched at 3/4 period (link_phy_tx) and captured by the trainer's
# phase; the static XOFF / UP / clock-sense inputs and the parity-free control outputs are not timed at the pins.
# Every crossing between the user clock (MIG ui_clk), the lane clocks and the recovered clock is a proper CDC
# (gray-pointer async FIFOs, 2-FF synchronisers, the false-pathed tx_pkts / retx snapshots).
set_clock_groups -asynchronous \
    -group [get_clocks -include_generated_clocks lclk_rx] \
    -group [get_clocks {link_l0 link_l90 link_l270 link_lfb lclk_fwd}] \
    -group [get_clocks -include_generated_clocks sys_clk -filter {NAME !~ link_* && NAME != lclk_fwd}]
# tx_pkts / retx: lclk -> clk_sys plain register snapshots of monotonic debug counters (link_core)
set_false_path -to [get_cells -hierarchical -regexp {.*u_link/(txp_s|retx_s)_reg\[[0-9]+\]$}]
puts "LINK_TIMING role=$role tx_clk=$tx_clk_port tx_data=[llength $tx_data_ports] rx_clk=$rx_clk_port rx_data=[llength $rx_data_ports]"
