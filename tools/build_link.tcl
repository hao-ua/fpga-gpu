# build_link.tcl — build link_test_top for one role.  Run from the repo root:
#   vivado -mode batch -source tools/build_link.tcl -tclargs <A|B> [LINK_MHZ]
# Output: tools/gen/link_test_<role>_<mhz>.bit (+ timing/utilisation reports)
set role [lindex $argv 0]
set mhz  [expr {[llength $argv] > 1 ? [lindex $argv 1] : 50}]
if {$role ne "A" && $role ne "B"} { puts "usage: -tclargs <A|B> [LINK_MHZ]"; exit 1 }
set part xc7a200tfbg484-2
set_param general.maxThreads 8
file mkdir tools/gen
set tag "link_test_${role}_${mhz}"
puts "=== BUILD $tag ==="

read_verilog -sv [concat src/rtl/common/uart_tx.sv [glob src/rtl/link/*.sv]]
read_xdc src/constrs/link_pins.xdc
set role_hex [expr {$role eq "A" ? "8'h41" : "8'h42"}]
synth_design -top link_test_top -part $part -include_dirs src/rtl/link \
    -generic ROLE=$role_hex -generic LINK_MHZ=$mhz

# ---- role-dependent source-synchronous timing (after synth so the ODDR pins exist) ----
# Lane pin map: forward lane A->B = JM1[31:0] + JM2[29:22] (clock JM1[20]),
# backward lane B->A = JM2[21:0] (clock JM2[20]). DDR data/parity ports get the output/input delay
# constraints; the control lines (launched at 3/4 period from lclk90, see link_phy_tx) are left to
# the automatic lclk -> lclk90 internal check and are not constrained at the pins.
set period [expr {1000.0 / $mhz}]
proc lane_ports {spec} {
    # spec = list of {header bitlist}
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
# forwarded clock out of the TX lane LCLK pin
create_generated_clock -name lclk_fwd -source [get_pins -hierarchical -filter {NAME =~ *u_phy_tx/u_oddr_clk/C}] \
    -divide_by 1 [get_ports $tx_clk_port]
# The receiver trains its capture clock to the data eye it actually sees, so the position of the
# forwarded clock edge relative to the data is irrelevant (what matters is data-to-data skew, which
# the uniform IOB/ODDR structure keeps within ~0.3 ns, and the measured EYE). The data outputs are
# therefore not timed against lclk_fwd: a +-period/10 promise is not achievable above 100 MHz (the
# ODDR->pad vs clock ODDR->pad relationship is ~1.5 ns at the fast corner) and would only report
# meaningless failures.
set_false_path -to $tx_data_ports
# recovered clock into the RX lane: centre-aligned DDR, data valid +-period/4 around each edge
create_clock -name lclk_rx -period $period [get_ports $rx_clk_port]
set dv [expr {$period / 4.0 - 1.0}]          ;# data valid this long before/after each edge
set_input_delay -clock lclk_rx -max [expr {$period/2.0 - $dv}] $rx_data_ports
set_input_delay -clock lclk_rx -min $dv $rx_data_ports
set_input_delay -clock lclk_rx -max [expr {$period/2.0 - $dv}] $rx_data_ports -clock_fall -add_delay
set_input_delay -clock lclk_rx -min $dv $rx_data_ports -clock_fall -add_delay
# The RX capture clock phase is found at run time by the trainer in link_phy_rx (MMCM dynamic fine
# phase shift), so the static check of pin -> IDDR against the recovered clock has no meaning: the
# input-delay constraints above only document the eye assumption, the path is a calibrated path.
set_false_path -from [get_clocks lclk_rx] -to [get_clocks lclk_rx_u]
# JTAG status register (BSCANE2 DRCK, <= ~30 MHz): its own domain; the user-domain snapshot is read
# asynchronously at CAPTURE (torn reads are detected by the reader via the duplicated seq field)
create_clock -name jtag_drck -period 33.333 [get_pins -hierarchical -filter {NAME =~ *u_bscan/DRCK}]
# Clock groups. clk_sys (100 MHz) and the lane clocks lclk/lclk90 come from the same MMCM, but every
# crossing between them is a proper CDC (gray-pointer async FIFOs, 2-FF synchronisers, the false-
# pathed tx_pkts snapshot): declare them asynchronous, otherwise above 100 MHz STA checks them with
# the small edge offsets of the 100:125 / 100:150 ratios and reports hundreds of false failures.
set_clock_groups -asynchronous \
    -group [get_clocks -include_generated_clocks lclk_rx] \
    -group [get_clocks clk_sys_u] \
    -group [get_clocks {lclk_u lclk90_u lclk270_u lclk_fwd}] \
    -group [get_clocks jtag_drck]

write_checkpoint -force tools/gen/${tag}_synth.dcp
report_timing_summary -file tools/gen/${tag}_timing_synth.txt
set unplaced [get_ports -quiet -filter {PACKAGE_PIN == ""}]
puts "UNPLACED_PORTS: [llength $unplaced] $unplaced"
puts "PIN_CHECK jm1\[20\]=[get_property PACKAGE_PIN [get_ports {jm1[20]}]] jm2\[20\]=[get_property PACKAGE_PIN [get_ports {jm2[20]}]]"
puts "LANE_CHECK tx_clk=$tx_clk_port tx_data=[llength $tx_data_ports] rx_clk=$rx_clk_port rx_data=[llength $rx_data_ports]"
if {[llength $unplaced] > 0} { puts "ERROR: unplaced ports"; exit 1 }
if {[info exists ::env(SYNTH_ONLY)]} { puts "SYNTH_ONLY_DONE $tag"; exit 0 }
opt_design
place_design
phys_opt_design
route_design
report_timing_summary -file tools/gen/${tag}_timing.txt
report_utilization    -file tools/gen/${tag}_util.txt
report_utilization    -hierarchical -hierarchical_depth 3 -file tools/gen/${tag}_util_hier.txt
report_io             -file tools/gen/${tag}_io.txt
write_bitstream -force tools/gen/${tag}.bit
puts "BIT_DONE ${tag}.bit"
