# build_mig_test.tcl — the MIG-only DDR3 PHY test (src/rtl/top/mig_cal_test_top.sv): the MIG at an
# overclocked rate + the MIG example design's traffic generator + a JTAG status register, nothing else. Run from the repo root:
#   vivado -mode batch -nojournal -source tools/build_mig_test.tcl -log tools/gen/build_mig_test_oc<MHz>_<datetime>.log -tclargs [OC_MHZ=120] [DATA_MODE=7]
# OC_MHZ 100 = the generated MIG IP (800 MT/s); 110 / 120 / 125 = the MIG's RTL with gpu_clk_mig's PLL settings (880 / 960 /
# 1000 MT/s). DATA_MODE: 7 = PRBS data, 2 = address as data. Output: tools/gen/mig_test_b<build>_oc<MHz>.bit + reports.
# The build id comes from the boards' shared counter (tools/gen/board_build_id.txt) so no two bitstreams share a number.
set oc [expr {[llength $argv] > 0 ? [lindex $argv 0] : 120}]
set dm [expr {[llength $argv] > 1 ? [lindex $argv 1] : 7}]
set ea [expr {[llength $argv] > 2 ? [lindex $argv 2] : 0x0fffffff}]   ;# last TG address (app address units); example: 0x00ffffff
set part xc7a200tfbg484-2
set_param general.maxThreads 8
set_part $part
file mkdir tools/gen

set bid_file tools/gen/board_build_id.txt
set bid 0
if {[file exists $bid_file]} { set fh [open $bid_file r]; catch {set bid [expr {int([string trim [read $fh]])}]}; close $fh }
incr bid
set fh [open $bid_file w]; puts $fh $bid; close $fh
set tag "mig_test_b${bid}_oc${oc}_d${dm}_e[format %x $ea]"
puts "=== BUILD $tag: clk_sys $oc MHz, DDR3 [expr {8 * $oc}] MT/s (tCK [expr {1000000 / (4 * $oc)}] ps), traffic generator data mode $dm, addresses 0..[format 0x%08x $ea] ==="

set ipdir tools/gen/ip/mig_7series_0
if {![file exists $ipdir/mig_7series_0.dcp]} { puts "ERROR: run tools/gen_mig.tcl first"; exit 1 }
if {$oc == 100} {
    read_ip $ipdir/mig_7series_0.xci
} else {
    set ud $ipdir/mig_7series_0/user_design
    set migv [list $ud/rtl/mig_7series_0.v $ud/rtl/mig_7series_0_mig.v]
    foreach d {clocking controller ecc ip_top phy ui} { set migv [concat $migv [glob -nocomplain $ud/rtl/$d/*.v]] }
    read_verilog $migv
    read_xdc $ud/constraints/mig_7series_0.xdc
}
read_verilog [glob $ipdir/mig_7series_0/example_design/rtl/traffic_gen/*.v]
read_verilog -sv [list src/rtl/gpu_clk_mig.sv src/rtl/link/link_status_jtag.sv src/rtl/top/mig_cal_test_top.sv]
read_xdc src/constrs/mig_cal_test.xdc
synth_design -top mig_cal_test_top -part $part -generic BUILD_ID=$bid -generic OC_MHZ=$oc -generic DATA_MODE=4'b[format %04b $dm] -generic END_ADDR=32'h[format %08x $ea]
# the JTAG status register's clock (BSCANE2 DRCK), asynchronous to everything else
create_clock -name jtag_drck -period 33.333 [get_pins -hierarchical -filter {NAME =~ *u_jtag/u_bscan/DRCK}]
set_clock_groups -asynchronous -group [get_clocks jtag_drck] -group [get_clocks -include_generated_clocks sys_clk]

proc failing_paths {file} {
    set fp [open $file w]
    foreach p [get_timing_paths -delay_type max -max_paths 500 -nworst 1 -unique_pins -slack_lesser_than 0] {
        puts $fp [format "%8.3f %s -> %s" [get_property SLACK $p] [get_property STARTPOINT_PIN $p] [get_property ENDPOINT_PIN $p]]
    }
    close $fp
}
set unplaced [get_ports -quiet -filter {PACKAGE_PIN == ""}]
puts "UNPLACED_PORTS: [llength $unplaced] $unplaced"
if {[llength $unplaced] > 0} { puts "ERROR: unplaced ports"; exit 1 }
opt_design
place_design
phys_opt_design
route_design
write_checkpoint -force tools/gen/${tag}_routed.dcp
report_timing_summary -file tools/gen/${tag}_timing.txt
failing_paths tools/gen/${tag}_failing.txt
report_utilization -file tools/gen/${tag}_util.txt
report_clock_utilization -file tools/gen/${tag}_clocks.txt
set wns [get_property SLACK [get_timing_paths -delay_type max]]
set whs [get_property SLACK [get_timing_paths -delay_type min]]
puts "TIMING WNS=$wns WHS=$whs"
write_bitstream -force tools/gen/${tag}.bit
puts "BIT_DONE tools/gen/${tag}.bit"
