# mig_test_jtag.tcl — read the MIG-only PHY test's status over JTAG (src/rtl/top/mig_cal_test_top.sv: BSCANE2 USER1,
# link_status_jtag, 32 words) plus the die temperature (XADC over JTAG), every interval_s, reads times.
#   vivado -mode batch -nojournal -nolog -source tools/mig_test_jtag.tcl -tclargs <serial> [reads=1] [interval_s=1]
# <serial>: the board's JTAG target serial (bringup/boards.local.tcl, not in git).
# One line per read: uptime, calibration, the traffic generator's counters and errors; the error record (words 14..31) when
# an error has been seen. Exit code 0 = calibrated and no error so far, 2 = error seen, 3 = not calibrated.
if {[llength $argv] < 1} { puts "usage: -tclargs <target-serial> [reads=1] [interval_s=1]"; exit 1 }
set serial [lindex $argv 0]
set reads  [expr {[llength $argv] > 1 ? [lindex $argv 1] : 1}]
set ival   [expr {[llength $argv] > 2 ? [lindex $argv 2] : 1}]
set NW 32
set FW [expr {$NW * 32}]
set N  [expr {8 + 8 + $FW + 8}]

open_hw_manager
connect_hw_server -allow_non_jtag
set tgt [lsearch -inline [get_hw_targets] *$serial*]
if {$tgt eq ""} { puts "ERROR: no JTAG target $serial among: [get_hw_targets]"; exit 1 }
current_hw_target $tgt
open_hw_target -jtag_mode true
run_state_hw_jtag reset
run_state_hw_jtag idle

proc hex2bin {h nbits} {
    set b ""
    foreach c [split [string toupper $h] ""] { if {[string is xdigit -strict $c]} { append b [format %04b 0x$c] } }
    set b [string repeat 0 $nbits]$b
    return [string range $b end-[expr {$nbits-1}] end]
}
proc field {bits hi lo} {
    set n [string length $bits]
    return [expr "0b[string range $bits [expr {$n-1-$hi}] [expr {$n-1-$lo}]]"]
}
# die temperature (UG480: XADC_DRP over JTAG, IR 0x37; cmd 1 = read, data on the next scan); 0x00 = now, 0x20 = max
proc xadc_temp {addr} {
    scan_ir_hw_jtag 6 -tdi 37
    scan_dr_hw_jtag 32 -tdi [format %08X [expr {(1 << 26) | ($addr << 16)}]]
    scan [scan_dr_hw_jtag 32 -tdi 00000000] %x v
    return [format %.1f [expr {(($v & 0xFFFF) >> 4) * 503.975 / 4096.0 - 273.15}]]
}
set rc 3
for {set r 0} {$r < $reads} {incr r} {
    set w {}
    for {set try 0} {$try < 5 && [llength $w] == 0} {incr try} {
        scan_ir_hw_jtag 6 -tdi 02
        set bits [hex2bin [scan_dr_hw_jtag $N -tdi 0] $N]
        if {[field $bits 7 0] == 0xA5 && [field $bits 15 8] == [field $bits [expr {$N-1}] [expr {$N-8}]]} {
            for {set k 0} {$k < $NW} {incr k} { lappend w [field $bits [expr {16 + 32*$k + 31}] [expr {16 + 32*$k}]] }
        }
    }
    if {[llength $w] == 0} { puts "[clock format [clock seconds] -format %H:%M:%S] NO FRAME (magic / seq mismatch 5 times): is the test image loaded?"; after [expr {$ival * 1000}]; continue }
    set layout [lindex $w 0]
    if {($layout & 0xFFFF0000) != 0xCA110000} { puts "[clock format [clock seconds] -format %H:%M:%S] layout [format %08X $layout] is not the MIG test's (CA11xxxx): another image is loaded"; after [expr {$ival * 1000}]; continue }
    set oc [expr {$layout & 0xFFFF}]
    set fl [lindex $w 2]
    set calib [expr {$fl & 1}]; set tgerr [expr {($fl >> 1) & 1}]; set seen [expr {($fl >> 2) & 1}]; set pat [expr {($fl >> 3) & 1}]
    set wdt [expr {($fl >> 4) & 7}]; set inrst [expr {($fl >> 7) & 1}]; set dmode [expr {($fl >> 8) & 15}]
    set wr [expr {([lindex $w 6] << 32) | [lindex $w 5]}]
    set rd [expr {([lindex $w 8] << 32) | [lindex $w 7]}]
    set tnow [xadc_temp 0]; set tmax [xadc_temp 0x20]
    puts [format "%s oc%d build %d dmode %d | up %.1f s calib %d at %d ms rst %d | wr %ld rd %ld cmds %ld rd_beats %ld | err %d (seen %d, %ld cycles) lanes %02X wdt %d pat_init %d | %s/%s C" \
        [clock format [clock seconds] -format %H:%M:%S] $oc [lindex $w 1] $dmode [expr {[lindex $w 3] / 1000.0}] $calib [lindex $w 4] [lindex $w 13] \
        $wr $rd [lindex $w 11] [lindex $w 12] $tgerr $seen [lindex $w 9] [lindex $w 10] $wdt $pat $tnow $tmax]
    if {$tgerr || $seen} {
        set es ""
        for {set k 14} {$k < 32} {incr k} { append es [format " %08X" [lindex $w $k]] }
        puts "  error_status (words 14..31, low word first):$es"
    }
    set rc [expr {($tgerr || $seen) ? 2 : ($calib ? 0 : 3)}]
    if {$r + 1 < $reads} { after [expr {$ival * 1000}] }
}
close_hw_target
disconnect_hw_server
close_hw_manager
exit $rc
