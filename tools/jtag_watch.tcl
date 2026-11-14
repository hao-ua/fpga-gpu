# jtag_watch.tcl — both boards' status over JTAG every few seconds, for watching a game or a hang: JTAG takes no ACKs from the
# host, unlike STATUS (board A answers whoever spoke last). One Vivado process, so the two boards' scans never overlap.
#   vivado -mode batch -nojournal -nolog -source tools/jtag_watch.tcl -tclargs <log> [interval_s=3] [stop_file]
# Stops when stop_file exists (default <log>.stop). <log>: one compact line per poll. <log>.raw: every status word of both
# boards in hex, word 0 first (A: names in a_status_jtag.tcl, B: words in b_jtag.tcl), for fields the compact line leaves out.
# The two boards' JTAG target serials: environment GPU2_JTAG_A / GPU2_JTAG_B, else bringup/boards.local.tcl (not in git:
# `set SER_A <serial>` / `set SER_B <serial>`; Vivado's get_hw_targets lists the serials).
set logname [lindex $argv 0]
set ival    [expr {[llength $argv] > 1 ? [lindex $argv 1] : 3}]
set stopf   [expr {[llength $argv] > 2 ? [lindex $argv 2] : "$logname.stop"}]
set SER_A ""; set SER_B ""
if {[file exists bringup/boards.local.tcl]} { source bringup/boards.local.tcl }
if {[info exists ::env(GPU2_JTAG_A)]} { set SER_A $::env(GPU2_JTAG_A) }
if {[info exists ::env(GPU2_JTAG_B)]} { set SER_B $::env(GPU2_JTAG_B) }
if {$SER_A eq "" || $SER_B eq ""} { puts "ERROR: board serials unknown: set GPU2_JTAG_A / GPU2_JTAG_B or write bringup/boards.local.tcl"; exit 1 }

open_hw_manager
connect_hw_server -allow_non_jtag
set tA [lsearch -inline [get_hw_targets] *$SER_A*]
set tB [lsearch -inline [get_hw_targets] *$SER_B*]
if {$tA eq "" || $tB eq ""} { puts "ERROR: targets A '$tA' B '$tB'"; exit 1 }

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
# die temperature in C: XADC_DRP over JTAG (UG480, IR 0x37; DR {cmd[3:0] at 29:26, addr[9:0] at 25:16, data}, cmd 1 = read, the
# data comes back on the next scan). Register 0x00 now, 0x20 the maximum since power-on.
proc xadc_temp {addr} {
    scan_ir_hw_jtag 6 -tdi 37
    scan_dr_hw_jtag 32 -tdi [format %08X [expr {(1 << 26) | ($addr << 16)}]]
    scan [scan_dr_hw_jtag 32 -tdi 00000000] %x v
    return [format %.1f [expr {(($v & 0xFFFF) >> 4) * 503.975 / 4096.0 - 273.15}]]
}
# the status words of one board, or {} when the frame never read clean; temp($tgt) = {now max}
proc read_words {tgt NW} {
    global temp
    set N [expr {8 + 8 + $NW * 32 + 8}]
    current_hw_target $tgt
    open_hw_target -jtag_mode true
    run_state_hw_jtag reset
    run_state_hw_jtag idle
    set w {}
    for {set try 0} {$try < 5 && [llength $w] == 0} {incr try} {
        scan_ir_hw_jtag 6 -tdi 02
        set bits [hex2bin [scan_dr_hw_jtag $N -tdi 0] $N]
        if {[field $bits 7 0] == 0xA5 && [field $bits 15 8] == [field $bits [expr {$N-1}] [expr {$N-8}]]} {
            for {set k 0} {$k < $NW} {incr k} { lappend w [field $bits [expr {16 + 32*$k + 31}] [expr {16 + 32*$k}]] }
        }
    }
    set temp($tgt) [list [xadc_temp 0] [xadc_temp 0x20]]
    close_hw_target
    return $w
}

set fd  [open $logname a]
set fdr [open $logname.raw a]
set hdr "time T:A_now/max,B_now/max B:up_s build cp rb stack trn far lock eye presents cmdpk cmderr | A:up_s errs last_err@seq exp_seq lf(trn/far/lock/alive) eye fwd bev crc/par disc/rew a_perr lock_drops trainings b_crc/par b_trn unconsumed credit_syncs rs_inflight/pkts geom_busy q_used"
puts $fd $hdr; flush $fd
while {![file exists $stopf]} {
    set t [clock format [clock seconds] -format %H:%M:%S]
    # STATUS layout 4: 48 words, the 24 status words and the 24 profile counters; a lower layout's 24-word
    # frame read as 48 fails the sequence check or shows a layout below 4
    set b [read_words $tB 48]
    if {[llength $b] == 0 || [lindex $b 0] < 0xB0000004} { set b [read_words $tB 24] }
    set a [read_words $tA 116]                          ;# STATUS layout 10: 116 words, as a_status_jtag.tcl
    if {[llength $a] == 0 || [llength $b] == 0} {
        puts $fd "$t READ_FAIL a [llength $a] b [llength $b]"
    } else {
        set fl [lindex $b 3]; set blf [lindex $b 22]; set lf [lindex $a 42]
        set line [format "%s T:%s/%s,%s/%s B:%d %d %d %d %d %d %d %d %d %d %d %d | A:%d %d %08X@%d %d %d/%d/%d/%d %d %d %d %d/%d %d/%d %d %d %d %d/%d %d %d %d %d/%d %08X %d" \
            $t [lindex $temp($tA) 0] [lindex $temp($tA) 1] [lindex $temp($tB) 0] [lindex $temp($tB) 1] \
            [expr {[lindex $b 2] / 1000}] [lindex $b 1] [expr {($fl >> 12) & 1}] [expr {($fl >> 26) & 1}] [expr {($fl >> 27) & 1}] \
            [expr {($blf >> 2) & 1}] [expr {($blf >> 3) & 1}] [expr {($blf >> 1) & 1}] [expr {($blf >> 24) & 255}] \
            [lindex $b 10] [expr {[lindex $b 23] >> 16}] [expr {[lindex $b 23] & 0xFFFF}] \
            [expr {[lindex $a 2] / 1000}] [lindex $a 29] [lindex $a 30] [lindex $a 31] [lindex $a 19] \
            [expr {($lf >> 2) & 1}] [expr {($lf >> 3) & 1}] [expr {($lf >> 1) & 1}] [expr {$lf & 1}] [expr {($lf >> 24) & 255}] \
            [lindex $a 43] [lindex $a 44] [expr {[lindex $a 46] >> 16}] [expr {[lindex $a 46] & 0xFFFF}] \
            [expr {[lindex $a 47] >> 16}] [expr {[lindex $a 47] & 0xFFFF}] [lindex $a 57] [lindex $a 58] [lindex $a 59] \
            [expr {[lindex $a 60] >> 16}] [expr {[lindex $a 60] & 0xFFFF}] [lindex $a 61] \
            [expr {([lindex $a 76] - [lindex $a 77]) & 0xFFFFFFFF}] [expr {[lindex $a 79] & 0xFFFF}] \
            [expr {[lindex $a 87] >> 16}] [expr {[lindex $a 87] & 0xFFFF}] [lindex $a 84] [lindex $a 26]]
        puts $fd $line
        set hb {}; foreach x $b { lappend hb [format %08X $x] }
        set ha {}; foreach x $a { lappend ha [format %08X $x] }
        puts $fdr "$t B $hb"
        puts $fdr "$t A $ha"
        flush $fdr
    }
    flush $fd
    after [expr {int($ival * 1000)}]
}
close $fd; close $fdr
disconnect_hw_server
close_hw_manager
