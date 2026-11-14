# b_jtag.tcl — board B over JTAG (bring-up): status (USER1, link_status_jtag) and commands (USER2,
# jtag_ctrl_reg -> b_bringup, 80-bit scans with a signature). Works without UART, Ethernet or the board link.
#   vivado -mode batch -nojournal -nolog -source tools/b_jtag.tcl -tclargs <serial> status [reads=1] [interval_s=1]
#   vivado ... -tclargs <serial> mode <0..3>                  0 640x480, 1 800x600, 2 1280x720, 3 1280x800
#   vivado ... -tclargs <serial> present <image> [now]        image 0..9 (b_bringup image set), now = flip at once
#   vivado ... -tclargs <serial> memtest <start_MB> <len_MB> [seed]   write + verify; REFILL afterwards if it hit 256 MB+
#   vivado ... -tclargs <serial> refill                       regenerate the test images
#   vivado ... -tclargs <serial> key short|long               same as pressing KEY1
#   vivado ... -tclargs <serial> linkctl <value>              board link control (b0 = fill words on B's transmit lane)
# A command is followed by one status read 0.5 s later (memtest: after 30 s per GB).
# Status frame (LSB first on TDO): {MAGIC=A5, seq, status_vec (word 0 first), seq}; 24 x 32-bit words (48 in STATUS layout 4), see
# src/rtl/top/board_b_core.sv.
if {[llength $argv] < 2} { puts "usage: -tclargs <target-serial> status|mode|present|memtest|refill|key ..."; exit 1 }
set serial [lindex $argv 0]
set op     [lindex $argv 1]
set NW 24
set N  [expr {8 + 8 + $NW * 32 + 8}]

open_hw_manager
connect_hw_server -allow_non_jtag
set tgt [lsearch -inline [get_hw_targets] *$serial*]
if {$tgt eq ""} { puts "ERROR: no JTAG target $serial"; exit 1 }
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
# one command = an 80-bit USER2 scan {signature B5C3, command[63:0]} (jtag_ctrl_reg ignores other scans)
proc send_cmd {hi lo} {
    scan_ir_hw_jtag 6 -tdi 03
    set rb [scan_dr_hw_jtag 80 -tdi [format B5C3%08X%08X $hi $lo]]
    puts [format "CMD sent %08X%08X (previous accepted scan read back: %s)" $hi $lo $rb]
}
proc read_status {N0 NW0} {
    # layout 4: the frame holds 48 words — the 24 status words and the 24 profile counters (b_render hwprof);
    # a frame of a lower layout holds 24 (read as 48 it fails the sequence check, or shows a layout below 4)
    set ok 0
    foreach NW {48 24} {
    set N [expr {8 + 8 + $NW * 32 + 8}]
    for {set try 0} {$try < 5 && !$ok} {incr try} {
        scan_ir_hw_jtag 6 -tdi 02
        set hex [scan_dr_hw_jtag $N -tdi 0]
        set bits [hex2bin $hex $N]
        set magic [field $bits 7 0]
        set seq0  [field $bits 15 8]
        set seq1  [field $bits [expr {$N-1}] [expr {$N-8}]]
        if {$magic == 0xA5 && $seq0 == $seq1 && ($NW == 24 || [field $bits 47 16] >= 0xB0000004)} { set ok 1 }
    }
    if {$ok} break
    }
    if {!$ok} { puts "READ_FAIL magic=[format %02X $magic] seq=$seq0/$seq1"; return }
    set w {}
    for {set k 0} {$k < $NW} {incr k} { lappend w [field $bits [expr {16 + 32*$k + 31}] [expr {16 + 32*$k}]] }
    set fl [lindex $w 3]
    set fcyc [lindex $w 7]
    set hz [expr {$fcyc > 0 ? 100.0e6 / $fcyc : 0.0}]
    set modes {640x480 800x600 1280x720 1280x800 64x48 80x40 ? ?}
    set m [expr {($fl >> 4) & 7}]
    puts [format "STATUS seq=%d layout=%08X build=%d up=%dms calib=%d mmcm_locked=%d video=%d mode_busy=%d mode=%d(%s) image_shown=%d fill_done=%d memtest_busy=%d key=%d lock_fail=%d cur_img=%d" \
        $seq0 [lindex $w 0] [lindex $w 1] [lindex $w 2] [expr {$fl & 1}] [expr {($fl >> 1) & 1}] [expr {($fl >> 2) & 1}] \
        [expr {($fl >> 3) & 1}] $m [lindex $modes $m] [expr {($fl >> 7) & 1}] [expr {($fl >> 8) & 1}] \
        [expr {($fl >> 9) & 1}] [expr {($fl >> 10) & 1}] [expr {($fl >> 11) & 1}] [expr {($fl >> 16) & 1023}]]
    puts [format "  frames=%d underruns=%d pixel_clock=%d kHz frame_rate=%.3f Hz mode_ok=%d mode_err=%d presents=%d dropped=%d presented_tag=%d" \
        [lindex $w 4] [lindex $w 5] [lindex $w 6] $hz [lindex $w 8] [lindex $w 9] [lindex $w 10] [lindex $w 11] [lindex $w 12]]
    if {[lindex $w 0] >= 0xB0000003} {
        # layout 3 (BRINGUP off): 13-17 = render quads / early-depth drops (2) / triangle visits / frames, 18-21 = the
        # texture unit's tap jobs / A-stage stall cycles / cache hits / cache misses (cumulative)
        set hits [lindex $w 20]; set misses [lindex $w 21]
        set mr [expr {$hits + $misses > 0 ? 100.0 * $misses / ($hits + $misses) : 0.0}]
        puts [format "  render: quads=%d ez_drops=%d/%d tris=%d frames=%d | TMU: jobs=%d astall=%d cache hits=%d misses=%d (%.1f%% misses)" \
            [lindex $w 13] [lindex $w 14] [lindex $w 15] [lindex $w 16] [lindex $w 17] [lindex $w 18] [lindex $w 19] $hits $misses $mr]
    } else {
    puts [format "  cmds=%d last_cmd=%08X%08X harness_errs=%d memtest_runs=%d memtest_errors=%d memtest_first_bad=0x%08X fills=%d keys_short=%d keys_long=%d" \
        [lindex $w 13] [lindex $w 14] [lindex $w 15] [lindex $w 16] [lindex $w 17] [lindex $w 18] [lindex $w 19] \
        [lindex $w 20] [expr {[lindex $w 21] & 0xFFFF}] [expr {[lindex $w 21] >> 16}]]
    }
    if {[lindex $w 0] >= 0xB0000002} {
        set lf [lindex $w 22]
        puts [format "  link: trained=%d far_up=%d lock=%d tx_enabled=%d guard_fail=%d eye=%d phase=%d | CMD packets=%d command errors=%d cp_busy=%d render_busy=%d stack_err=%d" \
            [expr {($lf >> 2) & 1}] [expr {($lf >> 3) & 1}] [expr {($lf >> 1) & 1}] [expr {($lf >> 5) & 1}] [expr {($lf >> 6) & 1}] \
            [expr {($lf >> 24) & 255}] [expr {($lf >> 16) & 255}] [expr {[lindex $w 23] >> 16}] [expr {[lindex $w 23] & 0xFFFF}] \
            [expr {($fl >> 12) & 1}] [expr {($fl >> 26) & 1}] [expr {($fl >> 27) & 1}]]
    }
    if {$NW >= 48} {
        # the profile counters, cumulative (tools/b_prof.ps1 turns a jtag_watch log of them into the split of the frame)
        set p {}; for {set k 24} {$k < 48} {incr k} { lappend p [lindex $w $k] }
        puts "  profile (words 24-47): $p"
    }
}

set wait_ms 500
switch -- $op {
    status {
        set reads [expr {[llength $argv] > 2 ? [lindex $argv 2] : 1}]
        set ival  [expr {[llength $argv] > 3 ? [lindex $argv 3] : 1}]
        for {set r 0} {$r < $reads} {incr r} {
            read_status $N $NW
            if {$r < $reads - 1} { after [expr {int($ival * 1000)}] }
        }
        set wait_ms -1
    }
    mode    { send_cmd 0x01000000 [expr {[lindex $argv 2] & 7}] }
    present {
        set now [expr {[llength $argv] > 3 && [lindex $argv 3] eq "now" ? 1 : 0}]
        send_cmd 0x02000000 [expr {($now << 16) | ([lindex $argv 2] & 1023)}]
    }
    memtest {
        set st [lindex $argv 2]; set ln [lindex $argv 3]
        set seed [expr {[llength $argv] > 4 ? [lindex $argv 4] : 0x13572468}]
        send_cmd [expr {(0x03 << 24) | (($st & 1023) << 14) | (($ln & 1023) << 4)}] [expr {$seed & 0xFFFFFFFF}]
        set wait_ms [expr {500 + 30 * $ln}]
    }
    refill  { send_cmd 0x04000000 0 }
    key     { send_cmd 0x05000000 [expr {[lindex $argv 2] eq "long" ? 1 : 0}] }
    linkctl { send_cmd 0x06000000 [expr {[lindex $argv 2] & 0xFFFFFFFF}] }
    default { puts "unknown command $op"; set wait_ms -1 }
}
if {$wait_ms >= 0} { after $wait_ms; read_status $N $NW }
close_hw_target
disconnect_hw_server
close_hw_manager
