# a_status_jtag.tcl — read board A's status vector over JTAG (BSCANE2 USER1, link_status_jtag).
#   vivado -mode batch -nojournal -source tools/a_status_jtag.tcl -tclargs <serial> [reads=1] [interval_s=1]
# Frame (LSB first on TDO): {MAGIC=A5, seq, status_vec (word 0 first), seq}; status_vec = STATUS_WORDS
# (116, layout 10) x 32 bits, the same words as the STATUS datagram
# (src/rtl/top/board_a_core.sv, host/fb/fbproto.py).
if {[llength $argv] < 1} { puts "usage: -tclargs <target-serial> [reads=1] [interval_s=1]"; exit 1 }
set serial [lindex $argv 0]
set reads  [expr {[llength $argv] > 1 ? [lindex $argv 1] : 1}]
set ival   [expr {[llength $argv] > 2 ? [lindex $argv 2] : 1}]
set NW 116
set FW [expr {$NW * 32}]
set N  [expr {8 + 8 + $FW + 8}]

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
set names {layout build uptime_ms flags phy rx_ok rx_fcs rx_len rx_ovf rx_er arp icmp udp ipcsum other pfdrop icmpdrop
           tx tx_resync exp_seq acc dup gap full bad acks q_used q_free cmds errs last_err last_err_seq mt_err mdio_polls
           snap_age replies replies_dropped b_flags b_frames b_pix_khz b_ur_moderr b_cp link_flags fwd b_events b_status
           link_crc_par link_disc_retx linkctl link_parmap ld_bursts ld_bursts_lg ld_hist ld_last_err ld_last_word ld_fills
           ld_fill_perr ld_perrs link_drops link_frm_trn b_link_crc_par b_link_info b_images b_pixels
           b_ld_hist b_ld_last_err b_link_parmap b_xoff b_retx b_disc b_cmd_pkts b_rx_words a_xoff a_naks_to a_rx_words reserved
           cr_sent cr_consumed cr_wait cur_sync b_cb_taken b_cur_pal b_curcmd_xops b_x_pixels
           geom_busy_tris geom_err render_quads rs_pkts_inflight
           b_cyc_frame b_cyc_flush b_cyc_band b_cyc_draw b_cyc_fs b_cyc_bandload b_cyc_starved b_cyc_ropstall
           a_cyc_geom a_cyc_render a_cyc_rs_credit a_cyc_rs_full a_visits
           a_prof_load a_prof_copy a_prof_feed a_prof_fetch_wait a_prof_core a_prof_asm a_prof_vo_stall a_prof_setup
           a_prof_push_stall a_prof_bin a_prof_clear a_prof_loads_draws a_prof_prims a_disp_idle a_disp_gwait}

for {set r 0} {$r < $reads} {incr r} {
    set ok 0
    for {set try 0} {$try < 5 && !$ok} {incr try} {
        scan_ir_hw_jtag 6 -tdi 02
        set hex [scan_dr_hw_jtag $N -tdi 0]
        set bits [hex2bin $hex $N]
        set magic [field $bits 7 0]
        set seq0  [field $bits 15 8]
        set seq1  [field $bits [expr {$N-1}] [expr {$N-8}]]
        if {$magic == 0xA5 && $seq0 == $seq1} { set ok 1 }
    }
    if {!$ok} { puts "READ_FAIL magic=[format %02X $magic] seq=$seq0/$seq1"; continue }
    set w {}
    for {set k 0} {$k < $NW} {incr k} { lappend w [field $bits [expr {16 + 32*$k + 31}] [expr {16 + 32*$k}]] }
    set fl [lindex $w 3]
    set spd [lindex {10M 100M 1000M ?} [expr {($fl >> 2) & 3}]]
    puts [format "STATUS seq=%d build=%d up=%dms calib=%d link=%d speed=%s fdx=%d mdio_init=%d host=%d tap=%d rxdly=%d adv1000=%d phyad=%d PHYSR=%04X BMSR=%04X" \
        $seq0 [lindex $w 1] [lindex $w 2] [expr {$fl & 1}] [expr {($fl >> 1) & 1}] $spd [expr {($fl >> 4) & 1}] \
        [expr {($fl >> 5) & 1}] [expr {($fl >> 6) & 1}] [expr {($fl >> 8) & 31}] [expr {($fl >> 13) & 1}] \
        [expr {($fl >> 14) & 1}] [expr {($fl >> 16) & 31}] [expr {[lindex $w 4] & 0xFFFF}] [expr {[lindex $w 4] >> 16}]]
    set line "  "
    for {set k 5} {$k < 37} {incr k} { append line [format "%s=%ld " [lindex $names $k] [lindex $w $k]] }
    puts $line
    set lf [lindex $w 42]; set bf [lindex $w 37]
    puts [format "  LINK trained=%d far_up=%d lock=%d b_alive=%d eye=%d fwd=%d b_events=%d b_status=%d crc=%d par=%d disc=%d rewinds=%d | B video=%d mode=%d img=%d frames=%d pix=%dkHz underruns=%d mode_err=%d cmds=%d errs=%d" \
        [expr {($lf >> 2) & 1}] [expr {($lf >> 3) & 1}] [expr {($lf >> 1) & 1}] [expr {$lf & 1}] [expr {($lf >> 24) & 255}] \
        [lindex $w 43] [lindex $w 44] [lindex $w 45] [expr {[lindex $w 46] >> 16}] [expr {[lindex $w 46] & 0xFFFF}] \
        [expr {[lindex $w 47] >> 16}] [expr {[lindex $w 47] & 0xFFFF}] \
        [expr {($bf >> 2) & 1}] [expr {($bf >> 4) & 7}] [expr {($bf >> 16) & 1023}] [lindex $w 38] [lindex $w 39] \
        [expr {[lindex $w 40] >> 16}] [expr {[lindex $w 40] & 0xFFFF}] [expr {[lindex $w 41] & 0xFFFF}] [expr {[lindex $w 41] >> 16}]]
    set h [lindex $w 52]; set le [lindex $w 53]
    puts [format "  DIAG fill_A=%d fill_B=%d rx_parity=%d parmap=%08X fills=%d fill_par=%d lock_drops=%d trn=%d hist(pos0/1-3/4+/longgap)=%d/%d/%d/%d bursts=%d long_gap=%d last(gap=%d pos=%d last=%d par=%X word=%08X) | B rx crc=%d par=%d | B images=%d pixels=%d" \
        [expr {[lindex $w 48] & 1}] [expr {([lindex $w 61] >> 16) & 1}] [lindex $w 57] [lindex $w 49] [lindex $w 55] [lindex $w 56] [lindex $w 58] [expr {[lindex $w 59] & 255}] \
        [expr {$h >> 24}] [expr {($h >> 16) & 255}] [expr {($h >> 8) & 255}] [expr {$h & 255}] [lindex $w 50] [lindex $w 51] \
        [expr {$le >> 16}] [expr {($le >> 8) & 255}] [expr {($le >> 4) & 1}] [expr {$le & 15}] [lindex $w 54] \
        [expr {[lindex $w 60] >> 16}] [expr {[lindex $w 60] & 0xFFFF}] [lindex $w 62] [lindex $w 63]]
    set bh [lindex $w 64]; set ble [lindex $w 65]
    puts [format "  B RX hist(pos0/1-3/4+/longgap)=%d/%d/%d/%d last(gap=%d pos=%d last=%d par=%X) parmap=%08X xoff=%d tx_timeouts=%d tx_rewinds=%d naks=%d disc=%d cmd_pkts=%d words=%d | A xoff=%d naks=%d tx_timeouts=%d words=%d" \
        [expr {$bh >> 24}] [expr {($bh >> 16) & 255}] [expr {($bh >> 8) & 255}] [expr {$bh & 255}] \
        [expr {$ble >> 16}] [expr {($ble >> 8) & 255}] [expr {($ble >> 4) & 1}] [expr {$ble & 15}] [lindex $w 66] [lindex $w 67] \
        [expr {[lindex $w 68] >> 16}] [expr {[lindex $w 68] & 0xFFFF}] [expr {[lindex $w 69] >> 16}] [expr {[lindex $w 69] & 0xFFFF}] \
        [lindex $w 70] [lindex $w 71] [lindex $w 72] [expr {[lindex $w 73] >> 16}] [expr {[lindex $w 73] & 0xFFFF}] [lindex $w 74]]
    if {$r < $reads - 1} { after [expr {int($ival * 1000)}] }
}
close_hw_target
disconnect_hw_server
close_hw_manager
