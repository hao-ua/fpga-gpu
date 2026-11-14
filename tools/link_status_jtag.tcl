# link_status_jtag.tcl — read the link_test status fields over JTAG (BSCANE2 USER1).
#   vivado -mode batch -source tools/link_status_jtag.tcl -tclargs <serial> [reads] [interval_s]
# Field layout must match link_test_top.sv `fields` (FW = 20 + 3*8 + 13*32 + 32 + 32 = 524 bits):
#   {lock(4)={0,0,trained,lock} rxclk(4) up(4)={0,0,training,far_up} guard(4) txen(4) trn(8) eye(8) ph(8)
#    txp rxw rxp par crc prbs seq frm xoff drop parmap retx disc biterr t}
# PARMAP = parity errors per half, saturating 8-bit: 64-bit lane {hi PAR1, hi PAR0, lo PAR1, lo PAR0},
# 32-bit lane {0, 0, high half, low half}. RETX = {timeouts16, rewinds16} of our transmitter,
# DISC = {naks16, discarded16} of our receiver (ARQ).
# Shorter layouts: FW=460 (no RETX/DISC), 428 (no PARMAP).
# The USER1 chain is 8+8+FW+8 bits. If in doubt, probe with a longer all-ones scan: the fill shows
# where the chain ends.
if {[llength $argv] < 1} { puts "usage: -tclargs <target-serial> [reads=1] [interval_s=1] [FW=460|428]"; exit 1 }
set serial [lindex $argv 0]
set reads  [expr {[llength $argv] > 1 ? [lindex $argv 1] : 1}]
set ival   [expr {[llength $argv] > 2 ? [lindex $argv 2] : 1}]
set FW     [expr {[llength $argv] > 3 ? [lindex $argv 3] : 524}]   ;# 460 = no RETX/DISC, 428 = no PARMAP
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
    # hex string (MSB first, as returned by scan_dr_hw_jtag) -> binary string of exactly nbits, MSB first
    set b ""
    foreach c [split [string toupper $h] ""] {
        if {![string is xdigit -strict $c]} { continue }
        append b [format %04b 0x$c]
    }
    set b [string repeat 0 $nbits]$b
    return [string range $b end-[expr {$nbits-1}] end]
}
proc field {bits hi lo} {
    # $bits is MSB-first binary of the whole frame; bit index counted from the LSB
    set n [string length $bits]
    set s [string range $bits [expr {$n-1-$hi}] [expr {$n-1-$lo}]]
    return [expr "0b$s"]
}

for {set r 0} {$r < $reads} {incr r} {
    set ok 0
    for {set try 0} {$try < 5 && !$ok} {incr try} {
        scan_ir_hw_jtag 6 -tdi 02                                  ;# USER1 (7-series IR = 6 bits)
        set hex [scan_dr_hw_jtag $N -tdi 0]
        set bits [hex2bin $hex $N]
        set magic [field $bits 7 0]
        set seq0  [field $bits 15 8]
        set seq1  [field $bits [expr {$N-1}] [expr {$N-8}]]
        if {$magic == 0xA5 && $seq0 == $seq1} { set ok 1 }
    }
    if {!$ok} { puts "READ_FAIL magic=[format %02X $magic] seq=$seq0/$seq1"; continue }
    set f [string range $bits [expr {$N-1-($FW+15)}] [expr {$N-1-16}]]   ;# fields, MSB first
    # walk the fields vector from the MSB
    set pos 0
    proc take {w} { global f pos; set s [string range $f $pos [expr {$pos+$w-1}]]; incr pos $w; return [expr "0b$s"] }
    set lockn [take 4]; set rxclk [take 4]; set upn [take 4]; set guard [take 4]; set txen [take 4]
    set trn [take 8]; set eye [take 8]; set ph [take 8]
    set txp [take 32]; set rxw [take 32]; set rxp [take 32]; set par [take 32]; set crc [take 32]
    set prbs [take 32]; set sq [take 32]; set frm [take 32]; set xoff [take 32]; set drop [take 32]
    if {$FW >= 460} { set parmap [take 32] } else { set parmap 0 }
    if {$FW >= 524} { set retx [take 32]; set disc [take 32] } else { set retx 0; set disc 0 }
    set biterr [take 32]; set t [take 32]
    puts [format "STATUS seq=%d LOCK=%d TRAINED=%d RXCLK=%d UP=%d TRAINING=%d GUARD=%d TXEN=%d TRN=%d EYE=%d PH=%d TXP=%ld RXW=%ld RXP=%ld PAR=%ld CRC=%ld PRBS=%ld SEQ=%ld FRM=%ld XOFF=%ld DROP=%ld PARMAP=%08X RETX=%d/%d DISC=%d/%d BITERR=%08X T=%ld" \
        $seq0 [expr {$lockn & 1}] [expr {($lockn >> 1) & 1}] $rxclk [expr {$upn & 1}] [expr {($upn >> 1) & 1}] $guard $txen $trn $eye $ph \
        $txp $rxw $rxp $par $crc $prbs $sq $frm $xoff $drop $parmap [expr {$retx >> 16}] [expr {$retx & 0xFFFF}] [expr {$disc >> 16}] [expr {$disc & 0xFFFF}] $biterr $t]
    if {$r < $reads - 1} { after [expr {int($ival * 1000)}] }
}
close_hw_target
disconnect_hw_server
close_hw_manager
