# program.tcl — volatile JTAG programming of one board selected by its Digilent/FTDI serial.
#   vivado -mode batch -source bringup/program.tcl -tclargs <serial> <bitfile>
# The serials of your boards: Vivado's hardware manager lists them (get_hw_targets); keep them in bringup/boards.local.tcl
# (not in git).
if {[llength $argv] != 2} { puts "usage: -tclargs <target-serial> <bitfile>"; exit 1 }
set serial [lindex $argv 0]
set bit    [lindex $argv 1]
if {![file exists $bit]} { puts "ERROR: no such bitstream: $bit"; exit 1 }
open_hw_manager
connect_hw_server -allow_non_jtag
set tgt [lsearch -inline [get_hw_targets] *$serial*]
if {$tgt eq ""} { puts "ERROR: no JTAG target with serial $serial among: [get_hw_targets]"; exit 1 }
current_hw_target $tgt
open_hw_target
set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
set_property PROGRAM.FILE $bit $dev
program_hw_devices $dev
refresh_hw_device $dev
puts "PROGRAMMED $dev on $tgt with $bit"
close_hw_target
disconnect_hw_server
close_hw_manager
