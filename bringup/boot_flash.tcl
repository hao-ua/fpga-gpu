# boot_flash.tcl — make one board reconfigure itself from its QSPI flash over JTAG (JPROGRAM pulse, no power cycle):
# the way back to the flashed image after a volatile test load (bringup/program.tcl).
#   vivado -mode batch -nojournal -source bringup/boot_flash.tcl -tclargs <serial>
# <serial>: the board's JTAG target serial (bringup/boards.local.tcl, not in git).
if {[llength $argv] != 1} { puts "usage: -tclargs <target-serial>"; exit 1 }
set serial [lindex $argv 0]
open_hw_manager
connect_hw_server -allow_non_jtag
set tgt [lsearch -inline [get_hw_targets] *$serial*]
if {$tgt eq ""} { puts "ERROR: no JTAG target with serial $serial among: [get_hw_targets]"; exit 1 }
current_hw_target $tgt
open_hw_target
set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
boot_hw_device $dev
after 3000
refresh_hw_device -quiet $dev
set done "?"
catch { set done [get_property REGISTER.IR.BIT5_DONE $dev] }
puts "BOOTED $dev on $tgt from flash: DONE=$done"
close_hw_target
disconnect_hw_server
close_hw_manager
