# flash_qspi.tcl — write a board's QSPI configuration flash over its JTAG (PERSISTENT: the board boots this image after a
# power cycle; the PA-Starlite boots from QSPI by default, a JTAG load still overrides it any time).
#   vivado -mode batch -nojournal -nolog -source bringup/flash_qspi.tcl -tclargs <serial> <image.mcs> [cfgmem part]
# The .mcs comes from tools/write_bit_spi.tcl (SPI x4, 33 MHz, compressed). The cfgmem part names Vivado's programming
# algorithm for the 128 Mb QSPI chip of the board. Board B's chip reads JEDEC ID ef/40/18 = Winbond W25Q128 (JV/FV), so
# the default w25q128jvq-spi-x1_x2_x4 is right (verified on board B by erase + program + verify).
# Only the *-spi-x1_x2_x4 names work on Artix-7 (the cfgmem-* generics are UltraScale+ only).
# Flashing loads Vivado's SPI bridge into the FPGA: the running design is GONE until the board is power-cycled (boots the
# flash) or re-programmed over JTAG (bringup/program.tcl). Erase + program + verify take a few minutes.
set serial [lindex $argv 0]
set mcs    [lindex $argv 1]
set part   [expr {[llength $argv] > 2 ? [lindex $argv 2] : "w25q128jvq-spi-x1_x2_x4"}]   ;# Artix-7 parts are the *-spi-x1_x2_x4 names (the cfgmem-* generics are UltraScale+)
if {![file exists $mcs]} { puts "ERROR: no $mcs"; exit 1 }
open_hw_manager
connect_hw_server -url localhost:3121
set tgt ""
foreach t [get_hw_targets] { if {[string match "*$serial*" $t]} { set tgt $t } }
if {$tgt eq ""} { puts "ERROR: no JTAG target with serial $serial among: [get_hw_targets]"; exit 1 }
open_hw_target $tgt
set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
refresh_hw_device -update_hw_probes false $dev
set parts [get_cfgmem_parts $part]
if {[llength $parts] == 0} { puts "ERROR: unknown cfgmem part $part; candidates: [get_cfgmem_parts *128*]"; exit 1 }
set cfg [create_hw_cfgmem -hw_device $dev [lindex $parts 0]]
set_property PROGRAM.ADDRESS_RANGE {use_file} $cfg
set_property PROGRAM.FILES [list $mcs] $cfg
set_property PROGRAM.PRM_FILE {} $cfg
set_property PROGRAM.UNUSED_PIN_TERMINATION {pull-none} $cfg
set_property PROGRAM.BLANK_CHECK 0 $cfg
set_property PROGRAM.ERASE 1 $cfg
set_property PROGRAM.CFG_PROGRAM 1 $cfg
set_property PROGRAM.VERIFY 1 $cfg
set_property PROGRAM.CHECKSUM 0 $cfg
puts "FLASH: loading the SPI bridge into $dev on $tgt"
create_hw_bitstream -hw_device $dev [get_property PROGRAM.HW_CFGMEM_BITFILE $dev]
program_hw_devices $dev
refresh_hw_device $dev
puts "FLASH: erase + program + verify of $mcs ($part)"
program_hw_cfgmem -hw_cfgmem $cfg
puts "FLASH_DONE $mcs on $tgt ($part) — power-cycle the board to boot it, or re-program over JTAG to continue now"
close_hw_target
