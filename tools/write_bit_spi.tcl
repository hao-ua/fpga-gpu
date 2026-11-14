# write_bit_spi.tcl — regenerate a build's bitstream for QSPI boot from its routed checkpoint (no re-route):
#   vivado -mode batch -nojournal -nolog -source tools/write_bit_spi.tcl -tclargs tools/gen/board_B_b55_routed.dcp tools/gen/board_B_b55_spi [16]
# -> <out>.bit (SPI x4 bus, 33 MHz configuration clock, compressed: the 77 Mbit XC7A200T image boots in well under a second
#    instead of ~25 s at the x1 / 3 MHz defaults the build leaves) and <out>.mcs for the QSPI flash of <size> MB (default 16 =
#    128 Mbit). The design is unchanged (bitstream options only); the routed timing of the build holds.
set dcp  [lindex $argv 0]
set out  [lindex $argv 1]
set size [expr {[llength $argv] > 2 ? [lindex $argv 2] : 16}]
open_checkpoint $dcp
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE YES [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
write_bitstream -force ${out}.bit
write_cfgmem -force -format mcs -size $size -interface SPIx4 -loadbit "up 0x0 ${out}.bit" ${out}.mcs
puts "SPI_BIT_DONE ${out}.bit ${out}.mcs"
