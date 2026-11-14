# gen_mig.tcl — (re)create the DDR3 MIG IP locally from ip/mig_7series_0/mig_a.prj and synthesize it
# out of context.  Run from the repo root:
#   vivado -mode batch -source tools/gen_mig.tcl
# Output: tools/gen/ip/mig_7series_0/{mig_7series_0.xci, mig_7series_0.dcp, sim + stub files}
# The stored ip/mig_7series_0/mig_7series_0.xci points its output directory at another project's
# path, so the board builds use this freshly generated copy instead. Same mig_a.prj
# (DDR3L 32-bit, 400 MHz, 4:1, ROW_BANK_COLUMN, strict ordering, pins of the PA200T-Starlite).
set part xc7a200tfbg484-2
set_part $part
set ipdir tools/gen/ip
file mkdir $ipdir
if {[file exists $ipdir/mig_7series_0]} { file delete -force $ipdir/mig_7series_0 }
create_ip -name mig_7series -vendor xilinx.com -library ip -module_name mig_7series_0 -dir $ipdir
file copy -force ip/mig_7series_0/mig_a.prj $ipdir/mig_7series_0/mig_a.prj
set_property -dict [list CONFIG.XML_INPUT_FILE {mig_a.prj} CONFIG.RESET_BOARD_INTERFACE {Custom} \
    CONFIG.MIG_DONT_TOUCH_PARAM {Custom} CONFIG.BOARD_MIG_PARAM {Custom}] [get_ips mig_7series_0]
generate_target all [get_ips mig_7series_0]
synth_ip [get_ips mig_7series_0]
puts "MIG_DCP [get_property IP_OUTPUT_DIR [get_ips mig_7series_0]]"
puts "GEN_MIG_DONE"
