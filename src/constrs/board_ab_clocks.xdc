# board_ab_clocks.xdc — the one-board design's video and JTAG clocks, read after synthesis (tools/build_board.tcl S).
# The video MMCM (board B's u_disp/u_vm/u_mmcm) is fed by the BUFG'd 200 MHz clock, so Vivado derives its outputs from
# sys_clk; they are renamed and declared asynchronous to the MIG clocks, as in board_b_video.xdc (whose comment lists the
# crossings). JTAG: board A's status register (USER1, DRCK); board B's JTAG registers are off on this board.
create_generated_clock -name vid_ser [get_pins -hierarchical -filter {NAME =~ *u_vm/u_mmcm/CLKOUT0}]
create_generated_clock -name vid_pix [get_pins -hierarchical -filter {NAME =~ *u_vm/u_mmcm/CLKOUT1}]
create_generated_clock -name vid_fb  [get_pins -hierarchical -filter {NAME =~ *u_vm/u_mmcm/CLKFBOUT}]
set_clock_groups -asynchronous \
    -group [get_clocks {vid_ser vid_pix vid_fb}] \
    -group [get_clocks -include_generated_clocks sys_clk -filter {NAME !~ vid_*}]

create_clock -name jtag_drck -period 33.333 [get_pins -hierarchical -filter {NAME =~ *u_jtag/u_bscan/DRCK}]
set_clock_groups -asynchronous -group [get_clocks jtag_drck] -group [get_clocks -include_generated_clocks sys_clk] \
    -group [get_clocks eth_rxc]
