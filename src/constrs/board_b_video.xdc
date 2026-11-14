# board_b_video.xdc — board B video and JTAG clocks, read after synthesis (tools/build_board.tcl B).
# The video MMCM (u_core/u_disp/u_vm/u_mmcm) is fed by the BUFG'd 200 MHz clock, so Vivado derives its
# outputs from sys_clk; they are renamed here and declared asynchronous to the MIG clocks. The static MMCM
# attributes are the fastest mode (1280x720: 371.25 MHz serial, 74.25 MHz pixel), so these derived
# clocks are the worst case of the four run-time modes.
# Crossings: line-buffer slot toggles (2-FF, ASYNC_REG), frame-sync / underrun toggles, gray-coded pixel
# counter, pixel-domain reset (rst_sync). The video timing registers change only while the pixel domain
# is held in reset. Palette and cursor: two dual-clock block RAMs (written in clk_sys, read in
# the pixel clock; a palette bank / shape bank is never written while displayed), and the cursor mailbox: a
# req / ack toggle pair (2-FF, ASYNC_REG) and a 48-bit state that is stable while req != ack.
create_generated_clock -name vid_ser [get_pins -hierarchical -filter {NAME =~ *u_vm/u_mmcm/CLKOUT0}]
create_generated_clock -name vid_pix [get_pins -hierarchical -filter {NAME =~ *u_vm/u_mmcm/CLKOUT1}]
create_generated_clock -name vid_fb  [get_pins -hierarchical -filter {NAME =~ *u_vm/u_mmcm/CLKFBOUT}]
set_clock_groups -asynchronous \
    -group [get_clocks {vid_ser vid_pix vid_fb}] \
    -group [get_clocks -include_generated_clocks sys_clk -filter {NAME !~ vid_*}]

# JTAG: status register (USER1, DRCK) and command register (USER2, TCK), asynchronous to everything
create_clock -name jtag_drck -period 33.333 [get_pins -hierarchical -filter {NAME =~ *u_jtag/u_bscan/DRCK}]
create_clock -name jtag_tck  -period 33.333 [get_pins -hierarchical -filter {NAME =~ *u_jctl/u_bscan/TCK}]
set_clock_groups -asynchronous -group [get_clocks jtag_drck] -group [get_clocks jtag_tck] \
    -group [get_clocks -include_generated_clocks sys_clk]
