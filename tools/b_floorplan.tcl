# Board B floorplan: a Pblock around tmu24's A stage (the burst buffer's one-cycle decision loop).
#
# Why: the loop hb_q -> the 64 tag compares -> a_one / a_fast -> a_push -> the next-head
# mux is 12 logic levels; its timing is set by the route (about 6.7 of its 9.0 ns). Where the placer keeps the stage
# together (hb_q, bt_v / bt_ref / pend and al_e within 30 slice columns) it nearly meets timing; where it pulls the
# allocation side 20-30 columns away from the head registers it misses by about 0.8 ns. The Pblock keeps the stage's
# registers and their combinational fan-in cone in one rectangle; other logic may still be placed inside it.
#
# Sourced by tools/build_board.tcl for board B after opt_design, before place_design (cell names are final then).
# B_PBLOCK=0 turns it off; B_PBLOCK_RECT overrides the rectangle (e.g. "SLICE_X100Y108:SLICE_X131Y135").

proc b_astage_cells {} {
    set T u_core/u_render/u_tmu
    # the stage's state: the head job, the tag CAM, the reference counts / pending bits, the allocation, the FIFO counts
    set pat {hb_q heq_q hinc_q bt_tag bt_v bd_v bt_ref bt_ap bwt_v bwt_e bwt_tag al_e alc_v pend jf_full jf_r lq_room lq_n
             ta_n ta_h ta_eq ta_inc}
    set regs {}
    foreach p $pat { foreach c [get_cells -quiet $T/${p}_reg*] { lappend regs $c } }
    set regs [get_cells -quiet $regs]
    # their combinational fan-in cone (all_fanin stops at the startpoints: registers, RAM outputs, ports), only tmu24's
    # own logic outside the cache
    set dpins [get_pins -quiet -of_objects $regs -filter {DIRECTION == IN && !IS_CLOCK}]
    set cone [get_cells -quiet [all_fanin -quiet -flat -only_cells $dpins]]
    set cone [filter -quiet $cone "(PRIMITIVE_GROUP == LUT || PRIMITIVE_GROUP == CARRY || PRIMITIVE_GROUP == MUXFX) && NAME =~ $T/* && NAME !~ $T/U_CACHE/*"]
    return [list $regs $cone]
}

proc b_floorplan {} {
    if {[info exists ::env(B_PBLOCK)] && $::env(B_PBLOCK) eq "0"} { puts "=== B floorplan: off (B_PBLOCK=0)"; return }
    # the default: the area where a placement that nearly met timing put 90 % of the stage's cells by itself
    # (SLICE_X104Y116:X135Y139), two rows taller. The stage is ≈ 880
    # flops, 1 760 LUT, 256 CARRY4 (one per slice) = 450–650 slices of the 832
    set rect [expr {[info exists ::env(B_PBLOCK_RECT)] ? $::env(B_PBLOCK_RECT) : "SLICE_X104Y114:SLICE_X135Y139"}]
    lassign [b_astage_cells] regs cone
    set nlut [llength [filter -quiet $cone {PRIMITIVE_GROUP == LUT}]]
    set ncy  [llength [filter -quiet $cone {PRIMITIVE_GROUP == CARRY}]]
    set nff  [llength $regs]
    set nsl  [llength [get_sites -quiet -range $rect]]
    puts [format "=== B floorplan: pb_tmu_a %s (%d slices) <- %d flops + %d LUT + %d CARRY4 of tmu24's A stage" $rect $nsl $nff $nlut $ncy]
    if {$nff < 500 || $nlut < 300} { puts "ERROR: B floorplan: the A stage's cells were not found (renamed?)"; exit 1 }
    if {[llength [get_pblocks -quiet pb_tmu_a]]} { delete_pblocks pb_tmu_a }
    create_pblock pb_tmu_a
    resize_pblock pb_tmu_a -add $rect
    add_cells_to_pblock pb_tmu_a [get_cells [concat $regs $cone]]
}
