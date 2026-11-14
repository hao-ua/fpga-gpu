// vtx_rbuf — the vertex result buffer of board A's front end (a_vertex.sv). Finished warps leave
//   the vertex core in launch order; this block copies each one's rows — gl_Position and the varyings the fragment
//   shader reads (varmask) — out of the core's output store (out_sel, one row per cycle, the row registered) into a
//   LANE-BANKED store: per lane one block RAM of vec4s, addressed by {result slot, vec4 index} (index 0..7 = varying,
//   8 = gl_Position, 9 = the gl_PointSize row when ps_en: a POINTS draw), so a row of 16 lanes is written in one
//   cycle and the assembler reads any vertex's vec4 in one.
//   NSLOT result slots: the feed allocates them in order (alloc clears the slot's done flag), the launch order maps the
//   core's retiring warp to its slot (a small FIFO), done[slot] rises after the last row, and a reference count per
//   slot (one per reference the feed pushes, one less per reference the assembler releases) says when a slot is free
//   again (slot_free). Read port: {rd_slot, rd_vidx, rd_lane} -> rd_data (the vec4), valid the cycle after.
`default_nettype none
module vtx_rbuf #(
    parameter int W = 24,
    parameter int NSLOT = 8,
    parameter int LQ = 8                         // launched-slot FIFO depth (>= the core's warps in flight)
)(
    input  wire        clk, rst,
    // vertex core results
    input  wire        out_valid,
    output reg         out_ack,
    output reg  [6:0]  out_sel,
    input  wire [16*W-1:0] out_row,
    // slots
    input  wire        launch,                   // slot launch_slot was launched into the core, with its draw's copy set:
    input  wire [$clog2(NSLOT)-1:0] launch_slot,
    input  wire [7:0]  launch_varmask,           //   the varyings to copy (the fragment shader's read set) and
    input  wire        launch_ps,                //   whether to copy the gl_PointSize row too (vec4 index 9; a POINTS draw)
                                                 //   (per launch: draws overlap in the core)
    input  wire        alloc,                    // slot alloc_slot begins a new use
    input  wire [$clog2(NSLOT)-1:0] alloc_slot,
    input  wire        ref_inc,                  // a reference to slot ref_inc_slot was pushed
    input  wire [$clog2(NSLOT)-1:0] ref_inc_slot,
    input  wire        ref_dec,                  // a reference to slot ref_dec_slot was released
    input  wire [$clog2(NSLOT)-1:0] ref_dec_slot,
    output wire [NSLOT-1:0] slot_free,
    output reg  [NSLOT-1:0] done,
    output wire        lq_empty,
    // read port (the assembler)
    input  wire [$clog2(NSLOT)-1:0] rd_slot,
    input  wire [3:0]  rd_vidx,
    input  wire [3:0]  rd_lane,
    output wire [4*W-1:0] rd_data
);
    localparam int SW = $clog2(NSLOT), AW = SW + 4;
    // ---- launched-slot FIFO: {point size row, varmask, slot}
    localparam int LW = $clog2(LQ);
    reg [SW+8:0] lq [0:LQ-1]; reg [LW-1:0] lq_wp, lq_rp; reg [LW:0] lq_cnt;
    assign lq_empty = (lq_cnt == '0);
    // ---- reference counts, and the in-flight guard: a slot is free only when no reference is left AND its warp is not
    //      between its allocation and the end of its copy (the assembler releases a draw's trailing
    //      references unread, so a count can reach 0 while the warp still runs; re-allocating the slot then would clear
    //      its done flag, which the late copy sets again for the wrong use). Slot 0 is the forming slot after reset.
    reg [6:0] rc [0:NSLOT-1];
    reg [NSLOT-1:0] inflight;
    genvar gs;
    generate for (gs = 0; gs < NSLOT; gs = gs + 1) begin : RC
        always @(posedge clk)
            if (rst) rc[gs] <= 7'd0;
            else rc[gs] <= rc[gs] + 7'(ref_inc && ref_inc_slot == SW'(gs)) - 7'(ref_dec && ref_dec_slot == SW'(gs));
        assign slot_free[gs] = (rc[gs] == 7'd0) & ~inflight[gs];
    end endgenerate
    // ---- the copy: rows of the retiring warp into the banks. out_sel = {item, comp} this cycle; the core's row for it
    //      arrives next cycle (registered read) and is written then at the address registered alongside.
    reg        c_run; reg [SW-1:0] c_slot; reg [9:0] c_rem;    // bit 9 = point size, 8 = position, 7..0 = varyings still to copy
    reg [1:0]  c_comp;
    reg        w_v, w_last; reg [AW-1:0] w_addr; reg [1:0] w_comp; reg [SW-1:0] w_slot;
    reg [3:0]  c_item;                                          // the vec4 being read (8 = position first, 9 = point size last)
    always @* begin c_item = 4'd9; for (int k = 7; k >= 0; k--) if (c_rem[k]) c_item = 4'(k); if (c_rem[8]) c_item = 4'd8; end
    wire c_last_item = ((c_rem & (c_rem - 10'd1)) == 10'd0);
    wire c_last = c_run & c_last_item & (c_comp == 2'd3);      // the last row of the warp is being read
    always @* out_sel = {(c_item == 4'd8) ? 5'd16 : (c_item == 4'd9) ? 5'd17 : {1'b0, c_item}, c_comp};
    always @(posedge clk) begin
        if (rst) begin c_run <= 1'b0; w_v <= 1'b0; w_last <= 1'b0; out_ack <= 1'b0; lq_wp <= '0; lq_rp <= '0; lq_cnt <= '0; done <= '0; inflight <= NSLOT'(1); end
        else begin
            out_ack <= 1'b0;
            if (launch) begin lq[lq_wp] <= {launch_ps, launch_varmask, launch_slot}; lq_wp <= lq_wp + 1'b1; end
            if (alloc) begin done[alloc_slot] <= 1'b0; inflight[alloc_slot] <= 1'b1; end
            if (w_v && w_last) inflight[w_slot] <= 1'b0;
            w_v <= c_run; w_last <= c_last; w_addr <= {c_slot, c_item}; w_comp <= c_comp; w_slot <= c_slot;
            if (!c_run) begin
                if (out_valid && lq_cnt != '0 && !out_ack) begin
                    c_run <= 1'b1; c_slot <= lq[lq_rp][SW-1:0]; c_rem <= {lq[lq_rp][SW+8], 1'b1, lq[lq_rp][SW+7:SW]}; c_comp <= 2'd0;
                end
            end else begin
                c_comp <= c_comp + 1'b1;
                if (c_comp == 2'd3) begin
                    c_rem <= c_rem & ~(10'd1 << c_item);
                    if (c_last_item) begin c_run <= 1'b0; out_ack <= 1'b1; lq_rp <= lq_rp + 1'b1; end
                end
            end
            if (w_v && w_last) done[w_slot] <= 1'b1;             // the last row is written this cycle
            lq_cnt <= lq_cnt + (LW+1)'(launch) - (LW+1)'(c_last);
        end
    end
    // ---- the lane banks: per lane one block RAM of vec4s {slot, vidx}, a component written per cycle through its bytes'
    //      write enables (W / 8 bytes a component), the vec4 read whole
    if (W % 8 != 0) $error("vtx_rbuf: W must be a multiple of 8 (byte write enables)");
    localparam int CB = W / 8;
    reg [3:0] rl_q;
    always @(posedge clk) rl_q <= rd_lane;
    wire [4*W-1:0] lane_q [0:15];
    genvar gl;
    generate for (gl = 0; gl < 16; gl = gl + 1) begin : BL
        (* ram_style="block" *) reg [4*W-1:0] m [0:(1 << AW)-1];
        reg [4*W-1:0] q;
        wire [4*W-1:0] wd = {4{out_row[gl*W +: W]}};
        always @(posedge clk) begin
            for (int k = 0; k < 4 * CB; k++)
                if (w_v && w_comp == 2'(k / CB)) m[w_addr][k*8 +: 8] <= wd[k*8 +: 8];
            q <= m[{rd_slot, rd_vidx}];
        end
        assign lane_q[gl] = q;
    end endgenerate
    assign rd_data = lane_q[rl_q];
endmodule
`default_nettype wire
