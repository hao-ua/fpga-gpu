// vtx_asm — primitive assembly of board A's vertex front end (a_vertex.sv). Takes the feed's
//   reference stream (one {slot, lane} per index, in index order) through a FIFO and forms primitives by the draw's mode
//   (ES 2.0 §2.6.1: 0 TRIANGLES; 1 TRIANGLE_STRIP with the first two vertices of every odd triangle swapped; 2
//   TRIANGLE_FAN; 3 POINTS; 4 LINES; 5 LINE_STRIP; 6 LINE_LOOP = the strip plus the closing segment last -> first;
//   an incomplete trailing primitive is dropped). For each vertex of a primitive it waits until the vertex's result slot
//   is complete (done), reads gl_Position, the fragment shader's varyings (varmask) and, for points, the gl_PointSize
//   row from vtx_rbuf, one vec4 per cycle, into a staging vertex, and presents it on the setup stage's interface
//   (vo_valid / vo_ready, vo_pos, vo_var, vo_psize; varyings outside varmask are 0; vo_first marks a primitive's first
//   vertex, vo_prim its kind: 0 triangle, 1 line, 2 point). A reference is released (ref_dec: the slot's count) when it
//   leaves the assembly window, so every pushed reference is released exactly once (a loop keeps its first vertex until
//   the closing segment). DRAWS QUEUE: start pushes {mode, count, varmask} into a small
//   draw FIFO, the assembler takes the next draw when the current one is over; draw_done pulses once the draw's last
//   vertex has been taken by the setup stage (the per-draw state of a_setup advances then).
//
//   THREE STAGES, so work on the next primitive runs beside the current one's:
//     the FORMER takes references (one per cycle) and forms the next primitive F while the reader is busy;
//     POSITIONS AHEAD: a triangle's three gl_Position rows are read as soon as it is formed — while the primitive before
//       it is still being presented — and handed to the setup stage (pa_req / pa_grant, then three pa_valid beats), which
//       starts their window transform at once: the 26 cycles of vtx_setup2 pass while the setup still works on the
//       primitive before. vo_la tells the setup stage that a presented triangle's positions went ahead;
//     the READER presents the primitive V vertex by vertex, but a vertex's rows are read back to back with the
//       previous vertex's while at most one earlier vertex is still on its way to the setup stage (one fits the output
//       register, one the staging vertex), and the primitive's references are released by a small queue of their own.
//   The draw's end waits for all of it: the window's last references are released only when no read can need them.
`default_nettype none
module vtx_asm #(
    parameter int W = 24,
    parameter int NL = 16,                       // lanes of the vertex core's warp (a reference's lane: 16, 8 or 4)
    parameter int NSLOT = 8,
    parameter int NVEC = 8,
    parameter int FD = 32,                       // reference FIFO depth (power of 2)
    parameter int DQ = 4                         // draw FIFO depth (power of 2)
)(
    input  wire        clk, rst,
    // draws (a FIFO: start pushes while !dq_full)
    input  wire        start,
    input  wire [2:0]  mode,                     // 0 triangles, 1 strip, 2 fan, 3 points, 4 lines, 5 line strip, 6 line loop
    input  wire [31:0] count,                    // indices of the draw (= references to consume; ES 2.0 sets no limit)
    input  wire [7:0]  varmask,
    output wire        dq_full,
    output reg         draw_done,                // a draw's last vertex has been taken (one pulse per draw, in order)
    output wire        busy,
    // references from the feed
    input  wire        ref_valid,
    input  wire [$clog2(NSLOT)-1:0] ref_slot,
    input  wire [((NL > 1) ? $clog2(NL) : 1)-1:0] ref_lane,
    output wire        ref_ready,
    output wire [5:0]  ref_cnt,
    // result buffer
    input  wire [NSLOT-1:0] done,
    output reg  [$clog2(NSLOT)-1:0] rd_slot,
    output reg  [3:0]  rd_vidx,
    output reg  [((NL > 1) ? $clog2(NL) : 1)-1:0] rd_lane,
    input  wire [4*W-1:0] rd_data,               // valid the cycle after rd_*
    output reg         ref_dec,
    output reg  [$clog2(NSLOT)-1:0] ref_dec_slot,
    // vertices to the setup stage
    output reg         vo_valid,
    input  wire        vo_ready,
    output reg  [4*W-1:0] vo_pos,
    output reg  [NVEC*4*W-1:0] vo_var,
    output reg  [W-1:0] vo_psize,                // gl_PointSize (points)
    output reg  [1:0]  vo_prim,                  // 0 triangle, 1 line, 2 point
    output reg         vo_first,                 // the primitive's first vertex
    output wire        vo_la,                    // the presented triangle's positions went ahead on pa_*
    // positions ahead (triangles): pa_req while a formed triangle's positions wait (pa_req_first: it is the draw's
    // first triangle), pa_grant when the setup stage can take the three; then three pa_valid beats (pa_first with the
    // first beat of a draw's first triangle), which the setup stage takes unconditionally
    output wire        pa_req,
    output wire        pa_req_first,
    input  wire        pa_grant,
    output reg         pa_valid,
    output reg  [4*W-1:0] pa_pos,
    output reg         pa_first
);
    localparam int LNW = (NL > 1) ? $clog2(NL) : 1;
    localparam int SW = $clog2(NSLOT), RW = SW + LNW, FW = $clog2(FD), DW = $clog2(DQ);   // a reference {slot, lane}
    // ---- draw FIFO: {varmask, count, mode}
    reg [42:0] dq [0:DQ-1]; reg [DW-1:0] d_wp, d_rp; reg [DW:0] d_cnt;
    assign dq_full = (d_cnt == (DW+1)'(DQ));
    wire d_v = (d_cnt != '0); reg d_pop;
    wire [42:0] d_head = dq[d_rp];
    wire [7:0] varmask_q = d_head[42:35];         // the current draw's (the head stays until the draw is over)
    always @(posedge clk) begin
        if (rst) begin d_wp <= '0; d_rp <= '0; d_cnt <= '0; end
        else begin
            if (start) begin dq[d_wp] <= {varmask, count, mode}; d_wp <= d_wp + 1'b1; end
            if (d_pop) d_rp <= d_rp + 1'b1;
            d_cnt <= d_cnt + (DW+1)'(start) - (DW+1)'(d_pop);
        end
    end
    // ---- reference FIFO (LUTRAM); the former pops the head in the cycle it takes it
    (* ram_style="distributed" *) reg [RW-1:0] fq [0:FD-1];
    reg [FW-1:0] f_wp, f_rp; reg [FW:0] f_cnt;
    wire f_push = ref_valid & ref_ready; wire f_pop;
    assign ref_ready = (f_cnt != (FW+1)'(FD));
    assign ref_cnt = 6'(f_cnt);
    wire [RW-1:0] f_head = fq[f_rp];
    always @(posedge clk) begin
        if (rst) begin f_wp <= '0; f_rp <= '0; f_cnt <= '0; end
        else begin
            if (f_push) begin fq[f_wp] <= {ref_slot, ref_lane}; f_wp <= f_wp + 1'b1; end
            if (f_pop) f_rp <= f_rp + 1'b1;
            f_cnt <= f_cnt + (FW+1)'(f_push) - (FW+1)'(f_pop);
        end
    end
    // ---- the former's window and the draw
    reg [RW-1:0] ha, hb, hf;                     // held references: triangles = the first two of the group; strip = the
                                                 // two previous; fan = the first and the previous; lines = the previous
                                                 // (ha), a loop's first (hf)
    reg [31:0] n, consumed, count_q;             // references taken into the window / consumed from the FIFO; the draw's count
    reg [1:0]  tn;                               // triangles / lines: references in the current group
    reg [2:0]  md;                               // the current draw's mode
    reg        d1st;                             // no primitive of the draw has been formed yet
    wire       tri_md = (md <= 3'd2);
    wire [1:0] psz_last = tri_md ? 2'd2 : (md == 3'd3) ? 2'd0 : 2'd1;   // the primitive's last vertex index
    wire [1:0] prim = tri_md ? 2'd0 : (md == 3'd3) ? 2'd2 : 2'd1;
    typedef enum logic [2:0] {F_IDLE, F_RUN, F_FLW, F_LOOPW, F_END} fst_t;
    fst_t fst;
    wire [2:0] st = fst;                         // (the name the benches' progress prints read)
    assign busy = (fst != F_IDLE) | d_v;
    // ---- F: the formed primitive, waiting for the reader (a triangle: its positions go ahead meanwhile, f_pd)
    reg        f_v, f_pd, f_first; reg [RW-1:0] f_a, f_b, f_c, f_r0, f_r1, f_r2; reg [1:0] f_rn;
    // ---- V: the primitive being presented; its references are released (pr_*) when its last row has been issued
    reg        v_v; reg [RW-1:0] wa, wb, wc, rel0, rel1, rel2; reg [1:0] rel_n;
    reg [1:0]  vi;                               // the vertex being read
    reg [9:0]  rrem; reg vr_act;                 // vec4s still to issue for it (bit 9 point size, 8 position, 7..0 varyings); issuing
    reg [1:0]  pr_n; reg [RW-1:0] pr0, pr1, pr2; // releases still to issue
    reg [1:0]  vout;                             // vertices issued and not yet taken by the setup stage (<= 2: output + staging)
    wire [RW-1:0] cur = (vi == 2'd0) ? wa : (vi == 2'd1) ? wb : wc;
    wire [9:0] rr = vr_act ? rrem : {md == 3'd3, 1'b1, varmask_q};
    reg  [3:0] ritem;
    always @* begin ritem = 4'd9; for (int k = 7; k >= 0; k--) if (rr[k]) ritem = 4'(k); if (rr[8]) ritem = 4'd8; end
    wire rlast = ((rr & (rr - 10'd1)) == 10'd0);
    // ---- positions ahead: three reads of F's gl_Position rows once the setup stage grants them
    reg        p_run; reg [1:0] p_i;
    wire [RW-1:0] p_ref = (p_i == 2'd0) ? f_a : (p_i == 2'd1) ? f_b : f_c;
    assign pa_req = f_v & ~f_pd & ~p_run;
    assign pa_req_first = f_first;
    // ---- the read pipeline: address out (rd_*), the buffer's row register, the lane register; what each read is for
    reg rd_v, rd_v2, rd_p, rd_p2, rd_l, rd_l2, rd_pf, rd_pf2, rd_f, rd_f2;   // valid; a position ahead; a vertex's last row; pa_first; vo_first
    reg [3:0] rd_item_q, rd_item_q2;
    // the staging vertex (the output holds the previous one meanwhile) and the output data, without a reset term (timing
    // at 120 MHz): the varyings outside the draw's varmask are cleared once, at the draw's start (sv_clr)
    reg [4*W-1:0] sp, sps; reg [NVEC*4*W-1:0] sv; reg sg_v, sg_first;
    (* max_fanout = 64 *) reg sv_clr;
    wire v_land = rd_v2 & ~rd_p2;
    wire sg_copy = sg_v & ~vo_valid;                                  // the complete staging vertex moves to the output
    always @(posedge clk) begin
        if (sv_clr) sv <= '0;
        else if (v_land && rd_item_q2 != 4'd8 && rd_item_q2 != 4'd9) sv[rd_item_q2*4*W +: 4*W] <= rd_data;   // the read issued two cycles ago lands
        if (v_land && rd_item_q2 == 4'd8) sp <= rd_data;
        if (v_land && rd_item_q2 == 4'd9) sps <= rd_data;
        if (v_land && rd_l2) sg_first <= rd_f2;
        if (sg_copy) begin vo_pos <= sp; vo_var <= sv; vo_psize <= sps[W-1:0]; vo_prim <= prim; vo_first <= sg_first; end
        if (rd_v2 && rd_p2) pa_pos <= rd_data;
    end
    assign vo_la = (vo_prim == 2'd0);
    // the reader's decisions (one read per cycle; the positions ahead first)
    wire p_iss = p_run && done[p_ref[RW-1 -: SW]];
    // a vertex's first row only when its slot is complete and at most one earlier vertex is still on its way; the
    // primitive's last row only when the release queue is free for its references
    wire v_iss = v_v && !p_run && (vr_act || (done[cur[RW-1 -: SW]] && vout <= 2'd1)) && !(rlast && vi == psz_last && pr_n != 2'd0);
    wire v_take = vo_valid & vo_ready;
    wire rd_idle = !rd_v && !rd_v2;
    wire pipe_empty = !f_v && !v_v && rd_idle && (pr_n == 2'd0);     // no read can need a reference any more
    // the former takes a reference whenever F is free
    assign f_pop = (fst == F_RUN) && !f_v && (consumed != count_q) && (f_cnt != '0);
    always @(posedge clk) begin
        if (rst) begin
            fst <= F_IDLE; vo_valid <= 1'b0; ref_dec <= 1'b0; rd_v <= 1'b0; rd_v2 <= 1'b0; d_pop <= 1'b0; draw_done <= 1'b0; sv_clr <= 1'b0;
            f_v <= 1'b0; v_v <= 1'b0; vr_act <= 1'b0; pr_n <= 2'd0; vout <= 2'd0; p_run <= 1'b0; pa_valid <= 1'b0; sg_v <= 1'b0;
            rd_p <= 1'b0; rd_p2 <= 1'b0; rd_l <= 1'b0; rd_l2 <= 1'b0;
        end else begin
            ref_dec <= 1'b0; rd_v <= 1'b0; d_pop <= 1'b0; draw_done <= 1'b0; sv_clr <= 1'b0; pa_valid <= 1'b0;
            rd_v2 <= rd_v; rd_p2 <= rd_p; rd_l2 <= rd_l; rd_pf2 <= rd_pf; rd_f2 <= rd_f; rd_item_q2 <= rd_item_q;
            // ---------------- the output and the staging vertex
            if (v_take) vo_valid <= 1'b0;
            if (sg_copy) vo_valid <= 1'b1;
            if (v_land && rd_l2) sg_v <= 1'b1;                       // (a vertex completing as the one before it moves on: it stays)
            else if (sg_copy) sg_v <= 1'b0;
            vout <= vout + 2'(v_iss && !vr_act) - 2'(v_take);
            if (rd_v2 && rd_p2) begin pa_valid <= 1'b1; pa_first <= rd_pf2; end
            // ---------------- releases, one per cycle
            if (pr_n != 2'd0) begin
                ref_dec <= 1'b1;
                ref_dec_slot <= (pr_n == 2'd3) ? pr2[RW-1 -: SW] : (pr_n == 2'd2) ? pr1[RW-1 -: SW] : pr0[RW-1 -: SW];
                pr_n <= pr_n - 2'd1;
            end
            // ---------------- the reader: the positions ahead of F, else the next row of V's vertex
            if (pa_req && pa_grant) begin p_run <= 1'b1; p_i <= 2'd0; end
            if (p_iss) begin
                rd_slot <= p_ref[RW-1 -: SW]; rd_lane <= p_ref[LNW-1:0]; rd_vidx <= 4'd8; rd_v <= 1'b1; rd_p <= 1'b1; rd_l <= 1'b0;
                rd_pf <= f_first && (p_i == 2'd0); rd_item_q <= 4'd8;
                p_i <= p_i + 2'd1;
                if (p_i == 2'd2) begin p_run <= 1'b0; f_pd <= 1'b1; end
            end else if (v_iss) begin
                rd_slot <= cur[RW-1 -: SW]; rd_lane <= cur[LNW-1:0]; rd_vidx <= ritem; rd_v <= 1'b1; rd_p <= 1'b0; rd_l <= rlast;
                rd_f <= (vi == 2'd0); rd_item_q <= ritem;
                rrem <= rr & ~(10'd1 << ritem); vr_act <= ~rlast;
                if (rlast) begin
                    if (vi == psz_last) begin v_v <= 1'b0; pr0 <= rel0; pr1 <= rel1; pr2 <= rel2; pr_n <= rel_n; end   // (pr_n is 0 here: v_iss)
                    else vi <= vi + 2'd1;
                end
            end
            // ---------------- F -> V when the reader is free of the primitive before (a triangle: once its positions are on their way)
            if (f_v && f_pd && (!v_v || (v_iss && !p_iss && rlast && vi == psz_last))) begin
                v_v <= 1'b1; wa <= f_a; wb <= f_b; wc <= f_c; rel0 <= f_r0; rel1 <= f_r1; rel2 <= f_r2; rel_n <= f_rn; vi <= 2'd0;
                f_v <= 1'b0;
            end
            // ---------------- the former
            case (fst)
                F_IDLE: if (d_v && !d_pop) begin                         // (!d_pop: the finished draw's pop lands this cycle)
                    md <= d_head[2:0]; count_q <= d_head[34:3]; n <= 32'd0; tn <= 2'd0; consumed <= 32'd0; d1st <= 1'b1;
                    sv_clr <= 1'b1; fst <= F_RUN;
                end
                F_RUN: begin
                    if (!f_v && consumed == count_q) fst <= F_FLW;
                    else if (f_pop) begin : TAKE                         // f_head = the reference taken this cycle
                        reg go; reg [RW-1:0] a, b, c, r0, r1, r2; reg [1:0] rn;
                        go = 1'b0; a = f_head; b = f_head; c = f_head; r0 = f_head; r1 = f_head; r2 = f_head; rn = 2'd0;
                        consumed <= consumed + 1'b1; n <= n + 1'b1;
                        case (md)
                            3'd1: begin                                 // strip: triangle i = n - 2; odd i swaps the first two
                                if (n >= 32'd2) begin
                                    if (n[0]) begin a = hb; b = ha; end else begin a = ha; b = hb; end
                                    c = f_head; r0 = ha; rn = 2'd1; go = 1'b1; ha <= hb; hb <= f_head;
                                end else begin if (n == 32'd0) ha <= f_head; else hb <= f_head; end
                            end
                            3'd2: begin                                 // fan: (first, previous, this)
                                if (n >= 32'd2) begin a = ha; b = hb; c = f_head; r0 = hb; rn = 2'd1; go = 1'b1; hb <= f_head; end
                                else begin if (n == 32'd0) ha <= f_head; else hb <= f_head; end
                            end
                            3'd3: begin a = f_head; r0 = f_head; rn = 2'd1; go = 1'b1; end   // points: one reference each
                            3'd4: begin                                 // lines: pairs
                                if (tn == 2'd1) begin a = ha; b = f_head; r0 = ha; r1 = f_head; rn = 2'd2; go = 1'b1; tn <= 2'd0; end
                                else begin ha <= f_head; tn <= 2'd1; end
                            end
                            3'd5, 3'd6: begin                           // line strip / loop: (previous, this); the loop keeps its first
                                if (n >= 32'd1) begin
                                    a = ha; b = f_head; r0 = ha; rn = (md == 3'd6 && n == 32'd1) ? 2'd0 : 2'd1; go = 1'b1; ha <= f_head;
                                end else begin ha <= f_head; hf <= f_head; end
                            end
                            default: begin                              // triangles: groups of three
                                if (tn == 2'd2) begin a = ha; b = hb; c = f_head; r0 = ha; r1 = hb; r2 = f_head; rn = 2'd3; go = 1'b1; tn <= 2'd0; end
                                else begin if (tn == 2'd0) ha <= f_head; else hb <= f_head; tn <= tn + 1'b1; end
                            end
                        endcase
                        if (go) begin
                            f_v <= 1'b1; f_pd <= ~tri_md; f_first <= d1st; d1st <= 1'b0;
                            f_a <= a; f_b <= b; f_c <= c; f_r0 <= r0; f_r1 <= r1; f_r2 <= r2; f_rn <= rn;
                        end
                    end
                end
                // the draw's references are consumed: once nothing can read them any more, release what the window still
                // holds (a loop: its closing segment (last, first) first, which then releases both)
                F_FLW: if (pipe_empty) begin
                    fst <= F_END;
                    case (md)
                        3'd0: begin
                            if (tn == 2'd2) begin pr0 <= ha; pr1 <= hb; pr_n <= 2'd2; end
                            else if (tn == 2'd1) begin pr0 <= ha; pr_n <= 2'd1; end
                        end
                        3'd1, 3'd2: begin
                            if (n >= 32'd2) begin pr0 <= ha; pr1 <= hb; pr_n <= 2'd2; end
                            else if (n == 32'd1) begin pr0 <= ha; pr_n <= 2'd1; end
                        end
                        3'd3: ;
                        3'd4: if (tn == 2'd1) begin pr0 <= ha; pr_n <= 2'd1; end
                        3'd5: if (n >= 32'd1) begin pr0 <= ha; pr_n <= 2'd1; end
                        default: begin                                  // loop
                            if (n >= 32'd2) begin
                                f_v <= 1'b1; f_pd <= 1'b1; f_first <= 1'b0; f_a <= ha; f_b <= hf; f_c <= hf; f_r0 <= ha; f_r1 <= hf; f_r2 <= hf; f_rn <= 2'd2;
                                fst <= F_LOOPW;
                            end else if (n == 32'd1) begin pr0 <= hf; pr_n <= 2'd1; end
                        end
                    endcase
                end
                F_LOOPW: if (pipe_empty) fst <= F_END;                   // the closing segment is through the reader
                // the draw is over when its last vertex has been taken and its last reference released
                F_END: if (pipe_empty && !sg_v && !vo_valid && !pa_valid) begin draw_done <= 1'b1; d_pop <= 1'b1; fst <= F_IDLE; end
                default: fst <= F_IDLE;
            endcase
        end
    end
    // synthesis translate_off
    always @(posedge clk) if (!rst) begin
        if (vout == 2'd3) $fatal(1, "vtx_asm: three vertices on their way to the setup stage (output + staging hold two)");
        if (v_land && sg_v && !sg_copy) $fatal(1, "vtx_asm: a row landed in the staging vertex before the complete vertex in it moved on");
        if (v_iss && rlast && vi == psz_last && pr_n != 2'd0) $fatal(1, "vtx_asm: a primitive's releases over the previous one's");
    end
    // synthesis translate_on
endmodule
`default_nettype wire
