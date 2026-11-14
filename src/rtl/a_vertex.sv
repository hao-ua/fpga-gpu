// a_vertex — board A's vertex stage: the vertex core shader_core_vs2 with its front end —
//   vtx_feed (index fetch, vertex cache, attribute gather into the core's lanes, warp launch, reference stream),
//   vtx_rbuf (the finished warps' rows copied into a lane-banked result buffer of NSLOT slots, reference counts) and
//   vtx_asm (TRIANGLES / STRIP / FAN from the reference stream, vertices read from the buffer and presented on
//   the setup stage's vertex interface, a_setup). One DDR read master (a mem_arb read port). Draw state the setup stage
//   needs (viewport, cull, varmask) is the caller's.
//   DRAW OVERLAP: draws do not wait for each other. dr_start is taken when dr_ok: the
//   feed's issue side is idle, the assembler's draw FIFO has room, no warp of the draw's PARITY (dr_bank, alternating per
//   draw) is still in the core (that parity's uniform / descriptor banks are free again: the draw before the previous one
//   has retired), and no copy runs. The UNIFORMS the host writes (param_*) go to a shadow (256 vec4 in block RAM, dirty
//   bits) and, at a draw's start, the entries changed since the bank was last filled (dirty since this draw ∪ dirty since
//   the previous one) are copied into the core's bank of the draw's parity (a few cycles; all 256 after reset, where the
//   shadow is zero: ES 2.0's initial uniform value), then the feed starts the draw. The program (iram) is loaded between
//   draws with the stage drained.
`default_nettype none
module a_vertex #(
    parameter int EXP_W = 7, MAN_W = 16,
    parameter int NWARP = 4,
    parameter int NL = 16,                      // the vertex core's lanes per warp = vertices per warp and result slot (16, 8, 4)
    parameter bit SFU_TAB2 = 1'b0,              // the vertex core's SFU tables shared by pairs of units (shader_core_vs2 SFU_TAB2)
    parameter bit INT_DIV = 1'b1,               // the vertex core's integer divider (shader_core_vs2 INT_DIV)
    parameter bit SINCOS = 1'b1,                // the vertex core's SIN / COS units (shader_core_vs2 SINCOS)
    parameter int NSLOT = 16,                    // result slots (>= the core's warps + the assembler's held references + the forming warp;
                                                 // <= 16); the buffer holds NSLOT x NL vertices
    parameter int NVEC = 8,
    parameter int NATTR = 16, NPARAM = 256, IRAM_DEPTH = 4096
)(
    input  wire        clk, rst,
    // program (between draws, the stage drained) / uniforms (the shadow: any time no copy runs)
    input  wire        iram_we,
    input  wire [$clog2(IRAM_DEPTH)-1:0] iram_waddr,
    input  wire [63:0] iram_wdata,
    input  wire        param_we,
    input  wire [$clog2(NPARAM)-1:0] param_idx,
    input  wire [4*(EXP_W+MAN_W+1)-1:0] param_data,
    // attribute descriptors (the shadow: any time no copy runs; vtx_feed: type codes of attr_conv, byte base / stride / offset)
    input  wire        ad_we,
    input  wire [3:0]  ad_idx,
    input  wire        ad_en,
    input  wire [2:0]  ad_type,
    input  wire        ad_norm,
    input  wire [2:0]  ad_size,
    input  wire [29:0] ad_base, ad_stride, ad_off,
    input  wire        ac_we,
    input  wire [3:0]  ac_idx,
    input  wire [4*(EXP_W+MAN_W+1)-1:0] ac_const,
    // draw (dr_start while dr_ok)
    input  wire        dr_start,
    input  wire [2:0]  dr_mode,                  // 0 triangles, 1 strip, 2 fan, 3 points, 4 lines, 5 line strip, 6 line loop
    input  wire        dr_indexed, dr_u8,
    input  wire [29:0] dr_idx_base,              // byte address of index 0
    input  wire [31:0] dr_first, dr_count,
    input  wire [7:0]  dr_varmask,               // the fragment shader's read set of this draw (copied out of the core, sent on)
    output wire        dr_ok,                    // a draw may start now
    output wire        copy_busy,                // a shadow is being copied: no uniform / descriptor write meanwhile
    output wire        draw_done,                // the assembler has presented a draw's last vertex (one pulse per draw, in order)
    output wire        dr_busy,                  // anything of any draw still in flight
    output wire        err_stack,                // the core's sticky stack-violation flag
    // DDR read master
    output wire        r_en,
    output wire [28:0] r_addr,
    input  wire        r_rdy, r_valid,
    input  wire [255:0] rd_data,
    // vertices to the setup stage (a_setup: 3 per triangle)
    output wire        vo_valid,
    input  wire        vo_ready,
    output wire [4*(EXP_W+MAN_W+1)-1:0] vo_pos,
    output wire [NVEC*4*(EXP_W+MAN_W+1)-1:0] vo_var,
    output wire [EXP_W+MAN_W:0] vo_psize,        // gl_PointSize (points)
    output wire [1:0]  vo_prim,                  // 0 triangle, 1 line, 2 point
    output wire        vo_first,
    // a triangle's gl_Position rows ahead of its vertices (vtx_asm -> a_setup); a bench without
    // a setup stage leaves pa_grant at its default (granted at once, the beats ignored)
    output wire        vo_la,
    output wire        pa_req, pa_req_first,
    input  wire        pa_grant = 1'b1,
    output wire        pa_valid,
    output wire [4*(EXP_W+MAN_W+1)-1:0] pa_pos,
    output wire        pa_first,
    // profile taps (STATUS layout 10, a_geom's cycle counters): the feed busy, the core not idle, the assembler busy
    output wire        prof_feed, prof_core, prof_asm
);
    localparam int W = 1 + EXP_W + MAN_W, TW = 4*W, SW = $clog2(NSLOT), PAW = $clog2(NPARAM);
    localparam int LNW = (NL > 1) ? $clog2(NL) : 1;                  // a lane of the core's warp
    wire        va_we, in_ready, in_start, in_bank, c_out_valid, c_out_ack, c_idle, c_err;
    wire [LNW-1:0] va_lane; wire [3:0] va_idx; wire [4*W-1:0] va_data; wire [NL-1:0] in_active; wire [1:0] in_warp;
    wire [6:0]  c_out_sel; wire [NL*W-1:0] c_out_row; wire [NWARP-1:0] busy_warps;
    // ---- the uniform shadow and its copy into the core's banks
    (* ram_style = "block" *) reg [TW-1:0] ushadow [0:NPARAM-1];
    initial for (int k = 0; k < NPARAM; k++) ushadow[k] = '0;      // ES 2.0: uniforms are 0 until set
    reg [NPARAM-1:0] dirty_cur, dirty_prev, cmask;
    reg  cp_run, cp_rd; reg [PAW-1:0] cp_idx; reg cbank; reg [TW-1:0] cp_q;
    // the copy's second stage (timing: the BRAM's latch straight to the NL lane banks' DI is too slow): cp_q2 is the
    // BRAM's output register; a third, fabric stage cp_q3 (timing at 120 MHz: the NL lanes' uniform copies are spread
    // over the die, the route alone from the BRAM's output register fills the cycle): the write lands two cycles after
    // the read
    reg  cp_rd2, cp_rd3; reg [PAW-1:0] cp_idx2, cp_idx3; reg [TW-1:0] cp_q2, cp_q3;
    reg  dpar;                                   // the parity the next draw gets
    reg  f_start, f_ps, f_indexed, f_u8; reg [29:0] f_idx_base; reg [31:0] f_first, f_count; reg [7:0] f_vm;   // the taken draw, held for the feed
    reg  [PAW-1:0] cm_idx; reg cm_any;
    always @* begin cm_any = 1'b0; cm_idx = '0; for (int k = NPARAM-1; k >= 0; k--) if (cmask[k]) begin cm_any = 1'b1; cm_idx = PAW'(k); end end
    always @(posedge clk) begin
        if (param_we) ushadow[param_idx] <= param_data;
        cp_q <= ushadow[cm_idx];
        cp_q2 <= cp_q;
        cp_q3 <= cp_q2;
    end
    wire f_issue_idle, f_dcopy_busy, a_dq_full;
    reg [NWARP-1:0] wpar;                        // each core warp slot's draw parity
    wire par_busy = |(busy_warps & ~(wpar ^ {NWARP{dpar}}));   // a warp of the next draw's parity still runs
    assign copy_busy = cp_run | f_dcopy_busy;
    assign dr_ok = f_issue_idle & ~a_dq_full & ~par_busy & ~cp_run & ~f_dcopy_busy & ~f_start;
    always @(posedge clk) begin
        if (rst) begin
            dirty_cur <= '1; dirty_prev <= '0; cmask <= '0; cp_run <= 1'b0; cp_rd <= 1'b0; cp_rd2 <= 1'b0; cp_rd3 <= 1'b0; dpar <= 1'b0; f_start <= 1'b0; wpar <= '0;
        end else begin
            f_start <= 1'b0;
            cp_rd2 <= cp_rd; cp_idx2 <= cp_idx; cp_rd3 <= cp_rd2; cp_idx3 <= cp_idx2;
            if (param_we) dirty_cur[param_idx] <= 1'b1;
            if (dr_start && dr_ok) begin                             // the draw is taken: its uniforms into bank dpar
                cmask <= dirty_cur | dirty_prev; dirty_prev <= dirty_cur; dirty_cur <= param_we ? (NPARAM'(1) << param_idx) : '0;
                cbank <= dpar; dpar <= ~dpar; cp_run <= 1'b1; cp_rd <= 1'b0;
                f_ps <= (dr_mode == 3'd3); f_indexed <= dr_indexed; f_u8 <= dr_u8; f_idx_base <= dr_idx_base; f_first <= dr_first; f_count <= dr_count; f_vm <= dr_varmask;
            end else if (cp_run) begin
                // one entry per cycle: the read is issued this cycle (cp_q lands with cp_rd / cp_idx next cycle, cp_q2 with
                // cp_rd2 / cp_idx2 the cycle after, cp_q3 with cp_rd3 / cp_idx3 one more = the core's write)
                cp_rd <= cm_any; cp_idx <= cm_idx;
                if (cm_any) cmask[cm_idx] <= 1'b0;
                else if (!cp_rd && !cp_rd2 && !cp_rd3) begin cp_run <= 1'b0; f_start <= 1'b1; end   // the last write has landed: start the feed
            end
            if (in_start && in_ready) wpar[in_warp] <= in_bank;
        end
    end
    shader_core_vs2 #(.EXP_W(EXP_W), .MAN_W(MAN_W), .NWARP(NWARP), .NL(NL), .NPARAM(NPARAM), .NATTR(NATTR), .IRAM_DEPTH(IRAM_DEPTH),
                      .SFU_TAB2(SFU_TAB2), .INT_DIV(INT_DIV), .SINCOS(SINCOS)) u_core (
        .clk, .rst, .iram_we, .iram_waddr, .iram_wdata, .param_we(cp_rd3), .param_bank(cbank), .param_idx(cp_idx3), .param_data(cp_q3),
        .va_we, .va_lane, .va_idx, .va_data, .in_ready, .in_warp, .in_start, .in_bank, .in_active, .busy_warps,
        .out_valid(c_out_valid), .out_ack(c_out_ack), .out_sel(c_out_sel), .out_row(c_out_row), .out_err(c_err),
        .err_stack, .idle(c_idle));
    // the draw's parameters are held for the feed until its copy is done (the dispatcher changes nothing meanwhile: dr_ok is low)
    wire        f_busy, a_busy, launch, alloc, ref_valid, ref_ready, ref_dec, lq_empty, launch_ps;
    wire [SW-1:0] launch_slot, alloc_slot, ref_slot, ref_dec_slot, rd_slot;
    wire [LNW-1:0] ref_lane, rd_lane; wire [3:0] rd_vidx; wire [5:0] ref_cnt; wire [7:0] launch_varmask;
    wire [NSLOT-1:0] slot_free, done; wire [4*W-1:0] rb_data;    // the result buffer's read bus
    vtx_feed #(.W(W), .NL(NL), .NSLOT(NSLOT)) u_feed (.clk, .rst,
        .dr_start(f_start), .dr_indexed(f_indexed), .dr_u8(f_u8), .dr_idx_base(f_idx_base), .dr_first(f_first), .dr_count(f_count),
        .dr_bank(cbank), .dr_varmask(f_vm), .dr_ps(f_ps),
        .busy(f_busy), .issue_idle(f_issue_idle), .dcopy_busy(f_dcopy_busy),
        .ad_we, .ad_idx, .ad_en, .ad_type, .ad_norm, .ad_size, .ad_base, .ad_stride, .ad_off, .ac_we, .ac_idx, .ac_const,
        .r_en, .r_addr, .r_rdy, .r_valid, .rd_data,
        .va_we, .va_lane, .va_idx, .va_data, .in_ready, .in_start, .in_bank, .in_active,
        .launch, .launch_slot, .launch_varmask, .launch_ps, .alloc, .alloc_slot, .slot_free,
        .ref_valid, .ref_slot, .ref_lane, .ref_ready, .ref_cnt);
    vtx_rbuf #(.W(W), .NL(NL), .NSLOT(NSLOT), .LQ(8)) u_rbuf (.clk, .rst,
        .out_valid(c_out_valid), .out_ack(c_out_ack), .out_sel(c_out_sel), .out_row(c_out_row),
        .launch, .launch_slot, .launch_varmask, .launch_ps, .alloc, .alloc_slot,
        .ref_inc(ref_valid & ref_ready), .ref_inc_slot(ref_slot), .ref_dec, .ref_dec_slot,
        .slot_free, .done, .lq_empty, .rd_slot, .rd_vidx, .rd_lane, .rd_data(rb_data));
    vtx_asm #(.W(W), .NL(NL), .NSLOT(NSLOT), .NVEC(NVEC)) u_asm (.clk, .rst,
        .start(dr_start & dr_ok), .mode(dr_mode), .count(dr_count), .varmask(dr_varmask), .dq_full(a_dq_full), .draw_done, .busy(a_busy),
        .ref_valid, .ref_slot, .ref_lane, .ref_ready, .ref_cnt,
        .done, .rd_slot, .rd_vidx, .rd_lane, .rd_data(rb_data), .ref_dec, .ref_dec_slot,
        .vo_valid, .vo_ready, .vo_pos, .vo_var, .vo_psize, .vo_prim, .vo_first,
        .vo_la, .pa_req, .pa_req_first, .pa_grant, .pa_valid, .pa_pos, .pa_first);
    assign dr_busy = f_busy | a_busy | ~lq_empty | ~c_idle | cp_run | f_start;
    assign prof_feed = f_busy; assign prof_core = ~c_idle; assign prof_asm = a_busy;
    wire _unused = &{1'b0, c_err};
endmodule
`default_nettype wire
