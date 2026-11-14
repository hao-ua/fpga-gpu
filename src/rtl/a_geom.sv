// a_geom — board A's geometry stage behind the dispatcher's command port: a_vertex (the vertex
//   core + its warp-forming front end) -> a_setup (clip, setup, triangle records in A's DDR) -> tri_binner2 (bin lists),
//   plus the vertex-program loader (the core's 64-bit instructions read from an A buffer, 4 per burst). The dispatcher
//   (fb_dispatch_a) drives it in command order: a frame begin (bin clear + triangle ids -> 0), the program load, uniform
//   writes, the attribute descriptors, the per-draw setup state, a draw; it waits for g_busy to fall before any change
//   that a running draw would see; the frame's FLUSH (g_render) runs a_render: the band walk over the bins and the
//   render stream to B.
//   FRAME BANKING: the records and the bin lists exist in two banks. A frame begin takes
//   the other bank (g_bank toggles), so the geometry of frame N + 1 runs while a_render streams frame N from its bank;
//   g_busy covers the geometry side only, g_rbusy the render side, and g_fb_ok says the bank the next frame begin would
//   take is not the one being streamed (the dispatcher waits for it otherwise).
//   DDR: three mem_arb read ports (the feed, the loader, the render side) and two write ports (records, bin lists),
//   flattened as mem_arb's.
`default_nettype none
module a_geom #(
    parameter int NVEC = 8, NSLOT = 16, NWARP = 4,
    parameter int NBANDS = 512, BANDW = 10, IDX_W = 20, DID_W = 12, PTR_W = 23, COUNT_W = 20,
    parameter [28:0] REC_BASE = 29'h1000000, BINLIST_BASE = 29'h5000000,   // bank 0: records at 64 MB, bin lists at 320 MB
    parameter [28:0] REC_BANK_W = 29'h2000000, POOL_W = 29'h800000,        // 128 MB of records (2^18), 32 MB of bin lists (2^22) per bank
    parameter int    NPROF = 13,                                            // profile counters (STATUS layout 10, words 101-113)
    parameter bit    PROF  = 1'b1,                                          // 0: no profile counters (those words read 0)
    parameter int    BAND8_MAXW = 1024,                                     // targets up to this wide get 8-row bands, wider ones 4
                                                                            // (board B's b_render BAND8_MAXW; 0: 4-row bands always)
    parameter bit    VS_SFU_TAB2 = 1'b0,                                    // the vertex core's SFU tables shared by pairs of units
    parameter bit    VS_INT_DIV  = 1'b1,                                    // the vertex core's integer divider
    parameter bit    VS_SINCOS   = 1'b1,                                    // the vertex core's SIN / COS units
    parameter int    VS_NL       = 16                                       // the vertex core's lanes per warp (16, 8 or 4)
)(
    input  wire        clk, rst,
    // command port
    input  wire        g_frame,                  // bin clear + triangle id reset, in the other bank
    output wire        g_frame_done,             // the clear is finished
    output wire        g_rbusy,                  // the render side is streaming a frame
    output wire        g_fb_ok,                  // the bank a frame begin would take is not being streamed
    output wire        g_draw_ok,                // a draw may start now (draws overlap)
    output wire        g_copy_busy,              // a uniform / descriptor shadow is being copied: no write meanwhile
    input  wire        g_ld_start,
    input  wire [28:0] g_ld_addr,                // word address of the program's first instruction
    input  wire [11:0] g_ld_len,                 // instructions
    output reg         g_ld_done,
    input  wire        g_param_we,
    input  wire [7:0]  g_param_idx,
    input  wire [95:0] g_param_data,
    input  wire        g_ad_we,
    input  wire [3:0]  g_ad_idx,
    input  wire        g_ad_en,
    input  wire [2:0]  g_ad_type,                // attr_conv type code (BYTE .. FP24)
    input  wire        g_ad_norm,
    input  wire [29:0] g_ad_base,                // bytes
    input  wire [2:0]  g_ad_size,
    input  wire [29:0] g_ad_stride, g_ad_off,    // bytes
    input  wire        g_ac_we,
    input  wire [3:0]  g_ac_idx,
    input  wire [95:0] g_ac_const,
    input  wire        g_st_we,
    input  wire [95:0] g_st_vp,                  // {syo, sys, sxo, sxs} FP24, 1/16 pixel
    input  wire [47:0] g_st_z,                   // {zo, zs} FP24: the depth range (window z = ndc_z * zs + zo)
    input  wire [95:0] g_st_gb,                  // {gyhi, gylo, gxhi, gxlo} FP24: the guard band's NDC bounds (the x / y clip planes)
    input  wire [47:0] g_st_sc,                  // {y1, x1, y0, x0}: the draw's rectangle in pixels (its scissor box in the target; fb_dispatch_a)
    input  wire [1:0]  g_st_cull,
    input  wire        g_st_cw,
    input  wire        g_st_z0 = 1'b0,           // the clip volume 0 <= z <= w (VS_STATE word 4 bit 24: a_setup zclip0_i)
    input  wire [6:0]  g_st_lw,                  // the line width (pixels)
    input  wire [7:0]  g_st_fsmask,
    input  wire [11:0] g_st_w, g_st_h,
    input  wire [12:0] g_st_did,
    input  wire        g_dr_start,
    input  wire [2:0]  g_dr_mode,
    input  wire        g_dr_indexed, g_dr_u8,
    input  wire [29:0] g_dr_idx_base,            // byte address of index 0
    input  wire [31:0] g_dr_first, g_dr_count,
    input  wire        g_render,                 // the frame's FLUSH: walk the bins, send the render stream
    input  wire        g_abort,                  // give the frame up (g_render_done follows)
    input  wire        g_abort_soft,             // end the frame in-band at the next item boundary (g_render_done follows)
    output wire        g_render_done,
    output wire        g_busy,
    output wire        err_stack,
    output wire        rec_ovf, bin_ovf,         // sticky per frame: records beyond the region / bin entries beyond the pool
    output wire [IDX_W-1:0] tri_count,
    output wire [31:0] c_quads,                  // the render side's statistics
    output wire [31:0] c_vis,                    //   primitive-band visits, cumulative (STATUS word 100)
    output wire [NPROF*32-1:0] prof,             // the geometry side's profile, cumulative cycle counters (STATUS words 101-113, see below)
    // DDR (mem_arb: read ports 0 = feed, 1 = loader, 2 = the render side; write ports 0 = records, 1 = bin lists)
    output wire [2:0]  r_en,
    output wire [86:0] r_addr,
    input  wire [2:0]  r_rdy, r_valid,
    input  wire [255:0] rd_data,
    output wire [1:0]  w_req, w_en,
    output wire [57:0] w_addr,
    output wire [511:0] w_data,
    output wire [63:0] w_mask,
    output wire [1:0]  w_wren, w_wend,
    input  wire [1:0]  w_rdy, w_wdf_rdy,
    // the render stream to link_a_port (RSTREAM)
    output wire        rs_valid,
    output wire [63:0] rs_data,
    input  wire        rs_ready
);
    localparam int W = 24, TW = 96;
    // ---- per-draw state registers
    reg [W-1:0] sxs, sxo, sys, syo, zs, zo; reg [1:0] cull; reg cw, z0; reg [6:0] lw; reg [7:0] fsmask; reg [11:0] scr_w, scr_h; reg [12:0] did; reg [95:0] gbv; reg [47:0] scv;
    always @(posedge clk) if (g_st_we) begin
        sxs <= g_st_vp[0 +: W]; sxo <= g_st_vp[W +: W]; sys <= g_st_vp[2*W +: W]; syo <= g_st_vp[3*W +: W]; zs <= g_st_z[0 +: W]; zo <= g_st_z[W +: W];
        cull <= g_st_cull; cw <= g_st_cw; z0 <= g_st_z0; lw <= g_st_lw; fsmask <= g_st_fsmask; scr_w <= g_st_w; scr_h <= g_st_h; did <= g_st_did; gbv <= g_st_gb; scv <= g_st_sc;
    end
    // ---- the program loader: one burst outstanding, 4 instructions per burst (two words each, the low half first)
    typedef enum logic [1:0] {L_IDLE, L_REQ, L_WAIT, L_WRITE} ld_t;
    ld_t ld_st;
    reg [28:0] ld_addr; reg [11:0] ld_left, ld_w; reg [255:0] ld_b; reg [1:0] ld_k;
    reg iram_we; reg [11:0] iram_waddr; reg [63:0] iram_wdata;
    wire ld_run = (ld_st != L_IDLE);
    assign r_en[1] = (ld_st == L_REQ); assign r_addr[57:29] = ld_addr;
    always @(posedge clk) begin
        if (rst) begin ld_st <= L_IDLE; g_ld_done <= 1'b0; iram_we <= 1'b0; end
        else begin
            g_ld_done <= 1'b0; iram_we <= 1'b0;
            case (ld_st)
                L_IDLE: if (g_ld_start) begin ld_addr <= g_ld_addr; ld_left <= g_ld_len; ld_w <= 12'd0; ld_st <= (g_ld_len == 12'd0) ? L_IDLE : L_REQ; g_ld_done <= (g_ld_len == 12'd0); end
                L_REQ:  if (r_rdy[1]) ld_st <= L_WAIT;
                L_WAIT: if (r_valid[1]) begin ld_b <= rd_data; ld_k <= 2'd0; ld_addr <= ld_addr + 29'd8; ld_st <= L_WRITE; end
                L_WRITE: begin
                    iram_we <= 1'b1; iram_waddr <= ld_w; iram_wdata <= ld_b[ld_k*64 +: 64]; ld_w <= ld_w + 1'b1; ld_left <= ld_left - 1'b1;
                    if (ld_left == 12'd1) begin ld_st <= L_IDLE; g_ld_done <= 1'b1; end
                    else if (ld_k == 2'd3) ld_st <= L_REQ;
                    else ld_k <= ld_k + 1'b1;
                end
                default: ld_st <= L_IDLE;
            endcase
        end
    end
    // ---- the vertex stage (draws overlap; the dispatcher starts a draw when g_draw_ok, writes uniforms /
    //      descriptors when !g_copy_busy; the setup stage's per-draw state queues in a FIFO that the assembler's draw_done advances)
    wire dr_busy, dr_ok, copy_busy, draw_done, vo_valid, vo_ready, vo_first, prof_feed, prof_core, prof_asm; wire [TW-1:0] vo_pos; wire [NVEC*TW-1:0] vo_var; wire [W-1:0] vo_psize; wire [1:0] vo_prim;
    wire vo_la, pa_req, pa_req_first, pa_grant, pa_valid, pa_first; wire [TW-1:0] pa_pos;   // a triangle's positions ahead of its vertices
    localparam int SFW = 6*W + 2 + 1 + 7 + 8 + 12 + 12 + 13 + 96 + 48 + 1;   // the state entry: vp, z, cull, cw, lw, fsmask, w, h, did, the guard band, the rectangle, z0
    reg [SFW-1:0] sfifo [0:3]; reg [1:0] sf_wp, sf_rp; reg [2:0] sf_cnt;
    wire sf_full = (sf_cnt == 3'd4);
    wire [SFW-1:0] sf_head = sfifo[sf_rp];
    always @(posedge clk) begin
        if (rst) begin sf_wp <= 2'd0; sf_rp <= 2'd0; sf_cnt <= 3'd0; end
        else begin
            if (g_dr_start) begin sfifo[sf_wp] <= {z0, scv, gbv, did, scr_h, scr_w, fsmask, lw, cw, cull, zo, zs, syo, sys, sxo, sxs}; sf_wp <= sf_wp + 2'd1; end
            if (draw_done) sf_rp <= sf_rp + 2'd1;
            sf_cnt <= sf_cnt + 3'(g_dr_start) - 3'(draw_done);
        end
    end
    assign g_draw_ok = dr_ok & ~sf_full;
    assign g_copy_busy = copy_busy;
    a_vertex #(.NVEC(NVEC), .NSLOT(NSLOT), .NWARP(NWARP), .NL(VS_NL), .SFU_TAB2(VS_SFU_TAB2), .INT_DIV(VS_INT_DIV), .SINCOS(VS_SINCOS)) u_vtx (.clk, .rst,
        .iram_we, .iram_waddr, .iram_wdata, .param_we(g_param_we), .param_idx(g_param_idx), .param_data(g_param_data),
        .ad_we(g_ad_we), .ad_idx(g_ad_idx), .ad_en(g_ad_en), .ad_type(g_ad_type), .ad_norm(g_ad_norm), .ad_base(g_ad_base), .ad_size(g_ad_size),
        .ad_stride(g_ad_stride), .ad_off(g_ad_off),
        .ac_we(g_ac_we), .ac_idx(g_ac_idx), .ac_const(g_ac_const),
        .dr_start(g_dr_start), .dr_mode(g_dr_mode), .dr_indexed(g_dr_indexed), .dr_u8(g_dr_u8), .dr_idx_base(g_dr_idx_base),
        .dr_first(g_dr_first), .dr_count(g_dr_count), .dr_varmask(fsmask), .dr_ok, .copy_busy, .draw_done, .dr_busy, .err_stack,
        .r_en(r_en[0]), .r_addr(r_addr[28:0]), .r_rdy(r_rdy[0]), .r_valid(r_valid[0]), .rd_data,
        .vo_valid, .vo_ready, .vo_pos, .vo_var, .vo_psize, .vo_prim, .vo_first,
        .vo_la, .pa_req, .pa_req_first, .pa_grant, .pa_valid, .pa_pos, .pa_first, .prof_feed, .prof_core, .prof_asm);
    // ---- the banks: g_frame takes the other bank for the new frame (the first frame after reset: bank 0); the render
    //      side streams the bank the frame was built in
    reg gbank, rbank; wire rn_busy;
    wire gbank_c = g_frame ? ~gbank : gbank;    // the bank of the frame being built, valid in the frame-begin cycle already (the clear samples it)
    always @(posedge clk) begin
        if (rst) begin gbank <= 1'b1; rbank <= 1'b0; end
        else begin
            if (g_frame) gbank <= ~gbank;
            if (g_render) rbank <= gbank;
        end
    end
    assign g_rbusy = rn_busy;
    assign g_fb_ok = ~(rn_busy & (rbank == ~gbank));
    // ---- setup and records
    wire s_busy, push_valid, push_ready; wire [IDX_W-1:0] push_tri; wire [12:0] push_did; wire [BANDW-1:0] push_lo, push_hi; wire [3:0] push_nb;
    a_setup #(.NVEC(NVEC), .BANDW(BANDW), .IDX_W(IDX_W), .DID_W(13), .REC_BASE(REC_BASE), .REC_BANK_W(REC_BANK_W),
              .REC_ENTRIES(int'(REC_BANK_W >> 7)), .BAND8_MAXW(BAND8_MAXW)) u_set (.clk, .rst, .frame_start(g_frame), .bank(gbank_c),   // 128 words per record
        .vo_valid, .vo_ready, .vo_pos, .vo_var, .vo_psize, .vo_prim,
        .vo_la, .pa_req, .pa_req_first, .pa_grant, .pa_valid, .pa_pos, .pa_first,
        .sxs_i(sf_head[0 +: W]), .sxo_i(sf_head[W +: W]), .sys_i(sf_head[2*W +: W]), .syo_i(sf_head[3*W +: W]), .zs_i(sf_head[4*W +: W]), .zo_i(sf_head[5*W +: W]),
        .cull_mode_i(sf_head[6*W +: 2]), .front_cw_i(sf_head[6*W+2]), .zclip0_i(sf_head[6*W+199]), .line_width_i(sf_head[6*W+3 +: 7]), .fsmask_i(sf_head[6*W+10 +: 8]),
        .scr_w_i(sf_head[6*W+18 +: 12]), .scr_h_i(sf_head[6*W+30 +: 12]), .draw_id_i(sf_head[6*W+42 +: 13]), .gb_i(sf_head[6*W+55 +: 96]), .sc_i(sf_head[6*W+151 +: 48]),
        .busy(s_busy), .tri_count, .rec_ovf,
        .w_req(w_req[0]), .w_en(w_en[0]), .w_addr(w_addr[28:0]), .w_data(w_data[255:0]), .w_mask(w_mask[31:0]), .w_wren(w_wren[0]), .w_wend(w_wend[0]),
        .w_rdy(w_rdy[0]), .w_wdf_rdy(w_wdf_rdy[0]),
        .push_valid, .push_ready, .push_tri_idx(push_tri), .push_draw_id(push_did), .push_band_lo(push_lo), .push_band_hi(push_hi),
        .push_tri_nb(push_nb));
    // ---- bins
    wire b_busy, b_clear_done, b_w_busy;
    wire [BANDW-1:0] q_band; wire [PTR_W-1:0] q_head; wire [COUNT_W-1:0] q_count;
    wire q_bank;
    tri_binner2 #(.NBANDS(NBANDS), .IDX_W(IDX_W), .DID_W(DID_W), .PTR_W(PTR_W), .COUNT_W(COUNT_W), .BINLIST_BASE(BINLIST_BASE), .POOL_W(POOL_W),
                  .POOL_ENTRIES(int'(POOL_W >> 1))) u_bin (.clk, .rst,
        .clear(g_frame), .clear_done(b_clear_done), .busy(b_busy), .ovf(bin_ovf), .push_bank(gbank_c),
        .push_valid, .push_ready, .tri_idx(push_tri), .draw_id(push_did[DID_W-1:0]), .band_lo(push_lo), .band_hi(push_hi), .tri_nb(push_nb),
        .q_bank, .q_band, .q_head, .q_count, .q_valid_head(),
        .can_start(1'b1), .w_en(w_en[1]), .w_busy(b_w_busy), .w_addr(w_addr[57:29]), .w_data(w_data[511:256]), .w_mask(w_mask[63:32]),
        .w_wren(w_wren[1]), .w_wend(w_wend[1]), .w_rdy(w_rdy[1]), .w_wdf_rdy(w_wdf_rdy[1]), .dbg_st());
    assign w_req[1] = b_w_busy;
    reg clearing;
    always @(posedge clk) if (rst) clearing <= 1'b0; else if (g_frame) clearing <= 1'b1; else if (b_clear_done) clearing <= 1'b0;
    assign g_frame_done = clearing & b_clear_done;
    // ---- the render side: band walk over the bins, the rasterizer, the render stream
    a_render #(.NL(4), .BANDW(BANDW), .IDX_W(IDX_W), .DID_W(DID_W), .PTR_W(PTR_W), .COUNT_W(COUNT_W), .REC_BASE(REC_BASE), .BINLIST_BASE(BINLIST_BASE),
               .REC_BANK_W(REC_BANK_W), .POOL_W(POOL_W), .BAND8_MAXW(BAND8_MAXW))
      u_rn (.clk, .rst, .go(g_render), .go_bank(gbank), .abort(g_abort), .abort_soft(g_abort_soft), .done(g_render_done), .busy(rn_busy), .scr_w, .scr_h,
        .c_vis,
        .q_bank, .q_band, .q_head, .q_count,
        .r_en(r_en[2]), .r_addr(r_addr[86:58]), .r_rdy(r_rdy[2]), .r_valid(r_valid[2]), .rd_data,
        .rs_valid, .rs_data, .rs_ready, .c_quads, .c_tris());
    assign g_busy = dr_busy | s_busy | b_busy | clearing | ld_run | vo_valid | push_valid | (sf_cnt != 3'd0);   // the geometry side (the render side: g_rbusy)
    // ---- the geometry side's profile (STATUS layout 10): cumulative cycle counters, the host takes deltas and
    //      divides by the geometry-busy cycles (STATUS word 96). Each counts the cycles its condition holds, so the stages
    //      overlap and the shares do not add up to 100 %:
    //        0 program loader running            1 uniform / descriptor shadow copy      2 vertex feed busy (fetch + launch)
    //        3 feed's DDR read request refused   4 vertex core not idle                  5 assembler busy
    //        6 a vertex waits for the setup      7 setup busy                            8 setup's push waits for the binner
    //        9 binner busy                      10 the frame's bin clear                11 {program loads[31:16], draws[15:0]}
    //       12 primitives presented to the setup (vo handshakes)
    reg [31:0] pc [0:NPROF-1];
    reg [15:0] pn_ld, pn_dr;
    always @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < NPROF; i++) pc[i] <= 32'd0;
            pn_ld <= 16'd0; pn_dr <= 16'd0;
        end else begin
            if (ld_run)                 pc[0]  <= pc[0] + 1;
            if (copy_busy)              pc[1]  <= pc[1] + 1;
            if (prof_feed)              pc[2]  <= pc[2] + 1;
            if (r_en[0] && !r_rdy[0])   pc[3]  <= pc[3] + 1;
            if (prof_core)              pc[4]  <= pc[4] + 1;
            if (prof_asm)               pc[5]  <= pc[5] + 1;
            if (vo_valid && !vo_ready)  pc[6]  <= pc[6] + 1;
            if (s_busy)                 pc[7]  <= pc[7] + 1;
            if (push_valid && !push_ready) pc[8] <= pc[8] + 1;
            if (b_busy)                 pc[9]  <= pc[9] + 1;
            if (clearing)               pc[10] <= pc[10] + 1;
            if (g_ld_start)             pn_ld <= pn_ld + 1;
            if (g_dr_start)             pn_dr <= pn_dr + 1;
            pc[11] <= {pn_ld, pn_dr};
            if (vo_valid && vo_ready)   pc[12] <= pc[12] + 1;
        end
    end
    generate for (genvar pi = 0; pi < NPROF; pi++) begin : g_prof
        assign prof[pi*32 +: 32] = PROF ? pc[pi] : 32'd0;          // PROF 0: the words read 0, the counters are dropped
    end endgenerate
    wire _unused = &{1'b0, vo_first, push_did[12]};
endmodule
`default_nettype wire
