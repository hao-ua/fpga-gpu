`timescale 1ns/1ps
// tb_a_geom — board A's geometry stage: a_vertex -> a_setup -> tri_binner, with mem_arb (1 read, 2 write ports) on the MIG
// model, on one draw of tools/a_setup_ref.py (which builds the draw with a_vtx_ref.py): the DDR image goes into the
// model, program / uniforms / descriptors / draw state through the ports; the binner pushes (tri id, band range), in
// order, and the triangle records in DDR (REC_BASE + id * 128 words) must equal the golden word for word.
//   +SEED_<n>   (tools/gen/av_<n>_*, as_<n>_state.mem, as_<n>_rec.hex)
`default_nettype none
module tb_a_geom;
    localparam int W = 24, TW = 96, NVEC = 8, VW = NVEC * TW, BANDW = 10, IDX_W = 20, DID_W = 12;
    localparam [28:0] REC_BASE = 29'h1000000, BINLIST_BASE = 29'h1800000;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    reg         iram_we = 0, param_we = 0, ad_we = 0, ac_we = 0, dr_start = 0, frame_start = 0, bin_clear = 0;
    reg  [11:0] iram_waddr = 0; reg [63:0] iram_wdata = 0; reg [7:0] param_idx = 0; reg [TW-1:0] param_data = 0;
    reg  [3:0]  ad_idx = 0, ac_idx = 0; reg ad_en = 0, ad_norm = 0; reg [29:0] ad_base = 0, ad_stride = 0, ad_off = 0; reg [2:0] ad_size = 0, ad_type = 0;
    reg  [TW-1:0] ac_const = 0;
    reg  [2:0]  dr_mode = 0; reg dr_indexed = 0, dr_u8 = 0; reg [29:0] dr_idx_base = 0; reg [31:0] dr_first = 0, dr_count = 0; reg [7:0] varmask = 0;
    reg  [W-1:0] sxs = 0, sxo = 0, sys = 0, syo = 0, zs = 0, zo = 0; reg [1:0] cull = 0; reg front_cw = 0, zclip0 = 0; reg [7:0] fsmask = 0; reg [12:0] draw_id = 0;
    reg  [11:0] scr_w = 0, scr_h = 0; reg [95:0] gbv = 0; reg [47:0] scv = 0;      // (scv: the draw's rectangle {y1, x1, y0, x0}, the state file's top 48 bits)
    wire        dr_busy, dr_ok, err_stack, vo_valid, vo_ready, vo_first, s_busy; wire [TW-1:0] vo_pos; wire [VW-1:0] vo_var;
    wire [W-1:0] vo_psize; wire [1:0] vo_prim; reg [6:0] lw = 1;
    wire [IDX_W-1:0] tri_count;
    // memory: mem_arb (1 read port: the feed; 2 write ports: records, bin lists) on the MIG model
    wire r_en, r_rdy, r_valid; wire [28:0] r_addr; wire [255:0] rd_data;
    wire [1:0] w_req, w_en, w_wren, w_wend, w_rdy, w_wdf_rdy; wire [57:0] w_addr; wire [511:0] w_data; wire [63:0] w_mask;
    wire app_en, app_wdf_wren, app_wdf_end, app_rdy, app_wdf_rdy, app_rd_valid; wire [2:0] app_cmd; wire [28:0] app_addr;
    wire [255:0] app_wdf_data, app_rd_data; wire [31:0] app_wdf_mask; int mig_errors; longint n_wr, n_rd;
    mem_arb #(.NR(1), .NW(2), .RO_AW(5), .RO_RESERVE(4), .MUST_RUN(1'b0)) arb (.clk, .rst,
        .r_en(r_en), .r_addr(r_addr), .r_rdy(r_rdy), .r_valid(r_valid), .rd_data,
        .w_req, .w_en, .w_addr, .w_data, .w_mask, .w_wren, .w_wend, .w_rdy, .w_wdf_rdy,
        .app_en, .app_cmd, .app_addr, .app_wdf_data, .app_wdf_mask, .app_wdf_wren, .app_wdf_end,
        .app_rdy, .app_wdf_rdy, .app_rd_data_valid(app_rd_valid), .app_rd_data);
    mig_app_model #(.RD_LAT(24), .RDY_PCT(70), .WDF_PCT(75)) mig (.ui_clk(clk), .rst, .app_en, .app_cmd, .app_addr, .app_wdf_data,
        .app_wdf_mask, .app_wdf_wren, .app_wdf_end, .app_rdy, .app_wdf_rdy, .app_rd_data, .app_rd_data_valid(app_rd_valid),
        .errors(mig_errors), .n_wr, .n_rd);
    a_vertex #(.NVEC(NVEC)) u_vtx (.clk, .rst, .iram_we, .iram_waddr, .iram_wdata, .param_we, .param_idx, .param_data,
        .ad_we, .ad_idx, .ad_en, .ad_type, .ad_norm, .ad_base, .ad_size, .ad_stride, .ad_off, .ac_we, .ac_idx, .ac_const,
        .dr_start, .dr_mode, .dr_indexed, .dr_u8, .dr_idx_base, .dr_first, .dr_count, .dr_varmask(varmask), .dr_ok, .copy_busy(), .draw_done(), .dr_busy, .err_stack,
        .r_en, .r_addr, .r_rdy, .r_valid, .rd_data, .vo_valid, .vo_ready, .vo_pos, .vo_var, .vo_psize, .vo_prim, .vo_first);
    wire push_valid, push_ready; wire [IDX_W-1:0] push_tri; wire [12:0] push_did; wire [BANDW-1:0] push_lo, push_hi; wire [3:0] push_nb;
    a_setup #(.NVEC(NVEC), .BANDW(BANDW), .IDX_W(IDX_W), .DID_W(13), .REC_BASE(REC_BASE)) u_set (.clk, .rst, .frame_start, .bank(1'b0),
        .vo_valid, .vo_ready, .vo_pos, .vo_var, .vo_psize, .vo_prim, .sxs_i(sxs), .sxo_i(sxo), .sys_i(sys), .syo_i(syo), .zs_i(zs), .zo_i(zo),
        .cull_mode_i(cull), .front_cw_i(front_cw), .zclip0_i(zclip0), .line_width_i(lw), .fsmask_i(fsmask), .draw_id_i(draw_id), .scr_w_i(scr_w), .scr_h_i(scr_h), .gb_i(gbv), .sc_i(scv),
        .busy(s_busy), .tri_count,
        .w_req(w_req[0]), .w_en(w_en[0]), .w_addr(w_addr[28:0]), .w_data(w_data[255:0]), .w_mask(w_mask[31:0]), .w_wren(w_wren[0]), .w_wend(w_wend[0]),
        .w_rdy(w_rdy[0]), .w_wdf_rdy(w_wdf_rdy[0]),
        .push_valid, .push_ready, .push_tri_idx(push_tri), .push_draw_id(push_did), .push_band_lo(push_lo), .push_band_hi(push_hi),
        .push_tri_nb(push_nb));
    wire tb_busy, tb_clear_done, tb_w_busy;
    tri_binner2 #(.NBANDS(512), .IDX_W(IDX_W), .DID_W(DID_W), .PTR_W(23), .COUNT_W(20), .BINLIST_BASE(BINLIST_BASE)) u_bin (.clk, .rst,
        .clear(bin_clear), .clear_done(tb_clear_done), .busy(tb_busy), .push_bank(1'b0),
        .push_valid, .push_ready, .tri_idx(push_tri), .draw_id(push_did[DID_W-1:0]), .band_lo(push_lo), .band_hi(push_hi), .tri_nb(push_nb),
        .q_bank(1'b0), .q_band('0), .q_head(), .q_count(), .q_valid_head(),
        .can_start(1'b1), .w_en(w_en[1]), .w_busy(tb_w_busy), .w_addr(w_addr[57:29]), .w_data(w_data[511:256]), .w_mask(w_mask[63:32]),
        .w_wren(w_wren[1]), .w_wend(w_wend[1]), .w_rdy(w_rdy[1]), .w_wdf_rdy(w_wdf_rdy[1]), .dbg_st());
    assign w_req[1] = tb_w_busy;

    reg [63:0] iram [0:4095]; reg [TW-1:0] prm [0:255]; reg [106:0] drw [0:0]; reg [193:0] att [0:15]; reg [343:0] stt [0:0];   // formats: tb_a_vertex; the state: a_setup_ref
    int exp_id [0:4095], exp_lo [0:4095], exp_hi [0:4095], exp_n [0:4095]; reg [31:0] exp_w [0:4095][0:111];   // records of up to 106 words (D1: 26 + 10 per read varying)
    initial begin
        string seed; int fd, n, nexp, err, cyc, b, npush, k; reg [31:0] bi; reg [255:0] bd; int v_id, v_lo, v_hi, v_n, v_w;
        if (!$value$plusargs("SEED_%s", seed)) seed = "1";
        $readmemh({"tools/gen/av_", seed, "_iram.mem"}, iram);
        $readmemh({"tools/gen/av_", seed, "_params.mem"}, prm);
        $readmemh({"tools/gen/av_", seed, "_draw.mem"}, drw);
        $readmemh({"tools/gen/av_", seed, "_attr.mem"}, att);
        $readmemh({"tools/gen/as_", seed, "_state.mem"}, stt);
        fd = $fopen({"tools/gen/av_", seed, "_ddr.hex"}, "r");
        if (fd == 0) begin $display("TB_FAIL tb_a_geom: no DDR image"); $finish; end
        b = 0; while ($fscanf(fd, "%h %h", bi, bd) == 2) begin mig.mem[longint'(bi)] = bd; b++; end
        $fclose(fd);
        fd = $fopen({"tools/gen/as_", seed, "_rec.hex"}, "r");
        if (fd == 0) begin $display("TB_FAIL tb_a_geom: no record golden"); $finish; end
        nexp = 0;
        while ($fscanf(fd, "%h %h %h %h", v_id, v_lo, v_hi, v_n) == 4) begin
            exp_id[nexp] = v_id; exp_lo[nexp] = v_lo; exp_hi[nexp] = v_hi; exp_n[nexp] = v_n;
            for (k = 0; k < v_n; k++) begin void'($fscanf(fd, "%h", v_w)); exp_w[nexp][k] = v_w; end
            nexp++;
        end
        $fclose(fd);
        repeat (4) @(posedge clk); rst <= 0; repeat (40) @(posedge clk);          // the float units' valid chains settle
        @(posedge clk); bin_clear <= 1; frame_start <= 1; @(posedge clk); bin_clear <= 0; frame_start <= 0;
        while (!tb_clear_done) @(posedge clk);
        n = 0; while (n < 4096 && !$isunknown(iram[n])) n++;
        for (k = 0; k < n; k++) begin @(posedge clk); iram_we <= 1; iram_waddr <= 12'(k); iram_wdata <= iram[k]; end
        for (k = 0; k < 256; k++) begin @(posedge clk); iram_we <= 0; param_we <= 1; param_idx <= 8'(k); param_data <= prm[k]; end
        @(posedge clk); param_we <= 0;
        for (k = 0; k < 16; k++) begin
            @(posedge clk); ad_we <= 1; ac_we <= 1; ad_idx <= 4'(k); ac_idx <= 4'(k);
            ad_base <= att[k][29:0]; ad_en <= att[k][30]; ad_size <= att[k][33:31]; ad_type <= att[k][36:34]; ad_norm <= att[k][37];
            ad_stride <= att[k][67:38]; ad_off <= att[k][97:68]; ac_const <= att[k][193:98];
        end
        @(posedge clk); ad_we <= 0; ac_we <= 0;
        dr_mode <= drw[0][2:0]; dr_indexed <= drw[0][3]; dr_u8 <= drw[0][4]; dr_first <= drw[0][36:5]; dr_count <= drw[0][68:37];
        dr_idx_base <= drw[0][98:69]; varmask <= drw[0][106:99];
        sxs <= stt[0][23:0]; sxo <= stt[0][47:24]; sys <= stt[0][71:48]; syo <= stt[0][95:72];
        cull <= stt[0][97:96]; front_cw <= stt[0][98]; draw_id <= stt[0][111:99]; scr_w <= stt[0][123:112]; scr_h <= stt[0][135:124]; fsmask <= stt[0][143:136]; scv <= stt[0][343:296];
        zs <= stt[0][175:152]; zo <= stt[0][199:176];                     // the depth range (window z = ndc_z * zs + zo)
        gbv <= stt[0][295:200];                                          // the guard band's NDC bounds {gyhi, gylo, gxhi, gxlo}
        lw <= stt[0][150:144]; zclip0 <= stt[0][151];                   // (bit 151: the clip volume 0 <= z <= w, a_setup_ref --z0)
        @(posedge clk); while (!dr_ok) @(posedge clk);
        dr_start <= 1; @(posedge clk); dr_start <= 0;
        // the pushes, in order
        npush = 0; err = 0; cyc = 0;
        while (cyc < 4_000_000) begin
            @(negedge clk); cyc++;
            if (push_valid && push_ready) begin
                if (npush < nexp) begin
                    if (push_tri != exp_id[npush] || push_lo != exp_lo[npush] || push_hi != exp_hi[npush]) begin
                        err++; if (err < 8) $display("  push %0d: id %0d bands %0d..%0d, expected id %0d bands %0d..%0d", npush, push_tri, push_lo, push_hi, exp_id[npush], exp_lo[npush], exp_hi[npush]);
                    end
                end else begin err++; if (err < 8) $display("  extra push %0d (id %0d)", npush, push_tri); end
                npush++;
            end
            if (cyc > 10 && !dr_busy && !s_busy && !tb_busy && !push_valid && !vo_valid) break;
        end
        repeat (8) @(posedge clk);
        if (npush != nexp) begin err++; $display("  %0d pushes, expected %0d", npush, nexp); end
        if (dr_busy || s_busy) begin err++; $display("  stage still busy after %0d cycles", cyc); end
        // the records in DDR
        for (k = 0; k < nexp; k++) begin
            int bad; bad = 0;
            for (int w = 0; w < exp_n[k]; w++) begin
                longint wa; reg [255:0] burst; reg [31:0] got;
                wa = longint'(REC_BASE) + exp_id[k] * 128 + w;
                burst = mig.mem.exists(wa >> 3) ? mig.mem[wa >> 3] : 256'd0;
                got = burst[(wa & 7) * 32 +: 32];
                if (got !== exp_w[k][w]) begin bad++; if (err + bad < 10) $display("  record %0d word %0d: got %08x expected %08x", exp_id[k], w, got, exp_w[k][w]); end
            end
            if (bad) err++;
        end
        if (err_stack) begin err++; $display("  core stack violation"); end
        if (mig_errors != 0) begin err++; $display("  MIG model errors %0d", mig_errors); end
        $display("AG seed %s: %0d records / pushes (%0d expected) in %0d cycles, %0d DDR reads, %0d writes, errors = %0d",
                 seed, npush, nexp, cyc, n_rd, n_wr, err);
        if (err == 0) $display("TB_PASS tb_a_geom %s", seed); else $display("TB_FAIL tb_a_geom %s", seed);
        $finish;
    end
    initial begin #80ms; $display("TB_FAIL tb_a_geom: TIMEOUT"); $finish; end
    // +PROFILE: cycles per state of the setup stage, the clipper and the binner (where a triangle's time goes)
    int hs [int], hc [int], hb [int]; int prof_on = 0;
    always @(posedge clk) if (prof_on && !rst) begin
        hs[int'(u_set.st)]++; hc[int'(u_set.u_clip.cs)]++; hb[int'(u_bin.st)]++;
    end
    initial begin
        if ($test$plusargs("PROFILE")) begin
            prof_on = 1;
            @(negedge clk); wait (u_set.tri_count != 0 || $time > 10ms);          // (print at the end: the main block finishes first)
        end
    end
    final if (prof_on) begin                   // state numbers = the enum order in a_setup / clip_near / tri_binner2
        int k;
        $display("PROFILE setup states:");  if (hs.first(k)) do $display("  st %2d %9d", k, hs[k]); while (hs.next(k));
        $display("PROFILE clipper states:"); if (hc.first(k)) do $display("  cs %2d %9d", k, hc[k]); while (hc.next(k));
        $display("PROFILE binner states:");  if (hb.first(k)) do $display("  st %2d %9d", k, hb[k]); while (hb.next(k));
    end
`ifdef GEOM_TRACE
    // the clipper's traffic per primitive (sim_gpu.ps1 -Defines GEOM_TRACE)
    always @(posedge clk) begin
        if (u_set.clip_start) $display("  TRACE clip start   tid %0d (w %h %h %h z %h %h %h)", u_set.tid, u_set.vW[0], u_set.vW[1], u_set.vW[2], u_set.vZ[0], u_set.vZ[1], u_set.vZ[2]);
        if (u_set.clip_tv)    $display("  TRACE clip tri     tid %0d: polygon %0d vertices, triangle %0d: refs %h %h %h x %h %h %h w %h %h %h", u_set.tid, u_set.u_clip.n_in, u_set.u_clip.ok,
                                       u_set.u_clip.pin[0], u_set.u_clip.pin[u_set.u_clip.ok], u_set.u_clip.pin[u_set.u_clip.ok + 1],
                                       u_set.clip_to[0*u_set.VB + 0*24 +: 24], u_set.clip_to[1*u_set.VB + 0*24 +: 24], u_set.clip_to[2*u_set.VB + 0*24 +: 24],
                                       u_set.clip_to[0*u_set.VB + 3*24 +: 24], u_set.clip_to[1*u_set.VB + 3*24 +: 24], u_set.clip_to[2*u_set.VB + 3*24 +: 24]);
        if (u_set.clip_done)  $display("  TRACE clip done    tid %0d", u_set.tid);
        if (u_set.st == u_set.S_CHK2 && !(u_set.rej_any | ((u_set.prim == 2'd0) ? (u_set.culled | u_set.offscreen) : 1'b0)) == 1'b0)
            $display("  TRACE dropped      tid %0d: rej %b culled %b offscreen %b", u_set.tid, u_set.rej_any, u_set.culled, u_set.offscreen);
    end
`endif
endmodule
`default_nettype wire
