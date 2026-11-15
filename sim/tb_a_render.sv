`timescale 1ns/1ps
// tb_a_render — board A's render side: the host commands of a draw (tools/gen/as_<seed>_cmds.hex as ingress entries)
// through fb_dispatch_a into a_geom (a_vertex -> a_setup -> tri_binner, records and bin lists in the MIG model's DDR),
// then a FLUSH appended by the bench: the dispatcher forwards it (the board link is "up": the CMD words are sunk) and
// starts a_render, whose render stream (rs_* beats, taken with random back-pressure) must equal
// tools/gen/as_<seed>_stream.hex (tools/a_raster_ref.py) beat for beat: BAND / TRI / TRISEL / QUAD / FRAME_END.
// TWO FRAMES: the draw's geometry commands are sent again right after the first FLUSH, then a second
// FLUSH: the second frame's FRAME_BEGIN must be taken while the first frame still streams (frame banking), its records
// and bin lists must land in bank 1 (every DDR read of the render side is checked against its frame's bank), and the
// stream must come out twice.
//   +SEED_<n>   (small targets: seeds 11, 12 of tools/gen_a_goldens.sh)
`default_nettype none
module tb_a_render;
    import fb_pkg::*;
    localparam int IDX_W = 20, BANDW = 10;
    localparam [28:0] REC_BASE = 29'h1000000, BINLIST_BASE = 29'h5000000, REC_BANK_W = 29'h2000000, POOL_W = 29'h800000;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    // ingress queue model
    reg [255:0] cmds [0:4095]; int ncmd, qi;
    // +RESIZE: the queue stops behind the other target's VS_STATE (entry rz_hold) until the first frame has streamed, so
    // the state stays wrong for the whole of its walk (a later pass's state in a real frame does)
    int rz_hold = -1; reg rz_gate = 0;
    wire q_valid = (qi < ncmd) && !(rz_gate && qi == rz_hold); wire q_ready; wire [255:0] q_data = cmds[qi];
    always @(posedge clk) if (rst) qi <= 0; else if (q_valid && q_ready) qi <= qi + 1;
    // the dispatcher (the transfer engine idle, the board link up: forwarded words are sunk)
    wire x_cv, x_wdv, x_rdr; wire [1:0] x_op, x_flags; wire [29:0] x_addr, x_len; wire [31:0] x_seed, x_wdd;
    wire ev_req, ev_valid, rd_req, rd_valid, f_valid, f_last; wire [15:0] ev_len, rd_len, f_len; wire [31:0] ev_data, rd_data32, f_data;
    reg  ev_done = 0, rd_done = 0; int ev_i = 0; reg [31:0] ev_w [0:5]; int events = 0, errors_ev = 0, fwd_words = 0;
    wire [31:0] c_cmds, c_errs, last_err, last_err_seq, mt_last, c_fwd;
    wire g_busy, g_frame_done, g_ld_done, g_render_done, g_render, g_frame, g_ld_start, g_param_we, g_ad_we, g_ac_we, g_st_we, g_dr_start;
    wire [28:0] g_ld_addr; wire [29:0] g_ad_base, g_dr_idx_base, g_ad_stride, g_ad_off; wire [11:0] g_ld_len, g_st_w, g_st_h; wire [7:0] g_param_idx, g_st_fsmask;
    wire [95:0] g_param_data, g_ac_const, g_st_vp; wire [3:0] g_ad_idx, g_ac_idx; wire g_ad_en, g_ad_norm, g_st_cw, g_st_z0, g_dr_indexed, g_dr_u8;
    wire [2:0] g_ad_size, g_ad_type, g_dr_mode; wire [31:0] g_dr_first, g_dr_count; wire [1:0] g_st_cull; wire [6:0] g_st_lw; wire [12:0] g_st_did;
    wire [47:0] g_st_z, g_st_sc; wire [95:0] g_st_gb; wire [31:0] c_vis; wire [6*32-1:0] c_cz;
    wire [3:0] g_st_zc; wire g_st_lazy, g_cz_d16; wire [23:0] g_cz_init;
    wire g_rbusy, g_fb_ok, g_draw_ok, g_copy_busy;
    fb_dispatch_a #(.B_WAIT(200)) disp (.clk, .rst, .q_valid, .q_data, .q_ready, .g_render_done, .g_render, .g_rbusy, .g_fb_ok, .g_draw_ok, .g_copy_busy,
        .g_ad_type, .g_ad_norm, .g_st_z, .g_st_gb, .g_st_sc, .g_st_zc, .g_st_lazy, .g_cz_d16, .g_cz_init,
        .x_cmd_valid(x_cv), .x_op, .x_addr, .x_len, .x_seed, .x_flags, .x_cmd_ready(1'b1), .x_busy(1'b0),
        .x_wd_valid(x_wdv), .x_wd_data(x_wdd), .x_wd_ready(1'b1), .x_rd_valid(1'b0), .x_rd_data(32'd0), .x_rd_ready(x_rdr),
        .x_mt_done(1'b0), .x_mt_errors(32'd0), .x_mt_first_bad(30'd0),
        .ev_req, .ev_len, .ev_valid, .ev_data, .ev_ready(1'b1), .ev_done,
        .rd_req, .rd_len, .rd_valid, .rd_data(rd_data32), .rd_ready(1'b1), .rd_done,
        .link_ok(1'b1), .sync_go(1'b0), .f_valid, .f_data, .f_len, .f_last, .f_ready(1'b1),
        .c_cmds, .c_errs, .last_err, .last_err_seq, .mt_last_errors(mt_last), .c_fwd,
        .g_busy, .g_frame_done, .g_ld_done, .g_frame, .g_ld_start, .g_param_we, .g_ad_we, .g_ac_we, .g_st_we, .g_dr_start,
        .g_ld_addr, .g_ad_base, .g_dr_idx_base, .g_ld_len, .g_st_w, .g_st_h, .g_param_idx, .g_st_fsmask, .g_param_data, .g_ac_const, .g_st_vp,
        .g_ad_idx, .g_ac_idx, .g_ad_en, .g_st_cw, .g_st_z0, .g_dr_indexed, .g_dr_u8, .g_ad_size, .g_dr_mode, .g_ad_stride, .g_ad_off, .g_dr_first, .g_dr_count,
        .g_st_cull, .g_st_lw, .g_st_did);
    always @(posedge clk) begin
        ev_done <= 1'b0;
        if (f_valid) fwd_words++;
        if (ev_valid) begin
            ev_w[ev_i] <= ev_data; ev_i <= ev_i + 1;
            if (ev_i == 5) begin ev_done <= 1'b1; ev_i <= 0; events++; end
        end
    end
    always @(posedge clk) if (ev_valid && ev_i == 2 && ev_data == {24'h0, EV_ERROR}) errors_ev <= errors_ev + 1;
    // the geometry stage on mem_arb (3 read, 2 write ports) and the MIG model
    wire [2:0] r_en, r_rdy, r_valid; wire [1:0] w_req, w_en, w_wren, w_wend, w_rdy, w_wdf_rdy; wire [86:0] r_addr; wire [57:0] w_addr; wire [255:0] rd_data;
    wire [511:0] w_data; wire [63:0] w_mask;
    wire app_en, app_wdf_wren, app_wdf_end, app_rdy, app_wdf_rdy, app_rd_valid; wire [2:0] app_cmd; wire [28:0] app_addr;
    wire [255:0] app_wdf_data, app_rd_data; wire [31:0] app_wdf_mask; int mig_errors; longint n_wr, n_rd;
    mem_arb #(.NR(3), .NW(2), .RO_AW(5), .RO_RESERVE(4), .MUST_RUN(3'b000)) arb (.clk, .rst,
        .r_en, .r_addr, .r_rdy, .r_valid, .rd_data, .w_req, .w_en, .w_addr, .w_data, .w_mask, .w_wren, .w_wend, .w_rdy, .w_wdf_rdy,
        .app_en, .app_cmd, .app_addr, .app_wdf_data, .app_wdf_mask, .app_wdf_wren, .app_wdf_end,
        .app_rdy, .app_wdf_rdy, .app_rd_data_valid(app_rd_valid), .app_rd_data);
    mig_app_model #(.RD_LAT(24), .RDY_PCT(70), .WDF_PCT(75)) mig (.ui_clk(clk), .rst, .app_en, .app_cmd, .app_addr, .app_wdf_data,
        .app_wdf_mask, .app_wdf_wren, .app_wdf_end, .app_rdy, .app_wdf_rdy, .app_rd_data, .app_rd_data_valid(app_rd_valid),
        .errors(mig_errors), .n_wr, .n_rd);
    wire err_stack; wire [IDX_W-1:0] tri_count; wire [31:0] c_quads;
    wire rs_valid, rs_ready; wire [63:0] rs_data;
    a_geom #(.REC_BASE(REC_BASE), .BINLIST_BASE(BINLIST_BASE), .REC_BANK_W(REC_BANK_W), .POOL_W(POOL_W)) u_geom (.clk, .rst,
        .g_frame, .g_frame_done, .g_rbusy, .g_fb_ok, .g_draw_ok, .g_copy_busy, .g_ld_start, .g_ld_addr, .g_ld_len, .g_ld_done, .g_param_we, .g_param_idx, .g_param_data,
        .g_ad_we, .g_ad_idx, .g_ad_en, .g_ad_type, .g_ad_norm, .g_ad_base, .g_ad_size, .g_ad_stride, .g_ad_off, .g_ac_we, .g_ac_idx, .g_ac_const,
        .g_st_we, .g_st_vp, .g_st_z, .g_st_gb, .g_st_sc, .g_st_cull, .g_st_cw, .g_st_z0, .g_st_lw, .g_st_fsmask, .g_st_w, .g_st_h, .g_st_did, .c_vis,
        .g_st_zc, .g_st_lazy, .g_cz_d16, .g_cz_init, .c_cz,
        .g_dr_start, .g_dr_mode, .g_dr_indexed, .g_dr_u8, .g_dr_idx_base, .g_dr_first, .g_dr_count, .g_render, .g_render_done, .g_abort(1'b0), .g_abort_soft(1'b0),
        .g_busy, .err_stack, .rec_ovf(), .bin_ovf(), .tri_count, .c_quads,
        .r_en, .r_addr, .r_rdy, .r_valid, .rd_data, .w_req, .w_en, .w_addr, .w_data, .w_mask, .w_wren, .w_wend, .w_rdy, .w_wdf_rdy,
        .rs_valid, .rs_data, .rs_ready);
    // +TRACE: the bank signals, the record writes and the render side's record reads
    always @(posedge clk) if ($test$plusargs("TRACE")) begin
        if (g_frame) $display("TRACE %0t FRAME_BEGIN gbank %b -> %b", $time, u_geom.gbank, u_geom.gbank_c);
        if (g_render) $display("TRACE %0t RENDER go_bank %b", $time, u_geom.gbank);
        if (u_geom.u_set.w_en && u_geom.u_set.w_rdy && u_geom.u_set.bc == 0) $display("TRACE %0t record write tid %0d bank %b addr %h", $time, u_geom.u_set.tid, u_geom.u_set.bank, u_geom.u_set.w_addr);
        if (u_geom.u_rn.r_acc) $display("TRACE %0t walker read kind %0d slot %0d q_bank %b addr %h (outstanding %0d)", $time, u_geom.u_rn.r_kind[1:0],
                                         u_geom.u_rn.r_kind[5:2], u_geom.u_rn.q_bank, u_geom.u_rn.r_addr, u_geom.u_rn.outstanding);
    end
    // +VTRACE: every list entry the fetcher takes and every visit the rasterizer takes (the walk's order)
    always @(posedge clk) if ($test$plusargs("VTRACE")) begin
        if (u_geom.u_rn.fs == u_geom.u_rn.F_QRY2) $display("VT %0t band %0d count %0d head %0h", $time, u_geom.u_rn.fband, u_geom.u_rn.q_count, u_geom.u_rn.q_head);
        if (u_geom.u_rn.rt_ent) $display("VT %0t line in: lane3 next %0h (l_left %0d)", $time, rd_data[224 +: 23], u_geom.u_rn.l_left);
        if (u_geom.u_rn.fs == u_geom.u_rn.F_ENT && u_geom.u_rn.n_left != 0 && u_geom.u_rn.ln_cnt != 0 && !u_geom.u_rn.v_full && u_geom.u_rn.can_issue && !u_geom.u_rn.line_go)
            $display("VT %0t entry band %0d lane %0d: tid %0d first %0d nb %0d (n_left %0d)", $time, u_geom.u_rn.fband, u_geom.u_rn.lane,
                     u_geom.u_rn.ent_cur[19:0], u_geom.u_rn.ent_cur[63], u_geom.u_rn.ent_cur[59:56], u_geom.u_rn.n_left);
        if (u_geom.u_rn.fs == u_geom.u_rn.F_TRI && u_geom.u_rn.r_en == 0 || (u_geom.u_rn.fs == u_geom.u_rn.F_TRI && u_geom.u_rn.r_rdy))
            $display("VT %0t tri issue? tid %0d fbi %0d fnb %0d bq_w %0d bq_r %0d bres %0d free %0d can %b line_go %b", $time, u_geom.u_rn.ftid, u_geom.u_rn.fbi,
                     u_geom.u_rn.fnb, u_geom.u_rn.bq_w, u_geom.u_rn.bq_r, u_geom.u_rn.bres, u_geom.u_rn.bq_free, u_geom.u_rn.can_issue, u_geom.u_rn.line_go);
        if (u_geom.u_rn.r_valid && u_geom.u_rn.fetch_on && u_geom.u_rn.kq_h[1:0] == 2'd2) $display("VT %0t tri burst in: word1 %0d (bq_w %0d)", $time, rd_data[63:32], u_geom.u_rn.bq_w);
        if (u_geom.u_rn.tb_ld) $display("VT %0t tri burst out: word1 %0d (bq_r %0d)", $time, u_geom.u_rn.bq[u_geom.u_rn.bq_r[4:0]][63:32], u_geom.u_rn.bq_r);
        if (u_geom.u_rn.drop_now) $display("VT %0t tri burst dropped: word1 %0d (bq_r %0d, %0d to drop)", $time, u_geom.u_rn.bq[u_geom.u_rn.bq_r[4:0]][63:32], u_geom.u_rn.bq_r, u_geom.u_rn.odrop);
        if (u_geom.u_rn.st == u_geom.u_rn.W_VISIT && !u_geom.u_rn.v_empty && u_geom.u_rn.v_rdy[u_geom.u_rn.vr] && !u_geom.u_rn.soft_pend)
            $display("VT %0t visit kind %0d tid %0d first %0d", $time, u_geom.u_rn.v_kind[u_geom.u_rn.vr], u_geom.u_rn.v_tid[u_geom.u_rn.vr], u_geom.u_rn.v_first[u_geom.u_rn.vr]);
    end
    // +CZTRACE: every coarse-depth decision (as tools/a_raster_ref.py prints them with CZ_TRACE=1)
    always @(posedge clk) if ($test$plusargs("CZTRACE") && u_geom.u_rn.d1_v)
        $display("CZ %0h %0h cov %0h zc %0h ok %0d lo %06h hi %06h val %06h ep %0d cut %0d inv %0d", u_geom.u_rn.d1_qx, u_geom.u_rn.d1_qy,
                 u_geom.u_rn.d1_cov, u_geom.u_rn.d1_zc, u_geom.u_rn.d1_ok, u_geom.u_rn.d1_lo, u_geom.u_rn.d1_hi, u_geom.u_rn.d1_val,
                 u_geom.u_rn.d1_ep, u_geom.u_rn.d1_cut, u_geom.u_rn.d1_inv);
    // the walk's fetch: the most reads in flight, the cycles the rasterizer waited for a visit's header
    int max_out = 0, hdr_wait = 0;
    always @(posedge clk) begin
        if (u_geom.u_rn.outstanding > max_out) max_out = u_geom.u_rn.outstanding;
        if (u_geom.u_rn.st == u_geom.u_rn.W_VISIT && !u_geom.u_rn.v_empty && !u_geom.u_rn.v_rdy[u_geom.u_rn.vr]) hdr_wait++;
    end
    // the stream sink: random back-pressure, every beat compared with the model's in order
    reg [63:0] exp_beats [$]; int nbeat = 0, err = 0, render_done = 0, overlap = 0, bank_bad = 0, frames_begun = 0, rz_mid = 0;
    reg rdy_r = 1; assign rs_ready = rdy_r;
    // +FASTSINK: the stream taken every cycle (the walk's own speed: rn_cyc counts the cycles the render side is busy)
    int rn_cyc = 0; bit fastsink; initial fastsink = $test$plusargs("FASTSINK");
    always @(posedge clk) if (u_geom.rn_busy) rn_cyc++;
    // +RNPROF: the rasterizer's cycles per state while the render side is busy (where the walk's time goes)
    int hw [int];
    always @(posedge clk) if (u_geom.rn_busy && $test$plusargs("RNPROF")) hw[int'(u_geom.u_rn.st)]++;
    final if ($test$plusargs("RNPROF")) begin
        int k; $display("RNPROF rasterizer states (enum order of a_render st_t):");
        if (hw.first(k)) do $display("  st %2d %9d", k, hw[k]); while (hw.next(k));
    end
    always @(posedge clk) begin
        rdy_r <= fastsink || ($urandom % 4 != 0);
        if (g_render_done) begin render_done++; rz_gate <= 1'b0; end
        if (g_frame) begin frames_begun++; if (u_geom.rn_busy) overlap++; end           // a frame begun while the previous one streams
        if (g_st_we && u_geom.rn_busy && (g_st_w != u_geom.scr_w || g_st_h != u_geom.scr_h)) rz_mid++;   // (+RESIZE) another target's state lands while a frame streams
        if (r_en[2] && r_rdy[2]) begin : BANKCHK                                     // the render side reads its frame's bank only
            reg [28:0] a; reg b; a = r_addr[86:58]; b = u_geom.u_rn.q_bank;
            if (!((a >= REC_BASE + (b ? REC_BANK_W : 0) && a < REC_BASE + (b ? REC_BANK_W : 0) + REC_BANK_W) ||
                  (a >= BINLIST_BASE + (b ? POOL_W : 0) && a < BINLIST_BASE + (b ? POOL_W : 0) + POOL_W))) begin
                bank_bad++; if (bank_bad < 5) $display("  render read %h outside bank %0d", a, b);
            end
        end
        if (rs_valid && rs_ready) begin
            if (exp_beats.size() == 0) begin err++; if (err < 10) $display("  beat %0d: %016x beyond the model's stream", nbeat, rs_data); end
            else begin
                reg [63:0] e; e = exp_beats.pop_front();
                if (rs_data !== e) begin err++; if (err < 10) $display("  beat %0d: got %016x expected %016x", nbeat, rs_data, e); end
            end
            nbeat++;
        end
    end

    initial begin
        string seed; int fd, nexp, cyc, cyc_lim, b, k, n_first; reg [31:0] bi; reg [255:0] bd; reg [63:0] bt;
        reg [255:0] flb; int cz_d16, cz_ds, cz_cden, cz_csc, cz_cdepth, cz_lazy; bit cz; int czc [0:5];
        if (!$value$plusargs("SEED_%s", seed)) seed = "11";
        // +CZ (coarse depth / lazy TRI items): the FLUSH carries the frame's depth from as_<seed>_cz.hex (the draws' VS_STATE
        // their flags and the lazy bit, a_setup_ref.py --zc / --lazy), the expected stream is the culled one
        // (as_<seed>_stream_cz.hex) and A's counters must equal the model's (as_<seed>_cz_counts.hex, twice: two frames)
        cz = $test$plusargs("CZ"); flb = {224'h0, 32'h0000_C000};
        if (cz) begin
            fd = $fopen({"tools/gen/as_", seed, "_cz.hex"}, "r");
            if (fd == 0 || $fscanf(fd, "%d %d %d %d %d %d", cz_d16, cz_ds, cz_cden, cz_csc, cz_cdepth, cz_lazy) != 6) begin $display("TB_FAIL tb_a_render: no as_%s_cz.hex", seed); $finish; end
            $fclose(fd);
            flb[31:0] = 32'h0000_C000 | (32'(cz_csc) << 17) | (32'(cz_cden) << 23) | (32'(cz_d16) << 25);
            flb[3*32 +: 32] = 32'(cz_cdepth); flb[6*32 +: 32] = cz_ds ? 32'h0000_C001 : 32'h0;
            fd = $fopen({"tools/gen/as_", seed, "_cz_counts.hex"}, "r");
            if (fd == 0 || $fscanf(fd, "%d %d %d %d %d %d", czc[0], czc[1], czc[2], czc[3], czc[4], czc[5]) != 6) begin $display("TB_FAIL tb_a_render: no as_%s_cz_counts.hex", seed); $finish; end
            $fclose(fd);
        end
        for (k = 0; k < 4096; k++) cmds[k] = 'x;
        $readmemh({"tools/gen/as_", seed, "_cmds.hex"}, cmds);
        ncmd = 0; while (ncmd < 4096 && !$isunknown(cmds[ncmd])) ncmd++;
        // the frame's FLUSH as the host sends it (7 words: {flags, image handle}, clear colour .., {mask, ds handle})
        cmds[ncmd] = {192'h0, 32'(ncmd + 1), 8'h51, T_FLUSH, 16'd28}; cmds[ncmd + 1] = flb; ncmd += 2;
        n_first = ncmd - 2;
        // +RESIZE: right behind the first FLUSH the frame's own VS_STATE again with ANOTHER target, 32 x 16 —
        // the next pass's state arriving while this frame is still being walked, as a render-to-texture pass followed by
        // the screen's does. VS_STATE waits for nothing; a_render must walk the frame with the target its FLUSH found
        // (the size is latched with `go`: read live, the band count would change under the walk). The second frame's own
        // VS_STATE puts the size back before its draw.
        if ($test$plusargs("RESIZE")) begin : RZ
            int i = 0; bit got = 0;
            while (i < n_first && !got) begin
                int nb_ = (cmds[i][15:0] + 31) / 32;
                if (cmds[i][23:16] == 8'h54) begin
                    cmds[ncmd] = cmds[i]; cmds[ncmd][63:32] = 32'(ncmd + 1); ncmd++;
                    for (int j = 1; j <= nb_; j++) begin cmds[ncmd] = cmds[i + j]; ncmd++; end
                    cmds[ncmd - nb_][191:160] = {16'd16, 16'd32};       // word 5 = {height, width}
                    got = 1; rz_hold = ncmd; rz_gate = 1'b1;
                end
                i += 1 + nb_;
            end
            if (!got) begin $display("TB_FAIL tb_a_render: +RESIZE found no VS_STATE"); $finish; end
        end
        // the second frame: the geometry commands (0x51 .. 0x56) of the stream again, then a FLUSH
        begin
            int n1 = n_first, i = 0;
            while (i < n1) begin
                int nb_ = (cmds[i][15:0] + 31) / 32; reg [7:0] ty_ = cmds[i][23:16];
                if (ty_ >= 8'h51 && ty_ <= 8'h56) begin
                    cmds[ncmd] = cmds[i]; cmds[ncmd][63:32] = 32'(ncmd + 1); ncmd++;
                    for (int j = 1; j <= nb_; j++) begin cmds[ncmd] = cmds[i + j]; ncmd++; end
                end
                i += 1 + nb_;
            end
            cmds[ncmd] = {192'h0, 32'(ncmd + 1), 8'h51, T_FLUSH, 16'd28}; cmds[ncmd + 1] = flb; ncmd += 2;
        end
        fd = $fopen({"tools/gen/av_", seed, "_ddr.hex"}, "r");
        if (fd == 0) begin $display("TB_FAIL tb_a_render: no DDR image"); $finish; end
        b = 0; while ($fscanf(fd, "%h %h", bi, bd) == 2) begin mig.mem[longint'(bi)] = bd; b++; end
        $fclose(fd);
        fd = $fopen({"tools/gen/as_", seed, cz ? "_stream_cz.hex" : "_stream.hex"}, "r");
        if (fd == 0) begin $display("TB_FAIL tb_a_render: no stream golden (python3 tools/a_raster_ref.py)"); $finish; end
        while ($fscanf(fd, "%h", bt) == 1) exp_beats.push_back(bt);
        $fclose(fd);
        nexp = exp_beats.size();
        for (k = 0; k < nexp; k++) exp_beats.push_back(exp_beats[k]);     // the second frame: the same stream again
        nexp = exp_beats.size();
        $display("tb_a_render seed %s: %0d commands (two frames), %0d expected beats", seed, ncmd, nexp);
        repeat (4) @(posedge clk); rst <= 0; repeat (40) @(posedge clk);
        cyc = 0;
        // the cycle limit grows with the stream: the sink takes 3 beats in 4 and the walk's own pace is ~1.6 cycles per
        // beat in the 640 x 480 seeds (seed 7: 11.6 M beats, 18.3 M cycles with +FASTSINK; a flat 20 M is too few)
        cyc_lim = (3 * nexp > 20_000_000) ? 3 * nexp : 20_000_000;
        begin : RUN                                               // a stall detector: no beat, command or frame event for 1 M cycles
            int last_prog = 0, last_beat = 0, last_cmds = 0, last_fb = 0, last_rd = 0;
            while (cyc < cyc_lim && render_done < 2) begin
                @(posedge clk); cyc++;
                if (nbeat != last_beat || c_cmds != last_cmds || frames_begun != last_fb || render_done != last_rd) begin
                    last_prog = cyc; last_beat = nbeat; last_cmds = c_cmds; last_fb = frames_begun; last_rd = render_done;
                end
                if (cyc - last_prog > 1_000_000) begin
                    err++;
                    $display("  STALL after %0d cycles: dispatcher st %0d g_busy %b g_rbusy %b g_fb_ok %b g_draw_ok %b copy_busy %b | commands %0d frames begun %0d rendered %0d beats %0d | render st %0d band %0d | feed ist %0d jobs %0d inflight %0d | asm st %0d dq %0d | setup st %0d | binner st %0d busy %b | core idle %b",
                             cyc, disp.st, g_busy, g_rbusy, g_fb_ok, g_draw_ok, g_copy_busy, c_cmds, frames_begun, render_done, nbeat, u_geom.u_rn.st, u_geom.u_rn.band,
                             u_geom.u_vtx.u_feed.ist, u_geom.u_vtx.u_feed.jq_cnt, u_geom.u_vtx.u_feed.inflight, u_geom.u_vtx.u_asm.st, u_geom.u_vtx.u_asm.d_cnt,
                             u_geom.u_set.st, u_geom.u_bin.st, u_geom.b_busy, u_geom.u_vtx.u_core.idle);
                    break;
                end
            end
        end
        repeat (20) @(posedge clk);
        if (render_done < 2) begin
            err++;
            $display("  %0d of 2 frames rendered after %0d cycles (dispatcher st %0d, g_busy %b, commands %0d, errors %0d, render st %0d band %0d beats %0d)",
                     render_done, cyc, disp.st, g_busy, c_cmds, c_errs, u_geom.u_rn.st, u_geom.u_rn.band, nbeat);
        end
        if (frames_begun != 2) begin err++; $display("  %0d frame begins (2 expected)", frames_begun); end
        if (overlap == 0 && !$test$plusargs("RESIZE")) begin err++; $display("  the second frame's FRAME_BEGIN was not taken while the first streamed (no overlap)"); end   // (+RESIZE holds the second frame back)
        if ($test$plusargs("RESIZE")) begin
            $display("  +RESIZE: a VS_STATE of another target landed %0d time(s) while a frame streamed", rz_mid);
            if (rz_mid == 0) begin err++; $display("  +RESIZE proved nothing: the other target's VS_STATE never landed while a frame streamed"); end
        end
        if (bank_bad != 0) begin err++; $display("  %0d render-side reads outside the frame's bank", bank_bad); end
        // (a frame can have no record at all — every triangle culled, an empty rectangle: seeds 19, 50, 52 — and then none in bank 1)
        if (tri_count != 0 && !mig.mem.exists(longint'(REC_BASE + REC_BANK_W) >> 3)) begin err++; $display("  no record in bank 1"); end
        if (errors_ev != 0 || c_errs != 0) begin err++; $display("  %0d error events (dispatcher errors %0d, last %h seq %0d)", errors_ev, c_errs, last_err, last_err_seq); end
        if (exp_beats.size() != 0) begin err++; $display("  %0d beats of the model not produced (%0d of %0d)", exp_beats.size(), nbeat, nexp); end
        if (err_stack) begin err++; $display("  core stack violation"); end
        // the counters (cumulative over the two frames): {blocks lowered, TRIs with no quad, TRIs held back, TRIs sent, quads sent, culled}
        $display("AR counters: culled %0d, quads sent %0d, TRI items sent %0d, held back %0d, sent with no quad %0d, blocks lowered %0d",
                 c_cz[0 +: 32], c_cz[32 +: 32], c_cz[64 +: 32], c_cz[96 +: 32], c_cz[128 +: 32], c_cz[160 +: 32]);
        if (cz) for (int i = 0; i < 6; i++) if (c_cz[32*i +: 32] != 32'(2 * czc[i])) begin
            err++; $display("  counter %0d: %0d, the model's %0d (two frames)", i, c_cz[32*i +: 32], 2 * czc[i]);
        end
        if (mig_errors != 0) begin err++; $display("  MIG model errors %0d", mig_errors); end
        $display("AR seed %s: %0d records, %0d beats (%0d expected, two frames), %0d quads, %0d visits, %0d cycles, %0d DDR reads, %0d forwarded words, overlap %0d, errors = %0d",
                 seed, tri_count, nbeat, nexp, c_quads, c_vis, cyc, n_rd, fwd_words, overlap, err);
        $display("AR fetch: up to %0d reads in flight, the rasterizer waited %0d cycles for headers; render side busy %0d cycles (two frames)",
                 max_out, hdr_wait, rn_cyc);
        if (err == 0) $display("TB_PASS tb_a_render %s", seed); else $display("TB_FAIL tb_a_render %s", seed);
        $finish;
    end
    initial begin #400ms; $display("TB_FAIL tb_a_render: TIMEOUT"); $finish; end
    wire _unused = &{1'b0, x_cv, x_op, x_addr, x_len, x_seed, x_flags, x_wdv, x_wdd, x_rdr, ev_req, ev_len, rd_req, rd_len, rd_valid, rd_data32, f_data, f_len, f_last, mt_last, c_fwd, events};
endmodule
`default_nettype wire
