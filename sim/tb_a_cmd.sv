`timescale 1ns/1ps
// tb_a_cmd — board A's 3D command layer: the host commands of a draw (tools/a_setup_ref.py's as_<seed>_cmds.hex, as
// ingress-queue entries) through fb_dispatch_a (buffer table, handle lookups, the geometry command port) into a_geom
// (a_vertex -> a_setup -> tri_binner + the program loader) with mem_arb on the MIG model holding the buffers; the FENCE
// at the end is answered by the dispatcher (no board B here). The records in DDR and the binner pushes must equal
// tools/gen/as_<seed>_rec.hex, as in tb_a_geom.
//   +SEED_<n>
//   +SEED2_<m>  a second scene after the first, WITHOUT a reset in between: its buffers are written over the first's at
//               the same addresses (the host's run-to-run flow on the hardware) and its commands run; its records (in
//               the other frame bank) and pushes must match its golden. A feed cache that is never invalidated
//               would serve the first scene's bursts — this is the test that catches it.
//   defines (sim_gpu.ps1 -Defines): VS_NL8 / VS_NL4 the vertex core's lanes per warp (default 16); VS_ONEB the one-board
//               configuration of the core (NWARP 4, SFU_TAB2 1, INT_DIV 0, SINCOS 0)
`default_nettype none
`ifdef VS_NL4
`define TB_GNLP .VS_NL(4),
`elsif VS_NL8
`define TB_GNLP .VS_NL(8),
`else
`define TB_GNLP
`endif
`ifdef VS_ONEB
`define TB_GCFG .NWARP(4), .VS_SFU_TAB2(1'b1), .VS_INT_DIV(1'b0), .VS_SINCOS(1'b0),
`else
`define TB_GCFG
`endif
module tb_a_cmd;
    import fb_pkg::*;
    localparam int IDX_W = 20, BANDW = 10;
    localparam [28:0] REC_BASE = 29'h1000000, BINLIST_BASE = 29'h1800000;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    // ingress queue model (qrst: the queue restarts for the second scene)
    reg [255:0] cmds [0:4095]; int ncmd = 0, qi; reg qrst = 0;
    wire q_valid = (qi < ncmd); wire q_ready; wire [255:0] q_data = cmds[qi];
    always @(posedge clk) if (rst || qrst) qi <= 0; else if (q_valid && q_ready) qi <= qi + 1;
    // the dispatcher (the transfer engine idle, the board link down: a FENCE is answered here after B_WAIT)
    wire x_cv, x_wdv, x_rdr; wire [1:0] x_op, x_flags; wire [29:0] x_addr, x_len; wire [31:0] x_seed, x_wdd;
    wire ev_req, ev_valid, rd_req, rd_valid, f_valid, f_last; wire [15:0] ev_len, rd_len, f_len; wire [31:0] ev_data, rd_data32, f_data;
    reg  ev_done = 0, rd_done = 0; int ev_i = 0; reg [31:0] ev_w [0:5]; int fence_seen = 0, events = 0, errors_ev = 0;
    wire [31:0] c_cmds, c_errs, last_err, last_err_seq, mt_last, c_fwd;
    wire g_busy, g_frame_done, g_ld_done, g_frame, g_ld_start, g_param_we, g_ad_we, g_ac_we, g_st_we, g_dr_start;
    wire [28:0] g_ld_addr; wire [29:0] g_ad_base, g_dr_idx_base, g_ad_stride, g_ad_off; wire [11:0] g_ld_len, g_st_w, g_st_h; wire [7:0] g_param_idx, g_st_fsmask;
    wire [95:0] g_param_data, g_ac_const, g_st_vp; wire [3:0] g_ad_idx, g_ac_idx; wire g_ad_en, g_ad_norm, g_st_cw, g_st_z0, g_dr_indexed, g_dr_u8;
    wire [2:0] g_ad_size, g_ad_type, g_dr_mode; wire [31:0] g_dr_first, g_dr_count; wire [1:0] g_st_cull; wire [6:0] g_st_lw; wire [12:0] g_st_did;
    wire [47:0] g_st_z, g_st_sc; wire [95:0] g_st_gb; wire [31:0] c_vis;
    wire g_draw_ok, g_copy_busy;
    fb_dispatch_a #(.B_WAIT(200)) disp (.clk, .rst, .q_valid, .q_data, .q_ready, .g_render_done(1'b0), .g_render(), .g_rbusy(1'b0), .g_fb_ok(1'b1), .g_draw_ok, .g_copy_busy,
        .g_ad_type, .g_ad_norm, .g_st_z, .g_st_gb, .g_st_sc,
        .x_cmd_valid(x_cv), .x_op, .x_addr, .x_len, .x_seed, .x_flags, .x_cmd_ready(1'b1), .x_busy(1'b0),
        .x_wd_valid(x_wdv), .x_wd_data(x_wdd), .x_wd_ready(1'b1), .x_rd_valid(1'b0), .x_rd_data(32'd0), .x_rd_ready(x_rdr),
        .x_mt_done(1'b0), .x_mt_errors(32'd0), .x_mt_first_bad(30'd0),
        .ev_req, .ev_len, .ev_valid, .ev_data, .ev_ready(1'b1), .ev_done,
        .rd_req, .rd_len, .rd_valid, .rd_data(rd_data32), .rd_ready(1'b1), .rd_done,
        .link_ok(1'b0), .sync_go(1'b0), .f_valid, .f_data, .f_len, .f_last, .f_ready(1'b1),
        .c_cmds, .c_errs, .last_err, .last_err_seq, .mt_last_errors(mt_last), .c_fwd,
        .g_busy, .g_frame_done, .g_ld_done, .g_frame, .g_ld_start, .g_param_we, .g_ad_we, .g_ac_we, .g_st_we, .g_dr_start,
        .g_ld_addr, .g_ad_base, .g_dr_idx_base, .g_ld_len, .g_st_w, .g_st_h, .g_param_idx, .g_st_fsmask, .g_param_data, .g_ac_const, .g_st_vp,
        .g_ad_idx, .g_ac_idx, .g_ad_en, .g_st_cw, .g_st_z0, .g_dr_indexed, .g_dr_u8, .g_ad_size, .g_dr_mode, .g_ad_stride, .g_ad_off, .g_dr_first, .g_dr_count,
        .g_st_cull, .g_st_lw, .g_st_did);
    // EVENT reply channel model: take the 6 words, then done
    always @(posedge clk) begin
        ev_done <= 1'b0;
        if (ev_valid) begin
            if (ev_i < 6) ev_w[ev_i] <= ev_data; ev_i <= ev_i + 1;
            if (ev_i == 5) begin ev_done <= 1'b1; events++; end
        end else ev_i <= 0;                           // (ev_valid may outlast the 6 words by a cycle: resync between events)
    end
    always @(posedge clk) if (ev_valid && ev_i == 3 && ev_w[2] == {24'h0, EV_FENCE} && ev_data == 32'h5EED) fence_seen <= fence_seen + 1;
    always @(posedge clk) if (ev_valid && ev_i == 2 && ev_data == {24'h0, EV_ERROR}) errors_ev <= errors_ev + 1;
    // the geometry stage on mem_arb (2 read, 2 write ports) and the MIG model
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
    wire err_stack; wire [IDX_W-1:0] tri_count;
    a_geom #(`TB_GNLP `TB_GCFG .REC_BASE(REC_BASE), .BINLIST_BASE(BINLIST_BASE)) u_geom (.clk, .rst,
        .g_frame, .g_frame_done, .g_rbusy(), .g_fb_ok(), .g_draw_ok, .g_copy_busy, .g_ld_start, .g_ld_addr, .g_ld_len, .g_ld_done, .g_param_we, .g_param_idx, .g_param_data,
        .g_ad_we, .g_ad_idx, .g_ad_en, .g_ad_type, .g_ad_norm, .g_ad_base, .g_ad_size, .g_ad_stride, .g_ad_off, .g_ac_we, .g_ac_idx, .g_ac_const,
        .g_st_we, .g_st_vp, .g_st_z, .g_st_gb, .g_st_sc, .g_st_cull, .g_st_cw, .g_st_z0, .g_st_lw, .g_st_fsmask, .g_st_w, .g_st_h, .g_st_did, .c_vis,
        .g_dr_start, .g_dr_mode, .g_dr_indexed, .g_dr_u8, .g_dr_idx_base, .g_dr_first, .g_dr_count, .g_render(1'b0), .g_render_done(), .g_abort(1'b0), .g_abort_soft(1'b0),
        .g_busy, .err_stack, .rec_ovf(), .bin_ovf(), .tri_count, .c_quads(),
        .r_en, .r_addr, .r_rdy, .r_valid, .rd_data, .w_req, .w_en, .w_addr, .w_data, .w_mask, .w_wren, .w_wend, .w_rdy, .w_wdf_rdy,
        .rs_valid(), .rs_data(), .rs_ready(1'b1));

    int exp_id [0:4095], exp_lo [0:4095], exp_hi [0:4095], exp_n [0:4095]; reg [31:0] exp_w [0:4095][0:111];   // records of up to 106 words (26 + 10 per read varying)
    int nexp;
    // a scene's files: its commands into the (restarted) ingress queue, its buffers over the MIG model's memory (the
    // bursts of a previous scene it does not cover stay, as on the hardware), its record golden
    task load_seed(input string seed);               // static: xsim's $fscanf does not fill automatic variables
        int fd, k, n, nb; reg [31:0] bi; reg [255:0] bd; int v_id, v_lo, v_hi, v_n, v_w;
        ncmd = 0; @(negedge clk); qrst <= 1'b1; @(negedge clk); qrst <= 1'b0;
        for (k = 0; k < 4096; k++) cmds[k] = 'x;
        $readmemh({"tools/gen/as_", seed, "_cmds.hex"}, cmds);
        fd = $fopen({"tools/gen/av_", seed, "_ddr.hex"}, "r");
        if (fd == 0) begin $display("TB_FAIL tb_a_cmd: no DDR image for seed %s", seed); $finish; end
        nb = 0;
        while ($fscanf(fd, "%h %h", bi, bd) == 2) begin mig.mem[longint'(bi)] = bd; nb++; end
        $fclose(fd);
        fd = $fopen({"tools/gen/as_", seed, "_rec.hex"}, "r");
        if (fd == 0) begin $display("TB_FAIL tb_a_cmd: no record golden for seed %s", seed); $finish; end
        nexp = 0;
        while ($fscanf(fd, "%h %h %h %h", v_id, v_lo, v_hi, v_n) == 4) begin
            exp_id[nexp] = v_id; exp_lo[nexp] = v_lo; exp_hi[nexp] = v_hi; exp_n[nexp] = v_n;
            for (k = 0; k < v_n; k++) begin void'($fscanf(fd, "%h", v_w)); exp_w[nexp][k] = v_w; end
            nexp++;
        end
        $fclose(fd);
        n = 0; while (n < 4096 && !$isunknown(cmds[n])) n++;
        $display("tb_a_cmd: seed %s loaded: %0d queue bursts, %0d DDR bursts (%0d in the model), %0d records expected", seed, n, nb, mig.mem.size(), nexp);
        @(negedge clk); ncmd = n;
    endtask
    // run the loaded scene to its FENCE; the pushes in order, then the records at rbase (the scene's frame bank)
    task run_check(input string seed, input longint rbase, inout int err);
        int npush, cyc, k, fb, cmds0; longint rd0, wr0;
        fb = fence_seen; cmds0 = c_cmds; rd0 = n_rd; wr0 = n_wr; npush = 0; cyc = 0;
        while (cyc < 3_000_000 && fence_seen == fb) begin
            @(negedge clk); cyc++;
            if (u_geom.push_valid && u_geom.push_ready) begin
                if (npush < nexp) begin
                    if (u_geom.push_tri != exp_id[npush] || u_geom.push_lo != exp_lo[npush] || u_geom.push_hi != exp_hi[npush]) begin
                        err++; if (err < 8) $display("  seed %s push %0d: id %0d bands %0d..%0d, expected id %0d bands %0d..%0d", seed, npush, u_geom.push_tri, u_geom.push_lo, u_geom.push_hi, exp_id[npush], exp_lo[npush], exp_hi[npush]);
                    end
                end else begin err++; if (err < 8) $display("  seed %s extra push %0d (id %0d)", seed, npush, u_geom.push_tri); end
                npush++;
            end
        end
        repeat (8) @(posedge clk);
        if (fence_seen == fb) begin
            err++;
            $display("  no FENCE event after %0d cycles (dispatcher st %0d, g_busy %b, commands %0d, errors %0d, last %h)", cyc, disp.st, g_busy, c_cmds, c_errs, last_err);
            $display("  feed st %0d i %0d cl %0d cs %0d r_en %b ref_valid %b in_ready %b | asm st %0d consumed %0d fifo %0d | core idle %b out_valid %b | setup st %0d busy %b | binner busy %b | loader st %0d | dr_busy %b",
                     u_geom.u_vtx.u_feed.ist, u_geom.u_vtx.u_feed.i, u_geom.u_vtx.u_feed.cl, u_geom.u_vtx.u_feed.cs, u_geom.u_vtx.u_feed.r_en,
                     u_geom.u_vtx.u_feed.ref_valid, u_geom.u_vtx.in_ready, u_geom.u_vtx.u_asm.st, u_geom.u_vtx.u_asm.consumed, u_geom.u_vtx.u_asm.f_cnt,
                     u_geom.u_vtx.u_core.idle, u_geom.u_vtx.u_core.out_valid, u_geom.u_set.st, u_geom.s_busy, u_geom.b_busy, u_geom.ld_st, u_geom.dr_busy);
        end
        if (errors_ev != 0 || c_errs != 0) begin err++; $display("  %0d error events (dispatcher errors %0d, last %h seq %0d)", errors_ev, c_errs, last_err, last_err_seq); end
        if (npush != nexp) begin err++; $display("  seed %s: %0d pushes, expected %0d", seed, npush, nexp); end
        if (g_busy) begin err++; $display("  geometry still busy"); end
        for (k = 0; k < nexp; k++) begin
            int bad; bad = 0;
            for (int w = 0; w < exp_n[k]; w++) begin
                longint wa; reg [255:0] burst; reg [31:0] got;
                wa = rbase + exp_id[k] * 128 + w;
                burst = mig.mem.exists(wa >> 3) ? mig.mem[wa >> 3] : 256'd0;
                got = burst[(wa & 7) * 32 +: 32];
                if (got !== exp_w[k][w]) begin bad++; if (err + bad < 10) $display("  seed %s record %0d word %0d: got %08x expected %08x", seed, exp_id[k], w, got, exp_w[k][w]); end
            end
            if (bad) err++;
        end
        $display("AC seed %s: %0d commands, %0d records / pushes (%0d expected) in %0d cycles, %0d DDR reads, %0d writes, %0d events, errors = %0d",
                 seed, c_cmds - cmds0, npush, nexp, cyc, n_rd - rd0, n_wr - wr0, events, err);
    endtask
    initial begin
        string seed, seed2, tag; int err;
        if (!$value$plusargs("SEED_%s", seed)) seed = "1";
        err = 0; tag = seed;
        load_seed(seed);
        repeat (4) @(posedge clk); rst <= 0; repeat (40) @(posedge clk);
        run_check(seed, longint'(REC_BASE), err);
        if ($value$plusargs("SEED2_%s", seed2)) begin
            // the second scene over the first, no reset: its FRAME_BEGIN takes the other bank
            load_seed(seed2); repeat (40) @(posedge clk);
            run_check(seed2, longint'(REC_BASE) + longint'(u_geom.REC_BANK_W), err);
            tag = {seed, "+", seed2};
        end
        if (err_stack) begin err++; $display("  core stack violation"); end
        if (mig_errors != 0) begin err++; $display("  MIG model errors %0d", mig_errors); end
        if (err == 0) $display("TB_PASS tb_a_cmd %s", tag); else $display("TB_FAIL tb_a_cmd %s", tag);
        $finish;
    end
    initial begin #80ms; $display("TB_FAIL tb_a_cmd: TIMEOUT"); $finish; end
    wire _unused = &{1'b0, x_cv, x_op, x_addr, x_len, x_seed, x_flags, x_wdv, x_wdd, x_rdr, ev_req, ev_len, rd_req, rd_len, rd_valid, rd_data32, f_valid, f_data, f_len, f_last, mt_last, c_fwd, tri_count};
endmodule
`default_nettype wire
