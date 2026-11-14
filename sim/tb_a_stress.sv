`timescale 1ns/1ps
// tb_a_stress — board A's geometry under a long stream of small draws (tools/a_stress_gen.py: GL 1.x through host/win/gl, the
// shape of Half-Life's frames). The same harness as tb_a_cmd (fb_dispatch_a -> a_geom on mem_arb + the MIG model), no record golden: the check
// is PROGRESS. Every command must be taken and the FENCE answered; a watchdog that sees nothing move (no command taken, no draw
// started or finished, no primitive pushed) for STALL cycles prints the state of every stage of the vertex front end and fails.
// Hazard monitors (each counts, prints its first occurrences as "HAZARD ...", and fails the run):
//   phantom read     the arbiter takes a read of the feed's port that the feed does not count (r_en & r_rdy, neither ix_go nor
//                    lk_go): a standing lookup request granted after the job FIFO has closed, which hangs the vertex front end
//                    (vtx_feed's jq_space_nx prevents it); `hl4 +MIGJIT_80` exercises it
//   stale reference  a result slot taken again (I_NEWSLOT) while the reference the feed still holds for the assembler points at it
//   bank             a warp launching on a uniform bank, or a CONST job reading a descriptor bank, that a later draw has refilled
//   +SEED_<name>     the stream tools/gen/as_<name>_cmds.hex with the DDR image av_<name>_ddr.hex (e.g. SEED_hl1); make the
//                    streams with tools/a_stress_gen.py (its docstring lists the proven set)
//   +STALL_<n>       watchdog cycles (default 200000)
//   +MIGJIT_<n> / +MIGSTALL_<n> / +MIGSEED_<s>   DDR timing variation (sim/mig_app_model.sv)
//   tools/sim_gpu.ps1 -Test tb_a_stress -All -Plusargs SEED_hl4,MIGJIT_80
//                     -Extra src/rtl/proto/fb_pkg.sv,src/rtl/res/buf_table.sv,src/rtl/proto/fb_dispatch_a.sv,src/rtl/mem/mem_arb.sv,sim/mig_app_model.sv
`default_nettype none
module tb_a_stress;
    import fb_pkg::*;
    localparam int IDX_W = 20, QN = 1 << 19;
    localparam [28:0] REC_BASE = 29'h1000000, BINLIST_BASE = 29'h1800000;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    reg [255:0] cmds [0:QN-1]; int ncmd = 0, qi;
    wire q_valid = (qi < ncmd); wire q_ready; wire [255:0] q_data = cmds[qi];
    always @(posedge clk) if (rst) qi <= 0; else if (q_valid && q_ready) qi <= qi + 1;
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
    always @(posedge clk) begin
        ev_done <= 1'b0;
        if (ev_valid) begin
            if (ev_i < 6) ev_w[ev_i] <= ev_data; ev_i <= ev_i + 1;
            if (ev_i == 5) begin ev_done <= 1'b1; events++; end
        end else ev_i <= 0;
    end
    always @(posedge clk) if (ev_valid && ev_i == 3 && ev_w[2] == {24'h0, EV_FENCE} && ev_data == 32'h5EED) fence_seen <= fence_seen + 1;
    always @(posedge clk) if (ev_valid && ev_i == 2 && ev_data == {24'h0, EV_ERROR}) errors_ev <= errors_ev + 1;
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
    a_geom #(.REC_BASE(REC_BASE), .BINLIST_BASE(BINLIST_BASE)) u_geom (.clk, .rst,
        .g_frame, .g_frame_done, .g_rbusy(), .g_fb_ok(), .g_draw_ok, .g_copy_busy, .g_ld_start, .g_ld_addr, .g_ld_len, .g_ld_done, .g_param_we, .g_param_idx, .g_param_data,
        .g_ad_we, .g_ad_idx, .g_ad_en, .g_ad_type, .g_ad_norm, .g_ad_base, .g_ad_size, .g_ad_stride, .g_ad_off, .g_ac_we, .g_ac_idx, .g_ac_const,
        .g_st_we, .g_st_vp, .g_st_z, .g_st_gb, .g_st_sc, .g_st_cull, .g_st_cw, .g_st_z0, .g_st_lw, .g_st_fsmask, .g_st_w, .g_st_h, .g_st_did, .c_vis,
        .g_dr_start, .g_dr_mode, .g_dr_indexed, .g_dr_u8, .g_dr_idx_base, .g_dr_first, .g_dr_count, .g_render(1'b0), .g_render_done(), .g_abort(1'b0), .g_abort_soft(1'b0),
        .g_busy, .err_stack, .rec_ovf(), .bin_ovf(), .tri_count, .c_quads(),
        .r_en, .r_addr, .r_rdy, .r_valid, .rd_data, .w_req, .w_en, .w_addr, .w_data, .w_mask, .w_wren, .w_wend, .w_rdy, .w_wdf_rdy,
        .rs_valid(), .rs_data(), .rs_ready(1'b1));
    // ---- progress: a command taken, a draw started or finished, a primitive pushed to the binner
    longint cyc = 0, last_prog = 0, n_push = 0, n_draw = 0, n_done = 0;
    always @(posedge clk) if (!rst) begin
        cyc <= cyc + 1;
        if ((q_valid && q_ready) || (u_geom.push_valid && u_geom.push_ready) || g_dr_start || u_geom.u_vtx.draw_done) last_prog <= cyc;
        if (u_geom.push_valid && u_geom.push_ready) n_push <= n_push + 1;
        if (g_dr_start) n_draw <= n_draw + 1;
        if (u_geom.u_vtx.draw_done) n_done <= n_done + 1;
        if (cyc % 250000 == 0 && cyc != 0) $display("MON @%0d: %0d / %0d bursts, draws %0d started %0d finished, %0d pushes", cyc, qi, ncmd, n_draw, n_done, n_push);
    end
    // ---- hazard monitors (they count; the run fails on any)
    // (1) STALE REFERENCE: the feed takes a new result slot (I_NEWSLOT) while the reference it has not yet handed to the
    //     assembler (ref_valid: the assembler's FIFO full) points at that very slot. The slot counts references only once they
    //     are pushed, so it reads free; the reference then names a lane of the slot's NEXT warp (a wrong vertex), and the
    //     assembler waits for that warp, which the feed launches only when it gets a lane or the threshold sees one.
    int n_stale = 0, n_ubank = 0, n_dbank = 0, n_phantom = 0;
    // (3) PHANTOM READ: the arbiter takes a read of the feed's port (r_en & r_rdy) that the feed does not count as issued
    //     (neither ix_go nor lk_go): its return-order FIFO then has no entry for it, the return pops the next read's
    //     destination, and every later return lands one place off (wrong bursts in the cache, the last destination pending
    //     for ever)
    always @(posedge clk) if (!rst && r_en[0] && r_rdy[0] && !(u_geom.u_vtx.u_feed.ix_go || u_geom.u_vtx.u_feed.lk_go)) begin
        n_phantom <= n_phantom + 1;
        if (n_phantom < 4) $display("  HAZARD phantom read @%0d: the arbiter took the feed's read (ix_req %b lk_req_r %b lk_act %b jq %0d) that the feed did not count",
                                    cyc, u_geom.u_vtx.u_feed.ix_req, u_geom.u_vtx.u_feed.lk_req_r, u_geom.u_vtx.u_feed.lk_act, u_geom.u_vtx.u_feed.jq_cnt);
    end
    always @(posedge clk) if (!rst && u_geom.u_vtx.u_feed.ist == 3'd6 && u_geom.u_vtx.u_feed.ns_ok && u_geom.u_vtx.u_feed.ref_valid &&
                              u_geom.u_vtx.u_feed.ref_slot == u_geom.u_vtx.u_feed.ns) begin
        n_stale <= n_stale + 1;
        if (n_stale < 4) $display("  HAZARD stale reference @%0d: slot %0d re-allocated while a reference to its lane %0d waits (assembler FIFO %0d, ref_ready %b)",
                                  cyc, u_geom.u_vtx.u_feed.ns, u_geom.u_vtx.u_feed.ref_lane, u_geom.u_vtx.u_asm.f_cnt, u_geom.u_vtx.u_feed.ref_ready);
    end
    // (2) BANKS: draws overlap with two uniform banks (a_vertex: the draw's parity) and two descriptor banks (vtx_feed). Each
    //     bank is given to the draw whose copy fills it; every warp launch (the LAUNCH job: its bank) and every CONST job (its
    //     bank, j_nm) must still find its own draw there. A draw N may start while draw N-2's jobs are still queued (par_busy
    //     sees the core's warps only), which would hand N-2's last warp N's uniforms.
    int av_seq = 0, fd_seq = 0, ubank_own [0:1], dbank_own [0:1];
    int jq_draw [$];
    initial begin ubank_own[0] = -1; ubank_own[1] = -1; dbank_own[0] = -1; dbank_own[1] = -1; end
    always @(posedge clk) if (!rst) begin
        int jd;
        if (u_geom.u_vtx.dr_start && u_geom.u_vtx.dr_ok) begin av_seq <= av_seq + 1; ubank_own[u_geom.u_vtx.dpar] <= av_seq; end
        if (u_geom.u_vtx.u_feed.ist == 3'd0 && u_geom.u_vtx.u_feed.dr_start) begin fd_seq <= fd_seq + 1; dbank_own[u_geom.u_vtx.u_feed.dr_bank] <= fd_seq; end
        if (u_geom.u_vtx.u_feed.jq_push) jq_draw.push_back(fd_seq - 1);            // (the pushing draw: the feed's current one)
        if (u_geom.u_vtx.u_feed.jq_pop) begin
            jd = jq_draw.pop_front();
            if (u_geom.u_vtx.u_feed.p0_launch && ubank_own[u_geom.u_vtx.u_feed.j_ty[1]] != jd) begin
                n_ubank <= n_ubank + 1;
                if (n_ubank < 4) $display("  HAZARD uniform bank @%0d: a warp of draw %0d launches on bank %0d, which holds draw %0d's uniforms",
                                          cyc, jd, u_geom.u_vtx.u_feed.j_ty[1], ubank_own[u_geom.u_vtx.u_feed.j_ty[1]]);
            end
            if (u_geom.u_vtx.u_feed.p0_const && dbank_own[u_geom.u_vtx.u_feed.j_nm] != jd) begin
                n_dbank <= n_dbank + 1;
                if (n_dbank < 4) $display("  HAZARD descriptor bank @%0d: a CONST job of draw %0d reads bank %0d, which holds draw %0d's",
                                          cyc, jd, u_geom.u_vtx.u_feed.j_nm, dbank_own[u_geom.u_vtx.u_feed.j_nm]);
            end
        end
    end
    task dump_state;
        string s; int k;
        $display("  dispatcher st %0d, queue %0d / %0d bursts taken, commands %0d, errors %0d (last %h), g_busy %b, g_draw_ok %b, g_copy_busy %b, draws started %0d / finished %0d",
                 disp.st, qi, ncmd, c_cmds, c_errs, last_err, g_busy, g_draw_ok, g_copy_busy, n_draw, n_done);
        $display("  a_vertex: dr_ok %b cp_run %b f_start %b dpar %b wpar %b busy_warps %b par_busy %b", u_geom.u_vtx.dr_ok, u_geom.u_vtx.cp_run, u_geom.u_vtx.f_start,
                 u_geom.u_vtx.dpar, u_geom.u_vtx.wpar, u_geom.u_vtx.busy_warps, u_geom.u_vtx.par_busy);
        $display("  feed: ist %0d i %0d / count %0d indexed %b cl %0d cs %0d end_pend %b ref_valid %b | jq %0d inflight %0d | ix_req %b lk_req_r %b s0_v %b s1_v %b ga %0d l_ph %b | rq w %0d r %0d",
                 u_geom.u_vtx.u_feed.ist, u_geom.u_vtx.u_feed.i, u_geom.u_vtx.u_feed.count, u_geom.u_vtx.u_feed.indexed, u_geom.u_vtx.u_feed.cl, u_geom.u_vtx.u_feed.cs,
                 u_geom.u_vtx.u_feed.end_pend, u_geom.u_vtx.u_feed.ref_valid, u_geom.u_vtx.u_feed.jq_cnt, u_geom.u_vtx.u_feed.inflight, u_geom.u_vtx.u_feed.ix_req,
                 u_geom.u_vtx.u_feed.lk_req_r, u_geom.u_vtx.u_feed.s0_v, u_geom.u_vtx.u_feed.s1_v, u_geom.u_vtx.u_feed.ga, u_geom.u_vtx.u_feed.l_ph,
                 u_geom.u_vtx.u_feed.rq_wp, u_geom.u_vtx.u_feed.rq_rp);
        s = "";
        for (k = 0; k < 16; k++) s = {s, $sformatf(" %0d", u_geom.u_vtx.u_feed.c_lock[k])};
        $display("  feed cache: c_v %b c_p %b c_cur %b cf_any %b cm_any %b | locks%s | index bursts v %b p %b idx_ready %b | job head kind %0d e0 %0d e1 %0d two %b",
                 u_geom.u_vtx.u_feed.c_v, u_geom.u_vtx.u_feed.c_p, u_geom.u_vtx.u_feed.c_cur, u_geom.u_vtx.u_feed.cf_any, u_geom.u_vtx.u_feed.cm_any, s,
                 u_geom.u_vtx.u_feed.bs_v, u_geom.u_vtx.u_feed.bs_p, u_geom.u_vtx.u_feed.idx_ready,
                 u_geom.u_vtx.u_feed.j_kind, u_geom.u_vtx.u_feed.j_e0, u_geom.u_vtx.u_feed.j_e1, u_geom.u_vtx.u_feed.j_two);
        s = "";
        for (k = 0; k < 16; k++) s = {s, $sformatf(" %0d", u_geom.u_vtx.u_rbuf.rc[k])};
        $display("  rbuf: slot_free %b done %b inflight %b lq %0d | refs%s", u_geom.u_vtx.u_rbuf.slot_free, u_geom.u_vtx.u_rbuf.done, u_geom.u_vtx.u_rbuf.inflight,
                 u_geom.u_vtx.u_rbuf.lq_cnt, s);
        $display("  core: idle %b in_ready %b out_valid %b", u_geom.u_vtx.u_core.idle, u_geom.u_vtx.in_ready, u_geom.u_vtx.u_core.out_valid);
        $display("  asm: fst %0d md %0d consumed %0d / %0d n %0d tn %0d fifo %0d draws queued %0d | F v %b pd %b first %b | V v %b vi %0d vr_act %b | p_run %b p_i %0d pa_req %b pa_grant %b | vout %0d pr_n %0d sg_v %b vo_valid %b vo_ready %b",
                 u_geom.u_vtx.u_asm.fst, u_geom.u_vtx.u_asm.md, u_geom.u_vtx.u_asm.consumed, u_geom.u_vtx.u_asm.count_q, u_geom.u_vtx.u_asm.n, u_geom.u_vtx.u_asm.tn,
                 u_geom.u_vtx.u_asm.f_cnt, u_geom.u_vtx.u_asm.d_cnt, u_geom.u_vtx.u_asm.f_v, u_geom.u_vtx.u_asm.f_pd, u_geom.u_vtx.u_asm.f_first,
                 u_geom.u_vtx.u_asm.v_v, u_geom.u_vtx.u_asm.vi, u_geom.u_vtx.u_asm.vr_act, u_geom.u_vtx.u_asm.p_run, u_geom.u_vtx.u_asm.p_i,
                 u_geom.u_vtx.u_asm.pa_req, u_geom.u_vtx.u_asm.pa_grant, u_geom.u_vtx.u_asm.vout, u_geom.u_vtx.u_asm.pr_n, u_geom.u_vtx.u_asm.sg_v,
                 u_geom.u_vtx.u_asm.vo_valid, u_geom.u_vtx.u_asm.vo_ready);
        $display("  asm refs: F %0d.%0d %0d.%0d %0d.%0d  V %0d.%0d %0d.%0d %0d.%0d  p_ref %0d.%0d cur %0d.%0d",
                 u_geom.u_vtx.u_asm.f_a >> 4, u_geom.u_vtx.u_asm.f_a & 15, u_geom.u_vtx.u_asm.f_b >> 4, u_geom.u_vtx.u_asm.f_b & 15, u_geom.u_vtx.u_asm.f_c >> 4, u_geom.u_vtx.u_asm.f_c & 15,
                 u_geom.u_vtx.u_asm.wa >> 4, u_geom.u_vtx.u_asm.wa & 15, u_geom.u_vtx.u_asm.wb >> 4, u_geom.u_vtx.u_asm.wb & 15, u_geom.u_vtx.u_asm.wc >> 4, u_geom.u_vtx.u_asm.wc & 15,
                 u_geom.u_vtx.u_asm.p_ref >> 4, u_geom.u_vtx.u_asm.p_ref & 15, u_geom.u_vtx.u_asm.cur >> 4, u_geom.u_vtx.u_asm.cur & 15);
        $display("  setup: st %0d busy %b la_out %0d la_cnt %0d la_pend %b vcnt %0d | binner busy %b | loader st %0d | arbiter reads in flight %0d r_en %b",
                 u_geom.u_set.st, u_geom.s_busy, u_geom.u_set.la_out, u_geom.u_set.la_cnt, u_geom.u_set.la_pend, u_geom.u_set.vcnt, u_geom.b_busy, u_geom.ld_st,
                 arb.ro_w - arb.ro_r, r_en);
    endtask
    initial begin
        string seed; int stall_max, err, fd, nb; reg [31:0] bi; reg [255:0] bd;
        if (!$value$plusargs("SEED_%s", seed)) seed = "hl1";
        if (!$value$plusargs("STALL_%d", stall_max)) stall_max = 200000;
        err = 0;
        for (int k = 0; k < QN; k++) cmds[k] = 'x;
        $readmemh({"tools/gen/as_", seed, "_cmds.hex"}, cmds);
        fd = $fopen({"tools/gen/av_", seed, "_ddr.hex"}, "r");
        if (fd == 0) begin $display("TB_FAIL tb_a_stress: no DDR image for %s", seed); $finish; end
        nb = 0;
        while ($fscanf(fd, "%h %h", bi, bd) == 2) begin mig.mem[longint'(bi)] = bd; nb++; end
        $fclose(fd);
        ncmd = 0; while (ncmd < QN && !$isunknown(cmds[ncmd])) ncmd++;
        $display("tb_a_stress: %s: %0d queue bursts, %0d DDR bursts", seed, ncmd, nb);
        repeat (4) @(posedge clk); rst <= 0; repeat (40) @(posedge clk);
        while (fence_seen == 0 && cyc - last_prog < stall_max) @(negedge clk);
        repeat (8) @(posedge clk);
        if (fence_seen == 0) begin
            err++;
            $display("  STALL: nothing moved for %0d cycles (at cycle %0d, last progress %0d)", stall_max, cyc, last_prog);
            dump_state();
        end
        if (errors_ev != 0 || c_errs != 0) begin err++; $display("  %0d error events (dispatcher errors %0d, last %h seq %0d)", errors_ev, c_errs, last_err, last_err_seq); end
        if (g_busy) begin err++; $display("  geometry still busy"); end
        if (err_stack) begin err++; $display("  core stack violation"); end
        if (mig_errors != 0) begin err++; $display("  MIG model errors %0d", mig_errors); end
        if (n_stale || n_ubank || n_dbank || n_phantom) begin
            err++; $display("  hazards: %0d stale references, %0d uniform-bank, %0d descriptor-bank, %0d phantom reads", n_stale, n_ubank, n_dbank, n_phantom);
        end
        $display("AS %s: %0d / %0d bursts taken, %0d commands, %0d draws started / %0d finished, %0d pushes, %0d cycles, %0d DDR reads, %0d writes, %0d events, errors = %0d",
                 seed, qi, ncmd, c_cmds, n_draw, n_done, n_push, cyc, n_rd, n_wr, events, err);
        if (err == 0) $display("TB_PASS tb_a_stress %s", seed); else $display("TB_FAIL tb_a_stress %s", seed);
        $finish;
    end
    wire _unused = &{1'b0, x_cv, x_op, x_addr, x_len, x_seed, x_flags, x_wdv, x_wdd, x_rdr, ev_req, ev_len, rd_req, rd_len, rd_valid, rd_data32, f_valid, f_data, f_len, f_last, mt_last, c_fwd, tri_count, c_vis};
endmodule
`default_nettype wire
