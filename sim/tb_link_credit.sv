`timescale 1ns/1ps
// tb_link_credit — the board link's credit flow control across board restarts:
// link_core role A + link_a_port and link_core role B + link_b_port, joined by the JM1/JM2 cables of
// tb_two_boards (per-wire delays, two oscillators). A random CMD packet source feeds A (2..400 words, word i of
// packet p = {p, i}); a consumer with random and long stalls takes B's command stream. Checks:
//   * safety: B's command buffer never refuses a word, and while both sides are synchronised A's words in flight
//     (sent - reported) are never fewer than B's buffer occupancy (the XOFF safety net stays unused);
//   * exactness: right after A synchronises with a stalled B, sent - reported = B's occupancy;
//   * stream: whole packets, consecutive words, ids increasing by 1, except one gap per A restart (the packets
//     lost with it) and a fresh start of the sequence after a B restart (its buffer is gone, the ARQ re-delivers);
//   * liveness: packets keep arriving after every case;
//   * at the end: A's sent = A's reported = B's taken = B's received, A synchronised, no unknown packets.
// Cases: start-up; A restarts while B stalls with a full buffer; A restarts while B keeps reporting credit (stale
// CREDITs with the new A's first SYNC id); B restarts (link + port) under a streaming A;
// A restarts, sends ~3800 words, then B restarts (a wrong resync rule leaves a permanent offset here); B's port alone
// restarts with the link up (its DDR calibration later than the link); B restarts again right after it took the
// SYNC (its answer is lost); B retrains mid-stream (no restart: no SYNC, no gap).
// Prints TB_PASS / TB_FAIL.
`default_nettype none
module tb_link_credit;
    import fb_pkg::*;
    reg clkA = 0, clk200A = 0, clkB = 0, clk200B = 0;
    always #5.0    clkA = ~clkA;
    always #2.5    clk200A = ~clk200A;
    always #4.9998 clkB = ~clkB;                  // board B's oscillator is 40 ppm fast
    always #2.4999 clk200B = ~clk200B;
    reg  rstA = 1, rstB = 1, rstBp = 0;           // rstBp: B's port alone, the link stays up
    int  errors = 0;
    // the first failure ends the run (a failing case would otherwise wait out every later timeout); +KEEPGOING continues
    task automatic fail(input string m);
        errors++; $display("FAIL: %s (t=%0t)", m, $time);
        if (!$test$plusargs("KEEPGOING")) begin $display("TB_FAIL tb_link_credit errors=%0d", errors); $finish; end
    endtask

    // ---------------- cables (tb_board_link); a board that is off (restarting) leaves its pins pulled low ----------------
    wire [31:0] cab1A, cab1B, cab2A, cab2B;
    reg  [31:0] cab1B_r = 0, cab2A_r = 0, cab2B_r = 0;
    bit         a_off = 1'b0, b_off = 1'b0;
    wire [31:0] cab1A_e = a_off ? 32'h0 : cab1A, cab2A_e = a_off ? 32'h0 : cab2A, cab2B_e = b_off ? 32'h0 : cab2B;
    real d1 [0:31], d2 [0:31];
    genvar g;
    generate
        for (g = 0; g < 32; g++) begin : g_c1          // JM1: A drives
            pulldown (cab1A[g]);
            assign cab1B[g] = cab1B_r[g];
            always @(cab1A_e[g]) cab1B_r[g] <= #(d1[g]) cab1A_e[g];
        end
        for (g = 0; g < 22; g++) begin : g_c2b         // JM2[21:0]: B drives
            pulldown (cab2B[g]);
            assign cab2A[g] = cab2A_r[g];
            always @(cab2B_e[g]) cab2A_r[g] <= #(d2[g]) cab2B_e[g];
        end
        for (g = 22; g < 32; g++) begin : g_c2a        // JM2[31:22]: A drives
            pulldown (cab2A[g]);
            assign cab2B[g] = cab2B_r[g];
            always @(cab2A_e[g]) cab2B_r[g] <= #(d2[g]) cab2A_e[g];
        end
    endgenerate
    initial begin
        for (int k = 0; k < 32; k++) begin
            d1[k] = 1.0 + 0.0137 * ((k * 7) % 16);
            d2[k] = (k < 22) ? 2.3 + 0.0137 * ((k * 5) % 16) : 1.0 + 0.0137 * ((k * 3) % 16);
        end
        d1[20] = 2.6;                                   // forward clock late against its data
        d2[20] = 1.0;                                   // backward clock early against its data
    end

    // ---------------- board A: link_core role A + link_a_port (board_a_core's parameters in tb_two_boards) ----------------
    wire        lclkA, lclk90A, lclk270A, rst_lclkA;
    link_clk #(.LINK_MHZ(100)) u_clkA (.clk200(clk200A), .rst(rstA), .lclk(lclkA), .lclk90(lclk90A), .lclk270(lclk270A),
        .rst_lclk(rst_lclkA), .locked());
    wire        a_tx_valid, a_tx_cnt2, a_tx_last, a_tx_ready, a_rx_valid, a_rx_cnt2, a_rx_last, a_rx_hdr, a_rx_ready;
    wire [63:0] a_tx_data, a_rx_data;
    link_core #(.ROLE("A"), .LINK_MHZ(100), .MMCM_RST_CYCLES(20_000), .GUARD_CYCLES(3000), .IDLE_TIMEOUT(200_000),
                .TRAIN_WORDS(256), .RETRY_CYCLES(20_000), .RELOCK_CYCLES(50_000), .RETRAIN_WINDOW(200_000),
                .SIM_SHIFTER(1'b1)) lka (
        .clk(clkA), .rst(rstA), .lclk(lclkA), .lclk90(lclk90A), .lclk270(lclk270A), .rst_lclk(rst_lclkA), .fill_en(1'b1),
        .tx_valid(a_tx_valid), .tx_data(a_tx_data), .tx_cnt2(a_tx_cnt2), .tx_last(a_tx_last), .tx_ready(a_tx_ready),
        .rx_valid(a_rx_valid), .rx_data(a_rx_data), .rx_cnt2(a_rx_cnt2), .rx_last(a_rx_last), .rx_hdr(a_rx_hdr),
        .rx_ready(a_rx_ready), .pkt_done(), .pkt_ok(),
        .lock(), .trained(), .rx_clk_ok(), .far_up(), .training(), .guard_fail(), .tx_enabled(), .xoff_events(),
        .eye_width(), .phase(), .train_cnt(), .tx_pkts(), .rx_words(), .rx_pkts(), .par_errs(), .crc_errs(), .seq_errs(),
        .frm_errs(), .lock_drops(), .par_map(), .retx(), .disc(), .diag(), .jm1(cab1A), .jm2(cab2A));
    reg         f_valid = 1'b0, f_last = 1'b0;
    reg  [31:0] f_data = 32'h0;
    reg  [15:0] f_len = 16'h0;
    wire        f_ready;
    wire [31:0] a_sent, a_consumed, a_c_sync, a_unk;
    link_a_port #(.IDLE_CYCLES(20_000), .ALIVE_CYCLES(100_000)) la (
        .clk(clkA), .rst(rstA),
        .tx_valid(a_tx_valid), .tx_data(a_tx_data), .tx_cnt2(a_tx_cnt2), .tx_last(a_tx_last), .tx_ready(a_tx_ready),
        .rx_valid(a_rx_valid), .rx_data(a_rx_data), .rx_last(a_rx_last), .rx_hdr(a_rx_hdr), .rx_ready(a_rx_ready),
        .f_valid(f_valid), .f_data(f_data), .f_len(f_len), .f_last(f_last), .f_ready(f_ready),
        .rs_valid(1'b0), .rs_data(64'h0), .rs_ready(), .rs_abort(1'b0), .resync(1'b0), .sync_go(),
        .cur_valid(1'b0), .cur_pos(32'h0),
        .ev_req(), .ev_len(), .ev_valid(), .ev_data(), .ev_ready(1'b1), .ev_done(1'b0),
        .rd_req(), .rd_len(), .rd_valid(), .rd_data(), .rd_ready(1'b1), .rd_done(1'b0),
        .b_alive(), .b_stat(), .c_bstat(), .c_bev(), .c_unk(a_unk), .c_brd(),
        .sent(a_sent), .consumed(a_consumed), .c_cur_sent(), .c_credit_wait(), .c_sync(a_c_sync), .rs_sent(), .rs_consumed(), .c_rs_pkts());

    // ---------------- board B: link_core role B + link_b_port ----------------
    wire        lclkB, lclk90B, lclk270B, rst_lclkB;
    link_clk #(.LINK_MHZ(100)) u_clkB (.clk200(clk200B), .rst(rstB), .lclk(lclkB), .lclk90(lclk90B), .lclk270(lclk270B),
        .rst_lclk(rst_lclkB), .locked());
    wire        b_tx_valid, b_tx_cnt2, b_tx_last, b_tx_ready, b_rx_valid, b_rx_cnt2, b_rx_last, b_rx_hdr, b_rx_ready;
    wire [63:0] b_tx_data, b_rx_data;
    wire [7:0]  b_train_cnt;
    link_core #(.ROLE("B"), .LINK_MHZ(100), .MMCM_RST_CYCLES(20_000), .GUARD_CYCLES(3000), .IDLE_TIMEOUT(200_000),
                .TRAIN_WORDS(256), .RETRY_CYCLES(20_000), .RELOCK_CYCLES(50_000), .RETRAIN_WINDOW(200_000),
                .SIM_SHIFTER(1'b1)) lkb (
        .clk(clkB), .rst(rstB), .lclk(lclkB), .lclk90(lclk90B), .lclk270(lclk270B), .rst_lclk(rst_lclkB), .fill_en(1'b1),
        .tx_valid(b_tx_valid), .tx_data(b_tx_data), .tx_cnt2(b_tx_cnt2), .tx_last(b_tx_last), .tx_ready(b_tx_ready),
        .rx_valid(b_rx_valid), .rx_data(b_rx_data), .rx_cnt2(b_rx_cnt2), .rx_last(b_rx_last), .rx_hdr(b_rx_hdr),
        .rx_ready(b_rx_ready), .pkt_done(), .pkt_ok(),
        .lock(), .trained(), .rx_clk_ok(), .far_up(), .training(), .guard_fail(), .tx_enabled(), .xoff_events(),
        .eye_width(), .phase(), .train_cnt(b_train_cnt), .tx_pkts(), .rx_words(), .rx_pkts(), .par_errs(), .crc_errs(),
        .seq_errs(), .frm_errs(), .lock_drops(), .par_map(), .retx(), .disc(), .diag(), .jm1(cab1B), .jm2(cab2B));
    wire        c_valid, c_first, c_last;
    wire [31:0] c_data;
    reg         c_ready = 1'b0;
    wire [31:0] b_consumed, b_c_sync, b_unk;
    link_b_port #(.STATUS_CYCLES(10_000)) lb (
        .clk(clkB), .rst(rstB | rstBp),
        .tx_valid(b_tx_valid), .tx_data(b_tx_data), .tx_cnt2(b_tx_cnt2), .tx_last(b_tx_last), .tx_ready(b_tx_ready),
        .rx_valid(b_rx_valid), .rx_data(b_rx_data), .rx_cnt2(b_rx_cnt2), .rx_last(b_rx_last), .rx_hdr(b_rx_hdr),
        .rx_ready(b_rx_ready),
        .c_valid(c_valid), .c_data(c_data), .c_first(c_first), .c_last(c_last), .c_ready(c_ready),
        .rs_take2(1'b0), .rs_valid2(), .rs_data2(),
        .e_valid(1'b0), .e_data(160'h0), .e_ready(),
        .r_req(1'b0), .r_words(16'h0), .r_valid(1'b0), .r_data(32'h0), .r_ready(),
        .fp_valid(), .fp_x(), .fp_y(),
        .stat({B_LSTAT_WORDS{32'h0}}), .c_cmd_pkts(), .c_unk(b_unk), .c_cur_pkts(), .consumed(b_consumed),
        .c_sync(b_c_sync));

    function automatic int unsigned xs32(inout int unsigned s);
        s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s;
    endfunction

    // ---------------- A: CMD packet source (stops only at a packet boundary) ----------------
    int unsigned rs_a = 32'h1234_5678, rs_b = 32'h9ABC_DEF1;
    int          lens [int];                      // packet id -> words
    int          src_p = 1, src_i = 0;
    bit          src_on = 1'b0;
    always @(posedge clkA) begin
        if (rstA) begin                           // A restarts: the packet in progress is abandoned
            if (src_i != 0) begin src_p = src_p + 1; src_i = 0; end
            f_valid <= 1'b0;
        end else begin
            if (f_valid && f_ready) begin
                src_i = src_i + 1;
                if (src_i == lens[src_p]) begin src_p = src_p + 1; src_i = 0; end
            end
            if (src_i == 0 && !lens.exists(src_p)) lens[src_p] = 2 + xs32(rs_a) % 399;
            if (src_on || src_i != 0) begin
                f_valid <= 1'b1; f_data <= {16'(src_p), 16'(src_i)}; f_len <= 16'(lens[src_p]);
                f_last <= (src_i == lens[src_p] - 1);
            end else f_valid <= 1'b0;
        end
    end

    // ---------------- B: command consumer and stream checks ----------------
    bit  stall = 1'b0;
    int  ready_pct = 90;
    int  last_p = 0, cur_p = 0, exp_i = 0, n_pkts = 0, n_gaps = 0;
    bit  have_last = 1'b0, gap_ok = 1'b0, in_pkt = 1'b0;
    task automatic take_word(input [31:0] d, input bit first, input bit last);
        int p, i;
        p = d[31:16]; i = d[15:0];
        if (first) begin
            if (in_pkt) fail($sformatf("packet %0d cut after %0d words by packet %0d", cur_p, exp_i, p));
            if (i != 0) fail($sformatf("packet %0d starts at word %0d", p, i));
            if (have_last && p != last_p + 1) begin
                if (p > last_p && gap_ok) begin n_gaps++; gap_ok = 1'b0; end
                else fail($sformatf("packet %0d after %0d (gap allowed %b)", p, last_p, gap_ok));
            end
            cur_p = p; exp_i = 0; in_pkt = 1'b1;
        end else if (!in_pkt) begin
            fail($sformatf("word %08h outside a packet", d)); return;
        end
        if (p != cur_p || i != exp_i) fail($sformatf("word %08h in packet %0d, expected word %0d", d, cur_p, exp_i));
        exp_i++;
        if (last != (exp_i == lens[cur_p])) fail($sformatf("packet %0d: last %b at word %0d of %0d", cur_p, last, exp_i, lens[cur_p]));
        if (last) begin in_pkt = 1'b0; last_p = cur_p; have_last = 1'b1; n_pkts++; end
    endtask
    always @(posedge clkB) begin
        if (rstB || rstBp) begin in_pkt = 1'b0; c_ready <= 1'b0; end
        else begin
            if (c_valid && c_ready) take_word(c_data, c_first, c_last);
            c_ready <= !stall && (xs32(rs_b) % 100 < ready_pct);
        end
    end

    // ---------------- safety monitors ----------------
    int full_hits = 0, inv_errs = 0;
    always @(posedge clkB) if (!rstB && !rstBp && lb.w_avail && lb.cb_full) begin
        if (full_hits < 3) fail("B's command buffer refused a word (credit above its free space)");
        full_hits++;
    end
    // A's words in flight (sent - reported) >= B's occupancy, once B counts in A's origin (same SYNC id)
    always @(posedge clkA) if (!rstA && !rstB && !rstBp && la.synced && la.sync_id != 0 && lb.sync_id == la.sync_id)
        if ($signed((a_sent - a_consumed) - (lb.rcvd - b_consumed)) < 0) begin
            if (inv_errs < 3) fail($sformatf("credit: A sent %0d reported %0d, B received %0d taken %0d",
                                             a_sent, a_consumed, lb.rcvd, b_consumed));
            inv_errs++;
        end

    // +DBG: non-CMD packets at both ports, SYNCs and CREDITs, link state
    bit dbg; int dbgf;
    initial begin dbg = $test$plusargs("DBG"); if (dbg) dbgf = $fopen("credit_dbg.log", "w"); end
    always @(posedge clkB) if (dbg && !rstB && b_rx_valid && b_rx_ready && b_rx_hdr && b_rx_data[31:24] != L_CMD && b_rx_data[31:24] != L_IDLE)
        $fdisplay(dbgf, "%t DBG B rx hdr %02h cnt2 %b last %b hi %08h | hi_turn %b in_cmd %b occ %0d", $time, b_rx_data[31:24],
                 b_rx_cnt2, b_rx_last, b_rx_data[63:32], lb.hi_turn, lb.in_cmd, lb.rcvd - b_consumed);
    always @(posedge clkB) if (dbg && lb.sync_ev) $fdisplay(dbgf, "%t DBG B took SYNC id %0d: rcvd %0d consumed %0d", $time, lb.sync_id, lb.rcvd, b_consumed);
    always @(posedge clkB) if (dbg && b_tx_valid && b_tx_ready && lb.xs == 4) $fdisplay(dbgf, "%t DBG B tx CREDIT word %08h", $time, b_tx_data[31:0]);
    always @(posedge clkA) if (dbg && a_tx_valid && a_tx_ready && a_tx_data[31:24] == L_SYNC && a_tx_cnt2 && !a_tx_last)
        $fdisplay(dbgf, "%t DBG A tx SYNC id %0d", $time, a_tx_data[38:32]);
    always @(posedge clkA) if (dbg && a_rx_valid && a_rx_ready && a_rx_hdr && a_rx_data[31:24] != L_STATUS)
        $fdisplay(dbgf, "%t DBG A rx hdr %08h rs %0d", $time, a_rx_data[31:0], la.rs);
    always @(la.ys) if (dbg) $fdisplay(dbgf, "%t DBG A ys %0d", $time, la.ys);
    always @(posedge clkA) if (dbg && a_rx_valid && a_rx_ready && la.rs == 7 && la.ri == 1)
        $fdisplay(dbgf, "%t DBG A rx CREDIT {%02h} %0d (ys %0d sync_id %0d)", $time, la.cr_h, a_rx_data[31:0], la.ys, la.sync_id);
    always @(lka.trained or lka.far_up or lkb.trained or lkb.far_up)
        if (dbg) $fdisplay(dbgf, "%t DBG link A trained %b far_up %b | B trained %b far_up %b", $time, lka.trained, lka.far_up, lkb.trained, lkb.far_up);

    // ---------------- helpers ----------------
    task automatic wait_sync(input int n, input int c0, input string what, input int tmo_us = 3000);
        int t = 0;
        while (!(la.synced && a_c_sync >= c0 + n) && t < tmo_us * 100) begin @(posedge clkA); t++; end
        if (!(la.synced && a_c_sync >= c0 + n))
            fail($sformatf("%s: A synced %b after %0d of %0d SYNCs", what, la.synced, a_c_sync - c0, n));
    endtask
    task automatic wait_pkts(input int n, input string what, input int tmo_us = 6000);
        int n0, t;
        n0 = n_pkts; t = 0;
        while (n_pkts < n0 + n && t < tmo_us * 100) begin @(posedge clkA); t++; end
        if (n_pkts < n0 + n) fail($sformatf("%s: %0d of %0d packets in %0d us", what, n_pkts - n0, n, tmo_us));
    endtask
    // A board restart as on hardware (reprogramming, or the reset button, which also resets the MIG and the link
    // clock): the board's pins float (pulled low) while it is in reset, so the far receiver loses its recovered clock
    // and retrains, which re-arms its boundary-word detection. On hardware that takes milliseconds or more; the
    // bench's receive-clock model (link_rx_shifter_sim) does not notice a stopped far clock as the MMCM does, so the
    // bench applies that consequence itself with a retrain pulse and holds the reset for only 50 us. (A reset of a
    // few cycles with the clocks running and the far receiver still trained is not handled by the link.)
    localparam int RST_CYCLES = 5_000;
    task automatic restart_a();
        @(negedge clkA); rstA = 1'b1; a_off = 1'b1; gap_ok = 1'b1;
        @(negedge clkB); force lkb.g_b.u_phy_rx.u_trainer.retrain = 1'b1;
        repeat (2) @(negedge clkB); release lkb.g_b.u_phy_rx.u_trainer.retrain;
        repeat (RST_CYCLES) @(negedge clkA); a_off = 1'b0; rstA = 1'b0;
    endtask
    task automatic restart_b();
        @(negedge clkB); rstB = 1'b1; b_off = 1'b1; have_last = 1'b0;
        @(negedge clkA); force lka.g_a.u_phy_rx.u_trainer.retrain = 1'b1;
        repeat (2) @(negedge clkA); release lka.g_a.u_phy_rx.u_trainer.retrain;
        repeat (RST_CYCLES) @(negedge clkB); b_off = 1'b0; rstB = 1'b0;
    endtask
    task automatic drain(input string what);
        int t = 0;
        src_on = 1'b0; stall = 1'b0;
        while (!(src_i == 0 && !f_valid && a_consumed == a_sent && b_consumed == a_sent && lb.rcvd == a_sent)
               && t < 300_000) begin @(posedge clkA); t++; end
        if (t == 300_000) fail($sformatf("%s: not drained: A sent %0d reported %0d, B received %0d taken %0d",
                                         what, a_sent, a_consumed, lb.rcvd, b_consumed));
    endtask

    initial begin
        #50_000_000;
        fail("timeout"); $display("TB_FAIL tb_link_credit errors=%0d", errors); $finish;
    end
    initial begin
        int c0, t, tc;
        repeat (20) @(posedge clkA); rstA <= 0; rstB <= 0;
        // ---- start-up: flush + SYNC, then a random stream ----
        wait_sync(2, 0, "start-up");
        src_on = 1'b1;
        wait_pkts(300, "start-up stream");
        if (a_c_sync != 2 || b_c_sync != 2) fail($sformatf("start-up: SYNCs A sent %0d, B took %0d (expected 2)", a_c_sync, b_c_sync));
        $display("ok: start-up: flush + SYNC (A %0d, B %0d), %0d packets streamed", a_c_sync, b_c_sync, n_pkts);
        // ---- 1. A restarts while B stalls with a full buffer ----
        stall = 1'b1; t = 0;
        while ((lb.rcvd - b_consumed) < 3700 && t < 300_000) begin @(posedge clkA); t++; end
        if ((lb.rcvd - b_consumed) < 3700) fail($sformatf("case 1: B's buffer holds only %0d words", lb.rcvd - b_consumed));
        restart_a();
        wait_sync(2, 0, "case 1");
        if ((a_sent - a_consumed) != (lb.rcvd - b_consumed))
            fail($sformatf("case 1: after the sync A counts %0d words in B, B holds %0d", a_sent - a_consumed, lb.rcvd - b_consumed));
        $display("ok: case 1: A restarted under a full B (%0d words held), synchronised with %0d in flight", lb.rcvd - b_consumed,
                 a_sent - a_consumed);
        repeat (30_000) @(posedge clkA);            // B still stalled: A must stop at its free space
        stall = 1'b0;
        wait_pkts(300, "case 1 after the stall");
        gap_ok = 1'b0;
        $display("ok: case 1: no word refused, stream resumed (%0d gap)", n_gaps);
        // ---- 1b. A restarts while B keeps taking words: B's CREDITs of A's previous incarnation carry the same SYNC
        //          id (1) as the new A's first SYNC and reach the new A first (the flush must keep them out) ----
        wait_pkts(50, "case 1b before");
        restart_a();
        wait_sync(2, 0, "case 1b");
        wait_pkts(300, "case 1b after");
        gap_ok = 1'b0;
        $display("ok: case 1b: A restarted while B reported credit with the same SYNC id, stream resumed");
        // ---- 2. B restarts (link + port) under a streaming A ----
        wait_pkts(100, "case 2 before");
        c0 = a_c_sync;
        restart_b();
        wait_sync(2, c0, "case 2");
        wait_pkts(300, "case 2 after");
        $display("ok: case 2: B restarted under a streaming A, %0d SYNCs, stream resumed", a_c_sync - c0);
        // ---- 3. A restarts, sends 10 x 399 words, then B restarts (a wrong resync rule: room 106 for ever) ----
        src_on = 1'b0;
        while (src_i != 0 || f_valid) @(posedge clkA);
        restart_a();
        wait_sync(2, 0, "case 3, A");
        c0 = src_p;
        for (int k = 0; k < 10; k++) lens[c0 + k] = 399;   // not presented yet: the source is idle
        src_on = 1'b1;
        while (!(src_p == c0 + 9 && src_i != 0)) @(posedge clkA);
        drain("case 3");
        if (a_sent != 3990) fail($sformatf("case 3: A sent %0d words, expected 3990", a_sent));
        $display("ok: case 3: A restarted and sent %0d words in all", a_sent);
        gap_ok = 1'b0; c0 = a_c_sync;
        restart_b();
        src_on = 1'b1;
        wait_sync(2, c0, "case 3, B");
        wait_pkts(300, "case 3 after");
        $display("ok: case 3: B restarted after 3990 words from A, stream resumed (packets up to 400 words)");
        // ---- 4. B's port alone restarts, the link stays up (B transmits nothing at that moment) ----
        drain("case 4");
        @(negedge clkB iff (lb.xs == 0 && !lb.tx_valid && !lb.cr_due && !lb.ev_full && lb.st_t < 9000));
        c0 = a_c_sync; rstBp = 1'b1; have_last = 1'b0;
        repeat (20) @(negedge clkB); rstBp = 1'b0;
        src_on = 1'b1;
        wait_sync(2, c0, "case 4");
        wait_pkts(300, "case 4 after");
        $display("ok: case 4: B's port restarted with the link up, %0d SYNCs, stream resumed", a_c_sync - c0);
        // ---- 5. B restarts right after it took the SYNC: its answer is lost ----
        c0 = a_c_sync;
        restart_b();
        @(posedge clkB iff (lb.sync_ev && lb.sync_id != 0));
        restart_b();                                // in reset from the next negedge: no CREDIT answers the SYNC
        wait_sync(4, c0, "case 5");
        wait_pkts(300, "case 5 after");
        $display("ok: case 5: B restarted before answering SYNC, A synchronised again (%0d SYNCs), stream resumed", a_c_sync - c0);
        // ---- 6. B retrains mid-stream: no restart, so no SYNC and no gap ----
        wait_pkts(50, "case 6 before");
        c0 = a_c_sync; tc = b_train_cnt;
        @(negedge clkB); force lkb.g_b.u_phy_rx.u_trainer.retrain = 1'b1;
        repeat (2) @(negedge clkB); release lkb.g_b.u_phy_rx.u_trainer.retrain;
        wait_pkts(300, "case 6 after");
        if (b_train_cnt == tc) fail("case 6: B did not retrain");
        if (a_c_sync != c0) fail($sformatf("case 6: %0d SYNCs after a retrain", a_c_sync - c0));
        $display("ok: case 6: B retrained (trainings %0d -> %0d), no SYNC, no gap", tc, b_train_cnt);
        // ---- final ----
        drain("final");
        if (full_hits != 0 || inv_errs != 0) fail($sformatf("%0d refused words, %0d credit violations", full_hits, inv_errs));
        if (!la.synced) fail("A not synchronised at the end");
        if (a_unk != 0 || b_unk != 0) fail($sformatf("unknown packets: A %0d B %0d", a_unk, b_unk));
        $display("INFO_TB packets %0d gaps %0d, A sent %0d = reported = B taken, SYNCs A %0d B %0d", n_pkts, n_gaps, a_sent,
                 a_c_sync, b_c_sync);
        if (errors == 0) $display("TB_PASS tb_link_credit");
        else $display("TB_FAIL tb_link_credit errors=%0d", errors);
        $finish;
    end
endmodule
`default_nettype wire
