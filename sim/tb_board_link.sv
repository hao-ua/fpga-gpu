// tb_board_link — two link_test_top instances (ROLE A and B) joined by JM1/JM2 "cables" with
// per-wire delays and per-pin directions (JM1 and JM2[29:22] are driven by A,
// JM2[21:0] by B). Checks: receiver training on both sides (different clock-vs-data skews on the two
// lanes -> different trained phases), lock, clean PRBS traffic over the 64-bit forward and 32-bit
// backward lane, XOFF back-pressure without loss, injected bit errors counted and recovered, a mid-run
// skew change that forces a retrain, and (with +GUARD_TEST) the power-up contention guard.
`timescale 1ns/1ps
module tb_board_link;
    // slightly different oscillators on the two boards
    reg clkA = 0, clkB = 0;
    always #2.500  clkA = ~clkA;         // 200.000 MHz
    always #2.4999 clkB = ~clkB;         // 200.008 MHz
    reg rstA_n = 0, rstB_n = 0;
    wire [1:0] ledA, ledB; wire uartA, uartB;

    // ---- cables ----
    // Each wire is a delay line from its driver's board to the other board. JM1: all 32 wires A -> B.
    // JM2: bits [21:0] B -> A (backward lane), bits [31:22] A -> B (forward lane extension; 30/31 unused).
    wire [31:0] cab1A, cab1B, cab2A, cab2B;         // A-side / B-side nets of the two cables
    reg  [31:0] cab1B_r = 0, cab2A_r = 0, cab2B_r = 0;
    real d1 [0:31], d2 [0:31];
    genvar g;
    generate
        for (g = 0; g < 32; g++) begin : g_c1          // JM1: A drives
            pulldown (cab1A[g]);
            assign cab1B[g] = cab1B_r[g];
            always @(cab1A[g]) cab1B_r[g] <= #(d1[g]) cab1A[g];
        end
        for (g = 0; g < 22; g++) begin : g_c2b         // JM2[21:0]: B drives
            pulldown (cab2B[g]);
            assign cab2A[g] = cab2A_r[g];
            always @(cab2B[g]) cab2A_r[g] <= #(d2[g]) cab2B[g];
        end
        for (g = 22; g < 32; g++) begin : g_c2a        // JM2[31:22]: A drives
            pulldown (cab2A[g]);
            assign cab2B[g] = cab2B_r[g];
            always @(cab2A[g]) cab2B_r[g] <= #(d2[g]) cab2A[g];
        end
    endgenerate
    initial begin
        for (int k = 0; k < 32; k++) begin
            d1[k] = 1.0 + 0.0137 * ((k * 7) % 16);                  // forward lane, JM1 part: ~1.0 .. 1.2 ns
            d2[k] = (k < 22) ? 2.3 + 0.0137 * ((k * 5) % 16)        // backward lane: ~2.3 .. 2.5 ns
                             : 1.0 + 0.0137 * ((k * 3) % 16);       // forward lane, JM2 part: like JM1
        end
        d1[20] = 2.6;                                   // forward clock 1.6 ns LATE against its data
        d2[20] = 1.0;                                   // backward clock 1.3 ns EARLY against its data
    end

    // STAT 10 ms and a 10x faster UART (DIV 87 -> 868 ns/bit) keep the two status lines inside ~25 ms
    // of simulated time; everything else is over by ~10 ms.
    localparam int GUARD = 3000, IDLE_TO = 200_000, STAT = 1_000_000, MRST = 20_000, UDIV = 87;
    localparam real UBIT = 868.0;                        // ns per UART bit at UDIV
    // RELOCK (500 us) is longer than the injection burst in step 4 (6 errors over ~60 us, a packet can be
    // 20 us) so isolated errors do not retrain; the persistent errors of step 5 do.
    localparam int TW = 256, SETTLE = 32, RETRY = 20_000, RELOCK = 50_000, RWIN = 200_000, RDROPS = 15;
    localparam int NSTEPS = 140;                        // scan points per period at 50 MHz (link_rx_trainer)
`ifdef REAL_MMCM
    localparam bit SIMSH = 1'b0;
`else
    localparam bit SIMSH = 1'b1;
`endif
    // GUARD_TOGGLES: the guard-test foreign 50 MHz clock gives ~3000 toggles in the 30 us window.
    // GEN_RATE_DIV 3: the generators (100 MHz) offer a beat every third cycle, slower than the 50 MHz
    // lanes consume (hardware runs producer and lane at equal rates) — the ARQ transmit ring must
    // cope with running empty in the middle of a packet.
    link_test_top #(.ROLE("A"), .LINK_MHZ(50), .GUARD_CYCLES(GUARD), .GUARD_TOGGLES(100), .IDLE_TIMEOUT(IDLE_TO), .STATUS_CYCLES(STAT), .MMCM_RST_CYCLES(MRST),
                    .TRAIN_WORDS(TW), .SETTLE_CYCLES(SETTLE), .RETRY_CYCLES(RETRY), .RELOCK_CYCLES(RELOCK),
                    .RETRAIN_WINDOW(RWIN), .RETRAIN_DROPS(RDROPS), .SIM_SHIFTER(SIMSH), .GEN_RATE_DIV(3), .UART_DIV(UDIV)) A (
        .sys_clk_p(clkA), .sys_clk_n(~clkA), .rst_btn_n(rstA_n), .key1_n(1'b1), .led(ledA), .uart_tx(uartA), .jm1(cab1A), .jm2(cab2A));
    link_test_top #(.ROLE("B"), .LINK_MHZ(50), .GUARD_CYCLES(GUARD), .GUARD_TOGGLES(100), .IDLE_TIMEOUT(IDLE_TO), .STATUS_CYCLES(STAT), .MMCM_RST_CYCLES(MRST),
                    .TRAIN_WORDS(TW), .SETTLE_CYCLES(SETTLE), .RETRY_CYCLES(RETRY), .RELOCK_CYCLES(RELOCK),
                    .RETRAIN_WINDOW(RWIN), .RETRAIN_DROPS(RDROPS), .SIM_SHIFTER(SIMSH), .GEN_RATE_DIV(3), .UART_DIV(UDIV)) B (
        .sys_clk_p(clkB), .sys_clk_n(~clkB), .rst_btn_n(rstB_n), .key1_n(1'b1), .led(ledB), .uart_tx(uartB), .jm1(cab1B), .jm2(cab2B));

    int errors = 0;
    task automatic check(input bit cond, input string msg);
        if (!cond) begin errors++; $display("ERROR @%0t: %s", $time, msg); end
    endtask
    task automatic show(input string tag);
        $display("[%0t] %s | A: lock=%0d trn=%0d up=%0d en=%0d txp=%0d rxw=%0d rxp=%0d par=%0d crc=%0d prbs=%0d seq=%0d frm=%0d xoff=%0d drop=%0d | B: lock=%0d trn=%0d up=%0d en=%0d txp=%0d rxw=%0d rxp=%0d par=%0d crc=%0d prbs=%0d seq=%0d frm=%0d xoff=%0d drop=%0d",
            $time, tag,
            A.lock, A.trained, A.far_up, A.tx_enabled, A.tx_pkts, A.rx_words, A.rx_pkts, A.par_errs, A.crc_errs, A.prbs_errs, A.seq_errs, A.frm_errs, A.xoff_events, A.lock_drops,
            B.lock, B.trained, B.far_up, B.tx_enabled, B.tx_pkts, B.rx_words, B.rx_pkts, B.par_errs, B.crc_errs, B.prbs_errs, B.seq_errs, B.frm_errs, B.xoff_events, B.lock_drops);
    endtask
    task automatic show_train(input string tag);
        $display("[%0t] %s | A: train_cnt=%0d eye=%0d phase=%0d payload=%0d retx=%0d/%0d disc=%0d/%0d | B: train_cnt=%0d eye=%0d phase=%0d payload=%0d retx=%0d/%0d disc=%0d/%0d",
            $time, tag, A.train_cnt, A.eye_width, A.phase, A.payload_words, A.retx[31:16], A.retx[15:0], A.disc[31:16], A.disc[15:0],
            B.train_cnt, B.eye_width, B.phase, B.payload_words, B.retx[31:16], B.retx[15:0], B.disc[31:16], B.disc[15:0]);
    endtask
    // PHY/data errors only: with ARQ the deframers' seq/frm counts are informational (resent
    // duplicates and abort beats are counted there by design)
    function automatic int errsum_A(); return A.par_errs + A.crc_errs + A.prbs_errs; endfunction
    // link_rx_diag word k of board 0 = A, 1 = B (link_core diag outputs)
    function automatic [31:0] dg(input int brd, input int k); return brd ? B.u_link.diag[k*32 +: 32] : A.u_link.diag[k*32 +: 32]; endfunction
    function automatic int errsum_B(); return B.par_errs + B.crc_errs + B.prbs_errs; endfunction
    function automatic int cdiff(input int a, input int b);   // (a - b) mod NSTEPS
        int d; d = (a - b) % NSTEPS; if (d < 0) d += NSTEPS; return d;
    endfunction
    // internal state of both directions, for diagnosing a stall
    task automatic dump();
        $display("DUMP A.arq_tx: wp=%0d bp=%0d ra=%0d s0v=%0d s1v=%0d inflight=%0d at_sstart=%0d rw_pend=%0d ctl_pend=%0d next_seq=%0d base_seq=%0d to_cnt=%0d u_empty=%0d",
            A.u_link.g_a.u_arq_tx.wp, A.u_link.g_a.u_arq_tx.bp, A.u_link.g_a.u_arq_tx.ra, A.u_link.g_a.u_arq_tx.s0v, A.u_link.g_a.u_arq_tx.s1v,
            A.u_link.g_a.u_arq_tx.inflight, A.u_link.g_a.u_arq_tx.at_sstart, A.u_link.g_a.u_arq_tx.rw_pend, A.u_link.g_a.u_arq_tx.ctl_pend,
            A.u_link.g_a.u_arq_tx.next_seq, A.u_link.g_a.u_arq_tx.base_seq, A.u_link.g_a.u_arq_tx.to_cnt, A.u_link.g_a.u_arq_tx.u_empty);
        $display("DUMP A.framer: at_header=%0d send_crc=%0d boundary=%0d far_up=%0d xoff=%0d f_empty=%0d | A.rxfifo count=%0d xoff_r=%0d",
            A.u_link.g_a.u_framer.at_header, A.u_link.g_a.u_framer.send_crc, A.u_link.g_a.u_framer.boundary, A.u_link.g_a.u_framer.far_up,
            A.u_link.g_a.u_framer.xoff, A.u_link.g_a.u_framer.f_empty, A.u_link.rxf_count, A.u_link.xoff_r);
        $display("DUMP B.deframer: have=%0d rx_ready=%0d synced=%0d in_pkt=%0d remaining=%0d lock=%0d | B.arq_rx: wt=%0d wc=%0d rp=%0d expected=%0d in_ready=%0d ack_cnt=%0d nak_tmr=%0d",
            B.u_link.g_b.u_deframer.have, B.u_link.g_b.u_deframer.rx_ready, B.u_link.g_b.u_deframer.synced, B.u_link.g_b.u_deframer.in_pkt,
            B.u_link.g_b.u_deframer.remaining, B.u_link.g_b.u_deframer.lock, B.u_link.g_b.u_arq_rx.wt, B.u_link.g_b.u_arq_rx.wc, B.u_link.g_b.u_arq_rx.rp,
            B.u_link.g_b.u_arq_rx.expected, B.u_link.g_b.u_arq_rx.in_ready, B.u_link.g_b.u_arq_rx.ack_cnt, B.u_link.g_b.u_arq_rx.nak_tmr);
        $display("DUMP B.arq_tx: wp=%0d bp=%0d ra=%0d s0v=%0d s1v=%0d at_sstart=%0d rw_pend=%0d ctl_pend=%0d next_seq=%0d base_seq=%0d u_empty=%0d | B.framer: at_header=%0d send_crc=%0d boundary=%0d far_up=%0d xoff=%0d f_empty=%0d | B.rxfifo count=%0d xoff_r=%0d",
            B.u_link.g_b.u_arq_tx.wp, B.u_link.g_b.u_arq_tx.bp, B.u_link.g_b.u_arq_tx.ra, B.u_link.g_b.u_arq_tx.s0v, B.u_link.g_b.u_arq_tx.s1v,
            B.u_link.g_b.u_arq_tx.at_sstart, B.u_link.g_b.u_arq_tx.rw_pend, B.u_link.g_b.u_arq_tx.ctl_pend, B.u_link.g_b.u_arq_tx.next_seq, B.u_link.g_b.u_arq_tx.base_seq,
            B.u_link.g_b.u_arq_tx.u_empty, B.u_link.g_b.u_framer.at_header, B.u_link.g_b.u_framer.send_crc, B.u_link.g_b.u_framer.boundary,
            B.u_link.g_b.u_framer.far_up, B.u_link.g_b.u_framer.xoff, B.u_link.g_b.u_framer.f_empty, B.u_link.rxf_count, B.u_link.xoff_r);
        $display("DUMP A.deframer: synced=%0d in_pkt=%0d remaining=%0d lock=%0d rx_ready=%0d | A.arq_rx: wt=%0d wc=%0d rp=%0d expected=%0d in_ready=%0d",
            A.u_link.g_a.u_deframer.synced, A.u_link.g_a.u_deframer.in_pkt, A.u_link.g_a.u_deframer.remaining, A.u_link.g_a.u_deframer.lock,
            A.u_link.g_a.u_deframer.rx_ready, A.u_link.g_a.u_arq_rx.wt, A.u_link.g_a.u_arq_rx.wc, A.u_link.g_a.u_arq_rx.rp,
            A.u_link.g_a.u_arq_rx.expected, A.u_link.g_a.u_arq_rx.in_ready);
    endtask
    // wait for both receive-packet counters to reach targets, or fail (a deadlock must not hang the bench)
    task automatic wait_pkts(input int a_pkts, input int b_pkts, input int limit_ns, input string msg);
        fork
            begin wait (A.rx_pkts >= a_pkts && B.rx_pkts >= b_pkts); end
            begin #(limit_ns); check(0, {"timeout: ", msg}); show("timeout"); show_train("timeout"); dump(); $display("TB_FAIL errors=%0d", errors); $finish; end
        join_any disable fork;
    endtask

    // UART line capture on A
    string uline = ""; int ulines = 0;
    initial begin
        forever begin
            reg [7:0] b; int i;
            @(negedge uartA); #(UBIT/2);
            for (i = 0; i < 8; i++) begin #(UBIT); b[i] = uartA; end
            #(UBIT);
            if (b == 8'h0A) begin $display("A UART: %s", uline); ulines++; uline = ""; end
            else if (b != 8'h0D) uline = {uline, string'(b)};
        end
    end

    // +PROBE: print the first popped beats of B's forward deframer (data/control alignment check), then stop
    bit probe = 0; int probe_n = 0;
    always @(posedge B.clk_sys) begin
        if (probe && B.u_link.g_b.u_deframer.pop && B.u_link.g_b.u_deframer.synced && probe_n < 60 &&
            (B.u_link.g_b.u_deframer.w_last || B.u_link.g_b.u_deframer.crc_here || !B.u_link.g_b.u_deframer.in_pkt ||
             B.u_link.g_b.u_deframer.err_frm)) begin
            probe_n++;
            $display("PROBE %0d: lo=%08h hi=%08h vhi=%0d last=%0d in_pkt=%0d remaining=%0d lo_is_crc=%0d hi_is_crc=%0d err_late=%0d err_early=%0d",
                probe_n, B.u_link.g_b.u_deframer.lo, B.u_link.g_b.u_deframer.hi, B.u_link.g_b.u_deframer.w_vhi, B.u_link.g_b.u_deframer.w_last,
                B.u_link.g_b.u_deframer.in_pkt, B.u_link.g_b.u_deframer.remaining, B.u_link.g_b.u_deframer.lo_is_crc, B.u_link.g_b.u_deframer.hi_is_crc,
                B.u_link.g_b.u_deframer.err_late, B.u_link.g_b.u_deframer.err_early);
            if (probe_n == 60) begin $display("PROBE done"); $finish; end
        end
    end
    // +PROBE also records, at A's forward TX, when the framer's LAST register rises, the next lclk /
    // lclk90 edges, and when LAST appears at the pin (control launch timing)
    reg plast_d = 0; int tx_probe_n = 0;
    always @(posedge A.u_link.g_a.u_framer.last) if (probe && tx_probe_n < 4) begin
        tx_probe_n++;
        $display("TXPROBE: framer.last rises @%0t (lclk=%0d lclk90=%0d)", $time, A.lclk, A.lclk90);
    end
    always @(posedge A.lclk) if (probe && tx_probe_n > 0 && tx_probe_n <= 4 && A.u_link.g_a.u_framer.last) $display("TXPROBE:   lclk rise @%0t with last=1", $time);
    always @(posedge A.lclk90) if (probe && tx_probe_n > 0 && tx_probe_n <= 4 && A.u_link.g_a.u_framer.last) $display("TXPROBE:   lclk90 rise @%0t with last=1", $time);
    always @(cab1A[17]) if (probe && tx_probe_n > 0 && tx_probe_n <= 4) $display("TXPROBE:   LAST pin (A side) -> %0d @%0t; D0 pin=%0d CLK pin=%0d", cab1A[17], $time, cab1A[0], cab1A[20]);
    always @(cab1A[20]) if (probe && tx_probe_n > 0 && tx_probe_n <= 4 && cab1A[20]) $display("TXPROBE:   CLK pin rise @%0t (LAST pin=%0d)", $time, cab1A[17]);
    bit guard_test = 0;
    reg fclk = 0;
    initial begin
        if ($test$plusargs("GUARD_TEST")) guard_test = 1;
        if ($test$plusargs("PROBE")) probe = 1;
        if (guard_test) begin
            // a foreign 50 MHz clock sits on A's forward-lane LCLK pin (JM1 pin 25) during A's guard window
            fork begin forever #10 fclk = ~fclk; end join_none
            force cab1A[20] = fclk;
        end
        #200 rstA_n = 1; rstB_n = 1;
        wait (A.mmcm_locked && B.mmcm_locked);
        $display("[%0t] MMCMs locked", $time);
        if (guard_test) begin
            #(GUARD * 10 + 5000);
            release cab1A[20];
            check(A.guard_fail == 1, "A must detect the foreign clock and trip the guard");
            check(A.tx_enabled == 0, "A must keep its TX lane disabled");
            check(B.guard_fail == 0 && B.tx_enabled == 1, "B (clean lane) must enable normally");
            // B transmits TRAIN words on JM2 (it never sees UP), so A's receiver trains; A's TX stays off.
            fork
                begin wait (A.trained); end
                begin #6_000_000; check(0, "timeout: A must train on B's TRAIN words"); end
            join_any disable fork;
            #20000;
            check(B.far_up == 0, "B must not see UP from A (A TX disabled)");
            check(B.lock == 0 && B.trained == 0, "B cannot train/lock: A never drives the forward lane");
            check(A.lock == 0, "A gets only TRAIN words from B, never a packet: no lock");
            check(ledA[1] !== 1'bx, "A LED2 drives the guard blink");
            show("guard"); show_train("guard");
            if (errors == 0) $display("TB_PASS (guard test)"); else $display("TB_FAIL errors=%0d", errors);
            $finish;
        end

        // ---------- 1. both receivers train, link comes up ----------
        fork
            begin wait (A.lock && B.lock && A.far_up && B.far_up); end
            begin #10_000_000; check(0, "timeout waiting for train/lock/up on both boards"); $finish; end
        join_any disable fork;
        show("linked"); show_train("linked");
        check(A.tx_enabled && B.tx_enabled, "both TX lanes enabled");
        check(A.rx_clk_ok && B.rx_clk_ok, "both RX-clock MMCMs locked on the forwarded clocks");
        check(A.train_cnt == 1 && B.train_cnt == 1, "exactly one training per receiver so far");
        check(A.eye_width >= 50 && B.eye_width >= 50, "eye >= 50 scan points (~7 ns of the 10 ns bit) on both receivers");
        // A receives the backward lane (clock 1.3 ns early), B the forward lane (clock 1.6 ns late):
        // A's trained phase must be ~2.9 ns (= 20 scan points of 142.9 ps) above B's, circularly
        check(cdiff(A.phase, B.phase) >= 16 && cdiff(A.phase, B.phase) <= 24, "trained phases differ by the skew difference (~20 points)");
        check(A.training == 0 && B.training == 0, "both framers left TRAIN mode");

        // ---------- 2. clean traffic (A -> B two words per beat, B -> A one) ----------
        wait_pkts(300, 300, 8_000_000, "300 packets each way");
        show("traffic"); show_train("traffic");
        check(errsum_A() == 0, "A must have zero errors after clean traffic");
        check(errsum_B() == 0, "B must have zero errors after clean traffic");
        check(A.payload_words > 50_000 && B.payload_words > 50_000, "payload words checked on both sides");
        check(B.payload_words > A.payload_words, "forward lane (64-bit) carries more words than backward (32-bit) in the same time");
        check(A.xoff_events == 0 && B.xoff_events == 0, "no XOFF during free-running traffic");
        // fill words: the starved generators leave gaps between packets, which the framers
        // fill; the receivers drop them before their FIFOs (the PRBS checkers above saw only real packets)
        // (the backward lane here carries B's data plus an ACK every 4 forward packets, so it rarely idles at a packet
        // boundary; tb_two_boards covers fill words on it with board B's real 1 ms traffic)
        check(dg(1, 5) > 0, $sformatf("fill words on the forward lane (A rx %0d, B rx %0d)", dg(0, 5), dg(1, 5)));
        check(dg(0, 6) == 0 && dg(1, 6) == 0 && dg(0, 7) == 0 && dg(1, 7) == 0, "receive diagnostics: no parity errors");
        check(dg(0, 0) > 0 && dg(1, 0) > 0, "receive diagnostics count bursts");

        // ---------- 3. XOFF back-pressure (stall B's consumer on the forward lane) ----------
        begin
            automatic int pkts_before = B.rx_pkts;
            force B.u_link.g_b.u_deframer.rx_ready = 1'b0;      // stalled consumer: no pops, so the checker sees nothing
            #100_000;                                           // 100 us: ~5000 beats would arrive -> FIFO must XOFF
            check(B.xoff_events >= 1, "B must have asserted XOFF while stalled");
            check(B.u_link.g_b.u_rxfifo.wfull == 0, "B RX FIFO must not overflow under XOFF");
            release B.u_link.g_b.u_deframer.rx_ready;
            wait_pkts(0, pkts_before + 200, 4_000_000, "200 packets after the XOFF stall");
            show("after-xoff");
            check(errsum_B() == 0, "no errors/loss after XOFF stall (seq/crc/prbs would show loss)");
            check(A.lock && B.lock, "both still locked after XOFF");
        end

        // ---------- 4. error injection on the forward lane (JM1 data bit 3 = D[3]) ----------
        begin
            automatic int inj; automatic int par_b = B.par_errs, crc_b = B.crc_errs;
            for (inj = 0; inj < 6; inj++) begin
                #(7_777 + inj * 1_234);
                force cab1A[3] = ~cab1A[3]; #20; release cab1A[3];   // corrupt one bit period
            end
            #200_000;
            show("after-inject"); show_train("after-inject");
            check(B.par_errs > par_b, "B must count parity errors from injected bits");
            check(B.crc_errs > crc_b || B.frm_errs > 0, "B must count a CRC (or frame) error from injected bits");
            check(B.lock_drops >= 1, "lock must have dropped at least once");
            // ARQ: the damaged packets were discarded and resent, the user stream stays perfect
            check(B.prbs_errs == 0 && B.bit_err_map == 0, "ARQ: B's delivered stream has no PRBS error despite the injected bits");
            check(B.disc[15:0] >= 1 && B.disc[31:16] >= 1, "ARQ: B discarded packets and requested NAKs");
            check(A.retx[15:0] >= 1, "ARQ: A rewound its transmitter on the NAK(s)");
            check(B.train_cnt == 1, "a few isolated errors must not trigger a retrain");
            // receive diagnostics on B: every parity error of a real word is classified (histogram) and the last one kept
            check(dg(1, 7) >= B.par_errs - par_b && dg(1, 7) > 0, $sformatf("B diag parity errors %0d (deframer %0d)", dg(1, 7), B.par_errs));
            begin automatic bit [31:0] hh = dg(1, 2), le = dg(1, 3);
                check(hh[31:24] + hh[23:16] + hh[15:8] == dg(1, 7), "B diag histogram buckets add up to its parity errors");
                check(le[3:0] != 0, "B diag last error carries its parity flags"); end
            wait (B.lock && A.far_up);
            begin automatic int p = B.rx_pkts; automatic int e = errsum_B(); wait_pkts(0, p + 100, 3_000_000, "100 packets after the injection");
                check(errsum_B() == e, "no further errors after recovery"); end
            check(A.par_errs == 0 && A.crc_errs == 0 && A.prbs_errs == 0, "backward lane untouched by the injection");
            check(A.bit_err_map == 0, "A saw no data-bit errors so far");
        end

        // ---------- 5. skew change on the backward lane clock -> A must retrain ----------
        begin
            automatic int ph0 = A.phase, tc0 = A.train_cnt, tcB = B.train_cnt; automatic int e;
            automatic int bpar = B.par_errs, bcrc = B.crc_errs;
            d2[20] = 7.0;                                         // clock now 4.7 ns LATE (was 1.3 ns early): +6 ns
            fork
                begin wait (A.train_cnt == tc0 + 1); end
                begin #5_000_000; check(0, "timeout: A must retrain after the skew change"); end
            join_any disable fork;
            // with the behavioural shifter the retrain comes from the deframer losing lock; the UNISIM MMCM
            // instead loses LOCKED on the abrupt input phase step, which restarts training directly
            if (SIMSH) check(A.lock_drops >= 1, "A lost lock on the skewed stream");
            fork
                begin wait (A.trained && A.lock && B.far_up); end
                begin #3_000_000; check(0, "timeout: link must come back after the retrain"); end
            join_any disable fork;
            show("after-skew"); show_train("after-skew");
            // the clock now arrives 6 ns later against the data, so the shifter must add 6 ns LESS: -42 points
            check(cdiff(ph0, A.phase) >= 37 && cdiff(ph0, A.phase) <= 47, "A's new phase moved by -6 ns (42 points down)");
            check(B.train_cnt == tcB, "B (other direction) did not retrain");
            // A's lane kept sending data. While A retrains no ACK reaches A, its window fills and a rewind may close the
            // packet in flight with LAST early (link_arq_tx abort): B sees a frame error and may drop lock for a moment, but
            // no wire error (parity / CRC), no retrain, and its delivered stream stays clean (checked below)
            check(B.par_errs == bpar && B.crc_errs == bcrc, "B saw no parity / CRC error while A retrained");
            e = errsum_A();
            begin automatic int p = A.rx_pkts; wait_pkts(p + 200, 0, 6_000_000, "200 packets on A after the retrain"); end
            check(errsum_A() == e, "no further errors on A after the retrain");
            // ARQ: the garbage during the misalignment and the packets cut by the retrain were all
            // discarded and resent — A's delivered stream never saw an error
            check(A.prbs_errs == 0 && A.bit_err_map == 0, "ARQ: A's delivered stream has no PRBS error across the retrain");
            // (B's resend after the outage happens through the relink rewind, which RETX does not count;
            //  the clean PRBS stream above is the proof that it happened)
        end

        // ---------- 6. board A restarts while B keeps running (sequence numbers resynchronise) ----------
        begin
            automatic int pB = B.prbs_errs, dB = B.disc[15:0]; automatic int e;
            rstA_n = 0; #2000; rstA_n = 1;                         // A: everything restarts from zero
            fork
                begin wait (A.trained && B.trained && A.lock && B.lock && A.far_up && B.far_up); end
                begin #10_000_000; check(0, "timeout: link must come back after A's restart"); dump(); $display("TB_FAIL errors=%0d", errors); $finish; end
            join_any disable fork;
            show("after-restart"); show_train("after-restart");
            // B's checker sees A's generator restart its sequence once; after that both streams must be clean
            check(B.prbs_errs <= pB + 1, "B: at most one PRBS discontinuity from A's generator restart");
            e = B.prbs_errs;
            begin automatic int pa = A.rx_pkts, pb = B.rx_pkts; wait_pkts(pa + 200, pb + 200, 8_000_000, "200 packets each way after A's restart"); end
            check(B.prbs_errs == e && A.prbs_errs == 0 && A.bit_err_map == 0, "ARQ: both delivered streams clean after the restart (sequence resync via boundary word)");
            // a restart legitimately costs the in-flight window plus what arrives before the boundary
            // word (hardware: ~220 packets); a resync failure would show as thousands
            check(B.disc[15:0] < dB + 500, "B did not discard a storm of packets after A's restart");
        end

        // ---------- 7. status lines ----------
        wait (ulines >= 2);
        show("final"); show_train("final");
        $display("B bit_err_map=%08h", B.bit_err_map);
        check(A.prbs_errs == 0 && B.prbs_errs <= 1, "ARQ: delivered streams error-free for the whole run (B: only A's generator restart)");
        if (errors == 0) $display("TB_PASS"); else $display("TB_FAIL errors=%0d", errors);
        $finish;
    end
endmodule
