`timescale 1ns/1ps
// tb_tex_cache_pf — the texture cache with the stream PREFETCH, the hashed index and 512 sets,
// randomised: ascending / descending burst streams of random length with random gaps, a hot set of repeated bursts (hits),
// a DDR model with random command acceptance and in-order latency 5..40, and invalidates (GENW 4: the generation wraps and
// the sweep runs several times) with the memory CONTENT changing at each — a stale line validated by a late prefetch fill
// would return the old epoch's data. Every result is checked in order against the model. Then a directed ascending sweep of
// fresh bursts measures what the prefetch buys (demand fetches per lookup; with PF the merges must carry a good share).
//   -Defines TB_TC256 / TB_TC_HASH / TB_TC_NOPF select the variants (as tb_b_render / tb_tmu24).
module tb_tex_cache_pf;
    reg clk=0; always #5 clk=~clk;
    reg rst=1, inval=0;
    reg         lu_valid=0; reg [25:0] lu_burst=0; wire lu_ready, lu_done; wire [255:0] lu_data;
    wire        mig_en; wire [28:0] mig_addr; reg mig_rdy=0, mig_rd_valid=0; reg [255:0] mig_rd_data=0;
    wire [31:0] n_hit, n_miss;
`ifdef TB_TC256
    localparam int SETS = 256;
`else
    localparam int SETS = 512;
`endif
`ifdef TB_TC_HASH
    localparam bit HASH = 1'b1;
`else
    localparam bit HASH = 1'b0;
`endif
`ifdef TB_TC_NOPF
    localparam bit PF = 1'b0;
`else
    localparam bit PF = 1'b1;
`endif
    tex_cache #(.GENW(4), .SETS(SETS), .HASH(HASH), .PF(PF)) DUT(.clk,.rst,.inval,.lu_valid,.lu_burst,.lu_ready,.lu_done,.lu_data,
        .mig_en,.mig_addr,.mig_rdy,.mig_rd_valid,.mig_rd_data,.n_hit,.n_miss);

    // ---- memory model: a burst's data = 8 copies of {epoch, burst, 00}; the epoch changes with every invalidate ----
    integer epoch = 0;
    function automatic [255:0] model(input [25:0] b, input integer ep); model = {8{ep[3:0], b, 2'b00}}; endfunction
    // ---- DDR: random acceptance, in-order returns 5..40 cycles after acceptance, data of the epoch at return time ----
    longint now = 0, last_due = 0; reg [28:0] dq_a [$]; longint dq_t [$]; integer fetches = 0;
    always @(posedge clk) begin
        now++;
        mig_rdy <= ($urandom_range(0, 3) != 0);
        mig_rd_valid <= 1'b0;
        if (mig_en && mig_rdy) begin
            longint d; d = now + $urandom_range(5, 40); if (d <= last_due) d = last_due + 1; last_due = d;
            dq_a.push_back(mig_addr); dq_t.push_back(d); fetches++;
        end
        if (dq_a.size() != 0 && dq_t[0] <= now) begin
            mig_rd_data <= model(dq_a[0][28:3], epoch); mig_rd_valid <= 1'b1;
            void'(dq_a.pop_front()); void'(dq_t.pop_front());
        end
    end
    // ---- results in order against the expectations queued at issue ----
    reg [255:0] expq [$]; reg [25:0] expb [$];
    integer issued = 0, done = 0, errs = 0;
    always @(posedge clk) if (lu_done) begin
        if (expq.size() == 0) begin errs++; if (errs <= 5) $display("FAIL: result with nothing issued at %0d", now); end
        else begin
            if (lu_data !== expq[0]) begin
                errs++;
                if (errs <= 5) $display("FAIL: lookup %0d burst %h: got %h exp %h", done, expb[0], lu_data[31:0], expq[0][31:0]);
            end
            void'(expq.pop_front()); void'(expb.pop_front());
        end
        done++;
    end
    task automatic lookup(input [25:0] b);
        lu_burst <= b; lu_valid <= 1'b1;
        do @(posedge clk); while (!lu_ready);              // accepted at this edge (lu_ready read before its update)
        expq.push_back(model(b, epoch)); expb.push_back(b); issued++;
        lu_valid <= 1'b0;
    endtask
    task automatic drain;
        while (done < issued && now < 50000000) @(posedge clk);
    endtask
    task automatic do_inval;                                // the TMU drained (prefetch fills may still be in flight)
        drain; repeat (3) @(posedge clk);
        epoch++; inval <= 1'b1; @(posedge clk); inval <= 1'b0;
        repeat (2) @(posedge clk);
    endtask

    integer hot [0:31]; integer hot_issued = 0;
    integer sweep_hit0, sweep_miss0, sweep_fetch0, sweep_merge0, sweep_pf0;
    // +PFDBG: trace the stage-B decisions on hot[0] (tags of the set, generation)
    bit dbg_on; initial dbg_on = $test$plusargs("PFDBG");
    always @(posedge clk) if (dbg_on && DUT.b_v && DUT.b_burst == 26'(hot[0]))
        $display("  t%0d hot0 %h %s idx %h hit %b pm %b gen %0d ways {%h %h %h %h} coll %b", now, DUT.b_burst, DUT.b_pf ? "PF" : "DEMAND", DUT.b_idx,
                 DUT.hit_any, DUT.b_pm_v, DUT.cur_gen, DUT.tq0, DUT.tq1, DUT.tq2, DUT.tq3, DUT.coll_q);
    wire [25:0] h0 = 26'(hot[0]);
    wire [8:0] h0idx = HASH ? (h0[8:0] ^ h0[17:9]) : h0[8:0];                 // (SETS 512)
    always @(posedge clk) if (dbg_on && DUT.do_fill && 9'(DUT.fidx) == h0idx)
        $display("  t%0d fill idx %h way %0d tag %h gen %0d pf %b", now, DUT.fidx, DUT.fvic, DUT.ftag, DUT.fgen, DUT.fpf);
    initial begin
        for (int i = 0; i < 32; i++) hot[i] = $urandom_range(0, (1 << 20) - 1);
        repeat (3) @(posedge clk); rst <= 1'b0; repeat (2) @(posedge clk);
        // ---- phase 0: directed — a burst looked up again after its fill hits; looked up before its fill (duplicate) does not ----
        lookup(26'h0012345); drain; repeat (3) @(posedge clk);
        lookup(26'h0012345); drain;
        $display("tb_tex_cache_pf directed: after the fill: hits %0d misses %0d (expect 1 / 1)", n_hit, n_miss);
        if (n_hit != 1) begin $display("FAIL: no hit on a filled line"); errs++; end
        lookup(26'h0012345); lookup(26'h0012345); lookup(26'h0012345); drain;
        $display("tb_tex_cache_pf directed: three back to back: hits %0d misses %0d (expect 4 / 1)", n_hit, n_miss);
        if (n_hit != 4) begin $display("FAIL: back-to-back hits lost"); errs++; end
        // ---- phase 1: random streams + hot bursts, 24 invalidates ----
        for (int round = 0; round < 24; round++) begin
            for (int n = 0; n < 700; ) begin
                if ($urandom_range(0, 4) == 0) begin        // a run of hot bursts (≈ a quarter of the lookups: the hits)
                    repeat (8) begin lookup(26'(hot[$urandom_range(0, 31)])); n++; hot_issued++; repeat ($urandom_range(0, 2)) @(posedge clk); end
                end else begin                               // a stream: length 1..40, up or down, gaps 0..3 (sometimes longer)
                    int len, dir; reg [25:0] b;
                    len = $urandom_range(1, 40); dir = $urandom_range(0, 2) == 0 ? -1 : 1;
                    b = 26'($urandom_range(0, (1 << 22) - 1));
                    for (int k = 0; k < len; k++) begin
                        lookup(b); n++;
                        b = b + 26'(dir);
                        if ($urandom_range(0, 7) == 0) repeat ($urandom_range(1, 12)) @(posedge clk);
                        else repeat ($urandom_range(0, 3)) @(posedge clk);
                    end
                end
                if ($urandom_range(0, 15) == 0) repeat ($urandom_range(0, 20)) @(posedge clk);
            end
            if (round % 3 == 2) begin                        // every third round the hot set moves too
                for (int i = 0; i < 32; i++) hot[i] = $urandom_range(0, (1 << 20) - 1);
            end
            do_inval;
        end
        drain;
        $display("tb_tex_cache_pf random: %0d lookups (%0d hot), hits %0d misses %0d, fetches %0d, prefetches %0d (dropped %0d), merges %0d, errors %0d",
                 issued, hot_issued, n_hit, n_miss, fetches, DUT.n_pf, DUT.n_pfdrop, DUT.n_merge, errs);
        // ---- phase 2: a directed ascending sweep of 2000 fresh bursts, 2-cycle gaps (a texel row walked left to right) ----
        sweep_hit0 = n_hit; sweep_miss0 = n_miss; sweep_fetch0 = fetches; sweep_merge0 = DUT.n_merge; sweep_pf0 = DUT.n_pf;
        begin : sweep
            reg [25:0] b; b = 26'h2000000;
            for (int k = 0; k < 2000; k++) begin lookup(b); b = b + 26'd1; repeat (2) @(posedge clk); end
        end
        drain;
        $display("tb_tex_cache_pf sweep: 2000 lookups, hits %0d misses %0d (merges %0d), fetches %0d, prefetches %0d",
                 n_hit - sweep_hit0, n_miss - sweep_miss0, DUT.n_merge - sweep_merge0, fetches - sweep_fetch0, DUT.n_pf - sweep_pf0);
        if (PF && (DUT.n_merge - sweep_merge0) + (n_hit - sweep_hit0) < 600) begin
            $display("FAIL: the prefetch carries too little of the sweep"); errs++;
        end
        if (!PF && (DUT.n_pf != 0 || DUT.n_merge != 0)) begin $display("FAIL: PF 0 prefetched"); errs++; end
        if (expq.size() != 0) begin $display("FAIL: %0d results missing", expq.size()); errs++; end
        if (errs == 0) $display("tb_tex_cache_pf: PASS (SETS %0d HASH %0d PF %0d)", SETS, HASH, PF);
        else $display("tb_tex_cache_pf: FAIL (%0d errors)", errs);
        $finish;
    end
    initial begin #80000000; $display("tb_tex_cache_pf: TIMEOUT (issued %0d done %0d)", issued, done); $finish; end
endmodule
