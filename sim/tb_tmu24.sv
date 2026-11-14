// tb_tmu24 — the pipelined texture unit tmu24 against tools/tmu_ref.py, bit for bit (python3 tools/tmu_ref.py fp24 ->
// tools/gen/tmu24_fp24_*.mem; -d FP16 for the FP16 build). 16 samplers in two phases (descriptors reloaded and the
// cache invalidated between them): random 2D / cube / 3D textures (POT and NPOT, 1..full levels, 4 formats, every filter
// and wrap combination, incomplete), random quads (implicit LOD from the quad, bias, explicit LOD, negative / large /
// edge coordinates; 2D samplers get a random r, which must change nothing). DDR model with random command acceptance and in-order latency 8..40 cycles. Requests with random
// gaps, then a 256-quad bilinear block back to back: its cycles per quad are printed (throughput). Then the level-table
// thrash case: the block's sampler alternating with sampler {bank 1, same slot}, which the
// bench maps to another 2D texture of a different size — the per-{bank, sampler} table must keep both valid (no 15-cycle
// recompute per request): bit-exact results and < 14 cycles per request.
`default_nettype none
module tb_tmu24;
`ifdef FP16
    localparam int EXP_W = 5, MAN_W = 10; localparam string TAG = "tmu24_fp16";
`else
    localparam int EXP_W = 7, MAN_W = 16; localparam string TAG = "tmu24_fp24";
`endif
    localparam int W = 1 + EXP_W + MAN_W, MEMW = 1 << 19, NMAX = 2048, NTP = 256;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    reg [31:0] mem [0:MEMW-1];
    reg [83:0] desc [0:31];                                // {vol, wrap r, depth, inc, cube, wrap t, wrap s, filt, levels, fmt, h, w, base}
    reg [6+16*W-1:0] reqm [0:NMAX-1];
    reg [16*W-1:0] expm [0:NMAX-1];
    reg [31:0] nq [0:1];
    integer phase = 0;

    reg req_valid = 0; wire req_ready; reg [4:0] req_s = 0; reg [1:0] req_m = 0; reg [8*W-1:0] req_uv = 0, req_zw = 0;
    wire [4:0] desc_idx; reg inval = 0;
    reg [3:0] alt_s = 0;                                   // the thrash case: sampler {bank 1, slot} = alt_s's descriptor
    wire [83:0] ds = desc_idx[4] ? desc[phase*16 + alt_s] : desc[phase*16 + desc_idx[3:0]];
    integer eidx [0:NMAX+2*NTP-1];                         // result n is compared with expm[eidx[n]] (identity until the thrash case)
    wire res_valid; wire [16*W-1:0] res;
    wire tmu_en; wire [28:0] tmu_addr; reg tmu_rdy = 0, tmu_rd_valid = 0; reg [255:0] tmu_rd_data = 0;
`ifdef TB_F2SINGLE
    localparam bit TB_F2D = 1'b0;                          // -Defines TB_F2SINGLE: F2 at one pixel per cycle (the F2_DUAL = 0 fallback)
`else
    localparam bit TB_F2D = 1'b1;
`endif
`ifdef TB_VOL3D
    localparam bit TB_V3 = 1'b1;                           // -Defines TB_VOL3D: the 3D texture path built; needs descriptors made
`else                                                      // with `tmu_ref.py fp24 --vol3d`. Default = the board's VOL3D 0 and vol-free descriptors
    localparam bit TB_V3 = 1'b0;
`endif
    // the texture cache's variants: -Defines TB_TC256 (32 KB), TB_TC_HASH, TB_TC_NOPF; default = the RTL's (64 KB, plain index, prefetch)
`ifdef TB_TC256
    defparam dut.TC_SETS = 256;
`endif
`ifdef TB_TC_HASH
    defparam dut.TC_HASH = 1'b1;
`endif
`ifdef TB_TC_NOPF
    defparam dut.TC_PF = 1'b0;
`endif
    // +MASK: every request with a random sample mask: the sampled pixels must equal the model's, the
    // others must be zero, and results stay >= 4 cycles apart; without it all ones
    bit mk_on; initial mk_on = $test$plusargs("MASK");
    reg [3:0] req_mask = 4'hF; reg [3:0] mkq [$];
    function automatic [3:0] new_mask(); new_mask = mk_on ? 4'($urandom_range(0, 15)) : 4'hF; endfunction
    tmu24 #(.EXP_W(EXP_W), .MAN_W(MAN_W), .F2_DUAL(TB_F2D), .VOL3D(TB_V3)) dut(.clk, .rst, .cache_inval(inval),
        .req_valid, .req_ready, .req_sampler(req_s), .req_mode(req_m), .req_uv, .req_zw, .req_mask,
        .desc_idx, .desc_base(ds[28:0]), .desc_w(ds[44:33]), .desc_h(ds[56:45]), .desc_fmt(ds[58:57]),
        .desc_levels(ds[62:59]), .desc_filt(ds[66:63]), .desc_wrap_s(ds[68:67]), .desc_wrap_t(ds[70:69]),
        .desc_cube(ds[71]), .desc_incomplete(ds[72]), .desc_vol(ds[83]), .desc_depth(ds[80:73]), .desc_wrap_r(ds[82:81]),
        .res_valid, .res_texels(res), .tmu_en, .tmu_addr, .tmu_rdy, .tmu_rd_valid, .tmu_rd_data);

    // ---- DDR model: random acceptance, in-order returns after 8..40 cycles ----
    longint now = 0, last_due = 0, n_ddr = 0; reg [28:0] dq_a [$]; longint dq_t [$];
    always @(posedge clk) begin
        now++;
        tmu_rdy <= ($urandom_range(0, 3) != 0);
        tmu_rd_valid <= 1'b0;
        if (tmu_en && tmu_rdy) begin
            longint d; d = now + $urandom_range(8, 40); if (d <= last_due) d = last_due + 1; last_due = d;
            dq_a.push_back(tmu_addr); dq_t.push_back(d); n_ddr++;
        end
        if (dq_a.size() != 0 && dq_t[0] <= now) begin
            for (int k = 0; k < 8; k++) tmu_rd_data[k*32 +: 32] <= mem[(dq_a[0] + k) % MEMW];
            tmu_rd_valid <= 1'b1; void'(dq_a.pop_front()); void'(dq_t.pop_front());
        end
    end

    // ---- results: in order against the model ----
    integer nout = 0, err = 0, ntot = 0; longint tp_t0 = 0, tp_t1 = 0, tp_tc = 0;
    longint res_last = -100, n_skip = 0, res_gap_min = 1000;
    always @(posedge clk) if (res_valid) begin : chk
        int e; reg [3:0] mk; reg [16*W-1:0] ex; reg bad;
        e = eidx[nout];
        mk = mkq.pop_front(); if (mk == 4'd0) mk = 4'd1;       // an empty mask samples pixel 0
        ex = (e >= 0) ? expm[e] : res;                         // eidx -1: a synthetic request (throughput only, its sampled pixels not compared)
        for (int p = 0; p < 4; p++) if (!mk[p]) begin ex[p*4*W +: 4*W] = '0; n_skip++; end
        if (now - res_last < 4) begin err++; $display("  quad %0d: result %0d cycles after the previous one", nout, now - res_last); end
        if (now - res_last < res_gap_min) res_gap_min = now - res_last;
        res_last = now;
        if (res !== ex) begin
            err++;
            if (err <= 6) begin
                $display("  quad %0d (request %0d, mask %b): got / exp", nout, e, mk);
                for (int p = 0; p < 4; p++)
                    $display("    px%0d  %h %h %h %h / %h %h %h %h", p, res[(p*4+0)*W +: W], res[(p*4+1)*W +: W], res[(p*4+2)*W +: W],
                             res[(p*4+3)*W +: W], ex[(p*4+0)*W +: W], ex[(p*4+1)*W +: W],
                             ex[(p*4+2)*W +: W], ex[(p*4+3)*W +: W]);
            end
        end
        nout++;
        if (nout == ntot - NTP) tp_tc = now;
        if (nout == ntot) tp_t1 = now;
    end

`ifdef TMUDBG
    // per-pixel filter sums (quad index = results so far), and the jobs of the filter: -d TMUDBG +DBGQ_<n>
    integer dq = -1, dbg_q = 0, nq_cv = 0;
    initial void'($value$plusargs("DBGQ_%d", dq));
    always @(posedge clk) begin
        if (dut.cv0_v) begin
            if (nq_cv == dq) $display("  dbg quad %0d px%0d P = %0d %0d %0d %0d", nq_cv, dut.cv0_p, dut.cv0_P[0], dut.cv0_P[1], dut.cv0_P[2], dut.cv0_P[3]);
            if (dut.cv0_lastpix) nq_cv++;
        end
        if (dut.fl1_v && nq_cv == dq)
            $display("  dbg job px%0d w %0d %0d %0d %0d | %0d %0d %0d %0d c %h %h %h %h | %h %h %h %h", dut.fl1_p,
                     dut.fl1_w[0], dut.fl1_w[1], dut.fl1_w[2], dut.fl1_w[3], dut.fl1_w[4], dut.fl1_w[5], dut.fl1_w[6], dut.fl1_w[7],
                     dut.fl1_c[0], dut.fl1_c[1], dut.fl1_c[2], dut.fl1_c[3], dut.fl1_c[4], dut.fl1_c[5], dut.fl1_c[6], dut.fl1_c[7]);
    end
`endif
    // a positive real -> FP24 bits (sign 0; the bench's synthetic coordinates)
    function automatic [W-1:0] f24(input real v);
        integer e; real m;
        e = 0; m = v;
        while (m >= 2.0) begin m = m / 2.0; e++; end
        while (m < 1.0)  begin m = m * 2.0; e--; end
        f24 = {1'b0, EXP_W'(e + ((1 << (EXP_W - 1)) - 1)), MAN_W'($rtoi((m - 1.0) * (2.0 ** MAN_W)))};
    endfunction
    // a synthetic quad for sampler s: pixel (px, py) at texture coordinate ((3 (2 qx + px) + 0.5) / 64, (3 (2 qy + py) + 0.5) / 64)
    // -> 3 texels per pixel on a 64 x 64 level 0 -> lambda = log2 3 = 1.58: a two-level (trilinear) pixel on every quad
    task automatic send_tri(input int s, input int qx, input int qy);
        req_s <= 5'(s); req_m <= 2'd0;
        for (int p = 0; p < 4; p++) begin
            req_uv[p*2*W +: W]   <= f24((3.0 * (2 * qx + p[0]) + 0.5) / 64.0);
            req_uv[p*2*W+W +: W] <= f24((3.0 * (2 * qy + p[1]) + 0.5) / 64.0);
            req_zw[p*2*W +: W] <= '0; req_zw[p*2*W+W +: W] <= '0;
        end
        begin reg [3:0] m; m = new_mask(); req_mask <= m; mkq.push_back(m); end
        req_valid <= 1'b1;
        do @(posedge clk); while (!req_ready);
        req_valid <= 1'b0;
    endtask
    task automatic send(input int i, input int sov = -1);       // sov >= 0: send request i as sampler sov (the thrash case)
        reg [16*W-1:0] c;
        c = reqm[i][16*W-1:0];
        req_s <= (sov < 0) ? {1'b0, reqm[i][6+16*W-1 -: 4]} : 5'(sov); req_m <= reqm[i][16*W +: 2];
        for (int p = 0; p < 4; p++) begin
            req_uv[p*2*W +: W] <= c[(p*4+0)*W +: W]; req_uv[p*2*W+W +: W] <= c[(p*4+1)*W +: W];
            req_zw[p*2*W +: W] <= c[(p*4+2)*W +: W]; req_zw[p*2*W+W +: W] <= c[(p*4+3)*W +: W];
        end
        begin reg [3:0] m; m = new_mask(); req_mask <= m; mkq.push_back(m); end
        req_valid <= 1'b1;
        do @(posedge clk); while (!req_ready);
        req_valid <= 1'b0;
    endtask

    int tp_first = 0;                                      // the throughput block's first request (reused by the latency probe)
    longint n_jobs = 0;                                    // tap jobs entering T0 (one per pixel)
    always @(posedge clk) if (dut.t_en && dut.f4_v) n_jobs++;
    // +TRIDBG: a compact state trace while tri_dbg is set (printed only when the state changes)
    reg tri_dbg = 0; string tri_prev = "";
    always @(posedge clk) if (tri_dbg && $test$plusargs("TRIDBG")) begin : tdb
        string s;
        s = $sformatf("rq%0d f1%b lt%b wf%0d f3%b%b f4%b t0%b t4%b ta%0d hv%b need%b alloc%b push%b lq%0d jf%0d rdy%b fl0%b cv0%b res%b ap%0d ref0..3=%0d,%0d,%0d,%0d",
                      dut.rq_t - dut.rq_h, dut.f1_v, dut.lt_busy, dut.wf_t - dut.wf_h, dut.f3a_v, dut.f3f_v, dut.f4_v, dut.t0_v, dut.t4_v, dut.ta_n,
                      dut.hv, dut.a_need, dut.a_alloc, dut.a_push, dut.lq_n, dut.jf_t - dut.jf_h, dut.fl_rdy, dut.fl0_v, dut.cv0_v, res_valid, dut.bt_ap,
                      dut.bt_ref[0], dut.bt_ref[1], dut.bt_ref[2], dut.bt_ref[3]);
        if (s != tri_prev) begin $display("  t%0d %s", now, s); tri_prev = s; end
    end
    initial begin
        int i0;
        $readmemh({"tools/gen/", TAG, "_mem.mem"}, mem);
        $readmemh({"tools/gen/", TAG, "_desc.mem"}, desc);
        $readmemh({"tools/gen/", TAG, "_req.mem"}, reqm);
        $readmemh({"tools/gen/", TAG, "_exp.mem"}, expm);
        $readmemh({"tools/gen/", TAG, "_n.mem"}, nq);
        for (int i = 0; i < NMAX + 2*NTP; i++) eidx[i] = (i < NMAX) ? i : 0;
        repeat (4) @(posedge clk); rst <= 0; @(posedge clk);
        // phase 0: random gaps
        ntot = nq[0];
        for (int i = 0; i < nq[0]; i++) begin
            send(i);
            repeat ($urandom_range(0, 3) == 0 ? $urandom_range(1, 20) : 0) @(posedge clk);
        end
        while (nout < nq[0] && now < 2000000) @(posedge clk);
        // reload the descriptors, invalidate the cache
        repeat (5) @(posedge clk); phase = 1; inval <= 1'b1; @(posedge clk); inval <= 1'b0; @(posedge clk);
        ntot = nq[0] + nq[1];
        i0 = nq[0];
        for (int i = i0; i < ntot - 2*NTP; i++) begin
            send(i);
            repeat ($urandom_range(0, 3) == 0 ? $urandom_range(1, 10) : 0) @(posedge clk);
        end
        while (nout < ntot - 2*NTP && now < 2000000) @(posedge clk);
        // throughput: the same 256 bilinear quads twice, back to back (cold cache, then warm)
        tp_first = ntot - 2*NTP;
        tp_t0 = now;
        for (int i = ntot - 2*NTP; i < ntot; i++) send(i);
        while (nout < ntot && now < 3000000) @(posedge clk);
        repeat (10) @(posedge clk);
        $display("TMU24 %s: bilinear 64x64, %0d quads: cold cache %.2f, warm %.2f cycles / quad",
                 TAG, NTP, real'(tp_tc - tp_t0) / NTP, real'(tp_t1 - tp_tc) / NTP);
        // ---- the level-table thrash case: the block's sampler s_tp (bank 0) alternating with {bank 1, s_tp},
        //      which ds maps to alt_s = another complete 2D sampler of phase 1 with other dimensions; alt_s's own phase-1
        //      requests are resent under the bank-1 name, so their expected results are already known
        begin : thrash
            int s_tp, cnt, tp0, ntot0; int i_alt [$]; longint t0; real cpr; reg [83:0] dtp;
            s_tp = reqm[ntot - 1][6+16*W-1 -: 4]; dtp = desc[16 + s_tp]; alt_s = 4'hF;
            begin : pick                                   // the complete 2D sampler of other dimensions with the most phase-1 requests
                int ns [0:15], best; reg [83:0] d;
                for (int s = 0; s < 16; s++) ns[s] = 0;
                for (int i = nq[0]; i < ntot - 2*NTP; i++) ns[reqm[i][6+16*W-1 -: 4]]++;
                best = 0;
                for (int s = 0; s < 16; s++) begin
                    d = desc[16 + s];
                    if (s != s_tp && !d[71] && !d[72] && !d[83] && (d[44:33] != dtp[44:33] || d[56:45] != dtp[56:45]) && ns[s] > best) begin alt_s = 4'(s); best = ns[s]; end
                end
            end
            for (int i = nq[0]; i < ntot - 2*NTP; i++) if (reqm[i][6+16*W-1 -: 4] == alt_s) i_alt.push_back(i);
            cnt = (i_alt.size() < 64) ? i_alt.size() : 64;
            if (alt_s == 4'hF || cnt < 8) $display("  thrash case skipped (second 2D sampler %0d, %0d requests)", alt_s, cnt);
            else begin
                tp0 = ntot - 2*NTP; ntot0 = ntot;
                for (int k = 0; k < cnt; k++) begin eidx[ntot0 + 2*k] = tp0 + k; eidx[ntot0 + 2*k + 1] = i_alt[k]; end
                ntot = ntot0 + 2*cnt;
                t0 = now;
                for (int k = 0; k < cnt; k++) begin send(tp0 + k, s_tp); send(i_alt[k], 16 + s_tp); end
                while (nout < ntot && now < 4000000) @(posedge clk);
                cpr = real'(now - t0) / real'(2*cnt);
                $display("  level-table thrash case: %0d requests alternating sampler %0d and {bank 1, %0d} (sampler %0d's %0dx%0d texture): %.2f cycles per request",
                         2*cnt, s_tp, s_tp, alt_s, desc[16 + alt_s][44:33], desc[16 + alt_s][56:45], cpr);
                if (cpr > 14.0) begin err++; $display("  THRASH: more than 14 cycles per request (the level table recomputes per request)"); end
            end
        end
        // ---- the request-to-result latency of ONE bilinear quad in an empty pipeline: after a cache invalidate (DDR misses,
        //      the bench's 8..40-cycle model), then the same quad again (every burst resident) — the TEX round trip a warp waits for
        repeat (10) @(posedge clk);
        begin : lat
            longint t, lc, lw;
            eidx[ntot] = tp_first; eidx[ntot + 1] = tp_first; ntot = ntot + 2;
            inval <= 1'b1; @(posedge clk); inval <= 1'b0; repeat (4) @(posedge clk);
            t = now; send(tp_first); while (nout < ntot - 1 && now < 5000000) @(posedge clk); lc = now - t;
            repeat (4) @(posedge clk);
            t = now; send(tp_first); while (nout < ntot && now < 5000000) @(posedge clk); lw = now - t;
            $display("  isolated bilinear quad, request -> result: cold (DDR misses) %0d cycles, warm (bursts resident) %0d cycles", lc, lw);
        end
        // ---- trilinear throughput: the throughput block's sampler is REDESCRIBED as 64 x 64 with 7 levels and
        //      LINEAR_MIPMAP_LINEAR (its level-0 data is the 64 x 64 texture; the higher levels read whatever follows in
        //      memory — these results are NOT compared, eidx -1), and synthetic quads stepping 3 texels per pixel (lambda =
        //      log2 3: two levels for every pixel) are sent twice back to back. The steady rate on the warm pass = the
        //      TMU's cycles per trilinear quad: ≈ 4 with one 8-tap job per pixel, ≈ 8 with a two-job filter.
        repeat (10) @(posedge clk);
        begin : trib                                       // ('tri' is a net-type keyword)
            localparam int NTRI = 64;
            int ts; longint t0, t1, tf, j0, d0, d1; real cw, cc;
            ts = reqm[tp_first][6+16*W-1 -: 4];
            desc[16 + ts] = {1'b0, 2'd0, 8'd1, 1'b0, 1'b0, 2'd0, 2'd0, 4'b1101, 4'd7, 2'd0, 12'd64, 12'd64, 4'd0, desc[16 + ts][28:0]};
            for (int k = 0; k < 2*NTRI; k++) eidx[ntot + k] = -1;
            ntot = ntot + 2*NTRI;
            t0 = now; d0 = n_ddr; for (int k = 0; k < NTRI; k++) send_tri(ts, k % 8, k / 8);
            while (nout < ntot - NTRI && now < 6000000) @(posedge clk);
            t1 = now; j0 = n_jobs; d1 = n_ddr; tri_dbg = 1'b1;
            fork
                for (int k = 0; k < NTRI; k++) send_tri(ts, k % 8, k / 8);
                begin
                    while (nout < ntot - NTRI + 1 && now < 6000000) @(posedge clk);  // the warm pass's first result
                    tf = now; tri_dbg = 1'b0;
                end
            join
            while (nout < ntot && now < 6000000) @(posedge clk);                 // ... and its last: the steady rate between them
            cc = real'(t1 - t0) / real'(NTRI); cw = real'(now - tf) / real'(NTRI - 1);
            $display("  trilinear block: %0d synthetic two-level quads (64x64, 7 levels, 3 texels per pixel) back to back: cold %.2f cycles per quad incl. latency (%0d DDR reads), warm steady rate %.2f cycles per quad, %.2f tap jobs per quad (warm pass: %0d DDR reads, first result %0d cycles after the first send, last %0d)",
                     NTRI, cc, d1 - d0, cw, real'(n_jobs - j0) / real'(NTRI), n_ddr - d1, tf - t1, now - t1);
            // reference rates: 7.17 with one job per pixel (4 jobs per quad, the burst buffer's one allocation per cycle is the
            // limit on this 3-texel stride), 10.97 with a two-job filter (8 jobs); a result above 9 means the two-job path
            if (cw > 9.0) $display("  NOTE: the trilinear rate is above 9 cycles per quad (C1 measured 7.17; the two-job filter 10.97)");
        end
        repeat (10) @(posedge clk);
        $display("TMU24 %s: %0d quads, errors = %0d; sample masks %s: %0d pixels not sampled (checked zero), tap jobs %0d, closest results %0d cycles apart",
                 TAG, nout, err, mk_on ? "random" : "all ones", n_skip, n_jobs, res_gap_min);
        if (err == 0 && nout == ntot) $display("TB_PASS tb_tmu24"); else $display("TB_FAIL tb_tmu24 (%0d of %0d quads)", nout, ntot);
        $finish;
    end
    initial begin #40ms; $display("TB_FAIL tb_tmu24: TIMEOUT (%0d quads out)", nout); $finish; end
endmodule
`default_nettype wire
