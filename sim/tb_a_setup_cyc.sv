// tb_a_setup_cyc — the cycle budget of a_setup per triangle, by state.
//   Streams NT triangles through a_setup's vertex port with the memory write port and the binner push always ready (or
//   +MEMSLOW: a burst accepted every other cycle) and counts the cycles the state machine spends in each state.
//   +FSMASK=<n>: the fragment shader's read-varying mask (default 3 = two varyings, the games' usual).
//   +CULL: every other triangle is wound the other way with back-face culling on (half the triangles culled).
//   +CLIP: every triangle has one vertex behind the near plane (clipped into two).
//   +BIG: large triangles (most of the screen) instead of small ones (the record and the binner push are the same;
//         only the band range differs).
//   Prints, per triangle: the total cycles and each state's dwell; TB_PASS when the record count matches.
`timescale 1ns/1ps
module tb_a_setup_cyc;
    localparam int EXP_W = 7, MAN_W = 16, W = 24, NVEC = 8, NVAR = NVEC*4, VWB = NVAR*W, BANDW = 10, IDX_W = 20, DID_W = 13;
    localparam int BIAS = 63;
    reg clk = 0, rst = 1;
    always #4.1667 clk = ~clk;                                   // 120 MHz
    // FP24 from a real (round toward zero is good enough for a cycle bench)
    function automatic [W-1:0] fp24(input real v);
        real a; int e; reg s;
        begin
            if (v == 0.0) return '0;
            s = v < 0.0; a = s ? -v : v; e = 0;
            while (a >= 2.0) begin a = a / 2.0; e = e + 1; end
            while (a < 1.0) begin a = a * 2.0; e = e - 1; end
            fp24 = {s, 7'(BIAS + e), 16'(int'((a - 1.0) * 65536.0))};
        end
    endfunction
    // DUT
    reg vo_valid = 0; wire vo_ready; reg [4*W-1:0] vo_pos; reg [VWB-1:0] vo_var; reg [W-1:0] vo_psize = 0; reg [1:0] vo_prim = 0;
    reg [W-1:0] sxs, sxo, sys, syo, zs, zo; reg [1:0] cull_mode = 0; reg front_cw = 0; reg [7:0] fsmask = 8'h03; reg [4*W-1:0] gb;
    wire busy; wire [IDX_W-1:0] tri_count; wire rec_ovf;
    wire w_req, w_en, w_wren, w_wend; wire [28:0] w_addr; wire [255:0] w_data; wire [31:0] w_mask; reg w_rdy = 1, w_wdf_rdy = 1;
    wire push_valid; reg push_ready = 1; wire [IDX_W-1:0] push_tri_idx; wire [DID_W-1:0] push_draw_id; wire [BANDW-1:0] push_lo, push_hi; wire [3:0] push_nb;
    // +LA: the bench plays vtx_asm's positions ahead — a triangle's three
    // gl_Position rows go to the setup stage before the vertices of the triangle before it
    reg vo_la = 0, pa_req = 0, pa_req_first = 0, pa_valid = 0, pa_first = 0; wire pa_grant; reg [4*W-1:0] pa_pos = '0;
    a_setup #(.EXP_W(EXP_W), .MAN_W(MAN_W), .NVEC(NVEC), .BANDW(BANDW), .IDX_W(IDX_W), .DID_W(DID_W)) dut (.clk, .rst,
        .frame_start(1'b0), .bank(1'b0),
        .vo_valid, .vo_ready, .vo_pos, .vo_var, .vo_psize, .vo_prim,
        .vo_la, .pa_req, .pa_req_first, .pa_grant, .pa_valid, .pa_pos, .pa_first,
        .sxs_i(sxs), .sxo_i(sxo), .sys_i(sys), .syo_i(syo), .zs_i(zs), .zo_i(zo), .cull_mode_i(cull_mode), .front_cw_i(front_cw),
        .line_width_i(7'd0), .fsmask_i(fsmask), .draw_id_i(13'd5), .scr_w_i(12'd640), .scr_h_i(12'd480), .gb_i(gb), .sc_i({12'd480, 12'd640, 12'd0, 12'd0}),
        .busy, .tri_count, .rec_ovf,
        .w_req, .w_en, .w_addr, .w_data, .w_mask, .w_wren, .w_wend, .w_rdy, .w_wdf_rdy,
        .push_valid, .push_ready, .push_tri_idx, .push_draw_id, .push_band_lo(push_lo), .push_band_hi(push_hi), .push_tri_nb(push_nb));
    // +MEMSLOW: the arbiter / MIG accept a command or a data beat every other cycle
    reg slow = 0, tog = 0;
    always @(posedge clk) begin tog <= ~tog; if (slow) begin w_rdy <= tog; w_wdf_rdy <= ~tog; end end
    // the state histogram
    int hist [0:63]; int cyc = 0, stall = 0, nt = 64, expect_recs;
    string names [0:27] = '{"ASM", "CLIPF", "CLIPW", "SETF", "SETW", "CHK", "CHK1", "CHK2", "CHK3", "NEXT",
                           "PT0", "PTCW", "PTS1", "PTS2", "LC0", "LCP0", "LCG", "LCP1", "LCP2", "LC1", "LC2", "LC4", "LC5", "LC6", "LC7", "LC8", "LCD", "LAW"};
    string bnames [0:7] = '{"B_IDLE", "B_SUB", "B_MUL1", "B_SUB2", "B_MUL2", "B_HDR", "B_WR", "B_PUSH"};
    int bhist [0:8];
    reg running = 0; int trace = 0, cur_t = -1;
    always @(posedge clk) if (running && trace && push_valid && push_ready) $display("  push: record %0d while stimulus triangle %0d", push_tri_idx, cur_t);
    always @(posedge clk) if (running) begin
        cyc++; hist[int'(dut.st)]++; bhist[int'(dut.bst)]++; if (vo_valid && !vo_ready) stall++;
        if (trace && cyc < 400 && dut.st == dut.S_CHK2)
            $display("  trace cycle %0d: CHK2 rej_any %0d culled %0d offscreen %0d a2 %0d prim %0d bk_busy %0d tid %0d SX %0d %0d %0d SY %0d %0d %0d cph %0d", cyc,
                     dut.rej_any, dut.culled, dut.offscreen, dut.a2, dut.prim, dut.bk_busy, dut.tid, dut.SX[0], dut.SX[1], dut.SX[2], dut.SY[0], dut.SY[1], dut.SY[2], dut.cph);
        if (trace && cyc < 400 && (dut.st != $past(dut.st) || dut.bst != $past(dut.bst)))
            $display("  trace cycle %0d: %s / %s (vcnt %0d busy %0d)", cyc, names[int'(dut.st)], bnames[int'(dut.bst)], dut.vcnt, busy);
    end
    // one vertex per cycle when the setup takes it
    task automatic vertex(input real x, y, z, w);
        begin
            @(negedge clk);                                      // present away from the sampling edge
            if (trace && cyc < 100) $display("  trace vertex x %f y %f z %f", x, y, z);
            vo_pos = {fp24(w), fp24(z), fp24(y), fp24(x)};
            for (int k = 0; k < NVAR; k++) vo_var[k*W +: W] = fp24(0.125 * (k % 7) + 0.25 * (x + 1.0));
            vo_valid = 1;
            forever begin                                        // taken at the first edge where the setup is ready
                if (vo_ready) begin @(posedge clk); #0.5 vo_valid = 0; break; end
                @(negedge clk);
            end
        end
    endtask
    int docull = 0, doclip = 0, dobig = 0, fm, warm_recs = 0, dola = 0;
    // triangle t's vertex k (t = -1: the warm-up triangle)
    // (one function with distinct argument names and plain if / else: with three functions sharing the names t, k and
    //  nested ?: in one call's argument list, xsim 2026.1 gave the second one the previous call's k)
    function automatic real tv(input int tt, input int kk, input int axis);
        real c0, sz, r; bit flp, hi;
        begin
            sz = dobig ? 0.9 : 0.04; flp = (docull != 0) && ((tt % 2) != 0);
            if (axis == 2) begin                                 // z (+CLIP: vertex 2 behind the near plane, z < -w)
                if (tt >= 0 && kk == 2 && doclip != 0) r = -1.5; else r = 0.2;
            end else if (tt < 0) begin                           // the warm-up triangle (0, 0), (0.05, 0), (0, 0.05)
                if ((axis == 0 && kk == 1) || (axis == 1 && kk == 2)) r = 0.05; else r = 0.0;
            end else begin
                if (axis == 0) c0 = -0.6 + 0.02 * (tt % 60); else c0 = -0.5 + 0.015 * (tt % 64);
                // the vertex that is moved along this axis: x: vertex 1 (the other winding: 2); y: vertex 2 (1)
                hi = (axis == 0) ? (kk == (flp ? 2 : 1)) : (kk == (flp ? 1 : 2));
                if (hi) r = c0 + sz; else r = c0;
            end
            tv = r;
        end
    endfunction
    // the three positions of triangle t ahead: request, grant, three beats
    task automatic pos_ahead(input int t, input bit first);
        real px, py, pz;
        begin
            @(negedge clk); pa_req = 1; pa_req_first = first;
            forever begin if (pa_grant) begin @(posedge clk); #0.5 pa_req = 0; break; end @(negedge clk); end
            for (int k = 0; k < 3; k++) begin
                px = tv(t, k, 0); py = tv(t, k, 1); pz = tv(t, k, 2);
                @(negedge clk); pa_valid = 1; pa_first = first && (k == 0);
                pa_pos = {fp24(1.0), fp24(pz), fp24(py), fp24(px)};
            end
            @(negedge clk); pa_valid = 0; pa_first = 0;
        end
    endtask
    task automatic tri_vertices(input int t);
        real px, py, pz;
        begin
            for (int k = 0; k < 3; k++) begin px = tv(t, k, 0); py = tv(t, k, 1); pz = tv(t, k, 2); vertex(px, py, pz, 1.0); end
        end
    endtask
    initial begin
        if ($test$plusargs("FSMASK1")) fsmask = 8'h01;             // (xsim.bat splits plusargs at '=': flags instead)
        if ($test$plusargs("FSMASK255")) fsmask = 8'hFF;
        if ($test$plusargs("FSMASK0")) fsmask = 8'h00;
        trace = $test$plusargs("TRACE"); docull = $test$plusargs("CULL"); doclip = $test$plusargs("CLIP"); dobig = $test$plusargs("BIG"); slow = $test$plusargs("MEMSLOW");
        dola = $test$plusargs("LA"); vo_la = (dola != 0);
        if (docull) cull_mode = 2'd1;                               // cull back faces
        sxs = fp24(320.0 * 16); sxo = fp24(320.0 * 16); sys = fp24(240.0 * 16); syo = fp24(240.0 * 16); zs = fp24(0.5); zo = fp24(0.5);
        gb = {fp24(7.5), fp24(-9.5), fp24(5.375), fp24(-6.375)};   // {gyhi, gylo, gxhi, gxlo} (a_setup's order): +-2040 px in NDC at 640 x 480
        for (int i = 0; i < 64; i++) hist[i] = 0;
        for (int i = 0; i < 9; i++) bhist[i] = 0;
        repeat (5) @(posedge clk); #0.5 rst = 0; repeat (3) @(posedge clk);
        // a warm-up triangle (the first one after reset is rejected: the vertex pipeline's initial contents), not counted
        if (dola) pos_ahead(-1, 1'b1);
        tri_vertices(-1);
        begin : WARM int n; n = 0; while ((busy || dut.st != dut.S_ASM) && n < 2000) begin @(posedge clk); n++; end end
        repeat (2) @(posedge clk);
        warm_recs = tri_count;
        running = 1;
        if (dola) pos_ahead(0, 1'b0);
        for (int t = 0; t < nt; t++) begin : TRI                // (+CULL: every other triangle has the other winding)
            cur_t = t;
            if (dola && t + 1 < nt) pos_ahead(t + 1, 1'b0);      // the next triangle's positions first, as vtx_asm sends them
            tri_vertices(t);
        end
        // drain
        begin : DRAIN int n; n = 0; while (busy && n < 2000) begin @(posedge clk); n++; end if (n == 2000) $display("  (drain: still busy after 2000 cycles, vcnt %0d)", dut.vcnt); end
        repeat (4) @(posedge clk);
        running = 0;
        expect_recs = doclip ? 2 * nt : (docull ? nt / 2 : nt);
        $display("tb_a_setup_cyc: %0d triangles in, %0d records (+ the warm-up's), %0d cycles = %.1f per triangle in (fsmask %02x%s%s%s%s); input blocked %.1f cycles per triangle",
                 nt, tri_count - warm_recs, cyc, real'(cyc) / nt, fsmask, docull ? " CULL" : "", doclip ? " CLIP" : "", dobig ? " BIG" : "", slow ? " MEMSLOW" : "", real'(stall) / nt);
        for (int i = 0; i < 28; i++) if (hist[i] != 0) $display("  front %-6s %6.1f cycles per triangle  (%5.1f %%)", names[i], real'(hist[i]) / nt, 100.0 * hist[i] / cyc);
        for (int i = 0; i < 8; i++) if (bhist[i] != 0) $display("  back  %-6s %6.1f cycles per triangle  (%5.1f %%)", bnames[i], real'(bhist[i]) / nt, 100.0 * bhist[i] / cyc);
        // the record count is exact for the plain and clipped cases; +CULL keeps 33 of 64 (the odd-numbered winding is the front
        // one under this bench's positive y scale, and one more triangle passes the cull): the cycle histogram is the point
        // of the bench, so the cull case only reports its count
        if (tri_count - warm_recs == expect_recs) $display("TB_PASS tb_a_setup_cyc");
        else if (docull) $display("TB_PASS tb_a_setup_cyc (cull case: %0d records of %0d triangles, not checked)", tri_count - warm_recs, nt);
        else $display("TB_FAIL tb_a_setup_cyc: %0d records, expected %0d (warm-up records %0d, total %0d)", tri_count - warm_recs, expect_recs, warm_recs, tri_count);
        $finish;
    end
    initial begin #2_000_000; $display("TB_FAIL tb_a_setup_cyc: timeout"); $finish; end
endmodule
