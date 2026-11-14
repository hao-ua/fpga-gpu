// tb_fs2 — the scalar fragment core (shader_core_fs2) runs one program on one warp and is
// compared with tools/isa64.py's reference (`python3 tools/isa64.py fs2 tests/<prog>.sasm [--interp]` ->
// tools/gen/<prog>_fs2_*.mem). Program, uniforms and the built-in rows go in through the runtime load ports; checks
// every scalar register the core wrote (a register still X was never written: the reference must hold 0),
// out_color, the kill mask and gl_FragDepth.
//   +PROG_<prog> [+TOL_<ulp>] [+ACTIVE_<hex16>] [+WARPS_<n>] [+TEXLAT_<n>]   (no "=" in the plusargs: sim_gpu.ps1
//   -Plusargs PROG_fs2_step2,WARPS_4). WARPS n (1..4): the program is launched n times back to back (the same inputs
//   on every warp), so the warps interleave in the core; every warp's results must match the golden. TEX goes to the
//   texture-unit model below (tmu_model2, = isa64.tex_model; random ready and latency 3..TEXLAT, default 40).
//   +OUTONLY: compare only the program's outputs (colour, depth, kill), not the registers — for compiled shaders,
//   whose unused lanes (a vec3's .w) hold undefined values that the approximate SFU models need not reproduce.
`default_nettype none
module tb_fs2;
    localparam int EXP_W = 7, MAN_W = 16, W = 1 + EXP_W + MAN_W, TW = 4 * W, LW = 16 * W, RW = 16 * TW;
`ifdef TB_NWARP8
    localparam int NPARAM = 64, NWARP = 8;                 // -Defines TB_NWARP8: the core with 8 warps (latency-hiding experiments;
`else                                                      // a value-less define — the .bat wrappers split "NAME=VALUE" at the '=')
    localparam int NPARAM = 64, NWARP = 4;
`endif
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    reg         iram_we = 0, param_we = 0, vw_we = 0, in_start = 0, out_ack = 0;
    reg         pre_start = 0;                   // +PRE_<k> (1..16): the program fetch ahead of the launch, k cycles before in_start
    reg  [12:0] iram_waddr = 0; reg [63:0] iram_wdata = 0;          // {bank, address}: the bench uses bank 0
    reg  [6:0]  param_idx = 0; reg [TW-1:0] param_data = 0;
    reg  [1:0]  vw_row = 0, vw_quad = 0; reg [8*W-1:0] vw_data = 0;
    reg  [15:0] in_active = 16'hFFFF; reg [11:0] in_tslot = 0; reg [3:0] in_front = 0;
    reg         tri_we = 0; reg [2:0] tri_slot = 0, tri_vidx = 0; reg [3*TW-1:0] tri_data = 0; wire tri_ready;
    wire        in_ready, out_valid, out_depthw, out_err, idle, err_stack; wire [1:0] in_warp;
    reg  [4:0]  out_sel = 0, out_sel2 = 5'd1; wire [TW-1:0] out_row, out_row2; wire [15:0] out_kill;   // even rows / odd rows (STO2)
    reg  [RW-1:0] out_color; reg [LW-1:0] out_depth;    // gathered row by row from the output store
    wire        tq_v, tq_rdy, tr_v; wire [4:0] tq_samp; wire [1:0] tq_mode; wire [8*W-1:0] tq_uv, tq_zw; wire [LW-1:0] tr_d;   // tq_samp[4] = the bank (0 here)
`ifdef TB_STO4
    localparam bit STO2 = 1'b0;                  // the 4-micro-op STO
`else
    localparam bit STO2 = 1'b1;
`endif
`ifdef TB_TEX2
    localparam bit TEX2 = 1'b1;                  // TEX coordinates one micro-op per pair
`else
    localparam bit TEX2 = 1'b0;                  // the board's: one TEX coordinate per micro-op
`endif
`ifdef TB_LDV2
    localparam int LDVM = 2;                     // -Defines TB_LDV2: one issued micro-op, a second multiplier and adder per lane
`elsif TB_LDV1
    localparam int LDVM = 1;                     // one issued micro-op, the second step by itself in the same lanes
`else
    localparam int LDVM = 0;                     // the board's (also -Defines TB_LDV0): two issued micro-ops per component
`endif
    // both extensions (XSFU, XCLAMP) are in the bench's core: the boards build SQRT / RCPM / RSQM, CLAMP is off there
    shader_core_fs2 #(.EXP_W(EXP_W), .MAN_W(MAN_W), .NWARP(NWARP), .NPARAM(NPARAM), .STO2(STO2), .TEX2(TEX2), .LDV_MODE(LDVM),
        .XSFU(1'b1), .XCLAMP(1'b1),
`ifdef TB_QRINF2
        .QR_INF(2),                              // -Defines TB_QRINF2: two quarter-rate micro-ops in flight per class
`endif
`ifdef TB_ADDDSP
        .ADD_DSP(1'b1)
`else
        .ADD_DSP(1'b0)
`endif
        ) dut (
        .clk, .rst, .iram_we, .iram_waddr, .iram_wdata, .param_we, .param_idx, .param_data,
        .vw_we, .vw_row, .vw_quad, .vw_data, .tri_we, .tri_ready, .tri_slot, .tri_vidx, .tri_data,
        .in_ready, .in_warp, .in_start, .in_bank(1'b0), .in_active, .in_front, .in_tslot, .pre_start, .pre_bank(1'b0),
        .tex_req_valid(tq_v), .tex_req_ready(tq_rdy), .tex_req_sampler(tq_samp), .tex_req_mode(tq_mode),
        .tex_req_uv(tq_uv), .tex_req_zw(tq_zw), .tex_res_valid(tr_v), .tex_res_texels(tr_d),
        .out_valid, .out_ack, .out_sel, .out_row, .out_sel2, .out_row2, .out_depthw, .out_kill, .out_err, .err_stack, .idle);
    tmu_model2 #(.W(W)) tmu (.clk, .rst, .req_valid(tq_v), .req_ready(tq_rdy), .req_sampler(tq_samp[3:0]), .req_mode(tq_mode),
        .req_uv(tq_uv), .req_zw(tq_zw), .res_valid(tr_v), .res_texels(tr_d));

    reg [63:0]   iram  [0:4095];
    reg [TW-1:0] prm   [0:NPARAM-1];
    reg [RW-1:0] vary  [0:15];                   // rows 0..15, lane 15 first (w z y x per lane)
    reg [RW-1:0] expg  [0:31];
    reg [RW-1:0] outc  [0:0];
    reg [15:0]   kill  [0:0];
    reg [LW-1:0] outz  [0:0];
    reg [0:0]    outzw [0:0], experr [0:0];
    reg [3*TW-1:0] tst [0:63];                   // slot s, varying i at s*8+i: {D2, D1, f0}
    reg [15:0]   tslot [0:0];                    // quad 3 .. 0, one hex digit each
    reg [16*2*W-1:0] lam [0:0];                  // lane 15 .. 0, each {l2, l1}
    function automatic int ulp_diff(input [W-1:0] a, input [W-1:0] b);
        int ia, ib;
        ia = a[W-1] ? -int'(a[W-2:0]) : int'(a[W-2:0]);
        ib = b[W-1] ? -int'(b[W-2:0]) : int'(b[W-2:0]);
        return (ia > ib) ? ia - ib : ib - ia;
    endfunction
    function automatic bit close(input [RW-1:0] got, input [RW-1:0] exp, input int tol);
        for (int k = 0; k < 64; k++)
            if ($isunknown(got[k*W +: W])) begin if (exp[k*W +: W] != '0) return 1'b0; end
            else if (ulp_diff(got[k*W +: W], exp[k*W +: W]) > tol) return 1'b0;
        return 1'b1;
    endfunction
    // register r of warp w as the golden packs it: lane l, component c at [(l*4+c)*W +: W]
    function automatic [RW-1:0] gpr_of(input int w, input int r);
        for (int l = 0; l < 16; l++)
            for (int c = 0; c < 4; c++)
                gpr_of[(l*4+c)*W +: W] = dut.gpr_peek(l, w*128 + r*4 + c);
    endfunction

    initial begin
        string prog; int tol, n, err, cyc, act, nw, nout, pre_k; bit outonly;
        tol = 0; act = 16'hFFFF; nw = 1; pre_k = 0;
        void'($value$plusargs("PRE_%d", pre_k));
        if (!$value$plusargs("PROG_%s", prog)) begin $display("FAIL: no +PROG_<name>"); $finish; end
        void'($value$plusargs("TOL_%d", tol)); void'($value$plusargs("ACTIVE_%h", act)); void'($value$plusargs("WARPS_%d", nw));
        outonly = $test$plusargs("OUTONLY");
        if (nw < 1 || nw > NWARP) nw = 1;
        for (int k = 0; k < 32; k++) expg[k] = '0;
        $readmemh({"tools/gen/", prog, "_fs2_iram.mem"}, iram);
        $readmemh({"tools/gen/", prog, "_fs2_params.mem"}, prm);
        $readmemh({"tools/gen/", prog, "_fs2_varin.mem"}, vary);
        $readmemh({"tools/gen/", prog, "_fs2_expgpr.mem"}, expg);
        $readmemh({"tools/gen/", prog, "_fs2_outc.mem"}, outc);
        $readmemh({"tools/gen/", prog, "_fs2_kill.mem"}, kill);
        $readmemh({"tools/gen/", prog, "_fs2_outz.mem"}, outz);
        $readmemh({"tools/gen/", prog, "_fs2_outzw.mem"}, outzw);
        $readmemh({"tools/gen/", prog, "_fs2_err.mem"}, experr);
        $readmemh({"tools/gen/", prog, "_fs2_tstore.mem"}, tst);
        $readmemh({"tools/gen/", prog, "_fs2_tslot.mem"}, tslot);
        $readmemh({"tools/gen/", prog, "_fs2_lam.mem"}, lam);
        repeat (4) @(posedge clk); rst <= 0;
        n = 0; while (n < 4096 && !$isunknown(iram[n])) n++;
        for (int k = 0; k < n; k++) begin @(posedge clk); iram_we <= 1; iram_waddr <= 13'(k); iram_wdata <= iram[k]; end
        for (int k = 0; k < NPARAM; k++) begin @(posedge clk); iram_we <= 0; param_we <= 1; param_idx <= 7'(k); param_data <= prm[k]; end
        @(posedge clk); param_we <= 0;
        // the triangle store (every slot and varying; 4 cycles per entry)
        for (int k = 0; k < 64; k++) begin
            @(posedge clk); tri_we <= 1; tri_slot <= 3'(k >> 3); tri_vidx <= 3'(k & 7); tri_data <= tst[k];
            @(posedge clk); tri_we <= 0;
            repeat (4) @(posedge clk);                       // the writer's 4 component cycles (tri_ready low)
        end
        @(posedge clk);
        if ($test$plusargs("DBG"))
            for (int k = 0; k < 12; k++) $display("DBG tstore entry %0d (slot %0d vidx %0d comp %0d) = %h", k, k >> 5, (k >> 2) & 7, k & 3, dut.TS[0].tmem[k]);
        // nw warps back to back: the launch rows (two components per pixel, one quad per write) from the golden's
        // built-in rows — row 0 gl_FragCoord.xy, 1 .zw, 2 gl_PointCoord.xy (mu1, mu2), 3 the weights {l2, l1};
        // gl_FrontFacing = row 9 .x of the quad's first pixel; launch with the quads' slots
        cyc = 0; err = 0; nout = 0;
        for (int w = 0; w < nw; w++) begin
            while (!in_ready) @(posedge clk);
            for (int r = 0; r < 4; r++)
                for (int q = 0; q < 4; q++) begin
                    @(posedge clk); vw_we <= 1; vw_row <= 2'(r); vw_quad <= 2'(q);
                    pre_start <= (pre_k != 0) && (r*4 + q == 16 - pre_k);   // (write i is 16 - i cycles before in_start)
                    for (int p = 0; p < 4; p++)
                        case (r)
                            0: vw_data[p*2*W +: 2*W] <= vary[8][((4*q + p)*4 + 0)*W +: 2*W];
                            1: vw_data[p*2*W +: 2*W] <= vary[8][((4*q + p)*4 + 2)*W +: 2*W];
                            2: vw_data[p*2*W +: 2*W] <= vary[10][((4*q + p)*4 + 0)*W +: 2*W];
                            default: vw_data[p*2*W +: 2*W] <= lam[0][(4*q + p)*2*W +: 2*W];
                        endcase
                end
            @(posedge clk); vw_we <= 0; pre_start <= 0; in_start <= 1; in_active <= 16'(act);
            for (int q = 0; q < 4; q++) begin
                in_tslot[q*3 +: 3] <= tslot[0][q*4 +: 3];
                in_front[q] <= (vary[9][(4*q*4)*W +: W] != '0);
            end
            @(posedge clk); in_start <= 0;
        end
        // collect nw results in launch order; each warp's registers are checked while its result is offered
        while (nout < nw && cyc < 200000) begin
            @(posedge clk); cyc++;
            if (out_valid) begin
                #1;
                for (int q = 0; q < 4; q++) begin                   // the output store, row by row
                    for (int c = 0; c < 4; c += 2) begin            // components c (out_sel) and c + 1 (out_sel2), as the back end reads them
                        out_sel = {2'(q), 3'(c)}; out_sel2 = {2'(q), 3'(c + 1)}; #1;
                        for (int p = 0; p < 4; p++) begin
                            out_color[((4*q + p)*4 + c)*W +: W]     = out_row[p*W +: W];
                            out_color[((4*q + p)*4 + c + 1)*W +: W] = out_row2[p*W +: W];
                        end
                    end
                    out_sel = {2'(q), 3'd4}; #1;
                    for (int p = 0; p < 4; p++) out_depth[(4*q + p)*W +: W] = out_row[p*W +: W];
                end
                for (int r = 0; r < (outonly ? 0 : 32); r++)
                    if (!close(gpr_of(dut.ol_w, r), expg[r], tol)) begin
                        err++; if (err < 8) $display("  warp %0d r%0d got %h\n      exp %h", dut.ol_w, r, gpr_of(dut.ol_w, r), expg[r]);
                    end
                if (!close(out_color, outc[0], tol)) begin err++; $display("  warp %0d out_color got %h\n            exp %h", dut.ol_w, out_color, outc[0]); end
                if (out_kill !== kill[0]) begin err++; $display("  warp %0d kill got %h exp %h", dut.ol_w, out_kill, kill[0]); end
                if (out_depthw !== outzw[0][0]) begin err++; $display("  warp %0d depth written %b exp %b", dut.ol_w, out_depthw, outzw[0][0]); end
                else if (out_depthw && !close({{(RW-LW){1'b0}}, out_depth}, {{(RW-LW){1'b0}}, outz[0]}, tol)) begin
                    err++; $display("  warp %0d depth got %h\n           exp %h", dut.ol_w, out_depth, outz[0]); end
                if (out_err !== experr[0][0]) begin err++; $display("  warp %0d err %b exp %b", dut.ol_w, out_err, experr[0][0]); end
                out_ack <= 1; @(posedge clk); out_ack <= 0; nout++;
            end
        end
        if (nout < nw) begin $display("FAIL: FS2 core watchdog at %0d cyc (%0d of %0d warps)", cyc, nout, nw); $display("TB_FAIL tb_fs2 %s", prog); $finish; end
        @(posedge clk); @(posedge clk);
        if (!idle) begin err++; $display("  core not idle after the retire"); end
        $display("FS2 %s: %0d instr, %0d warp(s) finished in %0d cyc, errors = %0d (tolerance %0d ULP)", prog, n, nw, cyc, err, tol);
        if (err == 0) $display("TB_PASS tb_fs2 %s", prog); else $display("TB_FAIL tb_fs2 %s", prog);
        $finish;
    end
    initial begin #20ms; $display("TB_FAIL tb_fs2: TIMEOUT"); $finish; end
endmodule

// Texture-unit model on tmu24's request / result interface. A quad request is accepted when
// ready (random, 3 of 4 cycles); its 16 texels are tools/isa64.py tex_model, per pixel: R = s, G = t's sign and
// exponent with q's low 16 mantissa bits (0 when t is zero), B = 2 r (exponent + 1 for a normal r), A = float(4 samp +
// mode + 1) — every coordinate component, the sampler and the mode reach a texel. Results return in request order
// after a random latency of 3..LAT cycles (+TEXLAT_<n>), at least 4 cycles apart (the core's contract, as tmu24).
module tmu_model2 #(parameter int W = 24, parameter int SEED = 7)(
    input  wire clk, rst, req_valid, output reg req_ready, input wire [3:0] req_sampler, input wire [1:0] req_mode,
    input  wire [8*W-1:0] req_uv, req_zw, output reg res_valid, output reg [16*W-1:0] res_texels);
    typedef struct { logic [16*W-1:0] d; longint due; } ent_t;
    ent_t q[$]; longint now = 0, last_due = 0; int unsigned rs = SEED; int lat = 40, nreq = 0;
    initial void'($value$plusargs("TEXLAT_%d", lat));
    function automatic int unsigned rnd(); rs = rs * 1103515245 + 12345; return rs >> 8; endfunction
    function automatic [4*W-1:0] texel4(input [W-1:0] cs, ct, cr, cq, input [3:0] s, input [1:0] m);
        int v, msb; logic [W-1:0] a;
        v = 4 * int'(s) + int'(m) + 1; msb = 0;
        for (int i = 0; i < 7; i++) if ((v >> i) & 1) msb = i;
        a = {1'b0, 7'(63 + msb), 16'(v << (16 - msb))};
        texel4[0 +: W]   = cs;
        texel4[W +: W]   = (ct[W-2 -: 7] == 7'd0) ? {W{1'b0}} : {ct[W-1 -: 8], cq[15:0]};
        texel4[2*W +: W] = (cr[W-2 -: 7] == 7'd0 || cr[W-2 -: 7] == 7'h7F) ? cr : cr + (W'(1) << 16);
        texel4[3*W +: W] = a;
    endfunction
    always @(posedge clk) begin
        now++;
        req_ready <= (rnd() % 4 != 0);
        res_valid <= 1'b0;
        if (rst) begin q.delete(); req_ready <= 1'b0; last_due = 0; end
        else begin
            if (req_valid && req_ready) begin
                ent_t e; longint d;
                for (int p = 0; p < 4; p++)
                    e.d[p*4*W +: 4*W] = texel4(req_uv[p*2*W +: W], req_uv[p*2*W+W +: W], req_zw[p*2*W +: W], req_zw[p*2*W+W +: W],
                                               req_sampler, req_mode);
                d = now + 3 + longint'(rnd() % (lat > 3 ? lat - 2 : 1));
                if (d < last_due + 4) d = last_due + 4;
                e.due = d; last_due = d; q.push_back(e); nreq++;
            end
            if (q.size() > 0 && q[0].due <= now) begin res_valid <= 1'b1; res_texels <= q[0].d; q.pop_front(); end
        end
    end
endmodule
`default_nettype wire
