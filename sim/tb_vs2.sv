// tb_vs2 — the vertex core on the scalar fragment core's pattern (shader_core_vs2) runs one program
// on one warp of 16 vertices and is compared with tools/isa64.py's reference (`python3 tools/isa64.py vs2 tests/<prog>.sasm`
// -> tools/gen/<prog>_vs2_*.mem). Program, uniforms and the 16 attributes of every lane go in through the runtime load
// ports; checks every scalar register the core wrote (a register still X was never written: the reference must hold 0),
// the 18 output rows (varyings 0..15, gl_Position, gl_PointSize) and err.
//   +PROG_<prog> [+TOL_<ulp>] [+ACTIVE_<hex16>] [+WARPS_<n>] [+OUTONLY]   (no "=" in the plusargs: sim_gpu.ps1
//   -Plusargs PROG_vs_cube,WARPS_4). WARPS n (1..4): the program is launched n times back to back (the same inputs on
//   every warp), so the warps interleave in the core; every warp's results must match the golden. OUTONLY: compare only
//   the outputs (for compiled shaders, whose unused lanes hold undefined values).
`default_nettype none
module tb_vs2;
    localparam int EXP_W = 7, MAN_W = 16, W = 1 + EXP_W + MAN_W, TW = 4 * W, LW = 16 * W, RW = 16 * TW;
    localparam int NPARAM = 256, NWARP = 4, NATTR = 16, NSLOT = 18;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    reg         iram_we = 0, param_we = 0, va_we = 0, in_start = 0, out_ack = 0;
    reg  [11:0] iram_waddr = 0; reg [63:0] iram_wdata = 0;
    reg  [7:0]  param_idx = 0; reg [TW-1:0] param_data = 0;
    reg  [3:0]  va_lane = 0, va_idx = 0; reg [TW-1:0] va_data = 0;
    reg  [15:0] in_active = 16'hFFFF;
    reg  [6:0]  out_sel = 0;
    wire        in_ready, out_valid, out_err, idle, err_stack; wire [1:0] in_warp;
    wire [LW-1:0] out_row;
    shader_core_vs2 #(.EXP_W(EXP_W), .MAN_W(MAN_W), .NWARP(NWARP), .NPARAM(NPARAM), .NATTR(NATTR)) dut (
        .clk, .rst, .iram_we, .iram_waddr, .iram_wdata, .param_we, .param_bank(1'b0), .param_idx, .param_data,
        .va_we, .va_lane, .va_idx, .va_data,
        .in_ready, .in_warp, .in_start, .in_bank(1'b0), .in_active, .busy_warps(),
        .out_valid, .out_ack, .out_sel, .out_row, .out_err, .err_stack, .idle);

    reg [63:0]   iram  [0:4095];
    reg [TW-1:0] prm   [0:NPARAM-1];
    reg [RW-1:0] attr  [0:NATTR-1];              // attribute i: 16 lanes, lane 15 first (w z y x per lane)
    reg [RW-1:0] expg  [0:31];
    reg [RW-1:0] outpos [0:0], outpsz [0:0];
    reg [RW-1:0] outv  [0:15];
    reg [0:0]    experr [0:0];
    reg [RW-1:0] got   [0:NSLOT-1];              // the output rows gathered per slot (lane l, comp c at [(l*4+c)*W +: W])
    function automatic int ulp_diff(input [W-1:0] a, input [W-1:0] b);
        int ia, ib;
        ia = a[W-1] ? -int'(a[W-2:0]) : int'(a[W-2:0]);
        ib = b[W-1] ? -int'(b[W-2:0]) : int'(b[W-2:0]);
        return (ia > ib) ? ia - ib : ib - ia;
    endfunction
    function automatic bit close(input [RW-1:0] got_v, input [RW-1:0] exp_v, input int tol);
        for (int k = 0; k < 64; k++)
            if ($isunknown(got_v[k*W +: W])) begin if (exp_v[k*W +: W] != '0) return 1'b0; end
            else if (ulp_diff(got_v[k*W +: W], exp_v[k*W +: W]) > tol) return 1'b0;
        return 1'b1;
    endfunction
    // register r of warp w as the golden packs it: lane l, component c at [(l*4+c)*W +: W]
    function automatic [RW-1:0] gpr_of(input int w, input int r);
        for (int l = 0; l < 16; l++)
            for (int c = 0; c < 4; c++)
                gpr_of[(l*4+c)*W +: W] = dut.gpr_peek(l, w*128 + r*4 + c);
    endfunction

    longint clks = 0, t0 = 0, tend = 0;          // clocks: the first launch -> the last retire (the throughput figure)
    always @(posedge clk) clks++;
    initial begin
        string prog; int tol, n, err, cyc, act, nw, nout; bit outonly;
        tol = 0; act = 16'hFFFF; nw = 1;
        if (!$value$plusargs("PROG_%s", prog)) begin $display("FAIL: no +PROG_<name>"); $finish; end
        void'($value$plusargs("TOL_%d", tol)); void'($value$plusargs("ACTIVE_%h", act)); void'($value$plusargs("WARPS_%d", nw));
        outonly = $test$plusargs("OUTONLY");
        if (nw < 1 || nw > NWARP) nw = 1;
        for (int k = 0; k < 32; k++) expg[k] = '0;
        for (int k = 0; k < 16; k++) outv[k] = '0;
        $readmemh({"tools/gen/", prog, "_vs2_iram.mem"}, iram);
        $readmemh({"tools/gen/", prog, "_vs2_params.mem"}, prm);
        $readmemh({"tools/gen/", prog, "_vs2_attr.mem"}, attr);
        $readmemh({"tools/gen/", prog, "_vs2_expgpr.mem"}, expg);
        $readmemh({"tools/gen/", prog, "_vs2_outpos.mem"}, outpos);
        $readmemh({"tools/gen/", prog, "_vs2_outpsz.mem"}, outpsz);
        $readmemh({"tools/gen/", prog, "_vs2_outv.mem"}, outv);
        $readmemh({"tools/gen/", prog, "_vs2_err.mem"}, experr);
        repeat (4) @(posedge clk); rst <= 0;
        n = 0; while (n < 4096 && !$isunknown(iram[n])) n++;
        for (int k = 0; k < n; k++) begin @(posedge clk); iram_we <= 1; iram_waddr <= 12'(k); iram_wdata <= iram[k]; end
        for (int k = 0; k < NPARAM; k++) begin @(posedge clk); iram_we <= 0; param_we <= 1; param_idx <= 8'(k); param_data <= prm[k]; end
        @(posedge clk); param_we <= 0;
        // nw warps back to back: the 16 attributes of every lane (one vec4 per write), then the launch
        cyc = 0; err = 0; nout = 0;
        for (int w = 0; w < nw; w++) begin
            while (!in_ready) @(posedge clk);
            for (int l = 0; l < 16; l++)
                for (int i = 0; i < NATTR; i++) begin
                    @(posedge clk); va_we <= 1; va_lane <= 4'(l); va_idx <= 4'(i); va_data <= attr[i][l*TW +: TW];
                end
            @(posedge clk); va_we <= 0; in_start <= 1; in_active <= 16'(act);
            if (w == 0) t0 = clks;
            @(posedge clk); in_start <= 0;
        end
        // collect nw results in launch order; each warp's registers are checked while its result is offered
        while (nout < nw && cyc < 200000) begin
            @(posedge clk); cyc++;
            if (out_valid) begin
                tend = clks;
                // the output store, row by row (the read is registered: out_sel at one edge, out_row after the next)
                for (int s = 0; s < NSLOT; s++)
                    for (int c = 0; c < 4; c++) begin
                        out_sel <= {5'(s), 2'(c)};
                        @(posedge clk); @(posedge clk); #1;
                        for (int l = 0; l < 16; l++) got[s][(l*4 + c)*W +: W] = out_row[l*W +: W];
                    end
                for (int r = 0; r < (outonly ? 0 : 32); r++)
                    if (!close(gpr_of(dut.rp, r), expg[r], tol)) begin
                        err++; if (err < 8) $display("  warp %0d r%0d got %h\n      exp %h", dut.rp, r, gpr_of(dut.rp, r), expg[r]);
                    end
                for (int s = 0; s < 16; s++)
                    if (!close(got[s], outv[s], tol)) begin err++; if (err < 12) $display("  warp %0d varying %0d got %h\n          exp %h", dut.rp, s, got[s], outv[s]); end
                if (!close(got[16], outpos[0], tol)) begin err++; $display("  warp %0d position got %h\n          exp %h", dut.rp, got[16], outpos[0]); end
                if (!close(got[17], outpsz[0], tol)) begin err++; $display("  warp %0d point size got %h\n          exp %h", dut.rp, got[17], outpsz[0]); end
                if (out_err !== experr[0][0]) begin err++; $display("  warp %0d err %b exp %b", dut.rp, out_err, experr[0][0]); end
                out_ack <= 1; @(posedge clk); out_ack <= 0; nout++;
            end
        end
        if (nout < nw) begin $display("FAIL: VS2 core watchdog at %0d cyc (%0d of %0d warps)", cyc, nout, nw); $display("TB_FAIL tb_vs2 %s", prog); $finish; end
        @(posedge clk); @(posedge clk);
        if (!idle) begin err++; $display("  core not idle after the retire"); end
        $display("VS2 %s: %0d instr, %0d warp(s), %0d clk from the first launch to the last retire (launches %0d clk apart), errors = %0d (tolerance %0d ULP)",
                 prog, n, nw, tend - t0, 16 * NATTR + 2, err, tol);
        if (err == 0) $display("TB_PASS tb_vs2 %s", prog); else $display("TB_FAIL tb_vs2 %s", prog);
        $finish;
    end
    initial begin #20ms; $display("TB_FAIL tb_vs2: TIMEOUT"); $finish; end
endmodule
`default_nettype wire
