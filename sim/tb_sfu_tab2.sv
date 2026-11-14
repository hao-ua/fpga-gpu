`timescale 1ns/1ps
// tb_sfu_tab2 — float_sfu with its tables outside (TAB_EXT, two units on one sfu_tab2) against float_sfu with its own
// tables, FP24, at both latencies the design uses (SPLIT1B = SPLIT6 = 1: the shader cores; 0: the setup units). The two
// units of the pair take different operands every cycle (II = 1, back to back); each must give, bit for bit, what a unit
// with its own tables gives for the same stream. Stimulus: every one of the 2 048 table indices of RCP, RSQRT, LOG2 and
// EXP2 over a spread of exponents (both signs; the LOG2 / EXP2 index 2 047, whose second end is the table's last entry,
// among them), then random operands with zeros, infinities and NaNs mixed in.
`default_nettype none
module tb_sfu_tab2;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    localparam int EXP_W = 7, MAN_W = 16, W = 24, K = 12, KLOG = 11;
    localparam string RCP = "tools/gen/fp24_rcp_seed.mem", RSQ = "tools/gen/fp24_rsqrt_seed.mem",
                      LG = "tools/gen/fp24_log2_lut.mem", EX = "tools/gen/fp24_exp2_lut.mem";
    reg         v;
    reg  [2:0]  op;
    reg  [W-1:0] x0, x1;
    int         errs = 0, n_out = 0;

    genvar gs;
    generate for (gs = 0; gs < 2; gs++) begin : SP
        localparam bit S = gs[0];
        wire rv0, rv1, dv0, dv1; wire [W-1:0] ry0, ry1, dy0, dy1;
        float_sfu #(.EXP_W(EXP_W), .MAN_W(MAN_W), .K(K), .KLOG(KLOG), .SPLIT1B(S), .SPLIT6(S),
                    .RCP_MEM(RCP), .RSQRT_MEM(RSQ), .LOG2_MEM(LG), .EXP2_MEM(EX)) r0 (.clk, .rst, .in_valid(v), .op, .x(x0),
            .out_valid(rv0), .result(ry0), .t_rcp_a(), .t_rsq_a(), .t_log_i(), .t_exp_i(), .t_rcp_q('0), .t_rsq_q('0),
            .t_la('0), .t_lb('0), .t_ea('0), .t_eb('0));
        float_sfu #(.EXP_W(EXP_W), .MAN_W(MAN_W), .K(K), .KLOG(KLOG), .SPLIT1B(S), .SPLIT6(S),
                    .RCP_MEM(RCP), .RSQRT_MEM(RSQ), .LOG2_MEM(LG), .EXP2_MEM(EX)) r1 (.clk, .rst, .in_valid(v), .op, .x(x1),
            .out_valid(rv1), .result(ry1), .t_rcp_a(), .t_rsq_a(), .t_log_i(), .t_exp_i(), .t_rcp_q('0), .t_rsq_q('0),
            .t_la('0), .t_lb('0), .t_ea('0), .t_eb('0));
        wire [K-1:0] ra0, ra1, sa0, sa1; wire [KLOG-1:0] li0, li1, ei0, ei1;
        wire [MAN_W+1:0] rq0, rq1, sq0, sq1; wire [23:0] la0, lb0, la1, lb1, ea0, eb0, ea1, eb1;
        float_sfu #(.EXP_W(EXP_W), .MAN_W(MAN_W), .K(K), .KLOG(KLOG), .SPLIT1B(S), .SPLIT6(S), .TAB_EXT(1'b1)) d0 (
            .clk, .rst, .in_valid(v), .op, .x(x0), .out_valid(dv0), .result(dy0),
            .t_rcp_a(ra0), .t_rsq_a(sa0), .t_log_i(li0), .t_exp_i(ei0), .t_rcp_q(rq0), .t_rsq_q(sq0),
            .t_la(la0), .t_lb(lb0), .t_ea(ea0), .t_eb(eb0));
        float_sfu #(.EXP_W(EXP_W), .MAN_W(MAN_W), .K(K), .KLOG(KLOG), .SPLIT1B(S), .SPLIT6(S), .TAB_EXT(1'b1)) d1 (
            .clk, .rst, .in_valid(v), .op, .x(x1), .out_valid(dv1), .result(dy1),
            .t_rcp_a(ra1), .t_rsq_a(sa1), .t_log_i(li1), .t_exp_i(ei1), .t_rcp_q(rq1), .t_rsq_q(sq1),
            .t_la(la1), .t_lb(lb1), .t_ea(ea1), .t_eb(eb1));
        sfu_tab2 #(.EXP_W(EXP_W), .MAN_W(MAN_W), .K(K), .KLOG(KLOG), .RCP_MEM(RCP), .RSQRT_MEM(RSQ), .LOG2_MEM(LG),
                   .EXP2_MEM(EX)) u_tab (.clk, .rcp_a0(ra0), .rcp_a1(ra1), .rsq_a0(sa0), .rsq_a1(sa1),
            .log_i0(li0), .log_i1(li1), .exp_i0(ei0), .exp_i1(ei1), .rcp_q0(rq0), .rcp_q1(rq1), .rsq_q0(sq0), .rsq_q1(sq1),
            .la0, .lb0, .la1, .lb1, .ea0, .eb0, .ea1, .eb1);
        always @(posedge clk) if (!rst) begin
            if (dv0 !== rv0 || dv1 !== rv1) begin
                errs++; if (errs < 10) $display("FAIL split %0d: valid %b%b, expected %b%b", S, dv0, dv1, rv0, rv1);
            end else if (rv0) begin
                n_out++;
                if (dy0 !== ry0) begin errs++; if (errs < 10) $display("FAIL split %0d unit 0: %h, expected %h", S, dy0, ry0); end
                if (dy1 !== ry1) begin errs++; if (errs < 10) $display("FAIL split %0d unit 1: %h, expected %h", S, dy1, ry1); end
            end
        end
    end endgenerate

    function automatic [W-1:0] fp(input bit s, input [EXP_W-1:0] e, input [MAN_W-1:0] m); fp = {s, e, m}; endfunction
    int ex [0:9] = '{63, 64, 62, 65, 61, 66, 70, 55, 72, 50};          // exponents around 1.0 (bias 63) and further out
    initial begin
        v = 0; op = 0; x0 = 0; x1 = 0;
        repeat (4) @(posedge clk); rst = 0; repeat (2) @(posedge clk);
        // every table index of each function: the top KLOG / K mantissa bits swept, the low bits from a counter; unit 1 runs
        // the sweep backwards (other indices, other ports' timing)
        for (int o = 0; o < 4; o++) begin
            for (int ei = 0; ei < 10; ei++) begin
                for (int t = 0; t < 4096; t++) begin
                    @(negedge clk);
                    v = 1; op = 3'(o);
                    x0 = fp(o == 3 ? t[0] : 1'b0, 7'(ex[ei]), {t[11:0], 4'(t * 5)});
                    x1 = fp(o == 3 ? ~t[0] : 1'b0, 7'(ex[(ei + 3) % 10]), {12'(4095 - t), 4'(t * 3)});
                end
            end
        end
        // random operands, the four ops mixed, specials among them
        for (int i = 0; i < 200_000; i++) begin
            @(negedge clk);
            v = ($urandom % 8) != 0;
            op = 3'($urandom % 4);
            x0 = W'($urandom); x1 = W'($urandom);
            case ($urandom % 16)
                0: x0 = '0;                                   // zero
                1: x0 = fp($urandom % 2, '1, '0);             // infinity
                2: x0 = fp(0, '1, 16'h8000);                  // NaN
                3: x1 = fp(0, 7'(63), '1);                    // just below 2.0: LOG2's index 2 047
                default: ;
            endcase
        end
        @(negedge clk); v = 0;
        repeat (40) @(posedge clk);
        $display("tb_sfu_tab2: %0d result pairs compared (both latencies), %0d errors", n_out, errs);
        if (errs == 0 && n_out > 100_000) $display("TB_PASS tb_sfu_tab2"); else $display("TB_FAIL tb_sfu_tab2");
        $finish;
    end
endmodule
`default_nettype wire
