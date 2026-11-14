`timescale 1ns/1ps
// tb_cvt_ow12 — INDEPENDENT golden for float_cvt_f2i at the OW=12 screen-coord width (the vtx_setup lane,
// where a width of OW+3 = 15 < SIG = 17 would give a negative pad and a sim FATAL). Proves the max(OW,MAN_W)+3
// width both (a) elaborates/runs without the negative-replication FATAL and (b) is FUNCTIONALLY correct:
// truncate-toward-zero + saturate to signed 12-bit [-2048, 2047], Inf/NaN -> 0.  FP24 (EXP_W=7,MAN_W=16).
module tb_cvt_ow12;
    localparam int EXP_W=7, MAN_W=16, W=1+EXP_W+MAN_W, OW=12, BIAS=63;
    reg clk=0; always #5 clk=~clk;
    reg rst=1, iv=0; reg [W-1:0] fval=0; wire ov; wire signed [OW-1:0] ival;
    float_cvt_f2i #(.EXP_W(EXP_W),.MAN_W(MAN_W),.OW(OW)) dut(.clk,.rst,.in_valid(iv),.fval,.out_valid(ov),.ival);

    // build an FP24 word from a real magnitude that is exactly representable at these test points
    function [W-1:0] fp(input sgn, input signed [8:0] e_unb, input [MAN_W-1:0] man);
        fp = { sgn, e_unb[EXP_W-1:0] + BIAS[EXP_W-1:0], man };
    endfunction

    // test vectors: {fval, expected signed ival}
    localparam int N=11;
    reg [W-1:0] tv [0:N-1]; reg signed [OW-1:0] exp [0:N-1];
    integer i, errs=0; reg signed [OW-1:0] got [0:N-1]; integer gi=0;
    initial begin
        // 0                       -> 0
        tv[0]=24'h000000;                       exp[0]=12'sd0;
        // 1.0 (e=0,man=0)         -> 1
        tv[1]=fp(0, 9'sd0,  16'h0000);          exp[1]=12'sd1;
        // 100 = 1.5625*2^6, man=0.5625*2^16=0x9000 -> 100
        tv[2]=fp(0, 9'sd6,  16'h9000);          exp[2]=12'sd100;
        // 2047 = IMAX, exactly 1.1111111111b*2^10, man=0xFFC0 -> 2047 (boundary, NOT saturated)
        tv[3]=fp(0, 9'sd10, 16'hFFC0);          exp[3]=12'sd2047;
        // 4096 = 2^12 (e=12,man=0) -> saturate to IMAX 2047
        tv[4]=fp(0, 9'sd12, 16'h0000);          exp[4]=12'sd2047;
        // -2048 = IMIN, = -1.0*2^11 (e=11,man=0) -> -2048
        tv[5]=fp(1, 9'sd11, 16'h0000);          exp[5]=-12'sd2048;
        // -4096 -> saturate to IMIN -2048
        tv[6]=fp(1, 9'sd12, 16'h0000);          exp[6]=-12'sd2048;
        // 0.5 (e=-1) -> truncate toward zero -> 0
        tv[7]=fp(0, -9'sd1, 16'h0000);          exp[7]=12'sd0;
        // 100.9 -> floor magnitude -> 100 : 100.9/64=1.5765625, man=0.5765625*65536=0x93A0
        tv[8]=fp(0, 9'sd6,  16'h93A0);          exp[8]=12'sd100;
        // -100.9 -> truncate toward zero -> -100
        tv[9]=fp(1, 9'sd6,  16'h93A0);          exp[9]=-12'sd100;
        // +Inf (e=127,man=0) -> 0
        tv[10]=fp(0, 9'sd64, 16'h0000);         exp[10]=12'sd0;

        // capture outputs (latency 2)
        forever begin @(posedge clk); if (ov) begin got[gi]=ival; gi=gi+1; end end
    end

    initial begin
        repeat(4) @(negedge clk); rst=0;
        for (i=0;i<N;i=i+1) begin @(negedge clk); iv=1; fval=tv[i]; end
        @(negedge clk); iv=0;
        repeat(6) @(negedge clk);
        for (i=0;i<N;i=i+1) begin
            if (got[i] !== exp[i]) begin errs=errs+1; $display("  FAIL vec%0d: got %0d exp %0d (fval=%h)", i, got[i], exp[i], tv[i]); end
            else $display("  ok vec%0d: %0d", i, got[i]);
        end
        $display("%s", errs==0 ? "tb_cvt_ow12: PASS (OW=12 f2i: no FATAL, truncate+saturate correct)" : "tb_cvt_ow12: FAIL");
        $finish;
    end
    initial begin #100000; $display("tb_cvt_ow12: TIMEOUT"); $finish; end
endmodule
