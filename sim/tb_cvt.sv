// tb_cvt — verifies float_cvt_f2i (FTOI) truncates toward zero AND SATURATES finite out-of-range values
// (rather than wrapping them). FP24 (EXP_W=7,MAN_W=16), OW=24 -> signed range [-2^23, 2^23-1].
`timescale 1ns/1ps
`default_nettype none
module tb_cvt;
    localparam W=24, OW=24, N=11;
    reg clk=0; always #5 clk=~clk;
    reg rst=1, iv=0; reg [W-1:0] fval;
    wire ov; wire signed [OW-1:0] ival;

    float_cvt_f2i #(.EXP_W(7),.MAN_W(16),.OW(OW)) dut(.clk,.rst,.in_valid(iv),.fval,.out_valid(ov),.ival);

    reg [W-1:0] inp [0:N-1];
    reg signed [OW-1:0] exp [0:N-1];
    integer i, errs=0, wi=0;
    // valid-driven collector: check each result as out_valid fires (robust to pipeline latency)
    always @(posedge clk) if (ov && !rst && wi<N) begin
        if (ival !== exp[wi]) begin errs=errs+1; $display("  FTOI[%0d] in=%h got %0d exp %0d", wi, inp[wi], ival, exp[wi]); end
        wi=wi+1;
    end
    initial begin
        inp[0]=24'h40D99A; exp[0]=3;          // 3.7
        inp[1]=24'hC0D99A; exp[1]=-3;         // -3.7
        inp[2]=24'h3E0000; exp[2]=0;          // 0.5
        inp[3]=24'hBE0000; exp[3]=0;          // -0.5
        inp[4]=24'h459000; exp[4]=100;        // 100
        inp[5]=24'hC59000; exp[5]=-100;       // -100
        inp[6]=24'h560000; exp[6]=8388607;    // 2^23 -> saturate
        inp[7]=24'hD60000; exp[7]=-8388608;   // -2^23 (in range)
        inp[8]=24'h57312D; exp[8]=8388607;    // 2e7 -> saturate
        inp[9]=24'hD7312D; exp[9]=-8388608;   // -2e7 -> saturate
        inp[10]=24'h000000; exp[10]=0;        // 0
        repeat(3) @(negedge clk); rst=0;
        for (i=0;i<N;i=i+1) begin @(negedge clk); iv<=1; fval<=inp[i]; end
        @(negedge clk); iv<=0;
        wait(wi==N); #1;
        $display("tb_cvt (FTOI truncate+saturate): errors = %0d", errs);
        $display("%s", errs==0 ? "PASS" : "FAIL");
        $finish;
    end
    initial begin #10000; $display("tb_cvt TIMEOUT (wi=%0d)", wi); $finish; end
endmodule
`default_nettype wire
