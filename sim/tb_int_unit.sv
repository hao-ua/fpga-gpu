// tb_int_unit — int_unit (IMUL low 24 bits; divider IDIV / IMOD / UDIV / UMOD: signed toward zero, x/0 = -1,
// -2^23/-1 = -2^23, remainder with the dividend's sign, x%0 = x, unsigned x/0 = all ones; the 16 IOP functions) with
// 16 lanes of 24-bit integers against the simulator's own integer arithmetic (independent of the Python model):
// edge operands (0, +-1, +-2^23-ish, the most negative value, powers of two) mixed with random ones; IMUL streamed
// back to back (II 1) and with gaps, IDIV back to back as fast as div_ready allows; results in order, none lost.
`default_nettype none
// -d W16: the FP16 lane width (W = 16, ONE = 0x3C00) of the single-board fragment core.
module tb_int_unit;
`ifdef W16
    localparam int NL = 16, W = 16, N = 400;
    localparam [W-1:0] ONE = 16'h3c00;
`else
    localparam int NL = 16, W = 24, N = 400;
    localparam [W-1:0] ONE = 24'h3f0000;
`endif
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    reg  [NL*W-1:0] a = 0, b = 0; reg mv = 0, dv = 0, iv = 0; reg [1:0] dm = 0; reg [3:0] fn = 0;
    wire vm, vd, dr, vi; wire [NL*W-1:0] ym, yd, yi;
    int_unit #(.NL(NL), .W(W), .ONE(ONE)) dut(.clk, .rst, .a, .b, .mul_valid(mv), .v_mul(vm), .y_mul(ym),
                                   .div_valid(dv), .div_mode(dm), .div_ready(dr), .v_div(vd), .y_div(yd),
                                   .iop_valid(iv), .iop_fn(fn), .v_iop(vi), .y_iop(yi));
    reg [NL*W-1:0] A[N], B[N], gm[$], gd[$], gi[$]; reg [1:0] DM[N]; reg [3:0] FN[N];
    always @(posedge clk) begin if (vm) gm.push_back(ym); if (vd) gd.push_back(yd); if (vi) gi.push_back(yi); end

    function automatic [W-1:0] pick();
        case ($urandom_range(0, 9))
            0: return '0;
            1: return W'(1);
            2: return {W{1'b1}};                          // -1
            3: return {1'b1, {(W-1){1'b0}}};              // -2^23
            4: return {1'b0, {(W-1){1'b1}}};              // 2^23 - 1
            5: return W'(1) << $urandom_range(0, W-1);
            6: return -(W'(1) << $urandom_range(0, W-2));
            7: return W'($urandom_range(0, 300)) - W'(150);
            default: return W'($urandom);
        endcase
    endfunction
    function automatic [W-1:0] emul(input [W-1:0] x, input [W-1:0] y);
        longint p; p = longint'($signed(x)) * longint'($signed(y)); return p[W-1:0];
    endfunction
    function automatic [W-1:0] ediv(input [W-1:0] x, input [W-1:0] y, input [1:0] m);
        int sx, sy, q; int unsigned ux, uy;
        sx = $signed(x); sy = $signed(y); ux = x; uy = y;
        case (m)
            2'd0: begin if (sy == 0) return {W{1'b1}}; q = sx / sy; return q[W-1:0]; end   // toward zero; wraps
            2'd1: begin if (sy == 0) return x; q = sx % sy; return q[W-1:0]; end           // SV %: dividend's sign
            2'd2: begin if (uy == 0) return {W{1'b1}}; return W'(ux / uy); end
            default: begin if (uy == 0) return x; return W'(ux % uy); end
        endcase
    endfunction
    function automatic [W-1:0] eiop(input [W-1:0] x, input [W-1:0] y, input [3:0] f);
        reg signed [W-1:0] sx, sy; int n; sx = x; sy = y; n = y[4:0];
        case (f)
            0: return x & y;  1: return x | y;  2: return x ^ y;  3: return ~x;
            4: return (n < W) ? x << n : '0;
            5: return (n < W) ? W'(sx >>> n) : {W{sx[W-1]}};
            6: return (n < W) ? x >> n : '0;
            7: return (sx < sy) ? x : y;   8: return (sx > sy) ? x : y;
            9: return (x < y) ? x : y;     10: return (x > y) ? x : y;
            11: return sx[W-1] ? -x : x;   12: return -x;
            13: return sx[W-1] ? {W{1'b1}} : (x != 0) ? W'(1) : W'(0);
            14: return (x < y) ? ONE : '0;
            default: return (x >= y) ? ONE : '0;
        endcase
    endfunction

    int err, i, l;
    initial begin
        for (i = 0; i < N; i++) begin
            for (l = 0; l < NL; l++) begin A[i][l*W +: W] = pick(); B[i][l*W +: W] = pick(); end
            DM[i] = 2'(i); FN[i] = 4'($urandom_range(0, 15));
            if (i % 5 == 0) for (l = 0; l < NL; l++) B[i][l*W +: W] = W'($urandom_range(0, 31));   // shift counts
        end
        A[0][0 +: W] = {1'b1, {(W-1){1'b0}}}; B[0][0 +: W] = {W{1'b1}};    // the overflow case in lane 0 of op 0
        repeat (4) @(posedge clk); rst <= 0; @(posedge clk);
        // IMUL: first half back to back, second half with random gaps
        for (i = 0; i < N; ) begin
            @(posedge clk);
            if (i >= N/2 && $urandom_range(0, 2) == 0) begin mv <= 0; continue; end
            mv <= 1; a <= A[i]; b <= B[i]; i++;
        end
        @(posedge clk); mv <= 0;
        // IDIV: a one-cycle request as soon as div_ready is high (back to back with the previous result)
        for (i = 0; i < N; i++) begin
            @(posedge clk); while (!dr) @(posedge clk);
            dv <= 1; a <= A[i]; b <= B[i]; dm <= DM[i];
            @(posedge clk); dv <= 0;
        end
        // IOP: back to back
        for (i = 0; i < N; i++) begin @(posedge clk); iv <= 1; a <= A[i]; b <= B[i]; fn <= FN[i]; end
        @(posedge clk); iv <= 0;
        repeat (60) @(posedge clk);
        err = 0;
        if (gm.size() != N) begin err++; $display("  IMUL: %0d results for %0d", gm.size(), N); end
        if (gd.size() != N) begin err++; $display("  IDIV: %0d results for %0d", gd.size(), N); end
        if (gi.size() != N) begin err++; $display("  IOP: %0d results for %0d", gi.size(), N); end
        for (i = 0; i < N && i < gi.size(); i++) for (l = 0; l < NL; l++)
            if (gi[i][l*W +: W] !== eiop(A[i][l*W +: W], B[i][l*W +: W], FN[i])) begin
                err++; if (err < 10) $display("  IOP fn %0d %h, %h: got %h exp %h", FN[i], A[i][l*W +: W], B[i][l*W +: W], gi[i][l*W +: W], eiop(A[i][l*W +: W], B[i][l*W +: W], FN[i]));
            end
        for (i = 0; i < N && i < gm.size(); i++) for (l = 0; l < NL; l++)
            if (gm[i][l*W +: W] !== emul(A[i][l*W +: W], B[i][l*W +: W])) begin
                err++; if (err < 10) $display("  IMUL %h * %h: got %h exp %h", A[i][l*W +: W], B[i][l*W +: W], gm[i][l*W +: W], emul(A[i][l*W +: W], B[i][l*W +: W]));
            end
        for (i = 0; i < N && i < gd.size(); i++) for (l = 0; l < NL; l++)
            if (gd[i][l*W +: W] !== ediv(A[i][l*W +: W], B[i][l*W +: W], DM[i])) begin
                err++; if (err < 10) $display("  DIV mode %0d %h, %h: got %h exp %h", DM[i], A[i][l*W +: W], B[i][l*W +: W], gd[i][l*W +: W], ediv(A[i][l*W +: W], B[i][l*W +: W], DM[i]));
            end
        $display("int_unit: %0d IMUL + %0d divider (4 modes) + %0d IOP vectors x %0d lanes x %0d bits, errors = %0d", gm.size(), gd.size(), gi.size(), NL, W, err);
        if (err == 0) $display("TB_PASS tb_int_unit"); else $display("TB_FAIL tb_int_unit");
        $finish;
    end
    initial begin #3ms; $display("TB_FAIL tb_int_unit: TIMEOUT"); $finish; end
endmodule
`default_nettype wire
