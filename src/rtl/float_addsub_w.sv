// float_addsub_w — float add / subtract for a wide significand (MAN_W above FP24's 16): flush to zero, round to nearest
// even, the arithmetic of float_addsub bit for bit, with a stage each for the alignment, the addition, the leading-zero
// count, the normalising shift and the rounding. op 0: a + b, op 1: a - b. Fixed latency 6, a new pair every cycle.
// Model: tools/float_ref.py fadd with Fmt(EXP_W, MAN_W).
`default_nettype none
module float_addsub_w #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 31
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 in_valid,
    input  wire                 op,          // 0 = add, 1 = subtract
    input  wire [EXP_W+MAN_W:0] a,
    input  wire [EXP_W+MAN_W:0] b,
    output reg                  out_valid,
    output reg  [EXP_W+MAN_W:0] result
);
    localparam int W     = 1 + EXP_W + MAN_W;
    localparam int SIG   = MAN_W + 1;
    localparam int INF_E = (1 << EXP_W) - 1;
    localparam int WM    = SIG + 4;                 // carry + significand + guard, round, sticky
    localparam int LZW   = $clog2(WM) + 1;
    localparam [W-1:0] QNAN = {1'b0, {EXP_W{1'b1}}, 1'b1, {(MAN_W-1){1'b0}}};

    function automatic [MAN_W:0] sig_of(input [W-1:0] x);
        sig_of = (|x[W-2:MAN_W]) ? {1'b1, x[MAN_W-1:0]} : '0;
    endfunction
    function automatic [LZW-1:0] clz(input [WM-1:0] v);          // the zeros above the leading one, below the carry bit
        integer i; logic done;
        clz = SIG+3; done = 0;
        for (i = SIG+2; i >= 0; i = i-1)
            if (!done && v[i]) begin clz = (SIG+2) - i; done = 1; end
    endfunction

    wire             s_a = a[W-1],  s_b = b[W-1] ^ op;
    wire [EXP_W-1:0] e_a = a[W-2:MAN_W], e_b = b[W-2:MAN_W];
    wire             a_zero = (e_a == 0), b_zero = (e_b == 0);
    wire             a_inf = (e_a == INF_E) && (a[MAN_W-1:0] == 0), b_inf = (e_b == INF_E) && (b[MAN_W-1:0] == 0);
    wire             a_nan = (e_a == INF_E) && (a[MAN_W-1:0] != 0), b_nan = (e_b == INF_E) && (b[MAN_W-1:0] != 0);
    wire [W-2:0]     mag_a = {e_a, a[MAN_W-1:0]}, mag_b = {e_b, b[MAN_W-1:0]};
    wire             a_ge = (mag_a >= mag_b);

    // ---- stage 0: the larger operand first, the classes, the alignment's shift (the exponent difference, at most the
    //      whole significand and its three extra bits: beyond that the smaller operand is the sticky bit alone)
    wire [EXP_W:0] ediff = a_ge ? ({1'b0, e_a} - {1'b0, e_b}) : ({1'b0, e_b} - {1'b0, e_a});
    wire           far_i = ediff > (SIG+3);
    reg              v0, sbig0, same0, nan0, inf0, infsign0, bothzero0, far;
    reg [EXP_W-1:0]  ebig0;
    reg [MAN_W:0]    sbsig0, sssig0;
    reg [$clog2(WM+1)-1:0] sh;
    always @(posedge clk) begin
        v0        <= in_valid & ~rst;
        nan0      <= a_nan | b_nan | (a_inf & b_inf & (s_a != s_b));
        inf0      <= a_inf | b_inf;
        infsign0  <= a_inf ? s_a : s_b;
        bothzero0 <= a_zero & b_zero;
        same0     <= (s_a == s_b);
        far       <= far_i;
        sh        <= far_i ? (SIG+3) : ediff[$clog2(WM+1)-1:0];
        if (a_ge) begin sbig0 <= s_a; ebig0 <= e_a; sbsig0 <= sig_of(a); sssig0 <= sig_of(b); end
        else      begin sbig0 <= s_b; ebig0 <= e_b; sbsig0 <= sig_of(b); sssig0 <= sig_of(a); end
    end

    // ---- stage 1: the smaller significand shifted right, the lost bits into the sticky
    wire [WM-1:0]  small_m = {1'b0, sssig0, 3'b000};
    wire           stick_lost = far ? (|sssig0) : (|(small_m & ((({{(WM-1){1'b0}}, 1'b1}) << sh) - 1)));
    reg              v1, sbig1, same1, nan1, inf1, infsign1, bothzero1;
    reg [EXP_W-1:0]  ebig1;
    reg [WM-1:0]     bigm1, smallin1;
    always @(posedge clk) begin
        v1 <= v0 & ~rst; sbig1 <= sbig0; same1 <= same0; ebig1 <= ebig0;
        nan1 <= nan0; inf1 <= inf0; infsign1 <= infsign0; bothzero1 <= bothzero0;
        bigm1    <= {1'b0, sbsig0, 3'b000};
        smallin1 <= (small_m >> sh) | {{(WM-1){1'b0}}, stick_lost};
    end

    // ---- stage 2: the sum or the difference
    reg              v2, sres2, nan2, inf2, infsign2, bothzero2, iszero2;
    reg [EXP_W-1:0]  ebig2;
    reg [WM-1:0]     mant2;
    always @(posedge clk) begin
        v2 <= v1 & ~rst; sres2 <= sbig1; ebig2 <= ebig1; nan2 <= nan1; inf2 <= inf1; infsign2 <= infsign1; bothzero2 <= bothzero1;
        mant2   <= same1 ? (bigm1 + smallin1) : (bigm1 - smallin1);
        iszero2 <= (~same1) && (bigm1 == smallin1);
    end

    // ---- stage 3: the leading-zero count
    reg              v3, sres3, nan3, inf3, infsign3, bothzero3, iszero3;
    reg [EXP_W-1:0]  ebig3;
    reg [WM-1:0]     mant3;
    reg [LZW-1:0]    lz3;
    always @(posedge clk) begin
        v3 <= v2 & ~rst; sres3 <= sres2; ebig3 <= ebig2; nan3 <= nan2; inf3 <= inf2; infsign3 <= infsign2;
        bothzero3 <= bothzero2; iszero3 <= iszero2;
        mant3 <= mant2; lz3 <= clz(mant2);
    end

    // ---- stage 4: the normalising shift, the exponent
    wire carry = mant3[WM-1];
    wire signed [EXP_W+2:0] lz_s = $signed({1'b0, lz3}), ebig_s = $signed({2'b0, ebig3});
    reg              v4, sres4, nan4, inf4, infsign4, bothzero4, iszero4, carry4, clo4;
    reg [WM-1:0]     norm4;
    reg signed [EXP_W+2:0] exp_n4;
    always @(posedge clk) begin
        v4 <= v3 & ~rst; sres4 <= sres3; nan4 <= nan3; inf4 <= inf3; infsign4 <= infsign3;
        bothzero4 <= bothzero3; iszero4 <= iszero3;
        norm4  <= carry ? (mant3 >> 1) : (mant3 << lz3);
        exp_n4 <= carry ? (ebig_s + 1) : (ebig_s - lz_s);
        carry4 <= carry; clo4 <= mant3[0];
    end

    // ---- stage 5: round and pack
    wire [MAN_W-1:0] man_n = norm4[SIG+1:3];
    wire g = norm4[2], r = norm4[1], s = norm4[0] | (carry4 & clo4);
    wire roundup = g & (r | s | man_n[0]);
    wire [MAN_W:0] man_r = {1'b0, man_n} + (MAN_W+1)'(roundup);
    wire carry2 = man_r[MAN_W];
    wire [MAN_W-1:0] man_f = carry2 ? {1'b0, man_r[MAN_W-1:1]} : man_r[MAN_W-1:0];
    wire signed [EXP_W+2:0] exp_f = exp_n4 + (carry2 ? 1 : 0);
    always @(posedge clk) begin
        out_valid <= v4 & ~rst;
        if (nan4)                       result <= QNAN;
        else if (inf4)                  result <= {infsign4, {EXP_W{1'b1}}, {MAN_W{1'b0}}};
        else if (iszero4 | bothzero4)   result <= {(W){1'b0}};
        else if (exp_f >= INF_E)        result <= {sres4, {EXP_W{1'b1}}, {MAN_W{1'b0}}};
        else if (exp_f <= 0)            result <= {sres4, {(W-1){1'b0}}};
        else                            result <= {sres4, exp_f[EXP_W-1:0], man_f};
    end
endmodule
`default_nettype wire
