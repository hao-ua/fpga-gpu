// float_mul_w — float multiply for a significand wider than one DSP48 takes (MAN_W 17 .. 33): flush to zero, round to
// nearest even, the arithmetic of float_mul bit for bit. The significand product is four partial products of at most
// 17 x 17 bits (one DSP48 each) summed over two further stages. Fixed latency 5, a new pair every cycle.
// Model: tools/float_ref.py fmul with Fmt(EXP_W, MAN_W).
`default_nettype none
module float_mul_w #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 31
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 in_valid,
    input  wire [EXP_W+MAN_W:0] a,
    input  wire [EXP_W+MAN_W:0] b,
    output reg                  out_valid,
    output reg  [EXP_W+MAN_W:0] result
);
    localparam int W     = 1 + EXP_W + MAN_W;
    localparam int SIG   = MAN_W + 1;                 // significand bits (1.M)
    localparam int LO    = 17;                        // the low part of a significand: one DSP48 operand
    localparam int HI    = SIG - LO;                  // the high part
    localparam int BIAS  = (1 << (EXP_W-1)) - 1;
    localparam int INF_E = (1 << EXP_W) - 1;
    localparam [W-1:0] QNAN = {1'b0, {EXP_W{1'b1}}, 1'b1, {(MAN_W-1){1'b0}}};
    if (HI < 1 || HI > LO) $error("float_mul_w: MAN_W must be 17 .. 33");

    function automatic [MAN_W:0] sig_of(input [W-1:0] x);
        sig_of = (|x[W-2:MAN_W]) ? {1'b1, x[MAN_W-1:0]} : '0;     // exponent 0 -> 0
    endfunction
    wire             s_a = a[W-1],           s_b = b[W-1];
    wire [EXP_W-1:0] e_a = a[W-2:MAN_W],     e_b = b[W-2:MAN_W];
    wire             a_zero = (e_a == 0),    b_zero = (e_b == 0);
    wire             a_inf  = (e_a == INF_E) && (a[MAN_W-1:0] == 0);
    wire             b_inf  = (e_b == INF_E) && (b[MAN_W-1:0] == 0);
    wire             a_nan  = (e_a == INF_E) && (a[MAN_W-1:0] != 0);
    wire             b_nan  = (e_b == INF_E) && (b[MAN_W-1:0] != 0);

    // ---- stage 0: the operands and their class
    reg                    v0, s0, nan0, inf0, zero0;
    reg signed [EXP_W+1:0] esum0;                     // e_a + e_b - bias
    reg [MAN_W:0]          siga0, sigb0;
    always @(posedge clk) begin
        v0    <= in_valid & ~rst;
        s0    <= s_a ^ s_b;
        esum0 <= $signed({2'b0, e_a}) + $signed({2'b0, e_b}) - BIAS;
        siga0 <= sig_of(a);
        sigb0 <= sig_of(b);
        nan0  <= a_nan | b_nan | (a_inf & b_zero) | (b_inf & a_zero);
        inf0  <= a_inf | b_inf;
        zero0 <= a_zero | b_zero;
    end

    // ---- stage 1: the four partial products
    reg                    v1, s1, nan1, inf1, zero1;
    reg signed [EXP_W+1:0] esum1;
    (* use_dsp = "yes" *) reg [2*LO-1:0]  pll;
    (* use_dsp = "yes" *) reg [LO+HI-1:0] plh, phl;
    (* use_dsp = "yes" *) reg [2*HI-1:0]  phh;
    always @(posedge clk) begin
        v1 <= v0 & ~rst; s1 <= s0; esum1 <= esum0; nan1 <= nan0; inf1 <= inf0; zero1 <= zero0;
        pll <= siga0[LO-1:0]   * sigb0[LO-1:0];
        plh <= siga0[LO-1:0]   * sigb0[SIG-1:LO];
        phl <= siga0[SIG-1:LO] * sigb0[LO-1:0];
        phh <= siga0[SIG-1:LO] * sigb0[SIG-1:LO];
    end

    // ---- stage 2: the middle sum; the outer products side by side (pll takes exactly 2 LO bits)
    reg                    v2, s2, nan2, inf2, zero2;
    reg signed [EXP_W+1:0] esum2;
    reg [LO+HI:0]          mid2;
    reg [2*SIG-1:0]        hl2;
    always @(posedge clk) begin
        v2 <= v1 & ~rst; s2 <= s1; esum2 <= esum1; nan2 <= nan1; inf2 <= inf1; zero2 <= zero1;
        mid2 <= {1'b0, plh} + {1'b0, phl};
        hl2  <= {phh, pll};
    end

    // ---- stage 3: the product, in [1, 4) x 2^(2 M)
    reg                    v3, s3, nan3, inf3, zero3;
    reg signed [EXP_W+1:0] esum3;
    reg [2*SIG-1:0]        prod3;
    always @(posedge clk) begin
        v3 <= v2 & ~rst; s3 <= s2; esum3 <= esum2; nan3 <= nan2; inf3 <= inf2; zero3 <= zero2;
        prod3 <= hl2 + {{(SIG-LO-1){1'b0}}, mid2, {LO{1'b0}}};
    end

    // ---- stage 4: normalise, round, assemble (float_mul's last stage)
    wire           hi      = prod3[2*SIG-1];          // the product is >= 2
    wire [SIG-1:0] mant_n  = hi ? prod3[2*SIG-1:SIG] : prod3[2*SIG-2:SIG-1];
    wire           guard   = hi ? prod3[SIG-1]       : prod3[SIG-2];
    wire           sticky  = hi ? |prod3[SIG-2:0]    : |prod3[SIG-3:0];
    wire signed [EXP_W+1:0] exp_n = esum3 + (hi ? 1 : 0);
    wire           roundup = guard & (sticky | mant_n[0]);
    wire [SIG:0]   mant_r  = {1'b0, mant_n} + (SIG+1)'(roundup);
    wire           carry   = mant_r[SIG];
    wire [MAN_W-1:0] man_final = carry ? mant_r[SIG-1:1] : mant_r[MAN_W-1:0];
    wire signed [EXP_W+1:0] exp_final = exp_n + (carry ? 1 : 0);
    always @(posedge clk) begin
        out_valid <= v3 & ~rst;
        if (nan3)                    result <= QNAN;
        else if (inf3)               result <= {s3, {EXP_W{1'b1}}, {MAN_W{1'b0}}};
        else if (zero3)              result <= {s3, {(W-1){1'b0}}};
        else if (exp_final >= INF_E) result <= {s3, {EXP_W{1'b1}}, {MAN_W{1'b0}}};   // overflow: Inf
        else if (exp_final <= 0)     result <= {s3, {(W-1){1'b0}}};                  // underflow: zero
        else                         result <= {s3, exp_final[EXP_W-1:0], man_final};
    end
endmodule
`default_nettype wire
