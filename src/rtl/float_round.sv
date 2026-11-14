// float_round — float -> float rounding to an integer value, latency 2, II = 1 (ISA TRUNC / FLR / CEIL / RNDE).
//   mode: 0 trunc (toward 0), 1 floor (toward -inf), 2 ceil (toward +inf), 3 round half to even.
// Exact for every input: |x| >= 2^MAN_W is already an integer and passes through, as do Inf / NaN (unlike
// FTOI + ITOF, which saturates at the integer width). Zero / denormal (FTZ) -> zero with the input's sign; a zero
// result keeps the input's sign (ceil(-0.3) = -0). Bit-exact to tools/float_ref.py fround.
//   S1: classify (special, zero, |x| < 1, integer already), fraction mask, round-up decision
//   S2: integer significand + one unit at the integer LSB, carry into the exponent, pack
`default_nettype none
module float_round #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16
)(
    input  wire                 clk, rst,
    input  wire                 in_valid,
    input  wire [1:0]           mode,
    input  wire [EXP_W+MAN_W:0] x,
    output reg                  out_valid,
    output reg  [EXP_W+MAN_W:0] result
);
    localparam int W = 1 + EXP_W + MAN_W, BIAS = (1 << (EXP_W-1)) - 1, INF_E = (1 << EXP_W) - 1;
    localparam int EXW = EXP_W + 2;
    localparam int FBW = $clog2(MAN_W + 1);                       // fraction-bit count 1..MAN_W

    wire             s  = x[W-1];
    wire [EXP_W-1:0] ef = x[W-2:MAN_W];
    wire [MAN_W-1:0] mf = x[MAN_W-1:0];
    wire signed [EXW-1:0] e = $signed({{(EXW-EXP_W){1'b0}}, ef}) - BIAS;
    wire [MAN_W:0]   sig = {1'b1, mf};

    // fraction bits of the significand when 0 <= e < MAN_W: F = MAN_W - e
    wire [FBW-1:0]   F    = MAN_W - e[FBW-1:0];
    wire [MAN_W:0]   fm   = ({{MAN_W{1'b0}}, 1'b1} << F) - 1'b1;      // fraction mask
    wire [MAN_W:0]   frac = sig & fm;
    wire             nz   = |frac;
    wire             half = |(frac & (fm ^ (fm >> 1)));             // bit F-1
    wire             stk  = |(frac & (fm >> 1));                    // bits below F-1
    wire             lsb  = |(sig & (fm + 1'b1));                   // bit F (the integer LSB)
    reg              up;
    always @* case (mode)
        2'd0:    up = 1'b0;
        2'd1:    up = s & nz;
        2'd2:    up = ~s & nz;
        default: up = half & (stk | lsb);
    endcase
    // |x| < 1: the result is 0 or 1 in magnitude
    wire one_small = (mode == 2'd1) ? s : (mode == 2'd2) ? ~s : (mode == 2'd3) ? ((e == -1) && (mf != 0)) : 1'b0;

    localparam [1:0] C_PASS = 2'd0, C_ZERO = 2'd1, C_ONE = 2'd2, C_MID = 2'd3;
    reg v1; reg [1:0] c1; reg s1; reg [W-1:0] x1; reg signed [EXW-1:0] e1; reg [MAN_W:0] isig1, unit1; reg up1;
    always @(posedge clk) begin
        v1 <= in_valid & ~rst; s1 <= s; x1 <= x; e1 <= e; up1 <= up;
        isig1 <= sig & ~fm; unit1 <= fm + 1'b1;
        if (ef == INF_E || e >= MAN_W)  c1 <= C_PASS;              // Inf / NaN / already an integer
        else if (ef == 0)               c1 <= C_ZERO;              // zero / denormal
        else if (e < 0)                 c1 <= one_small ? C_ONE : C_ZERO;
        else                            c1 <= C_MID;
    end
    wire [MAN_W+1:0] inc = {1'b0, isig1} + (up1 ? {1'b0, unit1} : '0);
    wire             cy  = inc[MAN_W+1];                           // rounded up to the next binade
    wire signed [EXW-1:0] eo = e1 + cy;
    always @(posedge clk) begin
        out_valid <= v1 & ~rst;
        case (c1)
            C_PASS:  result <= x1;
            C_ZERO:  result <= {s1, {(W-1){1'b0}}};
            C_ONE:   result <= {s1, BIAS[EXP_W-1:0], {MAN_W{1'b0}}};
            default: result <= {s1, eo[EXP_W-1:0] + BIAS[EXP_W-1:0], cy ? {MAN_W{1'b0}} : inc[MAN_W-1:0]};
        endcase
    end
endmodule
`default_nettype wire
