// float_cmp3 — float_cmp with a third operand and one more operation: op 6 = CLAMP, result = MIN(MAX(a, b), c), bit for
// bit what float_cmp gives for MAX then MIN (a clamp in one micro-op of the fragment core). Ops
// 0 .. 5 are float_cmp's, untouched: 0 MIN 1 MAX 2 SLT 3 SGE 4 SEQ 5 SNE. Latency 1, registered output.
//
// CLAMP in one level beside the others: t = MAX(a, b) is a or b, so MIN(t, c)'s compare is one of two that do not need
// t — a against c when t = a, b against c when t = b — and both run beside a against b. The NaN rules are the two
// operations' own (MIN / MAX return the operand that is not a NaN).
`default_nettype none
module float_cmp3 #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 in_valid,
    input  wire [2:0]           op,      // 0 MIN,1 MAX,2 SLT,3 SGE,4 SEQ,5 SNE,6 CLAMP
    input  wire [EXP_W+MAN_W:0] a,
    input  wire [EXP_W+MAN_W:0] b,
    input  wire [EXP_W+MAN_W:0] c,       // CLAMP's upper bound; unused by the other ops
    output reg                  out_valid,
    output reg  [EXP_W+MAN_W:0] result
);
    localparam int W     = 1 + EXP_W + MAN_W;
    localparam int INF_E = (1 << EXP_W) - 1;
    localparam [W-1:0] ONE  = (((1<<(EXP_W-1))-1) << MAN_W);   // 1.0
    localparam [W-1:0] ZERO = '0;

    function automatic f_zero(input [W-1:0] x); f_zero = (x[W-2:MAN_W] == 0); endfunction
    function automatic f_nan (input [W-1:0] x); f_nan  = (x[W-2:MAN_W] == INF_E) && (x[MAN_W-1:0] != 0); endfunction
    // x < y, float_cmp's rule: two zeros (FTZ: exponent 0) are equal, else sign then magnitude
    function automatic f_lt(input [W-1:0] x, input [W-1:0] y);
        if (f_zero(x) & f_zero(y)) f_lt = 1'b0;
        else if (x[W-1] ^ y[W-1])  f_lt = x[W-1];
        else if (~x[W-1])          f_lt = (x[W-2:0] < y[W-2:0]);
        else                       f_lt = (x[W-2:0] > y[W-2:0]);
    endfunction

    wire a_nan = f_nan(a), b_nan = f_nan(b), c_nan = f_nan(c);
    wire bothz = f_zero(a) & f_zero(b);
    wire eq = (a == b) | bothz;
    wire lt = f_lt(a, b);
    // CLAMP: t = MAX(a, b) takes a unless (a is a NaN) or (neither is and a < b)
    wire t_is_a  = ~a_nan & (b_nan | ~lt);
    wire t_nan   = t_is_a ? a_nan : b_nan;                       // (a NaN only when both are)
    wire lt_tc   = t_is_a ? f_lt(a, c) : f_lt(b, c);
    wire [W-1:0] t = t_is_a ? a : b;
    // MIN(t, c) = t_nan ? c : c_nan ? t : (t < c ? t : c)
    wire r_is_c  = t_nan | (~c_nan & ~lt_tc);

    reg [W-1:0] r;
    always @* begin
        case (op)
            3'd0: r = a_nan ? b : b_nan ? a : (lt ? a : b);   // MIN
            3'd1: r = a_nan ? b : b_nan ? a : (lt ? b : a);   // MAX
            3'd2: r = (a_nan|b_nan) ? ZERO : (lt        ? ONE : ZERO); // SLT
            3'd3: r = (a_nan|b_nan) ? ZERO : ((~lt)     ? ONE : ZERO); // SGE
            3'd4: r = (a_nan|b_nan) ? ZERO : (eq        ? ONE : ZERO); // SEQ
            3'd5: r = (a_nan|b_nan) ? ONE : ((~eq)      ? ONE : ZERO); // SNE
            default: r = r_is_c ? c : t;                      // 6 CLAMP
        endcase
    end
    always @(posedge clk) begin
        out_valid <= in_valid & ~rst;
        result    <= r;
    end
endmodule
`default_nettype wire
