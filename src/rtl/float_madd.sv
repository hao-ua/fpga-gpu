// float_madd — fused-style multiply-add  r = a*b + c  (round-mul, then round-add;
// 2-rounding, ~1 ULP — the standard GPU MAD, not an IEEE single-rounding FMA).
// Composes float_mul (lat 3) and float_addsub (FP16 lat 3 / FP24 lat 5) -> FP16 lat 6, FP24 lat 8, II=1.
// MAC (r += a*b) and DP3/DP4 are built from this unit by the core.
`default_nettype none
module float_madd #(
    parameter int EXP_W = 5,
    parameter int MAN_W = 10,
    parameter bit DSP_ALIGN = 1'b0             // the adder's alignment shift on a DSP48 (float_addsub, FP24 only)
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 in_valid,
    input  wire                 sub,          // 0: a*b+c, 1: a*b-c
    input  wire [EXP_W+MAN_W:0] a,
    input  wire [EXP_W+MAN_W:0] b,
    input  wire [EXP_W+MAN_W:0] c,
    output wire                 out_valid,
    output wire [EXP_W+MAN_W:0] result,
    // the rounded product a*b, 3 cycles after the operands whether or not in_valid was set (float_mul's datapath
    // is not gated): alu_cluster's dot products sum these instead of multiplying again
    output wire [EXP_W+MAN_W:0] prod_o
);
    localparam int W = 1 + EXP_W + MAN_W;

    wire [W-1:0] prod; wire mul_v;
    assign prod_o = prod;
    float_mul #(EXP_W,MAN_W) u_mul (
        .clk, .rst, .in_valid, .a, .b, .in_ready(),
        .out_valid(mul_v), .result(prod));

    // delay c + sub by the multiply latency (3) so they meet the product
    localparam int MUL_LAT = 3;
    reg [W-1:0] c_d   [0:MUL_LAT-1];
    reg         sub_d [0:MUL_LAT-1];
    integer i;
    always @(posedge clk) begin
        c_d[0]   <= c;   sub_d[0] <= sub;
        for (i=1; i<MUL_LAT; i=i+1) begin
            c_d[i] <= c_d[i-1]; sub_d[i] <= sub_d[i-1];
        end
    end

    float_addsub #(.EXP_W(EXP_W), .MAN_W(MAN_W), .DSP_ALIGN(DSP_ALIGN)) u_add (
        .clk, .rst, .in_valid(mul_v), .op(sub_d[MUL_LAT-1]),
        .a(prod), .b(c_d[MUL_LAT-1]), .in_ready(),
        .out_valid(out_valid), .result(result));
endmodule
`default_nettype wire
