// float_dot4_sum — the reduction half of float_dot4: result = (p0 + p1) + (p2 + p3) for 4 already-rounded products
// (alu_cluster takes them from its multiply-add lanes, which multiply the same operands: no second set of
// multipliers). dp3 = 1 replaces p3 by +0, exactly what float_dot4 gets from 0*0. Bit-identical to float_dot4 with
// the products delayed by the multiply latency. Latency 2 adds (FP16 6, FP24 10); in_valid / dp3 come with the
// products.
`default_nettype none
module float_dot4_sum #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16
)(
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         in_valid,
    input  wire                         dp3,
    input  wire [4*(EXP_W+MAN_W+1)-1:0] p,
    output wire                         out_valid,
    output wire [EXP_W+MAN_W:0]         result
);
    localparam int W = 1 + EXP_W + MAN_W;
    wire [W-1:0] p0 = p[0*W +: W], p1 = p[1*W +: W], p2 = p[2*W +: W];
    wire [W-1:0] p3 = dp3 ? {W{1'b0}} : p[3*W +: W];
    wire [W-1:0] s01, s23; wire vs01, vs23;
    float_addsub #(EXP_W,MAN_W) a01(.clk,.rst,.in_valid,.op(1'b0),.a(p0),.b(p1),.in_ready(),.out_valid(vs01),.result(s01));
    float_addsub #(EXP_W,MAN_W) a23(.clk,.rst,.in_valid,.op(1'b0),.a(p2),.b(p3),.in_ready(),.out_valid(vs23),.result(s23));
    float_addsub #(EXP_W,MAN_W) af (.clk,.rst,.in_valid(vs01),.op(1'b0),.a(s01),.b(s23),.in_ready(),.out_valid(out_valid),.result(result));
endmodule
`default_nettype wire
