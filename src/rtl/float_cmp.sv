// float_cmp — MIN/MAX/SLT/SGE/SEQ/SNE for FTZ floats, sign-magnitude compare.
// Latency 1, registered output. MIN/MAX return the selected operand; the
// set-compares return 1.0/0.0. (Finite-normal path; NaN -> compares false,
// min/max return the non-NaN operand.)
`default_nettype none
module float_cmp #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 in_valid,
    input  wire [2:0]           op,      // 0 MIN,1 MAX,2 SLT,3 SGE,4 SEQ,5 SNE
    input  wire [EXP_W+MAN_W:0] a,
    input  wire [EXP_W+MAN_W:0] b,
    output reg                  out_valid,
    output reg  [EXP_W+MAN_W:0] result
);
    localparam int W     = 1 + EXP_W + MAN_W;
    localparam int INF_E = (1 << EXP_W) - 1;
    localparam [W-1:0] ONE  = (((1<<(EXP_W-1))-1) << MAN_W);   // 1.0
    localparam [W-1:0] ZERO = '0;

    wire s_a=a[W-1], s_b=b[W-1];
    wire [W-2:0] mag_a=a[W-2:0], mag_b=b[W-2:0];
    wire a_zero=(a[W-2:MAN_W]==0), b_zero=(b[W-2:MAN_W]==0);
    wire a_nan=(a[W-2:MAN_W]==INF_E)&&(a[MAN_W-1:0]!=0);
    wire b_nan=(b[W-2:MAN_W]==INF_E)&&(b[MAN_W-1:0]!=0);
    wire bothz = a_zero & b_zero;

    wire eq = (a==b) | bothz;
    wire lt = bothz                  ? 1'b0 :
              (s_a ^ s_b)            ? s_a  :          // diff sign: neg < pos
              (~s_a)                 ? (mag_a <  mag_b):  // both pos
                                       (mag_a >  mag_b);  // both neg

    reg [W-1:0] r;
    always @* begin
        case (op)
            3'd0: r = a_nan ? b : b_nan ? a : (lt ? a : b);   // MIN
            3'd1: r = a_nan ? b : b_nan ? a : (lt ? b : a);   // MAX
            3'd2: r = (a_nan|b_nan) ? ZERO : (lt        ? ONE : ZERO); // SLT
            3'd3: r = (a_nan|b_nan) ? ZERO : ((~lt)     ? ONE : ZERO); // SGE
            3'd4: r = (a_nan|b_nan) ? ZERO : (eq        ? ONE : ZERO); // SEQ
            default: r = (a_nan|b_nan) ? ONE : ((~eq)   ? ONE : ZERO); // SNE
        endcase
    end
    always @(posedge clk) begin
        out_valid <= in_valid & ~rst;
        result    <= r;
    end
endmodule
`default_nettype wire
