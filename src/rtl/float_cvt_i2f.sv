// float_cvt_i2f — signed integer -> float (EXP_W,MAN_W), round-to-nearest-even.
// Latency 2 (input reg -> normalize/round -> output reg), II=1; SPLIT = 1: the normalized value registered before the round /
// pack, latency 3 (timing at 120 MHz: the attribute feed's 49-bit converters are 24 logic levels in one stage).
`default_nettype none
module float_cvt_i2f #(
    parameter int IW    = 32,
    parameter int EXP_W = 7,
    parameter int MAN_W = 16,
    parameter bit SPLIT = 1'b0
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 in_valid,
    input  wire signed [IW-1:0] ival,
    output reg                  out_valid,
    output reg  [EXP_W+MAN_W:0] result
);
    localparam int W    = 1+EXP_W+MAN_W;
    localparam int BIAS = (1<<(EXP_W-1))-1;
    localparam int INF_E= (1<<EXP_W)-1;

    // ===== S0: |ival| + MSB search (priority encoder) — registered so the
    //        shift+round path in S1 is a separate, shorter stage (latency 2 kept).
    wire [IW-1:0] magw = ival[IW-1] ? (~ival + 1'b1) : ival;   // |ival|
    integer j; reg [$clog2(IW+1)-1:0] msbc; reg foundc;
    always @* begin
        msbc=0; foundc=0;
        for (j=IW-1;j>=0;j=j-1) if (!foundc && magw[j]) begin msbc=j[$clog2(IW+1)-1:0]; foundc=1; end
    end
    reg v0; reg sgn0; reg [IW-1:0] mag0; reg [$clog2(IW+1)-1:0] msb0; reg found0;
    always @(posedge clk) begin
        v0  <= in_valid;
        sgn0<= ival[IW-1];
        mag0<= magw;
        msb0<= msbc;
        found0<= foundc;
    end

    // ===== S1: normalize to 1.MAN_W (S1a), round, pack (S1b; one stage unless SPLIT) =====
    reg [MAN_W-1:0] man_n; reg g_n, stk_n; reg signed [EXP_W+1:0] ex_n;
    wire [$clog2(IW+1)-1:0] msb = msb0;
    always @* begin
        ex_n = msb;
        if (msb >= MAN_W) begin
            man_n = mag0 >> (msb-MAN_W);
            g_n   = (msb-MAN_W>=1) ? mag0[msb-MAN_W-1] : 1'b0;
            stk_n = (msb-MAN_W>=2) ? |(mag0 & (({{(IW-1){1'b0}},1'b1})<<(msb-MAN_W-1))-1) : 1'b0;
        end else begin
            man_n = mag0 << (MAN_W-msb); g_n=0; stk_n=0;
        end
    end
    wire [MAN_W-1:0] man_s; wire g_s, stk_s; wire signed [EXP_W+1:0] ex_s; wire v_s, sgn_s, found_s;
    generate if (SPLIT) begin : g_split
        reg [MAN_W-1:0] man_r; reg g_r, stk_r; reg signed [EXP_W+1:0] ex_r; reg v_r, sgn_r, found_r;
        always @(posedge clk) begin man_r<=man_n; g_r<=g_n; stk_r<=stk_n; ex_r<=ex_n; v_r<=v0; sgn_r<=sgn0; found_r<=found0; end
        assign man_s=man_r; assign g_s=g_r; assign stk_s=stk_r; assign ex_s=ex_r; assign v_s=v_r; assign sgn_s=sgn_r; assign found_s=found_r;
    end else begin : g_one
        assign man_s=man_n; assign g_s=g_n; assign stk_s=stk_n; assign ex_s=ex_n; assign v_s=v0; assign sgn_s=sgn0; assign found_s=found0;
    end endgenerate
    reg [MAN_W-1:0] man; reg [MAN_W:0] mr;
    reg signed [EXP_W+1:0] ex;
    reg [W-1:0] res;
    always @* begin
        ex = ex_s;
        mr = {1'b0,man_s} + (g_s & (stk_s | man_s[0]));
        if (mr[MAN_W]) ex = ex + 1;
        man = mr[MAN_W-1:0];
        if (!found_s)               res = {sgn_s,{(W-1){1'b0}}};      // zero
        else if (ex+BIAS >= INF_E)  res = {sgn_s,{EXP_W{1'b1}},{MAN_W{1'b0}}};
        else                        res = {sgn_s, ex[EXP_W-1:0]+BIAS[EXP_W-1:0], man};
    end

    always @(posedge clk) begin
        out_valid <= v_s & ~rst;
        result    <= res;
    end
endmodule
`default_nettype wire
