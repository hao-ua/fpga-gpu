// vec4_sincos — one sincos_sfu serving a vec4 (4 lanes), mirroring vec4_sfu's "1 unit per 4 lanes"
// sharing: feeds the 4 components through the II=1 sincos_sfu on consecutive cycles and reassembles the
// vec4. Separate from vec4_sfu so SIN/COS run as a second transcendental port in parallel with RCP/etc.
//   op: 0 = SIN, 1 = COS.  Accepts a new vec4 every ~4-6 cycles (busy-gated); inner unit is II=1.
`default_nettype none
module vec4_sincos #(
    parameter int EXP_W = 5,
    parameter int MAN_W = 10,
    parameter int KSIN  = 10,
    parameter     SIN_MEM=""
)(
    input  wire                       clk,
    input  wire                       rst,
    input  wire                       in_valid,
    input  wire                       op,        // 0 SIN, 1 COS
    input  wire [4*(EXP_W+MAN_W+1)-1:0] x,
    output wire                       ready,     // can accept a new vec4
    output reg                        out_valid,
    output reg  [4*(EXP_W+MAN_W+1)-1:0] result
);
    localparam int W = 1 + EXP_W + MAN_W;

    reg  [4*W-1:0] xv;
    reg            opv;
    reg  [1:0]     feed_cnt, out_cnt;
    reg            feeding, busy;
    assign ready = ~busy;

    wire           sc_ov;
    wire [W-1:0]   sc_res;
    wire [W-1:0]   sc_x = xv[feed_cnt*W +: W];

    sincos_sfu #(.EXP_W(EXP_W),.MAN_W(MAN_W),.KSIN(KSIN),.SIN_MEM(SIN_MEM))
      u_sc(.clk,.rst,.in_valid(feeding),.op(opv),.x(sc_x),
           .out_valid(sc_ov),.result(sc_res));

    reg [3*W-1:0] partial;   // lanes 0..2 collected

    always @(posedge clk) begin
        out_valid <= 1'b0;
        if (rst) begin
            feeding<=1'b0; busy<=1'b0; feed_cnt<=0; out_cnt<=0;
        end else begin
            if (in_valid && ~busy) begin
                xv<=x; opv<=op; feeding<=1'b1; feed_cnt<=2'd0; out_cnt<=2'd0; busy<=1'b1;
            end
            if (feeding) begin
                if (feed_cnt==2'd3) feeding<=1'b0;
                feed_cnt <= feed_cnt + 2'd1;
            end
            if (sc_ov) begin
                if (out_cnt==2'd3) begin
                    result    <= {sc_res, partial};   // lane3 + lanes0..2
                    out_valid <= 1'b1;
                    busy      <= 1'b0;
                end else begin
                    partial[out_cnt*W +: W] <= sc_res;
                end
                out_cnt <= out_cnt + 2'd1;
            end
        end
    end
endmodule
`default_nettype wire
