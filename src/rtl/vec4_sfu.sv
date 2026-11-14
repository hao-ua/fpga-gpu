// vec4_sfu — one shared float_sfu serving a vec4 (4 lanes). Feeds the 4
// components through the II=1 SFU pipe on consecutive cycles and reassembles
// the vec4 result. This is the canonical "1 SFU per 4 lanes" sharing.
// Pipelined: the inner float_sfu is II=1/lat-10, so vec4s CHAIN back-to-back
// instead of busy-waiting the full feed+latency+collect round trip.
// A 1-deep input queue (the "2nd assembly slot") parks the next vec4 while the
// feeder streams the current one; the collector wraps mod 4 and emits results
// strictly in issue order. Steady-state II = 4 (the feed floor). A lone op is
// fed at once (accept -> feed lane0 next cycle), so fixed-latency TBs
// (tb_alu_cluster's 20-cycle RCP wait) hold.
`default_nettype none
module vec4_sfu #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16,
    parameter int K     = 12,
    parameter int KLOG  = 11,
    parameter     RCP_MEM="", RSQRT_MEM="", LOG2_MEM="", EXP2_MEM=""
)(
    input  wire                       clk,
    input  wire                       rst,
    input  wire                       in_valid,
    input  wire [2:0]                 op,        // 0 RCP,1 RSQRT,2 LOG2,3 EXP2 (SIN/COS: vec4_sincos)
    input  wire [4*(EXP_W+MAN_W+1)-1:0] x,
    output wire                       ready,     // can accept a new vec4
    output reg                        out_valid,
    output reg  [4*(EXP_W+MAN_W+1)-1:0] result
);
    localparam int W = 1 + EXP_W + MAN_W;

    reg  [4*W-1:0] xv, xq;            // feeder operand / queued operand
    reg  [2:0]     opv, opq;
    reg  [1:0]     feed_cnt, out_cnt;
    reg            feeding, qv;       // qv = input-queue slot occupied
    assign ready = ~qv;

    // feeder handoff: start a new vec4 directly from the input when the feeder is
    // free (idle, or on its last lane with an empty queue), else pop the queue.
    // take requires ~qv, so fstart_in and fstart_q are mutually exclusive.
    wire chain     = feeding & (feed_cnt==2'd3);
    wire take      = in_valid & ~qv;
    wire fstart_in = take & (~feeding | chain);
    wire fstart_q  = qv   & (~feeding | chain);

    wire           sfu_ov;
    wire [W-1:0]   sfu_res;
    wire [W-1:0]   sfu_x = xv[feed_cnt*W +: W];

    float_sfu #(.EXP_W(EXP_W),.MAN_W(MAN_W),.K(K),.KLOG(KLOG),
        .RCP_MEM(RCP_MEM),.RSQRT_MEM(RSQRT_MEM),.LOG2_MEM(LOG2_MEM),.EXP2_MEM(EXP2_MEM))
      u_sfu(.clk,.rst,.in_valid(feeding),.op(opv),.x(sfu_x),
            .out_valid(sfu_ov),.result(sfu_res));

    reg [3*W-1:0] partial;   // lanes 0..2 of the vec4 currently collecting

    always @(posedge clk) begin
        out_valid <= 1'b0;
        if (rst) begin
            feeding<=1'b0; qv<=1'b0; feed_cnt<=0; out_cnt<=0;
        end else begin
            // input side: park in the queue when the feeder can't take this vec4 now
            if (take & ~fstart_in) begin xq<=x; opq<=op; qv<=1'b1; end
            // feeder: II=1 into float_sfu; back-to-back vec4s share the pipe (II=4)
            if (fstart_in)     begin xv<=x;  opv<=op;  feed_cnt<=2'd0; feeding<=1'b1; end
            else if (fstart_q) begin xv<=xq; opv<=opq; feed_cnt<=2'd0; feeding<=1'b1; qv<=1'b0; end
            else if (feeding)  begin feed_cnt<=feed_cnt+2'd1; if (chain) feeding<=1'b0; end
            // collector: results arrive II=1 in issue order; groups of 4 per vec4.
            // out_cnt wraps 3->0 into the next vec4's group; partial is free the cycle
            // after result latches (the next group's lane0 lands 1+ cycles later).
            if (sfu_ov) begin
                if (out_cnt==2'd3) begin
                    result    <= {sfu_res, partial};   // lane3 + lanes0..2
                    out_valid <= 1'b1;
                end else begin
                    partial[out_cnt*W +: W] <= sfu_res;
                end
                out_cnt <= out_cnt + 2'd1;
            end
        end
    end
endmodule
`default_nettype wire
