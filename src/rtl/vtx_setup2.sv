// vtx_setup2 — per-vertex clip space -> window for board A's setup stage (a_setup.sv; model tools/a_setup_ref.py
//   vertex_setup). iw = 1/w (the SFU reciprocal), NDC = (x, y, z) * iw, window x / y in 1/16 PIXEL = round-half-even
//   (float_round RNDE) of NDC * scale16 + offset16 (the viewport constants premultiplied by 16 by the host), then to a
//   signed integer saturated to 16 bits (+-2048 pixels: the guard band); window z = ndc_z * zs + zo (the depth range:
//   zs = (f - n) / 2, zo = (n + f) / 2 from VS_STATE words 7 / 8, 0.5 / 0.5 for the default [0, 1]). reject = w <= 0
//   (sign set or FTZ zero). Latency-independent by construction: every value that must reach the output
//   beside its result rides a self-aligning FIFO, pushed by its producer's valid and popped by the consumer's.
`default_nettype none
module vtx_setup2 #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16,
    parameter int FIFO_DEPTH = 32               // >= the in_valid -> out_valid latency (~30 at II = 1); a power of 2
)(
    input  wire                 clk, rst,
    input  wire                 in_valid,
    input  wire [EXP_W+MAN_W:0] x, y, z, w,
    input  wire [EXP_W+MAN_W:0] sxs, sxo, sys, syo,      // window = ndc * s + o, in 1/16 pixel (FP24)
    input  wire [EXP_W+MAN_W:0] zs, zo,                  // window z = ndc_z * zs + zo: glDepthRangef's (f - n) / 2, (n + f) / 2
    output wire                 out_valid,
    output wire signed [15:0]   sx, sy,                   // 1/16 pixel, saturated to 16 bits
    output wire signed [17:0]   sx18, sy18,               // the same before the saturation (a point's centre)
    output wire [EXP_W+MAN_W:0] zw, iw_out,
    output wire                 reject
);
    localparam int W = 1 + EXP_W + MAN_W, AW = $clog2(FIFO_DEPTH);
    wire rej_in = w[W-1] | (w[W-2:MAN_W] == '0);
    // 1/w
    wire rv; wire [W-1:0] iw;
    float_sfu #(.EXP_W(EXP_W), .MAN_W(MAN_W), .K(12), .KLOG(11),
        .RCP_MEM("tools/gen/fp24_rcp_seed.mem"), .RSQRT_MEM("tools/gen/fp24_rsqrt_seed.mem"),
        .LOG2_MEM("tools/gen/fp24_log2_lut.mem"), .EXP2_MEM("tools/gen/fp24_exp2_lut.mem"))
      u_rcp (.clk, .rst, .in_valid, .op(3'd0), .x(w), .out_valid(rv), .result(iw));
    // self-aligning FIFOs: xyz (push in_valid, pop rv), rej (in_valid -> out_valid), iw (rv -> out_valid), z (zv -> out_valid)
    reg [3*W-1:0] xyzq [0:FIFO_DEPTH-1]; reg [AW:0] xyz_wp, xyz_rp;
    reg [W-1:0]   iwq  [0:FIFO_DEPTH-1]; reg [AW:0] iw_wp, iw_rp;
    reg [W-1:0]   zq   [0:FIFO_DEPTH-1]; reg [AW:0] z_wp, z_rp;
    reg           rejq [0:FIFO_DEPTH-1]; reg [AW:0] rej_wp, rej_rp;
    wire mvx, mvy, mvz, avx, avy, avz, rox, roy, fvx, fvy;
    wire [W-1:0] ndcx, ndcy, ndcz, sxf, syf, zwf, rxf, ryf;
    wire signed [17:0] ix, iy;
    wire [3*W-1:0] xyz_head = xyzq[xyz_rp[AW-1:0]];
    always @(posedge clk) begin
        if (rst) begin xyz_wp <= 0; xyz_rp <= 0; iw_wp <= 0; iw_rp <= 0; z_wp <= 0; z_rp <= 0; rej_wp <= 0; rej_rp <= 0; end
        else begin
            if (in_valid) begin xyzq[xyz_wp[AW-1:0]] <= {z, y, x}; xyz_wp <= xyz_wp + 1'b1; rejq[rej_wp[AW-1:0]] <= rej_in; rej_wp <= rej_wp + 1'b1; end
            if (rv) begin xyz_rp <= xyz_rp + 1'b1; iwq[iw_wp[AW-1:0]] <= iw; iw_wp <= iw_wp + 1'b1; end
            if (avz) begin zq[z_wp[AW-1:0]] <= zwf; z_wp <= z_wp + 1'b1; end
            if (out_valid) begin iw_rp <= iw_rp + 1'b1; z_rp <= z_rp + 1'b1; rej_rp <= rej_rp + 1'b1; end
        end
    end
    float_mul #(EXP_W, MAN_W) mx (.clk, .rst, .in_valid(rv), .a(xyz_head[0*W +: W]), .b(iw), .in_ready(), .out_valid(mvx), .result(ndcx));
    float_mul #(EXP_W, MAN_W) my (.clk, .rst, .in_valid(rv), .a(xyz_head[1*W +: W]), .b(iw), .in_ready(), .out_valid(mvy), .result(ndcy));
    float_mul #(EXP_W, MAN_W) mz (.clk, .rst, .in_valid(rv), .a(xyz_head[2*W +: W]), .b(iw), .in_ready(), .out_valid(mvz), .result(ndcz));
    float_madd #(EXP_W, MAN_W) ax (.clk, .rst, .in_valid(mvx), .sub(1'b0), .a(ndcx), .b(sxs), .c(sxo), .out_valid(avx), .result(sxf), .prod_o());
    float_madd #(EXP_W, MAN_W) ay (.clk, .rst, .in_valid(mvy), .sub(1'b0), .a(ndcy), .b(sys), .c(syo), .out_valid(avy), .result(syf), .prod_o());
    float_madd #(EXP_W, MAN_W) az (.clk, .rst, .in_valid(mvz), .sub(1'b0), .a(ndcz), .b(zs), .c(zo), .out_valid(avz), .result(zwf), .prod_o());
    float_round #(EXP_W, MAN_W) rx (.clk, .rst, .in_valid(avx), .mode(2'd3), .x(sxf), .out_valid(rox), .result(rxf));
    float_round #(EXP_W, MAN_W) ry (.clk, .rst, .in_valid(avy), .mode(2'd3), .x(syf), .out_valid(roy), .result(ryf));
    float_cvt_f2i #(.EXP_W(EXP_W), .MAN_W(MAN_W), .OW(18)) fx (.clk, .rst, .in_valid(rox), .fval(rxf), .out_valid(fvx), .ival(ix));
    float_cvt_f2i #(.EXP_W(EXP_W), .MAN_W(MAN_W), .OW(18)) fy (.clk, .rst, .in_valid(roy), .fval(ryf), .out_valid(fvy), .ival(iy));
    function automatic signed [15:0] sat16(input signed [17:0] v);
        sat16 = (v > 18'sd32767) ? 16'sd32767 : (v < -18'sd32768) ? -16'sd32768 : v[15:0];
    endfunction
    assign out_valid = fvx;
    assign sx = sat16(ix); assign sy = sat16(iy);
    assign sx18 = ix; assign sy18 = iy;
    assign iw_out = iwq[iw_rp[AW-1:0]];
    assign zw = zq[z_rp[AW-1:0]];
    assign reject = rejq[rej_rp[AW-1:0]];
    wire _unused = &{1'b0, mvz, fvy};
endmodule
`default_nettype wire
