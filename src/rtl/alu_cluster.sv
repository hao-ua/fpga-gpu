// alu_cluster — parameterized multi-lane float ALU block.
//   N_VEC4 vec4 groups, each = 4 madd lanes + 4 compares + 1 dot4 + 1 shared SFU.
//   Instantiate FP24/N_VEC4=1  -> VPU ALU (4 lanes)
//             FP16/N_VEC4=4  -> FPU ALU (16 lanes, 4 SFUs = 1 per pixel)
// Each functional unit drives its own result port + valid (they are independent
// units; assert the matching *_valid for the op you want). All outputs registered.
`default_nettype none
module alu_cluster #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16,
    parameter int N_VEC4= 1,
    parameter int K     = 12,
    parameter int KLOG  = 11,
    parameter int KSIN  = 10,
    parameter     RCP_MEM="", RSQRT_MEM="", LOG2_MEM="", EXP2_MEM="", SIN_MEM=""
)(
    input  wire clk,
    input  wire rst,
    // operands: NL = N_VEC4*4 lanes, lane l at [l*W +: W]
    input  wire [N_VEC4*4*(EXP_W+MAN_W+1)-1:0] a,
    input  wire [N_VEC4*4*(EXP_W+MAN_W+1)-1:0] b,
    input  wire [N_VEC4*4*(EXP_W+MAN_W+1)-1:0] c,
    // fast multiply-add (per lane): a*b +/- c
    input  wire                                madd_valid,
    input  wire                                madd_sub,
    output wire                                v_madd,
    output wire [N_VEC4*4*(EXP_W+MAN_W+1)-1:0] y_madd,
    // compare / min / max (per lane)
    input  wire                                cmp_valid,
    input  wire [2:0]                          cmp_op,
    output wire                                v_cmp,
    output wire [N_VEC4*4*(EXP_W+MAN_W+1)-1:0] y_cmp,
    // dot product (per vec4 group): dot(a,b)
    input  wire                                dot_valid,
    input  wire                                dp3,
    output wire                                v_dot,
    output wire [N_VEC4*(EXP_W+MAN_W+1)-1:0]   y_dot,
    // SFU (per vec4 group, operates on a): rcp/rsqrt/log2/exp2  (sin/cos: the sincos port below)
    input  wire                                sfu_valid,
    input  wire [2:0]                          sfu_op,
    output wire                                sfu_ready,
    output wire                                v_sfu,
    output wire [N_VEC4*4*(EXP_W+MAN_W+1)-1:0] y_sfu,
    // SINCOS (per vec4 group, operates on a): sin/cos — a SECOND transcendental port, runs in parallel
    // with the SFU (shorter latency; lets sin overlap rcp/rsqrt). op: 0 = SIN, 1 = COS.
    input  wire                                sincos_valid,
    input  wire                                sincos_op,
    output wire                                sincos_ready,
    output wire                                v_sincos,
    output wire [N_VEC4*4*(EXP_W+MAN_W+1)-1:0] y_sincos,
    // convert (per lane, on a): 0=float->int truncate (FTOI), 1=int->float (ITOF)
    // cvt_op: 0 FTOI (float -> int, truncate), 1 ITOF (int -> float), 2 FTOU (float -> unsigned, truncate, negative
    //         -> 0, saturate at 2^W - 1), 3 UTOF (unsigned -> float), 4..7 float -> float rounding TRUNC / FLR / CEIL /
    //         RNDE (float_round, exact over the whole range); all latency 2
    input  wire                                cvt_valid,
    input  wire [2:0]                          cvt_op,
    output wire                                v_cvt,
    output wire [N_VEC4*4*(EXP_W+MAN_W+1)-1:0] y_cvt,
    // integer ALU (separate hardware, int_unit): IMUL low W bits (latency 3), IDIV signed (one in flight, a, b)
    input  wire                                imul_valid,
    output wire                                v_imul,
    output wire [N_VEC4*4*(EXP_W+MAN_W+1)-1:0] y_imul,
    input  wire                                idiv_valid,
    input  wire [1:0]                          idiv_mode,     // 0 IDIV, 1 IMOD, 2 UDIV, 3 UMOD
    output wire                                idiv_ready,
    output wire                                v_idiv,
    output wire [N_VEC4*4*(EXP_W+MAN_W+1)-1:0] y_idiv,
    // integer ALU logic / shift / min / max / unsigned compares (IOP fn, latency 2)
    input  wire                                iop_valid,
    input  wire [3:0]                          iop_fn,
    output wire                                v_iop,
    output wire [N_VEC4*4*(EXP_W+MAN_W+1)-1:0] y_iop
)/* verilator lint_off WIDTH */;
    localparam int W  = 1 + EXP_W + MAN_W;
    localparam int NL = N_VEC4*4;

    wire [NL-1:0] vm, vc, vcv;
    wire [N_VEC4-1:0] vd, vs, sr, vsc, scr;
    wire [NL*(W+1)-1:0] f2i_o;                   // FTOI / FTOU: one signed W+1-bit converter, clamped after
    wire [NL*W-1:0] i2f_o, rnd_o;
    wire [NL*W-1:0] prod;                        // multiply-add lanes' rounded products (3 cycles after a, b)
    reg  [2:0] dot_v3, dp3_d3;                   // dot_valid / dp3 delayed by the multiply latency
    always @(posedge clk) begin
        dot_v3 <= {dot_v3[1:0], dot_valid & ~rst}; dp3_d3 <= {dp3_d3[1:0], dp3};
    end
    reg  [2:0] cvtop_d1, cvtop_d2;               // delay cvt_op to match latency-2 output
    always @(posedge clk) begin cvtop_d1 <= cvt_op; cvtop_d2 <= cvtop_d1; end

    genvar l, g;
    generate
        // per-lane fast units
        for (l=0; l<NL; l=l+1) begin : LANE
            float_madd #(EXP_W,MAN_W) u_madd(
                .clk,.rst,.in_valid(madd_valid),.sub(madd_sub),
                .a(a[l*W+:W]),.b(b[l*W+:W]),.c(c[l*W+:W]),
                .out_valid(vm[l]),.result(y_madd[l*W+:W]),.prod_o(prod[l*W+:W]));
            float_cmp #(EXP_W,MAN_W) u_cmp(
                .clk,.rst,.in_valid(cmp_valid),.op(cmp_op),
                .a(a[l*W+:W]),.b(b[l*W+:W]),
                .out_valid(vc[l]),.result(y_cmp[l*W+:W]));
            // FTOI / ITOF / rounding: three units (a merged unit with shared shifters is larger:
            // 9.4 k vs 8.9 k LUT per FS core)
            // W+1 bits: FTOU's range [0, 2^W - 1] and FTOI's [-2^(W-1), 2^(W-1) - 1] are both clamps of the W+1-bit
            // result (identical to a W-bit FTOI); ITOF / UTOF feed the operand sign- / zero-extended
            float_cvt_f2i #(.EXP_W(EXP_W),.MAN_W(MAN_W),.OW(W+1)) u_f2i(
                .clk,.rst,.in_valid(cvt_valid),.fval(a[l*W+:W]),
                .out_valid(vcv[l]),.ival(f2i_o[l*(W+1)+:W+1]));
            float_cvt_i2f #(.IW(W+1),.EXP_W(EXP_W),.MAN_W(MAN_W)) u_i2f(
                .clk,.rst,.in_valid(cvt_valid),.ival({(cvt_op != 3'd3) & a[l*W+W-1], a[l*W+:W]}),
                .out_valid(),.result(i2f_o[l*W+:W]));
            wire [W:0]   fr  = f2i_o[l*(W+1)+:W+1];
            wire [W-1:0] fti = (~fr[W] & fr[W-1])  ? {1'b0, {(W-1){1'b1}}}        // > 2^(W-1) - 1
                             : ( fr[W] & ~fr[W-1]) ? {1'b1, {(W-1){1'b0}}}        // < -2^(W-1)
                             : fr[W-1:0];
            wire [W-1:0] ftu = fr[W] ? {W{1'b0}} : fr[W-1:0];                     // negative -> 0
            float_round #(.EXP_W(EXP_W),.MAN_W(MAN_W)) u_rnd(
                .clk,.rst,.in_valid(cvt_valid),.mode(cvt_op[1:0]),.x(a[l*W+:W]),
                .out_valid(),.result(rnd_o[l*W+:W]));
            assign y_cvt[l*W+:W] = cvtop_d2[2] ? rnd_o[l*W+:W] : cvtop_d2[0] ? i2f_o[l*W+:W] : cvtop_d2[1] ? ftu : fti;
        end
        // per-group dot (the sum of the group's 4 multiply-add-lane products: same operands, same rounding as a
        // separate float_dot4, no extra multipliers) + shared SFU
        for (g=0; g<N_VEC4; g=g+1) begin : GRP
            float_dot4_sum #(EXP_W,MAN_W) u_dot(
                .clk,.rst,.in_valid(dot_v3[2]),.dp3(dp3_d3[2]),
                .p(prod[g*4*W+:4*W]),
                .out_valid(vd[g]),.result(y_dot[g*W+:W]));
            vec4_sfu #(.EXP_W(EXP_W),.MAN_W(MAN_W),.K(K),.KLOG(KLOG),
                .RCP_MEM(RCP_MEM),.RSQRT_MEM(RSQRT_MEM),.LOG2_MEM(LOG2_MEM),.EXP2_MEM(EXP2_MEM))
              u_sfu(.clk,.rst,.in_valid(sfu_valid),.op(sfu_op),
                    .x(a[g*4*W+:4*W]),.ready(sr[g]),
                    .out_valid(vs[g]),.result(y_sfu[g*4*W+:4*W]));
            vec4_sincos #(.EXP_W(EXP_W),.MAN_W(MAN_W),.KSIN(KSIN),.SIN_MEM(SIN_MEM))
              u_sincos(.clk,.rst,.in_valid(sincos_valid),.op(sincos_op),
                    .x(a[g*4*W+:4*W]),.ready(scr[g]),
                    .out_valid(vsc[g]),.result(y_sincos[g*4*W+:4*W]));
        end
    endgenerate

    int_unit #(.NL(NL), .W(W), .ONE(W'(((1<<(EXP_W-1))-1) << MAN_W))) u_int(.clk, .rst, .a, .b,
        .mul_valid(imul_valid), .v_mul(v_imul), .y_mul(y_imul),
        .div_valid(idiv_valid), .div_mode(idiv_mode), .div_ready(idiv_ready), .v_div(v_idiv), .y_div(y_idiv),
        .iop_valid(iop_valid), .iop_fn(iop_fn), .v_iop(v_iop), .y_iop(y_iop));

    // all lanes/groups share the same valid + latency -> use index 0 as the flag
    assign v_madd    = vm[0];
    assign v_cmp     = vc[0];
    assign v_dot     = vd[0];
    assign v_sfu     = vs[0];
    assign sfu_ready = sr[0];
    assign v_sincos    = vsc[0];
    assign sincos_ready= scr[0];
    assign v_cvt     = vcv[0];
endmodule
`default_nettype wire
