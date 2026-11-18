// float_sfu — RCP / RSQRT / LOG2 / EXP2, PIPELINED (latency 10, II=1; the LOG2 / EXP2 lerp subtract sits in
//   the S2 DSP pre-adder; SPLIT1B = 1 and SPLIT6 = 1 add a stage each: latency 12 with both, the setting of the
//   fragment and vertex cores). SIN/COS are the standalone sincos_sfu.
// Each stage is bounded (~one multiply or one normalize) so there is no long
// combinational path -> high Fmax. Accuracy <= 2 ULP. FTZ. Seeds/LUTs from .mem.
// The interface is in_valid/out_valid, so vec4_sfu / alu_cluster / cores are latency-transparent.
//   S1 unpack+lookups(+exp2 float->fixed) | S2 mul1 | S3 post1+mul2
//   S4 mul3(rsqrt)+assemble | S5 normalize-prep(LZC) | S6 round+pack+specials
`default_nettype none
module float_sfu #(
    parameter int EXP_W = 5,
    parameter int MAN_W = 10,
    parameter int K     = 8,
    parameter int KLOG  = 8,
    parameter bit SPLIT1B = 1'b0,                 // 1: every S1 output registered once more before S2 (latency + 1): the seed ROM's
                                                  //    block RAM read straight into the squaring DSP cascade is too slow at 120 MHz
                                                  //    (the vertex core's SFU_SPLIT1B); the setup / clip units keep 0
    parameter bit SPLIT6 = 1'b0,                  // 1: the round add in its own stage (latency 11) — the fragment core;
                                                  //    the vertex core passes SFU_SPLIT6
    parameter     RCP_MEM="", RSQRT_MEM="", LOG2_MEM="", EXP2_MEM=""
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 in_valid,
    input  wire [2:0]           op,             // 0 RCP 1 RSQRT 2 LOG2 3 EXP2
    input  wire [EXP_W+MAN_W:0] x,
    output reg                  out_valid,
    output reg  [EXP_W+MAN_W:0] result
);
    localparam int W=1+EXP_W+MAN_W, SIG=MAN_W+1, BIAS=(1<<(EXP_W-1))-1,
                   INF_E=(1<<EXP_W)-1, SW=MAN_W+2, LFW=24, FW=MAN_W+10, FRAC=MAN_W+8;
    localparam int PW=2*FW+6;
    localparam [W-1:0] QNAN={1'b0,{EXP_W{1'b1}},1'b1,{(MAN_W-1){1'b0}}};
    localparam int EXW = EXP_W+4;                 // signed exponent working width

    (* rom_style="block" *) reg [SW-1:0]  rcp_seed   [0:(1<<K)-1];
    (* rom_style="block" *) reg [SW-1:0]  rsqrt_seed [0:(1<<K)-1];
    (* rom_style="block" *) reg [LFW-1:0] log2_lut   [0:(1<<KLOG)];
    (* rom_style="block" *) reg [LFW-1:0] exp2_lut   [0:(1<<KLOG)];
    initial begin
        if(RCP_MEM!="")  $readmemh(RCP_MEM,rcp_seed);
        if(RSQRT_MEM!="")$readmemh(RSQRT_MEM,rsqrt_seed);
        if(LOG2_MEM!="") $readmemh(LOG2_MEM,log2_lut);
        if(EXP2_MEM!="") $readmemh(EXP2_MEM,exp2_lut);
    end

    // ===== S0: input register =====
    reg v0; reg [2:0] op0; reg [W-1:0] x0;
    always @(posedge clk) begin v0<=in_valid; op0<=op; x0<=x; end

    wire             s_x=x0[W-1];
    wire [EXP_W-1:0] ef=x0[W-2:MAN_W];
    wire [MAN_W-1:0] mf=x0[MAN_W-1:0];
    wire signed [EXW-1:0] e=$signed({{(EXW-EXP_W){1'b0}},ef})-BIAS;
    wire [SIG-1:0]   sig={1'b1,mf};
    wire x_zero=(ef==0),x_inf=(ef==INF_E)&&(mf==0),x_nan=(ef==INF_E)&&(mf!=0),x_neg=s_x;

    // ===== LOG2 near 1.0 (|x - 1| < 2^-NK): log2(1+u) = u * H, H = c1 + u*(-c1/2 + u*(c1/3 - u*c1/4)), c1 = 1/ln2,
    // u = x - 1 exact as U = u * 2^(MAN_W+1). Bit-exact to float_ref.log2_near (arithmetic shifts = floor). The table
    // path's 2^-LFW absolute accuracy is relative-accurate only where |log2 x| is not small; this path covers the rest.
    // One multiply (+ one add) per stage: S1 U | S2 H1 | S2b H2 | S3 H3 | S3b R = U*H3 | S4 joins the LOG2 result.
    // HF = 23 (float_ref LOG2_HF): |H| < 1.45 * 2^23 fits 25 bits signed, so each 13 x 25-bit Horner product is one
    // DSP48 (HF = 32 would need two); accuracy <= 0.55 ULP near 1 (series truncation 2^-22 dominates).
    localparam int NK = 5, HF = 23, UW = MAN_W - 3, HW = 25;    // HW 25: the DSP48 A port
    localparam int NRF = MAN_W + 1 + HF;                         // fraction bits of R
    localparam signed [HW-1:0] NK1 = 25'sh0b8aa3b, NK2 = 25'sh05c551e,   // round(2^HF / ln2 / d), d = 1, 2, 3, 4
                               NK3 = 25'sh03d8e14, NK4 = 25'sh02e2a8f;
    localparam int LTW = (EXW+LFW+3 > UW+HW) ? EXW+LFW+3 : UW+HW;   // width of the LOG2 fixed-point result
    wire [MAN_W:0] n_comp = (1 << MAN_W) - mf;                   // e == -1: |U| = 2^M - mf
    wire n_near = ~s_x & ~x_zero & (ef != INF_E) &
                  (((e == 0) && (mf < (1 << (MAN_W-NK)))) || ((e == -1) && (n_comp < (1 << (MAN_W-NK+1)))));
    wire signed [UW-1:0] n_U = (e == 0) ? $signed({1'b0, mf[UW-3:0], 1'b0}) : -$signed({1'b0, n_comp[UW-2:0]});

    // ===== S1: unpack + lookups + exp2 float->fixed =====
    reg v1; reg [2:0] op1; reg s1,z1,i1,n1,ng1; reg signed [EXW-1:0] e1,er1;
    reg [FW+1:0] r_Mr; reg [FW+2:0] rs_base;
    // ROM->BRAM: register the RAW seed reads (clean `rd <= rom[addr]` = a BRAM output register),
    // then apply the CONSTANT zero-pad + left-shift COMBINATIONALLY (wire-only), so the seed ROMs map to block
    // RAM instead of a fabric LUT-mux tree (an inline pad/shift in the read defeats rom_style="block").
    reg  [SW-1:0]  rcp_rd, rsqrt_rd;              // (r_Yr0 / rs_Ys0, the padded seeds, are formed after the optional S1b stage)
    reg [LFW-1:0] l_la,l_lb; reg [MAN_W-KLOG-1:0] l_lo;
    reg signed [EXW-1:0] x_ipart; reg [LFW-1:0] e_ea,e_eb; reg [FRAC-KLOG-1:0] e_lo;
    // exp2 float->fixed (combinational here)
    wire signed [EXW-1:0] esh=$signed(e)-MAN_W+FRAC;
    wire [SIG+FRAC+EXP_W:0] xext={{(FRAC+EXP_W){1'b0}},sig};
    wire [SIG+FRAC+EXP_W:0] xfix_u=(esh>=0)?(xext<<esh):(xext>>(-esh));
    wire signed [SIG+FRAC+EXP_W:0] xfix=s_x?-$signed(xfix_u):$signed(xfix_u);
    wire [FRAC-1:0] fpart=xfix[FRAC-1:0];
    wire elsb=e[0];
    // EXP2 of an argument beyond the fixed-point conversion: |x| >= 2^(EXW-1), the infinities included. The integer part
    // travels in EXW bits, so such an x would wrap (FP24: 2^-1100 would come out +inf, 2^-2040 256), and an infinity
    // would be shifted out whole and read as 0 (2^-inf = 1.0, so pow(0, y) = exp2(y * log2 0) would be 1). Such an x
    // gives the limit instead: 0 below, +inf above. It rides the `inf` flag, which EXP2 does not use otherwise (a NaN
    // keeps its priority in S6).
    wire x_ebig = ($signed(e) >= EXW-1);
    reg nr1; reg signed [UW-1:0] nU1;
    always @(posedge clk) begin
        nr1<=n_near; nU1<=n_U;
        v1<=v0; op1<=op0; s1<=s_x; z1<=x_zero; i1<=(op0==3'd3) ? x_ebig : x_inf; n1<=x_nan; ng1<=x_neg;
        e1<=e; er1<=$signed(e)>>>1;
        rcp_rd  <= rcp_seed[mf[MAN_W-1-:K]];                          // raw registered ROM read -> BRAM
        r_Mr <={{(FW+2-SIG){1'b0}},sig}<<(FW-MAN_W);
        rsqrt_rd<= rsqrt_seed[{elsb,mf[MAN_W-1-:(K-1)]}];             // raw registered ROM read -> BRAM
        rs_base<= elsb ? ({{(FW+3-SIG){1'b0}},sig}<<(FW-MAN_W+1))
                       : ({{(FW+3-SIG){1'b0}},sig}<<(FW-MAN_W));
        l_la<=log2_lut[mf[MAN_W-1-:KLOG]]; l_lb<=log2_lut[mf[MAN_W-1-:KLOG]+1];
        l_lo<=mf[MAN_W-KLOG-1:0];
        x_ipart<=xfix>>>FRAC;
        e_ea<=exp2_lut[fpart[FRAC-1-:KLOG]]; e_eb<=exp2_lut[fpart[FRAC-1-:KLOG]+1];
        e_lo<=fpart[FRAC-KLOG-1:0];
    end

    // ===== S1b (SPLIT1B only): every S1 output registered once more. Without it the rsqrt seed's block RAM read
    //       (rsqrt_rd) feeds the S2 squaring — two cascaded DSP48s — directly: 5.5 ns of logic to the second DSP's
    //       PCIN, too long at 120 MHz. Latency + 1; the cores are latency-transparent (out_valid), the setup /
    //       clip units keep SPLIT1B = 0 (their delay lines match latency 10). =====
    wire v1s; wire [2:0] op1s; wire s1s,z1s,i1s,n1s,ng1s; wire signed [EXW-1:0] e1s,er1s;
    wire [FW+1:0] r_Mrs; wire [FW+2:0] rs_bases; wire [SW-1:0] rcp_rds, rsqrt_rds;
    wire [LFW-1:0] l_las,l_lbs; wire [MAN_W-KLOG-1:0] l_los; wire signed [EXW-1:0] x_iparts; wire [LFW-1:0] e_eas,e_ebs; wire [FRAC-KLOG-1:0] e_los;
    wire nr1s; wire signed [UW-1:0] nU1s;
    generate if (SPLIT1B) begin : g_s1b
        reg v1r; reg [2:0] op1r; reg s1r,z1r,i1r,n1r,ng1r; reg signed [EXW-1:0] e1r,er1r;
        reg [FW+1:0] r_Mrr; reg [FW+2:0] rs_baser; reg [SW-1:0] rcp_rdr, rsqrt_rdr;
        reg [LFW-1:0] l_lar,l_lbr; reg [MAN_W-KLOG-1:0] l_lor; reg signed [EXW-1:0] x_ipartr; reg [LFW-1:0] e_ear,e_ebr; reg [FRAC-KLOG-1:0] e_lor;
        reg nr1r; reg signed [UW-1:0] nU1r;
        always @(posedge clk) begin
            v1r<=v1; op1r<=op1; s1r<=s1; z1r<=z1; i1r<=i1; n1r<=n1; ng1r<=ng1; e1r<=e1; er1r<=er1;
            r_Mrr<=r_Mr; rs_baser<=rs_base; rcp_rdr<=rcp_rd; rsqrt_rdr<=rsqrt_rd;
            l_lar<=l_la; l_lbr<=l_lb; l_lor<=l_lo; x_ipartr<=x_ipart; e_ear<=e_ea; e_ebr<=e_eb; e_lor<=e_lo;
            nr1r<=nr1; nU1r<=nU1;
        end
        assign v1s=v1r; assign op1s=op1r; assign s1s=s1r; assign z1s=z1r; assign i1s=i1r; assign n1s=n1r; assign ng1s=ng1r; assign e1s=e1r; assign er1s=er1r;
        assign r_Mrs=r_Mrr; assign rs_bases=rs_baser; assign rcp_rds=rcp_rdr; assign rsqrt_rds=rsqrt_rdr;
        assign l_las=l_lar; assign l_lbs=l_lbr; assign l_los=l_lor; assign x_iparts=x_ipartr; assign e_eas=e_ear; assign e_ebs=e_ebr; assign e_los=e_lor;
        assign nr1s=nr1r; assign nU1s=nU1r;
    end else begin : g_s1
        assign v1s=v1; assign op1s=op1; assign s1s=s1; assign z1s=z1; assign i1s=i1; assign n1s=n1; assign ng1s=ng1; assign e1s=e1; assign er1s=er1;
        assign r_Mrs=r_Mr; assign rs_bases=rs_base; assign rcp_rds=rcp_rd; assign rsqrt_rds=rsqrt_rd;
        assign l_las=l_la; assign l_lbs=l_lb; assign l_los=l_lo; assign x_iparts=x_ipart; assign e_eas=e_ea; assign e_ebs=e_eb; assign e_los=e_lo;
        assign nr1s=nr1; assign nU1s=nU1;
    end endgenerate
    // ROM->BRAM: the CONSTANT zero-pad + left-shift applied COMBINATIONALLY to the registered raw seed reads (wire-only)
    wire [FW+1:0]  r_Yr0  = {{(FW+2-SW){1'b0}}, rcp_rds  } << (FW-SW);
    wire [FW+1:0]  rs_Ys0 = {{(FW+2-SW){1'b0}}, rsqrt_rds} << (FW-SW);

    // ===== S2: mul1  (the LOG2/EXP2 lerp subtract (b-a) runs in
    //          the SAME cycle as the lerp multiply via the DSP48 PRE-ADDER — (A-D)*B is one DSP tick — so it
    //          needs no register stage of its own. b>=a for both LUTs (monotonic), so the
    //          subtract is non-negative. =====
    reg v2; reg [2:0] op2; reg s2,z2,i2,n2,ng2; reg signed [EXW-1:0] e2,er2;
    reg [PW-1:0] r_mp, rs_sqp; reg [FW+1:0] r_Yr0_2, rs_Ys0_2; reg [FW+2:0] rs_base2;
    reg [LFW+MAN_W-KLOG:0] l_lerp; reg [LFW-1:0] l_la2; reg signed [EXW-1:0] l_e2;
    reg [LFW+FRAC-KLOG:0] e_lerp; reg [LFW-1:0] e_ea2; reg signed [EXW-1:0] e_ip2;
    // (b-a) truncated to LFW bits (a non-monotonic LUT
    // pair l_lb<l_la must wrap in LFW, not sign-extend, or LOG2 goes far out of tolerance). Combinational
    // here; the subtract sits in the S2 multiply cycle.
    wire [LFW-1:0] l_diff_c = l_lbs - l_las;
    wire [LFW-1:0] e_diff_c = e_ebs - e_eas;
    wire signed [UW+HW-1:0] np1 = nU1s * NK4;
    reg nr2; reg signed [UW-1:0] nU2; reg signed [HW-1:0] nH1;
    always @(posedge clk) begin
        nr2<=nr1s; nU2<=nU1s; nH1 <= NK3 - (np1 >>> (MAN_W+1));
        v2<=v1s; op2<=op1s; s2<=s1s; z2<=z1s; i2<=i1s; n2<=n1s; ng2<=ng1s; e2<=e1s; er2<=er1s;
        r_mp <= r_Mrs * r_Yr0;      r_Yr0_2<=r_Yr0;
        rs_sqp<= rs_Ys0 * rs_Ys0;   rs_Ys0_2<=rs_Ys0; rs_base2<=rs_bases;
        l_lerp<= l_diff_c * l_los;  l_la2<=l_las; l_e2<=e1s;
        e_lerp<= e_diff_c * e_los;  e_ea2<=e_eas; e_ip2<=x_iparts;
    end

    // ===== S2b: register rcp NR subtract r_dr = 2 - Mr*Yr0 (pipelines S3 mul) =====
    reg v2b; reg [2:0] op2b; reg s2b,z2b,i2b,n2b,ng2b; reg signed [EXW-1:0] e2b,er2b;
    reg [FW+1:0] r_Yr0_2b, rs_Ys0_2b; reg [FW+2:0] rs_base2b; reg [PW-1:0] rs_sqp_b;
    reg [FW+2:0] r_dr_b;
    reg [LFW+MAN_W-KLOG:0] l_lerp_b; reg [LFW-1:0] l_la2b; reg signed [EXW-1:0] l_e2b;
    reg [LFW+FRAC-KLOG:0] e_lerp_b; reg [LFW-1:0] e_ea2b; reg signed [EXW-1:0] e_ip2b;
    wire [FW+2:0] r_myv = r_mp>>FW;
    wire [FW+2:0] r_two = {2'b10,{FW{1'b0}}};
    wire signed [UW+HW-1:0] np2 = nU2 * nH1;
    reg nr2b; reg signed [UW-1:0] nU2b; reg signed [HW-1:0] nH2;
    always @(posedge clk) begin
        nr2b<=nr2; nU2b<=nU2; nH2 <= (np2 >>> (MAN_W+1)) - NK2;
        v2b<=v2; op2b<=op2; s2b<=s2; z2b<=z2; i2b<=i2; n2b<=n2; ng2b<=ng2; e2b<=e2; er2b<=er2;
        r_dr_b <= r_two - r_myv;     r_Yr0_2b<=r_Yr0_2;
        rs_sqp_b<=rs_sqp; rs_Ys0_2b<=rs_Ys0_2; rs_base2b<=rs_base2;
        l_lerp_b<=l_lerp; l_la2b<=l_la2; l_e2b<=l_e2;
        e_lerp_b<=e_lerp; e_ea2b<=e_ea2; e_ip2b<=e_ip2;
    end

    // ===== S3: rcp mul2, rsqrt mul2, log2/exp2 assemble =====
    reg v3; reg [2:0] op3; reg s3,z3,i3,n3,ng3; reg signed [EXW-1:0] e3,er3;
    reg [PW-1:0] r_y1p, rs_tp; reg [FW+1:0] rs_Ys0_3;
    reg signed [EXW+LFW+2:0] l_tot; reg [LFW:0] e_frac3; reg signed [EXW-1:0] e_ip3;
    wire [FW+2:0] rs_y0sq = rs_sqp_b>>FW;
    wire signed [UW+HW-1:0] np3 = nU2b * nH2;
    reg nr3; reg signed [UW-1:0] nU3; reg signed [HW-1:0] nH3;
    always @(posedge clk) begin
        nr3<=nr2b; nU3<=nU2b; nH3 <= NK1 + (np3 >>> (MAN_W+1));
        v3<=v2b; op3<=op2b; s3<=s2b; z3<=z2b; i3<=i2b; n3<=n2b; ng3<=ng2b; e3<=e2b; er3<=er2b;
        r_y1p <= r_Yr0_2b * r_dr_b;
        rs_tp <= rs_base2b * rs_y0sq;   rs_Ys0_3<=rs_Ys0_2b;
        l_tot <= ($signed(l_e2b)<<<LFW) + $signed({1'b0,l_la2b + (l_lerp_b>>(MAN_W-KLOG))});
        e_frac3 <= e_ea2b + (e_lerp_b>>(FRAC-KLOG));   e_ip3<=e_ip2b;
    end

    // ===== S3b: register rsqrt NR subtract rs_hval = 1.5 - tval/2 =====
    reg v3b; reg [2:0] op3b; reg s3b,z3b,i3b,n3b,ng3b; reg signed [EXW-1:0] e3b,er3b;
    reg [PW-1:0] r_y1p_b; reg [FW+1:0] rs_Ys0_3b, rs_hval_b;
    reg signed [EXW+LFW+2:0] l_tot_b; reg [LFW:0] e_frac3b; reg signed [EXW-1:0] e_ip3b;
    wire [FW+3:0] rs_tval = rs_tp>>FW;
    wire [FW+3:0] rs_15   = {3'b011,{(FW-1){1'b0}}};
    wire [FW+3:0] rs_hval = rs_15 - (rs_tval>>1);
    wire signed [UW+HW-1:0] np4 = nU3 * nH3;
    reg nr3b; reg signed [UW+HW-1:0] nR3b;
    always @(posedge clk) begin
        nr3b<=nr3; nR3b<=np4;                                    // R = U * H3 = log2(x) * 2^NRF
        v3b<=v3; op3b<=op3; s3b<=s3; z3b<=z3; i3b<=i3; n3b<=n3; ng3b<=ng3; e3b<=e3; er3b<=er3;
        rs_hval_b<=rs_hval[FW+1:0]; r_y1p_b<=r_y1p; rs_Ys0_3b<=rs_Ys0_3;
        l_tot_b<=l_tot; e_frac3b<=e_frac3; e_ip3b<=e_ip3;
    end

    // ===== S4: rcp final shift, rsqrt mul3, assemble =====
    reg v4; reg [2:0] op4; reg s4,z4,i4,n4,ng4; reg signed [EXW-1:0] e4,er4;
    reg [FW+1:0] r_Yr1; reg [PW-1:0] rs_ys1p; reg signed [LTW-1:0] l_tot4; reg nr4;
    reg [LFW:0] e_twof; reg signed [EXW-1:0] e_ip4;
    always @(posedge clk) begin
        v4<=v3b; op4<=op3b; s4<=s3b; z4<=z3b; i4<=i3b; n4<=n3b; ng4<=ng3b; e4<=e3b; er4<=er3b;
        r_Yr1 <= r_y1p_b>>FW;
        rs_ys1p<= rs_Ys0_3b * rs_hval_b;
        l_tot4 <= nr3b ? LTW'(nR3b) : LTW'(l_tot_b);           // near 1: NRF fraction bits, else LFW
        nr4 <= nr3b;
        e_twof<=(1<<LFW)+e_frac3b; e_ip4<=e_ip3b;
    end

    // ===== S4b: log2 abs + MSB-search registered (splits the long norm path) =====
    integer bi;
    reg [LTW-1:0] l_mag_a; reg signed [LTW-1:0] l_msb_a; reg l_sgn_a, l_zero_a;
    always @* begin
        l_zero_a=(l_tot4==0); l_sgn_a=l_tot4[LTW-1]; l_mag_a=l_sgn_a?-l_tot4:l_tot4;
        l_msb_a=0; for(bi=LTW-1;bi>=0;bi=bi-1) if(l_msb_a==0 && l_mag_a[bi]) l_msb_a=bi;
    end
    reg vb; reg [2:0] opb; reg sb,zb,ib,nb,ngb; reg signed [EXW-1:0] eb,erb;
    reg [FW+1:0] r_Yr1_b; reg [PW-1:0] rs_ys1p_b; reg [LFW:0] e_twof_b; reg signed [EXW-1:0] e_ip4_b;
    reg [LTW-1:0] l_mag_b; reg signed [LTW-1:0] l_msb_b; reg l_sgn_b, l_zero_b, nrb;
    always @(posedge clk) begin
        nrb<=nr4;
        vb<=v4; opb<=op4; sb<=s4; zb<=z4; ib<=i4; nb<=n4; ngb<=ng4; eb<=e4; erb<=er4;
        r_Yr1_b<=r_Yr1; rs_ys1p_b<=rs_ys1p; e_twof_b<=e_twof; e_ip4_b<=e_ip4;
        l_mag_b<=l_mag_a; l_msb_b<=l_msb_a; l_sgn_b<=l_sgn_a; l_zero_b<=l_zero_a;
    end

    // ===== S5: normalize-prep (per op -> common {sgn,ex,man,g,stick}) =====
    reg v5; reg [2:0] op5; reg z5,i5,n5,ng5,s5;
    reg sgn5; reg signed [EXW:0] ex5; reg [MAN_W-1:0] man5; reg g5, st5; reg zero5;
    // {guard, sticky, mantissa}: MAN_W + 2 bits (S5 reads guard = bit MAN_W + 1, sticky = bit MAN_W: one bit wider and
    // RCP / RSQRT would truncate instead of rounding to nearest)
    function automatic [MAN_W+1:0] normv(input [FW+1:0] V, input signed [EXW:0] expb);
        logic [FW+1:0] sv; logic signed [EXW:0] ex; logic [MAN_W-1:0] mn; logic gg,ss;
        begin
            if (V[FW]) begin sv=V; ex=expb; end else begin sv=V<<1; ex=expb-1; end
            mn=sv[FW-1-:MAN_W]; gg=sv[FW-1-MAN_W]; ss=|(sv & ((1<<(FW-1-MAN_W))-1));
            normv={gg,ss,mn};
        end
    endfunction
    wire [FW+1:0] rs_Ys1 = rs_ys1p_b>>FW;
    wire [MAN_W+1:0] rcp_n   = normv(r_Yr1_b, -$signed(eb));
    wire signed [EXW:0] rcp_ex = r_Yr1_b[FW] ? -$signed(eb) : (-$signed(eb))-1;
    wire [MAN_W+1:0] rsq_n   = normv(rs_Ys1, -$signed(erb));
    wire signed [EXW:0] rsq_ex= rs_Ys1[FW] ? -$signed(erb) : (-$signed(erb))-1;
    // log2 part B: variable shift (from registered abs + msb). Use a PLAIN integer shift amount (gsh)
    // rather than a wide signed part-select `l_mag_b[l_msb_b-MAN_W-1]`: the wide signed index made xsim
    // return X for the guard bit under the core's bursty SFU driving (guard-bit X -> QNAN/X output).
    reg [MAN_W-1:0] l_man; reg l_g,l_s; reg signed [EXW:0] l_ex;
    integer gsh;
    localparam [LTW-1:0] LONE = 1;
    always @* begin
        l_ex = nrb ? (l_msb_b - NRF) : (l_msb_b - LFW);
        gsh=l_msb_b-MAN_W;
        if (l_msb_b>=MAN_W) begin
            l_man=l_mag_b>>gsh;
            l_g=(gsh>=1)?((l_mag_b>>(gsh-1))&1'b1):1'b0;
            l_s=(gsh>=2)?|(l_mag_b & ((LONE<<(gsh-1))-LONE)):1'b0;
        end else begin l_man=l_mag_b<<(MAN_W-l_msb_b); l_g=0; l_s=0; end
    end
    wire [MAN_W-1:0] e_man5 = e_twof_b[LFW-1-:MAN_W];
    wire e_g5=e_twof_b[LFW-1-MAN_W];
    wire e_s5=|(e_twof_b & ((1<<(LFW-1-MAN_W))-1));
    always @(posedge clk) begin
        v5<=vb; op5<=opb; z5<=zb; i5<=ib; n5<=nb; ng5<=ngb; s5<=sb;
        case (opb)
          2'd0: begin sgn5<=sb; ex5<=rcp_ex; man5<=rcp_n[MAN_W-1:0]; g5<=rcp_n[MAN_W+1]; st5<=rcp_n[MAN_W]; zero5<=1'b0; end
          2'd1: begin sgn5<=1'b0; ex5<=rsq_ex; man5<=rsq_n[MAN_W-1:0]; g5<=rsq_n[MAN_W+1]; st5<=rsq_n[MAN_W]; zero5<=1'b0; end
          2'd2: begin sgn5<=l_sgn_b; ex5<=l_ex; man5<=l_man; g5<=l_g; st5<=l_s; zero5<=l_zero_b; end
          default: begin sgn5<=1'b0; ex5<=$signed(e_ip4_b); man5<=e_man5; g5<=e_g5; st5<=e_s5; zero5<=1'b0; end
        endcase
    end

    // ===== S6a (SPLIT6 only): the round add registered. The cores set SPLIT6 = 1 (latency 11; the cores are
    // latency-transparent): otherwise round + pack + specials are one 17-level stage, too slow in the cores at 110 MHz
    // and above. The setup / clip units keep SPLIT6 = 0 (latency
    // 10, their delay lines match it). =====
    wire [MAN_W:0] mr5=({1'b0,man5}+ (g5 & (st5|man5[0])));
    wire v6; wire [2:0] op6; wire z6,i6,n6,ng6,s6,sgn6,zero6; wire [MAN_W:0] mr; wire signed [EXW:0] ex6;
    // the round's carry out (the mantissa rounds up to 2.0): all ones and a round up — known without the add, so the
    // exponent's part of S6 does not wait for the mantissa's carry chain
    wire carry;
    generate if (SPLIT6) begin : g_s6a
        reg v6r; reg [2:0] op6r; reg z6r,i6r,n6r,ng6r,s6r,sgn6r,zero6r; reg [MAN_W:0] mr6r; reg signed [EXW:0] ex6r;
        always @(posedge clk) begin
            v6r<=v5 & ~rst; op6r<=op5; z6r<=z5; i6r<=i5; n6r<=n5; ng6r<=ng5; s6r<=s5; sgn6r<=sgn5; zero6r<=zero5; mr6r<=mr5; ex6r<=ex5;
        end
        assign v6=v6r; assign op6=op6r; assign z6=z6r; assign i6=i6r; assign n6=n6r; assign ng6=ng6r; assign s6=s6r;
        assign sgn6=sgn6r; assign zero6=zero6r; assign mr=mr6r; assign ex6=ex6r;
        assign carry=mr6r[MAN_W];
    end else begin : g_s6
        assign v6=v5; assign op6=op5; assign z6=z5; assign i6=i5; assign n6=n5; assign ng6=ng5; assign s6=s5;
        assign sgn6=sgn5; assign zero6=zero5; assign mr=mr5; assign ex6=ex5;
        assign carry=g5 & (&man5);
    end endgenerate
    // ===== S6: pack + specials. The biased exponent, its overflow and its underflow are worked out for both values of
    // the carry beside the round add, and the carry selects: one short chain each instead of the round add, the
    // exponent's increment and its two comparisons one after the other. =====
    wire [MAN_W-1:0] man_f=mr[MAN_W-1:0];
    wire signed [EXW+2:0] exb0=ex6+BIAS, exb1=ex6+BIAS+1;       // ex_f + BIAS without / with the carry
    wire ovf=carry ? (exb1>=INF_E) : (exb0>=INF_E);
    wire unf=carry ? (exb1<=0) : (exb0<=0);
    wire [EXP_W-1:0] exp_f=carry ? exb1[EXP_W-1:0] : exb0[EXP_W-1:0];
    reg [W-1:0] normres;
    always @* begin
        if (zero6)                       normres={sgn6,{(W-1){1'b0}}};
        else if (ovf)                    normres={sgn6,{EXP_W{1'b1}},{MAN_W{1'b0}}};
        else if (unf)                    normres={sgn6,{(W-1){1'b0}}};
        else                             normres={sgn6,exp_f,man_f};
    end
    always @(posedge clk) begin
        out_valid <= v6 & ~rst;
        case (op6)
          3'd0: result <= n6 ? QNAN : z6 ? {s6,{EXP_W{1'b1}},{MAN_W{1'b0}}} : i6 ? {s6,{(W-1){1'b0}}} : normres;
          3'd1: result <= (ng6&&!z6) ? QNAN : z6 ? {1'b0,{EXP_W{1'b1}},{MAN_W{1'b0}}} : i6 ? {(W){1'b0}} : normres;
          3'd2: result <= (ng6|n6) ? QNAN : z6 ? {1'b1,{EXP_W{1'b1}},{MAN_W{1'b0}}} : i6 ? {1'b0,{EXP_W{1'b1}},{MAN_W{1'b0}}} : normres;
          default: result <= n6 ? QNAN : z6 ? {1'b0,BIAS[EXP_W-1:0],{MAN_W{1'b0}}}             // 3'd3 EXP2
                              : i6 ? (s6 ? {(W){1'b0}} : {1'b0,{EXP_W{1'b1}},{MAN_W{1'b0}}}) : normres;   // |x| >= 2^(EXW-1): 0 / +inf
        endcase
    end
endmodule
`default_nettype wire
