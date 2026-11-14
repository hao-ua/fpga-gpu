// sincos_sfu — SIN / COS at the format's own precision (FP24: <= 0.54 ULP measured, tools/float_ref.py), II = 1,
// latency 12. Bit-exact to float_ref.sfu_sin:
//   turns = |x| / 2pi in fixed point (sig * round(2^CFRAC / 2pi)), PF = 2+KSIN+IFB fraction bits kept (mod 1):
//   top 2 = quadrant, next KSIN = table index, low IFB = interpolation fraction. cos = phase + 1/4 turn. Odd
//   quadrants mirror the in-quarter phase (exact negate; zero there is the peak, 1.0). The quarter-wave table holds
//   sin((pi/2) i / N) in TFB-bit fixed point at the grid points (lut[N] = 1.0 implied); the value is
//   lut[i] + round((lut[i+1] - lut[i]) * frac / 2^IFB), rounded once (RNE) to the format. sin is odd (input sign),
//   cos even. Small angles: sin(x) = x for e <= SMALL_E (|x| < 2^-8 at FP24: x^2/6 below half an ULP).
//   op: 0 = SIN, 1 = COS.  Specials: +-0 -> sin 0 / cos 1; Inf / NaN -> QNaN; denormal inputs are zero (FTZ).
// Stages (each one multiply, one shift or one carry chain):
//   P1 unpack, specials, small flag | P2 sig * (1/2pi) [DSP] | P3 DSP out | P4 range-reduce shift -> phase |
//   P5 cos quarter, quadrant fold -> index / fraction | P6 table read, ports i and i+1 | P7 difference |
//   P8a difference * fraction as two DSP products (18 + 2 fraction bits) | P8 their sum + 1/2 | P9 value V |
//   P10 normalise | P11 round, pack, specials (registered)
`default_nettype none
module sincos_sfu #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16,
    parameter int KSIN  = 10,                   // table index bits (table depth N = 2^KSIN)
    parameter     SIN_MEM=""
)(
    input  wire                 clk, rst,
    input  wire                 in_valid,
    input  wire                 op,             // 0 = SIN, 1 = COS
    input  wire [EXP_W+MAN_W:0] x,
    output reg                  out_valid,
    output reg  [EXP_W+MAN_W:0] result
);
    localparam int W=1+EXP_W+MAN_W, SIG=MAN_W+1, BIAS=(1<<(EXP_W-1))-1, INF_E=(1<<EXP_W)-1;
    localparam int EXW = EXP_W+4;
    localparam int IFB = 20, TFB = 30;          // interpolation fraction bits, table fraction bits (float_ref SIN_*)
    localparam int QB  = KSIN + IFB;            // in-quarter phase bits
    localparam int PF  = 2 + QB;                // phase bits kept
    localparam int DW  = TFB - KSIN + 1;        // lut[i+1] - lut[i] < 2^DW (step (pi/2)/N * 2^TFB)
    localparam int VW  = TFB + 1;               // value V in [0, 2^TFB]
    localparam int CFRAC = (MAN_W<=10) ? 40 : 48;
    localparam [47:0] SIN_C = (MAN_W<=10) ? 48'h0028be60db94   // round(2^40 / 2pi)  (FP16)
                                          : 48'h28be60db9391;  // round(2^48 / 2pi)  (FP24)
    localparam int PRW = SIG + 48;
    localparam int SINCONST = MAN_W + CFRAC - PF;              // RS = SINCONST - e
    localparam int SMALL_E  = -((MAN_W + 2) / 2);
    localparam [W-1:0] QNAN   = {1'b0,{EXP_W{1'b1}},1'b1,{(MAN_W-1){1'b0}}};
    localparam [W-1:0] FP_ONE = {1'b0, BIAS[EXP_W-1:0], {MAN_W{1'b0}}};
    localparam [VW-1:0] V_ONE = {1'b1, {TFB{1'b0}}};

    (* rom_style="block" *) reg [TFB-1:0] sin_lut [0:(1<<KSIN)-1];
    initial if (SIN_MEM!="") $readmemh(SIN_MEM, sin_lut);

    // ---- P1: unpack |x|, specials, small-angle flag (DSP input register) ----
    wire             s_x=x[W-1];
    wire [EXP_W-1:0] ef=x[W-2:MAN_W];
    wire [MAN_W-1:0] mf=x[MAN_W-1:0];
    wire signed [EXW-1:0] e=$signed({{(EXW-EXP_W){1'b0}},ef})-BIAS;
    reg v1; reg [SIG-1:0] sig1; reg signed [EXW-1:0] e1; reg s1, cos1, z1, bad1, sm1; reg [W-1:0] x1;
    always @(posedge clk) begin
        v1<=in_valid & ~rst; sig1<={1'b1,mf}; e1<=e; s1<=s_x; cos1<=op; x1<=x;
        z1<=(ef==0); bad1<=(ef==INF_E); sm1<=(e <= SMALL_E) & ~op;      // sin only
    end
    // ---- P2: sig * (1/2pi) [DSP] ----
    reg v2; (* use_dsp="yes" *) reg [PRW-1:0] Pr2; reg signed [EXW-1:0] e2; reg s2, cos2, z2, bad2, sm2; reg [W-1:0] x2;
    always @(posedge clk) begin
        v2<=v1; Pr2<=sig1*SIN_C; e2<=e1; s2<=s1; cos2<=cos1; z2<=z1; bad2<=bad1; sm2<=sm1; x2<=x1;
    end
    // ---- P3: DSP output register ----
    reg v3; reg [PRW-1:0] Pr3; reg signed [EXW-1:0] e3; reg s3, cos3, z3, bad3, sm3; reg [W-1:0] x3;
    always @(posedge clk) begin
        v3<=v2; Pr3<=Pr2; e3<=e2; s3<=s2; cos3<=cos2; z3<=z2; bad3<=bad2; sm3<=sm2; x3<=x2;
    end
    // ---- P4: range reduction: PF fraction bits of turns (mod 1) ----
    wire signed [EXW+3:0] RS    = SINCONST - e3;
    wire [15:0]           rsabs = RS[EXW+3] ? (-RS) : RS;
    wire [PRW-1:0]        T     = RS[EXW+3] ? (Pr3 << rsabs) : (Pr3 >> rsabs);
    reg v4; reg [PF-1:0] ph4; reg s4, cos4, z4, bad4, sm4; reg [W-1:0] x4;
    always @(posedge clk) begin
        v4<=v3; ph4<=T[PF-1:0]; s4<=s3; cos4<=cos3; z4<=z3; bad4<=bad3; sm4<=sm3; x4<=x3;
    end
    // ---- P5: cos = +1/4 turn (quadrant + 1), fold odd quadrants (exact mirror), split index / fraction ----
    wire [1:0]    Q   = ph4[PF-1:QB] + {1'b0, cos4};
    wire [QB-1:0] p   = ph4[QB-1:0];
    wire [QB:0]   pm  = Q[0] ? ({1'b1, {QB{1'b0}}} - {1'b0, p}) : {1'b0, p};   // pm == 2^QB: the peak
    wire          sgn = cos4 ? Q[1] : (Q[1] ^ s4);                             // negate upper half; sin is odd
    reg v5; reg [KSIN-1:0] idx5; reg [IFB-1:0] fr5; reg peak5, last5, sign5, cos5, z5, bad5, sm5; reg [W-1:0] x5;
    always @(posedge clk) begin
        v5<=v4; idx5<=pm[QB-1:IFB]; fr5<=pm[IFB-1:0]; peak5<=pm[QB]; last5<=(pm[QB-1:IFB]=={KSIN{1'b1}});
        sign5<=sgn; cos5<=cos4; z5<=z4; bad5<=bad4; sm5<=sm4; x5<=x4;
    end
    // ---- P6: table read at i and i+1 (two read ports of one block RAM) ----
    wire [KSIN-1:0] idx5n = idx5 + 1'b1;
    reg v6; reg [TFB-1:0] La6, Lb6; reg [IFB-1:0] fr6; reg peak6, last6, sign6, cos6, z6, bad6, sm6; reg [W-1:0] x6;
    always @(posedge clk) begin
        v6<=v5; La6<=sin_lut[idx5]; Lb6<=sin_lut[idx5n]; fr6<=fr5; peak6<=peak5; last6<=last5;
        sign6<=sign5; cos6<=cos5; z6<=z5; bad6<=bad5; sm6<=sm5; x6<=x5;
    end
    // ---- P7: difference to the next grid point (lut[N] = 1.0) ----
    wire [VW-1:0] Lb = last6 ? V_ONE : {1'b0, Lb6};
    wire [VW-1:0] Dw = Lb - {1'b0, La6};
    reg v7; reg [DW-1:0] D7; reg [TFB-1:0] La7; reg [IFB-1:0] fr7; reg peak7, sign7, cos7, z7, bad7, sm7; reg [W-1:0] x7;
    always @(posedge clk) begin
        v7<=v6; D7<=Dw[DW-1:0]; La7<=La6; fr7<=fr6; peak7<=peak6;
        sign7<=sign6; cos7<=cos6; z7<=z6; bad7<=bad6; sm7<=sm6; x7<=x6;
    end
    // ---- P8a: difference * fraction as two DSP-sized products (the fraction's 20 bits exceed the DSP's 18: as one
    //      multiply, synthesis splits it over two DSPs with a 7-CARRY4 fabric adder between them, too slow at 120 MHz) ----
    reg v8a; (* use_dsp="yes" *) reg [DW+18-1:0] Pl8; (* use_dsp="yes" *) reg [DW+2-1:0] Ph8;
    reg [TFB-1:0] La8a; reg peak8a, sign8a, cos8a, z8a, bad8a, sm8a; reg [W-1:0] x8a;
    always @(posedge clk) begin
        v8a<=v7; Pl8<=D7*fr7[17:0]; Ph8<=D7*fr7[IFB-1:18]; La8a<=La7; peak8a<=peak7;
        sign8a<=sign7; cos8a<=cos7; z8a<=z7; bad8a<=bad7; sm8a<=sm7; x8a<=x7;
    end
    // ---- P8: the product + 1/2 LSB ----
    reg v8; reg [DW+IFB-1:0] P8; reg [TFB-1:0] La8; reg peak8, sign8, cos8, z8, bad8, sm8; reg [W-1:0] x8;
    always @(posedge clk) begin
        v8<=v8a; P8<={Ph8, 18'd0} + {{(IFB-18){1'b0}}, Pl8} + (1<<(IFB-1)); La8<=La8a; peak8<=peak8a;
        sign8<=sign8a; cos8<=cos8a; z8<=z8a; bad8<=bad8a; sm8<=sm8a; x8<=x8a;
    end
    // ---- P9: value V = lut[i] + round(D * frac / 2^IFB) ----
    reg v9; reg [VW-1:0] V9; reg sign9, cos9, z9, bad9, sm9; reg [W-1:0] x9;
    always @(posedge clk) begin
        v9<=v8; V9 <= peak8 ? V_ONE : ({1'b0, La8} + P8[DW+IFB-1:IFB]);
        sign9<=sign8; cos9<=cos8; z9<=z8; bad9<=bad8; sm9<=sm8; x9<=x8;
    end
    // ---- P10: normalise (leading one to bit TFB); exponent = -leading zeros ----
    reg [4:0] lz; integer k;
    always @* begin
        lz = 5'd0;
        for (k = 0; k < VW; k = k + 1) if (V9[k]) lz = VW - 1 - k;
    end
    reg v10; reg [VW-1:0] Vn10; reg [4:0] lz10; reg vz10, sign10, cos10, z10, bad10, sm10; reg [W-1:0] x10;
    always @(posedge clk) begin
        v10<=v9; Vn10<=V9 << lz; lz10<=lz; vz10<=(V9=='0);
        sign10<=sign9; cos10<=cos9; z10<=z9; bad10<=bad9; sm10<=sm9; x10<=x9;
    end
    // ---- P11: round to MAN_W+1 bits (RNE), pack, specials ----
    wire [MAN_W:0]   top  = Vn10[TFB -: MAN_W+1];
    wire             g    = Vn10[TFB-MAN_W-1];
    wire             st   = |Vn10[TFB-MAN_W-2:0];
    wire [MAN_W+1:0] q    = {1'b0, top} + (g & (st | top[0]));
    wire             cy   = q[MAN_W+1];
    wire signed [EXW:0] be = BIAS - $signed({1'b0, lz10}) + cy;
    wire [MAN_W-1:0] man  = cy ? {MAN_W{1'b0}} : q[MAN_W-1:0];
    always @(posedge clk) begin
        out_valid <= v10 & ~rst;
        result    <= bad10 ? QNAN
                   : z10   ? (cos10 ? FP_ONE : {W{1'b0}})
                   : sm10  ? x10
                   : (vz10 || be <= 0) ? {W{1'b0}}
                   : {sign10, be[EXP_W-1:0], man};
    end
endmodule
`default_nettype wire
