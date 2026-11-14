// float_addsub — parameterized IEEE-style float add/subtract, FTZ, RNE.
// op=0 -> a+b, op=1 -> a-b. Fully-pipelined (II=1). Verified vs tools/float_ref.py.
//
// LATENCY is WIDTH-SELECTED:
//   FP16 (MAN_W<=10): 3 stages (align+add fused, normalize+round fused). The small
//     10-bit mantissa keeps both fused paths short enough to close at 100 MHz, and
//     the FS core (the hazard-stall bottleneck) is FP16 -> madd 6, dp4 9 (~17%).
//   FP24 (MAN_W>=11): 5 stages (original) — its wider mantissa + the high-fanout
//     interp dot4 don't close when fused (measured WNS -1.14), so FP24 stays 5.
// Both structures are BIT-IDENTICAL (same logic, different register count).
`default_nettype none
module float_addsub #(
    parameter int EXP_W = 5,
    parameter int MAN_W = 10,
    // FP24 only (the 5-stage structure): the alignment shift on a DSP48 — the smaller significand times 2^(SIG+3-sh);
    // the product's top WM bits are the shifted value and its low SIG bits the lost ones (the sticky). The product
    // register is the align stage's register. Bit-identical; 1 DSP for the right-shift barrel and the sticky mask.
    parameter bit DSP_ALIGN = 1'b0
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 in_valid,
    input  wire                 op,          // 0=add, 1=sub
    input  wire [EXP_W+MAN_W:0] a,
    input  wire [EXP_W+MAN_W:0] b,
    output wire                 in_ready,
    output reg                  out_valid,
    output reg  [EXP_W+MAN_W:0] result
);
    localparam int W     = 1 + EXP_W + MAN_W;
    localparam int SIG   = MAN_W + 1;
    localparam int BIAS  = (1 << (EXP_W-1)) - 1;
    localparam int INF_E = (1 << EXP_W) - 1;
    localparam int WM    = SIG + 4;                 // carry + SIG + G,R,S
    // >>> ADDSUB LATENCY POLICY (source of truth) <<< FP16 (MAN_W<=10)=3, FP24=5.
    // Consumers matching delay lines replicate this one expression as ADDSUB_LAT; keep in sync.
    localparam int STAGES = (MAN_W <= 10) ? 3 : 5;
    localparam [W-1:0] QNAN = {1'b0, {EXP_W{1'b1}}, 1'b1, {(MAN_W-1){1'b0}}};

    assign in_ready = 1'b1;

    function automatic [MAN_W:0] sig_of(input [W-1:0] x);
        sig_of = (|x[W-2:MAN_W]) ? {1'b1, x[MAN_W-1:0]} : '0;
    endfunction
    function automatic [$clog2(WM):0] clz(input [WM-1:0] v);
        integer i; logic done;
        clz = SIG+3; done = 0;
        for (i = SIG+2; i >= 0; i = i-1)
            if (!done && v[i]) begin clz = (SIG+2) - i; done = 1; end
    endfunction

    // ---- decode a,b ----
    wire             s_a = a[W-1],  s_b = b[W-1] ^ op;
    wire [EXP_W-1:0] e_a = a[W-2:MAN_W], e_b = b[W-2:MAN_W];
    wire             a_zero=(e_a==0), b_zero=(e_b==0);
    wire             a_inf=(e_a==INF_E)&&(a[MAN_W-1:0]==0), b_inf=(e_b==INF_E)&&(b[MAN_W-1:0]==0);
    wire             a_nan=(e_a==INF_E)&&(a[MAN_W-1:0]!=0), b_nan=(e_b==INF_E)&&(b[MAN_W-1:0]!=0);
    wire [W-2:0] mag_a = {e_a, a[MAN_W-1:0]}, mag_b = {e_b, b[MAN_W-1:0]};
    wire a_ge = (mag_a >= mag_b);

    // ===== Stage 0: order, decode classes (common to both structures) =====
    reg                  v0, sbig0, same0;
    reg [EXP_W-1:0]      ebig0, esmall0;
    reg [MAN_W:0]        sbsig0, sssig0;
    reg                  nan0, inf0, infsign0, bothzero0;
    always @(posedge clk) begin
        v0 <= in_valid;
        nan0      <= a_nan | b_nan | (a_inf & b_inf & (s_a!=s_b));
        inf0      <= a_inf | b_inf;
        infsign0  <= a_inf ? s_a : s_b;
        bothzero0 <= a_zero & b_zero;
        same0     <= (s_a == s_b);
        if (a_ge) begin sbig0<=s_a; ebig0<=e_a; sbsig0<=sig_of(a); esmall0<=e_b; sssig0<=sig_of(b); end
        else      begin sbig0<=s_b; ebig0<=e_b; sbsig0<=sig_of(b); esmall0<=e_a; sssig0<=sig_of(a); end
    end

    // combinational align (from S0 regs) — used by both structures
    wire [EXP_W:0] ediff = {1'b0,ebig0} - {1'b0,esmall0};
    wire           far   = ediff > (SIG+3);
    wire [$clog2(WM+1)-1:0] sh = far ? (SIG+3) : ediff[$clog2(WM+1)-1:0];
    wire [WM-1:0]  big_m   = {1'b0, sbsig0, 3'b000};
    wire [WM-1:0]  small_m = {1'b0, sssig0, 3'b000};
    wire [WM-1:0]  small_a = small_m >> sh;
    wire           stick_lost = far ? (|sssig0)
                                    : (| (small_m & ((({{(WM-1){1'b0}},1'b1}) << sh) - 1)));
    wire [WM-1:0]  small_in = small_a | {{(WM-1){1'b0}}, stick_lost};

    generate
    if (STAGES == 3) begin : g3
        // ---- FP16: Stage 1 = align+add fused ----
        reg              v1, sres1, nan1, inf1, infsign1, bothzero1, iszero1;
        reg [EXP_W-1:0]  ebig1;
        reg [WM-1:0]     mant1;
        always @(posedge clk) begin
            v1<=v0; sres1<=sbig0; ebig1<=ebig0; nan1<=nan0; inf1<=inf0; infsign1<=infsign0; bothzero1<=bothzero0;
            mant1   <= same0 ? (big_m + small_in) : (big_m - small_in);
            iszero1 <= (~same0) && (big_m == small_in);
        end
        // ---- Stage 2 = normalize + round + pack fused ----
        wire carry = mant1[WM-1];
        wire [$clog2(WM):0] lz = clz(mant1);
        wire [WM-1:0] norm = carry ? (mant1 >> 1) : (mant1 << lz);
        wire signed [EXP_W+2:0] lz_s=$signed({1'b0,lz}), ebig_s=$signed({2'b0,ebig1});
        wire signed [EXP_W+2:0] exp_n = carry ? (ebig_s+1) : (ebig_s-lz_s);
        wire [MAN_W-1:0] man_n = norm[SIG+1:3];
        wire g=norm[2], r=norm[1], s=norm[0] | (carry & mant1[0]);
        wire roundup = g & (r | s | man_n[0]);
        wire [MAN_W:0] man_r = {1'b0,man_n} + roundup;
        wire carry2 = man_r[MAN_W];
        wire [MAN_W-1:0] man_f = carry2 ? {1'b0,man_r[MAN_W-1:1]} : man_r[MAN_W-1:0];
        wire signed [EXP_W+2:0] exp_f = exp_n + (carry2?1:0);
        always @(posedge clk) begin
            out_valid <= v1 & ~rst;
            if (nan1) result<=QNAN;
            else if (inf1) result<={infsign1,{EXP_W{1'b1}},{MAN_W{1'b0}}};
            else if (iszero1|bothzero1) result<={(W){1'b0}};
            else if (exp_f>=INF_E) result<={sres1,{EXP_W{1'b1}},{MAN_W{1'b0}}};
            else if (exp_f<=0)     result<={sres1,{(W-1){1'b0}}};
            else result<={sres1,exp_f[EXP_W-1:0],man_f};
        end
    end else begin : g5
        // ---- FP24: Stage 1a = align (barrel shift) its own stage ----
        reg              v1a, sbig1a, same1a, nan1a, inf1a, infsign1a, bothzero1a;
        reg [EXP_W-1:0]  ebig1a;
        reg [WM-1:0]     bigm1a;
        wire [WM-1:0]    smallin1a;
        always @(posedge clk) begin
            v1a<=v0; sbig1a<=sbig0; same1a<=same0; ebig1a<=ebig0;
            nan1a<=nan0; inf1a<=inf0; infsign1a<=infsign0; bothzero1a<=bothzero0;
            bigm1a<=big_m;
        end
        if (DSP_ALIGN) begin : g_dsp
            // sssig0 * 2^(SIG+3-sh) = (small_m >> sh) with SIG fraction bits below it (sh = SIG+3 when far: the whole
            // significand is fraction)
            wire [SIG+3:0] oh = (SIG+4)'(1) << ((SIG+3) - sh);
            (* use_dsp = "yes" *) reg [2*SIG+3:0] pal;
            always @(posedge clk) pal <= sssig0 * oh;
            assign smallin1a = pal[SIG +: WM] | {{(WM-1){1'b0}}, |pal[SIG-1:0]};
        end else begin : g_lut
            reg [WM-1:0] sr;
            always @(posedge clk) sr <= small_in;
            assign smallin1a = sr;
        end
        // ---- Stage 1b = add ----
        reg              v1, sres1, nan1, inf1, infsign1, bothzero1, iszero1;
        reg [EXP_W-1:0]  ebig1;
        reg [WM-1:0]     mant1;
        always @(posedge clk) begin
            v1<=v1a; sres1<=sbig1a; ebig1<=ebig1a; nan1<=nan1a; inf1<=inf1a; infsign1<=infsign1a; bothzero1<=bothzero1a;
            mant1   <= same1a ? (bigm1a + smallin1a) : (bigm1a - smallin1a);
            iszero1 <= (~same1a) && (bigm1a == smallin1a);
        end
        // ---- Stage 2a = normalize (CLZ+shift) ----
        wire carry = mant1[WM-1];
        wire [$clog2(WM):0] lz = clz(mant1);
        wire [WM-1:0] norm = carry ? (mant1 >> 1) : (mant1 << lz);
        wire signed [EXP_W+2:0] lz_s=$signed({1'b0,lz}), ebig_s=$signed({2'b0,ebig1});
        wire signed [EXP_W+2:0] exp_n = carry ? (ebig_s+1) : (ebig_s-lz_s);
        reg              v2a, sres2a, nan2a, inf2a, infsign2a, bothzero2a, iszero2a, carry2a, clo2a;
        reg [WM-1:0]     norm2a;
        reg signed [EXP_W+2:0] exp_n2a;
        always @(posedge clk) begin
            v2a<=v1; sres2a<=sres1; nan2a<=nan1; inf2a<=inf1; infsign2a<=infsign1;
            bothzero2a<=bothzero1; iszero2a<=iszero1;
            norm2a<=norm; exp_n2a<=exp_n; carry2a<=carry; clo2a<=mant1[0];
        end
        // ---- Stage 2b = round + pack ----
        wire [MAN_W-1:0] man_n = norm2a[SIG+1:3];
        wire g=norm2a[2], r=norm2a[1], s=norm2a[0] | (carry2a & clo2a);
        wire roundup = g & (r | s | man_n[0]);
        wire [MAN_W:0] man_r = {1'b0,man_n} + roundup;
        wire carry2 = man_r[MAN_W];
        wire [MAN_W-1:0] man_f = carry2 ? {1'b0,man_r[MAN_W-1:1]} : man_r[MAN_W-1:0];
        wire signed [EXP_W+2:0] exp_f = exp_n2a + (carry2?1:0);
        always @(posedge clk) begin
            out_valid <= v2a & ~rst;
            if (nan2a) result<=QNAN;
            else if (inf2a) result<={infsign2a,{EXP_W{1'b1}},{MAN_W{1'b0}}};
            else if (iszero2a|bothzero2a) result<={(W){1'b0}};
            else if (exp_f>=INF_E) result<={sres2a,{EXP_W{1'b1}},{MAN_W{1'b0}}};
            else if (exp_f<=0)     result<={sres2a,{(W-1){1'b0}}};
            else result<={sres2a,exp_f[EXP_W-1:0],man_f};
        end
    end
    endgenerate
endmodule
`default_nettype wire
