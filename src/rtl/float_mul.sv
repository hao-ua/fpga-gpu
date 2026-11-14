// float_mul — parameterized IEEE-style float multiply, FTZ, round-to-nearest-even.
// Fixed-latency (3) fully-pipelined: accepts a new pair every cycle (II=1).
// in_ready is always 1; backpressure is handled at module boundaries by skid
// buffers / the core ROB. Instantiate as FP16 (5,10) or
// FP24 (7,16). Verified against tools/float_ref.py golden vectors.
`default_nettype none
module float_mul #(
    parameter int EXP_W = 5,
    parameter int MAN_W = 10
)(
    input  wire                       clk,
    input  wire                       rst,
    input  wire                       in_valid,
    input  wire [EXP_W+MAN_W:0]       a,
    input  wire [EXP_W+MAN_W:0]       b,
    output wire                       in_ready,   // always 1
    output reg                        out_valid,
    output reg  [EXP_W+MAN_W:0]       result
);
    localparam int W     = 1 + EXP_W + MAN_W;
    localparam int SIG   = MAN_W + 1;                 // significand bits (1.M)
    localparam int BIAS  = (1 << (EXP_W-1)) - 1;
    localparam int INF_E = (1 << EXP_W) - 1;
    localparam [W-1:0] QNAN = {1'b0, {EXP_W{1'b1}}, 1'b1, {(MAN_W-1){1'b0}}};

    assign in_ready = 1'b1;

    // ---- unpack ----
    function automatic [MAN_W:0] sig_of(input [W-1:0] x);
        sig_of = (|x[W-2:MAN_W]) ? {1'b1, x[MAN_W-1:0]} : '0;  // FTZ: exp==0 -> 0
    endfunction

    wire             s_a = a[W-1],           s_b = b[W-1];
    wire [EXP_W-1:0] e_a = a[W-2:MAN_W],      e_b = b[W-2:MAN_W];
    wire             a_zero = (e_a == 0),     b_zero = (e_b == 0);
    wire             a_inf  = (e_a==INF_E) && (a[MAN_W-1:0]==0);
    wire             b_inf  = (e_b==INF_E) && (b[MAN_W-1:0]==0);
    wire             a_nan  = (e_a==INF_E) && (a[MAN_W-1:0]!=0);
    wire             b_nan  = (e_b==INF_E) && (b[MAN_W-1:0]!=0);

    // ===== Stage 0: register operands + decoded class =====
    reg                  v0;
    reg                  s0;
    reg signed [EXP_W+1:0] esum0;            // e_a + e_b - bias (signed, extra bits)
    reg [MAN_W:0]        siga0, sigb0;
    reg                  res_nan0, res_inf0, res_zero0;
    always @(posedge clk) begin
        v0       <= in_valid;
        s0       <= s_a ^ s_b;
        esum0    <= $signed({2'b0, e_a}) + $signed({2'b0, e_b}) - BIAS;
        siga0    <= sig_of(a);
        sigb0    <= sig_of(b);
        res_nan0 <= a_nan | b_nan | (a_inf & b_zero) | (b_inf & a_zero);
        res_inf0 <= (a_inf | b_inf);
        res_zero0<= (a_zero | b_zero);
    end

    // ===== Stage 1: significand multiply =====
    reg                   v1, s1;
    reg signed [EXP_W+1:0] esum1;
    reg [2*SIG-1:0]       prod1;
    reg                   res_nan1, res_inf1, res_zero1;
    always @(posedge clk) begin
        v1        <= v0;
        s1        <= s0;
        esum1     <= esum0;
        prod1     <= siga0 * sigb0;          // [2^(2M), 2^(2M+2)) for normals
        res_nan1  <= res_nan0;
        res_inf1  <= res_inf0;
        res_zero1 <= res_zero0;
    end

    // ===== Stage 2: normalize + round + assemble =====
    // prod in [1,4) * 2^(2M). Leading 1 at bit 2M+1 (>=2.0) or 2M ([1,2)).
    wire hi = prod1[2*SIG-1];                 // bit 2M+1 set => product >= 2.0
    // significand (SIG bits incl leading 1) and guard/sticky for each case
    wire [SIG-1:0] mant_hi = prod1[2*SIG-1 : SIG];      // [2M+1 : M+1]
    wire           g_hi    = prod1[SIG-1];             // bit M
    wire           st_hi   = |prod1[SIG-2:0];          // bits [M-1:0]
    wire [SIG-1:0] mant_lo = prod1[2*SIG-2 : SIG-1];    // [2M : M]
    wire           g_lo    = prod1[SIG-2];             // bit M-1
    wire           st_lo   = |prod1[SIG-3:0];          // bits [M-2:0]

    wire [SIG-1:0] mant_n  = hi ? mant_hi : mant_lo;
    wire           guard   = hi ? g_hi    : g_lo;
    wire           sticky  = hi ? st_hi   : st_lo;
    wire signed [EXP_W+1:0] exp_n = esum1 + (hi ? 1 : 0);   // stored-exp (biased)

    wire roundup = guard & (sticky | mant_n[0]);
    wire [SIG:0] mant_r = mant_n + roundup;            // may carry into bit SIG
    wire         carry  = mant_r[SIG];
    wire [MAN_W-1:0] man_final = carry ? mant_r[SIG:1] : mant_r[MAN_W-1:0];
    wire signed [EXP_W+1:0] exp_final = exp_n + (carry ? 1 : 0);

    always @(posedge clk) begin
        out_valid <= v1;
        if (rst) out_valid <= 1'b0;
        if (res_nan1)        result <= QNAN;
        else if (res_inf1)   result <= {s1, {EXP_W{1'b1}}, {MAN_W{1'b0}}};
        else if (res_zero1)  result <= {s1, {(W-1){1'b0}}};
        else if (exp_final >= INF_E)  result <= {s1, {EXP_W{1'b1}}, {MAN_W{1'b0}}}; // overflow->Inf
        else if (exp_final <= 0)      result <= {s1, {(W-1){1'b0}}};               // underflow->FTZ
        else result <= {s1, exp_final[EXP_W-1:0], man_final};
    end
endmodule
`default_nettype wire
