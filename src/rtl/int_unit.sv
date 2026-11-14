// int_unit — integer multiply / divide of the integer ALU, separate hardware from the FP ALU,
// NL lanes of W-bit two's-complement integers held in the shader lanes (W = 24 at FP24).
//   IMUL: low W bits of a*b (wraps modulo 2^W; GLSL ES leaves integer overflow undefined). Pipelined, latency 3,
//         II 1: in register | multiply | out register.
//   IDIV: signed quotient truncated toward zero. Defined for every input (RISC-V rules): x / 0 = -1 (all ones),
//         -2^(W-1) / -1 = -2^(W-1). Restoring division, one quotient bit per cycle in all lanes at once: one
//         division in flight, div_ready low while busy (W + 2 cycles), v_div pulses with the result.
// (IADD / ISUB / ISLT / ISGE / ISEQ / ISNE stay in the cores' 1-cycle integer slot, used by loop counters.)
`default_nettype none
//   IOP (fn, latency 2, II 1): 0 AND, 1 OR, 2 XOR, 3 NOT a, 4 SHL a << b[4:0], 5 SHR (arithmetic), 6 USHR (logical),
//         7 IMIN, 8 IMAX, 9 UMIN, 10 UMAX, 11 IABS, 12 INEG, 13 ISIGN (-1 / 0 / 1), 14 ULT, 15 UGE (unsigned compares
//         -> ONE (the format's 1.0) / 0, like ISLT / ISGE). Shift counts b[4:0] >= W give 0 (sign fill for SHR).
//   The divider's div_mode: 0 IDIV, 1 IMOD (remainder, sign of the dividend), 2 UDIV, 3 UMOD; x % 0 = x and
//         unsigned x / 0 = all ones fall out of the restoring division (divisor 0: every step subtracts 0).
//   DSEQ > 1 (the fragment core's 4-lane instance): a division job is DSEQ x NL lanes delivered as DSEQ
//         consecutive div_valid cycles (NL lanes each, div_mode with the first); the unit buffers them, runs DSEQ
//         rounds of the restoring division, and returns the results as DSEQ consecutive v_div cycles in the same
//         order. div_ready is low from the first cycle to the last result (≈ DSEQ x (W + 2) + DSEQ + 2 cycles).
module int_unit #(
    parameter int NL = 16,
    parameter int W  = 24,
    parameter [W-1:0] ONE = '0,                       // the lane format's 1.0 (compare results)
    parameter int DSEQ = 1                            // division rounds per job (jobs of DSEQ x NL lanes)
)(
    input  wire              clk, rst,
    input  wire [NL*W-1:0]   a, b,
    input  wire              mul_valid,
    output reg               v_mul,
    output reg  [NL*W-1:0]   y_mul,
    input  wire              div_valid,
    input  wire [1:0]        div_mode,
    output wire              div_ready,
    output wire              v_div,
    output wire [NL*W-1:0]   y_div,
    input  wire              iop_valid,
    input  wire [3:0]        iop_fn,
    output reg               v_iop,
    output reg  [NL*W-1:0]   y_iop
);
    localparam int CW = $clog2(W + 1);
    if (DSEQ < 1) $error("int_unit: DSEQ >= 1");

    // ---- IOP: logic, shifts, min / max, abs / neg / sign, unsigned compares (in register | compute) ----
    // Per lane: one right shifter (SHL = bit-reversed input and output; SHR fills with the sign; its "all shifted out"
    // case also gives the ISIGN / ULT / UGE constants), one subtractor shared by the compares (x - y, signed compares
    // with the MSBs flipped) and the negation (0 - x), a term t over (x, y, -x) for the logic ops, MIN / MAX, ABS and
    // NEG, and a 3-way result mux {t, shr, reversed shr}. The function decode is common to all lanes (registered with
    // the operands). OOC, 16 lanes x 24 bits: IOP 2 818 LUT, divider 1 538, IMUL 370 + 16 DSP; whole unit 4 734.
    reg [NL*W-1:0] ia1, ib1; reg vi1;
    reg f_shl, f_sra, f_sh, f_cmp, f_neg, f_abs, f_mm, f_min, f_sgn, f_isign, f_uge, f_log, f_k, f_cmpk; reg [1:0] f_lg;
    always @(posedge clk) begin
        ia1 <= a; ib1 <= b; vi1 <= iop_valid & ~rst; v_iop <= vi1 & ~rst;
        f_k   <= iop_fn >= 4'd13;                                // ISIGN ULT UGE: constants via the shifter
        f_cmpk <= iop_fn >= 4'd14;
        f_shl <= iop_fn == 4'd4; f_sra <= iop_fn == 4'd5; f_sh <= iop_fn >= 4'd4 && iop_fn <= 4'd6;
        f_mm  <= iop_fn >= 4'd7 && iop_fn <= 4'd10;              // IMIN IMAX UMIN UMAX
        f_min <= iop_fn == 4'd7 || iop_fn == 4'd9;
        f_sgn <= iop_fn == 4'd7 || iop_fn == 4'd8;               // signed compare
        f_cmp <= (iop_fn >= 4'd7 && iop_fn <= 4'd10) || iop_fn >= 4'd14;   // subtractor as x - y (else 0 - x)
        f_abs <= iop_fn == 4'd11; f_neg <= iop_fn == 4'd12; f_isign <= iop_fn == 4'd13;
        f_uge <= iop_fn == 4'd15; f_log <= iop_fn <= 4'd3; f_lg <= iop_fn[1:0];
    end
    function automatic [W-1:0] rev(input [W-1:0] v);
        for (int k = 0; k < W; k++) rev[k] = v[W-1-k];
    endfunction
    genvar li;
    generate for (li = 0; li < NL; li = li + 1) begin : IOPL
        wire [W-1:0] x = ia1[li*W +: W], y = ib1[li*W +: W];
        wire [4:0]   n    = y[4:0];
        // compares: x - y (MSBs flipped for signed), borrow = x < y; otherwise 0 - x
        wire [W-1:0] sa   = f_cmp ? {x[W-1] ^ f_sgn, x[W-2:0]} : '0;
        wire [W-1:0] sb   = f_cmp ? {y[W-1] ^ f_sgn, y[W-2:0]} : x;
        wire [W:0]   d    = {1'b0, sa} - {1'b0, sb};
        wire         lt   = d[W];
        wire         nz   = |x;
        // right shift in three stages of W bits (fill enters at the top): by n[1:0], by 4 n[3:2], by 16 n[4]. The last
        // stage's "everything shifted out" case also yields the constants: ISIGN = the sign in every bit but bit 0
        // (x != 0); ULT / UGE = ONE's bits where the compare holds.
        wire [W-1:0] sin  = f_shl ? rev(x) : x;
        wire         fill = f_sra & x[W-1];
        wire         ge   = (n >= W) | f_k;
        wire         kf   = f_isign ? x[W-1] : f_cmpk ? (lt ^ f_uge) : fill;
        wire         kf0  = f_isign ? nz : f_cmpk ? (ONE[0] & kf) : fill;
        wire [W+2:0]  e1 = {{3{fill}}, sin};
        wire [W+11:0] e2;
        wire [W+15:0] e3;
        wire [W-1:0]  shr;
        assign e2[W+11:W] = {12{fill}};
        assign e3[W+15:W] = {16{fill}};
        for (genvar k = 0; k < W; k++) begin : SH
            assign e2[k]  = e1[k + n[1:0]];
            assign e3[k]  = e2[k + {n[3:2], 2'b00}];
            wire kb = (k == 0) ? kf0 : (ONE[k] | ~f_cmpk) & kf;
            assign shr[k] = ge ? kb : n[4] ? e3[k + 16] : e3[k];
        end
        // t: logic ops, the MIN / MAX pick, |x| and -x (the subtractor), one LUT6 per bit: a lane-level 3-bit code
        // over (x_k, y_k, d_k)
        wire         pick_x = f_min ? lt : ~lt;
        wire [2:0]   tc   = f_log ? {1'b0, f_lg} : f_mm ? (pick_x ? 3'd4 : 3'd5)
                          : (f_neg | (f_abs & x[W-1])) ? 3'd6 : 3'd4;
        reg  [W-1:0] t;
        always @*
            for (int k = 0; k < W; k++)
                case (tc)
                    3'd0: t[k] = x[k] & y[k];  3'd1: t[k] = x[k] | y[k];  3'd2: t[k] = x[k] ^ y[k];  3'd3: t[k] = ~x[k];
                    3'd4: t[k] = x[k];         3'd5: t[k] = y[k];         default: t[k] = d[k];
                endcase
        wire [W-1:0] shr_r = rev(shr);
        always @(posedge clk)
            for (int k = 0; k < W; k++)
                y_iop[li*W + k] <= ~(f_sh | f_k) ? t[k] : f_shl ? shr_r[k] : shr[k];
    end endgenerate

    // ---- IMUL: low W bits of a*b = low W bits of (a[LB-1:0]*b + (a[W-1:LB]*b[W-LB-1:0]) << LB), LB = 17: the first
    // product (17 x W bits) fits one DSP48 (18 x 25), the second only feeds bits W-1..LB, so its low W-LB bits come
    // from a small LUT multiplier. One DSP per lane instead of two; latency 3: in register | products | sum. ----
    localparam int LB = 17;
    reg [NL*W-1:0] ma1, mb1; reg vm1, vm2;
    genvar l;
    generate for (l = 0; l < NL; l = l + 1) begin : MUL
        wire [W-1:0] ax = ma1[l*W +: W], bx = mb1[l*W +: W];
        if (W > LB + 1) begin : g_split                        // W = 24: one DSP + a small LUT multiplier
            (* use_dsp = "yes" *) reg [W-1:0]    plo;           // (a_lo * b) mod 2^W
            (* use_dsp = "no"  *) reg [W-LB-1:0] phi;           // (a_hi * b_lo) mod 2^(W-LB)
            always @(posedge clk) begin
                plo <= ax[LB-1:0] * bx;
                phi <= ax[W-1:LB] * bx[W-LB-1:0];
                y_mul[l*W +: W] <= plo + {phi, {LB{1'b0}}};
            end
        end else begin : g_one                                 // W <= 18: the whole product fits one DSP
            (* use_dsp = "yes" *) reg [W-1:0] p;
            always @(posedge clk) begin p <= ax * bx; y_mul[l*W +: W] <= p; end
        end
    end endgenerate
    always @(posedge clk) begin
        ma1 <= a; mb1 <= b; vm1 <= mul_valid & ~rst; vm2 <= vm1 & ~rst; v_mul <= vm2 & ~rst;
    end

    // ---- IDIV / IMOD / UDIV / UMOD: restoring division on |dividend| with the divisor kept signed. Each lane has
    // ONE W+1-bit adder: phase NEG (signed modes only) negates a negative dividend in place (0 - dq), then W steps
    // compute rs - |dv| as rs - dv or rs + dv by the divisor's sign. A final conditional negation (an incrementer)
    // gives the result's sign. busy W + 2 cycles (signed) / W + 1 (unsigned). ----
    localparam [1:0] P_NEG = 2'd1, P_IT = 2'd2, P_OUT = 2'd3;
    reg [1:0] ph; reg [CW-1:0] cnt; reg [1:0] mode;
    reg vd_i; reg [NL*W-1:0] yd_i;                             // one round's result (= the job's when DSEQ is 1)
    // DSEQ > 1: the job's operands in LUTRAM (entry = the feed cycle), one round per entry, the results back in order
    localparam int SW = (DSEQ > 1) ? $clog2(DSEQ) : 1;
    (* ram_style = "distributed" *) reg [NL*W-1:0] dba [0:DSEQ-1], dbb [0:DSEQ-1], dyo [0:DSEQ-1];
    reg cap, run, outp, rgo, vo; reg [SW-1:0] di, dr, dq; reg [NL*W-1:0] yo;
    wire cap_start = (DSEQ > 1) & div_valid & ~cap & ~run & ~outp;
    wire [SW-1:0] wi = cap ? di : '0;
    always @(posedge clk) begin
        if (div_valid & (cap_start | cap)) begin dba[wi] <= a; dbb[wi] <= b; end
        if (vd_i) dyo[dr] <= yd_i;
        yo <= dyo[dq];
        if (rst) begin cap <= 1'b0; run <= 1'b0; outp <= 1'b0; rgo <= 1'b0; vo <= 1'b0; di <= '0; dr <= '0; dq <= '0; end
        else begin
            rgo <= cap_start | (run & vd_i & (dr != SW'(DSEQ-1)));
            if (cap_start) begin cap <= (DSEQ > 1); di <= SW'(1); run <= 1'b1; dr <= '0; end
            else if (cap) begin di <= di + 1'b1; if (di == SW'(DSEQ-1)) cap <= 1'b0; end
            if (run & vd_i) begin
                if (dr == SW'(DSEQ-1)) begin run <= 1'b0; outp <= 1'b1; dq <= '0; end
                else dr <= dr + 1'b1;
            end
            vo <= outp;
            if (outp) begin dq <= dq + 1'b1; if (dq == SW'(DSEQ-1)) outp <= 1'b0; end
        end
    end
    wire [NL*W-1:0] da = (DSEQ > 1) ? dba[dr] : a, db = (DSEQ > 1) ? dbb[dr] : b;   // the round's operands
    wire dstart = (DSEQ > 1) ? rgo : div_valid;                // a round starts
    assign div_ready = (DSEQ > 1) ? ~(cap | run | outp) : (ph == 2'd0);
    assign v_div = (DSEQ > 1) ? vo : vd_i;
    assign y_div = (DSEQ > 1) ? yo : yd_i;
    wire start = dstart & (ph == 2'd0);
    wire uns   = (DSEQ > 1) ? mode[1] : div_mode[1];           // UDIV / UMOD: operands as unsigned
    always @(posedge clk) if ((DSEQ > 1) ? cap_start : start) mode <= div_mode;
    // the phase's next state, shared; every lane keeps its own copy of the phase (phl == ph always): the phase drives
    // the clock enables of the lane's 2 W-bit registers — from one register it would reach NL x 2W flops across the die
    // (a timing miss)
    reg [1:0] ph_n;
    always @* case (ph)
        2'd0:    ph_n = start ? (uns ? P_IT : P_NEG) : 2'd0;
        P_NEG:   ph_n = P_IT;
        P_IT:    ph_n = (cnt == W - 1) ? P_OUT : P_IT;
        default: ph_n = 2'd0;
    endcase
    generate for (l = 0; l < NL; l = l + 1) begin : DIV
        wire [W-1:0] sa = da[l*W +: W], sb = db[l*W +: W];
        reg  [W-1:0] dq, dv, rem; reg nq, nr, dz, an, bn;
        (* dont_touch = "true" *) reg [1:0] phl;              // the lane's copy of ph
        always @(posedge clk) phl <= rst ? 2'd0 : ph_n;
        wire         startl = dstart & (phl == 2'd0);
        wire         it   = (phl == P_IT);
        wire         addm = it & bn;                           // negative divisor: rs + dv
        wire [W:0]   ra   = it ? {rem, dq[W-1]} : '0;          // NEG: 0 - dq
        wire [W:0]   rb   = it ? {bn, dv} : {1'b0, dq};
        wire [W:0]   diff = addm ? ra + rb : ra - rb;
        wire         neg  = mode[0] ? nr : nq;
        always @(posedge clk) begin
            if (startl) begin dq <= sa; dv <= sb; rem <= '0; dz <= (sb == 0);
                             an <= ~uns & sa[W-1]; bn <= ~uns & sb[W-1];
                             nq <= ~uns & (sa[W-1] ^ sb[W-1]); nr <= ~uns & sa[W-1]; end
            else if (phl == P_NEG) begin if (an) dq <= diff[W-1:0]; end
            else if (it) begin dq <= {dq[W-2:0], ~diff[W]}; rem <= diff[W] ? ra[W-1:0] : diff[W-1:0]; end
            if (phl == P_OUT)
                yd_i[l*W +: W] <= (mode == 2'd0 && dz) ? {W{1'b1}}                        // IDIV x / 0 = -1
                                : ((mode[0] ? rem : dq) ^ {W{neg}}) + W'(neg);
        end
    end endgenerate
    always @(posedge clk) begin
        vd_i <= 1'b0;
        if (rst) begin ph <= 2'd0; cnt <= '0; end
        else begin
            ph <= ph_n;
            case (ph)
                2'd0:    if (start) cnt <= '0;
                P_IT:    cnt <= cnt + 1'b1;
                P_OUT:   vd_i <= 1'b1;
                default: ;
            endcase
        end
    end
endmodule
`default_nettype wire
