// quad_setup — board B per-pixel setup in front of the fragment shader. One pixel per
// cycle, fixed latency LAT, no back-pressure (the caller keeps room for what it issued). Bit-exact model:
// tools/b_ref.py quad_setup_e / quad_setup_px (FP24 units: float_madd = round(mul) then round(add), float_sfu RCP).
//   A pixel arrives as its quad's pixel-0 edge values e0 e1 e2 (signed 32-bit
//   integers from A's rasterizer, 1/256 pixel^2 >> the triangle's shift) and its index p in the quad; the triangle's
//   slot holds the per-pixel steps A_k (x), B_k (y) and R = RCP(i2f(a2 >> s)):
//   e_k(p) = e_k + A_k (p & 1) + B_k (p >> 1)        32-bit adds (two stages)
//   mu_k   = MUL(i2f(e_k(p)), R)                      float_cvt_i2f (RNE) then float_mul — the barycentrics
//   z    = MAD(mu2, dz2, MAD(mu1, dz1, z0))           window z (delta form: a constant depth is exact)
//   w_f  = MAD(mu2, q2, MAD(mu1, q1, MUL(mu0, q0)))    gl_FragCoord.w = interpolated 1/w_c (positive terms)
//   l_k  = MUL(mu_k * q_k, RCP(w_f))                   perspective weights; mu_k * q_k = the MAD's product
//   d24  = clamp(rne(z * (2^b - 1)) + o24, 0, 2^b - 1)            unorm depth of the attachment's b bits (24, or 16
//                                                                  for DEPTH_COMPONENT16: depth16; one depth domain)
//   gl_FragCoord = (x + 0.5, y + 0.5, rne(d24 / (2^b - 1)), w_f)
// Triangle constants: NT slots (tri_we), read by the pixel's slot at each stage, so pixels of a new triangle may
// follow at once; the caller must not rewrite a slot while pixels of it are in flight (LAT cycles). The polygon
// offset o24 = rne(factor * max(|dz/dx|, |dz/dy|) * (2^b - 1)) + units (saturated to +-2^25; r = one unit of the
// attachment) is computed when the slot is written and lands OFS_LAT cycles later, before any pixel of the slot
// reaches the depth stage. depth16 is constant through a frame.
`default_nettype none
module quad_setup #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16,
    parameter int TAGW  = 8,
    parameter int NT    = 4,                      // triangle slots
    localparam int W    = 1 + EXP_W + MAN_W,
    localparam int SW   = $clog2(NT)
)(
    input  wire          clk, rst,
    // triangle constants
    input  wire          tri_we,
    input  wire [SW-1:0] tri_slot,
    input  wire [W-1:0]  tri_z0, tri_dz1, tri_dz2, tri_q0, tri_q1, tri_q2, tri_dzdx, tri_dzdy,
    input  wire [W-1:0]  tri_R,                   // RCP(i2f(a2 >> s))
    input  wire signed [31:0] tri_A0, tri_B0, tri_A1, tri_B1, tri_A2, tri_B2,   // per-pixel edge steps (x, y) >> s
    input  wire          po_en,
    input  wire [W-1:0]  po_factor,
    input  wire signed [23:0] po_units,
    input  wire          depth16,                 // the depth attachment has 16 bits (else 24)
    // pixels
    input  wire          px_valid,
    input  wire [SW-1:0] px_slot,
    input  wire signed [31:0] px_e0, px_e1, px_e2,  // the quad's pixel-0 edge values
    input  wire [1:0]    px_p,                    // the pixel's index in its quad
    input  wire [12:0]   px_x, px_y,
    input  wire [TAGW-1:0] px_tag,
    // results (LAT cycles after px_valid)
    output wire          o_valid,
    output wire [TAGW-1:0] o_tag,
    output wire [W-1:0]  o_z, o_fw, o_l1, o_l2,
    output wire [23:0]   o_d24,
    output wire [W-1:0]  o_fcx, o_fcy, o_fcz,
    output wire [W-1:0]  o_mu1, o_mu2             // the barycentrics (gl_PointCoord of a point)
);
    localparam int M = MAN_W, E = EXP_W, BIAS = (1 << (E-1)) - 1;
    // the block's own reset register: one cycle behind the input, replicated by synthesis near its loads
    (* max_fanout = 64 *) reg rst_q = 1'b1;
    always @(posedge clk) rst_q <= rst;
    localparam int MUL_LAT = 3, MADD_LAT = 3 + ((MAN_W <= 10) ? 3 : 5), RCP_LAT = 10, I2F_LAT = 2;
    localparam int FL = 2 + I2F_LAT + MUL_LAT;          // the integer front: steps (2), i2f, x R -> the mus
    // stage times (cycles after the input register)
    localparam int T_W1 = MUL_LAT;                     // MAD(mu1, q1, mu0*q0) starts
    localparam int T_Z2 = MADD_LAT;                    // MAD(mu2, dz2, .) starts
    localparam int T_W2 = T_W1 + MADD_LAT;             // MAD(mu2, q2, .) starts
    localparam int T_ZR = T_Z2 + MADD_LAT;             // z ready
    localparam int T_RC = T_W2 + MADD_LAT;             // w_f ready, RCP starts
    localparam int T_L  = T_RC + RCP_LAT;              // r ready, lambda MULs start
    localparam int T_O  = T_L + MUL_LAT;               // lambdas ready = output
    localparam int T_D  = T_ZR + 4;                    // d24 ready (4 conversion stages), fcz 2 cycles later
    localparam int LAT  = T_O + 1 + FL + 1;            // + the front and its input register, + the input register
    localparam int OFS_LAT = MUL_LAT + 5;

    // ---------------------------------------------------------------- triangle slots
    reg [W-1:0] s_z0[0:NT-1], s_dz1[0:NT-1], s_dz2[0:NT-1], s_q0[0:NT-1], s_q1[0:NT-1], s_q2[0:NT-1], s_R[0:NT-1];
    reg signed [31:0] s_A[0:2][0:NT-1], s_B[0:2][0:NT-1];
    reg signed [26:0] s_o24[0:NT-1];
    always @(posedge clk) if (tri_we) begin
        s_z0[tri_slot] <= tri_z0; s_dz1[tri_slot] <= tri_dz1; s_dz2[tri_slot] <= tri_dz2;
        s_q0[tri_slot] <= tri_q0; s_q1[tri_slot] <= tri_q1;   s_q2[tri_slot] <= tri_q2;   s_R[tri_slot] <= tri_R;
        s_A[0][tri_slot] <= tri_A0; s_B[0][tri_slot] <= tri_B0; s_A[1][tri_slot] <= tri_A1; s_B[1][tri_slot] <= tri_B1;
        s_A[2][tri_slot] <= tri_A2; s_B[2][tri_slot] <= tri_B2;
    end

    // ---------------------------------------------------------------- the integer front: e(p), i2f, x R -> mu
    // f0: input register; f1: + A (p & 1); f2: + B (p >> 1); i2f (2); MUL by R (3) -> the mus land FL cycles after f0
    reg f0_v; reg [SW-1:0] f0_sl; reg signed [31:0] f0_e [0:2]; reg [1:0] f0_p; reg [12:0] f0_x, f0_y; reg [TAGW-1:0] f0_tag;
    reg f1_v, f2_v; reg [SW-1:0] f1_sl, f2_sl; reg [1:0] f1_p; reg signed [31:0] f1_e [0:2], f2_e [0:2];
    always @(posedge clk) begin
        f0_v <= rst_q ?1'b0 : px_valid;
        f0_sl <= px_slot; f0_e[0] <= px_e0; f0_e[1] <= px_e1; f0_e[2] <= px_e2; f0_p <= px_p; f0_x <= px_x; f0_y <= px_y; f0_tag <= px_tag;
        f1_v <= rst_q ?1'b0 : f0_v; f1_sl <= f0_sl; f1_p <= f0_p;
        f2_v <= rst_q ?1'b0 : f1_v; f2_sl <= f1_sl;
        for (int k = 0; k < 3; k++) begin
            f1_e[k] <= f0_e[k] + (f0_p[0] ? s_A[k][f0_sl] : 32'sd0);
            f2_e[k] <= f1_e[k] + (f1_p[1] ? s_B[k][f1_sl] : 32'sd0);
        end
    end
    wire [2:0] cv_v; wire [W-1:0] cv_r [0:2]; wire [2:0] fm_v; wire [W-1:0] fm_r [0:2];
    reg [SW-1:0] fc_sl [1:I2F_LAT];                     // the slot alongside the converters (R is read at the MUL)
    always @(posedge clk) begin
        fc_sl[1] <= f2_sl;
        for (int i = 2; i <= I2F_LAT; i++) fc_sl[i] <= fc_sl[i-1];
    end
    genvar gk;
    generate for (gk = 0; gk < 3; gk = gk + 1) begin : g_front
        float_cvt_i2f #(.IW(32), .EXP_W(EXP_W), .MAN_W(MAN_W)) u_cv (.clk, .rst(rst_q), .in_valid(f2_v), .ival(f2_e[gk]),
            .out_valid(cv_v[gk]), .result(cv_r[gk]));
        float_mul #(EXP_W, MAN_W) u_fm (.clk, .rst(rst_q), .in_valid(cv_v[gk]), .a(cv_r[gk]), .b(s_R[fc_sl[I2F_LAT]]), .in_ready(),
            .out_valid(fm_v[gk]), .result(fm_r[gk]));
    end endgenerate
    // slot / x / y / tag alongside the front (index = cycles after f0)
    reg [SW-1:0] fd_sl [1:FL]; reg [12:0] fd_x [1:FL], fd_y [1:FL]; reg [TAGW-1:0] fd_tag [1:FL];
    always @(posedge clk) begin
        fd_sl[1] <= f0_sl; fd_x[1] <= f0_x; fd_y[1] <= f0_y; fd_tag[1] <= f0_tag;
        for (int i = 2; i <= FL; i++) begin fd_sl[i] <= fd_sl[i-1]; fd_x[i] <= fd_x[i-1]; fd_y[i] <= fd_y[i-1]; fd_tag[i] <= fd_tag[i-1]; end
    end

    // z * (2^b - 1) rounded to nearest even, saturated to +-2^25 (b_ref z_to_int): stage A = magnitude product
    // and shift amount, stage B1 = shift, guard and sticky bits, stage B2 = round, sign, saturate.
    function automatic [52:0] zi_a(input [W-1:0] z, input d16);   // {kind[1:0] (0 num, 1 zero, 2 sat), sign, rsh[7:0], P[41:0]}
        reg s; reg [E-1:0] e; reg [M:0] sig; reg [41:0] p; reg [1:0] k; reg [7:0] rsh;
        s = z[W-1]; e = z[W-2 -: E]; sig = {1'b1, z[M-1:0]};
        p = d16 ? (42'(sig) << 16) - 42'(sig) : (42'(sig) << 24) - 42'(sig);
        k = 2'd0; rsh = 8'd0;
        if (e == 0 || (e == E'((1 << E) - 1) && z[M-1:0] != 0)) k = 2'd1;    // FTZ zero, NaN -> 0
        else if (e >= E'(BIAS + 1)) k = 2'd2;                                // |z| >= 2 (or Inf): saturate
        else rsh = 8'(BIAS + M) - 8'(e);                                     // value = P * 2^-(rsh), rsh >= 16
        zi_a = {k, s, rsh, 42'(p)};
    endfunction
    // B1: {kind, sign, guard, sticky, q[25:0]}; P < 2^41 and rsh >= 16, so q < 2^25 (rsh > 41: below 1/2, q = g = 0)
    function automatic [30:0] zi_b1(input [52:0] a);
        reg [1:0] k; reg s; reg [7:0] rsh; reg [41:0] p, q; reg g, st;
        {k, s, rsh, p} = a;
        if (k != 2'd0 || rsh > 8'd41) begin q = 42'd0; g = 1'b0; st = 1'b0; end
        else begin
            q = p >> rsh; g = p[rsh - 1'b1]; st = |(p & ((42'd1 << (rsh - 1'b1)) - 1'b1));
        end
        zi_b1 = {k, s, g, st, 26'(q)};
    endfunction
    // B2: round half to even (up = guard and (sticky or odd)), sign, saturate
    function automatic signed [26:0] zi_b2(input [30:0] b);
        reg [1:0] k; reg s, g, st; reg [25:0] q; reg [26:0] mag;
        {k, s, g, st, q} = b;
        mag = 27'(q) + 27'(g & (st | q[0]));
        if (k == 2'd1) zi_b2 = 27'sd0;
        else if (k == 2'd2) zi_b2 = s ? -27'sd33554432 : 27'sd33554432;
        else zi_b2 = s ? -$signed(mag) : $signed(mag);
    endfunction

    // ---------------------------------------------------------------- polygon offset (per slot write)
    wire [W-2:0] dzx_m = tri_dzdx[W-2:0], dzy_m = tri_dzdy[W-2:0];
    wire [W-1:0] po_m  = {1'b0, (dzx_m >= dzy_m) ? dzx_m : dzy_m};
    wire         pom_v; wire [W-1:0] pom_r;
    float_mul #(EXP_W, MAN_W) u_pom (.clk, .rst(rst_q), .in_valid(tri_we), .a(po_factor), .b(po_m), .in_ready(),
        .out_valid(pom_v), .result(pom_r));
    reg [SW-1:0] po_sl [0:MUL_LAT-1]; reg po_on [0:MUL_LAT-1]; reg signed [23:0] po_u [0:MUL_LAT-1];
    reg [52:0] po_a; reg po_av; reg [SW-1:0] po_asl; reg po_aon; reg signed [23:0] po_au;
    reg [30:0] po_c; reg po_cv; reg [SW-1:0] po_csl; reg po_con; reg signed [23:0] po_cu;
    reg signed [26:0] po_d; reg po_dv; reg [SW-1:0] po_dsl; reg po_don; reg signed [23:0] po_du;
    reg signed [26:0] po_b; reg po_bv; reg [SW-1:0] po_bsl;
    always @(posedge clk) begin
        po_sl[0] <= tri_slot; po_on[0] <= po_en; po_u[0] <= po_units;
        for (int i = 1; i < MUL_LAT; i++) begin po_sl[i] <= po_sl[i-1]; po_on[i] <= po_on[i-1]; po_u[i] <= po_u[i-1]; end
        po_av <= pom_v; po_a <= zi_a(pom_r, depth16); po_asl <= po_sl[MUL_LAT-1]; po_aon <= po_on[MUL_LAT-1];
        po_au <= po_u[MUL_LAT-1];
        po_cv <= po_av; po_c <= zi_b1(po_a); po_csl <= po_asl; po_con <= po_aon; po_cu <= po_au;
        po_dv <= po_cv; po_d <= zi_b2(po_c); po_dsl <= po_csl; po_don <= po_con; po_du <= po_cu;
        po_bv <= po_dv; po_bsl <= po_dsl;
        begin : OFS
            reg signed [27:0] v;
            v = 28'(po_d) + 28'(po_du);
            if (!po_don) po_b <= 27'sd0;
            else if (v > 28'sd33554432) po_b <= 27'sd33554432;
            else if (v < -28'sd33554432) po_b <= -27'sd33554432;
            else po_b <= 27'(v);
        end
        if (po_bv) s_o24[po_bsl] <= po_b;
    end

    // ---------------------------------------------------------------- input register (the mus from the front) + delay lines
    reg i_v; reg [SW-1:0] i_sl; reg [W-1:0] i_mu0, i_mu1, i_mu2; reg [12:0] i_x, i_y; reg [TAGW-1:0] i_tag;
    always @(posedge clk) begin
        i_v <= rst_q ?1'b0 : fm_v[0];
        i_sl <= fd_sl[FL]; i_mu0 <= fm_r[0]; i_mu1 <= fm_r[1]; i_mu2 <= fm_r[2]; i_x <= fd_x[FL]; i_y <= fd_y[FL]; i_tag <= fd_tag[FL];
    end
    // slot / mu1 / mu2 per cycle after the input register (index = cycle; the mus run to the output: gl_PointCoord)
    reg [SW-1:0] d_sl [1:T_D]; reg [W-1:0] d_mu1 [1:T_O]; reg [W-1:0] d_mu2 [1:T_O];
    always @(posedge clk) begin
        d_sl[1] <= i_sl; d_mu1[1] <= i_mu1; d_mu2[1] <= i_mu2;
        for (int i = 2; i <= T_D; i++)  d_sl[i]  <= d_sl[i-1];
        for (int i = 2; i <= T_O; i++) begin d_mu1[i] <= d_mu1[i-1]; d_mu2[i] <= d_mu2[i-1]; end
    end

    // ---------------------------------------------------------------- z
    wire z1_v, z2_v; wire [W-1:0] z1_r, z2_r;
    float_madd #(EXP_W, MAN_W) u_z1 (.clk, .rst(rst_q), .in_valid(i_v), .sub(1'b0), .a(i_mu1), .b(s_dz1[i_sl]),
        .c(s_z0[i_sl]), .out_valid(z1_v), .result(z1_r), .prod_o());
    float_madd #(EXP_W, MAN_W) u_z2 (.clk, .rst(rst_q), .in_valid(z1_v), .sub(1'b0), .a(d_mu2[T_Z2]),
        .b(s_dz2[d_sl[T_Z2]]), .c(z1_r), .out_valid(z2_v), .result(z2_r), .prod_o());

    // ---------------------------------------------------------------- 1 / w and the weights
    wire w0_v, w1_v, w2_v; wire [W-1:0] w0_r, w1_r, w2_r, t1_r, t2_r;
    float_mul  #(EXP_W, MAN_W) u_w0 (.clk, .rst(rst_q), .in_valid(i_v), .a(i_mu0), .b(s_q0[i_sl]), .in_ready(),
        .out_valid(w0_v), .result(w0_r));
    float_madd #(EXP_W, MAN_W) u_w1 (.clk, .rst(rst_q), .in_valid(w0_v), .sub(1'b0), .a(d_mu1[T_W1]), .b(s_q1[d_sl[T_W1]]),
        .c(w0_r), .out_valid(w1_v), .result(w1_r), .prod_o(t1_r));
    float_madd #(EXP_W, MAN_W) u_w2 (.clk, .rst(rst_q), .in_valid(w1_v), .sub(1'b0), .a(d_mu2[T_W2]), .b(s_q2[d_sl[T_W2]]),
        .c(w1_r), .out_valid(w2_v), .result(w2_r), .prod_o(t2_r));
    // the products mu_k * q_k appear MUL_LAT cycles after their MAD starts; hold them until the reciprocal is ready
    reg [W-1:0] d_t1 [T_W1+MUL_LAT+1:T_L], d_t2 [T_W2+MUL_LAT+1:T_L];
    always @(posedge clk) begin
        d_t1[T_W1+MUL_LAT+1] <= t1_r; d_t2[T_W2+MUL_LAT+1] <= t2_r;
        for (int i = T_W1+MUL_LAT+2; i <= T_L; i++) d_t1[i] <= d_t1[i-1];
        for (int i = T_W2+MUL_LAT+2; i <= T_L; i++) d_t2[i] <= d_t2[i-1];
    end
    wire r_v; wire [W-1:0] r_r;
    float_sfu #(.EXP_W(EXP_W), .MAN_W(MAN_W), .K(EXP_W >= 7 ? 12 : 8), .KLOG(EXP_W >= 7 ? 11 : 8),
        .RCP_MEM(EXP_W >= 7 ? "tools/gen/fp24_rcp_seed.mem" : "tools/gen/fp16_rcp_seed.mem"))
      u_rcp (.clk, .rst(rst_q), .in_valid(w2_v), .op(3'd0), .x(w2_r), .out_valid(r_v), .result(r_r));
    wire l1_v; wire [W-1:0] l1_r, l2_r;
    float_mul #(EXP_W, MAN_W) u_l1 (.clk, .rst(rst_q), .in_valid(r_v), .a(d_t1[T_L]), .b(r_r), .in_ready(), .out_valid(l1_v),
        .result(l1_r));
    float_mul #(EXP_W, MAN_W) u_l2 (.clk, .rst(rst_q), .in_valid(r_v), .a(d_t2[T_L]), .b(r_r), .in_ready(), .out_valid(),
        .result(l2_r));
    // w_f waits for the weights
    reg [W-1:0] d_fw [T_RC+1:T_O];
    always @(posedge clk) begin
        d_fw[T_RC+1] <= w2_r;
        for (int i = T_RC+2; i <= T_O; i++) d_fw[i] <= d_fw[i-1];
    end

    // ---------------------------------------------------------------- depth: d24 and gl_FragCoord.z
    reg [52:0] dz_a; reg [30:0] dz_b; reg signed [26:0] dz_i; reg [23:0] dz_d24;
    wire signed [27:0] dmax = depth16 ? 28'sd65535 : 28'sd16777215;
    always @(posedge clk) begin
        dz_a <= zi_a(z2_r, depth16);                               // T_ZR + 1
        dz_b <= zi_b1(dz_a);                                       // T_ZR + 2
        dz_i <= zi_b2(dz_b);                                       // T_ZR + 3
        begin : D24                                                // T_ZR + 4: + offset, clamp
            reg signed [27:0] v;
            v = 28'(dz_i) + 28'(s_o24[d_sl[T_ZR+3]]);
            dz_d24 <= (v < 0) ? 24'd0 : (v > dmax) ? 24'(dmax) : 24'(v);
        end
    end
    // gl_FragCoord.z = k / (2^b - 1) in two stages: leading zeros + normalise, then round + pack.
    // k / (2^24 - 1) = ({k, k} + eps) / 2^48 and k / (2^16 - 1) = ({k, k, k} + eps) / 2^48 with 0 < eps < 1 for
    // 0 < k < 2^b - 1, so the sticky bit is always set: round up iff the guard bit is (b_ref check_conv: exhaustive)
    function automatic [4:0] fz_lzc(input [23:0] k);
        fz_lzc = 5'd0;
        for (int i = 0; i < 24; i++) if (k[23 - i] == 1'b0 && fz_lzc == 5'(i)) fz_lzc = 5'(i + 1);
    endfunction
    function automatic [W-1:0] u24f_b(input kz, input ko, input [4:0] lz, input [47:0] n);
        reg [M+1:0] q; reg [E-1:0] be;
        if (kz) u24f_b = '0;
        else if (ko) u24f_b = W'(BIAS) << M;
        else begin
            q = {1'b0, n[47 -: M+1]} + (M+2)'(n[47-M-1]);    // n = the repeated k << lz: leading one at bit 47
            be = E'(BIAS - 1) - E'(lz);                       // value in [2^-1-lz, 2^-lz)
            if (q[M+1]) u24f_b = {1'b0, be + 1'b1, M'(0)};
            else u24f_b = {1'b0, be, q[M-1:0]};
        end
    endfunction
    wire [23:0] fz_kk = depth16 ? {dz_d24[15:0], 8'd0} : dz_d24;   // the b bits left-aligned (leading zeros)
    wire [47:0] fz_rep = depth16 ? {dz_d24[15:0], dz_d24[15:0], dz_d24[15:0]} : {dz_d24, dz_d24};
    reg fz_z, fz_o; reg [4:0] fz_lz; reg [47:0] fz_n;
    always @(posedge clk) begin                                    // T_D + 1
        fz_z <= (dz_d24 == 24'd0); fz_o <= (dz_d24 == 24'(dmax));
        fz_lz <= fz_lzc(fz_kk); fz_n <= fz_rep << fz_lzc(fz_kk);
    end
    reg [23:0] d_d24 [T_D+1:T_O]; reg [W-1:0] d_fz [T_D+2:T_O];
    always @(posedge clk) begin
        d_d24[T_D+1] <= dz_d24; d_fz[T_D+2] <= u24f_b(fz_z, fz_o, fz_lz, fz_n);
        for (int i = T_D+2; i <= T_O; i++) d_d24[i] <= d_d24[i-1];
        for (int i = T_D+3; i <= T_O; i++) d_fz[i] <= d_fz[i-1];
    end
    reg [W-1:0] d_z [T_ZR+1:T_O];
    always @(posedge clk) begin
        d_z[T_ZR+1] <= z2_r;
        for (int i = T_ZR+2; i <= T_O; i++) d_z[i] <= d_z[i-1];
    end

    // ---------------------------------------------------------------- x + 0.5, y + 0.5 (exact), tag
    function automatic [W-1:0] half_odd(input [12:0] c);           // (2c + 1) / 2, exact for M >= 13
        reg [13:0] v; reg [3:0] lz; reg [13:0] n; reg [M-1:0] man;
        v = {c, 1'b1}; lz = 4'd0;
        for (int i = 0; i < 14; i++) if (v[13 - i] == 1'b0 && lz == 4'(i)) lz = 4'(i + 1);
        n = v << lz;
        if (M >= 13) man = M'(n[12:0]) << (M - 13); else man = M'(n[12:0] >> (13 - M));
        half_odd = {1'b0, E'(BIAS + 12) - E'(lz), man};
    endfunction
    reg [W-1:0] d_fx [1:T_O], d_fy [1:T_O]; reg [TAGW-1:0] d_tag [1:T_O]; reg [T_O:1] d_v;
    always @(posedge clk) begin
        d_fx[1] <= half_odd(i_x); d_fy[1] <= half_odd(i_y); d_tag[1] <= i_tag;
        for (int i = 2; i <= T_O; i++) begin d_fx[i] <= d_fx[i-1]; d_fy[i] <= d_fy[i-1]; d_tag[i] <= d_tag[i-1]; end
        d_v <= rst_q ?'0 : {d_v[T_O-1:1], i_v};
    end

    // ---------------------------------------------------------------- outputs (cycle T_O, registered)
    reg r_ov; reg [TAGW-1:0] r_tag; reg [W-1:0] r_z, r_fw, r_l1, r_l2, r_fx, r_fy, r_fz, r_mu1, r_mu2; reg [23:0] r_d24;
    always @(posedge clk) begin
        r_ov <= rst_q ?1'b0 : d_v[T_O];
        r_tag <= d_tag[T_O]; r_z <= d_z[T_O]; r_fw <= d_fw[T_O]; r_l1 <= l1_r; r_l2 <= l2_r;
        r_d24 <= d_d24[T_O]; r_fz <= d_fz[T_O]; r_fx <= d_fx[T_O]; r_fy <= d_fy[T_O]; r_mu1 <= d_mu1[T_O]; r_mu2 <= d_mu2[T_O];
    end
    assign o_valid = r_ov; assign o_tag = r_tag; assign o_z = r_z; assign o_fw = r_fw; assign o_l1 = r_l1;
    assign o_l2 = r_l2; assign o_d24 = r_d24; assign o_fcx = r_fx; assign o_fcy = r_fy; assign o_fcz = r_fz;
    assign o_mu1 = r_mu1; assign o_mu2 = r_mu2;
    wire _unused = &{1'b0, z2_v, l1_v, r_v, f2_v, cv_v, fm_v[2:1]};
endmodule
`default_nettype wire
