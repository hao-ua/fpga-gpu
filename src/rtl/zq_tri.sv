// zq_tri — the four depth-bound words of a primitive's TRI item (w18..w21; bit-exact model: tools/zq_ref.py zq_words),
// computed once per primitive in board A's setup and read by zq_lane on board B:
//   w18 {ok, k[30:25], m1[24:0]}, w19 m2[24:0], w20 Clo, w21 Chi (signed, in units of 2^-24 window z)
//   k = eR - 47; m_j = round(mR mdj 2^(edj - 73)) with the sign of R dz_j; A', B' = floor(step 2^k) (18 bits);
//   Sx = A1' m1 + A2' m2, Sy = B1' m1 + B2' m2; Z0 = floor(z0 2^24); M = 2^(e0-54) + 2^(ed1-50) + 2^(ed2-50) + 16;
//   Clo = Z0 + floor(min(Sx, 0) / 2^14) + floor(min(Sy, 0) / 2^14) - M, Chi = Z0 + floor(max(..)) + .. + M + 4.
//   ok = 0 (all four words 0) for a line, R outside [2^-30, 1], z0 < 0, |z0| or |dz| >= 2, or a step beyond 18 bits.
// One 25 x 18 multiplier used in turn (2 mantissa products, 4 step products); `done` 12 cycles after `start`.
`default_nettype none
module zq_tri (
    input  wire         clk, rst,
    input  wire         start,
    input  wire [23:0]  R, z0, dz1, dz2,        // FP24 (s1 e7 m16)
    input  wire [31:0]  A1, B1, A2, B2,         // the per-pixel edge steps (>> s), as in the TRI item
    input  wire         line,
    output reg          done,
    output reg  [127:0] words                   // {w21, w20, w19, w18}
);
    reg [23:0] r_R, r_z0, r_dz1, r_dz2; reg [31:0] st [0:3]; reg r_line;
    reg [3:0] c; reg run;
    wire [6:0] eR = r_R[22:16], e0 = r_z0[22:16], ed1 = r_dz1[22:16], ed2 = r_dz2[22:16];
    wire [16:0] mR = {1'b1, r_R[15:0]}, m0 = {|e0, r_z0[15:0]};
    wire signed [5:0] k = 6'(7'(eR) - 7'd47);
    // the step shifter (one step per cycle, c = 0..3), as zq_lane's
    function automatic [17:0] shk(input [31:0] e, input signed [5:0] kk);
        reg [63:0] ev; reg [94:0] sh; reg [5:0] sa;
        ev = {{32{e[31]}}, e}; sa = 6'(kk + 6'sd14);
        sh = {31'd0, ev} << sa;
        shk = sh[31:14];
    endfunction
    function automatic [5:0] lsc(input [31:0] e);
        lsc = 6'd31;
        for (int i = 30; i >= 0; i--) if (e[i] != e[31] && lsc == 6'd31) lsc = 6'(30 - i);
    endfunction
    // m = round(P 2^(ed - 73)), P = mR md (34 bits): the slope per e' unit — in two steps (timing at 120 MHz: the
    // variable 36-bit round + shift straight off the multiplier is 19-21 logic levels): the rounding add with a
    // registered shift amount (slope_q), the shift and sign a cycle later (slope_m)
    function automatic [35:0] slope_q(input [33:0] P, input [6:0] sh);
        slope_q = {2'b00, P} + (36'd1 << (sh - 7'd1));
    endfunction
    function automatic signed [24:0] slope_m(input [35:0] q, input [6:0] sh, input zero, input neg);
        reg [35:0] qs; reg [24:0] mag;
        qs = q >> sh; mag = zero ? 25'd0 : qs[24:0];
        slope_m = neg ? -$signed(mag) : $signed(mag);
    endfunction
    reg [6:0] sh1_r, sh2_r; reg z1_r, z2_r, ng1_r, ng2_r; reg [35:0] q1, q2;   // the slopes' shift amounts, zero cases, signs
    // the output words' start-time terms, registered at c = 1 (computed in the assembly at c = 10 they would be 16 logic
    // levels from r_z0)
    reg signed [31:0] Z0_r, M_r; reg ok_r;
    // the multiplier: operands registered, the product one cycle later
    reg signed [24:0] ma; reg signed [17:0] mb; reg signed [42:0] mp;
    always @(posedge clk) mp <= ma * mb;
    reg signed [17:0] sp [0:3]; reg bad;
    reg signed [24:0] m1, m2; reg signed [43:0] acc, Sx, Sy;
    wire signed [43:0] mp44 = 44'(mp);
    // Z0 = floor(z0 2^24) = m0 2^(e0 - 55) (z0 >= 0 here), the margin
    wire [6:0] z0s = 7'd55 - e0;
    wire signed [31:0] Z0 = (e0 >= 7'd55) ? 32'({15'd0, m0} << (e0 - 7'd55)) : (z0s >= 7'd17) ? 32'd0 : 32'({15'd0, m0} >> z0s);
    function automatic [31:0] p2(input [6:0] e, input [6:0] base); p2 = (e > base) ? (32'd1 << (e - base)) : 32'd1; endfunction
    wire [31:0] M = p2(e0, 7'd54) + p2(ed1, 7'd50) + p2(ed2, 7'd50) + 32'd16;
    wire ok_in = !r_line && eR >= 7'd33 && eR <= 7'd63 && e0 <= 7'd63 && !(r_z0[23] && e0 != 7'd0) && ed1 <= 7'd63 && ed2 <= 7'd63;
    always @(posedge clk) begin
        done <= 1'b0;
        if (rst) begin run <= 1'b0; c <= 4'd0; end
        else if (start) begin
            r_R <= R; r_z0 <= z0; r_dz1 <= dz1; r_dz2 <= dz2; st[0] <= A1; st[1] <= A2; st[2] <= B1; st[3] <= B2; r_line <= line;
            run <= 1'b1; c <= 4'd0; bad <= 1'b0;
        end else if (run) begin
            c <= c + 4'd1;
            if (c <= 4'd3) begin                                  // the steps: A1' A2' B1' B2', each within 18 bits
                sp[c[1:0]] <= shk(st[c[1:0]], k);
                if (lsc(st[c[1:0]]) < 6'(k + 6'sd14)) bad <= 1'b1;
            end
            if (c == 4'd1) begin                                  // (r_* stand from c = 0: every start-time term registered here)
                sh1_r <= 7'd73 - ed1; z1_r <= (ed1 == 7'd0) || ((7'd73 - ed1) >= 7'd36); ng1_r <= r_R[23] ^ r_dz1[23];
                sh2_r <= 7'd73 - ed2; z2_r <= (ed2 == 7'd0) || ((7'd73 - ed2) >= 7'd36); ng2_r <= r_R[23] ^ r_dz2[23];
                Z0_r <= Z0; M_r <= $signed(M); ok_r <= ok_in;
            end
            case (c)
                4'd0: begin ma <= {8'd0, mR}; mb <= {1'b0, {|ed1, r_dz1[15:0]}}; end
                4'd1: begin ma <= {8'd0, mR}; mb <= {1'b0, {|ed2, r_dz2[15:0]}}; end
                4'd2: q1 <= slope_q(34'(mp), sh1_r);                                           // (mp = mR md1)
                4'd3: begin m1 <= slope_m(q1, sh1_r, z1_r, ng1_r); q2 <= slope_q(34'(mp), sh2_r); end   // (mp = mR md2)
                4'd4: begin m2 <= slope_m(q2, sh2_r, z2_r, ng2_r); ma <= m1; mb <= sp[0]; end // A1' m1 (m1 since c = 3)
                4'd5: begin ma <= m2; mb <= sp[1]; end                                        // A2' m2 (m2 since c = 4)
                4'd6: begin ma <= m1; mb <= sp[2]; acc <= mp44; end                           // B1' m1
                4'd7: begin ma <= m2; mb <= sp[3]; Sx <= acc + mp44; end                      // B2' m2
                4'd8: acc <= mp44;
                4'd9: Sy <= acc + mp44;
                4'd10: begin : W
                    reg signed [31:0] clo, chi; reg signed [43:0] nx, ny, px, py;
                    nx = (Sx < 0) ? Sx : 44'sd0; ny = (Sy < 0) ? Sy : 44'sd0; px = (Sx < 0) ? 44'sd0 : Sx; py = (Sy < 0) ? 44'sd0 : Sy;
                    clo = Z0_r + 32'(nx >>> 14) + 32'(ny >>> 14) - M_r;
                    chi = Z0_r + 32'(px >>> 14) + 32'(py >>> 14) + M_r + 32'sd4;
                    words <= (ok_r && !bad) ? {chi, clo, 7'd0, m2, 1'b1, k, m1} : 128'd0;
                    done <= 1'b1; run <= 1'b0;
                end
                default: ;
            endcase
        end
    end
endmodule
`default_nettype wire
