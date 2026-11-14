// pixel_blend_b — board B's ES 2.0 blend + dither + colour mask for one sample of an RGBA8 / RGBA4444 / RGBA5551 /
// RGB565 render target (used by b_rop). Model: tools/b_ref.py blend_fx / dither8.
// ES 2.0 §4.1.6: source (fragment) and destination components are fixed point with at least the framebuffer's bits,
// the blend acts as if in floating point, the result is clamped and converted by rounding to nearest; §4.1.7: then
// dithered (one of the two neighbouring representable values, by the value and x, y only) or, dither off, truncated.
// Every channel works in its own units: a channel of n bits has ONE = 16 (2^n - 1) (the n-bit step in 8.4 fixed point)
//   source component  s  = round(clamp(f, 0, 1) * ONE)  (b_render's back end; the source alpha is converted into the
//                          units of every channel it is a factor of: src_alpha)
//   destination       d  = the stored n-bit value (as 16 d in the same units); factors k / ONE, k = 0..ONE (table 4.2;
//                          the constant colour in channel units, the constant alpha in the units of each channel)
//   destination alpha (as a factor of channel c, in c's units): 16 dA when the alpha channel has n bits too, ONE or 0
//                          for the 1-bit alpha of RGBA5551, ONE for RGB565 (no alpha: 1.0)
//   result c = min(ONE, round((s * fs +- 16 d * fd) / ONE)) (half up; <= 0 -> 0); MIN / MAX: min / max(s, 16 d)
//   blend off: c = s;  then value = floor((c + t) / 16), t = 4x4 Bayer threshold (dither on) or 0: 0 .. 2^n - 1
// The division: w = (x + ONE / 2) >> 4 (< 2^21), floor(w / (2^n - 1)) = (w * ceil(2^33 / (2^n - 1))) >> 33, exact for
// every w of every n (b_ref check_div). RGB565's alpha channel (n = 0) computes nothing meaningful; it is never stored
// as a destination alpha (1.0 above) nor flushed.
// Six stages: A0 operands (the destination's values and the channel's units, registered straight off the band
// buffer: the BRAM read through the factor arithmetic in one cycle misses timing), A factors, B products,
// C sum, D division, E clamp / min / max / dither / mask. Inputs sampled at the clock edge N, final_color valid after
// edge N + 5 (6 register stages counting A0). fmt is constant through a frame.
// Colour layout: channel vectors {R[51:39], G[38:26], B[25:13], A[12:0]}; bytes {R[31:24], G, B, A[7:0]} hold the
// n-bit values right-aligned; mask {R,G,B,A}. src_alpha / blend_ca: {in G's units [25:13], in R and B's units [12:0]}
// (R and B have the same width in every format; the alpha channel uses its own source value / constant).
// Factor codes: 0 ZERO 1 ONE 2 SRC_COLOR 3 1-SRC_COLOR 4 DST_COLOR 5 1-DST_COLOR 6 SRC_ALPHA 7 1-SRC_ALPHA 8 DST_ALPHA
// 9 1-DST_ALPHA 10 CONSTANT_COLOR 11 1-CONSTANT_COLOR 12 CONSTANT_ALPHA 13 1-CONSTANT_ALPHA 14 SRC_ALPHA_SATURATE
// (source only); the alpha column maps colour factors to their alpha and SATURATE to ONE. Ops: 0 ADD 1 SUB (s - d)
// 2 REV_SUB 3 MIN 4 MAX. fmt (tmu24 codes): 0 RGBA8, 1 RGBA4444, 2 RGBA5551, 3 RGB565.
`default_nettype none
module pixel_blend_b (
    input  wire        clk,
    input  wire [1:0]  fmt,
    input  wire [51:0] src_c16,
    input  wire [25:0] src_alpha,
    input  wire [31:0] dst_color,
    input  wire [51:0] blend_c16,
    input  wire [25:0] blend_ca,
    input  wire        enable,
    input  wire [3:0]  src_factor, dst_factor, src_factor_a, dst_factor_a,
    input  wire [2:0]  op, op_a,
    input  wire [3:0]  mask,
    input  wire [3:0]  dith,                    // Bayer threshold (0 when dithering is off)
    output wire [31:0] final_color
);
    // channel widths per format: {R, G, B, A}
    function automatic [3:0] chn(input [1:0] f, input [1:0] ch);
        case (f)
            2'd0: chn = 4'd8;
            2'd1: chn = 4'd4;
            2'd2: chn = (ch == 2'd3) ? 4'd1 : 4'd5;
            default: chn = (ch == 2'd1) ? 4'd6 : (ch == 2'd3) ? 4'd0 : 4'd5;
        endcase
    endfunction
    function automatic [12:0] one_of(input [3:0] n);          // 16 (2^n - 1)
        one_of = (13'd1 << (n + 4)) - 13'd16;
    endfunction
    function automatic [33:0] divm(input [3:0] n);            // ceil(2^33 / (2^n - 1)) (b_ref check_div: exact)
        case (n)
            4'd8: divm = 34'd33686019; 4'd6: divm = 34'd136348169; 4'd5: divm = 34'd277094665;
            4'd4: divm = 34'd572662307; 4'd1: divm = 34'd8589934592; default: divm = 34'd0;
        endcase
    endfunction
    function automatic [12:0] fac(input [3:0] code, input [12:0] s, input [12:0] d16, input [12:0] sa, input [12:0] da16,
                                  input [12:0] cc, input [12:0] ca, input dst, input [12:0] one);
        case (code)
            4'd0: fac = 13'd0;            4'd1: fac = one;
            4'd2: fac = s;                4'd3: fac = one - s;
            4'd4: fac = d16;              4'd5: fac = one - d16;
            4'd6: fac = sa;               4'd7: fac = one - sa;
            4'd8: fac = da16;             4'd9: fac = one - da16;
            4'd10: fac = cc;              4'd11: fac = one - cc;
            4'd12: fac = ca;              4'd13: fac = one - ca;
            4'd14: fac = dst ? 13'd0 : ((sa < one - da16) ? sa : one - da16);
            default: fac = 13'd0;
        endcase
    endfunction
    function automatic [3:0] afac(input [3:0] code);            // alpha column of table 4.2
        case (code)
            4'd2: afac = 4'd6;  4'd3: afac = 4'd7;  4'd4: afac = 4'd8;  4'd5: afac = 4'd9;
            4'd10: afac = 4'd12; 4'd11: afac = 4'd13; 4'd14: afac = 4'd1;
            default: afac = code;
        endcase
    endfunction

    reg [1:0] r_fmt;                                            // constant through a frame
    always @(posedge clk) r_fmt <= fmt;
    reg [3:0]  n_a;                                             // the format-derived constants, registered (timing:
    always @(posedge clk) n_a <= chn(r_fmt, 2'd3);              // keeps r_fmt -> divm -> multiply -> compare out of one cycle)
    wire [7:0]  d_a  = dst_color[7:0];
    // shared control, staged
    reg z_en, a_en, b_en, c_en, d_en; reg [3:0] z_t, a_t, b_t, c_t, d_t, z_m, a_m, b_m, c_m, d_m;
    always @(posedge clk) begin
        z_en <= enable; a_en <= z_en; b_en <= a_en; c_en <= b_en; d_en <= c_en;
        z_t <= dith; a_t <= z_t; b_t <= a_t; c_t <= b_t; d_t <= c_t;
        z_m <= mask; a_m <= z_m; b_m <= a_m; c_m <= b_m; d_m <= c_m;
    end
    genvar g;
    generate for (g = 0; g < 4; g++) begin : CH                 // g = 0 R, 1 G, 2 B, 3 A
        localparam bit ALPHA = (g == 3);
        reg  [3:0]  n; reg [12:0] one, half; reg [33:0] m34;   // per channel, from the registered format (frame constants)
        always @(posedge clk) begin n <= chn(r_fmt, 2'(g)); one <= one_of(n); half <= {1'b0, one[12:1]}; m34 <= divm(n); end
        wire [12:0] s    = src_c16[(3-g)*13 +: 13];
        wire [7:0]  d    = dst_color[(3-g)*8 +: 8];
        wire [12:0] d16  = {1'b0, d, 4'd0};
        wire [12:0] cc   = blend_c16[(3-g)*13 +: 13];
        wire [12:0] sa   = ALPHA ? s : (g == 1) ? src_alpha[25:13] : src_alpha[12:0];
        wire [12:0] ca   = ALPHA ? cc : (g == 1) ? blend_ca[25:13] : blend_ca[12:0];
        // destination alpha as a factor of this channel, in its units
        wire [12:0] da16 = (n_a == 4'd0) ? one : (n_a == n) ? {1'b0, d_a, 4'd0} : (d_a[0] ? one : 13'd0);
        wire [3:0]  fsc = ALPHA ? afac(src_factor_a) : src_factor;
        wire [3:0]  fdc = ALPHA ? afac(dst_factor_a) : dst_factor;
        wire [2:0]  o   = ALPHA ? op_a : op;
        // A0: the operands registered (the destination comes straight off the band-buffer BRAM)
        reg [12:0] z_s, z_d16, z_sa, z_da16, z_cc, z_ca; reg [7:0] z_d; reg [3:0] z_fsc, z_fdc; reg [2:0] z_o;
        always @(posedge clk) begin
            z_s <= s; z_d16 <= d16; z_d <= d; z_sa <= sa; z_da16 <= da16; z_cc <= cc; z_ca <= ca;
            z_fsc <= fsc; z_fdc <= fdc; z_o <= o;
        end
        // A: factors
        reg [12:0] a_s, a_d16, a_fs, a_fd; reg [2:0] a_o; reg [7:0] a_d;
        always @(posedge clk) begin
            a_s <= z_s; a_d16 <= z_d16; a_d <= z_d; a_o <= z_o;
            a_fs <= fac(z_fsc, z_s, z_d16, z_sa, z_da16, z_cc, z_ca, 1'b0, one);
            a_fd <= fac(z_fdc, z_s, z_d16, z_sa, z_da16, z_cc, z_ca, 1'b1, one);
        end
        // B: products
        reg [25:0] b_p1, b_p2; reg [12:0] b_s, b_d16; reg [2:0] b_o; reg [7:0] b_d;
        always @(posedge clk) begin
            b_p1 <= 26'(a_s) * 26'(a_fs); b_p2 <= 26'(a_d16) * 26'(a_fd);
            b_s <= a_s; b_d16 <= a_d16; b_o <= a_o; b_d <= a_d;
        end
        // C: equation, + half, / 16; MIN / MAX
        wire signed [27:0] x = (b_o == 3'd1) ? $signed({2'b0, b_p1}) - $signed({2'b0, b_p2}) :
                               (b_o == 3'd2) ? $signed({2'b0, b_p2}) - $signed({2'b0, b_p1}) :
                                               $signed({2'b0, b_p1}) + $signed({2'b0, b_p2});
        wire signed [27:0] xh = x + $signed({15'd0, half});
        reg [20:0] c_w; reg c_zero; reg [12:0] c_mm, c_s; reg [2:0] c_o; reg [7:0] c_d;
        always @(posedge clk) begin
            c_zero <= (x <= 28'sd0);
            c_w <= xh[24:4];
            c_mm <= (b_o == 3'd3) ? ((b_s < b_d16) ? b_s : b_d16) : ((b_s > b_d16) ? b_s : b_d16);
            c_o <= b_o; c_s <= b_s; c_d <= b_d;
        end
        // D: / (2^n - 1)
        wire [54:0] p = 55'(c_w) * 55'(m34);
        wire [12:0] q = p[45:33];                            // floor(w / (2^n - 1)) <= 2 ONE + 1
        reg [12:0] d_q, d_mm, d_s; reg d_zero; reg [2:0] d_o; reg [7:0] d_d;
        always @(posedge clk) begin
            d_q <= (q > one) ? one : q;
            d_zero <= c_zero; d_mm <= c_mm; d_o <= c_o; d_s <= c_s; d_d <= c_d;
        end
        // E: select, dither / truncate, mask
        wire [12:0] v = !d_en ? d_s : (d_o == 3'd3 || d_o == 3'd4) ? d_mm : d_zero ? 13'd0 : d_q;
        wire [13:0] u = {1'b0, v} + {10'd0, d_t};
        reg [7:0] e_out;
        always @(posedge clk) e_out <= d_m[3-g] ? u[11:4] : d_d;
        assign final_color[(3-g)*8 +: 8] = e_out;
    end endgenerate
endmodule
`default_nettype wire
