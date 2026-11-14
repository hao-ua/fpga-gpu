// attr_conv — one vertex-attribute component from its client format to FP24 (ES 2.0 §2.8 glVertexAttribPointer types,
//   §2.1.2 the conversions). Pipeline: latency 6, II = 1 (in_valid -> out_valid 6 cycles later; for timing at 120 MHz
//   the input is registered — the feed's constant block RAM -> the byte select -> the type decode and the 48-bit negate
//   would be 19 logic levels — and the converter's normalize is split from its round).
//   ty: 0 BYTE, 1 UNSIGNED_BYTE, 2 SHORT, 3 UNSIGNED_SHORT, 4 FIXED, 5 FLOAT (IEEE 754 binary32), 6 FP24 (the GPU's own
//   format in the low 24 bits of a 32-bit word, taken as is). raw holds the component's bytes in its low bits, little
//   endian (a BYTE in [7:0], a SHORT in [15:0]).
//   * normalized integers (nm = 1; not FIXED / FLOAT / FP24): signed c -> (2c + 1) / (2^b - 1), unsigned c -> c / (2^b - 1)
//     (ES 2.0 §2.1.2 equations 2.2 / 2.3, b = 8 or 16) rounded to nearest even. Exactly: n / (2^b - 1) is the binary fraction
//     0.(n)(n)(n)... with the b-bit block n repeated for ever, so the 48-bit integer {n, n, n} (b = 16) / {n x 6} (b = 8)
//     through the integer -> float converter (RNE at 17 significant bits) and the exponent lowered by 48 is the correctly
//     rounded quotient: the bits below the rounding bit are never all zero (the repeated block n != 0 lies below it), so no
//     tie is ever misjudged, and a rounding bit of 0 leaves the truncation correct whatever follows.
//   * non-normalized integers: the integer exactly (|c| <= 65535 fits the 17-bit significand); FIXED: c / 2^16 (RNE from
//     the 32-bit integer, then the exponent lowered by 16).
//   * FLOAT: round to nearest even to the 16-bit mantissa; denormals and values below 2^-62 flush to (signed) zero as
//     float_ref.encode_real does (the underflow is judged on the unrounded exponent); above the FP24 range -> Inf; Inf -> Inf,
//     NaN -> a quiet NaN with the sign kept.
`default_nettype none
module attr_conv #(
    parameter int EXP_W = 7, MAN_W = 16
)(
    input  wire        clk, rst,
    input  wire        in_valid,
    input  wire [31:0] raw,
    input  wire [2:0]  ty,
    input  wire        nm,
    output reg         out_valid,
    output reg  [EXP_W+MAN_W:0] result
);
    localparam int W = 1 + EXP_W + MAN_W;
    localparam int BIAS = (1 << (EXP_W-1)) - 1;                 // 63
    localparam int IW = 49;                                     // the signed 48-bit magnitude for the integer path
    localparam [2:0] T_BYTE = 3'd0, T_UBYTE = 3'd1, T_SHORT = 3'd2, T_USHORT = 3'd3, T_FIXED = 3'd4, T_FLOAT = 3'd5, T_FP24 = 3'd6;
    // ---- stage P: the input registered
    reg         p_v, p_nm; reg [31:0] raw_p; reg [2:0] ty_p;
    always @(posedge clk) begin p_v <= in_valid & ~rst; raw_p <= raw; ty_p <= ty; p_nm <= nm; end
    // ---- stage A: the integer path's signed magnitude and exponent correction; the FLOAT path in full
    reg  signed [8:0]  n9;  reg signed [16:0] n17;             // 2c + 1 of a normalized signed byte / short
    reg  [7:0]  m8; reg [15:0] m16; reg [31:0] m32;            // |value| per width
    reg  [47:0] mag48; reg sgn; reg [5:0] eadj;
    reg  [W-1:0] fres;                                         // the FLOAT path's result
    always @* begin
        n9  = {raw_p[7:0], 1'b1}; n17 = {raw_p[15:0], 1'b1};
        sgn = 1'b0; m8 = raw_p[7:0]; m16 = raw_p[15:0]; m32 = raw_p; eadj = 6'd0; mag48 = 48'd0;
        case (ty_p)
            T_BYTE: begin
                if (p_nm) begin sgn = n9[8]; m8 = n9[8] ? 8'(-n9) : 8'(n9); mag48 = {6{m8}}; eadj = 6'd48; end
                else    begin sgn = raw_p[7]; m8 = raw_p[7] ? -raw_p[7:0] : raw_p[7:0]; mag48 = {40'd0, m8}; end
            end
            T_UBYTE: begin
                if (p_nm) begin mag48 = {6{raw_p[7:0]}}; eadj = 6'd48; end
                else    mag48 = {40'd0, raw_p[7:0]};
            end
            T_SHORT: begin
                if (p_nm) begin sgn = n17[16]; m16 = n17[16] ? 16'(-n17) : 16'(n17); mag48 = {3{m16}}; eadj = 6'd48; end
                else    begin sgn = raw_p[15]; m16 = raw_p[15] ? -raw_p[15:0] : raw_p[15:0]; mag48 = {32'd0, m16}; end
            end
            T_USHORT: begin
                if (p_nm) begin mag48 = {3{raw_p[15:0]}}; eadj = 6'd48; end
                else    mag48 = {32'd0, raw_p[15:0]};
            end
            default: begin                                      // FIXED: c / 2^16 (the other types take the FLOAT / FP24 paths)
                sgn = raw_p[31]; m32 = raw_p[31] ? -raw_p : raw_p; mag48 = {16'd0, m32}; eadj = 6'd16;
            end
        endcase
    end
    // FLOAT: s | e[7:0] | m[22:0] -> s | e7 | m16 with RNE on m[6:0] (the guard bit m[6], the sticky |m[5:0])
    wire        f_s = raw_p[31]; wire [7:0] f_e = raw_p[30:23]; wire [22:0] f_m = raw_p[22:0];
    wire signed [9:0] f_ex = $signed({2'b00, f_e}) - 10'sd127;  // the unbiased exponent before rounding
    wire        f_g = f_m[6], f_stk = |f_m[5:0];
    wire [17:0] f_mr = {2'b01, f_m[22:7]} + 18'(f_g & (f_stk | f_m[7]));   // 1.m16 + the rounding; bit 17 = carry out
    wire signed [9:0] f_exr = f_ex + 10'(f_mr[17]);
    wire [15:0] f_man = f_mr[17] ? 16'd0 : f_mr[15:0];
    always @* begin
        if (f_e == 8'd0)            fres = {f_s, {(W-1){1'b0}}};                                   // zero / denormal: FTZ
        else if (f_e == 8'hFF)      fres = (f_m == 23'd0) ? {f_s, {EXP_W{1'b1}}, {MAN_W{1'b0}}} : {f_s, {EXP_W{1'b1}}, 1'b1, {(MAN_W-1){1'b0}}};
        else if (f_ex < -10'sd62)   fres = {f_s, {(W-1){1'b0}}};                                   // below the FP24 range (judged unrounded, as encode_real)
        else if (f_exr > 10'sd63)   fres = {f_s, {EXP_W{1'b1}}, {MAN_W{1'b0}}};                      // overflow (after rounding)
        else                        fres = {f_s, EXP_W'(f_exr + 10'sd63), f_man};
    end
    reg a_v; reg signed [IW-1:0] a_ival; reg [5:0] a_eadj; reg [2:0] a_ty; reg [W-1:0] a_fres, a_rres;
    always @(posedge clk) begin
        a_v <= p_v & ~rst;
        a_ival <= sgn ? -$signed({1'b0, mag48}) : $signed({1'b0, mag48});
        a_eadj <= eadj; a_ty <= ty_p; a_fres <= fres; a_rres <= raw_p[W-1:0];
    end
    // ---- the integer -> float converter (3 cycles: SPLIT, the 49-bit normalize apart from the round / pack — in one
    //      stage they are 24 logic levels, too slow at 120 MHz) with the FLOAT / FP24 results carried alongside
    wire i_v; wire [W-1:0] i_r;
    float_cvt_i2f #(.IW(IW), .EXP_W(EXP_W), .MAN_W(MAN_W), .SPLIT(1'b1)) u_i2f (.clk, .rst, .in_valid(a_v), .ival(a_ival), .out_valid(i_v), .result(i_r));
    reg [5:0] b_eadj, c_eadj, d_eadj; reg [2:0] b_ty, c_ty, d_ty; reg [W-1:0] b_fres, c_fres, d_fres, b_rres, c_rres, d_rres;
    always @(posedge clk) begin
        b_eadj <= a_eadj; b_ty <= a_ty; b_fres <= a_fres; b_rres <= a_rres;
        c_eadj <= b_eadj; c_ty <= b_ty; c_fres <= b_fres; c_rres <= b_rres;
        d_eadj <= c_eadj; d_ty <= c_ty; d_fres <= c_fres; d_rres <= c_rres;
    end
    // ---- stage D: the exponent correction (never underflows: the smallest nonzero magnitude is 2^-16 for FIXED and
    //      >= 2^-16 for a normalized value; the converter's zero stays zero), then the type's result
    wire [EXP_W-1:0] i_e = i_r[W-2:MAN_W];
    wire [W-1:0] i_adj = (i_e == '0) ? i_r : {i_r[W-1], EXP_W'(i_e - d_eadj), i_r[MAN_W-1:0]};
    always @(posedge clk) begin
        out_valid <= i_v & ~rst;
        result <= (d_ty == T_FLOAT) ? d_fres : (d_ty == T_FP24) ? d_rres : i_adj;
    end
endmodule
`default_nettype wire
