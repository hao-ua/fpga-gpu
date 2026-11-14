// cube_rcp — the cube-map reciprocal of tmu24: R = floor(2^(M+26) / A) for A = {1, mantissa[M-1:0]} (M = 16: A in
// [2^16, 2^17), R in (2^25, 2^26]), EXACT, as a fixed-latency pipeline with one input per cycle.
//   s0  a registered; ROM index = a[15:6]
//   s1  y0 = ROM[idx] ≈ 2^33 / a (17 bits; the interval midpoint, relative error <= 2^-11)
//   s2  p0 = a * y0                                            (DSP)
//   s3  d0 = p0 - 2^33                                         (|d0| < 2^23: half an interval + the seed's rounding)
//   s4  m0 = y0 * (d0 >>> 6)                                   (DSP, 17 x 18 signed)
//   s5  y1 = (y0 << 7) - (m0 >>> 20) ≈ 2^40 / a                (24 bits; relative error <= 2^-21.4)
//   s6  p1 = a * y1                                            (DSP, 17 x 24)
//   s7  d1 = p1 - 2^40                                         (|d1| < 2^19)
//   s8  m1 = y1 * (d1 >>> 3)                                   (DSP, 24 x 18 signed)
//   s9  c  = (y1 << 2) - (m1 >>> 35) ≈ 2^42 / a                (27 bits; c in {R, R + 1, R + 2}: every floor above
//                                                               raises c, their sum < 1.001, Newton's own term < 2^-16)
//   s10 pc = a * c                                             (DSP x 2: a * c[17:0] + (a * c[26:18]) << 18)
//   s11 rem = 2^42 - pc; lt0 = rem < 0, lt1 = rem < -a, gea = rem >= a
//   s12 R = c - (lt1 ? 2 : lt0 ? 1 : 0) + gea; A = 2^16 exactly -> R = 2^26
// 13 registers from a to r (s0 = a0 .. s12 = r12): tmu24 aligns r with its side data at dv_v[12] (dv_v[0] = a0's edge).
// Verified exhaustively over all 2^16 mantissas by sim/tb_cube_rcp.sv.
`default_nettype none
module cube_rcp #(
    parameter int M = 16
)(
    input  wire        clk,
    input  wire [M:0]  a,                        // {1, mantissa}
    output wire [26:0] r
);
    localparam int LAT = 12;
    // ---- the seed ROM: y0[i] = round(2^33 / (2^16 + 64 i + 32)), 1024 x 17 bits (block or LUT ROM, registered read)
    reg [16:0] rom [0:1023];
    initial begin
        for (int i = 0; i < 1024; i++) begin
            longint den; den = 64'd65536 + 64'(i) * 64 + 32;
            rom[i] = 17'((64'd8589934592 + den / 2) / den);       // 2^33 = 8589934592
        end
    end
    // ---- s0
    reg [16:0] a0; reg [9:0] i0;
    always @(posedge clk) begin a0 <= a; i0 <= a[15:6]; end
    // ---- s1: the seed
    reg [16:0] y0, a1;
    always @(posedge clk) begin y0 <= rom[i0]; a1 <= a0; end
    // ---- s2: p0 = a * y0
    reg [33:0] p0; reg [16:0] y0_2, a2;
    always @(posedge clk) begin p0 <= a1 * y0; y0_2 <= y0; a2 <= a1; end
    // ---- s3: d0 = p0 - 2^33 (signed; |d0| <= 2^33 * 32 / 65632 + 0.5 * A < 4.26 M < 2^23: the interval half-width
    //      plus the seed's rounding — a 23-bit register overflows at A = 65600)
    reg signed [23:0] d0; reg [16:0] y0_3, a3;
    always @(posedge clk) begin d0 <= 24'($signed({1'b0, p0}) - 35'sd8589934592); y0_3 <= y0_2; a3 <= a2; end
    // ---- s4: m0 = y0 * (d0 >>> 6)   (|d0 / 64| < 2^17: an 18-bit signed multiplier input)
    reg signed [35:0] m0; reg [16:0] y0_4, a4;
    always @(posedge clk) begin m0 <= $signed({1'b0, y0_3}) * $signed(18'(d0 >>> 6)); y0_4 <= y0_3; a4 <= a3; end
    // ---- s5: y1 = (y0 << 7) - (m0 >>> 20)  (= y0 (2^33 - d0) / 2^26; 24 bits for a > 2^16; saturated for safety)
    reg [23:0] y1; reg [16:0] a5;
    always @(posedge clk) begin : s5
        reg signed [25:0] t;
        t = $signed({2'b0, y0_4, 7'd0}) - 26'($signed(m0 >>> 20));
        y1 <= (t < 0) ? 24'd0 : (t > 26'sd16777215) ? 24'hFFFFFF : 24'(t);
        a5 <= a4;
    end
    // ---- s6: p1 = a * y1
    reg [40:0] p1; reg [23:0] y1_6; reg [16:0] a6;
    always @(posedge clk) begin p1 <= a5 * y1; y1_6 <= y1; a6 <= a5; end
    // ---- s7: d1 = p1 - 2^40 (signed, |d1| < 2^19)
    reg signed [20:0] d1; reg [23:0] y1_7; reg [16:0] a7;
    always @(posedge clk) begin d1 <= 21'($signed({1'b0, p1}) - 42'sd1099511627776); y1_7 <= y1_6; a7 <= a6; end
    // ---- s8: m1 = y1 * (d1 >>> 3)
    reg signed [42:0] m1; reg [23:0] y1_8; reg [16:0] a8;
    always @(posedge clk) begin m1 <= $signed({1'b0, y1_7}) * $signed(18'(d1 >>> 3)); y1_8 <= y1_7; a8 <= a7; end
    // ---- s9: c = (y1 << 2) - (m1 >>> 35)
    reg [26:0] c; reg [16:0] a9;
    always @(posedge clk) begin : s9
        reg signed [28:0] t;
        t = $signed({3'b0, y1_8, 2'b0}) - 29'($signed(m1 >>> 35));
        c <= (t < 0) ? 27'd0 : 27'(t);
        a9 <= a8;
    end
    // ---- s10: pc = a * c (two products)
    reg [34:0] pcl; reg [25:0] pch; reg [26:0] c10; reg [16:0] a10;
    always @(posedge clk) begin pcl <= a9 * c[17:0]; pch <= a9 * c[26:18]; c10 <= c; a10 <= a9; end
    // ---- s11: rem = 2^42 - pc; the correction flags. c >= R always (every rounding above went the same way) and the
    //      floors add up to < 1.001, so c in {R, R + 1, R + 2}: rem < 0 -> one down, rem < -a -> two down
    reg lt0, lt1, gea; reg [26:0] c11; reg zero11;
    always @(posedge clk) begin : s11
        reg signed [44:0] rem;
        rem = 45'sd4398046511104 - $signed({1'b0, 44'(pcl) + (44'(pch) << 18)});   // 2^42
        lt0 <= (rem < 0);
        lt1 <= (rem < -$signed({28'd0, a10}));
        gea <= (rem >= $signed({28'd0, a10}));
        c11 <= c10; zero11 <= (a10[15:0] == 16'd0);
    end
    // ---- s12: the exact quotient
    reg [26:0] r12;
    always @(posedge clk) r12 <= zero11 ? 27'd67108864 : (c11 - (lt1 ? 27'd2 : lt0 ? 27'd1 : 27'd0) + 27'(gea));   // 2^26 for A = 2^16
    assign r = r12;
endmodule
`default_nettype wire
