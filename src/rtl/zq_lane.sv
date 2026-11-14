// zq_lane — conservative depth bounds of a quad (bit-exact model: tools/zq_ref.py zq_quad): from the quad's pixel-0
// edge values e1, e2 (as board A sends them) and its primitive's four
// TRI-item words (zq_tri.sv) -> [dlo, dhi], an interval of the attachment's depth units (24 or 16 bits) that holds the
// depth quad_setup gives every COVERED pixel of the quad (no polygon offset). Used ahead of quad_setup on board B (the
// early depth test).
//   e' = floor(e 2^k) (18 bits, else no bounds), S0 = e1' m1 + e2' m2, s = floor(S0 / 2^14),
//   24 bits: lo = Clo + s - 3, hi = Chi + s + 5; 16 bits: lo = (Clo + s - 1280) >> 8, hi = (Chi + s + 1536) >> 8;
//   dlo, dhi = lo, hi clamped to [0, 2^b - 1].
// Fully pipelined, one quad per cycle, LAT = 6; the words and d16 are taken with the quad and travel with it.
`default_nettype none
module zq_lane #(
    parameter int TAGW = 4
)(
    input  wire             clk, rst,
    input  wire             in_valid,
    input  wire [31:0]      in_e1, in_e2,
    input  wire [127:0]     in_zq,          // {w21 Chi, w20 Clo, w19 m2, w18 {ok, k[30:25], m1[24:0]}}
    input  wire             in_d16,
    input  wire [TAGW-1:0]  in_tag,
    output reg              out_valid,
    output reg              out_ok,         // bounds valid (the primitive's ok, both e' in range)
    output reg  [23:0]      out_dlo, out_dhi,
    output reg  [TAGW-1:0]  out_tag
);
    // floor(e 2^k), k in [-14, 16], and whether it fits 18 bits signed: e must fit 18 - k bits, i.e. its redundant
    // sign bits (lsc) number at least 14 + k
    function automatic [17:0] shk(input [31:0] e, input signed [5:0] k);
        reg [63:0] ev; reg [94:0] sh; reg [5:0] sa;
        ev = {{32{e[31]}}, e}; sa = 6'(k + 6'sd14);
        sh = {31'd0, ev} << sa;
        shk = sh[31:14];
    endfunction
    function automatic [5:0] lsc(input [31:0] e);   // leading bits equal to the sign, minus one (0 .. 31)
        lsc = 6'd31;
        for (int i = 30; i >= 0; i--) if (e[i] != e[31] && lsc == 6'd31) lsc = 6'(30 - i);
    endfunction
    // stage 1: the inputs
    reg v1, v2, v3, v4, v5; reg [TAGW-1:0] t1, t2, t3, t4, t5;
    reg [31:0] e1_1, e2_1; reg signed [5:0] k1; reg signed [24:0] m1_1, m2_1, m1_2, m2_2; reg ok1, ok2, ok3, ok4, ok5;
    reg d1, d2, d3, d4, d5; reg signed [31:0] clo1, clo2, clo3, clo4, chi1, chi2, chi3, chi4;
    // stage 2: e', range; stage 3: the products; stage 4: s; stage 5: lo / hi; stage 6: shifted, clamped
    reg signed [17:0] a2, b2; reg signed [42:0] p3, q3; reg signed [29:0] s4; reg signed [33:0] lo5, hi5;
    always @(posedge clk) begin
        v1 <= in_valid; t1 <= in_tag; e1_1 <= in_e1; e2_1 <= in_e2; k1 <= in_zq[30:25]; m1_1 <= in_zq[24:0]; m2_1 <= in_zq[56:32];
        ok1 <= in_zq[31]; d1 <= in_d16; clo1 <= in_zq[95:64]; chi1 <= in_zq[127:96];
        v2 <= v1; t2 <= t1; a2 <= shk(e1_1, k1); b2 <= shk(e2_1, k1); m1_2 <= m1_1; m2_2 <= m2_1;
        ok2 <= ok1 && (lsc(e1_1) >= 6'(k1 + 6'sd14)) && (lsc(e2_1) >= 6'(k1 + 6'sd14)); d2 <= d1; clo2 <= clo1; chi2 <= chi1;
        v3 <= v2; t3 <= t2; p3 <= a2 * m1_2; q3 <= b2 * m2_2; ok3 <= ok2; d3 <= d2; clo3 <= clo2; chi3 <= chi2;
        v4 <= v3; t4 <= t3; s4 <= 30'((44'(p3) + 44'(q3)) >>> 14); ok4 <= ok3; d4 <= d3; clo4 <= clo3; chi4 <= chi3;
        v5 <= v4; t5 <= t4; ok5 <= ok4; d5 <= d4;
        lo5 <= 34'(clo4) + 34'(s4) - (d4 ? 34'sd1280 : 34'sd3);
        hi5 <= 34'(chi4) + 34'(s4) + (d4 ? 34'sd1536 : 34'sd5);
        out_valid <= v5; out_tag <= t5; out_ok <= ok5;
        begin : CL
            reg signed [33:0] l, h; reg signed [33:0] mx;
            l = d5 ? (lo5 >>> 8) : lo5; h = d5 ? (hi5 >>> 8) : hi5; mx = d5 ? 34'sd65535 : 34'sd16777215;
            out_dlo <= (l < 0) ? 24'd0 : (l > mx) ? 24'(mx) : 24'(l);
            out_dhi <= (h < 0) ? 24'd0 : (h > mx) ? 24'(mx) : 24'(h);
        end
        if (rst) begin v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; v4 <= 1'b0; v5 <= 1'b0; out_valid <= 1'b0; end
    end
endmodule
`default_nettype wire
