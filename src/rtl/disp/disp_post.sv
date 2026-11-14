// disp_post — board B pixel post-processing between the line buffer and the TMDS encoders: palette lookup for P8
// pixels and the 64x64 hardware cursor.
//   * Line-buffer words (scan_reader): alpha 0xFF = direct colour 0xRRGGBBFF; alpha 0x00 / 0x01 = P8 index in
//     bits [31:24], looked up in palette bank alpha[0] (2 x 256 x 24 bit, written in wclk, read here).
//   * Cursor: two shape banks of 4096 words (written in wclk). Mode 0 colour 0xAARRGGBB, straight alpha:
//     out = round((c * a + p * (255 - a)) / 255); mode 1 AND / XOR: bit 24 = AND, [23:0] XOR colour:
//     out = (AND ? p : 0) ^ XOR; mode 2 premultiplied: out = min(255, c + round(p * (255 - a) / 255)).
//     The cursor's top-left is (x - hot_x, y - hot_y), clipped at the screen edges.
//   * Cursor state comes from wclk through a mailbox: mb_state is stable while mb_req != mb_ack; at the start of
//     vsync this side copies it if mb_req (synchronised) differs from mb_ack, then toggles mb_ack. So the cursor
//     changes only between frames. The pixel-domain reset turns the cursor off and clears mb_ack (the wclk side
//     resends its state after a mode change).
// Timing: in_de / in_hs / in_vs / in_x / in_y at cycle N (video_timing), in_px at N + 1 (line-buffer read);
// out_* at N + 5. /255 by Blinn's exact division (x + 128 + ((x + 128) >> 8)) >> 8 for 0 <= x <= 255 * 255.
`default_nettype none
module disp_post (
    // write side (clk_sys)
    input  wire        wclk,
    input  wire        pal_we,
    input  wire [8:0]  pal_wa,                    // {bank, index}
    input  wire [23:0] pal_wd,                    // RGB
    input  wire        cs_we,
    input  wire [12:0] cs_wa,                     // {bank, row[5:0], column[5:0]}
    input  wire [31:0] cs_wd,
    input  wire        mb_req,                    // toggle (wclk)
    input  wire [47:0] mb_state,                  // {en, mode[1:0], bank, hot_x[5:0], hot_y[5:0], x s16, y s16}
    output reg         mb_ack,                    // toggle (pclk)
    // pixel domain
    input  wire        pclk, prst,
    input  wire        vsync_start,               // pulse
    input  wire        in_de, in_hs, in_vs,       // active flags at cycle N
    input  wire [11:0] in_x, in_y,
    input  wire [31:0] in_px,                     // cycle N + 1
    output reg         out_de, out_hs, out_vs,    // cycle N + 5
    output reg  [23:0] out_rgb
);
    // ---------------- memories ----------------
    (* ram_style = "block" *) reg [23:0] pal  [0:511];
    (* ram_style = "block" *) reg [31:0] cram [0:8191];
    always @(posedge wclk) begin
        if (pal_we) pal[pal_wa] <= pal_wd;
        if (cs_we)  cram[cs_wa] <= cs_wd;
    end

    // ---------------- mailbox ----------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] req_s = 2'b00;
    always @(posedge pclk) req_s <= {req_s[0], mb_req};
    reg         c_en, c_bank; reg [1:0] c_mode;
    reg  signed [17:0] c_ox, c_oy;                // cursor top-left on the screen
    always @(posedge pclk) begin
        if (prst) begin c_en <= 1'b0; mb_ack <= 1'b0; end
        else if (vsync_start && req_s[1] != mb_ack) begin
            c_en <= mb_state[47]; c_mode <= mb_state[46:45]; c_bank <= mb_state[44];
            c_ox <= $signed({{2{mb_state[31]}}, mb_state[31:16]}) - $signed({12'h0, mb_state[43:38]});
            c_oy <= $signed({{2{mb_state[15]}}, mb_state[15:0]})  - $signed({12'h0, mb_state[37:32]});
            mb_ack <= ~mb_ack;
        end
    end

    // ---------------- N -> N+1: cursor address ----------------
    wire signed [17:0] cx = $signed({6'h0, in_x}) - c_ox;
    wire signed [17:0] cy = $signed({6'h0, in_y}) - c_oy;
    wire        in_cur = c_en && in_de && cx >= 0 && cx < 64 && cy >= 0 && cy < 64;
    reg         s1_de, s1_hs, s1_vs, s1_cur; reg [12:0] s1_ca;
    always @(posedge pclk) begin
        s1_de <= in_de; s1_hs <= in_hs; s1_vs <= in_vs; s1_cur <= in_cur;
        s1_ca <= {c_bank, cy[5:0], cx[5:0]};
    end
    // ---------------- N+1 -> N+2: palette and cursor reads ----------------
    reg  [23:0] pal_q;
    reg  [31:0] cur_q;
    reg         s2_de, s2_hs, s2_vs, s2_cur, s2_p8; reg [23:0] s2_dc; reg [1:0] s2_mode;
    wire        is_p8 = (in_px[7:1] == 7'h00);
    always @(posedge pclk) begin
        pal_q <= pal[{in_px[0], in_px[31:24]}];
        cur_q <= cram[s1_ca];
        s2_de <= s1_de; s2_hs <= s1_hs; s2_vs <= s1_vs; s2_cur <= s1_cur; s2_p8 <= is_p8; s2_dc <= in_px[31:8];
        s2_mode <= c_mode;
    end
    // ---------------- N+2 -> N+3: base pixel, cursor word ----------------
    reg         s3_de, s3_hs, s3_vs, s3_cur; reg [1:0] s3_mode; reg [23:0] s3_p; reg [31:0] s3_c;
    always @(posedge pclk) begin
        s3_de <= s2_de; s3_hs <= s2_hs; s3_vs <= s2_vs; s3_cur <= s2_cur; s3_mode <= s2_mode;
        s3_p <= s2_p8 ? pal_q : s2_dc; s3_c <= cur_q;
    end
    // ---------------- N+3 -> N+4: products ----------------
    wire [7:0]  a  = s3_c[31:24], ia = 8'd255 - s3_c[31:24];
    reg         s4_de, s4_hs, s4_vs, s4_cur; reg [1:0] s4_mode; reg [23:0] s4_p, s4_x; reg [31:0] s4_c;
    reg  [15:0] s4_ca [0:2], s4_pa [0:2];
    integer     ch;
    always @(posedge pclk) begin
        s4_de <= s3_de; s4_hs <= s3_hs; s4_vs <= s3_vs; s4_cur <= s3_cur; s4_mode <= s3_mode; s4_p <= s3_p; s4_c <= s3_c;
        s4_x <= (s3_c[24] ? s3_p : 24'h0) ^ s3_c[23:0];                 // AND / XOR
        for (ch = 0; ch < 3; ch = ch + 1) begin
            s4_ca[ch] <= s3_c[ch*8 +: 8] * a;
            s4_pa[ch] <= s3_p[ch*8 +: 8] * ia;
        end
    end
    // ---------------- N+4 -> N+5: blend, output ----------------
    function automatic [7:0] div255(input [16:0] x);                  // round(x / 255), x <= 65025
        reg [16:0] t; t = x + 17'd128; div255 = 8'((t + (t >> 8)) >> 8);
    endfunction
    reg  [23:0] blend, premul;
    integer     cq;
    always @(*) begin
        for (cq = 0; cq < 3; cq = cq + 1) begin : chan
            reg [8:0] s;
            blend[cq*8 +: 8] = div255({1'b0, s4_ca[cq]} + {1'b0, s4_pa[cq]});
            s = {1'b0, s4_c[cq*8 +: 8]} + {1'b0, div255({1'b0, s4_pa[cq]})};
            premul[cq*8 +: 8] = s[8] ? 8'hFF : s[7:0];
        end
    end
    always @(posedge pclk) begin
        out_de <= s4_de; out_hs <= s4_hs; out_vs <= s4_vs;
        out_rgb <= !s4_de ? 24'h0 : !s4_cur ? s4_p : (s4_mode == 2'd1) ? s4_x : (s4_mode == 2'd2) ? premul : blend;
    end
endmodule
`default_nettype wire
