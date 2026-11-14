`timescale 1ns/1ps
// tmds_capture — bench-side DVI receiver: samples the four TMDS lanes at the pads (DDR, one bit per
// half period of the serial clock, sampled a quarter period after each edge), finds the 10-bit word
// boundary from the clock lane (10'b0000011111, LSB first: a word starts at a 0 -> 1 transition), and
// decodes each word (DVI 1.0 §3.2: control tokens -> DE = 0, C0/C1 = hsync/vsync on the blue lane; data
// words -> 8 bits). Every decoded pixel toggles `ws`; de / hs / vs / r / g / b hold the word's values.
// `misalign` counts clock-lane transitions found away from a word boundary (expected only while the
// link (re)starts after reset or a mode change); `bad_ctl` counts control tokens on green/red lanes
// that differ from 00 or unknown 10-bit codes during blanking.
//
// Word-level mode (compiled with TMDS_WORD_SIM, `tools/sim_a.ps1 -Fast`): the serializers are stubs, so the
// words are taken from the encoders (w_b / w_g / w_r) at each pixel clock edge and decoded the same way.
// Everything after the encoders (OSERDES, bit order, word alignment) is then not covered; the bit-level
// default runs cover it. Several times faster for the long full-mode runs (+FULL).
`default_nettype none
module tmds_capture (
    input  wire       ser_clk,
    input  wire       lane_clk,
    input  wire [2:0] lane_d,               // 0 = blue, 1 = green, 2 = red
    input  wire       pix_clk,              // word-level mode only: pixel clock and the encoder outputs
    input  wire [9:0] w_b, w_g, w_r,
    output reg        ws = 1'b0,
    output reg        de = 1'b0, hs = 1'b0, vs = 1'b0,
    output reg  [7:0] r = 0, g = 0, b = 0,
    output int        misalign = 0,
    output int        bad_ctl = 0,
    output longint    words = 0
);
    realtime t_last = 0.0, per = 0.0;
`ifndef TMDS_WORD_SIM
    always @(posedge ser_clk) begin per = $realtime - t_last; t_last = $realtime; end
`endif

    reg [9:0] sh_c = 0, sh_0 = 0, sh_1 = 0, sh_2 = 0;
    int       n = 0;                        // bits collected in the current word
    reg       prev_c = 1'b0;

    function automatic [9:0] ctl_code(input [1:0] c);
        case (c) 2'b00: ctl_code = 10'b1101010100; 2'b01: ctl_code = 10'b0010101011;
                 2'b10: ctl_code = 10'b0101010100; default: ctl_code = 10'b1010101011; endcase
    endfunction
    function automatic int ctl_of(input [9:0] q);   // -1 = data word
        ctl_of = -1;
        for (int c = 0; c < 4; c++) if (q == ctl_code(c[1:0])) ctl_of = c;
    endfunction
    function automatic [7:0] dec(input [9:0] q);
        reg [7:0] d;
        d = q[9] ? ~q[7:0] : q[7:0];
        dec[0] = d[0];
        for (int i = 1; i < 8; i++) dec[i] = q[8] ? (d[i] ^ d[i-1]) : ~(d[i] ^ d[i-1]);
    endfunction

    task automatic word_in(input [9:0] q0, input [9:0] q1, input [9:0] q2);
        int c0, c1, c2;
        c0 = ctl_of(q0); c1 = ctl_of(q1); c2 = ctl_of(q2);
        if (c0 >= 0) begin
            de = 1'b0; hs = c0[0]; vs = c0[1];
            if (c1 != 0 || c2 != 0) bad_ctl++;
        end else begin
            de = 1'b1; b = dec(q0); g = dec(q1); r = dec(q2);
            if (c1 >= 0 || c2 >= 0) bad_ctl++;
        end
        words++;
        ws = ~ws;
    endtask

    task automatic bit_in(input bc, input [2:0] bd);
        // a word begins with the clock lane's first 1 after its zeros
        if (bc && !prev_c && n != 0) begin
            if (n != 10) misalign++;
            n = 0;
        end
        prev_c = bc;
        sh_c = {bc, sh_c[9:1]}; sh_0 = {bd[0], sh_0[9:1]}; sh_1 = {bd[1], sh_1[9:1]}; sh_2 = {bd[2], sh_2[9:1]};
        n++;
        if (n == 10) begin
            n = 0;
            word_in(sh_0, sh_1, sh_2);
        end
    endtask

`ifdef TMDS_WORD_SIM
    // the encoder outputs are registers on pix_clk: this block reads the values of the word that just ended
    always @(posedge pix_clk)
        if ((^{w_b, w_g, w_r}) !== 1'bx) word_in(w_b, w_g, w_r);
`else
    always @(ser_clk) begin
        if (per > 0.0) begin
            #(per / 4.0);
            if ((^{lane_clk, lane_d}) !== 1'bx) bit_in(lane_clk, lane_d);
        end
    end
`endif
endmodule
`default_nettype wire
