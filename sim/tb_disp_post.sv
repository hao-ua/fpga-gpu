`timescale 1ns/1ps
// tb_disp_post — unit bench of disp_post (palette lookup + hardware cursor).
// video_timing drives a 100 x 40 mode; the "line buffer" is a hash of (frame, x, y): direct colour words
// (alpha 0xFF) and P8 words {index, 0, 0, bank} with the frame's bank. Every output pixel and the sync
// alignment (5 cycles) are compared with a model:
//   * palettes: random contents; during a frame only the bank not used by that frame is rewritten;
//   * cursor: random 64 x 64 shapes in the hidden bank, modes 0 (straight alpha, real-number rounding),
//     1 (AND / XOR), 2 (premultiplied, saturating); random positions and hotspots including every clipping edge;
//   * mailbox: state changes requested mid-frame take effect at the next vsync (model), several in a row,
//     and after a pixel-domain reset the cursor is off until the mailbox is applied again.
`default_nettype none
module tb_disp_post;
    reg wclk = 0, pclk = 0;
    always #5.0 wclk = ~wclk;
    always #12.5 pclk = ~pclk;                    // 40 MHz
    int errors = 0, nprint = 0;
    task automatic report(input string s);
        errors++;
        if (nprint < 30) begin $display("%t FAIL %s", $time, s); nprint++; end
    endtask

    // ---------------- video timing ----------------
    reg prst = 1;
    wire hs_raw, vs_raw, de; wire [11:0] x, y, hc, vc;
    video_timing u_vt (.pixel_clk(pclk), .rst_n(~prst), .h_active(12'd100), .h_fp(12'd4), .h_sync_w(12'd4), .h_bp(12'd6),
        .v_active(12'd40), .v_fp(12'd2), .v_sync_w(12'd2), .v_bp(12'd3), .hsync_pol(1'b1), .vsync_pol(1'b1),
        .h_count(hc), .v_count(vc), .hsync(hs_raw), .vsync(vs_raw), .de(de), .x(x), .y(y));
    reg vs_q = 0;
    always @(posedge pclk) vs_q <= vs_raw;
    wire vsync_start = vs_raw & ~vs_q;

    // ---------------- line-buffer model ----------------
    int frame = 0;
    bit frame_bank = 0;                            // bank of this frame's P8 pixels
    function automatic bit [31:0] lbword(input int f, input bit bank, input int xx, input int yy);
        bit [31:0] h;
        h = (f * 32'h9E37_79B9) ^ (xx * 32'h85EB_CA6B) ^ (yy * 32'hC2B2_AE35);
        h = h ^ (h >> 13); h = h * 32'h27D4_EB2F; h = h ^ (h >> 15);
        if (h[3:0] < 6) return {h[31:24], 16'h0, 7'h0, bank};      // P8
        return {h[31:8], 8'hFF};
    endfunction
    reg [31:0] in_px = 0;
    always @(posedge pclk) in_px <= lbword(frame, frame_bank, x, y);

    // ---------------- DUT ----------------
    reg         pal_we = 0, cs_we = 0, mb_req = 0; reg [8:0] pal_wa = 0; reg [23:0] pal_wd = 0;
    reg  [12:0] cs_wa = 0; reg [31:0] cs_wd = 0; reg [47:0] mb_state = 0;
    wire        mb_ack, out_de, out_hs, out_vs; wire [23:0] out_rgb;
    disp_post dut (.wclk(wclk), .pal_we(pal_we), .pal_wa(pal_wa), .pal_wd(pal_wd), .cs_we(cs_we), .cs_wa(cs_wa),
        .cs_wd(cs_wd), .mb_req(mb_req), .mb_state(mb_state), .mb_ack(mb_ack),
        .pclk(pclk), .prst(prst), .vsync_start(vsync_start), .in_de(de), .in_hs(hs_raw), .in_vs(vs_raw),
        .in_x(x), .in_y(y), .in_px(in_px), .out_de(out_de), .out_hs(out_hs), .out_vs(out_vs), .out_rgb(out_rgb));

    // ---------------- model state ----------------
    bit [23:0] pal_m [0:511];
    bit [31:0] cram_m [0:8191];
    typedef struct { bit en; int mode; bit bank; int hx, hy, x, y; } cur_t;
    cur_t act_m, sent_m;
    bit   pend_m = 0;                              // a mailbox request the DUT applies at the next vsync
    bit   ack_m = 0;
    function automatic int rnd(input real f); return $rtoi(f + 0.5); endfunction
    function automatic bit [23:0] expect_rgb(input int xx, input int yy);
        bit [31:0] w, c; bit [23:0] p, o; int cx, cy;
        w = lbword(frame, frame_bank, xx, yy);
        p = (w[7:0] == 8'hFF) ? w[31:8] : pal_m[{w[0], w[31:24]}];
        cx = xx - (act_m.x - act_m.hx); cy = yy - (act_m.y - act_m.hy);
        if (!act_m.en || cx < 0 || cx >= 64 || cy < 0 || cy >= 64) return p;
        c = cram_m[{act_m.bank, 6'(cy), 6'(cx)}];
        for (int ch = 0; ch < 3; ch++) begin
            int cc, pp, a, v;
            cc = c[ch*8 +: 8]; pp = p[ch*8 +: 8]; a = c[31:24];
            case (act_m.mode)
                1: v = ((c[24] ? pp : 0) ^ cc);
                2: begin v = cc + rnd(pp * (255 - a) / 255.0); if (v > 255) v = 255; end
                default: v = rnd((cc * a + pp * (255 - a)) / 255.0);
            endcase
            o[ch*8 +: 8] = 8'(v);
        end
        return o;
    endfunction

    // pixel-domain model: mailbox at vsync, frame counter, expected-output delay line (5 cycles)
    typedef struct { bit de, hs, vs; bit [23:0] rgb; } exp_t;
    exp_t dl [0:4];
    int   n_px = 0, n_cur_px = 0, n_frames = 0;
    bit   req_seen [0:1];
    initial for (int i = 0; i < 5; i++) dl[i] = '{0, 0, 0, 0};
    always @(posedge pclk) begin
        exp_t e;
        // compare the output with the entry made 5 cycles ago
        e = dl[4];
        if (!prst) begin
            if (out_de !== e.de || out_hs !== e.hs || out_vs !== e.vs)
                report($sformatf("sync de/hs/vs %b%b%b expected %b%b%b", out_de, out_hs, out_vs, e.de, e.hs, e.vs));
            else if (e.de && out_rgb !== e.rgb) report($sformatf("frame %0d pixel %06h expected %06h", frame, out_rgb, e.rgb));
            if (e.de) n_px++;
        end
        for (int i = 4; i > 0; i--) dl[i] = dl[i - 1];
        if (prst) begin
            for (int i = 0; i < 5; i++) dl[i] = '{0, 0, 0, 0};
            act_m.en = 0; ack_m = 0;
        end else begin
            dl[0] = '{de, hs_raw, vs_raw, de ? expect_rgb(x, y) : 24'h0};
            if (de && act_m.en) begin
                int cx, cy; cx = x - (act_m.x - act_m.hx); cy = y - (act_m.y - act_m.hy);
                if (cx >= 0 && cx < 64 && cy >= 0 && cy < 64) n_cur_px++;
            end
            if (vsync_start) begin
                if (mb_req != ack_m) begin act_m = sent_m; ack_m = ~ack_m; end
                frame++; n_frames++;
                frame_bank = $urandom % 2;
            end
        end
    end

    // ---------------- wclk-side stimulus ----------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] ack_s = 0;
    always @(posedge wclk) ack_s <= {ack_s[0], mb_ack};
    task automatic wpal(input int a, input bit [23:0] v);
        @(posedge wclk); pal_we <= 1; pal_wa <= 9'(a); pal_wd <= v; pal_m[a] = v;
        @(posedge wclk); pal_we <= 0;
    endtask
    task automatic wshape(input bit bank, input int mode);
        for (int i = 0; i < 4096; i++) begin
            bit [31:0] v; v = $urandom;
            if (mode == 0 || mode == 2) case ($urandom % 4) 0: v[31:24] = 0; 1: v[31:24] = 255; default: ; endcase
            if (mode == 2) for (int ch = 0; ch < 3; ch++) if (v[ch*8 +: 8] > v[31:24]) v[ch*8 +: 8] = v[31:24];  // premultiplied
            @(posedge wclk); cs_we <= 1; cs_wa <= {bank, 12'(i)}; cs_wd <= v; cram_m[{bank, 12'(i)}] = v;
        end
        @(posedge wclk); cs_we <= 0;
    endtask
    // wait until the pixel side is in the middle of a frame (so a request is applied at the next vsync)
    task automatic mid_frame();
        @(posedge pclk); while (!(de && y >= 12'd8 && y <= 12'd30)) @(posedge pclk);
        @(posedge wclk);
    endtask
    task automatic send(input cur_t c);
        while (ack_s[1] != mb_req) @(posedge wclk);                    // previous one applied
        mid_frame();
        mb_state <= {c.en, 2'(c.mode), c.bank, 6'(c.hx), 6'(c.hy), 16'(c.x), 16'(c.y)};
        sent_m = c;
        @(posedge wclk); mb_req <= ~mb_req;
    endtask
    task automatic wait_frames(input int n);
        int f0; f0 = n_frames;
        while (n_frames < f0 + n) @(posedge pclk);
    endtask

    initial begin
        cur_t c;
        bit shown_bank;
        for (int i = 0; i < 512; i++) wpal(i, $urandom);
        wshape(0, 0);
        repeat (10) @(posedge pclk); prst <= 0;
        wait_frames(2);
        $display("ok: cursor off, palettes, %0d pixels", n_px);
        shown_bank = 0;
        for (int n = 0; n < 60; n++) begin
            int mode;
            mode = n % 3;
            // new shape in the hidden bank, then show it
            wshape(~shown_bank, mode);
            c.en = 1; c.mode = mode; c.bank = ~shown_bank; c.hx = $urandom % 64; c.hy = $urandom % 64;
            case ($urandom % 5)
                0: begin c.x = -($urandom % 80); c.y = $urandom % 60; end                       // off the left
                1: begin c.x = 60 + $urandom % 70; c.y = $urandom % 60; end                     // off the right
                2: begin c.x = $urandom % 120; c.y = -($urandom % 80); end                      // off the top
                3: begin c.x = $urandom % 120; c.y = 20 + $urandom % 70; end                    // off the bottom
                default: begin c.x = $urandom % 100; c.y = $urandom % 40; end
            endcase
            send(c); shown_bank = c.bank;
            wait_frames(2);
            // several moves, palette rewrites of the bank the frame does not use
            for (int m = 0; m < 3; m++) begin
                c.x = c.x + int'($urandom % 41) - 20; c.y = c.y + int'($urandom % 21) - 10;
                send(c);
                mid_frame();
                for (int k = 0; k < 8; k++) wpal({~frame_bank, 8'($urandom)}, $urandom);
                wait_frames(1);
            end
            if (n % 10 == 9) begin c.en = 0; send(c); wait_frames(2); c.en = 1; send(c); end
            if (n == 30) begin                                             // pixel-domain reset
                @(posedge pclk); prst <= 1; repeat (20) @(posedge pclk); prst <= 0;
                wait_frames(3);
            end
        end
        wait_frames(2);
        if (n_cur_px < 20000) report($sformatf("cursor coverage too low: %0d pixels", n_cur_px));
        $display("INFO_TB frames %0d, pixels %0d, cursor pixels %0d", n_frames, n_px, n_cur_px);
        if (errors == 0) $display("TB_PASS tb_disp_post");
        else $display("TB_FAIL tb_disp_post errors=%0d", errors);
        $finish;
    end
    initial begin #(500_000_000.0); $display("TB_FAIL tb_disp_post timeout"); $finish; end
endmodule
`default_nettype wire
