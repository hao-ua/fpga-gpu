// b_bringup — board B bring-up harness, beside the command processor (fb_cmd_b):
//   * fills a set of test images through xfer_lin (one OP_WRITE per memory row) and writes their
//     descriptors into img_table (entries 0..NIMG-1), then presents image 2 x mode;
//   * KEY1: short press = present the next image, long press = next video mode (0..3), then present
//     image 2 x mode (image 8 for mode 4 = 1024 x 768 and the simulation modes, mode_img);
//   * JTAG USER2 commands (jtag_ctrl_reg checks signature + scan length, tools/b_jtag.tcl), 64 bits {op[63:56], args}:
//       0x01 SET_MODE  [2:0] mode id
//       0x02 PRESENT   [9:0] image table entry, [16] at once (else at vsync)
//       0x03 MEMTEST   [55:46] start MB, [45:36] length MB (1..1023), [31:0] seed: write + verify;
//                      destroys images in the range (REFILL afterwards)
//       0x04 REFILL    regenerate every test image
//       0x05 KEY       [0] 0 = short press, 1 = long press
//       0x06 LINKCTL   [31:0] board link control (executed by board_b_core; b0 = fill words)
// Test image (display coordinates x, y of a w x h image): 1-pixel white border; 16x16 corner markers
// inset by 2 pixels (top-left red, top-right green, bottom-left blue, bottom-right yellow); image index
// + 1 black 8x8 squares at y 24..31, x 24 + 16j (visible on the bars); top half 8 colour bars (white, yellow, cyan, green,
// magenta, red, blue, black; bar j starts at j * (w >> 3)); next quarter a grid (grey 0x606060 every
// 32 pixels, 0xC0C0C0 every 128) on black; bottom quarter a grey ramp (x mod 256). An image with origin 1
// is stored bottom row first, so every correctly presented image looks the same way up.
// IMG_SET 0 = hardware set, 1 = small simulation set (see img_geom).
// Shares the display, xfer_lin and the image table write port with fb_cmd_b: it starts no command while
// ext_busy (key presses during a host command are ignored; a JTAG command waits), and fb_cmd_b starts none while
// this sequencer is `active`. Its mode_busy input includes the command processor's busy.
`default_nettype none
module b_bringup
    import img_pkg::*;
#(
    parameter int IMG_SET    = 0,
    parameter int DEBOUNCE   = 2_000_000,        // 20 ms
    parameter int LONG_PRESS = 60_000_000        // 600 ms
)(
    input  wire         clk, rst,
    input  wire         ext_busy,                // the command processor is working: start nothing new
    input  wire         key1_n,
    input  wire         cmd_valid,
    input  wire [63:0]  cmd,
    // image table
    output reg          it_we,
    output reg  [9:0]   it_widx,
    output img_desc_t   it_wdesc,
    output reg  [9:0]   it_ridx,
    input  wire         it_rvalid,
    input  img_desc_t   it_rdesc,
    // xfer_lin
    output reg          x_cmd_valid,
    output reg  [1:0]   x_op,
    output reg  [29:0]  x_addr, x_len,
    output reg  [31:0]  x_seed,
    output reg  [1:0]   x_flags,
    input  wire         x_cmd_ready,
    output reg          x_wd_valid,
    output reg  [31:0]  x_wd_data,
    input  wire         x_wd_ready,
    input  wire         x_mt_done,
    input  wire [31:0]  x_mt_errors,
    input  wire [29:0]  x_mt_first_bad,
    // display
    output reg          set_mode,
    output reg  [2:0]   set_mode_id,
    input  wire         mode_busy, mode_done, mode_ok,
    input  wire [2:0]   cur_mode,
    output reg          pres_valid,
    output reg  [29:0]  pres_base,
    output reg  [15:0]  pres_pitch,
    output reg  [12:0]  pres_w, pres_h,
    output reg          pres_565, pres_origin, pres_now,
    output reg  [31:0]  pres_tag,
    // status
    output reg          fill_done,
    output wire         busy,
    output wire         active,                  // the sequencer is executing (not waiting in its idle state)
    output wire         key_down,
    output reg  [9:0]   cur_img,
    output reg  [31:0]  st_cmds, st_errs, st_fills, st_mt_runs, st_mt_errors, st_keys,
    output reg  [29:0]  st_mt_first_bad,
    output reg  [63:0]  st_last_cmd
);
    localparam int NIMG = 10;
    localparam [29:0] IMG_BASE0 = 30'h1000_0000, IMG_STRIDE = 30'h0080_0000;    // 256 MB + i x 8 MB

    // the image presented for a mode (at boot, after a long press): 2 x mode for the four modes with a picture of their own,
    // image 8 (203 x 151, a window) for mode 4 (1024 x 768) and the simulation modes 5 / 6
    function automatic [9:0] mode_img(input [2:0] m);
        mode_img = (m >= 3'd4) ? 10'd8 : {6'h0, m, 1'b0};
    endfunction

    // ---------------- image set: {w, h, 565, origin, pitch} ----------------
    function automatic [41:0] img_geom(input int set, input int i);
        reg [11:0] w, h; reg f, o; reg [15:0] p;
        if (set == 0) begin
            case (i)
                0, 1: begin w = 640;  h = 480; end
                2, 3: begin w = 800;  h = 600; end
                4, 5: begin w = 1280; h = 720; end
                6, 7: begin w = 1280; h = 800; end
                8:    begin w = 203;  h = 151; end
                default: begin w = 1920; h = 1080; end
            endcase
            f = (i == 9) ? 1'b1 : (i < 8) ? i[0] : 1'b0;
            o = (i < 8) ? i[0] : 1'b0;
            p = f ? {3'b0, w, 1'b0} + ((i < 8) ? 16'd6 : 16'd0) : {2'b0, w, 2'b00};
        end else begin
            case (i)
                0: begin w = 64;  h = 48;  f = 0; o = 0; p = 256; end      // exactly the test mode
                1: begin w = 64;  h = 48;  f = 1; o = 1; p = 134; end      // RGB565, bottom-left, pitch + 6
                2: begin w = 37;  h = 29;  f = 0; o = 0; p = 148; end      // smaller: black around
                3: begin w = 100; h = 70;  f = 1; o = 1; p = 206; end      // larger: window
                4: begin w = 80;  h = 60;  f = 0; o = 1; p = 324; end
                5: begin w = 21;  h = 13;  f = 1; o = 0; p = 42;  end
                6: begin w = 64;  h = 48;  f = 0; o = 1; p = 260; end
                7: begin w = 63;  h = 47;  f = 1; o = 0; p = 128; end
                8: begin w = 203; h = 151; f = 0; o = 0; p = 812; end
                default: begin w = 9; h = 5; f = 1; o = 1; p = 18; end
            endcase
        end
        img_geom = {w, h, f, o, p};
    endfunction

    // ---------------- test pattern ----------------
    reg  [11:0] g_w, g_h, g_wm1, g_hm1, g_wm2, g_wm18, g_hm2, g_hm18, g_yh2, g_yh34, g_bw;
    reg  [11:0] g_b [1:7];
    reg  [9:0]  g_idx;
    // per-row flags (display row y)
    reg  [11:0] y;
    reg         r_bord, r_ct, r_cb, r_sq, r_top, r_grid, r_gl, r_gm;
    function automatic [23:0] pat(input [11:0] x);
        reg [2:0] bar; reg [11:0] xo; integer j;
        bar = 0;
        for (j = 1; j < 8; j = j + 1) if (x >= g_b[j]) bar = bar + 3'd1;
        xo = x - 12'd24;
        if (r_bord || x == 12'd0 || x == g_wm1)                 pat = 24'hFFFFFF;
        else if (r_ct && x >= 12'd2 && x < 12'd18)                pat = 24'hFF0000;
        else if (r_ct && x >= g_wm18 && x < g_wm2)                pat = 24'h00FF00;
        else if (r_cb && x >= 12'd2 && x < 12'd18)                pat = 24'h0000FF;
        else if (r_cb && x >= g_wm18 && x < g_wm2)                pat = 24'hFFFF00;
        else if (r_sq && x >= 12'd24 && !xo[3] && {2'b00, xo[11:4]} <= g_idx) pat = 24'h000000;
        else if (r_top) case (bar)
            3'd0: pat = 24'hFFFFFF; 3'd1: pat = 24'hFFFF00; 3'd2: pat = 24'h00FFFF; 3'd3: pat = 24'h00FF00;
            3'd4: pat = 24'hFF00FF; 3'd5: pat = 24'hFF0000; 3'd6: pat = 24'h0000FF; default: pat = 24'h000000;
        endcase
        else if (r_grid) pat = (r_gm || x[6:0] == 7'd0) ? 24'hC0C0C0 : (r_gl || x[4:0] == 5'd0) ? 24'h606060 : 24'h000000;
        else pat = {3{x[7:0]}};
    endfunction
    function automatic [15:0] to565(input [23:0] c);
        to565 = {c[23:19], c[15:10], c[7:3]};
    endfunction

    // ---------------- KEY1 ----------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] k_s = 2'b11;
    reg        k_db = 1'b0, k_long = 1'b0;
    reg [31:0] k_cnt = 0, k_hold = 0;
    reg        ev_short, ev_long;
    assign key_down = k_db;
    always @(posedge clk) begin
        k_s <= {k_s[0], key1_n};
        ev_short <= 1'b0; ev_long <= 1'b0;
        if (rst) begin k_db <= 1'b0; k_cnt <= 0; k_hold <= 0; k_long <= 1'b0; end
        else begin
            if ((~k_s[1]) == k_db) k_cnt <= 0;
            else if (k_cnt == DEBOUNCE - 1) begin
                k_cnt <= 0; k_db <= ~k_s[1];
                if (!(~k_s[1]) && !k_long) ev_short <= 1'b1;      // released before the long-press time
                k_hold <= 0; k_long <= 1'b0;
            end else k_cnt <= k_cnt + 1;
            if (k_db && !k_long) begin
                if (k_hold == LONG_PRESS - 1) begin k_long <= 1'b1; ev_long <= 1'b1; end
                else k_hold <= k_hold + 1;
            end
        end
    end

    // ---------------- command intake (one pending) ----------------
    reg        c_pend;
    reg [63:0] c_cmd;

    // ---------------- main sequencer ----------------
    typedef enum logic [4:0] {H_BOOT, H_FIMG, H_FIMG2, H_FROW, H_FSTRM, H_FDESC, H_IDLE, H_DEC,
                              H_PRES, H_PRES1, H_PRES2, H_MODE, H_MODEW, H_MT, H_MTW} hs_t;
    hs_t        hs;
    reg  [3:0]  fi;                                   // image being filled
    reg  [29:0] f_base, f_row;                        // image base, current row address
    reg  [15:0] f_pitch;
    reg         f_565, f_org;
    reg  [11:0] f_r;                                  // memory row
    reg  [11:0] f_x;                                  // next pixel x
    reg  [11:0] f_words;                              // words of the row still to produce
    reg  [9:0]  p_idx;                                // image to present
    reg         p_now, p_after_mode;
    assign busy   = (hs != H_IDLE) || c_pend;
    assign active = (hs != H_IDLE);

    wire [41:0] geo = img_geom(IMG_SET, fi);
    reg  [23:0] c0, c1;                               // always_comb: pat() reads the row / image registers
    always_comb begin c0 = pat(f_x); c1 = pat(f_x + 12'd1); end
    wire        wadv = !x_wd_valid || x_wd_ready;

    always @(posedge clk) begin
        it_we <= 1'b0; x_cmd_valid <= 1'b0; set_mode <= 1'b0; pres_valid <= 1'b0;
        if (rst) begin
            hs <= H_BOOT; fill_done <= 1'b0; cur_img <= 0; c_pend <= 1'b0; x_wd_valid <= 1'b0;
            st_cmds <= 0; st_errs <= 0; st_fills <= 0; st_mt_runs <= 0; st_mt_errors <= 0; st_mt_first_bad <= 0;
            st_keys <= 0; st_last_cmd <= 0; p_after_mode <= 1'b0; p_now <= 1'b0; p_idx <= 0;
        end else begin
            // JTAG commands: one may wait while the sequencer is busy, a second one is dropped
            if (cmd_valid) begin
                st_cmds <= st_cmds + 1; st_last_cmd <= cmd;
                if (c_pend) st_errs <= st_errs + 1;
                else begin c_pend <= 1'b1; c_cmd <= cmd; end
            end
            if (ev_short) st_keys <= st_keys + 32'h1;
            if (ev_long)  st_keys <= st_keys + 32'h10000;

            case (hs)
                // ---------------- fill ----------------
                H_BOOT: begin fi <= 0; fill_done <= 1'b0; hs <= H_FIMG; end
                H_FIMG: begin
                    {g_w, g_h, f_565, f_org, f_pitch} <= geo;
                    f_base <= IMG_BASE0 + IMG_STRIDE * fi; f_row <= IMG_BASE0 + IMG_STRIDE * fi;
                    g_idx <= {6'h0, fi}; f_r <= 0;
                    hs <= H_FIMG2;
                end
                H_FIMG2: begin
                    // marker bounds clamped at 0: for images narrower / lower than a marker the comparisons then
                    // behave as with signed bounds (x >= negative: always, x < negative: never)
                    g_wm1  <= g_w - 12'd1; g_hm1 <= g_h - 12'd1;
                    g_wm2  <= (g_w < 12'd2)  ? 12'd0 : g_w - 12'd2;
                    g_wm18 <= (g_w < 12'd18) ? 12'd0 : g_w - 12'd18;
                    g_hm2  <= (g_h < 12'd2)  ? 12'd0 : g_h - 12'd2;
                    g_hm18 <= (g_h < 12'd18) ? 12'd0 : g_h - 12'd18;
                    g_yh2 <= g_h >> 1; g_yh34 <= (g_h >> 1) + (g_h >> 2);
                    g_bw <= g_w >> 3;
                    g_b[1] <= g_w >> 3;                g_b[2] <= (g_w >> 3) << 1;
                    g_b[3] <= ((g_w >> 3) << 1) + (g_w >> 3); g_b[4] <= (g_w >> 3) << 2;
                    g_b[5] <= ((g_w >> 3) << 2) + (g_w >> 3); g_b[6] <= ((g_w >> 3) << 2) + ((g_w >> 3) << 1);
                    g_b[7] <= ((g_w >> 3) << 3) - (g_w >> 3);
                    y <= f_org ? g_h - 12'd1 : 12'd0;
                    hs <= H_FROW;
                end
                H_FROW: begin
                    // row flags of display row y (registered; the command waits for xfer_lin anyway)
                    r_bord <= (y == 12'd0) || (y == g_hm1);
                    r_ct   <= (y >= 12'd2) && (y < 12'd18);
                    r_cb   <= (y >= g_hm18) && (y < g_hm2);
                    r_sq   <= (y >= 12'd24) && (y < 12'd32);
                    r_top  <= (y < g_yh2);
                    r_grid <= (y < g_yh34);
                    r_gl   <= (y[4:0] == 5'd0);
                    r_gm   <= (y[6:0] == 7'd0);
                    if (x_cmd_ready && !x_cmd_valid) begin
                        x_cmd_valid <= 1'b1; x_op <= 2'd0; x_addr <= f_row; x_seed <= 0; x_flags <= 0;
                        x_len   <= f_565 ? {17'h0, g_w, 1'b0} : {16'h0, g_w, 2'b00};
                        f_words <= f_565 ? ((g_w + 12'd1) >> 1) : g_w;
                        f_x <= 0;
                        hs <= H_FSTRM;
                    end
                end
                H_FSTRM: begin
                    if (wadv) begin
                        if (f_words != 0) begin
                            x_wd_valid <= 1'b1;
                            if (f_565) begin
                                x_wd_data <= {(f_x + 12'd1 < g_w) ? to565(c1) : 16'h0, to565(c0)};
                                f_x <= f_x + 12'd2;
                            end else begin
                                x_wd_data <= {c0, 8'hFF};
                                f_x <= f_x + 12'd1;
                            end
                            f_words <= f_words - 12'd1;
                        end else begin
                            x_wd_valid <= 1'b0;
                            // row complete: next memory row
                            f_r <= f_r + 12'd1; f_row <= f_row + {14'h0, f_pitch};
                            y <= f_org ? y - 12'd1 : y + 12'd1;
                            hs <= (f_r == g_hm1) ? H_FDESC : H_FROW;
                        end
                    end
                end
                H_FDESC: if (x_cmd_ready) begin                 // the last row's writes are accepted
                    it_we <= 1'b1; it_widx <= {6'h0, fi};
                    it_wdesc <= '0;
                    it_wdesc.base <= f_base; it_wdesc.pitch <= f_pitch; it_wdesc.width <= {1'b0, g_w};
                    it_wdesc.height <= {1'b0, g_h}; it_wdesc.format <= f_565 ? FMT_RGB565 : FMT_RGBA8;
                    it_wdesc.levels <= 4'd1; it_wdesc.layers <= 8'd1; it_wdesc.origin <= f_org;
                    it_wdesc.usage <= (4'b1 << USE_SAMPLED) | (4'b1 << USE_SCANOUT) | (4'b1 << USE_TRANSFER);
                    if (fi == NIMG - 1) begin
                        fill_done <= 1'b1; st_fills <= st_fills + 1;
                        p_idx <= mode_img(cur_mode); p_now <= 1'b0; hs <= H_PRES;
                    end else begin fi <= fi + 4'd1; hs <= H_FIMG; end
                end
                // ---------------- idle: commands, keys ----------------
                H_IDLE: if (!ext_busy) begin
                    if (c_pend) begin c_pend <= 1'b0; hs <= H_DEC; end
                    else if (ev_short) begin
                        p_idx <= (cur_img == NIMG - 1) ? 10'd0 : cur_img + 10'd1; p_now <= 1'b0; hs <= H_PRES;
                    end else if (ev_long) begin
                        set_mode_id <= (cur_mode >= 3'd3) ? 3'd0 : cur_mode + 3'd1; p_after_mode <= 1'b1; hs <= H_MODE;
                    end
                end
                H_DEC: case (c_cmd[63:56])
                    8'h01: begin set_mode_id <= c_cmd[2:0]; p_after_mode <= 1'b0; hs <= H_MODE; end
                    8'h02: begin p_idx <= c_cmd[9:0]; p_now <= c_cmd[16]; hs <= H_PRES; end
                    8'h03: begin
                        if (c_cmd[45:36] == 10'd0 || {1'b0, c_cmd[55:46]} + {1'b0, c_cmd[45:36]} > 11'd1024) begin
                            st_errs <= st_errs + 1; hs <= H_IDLE;
                        end else hs <= H_MT;
                    end
                    8'h04: hs <= H_BOOT;
                    8'h05: begin
                        if (c_cmd[0]) begin
                            set_mode_id <= (cur_mode >= 3'd3) ? 3'd0 : cur_mode + 3'd1; p_after_mode <= 1'b1; hs <= H_MODE;
                        end else begin
                            p_idx <= (cur_img == NIMG - 1) ? 10'd0 : cur_img + 10'd1; p_now <= 1'b0; hs <= H_PRES;
                        end
                    end
                    8'h06: hs <= H_IDLE;                         // LINKCTL: executed by board_b_core
                    default: begin st_errs <= st_errs + 1; hs <= H_IDLE; end
                endcase
                // ---------------- present (descriptor from img_table) ----------------
                H_PRES:  begin it_ridx <= p_idx; hs <= H_PRES1; end
                H_PRES1: hs <= H_PRES2;
                H_PRES2: begin
                    if (it_rvalid && it_rdesc.usage[USE_SCANOUT] && it_rdesc.layout == 2'd0 &&
                        (it_rdesc.format == FMT_RGBA8 || it_rdesc.format == FMT_RGB565)) begin
                        pres_valid <= 1'b1; pres_base <= it_rdesc.base; pres_pitch <= it_rdesc.pitch;
                        pres_w <= it_rdesc.width; pres_h <= it_rdesc.height;
                        pres_565 <= (it_rdesc.format == FMT_RGB565); pres_origin <= it_rdesc.origin;
                        pres_now <= p_now; pres_tag <= {22'h0, p_idx};
                        cur_img <= p_idx;
                    end else st_errs <= st_errs + 1;
                    hs <= H_IDLE;
                end
                // ---------------- video mode ----------------
                H_MODE: if (!mode_busy) begin set_mode <= 1'b1; hs <= H_MODEW; end
                H_MODEW: if (mode_done) begin
                    if (!mode_ok) st_errs <= st_errs + 1;
                    if (p_after_mode && mode_ok) begin p_idx <= mode_img(set_mode_id); p_now <= 1'b0; hs <= H_PRES; end
                    else hs <= H_IDLE;
                end
                // ---------------- memory test ----------------
                H_MT: if (x_cmd_ready) begin
                    x_cmd_valid <= 1'b1; x_op <= 2'd2; x_addr <= {c_cmd[55:46], 20'h0}; x_len <= {c_cmd[45:36], 20'h0};
                    x_seed <= c_cmd[31:0]; x_flags <= 2'b11; hs <= H_MTW;
                end
                H_MTW: if (x_mt_done) begin
                    st_mt_runs <= st_mt_runs + 1; st_mt_errors <= x_mt_errors; st_mt_first_bad <= x_mt_first_bad;
                    hs <= H_IDLE;
                end
                default: hs <= H_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
