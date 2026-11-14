`timescale 1ns/1ps
// tb_scan_reader — scan_reader + display_line_buffer against mig_app_model (random ready) through mem_arb,
// with competing read/write traffic on two more ports, and a pixel-domain consumer that behaves like
// display_out (preroll, video_timing, line release at de-fall, vsync frame sync). Every displayed pixel is
// compared with the expected one, computed from the DDR model's bytes:
//   * random images: RGBA8 / RGB565 / P8, origin 0 / 1, width and height smaller and larger than the mode, pitch
//     = width x bpp + random extra (rows start at any pixel offset inside a burst), random 32-byte bases;
//   * line-buffer words: direct colour with alpha 0xFF, black 0x000000FF, P8 {index, 0, 0, bank}
//     with the palette bank the reader took at that frame's start (pal_bank toggles at random times);
//   * presents at vsync (the frame after the restart shows the new image), two presents in one frame (the
//     first is dropped), presents "at once" (each line shows the old or the new image, switching once);
//   * black before the first present, stop / mode change / restart (64-wide and 1280-wide modes), and no
//     line-buffer underrun anywhere.
`default_nettype none
module tb_scan_reader;
    reg clk = 0, rst = 1;
    always #5.0 clk = ~clk;
    real pix_half = 6.734;                     // 74.25 MHz
    reg  pix = 0;
    always #(pix_half) pix = ~pix;
    int errors = 0;

    // ---------------- DDR model + arbiter ----------------
    wire        app_en, app_wdf_wren, app_wdf_end, app_rdy, app_wdf_rdy, app_rd_data_valid;
    wire [2:0]  app_cmd; wire [28:0] app_addr; wire [255:0] app_wdf_data, app_rd_data; wire [31:0] app_wdf_mask;
    int mig_err; longint n_wr, n_rd;
    mig_app_model #(.RD_LAT(24), .RDY_PCT(75), .WDF_PCT(80), .SEED(7)) u_mig (
        .ui_clk(clk), .rst(rst), .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end), .app_rdy(app_rdy),
        .app_wdf_rdy(app_wdf_rdy), .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid),
        .errors(mig_err), .n_wr(n_wr), .n_rd(n_rd));
    wire [1:0] r_en, r_rdy, r_valid; wire [57:0] r_addr; wire [255:0] rd_data;
    wire w_req, w_en, w_wren, w_wend, w_rdy, w_wdf_rdy; wire [28:0] w_addr; wire [255:0] w_data; wire [31:0] w_mask;
    mem_arb #(.NR(2), .NW(1), .RO_AW(5), .RO_RESERVE(4), .MUST_RUN(2'b01)) u_arb (
        .clk(clk), .rst(rst), .r_en(r_en), .r_addr(r_addr), .r_rdy(r_rdy), .r_valid(r_valid), .rd_data(rd_data),
        .w_req(w_req), .w_en(w_en), .w_addr(w_addr), .w_data(w_data), .w_mask(w_mask), .w_wren(w_wren),
        .w_wend(w_wend), .w_rdy(w_rdy), .w_wdf_rdy(w_wdf_rdy),
        .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end),
        .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy), .app_rd_data_valid(app_rd_data_valid), .app_rd_data(app_rd_data));

    // competing traffic: R1 random reads (~40 % of cycles), W0 single-burst writes into 512 MB+ (never images)
    reg        n_ren = 0; reg [28:0] n_raddr = 0;
    always @(posedge clk) if (!rst) begin
        if (!n_ren || r_rdy[1]) begin n_ren <= ($urandom % 100) < 40; n_raddr <= {1'b0, 5'h0, 20'($urandom), 3'b000}; end
    end
    assign r_en[1] = n_ren; assign r_addr[57:29] = n_raddr;
    reg        nw_act = 0, nw_cmd = 0, nw_dat = 0; reg [28:0] nw_addr = 0; reg [255:0] nw_d = 0;
    always @(posedge clk) begin
        if (rst) begin nw_act <= 0; nw_cmd <= 0; nw_dat <= 0; end
        else if (!nw_act) begin
            if (($urandom % 100) < 3) begin
                nw_act <= 1; nw_cmd <= 1; nw_dat <= 1; nw_addr <= {1'b0, 3'b100, 22'($urandom), 3'b000};
                nw_d <= {8{$urandom}};
            end
        end else begin
            nw_cmd <= nw_cmd & ~w_rdy;
            nw_dat <= nw_dat & ~w_wdf_rdy;
            if (!(nw_cmd & ~w_rdy) && !(nw_dat & ~w_wdf_rdy)) nw_act <= 0;    // command and data both accepted
        end
    end
    assign w_req = nw_act; assign w_en = nw_cmd; assign w_addr = nw_addr; assign w_data = nw_d; assign w_mask = 32'h0;
    assign w_wren = nw_dat; assign w_wend = nw_dat;

    // ---------------- DUT ----------------
    reg         run = 0;
    reg  [10:0] mode_w = 64, mode_h = 12;
    reg  [11:0] h_fp = 8, h_sw = 8, h_bp = 8, v_fp = 2, v_sw = 2, v_bp = 4;
    reg         pres_valid = 0, pres_origin = 0, pres_now = 0; reg [1:0] pres_fmt = 0;
    reg  [29:0] pres_base = 0; reg [15:0] pres_pitch = 0; reg [12:0] pres_w = 0, pres_h = 0; reg [31:0] pres_tag = 0;
    wire        idle, pres_done, cur_valid, frame_start; wire [31:0] cur_tag, c_dropped;
    wire        lb_en, lb_done, lb_fs, lb_avail; wire [7:0] lb_idx; wire [255:0] lb_data;
    wire        frame_sync;
    reg         pal_bank = 0;
    always @(posedge clk) if (($urandom % 2000) == 0) pal_bank <= ~pal_bank;
    scan_reader dut (
        .clk(clk), .rst(rst), .run(run), .idle(idle), .frame_sync(frame_sync), .mode_w(mode_w), .mode_h(mode_h),
        .pres_valid(pres_valid), .pres_base(pres_base), .pres_pitch(pres_pitch), .pres_w(pres_w), .pres_h(pres_h),
        .pres_fmt(pres_fmt), .pres_origin(pres_origin), .pres_now(pres_now), .pres_tag(pres_tag),
        .pal_bank(pal_bank), .frame_start(frame_start),
        .pres_done(pres_done), .cur_valid(cur_valid), .cur_tag(cur_tag), .c_dropped(c_dropped),
        .r_en(r_en[0]), .r_addr(r_addr[28:0]), .r_rdy(r_rdy[0]), .r_valid(r_valid[0]), .rd_data(rd_data),
        .lb_en(lb_en), .lb_idx(lb_idx), .lb_data(lb_data), .lb_line_done(lb_done), .lb_frame_start(lb_fs),
        .lb_avail(lb_avail));

    wire px_rst;
    rst_sync #(.STAGES(4)) u_pr (.clk(pix), .arst(~run), .rst(px_rst));
    wire        rd_en, rd_avail; wire [10:0] rd_idx; wire [31:0] rd_px; reg rd_line_done = 0, rd_frame_sync = 0;
    display_line_buffer #(.N_LINES(16), .MAX_BURSTS_PER_LINE(160), .MAX_PIXELS_PER_LINE(1280)) u_lb (
        .wr_clk(clk), .wr_rst(rst), .wr_burst_en(lb_en), .wr_burst_idx(lb_idx), .wr_burst_data(lb_data),
        .wr_line_done(lb_done), .wr_frame_sync(lb_fs), .wr_buf_avail(lb_avail),
        .rd_clk(pix), .rd_rst(px_rst), .rd_pixel_en(rd_en), .rd_pixel_idx(rd_idx), .rd_pixel(rd_px),
        .rd_line_done(rd_line_done), .rd_frame_sync(rd_frame_sync), .rd_buf_avail(rd_avail));

    // ---------------- pixel-domain consumer (display_out behaviour) ----------------
    reg pre_done = 0; reg [10:0] pre_cnt = 0;
    always @(posedge pix or posedge px_rst)
        if (px_rst) begin pre_done <= 0; pre_cnt <= 0; end
        else if (!pre_done) begin
            if (!rd_avail) pre_cnt <= 0; else if (pre_cnt == 11'd64) pre_done <= 1; else pre_cnt <= pre_cnt + 1;
        end
    wire vt_rst_n = ~px_rst & pre_done;
    wire hs, vs, de; wire [11:0] x, y, hc, vc;
    video_timing u_vt (.pixel_clk(pix), .rst_n(vt_rst_n), .h_active({1'b0, mode_w}), .h_fp(h_fp), .h_sync_w(h_sw), .h_bp(h_bp),
        .v_active({1'b0, mode_h}), .v_fp(v_fp), .v_sync_w(v_sw), .v_bp(v_bp), .hsync_pol(1'b1), .vsync_pol(1'b1),
        .h_count(hc), .v_count(vc), .hsync(hs), .vsync(vs), .de(de), .x(x), .y(y));
    assign rd_en = de; assign rd_idx = x[10:0];
    reg de_q = 0, vs_q = 0, fs_tog = 0, ur_tog = 0;
    always @(posedge pix or negedge vt_rst_n)
        if (!vt_rst_n) begin de_q <= 0; vs_q <= 0; rd_line_done <= 0; rd_frame_sync <= 0; fs_tog <= 0; end
        else begin
            de_q <= de; vs_q <= vs;
            rd_line_done  <= de_q & ~de & rd_avail;
            if (de_q & ~de & ~rd_avail) ur_tog <= ~ur_tog;
            rd_frame_sync <= vs & ~vs_q;
            if (vs & ~vs_q) fs_tog <= ~fs_tog;
        end
    (* ASYNC_REG = "TRUE" *) reg [2:0] fs_s = 0, ur_s = 0;
    always @(posedge clk) begin fs_s <= {fs_s[1:0], fs_tog}; ur_s <= {ur_s[1:0], ur_tog}; end
    assign frame_sync = run & (fs_s[2] ^ fs_s[1]);
    int underruns = 0;
    always @(posedge clk) if (run && (ur_s[2] ^ ur_s[1])) underruns++;

    // ---------------- images and expected pixels ----------------
    typedef struct { bit v; bit [29:0] base; int pitch, w, h; int fmt; bit org; } img_t;
    img_t imgs [0:255];
    int   nimg = 0;
    function automatic int bpp(input int f); return (f == 0) ? 4 : (f == 1) ? 2 : 1; endfunction
    function automatic [31:0] exp565(input [15:0] c);
        exp565 = {c[15:11], c[15:13], c[10:5], c[10:9], c[4:0], c[4:2], 8'hFF};
    endfunction
    function automatic [31:0] expect_px(input int t, input bit bank, input int yy, input int xx);
        img_t I; longint a; int row;
        if (t < 0) return 32'h000000FF;
        I = imgs[t];
        if (!I.v || yy >= I.h || xx >= I.w) return 32'h000000FF;
        row = I.org ? (I.h - 1 - yy) : yy;
        a = I.base + longint'(row) * I.pitch + bpp(I.fmt) * xx;
        if (I.fmt == 1) return exp565({u_mig.peek_byte(a + 1), u_mig.peek_byte(a)});
        if (I.fmt == 2) return {u_mig.peek_byte(a), 16'h0, 7'h0, bank};
        return {u_mig.peek_byte(a + 3), u_mig.peek_byte(a + 2), u_mig.peek_byte(a + 1), 8'hFF};
    endfunction

    task automatic new_image(input int w, input int h, input int fmt, input bit org, input int extra);
        img_t I; longint b0, b1;
        I.v = 1; I.w = w; I.h = h; I.fmt = fmt; I.org = org;
        I.pitch = (w + extra) * bpp(fmt);
        I.base = 30'h0100_0000 + nimg * 30'h0010_0000 + 32 * ($urandom % 64);
        b0 = I.base >> 5; b1 = (I.base + longint'(h) * I.pitch + 63) >> 5;
        for (longint b = b0; b <= b1; b++) u_mig.mem[b] = {$urandom, $urandom, $urandom, $urandom, $urandom, $urandom, $urandom, $urandom};
        imgs[nimg] = I; nimg++;
    endtask

    // frame -> image: pushed at every reader restart (the image current after adoption) with the frame's bank
    int   fq [$];
    bit   fbq [$];
    int   cur_show = -1;          // image of the frame being displayed (-1 = black)
    int   alt_show = -1;          // "at once" present: the frame may switch to this image at a line boundary
    bit   cur_bank = 0;
    reg   restart_d1 = 0, restart_d2 = 0, bank_at = 0;
    int   n_p8_frames = 0, n_bank1 = 0;
    always @(posedge clk) begin
        restart_d1 <= frame_start; restart_d2 <= restart_d1;
        if (frame_start) bank_at <= pal_bank;                  // what the reader must take at this restart
        if (restart_d2) begin
            fq.push_back(dut.cur_valid ? int'(dut.cur_tag) : -1); fbq.push_back(bank_at);
            if (dut.f_pal !== bank_at) begin errors++; $display("%t FAIL reader took bank %0d, expected %0d", $time, dut.f_pal, bank_at); end
        end
    end
    // line capture and check
    reg  [31:0] line_px [0:1279];
    reg         de_d = 0; reg [11:0] x_d = 0, y_d = 0;
    int   frames_checked = 0, lines_checked = 0, switched = 0, n_switch = 0;
    always @(posedge pix) begin
        de_d <= de & vt_rst_n; x_d <= x; y_d <= y;
        if (de && vt_rst_n && x == 0 && y == 0) begin
            if (fq.size() == 0) begin errors++; $display("%t FAIL frame without a reader restart", $time); end
            else begin
                cur_show = fq.pop_front(); cur_bank = fbq.pop_front(); alt_show = -1; switched = 0;
                if (cur_show >= 0 && imgs[cur_show].fmt == 2) begin n_p8_frames++; if (cur_bank) n_bank1++; end
            end
        end
        if (de_d) line_px[x_d] = rd_px;
        if (de_d && !(de & vt_rst_n)) begin           // line finished: compare
            bit mA, mN; int xx;
            mA = 1; mN = (alt_show >= 0);
            for (xx = 0; xx < mode_w; xx++) begin
                if (line_px[xx] !== expect_px(cur_show, cur_bank, y_d, xx)) mA = 0;
                if (alt_show >= 0 && line_px[xx] !== expect_px(alt_show, cur_bank, y_d, xx)) mN = 0;
            end
            if (switched && !mN) begin errors++; $display("%t FAIL line %0d: switched to image %0d but does not match it", $time, y_d, alt_show); end
            else if (!switched && !mA && !mN) begin
                errors++;
                for (xx = 0; xx < mode_w; xx++) if (line_px[xx] !== expect_px(cur_show, cur_bank, y_d, xx)) begin
                    $display("%t FAIL image %0d (fmt %0d) line %0d x %0d: got %08h expected %08h", $time, cur_show,
                             (cur_show >= 0) ? imgs[cur_show].fmt : -1, y_d, xx, line_px[xx], expect_px(cur_show, cur_bank, y_d, xx));
                    break;
                end
            end
            if (!mA && mN && !switched) begin switched = 1; n_switch++; end
            lines_checked++;
            if (y_d == mode_h - 1) frames_checked++;
        end
    end

    // ---------------- stimulus helpers ----------------
    task automatic present(input int t, input bit now);
        @(posedge clk);
        pres_valid <= 1; pres_base <= imgs[t].base; pres_pitch <= imgs[t].pitch; pres_w <= imgs[t].w; pres_h <= imgs[t].h;
        pres_fmt <= 2'(imgs[t].fmt); pres_origin <= imgs[t].org; pres_now <= now; pres_tag <= t;
        @(posedge clk); pres_valid <= 0;
    endtask
    task automatic wait_frames(input int n);
        int f0; f0 = frames_checked;
        while (frames_checked < f0 + n) @(posedge clk);
    endtask
    task automatic set_mode(input int w, input int h, input int hf, input int hsw, input int hb,
                            input int vf, input int vsw, input int vb, input real mhz);
        @(posedge clk); run <= 0;
        @(posedge clk); while (!idle) @(posedge clk);
        repeat (20) @(posedge clk);
        mode_w <= w; mode_h <= h; h_fp <= hf; h_sw <= hsw; h_bp <= hb; v_fp <= vf; v_sw <= vsw; v_bp <= vb;
        pix_half = 500.0 / mhz;
        fq.delete(); fbq.delete();
        repeat (20) @(posedge clk);
        run <= 1;
    endtask
    task automatic random_image(input int mw, input int mh);
        int w, h, f; bit o;
        f = $urandom % 3; o = $urandom % 2;
        case ($urandom % 4)
            0: begin w = mw; h = mh; end
            1: begin w = 1 + $urandom % mw; h = 1 + $urandom % mh; end
            2: begin w = mw + 1 + $urandom % 120; h = mh + 1 + $urandom % 6; end
            default: begin w = 1 + $urandom % (mw + 100); h = 1 + $urandom % (mh + 4); end
        endcase
        new_image(w, h, f, o, $urandom % 19);
    endtask

    initial begin
        int i, t, d0;
        repeat (20) @(posedge clk);
        rst <= 0;
        repeat (20) @(posedge clk);
        // ---- phase A: 64x12 mode @ 40 MHz: black first, then random images at vsync ----
        set_mode(64, 12, 8, 8, 8, 2, 2, 4, 40.0);
        wait_frames(2);
        $display("ok: black before the first present (%0d frames)", frames_checked);
        for (i = 0; i < 40; i++) begin
            random_image(64, 12); present(nimg - 1, 0); wait_frames(2);
        end
        $display("ok: phase A, %0d images at vsync, %0d frames checked", 40, frames_checked);
        // two presents in one frame: the first is dropped, the second shown
        random_image(64, 12); random_image(64, 12);
        d0 = c_dropped;
        present(nimg - 2, 0); present(nimg - 1, 0); wait_frames(2);
        if (c_dropped != d0 + 1) begin errors++; $display("FAIL dropped count %0d, expected %0d", c_dropped, d0 + 1); end
        else $display("ok: replaced pending present counted as dropped");
        // presents at once: lines switch from the old to the new image within the frame (a 64-line mode, so
        // the reader is still inside the frame when the present arrives)
        set_mode(64, 64, 8, 8, 8, 2, 2, 4, 40.0);
        random_image(64, 64); present(nimg - 1, 0); wait_frames(2);
        for (i = 0; i < 16; i++) begin
            random_image(64, 64);
            t = 2 + $urandom % 30;
            @(posedge pix); while (!(de && y == 12'(t))) @(posedge pix);
            alt_show = nimg - 1;
            present(nimg - 1, 1); wait_frames(2);
            if (i % 4 == 3) begin present(nimg - 2, 0); wait_frames(2); end
        end
        $display("ok: presents at once (%0d frames switched mid-frame)", n_switch);
        // ---- phase B: 1280-wide lines at 74.25 MHz (720p line timing), full-width and window images ----
        set_mode(1280, 6, 110, 40, 220, 2, 2, 6, 74.25);
        for (i = 0; i < 14; i++) begin
            if (i < 3) new_image(1280, 6, i, i / 2, i * 7); else random_image(1280, 6);
            present(nimg - 1, 0); wait_frames(2);
        end
        $display("ok: phase B (1280 wide) done");
        // ---- phase C: back to a narrow mode with the image kept, then a faster pixel clock ----
        set_mode(96, 10, 8, 8, 8, 2, 2, 4, 40.0);
        wait_frames(2);
        for (i = 0; i < 10; i++) begin random_image(96, 10); present(nimg - 1, i % 3 == 0); wait_frames(2); end
        set_mode(800, 5, 40, 128, 88, 1, 4, 3, 150.0);       // 800 px per 7 us line: stress the reader
        for (i = 0; i < 8; i++) begin random_image(800, 5); present(nimg - 1, 0); wait_frames(2); end
        $display("ok: phase C (mode changes, 150 MHz pixel clock)");
        repeat (100) @(posedge clk);
        if (underruns != 0) begin errors++; $display("FAIL %0d line-buffer underruns", underruns); end
        if (mig_err != 0) begin errors++; $display("FAIL MIG model errors %0d", mig_err); end
        if (n_p8_frames < 20 || n_bank1 == 0 || n_bank1 == n_p8_frames) begin
            errors++; $display("FAIL P8 coverage: %0d P8 frames, %0d with bank 1", n_p8_frames, n_bank1);
        end
        $display("INFO_TB frames %0d lines %0d images %0d reads %0d, P8 frames %0d (bank 1: %0d)", frames_checked,
                 lines_checked, nimg, n_rd, n_p8_frames, n_bank1);
        if (errors == 0) $display("TB_PASS tb_scan_reader");
        else $display("TB_FAIL tb_scan_reader errors=%0d", errors);
        $finish;
    end
    initial begin #(200_000_000.0); $display("TB_FAIL tb_scan_reader timeout"); $finish; end
endmodule
`default_nettype wire
