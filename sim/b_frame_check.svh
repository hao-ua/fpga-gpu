// b_frame_check.svh — board B HDMI frame checker shared by tb_board_b and tb_two_boards (include inside the
// bench module). Needs: clk, int errors, the board_b_core instance `dut`, and tmds_capture outputs ws, c_de,
// c_hs, c_vs, c_r, c_g, c_b, misalign. Locks onto frames and compares every word with the expected timing
// of the running mode and the presented test image (b_bringup simulation set; command-processor presents
// carry the image handle in tag[9:0], harness presents the index).
// Palette and cursor: per frame the checker takes the palette bank the scanout tagged the
// frame with and the cursor state the pixel domain applied at that vsync (the state hand-over itself is checked by
// the bench against what it sent); P8 pixels are looked up in that bank, and the cursor is blended over the
// expected pixel with an independent model (real-number rounding).
    // ---------------- expected images (bench copy of b_bringup IMG_SET 1 + test pattern) ----------------
    typedef struct { int w, h; bit f, o; } geo_t;
    function automatic geo_t geo(input int i);
        geo_t g;
        case (i)
            0: g = '{64, 48, 0, 0};  1: g = '{64, 48, 1, 1};  2: g = '{37, 29, 0, 0};  3: g = '{100, 70, 1, 1};
            4: g = '{80, 60, 0, 1};  5: g = '{21, 13, 1, 0};  6: g = '{64, 48, 0, 1};  7: g = '{63, 47, 1, 0};
            8: g = '{203, 151, 0, 0}; default: g = '{9, 5, 1, 1};
        endcase
        return g;
    endfunction
    function automatic [23:0] pattern(input int i, input int x, input int y);
        geo_t g; int bw, bar;
        g = geo(i); bw = g.w / 8;
        bar = 0; for (int j = 1; j < 8; j++) if (x >= j * bw) bar++;
        if (x == 0 || y == 0 || x == g.w - 1 || y == g.h - 1) return 24'hFFFFFF;
        if (y >= 2 && y < 18 && x >= 2 && x < 18) return 24'hFF0000;
        if (y >= 2 && y < 18 && x >= g.w - 18 && x < g.w - 2) return 24'h00FF00;
        if (y >= g.h - 18 && y < g.h - 2 && x >= 2 && x < 18) return 24'h0000FF;
        if (y >= g.h - 18 && y < g.h - 2 && x >= g.w - 18 && x < g.w - 2) return 24'hFFFF00;
        if (y >= 24 && y < 32 && x >= 24 && ((x - 24) % 16) < 8 && (x - 24) / 16 <= i) return 24'h000000;
        if (y < g.h / 2) case (bar)
            0: return 24'hFFFFFF; 1: return 24'hFFFF00; 2: return 24'h00FFFF; 3: return 24'h00FF00;
            4: return 24'hFF00FF; 5: return 24'hFF0000; 6: return 24'h0000FF; default: return 24'h000000;
        endcase
        if (y < g.h / 2 + g.h / 4) begin
            if (x % 128 == 0 || y % 128 == 0) return 24'hC0C0C0;
            if (x % 32 == 0 || y % 32 == 0) return 24'h606060;
            return 24'h000000;
        end
        return {3{8'(x % 256)}};
    endfunction
    // A bench that also presents host-created images defines FC_HOST_IMAGES and provides
    // host_img_known(index) / host_img_rgb(index, x, y, palette bank) (display coordinates, black outside the image).
    int   fr_bank = 0;                                    // palette bank of the frame being checked
    function automatic [23:0] base_rgb(input int img, input int x, input int y);
        geo_t g; reg [23:0] c;
        if (img < 0) return 24'h0;
`ifdef FC_HOST_IMAGES
        if (host_img_known(img)) return host_img_rgb(img, x, y, fr_bank);
`endif
        g = geo(img);
        if (x >= g.w || y >= g.h) return 24'h0;
        c = pattern(img, x, y);
        if (g.f) c = {c[23:19], c[23:21], c[15:10], c[15:14], c[7:3], c[7:5]};
        return c;
    endfunction
    // cursor of the frame being checked (taken at the frame's first pixel from the pixel domain)
    bit   fc_en; int fc_mode, fc_bank, fc_ox, fc_oy;
    int   fc_cursor_px = 0;                               // displayed pixels the cursor covered
    function automatic int rnd_r(input real f); return $rtoi(f + 0.5); endfunction
    function automatic [23:0] expect_rgb(input int img, input int x, input int y);
        reg [23:0] p, o; bit [31:0] c; int cx, cy;
        p = base_rgb(img, x, y);
        cx = x - fc_ox; cy = y - fc_oy;
        if (!fc_en || cx < 0 || cx >= 64 || cy < 0 || cy >= 64) return p;
        c = dut.u_disp.u_dout.u_post.cram[{fc_bank[0], 6'(cy), 6'(cx)}];
        for (int ch = 0; ch < 3; ch++) begin
            int cc, pp, a, vv;
            cc = c[ch*8 +: 8]; pp = p[ch*8 +: 8]; a = c[31:24];
            case (fc_mode)
                1: vv = ((c[24] ? pp : 0) ^ cc);
                2: begin vv = cc + rnd_r(pp * (255 - a) / 255.0); if (vv > 255) vv = 255; end
                default: vv = rnd_r((cc * a + pp * (255 - a)) / 255.0);
            endcase
            o[ch*8 +: 8] = 8'(vv);
        end
        return o;
    endfunction
    task automatic take_cursor();
        fc_en = dut.u_disp.u_dout.u_post.c_en; fc_mode = dut.u_disp.u_dout.u_post.c_mode;
        fc_bank = dut.u_disp.u_dout.u_post.c_bank;
        fc_ox = $signed(dut.u_disp.u_dout.u_post.c_ox); fc_oy = $signed(dut.u_disp.u_dout.u_post.c_oy);
    endtask

    // ---------------- frame checker ----------------
    // image of the next frame = the scanout's current image right after its latest restart, with its palette bank
    int   last_restart_img = -1, last_restart_bank = 0;
    reg   rs_d1 = 0, rs_d2 = 0;
    always @(posedge clk) begin
        rs_d1 <= dut.u_disp.u_scan.restart; rs_d2 <= rs_d1;
        if (rs_d2) begin
            last_restart_img = dut.u_disp.u_scan.cur_valid ? int'(dut.u_disp.u_scan.cur_tag[9:0]) : -1;
            last_restart_bank = dut.u_disp.u_scan.f_pal;
        end
    end
    int   st = 0;                       // 0 = search vsync, 1 = wait for the first DE, 2 = locked
    int   h, v, ha, hf, hsw, hb, va, vf, vsw, vb, ht, vt; bit hp, vp;
    int   img = -1, alt = -1;           // image of this frame; "at once" alternative
    bit   l_a, l_n, switched;
    int   bad_x; reg [23:0] bad_got, bad_exp;
    int   frames_ok = 0, frames_img [int];
    int   last_frame_img = -99;
    int   misalign_locked = 0, nprint = 0;
    int   cur_mode_chk = -1;
    task automatic report(input string s);
        errors++;
        if (nprint < 25) begin $display("%t FAIL %s", $time, s); nprint++; end
    endtask
    always @(negedge dut.u_disp.run) st = 0;
    always @(ws) begin
        if (st == 0) begin
            if (dut.u_disp.run && !c_de && c_vs == dut.u_disp.t_vp) st = 1;
        end else if (st == 1) begin
            if (c_de) begin
                // frame start (h = 0, v = 0)
                ha = dut.u_disp.t_ha; hf = dut.u_disp.t_hf; hsw = dut.u_disp.t_hs; hb = dut.u_disp.t_hb;
                va = dut.u_disp.t_va; vf = dut.u_disp.t_vf; vsw = dut.u_disp.t_vs; vb = dut.u_disp.t_vb;
                hp = dut.u_disp.tim[1]; vp = dut.u_disp.tim[0];
                ht = ha + hf + hsw + hb; vt = va + vf + vsw + vb;
                cur_mode_chk = dut.u_disp.cur_mode;
                h = 0; v = 0; st = 2; img = last_restart_img; alt = -1; switched = 0; l_a = 1; l_n = 1;
            end
        end
        if (st == 2) begin
            bit de_e, hs_e, vs_e; reg [23:0] e, en;
            if (h == 0 && v == 0) begin
                img = last_restart_img; switched = 0; fr_bank = last_restart_bank; take_cursor();
            end
            if (c_de && fc_en && h - fc_ox >= 0 && h - fc_ox < 64 && v - fc_oy >= 0 && v - fc_oy < 64) fc_cursor_px++;
            de_e = (h < ha) && (v < va);
            hs_e = (h >= ha + hf && h < ha + hf + hsw) ? hp : !hp;
            vs_e = (v >= va + vf && v < va + vf + vsw) ? vp : !vp;
            if (c_de !== de_e) report($sformatf("mode %0d (h %0d, v %0d): DE %b expected %b", cur_mode_chk, h, v, c_de, de_e));
            else if (!c_de && (c_hs !== hs_e || c_vs !== vs_e))
                report($sformatf("mode %0d (h %0d, v %0d): HSYNC %b VSYNC %b expected %b %b", cur_mode_chk, h, v, c_hs, c_vs, hs_e, vs_e));
            else if (c_de) begin
                e = expect_rgb(img, h, v);
                en = expect_rgb(alt, h, v);
                if ({c_r, c_g, c_b} !== e && l_a) begin l_a = 0; bad_x = h; bad_got = {c_r, c_g, c_b}; bad_exp = e; end
                if (alt < 0 || {c_r, c_g, c_b} !== en) l_n = 0;
                if (h == ha - 1) begin                                     // end of an active line
                    if (switched ? !l_n : (!l_a && !l_n))
                        report($sformatf("mode %0d image %0d (alt %0d) line %0d differs, first at x %0d: %06h expected %06h",
                                         cur_mode_chk, img, alt, v, bad_x, bad_got, bad_exp));
                    if (!l_a && l_n) switched = 1;
                    l_a = 1; l_n = 1;
                end
            end
            if (misalign != misalign_locked) begin
                report($sformatf("TMDS word misalignment while locked (%0d)", misalign)); misalign_locked = misalign;
            end
            h++;
            if (h == ht) begin
                h = 0; v++;
                if (v == vt) begin
                    v = 0; frames_ok++; last_frame_img = switched ? alt : img;
                    if (frames_img.exists(last_frame_img)) frames_img[last_frame_img]++; else frames_img[last_frame_img] = 1;
                    if (switched) img = alt;
                    alt = -1;
                end
            end
        end else misalign_locked = misalign;
    end

    task automatic wait_img_frames(input int i, input int n, input int max_frames = 8);
        int f0, c0;
        f0 = frames_ok; c0 = frames_img.exists(i) ? frames_img[i] : 0;
        while ((frames_img.exists(i) ? frames_img[i] : 0) < c0 + n) begin
            @(posedge clk);
            if (frames_ok > f0 + max_frames) begin report($sformatf("image %0d not shown within %0d frames", i, max_frames)); return; end
        end
    endtask
