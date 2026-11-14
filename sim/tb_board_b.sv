`timescale 1ns/1ps
// tb_board_b — the whole board_b_core with a DDR model, the real video MMCM (UNISIM,
// reconfigured over DRP) and the real OSERDES serializers. tmds_capture decodes the HDMI lanes back into
// words; the checker locks onto frames and compares EVERY word of every frame with an independent model:
//   * timing: DE, HSYNC, VSYNC positions and polarities from the running mode's (h, v) counters;
//   * pixels: the presented test image (bench copy of b_bringup's simulation image set and test pattern,
//     RGB565 images quantised and expanded), shown at the top-left, black around / top-left window;
// and runs the harness through: boot (black, fill, first present), PRESENT of every image, a present at
// once (each line old or new image, switching once), KEY1 short press, a command over the real JTAG
// USER2 path (BSCANE2 + glbl) with read-back, MEMTEST, REFILL after a memtest over an image, SET_MODE
// (same mode, invalid mode). With +FULL also one full frame of each hardware mode 0..3 (KEY1 long press
// = next mode + present 2 x mode). Final checks: status vector, no underruns, no TMDS misalignment while
// locked, no MIG model errors.
`default_nettype none
module tb_board_b;
`include "video_modes.svh"
    reg clk = 0, clk200 = 0, rst_sys = 1, calib = 0;
    always #5.0 clk = ~clk;
    always #2.5 clk200 = ~clk200;
    int errors = 0;
    bit full;

    // ---------------- DDR model ----------------
    wire        app_en, app_wdf_wren, app_wdf_end, app_rdy, app_wdf_rdy, app_rd_data_valid;
    wire [2:0]  app_cmd; wire [28:0] app_addr; wire [255:0] app_wdf_data, app_rd_data; wire [31:0] app_wdf_mask;
    int mig_err; longint n_wr, n_rd;
    mig_app_model #(.RD_LAT(24), .RDY_PCT(85), .WDF_PCT(85), .SEED(3)) u_mig (
        .ui_clk(clk), .rst(rst_sys), .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end), .app_rdy(app_rdy),
        .app_wdf_rdy(app_wdf_rdy), .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid),
        .errors(mig_err), .n_wr(n_wr), .n_rd(n_rd));

    // ---------------- DUT ----------------
    reg         key1_n = 1;
    reg         dbg_v = 0; reg [63:0] dbg_cmd = 0;
    wire        hdmi_clk_p, hdmi_clk_n; wire [2:0] hdmi_data_p, hdmi_data_n; wire [1:0] led;
    wire [24*32-1:0] status_vec;
    tri0 [31:0] jm1, jm2;                              // board link headers: no partner (pull-downs)
    board_b_core #(.BUILD_ID(32'd77), .BRINGUP(1'b1), .DEFAULT_MODE(5), .SIM_MODE4(1'b1), .IMG_SET(1), .DEBOUNCE(100),
                   .LONG_PRESS(5000), .LOCK_TIMEOUT(200_000), .MS_CYCLES(1000), .JTAG(1'b1)) dut (
        .clk_sys(clk), .rst_sys(rst_sys), .calib_done(calib), .clk200(clk200), .key1_n(key1_n),
        .dbg_cmd_valid(dbg_v), .dbg_cmd(dbg_cmd),
        .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr), .app_wdf_data(app_wdf_data), .app_wdf_mask(app_wdf_mask),
        .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end), .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy),
        .app_rd_data_valid(app_rd_data_valid), .app_rd_data(app_rd_data),
        .hdmi_clk_p(hdmi_clk_p), .hdmi_clk_n(hdmi_clk_n), .hdmi_data_p(hdmi_data_p), .hdmi_data_n(hdmi_data_n),
        .jm1(jm1), .jm2(jm2), .led(led), .status_vec(status_vec));
    function automatic [31:0] sw(input int k); sw = status_vec[k*32 +: 32]; endfunction

    // ---------------- TMDS receiver ----------------
    wire ws, c_de, c_hs, c_vs; wire [7:0] c_r, c_g, c_b; int misalign, bad_ctl; longint words;
    tmds_capture u_cap (.ser_clk(dut.u_disp.ser_clk), .lane_clk(hdmi_clk_p), .lane_d(hdmi_data_p),
        .pix_clk(dut.u_disp.pix_clk), .w_b(dut.u_disp.u_dout.tmds_b), .w_g(dut.u_disp.u_dout.tmds_g),
        .w_r(dut.u_disp.u_dout.tmds_r), .ws(ws),
        .de(c_de), .hs(c_hs), .vs(c_vs), .r(c_r), .g(c_g), .b(c_b), .misalign(misalign), .bad_ctl(bad_ctl), .words(words));

    `include "b_frame_check.svh"

    // ---------------- stimulus ----------------
    task automatic cmd(input [63:0] c);
        @(posedge clk); dbg_v <= 1; dbg_cmd <= c;
        @(posedge clk); dbg_v <= 0;
    endtask
    task automatic present(input int i, input bit now = 0);
        cmd({8'h02, 39'h0, now, 6'h0, 10'(i)});
    endtask
    task automatic wait_idle();
        repeat (5) @(posedge clk);
        while (dut.g_bu.u_bu.busy || dut.u_disp.mode_busy) @(posedge clk);
    endtask
    // JTAG USER2 scan through the UNISIM BSCANE2 (glbl) — returns the value captured (the previous command)
    reg jt_tck = 1'b0, jt_tdi = 1'b0;                 // forced onto glbl's JTAG_TCK / JTAG_TDI wires
    task automatic jtag_tck();
        #50 jt_tck = 1'b1;
        #50 jt_tck = 1'b0;
    endtask
    // a DR scan of n bits (LSB first) on USER2; rb = the bits shifted out (the last accepted scan)
    task automatic jtag_user2(input [79:0] v, input int n, output [79:0] rb);
        rb = '0;
        glbl.JTAG_SEL2_GLBL = 1'b1; jtag_tck(); jtag_tck();
        glbl.JTAG_CAPTURE_GLBL = 1'b1; jtag_tck(); glbl.JTAG_CAPTURE_GLBL = 1'b0;
        glbl.JTAG_SHIFT_GLBL = 1'b1;
        for (int i = 0; i < n; i++) begin
            jt_tdi = v[i];
            #40 rb[i] = glbl.JTAG_USER_TDO2_GLBL;
            #10 jt_tck = 1'b1;
            #50 jt_tck = 1'b0;
        end
        glbl.JTAG_SHIFT_GLBL = 1'b0;
        glbl.JTAG_UPDATE_GLBL = 1'b1; jtag_tck(); glbl.JTAG_UPDATE_GLBL = 1'b0;
        jtag_tck(); glbl.JTAG_SEL2_GLBL = 1'b0;
    endtask

    initial begin
        reg [79:0] rb; int n0, i, f0, e0;
        full = $test$plusargs("FULL");
        force glbl.JTAG_TCK_GLBL = jt_tck; force glbl.JTAG_TDI_GLBL = jt_tdi;
        glbl.JTAG_CAPTURE_GLBL = 0; glbl.JTAG_SHIFT_GLBL = 0; glbl.JTAG_UPDATE_GLBL = 0;
        repeat (20) @(posedge clk); rst_sys <= 0;
        repeat (200) @(posedge clk); calib <= 1;
        // ---- boot: DRP to the test mode, black frames, fill, present image 8 ----
        wait_img_frames(-1, 1, 20);
        $display("ok: boot, black frame(s) before the first present (mode %0d)", dut.u_disp.cur_mode);
        wait_img_frames(8, 2, 40);
        $display("ok: boot present: image 8 (203x151, window) shown, fill passes %0d", sw(20));
        // ---- every image ----
        for (i = 0; i < 10; i++) begin wait_idle(); present(i); wait_img_frames(i, 2); end
        $display("ok: all 10 images presented and checked (%0d frames so far)", frames_ok);
        // ---- present at once ----
        wait_idle(); present(1); wait_img_frames(1, 1);
        @(posedge clk); while (!(st == 2 && v == 10)) @(posedge clk);
        alt = 3; present(3, 1); wait_img_frames(3, 2);
        $display("ok: present at once");
        // ---- KEY1 short press: next image ----
        wait_idle();
        key1_n <= 0; repeat (400) @(posedge clk); key1_n <= 1;
        wait_img_frames(4, 2);
        $display("ok: KEY1 short press -> image 4");
        // ---- JTAG USER2: scans without the signature or of another length are ignored ----
        n0 = sw(13);
        jtag_user2({16'h0000, 8'h02, 46'h0, 10'd6}, 64, rb);  // 64 bits: debug-core probe style
        jtag_user2({16'hB5C2, 8'h02, 46'h0, 10'd6}, 80, rb);  // wrong signature
        jtag_user2({16'hB5C3, 8'h02, 46'h0, 10'd6}, 81, rb);  // right signature, one bit too many
        repeat (20) @(posedge clk);
        if (sw(13) != n0) report($sformatf("JTAG: %0d bad scans accepted", sw(13) - n0));
        jtag_user2({16'hB5C3, 8'h02, 46'h0, 10'd5}, 80, rb);
        jtag_user2({16'hB5C3, 8'h00, 56'h0}, 80, rb);          // NOP: reads back the PRESENT command
        if (rb !== {16'hB5C3, 8'h02, 46'h0, 10'd5}) report($sformatf("JTAG read-back %020h", rb));
        wait_img_frames(5, 2);
        if (sw(13) != n0 + 2) report($sformatf("JTAG command count %0d, expected %0d", sw(13), n0 + 2));
        if (sw(16) == 0) report("NOP command not counted as an error");
        $display("ok: JTAG USER2 present + read-back, 3 malformed scans ignored, harness errors %0d (the NOP)", sw(16));
        // ---- LINKCTL (op 0x06, executed by board_b_core, ignored by the harness without an error) ----
        wait_idle(); e0 = sw(16);
        if (dut.linkctl !== 32'h1) report($sformatf("LINKCTL power-up %08h, expected 1", dut.linkctl));
        cmd({8'h06, 24'h0, 32'h0}); wait_idle();
        if (dut.linkctl !== 32'h0 || dut.u_link.fill_en !== 1'b0) report("LINKCTL 0 not applied");
        cmd({8'h06, 24'h0, 32'h1}); wait_idle();
        if (dut.linkctl !== 32'h1 || sw(16) != e0) report($sformatf("LINKCTL 1: %08h, harness errors %0d -> %0d", dut.linkctl, e0, sw(16)));
        else $display("ok: LINKCTL op 0x06 (fill words off / on), no harness error");
        // ---- MEMTEST ----
        wait_idle(); e0 = sw(16);
        cmd({8'h03, 10'd16, 10'd1, 4'h0, 32'h1234_5678}); wait_idle();
        if (sw(17) != 1 || sw(18) != 0) report($sformatf("memtest runs %0d errors %0d", sw(17), sw(18)));
        else $display("ok: MEMTEST 1 MB at 16 MB, 0 errors");
        // memtest over image 0, then REFILL restores it
        cmd({8'h03, 10'd256, 10'd1, 4'h0, 32'h0BAD_F00D}); wait_idle();
        cmd({8'h04, 56'h0}); wait_idle();
        if (sw(20) != 2) report($sformatf("fill passes %0d, expected 2", sw(20)));
        present(0); wait_img_frames(0, 2);
        $display("ok: MEMTEST over image 0 + REFILL");
        // ---- SET_MODE: same mode again, invalid mode ----
        n0 = sw(8);
        cmd({8'h01, 53'h0, 3'd5}); wait_idle();
        if (sw(8) != n0 + 1) report("SET_MODE 5 not counted");
        wait_img_frames(0, 2);
        n0 = sw(9);
        cmd({8'h01, 53'h0, 3'd7}); wait_idle();                   // 7: no such mode (5, 6 = simulation modes; 4 = 1024 x 768)
        if (sw(9) != n0 + 1) report("invalid mode not rejected");
        wait_img_frames(0, 1);
        $display("ok: SET_MODE (relock, invalid id rejected)");
        if (full) begin
            // ---- hardware modes: KEY1 long press = next mode (from 5 -> 0 -> 1 -> 2 -> 3) + present 2 x mode ----
            for (i = 0; i < 4; i++) begin
                wait_idle();
                if (i == 0) begin key1_n <= 0; repeat (6000) @(posedge clk); key1_n <= 1; end
                else cmd({8'h05, 55'h0, 1'b1});
                wait_idle();
                if (dut.u_disp.cur_mode != i) report($sformatf("long press: mode %0d, expected %0d", dut.u_disp.cur_mode, i));
                wait_img_frames(2 * i, 1, 3);
                $display("ok: mode %0d (%0dx%0d) full frame checked, pixel clock %0d kHz measured %0d per 10 us",
                         i, dut.u_disp.t_ha, dut.u_disp.t_va, vm_pix_khz(i), sw(6));
            end
            cmd({8'h01, 53'h0, 3'd5}); wait_idle(); wait_img_frames(6, 1);
        end
        // ---- final checks ----
        if (sw(0) != 32'hB000_0004 || sw(1) != 77) report($sformatf("status layout %08h build %0d", sw(0), sw(1)));   // layout 4: words 24-47 = b_render's profile counters
        if ((sw(3) & 32'h7) != 32'h7) report($sformatf("flags %08h: calib / locked / video", sw(3)));
        if (sw(5) != 0) report($sformatf("%0d line-buffer underruns", sw(5)));
        if (sw(6) < 399 || sw(6) > 401) report($sformatf("pixel clock %0d per 10 us, expected 400", sw(6)));
        if (sw(4) < frames_ok) report($sformatf("frame counter %0d < frames checked %0d", sw(4), frames_ok));
        if (bad_ctl != 0) report($sformatf("%0d bad TMDS control words", bad_ctl));
        if (mig_err != 0) report($sformatf("MIG model errors %0d", mig_err));
        $display("INFO_TB frames %0d words %0d misalign(at restarts) %0d frames_counter %0d frame_cyc %0d presents %0d dropped %0d",
                 frames_ok, words, misalign, sw(4), sw(7), sw(10), sw(11));
        if (errors == 0) $display("TB_PASS tb_board_b%s", full ? " (FULL)" : "");
        else $display("TB_FAIL tb_board_b errors=%0d", errors);
        $finish;
    end
    initial begin #(400_000_000.0); $display("TB_FAIL tb_board_b timeout"); $finish; end
endmodule
`default_nettype wire
