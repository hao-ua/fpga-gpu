`timescale 1ns/1ps
// tb_board_b3d — board B's 3D path end to end: board_b_core (DDR model, word-level HDMI) and an
// A-side link_core joined by the JM1 / JM2 cables of tb_link_credit. The bench sends, as board A would, the CMD packets
// of tools/b_ref.py frame <seed> (bl_cmds.hex: CREATE_BUFFER / WRITE_BUFFER of the fragment programs, MEM_WRITE of
// texels and the target's initial image, CREATE_IMAGE of target and textures, DRAW_STATE of every draw, FLUSH, FENCE)
// and the frame's render stream (br_stream.hex) in RSTREAM packets, each class paced by B's own consumed counters
// (credit). Checks: no EV_ERROR, the FENCE event after FLUSH, the RCREDIT packets arrive, and the target image in B's
// DDR equals br_expect.hex bit for bit.
`default_nettype none
module tb_board_b3d;
    import fb_pkg::*;
    reg clkA = 0, clk200A = 0, clkB = 0, clk200B = 0;
    always #5.0    clkA = ~clkA;
    always #2.5    clk200A = ~clk200A;
    always #4.9998 clkB = ~clkB;
    always #2.4999 clk200B = ~clk200B;
    reg  rstA = 1, rstB = 1, calB = 0;
    int  errors = 0;

    // ---------------- cables (tb_link_credit) ----------------
    wire [31:0] cab1A, cab1B, cab2A, cab2B;
    reg  [31:0] cab1B_r = 0, cab2A_r = 0, cab2B_r = 0;
    real d1 [0:31], d2 [0:31];
    genvar g;
    generate
        for (g = 0; g < 32; g++) begin : g_c1
            pulldown (cab1A[g]);
            assign cab1B[g] = cab1B_r[g];
            always @(cab1A[g]) cab1B_r[g] <= #(d1[g]) cab1A[g];
        end
        for (g = 0; g < 22; g++) begin : g_c2b
            pulldown (cab2B[g]);
            assign cab2A[g] = cab2A_r[g];
            always @(cab2B[g]) cab2A_r[g] <= #(d2[g]) cab2B[g];
        end
        for (g = 22; g < 32; g++) begin : g_c2a
            pulldown (cab2A[g]);
            assign cab2B[g] = cab2B_r[g];
            always @(cab2A[g]) cab2B_r[g] <= #(d2[g]) cab2A[g];
        end
    endgenerate
    initial begin
        for (int k = 0; k < 32; k++) begin
            d1[k] = 1.0 + 0.0137 * ((k * 7) % 16);
            d2[k] = (k < 22) ? 2.3 + 0.0137 * ((k * 5) % 16) : 1.0 + 0.0137 * ((k * 3) % 16);
        end
        d1[20] = 2.6; d2[20] = 1.0;
    end

    // ---------------- board A side: link_core role A, driven by the bench ----------------
    wire        lclkA, lclk90A, lclk270A, rst_lclkA;
    link_clk #(.LINK_MHZ(100)) u_clkA (.clk200(clk200A), .rst(rstA), .lclk(lclkA), .lclk90(lclk90A), .lclk270(lclk270A),
        .rst_lclk(rst_lclkA), .locked());
    reg         a_tx_valid = 0, a_tx_cnt2 = 0, a_tx_last = 0; reg [63:0] a_tx_data = 0; wire a_tx_ready;
    wire        a_rx_valid, a_rx_cnt2, a_rx_last, a_rx_hdr; wire [63:0] a_rx_data;
    wire        a_trained, a_far_up;
    link_core #(.ROLE("A"), .LINK_MHZ(100), .MMCM_RST_CYCLES(20_000), .GUARD_CYCLES(3000), .IDLE_TIMEOUT(200_000),
                .TRAIN_WORDS(256), .RETRY_CYCLES(20_000), .RELOCK_CYCLES(50_000), .RETRAIN_WINDOW(200_000),
                .SIM_SHIFTER(1'b1)) lka (
        .clk(clkA), .rst(rstA), .lclk(lclkA), .lclk90(lclk90A), .lclk270(lclk270A), .rst_lclk(rst_lclkA), .fill_en(1'b1),
        .tx_valid(a_tx_valid), .tx_data(a_tx_data), .tx_cnt2(a_tx_cnt2), .tx_last(a_tx_last), .tx_ready(a_tx_ready),
        .rx_valid(a_rx_valid), .rx_data(a_rx_data), .rx_cnt2(a_rx_cnt2), .rx_last(a_rx_last), .rx_hdr(a_rx_hdr),
        .rx_ready(1'b1), .pkt_done(), .pkt_ok(),
        .lock(), .trained(a_trained), .rx_clk_ok(), .far_up(a_far_up), .training(), .guard_fail(), .tx_enabled(),
        .xoff_events(), .eye_width(), .phase(), .train_cnt(), .tx_pkts(), .rx_words(), .rx_pkts(), .par_errs(), .crc_errs(),
        .seq_errs(), .frm_errs(), .lock_drops(), .par_map(), .retx(), .disc(), .diag(), .jm1(cab1A), .jm2(cab2A));

    // ---------------- board B ----------------
    wire        ben, bwren, bend, brdy, bwrdy, brv; wire [2:0] bcmd; wire [28:0] baddr; wire [255:0] bwd, brd;
    wire [31:0] bmask;
    int mig_err; longint n_wr, n_rd;
    mig_app_model #(.RD_LAT(20), .RDY_PCT(85), .WDF_PCT(85), .SEED(5)) mig (.ui_clk(clkB), .rst(rstB), .app_en(ben),
        .app_cmd(bcmd), .app_addr(baddr), .app_wdf_data(bwd), .app_wdf_mask(bmask), .app_wdf_wren(bwren),
        .app_wdf_end(bend), .app_rdy(brdy), .app_wdf_rdy(bwrdy), .app_rd_data(brd), .app_rd_data_valid(brv),
        .errors(mig_err), .n_wr(n_wr), .n_rd(n_rd));
    wire hdmi_clk_p, hdmi_clk_n; wire [2:0] hdmi_data_p, hdmi_data_n; wire [1:0] ledB; wire [24*32-1:0] bstat;
    board_b_core #(.BUILD_ID(32'd79), .DEFAULT_MODE(5), .SIM_MODE4(1'b1), .IMG_SET(1), .DEBOUNCE(100),
                   .LONG_PRESS(5000), .LOCK_TIMEOUT(200_000), .MS_CYCLES(1000), .JTAG(1'b0),
                   .LINK_MHZ(100), .LINK_GUARD(3000), .LINK_MMCM_RST(20_000), .LINK_IDLE_TO(200_000), .LINK_TRAIN_WORDS(256),
                   .LINK_RETRY(20_000), .LINK_RELOCK(50_000), .LINK_RWIN(200_000), .LSTAT_CYCLES(10_000),
                   .LINK_SIM_SHIFTER(1'b1), .B3D_NLINES(4), .B3D_MAXW(1280)) dut (   // the board's MAXW: 8-line bands up to 640 wide need its band buffer
        .clk_sys(clkB), .rst_sys(rstB), .calib_done(calB), .clk200(clk200B), .key1_n(1'b1),
        .dbg_cmd_valid(1'b0), .dbg_cmd(64'h0),
        .app_en(ben), .app_cmd(bcmd), .app_addr(baddr), .app_wdf_data(bwd), .app_wdf_mask(bmask),
        .app_wdf_wren(bwren), .app_wdf_end(bend), .app_rdy(brdy), .app_wdf_rdy(bwrdy),
        .app_rd_data_valid(brv), .app_rd_data(brd),
        .hdmi_clk_p(hdmi_clk_p), .hdmi_clk_n(hdmi_clk_n), .hdmi_data_p(hdmi_data_p), .hdmi_data_n(hdmi_data_n),
        .jm1(cab1B), .jm2(cab2B), .led(ledB), .status_vec(bstat));

    // ---------------- A-side receiver: B's packets (one word per beat on the backward lane) ----------------
    int n_fence = 0, n_ev = 0, n_rcredit = 0, n_credit = 0, n_status = 0;
    reg [31:0] rw [0:63]; int rn = 0; reg [7:0] rtype;
    always @(posedge clkA) if (a_rx_valid) begin
        if (a_rx_hdr) begin rtype = a_rx_data[31:24]; rn = 0; if (a_rx_cnt2) begin rw[rn] = a_rx_data[63:32]; rn++; end end
        else begin rw[rn] = a_rx_data[31:0]; rn++; if (a_rx_cnt2) begin rw[rn] = a_rx_data[63:32]; rn++; end end
        if (a_rx_last) begin
            case (rtype)
                L_EVENT: begin
                    n_ev++;
                    if (rw[1] == {24'h0, EV_FENCE} && rw[2] == 32'h5EED) n_fence++;
                    else if (rw[1] == {24'h0, EV_ERROR}) begin
                        errors++; $display("  EV_ERROR: seq %0d code %0d type %h detail %h", rw[0], rw[2], rw[3], rw[4]);
                    end
                end
                L_RCREDIT: n_rcredit++;
                L_CREDIT:  n_credit++;
                L_STATUS:  n_status++;
                default: ;
            endcase
        end
    end

    // ---------------- A-side sender ----------------
    task automatic send_beat(input [63:0] d, input bit cnt2, input bit last);
        a_tx_valid <= 1'b1; a_tx_data <= d; a_tx_cnt2 <= cnt2; a_tx_last <= last;
        do @(posedge clkA); while (!a_tx_ready);
        a_tx_valid <= 1'b0;
    endtask
    // a packet of n payload words: header beat {w0, header}, then pairs
    task automatic send_pkt(input [7:0] t, input reg [31:0] w [], input int n);
        send_beat({(n > 0) ? w[0] : 32'h0, t, 8'h00, 16'(n)}, n > 0, n <= 1);
        for (int k = 1; k < n; k += 2) send_beat({(k + 1 < n) ? w[k + 1] : 32'h0, w[k]}, k + 1 < n, k + 2 >= n);
    endtask

    longint cmd_sent = 0, rs_sent = 0;
    initial begin
        int fd, r, n, nb, k; reg [31:0] w []; reg [63:0] beats [$]; reg [63:0] b; longint t0;
        repeat (20) @(posedge clkA); rstA <= 0; rstB <= 0;
        repeat (200) @(posedge clkB); calB <= 1;
        fd = $fopen("tools/gen/br_stream.hex", "r");
        while ($fscanf(fd, "%h", b) == 1) beats.push_back(b);
        $fclose(fd);
        // the link up both ways (B's STATUS packets arrive)
        t0 = $time;
        while (!(a_trained && a_far_up && n_status > 1)) begin
            @(posedge clkA);
            if ($time - t0 > 20ms) begin $display("TB_FAIL tb_board_b3d: link did not come up"); $finish; end
        end
        $display("tb_board_b3d: link up at %0t", $time);
        // ---- commands (credit: B's command buffer, 4096 words) ----
        fd = $fopen("tools/gen/bl_cmds.hex", "r");
        forever begin
            r = $fscanf(fd, "%h", n);
            if (r != 1 || n == 0) break;
            w = new[n];
            for (int i = 0; i < n; i++) r = $fscanf(fd, "%h", w[i]);
            while (cmd_sent + n - dut.u_lport.consumed > 4096) @(posedge clkA);
            send_pkt(L_CMD, w, n); cmd_sent += n;
        end
        $fclose(fd);
        $display("tb_board_b3d: %0d command words sent at %0t", cmd_sent, $time);
        // ---- render stream (credit: B's render buffer, 2048 beats), 120 beats per packet ----
        k = 0;
        while (k < beats.size()) begin
            nb = (beats.size() - k > 120) ? 120 : beats.size() - k;
            while (rs_sent + nb - dut.u_lport.rs_consumed > 2048) @(posedge clkA);
            w = new[1 + 2 * nb];
            w[0] = 32'h0;
            for (int i = 0; i < nb; i++) begin w[1 + 2 * i] = beats[k + i][31:0]; w[2 + 2 * i] = beats[k + i][63:32]; end
            send_pkt(L_RSTREAM, w, 1 + 2 * nb); rs_sent += nb; k += nb;
        end
        $display("tb_board_b3d: %0d render beats sent at %0t", rs_sent, $time);
        t0 = $time;
        while (n_fence == 0) begin
            @(posedge clkA);
            if ($time - t0 > 20ms) begin $display("TB_FAIL tb_board_b3d: no FENCE event (%0d events)", n_ev); $finish; end
        end
        $display("tb_board_b3d: FENCE back at %0t: B quads %0d, frames %0d, draws %0d, RCREDIT %0d CREDIT %0d", $time,
                 dut.rd_quads, dut.rd_frames, dut.cp_draws, n_rcredit, n_credit);
        // ---- the target (and the depth-stencil image) in B's DDR ----
        begin
            reg [31:0] pv [0:23]; reg [51:0] c16; int W, H, bpp; reg [31:0] e; longint base;
            fd = $fopen("tools/gen/br_params.hex", "r");
            r = $fscanf(fd, "%h %h %h %h %h", pv[0], pv[1], pv[2], pv[3], c16);
            for (int i = 5; i < 24; i++) r = $fscanf(fd, "%h", pv[i]);
            $fclose(fd);
            base = pv[0]; W = pv[2]; H = pv[3]; bpp = (pv[16] == 0) ? 4 : 2;
            $display("tb_board_b3d: target format %0d, depth %0d bits, depth-stencil image %0d", pv[16], pv[17] ? 16 : 24, pv[18]);
            fd = $fopen("tools/gen/br_expect.hex", "r");
            for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) begin
                reg [255:0] bur; reg [31:0] got; longint wa;
                r = $fscanf(fd, "%h", e);
                wa = base + y * pv[1] * 8 + (bpp == 4 ? x : x / 2);
                bur = mig.mem.exists(wa >> 3) ? mig.mem[wa >> 3] : 256'h0;
                got = (bpp == 4) ? bur[(wa & 7) * 32 +: 32] : {16'd0, bur[(wa & 7) * 32 + (x & 1) * 16 +: 16]};
                if (got !== e) begin errors++; if (errors < 10) $display("  pixel (%0d, %0d): %h, expected %h", x, y, got, e); end
            end
            $fclose(fd);
            if (pv[18]) begin
                fd = $fopen("tools/gen/br_expect_ds.hex", "r");
                for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) begin
                    reg [255:0] bur; reg [31:0] got; longint wa;
                    r = $fscanf(fd, "%h", e);
                    wa = longint'(pv[19]) + y * pv[20] * 8 + x;
                    bur = mig.mem.exists(wa >> 3) ? mig.mem[wa >> 3] : 256'h0;
                    got = bur[(wa & 7) * 32 +: 32];
                    if (got !== e) begin errors++; if (errors < 10) $display("  depth-stencil (%0d, %0d): %h, expected %h", x, y, got, e); end
                end
                $fclose(fd);
            end
        end
        if (n_rcredit == 0) begin errors++; $display("  no RCREDIT packet"); end
        if (mig_err != 0) begin errors++; $display("  MIG model errors %0d", mig_err); end
        $display("tb_board_b3d: errors = %0d", errors);
        if (errors == 0) $display("TB_PASS tb_board_b3d"); else $display("TB_FAIL tb_board_b3d");
        $finish;
    end
    initial begin #200ms; $display("TB_FAIL tb_board_b3d: TIMEOUT"); $finish; end
endmodule
`default_nettype wire
