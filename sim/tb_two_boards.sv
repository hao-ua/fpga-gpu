`timescale 1ns/1ps
// tb_two_boards — the whole remote-display path: host model (fb protocol over the
// RGMII PHY model, Go-Back-N) -> board_a_core -> JM1/JM2 cables (per-wire delays, as tb_board_link) ->
// board_b_core -> HDMI, decoded by tmds_capture and checked word by word by b_frame_check.svh.
// The two boards run on slightly different oscillators. Checks:
//   * board link trains both ways, B's STATUS packets make A's link_ok;
//   * SET_MODE over Ethernet: 64x48 -> 80x40 (fractional MMCM) -> 64x48, EV_MODE back to the host, frames
//     of the new mode checked; an invalid id -> EV_ERROR ERR_RANGE;
//   * PRESENT over Ethernet of test images (handles 0xC000 + i): at vsync, at once, with hold followed by a
//     FENCE (event order), two presents in one frame (the first reported dropped), bad handles;
//   * FENCE ordering after an A-local MEMTEST; A's STATUS words 37-47 (B and link);
//   * board B memory over Ethernet: MEM_WRITE (unaligned), MEM_READ in several READ_DATA datagrams relayed by A,
//     MEMTEST, misaligned MEMTEST rejected;
//   * CREATE_IMAGE: every descriptor check (handle, format, size, pitch, levels, layout, heap); images created
//     and filled by WRITE_IMAGE in several transfer formats, split over datagrams at any pixel, into RGBA8 and
//     RGB565 storage, top-left and bottom-left; B's DDR compared byte for byte with the bench's conversion
//     model, the images presented and every displayed pixel checked; mip levels / layers addressing (also read
//     back with MEM_READ); WRITE_IMAGE rejects; DESTROY;
//   * +FULL: every hardware mode 0..4 over Ethernet with a complete frame each (4 = 1024 x 768).
// Prints TB_PASS / TB_FAIL.
`default_nettype none
module tb_two_boards;
    import net_pkg::*;
    import fb_pkg::*;
`include "video_modes.svh"
    reg clkA = 0, clk200A = 0, clkB = 0, clk200B = 0;
    always #5.0    clkA = ~clkA;
    always #2.5    clk200A = ~clk200A;
    always #4.9998 clkB = ~clkB;                  // board B's oscillator is 40 ppm fast
    always #2.4999 clk200B = ~clk200B;
    wire clk = clkB;                              // b_frame_check.svh runs on board B's clock
    reg  rstA = 1, rstB = 1, calA = 0, calB = 0;
    int  errors = 0;
    bit  full;

    // ---------------- cables (tb_board_link) ----------------
    wire [31:0] cab1A, cab1B, cab2A, cab2B;
    reg  [31:0] cab1B_r = 0, cab2A_r = 0, cab2B_r = 0;
    real d1 [0:31], d2 [0:31];
    genvar g;
    generate
        for (g = 0; g < 32; g++) begin : g_c1          // JM1: A drives
            pulldown (cab1A[g]);
            assign cab1B[g] = cab1B_r[g];
            always @(cab1A[g]) cab1B_r[g] <= #(d1[g]) cab1A[g];
        end
        for (g = 0; g < 22; g++) begin : g_c2b         // JM2[21:0]: B drives
            pulldown (cab2B[g]);
            assign cab2A[g] = cab2A_r[g];
            always @(cab2B[g]) cab2A_r[g] <= #(d2[g]) cab2B[g];
        end
        for (g = 22; g < 32; g++) begin : g_c2a        // JM2[31:22]: A drives
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
        d1[20] = 2.6;                                   // forward clock late against its data
        d2[20] = 1.0;                                   // backward clock early against its data
    end

    // ---------------- board A ----------------
    localparam [47:0] BMAC = 48'h00_0a_35_01_02_03, HMAC = 48'h02_00_00_00_00_01;
    localparam [31:0] BIP  = 32'hC0A8010A, HIP = 32'hC0A80170;
    localparam [15:0] HPORT = 16'd40000;
    reg         giga = 1; reg [15:0] physr = 16'h0000;
    wire rxc, rx_ctl, txc, tx_ctl, phy_rstn, mdc, mdio_o, mdio_oe; wire [3:0] rxd, txd;
    tri1 mdio;
    assign mdio = mdio_oe ? mdio_o : 1'bz;
    rgmii_phy_model u_phy (.giga(giga), .rgmii_rxc(rxc), .rgmii_rx_ctl(rx_ctl), .rgmii_rxd(rxd),
        .rgmii_txc(txc), .rgmii_tx_ctl(tx_ctl), .rgmii_txd(txd), .mdc(mdc), .mdio(mdio), .physr(physr));
    wire        aen, awren, aend, ardy, awrdy, arv; wire [2:0] acmd; wire [28:0] aaddr; wire [255:0] awd, ard; wire [31:0] amask;
    int a_mig_err; longint a_nwr, a_nrd;
    mig_app_model #(.RD_LAT(24), .RDY_PCT(80), .WDF_PCT(85), .SEED(3)) u_migA (
        .ui_clk(clkA), .rst(rstA), .app_en(aen), .app_cmd(acmd), .app_addr(aaddr), .app_wdf_data(awd), .app_wdf_mask(amask),
        .app_wdf_wren(awren), .app_wdf_end(aend), .app_rdy(ardy), .app_wdf_rdy(awrdy), .app_rd_data(ard),
        .app_rd_data_valid(arv), .errors(a_mig_err), .n_wr(a_nwr), .n_rd(a_nrd));
    wire [1:0] ledA;
    // link start-up times as tb_board_link; B counts as alive 1 ms after its last STATUS (every 100 us here)
    board_a_core #(.BUILD_ID(32'h0000_0A02), .IDELAY_DEFAULT(5'd3), .RING_LOG2(10), .MDIO_RST_MS(1), .MDIO_WAIT_MS(1),
                   .MDIO_POLL_US(100), .STATUS_PERIOD(300_000), .TRIAL_CYCLES(50_000), .PHY_TRIAL_CYCLES(600_000),
                   .SNAP_CYCLES(2000), .JTAG_STATUS(1'b0), .DHCP(1'b0),   // the fixed address at once (tb_dhcp tests DHCP)
                   .LINK_MHZ(100), .LINK_GUARD(3000), .LINK_MMCM_RST(20_000), .LINK_IDLE_TO(200_000), .LINK_TRAIN_WORDS(256),
                   .LINK_RETRY(20_000), .LINK_RELOCK(50_000), .LINK_RWIN(200_000), .KEEPALIVE(20_000), .B_ALIVE(100_000),
                   .LINK_SIM_SHIFTER(1'b1)) da (
        .clk_sys(clkA), .rst_sys(rstA), .calib_done(calA), .clk_idelay(clk200A),
        .rgmii_rxc(rxc), .rgmii_rx_ctl(rx_ctl), .rgmii_rxd(rxd), .rgmii_txc(txc), .rgmii_tx_ctl(tx_ctl), .rgmii_txd(txd),
        .phy_rstn(phy_rstn), .mdc(mdc), .mdio_o(mdio_o), .mdio_oe(mdio_oe), .mdio_i(mdio),
        .app_en(aen), .app_cmd(acmd), .app_addr(aaddr), .app_wdf_data(awd), .app_wdf_mask(amask),
        .app_wdf_wren(awren), .app_wdf_end(aend), .app_rdy(ardy), .app_wdf_rdy(awrdy),
        .app_rd_data_valid(arv), .app_rd_data(ard), .jm1(cab1A), .jm2(cab2A), .led(ledA));

    // ---------------- board B ----------------
    wire        ben, bwren, bend, brdy, bwrdy, brv; wire [2:0] bcmd; wire [28:0] baddr; wire [255:0] bwd, brd; wire [31:0] bmask;
    int b_mig_err; longint b_nwr, b_nrd;
    mig_app_model #(.RD_LAT(24), .RDY_PCT(85), .WDF_PCT(85), .SEED(5)) u_migB (
        .ui_clk(clkB), .rst(rstB), .app_en(ben), .app_cmd(bcmd), .app_addr(baddr), .app_wdf_data(bwd), .app_wdf_mask(bmask),
        .app_wdf_wren(bwren), .app_wdf_end(bend), .app_rdy(brdy), .app_wdf_rdy(bwrdy), .app_rd_data(brd),
        .app_rd_data_valid(brv), .errors(b_mig_err), .n_wr(b_nwr), .n_rd(b_nrd));
    wire hdmi_clk_p, hdmi_clk_n; wire [2:0] hdmi_data_p, hdmi_data_n; wire [1:0] ledB;
    wire [24*32-1:0] bstat;
    reg         b_dbg_v = 1'b0; reg [63:0] b_dbg_cmd = 64'h0;    // board B harness command port (stress test)
    board_b_core #(.BUILD_ID(32'd78), .BRINGUP(1'b1), .DEFAULT_MODE(5), .SIM_MODE4(1'b1), .IMG_SET(1), .DEBOUNCE(100),
                   .LONG_PRESS(5000), .LOCK_TIMEOUT(200_000), .MS_CYCLES(1000), .JTAG(1'b0),
                   .LINK_MHZ(100), .LINK_GUARD(3000), .LINK_MMCM_RST(20_000), .LINK_IDLE_TO(200_000), .LINK_TRAIN_WORDS(256),
                   .LINK_RETRY(20_000), .LINK_RELOCK(50_000), .LINK_RWIN(200_000), .LSTAT_CYCLES(10_000),
                   .LINK_SIM_SHIFTER(1'b1)) dut (
        .clk_sys(clkB), .rst_sys(rstB), .calib_done(calB), .clk200(clk200B), .key1_n(1'b1),
        .dbg_cmd_valid(b_dbg_v), .dbg_cmd(b_dbg_cmd),
        .app_en(ben), .app_cmd(bcmd), .app_addr(baddr), .app_wdf_data(bwd), .app_wdf_mask(bmask),
        .app_wdf_wren(bwren), .app_wdf_end(bend), .app_rdy(brdy), .app_wdf_rdy(bwrdy),
        .app_rd_data_valid(brv), .app_rd_data(brd),
        .hdmi_clk_p(hdmi_clk_p), .hdmi_clk_n(hdmi_clk_n), .hdmi_data_p(hdmi_data_p), .hdmi_data_n(hdmi_data_n),
        .jm1(cab1B), .jm2(cab2B), .led(ledB), .status_vec(bstat));
    function automatic [31:0] bsw(input int k); bsw = bstat[k*32 +: 32]; endfunction

    // ---------------- HDMI receiver + frame checker ----------------
    wire ws, c_de, c_hs, c_vs; wire [7:0] c_r, c_g, c_b; int misalign, bad_ctl; longint words;
    tmds_capture u_cap (.ser_clk(dut.u_disp.ser_clk), .lane_clk(hdmi_clk_p), .lane_d(hdmi_data_p),
        .pix_clk(dut.u_disp.pix_clk), .w_b(dut.u_disp.u_dout.tmds_b), .w_g(dut.u_disp.u_dout.tmds_g),
        .w_r(dut.u_disp.u_dout.tmds_r), .ws(ws),
        .de(c_de), .hs(c_hs), .vs(c_vs), .r(c_r), .g(c_g), .b(c_b), .misalign(misalign), .bad_ctl(bad_ctl), .words(words));

    // ---------------- host-created images: bench model (level 0, layer 0 as stored) ----------------
    // fmt: 0 RGBA8, 1 RGB565, 2 P8 (storage format codes 0x00 / 0x04 / 0x0C)
    typedef struct { int w, h, pitch, levels, layers; int fmt; bit org; longint base; } himg_t;
    himg_t     himg [int];                       // by image table index
    bit [31:0] hpix [int][int];                  // [index][row * w + col] = stored pixel value (unwritten: 0)
    function automatic int fbpp(input int fmt); return (fmt == 0) ? 4 : (fmt == 1) ? 2 : 1; endfunction
    function automatic bit host_img_known(input int idx); return himg.exists(idx); endfunction
    function automatic [23:0] host_img_rgb(input int idx, input int x, input int y, input int bank);
        himg_t m; int row; bit [31:0] v;
        m = himg[idx];
        if (x >= m.w || y >= m.h) return 24'h0;
        row = m.org ? m.h - 1 - y : y;
        v = hpix[idx].exists(row * m.w + x) ? hpix[idx][row * m.w + x] : 32'h0;
        if (m.fmt == 1) return {v[15:11], v[15:13], v[10:5], v[10:9], v[4:0], v[4:2]};   // scanout: replication
        if (m.fmt == 2) return dut.u_disp.u_dout.u_post.pal[{bank[0], v[7:0]}];         // the frame's palette bank
        return v[31:8];                          // stored 0xRRGGBBAA
    endfunction
    `define FC_HOST_IMAGES
    `include "b_frame_check.svh"

    // ---------------- host model (as tb_board_a) ----------------
    task automatic fail(input string m); report(m); endtask
    typedef byte unsigned bq [$];
    function automatic bq be16(input [15:0] v); bq q; q.push_back(v[15:8]); q.push_back(v[7:0]); return q; endfunction
    function automatic bq be32(input [31:0] v); return {be16(v[31:16]), be16(v[15:0])}; endfunction
    function automatic bq le16(input [15:0] v); bq q; q.push_back(v[7:0]); q.push_back(v[15:8]); return q; endfunction
    function automatic bq le32(input [31:0] v); return {le16(v[15:0]), le16(v[31:16])}; endfunction
    function automatic bq mac(input [47:0] m); bq q; for (int i = 5; i >= 0; i--) q.push_back(m[8*i +: 8]); return q; endfunction
    function automatic [15:0] csum(input bq d);
        bit [31:0] s = 0;
        for (int i = 0; i < d.size(); i += 2) s += {d[i], (i + 1 < d.size()) ? d[i+1] : 8'h00};
        while (s >> 16) s = (s & 16'hFFFF) + (s >> 16);
        return ~s[15:0];
    endfunction
    function automatic [63:0] gbe(input bq q, input int i, input int n);
        bit [63:0] v = 0; for (int k = 0; k < n; k++) v = (v << 8) | q[i + k]; return v;
    endfunction
    function automatic [31:0] gle32(input bq q, input int i);
        return {q[i+3], q[i+2], q[i+1], q[i]};
    endfunction
    function automatic bq udp_frame(input bq pl);
        bq u, h; bit [15:0] c;
        u = {be16(HPORT), be16(16'd5000), be16(8 + pl.size()), be16(16'h0000), pl};
        h = {8'h45, 8'h00, be16(20 + u.size()), be16(16'h0001), be16(16'h4000), 8'd64, 8'd17, be16(16'h0), be32(HIP), be32(BIP)};
        c = csum(h); h[10] = c[15:8]; h[11] = c[7:0];
        h = {mac(BMAC), mac(HMAC), be16(16'h0800), h, u};
        while (h.size() < 60) h.push_back(0);
        return h;
    endfunction
    function automatic bq msg(input [7:0] t, input [31:0] seq, input bq body);
        return {le16(FB_MAGIC), FB_VERSION, t, le32(seq), body};
    endfunction
    // replies
    int        n_ack = 0, n_status = 0;
    bit [31:0] ack_exp, ack_flags, ack_free;
    bit [31:0] st_w [0:STATUS_WORDS-1];
    bit [31:0] ev_seq [$], ev_code [$], ev_a0 [$], ev_a1 [$], ev_a2 [$];
    byte unsigned rd_bytes [bit [31:0]][int];    // READ_DATA bytes by tag and offset
    int        rd_dgrams = 0;
    event      ack_ev;
    always begin
        bq f, pl;
        @(posedge clkA);
        while (u_phy.txq.size() > 0) begin
            f = u_phy.txq.pop_front();
            f = f[8:f.size() - 5];
            if (gbe(f, 12, 2) != 16'h0800 || f[23] != 8'd17) continue;
            pl = f[42:42 + gbe(f, 38, 2) - 9];
            case (pl[3])
                R_ACK: begin ack_exp = gle32(pl, 8); ack_flags = gle32(pl, 12); ack_free = gle32(pl, 16); n_ack++; -> ack_ev; end
                R_STATUS: begin for (int i = 0; i < STATUS_WORDS; i++) st_w[i] = gle32(pl, 8 + 4*i); n_status++; end
                R_EVENT: begin ev_seq.push_back(gle32(pl, 4)); ev_code.push_back(gle32(pl, 8)); ev_a0.push_back(gle32(pl, 12));
                               ev_a1.push_back(gle32(pl, 16)); ev_a2.push_back(gle32(pl, 20)); end
                R_READ_DATA: begin
                    bit [31:0] tag, off;
                    tag = gle32(pl, 8); off = gle32(pl, 12);
                    for (int i = 16; i < pl.size(); i++) rd_bytes[tag][off + i - 16] = pl[i];
                    rd_dgrams++;
                end
                default: ;
            endcase
        end
    end
    // Go-Back-N sender
    bit [31:0] next_seq = 1;
    bq         sent_q [$];
    bit [31:0] sent_seq [$];
    localparam int WINDOW = 8;
    task automatic send_raw(input bq m); u_phy.send_frame(udp_frame(m)); endtask
    task automatic send_seq(input [7:0] t, input bq body, output bit [31:0] seq);
        seq = next_seq; next_seq++;
        while (sent_q.size() >= WINDOW) @(posedge clkA);
        begin bq m = msg(t, seq, body); sent_q.push_back(m); sent_seq.push_back(seq); send_raw(m); end
    endtask
    longint last_progress = 0; int unsigned cyc = 0;
    always @(posedge clkA) cyc++;
    initial forever begin
        @(ack_ev);
        while (sent_seq.size() > 0 && $signed(sent_seq[0] - ack_exp) < 0) begin
            void'(sent_q.pop_front()); void'(sent_seq.pop_front()); last_progress = cyc;
        end
        if ((ack_flags[ACKF_NAK] || ack_flags[ACKF_FULL]) && sent_q.size() > 0) begin
            if (ack_flags[ACKF_FULL]) repeat (2000) @(posedge clkA);
            for (int i = 0; i < sent_q.size(); i++) send_raw(sent_q[i]);
        end
    end
    initial forever begin
        repeat (1000) @(posedge clkA);
        if (sent_q.size() > 0 && cyc - last_progress > 60000) begin
            last_progress = cyc; for (int i = 0; i < sent_q.size(); i++) send_raw(sent_q[i]);
        end
    end
    // wait for the next EVENT and check it; returns a1 / a2
    task automatic expect_event(input [31:0] seq, input [31:0] code, input [31:0] a0, input bit chk_a0,
                                input int tmo_us = 3000, output bit [31:0] a1, output bit [31:0] a2);
        int t = 0;
        a1 = 0; a2 = 0;
        while (ev_seq.size() == 0 && t < tmo_us * 100) begin @(posedge clkA); t++; end
        if (ev_seq.size() == 0) begin fail($sformatf("no EVENT for seq %0d (code %h)", seq, code)); return; end
        if (ev_seq[0] != seq || ev_code[0] != code || (chk_a0 && ev_a0[0] != a0))
            fail($sformatf("EVENT seq %0d code %h a0 %h a1 %h a2 %h (expected seq %0d code %h a0 %h)",
                           ev_seq[0], ev_code[0], ev_a0[0], ev_a1[0], ev_a2[0], seq, code, a0));
        a1 = ev_a1[0]; a2 = ev_a2[0];
        void'(ev_seq.pop_front()); void'(ev_code.pop_front()); void'(ev_a0.pop_front()); void'(ev_a1.pop_front()); void'(ev_a2.pop_front());
    endtask
    realtime t_link_up = 0;
    always @(posedge da.link_ok) if (t_link_up == 0) t_link_up = $realtime;
    function automatic bq present_body(input [15:0] handle, input [15:0] flags, input [31:0] fid);
        return {le16(handle), le16(flags), le32(fid)};
    endfunction
    int n_fwd = 0, n_bev = 0;                          // commands forwarded to B / B events, for the STATUS check
    task automatic set_mode_eth(input int m, input int tmo_us = 5000);
        bit [31:0] s, a1, a2;
        send_seq(T_SET_MODE, le32(m), s); n_fwd++; n_bev++;
        expect_event(s, EV_MODE, m, 1, tmo_us, a1, a2);
        if (dut.u_disp.cur_mode != m) fail($sformatf("B mode %0d after SET_MODE %0d", dut.u_disp.cur_mode, m));
    endtask
    // a present becomes current at the next frame start (scan_reader): tmo_us must cover one frame of the running mode
    // (the bench's small mode 5: well under 3 ms; the hardware modes 0..4 of the FULL loop: 16.7 ms at 60 Hz)
    task automatic present_eth(input int i, input [15:0] flags, input [31:0] fid, input int tmo_us = 3000);
        bit [31:0] s, a1, a2;
        if (flags[0]) alt = i;                           // at once: lines of the current frame may switch to it
        send_seq(T_PRESENT, present_body(16'hC000 | 16'(i), flags, fid), s); n_fwd++; n_bev++;
        expect_event(s, EV_PRESENT, fid, 1, tmo_us, a1, a2);
        if (a2 != 0) fail($sformatf("present %0d reported as dropped", i));
    endtask
    task automatic wait_frame_start();
        @(posedge clk); while (!(st == 2 && v == 2 && h == 0)) @(posedge clk);
    endtask

    // ---------------- image commands (host side) and the expected bytes of B's DDR ----------------
    int n_berr = 0;                                  // EV_ERRORs board B reports (A STATUS word 41)
    int n_images = 0, n_pixels = 0;                  // images created, pixels written (A STATUS words 62, 63)
    byte unsigned exp_b [longint];                   // expected bytes of B's DDR written by the bench
    function automatic bq ci_body(input [15:0] hnd, input [3:0] usage, input [31:0] base, input [15:0] pitch,
                                  input [7:0] fmt, input [7:0] flags, input [15:0] w, input [15:0] hh,
                                  input [7:0] levels, input [7:0] layers);
        return {le16(hnd), le16({12'h0, usage}), le32(base), le16(pitch), fmt, flags, le16(w), le16(hh), levels, layers,
                le16(16'h0)};
    endfunction
    function automatic int tbpp_of(input [7:0] tf, input int fmt);
        case (tf)
            8'h00: return fbpp(fmt);
            8'h04: return 2;
            8'h0A, 8'h0B: return 3;
            default: return 4;
        endcase
    endfunction
    function automatic int rnd(input int val, input int bits);        // round(v * (2^bits - 1) / 255)
        real f; f = val * ((1 << bits) - 1) / 255.0;
        return $rtoi(f + 0.5);
    endfunction
    function automatic int up(input int c, input int bits);           // round(c * 255 / (2^bits - 1)), ES2 exact
        real f; f = c * 255.0 / ((1 << bits) - 1);
        return $rtoi(f + 0.5);
    endfunction
    // transfer bytes of one pixel -> stored value (RGBA8: 0xRRGGBBAA, RGB565: low 16 bits, P8: low byte)
    function automatic bit [31:0] convert(input [7:0] tf, input int fmt, input bit [31:0] b);
        int r, gg, bl, a; bit [15:0] q; bit s565; s565 = (fmt == 1);
        if (tf == 8'h00) return (fmt == 1) ? {16'h0, b[15:0]} : (fmt == 2) ? {24'h0, b[7:0]} : b;
        case (tf)
            8'h01: begin r = b[7:0];  gg = b[15:8]; bl = b[23:16]; a = b[31:24]; end
            8'h02: begin bl = b[7:0]; gg = b[15:8]; r = b[23:16];  a = b[31:24]; end
            8'h03: begin bl = b[7:0]; gg = b[15:8]; r = b[23:16];  a = 255; end
            8'h0A: begin r = b[7:0];  gg = b[15:8]; bl = b[23:16]; a = 255; end
            8'h0B: begin bl = b[7:0]; gg = b[15:8]; r = b[23:16];  a = 255; end
            default: begin
                q = b[15:0];
                r = up(q[15:11], 5); gg = up(q[10:5], 6); bl = up(q[4:0], 5);
                a = 255;
            end
        endcase
        if (s565) return {16'h0, 16'((rnd(r, 5) << 11) | (rnd(gg, 6) << 5) | rnd(bl, 5))};
        return {8'(r), 8'(gg), 8'(bl), 8'(a)};
    endfunction
    // bench copy of the level / layer layout
    task automatic level_geom(input himg_t m, input int level, input int layer, output int wl, output int hl,
                              output int pl, output longint off);
        longint chain, offl, stride; int bpp, wk, hk, pk;
        chain = 0; offl = 0; bpp = fbpp(m.fmt);
        for (int k = 0; k < m.levels; k++) begin
            wk = (m.w >> k) > 0 ? (m.w >> k) : 1; hk = (m.h >> k) > 0 ? (m.h >> k) : 1;
            pk = (k == 0) ? m.pitch : wk * bpp;
            if (k == level) begin wl = wk; hl = hk; pl = pk; offl = chain; end
            chain += (pk * hk + 3) / 4 * 4;
        end
        stride = (chain + 31) / 32 * 32;
        off = offl + layer * stride;
    endtask
    task automatic create_image(input int idx, input int fmt, input int w, input int hh, input int pitch, input bit org,
                                input longint base, input int levels = 1, input int layers = 1);
        bit [31:0] s; himg_t m;
        send_seq(T_CREATE_IMAGE, ci_body(16'hC000 | 16'(idx), 4'b1101, 32'(base), 16'(pitch),
                                         (fmt == 1) ? 8'h04 : (fmt == 2) ? 8'h0C : 8'h00,
                                         {7'h0, org}, 16'(w), 16'(hh), 8'(levels), 8'(layers)), s);
        n_fwd++; n_images++;
        m.w = w; m.h = hh; m.pitch = pitch; m.levels = levels; m.layers = layers; m.fmt = fmt; m.org = org; m.base = base;
        himg[idx] = m;
    endtask
    // a CREATE_IMAGE that board B must reject with `code`
    task automatic create_image_bad(input bq body, input [31:0] code);
        bit [31:0] s, a1, a2;
        send_seq(T_CREATE_IMAGE, body, s); n_fwd++; n_bev++; n_berr++;
        expect_event(s, EV_ERROR, code, 1, 3000, a1, a2);
    endtask
    // WRITE_IMAGE of pix (transfer pixel bytes, byte 0 in [7:0]) into rect (x, y, w, hh) from first0, per_dg pixels
    // per datagram; updates the expected DDR bytes and the display model
    task automatic write_image(input int idx, input int level, input int layer, input [7:0] tf, input int x, input int y,
                               input int w, input int hh, input bit [31:0] pix [$], input int first0, input int per_dg);
        himg_t m; int tb, k, n, wl, hl, pl, sb, p, row, col; bit [31:0] s, sv; bq body; longint off, a;
        m = himg[idx]; tb = tbpp_of(tf, m.fmt); sb = fbpp(m.fmt);
        level_geom(m, level, layer, wl, hl, pl, off);
        k = 0;
        while (k < pix.size()) begin
            n = (pix.size() - k < per_dg) ? pix.size() - k : per_dg;
            body = {le16(16'hC000 | 16'(idx)), 8'(level), 8'(layer), tf, 8'h00, le16(16'h0), le16(16'(x)), le16(16'(y)),
                    le16(16'(w)), le16(16'(hh)), le32(32'(first0 + k)), le32(32'h0)};
            for (int i = 0; i < n; i++) for (int q = 0; q < tb; q++) body.push_back(pix[k + i][q*8 +: 8]);
            send_seq(T_WRITE_IMAGE, body, s); n_fwd++; n_pixels += n;
            for (int i = 0; i < n; i++) begin
                p = first0 + k + i; row = y + p / w; col = x + p % w;
                sv = convert(tf, m.fmt, pix[k + i]);
                a = m.base + off + row * pl + col * sb;
                for (int q = 0; q < sb; q++) exp_b[a + q] = sv[q*8 +: 8];
                if (level == 0 && layer == 0) hpix[idx][row * m.w + col] = sv;
            end
            k += n;
        end
    endtask
    // a WRITE_IMAGE header + payload that board B must reject with `code`
    task automatic write_image_bad(input bq body, input [31:0] code);
        bit [31:0] s, a1, a2;
        send_seq(T_WRITE_IMAGE, body, s); n_fwd++; n_bev++; n_berr++;
        expect_event(s, EV_ERROR, code, 1, 3000, a1, a2);
    endtask
    task automatic fence_b(input [31:0] id);
        bit [31:0] s, a1, a2;
        send_seq(T_FENCE, le32(id), s); n_fwd++; n_bev++;
        expect_event(s, EV_FENCE, id, 1, 20000, a1, a2);
    endtask
    // compare B's DDR with the expected bytes in [a0, a0 + len)
    task automatic check_b_ddr(input longint a0, input longint len, input string what);
        int bad = 0;
        for (longint a = a0; a < a0 + len; a++)
            if (exp_b.exists(a) && u_migB.peek_byte(a) !== exp_b[a]) begin
                if (bad == 0) fail($sformatf("%s: B DDR byte %0h = %02h expected %02h", what, a, u_migB.peek_byte(a), exp_b[a]));
                bad++;
            end
        if (bad) fail($sformatf("%s: %0d bytes differ", what, bad));
    endtask
    // MEM_READ board 1 of [a0, a0 + len) and compare the READ_DATA bytes with B's DDR
    task automatic mem_read_b(input longint a0, input int len, input bit [31:0] tag, input string what);
        bit [31:0] s; int t, bad;
        rd_bytes.delete(tag);
        send_seq(T_MEM_READ, {8'd1, 8'd0, le16(16'h0), le32(32'(a0)), le32(32'(len)), le32(tag)}, s); n_fwd++;
        t = 0;
        while ((!rd_bytes.exists(tag) || rd_bytes[tag].size() < len) && t < 2_000_000) begin @(posedge clkA); t++; end
        if (!rd_bytes.exists(tag) || rd_bytes[tag].size() != len) begin
            fail($sformatf("%s: READ_DATA %0d of %0d bytes", what, rd_bytes.exists(tag) ? rd_bytes[tag].size() : 0, len));
            return;
        end
        bad = 0;
        for (int i = 0; i < len; i++) if (rd_bytes[tag][i] !== u_migB.peek_byte(a0 + i)) bad++;
        if (bad) fail($sformatf("%s: %0d READ_DATA bytes differ from B's DDR", what, bad));
    endtask
    function automatic bit [31:0] prng(input int i, input int salt);
        bit [31:0] x; x = 32'(i) * 32'h9E37_79B9 ^ 32'(salt) * 32'h85EB_CA6B; x ^= x >> 15; x *= 32'h2C1B_3C6D; x ^= x >> 13;
        return x;
    endfunction

    // ---------------- READ / FILL / COPY_IMAGE, palette, cursor: bench models ----------------
    int n_xops = 0, n_pal = 0, n_curcmd = 0, n_fastpos = 0;           // for the layout-5 STATUS check
    function automatic bit [31:0] ddr_px(input longint a, input int fmt);        // stored value in B's DDR now
        bit [31:0] v; v = 0;
        for (int q = 0; q < fbpp(fmt); q++) v[q*8 +: 8] = u_migB.peek_byte(a + q);
        return v;
    endfunction
    function automatic longint px_addr(input int idx, input int row, input int col);       // level 0, layer 0
        return himg[idx].base + longint'(row) * himg[idx].pitch + col * fbpp(himg[idx].fmt);
    endfunction
    // the value a pixel will have once every command sent so far has run: the bench's expected bytes where it has
    // them (commands may still be queued), else B's DDR
    function automatic bit [31:0] cur_px(input int idx, input int row, input int col);
        bit [31:0] v; longint a; v = 0; a = px_addr(idx, row, col);
        for (int q = 0; q < fbpp(himg[idx].fmt); q++) v[q*8 +: 8] = exp_b.exists(a + q) ? exp_b[a + q] : u_migB.peek_byte(a + q);
        return v;
    endfunction
    task automatic set_px(input int idx, input int row, input int col, input bit [31:0] v);
        longint a; a = px_addr(idx, row, col);
        for (int q = 0; q < fbpp(himg[idx].fmt); q++) exp_b[a + q] = v[q*8 +: 8];
        hpix[idx][row * himg[idx].w + col] = v;
    endtask
    task automatic dec(input int fmt, input bit [31:0] v, output int r, output int gg, output int bl, output int a);
        if (fmt == 1) begin r = up(v[15:11], 5); gg = up(v[10:5], 6); bl = up(v[4:0], 5); a = 255; end
        else begin r = v[31:24]; gg = v[23:16]; bl = v[15:8]; a = v[7:0]; end
    endtask
    function automatic bit [31:0] enc(input int fmt, input int r, input int gg, input int bl, input int a);
        if (fmt == 1) return {16'h0, 5'(rnd(r, 5)), 6'(rnd(gg, 6)), 5'(rnd(bl, 5))};
        return {8'(r), 8'(gg), 8'(bl), 8'(a)};
    endfunction
    // READ_IMAGE of a level-0 rectangle; the READ_DATA bytes (offset = from pixel 0 of the rectangle) against B's DDR
    task automatic read_image_chk(input int idx, input [7:0] tf, input int x, input int y, input int w, input int hh,
                                  input int first, input int count, input [31:0] tag, input string what);
        himg_t m; int tb, n, bad, t; bit [31:0] s; byte unsigned ex [$];
        m = himg[idx]; tb = tbpp_of(tf, m.fmt); n = (count == 0) ? w * hh - first : count;
        for (int p = first; p < first + n; p++) begin
            int r, gg, bl, a; bit [31:0] sv, tv;
            sv = cur_px(idx, y + p / w, x + p % w); dec(m.fmt, sv, r, gg, bl, a);
            case (tf)
                8'h00: tv = sv;
                8'h01: tv = {8'(a), 8'(bl), 8'(gg), 8'(r)};
                8'h02: tv = {8'(a), 8'(r), 8'(gg), 8'(bl)};
                8'h03: tv = {8'hFF, 8'(r), 8'(gg), 8'(bl)};
                8'h0A: tv = {8'h0, 8'(bl), 8'(gg), 8'(r)};
                8'h0B: tv = {8'h0, 8'(r), 8'(gg), 8'(bl)};
                default: tv = enc(1, r, gg, bl, a);
            endcase
            for (int q = 0; q < tb; q++) ex.push_back(tv[q*8 +: 8]);
        end
        rd_bytes.delete(tag);
        send_seq(T_READ_IMAGE, {le16(16'hC000 | 16'(idx)), 8'd0, 8'd0, tf, 8'h00, le16(16'h0), le16(16'(x)), le16(16'(y)),
                                le16(16'(w)), le16(16'(hh)), le32(32'(first)), le32(32'(count)), le32(tag)}, s);
        n_fwd++; n_xops++;
        t = 0;
        while ((!rd_bytes.exists(tag) || rd_bytes[tag].size() < ex.size()) && t < 500_000) begin @(posedge clkA); t++; end
        if (!rd_bytes.exists(tag) || rd_bytes[tag].size() != ex.size()) begin
            fail($sformatf("%s: READ_DATA %0d of %0d bytes", what, rd_bytes.exists(tag) ? rd_bytes[tag].size() : 0, ex.size()));
            return;
        end
        bad = 0;
        foreach (ex[i]) if (!rd_bytes[tag].exists(first * tb + i) || rd_bytes[tag][first * tb + i] !== ex[i]) begin
            if (bad == 0) fail($sformatf("%s: byte %0d = %02h expected %02h", what, i,
                                         rd_bytes[tag].exists(first * tb + i) ? rd_bytes[tag][first * tb + i] : 0, ex[i]));
            bad++;
        end
    endtask
    task automatic tdec(input [7:0] tf, input bit [31:0] v, output int r, output int gg, output int bl, output int a);
        case (tf)
            8'h01: begin r = v[7:0]; gg = v[15:8]; bl = v[23:16]; a = v[31:24]; end
            8'h02: begin bl = v[7:0]; gg = v[15:8]; r = v[23:16]; a = v[31:24]; end
            8'h03: begin bl = v[7:0]; gg = v[15:8]; r = v[23:16]; a = 255; end
            8'h0A: begin r = v[7:0]; gg = v[15:8]; bl = v[23:16]; a = 255; end
            8'h0B: begin bl = v[7:0]; gg = v[15:8]; r = v[23:16]; a = 255; end
            default: begin r = up(v[15:11], 5); gg = up(v[10:5], 6); bl = up(v[4:0], 5); a = 255; end
        endcase
    endtask
    // FILL_IMAGE (level 0); the model reads the current pixels from B's DDR, so the caller fences first
    task automatic fill_image(input int idx, input [7:0] tf, input [3:0] mask, input int x, input int y, input int w,
                              input int hh, input [31:0] value);
        himg_t m; bit [31:0] s, fv; int r, gg, bl, a;
        m = himg[idx];
        if (tf == 8'h00) fv = value; else begin tdec(tf, value, r, gg, bl, a); fv = enc(m.fmt, r, gg, bl, a); end
        for (int j = 0; j < hh; j++) for (int i = 0; i < w; i++) begin
            bit [31:0] ov, nv;
            ov = cur_px(idx, y + j, x + i); nv = ov;
            if (m.fmt == 0) begin
                if (mask[0]) nv[31:24] = fv[31:24]; if (mask[1]) nv[23:16] = fv[23:16];
                if (mask[2]) nv[15:8] = fv[15:8];   if (mask[3]) nv[7:0] = fv[7:0];
            end else if (m.fmt == 1) begin
                if (mask[0]) nv[15:11] = fv[15:11]; if (mask[1]) nv[10:5] = fv[10:5]; if (mask[2]) nv[4:0] = fv[4:0];
            end else if (mask != 0) nv[7:0] = fv[7:0];
            set_px(idx, y + j, x + i, nv);
        end
        send_seq(T_FILL_IMAGE, {le16(16'hC000 | 16'(idx)), 8'd0, 8'd0, tf, 8'(mask), le16(16'h0), le16(16'(x)), le16(16'(y)),
                                le16(16'(w)), le16(16'(hh)), le32(value)}, s);
        n_fwd++; n_xops++;
    endtask
    // COPY_IMAGE (level 0 -> level 0); the source is snapshotted from B's DDR (the caller fences first), so an
    // overlapping 1:1 copy must equal "read the whole source, then write"
    task automatic copy_image(input int si, input int di, input int sx, input int sy, input int sw, input int sh,
                              input int dx, input int dy, input int dw, input int dh, input [2:0] flags,
                              input [31:0] klo = 0, input [31:0] khi = 0);
        bit [31:0] s; bit [31:0] snap [int]; int sf, df;
        sf = himg[si].fmt; df = himg[di].fmt;
        for (int v = 0; v < sh; v++) for (int u = 0; u < sw; u++) snap[v * sw + u] = cur_px(si, sy + v, sx + u);
        for (int j = 0; j < dh; j++) for (int i = 0; i < dw; i++) begin
            int u, v, r, gg, bl, a; bit [31:0] sv; bit tr;
            u = ((2 * i + 1) * sw) / (2 * dw); v = ((2 * j + 1) * sh) / (2 * dh);
            if (flags[0]) u = sw - 1 - u;
            if (flags[1]) v = sh - 1 - v;
            sv = snap[v * sw + u]; tr = 0;
            if (flags[2]) begin
                if (sf == 0) tr = sv[31:24] >= klo[31:24] && sv[31:24] <= khi[31:24] && sv[23:16] >= klo[23:16] &&
                                  sv[23:16] <= khi[23:16] && sv[15:8] >= klo[15:8] && sv[15:8] <= khi[15:8];
                else if (sf == 1) tr = sv[15:11] >= klo[15:11] && sv[15:11] <= khi[15:11] && sv[10:5] >= klo[10:5] &&
                                       sv[10:5] <= khi[10:5] && sv[4:0] >= klo[4:0] && sv[4:0] <= khi[4:0];
                else tr = sv[7:0] >= klo[7:0] && sv[7:0] <= khi[7:0];
            end
            if (!tr) begin
                if (sf == 2) set_px(di, dy + j, dx + i, sv);
                else begin dec(sf, sv, r, gg, bl, a); set_px(di, dy + j, dx + i, enc(df, r, gg, bl, a)); end
            end
        end
        send_seq(T_COPY_IMAGE, {le16(16'hC000 | 16'(si)), 8'd0, 8'd0, le16(16'hC000 | 16'(di)), 8'd0, 8'd0,
                                le16(16'(sx)), le16(16'(sy)), le16(16'(sw)), le16(16'(sh)), le16(16'(dx)), le16(16'(dy)),
                                le16(16'(dw)), le16(16'(dh)), le32({29'h0, flags}), le32(klo), le32(khi)}, s);
        n_fwd++; n_xops++;
    endtask
    // a command board B must reject with `code`
    task automatic expect_reject(input [7:0] t, input bq body, input [31:0] code);
        bit [31:0] s, a1, a2;
        send_seq(t, body, s); n_fwd++; n_bev++; n_berr++;
        expect_event(s, EV_ERROR, code, 1, 3000, a1, a2);
    endtask
    // palette
    bit [23:0] pal_m [0:255];
    task automatic set_palette(input int first, input int count, input int salt);
        bq body; bit [31:0] s;
        body = {le16(16'(first)), le16(16'(count))};
        for (int k = 0; k < count; k++) begin
            pal_m[first + k] = prng(k, salt) & 32'h00FF_FFFF;
            body = {body, le32({8'h0, pal_m[first + k]})};
        end
        send_seq(T_SET_PALETTE, body, s); n_fwd++; n_pal++;
    endtask
    task automatic check_palette(input string what);        // the bank on screen holds the bench's palette
        int b, bad; b = dut.u_disp.u_ctl.pal_cur; bad = 0;
        if (dut.u_disp.u_ctl.pal_req !== dut.u_disp.u_ctl.pal_cur) fail($sformatf("%s: palette switch still pending", what));
        for (int i = 0; i < 256; i++) if (dut.u_disp.u_dout.u_post.pal[{b[0], 8'(i)}] !== pal_m[i]) bad++;
        if (bad) fail($sformatf("%s: %0d palette entries of the shown bank differ", what, bad));
    endtask
    // the display palette bank a frame is using is never written
    always @(posedge clkB) if (dut.u_disp.pal_we && dut.u_disp.pal_wa[8] == dut.u_disp.u_scan.f_pal)
        fail($sformatf("palette entry %0d written in the bank on screen", dut.u_disp.pal_wa[7:0]));
    // cursor
    bit [31:0] cur_shape [0:4095];
    int        cur_bank_m = 0;                              // the shape bank SHOW-take will select (model)
    task automatic cursor_shape(input int mode, input int salt);
        bit [31:0] s;
        for (int i = 0; i < 4096; i++) begin
            bit [31:0] v; v = prng(i, salt);
            if (mode != 1) case (v[1:0]) 0: v[31:24] = 0; 1: v[31:24] = 255; default: ; endcase
            if (mode == 2) for (int ch = 0; ch < 3; ch++) if (v[ch*8 +: 8] > v[31:24]) v[ch*8 +: 8] = v[31:24];
            cur_shape[i] = v;
        end
        for (int k = 0; k < 4096; k += 340) begin                 // 12 datagrams
            bq body; int n;
            n = (4096 - k < 340) ? 4096 - k : 340;
            body = {8'd1, 8'd0, le16(16'h0), le16(16'(k)), le16(16'h0)};
            for (int i = 0; i < n; i++) body = {body, le32(cur_shape[k + i])};
            send_seq(T_CURSOR, body, s); n_fwd++;
        end
    endtask
    task automatic cursor_show(input int mode, input bit take, input int hx, input int hy, input int x, input int y);
        bit [31:0] s;
        send_seq(T_CURSOR, {8'd2, 8'(mode), {6'h0, 1'b1, take}, 8'd0, 8'(hx), 8'(hy), le16(16'h0), le16(16'(x)), le16(16'(y))}, s);
        n_fwd++; n_curcmd++;
        if (take) cur_bank_m ^= 1;
    endtask
    task automatic cursor_check(input bit en, input int mode, input int ox, input int oy, input bit shape, input string what);
        int bad;
        if (dut.u_disp.u_dout.u_post.c_en !== en) fail($sformatf("%s: cursor on %b, expected %b", what, dut.u_disp.u_dout.u_post.c_en, en));
        else if (en) begin
            if (dut.u_disp.u_dout.u_post.c_mode != mode || $signed(dut.u_disp.u_dout.u_post.c_ox) != ox ||
                $signed(dut.u_disp.u_dout.u_post.c_oy) != oy || dut.u_disp.u_dout.u_post.c_bank != cur_bank_m)
                fail($sformatf("%s: cursor mode %0d at %0d,%0d bank %0d, expected mode %0d at %0d,%0d bank %0d", what,
                               dut.u_disp.u_dout.u_post.c_mode, $signed(dut.u_disp.u_dout.u_post.c_ox),
                               $signed(dut.u_disp.u_dout.u_post.c_oy), dut.u_disp.u_dout.u_post.c_bank, mode, ox, oy, cur_bank_m));
            if (shape) begin
                bad = 0;
                for (int i = 0; i < 4096; i++) if (dut.u_disp.u_dout.u_post.cram[{cur_bank_m[0], 12'(i)}] !== cur_shape[i]) bad++;
                if (bad) fail($sformatf("%s: %0d cursor shape words differ", what, bad));
            end
        end
    endtask
    task automatic cursor_fast(input int x, input int y);              // unsequenced CURSOR_POS (fast path)
        send_raw(msg(T_CURSOR_POS, 0, {le16(16'(x)), le16(16'(y))})); n_fastpos++;
    endtask

    // +DBG: trace board B's command processor and READ / FILL / COPY engine
    initial if ($test$plusargs("DBG") || $test$plusargs("QUICK2D")) forever begin
        @(dut.u_cp.st or dut.u_xf.run or dut.u_xf.qs);
        $display("%t DBG cp %0d (mtype %02h) xf run %b qs %0d rd_act %b wr_act %b half_full %b rj_v %b wj_v %b of_cnt %0d r_req %b r_off %0d r_len %0d",
                 $time, dut.u_cp.st, dut.u_cp.mtype, dut.u_xf.run, dut.u_xf.qs, dut.u_xf.rd_act, dut.u_xf.wr_act,
                 dut.u_xf.half_full, dut.u_xf.rj_v, dut.u_xf.wj_v, dut.u_xf.of_cnt, dut.u_cp.r_req, dut.u_cp.r_off, dut.u_cp.r_len);
    end
    initial begin
        bit [31:0] s, s2, a1, a2; int t; realtime t0; bit quick;
        full = $test$plusargs("FULL");
        quick = $test$plusargs("QUICK2D");               // only the link-up and the READ / FILL / COPY_IMAGE section (debugging)
        repeat (20) @(posedge clkA); rstA <= 0; rstB <= 0;
        repeat (200) @(posedge clkA); calA <= 1; calB <= 1;
        wait (da.u_eth.mdio_init_done);
        physr <= 16'h002C;
        wait (da.u_eth.link_up); repeat (2000) @(posedge clkA);
        send_raw(msg(T_HELLO, 0, le32(1)));
        @(ack_ev); if (ack_exp != 1) fail($sformatf("HELLO ACK exp %0d", ack_exp));
        // ---- board link up, B alive ----
        t = 0;
        while (!da.link_ok && t < 2_000_000) begin @(posedge clkA); t++; end
        if (!da.link_ok) begin fail("board link not up (A trained / far_up / B alive)"); end
        else $display("ok: board link up %.1f us after reset (A eye %0d, B eye %0d), B alive", t_link_up / 1000.0,
                      da.l_eye, dut.l_eye);
        // ---- B boot: black, fill, image 8 in the 64x48 test mode ----
        wait_img_frames(8, 2, 80);
        $display("ok: board B boot picture (image 8, mode %0d)", dut.u_disp.cur_mode);
        if (quick) begin
            bit [31:0] pix [$];
            create_image(20, 0, 64, 48, 288, 1'b0, 64'h2100_0000);
            for (int i = 0; i < 64 * 48; i++) pix.push_back(prng(i, 2));
            write_image(20, 0, 0, 8'h03, 0, 0, 64, 48, pix, 0, 333);
            create_image(21, 1, 64, 48, 134, 1'b1, 64'h2200_0000);
            pix.delete(); for (int i = 0; i < 64 * 48; i++) pix.push_back(prng(i, 5));
            write_image(21, 0, 0, 8'h04, 0, 0, 64, 48, pix, 0, 720);
            fence_b(32'd99);
            $display("ok: QUICK2D: images 20 / 21 created");
        end else begin
        // ---- mode switch over Ethernet ----
        set_mode_eth(6); wait_img_frames(8, 2);
        $display("ok: SET_MODE 6 (80x40, fractional MMCM) over Ethernet, frames checked");
        set_mode_eth(5); wait_img_frames(8, 2);
        $display("ok: SET_MODE 5 (64x48) over Ethernet, frames checked");
        send_seq(T_SET_MODE, le32(7), s); n_fwd++; n_bev++;
        expect_event(s, EV_ERROR, ERR_RANGE, 1, 3000, a1, a2);
        $display("ok: SET_MODE 7 rejected (EV_ERROR ERR_RANGE)");
        // ---- presents over Ethernet ----
        present_eth(3, 16'h0, 32'd1001); wait_img_frames(3, 2);
        present_eth(1, 16'h1, 32'd1002); wait_img_frames(1, 1);                      // at once
        $display("ok: PRESENT at vsync and at once, EV_PRESENT with frame ids");
        send_seq(T_PRESENT, present_body(16'hC002, 16'h2, 32'd1003), s); n_fwd++; n_bev++;   // hold
        send_seq(T_FENCE, le32(32'd9), s2); n_fwd++; n_bev++;
        expect_event(s, EV_PRESENT, 32'd1003, 1, 3000, a1, a2);
        expect_event(s2, EV_FENCE, 32'd9, 1, 3000, a1, a2);
        wait_img_frames(2, 1);
        $display("ok: PRESENT with hold, then FENCE: events in order");
        wait_frame_start();                                                          // both arrive before vsync
        send_seq(T_PRESENT, present_body(16'hC004, 16'h0, 32'd1004), s); n_fwd++; n_bev++;
        send_seq(T_PRESENT, present_body(16'hC005, 16'h0, 32'd1005), s2); n_fwd++; n_bev++;
        expect_event(s, EV_PRESENT, 32'd1004, 1, 3000, a1, a2);
        if (a2 != 1) fail("replaced present not reported as dropped");
        expect_event(s2, EV_PRESENT, 32'd1005, 1, 3000, a1, a2);
        wait_img_frames(5, 1);
        $display("ok: two presents in one frame: the first reported dropped, the second shown");
        send_seq(T_PRESENT, present_body(16'h8001, 16'h0, 32'd0), s); n_fwd++; n_bev++;
        expect_event(s, EV_ERROR, ERR_HANDLE, 1, 3000, a1, a2);
        send_seq(T_PRESENT, present_body(16'hC0FF, 16'h0, 32'd0), s); n_fwd++; n_bev++;
        expect_event(s, EV_ERROR, ERR_HANDLE, 1, 3000, a1, a2);
        $display("ok: bad handles rejected (EV_ERROR ERR_HANDLE)");
        // ---- FENCE after an A-local MEMTEST ----
        send_seq(T_MEMTEST, {8'd0, 8'd0, le16(0), le32(32'h0010_0000), le32(32'h0000_4000), le32(32'h1234), le32(3)}, s);
        send_seq(T_FENCE, le32(32'd10), s2); n_fwd++; n_bev++;
        expect_event(s, EV_MEMTEST, 0, 1, 5000, a1, a2);
        expect_event(s2, EV_FENCE, 32'd10, 1, 3000, a1, a2);
        $display("ok: A MEMTEST, then FENCE through B");
        // ---- board B memory over Ethernet: MEM_WRITE / MEM_READ (READ_DATA over the link) / MEMTEST, board = 1 ----
        begin
            bq data; int bad;
            for (int i = 0; i < 1000; i++) data.push_back(8'(prng(i, 1)));
            send_seq(T_MEM_WRITE, {8'd1, 8'd0, le16(16'h0), le32(32'h2000_0003), data}, s); n_fwd++;
            fence_b(32'd11);
            bad = 0;
            for (int i = 0; i < 1000; i++) if (u_migB.peek_byte(64'h2000_0003 + i) !== data[i]) bad++;
            if (bad) fail($sformatf("B MEM_WRITE: %0d bytes wrong in B's DDR", bad));
            mem_read_b(64'h2000_0001, 3001, 32'h77, "B MEM_READ 3001 bytes");     // 3 READ_DATA chunks
            send_seq(T_MEMTEST, {8'd1, 8'd0, le16(16'h0), le32(32'h2400_0000), le32(32'h0000_2000), le32(32'h5555), le32(3)}, s);
            n_fwd++; n_bev++;
            expect_event(s, EV_MEMTEST, 0, 1, 5000, a1, a2);
            send_seq(T_MEMTEST, {8'd1, 8'd0, le16(16'h0), le32(32'h2400_0004), le32(32'h0000_0020), le32(32'h5555), le32(3)}, s);
            n_fwd++; n_bev++; n_berr++;
            expect_event(s, EV_ERROR, ERR_LEN, 1, 3000, a1, a2);
            $display("ok: B MEM_WRITE 1000 bytes (unaligned), MEM_READ 3001 bytes in %0d READ_DATA datagrams, MEMTEST 8 KB, misaligned MEMTEST rejected",
                     rd_dgrams);
        end
        // ---- payload scrambler: a zero payload must not reach the forward wires as zeros ----
        begin
            bq z; int zero_beats, beats; bit mon;
            for (int i = 0; i < 1440; i++) z.push_back(8'h00);
            zero_beats = 0; beats = 0; mon = 1;
            fork
                begin
                    send_seq(T_MEM_WRITE, {8'd1, 8'd0, le16(16'h0), le32(32'h2000_1000), z}, s); n_fwd++;
                    fence_b(32'd17); mon = 0;
                end
                while (mon) begin
                    @(posedge da.lclk);
                    if (da.u_link.g_a.u_framer.valid_lo && da.u_link.g_a.u_framer.valid_hi) begin
                        beats++;
                        if (da.u_link.g_a.u_framer.lo == 0 && da.u_link.g_a.u_framer.hi == 0) zero_beats++;
                    end
                end
            join
            for (int i = 0; i < 1440; i++) if (u_migB.peek_byte(64'h2000_1000 + i) !== 8'h00) begin fail("zero MEM_WRITE not zero in B's DDR"); break; end
            if (zero_beats > 2) fail($sformatf("scrambler: %0d all-zero beats on the forward wires for a zero payload", zero_beats));
            $display("ok: scrambler: 1440 zero bytes crossed the forward lane with %0d all-zero beats of %0d, arrived as zeros", zero_beats, beats);
        end
        // ---- CREATE_IMAGE: rejected descriptors ----
        send_seq(T_CREATE_IMAGE, ci_body(16'h4014, 4'b0100, 32'h2100_0000, 16'd256, 8'h00, 8'h00, 16'd64, 16'd48, 8'd1, 8'd1), s);
        expect_event(s, EV_ERROR, ERR_HANDLE, 1, 3000, a1, a2);                      // no B bit: rejected by A
        // format 0x3F: unassigned (0x05 is RGBA4444, a valid format)
        create_image_bad(ci_body(16'hC014, 4'b0100, 32'h2100_0000, 16'd256, 8'h3F, 8'h00, 16'd64, 16'd48, 8'd1, 8'd1), ERR_RANGE);
        create_image_bad(ci_body(16'hC014, 4'b0100, 32'h2100_0000, 16'd256, 8'h00, 8'h00, 16'd0, 16'd48, 8'd1, 8'd1), ERR_RANGE);
        create_image_bad(ci_body(16'hC014, 4'b0100, 32'h2100_0000, 16'd252, 8'h00, 8'h00, 16'd64, 16'd48, 8'd1, 8'd1), ERR_RANGE);
        create_image_bad(ci_body(16'hC014, 4'b0100, 32'h2100_0000, 16'd130, 8'h04, 8'h00, 16'd64, 16'd48, 8'd2, 8'd1), ERR_RANGE);
        create_image_bad(ci_body(16'hC014, 4'b0100, 32'h2100_0000, 16'd256, 8'h00, 8'h00, 16'd64, 16'd48, 8'd8, 8'd1), ERR_RANGE);
        create_image_bad(ci_body(16'hC014, 4'b0100, 32'h2100_0000, 16'd256, 8'h00, 8'h02, 16'd64, 16'd48, 8'd1, 8'd1), ERR_RANGE);
        create_image_bad(ci_body(16'hC014, 4'b0100, 32'h0800_0000, 16'd256, 8'h00, 8'h00, 16'd64, 16'd48, 8'd1, 8'd1), ERR_HEAP);
        create_image_bad(ci_body(16'hC014, 4'b0100, 32'h2100_0010, 16'd256, 8'h00, 8'h00, 16'd64, 16'd48, 8'd1, 8'd1), ERR_HEAP);
        create_image_bad(ci_body(16'hC014, 4'b0100, 32'h3FFF_F000, 16'd256, 8'h00, 8'h00, 16'd64, 16'd48, 8'd1, 8'd1), ERR_HEAP);
        create_image_bad(ci_body(16'hC014, 4'b0100, 32'h3FFF_0000, 16'd256, 8'h00, 8'h00, 16'd64, 16'd48, 8'd1, 8'd255), ERR_HEAP);
        $display("ok: CREATE_IMAGE rejects: no B bit, format, width 0, pitch, mip pitch, levels, layout, heap base / alignment / end");
        // ---- CREATE_IMAGE + WRITE_IMAGE + PRESENT: RGBA8 top-left image, pitch with spare bytes ----
        begin
            bit [31:0] pix [$];
            create_image(20, 1'b0, 64, 48, 288, 1'b0, 64'h2100_0000);
            for (int i = 0; i < 64 * 48; i++) pix.push_back(prng(i, 2));
            write_image(20, 0, 0, 8'h03, 0, 0, 64, 48, pix, 0, 333);                // BGRX, datagrams end mid-row
            pix.delete();
            for (int i = 0; i < 20 * 9; i++) pix.push_back(prng(i, 3));
            write_image(20, 0, 0, 8'h0A, 5, 7, 20, 9, pix, 0, 47);                 // RGB888 sub-rectangle
            pix.delete();
            for (int i = 0; i < 11; i++) pix.push_back(prng(i, 4));
            write_image(20, 0, 0, 8'h02, 40, 30, 13, 3, pix, 17, 11);              // BGRA, starts at pixel 17 of the rect
            fence_b(32'd12);
            check_b_ddr(64'h2100_0000, 288 * 48, "image 20");
            present_eth(20, 16'h0, 32'd3001); wait_img_frames(20, 2);
            $display("ok: image 20 (RGBA8 64x48, pitch 288): BGRX in %0d-pixel datagrams, RGB888 / BGRA rectangles, shown",
                     333);
        end
        // ---- RGB565 bottom-left image: RGB565, RGBA8888 (rounded) and RAW data ----
        begin
            bit [31:0] pix [$];
            create_image(21, 1'b1, 64, 48, 134, 1'b1, 64'h2200_0000);
            for (int i = 0; i < 64 * 48; i++) pix.push_back(prng(i, 5));
            write_image(21, 0, 0, 8'h04, 0, 0, 64, 48, pix, 0, 720);
            pix.delete();
            for (int i = 0; i < 33 * 40; i++) pix.push_back(prng(i, 6));
            write_image(21, 0, 0, 8'h01, 30, 2, 33, 40, pix, 0, 360);
            pix.delete();
            for (int i = 0; i < 64 * 8; i++) pix.push_back(prng(i, 7));
            write_image(21, 0, 0, 8'h00, 0, 40, 64, 8, pix, 0, 700);
            fence_b(32'd13);
            check_b_ddr(64'h2200_0000, 134 * 48, "image 21");
            present_eth(21, 16'h1, 32'd3002); wait_img_frames(21, 2);
            $display("ok: image 21 (RGB565 64x48 bottom-left, pitch 134): RGB565, RGBA8888 -> 565 rounding, RAW, shown");
        end
        // ---- mip levels and layers: RGBA8 37x29, 3 levels, 2 layers ----
        begin
            bit [31:0] pix [$]; himg_t m; int wl, hl, pl; longint off;
            create_image(22, 1'b0, 37, 29, 148, 1'b0, 64'h2300_0000, 3, 2);
            for (int i = 0; i < 37 * 29; i++) pix.push_back(prng(i, 8));
            write_image(22, 0, 0, 8'h01, 0, 0, 37, 29, pix, 0, 360);
            pix.delete();
            for (int i = 0; i < 18 * 14; i++) pix.push_back(prng(i, 9));
            write_image(22, 1, 1, 8'h04, 0, 0, 18, 14, pix, 0, 252);
            pix.delete();
            for (int i = 0; i < 7 * 5; i++) pix.push_back(prng(i, 10));
            write_image(22, 2, 1, 8'h02, 1, 1, 7, 5, pix, 0, 35);
            fence_b(32'd14);
            m = himg[22]; level_geom(m, 0, 2, wl, hl, pl, off);                    // layer 2 offset = 2 x layer size
            check_b_ddr(64'h2300_0000, off, "image 22 (levels, layers)");
            mem_read_b(64'h2300_0000, 32'(off), 32'h78, "image 22 MEM_READ");
            $display("ok: image 22 (37x29, 3 levels, 2 layers): level 0 / layer 0, level 1 / layer 1, level 2 / layer 1 at %0d bytes per layer",
                     off / 2);
        end
        // ---- WRITE_IMAGE: rejected commands ----
        write_image_bad({le16(16'hC01E), 8'd0, 8'd0, 8'h01, 8'h00, le16(16'h0), le16(16'd0), le16(16'd0), le16(16'd1), le16(16'd1),
                         le32(32'h0), le32(32'h0), le32(32'h0)}, ERR_HANDLE);                       // index 30: not created
        write_image_bad({le16(16'hC014), 8'd0, 8'd0, 8'h01, 8'h00, le16(16'h0), le16(16'd60), le16(16'd0), le16(16'd5), le16(16'd1),
                         le32(32'h0), le32(32'h0), le32(32'h0)}, ERR_RANGE);                        // x + w > 64
        write_image_bad({le16(16'hC016), 8'd3, 8'd0, 8'h01, 8'h00, le16(16'h0), le16(16'd0), le16(16'd0), le16(16'd1), le16(16'd1),
                         le32(32'h0), le32(32'h0), le32(32'h0)}, ERR_RANGE);                        // level 3 of 3
        write_image_bad({le16(16'hC016), 8'd0, 8'd2, 8'h01, 8'h00, le16(16'h0), le16(16'd0), le16(16'd0), le16(16'd1), le16(16'd1),
                         le32(32'h0), le32(32'h0), le32(32'h0)}, ERR_RANGE);                        // layer 2 of 2
        write_image_bad({le16(16'hC014), 8'd0, 8'd0, 8'h05, 8'h00, le16(16'h0), le16(16'd0), le16(16'd0), le16(16'd1), le16(16'd1),
                         le32(32'h0), le32(32'h0), le32(32'h0)}, ERR_RANGE);                        // transfer format 0x05
        write_image_bad({le16(16'hC014), 8'd0, 8'd0, 8'h0A, 8'h00, le16(16'h0), le16(16'd0), le16(16'd0), le16(16'd4), le16(16'd1),
                         le32(32'h0), le32(32'h0), le32(32'h0)}, ERR_LEN);                          // 4 bytes of RGB888
        write_image_bad({le16(16'hC014), 8'd0, 8'd0, 8'h01, 8'h00, le16(16'h0), le16(16'd0), le16(16'd0), le16(16'd2), le16(16'd1),
                         le32(32'h1), le32(32'h0), le32(32'h0), le32(32'h0)}, ERR_RANGE);           // first 1 + 2 pixels > 2
        // DESTROY: the image can no longer be presented
        send_seq(T_DESTROY, {le16(16'hC015), le16(16'h0)}, s); n_fwd++;
        send_seq(T_PRESENT, present_body(16'hC015, 16'h0, 32'd0), s); n_fwd++; n_bev++; n_berr++;
        expect_event(s, EV_ERROR, ERR_HANDLE, 1, 3000, a1, a2);
        present_eth(20, 16'h0, 32'd3003); wait_img_frames(20, 1);
        $display("ok: WRITE_IMAGE rejects (handle, rectangle, level, layer, format, length, first pixel), DESTROY");
        end   // !quick
        // ---- READ_IMAGE: every transfer format, exact 5 / 6 -> 8 expansion, sub-ranges ----
        if (!quick) create_image(21, 1, 64, 48, 134, 1'b1, 64'h2200_0000);   // destroyed above; its pixels are still there
        read_image_chk(20, 8'h01, 0, 0, 64, 48, 0, 0, 32'h101, "READ 20 RGBA8888");
        read_image_chk(20, 8'h03, 5, 7, 20, 9, 0, 0, 32'h102, "READ 20 BGRX rect");
        read_image_chk(20, 8'h04, 3, 1, 50, 40, 0, 0, 32'h103, "READ 20 -> RGB565 (rounded)");
        read_image_chk(20, 8'h0A, 0, 0, 64, 48, 1000, 480, 32'h104, "READ 20 RGB888 pixels 1000..1479");
        read_image_chk(20, 8'h0B, 0, 0, 64, 48, 77, 0, 32'h105, "READ 20 BGR888 from pixel 77 to the end");
        read_image_chk(21, 8'h01, 0, 0, 64, 48, 0, 0, 32'h106, "READ 21 RGB565 -> RGBA8888 (exact expansion)");
        read_image_chk(21, 8'h04, 10, 3, 33, 40, 5, 300, 32'h107, "READ 21 RGB565 (identity) sub-range");
        read_image_chk(21, 8'h00, 0, 0, 64, 48, 0, 0, 32'h108, "READ 21 RAW");
        read_image_chk(21, 8'h02, 1, 1, 7, 5, 0, 0, 32'h109, "READ 21 BGRA small");
        $display("ok: READ_IMAGE: RGBA8888 / BGRX / RGB565 / RGB888 / BGR888 / RAW / BGRA from RGBA8 and RGB565, first / count");
        // ---- FILL_IMAGE: byte masks (RGBA8), RGB565 plain and read-modify-write, P8 ----
        create_image(24, 0, 64, 48, 256, 1'b0, 64'h2600_0000);
        create_image(25, 1, 40, 30, 80, 1'b0, 64'h2700_0000);
        create_image(23, 2, 64, 48, 70, 1'b0, 64'h2500_0000);
        begin
            bit [31:0] pix [$];
            for (int i = 0; i < 64 * 48; i++) pix.push_back(prng(i, 40));
            write_image(24, 0, 0, 8'h01, 0, 0, 64, 48, pix, 0, 360);
            pix.delete(); for (int i = 0; i < 40 * 30; i++) pix.push_back(prng(i, 41));
            write_image(25, 0, 0, 8'h04, 0, 0, 40, 30, pix, 0, 720);
            pix.delete(); for (int i = 0; i < 64 * 48; i++) pix.push_back(prng(i, 42) & 32'hFF);
            write_image(23, 0, 0, 8'h00, 0, 0, 64, 48, pix, 0, 1440);
        end
        fence_b(32'd30);
        fill_image(24, 8'h01, 4'hF, 4, 4, 20, 10, 32'h80FF_4020);          // RGBA8888, all channels
        fill_image(24, 8'h03, 4'h5, 30, 2, 30, 40, 32'h0011_2233);          // BGRX, R and B only
        fill_image(24, 8'h00, 4'h8, 0, 40, 64, 8, 32'h7F00_0000);           // RAW, alpha only
        fill_image(25, 8'h04, 4'h7, 2, 2, 10, 10, 32'h0000_F81F);           // RGB565 plain
        fill_image(25, 8'h01, 4'h2, 5, 15, 30, 12, 32'h00C0_8040);          // G only: read-modify-write
        fill_image(25, 8'h0A, 4'h5, 0, 0, 40, 3, 32'h00FF_00FF);            // R and B: read-modify-write
        fill_image(23, 8'h00, 4'h1, 10, 10, 20, 20, 32'h0000_00AB);         // P8 index
        fence_b(32'd31);
        check_b_ddr(64'h2600_0000, 256 * 48, "FILL image 24");
        check_b_ddr(64'h2700_0000, 80 * 30, "FILL image 25");
        check_b_ddr(64'h2500_0000, 70 * 48, "FILL image 23");
        read_image_chk(23, 8'h00, 3, 2, 60, 44, 11, 0, 32'h10A, "READ 23 P8 RAW");
        $display("ok: FILL_IMAGE: RGBA8 byte masks, RGB565 plain and read-modify-write, P8 (read back as RAW)");
        // ---- COPY_IMAGE: conversion, stretch, mirrors, colour key, overlap ----
        copy_image(21, 24, 0, 0, 64, 48, 0, 0, 64, 48, 3'b000);                          // RGB565 -> RGBA8 1:1
        copy_image(20, 24, 0, 0, 64, 48, 3, 3, 30, 20, 3'b001);                          // shrink, mirror x
        copy_image(25, 24, 5, 5, 13, 7, 30, 20, 34, 27, 3'b010);                         // RGB565 enlarge, mirror y
        fence_b(32'd32);
        copy_image(20, 24, 10, 10, 40, 30, 12, 8, 40, 30, 3'b100, 32'h0000_0000, 32'h80FF_FF00); // key: R <= 0x80
        fence_b(32'd47);                                                                 // the next copy reads image 24
        copy_image(24, 25, 0, 0, 64, 48, 0, 0, 40, 30, 3'b011);                          // RGBA8 -> RGB565, both mirrors
        fence_b(32'd33);
        copy_image(25, 25, 0, 0, 30, 20, 5, 4, 30, 20, 3'b000);                          // overlap: down / right
        fence_b(32'd34);
        copy_image(25, 25, 6, 6, 30, 20, 2, 1, 30, 20, 3'b000);                          // overlap: up / left
        fence_b(32'd35);
        copy_image(25, 25, 0, 5, 40, 20, 0, 6, 40, 20, 3'b000);                          // overlap: one row down
        fence_b(32'd36);
        copy_image(23, 23, 0, 0, 50, 40, 7, 3, 50, 40, 3'b100, 32'h0000_0010, 32'h0000_0040); // P8 overlap, index key
        fence_b(32'd48);
        copy_image(23, 23, 0, 0, 20, 10, 40, 30, 24, 18, 3'b011);                        // P8 stretch + mirrors
        fence_b(32'd37);
        check_b_ddr(64'h2600_0000, 256 * 48, "COPY image 24");
        check_b_ddr(64'h2700_0000, 80 * 30, "COPY image 25");
        check_b_ddr(64'h2500_0000, 70 * 48, "COPY image 23");
        present_eth(24, 16'h0, 32'd3100); wait_img_frames(24, 2);
        present_eth(25, 16'h0, 32'd3101); wait_img_frames(25, 2);
        $display("ok: COPY_IMAGE: RGB565 <-> RGBA8, stretch up / down, mirrors, colour key (RGBA8, P8), overlapping 1:1 copies, shown");
        // ---- palette: P8 image shown through the hardware palette ----
        set_palette(0, 256, 50);
        fence_b(32'd38);
        present_eth(23, 16'h0, 32'd3102); wait_img_frames(23, 2);
        check_palette("palette 0..255");
        set_palette(16, 32, 51); set_palette(200, 56, 52); set_palette(0, 1, 53);        // several in one frame: coalesce
        fence_b(32'd39);
        wait_img_frames(23, 3);
        check_palette("palette after partial updates");
        for (int k = 0; k < 6; k++) begin set_palette(0, 256, 60 + k); wait_img_frames(23, 1); end   // animation
        fence_b(32'd40); wait_img_frames(23, 2);
        check_palette("palette after the animation");
        $display("ok: SET_PALETTE: P8 frames through the palette, partial updates coalesced, animation (%0d publishes)",
                 dut.u_disp.u_ctl.c_pal_pub);
        // ---- hardware cursor: colour (straight alpha), AND / XOR, premultiplied; moves; fast path; hide ----
        present_eth(24, 16'h0, 32'd3103); wait_img_frames(24, 1);
        cursor_shape(0, 70); cursor_show(0, 1'b1, 5, 7, 20, 15);
        fence_b(32'd41); wait_img_frames(24, 2);
        cursor_check(1, 0, 15, 8, 1, "cursor colour");
        cursor_shape(1, 71); cursor_show(1, 1'b1, 0, 0, -20, 30);                      // AND / XOR, off the left edge
        fence_b(32'd42); wait_img_frames(24, 2);
        cursor_check(1, 1, -20, 30, 1, "cursor AND / XOR");
        cursor_shape(2, 72); cursor_show(2, 1'b1, 63, 63, 70, 50);                     // premultiplied, bottom-right
        fence_b(32'd43); wait_img_frames(24, 2);
        cursor_check(1, 2, 7, -13, 1, "cursor premultiplied");
        send_seq(T_CURSOR, {8'd4, 8'd0, le16(16'h0), le16(16'd30), le16(16'd60)}, s); n_fwd++; n_curcmd++;   // MOVE
        fence_b(32'd44); wait_img_frames(24, 2);
        cursor_check(1, 2, -33, -3, 0, "cursor MOVE");
        send_seq(T_CURSOR_POS, {le16(16'd40), le16(16'd20)}, s); n_fwd++; n_curcmd++;  // sequenced CURSOR_POS
        fence_b(32'd45); wait_img_frames(24, 2);
        cursor_check(1, 2, -23, -43, 0, "cursor CURSOR_POS sequenced");
        for (int k = 0; k < 5; k++) begin cursor_fast(10 + 9 * k, 5 + 7 * k); repeat (200) @(posedge clkA); end   // fast path
        wait_img_frames(24, 2);
        cursor_check(1, 2, 46 - 63, 33 - 63, 0, "cursor fast path");
        // a shape upload while the cursor moves every frame over the fast path must not stall (position-only
        // mailbox transfers do not block shape writes)
        begin
            bit mov; int f0;
            mov = 1; f0 = frames_ok;
            fork
                while (mov) begin cursor_fast(20 + (frames_ok % 7), 20); repeat (3000) @(posedge clkA); end
                begin cursor_shape(0, 73); fence_b(32'd49); mov = 0; end
            join
            if (frames_ok - f0 > 6) fail($sformatf("shape upload during fast-path moves took %0d frames", frames_ok - f0));
            repeat (5000) @(posedge clkA);                                     // the last fast-path datagram is in
            cursor_show(0, 1'b1, 0, 0, 30, 25); fence_b(32'd50);
            wait_img_frames(24, 2);
            cursor_check(1, 0, 30, 25, 1, "cursor shape uploaded during fast-path moves");
        end
        present_eth(23, 16'h0, 32'd3104); wait_img_frames(23, 2);                        // over a P8 image
        send_seq(T_CURSOR, {8'd3, 8'd0, le16(16'h0)}, s); n_fwd++; n_curcmd++;          // HIDE
        fence_b(32'd46); wait_img_frames(23, 2);
        cursor_check(0, 0, 0, 0, 0, "cursor HIDE");
        if (fc_cursor_px < 2000) fail($sformatf("cursor coverage: %0d displayed pixels under the cursor", fc_cursor_px));
        $display("ok: cursor: colour / AND-XOR / premultiplied shapes over RGBA8 and P8, clipped at the edges, MOVE, CURSOR_POS, fast path, HIDE (%0d pixels checked under it)",
                 fc_cursor_px);
        // ---- rejects of the new commands ----
        expect_reject(T_READ_IMAGE, {le16(16'hC017), 8'd0, 8'd0, 8'h01, 8'h00, le16(16'h0), le16(16'd0), le16(16'd0),
                      le16(16'd1), le16(16'd1), le32(0), le32(0), le32(32'h1)}, ERR_RANGE);          // P8 read as RGBA8888
        expect_reject(T_READ_IMAGE, {le16(16'hC014), 8'd0, 8'd0, 8'h01, 8'h00, le16(16'h0), le16(16'd0), le16(16'd0),
                      le16(16'd2), le16(16'd2), le32(4), le32(0), le32(32'h1)}, ERR_RANGE);          // first = w * h
        expect_reject(T_FILL_IMAGE, {le16(16'hC018), 8'd0, 8'd0, 8'h01, 8'h0F, le16(16'h0), le16(16'd60), le16(16'd0),
                      le16(16'd5), le16(16'd1), le32(0)}, ERR_RANGE);                                 // outside
        expect_reject(T_COPY_IMAGE, {le16(16'hC017), 8'd0, 8'd0, le16(16'hC018), 8'd0, 8'd0, le16(0), le16(0), le16(4),
                      le16(4), le16(0), le16(0), le16(4), le16(4), le32(0), le32(0), le32(0)}, ERR_RANGE);  // P8 -> RGBA8
        expect_reject(T_COPY_IMAGE, {le16(16'hC018), 8'd0, 8'd0, le16(16'hC019), 8'd0, 8'd0, le16(0), le16(0), le16(4),
                      le16(4), le16(38), le16(0), le16(4), le16(4), le32(0), le32(0), le32(0)}, ERR_RANGE); // dst outside
        expect_reject(T_SET_PALETTE, {le16(16'd0), le16(16'd0)}, ERR_RANGE);                         // count 0
        expect_reject(T_SET_PALETTE, {le16(16'd250), le16(16'd10)}, ERR_RANGE);                      // past 255
        expect_reject(T_SET_PALETTE, {le16(16'd0), le16(16'd2), le32(0)}, ERR_LEN);                  // one entry short
        expect_reject(T_CURSOR, {8'd9, 8'd0, le16(16'h0)}, ERR_RANGE);                               // unknown op
        expect_reject(T_CURSOR, {8'd2, 8'd3, 8'd0, 8'd0, 8'd0, 8'd0, le16(16'h0), le32(0)}, ERR_RANGE);   // mode 3
        expect_reject(T_CURSOR, {8'd1, 8'd0, le16(16'h0), le16(16'd4090), le16(16'h0), le32(0), le32(0), le32(0),
                      le32(0), le32(0), le32(0), le32(0)}, ERR_RANGE);                               // past pixel 4095
        $display("ok: rejects: READ format / first, FILL rectangle, COPY formats / rectangle, SET_PALETTE count / range / length, CURSOR op / mode / shape range");
        present_eth(20, 16'h0, 32'd3105); wait_img_frames(20, 1);
        if (quick) begin
            if (errors == 0) $display("TB_PASS tb_two_boards (QUICK2D)"); else $display("TB_FAIL tb_two_boards (QUICK2D) errors=%0d", errors);
            $finish;
        end
        // ---- stress (a): B's command processor stalls while a burst larger than its command buffer arrives ----
        // Credit flow control: B's 4096-word command buffer fills, board A stops sending (credit wait) and the
        // link never raises XOFF; a cursor fast-path move sent during the stall reaches B and is applied at the next
        // vsync while the processor is still stalled (it does not wait behind the queued commands).
        begin
            bit [31:0] pix [$]; int e0, x0, cw0, cp0, t;
            e0 = n_berr;
            present_eth(8, 16'h0, 32'd3010); wait_img_frames(8, 1);             // image 20 is rewritten off screen
            x0 = dut.l_xoff; cw0 = da.u_lport.c_credit_wait; cp0 = dut.u_lport.c_cur_pkts;
            // a 4 MB harness MEMTEST (32 MB, away from every image) keeps the bring-up harness active for ~5 ms: the
            // command processor waits at its next command meanwhile
            @(posedge clkB); b_dbg_v <= 1'b1; b_dbg_cmd <= {8'h03, 10'd32, 10'd4, 4'h0, 32'h5A5A_0001};
            @(posedge clkB); b_dbg_v <= 1'b0;
            for (int pass = 0; pass < 3; pass++) begin                               // 3 x 30 RGBA8888 datagrams, 37 KB
                pix.delete();
                for (int i = 0; i < 64 * 48; i++) pix.push_back(prng(i, 11 + pass * 100));
                write_image(20, 0, 0, 8'h01, 0, 0, 64, 48, pix, 0, 105);
            end
            repeat (20_000) @(posedge clkB);
            if (!dut.h_active) fail("stress (a): the harness stall ended too early");
            if (dut.u_lport.cb_cnt < 14'd3600)
                fail($sformatf("stress (a): B's command buffer holds %0d words, expected it nearly full", dut.u_lport.cb_cnt));
            if (da.u_lport.c_credit_wait == cw0) fail("stress (a): board A never waited for credit");
            cursor_fast(33, 22);
            t = 0; while (dut.u_lport.c_cur_pkts == cp0 && t < 2000) begin @(posedge clkB); t++; end
            if (dut.u_lport.c_cur_pkts == cp0) fail("stress (a): the cursor packet did not reach B during the stall");
            wait_frame_start(); wait_frame_start();
            if (!dut.h_active) fail("stress (a): the harness stall ended before the cursor check");
            // hotspot (0, 0): the last SHOW of the cursor section
            if ($signed(dut.u_disp.u_dout.u_post.c_ox) != 33 || $signed(dut.u_disp.u_dout.u_post.c_oy) != 22)
                fail($sformatf("stress (a): cursor at %0d,%0d during the stall, expected %0d,%0d",
                               $signed(dut.u_disp.u_dout.u_post.c_ox), $signed(dut.u_disp.u_dout.u_post.c_oy), 33, 22));
            fence_b(32'd15);
            if (dut.l_xoff != x0) fail($sformatf("stress (a): %0d XOFF events on B's receiver", dut.l_xoff - x0));
            check_b_ddr(64'h2100_0000, 288 * 48, "image 20 after the stall");
            if (n_berr != e0) fail("errors during the stalled burst");
            present_eth(20, 16'h0, 32'd3004); wait_img_frames(20, 2);
            $display("ok: stress (a): 90 WRITE_IMAGE datagrams through a stalled B: command buffer full, A waited %0d cycles for credit, no XOFF, cursor fast path applied during the stall, image exact",
                     da.u_lport.c_credit_wait - cw0);
        end
        // ---- stress (b): B retrains in the middle of a WRITE_IMAGE stream ----
        // A must wait for the link (no ERR_NO_B), A's transmitter rewinds to its oldest unacknowledged packet, B keeps its
        // expected sequence and drops what it already delivered (no command executed twice: B commands = forwarded)
        begin
            bit [31:0] pix [$]; int tc;
            tc = dut.l_train_cnt;
            present_eth(8, 16'h0, 32'd3011); wait_img_frames(8, 1);
            for (int i = 0; i < 64 * 48; i++) pix.push_back(prng(i, 12));
            fork
                write_image(20, 0, 0, 8'h03, 0, 0, 64, 48, pix, 0, 120);           // 26 BGRX datagrams
                begin
                    // retrain the cycle B commits a packet of the stream: its ACK cannot have reached A yet, so A
                    // resends it after the relink; B must not deliver it a second time
                    while (!dut.iw_dv) @(posedge clkB);
                    repeat (3) @(posedge dut.u_link.g_b.u_arq_rx.commit);
                    @(negedge clkB); force dut.u_link.g_b.u_phy_rx.u_trainer.retrain = 1'b1;
                    repeat (2) @(negedge clkB); release dut.u_link.g_b.u_phy_rx.u_trainer.retrain;
                end
            join
            fence_b(32'd16);
            if (dut.l_train_cnt == tc) fail("B did not retrain");
            check_b_ddr(64'h2100_0000, 288 * 48, "image 20 after B's retrain");
            present_eth(20, 16'h0, 32'd3005); wait_img_frames(20, 2);
            $display("ok: stress (b): B retrained mid-stream (trainings %0d -> %0d), A waited, image exact", tc, dut.l_train_cnt);
        end
        if (full) begin
            // modes 0..4 (4 = 1024 x 768: image 8, 203 x 151, black around it)
            for (int m = 0; m < 5; m++) begin
                set_mode_eth(m, 20000);
                present_eth(2 * m, 16'h0, 32'(2000 + m), 40000);   // two 60 Hz frames + the trip through both boards
                wait_img_frames(2 * m, 1, 3);
                $display("ok: mode %0d over Ethernet, full frame checked", m);
            end
            set_mode_eth(5);
        end
        // ---- A STATUS: board B and link words (after one B STATUS period, so B's counters are current) ----
        repeat (30_000) @(posedge clkA);
        t = n_status; send_raw(msg(T_STATUS_REQ, 0, {}));
        while (n_status == t) @(posedge clkA);
        if (st_w[0] != STATUS_LAYOUT) fail($sformatf("STATUS layout %0d", st_w[0]));
        if ((st_w[42] & 32'h0F) != 32'h0F) fail($sformatf("STATUS link flags %08h (b_alive, lock, trained, far_up)", st_w[42]));
        if (((st_w[37] >> 2) & 1) != 1 || ((st_w[37] >> 4) & 7) != 5) fail($sformatf("STATUS B flags %08h: video on, mode 5 expected", st_w[37]));
        if (st_w[38] == 0) fail("STATUS B frame counter 0");
        if (((st_w[37] >> 16) & 32'h3FF) != (full ? 32'd6 : 32'd20)) fail($sformatf("STATUS B image on screen %0d", (st_w[37] >> 16) & 32'h3FF));
        if (st_w[43] != n_fwd) fail($sformatf("STATUS forwarded %0d, expected %0d", st_w[43], n_fwd));
        if (st_w[44] != n_bev) fail($sformatf("STATUS B events %0d, expected %0d", st_w[44], n_bev));
        // B command errors: SET_MODE 7, the two bad present handles, and n_berr from the memory / image section
        if (st_w[41][15:0] != 16'(n_fwd) || st_w[41][31:16] != 16'(3 + n_berr))
            fail($sformatf("STATUS B commands / errors %08h, expected %0d / %0d", st_w[41], n_fwd, 3 + n_berr));
        $display("ok: A STATUS: link %08h, B flags %08h frames %0d pixel %0d kHz, forwarded %0d, B events %0d",
                 st_w[42], st_w[37], st_w[38], st_w[39], st_w[43], st_w[44]);
        // layout 3: link control / diagnostics, B's receiver and transfer counters
        if (st_w[48] != 32'h1) fail($sformatf("STATUS linkctl %08h (fill on by default)", st_w[48]));
        if (st_w[55] == 0) fail("STATUS: no fill words received by A (B's fill words between its packets)");
        if (st_w[56] != 0 || st_w[57] != 0) fail($sformatf("STATUS A receiver parity errors: fill %0d real %0d", st_w[56], st_w[57]));
        if (st_w[50] == 0) fail("STATUS: A receiver diagnostics counted no bursts");
        if (st_w[60] != 0) fail($sformatf("STATUS B receiver crc/parity %08h", st_w[60]));
        if (dut.u_link.diag[5*32 +: 32] == 0) fail("no fill words received by B (A's fill words between its IDLE / CMD packets)");
        if (dut.u_link.diag[6*32 +: 64] != 0) fail("B receiver diagnostics: parity errors");
        if (st_w[61][16] != 1'b1) fail($sformatf("STATUS B link info %08h: fill not on", st_w[61]));
        if (st_w[62] != n_images) fail($sformatf("STATUS B images created %0d, expected %0d", st_w[62], n_images));
        if (st_w[63] != n_pixels) fail($sformatf("STATUS B pixels written %0d, expected %0d", st_w[63], n_pixels));
        $display("ok: A STATUS layout 3: linkctl %08h, A rx fill words %0d, bursts %0d, B images %0d, B pixels %0d",
                 st_w[48], st_w[55], st_w[50], st_w[62], st_w[63]);
        // layout 4: B's receive diagnostics (clean here), XOFF / ARQ / word counters of both boards
        if (st_w[64] != 0 || st_w[65] != 0 || st_w[66] != 0)
            fail($sformatf("STATUS B receiver diagnostics not clean: hist %08h last %08h parmap %08h", st_w[64], st_w[65], st_w[66]));
        if (st_w[70] != n_fwd) fail($sformatf("STATUS B CMD packets %0d, expected %0d (forwarded)", st_w[70], n_fwd));
        if (st_w[71] == 0 || st_w[74] == 0) fail($sformatf("STATUS word counters B %0d A %0d", st_w[71], st_w[74]));
        if (st_w[75] != 32'hC0A8010A || st_w[3][26:25] != 2'b01)
            fail($sformatf("STATUS address %08h flags %08h: expected the fixed 192.168.1.10", st_w[75], st_w[3]));
        $display("ok: A STATUS layout 4: B rx words %0d xoff %0d retx %08h disc %08h, A rx words %0d xoff %0d naks/timeouts %08h",
                 st_w[71], st_w[67], st_w[68], st_w[69], st_w[74], st_w[72], st_w[73]);
        // layout 5: credit (everything sent was taken and reported), cursor, palette, READ / FILL / COPY counters
        if (st_w[76] != st_w[77] || st_w[80] != st_w[76])
            fail($sformatf("STATUS credit: A sent %0d, reported %0d, B taken %0d", st_w[76], st_w[77], st_w[80]));
        if (st_w[78] == 0) fail("STATUS: no credit wait counted (stress (a) fills B's buffer)");
        // credit sync: one flush + one SYNC at the start, none for stress (b)'s retrain (no restart)
        if (st_w[79][15:0] != 2 || dut.u_lport.c_sync != 2 || !da.u_lport.synced)
            fail($sformatf("STATUS: %0d credit SYNCs sent, B took %0d, A synced %b (expected 2 / 2 / 1)",
                           st_w[79][15:0], dut.u_lport.c_sync, da.u_lport.synced));
        if (st_w[79][31:16] != 16'(n_fastpos) || st_w[81][31:16] != 16'(n_fastpos))
            fail($sformatf("STATUS cursor fast path: A sent %0d, B received %0d, expected %0d", st_w[79][31:16], st_w[81][31:16], n_fastpos));
        if (st_w[81][15:0] != 16'(n_pal)) fail($sformatf("STATUS palette publishes %0d, expected %0d", st_w[81][15:0], n_pal));
        if (st_w[82][31:16] != 16'(n_curcmd)) fail($sformatf("STATUS cursor commands %0d, expected %0d", st_w[82][31:16], n_curcmd));
        if (st_w[82][15:0] != 16'(n_xops)) fail($sformatf("STATUS READ / FILL / COPY ops %0d, expected %0d", st_w[82][15:0], n_xops));
        $display("ok: A STATUS layout 5: credit sent %0d = taken, waited %0d cycles, cursor fast path %0d, palette %0d, cursor cmds %0d, xfer ops %0d (%0d pixels)",
                 st_w[76], st_w[78], st_w[79][31:16], st_w[81][15:0], st_w[82][31:16], st_w[82][15:0], st_w[83]);
        // ---- final ----
        if (bsw(5) != 0) fail($sformatf("%0d line-buffer underruns on B", bsw(5)));
        if (bad_ctl != 0) fail($sformatf("%0d bad TMDS control words", bad_ctl));
        if (a_mig_err != 0 || b_mig_err != 0) fail($sformatf("MIG model errors A %0d B %0d", a_mig_err, b_mig_err));
        if (ev_seq.size() != 0) fail($sformatf("%0d unexpected EVENTs left", ev_seq.size()));
        $display("INFO_TB frames %0d words %0d link A: tx %0d rx %0d crc %0d retx %08h disc %08h  B: crc %0d",
                 frames_ok, words, da.l_tx_pkts, da.l_rx_pkts, da.l_crc, da.l_retx, da.l_disc, dut.l_crc);
        if (errors == 0) $display("TB_PASS tb_two_boards%s", full ? " (FULL)" : "");
        else $display("TB_FAIL tb_two_boards errors=%0d", errors);
        $finish;
    end
    initial begin #(full_limit()); $display("TB_FAIL tb_two_boards timeout"); $finish; end
    function automatic real full_limit(); return $test$plusargs("FULL") ? 700_000_000.0 : 150_000_000.0; endfunction
endmodule
`default_nettype wire
