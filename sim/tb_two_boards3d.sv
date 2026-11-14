`timescale 1ns/1ps
// tb_two_boards3d — the 3D path end to end: host (Ethernet, Go-Back-N) -> board_a_core (geometry: vertex core, setup,
// records, bins; the render side's RSTREAM) -> JM1/JM2 cables -> board_b_core (quad setup, FS, ROP) -> B's DDR.
// The host sends B's resources and draw state (tools/gen/ab_<seed>_cmds.hex up to the FLUSH, from b_ref.py aframe:
// program buffer, the target's and the depth-stencil image's initial pixels, CREATE_IMAGE, DRAW_STATE), then board
// A's draw (as_<seed>_cmds.hex as datagrams, the buffers uploaded from av_<seed>_ddr.hex, as tb_board_a), then the
// FLUSH (A forwards it and streams the frame) and a FENCE (B's EVENT comes back through A). The target image is read
// back from B's DDR (MEM_READ board 1, READ_DATA relayed by A) and compared with ab_<seed>_expect.hex bit for bit.
//   +SEED_<n>   (11, 12: tools/gen_a_goldens.sh, then python3 tools/b_ref.py aframe <n>)
//   +BATCH      every sequenced command (B's resources and draw state, A's geometry, the FLUSHes, the FENCE) goes in
//               BATCH datagrams of up to 1400 body bytes (fb_pkg T_BATCH); the FENCE's EVENT must come back from B with
//               its sub-message index (B echoes the CMD header's byte 3)
`default_nettype none
module tb_two_boards3d;
    import net_pkg::*;
    import fb_pkg::*;
    reg clkA = 0, clk200A = 0, clkB = 0, clk200B = 0;
    always #5.0    clkA = ~clkA;
    always #2.5    clk200A = ~clk200A;
    always #4.9998 clkB = ~clkB;
    always #2.4999 clk200B = ~clk200B;
    reg  rstA = 1, rstB = 1, calA = 0, calB = 0;
    int  errs = 0;
    task automatic fail(input string m); errs++; $display("FAIL: %s", m); endtask

    // ---------------- cables (tb_board_link) ----------------
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
    board_a_core #(.BUILD_ID(32'h0000_0A03), .IDELAY_DEFAULT(5'd3), .RING_LOG2(10), .MDIO_RST_MS(1), .MDIO_WAIT_MS(1),
                   .MDIO_POLL_US(100), .STATUS_PERIOD(300_000), .TRIAL_CYCLES(50_000), .PHY_TRIAL_CYCLES(600_000),
                   .SNAP_CYCLES(2000), .JTAG_STATUS(1'b0),
                   .LINK_MHZ(100), .LINK_GUARD(3000), .LINK_MMCM_RST(20_000), .LINK_IDLE_TO(200_000), .LINK_TRAIN_WORDS(256),
                   .LINK_RETRY(20_000), .LINK_RELOCK(50_000), .LINK_RWIN(200_000), .KEEPALIVE(20_000), .B_ALIVE(100_000),
                   .RENDER_TIMEOUT(200_000), .LINK_SIM_SHIFTER(1'b1)) da (     // a frame B does not take is given up after 2 ms
        .clk_sys(clkA), .rst_sys(rstA), .calib_done(calA), .clk_idelay(clk200A),
        .rgmii_rxc(rxc), .rgmii_rx_ctl(rx_ctl), .rgmii_rxd(rxd), .rgmii_txc(txc), .rgmii_tx_ctl(tx_ctl), .rgmii_txd(txd),
        .phy_rstn(phy_rstn), .mdc(mdc), .mdio_o(mdio_o), .mdio_oe(mdio_oe), .mdio_i(mdio),
        .app_en(aen), .app_cmd(acmd), .app_addr(aaddr), .app_wdf_data(awd), .app_wdf_mask(amask),
        .app_wdf_wren(awren), .app_wdf_end(aend), .app_rdy(ardy), .app_wdf_rdy(awrdy),
        .app_rd_data_valid(arv), .app_rd_data(ard), .jm1(cab1A), .jm2(cab2A), .led(ledA));

    // ---------------- board B ----------------
    wire        ben, bwren, bend, brdy, bwrdy, brv; wire [2:0] bcmd; wire [28:0] baddr; wire [255:0] bwd, brd; wire [31:0] bmask;
    int b_mig_err; longint b_nwr, b_nrd;
    mig_app_model #(.RD_LAT(20), .RDY_PCT(85), .WDF_PCT(85), .SEED(5)) u_migB (
        .ui_clk(clkB), .rst(rstB), .app_en(ben), .app_cmd(bcmd), .app_addr(baddr), .app_wdf_data(bwd), .app_wdf_mask(bmask),
        .app_wdf_wren(bwren), .app_wdf_end(bend), .app_rdy(brdy), .app_wdf_rdy(bwrdy), .app_rd_data(brd),
        .app_rd_data_valid(brv), .errors(b_mig_err), .n_wr(b_nwr), .n_rd(b_nrd));
    wire hdmi_clk_p, hdmi_clk_n; wire [2:0] hdmi_data_p, hdmi_data_n; wire [1:0] ledB; wire [24*32-1:0] bstat;
    board_b_core #(.BUILD_ID(32'd80), .DEFAULT_MODE(5), .SIM_MODE4(1'b1), .IMG_SET(1), .DEBOUNCE(100),
                   .LONG_PRESS(5000), .LOCK_TIMEOUT(200_000), .MS_CYCLES(1000), .JTAG(1'b0),
                   .LINK_MHZ(100), .LINK_GUARD(3000), .LINK_MMCM_RST(20_000), .LINK_IDLE_TO(200_000), .LINK_TRAIN_WORDS(256),
                   .LINK_RETRY(20_000), .LINK_RELOCK(50_000), .LINK_RWIN(200_000), .LSTAT_CYCLES(10_000),
                   .LINK_SIM_SHIFTER(1'b1), .B3D_NLINES(4), .B3D_MAXW(1280)) db (   // the board's MAXW: 8-line bands up to 640 wide need its band buffer
        .clk_sys(clkB), .rst_sys(rstB), .calib_done(calB), .clk200(clk200B), .key1_n(1'b1),
        .dbg_cmd_valid(1'b0), .dbg_cmd(64'h0),
        .app_en(ben), .app_cmd(bcmd), .app_addr(baddr), .app_wdf_data(bwd), .app_wdf_mask(bmask),
        .app_wdf_wren(bwren), .app_wdf_end(bend), .app_rdy(brdy), .app_wdf_rdy(bwrdy),
        .app_rd_data_valid(brv), .app_rd_data(brd),
        .hdmi_clk_p(hdmi_clk_p), .hdmi_clk_n(hdmi_clk_n), .hdmi_data_p(hdmi_data_p), .hdmi_data_n(hdmi_data_n),
        .jm1(cab1B), .jm2(cab2B), .led(ledB), .status_vec(bstat));

    // ---------------- host model (as tb_board_a) ----------------
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
    function automatic [31:0] gle32(input bq q, input int i); return {q[i+3], q[i+2], q[i+1], q[i]}; endfunction
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
    int        n_ack = 0, n_status = 0, n_err_ev = 0;
    bit [31:0] ack_exp, ack_flags, ack_free;
    bit [31:0] st_w [0:STATUS_WORDS-1];
    bit [31:0] ev_seq [$], ev_code [$], ev_a0 [$], ev_a1 [$], ev_a2 [$];
    byte unsigned rd_bytes [bit [31:0]][int];    // READ_DATA bytes by tag and offset
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
                R_EVENT: begin
                    ev_seq.push_back(gle32(pl, 4)); ev_code.push_back(gle32(pl, 8)); ev_a0.push_back(gle32(pl, 12));
                    ev_a1.push_back(gle32(pl, 16)); ev_a2.push_back(gle32(pl, 20));
                    if (gle32(pl, 8) == EV_ERROR) begin
                        n_err_ev++;
                        $display("  EV_ERROR seq %0d code %0d type %h detail %h", gle32(pl, 4), gle32(pl, 12), gle32(pl, 16), gle32(pl, 20));
                    end
                end
                R_READ_DATA: begin
                    bit [31:0] tag, off;
                    tag = gle32(pl, 8); off = gle32(pl, 12);
                    for (int i = 16; i < pl.size(); i++) rd_bytes[tag][off + i - 16] = pl[i];
                end
                default: ;
            endcase
        end
    end
    // Go-Back-N sender
    bit [31:0] next_seq = 1;
    bq         sent_q [$];
    bit [31:0] sent_seq [$];
    int        n_resend = 0;
    localparam int WINDOW = 8;
    task automatic send_raw(input bq m); u_phy.send_frame(udp_frame(m)); endtask
    longint last_progress = 0; int unsigned cyc = 0;
    task automatic send_seq(input [7:0] t, input bq body, output bit [31:0] seq);
        seq = next_seq; next_seq++;
        while (sent_q.size() >= WINDOW) @(posedge clkA);
        if (sent_q.size() == 0) last_progress = cyc;
        begin bq m = msg(t, seq, body); sent_q.push_back(m); sent_seq.push_back(seq); send_raw(m); end
    endtask
    // +BATCH: sequenced commands collected into BATCH datagrams (sub-messages {u8 type, u8 0, u16 bytes, body padded})
    bit use_batch = 0; bq batch_q; int batch_cnt = 0, n_batch = 0;
    task automatic batch_send(output bit [31:0] seq);
        send_seq(T_BATCH, batch_q, seq); batch_q.delete(); batch_cnt = 0; n_batch++;
    endtask
    task automatic batch_flush();
        bit [31:0] s_;
        if (batch_cnt != 0) batch_send(s_);
    endtask
    // a command: plain, or (+BATCH) into the open batch, sent when it would exceed 1400 body bytes; `idx` = its sub-message
    // index (0 plain) and `seq` its datagram's seq once sent (`flush`: send the batch now, so seq is known for an EVENT)
    task automatic send_cmd(input [7:0] t, input bq body, input bit flush, output bit [31:0] seq, output bit [7:0] idx);
        if (!use_batch) begin send_seq(t, body, seq); idx = 0; end
        else begin
            if (batch_q.size() + 4 + body.size() > 1400) batch_flush();
            batch_q = {batch_q, t, 8'h00, le16(16'(body.size())), body};
            while (batch_q.size() % 4) batch_q.push_back(8'h00);
            idx = 8'(batch_cnt); batch_cnt++; seq = 0;
            if (flush) batch_send(seq);
        end
    endtask
    always @(posedge clkA) cyc++;
    task automatic resend_all();
        int n = sent_q.size();
        for (int i = 0; i < n && i < sent_q.size(); i++) begin send_raw(sent_q[i]); n_resend++; end
    endtask
    initial forever begin
        @(ack_ev);
        while (sent_seq.size() > 0 && $signed(sent_seq[0] - ack_exp) < 0) begin
            void'(sent_q.pop_front()); void'(sent_seq.pop_front()); last_progress = cyc;
        end
        if ((ack_flags[ACKF_NAK] || ack_flags[ACKF_FULL]) && sent_q.size() > 0) begin
            if (ack_flags[ACKF_FULL]) repeat (2000) @(posedge clkA);
            resend_all();
        end
    end
    initial forever begin
        repeat (1000) @(posedge clkA);
        if (sent_q.size() > 0 && cyc - last_progress > 60000) begin last_progress = cyc; resend_all(); end
    end
    task automatic wait_acked(input int tmo_us = 5000);
        int t = 0;
        while (sent_q.size() > 0 && t < tmo_us * 100) begin @(posedge clkA); t++; end
        if (sent_q.size() > 0) fail($sformatf("datagrams not acknowledged: %0d (exp %0d)", sent_q.size(), ack_exp));
    endtask
    // the EVENT's code word = {16'h0, sub-message index (BATCH), code}
    task automatic expect_event(input [31:0] seq, input [31:0] code, input [31:0] a0, input bit chk_a0, input int tmo_us = 3000,
                                input [7:0] idx = 8'd0);
        int t = 0;
        while (ev_seq.size() == 0 && t < tmo_us * 100) begin @(posedge clkA); t++; end
        if (ev_seq.size() == 0) begin fail($sformatf("no EVENT for seq %0d", seq)); return; end
        if (ev_seq[0] != seq || ev_code[0] != {16'h0, idx, code[7:0]} || (chk_a0 && ev_a0[0] != a0))
            fail($sformatf("EVENT seq %0d code %h a0 %h a1 %h (exp seq %0d code %h idx %0d a0 %h)", ev_seq[0], ev_code[0], ev_a0[0], ev_a1[0], seq, code, idx, a0));
        void'(ev_seq.pop_front()); void'(ev_code.pop_front()); void'(ev_a0.pop_front()); void'(ev_a1.pop_front()); void'(ev_a2.pop_front());
    endtask
    // a host command of the ab_<seed>_cmds.hex file (w0 = {type, bytes}, w1 = its own seq, body words); flush: send
    // its batch at once (an EVENT is expected under seq / idx)
    task automatic send_words(input reg [31:0] w [], input int n, output bit [31:0] s, input bit flush = 0, output bit [7:0] idx);
        bq body;
        for (int i = 2; i < n; i++) body = {body, le32(w[i])};
        send_cmd(w[0][23:16], body, flush, s, idx);
    endtask
    // MEM_READ board 1 of [a0, a0 + len) -> rd_bytes[tag]
    task automatic mem_read_b(input longint a0, input int len, input bit [31:0] tag);
        bit [31:0] s; int t;
        rd_bytes.delete(tag);
        send_seq(T_MEM_READ, {8'd1, 8'd0, le16(16'h0), le32(32'(a0)), le32(32'(len)), le32(tag)}, s);
        t = 0;
        while ((!rd_bytes.exists(tag) || rd_bytes[tag].size() < len) && t < 4_000_000) begin @(posedge clkA); t++; end
        if (!rd_bytes.exists(tag) || rd_bytes[tag].size() != len)
            fail($sformatf("READ_DATA %0d of %0d bytes (tag %h)", rd_bytes.exists(tag) ? rd_bytes[tag].size() : 0, len, tag));
    endtask

    // ---------------- a state monitor while the frame renders (every 500 us of sim time) ----------------
    bit in_frame = 0;
    bit lost_abort = 0, to_abort = 0, sync_abort = 0, held = 0, lost_armed = 0;   // +LOSTABORT / +TOABORT / +SYNCABORT below
    // B held (+LOSTABORT / +TOABORT / +SYNCABORT), then let go: B stopped — no CMD word and no render beat taken for 3 ms
    // while busy (a frame here takes < 2 ms), the board's hang
    initial begin
        reg [31:0] c0, r0; int still = 0;
        wait (lost_armed);
        forever begin
            c0 = db.u_lport.consumed; r0 = db.u_lport.rs_consumed;
            #1ms;
            still = (c0 == db.u_lport.consumed && r0 == db.u_lport.rs_consumed && db.u_cp.busy) ? still + 1 : 0;
            if (still == 3) begin
                $display("tb_two_boards3d: B HUNG at %0t: command processor st %0d (busy %b), b_render ms %0d (busy %b), render buffer %0d beats, A sent %0d / B consumed %0d beats, CMD words unconsumed %0d, B frames %0d, A dispatcher st %0d",
                         $time, db.u_cp.st, db.u_cp.busy, db.u_render.ms, db.u_render.busy, db.u_lport.rsf_cnt, da.u_lport.rs_sent, da.u_lport.rs_consumed,
                         da.u_lport.sent - da.u_lport.consumed, db.rd_frames, da.u_disp.st);
                $display("TB_FAIL tb_two_boards3d: B hung");
                $finish;
            end
        end
    end
    initial forever begin
        #500us;
        if (in_frame)
            $display("MON @%0t: A disp st %0d g_busy %b render st %0d band %0d quads %0d | link rs_sent %0d consumed %0d pkts %0d fifo %0d ts %0d synced %b | B rs buffered %0d quads %0d tris %0d frames %0d cp busy %b draws %0d",
                     $time, da.u_disp.st, da.g_busy, da.u_geom.u_rn.st, da.u_geom.u_rn.band, da.u_geom.c_quads,
                     da.u_lport.rs_sent, da.u_lport.rs_consumed, da.u_lport.c_rs_pkts, da.u_lport.rsf_cnt, da.u_lport.ts, da.u_lport.synced,
                     db.u_lport.rsf_cnt, db.rd_quads, db.rd_tris, db.rd_frames, db.cp_busy, db.cp_draws);
        if (in_frame)
            $display("MON2: A lport rx st %0d rd_pend %b evq %0d/%0d o_pend %b cmd sent %0d consumed %0d ys %0d | B lport tx st %0d tx_valid %b rs_rcvd %0d rs_consumed %0d rs_reported %0d cmd rcvd %0d consumed %0d | B rd_errs %0d cp cmds %0d errs %0d | host acks %0d events %0d",
                     da.u_lport.rs, da.u_lport.rd_pend, da.u_lport.ev_w, da.u_lport.ev_r, da.u_lport.o_pend, da.u_lport.sent, da.u_lport.consumed, da.u_lport.ys,
                     db.u_lport.xs, db.u_lport.tx_valid, db.u_lport.rs_rcvd, db.u_lport.rs_consumed, db.u_lport.rs_reported, db.u_lport.rcvd, db.u_lport.consumed,
                     db.rd_errs, db.cp_cmds, db.cp_errs, n_ack, ev_seq.size());
    end

    // ---------------- the test ----------------
    initial begin
        string seed; int fd, r, n, nb, k, t; reg [31:0] w [0:300]; reg [31:0] s, sfence; reg [255:0] bd; reg [31:0] bi;
        bit [31:0] img [longint]; bq body; bit [7:0] ty; int blen_; longint t0;
        reg [31:0] pv [0:23]; reg [51:0] c16; int W, H, bpp; reg [31:0] e; longint base; int bad;
        reg [31:0] flush_w [0:300]; int flush_n, fence_n; reg [31:0] fence_w [0:300]; int n_abort_ev = 0;
        reg [31:0] rs0, sy0, sflush;
        // B's MEM_WRITEs of the initial target / depth-stencil contents, kept for the second frame. 4 096 holds an 800- or
        // 1040-wide seed's memory (seeds 42 / 43); with too few the second frame draws over the first frame's result where
        // the rewrite stops (colour and depth differ)
        localparam int MWMAX = 4096;
        reg [31:0] mw [0:MWMAX-1][0:300]; int mw_n [0:MWMAX-1]; int nmw;
        bit [7:0] ix;
        if (!$value$plusargs("SEED_%s", seed)) seed = "11";
        use_batch = $test$plusargs("BATCH");
        repeat (20) @(posedge clkA); rstA <= 0; rstB <= 0;
        repeat (200) @(posedge clkA); calA <= 1; calB <= 1;
        wait (da.u_eth.mdio_init_done);
        physr <= 16'h002C;
        wait (da.u_eth.link_up); repeat (2000) @(posedge clkA);
        send_raw(msg(T_HELLO, 0, le32(1)));
        @(ack_ev); if (ack_exp != 1) fail($sformatf("HELLO ACK exp %0d", ack_exp));
        last_progress = cyc;
        // ---- the board link up both ways, B alive ----
        t = 0;
        while (!da.link_ok && t < 4_000_000) begin @(posedge clkA); t++; end
        if (!da.link_ok) begin fail("board link not up"); $display("TB_FAIL tb_two_boards3d"); $finish; end
        $display("tb_two_boards3d seed %s: board link up at %0t", seed, $time);
        // ---- B's resources and draw state (ab_<seed>_cmds.hex up to the FLUSH; FLUSH and FENCE kept for later) ----
        fd = $fopen({"tools/gen/ab_", seed, "_cmds.hex"}, "r");
        if (fd == 0) begin $display("TB_FAIL tb_two_boards3d: no tools/gen/ab_%s_cmds.hex (python3 tools/b_ref.py aframe %s)", seed, seed); $finish; end
        n = 0; flush_n = 0; fence_n = 0; nb = 0; nmw = 0;
        forever begin
            r = $fscanf(fd, "%h", n);
            if (r != 1 || n == 0) break;
            for (int i = 0; i < n; i++) r = $fscanf(fd, "%h", w[i]);
            if (w[0][23:16] == T_FLUSH) begin flush_n = n; for (int i = 0; i < n; i++) flush_w[i] = w[i]; end
            else if (w[0][23:16] == T_FENCE) begin fence_n = n; for (int i = 0; i < n; i++) fence_w[i] = w[i]; end
            else begin
                send_words(w, n, s, 0, ix); nb++;
                if (w[0][23:16] == T_MEM_WRITE) begin                      // B's initial memory (target, depth-stencil image): kept for the second frame
                    if (nmw == MWMAX) fail($sformatf("more than %0d MEM_WRITEs: the second frame's memory cannot be restored", MWMAX));
                    else begin mw_n[nmw] = n; for (int i = 0; i < n; i++) mw[nmw][i] = w[i]; nmw++; end
                end
            end
        end
        $fclose(fd);
        batch_flush(); wait_acked(60000);
        $display("tb_two_boards3d: %0d B commands sent (%0d batches) at %0t", nb, n_batch, $time);
        // ---- board A's draw (as tb_board_a's 3D section): the buffers from the DDR image, the command stream ----
        fd = $fopen({"tools/gen/av_", seed, "_ddr.hex"}, "r");
        while ($fscanf(fd, "%h %h", bi, bd) == 2) for (k = 0; k < 8; k++) img[longint'(bi) * 8 + k] = bd[k*32 +: 32];
        $fclose(fd);
        fd = $fopen({"tools/gen/as_", seed, "_cmds.hex"}, "r");
        nb = 0;
        while ($fscanf(fd, "%h", bd) == 1) begin
            if (bd[31:24] == Q_MARK) begin
                ty = bd[23:16]; blen_ = bd[15:0]; body.delete();
                for (k = 0; k < (blen_ + 31) / 32; k++) begin
                    void'($fscanf(fd, "%h", bd));
                    for (int j = 0; j < 32 && body.size() < blen_; j++) body.push_back(bd[j*8 +: 8]);
                end
                send_cmd(ty, body, ty == T_FENCE, s, ix); nb++;
                if (ty == T_FENCE) expect_event(s, EV_FENCE, 32'h5EED, 1, 200000, ix);   // forwarded to B: its EVENT comes back
                if (ty == T_CREATE_BUFFER) begin
                    longint base_w = gle32(body, 4) / 4, size = gle32(body, 8) / 4, used = 0;
                    bit [15:0] h = {body[1], body[0]};
                    batch_flush();                                          // the uploads below must follow the CREATE_BUFFER
                    foreach (img[ak]) if (ak >= base_w && ak < base_w + size && ak - base_w + 1 > used) used = ak - base_w + 1;
                    size = (used + 7) / 8 * 8;
                    for (longint o = 0; o < size; o += 350) begin
                        bq d; longint nw = (size - o > 350) ? 350 : size - o;
                        d.delete();
                        for (longint ww = 0; ww < nw; ww++) begin
                            bit [31:0] v = img.exists(base_w + o + ww) ? img[base_w + o + ww] : 32'h0;
                            d = {d, le32(v)};
                        end
                        send_seq(T_WRITE_BUFFER, {le16(h), le16(0), le32(32'(o * 4)), d}, s);
                    end
                    wait_acked(60000);
                end
            end
        end
        $fclose(fd);
        batch_flush();
        $display("tb_two_boards3d: A draw sent (%0d commands, %0d batches so far), records %0d at %0t", nb, n_batch, da.u_geom.tri_count, $time);
        // ---- +ABORT: a FLUSH B rejects (a handle with the B bit but no image): A streams the frame regardless, B consumes
        //      and drops it (b_render discard mode: every FLUSH owns one frame of the stream), then answers ERR_HANDLE. The
        //      real frame below must then still come out bit-exact, nothing left in flight. (A's own
        //      RENDER_TIMEOUT / ERR_RENDER path covers a dead B.)
        if ($test$plusargs("ABORT")) begin
            reg [31:0] bad_w [0:300]; reg [31:0] sbad;
            for (int i = 0; i < flush_n; i++) bad_w[i] = flush_w[i];
            bad_w[2][15:0] = 16'hCC00;
            send_words(bad_w, flush_n, sbad, 1, ix);
            expect_event(sbad, EV_ERROR, {24'h0, ERR_HANDLE}, 1, 6000, ix);
            t = 0; while (da.u_lport.rs_sent != da.u_lport.rs_consumed && t < 400_000) begin @(posedge clkA); t++; end
            $display("tb_two_boards3d: rejected FLUSH: ERR_HANDLE from B at %0t after its frame was dropped; A beats in flight %0d, B render buffer %0d, B frames %0d",
                     $time, da.u_lport.rs_sent - da.u_lport.rs_consumed, db.u_lport.rsf_cnt, db.rd_frames);
            if (da.u_lport.rs_sent != da.u_lport.rs_consumed) fail("render beats of the rejected frame still in flight");
            n_abort_ev = 1;
        end
        // ---- FLUSH (A forwards it, then streams the frame); then (frame banking) the same draw's
        //      geometry commands again as a SECOND FRAME while the first still streams; B's initial memory (the target and
        //      the depth-stencil image) is written again by the file's MEM_WRITEs, which queue on B behind the first FLUSH,
        //      so the second frame reproduces the golden exactly; then its FLUSH and the FENCE ----
        // ---- B's command processor held as a game's backlog holds it, so B has not reached the FLUSH while A streams its
        //      frame; then, before B is let go:
        //      +LOSTABORT  the link drops (A's link_ok low for 20 cycles). Nothing is given up: no error, no SYNC, both
        //                  frames render (a SYNC would empty B's render buffer and B would wait for ever at that FLUSH:
        //                  TB_FAIL "B hung").
        //      +TOABORT    A gives the frame up after RENDER_TIMEOUT (2 ms here): it ends in-band (FRAME_END with its abort
        //                  bit), B pairs it with the FLUSH: ERR_RENDER from A, the second frame below bit-exact.
        //      +SYNCABORT  A re-syncs the credit (as when B restarts: CREDIT "restarted"): the SYNC empties B's render
        //                  buffer, A drops the frame, B drops the FLUSH older than the SYNC: ERR_RENDER from A, the second
        //                  frame bit-exact.
        lost_abort = $test$plusargs("LOSTABORT"); to_abort = $test$plusargs("TOABORT"); sync_abort = $test$plusargs("SYNCABORT");
        held = lost_abort || to_abort || sync_abort;
        if (held) begin
            @(negedge clkB); force db.u_lport.c_ready = 1'b0; force db.u_cp.c_valid = 1'b0;
            rs0 = da.u_lport.rs_sent; sy0 = db.u_lport.c_sync;
        end
        send_words(flush_w, flush_n, s, held, ix);
        t0 = $time; in_frame = 1;
        if (held) begin
            sflush = s;
            t = 0; while (da.u_lport.rs_sent - rs0 < 64 && t < 400_000) begin @(posedge clkA); t++; end
            if (da.u_lport.rs_sent - rs0 < 64) fail("no render beats sent while B is held");
            if (lost_abort) begin
                @(negedge clkA); force da.u_disp.link_ok = 1'b0; repeat (20) @(negedge clkA); release da.u_disp.link_ok;
                repeat (20_000) @(posedge clkA);
                if (da.u_disp.r_aborted || db.u_lport.c_sync != sy0) fail("the link drop gave the frame up");
            end else if (to_abort) begin
                t = 0; while (!da.u_disp.r_aborted && t < 400_000) begin @(posedge clkA); t++; end
                if (!da.u_disp.r_aborted) fail("no RENDER_TIMEOUT abort while B is held");
                repeat (2000) @(posedge clkA);
                if (db.u_lport.c_sync != sy0) fail("a SYNC after the timeout abort");
            end else begin
                @(negedge clkA); force da.u_lport.y_restart = 1'b1; @(negedge clkA); release da.u_lport.y_restart;
                t = 0; while (db.u_lport.c_sync == sy0 && t < 400_000) begin @(posedge clkA); t++; end
                if (db.u_lport.c_sync == sy0) fail("no SYNC on B after the re-sync");
                t = 0; while (!da.u_disp.r_aborted && t < 400_000) begin @(posedge clkA); t++; end
                if (!da.u_disp.r_aborted) fail("the frame was not dropped with the SYNC");
            end
            $display("tb_two_boards3d: B held at %0t: b_render ms %0d, render buffer %0d beats, %0d beats sent, A aborted %b, SYNCs on B %0d",
                     $time, db.u_render.ms, db.u_lport.rsf_cnt, da.u_lport.rs_sent - rs0, da.u_disp.r_aborted, db.u_lport.c_sync - sy0);
            @(negedge clkB); release db.u_lport.c_ready; release db.u_cp.c_valid;
            lost_armed = 1;
            if (!lost_abort) begin
                expect_event(sflush, EV_ERROR, {24'h0, ERR_RENDER}, 1, 6000, ix);
                n_abort_ev = 1;
            end
        end
        if (!$test$plusargs("ONEFRAME")) begin
            fd = $fopen({"tools/gen/as_", seed, "_cmds.hex"}, "r");
            nb = 0;
            while ($fscanf(fd, "%h", bd) == 1) begin
                if (bd[31:24] == Q_MARK) begin
                    ty = bd[23:16]; blen_ = bd[15:0]; body.delete();
                    for (k = 0; k < (blen_ + 31) / 32; k++) begin
                        void'($fscanf(fd, "%h", bd));
                        for (int j = 0; j < 32 && body.size() < blen_; j++) body.push_back(bd[j*8 +: 8]);
                    end
                    if (ty >= 8'h51 && ty <= 8'h56) begin send_cmd(ty, body, 0, s, ix); nb++; end   // the geometry commands only
                end
            end
            $fclose(fd);
            batch_flush();
            $display("tb_two_boards3d: second frame's geometry sent (%0d commands) at %0t; A render busy %b", nb, $time, da.g_rbusy);
            for (int i = 0; i < nmw; i++) begin send_words(mw[i], mw_n[i], s, 0, ix); if (i % 8 == 7) begin batch_flush(); wait_acked(60000); end end
            batch_flush(); wait_acked(60000);
            send_words(flush_w, flush_n, s, 0, ix);
            $display("tb_two_boards3d: B's initial memory written again (%0d MEM_WRITEs) and the second FLUSH sent at %0t", nmw, $time);
        end
        send_words(fence_w, fence_n, sfence, 1, ix);              // +BATCH: in the batch after the FLUSH, its EVENT carries its index
        expect_event(sfence, EV_FENCE, 32'h5EED, 1, 16000, ix);   // two frames of these sizes take < 4 ms: a stall fails fast
        if (use_batch) $display("tb_two_boards3d: FENCE answered under batch seq %0d sub-message %0d (%0d batches sent)", sfence, ix, n_batch);
        in_frame = 0;
        $display("tb_two_boards3d: FENCE back at %0t (%0t after the FLUSH): A quads %0d in the last frame, %0d primitive-band visits in all frames, B quads %0d launched + %0d early-depth rejects, frames %0d, draws %0d",
                 $time, $time - t0, da.u_geom.c_quads, da.g_c_vis, db.rd_quads, db.rd_ezrej, db.rd_frames, db.cp_draws);
        // ---- the target (and the depth-stencil image) in B's DDR, through A ----
        fd = $fopen({"tools/gen/ab_", seed, "_params.hex"}, "r");
        r = $fscanf(fd, "%h %h %h %h %h", pv[0], pv[1], pv[2], pv[3], c16);
        for (int i = 5; i < 24; i++) r = $fscanf(fd, "%h", pv[i]);
        $fclose(fd);
        base = pv[0]; W = pv[2]; H = pv[3]; bpp = (pv[16] == 0) ? 4 : 2;
        $display("tb_two_boards3d: target %0d x %0d format %0d, depth %0d bits, depth-stencil image %0d", W, H, pv[16], pv[17] ? 16 : 24, pv[18]);
        mem_read_b(base * 4, H * pv[1] * 32, 32'h3D);
        fd = $fopen({"tools/gen/ab_", seed, "_expect.hex"}, "r"); bad = 0;
        for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) begin
            reg [31:0] got; int off;
            r = $fscanf(fd, "%h", e);
            off = y * pv[1] * 32 + x * bpp;
            got = 0; for (int q = 0; q < bpp; q++) got[q*8 +: 8] = rd_bytes[32'h3D].exists(off + q) ? rd_bytes[32'h3D][off + q] : 8'hXX;
            if (got !== e) begin bad++; if (bad < 10 || (bad < 400 && x == 0) || (bad % 50 == 0)) $display("  pixel (%0d, %0d): %h, expected %h (differing pixel %0d)", x, y, got, e, bad); end
        end
        $fclose(fd);
        if (bad) fail($sformatf("%0d pixels differ", bad)); else $display("tb_two_boards3d: %0d pixels bit-exact", W * H);
        if (pv[18]) begin
            mem_read_b(longint'(pv[19]) * 4, H * pv[20] * 32, 32'h3E);
            fd = $fopen({"tools/gen/ab_", seed, "_expect_ds.hex"}, "r"); bad = 0;
            for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) begin
                reg [31:0] got; int off;
                r = $fscanf(fd, "%h", e);
                off = y * pv[20] * 32 + x * 4;
                got = 0; for (int q = 0; q < 4; q++) got[q*8 +: 8] = rd_bytes[32'h3E].exists(off + q) ? rd_bytes[32'h3E][off + q] : 8'hXX;
                if (got !== e) begin bad++; if (bad < 10) $display("  depth-stencil (%0d, %0d): %h, expected %h", x, y, got, e); end
            end
            $fclose(fd);
            if (bad) fail($sformatf("%0d depth-stencil words differ", bad)); else $display("tb_two_boards3d: depth-stencil image bit-exact");
        end
        // ---- final status ----
        send_raw(msg(T_STATUS_REQ, 0, {}));
        begin int n0 = n_status; t = 0; while (n_status == n0 && t < 100000) begin @(posedge clkA); t++; end end
        $display("tb_two_boards3d final: cmds %0d errs %0d fwd %0d bev %0d | quads %0d rstream pkts %0d in flight %0d | resends %0d migA %0d migB %0d",
                 st_w[28], st_w[29], st_w[43], st_w[44], st_w[86], st_w[87][15:0], st_w[87][31:16], n_resend, a_mig_err, b_mig_err);
        if (n_err_ev != n_abort_ev) fail($sformatf("%0d error events (%0d expected)", n_err_ev, n_abort_ev));
        if (st_w[87][31:16] != 0) fail("render beats still in flight");
        if (st_w[87][15:0] == 0) fail("no RSTREAM packet sent");
        errs += a_mig_err + b_mig_err;
        if (errs == 0) $display("TB_PASS tb_two_boards3d %s", seed); else $display("TB_FAIL tb_two_boards3d %s errors=%0d", seed, errs);
        $finish;
    end
    initial begin #400ms; $display("TB_FAIL tb_two_boards3d TIMEOUT: dispatcher st %0d g_busy %b render st %0d beats sent %0d consumed %0d B quads %0d",
                                    da.u_disp.st, da.g_busy, da.u_geom.u_rn.st, da.u_lport.rs_sent, da.u_lport.rs_consumed, db.rd_quads); $finish; end
endmodule
`default_nettype wire
