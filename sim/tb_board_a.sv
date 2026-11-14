`timescale 1ns/1ps
// tb_board_a — board_a_core (RGMII PHY model at 1000 Mbit/s, MIG app model) driven by a host model that
// speaks the fb protocol: Go-Back-N sender with resend on NAK / FULL / timeout, reply decoder.
// The ingress ring is shrunk to 64 bursts (2 KB) so multi-datagram traffic hits the FULL path.
// Checks: HELLO -> ACK; STATUS; REG scratch/build; PHY_REG; CREATE/WRITE/READ_BUFFER byte-exact over
// several READ_DATA chunks; raw MEM_WRITE/MEM_READ; MEMTEST; FENCE; error EVENTs (bad handle, heap,
// range, board B, unknown type); a forced sequence gap (NAK + resend); a duplicate; the heartbeat ACK and the
// host's resend rule on it (a lost datagram whose NAK is lost too: resent within ~3 ms, not the 10 ms stall timer);
// BATCH datagrams (FENCE / READ_BUFFER inside, an error mid-batch ends it, a malformed sub-header); 40 back-to-back
// 1400-byte WRITE_BUFFERs through the small ring with readback; the 3D command stream as batches. Prints TB_PASS / TB_FAIL.
`default_nettype none
module tb_board_a;
    import net_pkg::*;
    import fb_pkg::*;
    reg clk = 0, clk200 = 0, rst = 1, calib = 0;
    always #5   clk = ~clk;
    always #2.5 clk200 = ~clk200;

    localparam [47:0] BMAC = 48'h00_0a_35_01_02_03, HMAC = 48'h02_00_00_00_00_01;
    localparam [31:0] BIP  = 32'hC0A8010A, HIP = 32'hC0A80170;
    localparam [15:0] HPORT = 16'd40000;

    // ---------------- DUT + models ----------------
    reg         giga = 1; reg [15:0] physr = 16'h0000;
    wire rxc, rx_ctl, txc, tx_ctl, phy_rstn, mdc, mdio_o, mdio_oe; wire [3:0] rxd, txd;
    tri1 mdio;
    assign mdio = mdio_oe ? mdio_o : 1'bz;
    rgmii_phy_model u_phy (.giga(giga), .rgmii_rxc(rxc), .rgmii_rx_ctl(rx_ctl), .rgmii_rxd(rxd),
        .rgmii_txc(txc), .rgmii_tx_ctl(tx_ctl), .rgmii_txd(txd), .mdc(mdc), .mdio(mdio), .physr(physr));

    wire        app_en, app_wdf_wren, app_wdf_end, app_rdy, app_wdf_rdy, app_rd_data_valid;
    wire [2:0]  app_cmd; wire [28:0] app_addr; wire [255:0] app_wdf_data, app_rd_data; wire [31:0] app_wdf_mask;
    int mig_err; longint mig_nwr, mig_nrd;
    mig_app_model #(.RD_LAT(24), .RDY_PCT(80), .WDF_PCT(85), .SEED(3)) u_mig (
        .ui_clk(clk), .rst(rst), .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr),
        .app_wdf_data(app_wdf_data), .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren),
        .app_wdf_end(app_wdf_end), .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy),
        .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid), .errors(mig_err), .n_wr(mig_nwr), .n_rd(mig_nrd));

    wire [1:0] led;
    tri0 [31:0] jm1, jm2;                              // board link headers: no board B (pull-downs)
    localparam int TAP0 = 3;                   // non-zero power-up tap: IDELAYE2 VAR_LOAD starts at 0 unless loaded
    board_a_core #(.BUILD_ID(32'h0000_0A01), .IDELAY_DEFAULT(TAP0), .RING_LOG2(6), .MDIO_RST_MS(1), .MDIO_WAIT_MS(1), .MDIO_POLL_US(100),
                   .STATUS_PERIOD(300_000), .TRIAL_CYCLES(50_000), .PHY_TRIAL_CYCLES(600_000),
                   .SNAP_CYCLES(2000), .JTAG_STATUS(1'b0), .B_WAIT(5000)) dut (   // no board B: give up after 50 us
        .clk_sys(clk), .rst_sys(rst), .calib_done(calib), .clk_idelay(clk200),
        .rgmii_rxc(rxc), .rgmii_rx_ctl(rx_ctl), .rgmii_rxd(rxd), .rgmii_txc(txc), .rgmii_tx_ctl(tx_ctl), .rgmii_txd(txd),
        .phy_rstn(phy_rstn), .mdc(mdc), .mdio_o(mdio_o), .mdio_oe(mdio_oe), .mdio_i(mdio),
        .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr), .app_wdf_data(app_wdf_data), .app_wdf_mask(app_wdf_mask),
        .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end), .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy),
        .app_rd_data_valid(app_rd_data_valid), .app_rd_data(app_rd_data), .jm1(jm1), .jm2(jm2), .led(led));

    int errs = 0;
    task automatic fail(input string m); errs++; $display("FAIL: %s", m); endtask

    // ---------------- byte helpers ----------------
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
    int unsigned rs = 777;
    function automatic bq rnd(input int n); bq q; for (int i = 0; i < n; i++) begin rs = rs * 1103515245 + 12345; q.push_back(rs >> 16); end return q; endfunction

    // ---------------- reply decoder ----------------
    int        n_ack = 0, n_status = 0, n_nak = 0;
    bit        ev_trace = 0;                       // print every EVENT arrival (the deferred-abort test)
    bit [31:0] ack_exp, ack_flags, ack_free;
    bit [31:0] st_w [0:STATUS_WORDS-1];
    bit [31:0] ev_seq [$], ev_code [$], ev_a0 [$], ev_a1 [$], ev_a2 [$];
    bit [31:0] rdd_tag [$], rdd_off [$]; int rdd_len [$]; byte unsigned rdd_bytes [$];
    bit [31:0] lr_w2 [$], lr_w3 [$]; bit [7:0] lr_t [$];
    event      ack_ev;
    always begin
        bq f, pl;
        @(posedge clk);
        while (u_phy.txq.size() > 0) begin
            f = u_phy.txq.pop_front();
            f = f[8:f.size() - 5];                     // strip preamble/SFD and FCS (checked in tb_eth)
            if (gbe(f, 12, 2) != 16'h0800 || f[23] != 8'd17) continue;
            if (gbe(f, 36, 2) != HPORT || gbe(f, 34, 2) != 16'd5000) fail("reply UDP ports");
            pl = f[42:42 + gbe(f, 38, 2) - 9];
            if (gle32(pl, 0) != {R_ACK, FB_VERSION, FB_MAGIC} && pl[0] != 8'h46) begin fail("reply magic"); continue; end
            case (pl[3])
                R_ACK: begin ack_exp = gle32(pl, 8); ack_flags = gle32(pl, 12); ack_free = gle32(pl, 16); n_ack++;
                             if (ack_flags[ACKF_NAK]) n_nak++; -> ack_ev; end
                R_STATUS: begin for (int i = 0; i < STATUS_WORDS; i++) st_w[i] = gle32(pl, 8 + 4*i); n_status++; end
                R_EVENT: begin ev_seq.push_back(gle32(pl, 4)); ev_code.push_back(gle32(pl, 8)); ev_a0.push_back(gle32(pl, 12));
                               ev_a1.push_back(gle32(pl, 16)); ev_a2.push_back(gle32(pl, 20));
                               if (ev_trace) $display("TRACE event @%0t seq %0d code %h a0 %h a1 %h a2 %h", $time, gle32(pl, 4), gle32(pl, 8), gle32(pl, 12), gle32(pl, 16), gle32(pl, 20)); end
                R_READ_DATA: begin rdd_tag.push_back(gle32(pl, 8)); rdd_off.push_back(gle32(pl, 12)); rdd_len.push_back(pl.size() - 16);
                                   for (int i = 16; i < pl.size(); i++) rdd_bytes.push_back(pl[i]); end
                R_PHY_REG, R_REG: begin lr_t.push_back(pl[3]); lr_w2.push_back(gle32(pl, 8)); lr_w3.push_back(gle32(pl, 12)); end
                default: fail($sformatf("unknown reply type %h", pl[3]));
            endcase
        end
    end

    // ---------------- Go-Back-N sender ----------------
    bit trace = 0, in3d = 0;                       // +TRACE (see the diagnostics below); in3d: the 3D section runs
    bit [31:0] next_seq = 1;
    bq         sent_q [$];                         // datagrams not yet acknowledged (seq order)
    bit [31:0] sent_seq [$];
    int        n_resend = 0;
    localparam int WINDOW = 8;
    task automatic send_raw(input bq m);
        if (trace && in3d) $display("TRACE send @%0t type %h seq %0d body %0d bytes", $time, m[3], {m[7], m[6], m[5], m[4]}, m.size() - 8);
        u_phy.send_frame(udp_frame(m));
    endtask
    longint last_progress = 0, last_resend = 0; int unsigned cyc = 0;
    always @(posedge clk) cyc++;
    task automatic send_seq(input [7:0] t, input bq body, output bit [31:0] seq);
        seq = next_seq; next_seq++;
        while (sent_q.size() >= WINDOW) @(posedge clk);
        if (sent_q.size() == 0) last_progress = cyc;    // as fbproto: the stall clock starts with the window's first datagram
        begin bq m = msg(t, seq, body); sent_q.push_back(m); sent_seq.push_back(seq); send_raw(m); end
    endtask
    // BATCH builder (fb_pkg T_BATCH): sub-messages {u8 type, u8 0, u16 bytes, body padded to a word} under one seq
    bq  batch_q; int batch_cnt = 0;
    task automatic batch_add(input [7:0] t, input bq body);
        batch_q = {batch_q, t, 8'h00, le16(16'(body.size())), body};
        while (batch_q.size() % 4) batch_q.push_back(8'h00);
        batch_cnt++;
    endtask
    task automatic batch_send(output bit [31:0] seq);
        send_seq(T_BATCH, batch_q, seq); batch_q.delete(); batch_cnt = 0;
    endtask
    // ACK processing + resend (the host rules of fbproto.Board): NAK / FULL -> resend; an ACK that expects the oldest
    // unacknowledged datagram after 2 ms without progress (and 4 ms after the last resend) -> resend (the heartbeat
    // ACK makes this fire ~1 ms after a loss whose NAK was lost); the stall timer is 10 ms
    bit drop_naks = 0;                             // test: the host "loses" every NAK
    initial forever begin
        @(ack_ev);
        while (sent_seq.size() > 0 && $signed(sent_seq[0] - ack_exp) < 0) begin
            void'(sent_q.pop_front()); void'(sent_seq.pop_front()); last_progress = cyc;
        end
        if ((ack_flags[ACKF_NAK] || ack_flags[ACKF_FULL]) && sent_q.size() > 0 && !drop_naks) begin
            if (ack_flags[ACKF_FULL]) repeat (2000) @(posedge clk);
            resend_all();
        end else if (sent_q.size() > 0 && ack_exp == sent_seq[0] && cyc - last_progress > 200_000 && cyc - last_resend > 400_000)
            resend_all();
    end
    task automatic resend_all();
        int n = sent_q.size();
        last_resend = cyc;
        for (int i = 0; i < n && i < sent_q.size(); i++) begin send_raw(sent_q[i]); n_resend++; end
    endtask
    initial forever begin                           // stall-timer resend (10 ms, as fbproto)
        repeat (1000) @(posedge clk);
        if (sent_q.size() > 0 && cyc - last_progress > 1_000_000) begin last_progress = cyc; resend_all(); end
    end
    task automatic wait_acked(input int tmo_us = 5000);
        int t = 0;
        while (sent_q.size() > 0 && t < tmo_us * 100) begin @(posedge clk); t++; end
        if (sent_q.size() > 0) fail($sformatf("datagrams not acknowledged: %0d (exp %0d)", sent_q.size(), ack_exp));
    endtask
    // the EVENT's code word = {16'h0, sub-message index (BATCH), code}
    task automatic expect_event(input [31:0] seq, input [31:0] code, input [31:0] a0, input bit chk_a0, input int tmo_us = 3000,
                                input [7:0] idx = 8'd0);
        int t = 0;
        while (ev_seq.size() == 0 && t < tmo_us * 100) begin @(posedge clk); t++; end
        if (ev_seq.size() == 0) begin fail($sformatf("no EVENT for seq %0d", seq)); return; end
        if (ev_seq[0] != seq || ev_code[0] != {16'h0, idx, code[7:0]} || (chk_a0 && ev_a0[0] != a0))
            fail($sformatf("EVENT seq %0d code %h a0 %h a1 %h (exp seq %0d code %h idx %0d a0 %h)", ev_seq[0], ev_code[0], ev_a0[0], ev_a1[0], seq, code, idx, a0));
        void'(ev_seq.pop_front()); void'(ev_code.pop_front()); void'(ev_a0.pop_front()); void'(ev_a1.pop_front()); void'(ev_a2.pop_front());
    endtask

    // actual IDELAYE2 tap of every RGMII input (UNISIM model CNTVALUEOUT)
    task automatic check_hw_tap(input int exp);
        int t[5];
        t[0] = dut.u_eth.u_rgmii_rx.g_in[0].u_dly.CNTVALUEOUT; t[1] = dut.u_eth.u_rgmii_rx.g_in[1].u_dly.CNTVALUEOUT;
        t[2] = dut.u_eth.u_rgmii_rx.g_in[2].u_dly.CNTVALUEOUT; t[3] = dut.u_eth.u_rgmii_rx.g_in[3].u_dly.CNTVALUEOUT;
        t[4] = dut.u_eth.u_rgmii_rx.g_in[4].u_dly.CNTVALUEOUT;
        for (int i = 0; i < 5; i++) if (t[i] != exp) begin fail($sformatf("IDELAY %0d hardware tap %0d, expected %0d", i, t[i], exp)); return; end
    endtask

    // ---------------- reference memory ----------------
    byte unsigned refm [longint];
    function automatic byte unsigned refb(longint a); return refm.exists(a) ? refm[a] : 8'h00; endfunction
    task automatic read_check(input [7:0] t, input bq hdr_fields, input longint ref_base, input int len, input [31:0] tag);
        bit [31:0] s;
        send_seq(t, {hdr_fields, le32(len), le32(tag)}, s);
        read_collect(ref_base, len, tag);
    endtask
    task automatic read_collect(input longint ref_base, input int len, input [31:0] tag);   // the READ_DATA of a sent read
        int t2 = 0, got = 0; bit ok = 1;
        while (got < len && t2 < 400000) begin
            @(posedge clk); t2++;
            while (rdd_tag.size() > 0) begin
                int n = rdd_len.pop_front(); bit [31:0] off = rdd_off.pop_front(); bit [31:0] tg = rdd_tag.pop_front();
                if (tg != tag || off != got) begin fail($sformatf("READ_DATA tag %h off %0d (exp %h %0d)", tg, off, tag, got)); ok = 0; end
                for (int i = 0; i < n; i++) begin
                    byte unsigned b = rdd_bytes.pop_front();
                    if (b != refb(ref_base + got + i)) begin
                        if (ok) $display("  mismatch at +%0d: %h exp %h", got + i, b, refb(ref_base + got + i));
                        ok = 0;
                    end
                end
                got += n;
            end
        end
        if (got != len) fail($sformatf("READ_DATA got %0d of %0d bytes", got, len));
        else if (!ok) fail("READ_DATA content");
        else $display("INFO_TB read %0d bytes ok (tag %h)", len, tag);
    endtask

    // ---------------- diagnostics: the dispatcher / geometry / ring state (TIMEOUT, and every 1 ms of the 3D section with +MON) ----------------
    bit only3d = 0, mon = 0, nobatch = 0;
    task automatic diag(input string why);
        $display("DIAG %s @%0t: dispatcher st %0d g_busy %b geom tris %0d | feed st %0d i %0d | asm st %0d | setup st %0d | binner busy %b clearing %b | loader st %0d | ring used %0d q_valid %b | sent_q %0d next_seq %0d ack_exp %0d flags %h resends %0d acks %0d | cmds %0d errs %0d last %h",
                 why, $time, dut.u_disp.st, dut.g_busy, dut.u_geom.tri_count, dut.u_geom.u_vtx.u_feed.ist, dut.u_geom.u_vtx.u_feed.i, dut.u_geom.u_vtx.u_asm.st,
                 dut.u_geom.u_set.st, dut.u_geom.b_busy, dut.u_geom.clearing, dut.u_geom.ld_st, dut.q_used, dut.qo_v, sent_q.size(), next_seq, ack_exp, ack_flags,
                 n_resend, n_ack, dut.c_cmds, dut.c_errs, dut.last_err);
        $display("DIAG proto: ps %0d exp_seq %0d acc %0d full %0d dup %0d gap %0d bad %0d nak_t %0d unacked %0d pf_v %b rem %0d | xfer xs %0d busy %b | sin_full %b in_ready %b",
                 dut.u_proto.ps, dut.u_proto.exp_seq, dut.u_proto.c_acc, dut.u_proto.c_full, dut.u_proto.c_dup, dut.u_proto.c_gap, dut.u_proto.c_bad,
                 dut.u_proto.nak_t, dut.u_proto.unacked, dut.pf_v, dut.u_proto.rem, dut.u_xfer.xs, dut.x_busy, dut.u_q.sin_full, dut.qi_rdy);
    endtask
    // +TRACE: every datagram sent and every ACK received during the 3D section
    initial trace = $test$plusargs("TRACE");
    always @(ack_ev) if (trace && in3d) $display("TRACE ack @%0t exp %0d flags %h free %0d", $time, ack_exp, ack_flags, ack_free);
    initial begin
        only3d = $test$plusargs("ONLY3D"); mon = $test$plusargs("MON"); nobatch = $test$plusargs("NOBATCH");
        forever begin #1ms; if (mon && in3d) diag("mon"); end
    end

    // ---------------- test ----------------
    localparam longint HEAP = 64'h1800_0000;         // A's host heap starts at 384 MB (fb_dispatch_a HEAP_LO)
    initial begin
        bit [31:0] s;
        repeat (20) @(posedge clk); rst <= 0;
        repeat (200) @(posedge clk); calib <= 1;
        wait (dut.u_eth.mdio_init_done);
        physr <= 16'h002C;                         // link up, 1000, full duplex
        wait (dut.u_eth.link_up); repeat (2000) @(posedge clk);

        // the power-up tap must be loaded into the delay lines (VAR_LOAD ignores IDELAY_VALUE)
        check_hw_tap(TAP0);
        // HELLO (new session) -> ACK expected 1
        send_raw(msg(T_HELLO, 0, le32(1)));
        @(ack_ev); if (ack_exp != 1) fail($sformatf("HELLO ACK exp %0d", ack_exp));
        last_progress = cyc;                            // the timeout resend counts from here
        if (!only3d) begin                              // +ONLY3D: straight to the 3D section (debugging)
        // STATUS
        send_raw(msg(T_STATUS_REQ, 0, {}));
        begin int t = 0; while (n_status == 0 && t < 100000) begin @(posedge clk); t++; end end
        if (n_status == 0) fail("no STATUS");
        else if (st_w[0] != STATUS_LAYOUT || st_w[1] != 32'h0A01 || !st_w[3][0] || !st_w[3][1] || st_w[3][3:2] != 2'b10)
            fail($sformatf("STATUS words %h %h flags %h", st_w[0], st_w[1], st_w[3]));
        else $display("INFO_TB STATUS ok: flags %h uptime %0d ms", st_w[3], st_w[2]);
        // REG scratch write/read, build id
        send_raw(msg(T_REG, 0, {8'd1, 8'd0, le16(REG_SCRATCH), le32(32'h1234_5678)}));
        send_raw(msg(T_REG, 0, {8'd0, 8'd0, le16(REG_SCRATCH), le32(0)}));
        send_raw(msg(T_REG, 0, {8'd0, 8'd0, le16(REG_BUILD), le32(0)}));
        repeat (20000) @(posedge clk);
        if (lr_t.size() != 3 || lr_w3[1] != 32'h1234_5678 || lr_w3[2] != 32'h0A01) fail($sformatf("REG replies %0d", lr_t.size()));
        else $display("INFO_TB REG ok");
        lr_t.delete(); lr_w2.delete(); lr_w3.delete();
        // REG_LINKCTL: power-up value 1 (fill words on), write 0 reaches link_core's fill input, write 1 restores it
        send_raw(msg(T_REG, 0, {8'd0, 8'd0, le16(REG_LINKCTL), le32(0)}));
        send_raw(msg(T_REG, 0, {8'd1, 8'd0, le16(REG_LINKCTL), le32(32'h0)}));
        repeat (20000) @(posedge clk);
        if (dut.u_link.fill_en !== 1'b0) fail("LINKCTL 0 did not clear link_core fill_en");
        send_raw(msg(T_REG, 0, {8'd1, 8'd0, le16(REG_LINKCTL), le32(32'h1)}));
        send_raw(msg(T_REG, 0, {8'd0, 8'd0, le16(REG_LINKCTL), le32(0)}));
        repeat (20000) @(posedge clk);
        if (lr_t.size() != 4 || lr_w3[0] != 32'h1 || lr_w3[1] != 32'h0 || lr_w3[2] != 32'h1 || lr_w3[3] != 32'h1 ||
            dut.u_link.fill_en !== 1'b1)
            fail($sformatf("REG_LINKCTL replies %0d", lr_t.size()));
        else $display("INFO_TB REG_LINKCTL ok (power-up 1, 0 / 1 reach link_core)");
        lr_t.delete(); lr_w2.delete(); lr_w3.delete();
        // IDELAY trial: reverts without traffic, sticks when other traffic arrives
        send_raw(msg(T_REG, 0, {8'd1, 8'd0, le16(REG_IDELAY), le32(32'h107)}));     // trial tap 7
        repeat (3000) @(posedge clk);
        send_raw(msg(T_REG, 0, {8'd0, 8'd0, le16(REG_IDELAY), le32(0)}));
        repeat (3000) @(posedge clk);
        if (dut.u_proto.idelay_tap != 7 || lr_w3.size() < 2 || lr_w3[1] != 32'h107) fail($sformatf("trial tap applied: tap %0d", dut.u_proto.idelay_tap));
        check_hw_tap(7);
        repeat (60000) @(posedge clk);                                                  // > TRIAL_CYCLES, no traffic
        if (dut.u_proto.idelay_tap != TAP0) fail($sformatf("trial tap not reverted: %0d", dut.u_proto.idelay_tap));
        check_hw_tap(TAP0);
        send_raw(msg(T_REG, 0, {8'd1, 8'd0, le16(REG_IDELAY), le32(32'h109)}));     // trial tap 9
        repeat (3000) @(posedge clk);
        send_raw(msg(T_STATUS_REQ, 0, {}));                                             // traffic -> commit
        repeat (60000) @(posedge clk);
        if (dut.u_proto.idelay_tap != 9 || dut.u_proto.trial_t != 0) fail($sformatf("trial not committed: tap %0d", dut.u_proto.idelay_tap));
        check_hw_tap(9);
        send_raw(msg(T_REG, 0, {8'd1, 8'd0, le16(REG_IDELAY), le32(32'h0)}));       // plain write tap 0
        repeat (3000) @(posedge clk);
        if (dut.u_proto.idelay_tap != 0 || dut.u_proto.trial_t != 0) fail("plain IDELAY write");
        else $display("INFO_TB IDELAY trial revert/commit ok");
        // PHY option trial: RXDLY on + tap 5 with a PHY re-init; reverts (re-init with RXDLY off) without
        // traffic, sticks when traffic arrives after the link is back
        begin
            int nw0 = u_phy.wlog.size(), nw1;
            send_raw(msg(T_REG, 0, {8'd1, 8'd0, le16(REG_PHYOPT), le32(32'h0005_0303)}));
            repeat (5000) @(posedge clk);
            if (!dut.u_proto.rxdly || dut.u_proto.idelay_tap != 5) fail("PHY trial not applied");
            check_hw_tap(5);
            wait (!dut.u_eth.link_up); wait (dut.u_eth.link_up);
            nw1 = u_phy.wlog.size();
            if (nw1 < nw0 + 8 || u_phy.wlog[nw0 + 3][15:0] != 16'h0019) fail("PHY trial: re-init without RXDLY write");
            repeat (650000) @(posedge clk);                                       // > PHY_TRIAL_CYCLES, no traffic
            if (dut.u_proto.rxdly || dut.u_proto.idelay_tap != 0) fail("PHY trial not reverted");
            check_hw_tap(0);
            wait (dut.u_eth.link_up); repeat (2000) @(posedge clk);
            begin   // the last write to page 0xd08 reg 0x15 (RX delay) must be the RXDLY-off value again
                int last = -1;
                for (int i = nw1; i < u_phy.wlog.size(); i++) if (u_phy.wlog[i][36:16] == {16'h0d08, 5'h15}) last = i;
                if (last < 0 || u_phy.wlog[last][15:0] != 16'h0011) fail("PHY trial revert: no re-init with RXDLY off");
            end
            // again, now with traffic once the link is up -> commit
            send_raw(msg(T_REG, 0, {8'd1, 8'd0, le16(REG_PHYOPT), le32(32'h0005_0303)}));
            repeat (5000) @(posedge clk);
            wait (dut.u_eth.link_up); repeat (2000) @(posedge clk);
            send_raw(msg(T_STATUS_REQ, 0, {}));
            repeat (650000) @(posedge clk);
            if (!dut.u_proto.rxdly || dut.u_proto.idelay_tap != 5 || dut.u_proto.trial_t != 0) fail("PHY trial not committed");
            // plain write back to RXDLY off, tap 0, then re-init
            send_raw(msg(T_REG, 0, {8'd1, 8'd0, le16(REG_PHYOPT), le32(32'h0000_0202)}));
            repeat (3000) @(posedge clk);
            send_raw(msg(T_REG, 0, {8'd1, 8'd0, le16(REG_REINIT), le32(1)}));
            repeat (3000) @(posedge clk);
            wait (dut.u_eth.link_up); repeat (2000) @(posedge clk);
            if (dut.u_proto.rxdly || dut.u_proto.idelay_tap != 0) fail("plain PHYOPT write");
            else $display("INFO_TB PHY option trial revert/commit ok");
        end
        lr_t.delete(); lr_w2.delete(); lr_w3.delete();
        // PHY_REG read page 0 reg 2
        send_raw(msg(T_PHY_REG, 0, {8'd0, 8'd2, le16(16'h0000), le16(16'h0), le16(16'h0)}));
        begin int t = 0; while (lr_t.size() == 0 && t < 200000) begin @(posedge clk); t++; end end
        if (lr_t.size() == 0 || lr_t[0] != R_PHY_REG || lr_w3[0][15:0] != 16'h001C) fail("PHY_REG reply");
        else $display("INFO_TB PHY_REG ok");

        // buffers
        send_seq(T_CREATE_BUFFER, {le16(16'd5), le16(16'h1), le32(HEAP), le32(32'h10000)}, s);
        begin bq d = rnd(1000); send_seq(T_WRITE_BUFFER, {le16(16'd5), le16(0), le32(13), d}, s);
              for (int i = 0; i < 1000; i++) refm[HEAP + 13 + i] = d[i]; end
        begin bq d = rnd(1400); send_seq(T_WRITE_BUFFER, {le16(16'd5), le16(0), le32(3000), d}, s);
              for (int i = 0; i < 1400; i++) refm[HEAP + 3000 + i] = d[i]; end
        wait_acked();
        read_check(T_READ_BUFFER, {le16(16'd5), le16(0), le32(0)}, HEAP, 4401, 32'h77);
        // raw memory, unaligned
        begin bq d = rnd(77); send_seq(T_MEM_WRITE, {8'd0, 8'd0, 8'd0, 8'd0, le32(32'h0900_0003), d}, s);
              for (int i = 0; i < 77; i++) refm[64'h0900_0003 + i] = d[i]; end
        read_check(T_MEM_READ, {8'd0, 8'd0, 8'd0, 8'd0, le32(32'h0900_0001)}, 64'h0900_0001, 81, 32'h99);
        // memtest
        send_seq(T_MEMTEST, {8'd0, 8'd0, 8'd0, 8'd0, le32(32'h1000_0000), le32(32'h4000), le32(32'hC0FFEE), le32(3)}, s);
        expect_event(s, EV_MEMTEST, 0, 1);
        // errors
        send_seq(T_WRITE_BUFFER, {le16(16'd6), le16(0), le32(0), rnd(8)}, s);   expect_event(s, EV_ERROR, ERR_HANDLE, 1);
        send_seq(T_CREATE_BUFFER, {le16(16'd7), le16(0), le32(32'h0100_0000), le32(64)}, s); expect_event(s, EV_ERROR, ERR_HEAP, 1);
        send_seq(T_READ_BUFFER, {le16(16'd5), le16(0), le32(32'hFFF0), le32(32), le32(1)}, s); expect_event(s, EV_ERROR, ERR_RANGE, 1);
        send_seq(T_CREATE_IMAGE, {le16(16'hC001), le16(0), le32(0), le32(0)}, s); expect_event(s, EV_ERROR, ERR_NO_B, 1);
        send_seq(8'h7F, {le32(0)}, s);                                             expect_event(s, EV_ERROR, ERR_TYPE, 1);   // 0x7F: unassigned (0x55 is DRAW)
        // forced gap: skip one sequence number, then send the real one
        begin
            bq m2 = msg(T_FENCE, next_seq + 1, le32(5));
            int nak0 = n_nak;
            send_raw(m2);                               // arrives before seq next_seq -> NAK
            repeat (5000) @(posedge clk);
            if (n_nak == nak0 || ack_exp != next_seq) fail($sformatf("gap: NAKs %0d ACK flags %h exp %0d (next %0d)", n_nak - nak0, ack_flags, ack_exp, next_seq));
            else $display("INFO_TB gap -> NAK ok");
        end
        send_seq(T_FENCE, le32(4), s); expect_event(s, EV_FENCE, 4, 1);
        send_seq(T_FENCE, le32(5), s); expect_event(s, EV_FENCE, 5, 1);
        // duplicate of an old datagram -> dropped, ACK
        send_raw(msg(T_FENCE, s, le32(5)));
        repeat (20000) @(posedge clk);
        if (ev_seq.size() != 0) fail("duplicate executed");
        // ---- heartbeat ACKs: with nothing in flight A still ACKs every ~1 ms ----
        begin
            int n0 = n_ack; repeat (500_000) @(posedge clk);
            if (n_ack - n0 < 4) fail($sformatf("heartbeat: %0d ACKs in 5 ms", n_ack - n0));
            else $display("INFO_TB heartbeat: %0d ACKs in 5 ms", n_ack - n0);
        end
        // ---- a lost datagram whose NAK is lost too: the heartbeat ACK expects the oldest unacknowledged datagram, 2 ms
        //      without progress -> the host resends (fbproto's rule) long before its 10 ms stall timer ----
        begin
            bq d = rnd(200); bit [31:0] s2; longint t_lost; int nr0 = n_resend, nk0 = n_nak;
            drop_naks = 1;
            s = next_seq; next_seq++;
            if (sent_q.size() == 0) last_progress = cyc;
            begin bq m = msg(T_WRITE_BUFFER, s, {le16(16'd5), le16(0), le32(6000), d}); sent_q.push_back(m); sent_seq.push_back(s); end   // never sent
            for (int i = 0; i < 200; i++) refm[HEAP + 6000 + i] = d[i];
            t_lost = cyc;
            send_seq(T_FENCE, le32(6), s2);             // a gap: A NAKs, the bench drops the NAK
            expect_event(s2, EV_FENCE, 6, 1, 20000);
            drop_naks = 0;
            $display("INFO_TB lost datagram + lost NAK: %0d NAK(s) ignored, resent after %0d us, %0d datagrams resent",
                     n_nak - nk0, (last_resend - t_lost) / 100, n_resend - nr0);
            if (last_resend - t_lost > 500_000 || n_resend - nr0 != 2 || n_nak == nk0) fail("heartbeat resend rule");
            read_check(T_READ_BUFFER, {le16(16'd5), le16(0), le32(6000)}, HEAP + 6000, 200, 32'h78);
        end
        // ---- BATCH (fb_pkg T_BATCH): several commands in one datagram, events / READ_DATA by sub-message index / tag ----
        begin
            bq d1 = rnd(500), d2 = rnd(300), d3 = rnd(100), d4 = rnd(16), d5 = rnd(16);
            longint B8 = HEAP + 64'h20000;
            batch_add(T_CREATE_BUFFER, {le16(16'd8), le16(16'h1), le32(32'(B8)), le32(32'h1000)});
            batch_add(T_WRITE_BUFFER, {le16(16'd8), le16(0), le32(0), d1});
            batch_add(T_WRITE_BUFFER, {le16(16'd8), le16(0), le32(500), d2});
            batch_add(T_FENCE, le32(91));
            batch_send(s);
            for (int i = 0; i < 500; i++) refm[B8 + i] = d1[i];
            for (int i = 0; i < 300; i++) refm[B8 + 500 + i] = d2[i];
            expect_event(s, EV_FENCE, 91, 1, 20000, 8'd3);
            read_check(T_READ_BUFFER, {le16(16'd8), le16(0), le32(0)}, B8, 800, 32'h61);
            // a READ_BUFFER inside a batch: its READ_DATA under the batch's seq, found by tag
            batch_add(T_WRITE_BUFFER, {le16(16'd8), le16(0), le32(800), d3});
            batch_add(T_READ_BUFFER, {le16(16'd8), le16(0), le32(0), le32(900), le32(32'h62)});
            batch_send(s);
            for (int i = 0; i < 100; i++) refm[B8 + 800 + i] = d3[i];
            read_collect(B8, 900, 32'h62);
            // an error in the middle ends the batch: the third sub-command must not run (bytes 16..31 keep d1's)
            batch_add(T_WRITE_BUFFER, {le16(16'd8), le16(0), le32(0), d4});
            batch_add(T_WRITE_BUFFER, {le16(16'd9), le16(0), le32(0), d5});     // no such buffer
            batch_add(T_WRITE_BUFFER, {le16(16'd8), le16(0), le32(16), d5});
            batch_send(s);
            for (int i = 0; i < 16; i++) refm[B8 + i] = d4[i];
            expect_event(s, EV_ERROR, ERR_HANDLE, 1, 20000, 8'd1);
            send_seq(T_FENCE, le32(92), s); expect_event(s, EV_FENCE, 92, 1, 20000);
            read_check(T_READ_BUFFER, {le16(16'd8), le16(0), le32(0)}, B8, 64, 32'h63);
            // a sub-header claiming more than the datagram holds: ERR_LEN (a1 = the sub-message's type), the dispatcher recovers
            send_seq(T_BATCH, {T_FENCE, 8'h00, le16(16'd2000), le32(93)}, s);
            expect_event(s, EV_ERROR, ERR_LEN, 1, 20000, 8'd0);
            send_seq(T_FENCE, le32(94), s); expect_event(s, EV_FENCE, 94, 1, 20000);
            $display("INFO_TB BATCH ok (fence / read inside, error mid-batch, malformed sub-header)");
        end
        // ---- a deferred ERR_RENDER (an aborted frame, reported at the next command boundary) while commands are queued:
        //      the queue must not lose the next entry's header in the cycle the error takes (else the
        //      PRESENT after a link-retrain abort is misparsed). No B here, so the monitor's flag is
        //      forced for the one D_HDR cycle after a memtest that holds two FENCEs in the queue ----
        begin
            bit [31:0] smt, s95, s96; int t;
            ev_trace = 1;
            send_seq(T_MEMTEST, {8'd0, 8'd0, 8'd0, 8'd0, le32(32'h1200_0000), le32(32'h0004_0000), le32(32'h2468), le32(3)}, smt);
            send_seq(T_FENCE, le32(95), s95); send_seq(T_FENCE, le32(96), s96);
            // fb_dispatch_a's dst_t: D_HDR = 0 ... D_MTWAIT = 8 (an enum constant is not reachable hierarchically in xsim)
            t = 0; while (dut.u_disp.st != 8 && t < 2_000_000) begin @(posedge clk); t++; end
            if (dut.u_disp.st != 8) fail("memtest never reached D_MTWAIT");
            $display("INFO_TB rr: memtest running (st %0d) at %0t, ring used %0d: forcing the abort flag", dut.u_disp.st, $time, dut.q_used);
            force dut.u_disp.rerr_pend = 1'b1; force dut.u_disp.r_seq = smt; force dut.u_disp.r_idx = 8'd0; force dut.u_disp.rerr_rt = 32'd12345;
            t = 0; while (dut.u_disp.st != 0 && t < 2_000_000) begin @(posedge clk); t++; end   // the memtest's event sent, back at D_HDR
            @(posedge clk);                                                                       // the branch has taken the flag
            // a released variable keeps the forced value until its next assignment: force it back to 0 first (else the
            // branch fires again at the next D_HDR and reports a second abort)
            force dut.u_disp.rerr_pend = 1'b0; @(posedge clk);
            release dut.u_disp.rerr_pend; release dut.u_disp.r_seq; release dut.u_disp.r_idx; release dut.u_disp.rerr_rt;
            $display("INFO_TB rr: released at %0t (st %0d)", $time, dut.u_disp.st);
            expect_event(smt, EV_MEMTEST, 0, 1, 20000);
            expect_event(smt, EV_ERROR, ERR_RENDER, 1, 20000);
            expect_event(s95, EV_FENCE, 95, 1, 20000);
            expect_event(s96, EV_FENCE, 96, 1, 20000);
            ev_trace = 0;
            $display("INFO_TB deferred ERR_RENDER with queued commands ok");
        end
        // 40 back-to-back 1400-byte writes through the 2 KB ring, then readback
        for (int n = 0; n < 40; n++) begin
            bq d = rnd(1400);
            send_seq(T_WRITE_BUFFER, {le16(16'd5), le16(0), le32(8000 + 1400*n), d}, s);
            for (int i = 0; i < 1400; i++) refm[HEAP + 8000 + 1400*n + i] = d[i];
        end
        wait_acked(20000);
        send_seq(T_FENCE, le32(77), s); expect_event(s, EV_FENCE, 77, 1, 20000);
        read_check(T_READ_BUFFER, {le16(16'd5), le16(0), le32(8000)}, HEAP + 8000, 56000, 32'h55);
        // stall the dispatcher with a long memtest while writes stream in -> the 2 KB ring fills (FULL)
        begin
            bit [31:0] smt;
            send_seq(T_MEMTEST, {8'd0, 8'd0, 8'd0, 8'd0, le32(32'h1100_0000), le32(32'h0010_0000), le32(32'h1357), le32(3)}, smt);
            for (int n = 0; n < 12; n++) begin
                bq d = rnd(1400);
                send_seq(T_WRITE_BUFFER, {le16(16'd5), le16(0), le32(1400*n), d}, s);
                for (int i = 0; i < 1400; i++) refm[HEAP + 1400*n + i] = d[i];
            end
            expect_event(smt, EV_MEMTEST, 0, 1, 20000);
            wait_acked(20000);
            send_seq(T_FENCE, le32(78), s); expect_event(s, EV_FENCE, 78, 1, 20000);
            read_check(T_READ_BUFFER, {le16(16'd5), le16(0), le32(0)}, HEAP, 16800, 32'h56);
        end
        end                                             // !only3d
        in3d = 1;
        // ---- 3D on A: the draw of tools/a_setup_ref.py 1 as host commands — the buffers uploaded with
        //      WRITE_BUFFER from the model's DDR image, the command stream (FRAME_BEGIN .. DRAW, FENCE) from as_1_cmds.hex
        //      as BATCH datagrams of up to 1400 body bytes (the CREATE_BUFFERs and uploads plain; the FENCE inside the last
        //      batch; +NOBATCH: one datagram per command), then the triangle records read back with MEM_READ and compared
        //      with as_1_rec.hex
        begin
            int fd, nb, k, n_rec, nw_, n_batch = 0; reg [31:0] bi, w0; reg [255:0] bd; int v_id, v_lo, v_hi, v_n, v_w;
            bit [31:0] img [longint]; longint a; bq body; bit [7:0] ty; int blen_;
            // the DDR image (word address -> word), as the model wrote it
            fd = $fopen("tools/gen/av_1_ddr.hex", "r");
            if (fd == 0) fail("no tools/gen/av_1_ddr.hex (python3 tools/a_setup_ref.py 1)");
            else begin
                while ($fscanf(fd, "%h %h", bi, bd) == 2) for (k = 0; k < 8; k++) img[longint'(bi) * 8 + k] = bd[k*32 +: 32];
                $fclose(fd);
                // the four buffers (handles 1..4 of the command stream): VBO A, VBO B, indices, program — WRITE_BUFFER
                // after their CREATE_BUFFER, so the command stream is replayed in two halves around the uploads
                fd = $fopen("tools/gen/as_1_cmds.hex", "r");
                if (fd == 0) fail("no tools/gen/as_1_cmds.hex");
                else begin
                    nb = 0;
                    while ($fscanf(fd, "%h", bd) == 1) begin
                        if (bd[31:24] == Q_MARK) begin                       // a header burst: type, body bytes
                            ty = bd[23:16]; blen_ = bd[15:0]; body.delete();
                            for (k = 0; k < (blen_ + 31) / 32; k++) begin
                                void'($fscanf(fd, "%h", bd));
                                for (int j = 0; j < 32 && body.size() < blen_; j++) body.push_back(bd[j*8 +: 8]);
                            end
                            if (ty == T_FENCE) begin
                                if (nobatch) begin send_seq(ty, body, s); expect_event(s, EV_FENCE, 32'h5EED, 1, 100000); end
                                else begin
                                    int fi_ = batch_cnt; batch_add(ty, body); batch_send(s);
                                    $display("INFO_TB 3D: fence seq %0d sent (batch sub-message %0d)", s, fi_);
                                    expect_event(s, EV_FENCE, 32'h5EED, 1, 100000, 8'(fi_));
                                end
                            end else if (ty != T_CREATE_BUFFER && !nobatch) begin
                                if (batch_q.size() + 4 + body.size() > 1400) begin batch_send(s); n_batch++; end
                                batch_add(ty, body);
                            end else begin
                                if (batch_cnt != 0) begin batch_send(s); n_batch++; end
                                send_seq(ty, body, s); $display("INFO_TB 3D: cmd %h seq %0d", ty, s);
                                if (ty == T_CREATE_BUFFER) begin                 // upload the buffer's USED words from the image
                                    longint base = gle32(body, 4) / 4, size = gle32(body, 8) / 4, used = 0;
                                    bit [15:0] h = {body[1], body[0]};
                                    foreach (img[ak]) if (ak >= base && ak < base + size && ak - base + 1 > used) used = ak - base + 1;
                                    size = (used + 7) / 8 * 8;
                                    for (longint o = 0; o < size; o += 350) begin
                                        bq d; longint n = (size - o > 350) ? 350 : size - o;
                                        d.delete();                  // static in an initial block: the previous chunk stays otherwise
                                        for (longint w = 0; w < n; w++) begin
                                            bit [31:0] v = img.exists(base + o + w) ? img[base + o + w] : 32'h0;
                                            d = {d, le32(v)};
                                        end
                                        send_seq(T_WRITE_BUFFER, {le16(h), le16(0), le32(32'(o * 4)), d}, s);
                                    end
                                    wait_acked(60000);
                                end
                            end
                            nb++;
                        end
                    end
                    $fclose(fd);
                    $display("INFO_TB 3D: %0d commands sent (%0d batches), geometry records %0d", nb, n_batch, dut.u_geom.tri_count);
                    // the records, word for word, through MEM_READ at 64 MB + id * 512
                    fd = $fopen("tools/gen/as_1_rec.hex", "r"); n_rec = 0;
                    while ($fscanf(fd, "%h %h %h %h", v_id, v_lo, v_hi, v_n) == 4) begin
                        a = 64'h0400_0000 + 64'(v_id) * 512;
                        for (k = 0; k < v_n; k++) begin
                            void'($fscanf(fd, "%h", v_w));
                            for (int j = 0; j < 4; j++) refm[a + 4*k + j] = v_w[8*j +: 8];
                        end
                        read_check(T_MEM_READ, {8'd0, 8'd0, 8'd0, 8'd0, le32(32'(a))}, a, 4 * v_n, 32'h3D00 + v_id);
                        n_rec++;
                    end
                    $fclose(fd);
                    if (dut.u_geom.tri_count != n_rec) fail($sformatf("3D: %0d records made, %0d expected", dut.u_geom.tri_count, n_rec));
                    else $display("INFO_TB 3D draw ok: %0d records bit-exact", n_rec);
                end
            end
        end
        // final status
        send_raw(msg(T_STATUS_REQ, 0, {}));
        begin int n0 = n_status, t = 0; while (n_status == n0 && t < 100000) begin @(posedge clk); t++; end end
        $display("INFO_TB final: exp %0d acc %0d dup %0d gap %0d full %0d bad %0d acks %0d cmds %0d errs %0d resends %0d mig_err %0d",
                 st_w[19], st_w[20], st_w[21], st_w[22], st_w[23], st_w[24], st_w[25], st_w[28], st_w[29], n_resend, mig_err);
        $display("INFO_TB final eth: ok %0d fcs %0d udp %0d other %0d pfdrop %0d tx %0d snap_age %0d sent %0d dropped %0d",
                 st_w[5], st_w[6], st_w[12], st_w[14], st_w[15], st_w[17], st_w[34], st_w[35], st_w[36]);
        if (st_w[5] == 0 || st_w[12] == 0 || st_w[17] == 0) fail("Ethernet counters in STATUS are zero");
        if (!only3d && st_w[29] != 8) fail($sformatf("command error count %0d (exp 8)", st_w[29]));
        if (!only3d && st_w[23] == 0) fail("FULL path never exercised");
        errs += mig_err;
        if (errs == 0) $display("TB_PASS tb_board_a");
        else           $display("TB_FAIL tb_board_a errors=%0d", errs);
        $finish;
    end
    initial begin
        #300ms;
        diag("TIMEOUT");
        $display("TB_FAIL tb_board_a TIMEOUT");
        $finish;
    end
endmodule
`default_nettype wire
