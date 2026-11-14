`timescale 1ns/1ps
// tb_dhcp — eth_stack's DHCP client against rgmii_phy_model and a scripted DHCP server, time scaled: one client
// millisecond = 20 clk_sys cycles, so one second = 200 us of simulation.
//   A (1000M) no server: DISCOVERs (fields, options, UDP checksum, timing), no address before the fallback time,
//     192.168.1.10 after it (ARP, UDP 5000).
//   B (100M, link bounced) OFFERs with a wrong xid / chaddr ignored, OFFER -> REQUEST -> ACK, three ARP probes
//     (fallback address still answered meanwhile), two announcements, then the lease answers ARP / ICMP / UDP and is
//     the replies' source; the fallback address no longer answers.
//   C renewal at T1 (unicast to the server), ACK with T1 / T2 options, silent renewal, rebinding at T2 (broadcast),
//     the lease's end drops the address, DISCOVER, fallback address 10 s later.
//   D (1000M, bounced) unicast OFFER with option overload (file + sname), NAK -> new DISCOVER after 1-2 s,
//     a conflicting ARP reply while probing -> DECLINE, DISCOVER 10 s later, another station probing the address ->
//     DECLINE, then a clean bind.
//   E bounced while bound: the address keeps answering, DISCOVER asks for it (option 50), the same address is bound
//     without probes; bounced again with the server silent: the fallback address after 10 s.
// Prints TB_PASS / TB_FAIL.
`default_nettype none
module tb_dhcp;
    import net_pkg::*;
    reg clk = 0, clk200 = 0, rst = 1;
    always #5   clk = ~clk;
    always #2.5 clk200 = ~clk200;
    localparam real SEC = 200_000.0;                 // ns per client second

    localparam [47:0] BMAC = 48'h00_0a_35_01_02_03, HMAC = 48'h02_00_00_00_00_01, SMAC = 48'h02_00_00_00_00_53,
                      OMAC = 48'h02_00_00_00_00_99;
    localparam [31:0] FB = 32'hC0A8010A, HIP = 32'hC0A80070, SIP = 32'hC0A80001;

    reg         giga = 1;
    reg  [15:0] physr = 16'h0000;
    wire rxc, rx_ctl, txc, tx_ctl, phy_rstn, mdc, mdio_o, mdio_oe; wire [3:0] rxd, txd;
    tri1 mdio;
    assign mdio = mdio_oe ? mdio_o : 1'bz;
    rgmii_phy_model u_phy (.giga(giga), .rgmii_rxc(rxc), .rgmii_rx_ctl(rx_ctl), .rgmii_rxd(rxd),
        .rgmii_txc(txc), .rgmii_tx_ctl(tx_ctl), .rgmii_txd(txd), .mdc(mdc), .mdio(mdio), .physr(physr));

    wire        pf_v; wire [35:0] pf_d;
    reg         rf_wen = 0; reg [35:0] rf_wd; wire [11:0] rf_wcount;
    wire        phy_resp; wire [15:0] phy_rd;
    wire        init_done, link_up, duplex; wire [1:0] speed; wire [15:0] physr_o, bmsr_o; wire [31:0] polls;
    wire [32*14-1:0] cnt; wire [31:0] age;
    wire [31:0] ip; wire ip_valid, ip_leased; wire [3:0] dstate;
    eth_stack #(.MY_MAC(BMAC), .FALLBACK_IP(FB), .DHCP(1'b1), .DHCP_MS_CYCLES(20), .DHCP_FALLBACK_MS(10_000),
                .SNAP_CYCLES(2000), .MDIO_RST_MS(1), .MDIO_WAIT_MS(1), .MDIO_POLL_US(100)) dut (
        .clk_sys(clk), .rst_sys(rst), .clk_idelay(clk200),
        .rgmii_rxc(rxc), .rgmii_rx_ctl(rx_ctl), .rgmii_rxd(rxd), .rgmii_txc(txc), .rgmii_tx_ctl(tx_ctl),
        .rgmii_txd(txd), .phy_rstn(phy_rstn), .mdc(mdc), .mdio_o(mdio_o), .mdio_oe(mdio_oe), .mdio_i(mdio),
        .pf_rvalid(pf_v), .pf_rdata(pf_d), .pf_ren(pf_v),
        .rf_wen(rf_wen), .rf_wdata(rf_wd), .rf_wcount(rf_wcount),
        .dly_ld(1'b0), .dly_val(5'd0), .phyad(5'd1), .rxdly(1'b0), .adv1000(1'b1), .reinit(1'b0),
        .phy_req(1'b0), .phy_req_wr(1'b0), .phy_req_page(16'h0000), .phy_req_reg(5'h02),
        .phy_req_wdata(16'h0), .phy_resp(phy_resp), .phy_resp_rdata(phy_rd),
        .mdio_init_done(init_done), .link_up(link_up), .duplex(duplex), .speed(speed),
        .physr(physr_o), .bmsr(bmsr_o), .mdio_polls(polls), .cnt_snap(cnt), .snap_age(age),
        .ip_addr(ip), .ip_valid(ip_valid), .ip_leased(ip_leased), .dhcp_state(dstate));

    int errs = 0;
    task automatic fail(input string m); errs++; $display("FAIL @%0.2f s: %s", $realtime / SEC, m); endtask
    function automatic real now_s(); return $realtime / SEC; endfunction

    // ---------------- frame builders / field access ----------------
    typedef byte unsigned bq [$];
    function automatic bq be16(input [15:0] v); bq q; q.push_back(v[15:8]); q.push_back(v[7:0]); return q; endfunction
    function automatic bq be32(input [31:0] v); return {be16(v[31:16]), be16(v[15:0])}; endfunction
    function automatic bq mac(input [47:0] m); bq q; for (int i = 5; i >= 0; i--) q.push_back(m[8*i +: 8]); return q; endfunction
    function automatic [15:0] csum(input bq d);
        bit [31:0] s = 0;
        for (int i = 0; i < d.size(); i += 2) s += {d[i], (i + 1 < d.size()) ? d[i+1] : 8'h00};
        while (s >> 16) s = (s & 16'hFFFF) + (s >> 16);
        return ~s[15:0];
    endfunction
    function automatic bq pad60(input bq f); while (f.size() < 60) f.push_back(8'h00); return f; endfunction
    function automatic bq ipv4(input [31:0] sip, input [31:0] dip, input [7:0] proto, input bq pl);
        bq h; bit [15:0] c;
        h = {8'h45, 8'h00, be16(20 + pl.size()), be16(16'h1234), be16(16'h4000), 8'd64, proto, be16(16'h0000), be32(sip), be32(dip)};
        c = csum(h);
        h[10] = c[15:8]; h[11] = c[7:0];
        return {h, pl};
    endfunction
    function automatic [63:0] gbe(input bq q, input int i, input int n);
        bit [63:0] v = 0;
        for (int k = 0; k < n; k++) v = (v << 8) | q[i + k];
        return v;
    endfunction
    function automatic string ipstr(input [31:0] a);
        return $sformatf("%0d.%0d.%0d.%0d", a[31:24], a[23:16], a[15:8], a[7:0]);
    endfunction

    // ---------------- board TX frames, sorted ----------------
    bq   dq [$]; real dq_t [$];                      // DHCP datagrams (UDP 68 -> 67)
    bq   aq [$]; real aq_t [$];                      // ARP requests (probes, announcements)
    bq   rq [$];                                     // ARP replies
    bq   oq [$];                                     // anything else (ICMP replies, reply-FIFO datagrams)
    always begin
        bq f, b;
        @(posedge clk);
        while (u_phy.txq.size() > 0) begin
            f = u_phy.txq.pop_front();
            b = f[8:f.size() - 5];
            begin
                bit [31:0] got = 0;
                for (int k = 0; k < 4; k++) got[8*k +: 8] = f[f.size() - 4 + k];
                if (got != u_phy.crc32(b)) fail("TX FCS");
            end
            if (gbe(b, 12, 2) == 16'h0806) begin
                if (gbe(b, 20, 2) == 16'd2) rq.push_back(b);
                else begin aq.push_back(b); aq_t.push_back(now_s()); end
            end else if (gbe(b, 12, 2) == 16'h0800 && b[23] == 8'd17 && gbe(b, 34, 2) == 16'd68) begin
                dq.push_back(b); dq_t.push_back(now_s());
            end else oq.push_back(b);
        end
    end

    // ---------------- payload FIFO: count the datagrams delivered ----------------
    int n_dg = 0;
    always @(posedge clk) if (pf_v && (pf_d[35:34] == K_LAST || (pf_d[35:34] == K_D0 && pf_d[15:0] == 0))) n_dg++;

    // ---------------- DHCP message decode + checks ----------------
    // (no strings or queue slices inside concatenations here: xsim crashes on them)
    typedef struct {
        real t; bit [47:0] dmac; bit [31:0] sip, dip, xid, ci, o50, o54; bit [15:0] secs, flags; bit [7:0] mt;
        bit h50, h54, h12, h55, h61; bit [8*32-1:0] hn;
    } dm_t;
    task automatic decode(input bq f, input real t, output dm_t d);
        bq ps, ih; int p; bit ended = 0;
        d.t = t; d.mt = 0; d.h50 = 0; d.h54 = 0; d.h12 = 0; d.h55 = 0; d.h61 = 0; d.hn = 0;
        if (f.size() < 342) begin fail($sformatf("DHCP frame %0d bytes", f.size())); return; end
        d.dmac = gbe(f, 0, 6);
        if (gbe(f, 6, 6) != BMAC) fail("DHCP source MAC");
        if (f[14] != 8'h45 || gbe(f, 16, 2) != 16'd328 || f[22] != 8'd64 || gbe(f, 20, 2) != 16'h4000) fail("DHCP IPv4 header");
        for (int i = 14; i < 34; i++) ih.push_back(f[i]);
        if (csum(ih) != 16'h0) fail("DHCP IPv4 checksum");
        d.sip = gbe(f, 26, 4); d.dip = gbe(f, 30, 4);
        if (gbe(f, 34, 2) != 16'd68 || gbe(f, 36, 2) != 16'd67 || gbe(f, 38, 2) != 16'd308) fail("DHCP UDP header");
        for (int i = 26; i < 34; i++) ps.push_back(f[i]);                     // pseudo header: addresses,
        ps.push_back(8'd0); ps.push_back(8'd17); ps.push_back(8'd1); ps.push_back(8'd52);   // protocol, length 308
        for (int i = 34; i < 342; i++) ps.push_back(f[i]);
        if (gbe(f, 40, 2) == 16'h0 || csum(ps) != 16'h0) fail($sformatf("DHCP UDP checksum %04h", gbe(f, 40, 2)));
        if (f[42] != 8'd1 || f[43] != 8'd1 || f[44] != 8'd6 || f[45] != 8'd0) fail("BOOTP op / htype / hlen / hops");
        d.xid = gbe(f, 46, 4); d.secs = gbe(f, 50, 2); d.flags = gbe(f, 52, 2); d.ci = gbe(f, 54, 4);
        for (int i = 58; i < 70; i++) if (f[i] != 0) begin fail("yiaddr / siaddr / giaddr not 0"); break; end
        if (gbe(f, 70, 6) != BMAC) fail("chaddr");
        for (int i = 76; i < 278; i++) if (f[i] != 0) begin fail("chaddr pad / sname / file not 0"); break; end
        if (gbe(f, 278, 4) != 32'h63825363) fail("magic cookie");
        p = 282;
        while (p < 342) begin
            if (ended) begin if (f[p] != 0) begin fail("bytes after the end option"); break; end p++; end
            else if (f[p] == 8'd255) begin ended = 1; p++; end
            else if (f[p] == 8'd0) p++;
            else begin
                int c = f[p], n = f[p + 1];
                case (c)
                    53: d.mt = f[p + 2];
                    50: begin d.h50 = 1; d.o50 = gbe(f, p + 2, 4); end
                    54: begin d.h54 = 1; d.o54 = gbe(f, p + 2, 4); end
                    12: begin d.h12 = 1; for (int i = 0; i < n; i++) d.hn = (d.hn << 8) | f[p + 2 + i]; end
                    55: begin d.h55 = 1; if (n != 4 || f[p+2] != 1 || f[p+3] != 3 || f[p+4] != 58 || f[p+5] != 59) fail("option 55"); end
                    61: begin d.h61 = 1; if (n != 7 || f[p+2] != 1 || gbe(f, p + 3, 6) != BMAC) fail("option 61"); end
                    default: fail($sformatf("unexpected option %0d", c));
                endcase
                p += 2 + n;
            end
        end
        if (!ended) fail("no end option");
        if (!d.h61) fail("no client identifier");
    endtask
    task automatic get_dhcp(output dm_t d, output bit got, input real tmo_s);
        real t0 = now_s();
        got = 0;
        while (dq.size() == 0 && now_s() - t0 < tmo_s) @(posedge clk);
        if (dq.size() == 0) return;
        begin bq f = dq.pop_front(); real t = dq_t.pop_front(); decode(f, t, d); got = 1; end
    endtask
    task automatic expect_dhcp(output dm_t d, input [7:0] mt, input real tmo_s, input string what);
        bit got;
        get_dhcp(d, got, tmo_s);
        if (!got) begin fail($sformatf("no %s within %0.1f s", what, tmo_s)); d.mt = 0; return; end
        if (d.mt != mt) fail($sformatf("%s: message type %0d, expected %0d", what, d.mt, mt));
        begin
            string s_sip, s_dip, s_ci, s50, s54;
            s_sip = ipstr(d.sip); s_dip = ipstr(d.dip); s_ci = ipstr(d.ci);
            s50 = "-"; s54 = "-";
            if (d.h50) s50 = ipstr(d.o50);
            if (d.h54) s54 = ipstr(d.o54);
            $display("INFO_TB %0.2f s: %s xid %08h secs %0d flags %04h from %s to %s ci %s 50 %s 54 %s", d.t, what,
                     d.xid, d.secs, d.flags, s_sip, s_dip, s_ci, s50, s54);
        end
    endtask
    // a broadcast DISCOVER / REQUEST(selecting) / REBIND / DECLINE
    task automatic chk_bcast(input dm_t d, input [31:0] sip, input string what);
        if (d.dmac != 48'hFFFF_FFFF_FFFF || d.dip != 32'hFFFF_FFFF || d.sip != sip)
            fail($sformatf("%s: to %012h %s from %s", what, d.dmac, ipstr(d.dip), ipstr(d.sip)));
    endtask

    // ---------------- server side ----------------
    function automatic bq opt4(input [7:0] c, input [31:0] v); return {c, 8'd4, be32(v)}; endfunction
    // a server reply: mt 2 OFFER, 5 ACK, 6 NAK; ovl 3 puts the server identifier into 'file' and the lease options
    // into 'sname' (option overload)
    function automatic bq reply(input [7:0] mt, input [31:0] xid, input [31:0] yi, input [31:0] lease,
                                input int t1 = -1, input int t2 = -1, input int ovl = 0, input [47:0] ch = BMAC);
        bq m, o54, ol, opts, sname, file;
        m = {8'd2, 8'd1, 8'd6, 8'd0, be32(xid), be16(16'd0), be16(16'h8000), be32(32'h0), be32(yi), be32(SIP),
             be32(32'h0), mac(ch)};
        repeat (10) m.push_back(8'h00);
        o54 = opt4(8'd54, SIP);
        ol = {};
        if (mt != 8'd6) begin
            ol = {opt4(8'd51, lease), opt4(8'd1, 32'hFFFF_FF00), opt4(8'd3, SIP)};
            if (t1 >= 0) ol = {ol, opt4(8'd58, t1)};
            if (t2 >= 0) ol = {ol, opt4(8'd59, t2)};
        end
        if (ovl == 3) begin
            opts  = {8'd53, 8'd1, mt, 8'd52, 8'd1, 8'd3, 8'd255};
            file  = {8'd0, o54, 8'd255};                 // a pad byte first
            sname = {ol, 8'd255};
        end else begin
            opts = {8'd53, 8'd1, mt, o54, ol, 8'd255};
            file = {}; sname = {};
        end
        while (sname.size() < 64) sname.push_back(8'h00);
        while (file.size() < 128) file.push_back(8'h00);
        m = {m, sname, file, 8'd99, 8'd130, 8'd83, 8'd99, opts};
        while (m.size() < 300) m.push_back(8'h00);
        return m;
    endfunction
    task automatic serve(input bq pl, input bit unicast = 0, input [31:0] dip_u = 32'h0);
        bq u = {be16(16'd67), be16(16'd68), be16(8 + pl.size()), be16(16'h0000), pl};
        u_phy.send_frame(pad60({mac(unicast ? BMAC : 48'hFFFF_FFFF_FFFF), mac(SMAC), be16(16'h0800),
                                ipv4(SIP, unicast ? dip_u : 32'hFFFF_FFFF, 8'd17, u)}));
    endtask

    // ---------------- host side ----------------
    // ARP request for tip from the host; answered = a reply came within 40 us
    task automatic arp_query(input [31:0] tip, output bit answered);
        int t = 0;
        rq.delete();
        u_phy.send_frame(pad60({mac(48'hFFFF_FFFF_FFFF), mac(HMAC), be16(16'h0806), be16(16'd1), be16(16'h0800), 8'd6, 8'd4,
                                be16(16'd1), mac(HMAC), be32(HIP), mac(48'h0), be32(tip)}));
        answered = 0;
        while (t < 4000 && !answered) begin
            @(posedge clk); t++;
            while (rq.size() > 0) begin
                bq r = rq.pop_front();
                if (gbe(r, 28, 4) == tip && gbe(r, 22, 6) == BMAC && gbe(r, 38, 4) == HIP && gbe(r, 0, 6) == HMAC) answered = 1;
                else fail("ARP reply fields");
            end
        end
    endtask
    task automatic expect_arp(input [31:0] tip, input bit yes, input string what);
        bit a;
        arp_query(tip, a);
        if (a != yes) fail($sformatf("%s: ARP for %s %s", what, ipstr(tip), a ? "answered" : "not answered"));
    endtask
    // UDP to dip:5000 from the host; delivered = it reached the payload FIFO
    task automatic udp_to(input [31:0] dip, output bit delivered);
        int n0 = n_dg;
        bq u = {be16(16'd7000), be16(16'd5000), be16(16'd28), be16(16'h0000)};
        for (int i = 0; i < 20; i++) u.push_back(i);
        u_phy.send_frame(pad60({mac(BMAC), mac(HMAC), be16(16'h0800), ipv4(HIP, dip, 8'd17, u)}));
        repeat (4000) @(posedge clk);
        delivered = (n_dg != n0);
    endtask
    // ARP probe / announcement checks
    task automatic get_arp(output bq a, output real t, output bit got, input real tmo_s);
        real t0 = now_s();
        got = 0;
        while (aq.size() == 0 && now_s() - t0 < tmo_s) @(posedge clk);
        if (aq.size() == 0) return;
        a = aq.pop_front(); t = aq_t.pop_front(); got = 1;
        if (gbe(a, 0, 6) != 48'hFFFF_FFFF_FFFF || gbe(a, 6, 6) != BMAC || gbe(a, 14, 6) != 48'h0001_0800_0604 ||
            gbe(a, 20, 2) != 16'd1 || gbe(a, 22, 6) != BMAC || gbe(a, 32, 6) != 48'h0)
            fail("ARP request fields");
    endtask
    // kind 0 = probe for addr, 1 = announcement of addr
    task automatic expect_arp_req(input int kind, input [31:0] addr, input real tmo_s, output real t, input string what);
        bq a; bit got;
        get_arp(a, t, got, tmo_s);
        if (!got) begin fail($sformatf("no %s within %0.1f s", what, tmo_s)); t = now_s(); return; end
        if (gbe(a, 38, 4) != addr || gbe(a, 28, 4) != (kind ? addr : 32'h0))
            fail($sformatf("%s: sender %s target %s", what, ipstr(gbe(a, 28, 4)), ipstr(gbe(a, 38, 4))));
        else $display("INFO_TB %0.2f s: %s %s", t, what, ipstr(addr));
    endtask
    task automatic other_arp(input [15:0] op, input [31:0] spa, input [31:0] tpa);
        u_phy.send_frame(pad60({mac(48'hFFFF_FFFF_FFFF), mac(OMAC), be16(16'h0806), be16(16'd1), be16(16'h0800), 8'd6, 8'd4,
                                be16(op), mac(OMAC), be32(spa), mac(48'h0), be32(tpa)}));
    endtask
    task automatic wait_s(input real s); real t0 = now_s(); while (now_s() - t0 < s) @(posedge clk); endtask
    task automatic wait_until(input real t); while (now_s() < t) @(posedge clk); endtask
    task automatic bounce(input bit g, output real t_up);
        physr <= 16'h0000; wait (!link_up);
        repeat (2000) @(posedge clk);
        giga <= g; physr <= g ? 16'h002C : 16'h001C;
        wait (link_up); t_up = now_s();
        dq.delete(); dq_t.delete(); aq.delete(); aq_t.delete();
    endtask
    task automatic chk_addr(input [31:0] a, input bit v, input bit l, input string what);
        if (ip_valid != v || (v && (ip != a || ip_leased != l)))
            fail($sformatf("%s: address %s valid %0d leased %0d", what, ipstr(ip), ip_valid, ip_leased));
    endtask
    // a probe sequence: 3 probes for addr 0-1 s after t0, 1-2 s apart; the announcements 2 s after the last, 2 s apart
    task automatic expect_probes(input [31:0] addr, input real t0, output real t_last);
        real t, tp;
        tp = t0;
        for (int i = 0; i < 3; i++) begin
            expect_arp_req(0, addr, 3.0, t, $sformatf("probe %0d", i + 1));
            if (i == 0 && (t - t0 > 1.1)) fail($sformatf("first probe %0.2f s after the ACK", t - t0));
            if (i > 0 && (t - tp < 0.95 || t - tp > 2.15)) fail($sformatf("probe gap %0.2f s", t - tp));
            tp = t;
        end
        t_last = tp;
    endtask

    // ---------------- sequence ----------------
    localparam [31:0] Y1 = 32'hC0A80064, Y2 = 32'hC0A80065, Y3 = 32'hC0A80066, Y4 = 32'hC0A80067, Y5 = 32'hC0A80068;
    initial begin
        dm_t d, r; bit got, a; real t_up, t, t1, t_req;
        repeat (20) @(posedge clk); rst <= 0;
        wait (init_done);
        if (ip_valid) fail("an address before any link");

        // ---------------- A: no server ----------------
        $display("INFO_TB --- A: 1000M, no server ---");
        physr <= 16'h002C; giga <= 1;
        wait (link_up); t_up = now_s();
        expect_dhcp(d, 8'd1, 1.5, "DISCOVER 1");
        chk_bcast(d, 32'h0, "DISCOVER");
        if (d.t - t_up > 1.1) fail($sformatf("first DISCOVER %0.2f s after link up", d.t - t_up));
        if (d.flags != 16'h8000 || d.ci != 0 || d.h50 || d.h54 || !d.h12 || d.hn != 256'("fpga-gpu") || !d.h55)
            fail("DISCOVER fields");
        r = d;
        wait_until(t_up + 2.5);
        chk_addr(0, 0, 0, "before the fallback time");
        expect_arp(FB, 0, "before the fallback time");
        expect_dhcp(d, 8'd1, 4.0, "DISCOVER 2");
        if (d.xid != r.xid) fail("DISCOVER retransmission changed xid");
        if (d.t - r.t < 2.9 || d.t - r.t > 5.1) fail($sformatf("DISCOVER retransmitted after %0.2f s", d.t - r.t));
        if (d.secs < 3) fail($sformatf("secs %0d", d.secs));
        wait_until(t_up + 9.5);
        chk_addr(0, 0, 0, "0.5 s before the fallback time");
        wait_until(t_up + 10.3);
        chk_addr(FB, 1, 0, "after the fallback time");
        expect_arp(FB, 1, "fallback address");
        udp_to(FB, a); if (!a) fail("UDP to the fallback address not delivered");

        // ---------------- B: 100M, a server ----------------
        $display("INFO_TB --- B: 100M, OFFER / REQUEST / ACK, probes ---");
        bounce(1'b0, t_up);
        chk_addr(FB, 1, 0, "fallback kept over the link bounce");
        expect_dhcp(d, 8'd1, 1.5, "DISCOVER");
        if (d.h50) fail("DISCOVER asks for an address it does not hold");
        r = d;
        serve(reply(8'd2, r.xid ^ 32'h1, Y1, 32'd30));                       // wrong xid
        serve(reply(8'd2, r.xid, Y1, 32'd30, -1, -1, 0, 48'h02_00_00_00_00_77)); // wrong chaddr
        get_dhcp(d, got, 0.5);
        if (got) fail("a REQUEST for an OFFER with a wrong xid / chaddr");
        serve(reply(8'd2, r.xid, Y1, 32'd30));
        expect_dhcp(d, 8'd3, 0.5, "REQUEST (selecting)");
        chk_bcast(d, 32'h0, "REQUEST (selecting)");
        if (d.xid != r.xid || !d.h50 || d.o50 != Y1 || !d.h54 || d.o54 != SIP || d.ci != 0 || d.flags != 16'h8000 || !d.h12)
            fail("REQUEST (selecting) fields");
        t_req = d.t;
        serve(reply(8'd5, r.xid, Y1, 32'd30));
        t = now_s();
        expect_probes(Y1, t, t1);
        chk_addr(FB, 1, 0, "while probing");
        expect_arp(FB, 1, "while probing");
        expect_arp_req(1, Y1, 2.5, t, "announcement 1");
        if (t - t1 < 1.9 || t - t1 > 2.3) fail($sformatf("announcement %0.2f s after the last probe", t - t1));
        chk_addr(Y1, 1, 1, "bound");
        expect_arp(Y1, 1, "lease");
        expect_arp(FB, 0, "fallback address after the bind");
        udp_to(Y1, a); if (!a) fail("UDP to the lease not delivered");
        udp_to(FB, a); if (a) fail("UDP to the old fallback address delivered");
        // ICMP echo to the lease: the reply comes from it
        begin
            bq icmp = {8'd8, 8'd0, be16(16'h0), be16(16'h0042), be16(16'h0007), 8'd1, 8'd2, 8'd3, 8'd4};
            bit [15:0] c = csum(icmp); icmp[2] = c[15:8]; icmp[3] = c[7:0];
            oq.delete();
            u_phy.send_frame(pad60({mac(BMAC), mac(HMAC), be16(16'h0800), ipv4(HIP, Y1, 8'd1, icmp)}));
            repeat (6000) @(posedge clk);
            if (oq.size() != 1) fail($sformatf("%0d frames for one ICMP echo", oq.size()));
            else begin
                bq e = oq.pop_front();
                if (e[23] != 8'd1 || gbe(e, 26, 4) != Y1 || gbe(e, 30, 4) != HIP) fail("ICMP echo reply from the lease");
            end
        end
        // a reply-FIFO datagram: source = the lease
        begin
            oq.delete();
            @(posedge clk);
            rf_wen <= 1; rf_wd <= {K_D0, 2'd3, 16'd6000, 16'd4}; @(posedge clk);
            rf_wd <= {K_DN, 2'd3, HIP}; @(posedge clk);
            rf_wd <= {K_DN, 2'd3, HMAC[31:0]}; @(posedge clk);
            rf_wd <= {K_DN, 2'd3, 16'h0, HMAC[47:32]}; @(posedge clk);
            rf_wd <= {K_LAST, 2'd3, 32'h04030201}; @(posedge clk);
            rf_wen <= 0;
            repeat (6000) @(posedge clk);
            if (oq.size() != 1) fail($sformatf("%0d frames for one reply datagram", oq.size()));
            else begin
                bq e = oq.pop_front();
                if (gbe(e, 26, 4) != Y1 || gbe(e, 34, 2) != 16'd5000 || gbe(e, 40, 2) != 16'h0) fail("reply datagram source");
            end
        end
        expect_arp_req(1, Y1, 2.5, t1, "announcement 2");
        if (t1 - t < 1.9 || t1 - t > 2.2) fail($sformatf("announcements %0.2f s apart", t1 - t));

        // ---------------- C: renewal, rebinding, the lease's end ----------------
        $display("INFO_TB --- C: renew, rebind, expiry ---");
        expect_dhcp(d, 8'd3, 16.0, "REQUEST (renewing)");
        if (d.t - t_req < 13.9 || d.t - t_req > 15.3) fail($sformatf("renewal %0.2f s after the REQUEST", d.t - t_req));
        if (d.dmac != SMAC || d.dip != SIP || d.sip != Y1 || d.ci != Y1 || d.h50 || d.h54 || d.flags != 0 || !d.h12)
            fail("REQUEST (renewing) fields");
        t_req = d.t;
        serve(reply(8'd5, d.xid, Y1, 32'd24, 6, 12), 1, Y1);
        wait_s(1.0);
        chk_addr(Y1, 1, 1, "renewed");
        expect_dhcp(d, 8'd3, 7.0, "REQUEST (renewing, T1 6 s)");
        if (d.t - t_req < 4.9 || d.t - t_req > 6.3) fail($sformatf("renewal %0.2f s after the last", d.t - t_req));
        expect_dhcp(r, 8'd3, 7.5, "REQUEST (rebinding)");
        chk_bcast(r, Y1, "REQUEST (rebinding)");
        if (r.ci != Y1 || r.h50 || r.h54 || r.flags != 0 || r.xid == d.xid) fail("REQUEST (rebinding) fields");
        if (r.t - t_req < 10.9 || r.t - t_req > 12.3) fail($sformatf("rebinding %0.2f s after the renewal ACK", r.t - t_req));
        wait_until(t_req + 23.0);
        chk_addr(Y1, 1, 1, "1 s before the lease's end");
        expect_dhcp(d, 8'd1, 3.0, "DISCOVER after the lease's end");
        if (d.t - t_req < 22.9 || d.t - t_req > 25.3) fail($sformatf("lease end %0.2f s after the renewal", d.t - t_req));
        if (d.h50) fail("DISCOVER asks for the expired address");
        chk_addr(0, 0, 0, "lease ended");
        expect_arp(Y1, 0, "expired lease");
        t = d.t;
        wait_until(t + 10.5);
        chk_addr(FB, 1, 0, "fallback after the lease's end");
        expect_arp(FB, 1, "fallback after the lease's end");

        // ---------------- D: overload, NAK, conflicts ----------------
        $display("INFO_TB --- D: 1000M, overload, NAK, conflicts ---");
        bounce(1'b1, t_up);
        expect_dhcp(d, 8'd1, 1.5, "DISCOVER");
        serve(reply(8'd2, d.xid, Y2, 32'd60, -1, -1, 3), 1, Y2);                 // unicast, option overload
        expect_dhcp(r, 8'd3, 0.5, "REQUEST for the overloaded OFFER");
        if (!r.h50 || r.o50 != Y2 || !r.h54 || r.o54 != SIP) fail("REQUEST after the overloaded OFFER");
        serve(reply(8'd6, r.xid, 32'h0, 32'd0));
        expect_dhcp(d, 8'd1, 2.5, "DISCOVER after the NAK");
        if (d.t - r.t < 0.95 || d.t - r.t > 2.2) fail($sformatf("DISCOVER %0.2f s after the NAK", d.t - r.t));
        if (d.xid == r.xid) fail("same xid after the NAK");
        chk_addr(FB, 1, 0, "after the NAK");
        serve(reply(8'd2, d.xid, Y3, 32'd60));
        expect_dhcp(r, 8'd3, 0.5, "REQUEST");
        serve(reply(8'd5, r.xid, Y3, 32'd60));
        expect_arp_req(0, Y3, 1.5, t, "probe 1");
        other_arp(16'd2, Y3, Y3);                                               // another station has the address
        expect_dhcp(d, 8'd4, 0.5, "DECLINE");
        chk_bcast(d, 32'h0, "DECLINE");
        if (!d.h50 || d.o50 != Y3 || !d.h54 || d.o54 != SIP || d.h12 || d.h55 || d.ci != 0 || d.secs != 0)
            fail("DECLINE fields");
        t = d.t;
        wait_s(2.5);
        if (aq.size() != 0) fail("probes after the conflict");
        chk_addr(FB, 1, 0, "after the DECLINE");
        expect_dhcp(d, 8'd1, 11.0, "DISCOVER after the DECLINE");
        if (d.t - t < 9.95 || d.t - t > 10.2) fail($sformatf("DISCOVER %0.2f s after the DECLINE", d.t - t));
        serve(reply(8'd2, d.xid, Y4, 32'd60));
        expect_dhcp(r, 8'd3, 0.5, "REQUEST");
        serve(reply(8'd5, r.xid, Y4, 32'd60));
        expect_arp_req(0, Y4, 1.5, t, "probe 1");
        other_arp(16'd1, 32'h0, Y4);                                            // another station probes for it
        expect_dhcp(d, 8'd4, 0.5, "DECLINE (probe conflict)");
        if (d.o50 != Y4) fail("DECLINE address");
        expect_dhcp(d, 8'd1, 11.0, "DISCOVER after the DECLINE");
        serve(reply(8'd2, d.xid, Y5, 32'd60));
        expect_dhcp(r, 8'd3, 0.5, "REQUEST");
        serve(reply(8'd5, r.xid, Y5, 32'd60));
        t = now_s();
        other_arp(16'd1, 32'hC0A80099, 32'hC0A80098);                           // unrelated ARP: no conflict
        expect_probes(Y5, t, t1);
        expect_arp_req(1, Y5, 2.5, t, "announcement 1");
        chk_addr(Y5, 1, 1, "bound after the conflicts");
        expect_arp(Y5, 1, "lease");

        // ---------------- E: link bounces while bound ----------------
        $display("INFO_TB --- E: bounce while bound ---");
        aq.delete(); aq_t.delete();
        bounce(1'b1, t_up);
        expect_arp(Y5, 1, "lease kept over the link bounce");
        expect_dhcp(d, 8'd1, 1.5, "DISCOVER");
        if (!d.h50 || d.o50 != Y5) fail("DISCOVER does not ask for the address held");
        serve(reply(8'd2, d.xid, Y5, 32'd60));
        expect_dhcp(r, 8'd3, 0.5, "REQUEST");
        serve(reply(8'd5, r.xid, Y5, 32'd60));
        expect_arp_req(1, Y5, 0.3, t, "announcement (same address, no probes)");
        chk_addr(Y5, 1, 1, "same address bound again");
        bounce(1'b1, t_up);
        expect_dhcp(d, 8'd1, 1.5, "DISCOVER");
        wait_until(t_up + 9.5);
        chk_addr(Y5, 1, 1, "lease kept until the fallback time");
        wait_until(t_up + 10.3);
        chk_addr(FB, 1, 0, "fallback after an unanswered re-acquisition");
        expect_arp(FB, 1, "fallback address");
        expect_arp(Y5, 0, "old lease after the fallback");

        if (errs == 0) $display("TB_PASS tb_dhcp");
        else           $display("TB_FAIL tb_dhcp errors=%0d", errs);
        $finish;
    end
    initial begin #60ms; $display("TB_FAIL tb_dhcp TIMEOUT at %0.2f s", now_s()); $finish; end
    always @(dstate or ip or ip_valid or ip_leased)
        $display("TRACE %0.2f s: DHCP state %0d, address %08h valid %0d leased %0d", now_s(), dstate, ip, ip_valid, ip_leased);
endmodule
`default_nettype wire
