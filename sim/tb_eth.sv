`timescale 1ns/1ps
// tb_eth — eth_stack against rgmii_phy_model, at 1000 then 100 Mbit/s:
//   MDIO init sequence, link/speed from PHYSR, ARP request -> reply, ICMP echo -> reply (checksums),
//   UDP to port 5000 -> payload FIFO (descriptor + payload, byte exact), bad FCS / bad IP checksum /
//   other port / RX_ER -> dropped and counted, reply FIFO -> UDP frame (headers, checksum, FCS),
//   20 back-to-back 1400-byte datagrams, host PHY register read, counter snapshot.
// Prints TB_PASS / TB_FAIL.
`default_nettype none
module tb_eth;
    import net_pkg::*;
    reg clk = 0, clk200 = 0, rst = 1;
    always #5   clk = ~clk;
    always #2.5 clk200 = ~clk200;

    localparam [47:0] BMAC = 48'h00_0a_35_01_02_03, HMAC = 48'h02_00_00_00_00_01;
    localparam [31:0] BIP  = 32'hC0A8010A, HIP = 32'hC0A80170;

    reg         giga = 1;
    reg  [15:0] physr = 16'h0000;
    wire rxc, rx_ctl, txc, tx_ctl, phy_rstn, mdc, mdio_o, mdio_oe; wire [3:0] rxd, txd;
    tri1 mdio;
    assign mdio = mdio_oe ? mdio_o : 1'bz;
    rgmii_phy_model u_phy (.giga(giga), .rgmii_rxc(rxc), .rgmii_rx_ctl(rx_ctl), .rgmii_rxd(rxd),
        .rgmii_txc(txc), .rgmii_tx_ctl(tx_ctl), .rgmii_txd(txd), .mdc(mdc), .mdio(mdio), .physr(physr));

    wire        pf_v; wire [35:0] pf_d; reg pf_ren = 0;
    reg         rf_wen = 0; reg [35:0] rf_wd; wire [11:0] rf_wcount;
    reg         phy_req = 0; wire phy_resp; wire [15:0] phy_rd;
    wire        init_done, link_up, duplex; wire [1:0] speed; wire [15:0] physr_o, bmsr_o; wire [31:0] polls;
    wire [32*14-1:0] cnt; wire [31:0] age;
    eth_stack #(.MY_MAC(BMAC), .MY_IP(BIP), .SNAP_CYCLES(2000), .MDIO_RST_MS(1), .MDIO_WAIT_MS(1),
                .MDIO_POLL_US(100)) dut (
        .clk_sys(clk), .rst_sys(rst), .clk_idelay(clk200),
        .rgmii_rxc(rxc), .rgmii_rx_ctl(rx_ctl), .rgmii_rxd(rxd), .rgmii_txc(txc), .rgmii_tx_ctl(tx_ctl),
        .rgmii_txd(txd), .phy_rstn(phy_rstn), .mdc(mdc), .mdio_o(mdio_o), .mdio_oe(mdio_oe), .mdio_i(mdio),
        .pf_rvalid(pf_v), .pf_rdata(pf_d), .pf_ren(pf_ren),
        .rf_wen(rf_wen), .rf_wdata(rf_wd), .rf_wcount(rf_wcount),
        .dly_ld(1'b0), .dly_val(5'd0), .phyad(5'd1), .rxdly(1'b0), .adv1000(1'b1), .reinit(1'b0),
        .phy_req(phy_req), .phy_req_wr(1'b0), .phy_req_page(16'h0000), .phy_req_reg(5'h02),
        .phy_req_wdata(16'h0), .phy_resp(phy_resp), .phy_resp_rdata(phy_rd),
        .mdio_init_done(init_done), .link_up(link_up), .duplex(duplex), .speed(speed),
        .physr(physr_o), .bmsr(bmsr_o), .mdio_polls(polls), .cnt_snap(cnt), .snap_age(age));

    int errs = 0;
    task automatic fail(input string m); errs++; $display("FAIL: %s", m); endtask
    function automatic [31:0] C(input int i); return cnt[32*i +: 32]; endfunction

    // ---------------- frame builders ----------------
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
    function automatic bq ipv4(input [31:0] sip, input [31:0] dip, input [7:0] proto, input bq pl, input bit bad = 0);
        bq h; bit [15:0] c;
        h = {8'h45, 8'h00, be16(20 + pl.size()), be16(16'h1234), be16(16'h4000), 8'd64, proto, be16(16'h0000), be32(sip), be32(dip)};
        c = csum(h); if (bad) c ^= 16'h0100;
        h[10] = c[15:8]; h[11] = c[7:0];
        return {h, pl};
    endfunction
    function automatic bq udp_frame(input [15:0] sport, input [15:0] dport, input bq pl, input bit badip = 0);
        bq u = {be16(sport), be16(dport), be16(8 + pl.size()), be16(16'h0000), pl};
        return pad60({mac(BMAC), mac(HMAC), be16(16'h0800), ipv4(HIP, BIP, 8'd17, u, badip)});
    endfunction
    function automatic bq rnd_bytes(input int n, input int seed);
        bq q; int unsigned r = seed;
        for (int i = 0; i < n; i++) begin r = r * 1103515245 + 12345; q.push_back(r >> 16); end
        return q;
    endfunction

    // ---------------- payload FIFO consumer ----------------
    // completed datagrams: parallel queues (header fields) + one flat byte queue for all payloads
    int          dq_len [$]; bit [15:0] dq_port [$]; bit [31:0] dq_ip [$]; bit [47:0] dq_mac [$];
    byte unsigned dq_bytes [$];
    int          c_len, c_got; bit [15:0] c_port; bit [31:0] c_ip; bit [47:0] c_mac;
    int          dstep = 0; int unsigned crnd = 99;
    always @(posedge clk) begin
        crnd = crnd * 1103515245 + 12345;
        if (pf_v && pf_ren) begin
            case (pf_d[35:34])
                K_D0: begin if (dstep != 0) fail("descriptor d0 out of order");
                            c_len = pf_d[15:0]; c_port = pf_d[31:16]; c_got = 0; dstep = 1; end
                K_DN: begin
                    case (dstep)
                        1: c_ip = pf_d[31:0];
                        2: c_mac[31:0] = pf_d[31:0];
                        3: c_mac[47:32] = pf_d[15:0];
                        default: fail("extra descriptor word");
                    endcase
                    dstep++;
                    if (dstep == 4 && c_len == 0) begin
                        dq_len.push_back(0); dq_port.push_back(c_port); dq_ip.push_back(c_ip); dq_mac.push_back(c_mac); dstep = 0;
                    end
                end
                default: begin
                    if (dstep != 4) fail("payload before descriptor");
                    for (int b = 0; b <= pf_d[33:32]; b++)
                        if (c_got < c_len) begin dq_bytes.push_back(pf_d[8*b +: 8]); c_got++; end
                    if (pf_d[35:34] == K_LAST) begin
                        if (c_got != c_len) fail("payload length mismatch");
                        dq_len.push_back(c_len); dq_port.push_back(c_port); dq_ip.push_back(c_ip); dq_mac.push_back(c_mac);
                        dstep = 0;
                    end
                end
            endcase
        end
        pf_ren <= (crnd >> 16) % 100 < 70;
    end

    task automatic expect_dgram(input bq pl, input [15:0] sport, input int tmo_us = 200);
        int t = 0, n;
        bit ok = 1;
        while (dq_len.size() == 0 && t < tmo_us * 100) begin @(posedge clk); t++; end
        if (dq_len.size() == 0) begin fail("no datagram in payload FIFO"); return; end
        n = dq_len.pop_front();
        if (n != pl.size() || dq_port[0] != sport || dq_ip[0] != HIP || dq_mac[0] != HMAC)
            fail($sformatf("descriptor len %0d/%0d port %0d ip %h mac %h", n, pl.size(), dq_port[0], dq_ip[0], dq_mac[0]));
        void'(dq_port.pop_front()); void'(dq_ip.pop_front()); void'(dq_mac.pop_front());
        for (int i = 0; i < n; i++) begin
            byte unsigned b = dq_bytes.pop_front();
            if (i < pl.size() && b != pl[i]) ok = 0;
        end
        if (!ok) fail("payload bytes differ");
    endtask

    // ---------------- TX frame checks ----------------
    task automatic get_tx(output bq f, input int tmo_us = 200);
        int t = 0;
        while (u_phy.txq.size() == 0 && t < tmo_us * 100) begin @(posedge clk); t++; end
        if (u_phy.txq.size() == 0) begin fail("no TX frame"); f = {}; return; end
        f = u_phy.txq.pop_front();
        // preamble + SFD
        for (int i = 0; i < 7; i++) if (f[i] != 8'h55) begin fail("TX preamble"); break; end
        if (f[7] != 8'hD5) fail("TX SFD");
        f = f[8:$];
        begin
            bq body = f[0:$-4];
            bit [31:0] c = u_phy.crc32(body);
            bit [31:0] got = 0;
            for (int k = 0; k < 4; k++) got[8*k +: 8] = f[f.size() - 4 + k];
            if (got != c) fail($sformatf("TX FCS %h exp %h", got, c));
            if (body.size() < 60) fail("TX frame shorter than 60");
            f = body;
        end
    endtask

    // element-wise compare of a[ao..] with b[bo..] to the end of b (xsim crashes on queue-slice compares)
    function automatic bit qtail_eq(input bq a, input int ao, input bq b, input int bo);
        if (a.size() - ao != b.size() - bo) return 0;
        for (int i = 0; i < b.size() - bo; i++) if (a[ao + i] != b[bo + i]) return 0;
        return 1;
    endfunction
    // big-endian field extraction (xsim cannot concatenate queue elements)
    function automatic [63:0] gbe(input bq q, input int i, input int n);
        bit [63:0] v = 0;
        for (int k = 0; k < n; k++) v = (v << 8) | q[i + k];
        return v;
    endfunction

    task automatic check_ip(input bq f, input [7:0] proto, output bq pl);
        bq h = f[14:33];
        int tl;
        if (gbe(f, 0, 6) != HMAC || gbe(f, 6, 6) != BMAC || gbe(f, 12, 2) != 16'h0800) fail("TX Ethernet header");
        if (csum(h) != 16'h0000) fail("TX IPv4 header checksum");
        if (h[9] != proto || gbe(h, 12, 4) != BIP || gbe(h, 16, 4) != HIP) fail("TX IPv4 fields");
        tl = gbe(h, 2, 2);
        pl = f[34:14 + tl - 1];
    endtask

    task automatic send_burst(input int n);
        for (int i = 0; i < n; i++) u_phy.send_frame(udp_frame(16'd7000 + i, 16'd5000, rnd_bytes(1400, 1000 + i)));
    endtask

    // ---------------- test sequence per speed ----------------
    task automatic run_suite(input string tag);
        bq f, pl, r;
        int c0_ok, c0_fcs, c0_ipcs, c0_other, c0_er;
        $display("INFO_TB --- suite %s ---", tag);
        repeat (3000) @(posedge clk);
        c0_ok = C(0); c0_fcs = C(1); c0_ipcs = C(8); c0_other = C(9); c0_er = C(4);
        // ARP
        f = pad60({mac(48'hFFFFFFFFFFFF), mac(HMAC), be16(16'h0806), be16(1), be16(16'h0800), 8'd6, 8'd4, be16(1),
                   mac(HMAC), be32(HIP), mac(48'h0), be32(BIP)});
        u_phy.send_frame(f);
        get_tx(r);
        if (r.size() >= 42) begin
            if (gbe(r, 12, 2) != 16'h0806 || gbe(r, 20, 2) != 16'h0002 || gbe(r, 22, 6) != BMAC ||
                gbe(r, 28, 4) != BIP || gbe(r, 32, 6) != HMAC || gbe(r, 38, 4) != HIP || gbe(r, 0, 6) != HMAC)
                fail("ARP reply fields");
            else $display("INFO_TB %s ARP reply ok", tag);
        end
        // ICMP echo, 100-byte payload
        begin
            bq data = rnd_bytes(100, 5), icmp;
            bit [15:0] c;
            icmp = {8'd8, 8'd0, be16(16'h0), be16(16'h0042), be16(16'h0007), data};
            c = csum(icmp); icmp[2] = c[15:8]; icmp[3] = c[7:0];
            u_phy.send_frame(pad60({mac(BMAC), mac(HMAC), be16(16'h0800), ipv4(HIP, BIP, 8'd1, icmp)}));
            get_tx(r); check_ip(r, 8'd1, pl);
            if (pl.size() != icmp.size() || pl[0] != 0 || csum(pl) != 0 || !qtail_eq(pl, 4, icmp, 4)) fail("ICMP echo reply");
            else $display("INFO_TB %s ICMP echo reply ok", tag);
        end
        // UDP datagrams of several sizes
        for (int k = 0; k < 7; k++) begin
            int sizes [7] = '{1, 3, 4, 5, 64, 333, 1472};
            int sz = sizes[k];
            bq p = rnd_bytes(sz, 100 + sz);
            u_phy.send_frame(udp_frame(16'd40000 + sz, 16'd5000, p));
            expect_dgram(p, 16'd40000 + sz);
        end
        $display("INFO_TB %s UDP delivery checked", tag);
        // drops
        u_phy.send_frame(udp_frame(16'd1, 16'd5000, rnd_bytes(50, 1)), 1);          // bad FCS
        u_phy.send_frame(udp_frame(16'd2, 16'd5000, rnd_bytes(50, 2), 1));          // bad IP checksum
        u_phy.send_frame(udp_frame(16'd3, 16'd5001, rnd_bytes(50, 3)));             // other port
        if (giga) u_phy.send_frame(udp_frame(16'd4, 16'd5000, rnd_bytes(50, 4)), 0, 30);   // RX_ER
        repeat (3000) @(posedge clk);
        if (dq_len.size() != 0) fail("a dropped datagram reached the payload FIFO");
        // reply FIFO -> UDP frame
        begin
            bq p = rnd_bytes(37, 77);
            @(posedge clk);
            rf_wen <= 1; rf_wd <= {K_D0, 2'd3, 16'd6000, 16'd37}; @(posedge clk);
            rf_wd <= {K_DN, 2'd3, HIP}; @(posedge clk);
            rf_wd <= {K_DN, 2'd3, HMAC[31:0]}; @(posedge clk);
            rf_wd <= {K_DN, 2'd3, 16'h0, HMAC[47:32]}; @(posedge clk);
            for (int w = 0; w < 10; w++) begin
                bit [31:0] word = 0;
                for (int b = 0; b < 4; b++) if (4*w + b < 37) word[8*b +: 8] = p[4*w + b];
                rf_wd <= {(w == 9) ? K_LAST : K_PAY, (w == 9) ? 2'd0 : 2'd3, word}; @(posedge clk);
            end
            rf_wen <= 0;
            get_tx(r); check_ip(r, 8'd17, pl);
            if (gbe(pl, 0, 2) != 16'd5000 || gbe(pl, 2, 2) != 16'd6000 || gbe(pl, 4, 2) != 16'd45 || !qtail_eq(pl, 8, p, 0))
                fail("UDP reply content");
            else $display("INFO_TB %s UDP reply ok", tag);
        end
        // 20 back-to-back 1400-byte datagrams
        fork
            send_burst(20);
        join_none
        for (int n = 0; n < 20; n++) expect_dgram(rnd_bytes(1400, 1000 + n), 16'd7000 + n, 2000);
        $display("INFO_TB %s 20 back-to-back datagrams ok", tag);
        repeat (6000) @(posedge clk);
        if (C(1) - c0_fcs != 1) fail($sformatf("FCS error counter delta %0d", C(1) - c0_fcs));
        if (C(8) - c0_ipcs != 1) fail($sformatf("IP checksum counter delta %0d", C(8) - c0_ipcs));
        if (C(9) - c0_other != 1) fail($sformatf("other counter delta %0d", C(9) - c0_other));
        if (giga && C(4) - c0_er != 1) fail($sformatf("RX_ER counter delta %0d", C(4) - c0_er));
        $display("INFO_TB %s counters ok=%0d fcs=%0d len=%0d ovf=%0d er=%0d arp=%0d icmp=%0d udp=%0d ipcs=%0d other=%0d pfdrop=%0d tx=%0d",
                 tag, C(0), C(1), C(2), C(3), C(4), C(5), C(6), C(7), C(8), C(9), C(10), C(12));
    endtask

    initial begin
        repeat (20) @(posedge clk); rst <= 0;
        // MDIO init with the link down
        wait (init_done);
        begin
            bit [36:0] exp [10] = '{{16'h0000, 5'h1f, 16'h0d08}, {16'h0d08, 5'h11, 16'h0109}, {16'h0d08, 5'h1f, 16'h0d08},
                                    {16'h0d08, 5'h15, 16'h0011}, {16'h0d08, 5'h1f, 16'h0a5d}, {16'h0a5d, 5'h10, 16'h0000},
                                    {16'h0a5d, 5'h1f, 16'h0000}, {16'h0000, 5'h04, 16'h0101},
                                    {16'h0000, 5'h09, 16'h0200}, {16'h0000, 5'h00, 16'h1340}};
            if (u_phy.wlog.size() < 10) fail("MDIO init: too few writes");
            else for (int i = 0; i < 10; i++)
                if (u_phy.wlog[i] != exp[i]) fail($sformatf("MDIO init write %0d = %h exp %h", i, u_phy.wlog[i], exp[i]));
            $display("INFO_TB MDIO init sequence checked (%0d writes)", u_phy.wlog.size());
        end
        // host PHY register read (page 0 reg 2 = PHY ID 1)
        @(posedge clk); phy_req <= 1; wait (phy_resp); @(posedge clk); phy_req <= 0;
        if (phy_rd != 16'h001C) fail($sformatf("PHY_REG read %h", phy_rd));
        // link up at 1000 Mbit/s
        physr <= 16'h002C; giga <= 1;
        wait (link_up); repeat (10) @(posedge clk);
        if (speed != 2'b10 || !duplex) fail("speed/duplex at 1000");
        run_suite("1000M");
        // link down, then up at 100 Mbit/s
        physr <= 16'h0000; wait (!link_up);
        giga <= 0; physr <= 16'h001C;
        wait (link_up); repeat (10) @(posedge clk);
        if (speed != 2'b01) fail("speed at 100");
        run_suite("100M");
        if (errs == 0) $display("TB_PASS tb_eth");
        else           $display("TB_FAIL tb_eth errors=%0d", errs);
        $finish;
    end
    initial begin #40ms; $display("TB_FAIL tb_eth TIMEOUT"); $finish; end
endmodule
`default_nettype wire
