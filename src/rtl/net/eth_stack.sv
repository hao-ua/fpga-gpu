// eth_stack — board A Ethernet: RGMII 100/1000 + MAC + ARP/ICMP/UDP + DHCP client + MDIO.
// Clock domains: eth_rxc (PHY receive clock, 125 / 25 MHz) for everything on the wire side, clk_sys for
// the payload FIFO read side, the reply FIFO write side, MDIO, the DHCP client, controls and the status snapshot.
//   payload FIFO (eth -> sys): UDP datagrams to MY_PORT, net_pkg word format (descriptor + payload)
//   reply FIFO   (sys -> eth): UDP datagrams to send, same format (descriptor = destination)
//   DHCP buffers: one received DHCP message (eth -> sys), one message to send (sys -> eth)
// The address comes from dhcp_client (FALLBACK_IP when DHCP = 0); it and the address being probed reach the
// wire side through a toggle handshake (they change at most every 1024 clk_sys cycles).
// The wire-side datapath is held in reset while the link is down; the FIFOs are only reset by rst_sys
// (consumers resynchronise on descriptor words if a datagram was cut by a link drop).
`default_nettype none
module eth_stack #(
    parameter [47:0] MY_MAC  = 48'h00_0a_35_01_02_03,
    parameter [31:0] FALLBACK_IP = {8'd192, 8'd168, 8'd1, 8'd10},
    parameter [15:0] MY_PORT = 16'd5000,
    parameter bit    DHCP    = 1'b1,
    parameter [8*32-1:0] HOSTNAME = "fpga-gpu",
    parameter int    IDELAY_DEFAULT = 0,
    parameter int    PF_AW   = 12,
    parameter int    RF_AW   = 11,
    parameter int    CLK_HZ  = 100_000_000,
    parameter int    DHCP_MS_CYCLES = CLK_HZ / 1000,
    parameter int    DHCP_FALLBACK_MS = 10_000,
    parameter int    SNAP_CYCLES = 100_000,     // counter snapshot period (1 ms)
    parameter int    MDIO_RST_MS = 20, MDIO_WAIT_MS = 100, MDIO_POLL_US = 5000
)(
    input  wire        clk_sys, rst_sys, clk_idelay,
    // RGMII / MDIO pins
    input  wire        rgmii_rxc, rgmii_rx_ctl,
    input  wire [3:0]  rgmii_rxd,
    output wire        rgmii_txc, rgmii_tx_ctl,
    output wire [3:0]  rgmii_txd,
    output wire        phy_rstn,
    output wire        mdc, mdio_o, mdio_oe,
    input  wire        mdio_i,
    // payload FIFO read side (clk_sys)
    output wire        pf_rvalid,
    output wire [35:0] pf_rdata,
    input  wire        pf_ren,
    // reply FIFO write side (clk_sys)
    input  wire        rf_wen,
    input  wire [35:0] rf_wdata,
    output wire [RF_AW:0] rf_wcount,
    // controls (clk_sys)
    input  wire        dly_ld,
    input  wire [4:0]  dly_val,
    input  wire [4:0]  phyad,
    input  wire        rxdly, adv1000, reinit,
    input  wire        phy_req, phy_req_wr,
    input  wire [15:0] phy_req_page,
    input  wire [4:0]  phy_req_reg,
    input  wire [15:0] phy_req_wdata,
    output wire        phy_resp,
    output wire [15:0] phy_resp_rdata,
    // status (clk_sys)
    output wire        mdio_init_done, link_up, duplex,
    output wire [1:0]  speed,
    output wire [15:0] physr, bmsr,
    output wire [31:0] mdio_polls,
    output reg  [32*14-1:0] cnt_snap,          // {c_resync,c_tx,c_icmpdrop,c_pfdrop,c_other,c_ipcsum,c_udp,c_icmp,c_arp,c_er,c_ovf,c_len,c_fcs,c_ok}
    output reg  [31:0] snap_age,               // clk_sys cycles since the last completed snapshot
    // the address in use (clk_sys)
    output wire [31:0] ip_addr,
    output wire        ip_valid, ip_leased,
    output wire [3:0]  dhcp_state
);
    import net_pkg::*;

    // ---------------- MDIO ----------------
    mdio_mgr #(.CLK_HZ(CLK_HZ), .RST_MS(MDIO_RST_MS), .WAIT_MS(MDIO_WAIT_MS), .POLL_US(MDIO_POLL_US)) u_mdio (
        .clk(clk_sys), .rst(rst_sys), .phyad(phyad), .rxdly(rxdly), .adv1000(adv1000), .reinit(reinit),
        .phy_rstn(phy_rstn), .mdc(mdc), .mdio_o(mdio_o), .mdio_oe(mdio_oe), .mdio_i(mdio_i),
        .req(phy_req), .req_wr(phy_req_wr), .req_page(phy_req_page), .req_reg(phy_req_reg),
        .req_wdata(phy_req_wdata), .resp(phy_resp), .resp_rdata(phy_resp_rdata),
        .init_done(mdio_init_done), .link_up(link_up), .speed(speed), .duplex(duplex),
        .physr(physr), .bmsr(bmsr), .polls(mdio_polls));

    // ---------------- receive clock, resets ----------------
    (* IODELAY_GROUP = "rgmii_delay" *) IDELAYCTRL u_idelayctrl (.REFCLK(clk_idelay), .RST(rst_sys), .RDY());

    wire rxc;
    wire rst_eth_hw, rst_eth_dp;
    rst_sync u_rs_hw (.clk(rxc), .arst(rst_sys), .rst(rst_eth_hw));
    rst_sync u_rs_dp (.clk(rxc), .arst(rst_sys | ~link_up), .rst(rst_eth_dp));
    (* ASYNC_REG = "TRUE" *) reg [1:0] giga_s = 0;
    always @(posedge rxc) giga_s <= {giga_s[0], speed == 2'b10};
    wire giga = giga_s[1];

    // ---------------- DHCP client (clk_sys) and its buffers ----------------
    localparam int DH_AW = 11, DT_AW = 9;
    wire        dh_rel, dt_req, dt_arp, dt_we, acd_on_s;
    wire [47:0] dt_mac; wire [31:0] dt_sip, dt_dip, acd_ip_s; wire [15:0] dt_len, dt_ucs;
    wire [DH_AW-1:0] dh_ra; wire [DT_AW-1:0] dt_wa; wire [7:0] dt_wd;
    reg  [7:0]  dh_rd;
    reg         dh_full = 1'b0;                    // Ethernet domain: a message is in the receive buffer
    reg  [15:0] dh_len; reg [47:0] dh_smac;
    (* ASYNC_REG = "TRUE" *) reg [2:0] dh_full_s = 0, dt_done_s = 0, acd_s = 0;
    reg         acd_q = 1'b0;
    wire        dt_done;
    reg         acd_t = 1'b0;                      // Ethernet domain: toggles on each conflict
    always @(posedge clk_sys) begin
        dh_full_s <= {dh_full_s[1:0], dh_full};
        dt_done_s <= {dt_done_s[1:0], dt_done};
        acd_s <= {acd_s[1:0], acd_t}; acd_q <= acd_s[2];
    end
    dhcp_client #(.MY_MAC(MY_MAC), .FALLBACK_IP(FALLBACK_IP), .ENABLE(DHCP), .HOSTNAME(HOSTNAME),
                  .MS_CYCLES(DHCP_MS_CYCLES), .FALLBACK_MS(DHCP_FALLBACK_MS), .RX_AW(DH_AW), .TX_AW(DT_AW)) u_dhcp (
        .clk(clk_sys), .rst(rst_sys), .link_up(link_up),
        .rx_full(dh_full_s[2]), .rx_len(dh_len), .rx_smac(dh_smac), .rx_rel(dh_rel), .rx_addr(dh_ra), .rx_data(dh_rd),
        .tx_req(dt_req), .tx_arp(dt_arp), .tx_mac(dt_mac), .tx_sip(dt_sip), .tx_dip(dt_dip), .tx_len(dt_len),
        .tx_ucs(dt_ucs), .tx_done(dt_done_s[2]), .tx_we(dt_we), .tx_wa(dt_wa), .tx_wd(dt_wd),
        .acd_ip(acd_ip_s), .acd_on(acd_on_s), .acd_hit(acd_s[2] ^ acd_q),
        .ip(ip_addr), .ip_valid(ip_valid), .ip_lease(ip_leased), .state(dhcp_state));

    // receive buffer: written by eth_net_rx, read by the client; dh_full / dh_rel four-phase
    (* ram_style = "block" *) reg [7:0] dh_buf [0:(1<<DH_AW)-1];
    wire              dh_we, dh_done; wire [DH_AW-1:0] dh_wa; wire [7:0] dh_wd;
    wire [15:0]       dh_len_w; wire [47:0] dh_smac_w;
    always @(posedge rxc) if (dh_we) dh_buf[dh_wa] <= dh_wd;
    always @(posedge clk_sys) dh_rd <= dh_buf[dh_ra];
    (* ASYNC_REG = "TRUE" *) reg [2:0] dh_rel_e = 0;
    always @(posedge rxc) begin
        dh_rel_e <= {dh_rel_e[1:0], dh_rel};
        if (rst_eth_hw) dh_full <= 1'b0;
        else if (dh_done) begin dh_full <= 1'b1; dh_len <= dh_len_w; dh_smac <= dh_smac_w; end
        else if (dh_rel_e[2]) dh_full <= 1'b0;
    end
    wire dh_busy = dh_full | dh_rel_e[2];

    // transmit buffer: written by the client, read by eth_tx; dt_req / dt_done four-phase
    (* ram_style = "block" *) reg [7:0] dt_buf [0:(1<<DT_AW)-1];
    wire [DT_AW-1:0] dt_ra; reg [7:0] dt_rd;
    always @(posedge clk_sys) if (dt_we) dt_buf[dt_wa] <= dt_wd;
    always @(posedge rxc) dt_rd <= dt_buf[dt_ra];
    (* ASYNC_REG = "TRUE" *) reg [2:0] dt_req_e = 0;
    always @(posedge rxc) dt_req_e <= {dt_req_e[1:0], dt_req};

    // address configuration {valid, address, probing, probed address} to the Ethernet domain: cfg_q changes together
    // with the toggle and then holds for 1024 cycles, so the wire side, which copies it on a toggle change, sees it
    // settled
    reg [65:0] cfg_q = 0; reg cfg_t = 1'b0; reg [9:0] cfg_hold = 0;
    wire [65:0] cfg_n = {ip_valid, ip_addr, acd_on_s, acd_ip_s};
    always @(posedge clk_sys)
        if (rst_sys) begin cfg_q <= 0; cfg_t <= 1'b0; cfg_hold <= 0; end
        else if (cfg_hold != 0) cfg_hold <= cfg_hold - 1'b1;
        else if (cfg_n != cfg_q) begin cfg_q <= cfg_n; cfg_t <= ~cfg_t; cfg_hold <= 10'd1023; end
    (* ASYNC_REG = "TRUE" *) reg [2:0] cfg_s = 0;
    reg cfg_seen = 1'b0; reg [65:0] e_cfg = 0;
    always @(posedge rxc) begin
        cfg_s <= {cfg_s[1:0], cfg_t};
        if (rst_eth_hw) begin cfg_seen <= 1'b0; e_cfg <= 0; end
        else if (cfg_s[2] != cfg_seen) begin cfg_seen <= cfg_s[2]; e_cfg <= cfg_q; end
    end
    wire        e_have = e_cfg[65], e_acd_on = e_cfg[32];
    wire [31:0] e_ip = e_cfg[64:33], e_acd_ip = e_cfg[31:0];
    wire        acd_hit;
    always @(posedge rxc) if (rst_eth_hw) acd_t <= 1'b0; else if (acd_hit) acd_t <= ~acd_t;

    // ---------------- receive path ----------------
    wire       r_dv, r_stb, r_er; wire [7:0] r_data;
    eth_rgmii_rx #(.IDELAY_DEFAULT(IDELAY_DEFAULT)) u_rgmii_rx (
        .clk_ctl(clk_sys), .dly_ld(dly_ld), .dly_val(dly_val),
        .rgmii_rxc(rgmii_rxc), .rgmii_rx_ctl(rgmii_rx_ctl), .rgmii_rxd(rgmii_rxd),
        .rxc(rxc), .rst(rst_eth_dp), .giga(giga), .dv(r_dv), .stb(r_stb), .data(r_data), .er(r_er));

    localparam int RING_AW = 15;
    wire              f_avail, f_release;
    wire [13:0]       f_len;
    wire [RING_AW-1:0] f_start, ring_ra;
    wire [7:0]        ring_rd;
    wire [31:0]       c_ok, c_fcs, c_len, c_ovf, c_er;
    eth_mac_rx #(.RING_AW(RING_AW)) u_mac_rx (
        .clk(rxc), .rst(rst_eth_dp), .dv(r_dv), .stb(r_stb), .er(r_er), .data(r_data),
        .f_avail(f_avail), .f_len(f_len), .f_start(f_start), .rd_addr(ring_ra), .rd_data(ring_rd),
        .f_release(f_release), .c_ok(c_ok), .c_fcs(c_fcs), .c_len(c_len), .c_ovf(c_ovf), .c_er(c_er));

    localparam int ICMP_AW = 11;
    wire        arp_req, arp_ack, icmp_req, icmp_ack;
    wire [47:0] arp_mac, icmp_mac;
    wire [31:0] arp_ip, icmp_ip;
    wire [15:0] icmp_len, icmp_csum;
    wire [ICMP_AW-1:0] icmp_ra;
    wire [7:0]  icmp_rd;
    wire        pf_wen; wire [35:0] pf_wdata; wire [PF_AW:0] pf_wcount;
    wire [31:0] c_arp, c_icmp, c_udp, c_ipcsum, c_other, c_pfdrop, c_icmpdrop;
    eth_net_rx #(.MY_MAC(MY_MAC), .MY_PORT(MY_PORT), .RING_AW(RING_AW), .ICMP_AW(ICMP_AW),
                 .PF_AW(PF_AW), .DH_AW(DH_AW)) u_net_rx (
        .clk(rxc), .rst(rst_eth_dp),
        .my_ip(e_ip), .have_ip(e_have), .acd_ip(e_acd_ip), .acd_on(e_acd_on), .acd_hit(acd_hit),
        .dh_busy(dh_busy), .dh_we(dh_we), .dh_wa(dh_wa), .dh_wd(dh_wd), .dh_done(dh_done), .dh_len(dh_len_w),
        .dh_smac(dh_smac_w),
        .f_avail(f_avail), .f_len(f_len), .f_start(f_start),
        .rd_addr(ring_ra), .ring_data(ring_rd), .f_release(f_release),
        .arp_req(arp_req), .arp_mac(arp_mac), .arp_ip(arp_ip), .arp_ack(arp_ack),
        .icmp_req(icmp_req), .icmp_mac(icmp_mac), .icmp_ip(icmp_ip), .icmp_len(icmp_len),
        .icmp_csum(icmp_csum), .icmp_ack(icmp_ack), .icmp_raddr(icmp_ra), .icmp_rdata(icmp_rd),
        .pf_wen(pf_wen), .pf_wdata(pf_wdata), .pf_wcount(pf_wcount),
        .c_arp(c_arp), .c_icmp(c_icmp), .c_udp(c_udp), .c_ipcsum(c_ipcsum), .c_other(c_other),
        .c_pfdrop(c_pfdrop), .c_icmpdrop(c_icmpdrop));

    wire pf_rrst;
    rst_sync u_rs_sys (.clk(clk_sys), .arst(rst_sys), .rst(pf_rrst));
    afifo_bram #(.W(36), .AW(PF_AW)) u_pf (
        .wclk(rxc), .wrst(rst_eth_hw), .wen(pf_wen), .wdata(pf_wdata), .wfull(), .wcount(pf_wcount),
        .rclk(clk_sys), .rrst(pf_rrst), .ren(pf_ren), .rdata(pf_rdata), .rvalid(pf_rvalid), .rcount());

    // ---------------- transmit path ----------------
    wire        rf_rvalid, rf_ren; wire [35:0] rf_rdata; wire [RF_AW:0] rf_rcount;
    afifo_bram #(.W(36), .AW(RF_AW)) u_rf (
        .wclk(clk_sys), .wrst(pf_rrst), .wen(rf_wen), .wdata(rf_wdata), .wfull(), .wcount(rf_wcount),
        .rclk(rxc), .rrst(rst_eth_hw), .ren(rf_ren), .rdata(rf_rdata), .rvalid(rf_rvalid), .rcount(rf_rcount));

    wire        tx_valid, tx_adv; wire [7:0] tx_data;
    wire [31:0] c_tx, c_resync;
    eth_tx #(.MY_MAC(MY_MAC), .MY_PORT(MY_PORT), .ICMP_AW(ICMP_AW), .RF_AW(RF_AW), .SVC_AW(DT_AW)) u_tx (
        .clk(rxc), .rst(rst_eth_dp), .giga(giga), .my_ip(e_ip),
        .svc_req(dt_req_e[2]), .svc_arp(dt_arp), .svc_mac(dt_mac), .svc_sip(dt_sip), .svc_dip(dt_dip),
        .svc_len(dt_len), .svc_ucs(dt_ucs), .svc_done(dt_done), .svc_raddr(dt_ra), .svc_rdata(dt_rd),
        .arp_req(arp_req), .arp_mac(arp_mac), .arp_ip(arp_ip), .arp_ack(arp_ack),
        .icmp_req(icmp_req), .icmp_mac(icmp_mac), .icmp_ip(icmp_ip), .icmp_len(icmp_len),
        .icmp_csum(icmp_csum), .icmp_ack(icmp_ack), .icmp_raddr(icmp_ra), .icmp_rdata(icmp_rd),
        .rf_valid(rf_rvalid), .rf_data(rf_rdata), .rf_count(rf_rcount), .rf_ren(rf_ren),
        .tx_valid(tx_valid), .tx_data(tx_data), .tx_adv(tx_adv), .c_tx(c_tx), .c_resync(c_resync));

    rgmii_tx u_rgmii_tx (.clk(rxc), .rst(rst_eth_dp), .speed(giga ? 2'b10 : 2'b01),
        .valid(tx_valid), .data(tx_data), .adv(tx_adv),
        .rgmii_txc(rgmii_txc), .rgmii_tx_ctl(rgmii_tx_ctl), .rgmii_txd(rgmii_txd));

    // ---------------- counter snapshot (eth -> sys, toggle handshake) ----------------
    wire [32*14-1:0] cnt_now = {c_resync, c_tx, c_icmpdrop, c_pfdrop, c_other, c_ipcsum, c_udp, c_icmp,
                                c_arp, c_er, c_ovf, c_len, c_fcs, c_ok};
    reg  [32*14-1:0] shadow;
    reg  snap_req = 0, snap_ack = 0;
    (* ASYNC_REG = "TRUE" *) reg [2:0] req_e = 0;
    (* ASYNC_REG = "TRUE" *) reg [2:0] ack_s = 0;
    always @(posedge rxc) begin
        req_e <= {req_e[1:0], snap_req};
        if (req_e[2] != req_e[1]) begin shadow <= cnt_now; snap_ack <= ~snap_ack; end
    end
    reg [31:0] st_t;
    reg        waiting;
    always @(posedge clk_sys) begin
        ack_s <= {ack_s[1:0], snap_ack};
        if (rst_sys) begin st_t <= 0; waiting <= 0; snap_age <= 0; cnt_snap <= 0; end
        else begin
            snap_age <= snap_age + 1;
            if (waiting) begin
                if (ack_s[2] != ack_s[1]) begin cnt_snap <= shadow; waiting <= 0; snap_age <= 0; end
            end else if (st_t >= SNAP_CYCLES - 1) begin
                st_t <= 0; snap_req <= ~snap_req; waiting <= 1'b1;
            end else st_t <= st_t + 1;
        end
    end
endmodule
`default_nettype wire
