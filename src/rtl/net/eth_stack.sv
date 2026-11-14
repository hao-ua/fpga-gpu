// eth_stack — board A Ethernet: RGMII 100/1000 + MAC + ARP/ICMP/UDP + MDIO.
// Clock domains: eth_rxc (PHY receive clock, 125 / 25 MHz) for everything on the wire side, clk_sys for
// the payload FIFO read side, the reply FIFO write side, MDIO, controls and the status snapshot.
//   payload FIFO (eth -> sys): UDP datagrams to MY_PORT, net_pkg word format (descriptor + payload)
//   reply FIFO   (sys -> eth): UDP datagrams to send, same format (descriptor = destination)
// The wire-side datapath is held in reset while the link is down; the FIFOs are only reset by rst_sys
// (consumers resynchronise on descriptor words if a datagram was cut by a link drop).
`default_nettype none
module eth_stack #(
    parameter [47:0] MY_MAC  = 48'h00_0a_35_01_02_03,
    parameter [31:0] MY_IP   = {8'd192, 8'd168, 8'd1, 8'd10},
    parameter [15:0] MY_PORT = 16'd5000,
    parameter int    IDELAY_DEFAULT = 0,
    parameter int    PF_AW   = 12,
    parameter int    RF_AW   = 11,
    parameter int    CLK_HZ  = 100_000_000,
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
    output reg  [31:0] snap_age                // clk_sys cycles since the last completed snapshot
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
    eth_net_rx #(.MY_MAC(MY_MAC), .MY_IP(MY_IP), .MY_PORT(MY_PORT), .RING_AW(RING_AW), .ICMP_AW(ICMP_AW),
                 .PF_AW(PF_AW)) u_net_rx (
        .clk(rxc), .rst(rst_eth_dp), .f_avail(f_avail), .f_len(f_len), .f_start(f_start),
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
    eth_tx #(.MY_MAC(MY_MAC), .MY_IP(MY_IP), .MY_PORT(MY_PORT), .ICMP_AW(ICMP_AW), .RF_AW(RF_AW)) u_tx (
        .clk(rxc), .rst(rst_eth_dp), .giga(giga),
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
