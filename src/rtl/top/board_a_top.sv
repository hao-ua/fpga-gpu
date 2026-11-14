// board_a_top — PA200T-Starlite board A (Ethernet host link + board link role A):
// Ethernet 100/1000 + DDR3 + host protocol + board link to B.
//   200 MHz oscillator -> gpu_clk_mig (DDR3 MIG, clk_sys = ui_clk 100 MHz, clk_idelay 200 MHz, which also
//   feeds the link transmit MMCM)
//   board_a_core: eth_stack, protocol engine, DDR ingress queue, dispatcher, transfer engine, link.
// LEDs: led[0] = DDR calibrated, led[1] = Ethernet link up.
`default_nettype none
module board_a_top #(
    parameter [31:0] BUILD_ID       = 32'd0,
    parameter int    IDELAY_DEFAULT = 19,      // measured working point on board A at 1000 Mbit/s:
    parameter int    RXDLY_DEFAULT  = 1,       // PHY RX delay on, tap 19 = centre of the error-free taps 7..31
    parameter int    OC_MHZ         = 100      // clk_sys = MIG ui_clk in MHz (DDR rate 8 x this); 100 = the generated MIG IP
)(
    input  wire        sys_clk_p, sys_clk_n,
    input  wire        rst_btn_n,
    // Ethernet (RTL8211F, RGMII)
    input  wire        rgmii_rxc, rgmii_rx_ctl,
    input  wire [3:0]  rgmii_rxd,
    output wire        rgmii_txc, rgmii_tx_ctl,
    output wire [3:0]  rgmii_txd,
    output wire        phy_rstn,
    output wire        eth_mdc,
    inout  wire        eth_mdio,
    // DDR3
    inout  wire [31:0] ddr3_dq,
    inout  wire [3:0]  ddr3_dqs_n, ddr3_dqs_p,
    output wire [14:0] ddr3_addr,
    output wire [2:0]  ddr3_ba,
    output wire        ddr3_ras_n, ddr3_cas_n, ddr3_we_n, ddr3_reset_n,
    output wire [0:0]  ddr3_ck_p, ddr3_ck_n, ddr3_cke, ddr3_cs_n, ddr3_odt,
    output wire [3:0]  ddr3_dm,
    // board link (JM1, JM2)
    inout  wire [31:0] jm1, jm2,
    output wire [1:0]  led
);
    wire clk_sys, clk_idelay, clk200_raw, rst_sys, calib_done;
    wire        app_en, app_wdf_wren, app_wdf_end, app_rdy, app_wdf_rdy, app_rd_data_valid;
    wire [2:0]  app_cmd; wire [28:0] app_addr; wire [255:0] app_wdf_data, app_rd_data; wire [31:0] app_wdf_mask;

    gpu_clk_mig #(.OC_MHZ(OC_MHZ)) u_clk_mig (
        .sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n), .rst_n(rst_btn_n),
        .clk_sys(clk_sys), .clk_idelay(clk_idelay), .clk200_raw(clk200_raw), .rst_sys(rst_sys), .calib_done(calib_done),
        .ddr3_dq(ddr3_dq), .ddr3_dqs_n(ddr3_dqs_n), .ddr3_dqs_p(ddr3_dqs_p), .ddr3_addr(ddr3_addr), .ddr3_ba(ddr3_ba),
        .ddr3_ras_n(ddr3_ras_n), .ddr3_cas_n(ddr3_cas_n), .ddr3_we_n(ddr3_we_n), .ddr3_reset_n(ddr3_reset_n),
        .ddr3_ck_p(ddr3_ck_p), .ddr3_ck_n(ddr3_ck_n), .ddr3_cke(ddr3_cke), .ddr3_cs_n(ddr3_cs_n), .ddr3_odt(ddr3_odt),
        .ddr3_dm(ddr3_dm),
        .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en), .app_wdf_data(app_wdf_data), .app_wdf_end(app_wdf_end),
        .app_wdf_wren(app_wdf_wren), .app_wdf_mask(app_wdf_mask), .app_rd_data(app_rd_data),
        .app_rd_data_valid(app_rd_data_valid), .app_rd_data_end(), .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy));

    wire mdio_o, mdio_oe, mdio_i;
    IOBUF u_mdio (.IO(eth_mdio), .I(mdio_o), .T(~mdio_oe), .O(mdio_i));

    board_a_core #(.BUILD_ID(BUILD_ID), .IDELAY_DEFAULT(IDELAY_DEFAULT[4:0]), .RXDLY_DEFAULT(RXDLY_DEFAULT[0]),
                   .CLK_HZ(OC_MHZ * 1_000_000)) u_core (
        .clk_sys(clk_sys), .rst_sys(rst_sys), .calib_done(calib_done), .clk_idelay(clk_idelay),
        .rgmii_rxc(rgmii_rxc), .rgmii_rx_ctl(rgmii_rx_ctl), .rgmii_rxd(rgmii_rxd),
        .rgmii_txc(rgmii_txc), .rgmii_tx_ctl(rgmii_tx_ctl), .rgmii_txd(rgmii_txd),
        .phy_rstn(phy_rstn), .mdc(eth_mdc), .mdio_o(mdio_o), .mdio_oe(mdio_oe), .mdio_i(mdio_i),
        .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr), .app_wdf_data(app_wdf_data), .app_wdf_mask(app_wdf_mask),
        .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end), .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy),
        .app_rd_data_valid(app_rd_data_valid), .app_rd_data(app_rd_data), .jm1(jm1), .jm2(jm2), .led(led));
endmodule
`default_nettype wire
