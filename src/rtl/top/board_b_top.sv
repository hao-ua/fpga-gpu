// board_b_top — PA200T-Starlite board B (pixels, HDMI):
// DDR3 + scanout in five video modes, test images, board link role B + command processor.
//   200 MHz oscillator -> gpu_clk_mig (DDR3 MIG, clk_sys = ui_clk 100 MHz, clk_idelay = BUFG'd 200 MHz,
//   which also feeds the video MMCM)
//   board_b_core: mem_arb, disp_b (video MMCM + scanout + TMDS), img_table, xfer_lin, b_bringup.
// The Ethernet PHY of board B is held in reset.
// LEDs: led[0] = DDR calibrated, led[1] = video running (blinks until the board link is up).
// KEY1: short = next image, long = next mode.
`default_nettype none
module board_b_top #(
    parameter [31:0] BUILD_ID = 32'd0,
    parameter int    OC_MHZ   = 100          // clk_sys = MIG ui_clk in MHz (DDR rate 8 x this); 100 = the generated MIG IP
)(
    input  wire        sys_clk_p, sys_clk_n,
    input  wire        rst_btn_n,
    input  wire        key1_n,
    // HDMI (TMDS_33)
    output wire        hdmi_clk_p, hdmi_clk_n,
    output wire [2:0]  hdmi_data_p, hdmi_data_n,
    // Ethernet PHY reset (held)
    output wire        phy_rstn,
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

    assign phy_rstn = 1'b0;

    board_b_core #(.BUILD_ID(BUILD_ID), .CLK_HZ(OC_MHZ * 1_000_000)) u_core (      // no boot screen: BRINGUP is off by default
        .clk_sys(clk_sys), .rst_sys(rst_sys), .calib_done(calib_done), .clk200(clk_idelay), .key1_n(key1_n),
        .dbg_cmd_valid(1'b0), .dbg_cmd(64'h0),
        .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr), .app_wdf_data(app_wdf_data), .app_wdf_mask(app_wdf_mask),
        .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end), .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy),
        .app_rd_data_valid(app_rd_data_valid), .app_rd_data(app_rd_data),
        .hdmi_clk_p(hdmi_clk_p), .hdmi_clk_n(hdmi_clk_n), .hdmi_data_p(hdmi_data_p), .hdmi_data_n(hdmi_data_n),
        .jm1(jm1), .jm2(jm2), .led(led), .status_vec());
endmodule
`default_nettype wire
