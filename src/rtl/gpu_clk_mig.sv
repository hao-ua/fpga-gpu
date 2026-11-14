// gpu_clk_mig — clocking + DDR3 (MIG) wrapper for gpu2.
//   200 MHz board diff clock -> IBUFDS -> MIG (DDR3 MT41K256M16, 32-bit @400 MHz)
//   MIG ui_clk (100 MHz) = clk_sys: the GPU + command decoder run synchronous with
//   memory (no CDC to the framebuffer). clk_idelay (200 MHz) feeds the RGMII
//   IDELAYCTRL. The native MIG app interface (256-bit, addr[28:0]) is exposed for
//   the framebuffer. NOTE: 100 MHz (not 125) — the stock board MIG runs DDR3 at
//   400 MHz; 100 MHz leaves comfortable margin.
//   OC_MHZ: 100 = the generated MIG IP as it is;
//   110 / 120 / 125 = the MIG's own RTL (tools/gen/ip/.../user_design/rtl, read by tools/build_board.tcl with OC_MHZ set)
//   with its PLL re-parameterised for VCO = 8 x OC_MHZ (200 MHz x M / D), tCK = 1 / (4 OC_MHZ) and CL 7 / CWL 6 (JEDEC for
//   tCK < 2.5 ns). The IP generator refuses > 800 MT/s on the -2 part (AR 67179), so this is an OVERCLOCK of the PHY: the
//   calibration on the board decides whether it runs.
`default_nettype none
module gpu_clk_mig #(
    parameter int OC_MHZ = 100
)(
    // board
    input  wire        sys_clk_p, sys_clk_n,   // 200 MHz differential
    input  wire        rst_n,                  // active-low board reset
    // generated clocks / status
    output wire        clk_sys,                // 100 MHz (= ui_clk)
    output wire        clk_idelay,             // 200 MHz (IDELAYCTRL ref)
    output wire        clk200_raw,             // 200 MHz post-IBUFDS (no BUFG) for video MMCMs
    output wire        rst_sys,                // sync reset, clk_sys domain (active high)
    output wire        calib_done,             // DDR3 init/calibration complete
    // DDR3 pins
    inout  wire [31:0] ddr3_dq,
    inout  wire [3:0]  ddr3_dqs_n, ddr3_dqs_p,
    output wire [14:0] ddr3_addr,
    output wire [2:0]  ddr3_ba,
    output wire        ddr3_ras_n, ddr3_cas_n, ddr3_we_n, ddr3_reset_n,
    output wire [0:0]  ddr3_ck_p, ddr3_ck_n, ddr3_cke, ddr3_cs_n, ddr3_odt,
    output wire [3:0]  ddr3_dm,
    // native app interface (clk_sys domain) -> framebuffer
    input  wire [28:0]  app_addr,
    input  wire [2:0]   app_cmd,
    input  wire         app_en,
    input  wire [255:0] app_wdf_data,
    input  wire         app_wdf_end, app_wdf_wren,
    input  wire [31:0]  app_wdf_mask,
    output wire [255:0] app_rd_data,
    output wire         app_rd_data_valid, app_rd_data_end,
    output wire         app_rdy, app_wdf_rdy
);
    wire clk200;
    IBUFDS u_ibufds (.O(clk200), .I(sys_clk_p), .IB(sys_clk_n));
    BUFG   u_bufg_idelay (.I(clk200), .O(clk_idelay));   // 200 MHz IDELAYCTRL ref
    assign clk200_raw = clk200;                          // unbuffered -> video MMCM inputs

    wire ui_clk, ui_clk_sync_rst;
    assign clk_sys = ui_clk;
    assign rst_sys = ui_clk_sync_rst;

    generate if (OC_MHZ == 100) begin : g_ip
    mig_7series_0 u_mig (
        .ddr3_dq(ddr3_dq),.ddr3_dqs_n(ddr3_dqs_n),.ddr3_dqs_p(ddr3_dqs_p),
        .ddr3_addr(ddr3_addr),.ddr3_ba(ddr3_ba),
        .ddr3_ras_n(ddr3_ras_n),.ddr3_cas_n(ddr3_cas_n),.ddr3_we_n(ddr3_we_n),
        .ddr3_reset_n(ddr3_reset_n),.ddr3_ck_p(ddr3_ck_p),.ddr3_ck_n(ddr3_ck_n),
        .ddr3_cke(ddr3_cke),.ddr3_cs_n(ddr3_cs_n),.ddr3_dm(ddr3_dm),.ddr3_odt(ddr3_odt),
        .app_addr(app_addr),.app_cmd(app_cmd),.app_en(app_en),
        .app_wdf_data(app_wdf_data),.app_wdf_end(app_wdf_end),.app_wdf_wren(app_wdf_wren),
        .app_wdf_mask(app_wdf_mask),
        .app_rd_data(app_rd_data),.app_rd_data_end(app_rd_data_end),.app_rd_data_valid(app_rd_data_valid),
        .app_rdy(app_rdy),.app_wdf_rdy(app_wdf_rdy),
        .app_sr_req(1'b0),.app_ref_req(1'b0),.app_zq_req(1'b0),
        .app_sr_active(),.app_ref_ack(),.app_zq_ack(),
        .ui_clk(ui_clk),.ui_clk_sync_rst(ui_clk_sync_rst),
        .init_calib_complete(calib_done),
        // sys_clk_i must be on a global clock buffer: the MIG routes it to its internal
        // IDELAYCTRL REFCLK, which DRC (PLIDC-9) requires be driven by a clock buffer, not
        // straight off the IBUFDS. clk_idelay is the BUFG-buffered 200 MHz.
        .sys_clk_i(clk_idelay),.sys_rst(rst_n));
    end else begin : g_oc
        // the MIG core from its RTL with the PLL at VCO = 8 x OC_MHZ: mem clock = VCO / 2 (DDR rate = VCO MT/s), sync pulse
        // VCO / 32, ui_clk = VCO / 8 = OC_MHZ; 200 MHz x M / D, the PLLE2's PFD at 200 / D MHz. freq_refclk: below tCK 2500 the
        // MIG's PHASERs run FREQ_REF_DIV = NONE (ddr_mc_phy_wrapper), i.e. the reference IS the memory clock -> CLKOUT0_DIVIDE 2
        // (the stock 800 MT/s core has DIV2 and a 2 x reference; a divide of 1 would put 880 MHz on the PLLE2 output, over
        // its 800 MHz limit, and 880 MHz on the ISERDES clocks). What no PLL setting fixes on the -2 part:
        // the PHASER IN_FIFO / OUT_FIFO clock (memory clock / 2 = 220 MHz at 880 MT/s) is 10 % over its 200 MHz speed-file
        // limit (-0.455 ns pulse width) — the reason the IP caps this speed grade at 800 MT/s.
        localparam int PLL_M = (OC_MHZ == 110) ? 22 : (OC_MHZ == 120) ? 24 : (OC_MHZ == 125) ? 5 : 0;
        localparam int PLL_D = (OC_MHZ == 125) ? 1 : 5;
        localparam int TCK_PS = 1_000_000 / (4 * OC_MHZ);          // 110: 2272 ps (880 MT/s)
        if (PLL_M == 0) $error("gpu_clk_mig: OC_MHZ must be 100, 110, 120 or 125");
        mig_7series_0_mig #(.CLKFBOUT_MULT(PLL_M), .DIVCLK_DIVIDE(PLL_D), .CLKOUT0_DIVIDE(2), .tCK(TCK_PS), .CL(7), .CWL(6),
                            .MMCM_VCO(8 * OC_MHZ)) u_mig (
        .ddr3_dq(ddr3_dq),.ddr3_dqs_n(ddr3_dqs_n),.ddr3_dqs_p(ddr3_dqs_p),
        .ddr3_addr(ddr3_addr),.ddr3_ba(ddr3_ba),
        .ddr3_ras_n(ddr3_ras_n),.ddr3_cas_n(ddr3_cas_n),.ddr3_we_n(ddr3_we_n),
        .ddr3_reset_n(ddr3_reset_n),.ddr3_ck_p(ddr3_ck_p),.ddr3_ck_n(ddr3_ck_n),
        .ddr3_cke(ddr3_cke),.ddr3_cs_n(ddr3_cs_n),.ddr3_dm(ddr3_dm),.ddr3_odt(ddr3_odt),
        .app_addr(app_addr),.app_cmd(app_cmd),.app_en(app_en),
        .app_wdf_data(app_wdf_data),.app_wdf_end(app_wdf_end),.app_wdf_wren(app_wdf_wren),
        .app_wdf_mask(app_wdf_mask),
        .app_rd_data(app_rd_data),.app_rd_data_end(app_rd_data_end),.app_rd_data_valid(app_rd_data_valid),
        .app_rdy(app_rdy),.app_wdf_rdy(app_wdf_rdy),
        .app_sr_req(1'b0),.app_ref_req(1'b0),.app_zq_req(1'b0),
        .app_sr_active(),.app_ref_ack(),.app_zq_ack(),
        .ui_clk(ui_clk),.ui_clk_sync_rst(ui_clk_sync_rst),
        .init_calib_complete(calib_done), .device_temp(),
        .sys_clk_i(clk_idelay),.sys_rst(rst_n));
    end endgenerate
endmodule
`default_nettype wire
