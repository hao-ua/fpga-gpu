// mig_cal_test_top — the DDR3 PHY alone at an overclocked rate: the MIG from gpu_clk_mig at
// OC_MHZ (DDR rate 8 x OC_MHZ MT/s) driven by the MIG example design's traffic generator (PRBS data, PRBS addresses,
// reads and writes over the whole 1 GB), nothing else on the die. Answers the one question no timing report can:
// does the PHY calibrate and run error-free with its PHASER FIFO clocks over the -2 speed file (960 MT/s = 20 % over)?
//   led[0] = calibration complete, led[1] = a read-back compare error was seen (sticky until reprogram / reset button).
//   Status over JTAG USER1 (link_status_jtag, tools/mig_test_jtag.tcl): 32 words, see `fields` below.
// Loaded over JTAG only; the flashed board image comes back with boot_hw_device (or a power cycle).
`default_nettype none
module mig_cal_test_top #(
    parameter [31:0] BUILD_ID  = 32'd0,
    parameter int    OC_MHZ    = 120,       // clk_sys = ui_clk in MHz; the DDR runs at 8 x this (110 = 880 MT/s, 120 = 960)
    parameter [3:0]  DATA_MODE = 4'b0111,   // traffic generator data: 7 = PRBS, 2 = address as data (the example's default)
    parameter [31:0] END_ADDR  = 32'h0fff_ffff  // last TG address (app address units); the example stops at 32'h00ff_ffff
)(
    input  wire        sys_clk_p, sys_clk_n,
    input  wire        rst_btn_n,
    // DDR3
    inout  wire [31:0] ddr3_dq,
    inout  wire [3:0]  ddr3_dqs_n, ddr3_dqs_p,
    output wire [14:0] ddr3_addr,
    output wire [2:0]  ddr3_ba,
    output wire        ddr3_ras_n, ddr3_cas_n, ddr3_we_n, ddr3_reset_n,
    output wire [0:0]  ddr3_ck_p, ddr3_ck_n, ddr3_cke, ddr3_cs_n, ddr3_odt,
    output wire [3:0]  ddr3_dm,
    output wire [1:0]  led
);
    // ---------------- clocks + MIG ----------------
    wire clk, rst, calib_done, clk_idelay, clk200_raw;
    wire         app_en, app_wdf_wren, app_wdf_end, app_rdy, app_wdf_rdy, app_rd_data_valid, app_rd_data_end;
    wire [2:0]   app_cmd;
    wire [28:0]  app_addr;
    wire [255:0] app_wdf_data, app_rd_data;
    wire [31:0]  app_wdf_mask;

    gpu_clk_mig #(.OC_MHZ(OC_MHZ)) u_clk_mig (
        .sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n), .rst_n(rst_btn_n),
        .clk_sys(clk), .clk_idelay(clk_idelay), .clk200_raw(clk200_raw), .rst_sys(rst), .calib_done(calib_done),
        .ddr3_dq(ddr3_dq), .ddr3_dqs_n(ddr3_dqs_n), .ddr3_dqs_p(ddr3_dqs_p), .ddr3_addr(ddr3_addr), .ddr3_ba(ddr3_ba),
        .ddr3_ras_n(ddr3_ras_n), .ddr3_cas_n(ddr3_cas_n), .ddr3_we_n(ddr3_we_n), .ddr3_reset_n(ddr3_reset_n),
        .ddr3_ck_p(ddr3_ck_p), .ddr3_ck_n(ddr3_ck_n), .ddr3_cke(ddr3_cke), .ddr3_cs_n(ddr3_cs_n), .ddr3_odt(ddr3_odt),
        .ddr3_dm(ddr3_dm),
        .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en), .app_wdf_data(app_wdf_data), .app_wdf_end(app_wdf_end),
        .app_wdf_wren(app_wdf_wren), .app_wdf_mask(app_wdf_mask), .app_rd_data(app_rd_data),
        .app_rd_data_valid(app_rd_data_valid), .app_rd_data_end(app_rd_data_end), .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy));

    // ---------------- the MIG example design's traffic generator ----------------
    // Parameters as example_top.v generates them for this core (DDR3, 32-bit, 4:1, BL8, 1 rank: app data 256 bits,
    // TG address = bank 3 + row 15 + column 10 = 28 bits), except the data mode (PRBS) and the address range (the whole
    // 1 GB: 2^28 app addresses of 4 bytes, so every bank and row is visited; the example stops at 64 MB).
    wire [31:0] tg_addr;
    assign app_addr = tg_addr[28:0];
    wire         tg_error, cmp_error, cmp_valid, pat_done, cmd_wdt, wr_wdt, rd_wdt;
    wire [47:0]  wr_cnt, rd_cnt;
    wire [3:0]   dq_lane_err, dq_lane_cmp;
    wire [575:0] err_status;                 // 64 + 2 x 256: {read data, expected data, address / command / status}
    wire [255:0] cmp_data;

    mig_7series_v4_2_traffic_gen_top #(
        .TCQ(100), .SIMULATION("FALSE"), .FAMILY("VIRTEX7"), .MEM_TYPE("DDR3"), .TST_MEM_INSTR_MODE("R_W_INSTR_MODE"),
        .nCK_PER_CLK(4), .NUM_DQ_PINS(32), .MEM_BURST_LEN(8), .MEM_COL_WIDTH(10),
        .PORT_MODE("BI_MODE"), .DATA_PATTERN("DGEN_ALL"), .CMD_PATTERN("CGEN_ALL"),
        .DATA_WIDTH(256), .ADDR_WIDTH(28), .MASK_SIZE(32),
        .BEGIN_ADDRESS(32'h0000_0000), .END_ADDRESS(END_ADDR), .PRBS_EADDR_MASK_POS(~END_ADDR),   // mask = the leading zeros
        .DATA_MODE(DATA_MODE), .SEL_VICTIM_LINE(0),
        .CMD_WDT('h3FF), .RD_WDT('h3FF), .WR_WDT('h1FFF), .EYE_TEST("FALSE")
    ) u_tg (
        .clk(clk), .rst(rst), .tg_only_rst(1'b0), .manual_clear_error(1'b0),
        .memc_init_done(calib_done),
        .memc_cmd_full(~app_rdy), .memc_cmd_en(app_en), .memc_cmd_instr(app_cmd), .memc_cmd_bl(), .memc_cmd_addr(tg_addr),
        .memc_wr_en(app_wdf_wren), .memc_wr_end(app_wdf_end), .memc_wr_mask(app_wdf_mask), .memc_wr_data(app_wdf_data),
        .memc_wr_full(~app_wdf_rdy),
        .memc_rd_en(), .memc_rd_data(app_rd_data), .memc_rd_empty(~app_rd_data_valid),
        .qdr_wr_cmd_o(), .qdr_rd_cmd_o(),
        // the example design's defaults (vio_modify_enable 0: the parameters above rule)
        .vio_pause_traffic(1'b0), .vio_modify_enable(1'b0), .vio_data_mode_value(DATA_MODE), .vio_addr_mode_value(3'b011),
        .vio_instr_mode_value(4'b0010), .vio_bl_mode_value(2'b10), .vio_fixed_bl_value(10'd16), .vio_fixed_instr_value(3'b001),
        .vio_data_mask_gen(1'b0), .fixed_addr_i(32'b0), .fixed_data_i(32'b0),
        .simple_data0(32'b0), .simple_data1(32'b0), .simple_data2(32'b0), .simple_data3(32'b0),
        .simple_data4(32'b0), .simple_data5(32'b0), .simple_data6(32'b0), .simple_data7(32'b0),
        .wdt_en_i(1'b1), .bram_cmd_i(39'b0), .bram_valid_i(1'b0), .bram_rdy_o(),
        .cmp_data(cmp_data), .cmp_data_valid(cmp_valid), .cmp_error(cmp_error),
        .wr_data_counts(wr_cnt), .rd_data_counts(rd_cnt), .dq_error_bytelane_cmp(dq_lane_cmp),
        .error(tg_error), .error_status(err_status), .cumlative_dq_lane_error(dq_lane_err),
        .cmd_wdt_err_o(cmd_wdt), .wr_wdt_err_o(wr_wdt), .rd_wdt_err_o(rd_wdt), .mem_pattern_init_done(pat_done));

    // ---------------- counters ----------------
    localparam int MS_CYCLES = OC_MHZ * 1000;
    reg [31:0] ms_tick = 0, uptime_ms = 0, calib_ms = 0, err_cnt = 0, cmd_cnt = 0, rd_beats = 0, rst_cnt = 0;
    reg        err_seen = 1'b0, calib_seen = 1'b0, was_running = 1'b0;
    always @(posedge clk) begin
        if (rst) begin
            ms_tick <= 0; uptime_ms <= 0; calib_ms <= 0; err_cnt <= 0; cmd_cnt <= 0; rd_beats <= 0;
            err_seen <= 1'b0; calib_seen <= 1'b0;
            if (was_running) rst_cnt <= rst_cnt + 1;      // the MIG reset came back after a release: counted, not hidden
            was_running <= 1'b0;
        end else begin
            was_running <= 1'b1;
            if (ms_tick == MS_CYCLES - 1) begin ms_tick <= 0; uptime_ms <= uptime_ms + 1; end
            else ms_tick <= ms_tick + 1;
            if (calib_done && !calib_seen) begin calib_seen <= 1'b1; calib_ms <= uptime_ms; end
            if (cmp_error) begin err_seen <= 1'b1; if (err_cnt != 32'hFFFF_FFFF) err_cnt <= err_cnt + 1; end
            if (app_en && app_rdy) cmd_cnt <= cmd_cnt + 1;
            if (app_rd_data_valid) rd_beats <= rd_beats + 1;
        end
    end
    assign led = {err_seen | tg_error, calib_done};

    // ---------------- status over JTAG (USER1) ----------------
    localparam int NW = 32;
    wire [NW*32-1:0] fields;
    assign fields[ 0*32 +: 32] = 32'hCA11_0000 | 32'(OC_MHZ);                       // layout + the clock it was built for
    assign fields[ 1*32 +: 32] = BUILD_ID;
    assign fields[ 2*32 +: 32] = {16'd0, 4'd0, DATA_MODE, rst, rd_wdt, wr_wdt, cmd_wdt, pat_done, err_seen, tg_error, calib_done};
    assign fields[ 3*32 +: 32] = uptime_ms;                                          // since the MIG released its reset
    assign fields[ 4*32 +: 32] = calib_ms;                                           // ms at which calibration completed
    assign fields[ 5*32 +: 32] = wr_cnt[31:0];
    assign fields[ 6*32 +: 32] = {16'd0, wr_cnt[47:32]};
    assign fields[ 7*32 +: 32] = rd_cnt[31:0];
    assign fields[ 8*32 +: 32] = {16'd0, rd_cnt[47:32]};
    assign fields[ 9*32 +: 32] = err_cnt;                                            // compare-error cycles
    assign fields[10*32 +: 32] = {24'd0, dq_lane_cmp, dq_lane_err};                  // byte lanes: last error / cumulative
    assign fields[11*32 +: 32] = cmd_cnt;                                            // app commands accepted
    assign fields[12*32 +: 32] = rd_beats;                                           // read data beats
    assign fields[13*32 +: 32] = rst_cnt;                                            // MIG resets after the first release
    assign fields[14*32 +: 18*32] = err_status;                                      // words 14..31: the TG's error record
    link_status_jtag #(.FW(NW * 32), .SNAP_CYCLES(100_000)) u_jtag (.clk(clk), .rst(rst), .fields(fields));
endmodule
`default_nettype wire
