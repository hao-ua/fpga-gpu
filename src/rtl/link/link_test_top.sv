// link_test_top — board top for the JM1/JM2 board-to-board link bring-up.
//   ROLE "A": transmits the 64-bit forward lane (JM1 + JM2[29:22]), receives the backward lane (JM2[21:0])
//             (board A = the Ethernet host board)
//   ROLE "B": the reverse (board B = the HDMI board)
// Clocks: 200 MHz osc -> MMCM (VCO 1000 MHz, or 900 for 150 MHz) -> clk_sys 100 MHz, lclk LINK_MHZ,
// lclk90 LINK_MHZ at 90 degrees.
// LED1 = RX lock, LED2 = sticky error (KEY1 clears; fast blink = contention guard tripped).
// UART (115200 8N1) prints the counters once a second; the same fields are readable over JTAG.
`default_nettype none
module link_test_top #(
    parameter [7:0] ROLE         = "A",
    parameter int   LINK_MHZ     = 50,              // 50, 100, 125, 150, 200 (link_div in link_funcs.svh)
    parameter int   GUARD_CYCLES = 10_000_000,
    parameter int   GUARD_TOGGLES= 100_000,
    parameter int   IDLE_TIMEOUT = 10_000_000,
    parameter int   STATUS_CYCLES= 100_000_000,
    parameter int   MMCM_RST_CYCLES = 1_000_000,
    parameter int   TRAIN_WORDS  = 1024,
    parameter int   SETTLE_CYCLES= 64,
    parameter int   RETRY_CYCLES = 100_000,
    parameter int   RELOCK_CYCLES= 100_000,
    parameter int   RETRAIN_WINDOW = 100_000_000,
    parameter int   RETRAIN_DROPS  = 4,
    parameter bit   SIM_SHIFTER  = 1'b0,
    parameter int   GEN_RATE_DIV = 1,               // generator offers a beat every N cycles (bench knob)
    parameter bit   FILL         = 1'b1,            // fill words between packets
    parameter int   UART_DIV     = 868
)(
    input  wire        sys_clk_p, sys_clk_n,
    input  wire        rst_btn_n,
    input  wire        key1_n,
    output wire [1:0]  led,
    output wire        uart_tx,
    inout  wire [31:0] jm1,
    inout  wire [31:0] jm2
);
    `include "link_funcs.svh"
    localparam int D   = link_div(LINK_MHZ);
    localparam int VCO = LINK_MHZ * D;                 // 1000 MHz, or 900 for 150 MHz
    // ---------------- clocks ----------------
    wire clk_in, clk_fb, clk_fb_buf, clk_sys_u, lclk_u, lclk90_u, lclk270_u, clk_sys, lclk, lclk90, lclk270, mmcm_locked;
    IBUFDS u_ibufds (.I(sys_clk_p), .IB(sys_clk_n), .O(clk_in));
    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"), .CLKIN1_PERIOD(5.000), .DIVCLK_DIVIDE(1),
        .CLKFBOUT_MULT_F(VCO / 200.0),
        .CLKOUT0_DIVIDE_F(D * 1.0),  .CLKOUT0_PHASE(0.0),
        .CLKOUT1_DIVIDE(D),          .CLKOUT1_PHASE(90.0),    // forwarded clock launch
        .CLKOUT2_DIVIDE(VCO / 100),  .CLKOUT2_PHASE(0.0),     // 100 MHz
        .CLKOUT3_DIVIDE(D),          .CLKOUT3_PHASE(270.0),   // control-line launch (link_phy_tx)
        .REF_JITTER1(0.010), .STARTUP_WAIT("FALSE")
    ) u_mmcm (
        .CLKIN1(clk_in), .CLKFBIN(clk_fb_buf), .CLKFBOUT(clk_fb), .CLKFBOUTB(),
        .CLKOUT0(lclk_u), .CLKOUT0B(), .CLKOUT1(lclk90_u), .CLKOUT1B(), .CLKOUT2(clk_sys_u), .CLKOUT2B(),
        .CLKOUT3(lclk270_u), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        .LOCKED(mmcm_locked), .PWRDWN(1'b0), .RST(1'b0));
    BUFG u_bfb (.I(clk_fb),    .O(clk_fb_buf));
    BUFG u_b0  (.I(lclk_u),    .O(lclk));
    BUFG u_b1  (.I(lclk90_u),  .O(lclk90));
    BUFG u_b2  (.I(clk_sys_u), .O(clk_sys));
    BUFG u_b3  (.I(lclk270_u), .O(lclk270));

    // ---------------- resets ----------------
    wire rst_async = ~rst_btn_n | ~mmcm_locked;
    (* ASYNC_REG = "TRUE" *) reg [2:0] rst_sys_s = 3'b111, rst_l_s = 3'b111;
    always @(posedge clk_sys or posedge rst_async) if (rst_async) rst_sys_s <= 3'b111; else rst_sys_s <= {rst_sys_s[1:0], 1'b0};
    always @(posedge lclk    or posedge rst_async) if (rst_async) rst_l_s   <= 3'b111; else rst_l_s   <= {rst_l_s[1:0], 1'b0};
    wire rst     = rst_sys_s[2];
    wire rst_lclk= rst_l_s[2];

    reg [1:0] key_s = 2'b11;
    always @(posedge clk_sys) key_s <= {key_s[0], key1_n};
    wire key_pressed = ~key_s[1];

    // ---------------- status tick (1 s) ----------------
    reg [31:0] tick_cnt = 0; reg tick = 0; reg [31:0] uptime = 0;
    always @(posedge clk_sys) begin
        tick <= 0;
        if (rst) begin tick_cnt <= 0; uptime <= 0; end
        else if (tick_cnt == STATUS_CYCLES - 1) begin tick_cnt <= 0; tick <= 1; uptime <= uptime + 1; end
        else tick_cnt <= tick_cnt + 1;
    end

    // ---------------- link ----------------
    wire        tx_valid, tx_cnt2, tx_last, tx_ready; wire [63:0] tx_data;
    wire        rx_valid, rx_cnt2, rx_last, rx_hdr, pkt_done, pkt_ok; wire [63:0] rx_data;
    wire        lock, trained, rx_clk_ok, far_up, training, guard_fail, tx_enabled;
    wire [7:0]  eye_width, phase, train_cnt;
    wire [31:0] xoff_events, tx_pkts, rx_words, rx_pkts, par_errs, crc_errs, seq_errs, frm_errs, lock_drops, par_map;
    wire [31:0] retx, disc;
    wire [31:0] prbs_errs, payload_words, bit_err_map;

    // A feeds the 64-bit forward lane two words per beat; B feeds the 32-bit backward lane one word per beat
    link_test_gen #(.WPB((ROLE == "A") ? 2 : 1), .RATE_DIV(GEN_RATE_DIV)) u_gen (.clk(clk_sys), .rst(rst), .enable(tx_enabled),
        .tx_valid(tx_valid), .tx_data(tx_data), .tx_cnt2(tx_cnt2), .tx_last(tx_last), .tx_ready(tx_ready));

    link_core #(.ROLE(ROLE), .LINK_MHZ(LINK_MHZ), .MMCM_RST_CYCLES(MMCM_RST_CYCLES), .GUARD_CYCLES(GUARD_CYCLES), .GUARD_TOGGLES(GUARD_TOGGLES), .IDLE_TIMEOUT(IDLE_TIMEOUT),
                .TRAIN_WORDS(TRAIN_WORDS), .SETTLE_CYCLES(SETTLE_CYCLES), .RETRY_CYCLES(RETRY_CYCLES), .RELOCK_CYCLES(RELOCK_CYCLES),
                .RETRAIN_WINDOW(RETRAIN_WINDOW), .RETRAIN_DROPS(RETRAIN_DROPS), .SIM_SHIFTER(SIM_SHIFTER)) u_link (
        .clk(clk_sys), .rst(rst), .lclk(lclk), .lclk90(lclk90), .lclk270(lclk270), .rst_lclk(rst_lclk), .fill_en(FILL),
        .tx_valid, .tx_data, .tx_cnt2, .tx_last, .tx_ready,
        .rx_valid, .rx_data, .rx_cnt2, .rx_last, .rx_hdr, .rx_ready(1'b1), .pkt_done, .pkt_ok,
        .lock, .trained, .rx_clk_ok, .far_up, .training, .guard_fail, .tx_enabled, .xoff_events,
        .eye_width, .phase, .train_cnt,
        .tx_pkts, .rx_words, .rx_pkts, .par_errs, .crc_errs, .seq_errs, .frm_errs, .lock_drops, .par_map, .retx, .disc,
        .diag(), .jm1(jm1), .jm2(jm2));

    link_test_chk u_chk (.clk(clk_sys), .rst(rst), .rx_valid(rx_valid), .rx_data(rx_data), .rx_cnt2(rx_cnt2), .rx_hdr(rx_hdr),
        .prbs_errs(prbs_errs), .payload_words(payload_words), .bit_err_map(bit_err_map));

    // ---------------- LEDs / sticky error ----------------
    reg [31:0] err_sum_d = 0; reg err_sticky = 0; reg [31:0] blink_cnt = 0;
    wire [31:0] err_sum = par_errs + crc_errs + seq_errs + frm_errs + prbs_errs;
    always @(posedge clk_sys) begin
        err_sum_d <= err_sum;
        blink_cnt <= blink_cnt + 1;
        if (rst | key_pressed) err_sticky <= 0;
        else if (err_sum != err_sum_d) err_sticky <= 1;
    end
    assign led[0] = lock;
    assign led[1] = guard_fail ? blink_cnt[23] : err_sticky;   // ~6 Hz blink @100 MHz if the guard tripped

    // ---------------- status UART / JTAG ----------------
    localparam [7:0] LC0 = 8'h30 + ((LINK_MHZ / 100) % 10);
    localparam [7:0] LC1 = 8'h30 + ((LINK_MHZ / 10) % 10);
    localparam [7:0] LC2 = 8'h30 + (LINK_MHZ % 10);
    localparam int TLEN = 288;      // >= template length; leading zero bytes are skipped
    localparam [8*TLEN-1:0] TMPL = {"ROLE=", ROLE, " LCLK=", LC0, LC1, LC2,
        " LOCK=# RXCLK=# UP=# GUARD=# TXEN=# TRN=## EYE=## PH=## TXP=######## RXW=######## RXP=######## PAR=######## CRC=######## PRBS=######## SEQ=######## FRM=######## XOFF=######## DROP=######## PARMAP=######## RETX=######## DISC=######## BITERR=######## T=########",
        8'h0D, 8'h0A};
    // field layout (tools/link_status_jtag.tcl must match): LOCK nibble = {2'b0, trained, lock},
    // UP nibble = {2'b0, training, far_up}; RETX = {timeouts, rewinds}, DISC = {naks, discarded}
    localparam int FW = 20 + 3*8 + 13*32 + 32 + 32;   // 524
    wire [FW-1:0] fields = {2'd0, trained, lock, 3'd0, rx_clk_ok, 2'd0, training, far_up, 3'd0, guard_fail, 3'd0, tx_enabled,
                            train_cnt, eye_width, phase,
                            tx_pkts, rx_words, rx_pkts, par_errs, crc_errs, prbs_errs, seq_errs, frm_errs,
                            xoff_events, lock_drops, par_map, retx, disc, bit_err_map, uptime};
    link_status_uart #(.UART_DIV(UART_DIV), .TLEN(TLEN), .TMPL(TMPL), .FW(FW)) u_status (
        .clk(clk_sys), .rst(rst), .tick(tick), .fields(fields), .txd(uart_tx));
    link_status_jtag #(.FW(FW), .SNAP_CYCLES(1_000_000)) u_jtag (.clk(clk_sys), .rst(rst), .fields(fields));
endmodule
`default_nettype wire
