// board_ab_core — boards A and B in one FPGA, one DDR3: board_a_core (Ethernet, protocol, dispatcher, geometry) and
// board_b_core (command processor, rendering, scanout) with their memory ports on ONE arbiter and their link ports joined
// by link_loop. Everything except the MIG / clock wrapper, so benches can attach a MIG model; clk_sys = MIG ui_clk.
//   The host protocol is the two-board one: the host talks to board A's Ethernet side, B's commands reach B's command
//   processor through link_a_port -> link_loop -> link_b_port, B's EVENTs / READ_DATA / STATUS come back the same way.
//   DDR3: ONE address space for both cores, as the two boards' address spaces were (word addresses, 4-byte units):
//   board A's fixed regions at the bottom (the ingress ring 0-64 MB, records and bin lists up to 256 MB, REC_* / BINLIST_*
//   / POOL_W below), B's draw buckets at 448-464 MB (B3D_DS_BASE, 16 MB), the rest the host's, which keeps A's buffers
//   and B's images apart (host heaps: A 256-448 MB, B 464 MB to 1 GB).
//   Arbiter ports, highest priority first: R0 B scanout (MUST_RUN), R1 A ingress ring, R2 A transfer engine, R3 B READ /
//   COPY, R4 B xfer_lin, R5-R7 B band load / TMU / buckets, R8-R10 A geometry; W0 A ring, W1 A transfer engine, W2-W3 A
//   geometry, W4 B WRITE_IMAGE, W5 B xfer_lin, W6 B FILL / COPY, W7 B band flush.
//   JTAG: board A's status register (USER1); board B's JTAG registers are off (A's STATUS carries B's words).
`default_nettype none
module board_ab_core #(
    parameter [31:0] BUILD_ID       = 32'd0,
    parameter [4:0]  IDELAY_DEFAULT = 5'd0,
    parameter bit    RXDLY_DEFAULT  = 1'b0,
    parameter int    CLK_HZ         = 100_000_000,
    // board A's geometry regions (word addresses): ring 0-64 MB (board_a_core), records 64-192 MB (two banks of 64 MB),
    // bin lists 192-256 MB (two pools of 32 MB)
    parameter [28:0] REC_BASE       = 29'h1000000,
    parameter [28:0] REC_BANK_W     = 29'h1000000,
    parameter [28:0] BINLIST_BASE   = 29'h3000000,
    parameter [28:0] POOL_W         = 29'h800000,
    // Resolution: render targets and video modes up to B3D_MAXW / DISP_MAXW wide (800 x 600 is the largest mode; wider
    // modes are refused, a wider target's FLUSH too). The band height, one rule for both halves: targets up to BAND8_MAXW
    // wide get B3D_NLMAX-row bands, wider ones B3D_NLINES (BAND8_MAXW 0: B3D_NLINES always); the band buffer's B3D_CELLS
    // per bank hold the larger of the two (2 048: 8 rows up to 512 wide, 4 rows up to 1024)
    parameter int    BAND8_MAXW     = 512,
    parameter int    B3D_NLMAX      = 8,
    parameter int    B3D_NLINES     = 4,
    parameter int    B3D_MAXW       = 800,
    parameter int    B3D_CELLS      = 2048,
    parameter int    DISP_MAXW      = 800,
    parameter int    DISP_LB_LINES  = 8,
    // warps in flight and lanes per warp: the vertex core (board A's geometry stage) and the fragment core
    parameter int    VS_NWARP       = 4,
    parameter int    B3D_NWARP      = 4,
    parameter int    B3D_NL         = 4,                // 16, 8 or 4 (a warp = B3D_NL / 4 quads)
    // B's draw buckets (word address): 448 MB, above board A's regions and below the host's B heap
    parameter [28:0] B3D_DS_BASE    = 29'h0700_0000,
    parameter bit    B3D_ROP_BLEND1 = 1'b1,             // one blend unit shared by a pair's two pixels (the same rate)
    // both shader cores' SFU tables shared by pairs of units (block RAM for nothing, bit-identical)
    parameter bit    SFU_TAB2       = 1'b1,
    // both shader cores' integer dividers: none (the compilers lower integer division to float ops; STATUS says so)
    parameter bit    INT_DIV        = 1'b0,
    // both shader cores' SIN / COS units: none (the compilers lower sin / cos to a range reduction and a polynomial;
    // STATUS says so: bit 31 of board A's flags and of board B's; bit 30 there = no integer divider)
    parameter bit    SINCOS         = 1'b0,
    // the fragment core's optional work-savers and queues, smaller than the two boards': SQRT / RCPM / RSQM as one
    // micro-op off (STATUS flags bit 28 clear: the compilers emit RSQRT + MUL etc.), 2 quarter-rate micro-ops in flight
    // per class, 4 TEX instructions live, 8 triangle slots
    parameter bit    B3D_XSFU       = 1'b0,
    parameter int    B3D_QRINF      = 2,
    parameter int    B3D_TEXQ       = 4,
    parameter int    B3D_NTS        = 8,
    // board A's geometry profile counters (STATUS words 101-113 read 0)
    parameter bit    GEOM_PROF      = 1'b0,
    parameter bit    BRINGUP        = 1'b0,
    parameter bit    SIM_MODE4      = 1'b0,
    parameter int    RO_AW          = 6,                // the shared arbiter's read-owner FIFO: 2**RO_AW reads in flight
    // the cores' timers and sizes (their defaults; benches shorten them as for the two boards)
    parameter int    RING_LOG2      = 21,
    parameter int    MDIO_RST_MS    = 20, MDIO_WAIT_MS = 100, MDIO_POLL_US = 5000,
    parameter int    STATUS_PERIOD  = CLK_HZ / 10,
    parameter int    TRIAL_CYCLES   = CLK_HZ / 5,
    parameter int    PHY_TRIAL_CYCLES = 8 * CLK_HZ,
    parameter int    SNAP_CYCLES    = CLK_HZ / 1000,
    parameter bit    JTAG_STATUS    = 1'b1,
    parameter int    KEEPALIVE      = CLK_HZ / 1000,
    parameter int    B_ALIVE        = CLK_HZ / 100,
    parameter int    RENDER_TIMEOUT = 3 * CLK_HZ,
    parameter [31:0] B_BUILD_ID     = BUILD_ID,
    parameter int    DEFAULT_MODE   = 1,
    parameter int    IMG_SET        = 0,
    parameter int    DEBOUNCE       = CLK_HZ / 50,
    parameter int    LONG_PRESS     = 6 * (CLK_HZ / 10),
    parameter int    LOCK_TIMEOUT   = CLK_HZ / 50,
    parameter int    MS_CYCLES      = CLK_HZ / 1000,
    parameter int    LSTAT_CYCLES   = CLK_HZ / 1000
)(
    input  wire         clk_sys, rst_sys, calib_done, clk200,
    // Ethernet PHY
    input  wire         rgmii_rxc, rgmii_rx_ctl,
    input  wire [3:0]   rgmii_rxd,
    output wire         rgmii_txc, rgmii_tx_ctl,
    output wire [3:0]   rgmii_txd,
    output wire         phy_rstn, mdc, mdio_o, mdio_oe,
    input  wire         mdio_i,
    input  wire         key1_n,
    // bench command port of board B's bring-up harness
    input  wire         dbg_cmd_valid,
    input  wire [63:0]  dbg_cmd,
    // MIG native port
    output wire         app_en,
    output wire [2:0]   app_cmd,
    output wire [28:0]  app_addr,
    output wire [255:0] app_wdf_data,
    output wire [31:0]  app_wdf_mask,
    output wire         app_wdf_wren, app_wdf_end,
    input  wire         app_rdy, app_wdf_rdy, app_rd_data_valid,
    input  wire [255:0] app_rd_data,
    // HDMI
    output wire         hdmi_clk_p, hdmi_clk_n,
    output wire [2:0]   hdmi_data_p, hdmi_data_n,
    output wire [1:0]   led
);
    // the arbiter's reset: board A's / B's DDR-side reset (both cores register the same rst_sys / calib_done twice)
    reg rst_d1 = 1'b1; (* max_fanout = 40 *) reg rst_ddr = 1'b1;
    always @(posedge clk_sys) begin rst_d1 <= rst_sys | ~calib_done; rst_ddr <= rst_d1; end

    // ---------------- the two cores ----------------
    wire [4:0]   ar_en, ar_rdy, ar_valid; wire [5*29-1:0] ar_addr;
    wire [3:0]   aw_req, aw_en, aw_wren, aw_wend, aw_rdy, aw_wdf_rdy; wire [4*29-1:0] aw_addr;
    wire [4*256-1:0] aw_data; wire [4*32-1:0] aw_mask;
    wire [5:0]   br_en, br_rdy, br_valid; wire [6*29-1:0] br_addr;
    wire [3:0]   bw_req, bw_en, bw_wren, bw_wend, bw_rdy, bw_wdf_rdy; wire [4*29-1:0] bw_addr;
    wire [4*256-1:0] bw_data; wire [4*32-1:0] bw_mask;
    wire [255:0] rd_data;
    wire         la_tx_valid, la_tx_cnt2, la_tx_last, la_tx_ready, la_rx_valid, la_rx_cnt2, la_rx_last, la_rx_hdr, la_rx_ready;
    wire         lb_tx_valid, lb_tx_cnt2, lb_tx_last, lb_tx_ready, lb_rx_valid, lb_rx_cnt2, lb_rx_last, lb_rx_hdr, lb_rx_ready;
    wire [63:0]  la_tx_data, la_rx_data, lb_tx_data, lb_rx_data;
    wire [31:0]  jm_a1, jm_a2, jm_b1, jm_b2;            // the cores' idle board-link headers
    wire [1:0]   led_a, led_b;

    board_a_core #(.BUILD_ID(BUILD_ID), .IDELAY_DEFAULT(IDELAY_DEFAULT), .RXDLY_DEFAULT(RXDLY_DEFAULT), .CLK_HZ(CLK_HZ),
                   .RING_LOG2(RING_LOG2), .MDIO_RST_MS(MDIO_RST_MS), .MDIO_WAIT_MS(MDIO_WAIT_MS), .MDIO_POLL_US(MDIO_POLL_US),
                   .STATUS_PERIOD(STATUS_PERIOD), .TRIAL_CYCLES(TRIAL_CYCLES), .PHY_TRIAL_CYCLES(PHY_TRIAL_CYCLES),
                   .SNAP_CYCLES(SNAP_CYCLES), .JTAG_STATUS(JTAG_STATUS), .KEEPALIVE(KEEPALIVE), .B_ALIVE(B_ALIVE),
                   .RENDER_TIMEOUT(RENDER_TIMEOUT), .VS_SFU_TAB2(SFU_TAB2), .VS_INT_DIV(INT_DIV), .VS_SINCOS(SINCOS), .EXT_ARB(1'b1), .EXT_LINK(1'b1), .REC_BASE(REC_BASE), .REC_BANK_W(REC_BANK_W),
                   .BINLIST_BASE(BINLIST_BASE), .POOL_W(POOL_W), .VS_NWARP(VS_NWARP), .BAND8_MAXW(BAND8_MAXW),
                   .GEOM_PROF(GEOM_PROF), .JTAG_LEAN(1'b1)) u_a (
        .clk_sys, .rst_sys, .calib_done, .clk_idelay(clk200),
        .rgmii_rxc, .rgmii_rx_ctl, .rgmii_rxd, .rgmii_txc, .rgmii_tx_ctl, .rgmii_txd,
        .phy_rstn, .mdc, .mdio_o, .mdio_oe, .mdio_i,
        .app_en(), .app_cmd(), .app_addr(), .app_wdf_data(), .app_wdf_mask(), .app_wdf_wren(), .app_wdf_end(),
        .app_rdy(1'b0), .app_wdf_rdy(1'b0), .app_rd_data_valid(1'b0), .app_rd_data(256'd0),
        .jm1(jm_a1), .jm2(jm_a2), .led(led_a),
        .xr_en(ar_en), .xr_addr(ar_addr), .xr_rdy(ar_rdy), .xr_valid(ar_valid), .xrd_data(rd_data),
        .xw_req(aw_req), .xw_en(aw_en), .xw_wren(aw_wren), .xw_wend(aw_wend), .xw_addr(aw_addr), .xw_data(aw_data),
        .xw_mask(aw_mask), .xw_rdy(aw_rdy), .xw_wdf_rdy(aw_wdf_rdy),
        .xl_tx_valid(la_tx_valid), .xl_tx_data(la_tx_data), .xl_tx_cnt2(la_tx_cnt2), .xl_tx_last(la_tx_last),
        .xl_tx_ready(la_tx_ready), .xl_rx_valid(la_rx_valid), .xl_rx_data(la_rx_data), .xl_rx_cnt2(la_rx_cnt2),
        .xl_rx_last(la_rx_last), .xl_rx_hdr(la_rx_hdr), .xl_rx_ready(la_rx_ready));

    board_b_core #(.BUILD_ID(B_BUILD_ID), .CLK_HZ(CLK_HZ), .BRINGUP(BRINGUP), .SIM_MODE4(SIM_MODE4), .JTAG(1'b0),
                   .DEFAULT_MODE(DEFAULT_MODE), .IMG_SET(IMG_SET), .DEBOUNCE(DEBOUNCE), .LONG_PRESS(LONG_PRESS),
                   .LOCK_TIMEOUT(LOCK_TIMEOUT), .MS_CYCLES(MS_CYCLES), .LSTAT_CYCLES(LSTAT_CYCLES),
                   .B3D_NLINES(B3D_NLINES), .B3D_MAXW(B3D_MAXW), .B3D_CELLS(B3D_CELLS), .B3D_NWARP(B3D_NWARP),
                   .B3D_NTS(B3D_NTS), .B3D_NLMAX(B3D_NLMAX), .B3D_BAND8_MAXW(BAND8_MAXW), .DISP_MAXW(DISP_MAXW),
                   .DISP_LB_LINES(DISP_LB_LINES), .B3D_SFU_TAB2(SFU_TAB2), .B3D_INT_DIV(INT_DIV), .B3D_SINCOS(SINCOS), .B3D_XSFU(B3D_XSFU),
                   .B3D_QRINF(B3D_QRINF), .B3D_TEXQ(B3D_TEXQ), .B3D_NL(B3D_NL), .B3D_DS_BASE(B3D_DS_BASE),
                   .B3D_ROP_BLEND1(B3D_ROP_BLEND1), .EXT_ARB(1'b1), .EXT_LINK(1'b1)) u_b (
        .clk_sys, .rst_sys, .calib_done, .clk200, .key1_n, .dbg_cmd_valid, .dbg_cmd,
        .app_en(), .app_cmd(), .app_addr(), .app_wdf_data(), .app_wdf_mask(), .app_wdf_wren(), .app_wdf_end(),
        .app_rdy(1'b0), .app_wdf_rdy(1'b0), .app_rd_data_valid(1'b0), .app_rd_data(256'd0),
        .hdmi_clk_p, .hdmi_clk_n, .hdmi_data_p, .hdmi_data_n,
        .jm1(jm_b1), .jm2(jm_b2), .led(led_b), .status_vec(),
        .xr_en(br_en), .xr_addr(br_addr), .xr_rdy(br_rdy), .xr_valid(br_valid), .xrd_data(rd_data),
        .xw_req(bw_req), .xw_en(bw_en), .xw_wren(bw_wren), .xw_wend(bw_wend), .xw_addr(bw_addr), .xw_data(bw_data),
        .xw_mask(bw_mask), .xw_rdy(bw_rdy), .xw_wdf_rdy(bw_wdf_rdy),
        .xl_tx_valid(lb_tx_valid), .xl_tx_data(lb_tx_data), .xl_tx_cnt2(lb_tx_cnt2), .xl_tx_last(lb_tx_last),
        .xl_tx_ready(lb_tx_ready), .xl_rx_valid(lb_rx_valid), .xl_rx_data(lb_rx_data), .xl_rx_cnt2(lb_rx_cnt2),
        .xl_rx_last(lb_rx_last), .xl_rx_hdr(lb_rx_hdr), .xl_rx_ready(lb_rx_ready));

    // ---------------- the board link inside the chip ----------------
    link_loop u_loop (.clk(clk_sys), .rst(rst_ddr),
        .a_tx_valid(la_tx_valid), .a_tx_data(la_tx_data), .a_tx_cnt2(la_tx_cnt2), .a_tx_last(la_tx_last), .a_tx_ready(la_tx_ready),
        .a_rx_valid(la_rx_valid), .a_rx_data(la_rx_data), .a_rx_cnt2(la_rx_cnt2), .a_rx_last(la_rx_last), .a_rx_hdr(la_rx_hdr),
        .a_rx_ready(la_rx_ready),
        .b_tx_valid(lb_tx_valid), .b_tx_data(lb_tx_data), .b_tx_cnt2(lb_tx_cnt2), .b_tx_last(lb_tx_last), .b_tx_ready(lb_tx_ready),
        .b_rx_valid(lb_rx_valid), .b_rx_data(lb_rx_data), .b_rx_cnt2(lb_rx_cnt2), .b_rx_last(lb_rx_last), .b_rx_hdr(lb_rx_hdr),
        .b_rx_ready(lb_rx_ready));

    // ---------------- one arbiter for both cores ----------------
    wire [10:0] r_en, r_rdy, r_valid; wire [11*29-1:0] r_addr;
    wire [7:0]  w_req, w_en, w_wren, w_wend, w_rdy, w_wdf_rdy; wire [8*29-1:0] w_addr;
    wire [8*256-1:0] w_data; wire [8*32-1:0] w_mask;
    // read ports: {A geometry R2-R4, B R3-R5, B R2, B R1, A R1, A R0, B R0} = 10 .. 0
    assign r_en   = {ar_en[4:2], br_en[5:3], br_en[2], br_en[1], ar_en[1], ar_en[0], br_en[0]};
    assign r_addr = {ar_addr[4*29 +: 29], ar_addr[3*29 +: 29], ar_addr[2*29 +: 29],
                     br_addr[5*29 +: 29], br_addr[4*29 +: 29], br_addr[3*29 +: 29], br_addr[2*29 +: 29], br_addr[1*29 +: 29],
                     ar_addr[1*29 +: 29], ar_addr[0*29 +: 29], br_addr[0*29 +: 29]};
    assign {ar_rdy[4:2], br_rdy[5:3], br_rdy[2], br_rdy[1], ar_rdy[1], ar_rdy[0], br_rdy[0]} = r_rdy;
    assign {ar_valid[4:2], br_valid[5:3], br_valid[2], br_valid[1], ar_valid[1], ar_valid[0], br_valid[0]} = r_valid;
    // write ports: {B W3-W0, A W3-W0} = 7 .. 0
    assign w_req  = {bw_req, aw_req};
    assign w_en   = {bw_en, aw_en};
    assign w_wren = {bw_wren, aw_wren};
    assign w_wend = {bw_wend, aw_wend};
    assign w_data = {bw_data, aw_data};
    assign w_mask = {bw_mask, aw_mask};
    assign w_addr = {bw_addr, aw_addr};
    assign {bw_rdy, aw_rdy} = w_rdy;
    assign {bw_wdf_rdy, aw_wdf_rdy} = w_wdf_rdy;

    mem_arb #(.NR(11), .NW(8), .RO_AW(RO_AW), .RO_RESERVE(4), .MUST_RUN(11'b000_0000_0001)) u_arb (
        .clk(clk_sys), .rst(rst_ddr), .r_en(r_en), .r_addr(r_addr), .r_rdy(r_rdy), .r_valid(r_valid), .rd_data(rd_data),
        .w_req(w_req), .w_en(w_en), .w_addr(w_addr), .w_data(w_data), .w_mask(w_mask), .w_wren(w_wren),
        .w_wend(w_wend), .w_rdy(w_rdy), .w_wdf_rdy(w_wdf_rdy),
        .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end),
        .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy), .app_rd_data_valid(app_rd_data_valid), .app_rd_data(app_rd_data));

    // synthesis translate_off
    // the shared address space's fixed regions: board B never writes below the end of board A's (the ring, the record
    // banks, the bin pools); A's geometry writes (W2, W3) stay inside its record banks and bin pools
    localparam [28:0] A_END = BINLIST_BASE + 2 * POOL_W;
    always @(posedge clk_sys) if (!rst_ddr) for (int i = 0; i < 8; i++) if (w_en[i] && w_rdy[i]) begin
        if (i >= 4 && w_addr[i*29 +: 29] < A_END)
            $fatal(1, "board_ab_core: board B write (port W%0d) at word %h, inside board A's regions (below %h)", i, w_addr[i*29 +: 29], A_END);
        if ((i == 2 || i == 3) && (w_addr[i*29 +: 29] < REC_BASE || w_addr[i*29 +: 29] >= A_END))
            $fatal(1, "board_ab_core: board A geometry write (port W%0d) at word %h, outside [%h, %h)", i, w_addr[i*29 +: 29], REC_BASE, A_END);
    end
    initial if (B3D_DS_BASE < A_END) $fatal(1, "board_ab_core: B3D_DS_BASE %h inside board A's regions (below %h)", B3D_DS_BASE, A_END);
    // synthesis translate_on

    // LEDs: DDR calibrated, Ethernet link up (board A's)
    assign led = led_a;
    wire _unused = &{1'b0, led_b, jm_a1, jm_a2, jm_b1, jm_b2};
endmodule
`default_nettype wire
