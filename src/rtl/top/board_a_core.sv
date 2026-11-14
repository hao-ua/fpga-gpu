// board_a_core — board A: everything except
// the MIG/clock wrapper, so benches can attach a MIG model. clk_sys = MIG ui_clk (100 MHz).
//   eth_stack -> fb_proto_a -> ddr_fifo (ingress ring, mem_arb R0/W0) -> fb_dispatch_a -> xfer_lin (R1/W1)
//   replies: fb_proto_a (ACK, PHY/REG, STATUS) + fb_dispatch_a (EVENT, READ_DATA) + link_a_port (B's EVENTs and READ_DATA)
//            -> fb_reply -> eth_stack
// Board link: link_core role A on JM1/JM2; fb_dispatch_a forwards B commands as CMD packets
// through link_a_port, which also sends the IDLE keep-alive and receives B's STATUS / EVENT packets.
// Status: a STATUS_WORDS vector, sent in STATUS datagrams and readable over JTAG (link_status_jtag,
// tools/a_status_jtag.tcl). DDR-side blocks are held in reset until calibration completes.
`default_nettype none
module board_a_core #(
    parameter [31:0] BUILD_ID       = 32'd0,
    parameter [4:0]  IDELAY_DEFAULT = 5'd0,
    parameter bit    RXDLY_DEFAULT  = 1'b0,
    parameter [47:0] MY_MAC         = 48'h00_0a_35_01_02_03,
    parameter [31:0] MY_IP          = {8'd192, 8'd168, 8'd1, 8'd10},
    parameter int    RING_LOG2      = 21,           // ingress ring: 2**21 bursts = 64 MB at DDR address 0
    parameter int    MDIO_RST_MS    = 20, MDIO_WAIT_MS = 100, MDIO_POLL_US = 5000,
    parameter int    CLK_HZ         = 100_000_000,  // clk_sys (the MIG ui_clk); the millisecond timers below derive from it
    parameter int    STATUS_PERIOD  = CLK_HZ / 10,          // 100 ms
    parameter int    TRIAL_CYCLES   = CLK_HZ / 5,           // 200 ms
    parameter int    PHY_TRIAL_CYCLES = 8 * CLK_HZ,         // 8 s
    parameter int    SNAP_CYCLES    = CLK_HZ / 1000,        // 1 ms
    parameter bit    JTAG_STATUS    = 1'b1,
    parameter bit    JTAG_LEAN      = 1'b0,                 // the JTAG frame read out bit by bit from the snapshot (link_status_jtag LEAN)
    // board link (simulation shortens the start-up times)
    parameter int    LINK_MHZ       = 100,
    parameter int    LINK_GUARD     = 10_000_000,
    parameter int    LINK_MMCM_RST  = 1_000_000,
    parameter int    LINK_IDLE_TO   = 10_000_000,
    parameter int    LINK_TRAIN_WORDS = 1024,
    parameter int    LINK_RETRY     = 100_000,
    parameter int    LINK_RELOCK    = 10_000_000,   // backstop only: relock is counted in words (link_core)
    parameter int    LINK_RWIN      = CLK_HZ,              // 1 s rate window
    parameter int    KEEPALIVE      = CLK_HZ / 1000,       // 1 ms
    parameter int    B_ALIVE        = CLK_HZ / 100,        // 10 ms
    parameter int    B_WAIT         = CLK_HZ / 10,         // 100 ms: a B command waits this long for the board link
    parameter int    RENDER_TIMEOUT = 3 * CLK_HZ,          // 3 s: a FLUSH's frame B has not finished by then is aborted
    parameter bit    LINK_SIM_SHIFTER = 1'b0,
    // one-chip board (board_ab_core): EXT_ARB 1 = the memory ports go out (xr_* / xw_*) to one arbiter shared with board B's
    // core and app_* stay idle; EXT_LINK 1 = link_a_port's link side goes out (xl_*) to link_loop and the board link is idle
    // (both instances stay, their outputs unused). Then the geometry stage's DDR regions (word addresses): records at REC_BASE
    // in two banks of REC_BANK_W words, bin lists at BINLIST_BASE in two pools of POOL_W words
    parameter bit    EXT_ARB        = 1'b0,
    parameter bit    EXT_LINK       = 1'b0,
    parameter [28:0] REC_BASE       = 29'h1000000,
    parameter [28:0] REC_BANK_W     = 29'h2000000,
    parameter [28:0] BINLIST_BASE   = 29'h5000000,
    parameter [28:0] POOL_W         = 29'h800000,
    // the geometry stage: the vertex core's warps in flight (2..4), and the widest target that gets 8-row bands (wider ones
    // get 4; it must be board B's b_render BAND8_MAXW; 0 = 4-row bands always)
    parameter int    VS_NWARP       = 4,
    parameter int    VS_NL          = 16,           // the vertex core's lanes per warp (16, 8 or 4; the same results, fewer LUTs)
    parameter int    BAND8_MAXW     = 1024,
    parameter bit    VS_SFU_TAB2    = 1'b0,         // the vertex core's SFU tables shared by pairs of units (bit-identical)
    parameter bit    VS_INT_DIV     = 1'b1,         // the vertex core's integer divider; 0: none (STATUS flags bit 31 set: the
                                                    // compiler lowers integer division to float ops)
    parameter bit    VS_SINCOS      = 1'b1,         // the vertex core's SIN / COS units; 0: none (STATUS flags bit 30 set: the
                                                    // compiler lowers sin / cos to a range reduction and a polynomial)
    parameter bit    GEOM_PROF      = 1'b1          // the geometry stage's profile counters (STATUS words 101-113; 0: they read 0)
)(
    input  wire         clk_sys, rst_sys, calib_done, clk_idelay,
    // Ethernet PHY
    input  wire         rgmii_rxc, rgmii_rx_ctl,
    input  wire [3:0]   rgmii_rxd,
    output wire         rgmii_txc, rgmii_tx_ctl,
    output wire [3:0]   rgmii_txd,
    output wire         phy_rstn, mdc, mdio_o, mdio_oe,
    input  wire         mdio_i,
    // MIG native port
    output wire         app_en,
    output wire [2:0]   app_cmd,
    output wire [28:0]  app_addr,
    output wire [255:0] app_wdf_data,
    output wire [31:0]  app_wdf_mask,
    output wire         app_wdf_wren, app_wdf_end,
    input  wire         app_rdy, app_wdf_rdy, app_rd_data_valid,
    input  wire [255:0] app_rd_data,
    // board link headers
    inout  wire [31:0]  jm1, jm2,
    output wire [1:0]   led,
    // EXT_ARB: the arbiter's ports (R0 ingress ring, R1 transfer engine, R2-R4 geometry; W0 ring, W1 transfer engine,
    // W2-W3 geometry), mem_arb's contract
    output wire [4:0]       xr_en,
    output wire [5*29-1:0]  xr_addr,
    input  wire [4:0]       xr_rdy, xr_valid,
    input  wire [255:0]     xrd_data,
    output wire [3:0]       xw_req, xw_en, xw_wren, xw_wend,
    output wire [4*29-1:0]  xw_addr,
    output wire [4*256-1:0] xw_data,
    output wire [4*32-1:0]  xw_mask,
    input  wire [3:0]       xw_rdy, xw_wdf_rdy,
    // EXT_LINK: link_a_port's side of the link, link_core's user contract
    output wire             xl_tx_valid,
    output wire [63:0]      xl_tx_data,
    output wire             xl_tx_cnt2, xl_tx_last,
    input  wire             xl_tx_ready,
    input  wire             xl_rx_valid,
    input  wire [63:0]      xl_rx_data,
    input  wire             xl_rx_cnt2, xl_rx_last, xl_rx_hdr,
    output wire             xl_rx_ready
);
    import fb_pkg::*;
    localparam int RF_AW = 11;

    // Resets, registered twice: the MIG's reset register sits beside the DDR banks while the users are spread over the
    // whole die (up to 9.5 ns of routing); the second stage may be replicated (max_fanout).
    // rst_d1 replicated by synthesis: alone it would drive ~70 replicas of rst_ddr spread over the die (about 7.8 ns of
    // route on 0 logic levels, too much at 120 MHz)
    reg rst_s1 = 1'b1; (* max_fanout = 12 *) reg rst_d1 = 1'b1;
    (* max_fanout = 40 *) reg rst_s = 1'b1, rst_ddr = 1'b1;
    always @(posedge clk_sys) begin
        rst_s1 <= rst_sys; rst_d1 <= rst_sys | ~calib_done;
        rst_s  <= rst_s1;  rst_ddr <= rst_d1;
    end

    // ---------------- Ethernet ----------------
    wire        pf_v, pf_ren; wire [35:0] pf_d;
    wire        rf_wen; wire [35:0] rf_wd; wire [RF_AW:0] rf_wcount;
    wire [4:0]  idelay_tap, phyad; wire idelay_ld, rxdly, adv1000, reinit; wire [31:0] linkctl;
    wire        phy_req, phy_wr, phy_resp; wire [15:0] phy_page, phy_wdata, phy_rdata; wire [4:0] phy_reg;
    wire        mdio_init_done, link_up, duplex; wire [1:0] speed; wire [15:0] physr, bmsr; wire [31:0] mdio_polls;
    wire [32*14-1:0] cnt; wire [31:0] snap_age;
    eth_stack #(.MY_MAC(MY_MAC), .MY_IP(MY_IP), .IDELAY_DEFAULT(IDELAY_DEFAULT), .RF_AW(RF_AW), .CLK_HZ(CLK_HZ),
                .SNAP_CYCLES(SNAP_CYCLES), .MDIO_RST_MS(MDIO_RST_MS), .MDIO_WAIT_MS(MDIO_WAIT_MS),
                .MDIO_POLL_US(MDIO_POLL_US)) u_eth (
        .clk_sys(clk_sys), .rst_sys(rst_s), .clk_idelay(clk_idelay),
        .rgmii_rxc(rgmii_rxc), .rgmii_rx_ctl(rgmii_rx_ctl), .rgmii_rxd(rgmii_rxd),
        .rgmii_txc(rgmii_txc), .rgmii_tx_ctl(rgmii_tx_ctl), .rgmii_txd(rgmii_txd),
        .phy_rstn(phy_rstn), .mdc(mdc), .mdio_o(mdio_o), .mdio_oe(mdio_oe), .mdio_i(mdio_i),
        .pf_rvalid(pf_v), .pf_rdata(pf_d), .pf_ren(pf_ren),
        .rf_wen(rf_wen), .rf_wdata(rf_wd), .rf_wcount(rf_wcount),
        .dly_ld(idelay_ld), .dly_val(idelay_tap), .phyad(phyad), .rxdly(rxdly), .adv1000(adv1000), .reinit(reinit),
        .phy_req(phy_req), .phy_req_wr(phy_wr), .phy_req_page(phy_page), .phy_req_reg(phy_reg),
        .phy_req_wdata(phy_wdata), .phy_resp(phy_resp), .phy_resp_rdata(phy_rdata),
        .mdio_init_done(mdio_init_done), .link_up(link_up), .duplex(duplex), .speed(speed),
        .physr(physr), .bmsr(bmsr), .mdio_polls(mdio_polls), .cnt_snap(cnt), .snap_age(snap_age));

    // ---------------- memory arbiter: R0/W0 ingress ring, R1/W1 transfer engine, R2/R3/R4 + W2/W3 the geometry stage
    //                  (vertex fetch, program load, the render side's bin lists / records; triangle records, bin lists)
    //                  ----------------
    wire [4:0] r_en, r_rdy, r_valid; wire [5*29-1:0] r_addr; wire [255:0] rd_data;
    wire [3:0] w_req, w_en, w_wren, w_wend, w_rdy, w_wdf_rdy; wire [4*29-1:0] w_addr;
    wire [4*256-1:0] w_data; wire [4*32-1:0] w_mask;
    wire [4:0] m_r_rdy, m_r_valid; wire [255:0] m_rd_data; wire [3:0] m_w_rdy, m_w_wdf_rdy;
    wire m_app_en, m_app_wdf_wren, m_app_wdf_end; wire [2:0] m_app_cmd; wire [28:0] m_app_addr;
    wire [255:0] m_app_wdf_data; wire [31:0] m_app_wdf_mask;
    mem_arb #(.NR(5), .NW(4), .RO_AW(5), .RO_RESERVE(4), .MUST_RUN(5'b00000)) u_arb (
        .clk(clk_sys), .rst(rst_ddr), .r_en(r_en), .r_addr(r_addr), .r_rdy(m_r_rdy), .r_valid(m_r_valid), .rd_data(m_rd_data),
        .w_req(w_req), .w_en(w_en), .w_addr(w_addr), .w_data(w_data), .w_mask(w_mask), .w_wren(w_wren),
        .w_wend(w_wend), .w_rdy(m_w_rdy), .w_wdf_rdy(m_w_wdf_rdy),
        .app_en(m_app_en), .app_cmd(m_app_cmd), .app_addr(m_app_addr), .app_wdf_data(m_app_wdf_data),
        .app_wdf_mask(m_app_wdf_mask), .app_wdf_wren(m_app_wdf_wren), .app_wdf_end(m_app_wdf_end),
        .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy), .app_rd_data_valid(app_rd_data_valid), .app_rd_data(app_rd_data));
    // EXT_ARB: the ports go to the shared arbiter instead; u_arb's outputs are then unused (and synthesis drops it)
    assign r_rdy   = EXT_ARB ? xr_rdy     : m_r_rdy;
    assign r_valid = EXT_ARB ? xr_valid   : m_r_valid;
    assign rd_data = EXT_ARB ? xrd_data   : m_rd_data;
    assign w_rdy   = EXT_ARB ? xw_rdy     : m_w_rdy;
    assign w_wdf_rdy = EXT_ARB ? xw_wdf_rdy : m_w_wdf_rdy;
    assign app_en       = EXT_ARB ? 1'b0 : m_app_en;
    assign app_cmd      = EXT_ARB ? 3'd0 : m_app_cmd;
    assign app_addr     = EXT_ARB ? 29'd0 : m_app_addr;
    assign app_wdf_data = EXT_ARB ? 256'd0 : m_app_wdf_data;
    assign app_wdf_mask = EXT_ARB ? 32'd0 : m_app_wdf_mask;
    assign app_wdf_wren = EXT_ARB ? 1'b0 : m_app_wdf_wren;
    assign app_wdf_end  = EXT_ARB ? 1'b0 : m_app_wdf_end;
    assign xr_en = r_en; assign xr_addr = r_addr;
    assign xw_req = w_req; assign xw_en = w_en; assign xw_addr = w_addr; assign xw_data = w_data; assign xw_mask = w_mask;
    assign xw_wren = w_wren; assign xw_wend = w_wend;

    // ---------------- ingress queue ----------------
    wire         qi_v, qi_rdy, qo_v, qo_rdy; wire [255:0] qi_d, qo_d; wire [RING_LOG2:0] q_used;
    ddr_fifo #(.BASE(29'h0), .RING_LOG2(RING_LOG2)) u_q (
        .clk(clk_sys), .rst(rst_ddr), .in_valid(qi_v), .in_data(qi_d), .in_ready(qi_rdy),
        .out_valid(qo_v), .out_data(qo_d), .out_ready(qo_rdy), .used(q_used),
        .w_req(w_req[0]), .w_en(w_en[0]), .w_addr(w_addr[28:0]), .w_data(w_data[255:0]), .w_mask(w_mask[31:0]),
        .w_wren(w_wren[0]), .w_wend(w_wend[0]), .w_rdy(w_rdy[0]), .w_wdf_rdy(w_wdf_rdy[0]),
        .r_en(r_en[0]), .r_addr(r_addr[28:0]), .r_rdy(r_rdy[0]), .r_valid(r_valid[0]), .rd_data(rd_data));

    // ---------------- protocol engine ----------------
    localparam int NS = 7;
    wire [NS-1:0] s_req, s_valid, s_ready, s_done; wire [NS*16-1:0] s_len; wire [NS*32-1:0] s_data;
    wire        host_valid; wire [47:0] host_mac; wire [31:0] host_ip; wire [15:0] host_port;
    wire [31:0] exp_seq, c_acc, c_dup, c_gap, c_full, c_bad, c_ack;
    wire [STATUS_WORDS*32-1:0] status_vec;
    wire        cur_valid; wire [31:0] cur_pos;      // cursor fast path
    fb_proto_a #(.RING_LOG2(RING_LOG2), .STATUS_PERIOD(STATUS_PERIOD), .TRIAL_CYCLES(TRIAL_CYCLES),
                 .PHY_TRIAL_CYCLES(PHY_TRIAL_CYCLES),
                 .IDELAY_DEFAULT(IDELAY_DEFAULT), .RXDLY_DEFAULT(RXDLY_DEFAULT)) u_proto (
        .clk(clk_sys), .rst(rst_s), .q_ok(calib_done),
        .pf_valid(pf_v), .pf_data(pf_d), .pf_ren(pf_ren),
        .q_valid(qi_v), .q_data(qi_d), .q_ready(qi_rdy), .q_used(q_used),
        .host_valid(host_valid), .host_mac(host_mac), .host_ip(host_ip), .host_port(host_port),
        .r_req(s_req[2:0]), .r_len(s_len[47:0]), .r_valid(s_valid[2:0]), .r_data(s_data[95:0]),
        .r_ready(s_ready[2:0]), .r_done(s_done[2:0]),
        .idelay_tap(idelay_tap), .idelay_ld(idelay_ld), .rxdly(rxdly), .adv1000(adv1000), .phyad(phyad),
        .reinit(reinit), .linkctl(linkctl), .build_id(BUILD_ID),
        .phy_req(phy_req), .phy_wr(phy_wr), .phy_page(phy_page), .phy_reg(phy_reg), .phy_wdata(phy_wdata),
        .phy_resp(phy_resp), .phy_rdata(phy_rdata),
        .cur_valid(cur_valid), .cur_pos(cur_pos),
        .status_vec(status_vec),
        .exp_seq(exp_seq), .c_acc(c_acc), .c_dup(c_dup), .c_gap(c_gap), .c_full(c_full), .c_bad(c_bad), .c_ack(c_ack));

    // ---------------- dispatcher + transfer engine ----------------
    wire        x_cv, x_crdy, x_busy, x_wdv, x_wdr, x_rdv, x_rdr, x_mtd;
    wire [1:0]  x_op, x_flags; wire [29:0] x_addr, x_len, x_mtbad; wire [31:0] x_seed, x_wdd, x_rdd, x_mterr;
    wire [31:0] c_cmds, c_errs, last_err, last_err_seq, mt_last, c_fwd, c_hdr_wait, c_gwait;
    wire        f_valid, f_last, f_ready, link_ok; wire [31:0] f_data; wire [15:0] f_len;
    wire        g_abort, g_abort_soft, g_resync, l_sync_go;
    fb_dispatch_a #(.B_WAIT(B_WAIT), .RENDER_TIMEOUT(RENDER_TIMEOUT)) u_disp (
        .clk(clk_sys), .rst(rst_ddr), .q_valid(qo_v), .q_data(qo_d), .q_ready(qo_rdy),
        .x_cmd_valid(x_cv), .x_op(x_op), .x_addr(x_addr), .x_len(x_len), .x_seed(x_seed), .x_flags(x_flags),
        .x_cmd_ready(x_crdy), .x_busy(x_busy), .x_wd_valid(x_wdv), .x_wd_data(x_wdd), .x_wd_ready(x_wdr),
        .x_rd_valid(x_rdv), .x_rd_data(x_rdd), .x_rd_ready(x_rdr),
        .x_mt_done(x_mtd), .x_mt_errors(x_mterr), .x_mt_first_bad(x_mtbad),
        .ev_req(s_req[3]), .ev_len(s_len[63:48]), .ev_valid(s_valid[3]), .ev_data(s_data[127:96]),
        .ev_ready(s_ready[3]), .ev_done(s_done[3]),
        .rd_req(s_req[4]), .rd_len(s_len[79:64]), .rd_valid(s_valid[4]), .rd_data(s_data[159:128]),
        .rd_ready(s_ready[4]), .rd_done(s_done[4]),
        .link_ok(link_ok), .f_valid(f_valid), .f_data(f_data), .f_len(f_len), .f_last(f_last), .f_ready(f_ready),
        .c_cmds(c_cmds), .c_errs(c_errs), .last_err(last_err), .last_err_seq(last_err_seq), .mt_last_errors(mt_last),
        .c_fwd(c_fwd), .c_hdr_wait(c_hdr_wait), .c_gwait(c_gwait),
        .g_busy(g_busy), .g_rbusy(g_rbusy), .g_fb_ok(g_fb_ok), .g_draw_ok(g_draw_ok), .g_copy_busy(g_copy_busy),
        .g_frame_done(g_frame_done), .g_ld_done(g_ld_done), .g_render_done(g_render_done), .g_render(g_render), .g_abort(g_abort),
        .g_abort_soft(g_abort_soft), .g_resync(g_resync), .sync_go(l_sync_go),
        .g_frame(g_frame), .g_ld_start(g_ld_start),
        .g_param_we(g_param_we), .g_ad_we(g_ad_we), .g_ac_we(g_ac_we), .g_st_we(g_st_we), .g_dr_start(g_dr_start),
        .g_ld_addr(g_ld_addr), .g_ad_base(g_ad_base), .g_dr_idx_base(g_dr_idx_base), .g_ld_len(g_ld_len), .g_st_w(g_st_w), .g_st_h(g_st_h),
        .g_param_idx(g_param_idx), .g_st_fsmask(g_st_fsmask), .g_param_data(g_param_data), .g_ac_const(g_ac_const), .g_st_vp(g_st_vp),
        .g_ad_idx(g_ad_idx), .g_ac_idx(g_ac_idx), .g_ad_en(g_ad_en), .g_ad_type(g_ad_type), .g_ad_norm(g_ad_norm), .g_st_cw(g_st_cw), .g_st_z0(g_st_z0), .g_dr_indexed(g_dr_indexed), .g_dr_u8(g_dr_u8),
        .g_ad_size(g_ad_size), .g_dr_mode(g_dr_mode), .g_ad_stride(g_ad_stride), .g_ad_off(g_ad_off), .g_dr_first(g_dr_first),
        .g_dr_count(g_dr_count), .g_st_cull(g_st_cull), .g_st_lw(g_st_lw), .g_st_did(g_st_did), .g_st_z(g_st_z), .g_st_gb(g_st_gb), .g_st_sc(g_st_sc));
    // ---------------- the geometry stage (3D on A): two banks of records and two pools of bin lists (the defaults: records
    //                  from 64 MB, 128 MB a bank; bin lists from 320 MB, 32 MB a pool; the host heap from 384 MB) ----------------
    wire        g_busy, g_rbusy, g_fb_ok, g_draw_ok, g_copy_busy, g_frame_done, g_ld_done, g_frame, g_ld_start, g_param_we, g_ad_we, g_ac_we, g_st_we, g_dr_start, g_err_stack;
    wire        g_rec_ovf, g_bin_ovf; wire [47:0] g_st_z, g_st_sc; wire [95:0] g_st_gb;
    wire        g_render, g_render_done, rs_v, rs_r; wire [63:0] rs_d; wire [31:0] g_c_quads, cr_rs_sent, cr_rs_consumed, c_rs_pkts;
    wire [28:0] g_ld_addr; wire [29:0] g_ad_base, g_dr_idx_base, g_ad_off, g_ad_stride; wire [11:0] g_ld_len, g_st_w, g_st_h; wire [7:0] g_param_idx, g_st_fsmask;
    wire [95:0] g_param_data, g_ac_const, g_st_vp; wire [3:0] g_ad_idx, g_ac_idx; wire g_ad_en, g_ad_norm, g_st_cw, g_st_z0, g_dr_indexed, g_dr_u8;
    wire [2:0]  g_ad_size, g_ad_type, g_dr_mode; wire [31:0] g_dr_first, g_dr_count; wire [1:0] g_st_cull; wire [12:0] g_st_did;
    wire [6:0]  g_st_lw; wire [19:0] g_tri_count;
    wire [31:0] g_c_vis; wire [13*32-1:0] g_prof;
    a_geom #(.REC_BASE(REC_BASE), .REC_BANK_W(REC_BANK_W), .BINLIST_BASE(BINLIST_BASE), .POOL_W(POOL_W), .NWARP(VS_NWARP), .VS_NL(VS_NL),
             .BAND8_MAXW(BAND8_MAXW), .VS_SFU_TAB2(VS_SFU_TAB2), .VS_INT_DIV(VS_INT_DIV), .VS_SINCOS(VS_SINCOS), .PROF(GEOM_PROF)) u_geom (.clk(clk_sys), .rst(rst_ddr),
        .g_frame(g_frame), .g_frame_done(g_frame_done), .g_rbusy(g_rbusy), .g_fb_ok(g_fb_ok), .g_draw_ok(g_draw_ok), .g_copy_busy(g_copy_busy),
        .g_ld_start(g_ld_start), .g_ld_addr(g_ld_addr), .g_ld_len(g_ld_len), .g_ld_done(g_ld_done),
        .g_param_we(g_param_we), .g_param_idx(g_param_idx), .g_param_data(g_param_data),
        .g_ad_we(g_ad_we), .g_ad_idx(g_ad_idx), .g_ad_en(g_ad_en), .g_ad_type(g_ad_type), .g_ad_norm(g_ad_norm), .g_ad_base(g_ad_base), .g_ad_size(g_ad_size),
        .g_ad_stride(g_ad_stride), .g_ad_off(g_ad_off),
        .g_ac_we(g_ac_we), .g_ac_idx(g_ac_idx), .g_ac_const(g_ac_const),
        .g_st_we(g_st_we), .g_st_vp(g_st_vp), .g_st_z(g_st_z), .g_st_gb(g_st_gb), .g_st_sc(g_st_sc), .g_st_cull(g_st_cull), .g_st_cw(g_st_cw), .g_st_z0(g_st_z0), .g_st_lw(g_st_lw), .g_st_fsmask(g_st_fsmask), .g_st_w(g_st_w), .g_st_h(g_st_h), .g_st_did(g_st_did),
        .g_dr_start(g_dr_start), .g_dr_mode(g_dr_mode), .g_dr_indexed(g_dr_indexed), .g_dr_u8(g_dr_u8), .g_dr_idx_base(g_dr_idx_base),
        .g_dr_first(g_dr_first), .g_dr_count(g_dr_count), .g_render(g_render), .g_abort(g_abort), .g_abort_soft(g_abort_soft), .g_render_done(g_render_done),
        .c_vis(g_c_vis), .prof(g_prof),
        .g_busy(g_busy), .err_stack(g_err_stack), .rec_ovf(g_rec_ovf), .bin_ovf(g_bin_ovf), .tri_count(g_tri_count), .c_quads(g_c_quads),
        .r_en(r_en[4:2]), .r_addr(r_addr[5*29-1:2*29]), .r_rdy(r_rdy[4:2]), .r_valid(r_valid[4:2]), .rd_data(rd_data),
        .w_req(w_req[3:2]), .w_en(w_en[3:2]), .w_addr(w_addr[4*29-1:2*29]), .w_data(w_data[4*256-1:2*256]), .w_mask(w_mask[4*32-1:2*32]),
        .w_wren(w_wren[3:2]), .w_wend(w_wend[3:2]), .w_rdy(w_rdy[3:2]), .w_wdf_rdy(w_wdf_rdy[3:2]),
        .rs_valid(rs_v), .rs_data(rs_d), .rs_ready(rs_r));
    xfer_lin #(.MT_PIPE(1'b1)) u_xfer (      // the pipelined memtest count (as on B; timing at 120 MHz)
        .clk(clk_sys), .rst(rst_ddr), .cmd_valid(x_cv), .cmd_op(x_op), .cmd_addr(x_addr), .cmd_len(x_len),
        .cmd_seed(x_seed), .cmd_flags(x_flags), .cmd_ready(x_crdy), .busy(x_busy),
        .wd_valid(x_wdv), .wd_data(x_wdd), .wd_ready(x_wdr),
        .rd_valid_o(x_rdv), .rd_data_o(x_rdd), .rd_ready_o(x_rdr),
        .mt_done(x_mtd), .mt_errors(x_mterr), .mt_first_bad(x_mtbad),
        .w_req(w_req[1]), .w_en(w_en[1]), .w_addr(w_addr[57:29]), .w_data(w_data[511:256]), .w_mask(w_mask[63:32]),
        .w_wren(w_wren[1]), .w_wend(w_wend[1]), .w_rdy(w_rdy[1]), .w_wdf_rdy(w_wdf_rdy[1]),
        .r_en(r_en[1]), .r_addr(r_addr[57:29]), .r_rdy(r_rdy[1]), .r_valid(r_valid[1]), .rdata(rd_data));

    // ---------------- board link ----------------
    wire lclk, lclk90, lclk270, rst_lclk, lclk_locked;
    link_clk #(.LINK_MHZ(LINK_MHZ), .OFF(EXT_LINK)) u_lclk (.clk200(clk_idelay), .rst(rst_s), .lclk(lclk), .lclk90(lclk90),
        .lclk270(lclk270), .rst_lclk(rst_lclk), .locked(lclk_locked));
    wire        l_tx_valid, l_tx_cnt2, l_tx_last, l_tx_ready, l_rx_valid, l_rx_cnt2, l_rx_last, l_rx_hdr, l_rx_ready;
    wire [63:0] l_tx_data, l_rx_data;
    wire        l_lock, l_trained, l_rx_clk_ok, l_far_up, l_training, l_guard_fail, l_tx_enabled;
    wire [7:0]  l_eye, l_phase, l_train_cnt;
    wire [31:0] l_xoff, l_tx_pkts, l_rx_words, l_rx_pkts, l_par, l_crc, l_seq, l_frm, l_drops, l_parmap, l_retx, l_disc;
    wire [8*32-1:0] l_diag;
    // link_core's own outputs; with EXT_LINK the link side is xl_* and the link reads as up and trained, its counters 0
    wire        lc_tx_ready, lc_rx_valid, lc_rx_cnt2, lc_rx_last, lc_rx_hdr; wire [63:0] lc_rx_data;
    wire        lc_lock, lc_trained, lc_rx_clk_ok, lc_far_up, lc_training, lc_guard_fail, lc_tx_enabled;
    wire [7:0]  lc_eye, lc_phase, lc_train_cnt;
    wire [31:0] lc_xoff, lc_tx_pkts, lc_rx_words, lc_rx_pkts, lc_par, lc_crc, lc_seq, lc_frm, lc_drops, lc_parmap, lc_retx, lc_disc;
    wire [8*32-1:0] lc_diag;
    link_core #(.ROLE("A"), .LINK_MHZ(LINK_MHZ), .MMCM_RST_CYCLES(LINK_MMCM_RST), .GUARD_CYCLES(LINK_GUARD),
                .IDLE_TIMEOUT(LINK_IDLE_TO), .TRAIN_WORDS(LINK_TRAIN_WORDS), .RETRY_CYCLES(LINK_RETRY),
                .RELOCK_CYCLES(LINK_RELOCK), .RETRAIN_WINDOW(LINK_RWIN), .SIM_SHIFTER(LINK_SIM_SHIFTER), .OFF(EXT_LINK)) u_link (
        .clk(clk_sys), .rst(rst_s), .lclk(lclk), .lclk90(lclk90), .lclk270(lclk270), .rst_lclk(rst_lclk),
        .fill_en(linkctl[0]),
        .tx_valid(l_tx_valid & ~EXT_LINK), .tx_data(l_tx_data), .tx_cnt2(l_tx_cnt2), .tx_last(l_tx_last), .tx_ready(lc_tx_ready),
        .rx_valid(lc_rx_valid), .rx_data(lc_rx_data), .rx_cnt2(lc_rx_cnt2), .rx_last(lc_rx_last), .rx_hdr(lc_rx_hdr),
        .rx_ready(l_rx_ready & ~EXT_LINK), .pkt_done(), .pkt_ok(),
        .lock(lc_lock), .trained(lc_trained), .rx_clk_ok(lc_rx_clk_ok), .far_up(lc_far_up), .training(lc_training),
        .guard_fail(lc_guard_fail), .tx_enabled(lc_tx_enabled), .xoff_events(lc_xoff),
        .eye_width(lc_eye), .phase(lc_phase), .train_cnt(lc_train_cnt),
        .tx_pkts(lc_tx_pkts), .rx_words(lc_rx_words), .rx_pkts(lc_rx_pkts), .par_errs(lc_par), .crc_errs(lc_crc),
        .seq_errs(lc_seq), .frm_errs(lc_frm), .lock_drops(lc_drops), .par_map(lc_parmap), .retx(lc_retx), .disc(lc_disc),
        .diag(lc_diag), .jm1(jm1), .jm2(jm2));
    assign l_tx_ready = EXT_LINK ? xl_tx_ready : lc_tx_ready;
    assign {l_rx_valid, l_rx_data, l_rx_cnt2, l_rx_last, l_rx_hdr} = EXT_LINK ? {xl_rx_valid, xl_rx_data, xl_rx_cnt2, xl_rx_last, xl_rx_hdr}
                                                                             : {lc_rx_valid, lc_rx_data, lc_rx_cnt2, lc_rx_last, lc_rx_hdr};
    assign {l_lock, l_trained, l_rx_clk_ok, l_far_up, l_training, l_guard_fail, l_tx_enabled} =
           EXT_LINK ? 7'b111_1001 : {lc_lock, lc_trained, lc_rx_clk_ok, lc_far_up, lc_training, lc_guard_fail, lc_tx_enabled};
    assign {l_eye, l_phase, l_train_cnt} = EXT_LINK ? 24'd0 : {lc_eye, lc_phase, lc_train_cnt};
    assign {l_xoff, l_tx_pkts, l_rx_words, l_rx_pkts, l_par, l_crc, l_seq, l_frm, l_drops, l_parmap, l_retx, l_disc} = EXT_LINK ? 384'd0 :
           {lc_xoff, lc_tx_pkts, lc_rx_words, lc_rx_pkts, lc_par, lc_crc, lc_seq, lc_frm, lc_drops, lc_parmap, lc_retx, lc_disc};
    assign l_diag = EXT_LINK ? '0 : lc_diag;
    assign xl_tx_valid = l_tx_valid; assign xl_tx_data = l_tx_data; assign xl_tx_cnt2 = l_tx_cnt2; assign xl_tx_last = l_tx_last;
    assign xl_rx_ready = l_rx_ready;
    wire        b_alive; wire [B_LSTAT_WORDS*32-1:0] b_stat; wire [31:0] c_bstat, c_bev, c_bunk, c_brd;
    wire [31:0] cr_sent, cr_consumed, c_cur_sent, c_credit_wait, c_sync, c_rs_wait;
    link_a_port #(.IDLE_CYCLES(KEEPALIVE), .ALIVE_CYCLES(B_ALIVE)) u_lport (
        .clk(clk_sys), .rst(rst_s),
        .tx_valid(l_tx_valid), .tx_data(l_tx_data), .tx_cnt2(l_tx_cnt2), .tx_last(l_tx_last), .tx_ready(l_tx_ready),
        .rx_valid(l_rx_valid), .rx_data(l_rx_data), .rx_last(l_rx_last), .rx_hdr(l_rx_hdr), .rx_ready(l_rx_ready),
        .f_valid(f_valid), .f_data(f_data), .f_len(f_len), .f_last(f_last), .f_ready(f_ready),
        .rs_valid(rs_v), .rs_data(rs_d), .rs_ready(rs_r), .rs_abort(g_abort), .resync(g_resync), .sync_go(l_sync_go),
        .cur_valid(cur_valid), .cur_pos(cur_pos),
        .sent(cr_sent), .consumed(cr_consumed), .c_cur_sent(c_cur_sent), .c_credit_wait(c_credit_wait),
        .c_sync(c_sync), .rs_sent(cr_rs_sent), .rs_consumed(cr_rs_consumed), .c_rs_pkts(c_rs_pkts), .c_rs_wait(c_rs_wait),
        .ev_req(s_req[5]), .ev_len(s_len[95:80]), .ev_valid(s_valid[5]), .ev_data(s_data[191:160]),
        .ev_ready(s_ready[5]), .ev_done(s_done[5]),
        .rd_req(s_req[6]), .rd_len(s_len[111:96]), .rd_valid(s_valid[6]), .rd_data(s_data[223:192]),
        .rd_ready(s_ready[6]), .rd_done(s_done[6]),
        .b_alive(b_alive), .b_stat(b_stat), .c_bstat(c_bstat), .c_bev(c_bev), .c_unk(c_bunk), .c_brd(c_brd));
    assign link_ok = l_trained & l_far_up & b_alive;

    // ---------------- reply writer ----------------
    wire [31:0] c_sent, c_dropped;
    fb_reply #(.NS(NS), .RF_AW(RF_AW)) u_reply (
        .clk(clk_sys), .rst(rst_s), .host_valid(host_valid), .host_mac(host_mac), .host_ip(host_ip),
        .host_port(host_port), .s_req(s_req), .s_len(s_len), .s_valid(s_valid), .s_data(s_data),
        .s_ready(s_ready), .s_done(s_done), .rf_wen(rf_wen), .rf_wdata(rf_wd), .rf_wcount(rf_wcount),
        .c_sent(c_sent), .c_dropped(c_dropped));

    // ---------------- status ----------------
    reg [31:0] ms_div, uptime_ms;
    always @(posedge clk_sys) begin
        if (rst_s) begin ms_div <= 0; uptime_ms <= 0; end
        else if (ms_div == 99_999) begin ms_div <= 0; uptime_ms <= uptime_ms + 1; end
        else ms_div <= ms_div + 1;
    end
    localparam int RING = 1 << RING_LOG2;
    wire [RING_LOG2:0] q_free = RING - q_used;
    // bit 31: the vertex core has no integer divider (IDIV / IMOD / UDIV / UMOD stop the warp; 0 on builds with one);
    // bit 30: the vertex core has no SIN / COS units (SIN / COS stop the warp)
    wire [31:0] flags = {~VS_INT_DIV, ~VS_SINCOS, 9'h0, phyad, 1'b0, adv1000, rxdly, idelay_tap, link_ok, host_valid, mdio_init_done,
                         duplex, speed, link_up, calib_done};
    wire [31:0] sw [0:STATUS_WORDS-1];
    assign sw[0]  = STATUS_LAYOUT;
    assign sw[1]  = BUILD_ID;
    assign sw[2]  = uptime_ms;
    assign sw[3]  = flags;
    assign sw[4]  = {bmsr, physr};
    genvar gi;
    generate for (gi = 0; gi < 14; gi = gi + 1) begin : g_cnt
        assign sw[5 + gi] = cnt[32*gi +: 32];
    end endgenerate
    assign sw[19] = exp_seq;
    assign sw[20] = c_acc;
    assign sw[21] = c_dup;
    assign sw[22] = c_gap;
    assign sw[23] = c_full;
    assign sw[24] = c_bad;
    assign sw[25] = c_ack;
    assign sw[26] = 32'(q_used);
    assign sw[27] = calib_done ? 32'({q_free, 5'b00000}) : 32'h0;
    assign sw[28] = c_cmds;
    assign sw[29] = c_errs;
    assign sw[30] = last_err;
    assign sw[31] = last_err_seq;
    assign sw[32] = mt_last;
    assign sw[33] = mdio_polls;
    assign sw[34] = snap_age;
    assign sw[35] = c_sent;
    assign sw[36] = c_dropped;
    // board B (from its STATUS packets, see board_b_core) and the board link
    assign sw[37] = b_stat[0*32 +: 32];              // B flags
    assign sw[38] = b_stat[1*32 +: 32];              // B frames
    assign sw[39] = b_stat[3*32 +: 32];              // B measured pixel clock, kHz
    assign sw[40] = {b_stat[2*32 +: 16], b_stat[5*32 +: 16]};   // {B underruns, B mode errors}
    assign sw[41] = b_stat[7*32 +: 32];              // B command processor {errors, commands}
    assign sw[42] = {l_eye, l_phase, 8'h0, 1'b0, l_guard_fail, l_tx_enabled, l_training, l_far_up, l_trained,
                     l_lock, b_alive};
    assign sw[43] = c_fwd;                           // commands forwarded to B
    assign sw[44] = c_bev;                           // B events relayed to the host
    assign sw[45] = c_bstat;                         // B status packets
    assign sw[46] = {l_crc[15:0], l_par[15:0]};      // A receiver: CRC / parity events
    assign sw[47] = {l_disc[15:0], l_retx[15:0]};    // A: packets discarded by our receiver, our TX rewinds
    // layout 3: link diagnostics (link_rx_diag on A's receiver), B's receiver and transfer counters
    assign sw[48] = linkctl;                         // REG_LINKCTL (b0 = fill words on A's transmit lane)
    assign sw[49] = l_parmap;                        // A receiver parity errors per half {.., high, low}
    genvar gd;
    generate for (gd = 0; gd < 8; gd = gd + 1) begin : g_diag
        assign sw[50 + gd] = l_diag[gd*32 +: 32];    // bursts, after long gap, histogram, last error, last word,
    end endgenerate                                  // fill words, fill parity errors, parity errors (real words)
    assign sw[58] = l_drops;                         // A receiver lock losses
    assign sw[59] = {l_frm[15:0], 8'h0, l_train_cnt}; // A receiver frame errors, trainings
    assign sw[60] = b_stat[8*32 +: 32];              // B receiver {CRC, parity}
    assign sw[61] = b_stat[9*32 +: 32];              // B {trainings, fill on, parity errors (real words)}
    assign sw[62] = b_stat[10*32 +: 32];             // B images created
    assign sw[63] = b_stat[11*32 +: 32];             // B pixels written (WRITE_IMAGE)
    // layout 4: B's receive diagnostics, XOFF / ARQ / word counters of both boards (B's from its STATUS words 12-19)
    assign sw[64] = b_stat[12*32 +: 32];             // B receiver histogram {pos 0, pos 1-3, pos 4+, after long gap}
    assign sw[65] = b_stat[13*32 +: 32];             // B receiver last error {gap, pos, 3'b0, last, parity flags}
    assign sw[66] = b_stat[14*32 +: 32];             // B receiver parity errors per half {hi PAR1, hi PAR0, lo PAR1, lo PAR0}
    assign sw[67] = b_stat[15*32 +: 32];             // B RX FIFO XOFF events (forward lane paused A)
    assign sw[68] = b_stat[16*32 +: 32];             // B transmitter {timeouts, rewinds}
    assign sw[69] = b_stat[17*32 +: 32];             // B receiver {NAKs requested, packets discarded}
    assign sw[70] = b_stat[18*32 +: 32];             // B CMD packets received (compare with sw[43])
    assign sw[71] = b_stat[19*32 +: 32];             // B words received (forward lane, real words)
    assign sw[72] = l_xoff;                          // A RX FIFO XOFF events (backward lane paused B)
    assign sw[73] = {l_disc[31:16], l_retx[31:16]};  // A receiver NAKs requested, A transmitter timeouts
    assign sw[74] = l_rx_words;                      // A words received (backward lane, real words)
    assign sw[75] = 32'h0;                           // reserved
    // layout 5: credit flow control, cursor fast path, palette and READ / FILL / COPY counters
    assign sw[76] = cr_sent;                         // A CMD words sent to B
    assign sw[77] = cr_consumed;                     // CMD words B reported as taken (CREDIT packets)
    assign sw[78] = c_credit_wait;                   // A cycles a CMD packet waited for credit
    assign sw[79] = {c_cur_sent[15:0], c_sync[15:0]};     // {cursor fast-path packets sent, credit SYNC packets sent}
    assign sw[80] = b_stat[20*32 +: 32];             // B CMD words taken from its command buffer
    assign sw[81] = b_stat[21*32 +: 32];             // B {cursor fast-path packets received, palette publishes}
    assign sw[82] = b_stat[22*32 +: 32];             // B {cursor commands, READ / FILL / COPY operations}
    assign sw[83] = b_stat[23*32 +: 32];             // B READ / FILL / COPY pixels
    // layout 6: A's geometry stage
    assign sw[84] = {10'd0, g_rbusy, g_busy, g_tri_count};    // {render side busy, geometry busy, triangle records since FRAME_BEGIN}
    assign sw[85] = {29'd0, g_bin_ovf, g_rec_ovf, g_err_stack};   // sticky flags: bin pool / band count overflow, record region overflow (per frame), the vertex core's stack violation
    assign sw[86] = g_c_quads;                       // quads of the last frame's render stream
    assign sw[87] = {16'(cr_rs_sent - cr_rs_consumed), c_rs_pkts[15:0]};   // {render beats in flight to B, RSTREAM packets}
    // layout 7: B's render engine cycle counters of its last frame (its STATUS words 24-31:
    // frame, flush, band change, draw change, FS busy, band set-up, stream empty, ROP stall), then A's cumulative cycle
    // counters (the host takes deltas): geometry side busy, render side (band walk) busy, render beats waiting for B's
    // credit, a_render stalled by the full render FIFO
    genvar gp;
    generate for (gp = 0; gp < 8; gp = gp + 1) begin : g_bperf
        assign sw[88 + gp] = b_stat[(24 + gp)*32 +: 32];
    end endgenerate
    reg [31:0] pa_geom, pa_render, pa_rsfull;
    always @(posedge clk_sys) begin
        if (rst_ddr) begin pa_geom <= 0; pa_render <= 0; pa_rsfull <= 0; end
        else begin
            if (g_busy) pa_geom <= pa_geom + 1;
            if (g_rbusy) pa_render <= pa_render + 1;
            if (rs_v && !rs_r) pa_rsfull <= pa_rsfull + 1;
        end
    end
    assign sw[96] = pa_geom;
    assign sw[97] = pa_render;
    assign sw[98] = c_rs_wait;
    assign sw[99] = pa_rsfull;
    assign sw[100] = g_c_vis;                        // layout 9: primitive-band visits of A's walk, cumulative (layout 8: culled quads)
    // layout 10: the geometry side's profile — 13 cumulative counters from a_geom (its comment lists them: loader,
    // shadow copy, feed, feed's refused reads, vertex core, assembler, vertex waiting for setup, setup, push waiting for the
    // binner, binner, bin clear, {loads, draws}, primitives into setup), then the dispatcher's idle cycles (no command queued)
    // and its cycles spent waiting in D_GWAIT for a geometry command
    generate for (gi = 0; gi < 13; gi = gi + 1) begin : g_aprof
        assign sw[101 + gi] = g_prof[gi*32 +: 32];
    end endgenerate
    assign sw[114] = c_hdr_wait;
    assign sw[115] = c_gwait;
    generate for (gi = 0; gi < STATUS_WORDS; gi = gi + 1) begin : g_vec
        assign status_vec[gi*32 +: 32] = sw[gi];
    end endgenerate

    generate if (JTAG_STATUS) begin : g_jtag
        // JTAG USER1 frame: {seq, status_vec (MSB = word STATUS_WORDS-1), seq, A5}, see tools/a_status_jtag.tcl
        link_status_jtag #(.FW(STATUS_WORDS * 32), .SNAP_CYCLES(1_000_000), .LEAN(JTAG_LEAN)) u_jtag (
            .clk(clk_sys), .rst(rst_s), .fields(status_vec));
    end endgenerate

    assign led = {link_up, calib_done};
    wire _unused_link = &{1'b0, lclk_locked, l_rx_clk_ok, l_tx_pkts, l_rx_pkts, l_seq, l_rx_cnt2, c_bunk, c_brd};
endmodule
`default_nettype wire
