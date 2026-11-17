// board_b_core — board B (pixels, HDMI): DDR3 + mem_arb, image
// descriptor table, scanout in five video modes through one DRP-reconfigured MMCM, the board link
// to A with the command processor (SET_MODE, PRESENT, FENCE, CREATE_IMAGE, DESTROY, WRITE_IMAGE,
// MEM_WRITE / MEM_READ / MEMTEST), and the bring-up harness (test images, KEY1 / JTAG USER2). Everything except
// the MIG/clock wrapper, so benches can attach a MIG model. clk_sys = MIG ui_clk (100 MHz).
//   mem_arb ports: R0 scanout (MUST_RUN), W0 transfer engine (img_write; R1 reserved for READ / COPY),
//                         R2/W1 xfer_lin = memory test and debug access (host MEM_*, the harness's image fill)
//   link_core role B -> link_b_port -> fb_cmd_b -> disp_b / img_table / img_write / xfer_lin; EVENTs, READ_DATA
//              and STATUS (1 ms) back
//   b_bringup (parameter BRINGUP, off in the hardware build, on only for the benches that drive it): fills test images (img_table entries 0..9 = handles
//              0xC000..0xC009), KEY1 / JTAG commands;
//              the two share xfer_lin, the image table write port and the display one at a time (fb_cmd_b starts
//              nothing while the harness is active, the harness nothing while fb_cmd_b is busy)
// Status: B_STATUS_WORDS x 32 bits over JTAG USER1 (link_status_jtag, tools/b_jtag.tcl); a subset goes to A
// in the link STATUS packet. DDR-side blocks are held in reset until calibration completes.
`default_nettype none
module board_b_core
    import img_pkg::*;
#(
    parameter [31:0] BUILD_ID     = 32'd0,
    parameter int    DEFAULT_MODE = 1,
    parameter bit    SIM_MODE4    = 1'b0,
    parameter int    IMG_SET      = 0,
    parameter bit    BRINGUP      = 1'b0,        // the bring-up harness (b_bringup: boot test images, KEY1, JTAG USER2 commands): off on the
                                                 // board (it costs ~440 slices); 1 only for tb_board_b and tb_two_boards
    parameter int    CLK_HZ       = 100_000_000,  // clk_sys (the MIG ui_clk); the millisecond timers below derive from it
    parameter int    DEBOUNCE     = CLK_HZ / 50,          // 20 ms
    parameter int    LONG_PRESS   = 6 * (CLK_HZ / 10),    // 600 ms
    parameter int    LOCK_TIMEOUT = CLK_HZ / 50,          // 20 ms
    parameter int    MS_CYCLES    = CLK_HZ / 1000,
    parameter bit    JTAG         = 1'b1,        // BSCANE2 status (USER1) and command (USER2) registers
    parameter int    B_STATUS_WORDS = 24,        // fixed (tools/b_jtag.tcl)
    // board link (simulation shortens the start-up times)
    parameter int    LINK_MHZ       = 100,
    parameter int    LINK_GUARD     = 10_000_000,
    parameter int    LINK_MMCM_RST  = 1_000_000,
    parameter int    LINK_IDLE_TO   = 10_000_000,
    parameter int    LINK_TRAIN_WORDS = 1024,
    parameter int    LINK_RETRY     = 100_000,
    parameter int    LINK_RELOCK    = 10_000_000,   // backstop only: relock is counted in words (link_core)
    parameter [31:0] LINKCTL_DEFAULT = 32'h1,      // b0: fill words on B's transmit lane (JTAG op 0x06 changes it)
    parameter int    LINK_RWIN      = CLK_HZ,            // 1 s rate window
    parameter int    LSTAT_CYCLES   = CLK_HZ / 1000,     // link STATUS period: 1 ms
    parameter bit    LINK_SIM_SHIFTER = 1'b0,
    // 3D: band rows (A's band height) and the widest render target
    parameter int    B3D_NLINES     = 4,
    parameter int    B3D_MAXW       = 1280,          // widest render target (1280 x 800 is the largest mode)
    parameter int    B3D_CELLS      = 4096,          // band-buffer cells per bank: 8-line bands up to 1024 wide
    parameter bit    VOL3D          = 1'b0,          // hardware 3D textures (tmu24 / fb_cmd_b): off on the board (slice room)
    // optional fragment-core operations (b_render FS_XSFU / FS_XCLAMP). STATUS flags bits 28 / 29 report them, so a
    // driver knows what the board's programs may contain
    parameter int    B3D_QRINF      = 4,             // quarter-rate micro-ops in flight per class (b_render FS_QRINF)
    parameter bit    B3D_XSFU       = 1'b1,          // SQRT, RCPM (a divide), RSQM: the SFU's result times an operand in one micro-op
    parameter bit    B3D_XCLAMP     = 1'b0,          // CLAMP in one micro-op: OFF for its cost —
                                                     // +1 664 LUT (float_cmp3 on 16 lanes) for one micro-op per clamp(), under 2 % on
                                                     // a ray-marching shader. In the RTL and the benches, behind this.
    parameter int    B3D_NWARP      = 8,             // fragment-core warps in flight (b_render FS_NWARP; 8 instead of 4 costs
                                                     // about 7.5 k LUT out of context)
    // slice-room knobs: the TMU's F2 at one pixel per cycle (B3D_F2_DUAL 0: −0.6 k LUT, the texture round trip 35 → 37
    // cycles) and the texture cache's prefetch off (B3D_TC_PF 0: ≈ −0.3 k LUT; the 64 KB capacity gives most of the miss
    // reduction). Both are bit-exact in the benches (tb_tmu24 TB_F2SINGLE / TB_TC_NOPF, the frames with TB_TC_NOPF);
    // set them to 1 when B has room.
    parameter bit    B3D_F2_DUAL    = 1'b0,
    parameter bit    B3D_TC_PF      = 1'b0,
    parameter int    B3D_NTS        = 16,            // fragment-core triangle slots (b_render NTS)
    // TEX_SKIP: uncovered pixels of a quad unsampled where the program allows; TEX_W2: two texture-result writers in the
    // fragment core; FS_PRE: the program fetch ahead of the launch, off — b_render FS_PRE 0 folds its logic away
    parameter bit    B3D_TEX_SKIP   = 1'b1,
    parameter bit    B3D_TEX_W2     = 1'b1,
    parameter bit    B3D_FS_PRE     = 1'b0
)(
    input  wire         clk_sys, rst_sys, calib_done,
    input  wire         clk200,                  // 200 MHz (BUFG) for the video and link MMCMs
    input  wire         key1_n,
    // bench command port (OR'ed with JTAG USER2)
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
    // board link headers
    inout  wire [31:0]  jm1, jm2,
    output wire [1:0]   led,
    output wire [B_STATUS_WORDS*32-1:0] status_vec
);
    import fb_pkg::*;
    localparam [31:0] B_STATUS_LAYOUT = 32'hB000_0004;   // 4: the JTAG frame carries 24 more words, 24-47 = b_render's hwprof (the link STATUS and status_vec are unchanged)
                                                         // (layout 3: words 18-21 = the texture unit's counters when BRINGUP is off
    // Resets, registered twice: the MIG's reset register sits beside the DDR banks while the users are spread over the
    // whole die (up to 9.5 ns of routing); the second stage may be replicated (max_fanout).
    reg rst_s1 = 1'b1; (* max_fanout = 12 *) reg rst_d1 = 1'b1;   // replicated towards rst_ddr's replicas (see board_a_core)
    (* max_fanout = 40 *) reg rst_s = 1'b1, rst_ddr = 1'b1;
    always @(posedge clk_sys) begin
        rst_s1 <= rst_sys; rst_d1 <= rst_sys | ~calib_done;
        rst_s  <= rst_s1;  rst_ddr <= rst_d1;
    end

    // ---------------- memory arbiter ----------------
    // 3D (b_render): R3 band load, R4 TMU, R5 buckets / programs; W3 band flush (B keeps no triangle records: board A
    // sends the TRI item again instead of TRISEL)
    wire [5:0] r_en, r_rdy, r_valid; wire [173:0] r_addr; wire [255:0] rd_data;
    wire [3:0] w_req, w_en, w_wren, w_wend, w_rdy, w_wdf_rdy; wire [115:0] w_addr;
    wire [1023:0] w_data; wire [127:0] w_mask;
    mem_arb #(.NR(6), .NW(4), .RO_AW(5), .RO_RESERVE(4), .MUST_RUN(6'b000001)) u_arb (
        .clk(clk_sys), .rst(rst_ddr), .r_en(r_en), .r_addr(r_addr), .r_rdy(r_rdy), .r_valid(r_valid), .rd_data(rd_data),
        .w_req(w_req), .w_en(w_en), .w_addr(w_addr), .w_data(w_data), .w_mask(w_mask), .w_wren(w_wren),
        .w_wend(w_wend), .w_rdy(w_rdy), .w_wdf_rdy(w_wdf_rdy),
        .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end),
        .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy), .app_rd_data_valid(app_rd_data_valid), .app_rd_data(app_rd_data));
    // W0: WRITE_IMAGE engine
    wire        iw_cv, iw_crdy, iw_dv, iw_drdy, iw_busy; wire [1:0] iw_sf; wire [29:0] iw_row; wire [15:0] iw_pitch;
    wire [12:0] iw_x, iw_w, iw_i0; wire [23:0] iw_npix; wire [7:0] iw_tfmt; wire [31:0] iw_dd, iw_pixels;
    img_write u_iw (
        .clk(clk_sys), .rst(rst_ddr), .cmd_valid(iw_cv), .cmd_ready(iw_crdy), .cmd_row(iw_row), .cmd_pitch(iw_pitch),
        .cmd_x(iw_x), .cmd_w(iw_w), .cmd_i0(iw_i0), .cmd_npix(iw_npix), .cmd_tfmt(iw_tfmt), .cmd_sf(iw_sf),
        .d_valid(iw_dv), .d_data(iw_dd), .d_ready(iw_drdy), .busy(iw_busy), .c_pixels(iw_pixels),
        .w_req(w_req[0]), .w_en(w_en[0]), .w_addr(w_addr[28:0]), .w_data(w_data[255:0]), .w_mask(w_mask[31:0]),
        .w_wren(w_wren[0]), .w_wend(w_wend[0]), .w_rdy(w_rdy[0]), .w_wdf_rdy(w_wdf_rdy[0]));
    // R1 / W2: READ / FILL / COPY_IMAGE engine
    wire        xf_cv, xf_crdy, xf_busy, xo_v, xo_rdy, xf_mx, xf_my, xf_bup, xf_key, xf_rmw;
    wire [1:0]  xf_op, xf_sf, xf_df; wire [29:0] xf_s0, xf_d0; wire [15:0] xf_sp, xf_dp;
    wire [12:0] xf_sx, xf_sy, xf_sw, xf_sh, xf_dx, xf_dy, xf_dw, xf_dh, xf_hq, xf_hu0, xf_vq, xf_vu0, xf_j0, xf_i0;
    wire [13:0] xf_hr2, xf_ha0, xf_vr2, xf_va0; wire [31:0] xf_klo, xf_khi, xf_val, xo_d, xf_ops, xf_pixels;
    wire [3:0]  xf_mask; wire [7:0] xf_tf; wire [23:0] xf_np;
    wire        xf_a1, xf_swz, xf_wrap; wire [7:0] xf_swz_sel; wire [12:0] xf_px, xf_py;
    img_xfer u_xf (
        .clk(clk_sys), .rst(rst_ddr), .cmd_valid(xf_cv), .cmd_ready(xf_crdy), .cmd_op(xf_op),
        .cmd_s_row0(xf_s0), .cmd_s_pitch(xf_sp), .cmd_s_fmt(xf_sf), .cmd_s_x(xf_sx), .cmd_s_y(xf_sy), .cmd_s_w(xf_sw),
        .cmd_s_h(xf_sh), .cmd_d_row0(xf_d0), .cmd_d_pitch(xf_dp), .cmd_d_fmt(xf_df), .cmd_d_x(xf_dx), .cmd_d_y(xf_dy),
        .cmd_d_w(xf_dw), .cmd_d_h(xf_dh), .cmd_hq(xf_hq), .cmd_hu0(xf_hu0), .cmd_vq(xf_vq), .cmd_vu0(xf_vu0),
        .cmd_hr2(xf_hr2), .cmd_ha0(xf_ha0), .cmd_vr2(xf_vr2), .cmd_va0(xf_va0), .cmd_mir_x(xf_mx), .cmd_mir_y(xf_my),
        .cmd_bottom_up(xf_bup), .cmd_key(xf_key), .cmd_key_lo(xf_klo), .cmd_key_hi(xf_khi), .cmd_value(xf_val),
        .cmd_mask(xf_mask), .cmd_rmw(xf_rmw), .cmd_tfmt(xf_tf), .cmd_j0(xf_j0), .cmd_i0(xf_i0), .cmd_npix(xf_np),
        .cmd_alpha1(xf_a1), .cmd_swz(xf_swz), .cmd_swz_sel(xf_swz_sel), .cmd_wrap(xf_wrap), .cmd_px(xf_px), .cmd_py(xf_py),
        .busy(xf_busy), .o_valid(xo_v), .o_data(xo_d), .o_ready(xo_rdy), .c_ops(xf_ops), .c_pixels(xf_pixels),
        .r_en(r_en[1]), .r_addr(r_addr[57:29]), .r_rdy(r_rdy[1]), .r_valid(r_valid[1]), .rd_data(rd_data),
        .w_req(w_req[2]), .w_en(w_en[2]), .w_addr(w_addr[86:58]), .w_data(w_data[767:512]), .w_mask(w_mask[95:64]),
        .w_wren(w_wren[2]), .w_wend(w_wend[2]), .w_rdy(w_rdy[2]), .w_wdf_rdy(w_wdf_rdy[2]));

    // ---------------- display (controls from fb_cmd_b, else from the harness) ----------------
    wire        mode_busy, mode_done, mode_ok, video_on, mmcm_locked, pres_done, cur_valid;
    wire [2:0]  cur_mode;
    wire [31:0] cur_tag, pres_dropped;
    wire [31:0] st_frames, st_underruns, st_pix_khz, st_frame_cyc, st_mode_ok, st_mode_err;
    // command processor side
    wire        c_set_mode, c_pres_valid, c_pres_origin, c_pres_now, cp_busy; wire [1:0] c_pres_fmt;
    wire [2:0]  c_set_mode_id;
    // palette / cursor (command processor and the fast path)
    wire        pm_we, pal_pub, pal_busy, cr_cmd, cr_take, cr_setpos, cs_wr, cs_ready, fp_valid;
    wire [7:0]  pm_idx; wire [23:0] pm_data; wire [2:0] cr_op; wire [1:0] cr_mode; wire [5:0] cr_hx, cr_hy;
    wire [15:0] cr_x, cr_y, fp_x, fp_y; wire [11:0] cs_idx; wire [31:0] cs_data;
    wire [31:0] st_pal_pub, st_fast_moves, st_cur_cmds;
    wire [29:0] c_pres_base; wire [15:0] c_pres_pitch; wire [12:0] c_pres_w, c_pres_h; wire [31:0] c_pres_tag;
    // harness side
    wire        h_set_mode, h_pres_valid, h_pres_565, h_pres_origin, h_pres_now;
    wire [2:0]  h_set_mode_id;
    wire [29:0] h_pres_base; wire [15:0] h_pres_pitch; wire [12:0] h_pres_w, h_pres_h; wire [31:0] h_pres_tag;
    // the harness starts nothing while the command processor is busy, so the two never request together
    wire        d_cp = c_pres_valid;
    disp_b #(.DEFAULT_MODE(DEFAULT_MODE), .SIM_MODE4(SIM_MODE4), .LOCK_TIMEOUT(LOCK_TIMEOUT), .MS_CYCLES(MS_CYCLES)) u_disp (
        .clk(clk_sys), .rst(rst_ddr), .clkin200(clk200),
        .set_mode(c_set_mode | h_set_mode), .set_mode_id(c_set_mode ? c_set_mode_id : h_set_mode_id),
        .mode_busy(mode_busy), .mode_done(mode_done),
        .mode_ok(mode_ok), .cur_mode(cur_mode), .video_on(video_on), .mmcm_locked(mmcm_locked),
        .pres_valid(c_pres_valid | h_pres_valid), .pres_base(d_cp ? c_pres_base : h_pres_base),
        .pres_pitch(d_cp ? c_pres_pitch : h_pres_pitch), .pres_w(d_cp ? c_pres_w : h_pres_w),
        .pres_h(d_cp ? c_pres_h : h_pres_h), .pres_fmt(d_cp ? c_pres_fmt : {1'b0, h_pres_565}),
        .pres_origin(d_cp ? c_pres_origin : h_pres_origin), .pres_now(d_cp ? c_pres_now : h_pres_now),
        .pres_tag(d_cp ? c_pres_tag : h_pres_tag),
        .pres_done(pres_done), .cur_valid(cur_valid), .cur_tag(cur_tag), .pres_dropped(pres_dropped),
        .pm_we(pm_we), .pm_idx(pm_idx), .pm_data(pm_data), .pal_pub(pal_pub), .pal_busy(pal_busy),
        .cr_cmd(cr_cmd), .cr_op(cr_op), .cr_mode(cr_mode), .cr_take(cr_take), .cr_setpos(cr_setpos),
        .cr_hx(cr_hx), .cr_hy(cr_hy), .cr_x(cr_x), .cr_y(cr_y), .cs_wr(cs_wr), .cs_idx(cs_idx), .cs_data(cs_data),
        .cs_ready(cs_ready), .fp_valid(fp_valid), .fp_x(fp_x), .fp_y(fp_y),
        .st_pal_pub(st_pal_pub), .st_fast_moves(st_fast_moves), .st_cur_cmds(st_cur_cmds),
        .r_en(r_en[0]), .r_addr(r_addr[28:0]), .r_rdy(r_rdy[0]), .r_valid(r_valid[0]), .rd_data(rd_data),
        .hdmi_clk_p(hdmi_clk_p), .hdmi_clk_n(hdmi_clk_n), .hdmi_data_p(hdmi_data_p), .hdmi_data_n(hdmi_data_n),
        .st_frames(st_frames), .st_underruns(st_underruns), .st_pix_khz(st_pix_khz), .st_frame_cyc(st_frame_cyc),
        .st_mode_ok(st_mode_ok), .st_mode_err(st_mode_err));

    // ---------------- image descriptor table (write port: command processor or harness) ----------------
    wire        it_rvalid, it_rvalid2; wire [9:0] it_ridx, it_ridx2; img_desc_t it_rdesc, it_rdesc2;
    wire        h_it_we, c_it_we, c_it_wvalid; wire [9:0] h_it_widx, c_it_widx; img_desc_t h_it_wdesc, c_it_wdesc;
    img_table u_it (.clk(clk_sys), .rst(rst_ddr), .we(h_it_we | c_it_we), .widx(c_it_we ? c_it_widx : h_it_widx),
        .wvalid(c_it_we ? c_it_wvalid : 1'b1), .wdesc(c_it_we ? c_it_wdesc : h_it_wdesc),
        .ridx(it_ridx), .rvalid(it_rvalid), .rdesc(it_rdesc), .ridx2(it_ridx2), .rvalid2(it_rvalid2), .rdesc2(it_rdesc2));

    // ---------------- memory test / debug engine (R2 / W1): command processor (x_own) or harness ----------------
    wire        x_cv, x_crdy, x_busy, x_wdv, x_wdr, x_mtd, x_rdv, x_rdr; wire [1:0] x_op, x_flags;
    wire [29:0] x_addr, x_len, x_mtbad; wire [31:0] x_seed, x_wdd, x_mterr, x_rdd;
    wire        h_x_cv, h_x_wdv; wire [1:0] h_x_op, h_x_flags; wire [29:0] h_x_addr, h_x_len; wire [31:0] h_x_seed, h_x_wdd;
    wire        c_x_own, c_x_cv, c_x_wdv, c_x_rdr; wire [1:0] c_x_op, c_x_flags; wire [29:0] c_x_addr, c_x_len;
    wire [31:0] c_x_seed, c_x_wdd;
    assign x_cv    = c_x_own ? c_x_cv    : h_x_cv;
    assign x_op    = c_x_own ? c_x_op    : h_x_op;
    assign x_addr  = c_x_own ? c_x_addr  : h_x_addr;
    assign x_len   = c_x_own ? c_x_len   : h_x_len;
    assign x_seed  = c_x_own ? c_x_seed  : h_x_seed;
    assign x_flags = c_x_own ? c_x_flags : h_x_flags;
    assign x_wdv   = c_x_own ? c_x_wdv   : h_x_wdv;
    assign x_wdd   = c_x_own ? c_x_wdd   : h_x_wdd;
    assign x_rdr   = c_x_own ? c_x_rdr   : 1'b1;
    xfer_lin #(.MT_PIPE(1'b1)) u_xfer (     // the pipelined memtest count (timing)
        .clk(clk_sys), .rst(rst_ddr), .cmd_valid(x_cv), .cmd_op(x_op), .cmd_addr(x_addr), .cmd_len(x_len),
        .cmd_seed(x_seed), .cmd_flags(x_flags), .cmd_ready(x_crdy), .busy(x_busy),
        .wd_valid(x_wdv), .wd_data(x_wdd), .wd_ready(x_wdr),
        .rd_valid_o(x_rdv), .rd_data_o(x_rdd), .rd_ready_o(x_rdr),
        .mt_done(x_mtd), .mt_errors(x_mterr), .mt_first_bad(x_mtbad),
        .w_req(w_req[1]), .w_en(w_en[1]), .w_addr(w_addr[57:29]), .w_data(w_data[511:256]), .w_mask(w_mask[63:32]),
        .w_wren(w_wren[1]), .w_wend(w_wend[1]), .w_rdy(w_rdy[1]), .w_wdf_rdy(w_wdf_rdy[1]),
        .r_en(r_en[2]), .r_addr(r_addr[86:58]), .r_rdy(r_rdy[2]), .r_valid(r_valid[2]), .rdata(rd_data));

    // ---------------- 3D render engine (FLUSH runs one frame of the render stream) ----------------
    wire        rs_valid, rs_ready, rs_valid2, rs_take2; wire [63:0] rs_data, rs_data2;   // two beats a cycle
    wire        rd_go, rd_discard, rd_done, rd_busy, rd_cdither, rd_cen, rd_csc, rd_err_stack, rs_abort; wire [28:0] rd_rt_base;
    wire [10:0] rd_rt_bpl, rd_rt_width, rd_ds_bpl; wire [51:0] rd_cc16; wire [3:0] rd_ccmask; wire [11:0] rd_rt_height;
    wire [12:0] rd_cx0, rd_cy0, rd_cx1, rd_cy1; wire [23:0] rd_cdepth; wire [7:0] rd_csten, rd_csmask;
    wire [1:0]  rd_rt_fmt; wire rd_depth16, rd_ds_en, rd_cden, rd_csen; wire [28:0] rd_ds_base;
    wire [31:0] rd_quads, rd_tris, rd_ezrej, rd_ezrej1, rd_frames, rd_errs, cp_draws;
    wire [255:0] rd_perf; wire rd_ferr;             // the last frame's cycle counters / stack violation
    wire [31:0] rd_tjobs, rd_tast, rd_thit, rd_tmiss;   // the texture unit's cumulative counters: status words 18-21
    wire [24*32-1:0] rd_prof;                           // the render engine's cumulative profile counters (b_render hwprof): JTAG words 24-47
    // FS_TEXQ stays b_render's default (no override here: with 2 live TEXes multi-texture shaders run about 8 cycles per
    // quad per sample)
    b_render #(.NLINES(B3D_NLINES), .MAXW(B3D_MAXW), .CELLS(B3D_CELLS), .NTS(B3D_NTS), .FS_NWARP(B3D_NWARP), .VOL3D(VOL3D), .F2_DUAL(B3D_F2_DUAL), .TC_PF(B3D_TC_PF),
               .TEX_SKIP(B3D_TEX_SKIP), .TEX_W2(B3D_TEX_W2), .FS_PRE(B3D_FS_PRE), .FS_XSFU(B3D_XSFU), .FS_XCLAMP(B3D_XCLAMP), .FS_QRINF(B3D_QRINF)) u_render (.clk(clk_sys), .rst(rst_ddr),
        .rs_valid, .rs_valid2, .rs_data, .rs_data2, .rs_ready, .rs_take2, .go(rd_go), .discard(rd_discard), .abort(rs_abort), .rt_base(rd_rt_base), .rt_bpl(rd_rt_bpl), .rt_width(rd_rt_width),
        .rt_height(rd_rt_height), .rt_fmt(rd_rt_fmt), .depth16(rd_depth16), .ds_en(rd_ds_en), .ds_base(rd_ds_base), .ds_bpl(rd_ds_bpl),
        .clear_c16(rd_cc16), .clear_dither(rd_cdither), .clear_color_en(rd_cen), .clear_sc_en(rd_csc),
        .clear_cmask(rd_ccmask), .clear_x0(rd_cx0), .clear_y0(rd_cy0), .clear_x1(rd_cx1), .clear_y1(rd_cy1),
        .clear_depth(rd_cdepth), .clear_stencil(rd_csten), .clear_depth_en(rd_cden), .clear_stencil_en(rd_csen),
        .clear_smask(rd_csmask), .done(rd_done), .busy(rd_busy),
        .r_en(r_en[5]), .r_addr(r_addr[145 +: 29]), .r_rdy(r_rdy[5]), .r_valid(r_valid[5]), .rd_data(rd_data),
        .t_en(r_en[4]), .t_addr(r_addr[116 +: 29]), .t_rdy(r_rdy[4]), .t_valid(r_valid[4]),
        .l_en(r_en[3]), .l_addr(r_addr[87 +: 29]), .l_rdy(r_rdy[3]), .l_valid(r_valid[3]),
        .fw_req(w_req[3]), .fw_en(w_en[3]), .fw_addr(w_addr[87 +: 29]), .fw_data(w_data[768 +: 256]),
        .fw_mask(w_mask[96 +: 32]), .fw_wren(w_wren[3]), .fw_wend(w_wend[3]), .fw_rdy(w_rdy[3]),
        .fw_wdf_rdy(w_wdf_rdy[3]),
        .c_quads(rd_quads), .c_tris(rd_tris), .c_ezrej(rd_ezrej), .c_ezrej1(rd_ezrej1), .c_frames(rd_frames), .c_errs(rd_errs),
        .err_stack(rd_err_stack), .perf(rd_perf), .frame_err(rd_ferr), .hwprof(rd_prof),
        .st_tjobs(rd_tjobs), .st_tast(rd_tast), .st_thit(rd_thit), .st_tmiss(rd_tmiss));

    // ---------------- board link ----------------
    wire lclk, lclk90, lclk270, rst_lclk, lclk_locked;
    link_clk #(.LINK_MHZ(LINK_MHZ)) u_lclk (.clk200(clk200), .rst(rst_s), .lclk(lclk), .lclk90(lclk90),
        .lclk270(lclk270), .rst_lclk(rst_lclk), .locked(lclk_locked));
    wire        l_tx_valid, l_tx_cnt2, l_tx_last, l_tx_ready, l_rx_valid, l_rx_cnt2, l_rx_last, l_rx_hdr, l_rx_ready;
    wire [63:0] l_tx_data, l_rx_data;
    wire        l_lock, l_trained, l_rx_clk_ok, l_far_up, l_training, l_guard_fail, l_tx_enabled;
    wire [7:0]  l_eye, l_phase, l_train_cnt;
    wire [31:0] l_xoff, l_tx_pkts, l_rx_words, l_rx_pkts, l_par, l_crc, l_seq, l_frm, l_drops, l_parmap, l_retx, l_disc;
    wire [8*32-1:0] l_diag;
    reg  [31:0] linkctl;
    link_core #(.ROLE("B"), .LINK_MHZ(LINK_MHZ), .MMCM_RST_CYCLES(LINK_MMCM_RST), .GUARD_CYCLES(LINK_GUARD),
                .IDLE_TIMEOUT(LINK_IDLE_TO), .TRAIN_WORDS(LINK_TRAIN_WORDS), .RETRY_CYCLES(LINK_RETRY),
                .RELOCK_CYCLES(LINK_RELOCK), .RETRAIN_WINDOW(LINK_RWIN), .SIM_SHIFTER(LINK_SIM_SHIFTER)) u_link (
        .clk(clk_sys), .rst(rst_s), .lclk(lclk), .lclk90(lclk90), .lclk270(lclk270), .rst_lclk(rst_lclk),
        .fill_en(linkctl[0]),
        .tx_valid(l_tx_valid), .tx_data(l_tx_data), .tx_cnt2(l_tx_cnt2), .tx_last(l_tx_last), .tx_ready(l_tx_ready),
        .rx_valid(l_rx_valid), .rx_data(l_rx_data), .rx_cnt2(l_rx_cnt2), .rx_last(l_rx_last), .rx_hdr(l_rx_hdr),
        .rx_ready(l_rx_ready), .pkt_done(), .pkt_ok(),
        .lock(l_lock), .trained(l_trained), .rx_clk_ok(l_rx_clk_ok), .far_up(l_far_up), .training(l_training),
        .guard_fail(l_guard_fail), .tx_enabled(l_tx_enabled), .xoff_events(l_xoff),
        .eye_width(l_eye), .phase(l_phase), .train_cnt(l_train_cnt),
        .tx_pkts(l_tx_pkts), .rx_words(l_rx_words), .rx_pkts(l_rx_pkts), .par_errs(l_par), .crc_errs(l_crc),
        .seq_errs(l_seq), .frm_errs(l_frm), .lock_drops(l_drops), .par_map(l_parmap), .retx(l_retx), .disc(l_disc),
        .diag(l_diag), .jm1(jm1), .jm2(jm2));
    wire        cw_valid, cw_first, cw_last, cw_ready, cw_old, ev_valid, ev_ready, lrd_req, lrd_valid, lrd_ready;
    wire [31:0] cw_data, c_cmd_pkts, c_lunk, lrd_data, c_cur_pkts, cb_consumed; wire [159:0] ev_data; wire [15:0] lrd_words;
    wire [B_LSTAT_WORDS*32-1:0] lstat;
    wire [31:0] c_rs_pkts;                          // RSTREAM packets received
    link_b_port #(.STATUS_CYCLES(LSTAT_CYCLES)) u_lport (
        .clk(clk_sys), .rst(rst_ddr),
        .tx_valid(l_tx_valid), .tx_data(l_tx_data), .tx_cnt2(l_tx_cnt2), .tx_last(l_tx_last), .tx_ready(l_tx_ready),
        .rx_valid(l_rx_valid), .rx_data(l_rx_data), .rx_cnt2(l_rx_cnt2), .rx_last(l_rx_last), .rx_hdr(l_rx_hdr),
        .rx_ready(l_rx_ready),
        .c_valid(cw_valid), .c_data(cw_data), .c_first(cw_first), .c_last(cw_last), .c_ready(cw_ready),
        .e_valid(ev_valid), .e_data(ev_data), .e_ready(ev_ready),
        .r_req(lrd_req), .r_words(lrd_words), .r_valid(lrd_valid), .r_data(lrd_data), .r_ready(lrd_ready),
        .rs_valid(rs_valid), .rs_valid2(rs_valid2), .rs_data(rs_data), .rs_data2(rs_data2), .rs_ready(rs_ready), .rs_take2(rs_take2),
        .fp_valid(fp_valid), .fp_x(fp_x), .fp_y(fp_y),
        .stat(lstat), .c_cmd_pkts(c_cmd_pkts), .c_unk(c_lunk), .c_cur_pkts(c_cur_pkts), .consumed(cb_consumed),
        .c_sync(), .c_rs_pkts(c_rs_pkts), .rs_consumed(), .rs_abort(rs_abort), .c_old(cw_old));

    // ---------------- command processor ----------------
    wire [15:0] cp_cmds, cp_errs; wire [31:0] cp_images; wire h_active;
    fb_cmd_b #(.SIM_MODE4(SIM_MODE4), .MAXW(B3D_MAXW), .VOL3D(VOL3D)) u_cp (
        .clk(clk_sys), .rst(rst_ddr),
        .c_valid(cw_valid), .c_data(cw_data), .c_first(cw_first), .c_last(cw_last), .c_ready(cw_ready),
        .c_old(cw_old), .sync_wipe(rs_abort),
        .e_valid(ev_valid), .e_data(ev_data), .e_ready(ev_ready),
        .r_req(lrd_req), .r_words(lrd_words), .r_valid(lrd_valid), .r_data(lrd_data), .r_ready(lrd_ready),
        .it_ridx(it_ridx2), .it_rvalid(it_rvalid2), .it_rdesc(it_rdesc2),
        .it_we(c_it_we), .it_widx(c_it_widx), .it_wvalid(c_it_wvalid), .it_wdesc(c_it_wdesc),
        .h_active(h_active), .x_own(c_x_own), .x_cmd_valid(c_x_cv), .x_op(c_x_op), .x_addr(c_x_addr), .x_len(c_x_len),
        .x_seed(c_x_seed), .x_flags(c_x_flags), .x_cmd_ready(x_crdy), .x_busy(x_busy),
        .x_wd_valid(c_x_wdv), .x_wd_data(c_x_wdd), .x_wd_ready(x_wdr),
        .x_rd_valid(x_rdv), .x_rd_data(x_rdd), .x_rd_ready(c_x_rdr),
        .x_mt_done(x_mtd), .x_mt_errors(x_mterr), .x_mt_first_bad(x_mtbad),
        .iw_cmd_valid(iw_cv), .iw_cmd_ready(iw_crdy), .iw_row(iw_row), .iw_pitch(iw_pitch), .iw_x(iw_x), .iw_w(iw_w),
        .iw_i0(iw_i0), .iw_npix(iw_npix), .iw_tfmt(iw_tfmt), .iw_sf(iw_sf),
        .iw_d_valid(iw_dv), .iw_d_data(iw_dd), .iw_d_ready(iw_drdy), .iw_busy(iw_busy),
        .xf_cmd_valid(xf_cv), .xf_cmd_ready(xf_crdy), .xf_op(xf_op), .xf_s_fmt(xf_sf), .xf_d_fmt(xf_df),
        .xf_s_row0(xf_s0), .xf_d_row0(xf_d0), .xf_s_pitch(xf_sp), .xf_d_pitch(xf_dp),
        .xf_s_x(xf_sx), .xf_s_y(xf_sy), .xf_s_w(xf_sw), .xf_s_h(xf_sh), .xf_d_x(xf_dx), .xf_d_y(xf_dy), .xf_d_w(xf_dw),
        .xf_d_h(xf_dh), .xf_hq(xf_hq), .xf_hu0(xf_hu0), .xf_vq(xf_vq), .xf_vu0(xf_vu0), .xf_hr2(xf_hr2), .xf_ha0(xf_ha0),
        .xf_vr2(xf_vr2), .xf_va0(xf_va0), .xf_mir_x(xf_mx), .xf_mir_y(xf_my), .xf_bottom_up(xf_bup), .xf_key(xf_key),
        .xf_rmw(xf_rmw), .xf_key_lo(xf_klo), .xf_key_hi(xf_khi), .xf_value(xf_val), .xf_mask(xf_mask), .xf_tfmt(xf_tf),
        .xf_j0(xf_j0), .xf_i0(xf_i0), .xf_npix(xf_np), .xf_busy(xf_busy), .xo_valid(xo_v), .xo_data(xo_d), .xo_ready(xo_rdy),
        .xf_alpha1(xf_a1), .xf_swz(xf_swz), .xf_swz_sel(xf_swz_sel), .xf_wrap(xf_wrap), .xf_px(xf_px), .xf_py(xf_py),
        .set_mode(c_set_mode), .set_mode_id(c_set_mode_id), .mode_busy(mode_busy), .mode_done(mode_done),
        .mode_ok(mode_ok), .video_on(video_on), .frames(st_frames),
        .pres_valid(c_pres_valid), .pres_base(c_pres_base), .pres_pitch(c_pres_pitch), .pres_w(c_pres_w),
        .pres_h(c_pres_h), .pres_fmt(c_pres_fmt), .pres_origin(c_pres_origin), .pres_now(c_pres_now),
        .pres_tag(c_pres_tag), .pres_done(pres_done), .cur_tag(cur_tag),
        .pm_we(pm_we), .pm_idx(pm_idx), .pm_data(pm_data), .pal_pub(pal_pub), .pal_busy(pal_busy),
        .cur_cmd(cr_cmd), .cur_op(cr_op), .cur_mode(cr_mode), .cur_take(cr_take), .cur_setpos(cr_setpos),
        .cur_hx(cr_hx), .cur_hy(cr_hy), .cur_x(cr_x), .cur_y(cr_y), .cs_wr(cs_wr), .cs_idx(cs_idx), .cs_data(cs_data),
        .cs_ready(cs_ready),
        .rd_go, .rd_discard, .rd_rt_base, .rd_rt_bpl, .rd_rt_width, .rd_rt_height, .rd_clear_c16(rd_cc16), .rd_clear_dither(rd_cdither),
        .rd_clear_en(rd_cen), .rd_clear_sc(rd_csc), .rd_clear_cmask(rd_ccmask), .rd_clear_x0(rd_cx0),
        .rd_clear_y0(rd_cy0), .rd_clear_x1(rd_cx1), .rd_clear_y1(rd_cy1), .rd_clear_depth(rd_cdepth),
        .rd_clear_stencil(rd_csten), .rd_rt_fmt, .rd_depth16, .rd_ds_en, .rd_ds_base, .rd_ds_bpl,
        .rd_clear_depth_en(rd_cden), .rd_clear_stencil_en(rd_csen), .rd_clear_smask(rd_csmask), .rd_done, .rd_frame_err(rd_ferr),
        .c_cmds(cp_cmds), .c_errs(cp_errs), .c_images(cp_images), .c_draws(cp_draws), .busy(cp_busy));

    // ---------------- commands: JTAG USER2 + bench port ----------------
    wire        j_cv; wire [63:0] j_cmd;
    generate if (JTAG) begin : g_jctl
        jtag_ctrl_reg #(.W(64)) u_jctl (.clk(clk_sys), .rst(rst_ddr), .cmd_valid(j_cv), .cmd_data(j_cmd));
    end else begin : g_nojctl
        assign j_cv = 1'b0; assign j_cmd = '0;
    end endgenerate
    wire        h_cv  = j_cv | dbg_cmd_valid;
    wire [63:0] h_cmd = j_cv ? j_cmd : dbg_cmd;
    // op 0x06 LINKCTL [31:0] is executed here (b_bringup ignores it): b0 = fill words on B's transmit lane
    always @(posedge clk_sys)
        if (rst_s) linkctl <= LINKCTL_DEFAULT;
        else if (h_cv && h_cmd[63:56] == 8'h06) linkctl <= h_cmd[31:0];

    // ---------------- bring-up harness (BRINGUP; without it the screen stays black until the first PRESENT, KEY1 does
    // nothing, the JTAG USER2 commands other than LINKCTL are ignored, status words 13-17 hold the render counters and
    // 18-21 read 0) ----------------
    wire        fill_done, h_busy, key_down; wire [9:0] cur_img;
    wire [31:0] h_cmds, h_errs, h_fills, h_mt_runs, h_mt_errors, h_keys; wire [29:0] h_mt_bad; wire [63:0] h_last;
    generate if (!BRINGUP) begin : g_nobu
        assign h_it_we = 1'b0; assign h_it_widx = '0; assign h_it_wdesc = '0; assign it_ridx = '0;
        assign h_x_cv = 1'b0; assign h_x_op = '0; assign h_x_addr = '0; assign h_x_len = '0; assign h_x_seed = '0;
        assign h_x_flags = '0; assign h_x_wdv = 1'b0; assign h_x_wdd = '0;
        assign h_set_mode = 1'b0; assign h_set_mode_id = '0;
        assign h_pres_valid = 1'b0; assign h_pres_base = '0; assign h_pres_pitch = '0; assign h_pres_w = '0; assign h_pres_h = '0;
        assign h_pres_565 = 1'b0; assign h_pres_origin = 1'b0; assign h_pres_now = 1'b0; assign h_pres_tag = '0;
        assign fill_done = 1'b0; assign h_busy = 1'b0; assign h_active = 1'b0; assign key_down = 1'b0; assign cur_img = '0;
        assign h_cmds = '0; assign h_errs = '0; assign h_fills = '0; assign h_mt_runs = '0; assign h_mt_errors = '0;
        assign h_keys = '0; assign h_mt_bad = '0; assign h_last = '0;
        wire _unused_bu = &{1'b0, key1_n, it_rvalid, x_crdy, x_wdr, x_mtd, x_mterr, x_mtbad};
    end else begin : g_bu
    b_bringup #(.IMG_SET(IMG_SET), .DEBOUNCE(DEBOUNCE), .LONG_PRESS(LONG_PRESS)) u_bu (
        .clk(clk_sys), .rst(rst_ddr), .ext_busy(cp_busy), .key1_n(key1_n), .cmd_valid(h_cv), .cmd(h_cmd),
        .it_we(h_it_we), .it_widx(h_it_widx), .it_wdesc(h_it_wdesc), .it_ridx(it_ridx), .it_rvalid(it_rvalid), .it_rdesc(it_rdesc),
        .x_cmd_valid(h_x_cv), .x_op(h_x_op), .x_addr(h_x_addr), .x_len(h_x_len), .x_seed(h_x_seed), .x_flags(h_x_flags),
        .x_cmd_ready(x_crdy), .x_wd_valid(h_x_wdv), .x_wd_data(h_x_wdd), .x_wd_ready(x_wdr),
        .x_mt_done(x_mtd), .x_mt_errors(x_mterr), .x_mt_first_bad(x_mtbad),
        .set_mode(h_set_mode), .set_mode_id(h_set_mode_id), .mode_busy(mode_busy | cp_busy), .mode_done(mode_done),
        .mode_ok(mode_ok), .cur_mode(cur_mode),
        .pres_valid(h_pres_valid), .pres_base(h_pres_base), .pres_pitch(h_pres_pitch), .pres_w(h_pres_w), .pres_h(h_pres_h),
        .pres_565(h_pres_565), .pres_origin(h_pres_origin), .pres_now(h_pres_now), .pres_tag(h_pres_tag),
        .fill_done(fill_done), .busy(h_busy), .active(h_active), .key_down(key_down), .cur_img(cur_img),
        .st_cmds(h_cmds), .st_errs(h_errs), .st_fills(h_fills), .st_mt_runs(h_mt_runs), .st_mt_errors(h_mt_errors),
        .st_keys(h_keys), .st_mt_first_bad(h_mt_bad), .st_last_cmd(h_last));
    end endgenerate

    // ---------------- status ----------------
    reg [31:0] ms_div, uptime_ms, c_pres;
    reg        lock_fail;
    always @(posedge clk_sys) begin
        if (rst_s) begin ms_div <= 0; uptime_ms <= 0; end
        else if (ms_div == MS_CYCLES - 1) begin ms_div <= 0; uptime_ms <= uptime_ms + 1; end
        else ms_div <= ms_div + 1;
        if (rst_ddr) begin c_pres <= 0; lock_fail <= 1'b0; end
        else begin
            if (pres_done) c_pres <= c_pres + 1;
            if (mode_done && !mode_ok) lock_fail <= 1'b1;
        end
    end
    // [25:16] = image table entry on screen (presented tag: harness = index, command processor = handle); [26] 3D render
    // engine busy, [27] a fragment-program stack violation since reset (3D); [28] the fragment core has SQRT / RCPM / RSQM,
    // [29] it has CLAMP (constants of the build; 0 on a board B built without them — a program with those opcodes
    // there issues them as no-ops); [30] / [31] the fragment core lacks the integer divider / SIN / COS (the libraries'
    // compilers then replace those ops): this core has both, so 0; [13] the band preload follows the next band a BAND item
    // names (board A may then leave bands out: the libraries set FLUSH flags bit 10 only with it); [14] COPY_IMAGE takes
    // flags 3..6 (alpha = 255, swizzle, tile, list) and runs one job while the next waits
    wire [31:0] flags = {2'h0, B3D_XCLAMP, B3D_XSFU, rd_err_stack, rd_busy, cur_tag[9:0], 1'b0, 1'b1, 1'b1, cp_busy, lock_fail, key_down, x_busy, fill_done, cur_valid, cur_mode,
                         mode_busy, video_on, mmcm_locked, calib_done};
    wire [31:0] lflags = {l_eye, l_phase, 8'h0, 1'b0, l_guard_fail, l_tx_enabled, l_training, l_far_up, l_trained,
                          l_lock, 1'b0};
    wire [31:0] sw [0:B_STATUS_WORDS-1];
    assign sw[0]  = B_STATUS_LAYOUT;
    assign sw[1]  = BUILD_ID;
    assign sw[2]  = uptime_ms;
    assign sw[3]  = flags;
    assign sw[4]  = st_frames;
    assign sw[5]  = st_underruns;
    assign sw[6]  = st_pix_khz;
    assign sw[7]  = st_frame_cyc;
    assign sw[8]  = st_mode_ok;
    assign sw[9]  = st_mode_err;
    assign sw[10] = c_pres;
    assign sw[11] = pres_dropped;
    assign sw[12] = cur_tag;
    // words 13..17 without the harness (readable by JTAG during a game, no UDP): the
    // render engine's cumulative quads launched, rejected late (after quad_setup) and early (ahead of it), triangles, frames
    assign sw[13] = BRINGUP ? h_cmds : rd_quads;
    assign sw[14] = BRINGUP ? h_last[63:32] : rd_ezrej;
    assign sw[15] = BRINGUP ? h_last[31:0] : rd_ezrej1;
    assign sw[16] = BRINGUP ? h_errs : rd_tris;
    assign sw[17] = BRINGUP ? h_mt_runs : rd_frames;
    // 18-21 (layout 3): the texture unit's tap jobs, A-stage stall cycles, cache hits, cache misses —
    // cumulative, so a watch differences successive samples; the bring-up harness shows its memtest / fill / key words instead
    assign sw[18] = BRINGUP ? h_mt_errors : rd_tjobs;
    assign sw[19] = BRINGUP ? {2'b00, h_mt_bad} : rd_tast;
    assign sw[20] = BRINGUP ? h_fills : rd_thit;
    assign sw[21] = BRINGUP ? h_keys : rd_tmiss;
    assign sw[22] = lflags;                          // board link flags (bit 0 unused on B)
    assign sw[23] = {c_cmd_pkts[15:0], cp_errs};     // {CMD packets received, command errors}
    genvar gi;
    generate for (gi = 0; gi < B_STATUS_WORDS; gi = gi + 1) begin : g_vec
        assign status_vec[gi*32 +: 32] = sw[gi];
    end endgenerate
    // link STATUS packet to A (fb_pkg B_LSTAT_WORDS = 32): A shows words 0, 1, 3, packs 2 / 5 and shows 7..31.
    // Words 12-19 (STATUS layout 4): our receiver's diagnostics (link_rx_diag histogram, last error, parity map),
    // our RX FIFO's XOFF events, our transmitter's {timeouts, rewinds}, our receiver's {NAKs, discards}, CMD
    // packets received, words received. Words 20-23 (layout 5): CMD words taken from the command
    // buffer (credit), {cursor fast-path packets, palette publishes}, {cursor commands, READ / FILL / COPY ops},
    // READ / FILL / COPY pixels. Words 24-31 (layout 7): the render engine's cycle counters of the
    // last frame (b_render perf 0..7: frame, flush, band change, draw change, FS busy, band set-up, stream empty, ROP stall).
    assign lstat = {rd_perf,
                    xf_pixels, {st_cur_cmds[15:0], xf_ops[15:0]}, {c_cur_pkts[15:0], st_pal_pub[15:0]}, cb_consumed,
                    l_rx_words, c_cmd_pkts, l_disc, l_retx, l_xoff, l_parmap, l_diag[3*32 +: 32], l_diag[2*32 +: 32],
                    iw_pixels, cp_images, {l_train_cnt, 7'h0, linkctl[0], l_diag[7*32 +: 16]}, {l_crc[15:0], l_par[15:0]},
                    {cp_errs, cp_cmds}, c_pres, st_mode_err, st_mode_ok, st_pix_khz, st_underruns, st_frames, flags};
    generate if (JTAG) begin : g_jst
        // JTAG USER1 frame: {seq, status_vec (MSB = word B_STATUS_WORDS-1), seq, A5}, see tools/b_jtag.tcl
        link_status_jtag #(.FW((B_STATUS_WORDS + 24) * 32), .SNAP_CYCLES(1_000_000)) u_jtag (
            .clk(clk_sys), .rst(rst_s), .fields({rd_prof, status_vec}));
    end endgenerate

    // LED0 = DDR calibrated, LED1 = video running and the board link trained (blinks while the
    // link is not up, steady when both are)
    reg [24:0] blink = 0;
    always @(posedge clk_sys) blink <= blink + 25'd1;
    assign led = {video_on & mmcm_locked & (l_trained & l_far_up | blink[24]), calib_done};
    wire _unused_link = &{1'b0, lclk_locked, l_rx_clk_ok, l_tx_pkts, l_rx_pkts, l_seq, l_frm, l_drops, c_lunk, h_busy,
                         l_diag, st_fast_moves, rd_quads, rd_tris, rd_ezrej, rd_frames, rd_errs, cp_draws, c_rs_pkts};
endmodule
`default_nettype wire
