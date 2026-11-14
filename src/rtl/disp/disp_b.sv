// disp_b — board B display subsystem:
//   video_mmcm (one MMCM, DRP per mode) -> pixel / serial clocks
//   scan_reader (DDR, mem_arb read port) -> display_line_buffer -> disp_out_b (video_timing, palette lookup,
//   hardware cursor, TMDS, OSERDES); disp_ctl_b = the clk_sys side of the palette and the cursor
// plus the mode controller and status counters.
// Mode change: stop the reader and hold the pixel domain in reset (the screen goes dark), wait until
// no read is outstanding, reprogram the MMCM, wait for LOCKED, load the mode's timing, restart. After reset
// (= DDR calibration) the default mode is set the same way. The presented image survives a mode change.
// All timing inputs of the pixel domain change only while it is held in reset.
`default_nettype none
module disp_b #(
    parameter int DEFAULT_MODE = 1,                 // 800x600
    parameter bit SIM_MODE4    = 1'b0,              // accept mode ids 5, 6 (64x48 / 80x40 simulation test modes)
    parameter int LOCK_TIMEOUT = 2_000_000,         // clk cycles
    parameter int MS_CYCLES    = 100_000,           // clk cycles per ms (pixel clock measurement)
    parameter int MAXW         = 1280,              // the widest mode: the line buffer holds LB_LINES lines of MAXW pixels;
    parameter int LB_LINES     = 16                 // a mode wider than MAXW is refused (mode_ok 0, as an unknown id)
)(
    input  wire         clk, rst,                   // clk_sys; rst = DDR-side reset (held until calibration)
    input  wire         clkin200,                   // 200 MHz for the video MMCM
    // mode control
    input  wire         set_mode,                   // pulse; accepted when !mode_busy
    input  wire [2:0]   set_mode_id,
    output wire         mode_busy,
    output reg          mode_done,                  // pulse: a requested change finished, mode_ok = result
    output reg          mode_ok,
    output reg  [2:0]   cur_mode,
    output wire         video_on,                   // pixel domain running
    output wire         mmcm_locked,
    // present
    input  wire         pres_valid,
    input  wire [29:0]  pres_base,
    input  wire [15:0]  pres_pitch,
    input  wire [12:0]  pres_w, pres_h,
    input  wire [1:0]   pres_fmt,                   // 0 RGBA8, 1 RGB565, 2 P8
    input  wire         pres_origin, pres_now,
    input  wire [31:0]  pres_tag,
    output wire         pres_done, cur_valid,
    output wire [31:0]  cur_tag, pres_dropped,
    // palette (SET_PALETTE) and hardware cursor (CURSOR, fast path), see disp_ctl_b
    input  wire         pm_we,
    input  wire [7:0]   pm_idx,
    input  wire [23:0]  pm_data,
    input  wire         pal_pub,
    output wire         pal_busy,
    input  wire         cr_cmd,                     // cursor command (disp_ctl_b cur_*)
    input  wire [2:0]   cr_op,
    input  wire [1:0]   cr_mode,
    input  wire         cr_take, cr_setpos,
    input  wire [5:0]   cr_hx, cr_hy,
    input  wire [15:0]  cr_x, cr_y,
    input  wire         cs_wr,
    input  wire [11:0]  cs_idx,
    input  wire [31:0]  cs_data,
    output wire         cs_ready,
    input  wire         fp_valid,
    input  wire [15:0]  fp_x, fp_y,
    output wire [31:0]  st_pal_pub, st_fast_moves, st_cur_cmds,
    // DDR read port (mem_arb, MUST_RUN)
    output wire         r_en,
    output wire [28:0]  r_addr,
    input  wire         r_rdy, r_valid,
    input  wire [255:0] rd_data,
    // HDMI (TMDS_33)
    output wire         hdmi_clk_p, hdmi_clk_n,
    output wire [2:0]   hdmi_data_p, hdmi_data_n,
    // status
    output reg  [31:0]  st_frames,                  // vsyncs while running
    output reg  [31:0]  st_underruns,               // display lines consumed without the next line ready
    output reg  [31:0]  st_pix_khz,                 // measured pixel clock, kHz (1 ms gate)
    output reg  [31:0]  st_frame_cyc,               // clk cycles between the last two vsyncs
    output reg  [31:0]  st_mode_ok,                 // completed mode changes (incl. the boot one)
    output reg  [31:0]  st_mode_err                 // rejected id, request while busy, MMCM not locked
);
`include "video_modes.svh"

    // ---------------- video MMCM ----------------
    wire pix_clk, ser_clk, vm_busy, vm_done, vm_ok, vm_locked_raw;
    reg  vm_req;
    reg  [2:0] tgt;
    video_mmcm #(.LOCK_TIMEOUT(LOCK_TIMEOUT)) u_vm (
        .clk(clk), .rst(rst), .clkin(clkin200), .req(vm_req), .req_mode(tgt), .busy(vm_busy), .done(vm_done),
        .ok(vm_ok), .locked(mmcm_locked), .locked_raw(vm_locked_raw), .pix_clk(pix_clk), .ser_clk(ser_clk));

    // ---------------- mode controller ----------------
    typedef enum logic [2:0] {M_BOOT, M_IDLE, M_STOP, M_DRP, M_START} ms_t;
    ms_t        ms;
    reg         run;
    reg  [97:0] tim;                                // vm_timing() of the running mode
    reg  [4:0]  settle;
    wire        scan_idle;
    wire        id_ok = ((set_mode_id < 3'd5) || (SIM_MODE4 && (set_mode_id == 3'd5 || set_mode_id == 3'd6))) &&
                        (vm_timing(set_mode_id)[97:86] <= 12'(MAXW));
    if (vm_timing(3'(DEFAULT_MODE))[97:86] > 12'(MAXW))
        $error("disp_b: DEFAULT_MODE is wider than MAXW");
    assign mode_busy = (ms != M_IDLE);
    assign video_on  = run;

    always @(posedge clk) begin
        mode_done <= 1'b0; vm_req <= 1'b0;
        if (rst) begin
            ms <= M_BOOT; run <= 1'b0; tgt <= 3'(DEFAULT_MODE); cur_mode <= 3'(VM_STATIC_MODE);
            tim <= vm_timing(3'(VM_STATIC_MODE)); mode_ok <= 1'b0; settle <= 0; st_mode_ok <= 0; st_mode_err <= 0;
        end else begin
            if (set_mode && ms != M_IDLE) st_mode_err <= st_mode_err + 1;
            case (ms)
                M_BOOT: begin tgt <= 3'(DEFAULT_MODE); settle <= 0; ms <= M_STOP; end
                M_IDLE: if (set_mode) begin
                    if (id_ok) begin tgt <= set_mode_id; settle <= 0; ms <= M_STOP; end
                    else begin mode_done <= 1'b1; mode_ok <= 1'b0; st_mode_err <= st_mode_err + 1; end
                end
                M_STOP: begin                         // reader stopped, pixel domain in reset
                    run <= 1'b0;
                    if (settle != 5'd31) settle <= settle + 5'd1;
                    else if (scan_idle && !vm_busy) begin vm_req <= 1'b1; ms <= M_DRP; end
                end
                M_DRP: if (vm_done) begin
                    if (vm_ok) begin tim <= vm_timing(tgt); cur_mode <= tgt; ms <= M_START; end
                    else begin mode_done <= 1'b1; mode_ok <= 1'b0; st_mode_err <= st_mode_err + 1; ms <= M_IDLE; end
                end
                M_START: begin run <= 1'b1; mode_ok <= 1'b1; mode_done <= 1'b1; st_mode_ok <= st_mode_ok + 1; ms <= M_IDLE; end
                default: ms <= M_IDLE;
            endcase
        end
    end
    wire [11:0] t_ha = tim[97:86], t_hf = tim[85:74], t_hs = tim[73:62], t_hb = tim[61:50];
    wire [11:0] t_va = tim[49:38], t_vf = tim[37:26], t_vs = tim[25:14], t_vb = tim[13:2];
    wire        t_hp = tim[1], t_vp = tim[0];

    // ---------------- pixel-domain reset: asynchronous assert, release synchronous to pix_clk ----------------
    wire px_rst;
    rst_sync #(.STAGES(4)) u_pxrst (.clk(pix_clk), .arst(~run | ~vm_locked_raw), .rst(px_rst));

    // ---------------- pixel -> clk crossings ----------------
    wire fs_tog, ur_tog;
    (* ASYNC_REG = "TRUE" *) reg [2:0] fs_s, ur_s;
    always @(posedge clk) begin fs_s <= {fs_s[1:0], fs_tog}; ur_s <= {ur_s[1:0], ur_tog}; end
    wire frame_sync = run & (fs_s[2] ^ fs_s[1]);
    wire underrun   = run & (ur_s[2] ^ ur_s[1]);

    // ---------------- palette / cursor control (clk_sys) ----------------
    wire        sc_fstart, pal_req, pal_we, cs_we, mb_req, mb_ack; wire [8:0] pal_wa; wire [23:0] pal_wd;
    wire [12:0] cs_wa; wire [31:0] cs_wd; wire [47:0] mb_state;
    disp_ctl_b u_ctl (
        .clk(clk), .rst(rst), .video_on(run), .frame_start(sc_fstart),
        .pm_we(pm_we), .pm_idx(pm_idx), .pm_data(pm_data), .pal_pub(pal_pub), .pal_busy(pal_busy), .pal_req(pal_req),
        .pal_we(pal_we), .pal_wa(pal_wa), .pal_wd(pal_wd),
        .cur_cmd(cr_cmd), .cur_op(cr_op), .cur_mode(cr_mode), .cur_take(cr_take), .cur_setpos(cr_setpos),
        .cur_hx(cr_hx), .cur_hy(cr_hy), .cur_x(cr_x), .cur_y(cr_y), .cs_wr(cs_wr), .cs_idx(cs_idx), .cs_data(cs_data),
        .cs_ready(cs_ready), .fp_valid(fp_valid), .fp_x(fp_x), .fp_y(fp_y),
        .cs_we(cs_we), .cs_wa(cs_wa), .cs_wd(cs_wd), .mb_req(mb_req), .mb_state(mb_state), .mb_ack(mb_ack),
        .c_pal_pub(st_pal_pub), .c_fast_moves(st_fast_moves), .c_cur_cmds(st_cur_cmds));

    // ---------------- scan reader ----------------
    wire        lb_en, lb_done, lb_fs, lb_avail; wire [7:0] lb_idx; wire [255:0] lb_data;
    scan_reader u_scan (
        .clk(clk), .rst(rst), .run(run), .idle(scan_idle), .frame_sync(frame_sync),
        .mode_w(t_ha[10:0]), .mode_h(t_va[10:0]),
        .pres_valid(pres_valid), .pres_base(pres_base), .pres_pitch(pres_pitch), .pres_w(pres_w), .pres_h(pres_h),
        .pres_fmt(pres_fmt), .pres_origin(pres_origin), .pres_now(pres_now), .pres_tag(pres_tag),
        .pal_bank(pal_req), .frame_start(sc_fstart),
        .pres_done(pres_done), .cur_valid(cur_valid), .cur_tag(cur_tag), .c_dropped(pres_dropped),
        .r_en(r_en), .r_addr(r_addr), .r_rdy(r_rdy), .r_valid(r_valid), .rd_data(rd_data),
        .lb_en(lb_en), .lb_idx(lb_idx), .lb_data(lb_data), .lb_line_done(lb_done), .lb_frame_start(lb_fs),
        .lb_avail(lb_avail));

    // ---------------- line buffer (16 lines x 1280 px) ----------------
    wire        rd_en, rd_line_done, rd_frame_sync, rd_avail; wire [10:0] rd_idx; wire [31:0] rd_px;
    display_line_buffer #(.N_LINES(LB_LINES), .MAX_BURSTS_PER_LINE((MAXW + 7) / 8), .MAX_PIXELS_PER_LINE(MAXW)) u_lb (
        .wr_clk(clk), .wr_rst(rst), .wr_burst_en(lb_en), .wr_burst_idx(lb_idx), .wr_burst_data(lb_data),
        .wr_line_done(lb_done), .wr_frame_sync(lb_fs), .wr_buf_avail(lb_avail),
        .rd_clk(pix_clk), .rd_rst(px_rst), .rd_pixel_en(rd_en), .rd_pixel_idx(rd_idx), .rd_pixel(rd_px),
        .rd_line_done(rd_line_done), .rd_frame_sync(rd_frame_sync), .rd_buf_avail(rd_avail));

    // ---------------- timing + palette + cursor + TMDS ----------------
    disp_out_b u_dout (
        .pixel_clk(pix_clk), .serial_clk_5x(ser_clk), .rst_n(~px_rst),
        .vt_h_active(t_ha), .vt_h_fp(t_hf), .vt_h_sync_w(t_hs), .vt_h_bp(t_hb),
        .vt_v_active(t_va), .vt_v_fp(t_vf), .vt_v_sync_w(t_vs), .vt_v_bp(t_vb),
        .vt_hsync_pol(t_hp), .vt_vsync_pol(t_vp),
        .lb_rd_pixel_en(rd_en), .lb_rd_pixel_idx(rd_idx), .lb_rd_pixel(rd_px),
        .lb_rd_line_done(rd_line_done), .lb_rd_frame_sync(rd_frame_sync), .lb_rd_buf_avail(rd_avail),
        .frame_sync_toggle(fs_tog),
        .wclk(clk), .pal_we(pal_we), .pal_wa(pal_wa), .pal_wd(pal_wd), .cs_we(cs_we), .cs_wa(cs_wa), .cs_wd(cs_wd),
        .mb_req(mb_req), .mb_state(mb_state), .mb_ack(mb_ack),
        .hdmi_clk_p(hdmi_clk_p), .hdmi_clk_n(hdmi_clk_n), .hdmi_data_p(hdmi_data_p), .hdmi_data_n(hdmi_data_n),
        .dbg_lb_underrun_toggle(ur_tog));

    // ---------------- status ----------------
    // pixel clock: free-running gray counter in pix_clk, sampled every ms
    reg  [19:0] pc_bin = 20'h0, pc_gray = 20'h0;
    always @(posedge pix_clk) begin pc_bin <= pc_bin + 20'd1; pc_gray <= (pc_bin + 20'd1) ^ ((pc_bin + 20'd1) >> 1); end
    (* ASYNC_REG = "TRUE" *) reg [19:0] pg_s1, pg_s2;
    always @(posedge clk) begin pg_s1 <= pc_gray; pg_s2 <= pg_s1; end
    reg  [19:0] pg_bin, pc_prev;
    integer gi;
    always @(*) begin
        pg_bin[19] = pg_s2[19];
        for (gi = 18; gi >= 0; gi = gi - 1) pg_bin[gi] = pg_bin[gi + 1] ^ pg_s2[gi];
    end
    reg  [31:0] ms_cnt, fc;
    always @(posedge clk) begin
        if (rst) begin
            ms_cnt <= 0; pc_prev <= 0; st_pix_khz <= 0; st_frames <= 0; st_underruns <= 0; st_frame_cyc <= 0; fc <= 0;
        end else begin
            if (ms_cnt == MS_CYCLES - 1) begin ms_cnt <= 0; st_pix_khz <= {12'h0, pg_bin - pc_prev}; pc_prev <= pg_bin; end
            else ms_cnt <= ms_cnt + 1;
            if (underrun) st_underruns <= st_underruns + 1;
            if (frame_sync) begin st_frames <= st_frames + 1; st_frame_cyc <= fc; fc <= 0; end
            else if (fc != 32'hFFFFFFFF) fc <= fc + 1;
        end
    end
endmodule
`default_nettype wire
