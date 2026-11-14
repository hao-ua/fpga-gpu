// fb_cmd_b — board B command processor, clk_sys. Executes the host
// commands board A forwards over the link strictly in order; a command starts after the previous one has
// finished (its DDR writes accepted by the arbiter, its READ_DATA handed to the link).
// Input: word stream from link_b_port (w0 = {0, type, body bytes} with c_first, w1 = seq, body words, c_last).
// Bodies (little-endian words):
//   SET_MODE     mode                                 -> disp_b; EV_MODE {mode} / ERR_RANGE / ERR_MODE
//   PRESENT      {flags, handle}, frame id             flags b0 at once, b1 hold until shown -> EV_PRESENT
//   FENCE        id                                   -> EV_FENCE {id}
//   CREATE_IMAGE {usage, handle}, base, {flags, format, pitch}, {height, width}, {0, vol, layers, levels}
//                flags: b0 origin, b2:1 layout, b4:3 log2(samples); vol: a 3D texture of layers = depth, one sample,
//                width, height <= 256, pitch = width * bpp, levels up to 1 + log2(max(width, height, depth))
//   DESTROY      {0, handle}
//   WRITE_IMAGE  {layer, level, handle}, {0, flags, format}, {y, x}, {h, w}, first pixel, 0, pixels...
//   READ_IMAGE   {layer, level, handle}, {0, flags, format}, {y, x}, {h, w}, first pixel, count (0 = rest), tag
//   FILL_IMAGE   {layer, level, handle}, {0, mask, format}, {y, x}, {h, w}, value
//   COPY_IMAGE   {slayer, slevel, src}, {dlayer, dlevel, dst}, {sy, sx}, {sh, sw}, {dy, dx}, {dh, dw}, flags
//                (b0 mirror x, b1 mirror y, b2 source colour key), key low, key high
//   SET_PALETTE  {count, first}, count x 0x00RRGGBB
//   CURSOR       {.., op}: op 1 SHAPE {0, first}, pixels...; op 2 SHOW {0, flags, mode, 2}, {0, hot_y, hot_x},
//                {y, x} (flags b0 take the new shape, b1 set the position); op 3 HIDE; op 4 MOVE {y, x}
//   CURSOR_POS   {y, x} (sequenced move; the fast path bypasses this processor)
//   MEM_WRITE    {0, board}, addr, data...            MEM_READ {0, board}, addr, len, tag
//   MEMTEST      {0, board}, addr, len, seed, flags   (addr, len 32-byte aligned) -> EV_MEMTEST
//   CREATE_BUFFER {usage, handle}, base, size   DESTROY {0, handle}   WRITE_BUFFER {0, handle}, offset, data...
//   READ_BUFFER  {0, handle}, offset, len, tag  — B buffers (handle bit 15 = 1, bit 14 = 0): fragment programs
//   DRAW_STATE   {0, draw id}, 23 words, n uniforms x 4 words: the draw's bucket in the
//                GPU-internal DDR region (DS_BYTE + id * 2048) for b_render: words 0..10 as given, the program's word
//                address (buffer + byte offset, inside the buffer), {n, length}, the constant alpha word (h14), 0, 0,
//                then the 8 samplers (h15..h22) resolved from the image table (base, size, format, levels, cube = 6
//                layers, 3D = vol with its depth and wrap r) and the uniforms
//   FLUSH        {flags, colour image}, clear colour {G, R}, {A, B} (channel units), {stencil, depth}, {y0, x0},
//                {y1, x1}, {stencil clear mask[23:16], depth-stencil image handle[15:0] (0: none)}: renders one frame
//                of the render stream (b_render) into the image (RGBA8 / RGBA4444 / RGBA5551 / RGB565, one sample,
//                width <= MAXW, pitch a multiple of 32 bytes; the depth-stencil image D24S8 of the same size, pitch a
//                multiple of 32 bytes covering the colour pitch's pixels); flags b0 clear colour, b1 clear scissor,
//                b5:2 clear colour mask, b6 dither, b7 clear depth, b8 clear stencil, b9 depth attachment of 16 bits
//   other        -> ERR_TYPE
// Image handles: bit 15 = 1 (board B), bit 14 = 1 (image), bits 13:10 = 0, bits 9:0 = table index.
// CREATE_IMAGE checks: storage format RGBA8 / RGB565 / P8 / RGBA4444 / RGBA5551 (the last two, like P8,
// transfer RAW only: texture storage for the TMU) / D24S8 (RAW only: the depth-stencil image of a FLUSH);
// width, height 1..2048 (MAX_TEXTURE_SIZE 2048); levels 1..12 and
// at most 1 + log2(max(width, height)); layers 1..255; samples 1, or 4 with one level; layout 0 (linear);
// pitch >= width * bpp and a multiple of bpp, = width * bpp when levels > 1 (ERR_RANGE); base 32-byte
// aligned and [base, base + layers * layer size * samples) inside the heap [HEAP_LO, HEAP_HI) (ERR_HEAP).
// Image transfers (WRITE / READ / FILL / COPY): a valid single-sample image (ERR_HANDLE); level < levels,
// layer < layers, transfer format known (P8: RAW only; COPY: P8 with P8 only), w, h >= 1 and the rectangle inside
// the level, pixel ranges inside the rectangle (ERR_RANGE); WRITE payload a multiple of the pixel size (ERR_LEN).
// Successful CREATE / DESTROY / WRITE / FILL / COPY_IMAGE, SET_PALETTE, CURSOR and MEM_WRITE send no event (FENCE
// reports completion); MEM_READ and READ_IMAGE answer with READ_DATA packets of up to RD_CHUNK bytes
// {seq, tag, offset, bytes, data} (READ_IMAGE: offset counts bytes from pixel 0 of the rectangle).
// Sharing with the bring-up harness (b_bringup): this processor starts a command only while the harness is not
// active (h_active), and the harness starts nothing while this processor is busy, so xfer_lin, the image table
// write port and the display controls have one user at a time. Presents from the harness carry tag bit 31 = 0,
// this processor's {1, handle}, which attributes pres_done.
`default_nettype none
module fb_cmd_b
    import img_pkg::*;
#(
    parameter bit    SIM_MODE4    = 1'b0,          // accept mode ids 5, 6 (simulation test modes; 4 = 1024 x 768)
    parameter int    HOLD_TIMEOUT = 50_000_000,    // 500 ms: a held PRESENT gives up waiting (e.g. replaced by KEY1)
    parameter [31:0] HEAP_LO      = 32'h1000_0000, // 256 MB
    parameter [31:0] HEAP_HI      = 32'h4000_0000, // 1 GB (exclusive)
    parameter [29:0] DS_BYTE      = 30'h0800_0000, // draw buckets (b_render DS_BASE * 4)
    parameter int    MAXW         = 2048,          // b_rop band width
    parameter bit    VOL3D        = 1'b0           // 3D images accepted (tmu24 VOL3D); 0: a volume CREATE_IMAGE is ERR_RANGE, which is
                                                   // what the host's g2r_has_3d probe expects of a B without hardware 3D
)(
    input  wire         clk, rst,
    // commands
    input  wire         c_valid,
    input  wire [31:0]  c_data,
    input  wire         c_first, c_last,
    output wire         c_ready,
    input  wire         c_old,                   // the head word arrived before the last wiping SYNC (link_b_port)
    input  wire         sync_wipe,               // one cycle: that SYNC (the render buffer is emptied now)
    // events
    output reg          e_valid,
    output reg  [159:0] e_data,
    input  wire         e_ready,
    // READ_DATA packets
    output reg          r_req,
    output reg  [15:0]  r_words,
    output wire         r_valid,
    output wire [31:0]  r_data,
    input  wire         r_ready,
    // image table: read port, write port
    output reg  [9:0]   it_ridx,
    input  wire         it_rvalid,
    input  img_desc_t   it_rdesc,
    output reg          it_we,
    output reg  [9:0]   it_widx,
    output reg          it_wvalid,
    output img_desc_t   it_wdesc,
    // xfer_lin (memory debug / test); x_own = this processor drives it
    input  wire         h_active,
    output reg          x_own,
    output reg          x_cmd_valid,
    output reg  [1:0]   x_op,
    output reg  [29:0]  x_addr, x_len,
    output reg  [31:0]  x_seed,
    output reg  [1:0]   x_flags,
    input  wire         x_cmd_ready, x_busy,
    output wire         x_wd_valid,
    output wire [31:0]  x_wd_data,
    input  wire         x_wd_ready,
    input  wire         x_rd_valid,
    input  wire [31:0]  x_rd_data,
    output wire         x_rd_ready,
    input  wire         x_mt_done,
    input  wire [31:0]  x_mt_errors,
    input  wire [29:0]  x_mt_first_bad,
    // image write engine (img_write)
    output reg          iw_cmd_valid,
    input  wire         iw_cmd_ready,
    output reg  [29:0]  iw_row,
    output reg  [15:0]  iw_pitch,
    output reg  [12:0]  iw_x, iw_w, iw_i0,
    output reg  [23:0]  iw_npix,
    output reg  [7:0]   iw_tfmt,
    output reg  [1:0]   iw_sf,
    output wire         iw_d_valid,
    output wire [31:0]  iw_d_data,
    input  wire         iw_d_ready,
    input  wire         iw_busy,
    // READ / FILL / COPY engine (img_xfer)
    output reg          xf_cmd_valid,
    input  wire         xf_cmd_ready,
    output reg  [1:0]   xf_op, xf_s_fmt, xf_d_fmt,
    output reg  [29:0]  xf_s_row0, xf_d_row0,
    output reg  [15:0]  xf_s_pitch, xf_d_pitch,
    output reg  [12:0]  xf_s_x, xf_s_y, xf_s_w, xf_s_h, xf_d_x, xf_d_y, xf_d_w, xf_d_h,
    output reg  [12:0]  xf_hq, xf_hu0, xf_vq, xf_vu0,
    output reg  [13:0]  xf_hr2, xf_ha0, xf_vr2, xf_va0,
    output reg          xf_mir_x, xf_mir_y, xf_bottom_up, xf_key, xf_rmw,
    output reg  [31:0]  xf_key_lo, xf_key_hi, xf_value,
    output reg  [3:0]   xf_mask,
    output reg  [7:0]   xf_tfmt,
    output reg  [12:0]  xf_j0, xf_i0,
    output reg  [23:0]  xf_npix,
    input  wire         xf_busy,
    input  wire         xo_valid,
    input  wire [31:0]  xo_data,
    output wire         xo_ready,
    // display
    output reg          set_mode,
    output reg  [2:0]   set_mode_id,
    input  wire         mode_busy, mode_done, mode_ok,
    input  wire         video_on,
    input  wire [31:0]  frames,
    output reg          pres_valid,
    output reg  [29:0]  pres_base,
    output reg  [15:0]  pres_pitch,
    output reg  [12:0]  pres_w, pres_h,
    output reg  [1:0]   pres_fmt,
    output reg          pres_origin, pres_now,
    output reg  [31:0]  pres_tag,
    input  wire         pres_done,
    input  wire [31:0]  cur_tag,
    // palette (disp_ctl_b)
    output reg          pm_we,
    output reg  [7:0]   pm_idx,
    output reg  [23:0]  pm_data,
    output reg          pal_pub,
    input  wire         pal_busy,
    // cursor (disp_ctl_b)
    output reg          cur_cmd,
    output reg  [2:0]   cur_op,
    output reg  [1:0]   cur_mode,
    output reg          cur_take, cur_setpos,
    output reg  [5:0]   cur_hx, cur_hy,
    output reg  [15:0]  cur_x, cur_y,
    output wire         cs_wr,
    output wire [11:0]  cs_idx,
    output wire [31:0]  cs_data,
    input  wire         cs_ready,
    // 3D: one frame on b_render (FLUSH)
    output reg          rd_go,
    output reg          rd_discard,              // with rd_go: the FLUSH was rejected — consume its frame, render nothing
    output reg  [28:0]  rd_rt_base,
    output reg  [10:0]  rd_rt_bpl, rd_rt_width,
    output reg  [11:0]  rd_rt_height,               // the FLUSH image's height (b_render: the band preload's limit)
    output reg  [51:0]  rd_clear_c16,
    output reg          rd_clear_dither, rd_clear_en, rd_clear_sc,
    output reg  [3:0]   rd_clear_cmask,
    output reg  [12:0]  rd_clear_x0, rd_clear_y0, rd_clear_x1, rd_clear_y1,
    output reg  [23:0]  rd_clear_depth,
    output reg  [7:0]   rd_clear_stencil,
    output reg  [1:0]   rd_rt_fmt,                  // 0 RGBA8, 1 RGBA4444, 2 RGBA5551, 3 RGB565 (tmu24 codes)
    output reg          rd_depth16,                 // DEPTH_COMPONENT16 attachment
    output reg          rd_ds_en,                   // a D24S8 depth-stencil image is attached
    output reg  [28:0]  rd_ds_base,
    output reg  [10:0]  rd_ds_bpl,
    output reg          rd_clear_depth_en, rd_clear_stencil_en,
    output reg  [7:0]   rd_clear_smask,
    input  wire         rd_done,
    input  wire         rd_frame_err,               // with rd_done: a fragment-program stack violation killed quads of the frame
    // statistics
    output reg  [15:0]  c_cmds, c_errs,
    output reg  [31:0]  c_images,                   // CREATE_IMAGE accepted
    output reg  [31:0]  c_draws,                    // DRAW_STATE stored
    output wire         busy
);
    import fb_pkg::*;
    localparam [1:0] OP_WRITE = 2'd0, OP_READ = 2'd1, OP_MEMTEST = 2'd2;
    localparam [1:0] XF_COPY = 2'd0, XF_FILL = 2'd1, XF_READ = 2'd2;
    localparam [7:0] TF_RAW = 8'h00, TF_RGBA = 8'h01, TF_BGRA = 8'h02, TF_BGRX = 8'h03, TF_565 = 8'h04,
                     TF_RGB = 8'h0A, TF_BGR = 8'h0B;

    typedef enum logic [6:0] {
        S_W0, S_W1, S_F, S_EXEC0, S_EXEC, S_DRAIN, S_EV,
        S_MODE, S_MODEW, S_PRES1, S_PRES2, S_PRES3, S_HOLD,
        S_CI0, S_CI0A, S_CI0B, S_CI1, S_CI2,
        S_X, S_MWD, S_XWAIT, S_RCH, S_RW, S_RWAIT, S_MTW,
        S_DS1, S_DS2, S_DS3,                                       // descriptor check + level geometry (subroutine)
        S_WI3, S_WI4, S_WI5, S_WI6, S_WI7, S_WI8, S_WID, S_WIW,
        S_RI3, S_RI4, S_RI5,
        S_FI3, S_FI4,
        S_CP2, S_CP3, S_CP4, S_CP5, S_CP6, S_CP7, S_CP8, S_CP9,
        S_XF, S_XFW,
        S_PALD, S_PALP, S_PALW,
        S_CSD,
        S_BUF1, S_BUF2, S_BUF3,                                    // B buffer lookup (WRITE / READ_BUFFER)
        S_DW0, S_DWH11, S_DWH12, S_DWL1, S_DWL2, S_DWL3, S_DWP, S_DWH13, S_DWH14, S_DWN, S_DWZ,   // DRAW_STATE bucket
        S_DWS0, S_DWS1, S_DWS2, S_DWS3, S_DWU, S_DWE,
        S_FL1, S_FL2, S_FL3, S_FL4, S_FLW                          // FLUSH
    } st_t;
    st_t        st, ret_st;
    reg  [7:0]  mtype;
    reg  [15:0] blen;
    reg  [31:0] mseq;
    reg  [31:0] f [0:8];
    reg  [3:0]  fi, nf;
    reg         got_last;                           // the command's last word has been taken
    reg         it_w;                               // the image-table read's second wait cycle (img_table port 2 has the block
                                                    // RAM's output register: ridx, two cycles, the descriptor)
    reg  [31:0] hold_t;
    assign busy = (st != S_W0);

    function automatic [3:0] nfields(input [7:0] t);
        case (t)
            T_SET_MODE, T_FENCE, T_DESTROY, T_SET_PALETTE, T_CURSOR, T_CURSOR_POS, T_DRAW_STATE: nfields = 4'd1;
            T_PRESENT, T_MEM_WRITE, T_WRITE_BUFFER: nfields = 4'd2;
            T_CREATE_BUFFER:                        nfields = 4'd3;
            T_MEM_READ, T_READ_BUFFER:              nfields = 4'd4;
            T_MEMTEST, T_CREATE_IMAGE, T_FILL_IMAGE: nfields = 4'd5;
            T_WRITE_IMAGE:                          nfields = 4'd6;
            T_READ_IMAGE, T_FLUSH:                  nfields = 4'd7;
            T_COPY_IMAGE:                           nfields = 4'd9;
            default:                                nfields = 4'd0;
        endcase
    endfunction
    // CURSOR: the field count depends on the op in its first field
    function automatic [3:0] cur_nf(input [7:0] op);
        case (op)
            CUR_SHAPE, CUR_MOVE: cur_nf = 4'd2;
            CUR_SHOW:            cur_nf = 4'd3;
            default:             cur_nf = 4'd1;
        endcase
    endfunction
    wire [3:0]  nf_now = (mtype == T_CURSOR && fi == 4'd0) ? cur_nf(c_data[7:0]) : nf;

    // event to send after the command (one at a time; the next command waits for it)
    reg         ev_pend;
    reg  [7:0]  ev_code; reg [31:0] ev_a0, ev_a1, ev_a2, ev_seq;
    // the command's sub-message index (CMD header byte 3: its place in a host BATCH, 0 otherwise), echoed in the
    // EVENT's code word bits [15:8] so the host can tell the events of one batch apart (fb_pkg T_BATCH)
    reg  [7:0]  msub, ev_sub;
    // outstanding present (waiting to become current)
    reg         op_v; reg [31:0] op_seq, op_fid; reg op_hold; reg [7:0] op_sub;
    // a present-done event waiting for the event channel
    reg         pd_v; reg [31:0] pd_seq, pd_fid, pd_frames, pd_dropped; reg [7:0] pd_sub;

    // ---------------- payload words of MEM_WRITE / WRITE_IMAGE / SET_PALETTE / CURSOR SHAPE ----------------
    reg  [15:0] dleft;                              // payload words still to deliver
    reg  [15:0] dk;                                 // index of the next payload word (palette / cursor)
    reg         dpad;                               // the command ended early: deliver zero words, then ERR_LEN
    wire        d_src = c_valid && !got_last;
    // DRAW_STATE: words passed through to the bucket (dw_pass), computed bucket words (dw_emit), command words held in
    // registers (dw_hold)
    wire        dw_pass = (st == S_DW0) || (st == S_DWU);
    wire        dw_emit = (st == S_DWP) || (st == S_DWN) || (st == S_DWZ) || (st == S_DWS3);
    wire        dw_hold = (st == S_DWH11) || (st == S_DWH12) || (st == S_DWH13) || (st == S_DWH14) || (st == S_DWS0);
    wire [31:0] dw_data;                                    // the computed word of the emitting state (below)
    wire        h_take  = dw_hold && d_src;                 // a held word arrives (got_last: it reads as 0, ERR_LEN)
    wire        h_word  = dw_hold && (d_src || got_last);
    wire        d_sink_rdy = ((st == S_MWD || dw_pass) && x_wd_ready) || (st == S_WID && iw_d_ready) || (st == S_PALD) ||
                             (st == S_CSD && cs_ready);
    assign x_wd_valid = dw_emit || (((st == S_MWD) || dw_pass) && dleft != 0 && (d_src || got_last));
    assign x_wd_data  = dw_emit ? dw_data : got_last ? 32'h0 : c_data;
    assign iw_d_valid = (st == S_WID) && dleft != 0 && (d_src || got_last);
    assign iw_d_data  = got_last ? 32'h0 : c_data;
    assign cs_wr      = (st == S_CSD) && dleft != 0 && d_src;
    assign cs_data    = c_data;
    wire        d_take = d_sink_rdy && dleft != 0 && d_src;
    wire        d_pad  = (((st == S_MWD || dw_pass) && x_wd_ready) || (st == S_WID && iw_d_ready)) && dleft != 0 && got_last;
    // B buffers (fragment programs)
    reg         bt_we, bt_wv; reg [9:0] bt_widx, bt_ridx; reg [3:0] bt_wu; reg [29:0] bt_wb, bt_ws;
    wire        bt_rv; wire [3:0] bt_ru; wire [29:0] bt_rb, bt_rs;
    buf_table u_bt (.clk(clk), .rst(rst), .we(bt_we), .widx(bt_widx), .wvalid(bt_wv), .wusage(bt_wu),
        .wbase(bt_wb), .wsize(bt_ws), .ridx(bt_ridx), .rvalid(bt_rv), .rusage(bt_ru), .rbase(bt_rb), .rsize(bt_rs));
    // the table read registered, and blen - 8 ahead of the range check (timing: the table's block-RAM output ->
    // adder -> 33-bit compare -> error() does not fit one cycle)
    reg         bq_v; reg [29:0] bq_b, bq_s; reg [15:0] bl8;
    wire        h_buf = f[0][15] && !f[0][14] && f[0][13:10] == 4'h0;
    // DRAW_STATE state
    reg  [6:0]  dw_n; reg [31:0] dw_h11, dw_off, dw_sw, dw_h14; reg [28:0] dw_pw; reg [11:0] dw_plen; reg [2:0] dw_si;
    reg  [1:0]  dw_k; reg [31:0] dw_s [0:3]; reg dw_err; reg [7:0] dw_ecode; reg [31:0] dw_edet; reg dw_mode;
    assign dw_data = (st == S_DWP) ? {3'b000, dw_pw} : (st == S_DWN) ? {9'h0, dw_n, 4'h0, dw_plen} :
                     (st == S_DWZ && dw_k == 2'd0) ? dw_h14 :                 // bucket word 13: the constant alpha word
                     (st == S_DWS3) ? dw_s[dw_k] : 32'h0;
    // FLUSH: the colour image's size, for the depth-stencil image's checks
    reg  [12:0] fl_w, fl_h; reg fl_bpp16;

    // ---------------- READ_DATA chunks (MEM_READ from xfer_lin, READ_IMAGE from img_xfer) ----------------
    reg  [29:0] r_len, r_off;
    reg  [31:0] r_tag, r_base;
    reg  [15:0] r_n, r_oi;
    reg         r_src;                              // 1: img_xfer
    wire        rd_v = r_src ? xo_valid : x_rd_valid;
    wire [31:0] rd_d = r_src ? xo_data : x_rd_data;
    assign r_valid    = r_req && (r_oi < 16'd4 || rd_v);
    assign r_data     = (r_oi == 16'd0) ? mseq : (r_oi == 16'd1) ? r_tag : (r_oi == 16'd2) ? (r_base + {2'b00, r_off}) :
                        (r_oi == 16'd3) ? {16'h0, r_n} : rd_d;
    assign x_rd_ready = r_req && !r_src && r_oi >= 16'd4 && r_ready;
    assign xo_ready   = r_req && r_src && r_oi >= 16'd4 && r_ready;

    assign c_ready = (st == S_W0) || (st == S_W1 && !got_last) || (st == S_F && fi < nf_now && !got_last) ||
                     (st == S_DRAIN && !got_last) || d_take || h_take;

    // ---------------- arithmetic helpers ----------------
    reg         g_start; wire g_done; wire [12:0] g_wl, g_hl; wire [7:0] g_dl; wire [15:0] g_pitchl; wire [31:0] g_off; wire [39:0] g_total;
    reg  [12:0] g_w, g_h; reg [15:0] g_pitch; reg [3:0] g_levels, g_level; reg [7:0] g_layers, g_layer; reg g_vol;
    reg  [1:0]  g_slog, g_bl;
    img_geom u_geom (.clk(clk), .rst(rst), .start(g_start), .width(g_w), .height(g_h), .pitch(g_pitch),
        .levels(g_levels), .layers(g_layers), .vol(g_vol), .samples_log2(g_slog), .bpp_log2(g_bl), .level(g_level),
        .layer(g_layer), .done(g_done), .wl(g_wl), .hl(g_hl), .dl(g_dl), .pitchl(g_pitchl), .off(g_off), .total(g_total));
    reg         dv_start; reg [31:0] dv_a; reg [15:0] dv_b; wire dv_done; wire [31:0] dv_q; wire [15:0] dv_r;
    udiv #(.N(32), .D(16)) u_div (.clk(clk), .rst(rst), .start(dv_start), .a(dv_a), .b(dv_b), .done(dv_done),
        .q(dv_q), .r(dv_r));

    // floor(log2(m)) for m = 1 .. 8191
    function automatic [3:0] flog2(input [12:0] m);
        flog2 = 0;
        for (int i = 0; i < 13; i++) if (m[i]) flog2 = 4'(i);
    endfunction
    // descriptor storage format -> engine code (0 RGBA8, 1 RGB565, 2 P8) and log2 bytes per pixel
    // (RGBA4444 / RGBA5551: 2 bytes per pixel, moved as RGB565 by the engines — RAW transfers only)
    function automatic bit raw_only(input [5:0] fmt);
        raw_only = (fmt == FMT_P8) || (fmt == FMT_RGBA4444) || (fmt == FMT_RGBA5551) || (fmt == FMT_D24S8);
    endfunction
    function automatic bit rt_fmt_ok(input [5:0] fmt);           // colour-renderable (FLUSH targets)
        rt_fmt_ok = (fmt == FMT_RGBA8) || (fmt == FMT_RGB565) || (fmt == FMT_RGBA4444) || (fmt == FMT_RGBA5551);
    endfunction
    function automatic [1:0] sf_of(input [5:0] fmt);
        sf_of = (fmt == FMT_RGB565 || fmt == FMT_RGBA4444 || fmt == FMT_RGBA5551) ? 2'd1 : (fmt == FMT_P8) ? 2'd2 : 2'd0;
    endfunction
    function automatic [1:0] bl_of(input [5:0] fmt);
        bl_of = (fmt == FMT_RGB565 || fmt == FMT_RGBA4444 || fmt == FMT_RGBA5551) ? 2'd1 : (fmt == FMT_P8) ? 2'd0 : 2'd2;
    endfunction
    // TMU format code (tmu24): 0 RGBA8, 1 RGBA4444, 2 RGBA5551, 3 RGB565
    function automatic [1:0] tmu_fmt(input [5:0] fmt);
        tmu_fmt = (fmt == FMT_RGBA4444) ? 2'd1 : (fmt == FMT_RGBA5551) ? 2'd2 : (fmt == FMT_RGB565) ? 2'd3 : 2'd0;
    endfunction
    function automatic bit tf_known(input [7:0] t);
        tf_known = (t == TF_RAW) || (t == TF_RGBA) || (t == TF_BGRA) || (t == TF_BGRX) || (t == TF_565) ||
                   (t == TF_RGB) || (t == TF_BGR);
    endfunction
    // v * bytes per pixel (1..4) by shifts and one add (no wide multiplier in one cycle)
    function automatic [31:0] mul_bpp(input [31:0] v, input [2:0] n);
        case (n)
            3'd1:    mul_bpp = v;
            3'd2:    mul_bpp = v << 1;
            3'd3:    mul_bpp = (v << 1) + v;
            default: mul_bpp = v << 2;
        endcase
    endfunction
    function automatic [2:0] tbpp_of(input [7:0] t, input [5:0] fmt);
        case (t)
            TF_RAW:         tbpp_of = 3'd1 << bl_of(fmt);
            TF_565:         tbpp_of = 3'd2;
            TF_RGB, TF_BGR: tbpp_of = 3'd3;
            default:        tbpp_of = 3'd4;
        endcase
    endfunction

    // ---------------- field views ----------------
    wire [15:0] hnd     = f[0][15:0];
    wire        h_img   = hnd[15] && hnd[14] && hnd[13:10] == 4'h0;
    wire [9:0]  h_idx   = hnd[9:0];
    // CREATE_IMAGE
    wire [3:0]  ci_use  = f[0][19:16];
    wire [31:0] ci_base = f[1];
    wire [15:0] ci_pitch = f[2][15:0];
    wire [7:0]  ci_fmt  = f[2][23:16];
    wire        ci_org  = f[2][24];
    wire [1:0]  ci_lay  = f[2][26:25];
    wire [1:0]  ci_slog = f[2][28:27];
    wire [15:0] ci_w    = f[3][15:0], ci_h = f[3][31:16];
    wire [7:0]  ci_lev  = f[4][7:0], ci_nl = f[4][15:8];
    wire        ci_vol  = f[4][16];
    wire        ci_fok  = (ci_fmt == {2'b00, FMT_RGBA8}) || (ci_fmt == {2'b00, FMT_RGB565}) || (ci_fmt == {2'b00, FMT_P8}) ||
                          (ci_fmt == {2'b00, FMT_RGBA4444}) || (ci_fmt == {2'b00, FMT_RGBA5551}) || (ci_fmt == {2'b00, FMT_D24S8});
    wire [1:0]  ci_bl   = bl_of(ci_fmt[5:0]);
    wire [12:0] ci_wh   = (ci_w[12:0] > ci_h[12:0]) ? ci_w[12:0] : ci_h[12:0];
    wire [12:0] ci_wm   = (ci_vol && {5'h0, ci_nl} > ci_wh) ? {5'h0, ci_nl} : ci_wh;
    wire [16:0] ci_rowb = {4'h0, ci_w[12:0]} << ci_bl;
    wire        ci_pmis = (ci_bl == 2'd2) ? |ci_pitch[1:0] : (ci_bl == 2'd1) ? ci_pitch[0] : 1'b0;
    reg         ci_bad_rng, ci_bad_heap;
    reg  [12:0] ci_wm_r; reg [16:0] ci_rowb_r; reg ci_pmis_r, ci_fok_r;   // S_CI0's registered terms (the check in two steps, for timing)
    // WRITE / READ / FILL_IMAGE (same layout of words 0..3)
    wire [7:0]  t_lev   = f[0][23:16], t_lay = f[0][31:24];
    wire [7:0]  t_tf    = f[1][7:0];
    wire [15:0] t_x     = f[2][15:0], t_y = f[2][31:16];
    wire [15:0] t_w     = f[3][15:0], t_h = f[3][31:16];
    // the rectangle check REGISTERED (timing at 120 MHz: the fields -> two 17-bit adds and compares -> the error event's
    // registers is 12 levels in one cycle); valid in S_WI3 / S_RI3 / S_FI3: the fields stand since
    // S_F, the level's size since the geometry's G_MUL, two cycles at least before g_done
    reg         t_bad;
    always @(posedge clk)
        t_bad <= (t_w == 0) || (t_h == 0) || ({1'b0, t_x} + {1'b0, t_w} > {4'h0, g_wl}) || ({1'b0, t_y} + {1'b0, t_h} > {4'h0, g_hl});
    // S_EXEC's field checks REGISTERED (timing at 120 MHz: the fields -> the 32 / 33-bit compares and sums -> the type case
    // -> the event / request registers is 15 levels in one cycle). S_EXEC0, one cycle between
    // the last field's write and S_EXEC, lets them settle: the fields, blen, fi and nf stand from the S_F -> S_EXEC0
    // transition on, so every value below is that of the command in hand throughout S_EXEC (also while it waits on
    // h_active). One cycle more per command; nothing per pixel.
    reg         xc_len_bad;                         // fewer fields than the type needs / a short blen
    reg         xc_sm_bad, xc_hp_bad, xc_dsl_bad, xc_dsr_bad, xc_brd_bad, xc_mt_bad, xc_rd0;
    reg         xc_pal_rng, xc_pal_len, xc_cs_len, xc_cs_rng, xc_cur_bad;
    always @(posedge clk) begin
        xc_len_bad <= (fi < nf) || (blen < {10'h0, nf, 2'b00});
        xc_sm_bad  <= f[0] > (SIM_MODE4 ? 32'd6 : 32'd4);
        xc_hp_bad  <= f[1][4:0] != 0 || f[1] < HEAP_LO || f[2] == 0 || {1'b0, f[1]} + {1'b0, f[2]} > {1'b0, HEAP_HI};
        xc_dsl_bad <= blen < 16'd96 || blen[3:0] != 4'd0 || blen > 16'd96 + 16'd1024;
        xc_dsr_bad <= f[0][31:13] != 19'h0;
        xc_brd_bad <= f[0][7:0] != 8'd1;
        xc_mt_bad  <= f[1][4:0] != 0 || f[2][4:0] != 0;
        xc_rd0     <= f[2] == 0;
        xc_pal_rng <= f[0][31:16] == 16'd0 || {16'h0, f[0][15:0]} + {16'h0, f[0][31:16]} > 32'd256;
        xc_pal_len <= blen != 16'd4 + {f[0][29:16], 2'b00};
        xc_cs_len  <= blen < 16'd8 || blen[1:0] != 2'b00;
        xc_cs_rng  <= {16'h0, f[1][15:0]} + {18'h0, (blen - 16'd8) >> 2} > 32'd4096;
        xc_cur_bad <= f[0][15:8] > 8'd2 || f[1][7:0] > 8'd63 || f[1][15:8] > 8'd63;
    end
    // COPY_IMAGE
    wire [15:0] cp_sx = f[2][15:0], cp_sy = f[2][31:16], cp_sw = f[3][15:0], cp_sh = f[3][31:16];
    wire [15:0] cp_dx = f[4][15:0], cp_dy = f[4][31:16], cp_dw = f[5][15:0], cp_dh = f[5][31:16];
    // descriptor subroutine: handle index / level / layer of the image it checks
    reg  [9:0]  ds_idx; reg [7:0] ds_lev, ds_lay; reg [31:0] ds_det;     // ds_det: error detail (the handle's word)
    img_desc_t  d;                                  // descriptor of the (last checked) target
    img_desc_t  sd;                                 // COPY: the source's descriptor
    reg  [29:0] s_row0; reg [15:0] s_pitchl; reg [12:0] s_wl, s_hl;   // COPY: the source level
    // COPY_IMAGE's source / destination rectangle checks REGISTERED (timing: the fields -> four 17-bit adds and compares
    // against the levels' sizes -> error()'s event registers does not fit one cycle); valid in S_CP3: the fields stand
    // since S_F, s_wl / s_hl since S_CP2 (the descriptor subroutine takes at least three cycles after it), the
    // destination's g_wl / g_hl since its geometry's G_MUL, two cycles before g_done
    reg         cp_bad_s, cp_bad_d;
    always @(posedge clk) begin
        cp_bad_s <= cp_sw == 0 || cp_sh == 0 || cp_dw == 0 || cp_dh == 0 ||
                    {1'b0, cp_sx} + {1'b0, cp_sw} > {4'h0, s_wl} || {1'b0, cp_sy} + {1'b0, cp_sh} > {4'h0, s_hl};
        cp_bad_d <= {1'b0, cp_dx} + {1'b0, cp_dw} > {4'h0, g_wl} || {1'b0, cp_dy} + {1'b0, cp_dh} > {4'h0, g_hl};
    end
    wire [5:0]  dfmt = d.format;
    reg  [2:0]  tbpp;
    reg  [15:0] pbytes;                             // payload bytes
    reg  [23:0] npix;
    reg  [25:0] wh;
    reg  [12:0] j0, i0;
    reg  [3:0]  dvi;                                // COPY: which DDA division

    task automatic error(input [7:0] code, input [31:0] detail);
        ev_pend <= 1'b1; ev_code <= EV_ERROR; ev_a0 <= {24'h0, code}; ev_a1 <= {24'h0, mtype}; ev_a2 <= detail;
        ev_seq <= mseq; ev_sub <= msub; c_errs <= c_errs + 16'd1; x_own <= 1'b0; st <= S_DRAIN;
    endtask
    task automatic finish();
        x_own <= 1'b0; st <= S_DRAIN;
    endtask
    // a FLUSH that fails its checks still owns one frame of the render stream (board A streams it regardless): b_render
    // consumes that frame and drops it, then the error goes out (no stale beats in front of the next frame)
    reg        fl_err; reg [7:0] fl_ecode; reg [31:0] fl_edet;
    // A FLUSH older than a wiping SYNC (its first word was in the command buffer then, or the command was in progress)
    // owns NO frame any more: the SYNC emptied the render buffer, and board A drops every frame whose FLUSH it forwarded
    // before the SYNC. It must not start b_render, which would take the next frame's stream (one frame behind) or wait
    // for ever (a deadlock). Board A reports ERR_RENDER for it; nothing here.
    reg        cm_old;
    wire       fl_orphan = cm_old || sync_wipe;
    task automatic flush_go();
        if (fl_orphan) finish(); else begin rd_go <= 1'b1; st <= S_FLW; end
    endtask
    task automatic flush_fail(input [7:0] code, input [31:0] detail);
        fl_err <= 1'b1; fl_ecode <= code; fl_edet <= detail;
        if (fl_orphan) error(code, detail); else begin rd_discard <= 1'b1; rd_go <= 1'b1; st <= S_FLW; end
    endtask
    task automatic event4(input [7:0] code, input [31:0] a0, input [31:0] a1, input [31:0] a2);
        ev_pend <= 1'b1; ev_code <= code; ev_a0 <= a0; ev_a1 <= a1; ev_a2 <= a2; ev_seq <= mseq; ev_sub <= msub;
        x_own <= 1'b0; st <= S_DRAIN;
    endtask
    task automatic desc_check(input [31:0] w, input st_t ret);      // w = {layer, level, handle}
        if (!(w[15] && w[14] && w[13:10] == 4'h0)) error(ERR_HANDLE, w);
        else begin
            it_ridx <= w[9:0]; ds_idx <= w[9:0]; ds_lev <= w[23:16]; ds_lay <= w[31:24]; ds_det <= w;
            ret_st <= ret; st <= S_DS1;
        end
    endtask

    always @(posedge clk) begin
        set_mode <= 1'b0; pres_valid <= 1'b0; it_we <= 1'b0; x_cmd_valid <= 1'b0; iw_cmd_valid <= 1'b0;
        xf_cmd_valid <= 1'b0; g_start <= 1'b0; dv_start <= 1'b0; pm_we <= 1'b0; pal_pub <= 1'b0; cur_cmd <= 1'b0;
        bt_we <= 1'b0; rd_go <= 1'b0;
        if (e_valid && e_ready) e_valid <= 1'b0;
        if (rst) begin
            st <= S_W0; got_last <= 1'b0; e_valid <= 1'b0; op_v <= 1'b0; pd_v <= 1'b0; ev_pend <= 1'b0; it_w <= 1'b0;
            for (int i = 0; i < 9; i++) f[i] <= 0;
            x_own <= 1'b0; r_req <= 1'b0; r_src <= 1'b0; c_cmds <= 0; c_errs <= 0; c_images <= 0; c_draws <= 0;
            dw_mode <= 1'b0; cm_old <= 1'b0;
        end else begin
            if (sync_wipe && st != S_W0) cm_old <= 1'b1;        // a wipe in the middle of a command: it is older (S_W0 sets it below)
            // present completion (the image became current) or replacement
            if (pres_done && cur_tag[31] && op_v) begin
                pd_v <= 1'b1; pd_seq <= op_seq; pd_sub <= op_sub; pd_fid <= op_fid; pd_frames <= frames; pd_dropped <= 0; op_v <= 1'b0;
            end
            // payload words
            if (d_take || d_pad) begin dleft <= dleft - 16'd1; dk <= dk + 16'd1; end
            if (d_take && c_last) got_last <= 1'b1;
            if (d_pad) dpad <= 1'b1;
            if (d_take && st == S_PALD) begin pm_we <= 1'b1; pm_idx <= 8'(f[0][7:0] + dk[7:0]); pm_data <= c_data[23:0]; end
            // READ_DATA words
            if (r_valid && r_ready) begin
                r_oi <= r_oi + 16'd1;
                if (r_oi + 16'd1 == r_words) r_req <= 1'b0;
            end
            case (st)
                // ---------------- intake ----------------
                S_W0: if (c_valid && c_first) begin
                    cm_old <= c_old || sync_wipe; it_w <= 1'b0;
                    mtype <= c_data[23:16]; blen <= c_data[15:0]; msub <= c_data[31:24]; got_last <= c_last; mseq <= 0;
                    fi <= 0; ev_pend <= 1'b0; dw_mode <= 1'b0;          // (the fields were cleared when the last command ended)
                    nf <= nfields(c_data[23:16]); dpad <= 1'b0;
                    st <= S_W1;
                end
                S_W1: begin
                    if (got_last) st <= S_EXEC0;
                    else if (c_valid) begin
                        mseq <= c_data; got_last <= c_last;
                        st <= (nf == 0 || c_last) ? S_EXEC0 : S_F;
                    end
                end
                S_F: begin
                    if (c_valid && fi < nf_now && !got_last) begin
                        f[fi] <= c_data; fi <= fi + 4'd1; got_last <= c_last; nf <= nf_now;
                        if (fi + 4'd1 == nf_now || c_last) st <= S_EXEC0;
                    end
                end
                S_EXEC0: st <= S_EXEC;                                 // the xc_* checks settle
                S_EXEC: if (!h_active) begin
                    c_cmds <= c_cmds + 16'd1;
                    if (xc_len_bad) error(ERR_LEN, {16'h0, blen});
                    else case (mtype)
                        T_SET_MODE: begin
                            if (xc_sm_bad) error(ERR_RANGE, f[0]);
                            else begin set_mode_id <= f[0][2:0]; st <= S_MODE; end
                        end
                        T_PRESENT: begin
                            if (!h_img) error(ERR_HANDLE, f[0]);
                            else begin it_ridx <= h_idx; st <= S_PRES1; end
                        end
                        T_FENCE: event4(EV_FENCE, f[0], 32'h0, 32'h0);
                        T_DESTROY: begin
                            if (h_img) begin it_we <= 1'b1; it_widx <= h_idx; it_wvalid <= 1'b0; it_wdesc <= '0; finish(); end
                            else if (h_buf) begin bt_we <= 1'b1; bt_widx <= f[0][9:0]; bt_wv <= 1'b0; finish(); end
                            else error(ERR_HANDLE, f[0]);
                        end
                        T_CREATE_BUFFER: begin
                            if (!h_buf) error(ERR_HANDLE, f[0]);
                            else if (xc_hp_bad) error(ERR_HEAP, f[1]);
                            else begin
                                bt_we <= 1'b1; bt_widx <= f[0][9:0]; bt_wv <= 1'b1; bt_wu <= f[0][19:16];
                                bt_wb <= f[1][29:0]; bt_ws <= f[2][29:0]; finish();
                            end
                        end
                        T_WRITE_BUFFER, T_READ_BUFFER: begin
                            if (!h_buf) error(ERR_HANDLE, f[0]);
                            else begin bt_ridx <= f[0][9:0]; st <= S_BUF1; end
                        end
                        T_DRAW_STATE: begin
                            // 4 + 23 * 4 + n * 16 bytes, n <= 64 uniforms
                            if (xc_dsl_bad) error(ERR_LEN, {16'h0, blen});
                            else if (xc_dsr_bad) error(ERR_RANGE, f[0]);
                            else begin
                                dw_n <= 7'((blen - 16'd96) >> 4); dw_err <= 1'b0; dw_mode <= 1'b1;
                                x_own <= 1'b1; x_op <= OP_WRITE; x_addr <= DS_BYTE + {f[0][12:0], 11'h0};
                                x_len <= 30'((16'd48 + ((blen - 16'd96) >> 2)) << 2);
                                dleft <= 16'd11; st <= S_X;
                            end
                        end
                        T_FLUSH: begin
                            fl_err <= 1'b0; rd_discard <= 1'b0;
                            if (!h_img) flush_fail(ERR_HANDLE, f[0]);
                            else begin it_ridx <= h_idx; st <= S_FL1; end
                        end
                        T_CREATE_IMAGE: begin
                            if (!h_img) error(ERR_HANDLE, f[0]);
                            else st <= S_CI0;
                        end
                        T_MEM_WRITE, T_MEM_READ, T_MEMTEST: begin
                            if (xc_brd_bad) error(ERR_BOARD, f[0]);
                            else if (mtype == T_MEMTEST && xc_mt_bad) error(ERR_LEN, f[2]);
                            else if (mtype == T_MEM_READ && xc_rd0) finish();
                            else begin
                                x_own <= 1'b1; x_addr <= f[1][29:0]; x_seed <= f[3]; x_flags <= f[4][1:0];
                                if (mtype == T_MEM_WRITE) begin
                                    x_op <= OP_WRITE; x_len <= {14'h0, blen - 16'd8}; dleft <= (blen - 16'd8 + 16'd3) >> 2;
                                end else if (mtype == T_MEM_READ) begin
                                    x_op <= OP_READ; x_len <= f[2][29:0]; r_len <= f[2][29:0]; r_off <= 0; r_tag <= f[3];
                                    r_base <= 0; r_src <= 1'b0;
                                end else begin x_op <= OP_MEMTEST; x_len <= f[2][29:0]; end
                                st <= S_X;
                            end
                        end
                        T_WRITE_IMAGE, T_READ_IMAGE, T_FILL_IMAGE: desc_check(f[0],
                            (mtype == T_WRITE_IMAGE) ? S_WI3 : (mtype == T_READ_IMAGE) ? S_RI3 : S_FI3);
                        T_COPY_IMAGE: desc_check(f[0], S_CP2);
                        T_SET_PALETTE: begin
                            if (xc_pal_rng) error(ERR_RANGE, f[0]);
                            else if (xc_pal_len) error(ERR_LEN, {16'h0, blen});
                            else begin dleft <= f[0][31:16]; dk <= 0; st <= S_PALD; end
                        end
                        T_CURSOR: case (f[0][7:0])
                            CUR_SHAPE: begin
                                if (xc_cs_len) error(ERR_LEN, {16'h0, blen});
                                else if (xc_cs_rng) error(ERR_RANGE, f[1]);
                                else begin dleft <= (blen - 16'd8) >> 2; dk <= 0; st <= S_CSD; end
                            end
                            CUR_SHOW: begin
                                if (xc_cur_bad) error(ERR_RANGE, f[0]);
                                else begin
                                    cur_cmd <= 1'b1; cur_op <= 3'd2; cur_mode <= f[0][9:8]; cur_take <= f[0][16];
                                    cur_setpos <= f[0][17]; cur_hx <= f[1][5:0]; cur_hy <= f[1][13:8];
                                    cur_x <= f[2][15:0]; cur_y <= f[2][31:16];
                                    finish();
                                end
                            end
                            CUR_HIDE: begin cur_cmd <= 1'b1; cur_op <= 3'd3; finish(); end
                            CUR_MOVE: begin cur_cmd <= 1'b1; cur_op <= 3'd4; cur_x <= f[1][15:0]; cur_y <= f[1][31:16]; finish(); end
                            default: error(ERR_RANGE, f[0]);
                        endcase
                        T_CURSOR_POS: begin cur_cmd <= 1'b1; cur_op <= 3'd4; cur_x <= f[0][15:0]; cur_y <= f[0][31:16]; finish(); end
                        default: error(ERR_TYPE, 32'h0);
                    endcase
                end
                // ---------------- end of a command: drop unread words, send its event ----------------
                S_DRAIN: begin
                    if (c_valid && !got_last && c_last) got_last <= 1'b1;
                    // the fields cleared here, at the command's end, not at the next first word (timing at 120 MHz: keeps
                    // the command buffer's block RAM -> c_first off the clear of 288 field flops)
                    if (got_last) begin st <= ev_pend ? S_EV : S_W0; for (int i = 0; i < 9; i++) f[i] <= 0; end
                end
                S_EV: if (!e_valid && !pd_v) begin
                    e_valid <= 1'b1; e_data <= {ev_a2, ev_a1, ev_a0, {16'h0, ev_sub, ev_code}, ev_seq};
                    ev_pend <= 1'b0; st <= S_W0;
                end
                // ---------------- SET_MODE ----------------
                S_MODE: if (!mode_busy) begin set_mode <= 1'b1; st <= S_MODEW; end
                S_MODEW: if (mode_done) begin
                    if (mode_ok) event4(EV_MODE, {29'h0, set_mode_id}, 32'h0, 32'h0);
                    else error(ERR_MODE, {29'h0, set_mode_id});
                end
                // ---------------- PRESENT ----------------
                S_PRES1: begin it_w <= ~it_w; if (it_w) st <= S_PRES2; end   // table read latency (two cycles)
                S_PRES2: if (!pres_done && !pd_v) begin             // never coincide with a completion report
                    if (!it_rvalid || !it_rdesc.usage[USE_SCANOUT] || it_rdesc.layout != 2'd0 ||
                        !(it_rdesc.format == FMT_RGBA8 || it_rdesc.format == FMT_RGB565 || it_rdesc.format == FMT_P8))
                        error(ERR_HANDLE, f[0]);
                    else begin
                        pres_valid <= 1'b1; pres_base <= it_rdesc.base; pres_pitch <= it_rdesc.pitch;
                        pres_w <= it_rdesc.width; pres_h <= it_rdesc.height;
                        pres_fmt <= sf_of(it_rdesc.format); pres_origin <= it_rdesc.origin;
                        pres_now <= f[0][16]; pres_tag <= {1'b1, 15'h0, f[0][15:0]};
                        // a present still waiting is replaced: report it as dropped
                        if (op_v) begin pd_v <= 1'b1; pd_seq <= op_seq; pd_sub <= op_sub; pd_fid <= op_fid; pd_frames <= frames; pd_dropped <= 1; end
                        op_v <= 1'b1; op_seq <= mseq; op_sub <= msub; op_fid <= f[1]; op_hold <= f[0][17];
                        st <= S_PRES3;
                    end
                end
                S_PRES3: begin hold_t <= 0; if (f[0][17]) st <= S_HOLD; else finish(); end   // flags bit 1: hold
                S_HOLD: begin
                    hold_t <= hold_t + 1;
                    if (!op_v || !video_on || hold_t >= HOLD_TIMEOUT) finish();
                end
                // ---------------- CREATE_IMAGE ----------------
                S_CI0: begin                                 // descriptor checks, registered in two steps (timing; the
                    ci_wm_r <= ci_wm; ci_rowb_r <= ci_rowb;  // largest side -> its log2 -> the level compare is 15 levels)
                    ci_pmis_r <= ci_pmis; ci_fok_r <= ci_fok;
                    ci_bad_heap <= ci_base[4:0] != 5'd0 || ci_base < HEAP_LO || ci_base >= HEAP_HI;
                    st <= S_CI0A;
                end
                S_CI0A: begin
                    ci_bad_rng <= !ci_fok_r || ci_w == 0 || ci_w > 16'd2048 ||
                                  ci_h == 0 || ci_h > 16'd2048 || ci_lev == 0 || ci_lev > 8'd12 ||
                                  ci_lev > {4'h0, flog2(ci_wm_r)} + 8'd1 || ci_nl == 0 ||
                                  !(ci_slog == 2'd0 || (ci_slog == 2'd2 && ci_lev == 8'd1)) || ci_lay != 2'd0 ||
                                  {1'b0, ci_pitch} < ci_rowb_r || ci_pmis_r ||
                                  ((ci_lev > 8'd1 || ci_vol) && {1'b0, ci_pitch} != ci_rowb_r) || f[2][31:29] != 3'd0 ||
                                  (ci_vol && (ci_w > 16'd256 || ci_h > 16'd256 || ci_slog != 2'd0)) || (!VOL3D && ci_vol);
                    st <= S_CI0B;
                end
                S_CI0B: begin
                    if (ci_bad_rng) error(ERR_RANGE, f[2]);
                    else if (ci_bad_heap) error(ERR_HEAP, ci_base);
                    else begin
                        g_w <= ci_w[12:0]; g_h <= ci_h[12:0]; g_pitch <= ci_pitch; g_levels <= ci_lev[3:0];
                        g_layers <= ci_nl; g_vol <= ci_vol; g_slog <= ci_slog; g_bl <= ci_bl; g_level <= 0; g_layer <= 0;
                        g_start <= 1'b1; st <= S_CI1;
                    end
                end
                S_CI1: if (g_done) begin
                    if ({8'h0, ci_base} + g_total > {8'h0, HEAP_HI}) error(ERR_HEAP, ci_base);
                    else st <= S_CI2;
                end
                S_CI2: begin
                    it_we <= 1'b1; it_widx <= h_idx; it_wvalid <= 1'b1;
                    it_wdesc <= '0;
                    it_wdesc.base <= ci_base[29:0]; it_wdesc.pitch <= ci_pitch; it_wdesc.width <= ci_w[12:0];
                    it_wdesc.height <= ci_h[12:0]; it_wdesc.format <= ci_fmt[5:0]; it_wdesc.levels <= ci_lev[3:0];
                    it_wdesc.layers <= ci_nl; it_wdesc.samples <= ci_slog; it_wdesc.layout <= ci_lay;
                    it_wdesc.origin <= ci_org; it_wdesc.usage <= ci_use; it_wdesc.vol <= ci_vol;
                    c_images <= c_images + 1;
                    finish();
                end
                // ---------------- MEM_WRITE / MEM_READ / MEMTEST on xfer_lin ----------------
                S_X: if (x_cmd_ready && !x_cmd_valid) begin
                    x_cmd_valid <= 1'b1;
                    case (x_op)
                        OP_WRITE: st <= dw_mode ? S_DW0 : S_MWD;
                        OP_READ:  st <= S_RCH;
                        default:  st <= S_MTW;
                    endcase
                end
                S_MWD: if (dleft == 0 && !d_take && !d_pad) st <= S_XWAIT;
                S_XWAIT: if (!x_busy && !x_cmd_valid && !xf_busy && !xf_cmd_valid) begin
                    if (dpad) error(ERR_LEN, {16'h0, blen}); else finish();
                end
                S_RCH: if (!r_req) begin
                    // next READ_DATA chunk, or the end
                    if (r_off >= r_len) st <= S_XWAIT;
                    else begin
                        r_n <= (r_len - r_off > 30'(RD_CHUNK)) ? 16'(RD_CHUNK) : 16'(r_len - r_off);
                        st <= S_RW;
                    end
                end
                S_RW: begin                                  // r_off stays until the chunk is out (sent as its offset)
                    r_req <= 1'b1; r_words <= 16'd4 + ((r_n + 16'd3) >> 2); r_oi <= 0;
                    st <= S_RWAIT;
                end
                S_RWAIT: if (!r_req) begin r_off <= r_off + {14'h0, r_n}; st <= S_RCH; end
                S_MTW: if (x_mt_done) event4(EV_MEMTEST, x_mt_errors, {2'b00, x_mt_first_bad}, {2'b00, x_len});
                // ---------------- descriptor subroutine: valid single-sample image, level / layer, geometry --------
                S_DS1: begin it_w <= ~it_w; if (it_w) st <= S_DS2; end       // table read latency (two cycles)
                S_DS2: begin
                    d <= it_rdesc;
                    if (!it_rvalid || it_rdesc.samples != 2'd0) error(ERR_HANDLE, ds_det);
                    else if (ds_lev >= {4'h0, it_rdesc.levels} || ds_lay >= it_rdesc.layers) error(ERR_RANGE, ds_det);
                    else begin
                        g_w <= it_rdesc.width; g_h <= it_rdesc.height; g_pitch <= it_rdesc.pitch;
                        g_levels <= it_rdesc.levels; g_layers <= it_rdesc.layers; g_vol <= it_rdesc.vol; g_slog <= it_rdesc.samples;
                        g_bl <= bl_of(it_rdesc.format); g_level <= ds_lev[3:0]; g_layer <= ds_lay;
                        g_start <= 1'b1; st <= S_DS3;
                    end
                end
                S_DS3: if (g_done) begin
                    if (d.vol && ds_lay >= g_dl) error(ERR_RANGE, ds_det);     // the depth halves with the level
                    else st <= ret_st;
                end
                // ---------------- WRITE_IMAGE ----------------
                S_WI3: begin
                    if (!tf_known(t_tf) || f[1][31:8] != 24'h0 || (raw_only(dfmt) && t_tf != TF_RAW)) error(ERR_RANGE, f[0]);
                    else if (t_bad)
                        error(ERR_RANGE, f[2]);
                    else begin
                        tbpp <= tbpp_of(t_tf, dfmt); pbytes <= blen - 16'd24;
                        wh <= t_w[12:0] * t_h[12:0];
                        dv_a <= {16'h0, blen - 16'd24}; dv_b <= {13'h0, tbpp_of(t_tf, dfmt)}; dv_start <= 1'b1;
                        st <= S_WI4;
                    end
                end
                S_WI4: if (dv_done) begin
                    if (dv_r != 0) error(ERR_LEN, {16'h0, blen});
                    else if (dv_q == 0) finish();                                 // no pixels: nothing to do
                    else if ({1'b0, f[4]} + {1'b0, dv_q} > {7'h0, wh}) error(ERR_RANGE, f[4]);
                    else begin
                        npix <= dv_q[23:0];
                        dv_a <= f[4]; dv_b <= {3'h0, t_w[12:0]}; dv_start <= 1'b1;
                        st <= S_WI5;
                    end
                end
                S_WI5: if (dv_done) begin j0 <= dv_q[12:0]; i0 <= dv_r[12:0]; st <= S_WI6; end
                S_WI6: begin
                    // byte address of the start of the first pixel's row in the level
                    iw_row <= d.base + g_off[29:0] + ({17'h0, t_y[12:0] + j0} * {14'h0, g_pitchl});
                    st <= S_WI7;
                end
                S_WI7: if (iw_cmd_ready) begin
                    iw_cmd_valid <= 1'b1; iw_pitch <= g_pitchl; iw_x <= t_x[12:0]; iw_w <= t_w[12:0]; iw_i0 <= i0;
                    iw_npix <= npix; iw_tfmt <= t_tf; iw_sf <= sf_of(dfmt);
                    dleft <= (pbytes + 16'd3) >> 2;
                    st <= S_WI8;
                end
                S_WI8: st <= S_WID;                          // the engine has taken the command
                S_WID: if (dleft == 0 && !d_take && !d_pad) st <= S_WIW;
                S_WIW: if (!iw_busy && !iw_cmd_valid) begin
                    if (dpad) error(ERR_LEN, {16'h0, blen}); else finish();
                end
                // ---------------- READ_IMAGE ----------------
                S_RI3: begin
                    if (!tf_known(t_tf) || f[1][31:8] != 24'h0 || (raw_only(dfmt) && t_tf != TF_RAW)) error(ERR_RANGE, f[0]);
                    else if (t_bad)
                        error(ERR_RANGE, f[2]);
                    else begin
                        tbpp <= tbpp_of(t_tf, dfmt); wh <= t_w[12:0] * t_h[12:0];
                        dv_a <= f[4]; dv_b <= {3'h0, t_w[12:0]}; dv_start <= 1'b1;
                        st <= S_RI4;
                    end
                end
                S_RI4: if (dv_done) begin
                    // count 0 = to the end of the rectangle
                    if ({1'b0, f[4]} >= {7'h0, wh} || (f[5] != 0 && {1'b0, f[4]} + {1'b0, f[5]} > {7'h0, wh}))
                        error(ERR_RANGE, f[4]);
                    else begin
                        j0 <= dv_q[12:0]; i0 <= dv_r[12:0];
                        npix <= (f[5] == 0) ? 24'(wh - f[4][25:0]) : f[5][23:0];
                        st <= S_RI5;
                    end
                end
                S_RI5: if (xf_cmd_ready) begin
                    xf_cmd_valid <= 1'b1; xf_op <= XF_READ; xf_s_fmt <= sf_of(dfmt); xf_d_fmt <= sf_of(dfmt);
                    xf_s_row0 <= d.base + g_off[29:0]; xf_s_pitch <= g_pitchl;
                    xf_s_x <= t_x[12:0]; xf_s_y <= t_y[12:0]; xf_s_w <= t_w[12:0]; xf_s_h <= t_h[12:0];
                    xf_tfmt <= t_tf; xf_j0 <= j0; xf_i0 <= i0; xf_npix <= npix;
                    xf_key <= 1'b0; xf_rmw <= 1'b0; xf_mir_x <= 1'b0; xf_mir_y <= 1'b0; xf_bottom_up <= 1'b0;
                    r_len <= 30'(mul_bpp({8'h0, npix}, tbpp)); r_off <= 0; r_tag <= f[6]; r_base <= mul_bpp(f[4], tbpp);
                    r_src <= 1'b1;
                    st <= S_RCH;
                end
                // ---------------- FILL_IMAGE ----------------
                S_FI3: begin
                    // (RGBA4444 / RGBA5551 fill whole pixels: the engine's channel masks are RGB565's)
                    if (!tf_known(t_tf) || (raw_only(dfmt) && t_tf != TF_RAW) ||
                        ((dfmt == FMT_RGBA4444 || dfmt == FMT_RGBA5551) && f[1][11:8] != 4'hF)) error(ERR_RANGE, f[0]);
                    else if (t_bad)
                        error(ERR_RANGE, f[2]);
                    else st <= S_FI4;
                end
                S_FI4: if (xf_cmd_ready) begin
                    xf_cmd_valid <= 1'b1; xf_op <= XF_FILL; xf_s_fmt <= sf_of(dfmt); xf_d_fmt <= sf_of(dfmt);
                    xf_d_row0 <= d.base + g_off[29:0]; xf_d_pitch <= g_pitchl;
                    xf_d_x <= t_x[12:0]; xf_d_y <= t_y[12:0]; xf_d_w <= t_w[12:0]; xf_d_h <= t_h[12:0];
                    xf_tfmt <= t_tf; xf_value <= f[4]; xf_mask <= f[1][11:8];
                    xf_rmw <= (dfmt == FMT_RGB565) && f[1][10:8] != 3'b000 && f[1][10:8] != 3'b111;
                    xf_key <= 1'b0; xf_mir_x <= 1'b0; xf_mir_y <= 1'b0; xf_bottom_up <= 1'b0;
                    st <= S_XFW;
                end
                // ---------------- COPY_IMAGE ----------------
                S_CP2: begin                                 // source checked: keep its level, check the destination
                    sd <= d; s_row0 <= d.base + g_off[29:0]; s_pitchl <= g_pitchl; s_wl <= g_wl; s_hl <= g_hl;
                    desc_check(f[1], S_CP3);
                end
                S_CP3: begin
                    if (((raw_only(sd.format) || raw_only(dfmt)) && sd.format != dfmt) || f[6][31:3] != 29'h0) error(ERR_RANGE, f[6]);
                    else if (cp_bad_s) error(ERR_RANGE, f[3]);           // (registered a cycle ahead, above)
                    else if (cp_bad_d) error(ERR_RANGE, f[5]);
                    else begin
                        // DDA divisions: sw / dw, sw / (2 dw), sh / dh, sh / (2 dh)
                        dvi <= 4'd0; dv_a <= {16'h0, cp_sw}; dv_b <= cp_dw; dv_start <= 1'b1; st <= S_CP4;
                    end
                end
                S_CP4: if (dv_done) begin
                    case (dvi)
                        4'd0: begin xf_hq <= dv_q[12:0]; xf_hr2 <= {dv_r[12:0], 1'b0};
                                    dv_a <= {16'h0, cp_sw}; dv_b <= {cp_dw[14:0], 1'b0}; end
                        4'd1: begin xf_hu0 <= dv_q[12:0]; xf_ha0 <= dv_r[13:0];
                                    dv_a <= {16'h0, cp_sh}; dv_b <= cp_dh; end
                        4'd2: begin xf_vq <= dv_q[12:0]; xf_vr2 <= {dv_r[12:0], 1'b0};
                                    dv_a <= {16'h0, cp_sh}; dv_b <= {cp_dh[14:0], 1'b0}; end
                        default: begin xf_vu0 <= dv_q[12:0]; xf_va0 <= dv_r[13:0]; end
                    endcase
                    dvi <= dvi + 4'd1;
                    if (dvi == 4'd3) st <= S_CP5; else dv_start <= 1'b1;
                end
                S_CP5: if (xf_cmd_ready) begin
                    xf_cmd_valid <= 1'b1; xf_op <= XF_COPY; xf_s_fmt <= sf_of(sd.format); xf_d_fmt <= sf_of(dfmt);
                    xf_s_row0 <= s_row0; xf_s_pitch <= s_pitchl;
                    xf_s_x <= cp_sx[12:0]; xf_s_y <= cp_sy[12:0]; xf_s_w <= cp_sw[12:0]; xf_s_h <= cp_sh[12:0];
                    xf_d_row0 <= d.base + g_off[29:0]; xf_d_pitch <= g_pitchl;
                    xf_d_x <= cp_dx[12:0]; xf_d_y <= cp_dy[12:0]; xf_d_w <= cp_dw[12:0]; xf_d_h <= cp_dh[12:0];
                    xf_mir_x <= f[6][0]; xf_mir_y <= f[6][1]; xf_key <= f[6][2]; xf_key_lo <= f[7]; xf_key_hi <= f[8];
                    // an overlapping 1:1 copy inside one surface is exact when rows go bottom-up for dy > sy
                    xf_bottom_up <= (s_row0 == d.base + g_off[29:0]) && (s_pitchl == g_pitchl) && cp_sw == cp_dw &&
                                    cp_sh == cp_dh && !f[6][1] && cp_dy > cp_sy;
                    xf_rmw <= 1'b0;
                    st <= S_XFW;
                end
                S_XFW: st <= S_XF;                           // the engine has taken the command
                S_XF: if (!xf_busy && !xf_cmd_valid) finish();
                // ---------------- SET_PALETTE ----------------
                S_PALD: if (dleft == 0 && !d_take) st <= S_PALP;
                S_PALP: if (!pal_busy) begin pal_pub <= 1'b1; st <= S_PALW; end
                S_PALW: if (!pal_busy && !pal_pub) finish();
                // ---------------- CURSOR SHAPE ----------------
                S_CSD: if (dleft == 0 && !d_take) finish();
                // ---------------- WRITE / READ_BUFFER (B buffers) ----------------
                S_BUF1: begin bl8 <= blen - 16'd8; st <= S_BUF2; end          // table read latency
                S_BUF2: begin bq_v <= bt_rv; bq_b <= bt_rb; bq_s <= bt_rs; st <= S_BUF3; end   // the read registered
                S_BUF3: begin
                    if (!bq_v) error(ERR_HANDLE, f[0]);
                    else if (mtype == T_WRITE_BUFFER) begin
                        if ({1'b0, f[1]} + {17'h0, bl8} > {3'b000, bq_s}) error(ERR_RANGE, f[1]);
                        else begin
                            x_own <= 1'b1; x_op <= OP_WRITE; x_addr <= bq_b + f[1][29:0]; x_len <= {14'h0, bl8};
                            dleft <= (bl8 + 16'd3) >> 2; st <= S_X;
                        end
                    end else begin
                        if (f[2] == 0) finish();
                        else if ({1'b0, f[1]} + {1'b0, f[2]} > {3'b000, bq_s}) error(ERR_RANGE, f[1]);
                        else begin
                            x_own <= 1'b1; x_op <= OP_READ; x_addr <= bq_b + f[1][29:0]; x_len <= f[2][29:0];
                            r_len <= f[2][29:0]; r_off <= 0; r_tag <= f[3]; r_base <= 0; r_src <= 1'b0; st <= S_X;
                        end
                    end
                end
                // ---------------- DRAW_STATE: the bucket, in its order ----------------
                S_DW0: if (dleft == 0 && !d_take && !d_pad) st <= S_DWH11;        // words 0..10 passed through
                S_DWH11: if (h_word) begin dw_h11 <= got_last ? 32'h0 : c_data; if (got_last) dpad <= 1'b1;
                                           if (h_take && c_last) got_last <= 1'b1; st <= S_DWH12; end
                S_DWH12: if (h_word) begin dw_off <= got_last ? 32'h0 : c_data; if (got_last) dpad <= 1'b1;
                                           if (h_take && c_last) got_last <= 1'b1;
                                           bt_ridx <= dw_h11[9:0]; st <= S_DWL1; end
                S_DWL1: st <= S_DWL2;
                S_DWL2: begin bq_v <= bt_rv; bq_b <= bt_rb; bq_s <= bt_rs; st <= S_DWL3; end   // the read registered
                S_DWL3: begin                                // program: a B buffer, word aligned, inside the buffer
                    dw_plen <= dw_h11[27:16]; dw_pw <= 29'((bq_b + dw_off[29:0]) >> 2);
                    if (!(dw_h11[15] && !dw_h11[14] && dw_h11[13:10] == 4'h0) || !bq_v) begin
                        dw_err <= 1'b1; dw_ecode <= ERR_HANDLE; dw_edet <= dw_h11; dw_plen <= 12'd0; dw_pw <= 29'd0;
                    end else if (dw_off[2:0] != 3'd0 || dw_h11[31:28] != 4'h0 ||            // 8-byte instructions
                                 {2'b00, dw_off} + {17'h0, dw_h11[27:16], 3'b000} > {4'h0, bq_s}) begin
                        dw_err <= 1'b1; dw_ecode <= ERR_RANGE; dw_edet <= dw_off; dw_plen <= 12'd0; dw_pw <= 29'd0;
                    end
                    st <= S_DWP;
                end
                S_DWP: if (x_wd_ready) st <= S_DWH13;
                S_DWH13: if (h_word) begin
                    if (got_last) dpad <= 1'b1;
                    if (h_take && c_last) got_last <= 1'b1;
                    if (!got_last && c_data != {25'h0, dw_n} && !dw_err) begin
                        dw_err <= 1'b1; dw_ecode <= ERR_LEN; dw_edet <= c_data;
                    end
                    st <= S_DWH14;
                end
                S_DWH14: if (h_word) begin                  // the constant alpha in the colour channels' units (bucket word 13)
                    dw_h14 <= got_last ? 32'h0 : c_data; if (got_last) dpad <= 1'b1;
                    if (h_take && c_last) got_last <= 1'b1;
                    dw_k <= 2'd0; st <= S_DWN;
                end
                S_DWN: if (x_wd_ready) begin dw_k <= 2'd0; st <= S_DWZ; end
                S_DWZ: if (x_wd_ready) begin                // word 13 = the constant alpha word, 14..15 = 0
                    dw_k <= dw_k + 2'd1;
                    if (dw_k == 2'd2) begin dw_si <= 3'd0; st <= S_DWS0; end
                end
                // samplers: {wrap_r[26:25], incomplete[24], wrap_t[23:22], wrap_s[21:20], filt[19:16], image handle} -> 4 words
                S_DWS0: if (h_word) begin
                    dw_sw <= got_last ? 32'h0 : c_data; if (got_last) dpad <= 1'b1;
                    if (h_take && c_last) got_last <= 1'b1;
                    it_ridx <= c_data[9:0]; dw_k <= 2'd0;
                    if (got_last || c_data[15:0] == 16'h0) begin
                        dw_s[0] <= 32'h0; dw_s[1] <= 32'h0; dw_s[2] <= 32'h0; dw_s[3] <= 32'h0; st <= S_DWS3;
                    end else st <= S_DWS1;
                end
                S_DWS1: begin it_w <= ~it_w; if (it_w) st <= S_DWS2; end     // table read latency (two cycles)
                S_DWS2: begin
                    dw_s[0] <= {3'b000, 29'(it_rdesc.base >> 2)};
                    dw_s[1] <= {4'h0, it_rdesc.height[11:0], 4'h0, it_rdesc.width[11:0]};
                    dw_s[2] <= {15'h0, !it_rdesc.vol && it_rdesc.layers == 8'd6, dw_sw[24], 1'b0, dw_sw[23:22], dw_sw[21:20], dw_sw[19:16],
                                it_rdesc.levels, tmu_fmt(it_rdesc.format)};
                    dw_s[3] <= {21'h0, it_rdesc.vol, dw_sw[26:25], it_rdesc.vol ? it_rdesc.layers : 8'd1};
                    if (!(dw_sw[15] && dw_sw[14] && dw_sw[13:10] == 4'h0) || !it_rvalid || it_rdesc.samples != 2'd0 ||
                        !(it_rdesc.format == FMT_RGBA8 || it_rdesc.format == FMT_RGB565 ||
                          it_rdesc.format == FMT_RGBA4444 || it_rdesc.format == FMT_RGBA5551) ||
                        it_rdesc.width > 13'd2048 || it_rdesc.height > 13'd2048 ||
                        !(it_rdesc.vol || it_rdesc.layers == 8'd1 || (it_rdesc.layers == 8'd6 && it_rdesc.width == it_rdesc.height))) begin
                        if (!dw_err) begin dw_err <= 1'b1; dw_ecode <= ERR_HANDLE; dw_edet <= dw_sw; end
                        dw_s[0] <= 32'h0; dw_s[1] <= 32'h0; dw_s[2] <= 32'h0; dw_s[3] <= 32'h0;
                    end
                    st <= S_DWS3;
                end
                S_DWS3: begin
                    if (x_wd_ready) begin
                        dw_k <= dw_k + 2'd1;
                        if (dw_k == 2'd3) begin
                            dw_si <= dw_si + 3'd1;
                            if (dw_si == 3'd7) begin
                                if (dw_n == 7'd0) st <= S_DWE; else begin dleft <= {7'h0, dw_n, 2'b00}; st <= S_DWU; end
                            end else st <= S_DWS0;
                        end
                    end
                end
                S_DWU: if (dleft == 0 && !d_take && !d_pad) st <= S_DWE;          // uniforms passed through
                S_DWE: if (!x_busy && !x_cmd_valid) begin
                    if (dw_err) error(dw_ecode, dw_edet);
                    else if (dpad) error(ERR_LEN, {16'h0, blen});
                    else begin c_draws <= c_draws + 1; finish(); end
                end
                // ---------------- FLUSH: one frame of the render stream into the image ----------------
                S_FL1: begin it_w <= ~it_w; if (it_w) st <= S_FL2; end       // table read latency (two cycles)
                S_FL2: begin                                 // the colour image
                    if (!it_rvalid || it_rdesc.samples != 2'd0 || !rt_fmt_ok(it_rdesc.format)) flush_fail(ERR_HANDLE, f[0]);
                    else if ({3'h0, it_rdesc.width} > 16'(MAXW) || it_rdesc.pitch[4:0] != 5'd0 || it_rdesc.pitch[15:5] == 11'd0)
                        flush_fail(ERR_RANGE, f[0]);
                    else begin
                        rd_rt_base <= 29'(it_rdesc.base >> 2); rd_rt_bpl <= it_rdesc.pitch[15:5];
                        rd_rt_width <= it_rdesc.width[10:0]; rd_rt_fmt <= tmu_fmt(it_rdesc.format);
                        rd_rt_height <= it_rdesc.height[11:0];
                        fl_w <= it_rdesc.width; fl_h <= it_rdesc.height; fl_bpp16 <= (it_rdesc.format != FMT_RGBA8);
                        rd_clear_c16 <= {f[1][12:0], f[1][25:13], f[2][12:0], f[2][25:13]};   // {R, G, B, A}
                        rd_clear_en <= f[0][16]; rd_clear_sc <= f[0][17]; rd_clear_cmask <= f[0][21:18];
                        rd_clear_dither <= f[0][22]; rd_clear_depth <= f[3][23:0]; rd_clear_stencil <= f[3][31:24];
                        rd_clear_depth_en <= f[0][23]; rd_clear_stencil_en <= f[0][24]; rd_depth16 <= f[0][25];
                        rd_clear_smask <= f[6][23:16];
                        rd_clear_x0 <= f[4][12:0]; rd_clear_y0 <= f[4][28:16]; rd_clear_x1 <= f[5][12:0];
                        rd_clear_y1 <= f[5][28:16];
                        if (f[6][15:0] == 16'h0) begin rd_ds_en <= 1'b0; flush_go(); end
                        else if (!(f[6][15] && f[6][14] && f[6][13:10] == 4'h0)) flush_fail(ERR_HANDLE, f[6]);
                        else begin it_ridx <= f[6][9:0]; st <= S_FL3; end
                    end
                end
                S_FL3: begin it_w <= ~it_w; if (it_w) st <= S_FL4; end       // table read latency (two cycles)
                S_FL4: begin                                 // the depth-stencil image: D24S8, the colour image's size
                    if (!it_rvalid || it_rdesc.samples != 2'd0 || it_rdesc.format != FMT_D24S8) flush_fail(ERR_HANDLE, f[6]);
                    else if (it_rdesc.width != fl_w || it_rdesc.height != fl_h || it_rdesc.pitch[4:0] != 5'd0 ||
                             {1'b0, it_rdesc.pitch[15:5]} < ({1'b0, rd_rt_bpl} << fl_bpp16))
                        flush_fail(ERR_RANGE, f[6]);
                    else begin
                        rd_ds_en <= 1'b1; rd_ds_base <= 29'(it_rdesc.base >> 2); rd_ds_bpl <= it_rdesc.pitch[15:5];
                        flush_go();
                    end
                end
                S_FLW: if (rd_done) begin                      // (a discarded frame: the error after it)
                    if (fl_err) error(fl_ecode, fl_edet);
                    else if (rd_frame_err) error(ERR_SHADER, f[0]);   // the frame is written, with holes: the GL error
                    else finish();
                end
                default: begin st <= S_W0; for (int i = 0; i < 9; i++) f[i] <= 0; end
            endcase
            // present-done events go out whenever the channel is free (they may complete at any later vsync)
            if (pd_v && !e_valid) begin                        // S_EV waits while pd_v
                e_valid <= 1'b1; e_data <= {pd_dropped, pd_frames, pd_fid, {16'h0, pd_sub, EV_PRESENT}, pd_seq};
                pd_v <= 1'b0;
            end
        end
    end
    assign cs_idx = 12'(f[1][15:0] + dk);
    wire _unused = &{1'b0, op_hold, it_rdesc.rsvd, d.rsvd, sd.rsvd, pbytes};
endmodule
`default_nettype wire
