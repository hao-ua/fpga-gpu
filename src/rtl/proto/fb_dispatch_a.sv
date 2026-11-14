// fb_dispatch_a — board A command dispatcher, clk_sys.
// Pops entries from the ingress queue (fb_pkg entry format), executes A-local commands in order on
// xfer_lin and the buffer table, and reports through reply channels EVENT (fb_reply 3) and
// READ_DATA (fb_reply 4). Commands for board B are forwarded unchanged, in queue order, as link CMD packets
// {w0 = {0, type, body bytes}, w1 = seq, body words} while link_ok (link trained both ways and B's STATUS seen):
// every command whose handle has the B bit (bit 15: CREATE / WRITE / READ_BUFFER, DESTROY), MEM_WRITE /
// MEM_READ / MEMTEST with board = 1, the image commands (CREATE_IMAGE, WRITE / READ / FILL / COPY_IMAGE,
// GEN_MIPMAP; images live on B only, so a handle without the B bit is ERR_HANDLE here), COPY_BUFFER and the
// display / render commands (SET_MODE, PRESENT, CURSOR, sequenced CURSOR_POS, SET_PALETTE, DRAW_STATE, FLUSH — a FLUSH
// then starts A's render side, which streams the frame's quads to B; the dispatcher
// does NOT wait for the stream's end: the next frame's FRAME_BEGIN and draws run meanwhile in the other record / bin
// bank (a FRAME_BEGIN waits only when its bank is the one being streamed; a FENCE waits for the stream too). A render
// monitor beside the command FSM gives a frame up after RENDER_TIMEOUT (ended in-band for B) or when a credit SYNC goes
// out (A's start, B restarted: B drops every FLUSH older than the SYNC, so a forwarded FLUSH not yet streamed is dropped
// too) and reports EV_ERROR ERR_RENDER with the FLUSH's seq at the next command boundary; a link drop alone gives
// nothing up (the link layer keeps the data across a retrain). B answers with its own EVENT / READ_DATA
// (link_a_port relays them) and reports what it does not implement as ERR_TYPE. Without link_ok they return
// EV_ERROR ERR_NO_B. FENCE waits for A's own work, then is forwarded to B (B's EVENT completes it) or, without
// B, answered here. Fields already read for the decision are re-emitted in front of the rest of the body.
// A command for B that finds the link down waits up to B_WAIT cycles for it (a retrain takes a few ms and the link
// layer holds the data across it); only then is B declared gone: that command and every further one fail at once
// with ERR_NO_B (FENCE: answered here) until the link is up again.
// Bodies (little-endian words after the 8-byte header):
//   MEM_WRITE    {pad, board}, addr, data...            MEM_READ   {pad, board}, addr, len, tag
//   MEMTEST      {pad, board}, addr, len, seed, flags   (addr, len 32-byte aligned; flags b0 write b1 verify)
//   CREATE_BUFFER {usage, handle}, base, size           DESTROY    {pad, handle}
//   WRITE_BUFFER {pad, handle}, offset, data...         READ_BUFFER {pad, handle}, offset, len, tag
//   FENCE        id
// Replies: EVENT words {hdr, seq, {sub-message index, code}, a0, a1, a2}; READ_DATA {hdr, seq, tag, offset, data...} in
// chunks of RD_CHUNK bytes.
// BATCH (fb_pkg T_BATCH): the entry's body is a sequence of sub-messages {u8 type, u8 flags,
// u16 body bytes, body padded to a word}. D_SUBHDR reads a sub-header from the body stream and the per-type FSM then
// runs on the sub-body exactly as on a datagram of its own (`wleft` = the sub-body's words, `eleft` = the entry's words
// after it); at D_END the sub-body's unread words are dropped and the next sub-header follows. The sub-message index
// (`sub_idx`, 0 outside a batch) goes into every EVENT's code word [15:8] and into the CMD header byte 3 of a command
// forwarded to B (B echoes it). An error found here (ERR_*, a malformed sub-header) ends the batch: the rest of the entry
// is dropped unexecuted; an error B finds in a forwarded sub-command is B's EVENT and does not stop the batch.
`default_nettype none
module fb_dispatch_a #(
    parameter [31:0] HEAP_LO = 32'h1800_0000,       // 384 MB: below it the ingress ring (64 MB) and the GPU-internal records / bin lists (two banks)
    parameter [31:0] HEAP_HI = 32'h4000_0000,       // 1 GB (exclusive)
    parameter int    B_WAIT  = 10_000_000,          // 100 ms: how long a B command waits for the board link
    parameter int    RENDER_TIMEOUT = 300_000_000   // 3 s: a FLUSH's frame that B has not finished by then is aborted (ERR_RENDER)
)(
    input  wire          clk, rst,
    // ingress queue pop side
    input  wire          q_valid,
    input  wire [255:0]  q_data,
    output wire          q_ready,
    // transfer engine
    output reg           x_cmd_valid,
    output reg  [1:0]    x_op,
    output reg  [29:0]   x_addr, x_len,
    output reg  [31:0]   x_seed,
    output reg  [1:0]    x_flags,
    input  wire          x_cmd_ready, x_busy,
    output wire          x_wd_valid,
    output wire [31:0]   x_wd_data,
    input  wire          x_wd_ready,
    input  wire          x_rd_valid,
    input  wire [31:0]   x_rd_data,
    output wire          x_rd_ready,
    input  wire          x_mt_done,
    input  wire [31:0]   x_mt_errors,
    input  wire [29:0]   x_mt_first_bad,
    // reply channel: EVENT
    output wire          ev_req,
    output wire [15:0]   ev_len,
    output wire          ev_valid,
    output wire [31:0]   ev_data,
    input  wire          ev_ready, ev_done,
    // reply channel: READ_DATA
    output wire          rd_req,
    output wire [15:0]   rd_len,
    output wire          rd_valid,
    output wire [31:0]   rd_data,
    input  wire          rd_ready, rd_done,
    // board link: CMD word stream to link_a_port
    input  wire          link_ok,
    output wire          f_valid,
    output wire [31:0]   f_data,
    output wire [15:0]   f_len,
    output wire          f_last,
    input  wire          f_ready,
    // statistics
    output reg  [31:0]   c_cmds, c_errs, last_err, last_err_seq, mt_last_errors,
    output reg  [31:0]   c_fwd,
    output reg  [31:0]   c_hdr_wait, c_gwait,    // profile (STATUS layout 10): cycles idle with no command queued, cycles a geometry command waited (D_GWAIT)
    // the geometry stage's command port (a_geom). Every geometry command waits for g_busy low
    // (the geometry side); a FRAME_BEGIN also for g_fb_ok (its bank is not being streamed). A FLUSH, once forwarded to
    // B, waits for the geometry and the previous stream (g_rbusy), starts A's render side (g_render) and goes on;
    // the monitor below watches g_render_done.
    // Draws overlap: a DRAW waits for g_draw_ok only, VS_UNIFORMS / VERTEX_ATTR for !g_copy_busy
    // (they write shadows), VS_STATE for nothing (the DRAW snapshots it); VS_PROGRAM and FRAME_BEGIN still drain (g_busy).
    input  wire          g_busy, g_rbusy, g_fb_ok, g_draw_ok, g_copy_busy, g_frame_done, g_ld_done, g_render_done,
    output reg           g_render,
    output reg           g_abort,                // one cycle: the render side and the link's render FIFO give the frame up
    output reg           g_abort_soft,           // one cycle (RENDER_TIMEOUT): the render side ends the frame IN-BAND at its next item
                                                 // boundary (FRAME_END with its abort bit): B pairs it with its FLUSH as any frame
    output reg           g_resync,               // one cycle (twice RENDER_TIMEOUT, the soft end did not get out): a credit re-sync,
                                                 // whose SYNC (sync_go) then kills the frame (g_abort) on both boards
    input  wire          sync_go,                // link_a_port sends a SYNC (A's start, B restarted)
    output reg           g_frame, g_ld_start, g_param_we, g_ad_we, g_ac_we, g_st_we, g_dr_start,
    output reg  [28:0]   g_ld_addr,
    output reg  [29:0]   g_ad_base, g_ad_stride, g_ad_off,   // bytes (VERTEX_ATTR: the buffer's base; stride, offset)
    output reg  [29:0]   g_dr_idx_base,         // byte address of index 0 (DRAW)
    output reg  [2:0]    g_ad_type,             // attr_conv type code
    output reg           g_ad_norm,
    output reg  [11:0]   g_ld_len, g_st_w, g_st_h,
    output reg  [7:0]    g_param_idx, g_st_fsmask,
    output reg  [95:0]   g_param_data, g_ac_const, g_st_vp,
    output reg  [95:0]   g_st_gb,                // {gyhi, gylo, gxhi, gxlo}: the guard band's NDC bounds (VS_STATE words 9..12)
    output reg  [47:0]   g_st_z,                 // {zo, zs}: the depth range (VS_STATE words 7, 8)
    output reg  [47:0]   g_st_sc,                // {y1, x1, y0, x0}, 12 bits each: the draw's rectangle [x0, x1) x [y0, y1) in pixels — its
                                                 // scissor box cut to the target (VS_STATE words 13, 14 when the command has them,
                                                 // else the whole target; x1 / y1 never beyond the target)
    output reg  [3:0]    g_ad_idx, g_ac_idx,
    output reg           g_ad_en, g_st_cw, g_dr_indexed, g_dr_u8,
    output reg           g_st_z0,                // VS_STATE word 4 bit 24: the clip volume 0 <= z <= w (depth reversed by the library)
    output reg  [2:0]    g_ad_size, g_dr_mode,
    output reg  [31:0]   g_dr_first, g_dr_count,
    output reg  [1:0]    g_st_cull,
    output reg  [6:0]    g_st_lw,                // the line width (pixels, Q10)
    output reg  [12:0]   g_st_did
);
    import fb_pkg::*;

    // ---------------- buffer table ----------------
    reg         bt_we; reg [9:0] bt_widx; reg bt_wv; reg [3:0] bt_wu; reg [29:0] bt_wb, bt_ws;
    reg  [9:0]  bt_ridx;
    wire        bt_rv; wire [3:0] bt_ru; wire [29:0] bt_rb, bt_rs;
    buf_table u_bt (.clk(clk), .rst(rst), .we(bt_we), .widx(bt_widx), .wvalid(bt_wv), .wusage(bt_wu),
        .wbase(bt_wb), .wsize(bt_ws), .ridx(bt_ridx), .rvalid(bt_rv), .rusage(bt_ru), .rbase(bt_rb), .rsize(bt_rs));

    // ---------------- entry / word unpacker ----------------
    typedef enum logic [4:0] {D_HDR, D_FIELDS, D_EXEC, D_LOOK1, D_LOOK2, D_XCMD, D_WDATA, D_RDATA,
                              D_MTWAIT, D_FENCE, D_EVENT, D_FWD, D_END, D_WAITB,
                              D_GWAIT, D_GFRAME, D_GLOAD, D_GUNI, D_GRWAIT, D_SUBHDR, D_LOOK1B} dst_t;
    dst_t        st;
    // the buffer table's read, registered once more before the range checks (timing at 120 MHz: the block RAM's latch ->
    // the tag compare -> the 32-bit add and compare -> the error fields is 13 levels); a lookup takes one cycle more (a
    // handful per command)
    reg          bl_v; reg [29:0] bl_b, bl_s;
    // the render monitor: cycles of the frame being streamed (RENDER_TIMEOUT), the FLUSH's seq / sub index for the error event
    reg  [31:0]  rt, rerr_rt, r_seq;
    reg  [7:0]   r_idx;
    reg          r_active, r_aborted, rerr_pend;
    reg          fl_dead;                           // a SYNC went out since the FLUSH waiting in D_GRWAIT was forwarded
    reg          r_hard;                            // the frame was killed (the SYNC) or a re-sync was asked for
    // BATCH: the entry is a batch; its words after the current sub-message; the running sub-message's index and the count
    reg          in_batch;
    reg  [15:0]  eleft;
    reg  [7:0]   sub_idx, sub_n;
    reg  [255:0] bw;
    reg  [2:0]   bwi;
    reg          bvalid;
    reg  [15:0]  wleft, bleft;                      // body words not consumed, body bursts not loaded
    reg  [7:0]   mtype;
    reg  [15:0]  blen;
    reg  [31:0]  mseq;
    reg  [31:0]  f [0:14];                          // fixed fields (VS_STATE has 13, or 15 with its rectangle)
    reg  [3:0]   fi, nf;
    reg  [31:0]  wcur;                              // = bw[bwi], a REGISTER (its block below)
    // VS_STATE's rectangle, words 13 {y0, x0} and 14 {y1, x1} (16 bits each, as the draw state's scissor words): x1 / y1
    // cut to the target of the same command (word 5), a start beyond 4 095 held there (an empty rectangle: nothing drawn)
    wire [11:0]  sc_x0c = (f[13][15:12] != 4'd0) ? 12'hFFF : f[13][11:0];
    wire [11:0]  sc_y0c = (f[13][31:28] != 4'd0) ? 12'hFFF : f[13][27:16];
    wire [11:0]  sc_x1c = (f[14][15:0]  > {4'd0, f[5][11:0]})  ? f[5][11:0]  : f[14][11:0];
    wire [11:0]  sc_y1c = (f[14][31:16] > {4'd0, f[5][27:16]}) ? f[5][27:16] : f[14][27:16];

    // field counts per type. VS_STATE's are 13, or 15 when its body has the two rectangle words (60 bytes):
    // the longer form is the same type, so a host that sends it to a board A that reads 13 loses nothing but the rectangle
    // (the two words are a sub-body's unread tail there), and one that sends 13 words here gets the whole target
    function automatic [3:0] nfields(input [7:0] t, input [15:0] len);
        case (t)
            T_MEM_WRITE, T_WRITE_BUFFER:  nfields = 4'd2;
            T_MEM_READ, T_READ_BUFFER:    nfields = 4'd4;
            T_MEMTEST:                    nfields = 4'd5;
            T_CREATE_BUFFER:              nfields = 4'd3;
            T_DESTROY, T_FENCE, T_CREATE_IMAGE, T_WRITE_IMAGE, T_READ_IMAGE, T_FILL_IMAGE, T_COPY_IMAGE,
            T_GEN_MIPMAP:                 nfields = 4'd1;
            T_VS_PROGRAM:                 nfields = 4'd3;     // {0, handle}, byte offset, instructions
            T_VS_UNIFORMS:                nfields = 4'd1;     // {count, first}, then count x 4 words
            T_VERTEX_ATTR:                nfields = 4'd8;     // {normalized[24], type[22:20], size[18:16], enabled[8], index[3:0]}, {0, handle}, stride (bytes), offset (bytes), const x y z w
            T_VS_STATE:                   nfields = (len >= 16'd60) ? 4'd15 : 4'd13;    // sxs sxo sys syo (x16), {0, fsmask, line width[15:9], front_cw, cull}, {height, width}, draw id, zs, zo (depth range), gxlo gxhi gylo gyhi (guard band) [, {y0, x0}, {y1, x1}: the rectangle]
            T_DRAW:                       nfields = 4'd5;     // {0, u8, indexed, mode}, {0, handle}, byte offset, first (32 bits), count (32 bits)
            T_FRAME_BEGIN:                nfields = 4'd1;
            default:                      nfields = 4'd0;
        endcase
    endfunction

    // forwarding to B: payload word fw_i (0 = {0, type, bytes}, 1 = seq, 2 .. 1 + fi = the fields already read,
    // then the unpacker's remaining body words)
    reg  [15:0]  fw_i, fw_len;
    wire         fw_body = (fw_i >= 16'd2 + {12'h0, fi});
    wire take_field = (st == D_FIELDS) && bvalid && fi < nf && wleft != 0;
    wire take_wd    = (st == D_WDATA) && bvalid && wleft != 0 && x_wd_ready;
    wire take_fwd   = (st == D_FWD) && fw_body && bvalid && wleft != 0 && f_ready;
    wire take_uni   = (st == D_GUNI) && bvalid && wleft != 0;
    wire take_sub   = (st == D_SUBHDR) && bvalid;                              // a sub-header word (eleft != 0 here)
    wire take_skip  = (st == D_END) && in_batch && bvalid && wleft != 0;      // a sub-body's unread words
    wire want_words = (st == D_FIELDS && fi < nf && wleft != 0) || (st == D_WDATA && wleft != 0) ||
                      (st == D_FWD && fw_body && wleft != 0) || (st == D_GUNI && wleft != 0) ||
                      (st == D_SUBHDR) || (st == D_END && in_batch && wleft != 0);
    wire [16:0]  sub_words = ({1'b0, wcur[31:16]} + 17'd3) >> 2;             // the sub-header's body in words
    assign f_valid  = (st == D_FWD) && (!fw_body || (bvalid && wleft != 0));
    assign f_data   = (fw_i == 16'd0) ? {sub_idx, mtype, blen} : (fw_i == 16'd1) ? mseq :
                      !fw_body ? f[fw_i[3:0] - 4'd2] : wcur;
    // the geometry commands: g_op = the pending operation after the g_busy wait
    reg  [2:0]   g_op;                              // 0 frame, 1 load, 2 uniforms, 3 attribute, 4 state, 5 draw
    // the D_GWAIT condition as a register (timing at 120 MHz: the geometry's busy — an OR over the vertex core's warps and
    // stages — would sit in the clock enable of every register loaded in D_GWAIT): it holds the previous cycle's condition
    // while in D_GWAIT, so a geometry command fires one cycle later (a few per frame) and never on a stale g_op
    reg gw_ok;
    // a geometry command that READS A's memory (a DRAW's indices and vertices, a program LOAD) waits for the transfer engine
    // to be idle as well: WRITE_BUFFER's last data words go to xfer_lin, whose burst FIFO can still hold up to 14 bursts of
    // them when the dispatcher reaches the next command, and a DRAW fetching the end of that buffer would read what the
    // streaming ring held before (one-frame phantom triangles). The write-after-read direction waits for the geometry in
    // D_XCMD.
    wire x_idle = !x_busy && !x_cmd_valid;
    wire gw_c = (g_op == 3'd0) ? (!g_busy && g_fb_ok) : (g_op == 3'd1) ? (!g_busy && x_idle) :
                (g_op == 3'd2 || g_op == 3'd3) ? !g_copy_busy : (g_op == 3'd4) ? 1'b1 : (g_draw_ok && x_idle);
    always @(posedge clk) gw_ok <= ~rst & (st == D_GWAIT) & gw_c;
    reg  [1:0]   uni_k;                             // uniform word 0..3 being packed
    reg  [29:0]  g_base;                            // the looked-up buffer base (bytes)
    assign f_len    = fw_len;
    assign f_last   = (fw_i == fw_len - 16'd1);
    wire load_body  = !bvalid && bleft != 0 && q_valid && (want_words || (st == D_END && !in_batch));
    // the deferred ERR_RENDER takes the D_HDR cycle without reading a header: the queue must NOT pop then (a pop would drop
    // the next entry's header burst, and the dispatcher would read that entry's body as a header)
    wire load_hdr   = (st == D_HDR) && q_valid && !rerr_pend;
    assign q_ready  = load_body || load_hdr;
    // the current body word as a register: loaded with word 0 of a burst, stepped with bwi (timing at 120 MHz: keeps the
    // 8 : 1 word select out of the path into the unpacker's decode). Same priority as bw / bwi
    wire bw_adv = take_field || take_wd || take_fwd || take_uni || take_skip || take_sub;
    wire [255:0] bw_src = load_body ? q_data : bw;
    always @(posedge clk) begin
        if (load_body) wcur <= q_data[31:0];
        if (bw_adv)    wcur <= bw_src[(bwi + 3'd1)*32 +: 32];
    end
    assign x_wd_valid = (st == D_WDATA) && bvalid && wleft != 0;
    assign x_wd_data  = wcur;

    // ---------------- reply state ----------------
    reg  [31:0] ev_w [0:5]; reg ev_pend; reg [2:0] ev_i;
    reg  [29:0] r_total, r_off; reg [31:0] r_tag; reg [15:0] r_n; reg r_pend; reg [15:0] r_i;
    wire [15:0] r_words = (r_n + 16'd3) >> 2;
    assign ev_req   = ev_pend;
    assign ev_len   = 16'd24;
    assign ev_valid = ev_pend;
    assign ev_data  = ev_w[ev_i];
    assign rd_req   = r_pend;
    assign rd_len   = 16'd16 + r_n;
    assign rd_valid = r_pend && ((r_i < 4) || x_rd_valid);
    assign rd_data  = (r_i == 0) ? hdr_w0(R_READ_DATA) : (r_i == 1) ? mseq : (r_i == 2) ? r_tag :
                      (r_i == 3) ? {2'b00, r_off} : x_rd_data;
    assign x_rd_ready = r_pend && r_i >= 4 && rd_ready;

    reg [7:0]  e_code; reg [31:0] e_a0, e_a1, e_a2; reg e_is_err;
    reg [1:0]  pend_op;                             // xfer op launched in D_XCMD
    localparam [1:0] OP_WRITE = 2'd0, OP_READ = 2'd1, OP_MEMTEST = 2'd2;

    // forward the command to board B: w0, seq, the fields read so far, then the rest of the body
    reg  [31:0]  bw_t;                              // D_WAITB timer
    reg          b_gone;                            // a wait timed out: fail B commands at once until link_ok
    task automatic forward();
        begin
            if (link_ok) begin fw_i <= 0; fw_len <= 16'd2 + {12'h0, fi} + wleft; st <= D_FWD; end
            else if (b_gone) raise_err(ERR_NO_B, 32'h0);
            else begin bw_t <= 0; st <= D_WAITB; end
        end
    endtask
    task automatic raise_err(input [7:0] code, input [31:0] detail);
        begin
            e_is_err <= 1'b1; e_code <= EV_ERROR; e_a0 <= {24'h0, code}; e_a1 <= {24'h0, mtype}; e_a2 <= detail;
            in_batch <= 1'b0;                       // an error ends a batch: the rest of the entry is dropped at D_END
            st <= D_EVENT;
        end
    endtask

    always @(posedge clk) begin
        bt_we <= 1'b0; x_cmd_valid <= 1'b0;
        g_frame <= 1'b0; g_ld_start <= 1'b0; g_param_we <= 1'b0; g_ad_we <= 1'b0; g_ac_we <= 1'b0; g_st_we <= 1'b0; g_dr_start <= 1'b0;
        g_render <= 1'b0; g_abort <= 1'b0; g_abort_soft <= 1'b0; g_resync <= 1'b0;
        if (rst) begin
            st <= D_HDR; bvalid <= 1'b0; bleft <= 0; wleft <= 0; ev_pend <= 1'b0; r_pend <= 1'b0;
            c_cmds <= 0; c_errs <= 0; last_err <= 0; last_err_seq <= 0; mt_last_errors <= 0; c_fwd <= 0; b_gone <= 1'b0;
            c_hdr_wait <= 0; c_gwait <= 0;
            r_active <= 1'b0; r_aborted <= 1'b0; r_hard <= 1'b0; rerr_pend <= 1'b0; rt <= 0; fl_dead <= 1'b0;
            in_batch <= 1'b0; sub_idx <= 0; sub_n <= 0; eleft <= 0;
        end else begin
            // ---- word unpacker ----
            if (load_body) begin bw <= q_data; bwi <= 0; bvalid <= !(st == D_END && !in_batch); bleft <= bleft - 1'b1; end
            if (take_field || take_wd || take_fwd || take_uni || take_skip || take_sub) begin
                bwi <= bwi + 1'b1;
                if (!take_sub) wleft <= wleft - 1'b1;       // a sub-header is an entry word, not a sub-body word
                if (bwi == 3'd7) bvalid <= 1'b0;
            end
            // ---- the render monitor: a frame B does not take is given up after RENDER_TIMEOUT: ended in-band
            //      at the render side's next item boundary (g_abort_soft; nothing dropped, so B's parser stays in step and pairs
            //      it with its FLUSH); if even that cannot get out (B takes nothing) for another RENDER_TIMEOUT, a credit re-sync
            //      (g_resync). A frame streaming when a SYNC goes out (A's start, B restarted, that re-sync) dies with it: the render
            //      side and the link FIFO drop it (g_abort), B empties its render buffer and drops every FLUSH older than the SYNC.
            //      Each time the FLUSH gets EV_ERROR ERR_RENDER, emitted at the next command boundary (D_HDR). A link drop alone gives
            //      nothing up: the link layer holds the data across a retrain, and an abort for it (a SYNC) could miss B's
            //      FLUSH and hang B.
            if (g_render) begin r_active <= 1'b1; r_aborted <= 1'b0; r_hard <= 1'b0; rt <= 0; r_seq <= mseq; r_idx <= sub_idx; end
            else if (r_active) begin
                if (g_render_done) begin r_active <= 1'b0; if (r_aborted) begin rerr_pend <= 1'b1; rerr_rt <= rt; end end
                else if (sync_go) begin g_abort <= 1'b1; r_aborted <= 1'b1; r_hard <= 1'b1; end
                else if (!r_aborted && rt == RENDER_TIMEOUT) begin g_abort_soft <= 1'b1; r_aborted <= 1'b1; rt <= rt + 1; end
                else if (!r_hard && rt == 2 * RENDER_TIMEOUT) begin g_resync <= 1'b1; r_hard <= 1'b1; end
                else if (rt != 32'hFFFF_FFFF) rt <= rt + 1;
            end
            if (sync_go && st == D_GRWAIT) fl_dead <= 1'b1;
            if (st == D_HDR && !q_valid && !rerr_pend) c_hdr_wait <= c_hdr_wait + 1;   // nothing to do: the host has not sent the next command
            if (st == D_GWAIT) c_gwait <= c_gwait + 1;                                 // a geometry command waits for the stage (draw / load / uniforms)

            case (st)
                D_HDR: if (rerr_pend) begin                 // an aborted frame's error, under its FLUSH's seq / sub index
                    rerr_pend <= 1'b0; mseq <= r_seq; mtype <= T_FLUSH; sub_idx <= r_idx; bleft <= 0; wleft <= 0; bvalid <= 1'b0;
                    raise_err(ERR_RENDER, rerr_rt);
                    e_a1 <= {24'h0, T_FLUSH};               // raise_err reads the previous command's type
                end else if (q_valid) begin
                    mtype <= q_data[23:16]; blen <= q_data[15:0]; mseq <= q_data[63:32];
                    bleft <= (q_data[15:0] + 16'd31) >> 5; wleft <= (q_data[15:0] + 16'd3) >> 2;
                    bvalid <= 1'b0; fi <= 0; nf <= nfields(q_data[23:16], q_data[15:0]);
                    for (int k = 0; k < 13; k++) f[k] <= 0;
                    sub_idx <= 0; sub_n <= 0; in_batch <= 1'b0;
                    if (q_data[31:24] != Q_MARK) begin
                        // corrupted queue entry: report and skip this burst
                        e_is_err <= 1'b1; e_code <= EV_ERROR; e_a0 <= {24'h0, ERR_LEN}; e_a1 <= 32'hFFFF_FFFF;
                        e_a2 <= q_data[31:0]; bleft <= 0; st <= D_EVENT;
                    end else if (q_data[23:16] == T_BATCH) begin   // the body = sub-messages: the entry's words, none of a sub-body yet
                        in_batch <= 1'b1; eleft <= (q_data[15:0] + 16'd3) >> 2; wleft <= 0;
                        st <= (q_data[15:0] < 16'd4) ? D_END : D_SUBHDR;
                    end else st <= D_FIELDS;
                end
                D_SUBHDR: if (take_sub) begin               // {u8 type, u8 flags, u16 body bytes}: the next sub-message
                    mtype <= wcur[7:0]; blen <= wcur[31:16]; sub_idx <= sub_n; sub_n <= sub_n + 1'b1;
                    fi <= 0; nf <= nfields(wcur[7:0], wcur[31:16]);
                    for (int k = 0; k < 13; k++) f[k] <= 0;
                    if (sub_words > {1'b0, eleft - 16'd1}) begin   // claims more than the entry holds: ERR_LEN ends the batch
                        wleft <= 0; eleft <= 0; in_batch <= 1'b0;
                        e_is_err <= 1'b1; e_code <= EV_ERROR; e_a0 <= {24'h0, ERR_LEN}; e_a1 <= {24'h0, wcur[7:0]};
                        e_a2 <= {16'h0, wcur[31:16]}; st <= D_EVENT;
                    end else begin
                        wleft <= sub_words[15:0]; eleft <= eleft - 16'd1 - sub_words[15:0]; st <= D_FIELDS;
                    end
                end
                D_FIELDS: begin
                    if (take_field) begin f[fi] <= wcur; fi <= fi + 1'b1; end
                    if (fi == nf || (wleft == 0 && !take_field)) st <= D_EXEC;
                end
                D_EXEC: begin
                    if (fi < nf) raise_err(ERR_LEN, {16'h0, blen});
                    else case (mtype)
                        T_MEM_WRITE, T_MEM_READ, T_MEMTEST: begin
                            if (f[0][7:0] == 8'd1) forward();
                            else if (f[0][7:0] != 8'd0) raise_err(ERR_BOARD, f[0]);
                            else if (mtype == T_MEMTEST && (f[1][4:0] != 0 || f[2][4:0] != 0)) raise_err(ERR_LEN, f[2]);
                            else begin
                                x_addr <= f[1][29:0];
                                x_seed <= f[3]; x_flags <= f[4][1:0];
                                if (mtype == T_MEM_WRITE)    begin pend_op <= OP_WRITE;   x_len <= blen - 16'd8; end
                                else if (mtype == T_MEM_READ) begin pend_op <= OP_READ;   x_len <= f[2][29:0]; r_tag <= f[3]; end
                                else                          begin pend_op <= OP_MEMTEST; x_len <= f[2][29:0]; end
                                st <= D_XCMD;
                            end
                        end
                        T_CREATE_BUFFER: begin
                            if (f[0][15]) forward();
                            else if (f[0][14]) raise_err(ERR_HANDLE, f[0]);
                            else if (f[1][4:0] != 0 || f[1] < HEAP_LO || f[2] == 0 || {1'b0, f[1]} + {1'b0, f[2]} > {1'b0, HEAP_HI})
                                raise_err(ERR_HEAP, f[1]);
                            else begin
                                bt_we <= 1'b1; bt_widx <= f[0][9:0]; bt_wv <= 1'b1; bt_wu <= f[0][19:16];
                                bt_wb <= f[1][29:0]; bt_ws <= f[2][29:0];
                                c_cmds <= c_cmds + 1; st <= D_END;
                            end
                        end
                        T_DESTROY: begin
                            if (f[0][15]) forward();
                            else if (f[0][14]) raise_err(ERR_HANDLE, f[0]);
                            else begin bt_we <= 1'b1; bt_widx <= f[0][9:0]; bt_wv <= 1'b0; c_cmds <= c_cmds + 1; st <= D_END; end
                        end
                        T_WRITE_BUFFER, T_READ_BUFFER: begin
                            if (f[0][15]) forward();
                            else if (f[0][14]) raise_err(ERR_HANDLE, f[0]);
                            else begin bt_ridx <= f[0][9:0]; st <= D_LOOK1; end
                        end
                        T_FENCE: st <= D_FENCE;
                        T_SET_MODE, T_PRESENT, T_COPY_BUFFER, T_CURSOR, T_CURSOR_POS, T_SET_PALETTE, T_FLUSH, T_DRAW_STATE: forward();
                        // ---- the geometry stage (A-local 3D)
                        T_FRAME_BEGIN: begin g_op <= 3'd0; st <= D_GWAIT; end
                        T_VS_PROGRAM: begin
                            if (f[0][15] || f[0][14]) raise_err(ERR_HANDLE, f[0]);
                            else if (f[1][4:0] != 0 || f[2] == 0 || f[2] > 32'd4096) raise_err(ERR_RANGE, f[2]);
                            else begin bt_ridx <= f[0][9:0]; st <= D_LOOK1; end
                        end
                        T_VS_UNIFORMS: begin
                            if (f[0][15:0] + f[0][31:16] > 32'd256 || f[0][31:16] == 0 || blen != 16'd4 + (f[0][31:16] << 4)) raise_err(ERR_RANGE, f[0]);
                            else begin g_op <= 3'd2; st <= D_GWAIT; end
                        end
                        T_VERTEX_ATTR: begin                    // size 1..4, type 0..6 (attr_conv: BYTE .. FLOAT, FP24)
                            if (f[0][18:16] == 0 || f[0][18:16] > 4 || f[0][22:20] == 3'd7) raise_err(ERR_RANGE, f[0]);
                            else if (f[0][8] && (f[2][31:30] != 0 || f[3][31:30] != 0)) raise_err(ERR_RANGE, f[2]);   // byte stride / offset inside a 1 GB space
                            else if (f[0][8] && (f[1][15] || f[1][14])) raise_err(ERR_HANDLE, f[1]);
                            else if (f[0][8]) begin bt_ridx <= f[1][9:0]; st <= D_LOOK1; end
                            else begin g_op <= 3'd3; st <= D_GWAIT; end            // a disabled attribute: no buffer
                        end
                        T_VS_STATE: begin                       // the target: 1..2048 pixels each way (A's bins: 512 bands of 4 rows)
                            if (f[5][15:0] == 0 || f[5][15:0] > 16'd2048 || f[5][31:16] == 0 || f[5][31:16] > 16'd2048) raise_err(ERR_RANGE, f[5]);
                            else begin g_op <= 3'd4; st <= D_GWAIT; end
                        end
                        T_DRAW: begin                           // first / count: 32 bits (ES 2.0 sets no limit on a draw's size)
                            if (f[0][2:0] > 3'd6) raise_err(ERR_RANGE, f[0]);     // triangles, strip, fan, points, lines, line strip, line loop
                            else if (f[0][8] && (f[1][15] || f[1][14])) raise_err(ERR_HANDLE, f[1]);
                            else if (f[0][8]) begin bt_ridx <= f[1][9:0]; st <= D_LOOK1; end
                            else begin g_op <= 3'd5; st <= D_GWAIT; end
                        end
                        T_CREATE_IMAGE, T_WRITE_IMAGE, T_READ_IMAGE, T_FILL_IMAGE, T_COPY_IMAGE, T_GEN_MIPMAP: begin
                            if (!f[0][15]) raise_err(ERR_HANDLE, f[0]);      // images exist on board B only
                            else forward();
                        end
                        default: raise_err(ERR_TYPE, 32'h0);
                    endcase
                end
                D_LOOK1: st <= D_LOOK1B;                     // table read latency
                D_LOOK1B: begin bl_v <= bt_rv; bl_b <= bt_rb; bl_s <= bt_rs; st <= D_LOOK2; end   // the read registered (bl_*)
                D_LOOK2: begin
                    if (!bl_v) raise_err(ERR_HANDLE, (mtype == T_VERTEX_ATTR || mtype == T_DRAW) ? f[1] : f[0]);
                    else if (mtype == T_VS_PROGRAM) begin   // offset + 8 bytes per instruction inside the buffer
                        if ({1'b0, f[1]} + {f[2][28:0], 3'd0} > {2'b00, bl_s}) raise_err(ERR_RANGE, f[1]);
                        else begin g_base <= bl_b; g_op <= 3'd1; st <= D_GWAIT; end
                    end else if (mtype == T_VERTEX_ATTR || mtype == T_DRAW) begin
                        if (mtype == T_DRAW && {1'b0, f[2]} > {2'b00, bl_s}) raise_err(ERR_RANGE, f[2]);
                        else begin g_base <= bl_b; g_op <= (mtype == T_DRAW) ? 3'd5 : 3'd3; st <= D_GWAIT; end
                    end else if (mtype == T_WRITE_BUFFER) begin
                        if ({1'b0, f[1]} + {16'h0, blen - 16'd8} > {2'b00, bl_s}) raise_err(ERR_RANGE, f[1]);
                        else begin x_addr <= bl_b + f[1][29:0]; x_len <= blen - 16'd8; pend_op <= OP_WRITE; st <= D_XCMD; end
                    end else begin
                        if ({1'b0, f[1]} + {1'b0, f[2]} > {2'b00, bl_s}) raise_err(ERR_RANGE, f[1]);
                        else begin x_addr <= bl_b + f[1][29:0]; x_len <= f[2][29:0]; r_tag <= f[3]; pend_op <= OP_READ; st <= D_XCMD; end
                    end
                end
                // a write into A's memory (MEM_WRITE / WRITE_BUFFER / MEMTEST) waits for the geometry to drain: a draw in
                // flight (draws overlap) may still be fetching the buffer, and the feed's burst cache is invalidated only
                // at a draw's start (else: stale bursts of a rewritten buffer)
                D_XCMD: if (x_cmd_ready && !x_cmd_valid && (pend_op == OP_READ || !g_busy)) begin
                    x_cmd_valid <= 1'b1; x_op <= pend_op;
                    case (pend_op)
                        OP_WRITE:   st <= D_WDATA;
                        OP_READ:    begin r_total <= x_len; r_off <= 0; st <= D_RDATA; end
                        default:    st <= D_MTWAIT;
                    endcase
                end
                D_WDATA: if (wleft == 0 && !take_wd) begin c_cmds <= c_cmds + 1; st <= D_END; end
                D_RDATA: begin
                    if (!r_pend) begin
                        if (r_off >= r_total) begin c_cmds <= c_cmds + 1; st <= D_END; end
                        else begin
                            r_pend <= 1'b1; r_i <= 0;
                            r_n <= (r_total - r_off > RD_CHUNK) ? RD_CHUNK[15:0] : 16'(r_total - r_off);
                        end
                    end else begin
                        if (rd_valid && rd_ready) r_i <= r_i + 1'b1;
                        if (rd_done) begin r_pend <= 1'b0; r_off <= r_off + r_n; end
                    end
                end
                D_MTWAIT: if (x_mt_done) begin
                    mt_last_errors <= x_mt_errors;
                    e_is_err <= 1'b0; e_code <= EV_MEMTEST; e_a0 <= x_mt_errors; e_a1 <= {2'b00, x_mt_first_bad};
                    e_a2 <= {2'b00, x_len};
                    c_cmds <= c_cmds + 1; st <= D_EVENT;
                end
                // ---- geometry commands: wait for what each needs, then apply (FRAME / load wait for their completion); a frame
                //      begin also waits until the bank it takes is no longer being streamed (g_fb_ok)
                D_GWAIT: if (gw_ok) begin                                 // (gw_c, registered: see its declaration)
                    case (g_op)
                        3'd0: begin g_frame <= 1'b1; st <= D_GFRAME; end
                        3'd1: begin g_ld_start <= 1'b1; g_ld_addr <= 29'((g_base + f[1][29:0]) >> 2); g_ld_len <= f[2][11:0]; st <= D_GLOAD; end   // bytes -> words
                        3'd2: begin g_param_idx <= f[0][7:0]; uni_k <= 2'd0; st <= D_GUNI; end
                        3'd3: begin
                            g_ad_we <= 1'b1; g_ac_we <= 1'b1; g_ad_idx <= f[0][3:0]; g_ac_idx <= f[0][3:0]; g_ad_en <= f[0][8];
                            g_ad_size <= f[0][18:16]; g_ad_type <= f[0][22:20]; g_ad_norm <= f[0][24];
                            g_ad_base <= g_base; g_ad_stride <= f[2][29:0]; g_ad_off <= f[3][29:0];
                            g_ac_const <= {f[7][23:0], f[6][23:0], f[5][23:0], f[4][23:0]};
                            c_cmds <= c_cmds + 1; st <= D_END;
                        end
                        3'd4: begin
                            g_st_we <= 1'b1; g_st_vp <= {f[3][23:0], f[2][23:0], f[1][23:0], f[0][23:0]}; g_st_z <= {f[8][23:0], f[7][23:0]};
                            g_st_gb <= {f[12][23:0], f[11][23:0], f[10][23:0], f[9][23:0]};
                            g_st_cull <= f[4][1:0]; g_st_cw <= f[4][8]; g_st_z0 <= f[4][24]; g_st_lw <= f[4][15:9]; g_st_fsmask <= f[4][23:16];
                            g_st_w <= f[5][11:0]; g_st_h <= f[5][27:16]; g_st_did <= f[6][12:0];
                            g_st_sc <= (nf == 4'd15) ? {sc_y1c, sc_x1c, sc_y0c, sc_x0c} : {f[5][27:16], f[5][11:0], 12'd0, 12'd0};
                            c_cmds <= c_cmds + 1; st <= D_END;
                        end
                        default: begin
                            g_dr_start <= 1'b1; g_dr_mode <= f[0][2:0]; g_dr_indexed <= f[0][8]; g_dr_u8 <= f[0][16];
                            g_dr_idx_base <= g_base + f[2][29:0];
                            g_dr_first <= f[3]; g_dr_count <= f[4];
                            c_cmds <= c_cmds + 1; st <= D_END;
                        end
                    endcase
                end
                D_GFRAME: if (g_frame_done) begin c_cmds <= c_cmds + 1; st <= D_END; end
                D_GLOAD:  if (g_ld_done) begin c_cmds <= c_cmds + 1; st <= D_END; end
                // a forwarded FLUSH: B runs the frame from the render stream, which A's render side now produces from the
                // frame's bank once the geometry has drained and the previous stream has ended; the dispatcher goes on
                // (the next frame's geometry overlaps the stream), the monitor above watches the stream
                // a FLUSH forwarded before a SYNC has no frame on B any more: it is not streamed (ERR_RENDER, detail 0)
                // (an error of A's, not of the command: a batch carries on)
                D_GRWAIT: if (fl_dead || sync_go) begin raise_err(ERR_RENDER, 32'h0); in_batch <= in_batch; end
                          else if (!g_busy && !g_rbusy) begin g_render <= 1'b1; st <= D_END; end
                D_GUNI: begin                                   // 4 body words per uniform -> one param write
                    if (take_uni) begin
                        g_param_data[uni_k*24 +: 24] <= wcur[23:0]; uni_k <= uni_k + 1'b1;
                        if (uni_k == 2'd3) g_param_we <= 1'b1;
                    end
                    if (g_param_we) g_param_idx <= g_param_idx + 1'b1;
                    if (wleft == 0 && !take_uni) begin c_cmds <= c_cmds + 1; st <= D_END; end
                end
                D_FENCE: if (!x_busy && !x_cmd_valid && !g_busy && !g_rbusy) begin   // A's whole work, the render stream included
                    if (link_ok) begin fw_i <= 0; fw_len <= 16'd2 + {12'h0, fi} + wleft; st <= D_FWD; end
                    else if (!b_gone) begin bw_t <= 0; st <= D_WAITB; end
                    else begin
                        e_is_err <= 1'b0; e_code <= EV_FENCE; e_a0 <= f[0]; e_a1 <= 0; e_a2 <= 0;
                        c_cmds <= c_cmds + 1; st <= D_EVENT;
                    end
                end
                D_FWD: if (f_valid && f_ready) begin
                    fw_i <= fw_i + 16'd1;
                    if (f_last) begin c_cmds <= c_cmds + 1; c_fwd <= c_fwd + 1; st <= (mtype == T_FLUSH) ? D_GRWAIT : D_END; fl_dead <= 1'b0; end
                end
                D_EVENT: begin
                    if (!ev_pend) begin
                        ev_pend <= 1'b1; ev_i <= 0;
                        ev_w[0] <= hdr_w0(R_EVENT); ev_w[1] <= mseq; ev_w[2] <= {16'h0, sub_idx, e_code};
                        ev_w[3] <= e_a0; ev_w[4] <= e_a1; ev_w[5] <= e_a2;
                        if (e_is_err) begin
                            c_errs <= c_errs + 1; last_err <= {e_a0[7:0], e_a1[7:0], 16'h0}; last_err_seq <= mseq;
                        end
                    end else begin
                        if (ev_valid && ev_ready) ev_i <= ev_i + 1'b1;
                        if (ev_done) begin ev_pend <= 1'b0; st <= D_END; end
                    end
                end
                D_END: if (in_batch) begin                   // a sub-message is done: drop what it did not read, then the next one
                    if (wleft == 0 && !take_skip) begin
                        if (eleft == 0) in_batch <= 1'b0;    // the batch's last: the entry ends as any other (below, next cycle)
                        else st <= D_SUBHDR;
                    end
                end else begin                               // discard the rest of the entry
                    bvalid <= 1'b0;
                    if (bleft == 0 && !load_body) st <= D_HDR;
                end
                D_WAITB: begin                              // the board link is down: wait for it (retrain) or give up
                    if (link_ok) begin fw_i <= 0; fw_len <= 16'd2 + {12'h0, fi} + wleft; st <= D_FWD; end
                    else if (bw_t == B_WAIT) begin
                        b_gone <= 1'b1;
                        if (mtype == T_FENCE) begin
                            e_is_err <= 1'b0; e_code <= EV_FENCE; e_a0 <= f[0]; e_a1 <= 0; e_a2 <= 0;
                            c_cmds <= c_cmds + 1; st <= D_EVENT;
                        end else raise_err(ERR_NO_B, 32'h0);
                    end else bw_t <= bw_t + 1;
                end
                default: st <= D_HDR;
            endcase
            if (link_ok) b_gone <= 1'b0;
        end
    end
endmodule
`default_nettype wire
