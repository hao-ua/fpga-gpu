// disp_ctl_b — clk_sys side of board B's palette and hardware cursor; the pixel side is disp_post.
// Palette: SET_PALETTE entries go into a master copy (256 x 24). A publish (pal_pub) cancels a bank switch that has
// not happened yet (pal_req := pal_cur, race-free against frame_start because both registers live here), copies the
// whole master into the bank not on screen (256 cycles) and requests that bank (pal_req). scan_reader takes pal_req
// at every frame start (frame_start) and tags the frame's P8 pixels with it; pal_cur follows on the same edge.
// Cursor: the desired state S {on, mode, bank, hot x / y, x, y} is changed by the command processor (SHOW / HIDE /
// MOVE) and by the fast path (fp_*); whenever it changed (or video restarted after a mode change) and the mailbox is
// free (mb_ack == mb_req), S is copied into mb_state and mb_req toggles; disp_post applies it at its next vsync.
// Shape words go into bank ~S.bank, accepted (cs_ready) while the bank on screen (the bank of the last applied
// mailbox) and the bank of the state in flight both equal S.bank, i.e. no bank swap is waiting for its vsync (a SHAPE
// after a SHOW waits at most one frame). Position-only mailbox transfers do not block it: a cursor moved every
// frame over the fast path must not stall a shape upload.
`default_nettype none
module disp_ctl_b (
    input  wire         clk, rst,
    input  wire         video_on,
    input  wire         frame_start,              // scan_reader restart pulse
    // palette (command processor)
    input  wire         pm_we,
    input  wire [7:0]   pm_idx,
    input  wire [23:0]  pm_data,
    input  wire         pal_pub,                  // pulse after a SET_PALETTE's entries
    output wire         pal_busy,
    output reg          pal_req,                  // bank for the next frames (to scan_reader)
    output reg          pal_we,                   // display palette write port (disp_post)
    output reg  [8:0]   pal_wa,
    output reg  [23:0]  pal_wd,
    // cursor (command processor)
    input  wire         cur_cmd,                  // pulse
    input  wire [2:0]   cur_op,                   // 2 SHOW, 3 HIDE, 4 MOVE
    input  wire [1:0]   cur_mode,
    input  wire         cur_take, cur_setpos,
    input  wire [5:0]   cur_hx, cur_hy,
    input  wire [15:0]  cur_x, cur_y,
    input  wire         cs_wr,                    // shape word (held until cs_ready)
    input  wire [11:0]  cs_idx,
    input  wire [31:0]  cs_data,
    output wire         cs_ready,
    // fast path (link_b_port)
    input  wire         fp_valid,
    input  wire [15:0]  fp_x, fp_y,
    // to disp_post
    output reg          cs_we,
    output reg  [12:0]  cs_wa,
    output reg  [31:0]  cs_wd,
    output reg          mb_req,
    output reg  [47:0]  mb_state,
    input  wire         mb_ack,                   // pixel domain
    // statistics
    output reg  [31:0]  c_pal_pub, c_fast_moves, c_cur_cmds
);
    // ---------------- palette ----------------
    (* ram_style = "block" *) reg [23:0] master [0:255];
    reg  [23:0] m_q;
    reg  [7:0]  m_ra;
    always @(posedge clk) begin
        if (pm_we) master[pm_idx] <= pm_data;
        m_q <= master[m_ra];
    end
    typedef enum logic [1:0] {P_IDLE, P_CANCEL, P_COPY, P_SET} pst_t;
    pst_t       ps;
    reg         pal_cur, tgt;
    reg  [8:0]  ci;                               // copy index (one ahead of the write: read latency)
    assign pal_busy = (ps != P_IDLE) || pal_pub;
    always @(posedge clk) begin
        pal_we <= 1'b0;
        if (rst) begin ps <= P_IDLE; pal_req <= 1'b0; pal_cur <= 1'b0; c_pal_pub <= 0; end
        else begin
            if (frame_start) pal_cur <= pal_req;
            case (ps)
                P_IDLE: if (pal_pub) ps <= P_CANCEL;
                P_CANCEL: begin
                    // afterwards pal_req == pal_cur, so no frame start changes the bank during the copy
                    if (!frame_start) pal_req <= pal_cur;
                    ci <= 9'd0; m_ra <= 8'd0;
                    ps <= P_COPY;
                end
                P_COPY: begin
                    if (ci == 9'd0) tgt <= ~pal_cur;
                    m_ra <= ci[7:0] + 8'd1;
                    // m_q holds master[ci - 1]; tgt is valid from ci = 1 on
                    if (ci != 9'd0) begin pal_we <= 1'b1; pal_wa <= {tgt, ci[7:0] - 8'd1}; pal_wd <= m_q; end
                    ci <= ci + 9'd1;
                    if (ci == 9'd256) ps <= P_SET;
                end
                P_SET: begin pal_req <= tgt; c_pal_pub <= c_pal_pub + 1; ps <= P_IDLE; end
                default: ps <= P_IDLE;
            endcase
        end
    end

    // ---------------- cursor state and mailbox ----------------
    reg         s_en, s_bank; reg [1:0] s_mode; reg [5:0] s_hx, s_hy; reg [15:0] s_x, s_y;
    reg         dirty, sent_bank, shown_bank, von_q;
    (* ASYNC_REG = "TRUE" *) reg [1:0] ack_s;
    wire        mb_free = (ack_s[1] == mb_req);
    assign cs_ready = (sent_bank == s_bank) && (shown_bank == s_bank);
    always @(posedge clk) begin
        cs_we <= 1'b0;
        ack_s <= {ack_s[0], mb_ack};
        if (rst) begin
            s_en <= 1'b0; s_bank <= 1'b0; s_mode <= 2'd0; s_hx <= 0; s_hy <= 0; s_x <= 0; s_y <= 0;
            dirty <= 1'b1; sent_bank <= 1'b0; shown_bank <= 1'b0; mb_req <= 1'b0; mb_state <= '0; von_q <= 1'b0;
            c_fast_moves <= 0; c_cur_cmds <= 0;
        end else begin
            von_q <= video_on;
            if (mb_free) shown_bank <= sent_bank;                          // the state in flight has been applied
            if (mb_free && dirty) begin
                mb_state <= {s_en, s_mode, s_bank, s_hx, s_hy, s_x, s_y}; mb_req <= ~mb_req; sent_bank <= s_bank;
                dirty <= 1'b0;
            end
            if (video_on && !von_q) dirty <= 1'b1;                          // resend after a mode change
            if (cur_cmd) begin
                c_cur_cmds <= c_cur_cmds + 1; dirty <= 1'b1;
                case (cur_op)
                    3'd2: begin
                        s_en <= 1'b1; s_mode <= cur_mode; s_hx <= cur_hx; s_hy <= cur_hy;
                        if (cur_take) s_bank <= ~s_bank;
                        if (cur_setpos) begin s_x <= cur_x; s_y <= cur_y; end
                    end
                    3'd3: s_en <= 1'b0;
                    default: begin s_x <= cur_x; s_y <= cur_y; end
                endcase
            end
            if (fp_valid) begin s_x <= fp_x; s_y <= fp_y; dirty <= 1'b1; c_fast_moves <= c_fast_moves + 1; end
            if (cs_wr && cs_ready) begin cs_we <= 1'b1; cs_wa <= {~s_bank, cs_idx}; cs_wd <= cs_data; end
        end
    end
endmodule
`default_nettype wire
