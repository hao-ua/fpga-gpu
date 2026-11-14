// link_b_port — board B's packet layer on the board link, clk_sys.
// RX (forward lane, 64-bit beats {hi, lo}, words in the order lo, hi):
//   * CMD packets -> a CMD_BUF_WORDS-word command buffer (block RAM) -> word stream to the command processor
//     (c_first marks w0 = {0, type, bytes}, c_last the last word). Board A sends a CMD packet only when it fits
//     (credit), so the receiver never back-pressures the link because the command processor is busy (a held
//     PRESENT, a long copy); the buffer's full flag remains a safety net (then the link's XOFF takes over).
//   * CURSOR packets {y, x} (fast path) -> fp_valid / fp_x / fp_y at once, never queued behind commands;
//   * SYNC {id} {words A sent before it} (credit sync): every CMD word ahead of it in the stream is in the
//     buffer, so both counters are re-based on A's count (rcvd := it, consumed := it - occupancy), the sync id is
//     kept for the CREDIT header, and a CREDIT goes out at once (the answer A waits for);
//   * RSTREAM (3D render stream): payload beats -> the RS_BUF_BEATS-beat render buffer (block RAM)
//     -> b_render (rs_*); a full buffer back-pressures the link (A respects the render credit, so it does not);
//     a SYNC with a third word re-bases the render counters as its second word does the CMD ones; A sends one only
//     when the credit sync restarts (A's start, B restarted), never for a single aborted frame (that one
//     ends in-band, FRAME_END with its abort bit), so every FLUSH still in the command buffer then has lost its
//     frame: c_old marks those words for the command processor;
//   * IDLE and unknown packets are dropped (unknown ones counted).
// TX (backward lane, one 32-bit word per beat), packets never interleave; priority CREDIT, EVENT, STATUS when due,
// READ_DATA (so a long read cannot starve STATUS: board A declares B gone after 10 ms without one):
//   * CREDIT {restarted, sync id} {words taken by the command processor, cumulative} when CREDIT_EVERY words were
//     taken since the last report, CREDIT_DELAY cycles after any smaller change, after a SYNC, and once after
//     reset (restarted = 1 on that first one: board A then synchronises again);
//   * RCREDIT {restarted, sync id} {render beats taken by b_render} by the same rules, but only once a render stream
//     or a SYNC with render count has arrived since reset (board A without 3D never sees one);
//   * STATUS every STATUS_CYCLES: header {L_STATUS, 0, B_LSTAT_WORDS} + the words of `stat` (snapshot);
//   * EVENT {seq, code, a0, a1, a2} from the command processor (one-entry handshake: e_valid / e_ready);
//   * READ_DATA: while r_req, a packet {L_READ_DATA, 0, r_words} + r_words words pulled from r_valid / r_data
//     (the command processor supplies {seq, tag, offset, bytes, data...}; it drops r_req with the last word).
`default_nettype none
module link_b_port #(
    parameter int STATUS_CYCLES = 100_000,     // 1 ms
    parameter int CREDIT_EVERY  = 256,
    parameter int CREDIT_DELAY  = 200          // 2 us
)(
    input  wire         clk, rst,
    // link_core user side (role B)
    output reg          tx_valid,
    output reg  [63:0]  tx_data,
    output wire         tx_cnt2,
    output reg          tx_last,
    input  wire         tx_ready,
    input  wire         rx_valid,
    input  wire [63:0]  rx_data,
    input  wire         rx_cnt2, rx_last, rx_hdr,
    output wire         rx_ready,
    // CMD word stream to the command processor
    output wire         c_valid,
    output wire [31:0]  c_data,
    output wire         c_first, c_last,
    input  wire         c_ready,
    // EVENT from the command processor
    input  wire         e_valid,
    input  wire [159:0] e_data,                 // {a2, a1, a0, code, seq}
    output wire         e_ready,
    // READ_DATA from the command processor
    input  wire         r_req,
    input  wire [15:0]  r_words,
    input  wire         r_valid,
    input  wire [31:0]  r_data,
    output wire         r_ready,
    // render stream to b_render (first-word-fall-through): the head beat and the beat after it (b_render parses TRI
    // items two beats a cycle); rs_ready takes the head, rs_take2 both
    output wire         rs_valid, rs_valid2,
    output wire [63:0]  rs_data, rs_data2,
    input  wire         rs_ready, rs_take2,
    // cursor fast path
    output reg          fp_valid,
    output reg  [15:0]  fp_x, fp_y,
    // status snapshot source
    input  wire [fb_pkg::B_LSTAT_WORDS*32-1:0] stat,
    output reg  [31:0]  c_cmd_pkts, c_unk, c_cur_pkts,
    output reg  [31:0]  consumed,               // CMD words taken by the command processor (credit, A's origin)
    output reg  [31:0]  c_sync,                 // SYNC packets received
    output reg  [31:0]  c_rs_pkts,              // RSTREAM packets received
    output reg  [31:0]  rs_consumed,            // render beats taken by b_render (render credit, A's origin)
    output wire         rs_abort,               // one cycle: a SYNC with the render count arrived — A (re)started the credit
                                                // sync (its own start, or B restarted): the render buffer is emptied,
                                                // b_render leaves its frame
    output wire         c_old                   // the CMD word at the head (c_data) arrived before the last such SYNC: a FLUSH
                                                // there lost its frame with the wipe
);
    import fb_pkg::*;
    assign tx_cnt2 = 1'b0;

    // ---------------- RX: beats -> words -> command buffer ----------------
    // a beat is held until both of its words were taken; the header word is consumed here
    // hi_turn: the next word to take from the current beat is hi. A CMD header beat (lo = header) sets it
    // one cycle after the header is seen, so its first payload word comes from hi.
    reg         in_cmd, first_pend, hi_turn;
    reg  [31:0] rcvd;                           // CMD words written into the buffer (same origin as consumed)
    reg  [6:0]  sync_id;                        // id of the last SYNC (0: none since reset, or a flush)
    reg         sync_pend;                      // a SYNC header beat (w0 = id in hi) was taken, w1 comes next
    reg  [6:0]  sync_nid;
    reg         sync_ev;                        // a SYNC was taken (one cycle): send a CREDIT
    reg         in_rs;                          // inside an RSTREAM packet: payload beats go to the render buffer
    reg  [31:0] rs_rcvd;                        // render beats written into the buffer (same origin as rs_consumed)
    reg         rs_seen;                        // a render stream or a SYNC with render count arrived since reset
    reg         rs_sync_ev;                     // a SYNC re-based the render counters (one cycle): send an RCREDIT
    wire        rs_beat = rx_valid && !rx_hdr && in_rs;
    // The render buffer: 2048 beats in TWO interleaved banks (even / odd beats by arrival, 1024 each — four RAMB36, as
    // many as a single 2048-beat FIFO): the head bank's output is the oldest beat, the other bank's
    // the one after it, so b_render sees two beats at once and may take one or both. wpar / rpar: the bank the next beat is
    // written to / read from.
    wire        rsf_full0, rsf_full1; wire [11:0] rsf_cnt0, rsf_cnt1;
    wire        rsf_full = rsf_full0 | rsf_full1;
    wire [12:0] rsf_cnt  = 13'(rsf_cnt0) + 13'(rsf_cnt1);
    reg         wpar, rpar;
    wire        rs_take = rs_beat && !rsf_full;
    wire        rs_pop  = rs_valid && (rs_ready || rs_take2);           // the head beat leaves
    wire        rs_pop2 = rs_valid && rs_valid2 && rs_take2;            // the beat after it too
    // a SYNC carrying the render count empties the buffer: A sends one at link start (nothing buffered) and after an
    // aborted frame (the stale beats of the frame B never took must not reach the next frame's parser)
    wire        rs_sync = rx_valid && !rx_hdr && sync_pend && rx_cnt2;
    assign      rs_abort = rs_sync;
    wire        rsv0, rsv1; wire [63:0] rsd0, rsd1;
    sfifo #(.W(64), .AW(10), .RAM_STYLE("block")) u_rs0 (.clk(clk), .rst(rst | rs_sync), .wen(rs_take && !rs_sync && !wpar), .wdata(rx_data),
        .full(rsf_full0), .count(rsf_cnt0), .ren(!rs_sync && (rpar ? rs_pop2 : rs_pop)), .rdata(rsd0), .rvalid(rsv0));
    sfifo #(.W(64), .AW(10), .RAM_STYLE("block")) u_rs1 (.clk(clk), .rst(rst | rs_sync), .wen(rs_take && !rs_sync && wpar), .wdata(rx_data),
        .full(rsf_full1), .count(rsf_cnt1), .ren(!rs_sync && (rpar ? rs_pop : rs_pop2)), .rdata(rsd1), .rvalid(rsv1));
    assign rs_valid  = rpar ? rsv1 : rsv0;  assign rs_data  = rpar ? rsd1 : rsd0;
    assign rs_valid2 = rpar ? rsv0 : rsv1;  assign rs_data2 = rpar ? rsd0 : rsd1;
    always @(posedge clk) begin
        if (rst | rs_sync) begin wpar <= 1'b0; rpar <= 1'b0; end
        else begin
            if (rs_take) wpar <= ~wpar;
            if (rs_pop && !rs_pop2) rpar <= ~rpar;                      // two beats taken: the parity stays
        end
    end
    wire        in_cmd_now = rx_hdr ? (rx_data[31:24] == L_CMD) : in_cmd;
    wire        w_avail = rx_valid && in_cmd_now && (hi_turn ? rx_cnt2 : (!rx_hdr));
    wire        w_is_last = rx_last && (hi_turn ? 1'b1 : !rx_cnt2);
    wire        cb_full, cb_v; wire [33:0] cb_d; wire [13:0] cb_cnt;
    wire        w_take = w_avail && !cb_full;
    sfifo #(.W(34), .AW(12), .RAM_STYLE("block"), .OREG(1'b1)) u_cb (.clk(clk), .rst(rst), .wen(w_take),   // OREG: else the block RAM's latch feeds fb_cmd_b's decode directly (timing at 120 MHz)
        .wdata({first_pend, w_is_last, hi_turn ? rx_data[63:32] : rx_data[31:0]}), .full(cb_full), .count(cb_cnt),
        .ren(c_valid && c_ready), .rdata(cb_d), .rvalid(cb_v));
    assign c_valid = cb_v;
    assign c_data  = cb_d[31:0];
    assign c_first = cb_d[33];
    assign c_last  = cb_d[32];
    // the beat is released when its last word was taken, or at once if it carries nothing for the buffer
    wire        beat_done = rs_beat ? rs_take : rx_valid && (
                    (!in_cmd_now) ||                                         // IDLE / CURSOR / SYNC / unknown packet
                    (rx_hdr && !rx_cnt2) ||                                  // CMD header alone
                    (w_take && (hi_turn || !rx_cnt2)));
    assign rx_ready = beat_done;
    wire        c_take = c_valid && c_ready;
    wire        sync_now = rx_valid && !rx_hdr && sync_pend;    // SYNC w1 (lo)
    // the words in the command buffer at a wiping SYNC are older than it (a SYNC beat writes no CMD word)
    reg  [13:0] ps_left;
    assign      c_old = (ps_left != 14'd0);
    always @(posedge clk) begin
        if (rst) ps_left <= 14'd0;
        else if (rs_sync) ps_left <= cb_cnt - 14'(c_take);
        else if (c_take && ps_left != 14'd0) ps_left <= ps_left - 14'd1;
    end
    always @(posedge clk) begin
        fp_valid <= 1'b0; sync_ev <= 1'b0; rs_sync_ev <= 1'b0;
        if (rst) begin
            in_cmd <= 1'b0; first_pend <= 1'b0; hi_turn <= 1'b0; c_cmd_pkts <= 0; c_unk <= 0; c_cur_pkts <= 0;
            consumed <= 0; rcvd <= 0; sync_id <= 7'd0; sync_pend <= 1'b0; c_sync <= 0;
            in_rs <= 1'b0; rs_rcvd <= 0; rs_consumed <= 0; rs_seen <= 1'b0; c_rs_pkts <= 0;
        end else begin
            // a SYNC beat writes no CMD word, so rcvd counts every word ahead of it
            if (sync_now) begin
                rcvd <= rx_data[31:0];
                consumed <= rx_data[31:0] - rcvd + consumed + {31'h0, c_take};
            end else begin
                if (w_take) rcvd <= rcvd + 1;
                if (c_take) consumed <= consumed + 1;
            end
            // the same for the render stream when the SYNC has its third word — and the buffer is emptied, so every beat
            // A counts as sent counts as taken
            if (sync_now && rx_cnt2) begin
                rs_rcvd <= rx_data[63:32];
                rs_consumed <= rx_data[63:32];
                rs_seen <= 1'b1; rs_sync_ev <= 1'b1;
            end else begin
                if (rs_take) rs_rcvd <= rs_rcvd + 1;
                if (rs_pop) rs_consumed <= rs_consumed + 32'd1 + 32'(rs_pop2);
            end
            if (rs_take && rx_last) in_rs <= 1'b0;
            if (sync_now) begin sync_pend <= 1'b0; sync_id <= sync_nid; sync_ev <= 1'b1; c_sync <= c_sync + 1; end
            if (rx_valid && rx_hdr && !hi_turn) begin
                // header beat: lo = header; a CMD's first payload word may follow in hi
                sync_pend <= 1'b0;
                if (rx_data[31:24] == L_CMD) begin
                    in_cmd <= !rx_last; first_pend <= 1'b1; c_cmd_pkts <= c_cmd_pkts + 1;
                    hi_turn <= rx_cnt2;
                end else begin
                    in_cmd <= 1'b0;
                    if (rx_data[31:24] == L_RSTREAM) begin
                        in_rs <= !rx_last; c_rs_pkts <= c_rs_pkts + 1; rs_seen <= 1'b1;
                    end else if (rx_data[31:24] == L_CURSOR && rx_cnt2) begin
                        fp_valid <= 1'b1; fp_x <= rx_data[47:32]; fp_y <= rx_data[63:48]; c_cur_pkts <= c_cur_pkts + 1;
                    end else if (rx_data[31:24] == L_SYNC && rx_cnt2 && !rx_last) begin
                        sync_pend <= 1'b1; sync_nid <= rx_data[38:32];
                    end else if (rx_data[31:24] != L_IDLE) c_unk <= c_unk + 1;
                end
            end
            if (w_take) begin
                first_pend <= 1'b0;
                if (!hi_turn && rx_cnt2) hi_turn <= 1'b1; else hi_turn <= 1'b0;
                if (w_is_last) in_cmd <= 1'b0;
            end
        end
    end

    // ---------------- TX: CREDIT + EVENT + STATUS + READ_DATA packets ----------------
    typedef enum logic [2:0] {X_IDLE, X_STAT, X_EV, X_RD, X_CRED, X_RCRED} xst_t;
    xst_t       xs;
    reg  [31:0] st_t;
    reg  [4:0]  xi;                             // payload word index (STATUS: B_LSTAT_WORDS <= 32)
    reg  [15:0] ri;
    reg  [B_LSTAT_WORDS*32-1:0] snap;
    reg  [159:0] ev;
    reg          ev_full;
    reg  [31:0] reported, cr_val;
    reg  [15:0] cr_t;
    reg         cr_force;                       // after reset (restarted) and after a SYNC
    reg         restarted;                      // no CREDIT sent since reset
    reg  [7:0]  cr_w0;                          // {restarted, sync id} of the CREDIT being sent
    wire [31:0] cr_diff = consumed - reported;
    wire        cr_due_c = cr_force || ((cr_diff != 0) && (cr_diff >= CREDIT_EVERY || cr_t >= CREDIT_DELAY));
    // render credit (RCREDIT): the same rules, only once a render stream was seen
    reg  [31:0] rs_reported, rs_val; reg [15:0] rs_t; reg rs_force, rs_restarted; reg [7:0] rs_w0;
    wire [31:0] rs_diff = rs_consumed - rs_reported;
    wire        rs_due_c = rs_seen && (rs_force || ((rs_diff != 0) && (rs_diff >= CREDIT_EVERY || rs_t >= CREDIT_DELAY)));
    // the due decisions are REGISTERS (timing at 120 MHz: the 32-bit difference -> compare would sit in the clock enable
    // of the status snapshot and every other X_IDLE branch), trusted only after
    // a full X_IDLE cycle (idle_d: `reported` moves in the last cycle of a credit, so the first idle cycle's value is stale);
    // a credit may thus go out one cycle later
    reg         cr_due_r, rs_due_r, idle_d;
    always @(posedge clk) begin cr_due_r <= cr_due_c; rs_due_r <= rs_due_c; idle_d <= (xs == X_IDLE); end
    wire        cr_due = cr_due_r & idle_d, rs_due = rs_due_r & idle_d;
    assign e_ready = !ev_full;
    wire        out_free = !tx_valid || tx_ready;
    assign r_ready = (xs == X_RD) && out_free && ri != 0;
    always @(posedge clk) begin
        if (tx_valid && tx_ready) tx_valid <= 1'b0;
        if (rst) begin
            xs <= X_IDLE; tx_valid <= 1'b0; st_t <= 0; ev_full <= 1'b0; ri <= 0; reported <= 0; cr_t <= 0;
            cr_force <= 1'b1; restarted <= 1'b1; rs_reported <= 0; rs_t <= 0; rs_force <= 1'b1; rs_restarted <= 1'b1;
        end else begin
            if (e_valid && !ev_full) begin ev <= e_data; ev_full <= 1'b1; end
            if (st_t != 32'hFFFFFFFF) st_t <= st_t + 1;
            if (cr_diff == 0) cr_t <= 0; else if (cr_t != 16'hFFFF) cr_t <= cr_t + 1;
            if (rs_diff == 0) rs_t <= 0; else if (rs_t != 16'hFFFF) rs_t <= rs_t + 1;
            case (xs)
                X_IDLE: if (out_free) begin
                    if (cr_due) begin                   // the id and the count of the same cycle
                        tx_valid <= 1'b1; tx_data <= {32'h0, L_CREDIT, 8'h00, 16'd2}; tx_last <= 1'b0;
                        cr_val <= consumed; cr_w0 <= {restarted, sync_id}; xi <= 0; xs <= X_CRED;
                        cr_force <= 1'b0; restarted <= 1'b0;
                    end else if (rs_due) begin
                        tx_valid <= 1'b1; tx_data <= {32'h0, L_RCREDIT, 8'h00, 16'd2}; tx_last <= 1'b0;
                        rs_val <= rs_consumed; rs_w0 <= {rs_restarted, sync_id}; xi <= 0; xs <= X_RCRED;
                        rs_force <= 1'b0; rs_restarted <= 1'b0;
                    end else if (ev_full) begin
                        tx_valid <= 1'b1; tx_data <= {32'h0, L_EVENT, 8'h00, 16'd5}; tx_last <= 1'b0; xi <= 0; xs <= X_EV;
                    end else if (st_t >= STATUS_CYCLES) begin
                        tx_valid <= 1'b1; tx_data <= {32'h0, L_STATUS, 8'h00, 16'(B_LSTAT_WORDS)}; tx_last <= 1'b0;
                        snap <= stat; xi <= 0; st_t <= 0; xs <= X_STAT;
                    end else if (r_req && r_words != 0) begin
                        tx_valid <= 1'b1; tx_data <= {32'h0, L_READ_DATA, 8'h00, r_words}; tx_last <= 1'b0;
                        ri <= r_words; xs <= X_RD;
                    end
                end
                X_CRED: if (out_free) begin
                    if (xi == 5'd0) begin
                        tx_valid <= 1'b1; tx_data <= {32'h0, 24'h0, cr_w0}; tx_last <= 1'b0; xi <= 5'd1;
                    end else begin
                        tx_valid <= 1'b1; tx_data <= {32'h0, cr_val}; tx_last <= 1'b1;
                        reported <= cr_val; cr_t <= 0; xs <= X_IDLE;
                    end
                end
                X_RCRED: if (out_free) begin
                    if (xi == 5'd0) begin
                        tx_valid <= 1'b1; tx_data <= {32'h0, 24'h0, rs_w0}; tx_last <= 1'b0; xi <= 5'd1;
                    end else begin
                        tx_valid <= 1'b1; tx_data <= {32'h0, rs_val}; tx_last <= 1'b1;
                        rs_reported <= rs_val; rs_t <= 0; xs <= X_IDLE;
                    end
                end
                X_RD: if (r_valid && r_ready) begin
                    tx_valid <= 1'b1; tx_data <= {32'h0, r_data}; tx_last <= (ri == 16'd1);
                    ri <= ri - 16'd1;
                    if (ri == 16'd1) xs <= X_IDLE;
                end
                X_STAT: if (out_free) begin
                    tx_valid <= 1'b1; tx_data <= {32'h0, snap[xi*32 +: 32]};
                    tx_last <= (xi == 5'(B_LSTAT_WORDS - 1)); xi <= xi + 5'd1;
                    if (xi == 5'(B_LSTAT_WORDS - 1)) xs <= X_IDLE;
                end
                X_EV: if (out_free) begin
                    tx_valid <= 1'b1; tx_data <= {32'h0, ev[xi*32 +: 32]};
                    tx_last <= (xi == 5'd4); xi <= xi + 5'd1;
                    if (xi == 5'd4) begin xs <= X_IDLE; ev_full <= 1'b0; end
                end
                default: xs <= X_IDLE;
            endcase
            if (sync_ev) cr_force <= 1'b1;              // after the case: a SYNC taken now is answered next
            if (rs_sync_ev) rs_force <= 1'b1;
        end
    end
    wire _unused = &{1'b0, cb_cnt, rsf_cnt};
endmodule
`default_nettype wire
