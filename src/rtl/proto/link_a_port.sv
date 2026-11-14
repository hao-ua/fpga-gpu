// link_a_port — board A's packet layer on the board link, clk_sys.
// TX (forward lane, 64-bit beats {hi, lo}, words in the order lo, hi):
//   * CMD packets from fb_dispatch_a: a word stream (first word with f_len = payload words, f_last on the
//     last); the link header {L_CMD, 0, len} goes in front. Credit: a packet starts only while the credit
//     is synchronised and if the words sent minus the words B reported as taken (CREDIT packets) plus its length
//     fit B's CMD_BUF_WORDS-word command buffer;
//   * SYNC {id, words sent} (credit sync), after A's reset and whenever B reports a restart: first a flush
//     (id 0), answered by a CREDIT with id 0 once every CREDIT B sent before it has arrived; then id k (1..127),
//     answered by the CREDIT with id k, whose count B has re-based on A's count at that point of the stream;
//   * CURSOR {y, x} (fast path): the latest position from cur_valid / cur_pos, sent at the next packet
//     boundary ahead of any CMD packet (no credit needed: B takes it at once);
//   * RSTREAM (the 3D render stream of a_render): beats queued in a block-RAM FIFO go out in
//     packets of up to RS_PKT beats (a packet starts when RS_PKT beats wait, or after RS_FLUSH_CYCLES without a new
//     beat) under the render credit: the beats sent minus the beats B reported as taken (RCREDIT) plus the packet
//     must fit B's RS_BUF_BEATS render buffer; the header beat {0, header} has len = 1 + 2 beats (as tb_board_b3d
//     drives B). The SYNC carries a third word {render beats sent} so B re-bases the render counters as the CMD ones
//     (and empties its render buffer: a SYNC drops the render FIFO here and every frame whose FLUSH went before it).
//     A SYNC goes out at A's start, when B restarted, and when the dispatcher gives a frame up for good (resync);
//   * IDLE keep-alive (header only) after IDLE_CYCLES without a transmitted beat.
// RX (backward lane, one word per beat):
//   * CREDIT {restarted, sync id} {words taken}: -> consumed when the id is the current SYNC's (others are from
//     before it); id 0 completes a flush, and outside one (or "restarted" anywhere) means B restarted -> sync again;
//   * RCREDIT {restarted, sync id} {render beats taken}: -> rs_consumed under the same id rule;
//   * STATUS: B_LSTAT_WORDS words, kept in b_stat; b_alive = a STATUS arrived within ALIVE_CYCLES;
//   * EVENT: {seq, code, a0, a1, a2} -> FIFO -> host EVENT datagram (fb_reply source protocol);
//   * READ_DATA: {seq, tag, offset, bytes, data...} -> host READ_DATA datagram {hdr, seq, tag, offset, data},
//     streamed from the link into its own fb_reply source once the four prefix words are in (the link is held
//     while the reply channel is busy);
//   * anything else is dropped and counted.
`default_nettype none
module link_a_port #(
    parameter int IDLE_CYCLES  = 100_000,      // 1 ms
    parameter int ALIVE_CYCLES = 1_000_000,    // 10 ms
    parameter int RS_PKT       = 64,           // render beats per RSTREAM packet at most
    parameter int RS_FLUSH_CYCLES = 32         // a shorter packet goes after this many cycles without a new beat
)(
    input  wire         clk, rst,
    // link_core user side (role A)
    output reg          tx_valid,
    output reg  [63:0]  tx_data,
    output reg          tx_cnt2,
    output reg          tx_last,
    input  wire         tx_ready,
    input  wire         rx_valid,
    input  wire [63:0]  rx_data,
    input  wire         rx_last, rx_hdr,
    output wire         rx_ready,
    // CMD word stream from the dispatcher
    input  wire         f_valid,
    input  wire [31:0]  f_data,
    input  wire [15:0]  f_len,
    input  wire         f_last,
    output wire         f_ready,
    // the render stream (a_render): 64-bit beats, in order
    input  wire         rs_valid,
    input  wire [63:0]  rs_data,
    output wire         rs_ready,
    input  wire         rs_abort,                // the frame is given up: drop the queued beats, pad the packet in flight
                                                 // with NOP items
    input  wire         resync,                  // re-sync the credit now (a SYNC; the dispatcher's last resort for a frame B
                                                 // cannot finish: B empties its render buffer and drops the older FLUSHes)
    output reg          sync_go,                 // one cycle: a SYNC goes out (a credit re-sync: A's start, B restarted);
                                                 // the render FIFO is dropped, every FLUSH forwarded before it is lost
    // cursor fast path (fb_proto_a): the latest position wins
    input  wire         cur_valid,
    input  wire [31:0]  cur_pos,                // {y, x}
    // EVENT reply source (fb_reply)
    output wire         ev_req,
    output wire [15:0]  ev_len,
    output wire         ev_valid,
    output wire [31:0]  ev_data,
    input  wire         ev_ready, ev_done,
    // READ_DATA reply source (fb_reply)
    output wire         rd_req,
    output wire [15:0]  rd_len,
    output wire         rd_valid,
    output wire [31:0]  rd_data,
    input  wire         rd_ready, rd_done,
    // board B
    output reg          b_alive,
    output reg  [fb_pkg::B_LSTAT_WORDS*32-1:0] b_stat,
    output reg  [31:0]  c_bstat, c_bev, c_unk, c_brd,
    output reg  [31:0]  sent, consumed,         // credit: CMD words sent / reported taken by B
    output reg  [31:0]  c_cur_sent, c_credit_wait, c_sync,
    output reg  [31:0]  rs_sent, rs_consumed,   // render credit: beats sent / reported taken by B
    output reg  [31:0]  c_rs_pkts,
    output reg  [31:0]  c_rs_wait               // cycles the render FIFO held beats without credit room
);
    import fb_pkg::*;

    // ---------------- TX packer ----------------
    typedef enum logic [2:0] {T_IDLE, T_BODY, T_KEEP, T_SYNC, T_RS} tst_t;
    tst_t       ts;
    reg  [31:0] idle_t;
    reg         hold_v;                     // a lo word waiting for its hi partner
    reg  [31:0] hold_w;
    reg  [31:0] sync_w, sync_rs;            // SYNC w1 / w2: the CMD words / render beats sent before it
    reg         cur_pend;
    reg  [31:0] cur_w;
    wire        out_free = !tx_valid || tx_ready;
    // the render stream FIFO and its packets
    // the render FIFO is popped combinationally as a beat goes out (one beat per cycle); an abort resets
    // it and pads the packet in flight with NOP items (zero beats) so B's parser keeps its framing
    wire        rsf_full, rsf_v; wire [63:0] rsf_d; wire [8:0] rsf_cnt;
    reg         rs_pad;                         // aborted mid-packet: the remaining beats go out as NOPs
    wire        rs_take = (ts == T_RS) && out_free && (rsf_v || rs_pad);
    wire        sync_start;                     // (defined with the packer)
    sfifo #(.W(64), .AW(7), .RAM_STYLE("block")) u_rsf (.clk(clk), .rst(rst | rs_abort | sync_start),
        .wen(rs_valid && !rsf_full && !rs_abort && !sync_start), .wdata(rs_data),
        .full(rsf_full), .count(rsf_cnt), .ren(rs_take && rsf_v), .rdata(rsf_d), .rvalid(rsf_v));
    assign rs_ready = !rsf_full;
    reg  [7:0]  rs_idle_t;                      // cycles since the last beat went into the FIFO
    reg  [11:0] rs_room;                        // RS_BUF_BEATS - (rs_sent - rs_consumed), registered like room
    reg  [6:0]  rs_n;                           // beats left in the packet being sent
    wire [8:0]  rs_avail = rsf_cnt;             // (a pop is seen by the count the next cycle; T_IDLE never pops)
    wire [6:0]  rs_fit   = (rs_room > RS_PKT) ? 7'(RS_PKT) : 7'(rs_room);
    wire [6:0]  rs_take_n = (rs_avail > {2'b00, rs_fit}) ? rs_fit : 7'(rs_avail);
    wire        rs_want  = (rs_avail != 0) && (rs_avail >= RS_PKT || rs_idle_t >= RS_FLUSH_CYCLES) && (rs_room != 0);   // never a packet of 0 beats
    // credit sync
    typedef enum logic [2:0] {Y_FLUSH, Y_FLUSH_W, Y_SYNC, Y_SYNC_W, Y_OK} yst_t;
    yst_t       ys;
    reg  [6:0]  sync_id;                        // id of the last SYNC k sent (0: none since reset)
    wire [6:0]  next_id = (sync_id == 7'd127) ? 7'd1 : sync_id + 7'd1;
    reg         y_zero, y_restart, y_ack;       // receiver -> packer, one cycle: CREDIT id 0 / restarted / id k
    // credit, registered (timing): room / synced lag one cycle behind sent, consumed and the sync state. That is
    // safe: a CMD packet has at least two words, so after a start the packer spends a cycle in T_BODY before the
    // next start can use the room; a CREDIT with the current id only ever adds room; and the CREDIT answering a
    // SYNC (which may lower it) sets consumed two cycles before synced rises.
    reg  [12:0] room;                           // CMD_BUF_WORDS - (sent - consumed)
    reg         synced;
    always @(posedge clk) begin : credit_r
        reg [31:0] infl, rinfl;
        infl = sent - consumed; rinfl = rs_sent - rs_consumed;
        synced  <= !rst && ys == Y_OK;
        room    <= (infl > CMD_BUF_WORDS) ? 13'd0 : 13'(CMD_BUF_WORDS - infl);
        rs_room <= (rinfl > RS_BUF_BEATS) ? 12'd0 : 12'(RS_BUF_BEATS - rinfl);
    end
    wire        fits = synced && (f_len <= {3'h0, room});
    assign      sync_start = (ts == T_IDLE) && out_free && !cur_pend && (ys == Y_FLUSH || ys == Y_SYNC);
    // a packet starts (header + first word) when it fits B's buffer and no cursor update is waiting
    wire        start_ok = (ts == T_IDLE) && !cur_pend && fits;
    // a word is taken when the output register can take the beat it completes
    assign f_ready = out_free && (start_ok || (ts == T_BODY));

    always @(posedge clk) begin
        if (tx_valid && tx_ready) tx_valid <= 1'b0;
        if (rst) begin
            ts <= T_IDLE; tx_valid <= 1'b0; hold_v <= 1'b0; idle_t <= 0; cur_pend <= 1'b0; sent <= 0;
            ys <= Y_FLUSH; sync_id <= 7'd0; rs_pad <= 1'b0; sync_go <= 1'b0;
            c_cur_sent <= 0; c_credit_wait <= 0; c_sync <= 0; rs_sent <= 0; c_rs_pkts <= 0; rs_idle_t <= 0; rs_n <= 0;
            c_rs_wait <= 0;
        end else begin
            sync_go <= sync_start;
            if (rs_avail != 0 && (rs_room == 0 || !synced)) c_rs_wait <= c_rs_wait + 1;   // render beats wait for B's credit
            if (rs_abort && ts == T_RS) rs_pad <= 1'b1;             // finish the announced packet with NOP beats
            if (tx_valid && tx_ready) idle_t <= 0;
            else if (idle_t != 32'hFFFFFFFF) idle_t <= idle_t + 1;
            if (rs_valid && !rsf_full) rs_idle_t <= 0;
            else if (rs_idle_t != 8'hFF) rs_idle_t <= rs_idle_t + 1;
            case (ts)
                T_IDLE: if (out_free) begin
                    if (cur_pend) begin                         // cursor position: one beat, header + {y, x}
                        tx_valid <= 1'b1; tx_data <= {cur_w, L_CURSOR, 8'h00, 16'd1}; tx_cnt2 <= 1'b1; tx_last <= 1'b1;
                        cur_pend <= 1'b0; c_cur_sent <= c_cur_sent + 1;
                    end else if (ys == Y_FLUSH || ys == Y_SYNC) begin     // credit sync: header + {id}, then {render beats sent, sent}
                        tx_valid <= 1'b1; tx_data <= {25'h0, (ys == Y_SYNC) ? next_id : 7'd0, L_SYNC, 8'h00, 16'd3};
                        tx_cnt2 <= 1'b1; tx_last <= 1'b0; sync_w <= sent; sync_rs <= rs_sent; ts <= T_SYNC; c_sync <= c_sync + 1;
                        if (ys == Y_SYNC) begin sync_id <= next_id; ys <= Y_SYNC_W; end else ys <= Y_FLUSH_W;
                    end else if (f_valid && fits) begin         // header + first payload word
                        tx_valid <= 1'b1; tx_data <= {f_data, L_CMD, 8'h00, f_len}; tx_cnt2 <= 1'b1;
                        tx_last <= f_last; ts <= f_last ? T_IDLE : T_BODY;
                        sent <= sent + {16'h0, f_len};
                    end else if (synced && rs_want) begin       // a render packet: header beat {0, header}, then the beats
                        tx_valid <= 1'b1; tx_data <= {32'h0, L_RSTREAM, 8'h00, 16'd1 + 16'(rs_take_n) * 16'd2}; tx_cnt2 <= 1'b1;
                        tx_last <= 1'b0; rs_n <= rs_take_n; rs_sent <= rs_sent + 32'(rs_take_n); c_rs_pkts <= c_rs_pkts + 1;
                        ts <= T_RS;
                    end else if (f_valid) begin
                        c_credit_wait <= c_credit_wait + 1;     // cycles waiting for credit (or its sync)
                    end else if (idle_t >= IDLE_CYCLES) begin
                        tx_valid <= 1'b1; tx_data <= {32'h0, L_IDLE, 8'h00, 16'h0}; tx_cnt2 <= 1'b0; tx_last <= 1'b1;
                        idle_t <= 0;
                    end
                end
                T_BODY: if (out_free && f_valid) begin
                    if (!hold_v && !f_last) begin hold_v <= 1'b1; hold_w <= f_data; end
                    else if (!hold_v) begin                     // last word alone
                        tx_valid <= 1'b1; tx_data <= {32'h0, f_data}; tx_cnt2 <= 1'b0; tx_last <= 1'b1; ts <= T_IDLE;
                    end else begin
                        tx_valid <= 1'b1; tx_data <= {f_data, hold_w}; tx_cnt2 <= 1'b1; tx_last <= f_last;
                        hold_v <= 1'b0; if (f_last) ts <= T_IDLE;
                    end
                end
                T_SYNC: if (out_free) begin
                    tx_valid <= 1'b1; tx_data <= {sync_rs, sync_w}; tx_cnt2 <= 1'b1; tx_last <= 1'b1; ts <= T_IDLE;
                end
                T_RS: if (rs_take) begin                            // one beat per cycle (a NOP beat while padding after an abort)
                    tx_valid <= 1'b1; tx_data <= rsf_v ? rsf_d : 64'h0; tx_cnt2 <= 1'b1; tx_last <= (rs_n == 7'd1);
                    rs_n <= rs_n - 7'd1;
                    if (rs_n == 7'd1) begin ts <= T_IDLE; rs_pad <= 1'b0; end
                end
                default: ts <= T_IDLE;
            endcase
            if (cur_valid) begin cur_pend <= 1'b1; cur_w <= cur_pos; end     // newest position wins
            // credit sync events (after the packer: a restart wins over a SYNC sent in this cycle). A CREDIT with id 0
            // after the flush is one B sent before it took our SYNC k (it was still emptying its buffer): ignored
            if (y_restart && ys != Y_FLUSH && ys != Y_FLUSH_W) ys <= Y_FLUSH;
            else if (resync && ys == Y_OK) ys <= Y_SYNC;          // the dispatcher gives a frame up for good
            else case (ys)
                Y_FLUSH_W: if (y_zero) ys <= Y_SYNC;
                Y_SYNC_W:  if (y_ack)  ys <= Y_OK;
                Y_OK:      if (y_zero) ys <= Y_FLUSH;  // B's id is 0 again: it restarted
                default: ;
            endcase
        end
    end

    // ---------------- RX parser ----------------
    typedef enum logic [3:0] {R_HDR, R_STAT, R_EV, R_SKIP, R_RDH, R_RDS, R_RDP, R_CRED, R_RCRED} rst_t;
    rst_t       rs;
    reg  [4:0]  ri;                             // payload word index (STATUS: B_LSTAT_WORDS <= 32)
    reg  [31:0] alive_t;
    reg  [7:0]  cr_h;                           // CREDIT header bits [23:16]: {restarted, sync id}
    reg  [159:0] ev_acc;
    // event FIFO: 4 entries of 5 words
    reg  [159:0] evq [0:3];
    reg  [2:0]  ev_w, ev_r;
    wire        evq_full  = (ev_w - ev_r) == 3'd4;
    wire        evq_empty = (ev_w == ev_r);
    // READ_DATA relay: prefix {seq, tag, offset, bytes}, then the datagram {hdr, seq, tag, offset, data...}.
    // rd_need = data words the datagram still needs from the link. A packet that ends early is padded with zero
    // words (R_RDP), the rest of a longer one is skipped, so a malformed packet can never stall the reply
    // writer. The next prefix is only taken once the previous datagram is out (rd_w is still being sent).
    reg  [31:0] rd_w [0:3];
    reg         rd_pend;
    reg  [15:0] rd_oi;                          // datagram word index
    reg  [15:0] rd_need;
    wire [31:0] rw = rx_data[31:0];
    wire [15:0] need_of = (rw[15:0] + 16'd3) >> 2;
    assign rd_req   = rd_pend;
    assign rd_len   = 16'd16 + rd_w[3][15:0];
    assign rd_valid = rd_pend && (rd_oi < 16'd4 || (rs == R_RDS && rx_valid) || rs == R_RDP);
    assign rd_data  = (rd_oi == 16'd0) ? hdr_w0(R_READ_DATA) : (rd_oi < 16'd4) ? rd_w[rd_oi[1:0] - 2'd1] :
                      (rs == R_RDP) ? 32'h0 : rx_data[31:0];
    // headers, STATUS, CREDIT and skipped words are always taken; only EVENT words need room in the event FIFO
    assign rx_ready = (rs == R_RDS) ? (rd_pend && rd_oi >= 16'd4 && rd_need != 0 && rd_ready) :
                      (rs == R_RDH) ? !rd_pend :
                      (rs == R_RDP) ? 1'b0 :
                      (rs == R_EV)  ? !evq_full : 1'b1;
    wire        rx_take = rx_valid && rx_ready;

    // the STATUS words in a block of their own (timing at 120 MHz: under rx_take their clock enable would carry
    // rx_ready's whole mux — fb_reply's word-count compare of the read-data path included; in R_STAT rx_ready is 1,
    // so the condition is rx_valid alone)
    always @(posedge clk) begin
        if (rst) b_stat <= '0;
        else if (rx_valid && !rx_hdr && rs == R_STAT) b_stat[ri*32 +: 32] <= rw;
    end
    always @(posedge clk) begin
        if (rst) begin
            rs <= R_HDR; ev_w <= 0; b_alive <= 1'b0; alive_t <= 0; c_bstat <= 0; c_unk <= 0;
            rd_pend <= 1'b0; rd_oi <= 0; rd_need <= 0; c_brd <= 0; consumed <= 0; rs_consumed <= 0;
            y_zero <= 1'b0; y_restart <= 1'b0; y_ack <= 1'b0;
        end else begin
            y_zero <= 1'b0; y_restart <= 1'b0; y_ack <= 1'b0;
            if (alive_t >= ALIVE_CYCLES) b_alive <= 1'b0; else alive_t <= alive_t + 1;
            if (rd_valid && rd_ready) rd_oi <= rd_oi + 16'd1;
            if (rd_done) begin
                rd_pend <= 1'b0; c_brd <= c_brd + 1;
                if (rs == R_RDP) rs <= R_HDR;
                if (rs == R_RDS) rs <= R_SKIP;                  // link words beyond the byte count
            end
            if (rx_take) begin
                if (rx_hdr) begin
                    ri <= 0;
                    if (rx_last) rs <= R_HDR;                   // header-only packet (none from B yet)
                    else if (rw[31:24] == L_STATUS && rw[15:0] != 16'd0 && rw[15:0] <= 16'(B_LSTAT_WORDS)) rs <= R_STAT;   // a shorter packet (a board B with fewer STATUS words) fills the first words
                    else if (rw[31:24] == L_EVENT && rw[15:0] == 16'd5) rs <= R_EV;
                    else if (rw[31:24] == L_READ_DATA && rw[15:0] >= 16'd4) rs <= R_RDH;
                    else if (rw[31:24] == L_CREDIT && rw[15:0] == 16'd2) rs <= R_CRED;
                    else if (rw[31:24] == L_RCREDIT && rw[15:0] == 16'd2) rs <= R_RCRED;
                    else begin rs <= R_SKIP; c_unk <= c_unk + 1; end
                end else begin
                    ri <= ri + 5'd1;
                    case (rs)
                        R_STAT: ;                                   // (b_stat: the block above)
                        R_EV:   ev_acc[ri*32 +: 32] <= rw;
                        R_RDH:  begin
                            rd_w[ri[1:0]] <= rw;
                            if (ri == 5'd3) begin
                                rd_pend <= 1'b1; rd_oi <= 0; rd_need <= need_of;
                                rs <= rx_last ? ((need_of != 0) ? R_RDP : R_HDR) : R_RDS;
                            end else if (rx_last) rs <= R_HDR;     // shorter than its prefix: nothing to send
                        end
                        R_RDS:  begin
                            rd_need <= rd_need - 16'd1;
                            if (rx_last) rs <= (rd_need != 16'd1) ? R_RDP : R_HDR;
                        end
                        R_CRED: if (ri == 5'd0) begin                   // w0 {restarted, sync id}
                            cr_h <= rw[7:0];
                            if (rx_last) rs <= R_HDR;
                        end else begin                                  // w1 the count
                            // only the current SYNC's count is in our origin (its id differs from 0 and from every
                            // CREDIT before the flush)
                            if (cr_h[6:0] == 7'd0) y_zero <= 1'b1;
                            else if (cr_h[6:0] == sync_id && (ys == Y_SYNC_W || ys == Y_OK)) begin
                                consumed <= rw; y_ack <= 1'b1;
                            end
                            y_restart <= cr_h[7];
                            if (rx_last) rs <= R_HDR; else rs <= R_SKIP;
                        end
                        R_RCRED: if (ri == 5'd0) begin                  // render credit: w0 {restarted, sync id}, w1 beats taken
                            cr_h <= rw[7:0];
                            if (rx_last) rs <= R_HDR;
                        end else begin
                            if (cr_h[6:0] != 7'd0 && cr_h[6:0] == sync_id && (ys == Y_SYNC_W || ys == Y_OK)) rs_consumed <= rw;
                            if (rx_last) rs <= R_HDR; else rs <= R_SKIP;
                        end
                        default: if (rx_last) rs <= R_HDR;
                    endcase
                    if (rx_last) begin
                        if (rs == R_STAT) begin c_bstat <= c_bstat + 1; b_alive <= 1'b1; alive_t <= 0; end
                        if (rs == R_EV) begin
                            evq[ev_w[1:0]] <= {rw, ev_acc[127:0]};
                            ev_w <= ev_w + 3'd1;
                        end
                    end
                end
            end
        end
    end

    // ---------------- EVENT datagrams ----------------
    reg  [2:0]  oi;
    reg         o_pend;
    wire [159:0] eh = evq[ev_r[1:0]];
    assign ev_req   = o_pend;
    assign ev_len   = 16'd24;
    assign ev_valid = o_pend;
    assign ev_data  = (oi == 3'd0) ? hdr_w0(R_EVENT) : eh[(oi - 3'd1)*32 +: 32];
    always @(posedge clk) begin
        if (rst) begin o_pend <= 1'b0; ev_r <= 0; oi <= 0; c_bev <= 0; end
        else if (!o_pend) begin
            if (!evq_empty) begin o_pend <= 1'b1; oi <= 0; end
        end else begin
            if (ev_valid && ev_ready) oi <= oi + 3'd1;
            if (ev_done) begin o_pend <= 1'b0; ev_r <= ev_r + 3'd1; c_bev <= c_bev + 1; end
        end
    end
endmodule
`default_nettype wire
