// link_deframer — link-layer RX (user clock domain). Pops words from the RX FIFO, checks
// parity flags, header length, sequence continuity and the CRC-32 trailer, forwards
// header+payload to the user (CRC word stripped, rx_last on the final forwarded word) and
// keeps the error counters + the lock state.
//
// `enable` is the receiver's trained flag: while low the FIFO is drained and nothing is counted
// (the far end is sending TRAIN words); when it rises the deframer discards words until a LAST
// (the far framer's boundary word) and then expects a header. `err_drop` pulses when lock is lost
// because of an error (not because of the idle timeout) — the caller's retrain monitor uses it.
`default_nettype none
module link_deframer #(
    parameter int LOCK_WORDS   = 64,          // clean words needed for lock
    parameter int IDLE_TIMEOUT = 10_000_000   // clk cycles without a word -> lock lost (100 ms @100 MHz)
)(
    input  wire        clk, rst,
    input  wire        enable,
    // RX FIFO read port (FWFT)
    input  wire        f_empty,
    input  wire [34:0] f_data,          // {par_err[1:0], last, word}
    output wire        f_ren,
    // user stream out
    output reg         rx_valid,
    output reg  [31:0] rx_data,
    output reg         rx_last,
    output reg         rx_hdr,          // rx_data is a header word
    input  wire        rx_ready,
    output reg         pkt_done,        // one pulse per completed packet
    output reg         pkt_ok,          // qualifies pkt_done
    output reg         pkt_abort,       // packet dropped because of a frame error (ARQ rolls back)
    output reg         bnd_seen,        // a LAST word ended the unsynced state: pkt_hdr holds it (boundary word)
    output reg  [31:0] pkt_hdr,         // header word of the packet in progress / just completed
    // status
    output reg         lock,
    output reg         err_drop,
    output reg  [31:0] rx_words, rx_pkts, par_errs, crc_errs, seq_errs, frm_errs, lock_drops,
    output reg  [31:0] par_map          // {16'd0, high-half, low-half}: saturating 8-bit parity error counts
);
    `include "link_funcs.svh"
    reg        in_pkt;
    reg        synced;                  // 0 after reset/enable: discard words until a LAST word passes, then
                                        // the next word is a header (the far framer sends a boundary word
                                        // with LAST before its first packet)
    reg        bnd_wait;                // the next LAST word in the unsynced state is that boundary word: only the first
                                        // LAST after enable is reported as one (bnd_seen), never the CRC word a resync
                                        // after a frame error lands on, and only without a parity error (a damaged
                                        // boundary must not hand the ARQ receiver a sequence number)
    reg [31:0] ks;                      // descrambler keystream for the next payload / CRC word (link_framer)
    wire [31:0] w_raw   = f_data[31:0];
    wire [31:0] w       = (synced && in_pkt) ? (w_raw ^ ks) : w_raw;      // headers / boundary words are plain
    wire        w_last  = f_data[32];
    wire [1:0]  w_perr  = f_data[34:33];
    wire        pop     = ~f_empty & (rx_ready | ~enable);   // drain freely while disabled
    assign f_ren = pop;

    reg [15:0] remaining;
    reg        rem_nz;                  // remaining != 0, a register: keeps the 16-bit compare off the CRC register's
                                        // enable (timing at 120 MHz)
    reg [7:0]  exp_seq;
    reg        seq_init;
    reg [31:0] crc;
    reg [31:0] streak;
    reg [31:0] idle_cnt;
    reg        good_pkt_since_err;

    wire hdr_now  = pop & synced & ~in_pkt;
    wire perr_now = pop & synced & (|w_perr);
    wire ctl_hdr  = (w[31:24] == 8'hFF) | (w[31:24] == 8'hFE);         // ARQ control packet: seq byte is an ack
    // error classification for this popped word
    wire err_hdr_last = hdr_now & w_last;                               // header can never carry LAST
    wire err_seq      = hdr_now & seq_init & ~ctl_hdr & (w[23:16] != exp_seq);
    wire err_early    = pop & synced & in_pkt & w_last & rem_nz;        // CRC arrived before len words
    wire err_late     = pop & synced & in_pkt & ~w_last & ~rem_nz;      // expected the CRC word
    wire err_crc      = pop & synced & in_pkt & w_last & (w != ~crc);
    // sequence continuity is the ARQ layer's business (retransmissions legitimately go back):
    // seq_errs only counts for information and does not affect lock
    wire any_err      = enable & (perr_now | err_hdr_last | err_early | err_late | err_crc);
    // the lock tracking runs one cycle behind the word (timing: it takes the CRC compare -> any_err off the streak's
    // clear); lock / err_drop are status for the retrain monitor, a cycle later is the same
    reg        le_err, le_pop, le_good;
    always @(posedge clk) begin
        le_err <= any_err & ~rst; le_pop <= pop & enable & ~rst; le_good <= pop & enable & in_pkt & w_last & ~rst;
    end

    always @(posedge clk) begin
        if (rst) begin
            in_pkt <= 0; synced <= 0; remaining <= 0; rem_nz <= 0; exp_seq <= 0; seq_init <= 0; crc <= 32'hFFFFFFFF; ks <= LINK_SSEED;
            rx_valid <= 0; rx_data <= 0; rx_last <= 0; rx_hdr <= 0; pkt_done <= 0; pkt_ok <= 0;
            pkt_abort <= 0; pkt_hdr <= 0; bnd_seen <= 0; bnd_wait <= 0;
            lock <= 0; err_drop <= 0; streak <= 0; idle_cnt <= 0; good_pkt_since_err <= 0;
            rx_words <= 0; rx_pkts <= 0; par_errs <= 0; crc_errs <= 0; seq_errs <= 0; frm_errs <= 0; lock_drops <= 0;
            par_map <= 0;
        end else begin
            rx_valid <= 0; rx_last <= 0; rx_hdr <= 0; pkt_done <= 0; pkt_ok <= 0; err_drop <= 0;
            pkt_abort <= 0; bnd_seen <= 0;
            if (perr_now) begin
                if (w_perr[0] && par_map[7:0]  != 8'hFF) par_map[7:0]  <= par_map[7:0]  + 1;
                if (w_perr[1] && par_map[15:8] != 8'hFF) par_map[15:8] <= par_map[15:8] + 1;
            end
            if (!enable) begin
                // untrained: drain, count nothing, forget the stream state
                in_pkt <= 0; synced <= 0; seq_init <= 0; idle_cnt <= 0; streak <= 0; good_pkt_since_err <= 0;
                lock <= 0; bnd_wait <= 1;
            end else begin
                if (pop && !synced) begin
                    idle_cnt <= 0;
                    if (w_last) begin                                   // boundary (or a resync) found: next word is a header
                        synced <= 1; bnd_seen <= bnd_wait & ~(|w_perr); bnd_wait <= 0; pkt_hdr <= w;
                    end
                end else if (pop) begin
                    rx_words <= rx_words + 1;
                    ks <= in_pkt ? xorshift32(ks) : LINK_SSEED;
                    idle_cnt <= 0;
                    if (|w_perr) par_errs <= par_errs + 1;
                    if (!in_pkt) begin
                        // header
                        rx_valid <= 1; rx_data <= w; rx_hdr <= 1; rx_last <= (w[15:0] == 16'd0);
                        remaining <= w[15:0]; rem_nz <= (w[15:0] != 16'd0);
                        crc <= crc32_word(32'hFFFFFFFF, w);
                        if (!ctl_hdr) begin exp_seq <= w[23:16] + 8'd1; seq_init <= 1'b1; end
                        pkt_hdr <= w;
                        if (err_seq) seq_errs <= seq_errs + 1;
                        if (w_last) begin frm_errs <= frm_errs + 1; in_pkt <= 0; pkt_abort <= 1; end   // malformed: stay at header
                        else in_pkt <= 1;
                    end else if (!w_last) begin
                        // payload word (or a missing-CRC error)
                        if (rem_nz) begin
                            rx_valid <= 1; rx_data <= w; rx_last <= (remaining == 16'd1);
                            remaining <= remaining - 1; rem_nz <= (remaining != 16'd1);
                            crc <= crc32_word(crc, w);
                        end else begin
                            frm_errs <= frm_errs + 1;          // too long: resync at the next LAST
                            in_pkt <= 0; synced <= 0; pkt_abort <= 1;
                        end
                    end else begin
                        // CRC word
                        in_pkt <= 0;
                        rx_pkts <= rx_pkts + 1;
                        pkt_done <= 1;
                        pkt_ok <= ~err_crc & ~err_early & ~perr_now;
                        if (err_crc) crc_errs <= crc_errs + 1;
                        if (err_early) frm_errs <= frm_errs + 1;
                    end
                end else if (idle_cnt != IDLE_TIMEOUT) idle_cnt <= idle_cnt + 1;

                // lock tracking (one cycle behind the word: le_*)
                if (le_err) begin
                    streak <= 0; good_pkt_since_err <= 0;
                    if (lock) begin lock <= 0; lock_drops <= lock_drops + 1; err_drop <= 1; end
                end else begin
                    if (le_pop && streak != LOCK_WORDS) streak <= streak + 1;
                    if (le_good) good_pkt_since_err <= 1;
                    if (!lock && streak == LOCK_WORDS && good_pkt_since_err) lock <= 1;
                    if (lock && idle_cnt == IDLE_TIMEOUT) begin lock <= 0; lock_drops <= lock_drops + 1; streak <= 0; end
                end
            end
        end
    end
endmodule
`default_nettype wire
