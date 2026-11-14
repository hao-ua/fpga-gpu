// link_fwd_deframer — link-layer RX of the 64-bit forward lane (user clock domain).
//
// Pops beats {par_err[3:0], last, valid_hi, hi, lo} from the RX FIFO. A packet starts with its
// header in `lo`; the CRC word is the last valid word of a LAST beat (`hi` when valid_hi, else `lo`).
// Checks: parity flags, header never with LAST unless it is a len-0 packet whose CRC is in `hi`,
// LAST exactly on the CRC beat, CRC-32 over header+payload, sequence continuity. Forwards header
// and payload (CRC stripped): rx_valid = `lo` is header/payload, rx_cnt2 = `hi` is payload too,
// rx_last on the beat holding the final payload word (or the header of a len-0 packet).
// `enable` = our receiver is trained (see link_deframer); `err_drop` = lock lost by an error.
// Payload and CRC words arrive scrambled (link_fwd_framer): descrambled here with the keystream restarting at each
// header (lo of a header beat plain, its hi the keystream's first word).
`default_nettype none
module link_fwd_deframer #(
    parameter int LOCK_WORDS   = 64,          // clean beats needed for lock
    parameter int IDLE_TIMEOUT = 10_000_000
)(
    input  wire        clk, rst,
    input  wire        enable,
    input  wire        f_empty,
    input  wire [69:0] f_data,          // {par_err[3:0], last, valid_hi, hi, lo}
    output wire        f_ren,
    output reg         rx_valid,
    output reg  [63:0] rx_data,         // {hi, lo}
    output reg         rx_cnt2,
    output reg         rx_last,
    output reg         rx_hdr,
    input  wire        rx_ready,
    output reg         pkt_done, pkt_ok,
    output reg         pkt_abort,       // packet dropped because of a frame error (ARQ rolls back)
    output reg         bnd_seen,        // a LAST beat ended the unsynced state: pkt_hdr holds its lo word (boundary)
    output reg  [31:0] pkt_hdr,         // header word of the packet in progress / just completed
    output reg         lock,
    output reg         err_drop,
    output reg  [31:0] rx_words, rx_pkts, par_errs, crc_errs, seq_errs, frm_errs, lock_drops,
    output reg  [31:0] par_map          // {hi PAR1, hi PAR0, lo PAR1, lo PAR0}: saturating 8-bit error counts per parity half
);
    `include "link_funcs.svh"
    // One beat of input pipelining: the FIFO's distributed-RAM read plus the slot-role / CRC / error
    // logic would be 14 LUT levels; everything below works on the registered beat instead.
    reg         have = 0;
    reg  [69:0] beat = 0;
    wire        pop  = have & (rx_ready | ~enable);           // consume the registered beat
    wire        take = ~f_empty & (~have | pop);              // refill when empty or being consumed
    assign f_ren = take;
    always @(posedge clk) begin
        if (rst) have <= 0;
        else if (take) begin beat <= f_data; have <= 1; end
        else if (pop) have <= 0;
    end
    reg  [31:0] ks;                                          // keystream for the next payload / CRC word
    reg         in_pkt, synced, seq_init;
    reg         bnd_wait;                                    // the next LAST beat while unsynced is the boundary (see link_deframer)
    wire [31:0] lo_raw = beat[31:0], hi_raw = beat[63:32];
    wire        plain_lo = ~synced | ~in_pkt;                // header or boundary in lo
    wire [31:0] ks_hi = plain_lo ? LINK_SSEED : xorshift32(ks);
    wire [31:0] lo = plain_lo ? lo_raw : (lo_raw ^ ks);
    wire [31:0] hi = synced ? (hi_raw ^ ks_hi) : hi_raw;
    wire        w_vhi  = beat[64];
    wire        w_last = beat[65];
    wire [3:0]  w_perr = beat[69:66];

    reg [15:0] remaining;              // payload words still expected (after the words consumed so far)
    reg        rem_is0, rem_is1;       // remaining == 0 / == 1, registers: keep the 16-bit compares and the decrement
                                       // off the slot roles, the CRC select and the frame-error class (timing at
                                       // 120 MHz)
    reg [7:0]  exp_seq;
    reg [31:0] crc;
    reg [31:0] streak, idle_cnt;
    reg        good_pkt_since_err;

    // slot roles for this beat
    wire        act       = pop & synced;
    wire        hdr_lo    = ~in_pkt;                                   // lo is a header
    wire        lo_is_crc = in_pkt & rem_is0;
    wire        lo_is_pay = in_pkt & ~rem_is0;
    wire [15:0] rem_lo    = hdr_lo ? lo[15:0] : (lo_is_pay ? remaining - 16'd1 : remaining);
    wire        rem_lo_nz = hdr_lo ? (lo[15:0] != 16'd0) : ~rem_is1;  // rem_lo != 0 where it is read (lo a header or payload)
    wire        hi_is_pay = w_vhi & ~lo_is_crc & rem_lo_nz;
    wire        hi_is_crc = w_vhi & ~lo_is_crc & ~rem_lo_nz;
    wire [15:0] rem_hi    = hi_is_pay ? rem_lo - 16'd1 : rem_lo;
    wire        crc_here  = lo_is_crc | hi_is_crc;
    wire [31:0] crc_lo    = hdr_lo ? crc32_word(32'hFFFFFFFF, lo) : (lo_is_pay ? crc32_word(crc, lo) : crc);
    wire [31:0] crc_hi    = hi_is_pay ? crc32_word(crc_lo, hi) : crc_lo;
    // errors
    wire err_par   = act & (|w_perr[1:0] | (w_vhi & |w_perr[3:2]));
    // sequence continuity is the ARQ layer's business (retransmissions legitimately go back);
    // seq_errs only counts for information and does not affect lock
    wire ctl_hdr   = (lo[31:24] == 8'hFF) | (lo[31:24] == 8'hFE);     // ARQ control packet: seq byte is an ack
    wire err_seq   = act & hdr_lo & seq_init & ~ctl_hdr & (lo[23:16] != exp_seq);
    wire err_crc   = act & ((lo_is_crc & (lo != ~crc)) | (hi_is_crc & (hi != ~crc_lo)));
    wire err_early = act & w_last & ~crc_here;                       // LAST but payload still expected
    wire err_late  = act & crc_here & ~w_last;                       // CRC slot without LAST
    wire err_after = act & lo_is_crc & w_vhi;                        // a word after the CRC in the same beat
    wire err_frm   = err_early | err_late | err_after;
    wire any_err   = enable & (err_par | err_crc | err_frm);
    // the lock tracking runs one cycle behind the beat (timing: it takes the CRC compare -> any_err off the streak's
    // clear); lock / err_drop are status for the retrain monitor, a cycle later is the same
    reg  le_err, le_pop, le_good;
    always @(posedge clk) begin
        le_err <= any_err & ~rst; le_pop <= pop & enable & ~rst; le_good <= act & crc_here & enable & ~rst;
    end

    always @(posedge clk) begin
        if (rst) begin
            in_pkt <= 0; synced <= 0; remaining <= 0; rem_is0 <= 1; rem_is1 <= 0; exp_seq <= 0; seq_init <= 0; crc <= 32'hFFFFFFFF; ks <= LINK_SSEED;
            rx_valid <= 0; rx_data <= 0; rx_cnt2 <= 0; rx_last <= 0; rx_hdr <= 0; pkt_done <= 0; pkt_ok <= 0;
            pkt_abort <= 0; pkt_hdr <= 0; bnd_seen <= 0; bnd_wait <= 0;
            lock <= 0; err_drop <= 0; streak <= 0; idle_cnt <= 0; good_pkt_since_err <= 0;
            rx_words <= 0; rx_pkts <= 0; par_errs <= 0; crc_errs <= 0; seq_errs <= 0; frm_errs <= 0; lock_drops <= 0;
            par_map <= 0;
        end else begin
            rx_valid <= 0; rx_cnt2 <= 0; rx_last <= 0; rx_hdr <= 0; pkt_done <= 0; pkt_ok <= 0; err_drop <= 0;
            pkt_abort <= 0; bnd_seen <= 0;
            if (act) begin
                if (w_perr[0] && par_map[7:0]   != 8'hFF) par_map[7:0]   <= par_map[7:0]   + 1;
                if (w_perr[1] && par_map[15:8]  != 8'hFF) par_map[15:8]  <= par_map[15:8]  + 1;
                if (w_vhi && w_perr[2] && par_map[23:16] != 8'hFF) par_map[23:16] <= par_map[23:16] + 1;
                if (w_vhi && w_perr[3] && par_map[31:24] != 8'hFF) par_map[31:24] <= par_map[31:24] + 1;
            end
            if (!enable) begin
                in_pkt <= 0; synced <= 0; seq_init <= 0; idle_cnt <= 0; streak <= 0; good_pkt_since_err <= 0;
                lock <= 0; bnd_wait <= 1;
            end else begin
                if (pop && !synced) begin
                    idle_cnt <= 0;
                    if (w_last) begin                                   // boundary (or a resync) found: next beat starts a packet
                        synced <= 1; bnd_seen <= bnd_wait & ~(|w_perr[1:0]); bnd_wait <= 0; pkt_hdr <= lo;
                    end
                end else if (pop) begin
                    rx_words <= rx_words + 1 + w_vhi;
                    ks <= w_vhi ? xorshift32(ks_hi) : ks_hi;        // past this beat's words
                    idle_cnt <= 0;
                    if (err_par) par_errs <= par_errs + 1;
                    if (err_seq) seq_errs <= seq_errs + 1;
                    if (err_crc) crc_errs <= crc_errs + 1;
                    if (err_frm) frm_errs <= frm_errs + 1;
                    // user output
                    rx_data <= {hi, lo};
                    rx_valid <= hdr_lo | lo_is_pay;
                    rx_hdr   <= hdr_lo;
                    rx_cnt2  <= hi_is_pay;
                    rx_last  <= (hdr_lo & (lo[15:0] == 0)) | (rem_hi == 0 & (lo_is_pay | hi_is_pay));
                    if (hdr_lo) begin
                        pkt_hdr <= lo;
                        if (!ctl_hdr) begin exp_seq <= lo[23:16] + 8'd1; seq_init <= 1'b1; end
                    end
                    if (err_frm) begin
                        in_pkt <= 0; synced <= 0;            // resync at the next LAST
                        pkt_abort <= 1;
                    end else if (crc_here) begin
                        in_pkt <= 0; rx_pkts <= rx_pkts + 1; pkt_done <= 1; pkt_ok <= ~err_crc & ~err_par;
                        crc <= 32'hFFFFFFFF;
                    end else begin
                        in_pkt <= 1; remaining <= rem_hi; rem_is0 <= (rem_hi == 16'd0); rem_is1 <= (rem_hi == 16'd1); crc <= crc_hi;
                    end
                end else if (idle_cnt != IDLE_TIMEOUT) idle_cnt <= idle_cnt + 1;

                // lock tracking (one cycle behind the beat: le_*)
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
