// link_core — one board's end of the asymmetric board link:
//   forward lane A -> B: 64-bit beats on 40 wires (all of JM1 + JM2 bits [29:22]);
//   backward lane B -> A: 32-bit words on 22 wires (JM2 bits [21:0]).
// ROLE "A" transmits on the forward lane and receives the backward one; ROLE "B" the reverse.
// Both user interfaces are 2-word beats {hi, lo}; on the 32-bit lane only `lo` is used (cnt2 = 0).
//
// Stack per direction: user FIFO (clk -> lclk) -> link_arq_tx (retransmit ring) -> framer -> PHY
// ... PHY -> RX FIFO (lclk_rx -> clk) -> deframer -> link_arq_rx (commit/rollback) -> user.
// Acknowledgements travel as control packets on the opposite lane; the two link_rec_sync
// instances carry "received ACK/NAK" and "send ACK/NAK" from the clk-domain receiver to the
// lclk-domain transmitter.
//
// Clock domains:  clk      user domain (FIFO far sides, deframer, ARQ RX, trainer, status)
//                 lclk     TX lane launch clock (from the MMCM), lclk90/lclk270 its phase copies
//                 lclk_rx  recovered RX lane clock (internal, phase set by the trainer)
//
// UP wire semantics: our UP = our receiver is trained on the far lane. The far framer sends TRAIN
// beats while it sees our UP low, data (after a boundary beat) once it is high. The retrain monitor
// drops `trained` when the deframer cannot hold lock: lock lost by an error and not regained within
// RELOCK_CYCLES, or RETRAIN_DROPS error-induced lock losses within RETRAIN_WINDOW. The idle timeout
// alone never triggers a retrain. XOFF: our RX FIFO fill level -> the far framer pauses.
// The relock window counts words received (RELOCK_WORDS), so a single error on a lane that carries a packet
// per millisecond does not force a retrain; RELOCK_CYCLES is only the backstop for a lane that delivers nothing.
// fill_en: our framer sends fill words between packets; fill words received are dropped
// before the RX FIFO. diag: receive diagnostics (link_rx_diag) as plain register snapshots in clk.
`default_nettype none
module link_core #(
    parameter [7:0] ROLE      = "A",
    parameter int LINK_MHZ     = 50,
    parameter int MMCM_RST_CYCLES = 1_000_000,
    parameter int GUARD_CYCLES = 10_000_000,   // clk cycles of tri-state watch before enabling TX (100 ms @100 MHz)
    parameter int GUARD_TOGGLES = 100_000,     // LCLK-pin toggles within GUARD_CYCLES that mean "foreign driver" (a real
                                               // clock gives millions; crosstalk glitches on the floating pin gave > 8 at 150 MHz)
    parameter int IDLE_TIMEOUT = 10_000_000,
    parameter int LOCK_WORDS   = 64,
    parameter int XOFF_ON      = 384,
    parameter int XOFF_OFF     = 256,
    parameter int TRAIN_WORDS  = 1024,
    parameter int SETTLE_CYCLES= 64,
    parameter int RETRY_CYCLES = 100_000,
    parameter int RELOCK_CYCLES= 100_000,     // backstop: lock not regained within this many clk cycles -> retrain
    parameter int RELOCK_WORDS = 4096,        // lock not regained within this many received words -> retrain
    parameter int RETRAIN_WINDOW = 100_000_000,
    parameter int RETRAIN_DROPS  = 4,
    parameter int ARQ_TO_CYCLES  = 20000,      // lane cycles without ACK progress -> resend (link_arq_tx)
    parameter int ARQ_ACK_EVERY  = 4,
    parameter int ARQ_ACK_TO     = 200,
    parameter int ARQ_NAK_HOLD   = 2000,
    parameter bit SIM_SHIFTER  = 1'b0
)(
    input  wire        clk, rst,
    input  wire        lclk, lclk90, lclk270, rst_lclk,
    input  wire        fill_en,               // clk domain (static setting)
    // user TX beats (packet: header in lo of the first beat, tx_last on the final beat, cnt2 = hi valid)
    input  wire        tx_valid,
    input  wire [63:0] tx_data,
    input  wire        tx_cnt2,
    input  wire        tx_last,
    output wire        tx_ready,
    // user RX beats (only complete, CRC-checked, in-order packets)
    output wire        rx_valid,
    output wire [63:0] rx_data,
    output wire        rx_cnt2,
    output wire        rx_last,
    output wire        rx_hdr,
    input  wire        rx_ready,
    output wire        pkt_done, pkt_ok,      // deframer level (before ARQ)
    // status (clk domain)
    output wire        lock, trained, rx_clk_ok, far_up, training,
    output reg         guard_fail,
    output reg         tx_enabled,
    output reg  [31:0] xoff_events,
    output wire [7:0]  eye_width, phase, train_cnt,
    output wire [31:0] tx_pkts, rx_words, rx_pkts, par_errs, crc_errs, seq_errs, frm_errs, lock_drops,
    output wire [31:0] par_map,         // parity errors per half (see the deframers)
    output wire [31:0] retx,            // {timeouts, rewinds} of our transmitter
    output wire [31:0] disc,            // {NAKs requested, packets discarded} by our receiver
    output wire [8*32-1:0] diag,        // link_rx_diag {perrs, fill_perr, fills, last_word, last_err, hist, bursts_lg, bursts}
    // headers
    inout  wire [31:0] jm1,
    inout  wire [31:0] jm2
);
    // ------------------------------------------------------------------ shared wires
    `include "link_funcs.svh"
    wire        rx_xoff_raw, rx_up_raw, lclk_rx, rx_locked, trained_i, lclk_sense;
    wire        tx_training_l; wire [31:0] tx_pkts_l;
    wire        lock_i, err_drop, retrain, rst_rx;
    wire [9:0]  rxf_count;
    wire [15:0] rewinds_l, timeouts_l, disc_c, naks_c;
    // ARQ records: receiver (clk) -> transmitter (lclk)
    wire        a_ack_stb, a_ack_nak, a_ctl_stb, a_ctl_nak;     // clk side (from link_arq_rx)
    wire [7:0]  a_ack_seq, a_ctl_seq;
    wire        l_ack_stb, l_ctl_stb; wire [8:0] l_ack_rec, l_ctl_rec;   // lclk side
    link_rec_sync #(.W(9)) u_sync_ack (.sclk(clk), .srst(rst), .s_stb(a_ack_stb), .s_data({a_ack_nak, a_ack_seq}),
                                       .dclk(lclk), .drst(rst_lclk), .d_stb(l_ack_stb), .d_data(l_ack_rec));
    link_rec_sync #(.W(9)) u_sync_ctl (.sclk(clk), .srst(rst), .s_stb(a_ctl_stb), .s_data({a_ctl_nak, a_ctl_seq}),
                                       .dclk(lclk), .drst(rst_lclk), .d_stb(l_ctl_stb), .d_data(l_ctl_rec));

    // far-end XOFF / UP (lclk_rx domain) -> lclk, for the framer
    (* ASYNC_REG = "TRUE" *) reg [1:0] xoff_s = 0, up_ls = 0;
    always @(posedge lclk) begin xoff_s <= {xoff_s[0], rx_xoff_raw}; up_ls <= {up_ls[0], rx_up_raw}; end
    // our RX FIFO fill -> XOFF to the far end (lclk_rx -> lclk); our trained -> UP wire (clk -> lclk); oe
    reg xoff_r = 0;
    (* ASYNC_REG = "TRUE" *) reg [1:0] our_xoff_s = 0, trn_s = 0, oe_s = 0, fill_s = 0;
    always @(posedge lclk) begin
        our_xoff_s <= {our_xoff_s[0], xoff_r};
        trn_s <= {trn_s[0], trained_i};
        oe_s  <= {oe_s[0], tx_enabled};
        fill_s <= {fill_s[0], fill_en};
    end
    // reset for the recovered-clock domain: async assert (user reset or MMCM unlocked), sync release
    wire rst_rx_a = rst | ~rx_locked;
    (* ASYNC_REG = "TRUE" *) reg [1:0] rst_rx_s = 2'b11;
    always @(posedge lclk_rx or posedge rst_rx_a)
        if (rst_rx_a) rst_rx_s <= 2'b11; else rst_rx_s <= {rst_rx_s[0], 1'b0};
    assign rst_rx = rst_rx_s[1];
    // words enter the RX FIFO only once trained (TRAIN words would otherwise fill it and raise XOFF)
    (* ASYNC_REG = "TRUE" *) reg [1:0] trn_rx_s = 0;
    always @(posedge lclk_rx) trn_rx_s <= rst_rx ? 2'b00 : {trn_rx_s[0], trained_i};
    // receive diagnostics and the fill-word filter (recovered-clock domain)
    wire        rx_pop;                                     // a word popped from the RX FIFO (clk)
    wire [31:0] dg [0:7];
    reg         hdr_pos;                                    // the next valid word stands at a header position
    // XOFF hysteresis in the RX FIFO write domain
    always @(posedge lclk_rx) begin
        if (rst_rx) xoff_r <= 0;
        else if (rxf_count >= XOFF_ON) xoff_r <= 1;
        else if (rxf_count <= XOFF_OFF) xoff_r <= 0;
    end

    generate
    if (ROLE == "A") begin : g_a
        // ---------------- TX: forward lane (64-bit beats) ----------------
        wire txf_full, txf_empty, txf_ren; wire [65:0] txf_rdata;
        assign tx_ready = ~txf_full;
        async_fifo #(.W(66), .AW(9)) u_txfifo (
            .wclk(clk),  .wrst(rst),      .wen(tx_valid & ~txf_full), .wdata({tx_last, tx_cnt2, tx_data}), .wfull(txf_full), .wcount(),
            .rclk(lclk), .rrst(rst_lclk), .ren(txf_ren), .rdata(txf_rdata), .rempty(txf_empty));
        wire arq_empty, arq_ren, arq_abort, bnd_fresh; wire [65:0] arq_rdata; wire [7:0] bnd_seq;
        link_arq_tx #(.W(66), .TO_CYCLES(ARQ_TO_CYCLES)) u_arq_tx (
            .clk(lclk), .rst(rst_lclk), .u_empty(txf_empty), .u_data(txf_rdata), .u_ren(txf_ren),
            .f_empty(arq_empty), .f_data(arq_rdata), .f_ren(arq_ren), .abort(arq_abort), .bnd_seq(bnd_seq), .bnd_fresh(bnd_fresh), .far_up(up_ls[1]), .our_up(trn_s[1]),
            .ack_stb(l_ack_stb), .ack_nak(l_ack_rec[8]), .ack_seq(l_ack_rec[7:0]),
            .ctl_stb(l_ctl_stb), .ctl_nak(l_ctl_rec[8]), .ctl_seq(l_ctl_rec[7:0]),
            .rewinds(rewinds_l), .timeouts(timeouts_l));
        wire [31:0] tlo, thi; wire tvlo, tvhi, tlast;
        link_fwd_framer u_framer (.clk(lclk), .rst(rst_lclk), .f_empty(arq_empty), .f_data(arq_rdata), .f_ren(arq_ren),
            .abort(arq_abort), .bnd_seq(bnd_seq), .bnd_fresh(bnd_fresh), .xoff(xoff_s[1]), .far_up(up_ls[1]), .fill(fill_s[1]), .lo(tlo), .hi(thi), .valid_lo(tvlo), .valid_hi(tvhi), .last(tlast),
            .tx_pkts(tx_pkts_l), .training(tx_training_l));
        link_fwd_phy_tx u_phy_tx (.lclk(lclk), .lclk90(lclk90), .lclk270(lclk270), .oe(oe_s[1]),
            .lo(tlo), .hi(thi), .valid_lo(tvlo), .valid_hi(tvhi), .last(tlast),
            .xoff(our_xoff_s[1]), .up(trn_s[1]), .lclk_sense(lclk_sense), .j1(jm1), .j2x(jm2[29:22]));
        // ---------------- RX: backward lane (32-bit words) ----------------
        wire [31:0] rword; wire rv, rl; wire [1:0] rperr;
        link_phy_rx #(.LINK_MHZ(LINK_MHZ), .MMCM_RST_CYCLES(MMCM_RST_CYCLES), .TRAIN_WORDS(TRAIN_WORDS),
                      .SETTLE_CYCLES(SETTLE_CYCLES), .RETRY_CYCLES(RETRY_CYCLES), .SIM_SHIFTER(SIM_SHIFTER)) u_phy_rx (
            .clk(clk), .rst(rst), .rst_rx(rst_rx), .retrain(retrain), .pins(jm2[21:0]), .lclk_rx(lclk_rx), .rx_locked(rx_locked),
            .word(rword), .valid(rv), .last(rl), .xoff(rx_xoff_raw), .up(rx_up_raw), .par_err(rperr),
            .trained(trained_i), .eye_width(eye_width), .phase(phase), .train_cnt(train_cnt));
        wire rxf_full, rxf_empty, rxf_ren; wire [34:0] rxf_rdata;
        wire is_fill = rv && rl && hdr_pos && rword[31:24] == LINK_T_FILL;
        always @(posedge lclk_rx) hdr_pos <= rst_rx ? 1'b1 : (rv ? rl : hdr_pos);
        link_rx_diag #(.PW(2)) u_diag (.lclk_rx(lclk_rx), .rst_rx(rst_rx), .en(trn_rx_s[1]), .valid(rv), .last(rl), .fill(is_fill),
            .word(rword), .perr(rperr), .bursts(dg[0]), .bursts_lg(dg[1]), .hist(dg[2]), .last_err(dg[3]), .last_word(dg[4]),
            .fills(dg[5]), .fill_perr(dg[6]), .perrs(dg[7]));
        assign rx_pop = rxf_ren;
        async_fifo #(.W(35), .AW(9)) u_rxfifo (
            .wclk(lclk_rx), .wrst(rst_rx), .wen(rv & ~rxf_full & trn_rx_s[1] & ~is_fill), .wdata({rperr, rl, rword}), .wfull(rxf_full), .wcount(rxf_count),
            .rclk(clk),     .rrst(rst),    .ren(rxf_ren), .rdata(rxf_rdata), .rempty(rxf_empty));
        wire [31:0] dfd; wire dfv, dfl, dfh, in_rdy, pabort, pbnd; wire [31:0] phdr;
        link_deframer #(.LOCK_WORDS(LOCK_WORDS), .IDLE_TIMEOUT(IDLE_TIMEOUT)) u_deframer (
            .clk(clk), .rst(rst), .enable(trained_i), .f_empty(rxf_empty), .f_data(rxf_rdata), .f_ren(rxf_ren),
            .rx_valid(dfv), .rx_data(dfd), .rx_last(dfl), .rx_hdr(dfh), .rx_ready(in_rdy),
            .pkt_done(pkt_done), .pkt_ok(pkt_ok), .pkt_abort(pabort), .bnd_seen(pbnd), .pkt_hdr(phdr), .lock(lock_i), .err_drop(err_drop),
            .rx_words(rx_words), .rx_pkts(rx_pkts), .par_errs(par_errs), .crc_errs(crc_errs),
            .seq_errs(seq_errs), .frm_errs(frm_errs), .lock_drops(lock_drops), .par_map(par_map));
        wire [31:0] od; wire oc2;
        link_arq_rx #(.DW(32), .ACK_EVERY(ARQ_ACK_EVERY), .ACK_TO(ARQ_ACK_TO), .NAK_HOLD(ARQ_NAK_HOLD)) u_arq_rx (
            .clk(clk), .rst(rst), .enable(trained_i),
            .in_valid(dfv), .in_data(dfd), .in_cnt2(1'b0), .in_last(dfl), .in_hdr(dfh),
            .pkt_done(pkt_done), .pkt_ok(pkt_ok), .pkt_abort(pabort), .bnd_seen(pbnd), .pkt_hdr(phdr), .in_ready(in_rdy),
            .out_valid(rx_valid), .out_data(od), .out_cnt2(oc2), .out_last(rx_last), .out_hdr(rx_hdr), .out_ready(rx_ready),
            .ack_stb(a_ack_stb), .ack_nak(a_ack_nak), .ack_seq(a_ack_seq),
            .ctl_stb(a_ctl_stb), .ctl_nak(a_ctl_nak), .ctl_seq(a_ctl_seq),
            .disc(disc_c), .naks(naks_c), .good_pkts());
        assign rx_data = {32'd0, od};
        assign rx_cnt2 = 1'b0;
    end else begin : g_b
        // ---------------- TX: backward lane (32-bit words, lo only) ----------------
        wire txf_full, txf_empty, txf_ren; wire [32:0] txf_rdata;
        assign tx_ready = ~txf_full;
        async_fifo #(.W(33), .AW(9)) u_txfifo (
            .wclk(clk),  .wrst(rst),      .wen(tx_valid & ~txf_full), .wdata({tx_last, tx_data[31:0]}), .wfull(txf_full), .wcount(),
            .rclk(lclk), .rrst(rst_lclk), .ren(txf_ren), .rdata(txf_rdata), .rempty(txf_empty));
        wire arq_empty, arq_ren, arq_abort, bnd_fresh; wire [32:0] arq_rdata; wire [7:0] bnd_seq;
        link_arq_tx #(.W(33), .TO_CYCLES(ARQ_TO_CYCLES)) u_arq_tx (
            .clk(lclk), .rst(rst_lclk), .u_empty(txf_empty), .u_data(txf_rdata), .u_ren(txf_ren),
            .f_empty(arq_empty), .f_data(arq_rdata), .f_ren(arq_ren), .abort(arq_abort), .bnd_seq(bnd_seq), .bnd_fresh(bnd_fresh), .far_up(up_ls[1]), .our_up(trn_s[1]),
            .ack_stb(l_ack_stb), .ack_nak(l_ack_rec[8]), .ack_seq(l_ack_rec[7:0]),
            .ctl_stb(l_ctl_stb), .ctl_nak(l_ctl_rec[8]), .ctl_seq(l_ctl_rec[7:0]),
            .rewinds(rewinds_l), .timeouts(timeouts_l));
        wire [31:0] tword; wire tv, tl;
        link_framer u_framer (.clk(lclk), .rst(rst_lclk), .f_empty(arq_empty), .f_data(arq_rdata), .f_ren(arq_ren),
            .abort(arq_abort), .bnd_seq(bnd_seq), .bnd_fresh(bnd_fresh), .xoff(xoff_s[1]), .far_up(up_ls[1]), .fill(fill_s[1]), .word(tword), .valid(tv), .last(tl), .tx_pkts(tx_pkts_l),
            .training(tx_training_l));
        link_phy_tx u_phy_tx (.lclk(lclk), .lclk90(lclk90), .lclk270(lclk270), .oe(oe_s[1]),
            .word(tword), .valid(tv), .last(tl), .xoff(our_xoff_s[1]), .up(trn_s[1]),
            .lclk_sense(lclk_sense), .pins(jm2[21:0]));
        // ---------------- RX: forward lane (64-bit beats) ----------------
        wire [31:0] rlo, rhi; wire rvlo, rvhi, rl; wire [3:0] rperr;
        link_fwd_phy_rx #(.LINK_MHZ(LINK_MHZ), .MMCM_RST_CYCLES(MMCM_RST_CYCLES), .TRAIN_WORDS(TRAIN_WORDS),
                          .SETTLE_CYCLES(SETTLE_CYCLES), .RETRY_CYCLES(RETRY_CYCLES), .SIM_SHIFTER(SIM_SHIFTER)) u_phy_rx (
            .clk(clk), .rst(rst), .rst_rx(rst_rx), .retrain(retrain), .j1(jm1), .j2x(jm2[29:22]),
            .lclk_rx(lclk_rx), .rx_locked(rx_locked),
            .lo(rlo), .hi(rhi), .valid_lo(rvlo), .valid_hi(rvhi), .last(rl), .xoff(rx_xoff_raw), .up(rx_up_raw), .par_err(rperr),
            .trained(trained_i), .eye_width(eye_width), .phase(phase), .train_cnt(train_cnt));
        wire rxf_full, rxf_empty, rxf_ren; wire [69:0] rxf_rdata;
        wire is_fill = rvlo && rl && hdr_pos && rlo[31:24] == LINK_T_FILL;
        always @(posedge lclk_rx) hdr_pos <= rst_rx ? 1'b1 : (rvlo ? rl : hdr_pos);
        link_rx_diag #(.PW(4)) u_diag (.lclk_rx(lclk_rx), .rst_rx(rst_rx), .en(trn_rx_s[1]), .valid(rvlo), .last(rl), .fill(is_fill),
            .word(rlo), .perr(rperr), .bursts(dg[0]), .bursts_lg(dg[1]), .hist(dg[2]), .last_err(dg[3]), .last_word(dg[4]),
            .fills(dg[5]), .fill_perr(dg[6]), .perrs(dg[7]));
        assign rx_pop = rxf_ren;
        async_fifo #(.W(70), .AW(9)) u_rxfifo (
            .wclk(lclk_rx), .wrst(rst_rx), .wen(rvlo & ~rxf_full & trn_rx_s[1] & ~is_fill), .wdata({rperr, rl, rvhi, rhi, rlo}), .wfull(rxf_full), .wcount(rxf_count),
            .rclk(clk),     .rrst(rst),    .ren(rxf_ren), .rdata(rxf_rdata), .rempty(rxf_empty));
        wire [63:0] dfd; wire dfv, dfc2, dfl, dfh, in_rdy, pabort, pbnd; wire [31:0] phdr;
        link_fwd_deframer #(.LOCK_WORDS(LOCK_WORDS), .IDLE_TIMEOUT(IDLE_TIMEOUT)) u_deframer (
            .clk(clk), .rst(rst), .enable(trained_i), .f_empty(rxf_empty), .f_data(rxf_rdata), .f_ren(rxf_ren),
            .rx_valid(dfv), .rx_data(dfd), .rx_cnt2(dfc2), .rx_last(dfl), .rx_hdr(dfh), .rx_ready(in_rdy),
            .pkt_done(pkt_done), .pkt_ok(pkt_ok), .pkt_abort(pabort), .bnd_seen(pbnd), .pkt_hdr(phdr), .lock(lock_i), .err_drop(err_drop),
            .rx_words(rx_words), .rx_pkts(rx_pkts), .par_errs(par_errs), .crc_errs(crc_errs),
            .seq_errs(seq_errs), .frm_errs(frm_errs), .lock_drops(lock_drops), .par_map(par_map));
        link_arq_rx #(.DW(64), .ACK_EVERY(ARQ_ACK_EVERY), .ACK_TO(ARQ_ACK_TO), .NAK_HOLD(ARQ_NAK_HOLD)) u_arq_rx (
            .clk(clk), .rst(rst), .enable(trained_i),
            .in_valid(dfv), .in_data(dfd), .in_cnt2(dfc2), .in_last(dfl), .in_hdr(dfh),
            .pkt_done(pkt_done), .pkt_ok(pkt_ok), .pkt_abort(pabort), .bnd_seen(pbnd), .pkt_hdr(phdr), .in_ready(in_rdy),
            .out_valid(rx_valid), .out_data(rx_data), .out_cnt2(rx_cnt2), .out_last(rx_last), .out_hdr(rx_hdr), .out_ready(rx_ready),
            .ack_stb(a_ack_stb), .ack_nak(a_ack_nak), .ack_seq(a_ack_seq),
            .ctl_stb(a_ctl_stb), .ctl_nak(a_ctl_nak), .ctl_seq(a_ctl_seq),
            .disc(disc_c), .naks(naks_c), .good_pkts());
    end
    endgenerate
    assign trained = trained_i;
    assign lock    = lock_i;
    assign disc    = {naks_c, disc_c};

    // ------------------------------------------------------------------ status syncs into clk
    (* ASYNC_REG = "TRUE" *) reg [1:0] rxl_s = 0, up_s = 0, xo_s = 0, trg_s = 0;
    reg xo_d = 0;
    always @(posedge clk) begin
        rxl_s <= {rxl_s[0], rx_locked};
        up_s  <= {up_s[0], rx_up_raw};
        xo_s  <= {xo_s[0], xoff_r};
        trg_s <= {trg_s[0], tx_training_l};
        xo_d  <= xo_s[1];
        if (rst) xoff_events <= 0; else if (xo_s[1] & ~xo_d) xoff_events <= xoff_events + 1;
    end
    assign rx_clk_ok = rxl_s[1];
    assign far_up    = up_s[1];
    assign training  = trg_s[1];
    // tx_pkts / retx cross lclk->clk through a plain register stage (false-pathed by name): they are
    // monotonically increasing debug counters read once a second; a transient mis-read is acceptable.
    (* ASYNC_REG = "TRUE" *) reg [31:0] txp_s = 0, retx_s = 0;
    always @(posedge clk) begin txp_s <= tx_pkts_l; retx_s <= {timeouts_l, rewinds_l}; end
    assign tx_pkts = txp_s;
    assign retx    = retx_s;
    (* ASYNC_REG = "TRUE" *) reg [31:0] dg_s [0:7];
    genvar di;
    generate for (di = 0; di < 8; di = di + 1) begin : g_dg
        always @(posedge clk) dg_s[di] <= dg[di];
        assign diag[di*32 +: 32] = dg_s[di];
    end endgenerate

    // ------------------------------------------------------------------ retrain monitor (clk)
    reg        relock_arm = 0, retrain_r = 0;
    reg [31:0] relock_t = 0, relock_w = 0, win_t = 0;
    reg [7:0]  drops_w = 0;
    always @(posedge clk) begin
        retrain_r <= 0;
        if (rst || !trained_i) begin
            relock_arm <= 0; relock_t <= 0; win_t <= 0; drops_w <= 0;
        end else begin
            if (err_drop) begin
                relock_arm <= 1; relock_t <= 0; relock_w <= 0;
                if (drops_w != 8'hFF) drops_w <= drops_w + 1;
            end else if (relock_arm) begin
                if (lock_i) relock_arm <= 0;
                else if (relock_w == RELOCK_WORDS || relock_t == RELOCK_CYCLES) begin retrain_r <= 1; relock_arm <= 0; end
                else begin relock_t <= relock_t + 1; if (rx_pop) relock_w <= relock_w + 1; end
            end
            if (win_t == RETRAIN_WINDOW) begin win_t <= 0; drops_w <= 0; end
            else win_t <= win_t + 1;
            if (drops_w >= RETRAIN_DROPS) begin retrain_r <= 1; drops_w <= 0; end
        end
    end
    assign retrain = retrain_r;

    // ------------------------------------------------------------------ contention guard
    // While our TX lane is tri-stated (first GUARD_CYCLES after reset) watch its LCLK pin. The board
    // pull-down keeps it low if nobody drives it; a toggling level means another driver is present.
    (* ASYNC_REG = "TRUE" *) reg [1:0] sense_s = 0;
    reg        sense_d = 0;
    reg [31:0] toggles = 0;
    reg [31:0] guard_cnt = 0;
    always @(posedge clk) begin
        sense_s <= {sense_s[0], lclk_sense};
        sense_d <= sense_s[1];
        if (rst) begin
            toggles <= 0; guard_cnt <= 0; guard_fail <= 0; tx_enabled <= 0;
        end else if (!tx_enabled && !guard_fail) begin
            if ((sense_s[1] ^ sense_d) && toggles != 32'hFFFFFFFF) toggles <= toggles + 1;
            if (guard_cnt == GUARD_CYCLES) begin
                if (toggles > GUARD_TOGGLES) guard_fail <= 1; else tx_enabled <= 1;
            end else guard_cnt <= guard_cnt + 1;
        end
    end
endmodule
`default_nettype wire
