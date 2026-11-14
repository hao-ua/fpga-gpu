// link_arq_rx — receive side of the go-back-N retransmission layer (user clock domain).
//
// Sits between the deframer and the user. Beats of a packet are written into a buffer (BRAM)
// tentatively; when the deframer reports the packet complete with a good CRC and the expected
// sequence number the write pointer is committed and the packet becomes readable, otherwise it is
// rolled back and the packet is discarded. Control packets (type 0xFF ACK / 0xFE NAK) are consumed
// here: the acknowledged sequence goes to the local link_arq_tx. The receiver asks its own
// transmitter to send an ACK every ACK_EVERY good packets or after ACK_TO cycles, and a NAK as
// soon as a packet is bad or out of sequence (rate-limited to one per NAK_HOLD cycles).
// Flow control: the deframer presents a popped word one cycle after it saw in_ready, so in_ready keeps a slack
// of IN_SLACK beats and every presented word is written (with writes gated by in_ready, a word popped as the buffer
// fills would be dropped after its CRC check and its packet committed short).
// Boundary word {0xFD, seq, 15'b0, fresh}: the sequence is adopted only when the far transmitter started a new
// numbering (fresh: reset, nothing acknowledged yet) or when this receiver has committed nothing since its own
// reset. After a retrain the transmitter rewinds to its oldest unacknowledged packet, which this receiver may
// already have delivered: it keeps expected, drops those duplicates and NAKs with expected, which also
// acknowledges them, so the transmitter moves on at once (adopting there would deliver duplicates).
`default_nettype none
module link_arq_rx #(
    parameter int DW        = 64,              // 64 (forward lane) or 32 (backward lane)
    parameter int BUF_AW    = 11,              // 2048 beats: two maximum packets
    parameter int ACK_EVERY = 4,
    parameter int ACK_TO    = 200,             // clk cycles
    parameter int NAK_HOLD  = 2000,            // clk cycles between NAKs for the same expected seq
    parameter int IN_SLACK  = 4                // free beats kept for words already popped by the deframer
)(
    input  wire          clk, rst,
    input  wire          enable,               // receiver trained
    // from the deframer
    input  wire          in_valid,
    input  wire [DW-1:0] in_data,
    input  wire          in_cnt2,
    input  wire          in_last,
    input  wire          in_hdr,
    input  wire          pkt_done, pkt_ok, pkt_abort,
    input  wire          bnd_seen,             // the deframer synced on a LAST word: pkt_hdr holds it
    input  wire [31:0]   pkt_hdr,              // header word of the packet that just completed / aborted
    output wire          in_ready,
    // to the user
    output reg           out_valid,
    output reg  [DW-1:0] out_data,
    output reg           out_cnt2, out_last, out_hdr,
    input  wire          out_ready,
    // to link_arq_tx (through link_rec_sync): received acknowledgements, and ACK/NAK to send
    // (`include` for the packet type constants)
    output reg           ack_stb, ack_nak,
    output reg  [7:0]    ack_seq,
    output reg           ctl_stb, ctl_nak,
    output reg  [7:0]    ctl_seq,
    // status
    output reg  [15:0]   disc,                 // packets discarded (bad or out of sequence)
    output reg  [15:0]   naks,                 // NAKs requested
    output reg  [31:0]   good_pkts
);
    `include "link_funcs.svh"
    localparam int DEPTH = 1 << BUF_AW;
    localparam int BW = DW + 3;
    (* ram_style = "block" *) reg [BW-1:0] buf_ram [0:DEPTH-1];
    reg [BUF_AW-1:0]  wt = 0, wc = 0, rp = 0;  // tentative write, committed write, read
    wire [BUF_AW-1:0] used = wt - rp;                     // modulo DEPTH (pointers wrap)
    assign in_ready = (used < DEPTH - IN_SLACK);
    always @(posedge clk) if (in_valid) buf_ram[wt] <= {in_hdr, in_cnt2, in_last, in_data};
    // the read in its own block with the same enable as the read side below: inside that block (with its reset branch
    // and the other registers) Vivado finds ram_style block "infeasible" and builds 2 K x 67 from 2880 LUTRAMs
    // (3858 LUT out of context, against 196 LUT + 4 RAMB36 this way)
    always @(posedge clk) if (~rst & (~out_valid | out_ready)) {out_hdr, out_cnt2, out_last, out_data} <= buf_ram[rp];

    reg [7:0]  expected = 0;
    reg        rx_fresh = 1;                              // nothing committed since reset
    reg [3:0]  ack_cnt = 0;
    reg [31:0] ack_tmr = 0, nak_tmr = 0;
    wire        is_ctl   = (pkt_hdr[31:24] == LINK_T_ACK) | (pkt_hdr[31:24] == LINK_T_NAK);
    // boundary word after a link-up; the zero bits make a CRC word that a resync after a frame error lands on unlikely
    // to pass for one (2^-23)
    wire        is_bnd   = bnd_seen & (pkt_hdr[31:24] == LINK_T_BND) & (pkt_hdr[15:1] == 15'd0);
    wire        bnd_adopt = is_bnd & (pkt_hdr[0] | rx_fresh);              // fresh numbering, or we know none
    wire        seq_ok   = (pkt_hdr[23:16] == expected);
    wire        commit   = pkt_done & pkt_ok & ~is_ctl & seq_ok;
    wire        bad      = (pkt_done & ~pkt_ok) | pkt_abort;                   // damaged packet
    wire        dup      = pkt_done & pkt_ok & ~is_ctl & ~seq_ok;              // old or future packet
    wire        nak_now  = enable & (bad | dup) & (nak_tmr == 0);

    always @(posedge clk) begin
        ack_stb <= 0; ctl_stb <= 0;
        // read side runs whenever not in reset: committed packets stay deliverable across a retrain
        if (rst) begin
            out_valid <= 0; rp <= 0;
        end else if (!out_valid || out_ready) begin
            out_valid <= (rp != wc);
            if (rp != wc) rp <= rp + 1;
        end
        if (rst) begin
            expected <= 0; disc <= 0; naks <= 0; good_pkts <= 0; wt <= 0; wc <= 0; rx_fresh <= 1;
            ack_cnt <= 0; ack_tmr <= 0; nak_tmr <= 0;
        end else if (!enable) begin
            // link down / retraining: drop only the tentative data (an unfinished packet); committed
            // packets were already acknowledged and must still reach the user; keep `expected` (the
            // far transmitter keeps its numbering and resends from the oldest unacknowledged packet)
            wt <= wc; ack_cnt <= 0; ack_tmr <= 0; nak_tmr <= 0;
        end else begin
            // write side
            if (in_valid) wt <= wt + 1;
            if (is_bnd) begin
                // a link-up: a restarted far transmitter (or a restarted us) gives the numbering; after a retrain
                // we keep ours (duplicates of packets already delivered are dropped below)
                if (bnd_adopt) expected <= pkt_hdr[23:16];
                wt <= wc; ack_cnt <= 0; nak_tmr <= 0;
            end else if (commit) begin
                wc <= in_valid ? wt + 1 : wt;
                expected <= expected + 8'd1; good_pkts <= good_pkts + 1; rx_fresh <= 0;
                if (ack_cnt == ACK_EVERY - 1) begin ack_cnt <= 0; ctl_stb <= 1; ctl_nak <= 0; ctl_seq <= expected + 8'd1; ack_tmr <= 0; end
                else begin ack_cnt <= ack_cnt + 1; end
            end else if (pkt_done | pkt_abort) begin
                wt <= wc;                                                   // roll back
                if (is_ctl & pkt_done & pkt_ok) begin
                    ack_stb <= 1; ack_nak <= (pkt_hdr[31:24] == 8'hFE); ack_seq <= pkt_hdr[23:16];
                end else begin
                    disc <= disc + 1;
                    if (nak_now) begin
                        // bad packet, or out of sequence (an old duplicate or a gap): NAK with expected
                        ctl_stb <= 1; ctl_nak <= 1; ctl_seq <= expected; naks <= naks + 1;
                        nak_tmr <= NAK_HOLD; ack_cnt <= 0; ack_tmr <= 0;
                    end
                end
            end
            // periodic ACK for packets received since the last one
            if (ack_cnt != 0) begin
                if (ack_tmr == ACK_TO) begin ctl_stb <= 1; ctl_nak <= 0; ctl_seq <= expected; ack_cnt <= 0; ack_tmr <= 0; end
                else ack_tmr <= ack_tmr + 1;
            end
            if (nak_tmr != 0) nak_tmr <= nak_tmr - 1;
        end
    end
endmodule
`default_nettype wire
