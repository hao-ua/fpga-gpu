// link_framer — link-layer TX (lane clock domain). Pulls packets from its input (link_arq_tx,
// FWFT: header first, `last` on the final payload word — on the header itself when len == 0;
// the sequence number in header[23:16] is already stamped by the ARQ layer), appends the CRC-32
// word (LAST on the wire is set only on that CRC word) and honours XOFF between any two words.
//
// Training handshake: while the far end's UP wire is low its receiver is
// not trained on our lane, so instead of data we send the training stream (xorshift32 sequence,
// VALID=1, LAST=0, XOFF ignored — nothing is consumed from the input). A packet cut by that switch
// is abandoned here; link_arq_tx rewinds to the start of the oldest unacknowledged packet when UP
// returns, so the next word consumed is a header again. When UP rises we first send one boundary
// word (TRAIN0 with LAST=1): the far deframer discards everything up to a LAST and then expects a
// header. Payload and CRC words leave scrambled (LINK_SSEED keystream restarting at each header, link_funcs.svh).
`default_nettype none
module link_framer (
    input  wire        clk, rst,
    // TX FIFO read port (FWFT)
    input  wire        f_empty,
    input  wire [32:0] f_data,          // {last, word}
    output wire        f_ren,
    input  wire        abort,           // link_arq_tx rewinds mid-packet: terminate the packet now (LAST)
    input  wire [7:0]  bnd_seq,         // sequence number carried by the boundary word
    input  wire        bnd_fresh,       // ... and whether it starts a new numbering (bit 0, link_arq_rx)
    input  wire        xoff,            // synchronised into this domain by the caller
    input  wire        far_up,          // far receiver trained on our lane (synchronised by the caller)
    input  wire        fill,            // send fill words between packets instead of idling (LINK_T_FILL)
    // to the PHY
    output reg  [31:0] word,
    output reg         valid,
    output reg         last,
    output reg  [31:0] tx_pkts,         // packets completed (CRC emitted)
    output reg         training         // sending TRAIN words (far_up low)
);
    `include "link_funcs.svh"
    reg        at_header;               // next FIFO word is a packet header
    reg        send_crc;                // the CRC word is pending
    reg        boundary;                // a boundary word is due before the first data word
    reg [31:0] tseq;                    // training stream state
    reg [31:0] crc;
    reg [31:0] ks;                      // scrambler keystream for the next payload / CRC word
    wire [31:0] f_word = f_data[31:0];
    wire        f_last = f_data[32];
    wire [31:0] w_out = f_word;

    // `abort` never coincides with a pop (link_arq_tx raises it only when the stream is idle or in a
    // cycle it knows is pop-free), so it must not gate f_ren: that would close a combinational loop
    // abort -> f_ren -> rewind -> abort.
    wire go = far_up & ~boundary & ~xoff;
    assign f_ren = go & ~send_crc & ~f_empty;

    always @(posedge clk) begin
        if (rst) begin
            at_header <= 1'b1; send_crc <= 1'b0; boundary <= 1'b1; tseq <= LINK_TSEED_LO;
            crc <= 32'hFFFFFFFF; ks <= LINK_SSEED;
            word <= 32'd0; valid <= 1'b0; last <= 1'b0; tx_pkts <= 32'd0; training <= 1'b1;
        end else begin
            valid <= 1'b0; last <= 1'b0;
            if (!far_up) begin
                // TRAIN mode (a packet in progress is abandoned; the ARQ layer resends it)
                training <= 1'b1; boundary <= 1'b1;
                send_crc <= 1'b0; at_header <= 1'b1; crc <= 32'hFFFFFFFF;
                word <= tseq; valid <= 1'b1; tseq <= xorshift32(tseq);
            end else begin
                training <= 1'b0;
                if (boundary) begin
                    word <= {LINK_T_BND, bnd_seq, 15'd0, bnd_fresh}; valid <= 1'b1; last <= 1'b1; boundary <= 1'b0;
                end else if (abort && (!at_header || send_crc)) begin
                    // dangling packet: close it with a (wrong) CRC word so the far deframer discards it
                    word <= ~crc ^ ks; valid <= 1'b1; last <= 1'b1;
                    send_crc <= 1'b0; crc <= 32'hFFFFFFFF; at_header <= 1'b1;
                end else if (go) begin
                    if (send_crc) begin
                        word <= ~crc ^ ks; valid <= 1'b1; last <= 1'b1;
                        send_crc <= 1'b0; crc <= 32'hFFFFFFFF; at_header <= 1'b1;
                        tx_pkts <= tx_pkts + 32'd1;
                    end else if (!f_empty) begin
                        word <= at_header ? w_out : (w_out ^ ks); valid <= 1'b1;   // header plain, payload scrambled
                        ks <= at_header ? LINK_SSEED : xorshift32(ks);
                        crc <= crc32_word(crc, w_out);
                        at_header <= 1'b0;
                        if (f_last) send_crc <= 1'b1;
                    end else if (fill && at_header) begin
                        word <= {LINK_T_FILL, tseq[23:0]}; valid <= 1'b1; last <= 1'b1; tseq <= xorshift32(tseq);
                    end
                end else if (fill && at_header && !send_crc && !boundary) begin
                    // paused by XOFF between packets: fill words never reach the far FIFO, so they may go on
                    word <= {LINK_T_FILL, tseq[23:0]}; valid <= 1'b1; last <= 1'b1; tseq <= xorshift32(tseq);
                end
            end
        end
    end
endmodule
`default_nettype wire
