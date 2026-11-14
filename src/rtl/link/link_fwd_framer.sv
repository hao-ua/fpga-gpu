// link_fwd_framer — link-layer TX of the 64-bit forward lane (lane clock domain).
//
// The input (link_arq_tx, FWFT) delivers beats {last, cnt2, hi, lo}: a packet starts with its
// header in `lo` (sequence number already stamped by the ARQ layer), every beat except possibly
// the last carries two words (cnt2 = 1). The framer runs CRC-32 over the words in order (lo, hi)
// and appends the CRC word: into the free `hi` slot of a one-word final beat, else into a beat of
// its own. LAST on the wire marks the beat whose last valid word is the CRC. XOFF is honoured
// between beats.
// Training handshake as in link_framer: training-stream beats (xorshift32 in lo, xorshift32b in hi)
// while the far UP is low, a boundary beat (FTRAIN0 with LAST) when it rises; a cut packet is
// abandoned here and resent by link_arq_tx.
// Payload and CRC words leave scrambled: keystream LINK_SSEED restarting at each header, one step per word in the order
// lo, hi (link_funcs.svh); the header in lo stays plain.
`default_nettype none
module link_fwd_framer (
    input  wire        clk, rst,
    input  wire        f_empty,
    input  wire [65:0] f_data,          // {last, cnt2, hi, lo}
    output wire        f_ren,
    input  wire        abort,           // link_arq_tx rewinds mid-packet: terminate the packet now (LAST)
    input  wire [7:0]  bnd_seq,         // sequence number carried by the boundary beat
    input  wire        bnd_fresh,       // ... and whether it starts a new numbering (bit 0, link_arq_rx)
    input  wire        xoff,
    input  wire        far_up,
    input  wire        fill,            // send fill beats between packets instead of idling (LINK_T_FILL in lo)
    output reg  [31:0] lo, hi,
    output reg         valid_lo, valid_hi, last,
    output reg  [31:0] tx_pkts,
    output reg         training
);
    `include "link_funcs.svh"
    reg        at_header, send_crc, boundary;
    reg [31:0] tlo, thi;                // training stream states
    reg [31:0] crc;
    reg [31:0] ks;                      // scrambler keystream for the next payload / CRC word
    wire [31:0] ks1 = xorshift32(ks), ks2 = xorshift32(ks1), ksh = xorshift32(LINK_SSEED);
    wire [31:0] f_lo = f_data[31:0], f_hi = f_data[63:32];
    wire        f_cnt2 = f_data[64], f_last = f_data[65];
    wire [31:0] lo_out = f_lo;
    wire [31:0] crc_lo = crc32_word(crc, lo_out);
    wire [31:0] crc_hi = crc32_word(crc_lo, f_hi);

    // `abort` never coincides with a pop (see link_framer), so it must not gate f_ren (loop)
    wire go = far_up & ~boundary & ~xoff;
    assign f_ren = go & ~send_crc & ~f_empty;

    always @(posedge clk) begin
        if (rst) begin
            at_header <= 1'b1; send_crc <= 1'b0; boundary <= 1'b1;
            tlo <= LINK_TSEED_LO; thi <= LINK_TSEED_HI;
            crc <= 32'hFFFFFFFF; ks <= LINK_SSEED;
            lo <= 0; hi <= 0; valid_lo <= 0; valid_hi <= 0; last <= 0; tx_pkts <= 0; training <= 1'b1;
        end else begin
            valid_lo <= 1'b0; valid_hi <= 1'b0; last <= 1'b0;
            if (!far_up) begin
                training <= 1'b1; boundary <= 1'b1;
                send_crc <= 1'b0; at_header <= 1'b1; crc <= 32'hFFFFFFFF;
                lo <= tlo; hi <= thi; valid_lo <= 1'b1; valid_hi <= 1'b1;
                tlo <= xorshift32(tlo); thi <= xorshift32b(thi);
            end else begin
                training <= 1'b0;
                if (boundary) begin
                    lo <= {LINK_T_BND, bnd_seq, 15'd0, bnd_fresh}; hi <= 32'd0; valid_lo <= 1'b1; last <= 1'b1; boundary <= 1'b0;
                end else if (abort && (!at_header || send_crc)) begin
                    // dangling packet: close it with a (wrong) CRC word so the far deframer discards it
                    lo <= ~crc ^ ks; hi <= 32'd0; valid_lo <= 1'b1; last <= 1'b1;
                    send_crc <= 1'b0; crc <= 32'hFFFFFFFF; at_header <= 1'b1;
                end else if (go) begin
                    if (send_crc) begin
                        lo <= ~crc ^ ks; hi <= 32'd0; valid_lo <= 1'b1; last <= 1'b1;
                        send_crc <= 1'b0; crc <= 32'hFFFFFFFF; at_header <= 1'b1;
                        tx_pkts <= tx_pkts + 32'd1;
                    end else if (!f_empty) begin
                        // a header beat: lo plain, hi takes the keystream's first word; else lo and hi the next two
                        lo <= at_header ? lo_out : (lo_out ^ ks); valid_lo <= 1'b1; at_header <= 1'b0;
                        if (f_last && !f_cnt2) begin
                            // one-word final beat: CRC rides in the free high slot
                            hi <= ~crc_lo ^ (at_header ? LINK_SSEED : ks1); valid_hi <= 1'b1; last <= 1'b1;
                            crc <= 32'hFFFFFFFF; at_header <= 1'b1;
                            tx_pkts <= tx_pkts + 32'd1;
                        end else begin
                            hi <= f_hi ^ (at_header ? LINK_SSEED : ks1); valid_hi <= f_cnt2;
                            crc <= f_cnt2 ? crc_hi : crc_lo;
                            if (f_last) send_crc <= 1'b1;
                            ks <= at_header ? (f_cnt2 ? ksh : LINK_SSEED) : (f_cnt2 ? ks2 : ks1);
                        end
                    end else if (fill && at_header) begin
                        lo <= {LINK_T_FILL, tlo[23:0]}; hi <= thi; valid_lo <= 1'b1; valid_hi <= 1'b1; last <= 1'b1;
                        tlo <= xorshift32(tlo); thi <= xorshift32b(thi);
                    end
                end else if (fill && at_header && !send_crc && !boundary) begin
                    // paused by XOFF between packets: fill beats never reach the far FIFO
                    lo <= {LINK_T_FILL, tlo[23:0]}; hi <= thi; valid_lo <= 1'b1; valid_hi <= 1'b1; last <= 1'b1;
                    tlo <= xorshift32(tlo); thi <= xorshift32b(thi);
                end
            end
        end
    end
endmodule
`default_nettype wire
