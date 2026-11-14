// link_arq_tx — transmit side of the go-back-N retransmission layer (lane clock domain).
//
// Sits between the user TX FIFO (read side) and the framer. Every user beat is copied into a ring
// (BRAM) and stays there until the far receiver has acknowledged its packet; the framer is fed from
// a send pointer into that ring. Packets are numbered here (header[23:16]), so a retransmitted
// packet carries its original number. A NAK, a timeout without ACK progress, or the far UP wire
// returning after a link outage rewinds the send pointer to the oldest unacknowledged packet.
// ACK/NAK requests from the local receiver are injected into the stream as one-beat control
// packets (type 0xFF = ACK, 0xFE = NAK, acknowledged sequence in the seq byte, length 0) at packet
// boundaries; control packets are not numbered and not retransmitted (ACKs are cumulative).
// The timeout only runs while our own receiver is trained (`our_up`): until then the far framer is
// in TRAIN mode and cannot deliver its ACKs, so a timeout would only resend packets the far side
// already has (without this gate, about 4 timeouts x 15 duplicates at every link-up on hardware).
//
// Beat layout: data[31:0] is the header word of a packet's first beat; bit W-1 = last; for the
// 64-bit lane bit W-2 = cnt2 (W = 66), for the 32-bit lane W = 33.
`default_nettype none
module link_arq_tx #(
    parameter int W         = 66,
    parameter int RING_AW   = 12,              // ring depth 4096 beats
    parameter int TO_CYCLES = 20000            // lane cycles without ACK progress -> rewind (200 us @100 MHz:
                                               // longer than ACK_EVERY maximum-size packets plus the round trip)
)(
    input  wire         clk, rst,              // lane clock domain
    // user FIFO read side (FWFT)
    input  wire         u_empty,
    input  wire [W-1:0] u_data,
    output wire         u_ren,
    // to the framer (FWFT)
    output wire         f_empty,
    output wire [W-1:0] f_data,
    input  wire         f_ren,
    output wire         abort,                 // rewinding mid-packet: the framer terminates its packet now
    output wire [7:0]   bnd_seq,               // sequence number the far receiver must expect next (boundary word)
    output reg          bnd_fresh,             // our numbering restarted (reset) and nothing of it acknowledged yet
    input  wire         far_up,                // far receiver trained (synchronised by the caller)
    input  wire         our_up,                // our receiver trained: the far framer can send us ACKs (synchronised)
    // acknowledgements received from the far end (via link_rec_sync)
    input  wire         ack_stb,               // pulse: ack_seq/ack_nak valid this cycle
    input  wire         ack_nak,
    input  wire [7:0]   ack_seq,               // far end's next expected sequence
    // ACK/NAK the local receiver wants sent (via link_rec_sync)
    input  wire         ctl_stb,
    input  wire         ctl_nak,
    input  wire [7:0]   ctl_seq,
    // status
    output reg  [15:0]  rewinds,
    output reg  [15:0]  timeouts
);
    `include "link_funcs.svh"
    localparam int DEPTH = 1 << RING_AW;
    // ---- ring + packet table ----
    (* ram_style = "block" *) reg [W-1:0] ring [0:DEPTH-1];
    reg [RING_AW-1:0] pstart [0:15];           // start address of packet seq[3:0]
    reg [RING_AW-1:0] wp = 0, bp = 0, ra = 0;  // write, base (oldest unacked), read-issue pointers
    reg [7:0]         next_seq = 0, base_seq = 0;
    reg               at_wstart = 1;           // next user beat is a header
    wire [7:0]  in_window = next_seq - base_seq;
    wire [RING_AW-1:0] occupancy = wp - bp;               // modulo DEPTH (pointers wrap)
    // a new packet needs a free table slot; the rest of a packet in progress only needs ring space
    wire        window_ok = (in_window < 8'd15) | ~at_wstart;
    wire        accept = ~u_empty & window_ok & (occupancy < DEPTH - 2);
    assign u_ren = accept;
    wire [W-1:0] u_stamped = at_wstart ? {u_data[W-1:24], next_seq, u_data[15:0]} : u_data;
    always @(posedge clk) begin
        if (accept) begin
            ring[wp] <= u_stamped;
            if (at_wstart) pstart[next_seq[3:0]] <= wp;
        end
    end

    // ---- read path: RAM with registered output + 2-entry skid = FWFT to the framer ----
    reg [W-1:0] rq = 0;                        // RAM output register
    reg         inflight = 0, discard = 0;     // a read was issued last cycle / its data must be dropped
    reg [W-1:0] s0 = 0, s1 = 0; reg s0v = 0, s1v = 0;
    reg         at_sstart = 1;                 // next real beat to send starts a packet
    reg         ctl_pend = 0, ctl_pend_nak = 0; reg [7:0] ctl_pend_seq = 0;
    wire        inject = ctl_pend & at_sstart;
    wire [31:0] ctl_hdr = {ctl_pend_nak ? LINK_T_NAK : LINK_T_ACK, ctl_pend_seq, 16'd0};
    wire [W-1:0] ctl_beat;
    generate
        if (W == 66) begin : g66
            assign ctl_beat = {1'b1, 1'b0, 32'd0, ctl_hdr};          // last, cnt2=0, hi=0, lo=hdr
        end else begin : g33
            assign ctl_beat = {1'b1, ctl_hdr};
        end
    endgenerate
    assign f_empty = inject ? 1'b0 : ~s0v;
    assign f_data  = inject ? ctl_beat : s0;
    wire pop_real  = f_ren & ~inject;
    wire pop_ctl   = f_ren & inject;
    wire [1:0] nskid = s0v + s1v + inflight;
    wire rewind;
    wire issue = (ra != wp) & ~rewind & ((nskid < 2) | pop_real);
    always @(posedge clk) rq <= ring[ra];

    // ---- control / ack bookkeeping ----
    reg [31:0] to_cnt = 0;
    reg        far_up_d = 0;
    wire       unacked = (base_seq != next_seq);
    wire       ack_in_range = ack_stb & (((ack_seq - base_seq) & 8'hFF) <= in_window) & (ack_seq != base_seq);
    wire       timeout = unacked & far_up & our_up & (to_cnt == TO_CYCLES);
    wire       relink  = far_up & ~far_up_d;
    // A NAK or timeout rewinds at the next packet boundary of the send stream (so the framer never
    // gets a new header spliced into the packet it is sending), or when the stream is STUCK in the
    // middle of a packet: nothing left to send, user data waiting, and the ring full so the rest of
    // the packet cannot arrive before ACKs free space — then `abort` makes the framer terminate the
    // dangling packet with a LAST beat. A ring that is merely momentarily empty (the producer is
    // not faster than the lane — the normal case on hardware) is NOT a reason to abort: the packet
    // completes and the rewind follows at its boundary. On relink the framer is at a header anyway,
    // so that rewind is immediate.
    reg        rw_pend = 0;
    wire       rw_req  = (ack_stb & ack_nak & unacked) | timeout;
    // `stuck` implies f_empty (no skid data; inject needs at_sstart, which is 0 whenever abort can
    // fire), so an abort cycle is always pop-free; the framers therefore need not gate on abort.
    wire       idle    = (ra == wp) & ~s0v & ~inflight;
    wire       stuck   = idle & ~u_empty & ~accept;
    wire       can_rw  = (at_sstart & ~pop_real) | stuck;
    assign rewind = relink | (can_rw & (rw_pend | rw_req));
    assign abort  = rewind & ~at_sstart;
    assign bnd_seq = base_seq;                 // after the relink rewind the stream restarts at base_seq
    // new base: start of packet ack_seq; if every complete packet is acknowledged, the start of the
    // packet being written (if one is) or the write pointer
    wire [RING_AW-1:0] bp_next = ~ack_in_range ? bp :
                                 (ack_seq != next_seq) ? pstart[ack_seq[3:0]] :
                                 (at_wstart ? wp : pstart[next_seq[3:0]]);

    always @(posedge clk) begin
        if (rst) begin
            wp <= 0; bp <= 0; ra <= 0; next_seq <= 0; base_seq <= 0; at_wstart <= 1;
            inflight <= 0; discard <= 0; s0v <= 0; s1v <= 0; at_sstart <= 1;
            ctl_pend <= 0; to_cnt <= 0; far_up_d <= 0; rewinds <= 0; timeouts <= 0; rw_pend <= 0; bnd_fresh <= 1;
        end else begin
            far_up_d <= far_up;
            if (rewind) rw_pend <= 0; else if (rw_req) rw_pend <= 1;
            // write side
            if (accept) begin
                wp <= wp + 1;
                if (at_wstart) at_wstart <= 0;
                if (u_data[W-1]) begin at_wstart <= 1; next_seq <= next_seq + 8'd1; end
            end
            // acknowledgements
            if (ack_in_range) begin base_seq <= ack_seq; bp <= bp_next; bnd_fresh <= 0; end
            if (ack_in_range | ~unacked | ~our_up) to_cnt <= 0;        // no ACK can arrive while we are untrained
            else if (to_cnt != TO_CYCLES) to_cnt <= to_cnt + 1;
            else to_cnt <= 0;
            if (timeout) timeouts <= timeouts + 1;
            if (ack_stb & ack_nak & unacked) rewinds <= rewinds + 1;
            // control packet requests (latest wins)
            if (ctl_stb) begin ctl_pend <= 1; ctl_pend_nak <= ctl_nak; ctl_pend_seq <= ctl_seq; end
            if (pop_ctl) ctl_pend <= 0;
            // read side
            if (rewind) begin
                ra <= bp_next; s0v <= 0; s1v <= 0; discard <= inflight; inflight <= 0; at_sstart <= 1;
            end else begin
                inflight <= issue;
                discard  <= 0;
                if (issue) ra <= ra + 1;
                // skid: arriving data (inflight && !discard) appends; pop_real removes the head
                case ({(inflight & ~discard), pop_real})
                2'b10: begin if (!s0v) begin s0 <= rq; s0v <= 1; end else begin s1 <= rq; s1v <= 1; end end
                2'b01: begin if (s1v) begin s0 <= s1; s1v <= 0; end else s0v <= 0; end
                2'b11: begin if (s1v) begin s0 <= s1; s1 <= rq; end else s0 <= rq; end
                default: ;
                endcase
                if (pop_real) at_sstart <= s0[W-1];
            end
        end
    end
endmodule

// link_rec_sync — carries a small record from one clock domain to another with a request/
// acknowledge toggle pair; the source may update again only after the sink took the record, so
// events arriving faster than the crossing coalesce into the latest values (fine for cumulative
// ACKs / "send me an ACK now" requests).
module link_rec_sync #(parameter int W = 9) (
    input  wire         sclk, srst,
    input  wire         s_stb,                 // source: new record (latest wins while busy)
    input  wire [W-1:0] s_data,
    input  wire         dclk, drst,
    output reg          d_stb,                 // sink: one-cycle pulse, d_data valid
    output reg  [W-1:0] d_data
);
    reg          req = 0, pend = 0; reg [W-1:0] hold = 0, xfer = 0;
    (* ASYNC_REG = "TRUE" *) reg [1:0] ack_s = 0;
    (* ASYNC_REG = "TRUE" *) reg [1:0] req_s = 0;
    reg          ack = 0, req_d = 0;
    wire busy = req ^ ack_s[1];
    always @(posedge sclk) begin
        ack_s <= {ack_s[0], ack};
        if (srst) begin req <= 0; pend <= 0; end
        else begin
            if (s_stb) begin hold <= s_data; pend <= 1; end
            if (!busy && (pend || s_stb)) begin xfer <= s_stb ? s_data : hold; req <= ~req; pend <= 0; end
        end
    end
    always @(posedge dclk) begin
        req_s <= {req_s[0], req};
        req_d <= req_s[1];
        d_stb <= 0;
        if (drst) begin ack <= 0; end
        else if (req_s[1] ^ req_d) begin d_data <= xfer; d_stb <= 1; ack <= ~ack; end
    end
endmodule
`default_nettype wire
