// link_loop — the board link inside one FPGA: link_a_port and link_b_port joined by one FIFO each way, in place of the two
// link_core instances and the wires between two boards. Both sides keep link_core's user contract: beats {hi, lo}, a
// packet's header in lo of its first beat, `last` on its final beat, cnt2 = hi valid; at the receiving end rx_hdr marks a
// packet's first beat. Packets arrive in order and whole; a beat the sender has not handed over yet is a pause inside the
// packet, which both ports take as a stall (they move beats by valid / ready). A -> B carries {cnt2, last, 64 bits};
// B -> A one 32-bit word per beat (link_b_port's backward-lane words, cnt2 = 0).
// While rst is high neither side may hand over a beat (tx_ready low), so no packet loses its head to a reset; rst must
// cover both ports' resets.
`default_nettype none
module link_loop #(
    parameter int AW_F = 9,                     // A -> B: 2**AW_F + 1 beats
    parameter int AW_B = 9                      // B -> A: 2**AW_B + 1 words
)(
    input  wire         clk, rst,
    // link_a_port: role A's user side
    input  wire         a_tx_valid,
    input  wire [63:0]  a_tx_data,
    input  wire         a_tx_cnt2, a_tx_last,
    output wire         a_tx_ready,
    output wire         a_rx_valid,
    output wire [63:0]  a_rx_data,
    output wire         a_rx_cnt2, a_rx_last, a_rx_hdr,
    input  wire         a_rx_ready,
    // link_b_port: role B's user side
    input  wire         b_tx_valid,
    input  wire [63:0]  b_tx_data,
    input  wire         b_tx_cnt2, b_tx_last,
    output wire         b_tx_ready,
    output wire         b_rx_valid,
    output wire [63:0]  b_rx_data,
    output wire         b_rx_cnt2, b_rx_last, b_rx_hdr,
    input  wire         b_rx_ready
);
    // ---------------- A -> B ----------------
    wire        f_full, f_v; wire [65:0] f_q;
    sfifo #(.W(66), .AW(AW_F)) u_ab (.clk, .rst, .wen(a_tx_valid & ~rst), .wdata({a_tx_cnt2, a_tx_last, a_tx_data}),
        .full(f_full), .count(), .ren(b_rx_ready & f_v), .rdata(f_q), .rvalid(f_v));
    assign a_tx_ready = ~f_full & ~rst;
    reg f_first;                                // the next beat out starts a packet
    always @(posedge clk)
        if (rst) f_first <= 1'b1;
        else if (f_v && b_rx_ready) f_first <= f_q[64];
    assign b_rx_valid = f_v;
    assign b_rx_data  = f_q[63:0];
    assign b_rx_last  = f_q[64];
    assign b_rx_cnt2  = f_q[65];
    assign b_rx_hdr   = f_v & f_first;

    // ---------------- B -> A ----------------
    wire        k_full, k_v; wire [32:0] k_q;
    sfifo #(.W(33), .AW(AW_B)) u_ba (.clk, .rst, .wen(b_tx_valid & ~rst), .wdata({b_tx_last, b_tx_data[31:0]}),
        .full(k_full), .count(), .ren(a_rx_ready & k_v), .rdata(k_q), .rvalid(k_v));
    assign b_tx_ready = ~k_full & ~rst;
    reg k_first;
    always @(posedge clk)
        if (rst) k_first <= 1'b1;
        else if (k_v && a_rx_ready) k_first <= k_q[32];
    assign a_rx_valid = k_v;
    assign a_rx_data  = {32'h0, k_q[31:0]};
    assign a_rx_last  = k_q[32];
    assign a_rx_cnt2  = 1'b0;
    assign a_rx_hdr   = k_v & k_first;

    wire _unused = &{1'b0, b_tx_cnt2, b_tx_data[63:32]};
endmodule
`default_nettype wire
