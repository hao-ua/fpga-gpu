// link_test_gen — bring-up traffic: TEST packets (type 0x01) of pseudo-random length 1..MAXLEN
// carrying one continuous xorshift32 stream, with an IDLE packet (type 0x00, len 0) every 64th.
// Output is a valid/ready stream of 2-word beats {hi, lo}: with WPB = 2 the header shares its beat
// with the first payload word and payload goes out two words per beat (cnt2 = 1 except for a
// one-word final beat); with WPB = 1 every beat carries one word in `lo` (cnt2 = 0).
// A beat is held until accepted (tx_valid & tx_ready).
`default_nettype none
module link_test_gen #(parameter int MAXLEN = 1024, parameter int WPB = 1, parameter int RATE_DIV = 1) (
    input  wire        clk, rst,
    input  wire        enable,
    output reg         tx_valid,
    output reg  [63:0] tx_data,
    output reg         tx_cnt2,
    output reg         tx_last,
    input  wire        tx_ready
);
    `include "link_funcs.svh"
    reg [31:0] prbs;            // next payload word
    reg [15:0] lfsr;            // length source
    reg [15:0] remaining;
    reg        in_pkt;
    reg [5:0]  pkt_no;
    wire [15:0] next_len = (lfsr % MAXLEN) + 16'd1;
    wire        idle_pkt = (pkt_no == 6'd63);
    // RATE_DIV > 1 offers a beat only every RATE_DIV cycles (bench: a producer slower than the lane,
    // which is the hardware situation the link must cope with)
    reg  [7:0]  div_cnt = 0;
    always @(posedge clk) div_cnt <= (rst || div_cnt == RATE_DIV - 1) ? 8'd0 : div_cnt + 8'd1;
    wire        slot = (div_cnt == 0);
    wire        adv = enable & slot & (~tx_valid | tx_ready);   // output register free (or being consumed now)
    wire [31:0] prbs2 = xorshift32(prbs);

    always @(posedge clk) begin
        if (rst) begin
            prbs <= 32'h2545F491; lfsr <= 16'hACE1; remaining <= 0; in_pkt <= 0; pkt_no <= 0;
            tx_valid <= 0; tx_data <= 0; tx_cnt2 <= 0; tx_last <= 0;
        end else begin
            if (tx_valid & tx_ready & ~adv) tx_valid <= 0;     // consumed, nothing new (enable low)
            if (adv) begin
                tx_valid <= 1;
                if (!in_pkt) begin
                    lfsr  <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
                    pkt_no <= pkt_no + 1;
                    if (idle_pkt) begin
                        tx_data <= 64'd0; tx_cnt2 <= 0; tx_last <= 1; in_pkt <= 0;
                    end else if (WPB == 2) begin
                        // header + first payload word in one beat
                        tx_data <= {prbs, 8'h01, 8'h00, next_len}; tx_cnt2 <= 1; prbs <= prbs2;
                        tx_last <= (next_len == 16'd1); remaining <= next_len - 16'd1; in_pkt <= (next_len != 16'd1);
                    end else begin
                        tx_data <= {32'd0, 8'h01, 8'h00, next_len}; tx_cnt2 <= 0; tx_last <= 0;
                        remaining <= next_len; in_pkt <= 1;
                    end
                end else if (WPB == 2 && remaining >= 16'd2) begin
                    tx_data <= {prbs2, prbs}; tx_cnt2 <= 1; prbs <= xorshift32(prbs2);
                    remaining <= remaining - 16'd2;
                    tx_last <= (remaining == 16'd2);
                    if (remaining == 16'd2) in_pkt <= 0;
                end else begin
                    tx_data <= {32'd0, prbs}; tx_cnt2 <= 0; prbs <= prbs2;
                    remaining <= remaining - 16'd1;
                    tx_last <= (remaining == 16'd1);
                    if (remaining == 16'd1) in_pkt <= 0;
                end
            end
        end
    end
endmodule
`default_nettype wire
