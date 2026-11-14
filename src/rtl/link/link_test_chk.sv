// link_test_chk — verifies the xorshift32 payload stream of TEST packets (type 0x01) across
// packet boundaries on a 2-word beat stream; self-resynchronises on the first word after any mismatch.
`default_nettype none
module link_test_chk (
    input  wire        clk, rst,
    input  wire        rx_valid,          // lo is a header or payload word
    input  wire [63:0] rx_data,           // {hi, lo}
    input  wire        rx_cnt2,           // hi is a payload word too
    input  wire        rx_hdr,            // lo is a header
    output reg  [31:0] prbs_errs,
    output reg  [31:0] payload_words,
    output reg  [31:0] bit_err_map        // sticky OR of (received ^ expected): a dead wire sets its bit(s)
);
    `include "link_funcs.svh"
    reg        synced;
    reg        in_test;
    reg        prev_ok;          // previous payload word matched -> this word's diff is meaningful per bit
    reg [31:0] expected;
    wire [31:0] lo = rx_data[31:0], hi = rx_data[63:32];
    // two words per cycle: evaluate lo then hi with temporaries
    reg        s, it, pok; reg [31:0] exp; reg [31:0] errs, pw, bem;
    always @(posedge clk) begin
        if (rst) begin synced <= 0; in_test <= 0; prev_ok <= 0; expected <= 0; prbs_errs <= 0; payload_words <= 0; bit_err_map <= 0; end
        else if (rx_valid) begin
            s = synced; it = in_test; pok = prev_ok; exp = expected; errs = prbs_errs; pw = payload_words; bem = bit_err_map;
            if (rx_hdr) it = (lo[31:24] == 8'h01);
            else if (it) begin
                pw = pw + 1;
                if (s && lo != exp) begin errs = errs + 1; if (pok) bem = bem | (lo ^ exp); pok = 0; end
                else pok = s;
                exp = xorshift32(lo); s = 1;
            end
            if (rx_cnt2 && it) begin
                pw = pw + 1;
                if (s && hi != exp) begin errs = errs + 1; if (pok) bem = bem | (hi ^ exp); pok = 0; end
                else pok = s;
                exp = xorshift32(hi); s = 1;
            end
            synced <= s; in_test <= it; prev_ok <= pok; expected <= exp; prbs_errs <= errs; payload_words <= pw; bit_err_map <= bem;
        end
    end
endmodule
`default_nettype wire
