// link_rx_diag — receive-side error diagnostics of one lane, recovered-clock domain.
// Classifies every captured word by its place in the traffic: a burst is a run of consecutive valid cycles; `pos`
// counts the valid words before this one in its burst, `gap` the idle cycles before the burst. For every parity
// error on a real (non-fill) word it updates a histogram and keeps a snapshot of the last one. Only words received while
// en (our receiver trained: the training scan samples at bad phases on purpose) count. All outputs are
// monotonic counters or snapshots read through plain register stages by the caller (debug only).
//   bursts     bursts seen;  bursts_lg   bursts that followed at least LONG_GAP idle cycles
//   hist       saturating 8-bit counts {errors at pos 0, pos 1..3, pos >= 4, errors in a burst after a long gap}
//   last_err   {gap before the burst [15:0], pos [7:0], 3'b0, last, parity flags [3:0]} of the last error
//   last_word  the word (low word of a 64-bit beat) of the last error
//   fills, fill_perr   fill words received, of which with a parity error;  perrs   parity errors, real words
`default_nettype none
module link_rx_diag #(
    parameter int PW       = 2,                // parity flags per word / beat
    parameter int LONG_GAP = 1024
)(
    input  wire          lclk_rx, rst_rx,
    input  wire          en,                   // receiver trained (lclk_rx domain)
    input  wire          valid, last, fill,    // fill: this valid word is a fill word (dropped by the caller)
    input  wire [31:0]   word,
    input  wire [PW-1:0] perr,
    output reg  [31:0]   bursts, bursts_lg, hist, last_err, last_word, fills, fill_perr, perrs
);
    reg        prev_v;
    reg [15:0] gap, bgap;
    reg [7:0]  pos;
    wire       start = valid && !prev_v;
    wire [7:0] cur_pos = start ? 8'd0 : pos;
    wire [15:0] cur_gap = start ? gap : bgap;
    function automatic [7:0] sat_inc(input [7:0] v); sat_inc = (v == 8'hFF) ? v : v + 8'd1; endfunction
    always @(posedge lclk_rx) begin
        if (rst_rx) begin
            prev_v <= 1'b0; gap <= 0; bgap <= 0; pos <= 0;
            bursts <= 0; bursts_lg <= 0; hist <= 0; last_err <= 0; last_word <= 0; fills <= 0; fill_perr <= 0; perrs <= 0;
        end else if (!en) begin
            prev_v <= 1'b0; gap <= 0;
        end else begin
            prev_v <= valid;
            if (valid) begin
                gap <= 0;
                pos <= (cur_pos == 8'hFF) ? cur_pos : cur_pos + 8'd1;
                if (start) begin
                    bgap <= gap; bursts <= bursts + 1;
                    if (gap >= 16'(LONG_GAP)) bursts_lg <= bursts_lg + 1;
                end
                if (fill) begin
                    fills <= fills + 1;
                    if (|perr) fill_perr <= fill_perr + 1;
                end else if (|perr) begin
                    perrs <= perrs + 1;
                    last_err <= {cur_gap, cur_pos, 3'b000, last, 4'(perr)};
                    last_word <= word;
                    if (cur_pos == 0)      hist[31:24] <= sat_inc(hist[31:24]);
                    else if (cur_pos < 4)  hist[23:16] <= sat_inc(hist[23:16]);
                    else                   hist[15:8]  <= sat_inc(hist[15:8]);
                    if (cur_gap >= 16'(LONG_GAP)) hist[7:0] <= sat_inc(hist[7:0]);
                end
            end else if (gap != 16'hFFFF) gap <= gap + 16'd1;
        end
    end
endmodule
`default_nettype wire
