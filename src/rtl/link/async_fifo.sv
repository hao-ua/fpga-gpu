// async_fifo — dual-clock FIFO, gray-coded pointers (Cummings style), LUTRAM storage,
// first-word-fall-through read side (rdata is valid whenever !rempty; ren pops).
// wcount = fill level as seen from the write side (conservative: never under-reports).
// wfull and rempty are REGISTERS (timing at 120 MHz: the pointer compares would head the deframers' and the
// dispatcher's paths, 9 levels across the die): each is the compare of its side's next pointer with the synchronised
// other pointer as it stands now. Conservative by at most one cycle (the other side can only have moved to make the FIFO
// less full / less empty by the time the flag is read), never optimistic; the synchroniser stages are not shortened.
`default_nettype none
module async_fifo #(
    parameter int W  = 36,
    parameter int AW = 9            // depth = 2**AW
)(
    input  wire          wclk, wrst,
    input  wire          wen,
    input  wire [W-1:0]  wdata,
    output wire          wfull,
    output wire [AW:0]   wcount,
    input  wire          rclk, rrst,
    input  wire          ren,
    output wire [W-1:0]  rdata,
    output wire          rempty
);
    reg [W-1:0] mem [0:(1<<AW)-1];
    reg  [AW:0] wptr_gray = 0, rptr_gray = 0;      // the two pointers that cross domains

    // ---- write side ----
    reg  [AW:0] wptr_bin = 0;
    (* ASYNC_REG = "TRUE" *) reg [AW:0] rptr_gray_w1 = 0, rptr_gray_w2 = 0;   // synced read pointer
    wire [AW:0] wptr_bin_next = wptr_bin + (wen & ~wfull);
    wire [AW:0] wptr_gray_next = wptr_bin_next ^ (wptr_bin_next >> 1);
    // gray -> binary of the synced read pointer
    reg  [AW:0] rptr_bin_w;
    integer i;
    always @(*) begin
        rptr_bin_w[AW] = rptr_gray_w2[AW];
        for (i = AW-1; i >= 0; i = i - 1) rptr_bin_w[i] = rptr_bin_w[i+1] ^ rptr_gray_w2[i];
    end
    assign wcount = wptr_bin - rptr_bin_w;
    reg wfull_r = 0;
    assign wfull  = wfull_r;
    always @(posedge wclk) begin
        if (wrst) begin wptr_bin <= 0; wptr_gray <= 0; rptr_gray_w1 <= 0; rptr_gray_w2 <= 0; wfull_r <= 0; end
        else begin
            rptr_gray_w1 <= rptr_gray; rptr_gray_w2 <= rptr_gray_w1;
            if (wen & ~wfull) mem[wptr_bin[AW-1:0]] <= wdata;
            wptr_bin  <= wptr_bin_next;
            wptr_gray <= wptr_gray_next;
            wfull_r   <= (wptr_gray_next == {~rptr_gray_w2[AW:AW-1], rptr_gray_w2[AW-2:0]});
        end
    end

    // ---- read side ----
    reg  [AW:0] rptr_bin = 0;
    (* ASYNC_REG = "TRUE" *) reg [AW:0] wptr_gray_r1 = 0, wptr_gray_r2 = 0;
    wire [AW:0] rptr_bin_next  = rptr_bin + (ren & ~rempty);
    wire [AW:0] rptr_gray_next = rptr_bin_next ^ (rptr_bin_next >> 1);
    reg rempty_r = 1;
    assign rempty = rempty_r;
    assign rdata  = mem[rptr_bin[AW-1:0]];
    always @(posedge rclk) begin
        if (rrst) begin rptr_bin <= 0; rptr_gray <= 0; wptr_gray_r1 <= 0; wptr_gray_r2 <= 0; rempty_r <= 1; end
        else begin
            wptr_gray_r1 <= wptr_gray; wptr_gray_r2 <= wptr_gray_r1;
            rptr_bin  <= rptr_bin_next;
            rptr_gray <= rptr_gray_next;
            rempty_r  <= (rptr_gray_next == wptr_gray_r2);
        end
    end
endmodule
`default_nettype wire
