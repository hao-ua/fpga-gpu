// afifo_bram — dual-clock FIFO in block RAM, gray-coded pointers, first-word-fall-through read side,
// with fill counts on both sides. Asynchronous-assert resets so a side whose clock is stopped (the PHY
// receive clock while the link is down) still resets its pointers.
//   wcount: words held as seen from the write side (conservative: never under-reports free space
//           incorrectly — the word sitting in the read-side output register is already freed).
//   rcount: words readable as seen from the read side, including the word in the output register.
// Capacity = 2**AW words in RAM + 1 in the output register.
`default_nettype none
module afifo_bram #(
    parameter int W  = 36,
    parameter int AW = 12
)(
    input  wire          wclk, wrst,
    input  wire          wen,
    input  wire [W-1:0]  wdata,
    output wire          wfull,
    output wire [AW:0]   wcount,
    input  wire          rclk, rrst,
    input  wire          ren,          // pop the current word (only when rvalid)
    output wire [W-1:0]  rdata,
    output wire          rvalid,
    output wire [AW:0]   rcount
);
    (* ram_style = "block" *) reg [W-1:0] mem [0:(1<<AW)-1];

    function automatic [AW:0] g2b(input [AW:0] g);
        integer i; begin g2b[AW] = g[AW]; for (i = AW-1; i >= 0; i = i - 1) g2b[i] = g2b[i+1] ^ g[i]; end
    endfunction

    // ---------------- write side ----------------
    reg  [AW:0] wbin = 0, wgray = 0;
    (* ASYNC_REG = "TRUE" *) reg [AW:0] rg_w1 = 0, rg_w2 = 0;
    reg  [AW:0] rgray = 0;
    wire        we_ok   = wen & ~wfull;
    wire [AW:0] wbin_n  = wbin + we_ok;
    always @(posedge wclk or posedge wrst) begin
        if (wrst) begin wbin <= 0; wgray <= 0; rg_w1 <= 0; rg_w2 <= 0; end
        else begin
            rg_w1 <= rgray; rg_w2 <= rg_w1;
            wbin <= wbin_n; wgray <= (wbin_n >> 1) ^ wbin_n;
        end
    end
    always @(posedge wclk) if (we_ok) mem[wbin[AW-1:0]] <= wdata;
    assign wfull  = (wgray == {~rg_w2[AW:AW-1], rg_w2[AW-2:0]});
    // the read pointer in binary, one more register stage (timing: the gray decode in front of the Ethernet receiver's
    // free-space compare does not fit 125 MHz); the count then lags one cycle on the read side, which only
    // over-reports the words held — still conservative
    reg  [AW:0] rbin_w = 0;
    always @(posedge wclk or posedge wrst) if (wrst) rbin_w <= 0; else rbin_w <= g2b(rg_w2);
    assign wcount = wbin - rbin_w;

    // ---------------- read side (prefetch into an output register) ----------------
    reg  [AW:0] rbin = 0;
    (* ASYNC_REG = "TRUE" *) reg [AW:0] wg_r1 = 0, wg_r2 = 0;
    reg  [W-1:0] dout;
    reg          dvalid = 1'b0;
    wire         fempty = (rgray == wg_r2);
    wire         rden   = ~fempty & (~dvalid | ren);
    wire [AW:0]  rbin_n = rbin + rden;
    always @(posedge rclk or posedge rrst) begin
        if (rrst) begin rbin <= 0; rgray <= 0; wg_r1 <= 0; wg_r2 <= 0; dvalid <= 1'b0; end
        else begin
            wg_r1 <= wgray; wg_r2 <= wg_r1;
            rbin <= rbin_n; rgray <= (rbin_n >> 1) ^ rbin_n;
            if (rden) dvalid <= 1'b1; else if (ren) dvalid <= 1'b0;
        end
    end
    always @(posedge rclk) if (rden) dout <= mem[rbin[AW-1:0]];
    assign rdata  = dout;
    assign rvalid = dvalid;
    assign rcount = (g2b(wg_r2) - rbin) + dvalid;
endmodule
`default_nettype wire
