// sfifo — single-clock FIFO, first-word-fall-through (prefetch output register), RAM style selectable.
// count = words held including the output register(s). Capacity 2**AW + 1 (+ 1 with OREG). full / count are registered
// (their values are exact every cycle: they are computed from the next state).
`default_nettype none
module sfifo #(
    parameter int    W  = 256,
    parameter int    AW = 9,
    parameter string RAM_STYLE = "block",     // "block" or "distributed"
    parameter bit    OREG = 1'b0              // 1: a second output register behind the RAM's own (the block RAM's latch then
                                              //    drives one flop instead of the reader's decode, e.g. the link port's
                                              //    command buffer -> fb_cmd_b at 120 MHz);
                                              //    the interface is unchanged, a word is readable one cycle later
)(
    input  wire         clk, rst,
    input  wire         wen,
    input  wire [W-1:0] wdata,
    output wire         full,
    output wire [AW+1:0] count,
    input  wire         ren,
    output wire [W-1:0] rdata,
    output wire         rvalid
);
    reg  [AW:0] wp, rp;
    reg  [W-1:0] dout;
    reg          dv;
    wire         empty = (wp == rp);
    // full and count are REGISTERS of their next-cycle values (timing at 120 MHz: `wp - rp` -> compare -> the reader's or
    // writer's decision would head long paths — the DDR ring FIFO's read request into the memory arbiter, the link port's
    // wipe count, img_write's buffer ready; cycle-exact: the same values one gate earlier)
    reg          full_r;
    reg  [AW+1:0] cnt_r;                       // words written and not yet read out (the RAM's, dout's and OREG's)
    assign full  = full_r;
    assign count = cnt_r;
    wire         we_ok = wen & ~full;
    wire         take1;                        // the RAM's output word leaves: to the reader, or (OREG) into the second register
    wire         rden  = ~empty & (~dv | take1);
    wire [AW:0]  held_n = (wp + (AW+1)'(we_ok)) - (rp + (AW+1)'(rden));   // sized: the difference must wrap at AW+1 bits
    always @(posedge clk) begin
        if (rst) begin wp <= 0; rp <= 0; dv <= 1'b0; full_r <= 1'b0; cnt_r <= '0; end
        else begin
            if (we_ok) wp <= wp + 1'b1;
            if (rden)  rp <= rp + 1'b1;
            if (rden) dv <= 1'b1; else if (take1) dv <= 1'b0;
            full_r <= held_n[AW];
            cnt_r  <= cnt_r + (AW+2)'(we_ok) - (AW+2)'(ren & rvalid);
        end
    end
    // (Vivado synthesis does not accept a parameter inside the ram_style attribute: two fixed variants)
    generate if (RAM_STYLE == "distributed") begin : g_lut
        (* ram_style = "distributed" *) reg [W-1:0] mem [0:(1<<AW)-1];
        always @(posedge clk) begin
            if (we_ok) mem[wp[AW-1:0]] <= wdata;
            if (rden)  dout <= mem[rp[AW-1:0]];
        end
    end else begin : g_bram
        (* ram_style = "block" *) reg [W-1:0] mem [0:(1<<AW)-1];
        always @(posedge clk) begin
            if (we_ok) mem[wp[AW-1:0]] <= wdata;
            if (rden)  dout <= mem[rp[AW-1:0]];
        end
    end endgenerate
    generate if (OREG) begin : g_oreg
        reg [W-1:0] q; reg qv;
        assign take1 = dv & (~qv | ren);
        always @(posedge clk) begin
            if (rst) qv <= 1'b0;
            else if (take1) qv <= 1'b1; else if (ren) qv <= 1'b0;
            if (take1) q <= dout;
        end
        assign rdata  = q;
        assign rvalid = qv;
    end else begin : g_noreg
        assign take1  = ren;
        assign rdata  = dout;
        assign rvalid = dv;
    end endgenerate
endmodule
`default_nettype wire
