// link_status_jtag — exposes a status snapshot through the JTAG USER1 register (BSCANE2), so the
// counters can be read from Vivado with scan_ir_hw_jtag / scan_dr_hw_jtag (tools/link_status_jtag.tcl)
// without an ILA and without the UART cable.
//
// Shifted-out frame (LSB first = first bit on TDO):  { MAGIC[7:0], seq[7:0], fields[FW-1:0], seq[7:0] }
// `fields` is re-snapshotted in the user clock domain every SNAP_CYCLES; the two copies of `seq`
// let the reader detect a torn read (snapshot changed during the shift) and simply read again.
//   LEAN 0: the frame is copied into an N-bit shift register at CAPTURE. LEAN 1: the same frame, bit p of it read
//   through a bit counter and a multiplexer (about a quarter of the LUTs, no second copy); the snapshot is not
//   refreshed while the register is captured and shifted (SEL and CAPTURE / SHIFT, brought into the user clock), so
//   the bits stay put. A snapshot landing as the hold starts shows as two different seq copies (the reader reads again).
`default_nettype none
module link_status_jtag #(
    parameter int FW          = 436,
    parameter int SNAP_CYCLES = 1_000_000,
    parameter bit LEAN        = 1'b0
)(
    input  wire          clk, rst,
    input  wire [FW-1:0] fields
);
    localparam int N = 8 + 8 + FW + 8;
    localparam [7:0] MAGIC = 8'hA5;

    wire capture, drck, sel, shift, tdi, update, tck, tms, runtest, reset;
    wire tdo;
    BSCANE2 #(.JTAG_CHAIN(1)) u_bscan (
        .CAPTURE(capture), .DRCK(drck), .RESET(reset), .RUNTEST(runtest), .SEL(sel),
        .SHIFT(shift), .TCK(tck), .TDI(tdi), .TMS(tms), .UPDATE(update), .TDO(tdo));

    // LEAN: a read in progress (Capture-DR, then the shift) holds the snapshot (two flops into the user clock)
    (* ASYNC_REG = "TRUE" *) reg [1:0] hold_s = 2'b00;
    always @(posedge clk) hold_s <= {hold_s[0], LEAN & sel & (capture | shift)};

    // user-domain snapshot
    reg [FW-1:0] snap = 0;
    reg [7:0]    seq = 0;
    reg [31:0]   t = 0;
    // the snapshot enable is a register a cycle ahead of the compare (timing at 120 MHz: the `t == SNAP_CYCLES - 1`
    // compare would drive the clock enable of the FW snapshot flops directly, and they sit beside their sources all over
    // the die); the snapshot lands on the cycle the compare names
    (* max_fanout = 64 *) reg snap_en = 1'b0;
    always @(posedge clk) begin
        if (rst) begin t <= 0; snap_en <= 1'b0; end
        else begin
            snap_en <= (t == SNAP_CYCLES - 2) && !hold_s[1];
            if (t == SNAP_CYCLES - 1) t <= 0; else t <= t + 1;
        end
        if (snap_en) begin snap <= fields; seq <= seq + 8'd1; end
    end

    wire [N-1:0] frame = {seq, snap, seq, MAGIC};
    generate if (LEAN) begin : g_lean
        // bit p of the frame next: the snapshot is read asynchronously, held still while shifting. The first seq copy is
        // the one at CAPTURE (latched), the rest is read live: a snapshot that lands before the hold takes effect (two
        // user clocks) shows as two different seq copies, as with LEAN 0
        reg [$clog2(N+1)-1:0] p = 0;
        reg tdo_r = 1'b0; reg [7:0] sq0 = 8'd0;
        always @(posedge drck) begin
            if (capture)    begin tdo_r <= frame[0]; p <= 1; sq0 <= seq; end
            else if (shift) begin
                tdo_r <= (p >= 8 && p < 16) ? sq0[p - 8] : (p < N) ? frame[p] : 1'b0;
                p <= (p < N) ? p + 1'b1 : p;
            end
        end
        assign tdo = tdo_r;
        wire _unused = &{1'b0, tdi};
    end else begin : g_sr
        // shift register in the DRCK domain; the snapshot is read asynchronously at CAPTURE (the two seq
        // copies catch a change in flight)
        reg [N-1:0] sr = 0;
        always @(posedge drck) begin
            if (capture)    sr <= frame;
            else if (shift) sr <= {tdi, sr[N-1:1]};
        end
        assign tdo = sr[0];
    end endgenerate
endmodule
`default_nettype wire
