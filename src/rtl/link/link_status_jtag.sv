// link_status_jtag — exposes a status snapshot through the JTAG USER1 register (BSCANE2), so the
// counters can be read from Vivado with scan_ir_hw_jtag / scan_dr_hw_jtag (tools/link_status_jtag.tcl)
// without an ILA and without the UART cable.
//
// Shifted-out frame (LSB first = first bit on TDO):  { MAGIC[7:0], seq[7:0], fields[FW-1:0], seq[7:0] }
// `fields` is re-snapshotted in the user clock domain every SNAP_CYCLES; the two copies of `seq`
// let the reader detect a torn read (snapshot changed during the shift) and simply read again.
`default_nettype none
module link_status_jtag #(
    parameter int FW          = 436,
    parameter int SNAP_CYCLES = 1_000_000
)(
    input  wire          clk, rst,
    input  wire [FW-1:0] fields
);
    localparam int N = 8 + 8 + FW + 8;
    localparam [7:0] MAGIC = 8'hA5;

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
            snap_en <= (t == SNAP_CYCLES - 2);
            if (t == SNAP_CYCLES - 1) t <= 0; else t <= t + 1;
        end
        if (snap_en) begin snap <= fields; seq <= seq + 8'd1; end
    end

    wire capture, drck, sel, shift, tdi, update, tck, tms, runtest, reset;
    wire tdo;
    BSCANE2 #(.JTAG_CHAIN(1)) u_bscan (
        .CAPTURE(capture), .DRCK(drck), .RESET(reset), .RUNTEST(runtest), .SEL(sel),
        .SHIFT(shift), .TCK(tck), .TDI(tdi), .TMS(tms), .UPDATE(update), .TDO(tdo));

    // shift register in the DRCK domain; the snapshot is read asynchronously at CAPTURE (the two seq
    // copies catch a change in flight)
    reg [N-1:0] sr = 0;
    always @(posedge drck) begin
        if (capture)    sr <= {seq, snap, seq, MAGIC};
        else if (shift) sr <= {tdi, sr[N-1:1]};
    end
    assign tdo = sr[0];
endmodule
`default_nettype wire
