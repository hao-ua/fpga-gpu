// jtag_ctrl_reg — a W-bit command register written over JTAG USER2 (BSCANE2 JTAG_CHAIN 2), so a Tcl script
// (tools/b_jtag.tcl) can send commands to the board without UART or Ethernet.
// A command is one DR scan of exactly W + 16 bits (LSB first): {SIG[15:0], command[W-1:0]}. It is accepted at
// Update-DR only when the scan had exactly that length and carried the signature, because hw_server scans every
// USER chain while it looks for debug cores (seen on hardware: 275 such scans while programming). An accepted
// command is handed to the clk domain as a one-cycle cmd_valid with cmd_data (toggle handshake; the value only
// changes at the next accepted scan, microseconds later). Capture-DR loads the last accepted scan, so every
// scan also reads back the previous command.
`default_nettype none
module jtag_ctrl_reg #(
    parameter int        W   = 64,
    parameter bit [15:0] SIG = 16'hB5C3
)(
    input  wire         clk, rst,
    output reg          cmd_valid,
    output reg  [W-1:0] cmd_data
);
    localparam int SW = W + 16;
    wire capture, drck, sel, shift, tdi, update, tck, tms, runtest, reset, tdo;
    BSCANE2 #(.JTAG_CHAIN(2)) u_bscan (
        .CAPTURE(capture), .DRCK(drck), .RESET(reset), .RUNTEST(runtest), .SEL(sel),
        .SHIFT(shift), .TCK(tck), .TDI(tdi), .TMS(tms), .UPDATE(update), .TDO(tdo));

    // TCK domain: the TAP leaves Capture-DR / Shift-DR / Update-DR on a rising TCK edge, while the matching
    // output is still high
    reg [SW-1:0] sr = '0, held = '0;
    reg [7:0]    cnt = 8'd0;                     // bits shifted since Capture-DR (saturating)
    reg          tog = 1'b0;
    always @(posedge tck) begin
        if (sel && capture) begin sr <= held; cnt <= 8'd0; end
        else if (sel && shift) begin sr <= {tdi, sr[SW-1:1]}; if (cnt != 8'hFF) cnt <= cnt + 8'd1; end
        if (sel && update && cnt == 8'(SW) && sr[SW-1 -: 16] == SIG) begin held <= sr; tog <= ~tog; end
    end
    assign tdo = sr[0];

    (* ASYNC_REG = "TRUE" *) reg [2:0] ts = 3'b000;
    always @(posedge clk) begin
        ts <= {ts[1:0], tog};
        cmd_valid <= ~rst & (ts[2] ^ ts[1]);
        if (ts[2] ^ ts[1]) cmd_data <= held[W-1:0];
    end
endmodule
`default_nettype wire
