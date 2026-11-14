// link_status_uart — prints one status line per `tick`. The template is a packed string in
// which every '#' consumes the next 4 bits (MSB first) of the `fields` vector as a hex digit.
`default_nettype none
module link_status_uart #(
    parameter int  UART_DIV = 868,
    parameter int  TLEN     = 160,                  // template bytes (zero bytes are skipped)
    parameter [8*TLEN-1:0] TMPL = "",
    parameter int  FW       = 364                   // fields width in bits (multiple of 4)
)(
    input  wire          clk, rst,
    input  wire          tick,
    input  wire [FW-1:0] fields,
    output wire          txd
);
    `include "link_funcs.svh"
    reg  [7:0]  tx_byte; reg tx_start; wire tx_busy;
    uart_tx #(.DIV(UART_DIV)) u_uart (.clk(clk), .rst(rst), .data(tx_byte), .start(tx_start), .busy(tx_busy), .txd(txd));

    reg          sending;
    reg [15:0]   idx;        // template byte index
    reg [15:0]   nib;        // next nibble index into the snapshot
    reg [FW-1:0] snap;
    wire [7:0]   tchar = TMPL[8*(TLEN-1-idx) +: 8];
    wire [3:0]   nibble = snap[FW-1-4*nib -: 4];

    always @(posedge clk) begin
        if (rst) begin sending <= 0; idx <= 0; nib <= 0; tx_start <= 0; snap <= 0; tx_byte <= 0; end
        else begin
            tx_start <= 0;
            if (!sending) begin
                if (tick) begin sending <= 1; idx <= 0; nib <= 0; snap <= fields; end
            end else if (idx == TLEN) begin
                sending <= 0;
            end else if (tchar == 8'h00) begin
                idx <= idx + 1;                              // padding byte
            end else if (!tx_busy && !tx_start) begin
                tx_start <= 1;
                if (tchar == "#") begin tx_byte <= hex_digit(nibble); nib <= nib + 1; end
                else tx_byte <= tchar;
                idx <= idx + 1;
            end
        end
    end
endmodule
`default_nettype wire
