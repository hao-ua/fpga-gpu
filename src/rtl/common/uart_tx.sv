// uart_tx — minimal 8N1 UART transmitter. DIV = clk / baud.
`default_nettype none
module uart_tx #(parameter int DIV = 868) (
    input  wire       clk, rst,
    input  wire [7:0] data,
    input  wire       start,
    output reg        busy,
    output reg        txd
);
    reg [15:0] baud_cnt;
    reg [3:0]  bit_idx;
    reg [9:0]  shifter;      // {stop, data[7:0], start}
    always @(posedge clk) begin
        if (rst) begin busy <= 0; txd <= 1'b1; baud_cnt <= 0; bit_idx <= 0; shifter <= 10'h3FF; end
        else if (!busy) begin
            txd <= 1'b1;
            if (start) begin busy <= 1'b1; shifter <= {1'b1, data, 1'b0}; baud_cnt <= 0; bit_idx <= 0; end
        end else begin
            txd <= shifter[0];
            if (baud_cnt == DIV - 1) begin
                baud_cnt <= 0;
                shifter  <= {1'b1, shifter[9:1]};
                if (bit_idx == 9) busy <= 1'b0; else bit_idx <= bit_idx + 1;
            end else baud_cnt <= baud_cnt + 1;
        end
    end
endmodule
`default_nettype wire
