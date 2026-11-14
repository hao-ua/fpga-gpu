// 10:1 TMDS serializer for Artix-7 using paired OSERDESE2 (master+slave) in DDR mode.
// Serial rate = 5x pixel clock using DDR on serial_clk_5x.
// Simulation only: with TMDS_WORD_SIM defined the OSERDES pair is left out and the pads stay low; the bench's
// tmds_capture then takes the 10-bit words at the pixel clock (word-level runs, sim/tmds_capture.sv).
`timescale 1ns/1ps
module tmds_serializer (
    input  wire        pixel_clk,
    input  wire        serial_clk_5x,
    input  wire        rst_n,
    input  wire [9:0]  data,
    output wire        tmds_p,
    output wire        tmds_n
);
`ifdef TMDS_WORD_SIM
    assign tmds_p = 1'b0;
    assign tmds_n = 1'b1;
`else
    wire serdes_shift1, serdes_shift2;
    wire serial_out;

    OSERDESE2 #(
        .DATA_RATE_OQ   ("DDR"),
        .DATA_RATE_TQ   ("SDR"),
        .DATA_WIDTH     (10),
        .SERDES_MODE    ("MASTER"),
        .TRISTATE_WIDTH (1)
    ) u_oserdes_master (
        .OQ       (serial_out),
        .OFB      (),
        .TQ       (),
        .TFB      (),
        .SHIFTOUT1(), .SHIFTOUT2(),
        .TBYTEOUT (),
        .CLK      (serial_clk_5x),
        .CLKDIV   (pixel_clk),
        .D1       (data[0]),
        .D2       (data[1]),
        .D3       (data[2]),
        .D4       (data[3]),
        .D5       (data[4]),
        .D6       (data[5]),
        .D7       (data[6]),
        .D8       (data[7]),
        .RST      (~rst_n),
        .SHIFTIN1 (serdes_shift1),
        .SHIFTIN2 (serdes_shift2),
        .T1       (1'b0), .T2(1'b0), .T3(1'b0), .T4(1'b0),
        .TCE      (1'b0),
        .OCE      (1'b1),
        .TBYTEIN  (1'b0)
    );

    OSERDESE2 #(
        .DATA_RATE_OQ   ("DDR"),
        .DATA_RATE_TQ   ("SDR"),
        .DATA_WIDTH     (10),
        .SERDES_MODE    ("SLAVE"),
        .TRISTATE_WIDTH (1)
    ) u_oserdes_slave (
        .OQ       (),
        .OFB      (),
        .TQ       (),
        .TFB      (),
        .SHIFTOUT1(serdes_shift1),
        .SHIFTOUT2(serdes_shift2),
        .TBYTEOUT (),
        .CLK      (serial_clk_5x),
        .CLKDIV   (pixel_clk),
        .D1       (1'b0),
        .D2       (1'b0),
        .D3       (data[8]),
        .D4       (data[9]),
        .D5       (1'b0),
        .D6       (1'b0),
        .D7       (1'b0),
        .D8       (1'b0),
        .RST      (~rst_n),
        .SHIFTIN1 (1'b0),
        .SHIFTIN2 (1'b0),
        .T1       (1'b0), .T2(1'b0), .T3(1'b0), .T4(1'b0),
        .TCE      (1'b0),
        .OCE      (1'b1),
        .TBYTEIN  (1'b0)
    );

    OBUFDS #(
        .IOSTANDARD("TMDS_33"),
        .SLEW("FAST")
    ) u_obufds (
        .O (tmds_p),
        .OB(tmds_n),
        .I (serial_out)
    );
`endif
endmodule
