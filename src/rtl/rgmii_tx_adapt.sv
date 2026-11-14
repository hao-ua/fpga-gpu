// rgmii_tx_adapt — speed-adaptive RGMII TX serializer (pure RTL, testable).
// Consumes GMII bytes through a FWFT handshake (valid/data + adv pop) and drives
// the ODDR data inputs for 10/100/1000:
//   * 1000: one byte per clock -> D1=low nibble (rising), D2=high nibble (falling)
//   * 10/100 (MII): one nibble per clock over two clocks; the nibble is duplicated
//     on both ODDR edges (D1=D2), low nibble first then high. adv pops on the 2nd.
// TX_CTL carries TXEN (rising) / TXEN^TXERR (falling); no error => both = valid.
`default_nettype none
module rgmii_tx_adapt (
    input  wire       clk,            // tx line clock (2.5/25/125 MHz)
    input  wire       rst,
    input  wire [1:0] speed,          // 00=10M,01=100M,10=1000M
    input  wire       valid,          // a GMII byte is available (FIFO ~empty)
    input  wire [7:0] data,           // current GMII byte (FWFT front)
    output wire       adv,            // pop: upstream presents next byte next clk
    output wire [3:0] txd_d1, txd_d2, // ODDR data: rising / falling nibble
    output wire       ctl_d1, ctl_d2  // ODDR TX_CTL: rising / falling
);
    wire       giga = (speed == 2'b10);
    reg        phase;                 // MII nibble phase (0=low,1=high)
    wire [3:0] nib  = phase ? data[7:4] : data[3:0];

    assign txd_d1 = giga ? data[3:0] : nib;
    assign txd_d2 = giga ? data[7:4] : nib;     // 1000: high nibble; 10/100: duplicate
    assign ctl_d1 = valid;
    assign ctl_d2 = valid;
    assign adv    = giga ? valid : (valid & phase);

    always @(posedge clk)
        if (rst)               phase <= 1'b0;
        else if (!giga & valid) phase <= ~phase;
        else                    phase <= 1'b0;
endmodule
`default_nettype wire
