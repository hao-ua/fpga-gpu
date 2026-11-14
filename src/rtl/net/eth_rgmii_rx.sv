// eth_rgmii_rx — RGMII receive for 100 and 1000 Mbit/s.
// IDELAYE2 (VAR_LOAD, one common tap for RXD[3:0] and RX_CTL, loaded from clk_ctl at run time) +
// IDDR (SAME_EDGE_PIPELINED) capture, then byte assembly selected by `giga`, which comes from the PHY's
// PHYSR register via mdio_mgr — not from RGMII in-band status (unreliable on the RTL8211F).
//   1000: byte = {falling nibble, rising nibble} every clock (rxc = 125 MHz)
//   100 : the nibble is sent on both edges; two rising nibbles (low first) make a byte (rxc = 25 MHz)
// Output, rxc domain: dv = frame in progress, stb = data is a new byte this cycle, er = RX_ER seen.
`default_nettype none
module eth_rgmii_rx #(
    parameter int IDELAY_DEFAULT = 0
)(
    input  wire       clk_ctl,                 // IDELAY control clock (clk_sys)
    input  wire       dly_ld,                  // load dly_val into all five IDELAYs
    input  wire [4:0] dly_val,
    input  wire       rgmii_rxc, rgmii_rx_ctl,
    input  wire [3:0] rgmii_rxd,
    output wire       rxc,                     // BUFG'd receive clock
    input  wire       rst,                     // rxc domain, synchronous
    input  wire       giga,                    // rxc domain
    output reg        dv,
    output reg        stb,
    output reg  [7:0] data,
    output reg        er
);
    BUFG u_bufg (.I(rgmii_rxc), .O(rxc));

    wire [4:0] pin  = {rgmii_rx_ctl, rgmii_rxd};
    wire [4:0] dly;
    wire [4:0] q1, q2;
    genvar i;
    generate for (i = 0; i < 5; i = i + 1) begin : g_in
        (* IODELAY_GROUP = "rgmii_delay" *)
        IDELAYE2 #(.IDELAY_TYPE("VAR_LOAD"), .DELAY_SRC("IDATAIN"), .IDELAY_VALUE(IDELAY_DEFAULT),
                   .HIGH_PERFORMANCE_MODE("TRUE"), .SIGNAL_PATTERN("DATA"), .REFCLK_FREQUENCY(200.0),
                   .CINVCTRL_SEL("FALSE"), .PIPE_SEL("FALSE"))
          u_dly (.C(clk_ctl), .LD(dly_ld), .CNTVALUEIN(dly_val), .CE(1'b0), .INC(1'b0), .LDPIPEEN(1'b0),
                 .REGRST(1'b0), .CINVCTRL(1'b0), .DATAIN(1'b0), .IDATAIN(pin[i]),
                 .DATAOUT(dly[i]), .CNTVALUEOUT());
        IDDR #(.DDR_CLK_EDGE("SAME_EDGE_PIPELINED"), .INIT_Q1(1'b0), .INIT_Q2(1'b0), .SRTYPE("SYNC"))
          u_iddr (.C(rxc), .CE(1'b1), .D(dly[i]), .R(1'b0), .S(1'b0), .Q1(q1[i]), .Q2(q2[i]));
    end endgenerate

    wire       ctl_r = q1[4];                  // RX_DV
    wire       ctl_f = q2[4];                  // RX_DV ^ RX_ER
    wire [3:0] nib_r = q1[3:0];
    wire [3:0] nib_f = q2[3:0];

    reg       phase;
    reg [3:0] lo;
    always @(posedge rxc) begin
        if (rst) begin
            dv <= 1'b0; stb <= 1'b0; er <= 1'b0; data <= 8'h00; phase <= 1'b0; lo <= 4'h0;
        end else begin
            dv  <= ctl_r;
            stb <= 1'b0;
            er  <= ctl_r & (ctl_r ^ ctl_f);
            if (!ctl_r) phase <= 1'b0;
            else if (giga) begin
                data <= {nib_f, nib_r};
                stb  <= 1'b1;
            end else if (!phase) begin
                lo <= nib_r; phase <= 1'b1;
            end else begin
                data <= {nib_r, lo}; stb <= 1'b1; phase <= 1'b0;
            end
        end
    end
endmodule
`default_nettype wire
