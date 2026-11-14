// link_phy_tx — transmit side of the 22-wire backward lane (B -> A, JM2 bits [21:0]).
//
// Lane vector bit order (pin): quiet lines on all four header neighbours of the clock — pins 23,
// 24, 26 and 27 around pin 25 (stacked 2x20 headers; see link_fwd_phy_tx for the rule):
//   [15:0]  D[15:0]  (5-20) DDR data: rising-edge half = word[15:0], falling-edge half = word[31:16]
//   [16]    VALID    (21)
//   [17]    PAR      (22) DDR: rising = even parity of word[15:0], falling = even parity of word[31:16]
//   [18]    LAST     (23)
//   [19]    UP       (24) "my receiver is trained"
//   [20]    LCLK     (25) forwarded clock (ODDR on the 90-degree clock -> edges centred in the data eye)
//   [21]    XOFF     (26) flow control for the OPPOSITE lane
// (pin 27 = JM2 bit 22 is the forward lane's XOFF, driven by the other board — also quiet)
//
// Control lines are launched a quarter period BEFORE their data beat (rising edge of lclk270): the
// framer's registers update at lclk edge E-T, the data ODDR picks them up at E, the control ODDR
// at E-T/4. With the receiver's capture edge trained to the middle of the first data half (E+T/4)
// a control line is sampled in the middle of its own window (margins T/2 both ways) — the trainer
// never measures the control lines, so their margin must come from construction. The internal
// path register -> control ODDR is 3T/4 long (a 90-degree launch clock would leave only T/4).
//
// All pins are IOBUFs so the lane can stay tri-stated (T=1) until `oe`. While tri-stated the
// LCLK pin is read back (`lclk_sense`) so the core can detect a foreign driver.
`default_nettype none
module link_phy_tx (
    input  wire        lclk,        // data launch clock
    input  wire        lclk90,      // forwarded-clock launch clock (90 degrees later)
    input  wire        lclk270,     // control-line launch clock (270 degrees)
    input  wire        oe,          // output enable (lclk domain, registered by caller)
    input  wire [31:0] word,
    input  wire        valid,
    input  wire        last,
    input  wire        xoff,
    input  wire        up,
    output wire        lclk_sense,  // raw level on the LCLK pin (for the contention guard)
    inout  wire [21:0] pins
);
    wire t = ~oe;
    wire par_lo = ^word[15:0];
    wire par_hi = ^word[31:16];

    // data + parity: 17 DDR outputs
    wire [16:0] d1 = {par_lo, word[15:0]};
    wire [16:0] d2 = {par_hi, word[31:16]};
    wire [16:0] q_ddr;
    genvar i;
    generate
        for (i = 0; i < 17; i = i + 1) begin : g_ddr
            ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_oddr (
                .Q(q_ddr[i]), .C(lclk), .CE(1'b1), .D1(d1[i]), .D2(d2[i]), .R(1'b0), .S(1'b0));
        end
        for (i = 0; i < 16; i = i + 1) begin : g_dbuf
            IOBUF u_io (.I(q_ddr[i]), .T(t), .O(), .IO(pins[i]));
        end
    endgenerate
    IOBUF u_par (.I(q_ddr[16]), .T(t), .O(), .IO(pins[17]));

    // control lines: ODDR on lclk270 with D1 == D2, so the pin changes only at lclk270's rising edge
    wire [3:0] ctl = {xoff, up, last, valid};
    wire [3:0] q_ctl;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_ctl
            ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_oddr (
                .Q(q_ctl[i]), .C(lclk270), .CE(1'b1), .D1(ctl[i]), .D2(ctl[i]), .R(1'b0), .S(1'b0));
        end
    endgenerate
    IOBUF u_vld (.I(q_ctl[0]), .T(t), .O(), .IO(pins[16]));
    IOBUF u_lst (.I(q_ctl[1]), .T(t), .O(), .IO(pins[18]));
    IOBUF u_up  (.I(q_ctl[2]), .T(t), .O(), .IO(pins[19]));
    IOBUF u_xof (.I(q_ctl[3]), .T(t), .O(), .IO(pins[21]));

    // forwarded clock: rising edge 90 degrees after the data launch edge
    wire q_clk;
    ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_oddr_clk (
        .Q(q_clk), .C(lclk90), .CE(1'b1), .D1(1'b1), .D2(1'b0), .R(1'b0), .S(1'b0));
    IOBUF u_clk (.I(q_clk), .T(t), .O(lclk_sense), .IO(pins[20]));
endmodule
`default_nettype wire
