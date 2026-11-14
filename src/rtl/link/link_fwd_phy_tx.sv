// link_fwd_phy_tx — transmit side of the 40-wire forward lane (A -> B): all of JM1 plus JM2 bits
// [29:22]. One beat = 64 bits per LCLK: 32 DDR data wires carry the low
// word on the rising half and the high word on the falling half.
//
// Pin placement rule (from hardware measurements): the boards are stacked header to header (2x20 pins,
// 2.54 mm pitch), so pin k's nearest neighbours are k+-2 (same row) and k+-1 (other row). All four
// neighbours of a forwarded clock (pins 23, 24, 26, 27 around pin 25) carry quiet lines (UP, XOFF,
// LAST, VALID), never full-rate data or parity; a parity wire gets the quietest remaining spot.
//
// JM1 vector bit (pin):  [15:0] D[15:0] (5-20)   [16] D16 (21)   [17] PAR0 (22, parity of D[15:0] per half)
//   [18] LAST (23)   [19] UP (24)   [20] LCLK (25, MRCC)   [21] VALID_HI (26)   [22] VALID_LO (27)
//   [30:23] D[24:17] (28-32, 37-39)   [31] D25 (40)
// JM2 bits 29:22 -> j2x[7:0] (pins 27-32, 37, 38):  [0] XOFF (27, next to B's clock)   [6:1] D[31:26]
//   [7] PAR1 (38, parity of D[31:16] per half)
//
// Control lines (VALID_LO, VALID_HI, LAST, XOFF, UP) are launched a quarter period before their
// data beat from lclk270 (see link_phy_tx). Both headers are driven from the same BUFG clocks.
`default_nettype none
module link_fwd_phy_tx (
    input  wire        lclk, lclk90, lclk270,
    input  wire        oe,
    input  wire [31:0] lo, hi,          // beat: lo = first word (rising half), hi = second (falling half)
    input  wire        valid_lo, valid_hi, last,
    input  wire        xoff, up,
    output wire        lclk_sense,
    inout  wire [31:0] j1,
    inout  wire [7:0]  j2x
);
    wire t = ~oe;
    // 34 DDR outputs: D[31:0], PAR0, PAR1
    wire [33:0] d1 = {^lo[31:16], ^lo[15:0], lo};
    wire [33:0] d2 = {^hi[31:16], ^hi[15:0], hi};
    wire [33:0] q;
    genvar i;
    generate
        for (i = 0; i < 34; i = i + 1) begin : g_ddr
            ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_oddr (
                .Q(q[i]), .C(lclk), .CE(1'b1), .D1(d1[i]), .D2(d2[i]), .R(1'b0), .S(1'b0));
        end
        for (i = 0; i < 17; i = i + 1) begin : g_b0          // D[16:0] -> j1[16:0]
            IOBUF u_io (.I(q[i]), .T(t), .O(), .IO(j1[i]));
        end
        for (i = 17; i < 25; i = i + 1) begin : g_b1         // D[24:17] -> j1[30:23]
            IOBUF u_io (.I(q[i]), .T(t), .O(), .IO(j1[i + 6]));
        end
        for (i = 26; i < 32; i = i + 1) begin : g_b2         // D[31:26] -> j2x[6:1]
            IOBUF u_io (.I(q[i]), .T(t), .O(), .IO(j2x[i - 25]));
        end
    endgenerate
    IOBUF u_d25  (.I(q[25]), .T(t), .O(), .IO(j1[31]));
    IOBUF u_par0 (.I(q[32]), .T(t), .O(), .IO(j1[17]));
    IOBUF u_par1 (.I(q[33]), .T(t), .O(), .IO(j2x[7]));

    // control lines: ODDR on lclk270 with D1 == D2 (pin changes only at lclk270's rising edge)
    wire [4:0] ctl = {up, xoff, valid_hi, last, valid_lo};
    wire [4:0] q_ctl;
    generate
        for (i = 0; i < 5; i = i + 1) begin : g_ctl
            ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_oddr (
                .Q(q_ctl[i]), .C(lclk270), .CE(1'b1), .D1(ctl[i]), .D2(ctl[i]), .R(1'b0), .S(1'b0));
        end
    endgenerate
    IOBUF u_vlo (.I(q_ctl[0]), .T(t), .O(), .IO(j1[22]));
    IOBUF u_lst (.I(q_ctl[1]), .T(t), .O(), .IO(j1[18]));
    IOBUF u_vhi (.I(q_ctl[2]), .T(t), .O(), .IO(j1[21]));
    IOBUF u_xof (.I(q_ctl[3]), .T(t), .O(), .IO(j2x[0]));
    IOBUF u_up  (.I(q_ctl[4]), .T(t), .O(), .IO(j1[19]));

    // forwarded clock on JM1 pin 25; neighbours 23/24/26/27 = LAST/UP/VALID_HI/VALID_LO
    wire q_clk;
    ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_oddr_clk (
        .Q(q_clk), .C(lclk90), .CE(1'b1), .D1(1'b1), .D2(1'b0), .R(1'b0), .S(1'b0));
    IOBUF u_clk (.I(q_clk), .T(t), .O(lclk_sense), .IO(j1[20]));
endmodule
`default_nettype wire
