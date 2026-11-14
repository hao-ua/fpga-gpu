// link_clk — board-link transmit clocks for the board designs (as link_test_top):
// 200 MHz (BUFG) -> MMCM (VCO 1000 MHz) -> lclk = LINK_MHZ, lclk90 / lclk270 = the same at 90 / 270 degrees.
// rst_lclk: released synchronously to lclk after LOCKED and the user reset.
// OFF 1 (the one-board design, no lanes): no MMCM or buffers; the clocks stay low, locked low, rst_lclk high.
`default_nettype none
module link_clk #(
    parameter int LINK_MHZ = 100,
    parameter bit OFF      = 1'b0
)(
    input  wire clk200,
    input  wire rst,                 // asynchronous, active high
    output wire lclk, lclk90, lclk270,
    output wire rst_lclk,
    output wire locked
);
    `include "link_funcs.svh"
    localparam int D   = link_div(LINK_MHZ);
    localparam int VCO = LINK_MHZ * D;
    generate if (OFF) begin : g_off
        assign {lclk, lclk90, lclk270, locked} = 4'b0000;
        assign rst_lclk = 1'b1;
    end else begin : g_on
        wire fb, fb_buf, l0, l90, l270;
        MMCME2_BASE #(
            .BANDWIDTH("OPTIMIZED"), .CLKIN1_PERIOD(5.000), .DIVCLK_DIVIDE(1),
            .CLKFBOUT_MULT_F(VCO / 200.0),
            .CLKOUT0_DIVIDE_F(D * 1.0), .CLKOUT0_PHASE(0.0),
            .CLKOUT1_DIVIDE(D),         .CLKOUT1_PHASE(90.0),     // forwarded clock launch
            .CLKOUT3_DIVIDE(D),         .CLKOUT3_PHASE(270.0),    // control-line launch (link_phy_tx)
            .REF_JITTER1(0.010), .STARTUP_WAIT("FALSE")
        ) u_mmcm (
            .CLKIN1(clk200), .CLKFBIN(fb_buf), .CLKFBOUT(fb), .CLKFBOUTB(),
            .CLKOUT0(l0), .CLKOUT0B(), .CLKOUT1(l90), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(),
            .CLKOUT3(l270), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
            .LOCKED(locked), .PWRDWN(1'b0), .RST(1'b0));
        BUFG u_bfb (.I(fb),   .O(fb_buf));
        BUFG u_b0  (.I(l0),   .O(lclk));
        BUFG u_b1  (.I(l90),  .O(lclk90));
        BUFG u_b3  (.I(l270), .O(lclk270));
        rst_sync #(.STAGES(3)) u_rst (.clk(lclk), .arst(rst | ~locked), .rst(rst_lclk));
    end endgenerate
endmodule
`default_nettype wire
