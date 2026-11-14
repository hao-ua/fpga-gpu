// link_rx_clock — recovered lane clock with run-time phase shift (shared by both RX PHYs).
//
// The forwarded LCLK goes through an MMCM in zero-delay (deskew) mode so that, nominally, the IDDRs
// sample at the pin edge; the trainer (link_rx_trainer) then moves CLKOUT0 with the MMCM's dynamic
// fine phase shift (1/56 of the VCO period per PSEN step) to the centre of the measured eye.
// A watchdog in the user clock domain resets the MMCM if it stays unlocked (far end silent, then
// returns — UG472 asks for a reset after the input clock was lost); a reset also returns the phase
// shifter to 0, which `mmcm_rst` tells the trainer.
`default_nettype none
module link_rx_clock #(
    parameter int  LINK_MHZ        = 50,
    parameter int  MMCM_RST_CYCLES = 1_000_000,    // clk cycles unlocked before an MMCM reset pulse
    parameter bit  SIM_SHIFTER     = 1'b0          // 1: behavioural shifter (sim/link_rx_shifter_sim.sv)
)(
    input  wire clk, rst,                          // user clock domain
    input  wire clkin,                             // forwarded lane clock from the pin (IOBUF O)
    input  wire psen, psincdec,                    // phase shift step request (clk domain)
    output wire psdone,
    output wire lclk_rx,                           // recovered clock (BUFG)
    output wire locked,                            // raw MMCM LOCKED
    output reg  rx_ok,                             // LOCKED synchronised into clk
    output reg  mmcm_rst
);
    `include "link_funcs.svh"
    localparam int  D   = link_div(LINK_MHZ);
    localparam int  VCO = LINK_MHZ * D;                      // MHz: 1000, or 900 for 150 MHz
    wire clk_fb, clk_fb_buf, lclk_rx_u;
    generate
        if (!SIM_SHIFTER) begin : g_mmcm
            MMCME2_ADV #(
                .BANDWIDTH("OPTIMIZED"), .COMPENSATION("ZHOLD"), .DIVCLK_DIVIDE(1),
                .CLKIN1_PERIOD(1000.0 / LINK_MHZ), .CLKIN2_PERIOD(10.0),
                .CLKFBOUT_MULT_F(D * 1.0), .CLKFBOUT_PHASE(0.0), .CLKFBOUT_USE_FINE_PS("FALSE"),
                .CLKOUT0_DIVIDE_F(D * 1.0), .CLKOUT0_PHASE(0.0), .CLKOUT0_DUTY_CYCLE(0.5),
                .CLKOUT0_USE_FINE_PS("TRUE"),
                .REF_JITTER1(0.050), .STARTUP_WAIT("FALSE")
            ) u_mmcm (
                .CLKIN1(clkin), .CLKIN2(1'b0), .CLKINSEL(1'b1),
                .CLKFBIN(clk_fb_buf), .CLKFBOUT(clk_fb), .CLKFBOUTB(),
                .CLKOUT0(lclk_rx_u), .CLKOUT0B(), .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(),
                .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
                .DADDR(7'd0), .DCLK(1'b0), .DEN(1'b0), .DI(16'd0), .DWE(1'b0), .DO(), .DRDY(),
                .PSCLK(clk), .PSEN(psen), .PSINCDEC(psincdec), .PSDONE(psdone),
                .CLKINSTOPPED(), .CLKFBSTOPPED(),
                .LOCKED(locked), .PWRDWN(1'b0), .RST(mmcm_rst));
        end else begin : g_sim
            link_rx_shifter_sim #(.VCO_MHZ(VCO)) u_sim (
                .clkin(clkin), .rst(mmcm_rst), .psclk(clk), .psen(psen), .psincdec(psincdec),
                .psdone(psdone), .locked(locked), .clkout(lclk_rx_u));
            assign clk_fb = 1'b0;
        end
    endgenerate
    BUFG u_bfb  (.I(clk_fb),    .O(clk_fb_buf));
    BUFG u_bufg (.I(lclk_rx_u), .O(lclk_rx));

    // lock watchdog (clk domain): hold RST while rst; otherwise pulse RST after MMCM_RST_CYCLES unlocked
    (* ASYNC_REG = "TRUE" *) reg [1:0] locked_s = 2'b00;
    reg [31:0] unl_cnt = 0;
    always @(posedge clk) begin
        locked_s <= {locked_s[0], locked};
        rx_ok    <= locked_s[1];
        if (rst) begin mmcm_rst <= 1'b1; unl_cnt <= 0; end
        else if (locked_s[1]) begin mmcm_rst <= 1'b0; unl_cnt <= 0; end
        else if (unl_cnt == MMCM_RST_CYCLES) begin mmcm_rst <= 1'b1; unl_cnt <= 0; end
        else begin mmcm_rst <= (unl_cnt < 16); unl_cnt <= unl_cnt + 1; end   // 16-cycle pulse after the counter wraps
    end
endmodule
`default_nettype wire
