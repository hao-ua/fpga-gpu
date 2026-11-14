// link_rx_shifter_sim — behavioural clock shifter for simulation (link_rx_clock, SIM_SHIFTER=1).
// Same interface as the MMCME2_ADV fine-phase-shift usage in link_rx_clock: one PSEN pulse moves the
// output clock by 1/56 of the VCO period in the PSINCDEC direction, PSDONE answers 12 PSCLK cycles
// later, LOCKED rises after a few input edges, RST returns the position to 0. It does not lose lock
// on input phase steps (the UNISIM model does) and runs much faster. Not synthesisable.
`timescale 1ns/1ps
module link_rx_shifter_sim #(parameter int VCO_MHZ = 1000) (
    input  wire clkin, rst, psclk, psen, psincdec,
    output reg  psdone = 0,
    output reg  locked = 0,
    output wire clkout
);
    localparam real STEP_NS  = 1000.0 / VCO_MHZ / 56.0;
    localparam int  LOCK_CYC = 64;
    integer   pos = 0;
    integer   lock_cnt = 0;
    reg [3:0] ps_tmr = 0;
    real      dly = 0.3;
    reg       clk_d = 0;
    always @(posedge psclk or posedge rst) begin
        if (rst) begin pos <= 0; psdone <= 0; ps_tmr <= 0; end
        else begin
            psdone <= 0;
            if (psen) begin pos <= psincdec ? pos + 1 : pos - 1; ps_tmr <= 4'd12; end
            else if (ps_tmr != 0) begin ps_tmr <= ps_tmr - 1; if (ps_tmr == 1) psdone <= 1; end
        end
    end
    always @(pos) dly = 0.3 + pos * STEP_NS;     // small fixed part keeps the delay positive
    always @(clkin) clk_d <= #(dly) clkin;
    assign clkout = clk_d;
    always @(posedge clkin or posedge rst) begin
        if (rst) begin lock_cnt <= 0; locked <= 0; end
        else if (lock_cnt == LOCK_CYC) locked <= 1;
        else lock_cnt <= lock_cnt + 1;
    end
endmodule
