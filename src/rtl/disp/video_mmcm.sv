// video_mmcm — board B video clocks from ONE MMCME2_ADV, reconfigured per video mode through its DRP port
// (Xilinx XAPP888 method): hold RST, read-modify-write the 23
// registers of vm_drp() (new = (old & keep_mask) | data), release RST, wait for LOCKED.
//   CLKOUT0 -> BUFG = ser_clk (5 x pixel, the OSERDES DDR 10:1 clock), CLKOUT1 -> BUFG = pix_clk,
//   CLKFBOUT -> BUFG -> CLKFBIN.
// The static attributes are mode VM_STATIC_MODE, the fastest one, so Vivado times the video domain for the
// worst case; the display controller reprograms the default mode after power-up.
// DRP clock = clk (clk_sys, 100 MHz). LOCKED is synchronised into clk; the raw LOCKED goes to the
// pixel-domain reset. A change that does not lock within LOCK_TIMEOUT cycles ends with ok = 0.
`default_nettype none
module video_mmcm #(
    parameter int LOCK_TIMEOUT = 2_000_000          // 20 ms at 100 MHz
)(
    input  wire       clk, rst,
    input  wire       clkin,                       // 200 MHz (BUFG)
    input  wire       req,                         // pulse: reconfigure to req_mode (ignored while busy)
    input  wire [2:0] req_mode,
    output reg        busy,
    output reg        done,                        // pulse at the end of a reconfiguration
    output reg        ok,                          // last reconfiguration locked
    output wire       locked,                      // LOCKED, synchronised to clk
    output wire       locked_raw,                  // LOCKED, asynchronous
    output wire       pix_clk, ser_clk
);
`include "video_modes.svh"

    // ---------------- MMCM ----------------
    wire        clkfb_out, clkfb_in, clk0, clk1;
    reg         mmcm_rst = 1'b0;
    reg  [6:0]  daddr = 7'h0;
    reg  [15:0] di = 16'h0;
    reg         den = 1'b0, dwe = 1'b0;
    wire [15:0] dout;
    wire        drdy, mmcm_locked;
    MMCME2_ADV #(
        .BANDWIDTH("OPTIMIZED"),
        .CLKIN1_PERIOD(5.000),
        .DIVCLK_DIVIDE(VM_STATIC_DIVCLK),
        .CLKFBOUT_MULT_F(VM_STATIC_MULT_F),
        .CLKFBOUT_PHASE(0.0),
        .CLKOUT0_DIVIDE_F(VM_STATIC_O0_F),
        .CLKOUT0_PHASE(0.0), .CLKOUT0_DUTY_CYCLE(0.5),
        .CLKOUT1_DIVIDE(VM_STATIC_O1),
        .CLKOUT1_PHASE(0.0), .CLKOUT1_DUTY_CYCLE(0.5),
        .REF_JITTER1(0.010),
        .STARTUP_WAIT("FALSE")
    ) u_mmcm (
        .CLKIN1(clkin), .CLKIN2(1'b0), .CLKINSEL(1'b1),
        .CLKFBIN(clkfb_in), .CLKFBOUT(clkfb_out), .CLKFBOUTB(),
        .CLKOUT0(clk0), .CLKOUT0B(), .CLKOUT1(clk1), .CLKOUT1B(),
        .CLKOUT2(), .CLKOUT2B(), .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        .DCLK(clk), .DADDR(daddr), .DEN(den), .DWE(dwe), .DI(di), .DO(dout), .DRDY(drdy),
        .PSCLK(1'b0), .PSEN(1'b0), .PSINCDEC(1'b0), .PSDONE(),
        .LOCKED(mmcm_locked), .CLKINSTOPPED(), .CLKFBSTOPPED(),
        .PWRDWN(1'b0), .RST(mmcm_rst));
    BUFG u_bfb  (.I(clkfb_out), .O(clkfb_in));
    BUFG u_bser (.I(clk0),      .O(ser_clk));
    BUFG u_bpix (.I(clk1),      .O(pix_clk));
    assign locked_raw = mmcm_locked;

    (* ASYNC_REG = "TRUE" *) reg [1:0] lk_s = 2'b00;
    always @(posedge clk) lk_s <= {lk_s[0], mmcm_locked};
    assign locked = lk_s[1];

    // ---------------- DRP sequencer ----------------
    typedef enum logic [2:0] {V_IDLE, V_RD, V_RDW, V_WR, V_WRW, V_LOCK} vs_t;
    vs_t        vs;
    reg  [2:0]  m;
    reg  [4:0]  k;
    reg  [31:0] tmo;
    wire [38:0] ent = vm_drp(m, k);                // {daddr, keep_mask, data}

    always @(posedge clk) begin
        den <= 1'b0; dwe <= 1'b0; done <= 1'b0;
        if (rst) begin
            vs <= V_IDLE; busy <= 1'b0; ok <= 1'b1; mmcm_rst <= 1'b0; m <= 3'd0; k <= 5'd0; tmo <= 0;
        end else case (vs)
            V_IDLE: if (req) begin
                m <= req_mode; k <= 5'd0; busy <= 1'b1; mmcm_rst <= 1'b1; vs <= V_RD;
            end
            V_RD:  begin daddr <= ent[38:32]; den <= 1'b1; vs <= V_RDW; end
            V_RDW: if (drdy) begin di <= (dout & ent[31:16]) | ent[15:0]; vs <= V_WR; end
            V_WR:  begin den <= 1'b1; dwe <= 1'b1; vs <= V_WRW; end
            V_WRW: if (drdy) begin
                if (k == 5'(VM_DRP_REGS - 1)) begin mmcm_rst <= 1'b0; tmo <= 0; vs <= V_LOCK; end
                else begin k <= k + 5'd1; vs <= V_RD; end
            end
            V_LOCK: begin
                // LOCKED falls with RST; wait for it to rise again (synchroniser delay covered by tmo > 4)
                if (locked && tmo > 32'd4) begin ok <= 1'b1; done <= 1'b1; busy <= 1'b0; vs <= V_IDLE; end
                else if (tmo == LOCK_TIMEOUT) begin ok <= 1'b0; done <= 1'b1; busy <= 1'b0; vs <= V_IDLE; end
                else tmo <= tmo + 1;
            end
            default: vs <= V_IDLE;
        endcase
    end
endmodule
`default_nettype wire
