// link_phy_rx — receive side of the 22-wire backward lane (B -> A, JM2 bits [21:0]).
//
// Lane vector bit order (link_phy_tx has the same table):
//   [15:0] D[15:0] DDR (rising = word[15:0], falling = word[31:16]), [16] VALID, [17] PAR (DDR, per
//   half), [18] LAST, [19] UP, [20] LCLK (MRCC), [21] XOFF (for the opposite lane).
// The 17 DDR lines are captured with IDDR SAME_EDGE_PIPELINED (both halves of one bit period
// aligned to the next rising edge); the 4 control lines are captured on the rising edge. The
// capture clock comes from link_rx_clock and is placed by link_rx_trainer.
`default_nettype none
module link_phy_rx #(
    parameter int  LINK_MHZ        = 50,
    parameter int  MMCM_RST_CYCLES = 1_000_000,
    parameter int  TRAIN_WORDS     = 1024,
    parameter int  SETTLE_CYCLES   = 64,
    parameter int  MIN_EYE         = 4,
    parameter int  RETRY_CYCLES    = 100_000,
    parameter bit  SIM_SHIFTER     = 1'b0
)(
    input  wire        clk, rst,                   // user clock domain
    input  wire        rst_rx,                     // lclk_rx-domain reset (from the caller, derived from rx_locked)
    input  wire        retrain,                    // clk: pulse -> drop `trained` and scan again
    inout  wire [21:0] pins,
    output wire        lclk_rx,
    output wire        rx_locked,
    output reg  [31:0] word,
    output reg         valid,
    output reg         last,
    output reg         xoff,        // far end asks us to stop sending on OUR tx lane
    output reg         up,          // far end's receiver is trained on OUR tx lane
    output reg  [1:0]  par_err,     // [0] = low half, [1] = high half (valid only with `valid`)
    // training status (clk domain)
    output wire        trained,
    output wire [7:0]  eye_width, phase, train_cnt
);
    `include "link_funcs.svh"
    localparam int D = link_div(LINK_MHZ);

    wire [21:0] pin_in;
    genvar i;
    generate
        for (i = 0; i < 22; i = i + 1) begin : g_in
            IOBUF u_io (.I(1'b0), .T(1'b1), .O(pin_in[i]), .IO(pins[i]));
        end
    endgenerate

    // ---- clock + trainer ----
    wire psen, psincdec, psdone, rx_ok, mmcm_rst, meas_req, meas_ack, meas_bad;
    link_rx_clock #(.LINK_MHZ(LINK_MHZ), .MMCM_RST_CYCLES(MMCM_RST_CYCLES), .SIM_SHIFTER(SIM_SHIFTER)) u_clk (
        .clk(clk), .rst(rst), .clkin(pin_in[20]), .psen(psen), .psincdec(psincdec), .psdone(psdone),
        .lclk_rx(lclk_rx), .locked(rx_locked), .rx_ok(rx_ok), .mmcm_rst(mmcm_rst));
    link_rx_trainer #(.NSTEPS(7 * D), .PS_INC(8), .TRAIN_WORDS(TRAIN_WORDS), .SETTLE_CYCLES(SETTLE_CYCLES),
                      .MIN_EYE(MIN_EYE), .RETRY_CYCLES(RETRY_CYCLES)) u_trainer (
        .clk(clk), .rst(rst), .rx_ok(rx_ok), .mmcm_rst(mmcm_rst), .retrain(retrain),
        .psen(psen), .psincdec(psincdec), .psdone(psdone),
        .meas_req(meas_req), .meas_ack(meas_ack), .meas_bad(meas_bad),
        .trained(trained), .eye_width(eye_width), .phase(phase), .train_cnt(train_cnt));

    // ---- capture (lclk_rx domain) ----
    wire [16:0] q1, q2;           // q1 = rising-edge sample, q2 = following falling-edge sample
    wire [16:0] ddr_in = {pin_in[17], pin_in[15:0]};
    generate
        for (i = 0; i < 17; i = i + 1) begin : g_iddr
            IDDR #(.DDR_CLK_EDGE("SAME_EDGE_PIPELINED"), .INIT_Q1(1'b0), .INIT_Q2(1'b0), .SRTYPE("SYNC")) u_iddr (
                .Q1(q1[i]), .Q2(q2[i]), .C(lclk_rx), .CE(1'b1), .D(ddr_in[i]), .R(1'b0), .S(1'b0));
        end
    endgenerate
    reg [3:0] ctl_q = 0;
    (* IOB = "TRUE" *) reg [3:0] ctl_iob = 0;
    always @(posedge lclk_rx) ctl_iob <= {pin_in[21], pin_in[19], pin_in[18], pin_in[16]};   // {xoff, up, last, valid}
    always @(posedge lclk_rx) ctl_q <= ctl_iob;              // align with the IDDR pipeline

    always @(posedge lclk_rx) begin
        word    <= {q2[15:0], q1[15:0]};
        par_err <= { (^q2[15:0]) ^ q2[16], (^q1[15:0]) ^ q1[16] };
        if (rst_rx) begin valid <= 0; last <= 0; xoff <= 0; up <= 0; end   // far end gone: no stale UP/XOFF
        else begin valid <= ctl_q[0]; last <= ctl_q[1]; up <= ctl_q[2]; xoff <= ctl_q[3]; end
    end

    link_rx_meas #(.W(32), .TRAIN_WORDS(TRAIN_WORDS)) u_meas (
        .lclk_rx(lclk_rx), .rst_rx(rst_rx), .meas_req(meas_req), .word(word), .valid(valid), .par_bad(|par_err),
        .meas_ack(meas_ack), .meas_bad(meas_bad));
endmodule
`default_nettype wire
