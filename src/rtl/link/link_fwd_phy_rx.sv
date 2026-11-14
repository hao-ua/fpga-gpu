// link_fwd_phy_rx — receive side of the 40-wire forward lane (A -> B); pin table in link_fwd_phy_tx.
// Captures one 64-bit beat per LCLK with 34 IDDRs (D[31:0], PAR0, PAR1) on the trained recovered
// clock (link_rx_clock + link_rx_trainer); the 5 control lines on the rising edge. Both headers'
// inputs are clocked from the same BUFG.
`default_nettype none
module link_fwd_phy_rx #(
    parameter int  LINK_MHZ        = 50,
    parameter int  MMCM_RST_CYCLES = 1_000_000,
    parameter int  TRAIN_WORDS     = 1024,
    parameter int  SETTLE_CYCLES   = 64,
    parameter int  MIN_EYE         = 4,
    parameter int  RETRY_CYCLES    = 100_000,
    parameter bit  SIM_SHIFTER     = 1'b0
)(
    input  wire        clk, rst,
    input  wire        rst_rx,
    input  wire        retrain,
    inout  wire [31:0] j1,
    inout  wire [7:0]  j2x,
    output wire        lclk_rx,
    output wire        rx_locked,
    output reg  [31:0] lo, hi,          // beat
    output reg         valid_lo, valid_hi, last,
    output reg         xoff, up,
    output reg  [3:0]  par_err,         // {hi[31:16], hi[15:0], lo[31:16], lo[15:0]}
    output wire        trained,
    output wire [7:0]  eye_width, phase, train_cnt
);
    `include "link_funcs.svh"
    localparam int D = link_div(LINK_MHZ);

    wire [31:0] p1; wire [7:0] p2;
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : g_in1
            IOBUF u_io (.I(1'b0), .T(1'b1), .O(p1[i]), .IO(j1[i]));
        end
        for (i = 0; i < 8; i = i + 1) begin : g_in2
            IOBUF u_io (.I(1'b0), .T(1'b1), .O(p2[i]), .IO(j2x[i]));
        end
    endgenerate

    wire psen, psincdec, psdone, rx_ok, mmcm_rst, meas_req, meas_ack, meas_bad;
    link_rx_clock #(.LINK_MHZ(LINK_MHZ), .MMCM_RST_CYCLES(MMCM_RST_CYCLES), .SIM_SHIFTER(SIM_SHIFTER)) u_clk (
        .clk(clk), .rst(rst), .clkin(p1[20]), .psen(psen), .psincdec(psincdec), .psdone(psdone),
        .lclk_rx(lclk_rx), .locked(rx_locked), .rx_ok(rx_ok), .mmcm_rst(mmcm_rst));
    link_rx_trainer #(.NSTEPS(7 * D), .PS_INC(8), .TRAIN_WORDS(TRAIN_WORDS), .SETTLE_CYCLES(SETTLE_CYCLES),
                      .MIN_EYE(MIN_EYE), .RETRY_CYCLES(RETRY_CYCLES)) u_trainer (
        .clk(clk), .rst(rst), .rx_ok(rx_ok), .mmcm_rst(mmcm_rst), .retrain(retrain),
        .psen(psen), .psincdec(psincdec), .psdone(psdone),
        .meas_req(meas_req), .meas_ack(meas_ack), .meas_bad(meas_bad),
        .trained(trained), .eye_width(eye_width), .phase(phase), .train_cnt(train_cnt));

    // ---- capture ----
    // ddr_in[31:0] = D[31:0], [32] = PAR0, [33] = PAR1  (pin table in link_fwd_phy_tx)
    wire [33:0] ddr_in = {p2[7], p1[17], p2[6:1], p1[31], p1[30:23], p1[16], p1[15:0]};
    wire [33:0] q1, q2;
    generate
        for (i = 0; i < 34; i = i + 1) begin : g_iddr
            IDDR #(.DDR_CLK_EDGE("SAME_EDGE_PIPELINED"), .INIT_Q1(1'b0), .INIT_Q2(1'b0), .SRTYPE("SYNC")) u_iddr (
                .Q1(q1[i]), .Q2(q2[i]), .C(lclk_rx), .CE(1'b1), .D(ddr_in[i]), .R(1'b0), .S(1'b0));
        end
    endgenerate
    // control: {up, xoff, valid_hi, last, valid_lo}
    wire [4:0] ctl_in = {p1[19], p2[0], p1[21], p1[18], p1[22]};   // {up, xoff, valid_hi, last, valid_lo}
    reg  [4:0] ctl_q = 0;
    (* IOB = "TRUE" *) reg [4:0] ctl_iob = 0;
    always @(posedge lclk_rx) ctl_iob <= ctl_in;
    always @(posedge lclk_rx) ctl_q <= ctl_iob;

    always @(posedge lclk_rx) begin
        lo <= q1[31:0]; hi <= q2[31:0];
        par_err <= { (^q2[31:16]) ^ q2[33], (^q2[15:0]) ^ q2[32], (^q1[31:16]) ^ q1[33], (^q1[15:0]) ^ q1[32] };
        if (rst_rx) begin valid_lo <= 0; valid_hi <= 0; last <= 0; xoff <= 0; up <= 0; end
        else begin valid_lo <= ctl_q[0]; last <= ctl_q[1]; valid_hi <= ctl_q[2]; xoff <= ctl_q[3]; up <= ctl_q[4]; end
    end

    link_rx_meas #(.W(64), .TRAIN_WORDS(TRAIN_WORDS)) u_meas (
        .lclk_rx(lclk_rx), .rst_rx(rst_rx), .meas_req(meas_req), .word({hi, lo}), .valid(valid_lo & valid_hi),
        .par_bad(|par_err), .meas_ack(meas_ack), .meas_bad(meas_bad));
endmodule
`default_nettype wire
