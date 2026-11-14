// mdio_mgr — RTL8211F management, clk_sys domain.
//   1. PHY hardware reset (RST_MS low), then WAIT_MS before the first MDIO access.
//   2. Init: the delay writes (page 0xd08: reg 0x11 = 0x0109 TXDLY on, reg 0x15 = 0x0011
//      RXDLY off — or 0x0019 when `rxdly` is set), EEE not advertised (page 0xa5d: reg 0x10 = 0; our MAC
//      has no LPI, and on an EEE link frames vanish while it wakes), advertisement (reg 4 = 0x0101
//      100BASE-TX FD, reg 9 = 0x0200 1000BASE-T FD or 0 when !adv1000), restart auto-negotiation
//      (reg 0 = 0x1340).
//   3. Every POLL_US: page 0xa43 reg 0x1A (PHYSR: link bit 2, duplex bit 3, speed bits 5:4), back to
//      page 0, reg 1 (BMSR). link_up / speed / duplex follow PHYSR (in-band RGMII status is not used).
//   4. Between polls: one host request (page, reg, read or write) -> resp + resp_rdata.
// `reinit` restarts from step 1 (used after changing rxdly / adv1000 / phyad).
// MDIO frames (Clause 22): 32-bit preamble, ST=01, OP (01 write, 10 read), PHYAD, REGAD, TA, DATA.
// The master changes MDIO on MDC falling edges; on reads it samples on MDC rising edges the bit the PHY
// drove after the previous rising edge (IEEE 802.3 22.3.4).
`default_nettype none
module mdio_mgr #(
    parameter int CLK_HZ  = 100_000_000,
    parameter int MDC_HZ  = 2_500_000,
    parameter int RST_MS  = 20,
    parameter int WAIT_MS = 100,
    parameter int POLL_US = 5000
)(
    input  wire        clk, rst,
    input  wire [4:0]  phyad,
    input  wire        rxdly, adv1000,
    input  wire        reinit,
    output reg         phy_rstn,
    output reg         mdc,
    output reg         mdio_o,
    output reg         mdio_oe,
    input  wire        mdio_i,
    // host request: hold req until resp
    input  wire        req,
    input  wire        req_wr,
    input  wire [15:0] req_page,
    input  wire [4:0]  req_reg,
    input  wire [15:0] req_wdata,
    output reg         resp,
    output reg  [15:0] resp_rdata,
    // status
    output reg         init_done,
    output reg         link_up,
    output reg  [1:0]  speed,                  // 00 10M, 01 100M, 10 1000M
    output reg         duplex,
    output reg  [15:0] physr, bmsr,
    output reg  [31:0] polls
);
    localparam int DIV       = CLK_HZ / (2 * MDC_HZ);
    localparam int MS        = CLK_HZ / 1000;
    localparam int RST_CYC   = RST_MS * MS;
    localparam int WAIT_CYC  = WAIT_MS * MS;
    localparam int POLL_CYC  = POLL_US * (CLK_HZ / 1_000_000);

    // ---------------- single-operation MDIO engine ----------------
    reg         op_go, op_wr, op_done, run;
    reg  [4:0]  op_reg;
    reg  [15:0] op_wd, op_rd;
    reg  [63:0] sh;
    reg  [6:0]  bidx;
    reg  [$clog2(DIV)-1:0] dcnt;
    wire tick     = (dcnt == DIV - 1);
    wire mdc_fall = run & tick & mdc;
    wire mdc_rise = run & tick & ~mdc;
    always @(posedge clk) begin
        op_done <= 1'b0;
        if (rst) begin run <= 0; mdc <= 0; dcnt <= 0; mdio_oe <= 0; mdio_o <= 1'b1; bidx <= 0; end
        else if (!run) begin
            mdc <= 1'b0; dcnt <= 0; mdio_oe <= 1'b0;
            if (op_go) begin
                run <= 1'b1; bidx <= 7'd63;
                sh  <= op_wr ? {32'hFFFF_FFFF, 2'b01, 2'b01, phyad, op_reg, 2'b10, op_wd}
                             : {32'hFFFF_FFFF, 2'b01, 2'b10, phyad, op_reg, 2'b00, 16'h0000};
                mdio_oe <= 1'b1; mdio_o <= 1'b1;
            end
        end else begin
            if (tick) begin dcnt <= 0; mdc <= ~mdc; end else dcnt <= dcnt + 1'b1;
            mdio_o  <= sh[63];
            mdio_oe <= op_wr | (bidx > 7'd17);          // reads: release for TA + data
            if (mdc_rise && !op_wr && bidx <= 7'd15) op_rd <= {op_rd[14:0], mdio_i};
            if (mdc_fall) begin
                sh <= {sh[62:0], 1'b0};
                if (bidx == 0) begin run <= 1'b0; op_done <= 1'b1; mdio_oe <= 1'b0; end
                else bidx <= bidx - 1'b1;
            end
        end
    end

    // ---------------- sequencer ----------------
    typedef enum logic [2:0] {S_RST, S_WAIT, S_INIT, S_IDLE, S_POLL, S_HOST} st_t;
    st_t        st;
    reg  [31:0] cnt;
    reg  [3:0]  step;
    reg         busy;                             // an op is in flight

    // op table: {wr, reg, data}
    function automatic [21:0] init_op(input [3:0] s, input rxd, input a1000);
        case (s)
            4'd0: init_op = {1'b1, 5'h1f, 16'h0d08};
            4'd1: init_op = {1'b1, 5'h11, 16'h0109};
            4'd2: init_op = {1'b1, 5'h1f, 16'h0d08};
            4'd3: init_op = {1'b1, 5'h15, rxd ? 16'h0019 : 16'h0011};
            4'd4: init_op = {1'b1, 5'h1f, 16'h0a5d};
            4'd5: init_op = {1'b1, 5'h10, 16'h0000};
            4'd6: init_op = {1'b1, 5'h1f, 16'h0000};
            4'd7: init_op = {1'b1, 5'h04, 16'h0101};
            4'd8: init_op = {1'b1, 5'h09, a1000 ? 16'h0200 : 16'h0000};
            default: init_op = {1'b1, 5'h00, 16'h1340};
        endcase
    endfunction
    function automatic [21:0] poll_op(input [3:0] s);
        case (s)
            4'd0: poll_op = {1'b1, 5'h1f, 16'h0a43};
            4'd1: poll_op = {1'b0, 5'h1a, 16'h0000};
            4'd2: poll_op = {1'b1, 5'h1f, 16'h0000};
            default: poll_op = {1'b0, 5'h01, 16'h0000};
        endcase
    endfunction

    reg [21:0] nop;
    always @(posedge clk) begin
        op_go <= 1'b0; resp <= 1'b0;
        if (rst || reinit) begin
            st <= S_RST; cnt <= 0; step <= 0; busy <= 0; phy_rstn <= 1'b0; init_done <= 1'b0;
            link_up <= 1'b0; speed <= 2'b01; duplex <= 1'b0; physr <= 16'h0; bmsr <= 16'h0;
            if (rst) polls <= 0;
        end else begin
            if (op_done) busy <= 1'b0;
            case (st)
                S_RST:  if (cnt == RST_CYC - 1) begin cnt <= 0; phy_rstn <= 1'b1; st <= S_WAIT; end
                        else cnt <= cnt + 1;
                S_WAIT: if (cnt == WAIT_CYC - 1) begin cnt <= 0; step <= 0; st <= S_INIT; end
                        else cnt <= cnt + 1;
                S_INIT: if (!busy && !op_go) begin
                            if (step == 4'd10) begin init_done <= 1'b1; cnt <= 0; st <= S_IDLE; end
                            else begin
                                nop = init_op(step, rxdly, adv1000);
                                op_wr <= nop[21]; op_reg <= nop[20:16]; op_wd <= nop[15:0];
                                op_go <= 1'b1; busy <= 1'b1; step <= step + 1'b1;
                            end
                        end
                S_IDLE: begin
                            cnt <= cnt + 1;
                            if (cnt >= POLL_CYC - 1) begin cnt <= 0; step <= 0; st <= S_POLL; end
                            else if (req) begin step <= 0; st <= S_HOST; end
                        end
                S_POLL: if (!busy && !op_go) begin
                            if (step == 4'd2) begin
                                physr <= op_rd;
                                link_up <= op_rd[2] && (op_rd != 16'hFFFF);
                                speed <= op_rd[5:4]; duplex <= op_rd[3];
                            end
                            if (step == 4'd4) begin bmsr <= op_rd; polls <= polls + 1; st <= S_IDLE; end
                            else begin
                                nop = poll_op(step);
                                op_wr <= nop[21]; op_reg <= nop[20:16]; op_wd <= nop[15:0];
                                op_go <= 1'b1; busy <= 1'b1; step <= step + 1'b1;
                            end
                        end
                S_HOST: if (!busy && !op_go) begin
                            case (step)
                                4'd0: begin op_wr <= 1'b1; op_reg <= 5'h1f; op_wd <= req_page; end
                                4'd1: begin op_wr <= req_wr; op_reg <= req_reg; op_wd <= req_wdata; end
                                default: begin op_wr <= 1'b1; op_reg <= 5'h1f; op_wd <= 16'h0000; end
                            endcase
                            if (step == 4'd2) resp_rdata <= req_wr ? req_wdata : op_rd;
                            if (step == 4'd3) begin resp <= 1'b1; st <= S_IDLE; end
                            else begin op_go <= 1'b1; busy <= 1'b1; step <= step + 1'b1; end
                        end
                default: st <= S_RST;
            endcase
        end
    end
endmodule
`default_nettype wire
