// rgmii_tx — speed-adaptive RGMII transmit. Runs in the TX line-clock domain
// (rgmii_txc = 2.5/25/125 MHz). Consumes GMII bytes via a FWFT handshake
// (valid/data + adv) so a gearbox FIFO can bridge the system-clock MAC to the
// line rate; rgmii_tx_adapt serializes to nibbles (MII @10/100, byte @1000) and
// ODDR primitives drive the pins. TXC is forwarded via ODDR (RGMII-ID: PHY adds
// the clock-to-data delay internally, configured over MDIO).
`default_nettype none
module rgmii_tx (
    input  wire        clk,           // tx line clock (rgmii_txc source)
    input  wire        rst,
    input  wire [1:0]  speed,         // 00=10M,01=100M,10=1000M
    input  wire        valid,         // a GMII byte is available
    input  wire [7:0]  data,          // current GMII byte (FWFT front)
    output wire        adv,           // pop: present next byte next clk
    output wire        rgmii_txc,
    output wire        rgmii_tx_ctl,
    output wire [3:0]  rgmii_txd
);
    wire [3:0] txd_d1, txd_d2; wire ctl_d1, ctl_d2;

    rgmii_tx_adapt u_adapt(.clk,.rst,.speed,.valid,.data,.adv,
        .txd_d1,.txd_d2,.ctl_d1,.ctl_d2);

    // forward TX clock (aligned; PHY supplies RGMII internal delay)
    ODDR #(.DDR_CLK_EDGE("SAME_EDGE"),.INIT(1'b0),.SRTYPE("SYNC"))
      u_oddr_txc (.Q(rgmii_txc),.C(clk),.CE(1'b1),.D1(1'b1),.D2(1'b0),.R(1'b0),.S(1'b0));

    ODDR #(.DDR_CLK_EDGE("SAME_EDGE"),.INIT(1'b0),.SRTYPE("SYNC"))
      u_oddr_ctl (.Q(rgmii_tx_ctl),.C(clk),.CE(1'b1),.D1(ctl_d1),.D2(ctl_d2),.R(1'b0),.S(1'b0));

    genvar i;
    generate for (i=0;i<4;i=i+1) begin : txdata
        ODDR #(.DDR_CLK_EDGE("SAME_EDGE"),.INIT(1'b0),.SRTYPE("SYNC"))
          u_oddr_txd (.Q(rgmii_txd[i]),.C(clk),.CE(1'b1),.D1(txd_d1[i]),.D2(txd_d2[i]),.R(1'b0),.S(1'b0));
    end endgenerate
endmodule
`default_nettype wire
