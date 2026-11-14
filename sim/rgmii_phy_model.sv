`timescale 1ns/1ps
// rgmii_phy_model — RTL8211F-like PHY for the board-A benches.
//  * RX (PHY -> FPGA): rxc 125 MHz (giga) or 25 MHz; send_frame() adds preamble/SFD and FCS (optionally
//    corrupted). 1000: low nibble around the rising edge, high nibble around the falling edge.
//    100: each nibble held a whole cycle, low nibble first. Data changes a quarter period away from the
//    sampling edges (the FPGA side uses IDELAY tap 0 in simulation).
//  * TX (FPGA -> PHY): decodes rgmii_txc/tx_ctl/txd into frames (preamble included) in `txq`.
//  * MDIO slave (Clause 22) with the RTL8211F page register 0x1f; PHYSR = page 0xa43 reg 0x1a is
//    driven by `physr`; PHY ID regs 2/3 = 0x001C / 0xC916; other registers are plain storage. Every
//    write is logged in `wlog` as {page, reg, data}.
`default_nettype none
module rgmii_phy_model #(parameter [4:0] PHYAD = 5'd1) (
    input  wire       giga,
    output reg        rgmii_rxc,
    output reg        rgmii_rx_ctl,
    output reg  [3:0] rgmii_rxd,
    input  wire       rgmii_txc, rgmii_tx_ctl,
    input  wire [3:0] rgmii_txd,
    input  wire       mdc,
    inout  wire       mdio,
    input  wire [15:0] physr
);
    // ---------------- receive clock ----------------
    initial begin rgmii_rxc = 0; rgmii_rx_ctl = 0; rgmii_rxd = 0; end
    always begin
        if (giga) begin #4 rgmii_rxc = ~rgmii_rxc; end
        else      begin #20 rgmii_rxc = ~rgmii_rxc; end
    end

    function automatic bit [31:0] crc32(input byte unsigned d [$]);
        bit [31:0] c = 32'hFFFFFFFF;
        foreach (d[i]) for (int b = 0; b < 8; b++) c = (c[0] ^ d[i][b]) ? ((c >> 1) ^ 32'hEDB88320) : (c >> 1);
        return ~c;
    endfunction

    semaphore rx_lock = new(1);
    // frame = destination MAC .. payload (no preamble, no FCS); bad_fcs flips one FCS bit; er_at >= 0 raises RX_ER
    task automatic send_frame(input byte unsigned frame [$], input bit bad_fcs = 0, input int er_at = -1);
        byte unsigned w [$];
        bit [31:0] f;
        rx_lock.get(1);
        f = crc32(frame);
        if (bad_fcs) f[5] = ~f[5];
        repeat (7) w.push_back(8'h55); w.push_back(8'hD5);
        foreach (frame[i]) w.push_back(frame[i]);
        for (int i = 0; i < 4; i++) w.push_back(f[8*i +: 8]);
        if (giga) @(negedge rgmii_rxc); else @(posedge rgmii_rxc);
        foreach (w[i]) begin
            if (giga) begin
                // low nibble straddles the rising edge, high nibble the falling edge
                #2 rgmii_rx_ctl = 1'b1; rgmii_rxd = w[i][3:0];
                @(posedge rgmii_rxc); #2 rgmii_rx_ctl = 1'b1 ^ (er_at >= 0 && i == er_at + 8); rgmii_rxd = w[i][7:4];
                @(negedge rgmii_rxc);
            end else begin
                #30 rgmii_rx_ctl = 1'b1; rgmii_rxd = w[i][3:0];
                @(posedge rgmii_rxc);
                #30 rgmii_rxd = w[i][7:4];
                @(posedge rgmii_rxc);
            end
        end
        #(giga ? 2 : 30) rgmii_rx_ctl = 1'b0; rgmii_rxd = 4'h0;
        repeat (12) @(posedge rgmii_rxc);           // inter-frame gap (+ nibble pairs at 100)
        if (!giga) repeat (12) @(posedge rgmii_rxc);
        rx_lock.put(1);
    endtask

    // ---------------- transmit decoder ----------------
    byte unsigned txq [$][$];
    byte unsigned cur [$];
    bit           in_f = 0, half = 0;
    bit [3:0]     lo;
    always @(posedge rgmii_txc) begin
        #(giga ? 1.5 : 10);
        if (rgmii_tx_ctl) begin
            in_f = 1;
            if (giga) lo = rgmii_txd;
            else begin
                if (!half) begin lo = rgmii_txd; half = 1; end
                else begin cur.push_back({rgmii_txd, lo}); half = 0; end
            end
        end else if (in_f) begin
            txq.push_back(cur); cur = {}; in_f = 0; half = 0;
        end
    end
    always @(negedge rgmii_txc) if (giga) begin
        #1.5;
        if (rgmii_tx_ctl && in_f) cur.push_back({rgmii_txd, lo});
    end

    // ---------------- MDIO slave ----------------
    bit [15:0] regs [bit [20:0]];              // {page, reg}
    bit [15:0] page = 16'h0000;
    bit [36:0] wlog [$];
    reg        m_oe = 0, m_o = 1;
    assign mdio = m_oe ? m_o : 1'bz;
    int        ones = 0, st = 0, bitn = 0;
    bit [11:0] hdr;
    bit [15:0] wd, rdv;
    bit        is_rd;
    function automatic bit [15:0] rdreg(input bit [4:0] r);
        if (r == 5'h1f) return page;
        if (r == 5'h02) return 16'h001C;
        if (r == 5'h03) return 16'hC916;
        if (r == 5'h1a && page == 16'h0a43) return physr;
        if (r == 5'h01) return {13'h0, physr[2], 2'b0} | 16'h7949;
        return regs.exists({page, r}) ? regs[{page, r}] : 16'h0;
    endfunction
    always @(posedge mdc) begin
        bit s; s = mdio;
        case (st)
            0: begin                            // preamble ones, then 0 1
                if (s) ones++;
                else if (ones >= 32) begin st = 1; end
                else ones = 0;
            end
            1: begin if (s) begin st = 2; bitn = 0; end else begin st = 0; ones = 0; end end
            2: begin                            // op(2) phyad(5) regad(5)
                hdr = {hdr[10:0], s}; bitn++;
                if (bitn == 12) begin
                    is_rd = (hdr[11:10] == 2'b10);
                    st = 3; bitn = 0;
                    if (is_rd) rdv = rdreg(hdr[4:0]);
                end
            end
            3: begin                            // read: TA + 16 data bits driven after rising edges
                if (is_rd) begin
                    if (hdr[9:5] == PHYAD) begin
                        #20;
                        if (bitn == 0) begin m_oe = 1; m_o = 0; end
                        else if (bitn <= 16) begin m_o = rdv[16 - bitn]; end
                    end
                    bitn++;
                    if (bitn == 18) begin #20 m_oe = 0; st = 0; ones = 0; end
                end else begin                  // write: TA (2) then 16 data bits
                    bitn++;
                    if (bitn > 2) wd = {wd[14:0], s};
                    if (bitn == 18) begin
                        if (hdr[9:5] == PHYAD) begin
                            wlog.push_back({page, hdr[4:0], wd});
                            if (hdr[4:0] == 5'h1f) page = wd; else regs[{page, hdr[4:0]}] = wd;
                        end
                        st = 0; ones = 0;
                    end
                end
            end
        endcase
    end
endmodule
`default_nettype wire
