// fb_proto_a — board A host protocol engine, clk_sys.
// Reads UDP datagrams from the eth_stack payload FIFO (descriptor + payload, net_pkg format):
//   * checks magic/version, learns the host address (MAC, IP, port) from every valid datagram;
//   * seq == 0  -> local: HELLO (flag bit 0: new session, expected seq := 1, answered with an ACK),
//                  STATUS_REQ, PHY_REG (via mdio_mgr), REG (board registers), CURSOR_POS (the cursor fast path,
//                  {y, x} straight to link_a_port, no queue, no reply);
//   * seq != 0  -> Go-Back-N: accepted only if seq == expected, the DDR queue is usable and has room for
//                  the whole entry; then written to the ingress queue as one header burst + body bursts
//                  (fb_pkg entry format) and expected++. Otherwise dropped whole (gap -> NAK, duplicate
//                  -> ACK, no room -> ACK with FULL), NAK/FULL/duplicate ACKs rate-limited by NAK_HOLD.
//   * ACK every ACK_EVERY accepted datagrams or ACK_TO_CYCLES after the last unacknowledged one, and a heartbeat
//     ACK every HB_CYCLES while a host is known (a lost datagram or a lost NAK then shows
//     up on the host as an ACK without progress within ~1 ms instead of its stall timer).
//   * STATUS on request and every STATUS_PERIOD once a host is known.
//   * REG_IDELAY write with value bit 8 set = trial: the tap is applied, and reverted to the previous one
//     after TRIAL_CYCLES unless another valid datagram arrives first (so an RX timing sweep cannot lock
//     the host out with a dead tap). REG_PHYOPT write with bit 8 = PHY trial: new RXDLY / 1000BASE-T
//     advertisement (and optionally a tap, bit 9 + [20:16]) applied with a PHY re-init; everything
//     reverts (and re-inits again) after PHY_TRIAL_CYCLES unless other traffic arrives.
// Reply channels to fb_reply: 0 = ACK, 1 = local reply (PHY_REG / REG), 2 = STATUS.
// A descriptor word (K_D0) arriving in the middle of a datagram means the datagram was cut by an
// Ethernet link drop: the partial datagram is abandoned (counted as bad) and parsing restarts there.
`default_nettype none
module fb_proto_a #(
    parameter int        RING_LOG2     = 21,
    parameter int        ACK_EVERY     = 16,
    parameter int        ACK_TO_CYCLES = 10_000,       // 100 us
    parameter int        NAK_HOLD      = 5_000,        // 50 us
    parameter int        HB_CYCLES     = 100_000,      // 1 ms: heartbeat ACK period (from the last ACK of any kind)
    parameter int        STATUS_PERIOD = 10_000_000,   // 100 ms
    parameter int        TRIAL_CYCLES  = 20_000_000,   // 200 ms: IDELAY trial revert time
    parameter int        PHY_TRIAL_CYCLES = 800_000_000, // 8 s: PHY option trial (includes re-init + auto-negotiation)
    parameter [4:0]      IDELAY_DEFAULT = 5'd0,
    parameter bit        RXDLY_DEFAULT  = 1'b0,         // PHY RX clock delay at power-up
    parameter [31:0]     LINKCTL_DEFAULT = 32'h1        // REG_LINKCTL at power-up (fill words on)
)(
    input  wire          clk, rst,
    input  wire          q_ok,                          // DDR calibrated
    // payload FIFO (FWFT)
    input  wire          pf_valid,
    input  wire [35:0]   pf_data,
    output wire          pf_ren,
    // ingress queue push side
    output reg           q_valid,
    output reg  [255:0]  q_data,
    input  wire          q_ready,
    input  wire [RING_LOG2:0] q_used,
    // learned host address
    output reg           host_valid,
    output reg  [47:0]   host_mac,
    output reg  [31:0]   host_ip,
    output reg  [15:0]   host_port,
    // reply channels 0..2
    output wire [2:0]    r_req,
    output wire [47:0]   r_len,
    output wire [2:0]    r_valid,
    output wire [95:0]   r_data,
    input  wire [2:0]    r_ready,
    input  wire [2:0]    r_done,
    // board registers
    output reg  [4:0]    idelay_tap,
    output reg           idelay_ld,
    output reg           rxdly, adv1000,
    output reg  [4:0]    phyad,
    output reg           reinit,
    output reg  [31:0]   linkctl,           // REG_LINKCTL: bit 0 = board link fill words on our transmit lane
    input  wire [31:0]   build_id,
    // PHY register access (mdio_mgr host request)
    output reg           phy_req,
    output reg           phy_wr,
    output reg  [15:0]   phy_page,
    output reg  [4:0]    phy_reg,
    output reg  [15:0]   phy_wdata,
    input  wire          phy_resp,
    input  wire [15:0]   phy_rdata,
    // cursor fast path
    output reg           cur_valid,
    output reg  [31:0]   cur_pos,
    // status
    input  wire [fb_pkg::STATUS_WORDS*32-1:0] status_vec,
    output reg  [31:0]   exp_seq, c_acc, c_dup, c_gap, c_full, c_bad, c_ack
);
    import net_pkg::*;
    import fb_pkg::*;
    localparam int RING = 1 << RING_LOG2;

    // ---------------- datagram parser ----------------
    typedef enum logic [3:0] {P_D0, P_D1, P_D2, P_D3, P_H0, P_H1, P_DECIDE, P_LOCAL, P_EXEC,
                              P_QHDR, P_QBODY, P_SKIP} pst_t;
    pst_t        ps;
    reg  [15:0]  dlen, dport, rem;                  // datagram bytes, source port, payload words left
    reg  [31:0]  dip;
    reg  [47:0]  dmac;
    reg  [7:0]   mtype;
    reg  [31:0]  mseq;
    reg          hdr_ok;
    reg  [15:0]  blen;                              // body bytes
    reg  [31:0]  lw [0:3];                          // local message body words
    reg  [2:0]   lwi;
    reg  [2:0]   bwi;                               // word index inside the body burst
    reg  [255:0] bbuf;

    wire [1:0]   kind     = pf_data[35:34];
    wire         is_d0    = pf_valid && kind == K_D0;
    wire         can_push = !q_valid || q_ready;
    wire [RING_LOG2:0] q_free = RING - q_used;
    wire [15:0]  need     = 16'd1 + ((blen + 16'd31) >> 5);
    wire signed [31:0] sd = mseq - exp_seq;

    reg pop;
    always @(*) begin
        pop = 1'b0;
        case (ps)
            P_D0:              pop = pf_valid;                       // descriptor (or junk to skip)
            P_D1, P_D2, P_D3:  pop = pf_valid && kind == K_DN;
            P_H0, P_H1, P_LOCAL, P_SKIP: pop = pf_valid && !is_d0 && rem != 0;
            P_QBODY:           pop = pf_valid && !is_d0 && rem != 0 && can_push;
            default:           pop = 1'b0;
        endcase
    end
    assign pf_ren = pop;

    // ---------------- ACK state ----------------
    reg  [7:0]   unacked;
    reg  [31:0]  ack_t, nak_t, hb_t;
    reg          ack_now, nak_f, full_f;
    // local reply state
    reg          lr_pend; reg [31:0] lr_w [0:3];
    reg          phy_busy;
    // status timer
    reg  [31:0]  st_t;
    reg          status_now;
    reg  [31:0]  scratch;
    reg  [4:0]   tap_prev;
    reg          rxd_prev, adv_prev;
    reg  [31:0]  trial_t;
    reg  [16:0]  boot_t;                            // IDELAYE2 in VAR_LOAD starts at tap 0 (IDELAY_VALUE is
                                                    // ignored): load the default tap once after reset
    // reply serializer state (declared here: the parser clears its request flags on start)
    reg          a_busy, s_busy;
    wire         a_start = ack_now && !a_busy;
    wire         s_start = status_now && !s_busy;

    always @(posedge clk) begin
        idelay_ld <= 1'b0; reinit <= 1'b0; cur_valid <= 1'b0;
        if (q_valid && q_ready) q_valid <= 1'b0;
        if (rst) begin
            ps <= P_D0; q_valid <= 1'b0; host_valid <= 1'b0; exp_seq <= 32'd1;
            c_acc <= 0; c_dup <= 0; c_gap <= 0; c_full <= 0; c_bad <= 0;
            unacked <= 0; ack_t <= 0; nak_t <= 0; hb_t <= 0; ack_now <= 0; nak_f <= 0; full_f <= 0;
            idelay_tap <= IDELAY_DEFAULT; rxdly <= RXDLY_DEFAULT; adv1000 <= 1'b1; phyad <= 5'd1;
            phy_req <= 1'b0; phy_busy <= 1'b0; lr_pend <= 1'b0; st_t <= 0; status_now <= 1'b0; scratch <= 0;
            linkctl <= LINKCTL_DEFAULT;
            tap_prev <= IDELAY_DEFAULT; rxd_prev <= RXDLY_DEFAULT; adv_prev <= 1'b1; trial_t <= 0; boot_t <= 0;
        end else begin
            // ---- initial IDELAY load, 1 ms after reset (IDELAYCTRL long ready by then) ----
            if (boot_t != 17'h1FFFF) boot_t <= boot_t + 1'b1;
            if (boot_t == 17'd100_000) idelay_ld <= 1'b1;
            // ---- parser ----
            case (ps)
                P_D0: if (pf_valid) begin
                    if (kind == K_D0) begin dlen <= pf_data[15:0]; dport <= pf_data[31:16]; ps <= P_D1; end
                    else c_bad <= c_bad + 1;                 // stray word: skipped
                end
                P_D1: if (pf_valid) begin if (kind == K_DN) begin dip <= pf_data[31:0]; ps <= P_D2; end else ps <= P_D0; end
                P_D2: if (pf_valid) begin if (kind == K_DN) begin dmac[31:0] <= pf_data[31:0]; ps <= P_D3; end else ps <= P_D0; end
                P_D3: if (pf_valid) begin
                    if (kind == K_DN) begin
                        dmac[47:32] <= pf_data[15:0];
                        rem <= (dlen + 16'd3) >> 2;
                        if (dlen < 16'd8) begin c_bad <= c_bad + 1; ps <= P_SKIP; end
                        else ps <= P_H0;
                    end else ps <= P_D0;
                end
                P_H0: if (is_d0) begin c_bad <= c_bad + 1; ps <= P_D0; end
                      else if (pf_valid) begin
                          hdr_ok <= pf_data[15:0] == FB_MAGIC && pf_data[23:16] == FB_VERSION;
                          mtype <= pf_data[31:24]; rem <= rem - 1'b1; ps <= P_H1;
                      end
                P_H1: if (is_d0) begin c_bad <= c_bad + 1; ps <= P_D0; end
                      else if (pf_valid) begin mseq <= pf_data[31:0]; rem <= rem - 1'b1; blen <= dlen - 16'd8; ps <= P_DECIDE; end
                P_DECIDE: begin
                    if (!hdr_ok) begin c_bad <= c_bad + 1; ps <= P_SKIP; end
                    else begin
                        host_valid <= 1'b1; host_mac <= dmac; host_ip <= dip; host_port <= dport;
                        if (mseq == 32'd0) begin lwi <= 0; lw[0] <= 0; lw[1] <= 0; lw[2] <= 0; lw[3] <= 0; ps <= P_LOCAL; end
                        else if (sd == 0 && q_ok && q_free >= need) begin
                            ps <= P_QHDR;
                        end else begin
                            if (sd == 0)      begin c_full <= c_full + 1; full_f <= 1'b1; end
                            else if (sd > 0)  begin c_gap  <= c_gap + 1;  nak_f  <= 1'b1; end
                            else                    c_dup  <= c_dup + 1;
                            if (nak_t == 0) begin ack_now <= 1'b1; nak_t <= NAK_HOLD; end
                            ps <= P_SKIP;
                        end
                    end
                end
                P_LOCAL: begin                                // collect up to 4 body words
                    if (is_d0) begin c_bad <= c_bad + 1; ps <= P_D0; end
                    else if (rem == 0) ps <= P_EXEC;
                    else if (pf_valid) begin
                        if (lwi < 4) begin lw[lwi[1:0]] <= pf_data[31:0]; lwi <= lwi + 1'b1; end
                        rem <= rem - 1'b1;
                    end
                end
                P_EXEC: begin
                    case (mtype)
                        T_HELLO:      begin if (lw[0][0]) exp_seq <= 32'd1; ack_now <= 1'b1; end
                        T_STATUS_REQ: status_now <= 1'b1;
                        T_CURSOR_POS: begin cur_valid <= 1'b1; cur_pos <= lw[0]; end
                        T_PHY_REG: if (!phy_busy) begin
                            phy_req <= 1'b1; phy_busy <= 1'b1; phy_wr <= lw[0][0];
                            phy_reg <= lw[0][12:8]; phy_page <= lw[0][31:16]; phy_wdata <= lw[1][15:0];
                        end
                        T_REG: begin : reg_acc
                            reg [31:0] rv; rv = 32'h0;
                            case (lw[0][31:16])
                                REG_IDELAY:  begin                // value: [4:0] tap, [8] trial
                                                 if (lw[0][0]) begin
                                                     idelay_tap <= lw[1][4:0]; idelay_ld <= 1'b1;
                                                     if (lw[1][8]) begin
                                                         if (trial_t == 0) begin            // save the committed state
                                                             tap_prev <= idelay_tap; rxd_prev <= rxdly; adv_prev <= adv1000;
                                                         end
                                                         trial_t <= TRIAL_CYCLES;
                                                     end else begin
                                                         tap_prev <= lw[1][4:0]; rxd_prev <= rxdly; adv_prev <= adv1000; trial_t <= 0;
                                                     end
                                                 end
                                                 rv = lw[0][0] ? lw[1] & 32'h11F : {23'h0, trial_t != 0, 3'h0, idelay_tap};
                                             end
                                REG_PHYOPT:  begin                // value: [0] rxdly, [1] adv1000, [8] trial (+PHY re-init), [9] set tap [20:16]
                                                 if (lw[0][0]) begin
                                                     rxdly <= lw[1][0]; adv1000 <= lw[1][1];
                                                     if (lw[1][9]) begin idelay_tap <= lw[1][20:16]; idelay_ld <= 1'b1; end
                                                     if (lw[1][8]) begin
                                                         if (trial_t == 0) begin
                                                             tap_prev <= idelay_tap; rxd_prev <= rxdly; adv_prev <= adv1000;
                                                         end
                                                         trial_t <= PHY_TRIAL_CYCLES; reinit <= 1'b1;
                                                     end else begin
                                                         rxd_prev <= lw[1][0]; adv_prev <= lw[1][1];
                                                         tap_prev <= lw[1][9] ? lw[1][20:16] : idelay_tap; trial_t <= 0;
                                                     end
                                                 end
                                                 rv = lw[0][0] ? lw[1] & 32'h001F_0303 : {23'h0, trial_t != 0, 6'h0, adv1000, rxdly};
                                             end
                                REG_PHYAD:   begin if (lw[0][0]) phyad <= lw[1][4:0];
                                                   rv = lw[0][0] ? lw[1] & 32'h1F : {27'h0, phyad}; end
                                REG_REINIT:  begin if (lw[0][0]) reinit <= 1'b1; rv = 32'h0; end
                                REG_SCRATCH: begin if (lw[0][0]) scratch <= lw[1]; rv = lw[0][0] ? lw[1] : scratch; end
                                REG_BUILD:   rv = build_id;
                                REG_LINKCTL: begin if (lw[0][0]) linkctl <= lw[1]; rv = lw[0][0] ? lw[1] : linkctl; end
                                default:     rv = 32'hFFFF_FFFF;
                            endcase
                            if (!lr_pend) begin
                                lr_pend <= 1'b1;
                                lr_w[0] <= hdr_w0(R_REG); lr_w[1] <= 32'h0;
                                lr_w[2] <= {lw[0][31:16], 8'h00, lw[0][7:0]}; lr_w[3] <= rv;
                            end
                        end
                        default: c_bad <= c_bad + 1;
                    endcase
                    ps <= P_D0;
                end
                P_QHDR: if (can_push) begin
                    q_valid <= 1'b1;
                    q_data  <= {192'h0, mseq, Q_MARK, mtype, blen};
                    exp_seq <= exp_seq + 1; c_acc <= c_acc + 1; unacked <= unacked + 1'b1;
                    bwi <= 0; bbuf <= 256'h0;
                    ps <= (rem == 0) ? P_D0 : P_QBODY;
                end
                P_QBODY: begin
                    if (is_d0) begin c_bad <= c_bad + 1; ps <= P_D0; end      // cut datagram (entry stays short)
                    else if (pop) begin
                        if (bwi == 3'd7 || rem == 16'd1) begin
                            q_valid <= 1'b1;
                            q_data  <= (bbuf & ~(256'hFFFFFFFF << (bwi*32))) | ({224'h0, pf_data[31:0]} << (bwi*32));
                            bbuf <= 256'h0; bwi <= 0;
                        end else begin
                            bbuf[bwi*32 +: 32] <= pf_data[31:0]; bwi <= bwi + 1'b1;
                        end
                        rem <= rem - 1'b1;
                        if (rem == 16'd1) ps <= P_D0;
                    end
                end
                P_SKIP: begin
                    if (is_d0) ps <= P_D0;
                    else if (rem == 0) ps <= P_D0;
                    else if (pf_valid) rem <= rem - 1'b1;
                end
                default: ps <= P_D0;
            endcase

            // ---- PHY register reply ----
            if (phy_resp) phy_req <= 1'b0;
            if (phy_resp && phy_busy) begin
                phy_busy <= 1'b0;
                if (!lr_pend) begin
                    lr_pend <= 1'b1;
                    lr_w[0] <= hdr_w0(R_PHY_REG); lr_w[1] <= 32'h0;
                    lr_w[2] <= {phy_page, 3'b000, phy_reg, 7'h0, phy_wr}; lr_w[3] <= {16'h0, phy_rdata};
                end
            end

            // ---- trial (IDELAY tap / PHY options): revert unless other traffic arrived. Placed after the
            //      parser so it wins; the cycle that executes a trial write itself does not count down. ----
            if (trial_t != 0) begin
                if (ps == P_DECIDE && hdr_ok && !(mseq == 0 && mtype == T_REG)) trial_t <= 0;   // committed
                else if (!(ps == P_EXEC && mtype == T_REG && lw[0][0] &&
                           (lw[0][31:16] == REG_IDELAY || lw[0][31:16] == REG_PHYOPT))) begin
                    if (trial_t == 32'd1) begin
                        idelay_tap <= tap_prev; idelay_ld <= 1'b1;
                        rxdly <= rxd_prev; adv1000 <= adv_prev;
                        if (rxdly != rxd_prev || adv1000 != adv_prev) reinit <= 1'b1;
                    end
                    trial_t <= trial_t - 1;
                end
            end

            // ---- ACK timing ----
            if (nak_t != 0) nak_t <= nak_t - 1;
            if (unacked >= ACK_EVERY) ack_now <= 1'b1;
            if (unacked != 0) begin
                if (ack_t >= ACK_TO_CYCLES) ack_now <= 1'b1; else ack_t <= ack_t + 1;
            end else ack_t <= 0;
            // heartbeat: an ACK HB_CYCLES after the last one went out, while a host is known
            if (hb_t >= HB_CYCLES) begin if (host_valid) ack_now <= 1'b1; end
            else hb_t <= hb_t + 1;

            // ---- periodic STATUS ----
            if (st_t >= STATUS_PERIOD) begin st_t <= 0; if (host_valid) status_now <= 1'b1; end
            else st_t <= st_t + 1;

            // ---- reply channel bookkeeping ----
            if (a_start) begin ack_now <= 1'b0; unacked <= 0; ack_t <= 0; hb_t <= 0; nak_f <= 1'b0; full_f <= 1'b0; end
            if (r_done[1]) lr_pend <= 1'b0;
            if (s_start) status_now <= 1'b0;
        end
    end

    // ---------------- reply serializers ----------------
    // ACK: 6 words, snapshot taken when the request starts
    reg [2:0]  a_i; reg [31:0] a_w [0:5];
    always @(posedge clk) begin
        if (rst) begin a_busy <= 1'b0; c_ack <= 0; end
        else begin
            if (a_start) begin
                a_busy <= 1'b1; a_i <= 0; c_ack <= c_ack + 1;
                a_w[0] <= hdr_w0(R_ACK); a_w[1] <= 32'h0; a_w[2] <= exp_seq;
                a_w[3] <= {30'h0, full_f, nak_f}; a_w[4] <= q_ok ? 32'({q_free, 5'b00000}) : 32'h0; a_w[5] <= 32'h0;
            end else if (a_busy) begin
                if (r_valid[0] && r_ready[0]) a_i <= a_i + 1'b1;
                if (r_done[0]) a_busy <= 1'b0;
            end
        end
    end
    // STATUS: 2 header words + STATUS_WORDS, snapshot at start
    reg [6:0]  s_i; reg [31:0] s_w [0:STATUS_WORDS+1];
    integer k;
    always @(posedge clk) begin
        if (rst) s_busy <= 1'b0;
        else if (s_start) begin
            s_busy <= 1'b1; s_i <= 0;
        end else if (s_busy) begin
            if (r_valid[2] && r_ready[2]) s_i <= s_i + 1'b1;
            if (r_done[2]) s_busy <= 1'b0;
        end
    end
    // the snapshot itself without the reset term (timing at 120 MHz: ~rst in every s_w flop's clock enable would arrive
    // 8 ns from the reset replica; a snapshot taken during reset is replaced at the next s_start)
    always @(posedge clk) if (s_start) begin
        s_w[0] <= hdr_w0(R_STATUS); s_w[1] <= 32'h0;
        for (k = 0; k < STATUS_WORDS; k = k + 1) s_w[k + 2] <= status_vec[k*32 +: 32];
    end
    // local reply: 4 words
    reg [1:0] l_i;
    always @(posedge clk) begin
        if (rst || !lr_pend) l_i <= 0;
        else if (r_valid[1] && r_ready[1]) l_i <= l_i + 1'b1;
    end

    assign r_req   = {s_busy, lr_pend, a_busy};
    assign r_len   = {16'(8 + 4*STATUS_WORDS), 16'd16, 16'd24};
    assign r_valid = {s_busy, lr_pend, a_busy};
    assign r_data  = {s_w[s_i], lr_w[l_i], a_w[a_i]};
endmodule
`default_nettype wire
