// dhcp_client — board A's IPv4 address: a DHCP client (RFC 2131) that checks a leased address with ARP probes
// (RFC 5227 timing) before using it, and a fixed fallback address.
//   * Each link-up starts an acquisition: DISCOVER, REQUEST for the first valid OFFER, ACK, ARP probes, BOUND.
//     The address in use stays in use meanwhile. When no lease is bound within FALLBACK_MS of the link coming up
//     (or of the address being dropped), and no REQUEST or probe is under way, FALLBACK_IP is used; the DISCOVERs go
//     on (the back-off tops out at 64 s) and a lease bound later replaces the fallback address.
//   * BOUND: REQUEST to the server at T1 (renewing), broadcast at T2 (rebinding), each retransmitted after half the
//     time left (at least 60 s); a NAK or the end of the lease drops the address.
//   * An ARP conflict while probing sends a DECLINE; the acquisition restarts 10 s later.
//   * Two ARP announcements, 2 s apart, follow each bind.
//   * ENABLE = 0: FALLBACK_IP from reset and no DHCP traffic.
// The datagram buffers are in eth_stack: a received DHCP message is read through rx_addr / rx_data (one-cycle read
// latency), a message to send is written through tx_we / tx_wa / tx_wd. rx_full / rx_rel and tx_req / tx_done are
// four-phase handshakes; rx_len / rx_smac and the tx_* fields hold still while their request is up.
`default_nettype none
module dhcp_client #(
    parameter [47:0]     MY_MAC      = 48'h00_0a_35_01_02_03,
    parameter [31:0]     FALLBACK_IP = {8'd192, 8'd168, 8'd1, 8'd10},
    parameter bit        ENABLE      = 1'b1,
    parameter [8*32-1:0] HOSTNAME    = "fpga-gpu",       // option 12, at most 28 characters
    parameter int        MS_CYCLES   = 100_000,          // clk cycles per millisecond
    parameter int        FALLBACK_MS = 10_000,
    parameter int        RX_AW       = 11,               // receive buffer: 2**RX_AW bytes
    parameter int        TX_AW       = 9                 // transmit buffer: 2**TX_AW bytes (a message is 300)
)(
    input  wire              clk, rst,
    input  wire              link_up,
    // received DHCP message (the UDP payload of a datagram to port 68)
    input  wire              rx_full,
    input  wire [15:0]       rx_len,
    input  wire [47:0]       rx_smac,
    output reg               rx_rel,
    output reg  [RX_AW-1:0]  rx_addr,
    input  wire [7:0]        rx_data,
    // to send: a UDP datagram 68 -> 67 from the transmit buffer, or an ARP request
    output reg               tx_req,
    output reg               tx_arp,          // 1: ARP request, sender address tx_sip, target address tx_dip
    output reg  [47:0]       tx_mac,
    output reg  [31:0]       tx_sip, tx_dip,
    output reg  [15:0]       tx_len, tx_ucs,
    input  wire              tx_done,
    output reg               tx_we,
    output reg  [TX_AW-1:0]  tx_wa,
    output reg  [7:0]        tx_wd,
    // address conflict detection: acd_hit = an ARP from another station has acd_ip as its sender, or probes for it
    output reg  [31:0]       acd_ip,
    output reg               acd_on,
    input  wire              acd_hit,
    // the address in use
    output reg  [31:0]       ip,
    output reg               ip_valid,
    output reg               ip_lease,        // 1: leased, 0: FALLBACK_IP
    output wire [3:0]        state
);
    function automatic int hn_len(input [8*32-1:0] s);
        int n;
        n = 0;
        for (int i = 0; i < 32; i++) if (s[8*i +: 8] != 8'h00) n = i + 1;
        return n;
    endfunction
    localparam int HN = hn_len(HOSTNAME);

    function automatic [7:0] be(input [31:0] v, input [1:0] k);
        be = v[31 - 8*k -: 8];
    endfunction
    function automatic [7:0] mac_b(input [2:0] k);
        mac_b = MY_MAC[47 - 8*k -: 8];
    endfunction
    function automatic [7:0] host_b(input [5:0] k);
        host_b = (k < HN) ? HOSTNAME[8*(HN - 1 - k) +: 8] : 8'h00;
    endfunction
    // a usable unicast address: not 0.x, 127.x, multicast or reserved
    function automatic bit addr_ok(input [31:0] a);
        addr_ok = a[31:24] != 8'd0 && a[31:24] != 8'd127 && a[31:24] < 8'd224;
    endfunction

    // ---------------- time base, random numbers ----------------
    localparam int MSW = $clog2(MS_CYCLES + 1);
    reg [MSW-1:0] ms_div;
    reg [9:0]     ms_cnt;
    reg           tick;                            // one cycle per millisecond
    reg [31:0]    now_s;                           // seconds since reset
    always @(posedge clk) begin
        tick <= 1'b0;
        if (rst) begin ms_div <= 0; ms_cnt <= 0; now_s <= 0; end
        else if (ms_div == MSW'(MS_CYCLES - 1)) begin
            ms_div <= 0; tick <= 1'b1;
            if (ms_cnt == 10'd999) begin ms_cnt <= 0; now_s <= now_s + 1'b1; end
            else ms_cnt <= ms_cnt + 1'b1;
        end else ms_div <= ms_div + 1'b1;
    end
    reg [31:0] lfsr;                               // x^32 + x^22 + x^2 + x + 1, stepped every cycle
    always @(posedge clk)
        if (rst) lfsr <= MY_MAC[31:0] | 32'h1;
        else     lfsr <= lfsr[0] ? ((lfsr >> 1) ^ 32'h80200003) : (lfsr >> 1);

    // ---------------- client state (main FSM below) ----------------
    typedef enum logic [3:0] {S_IDLE, S_DELAY, S_SELECT, S_REQUEST, S_ACK, S_PROBE, S_BOUND, S_RENEW, S_REBIND,
                              S_HOLD} st_t;
    st_t st;
    assign state = st;
    reg [31:0] xid, acq_s;                         // transaction id, start of the acquisition / renewal (secs)
    reg [31:0] off_yi, off_srv;                    // the OFFER being requested
    reg [31:0] n_yi;                               // the address being probed
    reg [31:0] srv_ip;                             // server identifier of the lease
    reg [47:0] srv_mac;                            // where the server's ACK came from (renewals go there)
    wire [31:0] el = now_s - acq_s;
    wire [15:0] secs = (el[31:16] != 0) ? 16'hFFFF : el[15:0];

    // ---------------- sender: builds a message into the transmit buffer, then hands it over ----------------
    localparam [2:0] K_DISCOVER = 3'd0, K_REQ_SEL = 3'd1, K_REQ_RENEW = 3'd2, K_REQ_REBIND = 3'd3, K_DECLINE = 3'd4,
                     K_PROBE = 3'd5, K_ANNOUNCE = 3'd6;
    reg        snd_go;                             // main FSM: send snd_kind (only while snd_idle)
    reg [2:0]  snd_kind;
    typedef enum logic [2:0] {T_IDLE, T_INIT, T_BUILD, T_SUM1, T_SUM2, T_REQ, T_LOW} ts_t;
    ts_t ts;
    wire       snd_idle = (ts == T_IDLE) && !snd_go;
    reg [8:0]  wa;                                 // message byte being written (0..299)
    reg [2:0]  seg;                                // option being written: 53 61 50 54 12 55 end, 7 = pad
    reg [5:0]  sj;
    reg [7:0]  b_inc;                              // options of this message (bit = seg), bit 7 = pad
    reg [7:0]  b_mt;
    reg [31:0] b_xid, b_ci, b_rip, b_srv;
    reg [15:0] b_secs;
    reg        b_bc;
    reg [23:0] sum;
    reg [7:0]  sum_hi;
    reg [16:0] fold;
    reg [15:0] us;
    reg [7:0]  bb;

    function automatic [5:0] seg_len(input [2:0] s);
        case (s)
            3'd0:    seg_len = 6'd3;               // 53 message type
            3'd1:    seg_len = 6'd9;               // 61 client identifier: type 1 + MAC
            3'd2:    seg_len = 6'd6;               // 50 requested address
            3'd3:    seg_len = 6'd6;               // 54 server identifier
            3'd4:    seg_len = 6'(HN + 2);         // 12 host name
            3'd5:    seg_len = 6'd6;               // 55 parameter request list: mask, router, T1, T2
            default: seg_len = 6'd1;               // 255 end
        endcase
    endfunction

    // fixed part: op, htype, hlen, hops, xid, secs, flags, ciaddr, yiaddr / siaddr / giaddr 0, chaddr, sname / file 0,
    // magic cookie
    wire [7:0] fix_b =
        (wa <= 9'd1) ? 8'd1 : (wa == 9'd2) ? 8'd6 :
        (wa >= 9'd4 && wa <= 9'd7) ? be(b_xid, wa[1:0]) :
        (wa == 9'd8) ? b_secs[15:8] : (wa == 9'd9) ? b_secs[7:0] :
        (wa == 9'd10) ? {b_bc, 7'd0} :
        (wa >= 9'd12 && wa <= 9'd15) ? be(b_ci, wa[1:0]) :
        (wa >= 9'd28 && wa <= 9'd33) ? mac_b(3'(wa - 9'd28)) :
        (wa == 9'd236 || wa == 9'd239) ? 8'd99 : (wa == 9'd237) ? 8'd130 : (wa == 9'd238) ? 8'd83 : 8'h00;
    reg [7:0] opt_b;
    always @* begin
        case (seg)
            3'd0: opt_b = (sj == 0) ? 8'd53 : (sj == 1) ? 8'd1 : b_mt;
            3'd1: opt_b = (sj == 0) ? 8'd61 : (sj == 1) ? 8'd7 : (sj == 2) ? 8'd1 : mac_b(3'(sj - 6'd3));
            3'd2: opt_b = (sj == 0) ? 8'd50 : (sj == 1) ? 8'd4 : be(b_rip, 2'(sj - 6'd2));
            3'd3: opt_b = (sj == 0) ? 8'd54 : (sj == 1) ? 8'd4 : be(b_srv, 2'(sj - 6'd2));
            3'd4: opt_b = (sj == 0) ? 8'd12 : (sj == 1) ? 8'(HN) : host_b(sj - 6'd2);
            3'd5: opt_b = (sj == 0) ? 8'd55 : (sj == 1) ? 8'd4 : (sj == 2) ? 8'd1 : (sj == 3) ? 8'd3 :
                          (sj == 4) ? 8'd58 : 8'd59;
            default: opt_b = 8'd255;
        endcase
    end

    always @(posedge clk) begin
        tx_we <= 1'b0;
        if (rst) begin ts <= T_IDLE; tx_req <= 1'b0; tx_arp <= 1'b0; end
        else case (ts)
            T_IDLE: if (snd_go) begin
                if (snd_kind == K_PROBE || snd_kind == K_ANNOUNCE) begin
                    // RFC 5227: probe = sender 0.0.0.0, announcement = sender and target the address
                    tx_arp <= 1'b1; tx_mac <= 48'hFFFF_FFFF_FFFF; tx_len <= 16'd0; tx_ucs <= 16'd0;
                    tx_sip <= (snd_kind == K_PROBE) ? 32'h0 : ip;
                    tx_dip <= (snd_kind == K_PROBE) ? n_yi : ip;
                    tx_req <= 1'b1; ts <= T_REQ;
                end else begin
                    tx_arp <= 1'b0; tx_len <= 16'd300;
                    tx_mac <= (snd_kind == K_REQ_RENEW) ? srv_mac : 48'hFFFF_FFFF_FFFF;
                    tx_dip <= (snd_kind == K_REQ_RENEW) ? srv_ip : 32'hFFFF_FFFF;
                    tx_sip <= (snd_kind == K_REQ_RENEW || snd_kind == K_REQ_REBIND) ? ip : 32'h0;
                    b_xid <= xid; b_secs <= (snd_kind == K_DECLINE) ? 16'd0 : secs;
                    b_mt  <= (snd_kind == K_DISCOVER) ? 8'd1 : (snd_kind == K_DECLINE) ? 8'd4 : 8'd3;
                    b_bc  <= (snd_kind == K_DISCOVER || snd_kind == K_REQ_SEL);
                    b_ci  <= (snd_kind == K_REQ_RENEW || snd_kind == K_REQ_REBIND) ? ip : 32'h0;
                    b_rip <= (snd_kind == K_DECLINE) ? n_yi : (snd_kind == K_REQ_SEL) ? off_yi : ip;
                    b_srv <= (snd_kind == K_DECLINE) ? srv_ip : off_srv;
                    // bits: 0 = 53, 1 = 61, 2 = 50, 3 = 54, 4 = 12, 5 = 55, 6 = end, 7 = pad
                    b_inc <= {1'b1, 1'b1, (snd_kind != K_DECLINE), (snd_kind != K_DECLINE),
                              (snd_kind == K_REQ_SEL || snd_kind == K_DECLINE),
                              (snd_kind == K_REQ_SEL || snd_kind == K_DECLINE ||
                               (snd_kind == K_DISCOVER && ip_valid && ip_lease)),
                              1'b1, 1'b1};
                    wa <= 0; seg <= 0; sj <= 0;
                    ts <= T_INIT;
                end
            end
            T_INIT: begin
                // UDP checksum: pseudo header (addresses, protocol 17, length 308) + UDP header (68, 67, 308)
                sum <= 24'(tx_sip[31:16]) + 24'(tx_sip[15:0]) + 24'(tx_dip[31:16]) + 24'(tx_dip[15:0]) +
                       24'd17 + 24'd308 + 24'd68 + 24'd67 + 24'd308;
                ts <= T_BUILD;
            end
            T_BUILD: begin
                if (wa == 9'd300) ts <= T_SUM1;
                else if (wa >= 9'd240 && !b_inc[seg]) begin seg <= seg + 1'b1; sj <= 0; end
                else begin
                    bb = (wa < 9'd240) ? fix_b : (seg == 3'd7) ? 8'h00 : opt_b;
                    tx_we <= 1'b1; tx_wa <= TX_AW'(wa); tx_wd <= bb; wa <= wa + 1'b1;
                    if (!wa[0]) sum_hi <= bb; else sum <= sum + {8'd0, sum_hi, bb};
                    if (wa >= 9'd240 && seg != 3'd7) begin
                        if (sj == seg_len(seg) - 6'd1) begin seg <= seg + 1'b1; sj <= 0; end
                        else sj <= sj + 1'b1;
                    end
                end
            end
            T_SUM1: begin fold <= 17'(sum[15:0]) + 17'(sum[23:16]); ts <= T_SUM2; end
            T_SUM2: begin
                us = fold[15:0] + 16'(fold[16]);
                tx_ucs <= (us == 16'hFFFF) ? 16'hFFFF : ~us;      // a computed 0 is sent as FFFF (RFC 768)
                tx_req <= 1'b1; ts <= T_REQ;
            end
            T_REQ: if (tx_done || !link_up) begin tx_req <= 1'b0; ts <= T_LOW; end
            T_LOW: if (!tx_done) ts <= T_IDLE;
            default: ts <= T_IDLE;
        endcase
    end

    // ---------------- parser: reads a received message from the buffer, three cycles per byte ----------------
    typedef enum logic [2:0] {P_IDLE, P_RD, P_W, P_USE, P_EVAL, P_DONE} ps_t;
    ps_t ps;
    typedef enum logic [1:0] {O_CODE, O_LEN, O_DATA} os_t;
    os_t os;
    reg        ev;                                 // a parsed message waits for the main FSM (no new parse)
    reg [11:0] p, p_end;
    reg        fixed;                              // in the fixed part (bytes 0..239)
    reg [1:0]  region;                             // option regions: 0 options, 1 file, 2 sname (option overload)
    reg        ok;
    reg [7:0]  ocode, olen, ocnt;
    reg [23:0] oval;
    reg [1:0]  ovl;
    reg [31:0] p_xid, p_yi, p_srv, p_lease, p_t1, p_t2;
    reg [7:0]  p_mt;
    reg        p_srv_p, p_lease_p, p_t1_p, p_t2_p;
    reg [47:0] p_smac;
    reg        pr_done;
    wire [7:0]  rb = rx_data;
    wire [31:0] v4 = {oval, rb};
    always @(posedge clk) begin
        pr_done <= 1'b0;
        if (rst) begin ps <= P_IDLE; rx_rel <= 1'b0; end
        else case (ps)
            P_IDLE: begin
                if (rx_rel && !rx_full) rx_rel <= 1'b0;
                if (rx_full && !rx_rel && !ev) begin
                    p <= 0; fixed <= 1'b1; region <= 0; os <= O_CODE; ovl <= 0;
                    ok <= (rx_len >= 16'd244) && (rx_len <= 16'(1 << RX_AW));
                    p_mt <= 0; p_srv_p <= 0; p_lease_p <= 0; p_t1_p <= 0; p_t2_p <= 0; p_smac <= rx_smac;
                    ps <= P_RD;
                end
            end
            P_RD: begin
                if (!fixed && p >= p_end) begin
                    // end of an option region
                    if (region == 2'd0 && ovl[0]) begin region <= 2'd1; p <= 12'd108; p_end <= 12'd236; os <= O_CODE; end
                    else if (region != 2'd2 && ovl[1]) begin region <= 2'd2; p <= 12'd44; p_end <= 12'd108; os <= O_CODE; end
                    else ps <= P_EVAL;
                end else begin rx_addr <= p[RX_AW-1:0]; ps <= P_W; end
            end
            P_W: ps <= P_USE;
            P_USE: begin
                ps <= P_RD;
                if (fixed) begin
                    if (p == 12'd0 && rb != 8'd2) ok <= 1'b0;                  // BOOTREPLY
                    if (p == 12'd1 && rb != 8'd1) ok <= 1'b0;                  // Ethernet
                    if (p == 12'd2 && rb != 8'd6) ok <= 1'b0;
                    if (p >= 12'd4 && p <= 12'd7) p_xid <= {p_xid[23:0], rb};
                    if (p >= 12'd16 && p <= 12'd19) p_yi <= {p_yi[23:0], rb};
                    if (p >= 12'd28 && p <= 12'd33 && rb != mac_b(3'(p - 12'd28))) ok <= 1'b0;
                    if ((p == 12'd236 && rb != 8'd99) || (p == 12'd237 && rb != 8'd130) || (p == 12'd238 && rb != 8'd83))
                        ok <= 1'b0;
                    if (p == 12'd239) begin
                        fixed <= 1'b0; p <= 12'd240; p_end <= rx_len[11:0];
                        if (!ok || rb != 8'd99) ps <= P_DONE;
                    end else p <= p + 1'b1;
                end else case (os)
                    O_CODE: begin
                        if (rb == 8'd255) p <= p_end;                          // end: the region is done
                        else if (rb == 8'd0) p <= p + 1'b1;                    // pad
                        else begin ocode <= rb; os <= O_LEN; p <= p + 1'b1; end
                    end
                    O_LEN: begin olen <= rb; ocnt <= 0; p <= p + 1'b1; os <= (rb == 8'd0) ? O_CODE : O_DATA; end
                    default: begin
                        oval <= {oval[15:0], rb};
                        if (ocnt == 8'd0 && olen == 8'd1) begin
                            if (ocode == 8'd53) p_mt <= rb;
                            if (ocode == 8'd52 && region == 2'd0) ovl <= rb[1:0];
                        end
                        if (ocnt == 8'd3 && olen == 8'd4) case (ocode)
                            8'd54: begin p_srv <= v4; p_srv_p <= 1'b1; end
                            8'd51: begin p_lease <= v4; p_lease_p <= 1'b1; end
                            8'd58: begin p_t1 <= v4; p_t1_p <= 1'b1; end
                            8'd59: begin p_t2 <= v4; p_t2_p <= 1'b1; end
                            default: ;
                        endcase
                        ocnt <= ocnt + 1'b1; p <= p + 1'b1;
                        if (ocnt == olen - 8'd1) os <= O_CODE;
                    end
                endcase
            end
            P_EVAL: begin if (ok) pr_done <= 1'b1; ps <= P_DONE; end
            P_DONE: begin rx_rel <= 1'b1; ps <= P_IDLE; end
            default: ps <= P_IDLE;
        endcase
    end

    // ---------------- main FSM ----------------
    reg        lu_q, fresh;                        // fresh: a lease was bound since the link came up
    reg [19:0] fb_ms;                              // fallback countdown
    reg [16:0] tmr;                                // retransmission / probe countdown, ms
    reg [2:0]  k;                                  // retransmissions so far
    reg [1:0]  pc;                                 // probes sent
    reg [1:0]  ann_n;                              // announcements still to send
    reg [11:0] ann_tmr;
    reg        acd_c;                              // conflict seen while probing
    reg [31:0] t_req;                              // when the REQUEST was sent (the lease counts from it)
    reg [31:0] l_t1, l_t2, l_exp, rt_at;
    reg        l_inf;
    // the ACK being taken
    reg [31:0] a_yi, a_srv, a_lease, a_t1, a_t2;
    reg        a_srv_p, a_t1_p, a_t2_p;
    reg [47:0] a_smac;
    reg [1:0]  ak;
    reg [31:0] r_d1, r_d2, r_t1, r_t2;
    reg [31:0] n_t1, n_t2, n_exp;
    reg        n_inf;
    reg [31:0] h_t2, h_exp;                        // retransmission delay while renewing / rebinding

    function automatic [16:0] backoff(input [2:0] kk, input [10:0] r);
        backoff = (17'd4000 << kk) - 17'd1024 + 17'(r);         // 4, 8, 16, 32, 64 s, +-1 s
    endfunction
    wire [31:0] d_t1 = now_s - l_t1, d_t2 = now_s - l_t2, d_exp = now_s - l_exp, d_rt = now_s - rt_at;
    wire        past_t1 = !d_t1[31], past_t2 = !d_t2[31], past_exp = !d_exp[31], past_rt = !d_rt[31];
    wire [31:0] r_t2l = l_t2 - now_s, r_expl = l_exp - now_s;
    always @(posedge clk) begin
        h_t2  <= (r_t2l[31]  || r_t2l[31:1]  < 32'd60) ? 32'd60 : {1'b0, r_t2l[31:1]};
        h_exp <= (r_expl[31] || r_expl[31:1] < 32'd60) ? 32'd60 : {1'b0, r_expl[31:1]};
    end
    wire expired = ip_valid && ip_lease && !l_inf && past_exp;
    wire fb_due  = !fresh && fb_ms == 0 && !(ip_valid && !ip_lease) &&
                   (st == S_DELAY || st == S_SELECT || st == S_HOLD);

    task automatic snd(input [2:0] kind);
        begin snd_go <= 1'b1; snd_kind <= kind; end
    endtask
    task automatic start_acq(input [16:0] delay);
        begin st <= S_DELAY; tmr <= delay; xid <= lfsr; acq_s <= now_s; k <= 0; acd_on <= 1'b0; end
    endtask
    task automatic drop_addr;
        begin ip_valid <= 1'b0; ip_lease <= 1'b0; fresh <= 1'b0; fb_ms <= 20'(FALLBACK_MS); end
    endtask

    always @(posedge clk) begin
        snd_go <= 1'b0;
        lu_q <= link_up;
        if (tick && tmr != 0) tmr <= tmr - 1'b1;
        if (tick && ann_tmr != 0) ann_tmr <= ann_tmr - 1'b1;
        if (tick && fb_ms != 0) fb_ms <= fb_ms - 1'b1;
        if (pr_done) ev <= 1'b1;
        if (acd_hit && acd_on) acd_c <= 1'b1;
        if (rst) begin
            st <= S_IDLE; ev <= 1'b0; acd_c <= 1'b0; acd_on <= 1'b0; acd_ip <= 32'h0; lu_q <= 1'b0;
            ip <= ENABLE ? 32'h0 : FALLBACK_IP; ip_valid <= !ENABLE; ip_lease <= 1'b0;
            fresh <= 1'b0; fb_ms <= 0; tmr <= 0; k <= 0; pc <= 0; ann_n <= 0; ann_tmr <= 0; ak <= 0;
            l_inf <= 1'b1; l_t1 <= 0; l_t2 <= 0; l_exp <= 0; rt_at <= 0; acq_s <= 0; xid <= 0;
            srv_ip <= 0; srv_mac <= 0; off_yi <= 0; off_srv <= 0; n_yi <= 0;
        end else if (!ENABLE) ev <= 1'b0;
        else if (!link_up) begin
            st <= S_IDLE; acd_on <= 1'b0; acd_c <= 1'b0; ev <= 1'b0;
            if (expired) begin ip_valid <= 1'b0; ip_lease <= 1'b0; end
        end else if (!lu_q) begin
            // the link came up: a new acquisition after a random 0..1 s; the fallback countdown starts
            start_acq(17'(lfsr[9:0]));
            fresh <= 1'b0; fb_ms <= 20'(FALLBACK_MS);
        end else if (expired) begin
            drop_addr();
            if (st == S_BOUND || st == S_RENEW || st == S_REBIND) start_acq(17'(lfsr[9:0]));
        end else if (fb_due) begin
            ip <= FALLBACK_IP; ip_valid <= 1'b1; ip_lease <= 1'b0;
        end else if (snd_idle) begin
            if (ev) begin
                ev <= 1'b0;
                if (p_xid == xid) case (st)
                    S_SELECT: if (p_mt == 8'd2 && p_srv_p && addr_ok(p_yi)) begin
                        off_yi <= p_yi; off_srv <= p_srv;
                        snd(K_REQ_SEL); t_req <= now_s; st <= S_REQUEST; k <= 0; tmr <= backoff(3'd0, lfsr[10:0]);
                    end
                    S_REQUEST: begin
                        if (p_mt == 8'd5 && (!p_srv_p || p_srv == off_srv) && p_yi == off_yi) begin
                            a_yi <= p_yi; a_srv <= p_srv; a_srv_p <= p_srv_p; a_smac <= p_smac;
                            a_lease <= p_lease_p ? p_lease : 32'hFFFF_FFFF;
                            a_t1 <= p_t1; a_t1_p <= p_t1_p; a_t2 <= p_t2; a_t2_p <= p_t2_p;
                            ak <= 0; st <= S_ACK;
                        end else if (p_mt == 8'd6 && (!p_srv_p || p_srv == off_srv))
                            start_acq(17'd1000 + 17'(lfsr[9:0]));
                    end
                    S_RENEW, S_REBIND: begin
                        if (p_mt == 8'd5 && addr_ok(p_yi)) begin
                            a_yi <= p_yi; a_srv <= p_srv; a_srv_p <= p_srv_p; a_smac <= p_smac;
                            a_lease <= p_lease_p ? p_lease : 32'hFFFF_FFFF;
                            a_t1 <= p_t1; a_t1_p <= p_t1_p; a_t2 <= p_t2; a_t2_p <= p_t2_p;
                            ak <= 0; st <= S_ACK;
                        end else if (p_mt == 8'd6) begin
                            drop_addr(); start_acq(17'd1000 + 17'(lfsr[9:0]));
                        end
                    end
                    default: ;
                endcase
            end else if (ann_n != 0 && ann_tmr == 0 && (st == S_BOUND || st == S_RENEW || st == S_REBIND)) begin
                snd(K_ANNOUNCE); ann_n <= ann_n - 1'b1; ann_tmr <= 12'd2000;
            end else case (st)
                S_DELAY: if (tmr == 0) begin
                    snd(K_DISCOVER); st <= S_SELECT; k <= 0; tmr <= backoff(3'd0, lfsr[10:0]);
                end
                S_SELECT: if (tmr == 0) begin
                    snd(K_DISCOVER);
                    if (k != 3'd4) k <= k + 1'b1;
                    tmr <= backoff((k == 3'd4) ? 3'd4 : k + 1'b1, lfsr[10:0]);
                end
                S_REQUEST: if (tmr == 0) begin
                    if (k == 3'd2) start_acq(17'd0);
                    else begin snd(K_REQ_SEL); t_req <= now_s; k <= k + 1'b1; tmr <= backoff(k + 1'b1, lfsr[10:0]); end
                end
                S_ACK: begin
                    // lease times relative to the REQUEST: T1 / T2 from the ACK if they are ordered, else 1/2 and 7/8
                    ak <= ak + 1'b1;
                    case (ak)
                        2'd0: begin
                            r_d1 <= {1'b0, a_lease[31:1]}; r_d2 <= a_lease - {3'd0, a_lease[31:3]};
                            r_t1 <= a_t1_p ? a_t1 : {1'b0, a_lease[31:1]};
                            r_t2 <= a_t2_p ? a_t2 : a_lease - {3'd0, a_lease[31:3]};
                        end
                        2'd1: if (!(r_t1 < r_t2 && r_t2 < a_lease)) begin r_t1 <= r_d1; r_t2 <= r_d2; end
                        2'd2: begin
                            // FFFFFFFF = infinite; leases of 2^31 s and more are taken as infinite too (the timers
                            // compare with a signed difference)
                            n_t1 <= t_req + r_t1; n_t2 <= t_req + r_t2; n_exp <= t_req + a_lease;
                            n_inf <= a_lease[31];
                        end
                        default: begin
                            srv_ip <= a_srv_p ? a_srv : off_srv; srv_mac <= a_smac;
                            if (ip_valid && ip_lease && ip == a_yi) begin
                                // the address in use, extended
                                l_t1 <= n_t1; l_t2 <= n_t2; l_exp <= n_exp; l_inf <= n_inf;
                                fresh <= 1'b1; st <= S_BOUND; ann_n <= 2'd2; ann_tmr <= 0;
                            end else begin
                                n_yi <= a_yi; acd_ip <= a_yi; acd_on <= 1'b1; acd_c <= 1'b0; pc <= 0;
                                tmr <= 17'(lfsr[9:0]);                           // PROBE_WAIT
                                st <= S_PROBE;
                            end
                        end
                    endcase
                end
                S_PROBE: begin
                    if (acd_c) begin
                        snd(K_DECLINE); acd_on <= 1'b0; acd_c <= 1'b0; st <= S_HOLD; tmr <= 17'd10_000;
                    end else if (tmr == 0) begin
                        if (pc != 2'd3) begin
                            snd(K_PROBE); pc <= pc + 1'b1;
                            tmr <= (pc == 2'd2) ? 17'd2000 : 17'd1000 + 17'(lfsr[9:0]);   // ANNOUNCE_WAIT after the last
                        end else begin
                            ip <= n_yi; ip_valid <= 1'b1; ip_lease <= 1'b1;
                            l_t1 <= n_t1; l_t2 <= n_t2; l_exp <= n_exp; l_inf <= n_inf;
                            fresh <= 1'b1; acd_on <= 1'b0; st <= S_BOUND; ann_n <= 2'd2; ann_tmr <= 0;
                        end
                    end
                end
                S_BOUND: if (!l_inf && past_t1) begin
                    xid <= lfsr; acq_s <= now_s;
                    snd(K_REQ_RENEW); t_req <= now_s; st <= S_RENEW; rt_at <= now_s + h_t2;
                end
                S_RENEW: begin
                    if (past_t2) begin
                        xid <= lfsr;
                        snd(K_REQ_REBIND); t_req <= now_s; st <= S_REBIND; rt_at <= now_s + h_exp;
                    end else if (past_rt) begin
                        snd(K_REQ_RENEW); t_req <= now_s; rt_at <= now_s + h_t2;
                    end
                end
                S_REBIND: if (past_rt) begin
                    snd(K_REQ_REBIND); t_req <= now_s; rt_at <= now_s + h_exp;
                end
                S_HOLD: if (tmr == 0) start_acq(17'd0);
                default: ;
            endcase
        end
    end
endmodule
`default_nettype wire
