// eth_net_rx — frame parser behind eth_mac_rx. Reads one
// committed (FCS-checked) frame at a time from the MAC ring, one byte per clock, and handles:
//   * ARP request for my_ip            -> arp_req (sender MAC/IP) for the TX side (only while have_ip)
//   * any ARP request or reply         -> acd_hit when another station has acd_ip as its sender address, or
//                                         probes for it (sender 0.0.0.0), while acd_on (RFC 5227 conflict)
//   * IPv4 header: version 4, IHL >= 5, header checksum, no fragments; destination my_ip (to our MAC), or
//     255.255.255.255, or any address to our MAC for DHCP
//   * ICMP echo request to my_ip       -> message copied into the ICMP buffer, icmp_req for the TX side
//                                         with the reply checksum (RFC 1624 incremental update 8 -> 0)
//   * UDP to my_ip:MY_PORT             -> descriptor (d0..d3) + payload words into the payload FIFO
//                                         (net_pkg word format). The whole datagram is dropped if the
//                                         FIFO lacks room for it, so the FIFO never holds a partial one.
//   * UDP 67 -> 68 (DHCP to the client) -> payload into the DHCP buffer, dh_done with its length and the
//                                         frame's source MAC; dropped while dh_busy
// Everything else is counted and dropped. Address issue stops at a barrier after the last UDP header
// byte until the drop/accept decision is made and the descriptor is written.
`default_nettype none
module eth_net_rx #(
    parameter [47:0] MY_MAC  = 48'h00_0a_35_01_02_03,
    parameter [15:0] MY_PORT = 16'd5000,
    parameter int    RING_AW = 15,
    parameter int    ICMP_AW = 11,            // 2 KB ICMP buffer
    parameter int    PF_AW   = 12,            // payload FIFO depth 2**PF_AW words
    parameter int    DH_AW   = 11             // DHCP buffer: 2**DH_AW bytes
)(
    input  wire               clk, rst,
    // the address in use and the address being probed (they change rarely; see eth_stack)
    input  wire [31:0]        my_ip,
    input  wire               have_ip,
    input  wire [31:0]        acd_ip,
    input  wire               acd_on,
    output reg                acd_hit,
    // DHCP buffer (write side)
    input  wire               dh_busy,
    output reg                dh_we,
    output reg  [DH_AW-1:0]   dh_wa,
    output reg  [7:0]         dh_wd,
    output reg                dh_done,        // a whole datagram is in the buffer
    output reg  [15:0]        dh_len,
    output reg  [47:0]        dh_smac,
    // committed frames from eth_mac_rx
    input  wire               f_avail,
    input  wire [13:0]        f_len,
    input  wire [RING_AW-1:0] f_start,
    output reg  [RING_AW-1:0] rd_addr,
    input  wire [7:0]         ring_data,      // ring byte at rd_addr, one cycle after rd_addr
    output reg                f_release,
    // ARP reply request (held until the TX side takes it)
    output reg                arp_req,
    output reg  [47:0]        arp_mac,
    output reg  [31:0]        arp_ip,
    input  wire               arp_ack,
    // ICMP echo reply request (buffer is owned by TX until icmp_ack)
    output reg                icmp_req,
    output reg  [47:0]        icmp_mac,
    output reg  [31:0]        icmp_ip,
    output reg  [15:0]        icmp_len,
    output reg  [15:0]        icmp_csum,
    input  wire               icmp_ack,
    input  wire [ICMP_AW-1:0] icmp_raddr,
    output reg  [7:0]         icmp_rdata,
    // payload FIFO (write side)
    output reg                pf_wen,
    output reg  [35:0]        pf_wdata,
    input  wire [PF_AW:0]     pf_wcount,
    // counters
    output reg  [31:0]        c_arp, c_icmp, c_udp, c_ipcsum, c_other, c_pfdrop, c_icmpdrop
);
    import net_pkg::*;

    (* ram_style = "block" *) reg [7:0] icmp_buf [0:(1<<ICMP_AW)-1];
    reg              ib_we;
    reg [ICMP_AW-1:0] ib_wa;
    reg [7:0]        ib_wd;
    always @(posedge clk) begin
        if (ib_we) icmp_buf[ib_wa] <= ib_wd;
        icmp_rdata <= icmp_buf[icmp_raddr];
    end

    typedef enum logic [1:0] {S_IDLE, S_RUN, S_DRAIN, S_GAP} st_t;
    st_t st;
    reg [13:0] flen, ai;                       // frame length, next index to issue
    reg        p_v;  reg [13:0] p_idx;         // issued this cycle (data next cycle)
    reg        r_v;  reg [13:0] r_idx;         // ring_data holds byte r_idx when r_v
    reg        d_v;  reg [13:0] bi;            // rd_data (registered ring_data) holds byte bi when d_v
    reg  [7:0] rd_data;                        // register after the block RAM: keeps the parse logic off the BRAM clock-to-out (125 MHz)
    reg        done;                           // stop issuing, release when drained

    // parse context
    reg        is_me, is_bc;
    reg [47:0] smac;
    reg [15:0] etype;
    reg        arp_ok;
    reg [1:0]  arp_op;
    reg [31:0] spa, tpa;
    reg        sha_me, spa_acd, spa_zero;
    reg        d_hi_me, d_hi_bc, d_me, d_bc;   // destination address = my_ip / broadcast (upper 3 bytes, all 4)
    reg        dmatch;                         // IPv4 to my_ip at our MAC
    reg        dh_go;
    reg [3:0]  ihl;
    reg [13:0] hdr_end;                        // first byte after the IPv4 header
    reg [15:0] tl, csum;
    reg [7:0]  cs_hi;
    reg        frag_ok, ip_ok;
    reg [7:0]  proto;
    reg [31:0] sip, dip;
    reg [13:0] ip_end;                         // 14 + total length
    // ICMP
    reg        icmp_go;
    reg [7:0]  icmp_type;
    reg [15:0] icmp_ck;
    // UDP
    reg [15:0] sport, dport, ulen, plen;
    reg [13:0] barrier;                        // do not issue index barrier+1 while bar_hold
    reg        bar_hold;
    reg        udp_go;
    reg [2:0]  dsc;                            // descriptor write step (4..1), 0 = idle
    reg [31:0] pw;                             // payload word being assembled
    reg [15:0] pk;                             // payload byte index

    wire [13:0] udp_dec = hdr_end + 14'd7;     // last UDP header byte
    wire        issue = (st == S_RUN) && !done && (ai < flen) && !(bar_hold && (ai > barrier));
    wire [PF_AW:0] pf_free = (1 << PF_AW) - pf_wcount;

    function automatic [7:0] mac_byte(input [47:0] m, input [2:0] k);
        mac_byte = m[47 - 8*k -: 8];
    endfunction

    // the IPv4 destination decided at the last header byte (byte 33 when IHL = 5: its last address byte is live)
    wire dm_now   = (bi == 14'd33) ? (d_hi_me && rd_data == my_ip[7:0]) : d_me;
    wire db_now   = (bi == 14'd33) ? (d_hi_bc && rd_data == 8'hFF) : d_bc;
    wire dmatch_n = dm_now && have_ip && is_me;

    always @(posedge clk) begin
        f_release <= 1'b0; pf_wen <= 1'b0; ib_we <= 1'b0; dh_we <= 1'b0; dh_done <= 1'b0; acd_hit <= 1'b0;
        if (arp_ack)  arp_req  <= 1'b0;
        if (icmp_ack) icmp_req <= 1'b0;
        if (rst) begin
            st <= S_IDLE; p_v <= 0; r_v <= 0; d_v <= 0; done <= 0; arp_req <= 0; icmp_req <= 0; dsc <= 0;
            c_arp <= 0; c_icmp <= 0; c_udp <= 0; c_ipcsum <= 0; c_other <= 0; c_pfdrop <= 0; c_icmpdrop <= 0;
        end else begin
            // ---------------- issue stage ----------------
            p_v <= issue;
            if (issue) begin rd_addr <= f_start + RING_AW'(ai); p_idx <= ai; ai <= ai + 1'b1; end
            r_v <= p_v; r_idx <= p_idx;
            d_v <= r_v; bi <= r_idx; rd_data <= ring_data;

            // ---------------- descriptor writer ----------------
            if (dsc != 0) begin
                pf_wen <= 1'b1;
                case (dsc)
                    3'd4: pf_wdata <= {K_D0, 2'd3, sport, plen};
                    3'd3: pf_wdata <= {K_DN, 2'd3, sip};
                    3'd2: pf_wdata <= {K_DN, 2'd3, smac[31:0]};
                    default: pf_wdata <= {K_DN, 2'd3, 16'h0, smac[47:32]};
                endcase
                dsc <= dsc - 1'b1;
                if (dsc == 3'd1) bar_hold <= 1'b0;
            end

            case (st)
                S_IDLE: if (f_avail) begin
                    st <= S_RUN; flen <= f_len; ai <= 0; done <= 0;
                    is_me <= 1'b1; is_bc <= 1'b1; arp_ok <= 1'b1; ip_ok <= 1'b1; frag_ok <= 1'b1;
                    csum <= 16'h0; ihl <= 4'd0; hdr_end <= 14'h3FFF; ip_end <= 14'h3FFF;
                    icmp_go <= 1'b0; udp_go <= 1'b0; dh_go <= 1'b0; dmatch <= 1'b0; bar_hold <= 1'b0; barrier <= 14'h3FFF;
                    pk <= 0; pw <= 0;
                end
                S_RUN: begin
                    if (d_v && !done) begin
                        // -------- Ethernet header --------
                        if (bi < 14'd6) begin
                            if (rd_data != mac_byte(MY_MAC, bi[2:0])) is_me <= 1'b0;
                            if (rd_data != 8'hFF) is_bc <= 1'b0;
                        end else if (bi < 14'd12) smac <= {smac[39:0], rd_data};
                        else if (bi == 14'd12) etype[15:8] <= rd_data;
                        else if (bi == 14'd13) begin
                            etype[7:0] <= rd_data;
                            if (!(is_me || is_bc)) begin done <= 1'b1; c_other <= c_other + 1; end
                            else if ({etype[15:8], rd_data} == 16'h0806) begin
                                if (flen < 14'd42) begin done <= 1'b1; c_other <= c_other + 1; end
                            end else if ({etype[15:8], rd_data} == 16'h0800) begin
                                if (flen < 14'd34) begin done <= 1'b1; c_other <= c_other + 1; end
                            end else begin done <= 1'b1; c_other <= c_other + 1; end
                        end
                        // -------- ARP --------
                        else if (etype == 16'h0806) begin
                            case (bi)
                                14'd14: if (rd_data != 8'h00) arp_ok <= 1'b0;
                                14'd15: if (rd_data != 8'h01) arp_ok <= 1'b0;
                                14'd16: if (rd_data != 8'h08) arp_ok <= 1'b0;
                                14'd17: if (rd_data != 8'h00) arp_ok <= 1'b0;
                                14'd18: if (rd_data != 8'h06) arp_ok <= 1'b0;
                                14'd19: if (rd_data != 8'h04) arp_ok <= 1'b0;
                                14'd20: if (rd_data != 8'h00) arp_ok <= 1'b0;
                                14'd21: begin                                  // operation: 1 request, 2 reply
                                    arp_op <= rd_data[1:0];
                                    if (rd_data != 8'h01 && rd_data != 8'h02) arp_ok <= 1'b0;
                                end
                                14'd22, 14'd23, 14'd24, 14'd25, 14'd26, 14'd27: smac <= {smac[39:0], rd_data};
                                14'd28, 14'd29, 14'd30, 14'd31: spa <= {spa[23:0], rd_data};
                                14'd34: begin sha_me <= (smac == MY_MAC); spa_acd <= (spa == acd_ip); spa_zero <= (spa == 32'h0); end
                                14'd38, 14'd39, 14'd40: tpa <= {tpa[23:0], rd_data};
                                14'd41: begin
                                    done <= 1'b1;
                                    if (arp_ok && acd_on && !sha_me &&
                                        (spa_acd || (arp_op == 2'd1 && spa_zero && {tpa[23:0], rd_data} == acd_ip)))
                                        acd_hit <= 1'b1;
                                    if (arp_ok && arp_op == 2'd1 && have_ip && {tpa[23:0], rd_data} == my_ip && !arp_req) begin
                                        arp_req <= 1'b1; arp_mac <= smac; arp_ip <= spa; c_arp <= c_arp + 1;
                                    end else c_other <= c_other + 1;
                                end
                                default: ;
                            endcase
                        end
                        // -------- IPv4 header --------
                        else if (bi < hdr_end || bi == 14'd14) begin
                            // checksum over the header, 16-bit big-endian words from byte 14
                            if (!bi[0]) cs_hi <= rd_data;
                            else        csum <= oc_add(csum, {cs_hi, rd_data});
                            case (bi)
                                14'd14: begin
                                    ihl <= rd_data[3:0];
                                    hdr_end <= 14'd14 + {rd_data[3:0], 2'b00};
                                    if (rd_data[7:4] != 4'd4 || rd_data[3:0] < 4'd5) ip_ok <= 1'b0;
                                end
                                14'd16: tl[15:8] <= rd_data;
                                14'd17: tl[7:0]  <= rd_data;
                                14'd20: if (rd_data[5:0] != 6'd0) frag_ok <= 1'b0;   // MF + offset[12:8]
                                14'd21: if (rd_data != 8'd0)      frag_ok <= 1'b0;
                                14'd23: proto <= rd_data;
                                14'd26, 14'd27, 14'd28, 14'd29: sip <= {sip[23:0], rd_data};
                                14'd30, 14'd31, 14'd32, 14'd33: dip <= {dip[23:0], rd_data};
                                default: ;
                            endcase
                            if (bi == 14'd32) begin
                                d_hi_me <= ({dip[15:0], rd_data} == my_ip[31:8]);
                                d_hi_bc <= ({dip[15:0], rd_data} == 24'hFF_FFFF);
                            end
                            if (bi == 14'd33) begin
                                d_me <= d_hi_me && rd_data == my_ip[7:0];
                                d_bc <= d_hi_bc && rd_data == 8'hFF;
                            end
                            if (bi == hdr_end - 14'd1) begin
                                // decision at the last header byte (the odd byte: csum completes now)
                                if (!ip_ok || oc_add(csum, {cs_hi, rd_data}) != 16'hFFFF) begin
                                    done <= 1'b1; c_ipcsum <= c_ipcsum + 1;
                                end else if (!frag_ok || tl > {2'b00, flen} - 16'd14 ||
                                             tl < {10'd0, ihl, 2'b00} + 16'd8) begin
                                    done <= 1'b1; c_other <= c_other + 1;
                                end else begin
                                    ip_end <= 14'd14 + tl[13:0];
                                    dmatch <= dmatch_n;
                                    if (proto == 8'd1 && dmatch_n) begin
                                        if (icmp_req || (tl - {10'd0, ihl, 2'b00}) > (1 << ICMP_AW)) begin
                                            done <= 1'b1; c_icmpdrop <= c_icmpdrop + 1;
                                        end else begin
                                            icmp_go <= 1'b1; icmp_len <= tl - {10'd0, ihl, 2'b00};
                                        end
                                    end else if (proto == 8'd17 && (dmatch_n || db_now || is_me)) begin
                                        barrier <= hdr_end + 14'd7; bar_hold <= 1'b1;
                                    end else begin done <= 1'b1; c_other <= c_other + 1; end
                                end
                            end
                        end
                        // -------- ICMP message --------
                        else if (icmp_go) begin
                            if (bi < ip_end) begin
                                ib_we <= 1'b1; ib_wa <= bi - hdr_end; ib_wd <= rd_data;
                                if (bi == hdr_end) icmp_type <= rd_data;
                                if (bi == hdr_end + 14'd2) icmp_ck[15:8] <= rd_data;
                                if (bi == hdr_end + 14'd3) icmp_ck[7:0]  <= rd_data;
                            end
                            if (bi == ip_end - 14'd1) begin
                                done <= 1'b1;
                                if (icmp_type == 8'd8) begin
                                    icmp_req <= 1'b1; icmp_mac <= smac; icmp_ip <= sip; c_icmp <= c_icmp + 1;
                                    // HC' = ~(~HC + ~m + m'), m = 0x0800 (type 8, code 0), m' = 0x0000
                                    icmp_csum <= ~oc_add(~icmp_ck, 16'hF7FF);
                                end else c_other <= c_other + 1;
                            end
                        end
                        // -------- UDP --------
                        else if (proto == 8'd17) begin
                            if (bi <= udp_dec) begin
                                case (bi - hdr_end)
                                    14'd0: sport[15:8] <= rd_data;
                                    14'd1: sport[7:0]  <= rd_data;
                                    14'd2: dport[15:8] <= rd_data;
                                    14'd3: dport[7:0]  <= rd_data;
                                    14'd4: ulen[15:8]  <= rd_data;
                                    14'd5: ulen[7:0]   <= rd_data;
                                    default: ;
                                endcase
                                if (bi == udp_dec) begin
                                    if (ulen < 16'd8 || ulen > tl - {10'd0, ihl, 2'b00}) begin
                                        done <= 1'b1; c_other <= c_other + 1; bar_hold <= 1'b0;
                                    end else if (dport == MY_PORT && dmatch) begin
                                        if (pf_free < (((ulen - 16'd8) + 16'd3) >> 2) + 16'd4) begin
                                            done <= 1'b1; c_pfdrop <= c_pfdrop + 1; bar_hold <= 1'b0;
                                        end else begin
                                            plen <= ulen - 16'd8; udp_go <= 1'b1; dsc <= 3'd4;
                                            c_udp <= c_udp + 1;
                                            if (ulen == 16'd8) begin
                                                // empty payload: descriptor only (d0 says len = 0, no payload words)
                                                done <= 1'b1;
                                            end
                                        end
                                    end else if (dport == 16'd68 && sport == 16'd67 && ulen != 16'd8 && !dh_busy &&
                                                 ulen - 16'd8 <= 16'(1 << DH_AW)) begin
                                        plen <= ulen - 16'd8; dh_go <= 1'b1; bar_hold <= 1'b0;
                                    end else begin
                                        done <= 1'b1; c_other <= c_other + 1; bar_hold <= 1'b0;
                                    end
                                end
                            end else if (dh_go) begin
                                dh_we <= 1'b1; dh_wa <= pk[DH_AW-1:0]; dh_wd <= rd_data; pk <= pk + 1'b1;
                                if (pk == plen - 16'd1) begin
                                    done <= 1'b1; dh_done <= 1'b1; dh_len <= plen; dh_smac <= smac;
                                end
                            end else if (udp_go) begin
                                pw[8*pk[1:0] +: 8] <= rd_data;
                                pk <= pk + 1'b1;
                                if (pk[1:0] == 2'd3 || pk == plen - 16'd1) begin
                                    pf_wen <= 1'b1;
                                    pf_wdata <= {(pk == plen - 16'd1) ? K_LAST : K_PAY, pk[1:0],
                                                 (pk[1:0] == 2'd0) ? {24'h0, rd_data} :
                                                 (pk[1:0] == 2'd1) ? {16'h0, rd_data, pw[7:0]} :
                                                 (pk[1:0] == 2'd2) ? {8'h0, rd_data, pw[15:0]} :
                                                                     {rd_data, pw[23:0]}};
                                end
                                if (pk == plen - 16'd1) done <= 1'b1;
                            end
                        end
                    end
                    if (done || (ai == flen && !p_v && !r_v && !d_v)) st <= S_DRAIN;
                end
                S_DRAIN: if (!p_v && !r_v && !d_v && dsc == 0) begin
                    f_release <= 1'b1; st <= S_GAP;
                end
                S_GAP: st <= S_IDLE;                   // f_avail reflects the release one cycle later
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
