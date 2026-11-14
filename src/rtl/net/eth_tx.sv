// eth_tx — frame generator + TX ring + drainer, in the
// Ethernet clock domain. Sources, in priority order:
//   1. ARP reply      (eth_net_rx arp_req)
//   2. ICMP echo reply (eth_net_rx icmp_req; message read from its ICMP buffer, type 8 -> 0, new checksum)
//   3. UDP datagram   (reply FIFO from clk_sys: descriptor d0..d3 + payload words, net_pkg format);
//                      started only once the whole datagram is in the FIFO, so it never underruns.
// The generator writes a complete frame — preamble, SFD, header, payload, zero pad to 60 bytes, FCS —
// into a byte ring and commits its length; generation may stall freely. The drainer sends committed
// frames to rgmii_tx (FWFT valid/data/adv) back to back with a 12-byte inter-frame gap, so writing the
// next frame overlaps sending the current one and 1 Gbit/s replies run at line rate.
`default_nettype none
module eth_tx #(
    parameter [47:0] MY_MAC  = 48'h00_0a_35_01_02_03,
    parameter [31:0] MY_IP   = {8'd192, 8'd168, 8'd1, 8'd10},
    parameter [15:0] MY_PORT = 16'd5000,
    parameter int    ICMP_AW = 11,
    parameter int    RF_AW   = 11,            // reply FIFO depth 2**RF_AW words
    parameter int    TXR_AW  = 13             // 8 KB TX ring
)(
    input  wire               clk, rst,
    input  wire               giga,
    // ARP reply request
    input  wire               arp_req,
    input  wire [47:0]        arp_mac,
    input  wire [31:0]        arp_ip,
    output reg                arp_ack,
    // ICMP echo reply request
    input  wire               icmp_req,
    input  wire [47:0]        icmp_mac,
    input  wire [31:0]        icmp_ip,
    input  wire [15:0]        icmp_len,
    input  wire [15:0]        icmp_csum,
    output reg                icmp_ack,
    output reg  [ICMP_AW-1:0] icmp_raddr,
    input  wire [7:0]         icmp_rdata,
    // reply FIFO read side (FWFT)
    input  wire               rf_valid,
    input  wire [35:0]        rf_data,
    input  wire [RF_AW:0]     rf_count,
    output wire               rf_ren,          // combinational pop of the FWFT head
    // to rgmii_tx
    output wire               tx_valid,
    output wire [7:0]         tx_data,
    input  wire               tx_adv,
    // counters
    output reg  [31:0]        c_tx, c_resync
);
    import net_pkg::*;
    localparam int RING = 1 << TXR_AW;

    // ---------------- TX ring ----------------
    (* ram_style = "block" *) reg [7:0] ring [0:RING-1];
    reg              w_en;
    reg [7:0]        w_byte;
    reg [TXR_AW-1:0] w_adr;
    reg [TXR_AW:0]   wp, wp_frame, rp;
    always @(posedge clk) if (w_en) ring[w_adr] <= w_byte;

    localparam int LF_AW = 4;
    reg [11:0]     lf [0:(1<<LF_AW)-1];
    reg [LF_AW:0]  lf_w, lf_r;
    wire [LF_AW:0] lf_held  = lf_w - lf_r;      // sized differences: they must wrap at the pointer width
    wire           lf_full  = lf_held[LF_AW];
    wire           lf_empty = (lf_w == lf_r);
    wire [TXR_AW:0] ring_held = wp - rp;
    wire [TXR_AW+1:0] ring_free = RING - ring_held;

    // ---------------- generator ----------------
    typedef enum logic [3:0] {G_IDLE, G_DESC, G_CS1, G_CS2, G_CS3, G_PRE, G_HDR, G_IC1,
                              G_PAY, G_PAD, G_FCS, G_COMMIT} gst_t;
    gst_t      gs;
    reg [1:0]  src;                           // 0 ARP, 1 ICMP, 2 UDP
    reg [335:0] hdr;                          // up to 42 header bytes, sent MSB first
    reg [5:0]  hdr_n, hi;                     // header length / index
    reg [15:0] pay_n, pj;                     // payload length / index
    reg [11:0] body;                          // bytes from DA (for padding)
    reg [31:0] crc;
    reg [2:0]  pre_i;
    reg [1:0]  fcs_i, di;
    reg [15:0] ip_id;
    reg [15:0] d_len, d_port;
    reg [31:0] d_ip;
    reg [47:0] d_mac;
    reg [47:0] t_mac; reg [31:0] t_ip; reg [7:0] t_proto; reg [15:0] t_tl;
    reg [19:0] cs_sum;
    reg [15:0] cs;

    wire [15:0] need_words = ((rf_data[15:0] + 16'd3) >> 2) + 16'd4;
    wire        udp_ready  = rf_valid && rf_data[35:34] == K_D0 && rf_count >= need_words;
    // registered: keeps the BRAM clock-to-out + length arithmetic + compare off the state register (125 MHz).
    // Safe: the FIFO head only changes when this generator pops it, and a set udp_ready_q stays true
    // (the count can only grow) until G_DESC consumes the datagram.
    reg         udp_ready_q;
    always @(posedge clk) udp_ready_q <= !rst && udp_ready && (gs == G_IDLE);
    wire        can_start  = !lf_full && ring_free >= 2200;
    // the reply FIFO is first-word-fall-through: a word is consumed in the cycle it is used
    wire        rf_skip    = (gs == G_IDLE) && can_start && !arp_req && !icmp_req && rf_valid && rf_data[35:34] != K_D0;
    wire        rf_desc    = (gs == G_DESC) && rf_valid;
    wire        rf_pay     = (gs == G_PAY) && (src == 2'd2) && (pj[1:0] == 2'd3 || pj == pay_n - 16'd1);
    assign rf_ren = rf_skip | rf_desc | rf_pay;

    task automatic put(input [7:0] b, input do_crc);
        begin
            w_en <= 1'b1; w_byte <= b; w_adr <= wp[TXR_AW-1:0]; wp <= wp + 1'b1;
            if (do_crc) begin crc <= crc32_byte(crc, b); body <= body + 1'b1; end
        end
    endtask

    always @(posedge clk) begin
        w_en <= 1'b0; arp_ack <= 1'b0; icmp_ack <= 1'b0;
        if (rst) begin
            gs <= G_IDLE; wp <= 0; wp_frame <= 0; lf_w <= 0; ip_id <= 16'h0; c_tx <= 0; c_resync <= 0;
        end else case (gs)
            G_IDLE: begin
                crc <= 32'hFFFFFFFF; body <= 0; pre_i <= 0; hi <= 0; pj <= 0; fcs_i <= 0; wp_frame <= wp;
                if (can_start) begin
                    if (arp_req) begin
                        src <= 2'd0; arp_ack <= 1'b1; hdr_n <= 6'd42; pay_n <= 16'd0;
                        hdr <= {arp_mac, MY_MAC, 16'h0806, 16'h0001, 16'h0800, 8'd6, 8'd4, 16'h0002,
                                MY_MAC, MY_IP, arp_mac, arp_ip};
                        gs <= G_PRE;
                    end else if (icmp_req) begin
                        src <= 2'd1; t_mac <= icmp_mac; t_ip <= icmp_ip; t_proto <= 8'd1;
                        t_tl <= icmp_len + 16'd20; hdr_n <= 6'd34; pay_n <= icmp_len;
                        gs <= G_CS1;
                    end else if (rf_skip) begin
                        c_resync <= c_resync + 1;                      // not a datagram start: skipped
                    end else if (udp_ready_q && rf_valid && rf_data[35:34] == K_D0) begin
                        src <= 2'd2; di <= 0; gs <= G_DESC;
                    end
                end
            end
            G_DESC: if (rf_valid) begin            // d0..d3 (FWFT: one per cycle, popped by rf_desc)
                case (di)
                    2'd0: begin d_len <= rf_data[15:0]; d_port <= rf_data[31:16]; end
                    2'd1: d_ip <= rf_data[31:0];
                    2'd2: d_mac[31:0] <= rf_data[31:0];
                    default: d_mac[47:32] <= rf_data[15:0];
                endcase
                if (di != 0 && rf_data[35:34] != K_DN) begin c_resync <= c_resync + 1; gs <= G_IDLE; end
                else if (di == 2'd3) begin
                    t_mac <= {rf_data[15:0], d_mac[31:0]}; t_ip <= d_ip; t_proto <= 8'd17;
                    t_tl <= d_len + 16'd28; hdr_n <= 6'd42; pay_n <= d_len;
                    gs <= G_CS1;
                end
                di <= di + 1'b1;
            end
            G_CS1: begin
                cs_sum <= 20'h4500 + t_tl + ip_id + 20'h4000 + {12'h040, t_proto} +
                          MY_IP[31:16] + MY_IP[15:0] + t_ip[31:16] + t_ip[15:0];
                gs <= G_CS2;
            end
            G_CS2: begin cs_sum <= cs_sum[15:0] + cs_sum[19:16]; gs <= G_CS3; end
            G_CS3: begin
                cs = ~(cs_sum[15:0] + cs_sum[19:16]);
                if (src == 2'd1)
                    hdr <= {t_mac, MY_MAC, 16'h0800, 16'h4500, t_tl, ip_id, 16'h4000, 8'd64, t_proto, cs,
                            MY_IP, t_ip, 64'h0};
                else
                    hdr <= {t_mac, MY_MAC, 16'h0800, 16'h4500, t_tl, ip_id, 16'h4000, 8'd64, t_proto, cs,
                            MY_IP, t_ip, MY_PORT, d_port, d_len + 16'd8, 16'h0000};
                ip_id <= ip_id + 1'b1;
                gs <= G_PRE;
            end
            G_PRE: begin
                put((pre_i == 3'd7) ? 8'hD5 : 8'h55, 1'b0);
                pre_i <= pre_i + 1'b1;
                if (pre_i == 3'd7) gs <= G_HDR;
            end
            G_HDR: begin
                put(hdr[335:328], 1'b1);
                hdr <= {hdr[327:0], 8'h00};
                hi <= hi + 1'b1;
                if (hi == hdr_n - 6'd1) begin
                    if (pay_n == 0)       gs <= G_PAD;
                    else if (src == 2'd1) begin icmp_raddr <= 0; gs <= G_IC1; end
                    else                  gs <= G_PAY;
                end
            end
            // ICMP buffer read pipeline: raddr registered here, buffer output registered one edge later,
            // so the address must run two bytes ahead: HDR(last) raddr=0, IC1 raddr=1, PAY(j) raddr=j+2.
            G_IC1: begin icmp_raddr <= 1; gs <= G_PAY; end
            G_PAY: begin
                if (src == 2'd1) begin
                    // ICMP: type -> 0 (echo reply), checksum replaced, rest echoed
                    put((pj == 16'd0) ? 8'h00 : (pj == 16'd2) ? icmp_csum[15:8] :
                        (pj == 16'd3) ? icmp_csum[7:0] : icmp_rdata, 1'b1);
                    icmp_raddr <= pj[ICMP_AW-1:0] + 2'd2;
                end else begin
                    put(rf_data[8*pj[1:0] +: 8], 1'b1);          // word popped by rf_pay at lane 3 / last
                end
                pj <= pj + 1'b1;
                if (pj == pay_n - 16'd1) begin
                    if (src == 2'd1) icmp_ack <= 1'b1;
                    gs <= G_PAD;
                end
            end
            G_PAD: begin
                if (body < 12'd60) put(8'h00, 1'b1);
                else gs <= G_FCS;
            end
            G_FCS: begin
                put(~crc[8*fcs_i +: 8], 1'b0);
                fcs_i <= fcs_i + 1'b1;
                if (fcs_i == 2'd3) gs <= G_COMMIT;
            end
            G_COMMIT: begin
                lf[lf_w[LF_AW-1:0]] <= wp - wp_frame;
                lf_w <= lf_w + 1'b1;
                c_tx <= c_tx + 1;
                gs <= G_IDLE;
            end
            default: gs <= G_IDLE;
        endcase
    end

    // ---------------- drainer ----------------
    typedef enum logic [1:0] {D_IDLE, D_SEND, D_IFG} dst_t;
    dst_t       ds;
    reg [11:0]  remain;
    reg [5:0]   ifg;
    reg [7:0]   rdata;
    wire        pop = tx_adv && (ds == D_SEND);
    wire [TXR_AW:0] rp_n = rp + pop;
    always @(posedge clk) rdata <= ring[rp_n[TXR_AW-1:0]];
    assign tx_valid = (ds == D_SEND);
    assign tx_data  = rdata;
    always @(posedge clk) begin
        if (rst) begin ds <= D_IDLE; rp <= 0; lf_r <= 0; ifg <= 0; end
        else begin
            rp <= rp_n;
            case (ds)
                D_IDLE: if (!lf_empty) begin remain <= lf[lf_r[LF_AW-1:0]]; lf_r <= lf_r + 1'b1; ds <= D_SEND; end
                D_SEND: if (pop) begin
                            remain <= remain - 1'b1;
                            if (remain == 12'd1) begin ds <= D_IFG; ifg <= giga ? 6'd14 : 6'd26; end
                        end
                D_IFG:  if (ifg == 0) ds <= D_IDLE; else ifg <= ifg - 1'b1;
                default: ds <= D_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
