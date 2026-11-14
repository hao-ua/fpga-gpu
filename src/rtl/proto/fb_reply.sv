// fb_reply — reply datagram writer (clk_sys). Arbitrates NS reply sources (index 0 = highest priority)
// and writes each datagram into the eth_stack reply FIFO: descriptor d0..d3 (destination = the host
// address learned by fb_proto_a) followed by the payload words (net_pkg format). A datagram is started
// only when the FIFO has room for all of it. Without a known host, or if the FIFO stays full for
// DROP_CYCLES (link down), the datagram is dropped (the source still sees s_done).
// Source protocol: hold s_req with s_len (payload bytes, header included) stable; once granted, supply
// ceil(s_len/4) words on s_valid/s_data, each taken when s_ready; s_done pulses at the end.
`default_nettype none
module fb_reply #(
    parameter int NS = 5,
    parameter int RF_AW = 11,
    parameter int DROP_CYCLES = 1_000_000
)(
    input  wire              clk, rst,
    input  wire              host_valid,
    input  wire [47:0]       host_mac,
    input  wire [31:0]       host_ip,
    input  wire [15:0]       host_port,
    input  wire [NS-1:0]     s_req,
    input  wire [NS*16-1:0]  s_len,
    input  wire [NS-1:0]     s_valid,
    input  wire [NS*32-1:0]  s_data,
    output wire [NS-1:0]     s_ready,
    output reg  [NS-1:0]     s_done,
    output reg               rf_wen,
    output reg  [35:0]       rf_wdata,
    input  wire [RF_AW:0]    rf_wcount,
    output reg  [31:0]       c_sent, c_dropped
);
    import net_pkg::*;
    localparam int SW = (NS > 1) ? $clog2(NS) : 1;

    // R_GAP: one cycle after every datagram so the source sees s_done and drops s_req before re-arbitration
    typedef enum logic [2:0] {R_IDLE, R_DESC, R_PAY, R_DROP, R_GAP} rst_t;
    rst_t        st;
    reg [SW-1:0] cur;
    reg [15:0]   len, nw, wi;
    reg [1:0]    di;
    reg [31:0]   wait_c;
    reg [47:0]   d_mac; reg [31:0] d_ip; reg [15:0] d_port;

    // highest-priority request, REGISTERED (timing at 120 MHz: a source's request -> the priority pick -> the length
    // select -> the length / word count registers is 11 levels): the pick made last
    // cycle is used if that source still requests (they hold s_req until s_done); a stale pick costs one idle cycle
    reg          any, any_r; reg [SW-1:0] pick, pick_r;
    integer i;
    always @(*) begin
        any = 1'b0; pick = '0;
        for (i = NS - 1; i >= 0; i = i - 1) if (s_req[i]) begin any = 1'b1; pick = i; end
    end
    always @(posedge clk) begin any_r <= any & ~rst; pick_r <= pick; end
    wire        go_r    = any_r & s_req[pick_r];
    wire [15:0] p_len   = s_len[pick_r*16 +: 16];
    wire [15:0] p_words = (p_len + 16'd3) >> 2;
    wire [RF_AW+1:0] free = (1 << RF_AW) - rf_wcount;

    genvar g;
    generate for (g = 0; g < NS; g = g + 1) begin : g_rdy
        assign s_ready[g] = ((st == R_PAY) || (st == R_DROP && wi != nw)) && (cur == g);
    end endgenerate
    wire        cv = s_valid[cur];
    wire [31:0] cd = s_data[cur*32 +: 32];

    always @(posedge clk) begin
        rf_wen <= 1'b0; s_done <= '0;
        if (rst) begin st <= R_IDLE; c_sent <= 0; c_dropped <= 0; wait_c <= 0; end
        else case (st)
            R_IDLE: if (go_r) begin
                if (!host_valid) begin
                    cur <= pick_r; len <= p_len; nw <= p_words; wi <= 0; st <= R_DROP;
                end else if (free >= p_words + 16'd4) begin
                    cur <= pick_r; len <= p_len; nw <= p_words; wi <= 0; di <= 0;
                    d_mac <= host_mac; d_ip <= host_ip; d_port <= host_port;
                    wait_c <= 0; st <= R_DESC;
                end else if (wait_c >= DROP_CYCLES) begin
                    cur <= pick_r; len <= p_len; nw <= p_words; wi <= 0; wait_c <= 0; st <= R_DROP;
                end else wait_c <= wait_c + 1;
            end
            R_DESC: begin
                rf_wen <= 1'b1;
                case (di)
                    2'd0: rf_wdata <= {K_D0, 2'd3, d_port, len};
                    2'd1: rf_wdata <= {K_DN, 2'd3, d_ip};
                    2'd2: rf_wdata <= {K_DN, 2'd3, d_mac[31:0]};
                    default: rf_wdata <= {K_DN, 2'd3, 16'h0, d_mac[47:32]};
                endcase
                di <= di + 1'b1;
                if (di == 2'd3) st <= (nw == 0) ? R_GAP : R_PAY;
                if (di == 2'd3 && nw == 0) begin s_done[cur] <= 1'b1; c_sent <= c_sent + 1; end
            end
            R_PAY: if (cv) begin
                rf_wen <= 1'b1;
                rf_wdata <= {(wi == nw - 1) ? K_LAST : K_PAY, (wi == nw - 1) ? 2'(len - 16'd1) : 2'd3, cd};
                wi <= wi + 1'b1;
                if (wi == nw - 1) begin st <= R_GAP; s_done[cur] <= 1'b1; c_sent <= c_sent + 1; end
            end
            R_DROP: begin                      // consume the source's words without writing
                if (wi == nw) begin st <= R_GAP; s_done[cur] <= 1'b1; c_dropped <= c_dropped + 1; end
                else if (cv) wi <= wi + 1'b1;
            end
            R_GAP: st <= R_IDLE;
            default: st <= R_IDLE;
        endcase
    end
endmodule
`default_nettype wire
