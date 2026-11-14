// mem_arb — parameterised multi-port arbiter for the MIG native port. Port contract and rules:
//   read port  i: r_en, r_addr -> r_rdy (command accepted), r_valid (its data is on the shared rd_data)
//   write port j: w_req (wants bus ownership for a transaction), w_en/w_addr (command),
//                 w_data/w_mask/w_wren/w_wend (write data), -> w_rdy, w_wdf_rdy
// * Command channel: reads in port order (0 = highest), then the current write owner.
// * Write-ownership FSM: one writer at a time owns the command *and* data channels, so MIG's in-order
//   pairing of write commands with write data holds. Ownership goes round-robin among requesting
//   writers. The owner keeps it until it drops w_req (allowed only when its commands and data are
//   balanced) or, while another writer waits, until it has issued W_QUANTUM commands and its accepted
//   commands and data beats are balanced.
// * Read-owner FIFO: the issuing port id is pushed per accepted read and popped per returned beat
//   (MIG runs with strict ordering). Ports outside MUST_RUN cannot take the last RO_RESERVE slots.
// * Aging: a port that requests without being granted for STARVE_MAX cycles becomes urgent; while any
//   port is urgent only urgent ports (and MUST_RUN reads) may issue.
// Each command is one 32-byte burst (BL8, 4:1): app_addr in 4-byte units, burst aligned (addr[2:0]=0).
`default_nettype none
module mem_arb #(
    parameter int        NR = 2,
    parameter int        NW = 2,
    parameter int        RO_AW = 5,                 // read-owner FIFO depth 2**RO_AW
    parameter int        RO_RESERVE = 4,
    parameter [NR-1:0]   MUST_RUN = '0,
    parameter int        STARVE_MAX = 255,
    parameter int        W_QUANTUM = 32,
    parameter bit        RD_REG = 1'b1                  // the read return registered once before the ports (timing at
                                                        // 120 MHz: the MIG's read FIFO flags at the DDR pins would reach
                                                        // the ROP's load registers across the die directly);
                                                        // every read's data one cycle later (the ports wait on r_valid)
)(
    input  wire                 clk, rst,
    // read ports
    input  wire [NR-1:0]        r_en,
    input  wire [NR*29-1:0]     r_addr,
    output wire [NR-1:0]        r_rdy,
    output wire [NR-1:0]        r_valid,
    output wire [255:0]         rd_data,
    // write ports
    input  wire [NW-1:0]        w_req, w_en,
    input  wire [NW*29-1:0]     w_addr,
    input  wire [NW*256-1:0]    w_data,
    input  wire [NW*32-1:0]     w_mask,
    input  wire [NW-1:0]        w_wren, w_wend,
    output wire [NW-1:0]        w_rdy, w_wdf_rdy,
    // MIG native port
    output wire                 app_en,
    output wire [2:0]           app_cmd,
    output wire [28:0]          app_addr,
    output wire [255:0]         app_wdf_data,
    output wire [31:0]          app_wdf_mask,
    output wire                 app_wdf_wren, app_wdf_end,
    input  wire                 app_rdy, app_wdf_rdy,
    input  wire                 app_rd_data_valid,
    input  wire [255:0]         app_rd_data
);
    localparam int IDW = (NR > 1) ? $clog2(NR) : 1;
    localparam int WIW = (NW > 1) ? $clog2(NW) : 1;
    localparam [2:0] CMD_WRITE = 3'b000, CMD_READ = 3'b001;

    integer i, k;

    // ---------------- state ----------------
    reg  [IDW-1:0]  ro [0:(1<<RO_AW)-1];
    reg  [RO_AW:0]  ro_w, ro_r;
    reg             wo_v;
    reg  [WIW-1:0]  wo, rr;
    reg  signed [7:0] bal;                         // owner: data beats accepted - commands accepted
    reg  [7:0]      qcnt;
    reg  [7:0]      sc_r [0:NR-1];
    reg  [7:0]      sc_w [0:NW-1];
    reg  [NR-1:0]   urg_r;
    reg  [NW-1:0]   urg_w;
    reg  [NR-1:0]   g_r;
    reg             g_w;
    reg  [IDW-1:0]  g_id;

    wire [RO_AW:0]  ro_cnt  = ro_w - ro_r;
    wire            ro_full = ro_cnt == (1 << RO_AW);
    wire            ro_bulk = ro_cnt >= (1 << RO_AW) - RO_RESERVE;
    wire            any_urg = |urg_r | |urg_w;

    // ---------------- command grant ----------------
    always @(*) begin
        g_r = '0; g_w = 1'b0; g_id = '0;
        for (i = NR - 1; i >= 0; i = i - 1)
            if (r_en[i] && (MUST_RUN[i] ? !ro_full : !ro_bulk) && (MUST_RUN[i] || urg_r[i] || !any_urg)) begin
                g_r = '0; g_r[i] = 1'b1; g_id = i;
            end
        if (g_r == '0 && wo_v && w_en[wo] && (urg_w[wo] || !any_urg)) g_w = 1'b1;
    end

    assign app_en   = (|g_r) | g_w;
    assign app_cmd  = (|g_r) ? CMD_READ : CMD_WRITE;
    assign app_addr = (|g_r) ? r_addr[g_id*29 +: 29] : w_addr[wo*29 +: 29];
    assign r_rdy    = g_r & {NR{app_rdy}};
    genvar gj;
    generate for (gj = 0; gj < NW; gj = gj + 1) begin : g_wp
        assign w_rdy[gj]     = g_w && (wo == gj) && app_rdy;
        assign w_wdf_rdy[gj] = wo_v && (wo == gj) && app_wdf_rdy;
    end endgenerate

    // ---------------- write data channel (owner only) ----------------
    assign app_wdf_data = w_data[wo*256 +: 256];
    assign app_wdf_mask = w_mask[wo*32 +: 32];
    assign app_wdf_wren = wo_v && w_wren[wo];
    assign app_wdf_end  = wo_v && w_wend[wo];

    // ---------------- write ownership ----------------
    reg             nxt_v; reg [WIW-1:0] nxt; reg others;
    always @(*) begin
        nxt_v = 1'b0; nxt = '0; others = 1'b0;
        for (k = NW - 1; k >= 0; k = k - 1)
            if (w_req[(rr + k) % NW]) begin nxt_v = 1'b1; nxt = (rr + k) % NW; end
        for (k = 0; k < NW; k = k + 1) if (w_req[k] && k != wo) others = 1'b1;
    end
    wire own_d  = app_wdf_wren & app_wdf_rdy;
    wire own_c  = g_w & app_rdy;
    wire rotate = wo_v && others && (bal == 0) && !own_d && !own_c && (qcnt >= W_QUANTUM);
    always @(posedge clk) begin
        if (rst) begin wo_v <= 1'b0; wo <= '0; rr <= '0; bal <= 0; qcnt <= 0; end
        else if (!wo_v || !w_req[wo] || rotate) begin
            wo_v <= nxt_v; wo <= nxt; bal <= 0; qcnt <= 0;
            if (nxt_v) rr <= (nxt + 1) % NW;
        end else begin
            bal <= bal + (own_d ? 8'sd1 : 8'sd0) - (own_c ? 8'sd1 : 8'sd0);
            if (own_c && qcnt != 8'hFF) qcnt <= qcnt + 1'b1;
        end
    end

    // ---------------- read data return ----------------
    wire [IDW-1:0] ro_head = ro[ro_r[RO_AW-1:0]];
    reg            rdv_r; reg [255:0] rdd_r; reg [IDW-1:0] roh_r;    // the return registered (RD_REG)
    always @(posedge clk) begin
        rdv_r <= app_rd_data_valid & ~rst; rdd_r <= app_rd_data; roh_r <= ro_head;
    end
    assign rd_data = RD_REG ? rdd_r : app_rd_data;
    generate for (gj = 0; gj < NR; gj = gj + 1) begin : g_rv
        assign r_valid[gj] = RD_REG ? (rdv_r && (roh_r == gj)) : (app_rd_data_valid && (ro_head == gj));
    end endgenerate
    always @(posedge clk) begin
        if (rst) begin ro_w <= 0; ro_r <= 0; end
        else begin
            if ((|g_r) && app_rdy) begin ro[ro_w[RO_AW-1:0]] <= g_id; ro_w <= ro_w + 1'b1; end
            if (app_rd_data_valid) ro_r <= ro_r + 1'b1;
        end
    end

    // ---------------- aging counters ----------------
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < NR; i = i + 1) begin sc_r[i] <= 0; urg_r[i] <= 1'b0; end
            for (i = 0; i < NW; i = i + 1) begin sc_w[i] <= 0; urg_w[i] <= 1'b0; end
        end else begin
            for (i = 0; i < NR; i = i + 1) begin
                if (r_en[i] && !r_rdy[i]) begin if (sc_r[i] != 8'hFF) sc_r[i] <= sc_r[i] + 1'b1; end
                else sc_r[i] <= 0;
                urg_r[i] <= r_en[i] && !r_rdy[i] && (sc_r[i] >= STARVE_MAX);
            end
            for (i = 0; i < NW; i = i + 1) begin
                if (wo_v && wo == i && w_en[i] && !w_rdy[i]) begin if (sc_w[i] != 8'hFF) sc_w[i] <= sc_w[i] + 1'b1; end
                else sc_w[i] <= 0;
                urg_w[i] <= wo_v && wo == i && w_en[i] && !w_rdy[i] && (sc_w[i] >= STARVE_MAX);
            end
        end
    end
endmodule
`default_nettype wire
