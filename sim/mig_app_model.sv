`timescale 1ns/1ps
// mig_app_model — behavioural MIG 7-series native-port model for the remote-display benches.
// * app_rdy / app_wdf_rdy are randomised (RDY_PCT / WDF_PCT percent of cycles high).
// * Commands and write data are queued separately and paired in order (data may precede its command).
// * Commands execute strictly in order (MIG "Strict" ordering): a write applies its byte-masked data
//   when it reaches the head, a read samples memory then and returns RD_LAT cycles later.
// * Checks: burst-aligned addresses (app_addr[2:0] == 0), rank bit 28 == 0, app_wdf_end with every beat,
//   no read data lost. Violations increment `errors` and print.
// Memory: sparse, keyed by burst index (app_addr >> 3), 32 bytes per burst, unwritten bytes read as 0.
`default_nettype none
module mig_app_model #(
    parameter int RD_LAT  = 24,
    parameter int RDY_PCT = 80,
    parameter int WDF_PCT = 85,
    parameter int SEED    = 1
)(
    input  wire         ui_clk, rst,
    input  wire         app_en,
    input  wire [2:0]   app_cmd,
    input  wire [28:0]  app_addr,
    input  wire [255:0] app_wdf_data,
    input  wire [31:0]  app_wdf_mask,
    input  wire         app_wdf_wren, app_wdf_end,
    output reg          app_rdy,
    output reg          app_wdf_rdy,
    output reg  [255:0] app_rd_data,
    output reg          app_rd_data_valid,
    output int          errors,
    output longint      n_wr, n_rd
);
    reg [255:0] mem [longint];
    typedef struct { bit rd; longint a; } cmd_t;
    typedef struct { bit [255:0] d; bit [31:0] m; } dat_t;
    typedef struct { longint due; bit [255:0] d; } ret_t;
    cmd_t cq[$]; dat_t dq[$]; ret_t rq[$];
    longint cyc = 0;
    int unsigned rs = SEED;
    // DDR timing variation (the real MIG: refresh / bank conflicts): +MIGJIT_n adds 0..n random cycles to every read's
    // latency (returns stay in order), +MIGSTALL_n holds app_rdy / app_wdf_rdy low for 1..n cycles now and then (0.5 %
    // of cycles start a stall), +MIGSEED_s seeds the model's random stream instead of the SEED parameter
    int jit = 0, stl = 0, mseed = 0, stall_left = 0; longint last_due = 0;
    initial begin
        void'($value$plusargs("MIGJIT_%d", jit)); void'($value$plusargs("MIGSTALL_%d", stl));
        if ($value$plusargs("MIGSEED_%d", mseed)) rs = mseed;
    end

    function automatic bit [255:0] rd_mem(longint b);
        rd_mem = mem.exists(b) ? mem[b] : 256'h0;
    endfunction

    initial begin errors = 0; n_wr = 0; n_rd = 0; app_rdy = 0; app_wdf_rdy = 0; app_rd_data_valid = 0; end

    always @(posedge ui_clk) begin
        cyc <= cyc + 1;
        app_rd_data_valid <= 1'b0;
        if (rst) begin
            cq.delete(); dq.delete(); rq.delete(); app_rdy <= 0; app_wdf_rdy <= 0;
        end else begin
            // accept command / data (handshake evaluated with the values presented this cycle)
            if (app_en && app_rdy) begin
                if (app_addr[2:0] != 0 || app_addr[28]) begin
                    errors = errors + 1; $display("%t MIG_MODEL ERROR: bad address %h", $time, app_addr);
                end
                if (app_cmd != 3'b000 && app_cmd != 3'b001) begin
                    errors = errors + 1; $display("%t MIG_MODEL ERROR: bad command %0d", $time, app_cmd);
                end
                cq.push_back('{rd: (app_cmd == 3'b001), a: app_addr >> 3});
            end
            if (app_wdf_wren && app_wdf_rdy) begin
                if (!app_wdf_end) begin errors = errors + 1; $display("%t MIG_MODEL ERROR: wdf_end low", $time); end
                dq.push_back('{d: app_wdf_data, m: app_wdf_mask});
            end
            // execute the head command
            if (cq.size() > 0) begin
                if (cq[0].rd) begin : RD
                    longint d; rs = rs * 1103515245 + 12345;
                    d = cyc + RD_LAT + (jit > 0 ? longint'((rs >> 16) % (jit + 1)) : 0);
                    if (d <= last_due) d = last_due + 1;                // in order, one per cycle at most
                    last_due = d;
                    rq.push_back('{due: d, d: rd_mem(cq[0].a)});
                    void'(cq.pop_front()); n_rd = n_rd + 1;
                end else if (dq.size() > 0) begin
                    bit [255:0] cur; cur = rd_mem(cq[0].a);
                    for (int b = 0; b < 32; b++) if (!dq[0].m[b]) cur[b*8 +: 8] = dq[0].d[b*8 +: 8];
                    mem[cq[0].a] = cur;
                    void'(cq.pop_front()); void'(dq.pop_front()); n_wr = n_wr + 1;
                end
            end
            if (rq.size() > 0 && rq[0].due <= cyc) begin
                app_rd_data_valid <= 1'b1; app_rd_data <= rq[0].d; void'(rq.pop_front());
            end
            // randomised ready for the next cycle (and the occasional long stall)
            if (stall_left > 0) begin stall_left = stall_left - 1; app_rdy <= 1'b0; app_wdf_rdy <= 1'b0; end
            else begin
                rs = rs * 1103515245 + 12345; app_rdy     <= ((rs >> 16) % 100) < RDY_PCT && cq.size() < 16;
                rs = rs * 1103515245 + 12345; app_wdf_rdy <= ((rs >> 16) % 100) < WDF_PCT && dq.size() < 16;
                if (stl > 0) begin rs = rs * 1103515245 + 12345; if (((rs >> 16) % 1000) < 5) stall_left = 1 + int'((rs >> 8) % stl); end
            end
        end
    end

    // bench access to memory contents
    function automatic bit [7:0] peek_byte(longint byte_addr);
        bit [255:0] b; b = rd_mem(byte_addr >> 5);
        peek_byte = b[(byte_addr & 31) * 8 +: 8];
    endfunction
endmodule
`default_nettype wire
