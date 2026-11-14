`timescale 1ns/1ps
// tb_mem — mem_arb (2R+2W) + ddr_fifo (R0/W0) + xfer_lin (R1/W1) against mig_app_model.
//  1. ddr_fifo: 4000 random bursts pushed and popped with random stalls through a 64-burst ring
//     (wraps ~60 times, hits full); order and data checked, concurrently with 2-3.
//  2. xfer_lin: 300 random WRITE / READ operations, any alignment, length 1..700, checked byte-exact
//     against a reference byte map (unwritten bytes = 0).
//  3. MEMTEST write+verify over 64 KB -> 0 errors; one word corrupted in the model, verify-only ->
//     exactly 1 error at the corrupted address.
// Prints TB_PASS / TB_FAIL.
`default_nettype none
module tb_mem;
    reg clk = 0, rst = 1;
    always #5 clk = ~clk;                      // 100 MHz

    // ---------------- MIG model + arbiter ----------------
    wire        app_en, app_wdf_wren, app_wdf_end, app_rdy, app_wdf_rdy, app_rd_data_valid;
    wire [2:0]  app_cmd; wire [28:0] app_addr; wire [255:0] app_wdf_data, app_rd_data; wire [31:0] app_wdf_mask;
    int         mig_err; longint mig_nwr, mig_nrd;
    mig_app_model #(.RD_LAT(24), .RDY_PCT(75), .WDF_PCT(80), .SEED(7)) u_mig (
        .ui_clk(clk), .rst(rst), .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr),
        .app_wdf_data(app_wdf_data), .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren),
        .app_wdf_end(app_wdf_end), .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy),
        .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid), .errors(mig_err),
        .n_wr(mig_nwr), .n_rd(mig_nrd));

    wire [1:0] r_en, r_rdy, r_valid; wire [57:0] r_addr; wire [255:0] rd_data;
    wire [1:0] w_req, w_en, w_wren, w_wend, w_rdy, w_wdf_rdy; wire [57:0] w_addr;
    wire [511:0] w_data; wire [63:0] w_mask;
    mem_arb #(.NR(2), .NW(2), .RO_AW(5), .RO_RESERVE(4), .MUST_RUN(2'b00), .W_QUANTUM(8)) u_arb (
        .clk(clk), .rst(rst), .r_en(r_en), .r_addr(r_addr), .r_rdy(r_rdy), .r_valid(r_valid), .rd_data(rd_data),
        .w_req(w_req), .w_en(w_en), .w_addr(w_addr), .w_data(w_data), .w_mask(w_mask), .w_wren(w_wren),
        .w_wend(w_wend), .w_rdy(w_rdy), .w_wdf_rdy(w_wdf_rdy),
        .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end),
        .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy), .app_rd_data_valid(app_rd_data_valid), .app_rd_data(app_rd_data));

    // ---------------- ddr_fifo on port 0 ----------------
    reg          q_in_v = 0; reg [255:0] q_in_d; wire q_in_rdy;
    wire         q_out_v; wire [255:0] q_out_d; reg q_out_rdy = 0; wire [6:0] q_used;
    ddr_fifo #(.BASE(29'h0), .RING_LOG2(6), .IN_AW(4), .OUT_AW(4)) u_q (
        .clk(clk), .rst(rst), .in_valid(q_in_v), .in_data(q_in_d), .in_ready(q_in_rdy),
        .out_valid(q_out_v), .out_data(q_out_d), .out_ready(q_out_rdy), .used(q_used),
        .w_req(w_req[0]), .w_en(w_en[0]), .w_addr(w_addr[28:0]), .w_data(w_data[255:0]), .w_mask(w_mask[31:0]),
        .w_wren(w_wren[0]), .w_wend(w_wend[0]), .w_rdy(w_rdy[0]), .w_wdf_rdy(w_wdf_rdy[0]),
        .r_en(r_en[0]), .r_addr(r_addr[28:0]), .r_rdy(r_rdy[0]), .r_valid(r_valid[0]), .rd_data(rd_data));

    // ---------------- xfer_lin on port 1 ----------------
    reg         x_cv = 0; reg [1:0] x_op; reg [29:0] x_addr, x_len; reg [31:0] x_seed; reg [1:0] x_flags;
    wire        x_crdy, x_busy;
    reg         wd_v = 0; reg [31:0] wd_d; wire wd_rdy;
    wire        rdo_v; wire [31:0] rdo_d; reg rdo_rdy = 0;
    wire        mt_done; wire [31:0] mt_err; wire [29:0] mt_bad;
    xfer_lin u_x (
        .clk(clk), .rst(rst), .cmd_valid(x_cv), .cmd_op(x_op), .cmd_addr(x_addr), .cmd_len(x_len),
        .cmd_seed(x_seed), .cmd_flags(x_flags), .cmd_ready(x_crdy), .busy(x_busy),
        .wd_valid(wd_v), .wd_data(wd_d), .wd_ready(wd_rdy),
        .rd_valid_o(rdo_v), .rd_data_o(rdo_d), .rd_ready_o(rdo_rdy),
        .mt_done(mt_done), .mt_errors(mt_err), .mt_first_bad(mt_bad),
        .w_req(w_req[1]), .w_en(w_en[1]), .w_addr(w_addr[57:29]), .w_data(w_data[511:256]), .w_mask(w_mask[63:32]),
        .w_wren(w_wren[1]), .w_wend(w_wend[1]), .w_rdy(w_rdy[1]), .w_wdf_rdy(w_wdf_rdy[1]),
        .r_en(r_en[1]), .r_addr(r_addr[57:29]), .r_rdy(r_rdy[1]), .r_valid(r_valid[1]), .rdata(rd_data));

    int errs = 0;
    int unsigned rnd = 32'h1234;
    function automatic int unsigned rr();
        rnd = rnd * 1103515245 + 12345; return rnd >> 8;
    endfunction

    // ---------------- 1. ddr_fifo traffic ----------------
    localparam int NQ = 4000;
    bit [255:0] qexp [$];
    int qpushed = 0, qpopped = 0;
    always @(posedge clk) if (!rst) begin
        if (q_in_v && q_in_rdy) begin qexp.push_back(q_in_d); qpushed++; end
        if (q_out_v && q_out_rdy) begin
            if (qexp.size() == 0) begin errs++; $display("FIFO ERROR: pop with nothing expected"); end
            else begin
                if (q_out_d !== qexp[0]) begin
                    int pos = -1;
                    for (int z = 0; z < qexp.size(); z++) if (qexp[z] === q_out_d) begin pos = z; break; end
                    errs++; if (errs < 10) $display("FIFO ERROR at %0d: got %h exp %h (got = expected[+%0d]) pushed=%0d used=%0d wr_cmd=%0d rd_cmd=%0d",
                        qpopped, q_out_d[63:0], qexp[0][63:0], pos, qpushed, q_used, u_q.wr_cmd, u_q.rd_cmd);
                end
                void'(qexp.pop_front());
            end
            qpopped++;
        end
        if (!(q_in_v && !q_in_rdy)) begin
            q_in_v <= (qpushed + (q_in_v && q_in_rdy) < NQ) && (rr() % 100 < 60);
            q_in_d <= {rr(), rr(), rr(), rr(), rr(), rr(), rr(), rr()};
        end
        q_out_rdy <= (rr() % 100) < ((qpopped < NQ/2) ? 25 : 70);   // first half: let it fill
    end

    // ---------------- 2/3. xfer_lin operations ----------------
    bit [7:0] ref_mem [longint];
    function automatic bit [7:0] refb(longint a); return ref_mem.exists(a) ? ref_mem[a] : 8'h00; endfunction

    task automatic do_write(input longint addr, input int len);
        bit [7:0] bytes [$]; int nw;
        for (int i = 0; i < len; i++) begin bytes.push_back(rr()); ref_mem[addr + i] = bytes[i]; end
        @(posedge clk); x_op <= 2'd0; x_addr <= addr; x_len <= len; x_cv <= 1'b1;
        @(posedge clk); while (!x_crdy) @(posedge clk); x_cv <= 1'b0;
        nw = (len + 3) / 4;
        for (int w = 0; w < nw; w++) begin
            bit [31:0] word = 32'hDEADBEEF;      // bytes past len are garbage on purpose
            for (int b = 0; b < 4; b++) if (4*w + b < len) word[8*b +: 8] = bytes[4*w + b];
            wd_v <= 1'b1; wd_d <= word;
            @(posedge clk); while (!wd_rdy) @(posedge clk);
            // random gap
            wd_v <= 1'b0; repeat (rr() % 3) @(posedge clk);
        end
        wd_v <= 1'b0;
        while (x_busy) @(posedge clk);
    endtask

    task automatic do_read(input longint addr, input int len);
        int nw, got = 0;
        @(posedge clk); x_op <= 2'd1; x_addr <= addr; x_len <= len; x_cv <= 1'b1;
        @(posedge clk); while (!x_crdy) @(posedge clk); x_cv <= 1'b0;
        nw = (len + 3) / 4;
        while (got < nw) begin
            rdo_rdy <= (rr() % 100) < 70;
            @(posedge clk);
            if (rdo_v && rdo_rdy) begin
                for (int b = 0; b < 4; b++) if (4*got + b < len)
                    if (rdo_d[8*b +: 8] !== refb(addr + 4*got + b)) begin
                        errs++;
                        if (errs < 20) $display("READ ERROR addr %h+%0d: got %h exp %h (len %0d)",
                                                addr, 4*got + b, rdo_d[8*b +: 8], refb(addr + 4*got + b), len);
                    end
                got++;
            end
        end
        rdo_rdy <= 1'b0;
        repeat (3) @(posedge clk);
        if (rdo_v) begin errs++; $display("READ ERROR: extra word after %0d", nw); end
        while (x_busy) @(posedge clk);
    endtask

    task automatic do_memtest(input longint addr, input int len, input bit [31:0] seed, input bit [1:0] flags,
                              input int exp_err, input longint exp_bad);
        @(posedge clk); x_op <= 2'd2; x_addr <= addr; x_len <= len; x_seed <= seed; x_flags <= flags; x_cv <= 1'b1;
        @(posedge clk); while (!x_crdy) @(posedge clk); x_cv <= 1'b0;
        @(posedge clk); while (!mt_done) @(posedge clk);
        if (mt_err != exp_err || (exp_err > 0 && mt_bad != exp_bad)) begin
            errs++; $display("MEMTEST ERROR: errors %0d (exp %0d) first_bad %h (exp %h)", mt_err, exp_err, mt_bad, exp_bad);
        end else $display("MEMTEST ok: flags %b errors %0d first_bad %h", flags, mt_err, mt_bad);
        while (x_busy) @(posedge clk);
    endtask

    initial begin
        longint base = 64'h100000;              // 1 MB, clear of the ddr_fifo ring
        repeat (10) @(posedge clk); rst <= 1'b0; repeat (5) @(posedge clk);
        // 2. random writes / reads in a 16 KB window (forces overlapping, unaligned accesses)
        for (int n = 0; n < 300; n++) begin
            longint a = base + (rr() % 16384);
            int     len = 1 + (rr() % ((n % 5 == 0) ? 700 : 70));
            if (rr() % 100 < 55) do_write(a, len); else do_read(a, len);
        end
        // full-window readback
        do_read(base, 16384 + 700);
        // 3. memtest
        do_memtest(64'h200000, 65536, 32'hA5C3_0F1E, 2'b11, 0, 0);
        begin
            longint bad = 64'h200000 + 4 * 777;
            longint b = bad >> 5; bit [255:0] v = u_mig.mem[b];
            v[((bad & 31) / 4) * 32 +: 32] ^= 32'h0000_0100;
            u_mig.mem[b] = v;
            do_memtest(64'h200000, 65536, 32'hA5C3_0F1E, 2'b10, 1, bad);
        end
        // drain the FIFO traffic
        while (qpopped < NQ) @(posedge clk);
        repeat (50) @(posedge clk);
        $display("ddr_fifo: pushed %0d popped %0d left %0d; MIG writes %0d reads %0d model errors %0d",
                 qpushed, qpopped, qexp.size(), mig_nwr, mig_nrd, mig_err);
        if (qexp.size() != 0) errs++;
        errs += mig_err;
        if (errs == 0) $display("TB_PASS tb_mem");
        else           $display("TB_FAIL tb_mem errors=%0d", errs);
        $finish;
    end
    initial begin #20ms; $display("TB_FAIL tb_mem TIMEOUT qpopped=%0d", qpopped); $finish; end
endmodule
`default_nettype wire
