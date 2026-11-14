// tb_status_jtag — link_status_jtag LEAN 1 against the frame LEAN 0 shifts: {seq, fields, seq, A5} on TDO, read again and
// again while the fields change every user clock. Each read's fields must be exactly the snapshot its seq names (the
// bench records every snapshot); LEAN 1's frame is never torn (it holds the snapshot while shifting), LEAN 0's is torn
// by design when a snapshot lands during a slow read (the reader reads again).
// BSCANE2 is a stub here, its outputs driven by the bench.
`timescale 1ns/1ps
`default_nettype none
module BSCANE2 #(parameter int JTAG_CHAIN = 1) (
    output wire CAPTURE, DRCK, RESET, RUNTEST, SEL, SHIFT, TCK, TDI, TMS, UPDATE,
    input  wire TDO);
    assign CAPTURE = tb_status_jtag.capture; assign DRCK = tb_status_jtag.drck; assign SEL = tb_status_jtag.sel;
    assign SHIFT = tb_status_jtag.shift; assign TDI = 1'b0; assign RESET = 1'b0; assign RUNTEST = 1'b0;
    assign TCK = tb_status_jtag.drck; assign TMS = 1'b0; assign UPDATE = 1'b0;
endmodule

module tb_status_jtag;
    localparam int FW = 96, N = 8 + 8 + FW + 8, SNAP = 37;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    reg capture = 0, drck = 0, sel = 0, shift = 0;
    reg [FW-1:0] fields = 0;
    always @(posedge clk) fields <= {fields[FW-33:0], fields[FW-1:FW-32] + 32'h9E37_79B9};
    link_status_jtag #(.FW(FW), .SNAP_CYCLES(SNAP), .LEAN(1'b0)) u_old (.clk, .rst, .fields);
    link_status_jtag #(.FW(FW), .SNAP_CYCLES(SNAP), .LEAN(1'b1)) u_new (.clk, .rst, .fields);
    // every snapshot, by the seq it gets (a byte: the bench reads far fewer than 256 snapshots apart)
    reg [FW-1:0] rec_old [0:255], rec_new [0:255];
    always @(posedge clk) begin
        if (u_old.snap_en) rec_old[8'(u_old.seq + 8'd1)] <= fields;
        if (u_new.snap_en) rec_new[8'(u_new.seq + 8'd1)] <= fields;
    end

    reg [N-1:0] f_old, f_new;
    task automatic tck_pulse(input int half); begin #(half) drck = 1; #(half) drck = 0; end endtask
    // one DR scan: CAPTURE (one DRCK edge), then N-1 SHIFT edges; TDO sampled before each shift edge
    task automatic scan(input int half); begin
        sel = 1; capture = 1; tck_pulse(half); capture = 0; shift = 1;
        for (int i = 0; i < N; i++) begin
            f_new[i] = u_new.tdo; f_old[i] = u_old.tdo;
            if (i < N - 1) tck_pulse(half);
        end
        shift = 0; sel = 0;
    end endtask
    int errs = 0, torn_old = 0, torn_new = 0, torn_slow = 0, torn_slow_old = 0, ok_old = 0, reads = 0, held = 0;
    initial begin
        repeat (5) @(posedge clk); rst = 0;
        repeat (3 * SNAP) @(posedge clk);
        for (int r = 0; r < 300; r++) begin
            // slow scans (several snapshots due during one) and fast ones
            scan((r % 3 == 0) ? 41 : (r % 3 == 1) ? 3 : 1);
            reads++;
            if (f_new[7:0] !== 8'hA5 || f_old[7:0] !== 8'hA5) begin errs++; $display("TB_ERR read %0d: magic %h / %h", r, f_new[7:0], f_old[7:0]); end
            if (f_new[15:8] !== f_new[N-1:N-8]) begin torn_new++; if (r % 3 == 0) torn_slow++; end
            else if (f_new[FW+15:16] !== rec_new[f_new[15:8]]) begin
                errs++; $display("TB_ERR read %0d: LEAN fields are not snapshot %0d\n got  %h\n want %h", r, f_new[15:8], f_new[FW+15:16], rec_new[f_new[15:8]]);
            end
            if (f_old[15:8] !== f_old[N-1:N-8]) begin torn_old++; if (r % 3 == 0) torn_slow_old++; end
            else if (f_old[FW+15:16] !== rec_old[f_old[15:8]]) begin errs++; $display("TB_ERR read %0d: LEAN 0 fields are not snapshot %0d (bench)", r, f_old[15:8]); end
            else ok_old++;
            repeat (r % 7 + 1) @(posedge clk);
        end
        held = int'(8'(u_old.seq - u_new.seq));
        if (held == 0) begin errs++; $display("TB_ERR the LEAN snapshot was never held (the slow reads did not exercise it)"); end
        // a slow read is torn only when a snapshot lands as the hold starts (LEAN 0 captures all at once: hardly ever)
        if (torn_slow * 20 > reads / 3) begin errs++; $display("TB_ERR LEAN slow reads torn %0d times (LEAN 0 %0d)", torn_slow, torn_slow_old); end
        if (errs == 0) $display("TB_PASS tb_status_jtag: %0d reads, every whole frame its snapshot; torn LEAN %0d (slow %0d), LEAN 0 %0d (slow %0d), %0d LEAN 0 whole; LEAN held %0d snapshots back",
                                reads, torn_new, torn_slow, torn_old, torn_slow_old, ok_old, held);
        else $display("TB_FAIL tb_status_jtag: %0d errors", errs);
        $finish;
    end
endmodule
`default_nettype wire
