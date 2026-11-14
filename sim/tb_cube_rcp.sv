`timescale 1ns/1ps
// tb_cube_rcp — exhaustive check of cube_rcp (tmu24's cube-map reciprocal): every A = {1, m} for the
// 2^16 mantissas, one per cycle, R must equal floor(2^42 / A) exactly (64-bit reference); also prints the pipeline latency.
//   powershell -File tools/sim_gpu.ps1 -Test tb_cube_rcp
`default_nettype none
module tb_cube_rcp;
    reg clk = 0; always #5 clk = ~clk;
    reg [16:0] a = 17'h10000;
    wire [26:0] r;
    cube_rcp #(.M(16)) dut(.clk, .a, .r);
    localparam int LAT = 13;                       // registers from a to r: s0 (a0) .. s12 (r12) = 13 (tmu24's DQ = 12 counts from dv_v[0] = a0)
    reg [16:0] aq [$]; integer n_in = 0, n_out = 0, err = 0;
    longint ref_r;
    // +DBGA_<n>: trace the pipeline registers for 14 cycles once mantissa word n enters s0
    integer dbga = -1, dbgn = 0;
    initial void'($value$plusargs("DBGA_%d", dbga));
    always @(posedge clk) begin
        if (dbga >= 0 && dut.a0 == 17'(dbga) && dbgn == 0) dbgn = 14;
        if (dbgn > 0) begin
            $display("  t%0d a0=%0d y0=%0d p0=%0d d0=%0d m0=%0d y1=%0d p1=%0d d1=%0d m1=%0d c=%0d pcl=%0d pch=%0d lt0=%b lt1=%b gea=%b r=%0d",
                     14 - dbgn, dut.a0, dut.y0, dut.p0, dut.d0, dut.m0, dut.y1, dut.p1, dut.d1, dut.m1, dut.c, dut.pcl, dut.pch, dut.lt0, dut.lt1, dut.gea, dut.r12);
            dbgn--;
        end
    end
    initial begin
        // 2 idle cycles, then all mantissas back to back, then drain
        repeat (2) @(posedge clk);
        for (int m = 0; m < 65536; m++) begin
            a <= 17'h10000 | 17'(m);
            @(posedge clk);
            aq.push_back(17'h10000 | 17'(m)); n_in++;
            // the result of the value entered LAT cycles ago
            if (n_in > LAT) begin : chk
                reg [16:0] av; av = aq.pop_front();
                ref_r = 64'd4398046511104 / longint'(av);          // 2^42 / A
                if (longint'(r) != ref_r) begin
                    err++;
                    if (err <= 10) $display("  A=%0d (0x%05h): got %0d expected %0d (diff %0d)", av, av, r, ref_r, longint'(r) - ref_r);
                end
                n_out++;
            end
        end
        for (int k = 0; k < LAT; k++) begin
            @(posedge clk);
            begin : drain
                reg [16:0] av; av = aq.pop_front();
                ref_r = 64'd4398046511104 / longint'(av);
                if (longint'(r) != ref_r) begin
                    err++;
                    if (err <= 10) $display("  A=%0d (0x%05h): got %0d expected %0d (diff %0d)", av, av, r, ref_r, longint'(r) - ref_r);
                end
                n_out++;
            end
        end
        $display("cube_rcp: %0d mantissas checked, errors = %0d (latency %0d)", n_out, err, LAT);
        if (err == 0 && n_out == 65536) $display("TB_PASS tb_cube_rcp"); else $display("TB_FAIL tb_cube_rcp");
        $finish;
    end
endmodule
`default_nettype wire
