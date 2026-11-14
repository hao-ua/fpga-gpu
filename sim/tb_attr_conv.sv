`timescale 1ns/1ps
// tb_attr_conv — the vertex-attribute component converter (attr_conv: BYTE / UNSIGNED_BYTE / SHORT / UNSIGNED_SHORT /
// FIXED / FLOAT / FP24, normalized or not -> FP24) against tools/attr_conv_ref.py's vectors (tools/gen/attr_conv_vectors.hex,
// the model a_vtx_ref.conv): every vector is driven at II = 1 with random gaps and the result compared bit for bit.
`default_nettype none
module tb_attr_conv;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    reg in_valid = 0; reg [31:0] raw = 0; reg [2:0] ty = 0; reg nm = 0;
    wire out_valid; wire [23:0] result;
    attr_conv dut (.clk, .rst, .in_valid, .raw, .ty, .nm, .out_valid, .result);
    int fd, n = 0, nout = 0, err = 0; reg [2:0] vty [$]; reg vnm [$]; reg [31:0] vraw [$]; reg [23:0] vexp [$];
    reg [31:0] a, b, c, d;
    initial begin
        fd = $fopen("tools/gen/attr_conv_vectors.hex", "r");
        if (fd == 0) begin $display("TB_FAIL tb_attr_conv: no vectors (python3 tools/attr_conv_ref.py)"); $finish; end
        while ($fscanf(fd, "%h %h %h %h", a, b, c, d) == 4) begin vty.push_back(a[2:0]); vnm.push_back(b[0]); vraw.push_back(c); vexp.push_back(d[23:0]); n++; end
        $fclose(fd);
        repeat (3) @(posedge clk); rst <= 0; @(posedge clk);
        for (int k = 0; k < n; k++) begin
            while ($urandom % 4 == 0) begin @(posedge clk); in_valid <= 0; end
            @(posedge clk); in_valid <= 1; raw <= vraw[k]; ty <= vty[k]; nm <= vnm[k];
        end
        @(posedge clk); in_valid <= 0;
        repeat (10) @(posedge clk);
        if (nout != n) begin err++; $display("  %0d results, %0d vectors", nout, n); end
        $display("attr_conv: %0d vectors, errors = %0d", n, err);
        if (err == 0) $display("TB_PASS tb_attr_conv"); else $display("TB_FAIL tb_attr_conv");
        $finish;
    end
    always @(posedge clk) if (out_valid && !rst) begin
        if (nout < n && result !== vexp[nout]) begin
            err++;
            if (err < 12) $display("  vector %0d: ty %0d nm %0d raw %08x -> %06x, expected %06x", nout, vty[nout], vnm[nout], vraw[nout], result, vexp[nout]);
        end
        nout++;
    end
    initial begin #2ms; $display("TB_FAIL tb_attr_conv: TIMEOUT"); $finish; end
endmodule
`default_nettype wire
