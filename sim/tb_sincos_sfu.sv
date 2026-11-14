// tb_sincos_sfu — sincos_sfu (interpolated quarter-wave table) at FP24 and FP16 against tools/float_ref.py:
//   1. golden vectors tools/gen/fp{24,16}_{sin,cos}_vectors.hex (float_ref.sfu_sin; specials, small-angle edge,
//      large angles, random) streamed back to back (II = 1, both ops interleaved) and again with random gaps:
//      every result must match BIT FOR BIT, in order, with no lost or extra outputs;
//   2. specials not in the files: +-Inf / NaN -> QNaN, -0 and denormals -> sin 0 / cos 1;
//   3. accuracy against the simulator's own $sin / $cos (independent of float_ref): max absolute error over the
//      FP24 vectors, reported, must stay below 2^-17 (FP24 half-ULP at 1.0 is 2^-17).
`default_nettype none
module tb_sincos_sfu;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;

    reg        v24 = 0, op24 = 0; reg [23:0] x24 = 0; wire o24; wire [23:0] r24;
    sincos_sfu #(.EXP_W(7), .MAN_W(16), .KSIN(10), .SIN_MEM("tools/gen/fp24_sin_lut.mem"))
        u24(.clk, .rst, .in_valid(v24), .op(op24), .x(x24), .out_valid(o24), .result(r24));
    reg        v16 = 0, op16 = 0; reg [15:0] x16 = 0; wire o16; wire [15:0] r16;
    sincos_sfu #(.EXP_W(5), .MAN_W(10), .KSIN(10), .SIN_MEM("tools/gen/fp16_sin_lut.mem"))
        u16(.clk, .rst, .in_valid(v16), .op(op16), .x(x16), .out_valid(o16), .result(r16));

    // per format: inputs, ops, expected, results in arrival order
    reg [23:0] in24[$], ex24[$], got24[$]; reg op24q[$];
    reg [15:0] in16[$], ex16[$], got16[$]; reg op16q[$];
    always @(posedge clk) begin
        if (o24) got24.push_back(r24);
        if (o16) got16.push_back(r16);
    end

    function automatic real fp24_real(input [23:0] b);
        int e; real m;
        e = int'(b[22:16]);
        if (e == 0) return 0.0;
        m = 1.0 + real'(b[15:0]) / 65536.0;
        fp24_real = (b[23] ? -m : m) * (2.0 ** (e - 63));
    endfunction

    task automatic load(input string f, input bit cos, input int fmt);
        int fd, c; reg [31:0] a, b;
        fd = $fopen(f, "r");
        if (fd == 0) begin $display("  MISSING %s", f); return; end
        while (!$feof(fd)) begin
            c = $fscanf(fd, "%h %h", a, b);
            if (c == 2) begin
                if (fmt == 24) begin in24.push_back(a[23:0]); ex24.push_back(b[23:0]); op24q.push_back(cos); end
                else           begin in16.push_back(a[15:0]); ex16.push_back(b[15:0]); op16q.push_back(cos); end
            end
        end
        $fclose(fd);
    endtask
    // specials (expected per op)
    task automatic special24(input [23:0] x, input [23:0] es, input [23:0] ec);
        in24.push_back(x); ex24.push_back(es); op24q.push_back(1'b0);
        in24.push_back(x); ex24.push_back(ec); op24q.push_back(1'b1);
    endtask
    task automatic special16(input [15:0] x, input [15:0] es, input [15:0] ec);
        in16.push_back(x); ex16.push_back(es); op16q.push_back(1'b0);
        in16.push_back(x); ex16.push_back(ec); op16q.push_back(1'b1);
    endtask

    localparam [23:0] Q24 = 24'h7f8000, ONE24 = {1'b0, 7'd63, 16'd0};
    localparam [15:0] Q16 = 16'h7e00,   ONE16 = 16'h3c00;
    int err, n24, n16, i, pass;
    real maxabs, t, g;
    initial begin
        // interleave the sin and cos files: alternate entries so both ops are in flight together
        begin
            reg [23:0] si[$], se[$], ci[$], ce[$]; reg so[$], co[$];
            load("tools/gen/fp24_sin_vectors.hex", 0, 24); si = in24; se = ex24; in24 = {}; ex24 = {}; op24q = {};
            load("tools/gen/fp24_cos_vectors.hex", 1, 24); ci = in24; ce = ex24; in24 = {}; ex24 = {}; op24q = {};
            for (i = 0; i < si.size() || i < ci.size(); i++) begin
                if (i < si.size()) begin in24.push_back(si[i]); ex24.push_back(se[i]); op24q.push_back(1'b0); end
                if (i < ci.size()) begin in24.push_back(ci[i]); ex24.push_back(ce[i]); op24q.push_back(1'b1); end
            end
        end
        load("tools/gen/fp16_sin_vectors.hex", 0, 16); load("tools/gen/fp16_cos_vectors.hex", 1, 16);
        special24(24'h7f0000, Q24, Q24); special24(24'hff0000, Q24, Q24); special24(24'h7f0123, Q24, Q24);
        special24(24'h800000, 24'h0, ONE24); special24(24'h000123, 24'h0, ONE24);   // -0, denormal
        special16(16'h7c00, Q16, Q16); special16(16'hfc00, Q16, Q16); special16(16'h7c11, Q16, Q16);
        special16(16'h8000, 16'h0, ONE16); special16(16'h0123, 16'h0, ONE16);
        n24 = in24.size(); n16 = in16.size();
        repeat (4) @(posedge clk); rst <= 0; @(posedge clk);
        err = 0;
        for (pass = 0; pass < 2; pass++) begin                      // 0: back to back, 1: random gaps
            got24 = {}; got16 = {};
            for (i = 0; i < n24 || i < n16; ) begin
                @(posedge clk);
                if (pass == 1 && $urandom_range(0, 2) == 0) begin v24 <= 0; v16 <= 0; continue; end
                v24 <= (i < n24); if (i < n24) begin x24 <= in24[i]; op24 <= op24q[i]; end
                v16 <= (i < n16); if (i < n16) begin x16 <= in16[i]; op16 <= op16q[i]; end
                i++;
            end
            @(posedge clk); v24 <= 0; v16 <= 0;
            repeat (30) @(posedge clk);
            if (got24.size() != n24) begin err++; $display("  pass %0d FP24: %0d results for %0d inputs", pass, got24.size(), n24); end
            if (got16.size() != n16) begin err++; $display("  pass %0d FP16: %0d results for %0d inputs", pass, got16.size(), n16); end
            for (i = 0; i < n24 && i < got24.size(); i++)
                if (got24[i] !== ex24[i]) begin
                    err++; if (err < 10) $display("  FP24 %s(%h): got %h exp %h", op24q[i] ? "cos" : "sin", in24[i], got24[i], ex24[i]);
                end
            for (i = 0; i < n16 && i < got16.size(); i++)
                if (got16[i] !== ex16[i]) begin
                    err++; if (err < 10) $display("  FP16 %s(%h): got %h exp %h", op16q[i] ? "cos" : "sin", in16[i], got16[i], ex16[i]);
                end
        end
        // accuracy against $sin / $cos (finite inputs only)
        maxabs = 0.0;
        for (i = 0; i < n24 && i < got24.size(); i++) begin
            if (in24[i][22:16] == 7'h7f || in24[i][22:16] == 7'h00) continue;
            t = op24q[i] ? $cos(fp24_real(in24[i])) : $sin(fp24_real(in24[i]));
            g = fp24_real(got24[i]);
            if ((g > t ? g - t : t - g) > maxabs) maxabs = (g > t ? g - t : t - g);
        end
        if (maxabs >= 2.0 ** -17) begin err++; $display("  FP24 accuracy: max abs %e >= 2^-17", maxabs); end
        $display("sincos_sfu: FP24 %0d + FP16 %0d vectors x 2 passes, FP24 max abs error vs $sin/$cos %e (2^%0.1f), errors = %0d",
                 n24, n16, maxabs, $ln(maxabs) / $ln(2.0), err);
        if (err == 0) $display("TB_PASS tb_sincos_sfu"); else $display("TB_FAIL tb_sincos_sfu");
        $finish;
    end
    initial begin #5ms; $display("TB_FAIL tb_sincos_sfu: TIMEOUT"); $finish; end
endmodule
`default_nettype wire
