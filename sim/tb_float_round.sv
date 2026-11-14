// tb_float_round — float_round (TRUNC / FLR / CEIL / RNDE, float -> float) at FP24 and FP16 against
// tools/float_ref.py fround: tools/gen/fp{24,16}_round_vectors.hex (x, then the four expected results; fractions,
// halves, binade edges, |x| >= 2^M, +-0, denormals, +-Inf, NaN, random) streamed back to back (II = 1) with the
// mode cycling 0..3 per vector; every result must match bit for bit, in order, none lost.
`default_nettype none
module tb_float_round;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    reg v24 = 0; reg [1:0] m24 = 0; reg [23:0] x24 = 0; wire o24; wire [23:0] r24;
    float_round #(.EXP_W(7), .MAN_W(16)) u24(.clk, .rst, .in_valid(v24), .mode(m24), .x(x24), .out_valid(o24), .result(r24));
    reg v16 = 0; reg [1:0] m16 = 0; reg [15:0] x16 = 0; wire o16; wire [15:0] r16;
    float_round #(.EXP_W(5), .MAN_W(10)) u16(.clk, .rst, .in_valid(v16), .mode(m16), .x(x16), .out_valid(o16), .result(r16));

    reg [23:0] in24[$], ex24[$], got24[$]; reg [1:0] md24[$];
    reg [15:0] in16[$], ex16[$], got16[$]; reg [1:0] md16[$];
    always @(posedge clk) begin
        if (o24) got24.push_back(r24);
        if (o16) got16.push_back(r16);
    end
    task automatic load(input string f, input int fmt);
        int fd, c; reg [31:0] x, a, b, d, e;
        fd = $fopen(f, "r");
        if (fd == 0) begin $display("  MISSING %s", f); return; end
        while (!$feof(fd)) begin
            c = $fscanf(fd, "%h %h %h %h %h", x, a, b, d, e);
            if (c == 5) for (int m = 0; m < 4; m++) begin
                reg [31:0] y; y = (m == 0) ? a : (m == 1) ? b : (m == 2) ? d : e;
                if (fmt == 24) begin in24.push_back(x[23:0]); ex24.push_back(y[23:0]); md24.push_back(2'(m)); end
                else           begin in16.push_back(x[15:0]); ex16.push_back(y[15:0]); md16.push_back(2'(m)); end
            end
        end
        $fclose(fd);
    endtask
    int err, i;
    string MN[4] = '{"trunc", "floor", "ceil", "rne"};
    initial begin
        load("tools/gen/fp24_round_vectors.hex", 24); load("tools/gen/fp16_round_vectors.hex", 16);
        repeat (4) @(posedge clk); rst <= 0; @(posedge clk);
        for (i = 0; i < in24.size() || i < in16.size(); i++) begin
            @(posedge clk);
            v24 <= (i < in24.size()); if (i < in24.size()) begin x24 <= in24[i]; m24 <= md24[i]; end
            v16 <= (i < in16.size()); if (i < in16.size()) begin x16 <= in16[i]; m16 <= md16[i]; end
        end
        @(posedge clk); v24 <= 0; v16 <= 0;
        repeat (10) @(posedge clk);
        err = 0;
        if (in24.size() == 0 || got24.size() != in24.size()) begin err++; $display("  FP24: %0d results for %0d inputs", got24.size(), in24.size()); end
        if (in16.size() == 0 || got16.size() != in16.size()) begin err++; $display("  FP16: %0d results for %0d inputs", got16.size(), in16.size()); end
        for (i = 0; i < in24.size() && i < got24.size(); i++)
            if (got24[i] !== ex24[i]) begin err++; if (err < 10) $display("  FP24 %s(%h): got %h exp %h", MN[md24[i]], in24[i], got24[i], ex24[i]); end
        for (i = 0; i < in16.size() && i < got16.size(); i++)
            if (got16[i] !== ex16[i]) begin err++; if (err < 10) $display("  FP16 %s(%h): got %h exp %h", MN[md16[i]], in16[i], got16[i], ex16[i]); end
        $display("float_round: FP24 %0d + FP16 %0d results, errors = %0d", got24.size(), got16.size(), err);
        if (err == 0) $display("TB_PASS tb_float_round"); else $display("TB_FAIL tb_float_round");
        $finish;
    end
    initial begin #2ms; $display("TB_FAIL tb_float_round: TIMEOUT"); $finish; end
endmodule
`default_nettype wire
