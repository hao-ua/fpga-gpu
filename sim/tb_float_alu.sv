// tb_float_alu — checks float_mul / float_addsub against tools/float_ref.py
// golden vectors (a b prod sum), for FP16 and FP24. Run with xsim.
`default_nettype none
module tb_float_alu;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;

    // ---- FP16 DUTs ----
    reg  [15:0] a16, b16; reg v16;
    wire [15:0] mul16, add16; wire mv16, av16;
    float_mul    #(5,10) um16(.clk,.rst,.in_valid(v16),.a(a16),.b(b16),.in_ready(),.out_valid(mv16),.result(mul16));
    float_addsub #(5,10) ua16(.clk,.rst,.in_valid(v16),.op(1'b0),.a(a16),.b(b16),.in_ready(),.out_valid(av16),.result(add16));

    // ---- FP24 DUTs ----
    reg  [23:0] a24, b24; reg v24;
    wire [23:0] mul24, add24; wire mv24, av24;
    float_mul    #(7,16) um24(.clk,.rst,.in_valid(v24),.a(a24),.b(b24),.in_ready(),.out_valid(mv24),.result(mul24));
    float_addsub #(7,16) ua24(.clk,.rst,.in_valid(v24),.op(1'b0),.a(a24),.b(b24),.in_ready(),.out_valid(av24),.result(add24));

    integer fd, code, n, em, ea;
    reg [31:0] ta, tb, tp, ts;

    initial begin
        a16=0;b16=0;v16=0; a24=0;b24=0;v24=0;
        repeat(4) @(posedge clk); rst=0;

        // ===== FP16 =====
        fd = $fopen("tools/gen/fp16_muladd_vectors.hex","r");
        if (fd==0) begin $display("FATAL: cannot open fp16 vectors"); $finish; end
        n=0; em=0; ea=0;
        while (!$feof(fd)) begin
            code = $fscanf(fd, "%h %h %h %h", ta, tb, tp, ts);
            if (code==4) begin
                a16=ta[15:0]; b16=tb[15:0]; v16=1;
                @(posedge clk); v16=0;
                @(posedge clk); @(posedge clk); @(posedge clk); @(posedge clk); #1;   // 3-cycle latency
                if (mul16 !== tp[15:0]) begin em=em+1; if(em<6) $display("FP16 MUL %h*%h => %h exp %h", a16,b16,mul16,tp[15:0]); end
                if (add16 !== ts[15:0]) begin ea=ea+1; if(ea<6) $display("FP16 ADD %h+%h => %h exp %h", a16,b16,add16,ts[15:0]); end
                n=n+1;
            end
        end
        $fclose(fd);
        $display("FP16: %0d vectors  MUL err=%0d  ADD err=%0d", n, em, ea);

        // ===== FP24 =====
        fd = $fopen("tools/gen/fp24_muladd_vectors.hex","r");
        if (fd==0) begin $display("FATAL: cannot open fp24 vectors"); $finish; end
        n=0; em=0; ea=0;
        while (!$feof(fd)) begin
            code = $fscanf(fd, "%h %h %h %h", ta, tb, tp, ts);
            if (code==4) begin
                a24=ta[23:0]; b24=tb[23:0]; v24=1;
                @(posedge clk); v24=0;
                @(posedge clk); @(posedge clk); @(posedge clk); @(posedge clk); #1;
                if (mul24 !== tp[23:0]) begin em=em+1; if(em<6) $display("FP24 MUL %h*%h => %h exp %h", a24,b24,mul24,tp[23:0]); end
                if (add24 !== ts[23:0]) begin ea=ea+1; if(ea<6) $display("FP24 ADD %h+%h => %h exp %h", a24,b24,add24,ts[23:0]); end
                n=n+1;
            end
        end
        $fclose(fd);
        $display("FP24: %0d vectors  MUL err=%0d  ADD err=%0d", n, em, ea);
        $display("TB DONE");
        $finish;
    end
endmodule
`default_nettype wire
