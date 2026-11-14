// tb_float_sfu — checks float_sfu RCP/RSQRT/LOG2/EXP2 against float_ref.py (SIN/COS: tb_sincos_sfu)
// golden (positive finite-normal outputs) within +/-2 ULP (raw-int distance).
`default_nettype none
module tb_float_sfu;
    reg clk=0; always #5 clk=~clk;
    reg rst=1;
    localparam int TOL = 2;

    // TB_SPLIT: both extra stages (SPLIT1B, SPLIT6 — the vertex core's setting, latency 12);
    // the inputs are held, so the fixed wait below covers every latency up to 13
`ifdef TB_SPLIT
    localparam bit SP = 1'b1;
`else
    localparam bit SP = 1'b0;
`endif
    reg [15:0] x16; reg [2:0] op16; reg v16; wire [15:0] r16; wire vv16;
    float_sfu #(.EXP_W(5),.MAN_W(10),.K(8),.KLOG(8),.SPLIT1B(SP),.SPLIT6(SP),
        .RCP_MEM("tools/gen/fp16_rcp_seed.mem"), .RSQRT_MEM("tools/gen/fp16_rsqrt_seed.mem"),
        .LOG2_MEM("tools/gen/fp16_log2_lut.mem"), .EXP2_MEM("tools/gen/fp16_exp2_lut.mem"))
      u16(.clk,.rst,.in_valid(v16),.op(op16),.x(x16),.out_valid(vv16),.result(r16));

    reg [23:0] x24; reg [2:0] op24; reg v24; wire [23:0] r24; wire vv24;
    float_sfu #(.EXP_W(7),.MAN_W(16),.K(12),.KLOG(11),.SPLIT1B(SP),.SPLIT6(SP),
        .RCP_MEM("tools/gen/fp24_rcp_seed.mem"), .RSQRT_MEM("tools/gen/fp24_rsqrt_seed.mem"),
        .LOG2_MEM("tools/gen/fp24_log2_lut.mem"), .EXP2_MEM("tools/gen/fp24_exp2_lut.mem"))
      u24(.clk,.rst,.in_valid(v24),.op(op24),.x(x24),.out_valid(vv24),.result(r24));

    integer fd, code, n, err, worst, total_err = 0, total_n = 0;
    reg [31:0] xb, eb; integer d;

    int tol = TOL;                       // per run: TOL (vs ideal maths), 0 for float_ref's bit-exact LOG2-near-1 vectors
    task run16(input [2:0] opc, input string fname);
        begin
            fd=$fopen(fname,"r"); n=0; err=0; worst=0;
            if (fd==0) begin $display("  MISSING %0s", fname); total_err = total_err + 1; disable run16; end   // a missing golden fails the bench
            while (!$feof(fd)) begin
                code=$fscanf(fd,"%h %h", xb, eb);
                if (code==2) begin
                    x16=xb[15:0]; op16=opc; v16=1;
                    repeat(13) @(posedge clk); #1;  // pipelined SFU latency (10 .. 12 by the SPLIT parameters)
                    d = (r16 >= eb[15:0]) ? (r16 - eb[15:0]) : (eb[15:0] - r16);
                    if (d>tol) begin err=err+1; if(err<5) $display("    x=%h got=%h exp=%h d=%0d",x16,r16,eb[15:0],d); end
                    if (d>worst) worst=d;
                    n=n+1;
                end
            end
            $fclose(fd);
            $display("  FP16 op%0d: %0d vec  err(>%0dulp)=%0d  worst=%0d ulp", opc,n,tol,err,worst);
            total_err = total_err + err + (n == 0); total_n = total_n + n;
        end
    endtask

    task run24(input [2:0] opc, input string fname);
        begin
            fd=$fopen(fname,"r"); n=0; err=0; worst=0;
            if (fd==0) begin $display("  MISSING %0s", fname); total_err = total_err + 1; disable run24; end
            while (!$feof(fd)) begin
                code=$fscanf(fd,"%h %h", xb, eb);
                if (code==2) begin
                    x24=xb[23:0]; op24=opc; v24=1;
                    repeat(13) @(posedge clk); #1;  // pipelined SFU latency (10 .. 12 by the SPLIT parameters)
                    d = (r24 >= eb[23:0]) ? (r24 - eb[23:0]) : (eb[23:0] - r24);
                    if (d>tol) begin err=err+1; if(err<5) $display("    x=%h got=%h exp=%h d=%0d",x24,r24,eb[23:0],d); end
                    if (d>worst) worst=d;
                    n=n+1;
                end
            end
            $fclose(fd);
            $display("  FP24 op%0d: %0d vec  err(>%0dulp)=%0d  worst=%0d ulp", opc,n,tol,err,worst);
            total_err = total_err + err + (n == 0); total_n = total_n + n;
        end
    endtask

    initial begin
        v16=0; v24=0; repeat(4) @(posedge clk); rst=0;
        $display("== FP16 ==");
        run16(0,"tools/gen/fp16_rcp_vectors.hex");
        run16(1,"tools/gen/fp16_rsqrt_vectors.hex");
        run16(2,"tools/gen/fp16_log2_vectors.hex");
        run16(3,"tools/gen/fp16_exp2_vectors.hex");
        tol = 0; run16(2,"tools/gen/fp16_log2near_vectors.hex"); tol = TOL;   // LOG2 near 1.0: bit-exact
        tol = 0; run16(3,"tools/gen/fp16_exp2spec_vectors.hex"); tol = TOL;   // EXP2 at its limits (infinities, |x| beyond the fixed-point range): exact
        $display("== FP24 ==");
        run24(0,"tools/gen/fp24_rcp_vectors.hex");
        run24(1,"tools/gen/fp24_rsqrt_vectors.hex");
        run24(2,"tools/gen/fp24_log2_vectors.hex");
        run24(3,"tools/gen/fp24_exp2_vectors.hex");
        tol = 0; run24(2,"tools/gen/fp24_log2near_vectors.hex"); tol = TOL;   // LOG2 near 1.0: bit-exact
        tol = 0; run24(3,"tools/gen/fp24_exp2spec_vectors.hex"); tol = TOL;   // EXP2 at its limits: exact
        // SIN / COS: tb_sincos_sfu (sincos_sfu, bit-exact)
        $display("TB DONE: %0d vectors, errors = %0d", total_n, total_err);
        if (total_err == 0) $display("TB_PASS tb_float_sfu"); else $display("TB_FAIL tb_float_sfu");
        $finish;
    end
endmodule
`default_nettype wire
