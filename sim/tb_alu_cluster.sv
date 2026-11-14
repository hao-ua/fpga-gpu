// tb_alu_cluster — validates the assembled VPU (FP24, 4 lanes) and FPU
// (FP16, 16 lanes) ALU blocks against tools/float_ref.py golden.
//   madd/cmp/dot4: bit-exact (compositions of verified units)
//   sfu (rcp): +/-2 ULP
`default_nettype none
module tb_alu_cluster;
    reg clk=0; always #5 clk=~clk;
    reg rst=1;
    localparam int TOL=2;

    // ---------- FP24 VPU: N_VEC4=1, 4 lanes, W=24 ----------
    localparam int W24=24, NL24=4;
    reg  [NL24*W24-1:0] a24,b24,c24;
    reg                 mv24,cv24,dv24,sv24,sub24,dp324; reg[2:0] cop24; reg[2:0] sop24;
    wire                vm24,vc24,vd24,vs24,sr24;
    wire [NL24*W24-1:0] ym24,yc24,ys24; wire [W24-1:0] yd24;
    alu_cluster #(.EXP_W(7),.MAN_W(16),.N_VEC4(1),.K(12),.KLOG(11),
        .RCP_MEM("tools/gen/fp24_rcp_seed.mem"),.RSQRT_MEM("tools/gen/fp24_rsqrt_seed.mem"),
        .LOG2_MEM("tools/gen/fp24_log2_lut.mem"),.EXP2_MEM("tools/gen/fp24_exp2_lut.mem"))
      U24(.clk,.rst,.a(a24),.b(b24),.c(c24),
          .madd_valid(mv24),.madd_sub(sub24),.v_madd(vm24),.y_madd(ym24),
          .cmp_valid(cv24),.cmp_op(cop24),.v_cmp(vc24),.y_cmp(yc24),
          .dot_valid(dv24),.dp3(dp324),.v_dot(vd24),.y_dot(yd24),
          .sfu_valid(sv24),.sfu_op(sop24),.sfu_ready(sr24),.v_sfu(vs24),.y_sfu(ys24));

    // ---------- FP16 FPU: N_VEC4=4, 16 lanes, W=16 ----------
    localparam int W16=16, NL16=16, NG16=4;
    reg  [NL16*W16-1:0] a16,b16,c16;
    reg                 mv16,cv16,dv16,sv16,sub16,dp316; reg[2:0] cop16; reg[2:0] sop16;
    wire                vm16,vc16,vd16,vs16,sr16;
    wire [NL16*W16-1:0] ym16,yc16,ys16; wire [NG16*W16-1:0] yd16;
    alu_cluster #(.EXP_W(5),.MAN_W(10),.N_VEC4(4),.K(8),.KLOG(8),
        .RCP_MEM("tools/gen/fp16_rcp_seed.mem"),.RSQRT_MEM("tools/gen/fp16_rsqrt_seed.mem"),
        .LOG2_MEM("tools/gen/fp16_log2_lut.mem"),.EXP2_MEM("tools/gen/fp16_exp2_lut.mem"))
      U16(.clk,.rst,.a(a16),.b(b16),.c(c16),
          .madd_valid(mv16),.madd_sub(sub16),.v_madd(vm16),.y_madd(ym16),
          .cmp_valid(cv16),.cmp_op(cop16),.v_cmp(vc16),.y_cmp(yc16),
          .dot_valid(dv16),.dp3(dp316),.v_dot(vd16),.y_dot(yd16),
          .sfu_valid(sv16),.sfu_op(sop16),.sfu_ready(sr16),.v_sfu(vs16),.y_sfu(ys16));

    integer fd,code,n,err,i,d;
    reg [31:0] ta,tb,tc,te, x0,x1,x2,x3,b0,b1,b2,b3;

    initial begin
        {mv24,cv24,dv24,sv24,mv16,cv16,dv16,sv16}=0;
        repeat(4) @(posedge clk); rst=0;

        // ===================== FP24 =====================
        $display("== FP24 VPU (4 lanes) ==");
        // ---- MADD (broadcast scalar to all 4 lanes, bit-exact) ----
        fd=$fopen("tools/gen/fp24_madd_vectors.hex","r"); n=0; err=0;
        while(!$feof(fd)) begin
            code=$fscanf(fd,"%h %h %h %h",ta,tb,tc,te);
            if(code==4) begin
                a24={NL24{ta[W24-1:0]}}; b24={NL24{tb[W24-1:0]}}; c24={NL24{tc[W24-1:0]}};
                sub24=0; mv24=1; @(posedge clk); mv24=0;
                repeat(8) @(posedge clk); #1;
                for(i=0;i<NL24;i++) if(ym24[i*W24+:W24]!==te[W24-1:0]) err++;
                n++;
            end
        end $fclose(fd); $display("  MADD: %0d vec x4 lanes  err=%0d",n,err);
        // ---- DOT4 (distinct lanes, bit-exact) ----
        fd=$fopen("tools/gen/fp24_dot4_vectors.hex","r"); n=0; err=0;
        while(!$feof(fd)) begin
            code=$fscanf(fd,"%h %h %h %h %h %h %h %h %h",x0,x1,x2,x3,b0,b1,b2,b3,te);
            if(code==9) begin
                a24={x3[W24-1:0],x2[W24-1:0],x1[W24-1:0],x0[W24-1:0]};
                b24={b3[W24-1:0],b2[W24-1:0],b1[W24-1:0],b0[W24-1:0]};
                dp324=0; dv24=1; @(posedge clk); dv24=0;
                repeat(13) @(posedge clk); #1;
                if(yd24!==te[W24-1:0]) err++;
                n++;
            end
        end $fclose(fd); $display("  DOT4: %0d vec  err=%0d",n,err);
        // ---- CMP MIN/MAX/SLT (broadcast, bit-exact) ----
        fd=$fopen("tools/gen/fp24_cmp_vectors.hex","r"); n=0; err=0;
        while(!$feof(fd)) begin
            code=$fscanf(fd,"%h %h %h %h %h",ta,tb,x0,x1,x2);  // a b min max slt
            if(code==5) begin
                a24={NL24{ta[W24-1:0]}}; b24={NL24{tb[W24-1:0]}};
                cop24=3'd0; cv24=1; @(posedge clk); cv24=0; @(posedge clk); #1; // MIN, lat1
                if(yc24[0+:W24]!==x0[W24-1:0]) err++;
                cop24=3'd1; cv24=1; @(posedge clk); cv24=0; @(posedge clk); #1; // MAX
                if(yc24[0+:W24]!==x1[W24-1:0]) err++;
                cop24=3'd2; cv24=1; @(posedge clk); cv24=0; @(posedge clk); #1; // SLT
                if(yc24[0+:W24]!==x2[W24-1:0]) err++;
                n++;
            end
        end $fclose(fd); $display("  CMP : %0d vec x3 ops  err=%0d",n,err);
        // ---- SFU RCP (broadcast, +/-2 ULP) ----
        fd=$fopen("tools/gen/fp24_rcp_vectors.hex","r"); n=0; err=0;
        while(!$feof(fd)) begin
            code=$fscanf(fd,"%h %h",ta,te);
            if(code==2) begin
                a24={NL24{ta[W24-1:0]}}; sop24=2'd0; sv24=1; @(posedge clk); sv24=0;
                repeat(20) @(posedge clk); #1;      // deterministic vec4_sfu latency (pipelined SFU)
                for(i=0;i<NL24;i++) begin
                    d = (ys24[i*W24+:W24]>=te[W24-1:0]) ? ys24[i*W24+:W24]-te[W24-1:0] : te[W24-1:0]-ys24[i*W24+:W24];
                    if(d>TOL) err++;
                end
                n++;
            end
        end $fclose(fd); $display("  RCP : %0d vec x4 lanes  err=%0d",n,err);

        // ===================== FP16 =====================
        $display("== FP16 FPU (16 lanes) ==");
        fd=$fopen("tools/gen/fp16_madd_vectors.hex","r"); n=0; err=0;
        while(!$feof(fd)) begin
            code=$fscanf(fd,"%h %h %h %h",ta,tb,tc,te);
            if(code==4) begin
                a16={NL16{ta[W16-1:0]}}; b16={NL16{tb[W16-1:0]}}; c16={NL16{tc[W16-1:0]}};
                sub16=0; mv16=1; @(posedge clk); mv16=0;
                repeat(8) @(posedge clk); #1;
                for(i=0;i<NL16;i++) if(ym16[i*W16+:W16]!==te[W16-1:0]) err++;
                n++;
            end
        end $fclose(fd); $display("  MADD: %0d vec x16 lanes err=%0d",n,err);
        fd=$fopen("tools/gen/fp16_dot4_vectors.hex","r"); n=0; err=0;
        while(!$feof(fd)) begin       // 4 groups -> read 4 lines/test
            code=$fscanf(fd,"%h %h %h %h %h %h %h %h %h",x0,x1,x2,x3,b0,b1,b2,b3,te);
            if(code==9) begin
                // broadcast same vec4 to all 4 groups; check all 4 group outputs
                a16=0; b16=0;
                for(i=0;i<NG16;i++) begin
                    a16[i*4*W16 +: 4*W16] = {x3[W16-1:0],x2[W16-1:0],x1[W16-1:0],x0[W16-1:0]};
                    b16[i*4*W16 +: 4*W16] = {b3[W16-1:0],b2[W16-1:0],b1[W16-1:0],b0[W16-1:0]};
                end
                dp316=0; dv16=1; @(posedge clk); dv16=0;
                repeat(13) @(posedge clk); #1;
                for(i=0;i<NG16;i++) if(yd16[i*W16+:W16]!==te[W16-1:0]) err++;
                n++;
            end
        end $fclose(fd); $display("  DOT4: %0d vec x4 grp   err=%0d",n,err);
        fd=$fopen("tools/gen/fp16_cmp_vectors.hex","r"); n=0; err=0;
        while(!$feof(fd)) begin
            code=$fscanf(fd,"%h %h %h %h %h",ta,tb,x0,x1,x2);
            if(code==5) begin
                a16={NL16{ta[W16-1:0]}}; b16={NL16{tb[W16-1:0]}};
                cop16=3'd0; cv16=1; @(posedge clk); cv16=0; @(posedge clk); #1;
                if(yc16[0+:W16]!==x0[W16-1:0]) err++;
                cop16=3'd1; cv16=1; @(posedge clk); cv16=0; @(posedge clk); #1;
                if(yc16[0+:W16]!==x1[W16-1:0]) err++;
                cop16=3'd2; cv16=1; @(posedge clk); cv16=0; @(posedge clk); #1;
                if(yc16[0+:W16]!==x2[W16-1:0]) err++;
                n++;
            end
        end $fclose(fd); $display("  CMP : %0d vec x3 ops  err=%0d",n,err);
        fd=$fopen("tools/gen/fp16_rcp_vectors.hex","r"); n=0; err=0;
        while(!$feof(fd)) begin
            code=$fscanf(fd,"%h %h",ta,te);
            if(code==2) begin
                a16={NL16{ta[W16-1:0]}}; sop16=2'd0; sv16=1; @(posedge clk); sv16=0;
                repeat(20) @(posedge clk); #1;      // deterministic vec4_sfu latency (pipelined SFU)
                for(i=0;i<NL16;i++) begin
                    d = (ys16[i*W16+:W16]>=te[W16-1:0]) ? ys16[i*W16+:W16]-te[W16-1:0] : te[W16-1:0]-ys16[i*W16+:W16];
                    if(d>TOL) err++;
                end
                n++;
            end
        end $fclose(fd); $display("  RCP : %0d vec x16 lanes err=%0d",n,err);

        $display("TB DONE"); $finish;
    end
endmodule
`default_nettype wire
