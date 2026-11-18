// tb_float_w — the clip format's adder and multiplier (float_addsub_w, float_mul_w: FP24's exponent, a 31-bit mantissa)
// against tools/float_ref.py's exact arithmetic (python3 tools/a_setup_ref.py --clipvec -> tools/gen/fpw_vectors.hex:
// 'op a b result', op 0 a + b, 1 a - b, 2 a * b — the special values against each other, random operands, cancelling
// differences, short mantissas, overflow and underflow), bit for bit. One operation per cycle with idle gaps at
// random: the units take a new pair every cycle and answer in order after their latency.
`default_nettype none
module tb_float_w;
    localparam int EXP_W=7, MAN_W=31, W=1+EXP_W+MAN_W, NMAX=20000;
    reg clk=0; always #5 clk=~clk; reg rst=1;
    reg as_in=0, as_op=0, mu_in=0; reg [W-1:0] xa=0, xb=0;
    wire as_v, mu_v; wire [W-1:0] as_r, mu_r;
    float_addsub_w #(EXP_W,MAN_W) U_AS(.clk,.rst,.in_valid(as_in),.op(as_op),.a(xa),.b(xb),.out_valid(as_v),.result(as_r));
    float_mul_w    #(EXP_W,MAN_W) U_MU(.clk,.rst,.in_valid(mu_in),.a(xa),.b(xb),.out_valid(mu_v),.result(mu_r));

    reg [1:0] vop [0:NMAX-1]; reg [W-1:0] va [0:NMAX-1], vb [0:NMAX-1], vr [0:NMAX-1];
    integer aq [$], mq [$];                       // the vector index of every operation in flight, per unit
    integer fd, code, n, i, errors, na, nm, la, lm, t_as, t_mu, k;
    integer tin [0:NMAX-1];
    always @(posedge clk) if (!rst) begin
        if (as_v) begin
            if (aq.size() == 0) begin errors=errors+1; $display(" add: a result without an operation"); end
            else begin
                k = aq.pop_front(); na = na + 1; la = $time/10 - tin[k];
                if (as_r !== vr[k]) begin errors=errors+1; if (errors<12) $display(" vec%0d op%0d %h %h: got %h exp %h",k,vop[k],va[k],vb[k],as_r,vr[k]); end
                if (t_as == 0) t_as = la; else if (la != t_as) begin errors=errors+1; if (errors<12) $display(" add: latency %0d, was %0d",la,t_as); end
            end
        end
        if (mu_v) begin
            if (mq.size() == 0) begin errors=errors+1; $display(" mul: a result without an operation"); end
            else begin
                k = mq.pop_front(); nm = nm + 1; lm = $time/10 - tin[k];
                if (mu_r !== vr[k]) begin errors=errors+1; if (errors<12) $display(" vec%0d mul %h %h: got %h exp %h",k,va[k],vb[k],mu_r,vr[k]); end
                if (t_mu == 0) t_mu = lm; else if (lm != t_mu) begin errors=errors+1; if (errors<12) $display(" mul: latency %0d, was %0d",lm,t_mu); end
            end
        end
    end
    initial begin
        errors=0; na=0; nm=0; t_as=0; t_mu=0;
        fd=$fopen("tools/gen/fpw_vectors.hex","r");
        if (fd == 0) begin $display("FAIL: no vector file"); $finish; end
        code=$fscanf(fd,"%d",n);
        if (n > NMAX) begin $display("FAIL: %0d vectors, room for %0d", n, NMAX); $finish; end
        for (i=0;i<n;i=i+1) code=$fscanf(fd,"%d %h %h %h",vop[i],va[i],vb[i],vr[i]);
        $fclose(fd);
        repeat(4) @(posedge clk); rst<=0; @(posedge clk);
        for (i=0;i<n;i=i+1) begin
            xa<=va[i]; xb<=vb[i]; as_op<=vop[i][0]; as_in<=(vop[i]!=2); mu_in<=(vop[i]==2);
            @(posedge clk);
            tin[i] = $time/10;
            if (vop[i]==2) mq.push_back(i); else aq.push_back(i);
            if ($urandom_range(0,7) == 0) begin as_in<=0; mu_in<=0; repeat($urandom_range(1,3)) @(posedge clk); end
        end
        as_in<=0; mu_in<=0;
        repeat(20) @(posedge clk);
        if (aq.size() != 0 || mq.size() != 0) begin errors=errors+1; $display(" %0d + %0d operations without a result", aq.size(), mq.size()); end
        $display("FLOAT_W: %0d vectors, %0d additions (latency %0d), %0d multiplications (latency %0d), errors = %0d", n, na, t_as, nm, t_mu, errors);
        $display("%s", (errors==0 && na+nm==n && n>0) ? "PASS":"FAIL"); $finish;
    end
endmodule
`default_nettype wire
