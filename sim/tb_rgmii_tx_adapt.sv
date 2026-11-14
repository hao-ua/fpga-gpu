`timescale 1ns/1ps
// tb_rgmii_tx_adapt — drive a byte stream through the serializer (FWFT model) for
// 1000 and 100 modes; reconstruct bytes from the rising-edge nibble stream and
// check they match, and that adv pops at the right cadence.
module tb_rgmii_tx_adapt;
    reg clk=0; always #5 clk=~clk;
    reg rst=1; reg [1:0] speed;
    reg [7:0] B [0:5];
    initial begin B[0]=8'h55;B[1]=8'hD5;B[2]=8'hAB;B[3]=8'h12;B[4]=8'h34;B[5]=8'h7F; end
    localparam N=6;

    // FWFT source model
    integer ptr=0; reg run=0;
    wire valid = run & (ptr<N);
    wire [7:0] data = (ptr<N) ? B[ptr] : 8'h0;
    wire adv; wire [3:0] txd_d1,txd_d2; wire ctl_d1,ctl_d2;
    always @(posedge clk) if (valid & adv) ptr<=ptr+1;

    rgmii_tx_adapt DUT(.clk,.rst,.speed,.valid,.data,.adv,.txd_d1,.txd_d2,.ctl_d1,.ctl_d2);

    // reconstruct: collect rising-edge nibbles while ctl_d1; giga -> {d2,d1}, mii -> pair d1
    integer cap=0, errs=0; reg [7:0] got[0:31];
    reg half; reg [3:0] lo;
    task reset_cap; begin cap=0; half=0; end endtask
    always @(posedge clk) if (run & ctl_d1) begin
        if (speed==2'b10) begin got[cap]={txd_d2,txd_d1}; cap=cap+1; end
        else begin
            if (!half) begin lo<=txd_d1; half<=1; end
            else begin got[cap]={txd_d1,lo}; cap=cap+1; half<=0; end
        end
    end

    integer k;
    task check(input [127:0] tag); begin
        if (cap!==N) begin $display("FAIL %0s cap=%0d exp %0d",tag,cap,N); errs=errs+1; end
        for (k=0;k<N&&k<cap;k=k+1)
            if (got[k]!==B[k]) begin $display("FAIL %0s b%0d got %h exp %h",tag,k,got[k],B[k]); errs=errs+1; end
    end endtask

    initial begin
        repeat(3) @(negedge clk); rst<=0;
        // 1000
        speed<=2'b10; @(negedge clk); reset_cap; ptr=0; run<=1;
        wait(ptr==N); @(negedge clk); run<=0; repeat(2) @(posedge clk);
        check("giga");
        // 100
        @(negedge clk); speed<=2'b01; @(negedge clk); reset_cap; ptr=0; run<=1;
        wait(ptr==N); @(negedge clk); run<=0; repeat(2) @(posedge clk);
        check("mii100");

        if (errs==0) $display("tb_rgmii_tx_adapt: PASS");
        else         $display("tb_rgmii_tx_adapt: FAIL (%0d errors)",errs);
        $finish;
    end
    initial begin #20000; $display("tb_rgmii_tx_adapt: TIMEOUT"); $finish; end
endmodule
