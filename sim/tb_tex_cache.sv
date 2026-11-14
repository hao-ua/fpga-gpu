`timescale 1ns/1ps
// tb_tex_cache — 4-way texture burst cache: miss->fill->hit (hit must not re-fetch), 4-way fill,
// eviction on the 5th tag in a set, and correct data return.
module tb_tex_cache;
    reg clk=0; always #5 clk=~clk;
    reg rst=1;
    reg         lu_valid=0; reg [25:0] lu_burst; wire lu_done; wire [255:0] lu_data;
    wire        mig_en; wire [28:0] mig_addr; reg mig_rd_valid=0; reg [255:0] mig_rd_data;
    wire [31:0] n_hit, n_miss;
    // PF 0: the fetch counts below are the demand misses alone (the prefetch and merge paths: tb_tex_cache_pf)
    tex_cache #(.PF(1'b0)) DUT(.clk,.rst,.lu_valid,.lu_burst,.lu_done,.lu_data,
        .mig_en,.mig_addr,.mig_rdy(1'b1),.mig_rd_valid,.mig_rd_data,.n_hit,.n_miss);

    // DDR model: 3-cycle latency; data = 8 copies of {4'hC, addr[27:0]}; counts fetches.
    function automatic [255:0] model(input [28:0] a); model = {8{4'hC, a[27:0]}}; endfunction
    integer fetches=0; reg [2:0] sh=0; reg [28:0] maddr_l;
    always @(posedge clk) begin
        mig_rd_valid<=1'b0;
        if (mig_en) begin maddr_l<=mig_addr; if (sh==0) fetches<=fetches+1; end
        sh <= {sh[1:0], mig_en};
        if (sh[2]) begin mig_rd_valid<=1'b1; mig_rd_data<=model(maddr_l); end
    end

    integer errs=0, fexp;
    task lookup(input [25:0] b); begin
        @(negedge clk); lu_burst<=b; lu_valid<=1; @(negedge clk); lu_valid<=0;
        @(posedge clk); while(!lu_done) @(posedge clk);
    end endtask
    task chk(input [25:0] b, input integer exp_fetches, input [255:0] tag); begin
        lookup(b);
        if (lu_data!==model({b,3'b000})) begin $display("FAIL %0s burst=%h data=%h exp %h",tag,b,lu_data,model({b,3'b000})); errs=errs+1; end
        if (fetches!==exp_fetches) begin $display("FAIL %0s burst=%h fetches=%0d exp %0d",tag,b,fetches,exp_fetches); errs=errs+1; end
        else $display("  OK %0s burst=%h fetches=%0d",tag,b,fetches);
    end endtask

    initial begin
        repeat(3) @(negedge clk); rst<=0; @(negedge clk);
        // index 0x10; tags 1,2,3,4,5 -> bursts {tag,0x10}
        chk(26'h0000110, 1, "miss A");      // first: miss -> fetch (fetches=1)
        chk(26'h0000110, 1, "hit  A");      // again: HIT, no new fetch (fetches stays 1)
        chk(26'h0000210, 2, "miss B");      // way 2: miss
        chk(26'h0000210, 2, "hit  B");      // hit
        chk(26'h0000310, 3, "miss C");      // way 3
        chk(26'h0000410, 4, "miss D");      // way 4 (set now full: tags 1,2,3,4)
        chk(26'h0000110, 4, "hit  A2");     // A still cached (4-way holds all 4)
        chk(26'h0000510, 5, "miss E");      // 5th tag -> eviction of one way
        // A may or may not survive depending on PLRU victim; just confirm E is cached now:
        chk(26'h0000510, 5, "hit  E");      // E hit, no new fetch

        $display("hits=%0d misses=%0d fetches=%0d", n_hit, n_miss, fetches);
        if (errs==0) $display("tb_tex_cache: PASS");
        else         $display("tb_tex_cache: FAIL (%0d errors)", errs);
        $finish;
    end
    initial begin #100000; $display("tb_tex_cache: TIMEOUT"); $finish; end
endmodule
