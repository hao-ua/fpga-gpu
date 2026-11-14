`timescale 1ns/1ps
// tb_tex_cache_wrap — exercises the A2#3/#7 generation-tag invalidation AND the gen-wrap SWEEP path
// (untested by tb_tex_cache, which never drives inval). Uses GENW=3 so the generation wraps after 7
// invals -> the 256-cycle sweep fires quickly. Verifies: (1) inval invalidates every line (next lookup
// re-fetches), (2) fills after an inval hit again, (3) across the wrap+sweep, NO stale line falsely hits
// and new fills still work (i.e. the sweep truly cleared everything and cur_gen restarted correctly).
module tb_tex_cache_wrap;
    reg clk=0; always #5 clk=~clk;
    reg rst=1, inval=0;
    reg         lu_valid=0; reg [25:0] lu_burst; wire lu_done; wire [255:0] lu_data;
    wire        mig_en; wire [28:0] mig_addr; reg mig_rd_valid=0; reg [255:0] mig_rd_data;
    wire [31:0] n_hit, n_miss;
    tex_cache #(.GENW(3), .PF(1'b0)) DUT(.clk,.rst,.inval,.lu_valid,.lu_burst,.lu_done,.lu_data,   // PF 0: exact fetch counts (prefetch: tb_tex_cache_pf)
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

    integer errs=0, prev;
    task lookup(input [25:0] b); begin
        @(negedge clk); while (!DUT.lu_ready) @(negedge clk);   // wait out any in-progress sweep
        lu_burst<=b; lu_valid<=1; @(negedge clk); lu_valid<=0;
        @(posedge clk); while(!lu_done) @(posedge clk);
    end endtask
    // expect_fetch=1 -> this lookup must MISS (fetch++); 0 -> must HIT (no fetch). Always checks data.
    task chk(input [25:0] b, input integer expect_fetch, input [255:0] tag); begin
        prev=fetches; lookup(b);
        if (lu_data!==model({b,3'b000})) begin $display("FAIL %0s burst=%h data=%h exp %h",tag,b,lu_data,model({b,3'b000})); errs=errs+1; end
        if ((fetches-prev)!==expect_fetch) begin $display("FAIL %0s burst=%h dfetch=%0d exp %0d",tag,b,fetches-prev,expect_fetch); errs=errs+1; end
        else $display("  OK %0s burst=%h dfetch=%0d",tag,b,fetches-prev);
    end endtask
    task do_inval; begin @(negedge clk); inval<=1; @(negedge clk); inval<=0; end endtask

    integer g;
    initial begin
        repeat(3) @(negedge clk); rst<=0; @(negedge clk);
        chk(26'h0000110, 1, "fill A");        // miss -> fetch
        chk(26'h0000110, 0, "hit  A");        // hit, no fetch
        // 10 invalidation cycles (GENW=3 wraps after 7 -> the sweep fires mid-loop). After EACH inval,
        // the previously-cached line must MISS (re-fetch); then it caches again (hit).
        for (g=0; g<10; g=g+1) begin
            do_inval;
            chk(26'h0000110, 1, "post-inval miss");   // inval must invalidate -> re-fetch
            chk(26'h0000110, 0, "re-cached hit");     // cached again under the new gen
        end
        // After crossing the wrap/sweep, a DIFFERENT never-seen line must still MISS (no false hit from
        // a stale gen aliasing cur_gen), and a distinct set of lines must all cache correctly.
        chk(26'h0000730, 1, "new line miss");
        chk(26'h0000730, 0, "new line hit");
        chk(26'h0000110, 0, "A still hit");           // A cached under the current gen

        if (errs==0) $display("tb_tex_cache_wrap: PASS (gen invalidation + wrap sweep correct)");
        else         $display("tb_tex_cache_wrap: FAIL (%0d errors)", errs);
        $finish;
    end
    initial begin #200000; $display("tb_tex_cache_wrap: TIMEOUT"); $finish; end
endmodule
