// buf_table — buffer descriptor table: 1024 entries of
// {usage[3:0], base[29:5] (32-byte aligned), size[29:0]} in block RAM. Read port has one cycle of latency.
// Validity: a GENERATION TAG stored with the entry instead of 1 024 valid flip-flops and
// their 1 024:1 read mux (≈ 1.6 k LUT on each board). `gen` counts the resets (initialised at configuration, +1 per
// reset edge, never 0); an entry is valid when its stored tag equals the current generation and its valid bit is set,
// so a reset invalidates the whole table at once without clearing the RAM (the RAM powers up as 0 = tag 0, never
// current). A stale entry becomes visible again only after 4 095 further resets with that index never rewritten.
// `rvalid` is the tag compare on the registered RAM output (a 12-bit equality: its consumers register the result
// before their range checks — fb_cmd_b's bq_*, fb_dispatch_a's D_LOOK2 decision).
`default_nettype none
module buf_table (
    input  wire        clk, rst,
    input  wire        we,
    input  wire [9:0]  widx,
    input  wire        wvalid,
    input  wire [3:0]  wusage,
    input  wire [29:0] wbase,
    input  wire [29:0] wsize,
    input  wire [9:0]  ridx,
    output wire        rvalid,
    output reg  [3:0]  rusage,
    output wire [29:0] rbase,
    output reg  [29:0] rsize
);
    reg  [11:0] gen = 12'd1;                        // the current generation (configuration value 1, counts resets)
    reg         rst_d = 1'b0;
    always @(posedge clk) begin
        rst_d <= rst;
        if (rst & ~rst_d) gen <= (gen == 12'hFFF) ? 12'd1 : gen + 12'd1;
    end
    (* ram_style = "block" *) reg [71:0] mem [0:1023];   // {tag[11:0], valid, usage[3:0], base[24:0], size[29:0]} = 72 bits
    initial for (int i = 0; i < 1024; i++) mem[i] = 72'h0;   // the configuration value (tag 0 = never current); X in xsim otherwise
    reg  [11:0]  rtag; reg rv;
    reg  [24:0]  rb;
    always @(posedge clk) begin
        if (we & ~rst) mem[widx] <= {gen, wvalid, wusage, wbase[29:5], wsize};
        {rtag, rv, rusage, rb, rsize} <= mem[ridx];
    end
    assign rvalid = rv & (rtag == gen) & ~rst;
    assign rbase  = {rb, 5'b00000};
endmodule
`default_nettype wire
