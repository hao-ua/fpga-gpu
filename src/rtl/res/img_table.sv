// img_table — image descriptor table of board B: 1024 entries of
// img_pkg::img_desc_t (128 bits) in block RAM, valid bits in flip-flops so a reset clears the table at once.
// Two independent read ports (the memory is kept twice, every write goes to both copies), each with one
// cycle of latency: port 0 for the bring-up harness, port 1 for the command processor.
`default_nettype none
module img_table
    import img_pkg::*;
(
    input  wire        clk, rst,
    input  wire        we,
    input  wire [9:0]  widx,
    input  wire        wvalid,
    input  img_desc_t  wdesc,
    input  wire [9:0]  ridx,
    output reg         rvalid,
    output img_desc_t  rdesc,
    input  wire [9:0]  ridx2,
    output reg         rvalid2,
    output img_desc_t  rdesc2
);
    (* ram_style = "block" *) reg [127:0] mem  [0:1023];
    (* ram_style = "block" *) reg [127:0] mem2 [0:1023];
    reg [1023:0] vbits;
    // the write is registered here first (timing: the command processor's write enable would travel across the die
    // into the 1 024 valid bits); a write lands one cycle later — every writer reads back no earlier than
    // two cycles after its write (fb_cmd_b: a command's write, then the next command's header; b_bringup: JTAG-paced)
    reg we_r; reg [9:0] widx_r; reg wvalid_r; img_desc_t wdesc_r;
    always @(posedge clk) begin we_r <= we & ~rst; widx_r <= widx; wvalid_r <= wvalid; wdesc_r <= wdesc; end
    always @(posedge clk) begin
        if (we_r) mem[widx_r] <= wdesc_r;
        rdesc <= mem[ridx];
    end
    // port 2 (the command processor) has the block RAM's output register as well (timing: else the RAM's 2.1 ns
    // clock-to-out sits in front of fb_cmd_b's checks): the
    // descriptor arrives one cycle later, which every read of fb_cmd_b already allows for (set ridx2, one wait state, use)
    img_desc_t rdesc2_q; reg rvalid2_q;
    always @(posedge clk) begin
        if (we_r) mem2[widx_r] <= wdesc_r;
        rdesc2_q <= mem2[ridx2];
        rdesc2 <= rdesc2_q;
    end
    always @(posedge clk) begin
        if (rst) vbits <= '0;
        else if (we_r) vbits[widx_r] <= wvalid_r;
        rvalid  <= rst ? 1'b0 : vbits[ridx];
        rvalid2_q <= rst ? 1'b0 : vbits[ridx2];
        rvalid2 <= rst ? 1'b0 : rvalid2_q;
    end
endmodule
`default_nettype wire
