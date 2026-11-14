// ddr_fifo — a FIFO of 32-byte bursts kept in a DDR ring (board A ingress queue). Push side and pop side are 256-bit streams; a BRAM staging
// FIFO on each side decouples them from the memory arbiter.
//   used = bursts pushed and not yet popped (ring + both staging FIFOs). The producer checks
//   RING - used before committing a multi-burst entry; in_ready additionally guards the ring.
// A burst becomes readable once its write command has been accepted: MIG strict ordering makes a later
// read of that address return the written data.
// Write master protocol: one data beat ahead of its command at most (data first, then command).
`default_nettype none
module ddr_fifo #(
    parameter [28:0] BASE      = 29'h0,       // ring base, app_addr units (4 bytes), burst aligned
    parameter int    RING_LOG2 = 21,          // ring size in bursts (2**21 x 32 B = 64 MB)
    parameter int    IN_AW     = 9,
    parameter int    OUT_AW    = 9
)(
    input  wire              clk, rst,
    input  wire              in_valid,
    input  wire [255:0]      in_data,
    output wire              in_ready,
    output wire              out_valid,
    output wire [255:0]      out_data,
    input  wire              out_ready,
    output wire [RING_LOG2:0] used,
    // mem_arb write port
    output wire              w_req,
    output wire              w_en,
    output wire [28:0]       w_addr,
    output wire [255:0]      w_data,
    output wire [31:0]       w_mask,
    output wire              w_wren, w_wend,
    input  wire              w_rdy, w_wdf_rdy,
    // mem_arb read port
    output wire              r_en,
    output wire [28:0]       r_addr,
    input  wire              r_rdy, r_valid,
    input  wire [255:0]      rd_data
);
    localparam int RING = 1 << RING_LOG2;
    localparam int OUT_DEPTH = 1 << OUT_AW;

    reg  [RING_LOG2:0] pushed, popped, wr_dat, wr_cmd, rd_cmd, rd_ret;
    // used and the read request are REGISTERS of their next-cycle values (timing at 120 MHz: `used` would feed the
    // protocol's gap / full decision 12 levels deep, and the read request — the staging FIFO's count + the outstanding
    // reads against its depth — would head the arbiter's grant chain into every other port's ready); cycle-exact
    reg  [RING_LOG2:0] used_r;
    assign used = used_r;

    // ---------------- push side ----------------
    wire         sin_full, sin_v; wire [255:0] sin_d; wire sin_pop;
    assign in_ready = !sin_full && (used < RING);
    sfifo #(.W(256), .AW(IN_AW)) u_sin (.clk(clk), .rst(rst), .wen(in_valid & in_ready), .wdata(in_data),
        .full(sin_full), .count(), .ren(sin_pop), .rdata(sin_d), .rvalid(sin_v));

    wire data_turn = (wr_dat == wr_cmd);
    assign w_wren  = sin_v && data_turn;
    assign w_wend  = w_wren;
    assign w_data  = sin_d;
    assign w_mask  = 32'h0;
    assign sin_pop = w_wren && w_wdf_rdy;
    assign w_en    = !data_turn;
    assign w_addr  = BASE + {wr_cmd[RING_LOG2-1:0], 3'b000};
    assign w_req   = sin_v || !data_turn;

    // ---------------- pop side ----------------
    wire         sout_v; wire [255:0] sout_d;
    // occ = reads accepted and not yet popped (= the outstanding reads + the staging FIFO's count: every accepted read
    // returns one word into it); a read may be issued while occ < OUT_DEPTH
    reg  [OUT_AW+1:0] occ;
    reg          have_r, room_r;               // a burst is readable (rd_cmd != wr_cmd) / the staging FIFO has room
    assign r_en   = have_r & room_r;
    assign r_addr = BASE + {rd_cmd[RING_LOG2-1:0], 3'b000};
    sfifo #(.W(256), .AW(OUT_AW)) u_sout (.clk(clk), .rst(rst), .wen(r_valid), .wdata(rd_data),
        .full(), .count(), .ren(out_valid & out_ready), .rdata(sout_d), .rvalid(sout_v));
    assign out_valid = sout_v;
    assign out_data  = sout_d;
    wire push = in_valid & in_ready, pop = out_valid & out_ready, w_acc = w_en & w_rdy, r_acc = r_en & r_rdy;
    wire [RING_LOG2:0] wr_cmd_n = wr_cmd + (RING_LOG2+1)'(w_acc), rd_cmd_n = rd_cmd + (RING_LOG2+1)'(r_acc);
    wire [OUT_AW+1:0]  occ_n = occ + (OUT_AW+2)'(r_acc) - (OUT_AW+2)'(pop);

    always @(posedge clk) begin
        if (rst) begin pushed <= 0; popped <= 0; wr_dat <= 0; wr_cmd <= 0; rd_cmd <= 0; rd_ret <= 0;
                       used_r <= 0; occ <= 0; have_r <= 1'b0; room_r <= 1'b1; end
        else begin
            if (push)    pushed <= pushed + 1'b1;
            if (pop)     popped <= popped + 1'b1;
            if (sin_pop) wr_dat <= wr_dat + 1'b1;
            wr_cmd <= wr_cmd_n; rd_cmd <= rd_cmd_n;
            if (r_valid) rd_ret <= rd_ret + 1'b1;
            used_r <= (pushed + (RING_LOG2+1)'(push)) - (popped + (RING_LOG2+1)'(pop));
            occ    <= occ_n;
            have_r <= (wr_cmd_n != rd_cmd_n);
            room_r <= (occ_n < (OUT_AW+2)'(OUT_DEPTH));
        end
    end
    wire _unused = &{1'b0, rd_ret};
endmodule
`default_nettype wire
