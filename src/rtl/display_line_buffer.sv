// display_line_buffer: small dual-clock ping/pong LINE buffer between the
// (linear) display_reader (ui_clk producer) and display_out (pixel_clk
// consumer). Replaces the 32-line tile-row buffer now that the framebuffer is
// LINEAR scanline order: DRAM -> line buffer -> HDMI. Each slot holds ONE
// scanline; N_LINES slots ping/pong. ~N_LINES*MAX_BURSTS_PER_LINE*256b BRAM
// (16*160 = 2560 x 256b ~ 18 RAMB36 at 1280-wide, vs ~73 for the 64-line
// tile-row buffer). N_LINES=16 gives ample read-ahead so the reader can burst
// then yield the MEMORY arbiter to the renderer/TMU between refills.
//
// CDC: each slot has a 1-bit ownership toggle crossed by a 2FF synchronizer
// (proven scheme from the tile-row buffer) — metastability-safe for any N_LINES.
//   slot i FULL (consumer may read) when wr_fill[i] != rd_drain[i]
//   slot i FREE (producer may write) when wr_fill[i] == rd_drain[i]
`timescale 1ns/1ps
module display_line_buffer #(
    parameter int N_LINES             = 16,       // ping/pong line slots (>=8)
    parameter int MAX_BURSTS_PER_LINE = 160,      // 1280/8
    parameter int MAX_PIXELS_PER_LINE = 1280
)(
    // ----- Write side (ui_clk) -----
    input  wire         wr_clk,
    input  wire         wr_rst,
    input  wire         wr_burst_en,
    input  wire [$clog2(MAX_BURSTS_PER_LINE)-1:0] wr_burst_idx,   // x (burst in line)
    input  wire [255:0] wr_burst_data,
    input  wire         wr_line_done,       // pulse: release this line slot
    input  wire         wr_frame_sync,
    output wire         wr_buf_avail,        // a free slot exists

    // ----- Read side (pixel_clk) -----
    input  wire         rd_clk,
    input  wire         rd_rst,
    input  wire         rd_pixel_en,
    input  wire [$clog2(MAX_PIXELS_PER_LINE)-1:0] rd_pixel_idx,
    output wire [31:0]  rd_pixel,
    input  wire         rd_line_done,       // pulse: free the consumed line slot
    input  wire         rd_frame_sync,
    output wire         rd_buf_avail         // a filled line is ready
);
    localparam int LW = $clog2(N_LINES);
    localparam int BW = $clog2(MAX_BURSTS_PER_LINE);

    (* ram_style="block" *) reg [255:0] mem [0:N_LINES*MAX_BURSTS_PER_LINE-1];

    reg [LW-1:0]      wr_idx, rd_idx;
    reg [N_LINES-1:0] wr_fill, rd_drain;
    (* ASYNC_REG="TRUE" *) reg [N_LINES-1:0] fill_s1, fill_s2;     // wr_fill -> rd
    (* ASYNC_REG="TRUE" *) reg [N_LINES-1:0] drain_s1, drain_s2;   // rd_drain -> wr

    // ---- producer (ui_clk) ----
    always @(posedge wr_clk) begin
        if (wr_rst || wr_frame_sync) begin wr_idx<='0; wr_fill<='0; end
        else if (wr_line_done) begin
            wr_fill[wr_idx] <= ~wr_fill[wr_idx];
            wr_idx <= (wr_idx==LW'(N_LINES-1)) ? '0 : wr_idx+1'b1;
        end
    end
    always @(posedge wr_clk) begin drain_s1<=rd_drain; drain_s2<=drain_s1; end
    reg [3:0] wr_settle;
    always @(posedge wr_clk) begin
        if (wr_rst) wr_settle<=0; else if (wr_frame_sync) wr_settle<=4'd8;
        else if (wr_settle!=0) wr_settle<=wr_settle-4'd1;
    end
    assign wr_buf_avail = (wr_settle==0) && (wr_fill[wr_idx]==drain_s2[wr_idx]);

    localparam int AW = $clog2(N_LINES*MAX_BURSTS_PER_LINE);
    wire [AW-1:0] wr_addr = wr_idx*MAX_BURSTS_PER_LINE + wr_burst_idx;
    always @(posedge wr_clk) if (wr_burst_en && wr_buf_avail) mem[wr_addr] <= wr_burst_data;

    // ---- consumer (pixel_clk) ----
    always @(posedge rd_clk) begin
        if (rd_rst || rd_frame_sync) begin rd_idx<='0; rd_drain<='0; end
        else if (rd_line_done) begin
            rd_drain[rd_idx] <= ~rd_drain[rd_idx];
            rd_idx <= (rd_idx==LW'(N_LINES-1)) ? '0 : rd_idx+1'b1;
        end
    end
    always @(posedge rd_clk) begin fill_s1<=wr_fill; fill_s2<=fill_s1; end
    reg [3:0] rd_settle;
    always @(posedge rd_clk) begin
        if (rd_rst) rd_settle<=0; else if (rd_frame_sync) rd_settle<=4'd8;
        else if (rd_settle!=0) rd_settle<=rd_settle-4'd1;
    end
    assign rd_buf_avail = (rd_settle==0) && (fill_s2[rd_idx]!=rd_drain[rd_idx]);

    wire [AW-1:0] rd_addr = rd_idx*MAX_BURSTS_PER_LINE + rd_pixel_idx[BW+2:3];
    reg [255:0] rd_burst_q; reg [2:0] rd_sel_q;
    always @(posedge rd_clk) if (rd_pixel_en) begin
        rd_burst_q <= mem[rd_addr]; rd_sel_q <= rd_pixel_idx[2:0];
    end
    assign rd_pixel = rd_burst_q[rd_sel_q*32 +: 32];
endmodule
`default_nettype wire
