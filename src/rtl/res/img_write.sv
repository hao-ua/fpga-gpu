// img_write — board B transfer engine, WRITE_IMAGE path, clk_sys,
// on mem_arb write port W0. One command at a time, one pixel per clock:
//   payload words (little-endian bytes) -> byte unpacker (tbpp bytes per pixel) -> decoder (transfer format ->
//   RGBA8, registered) -> encoder (RGBA8 -> storage format) -> rect walker -> 32-byte bursts with byte masks -> W0.
// The caller (fb_cmd_b) has validated the command and computed the rectangle's geometry:
//   cmd_row    byte address of the start of the first pixel's row in the level (base + level/layer offset
//              + (y + first_pixel / w) * pitch)
//   cmd_pitch  row pitch of the level, cmd_x / cmd_w the rectangle's x and width, cmd_i0 = first_pixel % w,
//   cmd_npix   pixels carried by the payload (>= 1), exactly cmd_npix * tbpp payload bytes follow.
// Transfer formats: 0x00 RAW (storage bytes as they are), 0x01 RGBA8888 (R, G, B, A), 0x02 BGRA8888,
// 0x03 BGRX8888 (alpha 255), 0x04 RGB565 (16-bit little-endian), 0x0A RGB888, 0x0B BGR888.
// Storage formats (cmd_sf): 0 RGBA8 = 32-bit little-endian word 0xRRGGBBAA, 1 RGB565 = 16-bit little-endian
// {R5, G6, B5}, 2 P8 = one byte (palette index; RAW only).
// Conversions (ES 2.0 §2.1.2 / §4.3.1): 5 / 6 -> 8 bits exact, round(c * 255 / 31) and
// round(c * 255 / 63); 8 -> 5 / 6 bits rounded to nearest (round(v * 31 / 255)); no ties exist. Pixels never
// straddle a burst (pixel addresses are bpp aligned: base 32-byte aligned, level offsets 4-byte aligned, pitch and
// x * bpp multiples of bpp).
// `busy` falls only after every write command and data beat has been accepted by the arbiter (MIG strict
// ordering then makes the data visible to any later read), as in xfer_lin.
// SHALLOW_RAM: the style of the shallow, wide FIFO (u_bf) — block RAM, or LUTs in the one-board build (ONE_CHIP), where
// block RAM is the scarcer resource; the behaviour is the same
`ifndef SHALLOW_RAM
`ifdef ONE_CHIP
`define SHALLOW_RAM "distributed"
`else
`define SHALLOW_RAM "block"
`endif
`endif
`default_nettype none
module img_write (
    input  wire         clk, rst,
    input  wire         cmd_valid,
    output wire         cmd_ready,
    input  wire [29:0]  cmd_row,
    input  wire [15:0]  cmd_pitch,
    input  wire [12:0]  cmd_x, cmd_w, cmd_i0,
    input  wire [23:0]  cmd_npix,
    input  wire [7:0]   cmd_tfmt,
    input  wire [1:0]   cmd_sf,                 // storage format: 0 RGBA8, 1 RGB565, 2 P8
    // payload words
    input  wire         d_valid,
    input  wire [31:0]  d_data,
    output wire         d_ready,
    output wire         busy,
    output reg  [31:0]  c_pixels,               // pixels written (statistics)
    // mem_arb write port
    output wire         w_req, w_en,
    output wire [28:0]  w_addr,
    output wire [255:0] w_data,
    output wire [31:0]  w_mask,
    output wire         w_wren, w_wend,
    input  wire         w_rdy, w_wdf_rdy
);
    localparam [7:0] TF_RAW = 8'h00, TF_RGBA = 8'h01, TF_BGRA = 8'h02, TF_BGRX = 8'h03, TF_565 = 8'h04,
                     TF_RGB = 8'h0A, TF_BGR = 8'h0B;
    // exact 5 / 6 -> 8 bit expansion (as img_xfer)
    localparam logic [7:0] E5 [0:31] = '{8'd0, 8'd8, 8'd16, 8'd25, 8'd33, 8'd41, 8'd49, 8'd58, 8'd66, 8'd74, 8'd82,
        8'd90, 8'd99, 8'd107, 8'd115, 8'd123, 8'd132, 8'd140, 8'd148, 8'd156, 8'd165, 8'd173, 8'd181, 8'd189, 8'd197,
        8'd206, 8'd214, 8'd222, 8'd230, 8'd239, 8'd247, 8'd255};
    localparam logic [7:0] E6 [0:63] = '{8'd0, 8'd4, 8'd8, 8'd12, 8'd16, 8'd20, 8'd24, 8'd28, 8'd32, 8'd36, 8'd40,
        8'd45, 8'd49, 8'd53, 8'd57, 8'd61, 8'd65, 8'd69, 8'd73, 8'd77, 8'd81, 8'd85, 8'd89, 8'd93, 8'd97, 8'd101,
        8'd105, 8'd109, 8'd113, 8'd117, 8'd121, 8'd125, 8'd130, 8'd134, 8'd138, 8'd142, 8'd146, 8'd150, 8'd154, 8'd158,
        8'd162, 8'd166, 8'd170, 8'd174, 8'd178, 8'd182, 8'd186, 8'd190, 8'd194, 8'd198, 8'd202, 8'd206, 8'd210, 8'd215,
        8'd219, 8'd223, 8'd227, 8'd231, 8'd235, 8'd239, 8'd243, 8'd247, 8'd251, 8'd255};

    // 8 -> 5 / 6 bits, round to nearest: round(x / 255) with x = v * (2^n - 1) = (v << n) - v, by Blinn's exact
    // division (x + 128 + ((x + 128) >> 8)) >> 8 (exact for 0 <= x <= 255 * 255); shifts and adds only
    function automatic [4:0] r5(input [7:0] v);
        reg [13:0] t;
        t = ({1'b0, v, 5'b0} - {6'b0, v}) + 14'd128; r5 = 5'((t + (t >> 8)) >> 8);
    endfunction
    function automatic [5:0] r6(input [7:0] v);
        reg [14:0] t;
        t = ({1'b0, v, 6'b0} - {7'b0, v}) + 15'd128; r6 = 6'((t + (t >> 8)) >> 8);
    endfunction

    // ---------------- command state ----------------
    typedef enum logic [1:0] {I_IDLE, I_RUN, I_DRAIN} ist_t;
    ist_t       is;
    reg  [7:0]  tfmt;
    reg  [1:0]  sf;
    wire        s565 = (sf == 2'd1);
    reg  [2:0]  tbpp;                           // transfer bytes per pixel
    reg  [15:0] pitch;
    reg  [12:0] w, col;                         // rect width, column of the next pixel (walker)
    reg  [29:0] row_start;                      // address of the rect's first pixel in the current row
    reg  [29:0] paddr;                          // address of the next pixel
    reg  [23:0] px_in;                          // pixels still to unpack
    reg  [21:0] words_in;                       // payload words still to accept
    assign cmd_ready = (is == I_IDLE);

    // ---------------- stage 0: byte unpacker ----------------
    reg  [63:0] bb;                             // byte buffer, byte 0 = next
    reg  [3:0]  bn;                             // bytes held
    wire        s0_emit_ok = (is == I_RUN) && px_in != 0 && bn >= {1'b0, tbpp};
    wire        go;                             // the pipeline may advance (room for a burst)
    wire        s0_emit = s0_emit_ok && go;
    wire [3:0]  bn_after = bn - (s0_emit ? {1'b0, tbpp} : 4'd0);
    assign d_ready = (is == I_RUN) && words_in != 0 && bn_after <= 4'd4;
    wire        take = d_valid && d_ready;
    wire [63:0] bb_shift = s0_emit ? (bb >> (8 * tbpp)) : bb;

    // ---------------- stage 1: decode + encode ----------------
    reg         p1_v, p1_last;
    reg  [31:0] p1_b;                           // the pixel's transfer bytes (byte 0 in [7:0])
    reg  [29:0] p1_addr;
    reg  [7:0]  cr, cg, cb, ca;
    always @(*) begin
        case (tfmt)
            TF_RGBA: begin cr = p1_b[7:0];   cg = p1_b[15:8]; cb = p1_b[23:16]; ca = p1_b[31:24]; end
            TF_BGRA: begin cb = p1_b[7:0];   cg = p1_b[15:8]; cr = p1_b[23:16]; ca = p1_b[31:24]; end
            TF_BGRX: begin cb = p1_b[7:0];   cg = p1_b[15:8]; cr = p1_b[23:16]; ca = 8'hFF; end
            TF_RGB:  begin cr = p1_b[7:0];   cg = p1_b[15:8]; cb = p1_b[23:16]; ca = 8'hFF; end
            TF_BGR:  begin cb = p1_b[7:0];   cg = p1_b[15:8]; cr = p1_b[23:16]; ca = 8'hFF; end
            default: begin                    // TF_565: {R5, G6, B5}, exact expansion
                cr = E5[p1_b[15:11]]; cg = E6[p1_b[10:5]]; cb = E5[p1_b[4:0]]; ca = 8'hFF;
            end
        endcase
    end
    // stage 1b: decoded pixel (registered), encoded into the storage format on the way to stage 2
    reg         q_v, q_last;
    reg  [29:0] q_addr;
    reg  [31:0] q_raw;
    reg  [7:0]  q_r, q_g, q_b, q_a;
    wire [31:0] enc = (tfmt == TF_RAW) ? q_raw :
                      s565 ? {16'h0, r5(q_r), r6(q_g), r5(q_b)} : {q_r, q_g, q_b, q_a};

    // ---------------- stage 2: burst assembly ----------------
    reg         p2_v, p2_last;
    reg  [31:0] p2_d;
    reg  [29:0] p2_addr;
    reg  [255:0] bur_d;
    reg  [31:0] bur_be;                         // bytes written in the burst being assembled
    reg  [24:0] bur_i;                          // its burst index
    reg         bur_have;

    // burst FIFO {burst index, mask (1 = byte not written), data}
    reg          bf_wen; reg [312:0] bf_wd;
    wire         bf_v; wire [312:0] bf_d; wire [5:0] bf_cnt; wire bf_pop;
    sfifo #(.W(313), .AW(4), .RAM_STYLE(`SHALLOW_RAM)) u_bf (.clk(clk), .rst(rst), .wen(bf_wen), .wdata(bf_wd),  // 210 LUT as LUTRAM; same cycles
        .full(), .count(bf_cnt), .ren(bf_pop), .rdata(bf_d), .rvalid(bf_v));
    assign go = (bf_cnt < 6'd13);               // up to 2 pushes in flight behind the check

    // issue: data beat first, then its command (as xfer_lin), address latched from the popped entry
    reg  [31:0] wr_dat, wr_cmd, n_push;
    reg  [24:0] cmd_bi;
    wire        data_turn = (wr_dat == wr_cmd);
    assign w_wren = bf_v && data_turn;
    assign w_wend = w_wren;
    assign w_data = bf_d[255:0];
    assign w_mask = bf_d[287:256];
    assign bf_pop = w_wren && w_wdf_rdy;
    assign w_en   = !data_turn;
    assign w_addr = {1'b0, cmd_bi, 3'b000};
    assign w_req  = bf_v || !data_turn;

    assign busy = (is != I_IDLE);

    wire [1:0]  bl = (sf == 2'd0) ? 2'd2 : (sf == 2'd1) ? 2'd1 : 2'd0;   // log2 storage bpp
    wire [24:0] p2_bi = p2_addr[29:5];
    wire [4:0]  p2_o  = p2_addr[4:0];
    wire [31:0] p2_pm = (bl == 2'd2) ? 32'hFFFFFFFF : (bl == 2'd1) ? 32'h0000FFFF : 32'h000000FF;
    wire [31:0] p2_be = ((bl == 2'd2) ? 32'hF : (bl == 2'd1) ? 32'h3 : 32'h1) << p2_o;
    wire [255:0] p2_dm = {224'h0, p2_pm} << {p2_o, 3'b000};
    wire [255:0] p2_dv = {224'h0, p2_d & p2_pm} << {p2_o, 3'b000};
    wire         p2_new = bur_have && p2_bi != bur_i;          // the pixel opens a new burst
    wire [255:0] mrg_d  = (bur_d & ~p2_dm) | p2_dv;            // the pixel merged into the open burst
    wire [31:0]  mrg_be = bur_be | p2_be;

    always @(posedge clk) begin
        bf_wen <= 1'b0;
        if (rst) begin
            is <= I_IDLE; p1_v <= 1'b0; q_v <= 1'b0; p2_v <= 1'b0; bur_have <= 1'b0; bn <= 0; px_in <= 0; words_in <= 0;
            wr_dat <= 0; wr_cmd <= 0; n_push <= 0; c_pixels <= 0;
        end else begin
            if (bf_pop) begin wr_dat <= wr_dat + 1; cmd_bi <= bf_d[312:288]; end
            if (w_en && w_rdy) wr_cmd <= wr_cmd + 1;
            case (is)
                I_IDLE: if (cmd_valid) begin : start
                    reg [1:0] cbl; reg [23:0] ctb;
                    cbl = (cmd_sf == 2'd0) ? 2'd2 : (cmd_sf == 2'd1) ? 2'd1 : 2'd0;
                    ctb = (cmd_tfmt == TF_RAW) ? (24'd1 << cbl) : (cmd_tfmt == TF_565) ? 24'd2 :
                          (cmd_tfmt == TF_RGB || cmd_tfmt == TF_BGR) ? 24'd3 : 24'd4;
                    tfmt <= cmd_tfmt; sf <= cmd_sf; pitch <= cmd_pitch; w <= cmd_w; col <= cmd_i0;
                    row_start <= cmd_row + ({17'h0, cmd_x} << cbl);
                    paddr <= cmd_row + ({17'h0, cmd_x + cmd_i0} << cbl);
                    tbpp <= ctb[2:0];
                    px_in <= cmd_npix;
                    begin : nbytes                      // npix x {1, 2, 3, 4} bytes as shifts / an add, not a DSP multiply
                        reg [23:0] npb;                 // (timing: the DSP's 5.4 ns of logic would head a failing path)
                        npb = (ctb == 24'd1) ? cmd_npix : (ctb == 24'd2) ? {cmd_npix[22:0], 1'b0} :
                              (ctb == 24'd3) ? ({cmd_npix[22:0], 1'b0} + cmd_npix) : {cmd_npix[21:0], 2'b00};
                        words_in <= (npb + 24'd3) >> 2;
                    end
                    bb <= 0; bn <= 0; bur_have <= 1'b0; bur_be <= 0; bur_d <= '0;
                    wr_dat <= 0; wr_cmd <= 0; n_push <= 0;
                    is <= I_RUN;
                end
                I_RUN: begin
                    // ---- stage 0 ----
                    if (take) begin
                        bb <= bb_shift | ({32'h0, d_data} << (8 * bn_after));
                        words_in <= words_in - 1'b1;
                    end else bb <= bb_shift;
                    bn <= bn_after + (take ? 4'd4 : 4'd0);
                    if (go) begin
                        p1_v <= s0_emit;
                        if (s0_emit) begin
                            p1_b <= bb[31:0]; p1_addr <= paddr; p1_last <= (px_in == 24'd1);
                            px_in <= px_in - 1'b1;
                            if (col == w - 13'd1) begin
                                col <= 0; row_start <= row_start + {14'h0, pitch}; paddr <= row_start + {14'h0, pitch};
                            end else begin
                                col <= col + 13'd1; paddr <= paddr + (30'd1 << bl);
                            end
                        end
                        // ---- stage 1: decode ----
                        q_v <= p1_v;
                        if (p1_v) begin
                            q_raw <= p1_b; q_r <= cr; q_g <= cg; q_b <= cb; q_a <= ca; q_addr <= p1_addr; q_last <= p1_last;
                        end
                        // ---- stage 1b: encode ----
                        p2_v <= q_v;
                        if (q_v) begin p2_d <= enc; p2_addr <= q_addr; p2_last <= q_last; end
                        // ---- stage 2 ----
                        if (p2_v) begin
                            c_pixels <= c_pixels + 1;
                            bur_i <= p2_bi;
                            if (p2_new) begin
                                // push the open burst; this pixel opens the next one (pushed in I_DRAIN if last)
                                bf_wen <= 1'b1; bf_wd <= {bur_i, ~bur_be, bur_d}; n_push <= n_push + 1;
                                bur_d <= p2_dv; bur_be <= p2_be; bur_have <= 1'b1;
                                if (p2_last) is <= I_DRAIN;
                            end else if (p2_last) begin
                                bf_wen <= 1'b1; bf_wd <= {p2_bi, ~mrg_be, mrg_d}; n_push <= n_push + 1;
                                bur_have <= 1'b0; bur_be <= 0;
                                is <= I_DRAIN;
                            end else begin
                                bur_d <= mrg_d; bur_be <= mrg_be; bur_have <= 1'b1;
                            end
                        end
                    end
                end
                I_DRAIN: begin
                    // a pending final burst (the last pixel opened a new one), then wait for the arbiter
                    if (bur_have && !bf_wen && go) begin
                        bf_wen <= 1'b1; bf_wd <= {bur_i, ~bur_be, bur_d}; n_push <= n_push + 1; bur_have <= 1'b0;
                    end
                    p1_v <= 1'b0; q_v <= 1'b0; p2_v <= 1'b0;
                    if (!bur_have && !bf_wen && !bf_v && wr_dat == n_push && wr_cmd == n_push) is <= I_IDLE;
                end
                default: is <= I_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
