// img_xfer — board B transfer engine for READ_IMAGE, FILL_IMAGE and COPY_IMAGE,
// clk_sys, on mem_arb read port R1 and write port W2 (img_write keeps W0 for WRITE_IMAGE).
// The caller (fb_cmd_b) validates the command and computes the geometry; this engine walks rows:
//   sequencer  one row job after the other: COPY destination rows top-down (bottom-up for a 1:1 copy with dy > sy,
//              which makes overlapping copies exact), the source row from the vertical DDA and mirror; FILL rows;
//              READ rows from the first pixel's row. Each row gives a read job (memory row segment -> row buffer
//              half) and a write job (half -> destination row / READ output); a row whose source row equals the
//              previous one reuses its half (vertical upscale), a plain FILL has no read job.
//   reader     R1 bursts of the segment -> pixel unpacker (1 / 2 / 4 bytes) -> source colour key flag -> row buffer
//              (two halves of 2**RB_LOG2 pixels x {key, raw}); it runs one row ahead of the writer.
//   writer     per destination pixel: buffer index (COPY: horizontal DDA, mirror), buffer read, storage -> RGBA8
//              (RGB565 exact: round(c * 255 / 31), round(c * 255 / 63), alpha 255), -> destination storage format
//              (8 -> 5 / 6 bits rounded to nearest) with byte masks (transparent pixels, FILL channel masks), FILL
//              read-modify-write of RGB565 channels, or -> READ transfer format; then 32-byte bursts -> W2, or the
//              transfer bytes packed little-endian into words -> o_*.
// Geometry inputs: *_row0 = byte address of row 0 of the level (base + level / layer offset), pitches, rectangles.
// COPY DDA (u(i) = floor((2i + 1) * sw / (2 * dw))): hq = sw / dw, hr2 = 2 * (sw % dw), hu0 = sw / (2 * dw),
// ha0 = sw % (2 * dw); likewise v. Storage format codes: 0 RGBA8 (word 0xRRGGBBAA), 1 RGB565 ({R5, G6, B5}), 2 P8
// (palette index; copied / filled / read as raw values only). `busy` falls after every write was accepted by the
// arbiter and every READ word was taken.
// SHALLOW_RAM: the style of the shallow, wide FIFOs (u_df, u_bf) — block RAM, or LUTs in the one-board build
// (ONE_CHIP), where block RAM is the scarcer resource; the behaviour is the same
`ifndef SHALLOW_RAM
`ifdef ONE_CHIP
`define SHALLOW_RAM "distributed"
`else
`define SHALLOW_RAM "block"
`endif
`endif
`default_nettype none
module img_xfer #(
    parameter int RB_LOG2 = 12                    // row buffer half: 2**RB_LOG2 pixels (4096 = the widest image)
)(
    input  wire         clk, rst,
    input  wire         cmd_valid,
    output wire         cmd_ready,
    input  wire [1:0]   cmd_op,                   // 0 COPY, 1 FILL, 2 READ
    input  wire [29:0]  cmd_s_row0,               // COPY source, READ rectangle
    input  wire [15:0]  cmd_s_pitch,
    input  wire [1:0]   cmd_s_fmt,
    input  wire [12:0]  cmd_s_x, cmd_s_y, cmd_s_w, cmd_s_h,
    input  wire [29:0]  cmd_d_row0,               // COPY / FILL destination
    input  wire [15:0]  cmd_d_pitch,
    input  wire [1:0]   cmd_d_fmt,
    input  wire [12:0]  cmd_d_x, cmd_d_y, cmd_d_w, cmd_d_h,
    input  wire [12:0]  cmd_hq, cmd_hu0, cmd_vq, cmd_vu0,
    input  wire [13:0]  cmd_hr2, cmd_ha0, cmd_vr2, cmd_va0,
    input  wire         cmd_mir_x, cmd_mir_y, cmd_bottom_up, cmd_key,
    input  wire [31:0]  cmd_key_lo, cmd_key_hi,
    input  wire [31:0]  cmd_value,                // FILL: one pixel in transfer format cmd_tfmt (RAW: storage value)
    input  wire [3:0]   cmd_mask,                 // FILL: b0 R, b1 G, b2 B, b3 A
    input  wire         cmd_rmw,                  // FILL: RGB565 with a partial R/G/B mask
    input  wire [7:0]   cmd_tfmt,                 // FILL value / READ output transfer format
    input  wire [12:0]  cmd_j0, cmd_i0,           // READ: first pixel = row j0, column i0 of the rectangle
    input  wire [23:0]  cmd_npix,                 // READ: pixels to send (>= 1)
    output wire         busy,
    output wire         o_valid,
    output wire [31:0]  o_data,
    input  wire         o_ready,
    output reg  [31:0]  c_ops, c_pixels,
    // mem_arb read port (R1)
    output wire         r_en,
    output wire [28:0]  r_addr,
    input  wire         r_rdy, r_valid,
    input  wire [255:0] rd_data,
    // mem_arb write port (W2)
    output wire         w_req, w_en,
    output wire [28:0]  w_addr,
    output wire [255:0] w_data,
    output wire [31:0]  w_mask,
    output wire         w_wren, w_wend,
    input  wire         w_rdy, w_wdf_rdy
);
    localparam [1:0] OP_COPY = 2'd0, OP_FILL = 2'd1, OP_READ = 2'd2;
    localparam [7:0] TF_RAW = 8'h00, TF_RGBA = 8'h01, TF_BGRA = 8'h02, TF_BGRX = 8'h03, TF_565 = 8'h04,
                     TF_RGB = 8'h0A, TF_BGR = 8'h0B;
    localparam int   RBA = RB_LOG2 + 1;           // row buffer address bits (half + index)

    // exact 5 / 6 -> 8 bit expansion: round(c * 255 / 31), round(c * 255 / 63) (ES 2.0 §4.3.1; no ties exist)
    localparam logic [7:0] E5 [0:31] = '{8'd0, 8'd8, 8'd16, 8'd25, 8'd33, 8'd41, 8'd49, 8'd58, 8'd66, 8'd74, 8'd82,
        8'd90, 8'd99, 8'd107, 8'd115, 8'd123, 8'd132, 8'd140, 8'd148, 8'd156, 8'd165, 8'd173, 8'd181, 8'd189, 8'd197,
        8'd206, 8'd214, 8'd222, 8'd230, 8'd239, 8'd247, 8'd255};
    localparam logic [7:0] E6 [0:63] = '{8'd0, 8'd4, 8'd8, 8'd12, 8'd16, 8'd20, 8'd24, 8'd28, 8'd32, 8'd36, 8'd40,
        8'd45, 8'd49, 8'd53, 8'd57, 8'd61, 8'd65, 8'd69, 8'd73, 8'd77, 8'd81, 8'd85, 8'd89, 8'd93, 8'd97, 8'd101,
        8'd105, 8'd109, 8'd113, 8'd117, 8'd121, 8'd125, 8'd130, 8'd134, 8'd138, 8'd142, 8'd146, 8'd150, 8'd154, 8'd158,
        8'd162, 8'd166, 8'd170, 8'd174, 8'd178, 8'd182, 8'd186, 8'd190, 8'd194, 8'd198, 8'd202, 8'd206, 8'd210, 8'd215,
        8'd219, 8'd223, 8'd227, 8'd231, 8'd235, 8'd239, 8'd243, 8'd247, 8'd251, 8'd255};
    // 8 -> 5 / 6 bits, round to nearest (Blinn's exact division, as img_write)
    function automatic [4:0] r5(input [7:0] v);
        reg [13:0] t;
        t = ({1'b0, v, 5'b0} - {6'b0, v}) + 14'd128; r5 = 5'((t + (t >> 8)) >> 8);
    endfunction
    function automatic [5:0] r6(input [7:0] v);
        reg [14:0] t;
        t = ({1'b0, v, 6'b0} - {7'b0, v}) + 15'd128; r6 = 6'((t + (t >> 8)) >> 8);
    endfunction
    function automatic [1:0] bl_of(input [1:0] f);           // log2 bytes per pixel
        bl_of = (f == 2'd0) ? 2'd2 : (f == 2'd1) ? 2'd1 : 2'd0;
    endfunction
    // stored value -> {r, g, b, a}
    function automatic [31:0] st_rgba(input [1:0] f, input [31:0] v);
        st_rgba = (f == 2'd1) ? {E5[v[15:11]], E6[v[10:5]], E5[v[4:0]], 8'hFF} : v;
    endfunction
    // {r, g, b, a} -> stored value (RGBA8 / RGB565)
    function automatic [31:0] rgba_st(input [1:0] f, input [31:0] c);
        rgba_st = (f == 2'd1) ? {16'h0, r5(c[31:24]), r6(c[23:16]), r5(c[15:8])} : c;
    endfunction
    // transfer bytes (byte 0 in [7:0]) -> {r, g, b, a}
    function automatic [31:0] tf_rgba(input [7:0] tf, input [31:0] b);
        case (tf)
            TF_RGBA: tf_rgba = {b[7:0], b[15:8], b[23:16], b[31:24]};
            TF_BGRA: tf_rgba = {b[23:16], b[15:8], b[7:0], b[31:24]};
            TF_BGRX: tf_rgba = {b[23:16], b[15:8], b[7:0], 8'hFF};
            TF_RGB:  tf_rgba = {b[7:0], b[15:8], b[23:16], 8'hFF};
            TF_BGR:  tf_rgba = {b[23:16], b[15:8], b[7:0], 8'hFF};
            default: tf_rgba = {E5[b[15:11]], E6[b[10:5]], E5[b[4:0]], 8'hFF};    // TF_565
        endcase
    endfunction
    // {r, g, b, a} (and the raw value for RAW) -> transfer bytes
    function automatic [31:0] rgba_tf(input [7:0] tf, input [31:0] c, input [31:0] raw);
        case (tf)
            TF_RAW:  rgba_tf = raw;
            TF_RGBA: rgba_tf = {c[7:0], c[15:8], c[23:16], c[31:24]};
            TF_BGRA: rgba_tf = {c[7:0], c[31:24], c[23:16], c[15:8]};
            TF_BGRX: rgba_tf = {8'hFF, c[31:24], c[23:16], c[15:8]};
            TF_RGB:  rgba_tf = {8'h00, c[15:8], c[23:16], c[31:24]};
            TF_BGR:  rgba_tf = {8'h00, c[31:24], c[23:16], c[15:8]};
            default: rgba_tf = {16'h0, r5(c[31:24]), r6(c[23:16]), r5(c[15:8])};   // TF_565
        endcase
    endfunction

    // ---------------- command registers ----------------
    reg         run;                              // a command is being executed
    reg  [1:0]  op, sf, df;
    reg  [29:0] s_row0, d_row0;
    reg  [15:0] s_pitch, d_pitch;
    reg  [12:0] sx, sy, sw, sh, dx, dy, dw, dh;
    reg  [12:0] hq, hu0, vq, vu0;
    reg  [13:0] hr2, ha0, vr2, va0;
    reg         mx, my, bup, key_en, rmw;
    reg  [31:0] klo, khi, fill_raw;
    reg  [3:0]  fill_be;
    reg  [15:0] m565;
    reg  [7:0]  tfmt;
    reg  [2:0]  tbpp;
    reg  [12:0] j0, i0;
    reg  [23:0] npix;
    wire [1:0]  sbl = bl_of(sf), dbl = bl_of(df);
    wire [1:0]  rbl = (op == OP_FILL) ? dbl : sbl;    // pixel size of the rows the reader reads
    wire [1:0]  rf  = (op == OP_FILL) ? df : sf;
    wire        start, fin;                       // command intake / end (bottom of the file)
    reg         rb_last_q;                        // the row buffer write in flight is a job's last pixel
    reg         wj_hold;                          // one cycle after a write job ends
    assign cmd_ready = !run;

    // ---------------- job FIFOs ----------------
    // read job:  {half, start byte address, pixels}
    // write job: {last, nohalf, keep, half, row address incl. the rectangle's x, first buffer index, pixels}
    reg         rj_wen, wj_wen;
    reg  [43:0] rj_wd;
    reg  [59:0] wj_wd;
    wire        rj_v, wj_v, rj_pop, wj_pop; wire [43:0] rj_d; wire [59:0] wj_d; wire [3:0] rj_cnt, wj_cnt;
    sfifo #(.W(44), .AW(2), .RAM_STYLE("distributed")) u_rj (.clk(clk), .rst(rst), .wen(rj_wen), .wdata(rj_wd),
        .full(), .count(rj_cnt), .ren(rj_pop), .rdata(rj_d), .rvalid(rj_v));
    sfifo #(.W(60), .AW(2), .RAM_STYLE("distributed")) u_wj (.clk(clk), .rst(rst), .wen(wj_wen), .wdata(wj_wd),
        .full(), .count(wj_cnt), .ren(wj_pop), .rdata(wj_d), .rvalid(wj_v));

    // ---------------- sequencer ----------------
    typedef enum logic [2:0] {Q_IDLE, Q_ROW, Q_MUL, Q_ADD, Q_PUSH, Q_DONE} qst_t;
    qst_t       qs;
    reg  [12:0] k, nrows;
    reg  [12:0] vcur, vnxt, vprev;
    reg  [13:0] vacc;
    reg         have_prev, hsel;
    reg  [23:0] remain;
    reg  [12:0] q_srow, q_drow, q_start, q_cnt;
    reg         q_read, q_keep, q_last;
    reg  [28:0] q_ps, q_pd;
    reg  [29:0] q_raddr, q_waddr;
    wire [14:0] vsum = {1'b0, vacc} + {1'b0, vr2};
    wire        vcar = vsum >= {1'b0, dh, 1'b0};
    wire [12:0] v_eff = bup ? (dh - 13'd1 - k) : my ? (sh - 13'd1 - vcur) : vcur;   // source row offset of row k
    wire        room = (rj_cnt < 4'd3) && (wj_cnt < 4'd3);

    always @(posedge clk) begin
        rj_wen <= 1'b0; wj_wen <= 1'b0;
        if (rst) qs <= Q_IDLE;
        else case (qs)
            Q_IDLE: ;                                               // started by the command intake below
            Q_ROW: begin
                q_last <= 1'b0; q_keep <= 1'b0; q_start <= 13'd0;
                case (op)
                    OP_COPY: begin
                        q_srow <= sy + v_eff; q_drow <= dy + (bup ? (dh - 13'd1 - k) : k);
                        q_read <= !have_prev || v_eff != vprev;
                        q_keep <= !bup && (k + 13'd1 < nrows) && (vnxt == vcur);
                        q_cnt <= dw; q_last <= (k + 13'd1 == nrows);
                    end
                    OP_FILL: begin
                        q_srow <= dy + k; q_drow <= dy + k; q_read <= rmw; q_cnt <= dw; q_last <= (k + 13'd1 == nrows);
                    end
                    default: begin                                  // READ
                        q_srow <= sy + j0 + k; q_drow <= 13'd0; q_read <= 1'b1;
                        q_start <= (k == 0) ? i0 : 13'd0;
                        if ({11'h0, sw - ((k == 0) ? i0 : 13'd0)} >= remain) begin
                            q_cnt <= remain[12:0]; q_last <= 1'b1;
                        end else q_cnt <= sw - ((k == 0) ? i0 : 13'd0);
                    end
                endcase
                qs <= Q_MUL;
            end
            Q_MUL: begin
                q_ps <= q_srow * ((op == OP_FILL) ? d_pitch : s_pitch);
                q_pd <= q_drow * d_pitch;
                qs <= Q_ADD;
            end
            Q_ADD: begin
                q_raddr <= ((op == OP_FILL) ? d_row0 + {1'b0, q_ps} + ({17'h0, dx} << dbl)
                                            : s_row0 + {1'b0, q_ps} + ({17'h0, sx} << sbl));
                q_waddr <= d_row0 + {1'b0, q_pd} + ({17'h0, dx} << dbl);
                qs <= Q_PUSH;
            end
            Q_PUSH: if (room) begin
                if (q_read) begin
                    rj_wen <= 1'b1; rj_wd <= {!hsel, q_raddr, (op == OP_FILL) ? dw : sw};
                    hsel <= !hsel;
                end
                wj_wen <= 1'b1;
                wj_wd <= {q_last, (op == OP_FILL && !rmw), q_keep, q_read ? !hsel : hsel, q_waddr, q_start, q_cnt};
                have_prev <= 1'b1; vprev <= v_eff;
                if (op == OP_READ) remain <= remain - {11'h0, q_cnt};
                // next row: advance the vertical DDA (vcur = v(k + 1), vnxt = v(k + 2))
                vcur <= vnxt;
                vnxt <= vnxt + vq + {12'h0, vcar};
                vacc <= vcar ? 14'(vsum - {1'b0, dh, 1'b0}) : vsum[13:0];
                k <= k + 13'd1;
                qs <= q_last ? Q_DONE : Q_ROW;
            end
            Q_DONE: ;                                               // left by the command end
            default: qs <= Q_IDLE;
        endcase
        if (start) begin
            k <= 13'd0; have_prev <= 1'b0; hsel <= 1'b1;               // the first read job takes half 0
            remain <= cmd_npix;
            nrows <= (cmd_op == OP_READ) ? 13'd0 : cmd_d_h;
            // v(0) = vu0 with accumulator va0; vnxt = v(1)
            vcur <= cmd_vu0;
            begin : v1
                reg [14:0] s1; s1 = {1'b0, cmd_va0} + {1'b0, cmd_vr2};
                if (s1 >= {1'b0, cmd_d_h, 1'b0}) begin vnxt <= cmd_vu0 + cmd_vq + 13'd1; vacc <= 14'(s1 - {1'b0, cmd_d_h, 1'b0}); end
                else begin vnxt <= cmd_vu0 + cmd_vq; vacc <= s1[13:0]; end
            end
            qs <= Q_ROW;
        end
        if (fin) qs <= Q_IDLE;
    end

    // ---------------- row buffer ----------------
    (* ram_style = "block" *) reg [32:0] rb [0:(1 << RBA) - 1];
    reg         rb_we; reg [RBA-1:0] rb_wa; reg [32:0] rb_wd;
    reg  [32:0] rb_q;
    reg  [1:0]  half_full;
    wire        rb_re;
    wire [RBA-1:0] rb_ra;
    always @(posedge clk) begin
        if (rb_we) rb[rb_wa] <= rb_wd;
        if (rb_re) rb_q <= rb[rb_ra];
    end

    // ---------------- reader ----------------
    reg         rd_act;                           // a job: issuing and / or unpacking
    reg         rd_half;
    reg  [24:0] rd_bi;                            // next burst to issue
    reg  [9:0]  rd_left;                          // bursts still to issue
    reg  [12:0] up_left;                          // pixels still to unpack
    reg  [4:0]  up_p;                             // pixel index inside the head burst
    reg  [12:0] up_idx;                           // buffer index of the next pixel
    reg  [7:0]  n_iss, n_pop;
    wire [7:0]  infl = n_iss - n_pop;
    assign r_en   = rd_act && rd_left != 0 && infl < 8'd16;
    assign r_addr = {1'b0, rd_bi, 3'b000};
    wire        iss = r_en && r_rdy;
    wire        df_v; wire [255:0] df_d; wire df_pop;
    sfifo #(.W(256), .AW(4), .RAM_STYLE(`SHALLOW_RAM)) u_df (.clk(clk), .rst(rst), .wen(r_valid), .wdata(rd_data),  // 172 LUT as LUTRAM; same cycles
        .full(), .count(), .ren(df_pop), .rdata(df_d), .rvalid(df_v));
    wire [4:0]  ppb_m1 = (rbl == 2'd2) ? 5'd7 : (rbl == 2'd1) ? 5'd15 : 5'd31;
    wire        up_emit = rd_act && up_left != 0 && df_v;
    assign df_pop = up_emit && (up_p == ppb_m1 || up_left == 13'd1);
    // job start: the half must be free
    wire        rj_take = run && !rd_act && rj_v && !half_full[rj_d[43]];
    assign rj_pop = rj_take;
    wire [29:0] rj_s  = rj_d[42:13];
    wire [12:0] rj_n  = rj_d[12:0];
    wire [29:0] rj_e1 = rj_s + ({17'h0, rj_n} << rbl) - 30'd1;
    // unpack stage K0 (pixel extracted) and K1 (key compared, written)
    reg         k0_v, k0_last, k0_half;
    reg  [31:0] k0_raw;
    reg  [RB_LOG2-1:0] k0_idx;
    function automatic bit in_rng(input [7:0] v, input [7:0] lo, input [7:0] hi);
        in_rng = (v >= lo) && (v <= hi);
    endfunction
    wire        k0_key = key_en && (
                    (rf == 2'd0) ? (in_rng(k0_raw[31:24], klo[31:24], khi[31:24]) && in_rng(k0_raw[23:16], klo[23:16], khi[23:16]) &&
                                    in_rng(k0_raw[15:8], klo[15:8], khi[15:8])) :
                    (rf == 2'd1) ? (in_rng({3'b0, k0_raw[15:11]}, {3'b0, klo[15:11]}, {3'b0, khi[15:11]}) &&
                                    in_rng({2'b0, k0_raw[10:5]}, {2'b0, klo[10:5]}, {2'b0, khi[10:5]}) &&
                                    in_rng({3'b0, k0_raw[4:0]}, {3'b0, klo[4:0]}, {3'b0, khi[4:0]})) :
                                   in_rng(k0_raw[7:0], klo[7:0], khi[7:0]));
    reg         rel_v; reg rel_half;              // the writer releases a half (from below)
    always @(posedge clk) begin
        rb_we <= 1'b0; k0_v <= 1'b0;
        if (rst || !run) begin
            rd_act <= 1'b0; n_iss <= 0; n_pop <= 0; half_full <= 2'b00;
        end else begin
            if (iss) begin rd_bi <= rd_bi + 25'd1; rd_left <= rd_left - 10'd1; n_iss <= n_iss + 8'd1; end
            if (df_pop) n_pop <= n_pop + 8'd1;
            if (rj_take) begin
                rd_act <= 1'b1; rd_half <= rj_d[43]; rd_bi <= rj_s[29:5];
                rd_left <= 10'(rj_e1[29:5] - rj_s[29:5]) + 10'd1;
                up_left <= rj_n; up_idx <= 13'd0;
                up_p <= (rbl == 2'd2) ? {2'b00, rj_s[4:2]} : (rbl == 2'd1) ? {1'b0, rj_s[4:1]} : rj_s[4:0];
            end
            if (up_emit) begin
                k0_v <= 1'b1; k0_half <= rd_half; k0_idx <= up_idx[RB_LOG2-1:0]; k0_last <= (up_left == 13'd1);
                case (rbl)
                    2'd2:    k0_raw <= df_d[up_p[2:0] * 32 +: 32];
                    2'd1:    k0_raw <= {16'h0, df_d[up_p[3:0] * 16 +: 16]};
                    default: k0_raw <= {24'h0, df_d[up_p * 8 +: 8]};
                endcase
                up_left <= up_left - 13'd1; up_idx <= up_idx + 13'd1;
                up_p <= (up_p == ppb_m1) ? 5'd0 : up_p + 5'd1;
                if (up_left == 13'd1) rd_act <= 1'b0;                // the issue side is done by then
            end
            if (k0_v) begin
                rb_we <= 1'b1; rb_wa <= {k0_half, k0_idx}; rb_wd <= {k0_key, k0_raw};
            end
            // the half is complete once its last pixel is written (rb write lands the cycle after)
            if (rb_we && rb_last_q) half_full[rb_wa[RBA-1]] <= 1'b1;
            if (rel_v) half_full[rel_half] <= 1'b0;
        end
    end
    always @(posedge clk) rb_last_q <= k0_v && k0_last;

    // ---------------- writer: stage A (index / address), B (buffer read), C, D (decode), E (encode) ----------------
    wire        go;
    reg         wr_act, wr_half, wr_keep, wr_noh, wr_last;
    reg  [29:0] wr_addr;
    reg  [12:0] wr_i, wr_cnt, wr_start, wr_u;
    reg  [13:0] wr_acc;
    wire [14:0] hsum = {1'b0, wr_acc} + {1'b0, hr2};
    wire        hcar = hsum >= {1'b0, dw, 1'b0};
    // write job fields: [59] last, [58] no half (plain FILL), [57] keep the half, [56] half, [55:26] row address,
    // [25:13] first buffer index, [12:0] pixels
    wire        wj_take = run && !wr_act && wj_v && (wj_d[58] || half_full[wj_d[56]]) && !wj_hold;
    assign wj_pop = wj_take;
    wire        a_v   = wr_act;
    wire [12:0] a_idx = (op == OP_COPY) ? (mx ? (sw - 13'd1 - wr_u) : wr_u) : (wr_start + wr_i);
    wire        a_end = (wr_i == wr_cnt - 13'd1);
    assign rb_re = go && a_v;
    assign rb_ra = {wr_half, a_idx[RB_LOG2-1:0]};
    reg         b_v, b_last, c_v, c_last, d_v, d_last, e_v, e_last;
    reg  [29:0] b_addr, c_addr, d_addr, e_addr;
    reg  [32:0] c_q;
    reg  [31:0] d_c, d_raw, e_val;
    reg         d_key;
    reg  [3:0]  e_be;
    always @(posedge clk) begin
        rel_v <= 1'b0; wj_hold <= 1'b0;
        if (rst || !run) begin
            wr_act <= 1'b0; b_v <= 1'b0; c_v <= 1'b0; d_v <= 1'b0; e_v <= 1'b0;
        end else begin
            if (wj_take) begin
                wr_act <= 1'b1; wr_last <= wj_d[59]; wr_noh <= wj_d[58]; wr_keep <= wj_d[57]; wr_half <= wj_d[56];
                wr_addr <= wj_d[55:26]; wr_start <= wj_d[25:13]; wr_cnt <= wj_d[12:0];
                wr_i <= 13'd0; wr_u <= hu0; wr_acc <= ha0;
            end
            if (go) begin
                // A -> B
                b_v <= a_v; b_addr <= wr_addr + ({17'h0, wr_i} << dbl); b_last <= a_v && a_end && wr_last;
                if (a_v) begin
                    wr_i <= wr_i + 13'd1;
                    wr_u <= wr_u + hq + {12'h0, hcar};
                    wr_acc <= hcar ? 14'(hsum - {1'b0, dw, 1'b0}) : hsum[13:0];
                    if (a_end) begin
                        wr_act <= 1'b0; wj_hold <= 1'b1;
                        if (!wr_keep && !wr_noh) begin rel_v <= 1'b1; rel_half <= wr_half; end
                    end
                end
                // B -> C (buffer data registered)
                c_v <= b_v; c_addr <= b_addr; c_last <= b_last; c_q <= rb_q;
                // C -> D: decode the stored value
                d_v <= c_v; d_addr <= c_addr; d_last <= c_last; d_raw <= c_q[31:0]; d_key <= c_q[32];
                d_c <= st_rgba((op == OP_FILL) ? df : sf, c_q[31:0]);
                // D -> E: encode
                e_v <= d_v; e_addr <= d_addr; e_last <= d_last;
                case (op)
                    OP_COPY: begin
                        e_val <= (df == 2'd2) ? {24'h0, d_raw[7:0]} : rgba_st(df, d_c);
                        e_be  <= d_key ? 4'h0 : (df == 2'd0) ? 4'hF : (df == 2'd1) ? 4'h3 : 4'h1;
                    end
                    OP_FILL: begin
                        e_val <= rmw ? ((d_raw & ~{16'h0, m565}) | (fill_raw & {16'h0, m565})) : fill_raw;
                        e_be  <= rmw ? 4'h3 : fill_be;
                    end
                    default: begin e_val <= rgba_tf(tfmt, d_c, d_raw); e_be <= 4'h0; end
                endcase
            end
        end
    end

    // ---------------- stage F: burst assembly (COPY / FILL) ----------------
    reg          bf_wen; reg [312:0] bf_wd;
    wire         bf_v; wire [312:0] bf_d; wire [5:0] bf_cnt; wire bf_pop;
    sfifo #(.W(313), .AW(4), .RAM_STYLE(`SHALLOW_RAM)) u_bf (.clk(clk), .rst(rst), .wen(bf_wen), .wdata(bf_wd),  // 210 LUT as LUTRAM; same cycles
        .full(), .count(bf_cnt), .ren(bf_pop), .rdata(bf_d), .rvalid(bf_v));
    reg  [255:0] bur_d;
    reg  [31:0]  bur_be;
    reg  [24:0]  bur_i;
    reg          bur_have, fin_pend;
    wire [24:0]  e_bi  = e_addr[29:5];
    wire [4:0]   e_o   = e_addr[4:0];
    wire [31:0]  e_be32 = {28'h0, e_be} << e_o;
    reg  [255:0] e_bm;                            // byte mask of the pixel as bits
    integer      bq;
    always @(*) for (bq = 0; bq < 32; bq = bq + 1) e_bm[bq*8 +: 8] = {8{e_be32[bq]}};
    wire [255:0] e_dv  = ({224'h0, e_val} << {e_o, 3'b000}) & e_bm;
    wire         e_new = bur_have && e_bi != bur_i;
    wire [255:0] m_d   = (bur_d & ~e_bm) | e_dv;
    wire [31:0]  m_be  = bur_be | e_be32;
    // issue: data beat first, then its command (as img_write)
    reg  [31:0]  wr_dat, wr_cmd, n_push;
    reg  [24:0]  cmd_bi;
    wire         data_turn = (wr_dat == wr_cmd);
    assign w_wren = bf_v && data_turn;
    assign w_wend = w_wren;
    assign w_data = bf_d[255:0];
    assign w_mask = bf_d[287:256];
    assign bf_pop = w_wren && w_wdf_rdy;
    assign w_en   = !data_turn;
    assign w_addr = {1'b0, cmd_bi, 3'b000};
    assign w_req  = bf_v || !data_turn;

    // ---------------- stage F': byte packer (READ) -> output FIFO ----------------
    reg          of_wen; reg [31:0] of_wd;
    wire [6:0]   of_cnt;
    sfifo #(.W(32), .AW(5), .RAM_STYLE("distributed")) u_of (.clk(clk), .rst(rst), .wen(of_wen), .wdata(of_wd),
        .full(), .count(of_cnt), .ren(o_valid && o_ready), .rdata(o_data), .rvalid(o_valid));
    reg  [63:0]  pk_b;
    reg  [3:0]   pk_n;
    wire [3:0]   pk_n2  = pk_n + {1'b0, tbpp};
    wire [63:0]  pk_b2  = pk_b | ({32'h0, e_val} << (8 * pk_n));
    assign go = (bf_cnt < 6'd11) && (of_cnt < 7'd26) && !fin_pend;

    always @(posedge clk) begin
        bf_wen <= 1'b0; of_wen <= 1'b0;
        if (rst || !run) begin
            bur_have <= 1'b0; bur_be <= 0; fin_pend <= 1'b0; pk_n <= 0; pk_b <= 0;
            if (rst) begin wr_dat <= 0; wr_cmd <= 0; n_push <= 0; c_pixels <= 0; end
        end else begin
            if (bf_pop) begin wr_dat <= wr_dat + 1; cmd_bi <= bf_d[312:288]; end
            if (w_en && w_rdy) wr_cmd <= wr_cmd + 1;
            if (fin_pend) begin                       // a final burst / word left over from the last pixel
                if (op == OP_READ) begin
                    of_wen <= 1'b1; of_wd <= pk_b[31:0]; pk_n <= 0;
                end else if (bur_have) begin
                    if (bur_be != 0) begin bf_wen <= 1'b1; bf_wd <= {bur_i, ~bur_be, bur_d}; n_push <= n_push + 1; end
                    bur_have <= 1'b0;
                end
                fin_pend <= 1'b0;
            end else if (go && e_v) begin
                c_pixels <= c_pixels + 1;
                if (op == OP_READ) begin
                    if (pk_n2 >= 4'd4) begin
                        of_wen <= 1'b1; of_wd <= pk_b2[31:0];
                        pk_b <= pk_b2 >> 32; pk_n <= pk_n2 - 4'd4;
                        if (e_last && pk_n2 > 4'd4) fin_pend <= 1'b1;
                    end else begin
                        pk_b <= pk_b2; pk_n <= pk_n2;
                        if (e_last) begin of_wen <= 1'b1; of_wd <= pk_b2[31:0]; pk_n <= 0; end
                    end
                end else begin
                    bur_i <= e_bi;
                    if (e_new) begin
                        if (bur_be != 0) begin bf_wen <= 1'b1; bf_wd <= {bur_i, ~bur_be, bur_d}; n_push <= n_push + 1; end
                        bur_d <= e_dv; bur_be <= e_be32; bur_have <= 1'b1;
                        if (e_last) fin_pend <= 1'b1;
                    end else if (e_last) begin
                        if (m_be != 0) begin bf_wen <= 1'b1; bf_wd <= {e_bi, ~m_be, m_d}; n_push <= n_push + 1; end
                        bur_have <= 1'b0; bur_be <= 0;
                    end else begin
                        bur_d <= m_d; bur_be <= m_be; bur_have <= 1'b1;
                    end
                end
            end
        end
    end

    // ---------------- command intake and end ----------------
    reg  last_seen;                               // the last pixel has left stage F
    assign start = cmd_valid && !run;
    // FIFO counts include words still in the memory behind the output register (a fresh write is not yet rvalid)
    wire drained = (qs == Q_DONE) && rj_cnt == 0 && !rd_act && wj_cnt == 0 && !wr_act && !b_v && !c_v && !d_v && !e_v &&
                   !fin_pend && !bur_have && bf_cnt == 0 && !bf_wen && wr_dat == n_push && wr_cmd == n_push &&
                   !of_wen && of_cnt == 0 && last_seen;
    assign fin = run && drained;
    assign busy = run;
    always @(posedge clk) begin
        if (rst) begin run <= 1'b0; c_ops <= 0; last_seen <= 1'b0; end
        else begin
            if (go && e_v && e_last) last_seen <= 1'b1;
            if (fin) begin run <= 1'b0; c_ops <= c_ops + 1; end
            if (start) begin
                run <= 1'b1; last_seen <= 1'b0;
                op <= cmd_op; sf <= cmd_s_fmt; df <= cmd_d_fmt;
                s_row0 <= cmd_s_row0; s_pitch <= cmd_s_pitch; d_row0 <= cmd_d_row0; d_pitch <= cmd_d_pitch;
                sx <= cmd_s_x; sy <= cmd_s_y; sw <= cmd_s_w; sh <= cmd_s_h;
                dx <= cmd_d_x; dy <= cmd_d_y; dw <= cmd_d_w; dh <= cmd_d_h;
                hq <= cmd_hq; hu0 <= cmd_hu0; vq <= cmd_vq; vu0 <= cmd_vu0;
                hr2 <= cmd_hr2; ha0 <= cmd_ha0; vr2 <= cmd_vr2; va0 <= cmd_va0;
                mx <= cmd_mir_x; my <= cmd_mir_y; bup <= cmd_bottom_up; key_en <= cmd_key && cmd_op == OP_COPY;
                klo <= cmd_key_lo; khi <= cmd_key_hi; rmw <= cmd_rmw && cmd_op == OP_FILL;
                tfmt <= cmd_tfmt; j0 <= cmd_j0; i0 <= cmd_i0; npix <= cmd_npix;
                case (cmd_tfmt)
                    TF_RAW:         tbpp <= (cmd_s_fmt == 2'd0) ? 3'd4 : (cmd_s_fmt == 2'd1) ? 3'd2 : 3'd1;
                    TF_565:         tbpp <= 3'd2;
                    TF_RGB, TF_BGR: tbpp <= 3'd3;
                    default:        tbpp <= 3'd4;
                endcase
                // FILL: the value in the destination's storage format, byte enables / RGB565 merge mask
                fill_raw <= (cmd_tfmt == TF_RAW) ? cmd_value : rgba_st(cmd_d_fmt, tf_rgba(cmd_tfmt, cmd_value));
                case (cmd_d_fmt)
                    2'd0:    fill_be <= {cmd_mask[0], cmd_mask[1], cmd_mask[2], cmd_mask[3]};   // bytes A, B, G, R
                    2'd1:    fill_be <= (cmd_mask[2:0] == 3'b111) ? 4'h3 : 4'h0;
                    default: fill_be <= (cmd_mask != 4'h0) ? 4'h1 : 4'h0;
                endcase
                m565 <= {{5{cmd_mask[0]}}, {6{cmd_mask[1]}}, {5{cmd_mask[2]}}};
            end
        end
    end
    wire _unused = &{1'b0, npix, rbl, rj_cnt, wj_cnt, j0};
endmodule
`default_nettype wire
