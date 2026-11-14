// scan_reader — scanout reader of board B: streams the presented image from DDR (one mem_arb read port, MUST_RUN) into
// display_line_buffer, one display line per slot, 8 pixel words per slot burst.
//   * image = base (32-byte aligned), pitch (bytes, a multiple of the pixel size), width, height, format
//     (pres_fmt) 0 RGBA8, 1 RGB565 (expanded to RGBA8 by bit replication: display only),
//     2 P8 (palette index), origin: 0 = rows top first, 1 = rows bottom first (GL), i.e. display line y shows
//     memory row height-1-y;
//   * line-buffer words: direct colour 0xRRGGBBFF (alpha forced to 0xFF, black fill 0x000000FF); P8
//     {index, 16'h0, 7'h0, bank}: alpha 0x00 / 0x01 = palette bank, taken from pal_bank at each frame start
//     (frame_start pulse), so a frame never mixes two palettes (the pixel domain looks the index up);
//   * an image smaller than the mode is shown at the top-left with black around it, a larger one shows its
//     top-left window; before the first present the screen is black;
//   * rows may start anywhere inside a 32-byte burst: a line is fetched as whole bursts and realigned by the
//     pixel offset of its first pixel;
//   * a present takes effect at the next frame start (vsync) or, with pres_now, at the next line the reader
//     starts (tearing allowed). A newer present replaces a pending one (counted in c_dropped).
// Structure: the issue walker computes every display line (row address by multiply, burst range, offset),
// pushes a line record and issues the line's reads; the drain walker pops a record, turns the returned
// bursts into 8-pixel units (RGB565: two per burst), realigns them, blanks the pixels past the image width,
// fills black up to the mode width and releases the slot. Issued-but-undrained bursts never exceed the
// in-flight FIFO (2**IB_AW), so returned data always has room. A frame restart (vsync or run start) flushes
// both FIFOs and discards the reads still outstanding.
// SHALLOW_RAM: the style of the shallow, wide burst FIFO (u_ib) — block RAM, or LUTs in the one-board build (ONE_CHIP),
// where block RAM is the scarcer resource; the behaviour is the same
`ifndef SHALLOW_RAM
`ifdef ONE_CHIP
`define SHALLOW_RAM "distributed"
`else
`define SHALLOW_RAM "block"
`endif
`endif
`default_nettype none
module scan_reader #(
    parameter int IB_AW = 4,                 // in-flight bursts: 2**IB_AW
    parameter int LR_AW = 5                  // line records: 2**LR_AW (+1)
)(
    input  wire         clk, rst,
    input  wire         run,                 // 0: stop issuing, let the outstanding reads return, then idle
    output wire         idle,
    input  wire         frame_sync,          // vsync pulse (clk domain): restart at display line 0
    input  wire [10:0]  mode_w,              // active width: multiple of 8, <= 1280 (stable while running)
    input  wire [10:0]  mode_h,
    // present
    input  wire         pres_valid,
    input  wire [29:0]  pres_base,
    input  wire [15:0]  pres_pitch,
    input  wire [12:0]  pres_w, pres_h,
    input  wire [1:0]   pres_fmt,
    input  wire         pres_origin, pres_now,
    input  wire [31:0]  pres_tag,
    input  wire         pal_bank,            // palette bank for the frames that start from now on
    output wire         frame_start,         // pulse: a frame (re)starts; pal_bank is taken
    output reg          pres_done,           // pulse: a present became current
    output reg          cur_valid,
    output reg  [31:0]  cur_tag,
    output reg  [31:0]  c_dropped,
    // DDR read port (mem_arb contract: hold r_en/r_addr until r_rdy; data returns in order on r_valid)
    output wire         r_en,
    output wire [28:0]  r_addr,
    input  wire         r_rdy, r_valid,
    input  wire [255:0] rd_data,
    // line buffer write port (display_line_buffer)
    output reg          lb_en,
    output reg  [7:0]   lb_idx,
    output reg  [255:0] lb_data,
    output reg          lb_line_done,
    output reg          lb_frame_start,      // pulse: reset the line buffer's write side
    input  wire         lb_avail
);
    localparam int IBN = 1 << IB_AW;

    // RGB565 -> 0xRRGGBBAA by bit replication (as the TMU)
    function automatic [31:0] exp565(input [15:0] c);
        exp565 = {c[15:11], c[15:13], c[10:5], c[10:9], c[4:0], c[4:2], 8'hFF};
    endfunction
    function automatic [255:0] unpack565(input [255:0] b, input hi);
        integer q;
        for (q = 0; q < 8; q = q + 1) unpack565[q*32 +: 32] = exp565(b[(hi ? 128 : 0) + q*16 +: 16]);
    endfunction
    // P8: 8 indices of quarter qd -> {index, 0, 0, bank}
    function automatic [255:0] unpack8(input [255:0] b, input [1:0] qd, input bank);
        integer q;
        for (q = 0; q < 8; q = q + 1) unpack8[q*32 +: 32] = {b[qd*64 + q*8 +: 8], 16'h0, 7'h0, bank};
    endfunction
    // RGBA8: alpha forced to 0xFF (the alpha byte tags palette pixels)
    function automatic [255:0] opaque(input [255:0] b);
        integer q;
        for (q = 0; q < 8; q = q + 1) opaque[q*32 +: 32] = {b[q*32 + 8 +: 24], 8'hFF};
    endfunction
    localparam [255:0] BLACK8 = {8{32'h000000FF}};

    // ---------------- current / pending image ----------------
    reg        pv, p_now, p_org;
    reg [1:0]  p_fmt;
    reg [29:0] p_base; reg [15:0] p_pitch; reg [12:0] p_w, p_h; reg [31:0] p_tag;
    reg        c_org;
    reg [1:0]  c_fmt;
    reg [29:0] c_base; reg [15:0] c_pitch; reg [12:0] c_w, c_h;
    reg        f_pal;                        // palette bank of the frame being read

    // ---------------- issue walker ----------------
    typedef enum logic [2:0] {I_STOP, I_LINE, I_C1, I_C2, I_C3, I_C4, I_PUSH, I_ISSUE} ist_t;
    ist_t       ist;
    reg         run_q;
    reg  [10:0] iy;                          // display line being set up / issued
    reg         l_has;
    reg  [12:0] l_row, l_n;
    reg  [28:0] l_prod;
    reg  [29:0] l_s;
    reg  [14:0] l_nbytes;
    reg  [7:0]  l_k, l_nb;
    reg  [2:0]  l_lq;
    reg  [4:0]  l_off;                       // pixel offset of the row's first pixel inside its first burst
    reg  [24:0] i_b;                         // next burst index to issue
    reg  [7:0]  i_cnt;                       // bursts of the line still to issue
    reg  [7:0]  n_iss, n_pop;                // since the last restart (credit), wrap-safe
    reg  [7:0]  n_out, disc;                 // reads outstanding at the arbiter / of those, to discard

    wire restart    = run & (~run_q | frame_sync);
    assign frame_start = restart;
    wire line_start = (ist == I_LINE) & ~restart & (iy != mode_h);
    wire adopt      = pv & ((restart) | (line_start & p_now));
    wire [7:0] infl = n_iss - n_pop;
    assign r_en   = (ist == I_ISSUE) && (i_cnt != 8'd0) && (infl < 8'(IBN));
    assign r_addr = {1'b0, i_b, 3'b000};
    wire iss = r_en & r_rdy;

    // record FIFO: {fmt[1:0], off[4:0], nb[7:0], k[7:0], lq[2:0]}
    wire        lr_full, lr_v; wire [25:0] lr_d; reg lr_pop; wire [LR_AW+1:0] lr_cnt;
    wire        lr_push = (ist == I_PUSH) && !lr_full && !restart;
    sfifo #(.W(26), .AW(LR_AW), .RAM_STYLE("distributed")) u_lr (.clk(clk), .rst(rst | restart),
        .wen(lr_push), .wdata({c_fmt, l_off, l_nb, l_k, l_lq}), .full(lr_full), .count(lr_cnt),
        .ren(lr_pop), .rdata(lr_d), .rvalid(lr_v));

    // in-flight data FIFO
    wire        ib_v; wire [255:0] ib_d; reg ib_pop; wire [IB_AW+1:0] ib_cnt;
    wire        ib_push = r_valid && (disc == 8'd0) && !restart;
    sfifo #(.W(256), .AW(IB_AW), .RAM_STYLE(`SHALLOW_RAM)) u_ib (.clk(clk), .rst(rst | restart),  // 16 x 256 in LUTRAM: 172 LUT; same cycles
        .wen(ib_push), .wdata(rd_data), .full(), .count(ib_cnt), .ren(ib_pop), .rdata(ib_d), .rvalid(ib_v));

    assign idle = (ist == I_STOP) && (n_out == 8'd0);

    always @(posedge clk) begin
        pres_done <= 1'b0; lb_frame_start <= 1'b0;
        if (rst) begin
            ist <= I_STOP; run_q <= 1'b0; iy <= 0; pv <= 1'b0; p_now <= 1'b0; cur_valid <= 1'b0; cur_tag <= 0;
            c_dropped <= 0; n_iss <= 0; n_pop <= 0; n_out <= 0; disc <= 0; i_cnt <= 0;
            c_base <= 0; c_pitch <= 0; c_w <= 0; c_h <= 0; c_fmt <= 2'd0; c_org <= 1'b0; f_pal <= 1'b0;
        end else begin
            run_q <= run;
            if (restart) f_pal <= pal_bank;
            // ---- present intake / adoption ----
            if (adopt) begin
                cur_valid <= 1'b1; cur_tag <= p_tag; pres_done <= 1'b1;
                c_base <= p_base; c_pitch <= p_pitch; c_w <= p_w; c_h <= p_h; c_fmt <= p_fmt; c_org <= p_org;
            end
            if (pres_valid) begin
                pv <= 1'b1; p_now <= pres_now; p_base <= pres_base; p_pitch <= pres_pitch; p_w <= pres_w;
                p_h <= pres_h; p_fmt <= pres_fmt; p_org <= pres_origin; p_tag <= pres_tag;
                if (pv && !adopt) c_dropped <= c_dropped + 1;
            end else if (adopt) pv <= 1'b0;

            // ---- outstanding reads and discards ----
            n_out <= n_out + (iss ? 8'd1 : 8'd0) - (r_valid ? 8'd1 : 8'd0);
            if (restart) disc <= n_out + (iss ? 8'd1 : 8'd0) - (r_valid ? 8'd1 : 8'd0);
            else if (r_valid && disc != 8'd0) disc <= disc - 8'd1;
            if (restart) begin n_iss <= 0; n_pop <= 0; end
            else begin
                if (iss) n_iss <= n_iss + 8'd1;
                if (ib_pop) n_pop <= n_pop + 8'd1;
            end

            // ---- issue walker ----
            if (!run) ist <= I_STOP;
            else if (restart) begin ist <= I_LINE; iy <= 0; lb_frame_start <= 1'b1; i_cnt <= 0; end
            else case (ist)
                I_STOP:  ;                                     // left by restart (run rising)
                I_LINE:  if (iy != mode_h) ist <= I_C1;        // adopt (pres_now) happens this cycle
                I_C1: begin
                    l_has <= cur_valid && ({2'b00, iy} < c_h);
                    l_row <= c_org ? (c_h - 13'd1 - {2'b00, iy}) : {2'b00, iy};
                    l_n   <= (c_w < {2'b00, mode_w}) ? c_w : {2'b00, mode_w};
                    ist   <= I_C2;
                end
                I_C2: begin
                    l_prod   <= l_row * c_pitch;                // 13 x 16 bits (DSP)
                    l_nbytes <= (c_fmt == 2'd2) ? {2'b00, l_n} : (c_fmt == 2'd1) ? {1'b0, l_n, 1'b0} : {l_n, 2'b00};
                    l_k      <= l_has ? 8'((l_n + 13'd7) >> 3) : 8'd0;
                    l_lq     <= 3'(l_n - 13'd1);
                    ist      <= I_C3;
                end
                I_C3: begin l_s <= c_base + {1'b0, l_prod}; ist <= I_C4; end
                I_C4: begin
                    l_nb  <= l_has ? 8'(({11'h0, l_s[4:0]} + {1'b0, l_nbytes} + 16'd31) >> 5) : 8'd0;
                    l_off <= (c_fmt == 2'd2) ? l_s[4:0] : (c_fmt == 2'd1) ? {1'b0, l_s[4:1]} : {2'b00, l_s[4:2]};
                    ist   <= I_PUSH;
                end
                I_PUSH: if (!lr_full) begin
                    i_b <= l_s[29:5]; i_cnt <= l_nb;
                    if (l_nb != 8'd0) ist <= I_ISSUE;
                    else begin iy <= iy + 11'd1; ist <= I_LINE; end
                end
                I_ISSUE: if (iss) begin
                    i_b <= i_b + 25'd1; i_cnt <= i_cnt - 8'd1;
                    if (i_cnt == 8'd1) begin iy <= iy + 11'd1; ist <= I_LINE; end
                end
                default: ist <= I_STOP;
            endcase
        end
    end

    // ---------------- drain walker ----------------
    // A burst holds 1 (RGBA8), 2 (RGB565) or 4 (P8) units of 8 pixels; d_q is the unit inside the head burst.
    // The row's first pixel sits at unit off[4:3] (skipped units) and pixel off[2:0] of its first burst.
    reg         d_act, d_hp;
    reg  [1:0]  d_fmt, d_skip, d_q;
    reg  [2:0]  d_sh, d_lq;
    reg  [7:0]  d_nb, d_k, d_o;
    reg  [255:0] d_prev;
    wire [7:0]  d_u = mode_w[10:3];          // slot bursts per display line
    wire [1:0]  d_qlast = (d_fmt == 2'd2) ? 2'd3 : (d_fmt == 2'd1) ? 2'd1 : 2'd0;

    reg          out_v, consume, line_end;
    reg  [255:0] unit, out_d;
    integer      q;
    always @(*) begin
        out_v = 1'b0; consume = 1'b0; line_end = 1'b0; ib_pop = 1'b0; lr_pop = 1'b0;
        unit  = (d_fmt == 2'd2) ? unpack8(ib_d, d_q, f_pal) : (d_fmt == 2'd1) ? unpack565(ib_d, d_q[0]) : opaque(ib_d);
        // the unit the walker emits WHEN it emits one, formed from the walker's state alone: the line buffer's room flag
        // and restart gate only out_v, which keeps lb_avail off the 256-bit out_d select into lb_data (timing); the
        // line buffer takes lb_data only with lb_en, so the value without it is free
        if (d_nb != 8'd0) out_d = (d_sh == 3'd0) ? unit : 256'({unit, d_prev} >> (32 * d_sh));
        else if (d_o < d_k) out_d = 256'({BLACK8, d_prev} >> (32 * d_sh));   // stream ended: last unit, black after
        else out_d = BLACK8;                                                   // black to the mode width
        // pixels past the image width in the last image unit are black
        if (d_k != 8'd0 && d_o == d_k - 8'd1)
            for (q = 0; q < 8; q = q + 1) if (q > d_lq) out_d[q*32 +: 32] = 32'h000000FF;
        if (!d_act) lr_pop = lr_v && run && !restart;
        else if (lb_avail && !restart) begin
            if (d_nb != 8'd0) begin
                if (ib_v) begin
                    consume = 1'b1;
                    ib_pop  = (d_q == d_qlast);
                    if (d_skip != 2'd0) ;                                  // units before the row's first pixel
                    else if (d_sh == 3'd0) begin
                        if (d_o < d_k) out_v = 1'b1;
                    end else if (d_hp && d_o < d_k) out_v = 1'b1;
                end
            end else if (d_o < d_k) out_v = 1'b1;
            else if (d_o < d_u) out_v = 1'b1;
            else line_end = 1'b1;
        end
    end

    always @(posedge clk) begin
        lb_en <= 1'b0; lb_line_done <= 1'b0;
        if (rst || restart || !run) begin
            d_act <= 1'b0;
        end else begin
            lb_en <= out_v; lb_idx <= d_o; lb_data <= out_d; lb_line_done <= line_end;
            if (out_v) d_o <= d_o + 8'd1;
            if (consume) begin
                d_q <= (d_q == d_qlast) ? 2'd0 : d_q + 2'd1;
                if (ib_pop) d_nb <= d_nb - 8'd1;
                if (d_skip != 2'd0) d_skip <= d_skip - 2'd1;
                else begin d_prev <= unit; d_hp <= 1'b1; end
            end
            if (line_end) d_act <= 1'b0;
            if (lr_pop) begin
                d_act <= 1'b1; d_o <= 8'd0; d_q <= 2'd0; d_hp <= 1'b0;
                {d_fmt, d_skip, d_sh, d_nb, d_k, d_lq} <= {lr_d[25:24], lr_d[23:22], lr_d[21:19], lr_d[18:11],
                                                         lr_d[10:3], lr_d[2:0]};
            end
        end
    end
endmodule
`default_nettype wire
