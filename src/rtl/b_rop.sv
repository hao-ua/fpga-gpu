// b_rop — board B's band buffer + ES 2.0 output merger.
// Per fragment, in spec order: scissor -> stencil -> depth -> blend -> dither -> colour mask
// -> write, RMW-free against DDR (the band lives on chip, flushed to the render-target image once per band).
// ES 2.0 exactness:
//   * colour arrives per channel as round(clamp(f, 0, 1) * ONE), ONE = 16 (2^n - 1) for a channel of n bits (8.4
//     fixed point of the channel's step; c16 = 4080 units for RGBA8); blending is exact on those fixed-point values
//     (pixel_blend_b: round((s fs +- 16 d fd) / ONE)); the blend constant is in the same units;
//   * dithering after blending (§4.1.7) or truncation;
//   * clears are dithered when the clear's dither flag is set (§4.2.3: Clear applies scissor and dithering): the
//     clear colour is in channel units and every cleared pixel gets dither(clear, x, y) — on the lazy-clear
//     substitution in the RMW read, the flush and the tile-load overlay; the clear's colour mask is its own input;
//   * depth is unorm24 throughout (one depth domain): fragment z, clear depth and gl_FragDepth arrive as u24 (a
//     DEPTH_COMPONENT16 target's values are 16-bit, right-aligned: the comparisons do not care).
// Render-target formats (fmt as tmu24: 0 RGBA8, 1 RGBA4444, 2 RGBA5551, 3 RGB565): the band cell keeps a 32-bit
// colour field {R, G, B, A} of n-bit values right-aligned in bytes; the band load unpacks and the flush packs the
// 16-bit pixels (16 per burst: pixel k at [16k +: 16], as the TMU and the scanout read them). A group = the pixels of
// one colour burst (8 or 16).
// Depth / stencil persistence: an optional D24S8 image (ds_en; {stencil[31:24], depth[23:0]} per pixel, 32 bpp,
// 1 or 2 bursts per group) is read by the band load / preload and written by the flush, so depth and stencil keep
// their values across FLUSHes; separate clear flags for depth and stencil (the stencil clear masked by clear_smask),
// all clears inside the clear scissor. Without the image depth and stencil start from the clear values.
// Band buffer: three banks (render / flush / preload), lazy epoch clear (only when colour, depth and stencil are all
// cleared without scissor and masks); each bank is an inferred RAM or bb_bank (BBUF_BLOCKS). The flush
// reads one cell (a pixel pair) per cycle, packs into two alternating group accumulators while the
// writer issues the finished group's bursts, and writes an untouched fast-cleared band as its constant clear pattern
// (two groups read per line, the writer repeats the first). RMW pipeline: accept (port A read) -> v0 (destination on rd_a,
// blend starts) -> v1 (stencil / depth decision) -> v2..v5 -> v6 (port A write of the merged cell, blend result
// ready; pixel_blend_b has 6 stages). Accept waits while v6 writes and while the cell is
// in flight in v0..v5.
// MSAA (SAMPLES = 4): per-sample depth / stencil / blend lanes, box resolve at the flush.
// Timing: the band load and the preload issue their DDR bursts from row products kept in
// registers (y x bursts per line, + bpl per line) — no multiply in the issue cycle; the bank selects are one-hot copies.
// PIXEL PAIRS: a band-buffer cell holds the two horizontally adjacent pixels (2k, y) and
// (2k + 1, y) of a line — {pixel 1, pixel 0}, one epoch per cell —, so every port operation moves two pixels: fragments
// arrive as a quad row (frag_x even, frag_pv = the pixels present), the RMW pipe merges both pixels of the cell in one
// pass (2 x SAMPLES lanes), the early-depth peek returns both depths, the flush reads one cell = one pair per cycle on
// port A alone, the band load / preload write one cell per cycle. Cells per line cw = ceil(width / 2); a quad's two
// cells are its own, so the hazard check keeps its meaning. Port A takes 2 cycles per PAIR (read +
// write), 4 per full quad.
`default_nettype none
module b_rop #(
    parameter int CW     = 13,
    parameter int MAXW   = 1280,
    parameter int NLINES = 4,                     // band rows at the widest target: the banks hold MAXW/2 x NLINES cells
    parameter int NLMAX  = 8,                     // band rows a narrower target may use in the same cells (nl)
    // the cells (pixel pairs) a bank holds: at least MAXW/2 x NLINES; the board's 4 096 = 512 x 8 is the depth
    // of the 4 K x 9 blocks the banks are built from anyway (the same RAMB36 as 2 560 cells), so targets up to 1024 wide get
    // 8-line bands
    parameter int CELLS  = ((MAXW + 1) / 2) * NLINES,
    parameter int SAMPLES = 1,
    parameter bit BBUF_BLOCKS = 1'b0              // 1: band-buffer banks as explicit 1k x 36 RAMB36 blocks (bb_bank: 30 instead
                                                  // of 54 RAMB36 at MAXW 1280, but +≈ 1 k LUT of block-select muxes — measured
                                                  // out of context: 5 770 vs 4 8xx LUT); 0: Vivado's inferred mapping
)(
    input  wire        clk, rst,
    input  wire [10:0] width, band_y0,
    input  wire [3:0]  nl,                        // this frame's band rows (NLINES or NLMAX; b_render's rule from the width: nl x cells per line <= the bank)
    input  wire [1:0]  fmt,                       // render-target format (tmu24 codes)
    input  wire [28:0] color_base,                // render target, 32-bit word address (bytes / 4)
    input  wire [10:0] bursts_per_line,           // pitch / 32 bytes
    input  wire        ds_en,                     // a depth-stencil image is attached
    input  wire [28:0] ds_base,                   // its word address
    input  wire [10:0] ds_bpl,                    // its pitch / 32 bytes (>= bursts_per_line x bursts per group)
    input  wire        band_start, band_flush_req,
    output reg         band_flush_done,
    output wire        band_busy,
    output wire        band_run,                  // the band is set up (cleared / loaded): fragments and peeks are valid
    output wire        flush_busy,
    output wire        rmw_idle,
    // the band after this one: the preload loads it (next_ok: it is announced), peeks may target it (pk_next);
    // pl_rdy / pl_rdy_y0 = the preload bank holds band pl_rdy_y0 complete
    input  wire        next_ok,
    // pf_clear (b_render's go): a frame starts — no band preloaded before it is used. With the preload started
    // in advance (b_render: the band below the rendering one) a frame that ends early (an abort) can leave one behind; a
    // preload still running is completed and dropped (pl_void)
    input  wire        pf_clear,
    input  wire [10:0] next_y0,
    output wire        pl_rdy,
    output wire [10:0] pl_rdy_y0,
    // scissor
    input  wire        scissor_en,
    input  wire signed [CW-1:0] sc_x0, sc_y0, sc_x1, sc_y1,
    // stencil (unsuffixed = front face, *_back = back face)
    input  wire        stencil_en,
    input  wire [2:0]  stencil_func,      stencil_func_back,
    input  wire [7:0]  stencil_ref,       stencil_ref_back,
    input  wire [7:0]  stencil_vmask,     stencil_vmask_back,
    input  wire [7:0]  stencil_wmask,     stencil_wmask_back,
    input  wire [2:0]  op_sfail,  op_dpfail,  op_dppass,
    input  wire [2:0]  op_sfail_back, op_dpfail_back, op_dppass_back,
    // depth
    input  wire        depth_test_en, depth_mask,
    input  wire [2:0]  depth_func,
    // blend / colour mask / dither
    input  wire        blend_en,
    input  wire [3:0]  blend_src_factor, blend_dst_factor, blend_src_factor_a, blend_dst_factor_a,
    input  wire [2:0]  blend_op, blend_op_a,
    input  wire [51:0] blend_c16,                 // constant colour {R, G, B, A} in channel units
    input  wire [25:0] blend_ca,                  // constant alpha in G's units [25:13], in R / B's units [12:0]
    input  wire [3:0]  color_mask,
    input  wire        dither_en,
    // clear (per render pass)
    input  wire [51:0] clear_c16,                 // clear colour in channel units
    input  wire        clear_dither,
    input  wire [3:0]  clear_cmask,
    input  wire [23:0] clear_depth,
    input  wire [7:0]  clear_stencil,
    input  wire        clear_color_en, clear_scissor_en,
    input  wire        clear_depth_en, clear_stencil_en,
    input  wire [7:0]  clear_smask,
    input  wire signed [CW-1:0] clear_sc_x0, clear_sc_y0, clear_sc_x1, clear_sc_y1,
    // band LOAD read master (colour and depth-stencil bursts)
    output wire        lr_en, output wire [28:0] lr_addr,
    input  wire        lr_rdy, lr_rd_valid, input wire [255:0] lr_rd_data,
    // fragment in
    // fragments as pixel PAIRS (a quad row): frag_x = the even x, frag_pv[p] = pixel x + p present; pixel p's fields at
    // index p (frag_c16[p*52 +: 52], frag_sa[p*26 +: 26], frag_z[(p*SAMPLES + s)*24 +: 24], frag_scov[p*SAMPLES + s])
    input  wire        frag_valid,
    input  wire [1:0]  frag_pv,
    input  wire signed [CW-1:0] frag_x, frag_y,
    input  wire [2*52-1:0] frag_c16,              // {R, G, B, A} in channel units, per pixel
    input  wire [2*26-1:0] frag_sa,               // source alpha in G's units [25:13], in R / B's units [12:0], per pixel
    input  wire [2*SAMPLES*24-1:0] frag_z,        // per-sample unorm24 depth, per pixel
    input  wire [2*SAMPLES-1:0]    frag_scov,
    input  wire        frag_front,
    output wire        frag_ready,
    // flush -> DDR
    output reg         w_en, output reg [28:0] w_addr,
    output reg [255:0] w_data, output reg [31:0] w_mask, output reg w_wren, w_wend,
    input  wire        w_rdy, w_wdf_rdy,
    // early-depth peek (sample 0) of the PAIR (pk_x, pk_y), (pk_x + 1, pk_y) — pk_x even —, pk_z = {depth of x + 1, depth
    // of x} TWO cycles after pk_en; valid only while band_run (during a tile load a cell not yet reloaded holds an older
    // band's depth under the current epoch). pk_next: the peek is for the NEXT band: under the lazy fast clear
    // its initial depth is the clear depth (nothing read); else the preload bank's cell, valid once pl_ok for that band
    input  wire        pk_en, pk_next, input wire [CW-1:0] pk_x, pk_y,
    output wire [47:0] pk_z,
    // the cell addresses of the fragment pair and of the peek, formed by b_render a stage ahead (timing: row_base[y -
    // band_y0] + x / 2 stays off the paths into the bank addresses and the hazard compares)
    input  wire [$clog2(CELLS)-1:0] frag_addr, pk_addr,
    // the cell address b_render presents NEXT cycle (its next fragment's when this one leaves, else this one's again): the
    // hazard check runs a cycle ahead on it (timing: frag_addr -> six address compares -> accept -> ready -> the quad
    // hand-over's ~300 registers in b_render does not fit one cycle)
    input  wire [$clog2(CELLS)-1:0] frag_addr_n,
    output wire [3:0]  dbg_st,
    output wire [2:0]  dbg_fst,
    output wire [1:0]  dbg_pst
);
    localparam int DW    = 24;
    localparam int NS    = SAMPLES;
    localparam int MAXC  = (MAXW + 1) / 2;          // cells (pixel pairs) per line at most
    localparam int AW    = $clog2(CELLS);
    if (CELLS < MAXC * NLINES) $error("b_rop: CELLS below MAXW/2 x NLINES");
    localparam int CELLW = 8 + DW + 32;             // one sample: {stencil, depth, colour}
    localparam int PW    = NS*CELLW;                // one pixel
    localparam int P2W   = 2*PW;                    // one cell = the pair {pixel 1, pixel 0}
    localparam int NL2   = 2*NS;                    // merge lanes: lane p*NS + s = pixel p, sample s
    wire [10:0]  cw      = (width + 11'd1) >> 1;    // cells per line
    localparam int SH    = (NS==4)?2 : (NS==2)?1 : 0;
    localparam int EPOCHW = 8;
    localparam [EPOCHW-1:0] EPOCH_MAX = {EPOCHW{1'b1}};
    localparam int BLAT  = 6;                       // pixel_blend_b latency (the result lands with v6)

    // ---- the target's pixel groups ----
    wire        bpp16 = (fmt != 2'd0);
    wire [3:0]  psh   = bpp16 ? 4'd4 : 4'd3;            // log2 pixels per colour burst
    wire [3:0]  pxl   = bpp16 ? 4'd15 : 4'd7;           // last pixel of a group
    wire [1:0]  nds   = ds_en ? (bpp16 ? 2'd2 : 2'd1) : 2'd0;   // depth-stencil bursts per group
    // burst addresses from a row product kept in a register (row = y x bursts per line, < 2^22): the band load, the
    // preload and the flush issue their bursts from these (timing: no multiply in the issue
    // cycle); colour burst of group g in row y, and its i-th depth-stencil burst
    function automatic [28:0] cbaddr_r(input [21:0] row, input [10:0] g);
        cbaddr_r = color_base + 29'(({7'd0, row} + {18'd0, g}) << 3);
    endfunction
    function automatic [28:0] dbaddr_r(input [21:0] row, input [10:0] g, input [1:0] i);
        dbaddr_r = ds_base + 29'(({7'd0, row} + (bpp16 ? {17'd0, g, 1'b0} : {18'd0, g}) + {27'd0, i}) << 3);
    endfunction
    // loaded colour of pixel k of a burst -> the cell's {R, G, B, A} bytes (n-bit values right-aligned)
    function automatic [31:0] unpack(input [255:0] b, input [3:0] k, input [1:0] f);
        reg [15:0] h; h = b[k*16 +: 16];
        case (f)
            2'd0:    unpack = b[k[2:0]*32 +: 32];
            2'd1:    unpack = {4'd0, h[15:12], 4'd0, h[11:8], 4'd0, h[7:4], 4'd0, h[3:0]};
            2'd2:    unpack = {3'd0, h[15:11], 3'd0, h[10:6], 3'd0, h[5:1], 7'd0, h[0]};
            default: unpack = {3'd0, h[15:11], 2'd0, h[10:5], 3'd0, h[4:0], 8'd0};
        endcase
    endfunction
    function automatic [15:0] pack16(input [31:0] c, input [1:0] f);
        case (f)
            2'd1:    pack16 = {c[27:24], c[19:16], c[11:8], c[3:0]};
            2'd2:    pack16 = {c[28:24], c[20:16], c[12:8], c[0]};
            default: pack16 = {c[28:24], c[21:16], c[12:8]};
        endcase
    endfunction

    function automatic cmp(input [2:0] f, input [DW-1:0] a, b);
        case (f) 3'd0:cmp=0; 3'd1:cmp=(a<b); 3'd2:cmp=(a==b); 3'd3:cmp=(a<=b);
                 3'd4:cmp=(a>b); 3'd5:cmp=(a!=b); 3'd6:cmp=(a>=b); default:cmp=1; endcase
    endfunction
    function automatic cmp8(input [2:0] f, input [7:0] a, b);
        case (f) 3'd0:cmp8=0; 3'd1:cmp8=(a<b); 3'd2:cmp8=(a==b); 3'd3:cmp8=(a<=b);
                 3'd4:cmp8=(a>b); 3'd5:cmp8=(a!=b); 3'd6:cmp8=(a>=b); default:cmp8=1; endcase
    endfunction
    function automatic [7:0] sop(input [2:0] o, input [7:0] s, ref8);
        case (o) 3'd0:sop=s; 3'd1:sop=0; 3'd2:sop=ref8;
                 3'd3:sop=(s==8'hff)?8'hff:s+8'd1; 3'd4:sop=(s==0)?8'd0:s-8'd1;
                 3'd5:sop=~s; 3'd6:sop=s+8'd1; default:sop=s-8'd1; endcase
    endfunction
    function automatic [3:0] bayer(input [1:0] yy, xx);
        case ({yy,xx})
            4'h0:bayer=0;  4'h1:bayer=8;  4'h2:bayer=2;  4'h3:bayer=10;
            4'h4:bayer=12; 4'h5:bayer=4;  4'h6:bayer=14; 4'h7:bayer=6;
            4'h8:bayer=3;  4'h9:bayer=11; 4'ha:bayer=1;  4'hb:bayer=9;
            4'hc:bayer=15; 4'hd:bayer=7;  4'he:bayer=13; default:bayer=5;
        endcase
    endfunction
    // the clear colour at pixel (x, y): floor((c + t) / 16) per channel, t = Bayer threshold when the clear dithers
    function automatic [31:0] dclear(input [CW-1:0] x, input [CW-1:0] y);
        reg [3:0] t; reg [13:0] u;
        t = clear_dither ? bayer(y[1:0], x[1:0]) : 4'd0;
        for (int c = 0; c < 4; c++) begin
            u = {1'b0, clear_c16[(3-c)*13 +: 13]} + {10'd0, t};
            dclear[(3-c)*8 +: 8] = u[11:4];
        end
    endfunction
    function automatic [31:0] resolve(input [PW-1:0] px);
        integer k; reg [11:0] sr,sg,sb,sa; begin
            sr=0; sg=0; sb=0; sa=0;
            for (k=0;k<NS;k=k+1) begin
                sr = sr + px[k*CELLW+24 +: 8]; sg = sg + px[k*CELLW+16 +: 8];
                sb = sb + px[k*CELLW+8  +: 8]; sa = sa + px[k*CELLW+0  +: 8];
            end
            resolve = {sr[7+SH:SH], sg[7+SH:SH], sb[7+SH:SH], sa[7+SH:SH]};
        end
    endfunction
    // inside the clear's scissor box (or no box)
    function automatic in_box(input signed [CW-1:0] x, y);
        in_box = ~clear_scissor_en | (x >= clear_sc_x0 && x < clear_sc_x1 && y >= clear_sc_y0 && y < clear_sc_y1);
    endfunction
    wire [31:0] ccm32 = {{8{clear_cmask[3]}},{8{clear_cmask[2]}},{8{clear_cmask[1]}},{8{clear_cmask[0]}}};
    // the pixel of a loaded pixel: colour (the clear, through its mask, over the loaded colour inside the box), depth and
    // stencil (the loaded depth-stencil word, or the clear values where cleared / when no image is attached)
    function automatic [PW-1:0] ld_cell(input signed [CW-1:0] x, y, input [31:0] c, input [31:0] dsw);
        reg ib; reg [31:0] col; reg [23:0] dep; reg [7:0] st;
        ib  = in_box(x, y);
        col = (clear_color_en & ib) ? ((dclear(x, y) & ccm32) | (c & ~ccm32)) : c;
        dep = ds_en ? ((clear_depth_en & ib) ? clear_depth : dsw[23:0]) : clear_depth;
        st  = ds_en ? ((clear_stencil_en & ib) ? ((clear_stencil & clear_smask) | (dsw[31:24] & ~clear_smask)) : dsw[31:24])
                    : clear_stencil;
        ld_cell = {NS{ {st, dep, col} }};
    endfunction

    // ---- triple-buffered band buffer (render rbk / flush fbk / preload pbk) ----
    // Each bank is a true-dual-port RAM of CELLS cells (pixel pairs): port A = the render path (RMW read / write),
    // the flush's read or the preload's write; port B = the early-depth peek (render bank; the flush bank's port B is
    // free — a 4-pixel flush read is the next step if the packer should ever bind). BBUF_BLOCKS selects explicit 1k x 36
    // RAMB36 blocks (bb_bank) at the price of their wider output muxes.
    localparam int BW = P2W + EPOCHW;
    reg [1:0] rbk, fbk, pbk;
    reg [2:0] rbk1, fbk1, pbk1;                   // the same as one-hot selects for the bank muxes
    reg [AW-1:0] addr_a; reg [BW-1:0] wr_a; reg we_a;
    reg [AW-1:0] fl_addr;                         // the flush's cell (the pair)
    reg [AW-1:0] pl_addr; reg [BW-1:0] pl_wdata; reg pl_we;
    wire [AW-1:0] ren_addr;
    wire ren_we; wire [BW-1:0] ren_wdata;
    localparam int LB = (NLINES > 1) ? $clog2(NLINES) : 1;
    if ((NLINES & (NLINES-1)) != 0) $error("b_rop: NLINES must be a power of 2");
    // (the cell addresses pk_addr / frag_addr arrive from b_render: row_base[y - y0] + x / 2 of the fragment's band)
    wire [BW-1:0] bqa [0:2], bqb [0:2];           // the banks' port A / port B read data (registered in the RAM)
    genvar gb;
    generate for (gb = 0; gb < 3; gb = gb + 1) begin : BANK
        wire [AW-1:0] aa = rbk1[gb] ? ren_addr : fbk1[gb] ? fl_addr : pl_addr;
        wire          wea = rbk1[gb] ? ren_we : pbk1[gb] ? pl_we : 1'b0;
        wire [BW-1:0] da = rbk1[gb] ? ren_wdata : pl_wdata;
        wire [AW-1:0] ab = pk_addr;
        wire          eb = pk_en;                     // the peek reads the render bank, or the preload bank (pk_next)
        if (BBUF_BLOCKS) begin : BLK
            bb_bank #(.W(BW), .AW(AW), .DEPTH(CELLS)) u_bb (.clk, .a_addr(aa), .a_we(wea), .a_wdata(da), .a_rdata(bqa[gb]),
                                                               .b_en(eb), .b_addr(ab), .b_rdata(bqb[gb]));
        end else begin : INF                      // the inferred array (port A read-first)
            (* ram_style="block" *) reg [BW-1:0] m [0:CELLS-1];
            reg [BW-1:0] qa, qb;
            always @(posedge clk) begin if (wea) m[aa] <= da; qa <= m[aa]; end
            always @(posedge clk) if (eb) qb <= m[ab];
            assign bqa[gb] = qa; assign bqb[gb] = qb;
            // synthesis translate_off
            // cross-port collision: port B reads the cell port A writes this cycle (the RAMB36 does not guarantee the
            // read value; the model returns the old cell) — counted for the benches
            integer n_col = 0;
            always @(posedge clk) if (wea && eb && aa == ab) n_col = n_col + 1;
            // synthesis translate_on
        end
    end endgenerate
    wire [BW-1:0] rd_a   = rbk1[0] ? bqa[0] : rbk1[1] ? bqa[1] : bqa[2];
    wire [BW-1:0] fl_rd  = fbk1[0] ? bqa[0] : fbk1[1] ? bqa[1] : bqa[2];   // the flush's cell: pixel 0 at [0 +: PW], 1 at [PW +: PW]
    reg [EPOCHW-1:0] cur_epoch [0:2];
    reg [EPOCHW-1:0] fl_epoch;
    reg pf_ok [0:2]; reg [10:0] pf_y0 [0:2];
    reg [CW-1:0] s0_x, s0_y;                                    // the pair in v0 (its even x: its clear colours)
    function automatic [PW-1:0] clear_px(input [CW-1:0] x, y);  // a never-written pixel of a fast-cleared band
        clear_px = {NS{ {clear_stencil, clear_depth, dclear(x, y)} }};
    endfunction
    wire [P2W-1:0] clear_cell = {clear_px(s0_x + 1'b1, s0_y), clear_px(s0_x, s0_y)};
    wire           rd_hit  = (rd_a[P2W +: EPOCHW] == cur_epoch[rbk]);
    wire [P2W-1:0] rd_eff  = rd_hit ? rd_a[P2W-1:0] : clear_cell;
    wire           fl_hit  = (fl_rd[P2W +: EPOCHW] == fl_epoch);
    wire [BW-1:0] rd_b  = rbk1[0] ? bqb[0] : rbk1[1] ? bqb[1] : bqb[2];
    wire [BW-1:0] rd_bn = pbk1[0] ? bqb[0] : pbk1[1] ? bqb[1] : bqb[2];     // the preload bank (the next band's)
    reg  pk_next_q;                                                          // aligned with the read
    always @(posedge clk) if (pk_en) pk_next_q <= pk_next;
    wire fast;
    // the peeked depths registered (timing: block-RAM output -> epoch compare -> bank select -> b_render's depth
    // compare does not fit one cycle): pk_z = {pixel 1, pixel 0} two cycles after pk_en
    reg [47:0] pk_z_q;
    always @(posedge clk)
        pk_z_q <= pk_next_q ? ((fast || rd_bn[P2W +: EPOCHW] != cur_epoch[pbk]) ? {2{clear_depth}} : {rd_bn[PW+32 +: DW], rd_bn[32 +: DW]})
                            : ((rd_b[P2W +: EPOCHW] == cur_epoch[rbk]) ? {rd_b[PW+32 +: DW], rd_b[32 +: DW]} : {2{clear_depth}});
    assign pk_z = pk_z_q;

    typedef enum logic [3:0] {S_IDLE,S_CLR,S_RUN,S_LD_RD,S_LD_WR,S_WIPE,S_PACK} st_t; st_t st;
    typedef enum logic [1:0] {PL_IDLE,PL_POP,PL_WR} pst_t; pst_t pst;
    assign dbg_st = st; assign dbg_pst = pst;
    reg [AW-1:0] caddr;
    reg sld_lren, pl_lren; reg [28:0] sld_lraddr, pl_lraddr;
    reg [21:0] ld_rowc, ld_rowd, pl_rowc, pl_rowd;             // row x bursts per line (colour / depth-stencil)
    wire [10:0] pl_y0n = next_y0;                               // the next band's first row (announced by the sequencer)
    wire ld_active = (st==S_LD_RD);
    assign lr_en   = ld_active ? sld_lren   : pl_lren;
    assign lr_addr = ld_active ? sld_lraddr : pl_lraddr;
    // lazy fast clear: colour, depth and stencil all cleared, no scissor, no masks (else the band is loaded)
    wire ds_clr_all = clear_depth_en & clear_stencil_en & (clear_smask == 8'hFF);
    assign fast = clear_color_en & ~clear_scissor_en & (clear_cmask == 4'hf) & (~ds_en | ds_clr_all);
    wire load_modify = ~fast;
    assign pl_rdy = pf_ok[pbk]; assign pl_rdy_y0 = pf_y0[pbk];
    reg  band_fast, touched;                                    // the render band: lazily fast-cleared / a fragment accepted
    // ---- preload (the next band's load, in the background) ----
    localparam int PLFD = 8;
    (* ram_style="distributed" *) reg [255:0] plfifo [0:PLFD-1];
    reg [3:0]  plf_wp, plf_rp, pl_out;
    reg [10:0] pl_line, pl_burst; reg [3:0] pl_px; reg [255:0] pl_ld_data, pl_ds0, pl_ds1; reg [10:0] pl_y0;
    reg [1:0]  pl_pk, pl_ik;                                    // burst of the group being popped / issued
    reg [10:0] pl_iline, pl_iburst;
    reg        pl_active, pl_iss_active;
    reg        pl_void;                                         // the running preload belongs to an earlier frame (pf_clear)
    wire       plf_empty   = (plf_wp == plf_rp);
    wire [10:0] nl_1 = 11'(nl) - 11'd1;                         // the band's last line
    wire       pl_iss_last = (pl_iburst==bursts_per_line-1) && (pl_iline==nl_1) && (pl_ik == nds);
    // (the load and the preload write one cell = the pair of pixels pl_px / ld_px (even) and + 1 of the group per cycle)
    wire signed [CW-1:0] pl_x   = CW'((pl_burst << psh) + pl_px);
    wire signed [CW-1:0] pl_row = $signed({2'b0,pl_y0}) + $signed({8'd0,pl_line[3:0]});
    wire [3:0]  pl_px1 = pl_px | 4'd1;
    wire [31:0] pl_dsw  = pl_px[3] ? pl_ds1[pl_px[2:0]*32 +: 32]  : pl_ds0[pl_px[2:0]*32 +: 32];
    wire [31:0] pl_dsw1 = pl_px[3] ? pl_ds1[pl_px1[2:0]*32 +: 32] : pl_ds0[pl_px1[2:0]*32 +: 32];
    wire [P2W-1:0] pl_cell = {ld_cell(pl_x + 1'b1, pl_row, unpack(pl_ld_data, pl_px1, fmt), pl_dsw1),
                              ld_cell(pl_x,        pl_row, unpack(pl_ld_data, pl_px,  fmt), pl_dsw)};
    // ---- foreground load ----
    reg [10:0] ld_line, ld_burst; reg [3:0] ld_px; reg [255:0] ld_data, ld_ds0, ld_ds1; reg [1:0] ld_iss, ld_got;
    wire signed [CW-1:0] ld_x   = CW'((ld_burst << psh) + ld_px);
    wire signed [CW-1:0] ld_row = $signed({2'b0,band_y0}) + $signed({8'd0,ld_line[3:0]});
    wire [10:0] ld_y = band_y0 + ld_line;
    wire [3:0]  ld_px1 = ld_px | 4'd1;
    wire [31:0] ld_dsw  = ld_px[3] ? ld_ds1[ld_px[2:0]*32 +: 32]  : ld_ds0[ld_px[2:0]*32 +: 32];
    wire [31:0] ld_dsw1 = ld_px[3] ? ld_ds1[ld_px1[2:0]*32 +: 32] : ld_ds0[ld_px1[2:0]*32 +: 32];
    wire [P2W-1:0] ld_cellv = {ld_cell(ld_x + 1'b1, ld_row, unpack(ld_data, ld_px1, fmt), ld_dsw1),
                               ld_cell(ld_x,        ld_row, unpack(ld_data, ld_px,  fmt), ld_dsw)};

    // ---- RMW pipeline: accept -> v0 .. v6 (write) ----
    reg v0, v1, v2, v3, v4, v5, v6;
    reg [AW-1:0] s0_addr, s1_addr, s2_addr, s3_addr, s4_addr, s5_addr, s6_addr;
    // lanes NL2 = 2 x NS: lane p*NS + s = pixel p of the pair, sample s (the cell is pixel-major: lane l at [l*CELLW +: CELLW])
    reg [DW-1:0] s0_fz [0:NL2-1], s1_fz [0:NL2-1], s2_fz [0:NL2-1], s3_fz [0:NL2-1], s4_fz [0:NL2-1], s5_fz [0:NL2-1], s6_fz [0:NL2-1];
    reg [NL2-1:0] s0_scov, s1_scov;               // a pixel not present / outside the scissor has no covered samples
    reg [51:0]   s0_c16 [0:1]; reg [25:0] s0_sa [0:1];
    reg [3:0]    s0_t [0:1];
    reg          s0_ff, s1_ff;
    reg [NL2-1:0] s2_cw, s3_cw, s4_cw, s5_cw, s6_cw;
    reg [7:0]    s2_sten [0:NL2-1], s3_sten [0:NL2-1], s4_sten [0:NL2-1], s5_sten [0:NL2-1], s6_sten [0:NL2-1];
    reg [DW-1:0] s2_keepz [0:NL2-1], s3_keepz [0:NL2-1], s4_keepz [0:NL2-1], s5_keepz [0:NL2-1], s6_keepz [0:NL2-1];
    reg [31:0]   s2_keepc [0:NL2-1], s3_keepc [0:NL2-1], s4_keepc [0:NL2-1], s5_keepc [0:NL2-1], s6_keepc [0:NL2-1];
    reg [P2W-1:0] rd_eff_r;
    wire signed [CW-1:0] frag_x1 = frag_x + 1'b1;
    wire in_sc_y = (frag_y>=sc_y0 && frag_y<sc_y1);
    wire [1:0] in_scissor = {~scissor_en | (frag_x1>=sc_x0 && frag_x1<sc_x1 && in_sc_y), ~scissor_en | (frag_x>=sc_x0 && frag_x<sc_x1 && in_sc_y)};
    wire [1:0] pv_eff = frag_pv & in_scissor;     // the pixels that enter the pipe
    // haz: the fragment's cell is in stages 0..5 (a read-modify-write in flight). A REGISTER, computed a cycle ahead from
    // frag_addr_n against the stages as they will stand next cycle: the fragment accepted now in stage 0, stages 0..4 moved
    // up one (the pipeline shifts every cycle; the valid bits gate). Exact: haz_r at t+1 == the direct compare at t+1.
    reg  haz_r;
    wire accept_pipe;
    always @(posedge clk) begin
        if (rst) haz_r <= 1'b0;
        else haz_r <= (accept_pipe && (frag_addr_n==frag_addr)) || (v0 && (frag_addr_n==s0_addr)) || (v1 && (frag_addr_n==s1_addr)) ||
                      (v2 && (frag_addr_n==s2_addr)) || (v3 && (frag_addr_n==s3_addr)) || (v4 && (frag_addr_n==s4_addr));
    end
    wire haz = haz_r;
    wire accept_ok = ~v6 & ~we_a & ~haz;
    assign frag_ready = (st==S_RUN) & ((pv_eff != 2'b00) ? accept_ok : 1'b1);
    assign accept_pipe = (st==S_RUN) & frag_valid & (pv_eff != 2'b00) & accept_ok;
    // synthesis translate_off
    always @(posedge clk) if (!rst) begin
        if (accept_pipe && (frag_y < $signed({2'b0,band_y0}) || frag_y >= $signed({2'b0,band_y0}) + $signed({9'd0, nl})))
            $fatal(1, "b_rop: accepted fragment y=%0d outside band [%0d,%0d)", frag_y, band_y0, band_y0+nl);
        if (frag_valid && frag_x[0]) $fatal(1, "b_rop: a fragment pair with an odd x (%0d)", frag_x);
        if (pk_en && pk_x[0]) $fatal(1, "b_rop: a peek with an odd x (%0d)", pk_x);
        if (pk_en && !pk_next && ({1'b0,pk_y} < {3'b0,band_y0} || {1'b0,pk_y} >= {3'b0,band_y0} + {10'd0, nl}))
            $fatal(1, "b_rop: early-depth peek y=%0d outside band [%0d,%0d)", pk_y, band_y0, band_y0+nl);
        if (pk_en && pk_next && !fast && !(pf_ok[pbk] && {1'b0,pk_y} >= {3'b0,pf_y0[pbk]} && {1'b0,pk_y} < {3'b0,pf_y0[pbk]} + {10'd0, nl}))
            $fatal(1, "b_rop: next-band peek y=%0d without its preload (pf_ok %b y0 %0d)", pk_y, pf_ok[pbk], pf_y0[pbk]);
    end
    // synthesis translate_on
    assign band_busy  = (st!=S_IDLE) & (st!=S_RUN);
    assign band_run   = (st==S_RUN);
    wire pipe_empty   = ~v0 & ~v1 & ~v2 & ~v3 & ~v4 & ~v5 & ~v6;
    assign rmw_idle   = pipe_empty & ~we_a;
    assign ren_addr = accept_pipe ? frag_addr : (v6 ? s6_addr : addr_a);

    wire [2:0] sfn  = s1_ff ? stencil_func   : stencil_func_back;
    wire [7:0] sref = s1_ff ? stencil_ref    : stencil_ref_back;
    wire [7:0] svm  = s1_ff ? stencil_vmask  : stencil_vmask_back;
    wire [7:0] swm  = s1_ff ? stencil_wmask  : stencil_wmask_back;
    wire [2:0] osf  = s1_ff ? op_sfail       : op_sfail_back;
    wire [2:0] odpf = s1_ff ? op_dpfail      : op_dpfail_back;
    wire [2:0] odpp = s1_ff ? op_dppass      : op_dppass_back;

    wire [31:0] blend_final_s [0:NL2-1];
    wire [7:0]  new_sten_s    [0:NL2-1];
    wire        cw_s          [0:NL2-1];
    wire [P2W+EPOCHW-1:0] pipe_wr_word;
    assign pipe_wr_word[P2W +: EPOCHW] = cur_epoch[rbk];
    genvar gs;
    generate for (gs=0; gs<NL2; gs=gs+1) begin : LANE
        localparam int GP = gs / NS;                                 // the lane's pixel of the pair
        wire [31:0]   rd_color = rd_eff[gs*CELLW       +: 32];      // blend destination: live in v0
        wire [DW-1:0] rd_depth = rd_eff_r[gs*CELLW+32    +: DW];
        wire [7:0]    rd_sten  = rd_eff_r[gs*CELLW+32+DW +: 8];
        wire spass = ~stencil_en | cmp8(sfn, sref & svm, rd_sten & svm);
        wire dpass = ~depth_test_en | cmp(depth_func, s1_fz[gs], rd_depth);
        wire [2:0] sop_sel = spass ? (dpass ? odpp : odpf) : osf;
        wire [7:0] sop_raw = sop(sop_sel, rd_sten, sref);
        wire [7:0] sten_new = stencil_en ? ((sop_raw & swm)|(rd_sten & ~swm)) : rd_sten;
        assign cw_s[gs]       = s1_scov[gs] & spass & dpass;
        assign new_sten_s[gs] = s1_scov[gs] ? sten_new : rd_sten;
        pixel_blend_b u_pb (.clk, .fmt, .src_c16(s0_c16[GP]), .src_alpha(s0_sa[GP]), .dst_color(rd_color), .blend_c16, .blend_ca,
            .enable(blend_en), .src_factor(blend_src_factor), .dst_factor(blend_dst_factor),
            .src_factor_a(blend_src_factor_a), .dst_factor_a(blend_dst_factor_a),
            .op(blend_op), .op_a(blend_op_a), .mask(color_mask), .dith(s0_t[GP]), .final_color(blend_final_s[gs]));
        assign pipe_wr_word[gs*CELLW +: CELLW] = { s6_sten[gs],
                                                   (s6_cw[gs] & depth_mask) ? s6_fz[gs] : s6_keepz[gs],
                                                   s6_cw[gs] ? blend_final_s[gs] : s6_keepc[gs] };
    end endgenerate
    assign ren_we    = we_a | v6;
    assign ren_wdata = v6 ? pipe_wr_word : wr_a;

    // ---- flush: the band (bank fbk) is read one cell (a pixel pair) per cycle (port A) and packed into one
    //      of two group accumulators (a group = the pixels of one colour burst, 8 or 16); the writer issues each full
    //      group's 1 + nds bursts while the next group is packed (the packer reserves an accumulator at a group's first
    //      pair, so it never stalls inside a group). A fast-cleared band no fragment touched (every cell misses the
    //      band's epoch: the whole band is the clear pattern, which repeats every 4 pixels) is read for the first and
    //      the last group of each line only; the writer repeats the first group's words for the bursts in between
    //      (bit-exact when those bursts are fully inside the width: const_ok). The band is "flushed" (band_flush_done,
    //      the banks rotate) once its cells are read; the writes of its last groups overlap the next band's start.
    wire       const_ok = ({4'd0, bursts_per_line - 11'd1} << psh) < {4'd0, width};   // bursts 0 .. bpl-2 fully visible
    reg        fl_const;                                      // the band being flushed uses the constant pattern
    reg        pk_run;                                        // the packer walks the band's groups
    reg [10:0] fl_line, fl_burst; reg [3:0] fl_px;            // the pair being issued: pixels fl_px, fl_px + 1 of its group
    reg [AW-1:0] fl_lbase;                                    // fl_line x width
    reg [21:0] fl_rowc, fl_rowd;                              // row products of fl_line (no multiply in the issue path)
    reg [10:0] fl_band_y0;
    reg        ia, pa, wa;                                    // accumulator being issued into / packed into / written
    reg        acc_full [0:1];
    reg [255:0] acc_col [0:1], acc_ds0 [0:1], acc_ds1 [0:1];
    reg [21:0] acc_rowc [0:1], acc_rowd [0:1]; reg [10:0] acc_burst [0:1], acc_rep [0:1];
    // the pair in flight: issue (fl_addr, one cell) -> RAM read -> pack, two cycles
    reg        sh1_v, sh2_v, sh1_last, sh2_last; reg [3:0] sh1_px, sh2_px; reg [CW-1:0] sh1_x, sh2_x, sh1_y, sh2_y;
    reg [21:0] sh1_rowc, sh2_rowc, sh1_rowd, sh2_rowd; reg [10:0] sh1_burst, sh2_burst, sh1_rep, sh2_rep;
    wire       pk_busy  = pk_run | sh1_v | sh2_v;             // cells of the flush bank still to be read
    wire       grp_last = (fl_burst == bursts_per_line - 11'd1);                 // the line's last group
    wire [10:0] grp_rep  = (fl_const && fl_burst == 11'd0 && !grp_last) ? bursts_per_line - 11'd1 : 11'd1;
    wire [10:0] grp_next = fl_const ? bursts_per_line - 11'd1 : fl_burst + 11'd1;   // the line's next group
    wire [AW-1:0] fl_gaddr = fl_lbase + AW'(((fl_burst << psh) + fl_px) >> 1);   // the pair's cell
    // the pack stage: the cell's pixel 0 (even x) / pixel 1 (odd x) -> colour and depth-stencil words
    wire [CW-1:0] sh2_xb = sh2_x + 1'b1;
    wire [PW-1:0] fl_p0 = fl_rd[0 +: PW], fl_p1 = fl_rd[PW +: PW];
    wire        fl_visa = (sh2_x  < {2'b00, width});          // padding pixels of the pitch are written as 0
    wire        fl_visb = (sh2_xb < {2'b00, width});
    wire [31:0] fl_cola = !fl_visa ? 32'd0 : fl_hit ? resolve(fl_p0) : dclear(sh2_x,  sh2_y);
    wire [31:0] fl_colb = !fl_visb ? 32'd0 : fl_hit ? resolve(fl_p1) : dclear(sh2_xb, sh2_y);
    wire [31:0] fl_dswa = !fl_visa ? 32'd0 : fl_hit ? {fl_p0[32+DW +: 8], fl_p0[32 +: DW]} : {clear_stencil, clear_depth};
    wire [31:0] fl_dswb = !fl_visb ? 32'd0 : fl_hit ? {fl_p1[32+DW +: 8], fl_p1[32 +: DW]} : {clear_stencil, clear_depth};
    wire [63:0] fl_pair = bpp16 ? {32'd0, pack16(fl_colb, fmt), pack16(fl_cola, fmt)} : {fl_colb, fl_cola};
    // the writer: the full accumulator wa, 1 + nds bursts per group, acc_rep groups (the constant pattern)
    typedef enum logic [1:0] {W_IDLE, W_WB, W_WK} wst_t; wst_t ws;
    reg [1:0]  fwi; reg [10:0] w_burst, w_rep; reg [21:0] w_rowc, w_rowd;
    reg        fl_cmd_ack, fl_wdf_ack;
    assign flush_busy = pk_busy | acc_full[0] | acc_full[1] | (ws != W_IDLE);
    assign dbg_fst = {1'b0, ws};
    integer k;
    always @(posedge clk) begin
        if (rst) begin st<=S_IDLE; pst<=PL_IDLE; rbk<=2'd0; fbk<=2'd1; pbk<=2'd2;
                       rbk1<=3'b001; fbk1<=3'b010; pbk1<=3'b100;
                       v0<=1'b0; v1<=1'b0; v2<=1'b0; v3<=1'b0; v4<=1'b0; v5<=1'b0; v6<=1'b0;
                       pk_run<=1'b0; sh1_v<=1'b0; sh2_v<=1'b0; ia<=1'b0; pa<=1'b0; wa<=1'b0; acc_full[0]<=1'b0; acc_full[1]<=1'b0;
                       ws<=W_IDLE; fwi<=2'd0; band_fast<=1'b0; touched<=1'b0; fl_const<=1'b0;
                       we_a<=0; pl_we<=0; w_en<=0; w_wren<=0; w_wend<=0;
                       band_flush_done<=0; caddr<=0; sld_lren<=0; pl_lren<=0; ld_iss<=2'd0; ld_got<=2'd0;
                       pl_active<=1'b0; pl_iss_active<=1'b0; plf_wp<=4'd0; plf_rp<=4'd0; pl_out<=4'd0; pl_pk<=2'd0; pl_ik<=2'd0;
                       pf_ok[0]<=1'b0; pf_ok[1]<=1'b0; pf_ok[2]<=1'b0; pl_void<=1'b0;
                       cur_epoch[0]<=EPOCH_MAX; cur_epoch[1]<=EPOCH_MAX; cur_epoch[2]<=EPOCH_MAX; end
        else begin
            we_a<=0; pl_we<=0; band_flush_done<=0;
            case (st)
                S_IDLE: if (band_start) begin
                    band_fast <= fast; touched <= 1'b0;
                    if (fast) st<=S_CLR;
                    else if (pf_ok[rbk] && pf_y0[rbk]==band_y0) begin pf_ok[rbk]<=1'b0; st<=S_PACK; end
                    else begin ld_line<=0; ld_burst<=0; ld_iss<=2'd0; ld_got<=2'd0; st<=S_LD_RD;
                               pf_ok[rbk]<=1'b0;                // a preload of another band (none in a whole frame) is dropped
                               ld_rowc<=22'({11'd0, band_y0} * {11'd0, bursts_per_line}); ld_rowd<=22'({11'd0, band_y0} * {11'd0, ds_bpl}); end
                end
                S_PACK: st<=S_RUN;
                S_CLR: begin
                    if (cur_epoch[rbk]==EPOCH_MAX) begin caddr<=0; st<=S_WIPE; end
                    else begin cur_epoch[rbk]<=cur_epoch[rbk]+1'b1; st<=S_RUN; end
                end
                S_WIPE: begin
                    addr_a<=caddr; wr_a<={BW{1'b0}}; we_a<=1'b1;
                    if (caddr==AW'(cw*nl-1)) begin cur_epoch[rbk]<=1; st<=S_RUN; end else caddr<=caddr+1'b1;
                end
                S_RUN: begin
                    if (band_flush_req) begin                 // the previous band's cells all read (its writes may go on)
                        if (!pk_busy && pst==PL_IDLE && pipe_empty) begin
                            rbk<=pbk; fbk<=rbk; pbk<=fbk; rbk1<=pbk1; fbk1<=rbk1; pbk1<=fbk1;
                            fl_band_y0 <= band_y0; fl_epoch <= cur_epoch[rbk]; fl_const <= band_fast & ~touched & const_ok;
                            fl_line<=0; fl_burst<=0; fl_px<=0; fl_lbase<='0;
                            fl_rowc<=22'({11'd0, band_y0} * {11'd0, bursts_per_line}); fl_rowd<=22'({11'd0, band_y0} * {11'd0, ds_bpl});
                            pk_run<=1'b1; band_flush_done<=1'b1; st<=S_IDLE;
                        end
                    end
                end
                // the group's bursts: colour, then the depth-stencil ones; issued back to back, collected in order
                S_LD_RD: begin
                    if (!sld_lren) begin
                        if (ld_iss != 2'd1 + nds) begin
                            sld_lren<=1'b1;
                            sld_lraddr<=(ld_iss == 2'd0) ? cbaddr_r(ld_rowc, ld_burst) : dbaddr_r(ld_rowd, ld_burst, ld_iss - 2'd1);
                        end
                    end else if (lr_rdy) begin sld_lren<=1'b0; ld_iss<=ld_iss + 2'd1; end
                    if (lr_rd_valid) begin
                        if (ld_got == 2'd0) ld_data<=lr_rd_data; else if (ld_got == 2'd1) ld_ds0<=lr_rd_data; else ld_ds1<=lr_rd_data;
                        ld_got<=ld_got + 2'd1;
                        if (ld_got + 2'd1 == 2'd1 + nds) begin ld_px<=4'd0; st<=S_LD_WR; end
                    end
                end
                S_LD_WR: begin                            // one cell (pixel pair) per cycle
                    addr_a <= AW'(ld_line*cw) + AW'(((ld_burst << psh) + ld_px) >> 1);
                    wr_a   <= {cur_epoch[rbk], ld_cellv};
                    we_a   <= (ld_x < $signed({2'b00, width}));
                    if (ld_px==pxl - 4'd1) begin
                        ld_iss<=2'd0; ld_got<=2'd0;
                        if (ld_burst==bursts_per_line-1) begin ld_burst<=0;
                            if (ld_line==nl_1) st<=S_RUN;
                            else begin ld_line<=ld_line+1'b1; st<=S_LD_RD;
                                       ld_rowc<=ld_rowc + {11'd0, bursts_per_line}; ld_rowd<=ld_rowd + {11'd0, ds_bpl}; end
                        end else begin ld_burst<=ld_burst+1'b1; st<=S_LD_RD; end
                    end else ld_px<=ld_px+4'd2;
                end
                default: st<=S_IDLE;
            endcase
            // ===== RMW pipeline =====
            v0 <= accept_pipe;
            if (accept_pipe) begin
                touched <= 1'b1;
                s0_addr<=frag_addr; s0_ff<=frag_front; s0_x<=frag_x; s0_y<=frag_y;
                for (int p = 0; p < 2; p++) begin
                    s0_c16[p]<=frag_c16[p*52 +: 52]; s0_sa[p]<=frag_sa[p*26 +: 26];
                    s0_t[p]<=dither_en ? bayer(frag_y[1:0], {frag_x[1], 1'(p)}) : 4'd0;   // (x + p)[1:0] of the even x
                    for (k=0;k<NS;k=k+1) begin s0_fz[p*NS+k]<=frag_z[(p*NS+k)*DW +: DW]; s0_scov[p*NS+k]<=frag_scov[p*NS+k] & pv_eff[p]; end
                end
            end
            v1 <= v0;
            if (v0) begin
                rd_eff_r<=rd_eff; s1_addr<=s0_addr; s1_scov<=s0_scov; s1_ff<=s0_ff;
                for (k=0;k<NL2;k=k+1) s1_fz[k]<=s0_fz[k];
            end
            v2 <= v1;
            if (v1) begin
                s2_addr<=s1_addr;
                for (k=0;k<NL2;k=k+1) begin
                    s2_cw[k]<=cw_s[k]; s2_sten[k]<=new_sten_s[k]; s2_fz[k]<=s1_fz[k];
                    s2_keepz[k]<=rd_eff_r[k*CELLW+32 +: DW]; s2_keepc[k]<=rd_eff_r[k*CELLW +: 32];
                end
            end
            v3 <= v2; v4 <= v3; v5 <= v4; v6 <= v5;
            s3_addr<=s2_addr; s4_addr<=s3_addr; s5_addr<=s4_addr; s6_addr<=s5_addr;
            s3_cw<=s2_cw; s4_cw<=s3_cw; s5_cw<=s4_cw; s6_cw<=s5_cw;
            for (k=0;k<NL2;k=k+1) begin
                s3_sten[k]<=s2_sten[k]; s4_sten[k]<=s3_sten[k]; s5_sten[k]<=s4_sten[k]; s6_sten[k]<=s5_sten[k];
                s3_fz[k]<=s2_fz[k]; s4_fz[k]<=s3_fz[k]; s5_fz[k]<=s4_fz[k]; s6_fz[k]<=s5_fz[k];
                s3_keepz[k]<=s2_keepz[k]; s4_keepz[k]<=s3_keepz[k]; s5_keepz[k]<=s4_keepz[k]; s6_keepz[k]<=s5_keepz[k];
                s3_keepc[k]<=s2_keepc[k]; s4_keepc[k]<=s3_keepc[k]; s5_keepc[k]<=s4_keepc[k]; s6_keepc[k]<=s5_keepc[k];
            end
            // ===== FLUSH (background, bank fbk): packer =====
            // issue: one cell (pixel pair) per cycle; a group's first pair waits for its accumulator (the one the writer
            // freed last). The cell is read the next cycle and packed the cycle after (sh1 -> sh2).
            if (pk_run && (fl_px != 4'd0 || !acc_full[ia])) begin
                fl_addr <= fl_gaddr;
                sh1_v <= 1'b1; sh1_px <= fl_px; sh1_last <= (fl_px == pxl - 4'd1);
                sh1_x <= CW'((fl_burst << psh) + fl_px); sh1_y <= CW'(fl_band_y0 + fl_line);
                sh1_rowc <= fl_rowc; sh1_rowd <= fl_rowd; sh1_burst <= fl_burst; sh1_rep <= grp_rep;
                if (fl_px == pxl - 4'd1) begin                       // the group's last pair: the next group
                    fl_px <= 4'd0; ia <= ~ia;
                    if (grp_last) begin
                        fl_burst <= 11'd0; fl_lbase <= fl_lbase + AW'(cw);
                        fl_rowc <= fl_rowc + {11'd0, bursts_per_line}; fl_rowd <= fl_rowd + {11'd0, ds_bpl};
                        if (fl_line == nl_1) pk_run <= 1'b0; else fl_line <= fl_line + 11'd1;
                    end else fl_burst <= grp_next;
                end else fl_px <= fl_px + 4'd2;
            end else sh1_v <= 1'b0;
            sh2_v <= sh1_v; sh2_px <= sh1_px; sh2_last <= sh1_last; sh2_x <= sh1_x; sh2_y <= sh1_y;
            sh2_rowc <= sh1_rowc; sh2_rowd <= sh1_rowd; sh2_burst <= sh1_burst; sh2_rep <= sh1_rep;
            // pack: the pair into accumulator pa (RGBA8: 64-bit lane sh2_px[2:1]; 16-bit formats: 32-bit lane sh2_px[3:1];
            // depth-stencil: 64-bit lane sh2_px[2:1] of the first or the second burst)
            if (sh2_v) begin
                for (int a = 0; a < 2; a++) if (pa == 1'(a)) begin
                    if (bpp16) begin for (int l = 0; l < 8; l++) if (sh2_px[3:1] == 3'(l)) acc_col[a][l*32 +: 32] <= fl_pair[31:0]; end
                    else       begin for (int l = 0; l < 4; l++) if (sh2_px[2:1] == 2'(l)) acc_col[a][l*64 +: 64] <= fl_pair; end
                    for (int l = 0; l < 4; l++) if (sh2_px[2:1] == 2'(l)) begin
                        if (sh2_px[3]) acc_ds1[a][l*64 +: 64] <= {fl_dswb, fl_dswa}; else acc_ds0[a][l*64 +: 64] <= {fl_dswb, fl_dswa};
                    end
                    if (sh2_last) begin
                        acc_full[a] <= 1'b1; acc_rowc[a] <= sh2_rowc; acc_rowd[a] <= sh2_rowd; acc_burst[a] <= sh2_burst; acc_rep[a] <= sh2_rep;
                    end
                end
                if (sh2_last) pa <= ~pa;
            end
            // ===== FLUSH: writer (the arbiter's ready only ends the wait; the next burst is set up in W_WB) =====
            case (ws)
                W_IDLE: if (acc_full[wa]) begin
                    w_rowc <= acc_rowc[wa]; w_rowd <= acc_rowd[wa]; w_burst <= acc_burst[wa]; w_rep <= acc_rep[wa]; fwi <= 2'd0; ws <= W_WB;
                end
                W_WB: begin                // burst fwi of the group: 0 the colour burst, 1.. the depth-stencil bursts
                    w_en<=1'b1; w_wren<=1'b1; w_wend<=1'b1; w_mask<=32'd0;
                    w_addr<=(fwi == 2'd0) ? cbaddr_r(w_rowc, w_burst) : dbaddr_r(w_rowd, w_burst, fwi - 2'd1);
                    w_data<=(fwi == 2'd0) ? acc_col[wa] : (fwi == 2'd1) ? acc_ds0[wa] : acc_ds1[wa];
                    fl_cmd_ack<=0; fl_wdf_ack<=0; ws<=W_WK;
                end
                W_WK: begin
                    if (w_en   && w_rdy)     begin w_en<=0;   fl_cmd_ack<=1; end
                    if (w_wren && w_wdf_rdy) begin w_wren<=0; w_wend<=0; fl_wdf_ack<=1; end
                    if ((fl_cmd_ack||(w_en&&w_rdy)) && (fl_wdf_ack||(w_wren&&w_wdf_rdy))) begin
                        if (fwi != nds) begin fwi <= fwi + 2'd1; ws <= W_WB; end
                        else if (w_rep != 11'd1) begin w_rep <= w_rep - 11'd1; w_burst <= w_burst + 11'd1; fwi <= 2'd0; ws <= W_WB; end
                        else begin                                   // the group is written: the next one without a gap
                            acc_full[wa] <= 1'b0; wa <= ~wa;
                            if (acc_full[~wa]) begin
                                w_rowc <= acc_rowc[~wa]; w_rowd <= acc_rowd[~wa]; w_burst <= acc_burst[~wa]; w_rep <= acc_rep[~wa];
                                fwi <= 2'd0; ws <= W_WB;
                            end else ws <= W_IDLE;
                        end
                    end
                end
                default: ws <= W_IDLE;
            endcase
            // ===== PRELOAD of the next band (load-modify passes): groups of 1 + nds bursts through a FIFO =====
            if (pl_active && lr_rd_valid) begin plfifo[plf_wp[2:0]] <= lr_rd_data; plf_wp <= plf_wp + 1'b1; end
            if (pl_iss_active) begin
                if (!pl_lren) begin
                    if (pl_out < 4'(PLFD)) begin
                        pl_lren   <= 1'b1;
                        pl_lraddr <= (pl_ik == 2'd0) ? cbaddr_r(pl_rowc, pl_iburst) : dbaddr_r(pl_rowd, pl_iburst, pl_ik - 2'd1);
                    end
                end else if (lr_rdy) begin
                    pl_lren <= 1'b0;
                    if (pl_iss_last) pl_iss_active <= 1'b0;
                    else if (pl_ik != nds) pl_ik <= pl_ik + 2'd1;
                    else begin
                        pl_ik <= 2'd0;
                        if (pl_iburst==bursts_per_line-1) begin pl_iburst<=0; pl_iline<=pl_iline+1'b1;
                                                                pl_rowc<=pl_rowc + {11'd0, bursts_per_line}; pl_rowd<=pl_rowd + {11'd0, ds_bpl}; end
                        else pl_iburst<=pl_iburst+1'b1;
                    end
                end
            end
            pl_out <= pl_out + ((pl_iss_active && pl_lren && lr_rdy) ? 4'd1 : 4'd0)
                             - ((pst==PL_POP && !plf_empty)          ? 4'd1 : 4'd0);
            case (pst)
                PL_IDLE: if (load_modify && next_ok && (st==S_RUN) && ~band_flush_req && ~pf_ok[pbk] && ~ld_active) begin
                    pl_y0 <= pl_y0n;
                    pl_rowc <= 22'({11'd0, pl_y0n} * {11'd0, bursts_per_line}); pl_rowd <= 22'({11'd0, pl_y0n} * {11'd0, ds_bpl});
                    pl_line<=0; pl_burst<=0; pl_iline<=0; pl_iburst<=0; pl_ik<=2'd0; pl_pk<=2'd0;
                    plf_wp<=4'd0; plf_rp<=4'd0; pl_out<=4'd0; pl_active<=1'b1; pl_iss_active<=1'b1; pl_void<=1'b0; pst<=PL_POP;
                end
                PL_POP: if (!plf_empty) begin
                    if (pl_pk == 2'd0) pl_ld_data <= plfifo[plf_rp[2:0]]; else if (pl_pk == 2'd1) pl_ds0 <= plfifo[plf_rp[2:0]];
                    else pl_ds1 <= plfifo[plf_rp[2:0]];
                    plf_rp<=plf_rp+1'b1;
                    if (pl_pk == nds) begin pl_pk<=2'd0; pl_px<=4'd0; pst<=PL_WR; end else pl_pk<=pl_pk+2'd1;
                end
                PL_WR: begin                              // one cell (pixel pair) per cycle
                    pl_addr  <= AW'(pl_line*cw) + AW'(((pl_burst << psh) + pl_px) >> 1);
                    pl_wdata <= {cur_epoch[pbk], pl_cell};
                    pl_we    <= (pl_x < $signed({2'b00, width}));
                    if (pl_px==pxl - 4'd1) begin
                        if (pl_burst==bursts_per_line-1) begin pl_burst<=0;
                            if (pl_line==nl_1) begin pf_ok[pbk]<=!pl_void; pf_y0[pbk]<=pl_y0; pl_active<=1'b0; pst<=PL_IDLE; end
                            else begin pl_line<=pl_line+1'b1; pst<=PL_POP; end
                        end else begin pl_burst<=pl_burst+1'b1; pst<=PL_POP; end
                    end else pl_px<=pl_px+4'd2;
                end
                default: pst<=PL_IDLE;
            endcase
            // a frame starts (after the case: it wins over a preload completing in the same cycle)
            if (pf_clear) begin pf_ok[0]<=1'b0; pf_ok[1]<=1'b0; pf_ok[2]<=1'b0; if (pst!=PL_IDLE) pl_void<=1'b1; end
        end
    end
    // synthesis translate_off
    // a band started with a preload of another band in its bank: only after a frame that ended early, never within one
    int n_pl_drop = 0;
    always @(posedge clk) if (!rst && st==S_IDLE && band_start && !fast && pf_ok[rbk] && pf_y0[rbk]!=band_y0) begin
        n_pl_drop++; $display("b_rop: band y0 %0d starts with a preload of band %0d in its bank (dropped, loaded)", band_y0, pf_y0[rbk]);
    end
    // synthesis translate_on
endmodule

// bb_bank — one band-buffer bank: W bits x DEPTH cells as ceil(W/36) slices of 1k x 36 RAMB36 blocks (DEPTH <= 8k).
// True dual port: port A reads every cycle (the old cell when it writes the same address) and writes; port B reads when
// enabled. Read data is registered in the RAM (one cycle after the address), as in b_rop's inferred arrays; the
// block select is registered alongside, so a bank costs ceil(DEPTH / 1024) x ceil(W / 36) RAMB36 and a small output mux.
module bb_bank #(parameter int W = 72, parameter int AW = 13, parameter int DEPTH = 5120) (
    input  wire          clk,
    input  wire [AW-1:0] a_addr, input wire a_we, input wire [W-1:0] a_wdata, output wire [W-1:0] a_rdata,
    input  wire          b_en, input wire [AW-1:0] b_addr, output wire [W-1:0] b_rdata
);
    localparam int NBLK = (DEPTH + 1023) / 1024;
    localparam int NSL  = (W + 35) / 36;
    localparam int SW   = NSL * 36;
    localparam int BB   = (NBLK > 1) ? $clog2(NBLK) : 1;
    wire [SW-1:0]  a_wd  = SW'(a_wdata);
    wire [AW+10:0] a_ext = {11'd0, a_addr}, b_ext = {11'd0, b_addr};     // so that >> 10 / [9:0] exist for any AW
    wire [BB-1:0]  a_blk = BB'(a_ext >> 10), b_blk = BB'(b_ext >> 10);
    wire [9:0]     a_lo  = a_ext[9:0], b_lo = b_ext[9:0];
    reg  [BB-1:0]  a_blk_q, b_blk_q;
    always @(posedge clk) begin a_blk_q <= a_blk; if (b_en) b_blk_q <= b_blk; end
    wire [SW-1:0] ra [0:NBLK-1], rb [0:NBLK-1];
    genvar k, s;
    generate for (k = 0; k < NBLK; k = k + 1) begin : BLK
        for (s = 0; s < NSL; s = s + 1) begin : SL
            (* ram_style = "block" *) reg [35:0] m [0:1023];
            reg [35:0] qa, qb;
            always @(posedge clk) begin
                if (a_we && a_blk == BB'(k)) m[a_lo] <= a_wd[s*36 +: 36];
                qa <= m[a_lo];
            end
            always @(posedge clk) if (b_en) qb <= m[b_lo];
            assign ra[k][s*36 +: 36] = qa;
            assign rb[k][s*36 +: 36] = qb;
        end
    end endgenerate
    assign a_rdata = W'(ra[a_blk_q]);
    assign b_rdata = W'(rb[b_blk_q]);
endmodule
`default_nettype wire
