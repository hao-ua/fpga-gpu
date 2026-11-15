// a_setup — board A's triangle setup, record and binning stage (model tools/a_setup_ref.py).
//   Takes the vertex stream of a_vertex (3 vertices per triangle: gl_Position + 8 varyings), near-clips (clip_near, w >=
//   2^-7, 0 / 1 / 2 triangles out), runs vtx_setup2 on the 3 vertices (1/w, window x / y in 1/16 pixel, window z),
//   computes the signed area (1/256 pixel^2, 34 bits), facing (area > 0, flipped by front_cw) and the cull, drops
//   degenerate / culled triangles and those entirely outside the render target, then builds the TRIANGLE RECORD:
//     burst 0 (raster header): {0xA, prim, front, s[17:16], fsmask} | tri id | draw id | area >> s | {y0, x0} | {y1, x1} |
//               {y2, x2} | {band hi, lo}
//     words 8..: the TRI item as B consumes it: {2, prim, front, varmask} | id | draw id | R |
//               z0 | dz1 | dz2 | q0 | q1 | q2 | dz/dx | dz/dy | A0 B0 A1 B1 A2 B2 | the depth-bound words w18..w21
//               (zq_tri: the conservative depth of a quad) | per read varying (fsmask,
//               ascending) f0, D1, D2 (320 bits)
//   INTEGER EDGES: the rasterizer sends B integer edge values; s = max(0, bits(|area|) - 30) keeps them in 32 bits
//   (the raster header carries it), R = rcp(i2f(area >>> s)), the per-pixel edge steps A1 = 16 dy2, B1 = -16 dx2,
//   A2 = -16 dy1, B2 = 16 dx1, A0 = -A1 - A2, B0 = -B1 - B2, all >>> s (tools/b_ref.py tri_edges).
//   dz1 = z1 - z0, dz2 = z2 - z0 (FP24, rounded once); dz/dx = (dz1 * fdy2 - dz2 * fdy1) * rcp(area / 16), dz/dy =
//   (dz2 * fdx1 - dz1 * fdx2) * rcp(area / 16) with the 1/16-pixel deltas converted to FP24 (exact) and the area's
//   exponent lowered by 4; D1 = f1 - f0, D2 = f2 - f0 per component. One float_addsub (the differences stream at II = 1),
//   one float_mul, one reciprocal, one integer -> float converter. The record is written to A's DDR at REC_BASE + id *
//   128 words (512 B per id, as B's record store) through a mem_arb write port, and {id, draw id, band range} is pushed
//   to tri_binner; bands are NL rows.
`default_nettype none
module a_setup #(
    parameter int EXP_W = 7, MAN_W = 16,
    parameter int NVEC = 8,
    parameter int NL = 4,                       // rows per band
    parameter int BAND8_MAXW = 1024,            // targets up to this wide: 8-row bands (board B's b_render BAND8_MAXW)
    parameter int BANDW = 10,
    parameter int IDX_W = 20, DID_W = 13,
    parameter [28:0] REC_BASE = 29'h1000000,    // word address of bank 0's record 0 (64 MB)
    parameter [28:0] REC_BANK_W = 29'h2000000,  // words per record bank (128 MB: 2^18 records of 512 B); bank 1 follows bank 0
    parameter int REC_ENTRIES = 262144          // records a bank holds (512 B each)
)(
    input  wire        clk, rst,
    input  wire        frame_start,             // tri id counter -> 0
    input  wire        bank,                    // the record bank of the frame being built (two banks, so the
                                                // next frame's geometry runs while the render side streams the previous one)
    // the vertex stream (a_vertex vo_*)
    input  wire        vo_valid,
    output wire        vo_ready,
    input  wire [4*(EXP_W+MAN_W+1)-1:0] vo_pos,
    input  wire [NVEC*4*(EXP_W+MAN_W+1)-1:0] vo_var,
    input  wire [EXP_W+MAN_W:0] vo_psize,       // gl_PointSize (points)
    input  wire [1:0]  vo_prim,                 // 0 triangle (3 vertices), 1 line (2), 2 point (1)
    // window positions ahead (vtx_asm): the three gl_Position rows of the triangle
    // AFTER the one being presented arrive while the setup still works on the one before, and go straight into
    // vtx_setup2 — its 26 cycles pass before the triangle is in hand. pa_req (pa_req_first: a draw's first triangle) is
    // answered by pa_grant when their results have room (and, for a draw's first, the front is idle and nothing of the
    // draw before is in the unit: the viewport registers change then); the three pa_valid beats that follow are taken
    // unconditionally (a direct issue waits a cycle). vo_la with a triangle's vertices: its positions went ahead.
    input  wire        vo_la = 1'b0,
    input  wire        pa_req = 1'b0, pa_req_first = 1'b0,
    output wire        pa_grant,
    input  wire        pa_valid = 1'b0,
    input  wire [4*(EXP_W+MAN_W+1)-1:0] pa_pos = '0,
    input  wire        pa_first = 1'b0,
    // draw state (of the draw whose primitive is presented; latched per primitive at its first vertex —
    // draws overlap, the caller's state FIFO advances when the assembler's draw_done says a draw's vertices are all taken)
    input  wire [EXP_W+MAN_W:0] sxs_i, sxo_i, sys_i, syo_i,   // viewport in 1/16 pixel
    input  wire [EXP_W+MAN_W:0] zs_i, zo_i,     // depth range: window z = ndc_z * zs + zo
    input  wire [1:0]  cull_mode_i,             // 0 none, 1 cull back, 2 cull front, 3 both
    input  wire        front_cw_i,
    input  wire        zclip0_i = 1'b0,         // the clip volume is 0 <= z <= w, not -w <= z <= w (VS_STATE word 4 bit 24:
                                                // a library that keeps depth reversed sends z' = w - z, far at 0; plane 1 = z >= 0)
    input  wire [6:0]  line_width_i,            // pixels (0 = 1; clamped to MAXLW)
    input  wire [3:0]  zc_i = 4'd0,             // the draw's coarse-depth flags {INV, STRICT, CULL, OCC} (VS_STATE word 4 [28:25]):
                                                // raster header word 2 [15:13] (zc_enc), for a_render's coarse depth
    input  wire [7:0]  fsmask_i,                // the fragment shader's read varyings (the record's varying mask)
    input  wire [DID_W-1:0] draw_id_i,
    input  wire [11:0] scr_w_i, scr_h_i,        // render target in pixels
    input  wire [4*(EXP_W+MAN_W+1)-1:0] gb_i,   // {gyhi, gylo, gxhi, gxlo}: the guard band's NDC bounds (VS_STATE words 9..12)
    // the draw's rectangle [x0, x1) x [y0, y1) in pixels, {y1, x1, y0, x0} (its scissor box cut to the target,
    // the whole target without one — VS_STATE words 13, 14). Nothing outside it can be written (board B's scissor test
    // masks it), so a primitive that has no pixel in it is dropped here and one that crosses it is binned only into the
    // bands the rectangle covers; the rectangle goes into the raster header for a_render's walk.
    input  wire [47:0] sc_i,
    output wire        busy,
    output wire [IDX_W-1:0] tri_count,          // records emitted since frame_start
    output reg         rec_ovf,                 // sticky until the next frame_start: a record beyond REC_ENTRIES was dropped
    // record write master (mem_arb write port): the bursts of a record pipelined, the command never ahead of its data
    output wire        w_req,                   // (only while a burst can go or a command is owed: the binner writes in between)
    output wire        w_en,
    output reg  [28:0] w_addr,                  // (per burst: the record's base + the burst's offset, registered)
    output reg  [255:0] w_data,
    output wire [31:0] w_mask,
    output wire        w_wren, w_wend,
    input  wire        w_rdy, w_wdf_rdy,
    // tri_binner push
    output reg         push_valid,
    input  wire        push_ready,
    output reg  [IDX_W-1:0] push_tri_idx,
    output reg  [DID_W-1:0] push_draw_id,
    output reg  [BANDW-1:0] push_band_lo, push_band_hi,
    output reg  [3:0]  push_tri_nb              // the record's TRI bursts after its header burst (the binner's entry flags)
);
    localparam int W = 1 + EXP_W + MAN_W, TW = 4*W, NVAR = NVEC*4, VWB = NVAR*W, VB = (4 + NVAR)*W;
    // the draw state of the primitive in hand (latched with its first vertex)
    reg [W-1:0] sxs, sxo, sys, syo, zs, zo; reg [1:0] cull_mode; reg front_cw, zclip0; reg [6:0] line_width; reg [7:0] fsmask; reg [DID_W-1:0] draw_id; reg [11:0] scr_w, scr_h;
    reg [4*W-1:0] gb;
    // the coarse-depth flags in three header bits: bit 0 = the draw's failing quads may be culled (CULL or OCC),
    // bit 1 = OCC with bit 0, INV without it, bit 2 = STRICT with bit 0; INV wins (a draw that may raise the depth is never culled)
    function automatic [2:0] zc_enc(input [3:0] z);   // z = {INV, STRICT, CULL, OCC}
        reg c; c = z[1] | z[0];
        zc_enc = z[3] ? 3'b010 : {z[2] & c, z[0], c};
    endfunction
    reg [2:0] zc, b_zc;
    reg [11:0] sc_x0, sc_y0, sc_x1, sc_y1; reg sc_emp;      // the draw's rectangle; empty (no pixel: every primitive of the draw is dropped)
    localparam [W-1:0] HALF = ((((1 << (EXP_W-1)) - 1) - 1) << MAN_W);
    localparam [W-1:0] ONE  = ((((1 << (EXP_W-1)) - 1)) << MAN_W);
    localparam [W-1:0] F8   = ((((1 << (EXP_W-1)) - 1) + 3) << MAN_W);       // 8.0 (the point size in 1/8 pixel: an even 1/16-pixel size)
    localparam [W-1:0] WEPS = ((((1 << (EXP_W-1)) - 1) - 7) << MAN_W);       // 2^-7, the near clip plane (clip_near)
    localparam int MAXPT = 64, MAXLW = 64;                                   // point size / line width limits (pixels)
    assign w_mask = 32'd0;
    // ---- the assembled primitive (clip space) and the clipped ones. A point is vertex 0 copied into 1 and 2, a
    //      line a -> b is vertices 0, 1 with 0 copied into 2, so the difference stream makes dz2 = D2 = 0 (and dz1 =
    //      D1 = 0 for a point) with no extra data path; the kind steers the clip, the checks and the record's words.
    reg [W-1:0] vX [0:2], vY [0:2], vZ [0:2], vW [0:2]; reg [VWB-1:0] vVar [0:2];
    reg [1:0] vcnt, prim; reg [W-1:0] psz;
    // a point's vertices 1 / 2 and a line's vertex 2 ARE vertex 0: they are written WITH vertex 0 (at assembly, and when
    // the line clip moves vertex 0) instead of copied afterwards — a copy would be a write source of its own on 2 400
    // flops; a dynamic vertex select on the readers costs more
    function automatic w_in(input [W-1:0] w); w_in = ~w[W-1] & (w[W-2:0] >= WEPS[W-2:0]); endfunction
    function automatic [W-1:0] attr(input [1:0] v, input [5:0] k);          // attribute k of vertex v: x y z w, then the varyings
        attr = (k == 6'd0) ? vX[v] : (k == 6'd1) ? vY[v] : (k == 6'd2) ? vZ[v] : (k == 6'd3) ? vW[v] : vVar[v][(k - 6'd4)*W +: W];
    endfunction
    reg [5:0] lc_k; reg lc_out;                                            // the line clip: attribute index, the outside vertex
    reg [3+NVAR:0] lc_oh;                                                  // one-hot of lc_k, a cycle late (lc_k stands >= 3 cycles before S_LC8 writes with it)
    always @(posedge clk) lc_oh <= (4+NVAR)'(1) << lc_k;
    reg [2:0] lc_p;                                                        // the clip plane being tested (points: 0..2; lines: 0..6 with the guard band)
    reg       lc_v;                                                        // the line end whose guard-plane product is in flight
    // the guard planes 3..6: d = g_hi w - x / y (3, 5), x / y - g_lo w (4, 6), with g from gb = {gyhi, gylo, gxhi, gxlo}
    function automatic [W-1:0] gconst(input [2:0] p); gconst = (p == 3'd3) ? gb[1*W +: W] : (p == 3'd4) ? gb[0 +: W] : (p == 3'd5) ? gb[3*W +: W] : gb[2*W +: W]; endfunction
    function automatic [W-1:0] gattr(input [1:0] v, input [2:0] p); gattr = (p == 3'd3 || p == 3'd4) ? vX[v] : vY[v]; endfunction
    reg [W-1:0] lc_t, lc_da, lc_din;                                       // t, d of vertex 0, d of the inside vertex
    reg ps_conv;                                                           // vtx_setup2 converts the point size (x = size, scale 16)
    wire [6:0] lw_c = (line_width == 7'd0) ? 7'd1 : (line_width > 7'(MAXLW)) ? 7'(MAXLW) : line_width;
    // the clip volume's planes (clip_near, a_setup_ref plane_d): the signed distance of vertex v, inside when its sign is clear
    function automatic [W-1:0] pd_a(input [1:0] v, input [2:0] p); pd_a = (p == 3'd1) ? vZ[v] : vW[v]; endfunction   // a - b or a + b
    function automatic [W-1:0] pd_b(input [1:0] v, input [2:0] p); pd_b = (p == 3'd0) ? WEPS : (p == 3'd1) ? (zclip0 ? '0 : vW[v]) : vZ[v]; endfunction
    function automatic pd_sub(input [2:0] p); pd_sub = (p != 3'd1); endfunction   // 0: w - eps, 1: z + w (zclip0: z + 0), 2: w - z
    function automatic aen(input [5:0] k); aen = (k < 6'd4) ? 1'b1 : fsmask[(k - 6'd4) >> 2]; endfunction   // attribute k is used
    reg clip_start, clip_ack; wire clip_busy, clip_tv, clip_done, clip_pass; wire [3*VB-1:0] clip_to;
    wire [VB-1:0] cv0 = {vVar[0], vW[0], vZ[0], vY[0], vX[0]}, cv1 = {vVar[1], vW[1], vZ[1], vY[1], vX[1]}, cv2 = {vVar[2], vW[2], vZ[2], vY[2], vX[2]};
    clip_near #(.EXP_W(EXP_W), .MAN_W(MAN_W), .NVAR(NVAR)) u_clip (.clk, .rst, .start(clip_start), .verts_in({cv2, cv1, cv0}), .vmask(fsmask), .gb(gb), .z0(zclip0),
        .busy(clip_busy), .tri_valid(clip_tv), .tri_ack(clip_ack), .tri_out(clip_to), .pass(clip_pass), .done(clip_done));
    // ---- per-vertex setup
    reg sv_in; reg [W-1:0] svx, svy, svz, svw; wire sv_out; wire signed [15:0] s_sx, s_sy; wire [W-1:0] s_zw, s_iw; wire s_rej;
    // the point size goes through the same unit as a pseudo vertex (x = size, w = 1, scale 16, offset 0): sx = RNE(16 size)
    wire [W-1:0] vs_sxs = ps_conv ? F8 : sxs, vs_sxo = ps_conv ? '0 : sxo;
    wire signed [17:0] s_sx18, s_sy18;
    vtx_setup2 #(.EXP_W(EXP_W), .MAN_W(MAN_W)) u_set (.clk, .rst, .in_valid(sv_in), .x(svx), .y(svy), .z(svz), .w(svw),
        .sxs(vs_sxs), .sxo(vs_sxo), .sys, .syo, .zs, .zo, .out_valid(sv_out), .sx(s_sx), .sy(s_sy), .sx18(s_sx18), .sy18(s_sy18), .zw(s_zw), .iw_out(s_iw), .reject(s_rej));
    reg signed [15:0] SX [0:2], SY [0:2]; reg [W-1:0] ZW [0:2], IW [0:2]; reg rej_any; reg [1:0] vert, svcol;
    reg signed [17:0] SX18, SY18;                                          // vertex 0's window position before the 16-bit saturation (a point's centre)
    reg [10:0] ps_u;                                                       // the point size in 1/16 pixel, 16 .. 16 MAXPT
    // ---- triangle quantities
    wire signed [16:0] dx1 = SX[1] - SX[0], dy1 = SY[1] - SY[0], dx2 = SX[2] - SX[0], dy2 = SY[2] - SY[0];
    reg signed [33:0] m_a, m_b, a2, l_a, l_b, len2;
    always @(posedge clk) begin m_a <= dx1 * dy2; m_b <= dx2 * dy1; a2 <= m_a - m_b; l_a <= dx1 * dx1; l_b <= dy1 * dy1; len2 <= l_a + l_b; end
    wire front = (~a2[33] & (a2 != 0)) ^ front_cw;
    wire culled = (a2 == 0) | (cull_mode == 2'd1 & ~front) | (cull_mode == 2'd2 & front) | (cull_mode == 2'd3);   // 3 = FRONT_AND_BACK (ES 2.0 §3.5.1)
    // ---- the FRONT / BACK split: the state machine above
    //      S_CHK2 (collect, clip, per-vertex setup, area / cull) is the FRONT; the rest (the depth planes, the record, the push)
    //      is the BACK, a second state machine (bst) fed by this one-entry snapshot when the front accepts a primitive and the
    //      back is idle. The back derives everything else (deltas, shift, steps, band) from its copy; the front works on the
    //      next primitive meanwhile. The difference stream stays in the front (it starts at S_SETF from the live vertices), but
    //      its results land in a record buffer and dz pair of their own (two sets, the stream's and the back's).
    reg signed [15:0] bSX [0:2], bSY [0:2]; reg [W-1:0] bZW [0:2], bIW [0:2]; reg signed [17:0] bSX18, bSY18;
    reg signed [33:0] b_a2, b_len2; reg [10:0] b_ps_u; reg [1:0] b_prim; reg b_fcw; reg [6:0] b_lw; reg [7:0] b_fsmask; reg [DID_W-1:0] b_did;
    reg [11:0] b_row_lo, b_row_hi;
    reg [47:0] b_sc;                                           // the rectangle of the primitive in the back, {y1, x1, y0, x0} (the header's)
    reg rej_r, ovf_r;                                          // the front's verdict on the primitive in hand (S_CHK2 -> S_CHK3)
    wire signed [16:0] b_dx1 = bSX[1] - bSX[0], b_dy1 = bSY[1] - bSY[0], b_dx2 = bSX[2] - bSX[0], b_dy2 = bSY[2] - bSY[0];
    wire b_front = (~b_a2[33] & (b_a2 != 0)) ^ b_fcw;
    wire [6:0] b_lw_c = (b_lw == 7'd0) ? 7'd1 : (b_lw > 7'(MAXLW)) ? 7'(MAXLW) : b_lw;
    // the shift that keeps |key >> s| < 2^30 (|key| < 2^33 here), the shifted key, the per-pixel edge steps >>> s;
    // the key is the area (triangles) or the squared length (lines); a point's is its size (no shift). Both arms of a2s
    // must be signed: an unsigned point arm makes the whole ?: unsigned and the >>> a logical shift, so a negative area
    // with s > 0 would lose its sign in bits 33:32 (R positive and ~2^-34: wrong depth on huge triangles)
    // (the BACK's: from its snapshot)
    wire signed [33:0] keyv = (b_prim == 2'd1) ? b_len2 : b_a2;
    wire signed [33:0] a2m = keyv[33] ? -keyv : keyv;
    wire [1:0] esh = (b_prim == 2'd2) ? 2'd0 : a2m[32] ? 2'd3 : a2m[31] ? 2'd2 : a2m[30] ? 2'd1 : 2'd0;
    wire signed [33:0] a2s = (b_prim == 2'd2) ? $signed(34'(b_ps_u)) : keyv >>> esh;
    // lines: e1 = (c - a) . (b - a): steps A1 = 16 dx, B1 = 16 dy, A0 / B0 their negations, e2 = 0
    wire signed [31:0] lA1 = 32'($signed({b_dx1, 4'd0})) >>> esh, lB1 = 32'($signed({b_dy1, 4'd0})) >>> esh,
                       lA0 = 32'(-$signed({b_dx1, 4'd0})) >>> esh, lB0 = 32'(-$signed({b_dy1, 4'd0})) >>> esh;
    // checks: a point's square [xl, xl + ps) x [yb, yb + ps) outside the target; a line's bounding box with a margin
    wire signed [18:0] xl = 19'(SX18) - 19'(ps_u >> 1), yb = 19'(SY18) - 19'(ps_u >> 1);   // the square's left / bottom edge (the 18-bit centre)
    // (every bound is the draw's rectangle, sc_*: the target itself when the draw has no scissor box)
    wire pt_off = ($signed(xl + 19'(ps_u)) <= $signed({3'b000, sc_x0, 4'd0})) | (xl >= $signed({3'b000, sc_x1, 4'd0})) |
                  ($signed(yb + 19'(ps_u)) <= $signed({3'b000, sc_y0, 4'd0})) | (yb >= $signed({3'b000, sc_y1, 4'd0}));
    wire signed [16:0] lmar = 17'({lw_c, 4'd0});
    wire signed [15:0] lxmin = (SX[0] < SX[1]) ? SX[0] : SX[1], lxmax = (SX[0] < SX[1]) ? SX[1] : SX[0];
    wire signed [15:0] lymin = (SY[0] < SY[1]) ? SY[0] : SY[1], lymax = (SY[0] < SY[1]) ? SY[1] : SY[0];
    wire signed [17:0] lmar18 = 18'(lmar);
    wire ln_off = (18'(lxmax) < $signed({2'b00, sc_x0, 4'd0}) - lmar18) | (18'(lxmin) >= $signed({2'b00, sc_x1, 4'd0}) + lmar18) |
                  (18'(lymax) < $signed({2'b00, sc_y0, 4'd0}) - lmar18) | (18'(lymin) >= $signed({2'b00, sc_y1, 4'd0}) + lmar18);
    wire signed [22:0] eA1 = $signed({b_dy2, 4'd0}), eB1 = -$signed({b_dx2, 4'd0}), eA2 = -$signed({b_dy1, 4'd0}), eB2 = $signed({b_dx1, 4'd0});   // (the back's)
    wire signed [23:0] eA0 = -24'(eA1) - 24'(eA2), eB0 = -24'(eB1) - 24'(eB2);
    wire signed [31:0] sA0 = 32'(eA0) >>> esh, sB0 = 32'(eB0) >>> esh, sA1 = 32'(eA1) >>> esh, sB1 = 32'(eB1) >>> esh,
                       sA2 = 32'(eA2) >>> esh, sB2 = 32'(eB2) >>> esh;
    function automatic signed [15:0] min3(input signed [15:0] a, b, c); min3 = (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c); endfunction
    function automatic signed [15:0] max3(input signed [15:0] a, b, c); max3 = (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c); endfunction
    wire signed [15:0] xmin = min3(SX[0], SX[1], SX[2]), xmax = max3(SX[0], SX[1], SX[2]);
    wire signed [15:0] ymin = min3(SY[0], SY[1], SY[2]), ymax = max3(SY[0], SY[1], SY[2]);
    wire offscreen = ($signed({xmax[15], xmax}) < $signed({1'b0, sc_x0, 4'd0})) | ($signed({xmin[15], xmin}) >= $signed({1'b0, sc_x1, 4'd0})) |
                     ($signed({ymax[15], ymax}) < $signed({1'b0, sc_y0, 4'd0})) | ($signed({ymin[15], ymin}) >= $signed({1'b0, sc_y1, 4'd0}));
    function automatic [11:0] clampy(input signed [17:0] yq, input [11:0] lo, input [11:0] hi);   // pixel row of a 1/16 coordinate, clamped to lo .. hi - 1 (the rectangle's rows)
        reg signed [17:0] r; r = yq >>> 4;
        clampy = (r < $signed({6'd0, lo})) ? lo : (r >= $signed({6'd0, hi})) ? hi - 1'b1 : r[11:0];
    endfunction
    // the rows a primitive can touch: a triangle's y range; a point's square; a line's y range widened by the width.
    // Two register stages after the window positions settle (S_CHK: the extremes; S_CHK1: the clamped rows) — the
    // one-cycle chain into the record words misses timing; the values are read from the hand-off on.
    reg signed [15:0] ymin_r, ymax_r, lymin_r, lymax_r, xmin_r, xmax_r; reg signed [18:0] yb_r;
    reg [11:0] row_lo, row_hi;
    wire [11:0] lrow_hi0 = clampy(18'(lymax_r), sc_y0, sc_y1);
    wire [12:0] lrow_hi1 = 13'(lrow_hi0) + 13'(lw_c) - 13'd1;
    // A triangle: the pixel rows / columns whose CENTRE (16 c + 8) lies in its
    // bounding box — c_lo = (min + 7) >> 4, c_hi = (max - 8) >> 4. Only those can hold a covered pixel, so the bands are
    // those of the centre rows (the rows containing ymin / ymax would give visits without a quad), and a
    // triangle with no centre row or column, or none inside the target, is dropped (nocen, registered in S_CHK1; it
    // contains the off-screen test). a_setup_ref tri_setup has the same rule.
    wire signed [12:0] cx_lo = 13'((17'(xmin_r) + 17'sd7) >>> 4), cx_hi = 13'((17'(xmax_r) - 17'sd8) >>> 4);
    wire signed [12:0] cy_lo = 13'((17'(ymin_r) + 17'sd7) >>> 4), cy_hi = 13'((17'(ymax_r) - 17'sd8) >>> 4);
    wire nocen_c = (cx_lo > cx_hi) | (cy_lo > cy_hi) | (cx_hi < $signed({1'b0, sc_x0})) | (cy_hi < $signed({1'b0, sc_y0})) |
                   (cx_lo >= $signed({1'b0, sc_x1})) | (cy_lo >= $signed({1'b0, sc_y1}));
    reg  nocen;
    wire [11:0] row_lo_c = (prim == 2'd2) ? clampy(18'(yb_r), sc_y0, sc_y1) : (prim == 2'd1) ? clampy(18'(lymin_r) - 18'({lw_c, 3'd0}) + 18'd8, sc_y0, sc_y1) :
                           clampy(18'(ymin_r) + 18'sd7, sc_y0, sc_y1);
    wire [11:0] row_hi_c = (prim == 2'd2) ? clampy(18'(yb_r) + 18'(ps_u) - 18'd1, sc_y0, sc_y1) :
                           (prim == 2'd1) ? ((lrow_hi1 >= 13'(sc_y1)) ? sc_y1 - 1'b1 : lrow_hi1[11:0]) : clampy(18'(ymax_r) - 18'sd8, sc_y0, sc_y1);
    // bands of 8 rows for targets up to BAND8_MAXW = 1024 wide, else NL = 4 (board B's band buffer holds 8 lines of
    // such a target; a_render walks with the same rule)
    wire [1:0] band_sh = (scr_w <= 12'(BAND8_MAXW)) ? 2'd3 : 2'd2;
    wire [BANDW-1:0] band_lo = BANDW'(b_row_lo >> band_sh), band_hi = BANDW'(b_row_hi >> band_sh);   // (the back's; the registers: after the state machine's declaration)
    // ---- shared float units
    reg as_in, as_op; reg [W-1:0] as_a, as_b; wire as_v; wire [W-1:0] as_r;
    float_addsub #(EXP_W, MAN_W) u_as (.clk, .rst, .in_valid(as_in), .a(as_a), .b(as_b), .op(as_op), .out_valid(as_v), .result(as_r));
    reg mu_in; reg [W-1:0] mu_a, mu_b; wire mu_v; wire [W-1:0] mu_r;
    float_mul #(EXP_W, MAN_W) u_mu (.clk, .rst, .in_valid(mu_in), .a(mu_a), .b(mu_b), .in_ready(), .out_valid(mu_v), .result(mu_r));
    reg rc_in; reg [W-1:0] rc_x; wire rc_v; wire [W-1:0] rc_r;
    // (both units below are used through their out_valid — the chain's collector and S_LC2 / S_LC4 wait for rc_v / cv_v — so
    //  their split stages are latency-transparent here: rc latency 12, cv 3)
    float_sfu #(.EXP_W(EXP_W), .MAN_W(MAN_W), .K(12), .KLOG(11), .SPLIT1B(1'b1), .SPLIT6(1'b1),
        .RCP_MEM("tools/gen/fp24_rcp_seed.mem"), .RSQRT_MEM("tools/gen/fp24_rsqrt_seed.mem"),
        .LOG2_MEM("tools/gen/fp24_log2_lut.mem"), .EXP2_MEM("tools/gen/fp24_exp2_lut.mem"))
      u_rc (.clk, .rst, .in_valid(rc_in), .op(3'd0), .x(rc_x), .out_valid(rc_v), .result(rc_r));
    reg cv_in; reg signed [33:0] cv_i; wire cv_v; wire [W-1:0] cv_r;
    float_cvt_i2f #(.IW(34), .EXP_W(EXP_W), .MAN_W(MAN_W), .SPLIT(1'b1)) u_cv (.clk, .rst, .in_valid(cv_in), .ival(cv_i), .out_valid(cv_v), .result(cv_r));
    // ---- the depth-bound words w18..w21 (zq_tri; board B's early depth test), started when R, z0, dz1, dz2
    //      are known (leaving S_SUB) and done long before the record's header goes out (S_MUL1 .. S_MUL2 take longer)
    reg zq_go, zq_rdy, zq_pend; wire zq_done; wire [127:0] zq_w;
    // the two record buffers' selectors (the split, see the record below): s_buf = the stream's, bbuf = the back's, sbuf = the
    // next stream's; the stream's dz1 / dz2 per buffer; strm_ok[b]: buffer b's stream has landed everything
    reg s_buf, sbuf, bbuf; reg [W-1:0] dzq1 [0:1], dzq2 [0:1]; reg [1:0] strm_ok;
    zq_tri u_zq (.clk, .rst, .start(zq_go), .R(r_R), .z0(bZW[0]), .dz1(dzq1[bbuf]), .dz2(dzq2[bbuf]),
        .A1((b_prim == 2'd2) ? 32'd16 : sA1), .B1((b_prim == 2'd2) ? 32'd0 : sB1),
        .A2((b_prim == 2'd2) ? 32'd0 : sA2), .B2((b_prim == 2'd2) ? -32'sd16 : sB2), .line(b_prim == 2'd1), .done(zq_done), .words(zq_w));
    // ---- the record (110 words at most: 30 + 10 per read varying; 14 bursts) in BLOCK RAM: 256 bits x 32 rows =
    //      two buffers x 16 bursts (the split: the stream lands its blocks in s_buf, the buffer its primitive was started in;
    //      the back writes the header rows and reads the bursts of bbuf; sbuf is the buffer the next stream takes), eight
    //      32-bit columns with write enables of their own. ONE write port, shared by a scheduler below: a varying block (10
    //      words at 30 + 10 k: two rows) goes first, the header (rows 0 .. 3 of the back's buffer; row 3's columns 6 / 7 are
    //      block 0's first words and stay) follows a row per cycle; the slopes (words 18 / 19) ride in registers until then.
    //      The read port feeds w_data a row per beat. (In flip-flops with per-word writes the two buffers would cost
    //      ≈ 4 k FF + 6 k LUT.)
    (* ram_style = "block" *) reg [255:0] rmem [0:31];
    reg blkq_v = 1'b0, blkq_ph = 1'b0, blkq_buf = 1'b0; reg [3:0] blkq_k = 4'd0; reg [319:0] blkq_d;   // the block queue: one block, two rows
    reg [3:0] hw_pend = 4'd0;                                                                         // the header writer: rows still to write
    reg bk_go, b_slp_ok;                                                                              // the hand-off pulse; the slopes have landed
    reg push_sent;                                                                                    // the back's binner push has been taken
    reg [W-1:0] b_dzdx, b_dzdy;                                                                       // dz/dx, dz/dy (words 18 / 19)
    reg [IDX_W-1:0] tid; assign tri_count = tid;
    reg [W-1:0] fd [0:3], fa2, r_a2, r_R, mm [0:3], nx, ny;
    typedef enum logic [3:0] {B_IDLE, B_SUB, B_MUL1, B_SUB2, B_MUL2, B_HDR, B_WR, B_PUSH} bst_t;
    bst_t bst;
    wire bk_busy = (bst != B_IDLE);
    // the shared adder's results in issue order (as_kind rides with as_in): 0 = the stream's varying differences (or a
    // point / line clip's, which run with the back idle), 1 = the back's S_SUB2, 2 = the stream's dz1 / dz2
    reg [1:0] as_kind; reg [1:0] akq [0:7]; reg [2:0] akq_wp, akq_rp;
    wire [1:0] ak_head = akq[akq_rp];
    reg [3:0] nv; reg [7:0] srem, crem; reg [2:0] sph, cph; reg [1:0] sc, cc; reg [3:0] cblk, sblk;   // sub stream: issue / collect walks
    reg [1:0] dz_i, dzc;                                    // dz1 / dz2 issued / collected (2 = both)
    reg [1:0] dz_ok;                                        // per record buffer: its dz1 / dz2 have landed (strm_ok: its varying blocks)
    reg [W-1:0] d1 [0:3], d2 [0:3];
    function automatic [2:0] lsb8(input [7:0] m); lsb8 = 3'd7; for (int k = 7; k >= 0; k--) if (m[k]) lsb8 = 3'(k); endfunction
    wire [2:0] s_i = lsb8(srem), c_i = lsb8(crem);
    wire [TW-1:0] f0v = vVar[0][c_i*TW +: TW];
    // issue side (from the triangle's start, beside vtx_setup2): the varyings — phase 2 = D1
    // of the current bit (comps 0..3), 3 = D2, then the next bit; 4 = all issued — and dz1 / dz2, which go as soon as the
    // window z exists (zw_ok), AHEAD of the varyings still to come (the back's depth planes, the depth
    // bound and header row 1 wait for them, the varying blocks only for their record bursts). The collect side takes
    // each result by its kind; a varying block is written when D2.w lands. A triangle rejected after its vertex setup
    // leaves results in flight: drop_n counts them, the collector drops them (s_fly: issued, not yet collected); the
    // point and line paths, which share the adder, start only with none left.
    reg s_run, zw_ok; reg [5:0] s_fly, drop_n;
    reg [2:0] k_i, k_c;                                     // the back's mul / sub issue + collect counters
    wire b_sub2_iss = (bst == B_SUB2) && (k_i < 3'd2);     // the back's subtraction goes first; the stream waits a cycle
    wire s_dz   = s_run && zw_ok && (dz_i != 2'd2);         // a dz difference is next
    wire s_var  = s_run && (sph == 3'd2 || sph == 3'd3) && !s_dz;
    wire s_iss  = (s_dz || s_var) && !b_sub2_iss;
    wire s_drop = as_v && (ak_head != 2'd1) && (drop_n != 6'd0);
    wire s_col  = as_v && (ak_head == 2'd0) && (drop_n == 6'd0) && (cph != 3'd4);   // a varying difference lands
    wire s_cdz  = as_v && (ak_head == 2'd2) && (drop_n == 6'd0) && (dzc != 2'd2);   // dz1 / dz2 lands (dzc: only while two are
                                                                                    // owed, as s_col's cph: nothing after a reset counts)
    wire [5:0] s_fly_n = s_fly + 6'(s_iss) - 6'(s_col | s_cdz);
    wire s_done = (cph == 3'd4) && (dzc == 2'd2);           // the stream has collected everything (or none runs)
    reg [4:0] nb, bc, bd;                                    // bursts to write, commands accepted, data beats accepted
    // the record write runs beside the state machine:
    // started when the triangle is handed to the back (S_CHK3), it writes the varying-block bursts 4 .. nb-1 as soon as
    // the difference stream has landed them (blk_ok), then the header bursts; the i-th burst written is wseq(i): 4, 5, ..,
    // nb-1, then the header rows 0, 1, 3, 2. B_WR only waits for it to finish.
    // A header burst goes out as soon as ITS row has landed (hrow_ok: the row was
    // written two cycles ago, so w_data has read it), not when all four have —
    // rows 0 and 1 leave during the depth planes and one burst is left when the last row lands. The header bursts' order
    // is 0, 1, 3, 2: row 3 (the depth bound) is usually ready before row 2 (the slopes).
    reg w_run, w_arm, blk_ok; reg [3:0] hrow_ok; reg [28:0] w_base;   // w_arm: w_run a cycle old — w_data has loaded the first burst, blk_ok is the new buffer's
    function automatic [3:0] wseq(input [4:0] i, input [4:0] n);
        reg [4:0] k; k = i - (n - 5'd4);
        wseq = (i < n - 5'd4) ? 4'(i + 5'd4) : (k == 5'd2) ? 4'd3 : (k == 5'd3) ? 4'd2 : 4'(k);
    endfunction
    wire [3:0] w_bd_b = wseq(bd, nb);
    wire w_bd_ok = w_arm && ((w_bd_b >= 4'd4) ? blk_ok : hrow_ok[w_bd_b[1:0]]);   // the next data beat's burst has landed (and is in w_data)
    // The converter (i2f) and the reciprocal. The four deltas fdx1 fdy1 fdx2
    // fdy2 are converted by the FRONT, from the cycle the third window position lands (sx_last: dx1 / dy1 exist then, dx2 /
    // dy2 a cycle later), so they are in fd when the back starts and its depth planes begin at once (issued after the
    // hand-off, behind the two reciprocals' inputs, they would cost every back 11 cycles). The converter's results come
    // back in issue order; cvq holds what each is: 0..3 = fd[k], 4 = the area, 5 = the key >>> s. The front converts only
    // while the back no longer reads fd (fd_free: its four products are issued) and the chain is not issuing; the
    // hand-off waits for the four issues (fc_i == 4), the back's B_SUB for the last result (fd_ok).
    // The reciprocal chain — R = rcp(i2f(key >>> s)) first (it gates the depth bound's 12 cycles and header row 1), then
    // rcp(a2 / 16) for the slopes — is started by the FRONT too, in S_CHK3 once the primitive is accepted (ch_st): at the
    // hand-off when the back is idle, and otherwise while the front waits for the back, as soon as the back has used its
    // own r_a2 / r_R (rr_free: the slopes are issued, header row 1 is written, the depth bound has started), so the next
    // back finds them landed or well on the way (ch_R: R is in, ch_done: both). Its two inputs are the front's area and
    // key (prim / a2 / len2 / the point size stand in the front's registers for the two issue cycles after a hand-off:
    // the next primitive's first vertex is taken later than that).
    reg ch_run, ch_done, ch_R, ch_st; reg [1:0] ch_i;
    reg [2:0] fc_i; reg fd_ok;
    reg [2:0] cv_kind; reg [2:0] cvq [0:3]; reg [1:0] cvq_wp, cvq_rp; reg [2:0] cvq_n;
    typedef enum logic [5:0] {S_ASM, S_CLIPF, S_CLIPW, S_SETF, S_SETW, S_CHK, S_CHK1, S_CHK2, S_CHK3, S_NEXT,
                              S_PT0, S_PTCW, S_PTS1, S_PTS2, S_LC0, S_LCP0, S_LCG, S_LCP1, S_LCP2, S_LC1, S_LC2, S_LC4, S_LC5, S_LC6, S_LC7, S_LC8, S_LCD,
                              S_LAW} st_t;
    st_t st;
    wire rr_free = !bk_busy || ((bst == B_WR) && !hw_pend[1] && !zq_pend);
    wire ch_go   = (st == S_CHK3) && !(rej_r | ovf_r) && !ch_st && !ch_run && rr_free;   // the accepted primitive's chain starts
    // the front hands its primitive to the back at this clock edge: its deltas are issued, its chain is started or starts
    // now (ch_st || ch_go — with the back idle rr_free holds, so that is ch_st || !ch_run: registers only, the term
    // drives the snapshot's clock enables)
    wire hand = (st == S_CHK3) && !(rej_r | ovf_r) && !bk_busy && (fc_i == 3'd4) && (ch_st || !ch_run);
    // ---- the window positions ahead. vtx_setup2's results come out in issue order: svk says which are positions ahead
    //      (they go into the result queue laq: {reject, 1/w, window z, y, x}, 8 entries = two triangles and room) and
    //      which were issued by the state machine (S_SETF: clipped triangles, lines, points; S_PTS1). la_out counts the
    //      positions ahead from their beat until the state machine has taken (S_LAW) or discarded them (la_drop: the
    //      three of a triangle that turned out to need the clipper — its pieces are set up directly — or was clipped
    //      away); la_pend: the triangle in hand still has its three.
    reg  sv_kind; reg [15:0] svk; reg [3:0] svk_wp, svk_rp;
    wire svk_ne = (svk_wp != svk_rp);            // a result is owed (a valid out of the unit after a reset, with none owed, is not one)
    wire sv_la_out = sv_out && svk_ne && svk[svk_rp], sv_dir_out = sv_out && svk_ne && !svk[svk_rp];
    (* ram_style = "distributed" *) reg [80:0] laq [0:7];
    reg  [2:0] la_wp, la_rp; reg [3:0] la_cnt, la_out, la_drop; reg la_pend;
    wire [80:0] la_head = laq[la_rp];
    wire la_have = (la_cnt != 4'd0);
    wire la_dpop = la_have && (la_drop != 4'd0);
    wire la_take = (st == S_LAW) && la_have && (la_drop == 4'd0) && (vert != 2'd0 || s_done);   // (a new stream: as S_SETF)
    // nothing enters the stage for 63 cycles after a reset (rst_ok gates vo_ready and pa_grant): the float units' valid
    // pipelines are not all reset, and a valid of theirs from before the reset must not be counted as the result of an
    // operation issued after it (the result-kind queues above pair issues and results by count)
    reg  [5:0] rfl; wire rst_ok = &rfl;
    always @(posedge clk) if (rst) rfl <= 6'd0; else if (!rst_ok) rfl <= rfl + 6'd1;
    assign pa_grant = rst_ok && (la_out <= 4'd3) && (!pa_req_first || ((st == S_ASM) && (vcnt == 2'd0) && (la_out == 4'd0)));
    always @(posedge clk) if (sv_la_out) laq[la_wp] <= {s_rej, s_iw, s_zw, s_sy, s_sx};
    // the primitive's third window position lands (SX[0], SX[1] are in)
    wire sx_last = ((st == S_SETW) && sv_dir_out && (svcol == 2'd2)) || (la_take && (vert == 2'd2));
    wire fd_free = !((bst == B_SUB) || (bst == B_MUL1 && k_i < 3'd4));
    wire fc_can  = fd_free && !(ch_run && ch_i < 2'd2);
    // the front's key >>> s (as the back's keyv / esh / a2s above, from the front's registers): the chain's R input
    wire signed [33:0] f_keyv = (prim == 2'd1) ? len2 : a2;
    wire signed [33:0] f_a2m  = f_keyv[33] ? -f_keyv : f_keyv;
    wire [1:0]         f_esh  = (prim == 2'd2) ? 2'd0 : f_a2m[32] ? 2'd3 : f_a2m[31] ? 2'd2 : f_a2m[30] ? 2'd1 : 2'd0;
    wire signed [33:0] f_a2s  = (prim == 2'd2) ? $signed(34'(ps_u)) : f_keyv >>> f_esh;
    // a new vertex overwrites vVar: only once the previous primitive's stream has collected everything (its last block reads vVar[0])
    assign vo_ready = (st == S_ASM) && s_done && rst_ok;
    assign busy = (st != S_ASM) | (vcnt != 2'd0) | bk_busy | !s_done | (drop_n != 6'd0) | (la_out != 4'd0) | pa_valid | la_pend;
    assign w_en   = w_run && (bc < nb) && ((bc < bd) || (bc == bd && w_wren));   // a command only once its data is in (or goes now)
    assign w_wren = w_run && (bd < nb) && (bd <= bc + 5'd2) && w_bd_ok;   // data at most two bursts ahead of the commands
    assign w_wend = w_wren;
    // The write port is requested only while a beat can go or a command is owed (held from the hand-off to the record's
    // last burst, it would make the binner's writes of the primitive before wait for the whole record). mem_arb changes
    // the owner at a clock edge where the owner's request is low: in such a cycle w_wren and w_en are low as well (no
    // beat ready, commands and data level), so no transfer is cut in two.
    assign w_req  = w_run && (((bd < nb) && w_bd_ok) || (bc != bd));
    // the band range's two register stages (declared with the row wires above)
    always @(posedge clk) begin
        if (st == S_CHK) begin ymin_r <= ymin; ymax_r <= ymax; xmin_r <= xmin; xmax_r <= xmax; lymin_r <= lymin; lymax_r <= lymax; yb_r <= yb; end
        if (st == S_CHK1) begin row_lo <= row_lo_c; row_hi <= row_hi_c; nocen <= nocen_c; end
    end
    // ---- the record memory's writes and read, in a block of their own without the reset term (timing at 120 MHz:
    //      a reset term in every buffer flop's clock enable is a long route from the reset replica; the rows are
    //      rewritten in full before every record, so a stray write after a reset is harmless; the queue / writer flags
    //      power up clear).
    wire rw_blk = s_col && (cph == 3'd3) && (cc == 2'd3);                // the collector lands a block {0, D2, D1, f0} (slot cblk, buffer s_buf)
    // the header rows go out as their words become known: row 0 (words 0-7) at the hand-off, row 1 (8-15) once R and dz1 / dz2
    // are in, row 2 (16-23) once the slopes are, row 3 (24-29) once the depth bound is
    wire [3:0] hw_ok = {zq_rdy, b_slp_ok, ch_R && dz_ok[bbuf], 1'b1};
    wire [3:0] hw_can = hw_pend & hw_ok;
    wire [6:0] blk_o = 7'd30 + 7'(blkq_k) * 7'd10;                      // the queued block's first word; its two rows: blk_o >> 3, + 1
    wire [4:0] rd_addr = {bbuf, (w_wren && w_wdf_rdy) ? wseq(bd + 5'd1, nb) : wseq(bd, nb)};   // the beat's row (the next one as a beat goes)
    function automatic [31:0] hdr_word(input [4:0] i);                 // header word i (0 .. 29) of the back's buffer, from its snapshot
        case (i)
            // the rectangle (b_sc = {y1, x1, y0, x0}) in the header row's spare bits, for a_render: x0 = word 1 [31:20],
            // y0 = word 2 [31:20], y1 = {word 0 [7:0], word 2 [19:16]}, x1 = {word 7 [31:26], word 7 [15:10]}
            5'd0:  hdr_word = {4'hA, b_prim, (b_prim == 2'd0) ? b_front : 1'b1, (b_prim == 2'd1) ? b_lw_c : 7'd0, esh, b_fsmask, b_sc[47:40]};
            5'd1:  hdr_word = {b_sc[11:0], 20'(tid)};
            5'd2:  hdr_word = {b_sc[23:12], b_sc[39:36], b_zc, 13'(b_did)};
            5'd3:  hdr_word = a2s[31:0];
            5'd4:  hdr_word = {bSY[0], bSX[0]};
            5'd5:  hdr_word = (b_prim == 2'd2) ? 32'(bSX18) : {bSY[1], bSX[1]};             // a point: its 18-bit centre in words 5 / 6
            5'd6:  hdr_word = (b_prim == 2'd0) ? {bSY[2], bSX[2]} : (b_prim == 2'd2) ? 32'(bSY18) : 32'd0;
            5'd7:  hdr_word = {b_sc[35:30], 10'(band_hi), b_sc[29:24], 10'(band_lo)};
            5'd8:  hdr_word = {4'd2, b_prim, (b_prim == 2'd0) ? b_front : 1'b1, 9'd0, b_fsmask, 8'd0};
            5'd9:  hdr_word = 32'(tid);            5'd10: hdr_word = 32'(b_did);              5'd11: hdr_word = {8'd0, r_R};
            5'd12: hdr_word = {8'd0, bZW[0]};      5'd13: hdr_word = {8'd0, dzq1[bbuf]};      5'd14: hdr_word = {8'd0, dzq2[bbuf]};
            5'd15: hdr_word = {8'd0, bIW[0]};      5'd16: hdr_word = (b_prim == 2'd2) ? 32'd0 : {8'd0, bIW[1]};
            5'd17: hdr_word = (b_prim == 2'd0) ? {8'd0, bIW[2]} : 32'd0;
            5'd18: hdr_word = (b_prim == 2'd0) ? {8'd0, b_dzdx} : 32'd0;                     // no polygon offset slope for points / lines
            5'd19: hdr_word = (b_prim == 2'd0) ? {8'd0, b_dzdy} : 32'd0;
            5'd20: hdr_word = (b_prim == 2'd2) ? 32'd0 : (b_prim == 2'd1) ? lA0 : sA0;
            5'd21: hdr_word = (b_prim == 2'd2) ? 32'd0 : (b_prim == 2'd1) ? lB0 : sB0;
            5'd22: hdr_word = (b_prim == 2'd2) ? 32'd16 : (b_prim == 2'd1) ? lA1 : sA1;
            5'd23: hdr_word = (b_prim == 2'd2) ? 32'd0 : (b_prim == 2'd1) ? lB1 : sB1;
            5'd24: hdr_word = (b_prim == 2'd0) ? sA2 : 32'd0;
            5'd25: hdr_word = (b_prim == 2'd2) ? -32'sd16 : (b_prim == 2'd1) ? 32'd0 : sB2;
            5'd26: hdr_word = zq_w[31:0];          5'd27: hdr_word = zq_w[63:32];
            5'd28: hdr_word = zq_w[95:64];         5'd29: hdr_word = zq_w[127:96];
            default: hdr_word = 32'd0;
        endcase
    endfunction
    always @(posedge clk) begin : RECMEM
        reg [4:0] a; reg [7:0] we; reg [255:0] d; reg [1:0] hr;
        we = 8'h00; a = 5'd0; d = '0;
        hr = hw_can[0] ? 2'd0 : hw_can[1] ? 2'd1 : hw_can[2] ? 2'd2 : 2'd3;   // the lowest header row that is pending and ready
        if (blkq_v) begin                                           // a block's row (the first: columns from blk_o & 7 up; the second: the rest)
            a = {blkq_buf, 4'(blk_o[6:3] + 7'(blkq_ph))};
            for (int c = 0; c < 8; c++) begin : BC
                int j; j = blkq_ph ? (8 + c - int'(blk_o[2:0])) : (c - int'(blk_o[2:0]));
                if (j >= 0 && j <= 9) begin we[c] = 1'b1; d[c*32 +: 32] = blkq_d[j*32 +: 32]; end
            end
            if (blkq_ph) blkq_v <= 1'b0; else blkq_ph <= 1'b1;
        end else if (hw_can != 4'd0) begin                          // a header row (row 3: columns 0 .. 5 only)
            a = {bbuf, 2'b00, hr}; we = (hr == 2'd3) ? 8'h3F : 8'hFF;
            for (int c = 0; c < 8; c++) d[c*32 +: 32] = hdr_word({hr, 3'(c)});
            hw_pend[hr] <= 1'b0;
        end
        for (int c = 0; c < 8; c++) if (we[c]) rmem[a][c*32 +: 32] <= d[c*32 +: 32];
        w_data <= rmem[rd_addr];
        if (hand) hw_pend <= 4'hF;                                  // a new back primitive: its four header rows (row 0 is written in
                                                                    // the next cycle, from the snapshot the hand-off loads)
        if (rw_blk) begin                                           // (after the writer: a block landing as the previous one's last row goes out wins)
            blkq_v <= 1'b1; blkq_ph <= 1'b0; blkq_buf <= s_buf; blkq_k <= cblk;
            blkq_d <= {32'd0, as_r, d2[2], d2[1], d2[0], d1[3], d1[2], d1[1], d1[0], f0v};
        end
    end
    always @(posedge clk) begin
        if (rst) begin
            st <= S_ASM; vcnt <= 2'd0; clip_start <= 1'b0; sv_in <= 1'b0; as_in <= 1'b0; mu_in <= 1'b0; rc_in <= 1'b0; cv_in <= 1'b0;
            push_valid <= 1'b0; tid <= '0; ch_run <= 1'b0; ch_done <= 1'b0; ch_R <= 1'b0; ch_st <= 1'b0;
            w_run <= 1'b0; w_arm <= 1'b0; blk_ok <= 1'b0; hrow_ok <= 4'd0; zq_pend <= 1'b0;
            bst <= B_IDLE; s_buf <= 1'b0; sbuf <= 1'b0; bbuf <= 1'b0; strm_ok <= 2'b00; dz_ok <= 2'b00; as_kind <= 2'd0; akq_wp <= 3'd0; akq_rp <= 3'd0;
            bk_go <= 1'b0; b_slp_ok <= 1'b0; push_sent <= 1'b0;
            ps_conv <= 1'b0; prim <= 2'd0; rec_ovf <= 1'b0; clip_ack <= 1'b0; zq_go <= 1'b0; zq_rdy <= 1'b0;
            cph <= 3'd4; dzc <= 2'd2;                        // the collector idle (without the reset: X in sim / 0 in hardware)
            sph <= 3'd4; dz_i <= 2'd2; s_run <= 1'b0; zw_ok <= 1'b0; s_fly <= 6'd0; drop_n <= 6'd0;
            fc_i <= 3'd4; fd_ok <= 1'b0; cv_kind <= 3'd0; cvq_wp <= 2'd0; cvq_rp <= 2'd0; cvq_n <= 3'd0;
            sv_kind <= 1'b0; svk_wp <= 4'd0; svk_rp <= 4'd0; la_wp <= 3'd0; la_rp <= 3'd0; la_cnt <= 4'd0; la_out <= 4'd0; la_drop <= 4'd0; la_pend <= 1'b0;
        end else begin
            clip_start <= 1'b0; clip_ack <= 1'b0; sv_in <= 1'b0; as_in <= 1'b0; mu_in <= 1'b0; rc_in <= 1'b0; cv_in <= 1'b0; zq_go <= 1'b0;
            as_kind <= 2'd0; bk_go <= 1'b0;                              // (the back's S_SUB2 and the stream's dz set as_kind with their as_in; S_CHK3 pulses bk_go)
            // ---- vtx_setup2's result kinds (pushed with an issue, popped with its result) and the positions-ahead queue
            sv_kind <= 1'b0;
            if (sv_in) begin svk[svk_wp] <= sv_kind; svk_wp <= svk_wp + 4'd1; end
            if (sv_out && svk_ne) svk_rp <= svk_rp + 4'd1;
            if (sv_la_out) la_wp <= la_wp + 3'd1;
            if (la_dpop || la_take) la_rp <= la_rp + 3'd1;
            la_cnt  <= la_cnt + 4'(sv_la_out) - 4'(la_dpop || la_take);
            la_out  <= la_out + 4'(pa_valid) - 4'(la_dpop || la_take);
            la_drop <= la_drop - 4'(la_dpop);                            // (+ 3 in S_CLIPW when the triangle in hand gives its three up)
            if (zq_done) zq_rdy <= 1'b1;
            if (frame_start) begin tid <= '0; rec_ovf <= 1'b0; end
            // ---- the adder's result kinds: pushed as the adder takes an operation, popped as its result comes out
            if (as_in) begin akq[akq_wp] <= as_kind; akq_wp <= akq_wp + 3'd1; end
            if (as_v) akq_rp <= akq_rp + 3'd1;
            // ---- the depth-bound words start once R (the chain's second reciprocal) and dz1 / dz2 are in
            if (zq_pend && ch_R && dz_ok[bbuf]) begin zq_go <= 1'b1; zq_pend <= 1'b0; end
            // ---- the record write (its sequence and gating: wseq / w_bd_ok above)
            blk_ok <= strm_ok[bbuf] && !blkq_v;                          // the back's varying blocks landed and are written (a cycle after)
            hrow_ok <= hand ? 4'd0 : ~hw_pend;                           // header row r was written (hw_pend[r] low since last cycle): w_data can hold it now
            w_arm <= w_run;
            if (w_run) begin                                             // (w_data: the record memory's read, RECMEM above)
                if (w_en && w_rdy) begin bc <= bc + 5'd1; w_addr <= w_base + {20'd0, wseq(bc + 5'd1, nb), 3'd0}; end
                if (w_wren && w_wdf_rdy) bd <= bd + 5'd1;
                if (bc == nb && bd == nb) w_run <= 1'b0;
            end
            // ---- the converter: the front's four deltas (from sx_last: dx1 in that very cycle when it can, then dy1, dx2,
            //      dy2), the back's chain (the area, then the key >>> s); its results by kind (cvq, in issue order)
            if (sx_last) fd_ok <= 1'b0;
            if (sx_last && fc_can) begin cv_in <= 1'b1; cv_kind <= 3'd0; cv_i <= 34'(dx1); fc_i <= 3'd1; end
            else if (sx_last) fc_i <= 3'd0;
            else if (fc_i < 3'd4 && fc_can) begin
                cv_in <= 1'b1; cv_kind <= {1'b0, fc_i[1:0]}; fc_i <= fc_i + 3'd1;
                cv_i <= (fc_i[1:0] == 2'd0) ? 34'(dx1) : (fc_i[1:0] == 2'd1) ? 34'(dy1) : (fc_i[1:0] == 2'd2) ? 34'(dx2) : 34'(dy2);
            end
            if (ch_go) begin ch_run <= 1'b1; ch_st <= 1'b1; ch_done <= 1'b0; ch_R <= 1'b0; ch_i <= 2'd0; end
            if (ch_run && ch_i < 2'd2) begin                             // (the front's values; never in a cycle the deltas convert: fc_can)
                cv_in <= 1'b1; cv_kind <= ch_i[0] ? 3'd4 : 3'd5; cv_i <= ch_i[0] ? a2 : f_a2s; ch_i <= ch_i + 2'd1;
            end
            if (cv_in) begin cvq[cvq_wp] <= cv_kind; cvq_wp <= cvq_wp + 2'd1; cvq_n <= cvq_n + 3'd1 - 3'(cv_v && cvq_n != 3'd0); end
            else if (cv_v && cvq_n != 3'd0) cvq_n <= cvq_n - 3'd1;
            if (cv_v && cvq_n != 3'd0) begin                             // (a result is owed: a valid after a reset with none owed is not one)
                cvq_rp <= cvq_rp + 2'd1;
                case (cvq[cvq_rp])
                    3'd4: begin rc_in <= 1'b1; rc_x <= {cv_r[W-1], cv_r[W-2:MAN_W] - 7'd4, cv_r[MAN_W-1:0]}; end   // the area / 16: exponent - 4 (the area is not 0 here)
                    3'd5: begin rc_in <= 1'b1; rc_x <= cv_r; end        // R = rcp(i2f(key >>> s)) (a line's squared length, a point's size)
                    default: begin fd[cvq[cvq_rp][1:0]] <= cv_r; if (cvq[cvq_rp][1:0] == 2'd3 && !sx_last) fd_ok <= 1'b1; end
                endcase
            end
            if (ch_run && rc_v) begin                                    // the two reciprocals, in issue order
                if (!ch_R) begin r_R <= rc_r; ch_R <= 1'b1; end          // (header row 1 and the depth bound wait for ch_R, B_MUL2 for ch_done)
                else begin r_a2 <= rc_r; ch_done <= 1'b1; ch_run <= 1'b0; end
            end
            // ---- the difference stream: its issue (one per cycle while it runs) and its collector (in every state; results
            //      arrive in issue order); a rejected triangle's results in flight are dropped first
            s_fly <= s_fly_n;
            if (s_drop) drop_n <= drop_n - 6'd1;
            if (s_iss) begin
                as_in <= 1'b1; as_op <= 1'b1;
                if (s_dz) begin                                          // dz1 = z1 - z0, dz2 = z2 - z0
                    as_kind <= 2'd2; as_a <= dz_i[0] ? ZW[2] : ZW[1]; as_b <= ZW[0]; dz_i <= dz_i + 2'd1;
                    if (dz_i == 2'd1 && sph == 3'd4) s_run <= 1'b0;
                end else if (sph == 3'd2) begin
                    as_a <= vVar[1][(s_i*4 + sc)*W +: W]; as_b <= vVar[0][(s_i*4 + sc)*W +: W];
                    sc <= sc + 1'b1; if (sc == 2'd3) sph <= 3'd3;
                end else begin
                    as_a <= vVar[2][(s_i*4 + sc)*W +: W]; as_b <= vVar[0][(s_i*4 + sc)*W +: W];
                    sc <= sc + 1'b1;
                    if (sc == 2'd3) begin
                        srem <= srem & ~(8'd1 << s_i);
                        if ((srem & ~(8'd1 << s_i)) == 8'd0) begin sph <= 3'd4; if (dz_i == 2'd2) s_run <= 1'b0; end
                        else sph <= 3'd2;
                    end
                end
            end
            if (s_cdz) begin
                if (dzc[0]) begin dzq2[s_buf] <= as_r; dz_ok[s_buf] <= 1'b1; end else dzq1[s_buf] <= as_r;
                dzc <= dzc + 2'd1;
            end
            if (s_col) begin
                if (cph == 3'd2) begin d1[cc] <= as_r; cc <= cc + 1'b1; if (cc == 2'd3) cph <= 3'd3; end
                else begin
                    d2[cc] <= as_r; cc <= cc + 1'b1;
                    if (cc == 2'd3) begin                           // the block {0, D2, D1, f0} into slot cblk: words 30 + 10 cblk .. (the record block below writes it)
                        cblk <= cblk + 1'b1; crem <= crem & ~(8'd1 << c_i);
                        if ((crem & ~(8'd1 << c_i)) == 8'd0) begin cph <= 3'd4; strm_ok[s_buf] <= 1'b1; end   // the last block: the buffer's varyings are complete
                        else cph <= 3'd2;
                    end
                end
            end
            case (st)
                S_ASM: if (vo_valid && vo_ready) begin               // the primitive's vertices in (3, 2 or 1); vo_ready: the previous stream has landed
                    for (int i = 0; i < 3; i++)                      // a point's vertex into all three, a line's first into 0 and 2
                        if (2'(i) == vcnt || vo_prim == 2'd2 || (vo_prim == 2'd1 && vcnt == 2'd0 && i == 2)) begin
                            vX[i] <= vo_pos[0 +: W]; vY[i] <= vo_pos[W +: W]; vZ[i] <= vo_pos[2*W +: W]; vW[i] <= vo_pos[3*W +: W];
                            vVar[i] <= vo_var;
                        end
                    if (vcnt == 2'd0) begin                          // the first vertex: the kind, the size, the draw's state
                        prim <= vo_prim; psz <= vo_psize;
                        sxs <= sxs_i; sxo <= sxo_i; sys <= sys_i; syo <= syo_i; zs <= zs_i; zo <= zo_i; cull_mode <= cull_mode_i; front_cw <= front_cw_i; zclip0 <= zclip0_i;
                        line_width <= line_width_i; fsmask <= fsmask_i; draw_id <= draw_id_i; scr_w <= scr_w_i; scr_h <= scr_h_i; gb <= gb_i;
                        zc <= zc_enc(zc_i);
                        {sc_y1, sc_x1, sc_y0, sc_x0} <= sc_i; sc_emp <= (sc_i[11:0] >= sc_i[35:24]) | (sc_i[23:12] >= sc_i[47:36]);
                    end
                    lc_p <= 3'd0;
                    if (vo_prim == 2'd2) begin vcnt <= 2'd0; st <= S_PT0; end
                    else if (vo_prim == 2'd1) begin if (vcnt == 2'd1) begin vcnt <= 2'd0; st <= S_LC0; end else vcnt <= vcnt + 1'b1; end
                    else if (vcnt == 2'd2) begin vcnt <= 2'd0; la_pend <= vo_la; st <= S_CLIPF; end else vcnt <= vcnt + 1'b1;   // (la_pend: its window positions are on their way)
                end
                // ---------------- a point: discarded outside the w, near or far plane (ES 2.0 §2.13), else vertex 0 copied
                //                  into 1 / 2 and the size converted
                S_PT0: if (drop_n == 6'd0 && !bk_busy) begin             // (entered with lc_p = 0 from S_ASM; the next plane after a pass;
                    // a rejected triangle's results drained first and the back idle: the adder, multiplier and SFU are shared)
                    as_in <= 1'b1; as_op <= pd_sub(lc_p); as_a <= pd_a(2'd0, lc_p); as_b <= pd_b(2'd0, lc_p); st <= S_PTCW;
                end
                S_PTCW: if (as_v) begin
                    if (as_r[W-1]) st <= S_NEXT;                         // outside: dropped
                    else if (lc_p != 3'd2) begin lc_p <= lc_p + 3'd1; st <= S_PT0; end   // points: the w, near and far planes only (§2.13)
                    else begin ps_conv <= 1'b1; st <= S_PTS1; end        // (vertices 1 / 2 hold vertex 0 since the assembly)
                end
                S_PTS1: if (!pa_valid) begin sv_in <= 1'b1; svx <= psz; svy <= '0; svz <= '0; svw <= ONE; st <= S_PTS2; end
                S_PTS2: if (sv_dir_out) begin                            // size = 2 RNE(8 gl_PointSize) — even, so the square is centred exactly (§3.3) —
                    ps_u <= (s_sx < 16'sd8) ? 11'd16 : (s_sx > 16'sd512) ? 11'd1024 : {s_sx[9:0], 1'b0};   // clamped to 16 .. 16 MAXPT (<= 0, NaN, Inf -> the minimum)
                    ps_conv <= 1'b0; vert <= 2'd0; rej_any <= 1'b0; st <= S_SETF;
                end
                // ---------------- a line: against the w, near and far planes in turn — both ends outside -> dropped; one
                //                  outside -> moved onto the plane by the lerp of every used attribute FROM THE INSIDE end
                //                  (t = d_in rcp(d_in - d_out), as clip_near; a_setup_ref clip_line)
                S_LC0: if (drop_n == 6'd0 && !bk_busy) begin lc_p <= 3'd0; st <= S_LCP0; end   // (a rejected triangle's results drained first, the back idle)
                // d of end 0 (S_LCP0 -> S_LCP1) and end 1 (S_LCP1 -> S_LCP2): planes 0..2 one subtraction, the guard planes a
                // product g w first (S_LCG issues the subtraction when it lands)
                S_LCP0: begin
                    if (lc_p < 3'd3) begin as_in <= 1'b1; as_op <= pd_sub(lc_p); as_a <= pd_a(2'd0, lc_p); as_b <= pd_b(2'd0, lc_p); st <= S_LCP1; end
                    else begin mu_in <= 1'b1; mu_a <= gconst(lc_p); mu_b <= vW[0]; lc_v <= 1'b0; st <= S_LCG; end
                end
                S_LCG: if (mu_v) begin
                    as_in <= 1'b1; as_op <= 1'b1;
                    if (lc_p[0]) begin as_a <= mu_r; as_b <= gattr({1'b0, lc_v}, lc_p); end else begin as_a <= gattr({1'b0, lc_v}, lc_p); as_b <= mu_r; end
                    st <= lc_v ? S_LCP2 : S_LCP1;
                end
                S_LCP1: if (as_v) begin
                    lc_da <= as_r;
                    if (lc_p < 3'd3) begin as_in <= 1'b1; as_op <= pd_sub(lc_p); as_a <= pd_a(2'd1, lc_p); as_b <= pd_b(2'd1, lc_p); st <= S_LCP2; end
                    else begin mu_in <= 1'b1; mu_a <= gconst(lc_p); mu_b <= vW[1]; lc_v <= 1'b1; st <= S_LCG; end
                end
                S_LCP2: if (as_v) begin : LCP2
                    reg ia, ib;
                    ia = ~lc_da[W-1]; ib = ~as_r[W-1];
                    if (!ia && !ib) st <= S_NEXT;                                         // outside this plane: dropped
                    else if (ia != ib) begin                                              // a crossing: t from the inside end
                        lc_out <= ia; lc_din <= ia ? lc_da : as_r;
                        as_in <= 1'b1; as_op <= 1'b1; as_a <= ia ? lc_da : as_r; as_b <= ia ? as_r : lc_da; st <= S_LC1;   // d_in - d_out
                    end
                    else if (lc_p == 3'd6) st <= S_LCD;                                   // inside every plane
                    else begin lc_p <= lc_p + 3'd1; st <= S_LCP0; end
                end
                S_LC1: if (as_v) begin rc_in <= 1'b1; rc_x <= as_r; st <= S_LC2; end                     // rcp(d_in - d_out)
                S_LC2: if (rc_v) begin mu_in <= 1'b1; mu_a <= lc_din; mu_b <= rc_r; st <= S_LC4; end     // t = d_in rcp(..)
                S_LC4: if (mu_v) begin lc_t <= mu_r; lc_k <= 6'd0; st <= S_LC5; end
                S_LC5: if (!aen(lc_k)) begin                                               // an unused varying: left as it is
                    if (lc_k == 6'(3 + NVAR)) begin if (lc_p == 3'd6) st <= S_LCD; else begin lc_p <= lc_p + 3'd1; st <= S_LCP0; end end
                    else lc_k <= lc_k + 6'd1;
                end else begin as_in <= 1'b1; as_op <= 1'b1; as_a <= attr({1'b0, lc_out}, lc_k); as_b <= attr({1'b0, ~lc_out}, lc_k); st <= S_LC6; end   // out_k - in_k
                S_LC6: if (as_v) begin mu_in <= 1'b1; mu_a <= lc_t; mu_b <= as_r; st <= S_LC7; end                            // t (out_k - in_k)
                S_LC7: if (mu_v) begin as_in <= 1'b1; as_op <= 1'b0; as_a <= attr({1'b0, ~lc_out}, lc_k); as_b <= mu_r; st <= S_LC8; end   // in_k + ..
                S_LC8: if (as_v) begin                                   // the lerped attribute lands in the outside vertex (and in
                    for (int i = 0; i < 3; i++)                          //  vertex 2, vertex 0's twin, when that is vertex 0)
                        if (2'(i) == {1'b0, lc_out} || (i == 2 && !lc_out)) begin   // lc_oh: lc_k's registered one-hot (timing at
                            // 120 MHz: the index decode stays out of the clock enable of every vVar flop)
                            if (lc_oh[0]) vX[i] <= as_r; if (lc_oh[1]) vY[i] <= as_r; if (lc_oh[2]) vZ[i] <= as_r; if (lc_oh[3]) vW[i] <= as_r;
                            for (int k = 0; k < NVAR; k++) if (lc_oh[4 + k]) vVar[i][k*W +: W] <= as_r;
                        end
                    if (lc_k == 6'(3 + NVAR)) begin                       // all 4 + NVAR attributes done: the next plane
                        if (lc_p == 3'd6) st <= S_LCD; else begin lc_p <= lc_p + 3'd1; st <= S_LCP0; end
                    end else begin lc_k <= lc_k + 6'd1; st <= S_LC5; end
                end
                S_LCD: begin vert <= 2'd0; rej_any <= 1'b0; st <= S_SETF; end   // (vertex 2 holds vertex 0, the clip's writes included: dz2 = D2 = 0)
                // ---------------- a triangle: clip_near emits its 0 .. 4 triangles one at a time (tri_valid / tri_ack)
                S_CLIPF: begin clip_start <= 1'b1; st <= S_CLIPW; end
                S_CLIPW: begin
                    if (clip_tv) begin                                   // the next clipped triangle: set it up (an untouched one is
                        if (!clip_pass) for (int i = 0; i < 3; i++) begin   // still in vX .. vVar: the clipper copies nothing then)
                            vX[i] <= clip_to[i*VB + 0*W +: W]; vY[i] <= clip_to[i*VB + 1*W +: W];
                            vZ[i] <= clip_to[i*VB + 2*W +: W]; vW[i] <= clip_to[i*VB + 3*W +: W];
                            vVar[i] <= clip_to[i*VB + 4*W +: VWB];
                        end
                        vert <= 2'd0; rej_any <= 1'b0;
                        // an untouched triangle whose window positions went ahead takes them from the queue (S_LAW); a
                        // clipped one's pieces are set up here (S_SETF), and the original's three results are discarded
                        st <= (clip_pass && la_pend) ? S_LAW : S_SETF;
                        if (!clip_pass && la_pend) begin la_pend <= 1'b0; la_drop <= la_drop + 4'd3 - 4'(la_dpop); end
                    end else if (clip_done) begin                        // none (left)
                        st <= S_ASM;
                        if (la_pend) begin la_pend <= 1'b0; la_drop <= la_drop + 4'd3 - 4'(la_dpop); end   // clipped away: its three too
                    end
                end
                // the triangle's three window positions from the queue, one a cycle as they are there (they were issued
                // when the triangle was formed: usually all three wait); the difference stream starts with the first,
                // exactly as in S_SETF
                S_LAW: if (la_take) begin
                    SX[vert] <= la_head[15:0]; SY[vert] <= la_head[31:16]; ZW[vert] <= la_head[32 +: W]; IW[vert] <= la_head[32 + W +: W];
                    rej_any <= rej_any | la_head[80];
                    if (vert == 2'd2) begin vert <= 2'd0; la_pend <= 1'b0; zw_ok <= 1'b1; st <= S_CHK; end else vert <= vert + 1'b1;   // ZW[0..2] valid next cycle
                    if (vert == 2'd0) begin
                        srem <= fsmask; crem <= fsmask; sc <= 2'd0; cc <= 2'd0; sblk <= 4'd0; cblk <= 4'd0;
                        sph <= (fsmask == 8'd0) ? 3'd4 : 3'd2; cph <= (fsmask == 8'd0) ? 3'd4 : 3'd2; dz_i <= 2'd0; dzc <= 2'd0;
                        s_run <= 1'b1; zw_ok <= 1'b0;
                        s_buf <= sbuf; strm_ok[sbuf] <= (fsmask == 8'd0); dz_ok[sbuf] <= 1'b0;
                        nv <= 4'(fsmask[0]) + 4'(fsmask[1]) + 4'(fsmask[2]) + 4'(fsmask[3]) + 4'(fsmask[4]) + 4'(fsmask[5]) + 4'(fsmask[6]) + 4'(fsmask[7]);
                    end
                end
                S_SETF: if (!pa_valid && (vert != 2'd0 || s_done)) begin   // (clipped triangles, points, lines; a position ahead goes first); a new stream only once the
                    sv_in <= 1'b1; svx <= vX[vert]; svy <= vY[vert]; svz <= vZ[vert]; svw <= vW[vert];   // previous one has landed (a clipper fan's next triangle comes straight here)
                    if (vert == 2'd2) begin vert <= 2'd0; svcol <= 2'd0; st <= S_SETW; end else vert <= vert + 1'b1;
                    if (vert == 2'd0) begin
                        // the difference stream from here, beside the vertex setup: the varyings' D1 / D2 at once (the vertices
                        // are in vVar), dz1 / dz2 once the window z exists (zw_ok)
                        srem <= fsmask; crem <= fsmask; sc <= 2'd0; cc <= 2'd0; sblk <= 4'd0; cblk <= 4'd0;
                        sph <= (fsmask == 8'd0) ? 3'd4 : 3'd2; cph <= (fsmask == 8'd0) ? 3'd4 : 3'd2; dz_i <= 2'd0; dzc <= 2'd0;
                        s_run <= 1'b1; zw_ok <= 1'b0;
                        s_buf <= sbuf; strm_ok[sbuf] <= (fsmask == 8'd0); dz_ok[sbuf] <= 1'b0;   // the stream lands in the buffer the back is not reading
                        nv <= 4'(fsmask[0]) + 4'(fsmask[1]) + 4'(fsmask[2]) + 4'(fsmask[3]) + 4'(fsmask[4]) + 4'(fsmask[5]) + 4'(fsmask[6]) + 4'(fsmask[7]);
                    end
                end
                S_SETW: if (sv_dir_out) begin
                    SX[svcol] <= s_sx; SY[svcol] <= s_sy; ZW[svcol] <= s_zw; IW[svcol] <= s_iw; rej_any <= rej_any | s_rej;
                    if (svcol == 2'd0) begin SX18 <= s_sx18; SY18 <= s_sy18; end
                    if (svcol == 2'd2) begin st <= S_CHK; zw_ok <= 1'b1; end else svcol <= svcol + 1'b1;   // ZW[0..2] valid next cycle
                end
                S_CHK: st <= S_CHK1;                                   // the area: products, then the difference (2 registers)
                S_CHK1: st <= S_CHK2;
                S_CHK2: begin                                           // the verdict, registered (else the line / point / cull terms
                    rej_r <= rej_any | sc_emp | ((prim == 2'd0) ? (culled | offscreen | nocen) : (prim == 2'd2) ? pt_off : (ln_off | (len2 == 34'd0)));
                    ovf_r <= (tid >= IDX_W'(REC_ENTRIES));              //   drive the clock enables of the whole snapshot)
                    ch_st <= 1'b0;                                      // (this primitive's reciprocal chain has not started)
                    st <= S_CHK3;                                       // the record region full: drop the primitive, flag it
                end
                S_CHK3: begin
                    if (rej_r | ovf_r) begin
                        st <= S_NEXT;                                   // the stream stops; its results in flight are dropped
                        s_run <= 1'b0; sph <= 3'd4; cph <= 3'd4; dz_i <= 2'd2; dzc <= 2'd2; s_fly <= 6'd0; drop_n <= drop_n - 6'(s_drop) + s_fly_n;
                        if (ovf_r & ~rej_r) rec_ovf <= 1'b1;
                    end else if (hand) begin                            // the hand-off to the back (the front waits while the back works):
                        for (int i = 0; i < 3; i++) begin bSX[i] <= SX[i]; bSY[i] <= SY[i]; bZW[i] <= ZW[i]; bIW[i] <= IW[i]; end
                        bSX18 <= SX18; bSY18 <= SY18; b_a2 <= a2; b_len2 <= len2; b_ps_u <= ps_u; b_prim <= prim; b_fcw <= front_cw;
                        b_lw <= line_width; b_fsmask <= fsmask; b_did <= draw_id; b_zc <= zc; b_row_lo <= row_lo; b_row_hi <= row_hi;
                        b_sc <= {sc_y1, sc_x1, sc_y0, sc_x0};
                        bbuf <= s_buf; sbuf <= ~s_buf;                  //   the back takes this stream's buffer, the next stream the other
                        bk_go <= 1'b1; b_slp_ok <= 1'b0;                //   (the header writer: four rows pending; the slopes not yet)
                        zq_pend <= 1'b1; zq_rdy <= 1'b0;                //   the record write and the depth bound alongside (the chain: ch_go)
                        nb <= 5'((30 + 10*nv + 7) / 8); bc <= 5'd0; bd <= 5'd0; w_run <= 1'b1;
                        w_base <= REC_BASE + (bank ? REC_BANK_W : 29'd0) + {tid, 7'd0};
                        w_addr <= REC_BASE + (bank ? REC_BANK_W : 29'd0) + {tid, 7'd0} + {20'd0, wseq(5'd0, 5'((30 + 10*nv + 7) / 8)), 3'd0};
                        bst <= B_SUB; st <= S_NEXT;
                    end
                end
                S_NEXT: if (prim == 2'd0) begin clip_ack <= 1'b1; st <= S_CLIPW; end else st <= S_ASM;   // a triangle: ask the clipper for the next
                default: st <= S_ASM;
            endcase
            // ---------------- the BACK: the depth planes, the header, the record's bursts, the binner push of the primitive in the snapshot
            case (bst)
                B_IDLE: ;
                B_SUB: if (dz_ok[bbuf] && fd_ok) begin                  // dz1 / dz2 and the converted deltas (the front's) are in
                    k_i <= 3'd0; k_c <= 3'd0; bst <= B_MUL1;            // (the reciprocals may still be in flight: B_MUL2 waits for the first)
                end
                B_MUL1: begin                                           // dz1*fdy2, dz2*fdy1, dz2*fdx1, dz1*fdx2
                    if (k_i < 3'd4) begin
                        mu_in <= 1'b1; k_i <= k_i + 1'b1;
                        case (k_i)
                            3'd0: begin mu_a <= dzq1[bbuf]; mu_b <= fd[3]; end 3'd1: begin mu_a <= dzq2[bbuf]; mu_b <= fd[1]; end
                            3'd2: begin mu_a <= dzq2[bbuf]; mu_b <= fd[0]; end default: begin mu_a <= dzq1[bbuf]; mu_b <= fd[2]; end
                        endcase
                    end
                    if (mu_v) begin mm[k_c[1:0]] <= mu_r; k_c <= k_c + 1'b1; if (k_c == 3'd3) begin k_i <= 3'd0; k_c <= 3'd0; bst <= B_SUB2; end end
                end
                B_SUB2: begin                                           // nx = m0 - m1, ny = m2 - m3 (kind 1 on the shared adder; the stream yields)
                    if (b_sub2_iss) begin as_in <= 1'b1; as_op <= 1'b1; as_kind <= 2'd1; k_i <= k_i + 1'b1; as_a <= k_i[0] ? mm[2] : mm[0]; as_b <= k_i[0] ? mm[3] : mm[1]; end
                    if (as_v && ak_head == 2'd1) begin if (k_c[0]) ny <= as_r; else nx <= as_r; k_c <= k_c + 1'b1; if (k_c == 3'd1) begin k_i <= 3'd0; k_c <= 3'd0; bst <= B_MUL2; end end
                end
                B_MUL2: begin                                           // dz/dx = nx * r, dz/dy = ny * r (r = rcp(area / 16): the chain's first reciprocal)
                    if (k_i < 3'd2 && ch_done) begin mu_in <= 1'b1; k_i <= k_i + 1'b1; mu_a <= k_i[0] ? ny : nx; mu_b <= r_a2; end
                    if (mu_v) begin                                 // words 18 / 19, held for header row 2
                        if (k_c[0]) begin b_dzdy <= mu_r; b_slp_ok <= 1'b1; end else b_dzdx <= mu_r;
                        k_c <= k_c + 1'b1; if (k_c == 3'd1) bst <= B_WR;
                    end
                end
                // every burst accepted (the write machine, beside the state machines: a header burst needs its row written,
                // hrow_ok, so the rows are all in) and the binner has taken the push (below). (B_HDR and B_PUSH are never
                // entered.)
                B_WR: if (!w_run && push_sent) begin tid <= tid + 1'b1; bst <= B_IDLE; end
                default: bst <= B_IDLE;
            endcase
            // ---------------- the binner push goes out at the hand-off: the binner reads its band heads and writes its bin lines beside the back's
            // arithmetic and between the record's bursts (w_req above). Its entries name a record that lands a little later;
            // the render side reads bins and records only once the geometry side is idle. The back ends when both the
            // record and the push are through (B_WR), so pushes stay in record order.
            if (bk_go) begin
                push_valid <= 1'b1; push_sent <= 1'b0; push_tri_idx <= tid; push_draw_id <= b_did; push_band_lo <= band_lo; push_band_hi <= band_hi;
                push_tri_nb <= 4'(nb - 5'd1);
            end else if (push_valid && push_ready) begin push_valid <= 1'b0; push_sent <= 1'b1; end
            // ---------------- a position ahead goes into vtx_setup2 the cycle it arrives (after the state machine above, whose
            // own issues wait for it). A draw's first one loads the viewport and depth range: the front is idle and the unit
            // empty then (pa_grant), and every position of the draw — ahead or issued by the state machine — uses them.
            if (pa_valid) begin
                sv_in <= 1'b1; sv_kind <= 1'b1; svx <= pa_pos[0 +: W]; svy <= pa_pos[W +: W]; svz <= pa_pos[2*W +: W]; svw <= pa_pos[3*W +: W];
                if (pa_first) begin sxs <= sxs_i; sxo <= sxo_i; sys <= sys_i; syo <= syo_i; zs <= zs_i; zo <= zo_i; end
            end
        end
    end
    // synthesis translate_off
    // the result-kind queues and the positions-ahead accounting
    always @(posedge clk) if (!rst && rst_ok) begin
        if (la_cnt > 4'd8) $fatal(1, "a_setup: the positions-ahead queue overflowed (%0d)", la_cnt);
        if (la_out > 4'd6) $fatal(1, "a_setup: %0d positions ahead outstanding (two triangles at most)", la_out);
        if (sv_out && !svk_ne) $fatal(1, "a_setup: a vtx_setup2 result that no issue owns");
        if (cv_v && cvq_n == 3'd0) $fatal(1, "a_setup: a converter result that no issue owns");
        if (pa_valid && pa_first && !(st == S_ASM && vcnt == 2'd0)) $fatal(1, "a_setup: a draw's first position ahead with the front busy (state %0d)", st);
        if (pa_valid && (st == S_SETW || st == S_PTS2) && ps_conv) $fatal(1, "a_setup: a position ahead during a point-size conversion");
        if (la_take && !la_pend) $fatal(1, "a_setup: S_LAW takes a position the triangle in hand does not own");
        if (hand && !(ch_st || ch_go)) $fatal(1, "a_setup: a hand-off without the primitive's reciprocal chain");
    end
`ifdef A_SETUP_DBG
    int dbg_c = 0;
    always @(posedge clk) begin
        dbg_c <= dbg_c + 1;
        if (!rst && (dbg_c % 20000 == 19999))
            $display("a_setup dbg: st %0d bst %0d tid %0d | fc_i %0d fd_ok %b fd_free %b | ch run %b st %b R %b done %b i %0d | s_run %b sph %0d cph %0d dz_i %0d dzc %0d zw_ok %b dz_ok %b strm_ok %b sbuf %b%b%b | w_run %b bc %0d bd %0d nb %0d hw_pend %b hrow_ok %b blk_ok %b | push v %b sent %b | zq pend %b rdy %b | drop_n %0d s_fly %0d | cvq %0d/%0d akq %0d/%0d | la out %0d cnt %0d drop %0d pend %b",
                     st, bst, tid, fc_i, fd_ok, fd_free, ch_run, ch_st, ch_R, ch_done, ch_i, s_run, sph, cph, dz_i, dzc, zw_ok, dz_ok, strm_ok, s_buf, sbuf, bbuf,
                     w_run, bc, bd, nb, hw_pend, hrow_ok, blk_ok, push_valid, push_sent, zq_pend, zq_rdy, drop_n, s_fly, cvq_wp, cvq_rp, akq_wp, akq_rp,
                     la_out, la_cnt, la_drop, la_pend);
    end
`endif
    // synthesis translate_on
    wire _unused = &{1'b0, clip_busy, sblk};
endmodule
`default_nettype wire
