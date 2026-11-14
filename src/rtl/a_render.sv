// a_render — board A's render side (model tools/a_raster_ref.py): after `go` (the host's FLUSH)
// it walks the frame's bands in order and emits the render stream for board B:
//   for band b = 0 .. ceil(scr_h / nl) - 1:  BAND {4, y0 = nl b}   (nl = 8 rows for targets up to BAND8_MAXW = 1024 wide,
//                                                                    else NL = 4: B's band buffer holds 8 lines of such a
//                                                                    target)
//     for every entry of band b's bin list (tri_binner: BIN_HEADS[b] -> BINLIST head .. next, oldest first):
//       the primitive's record is read from A's DDR (REC_BASE + id * 128 words): the raster header burst gives the
//       kind (triangle / line / point), the shift s, the area's sign, the window positions (1/16 pixel), a point's
//       size, a line's width and the first band the primitive touches; its TRI item (words 8.., 11 + 5 nv record beats,
//       plus two beats this walk inserts after the 11th: the reference edge values er0 er1 er2 at the centre of pixel
//       (0, 0) >> s and a 0) is sent in its first band and in every later band where
//       it is not already B's current primitive (B does not re-read a record from its DDR: that would cost it ≈ 45
//       cycles a visit); then the rasterizer sends a ONE-beat QUAD {cov, qy, qx} for
//       every 2 x 2 quad with a covered pixel — B forms the quad's edge values from the references and the TRI's steps:
//         triangle — the bounding box ∩ the band ∩ the target, a pixel centre (16 px + 8, 16 py + 8) covered iff every
//                    edge value normalised to the orientation is > 0, or == 0 on the edge that owns its boundary
//                    (normalised x step > 0, or 0 with the y step > 0: ES 2.0 §3.5.1); e = the three edges;
//         point    — the square [xl, xl + ps) x [yb, yb + ps) (xl = x - ps/2 ..): covered iff 0 <= cx - xl < ps and
//                    0 < yt - cy <= ps (§3.3; t runs top to bottom); e = (ps, cx - xl, yt - cy);
//         line     — §3.4.1 / §3.4.2 in Bresenham form: x-major (|dx| >= |dy|) one fragment per column whose centre lies
//                    in [xa, xb) (dx > 0) or (xb, xa] (dx < 0), in the row floor((y(cx) - 8 (lw - 1)) / 16) with
//                    y(cx) = ya + (cx - xa) dy / dx exact (one division per line and band, then one adjust per column),
//                    replicated over lw rows; y-major with x and y exchanged; the quads of a column pair (x-major) /
//                    row pair (y-major) merged; e = (len2 - e1, e1, 0) with e1 = (c - a) . (b - a).
//   FRAME_END {5}
// Triangles / points: one quad position per cycle — the coverage decision of a position is registered (EVAL) while the
// position before it is pushed; a row ends as soon as both its pixel rows are past the triangle (an edge outside that
// does not grow with x). gl_FrontFacing of points / lines is true (the record's front bit).
// DDR reads: a FETCHER runs ahead of the rasterizer. It never waits for a read it has issued (two serial DDR round trips
// per primitive visit would cost the walk about half its time in a game): a band's list lines (four
// entries per 32-byte burst, chunked by the binner) are read one ahead — the next line's pointer is in the last entry of
// the line before; per entry the primitive's header read is issued at once (the entry's flags say whether this is the
// primitive's first band and how many TRI bursts its record has, so nothing waits for the header), and its TRI bursts
// follow BACK TO BACK (the arbiter returns a port's reads in order) into a 32-line burst FIFO the rasterizer streams
// from (or drops, when the primitive is still current). Visits (band markers, primitives, the end) queue 16 deep; a
// primitive's header lands in its slot of the queue when it returns, and the rasterizer takes a visit once it has. Up to
// 30 reads are in flight.
`default_nettype none
module a_render #(
    parameter int NL = 4, BANDW = 10, IDX_W = 20, DID_W = 12, PTR_W = 23, COUNT_W = 16,
    parameter int BAND8_MAXW = 1024,             // targets up to this wide: 8-line bands (board B's b_render BAND8_MAXW)
    parameter [28:0] REC_BASE = 29'h1000000, BINLIST_BASE = 29'h1800000,
    parameter [28:0] REC_BANK_W = 29'h2000000, POOL_W = 29'h800000   // bank 1's records / bin lists follow bank 0's
)(
    input  wire        clk, rst,
    input  wire        go,                       // start a frame (the bins are complete)
    input  wire        go_bank,                  // the record / bin-list bank of that frame
    input  wire        abort,                    // give the frame up for good (a credit re-sync): idle, the queue dropped, done pulses
    input  wire        abort_soft,               // end the frame IN-BAND at the next primitive visit: FRAME_END with its abort bit,
                                                 // nothing dropped (B's parser stays in step and pairs it with its FLUSH); done follows
    output reg         done,                     // FRAME_END sent (or the frame aborted)
    output wire        busy,
    input  wire [11:0] scr_w, scr_h,             // the render target (pixels)
    output reg  [31:0] c_vis,                    // primitive-band visits, cumulative (STATUS word 100: the walk's per-visit cost)
    // bin heads (tri_binner's query port: q_* valid the cycle after q_band)
    output reg         q_bank,
    output reg  [BANDW-1:0] q_band,
    input  wire [PTR_W-1:0] q_head,
    input  wire [COUNT_W-1:0] q_count,
    // DDR read port (mem_arb)
    output reg         r_en,
    output reg  [28:0] r_addr,
    input  wire        r_rdy, r_valid,
    input  wire [255:0] rd_data,
    // the render stream (64-bit beats {w1, w0}, in order)
    output reg         rs_valid,
    output reg  [63:0] rs_data,
    input  wire        rs_ready,
    // statistics
    output reg  [31:0] c_quads, c_tris           // quads sent, primitive-band visits
);
    localparam int EW = 36;                       // edge values: 17 x 17 products summed (1/256 pixel^2)
    typedef enum logic [5:0] {W_IDLE, W_VISIT, W_BANDB, W_TRIW, W_TRIE, W_TRIR, W_SWAIT,
                              W_ROW, W_EVAL, W_NEXT, W_FEND,
                              W_LS1, W_LS2, W_LS2B, W_LS2C, W_LS3, W_LDIV0, W_LDIV, W_LDIV1, W_LCOL, W_LPAIR, W_LPAIRB, W_LPAIR2,
                              W_LQ0, W_LQ, W_LQ1, W_LQA, W_LPEND} st_t;
    // the setup sequencer: a visit's setup (S1..S3) and the reference edge values (R1..R3) run beside the
    // TRI item instead of before it — the references go in after the item's 11th beat, at least 12 cycles into it, and
    // the sequencer is done 9 cycles in; the walk waits for it only where it needs its results (W_TRIR, W_SWAIT).
    // W1..W3 (the row setup, hidden): the band's first quad row's start values (rs), which the walk then
    // steps by two pixel rows per row — the rows need no origin / product / sum / lane states of their own
    typedef enum logic [3:0] {SP_IDLE, SP_S1, SP_S2, SP_S3, SP_R1, SP_R2, SP_R3, SP_W1, SP_W2, SP_W3} sp_t;
    st_t st;
    sp_t sp;
    wire sp_busy = (sp != SP_IDLE);
    wire out_free = !rs_valid || rs_ready;

    // ---- the fetcher: bin lists, headers, TRI bursts ahead of the rasterizer
    typedef enum logic [2:0] {F_IDLE, F_BAND, F_QRY, F_QRY2, F_ENT, F_TRI} fs_t;
    fs_t fs;
    wire [28:0] rec_base = REC_BASE + (q_bank ? REC_BANK_W : 29'd0), pool_base = BINLIST_BASE + (q_bank ? POOL_W : 29'd0);
    // the band height: 8 rows up to BAND8_MAXW = 1024 wide — board
    // B's rule (b_render BAND8_MAXW: its band buffer's 4 096 cells), a_setup bins with it
    // The frame's target is latched with `go`. VS_STATE waits for nothing (the DRAW snapshots it), so the next pass's
    // state can arrive while this frame is still being walked: read live, scr_w / scr_h would change the band height,
    // the band count and the clamps under the walk whenever the two passes' targets differ in size
    // (a render-to-texture pass followed by the screen's). `go` is the FLUSH's own pulse, so the registers then hold the
    // size of the frame being flushed; the fetcher and the walk use them from the next cycle on.
    reg  [11:0] f_w, f_h;
    always @(posedge clk) if (go) begin f_w <= scr_w; f_h <= scr_h; end
    wire [3:0]  nl   = (f_w <= 12'(BAND8_MAXW)) ? 4'd8 : 4'(NL);
    wire [1:0]  nlsh = (f_w <= 12'(BAND8_MAXW)) ? 2'd3 : 2'd2;        // log2 nl
    wire [BANDW-1:0] nbands = BANDW'((f_h + 12'(nl) - 12'd1) >> nlsh);
    reg  [BANDW-1:0] fband;                        // the band the fetcher walks
    reg  [COUNT_W-1:0] n_left;                     // the band's entries not yet issued
    reg  [COUNT_W-1:0] l_left;                     // the band's list lines not yet requested
    reg  [IDX_W-1:0] ftid;
    reg  [3:0]  fbi, fnb;                          // TRI bursts: the next to issue, the count (every visit)
    // the band's list lines: a FIFO of two (the line being issued from + the next); one line request pending at a time
    (* ram_style = "distributed" *) reg [255:0] ln [0:1];
    reg  [1:0]  ln_w, ln_r; reg [1:0] lane;        // lines landed, consumed; the next entry's lane in the head line
    reg         lq_v; reg [PTR_W-1:0] lq_ptr;      // a line to request (its first entry), known
    reg         lq_fly;                            // a requested line not landed yet
    wire [1:0]  ln_cnt = ln_w - ln_r;
    wire [63:0] ent_cur = ln[ln_r[0]][lane*64 +: 64];   // {flags {first, 0, TRI bursts}, next, {draw id, tri id}}
    // reads in flight: their kinds (and a header's visit slot) in issue order (the arbiter returns them in order), the count
    localparam [1:0] K_ENT = 2'd0, K_HDR = 2'd1, K_TRI = 2'd2;
    reg  [5:0]  kq [0:31]; reg [5:0] kq_w, kq_r;  // {slot[3:0], kind[1:0]}
    wire [5:0]  outstanding = kq_w - kq_r;
    reg  [5:0]  r_kind;                            // of the request on r_en / r_addr
    wire        r_acc = r_en && r_rdy;
    wire        can_issue = (!r_en || r_rdy) && (outstanding < 6'd30);   // a new request may be presented next cycle
    reg         fetch_on;                          // returns are taken (cleared by an abort: stale returns are dropped)
    wire [5:0]  kq_h = kq[kq_r[4:0]];
    wire        rt_hdr = r_valid && fetch_on && (kq_h[1:0] == K_HDR);    // a header return: into its visit slot
    wire        rt_ent = r_valid && fetch_on && (kq_h[1:0] == K_ENT);    // a list line return
    wire        line_go = lq_v && !lq_fly && (ln_cnt != 2'd2) && can_issue;   // the line request goes before any other
    // the burst FIFO (TRI bursts, in visit order): 32 lines (every visit carries its bursts: 4 to 6 a visit, so 32
    // keeps about 6 visits of look-ahead); `bres` = slots reserved for bursts issued, not landed
    // (block RAM rather than 8 192 bits of LUTRAM; the one read is registered into tb, so the RAM's output
    // register is tb itself, loaded in W_TRIW. bq_cnt != 0 keeps the read address off the write address.)
    (* ram_style = "block" *) reg [255:0] bq [0:31];
    reg  [5:0]  bq_w, bq_r; reg [5:0] bres;
    wire [5:0]  bq_cnt = bq_w - bq_r;
    wire [6:0]  bq_free = 7'd32 - 7'(bq_cnt) - 7'(bres);
    // the reservation changes in ONE assignment: a burst landing (- 1) and a new item's bursts reserved (+ nb) can meet in a
    // cycle, reads overlap (two assignments would lose the decrement: bres would grow, bq_free wrap, bursts be overwritten)
    wire        tri_rt  = r_valid && fetch_on && (kq_h[1:0] == K_TRI);                                  // a TRI burst lands
    wire        tri_rsv = (fs == F_TRI) && (fbi == 4'd1) && !(bq_free < 7'(fnb)) && can_issue && !line_go;   // F_TRI's first issue
    // the visit FIFO (16 deep): {kind, band, first, tid} and the header (LUTRAM, written when its read returns: rdy)
    localparam [1:0] V_BAND = 2'd0, V_PRIM = 2'd1, V_END = 2'd2;
    localparam int VQ = 16;
    reg  [1:0]  v_kind [0:VQ-1]; reg [BANDW-1:0] v_band [0:VQ-1]; reg v_first [0:VQ-1];
    reg  [IDX_W-1:0] v_tid [0:VQ-1]; reg [VQ-1:0] v_rdy;
    // v_hdr is block RAM (not 4 096 bits of LUTRAM feeding the popcount of beats_left in the same
    // cycle): hdr_q = the header at the head visit, read every cycle with the address it was read at (hdr_q_a). The walk
    // takes a primitive only when hdr_q is its own (hdr_q_a == vr: one cycle after a band marker moved v_r) and its v_rdy
    // has been set for a full cycle (v_rdy_q), so hdr_q never comes from the edge the header landed on (a block RAM's
    // read of the address being written is undefined, UG473). Costs one cycle after a band marker and one when a header
    // lands while the walk waits for it; the slot's stale v_rdy_q bit cannot pass a fresh primitive: v_rdy itself is 0
    // from its allocation until its header lands. Band markers and the end need no header and take v_rdy alone.
    // NOTE: Vivado 2026.1 calls ram_style = "block" "infeasible" for this array and keeps it in 43 RAM32M (≈ 170 LUT);
    // a 32-deep declaration and an enabled read (bq's pattern) change nothing — it stays LUTRAM, the attribute documents
    // the intent. bq and vtx_feed.d_const do convert.
    (* ram_style = "block" *) reg [255:0] v_hdr [0:VQ-1];
    reg  [255:0] hdr_q; reg [3:0] hdr_q_a; reg [VQ-1:0] v_rdy_q;
    reg  [4:0]  v_w, v_r;
    wire [3:0]  vr = v_r[3:0];
    wire        v_full = (v_w - v_r) == 5'(VQ), v_empty = (v_w == v_r);
    always @(posedge clk) begin
        if (rt_hdr) v_hdr[kq_h[5:2]] <= rd_data;
        if (rt_ent) ln[ln_w[0]] <= rd_data;
        hdr_q <= v_hdr[vr]; hdr_q_a <= vr; v_rdy_q <= v_rdy;
    end
    reg         go_pend;
    reg         f_stop;                            // the rasterizer ended the frame early (a soft abort): the fetcher stops

    // ---- the walk (the rasterizer's side)
    reg  [BANDW-1:0] band; reg [10:0] y0;
    reg  [IDX_W-1:0] tid, cur_tri; reg cur_ok;
    reg  [255:0] hdr; reg [255:0] tb; reg [1:0] k; reg [6:0] beats_left;
    reg  [6:0]   nvb;                              // the TRI item's varying beats (5 nv): the reference beats go in before them
    reg          tri_skip;                         // the visit's TRI bursts are dropped, not sent (the primitive is current)
    // the reference edge values at the centre of pixel (0, 0) — (8, 8) in 1/16 pixel — shifted by s and truncated to 32 bits (B
    // reconstructs a quad's values modulo 2^32: the true value of an emitted quad fits, A's s guarantees it)
    reg signed [17:0] cxr, cyr;                    // 8 - x0, 8 - y0
    reg  [31:0]  er [0:2];
    wire [1:0]  esh = hdr[17:16];
    wire [1:0]  prim = hdr[27:26];                 // 0 triangle, 1 line, 2 point
    wire [6:0]  lw = hdr[24:18];                   // a line's width (pixels)
    wire [10:0] ps = hdr[96 +: 11];                // a point's size (1/16 pixel)
    wire [7:0]  varmask = hdr[15:8];
    wire signed [15:0] SX0 = hdr[128 +: 16], SY0 = hdr[144 +: 16], SX1 = hdr[160 +: 16], SY1 = hdr[176 +: 16],
                       SX2 = hdr[192 +: 16], SY2 = hdr[208 +: 16];
    wire [BANDW-1:0] band_lo = hdr[224 +: BANDW];
    wire signed [17:0] PX18 = hdr[160 +: 18], PY18 = hdr[192 +: 18];   // a point's centre before the 16-bit saturation
    wire        a2_neg = hdr[127];                 // sign of a2 >> s (= sign of a2)
    // the primitive's rectangle [x0, x1) x [y0, y1) in pixels (a_setup hdr_word: its draw's scissor box cut to
    // the target, the whole target without one, never empty): the walk's bounds. Board B's scissor test masks everything
    // outside it, so nothing outside is walked or sent.
    wire [11:0] h_rx0 = hdr[52 +: 12], h_ry0 = hdr[84 +: 12], h_rx1 = {hdr[250 +: 6], hdr[234 +: 6]}, h_ry1 = {hdr[7:0], hdr[80 +: 4]};

    // ---- the rasterizer's primitive state (triangles and points share the quad walk)
    reg signed [16:0] dx1, dy1, dx2, dy2;
    reg signed [23:0] A [0:2], B [0:2];            // per-pixel edge steps (x, y), 1/256 pixel^2
    reg signed [EW-1:0] q1, q2, a2;
    reg front; reg tl [0:2];
    reg signed [17:0] xmin_u, xmax_u, ymin_u, ymax_u;   // 1/16 pixel; a point's square may leave 16 bits
    reg [11:0] xmin_px, xmax_px, ymin_px, ymax_px; reg bbox_ok;
    // a triangle's / point's bounding box is in the target (the sequencer's S2); bbox_ok is a line's (W_LS2B). Two registers
    // with one writer each (one register written by both puts a mux level on the line walk's scr_w -> lc_hi -> bbox_ok
    // path)
    reg tbox_ok;
    // after the TRI item (sent or skipped): lines to their walk, the others to the quad rows (the setup sequencer ran beside
    // the item; tri_end is taken only once it is done — tbox_ok is its S2's)
    st_t tri_end;
    always @* tri_end = (prim == 2'd1) ? W_LS1 : (tbox_ok ? W_ROW : W_NEXT);
    reg [10:0] qx, qx_max, qy; reg [2:0] rows_left;    // quad rows left in the band (nl / 2: 2 or 4)
    reg signed [17:0] cxo, cyo;                    // (centre - vertex 0) of the band's first row's first quad pixel 0 (SP_W1)
    reg signed [EW-1:0] p1, p2, p3, p4, Ec [0:2];
    // rs: the edge values at pixel 0 of the first quad of quad row qy (every row of a visit starts at xmin_px & ~1). The
    // sequencer sets them for the band's first row (SP_W3, the products as the rows had them); each step of qy adds two
    // pixel rows' y steps, 2 B_k — exact: the row values are linear in cyo (+32 per quad row), B_1 = -16 dx2, B_2 = 16 dx1,
    // B_0 = -B_1 - B_2, a point's square B = (0, 0, -16); all EW-bit integers
    reg signed [EW-1:0] rs [0:2];
    // the other three lanes' edge values as RUNNING SUMS (Ec + A, Ec + B, Ec + A + B), stepped with Ec: the coverage
    // compare then has no adder in front of it (Ec -> adders -> compares -> cov is too long a path)
    reg signed [EW-1:0] EcA [0:2], EcB [0:2], EcAB [0:2];
    reg signed [17:0] pxl, pyt;                    // a point's left edge, top edge (1/16 pixel)
    function automatic signed [15:0] min3(input signed [15:0] a, b, c); min3 = (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c); endfunction
    function automatic signed [15:0] max3(input signed [15:0] a, b, c); max3 = (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c); endfunction
    function automatic [11:0] clampx(input signed [17:0] u, input [11:0] lo, input [11:0] hi);   // pixel of a 1/16 coordinate, clamped to lo .. hi - 1
        reg signed [17:0] r; r = u >>> 4;
        clampx = (r < $signed({6'd0, lo})) ? lo : (r >= $signed({6'd0, hi})) ? hi - 1'b1 : r[11:0];
    endfunction
    // a pixel of the quad at (qx, qy) is inside the box xmin_px .. xmax_px, ymin_px .. ymax_px (the primitive's bounding
    // box cut to its rectangle; a covered pixel is in the bounding box anyway, so this is the rectangle's test and so
    // the target's). No compare on the walk's path: the rows' two bits are registered as the row is entered
    // (W_ROW); a column below xmin_px can only be the left one of the row's first quad (xmin_px odd), one above xmax_px
    // only the right one of its last (xmax_px even), and `qx < qx_max` is the compare the row's end already makes.
    reg         row_ok0, row_ok1, x_first;
    wire        xok0 = !(x_first && xmin_px[0]);
    wire        xok1 = (qx < qx_max) || xmax_px[0];
    // ---- EVAL: the 4 lanes' edge values, coverage with the tie rule (triangles) or the square (points), the pixel-0 values shifted
    wire signed [EW-1:0] eL [0:3][0:2];
    genvar gk;
    generate for (gk = 0; gk < 3; gk = gk + 1) begin : g_lane
        assign eL[0][gk] = Ec[gk];
        assign eL[1][gk] = EcA[gk];
        assign eL[2][gk] = EcB[gk];
        assign eL[3][gk] = EcAB[gk];
    end endgenerate
    // the tie rule from signs alone (a 36-bit negation, then a compare, would be two carry chains: too slow):
    // n = (fr ? e : -e) > 0 is fr ? (e > 0) : (e < 0), n == 0 is e == 0 — exact for |e| < 2^35 (the edge range)
    function automatic edge_ok(input signed [EW-1:0] e, input fr, input t);
        reg z; z = (e == '0);
        edge_ok = (fr ? (!e[EW-1] && !z) : e[EW-1]) || (z && t);
    endfunction
    // a point's square: 0 <= e < ps and 0 < e <= ps with ps < 2^11 — the high bits zero, then an 11-bit compare
    function automatic pt_in(input signed [EW-1:0] e, input [10:0] s, input open_lo);
        pt_in = (e[EW-1:11] == '0) && (open_lo ? (e[10:0] != 11'd0 && e[10:0] <= s) : (e[10:0] < s));
    endfunction
    wire [11:0] px0 = {qx, 1'b0}, py0 = {qy, 1'b0};
    reg  [3:0] cov_c;
    always @* begin
        for (int p = 0; p < 4; p++)
            cov_c[p] = (p[0] ? xok1 : xok0) && (p[1] ? row_ok1 : row_ok0) &&
                       ((prim == 2'd2) ? (pt_in(eL[p][1], ps, 1'b0) && pt_in(eL[p][2], ps, 1'b1))
                                       : (edge_ok(eL[p][0], front, tl[0]) && edge_ok(eL[p][1], front, tl[1]) && edge_ok(eL[p][2], front, tl[2])));
    end
    // the quad loop's second stage (one quad position per cycle): the position evaluated last cycle,
    // pushed while the next one is evaluated
    reg p_v; reg [3:0] p_cov; reg [31:0] p_e [0:2]; reg [10:0] p_qx, p_qy;
    wire p_push = p_v && (p_cov != 4'd0);
    // a row ends early once both of its pixel rows are past the triangle: at the quad's right pixel (lanes 1 and 3) some edge
    // is outside (normalised value < 0) and does not grow with x (normalised step <= 0), so no later quad of the row is covered
    // (exact: the tie rule needs a value of 0, not < 0). Points have no such edges: their square is their box.
    // (signs only, as edge_ok; e_nogrow registered per row in W_ROW: the step's sign does not change within a primitive)
    reg  [2:0] e_nogrow; wire [2:0] dead_t, dead_b;
    generate for (gk = 0; gk < 3; gk = gk + 1) begin : g_dead
        assign dead_t[gk] = e_nogrow[gk] && (front ? EcA[gk][EW-1] : (!EcA[gk][EW-1] && EcA[gk] != '0));
        assign dead_b[gk] = e_nogrow[gk] && (front ? EcAB[gk][EW-1] : (!EcAB[gk][EW-1] && EcAB[gk] != '0));
    end endgenerate
    wire row_dead = (prim != 2'd2) && (|dead_t) && (|dead_b);
    function automatic [31:0] shr_s(input signed [EW-1:0] e, input [1:0] s);
        reg signed [EW-1:0] r; r = e >>> s; shr_s = r[31:0];
    endfunction

    // ---- lines: the major / minor walk
    reg         xmaj;                              // x-major: columns walked, rows computed
    reg signed [15:0] ma, na;                      // the start's major / minor coordinate (1/16 pixel)
    reg signed [16:0] dm, dn;                      // b - a along the major / minor axis
    reg signed [EW-1:0] len2;                      // dx^2 + dy^2 (1/256 pixel^2)
    reg [11:0]  lc_lo, lc_hi, cc;                  // the major pixel range, the current major pixel
    reg signed [21:0] den, lstep;                  // 16 |dm|; 16 dn sign(dm): the numerator's step per major pixel
    reg signed [EW-1:0] num;                       // (na - 8 (lw - 1)) dm + (16 c + 8 - ma) dn, sign(dm) applied
    reg signed [EW-1:0] lp1, lp2;
    reg signed [17:0] lo_rq, hi_rq;                // the major range before the clamps (registered: timing)
    reg         lp_empty;                          // the pair has no quad in the band (W_LPAIR -> W_LPAIR2)
    reg [EW-1:0] dv_n, dv_q; reg [21:0] dv_r; reg [5:0] dv_i; reg dv_neg;   // the restoring divider: |num| / den
    reg signed [13:0] q_n; reg signed [21:0] r_n;  // the current major pixel's first minor pixel and the remainder
    reg signed [13:0] coln [0:1]; reg colv [0:1];  // the pair's two major pixels: first minor pixel, in range
    reg [10:0]  qm, qn, qn_lo, qn_hi;              // the quad along the major / minor axis and the minor range
    reg signed [13:0] lo_nq, hi_nq;                // the pair's minor quad range before the clamps (registered: timing)
    reg signed [17:0] l_lo, l_hi;                  // the major pixel range's bounds, the minor quad range's (W_LS2): the rectangle,
    reg signed [13:0] n_lo, n_hi;                  //   its rows cut to the band's
    wire [11:0] b_ylo  = (h_ry0 > {1'b0, y0}) ? h_ry0 : {1'b0, y0};                       // the band's rows in the rectangle
    wire [11:0] b_yend = {1'b0, y0} + 12'(nl) - 12'd1;
    wire [11:0] b_yhi  = ((h_ry1 - 12'd1) < b_yend) ? (h_ry1 - 12'd1) : b_yend;
    wire signed [13:0] lw14 = $signed({7'd0, lw});
    wire signed [13:0] nmin = (colv[0] && colv[1]) ? ((coln[0] < coln[1]) ? coln[0] : coln[1]) : colv[0] ? coln[0] : coln[1];
    wire signed [13:0] nmax = ((colv[0] && colv[1]) ? ((coln[0] < coln[1]) ? coln[1] : coln[0]) : colv[0] ? coln[0] : coln[1]) + lw14 - 14'sd1;
    wire [11:0] lqx_c = xmaj ? 12'(qm) : 12'(qn), lqy_c = xmaj ? 12'(qn) : 12'(qm);
    reg  [11:0] lqx, lqy;                          // the quad's x / y: registered in W_LQ0 (timing: the major / minor mux would feed the DSP)
    reg signed [17:0] lcx, lcy;                    // 32 q + 8 - a, e1's operands at the quad's pixel 0: registered in W_LQ0
    reg  [3:0] lcov_c;
    always @* begin
        for (int p = 0; p < 4; p++) begin : LC
            reg [11:0] px, py; reg [0:0] ci; reg signed [13:0] mn;
            px = {lqx[10:0], p[0]}; py = {lqy[10:0], p[1]};
            ci = xmaj ? p[0] : p[1];                                       // the lane's major pixel within the pair
            mn = xmaj ? 14'(py) : 14'(px);                                 // the lane's minor pixel
            lcov_c[p] = colv[ci] && (mn >= coln[ci]) && (mn < coln[ci] + lw14) && (px >= h_rx0) && (px < h_rx1) && (py >= h_ry0) && (py < h_ry1);
        end
    end
    // e1 = (c0 - a) . (b - a) at the quad's pixel 0 (two products of lcx / lcy; dx = xb - xa = dx1, dy = dy1)
    wire signed [21:0] r_step = r_n + lstep;       // the remainder after one major pixel (|lstep| <= den: one adjust)
    reg  [3:0] lcov; reg [31:0] le_s [0:2];

    // ================================================================ the fetcher and the read returns
    always @(posedge clk) begin
        if (r_acc) begin r_en <= 1'b0; kq[kq_w[4:0]] <= r_kind; kq_w <= kq_w + 6'd1; end
        if (rst) begin
            fs <= F_IDLE; r_en <= 1'b0; kq_w <= 6'd0; kq_r <= 6'd0; fetch_on <= 1'b0; lq_v <= 1'b0; lq_fly <= 1'b0;
            ln_w <= 2'd0; ln_r <= 2'd0; bq_w <= 6'd0; bres <= 6'd0; v_w <= 5'd0; v_rdy <= '0; q_band <= '0; q_bank <= 1'b0;
        end else begin
            // the returns, in issue order (dropped after an abort: stale). A header lands in its visit slot (rt_hdr, the
            // LUTRAM write above), a line in the line FIFO; a line's last entry points at the band's next line
            if (r_valid) begin
                kq_r <= kq_r + 6'd1;
                if (rt_hdr) v_rdy[kq_h[5:2]] <= 1'b1;
                if (rt_ent) begin
                    ln_w <= ln_w + 2'd1; lq_fly <= 1'b0;
                    if (l_left != '0) begin lq_v <= 1'b1; lq_ptr <= rd_data[224 +: PTR_W]; l_left <= l_left - 1'b1; end
                end
                if (tri_rt) begin bq[bq_w[4:0]] <= rd_data; bq_w <= bq_w + 6'd1; end
            end
            if (tri_rt || tri_rsv) bres <= bres - 6'(tri_rt) + (tri_rsv ? 6'(fnb) : 6'd0);   // (the resets below win)
            if (abort || f_stop) begin                         // stale returns drain into nothing; no new reads until they have
                fs <= F_IDLE; fetch_on <= 1'b0; r_en <= 1'b0; bq_w <= 6'd0; bres <= 6'd0; v_w <= 5'd0; v_rdy <= '0;
                lq_v <= 1'b0; lq_fly <= 1'b0; ln_w <= 2'd0; ln_r <= 2'd0;
            end else begin
              // a band's next list line: requested as soon as its pointer is known and the FIFO has a place for it
              if (line_go) begin
                  r_en <= 1'b1; r_addr <= pool_base + {lq_ptr[PTR_W-1:2], 3'b000}; r_kind <= {4'd0, K_ENT}; lq_v <= 1'b0; lq_fly <= 1'b1;
              end
              case (fs)
                F_IDLE: if ((go || go_pend) && outstanding == 6'd0 && !r_en) begin
                            fetch_on <= 1'b1; q_bank <= go_bank; fband <= '0; ln_w <= 2'd0; ln_r <= 2'd0; lq_v <= 1'b0; lq_fly <= 1'b0;
                            bq_w <= 6'd0; bres <= 6'd0; v_w <= 5'd0; v_rdy <= '0; fs <= F_BAND;
                        end
                // a band: its marker into the visit queue (ready at once), then its list
                F_BAND: if (!v_full) begin
                            v_kind[v_w[3:0]] <= (fband == nbands) ? V_END : V_BAND; v_band[v_w[3:0]] <= fband; v_rdy[v_w[3:0]] <= 1'b1;
                            v_w <= v_w + 5'd1;
                            if (fband == nbands) fs <= F_IDLE;
                            else begin q_band <= fband; fs <= F_QRY; end
                        end
                F_QRY:  fs <= F_QRY2;                                        // the binner registers heads[q_band] this edge
                F_QRY2: begin                                                // the first line (a band's lines hold 4 entries each)
                            n_left <= q_count; lane <= 2'd0;
                            if (q_count != '0) begin lq_v <= 1'b1; lq_ptr <= q_head; l_left <= COUNT_W'((q_count - 1'b1) >> 2); end
                            fs <= F_ENT;
                        end
                // an entry: its header read, its visit queued (not ready: the header lands later), its TRI bursts next — in
                // every band (B does not re-read a revisited triangle's record from its DDR: A sends the TRI
                // item again). The line goes when its last entry has been taken.
                F_ENT:  if (n_left == '0) begin fband <= fband + 1'b1; fs <= F_BAND; end
                        else if (ln_cnt != 2'd0 && !v_full && can_issue && !line_go) begin
                            r_en <= 1'b1; r_addr <= rec_base + {ent_cur[IDX_W-1:0], 7'd0}; r_kind <= {v_w[3:0], K_HDR};
                            v_kind[v_w[3:0]] <= V_PRIM; v_first[v_w[3:0]] <= ent_cur[63]; v_tid[v_w[3:0]] <= ent_cur[IDX_W-1:0];
                            v_rdy[v_w[3:0]] <= 1'b0; v_w <= v_w + 5'd1;
                            ftid <= ent_cur[IDX_W-1:0]; fnb <= ent_cur[59:56]; fbi <= 4'd1;
                            n_left <= n_left - 1'b1;
                            if (lane == 2'd3 || n_left == COUNT_W'(1)) begin ln_r <= ln_r + 2'd1; lane <= 2'd0; end
                            else lane <= lane + 2'd1;
                            fs <= F_TRI;
                        end
                // the record's TRI bursts back to back, once the burst FIFO has room for all of them
                F_TRI:  if (fbi == 4'd1 && bq_free < 7'(fnb)) ;             // wait for room
                        else if (can_issue && !line_go) begin
                            r_en <= 1'b1; r_addr <= rec_base + {ftid, 7'd0} + {fbi, 3'd0}; r_kind <= {4'd0, K_TRI};
                            fbi <= fbi + 4'd1;              // (the reservation at fbi == 1: tri_rsv above)
                            if (fbi == fnb) fs <= F_ENT;
                        end
                default: fs <= F_IDLE;
              endcase
            end
        end
    end

    // ================================================================ the rasterizer
    // Its items go into the output queue (oq_push: one beat, or a quad's two beats), which sends them out as the render
    // stream (below).
    localparam int OQD = 8;
    // The beats in LUTRAM: written from the registered push, one cycle after it, in a block of their own (one write port);
    // read asynchronously once written (oq_ww).
    (* ram_style = "distributed" *) reg [63:0] oq_b0 [0:OQD-1];
    (* ram_style = "distributed" *) reg [63:0] oq_b1 [0:OQD-1];
    reg oq_k [0:OQD-1], oq_fe [0:OQD-1];
    reg  [3:0]  oq_wp, oq_rp, oq_ww;
    wire        oq_room = (oq_wp - oq_rp) < 4'(OQD);
    wire        q_stall = p_push && !oq_room;          // the quad loop: the queue is full, both stages hold
    reg         oq_pv; reg [2:0] oq_pa; reg [63:0] oq_pb0, oq_pb1;   // the push, one cycle later: the beats' write
    reg          soft_pend;                                   // a soft abort waits for a visit boundary (f_stop: the fetcher)
    task automatic oq_push(input bit quad, input bit fe, input [63:0] b0, input [63:0] b1);
        oq_k[oq_wp[2:0]] <= quad; oq_fe[oq_wp[2:0]] <= fe; oq_wp <= oq_wp + 4'd1;
        oq_pv <= 1'b1; oq_pa <= oq_wp[2:0]; oq_pb0 <= b0; oq_pb1 <= b1;
    endtask
    always @(posedge clk) if (oq_pv) begin oq_b0[oq_pa] <= oq_pb0; oq_b1[oq_pa] <= oq_pb1; end
    // the burst FIFO's read: tb is the block RAM's output register (W_TRIW takes a burst when one has landed)
    // the next burst is also read on a burst's 4th beat (W_TRIE, k = 3) when it has landed, so a TRI item
    // streams one beat a cycle across its bursts; W_TRIW only waits for a burst that has not landed (a wait there for
    // every burst would cost one cycle per 4 beats). The beat goes out from the old tb in the same edge.
    wire tb_next = oq_room && (bq_cnt != 6'd0) &&
                   ((st == W_TRIE && k == 2'd3 && beats_left != 7'd1) ||
                    (st == W_TRIR && !sp_busy && beats_left != 7'd0 && k == 2'd0));
    always @(posedge clk) if ((st == W_TRIW && bq_cnt != 6'd0 && !tri_skip) || tb_next) tb <= bq[bq_r[4:0]];
    always @(posedge clk) begin
        oq_pv <= 1'b0; f_stop <= 1'b0;
        if (rst) begin
            st <= W_IDLE; c_quads <= 0; c_tris <= 0; c_vis <= 0; cur_ok <= 1'b0; go_pend <= 1'b0; v_r <= 5'd0; bq_r <= 6'd0; oq_wp <= 4'd0;
            soft_pend <= 1'b0; p_v <= 1'b0; sp <= SP_IDLE;
        end else if (abort) begin                                // the dispatcher gives the frame up
            st <= W_IDLE; go_pend <= 1'b0; v_r <= 5'd0; bq_r <= 6'd0; oq_wp <= 4'd0; soft_pend <= 1'b0; p_v <= 1'b0;   // (the queue's other pointers: below)
            sp <= SP_IDLE;
        end else begin
          if (abort_soft && st != W_IDLE) soft_pend <= 1'b1;
          // the quad loop's second stage: the position evaluated last cycle goes into the queue if it covers a pixel (only
          // W_EVAL fills it; W_ROW, which follows it, pushes nothing and leaves the band only once it is empty: no other push
          // meets it)
          if (p_push && oq_room) begin                           // one beat, no edge values
              oq_push(1'b0, 1'b0, {32'd0, 4'd1, p_cov, 12'(p_qy), 12'(p_qx)}, 64'd0); c_quads <= c_quads + 1; p_v <= 1'b0;
          end else if (p_v && !p_push) p_v <= 1'b0;
          case (st)
            W_IDLE: begin
                if (go) go_pend <= 1'b1;
                if ((go || go_pend) && fs == F_IDLE && outstanding == 6'd0 && !r_en) begin   // the fetcher starts this cycle too
                    go_pend <= 1'b0; cur_ok <= 1'b0; c_quads <= 0; c_tris <= 0; v_r <= 5'd0; bq_r <= 6'd0; st <= W_VISIT;
                end
            end
            // ---------------------------------------------------------------- the next visit: a band marker, a primitive, the end
            // a soft abort ends the frame here, between two visits (the stream is at an item boundary): FRAME_END with its
            // abort bit, the fetcher stopped. A primitive waits for its header.
            W_VISIT: if (soft_pend) begin
                        if (oq_room) begin oq_push(1'b0, 1'b1, {32'h0, 4'd5, 27'd0, 1'b1}, 64'd0); soft_pend <= 1'b0; f_stop <= 1'b1; st <= W_IDLE; end
                     end else if (!v_empty && v_rdy[vr] && (v_kind[vr] != V_PRIM || (v_rdy_q[vr] && hdr_q_a == vr))) begin
                        if (v_kind[vr] == V_BAND) begin band <= v_band[vr]; y0 <= 11'(v_band[vr]) << nlsh; st <= W_BANDB; end
                        else if (v_kind[vr] == V_END) begin v_r <= v_r + 5'd1; st <= W_FEND; end
                        else begin : VP
                            reg [6:0] nvb_c;
                            nvb_c = 7'd5 * 7'(4'(hdr_q[8]) + 4'(hdr_q[9]) + 4'(hdr_q[10]) + 4'(hdr_q[11]) +
                                              4'(hdr_q[12]) + 4'(hdr_q[13]) + 4'(hdr_q[14]) + 4'(hdr_q[15]));
                            hdr <= hdr_q; tid <= v_tid[vr]; v_r <= v_r + 5'd1;
                            beats_left <= 7'd11 + nvb_c; nvb <= nvb_c;
                            // the TRI item whenever the primitive is not B's current one (its first band, or another came
                            // between); else its bursts are only dropped (a primitive whose two bands' visits follow each other).
                            // The setup sequencer starts with the item (the reference edge values: in it after beat 11)
                            tri_skip <= !(v_first[vr] || !cur_ok || v_tid[vr] != cur_tri); st <= W_TRIW; sp <= SP_S1;
                        end
                     end
            W_BANDB: if (oq_room) begin                                  // the BAND item
                        oq_push(1'b0, 1'b0, {32'h0, 4'd4, 17'd0, y0}, 64'd0); v_r <= v_r + 5'd1; st <= W_VISIT;
                     end
            // ---------------------------------------------------------------- TRI item: the record's bursts from the burst FIFO as beats
            W_TRIW: if (bq_cnt != 6'd0) begin
                        bq_r <= bq_r + 6'd1;
                        if (tri_skip) begin if (beats_left <= 7'd4) st <= sp_busy ? W_SWAIT : tri_end; else beats_left <= beats_left - 7'd4; end   // a burst a cycle
                        else begin k <= 2'd0; st <= W_TRIE; end          // (tb loads from the burst FIFO above)
                    end
            W_TRIE: if (oq_room) begin
                        oq_push(1'b0, 1'b0, tb[k*64 +: 64], 64'd0); k <= k + 2'd1; beats_left <= beats_left - 7'd1;
                        if (beats_left - 7'd1 == nvb) st <= W_TRIR;      // the header's 11th beat went: the references
                        else if (beats_left == 7'd1) st <= sp_busy ? W_SWAIT : tri_end;
                        else if (k == 2'd3) begin
                            if (tb_next) bq_r <= bq_r + 6'd1;            // the next burst read with this beat (k wraps to 0)
                            else st <= W_TRIW;                           // not landed yet: wait for it
                        end
                    end
            // the two reference beats {er1, er0}, {0, er2} as ONE queue entry (the queue's two-beat form: the stream still
            // carries two beats, the walk spends one cycle) once the sequencer has them
            W_TRIR: if (oq_room && !sp_busy) begin
                        oq_push(1'b1, 1'b0, {er[1], er[0]}, {32'd0, er[2]});
                        // k wrapped: the next burst (read here if it has landed); the 11th beat is k = 2, so k = 3 here
                        if (beats_left == 7'd0) st <= tri_end;
                        else if (k != 2'd0) st <= W_TRIE;
                        else if (tb_next) begin bq_r <= bq_r + 6'd1; st <= W_TRIE; end
                        else st <= W_TRIW;
                    end
            W_SWAIT: if (!sp_busy) st <= tri_end;                        // the item was shorter than the setup (a skipped TRI)
            // ---------------------------------------------------------------- a quad row of the band: its first quad from rs
            // (one cycle a row: no origin, products, sums or lanes of its own, and no wait for the previous row's last
            // quad). A row that is entered goes to W_EVAL at once — its first position
            // pushes the previous row's last quad as within a row; only the band's end waits for that quad (W_NEXT is
            // followed by pushes of other items).
            W_ROW:  if (rows_left == 2'd0) begin if (!p_v) st <= W_NEXT; end
                    else begin
                        rows_left <= rows_left - 2'd1;
                        // the tie-rule owners (normalised steps > 0, or 0 with y > 0)
                        for (int i = 0; i < 3; i++) tl[i] <= front ? (A[i] > 0 || (A[i] == 0 && B[i] > 0)) : (A[i] < 0 || (A[i] == 0 && B[i] < 0));
                        for (int i = 0; i < 3; i++) e_nogrow[i] <= front ? (A[i] <= 0) : (A[i] >= 0);   // the normalised x step <= 0
                        if ({qy, 1'b1} < ymin_px || {qy, 1'b0} > ymax_px) begin   // the row misses the primitive
                            qy <= qy + 11'd1;
                            for (int i = 0; i < 3; i++) rs[i] <= rs[i] + EW'(B[i] <<< 1);
                        end else begin
                            qx <= 11'(xmin_px >> 1); x_first <= 1'b1;
                            row_ok0 <= ({qy, 1'b0} >= ymin_px) && ({qy, 1'b0} <= ymax_px);   // the row's two pixel rows in the box
                            row_ok1 <= ({qy, 1'b1} >= ymin_px) && ({qy, 1'b1} <= ymax_px);
                            for (int i = 0; i < 3; i++) begin
                                Ec[i] <= rs[i]; EcA[i] <= rs[i] + EW'(A[i]); EcB[i] <= rs[i] + EW'(B[i]); EcAB[i] <= rs[i] + EW'(A[i]) + EW'(B[i]);
                            end
                            st <= W_EVAL;
                        end
                    end
            // ---------------------------------------------------------------- the quads of the row, one position per cycle: the
            // coverage and the shifted pixel-0 values of this position into the second stage (p_*), which pushes the previous
            // one (a quad's two beats in one push) at the top of this block
            W_EVAL: if (!q_stall) begin
                p_v <= 1'b1; p_cov <= cov_c; p_qx <= qx; p_qy <= qy;
                for (int i = 0; i < 3; i++) p_e[i] <= shr_s(Ec[i], esh);
                if (qx < qx_max && !row_dead) begin
                    qx <= qx + 11'd1; x_first <= 1'b0;
                    for (int i = 0; i < 3; i++) begin
                        Ec[i] <= Ec[i] + EW'(A[i] <<< 1); EcA[i] <= EcA[i] + EW'(A[i] <<< 1);
                        EcB[i] <= EcB[i] + EW'(A[i] <<< 1); EcAB[i] <= EcAB[i] + EW'(A[i] <<< 1);
                    end
                end else begin
                    qy <= qy + 11'd1; st <= W_ROW;
                    for (int i = 0; i < 3; i++) rs[i] <= rs[i] + EW'(B[i] <<< 1);   // the next row's first quad
                end
            end
            // ---------------------------------------------------------------- lines: setup, the major range, the first division
            W_LS1: begin
                lp1 <= dx1 * dx1; lp2 <= dy1 * dy1;
                xmaj <= ((dx1 < 0) ? -dx1 : dx1) >= ((dy1 < 0) ? -dy1 : dy1);
                st <= W_LS2;
            end
            // (three cycles: the range, its clamps, the numerator's second product — one cycle would be far too long)
            W_LS2: begin : LS2
                reg signed [15:0] a_m, b_m, a_n; reg signed [16:0] d_m, d_n;
                len2 <= lp1 + lp2;
                if (xmaj) begin a_m = SX0; b_m = SX1; a_n = SY0; d_m = dx1; d_n = dy1; end
                else      begin a_m = SY0; b_m = SY1; a_n = SX0; d_m = dy1; d_n = dx1; end
                ma <= a_m; na <= a_n; dm <= d_m; dn <= d_n;
                // the major pixels whose centres lie in [a, b) (d_m > 0) or (b, a] (d_m < 0) — clamped next cycle
                if (d_m > 0) begin lo_rq <= (18'(a_m) + 18'sd7) >>> 4; hi_rq <= (18'(b_m) - 18'sd9) >>> 4; end
                else         begin lo_rq <= (18'(b_m) + 18'sd8) >>> 4; hi_rq <= (18'(a_m) - 18'sd8) >>> 4; end
                den <= 22'((d_m < 0) ? -d_m : d_m) <<< 4;
                lstep <= (d_m < 0) ? -(22'(d_n) <<< 4) : (22'(d_n) <<< 4);
                // the walk's bounds along the major and the minor axis, registered a cycle ahead of their clamps: the
                // rectangle's columns, and its rows cut to the band's (b_ylo .. b_yhi, never empty: a_setup bins a primitive
                // only into the bands its rectangle covers)
                l_lo <= xmaj ? $signed({6'd0, h_rx0}) : $signed({6'd0, b_ylo});
                l_hi <= xmaj ? $signed({6'd0, h_rx1}) - 18'sd1 : $signed({6'd0, b_yhi});
                n_lo <= xmaj ? $signed({3'd0, b_ylo[11:1]}) : $signed({3'd0, h_rx0[11:1]});
                n_hi <= xmaj ? $signed({3'd0, b_yhi[11:1]}) : $signed(14'((h_rx1 - 12'd1) >> 1));
                st <= W_LS2B;
            end
            W_LS2B: begin : LS2B                                         // the clamps to the rectangle (y-major: to the band's rows in it)
                reg signed [17:0] lo_c, hi_c;
                lo_c = (lo_rq < l_lo) ? l_lo : lo_rq;
                hi_c = (hi_rq > l_hi) ? l_hi : hi_rq;
                lc_lo <= lo_c[11:0]; lc_hi <= hi_c[11:0]; cc <= lo_c[11:0];
                bbox_ok <= !(lo_c > hi_c) && !(hi_rq < 0);
                // the numerator at the first major pixel: (na - 8 (lw - 1)) dm + (16 lo + 8 - ma) dn — the first product, from
                // the registered na / dm (timing: not from the xmaj mux)
                lp1 <= (18'(na) - $signed(18'({lw, 3'd0})) + 18'sd8) * dm;
                st <= W_LS2C;
            end
            W_LS2C: begin                                                // the numerator's second product
                lp2 <= ($signed(18'({lc_lo, 4'd8})) - 18'(ma)) * dn;
                st <= W_LS3;
            end
            W_LS3: if (!bbox_ok) st <= W_NEXT;
                   else begin num <= (dm < 0) ? -(lp1 + lp2) : (lp1 + lp2); st <= W_LDIV0; end
            W_LDIV0: begin                                               // |num| / den, restoring, EW cycles
                dv_neg <= num[EW-1]; dv_n <= num[EW-1] ? -num : num; dv_q <= '0; dv_r <= '0; dv_i <= 6'(EW); st <= W_LDIV;
            end
            W_LDIV: begin : DIV
                reg [22:0] t;
                t = {dv_r[21:0], dv_n[EW-1]};
                if (t >= {1'b0, den[21:0]}) begin dv_r <= 22'(t - {1'b0, den[21:0]}); dv_q <= {dv_q[EW-2:0], 1'b1}; end
                else begin dv_r <= t[21:0]; dv_q <= {dv_q[EW-2:0], 1'b0}; end
                dv_n <= {dv_n[EW-2:0], 1'b0};
                dv_i <= dv_i - 6'd1;
                if (dv_i == 6'd1) st <= W_LDIV1;
            end
            W_LDIV1: begin                                               // floor for a negative numerator
                if (dv_neg) begin
                    q_n <= (dv_r != 0) ? -14'(dv_q) - 14'd1 : -14'(dv_q);
                    r_n <= (dv_r != 0) ? $signed(den) - $signed({1'b0, dv_r[21:0]}) : 22'sd0;
                end else begin q_n <= 14'(dv_q); r_n <= $signed({1'b0, dv_r[21:0]}); end
                colv[0] <= 1'b0; colv[1] <= 1'b0; st <= W_LCOL;
            end
            // ---------------------------------------------------------------- lines: a major pixel's first minor pixel, the pair's quads
            W_LCOL: begin
                coln[cc[0]] <= q_n; colv[cc[0]] <= 1'b1;
                if (cc[0] == 1'b1 || cc == lc_hi) st <= W_LPAIR;
                else begin                                               // the next major pixel: + lstep, one adjust
                    if (r_step >= den) begin r_n <= r_step - den; q_n <= q_n + 14'd1; end
                    else if (r_step < 0) begin r_n <= r_step + den; q_n <= q_n - 14'd1; end
                    else r_n <= r_step;
                    cc <= cc + 12'd1;
                end
            end
            W_LPAIR: begin                                               // the pair's minor quad range before the clamps (timing: the
                qm <= 11'(cc >> 1);                                      // two columns' min / max + the clamps are too long for one cycle)
                lo_nq <= nmin >>> 1; hi_nq <= nmax >>> 1;
                st <= W_LPAIRB;
            end
            W_LPAIRB: begin : LP                                         // the clamps: x-major to the band's quad rows in the rectangle, y-major to its columns
                reg signed [13:0] lo_q, hi_q;                            // (n_lo / n_hi: registered in W_LS2)
                lo_q = lo_nq; hi_q = hi_nq;
                if (lo_q < n_lo) lo_q = n_lo;
                if (hi_q > n_hi) hi_q = n_hi;
                qn_lo <= lo_q[10:0]; qn_hi <= hi_q[10:0]; qn <= lo_q[10:0];
                lp_empty <= (lo_q > hi_q);                               // registered: the compare would feed the state decode
                st <= W_LPAIR2;
            end
            W_LPAIR2: st <= lp_empty ? W_LPEND : W_LQ0;
            W_LQ0: begin                                                 // the quad's position and e1's operands, registered (timing: qn ->
                lqx <= lqx_c; lqy <= lqy_c;                              // the major / minor mux -> the 18-bit subtract -> DSP is too long)
                lcx <= $signed(18'({lqx_c[10:0], 5'd8})) - 18'(SX0);     // 32 q + 8 - a
                lcy <= $signed(18'({lqy_c[10:0], 5'd8})) - 18'(SY0);
                st <= W_LQ;
            end
            W_LQ: begin                                                  // the quad's coverage and e1's two products, registered
                lcov <= lcov_c;
                lp1 <= lcx * dx1; lp2 <= lcy * dy1;                      // (lp1 / lp2 are free after W_LS3)
                st <= W_LQ1;
            end
            W_LQ1: begin                                                 // the quad's edge values, shifted
                le_s[1] <= shr_s(lp1 + lp2, esh); le_s[0] <= shr_s(len2 - (lp1 + lp2), esh); le_s[2] <= 32'd0;
                st <= W_LQA;
            end
            W_LQA: if (lcov == 4'd0 || oq_room) begin
                       if (lcov != 4'd0) begin oq_push(1'b0, 1'b0, {32'd0, 4'd1, lcov, lqy, lqx}, 64'd0); c_quads <= c_quads + 1; end   // one beat
                       if (qn < qn_hi) begin qn <= qn + 11'd1; st <= W_LQ0; end
                       else st <= W_LPEND;
                   end
            W_LPEND: begin
                colv[0] <= 1'b0; colv[1] <= 1'b0;
                if (cc < lc_hi) begin                                    // on to the next pair's first major pixel
                    if (r_step >= den) begin r_n <= r_step - den; q_n <= q_n + 14'd1; end
                    else if (r_step < 0) begin r_n <= r_step + den; q_n <= q_n - 14'd1; end
                    else r_n <= r_step;
                    cc <= cc + 12'd1; st <= W_LCOL;
                end else st <= W_NEXT;
            end
            // ---------------------------------------------------------------- next visit
            W_NEXT: st <= W_VISIT;
            W_FEND: if (oq_room) begin oq_push(1'b0, 1'b1, {32'h0, 4'd5, 28'd0}, 64'd0); st <= W_IDLE; end   // done: when it leaves the queue
            default: st <= W_IDLE;
          endcase
          // ---------------------------------------------------------------- the setup sequencer, beside the TRI item
          // (in the walk before the item it would cost 5 serial cycles a visit). Started by W_VISIT with the item; its
          // registers are read only
          // after it is idle: the references by W_TRIR, the rest by the walk after tri_end (W_SWAIT holds it until then).
          case (sp)
            SP_S1: begin                                                 // triangles, points and lines
                cur_tri <= tid; cur_ok <= 1'b1; c_tris <= c_tris + 1; c_vis <= c_vis + 1;
                dx1 <= SX1 - SX0; dy1 <= SY1 - SY0; dx2 <= SX2 - SX0; dy2 <= SY2 - SY0;
                pxl <= PX18 - 18'(ps >> 1); pyt <= PY18 - 18'(ps >> 1) + 18'(ps);
                if (prim == 2'd2) begin                                  // the square around the point's 18-bit centre (record words 5 / 6)
                    xmin_u <= PX18 - 18'(ps >> 1); xmax_u <= PX18 - 18'(ps >> 1) + 18'(ps) - 18'd1;
                    ymin_u <= PY18 - 18'(ps >> 1); ymax_u <= PY18 - 18'(ps >> 1) + 18'(ps) - 18'd1;
                end else begin
                    xmin_u <= 18'(min3(SX0, SX1, SX2)); xmax_u <= 18'(max3(SX0, SX1, SX2)); ymin_u <= 18'(min3(SY0, SY1, SY2)); ymax_u <= 18'(max3(SY0, SY1, SY2));
                end
                sp <= (prim == 2'd1) ? (tri_skip ? SP_IDLE : SP_R1) : SP_S2;
            end
            SP_S2: begin                                                 // triangles and points
                q1 <= dx1 * dy2; q2 <= dx2 * dy1;
                if (prim == 2'd2) begin A[1] <= 24'sd16; B[1] <= 24'sd0; A[2] <= 24'sd0; B[2] <= -24'sd16; end
                else begin A[1] <= $signed({dy2, 4'd0}); B[1] <= -$signed({dx2, 4'd0}); A[2] <= -$signed({dy1, 4'd0}); B[2] <= $signed({dx1, 4'd0}); end
                xmin_px <= clampx(xmin_u, h_rx0, h_rx1); xmax_px <= clampx(xmax_u, h_rx0, h_rx1);       // the bounding box cut to the rectangle
                ymin_px <= clampx(ymin_u, h_ry0, h_ry1); ymax_px <= clampx(ymax_u, h_ry0, h_ry1);
                tbox_ok <= !(xmax_u < $signed({2'b00, h_rx0, 4'd0})) && !(ymax_u < $signed({2'b00, h_ry0, 4'd0})) &&
                           !(xmin_u >= $signed({2'b00, h_rx1, 4'd0})) && !(ymin_u >= $signed({2'b00, h_ry1, 4'd0}));
                sp <= SP_S3;
            end
            SP_S3: begin
                a2 <= q1 - q2; front <= !a2_neg;
                if (prim == 2'd2) begin A[0] <= 24'sd0; B[0] <= 24'sd0; end
                else begin A[0] <= -A[1] - A[2]; B[0] <= -B[1] - B[2]; end
                qy <= 11'(y0 >> 1); rows_left <= 3'(nl >> 1); qx_max <= 11'(xmax_px >> 1);
                sp <= tri_skip ? SP_W1 : SP_R1;                          // the references only for a TRI item that is sent
            end
            // the reference edge values at pixel (0, 0)
            SP_R1: begin
                cxr <= 18'sd8 - 18'(SX0); cyr <= 18'sd8 - 18'(SY0);
                lp1 <= dx1 * dx1; lp2 <= dy1 * dy1;                      // a line's len2 (W_LS1 computes them again)
                sp <= SP_R2;
            end
            SP_R2: begin
                if (prim == 2'd1) begin p1 <= cxr * dx1; p2 <= cyr * dy1; end          // e1 = (c - a) . (b - a)
                else begin p1 <= cxr * dy2; p2 <= dx2 * cyr; p3 <= dx1 * cyr; p4 <= cxr * dy1; end
                len2 <= lp1 + lp2;
                sp <= SP_R3;
            end
            SP_R3: begin : REF
                reg signed [EW-1:0] e1r, e2r;
                if (prim == 2'd2) begin e1r = EW'(18'sd8 - pxl); e2r = EW'(pyt - 18'sd8); end   // the square: cx - xl, yt - cy
                else if (prim == 2'd1) begin e1r = p1 + p2; e2r = '0; end
                else begin e1r = p1 - p2; e2r = p3 - p4; end
                er[1] <= shr_s(e1r, esh); er[2] <= shr_s(e2r, esh);
                er[0] <= shr_s((prim == 2'd2) ? EW'(ps) : (prim == 2'd1) ? len2 - e1r : a2 - e1r - e2r, esh);
                sp <= (prim == 2'd1) ? SP_IDLE : SP_W1;                  // a line walks on its own (W_LS1..)
            end
            // the band's first quad row (qy = y0 / 2, S3) at xmin_px & ~1: origin, products, sums — once a visit and
            // beside the TRI item; the lanes are formed as a row is entered
            SP_W1: begin
                cxo <= 18'({xmin_px[11:1], 1'b0, 4'd8}) - 18'(SX0);       // 16 * (xmin_px & ~1) + 8 - x0
                cyo <= 18'({qy, 1'b0, 4'd8}) - 18'(SY0);                  // 16 * (2 qy) + 8 - y0
                sp <= SP_W2;
            end
            SP_W2: begin p1 <= cxo * dy2; p2 <= dx2 * cyo; p3 <= dx1 * cyo; p4 <= cxo * dy1; sp <= SP_W3; end
            SP_W3: begin
                if (prim == 2'd2) begin                                  // the square: e1 = cx - xl, e2 = yt - cy, e0 = ps
                    rs[1] <= EW'($signed(18'({xmin_px[11:1], 1'b0, 4'd8})) - pxl); rs[2] <= EW'(pyt - $signed(18'({qy, 1'b0, 4'd8}))); rs[0] <= EW'(ps);
                end else begin rs[1] <= p1 - p2; rs[2] <= p3 - p4; rs[0] <= a2 - (p1 - p2) - (p3 - p4); end
                sp <= SP_IDLE;
            end
            default: ;
          endcase
        end
    end
    // synthesis translate_off
    // the setup sequencer's results are read only once it is idle: a walk state that reads them while it runs
    // would take the previous visit's values
    always @(posedge clk) if (!rst && !abort && sp_busy && !(st == W_TRIW || st == W_TRIE || st == W_TRIR || st == W_SWAIT))
        $fatal(1, "a_render: walk state %s while the setup sequencer runs (%s)", st.name(), sp.name());
    // synthesis translate_on
    // ================================================================ out: the queue's beats as the render stream
    reg         fr_act, e_half;
    wire [2:0]  rq = oq_rp[2:0];
    wire        out_ok = (oq_rp != oq_ww);                  // the head is written
    always @(posedge clk) begin
        if (rs_valid && rs_ready) rs_valid <= 1'b0;
        done <= 1'b0;
        if (rst) begin
            oq_rp <= 4'd0; oq_ww <= 4'd0; rs_valid <= 1'b0; fr_act <= 1'b0; e_half <= 1'b0;
        end else if (abort) begin
            oq_rp <= 4'd0; oq_ww <= 4'd0; rs_valid <= 1'b0; e_half <= 1'b0;
            if (fr_act) begin done <= 1'b1; fr_act <= 1'b0; end
        end else begin
            if (go && !fr_act) fr_act <= 1'b1;
            if (oq_pv) oq_ww <= oq_ww + 4'd1;                    // the push's LUTRAM write lands this edge
            // out: a quad's two beats; any other item's one; FRAME_END ends the frame
            if (out_ok && out_free) begin
                rs_valid <= 1'b1; rs_data <= e_half ? oq_b1[rq] : oq_b0[rq];
                if (oq_k[rq] && !e_half) e_half <= 1'b1;
                else begin
                    e_half <= 1'b0; oq_rp <= oq_rp + 4'd1;
                    if (oq_fe[rq]) begin done <= 1'b1; fr_act <= 1'b0; end
                end
            end
        end
    end
    assign busy = (st != W_IDLE) || (fs != F_IDLE) || (oq_wp != oq_rp);
    // synthesis translate_off
    always @(posedge clk) if (!rst && rt_hdr && v_rdy[kq_h[5:2]]) $fatal(1, "a_render: a header returned into a ready visit slot");
    always @(posedge clk) if (!rst && rt_ent && ln_cnt == 2'd2) $fatal(1, "a_render: a list line returned into a full line FIFO");
    always @(posedge clk) if (!rst && 7'(bq_cnt) + 7'(bres) > 7'd32) $fatal(1, "a_render: TRI bursts landed and reserved beyond the burst FIFO");
    // synthesis translate_on
    wire _unused = &{1'b0, hdr, q1, q2, varmask, lp1, lp2, ent_cur[55:32], ent_cur[62:60]};
endmodule
`default_nettype wire
