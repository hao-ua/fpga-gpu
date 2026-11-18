// clip_near — clipping of one triangle against the clip volume (ES 2.0 §2.13). Sutherland-Hodgman, SEVEN planes in
// order: plane 0 w ≥ 2⁻⁷ (the guard against the division), plane 1 z + w ≥ 0 (near;
// with z0 set it is z ≥ 0: the volume 0 ≤ z ≤ w of a library that keeps depth reversed, model clip(.., z0)),
// plane 2 w − z ≥ 0 (far), then the GUARD BAND: plane 3 gxhi·w − x ≥ 0, plane 4
// x − gxlo·w ≥ 0, plane 5 gyhi·w − y ≥ 0, plane 6 y − gylo·w ≥ 0 — the four constants (gb, FP24, VS_STATE words 9..12) are
// the NDC values whose window coordinate is ±2040 px, inside vtx_setup2's 16-bit saturation, so a vertex far outside the
// view is clipped exactly instead of being moved by the saturation. A vertex = NATTR scalars (x, y, z,
// w, then NVAR varyings). Per plane the signed distances of all polygon vertices are issued ONE PER CYCLE (two fetch
// ports: w and z / x / y of the vertex; the guard planes multiply first, the products feed the subtracter as they land)
// and collected in order; a plane every vertex is inside leaves the polygon as it is (no walk); otherwise the edges are
// walked: an inside start vertex is kept, a crossing edge appends the intersection with every ENABLED attribute lerped
// FROM THE INSIDE vertex: t = d_in / (d_in − d_out), new = v_in + t · (v_out − v_in) (the same point for
// both triangles sharing the edge). The lerp is PIPELINED (done one attribute at a time it costs a clipped triangle
// ~900 cycles of setup): the enabled attributes' differences are issued one per cycle, each is multiplied by t as it
// lands and added to the inside value as that lands (the adder is shared: an addition goes first, the walk waits a
// cycle); a masked-out varying of an intersection is not written at all, it READS as 0. The polygon grows by at most one
// vertex per plane (≤ 10); the original vertices stay
// in verts_in and the ≤ 14 intersections live in a small RAM, the polygon is a list of references. The triangles of the
// fan (0, k, k + 1), up to 8, are emitted one at a time — TWO attributes per cycle into tri_out — tri_valid, then the
// caller's tri_ack asks for the next; done when none is left (also when the polygon vanished). verts_in is sampled at
// start. Model: tools/a_setup_ref.py clip().
//   THE CLIP FORMAT. A vertex far outside the view — a coordinate of 2^18 beside a w of 2^3 — leaves the shader format
// no bits for the part of an edge that is inside: the plane distances, t and the intersections need the difference of
// two such numbers to a fraction of a pixel. So everything between the vertices in and the vertices out is of a format
// of the clipper's own: the shader format's sign and exponent with a mantissa of CM_W bits. The vertices are extended
// exactly on the way in (zeros below the mantissa), every distance, t and every stored intersection has the wide
// mantissa, and a vertex is rounded to the shader format (to nearest even) once, in the output copy. t: the shader
// format's reciprocal unit on the denominator rounded to that format, r0, then one Newton step and the product in the
// clip format: t = d_in · r0 · (2 − den · r0) (model clip_t).
//   LINES (line): the two ends are vertices 0 and 1 (vertex 2 = vertex 0, the caller's copy). The same planes: both
// ends outside one -> done without a result; one outside -> it moves onto the plane (the edge 0 -> 1 alone is walked, so
// end 0 stays end 0); the result is ONE output (0, 1, 0), pass when no end moved. Model: clip_line().
`default_nettype none
module clip_near #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16,
    parameter int NVAR  = 32,
    parameter int CM_W  = 31                     // the clip format's mantissa
)(
    input  wire clk,
    input  wire rst,
    input  wire start,
    input  wire line,                            // with start: a line (vertices 0, 1), not a triangle
    input  wire [3*(4+NVAR)*(EXP_W+MAN_W+1)-1:0] verts_in,
    input  wire [NVAR/4-1:0] vmask,              // the varyings to lerp (the others of an intersection read 0)
    input  wire [4*(EXP_W+MAN_W+1)-1:0] gb,      // {gyhi, gylo, gxhi, gxlo}: the guard band's NDC bounds
    input  wire z0,                              // plane 1 is z >= 0, not z + w >= 0: the clip volume 0 <= z <= w (the
                                                 // depth kept reversed by the libraries, far at z = 0; stable while busy)
    output reg  busy,
    output reg  tri_valid,                       // one cycle: tri_out holds a triangle; held until tri_ack
    input  wire tri_ack,                         // the caller took it: emit the next (or done)
    output wire [3*(4+NVAR)*(EXP_W+MAN_W+1)-1:0] tri_out,
    output reg  pass,                            // with tri_valid: the input triangle passes untouched — tri_out is NOT written,
                                                 // the caller keeps its own copy (the 54-cycle output copy is the
                                                 // largest single cost per triangle, and the common case needs none)
    output reg  done                             // one cycle: no triangle (left)
);
    localparam int W     = 1+EXP_W+MAN_W;
    localparam int WC    = 1+EXP_W+CM_W;         // a number of the clip format
    localparam int XB    = CM_W-MAN_W;           // its mantissa bits below the shader format's
    localparam int NATTR = 4+NVAR;
    localparam int VB    = NATTR*W;
    localparam int BIAS  = (1<<(EXP_W-1))-1;
    localparam int INF_E = (1<<EXP_W)-1;
    localparam int NP    = 10;                   // polygon vertices at most (3 + one per plane)
    localparam int XS    = 14;                   // intersection slots (≤ 2 per plane)
    localparam int NPAIR = NATTR/2;              // attribute pairs of the output copy
    localparam [W-1:0] WEPS = (BIAS-7) << MAN_W; // 2^-7
    localparam [WC-1:0] TWO = WC'(BIAS+1) << CM_W;   // 2.0 in the clip format
    if (NATTR % 2 != 0) $error("clip_near: NATTR must be even");
    if (XB < 2) $error("clip_near: CM_W must exceed MAN_W by 2 at least");
    // the shader format -> the clip format, exactly; and back: to nearest even, a mantissa that rounds up to 2 raises
    // the exponent, past the largest one the result is Inf
    function automatic [WC-1:0] cw(input [W-1:0] v); cw = {v, {XB{1'b0}}}; endfunction
    function automatic [W-1:0] c24(input [WC-1:0] v);
        reg [MAN_W:0] q; reg [EXP_W:0] e;
        q = {1'b0, v[CM_W-1:XB]} + (MAN_W+1)'(v[XB-1] & ((|v[XB-2:0]) | v[XB]));
        e = {1'b0, v[WC-2:CM_W]} + (EXP_W+1)'(q[MAN_W]);
        if (v[WC-2:CM_W] == '0)      c24 = {v[WC-1], {(W-1){1'b0}}};
        else if (&v[WC-2:CM_W])      c24 = {v[WC-1], {EXP_W{1'b1}}, |v[CM_W-1:0], {(MAN_W-1){1'b0}}};
        else if (e >= (EXP_W+1)'(INF_E)) c24 = {v[WC-1], {EXP_W{1'b1}}, {MAN_W{1'b0}}};
        else                         c24 = {v[WC-1], e[EXP_W-1:0], q[MAN_W-1:0]};
    endfunction

    // ---- the polygon: references {is_intersection, index[3:0]}; the intersections' attributes in a small RAM
    reg [4:0] pin [0:NP-1], pout [0:NP-1]; reg [3:0] n_in, n_out, xs;
    reg ln;                                      // a line is being clipped
    reg [WC-1:0] dv [0:NP-1];                    // the plane distance of every polygon vertex
    (* ram_style="distributed" *) reg [WC-1:0] xmem [0:XS*NATTR-1];
    function automatic aen(input [5:0] ak);     // attribute enabled: x y z w always, a varying when its vec4 is masked in
        aen = (ak < 6'd4) ? 1'b1 : vmask[(ak - 6'd4) >> 2];
    endfunction
    // the input vertices, captured at start (the caller reuses its vertex registers for the triangles it gets back), as
    // WORDS: a word-indexed array reads as a 108 : 1 mux per bit; the part-select of one flat vector with a computed
    // offset synthesises to ~2000 LUTs per port
    reg  [W-1:0] vw [0:3*NATTR-1];
    // two fetch ports: attribute ak of reference ra, attribute akb of reference rb (registered one cycle before use)
    reg  [4:0] ra, rb; reg [5:0] ak, akb;
    wire [6:0] oa_i = 7'(ra[1:0]*NATTR) + 7'(ak), ob_i = 7'(rb[1:0]*NATTR) + 7'(akb);
    wire [W-1:0] oa = vw[oa_i], ob = vw[ob_i];
    wire [WC-1:0] xa = aen(ak) ? xmem[ra[3:0]*NATTR + ak] : '0,   xb = aen(akb) ? xmem[rb[3:0]*NATTR + akb] : '0;   // a masked varying of an intersection reads 0 (never written)
    wire [WC-1:0] atta = ra[4] ? xa : cw(oa), attb = rb[4] ? xb : cw(ob);   // (an input vertex: extended)
    // the output triangle as words too (even / odd attribute of the pair written per cycle): a 54-way write decode each
    // instead of the flat vector's computed-offset write (~3800 LUTs)
    reg  [W-1:0] twe [0:3*NPAIR-1], two [0:3*NPAIR-1];             // written at word pair ov * NPAIR + pr (O_PAIR)
    genvar gp;
    generate for (gp = 0; gp < 3*NPAIR; gp = gp + 1) begin : TO
        assign tri_out[(2*gp)*W +: W] = twe[gp];
        assign tri_out[(2*gp+1)*W +: W] = two[gp];
    end endgenerate
    // the output pair goes through a register — the two fetched words and a one-hot of the pair they go to — before the
    // write (timing at 120 MHz: the port's address arithmetic -> the vertex / intersection RAM read -> the 54-way write
    // decode is too long for one cycle), and is rounded to the shader format on its way from that register into the
    // triangle; the last pair lands with tri_valid (O_VALID is one cycle)
    reg  ow_v; reg [3*NPAIR-1:0] ow_oh; reg [WC-1:0] ow_a, ow_b;
    wire [W-1:0] gxlo = gb[0*W +: W], gxhi = gb[1*W +: W], gylo = gb[2*W +: W], gyhi = gb[3*W +: W];

    // ---- shared float units: the adder and the multiplier of the clip format, the reciprocal of the shader format
    reg as_in, as_op; reg [WC-1:0] as_a, as_b; wire as_v; wire [WC-1:0] as_r;
    float_addsub_w #(EXP_W,CM_W) U_AS(.clk,.rst,.in_valid(as_in),.a(as_a),.b(as_b),.op(as_op),.out_valid(as_v),.result(as_r));
    reg mu_in; reg [WC-1:0] mu_a, mu_b; wire mu_v; wire [WC-1:0] mu_r;
    float_mul_w #(EXP_W,CM_W) U_MU(.clk,.rst,.in_valid(mu_in),.a(mu_a),.b(mu_b),.out_valid(mu_v),.result(mu_r));
    reg rc_in; reg [W-1:0] rc_x; wire rc_v; wire [W-1:0] rc_r;
    float_sfu #(.EXP_W(EXP_W),.MAN_W(MAN_W),.K(EXP_W==7?12:8),.KLOG(EXP_W==7?11:8),
        .RCP_MEM(EXP_W==7?"tools/gen/fp24_rcp_seed.mem":"tools/gen/fp16_rcp_seed.mem"),
        .RSQRT_MEM(EXP_W==7?"tools/gen/fp24_rsqrt_seed.mem":"tools/gen/fp16_rsqrt_seed.mem"),
        .LOG2_MEM(EXP_W==7?"tools/gen/fp24_log2_lut.mem":"tools/gen/fp16_log2_lut.mem"),
        .EXP2_MEM(EXP_W==7?"tools/gen/fp24_exp2_lut.mem":"tools/gen/fp16_exp2_lut.mem"))
      U_RC(.clk,.rst,.in_valid(rc_in),.op(3'd0),.x(rc_x),.out_valid(rc_v),.result(rc_r));

    typedef enum logic [5:0] {S_IDLE,
        X_ISS, X_COL, X_JUDGE,                            // the prescan: all seven planes' distances of the input triangle
        D_START, D_ISS, D_COL, D_JUDGE,                   // the distances of the polygon's vertices (pipelined), the verdict
        E_EDGE,                                           // the edge walk
        T_DEN, T_DENW, T_RCPW, T_N1W, T_N2W, T_N3W, T_MULW,   // t of a crossing edge
        L_START, L_RUN, L_DRAIN,                          // the pipelined lerp of the enabled attributes
        P_NEXT, P_SKIP,                                   // the plane is done (walked / untouched)
        O_START, O_PAIR, O_VALID, O_HOLD,                 // the output triangles
        S_DONE} st_t; st_t cs;
    reg [2:0] plane; reg [3:0] vi, ei, ok, ci;           // vertex / edge / output triangle / collected index
    reg [1:0] ov; reg [4:0] pr;                           // output vertex 0..2, attribute pair
    reg       fvv;                                        // D_ISS: the fetched vertex vi's operands are on the ports
    reg [4:0] rin, rout; reg [WC-1:0] din, dout, treg;
    reg [WC-1:0] den, r0;                                 // t's denominator d_in - d_out and its first reciprocal
    // the pipelined lerp: the walk over the enabled attributes (l_ak on the fetch ports), the queues that carry the inside
    // value and the attribute from the subtraction's issue to the addition's (inq) and from the addition's issue to its
    // result (outq), and the kind of every adder result in issue order (kq: 0 = a difference -> the multiplier, 1 = final).
    // 16 entries each: an inq entry lives the adder's latency + the multiplier's + 2 cycles (13), and the walk issues one
    // a cycle until the first product lands.
    reg        l_iss; reg [5:0] l_ak, l_n_iss, l_n_fin;
    reg [WC-1:0] inq [0:15]; reg [5:0] inq_ak [0:15]; reg [3:0] inq_wp, inq_rp;
    reg [5:0]  outq [0:15]; reg [3:0] outq_wp, outq_rp;
    reg        kq [0:15]; reg [3:0] kq_wp, kq_rp;
    function automatic [5:0] nxt_ak(input [5:0] a);    // the next enabled attribute after a (NATTR when none is left)
        nxt_ak = 6'(NATTR);
        for (int k = NATTR-1; k > 0; k--) if (6'(k) > a && aen(6'(k))) nxt_ak = 6'(k);
    endfunction
    // the output copy walks only the ENABLED attribute pairs: a masked-out varying's words of tri_out keep
    // what they held, which the caller never reads (a_setup's difference stream and record take fsmask's
    // varyings only; the clipper itself lerps and fetches enabled attributes only). With two varyings 6 pairs of 18.
    function automatic [4:0] nxt_pr(input [4:0] p);    // the next enabled pair after p (NPAIR when none is left)
        nxt_pr = 5'(NPAIR);
        for (int k = NPAIR-1; k > 0; k--) if (5'(k) > p && aen(6'(2*k))) nxt_pr = 5'(k);
    endfunction
    reg [WC-1:0] xyq [0:15]; reg xyq_hi [0:15]; reg [3:0] xy_wp, xy_rp;   // the x / y operands of the guard planes, in issue order (the mul's latency); hi: a g_hi plane
    reg       any_out, all_out;
    // the prescan (the input triangle against all seven planes in one pipelined batch, 21 issues): per plane the three
    // distances and whether any / every vertex is outside; while the polygon is still the input triangle (orig) a plane
    // without an outside vertex is skipped at once and the first clipping plane uses the prescanned distances. (A line's
    // vertex 2 is its vertex 0: any / every over the three is any / every over the two ends.)
    reg [WC-1:0] xdv [0:6][0:2]; reg [6:0] x_any, x_all; reg orig; reg [4:0] x_i, x_c;
    wire [2:0] xi_pl = x_i / 3, xc_pl = x_c / 3; wire [1:0] xi_v = 2'(x_i % 3), xc_v = 2'(x_c % 3);   // issue / collect: plane, vertex
    wire [W-1:0] xg = (xi_pl == 3'd3) ? gxhi : (xi_pl == 3'd4) ? gxlo : (xi_pl == 3'd5) ? gyhi : gylo;
    wire [3:0] nxt = (ei == n_in - 4'd1) ? 4'd0 : ei + 4'd1;
    wire ia = ~dv[ei][WC-1], ib = ~dv[nxt][WC-1];
    wire gplane = plane[2] | (plane == 3'd3);            // planes 3..6: a product first
    wire [W-1:0] gconst = (plane == 3'd3) ? gxhi : (plane == 3'd4) ? gxlo : (plane == 3'd5) ? gyhi : gylo;
    wire [5:0] akb_of_plane = (plane == 3'd1 || plane == 3'd2) ? 6'd2 : (plane == 3'd3 || plane == 3'd4) ? 6'd0 : 6'd1;
    function automatic [4:0] tri_ref(input [1:0] v, input [3:0] k);   // the fan's triangle k: vertices 0, k, k + 1 (a line: 0, 1, 0)
        tri_ref = (v == 2'd0) ? pin[0] : (v == 2'd1) ? pin[k] : ln ? pin[0] : pin[k + 4'd1];
    endfunction
    // TRIVIAL ACCEPT: a triangle certainly inside all seven planes passes at once, without the 21-issue prescan
    // (about a third of a small triangle's setup). From the exponents alone, per vertex: w >= 2^-7 (plane 0); |z| < w
    // (planes 1, 2: e(z) < e(w), so |z| < 2^(e(z)-B+1) <= 2^(e(w)-B) <= w); |x| < 2^kx w (planes 3, 4) with
    // 2^kx <= min(gxhi, -gxlo), i.e. kx = min(e(gxhi), e(gxlo)) - B for gxhi > 0 > gxlo: e(x) < e(w) + kx; y likewise. Every
    // inequality is strict, so the prescan would find no vertex outside either: the result is the same, only sooner.
    function automatic [EXP_W-1:0] ex(input [W-1:0] v); ex = v[MAN_W +: EXP_W]; endfunction
    wire [EXP_W:0] kxe = (ex(gxhi) < ex(gxlo)) ? {1'b0, ex(gxhi)} : {1'b0, ex(gxlo)};   // min exponent (+ B: compared with + B below)
    wire [EXP_W:0] kye = (ex(gyhi) < ex(gylo)) ? {1'b0, ex(gyhi)} : {1'b0, ex(gylo)};
    wire gx_ok = !gxhi[W-1] && ex(gxhi) != 0 && gxlo[W-1] && ex(gxlo) != 0;
    wire gy_ok = !gyhi[W-1] && ex(gyhi) != 0 && gylo[W-1] && ex(gylo) != 0;
    wire [2:0] v_in;
    genvar gv;
    generate for (gv = 0; gv < 3; gv = gv + 1) begin : TA
        wire [W-1:0] vx = verts_in[(gv*NATTR + 0)*W +: W], vy = verts_in[(gv*NATTR + 1)*W +: W],
                     vz = verts_in[(gv*NATTR + 2)*W +: W], vw4 = verts_in[(gv*NATTR + 3)*W +: W];
        wire [EXP_W+1:0] ew = {2'b00, ex(vw4)};
        assign v_in[gv] = !vw4[W-1] && ex(vw4) >= EXP_W'(BIAS - 7) &&
                          (ex(vz) == 0 || (ex(vz) < ex(vw4) && !(z0 && vz[W-1]))) &&   // z0: a negative z is outside plane 1 (z + 0 keeps its sign; a zero adds to +0)
                          (ex(vx) == 0 || ({2'b00, ex(vx)} + (EXP_W+2)'(BIAS) < ew + {1'b0, kxe})) &&
                          (ex(vy) == 0 || ({2'b00, ex(vy)} + (EXP_W+2)'(BIAS) < ew + {1'b0, kye}));
    end endgenerate
    wire t_accept = gx_ok && gy_ok && (&v_in);

    always @(posedge clk) begin
        if (rst) begin cs<=S_IDLE; busy<=0; tri_valid<=0; done<=0; as_in<=0; mu_in<=0; rc_in<=0; pass<=0; end
        else begin
            tri_valid<=0; done<=0; as_in<=0; mu_in<=0; rc_in<=0;
            // the distance phase's collection (results may land while later vertices are still being issued) and, for the
            // guard planes, the subtraction as each product lands
            if ((cs == D_ISS || cs == D_COL) && mu_v) begin
                as_in<=1'b1; as_op<=1'b1; xy_rp<=xy_rp+4'd1;
                if (plane[0]) begin as_a<=mu_r; as_b<=xyq[xy_rp]; end       // planes 3, 5: g_hi w - x / y
                else          begin as_a<=xyq[xy_rp]; as_b<=mu_r; end       // planes 4, 6: x / y - g_lo w
            end
            if ((cs == D_ISS || cs == D_COL) && as_v) begin
                dv[ci]<=as_r; any_out<=any_out | as_r[WC-1]; all_out<=all_out & as_r[WC-1]; ci<=ci+4'd1;
            end
            // ---- the lerp pipeline (L_RUN / L_DRAIN): the adder's results in issue order, their kind from kq; a product lands
            //      -> its addition is issued at once (ahead of the walk, whose ports hold their attribute meanwhile)
            if ((cs == L_RUN || cs == L_DRAIN) && as_v) begin
                kq_rp<=kq_rp+4'd1;
                if (!kq[kq_rp]) begin mu_in<=1'b1; mu_a<=treg; mu_b<=as_r; end                      // t (out - in)
                else begin xmem[xs*NATTR + outq[outq_rp]]<=as_r; outq_rp<=outq_rp+4'd1; l_n_fin<=l_n_fin+6'd1; end   // in + t (out - in)
            end
            if (cs == L_RUN || cs == L_DRAIN) begin
                if (mu_v) begin
                    as_in<=1'b1; as_op<=1'b0; as_a<=inq[inq_rp]; as_b<=mu_r; inq_rp<=inq_rp+4'd1;
                    outq[outq_wp]<=inq_ak[inq_rp]; outq_wp<=outq_wp+4'd1; kq[kq_wp]<=1'b1; kq_wp<=kq_wp+4'd1;
                end else if (cs == L_RUN && l_iss) begin : LW                                        // the walk: out - in of l_ak
                    reg [5:0] na; na = nxt_ak(l_ak);
                    as_in<=1'b1; as_op<=1'b1; as_a<=attb; as_b<=atta;
                    inq[inq_wp]<=atta; inq_ak[inq_wp]<=l_ak; inq_wp<=inq_wp+4'd1; kq[kq_wp]<=1'b0; kq_wp<=kq_wp+4'd1;
                    l_n_iss<=l_n_iss+6'd1;
                    if (na >= 6'(NATTR)) l_iss<=1'b0;
                    else begin l_ak<=na; ak<=na; akb<=na; end
                end
            end
            // the prescan's products (planes 3..6, in issue order: the x / y operand waits in xyq) and its collection
            if ((cs == X_ISS || cs == X_COL) && mu_v) begin
                as_in<=1'b1; as_op<=1'b1; xy_rp<=xy_rp+4'd1;
                if (xyq_hi[xy_rp]) begin as_a<=mu_r; as_b<=xyq[xy_rp]; end  // planes 3, 5: g_hi w - x / y
                else               begin as_a<=xyq[xy_rp]; as_b<=mu_r; end  // planes 4, 6: x / y - g_lo w
            end
            if ((cs == X_ISS || cs == X_COL) && as_v) begin
                xdv[xc_pl][xc_v]<=as_r; x_any[xc_pl]<=x_any[xc_pl] | as_r[WC-1]; x_all[xc_pl]<=x_all[xc_pl] & as_r[WC-1]; x_c<=x_c+5'd1;
            end
            case (cs)
                S_IDLE: if (start) begin
                    for (int i = 0; i < 3*NATTR; i++) vw[i]<=verts_in[i*W +: W];
                    busy<=1; pin[0]<=5'd0; pin[1]<=5'd1; pin[2]<=5'd2; n_in<=line ? 4'd2 : 4'd3; ln<=line; n_out<=4'd0; xs<=4'd0; plane<=3'd0; orig<=1'b1;
                    x_i<=5'd0; x_c<=5'd0; x_any<=7'd0; x_all<=7'h7F; xy_wp<=4'd0; xy_rp<=4'd0;
                    ra<=5'd0; rb<=5'd0; ak<=6'd3; akb<=6'd3;                                 // (vertex 0 of plane 0: w, w)
                    if (t_accept) begin ok<=4'd1; pass<=1'b1; cs<=O_VALID; end              // trivially inside: as X_JUDGE's pass
                    else cs<=X_ISS;
                end
                // ---- the prescan: issue (plane, vertex) = x_i one per cycle, the ports set a cycle ahead for x_i + 1
                X_ISS: begin : XI
                    reg [4:0] n1; reg [2:0] npl; reg [1:0] nv;
                    n1 = x_i + 5'd1; npl = n1 / 3; nv = 2'(n1 % 3);
                    case (xi_pl)
                        3'd0:    begin as_in<=1'b1; as_a<=atta; as_b<=cw(WEPS); as_op<=1'b1; end  // w - eps
                        3'd1:    begin as_in<=1'b1; as_a<=attb; as_b<=z0 ? '0 : atta; as_op<=1'b0; end   // z + w (z0: z + 0)
                        3'd2:    begin as_in<=1'b1; as_a<=atta; as_b<=attb; as_op<=1'b1; end      // w - z
                        default: begin mu_in<=1'b1; mu_a<=cw(xg); mu_b<=atta; xyq[xy_wp]<=attb; xyq_hi[xy_wp]<=xi_pl[0]; xy_wp<=xy_wp+4'd1; end
                    endcase
                    if (x_i == 5'd20) cs<=X_COL;
                    else begin
                        x_i<=n1; ra<=5'(nv); rb<=5'(nv); ak<=6'd3;
                        akb<=(npl == 3'd1 || npl == 3'd2) ? 6'd2 : (npl == 3'd3 || npl == 3'd4) ? 6'd0 : (npl >= 3'd5) ? 6'd1 : 6'd3;
                    end
                end
                X_COL: if (x_c == 5'd21) cs<=X_JUDGE;
                X_JUDGE: begin                                           // the input triangle's verdict per plane
                    if (|x_all) cs<=S_DONE;                              // entirely outside one plane: nothing
                    else if (x_any == 7'd0) begin ok<=4'd1; pass<=1'b1; cs<=O_VALID; end   // inside everything: the triangle as it is, no copy
                    else if (x_any[0]) begin                             // plane 0 clips: its prescanned distances
                        for (int i = 0; i < 3; i++) dv[i]<=xdv[0][i];
                        any_out<=1'b1; all_out<=1'b0; ei<=4'd0; n_out<=4'd0; cs<=E_EDGE;
                    end else cs<=P_SKIP;                                 // plane 0 untouched: P_SKIP advances to the first clipping plane
                end
                // ---- the distances of vertices 0 .. n_in - 1 to the plane, one issue per cycle: the ports are set for
                //      vertex vi one cycle ahead (D_START: vertex 0), D_ISS issues vi and addresses vi + 1
                D_START: begin
                    ra<=pin[0]; rb<=pin[0]; ak<=6'd3; akb<=akb_of_plane; vi<=4'd0; fvv<=1'b1; ci<=4'd0;
                    xy_wp<=4'd0; xy_rp<=4'd0; any_out<=1'b0; all_out<=1'b1; cs<=D_ISS;
                end
                D_ISS: begin
                    if (fvv) begin
                        case (plane)
                            3'd0:    begin as_in<=1'b1; as_a<=atta; as_b<=cw(WEPS); as_op<=1'b1; end  // w - eps
                            3'd1:    begin as_in<=1'b1; as_a<=attb; as_b<=z0 ? '0 : atta; as_op<=1'b0; end   // z + w (z0: z + 0)
                            3'd2:    begin as_in<=1'b1; as_a<=atta; as_b<=attb; as_op<=1'b1; end      // w - z
                            default: begin mu_in<=1'b1; mu_a<=cw(gconst); mu_b<=atta; xyq[xy_wp]<=attb; xyq_hi[xy_wp]<=plane[0]; xy_wp<=xy_wp+4'd1; end   // g w, then - x / y
                        endcase
                    end
                    if (vi == n_in - 4'd1) begin fvv<=1'b0; cs<=D_COL; end
                    else begin vi<=vi+4'd1; ra<=pin[vi+4'd1]; rb<=pin[vi+4'd1]; end
                end
                D_COL: if (ci == n_in) cs<=D_JUDGE;                      // every result collected (above, in issue order)
                D_JUDGE: begin
                    if (all_out) cs<=S_DONE;                             // nothing left
                    else if (!any_out) cs<=P_SKIP;                       // untouched: the polygon stays
                    else begin ei<=4'd0; n_out<=4'd0; cs<=E_EDGE; end
                end
                // ---- the edge walk: ei -> nxt (a line: the edge 0 -> 1 alone crosses; at end 1 only that end is kept)
                E_EDGE: begin
                    if (ia) begin pout[n_out]<=pin[ei]; n_out<=n_out+4'd1; end
                    if (ia != ib && !(ln && ei != 4'd0)) begin                        // a crossing: lerp from the inside vertex
                        rin<=ia ? pin[ei] : pin[nxt]; rout<=ia ? pin[nxt] : pin[ei];
                        din<=ia ? dv[ei] : dv[nxt];  dout<=ia ? dv[nxt] : dv[ei];
                        cs<=T_DEN;
                    end else if (ei == n_in - 4'd1) cs<=P_NEXT;
                    else ei<=ei+4'd1;
                end
                // t = d_in / (d_in - d_out): den, r0 = rcp(den rounded to the shader format), then in the clip format
                // den r0, 2 - den r0, r0 (2 - den r0), and d_in times that
                T_DEN:  begin as_a<=din; as_b<=dout; as_op<=1'b1; as_in<=1'b1; cs<=T_DENW; end
                T_DENW: if (as_v) begin den<=as_r; rc_x<=c24(as_r); rc_in<=1'b1; cs<=T_RCPW; end
                T_RCPW: if (rc_v) begin r0<=cw(rc_r); mu_a<=den; mu_b<=cw(rc_r); mu_in<=1'b1; cs<=T_N1W; end
                T_N1W:  if (mu_v) begin as_a<=TWO; as_b<=mu_r; as_op<=1'b1; as_in<=1'b1; cs<=T_N2W; end
                T_N2W:  if (as_v) begin mu_a<=r0; mu_b<=as_r; mu_in<=1'b1; cs<=T_N3W; end
                T_N3W:  if (mu_v) begin mu_a<=din; mu_b<=mu_r; mu_in<=1'b1; cs<=T_MULW; end
                T_MULW: if (mu_v) begin treg<=mu_r; cs<=L_START; end
                // the lerp xmem[xs][k] = in + t (out - in) of every enabled attribute k, pipelined (the block above the case)
                L_START: begin                                            // attribute 0 (x) on the ports, the queues empty
                    ak<=6'd0; akb<=6'd0; ra<=rin; rb<=rout; l_ak<=6'd0; l_iss<=1'b1; l_n_iss<=6'd0; l_n_fin<=6'd0;
                    inq_wp<=4'd0; inq_rp<=4'd0; outq_wp<=4'd0; outq_rp<=4'd0; kq_wp<=4'd0; kq_rp<=4'd0; cs<=L_RUN;
                end
                L_RUN:   if (!l_iss) cs<=L_DRAIN;                        // the last difference is issued
                L_DRAIN: if (l_n_fin == l_n_iss) begin                   // every result is in the intersection's slot
                    pout[n_out]<={1'b1, xs}; n_out<=n_out+4'd1; xs<=xs+4'd1;
                    if (ei == n_in - 4'd1) cs<=P_NEXT; else begin ei<=ei+4'd1; cs<=E_EDGE; end
                end
                // ---- the plane is done: the output polygon becomes the input of the next plane (P_NEXT) or stays (P_SKIP);
                //      after the last plane the fan goes out. While the polygon is still the input triangle (orig) the next
                //      plane's verdict comes from the prescan: skipped, or walked with the prescanned distances
                P_NEXT: begin
                    for (int i = 0; i < NP; i++) pin[i]<=pout[i];
                    n_in<=n_out; orig<=1'b0;
                    if (n_out < (ln ? 4'd2 : 4'd3)) cs<=S_DONE;
                    else if (plane == 3'd6) begin ok<=4'd1; cs<=O_START; end
                    else begin plane<=plane+3'd1; cs<=D_START; end
                end
                P_SKIP: begin : PS
                    reg [2:0] pn; pn = plane + 3'd1;
                    if (plane == 3'd6) begin ok<=4'd1; cs<=O_START; end
                    else if (!orig) begin plane<=pn; cs<=D_START; end
                    else if (!x_any[pn]) begin plane<=pn; cs<=P_SKIP; end   // untouched: on to the plane after
                    else begin                                                // the first clipping plane: its prescanned distances
                        plane<=pn; for (int i = 0; i < 3; i++) dv[i]<=xdv[pn][i];
                        any_out<=1'b1; all_out<=1'b0; ei<=4'd0; n_out<=4'd0; cs<=E_EDGE;
                    end
                end
                // ---- output: triangle ok of the fan (0, ok, ok + 1), two attributes per cycle (the ports set a cycle ahead)
                O_START: begin ov<=2'd0; pr<=5'd0; ra<=tri_ref(2'd0, ok); rb<=tri_ref(2'd0, ok); ak<=6'd0; akb<=6'd1; pass<=1'b0; cs<=O_PAIR; end
                O_PAIR: begin : OP                                  // (the pair itself: the ow_* block below)
                    reg [4:0] np; np = nxt_pr(pr);
                    if (np >= 5'(NPAIR)) begin                      // the vertex's last enabled pair
                        pr<=5'd0; ak<=6'd0; akb<=6'd1;
                        if (ov == 2'd2) cs<=O_VALID;
                        else begin ov<=ov+2'd1; ra<=tri_ref(ov+2'd1, ok); rb<=tri_ref(ov+2'd1, ok); end
                    end else begin pr<=np; ak<=6'(2*np); akb<=6'(2*np+1); end
                end
                O_VALID: begin tri_valid<=1'b1; cs<=O_HOLD; end
                O_HOLD:  if (tri_ack) begin
                    if (ok + 4'd2 >= n_in) cs<=S_DONE;                                  // (0, k, k+1) was the last
                    else begin ok<=ok+4'd1; cs<=O_START; end
                end
                S_DONE:  begin busy<=0; done<=1'b1; cs<=S_IDLE; end
                default: cs<=S_DONE;
            endcase
        end
    end
    // the output pair's register and write (no reset: a stray write after a reset lands in words no triangle is read from)
    always @(posedge clk) begin
        ow_v <= (cs == O_PAIR); ow_oh <= (3*NPAIR)'(1) << (7'(ov)*7'(NPAIR) + 7'(pr)); ow_a <= atta; ow_b <= attb;
        if (ow_v) for (int g = 0; g < 3*NPAIR; g++) if (ow_oh[g]) begin twe[g] <= c24(ow_a); two[g] <= c24(ow_b); end
    end
    // synthesis translate_off
    // the lerp's queues: a write never lands on an entry still waiting, a read never finds its queue empty
    always @(posedge clk) if (!rst && (cs == L_RUN || cs == L_DRAIN)) begin
        if (4'(inq_wp - inq_rp) == 4'd15 || 4'(outq_wp - outq_rp) == 4'd15 || 4'(kq_wp - kq_rp) == 4'd15)
            $fatal(1, "clip_near: a lerp queue is full (inq %0d, outq %0d, kq %0d)", 4'(inq_wp - inq_rp), 4'(outq_wp - outq_rp), 4'(kq_wp - kq_rp));
        if (as_v && kq_wp == kq_rp) $fatal(1, "clip_near: an adder result that no issue owns");
        if (mu_v && inq_wp == inq_rp) $fatal(1, "clip_near: a product that no difference owns");
    end
    // synthesis translate_on
endmodule
`default_nettype wire
