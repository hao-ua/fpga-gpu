// tmu24 — pipelined ES 2.0 texture unit. One request = a 2x2 quad; returns 4 RGBA
// texels as floats of the shader format (EXP_W / MAN_W: FP24 on board B, FP16 on the single-board path).
// Bit-exact model: tools/tmu_ref.py (it also checks every result against the exact ES 2.0 value: <= 0.5 ULP).
//   * 2D (s, t) and cube maps (x, y, z; major axis per ES 2.0 table 3.11, faces +X -X +Y -Y +Z -Z stored one after
//     the other, each a full mip chain); LOD = log2(rho) (+ bias = coord.w, req_mode 1) or explicit (coord.w, mode
//     2); rho = max(|du/dx|, |du/dy|, |dv/dx|, |dv/dy|) from the quad; cube differences in pixel 0's face.
//   * magnification c rule, NEAREST / LINEAR within a level, *_MIPMAP_NEAREST d = ceil(lambda + 1/2) - 1,
//     *_MIPMAP_LINEAR blending two levels (trilinear); REPEAT / CLAMP_TO_EDGE / MIRRORED_REPEAT for any size and any
//     sign of the coordinate; RGBA8, RGBA4444, RGBA5551, RGB565 with components c / (2^n - 1) exactly; filtered sums
//     exact integers, converted once (no 8-bit truncation); incomplete texture (descriptor flag) -> (0, 0, 0, 1).
//   * beyond ES 2.0, 3D textures (OES_texture_3D, s, t, r = x, y, z; desc_vol, depth 1..255, w, h <= 256, the depth halves
//     with the level, slices one after the other in each level): rho also takes |dw/dx|, |dw/dy|; LINEAR = two 4-tap jobs
//     per level, one per slice, in the accumulation trilinear uses, slice k0 taking (lw (256 - c) + 128) >> 8 of the level
//     weight lw and k1 the rest (exact with one level; 3D LINEAR_MIPMAP_LINEAR rounds the product to 1/256, so
//     the filter and the conversion keep their widths). A 3D LINEAR quad = 8 jobs, with two levels 16.
//   Pipeline (several quads in flight; in-order results):
//     request queue (16) -> F1 descriptor + per-{bank, sampler} level table (offsets of the 12 levels of a texture
//     <= 2048, computed once per descriptor change) -> F2 coordinates (2D: fixed point Q.24; cube: face + ops into ONE
//     pipelined reciprocal (cube_rcp), 4 + projections per quad, results stored per wait-FIFO slot) -> wait FIFO -> F3 LOD
//     (4 differences, log2) -> F4 per-pixel lambda + level select, one JOB per pixel and level -> T1-T4 taps
//     (wrap, indices, weights, word addresses) -> A burst buffer (16 entries in block RAM: reuse, else allocate +
//     one cache lookup per cycle) -> job FIFO -> FL filter (burst-buffer read, 4 taps x 4 channels, integer
//     weighted sums, two levels accumulate) -> CV float conversion (P / ((2^n - 1) 2^24), RNE) -> output.
//   Throughput (cache hits): one job per cycle -> a bilinear quad every ~4 cycles, a trilinear one every ~8.
//   Sample mask (req_mask): F4 emits jobs for the mask's pixels only; the
//   others return zeros (the output buffer is cleared with each result). F2 / F3 still take all the quad's coordinates,
//   so the level of detail — and with it every sampled texel — is what it is with all four sampled. A quad's last job
//   leaves the job FIFO at least 4 cycles after the previous one of the same req_cls (the consumer's result writer).
// Descriptor: desc_* is read combinationally for desc_idx (the request F1 takes); filt = {mag, mip[1:0], min}
// (mip 0 none, 1 nearest, 2 linear); wrap 0 REPEAT, 1 CLAMP_TO_EDGE, 2 MIRRORED_REPEAT; fmt 0 RGBA8, 1 RGBA4444,
// 2 RGBA5551, 3 RGB565; base and offsets in 32-bit words; 16-bit texels two per word (low half first).
`default_nettype none
module tmu24 #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16,
    parameter bit F2_DUAL = 1'b1,             // F2 converts two pixels per cycle (≈ +0.6 k LUT); 0 = one
    parameter bit VOL3D   = 1'b0,             // 3D textures (OES_texture_3D) in hardware: 0 = the volume path is constant-folded away
                                              // (desc_vol / desc_depth read as 0 / 1, every sampler is 2D or cube; −465 LUT / −2 DSP,
                                              // a feature no game of the era uses); the host's g2r_has_3d probe then fails its volume
                                              // CREATE_IMAGE and takes the program paths
    parameter int TC_SETS = 512,              // the texture cache: sets of 4 x 32-byte lines (512 = 64 KB;
    parameter bit TC_HASH = 1'b0,             // 256 = 32 KB), XOR-hashed set index (measured worse: off), next-burst
    parameter bit TC_PF   = 1'b1              // prefetch with merging onto in-flight fills
)(
    input  wire        clk, rst,
    input  wire        cache_inval,          // pulse: texture data changed (render-to-texture / upload) — when idle
    input  wire        req_valid,
    output wire        req_ready,
    input  wire [4:0]  req_sampler,         // descriptor index (board B: {draw-state bank, sampler[3:0]})
    input  wire [1:0]  req_mode,             // 0 implicit LOD, 1 bias = .w, 2 explicit LOD = .w
    input  wire [8*(1+EXP_W+MAN_W)-1:0] req_uv,   // pixel p: x at [p*2W +: W], y at [p*2W+W +: W]
    input  wire [8*(1+EXP_W+MAN_W)-1:0] req_zw,   // pixel p: z at [p*2W +: W], w at [p*2W+W +: W]
    // the pixels of the quad to SAMPLE: a pixel outside the mask gets no tap
    // job and returns zeros; the level of detail still comes from all the quad's coordinates, so the sampled pixels'
    // texels do not depend on the mask. 0 reads as pixel 0 alone. All ones = every pixel.
    input  wire [3:0]  req_mask = 4'hF,
    // the consumer's result writer for this quad (shader_core_fs2 TEXW2: two writers, a result takes 4 cycles of its
    // writer): results of the same class leave at least 4 cycles apart, results of different classes as they come
    input  wire        req_cls = 1'b0,
    output wire [4:0]  desc_idx,
    input  wire [28:0] desc_base,
    input  wire [11:0] desc_w, desc_h,
    input  wire [1:0]  desc_fmt,
    input  wire [3:0]  desc_levels,
    input  wire [3:0]  desc_filt,
    input  wire [1:0]  desc_wrap_s, desc_wrap_t,
    input  wire        desc_cube, desc_incomplete,
    input  wire        desc_vol,                 // 3D texture
    input  wire [7:0]  desc_depth,               // its depth; 0 reads as 1 (an unused sampler), 1 unless desc_vol
    input  wire [1:0]  desc_wrap_r,
    output reg         res_valid,
    output reg  [16*(1+EXP_W+MAN_W)-1:0] res_texels, // pixel p, component c (R G B A) at [(p*4+c)*W +: W]
    output wire        tmu_en,
    output wire [28:0] tmu_addr,
    input  wire        tmu_rdy,
    input  wire        tmu_rd_valid,
    input  wire [255:0] tmu_rd_data,
    // cumulative counters for the STATUS words: tap jobs, cycles the A stage's head
    // job could not go on (bursts missing / the job FIFO full), the cache's hits and misses; reset with rst only
    output reg  [31:0] st_jobs, st_astall,
    output wire [31:0] st_hits, st_misses,
    // profile events: {the filter's head job waits for its bursts' data, the unit holds work}
    output wire [1:0]  st_ev
);
    localparam int W = 1 + EXP_W + MAN_W, M = MAN_W, E = EXP_W;
    // the block's own reset register: one cycle behind the input, replicated by synthesis near its loads
    (* max_fanout = 64 *) reg rst_q = 1'b1;
    always @(posedge clk) rst_q <= rst;
    localparam int BIAS = (1 << (E-1)) - 1, EMAX = (1 << E) - 1;
    localparam int CW = 44;                                  // coordinate fixed point: signed Q.24, |v| <= 2^41
    localparam signed [CW-1:0] LIMC = 44'sd2199023255552;    // 2^41
    localparam signed [14:0] LIMQ8 = 15'sd8191, NEGL = -15'sd16384;   // lambda clamp / "minus infinity"
    localparam [W-1:0] ONE = W'(BIAS) << M;

    // ================================================================ helpers
    // float -> signed fixed point with FB fraction bits, rounded toward -inf, saturated (tools/tmu_ref.py fix()), in
    // two pipeline stages. Stage A: {sat, neg, sticky, magnitude}: the mantissa shifted left by k (<= LB - M, larger
    // -> sat) or right by -k (the shifted-out bits -> sticky); LB = 41 (|v| <= 2^41) or 13 (Q.8 lambda: |v| <= 8191).
    function automatic [CW+2:0] fxa(input [W-1:0] b, input integer FB, input integer LB);
        reg s, sat, st; reg [E-1:0] e; reg [M:0] mant; reg signed [9:0] k; reg [5:0] kl, sh; reg [CW-1:0] mag;
        s = b[W-1]; e = b[W-2 -: E]; mant = {1'b1, b[M-1:0]};
        k = 10'(e) - 10'(BIAS) - 10'(M) + 10'(FB);
        sat = 1'b0; st = 1'b0; mag = '0;
        if (e == 0) ;
        else if (e == E'(EMAX)) sat = 1'b1;
        else if (LB == 41 ? ((k > 10'(41 - M)) || (k == 10'(41 - M) && b[M-1:0] != 0)) : (k >= 10'(13 - M))) sat = 1'b1;
        else if (k >= 0) begin kl = 6'(k); mag = CW'(mant) << kl; end
        else if (-k > 10'(M + 1)) st = 1'b1;
        else begin sh = 6'(-k); mag = CW'(mant >> sh); st = |(mant & (((M+1)'(1)) << sh) - 1'b1); end
        fxa = {sat, s & (e != 0), st, mag};
    endfunction
    // stage B: +-lim when saturated, -(mag + sticky) for negatives (floor), lim = 2^41 or 8191
    function automatic signed [CW-1:0] fxb(input [CW+2:0] a, input signed [CW-1:0] lim);
        reg sat, neg, st; reg [CW-1:0] mag; reg signed [CW-1:0] r;
        {sat, neg, st, mag} = a;
        if (sat) r = neg ? -lim : lim;
        else if (neg) begin r = -$signed(mag) - $signed(CW'(st)); if (r < -lim) r = -lim; end
        else r = $signed(mag);
        fxb = r;
    endfunction
    function automatic [5:0] msb64(input [63:0] v);           // index of the leading one (0 for v == 0)
        msb64 = 0; for (int i = 0; i < 64; i++) if (v[i]) msb64 = 6'(i);
    endfunction
    // round(256 * log2(1 + k/256)) (exact at powers of two; tools/tmu_ref.py LOG2T)
    localparam logic [7:0] LOG2T [0:255] = '{
        8'd0, 8'd1, 8'd3, 8'd4, 8'd6, 8'd7, 8'd9, 8'd10, 8'd11, 8'd13, 8'd14, 8'd16, 8'd17, 8'd18, 8'd20, 8'd21,
        8'd22, 8'd24, 8'd25, 8'd26, 8'd28, 8'd29, 8'd30, 8'd32, 8'd33, 8'd34, 8'd36, 8'd37, 8'd38, 8'd40, 8'd41, 8'd42,
        8'd44, 8'd45, 8'd46, 8'd47, 8'd49, 8'd50, 8'd51, 8'd52, 8'd54, 8'd55, 8'd56, 8'd57, 8'd59, 8'd60, 8'd61, 8'd62,
        8'd63, 8'd65, 8'd66, 8'd67, 8'd68, 8'd69, 8'd71, 8'd72, 8'd73, 8'd74, 8'd75, 8'd77, 8'd78, 8'd79, 8'd80, 8'd81,
        8'd82, 8'd84, 8'd85, 8'd86, 8'd87, 8'd88, 8'd89, 8'd90, 8'd92, 8'd93, 8'd94, 8'd95, 8'd96, 8'd97, 8'd98, 8'd99,
        8'd100, 8'd102, 8'd103, 8'd104, 8'd105, 8'd106, 8'd107, 8'd108, 8'd109, 8'd110, 8'd111, 8'd112, 8'd113, 8'd114, 8'd116, 8'd117,
        8'd118, 8'd119, 8'd120, 8'd121, 8'd122, 8'd123, 8'd124, 8'd125, 8'd126, 8'd127, 8'd128, 8'd129, 8'd130, 8'd131, 8'd132, 8'd133,
        8'd134, 8'd135, 8'd136, 8'd137, 8'd138, 8'd139, 8'd140, 8'd141, 8'd142, 8'd143, 8'd144, 8'd145, 8'd146, 8'd147, 8'd148, 8'd149,
        8'd150, 8'd151, 8'd152, 8'd153, 8'd154, 8'd155, 8'd155, 8'd156, 8'd157, 8'd158, 8'd159, 8'd160, 8'd161, 8'd162, 8'd163, 8'd164,
        8'd165, 8'd166, 8'd167, 8'd168, 8'd169, 8'd169, 8'd170, 8'd171, 8'd172, 8'd173, 8'd174, 8'd175, 8'd176, 8'd177, 8'd178, 8'd178,
        8'd179, 8'd180, 8'd181, 8'd182, 8'd183, 8'd184, 8'd185, 8'd185, 8'd186, 8'd187, 8'd188, 8'd189, 8'd190, 8'd191, 8'd192, 8'd192,
        8'd193, 8'd194, 8'd195, 8'd196, 8'd197, 8'd198, 8'd198, 8'd199, 8'd200, 8'd201, 8'd202, 8'd203, 8'd203, 8'd204, 8'd205, 8'd206,
        8'd207, 8'd208, 8'd208, 8'd209, 8'd210, 8'd211, 8'd212, 8'd212, 8'd213, 8'd214, 8'd215, 8'd216, 8'd216, 8'd217, 8'd218, 8'd219,
        8'd220, 8'd220, 8'd221, 8'd222, 8'd223, 8'd224, 8'd224, 8'd225, 8'd226, 8'd227, 8'd228, 8'd228, 8'd229, 8'd230, 8'd231, 8'd231,
        8'd232, 8'd233, 8'd234, 8'd234, 8'd235, 8'd236, 8'd237, 8'd238, 8'd238, 8'd239, 8'd240, 8'd241, 8'd241, 8'd242, 8'd243, 8'd244,
        8'd244, 8'd245, 8'd246, 8'd247, 8'd247, 8'd248, 8'd249, 8'd249, 8'd250, 8'd251, 8'd252, 8'd252, 8'd253, 8'd254, 8'd255, 8'd255};


    // ================================================================ request queue (16 = four warps' TEXes)
    // (with 4, one warp's TEX fills it, the core's staging then waits for F1 / F2 to drain the four quads, so every
    // TEX's TMU time adds to the issue time: +4 cycles per quad per sample)
    localparam int RQ = 16;
    reg [4:0] rq_h, rq_t;
    // rq_c in LUTRAM: as a block RAM, f1_c would be its output (2 ns clock-to-out into F1 / F2's logic)
    reg [4:0] rq_s [0:RQ-1]; reg [1:0] rq_m [0:RQ-1]; (* ram_style="distributed" *) reg [16*W-1:0] rq_c [0:RQ-1];
    (* ram_style="distributed" *) reg [4:0] rq_mk [0:RQ-1];   // the request's {writer class, sample mask}
    assign req_ready = ((rq_t - rq_h) < 5'(RQ)) & ~rst_q;
    wire rq_ne = (rq_t != rq_h);
    assign desc_idx = rq_s[rq_h[3:0]];

    // ================================================================ per-{bank, sampler} level table
    // lt_off[{bank, samp, level}] = word offset of the level in one face; lt_chain = one face's mip chain; key = what
    // they depend on. Recomputed (16 cycles) when a request's descriptor differs from the key. Keyed by the 5-bit
    // request sampler: with the sampler's 4 bits alone, two draws in flight binding
    // textures of different sizes to the same slot would recompute the table on EVERY request (15 cycles, F1 blocked).
    // lt_off in a RAMB18 with a registered read: the tap stage T2 presents {sampler, level}, T3 uses
    // the offset
    (* ram_style="block" *) reg [25:0] lt_off [0:511];
    reg [25:0] lt_rd;
    // lt_chain / lt_key in LUTRAM (7 RAM32M read asynchronously from f1_s): in a block RAM, lt_key's 2.1 ns clock-to-out
    // sits in front of the 37-bit key compare -> f2_take
    (* ram_style = "distributed" *) reg [25:0] lt_chain [0:31];
    (* ram_style = "distributed" *) reg [36:0] lt_key [0:31];
    reg [31:0] lt_ok;
    reg lt_busy; reg [4:0] lt_l; reg [4:0] lt_s; reg [25:0] lt_o; reg [11:0] lt_w, lt_h, lt_wl, lt_hl; reg [23:0] lt_sz; reg [3:0] lt_lv; reg lt_b16;
    reg [7:0] lt_d, lt_dl, lt_dl2; reg [25:0] lt_sv;

    // ================================================================ F1: take a request + its descriptor
    reg f1_v; reg [4:0] f1_s; reg [1:0] f1_m; reg [16*W-1:0] f1_c; reg [4:0] f1_mk;
    reg [28:0] f1_base; reg [11:0] f1_w, f1_h; reg [1:0] f1_fmt, f1_ws, f1_wt; reg [3:0] f1_lv, f1_filt; reg f1_cube, f1_inc;
    reg [7:0] f1_d; reg [1:0] f1_wr; reg f1_vol;
    wire [36:0] f1_key = {f1_d, f1_w, f1_h, f1_lv, f1_fmt != 2'd0};
    wire f1_ltok = lt_ok[f1_s] && (lt_key[f1_s] == f1_key);
    wire f2_take;                                            // F2 accepts F1's quad this cycle
    wire f1_free = ~f1_v | f2_take;

    // ================================================================ F2: coordinates -> wait FIFO (+ divider ops)
    // One pixel per cycle (4 cycles a quad; a cube quad 4 + projections, issuing its divider ops): x, y -> Q.24 and
    // w -> Q.8 in two stages (fxa, fxb) into the per-pixel wait-FIFO memories; the quad's own fields at the take,
    // the entry committed one cycle later (after its last pixel's stage B).
    // A quad's record stays in its slot from the F2 take until F4 takes the quad (the 525-bit record does not travel
    // through F3's seven stages: that would be ≈ 2.4 k FF + 0.5 k SRL).
    // Pointers: wf_t fills, wf_r is F3's head (pop), wf_h frees (F4 take); WF 16 keeps F2's room ≥ 8 waiting quads.
    localparam int WF = 16;
    reg [4:0] wf_h, wf_t, wf_r; reg wf_cp;
    wire [3:0] wf_slot = 4'(wf_t + 5'(wf_cp));                // the slot F2 fills
    wire [3:0] wr = wf_r[3:0];                                // F3's head slot
    reg  [3:0] f3e_slot, f3f_slot;                            // the slots of the quads in F3's stages e and f
    reg [2:0] wf_face [0:WF-1][0:3];
    reg [1:0] wf_psel [0:WF-1][1:3];                          // F3's coordinate source per pixel 1..3: projection j (0..2) or 3 = own face
                                                              // (decoded at the slot's fill: the face compares and the
                                                              // running count in front of stage a's subtract are 17 levels)
    wire signed [CW-1:0] wsx [0:3], wtx [0:3], wrx [0:3];     // the F3 head's per-pixel coordinates (stage a)
    wire signed [CW-1:0] fsx [0:3], ftx [0:3], frx [0:3]; wire [14:0] eq8 [0:3];   // F4's quad (stage f's slot); stage f's q8 (stage e's slot)
    reg [4:0] wf_s [0:WF-1]; reg [1:0] wf_m [0:WF-1]; reg [28:0] wf_base [0:WF-1]; reg [11:0] wf_w [0:WF-1], wf_h2 [0:WF-1];
    reg [1:0] wf_fmt [0:WF-1], wf_ws [0:WF-1], wf_wt [0:WF-1]; reg [3:0] wf_lv [0:WF-1], wf_filt [0:WF-1];
    reg wf_cube [0:WF-1], wf_inc [0:WF-1]; reg [2:0] wf_np [0:WF-1]; reg [25:0] wf_chain [0:WF-1];
    reg [7:0] wf_d [0:WF-1]; reg [1:0] wf_wr [0:WF-1]; reg wf_vol [0:WF-1];
    (* ram_style="distributed" *) reg [4:0] wf_mk [0:WF-1];   // the quad's {writer class, sample mask} (read at F4's take)
    wire wf_full = ((wf_t + 5'(wf_cp)) - wf_h) >= 5'(WF);
    // cube face per pixel (F1 registers)
    function automatic [2:0] cface(input [W-1:0] x, y, z);
        reg [W-2:0] ax, ay, az;
        ax = x[W-2:0]; ay = y[W-2:0]; az = z[W-2:0];
        if (ax >= ay && ax >= az) cface = {2'd0, x[W-1]};
        else if (ay >= az)        cface = {2'd1, y[W-1]};
        else                      cface = {2'd2, z[W-1]};
    endfunction
    // the faces are registered with the request at F1 (keeps cface off the path f1_c -> f2_diff -> f2_take -> the slot
    // arrays' write enables)
    (* max_fanout = 64 *) reg [2:0] f1_face [0:3];               // f1_face[0] drives ≈ 476 loads (1.2 ns of route unreplicated)
    genvar gp;
    // divider ops of a cube quad: 0..3 own faces, then the pixels whose face differs from pixel 0's, projected
    reg [2:0] f2_op;                                         // cycle of the quad in F2 (pixel / op index)
    wire [3:0] f2_diff = {f1_face[3] != f1_face[0], f1_face[2] != f1_face[0], f1_face[1] != f1_face[0], 1'b0};
    wire [2:0] f1_np = 3'(f2_diff[1]) + 3'(f2_diff[2]) + 3'(f2_diff[3]);
    // op k -> (pixel, face, proj): k < 4: (k, face_k, 0); else the (k-4)-th pixel with a different face
    reg [1:0] op_p; reg [2:0] op_f; reg op_proj;
    always @* begin
        op_p = f2_op[1:0]; op_f = f1_face[f2_op[1:0]]; op_proj = 1'b0;
        if (f2_op >= 3'd4) begin
            op_proj = 1'b1; op_f = f1_face[0];
            case (f2_op - 3'd4)
                3'd0:    op_p = f2_diff[1] ? 2'd1 : f2_diff[2] ? 2'd2 : 2'd3;
                3'd1:    op_p = (f2_diff[1] & f2_diff[2]) ? 2'd2 : 2'd3;
                default: op_p = 2'd3;
            endcase
        end
    end
    wire [2:0] f2_nops = 3'd4 + f1_np;
    wire f2_cube = f1_cube & ~f1_inc;
    wire f2_run = f1_v & f1_ltok & ~wf_full;                 // divider results live per wait-FIFO slot: no back-pressure
    wire div_in_v = f2_run & f2_cube;                         // one divider op per cycle
    // F2 converts TWO pixels per cycle for 2D / 3D quads (at one pixel per cycle F2 is 4 cycles of the round trip
    // and sets the front end's rate, busy 70-80 % on two-texture content); cube quads keep one
    // pixel per cycle, each with its divider op. The pair of cycle f2_op: pixels {f2_hi, 0} (set A) and {f2_hi, 1} (set B).
    wire f2_dual = F2_DUAL & ~f2_cube;
    wire f2_hi = f2_dual ? f2_op[0] : f2_op[1];
    wire [1:0] f2_pa = f2_dual ? {f2_hi, 1'b0} : f2_op[1:0];    // set A's pixel (set B's: {f2_hi, 1})
    assign f2_take = f2_run & (f2_op == (f2_dual ? 3'd1 : f2_cube ? f2_nops - 3'd1 : 3'd3));
    // stage A of the conversions, stage B writes the pixels' memories: with F2_DUAL even pixels come from set A and odd
    // ones from set B (cube quads' odd pixels too, one per cycle); without it every pixel comes through set A
    reg fb_va, fb_vb, fb_hi; reg [1:0] fb_pa; reg [3:0] fb_slot;
    reg [CW+2:0] fba_s, fba_t, fba_r, fba_q, fbb_s, fbb_t, fbb_r, fbb_q;
    generate for (gp = 0; gp < 4; gp++) begin : WFP
        (* ram_style="distributed" *) reg [CW-1:0] sx [0:WF-1], tx [0:WF-1], rx [0:WF-1];
        (* ram_style="distributed" *) reg [14:0] q8 [0:WF-1];
        wire fb_we; wire [CW+2:0] fb_s, fb_t, fb_r, fb_q;
        if (F2_DUAL && (gp % 2)) begin : SB
            assign fb_we = fb_vb & (fb_hi == ((gp / 2) != 0));
            assign fb_s = fbb_s; assign fb_t = fbb_t; assign fb_r = fbb_r; assign fb_q = fbb_q;
        end else begin : SA
            assign fb_we = fb_va & (F2_DUAL ? (fb_hi == ((gp / 2) != 0)) : (fb_pa == 2'(gp)));
            assign fb_s = fba_s; assign fb_t = fba_t; assign fb_r = fba_r; assign fb_q = fba_q;
        end
        always @(posedge clk) if (fb_we) begin
            sx[fb_slot] <= fxb(fb_s, LIMC); tx[fb_slot] <= fxb(fb_t, LIMC); rx[fb_slot] <= fxb(fb_r, LIMC);
            q8[fb_slot] <= 15'(fxb(fb_q, CW'(LIMQ8)));
        end
        assign wsx[gp] = sx[wr]; assign wtx[gp] = tx[wr]; assign wrx[gp] = rx[wr];                   // stage a: the F3 head
        assign fsx[gp] = sx[f3f_slot]; assign ftx[gp] = tx[f3f_slot]; assign frx[gp] = rx[f3f_slot];  // F4's take: stage f's quad
        assign eq8[gp] = q8[f3e_slot];                                     // stage f: the per-pixel lambda term of stage e's quad
    end endgenerate

    // ---- cube divider pipeline: R = floor(2^(M+26) / A); then q = sc / |ma|, sfix = (+-q*2^25 + 2^25) / 4 ----
    // op operands (sc, tc, ma with the table's signs), special cases resolved up front
    function automatic [3*W-1:0] cube_sel(input [W-1:0] x, y, z, input [2:0] f);   // {ma, tc, sc}
        reg [W-1:0] sc, tc, ma; reg [W-1:0] sb; sb = {1'b1, {(W-1){1'b0}}};
        case (f)
            3'd0: begin sc = z ^ sb; tc = y ^ sb; ma = x; end   // +X: -rz, -ry
            3'd1: begin sc = z;      tc = y ^ sb; ma = x; end   // -X: +rz, -ry
            3'd2: begin sc = x;      tc = z;      ma = y; end   // +Y: +rx, +rz
            3'd3: begin sc = x;      tc = z ^ sb; ma = y; end   // -Y: +rx, -rz
            3'd4: begin sc = x;      tc = y ^ sb; ma = z; end   // +Z: +rx, -ry
            default: begin sc = x ^ sb; tc = y ^ sb; ma = z; end   // -Z: -rx, -ry
        endcase
        cube_sel = {ma, tc, sc};
    endfunction
    wire [3*W-1:0] opv = cube_sel(f1_c[(op_p*4+0)*W +: W], f1_c[(op_p*4+1)*W +: W], f1_c[(op_p*4+2)*W +: W], op_f);
    wire [W-1:0] op_sc = opv[0 +: W], op_tc = opv[W +: W], op_ma = opv[2*W +: W];
    wire [E-1:0] op_me = op_ma[W-2 -: E];
    wire op_spec = (op_me == 0) | (op_proj & (op_ma[W-1] != op_f[0]));
    // the reciprocal R = floor(2^(M+26) / A), A = {1, ma}: for M = 16 cube_rcp (a 12-stage Newton pipeline with an exact
    // final correction, verified exhaustively by sim/tb_cube_rcp.sv), for other mantissa
    // widths a 27-stage restoring divider (≈ 3.0 k LUT at M = 16). DQ = the latency from the registered
    // operand to R; RW = R's width (R <= 2^26).
    localparam int RW = 27, DQ = (M == 16) ? 12 : 27;
    reg [DQ:0] dv_v;
    // side data (fixed latency, no reset -> shift registers): {slot, pixel, projection index, spec, proj, ma exponent,
    // tc, sc, 0}; slot / pixel / index name where the result is stored
    localparam int SDW = 2*W + E + 4 + 7;                      // the slot is 4 bits (WF 16)
    reg [SDW-1:0] dv_sd [0:DQ];
    wire [RW-1:0] dR;
    always @(posedge clk) begin
        dv_v[0] <= div_in_v & ~rst_q;
        dv_sd[0] <= {wf_slot, op_p, 2'(f2_op - 3'd4), op_spec, op_proj, (op_me == E'(EMAX)) ? E'(EMAX - 1) : op_me,
                     op_tc, op_sc, 1'b0};
        for (int k = 0; k < DQ; k++) begin dv_v[k+1] <= dv_v[k] & ~rst_q; dv_sd[k+1] <= dv_sd[k]; end
    end
    generate if (M == 16) begin : RCP
        cube_rcp #(.M(M)) U_RCP(.clk, .a({1'b1, op_ma[M-1:0]}), .r(dR));
    end else begin : DIV                                      // restoring: compare, subtract, next dividend bit (0) in
        reg [M+1:0] dv_r [0:DQ]; reg [M:0] dv_a [0:DQ]; reg [RW-1:0] dv_q [0:DQ];
        always @(posedge clk) begin
            dv_a[0] <= {1'b1, op_ma[M-1:0]}; dv_r[0] <= (M+2)'(1) << M; dv_q[0] <= '0;   // the dividend's leading 1 and M zeros
            for (int k = 0; k < DQ; k++) begin
                dv_a[k+1] <= dv_a[k];
                if (dv_r[k] >= {1'b0, dv_a[k]}) begin dv_r[k+1] <= (dv_r[k] - {1'b0, dv_a[k]}) << 1; dv_q[k+1] <= {dv_q[k][RW-2:0], 1'b1}; end
                else                             begin dv_r[k+1] <= dv_r[k] << 1;                     dv_q[k+1] <= {dv_q[k][RW-2:0], 1'b0}; end
            end
        end
        assign dR = dv_q[DQ];
    end endgenerate
    // post: P = S * R, shift by M + 1 + ea - es, sfix = (+-qq + 2^25) >> 2
    wire [SDW-1:0] dsd = dv_sd[DQ];
    wire [3:0] d_slot = dsd[SDW-1 -: 4]; wire [1:0] d_px = dsd[SDW-5 -: 2], d_j = dsd[SDW-7 -: 2];
    wire d_spec = dsd[SDW-9], d_proj = dsd[SDW-10]; wire [E-1:0] d_me = dsd[SDW-11 -: E];
    wire [W-1:0] d_tc = dsd[1+W +: W], d_sc = dsd[1 +: W];
    reg p1_v, p1_spec, p1_proj; reg [M+RW:0] p1_ps, p1_pt; reg signed [9:0] p1_shs, p1_sht; reg p1_ss, p1_st, p1_zs, p1_zt;
    reg [3:0] p1_slot, p2_slot, d2_slot; reg [1:0] p1_px, p1_j, p2_px, p2_j, d2_px, d2_j;
    always @(posedge clk) begin
        p1_v <= dv_v[DQ] & ~rst_q; p1_spec <= d_spec; p1_proj <= d_proj; p1_slot <= d_slot; p1_px <= d_px; p1_j <= d_j;
        p1_ps <= {1'b1, d_sc[M-1:0]} * dR; p1_pt <= {1'b1, d_tc[M-1:0]} * dR;
        p1_shs <= 10'(M + 1) + 10'(d_me) - 10'(d_sc[W-2 -: E]); p1_sht <= 10'(M + 1) + 10'(d_me) - 10'(d_tc[W-2 -: E]);
        p1_ss <= d_sc[W-1]; p1_st <= d_tc[W-1]; p1_zs <= (d_sc[W-2 -: E] == 0); p1_zt <= (d_tc[W-2 -: E] == 0);
    end
    // p1b: |P| shifted (the two-way 64-bit barrel shift and the zero cases in a stage of their own: shift + clamp
    //      into p2 are 18 levels, too many at 120 MHz); p2: |q| clamped to LIMC;
    //      d2: sfix = (+-|q| + 2^25) >> 2. The pipe is latency-transparent: results land in the wait-FIFO slots, F3 counts them.
    function automatic [63:0] cq_sh(input [M+RW:0] P, input signed [9:0] sh, input z);
        if (z) cq_sh = 64'd0;
        else if (sh >= 0) cq_sh = (sh > 63) ? 64'd0 : (64'(P) >> sh);
        else cq_sh = (-sh > 30) ? 64'(LIMC) : (64'(P) << (-sh));
    endfunction
    function automatic [CW-1:0] cq_clamp(input [63:0] qq);
        cq_clamp = (qq > 64'(LIMC)) ? CW'(LIMC) : CW'(qq);
    endfunction
    function automatic signed [CW-1:0] cq_fix(input [CW-1:0] q, input sg);
        reg signed [CW-1:0] v;
        v = (sg ? -q : q) + (CW'(1) << 25);
        cq_fix = v >>> 2;
    endfunction
    reg p1b_v, p1b_spec, p1b_proj, p1b_ss, p1b_st; reg [63:0] p1b_qs, p1b_qt; reg [3:0] p1b_slot; reg [1:0] p1b_px, p1b_j;
    always @(posedge clk) begin
        p1b_v <= p1_v & ~rst_q; p1b_spec <= p1_spec; p1b_proj <= p1_proj; p1b_ss <= p1_ss; p1b_st <= p1_st;
        p1b_slot <= p1_slot; p1b_px <= p1_px; p1b_j <= p1_j;
        p1b_qs <= cq_sh(p1_ps, p1_shs, p1_zs); p1b_qt <= cq_sh(p1_pt, p1_sht, p1_zt);
    end
    reg p2_v, p2_spec, p2_proj, p2_ss, p2_st; reg [CW-1:0] p2_qs, p2_qt;
    always @(posedge clk) begin
        p2_v <= p1b_v & ~rst_q; p2_spec <= p1b_spec; p2_proj <= p1b_proj; p2_ss <= p1b_ss; p2_st <= p1b_st;
        p2_slot <= p1b_slot; p2_px <= p1b_px; p2_j <= p1b_j;
        p2_qs <= cq_clamp(p1b_qs); p2_qt <= cq_clamp(p1b_qt);
    end
    reg d2_v, d2_proj; reg signed [CW-1:0] d2_s, d2_t;
    always @(posedge clk) begin
        d2_v <= p2_v & ~rst_q; d2_proj <= p2_proj; d2_slot <= p2_slot; d2_px <= p2_px; d2_j <= p2_j;
        if (p2_spec) begin d2_s <= p2_proj ? LIMC : CW'(1 << 23); d2_t <= p2_proj ? LIMC : CW'(1 << 23); end
        else begin d2_s <= cq_fix(p2_qs, p2_ss); d2_t <= cq_fix(p2_qt, p2_st); end
    end
    // results per wait-FIFO slot: the own-face (s, t) of pixel p in cs / ct [p], the j-th projection
    // (the j-th pixel whose face differs from pixel 0's) in ps / pt [j]; F3 reads its head slot's fixed positions, F4
    // its quad's own-face coordinates. wf_rc[slot] counts the results landed: F3 waits for 4 + np of them.
    wire signed [CW-1:0] cs_h [0:3], ct_h [0:3], ps_h [0:2], pt_h [0:2], cs_f [0:3], ct_f [0:3];
    generate for (gp = 0; gp < 4; gp++) begin : CSP
        (* ram_style="distributed" *) reg [CW-1:0] cs [0:WF-1], ct [0:WF-1];
        always @(posedge clk) if (d2_v && !d2_proj && d2_px == 2'(gp)) begin cs[d2_slot] <= d2_s; ct[d2_slot] <= d2_t; end
        assign cs_h[gp] = cs[wr]; assign ct_h[gp] = ct[wr];
        assign cs_f[gp] = cs[f3f_slot]; assign ct_f[gp] = ct[f3f_slot];
    end endgenerate
    generate for (gp = 0; gp < 3; gp++) begin : PSP
        (* ram_style="distributed" *) reg [CW-1:0] ps [0:WF-1], pt [0:WF-1];
        always @(posedge clk) if (d2_v && d2_proj && d2_j == 2'(gp)) begin ps[d2_slot] <= d2_s; pt[d2_slot] <= d2_t; end
        assign ps_h[gp] = ps[wr]; assign pt_h[gp] = pt[wr];
    end endgenerate
    reg [2:0] wf_rc [0:WF-1];

    // ================================================================ F3: LOD (from the wait FIFO's F3 head, slot wr)
    // 2D quads go as soon as they are at the head; cube quads once their 4 + np divider results are in. The pipeline
    // carries the quad's SLOT and its LOD terms only; everything else stays in the slot (read again by stage f and F4).
    wire wf_ne = (wf_t != wf_r);
    wire [2:0] hd_ops = 3'd4 + wf_np[wr];
    wire f3_in = wf_ne & (~wf_cube[wr] | wf_inc[wr] | (wf_rc[wr] == hd_ops));
    // F3 runs freely (no enable on its flops); its stage e and a hold register form a 2-entry skid
    // buffer towards stage f (the per-pixel lambda), and f3_n counts the quads popped and not yet taken by
    // F4 (≤ 3: F4 takes one quad per ≥ 4 cycles, so three in flight keep it fed through the 6-stage pipe)
    reg [1:0] f3_n;
    wire f3_pop = f3_in & (f3_n != 2'd3);                      // (3 with stage f: a..e + hold + f hold 7)
    // the head's coordinates for the derivatives: 2D from the FIFO; cube from the slot's divider results (own face,
    // then the projections of the pixels whose face differs from pixel 0's)
    reg signed [CW-1:0] h_ls [0:3], h_lt [0:3];
    always @* begin
        h_ls[0] = wf_cube[wr] ? cs_h[0] : wsx[0]; h_lt[0] = wf_cube[wr] ? ct_h[0] : wtx[0];
        for (int p = 1; p < 4; p++) begin : proj
            reg [1:0] k; k = wf_psel[wr][p];                   // (the j-th pixel with a face other than pixel 0's: projection j)
            h_ls[p] = ~wf_cube[wr] ? wsx[p] : (k == 2'd3) ? cs_h[p] : ps_h[(k == 2'd3) ? 2'd2 : k];
            h_lt[p] = ~wf_cube[wr] ? wtx[p] : (k == 2'd3) ? ct_h[p] : pt_h[(k == 2'd3) ? 2'd2 : k];
        end
    end
    // F3 pipeline: a |differences| (TR - TL, BL - TL; six terms, r's two count for 3D only) -> b leading one + 18-bit
    // mantissa -> c x dim -> d log2 per term -> d3 max per axis pair -> e max -> f per-pixel lambda. Each stage carries
    // its quad's slot. (d3: the max of six in one stage is three compares in a row.)
    function automatic [CW:0] absd(input signed [CW-1:0] a, input signed [CW-1:0] b);
        reg signed [CW:0] d; d = {a[CW-1], a} - {b[CW-1], b}; absd = d[CW] ? -d : d;
    endfunction
    function automatic signed [14:0] mx2(input signed [14:0] a, input signed [14:0] b); mx2 = (a > b) ? a : b; endfunction
    reg f3a_v, f3b_v, f3c_v, f3d2_v, f3d3_v, f3e_v, f3h_v, f3a_vol;
    reg [3:0] f3a_slot, f3b_slot, f3c_slot, f3d2_slot, f3d3_slot, f3h_slot;
    reg [CW:0] f3a_d [0:5]; reg [11:0] f3a_dim [0:5], f3b_dim [0:5];
    reg [5:0] f3b_ed [0:5], f3c_ed [0:5]; reg [17:0] f3b_m [0:5]; reg [5:0] f3b_nz, f3c_nz;
    reg [29:0] f3c_t [0:5]; reg signed [14:0] f3d2_l [0:5], f3d3_l [0:2]; reg signed [14:0] f3e_lb, f3h_lb; reg f3e_neg, f3h_neg;
    // f: the per-pixel lambda (lambda_base + the per-pixel q8 term, clamped) in a stage of its own ahead of F4's level
    // select (f3e_lb -> add -> clamp -> the level thresholds -> f4_sel in one stage is 15 levels)
    reg f3f_v; reg signed [14:0] f3f_lam [0:3];
    function automatic signed [14:0] clq(input signed [15:0] v);
        clq = (v > 16'(LIMQ8)) ? LIMQ8 : (v < -16'(LIMQ8)) ? -LIMQ8 : 15'(v);
    endfunction
    wire [1:0] f3e_m = wf_m[f3e_slot];
    wire f4_take;                                             // F4 takes the F3f quad
    wire f3f_free = ~f3f_v | f4_take;                         // f is empty or emptied this cycle
    wire f3f_take = f3e_v & f3f_free;                         // f takes e
    wire f3e_free = ~f3e_v | f3f_take;                        // e is empty or emptied this cycle
    // ---- e's input: the max of d3's three ----
    wire signed [14:0] mx_lb = mx2(mx2(f3d3_l[0], f3d3_l[1]), f3d3_l[2]);
    wire mx_neg = (mx_lb == NEGL);
    always @(posedge clk) begin
        if (rst_q) begin f3a_v <= 0; f3b_v <= 0; f3c_v <= 0; f3d2_v <= 0; f3d3_v <= 0; f3e_v <= 0; f3h_v <= 0; f3f_v <= 0; f3_n <= 2'd0; end
        else begin
            f3a_v <= f3_pop; f3b_v <= f3a_v; f3c_v <= f3b_v; f3d2_v <= f3c_v; f3d3_v <= f3d2_v;
            if (f3f_free) f3f_v <= f3e_v;
            // the skid: e takes the hold register first, else d3; d3's quad goes to hold while e is busy (hold is free
            // then: at most 2 quads are in a..e + hold)
            if (f3e_free) begin
                if (f3h_v) begin f3e_v <= 1'b1; f3h_v <= f3d3_v; end
                else f3e_v <= f3d3_v;
            end else if (f3d3_v) f3h_v <= 1'b1;
            f3_n <= f3_n + 2'(f3_pop) - 2'(f4_take & f3f_v);
        end
        // the data advances every cycle (no enable); e / hold as above
        begin
            // ---- f: the per-pixel lambda (the q8 term read from stage e's slot) ----
            if (f3f_free) begin
                f3f_slot <= f3e_slot;
                for (int p = 0; p < 4; p++) begin : lm
                    reg signed [14:0] q8; q8 = eq8[p];
                    if (f3e_m == 2'd2)      f3f_lam[p] <= q8;
                    else if (f3e_neg)       f3f_lam[p] <= NEGL;
                    else if (f3e_m == 2'd1) f3f_lam[p] <= clq(16'(f3e_lb) + 16'(q8));
                    else                    f3f_lam[p] <= clq(16'(f3e_lb));
                end
            end
            // ---- a ----
            f3a_slot <= wr;
            f3a_d[0] <= absd(h_ls[1], h_ls[0]); f3a_d[1] <= absd(h_ls[2], h_ls[0]);
            f3a_d[2] <= absd(h_lt[1], h_lt[0]); f3a_d[3] <= absd(h_lt[2], h_lt[0]);
            f3a_d[4] <= absd(wrx[1], wrx[0]);   f3a_d[5] <= absd(wrx[2], wrx[0]);
            f3a_dim[0] <= wf_w[wr]; f3a_dim[1] <= wf_w[wr];
            f3a_dim[2] <= wf_cube[wr] ? wf_w[wr] : wf_h2[wr]; f3a_dim[3] <= wf_cube[wr] ? wf_w[wr] : wf_h2[wr];
            f3a_dim[4] <= 12'(wf_d[wr]); f3a_dim[5] <= 12'(wf_d[wr]); f3a_vol <= wf_vol[wr];
            // ---- b: ed = leading one of |d|, m18 = its top 18 bits ----
            f3b_slot <= f3a_slot;
            for (int k = 0; k < 6; k++) begin : nb
                reg [5:0] ed;
                ed = msb64(64'(f3a_d[k]));
                f3b_m[k] <= (ed >= 6'd17) ? 18'(f3a_d[k] >> (ed - 6'd17)) : 18'(f3a_d[k] << (6'd17 - ed));
                f3b_ed[k] <= ed; f3b_nz[k] <= (f3a_d[k] != 0) & ((k < 4) | f3a_vol); f3b_dim[k] <= f3a_dim[k];
            end
            // ---- c: t = m18 * dim ----
            f3c_slot <= f3b_slot;
            for (int k = 0; k < 6; k++) begin f3c_t[k] <= f3b_m[k] * f3b_dim[k]; f3c_ed[k] <= f3b_ed[k]; end
            f3c_nz <= f3b_nz;
            // ---- d: lambda_k = (ed - 17 + mt - 24) * 256 + log2t(next 8 bits of t) ----
            f3d2_slot <= f3c_slot;
            for (int k = 0; k < 6; k++) begin : lg
                reg [5:0] mt; reg [7:0] k8;
                mt = msb64(64'(f3c_t[k]));
                k8 = 8'(f3c_t[k] >> (mt - 6'd8));
                f3d2_l[k] <= f3c_nz[k] ? (15'(signed'(7'(f3c_ed[k]) - 7'sd17 + 7'(mt) - 7'sd24)) * 15'sd256 + 15'(LOG2T[k8])) : NEGL;
            end
            // ---- d3: max per axis ----
            f3d3_slot <= f3d2_slot;
            for (int k = 0; k < 3; k++) f3d3_l[k] <= mx2(f3d2_l[2*k], f3d2_l[2*k+1]);
            // ---- e: lambda_base = max (from hold when it is occupied; d3 into hold when e is busy) ----
            if (f3e_free) begin
                if (f3h_v) begin f3e_slot <= f3h_slot; f3e_lb <= f3h_lb; f3e_neg <= f3h_neg; end
                else begin f3e_slot <= f3d3_slot; f3e_lb <= mx_lb; f3e_neg <= mx_neg; end
            end
            if (f3d3_v & (f3h_v | ~f3e_free)) begin f3h_slot <= f3d3_slot; f3h_lb <= mx_lb; f3h_neg <= mx_neg; end
        end
    end
    // the F3 output (stage f) as F4 reads it: the quad's record from its slot (freed when F4 takes it)
    wire f3d_v = f3f_v;
    wire [3:0] fs = f3f_slot;
    wire f3d_cube = wf_cube[fs], f3d_inc = wf_inc[fs];
    wire signed [CW-1:0] f3d_s [0:3], f3d_t [0:3]; wire [2:0] f3d_face [0:3];
    generate for (gp = 0; gp < 4; gp++) begin : F3O
        assign f3d_s[gp] = f3d_cube ? cs_f[gp] : fsx[gp]; assign f3d_t[gp] = f3d_cube ? ct_f[gp] : ftx[gp];
        assign f3d_face[gp] = wf_face[fs][gp];
    end endgenerate
    wire [4:0] f3d_samp = wf_s[fs];
    wire [3:0] f3d_lv = wf_lv[fs], f3d_filt = wf_filt[fs];
    wire [1:0] f3d_fmt = wf_fmt[fs], f3d_ws = wf_ws[fs], f3d_wt = wf_wt[fs];
    wire [28:0] f3d_base = wf_base[fs]; wire [11:0] f3d_w = wf_w[fs], f3d_h = wf_h2[fs];
    wire [25:0] f3d_chain = wf_chain[fs];
    wire [7:0] f3d_d = wf_d[fs]; wire [1:0] f3d_wr = wf_wr[fs]; wire f3d_vol = wf_vol[fs];
    // ================================================================ F4: level select per pixel, jobs
    // select: {two, lin, d1, d2, lw1, lw2}
    function automatic [27:0] sel_lv(input signed [14:0] lam, input [3:0] filt, input [3:0] levels);
        reg magl, minl; reg [1:0] mip; reg [3:0] q, d1; reg signed [14:0] c; reg [8:0] fr; reg two, lin; reg [3:0] dd2;
        reg [8:0] w1, w2;
        magl = filt[3]; mip = filt[2:1]; minl = filt[0];
        q = (levels == 0) ? 4'd0 : levels - 4'd1;
        c = (magl & ~minl & (mip != 2'd0) & (mip != 2'd3)) ? 15'sd128 : 15'sd0;   // c = 0.5: MAG LINEAR, MIN NEAREST_MIPMAP_*
        two = 1'b0; d1 = 0; dd2 = 0; w1 = 9'd256; w2 = 9'd0;
        if (lam <= c) lin = magl;
        else begin
            lin = minl;
            if (mip == 2'd1) begin
                if (lam <= 15'sd128) d1 = 0;
                else begin : nm
                    reg signed [14:0] dn; dn = (lam + 15'sd127) >>> 8;
                    d1 = (dn > 15'(q)) ? q : 4'(dn);
                end
            end else if (mip >= 2'd2) begin
                if (lam >= 15'(q) * 15'sd256) d1 = q;
                else begin
                    d1 = 4'(lam >>> 8); fr = {1'b0, lam[7:0]};
                    if (fr != 0) begin two = 1'b1; dd2 = d1 + 4'd1; w1 = 9'd256 - fr; w2 = fr; end
                end
            end
        end
        sel_lv = {two, lin, d1, dd2, w1, w2};
    endfunction
    reg f4_v; reg [27:0] f4_sel [0:3];
    reg signed [CW-1:0] f4_s [0:3], f4_t [0:3]; reg [2:0] f4_face [0:3];
    reg [4:0] f4_samp; reg [28:0] f4_base; reg [11:0] f4_w, f4_h; reg [1:0] f4_fmt, f4_ws, f4_wt; reg f4_inc; reg [25:0] f4_chain;
    reg signed [CW-1:0] f4_r [0:3]; reg [7:0] f4_d; reg [1:0] f4_wr; reg f4_vol;
    reg [1:0] f4_p; reg f4_k, f4_z;                           // job emission: pixel, level index, slice index
    // jobs for the pixels of the quad's sample mask only: the emission starts at the mask's first
    // pixel and steps to its next one; the quad's last job is the last job of the mask's last pixel
    reg [3:0] f4_mk; reg f4_cl;
    wire [3:0] f3d_mk = wf_mk[fs][3:0];
    wire [3:0] f4_above = f4_mk & (4'hE << f4_p);             // the mask's pixels after f4_p
    wire f4_plast = (f4_above == 4'd0);
    wire [1:0] f4_pn = f4_above[1] ? 2'd1 : f4_above[2] ? 2'd2 : 2'd3;
    wire t_take;                                              // the tap stage takes a job this cycle
    wire f4_two = f4_sel[f4_p][27];
    wire f4_zl = f4_vol & f4_sel[f4_p][26];                   // 3D LINEAR: two jobs per level
    wire f4_zlast = ~f4_zl | f4_z;                            // the level's last job
    wire f4_lastjob = f4_plast & (f4_inc | (f4_zlast & (~f4_two | f4_k)));
    assign f4_take = ~f4_v | (t_take & f4_lastjob);
    always @(posedge clk) begin
        if (rst_q) f4_v <= 1'b0;
        else begin
            if (f4_take) begin
                f4_v <= f3d_v; f4_k <= 0; f4_z <= 0; f4_mk <= f3d_mk; f4_cl <= wf_mk[fs][4];
                f4_p <= f3d_mk[0] ? 2'd0 : f3d_mk[1] ? 2'd1 : f3d_mk[2] ? 2'd2 : 2'd3;
                for (int p = 0; p < 4; p++) begin : sl
                    f4_sel[p] <= sel_lv(f3f_lam[p], f3d_filt, f3d_lv);
                    f4_s[p] <= f3d_s[p]; f4_t[p] <= f3d_t[p]; f4_r[p] <= frx[p]; f4_face[p] <= f3d_cube ? f3d_face[p] : 3'd0;
                end
                f4_samp <= f3d_samp; f4_base <= f3d_base; f4_w <= f3d_w; f4_h <= f3d_cube ? f3d_w : f3d_h; f4_fmt <= f3d_fmt;
                f4_ws <= f3d_ws; f4_wt <= f3d_wt; f4_inc <= f3d_inc; f4_chain <= f3d_chain;
                f4_d <= f3d_d; f4_wr <= f3d_wr; f4_vol <= f3d_vol;
            end else if (t_take) begin
                if (~f4_inc & f4_zl & ~f4_z) f4_z <= 1'b1;
                else begin
                    f4_z <= 1'b0;
                    if (~f4_inc & f4_two & ~f4_k) f4_k <= 1'b1;
                    else begin f4_k <= 1'b0; f4_p <= f4_pn; end
                end
            end
        end
    end
    // the job being offered
    wire [27:0] js = f4_sel[f4_p];
    wire        j_two = js[27], j_lin = js[26];
    wire [3:0]  j_lvl = f4_k ? js[21:18] : js[25:22];
    wire [8:0]  j_lw  = f4_k ? js[8:0] : js[17:9];
    wire        j_first = ~f4_k & ~f4_z, j_last = f4_inc | (f4_zlast & (~j_two | f4_k));

    // ================================================================ T0-T4: taps of one pixel and level (and slice)
    reg t0_v, t1_v, t2_v, tb_v, tc_v, t3_v, t4_v;
    wire t_en;                                                // the tap pipeline advances
    assign t_take = f4_v & t_en;
    // T1: N, M; s' after the wrap mode; u = s' * N
    function automatic [24:0] wrap_sp(input signed [CW-1:0] sf, input [1:0] wr);
        reg [24:0] mm;
        case (wr)
            2'd1: wrap_sp = (sf < 0) ? 25'd0 : (sf > (CW'(1) << 24)) ? (25'd1 << 24) : 25'(sf);
            2'd2: begin mm = 25'(sf); wrap_sp = mm[24] ? ((26'd1 << 25) - 26'(mm)) : mm; end
            default: wrap_sp = {1'b0, sf[23:0]};
        endcase
    endfunction
    // T0: s', t' after the wrap modes; N, M
    reg [24:0] t0_sp, t0_tp; reg [11:0] t0_n, t0_m; reg [1:0] t0_ws, t0_wt; reg t0_lin;
    reg [1:0] t0_p; reg t0_first, t0_last, t0_lastpix, t0_inc, t0_cl; reg [8:0] t0_lw; reg [3:0] t0_lvl; reg [4:0] t0_samp;
    reg [28:0] t0_base; reg [25:0] t0_chain; reg [2:0] t0_face; reg [1:0] t0_fmt;
    reg [24:0] t0_rp; reg [7:0] t0_d; reg [1:0] t0_wr; reg t0_rl, t0_z;   // r', the level's depth, r LINEAR, the slice index
    // T1: u = s' * N, v = t' * M; the clamp bound N*2^24 - 2^23 (CLAMP_TO_EDGE / MIRRORED_REPEAT)
    reg [36:0] t1_u, t1_v2, t1_hu, t1_hv; reg [11:0] t1_n, t1_m; reg [1:0] t1_ws, t1_wt; reg t1_lin;
    reg [1:0] t1_p; reg t1_first, t1_last, t1_lastpix, t1_inc, t1_cl; reg [8:0] t1_lw; reg [3:0] t1_lvl; reg [4:0] t1_samp;
    reg [28:0] t1_base; reg [25:0] t1_chain; reg [2:0] t1_face; reg [1:0] t1_fmt;
    reg [36:0] t1_w, t1_hw; reg [7:0] t1_d; reg [1:0] t1_wr; reg t1_rl, t1_z;
    // T2: clamped u, v -> integer part + top 8 fraction bits
    reg [12:0] t2_ui, t2_vi; reg [7:0] t2_uf, t2_vf; reg [11:0] t2_n, t2_m; reg [1:0] t2_ws, t2_wt;
    reg [1:0] t2_p; reg t2_first, t2_last, t2_lastpix, t2_inc, t2_lin, t2_cl; reg [8:0] t2_lw; reg [3:0] t2_lvl; reg [4:0] t2_samp;
    reg [28:0] t2_base; reg [25:0] t2_chain; reg [2:0] t2_face; reg [1:0] t2_fmt;
    reg [12:0] t2_ri; reg [7:0] t2_rf, t2_d; reg [1:0] t2_wr; reg t2_rl, t2_z;
    // T2b: tap indices + fractions: LINEAR i0 = floor(u - 1/2) = int(u) - !u[23], alpha = {~u[23], u[22:16]}
    reg [11:0] tb_i0, tb_i1, tb_j0, tb_j1; reg [7:0] tb_a, tb_b; reg [11:0] tb_n;
    reg [1:0] tb_p; reg tb_first, tb_last, tb_lastpix, tb_inc, tb_lin, tb_cl; reg [8:0] tb_lw;
    reg [28:0] tb_base; reg [25:0] tb_chain; reg [2:0] tb_face; reg [1:0] tb_fmt;
    reg [7:0] tb_k0, tb_k1, tb_c; reg [8:0] tb_m; reg tb_z; reg [3:0] tb_lvl; reg [4:0] tb_samp;   // tb_m: M of a 3D level (<= 256)
    // T2c: the row counted across the slices, J = k M + j (k = 0 unless 3D); the job's part of the level weight
    reg [15:0] tc_j0, tc_j1; reg [11:0] tc_i0, tc_i1, tc_n; reg [7:0] tc_a, tc_b;
    reg [1:0] tc_p; reg tc_first, tc_last, tc_lastpix, tc_inc, tc_lin, tc_cl;
    (* shreg_extract = "no" *) reg [8:0] tc_lw;               // kept a flop: as an SRL it costs the 3D weight product behind it 0.8 ns (synthesis)
    reg [28:0] tc_base; reg [25:0] tc_chain; reg [2:0] tc_face; reg [1:0] tc_fmt;
    // T3: row bases, the level's base word; the bilinear fractions travel as such (the 4 weights and their level-weight
    // products are formed when the job leaves the job FIFO: it holds the fractions, not 4 x 25-bit weights)
    reg [24:0] t3_r0, t3_r1; reg [11:0] t3_i0, t3_i1; reg [7:0] t3_a, t3_b; reg t3_lin; reg [28:0] t3_fb;
    reg [1:0] t3_p; reg t3_first, t3_last, t3_lastpix, t3_inc, t3_cl; (* shreg_extract = "no" *) reg [8:0] t3_lw; reg [1:0] t3_fmt;
    // T4: word addresses; {lw, lin, b, a} = the job's weight fractions (26 bits)
    reg [28:0] t4_word [0:3]; reg [3:0] t4_half; reg [25:0] t4_fr;
    reg [1:0] t4_p; reg t4_first, t4_last, t4_lastpix, t4_inc, t4_cl; reg [1:0] t4_fmt;
    function automatic [11:0] ldim(input [11:0] d, input [3:0] l);
        ldim = ((d >> l) == 0) ? 12'd1 : (d >> l);
    endfunction
    function automatic [20:0] clampu(input [36:0] u_in, input [36:0] hi, input [1:0] wr);   // {int part, 8 fraction bits}
        reg [36:0] u; u = u_in;
        if (wr == 2'd1 || wr == 2'd2) begin if (u < (37'd1 << 23)) u = 37'd1 << 23; if (u > hi) u = hi; end
        clampu = {u[36:24], u[23:16]};
    endfunction
    // (i0, i1, alpha) from the integer part and the fraction (tools/tmu_ref.py axis_taps)
    function automatic [31:0] axtap(input [12:0] ui, input [7:0] uf, input [11:0] n, input [1:0] wr, input lin);
        reg signed [13:0] i0; reg [11:0] i1; reg [7:0] a;
        if (!lin) begin i0 = 14'(ui); i1 = 12'(ui); a = 0; end
        else begin
            i0 = $signed({1'b0, ui}) - (uf[7] ? 14'sd0 : 14'sd1); a = {~uf[7], uf[6:0]};
            if (wr == 2'd1 || wr == 2'd2) i1 = (12'(i0) + 12'd1 > n - 12'd1) ? (n - 12'd1) : 12'(i0) + 12'd1;
            else begin
                if (i0 < 0) i0 = 14'(n) - 14'sd1;
                i1 = (12'(i0) + 12'd1 == n) ? 12'd0 : 12'(i0) + 12'd1;
            end
        end
        axtap = {12'(i0), i1, a};
    endfunction
    // T4 -> A: a 4-deep FIFO; the tap pipeline advances on its registered count (no combinational stall path)
    reg [2:0] ta_n;
    assign t_en = (ta_n < 3'd4);
    always @(posedge clk) if (t_en) lt_rd <= lt_off[{tb_samp, tb_lvl}];   // level offset: read with T2c, used in T3
    always @(posedge clk) begin
        if (rst_q) begin t0_v <= 0; t1_v <= 0; t2_v <= 0; tb_v <= 0; tc_v <= 0; t3_v <= 0; t4_v <= 0; end
        else if (t_en) begin
            // ---- T0 ----
            t0_v <= f4_v;
            begin : t0b
                reg [11:0] n, m;
                n = ldim(f4_w, j_lvl); m = ldim(f4_h, j_lvl);
                t0_sp <= wrap_sp(f4_s[f4_p], f4_ws); t0_tp <= wrap_sp(f4_t[f4_p], f4_wt); t0_n <= n; t0_m <= m;
                t0_rp <= wrap_sp(f4_r[f4_p], f4_wr); t0_d <= 8'(ldim(12'(f4_d), j_lvl));
            end
            t0_ws <= f4_ws; t0_wt <= f4_wt; t0_lin <= j_lin; t0_p <= f4_p; t0_first <= j_first; t0_last <= j_last;
            t0_lastpix <= f4_plast; t0_cl <= f4_cl; t0_inc <= f4_inc; t0_lw <= j_lw; t0_lvl <= j_lvl; t0_samp <= f4_samp;
            t0_base <= f4_base; t0_chain <= f4_chain; t0_face <= f4_face[f4_p]; t0_fmt <= f4_fmt;
            t0_wr <= f4_wr; t0_rl <= f4_zl; t0_z <= f4_z;
            // ---- T1 ----
            t1_v <= t0_v;
            t1_u <= t0_sp * t0_n; t1_v2 <= t0_tp * t0_m; t1_n <= t0_n; t1_m <= t0_m;
            t1_hu <= ({25'd0, t0_n} << 24) - (37'd1 << 23); t1_hv <= ({25'd0, t0_m} << 24) - (37'd1 << 23);
            t1_ws <= t0_ws; t1_wt <= t0_wt; t1_lin <= t0_lin; t1_p <= t0_p; t1_first <= t0_first; t1_last <= t0_last;
            t1_lastpix <= t0_lastpix; t1_cl <= t0_cl; t1_inc <= t0_inc; t1_lw <= t0_lw; t1_lvl <= t0_lvl; t1_samp <= t0_samp;
            t1_base <= t0_base; t1_chain <= t0_chain; t1_face <= t0_face; t1_fmt <= t0_fmt;
            t1_w <= t0_rp * t0_d; t1_hw <= ({29'd0, t0_d} << 24) - (37'd1 << 23); t1_d <= t0_d; t1_wr <= t0_wr; t1_rl <= t0_rl; t1_z <= t0_z;
            // ---- T2 ----
            t2_v <= t1_v;
            {t2_ui, t2_uf} <= clampu(t1_u, t1_hu, t1_ws); {t2_vi, t2_vf} <= clampu(t1_v2, t1_hv, t1_wt);
            t2_n <= t1_n; t2_m <= t1_m; t2_ws <= t1_ws; t2_wt <= t1_wt;
            t2_lin <= t1_lin; t2_p <= t1_p; t2_first <= t1_first; t2_last <= t1_last; t2_lastpix <= t1_lastpix; t2_cl <= t1_cl;
            t2_inc <= t1_inc; t2_lw <= t1_lw; t2_lvl <= t1_lvl; t2_samp <= t1_samp; t2_base <= t1_base; t2_chain <= t1_chain;
            t2_face <= t1_face; t2_fmt <= t1_fmt;
            {t2_ri, t2_rf} <= clampu(t1_w, t1_hw, t1_wr); t2_d <= t1_d; t2_wr <= t1_wr; t2_rl <= t1_rl; t2_z <= t1_z;
            // ---- T2b ----
            tb_v <= t2_v;
            begin : t2bb
                reg [31:0] xs, ys, zs;
                xs = axtap(t2_ui, t2_uf, t2_n, t2_ws, t2_lin); ys = axtap(t2_vi, t2_vf, t2_m, t2_wt, t2_lin);
                zs = axtap(t2_ri, t2_rf, 12'(t2_d), t2_wr, t2_rl);    // not 3D: depth 1, so k0 = k1 = 0 and, nearest, c = 0
                tb_i0 <= xs[31:20]; tb_i1 <= xs[19:8]; tb_a <= xs[7:0];
                tb_j0 <= ys[31:20]; tb_j1 <= ys[19:8]; tb_b <= ys[7:0];
                tb_k0 <= zs[27:20]; tb_k1 <= zs[15:8]; tb_c <= zs[7:0];
            end
            tb_n <= t2_n; tb_lin <= t2_lin; tb_p <= t2_p; tb_first <= t2_first; tb_last <= t2_last; tb_lastpix <= t2_lastpix; tb_cl <= t2_cl;
            tb_inc <= t2_inc; tb_lw <= t2_lw; tb_base <= t2_base; tb_chain <= t2_chain;
            tb_face <= t2_face; tb_fmt <= t2_fmt; tb_m <= t2_m[8:0]; tb_z <= t2_z; tb_lvl <= t2_lvl; tb_samp <= t2_samp;
            // ---- T2c ----
            tc_v <= tb_v;
            begin : t2cb
                reg [16:0] km; reg [8:0] m;
                km = 17'(tb_z ? tb_k1 : tb_k0) * 17'(tb_m);
                m = 9'((18'(tb_lw) * (18'd256 - 18'(tb_c)) + 18'd128) >> 8);
                tc_j0 <= 16'(km) + 16'(tb_j0); tc_j1 <= 16'(km) + 16'(tb_j1);
                tc_lw <= tb_z ? tb_lw - m : m;
            end
            tc_i0 <= tb_i0; tc_i1 <= tb_i1; tc_n <= tb_n; tc_a <= tb_a; tc_b <= tb_b; tc_lin <= tb_lin;
            tc_p <= tb_p; tc_first <= tb_first; tc_last <= tb_last; tc_lastpix <= tb_lastpix; tc_cl <= tb_cl; tc_inc <= tb_inc;
            tc_base <= tb_base; tc_chain <= tb_chain; tc_face <= tb_face; tc_fmt <= tb_fmt;
            // ---- T3 ----
            t3_v <= tc_v;
            t3_r0 <= tc_j0 * tc_n; t3_r1 <= tc_j1 * tc_n; t3_i0 <= tc_i0; t3_i1 <= tc_i1;
            t3_a <= tc_a; t3_b <= tc_b; t3_lin <= tc_lin;
            t3_fb <= tc_base + 29'(lt_rd) + (tc_face[0] ? 29'(tc_chain) : 29'd0) + (tc_face[1] ? 29'(tc_chain) << 1 : 29'd0)
                                         + (tc_face[2] ? 29'(tc_chain) << 2 : 29'd0);
            t3_p <= tc_p; t3_first <= tc_first; t3_last <= tc_last; t3_lastpix <= tc_lastpix; t3_cl <= tc_cl; t3_inc <= tc_inc; t3_lw <= tc_lw;
            t3_fmt <= tc_fmt;
            // ---- T4 ----
            t4_v <= t3_v;
            for (int k = 0; k < 4; k++) begin : t4b
                reg [24:0] T;
                T = 25'((k >= 2) ? t3_r1 : t3_r0) + 25'(k[0] ? t3_i1 : t3_i0);
                t4_word[k] <= t3_fb + ((t3_fmt == 2'd0) ? 29'(T) : 29'(T >> 1));
                t4_half[k] <= T[0];
            end
            t4_fr <= {t3_lw, t3_lin, t3_b, t3_a};
            t4_p <= t3_p; t4_first <= t3_first; t4_last <= t3_last; t4_lastpix <= t3_lastpix; t4_cl <= t3_cl; t4_inc <= t3_inc; t4_fmt <= t3_fmt;
        end
    end

    // ================================================================ A: burst buffer (16 entries)
    // A job's 4 taps must each name an entry holding (or about to hold) its burst. Missing bursts are allocated one
    // per cycle (round robin, only an entry no pending job references) and looked up in the cache, whose results
    // arrive in lookup order and fill the entries in allocation order. Jobs come from a 4-deep FIFO behind T4 (with
    // the pairwise "same burst" bits of their taps); the reference count of a pushed job lands one cycle later
    // (pend: those entries cannot be reallocated meanwhile).
    reg [28:0] ta_word [0:3][0:3]; reg [3:0] ta_half [0:3]; reg [25:0] ta_fr [0:3]; reg [5:0] ta_eq [0:3];
    reg [1:0] ta_p [0:3], ta_fmt [0:3]; reg ta_first [0:3], ta_last [0:3], ta_lastpix [0:3], ta_inc [0:3], ta_cl [0:3];
    reg [1:0] ta_h, ta_t;
    wire ta_push = t_en & t4_v;
    wire hv = (ta_n != 3'd0);                                 // a job at the head
    wire [1:0] hh = ta_h;
    reg [25:0] bt_tag [0:15]; reg [15:0] bt_v, bd_v; reg [5:0] bt_ref [0:15]; reg [3:0] bt_ap;   // bt_ref <= JF + 1
    // the head job's burst addresses, same-burst bits and increment flag, REGISTERED (it cuts the loop ta_h -> the head
    // mux -> 16 tag compares -> the fast-path check -> a_push -> ta_h): the next head is entry
    // ta_h + a_push, or the job pushed this cycle when the FIFO holds no other ahead of it
    wire [5:0] t4_eq = {t4_word[2][28:3] == t4_word[3][28:3], t4_word[1][28:3] == t4_word[3][28:3],
                        t4_word[1][28:3] == t4_word[2][28:3], t4_word[0][28:3] == t4_word[3][28:3],
                        t4_word[0][28:3] == t4_word[2][28:3], t4_word[0][28:3] == t4_word[1][28:3]};
    wire a_push;
    // both candidates for the next head (no push: entry ta_h, or the job pushed this cycle when the FIFO is empty; push:
    // entry ta_h + 1, or the pushed job when it is the only one left) are formed from registers, and a_push — the end of
    // the 16-tag compare / fast-path chain — picks between them in one 2:1 mux (not a ta_eq[ta_h + a_push]
    // index mux behind the whole decision)
    wire [1:0] hn0 = ta_h, hn1 = ta_h + 2'd1;
    wire hf0 = ta_push & (ta_n == 3'd0), hf1 = ta_push & (ta_n == 3'd1);
    reg [25:0] hb_q [0:3]; reg [5:0] heq_q; reg hinc_q;
    always @(posedge clk) begin
        for (int k = 0; k < 4; k++)
            hb_q[k] <= a_push ? (hf1 ? t4_word[k][28:3] : ta_word[hn1][k][28:3]) : (hf0 ? t4_word[k][28:3] : ta_word[hn0][k][28:3]);
        heq_q  <= a_push ? (hf1 ? t4_eq  : ta_eq[hn1])  : (hf0 ? t4_eq  : ta_eq[hn0]);
        hinc_q <= a_push ? (hf1 ? t4_inc : ta_inc[hn1]) : (hf0 ? t4_inc : ta_inc[hn0]);
    end
    wire [25:0] a_b [0:3];
    generate for (gp = 0; gp < 4; gp++) begin : AB assign a_b[gp] = hb_q[gp]; end endgenerate
    // the tag write buffered one cycle (the head's bursts -> the compares -> the first missing tap -> the tag mux ->
    // the 16 x 26 tag flops' data and enables is too long for one cycle at 120 MHz): the
    // allocation registers {entry, tag} (bwt_*), the tag array takes them next cycle; meanwhile the compares see the new
    // entry through the bypass and ignore its stale stored tag
    reg bwt_v; reg [3:0] bwt_e; reg [25:0] bwt_tag;
    // ---- the head's match matrix, a REGISTER ----
    // hm[k][e]: the head's tap k names the burst entry e holds — its stored tag, or for the entry whose tag write is
    // buffered (bwt_*) the buffered tag (one-hot: the tags are unique). As 64 tag compares on hb_q it would sit
    // INSIDE the A stage's one-cycle decision loop (hb_q -> the compares -> a_one / a_fast -> a_push -> the next-head mux:
    // 12 levels, 6.4–6.7 ns of route). Instead each next-head candidate's matrix
    // is formed a cycle ahead from registers, and a_push / a_alloc only pick and mask at the end (the update below a_push):
    //   the head staying (no push): its own matrix with the entries' valid bits applied;
    //   a new job (ta_word[hn1] after a push, t4_word into an empty FIFO): its bursts against the entries' tags as they
    //   stand (bt_tag, or bwt_tag for the buffered entry) and valid bits (pm1 / pm4);
    //   either way entry al_e's column, when it is allocated this cycle, from the allocated burst hb_q[a_mk] — staying: the
    //   head's same-burst bits; a new job: its taps against all four head bursts, compared ahead (x1 / x4) — which is
    //   next cycle's buffered write; a reset / invalidate leaves only that column (as bwt_v <= a_alloc & ~rst_q does).
    // Exact: the same matches as the compares on hb_q (simulation check below); +128 compares off the loop.
    reg [15:0] hm [0:3];
    reg [3:0] a_hit; reg [3:0] a_e [0:3];
    always @* begin
        for (int k = 0; k < 4; k++) begin
            a_hit[k] = |hm[k]; a_e[k] = 4'd0;
            for (int e = 0; e < 16; e++) if (hm[k][e]) a_e[k] = a_e[k] | 4'(e);   // one-hot -> its index
        end
    end
    // the candidates' bursts (registers): the job after the head, the job entering an empty / one-entry FIFO
    wire [25:0] cw1 [0:3], cw4 [0:3];
    generate for (gp = 0; gp < 4; gp++) begin : HMCW
        assign cw1[gp] = ta_word[hn1][gp][28:3]; assign cw4[gp] = t4_word[gp][28:3];
    end endgenerate
    reg [15:0] pm1 [0:3], pm4 [0:3];                          // a new job's matches against the entries as they stand
    reg [3:0]  x1 [0:3], x4 [0:3];                            // x?[k][j]: its tap k's burst = the head's tap j's
    always @* begin
        for (int k = 0; k < 4; k++) begin
            for (int e = 0; e < 16; e++) begin
                pm1[k][e] = bt_v[e] && ((bwt_v && bwt_e == 4'(e)) ? (bwt_tag == cw1[k]) : (bt_tag[e] == cw1[k]));
                pm4[k][e] = bt_v[e] && ((bwt_v && bwt_e == 4'(e)) ? (bwt_tag == cw4[k]) : (bt_tag[e] == cw4[k]));
            end
            for (int j = 0; j < 4; j++) begin x1[k][j] = (hb_q[j] == cw1[k]); x4[k][j] = (hb_q[j] == cw4[k]); end
        end
    end
    wire h_inc = hinc_q;
    wire a_need = hv & ~h_inc & ~&a_hit;                      // a burst still missing
    wire [1:0] a_mk = ~a_hit[0] ? 2'd0 : ~a_hit[1] ? 2'd1 : ~a_hit[2] ? 2'd2 : 2'd3;   // first missing tap
    function automatic same(input [5:0] eq, input [1:0] a, input [1:0] b);   // taps a, b hit the same burst
        case ({a, b})
            4'b0000, 4'b0101, 4'b1010, 4'b1111: same = 1'b1;
            4'b0001, 4'b0100: same = eq[0];  4'b0010, 4'b1000: same = eq[1];  4'b0011, 4'b1100: same = eq[2];
            4'b0110, 4'b1001: same = eq[3];  4'b0111, 4'b1101: same = eq[4];  default: same = eq[5];
        endcase
    endfunction
    reg [4:0] lq_n;                                           // lookups queued, not yet accepted by the cache
    reg       lq_room;                                        // lq_n < 16, a register of its next value (keeps the 5-bit
                                                              // compare's carry chain out of the A stage's decision loop)
    reg [25:0] lq_b [0:15]; reg [3:0] lq_h, lq_t;
    // last cycle's pushed job's entries, one bit per entry, REGISTERED: the free-entry search starts from flops, not
    // from a push flag and four compares of the pushed job's entries
    reg [15:0] pend;
    // an entry may be reused when no queued job references it and its last fill has landed (a pending fill would
    // otherwise arrive under the new tag). The victim is the first such entry at or after bt_ap (ring order, so the
    // oldest free one), not entry bt_ap itself: waiting on bt_ap blocks a miss-heavy frame's allocation in 25 % of
    // all cycles with free entries elsewhere — an entry stays referenced as long as later jobs keep hitting it
    wire [15:0] bt_free;
    generate for (gp = 0; gp < 16; gp++) begin : FREE
        assign bt_free[gp] = (bt_ref[gp] == 0) & ~pend[gp] & (bd_v[gp] | ~bt_v[gp]);
    end endgenerate
    wire [31:0] bt_rot2 = {bt_free, bt_free} >> bt_ap;        // bit 0 = entry bt_ap
    reg  [3:0] ff_i, ff2_i; reg ff_v, ff2_v;
    always @* begin
        ff_v = 1'b0; ff_i = 4'd0;
        for (int e = 15; e >= 0; e--) if (bt_rot2[e]) begin ff_v = 1'b1; ff_i = 4'(e); end
        ff2_v = 1'b0; ff2_i = 4'd0;                           // the free entry after the first
        for (int e = 15; e >= 0; e--) if (bt_rot2[e] && 4'(e) != ff_i) begin ff2_v = 1'b1; ff2_i = 4'(e); end
    end
    // the victim is chosen a cycle AHEAD (bt_ref / bt_v -> bt_free -> the rotate + first-one search -> al_e -> the
    // fast-path check -> a_push -> the head registers is too long for one cycle at 120 MHz):
    // the first and the second free entry from bt_ap are found this cycle and registered; next cycle's
    // candidate is the first (no allocation now) or the second (one now), and it is used if it is still free — an entry
    // can only have stopped being free through pend (a hit of the job pushed now), then the allocation waits a cycle
    reg  [3:0] al_e; reg alc_v;                               // the entry allocated this cycle (if a_alloc)
    wire a_alloc = a_need & alc_v & bt_free[al_e] & lq_room;
    always @(posedge clk) begin
        if (rst_q) begin alc_v <= 1'b0; al_e <= 4'd0; end
        else begin alc_v <= a_alloc ? ff2_v : ff_v; al_e <= bt_ap + (a_alloc ? ff2_i : ff_i); end
    end
    // job FIFO to the filter (32: 16 fill up behind jobs waiting for DDR data)
    localparam int JF = 32, JFW = $clog2(JF);
    reg [JFW:0] jf_h, jf_t, jf_r;                             // jf_r: tail including the job being written (jw_v)
    wire [JFW-1:0] jt = jf_t[JFW-1:0];
    reg  jf_full;                                             // (jf_r - jf_h) >= JF, a register of its next value (keeps the
                                                              // 6-bit subtract and compare out of the A stage's decision loop)
    // fast path: the job lacks one burst only (every missing tap wants a_b[a_mk]) and no hit is on the entry being
    // reallocated -> allocate and push in the same cycle, the missing taps naming the new entry
    reg a_one;
    always @* begin                                           // a hit on the entry being reallocated: the raw match vector indexed
        a_one = 1'b1;                                         // by the registered al_e (no 16 : 1 encode then a 4-bit
        for (int k = 0; k < 4; k++)                           // compare, and no one-hot register: its decode is a long path)
            if (a_hit[k] ? hm[k][al_e] : ~same(heq_q, 2'(k), a_mk)) a_one = 1'b0;
    end
    wire a_fast = a_need & a_one & a_alloc & ~jf_full;
    reg [3:0] a_ej [0:3];                                     // the pushed job's entries
    always @* for (int k = 0; k < 4; k++) a_ej[k] = a_hit[k] ? a_e[k] : al_e;
    assign a_push = hv & (~a_need | a_fast) & ~jf_full;
    // the next head's match matrix (see hm): the candidates as the head mux picks them, entry al_e's column from this
    // cycle's allocation
    wire hm_keep = ~(rst_q | cache_inval);
    always @(posedge clk) begin
        for (int k = 0; k < 4; k++) for (int e = 0; e < 16; e++) begin : HMU
            reg ac, n_st, n_1, n_4;
            ac   = a_alloc & ~rst_q & (al_e == 4'(e));            // entry e takes the burst hb_q[a_mk] now
            n_st = ac ? same(heq_q, 2'(k), a_mk) : (hm_keep & bt_v[e] & hm[k][e]);
            n_1  = ac ? x1[k][a_mk] : (hm_keep & pm1[k][e]);
            n_4  = ac ? x4[k][a_mk] : (hm_keep & pm4[k][e]);
            hm[k][e] <= a_push ? (hf1 ? n_4 : n_1) : (hf0 ? n_4 : n_st);
        end
    end
    // synthesis translate_off
    // the matrix equals the direct compares (the head's bursts against the tags, the buffered write bypassed)
    always @(posedge clk) if (!rst_q && hv) for (int k = 0; k < 4; k++) begin : HMCHK
        reg [15:0] r;
        for (int e = 0; e < 16; e++)
            r[e] = (bwt_v && bwt_e == 4'(e)) ? (bwt_tag == hb_q[k]) : (bt_v[e] && bt_tag[e] == hb_q[k]);
        if (hm[k] !== r) $fatal(1, "tmu24: the head's match row %0d is %h, the compares give %h", k, hm[k], r);
        if (!$onehot0(hm[k])) $fatal(1, "tmu24: two entries hold tap %0d's burst (%h)", k, hm[k]);
    end
    // synthesis translate_on
    // one packed LUTRAM word per job (one write at jt, one read at the head jh): {inc, lastpix, last, first, fmt[1:0],
    // p[1:0], fr[25:0], hf[3:0], l[3]..l[0] (3 bits each), e[3]..e[0] (4 bits each)} = 66 bits. As separate flop arrays
    // the FIFO would be flops and read muxes: ≈ 1 k LUT more at 32 entries.
    (* ram_style = "distributed" *) reg [66:0] jf_q [0:JF-1];
    // cache
    wire lu_ready, lu_done; wire [255:0] lu_data;
    wire lu_valid = (lq_n != 0);
    tex_cache #(.SETS(TC_SETS), .HASH(TC_HASH), .PF(TC_PF)) U_CACHE(.clk, .rst(rst_q), .inval(cache_inval), .lu_valid, .lu_burst(lq_b[lq_h]), .lu_ready, .lu_done, .lu_data,
        .mig_en(tmu_en), .mig_addr(tmu_addr), .mig_rdy(tmu_rdy), .mig_rd_valid(tmu_rd_valid), .mig_rd_data(tmu_rd_data),
        .n_hit(st_hits), .n_miss(st_misses));
    // the events are registered first: a_push is the end of the A stage's decision chain, and a 32-bit increment behind
    // it would be the unit's worst path
    reg st_job_q, st_ast_q;
    always @(posedge clk) begin
        st_job_q <= t_take; st_ast_q <= hv & ~a_push;
        if (rst_q) begin st_jobs <= 32'd0; st_astall <= 32'd0; end
        else begin st_jobs <= st_jobs + 32'(st_job_q); st_astall <= st_astall + 32'(st_ast_q); end
    end
    reg [3:0] fo_e [0:15]; reg [3:0] fo_h, fo_t;              // entries awaiting data, in lookup order
    reg fl_rel; reg [3:0] fl_rel_e [0:3]; reg [3:0] fl_rel_m;   // filter releases its job's entries
    // the tag write in a block of its own, without the reset / invalidate term (~(rst_q | cache_inval) in the
    // 16 x 26 tag flops' clock enables comes 8.5 ns from the reset replica;
    // a tag written during an invalidate is dead — bt_v is cleared)
    always @(posedge clk) begin bwt_v <= a_alloc & ~rst_q; bwt_e <= al_e; bwt_tag <= a_b[a_mk]; end
    always @(posedge clk) if (bwt_v) bt_tag[bwt_e] <= bwt_tag;
    always @(posedge clk) begin
        // T4 -> A FIFO
        if (rst_q) begin ta_h <= 0; ta_t <= 0; ta_n <= 0; end
        else begin
            if (ta_push) begin
                for (int k = 0; k < 4; k++) ta_word[ta_t][k] <= t4_word[k];
                ta_fr[ta_t] <= t4_fr; ta_eq[ta_t] <= t4_eq;
                ta_half[ta_t] <= t4_half; ta_p[ta_t] <= t4_p; ta_fmt[ta_t] <= t4_fmt; ta_first[ta_t] <= t4_first;
                ta_last[ta_t] <= t4_last; ta_lastpix[ta_t] <= t4_lastpix; ta_inc[ta_t] <= t4_inc; ta_cl[ta_t] <= t4_cl;
                ta_t <= ta_t + 2'd1;
            end
            if (a_push) ta_h <= ta_h + 2'd1;
            ta_n <= ta_n + 3'(ta_push) - 3'(a_push);
        end
        if (rst_q | cache_inval) begin bt_v <= 0; bd_v <= 0; bt_ap <= 0; lq_n <= 0; lq_room <= 1'b1; lq_h <= 0; lq_t <= 0; fo_h <= 0; fo_t <= 0;
                                     pend <= '0; for (int e = 0; e < 16; e++) bt_ref[e] <= 0; end
        else begin
            lq_room <= (lq_n + 5'(a_alloc) - 5'(lu_valid & lu_ready)) < 5'd16;
            // allocate one missing burst
            if (a_alloc) begin                                    // (the tag itself: the block below, without the reset term)
                bt_v[al_e] <= 1'b1; bd_v[al_e] <= 1'b0; bt_ap <= al_e + 4'd1;
                lq_b[lq_t] <= a_b[a_mk]; lq_t <= lq_t + 4'd1; fo_e[fo_t] <= al_e; fo_t <= fo_t + 4'd1;
            end
            if (lu_valid & lu_ready) lq_h <= lq_h + 4'd1;
            lq_n <= lq_n + 5'(a_alloc) - 5'(lu_valid & lu_ready);
            if (lu_done) begin bd_v[fo_e[fo_h]] <= 1'b1; fo_h <= fo_h + 4'd1; end
            // reference counts: +1 per distinct entry of last cycle's pushed job, -1 per entry the filter released
            for (int e = 0; e < 16; e++)
                pend[e] <= a_push & ~h_inc & ((a_ej[0] == e) | (a_ej[1] == e) | (a_ej[2] == e) | (a_ej[3] == e));
            for (int e = 0; e < 16; e++) begin : rc
                reg dec;
                dec = fl_rel & (((fl_rel_e[0] == e) & fl_rel_m[0]) | ((fl_rel_e[1] == e) & fl_rel_m[1]) |
                                ((fl_rel_e[2] == e) & fl_rel_m[2]) | ((fl_rel_e[3] == e) & fl_rel_m[3]));
                bt_ref[e] <= bt_ref[e] + 6'(pend[e]) - 6'(dec);
            end
        end
    end
    // entry -> burst buffer in block RAM: 16 x 256-bit entries, written on lu_done for the entry
    // fo_e[fo_h], 4 read copies (one per tap). A tap's entry is read when its job leaves the job FIFO (fl_rdy: the
    // entry's data has landed), the 32-bit word selected and decoded one cycle later (FL0 -> FL1). An entry is
    // refilled only after the filter released it (bt_ref), so a read never meets a write to the same entry.
    genvar gc;
    wire jf_ne = (jf_t != jf_h);
    wire [JFW-1:0] jh = jf_h[JFW-1:0];
    wire [66:0] jq = jf_q[jh];                                // the head job
    wire [3:0] jq_e [0:3]; wire [2:0] jq_l [0:3];
    generate for (gp = 0; gp < 4; gp++) begin : JQ
        assign jq_e[gp] = jq[gp*4 +: 4]; assign jq_l[gp] = jq[16 + gp*3 +: 3];
    end endgenerate
    wire [3:0] jq_hf = jq[31:28]; wire [25:0] jq_fr = jq[57:32]; wire [1:0] jq_p = jq[59:58], jq_fmt = jq[61:60];
    wire jq_first = jq[62], jq_last = jq[63], jq_lastpix = jq[64], jq_inc = jq[65], jq_cl = jq[66];
    // a quad's last job (the one that completes its result) leaves at least 4 cycles after the previous quad's of the
    // same writer class: the consumer writes a result over 4 cycles of that writer (shader_core_fs2's staging), and
    // everything behind this pop has a fixed latency. With every pixel sampled a quad has >= 4 jobs and this never
    // waits; with a sample mask it paces quads of fewer than 4 jobs.
    reg [1:0] lp_c [0:1]; reg [1:0] lp_n;                     // per class: cycles to go; lp_n = "not zero" (a flop each)
    wire jq_res = jq_last & jq_lastpix;
    wire fl_rdy = jf_ne & (jq_inc | (bd_v[jq_e[0]] & bd_v[jq_e[1]] & bd_v[jq_e[2]] & bd_v[jq_e[3]])) & ~(jq_res & lp_n[jq_cl]);
    reg [255:0] bq [0:3];
    generate for (gc = 0; gc < 4; gc++) begin : RFC
        (* ram_style="block" *) reg [255:0] bm [0:15];
        always @(posedge clk) begin
            if (lu_done) bm[fo_e[fo_h]] <= lu_data;
            if (fl_rdy) bq[gc] <= bm[jq_e[gc]];
        end
    end endgenerate
    // push the job (entries per tap: a distinct entry is referenced once in the counts). The job is written into the
    // FIFO one cycle after the push from a registered copy (jw_*), so the push decision (head mux, 16 tag compares,
    // the fast-path check) does not drive the FIFO's write enables (13 logic levels into ~300 LUTRAM
    // write enables); jf_r counts it at once for jf_full.
    reg jw_v; reg [3:0] jw_e [0:3]; reg [2:0] jw_l [0:3]; reg [25:0] jw_fr; reg [3:0] jw_hf; reg [1:0] jw_p, jw_fmt;
    reg jw_first, jw_last, jw_lastpix, jw_inc, jw_cl;
    always @(posedge clk) begin
        if (rst_q) begin jf_t <= 0; jf_r <= 0; jw_v <= 1'b0; jf_full <= 1'b0; end
        else begin
            jw_v <= a_push;
            if (a_push) jf_r <= jf_r + 1'b1;
            jf_full <= ((jf_r + (JFW+1)'(a_push)) - (jf_h + (JFW+1)'(fl_rdy))) >= (JFW+1)'(JF);   // next cycle's fill
            if (jw_v) begin
                jf_q[jt] <= {jw_cl, jw_inc, jw_lastpix, jw_last, jw_first, jw_fmt, jw_p, jw_fr, jw_hf,
                             jw_l[3], jw_l[2], jw_l[1], jw_l[0], jw_e[3], jw_e[2], jw_e[1], jw_e[0]};
                jf_t <= jf_t + 1'b1;
            end
        end
        for (int k = 0; k < 4; k++) begin jw_e[k] <= a_ej[k]; jw_l[k] <= ta_word[hh][k][2:0]; end
        jw_fr <= ta_fr[hh];
        jw_hf <= ta_half[hh]; jw_p <= ta_p[hh]; jw_fmt <= ta_fmt[hh]; jw_first <= ta_first[hh];
        jw_last <= ta_last[hh]; jw_lastpix <= ta_lastpix[hh]; jw_inc <= h_inc; jw_cl <= ta_cl[hh];
    end
    // ================================================================ FL: filter
    // the distinct entries of the head job (each released once)
    wire [3:0] fl_dist = {(jq_e[3] != jq_e[0]) & (jq_e[3] != jq_e[1]) & (jq_e[3] != jq_e[2]),
                          (jq_e[2] != jq_e[0]) & (jq_e[2] != jq_e[1]),
                          (jq_e[1] != jq_e[0]), 1'b1};
    // decode the 4 texels -> channels {R, G, B, A}, 8-bit fields
    function automatic [31:0] dec(input [31:0] wd, input hf, input [1:0] fmt);
        reg [15:0] h; h = hf ? wd[31:16] : wd[15:0];
        case (fmt)
            2'd0:    dec = {wd[7:0], wd[15:8], wd[23:16], wd[31:24]};                       // A B G R
            2'd1:    dec = {4'd0, h[3:0], 4'd0, h[7:4], 4'd0, h[11:8], 4'd0, h[15:12]};
            2'd2:    dec = {7'd0, h[0], 3'd0, h[5:1], 3'd0, h[10:6], 3'd0, h[15:11]};
            default: dec = {8'd0, 3'd0, h[4:0], 2'd0, h[10:5], 3'd0, h[15:11]};
        endcase
    endfunction
    reg fl0_v; reg [2:0] fl0_l [0:3]; reg [16:0] fl0_bw [0:3]; reg [8:0] fl0_lw; reg [3:0] fl0_hf; reg [1:0] fl0_p, fl0_fmt; reg fl0_first, fl0_last, fl0_lastpix, fl0_inc;
    reg fl1_v; reg [31:0] fl1_c [0:3]; reg [24:0] fl1_w [0:3]; reg [1:0] fl1_p, fl1_fmt; reg fl1_first, fl1_last, fl1_lastpix, fl1_inc;
    reg fl2_v; reg [32:0] fl2_pr [0:3][0:3]; reg [1:0] fl2_p, fl2_fmt; reg fl2_first, fl2_last, fl2_lastpix, fl2_inc;
    reg [31:0] acc [0:3];
    reg cv0_v; reg [31:0] cv0_P [0:3]; reg [1:0] cv0_p, cv0_fmt; reg cv0_lastpix, cv0_inc;
    always @(posedge clk) begin
        if (rst_q) begin jf_h <= 0; fl0_v <= 0; fl1_v <= 0; fl2_v <= 0; cv0_v <= 0; fl_rel <= 0; lp_c[0] <= 2'd0; lp_c[1] <= 2'd0; lp_n <= 2'b00; end
        else begin
            for (int k = 0; k < 2; k++) begin
                if (fl_rdy & jq_res & (jq_cl == 1'(k))) begin lp_c[k] <= 2'd3; lp_n[k] <= 1'b1; end
                else if (lp_n[k]) begin lp_c[k] <= lp_c[k] - 2'd1; lp_n[k] <= (lp_c[k] != 2'd1); end
            end
            // FL0: the job leaves the FIFO (the burst buffer reads its 4 entries), release the entries; the bilinear
            // weights from the job's fractions
            fl0_v <= fl_rdy; fl_rel <= fl_rdy & ~jq_inc;
            if (fl_rdy) begin : f0
                reg [25:0] fr; reg [7:0] a, b;
                fr = jq_fr; a = fr[7:0]; b = fr[15:8];
                jf_h <= jf_h + 1'b1;
                for (int k = 0; k < 4; k++) begin fl0_l[k] <= jq_l[k]; fl_rel_e[k] <= jq_e[k]; end
                if (fr[16]) begin
                    fl0_bw[0] <= (9'd256 - a) * (9'd256 - b); fl0_bw[1] <= {1'b0, a} * (9'd256 - b);
                    fl0_bw[2] <= (9'd256 - a) * {1'b0, b};    fl0_bw[3] <= {1'b0, a} * {1'b0, b};
                end else begin fl0_bw[0] <= 17'd65536; fl0_bw[1] <= 0; fl0_bw[2] <= 0; fl0_bw[3] <= 0; end
                fl0_lw <= fr[25:17];
                fl_rel_m <= fl_dist;
                fl0_hf <= jq_hf; fl0_p <= jq_p; fl0_fmt <= jq_fmt; fl0_first <= jq_first; fl0_last <= jq_last;
                fl0_lastpix <= jq_lastpix; fl0_inc <= jq_inc;
            end
            // FL1: the taps' words out of the read entries, decoded; the weights x the level weight
            fl1_v <= fl0_v;
            if (fl0_v) begin
                for (int k = 0; k < 4; k++) begin fl1_c[k] <= dec(bq[k][fl0_l[k]*32 +: 32], fl0_hf[k], fl0_fmt); fl1_w[k] <= fl0_bw[k] * fl0_lw; end
                fl1_p <= fl0_p; fl1_fmt <= fl0_fmt; fl1_first <= fl0_first; fl1_last <= fl0_last;
                fl1_lastpix <= fl0_lastpix; fl1_inc <= fl0_inc;
            end
            // FL2: products (tap x channel)
            fl2_v <= fl1_v;
            for (int k = 0; k < 4; k++) for (int ch = 0; ch < 4; ch++) fl2_pr[k][ch] <= fl1_c[k][ch*8 +: 8] * fl1_w[k];
            fl2_p <= fl1_p; fl2_fmt <= fl1_fmt; fl2_first <= fl1_first; fl2_last <= fl1_last; fl2_lastpix <= fl1_lastpix; fl2_inc <= fl1_inc;
            // FL3: sum the taps, accumulate the levels; the pixel's last level -> conversion
            cv0_v <= fl2_v & fl2_last;
            if (fl2_v) begin
                for (int ch = 0; ch < 4; ch++) begin : sm
                    reg [31:0] s;
                    s = 32'(fl2_pr[0][ch]) + 32'(fl2_pr[1][ch]) + 32'(fl2_pr[2][ch]) + 32'(fl2_pr[3][ch]);
                    if (fl2_inc) s = 0;
                    acc[ch] <= fl2_first ? s : acc[ch] + s;
                    cv0_P[ch] <= fl2_first ? s : acc[ch] + s;
                end
                cv0_p <= fl2_p; cv0_fmt <= fl2_fmt; cv0_lastpix <= fl2_lastpix; cv0_inc <= fl2_inc;
            end
        end
    end

    // ================================================================ CV: P / ((2^n - 1) 2^24) -> float (4 channels)
    // n per format and channel; KD = ceil(2^(32+s) / (2^n - 1)) (s = n, 0 for n = 1): Q = Pn * KD over-estimates by
    // < 2^32, less than a non-exact quotient's distance to any rounding boundary, so RNE of Q is correctly rounded; an
    // exact quotient (2^n - 1 divides P) has zero bits [31+s:32], and its error term is dropped (ties to even)
    function automatic [3:0] chn(input [1:0] fmt, input [1:0] ch);
        case (fmt)
            2'd0: chn = 4'd8;
            2'd1: chn = 4'd4;
            2'd2: chn = (ch == 2'd3) ? 4'd1 : 4'd5;
            default: chn = (ch == 2'd1) ? 4'd6 : (ch == 2'd3) ? 4'd0 : 4'd5;   // RGB565: alpha constant 1.0
        endcase
    endfunction
    function automatic [32:0] kd(input [3:0] n);
        case (n)
            4'd8: kd = 33'd4311810306; 4'd4: kd = 33'd4581298450; 4'd5: kd = 33'd4433514629; 4'd6: kd = 33'd4363141381;
            default: kd = 33'd4294967296;
        endcase
    endfunction
    reg cv1_v; reg [31:0] cv1_Pn [0:3]; reg [5:0] cv1_msb [0:3]; reg [3:0] cv1_n [0:3]; reg [3:0] cv1_z;
    reg [1:0] cv1_p; reg cv1_lastpix, cv1_inc;
    reg cv2_v; reg [64:0] cv2_Q [0:3]; reg [5:0] cv2_msb [0:3]; reg [3:0] cv2_n [0:3]; reg [3:0] cv2_z; reg [1:0] cv2_p; reg cv2_lastpix, cv2_inc;
    // cv3: exact-quotient mask, leading bit, mantissa + guard + sticky; cv4: round to nearest even, exponent, pack
    reg cv3_v; reg [M+1:0] cv3_mant [0:3]; reg [3:0] cv3_g, cv3_st, cv3_hi; reg [5:0] cv3_msb [0:3]; reg [3:0] cv3_n [0:3];
    reg [3:0] cv3_z; reg [1:0] cv3_p; reg cv3_lastpix, cv3_inc;
    reg [16*W-1:0] obuf = '0;                                 // cleared with each result (not by the reset: an unsampled pixel's
                                                              // value is never used; the initial value keeps X out of a bench)
    always @(posedge clk) begin
        if (rst_q) begin cv1_v <= 0; cv2_v <= 0; cv3_v <= 0; res_valid <= 0; end
        else begin
            cv1_v <= cv0_v;
            for (int ch = 0; ch < 4; ch++) begin : c1
                reg [5:0] mb; mb = msb64(64'(cv0_P[ch]));
                cv1_msb[ch] <= mb; cv1_Pn[ch] <= cv0_P[ch] << (6'd31 - mb); cv1_n[ch] <= chn(cv0_fmt, 2'(ch));
                cv1_z[ch] <= (cv0_P[ch] == 0);
            end
            cv1_p <= cv0_p; cv1_lastpix <= cv0_lastpix; cv1_inc <= cv0_inc;
            cv2_v <= cv1_v;
            for (int ch = 0; ch < 4; ch++) begin cv2_Q[ch] <= 65'(cv1_Pn[ch]) * 65'(kd(cv1_n[ch])); cv2_msb[ch] <= cv1_msb[ch]; cv2_n[ch] <= cv1_n[ch]; end
            cv2_z <= cv1_z; cv2_p <= cv1_p; cv2_lastpix <= cv1_lastpix; cv2_inc <= cv1_inc;
            // ---- cv3 ----
            cv3_v <= cv2_v;
            for (int ch = 0; ch < 4; ch++) begin : c3
                reg [64:0] q; reg [3:0] s; reg hi;
                q = cv2_Q[ch];
                s = (cv2_n[ch] == 4'd1) ? 4'd0 : cv2_n[ch];
                if (((q >> 32) & ((65'd1 << s) - 65'd1)) == 0) q = (q >> (7'd32 + 7'(s))) << (7'd32 + 7'(s));   // exact
                hi = q[64];                                   // leading bit at 64 (else 63)
                cv3_hi[ch] <= hi;
                cv3_mant[ch] <= hi ? (M+2)'(q >> (64 - M)) : (M+2)'(q >> (63 - M));
                cv3_g[ch]  <= hi ? q[63 - M] : q[62 - M];
                cv3_st[ch] <= hi ? |q[62-M:0] : |q[61-M:0];
                cv3_msb[ch] <= cv2_msb[ch]; cv3_n[ch] <= cv2_n[ch];
            end
            cv3_z <= cv2_z; cv3_p <= cv2_p; cv3_lastpix <= cv2_lastpix; cv3_inc <= cv2_inc;
            // ---- cv4 ----
            res_valid <= 1'b0;
            if (cv3_v) begin : c4
                reg [W*4-1:0] px;
                for (int ch = 0; ch < 4; ch++) begin : rn
                    reg [M+1:0] mant; reg [6:0] lq; integer eb;
                    mant = cv3_mant[ch]; lq = cv3_hi[ch] ? 7'd64 : 7'd63;
                    if (cv3_g[ch] & (cv3_st[ch] | mant[0])) mant = mant + 1'b1;
                    if (mant[M+1]) begin mant = mant >> 1; lq = lq + 7'd1; end
                    eb = int'(lq) + int'(cv3_msb[ch]) - 31 - 24 - 32 - ((cv3_n[ch] == 4'd1) ? 0 : int'(cv3_n[ch])) + BIAS;
                    if (cv3_inc)                px[ch*W +: W] = (ch == 3) ? ONE : '0;
                    else if (cv3_n[ch] == 4'd0) px[ch*W +: W] = ONE;
                    else if (cv3_z[ch] || eb <= 0) px[ch*W +: W] = '0;
                    else                        px[ch*W +: W] = {1'b0, E'(eb), mant[M-1:0]};
                end
                obuf[cv3_p*4*W +: 4*W] <= px;
                if (cv3_lastpix) begin
                    res_valid <= 1'b1;
                    res_texels <= obuf;
                    res_texels[cv3_p*4*W +: 4*W] <= px;
                    obuf <= '0;                               // a pixel the next quad does not sample returns zeros
                end
            end
        end
    end
    // some quad is inside the unit (profiling / benches)
    wire busy = rq_ne | f1_v | wf_ne | f3a_v | f3b_v | f3c_v | f3d2_v | f3d3_v | f3e_v | f3h_v | f3f_v | f4_v | t0_v | t1_v | t2_v | tb_v | tc_v | t3_v | t4_v | hv | jw_v | jf_ne |
                fl0_v | fl1_v | fl2_v | cv0_v | cv1_v | cv2_v | cv3_v;
    assign st_ev = {jf_ne & ~fl_rdy, busy};

    // ================================================================ request queue, F1, F2, level table
    always @(posedge clk) begin
        if (rst_q) begin rq_h <= 0; rq_t <= 0; f1_v <= 0; f2_op <= 0; wf_h <= 0; wf_t <= 0; wf_r <= 0; wf_cp <= 0; fb_va <= 0; fb_vb <= 0;
                       lt_ok <= 0; lt_busy <= 0; for (int i = 0; i < WF; i++) wf_rc[i] <= 3'd0; end
        else begin
            if (req_valid & req_ready) begin
                rq_s[rq_t[3:0]] <= req_sampler; rq_m[rq_t[3:0]] <= req_mode;
                rq_mk[rq_t[3:0]] <= {req_cls, (req_mask == 4'd0) ? 4'd1 : req_mask};
                for (int p = 0; p < 4; p++) begin
                    rq_c[rq_t[3:0]][(p*4+0)*W +: W] <= req_uv[p*2*W +: W]; rq_c[rq_t[3:0]][(p*4+1)*W +: W] <= req_uv[p*2*W+W +: W];
                    rq_c[rq_t[3:0]][(p*4+2)*W +: W] <= req_zw[p*2*W +: W]; rq_c[rq_t[3:0]][(p*4+3)*W +: W] <= req_zw[p*2*W+W +: W];
                end
                rq_t <= rq_t + 5'd1;
            end
            // F1
            if (f1_free & rq_ne & ~lt_busy) begin
                f1_v <= 1'b1; f1_s <= rq_s[rq_h[3:0]]; f1_m <= rq_m[rq_h[3:0]]; f1_c <= rq_c[rq_h[3:0]];   // the level table is per {bank, sampler}
                f1_mk <= rq_mk[rq_h[3:0]];                     // (with the writer class)
                for (int p = 0; p < 4; p++)
                    f1_face[p] <= cface(rq_c[rq_h[3:0]][(p*4+0)*W +: W], rq_c[rq_h[3:0]][(p*4+1)*W +: W], rq_c[rq_h[3:0]][(p*4+2)*W +: W]);
                f1_base <= desc_base; f1_w <= (desc_w == 0) ? 12'd1 : desc_w; f1_h <= (desc_h == 0) ? 12'd1 : desc_h;
                f1_fmt <= desc_fmt; f1_lv <= desc_levels; f1_filt <= desc_filt; f1_ws <= desc_wrap_s; f1_wt <= desc_wrap_t;
                f1_cube <= desc_cube; f1_inc <= desc_incomplete | (desc_w == 0) | (desc_h == 0);
                f1_d <= (VOL3D && desc_depth != 0) ? desc_depth : 8'd1; f1_wr <= desc_wrap_r; f1_vol <= VOL3D & desc_vol;
                rq_h <= rq_h + 5'd1;
            end else if (f2_take) f1_v <= 1'b0;
            // level table: start for F1's sampler when its key differs; pipelined: level dims (lt_l), w h (lt_l - 1),
            // w h d (lt_l - 2), the offset accumulation (lt_l - 3); 16 cycles for the 12 levels of a texture <= 2048
            // (MAX_TEXTURE_SIZE 2048: levels 1..12, fb_cmd_b CREATE_IMAGE), entries 13..15 of a sampler unused
            if (~lt_busy & f1_v & ~f1_ltok) begin
                lt_busy <= 1'b1; lt_s <= f1_s; lt_l <= 0; lt_o <= 0; lt_w <= f1_w; lt_h <= f1_h; lt_d <= f1_d; lt_lv <= f1_lv; lt_b16 <= (f1_fmt != 2'd0);
            end else if (lt_busy) begin : ltc
                reg [3:0] la;
                lt_wl <= ldim(lt_w, lt_l[3:0]); lt_hl <= ldim(lt_h, lt_l[3:0]); lt_dl <= 8'(ldim(12'(lt_d), lt_l[3:0]));
                lt_sz <= lt_wl * lt_hl; lt_dl2 <= lt_dl;
                lt_sv <= lt_sz * lt_dl2;
                if (lt_l >= 5'd3) begin
                    la = 4'(lt_l - 5'd3);
                    lt_off[{lt_s, la}] <= lt_o;
                    if (la < lt_lv) lt_o <= lt_o + (lt_b16 ? ((lt_sv + 26'd1) >> 1) : lt_sv);
                    // the face stride = the chain rounded up to a 32-byte burst (8 words): img_geom lays the faces out
                    // so (CREATE_IMAGE / WRITE_IMAGE); the unrounded sum would sample 8- and 9-level 256^2 cube maps
                    // from the wrong face offset
                    if (la == lt_lv) lt_chain[lt_s] <= (lt_o + 26'd7) & ~26'd7;
                end
                lt_l <= lt_l + 5'd1;
                if (lt_l == 5'd15) begin
                    lt_busy <= 1'b0; lt_ok[lt_s] <= 1'b1; lt_key[lt_s] <= {lt_d, lt_w, lt_h, lt_lv, lt_b16};
                end
            end
            // F2: pixel f2_op's conversions (stage A) and cube ops; the quad's own fields at the take, commit next cycle
            if (f2_run) f2_op <= f2_take ? 3'd0 : f2_op + 3'd1;
            fb_va <= f2_run & (F2_DUAL ? (f2_dual | ((f2_op < 3'd4) & ~f2_op[0])) : (f2_op < 3'd4));
            fb_vb <= f2_run & F2_DUAL & (f2_dual | ((f2_op < 3'd4) & f2_op[0]));
            fb_hi <= f2_hi; fb_pa <= f2_pa; fb_slot <= wf_slot;
            begin : f2a
                reg [1:0] pa, pb; pa = f2_pa; pb = {f2_hi, 1'b1};
                fba_s <= fxa(f1_c[(pa*4+0)*W +: W], 24, 41); fba_t <= fxa(f1_c[(pa*4+1)*W +: W], 24, 41);
                fba_r <= fxa(f1_c[(pa*4+2)*W +: W], 24, 41); fba_q <= fxa(f1_c[(pa*4+3)*W +: W], 8, 13);
                fbb_s <= fxa(f1_c[(pb*4+0)*W +: W], 24, 41); fbb_t <= fxa(f1_c[(pb*4+1)*W +: W], 24, 41);
                fbb_r <= fxa(f1_c[(pb*4+2)*W +: W], 24, 41); fbb_q <= fxa(f1_c[(pb*4+3)*W +: W], 8, 13);
            end
            if (f2_take) begin
                for (int p = 0; p < 4; p++) wf_face[wf_slot][p] <= f1_face[p];
                wf_psel[wf_slot][1] <= f2_diff[1] ? 2'd0 : 2'd3;
                wf_psel[wf_slot][2] <= f2_diff[2] ? (f2_diff[1] ? 2'd1 : 2'd0) : 2'd3;
                wf_psel[wf_slot][3] <= f2_diff[3] ? 2'(f2_diff[1]) + 2'(f2_diff[2]) : 2'd3;
                wf_s[wf_slot] <= f1_s; wf_m[wf_slot] <= f1_m; wf_base[wf_slot] <= f1_base; wf_w[wf_slot] <= f1_w;
                wf_h2[wf_slot] <= f1_h; wf_fmt[wf_slot] <= f1_fmt; wf_ws[wf_slot] <= f1_ws; wf_wt[wf_slot] <= f1_wt;
                wf_lv[wf_slot] <= f1_lv; wf_filt[wf_slot] <= f1_filt; wf_cube[wf_slot] <= f1_cube; wf_inc[wf_slot] <= f1_inc;
                wf_np[wf_slot] <= f1_np; wf_chain[wf_slot] <= lt_chain[f1_s];
                wf_d[wf_slot] <= f1_d; wf_wr[wf_slot] <= f1_wr; wf_vol[wf_slot] <= f1_vol; wf_mk[wf_slot] <= f1_mk;
            end
            wf_cp <= f2_take;
            if (wf_cp) wf_t <= wf_t + 5'd1;
            if (f3_pop) wf_r <= wf_r + 5'd1;                  // F3 took the head: the slot stays allocated
            if (f4_take & f3f_v) wf_h <= wf_h + 5'd1;         // F4 took stage f's quad (the oldest allocated slot): free it
            // divider results landed per slot; a slot's count restarts when F2 allocates it (its previous occupant's
            // results all landed before it was popped; none of its own can land in the allocation cycle: the divider
            // is >= 15 cycles deep (DQ + the post stages) and a quad issues at most 7 ops)
            for (int i = 0; i < WF; i++) begin
                if (f2_take && wf_slot == 4'(i)) wf_rc[i] <= 3'd0;
                else if (d2_v && d2_slot == 4'(i)) wf_rc[i] <= wf_rc[i] + 3'd1;
            end
        end
    end
    // synthesis translate_off
    always @(posedge clk) if (!rst_q && f2_take && d2_v && d2_slot == wf_slot)
        $fatal(1, "tmu24: a divider result lands in the cycle its slot %0d is reallocated", wf_slot);
    // synthesis translate_on
endmodule
`default_nettype wire
