// b_render — board B's render engine: after `go` (FLUSH) it consumes one frame of the render
// stream and draws it into the render-target image: quad_setup -> early depth -> fragment shader (the scalar
// core shader_core_fs2: warps of 4 quads, FP24, LDV interpolation) <-> tmu24 -> b_rop, in stream
// order; `done` once the frame's FRAME_END is written.
// Items (64-bit beats {w1, w0}); integer edge values instead of barycentrics;
// a QUAD is ONE beat and B forms its edge values itself:
//   QUAD      1 beat: w0 {1, cov[27:24], qy[23:12], qx[11:0]}, w1 0. The edge values at the quad's pixel 0 are
//             e_k = er_k + A_k (2 qx) + B_k (2 qy) in 32-bit wrapping arithmetic from the current TRI's reference values
//             er_k (the edge values at the centre of pixel (0, 0), >> s) and steps (exact for s = 0 — every primitive
//             that fits the target — and off by the steps' dropped fraction bits otherwise); quad_setup steps to pixel p
//             and forms the mus
//   TRI       13 + 5 nv beats: w0 {2, prim[27:26], front[25], varying mask[15:8]}, w1 triangle id[19:0], w2 draw id[12:0],
//             w3 R = RCP(i2f(a2 >> s)) (FP24), w4..w11 z0 dz1 dz2 q0 q1 q2 dz/dx dz/dy (FP24), w12..w17 the per-pixel
//             edge steps A0 B0 A1 B1 A2 B2 (signed 32-bit, x and y per edge), w18..w21 the depth-bound words (zq_tri:
//             {ok, k, m1}, m2, Clo, Chi), w22..w24 the reference edge values er0 er1 er2 (signed
//             32-bit), w25 0, per set mask bit (ascending) 10 words: f0, D1, D2 (vec4, 96 bits each), 0. Made
//             current. Parsed TWO beats per cycle when the render buffer holds them (the header's 12 beats
//             after the first are 6 pairs, a varying's 5 beats are taken in pairs across varying boundaries). Board A
//             sends the TRI item again in every band where the triangle is not current; B keeps no record store.
//   TRISEL    1 beat: w0 {3, ..}, w1 triangle id: a no-op when it names the current triangle; any other is counted in
//             c_errs and skipped (board A resends the TRI item instead)
//   BAND      1 beat: w0 {4, .., y0[10:0]}: flushes the band in progress, starts band y0 (nl rows: NLMAX = 8 when the
//             target is at most BAND8_MAXW = 1024 wide — the band buffer's CELLS = 4 096 cells then hold 8 lines of it —
//             else NLINES = 4; board A bins and walks with the same rule)
//   FRAME_END 1 beat: w0 {5, .., aborted[0]}: flushes the last band, waits for its writes, done (aborted: board A gave
//             the frame up after RENDER_TIMEOUT and ended it in-band, so the pairing of FLUSHes and frames holds)
//   NOP / other kinds: 1 beat, skipped (other kinds counted in c_errs)
// Per draw: when a triangle of another draw becomes current, every quad in flight drains and the draw's bucket
// (DS_BASE + draw id * 512 words, written by the command processor) is read: bursts 0-1 = state words 0..15, bursts
// 2-5 = 8 samplers (4 words each), bursts 6.. = uniforms (2 vec4 per burst, 4 words each); the program (B buffer word
// address + length from the state) is reloaded only when it differs from the loaded one.
//   state words: 0 flags {cmask[19:16], po[9], scissor[8], dither[7], blend[6], stencil[5], depth func[4:2], depth
//   mask[1], depth test[0]}; 1 / 2 stencil front / back {func[26:24], wmask[23:16], vmask[15:8], ref[7:0]}; 3 ops
//   {back dppass[24:22] dpfail[21:19] sfail[18:16], front dppass[8:6] dpfail[5:3] sfail[2:0]}; 4 blend {op_a[22:20],
//   op[18:16], dst_a[15:12], src_a[11:8], dst[7:4], src[3:0]}; 5 / 6 blend constant {G[25:13], R[12:0]} / {A, B}
//   in the target's channel units (round(c * 16 (2^n - 1))); 7 / 8 scissor {y0[28:16], x0[12:0]} / {y1, x1};
//   9 polygon-offset factor (FP24); 10 units (s24); 11 program word address; 12 {uniform count[22:16], program
//   length[11:0]}; 13 the constant alpha in the units of the colour channels {in G's units[25:13], in R and B's[12:0]}.
//   sampler: base word address; {h[27:16], w[11:0]}; {cube[16], incomplete[15], wrap_t[13:12], wrap_s[11:10],
//   filt[9:6], levels[5:2], fmt[1:0]}; 0.
// Quad path: a QUAD is collected (counted in flight at once, so its triangle's slots are not reused under it), its 4
// pixels go through quad_setup when the quad FIFO behind it has room, the collector joins each quad's 4 results, the
// launch stage runs early depth (only where a stale stored depth can only make the test pass more: depth test on,
// stencil off, gl_FragDepth not written (the program holds no STOZ), func not NOTEQUAL unless depth writes are off)
// and adds the quad to the warp being formed: its launch rows (gl_FragCoord.xy, .zw, gl_PointCoord (mu1, mu2),
// the weights lambda1, lambda2; two components per pixel; only the weights when the program reads no built-in) are written
// into the core, its facing bit and triangle slot noted; the warp starts with 4 quads, or earlier when the quad path is
// empty and the sequencer waits for quads to finish (slot reuse, a draw change, a band or the frame end) or after 32
// idle cycles. Finished warps leave in launch order, through the core's output latch, which frees a warp's slot before
// its rows are read (the per-quad metadata therefore has one warp entry more than the core has slots);
// the back end reads each quad's colour and depth rows from the core's output store and each covered, kept
// pixel goes to b_rop with its colour in the render target's channel units (round(c * 16 (2^n - 1)) per channel of n
// bits; its alpha also in the units of the colour channels, for the alpha factors) and its depth (gl_FragDepth if
// written, else the interpolated one) as unorm of the attachment's 24 or 16 bits. The program is loaded in 64-bit
// instructions (4 per DDR burst; the bucket's length counts instructions).
// The frame's render target (FLUSH): format rt_fmt, depth attachment of 24 or 16 bits (depth16), an optional D24S8
// depth-stencil image (ds_*) that the bands load and flush; the clear flags per buffer.
`default_nettype none
module b_render #(
    parameter int NLINES  = 4,                     // band rows at the widest target (the band buffer's depth: MAXW/2 x NLINES cells)
    parameter int NLMAX   = 8,                     // band rows for targets up to BAND8_MAXW wide (the same cells)
    parameter int BAND8_MAXW = 1024,
    parameter int CELLS   = 4096,                  // the band buffer's cells per bank (b_rop): >= MAXW/2 x NLINES and NLMAX x BAND8_MAXW/2
    parameter int MAXW    = 2048,
    parameter int NTS     = 16,                    // fragment-core triangle slots: the triangle visits with quads alive between the
                                                   // early queue and the ROP (with 8 the sequencer waits 20 % of an
                                                   // OpenArena-like frame for a slot; the store's block RAM is the same)
    parameter int FS_NWARP = 4,                    // shader_core_fs2 warps in flight
    parameter int FS_TEXQ  = 8,                    // TEX instructions live at once (with tmu24's 16-deep request queue four warps
                                                   // x two samples live; with 2 the 4-sample full-screen pass is TEX-latency-bound:
                                                   // 46 cycles per quad against an issue ideal of 25)
    parameter [28:0] DS_BASE  = 29'h0200_0000,
    parameter bit    VOL3D    = 1'b0,              // tmu24's 3D texture path (see there); off = constant-folded away
    parameter bit    F2_DUAL  = 1'b1,              // tmu24's F2 at two pixels per cycle (0 = the −0.6 k LUT fallback)
    parameter bit    TC_PF    = 1'b1,              // tmu24's texture-cache prefetch (0 = the 64 KB cache alone)
    parameter bit    TEX_SKIP = 1'b1,              // uncovered pixels of a quad are not sampled when the program cannot tell
                                                   // (the program scan below, tmu24's req_mask);
                                                   // 0 = every pixel of every quad sampled
    parameter bit    TEX_W2   = 1'b1,              // the fragment core's two texture-result writers (with TEX_SKIP: quads of fewer
                                                   // than 4 samples return faster than the one writer's 4 cycles); 0 = one
    parameter bit    FS_STO2  = 1'b1,              // the fragment core's colour store in 2 micro-ops (x y, z w) instead of 4
                                                   // (the back end's two row reads are the store's even / odd halves)
    // the next three stay 0 on the boards (FS_LDV 2's second multiplier and adder: +3.2 % / +4.7 % OpenArena frame rate
    // for +4 406 LUT and a timing miss; the profile counters are for measuring builds: they cost timing at 97.5 % of the
    // slices). Each one is bit-identical in what it computes.
    parameter bit    PROF     = 1'b0,              // the cumulative profile counters (hwprof): where the frame's cycles go;
                                                   // 0 = the 24 words read 0 (the JTAG frame keeps its 48 words)
    parameter bit    FS_ADD_DSP = 1'b0,            // the fragment core's lane adders align on a DSP48 each (-485 LUT for
                                                   // +32 DSP out of context with FS_LDV 2, and the core's worst synthesized path shorter)
    parameter int    FS_LDV   = 0,                 // the fragment core's varying load (shader_core_fs2 LDV_MODE): 0 two issued
                                                   // micro-ops per component, 1 one + the second step by
                                                   // itself in the same lanes, 2 one + a second multiplier and adder per lane
    // FS_TEX2, L_FAST and BE_NOZ below are off (together +0.8 % of frame rate for +1.1 k LUT and a timing miss);
    // the core's TEX write ordering (hzr) does not depend on them
    parameter int    FS_QRINF = 4,                 // the fragment core's quarter-rate micro-ops in flight per class (shader_core_fs2
                                                   // QR_INF; 4: a trig-heavy shader does not wait for its class)
    parameter bit    FS_SFU_TAB2 = 1'b0,          // the fragment core's SFU tables shared by pairs of units (shader_core_fs2 SFU_TAB2)
    parameter bit    FS_XSFU  = 1'b1,             // the fragment core's SQRT / RCPM / RSQM: the SFU's result times an operand, one
                                                   // micro-op (shader_core_fs2 XSFU)
    parameter bit    FS_XCLAMP = 1'b0,             // the fragment core's CLAMP: MIN(MAX(x, lo), hi) in one micro-op (XCLAMP); off
                                                   // on the boards (+1 664 LUT for one micro-op per clamp)
    parameter bit    FS_TEX2  = 1'b0,              // the fragment core's TEX coordinates two per micro-op (a 2D lookup: 1 instead of 2)
    parameter bit    L_FAST   = 1'b0,              // launch: a full warp starts with its last quad's last row and the next warp's
                                                   // first row follows after ONE idle cycle; 0 = L_GO + two idle cycles
    parameter bit    BE_NOZ   = 1'b0,              // back end: a warp whose program writes no gl_FragDepth is read in 2 cycles per
                                                   // quad (no depth row) instead of 3
    parameter bit    FS_PRE   = 1'b0               // the fragment core fetches a warp's program from its first quad on.
                                                   // OFF: 5 idle issue cycles per warp on
                                                   // programs with one-cycle operations beside multiply-add chains (fb_bench alu_16 −5…−7 %
                                                   // on the boards) for 0.1–1 % elsewhere. With 0, fs_pre is constant and the core's
                                                   // fetch-ahead logic (pf) is removed by synthesis.
)(
    input  wire        clk, rst,
    // render stream (render buffer of link_b_port): the head beat and the one after it (TRI items are parsed
    // two beats per cycle); rs_ready takes the head, rs_take2 both
    input  wire        rs_valid, rs_valid2,
    input  wire [63:0] rs_data, rs_data2,
    output wire        rs_ready, rs_take2,
    // frame (command processor, FLUSH): render target and clear of the pass
    input  wire        go,
    input  wire        discard,                  // with go: the FLUSH was rejected — consume its frame to FRAME_END, render nothing
    input  wire        abort,                    // A gave the frame up (credit SYNC): leave the frame as if FRAME_END came
    input  wire [28:0] rt_base,                  // word address
    input  wire [10:0] rt_bpl,                   // pitch / 32 bytes
    input  wire [10:0] rt_width,
    input  wire [11:0] rt_height,                // the target's height (the FLUSH's image): where the band preload stops
    input  wire [1:0]  rt_fmt,                   // 0 RGBA8, 1 RGBA4444, 2 RGBA5551, 3 RGB565 (tmu24 codes)
    input  wire        depth16,                  // DEPTH_COMPONENT16 attachment (else 24 bits)
    input  wire        ds_en,                    // depth-stencil image attached (D24S8: persistence across FLUSHes)
    input  wire [28:0] ds_base,
    input  wire [10:0] ds_bpl,
    input  wire [51:0] clear_c16,                // clear colour in the target's channel units
    input  wire        clear_dither, clear_color_en, clear_sc_en,
    input  wire [3:0]  clear_cmask,
    input  wire [12:0] clear_x0, clear_y0, clear_x1, clear_y1,
    input  wire [23:0] clear_depth,              // in the depth attachment's units
    input  wire [7:0]  clear_stencil,
    input  wire        clear_depth_en, clear_stencil_en,
    input  wire [7:0]  clear_smask,
    output reg         done,
    output wire        busy,
    // DDR read master: buckets, uniforms, programs (rd_data is the arbiter's shared read data)
    output reg         r_en, output reg [28:0] r_addr, input wire r_rdy, r_valid, input wire [255:0] rd_data,
    // DDR read masters: textures, band load
    output wire        t_en, output wire [28:0] t_addr, input wire t_rdy, t_valid,
    output wire        l_en, output wire [28:0] l_addr, input wire l_rdy, l_valid,
    // DDR write master: band flush
    output wire        fw_req, output wire fw_en, output wire [28:0] fw_addr, output wire [255:0] fw_data,
    output wire [31:0] fw_mask, output wire fw_wren, fw_wend, input wire fw_rdy, fw_wdf_rdy,
    // status
    output reg  [31:0] c_quads, c_tris, c_ezrej, c_frames, c_errs,
    output reg  [31:0] c_ezrej1,                 // quads the early queue rejected (dropped ahead of quad_setup)
    output wire        err_stack,                // a fragment-program stack violation since reset (sticky, the core's)
    output reg  [255:0] perf,                    // the last frame's cycle counters (latched with done)
    output wire [31:0]  st_tjobs, st_tast, st_thit, st_tmiss,   // the texture unit's cumulative counters
    // 24 cumulative 32-bit counters since reset (word k at [k*32 +: 32]), counted while a frame is in progress (PROF):
    //  0 cycles          1 flush busy       2 band change       3 draw change       4 FS busy      5 band set-up
    //  6 stream empty    7 ROP stall        (0..7 as perf, summed over the frames)
    //  8 cycles with an issue               9 sum of warps waiting with a texture lookup outstanding
    // 10 sum of warps whose head micro-op is not loaded (program fetch)       11 sum of warps waiting otherwise
    // 12 sum of busy warps                 13 sum of live TEXes                14 warps launched
    // 15 pixel-issuer cycles (4 per quad through quad_setup)                   16 a QUAD waits for room in the early queue
    // 17 a quad that passed waits for a warp slot                              18 the texture unit holds work
    // 19 the texture filter's head waits for data                              20 texture requests (quads)
    // 21 sum of quads inside the texture unit (request .. result; / word 20 = the round trip)
    // 22 texture DDR reads                 23 sum of texture DDR reads outstanding (/ word 22 = cycles per read)
    output wire [24*32-1:0] hwprof,
    output reg         frame_err                 // the last frame hit a stack violation (its quads were killed; with done)
);
    localparam int W = 24, TW = 96, VW = 384, CW = 13;
    // the engine's own reset register, also the reset of quad_setup, the FS core, the TMU and the ROP: one
    // cycle behind board_b_core's (the core and the TMU add one of their own; nothing starts within those cycles)
    (* max_fanout = 64 *) reg rst_q = 1'b1;
    always @(posedge clk) rst_q <= rst;
    localparam int TSW = $clog2(NTS);
    localparam [W-1:0] F_ONE = 24'h3F0000;

    // ================================================================ conversions
    // Three pipeline steps each: A = kind, product, shift amount; B1 = shift and round bits; B2 = round, saturate.
    // channel value = clamp(f, 0, 1) * ONE rounded to nearest (no ties), NaN -> 0 (b_ref f24_cn); ONE = 16 (2^n - 1)
    // for a channel of n bits of the render target (4080 = c16 for RGBA8)
    function automatic [3:0] chn(input [1:0] f, input [1:0] ch);   // channel widths {R, G, B, A} per format (as tmu24)
        case (f)
            2'd0: chn = 4'd8;
            2'd1: chn = 4'd4;
            2'd2: chn = (ch == 2'd3) ? 4'd1 : 4'd5;
            default: chn = (ch == 2'd1) ? 4'd6 : (ch == 2'd3) ? 4'd0 : 4'd5;
        endcase
    endfunction
    function automatic [12:0] one_of(input [3:0] n);
        one_of = (13'd1 << (n + 4)) - 13'd16;
    endfunction
    function automatic [37:0] cn_a(input [W-1:0] f, input [3:0] n);  // {kind (0 num, 1 zero, 2 one), rsh[6:0], p[28:0]}
        reg [6:0] e; reg [16:0] sig; reg [1:0] k;
        e = f[22:16]; sig = {1'b1, f[15:0]};
        k = (f[23] || e == 7'd0 || (e == 7'h7F && f[15:0] != 0)) ? 2'd1 : (e >= 7'd63) ? 2'd2 : 2'd0;
        cn_a = {k, 7'd79 - e, (29'(sig) << (n + 4)) - (29'(sig) << 4)};   // value * ONE = p * 2^-rsh, rsh >= 17
    endfunction
    function automatic [14:0] c16_b1(input [37:0] a);           // {kind, round bit, q[11:0]}
        reg [1:0] k; reg [6:0] rsh; reg [28:0] p;
        {k, rsh, p} = a;
        if (k != 2'd0 || rsh > 7'd29) c16_b1 = {k, 13'd0};         // p < 2^29: below 1/2
        else c16_b1 = {k, p[rsh - 7'd1], 12'(p >> rsh)};
    endfunction
    function automatic [12:0] c16_b2(input [14:0] b, input [12:0] one);
        c16_b2 = (b[14:13] == 2'd1) ? 13'd0 : (b[14:13] == 2'd2) ? one : 13'(b[11:0]) + 13'(b[12]);
    endfunction
    // depth = clamp(f, 0, 1) * (2^b - 1) rounded to nearest even (b_ref f24_u24 / f24_u16): gl_FragDepth, b = 24 or 16
    function automatic [50:0] u24_a(input [W-1:0] f, input d16);   // {kind, rsh[6:0], p[41:0]}
        reg [6:0] e; reg [16:0] sig; reg [1:0] k;
        e = f[22:16]; sig = {1'b1, f[15:0]};
        k = (f[23] || e == 7'd0 || (e == 7'h7F && f[15:0] != 0)) ? 2'd1 : (e >= 7'd63) ? 2'd2 : 2'd0;
        u24_a = {k, 7'd79 - e, d16 ? (42'(sig) << 16) - 42'(sig) : (42'(sig) << 24) - 42'(sig)};
    endfunction
    function automatic [27:0] u24_b1(input [50:0] a);           // {kind, guard, sticky, q[23:0]}; q < 2^24
        reg [1:0] k; reg [6:0] rsh; reg [41:0] p;
        {k, rsh, p} = a;
        if (k != 2'd0 || rsh > 7'd41) u24_b1 = {k, 26'd0};
        else u24_b1 = {k, p[rsh - 7'd1], |(p & ((42'd1 << (rsh - 7'd1)) - 42'd1)), 24'(p >> rsh)};
    endfunction
    function automatic [23:0] u24_b2(input [27:0] b, input d16);   // half to even: up = guard & (sticky | odd)
        u24_b2 = (b[27:26] == 2'd1) ? 24'd0 : (b[27:26] == 2'd2) ? (d16 ? 24'h00FFFF : 24'hFFFFFF)
                                                                  : b[23:0] + 24'(b[25] & (b[24] | b[0]));
    endfunction
    wire [3:0] n_r = chn(rt_fmt, 2'd0), n_g = chn(rt_fmt, 2'd1), n_b = chn(rt_fmt, 2'd2), n_a = chn(rt_fmt, 2'd3);
    wire [12:0] one_r = one_of(n_r), one_g = one_of(n_g), one_b = one_of(n_b), one_a = one_of(n_a);
    function automatic dcmp(input [2:0] f, input [23:0] a, b);     // depth test pass (as b_rop)
        case (f) 3'd0: dcmp = 0; 3'd1: dcmp = (a < b); 3'd2: dcmp = (a == b); 3'd3: dcmp = (a <= b);
                 3'd4: dcmp = (a > b); 3'd5: dcmp = (a != b); 3'd6: dcmp = (a >= b); default: dcmp = 1; endcase
    endfunction

    // ================================================================ draw state (bucket): two banks
    // db = the bank of the active draw (quads collected from now on carry it); the next draw's bucket loads into ~db as
    // soon as no quad of the draw that used ~db is alive and the ROP has left it; the program / uniforms go to the
    // core's bank, the samplers to descriptor entries {bank, k}, the ROP state is selected by the bank of the fragment
    // at the ROP input (rop_bank, switched when the RMW pipe is empty), polygon offset by db at the TRI's write.
    typedef struct packed {
        logic        depth_test, depth_mask, stencil, blend, dither, scissor, po;
        logic [2:0]  depth_func;
        logic [3:0]  cmask;
        logic [1:0][7:0] sref, svm, swm;         // [1] front, [0] back
        logic [1:0][2:0] sfn, osf, odf, odp;
        logic [3:0]  bsf, bdf, bsfa, bdfa;
        logic [2:0]  bop, bopa;
        logic [51:0] bcc;                        // blend constant {R, G, B, A} in channel units
        logic [25:0] bca;                        // constant alpha in G's units [25:13], in R / B's units [12:0]
        logic [12:0] sx0, sy0, sx1, sy1;
        logic [W-1:0] pofac; logic [23:0] pounits;
        logic [28:0] prog; logic [11:0] plen; logic [6:0] nuni;
    } dst_t;
    dst_t       ds [0:1];
    reg         db, rop_bank;                    // the active draw's bank; the bank of the ROP's state
    wire        ld_bank = ~db;                   // the bank being loaded
    dst_t       dsr, dsl;                        // the ROP's state; the bank being loaded (read back by the loader)
    assign dsr = ds[rop_bank]; assign dsl = ds[ld_bank];
    // samplers {bank, k[3:0]} (the bucket loads k < 8), one 80-bit word each: {vol, wrap r[1:0], depth[7:0], cube, inc, wrap t, wrap s,
    // filt[3:0], levels[3:0], fmt[1:0], h[11:0], w[11:0], base[28:0]}, in two LUTRAM banks by k[0] (the bucket loads samplers 2j and
    // 2j + 1 in one cycle: one write per bank), read by dsel = {bank, k}; the read is asynchronous (tmu24's F1 latches it).
    (* ram_style = "distributed" *) reg [79:0] s_even [0:15], s_odd [0:15];
    function automatic [79:0] samp_word(input [127:0] r);   // the bucket's 128-bit sampler record -> the stored word
        samp_word = {r[106], r[104 +: 2], r[96 +: 8], r[80], r[79], r[76 +: 2], r[74 +: 2], r[70 +: 4], r[66 +: 4], r[64 +: 2], r[48 +: 12], r[32 +: 12], r[0 +: 29]};
    endfunction
    // per bank: the loaded program, it writes depth (holds a STOZ), it reads a built-in (LDV 8..15: all 4 launch rows)
    reg  [28:0] ld_prog [0:1]; reg [11:0] ld_plen [0:1]; reg [1:0] prog_ok, wfd, ubi;
    reg  [12:0] active_draw; reg draw_ok;
    // per bank: the draw whose state (bucket, uniforms, program) it holds, loaded in THIS frame: a draw
    // that returns while the other bank still holds its state (two states alternating: A B A B ...) switches banks
    // without a reload — the bank's quads in flight run under the same state, so nothing waits
    reg  [1:0]  bk_ok; reg [12:0] bk_draw [0:1];
    wire        bk_swap;                          // (defined with t_draw below)

    // ================================================================ burst fetch engine (shared read master)
    // fe_go: fetch fe_n bursts from fe_a into rf (16 x 256); consumers pop with rf_pop. Reads are issued while fewer than
    // 15 bursts are outstanding or buffered.
    reg          fe_go; reg [28:0] fe_a; reg [9:0] fe_n;
    reg  [9:0]   fe_tot, fe_iss, fe_pop;
    reg  [255:0] rf_mem [0:15]; reg [3:0] rf_wp, rf_rp;
    wire         rf_nonempty = rf_wp != rf_rp;
    wire [255:0] rf_head = rf_mem[rf_rp];
    reg          rf_pop;
    wire         fe_done = (fe_pop == fe_tot);                    // every burst returned and popped
    always @(posedge clk) begin
        if (rst_q) begin r_en <= 1'b0; fe_tot <= 10'd0; fe_iss <= 10'd0; fe_pop <= 10'd0; rf_wp <= 4'd0; rf_rp <= 4'd0; end
        else if (fe_go) begin
            r_en <= 1'b0; r_addr <= fe_a; fe_tot <= fe_n; fe_iss <= 10'd0; fe_pop <= 10'd0; rf_wp <= 4'd0; rf_rp <= 4'd0;
        end else begin
            if (r_en && r_rdy) begin r_en <= 1'b0; fe_iss <= fe_iss + 10'd1; r_addr <= r_addr + 29'd8; end
            else if (!r_en && fe_iss != fe_tot && (fe_iss - fe_pop) < 10'd15) r_en <= 1'b1;
            if (r_valid) begin rf_mem[rf_wp] <= rd_data; rf_wp <= rf_wp + 4'd1; end
            if (rf_pop) begin rf_rp <= rf_rp + 4'd1; fe_pop <= fe_pop + 10'd1; end
        end
    end

    // ================================================================ beat source: the render buffer
    wire         sb_valid = rs_valid, sb2_valid = rs_valid2;
    wire [63:0]  sb_data  = rs_data,  sb2_data  = rs_data2;
    reg          sb_take, sb_take2;               // take the head beat / both beats
    assign rs_ready = sb_take | sb_take2;
    assign rs_take2 = sb_take2;
    wire [31:0]  w0 = sb_data[31:0], w1 = sb_data[63:32], w2 = sb2_data[31:0], w3 = sb2_data[63:32];
    // the band height of this frame's target: NLMAX lines up to BAND8_MAXW wide, else NLINES — board A's rule too
    wire [3:0]   nl = (rt_width <= 11'(BAND8_MAXW)) ? 4'(NLMAX) : 4'(NLINES);
    if (NLMAX * ((BAND8_MAXW + 1) / 2) > CELLS)
        $error("b_render: NLMAX lines of a BAND8_MAXW-wide target do not fit the band buffer (CELLS)");
    if (NLINES * ((MAXW + 1) / 2) > CELLS)
        $error("b_render: NLINES lines of a MAXW-wide target do not fit the band buffer (CELLS)");

    // ================================================================ quad_setup, quad info, quad FIFO
    // QFD 16: quad_setup + the quad FIFO may hold 16 quads, so its 45-cycle latency
    // does not cap the quad path at about 6 cycles per quad
    localparam int QFD = 16;
    reg          qs_we; reg [1:0] qs_slot;
    reg  [W-1:0] t_sc [0:7]; reg [W-1:0] t_R; reg signed [31:0] t_st [0:5];      // the TRI's scalars, R, edge steps
    reg          px_valid; reg [1:0] px_slot; reg signed [31:0] px_e0, px_e1, px_e2; reg [1:0] px_p; reg [12:0] px_x, px_y; reg [1:0] px_tag;
    wire         o_valid; wire [1:0] o_tag; wire [W-1:0] o_fw, o_l1, o_l2, o_fcx, o_fcy, o_fcz, o_mu1, o_mu2; wire [23:0] o_d24;
    quad_setup #(.EXP_W(7), .MAN_W(16), .TAGW(2), .NT(4)) u_qs (.clk, .rst(rst_q),
        .tri_we(qs_we), .tri_slot(qs_slot), .tri_z0(t_sc[0]), .tri_dz1(t_sc[1]), .tri_dz2(t_sc[2]), .tri_q0(t_sc[3]),
        .tri_q1(t_sc[4]), .tri_q2(t_sc[5]), .tri_dzdx(t_sc[6]), .tri_dzdy(t_sc[7]), .tri_R(t_R),
        .tri_A0(t_st[0]), .tri_B0(t_st[1]), .tri_A1(t_st[2]), .tri_B1(t_st[3]), .tri_A2(t_st[4]), .tri_B2(t_st[5]),
        .po_en(ds[db].po), .po_factor(ds[db].pofac), .po_units(ds[db].pounits), .depth16,   // per slot at the TRI's write (M_TRIQS, after the load)
        .px_valid, .px_slot, .px_e0, .px_e1, .px_e2, .px_p, .px_x, .px_y, .px_tag,
        .o_valid, .o_tag, .o_z(), .o_fw, .o_l1, .o_l2, .o_d24, .o_fcx, .o_fcy, .o_fcz, .o_mu1, .o_mu2);
    // quad info (in issue order): coordinates, coverage, slots, facing; (+EZAUDIT) the early stage rejected it
    reg  [11:0] qi_qx [0:15], qi_qy [0:15]; reg [3:0] qi_cov [0:15]; reg [TSW-1:0] qi_ts [0:15]; reg [1:0] qi_qs [0:15];
    reg         qi_front [0:15], qi_bank [0:15]; reg [1:0] qi_slot [0:15]; reg qi_er [0:15];
    reg  [3:0]  qi_wp, qi_rp;
    // quad FIFO (collector -> launch)
    reg  [11:0] qf_qx [0:QFD-1], qf_qy [0:QFD-1]; reg [3:0] qf_cov [0:QFD-1]; reg [TSW-1:0] qf_ts [0:QFD-1];
    reg         qf_front [0:QFD-1], qf_bank [0:QFD-1]; reg [1:0] qf_slot [0:QFD-1]; reg [191:0] qf_pc [0:QFD-1];
    reg  [95:0] qf_d24 [0:QFD-1]; reg [95:0] qf_fz [0:QFD-1], qf_fw [0:QFD-1]; reg [191:0] qf_lam [0:QFD-1];
    reg  [95:0] qf_fc [0:QFD-1];                 // gl_FragCoord.xy {y + 1.5, y + 0.5, x + 1.5, x + 0.5} (2 qx, 2 qy)
    reg         qf_er [0:QFD-1];                 // (+EZAUDIT) the early stage rejected it: the late test must too
    (* max_fanout = 32 *) reg [4:0] qf_wp;                     // write pointer of ≈ 900 LUTRAM bits: replicated (timing)
    reg  [4:0]  qf_rp;
    wire [4:0]  qf_cnt = qf_wp - qf_rp;
    reg  [4:0]  qs_infl;                         // quads in quad_setup
    // the early queue: collected quads wait here, AHEAD of quad_setup, for
    // their conservative depth bounds (zq_lane, from the TRI item's words w18..w21) and a peek of the stored depths; a quad
    // whose every covered pixel certainly fails the depth test is dropped here (about 2 cycles, the QUAD item's 2 beats,
    // instead of the whole quad path). The others take the exact test after quad_setup, with a fresh peek (a peek
    // this early is staler: fewer quads would be rejected before the shader). The two stages share b_rop's peek port, the
    // late one first. In order: push (sequencer) -> bounds (lane, LAT 6) -> peek (eq_pp) -> issue (eq_rp).
    localparam int EQD = 16;
    (* ram_style = "distributed" *) reg [11:0] eq_qx [0:EQD-1], eq_qy [0:EQD-1];
    (* ram_style = "distributed" *) reg [95:0] eq_e [0:EQD-1];          // {e2, e1, e0}
    (* ram_style = "distributed" *) reg [3:0] eq_cov [0:EQD-1];
    (* ram_style = "distributed" *) reg [TSW+6-1:0] eq_m [0:EQD-1];     // {ts, qs, front, bank, slot}
    (* ram_style = "distributed" *) reg [47:0] eq_bd [0:EQD-1];         // {dhi, dlo} from the lane
    reg  [EQD-1:0] eq_lv, eq_zok, eq_vd, eq_rej;           // bounds in / valid, verdict in / reject
    reg  [4:0]  eq_wp, eq_rp, eq_pp;
    wire [4:0]  eq_cnt = eq_wp - eq_rp;

    // ================================================================ fragment shader (scalar warps of 4 quads) + TMU
    wire        fs_in_ready, fs_in_ready_nx, fs_out_valid, fs_idle, fs_tri_ready, fs_out_depthw, fs_out_err; wire [1:0] fs_in_warp;
    reg         fs_start, fs_ack, fs_vw_we; reg [1:0] fs_vw_row, fs_vw_quad; reg [8*W-1:0] fs_vw_data;
    reg         fs_pre;                                      // the warp being formed got its first quad: the core fetches its program ahead
    reg  [15:0] fs_in_active; reg [3:0] fs_in_front; reg [4*TSW-1:0] fs_in_tslot;
    reg  [15:0] fs_in_tmask;                                 // the lanes whose texels are used (coverage, or all)
    reg  [1:0]  tsk;                                         // the bank's program may leave uncovered pixels unsampled
    reg         tk_start; wire tk_rdy, tk_badn; reg tk_bad;  // ... the scan that decides it (below)
    wire [3:0]  tx_mask; wire tx_cls;
    reg         fs_tri_we; reg [TSW-1:0] fs_tri_slot; reg [2:0] fs_tri_vidx; reg [3*TW-1:0] fs_tri_data;
    reg         ld_iram_we, ld_par_we; reg [11:0] ld_iram_addr; reg [63:0] ld_iram_data; reg [5:0] ld_par_idx;
    reg  [TW-1:0] ld_par_data; reg ld_wfd, ld_ubi;           // the program being loaded writes gl_FragDepth / reads a built-in
    reg         cur_wbank;                                   // the bank of the warp being formed
    wire [4:0]  fs_out_sel, fs_out_sel2; wire [4*W-1:0] fs_out_row, fs_out_row2; wire [15:0] fs_out_kill;
    wire        tx_req, tx_ready, tx_res_v; wire [4:0] tx_samp; wire [1:0] tx_mode; wire [8*W-1:0] tx_uv, tx_zw;
    wire [VW-1:0] tx_res;
    wire [21:0] fs_pev; wire [1:0] tm_pev;                   // profile events of the core and the texture unit
    shader_core_fs2 #(.PROF(PROF), .EXP_W(7), .MAN_W(16), .NWARP(FS_NWARP), .NPARAM(64), .NTS(NTS), .TEXQ(FS_TEXQ), .TEXW2(TEX_W2), .STO2(FS_STO2), .TEX2(FS_TEX2), .LDV_MODE(FS_LDV), .ADD_DSP(FS_ADD_DSP), .XSFU(FS_XSFU), .XCLAMP(FS_XCLAMP), .QR_INF(FS_QRINF), .SFU_TAB2(FS_SFU_TAB2)) u_fs (.clk, .rst(rst_q),
        .iram_we(ld_iram_we), .iram_waddr({ld_bank, ld_iram_addr}), .iram_wdata(ld_iram_data),
        .param_we(ld_par_we), .param_idx({ld_bank, ld_par_idx}), .param_data(ld_par_data),
        .vw_we(fs_vw_we), .vw_row(fs_vw_row), .vw_quad(fs_vw_quad), .vw_data(fs_vw_data),
        .tri_we(fs_tri_we), .tri_ready(fs_tri_ready), .tri_slot(fs_tri_slot), .tri_vidx(fs_tri_vidx), .tri_data(fs_tri_data),
        .in_ready(fs_in_ready), .in_ready_nx(fs_in_ready_nx), .in_warp(fs_in_warp), .in_start(fs_start), .in_bank(cur_wbank), .in_active(fs_in_active),
        .in_front(fs_in_front), .in_tslot(fs_in_tslot), .in_tmask(fs_in_tmask), .pre_start(fs_pre), .pre_bank(cur_wbank),
        .tex_req_valid(tx_req), .tex_req_ready(tx_ready), .tex_req_sampler(tx_samp), .tex_req_mode(tx_mode), .tex_req_mask(tx_mask), .tex_req_cls(tx_cls),
        .tex_req_uv(tx_uv), .tex_req_zw(tx_zw), .tex_res_valid(tx_res_v), .tex_res_texels(tx_res),
        .out_valid(fs_out_valid), .out_ack(fs_ack), .out_sel(fs_out_sel), .out_row(fs_out_row), .out_sel2(fs_out_sel2), .out_row2(fs_out_row2),
        .out_depthw(fs_out_depthw), .out_kill(fs_out_kill), .out_err(fs_out_err), .err_stack(err_stack), .idle(fs_idle), .prof_ev(fs_pev));
    wire [4:0] dsel; reg tex_inval;
    wire [79:0] s_sel = dsel[0] ? s_odd[dsel[4:1]] : s_even[dsel[4:1]];           // the sampler table's word for dsel = {bank, k}
    tmu24 #(.EXP_W(7), .MAN_W(16), .VOL3D(VOL3D), .F2_DUAL(F2_DUAL), .TC_PF(TC_PF)) u_tmu (.clk, .rst(rst_q), .cache_inval(tex_inval),
        .req_valid(tx_req), .req_ready(tx_ready), .req_sampler(tx_samp), .req_mode(tx_mode), .req_uv(tx_uv), .req_zw(tx_zw),
        .req_mask(tx_mask), .req_cls(tx_cls), .desc_idx(dsel), .desc_base(s_sel[0 +: 29]), .desc_w(s_sel[29 +: 12]), .desc_h(s_sel[41 +: 12]),      // dsel = {bank, sampler}
        .desc_fmt(s_sel[53 +: 2]), .desc_levels(s_sel[55 +: 4]), .desc_filt(s_sel[59 +: 4]),
        .desc_wrap_s(s_sel[63 +: 2]), .desc_wrap_t(s_sel[65 +: 2]), .desc_cube(s_sel[68]),
        .desc_incomplete(s_sel[67]), .desc_vol(s_sel[79]), .desc_depth(s_sel[69 +: 8]), .desc_wrap_r(s_sel[77 +: 2]),
        .res_valid(tx_res_v), .res_texels(tx_res),
        .tmu_en(t_en), .tmu_addr(t_addr), .tmu_rdy(t_rdy), .tmu_rd_valid(t_valid), .tmu_rd_data(rd_data),
        .st_jobs(st_tjobs), .st_astall(st_tast), .st_hits(st_thit), .st_misses(st_tmiss), .st_ev(tm_pev));

    // ================================================================ ROP
    reg         band_on; reg [10:0] band_y0; reg band_start, band_flush_req;
    wire        band_flush_done, band_busy, band_run, flush_busy, rmw_idle;
    // band queue: the sequencer announces bands (BAND items) into 4 slots without waiting for
    // anything; quads carry their slot; the band manager renders the slots in order — a band is flushed once the next
    // one is announced (or the frame ends), none of its quads is alive and none of its fragments is behind the core;
    // fragments of a later band wait at the ROP input; peeks of the next band are served from the preload bank
    reg  [10:0] bq_y0 [0:3]; reg [1:0] nq_slot, rs_slot; reg [2:0] bands_q; reg fend;
    // the ROP's cell addresses are formed HERE (timing: band_y0 -> the line subtract -> the row-base mux -> the cell adder
    // stays off the paths into the ROP's bank addresses and the hazard
    // compares): a fragment's band line comes from ITS band's y0 (bq_y0 of its slot — a fragment of the next band waits
    // at br while band_y0 still names the current one), the line's cell base from rb_tab (line x cells per line)
    // (the table has NLMAX lines: a fragment's line within its band is y - y0 mod NLMAX, 0..3 for 4-line bands, 0..7 for 8)
    localparam int RB_AW = $clog2(CELLS), RB_LB = $clog2(NLMAX);
    reg  [RB_AW-1:0] rb_tab [0:NLMAX-1];
    always @(posedge clk) for (int ri = 0; ri < NLMAX; ri++) rb_tab[ri] <= RB_AW'(ri * ((rt_width + 11'd1) >> 1));
    reg  [RB_AW-1:0] br_addr, pk_addr, c2_addr;  // c2_addr: the c2 fragment's cell address (see br_addr_c)
    wire [RB_AW-1:0] br_addr_c, br_addr_n;       // the c2 fragment's cell address; the address br presents next cycle (the ROP's early hazard check)
    // the scissor test too is applied at the br stage, from the fragment's OWN draw-state bank (timing: in the ROP the scissor
    // compares on frag_x would head the address mux of its banks); the ROP gets the pixels
    // pre-masked and scissor_en = 0. The compares are the ROP's (signed CW-bit, as its ports read the 13-bit box).
    function automatic [1:0] sc_mask(input dst_t s, input [CW-1:0] x, input [CW-1:0] y);
        reg signed [CW-1:0] xs, x1s, ys, bx0, bx1, by0, by1; reg in_y;
        xs = x; x1s = x + 1'b1; ys = y; bx0 = s.sx0; bx1 = s.sx1; by0 = s.sy0; by1 = s.sy1;
        in_y = (ys >= by0) && (ys < by1);
        sc_mask = {~s.scissor | ((x1s >= bx0) && (x1s < bx1) && in_y), ~s.scissor | ((xs >= bx0) && (xs < bx1) && in_y)};
    endfunction
    typedef enum logic [1:0] {BM_IDLE, BM_RUN, BM_FLUSH} bm_t; bm_t bm;
    wire        pl_rdy; wire [10:0] pl_rdy_y0;                     // the preload bank holds band pl_rdy_y0
    wire [1:0]  nx_slot = rs_slot + 2'd1;
    wire        nx_ok = (bands_q > 3'd1);                          // the next band is announced
    wire        fast_frame = clear_color_en & ~clear_sc_en & (clear_cmask == 4'hf) &
                             (~ds_en | (clear_depth_en & clear_stencil_en & (clear_smask == 8'hFF)));   // as b_rop's fast
    // the back end's fragments are pixel PAIRS (a quad row): br_x even, br_pv the pixels present,
    // pixel p's fields at index p
    reg         br_valid; reg [1:0] br_pv; reg [CW-1:0] br_x, br_y; reg [103:0] br_c16; reg [51:0] br_sa; reg [47:0] br_z; reg br_front, br_bank;
    reg  [1:0]  br_slot, c1_slot, c2_slot;       // the fragments' band slots (back end -> ROP)
    reg         c1_v, c2_v, bq_v;                // back-end conversion steps / the quad being converted in use
    reg         c1_bank, c2_bank;                // the conversion steps' draw-state banks
    wire        br_ready, rop_ready;
    reg         pk_en, pk_next; reg [CW-1:0] pk_x, pk_y; wire [47:0] pk_z;     // the peek of a pair: {depth x + 1, depth x}
    // the ROP's state follows the fragment at its input: a fragment of the other bank waits until the RMW pipe is empty,
    // then rop_bank switches (fragments arrive in launch order, so every fragment of the old bank is through by then);
    // a fragment of a later band waits until the band manager has started its band
    wire        rop_sw   = br_valid && (br_bank != rop_bank);
    wire        rop_hold = rop_sw || (br_valid && (br_slot != rs_slot || !band_on));
    assign      br_ready = rop_ready && !rop_hold;
    // the band preload in advance: board A walks every band in order, so the band
    // below the one rendering is the next one — the ROP preloads it from the band's start, not from its BAND item (which
    // comes after the band's last quad: a preload then hides only behind the pipeline's drain, and an 8-line band's preload
    // is twice as long). Registered (the ROP multiplies the preload's y0 in its start cycle); the announced band takes
    // over once its BAND item is parsed (the same y0); none below the target's last band.
    reg  [11:0] spn_y0; reg spn_ok;
    always @(posedge clk) begin
        spn_y0 <= 12'(band_y0) + 12'(nl);
        spn_ok <= band_on && (12'(band_y0) + 12'(nl) < rt_height);
    end
    wire        pl_nok = nx_ok || spn_ok;
    wire [10:0] pl_ny0 = nx_ok ? bq_y0[nx_slot] : spn_y0[10:0];
    b_rop #(.CW(CW), .MAXW(MAXW), .NLINES(NLINES), .NLMAX(NLMAX), .CELLS(CELLS), .SAMPLES(1)) u_rop (.clk, .rst(rst_q), .width(rt_width), .nl, .band_y0, .fmt(rt_fmt),
        .color_base(rt_base), .bursts_per_line(rt_bpl), .ds_en, .ds_base, .ds_bpl,
        .band_start, .band_flush_req, .band_flush_done, .band_busy,
        .band_run, .flush_busy, .rmw_idle, .next_ok(pl_nok), .next_y0(pl_ny0), .pl_rdy, .pl_rdy_y0, .pf_clear(go),
        .scissor_en(1'b0), .sc_x0(dsr.sx0), .sc_y0(dsr.sy0), .sc_x1(dsr.sx1), .sc_y1(dsr.sy1),   // the scissor is applied at br (sc_mask)
        .stencil_en(dsr.stencil), .stencil_func(dsr.sfn[1]), .stencil_func_back(dsr.sfn[0]), .stencil_ref(dsr.sref[1]),
        .stencil_ref_back(dsr.sref[0]), .stencil_vmask(dsr.svm[1]), .stencil_vmask_back(dsr.svm[0]),
        .stencil_wmask(dsr.swm[1]), .stencil_wmask_back(dsr.swm[0]), .op_sfail(dsr.osf[1]), .op_dpfail(dsr.odf[1]),
        .op_dppass(dsr.odp[1]), .op_sfail_back(dsr.osf[0]), .op_dpfail_back(dsr.odf[0]), .op_dppass_back(dsr.odp[0]),
        .depth_test_en(dsr.depth_test), .depth_mask(dsr.depth_mask), .depth_func(dsr.depth_func),
        .blend_en(dsr.blend), .blend_src_factor(dsr.bsf), .blend_dst_factor(dsr.bdf), .blend_src_factor_a(dsr.bsfa),
        .blend_dst_factor_a(dsr.bdfa), .blend_op(dsr.bop), .blend_op_a(dsr.bopa), .blend_c16(dsr.bcc), .blend_ca(dsr.bca),
        .color_mask(dsr.cmask), .dither_en(dsr.dither), .clear_c16, .clear_dither, .clear_cmask, .clear_depth, .clear_stencil,
        .clear_color_en, .clear_scissor_en(clear_sc_en), .clear_depth_en, .clear_stencil_en, .clear_smask,
        .clear_sc_x0(clear_x0), .clear_sc_y0(clear_y0), .clear_sc_x1(clear_x1), .clear_sc_y1(clear_y1),
        .lr_en(l_en), .lr_addr(l_addr), .lr_rdy(l_rdy), .lr_rd_valid(l_valid), .lr_rd_data(rd_data),
        .frag_valid(br_valid && !rop_hold), .frag_pv(br_pv), .frag_x(br_x), .frag_y(br_y), .frag_c16(br_c16), .frag_sa(br_sa), .frag_z(br_z), .frag_scov(2'b11),
        .frag_front(br_front), .frag_ready(rop_ready),
        .w_en(fw_en), .w_addr(fw_addr), .w_data(fw_data), .w_mask(fw_mask), .w_wren(fw_wren), .w_wend(fw_wend),
        .w_rdy(fw_rdy), .w_wdf_rdy(fw_wdf_rdy), .pk_en, .pk_next, .pk_x, .pk_y, .pk_z, .frag_addr(br_addr), .frag_addr_n(br_addr_n), .pk_addr,
        .dbg_st(), .dbg_fst(), .dbg_pst());
    assign fw_req = flush_busy;
    always @(posedge clk) begin
        if (rst_q) rop_bank <= 1'b0;
        else if (rop_sw && rmw_idle) rop_bank <= br_bank;
    end

    // ================================================================ in-flight accounting
    // a quad is counted from its collection (QUAD item complete) until it is dropped by early depth or its last pixel
    // went to the ROP; per quad_setup slot until its results left quad_setup; per FS triangle slot until it finished;
    // per draw-state bank (q_live_b) until the bank may be reloaded
    reg  [15:0] q_live, q_live_b [0:1], q_live_s [0:3]; reg [7:0] ts_cnt [0:NTS-1]; reg [5:0] qs_cnt [0:3];
    reg         ev_col; reg [TSW-1:0] ev_col_ts; reg [1:0] ev_col_qs, ev_col_sl; reg ev_col_bk;   // a quad collected
    reg         ev_qso; reg [1:0] ev_qso_qs;                                   // a quad left quad_setup
    reg         ev_fin; reg [TSW-1:0] ev_fin_ts; reg ev_fin_bk; reg [1:0] ev_fin_sl;     // a quad finished (back end)
    reg         ev_drop; reg [TSW-1:0] ev_drop_ts; reg ev_drop_bk; reg [1:0] ev_drop_sl;  // a quad dropped (early depth)
    reg         ev_edrop; reg [TSW-1:0] ev_edrop_ts; reg [1:0] ev_edrop_qs, ev_edrop_sl; reg ev_edrop_bk, ev_edrop_fr;   // dropped in
                                                                                   // the early queue (never in quad_setup)
    // q_live_s[i] == 0 as a register of the counter's next value (timing: rs_slot -> the 16-bit 4 : 1 select -> the zero
    // compare -> bm_go_flush -> pk_block -> the peek's address select is too long for one cycle); exact at every cycle.
    // It also says "no quad of slot i in the QUAD pipeline": a QUAD taken at cycle t is pushed into
    // the early queue at t + 3 and counted in q_live_s from t + 5, while a BAND item right behind it announces the next band
    // (nx_ok) at t + 3 and a FRAME_END sets fend at t + 3 — otherwise the band is flushed with its last quad in the pipe, the
    // quad then waits in the early queue for a band that is gone and the sequencer for its bank (a stall; tb_b_render
    // 800 x 160 seeds 2 / 4 / 5 cover it).
    // qp_sn[i]: a quad of slot i is in a pipe stage (qp_v[0..3]) in the NEXT cycle — the stages only shift.
    reg  [3:0]  qz_s;
    reg  [NTS-1:0] tsz;                          // fragment-core triangle slot i has no live quad (ts_cnt[i] == 0, registered)
    wire        qp_in;
    wire [3:0]  qp_sn;
    always @(posedge clk) begin
        if (rst_q) begin
            q_live <= 16'd0; q_live_b[0] <= 16'd0; q_live_b[1] <= 16'd0;
            for (int i = 0; i < 4; i++) q_live_s[i] <= 16'd0;
            qz_s <= 4'hF; tsz <= '1;
            for (int i = 0; i < NTS; i++) ts_cnt[i] <= 8'd0;
            for (int i = 0; i < 4; i++) qs_cnt[i] <= 6'd0;
        end else begin
            q_live <= q_live + 16'(ev_col) - 16'(ev_fin) - 16'(ev_drop) - 16'(ev_edrop);
            for (int i = 0; i < 2; i++)
                q_live_b[i] <= q_live_b[i] + 16'(ev_col && ev_col_bk == 1'(i)) - 16'(ev_fin && ev_fin_bk == 1'(i))
                                           - 16'(ev_drop && ev_drop_bk == 1'(i)) - 16'(ev_edrop && ev_edrop_bk == 1'(i));
            for (int i = 0; i < 4; i++) begin : QLS
                reg [15:0] nq;
                nq = q_live_s[i] + 16'(ev_col && ev_col_sl == 2'(i)) - 16'(ev_fin && ev_fin_sl == 2'(i))
                                 - 16'(ev_drop && ev_drop_sl == 2'(i)) - 16'(ev_edrop && ev_edrop_sl == 2'(i));
                q_live_s[i] <= nq;
                qz_s[i] <= (nq == 16'd0) && !qp_sn[i];
            end
            for (int i = 0; i < NTS; i++) begin : TSC             // tsz[i] = ts_cnt[i] == 0, a register of the next value (as qz_s:
                reg [7:0] nt;                                     //   M_TSWAIT reads one flop instead of a 16 : 1 select + compare)
                nt = ts_cnt[i] + 8'(ev_col && ev_col_ts == TSW'(i)) - 8'(ev_fin && ev_fin_ts == TSW'(i))
                               - 8'(ev_drop && ev_drop_ts == TSW'(i)) - 8'(ev_edrop && ev_edrop_ts == TSW'(i));
                ts_cnt[i] <= nt; tsz[i] <= (nt == 8'd0);
            end
            for (int i = 0; i < 4; i++)
                qs_cnt[i] <= qs_cnt[i] + 6'(ev_col && ev_col_qs == 2'(i)) - 6'(ev_qso && ev_qso_qs == 2'(i))
                                       - 6'(ev_edrop && ev_edrop_qs == 2'(i));
        end
    end
    // the bank ~db may be reloaded: no quad of its draw alive, none of its fragments between the back end and the ROP (a
    // quad stops counting when its last row enters the conversion steps), and none in the ROP's pipe: a pipe holds one bank,
    // so that means the ROP is on the other bank or empty. ("The ROP is on bank db" is not enough with the bank
    // reuse: after draws A B A, db is back on A while B's fragments wait behind a stalled ROP, and the next draw's
    // bucket would overwrite their state: wrong blend / depth / stencil on a few pixels; +BESTALL in tb_b_render covers it.)
    wire tail_ld   = (c1_v && c1_bank == ld_bank) || (c2_v && c2_bank == ld_bank) || (br_valid && br_bank == ld_bank);
    wire bank_free = (q_live_b[ld_bank] == 16'd0) && !tail_ld && ((rop_bank != ld_bank) || rmw_idle);
    // the current band may be flushed: none of its quads alive, none of its fragments behind the core (b_rop itself waits
    // for its RMW pipe)
    wire tail_cur = (c1_v && c1_slot == rs_slot) || (c2_v && c2_slot == rs_slot) || (br_valid && br_slot == rs_slot);
    wire bm_go_flush = (bm == BM_RUN) && (nx_ok || fend) && qz_s[rs_slot] && !tail_cur;

    // ================================================================ item sequencer
    typedef enum logic [4:0] {M_IDLE, M_ITEM, M_TRI, M_TSWAIT, M_TRIVAR, M_TRIEND, M_DSWAIT,
                              M_DSREAD, M_UNI, M_PROG, M_TRIQS, M_BAND, M_FEND,
                              M_FDONE, M_DISC, M_DSKIP} ms_t;
    ms_t ms;
    reg [7:0] dsk_n;                             // beats left to skip in a discarded item
    assign busy = (ms != M_IDLE);
    // an abort with `go` in the same cycle ends that frame at once (the command processor decided to start it in the cycle
    // before the wiping SYNC arrived, so its stream is gone); outside a frame an abort has nothing to end — the command
    // processor drops every FLUSH older than the SYNC
    wire abort_take = abort && (ms != M_IDLE || go) && ms != M_FEND && ms != M_FDONE;
    wire bq_push = (ms == M_BAND) && (bands_q != 3'd4) && !abort_take;       // a band announced
    wire bq_pop  = (bm == BM_FLUSH) && band_flush_done;                     // a band flushed
    // the QUAD pipeline: a QUAD item (one beat) is taken in M_ITEM into stage 0; stage 1 multiplies the
    // current triangle's steps by the quad's position (2 qx, 2 qy), stage 2 registers the products (the DSPs' output
    // register), stage 3 adds the reference values, stage 4 pushes the quad with its edge values into the early queue and
    // the bounds lane. One quad per cycle; the parser goes on meanwhile. The triangle's steps and references are the
    // CURRENT one's (cur_st / cur_er, set at M_TRIQS, which waits for this pipe to drain).
    reg          qp_v [0:3]; reg [3:0] qp_cov [0:3]; reg [11:0] qp_qx [0:3], qp_qy [0:3]; reg [TSW+6-1:0] qp_m [0:3];
    reg signed [31:0] qp_pa [0:2], qp_pb [0:2];     // stage 1 -> 2: the products A_k (2 qx), B_k (2 qy) (low 32 bits)
    reg signed [31:0] qp_e [0:2];                   // stage 2 -> 3: the sums
    wire [2:0]  qp_cnt = 3'(qp_v[0]) + 3'(qp_v[1]) + 3'(qp_v[2]) + 3'(qp_v[3]);
    wire        qp_busy = qp_v[0] | qp_v[1] | qp_v[2] | qp_v[3];
    wire        zl_in;                              // a quad leaves the pipe: the lane's input and the early queue's write (below)
    reg signed [31:0] cur_st [0:5], cur_er [0:2]; reg signed [31:0] t_er [0:2];   // the current TRI's steps / references; the one being parsed
    // the current triangle's depth-bound words for the lane (w18..w21; ok cleared under polygon offset, which the bounds
    // leave out — such quads take the late test only)
    reg  [127:0] cur_zq;
    reg  [5:0]  zl_sr;                                      // a quad in each stage of the lane (below)
    wire        zl_busy = |zl_sr;
    // the triangle being parsed / the current one
    reg  [5:0]  tb; reg [7:0] t_mask, t_rem; reg [2:0] t_vb; reg [255:0] t_vbuf; reg [19:0] t_id; reg [12:0] t_draw;
    wire        t_lastv = ((t_rem & (t_rem - 8'd1)) == 8'd0);   // the varying being taken is the TRI's last
    reg  [127:0] t_zq;                            // the TRI's words w18..w21
    assign bk_swap = draw_ok && (t_draw != active_draw) && bk_ok[~db] && (bk_draw[~db] == t_draw);
    reg         t_front; reg [TSW-1:0] t_ts;
    reg  [19:0] cur_tri; reg cur_ok; reg [TSW-1:0] cur_ts; reg [1:0] cur_qs; reg cur_front;
    // bucket / program load
    reg  [255:0] ds_b0; reg [2:0] ds_i; reg [6:0] un_i; reg un_h; reg [11:0] pg_n; reg [1:0] pg_w;
    wire [2:0]  t_lsb = t_rem[0]?3'd0:t_rem[1]?3'd1:t_rem[2]?3'd2:t_rem[3]?3'd3:t_rem[4]?3'd4:t_rem[5]?3'd5:t_rem[6]?3'd6:3'd7;
    wire        quad_blk = ((6'(eq_cnt) + 6'(qp_cnt)) >= 6'(EQD)) && (w0[31:28] == 4'd1);   // a QUAD waits for room in the early queue (the pipe counted)
    reg  [10:0] nb_y0;                                                // the next band's y0 (band_y0 changes at its start)
    // pixel issuer handshake (declared here, driven below). The issuer takes the early queue's head once its verdict is in:
    // a rejected quad is dropped (e_drop), any other goes to quad_setup when the quad path has room; the
    // next quad is taken in the cycle the issuer sends the last pixel of the previous one (4 cycles per quad).
    // +EZAUDIT (simulation): rejected quads are not dropped but checked against the exact test at the collector.
    reg         ez_audit = 1'b0;
    // synthesis translate_off
    initial ez_audit = $test$plusargs("EZAUDIT");
    // synthesis translate_on
    reg         pi_run; reg [1:0] pi_n; reg signed [31:0] q_e_h [0:2]; reg [11:0] q_qx_h, q_qy_h;
    wire [3:0]  eq_h  = eq_rp[3:0];
    wire        eq_hv = (eq_cnt != 5'd0) && eq_vd[eq_h];
    wire        e_drop = eq_hv && eq_rej[eq_h] && !ez_audit;
    wire        q_take = eq_hv && (!eq_rej[eq_h] || ez_audit) && (!pi_run || pi_n == 2'd3) &&
                         ((6'(qs_infl) + 6'(qf_cnt)) < 6'(QFD));
    // TRI varyings in pairs: with the two head beats both valid, a pair is taken unless the second beat would be the
    // next item's (t_vb == 4 of the last varying: the item ends with this beat); a pair that completes a varying (t_vb 3 or
    // 4) needs the core's writer, as a single completing beat does
    wire        tv_pair = sb_valid && sb2_valid && !(t_vb == 3'd4 && t_lastv);
    wire        tv_ok  = tv_pair ? (t_vb < 3'd3 || fs_tri_ready) : (sb_valid && (t_vb != 3'd4 || fs_tri_ready));
    always @* begin
        sb_take2 = 1'b0;
        case (ms)
            M_ITEM:   sb_take = sb_valid && !quad_blk;
            M_TRI:    begin sb_take = 1'b0; sb_take2 = sb_valid && sb2_valid; end      // the header's 6 pairs
            M_TRIVAR: begin sb_take = tv_ok && !tv_pair; sb_take2 = tv_ok && tv_pair; end
            M_DISC, M_DSKIP: sb_take = sb_valid;             // a discarded frame: every beat taken, nothing done
            default:  sb_take = 1'b0;
        endcase
    end

    always @(posedge clk) begin
        if (rst_q) begin
            ms <= M_IDLE; done <= 1'b0;
            eq_wp <= 5'd0; qs_we <= 1'b0; fs_tri_we <= 1'b0; fe_go <= 1'b0; band_y0 <= 11'd0;
            band_on <= 1'b0; band_start <= 1'b0; band_flush_req <= 1'b0; cur_ok <= 1'b0; draw_ok <= 1'b0;
            prog_ok <= 2'b00; wfd <= 2'b00; ubi <= 2'b00; db <= 1'b0; cur_ts <= '0; cur_qs <= 2'd0; ld_iram_we <= 1'b0; ld_par_we <= 1'b0; tex_inval <= 1'b0;
            ld_wfd <= 1'b0; ld_ubi <= 1'b0; tsk <= 2'b00; tk_start <= 1'b0;
            bk_ok <= 2'b00;
            c_tris <= 0; c_frames <= 0; c_errs <= 0; ev_col <= 1'b0;
            ds[0] <= '0; ds[1] <= '0; ds[0].cmask <= 4'hF; ds[1].cmask <= 4'hF;
            bands_q <= 3'd0; nq_slot <= 2'd3; rs_slot <= 2'd0; bm <= BM_IDLE; fend <= 1'b0;
        end else begin
            done <= 1'b0; qs_we <= 1'b0; fs_tri_we <= 1'b0; band_start <= 1'b0; ld_iram_we <= 1'b0; ld_par_we <= 1'b0;
            tex_inval <= 1'b0; fe_go <= 1'b0; ev_col <= 1'b0; tk_start <= 1'b0;
            // a QUAD leaves the quad pipeline into the early queue: the entry is written beside the lane's input
            if (zl_in) begin
                eq_wp <= eq_wp + 5'd1;
                ev_col <= 1'b1; ev_col_ts <= qp_m[2][TSW+6-1 -: TSW]; ev_col_qs <= qp_m[2][5:4]; ev_col_bk <= qp_m[2][2]; ev_col_sl <= qp_m[2][1:0];
            end
            bands_q <= bands_q + 3'(bq_push) - 3'(bq_pop);
            // ---- band manager: the announced bands in order; the sequencer never waits for a band ----
            case (bm)
                BM_IDLE: if (bands_q != 3'd0 && !band_busy && !band_on && !band_flush_req) begin
                    band_y0 <= bq_y0[rs_slot]; band_start <= 1'b1; band_on <= 1'b1; bm <= BM_RUN;
                end
                BM_RUN: if (bm_go_flush) begin band_flush_req <= 1'b1; bm <= BM_FLUSH; end
                BM_FLUSH: if (band_flush_done) begin band_flush_req <= 1'b0; band_on <= 1'b0; rs_slot <= rs_slot + 2'd1; bm <= BM_IDLE; end
                default: bm <= BM_IDLE;
            endcase
            if (abort_take) begin
                ms <= M_FEND;                                    // the frame ends here: the bands finish and flush, done
            end else
            case (ms)
                // a new frame: the host may have rewritten any bucket, program buffer or image since the last FLUSH
                // (DRAW_STATE / WRITE_BUFFER / WRITE_IMAGE are serialized with it), so the cached draw / program are
                // forgotten and the texture cache invalidated once here (otherwise a frame whose first
                // draw id equals the previous frame's active one keeps the stale state)
                M_IDLE: if (go) begin ms <= discard ? M_DISC : M_ITEM; draw_ok <= 1'b0; prog_ok <= 2'b00; bk_ok <= 2'b00; tex_inval <= 1'b1; fend <= 1'b0; end
                // ---------------------------------------------------------------- a discarded frame (the FLUSH was rejected):
                // every FLUSH has exactly one frame in the render stream, so it is consumed up to its FRAME_END and dropped
                // — otherwise its beats would sit in the render buffer in front of the next frame's
                M_DISC: if (sb_valid) begin
                    case (w0[31:28])
                        4'd2: begin dsk_n <= 8'd12 + 8'd5 * 8'($countones(w0[15:8])); ms <= M_DSKIP; end  // TRI: 12 + 5 nv more (a QUAD: none)
                        4'd5: ms <= M_FDONE;                                                          // FRAME_END
                        default: ;                                                                    // TRISEL, BAND, NOP
                    endcase
                end
                M_DSKIP: if (sb_valid) begin dsk_n <= dsk_n - 8'd1; if (dsk_n == 8'd1) ms <= M_DISC; end
                // ---------------------------------------------------------------- dispatch on the item's first beat
                M_ITEM: if (sb_valid && !quad_blk) begin
                    case (w0[31:28])
                        4'd1: ;                                                          // QUAD: into the quad pipeline (below), no state
                        4'd2: begin t_mask <= w0[15:8]; t_rem <= w0[15:8]; t_front <= w0[25]; t_id <= w1[19:0];
                                    tb <= 6'd1; t_ts <= cur_ts + 1'b1;
                                    ms <= M_TRI; end
                        4'd3: if (!(cur_ok && w1[19:0] == cur_tri)) c_errs <= c_errs + 1;   // no record store: only the current one
                        4'd4: begin nb_y0 <= w0[10:0]; ms <= M_BAND; end
                        4'd5: ms <= M_FEND;
                        4'd0: ;
                        default: c_errs <= c_errs + 1;
                    endcase
                end
                // ---------------------------------------------------------------- TRI: beats 1..12 in pairs (draw, R, scalars,
                // steps, depth bounds, the reference edge values); tb = the pair's first beat (1, 3, .., 11)
                M_TRI: if (sb_valid && sb2_valid) begin
                    case (tb)
                        6'd1: begin t_draw <= w0[12:0]; t_R <= w1[23:0]; t_sc[0] <= w2[23:0]; t_sc[1] <= w3[23:0]; end
                        6'd3: begin t_sc[2] <= w0[23:0]; t_sc[3] <= w1[23:0]; t_sc[4] <= w2[23:0]; t_sc[5] <= w3[23:0]; end
                        6'd5: begin t_sc[6] <= w0[23:0]; t_sc[7] <= w1[23:0]; t_st[0] <= w2; t_st[1] <= w3; end
                        6'd7: begin t_st[2] <= w0; t_st[3] <= w1; t_st[4] <= w2; t_st[5] <= w3; end
                        6'd9: t_zq <= {sb2_data, sb_data};                                   // w18 .. w21
                        default: begin t_er[0] <= w0; t_er[1] <= w1; t_er[2] <= w2; end     // w22 .. w24 (w25 = 0)
                    endcase
                    tb <= tb + 6'd2;
                    if (tb == 6'd11) begin t_vb <= 3'd0; ms <= (t_mask == 8'd0) ? M_TRIEND : M_TSWAIT; end
                end
                M_TSWAIT: if (tsz[t_ts]) ms <= M_TRIVAR;                   // the triangle slot is free (no live quad of its last triangle)
                // ---------------------------------------------------------------- TRI: varyings, 5 beats each, one or two beats a cycle
                M_TRIVAR: if (tv_ok) begin
                    if (!tv_pair) begin                                              // one beat
                        if (t_vb != 3'd4) t_vbuf[t_vb*64 +: 64] <= sb_data;
                        t_vb <= t_vb + 3'd1;
                        if (t_vb == 3'd4) begin
                            fs_tri_we <= 1'b1; fs_tri_slot <= t_ts; fs_tri_vidx <= t_lsb;
                            fs_tri_data <= {sb_data[31:0], t_vbuf};
                            t_rem <= t_rem & (t_rem - 8'd1); t_vb <= 3'd0;
                            if (t_lastv) ms <= M_TRIEND;
                        end
                    end else if (t_vb < 3'd3) begin                                   // beats vb, vb + 1 into the buffer
                        t_vbuf[t_vb*64 +: 64] <= sb_data; t_vbuf[(t_vb+3'd1)*64 +: 64] <= sb2_data;
                        t_vb <= t_vb + 3'd2;
                    end else if (t_vb == 3'd3) begin                                  // beat 3 and the varying's last beat
                        fs_tri_we <= 1'b1; fs_tri_slot <= t_ts; fs_tri_vidx <= t_lsb;
                        fs_tri_data <= {sb2_data[31:0], sb_data, t_vbuf[191:0]};
                        t_rem <= t_rem & (t_rem - 8'd1); t_vb <= 3'd0;
                        if (t_lastv) ms <= M_TRIEND;
                    end else begin                                                    // the last beat, and the next varying's first
                        fs_tri_we <= 1'b1; fs_tri_slot <= t_ts; fs_tri_vidx <= t_lsb;
                        fs_tri_data <= {sb_data[31:0], t_vbuf};
                        t_rem <= t_rem & (t_rem - 8'd1);
                        t_vbuf[63:0] <= sb2_data; t_vb <= 3'd1;                       // (never the last varying: tv_pair excludes it)
                    end
                end
                // ---------------------------------------------------------------- TRI end
                M_TRIEND: begin
                    c_tris <= c_tris + 1;
                    ms <= ((draw_ok && t_draw == active_draw) || bk_swap) ? M_TRIQS : M_DSWAIT;
                    if (bk_swap) begin active_draw <= t_draw; db <= ~db; end   // the other bank holds this draw: no load
                end
                // ---------------------------------------------------------------- per-draw load into the other bank:
                // no drain — the active draw's quads keep flowing; only the bank's previous draw must be gone
                M_DSWAIT: if (bank_free) begin
                    fe_go <= 1'b1; fe_a <= DS_BASE + {t_draw, 9'd0}; fe_n <= 10'd6; ds_i <= 3'd0; ms <= M_DSREAD;
                end
                M_DSREAD: if (rf_nonempty && !fe_go) begin
                    ds_i <= ds_i + 3'd1;
                    if (ds_i == 3'd0) ds_b0 <= rf_head;
                    else if (ds_i == 3'd1) begin : UNPACK
                        reg [511:0] s; dst_t n; s = {rf_head, ds_b0};
                        n.depth_test = s[0]; n.depth_mask = s[1]; n.depth_func = s[4:2]; n.stencil = s[5];
                        n.blend = s[6]; n.dither = s[7]; n.scissor = s[8]; n.po = s[9]; n.cmask = s[19:16];
                        n.sref[1] = s[32 +: 8]; n.svm[1] = s[40 +: 8]; n.swm[1] = s[48 +: 8]; n.sfn[1] = s[56 +: 3];
                        n.sref[0] = s[64 +: 8]; n.svm[0] = s[72 +: 8]; n.swm[0] = s[80 +: 8]; n.sfn[0] = s[88 +: 3];
                        n.osf[1] = s[96 +: 3]; n.odf[1] = s[99 +: 3]; n.odp[1] = s[102 +: 3];
                        n.osf[0] = s[112 +: 3]; n.odf[0] = s[115 +: 3]; n.odp[0] = s[118 +: 3];
                        n.bsf = s[128 +: 4]; n.bdf = s[132 +: 4]; n.bsfa = s[136 +: 4]; n.bdfa = s[140 +: 4];
                        n.bop = s[144 +: 3]; n.bopa = s[148 +: 3];
                        n.bcc = {s[160 +: 13], s[173 +: 13], s[192 +: 13], s[205 +: 13]};
                        n.sx0 = s[224 +: 13]; n.sy0 = s[240 +: 13]; n.sx1 = s[256 +: 13]; n.sy1 = s[272 +: 13];
                        n.pofac = s[288 +: 24]; n.pounits = s[320 +: 24];
                        n.prog = s[352 +: 29]; n.plen = s[384 +: 12]; n.nuni = s[400 +: 7];
                        n.bca = {s[429 +: 13], s[416 +: 13]};                    // word 13: {Ca in G units, Ca in R/B units}
                        ds[ld_bank] <= n;
                    end else begin
                        // samplers 2 (ds_i - 2) and 2 (ds_i - 2) + 1: the even one into s_even, the odd one into s_odd, at {bank, k[3:1]}
                        s_even[{ld_bank, 3'(int'(ds_i) - 2)}] <= samp_word(rf_head[0 +: 128]);
                        s_odd [{ld_bank, 3'(int'(ds_i) - 2)}] <= samp_word(rf_head[128 +: 128]);
                        if (ds_i == 3'd5) begin
                            fe_go <= 1'b1; fe_a <= DS_BASE + {t_draw, 9'd0} + 29'd48;
                            fe_n <= 10'((dsl.nuni + 7'd1) >> 1); un_i <= 7'd0; un_h <= 1'b0; ms <= M_UNI;
                        end
                    end
                end
                // uniforms: one per cycle, two per burst
                M_UNI: if (!fe_go) begin
                    if (un_i == dsl.nuni) begin
                        if (fe_done) begin
                            if (prog_ok[ld_bank] && ld_prog[ld_bank] == dsl.prog && ld_plen[ld_bank] == dsl.plen) begin
                                draw_ok <= 1'b1; active_draw <= t_draw; db <= ld_bank; ms <= M_TRIQS;
                                bk_ok[ld_bank] <= 1'b1; bk_draw[ld_bank] <= t_draw;
                            end else begin                                 // 64-bit instructions, 4 per burst
                                fe_go <= 1'b1; fe_a <= dsl.prog; fe_n <= 10'((dsl.plen + 12'd3) >> 2);
                                pg_n <= 12'd0; pg_w <= 2'd0; ld_wfd <= 1'b0; ld_ubi <= 1'b0; tk_start <= 1'b1; ms <= M_PROG;
                            end
                        end
                    end else if (rf_nonempty) begin
                        ld_par_we <= 1'b1; ld_par_idx <= un_i[5:0];
                        ld_par_data <= {rf_head[un_h*128+96 +: 24], rf_head[un_h*128+64 +: 24],
                                        rf_head[un_h*128+32 +: 24], rf_head[un_h*128 +: 24]};
                        un_i <= un_i + 7'd1; un_h <= ~un_h;
                    end
                end
                // program: four 64-bit instructions per burst; STOZ (op 21) anywhere -> the program writes gl_FragDepth, LDV
                // (op 17) of a built-in (word bit 3) anywhere -> its warps load all 4 launch rows. The words also pass the
                // TEX_SKIP scan one cycle after they are taken (tk_rdy: its table is clear; tk_badn: the last word's verdict)
                M_PROG: if (!fe_go) begin
                    if (pg_n == dsl.plen) begin
                        if (fe_done) begin
                            prog_ok[ld_bank] <= 1'b1; ld_prog[ld_bank] <= dsl.prog; ld_plen[ld_bank] <= dsl.plen; wfd[ld_bank] <= ld_wfd; ubi[ld_bank] <= ld_ubi;
                            tsk[ld_bank] <= ~(tk_bad | tk_badn);
                            draw_ok <= 1'b1; active_draw <= t_draw; db <= ld_bank; ms <= M_TRIQS;
                            bk_ok[ld_bank] <= 1'b1; bk_draw[ld_bank] <= t_draw;
                        end
                    end else if (rf_nonempty && tk_rdy) begin
                        ld_iram_we <= 1'b1; ld_iram_addr <= pg_n; ld_iram_data <= rf_head[pg_w*64 +: 64];
                        if (rf_head[pg_w*64 + 57 +: 7] == 7'd21) ld_wfd <= 1'b1;
                        if (rf_head[pg_w*64 + 57 +: 7] == 7'd17 && rf_head[pg_w*64 + 3]) ld_ubi <= 1'b1;
                        pg_n <= pg_n + 12'd1; pg_w <= pg_w + 2'd1;
                    end
                end
                // ---------------------------------------------------------------- the triangle becomes current
                M_TRIQS: if (qs_cnt[cur_qs + 2'd1] == 6'd0 && !zl_busy && !qp_busy) begin   // (the quad pipe too: it reads cur_st / cur_er)
                    qs_we <= 1'b1; qs_slot <= cur_qs + 2'd1; cur_qs <= cur_qs + 2'd1;
                    cur_ts <= t_ts; cur_tri <= t_id; cur_ok <= 1'b1; cur_front <= t_front;
                    cur_zq <= {t_zq[127:32], t_zq[31] & ~ds[db].po, t_zq[30:0]};
                    for (int k = 0; k < 6; k++) cur_st[k] <= t_st[k];
                    for (int k = 0; k < 3; k++) cur_er[k] <= t_er[k];
                    ms <= M_ITEM;
                end
                // ---------------------------------------------------------------- BAND / FRAME_END
                // a BAND item only announces the band (slot nq_slot + 1); the band manager renders the slots in order,
                // the sequencer waits only while all 4 slots are announced
                M_BAND: if (bq_push) begin bq_y0[nq_slot + 2'd1] <= nb_y0; nq_slot <= nq_slot + 2'd1; ms <= M_ITEM; end
                M_FEND: begin fend <= 1'b1; if (bands_q == 3'd0 && bm == BM_IDLE) ms <= M_FDONE; end   // every band flushed
                M_FDONE: if (!flush_busy) begin done <= 1'b1; c_frames <= c_frames + 1; ms <= M_IDLE; end
                default: ms <= M_IDLE;
            endcase
        end
    end
    // synthesis translate_off
    always @(posedge clk) if (!rst_q && ms == M_DSREAD && ds_i == 3'd1 && rf_nonempty && !fe_go &&
                              (q_live_b[ld_bank] != 16'd0 || tail_ld || (rop_bank == ld_bank && !u_rop.pipe_empty)))
        $fatal(1, "b_render: the state of bank %0d reloaded under its fragments", ld_bank);
    // synthesis translate_on
    // ================================================================ TEX_SKIP: may a program leave uncovered pixels unsampled?
    // A quad's uncovered pixels run the program too (their values feed DDX / DDY and the texture unit's level of detail),
    // and their own results are thrown away. With tsk set for a bank, the texture unit returns zeros instead of texels
    // for them (tmu24 req_mask) — which a covered pixel must not be able to see. The scan follows, per register
    // component, "this value may differ in an uncovered lane" (tk_t; set by a TEX, inherited through every operation,
    // cleared by an unconditional write from clean sources) over the program's words in order, and refuses the program
    // (tk_bad) when such a value reaches another lane: DDX / DDY of it, a TEX's coordinates, or anything this scan does
    // not follow (loops, BRA, an unknown opcode). IF / ELSE / ENDIF are followed: under an IF whose predicate may
    // differ (tk_ct) every write is marked, and a conditional write never clears a mark. DISCARD only kills its lane.
    // The table is cleared at the start of each scan (32 cycles, beside the program's DDR fetch; tk_rdy holds the words).
    localparam [63:0] TK_N0 = 64'h8000_0000_3FD3_0005,       // no register source: NOP MOVI LDP LDV END, the control ops
                      TK_N1 = 64'h0180_3F07_C028_7802,       // one: MOV STO STOZ RCP RSQRT LOG2 EXP2 FTOI ITOF DDX DDY TEX SIN COS TRUNC FLR CEIL RNDE FTOU UTOF
                      TK_N3 = 64'h0000_4000_0000_0040,       // three: MAD SEL (every other operation: two)
                      TK_WR = 64'h01F7_FFFF_C003_7FFE,       // writes rd.wm
                      TK_NG = 64'h7E08_0000_18D4_0000;       // not followed: STV BRA LOOP ENDL BREAK CONT IOP, 57..62
    (* ram_style = "distributed" *) reg [3:0] tk_t [0:31];
    reg  tk_clr, tk_ct, tk_tp; reg [4:0] tk_cc; reg [3:0] tk_ifd;
    assign tk_rdy = ~tk_clr & ~tk_start;
    wire [63:0] tw = ld_iram_data;
    wire [5:0]  tw_op = tw[62:57]; wire [3:0] tw_wm = tw[56:53]; wire [4:0] tw_rd = tw[52:48];
    wire        tw_tex = (tw[63:57] == 7'd34), tw_setp = (tw[63:57] == 7'd15), tw_if = (tw[63:57] == 7'd24), tw_endif = (tw[63:57] == 7'd26);
    wire        tw_dd = (tw[63:57] == 7'd32) | (tw[63:57] == 7'd33);
    // three read ports (the sources) and the write port, whose own read gives rd's marks (one RAM32M per 2 bits: the
    // fourth read must be AT the write address, so it follows the clear's address too — unused then)
    wire [4:0]  tk_wa = tk_clr ? tk_cc : tw_rd;
    wire [3:0]  tk_o0 = tk_t[tw[47:43]], tk_o1 = tk_t[tw[32:28]], tk_o2 = tk_t[tw[17:13]], tk_od = tk_t[tk_wa];
    // the components an operation reads of each source: by the written component's swizzle, except DP3 / DP4 (positions
    // 0..2 / 0..3), TEX (its coordinate set, bits [9:6]; 0 = all four) and SETP (one micro-op: wm's first component)
    reg  [3:0]  tw_rs;
    always @* begin
        tw_rs = tw_wm;
        if (tw[63:57] == 7'd9) tw_rs = 4'b0111;
        else if (tw[63:57] == 7'd10) tw_rs = 4'b1111;
        else if (tw_tex) tw_rs = (tw[9:6] == 4'd0) ? 4'hF : tw[9:6];
        else if (tw_setp) tw_rs = tw_wm[0] ? 4'b0001 : tw_wm[1] ? 4'b0010 : tw_wm[2] ? 4'b0100 : tw_wm[3] ? 4'b1000 : 4'b0001;
    end
    function automatic tk_rd(input [3:0] t, input [7:0] swz, input [3:0] rs);
        tk_rd = (rs[0] & t[swz[1:0]]) | (rs[1] & t[swz[3:2]]) | (rs[2] & t[swz[5:4]]) | (rs[3] & t[swz[7:6]]);
    endfunction
    wire        tk_s0 = ~TK_N0[tw_op] & tk_rd(tk_o0, tw[42:35], tw_rs);
    wire        tk_src = tk_s0 | (~TK_N0[tw_op] & ~TK_N1[tw_op] & tk_rd(tk_o1, tw[27:20], tw_rs)) | (TK_N3[tw_op] & tk_rd(tk_o2, tw[12:5], tw_rs));
    wire        tk_cond = (tk_ifd != 4'd0);                  // the word runs under an IF: a lane may not execute it
    assign      tk_badn = ld_iram_we & (tw[63] | TK_NG[tw_op] | ((tw_dd | tw_tex) & tk_s0) |
                                        (tw_if & (tk_ifd == 4'd15)) | (tw_endif & ~tk_cond));
    wire        tk_new = tk_src | tw_tex | tk_ct;
    wire        tk_we = tk_clr | (ld_iram_we & ~tw[63] & TK_WR[tw_op]);
    reg  [3:0]  tk_wd;
    always @* for (int c = 0; c < 4; c++) tk_wd[c] = ~tk_clr & (tw_wm[c] ? (tk_new | (tk_cond & tk_od[c])) : tk_od[c]);
    always @(posedge clk) if (tk_we) tk_t[tk_wa] <= tk_wd;
    always @(posedge clk) begin
        if (rst_q) begin tk_clr <= 1'b0; tk_bad <= 1'b0; tk_ct <= 1'b0; tk_tp <= 1'b0; tk_ifd <= 4'd0; end
        else if (tk_start) begin tk_clr <= 1'b1; tk_cc <= 5'd0; tk_bad <= 1'b0; tk_ct <= 1'b0; tk_tp <= 1'b0; tk_ifd <= 4'd0; end
        else begin
            if (tk_clr) begin tk_cc <= tk_cc + 5'd1; if (tk_cc == 5'd31) tk_clr <= 1'b0; end
            if (ld_iram_we) begin
                if (tk_badn) tk_bad <= 1'b1;
                if (tw_setp) tk_tp <= tk_src | tk_ct | (tk_cond & tk_tp);
                if (tw_if) begin tk_ifd <= tk_ifd + 4'd1; if (tk_tp) tk_ct <= 1'b1; end
                if (tw_endif) begin tk_ifd <= tk_ifd - 4'd1; if (tk_ifd == 4'd1) tk_ct <= 1'b0; end
            end
        end
    end
    // synthesis translate_off
    always @(posedge clk) if (!rst_q && ld_iram_we && tk_clr) $fatal(1, "b_render: a program word reached the B4 scan while its table is being cleared");
    // synthesis translate_on

    // pops of the fetch FIFO: bucket bursts, uniforms (2 per burst), program words
    always @* begin
        rf_pop = 1'b0;
        if (ms == M_DSREAD && rf_nonempty && !fe_go) rf_pop = 1'b1;
        else if (ms == M_UNI && !fe_go && un_i != dsl.nuni && rf_nonempty && un_h) rf_pop = 1'b1;
        else if (ms == M_UNI && !fe_go && un_i == dsl.nuni && rf_nonempty) rf_pop = 1'b1;   // an odd count's unused half
        else if (ms == M_PROG && !fe_go && pg_n != dsl.plen && rf_nonempty && tk_rdy && (pg_w == 2'd3 || pg_n + 12'd1 == dsl.plen))
            rf_pop = 1'b1;
        else if (ms == M_PROG && !fe_go && pg_n == dsl.plen && rf_nonempty) rf_pop = 1'b1;
    end

    // ================================================================ the QUAD pipeline: position -> edge values -> early queue
    assign      qp_in = (ms == M_ITEM) && sb_valid && !quad_blk && (w0[31:28] == 4'd1);
    for (genvar i = 0; i < 4; i++) begin : QPSN
        assign qp_sn[i] = (qp_in && nq_slot == 2'(i)) || (qp_v[0] && qp_m[0][1:0] == 2'(i)) ||
                          (qp_v[1] && qp_m[1][1:0] == 2'(i)) || (qp_v[2] && qp_m[2][1:0] == 2'(i));
    end
    always @(posedge clk) begin
        if (rst_q) begin for (int i = 0; i < 4; i++) qp_v[i] <= 1'b0; end
        else begin
            // stage 0: the item's words and the quad's metadata (the current triangle's slots, the active bank, the band slot)
            qp_v[0] <= qp_in;
            if (qp_in) begin qp_cov[0] <= w0[27:24]; qp_qy[0] <= w0[23:12]; qp_qx[0] <= w0[11:0]; qp_m[0] <= {cur_ts, cur_qs, cur_front, db, nq_slot}; end
            // stage 1: the products (DSP; the result registered again in stage 2)
            qp_v[1] <= qp_v[0];
            if (qp_v[0]) begin
                for (int k = 0; k < 3; k++) begin
                    qp_pa[k] <= 32'(cur_st[2*k]   * $signed({1'b0, qp_qx[0], 1'b0}));
                    qp_pb[k] <= 32'(cur_st[2*k+1] * $signed({1'b0, qp_qy[0], 1'b0}));
                end
                qp_cov[1] <= qp_cov[0]; qp_qx[1] <= qp_qx[0]; qp_qy[1] <= qp_qy[0]; qp_m[1] <= qp_m[0];
            end
            // stage 2: the sums with the reference values (32-bit wrapping: the true value of an emitted quad fits)
            qp_v[2] <= qp_v[1];
            if (qp_v[1]) begin
                for (int k = 0; k < 3; k++) qp_e[k] <= cur_er[k] + qp_pa[k] + qp_pb[k];
                qp_cov[2] <= qp_cov[1]; qp_qx[2] <= qp_qx[1]; qp_qy[2] <= qp_qy[1]; qp_m[2] <= qp_m[1];
            end
            // stage 3: the push (below, with the early queue's write); qp_v[3] marks the cycle of the push
            qp_v[3] <= qp_v[2];
            if (qp_v[2]) begin qp_cov[3] <= qp_cov[2]; qp_qx[3] <= qp_qx[2]; qp_qy[3] <= qp_qy[2]; qp_m[3] <= qp_m[2]; end
        end
    end
    wire [31:0] qp_e2 = qp_e[2], qp_e1 = qp_e[1], qp_e0 = qp_e[0];
    // ================================================================ early queue: bounds (zq_lane), peek, verdict
    // the lane: the quad's pixel-0 values e1 e2 (from the quad pipeline) and the current triangle's words; its bounds land in
    // the entry LAT cycles later. The words change only at M_TRIQS, which waits for the pipe and the lane to be empty.
    assign      zl_in = qp_v[2];
    wire        zl_v, zl_ok; wire [23:0] zl_lo, zl_hi; wire [3:0] zl_tag;
    zq_lane #(.TAGW(4)) u_zl (.clk, .rst(rst_q), .in_valid(zl_in), .in_e1(qp_e1), .in_e2(qp_e2), .in_zq(cur_zq), .in_d16(depth16),
        .in_tag(eq_wp[3:0]), .out_valid(zl_v), .out_ok(zl_ok), .out_dlo(zl_lo), .out_dhi(zl_hi), .out_tag(zl_tag));
    always @(posedge clk) begin
        zl_sr <= rst_q ? 6'd0 : {zl_sr[4:0], zl_in};
        if (zl_v) eq_bd[zl_tag] <= {zl_hi, zl_lo};
        // the early queue's entry (the room was checked when the item was taken; only this writes the queue)
        if (zl_in) begin
            eq_qx[eq_wp[3:0]] <= qp_qx[2]; eq_qy[eq_wp[3:0]] <= qp_qy[2]; eq_cov[eq_wp[3:0]] <= qp_cov[2];
            eq_e[eq_wp[3:0]] <= {qp_e2, qp_e1, qp_e0}; eq_m[eq_wp[3:0]] <= qp_m[2];
        end
    end
    // Early-depth eligibility of a quad, with the state of its bank. Its own draw: depth test on, stencil off (a depth-failing
    // fragment still runs the stencil ops), gl_FragDepth not written, and its own depth writes may only make a stale stored
    // depth pass MORE (func not NOTEQUAL unless writes are off). The OTHER bank's draw, when
    // any of its quads or fragments is still in flight (they may write depth after the peek), must write in a direction
    // that keeps the stale depth conservative for this quad's function: lowering writes (LESS / LEQUAL with writes on) for
    // LESS / LEQUAL, raising ones for GREATER / GEQUAL, none at all for EQUAL / NOTEQUAL. (Otherwise a LESS draw's quad
    // peeked while an ALWAYS or GREATER draw's writes are in flight could be dropped although the depth it should be
    // tested against is farther; the early peek, ahead of quad_setup, widens that window.)
    // (A draw with the depth test OFF and the mask ON writes every fragment's depth in b_rop — "the test
    // passes" — so its writes go either way (wdir 3), or a LESS quad of the other bank peeked while
    // such fragments are in flight could be dropped against the depth they are about to raise. The host libraries
    // never send that state (they send the mask only with the test: ES 2.0 writes no depth without the test); the
    // bench's random states do: BR_SKIPTEST frames, seed 23.)
    function automatic [1:0] wdir(input dst_t s);        // the depth writes of a draw: 0 none, 1 lower, 2 raise, 3 either way
        if (!s.depth_mask) wdir = 2'd0;
        else if (!s.depth_test) wdir = 2'd3;
        else case (s.depth_func) 3'd1, 3'd3: wdir = 2'd1; 3'd4, 3'd6: wdir = 2'd2; 3'd0, 3'd2: wdir = 2'd0; default: wdir = 2'd3; endcase
    endfunction
    function automatic dcompat(input [2:0] f, input [1:0] d);
        case (d) 2'd0: dcompat = 1'b1; 2'd1: dcompat = (f == 3'd0 || f == 3'd1 || f == 3'd3);
                 2'd2: dcompat = (f == 3'd0 || f == 3'd4 || f == 3'd6); default: dcompat = (f == 3'd0); endcase
    endfunction
    // reject a pixel: every depth in [lo, hi] fails func against the stored depth sd
    function automatic rejpx(input [2:0] f, input [23:0] lo, hi, sd);
        case (f)
            3'd0: rejpx = 1'b1;                                  // NEVER
            3'd2: rejpx = (lo > sd) || (hi < sd);                // EQUAL
            3'd5: rejpx = (lo == sd) && (hi == sd);              // NOTEQUAL
            3'd7: rejpx = 1'b0;                                  // ALWAYS
            default: rejpx = !dcmp(f, lo, sd) && !dcmp(f, hi, sd);   // LESS LEQUAL GREATER GEQUAL: monotone
        endcase
    endfunction
    // the other bank's quads / fragments in flight: in the quad path (q_live_b counts from collection, the early queue
    // included), between the back end and the ROP, or in the ROP's pipe
    wire o_busy0 = (q_live_b[1] != 16'd0) || (c1_v && c1_bank) || (c2_v && c2_bank) || (br_valid && br_bank) || (rop_bank && !rmw_idle);
    wire o_busy1 = (q_live_b[0] != 16'd0) || (c1_v && !c1_bank) || (c2_v && !c2_bank) || (br_valid && !br_bank) || (!rop_bank && !rmw_idle);
    wire [1:0] elig_b;                                   // per bank: its quads may be tested early now
    assign elig_b[0] = ds[0].depth_test & ~ds[0].stencil & ~wfd[0] & (~ds[0].depth_mask | (ds[0].depth_func != 3'd5)) &
                       (~o_busy0 | dcompat(ds[0].depth_func, wdir(ds[1])));
    assign elig_b[1] = ds[1].depth_test & ~ds[1].stencil & ~wfd[1] & (~ds[1].depth_mask | (ds[1].depth_func != 3'd5)) &
                       (~o_busy1 | dcompat(ds[1].depth_func, wdir(ds[0])));
    // the peek's band: the current band once it is set up, or the next band from the preload bank (its initial
    // depth is the clear depth under the fast clear, else the preloaded cell once the preload holds that band); none
    // while the banks are about to rotate (the read and its bank select must not straddle the rotation)
    // (wires, not a function: a continuous assignment re-evaluates on its function's arguments only)
    wire nx_peek  = nx_ok && (fast_frame || (pl_rdy && pl_rdy_y0 == bq_y0[nx_slot]));
    wire pk_block = bm_go_flush || (bm == BM_FLUSH);
    // the early stage's quad (eq_pp): its bounds are in (eq_lv), then it is skipped (ineligible: passes, no peek) or peeks
    wire [3:0] ep = eq_pp[3:0];
    wire       ep_ne = (eq_pp != eq_wp);
    wire [TSW+6-1:0] ep_m = eq_m[ep];
    wire       ep_bank = ep_m[2]; wire [1:0] ep_slot = ep_m[1:0];
    wire       ez_elig = elig_b[ep_bank];

    // ================================================================ pixel issuer: the early queue's head -> quad_setup
    always @(posedge clk) begin
        if (rst_q) begin pi_run <= 1'b0; px_valid <= 1'b0; qi_wp <= 4'd0; eq_rp <= 5'd0; ev_edrop <= 1'b0; c_ezrej1 <= 0; end
        else begin
            px_valid <= 1'b0; ev_edrop <= 1'b0;
            if (pi_run) begin
                px_valid <= 1'b1; px_slot <= qi_qs[qi_wp - 4'd1]; px_tag <= pi_n;
                px_e0 <= q_e_h[0]; px_e1 <= q_e_h[1]; px_e2 <= q_e_h[2]; px_p <= pi_n;
                px_x <= {q_qx_h, pi_n[0]}; px_y <= {q_qy_h, pi_n[1]};
                pi_n <= pi_n + 2'd1;
                if (pi_n == 2'd3) pi_run <= 1'b0;
            end
            if (q_take) begin                                    // (after the pixel above: the overlap restarts the issuer)
                pi_run <= 1'b1; pi_n <= 2'd0;
                qi_qx[qi_wp] <= eq_qx[eq_h]; qi_qy[qi_wp] <= eq_qy[eq_h]; qi_cov[qi_wp] <= eq_cov[eq_h];
                {qi_ts[qi_wp], qi_qs[qi_wp], qi_front[qi_wp], qi_bank[qi_wp], qi_slot[qi_wp]} <= eq_m[eq_h];
                qi_er[qi_wp] <= eq_rej[eq_h];
                qi_wp <= qi_wp + 4'd1; eq_rp <= eq_rp + 5'd1;
                if (eq_rej[eq_h]) c_ezrej1 <= c_ezrej1 + 1;      // (+EZAUDIT: counted, sent on, checked by the late test)
            end
            if (e_drop) begin                                    // certainly hidden: dropped before quad_setup
                eq_rp <= eq_rp + 5'd1; c_ezrej1 <= c_ezrej1 + 1;
                {ev_edrop_ts, ev_edrop_qs, ev_edrop_fr, ev_edrop_bk, ev_edrop_sl} <= eq_m[eq_h]; ev_edrop <= 1'b1;
            end
        end
    end
    // the issuer's copy of the quad (the head may change while the pixels go out)
    always @(posedge clk) if (q_take) begin
        q_e_h[0] <= eq_e[eq_h][31:0]; q_e_h[1] <= eq_e[eq_h][63:32]; q_e_h[2] <= eq_e[eq_h][95:64];
        q_qx_h <= eq_qx[eq_h]; q_qy_h <= eq_qy[eq_h];
    end

    // ================================================================ collector: 4 pixel results -> quad FIFO
    reg [95:0] cl_d24, cl_fz, cl_fw; reg [191:0] cl_lam, cl_pc; reg [71:0] cl_fc;
    always @(posedge clk) begin
        if (rst_q) begin qf_wp <= 5'd0; qi_rp <= 4'd0; ev_qso <= 1'b0; qs_infl <= 5'd0; end
        else begin
            ev_qso <= 1'b0;
            qs_infl <= qs_infl + 5'(q_take) - 5'(o_valid && o_tag == 2'd3);
            if (o_valid) begin
                cl_d24[o_tag*24 +: 24] <= o_d24; cl_fz[o_tag*24 +: 24] <= o_fcz; cl_fw[o_tag*24 +: 24] <= o_fw;
                cl_lam[o_tag*48 +: 48] <= {o_l2, o_l1}; cl_pc[o_tag*48 +: 48] <= {o_mu2, o_mu1};
                if (o_tag == 2'd0) cl_fc[47:0] <= {o_fcy, o_fcx};          // pixel 0: x + 0.5, y + 0.5
                if (o_tag == 2'd1) cl_fc[71:48] <= o_fcx;                   // pixel 1: x + 1.5
                if (o_tag == 2'd3) begin
                    qf_fc[qf_wp[3:0]] <= {o_fcy, cl_fc[47:24], cl_fc[71:48], cl_fc[23:0]};   // pixel 3: y + 1.5
                    qf_qx[qf_wp[3:0]] <= qi_qx[qi_rp]; qf_qy[qf_wp[3:0]] <= qi_qy[qi_rp]; qf_cov[qf_wp[3:0]] <= qi_cov[qi_rp];
                    qf_ts[qf_wp[3:0]] <= qi_ts[qi_rp]; qf_front[qf_wp[3:0]] <= qi_front[qi_rp]; qf_bank[qf_wp[3:0]] <= qi_bank[qi_rp];
                    qf_slot[qf_wp[3:0]] <= qi_slot[qi_rp]; qf_er[qf_wp[3:0]] <= qi_er[qi_rp];
                    qf_pc[qf_wp[3:0]] <= {o_mu2, o_mu1, cl_pc[143:0]};     // gl_PointCoord rows: the mus per pixel
                    qf_d24[qf_wp[3:0]] <= {o_d24, cl_d24[71:0]}; qf_fz[qf_wp[3:0]] <= {o_fcz, cl_fz[71:0]};
                    qf_fw[qf_wp[3:0]] <= {o_fw, cl_fw[71:0]}; qf_lam[qf_wp[3:0]] <= {o_l2, o_l1, cl_lam[143:0]};
                    qf_wp <= qf_wp + 5'd1; qi_rp <= qi_rp + 4'd1;
                    ev_qso <= 1'b1; ev_qso_qs <= qi_qs[qi_rp];
                end
            end
        end
    end

    // ================================================================ launch: early depth, warps of 4 quads, start
    // Two stages over the quad FIFO (no serial steps per quad):
    //   peek stage — runs ahead of the row stage (index qf_pk): the early-depth peeks of a quad's 2 ROWS (pixel pairs)
    //   go out one per cycle, back to back across quads (pk_en is registered here and b_rop registers the
    //   read and then the depths, so a row's stored depths are on pk_z three cycles after its issue); any covered pixel
    //   passing keeps the quad; the verdict lands in qf_ez[slot] / qf_ezv[slot] (a quad not eligible passes at once). The
    //   port is shared with the early queue's peeks: this stage first, the early queue in the cycles
    //   it leaves free; the pipeline carries each peek's stage;
    //   row stage — a quad whose verdict is in is dropped (one cycle) or joins the warp being formed (quad lq of warp
    //   fs_in_warp): its launch rows go out one per cycle (L_ROW; a program reading no built-in needs only row 3, the
    //   weights), its metadata is noted with the last row, and the next quad's rows follow without a gap when its verdict
    //   is in. The warp starts (L_GO) with its 4th quad, or earlier — lf_force — when nothing is between the early queue and
    //   the warp and the sequencer waits for quads to finish (a triangle slot, a draw change (so quads of two draws never
    //   share a warp), a band or the frame end), or when no quad could join it for 32 cycles (counted with quads in the
    //   FIFO too: a head waiting for the peek of a band two ahead, behind a partial warp of the
    //   current band, would wait for ever: the band cannot end while the partial warp holds its quads).
    // ≈ 4 cycles per quad, 1 without built-ins.
    typedef enum logic [1:0] {L_IDLE, L_ROW, L_GO} ls_t;
    ls_t ls;
    wire [3:0] qh = qf_rp[3:0];
    reg [1:0] lq, lrow; reg [5:0] lf_idle;
    wire seq_wait = (ms == M_TSWAIT) || (ms == M_TRIQS) || (ms == M_DSWAIT) || (ms == M_BAND) || (ms == M_FEND);
    wire qpath_empty = !pi_run && (qs_infl == 5'd0) && (qf_cnt == 5'd0);
    wire lf_force = (lq != 2'd0) && ((qpath_empty && seq_wait) || (lf_idle == 6'd32));
    // metadata of the launched quads in launch order, its own ring: FS_NWARP + 1 warp entries, because the core's output
    // latch lets a slot's next warp fill while the back end still reads the previous one. One write site and one read
    // site, so LUTRAM ({warp entry, quad}) instead of 2-D register arrays (2 080 FF, 940 LUT).
    // mw_q = {slot, front, depth, triangle slot, coverage, qy, qx}, mw_w = {the warp's draw-state bank, its last quad}
    localparam int MQW = 2 + 1 + 96 + TSW + 4 + 12 + 12;
    (* ram_style="distributed" *) reg [MQW-1:0] mw_q [0:(FS_NWARP+1)*4-1];
    (* ram_style="distributed" *) reg [2:0] mw_w [0:FS_NWARP];
    localparam int MWW = $clog2(FS_NWARP + 1);               // the ring pointers' width (3 at FS_NWARP 4; 4 lets a bench run 8 warps)
    reg [MWW-1:0] mw_wp;
    // ---- peek stage (late) and the early queue's peeks ----
    reg  [4:0] qf_pk; reg pk_row, ek_row;                   // the quad being peeked (qf_rp <= qf_pk <= qf_wp), its row; the early queue's row
    reg        qf_ez [0:QFD-1], qf_ezv [0:QFD-1];           // early-depth verdict per FIFO slot, valid
    reg        pd1_v, pd2_v, pd3_v, pd1_e, pd2_e, pd3_e; reg [3:0] pd1_s, pd2_s, pd3_s; reg pd1_p, pd2_p, pd3_p;
    reg        pk_acc, ek_acc;                               // row 0's result: a pixel passes (late) / both reject (early)
    wire [3:0] qp = qf_pk[3:0];
    wire pk_ne  = (qf_pk != qf_wp);
    wire l_elig = elig_b[qf_bank[qp]];
    wire l_band = ((qf_slot[qp] == rs_slot) && band_run) || ((qf_slot[qp] == nx_slot) && nx_peek);
    wire e_band = ((ep_slot == rs_slot) && band_run) || ((ep_slot == nx_slot) && nx_peek);
    // eligibility is decided at a quad's first row (it may change while the quad waits: the other bank's in-flight count);
    // a quad whose first row went out always gets its second, so the row toggles stay with their quads
    wire l_req  = pk_ne && (l_elig || pk_row) && !pk_block && l_band;
    wire e_req  = ep_ne && eq_lv[ep] && (ez_elig || ek_row) && !pk_block && e_band;
    // the verdicts, three cycles after a row's peek. Late: a covered pixel of the row passes — lanes {row, 0} / {row, 1}
    // against the pair's depths pk_z[23:0] / [47:24]. Early: every covered pixel of the row certainly fails (the bounds).
    dst_t ds_pv;
    assign ds_pv = ds[qf_bank[pd3_s]];
    wire pk_hit = (qf_cov[pd3_s][{pd3_p, 1'b0}] && dcmp(ds_pv.depth_func, qf_d24[pd3_s][{pd3_p, 1'b0}*24 +: 24], pk_z[23:0])) ||
                  (qf_cov[pd3_s][{pd3_p, 1'b1}] && dcmp(ds_pv.depth_func, qf_d24[pd3_s][{pd3_p, 1'b1}*24 +: 24], pk_z[47:24]));
    wire [47:0] pd_bd = eq_bd[pd3_s]; wire [3:0] pd_cov = eq_cov[pd3_s];
    wire [2:0]  pd_f  = ds[eq_m[pd3_s][2]].depth_func;
    wire pd_rej = (!pd_cov[{pd3_p, 1'b0}] || rejpx(pd_f, pd_bd[23:0], pd_bd[47:24], pk_z[23:0])) &&
                  (!pd_cov[{pd3_p, 1'b1}] || rejpx(pd_f, pd_bd[23:0], pd_bd[47:24], pk_z[47:24]));
    // ---- row stage ----
    wire hd_ok   = (qf_cnt != 5'd0) && qf_ezv[qh];          // the head's verdict is in
    wire hd_pass = hd_ok && qf_ez[qh];
    wire rs_drop = (ls == L_IDLE) && hd_ok && !qf_ez[qh];   // every covered pixel of the head fails: drop
    // the first quad of a warp needs the next warp slot free. L_FAST: in the cycle of fs_start the core's slot pointer
    // still names the warp being started, so the slot after it is asked for (fs_in_ready_nx); a row decided now is
    // written two cycles later, after the launch moved the pointer. (lq != 0 never meets fs_start: it follows lq <= 0.)
    wire ctx_ok  = L_FAST ? ((lq != 2'd0) || (fs_start ? fs_in_ready_nx : fs_in_ready))
                          : ((lq != 2'd0 || fs_in_ready) && !fs_start);
    wire hd_bank = (lq == 2'd0) || (qf_bank[qh] == cur_wbank);   // a warp never mixes draws: the head joins only its own bank's warp
    wire lw_ctx  = (ls == L_IDLE) && hd_pass && !ctx_ok;    // (profiling) a quad waits for a warp slot
    wire [3:0] qn = qf_rp[3:0] + 4'd1;
    wire nx_pass = (qf_cnt > 5'd1) && qf_ezv[qn] && qf_ez[qn] && (qf_bank[qn] == qf_bank[qh]);   // the quad after the head can follow at once
    wire [1:0] lrow0 = ubi[qf_bank[qh]] ? 2'd0 : 2'd3;     // the head's first launch row (nx_pass: the next quad has its bank)
    always @(posedge clk) begin
        if (rst_q) begin ls <= L_IDLE; mw_wp <= '0; qf_rp <= 5'd0; qf_pk <= 5'd0; pk_row <= 1'b0; fs_start <= 1'b0; fs_vw_we <= 1'b0; pk_en <= 1'b0; fs_pre <= 1'b0;
                       ev_drop <= 1'b0; pd1_v <= 1'b0; pd2_v <= 1'b0; pd3_v <= 1'b0; pk_acc <= 1'b0; ek_acc <= 1'b0; ek_row <= 1'b0;
                       c_quads <= 0; c_ezrej <= 0; lq <= 2'd0; lf_idle <= 6'd0; fs_in_active <= 16'd0; fs_in_front <= 4'd0; fs_in_tmask <= 16'hFFFF;
                       fs_in_tslot <= '0; cur_wbank <= 1'b0; eq_pp <= 5'd0; eq_lv <= '0; eq_vd <= '0;
                       for (int i = 0; i < QFD; i++) begin qf_ez[i] <= 1'b0; qf_ezv[i] <= 1'b0; end end
        else begin
            fs_start <= 1'b0; fs_vw_we <= 1'b0; pk_en <= 1'b0; ev_drop <= 1'b0; pd1_v <= 1'b0; fs_pre <= 1'b0;
            if (ls == L_IDLE && lq != 2'd0 && !hd_ok) begin if (lf_idle != 6'd32) lf_idle <= lf_idle + 6'd1; end
            else lf_idle <= 6'd0;
            // ---- the early queue: bounds in, a new entry ----
            if (zl_v) begin eq_lv[zl_tag] <= 1'b1; eq_zok[zl_tag] <= zl_ok; end
            if (zl_in) begin eq_lv[eq_wp[3:0]] <= 1'b0; eq_vd[eq_wp[3:0]] <= 1'b0; end
            // ---- peeks: a quad not eligible gets its verdict (pass) at once; the late stage's row first, else the early queue's ----
            if (pk_ne && !l_elig && !pk_row) begin qf_ez[qp] <= 1'b1; qf_ezv[qp] <= 1'b1; qf_pk <= qf_pk + 5'd1; end
            if (ep_ne && eq_lv[ep] && !ez_elig && !ek_row) begin eq_rej[ep] <= 1'b0; eq_vd[ep] <= 1'b1; eq_pp <= eq_pp + 5'd1; end
            if (l_req) begin
                pk_en <= 1'b1; pk_next <= !((qf_slot[qp] == rs_slot) && band_run); pk_x <= {qf_qx[qp], 1'b0}; pk_y <= {qf_qy[qp], pk_row};
                pk_addr <= rb_tab[RB_LB'({qf_qy[qp], pk_row}) - bq_y0[qf_slot[qp]][RB_LB-1:0]] + RB_AW'(qf_qx[qp]);
                pd1_v <= 1'b1; pd1_e <= 1'b0; pd1_s <= qp; pd1_p <= pk_row;
                pk_row <= ~pk_row;
                if (pk_row) qf_pk <= qf_pk + 5'd1;
            end else if (e_req) begin
                pk_en <= 1'b1; pk_next <= !((ep_slot == rs_slot) && band_run); pk_x <= {eq_qx[ep], 1'b0}; pk_y <= {eq_qy[ep], ek_row};
                pk_addr <= rb_tab[RB_LB'({eq_qy[ep], ek_row}) - bq_y0[ep_slot][RB_LB-1:0]] + RB_AW'(eq_qx[ep]);
                pd1_v <= 1'b1; pd1_e <= 1'b1; pd1_s <= ep; pd1_p <= ek_row;
                ek_row <= ~ek_row;
                if (ek_row) eq_pp <= eq_pp + 5'd1;
            end
            // ---- the verdicts (the depths are on pk_z three cycles after the issue) ----
            pd2_v <= pd1_v; pd2_e <= pd1_e; pd2_s <= pd1_s; pd2_p <= pd1_p;
            pd3_v <= pd2_v; pd3_e <= pd2_e; pd3_s <= pd2_s; pd3_p <= pd2_p;
            if (pd3_v && !pd3_e) begin
                if (pd3_p) begin qf_ez[pd3_s] <= pk_acc | pk_hit; qf_ezv[pd3_s] <= 1'b1; pk_acc <= 1'b0; end
                else pk_acc <= pk_hit;
            end
            if (pd3_v && pd3_e) begin
                if (pd3_p) begin eq_rej[pd3_s] <= eq_zok[pd3_s] && ek_acc && pd_rej; eq_vd[pd3_s] <= 1'b1; end
                else ek_acc <= pd_rej;
            end
            // ---- row stage ----
            case (ls)
                L_IDLE: begin
                    if (rs_drop) begin
                        qf_rp <= qf_rp + 5'd1; qf_ezv[qh] <= 1'b0; ev_drop <= 1'b1; ev_drop_ts <= qf_ts[qh]; ev_drop_bk <= qf_bank[qh];
                        ev_drop_sl <= qf_slot[qh]; c_ezrej <= c_ezrej + 1;
                    end else if (hd_pass && !hd_bank) ls <= L_GO;        // the head is of another draw: start the warp as it is
                    // a warp's first quad: its bank is known and stays loaded until the warp has gone (its quads are counted in
                    // q_live_b), so the core starts fetching the program now — >= 3 cycles before the start (fs_pre)
                    else if (hd_pass && ctx_ok) begin lrow <= lrow0; ls <= L_ROW; if (lq == 2'd0) begin cur_wbank <= qf_bank[qh]; fs_pre <= FS_PRE; end end
                    else if (!hd_ok && lf_force) ls <= L_GO;
                end
                // rows 0..3 of quad lq: gl_FragCoord.xy, .zw, gl_PointCoord (mu1, mu2), the weights {lambda2, lambda1};
                // with the last row the quad's metadata, then the next quad's rows or the warp start
                L_ROW: begin
                    fs_vw_we <= 1'b1; fs_vw_row <= lrow; fs_vw_quad <= lq;
                    case (lrow)
                        2'd0: for (int p = 0; p < 4; p++) fs_vw_data[p*2*W +: 2*W] <= {qf_fc[qh][48 + p[1]*24 +: 24], qf_fc[qh][p[0]*24 +: 24]};
                        2'd1: for (int p = 0; p < 4; p++) fs_vw_data[p*2*W +: 2*W] <= {qf_fw[qh][p*24 +: 24], qf_fz[qh][p*24 +: 24]};
                        2'd2: fs_vw_data <= qf_pc[qh];
                        default: fs_vw_data <= qf_lam[qh];
                    endcase
                    lrow <= lrow + 2'd1;
                    if (lrow == 2'd3) begin
                        mw_q[{mw_wp, lq}] <= {qf_slot[qh], qf_front[qh], qf_d24[qh], qf_ts[qh], qf_cov[qh], qf_qy[qh], qf_qx[qh]};
                        mw_w[mw_wp] <= {qf_bank[qh], lq};
                        fs_in_tslot[lq*TSW +: TSW] <= qf_ts[qh];
                        if (lq == 2'd0) begin fs_in_active <= 16'h000F; fs_in_front <= {3'b0, qf_front[qh]}; end
                        else begin fs_in_active[lq*4 +: 4] <= 4'hF; fs_in_front[lq] <= qf_front[qh]; end
                        // TEX_SKIP: the quad's texels are used in its covered pixels only, when its program allows it
                        fs_in_tmask[lq*4 +: 4] <= (TEX_SKIP && tsk[qf_bank[qh]]) ? qf_cov[qh] : 4'hF;
                        qf_rp <= qf_rp + 5'd1; qf_ezv[qh] <= 1'b0; c_quads <= c_quads + 1;
                        // the warp is full: L_FAST starts it with this row (the launch reads the registers written here
                        // one cycle later, beside the row's write into the slot it launches); else through L_GO
                        if (lq == 2'd3 && L_FAST) begin
                            fs_start <= 1'b1; lq <= 2'd0; ls <= L_IDLE; mw_wp <= (mw_wp == MWW'(FS_NWARP)) ? '0 : mw_wp + 1'b1;
                        end else if (lq == 2'd3) ls <= L_GO;
                        else begin lq <= lq + 2'd1; if (nx_pass) lrow <= lrow0; else ls <= L_IDLE; end
                    end
                end
                L_GO: begin fs_start <= 1'b1; lq <= 2'd0; ls <= L_IDLE; mw_wp <= (mw_wp == MWW'(FS_NWARP)) ? '0 : mw_wp + 1'b1; end
                default: ls <= L_IDLE;
            endcase
        end
    end
    // synthesis translate_off
    always @(posedge clk) if (!rst_q) begin
        if (pd3_v && !pd3_e && pd3_p && ls == L_IDLE && rs_drop && pd3_s == qh)
            $fatal(1, "b_render: early-depth verdict and the head's drop on the same slot %0d", qh);
        if ((qf_pk - qf_rp) > qf_cnt) $fatal(1, "b_render: the peek index ran past the quad FIFO");
        if ((eq_pp - eq_rp) > eq_cnt) $fatal(1, "b_render: the peek index ran past the early queue");
        if (ms == M_TRIQS && zl_in) $fatal(1, "b_render: a quad entered the lane while the triangle changes");
        if (zl_in && eq_cnt == 5'(EQD)) $fatal(1, "b_render: a quad pushed into a full early queue");
        // +EZAUDIT: a quad the early queue rejected (sent on instead of dropped) must fail the exact test as well, against
        // depths at least as fresh: the bounds are conservative and stale depths only make the test pass more
        if (pd3_v && !pd3_e && pd3_p && qf_er[pd3_s] && (pk_acc | pk_hit))
            $fatal(1, "b_render: the early depth test rejected a quad that passes at (%0d, %0d) cov %b", qf_qx[pd3_s], qf_qy[pd3_s], qf_cov[pd3_s]);
        // the registered forms (timing) equal the direct ones: qz_s = no quad of the slot alive or in the QUAD pipe
        for (int i = 0; i < 4; i++) begin : QZC
            reg inp;
            inp = 1'b0;
            for (int k = 0; k < 4; k++) if (qp_v[k] && qp_m[k][1:0] == 2'(i)) inp = 1'b1;
            if (qz_s[i] != ((q_live_s[i] == 16'd0) && !inp))
                $fatal(1, "b_render: qz_s[%0d] %b, q_live_s %0d, in the quad pipe %b", i, qz_s[i], q_live_s[i], inp);
        end
        for (int i = 0; i < NTS; i++)
            if (tsz[i] != (ts_cnt[i] == 8'd0)) $fatal(1, "b_render: tsz[%0d] %b, ts_cnt %0d", i, tsz[i], ts_cnt[i]);
        // a band starts its flush with none of its quads alive: not counted in q_live_s, none in the QUAD pipe, none being
        // taken (the race: the last quad in the pipe while a BAND / FRAME_END behind it lets the flush go)
        if (bm_go_flush) begin
            if (q_live_s[rs_slot] != 16'd0) $fatal(1, "b_render: band slot %0d flushes with %0d live quads", rs_slot, q_live_s[rs_slot]);
            if (qp_in && nq_slot == rs_slot) $fatal(1, "b_render: band slot %0d flushes while a quad of it is taken", rs_slot);
            for (int k = 0; k < 4; k++)
                if (qp_v[k] && qp_m[k][1:0] == rs_slot)
                    $fatal(1, "b_render: band slot %0d flushes with a quad of it in the quad pipe (stage %0d)", rs_slot, k);
        end
        // a quad enters the early queue only for a band that is announced and not flushed (slot rs_slot .. rs_slot +
        // bands_q - 1; the current one not while it flushes)
        begin : ZLC
            reg [1:0] d;
            d = qp_m[2][1:0] - rs_slot;                          // the quad's band, counted from the current one (mod 4)
            if (zl_in && ({1'b0, d} >= bands_q || (d == 2'd0 && bm == BM_FLUSH)))
                $fatal(1, "b_render: a quad of band slot %0d entered the early queue, band manager at slot %0d (bm %0d, %0d bands)",
                       qp_m[2][1:0], rs_slot, bm, bands_q);
        end
        // a draw-state bank is reloaded with no quad of it in the QUAD pipe either (the TRI between them takes >= 8 cycles,
        // the pipe 4: by construction, checked)
        if (ms == M_DSWAIT && bank_free)
            for (int k = 0; k < 4; k++)
                if (qp_v[k] && qp_m[k][2] == ld_bank) $fatal(1, "b_render: bank %0d reloaded with a quad of it in the quad pipe", ld_bank);
    end
    // synthesis translate_on

    // ================================================================ back end: finished warps -> b_rop, in order
    // Warp rq (launch order): quads 0..its last, each read as 5 rows from the core's output store, TWO rows per cycle
    // (out_sel / out_sel2 = {quad, 2k} / {quad, 2k + 1}: colour x y, z w, then depth) into the row buffer rb_*, handed to
    // the pixel stage in the depth row's cycle when it is free (else B_HAND waits); the warp is acked after its last
    // quad's rows. Pixel stage: one quad ROW = one pixel PAIR per cycle through the conversion steps:
    // c1 = lane select + A, c2 = B1, br_* = B2; the quad is done with its second row (ev_fin), and the next quad's rows
    // may be handed over in that same cycle (≈ 3 cycles per quad: the rows). All steps advance together whenever the
    // output register is free (bc_adv); drained waits for c1 / c2 as well (the quads themselves are counted in q_live
    // until ev_fin).
    typedef enum logic [1:0] {B_IDLE, B_ROW, B_HAND} bs_t;
    bs_t bs;
    reg [MWW-1:0] rq; reg [1:0] bqi, brow; reg [VW-1:0] rb_col; reg [4*W-1:0] rb_dep;
    wire [2:0] mw_rw = mw_w[rq];                                       // {bank, last quad} of warp rq
    reg bl; reg [VW-1:0] bq_col; reg [4*W-1:0] bq_dep; reg bq_dw; reg [3:0] bq_kill;
    reg [11:0] bq_qx, bq_qy; reg [3:0] bq_cov; reg [95:0] bq_d24; reg bq_front, bq_bank; reg [TSW-1:0] bq_ts; reg [1:0] bq_slot;
    assign fs_out_sel  = {bqi, brow, 1'b0};
    assign fs_out_sel2 = {bqi, brow, 1'b1};
    wire bc_adv  = !br_valid || br_ready;
    wire bq_free = !bq_v || (bc_adv && bl);                           // empty, or its last row leaves this cycle
    // BE_NOZ: a warp whose program wrote no gl_FragDepth has no depth row to read — its quads are handed over in the
    // cycle of their second colour row (z w taken straight from the store's outputs), 2 cycles per quad instead of 3
    wire be_noz  = BE_NOZ && !fs_out_depthw;
    wire be_h1   = (bs == B_ROW) && (brow == 2'd1) && be_noz;          // the no-depth hand-over cycle
    wire hand_ok = (bs == B_ROW && brow == 2'd2) || be_h1 || (bs == B_HAND);    // the quad's rows are (being) read
    reg [CW-1:0] c1_x, c1_y, c2_x, c2_y; reg c1_front, c2_front, c1_dw, c2_dw; reg [1:0] c1_pv, c2_pv;
    reg [37:0] c1_c [0:1][0:3]; reg [14:0] c2_c [0:1][0:3]; reg [50:0] c1_z [0:1]; reg [27:0] c2_z [0:1]; reg [23:0] c1_d24 [0:1], c2_d24 [0:1];
    reg [37:0] c1_ag [0:1], c1_arb [0:1]; reg [14:0] c2_ag [0:1], c2_arb [0:1];   // the source alpha in G's and in R / B's units
    // the c2 fragment's cell address, a REGISTER formed as the fragment enters c2 (timing: c2_slot -> bq_y0 -> the row
    // subtract -> rb_tab -> the add -> br_addr_n -> the ROP's six hazard compares -> haz_r is too long). Exact: rb_tab follows
    // rt_width (constant within a frame) and a slot's bq_y0 is rewritten only once the slot is free — after its band's
    // flush, which waits for every fragment of the slot to leave c1 / c2 / br (tail_cur)
    assign br_addr_c = c2_addr;
    assign br_addr_n = bc_adv ? br_addr_c : br_addr;                   // what br_addr holds next cycle (the ROP's early hazard check)
    // synthesis translate_off
    always @(posedge clk) if (!rst_q && c2_v && c2_addr != rb_tab[RB_LB'(c2_y[RB_LB-1:0]) - bq_y0[c2_slot][RB_LB-1:0]] + RB_AW'(c2_x >> 1))
        $fatal(1, "b_render: c2_addr %0d differs from the c2 fragment's (%0d, %0d) cell", c2_addr, c2_x, c2_y);
    // synthesis translate_on
    always @(posedge clk) begin
        if (rst_q) begin rq <= '0; bqi <= 2'd0; brow <= 2'd0; bs <= B_IDLE; bq_v <= 1'b0; fs_ack <= 1'b0; br_valid <= 1'b0;
                       ev_fin <= 1'b0; c1_v <= 1'b0; c2_v <= 1'b0; bl <= 1'b0; end
        else begin
            fs_ack <= 1'b0; ev_fin <= 1'b0;
            // ---- pixel stage (first: the hand-over below may reload bq in the cycle its last row leaves) ----
            if (bc_adv) begin
                c1_v <= 1'b0;
                if (bq_v) begin : PX
                    reg [1:0] pv2;                                    // the row's pixels present: covered, not killed
                    pv2 = {bq_cov[{bl, 1'b1}] & ~bq_kill[{bl, 1'b1}], bq_cov[{bl, 1'b0}] & ~bq_kill[{bl, 1'b0}]};
                    c1_v <= |pv2; c1_pv <= pv2;
                    c1_x <= {bq_qx, 1'b0}; c1_y <= {bq_qy, bl}; c1_front <= bq_front; c1_dw <= bq_dw; c1_bank <= bq_bank; c1_slot <= bq_slot;
                    for (int p = 0; p < 2; p++) begin : LN
                        int l; l = 2 * int'(bl) + p;                  // the quad lane of pixel p of row bl
                        c1_c[p][0] <= cn_a(bq_col[l*TW + 0*W +: W], n_r); c1_c[p][1] <= cn_a(bq_col[l*TW + 1*W +: W], n_g);
                        c1_c[p][2] <= cn_a(bq_col[l*TW + 2*W +: W], n_b); c1_c[p][3] <= cn_a(bq_col[l*TW + 3*W +: W], n_a);
                        c1_ag[p]  <= cn_a(bq_col[l*TW + 3*W +: W], n_g); c1_arb[p] <= cn_a(bq_col[l*TW + 3*W +: W], n_r);
                        c1_z[p] <= u24_a(bq_dep[l*W +: W], depth16); c1_d24[p] <= bq_d24[l*24 +: 24];
                    end
                    bl <= ~bl;
                    if (bl) begin bq_v <= 1'b0; ev_fin <= 1'b1; ev_fin_ts <= bq_ts; ev_fin_bk <= bq_bank; ev_fin_sl <= bq_slot; end
                end
                c2_v <= c1_v; c2_pv <= c1_pv; c2_x <= c1_x; c2_y <= c1_y; c2_front <= c1_front; c2_dw <= c1_dw; c2_bank <= c1_bank;
                c2_slot <= c1_slot;
                c2_addr <= rb_tab[RB_LB'(c1_y[RB_LB-1:0]) - bq_y0[c1_slot][RB_LB-1:0]] + RB_AW'(c1_x >> 1);
                for (int p = 0; p < 2; p++) begin
                    for (int c = 0; c < 4; c++) c2_c[p][c] <= c16_b1(c1_c[p][c]);
                    c2_ag[p] <= c16_b1(c1_ag[p]); c2_arb[p] <= c16_b1(c1_arb[p]); c2_z[p] <= u24_b1(c1_z[p]); c2_d24[p] <= c1_d24[p];
                end
                br_valid <= c2_v; br_pv <= c2_pv & sc_mask(ds[c2_bank], c2_x, c2_y); br_x <= c2_x; br_y <= c2_y; br_front <= c2_front; br_bank <= c2_bank; br_slot <= c2_slot;
                br_addr <= br_addr_c;
                for (int p = 0; p < 2; p++) begin
                    br_c16[p*52 +: 52] <= {c16_b2(c2_c[p][0], one_r), c16_b2(c2_c[p][1], one_g), c16_b2(c2_c[p][2], one_b), c16_b2(c2_c[p][3], one_a)};
                    br_sa[p*26 +: 26]  <= {c16_b2(c2_ag[p], one_g), c16_b2(c2_arb[p], one_r)};
                    br_z[p*24 +: 24]   <= c2_dw ? u24_b2(c2_z[p], depth16) : c2_d24[p];
                end
            end
            // ---- rows: two per cycle; the hand-over with the depth row when the pixel stage is free ----
            case (bs)
                B_IDLE: if (fs_out_valid && !fs_ack) begin bqi <= 2'd0; brow <= 2'd0; bs <= B_ROW; end
                B_ROW: begin                                        // out_sel / out_sel2 = {bqi, brow, 0 / 1} this cycle
                    if (brow == 2'd2) begin rb_dep <= fs_out_row; if (!bq_free) bs <= B_HAND; end
                    else begin : RW
                        int c0; c0 = brow[0] ? 2 : 0;               // components c0 (out_row) and c0 + 1 (out_row2)
                        for (int p = 0; p < 4; p++) begin
                            rb_col[p*TW + c0*W +: W]       <= fs_out_row[p*W +: W];
                            rb_col[p*TW + (c0 + 1)*W +: W] <= fs_out_row2[p*W +: W];
                        end
                        if (be_h1) begin if (!bq_free) bs <= B_HAND; end   // no depth row: the hand-over is now (below) or waits
                        else brow <= brow + 2'd1;
                    end
                end
                B_HAND: ;
                default: bs <= B_IDLE;
            endcase
            if (hand_ok && bq_free) begin                           // the quad's rows into the pixel stage
                bq_v <= 1'b1; bl <= 1'b0; bq_col <= rb_col; bq_dep <= (bs == B_ROW) ? fs_out_row : rb_dep; bq_dw <= fs_out_depthw;
                if (be_h1)                                          // z w are on the store's outputs this cycle (bq_dep: unused)
                    for (int p = 0; p < 4; p++) begin
                        bq_col[p*TW + 2*W +: W] <= fs_out_row[p*W +: W]; bq_col[p*TW + 3*W +: W] <= fs_out_row2[p*W +: W];
                    end
                bq_kill <= fs_out_kill[bqi*4 +: 4];
                {bq_slot, bq_front, bq_d24, bq_ts, bq_cov, bq_qy, bq_qx} <= mw_q[{rq, bqi}]; bq_bank <= mw_rw[2];
                if (bqi == mw_rw[1:0]) begin fs_ack <= 1'b1; rq <= (rq == MWW'(FS_NWARP)) ? '0 : rq + 1'b1; bs <= B_IDLE; end
                else begin bqi <= bqi + 2'd1; brow <= 2'd0; bs <= B_ROW; end
            end
        end
    end
    // ================================================================ per-frame cycle counters
    // Counted from go to done and latched into perf with done: 0 frame cycles, 1 flush busy (packer / accumulators /
    // writer), 2 at a band change (M_BAND, or fragments waiting for their band), 3 at a draw change (M_DSWAIT + the bucket /
    // uniform / program load), 4 FS busy, 5 band set-up (clear / wipe / load), 6 the render stream empty while an item is
    // expected (A / link bound), 7 the back end stalled by the ROP. frame_err: a stack violation killed a warp this frame.
    reg [31:0] pc [0:7]; reg ferr;
    always @(posedge clk) begin
        if (rst_q) begin for (int i = 0; i < 8; i++) pc[i] <= 32'd0; ferr <= 1'b0; perf <= '0; frame_err <= 1'b0; end
        else if (go) begin for (int i = 0; i < 8; i++) pc[i] <= 32'd0; ferr <= 1'b0; end
        else if (ms != M_IDLE) begin
            pc[0] <= pc[0] + 32'd1;
            if (flush_busy) pc[1] <= pc[1] + 32'd1;
            if (ms == M_BAND || (rop_hold && !rop_sw)) pc[2] <= pc[2] + 32'd1;   // the band queue full / fragments waiting for their band
            if (ms == M_DSWAIT || ms == M_DSREAD || ms == M_UNI || ms == M_PROG) pc[3] <= pc[3] + 32'd1;
            if (!fs_idle) pc[4] <= pc[4] + 32'd1;
            if (band_busy) pc[5] <= pc[5] + 32'd1;
            if (ms == M_ITEM && !sb_valid) pc[6] <= pc[6] + 32'd1;
            if (br_valid && !br_ready) pc[7] <= pc[7] + 32'd1;
            if (fs_out_valid && fs_out_err) ferr <= 1'b1;
            if (ms == M_FDONE && !flush_busy) begin                       // with done
                for (int i = 0; i < 8; i++) perf[i*32 +: 32] <= pc[i];
                frame_err <= ferr;
            end
        end
    end
    // ================================================================ cumulative profile counters (PROF; hwprof's comment)
    // Every event is registered before its counter (a counter's enable never sits behind a decision chain); the core's
    // events arrive two cycles late and are gated with the frame's run bit as it is then — the sums are off by a few
    // cycles per frame at most. tq_n = quads inside the texture unit, dq_n = its DDR reads outstanding.
    generate if (PROF) begin : HWP
        reg [31:0] hp [0:23];
        reg        e_run, e_fl, e_bc, e_dc, e_fs, e_bs, e_se, e_rs, e_st, e_pi, e_eq, e_lw, e_tb, e_fw, e_rq, e_dr;
        reg [5:0]  tq_n; reg [4:0] dq_n;
        wire       run = (ms != M_IDLE);
        always @(posedge clk) begin
            e_run <= run; e_fl <= run & flush_busy; e_bc <= run & (ms == M_BAND || (rop_hold && !rop_sw));
            e_dc <= run & (ms == M_DSWAIT || ms == M_DSREAD || ms == M_UNI || ms == M_PROG);
            e_fs <= run & ~fs_idle; e_bs <= run & band_busy; e_se <= (ms == M_ITEM) & ~sb_valid; e_rs <= run & br_valid & ~br_ready;
            e_st <= fs_start; e_pi <= pi_run; e_eq <= (ms == M_ITEM) & sb_valid & quad_blk; e_lw <= lw_ctx;
            e_tb <= run & tm_pev[0]; e_fw <= run & tm_pev[1]; e_rq <= tx_req & tx_ready; e_dr <= t_en & t_rdy;
            if (rst_q) begin tq_n <= 6'd0; dq_n <= 5'd0; for (int i = 0; i < 24; i++) hp[i] <= 32'd0; end
            else begin
                tq_n <= tq_n + 6'(tx_req & tx_ready) - 6'(tx_res_v);
                dq_n <= dq_n + 5'(t_en & t_rdy) - 5'(t_valid);
                hp[0] <= hp[0] + 32'(e_run);  hp[1] <= hp[1] + 32'(e_fl);  hp[2] <= hp[2] + 32'(e_bc);  hp[3] <= hp[3] + 32'(e_dc);
                hp[4] <= hp[4] + 32'(e_fs);   hp[5] <= hp[5] + 32'(e_bs);  hp[6] <= hp[6] + 32'(e_se);  hp[7] <= hp[7] + 32'(e_rs);
                if (e_run) begin
                    hp[8]  <= hp[8]  + 32'(fs_pev[21]);     hp[9]  <= hp[9]  + 32'(fs_pev[20:17]); hp[10] <= hp[10] + 32'(fs_pev[12:9]);
                    hp[11] <= hp[11] + 32'(fs_pev[16:13]);  hp[12] <= hp[12] + 32'(fs_pev[8:5]);   hp[13] <= hp[13] + 32'(fs_pev[4:0]);
                    hp[21] <= hp[21] + 32'(tq_n);           hp[23] <= hp[23] + 32'(dq_n);
                end
                hp[14] <= hp[14] + 32'(e_st); hp[15] <= hp[15] + 32'(e_pi); hp[16] <= hp[16] + 32'(e_eq); hp[17] <= hp[17] + 32'(e_lw);
                hp[18] <= hp[18] + 32'(e_tb); hp[19] <= hp[19] + 32'(e_fw); hp[20] <= hp[20] + 32'(e_rq); hp[22] <= hp[22] + 32'(e_dr);
            end
        end
        for (genvar hi = 0; hi < 24; hi++) begin : HO assign hwprof[hi*32 +: 32] = hp[hi]; end
    end else begin : NHWP
        assign hwprof = '0;
    end endgenerate
    wire _unused = &{1'b0, o_valid, lw_ctx, ev_edrop_fr};
endmodule
`default_nettype wire
