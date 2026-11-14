// shader_core_fs2 — board B's scalar fragment core (ISA: tools/isa64.py).
//   A warp = 16 lanes = 4 quads (lane l: quad l >> 2, pixel l & 3 = TL TR BL BR); NWARP warps in flight, each with its
//   own PC / prefetch queue, masks, predicate, stacks, scoreboard and 32 vec4 registers. One instruction = one vec4
//   operation executed as one MICRO-OP per written component (x, y, z, w order), each micro-op = the same scalar op on
//   all 16 lanes; a source's swizzle is only an address (component swz_k[c] of register rs_k).
//   Stages: F prefetch (per-warp queues from the 64-bit iram, the instruction classes decoded at the queue write) |
//   S select (oldest ready warp; the micro-op's hazards on the per-scalar-register scoreboard, unit and write-slot
//   availability; control flow; the head micro-op's fields are registered one cycle ahead and a taken branch
//   flushes the queue one cycle after its issue) | X register addresses {warp, reg, comp} to the three block-RAM copies, params /
//   varin / triangle-store addresses | X2 operands, routing, dispatch to the units | W one write per cycle (16 lanes x
//   W bits, per-lane enables = the active mask at issue). A fixed-latency ("scheduled") result frees its scalar
//   register one cycle before its write (a block-RAM read presented in the write's cycle returns the new value); a
//   buffered result frees it at the write.
//   Scheduled paths (latency from X2): madd 8 (ADD SUB MUL MAD DDX DDY and the chain micro-ops), cmp 1 (MIN MAX SET;
//   SETP into the predicate), the 1-cycle path (MOV MOVI LDP LDV of a built-in SEL IADD ISUB ISLT ISGE ISEQ ISNE),
//   IMUL 3 (16 DSP lanes).
//   Chains: DP4 = 4 madd micro-ops into two per-warp accumulators (p0 -> acc1, p2 -> acc2, p1 + acc1
//   -> acc1, p3 + acc2 -> acc2) then acc1 + acc2 written to every wm component (bit-identical to float_dot4_sum's
//   (p0 + p1) + (p2 + p3)); DP3 the same without p3; LDV of a user varying = a wavefront: the lambda1 *
//   D1 + f0 -> acc1.c steps of all written components back to back, then, once the last one has landed, the lambda2 *
//   D2 + acc1.c -> rd.c steps back to back (each component's two steps are at least 9 cycles apart, so the components
//   must not run one after the other), with lambda per lane in built-in row 3 and {f0, D1, D2} per quad in
//   the triangle store (4 block-RAM copies, one per quad, addressed by the quad's slot). Accumulator steps never touch
//   the write port; the accumulators live in LUTRAM (read in X): acc1 has one entry per {warp, component} for LDV, a DP
//   uses entry 0.
//   Built-ins: per lane two LUTRAMs hold the launch rows' two components (gl_FragCoord.xy / .zw, gl_PointCoord, the
//   lambdas); gl_FrontFacing is a bit per quad; LDV 9 / 10 form (front, 0, 0, 0) / (mu1, mu2, 0, 1) on the read.
//   Outputs: STO / STOZ write one LUTRAM per lane ({warp, component | depth}); STO2: a STO is 2 micro-ops
//   of two components each (x y, then z w; the odd one read through copy 1) into an even and an odd LUTRAM per lane,
//   read by out_sel (x, z, depth) and out_sel2 (y, w). The oldest finished warp retires into the
//   output latch as soon as the latch is free or being freed, which frees its slot at once; its rows stay in the
//   slot's output store and are read through out_sel ({quad, idx}) as 4 pixels each until out_ack, and the slot's next
//   warp may run but stalls at its first STO / STOZ until then.
//   Quarter-rate paths (4 units each, lane 4k+i through unit k in cycle i of a 4-cycle feed, results collected into
//   one 16-lane write): SFU (RCP RSQRT LOG2 EXP2, 11 + 4 with the SPLIT6 stage), SIN / COS (11 + 4), convert (FTOI ITOF FTOU UTOF TRUNC FLR
//   CEIL RNDE, 2 + 4), IMUL (3 + 4), the divider (4 rounds of W + 2, one in flight; no IOP unit). One shared
//   feeder: a quarter-rate micro-op issues every 4th cycle at most, up to
//   QR_INF in flight per class; results wait in one shared queue of 4 x QR_INF entries
//   and take free write cycles. The divider (IDIV
//   IMOD UDIV UMOD, 16 lanes, W + 2 cycles, one in flight) also returns a buffered result.
//   Control flow (S stage, 16-lane masks, as shader_core_fs): IF / ELSE / ENDIF with the 8-deep reconvergence stack,
//   LOOP / ENDL / BREAK / CONT with the 4-deep loop stack (entering, loop and continue masks), BRA, DISCARD into the
//   kill mask, SETP into the per-lane predicate; a taken branch flushes the warp's prefetch queue. A stack violation
//   (push on a full / pop on an empty stack) does not issue: the warp is killed, err set, err_stack sticky.
//   TEX: 4 coordinate micro-ops (k = 0..3 read rs0[swz[k]] of all 16 lanes into the coordinate
//   staging, owned by one TEX from its first micro-op to its last request), then one request per quad with an active
//   lane to the texture unit (tmu24's interface, one per cycle when ready, lanes 4q..4q+3 = pixels TL TR BL BR; a TEX
//   without active lanes sends nothing); results return in order and are written over 4 cycles (one component row
//   each) into the result staging (4 LUTRAM quad-blocks x TEXQ entries x 4 components) by the writer of the quad's
//   class (TEXW2: two writers, even / odd quads — a writer takes one result per 4 cycles, tmu24 paces by class; with
//   one writer every result is 4 cycles after the last). A request carries its quad's sample mask (in_tmask: the
//   texture unit returns zeros for the other pixels). A complete entry drains through the buffered write
//   path, one 16-lane write per wm component. Non-blocking: micro-op k checks coordinate k (RAW) and rd.k (WAW);
//   rd.wm is pending from the last micro-op to each component's write (so TEX rd, rd works); TEXQ TEXes live at once
//   (issued, texels not all written). TEX2: a coordinate micro-op hands TWO coordinates of the set to
//   the staging (the second through read copy 1, its WAW check through source 2's hazard bit): a 2D lookup is one
//   micro-op, a set of 3 or 4 two.
`default_nettype none
module shader_core_fs2 #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16,
    parameter int NWARP = 4,                    // warps in flight (2..4)
    parameter int NPARAM = 64,                  // uniforms (vec4)
    parameter int IRAM_DEPTH = 4096,            // instructions (64-bit words)
    parameter int QD = 4,                       // prefetch queue depth per warp
    parameter int NTS = 8,                      // triangle slots
    parameter int TEXQ = 8,                     // TEX instructions live at once (issued, texels not yet all written; four warps x two live samples)
    parameter bit TEXW2 = 1'b1,                 // two texture-result writers (even / odd quads of a warp:
                                                // quads with a sample mask return faster than one per 4 cycles); 0 = one
    parameter bit TEX2 = 1'b0,                  // a TEX hands a coordinate PAIR per micro-op to the coordinate staging (a 2D lookup: 1
                                                // micro-op instead of 2; the odd component through read copy 1). 0 = one
                                                // coordinate each. OFF on the boards: with L_FAST and
                                                // BE_NOZ it gives +0.8 % of frame rate for +1.1 k LUT and a 0.2 ns timing miss
    parameter bit STO2 = 1'b1,                  // STO in 2 micro-ops (x y, then z w: the second component through read copy 1, the
                                                // output store in an even and an odd half). out_sel then reads idx 0 / 2 / 4
                                                // only and out_sel2 idx 1 / 3 only. 0 = 4 micro-ops
    parameter int LDV_MODE = 0,                 // LDV of a user varying (lambda1 * D1 + f0, then lambda2 * D2 + that)
                                                // (the default is the boards': no second multiplier / adder):
                                                // 0 = two ISSUED micro-ops per component (the lambda1 / lambda2 wavefront through
                                                //     acc1).
                                                // 1 = one issued micro-op; the second step is not issued: the first results wait
                                                //     in a ring (acc1's entries above the warps' DP entries, D2 and the write's
                                                //     address beside them) and the oldest enters the SAME multiply-add lanes in
                                                //     the first cycle the micro-op in X2 does not use them (a STO, a TEX
                                                //     coordinate, a one-cycle op, a bubble) and its write slot is free;
                                                //     multiply-add micro-ops are held back only when LDV_TH results wait or the
                                                //     oldest has waited LDV_WT cycles (mblk).
                                                // 2 = one issued micro-op and a SECOND multiplier + adder per lane: lambda2 * D2
                                                //     beside the multiply-add, its product added to the multiply-add's result —
                                                //     one fixed-latency result (MADD_LAT + ADDSUB_LAT), a lane cycle per component.
                                                // All three bit-identical.
    parameter int LDV_TH = 4, LDV_WT = 12,
    // Two fused operations (SFU times an operand, and clamp), each ONE micro-op per component and
    // bit for bit the sequence it replaces (tools/isa64.py OPS; tests/fs2_xsfu.sasm, fs2_clamp.sasm):
    parameter bit XSFU = 1'b1,                  // SQRT rd, x (= RSQRT, MUL, SEL), RCPM rd, x, y = y * RCP(x) (a divide: RCP, MUL),
                                                // RSQM rd, x, y = y * RSQRT(x): a multiplier behind each of the four units takes the
                                                // unit's result and a second operand that travelled beside it; every result of the
                                                // unit passes that stage (+3 cycles), so the stream stays in order
    parameter bit XCLAMP = 1'b0,                // CLAMP rd, x, lo, hi = MIN(MAX(x, lo), hi) (MAX, MIN): float_cmp3 per lane.
                                                // OFF on the boards (float_cmp3 is 182 LUT against
                                                // float_cmp's 78, +1 664 on 16 lanes, for one micro-op per clamp); tb_fs2 has it on
                                                // (either parameter 0 = its opcodes kill the warp, as an IOP does: C_BAD)
    parameter int QR_INF = 4,                   // micro-ops in flight per quarter-rate class (SFU, SIN / COS, convert, IMUL): 2 or 4.
                                                // With 2: a sin / cos is ~21 cycles in flight, so the class takes one micro-op
                                                // per ~10.5 cycles while the feeder allows one per 4 — a trig-heavy shader (a
                                                // Shadertoy tunnel: 791 SIN / COS of 5 640 micro-ops a warp) issues in 52 % of the cycles,
                                                // 72 % of the idle ones a SIN / COS waiting for its class. The units are 15-cycle
                                                // pipelines fed every 4th cycle: they hold four feeds. 4 = a collector's meta queue of 4,
                                                // the shared result queue of 16 (tb_fs2 at 8 warps: that warp's cycles -26 %)
    parameter bit PROF = 1'b0,                  // the profile events (prof_ev) for the board's cycle counters (b_render PROF)
    parameter bit ADD_DSP = 1'b0,               // the lanes' adders (the multiply-add's and LDV_MODE 2's second one) do their alignment
                                                // shift on a DSP48 each (float_addsub DSP_ALIGN): LUTs for DSPs, bit-identical
    parameter     RCP_MEM = "tools/gen/fp24_rcp_seed.mem", RSQRT_MEM = "tools/gen/fp24_rsqrt_seed.mem",
                  LOG2_MEM = "tools/gen/fp24_log2_lut.mem", EXP2_MEM = "tools/gen/fp24_exp2_lut.mem",
                  SIN_MEM = "tools/gen/fp24_sin_lut.mem"
)(
    input  wire clk, rst,
    // program / uniform load: two banks — {bank, address}; a warp runs on the bank it was
    // launched with (in_bank), so the other bank may be loaded while warps run
    input  wire        iram_we,
    input  wire [$clog2(IRAM_DEPTH):0] iram_waddr,
    input  wire [63:0] iram_wdata,
    input  wire        param_we,
    input  wire [$clog2(NPARAM):0] param_idx,
    input  wire [4*(EXP_W+MAN_W+1)-1:0] param_data,
    // built-in rows of the warp being launched (in_warp), two components per pixel, one quad per write (pixel p:
    // component 0 at [p*2W +: W], 1 at [p*2W+W +: W]): row 0 gl_FragCoord.xy, 1 gl_FragCoord.zw, 2 gl_PointCoord
    // (mu1, mu2), 3 the interpolation weights (lambda1, lambda2); gl_FrontFacing comes with the launch (in_front, one
    // bit per quad). LDV 8 reads rows 0 / 1, LDV 9 = (front ? 1.0 : 0, 0, 0, 0), LDV 10 = (mu1, mu2, 0, 1.0), LDV 11..15
    // = 0 (the only built-ins of the board-B core; user varyings 0..7 come from the triangle store)
    input  wire        vw_we,
    input  wire [1:0]  vw_row, vw_quad,
    input  wire [8*(EXP_W+MAN_W+1)-1:0] vw_data,
    // triangle store: {D2, D1, f0} (vec4 each) of user varying tri_vidx in slot tri_slot; accepted when tri_ready
    // (the write takes 4 cycles: one per component); a slot is rewritten only when no warp in the core uses it
    input  wire        tri_we,
    output wire        tri_ready,
    input  wire [$clog2(NTS)-1:0] tri_slot,
    input  wire [2:0]  tri_vidx,
    input  wire [12*(EXP_W+MAN_W+1)-1:0] tri_data,
    // launch (in order): in_ready for warp in_warp, in_start launches it with its active lanes and its quads' slots
    output wire        in_ready,
    output wire        in_ready_nx,                     // the warp slot after in_warp is free too: the launcher may fill it from
                                                        // the cycle of in_start on (its rows land after the launch moved the slot)
    output wire [1:0]  in_warp,
    input  wire        in_start,
    input  wire        in_bank,                                      // the warp's program / uniform / sampler bank
    input  wire [15:0] in_active,
    input  wire [3:0]  in_front,                                    // gl_FrontFacing of quad q
    input  wire [4*$clog2(NTS)-1:0] in_tslot,                       // quad q's slot at [q*TSW +: TSW]
    // the lanes whose texels are USED: a TEX request carries its quad's 4 bits
    // as the sample mask (tex_req_mask) and the texture unit returns zeros for the others. The launcher passes a quad's
    // coverage only when the warp's program cannot tell the difference (b_render's program scan: no derivative of,
    // texture coordinate from, or control flow that reaches either through a value derived from a texel), all ones
    // otherwise. The core itself runs every lane.
    input  wire [15:0] in_tmask = 16'hFFFF,
    // pre_start (optional; a pulse at least one cycle before the launch, with warp in_warp idle): the launcher has begun
    // to fill warp in_warp with quads of bank pre_bank, so its program fetch starts now and the first instruction is at
    // the queue's head when the launch comes (it hides the 5 cycles between a launch
    // and the first issue). The launch must follow with in_bank = pre_bank, and that bank's program must not be
    // rewritten in between (b_render: the forming warp's quads keep the bank from being reloaded). Without a pre_start
    // the fetch starts at the launch.
    input  wire        pre_start = 1'b0,
    input  wire        pre_bank = 1'b0,
    // texture unit (tmu24's request / result interface): one quad per request, pixel p's s, t at uv[p*2W +: W] and
    // [p*2W+W +: W], r, q in zw; the 4 quads of a TEX in order; one quad's 16 texels per res_valid (pixel p, component c
    // at [(p*4+c)*W +: W]) in request order; results of the same tex_req_cls AT LEAST 4 CYCLES APART (the core writes a
    // result into its staging over 4 cycles of that class's writer; tmu24 paces by the class it was given with the
    // request; a bench assertion checks it)
    output wire        tex_req_valid,
    input  wire        tex_req_ready,
    output wire [4:0]  tex_req_sampler,                 // {the warp's bank, sampler[3:0]}
    output wire [1:0]  tex_req_mode,
    output wire [3:0]  tex_req_mask,                    // the quad's pixels to sample (in_tmask of its warp)
    output wire        tex_req_cls,                     // the result writer the quad's texels will take (TEXW2: its index's bit 0)
    output wire [8*(EXP_W+MAN_W+1)-1:0] tex_req_uv, tex_req_zw,
    input  wire        tex_res_valid,
    input  wire [16*(EXP_W+MAN_W+1)-1:0] tex_res_texels,
    // results in launch order: out_valid for the warp in the output latch; its colour and depth are read row by row —
    // out_sel = {quad[1:0], idx[2:0]} (idx 0..3 = colour component, 4 = gl_FragDepth) selects out_row = the quad's 4
    // pixels (pixel p at [p*W +: W]); the output store is LUTRAM, so a row is a combinational read. in_ready can rise
    // before the out_ack of the warp that held the slot: a launcher keeps per-warp data of up to NWARP + 1 warps
    output wire        out_valid,
    input  wire        out_ack,
    input  wire [4:0]  out_sel,
    output wire [4*(EXP_W+MAN_W+1)-1:0] out_row,
    input  wire [4:0]  out_sel2,                        // a second read of the output store (the back
    output wire [4*(EXP_W+MAN_W+1)-1:0] out_row2,       // end reads two rows per cycle)
    output wire        out_depthw,                      // the program wrote gl_FragDepth
    output wire [15:0] out_kill,
    output wire        out_err,
    output reg         err_stack,
    output wire        idle,
    // profile events, two cycles behind the cycle t they describe (registered off the issue path; PROF):
    // {an issue in t, warps waiting with a texture lookup outstanding [3:0], warps waiting otherwise [3:0], warps whose
    // head micro-op is not loaded (program fetch) [3:0], busy warps [3:0], live TEXes [4:0]} — the three waiting counts
    // leave out the warp that issued in t; a busy warp in none of them has finished (waits to retire) or was killed
    output reg  [21:0] prof_ev
);
    localparam int W = 1 + EXP_W + MAN_W, TW = 4*W, LW = 16*W;      // scalar, vec4, one component of 16 lanes
    // the core's own reset register: one cycle behind the input, replicated by synthesis near its loads
    (* max_fanout = 64 *) reg rst_q = 1'b1;
    always @(posedge clk) rst_q <= rst;
    localparam int AW = $clog2(IRAM_DEPTH), PAW = $clog2(NPARAM), WW = (NWARP > 1) ? $clog2(NWARP) : 1;
    localparam int TSW = $clog2(NTS), TAW = TSW + 5;                 // triangle store address {slot, vidx, comp}
    localparam int GAW = WW + 7;                                    // {warp, reg[4:0], comp[1:0]}
    localparam int MW = WW + 7 + 16;                                // result meta {warp, scalar address, lanes}
    localparam int TQW = (TEXQ > 1) ? $clog2(TEXQ) : 1, TLW = $clog2(TEXQ + 1);   // TEX entry pointers / live count
    localparam int TMW = WW + 5 + 4 + 16;                           // TEX meta {warp, rd, wm, lanes}
    localparam int ADDSUB_LAT = (MAN_W <= 10) ? 3 : 5, MADD_LAT = 3 + ADDSUB_LAT;
    localparam bit LDV1 = (LDV_MODE != 0), LDVR = (LDV_MODE == 1), LDV2P = (LDV_MODE == 2);   // one issued micro-op / the ring / the second adder
    localparam [W-1:0] ONE1 = (((1 << (EXP_W-1)) - 1) << MAN_W);
    localparam [LW-1:0] ONEL = {16{ONE1}}, ZEROL = '0;
    // opcodes
    localparam NOP=0,MOV=1,MOVI=2,ADD=3,SUB=4,MUL=5,MAD=6,MIN=7,MAX=8,DP3=9,DP4=10,RCP=11,RSQRT=12,LOG2=13,EXP2=14,
               SETP=15,LDP=16,LDV=17,STV=18,STO=19,BRA=20,STOZ=21,LOOP=22,ENDL=23,IFOP=24,ELSEOP=25,ENDIFOP=26,
               BREAKOP=27,CONTOP=28,DISCARD=29,FTOI=30,ITOF=31,DDX=32,DDY=33,TEX=34,SET=35,IADD=36,ISUB=37,ISLT=38,
               ISGE=39,SIN=40,COS=41,TRUNC=42,FLR=43,CEIL=44,RNDE=45,SEL=46,IMUL=47,IDIV=48,ISEQ=49,ISNE=50,IOP=51,
               IMOD=52,UDIV=53,UMOD=54,FTOU=55,UTOF=56,SQRT=58,RCPM=59,RSQM=60,CLAMP=61,ENDOP=63;
    // madd micro-op kinds (the chain steps route the madd's operands and target)
    localparam [2:0] MK_N = 0, MK_P1 = 1, MK_P2 = 2, MK_A1 = 3, MK_A2 = 4, MK_F = 5, MK_L0 = 6, MK_L1 = 7;

    // ================================================================ instruction word fields, classes, helpers
    function automatic [6:0] f_op (input [63:0] w); f_op  = w[63:57]; endfunction
    function automatic [3:0] f_wm (input [63:0] w); f_wm  = w[56:53]; endfunction
    function automatic [3:0] f_cm (input [63:0] w); f_cm  = w[9:6]; endfunction      // TEX: the coordinate set (0 = all four)
    function automatic [4:0] f_rd (input [63:0] w); f_rd  = w[52:48]; endfunction
    function automatic [4:0] f_rs (input [63:0] w, input [1:0] k);
        case (k) 2'd0: f_rs = w[47:43]; 2'd1: f_rs = w[32:28]; default: f_rs = w[17:13]; endcase
    endfunction
    function automatic [7:0] f_swz(input [63:0] w, input [1:0] k);
        case (k) 2'd0: f_swz = w[42:35]; 2'd1: f_swz = w[27:20]; default: f_swz = w[12:5]; endcase
    endfunction
    function automatic [1:0] f_mod(input [63:0] w, input [1:0] k);
        case (k) 2'd0: f_mod = w[34:33]; 2'd1: f_mod = w[19:18]; default: f_mod = w[4:3]; endcase
    endfunction
    function automatic [1:0] swz_c(input [7:0] swz, input [1:0] c); swz_c = swz[2*c +: 2]; endfunction
    // the cur-th set bit of wm (cur counts the micro-ops already issued)
    function automatic [1:0] comp_of(input [3:0] wm, input [1:0] cur);
        reg [2:0] n; n = 0; comp_of = 2'd0;
        for (int c = 0; c < 4; c++) if (wm[c]) begin if (n == 3'(cur)) comp_of = 2'(c); n = n + 1; end
    endfunction
    function automatic [2:0] popc(input [3:0] wm); popc = 3'(wm[0]) + 3'(wm[1]) + 3'(wm[2]) + 3'(wm[3]); endfunction
    function automatic is_madd(input [6:0] op); is_madd = (op==ADD)||(op==SUB)||(op==MUL)||(op==MAD)||(op==DDX)||(op==DDY); endfunction
    function automatic is_cmp (input [6:0] op); is_cmp  = (op==MIN)||(op==MAX)||(op==SET)||(XCLAMP && (op==CLAMP)); endfunction
    function automatic is_xsf (input [6:0] op); is_xsf  = XSFU && ((op==SQRT)||(op==RCPM)||(op==RSQM)); endfunction   // the unit's result times an operand
    function automatic is_xbad(input [6:0] op);        // the additions' opcodes in a core built without them
        is_xbad = (!XSFU && ((op==SQRT)||(op==RCPM)||(op==RSQM))) || (!XCLAMP && (op==CLAMP));
    endfunction
    function automatic is_int (input [6:0] op); is_int  = (op==IADD)||(op==ISUB)||(op==ISLT)||(op==ISGE)||(op==ISEQ)||(op==ISNE); endfunction
    function automatic is_one (input [6:0] op, input [63:0] w);  // the 1-cycle path (LDV: built-ins only)
        is_one = (op==MOV)||(op==MOVI)||(op==LDP)||((op==LDV) && w[3])||(op==SEL)||is_int(op);
    endfunction
    function automatic is_sfu (input [6:0] op); is_sfu  = (op==RCP)||(op==RSQRT)||(op==LOG2)||(op==EXP2)||is_xsf(op); endfunction
    function automatic is_sc  (input [6:0] op); is_sc   = (op==SIN)||(op==COS); endfunction
    function automatic is_cvt (input [6:0] op); is_cvt  = (op==FTOI)||(op==ITOF)||(op==FTOU)||(op==UTOF)||(op==TRUNC)||(op==FLR)||(op==CEIL)||(op==RNDE); endfunction
    function automatic is_idiv(input [6:0] op); is_idiv = (op==IDIV)||(op==IMOD)||(op==UDIV)||(op==UMOD); endfunction
    function automatic is_ctl (input [6:0] op);
        is_ctl = (op==IFOP)||(op==ELSEOP)||(op==ENDIFOP)||(op==LOOP)||(op==ENDL)||(op==BREAKOP)||(op==CONTOP)||(op==BRA)||(op==DISCARD);
    endfunction
    function automatic [LW-1:0] applymod(input [LW-1:0] v, input [1:0] m);
        for (int k = 0; k < 16; k++) begin
            applymod[k*W +: W] = v[k*W +: W];
            if (m == 2'b01) applymod[k*W+W-1] = ~v[k*W+W-1];
            else if (m == 2'b10) applymod[k*W+W-1] = 1'b0;
            else if (m == 2'b11) applymod[k*W+W-1] = 1'b1;
        end
    endfunction
    // integer slot (one adder per lane, as shader_core_fs) and SEL
    function automatic [LW-1:0] int_alu(input [LW-1:0] a, input [LW-1:0] b, input [2:0] iop);
        reg [W-1:0] x, z; reg [W:0] d; reg sub, r;
        sub = (iop != 3'd0);
        for (int k = 0; k < 16; k++) begin
            x = a[k*W +: W]; z = b[k*W +: W];
            d = {x[W-1], x} + ({z[W-1], z} ^ {(W+1){sub}}) + (W+1)'(sub);
            case (iop)
                3'd2: r = d[W];  3'd3: r = ~d[W];  3'd4: r = (x == z);  default: r = (x != z);
            endcase
            int_alu[k*W +: W] = (iop[2] | iop[1]) ? (r ? ONE1 : {W{1'b0}}) : d[W-1:0];
        end
    endfunction
    function automatic [LW-1:0] selv(input [LW-1:0] c, input [LW-1:0] a, input [LW-1:0] b);
        for (int k = 0; k < 16; k++) selv[k*W +: W] = (c[k*W+W-2 -: EXP_W] != 0) ? a[k*W +: W] : b[k*W +: W];
    endfunction
    // instruction classes, decoded once when a word enters a prefetch queue (the S stage reads registered bits)
    localparam int CLS_W = 32;
    localparam int C_MADD = 0, C_CMP = 1, C_ONE = 2, C_WR = 3, C_STO = 4, C_END = 5, C_N0 = 6, C_N1 = 7, C_N3 = 8, C_TEX = 9,
                   C_SFU = 10, C_SC = 11, C_CVT = 12, C_BAD = 13, C_IMUL = 14, C_IDIV = 15, C_QR = 16,
                   C_DP = 17, C_DP4 = 18, C_LDVI = 19, C_SETP = 20, C_STOZ = 21, C_CTL = 22,
                   // the control ops one bit each (registered head bits like the rest: decoding them in the S stage
                   // from the 7-bit opcode read out of the queue's LUTRAM would sit on the ready path)
                   C_IF = 23, C_ELSE = 24, C_ENDIF = 25, C_LOOP = 26, C_ENDL = 27, C_BRK = 28, C_CONT = 29, C_BRA = 30,
                   C_DISC = 31;
    function automatic [CLS_W-1:0] decode_cls(input [63:0] w);
        reg [6:0] op; reg ldvi; op = f_op(w); ldvi = (op == LDV) && !w[3];
        decode_cls = '0;
        decode_cls[C_MADD] = is_madd(op); decode_cls[C_CMP] = is_cmp(op); decode_cls[C_ONE] = is_one(op, w);
        decode_cls[C_STO] = (op == STO); decode_cls[C_END] = (op == ENDOP);
        decode_cls[C_SFU] = is_sfu(op); decode_cls[C_SC] = is_sc(op); decode_cls[C_CVT] = is_cvt(op);
        // IOP (the bit operations, shifts, integer min / max / abs / sign, unsigned compares): no language of the GL <= 2.0
        // era can express them and no compiler emits them — not in board B's hardware;
        // an IOP at the head kills its warp like a stack violation (C_BAD)
        decode_cls[C_BAD] = (op == IOP) || is_xbad(op); decode_cls[C_IMUL] = (op == IMUL); decode_cls[C_IDIV] = is_idiv(op);
        // IMUL and the divider are quarter-rate classes (4 lanes x 4 cycles on the 4-lane integer unit)
        decode_cls[C_QR] = is_sfu(op)||is_sc(op)||is_cvt(op)||(op == IMUL)||is_idiv(op);
        decode_cls[C_IF] = (op == IFOP); decode_cls[C_ELSE] = (op == ELSEOP); decode_cls[C_ENDIF] = (op == ENDIFOP);
        decode_cls[C_LOOP] = (op == LOOP); decode_cls[C_ENDL] = (op == ENDL); decode_cls[C_BRK] = (op == BREAKOP);
        decode_cls[C_CONT] = (op == CONTOP); decode_cls[C_BRA] = (op == BRA); decode_cls[C_DISC] = (op == DISCARD);
        decode_cls[C_DP] = (op == DP3)||(op == DP4); decode_cls[C_DP4] = (op == DP4); decode_cls[C_LDVI] = ldvi;
        decode_cls[C_SETP] = (op == SETP); decode_cls[C_STOZ] = (op == STOZ); decode_cls[C_CTL] = is_ctl(op);
        decode_cls[C_WR] = is_madd(op)||is_cmp(op)||is_one(op, w)||is_sfu(op)||is_sc(op)||is_cvt(op)||
                           (op==IMUL)||is_idiv(op)||(op==DP3)||(op==DP4)||ldvi;
        decode_cls[C_N0] = (op==MOVI)||(op==LDP)||(op==LDV)||(op==NOP)||(op==ENDOP)||is_ctl(op);
        // (STO2: a STO micro-op reads two components of rs0, the second one through source 1's address and hazard bit)
        // (RCPM / RSQM read two registers — the unit's argument and the operand its result is multiplied by; SQRT one)
        decode_cls[C_N1] = (op==MOV)||(op==DDX)||(op==DDY)||((op==STO) && !STO2)||(op==STOZ)||
                           (is_sfu(op) && !(XSFU && ((op==RCPM)||(op==RSQM))))||is_sc(op)||is_cvt(op);
        decode_cls[C_N3] = (op==MAD)||(op==SEL)||(XCLAMP && (op==CLAMP));
        decode_cls[C_TEX] = (op == TEX);
    endfunction
    // the three components other than c, ascending: {o2, o1, o0}
    function automatic [5:0] rd_others(input [1:0] c);
        case (c) 2'd0: rd_others = {2'd3, 2'd2, 2'd1}; 2'd1: rd_others = {2'd3, 2'd2, 2'd0};
                 2'd2: rd_others = {2'd3, 2'd1, 2'd0}; default: rd_others = {2'd2, 2'd1, 2'd0}; endcase
    endfunction
    // micro-ops of an instruction
    function automatic [3:0] n_uops(input [CLS_W-1:0] cls, input [3:0] wm);
        if (cls[C_DP4]) n_uops = 4'd4 + 4'(popc(wm));
        else if (cls[C_DP]) n_uops = 4'd3 + 4'(popc(wm));
        // LDV1: one micro-op per component (its lambda2 step is not issued); else lambda1 steps at ui = 0.., lambda2 at ui = 4..
        else if (cls[C_LDVI]) n_uops = LDV1 ? 4'(popc(wm)) : (4'd4 + 4'(popc(wm)));
        else if (cls[C_STO]) n_uops = STO2 ? 4'd2 : 4'd4;
        // TEX (wm = the coordinate set here, uop_dec): TEX2 one micro-op per PAIR (x y / z w) the set touches, else one per
        // component — and at least two: the micro-ops carry the checks for earlier writers of rd, three components in the
        // first and the fourth in the second (a one-coordinate set repeats its coordinate; no compiler emits one)
        else if (cls[C_TEX]) n_uops = TEX2 ? (4'(|wm[1:0]) + 4'(|wm[3:2])) : ((popc(wm) < 3'd2) ? 4'd2 : 4'(popc(wm)));
        else if (cls[C_WR]) n_uops = 4'(popc(wm));
        else n_uops = 4'd1;
    endfunction
    // the fields of micro-op ui of an instruction, computed one cycle ahead and registered per warp (S starts its
    // hazard lookups from registers): {first, last, dpfin, mk[2:0], rc[1:0], wc[1:0], s0a, s1a, s2a, rda (7 bits each)}
    localparam int UW = 38;
    function automatic [UW-1:0] uop_dec(input [63:0] w, input [CLS_W-1:0] c, input [2:0] ui);
        reg [3:0] wm, tm, nu; reg [2:0] nst, mk; reg dpfin, last, sto2, tex2, tex1, pr; reg [1:0] dp_rc, cur, wc, rc, c2;
        reg [5:0] oth;
        wm = c[C_STO] ? 4'hF : f_wm(w);
        sto2 = STO2 & c[C_STO];
        tex2 = TEX2 & c[C_TEX];
        tex1 = ~TEX2 & c[C_TEX];
        // TEX: one micro-op per component of its coordinate set (ISA bits [9:6]; 0 = all four, the legacy encoding):
        // a 2D sampler's TEX fetches s, t (+ the bias / lod in w) only — 2 or 3 micro-ops instead of 4
        tm = c[C_TEX] ? ((f_cm(w) == 4'd0) ? 4'hF : f_cm(w)) : wm;
        nu = n_uops(c, tm);
        last = (4'(ui) + 4'd1 == nu);
        nst = c[C_DP4] ? 3'd4 : 3'd3;                               // DP: accumulator steps before the finals
        dpfin = c[C_DP] & (ui >= nst);
        dp_rc = (ui == 3'd0) ? 2'd0 : (ui == 3'd1) ? 2'd2 : (ui == 3'd2) ? 2'd1 : 2'd3;
        cur = c[C_DP] ? 2'(ui - nst) : ui[1:0];
        if (tex1 && (3'(ui) >= popc(tm))) cur = 2'(popc(tm) - 3'd1);  // (a one-coordinate set's second micro-op: the coordinate again)
        wc = comp_of(tm, cur);                                      // TEX micro-op k: the k-th coordinate of its set
        // TEX, the earlier writers of rd (WAW). A TEX reads one register, so the hazard bits of sources 1 and 2 are free:
        // their addresses name components of rd — the first micro-op checks the three components other than the set's
        // first (that one through rda, as always), every later one the remaining fourth. Each bit counts only if the TEX
        // writes that component (masked with wm where the hazard bits load). Complete for every coordinate set
        // (tests/fs2_texwaw.sasm).
        oth = rd_others(comp_of(tm, 2'd0));
        // STO2: micro-op k stores components 2k (source 0's address: rs0[swz[2k]]) and 2k + 1 (source 1's address:
        // rs0[swz[2k + 1]], read through copy 1 with source 0's modifier)
        if (sto2) wc = {ui[0], 1'b0};
        // TEX2: one micro-op per coordinate PAIR the set touches (pair 0 = x y, pair 1 = z w): the even component
        // through source 0 / read copy 0, the odd one through source 1's address rs0[swz[odd]] / read copy 1
        // — so each staging component has ONE source (pairing in set order would put a 2 : 1 mux on its 1 536
        // bits). A pair with one component in the set reads it on both (wc = c2). The 2D set is one micro-op, .xyw /
        // .xyz / all four are two. c2 travels to X in source 2's component field, which a TEX does not use. (The
        // earlier writers of rd: hzr below.)
        c2 = wc;
        if (tex2) begin
            pr = ui[0] | ~(|tm[1:0]);                                // the micro-op's pair: the first present one, then pair 1
            wc = tm[{pr, 1'b0}] ? {pr, 1'b0} : {pr, 1'b1};
            c2 = tm[{pr, 1'b1}] ? {pr, 1'b1} : {pr, 1'b0};
        end
        rc = (c[C_DP] & ~dpfin) ? dp_rc : wc;
        mk = MK_N;
        if (c[C_DP]) mk = dpfin ? MK_F : (ui == 3'd0) ? MK_P1 : (ui == 3'd1) ? MK_P2 : (ui == 3'd2) ? MK_A1 : MK_A2;
        else if (c[C_LDVI]) mk = ui[2] ? MK_L1 : MK_L0;
        uop_dec = {(ui == 3'd0), last, dpfin, mk, rc, wc,
                   f_rs(w, 2'd0), swz_c(f_swz(w, 2'd0), rc),
                   tex2 ? {f_rs(w, 2'd0), swz_c(f_swz(w, 2'd0), c2)} :
                   tex1 ? {f_rd(w), (ui == 3'd0) ? oth[1:0] : oth[5:4]} :
                   sto2 ? {f_rs(w, 2'd0), swz_c(f_swz(w, 2'd0), rc | 2'd1)} : {f_rs(w, 2'd1), swz_c(f_swz(w, 2'd1), rc)},
                   tex2 ? {f_rd(w), c2} :
                   tex1 ? {f_rd(w), (ui == 3'd0) ? oth[3:2] : oth[5:4]} : {f_rs(w, 2'd2), swz_c(f_swz(w, 2'd2), rc)}, f_rd(w), wc};
    endfunction
    // the index of the micro-op after ui, kept in a register of its own so uop_dec starts from a register: an LDV goes
    // from its last lambda1 step to ui = 4 (a compare + subtract inside uop_dec costs ≈ 1 ns in the hz paths)
    function automatic [2:0] uop_nxt(input [63:0] w, input [CLS_W-1:0] c, input [2:0] ui);
        uop_nxt = (!LDV1 && c[C_LDVI] && (ui + 3'd1 == popc(f_wm(w)))) ? 3'd4 : ui + 3'd1;
    endfunction

    // ================================================================ program, uniforms, built-in rows, triangle store
    (* ram_style="block" *) reg [63:0] iram [0:2*IRAM_DEPTH-1];      // two banks, {bank, pc}
    (* ram_style="block" *) reg [TW-1:0] params [0:2*NPARAM-1];
    always @(posedge clk) if (iram_we) iram[iram_waddr] <= iram_wdata;
    always @(posedge clk) if (param_we) params[param_idx] <= param_data;
    reg [NWARP-1:0] bank;                                           // the warps' banks
    // ap addresses every lane's varying store, x_w every lane's GPR copies and the accumulators: hundreds of loads across
    // the die (≈ 9 ns of route on 0–1 logic levels from one register) — max_fanout lets synthesis replicate them
    (* max_fanout = 16 *) reg [WW-1:0] ap;
    reg [WW-1:0] rp;                                                // allocate / retire pointers
    reg         x_valid; (* max_fanout = 32 *) reg [WW-1:0] x_w; reg [63:0] x_ins; reg [1:0] x_rc, x_wc; reg [2:0] x_mk; reg [15:0] x_am, x_sm;
    reg         x_first, x_last;                                    // the micro-op is its instruction's first / last
    // a two-component micro-op (STO2's STO, TEX2's TEX: the issuing warp's class bit): read copy 1 fetches rs0's
    // component x_s1c (the micro-op's source-1 address) instead of rs1; x_c2 = a TEX's second coordinate
    reg         x_two; reg [1:0] x_s1c, x_c2;
    wire        x_ldvi = (f_op(x_ins) == LDV) & ~x_ins[3];
    // built-in storage: per lane two LUTRAMs (component 0 / 1 of a row) x {warp, row}; written one quad-row per cycle
    // for the warp being launched (ap), read in X2 at the registered address; gl_FrontFacing = one bit per quad
    localparam int VAW = WW + 2;
    reg  [VAW-1:0] x2_va; reg x2_vh; reg [1:0] x2_vk;                 // address, component half, kind (0 memory 1 zero 2 one 3 front)
    reg  [3:0] ffq [0:NWARP-1];
    (* ram_style = "distributed" *) reg [15:0] tmk [0:NWARP-1];     // the warps' texel-use masks (in_tmask)
    wire [LW-1:0] ve_q, vo_q;
    genvar gq;
    generate for (gq = 0; gq < 16; gq = gq + 1) begin : VL
        (* ram_style="distributed" *) reg [W-1:0] ve [0:NWARP*4-1];
        (* ram_style="distributed" *) reg [W-1:0] vo [0:NWARP*4-1];
        always @(posedge clk) if (vw_we && vw_quad == 2'(gq >> 2)) begin
            ve[{ap, vw_row}] <= vw_data[(gq & 3)*2*W +: W]; vo[{ap, vw_row}] <= vw_data[(gq & 3)*2*W + W +: W];
        end
        assign ve_q[gq*W +: W] = ve[x2_va]; assign vo_q[gq*W +: W] = vo[x2_va];
    end endgenerate
    // triangle store: entry {slot, vidx, comp} = {D2.c, D1.c, f0.c}; 4 block-RAM copies (one per quad), written by
    // a 4-cycle component writer, read in X at the quad's slot
    reg        tw_run; reg [1:0] tw_c; reg [TSW-1:0] tw_slot; reg [2:0] tw_vidx; reg [3*TW-1:0] tw_data;
    assign tri_ready = ~tw_run;
    always @(posedge clk) begin
        if (rst_q) begin tw_run <= 1'b0; tw_c <= 2'd0; end
        else if (tri_we & ~tw_run) begin tw_run <= 1'b1; tw_c <= 2'd0; tw_slot <= tri_slot; tw_vidx <= tri_vidx; tw_data <= tri_data; end
        else if (tw_run) begin tw_c <= tw_c + 2'd1; if (tw_c == 2'd3) tw_run <= 1'b0; end
    end
    wire [3*W-1:0] tw_entry = {tw_data[2*TW + tw_c*W +: W], tw_data[TW + tw_c*W +: W], tw_data[tw_c*W +: W]};
    reg  [4*TSW-1:0] ts [0:NWARP-1];                                // the quads' slots per warp
    wire [3*W-1:0] trd [0:3];
    generate for (gq = 0; gq < 4; gq = gq + 1) begin : TS
        (* ram_style="block" *) reg [3*W-1:0] tmem [0:NTS*32-1];
        reg [3*W-1:0] q;
        wire [TAW-1:0] raddr = {ts[x_w][gq*TSW +: TSW], x_ins[2:0], swz_c(f_swz(x_ins, 2'd0), x_rc)};
        always @(posedge clk) begin
            if (tw_run) tmem[{tw_slot, tw_vidx, tw_c}] <= tw_entry;
            q <= tmem[raddr];
        end
        assign trd[gq] = q;
    end endgenerate

    // ================================================================ stage registers and shared state (declared first)
    localparam int WBW = 16;                        // write slots ahead
    (* shreg_extract = "no" *) reg [WBW-1:0] wb_busy;  // bit j: a GPR write j cycles from now (from this S cycle); flops, not an SRL
                                                   // (as SRLs, synthesis merges bit 7 with mdl_v and slows q_rp -> wb_busy)
    reg  x2_valid; reg [WW-1:0] x2_w; reg [1:0] x2_wc; reg [15:0] x2_am, x2_sm; reg [4:0] x2_rd; reg [2:0] x2_mk; reg x2_ll;
    reg  x2_madd, x2_cmp, x2_setp, x2_one, x2_sto, x2_stoz, x2_int, x2_sel, x2_ldp, x2_ldv, x2_movi, x2_sub, x2_ddx, x2_ddy;
    reg  x2_qr, x2_sfu, x2_sc, x2_cvt, x2_imul, x2_idiv;
    reg  x2_sfum, x2_sqrt;                          // XSFU: the unit's result is multiplied by an operand / by its own argument (SQRT)
    reg  [1:0] x2_cmpop, x2_m0, x2_m1, x2_m2, x2_sc_c, x2_madm, x2_sfuop, x2_divmode, x2_tgt; reg [2:0] x2_intop, x2_cvtop;
    reg  x2_cos; reg [W-1:0] x2_imm; reg [2:0] x2_cc;
    reg  x2_tex, x2_tfirst, x2_tlast; reg [1:0] x2_tc, x2_tc2, x2_mode; reg [3:0] x2_samp;   // TEX coordinate micro-op: component x2_tc (and x2_tc2, TEX2)
    reg  [TW-1:0] x2_pq;
    wire [LW-1:0] ar, br, cr;                       // X2 operands (register copies 0, 1, 2)
    wire [LW-1:0] a_m, b_m, c_m;                    // with the source modifiers
    reg  ec_v; reg [WW-1:0] ec_w; reg [6:0] ec_a;  // scoreboard clears: a scheduled result one cycle before its write
    reg  bw_v; reg [WW-1:0] bw_w; reg [6:0] bw_a;  // a buffered result at its write
    reg  [NWARP-1:0] odw;                          // the warp wrote gl_FragDepth (the output store itself: W stage)
    reg  [15:0] pred [0:NWARP-1]; reg [NWARP-1:0] pred_pend;
    // chain accumulators in LUTRAM (read in X for the X2 operand network, written at a chain step's landing): acc1 per
    // {warp, component} (a DP uses entry 0), acc2 per warp
    // LDV1: acc1 = one DP entry per warp (address = the warp) and above them a ring of RN entries for the LDVs' first
    // results (the ring pointers are acc1 addresses NWARP .. ADEP-1); else one entry per {warp, component}
    localparam int RN = 24;
    localparam int ADEP = LDVR ? (NWARP + RN) : (NWARP * 4), AAW = $clog2(ADEP);
    (* ram_style="distributed" *) reg [LW-1:0] acc1 [0:ADEP-1];
    function automatic [AAW-1:0] nxt_r(input [AAW-1:0] p); nxt_r = (p == AAW'(ADEP-1)) ? AAW'(NWARP) : p + 1'b1; endfunction
    (* ram_style="distributed" *) reg [LW-1:0] acc2 [0:NWARP-1];
    reg  [LW-1:0] acc1_x, acc2_x; reg [NWARP-1:0] acc1_rdy, acc2_rdy;
    // the LDV accumulator's readiness per {warp, component}: a lambda2 step waits
    // for ITS component's lambda1 step, not for the last one; readiness is set one madd stage before the landing (the
    // LUTRAM is written at the end of the landing cycle and X reads it a cycle after an issue), the DP accumulators too
    reg  [3:0] ldv_rdy [0:NWARP-1];
    wire acc1_land, acc2_land, ldv_land; wire [WW-1:0] acc_land_w; wire [1:0] ldv_land_c;   // a chain step readies an accumulator (one stage before it lands)
    // LDV1, the second steps: rg_a = the ring entry the next first step in X2 takes (D2 and the write's meta stored
    // there), rg_l = the entry the next first result lands in, rg_r = the oldest waiting one, rg_lq = results landed
    // and not yet started. fire_n: the oldest starts next cycle (au_r: it is at the lanes' inputs, its meta in au_m, D2
    // in d2_r, the first result in acc1_x, lambda2 on the built-in read). mblk: no multiply-add micro-op issues.
    reg  mblk, au_r;
    reg  [AAW-1:0] rg_a, rg_l, rg_r; reg [4:0] rg_lq; reg [3:0] rg_wait;
    (* ram_style="distributed" *) reg [4*W-1:0] d2s [0:ADEP-1];
    (* ram_style="distributed" *) reg [MW-1:0]  rmeta [0:ADEP-1];
    reg  [MW-1:0] au_m; reg [4*W-1:0] d2_r;
    wire fire_n;
    // quarter-rate feeder / in-flight accounting (S reads these)
    reg  [2:0] qr_hold;                            // cycles until the next quarter-rate micro-op may issue
    // (the class is full at QR_INF: the flag is a register of its own — the count's compare is not on the issue path)
    localparam int QIW = $clog2(QR_INF + 1);       // an in-flight count 0..QR_INF
    reg  [QIW-1:0] inf_sfu, inf_sc, inf_cvt, inf_imul; reg inf_idiv;
    reg  qf_sfu, qf_sc, qf_cvt, qf_imul;
    wire idiv_ready;
    // TEX: the coordinate staging's owner (first micro-op issued, last request not yet accepted), TEXes live, their
    // meta in issue order (results return in the same order)
    reg  tex_own_v; reg [TLW-1:0] tex_live; reg tex_full;
    reg  [TMW-1:0] tmeta [0:TEXQ-1]; reg [TQW-1:0] tm_wp, tm_own;   // the staging owner's entry
    reg  tmw_v; reg [TQW-1:0] tmw_e; reg [TMW-1:0] tmw_d;          // the meta write, one cycle after the S decision (off
                                                                   // the issue -> LUTRAM WE path; the entry is read >= 3 cycles later)
    wire bw_tex, tex_last, seq_done;               // the head TEX writes a component this cycle / its last one; last request accepted
    function automatic [TQW-1:0] nxt_t(input [TQW-1:0] p); nxt_t = (p == TQW'(TEXQ-1)) ? '0 : p + 1'b1; endfunction
    // prefetch (FETCH 2 of shader_core_fs): grant fgo / fsel, the RAM's read fr_*, its output register fq_*. fq_ins is the
    // block RAM's output register, 1.4 ns faster out of the RAM (on the path iram -> fq_cls -> uop_dec -> hz): one more
    // fetch cycle, seen after a taken branch and at a launch, otherwise hidden by the queue
    localparam int QCW = $clog2(QD + 1);
    localparam int QPW = (QD > 1) ? $clog2(QD) : 1;             // queue pointers (QD a power of two)
    if ((QD & (QD - 1)) != 0) $error("shader_core_fs2: QD must be a power of 2");
    wire [NWARP-1:0] qreq_v, qlow_v;
    wire [AW-1:0] qfpc_v [0:NWARP-1];
    wire [1:0]    qep_v  [0:NWARP-1];
    reg           fgo; reg [WW-1:0] fsel, f_rr;
    reg           fgo_r; reg [WW-1:0] fsel_r;                       // the grant registered: the read happens then
    reg  [63:0]   fr_ins; reg fr_v; reg [WW-1:0] fr_w; reg [1:0] fr_ep; reg [AW-1:0] fr_pc;
    reg  [63:0]   fq_ins; reg fq_v; reg [WW-1:0] fq_w; reg [1:0] fq_ep; reg [AW-1:0] fq_pc;
    wire [CLS_W-1:0] fq_cls = decode_cls(fq_ins);
    // per-warp S outputs
    wire [NWARP-1:0] rdy_v, busy_v, fin_v, err_v, first_v, last_v, tex1_v, two_v;
    wire [NWARP-1:0] pa_v, ph_v;                                    // (profile) the warp runs (busy, not finished, not killed) / its head micro-op is loaded
    wire [3:0]  xc_v [0:NWARP-1];                                   // {s1a[1:0], s2a[1:0]} of each warp's head micro-op
    wire [WBW-1:0] wbm_v [0:NWARP-1];                               // the wb_busy bit the warp's issue would set
    wire [63:0]   ins_v [0:NWARP-1];
    wire [CLS_W-1:0] cls_v [0:NWARP-1];
    wire [1:0]    rc_v [0:NWARP-1], wc_v [0:NWARP-1];
    wire [2:0]    mk_v [0:NWARP-1];
    wire [15:0]   am_v  [0:NWARP-1], sm_v [0:NWARP-1], kill_v [0:NWARP-1];
    reg  [WW-1:0] sel; reg issue;

    // ================================================================ warp bookkeeping
    wire launch = in_start & ~busy_v[ap];
    assign in_ready = ~busy_v[ap];
    assign in_ready_nx = ~busy_v[nxt(ap)];                          // the slot after ap is free (it becomes ap at a launch)
    assign in_warp  = 2'(ap);
    // output latch: the oldest finished warp leaves its slot when the latch is free or being freed (a warp's short life
    // would otherwise include the back end's 3 cycles per quad); ol_w = the slot whose output store the latch reads,
    // ol_own the same one-hot (the slot's next warp stalls at STO / STOZ)
    reg ol_v; reg [WW-1:0] ol_w; reg [NWARP-1:0] ol_own; reg [15:0] ol_kill; reg ol_dw, ol_err;
    wire retire = busy_v[rp] & fin_v[rp] & (~ol_v | out_ack);
    assign out_valid = ol_v;
    assign idle = ~|busy_v & ~ol_v;
    always @(posedge clk) begin
        if (rst_q) begin ol_v <= 1'b0; ol_own <= '0; end
        else if (retire) begin ol_v <= 1'b1; ol_own <= NWARP'(1) << rp; end
        else if (out_ack) begin ol_v <= 1'b0; ol_own <= '0; end
        if (retire) begin ol_w <= rp; ol_kill <= kill_v[rp]; ol_dw <= odw[rp]; ol_err <= err_v[rp]; end
    end
    function automatic [WW-1:0] nxt(input [WW-1:0] p); nxt = (p == WW'(NWARP-1)) ? '0 : p + 1'b1; endfunction
    always @(posedge clk) begin
        if (rst_q) begin ap <= '0; rp <= '0; err_stack <= 1'b0; bank <= '0; end
        else begin
            if (pre_start & ~busy_v[ap]) bank[ap] <= pre_bank;      // the program fetch ahead of the launch reads this bank
            if (launch) begin ap <= nxt(ap); ts[ap] <= in_tslot; ffq[ap] <= in_front; bank[ap] <= in_bank; tmk[ap] <= in_tmask; end
            if (retire) rp <= nxt(rp);
            if (|err_v) err_stack <= 1'b1;
        end
    end
    // oldest ready warp first. The grant is formed per warp (the select, then "sel == gw" per warp, then
    // the pending updates would be one long path): grant[w] = w ready and no warp ahead of it in rp order ready — a
    // function of rdy_v and rp only; sel / issue follow from the grants
    function automatic [NWARP-1:0] older_of(input int w, input int r);  // the warps ahead of w in the order rp, rp+1, ..
        older_of = '0;
        for (int v = 0; v < NWARP; v++) if ((v - r + NWARP) % NWARP < (w - r + NWARP) % NWARP) older_of[v] = 1'b1;
    endfunction
    wire [NWARP-1:0] grant;
    genvar gg;
    generate for (gg = 0; gg < NWARP; gg = gg + 1) begin : GR
        assign grant[gg] = rdy_v[gg] & ~|(rdy_v & older_of(gg, int'(rp)));
    end endgenerate
    // sel = the OR-encode of the one-hot grant, one LUT per bit (a priority chain over the grants with 8:1 binary
    // muxes behind it costs two levels and 1.4 ns of select fan-out at 8 warps); sel is read only under issue
    always @* begin
        issue = |rdy_v; sel = '0;
        for (int k = 0; k < NWARP; k++) sel = sel | ({WW{grant[k]}} & WW'(k));
    end

    // ================================================================ per-warp state (S stage)
    genvar gw;
    generate for (gw = 0; gw < NWARP; gw = gw + 1) begin : WRP
        reg busy, fin, err;
        reg [15:0] active, killed;
        reg [127:0] pending;                                        // one bit per scalar register {reg, comp}
        reg [2:0]  uidx;                                            // the index of the micro-op after u (uop_nxt)
        reg [15:0] st_ent [0:7], st_then [0:7]; reg [3:0] ssp;      // IF stack
        reg ssp_z, ssp_f;                                           // ssp == 0 / == 8, registers (keeps the compares out of
                                                                    // the issue chain), as lsp_z / lsp_f
        reg [15:0] lcnt [0:3]; reg [AW-1:0] lpc [0:3]; reg [2:0] lsp; // loop stack: the levels BELOW the top (see lt_*)
        reg [15:0] lent [0:3], lmask [0:3], cmask [0:3];            // entering / still looping / continued lanes
        // the loop stack's top as REGISTERS (otherwise `lsp` -> the lsp-1 index -> the loop-live mask
        // and the ENDL decision -> ready -> the grant -> the masks' writes is one long path):
        // lt_* = entry lsp-1 whenever lsp > 0; a push writes the old top back into the arrays at lsp-1, a pop reloads the
        // top from entry lsp-2; BRK / CONT / a repeated ENDL change the top only. lsp_z / lsp_f = lsp == 0 / == 4.
        reg [15:0] lt_cnt, lt_ent, lt_mask, lt_cmask; reg [AW-1:0] lt_pc; reg lsp_z, lsp_f;
        wire       lt_again = (lt_cnt > 16'd1) & (lt_mask != 16'd0);   // the ENDL repeats
        // circular prefetch queue (head = entry q_rp, writes at q_wp; a pop moves the read pointer, not the entries:
        // moving them fans the pop's enable out to every queue register), the classes decoded at
        // the queue write
        reg [63:0] q_ins [0:QD-1]; reg [AW-1:0] q_pc [0:QD-1]; reg [CLS_W-1:0] q_cls [0:QD-1];
        reg [QCW-1:0] q_cnt, q_alloc; reg [AW-1:0] q_fpc; reg [1:0] q_ep;
        reg [QPW-1:0] q_rp, q_wp;
        reg [QPW-1:0] q_rp1, q_rp2;                                 // q_rp + 1 / + 2, registers
        // the head word and the word after it are REGISTERS mirroring q_ins[q_rp] / q_ins[q_rp1] (they cut the path
        // pointer -> LUTRAM read -> uop_dec -> the 128 : 1 pending lookup -> hz, mostly route; q_rp1 is a register
        // for the same path). A pop moves
        // the next word up and refills the next from entry q_rp2 (or the word being written); a write into an empty /
        // one-entry queue lands in them directly. Cycle-exact: every read of ins / nx_ins sees the same values.
        reg [63:0] h_ins, n_ins; reg [CLS_W-1:0] n_cls;
        wire [63:0] ins = h_ins;
        wire [AW-1:0] ipc = q_pc[q_rp];
        // the head's classes and write mask are REGISTERS, loaded with u (`q_cls[q_rp]`, an asynchronous LUTRAM
        // read, would head the select paths — 1.9 ns from the pointer to the first LUT of the ready logic); the
        // control ops are class bits too, so nothing on the ready path decodes the opcode
        reg  [CLS_W-1:0] cls; reg [3:0] wm;
        wire        iv  = (q_cnt != '0);
        wire [15:0] imm = ins[15:0];
        // the head micro-op's fields (registered: loaded with the next micro-op's at issue, with the next head's at
        // a pop, with the first word's when the queue was empty)
        reg  [UW-1:0] u;
        wire        first = u[37], last = u[36], dpfin = u[35];
        wire [2:0]  mk = u[34:32];
        wire [1:0]  rc = u[31:30], wc = u[29:28];
        wire [6:0]  s0a = u[27:21], s1a = u[20:14], s2a = u[13:7], rda = u[6:0];
        wire [63:0] nx_ins = (q_cnt > QCW'(1)) ? n_ins : fq_ins;   // the word after the head (the queue's refill of h_ins)
        wire [3:0]  n_wm  = n_cls[C_STO]  ? 4'hF : f_wm(n_ins);
        wire [3:0]  fq_wm = fq_cls[C_STO] ? 4'hF : f_wm(fq_ins);
        wire madd_u = cls[C_MADD] | cls[C_DP] | cls[C_LDVI];       // a madd micro-op
        // this micro-op writes rd.wc (LDV1: an LDV's one micro-op does, through its automatic second step)
        wire gpr_u  = cls[C_WR] & ~(cls[C_DP] & ~dpfin) & ~(cls[C_LDVI] & (mk == MK_L0) & ~LDV1);
        // hazards
        wire rd0 = ~cls[C_N0] & ~dpfin & ~cls[C_LDVI];              // reads rs0 this micro-op (TEX: coordinate k)
        wire rd1 = rd0 & ~cls[C_N1] & ~cls[C_LDVI] & ~(cls[C_TEX] & ~TEX2);   // (TEX2: the second coordinate)
        wire rd2 = cls[C_N3];
        wire acc_need = (mk == MK_A1) | (mk == MK_F);                // the DP accumulator (entry 0)
        wire acc2_need = (mk == MK_A2) | (mk == MK_F);
        // TEX micro-op k reads its coordinate(s) (the RAW checks of s0a and, TEX2, s1a) and the TEX waits for every
        // earlier writer of a component of rd it writes (the WAW check on all of rd.wm: through the hazard bits of
        // sources 1 / 2 and rda, whose addresses uop_dec points at rd's components — see there; TEX2, whose one
        // micro-op has no free bits left for it: hzr below). rd.wm turns pending at the LAST micro-op (so TEX rd,
        // rd.xxxx cannot deadlock on its own bits; nothing of this warp issues in between, so a component checked at
        // any one micro-op stays clear). The first micro-op also needs the coordinate staging unowned and a live slot.
        // (Checking rd.k at micro-op k alone is not enough with coordinate sets: a 2D TEX would check rd.x and rd.y
        // only, and an earlier write to rd.z / rd.w still
        // in flight — a multiply-add or a TEX of the other arm of an IF — would clear the TEX's pending bit when it lands,
        // so a later read would take that value instead of the texel: tests/fs2_texwaw.sasm.)
        wire        tex_first = cls[C_TEX] & first;
        wire        tex_h = tex_first & (tex_own_v | tex_full);     // tex_full: the registered compare
        // registered operand hazards (otherwise pending[s0a] (a 128:1 lookup) -> stall -> arbiter -> the other warps'
        // pending updates is one 14-level path): hz[k] = pending[address k of u] for s0a, s1a,
        // s2a, rda. Recomputed (hc) from the micro-op u is about to load — u_cand = what u gets at an issue or at an
        // empty-queue fill — with this cycle's scoreboard clears and this warp's own sets applied; while u holds, the
        // clears of its addresses take the bits down (sets come only from this warp's issue, which reloads u)
        wire iss;
        // the hazard candidate's addresses are REGISTERS (otherwise the next-word select -> uop_dec ->
        // the 128 : 1 pending lookup -> hz is one 12-level path): sa_r = the addresses of the micro-op after u in
        // the same instruction, na_r = those of the next word's first micro-op (mirroring n_ins), fa_r = those of a word that
        // arrived into a dry queue. Such a word's head micro-op loads ONE CYCLE after it arrives (fill_d; the queue is dry
        // only after a launch or a taken branch), so every load of u / hz reads the addresses from registers.
        reg  fill_d; reg [27:0] sa_r, na_r, fa_r;                  // (fill_q / pop_dry / u_load below, after q_wr)
        // the micro-op u loads: the same instruction's next (iss, not last), the next word's first (iss, last) or the
        // arrived word's first (fill_d); sa_n = the micro-op after that one (next cycle's u_same). BOTH candidates are
        // decoded from registers and the grant only selects between finished results (with the select in front of the
        // decodes, the grant -> the word select -> uop_nxt -> uop_dec -> sa_r is 11 levels, 7 ns of route). The select
        // itself is registers only: fill_d
        // (no issue that cycle, hv is off) picks the same word at index 0, else `last` picks the next word.
        wire [2:0]       us_i  = fill_d ? 3'd0 : uidx;
        wire [UW-1:0]    us_ld = uop_dec(ins, cls, us_i);           // the same word's micro-op us_i
        wire [2:0]       us_in = uop_nxt(ins, cls, us_i);
        wire [UW-1:0]    us_sa = uop_dec(ins, cls, us_in);
        wire [UW-1:0]    un_ld = uop_dec(n_ins, n_cls, 3'd0);       // the next word's first micro-op
        wire [2:0]       un_in = uop_nxt(n_ins, n_cls, 3'd0);
        wire [UW-1:0]    un_sa = uop_dec(n_ins, n_cls, un_in);
        wire             ul_nx = last & ~fill_d;                     // = (iss & last) whenever u_load
        wire [UW-1:0]    u_ld  = ul_nx ? un_ld : us_ld;
        wire [2:0]       ul_in = ul_nx ? un_in : us_in;
        wire [UW-1:0]    sa_n  = ul_nx ? un_sa : us_sa;
        wire [UW-1:0]    na_src = uop_dec((q_cnt > QCW'(2)) ? q_ins[q_rp2] : fq_ins, (q_cnt > QCW'(2)) ? q_cls[q_rp2] : fq_cls, 3'd0);   // n_ins at a pop
        wire [UW-1:0]    fq_dec = uop_dec(fq_ins, fq_cls, 3'd0);     // the arriving word's first micro-op
        wire [UW-1:0] u_cand = u_ld;                                 // (hl_c's c_wc / c_mk only)
        wire [27:0] ca_r = fill_d ? fa_r : (last ? na_r : sa_r);
        wire [6:0] ca [0:3], ua [0:3];
        assign ca[0] = ca_r[27:21]; assign ca[1] = ca_r[20:14]; assign ca[2] = ca_r[13:7]; assign ca[3] = ca_r[6:0];
        assign ua[0] = s0a; assign ua[1] = s1a; assign ua[2] = s2a; assign ua[3] = rda;
        wire ecw = ec_v & (ec_w == WW'(gw)), bww = bw_v & (bw_w == WW'(gw));
        reg [3:0] hz, hc, hclr;
        // the scoreboard SET one cycle late (iss — the end of the arbiter's chain over every warp's
        // readiness — would reach the 128 pending flops of each warp through the rda decode):
        // the issue records {rda, the TEX last micro-op's rd.wm} in registers and pending takes them next cycle. Cycle-exact:
        // hc sees the set in flight (a u load the cycle after an issue = an empty-queue fill), drained counts it, and no clear
        // of the same register can land in that cycle (a WAW issue needs the earlier write's clear this cycle or before, and
        // only this warp's issue writes)
        reg set_v, set_tv; reg [6:0] set_a; reg [4:0] set_ta; reg [3:0] set_tm;
        // the set's address one-hot in two halves (set_a's 7 -> 128 decode into the worst-placed warps'
        // pending flops is 3 levels and 8 ns of route): set_hoh = rda[6:2] (shared by the scalar set and the
        // TEX set, whose set_ta is rda[6:2] too), set_loh = rda[1:0]
        reg [31:0] set_hoh; reg [3:0] set_loh;
        // the launch's clear of the 128 pending bits one cycle after the launch (launch -> 1 024 flops across
        // 8 warps is 7 ns of route); the first fetch lands two cycles later still, and no set or clear of
        // this slot's new occupant can exist that cycle (the previous one retired drained)
        reg lclr;
        // the LDV lambda2 hazard as a register too: set when u loads a lambda2 step whose component's lambda1
        // step has not reached its landing stage (this cycle's landing and this warp's own lambda1 issue applied), cleared
        // when it does; the per-component ldv_rdy bits are read only here, off the issue path
        reg  hl;
        wire [1:0] c_wc = u_cand[29:28]; wire [2:0] c_mk = u_cand[34:32];
        wire ldv_lw = ldv_land & (acc_land_w == WW'(gw));
        wire hl_c = ~LDV1 & (c_mk == MK_L1) & ~((ldv_rdy[gw][c_wc] | (ldv_lw & (ldv_land_c == c_wc))) & ~(iss & (mk == MK_L0) & (wc == c_wc)));
        wire hl_clr = ldv_lw & (ldv_land_c == wc);
        always @* for (int k = 0; k < 4; k++) begin
            hc[k]   = (pending[ca[k]] & ~((ecw & (ec_a == ca[k])) | (bww & (bw_a == ca[k])))) |
                      (iss & ((gpr_u & (rda == ca[k])) | (cls[C_TEX] & last & wm[ca[k][1:0]] & (rda[6:2] == ca[k][6:2])))) |
                      (set_v & (set_a == ca[k])) | (set_tv & set_tm[ca[k][1:0]] & (set_ta == ca[k][6:2]));   // last cycle's set, landing now
            hclr[k] = (ecw & (ec_a == ua[k])) | (bww & (bw_a == ua[k]));
        end
        // a TEX's WAW checks. The hazard bits whose address names a component of rd — rda always, sources 1 and 2 without
        // TEX2 (uop_dec), source 2 with it — count only where the TEX writes that component: masked here, where the bits
        // load, with the candidate's class and write mask, so the stall term takes them as they are.
        wire       c_tex = ul_nx ? n_cls[C_TEX] : cls[C_TEX];        // the micro-op u loads is a TEX's; its write mask
        wire [3:0] c_wm  = ul_nx ? n_wm : wm;
        reg  [3:0] hcm;
        always @* begin
            hcm = hc;
            if (c_tex) begin
                hcm[3] = hc[3] & c_wm[ca[3][1:0]];
                hcm[2] = hc[2] & c_wm[ca[2][1:0]];
                if (!TEX2) hcm[1] = hc[1] & c_wm[ca[1][1:0]];
            end
        end
        // TEX2 only (its one micro-op of a 2D TEX has two bits for rd, not four): hzr[c] = the head is a TEX that writes
        // rd.c and an earlier write to rd.c is in flight — the same construction as hz[3], once per component of rd.
        // Without TEX2 the registers are constant 0 and fold away.
        reg  [3:0] hzr, hcr, hrclr;
        wire [4:0] car = ca[3][6:2], uar = rda[6:2];
        always @* for (int c = 0; c < 4; c++) begin
            hcr[c]   = TEX2 & c_tex & c_wm[c] &
                       ((pending[{car, 2'(c)}] & ~((ecw & (ec_a == {car, 2'(c)})) | (bww & (bw_a == {car, 2'(c)})))) |
                        (iss & ((gpr_u & (rda == {car, 2'(c)})) | (cls[C_TEX] & last & wm[c] & (rda[6:2] == car)))) |
                        (set_v & (set_a == {car, 2'(c)})) | (set_tv & set_tm[c] & (set_ta == car)));
            hrclr[c] = (ecw & (ec_a == {uar, 2'(c)})) | (bww & (bw_a == {uar, 2'(c)}));
        end
        wire stall = (rd0 & hz[0]) | ((rd1 | cls[C_TEX]) & hz[1]) | ((rd2 | cls[C_TEX]) & hz[2]) |
                     ((gpr_u | cls[C_TEX]) & hz[3]) | (|hzr) |
                     (acc_need & ~acc1_rdy[gw]) | hl | (acc2_need & ~acc2_rdy[gw]) | tex_h |
                     ((cls[C_SETP] | cls[C_IF] | cls[C_BRK] | cls[C_CONT]) & pred_pend[gw]) |
                     ((cls[C_STO] | cls[C_STOZ]) & ol_own[gw]);      // this slot's output store is still being read
        // scheduled writes reserve their write cycle; quarter-rate (IMUL, SFU, SIN / COS, converts) and divider
        // results are buffered
        // (LDV1: an LDV's write slot is taken when its second step starts, not at the issue)
        wire sched = gpr_u & (madd_u | cls[C_CMP] | cls[C_ONE]) & ~(LDVR & cls[C_LDVI]);
        wire [4:0] lat = (LDV2P & cls[C_LDVI]) ? 5'(MADD_LAT + ADDSUB_LAT) : madd_u ? 5'(MADD_LAT) : 5'd1;   // from X2 (LDV2P: the LDV's second adder behind the multiply-add)
        wire wbc = sched & wb_busy[lat + 5'd2];                      // the write lands 2 + lat cycles after S
        wire mbc = madd_u & mblk;                                    // LDV1: the lanes are kept free for waiting second steps
        wire ubusy = cls[C_QR] & ((qr_hold != 3'd0) | (cls[C_SFU] & qf_sfu) | (cls[C_SC] & qf_sc) |
                                  (cls[C_CVT] & qf_cvt) | (cls[C_IMUL] & qf_imul) |
                                  (cls[C_IDIV] & (inf_idiv | ~idiv_ready)));
        // stack violations and unsupported ops (IOP): the instruction does not issue, the warp is killed and retires
        wire sviol = (cls[C_IF] && ssp_f) || ((cls[C_ELSE] || cls[C_ENDIF]) && ssp_z) ||
                     (cls[C_LOOP] && lsp_f) || ((cls[C_ENDL] || cls[C_BRK] || cls[C_CONT]) && lsp_z) || cls[C_BAD];
        wire is_end = cls[C_END];
        // a taken branch flushes the queue one cycle after its issue (q_flush_d): the warp does not issue meanwhile
        // and the stale head is ignored
        reg  q_flush_d; reg [AW-1:0] npc_d;
        wire hv = busy & ~fin & ~err & iv & ~q_flush_d & ~fill_d;    // a valid head (loaded: not the cycle it fills)
        assign rdy_v[gw] = hv & ~is_end & ~sviol & ~stall & ~wbc & ~ubusy & ~mbc;
        wire drained = (pending == '0) & ~set_v & ~set_tv & ~pred_pend[gw];
        assign iss = grant[gw];
        // control flow: the next PC if this instruction issues (jump = not the next word)
        wire [15:0] loop_live = lsp_z ? 16'hFFFF : (lt_mask & ~lt_cmask);
        reg [AW-1:0] npc; reg jump;
        always @* begin
            npc = ipc + 1'b1; jump = 1'b1;
            if (cls[C_BRA]) npc = imm[AW-1:0];
            else if (cls[C_ENDL] && ~lsp_z && lt_again) npc = lt_pc;
            else if (cls[C_IF] && (active & pred[gw]) == 16'd0) npc = imm[AW-1:0];
            else if (cls[C_ELSE] && (st_ent[ssp-1] & ~st_then[ssp-1] & loop_live) == 16'd0) npc = imm[AW-1:0];
            else jump = 1'b0;
        end
        wire q_pop   = iss & last;
        wire q_fet   = fgo_r & (fsel_r == WW'(gw));                 // the registered grant: this cycle's RAM read is ours
        wire q_wr    = fq_v & (fq_w == WW'(gw)) & (fq_ep == q_ep) & ~q_flush_d;
        wire fill_q  = q_wr & (q_cnt == '0);                        // a word arrives into an empty queue
        wire pop_dry = iss & last & (q_cnt == QCW'(1));             // the head leaves with at most the arriving word behind it
        wire u_load  = (iss & ~pop_dry) | fill_d;                   // u / hz / hl load this cycle
        // the request counts the read being made this cycle (the grant is a register, see fgo_r)
        wire [QCW-1:0] q_alloc_n = q_alloc + QCW'(q_fet);
        // pf: the slot is idle and its next warp's program is being fetched ahead of the launch (pre_start .. launch):
        // the queue fills and the head micro-op loads exactly as after a launch — the slot's scoreboard is clear (the
        // previous warp retired drained), so the hazard bits it loads are the launch's zeros — and nothing issues (hv
        // needs busy). The launch then only starts the execution state.
        reg  pf;
        wire pre = pre_start & (ap == WW'(gw)) & ~busy & ~pf;
        assign qreq_v[gw] = (pf | (busy & ~fin & ~err)) & (q_alloc_n < QCW'(QD));
        assign qlow_v[gw] = q_alloc_n < QCW'(2);
        assign qfpc_v[gw] = q_fpc; assign qep_v[gw] = q_ep;
        assign ins_v[gw] = ins; assign cls_v[gw] = cls; assign rc_v[gw] = rc; assign wc_v[gw] = wc; assign mk_v[gw] = mk;
        assign am_v[gw] = active; assign sm_v[gw] = active & ~killed; assign kill_v[gw] = killed;
        assign busy_v[gw] = busy; assign fin_v[gw] = fin; assign err_v[gw] = err; assign first_v[gw] = first; assign last_v[gw] = last;
        assign wbm_v[gw] = sched ? (WBW'(1) << (lat + 5'd1)) : WBW'(0);   // the wb_busy bit this warp's issue would set
        assign tex1_v[gw] = tex_first;
        assign pa_v[gw] = busy & ~fin & ~err; assign ph_v[gw] = iv & ~q_flush_d & ~fill_d;
        assign two_v[gw] = (STO2 & cls[C_STO]) | (TEX2 & cls[C_TEX]); assign xc_v[gw] = {s1a[1:0], s2a[1:0]};
        always @(posedge clk) begin
            if (rst_q) begin busy<=1'b0; fin<=1'b0; err<=1'b0; q_cnt<='0; q_alloc<='0; q_ep<=2'd0; q_flush_d<=1'b0;
                           q_rp<='0; q_rp1<=QPW'(1); q_rp2<=QPW'(2); q_wp<='0; lclr<=1'b0; fill_d<=1'b0; pf<=1'b0; end
            else if (launch & (ap == WW'(gw)) & ~pf) begin           // a launch without a fetch ahead: the queue restarts here
                busy<=1'b1; fin<=1'b0; err<=1'b0; q_cnt<='0; q_alloc<='0; q_fpc<='0; q_ep<=q_ep+2'd1; q_flush_d<=1'b0;
                q_rp<='0; q_rp1<=QPW'(1); q_rp2<=QPW'(2); q_wp<='0; fill_d<=1'b0;
                lclr<=1'b1; set_v<=1'b0; set_tv<=1'b0; hz<=4'd0; hzr<=4'd0; hl<=1'b0; active<=in_active; killed<='0; ssp<=4'd0; lsp<=3'd0;
                lsp_z<=1'b1; lsp_f<=1'b0; ssp_z<=1'b1; ssp_f<=1'b0;
            end else begin
                lclr <= 1'b0;
                if (retire & (rp == WW'(gw))) begin busy<=1'b0; fin<=1'b0; end
                // queue: pop the head (read pointer) / take the fetched word (write pointer) / the deferred redirect
                q_flush_d <= iss & jump; npc_d <= npc;
                if (q_flush_d) begin q_cnt<='0; q_alloc<='0; q_fpc<=npc_d; q_ep<=q_ep+2'd1; q_rp<='0; q_rp1<=QPW'(1); q_rp2<=QPW'(2); q_wp<='0; end
                else begin
                    q_cnt   <= q_cnt + QCW'(q_wr) - QCW'(q_pop);
                    q_alloc <= q_alloc + QCW'(q_fet) - QCW'(q_pop);
                    if (q_fet) q_fpc <= q_fpc + 1'b1;
                    if (q_pop) begin
                        q_rp <= q_rp1; q_rp1 <= q_rp2; q_rp2 <= q_rp2 + QPW'(1);
                        // the head and next registers move up: the next head = nx_ins (entry q_rp1, or the word arriving
                        // into a one-entry queue), the next next = entry q_rp2 when it exists, else the word arriving
                        h_ins <= nx_ins;
                        n_ins <= (q_cnt > QCW'(2)) ? q_ins[q_rp2] : fq_ins;
                        n_cls <= (q_cnt > QCW'(2)) ? q_cls[q_rp2] : fq_cls;
                        na_r  <= na_src[27:0];
                    end else if (q_wr) begin
                        if (q_cnt == '0) h_ins <= fq_ins;
                        if (q_cnt == QCW'(1)) begin n_ins <= fq_ins; n_cls <= fq_cls; na_r <= fq_dec[27:0]; end
                    end
                    if (q_wr) begin q_ins[q_wp]<=fq_ins; q_pc[q_wp]<=fq_pc; q_cls[q_wp]<=fq_cls; q_wp<=q_wp + QPW'(1); end
                end
                // the head micro-op's fields for the next cycle (and the head's classes / write mask when the head moves);
                // a word arriving into a dry queue (fill_q, or pop_dry with the word arriving) sets the classes and fa_r
                // now and loads u next cycle (fill_d)
                if (u_load) begin
                    u <= u_ld; uidx <= ul_in; sa_r <= sa_n[27:0];
                    if (iss & last) begin cls <= n_cls; wm <= n_wm; end
                end
                if (pop_dry | fill_q) begin cls <= fq_cls; wm <= fq_wm; fa_r <= fq_dec[27:0]; end
                fill_d <= ((pop_dry & q_wr) | fill_q) & ~(iss & jump);
                if (u_load) begin hz <= hcm; hzr <= hcr; hl <= hl_c; end
                else begin hz <= hz & ~hclr; hzr <= hzr & ~hrclr; hl <= hl & ~hl_clr; end
                if (hv & sviol) begin err<=1'b1; killed<=16'hFFFF; end
                if (busy & ~fin & drained & ((hv & is_end) | err)) fin<=1'b1;
                begin : PEND
                    reg [127:0] pn;
                    pn = pending;
                    if (ec_v && ec_w == WW'(gw)) pn[ec_a] = 1'b0;
                    if (bw_v && bw_w == WW'(gw)) pn[bw_a] = 1'b0;
                    // last cycle's issue (see set_v): the scalar set at {set_hoh, set_loh} = set_a, the TEX set at
                    // {set_hoh, set_tm} = {set_ta, the written components}
                    for (int h = 0; h < 32; h++) for (int c = 0; c < 4; c++)
                        if (set_hoh[h] & ((set_v & set_loh[c]) | (set_tv & set_tm[c]))) pn[h*4 + c] = 1'b1;
                    pending <= lclr ? '0 : pn;
                    set_v <= iss & gpr_u; set_a <= rda; set_tv <= iss & cls[C_TEX] & last; set_tm <= wm; set_ta <= rda[6:2];
                    set_hoh <= 32'd1 << rda[6:2]; set_loh <= 4'd1 << rda[1:0];
                end
                // control flow, masks, stacks at issue
                if (iss) begin
                    if (cls[C_DISC]) killed <= killed | active;
                    if (cls[C_LOOP]) begin                      // push: the old top back into the arrays, the loop on top
                        if (~lsp_z) begin
                            lcnt[2'(lsp-3'd1)]<=lt_cnt; lpc[2'(lsp-3'd1)]<=lt_pc; lent[2'(lsp-3'd1)]<=lt_ent;
                            lmask[2'(lsp-3'd1)]<=lt_mask; cmask[2'(lsp-3'd1)]<=lt_cmask;
                        end
                        lt_cnt<=imm; lt_pc<=ipc+1'b1; lt_ent<=active; lt_mask<=active; lt_cmask<=16'd0;
                        lsp<=lsp+1'b1; lsp_z<=1'b0; lsp_f<=(lsp == 3'd3);
                    end
                    if (cls[C_BRK])  begin lt_mask <= lt_mask & ~(active & pred[gw]); active <= active & ~pred[gw]; end
                    if (cls[C_CONT]) begin lt_cmask <= lt_cmask | (active & pred[gw]);  active <= active & ~pred[gw]; end
                    if (cls[C_ENDL]) begin
                        if (lt_again) begin
                            lt_cnt <= lt_cnt - 1'b1; active <= lt_mask; lt_cmask <= 16'd0;
                        end else begin                          // pop: the level below becomes the top
                            lsp <= lsp - 1'b1; active <= lt_ent; lsp_z <= (lsp == 3'd1); lsp_f <= 1'b0;
                            lt_cnt <= lcnt[2'(lsp-3'd2)]; lt_pc <= lpc[2'(lsp-3'd2)]; lt_ent <= lent[2'(lsp-3'd2)];
                            lt_mask <= lmask[2'(lsp-3'd2)]; lt_cmask <= cmask[2'(lsp-3'd2)];
                        end
                    end
                    if (cls[C_IF]) begin st_ent[ssp[2:0]] <= active; st_then[ssp[2:0]] <= active & pred[gw]; ssp <= ssp + 1'b1;
                                         ssp_z <= 1'b0; ssp_f <= (ssp == 4'd7); active <= active & pred[gw]; end
                    if (cls[C_ELSE])  active <= st_ent[ssp-1] & ~st_then[ssp-1] & loop_live;
                    if (cls[C_ENDIF]) begin active <= st_ent[ssp-1] & loop_live; ssp <= ssp - 1'b1; ssp_z <= (ssp == 4'd1); ssp_f <= 1'b0; end
                end
                // the fetch ahead of the launch (after everything above: these assignments win). pre: the idle slot's queue
                // restarts at word 0 under a new epoch (a stale fetch of the previous warp is dropped by the epoch, as at a
                // launch), the fetch requests begin, and the scoreboard is cleared (lclr: it is clear already when the
                // previous warp retired drained, but not after a reset mid-frame; the first word lands >= 3 cycles later,
                // so the head micro-op's hazard bits are loaded from a clear scoreboard). The launch of a slot that fetched
                // ahead keeps the queue, the head micro-op and its hazard bits and starts only the execution state; a word
                // arriving in that very cycle is taken by the queue logic above.
                if (launch & (ap == WW'(gw))) begin
                    busy<=1'b1; fin<=1'b0; err<=1'b0; active<=in_active; killed<='0; ssp<=4'd0; lsp<=3'd0;
                    lsp_z<=1'b1; lsp_f<=1'b0; ssp_z<=1'b1; ssp_f<=1'b0; pf<=1'b0;
                end else if (pre) begin
                    pf<=1'b1; q_cnt<='0; q_alloc<='0; q_fpc<='0; q_ep<=q_ep+2'd1; q_flush_d<=1'b0; lclr<=1'b1;
                    q_rp<='0; q_rp1<=QPW'(1); q_rp2<=QPW'(2); q_wp<='0; fill_d<=1'b0; hz<=4'd0; hzr<=4'd0; hl<=1'b0;
                end
            end
        end
        // synthesis translate_off
        // the fetch ahead relies on the idle slot's scoreboard being clear and on one pre_start per warp
        always @(posedge clk) if (!rst_q) begin
            // one writer in flight per register component: a pending bit is set only when it is clear or being cleared
            // (the WAW checks hz[3] / hzr; two writers on one bit let the first one's landing clear the second's mark)
            if (busy && !lclr && (set_v || set_tv))
                for (int h = 0; h < 32; h++) for (int c = 0; c < 4; c++)
                    if (set_hoh[h] && ((set_v && set_loh[c]) || (set_tv && set_tm[c])) && pending[h*4 + c] &&
                        !(ec_v && ec_w == WW'(gw) && ec_a == 7'(h*4 + c)) && !(bw_v && bw_w == WW'(gw) && bw_a == 7'(h*4 + c)))
                        $fatal(1, "shader_core_fs2: warp %0d sets pending r%0d.%0d while an earlier write to it is in flight", gw, h, c);
            if (launch && ap == WW'(gw) && pf && fill_d === 1'b0 && iv && (hz != 4'd0 || hzr != 4'd0 || hl))
                $fatal(1, "shader_core_fs2: warp %0d fetched ahead and launches with a hazard bit set (hz %b hl %b)", gw, hz, hl);
            if (launch && ap == WW'(gw) && pf && (pending != '0 || set_v || set_tv))
                $fatal(1, "shader_core_fs2: warp %0d fetched ahead and launches with its scoreboard not clear", gw);
            if (pre_start && ap == WW'(gw) && (busy || pf))
                $fatal(1, "shader_core_fs2: pre_start for warp %0d which is %s", gw, busy ? "busy" : "already fetching ahead");
            if (pf && busy) $fatal(1, "shader_core_fs2: warp %0d runs with its fetch-ahead flag set", gw);
        end
        // synthesis translate_on
    end endgenerate

    // ---- prefetch grant: warps with fewer than 2 words present or in flight first, round-robin within each class
    always @* begin
        fgo = 1'b0; fsel = f_rr;
        for (int k = NWARP-1; k >= 0; k--)
            if (qreq_v[(f_rr + k) % NWARP]) begin fgo = 1'b1; fsel = WW'((f_rr + k) % NWARP); end
        for (int k = NWARP-1; k >= 0; k--)
            if (qreq_v[(f_rr + k) % NWARP] && qlow_v[(f_rr + k) % NWARP]) begin fgo = 1'b1; fsel = WW'((f_rr + k) % NWARP); end
    end
    // the grant is a REGISTER (the queue counters of 8 warps -> the two priority loops -> the 8 : 1 address select ->
    // the instruction RAM's address pins is too long for one cycle, most of it route): the RAM is read the cycle after the decision, from the warp's then current q_fpc (the warp counts the read
    // into q_alloc / q_fpc that cycle, and its request excludes it: q_alloc_n). One more cycle of fetch latency, seen
    // after a taken branch (every loop iteration) and at a launch; hidden by the queue otherwise.
    always @(posedge clk) begin if (fgo_r) fr_ins <= iram[{bank[fsel_r], qfpc_v[fsel_r]}]; fq_ins <= fr_ins; end
    always @(posedge clk) begin
        if (rst_q) begin fgo_r <= 1'b0; fr_v <= 1'b0; fq_v <= 1'b0; f_rr <= '0; end
        else begin
            fgo_r <= fgo; fsel_r <= fsel;
            fr_v <= fgo_r; fr_w <= fsel_r; fr_ep <= qep_v[fsel_r]; fr_pc <= qfpc_v[fsel_r];
            fq_v <= fr_v; fq_w <= fr_w; fq_ep <= fr_ep; fq_pc <= fr_pc;
            if (fgo) f_rr <= nxt(fsel);
        end
    end

    // ================================================================ S -> X, the shared counters, predicate / acc flags
    // The issuing warp's fields = AND-OR of the one-hot grant and every warp's bundle: two LUT levels at 8 warps with no
    // select encode in front (`ins_v[sel]` behind a priority-encoded sel is 14 levels). Bundle layout,
    // top down: ins 64, am 16, sm 16, wbm WBW, mk 3, rc 2, wc 2, first, last, qr, sfu, sc, cvt, imul, idiv, tex1.
    localparam int BW = 112 + WBW;
    wire [BW-1:0] bnd_v [0:NWARP-1];
    generate for (gg = 0; gg < NWARP; gg = gg + 1) begin : BND
        assign bnd_v[gg] = {ins_v[gg], am_v[gg], sm_v[gg], wbm_v[gg], mk_v[gg], rc_v[gg], wc_v[gg], first_v[gg], last_v[gg],
                            cls_v[gg][C_QR], cls_v[gg][C_SFU], cls_v[gg][C_SC], cls_v[gg][C_CVT],
                            cls_v[gg][C_IMUL], cls_v[gg][C_IDIV], tex1_v[gg]};
    end endgenerate
    wire [BW-1:0] s_bnd;
    genvar gb;
    generate for (gb = 0; gb < BW; gb = gb + 1) begin : OH
        wire [NWARP-1:0] col;
        for (genvar k2 = 0; k2 < NWARP; k2 = k2 + 1) begin : C assign col[k2] = bnd_v[k2][gb]; end
        assign s_bnd[gb] = |(grant & col);
    end endgenerate
    wire [63:0] s_ins = s_bnd[BW-1 -: 64];
    wire [15:0] s_am = s_bnd[WBW+32 +: 16], s_sm = s_bnd[WBW+16 +: 16];
    wire [WBW-1:0] s_wbm = s_bnd[16 +: WBW];
    wire [2:0]  s_mk = s_bnd[15:13];
    wire [1:0]  s_rc = s_bnd[12:11], s_wc = s_bnd[10:9];
    wire        s_first = s_bnd[8], s_last = s_bnd[7];
    wire        s_qr = s_bnd[6], s_sfu = s_bnd[5], s_sc = s_bnd[4], s_cvt = s_bnd[3], s_imul = s_bnd[2],
                s_idiv = s_bnd[1], s_tex1 = s_bnd[0];               // s_tex1: a TEX's first micro-op issues
    // STO2 / TEX2: the issuing warp's "two components" class bit (one more one-hot column, the same depth as the
    // bundle's) and the component fields of its second read (four more columns)
    wire        s_two = |(grant & two_v);
    wire [3:0]  s_xc;
    generate for (gb = 0; gb < 4; gb = gb + 1) begin : XCO
        wire [NWARP-1:0] col;
        for (genvar k3 = 0; k3 < NWARP; k3 = k3 + 1) begin : C assign col[k3] = xc_v[k3][gb]; end
        assign s_xc[gb] = |(grant & col);
    end endgenerate
    wire bw_sfu, bw_sc, bw_cvt, bw_imul, bw_idiv;                  // a buffered class writes this cycle
    reg  cdl_v, cdl_setp; reg [WW-1:0] cdl_w; reg [6:0] cdl_a; reg [15:0] cdl_l;
    wire [LW-1:0] y_cmp;
    always @(posedge clk) begin
        if (rst_q) begin x_valid <= 1'b0; wb_busy <= '0; qr_hold <= 3'd0; inf_sfu <= '0; inf_sc <= '0; inf_cvt <= '0;
                       inf_imul <= '0; qf_sfu <= 1'b0; qf_sc <= 1'b0; qf_cvt <= 1'b0; qf_imul <= 1'b0;
                       inf_idiv <= 1'b0; pred_pend <= '0; acc1_rdy <= '0; acc2_rdy <= '0;
                       for (int w = 0; w < NWARP; w++) ldv_rdy[w] <= 4'd0;
                       tex_own_v <= 1'b0; tex_live <= '0; tex_full <= 1'b0; tm_wp <= '0; tmw_v <= 1'b0; end
        else begin
            x_valid <= issue;
            tmw_v <= s_tex1; tmw_e <= tm_wp; tmw_d <= {sel, f_rd(s_ins), f_wm(s_ins), s_am};
            if (tmw_v) tmeta[tmw_e] <= tmw_d;
            if (issue) begin x_w <= sel; x_ins <= s_ins; x_rc <= s_rc; x_wc <= s_wc; x_mk <= s_mk;
                             x_first <= s_first; x_last <= s_last; x_two <= s_two; x_s1c <= s_xc[3:2]; x_c2 <= s_xc[1:0];
                             x_am <= s_am; x_sm <= s_sm; end
            // TEX: the staging is owned from the first micro-op to the last request; live count; meta in issue order
            if (seq_done) tex_own_v <= 1'b0;
            if (s_tex1) begin tex_own_v <= 1'b1; tm_own <= tm_wp; tm_wp <= nxt_t(tm_wp); end
            tex_live <= tex_live + TLW'(s_tex1) - TLW'(bw_tex & tex_last);
            tex_full <= (tex_live + TLW'(s_tex1) - TLW'(bw_tex & tex_last)) == TLW'(TEXQ);   // = next cycle's (tex_live == TEXQ)
            // the write lands 2 + lat cycles after this S cycle = 1 + lat cycles after the next one (checked there)
            // (LDV1: a second step starting next cycle writes MADD_LAT cycles after that; fire_n saw the slot free)
            wb_busy <= (wb_busy >> 1) | s_wbm | (fire_n ? (WBW'(1) << MADD_LAT) : WBW'(0));
            // quarter-rate micro-ops 4 cycles apart (the feeder is busy 4 cycles from X2); in flight per class
            qr_hold <= s_qr ? 3'd3 : (qr_hold != 3'd0) ? qr_hold - 3'd1 : 3'd0;
            inf_sfu <= inf_sfu + QIW'(s_sfu) - QIW'(bw_sfu);
            inf_sc  <= inf_sc  + QIW'(s_sc)  - QIW'(bw_sc);
            inf_cvt <= inf_cvt + QIW'(s_cvt) - QIW'(bw_cvt);
            inf_imul <= inf_imul + QIW'(s_imul) - QIW'(bw_imul);
            qf_sfu  <= (inf_sfu  + QIW'(s_sfu)  - QIW'(bw_sfu))  == QIW'(QR_INF);
            qf_sc   <= (inf_sc   + QIW'(s_sc)   - QIW'(bw_sc))   == QIW'(QR_INF);
            qf_cvt  <= (inf_cvt  + QIW'(s_cvt)  - QIW'(bw_cvt))  == QIW'(QR_INF);
            qf_imul <= (inf_imul + QIW'(s_imul) - QIW'(bw_imul)) == QIW'(QR_INF);
            inf_idiv <= (inf_idiv | s_idiv) & ~bw_idiv;
            // predicate: pending from the SETP's issue until its compare lands. The per-warp flags are keyed on the
            // warp's own grant and fields (`issue && sel == w && field_v[sel]` would put the select mux in front of
            // every one of them)
            for (int w = 0; w < NWARP; w++) begin
                if (grant[w] && cls_v[w][C_SETP]) pred_pend[w] <= 1'b1;
                else if (cdl_v && cdl_setp && cdl_w == WW'(w)) pred_pend[w] <= 1'b0;
                if (cdl_v && cdl_setp && cdl_w == WW'(w))
                    for (int l = 0; l < 16; l++) pred[w][l] <= (y_cmp[l*W +: W] != '0);
                // accumulators: not ready from the producing step's issue until one stage before it lands (the LDV's
                // per component: its lambda2 step issues as soon as its own lambda1 step is in, not after the LAST
                // lambda1 step has landed)
                if (grant[w] && (mk_v[w] == MK_P1 || mk_v[w] == MK_A1)) acc1_rdy[w] <= 1'b0;
                else if (acc1_land && acc_land_w == WW'(w)) acc1_rdy[w] <= 1'b1;
                if (grant[w] && (mk_v[w] == MK_P2 || mk_v[w] == MK_A2)) acc2_rdy[w] <= 1'b0;
                else if (acc2_land && acc_land_w == WW'(w)) acc2_rdy[w] <= 1'b1;
                for (int c = 0; c < 4; c++) begin
                    if (grant[w] && mk_v[w] == MK_L0 && wc_v[w] == 2'(c)) ldv_rdy[w][c] <= 1'b0;
                    else if (ldv_land && acc_land_w == WW'(w) && ldv_land_c == 2'(c)) ldv_rdy[w][c] <= 1'b1;
                end
            end
        end
    end

    // ================================================================ X: addresses, pre-decode
    wire [6:0] x_op = f_op(x_ins);
    wire [GAW-1:0] ra [0:2];
    // copy 2 reads rs2 (MAD, SEL) — or rs1 for ADD / SUB, whose addend then arrives on the c operand without a mux
    wire x_addsub = (x_op == ADD) || (x_op == SUB);
    wire [1:0] x_k2 = x_addsub ? 2'd1 : 2'd2;
    genvar gk;
    // copy 1 reads rs1 — or, for a two-component micro-op, the second component of rs0 (x_s1c = the micro-op's
    // source-1 address: STO2 rs0[swz[x_rc + 1]], TEX2 rs0[swz[c2]])
    assign ra[0] = {x_w, f_rs(x_ins, 2'd0), swz_c(f_swz(x_ins, 2'd0), x_rc)};
    assign ra[1] = x_two ? {x_w, f_rs(x_ins, 2'd0), x_s1c}
                         : {x_w, f_rs(x_ins, 2'd1), swz_c(f_swz(x_ins, 2'd1), x_rc)};
    assign ra[2] = {x_w, f_rs(x_ins, x_k2), swz_c(f_swz(x_ins, x_k2), x_rc)};
    wire [PAW-1:0] x_pidx = x_ins[PAW-1:0];
    // LDV1: the oldest waiting second step starts next cycle when the micro-op now in X will not use the multiply-add
    // lanes, the accumulator read or the built-in read in X2 (no multiply-add class, no LDV of any kind; a bubble
    // counts) and the write slot MADD_LAT cycles after that is free. It takes the X -> X2 registers a multiply-add
    // step reads: the accumulator (its first result), the built-in row (lambda2 = row 3's odd half).
    wire x_lbusy = x_valid & (is_madd(x_op) | (x_op == DP3) | (x_op == DP4) | (x_op == LDV));
    assign fire_n = LDVR & ~rst_q & (rg_lq != 5'd0) & ~x_lbusy & ~wb_busy[MADD_LAT+1];
    wire [AAW-1:0] acc1_ra = fire_n ? rg_r : LDVR ? AAW'(x_w) : AAW'({x_w, (x_mk == MK_L1) ? x_wc : 2'd0});
    always @(posedge clk) begin
        // a chain step issues only after its accumulator landed
        acc1_x <= acc1[acc1_ra]; acc2_x <= acc2[x_w];
        x2_valid <= x_valid & ~rst_q;
        x2_w <= x_w; x2_wc <= x_wc; x2_mk <= x_mk; x2_am <= x_am; x2_sm <= x_sm; x2_rd <= f_rd(x_ins);
        x2_madd <= is_madd(x_op) | (x_op == DP3) | (x_op == DP4) | x_ldvi;
        // the target: 0 register, 1 acc1 entry 0, 2 acc2, 3 a lambda1 step: acc1 entry = its component; x2_ll = it is the LDV's last
        x2_tgt <= (x_mk == MK_P1 || x_mk == MK_A1) ? 2'd1 : (x_mk == MK_P2 || x_mk == MK_A2) ? 2'd2 : (x_mk == MK_L0) ? 2'd3 : 2'd0;
        x2_ll <= (x_wc == (x_ins[56] ? 2'd3 : x_ins[55] ? 2'd2 : x_ins[54] ? 2'd1 : 2'd0));
        x2_cmp <= is_cmp(x_op); x2_setp <= (x_op == SETP); x2_one <= is_one(x_op, x_ins);
        x2_sto <= (x_op == STO); x2_stoz <= (x_op == STOZ);
        x2_int <= is_int(x_op); x2_sel <= (x_op == SEL); x2_ldp <= (x_op == LDP); x2_ldv <= (x_op == LDV) & x_ins[3];
        x2_movi <= (x_op == MOVI); x2_ddx <= (x_op == DDX); x2_ddy <= (x_op == DDY);
        x2_sub <= (x_op == SUB) || (x_op == DDX) || (x_op == DDY);
        x2_madm <= (x_op == ADD || x_op == SUB || x_op == DDX || x_op == DDY) ? 2'd1 : (x_op == MUL) ? 2'd3 : 2'd0;
        x2_cmpop <= (x_op == MIN) ? 2'd0 : (x_op == MAX) ? 2'd1 : (XCLAMP && x_op == CLAMP) ? 2'd3 : 2'd2;
        x2_cc <= x_ins[15:13];                                      // SET / SETP cc [16:13]: 2 SLT 3 SGE 4 SEQ 5 SNE
        x2_intop <= (x_op==IADD) ? 3'd0 : (x_op==ISUB) ? 3'd1 : (x_op==ISLT) ? 3'd2 : (x_op==ISGE) ? 3'd3 : (x_op==ISEQ) ? 3'd4 : 3'd5;
        x2_m0 <= f_mod(x_ins, 2'd0); x2_m1 <= f_mod(x_ins, x_two ? 2'd0 : 2'd1); x2_m2 <= f_mod(x_ins, x_k2);
        x2_imm <= x_ins[4 +: W];
        // the params component: LDP the swizzled one
        x2_sc_c <= swz_c(f_swz(x_ins, 2'd0), x_rc);
        x2_pq <= params[{bank[x_w], x_pidx}];
        // the built-in read: LDV 8 -> row sc[1], half sc[0]; LDV 9 -> front / 0; LDV 10 -> row 2 (mu1, mu2), 0, 1.0;
        // LDV 11..15 -> 0; lambda1 / lambda2 (MK_L0 / MK_L1) -> row 3, half mk[0]
        begin : VSEL
            reg [1:0] sc; sc = swz_c(f_swz(x_ins, 2'd0), x_rc);
            x2_vk <= 2'd0; x2_vh <= sc[0]; x2_va <= {x_w, 1'b0, sc[1]};
            if (x_ldvi) begin x2_va <= {x_w, 2'd3}; x2_vh <= x_mk[0]; end
            else if (x_ins[2:0] == 3'd1) x2_vk <= (sc == 2'd0) ? 2'd3 : 2'd1;
            else if (x_ins[2:0] == 3'd2) begin x2_va <= {x_w, 2'd2}; if (sc[1]) x2_vk <= sc[0] ? 2'd2 : 2'd1; end
            else if (x_ins[2:0] != 3'd0) x2_vk <= 2'd1;
            // LDV1: a second step starting next cycle reads lambda2 of its warp (the micro-op in X is no LDV then)
            if (fire_n) begin x2_va <= {rmeta[rg_r][MW-1 -: WW], 2'd3}; x2_vh <= 1'b1; x2_vk <= 2'd0; end
        end
        // quarter-rate classes, IMUL, divider
        x2_sfu <= is_sfu(x_op); x2_sc <= is_sc(x_op); x2_cvt <= is_cvt(x_op);
        x2_qr <= is_sfu(x_op) | is_sc(x_op) | is_cvt(x_op) | (x_op == IMUL) | is_idiv(x_op);
        x2_imul <= (x_op == IMUL); x2_idiv <= is_idiv(x_op);
        x2_sfuop <= (x_op == RCP || x_op == RCPM) ? 2'd0 : (x_op == RSQRT || x_op == SQRT || x_op == RSQM) ? 2'd1 : (x_op == LOG2) ? 2'd2 : 2'd3;
        x2_sfum <= is_xsf(x_op); x2_sqrt <= XSFU && (x_op == SQRT);
        x2_cos <= (x_op == COS);
        x2_cvtop <= (x_op == ITOF) ? 3'd1 : (x_op == FTOU) ? 3'd2 : (x_op == UTOF) ? 3'd3 :
                    (x_op >= TRUNC && x_op <= RNDE) ? 3'(3'd4 + 3'(x_op - TRUNC)) : 3'd0;   // 4..7 = float_round modes
        x2_divmode <= (x_op == IMOD) ? 2'd1 : (x_op == UDIV) ? 2'd2 : (x_op == UMOD) ? 2'd3 : 2'd0;
        x2_tex <= (x_op == TEX); x2_tc <= x_rc; x2_tc2 <= x_two ? x_c2 : x_rc; x2_samp <= x_ins[3:0]; x2_mode <= x_ins[5:4];
        x2_tfirst <= x_first; x2_tlast <= x_last;
    end

    // ================================================================ register file: 3 copies x 16 lanes, block RAM
    reg         gpr_we; reg [GAW-1:0] gpr_wa; reg [15:0] gpr_wl; reg [LW-1:0] gpr_wd;
    genvar gl;
    generate for (gl = 0; gl < 16; gl = gl + 1) begin : GPRL
        for (gk = 0; gk < 3; gk = gk + 1) begin : CP
            (* ram_style="block" *) reg [W-1:0] m [0:NWARP*128-1];
            reg [W-1:0] q;
            always @(posedge clk) begin
                if (gpr_we & gpr_wl[gl]) m[gpr_wa] <= gpr_wd[gl*W +: W];
                q <= m[ra[gk]];
            end
        end
        assign ar[gl*W +: W] = CP[0].q;
        assign br[gl*W +: W] = CP[1].q;
        assign cr[gl*W +: W] = CP[2].q;
    end endgenerate
`ifndef SYNTHESIS
    // simulation view: scalar register a = {warp, reg, comp} of lane l (benches: dut.gpr_peek(l, a))
    function automatic [W-1:0] gpr_peek(input int l, input int a);
        case (l)
            0: gpr_peek = GPRL[0].CP[0].m[a];   1: gpr_peek = GPRL[1].CP[0].m[a];   2: gpr_peek = GPRL[2].CP[0].m[a];
            3: gpr_peek = GPRL[3].CP[0].m[a];   4: gpr_peek = GPRL[4].CP[0].m[a];   5: gpr_peek = GPRL[5].CP[0].m[a];
            6: gpr_peek = GPRL[6].CP[0].m[a];   7: gpr_peek = GPRL[7].CP[0].m[a];   8: gpr_peek = GPRL[8].CP[0].m[a];
            9: gpr_peek = GPRL[9].CP[0].m[a];   10: gpr_peek = GPRL[10].CP[0].m[a]; 11: gpr_peek = GPRL[11].CP[0].m[a];
            12: gpr_peek = GPRL[12].CP[0].m[a]; 13: gpr_peek = GPRL[13].CP[0].m[a]; 14: gpr_peek = GPRL[14].CP[0].m[a];
            default: gpr_peek = GPRL[15].CP[0].m[a];
        endcase
    endfunction
`endif

    // ================================================================ X2: operands, routing, dispatch
    assign a_m = applymod(ar, x2_m0);
    assign b_m = applymod(br, x2_m1);
    assign c_m = applymod(cr, x2_m2);
    wire [W-1:0]  pq_c = x2_pq[x2_sc_c*W +: W];                     // params broadcast
    reg  [LW-1:0] vr_c;                                             // the built-in component of every lane (also lambda)
    always @* for (int l = 0; l < 16; l++)
        case (x2_vk)
            2'd0:    vr_c[l*W +: W] = x2_vh ? vo_q[l*W +: W] : ve_q[l*W +: W];
            2'd1:    vr_c[l*W +: W] = '0;
            2'd2:    vr_c[l*W +: W] = ONE1;
            default: vr_c[l*W +: W] = ffq[x2_w][l >> 2] ? ONE1 : '0;
        endcase
    reg  [LW-1:0] t_f0, t_d1, t_d2;                                 // the triangle store's values per lane
    always @* for (int l = 0; l < 16; l++) begin
        t_f0[l*W +: W] = trd[l >> 2][0 +: W]; t_d1[l*W +: W] = trd[l >> 2][W +: W]; t_d2[l*W +: W] = trd[l >> 2][2*W +: W];
    end
    // madd operands: ADD a*1+b, SUB a*1-b (b read through copy 2: it is the c operand), MUL a*b+0, MAD a*b+c, DDX /
    // DDY right/bottom*1 - left/top (in the quad); chain steps: products with ZERO / an accumulator as the addend, the
    // DP final acc1*1 + acc2, LDV lambda*D + f0 / acc1
    // LDV1: the LDV micro-op in X2 is lambda1 * D1 + f0 into a ring entry of acc1; its second step, lambda2 * D2 + that
    // result, enters the lanes later by itself (au_r) as an MK_L1 step whose operands were set up by fire_n: lambda2 on
    // the built-in read (vr_c), D2 from the ring (d2_r, per quad), the first result in acc1_x. The micro-op in X2 that
    // cycle is not a multiply-add one (fire_n looked at it in X), so the lanes are free.
    wire [LW-1:0] y_madd;
    wire [LW-1:0] d2_q;
    generate for (gl = 0; gl < 16; gl = gl + 1) begin : D2L assign d2_q[gl*W +: W] = d2_r[(gl >> 2)*W +: W]; end endgenerate
    wire [2:0] m_mk = au_r ? MK_L1 : x2_mk;
    reg [LW-1:0] ma, mb, mc;
    always @* begin
        ma = a_m; mb = b_m; mc = c_m;
        case (m_mk)
            MK_P1, MK_P2: mc = ZEROL;
            MK_A1: mc = acc1_x;
            MK_A2: mc = acc2_x;
            MK_F:  begin ma = acc1_x; mb = ONEL; mc = acc2_x; end
            MK_L0: begin ma = vr_c; mb = t_d1; mc = t_f0; end
            MK_L1: begin ma = vr_c; mb = LDVR ? d2_q : t_d2; mc = acc1_x; end
            default: begin
                case (x2_madm)
                    2'd1: begin mb = ONEL; end
                    2'd3: begin mc = ZEROL; end
                    default: ;
                endcase
                if (x2_ddx | x2_ddy)
                    for (int l = 0; l < 16; l++) begin : dd
                        int q, p, hi, lo;
                        q = l >> 2; p = l & 3;
                        hi = x2_ddx ? (p | 1) : (p | 2); lo = x2_ddx ? (p & ~1) : (p & ~2);
                        ma[l*W +: W] = a_m[(4*q + hi)*W +: W]; mb[l*W +: W] = ONE1; mc[l*W +: W] = a_m[(4*q + lo)*W +: W];
                    end
            end
        endcase
    end
    wire [15:0] vm, vc;
    wire [2:0] cmp_op = (x2_cmpop == 2'd2) ? x2_cc : (x2_cmpop == 2'd3) ? 3'd6 : {1'b0, x2_cmpop};   // float_cmp: 0 MIN 1 MAX 2 SLT 3 SGE 4 SEQ 5 SNE; float_cmp3: 6 CLAMP
    generate for (gl = 0; gl < 16; gl = gl + 1) begin : LANE
        float_madd #(.EXP_W(EXP_W), .MAN_W(MAN_W), .DSP_ALIGN(ADD_DSP)) u_madd (.clk, .rst(rst_q), .in_valid((x2_valid & x2_madd) | au_r), .sub(x2_sub & (m_mk == MK_N)),
            .a(ma[gl*W +: W]), .b(mb[gl*W +: W]), .c(mc[gl*W +: W]), .out_valid(vm[gl]), .result(y_madd[gl*W +: W]), .prod_o());
        if (XCLAMP) begin : C3                                      // CLAMP rd, x, lo, hi: the third operand is its upper bound
            float_cmp3 #(EXP_W, MAN_W) u_cmp (.clk, .rst(rst_q), .in_valid(x2_valid & (x2_cmp | x2_setp)), .op(cmp_op),
                .a(a_m[gl*W +: W]), .b(b_m[gl*W +: W]), .c(c_m[gl*W +: W]), .out_valid(vc[gl]), .result(y_cmp[gl*W +: W]));
        end else begin : C2
            float_cmp #(EXP_W, MAN_W) u_cmp (.clk, .rst(rst_q), .in_valid(x2_valid & (x2_cmp | x2_setp)), .op(cmp_op),
                .a(a_m[gl*W +: W]), .b(b_m[gl*W +: W]), .out_valid(vc[gl]), .result(y_cmp[gl*W +: W]));
        end
    end endgenerate
    // LDV2P: an LDV's second multiplier and adder per lane. In the micro-op's X2 cycle t the multiply-add takes
    // lambda1 * D1 + f0 (MK_L0, as always) and this multiplier lambda2 * D2 — lambda2 is the odd half of built-in row 3,
    // read beside lambda1, D2 comes out of the triangle store beside D1 and f0. The product (t + 3) waits ADDSUB_LAT
    // cycles for the multiply-add's result (t + MADD_LAT) and the adder gives product + result at t + MADD_LAT +
    // ADDSUB_LAT: the same two rounded operations, in the same operand order, as the lambda2 step through float_madd.
    wire [LW-1:0] y_ldv;
    generate if (LDV2P) begin : L2P
        for (gl = 0; gl < 16; gl = gl + 1) begin : LN
            wire [W-1:0] p2;
            float_mul #(EXP_W, MAN_W) u_mul2 (.clk, .rst(rst_q), .in_valid(1'b1), .a(vo_q[gl*W +: W]), .b(t_d2[gl*W +: W]),
                .in_ready(), .out_valid(), .result(p2));
            reg [W-1:0] p2d [0:ADDSUB_LAT-1];
            always @(posedge clk) begin
                p2d[0] <= p2;
                for (int i = 1; i < ADDSUB_LAT; i++) p2d[i] <= p2d[i-1];
            end
            float_addsub #(.EXP_W(EXP_W), .MAN_W(MAN_W), .DSP_ALIGN(ADD_DSP)) u_add2 (.clk, .rst(rst_q), .in_valid(1'b1), .op(1'b0), .a(p2d[ADDSUB_LAT-1]),
                .b(y_madd[gl*W +: W]), .in_ready(), .out_valid(), .result(y_ldv[gl*W +: W]));
        end
    end else begin : NL2P
        assign y_ldv = '0;
    end endgenerate
    reg [LW-1:0] one_y;                                             // the 1-cycle path
    always @* begin
        one_y = a_m;                                                // MOV
        if (x2_movi) one_y = {16{x2_imm}};
        else if (x2_ldp) one_y = {16{pq_c}};
        else if (x2_ldv) one_y = vr_c;
        else if (x2_sel) one_y = selv(ar, br, cr);
        else if (x2_int) one_y = int_alu(ar, br, x2_intop);
    end
    // STO / STOZ: colour and depth of the active, unkilled lanes into the output store = one LUTRAM per lane, rows
    // {warp, idx} (idx 0..3 the colour component, 4 gl_FragDepth); read for the latched warp at {ol_w, out_sel[2:0]},
    // the quad out_sel[4:3] picked from the 16 lanes. A warp's last STO can be in X2 when the warp retires (its END
    // two cycles behind), and the store is first read two cycles after that, so its row is complete; odw is set in X
    // because the latch samples it AT the retire
    // STO2: a STO micro-op stores two components — x2_wc = 0 / 2 from a_m, x2_wc + 1 from b_m (the same
    // register through read copy 1) — so the store is two LUTRAMs per lane with one write and one read port each: the
    // even half, rows {warp, 0 = x, 1 = z, 2 = depth}, read by out_sel (idx 0 / 2 / 4), and the odd half, rows {warp,
    // 0 = y, 1 = w}, read by out_sel2 (idx 1 / 3). The back end reads exactly that pair per cycle (b_render B_ROW).
    wire [2:0] o_idx = x2_stoz ? 3'd4 : {1'b0, x2_wc};
    wire [LW-1:0] orow, orow2;
    generate if (STO2) begin : OS2
        wire [1:0] oe_idx = x2_stoz ? 2'd2 : {1'b0, x2_wc[1]};
        for (gl = 0; gl < 16; gl = gl + 1) begin : OL
            (* ram_style="distributed" *) reg [W-1:0] ome [0:NWARP*4-1];
            (* ram_style="distributed" *) reg [W-1:0] omo [0:NWARP*2-1];
            always @(posedge clk) if (x2_valid && (x2_sto || x2_stoz) && x2_sm[gl]) ome[{x2_w, oe_idx}] <= a_m[gl*W +: W];
            always @(posedge clk) if (x2_valid && x2_sto && x2_sm[gl]) omo[{x2_w, x2_wc[1]}] <= b_m[gl*W +: W];
            assign orow[gl*W +: W]  = ome[{ol_w, out_sel[2:1]}];
            assign orow2[gl*W +: W] = omo[{ol_w, out_sel2[1]}];
        end
    end else begin : OS1
        for (gl = 0; gl < 16; gl = gl + 1) begin : OL
            (* ram_style="distributed" *) reg [W-1:0] omem [0:NWARP*8-1];
            always @(posedge clk) if (x2_valid && (x2_sto || x2_stoz) && x2_sm[gl]) omem[{x2_w, o_idx}] <= a_m[gl*W +: W];
            assign orow[gl*W +: W]  = omem[{ol_w, out_sel[2:0]}];
            assign orow2[gl*W +: W] = omem[{ol_w, out_sel2[2:0]}];
        end
    end endgenerate
    // synthesis translate_off
    always @(posedge clk) if (STO2 && !rst_q && ol_v && (out_sel[0] === 1'b1))
        $fatal(1, "shader_core_fs2: out_sel reads an odd row (STO2: idx 1 / 3 are out_sel2's)");
    // synthesis translate_on
    assign out_row  = orow[out_sel[4:3]*4*W +: 4*W];
    assign out_row2 = orow2[out_sel2[4:3]*4*W +: 4*W];
    assign out_depthw = ol_dw;
    assign out_kill   = ol_kill;
    assign out_err    = ol_err;
    always @(posedge clk)
        for (int w = 0; w < NWARP; w++)
            if (launch && ap == WW'(w)) odw[w] <= 1'b0;
            else if (x_valid && (x_op == STOZ) && x_w == WW'(w)) odw[w] <= 1'b1;
    // synthesis translate_off
    always @(posedge clk) if (!rst_q && x2_valid && (x2_sto || x2_stoz) && ol_v && (x2_w == ol_w))
        $fatal(1, "shader_core_fs2: STO into the output store being read (warp %0d)", x2_w);
    // synthesis translate_on
    // (IMUL and the divider run on the quarter-rate unit u_iop4, their results buffered like the SFU's: a 16-lane
    // int_unit here would cost 3.8 k LUT + 16 DSP for ops the era's shaders use at most once per pixel.)
    wire v_div;
    // delay lines X2 -> W: madd (MADD_LAT, with the target, x2_tgt), cmp (1), the 1-cycle path (1)
    (* shreg_extract = "no" *) reg [MADD_LAT-1:0] mdl_v;   // flops (its stages feed the landing / readiness logic)
    reg [WW-1:0] mdl_w [0:MADD_LAT-1]; reg [6:0] mdl_a [0:MADD_LAT-1]; reg [15:0] mdl_l [0:MADD_LAT-1];
    reg [1:0] mdl_t [0:MADD_LAT-1]; reg mdl_f [0:MADD_LAT-1];
    reg sdl_v; reg [WW-1:0] sdl_w; reg [6:0] sdl_a; reg [15:0] sdl_l; reg [LW-1:0] sdl_y;
    always @(posedge clk) begin
        if (rst_q) begin mdl_v <= '0; cdl_v <= 1'b0; sdl_v <= 1'b0; end
        else begin
            mdl_v <= {mdl_v[MADD_LAT-2:0], (x2_valid & x2_madd) | au_r};
            cdl_v <= x2_valid & (x2_cmp | x2_setp);
            sdl_v <= x2_valid & x2_one;
        end
        for (int i = MADD_LAT-1; i > 0; i--) begin mdl_w[i] <= mdl_w[i-1]; mdl_a[i] <= mdl_a[i-1]; mdl_l[i] <= mdl_l[i-1]; mdl_t[i] <= mdl_t[i-1]; mdl_f[i] <= mdl_f[i-1]; end
        mdl_w[0] <= x2_w; mdl_a[0] <= {x2_rd, x2_wc}; mdl_l[0] <= x2_am; mdl_t[0] <= x2_tgt; mdl_f[0] <= x2_ll;
        // LDV1: an LDV's second step is a register write with the meta its first step left in the ring
        if (au_r) begin mdl_w[0] <= au_m[MW-1 -: WW]; mdl_a[0] <= au_m[16 +: 7]; mdl_l[0] <= au_m[15:0]; mdl_t[0] <= 2'd0; end
        cdl_w <= x2_w; cdl_a <= {x2_rd, x2_wc}; cdl_l <= x2_am; cdl_setp <= x2_setp;
        sdl_w <= x2_w; sdl_a <= {x2_rd, x2_wc}; sdl_l <= x2_am; sdl_y <= one_y;
    end
    // LDV2P: an LDV's entry goes on from the madd delay line's tail (target 3) through the second adder's stages; its
    // write is the cycle after stage L2 - 2 (ldv_gw), the scoreboard clear at that stage
    localparam int L2 = ADDSUB_LAT;
    reg [L2-1:0] l2_v; reg [WW-1:0] l2_w [0:L2-1]; reg [6:0] l2_a [0:L2-1]; reg [15:0] l2_l [0:L2-1];
    reg ldv_gw;
    always @(posedge clk) begin
        if (rst_q) begin l2_v <= '0; ldv_gw <= 1'b0; end
        else begin
            l2_v <= {l2_v[L2-2:0], LDV2P & mdl_v[MADD_LAT-1] & (mdl_t[MADD_LAT-1] == 2'd3)};
            ldv_gw <= l2_v[L2-2];
        end
        l2_w[0] <= mdl_w[MADD_LAT-1]; l2_a[0] <= mdl_a[MADD_LAT-1]; l2_l[0] <= mdl_l[MADD_LAT-1];
        for (int i = 1; i < L2; i++) begin l2_w[i] <= l2_w[i-1]; l2_a[i] <= l2_a[i-1]; l2_l[i] <= l2_l[i-1]; end
    end
    // accumulator landings (chain steps): every lane, no write port
    wire mdl_last = mdl_v[MADD_LAT-1];
    // a madd result for a register (not an accumulator) at the write port, a register of its own: it decides whether a
    // buffered result writes, and that goes on to the scoreboard (the path mdl_t -> bw_a -> hz)
    reg madd_gw;
    always @(posedge clk) madd_gw <= ~rst_q & mdl_v[MADD_LAT-2] & (mdl_t[MADD_LAT-2] == 2'd0);
    // readiness one stage ahead of the landing: the step that sees it issues in the landing cycle and
    // reads the accumulator in X, the cycle after the LUTRAM write; the LDV's per component (mdl_a[1:0] = the component)
    assign acc1_land = mdl_v[MADD_LAT-2] & (mdl_t[MADD_LAT-2] == 2'd1);
    assign acc2_land = mdl_v[MADD_LAT-2] & (mdl_t[MADD_LAT-2] == 2'd2);
    assign ldv_land  = mdl_v[MADD_LAT-2] & (mdl_t[MADD_LAT-2] == 2'd3);
    assign acc_land_w = mdl_w[MADD_LAT-2]; assign ldv_land_c = mdl_a[MADD_LAT-2][1:0];
    // the accumulators' write ports take their address and enables from registers of their own (the same values one
    // delay-line stage earlier): the two address bits reach ≈ 500 LUTRAM address pins — from one register they cross
    // the die (≈ 2 ns of route); max_fanout lets synthesis replicate these
    (* max_fanout = 32 *) reg [WW-1:0] accw_w;
    (* max_fanout = 32 *) reg [AAW-1:0] accw_a;                     // acc1's entry: a DP's (the warp's), an LDV first result's
    (* max_fanout = 32 *) reg accw_1, accw_2;
    reg accw_3;                                                     // LDV1: the landing is an LDV's first result (ring entry rg_l)
    wire ld3 = mdl_v[MADD_LAT-2] & (mdl_t[MADD_LAT-2] == 2'd3);
    always @(posedge clk) begin
        accw_w <= mdl_w[MADD_LAT-2];
        accw_a <= !LDVR ? AAW'({mdl_w[MADD_LAT-2], (mdl_t[MADD_LAT-2] == 2'd3) ? mdl_a[MADD_LAT-2][1:0] : 2'd0})
                        : ld3 ? rg_l : AAW'(mdl_w[MADD_LAT-2]);
        accw_1 <= ~rst_q & mdl_v[MADD_LAT-2] & (LDV2P ? (mdl_t[MADD_LAT-2] == 2'd1) : mdl_t[MADD_LAT-2][0]);   // (LDV2P: target 3 goes on to the second adder)
        accw_2 <= ~rst_q & mdl_v[MADD_LAT-2] & (mdl_t[MADD_LAT-2] == 2'd2);
        accw_3 <= LDVR & ~rst_q & ld3;
    end
    always @(posedge clk) begin
        if (accw_1) acc1[accw_a] <= y_madd;
        if (accw_2) acc2[accw_w] <= y_madd;
    end
    // LDV1, the ring of waiting second steps (declarations and fire_n: with the shared state / in X)
    reg [1:0] rg_cd;
    wire blk_n = LDVR & ((rg_lq >= 5'(RN / 2)) | (((rg_lq >= 5'(LDV_TH)) | (rg_wait >= 4'(LDV_WT))) & (rg_cd == 2'd0)));
    always @(posedge clk) begin
        if (rst_q) begin rg_a <= AAW'(NWARP); rg_l <= AAW'(NWARP); rg_r <= AAW'(NWARP); rg_lq <= 5'd0; rg_wait <= 4'd0; au_r <= 1'b0; mblk <= 1'b0; rg_cd <= 2'd0; end
        else begin
            if (LDVR && x2_valid && x2_madd && x2_mk == MK_L0) rg_a <= nxt_r(rg_a);
            if (LDVR && ld3) rg_l <= nxt_r(rg_l);
            if (fire_n) rg_r <= nxt_r(rg_r);
            rg_lq <= rg_lq + 5'(accw_3) - 5'(fire_n);              // accw_3: the result is written at this edge, readable from the next cycle
            rg_wait <= (fire_n || rg_lq == 5'd0) ? 4'd0 : (rg_wait != 4'd15) ? rg_wait + 4'd1 : rg_wait;
            au_r <= fire_n;
            // one closed issue cycle frees the lanes for one second step (it starts two cycles later), so the hold is a
            // one-cycle pulse and the state is looked at again once that start shows (rg_cd); a ring half full holds
            // every cycle (in flight behind it: at most the stages S .. the landing)
            mblk <= blk_n;
            rg_cd <= blk_n ? 2'd2 : (rg_cd != 2'd0) ? rg_cd - 2'd1 : 2'd0;
        end
        if (LDVR && x2_valid && x2_madd && x2_mk == MK_L0) begin
            d2s[rg_a] <= {trd[3][2*W +: W], trd[2][2*W +: W], trd[1][2*W +: W], trd[0][2*W +: W]};
            rmeta[rg_a] <= {x2_w, x2_rd, x2_wc, x2_am};
        end
        if (fire_n) begin au_m <= rmeta[rg_r]; d2_r <= d2s[rg_r]; end
    end

    // ================================================================ quarter-rate feeder and units
    // X2 captures the 16-lane operands and starts the 4-cycle feed; cycle i sends lane 4k+i to unit k
    reg qr_run; reg [1:0] qr_i; reg [LW-1:0] qr_x, qr_x2; reg [4:0] qr_cls; reg [1:0] qr_sfuop; reg qr_cos;
    reg [2:0] qr_cvtop; reg [1:0] qr_divmode;
    reg qr_fm, qr_sq;                                               // XSFU: this feed's results are multiplied by qr_x2 / it is a SQRT
    wire qr_start = x2_valid & x2_qr;
    wire [MW-1:0] x2_meta = {x2_w, x2_rd, x2_wc, x2_am};
    always @(posedge clk) begin
        if (rst_q) begin qr_run <= 1'b0; qr_i <= 2'd0; end
        else if (qr_start) begin qr_run <= 1'b1; qr_i <= 2'd0; end
        else if (qr_run) begin qr_i <= qr_i + 2'd1; if (qr_i == 2'd3) qr_run <= 1'b0; end
        if (qr_start) begin
            qr_x <= (x2_imul | x2_idiv) ? ar : a_m;               // integers: raw registers
            // the second operand: the integers' raw register; XSFU: what the unit's result is multiplied by — the second
            // source with its modifier (RCPM, RSQM) or the unit's own argument (SQRT)
            qr_x2 <= !x2_sfum ? br : x2_sqrt ? a_m : b_m;
            qr_cls <= {x2_idiv, x2_imul, x2_cvt, x2_sc, x2_sfu}; qr_sfuop <= x2_sfuop; qr_cos <= x2_cos;
            qr_cvtop <= x2_cvtop; qr_divmode <= x2_divmode; qr_fm <= x2_sfum; qr_sq <= x2_sqrt;
        end
    end
    wire [W-1:0] ux [0:3], ux2 [0:3];                               // unit k's operands this feed cycle
    wire [3:0] u_sfu_ov, u_sc_ov, u_cvt_ov;
    wire [W-1:0] u_sfu_y [0:3], u_sc_y [0:3], u_cvt_y [0:3], u_imul_y [0:3];
    wire [4*W-1:0] int4_a, int4_b, int4_ym, y_div4; wire int4_vm;
    reg [2:0] cv_d1, cv_d2;                                         // convert: the op travels with its results
    always @(posedge clk) begin cv_d1 <= qr_cvtop; cv_d2 <= cv_d1; end
    genvar gu;
    generate for (gu = 0; gu < 4; gu = gu + 1) begin : QU
        assign ux[gu]  = qr_x [(4*gu + qr_i)*W +: W];
        assign ux2[gu] = qr_x2[(4*gu + qr_i)*W +: W];
        wire su_ov; wire [W-1:0] su_y;                              // the unit's own result
        float_sfu #(.EXP_W(EXP_W), .MAN_W(MAN_W), .K(EXP_W >= 7 ? 12 : 8), .KLOG(EXP_W >= 7 ? 11 : 8), .SPLIT6(1'b1), .SPLIT1B(1'b1),   // SPLIT1B: splits the rsqrt seed RAM -> squaring DSPs path (timing at 120 MHz)
            .RCP_MEM(RCP_MEM), .RSQRT_MEM(RSQRT_MEM), .LOG2_MEM(LOG2_MEM), .EXP2_MEM(EXP2_MEM)) u_sfu (
            .clk, .rst(rst_q), .in_valid(qr_run & qr_cls[0]), .op({1'b0, qr_sfuop}), .x(ux[gu]),
            .out_valid(su_ov), .result(su_y));
        // XSFU: the result times an operand. The operand and the two flags travel beside the unit (SL = its latency with
        // SPLIT1B and SPLIT6, checked below); a multiplier takes both; beside it the unit's result, the operand and the
        // flags wait its 3 cycles, and the stage hands on the unit's result (a plain RCP / RSQRT / LOG2 / EXP2), the
        // operand itself (SQRT of a zero: SEL's rule, FTZ) or the product — with MUL's rule that a zero product is +0
        // (float_madd adds +0 to it). Every result of the unit passes the stage, so the stream keeps its order and the
        // collector below sees one valid per feed cycle as without the stage, 3 cycles later.
        if (XSFU) begin : XM
            localparam int SL = 12, ML = 3;
            reg [W+1:0] md [0:SL-1];                                // {SQRT, multiply, operand}
            always @(posedge clk) begin
                md[0] <= {qr_sq, qr_fm, ux2[gu]};
                for (int i = 1; i < SL; i++) md[i] <= md[i-1];
            end
            wire [W-1:0] xm_m = md[SL-1][W-1:0];
            wire [W-1:0] xm_p; wire xm_pv;
            float_mul #(EXP_W, MAN_W) u_xm (.clk, .rst(rst_q), .in_valid(su_ov), .a(xm_m), .b(su_y), .in_ready(), .out_valid(xm_pv), .result(xm_p));
            reg [2*W+1:0] bd [0:ML-1];                              // {SQRT, multiply, operand, the unit's result}
            always @(posedge clk) begin
                bd[0] <= {md[SL-1][W+1:W], xm_m, su_y};
                for (int i = 1; i < ML; i++) bd[i] <= bd[i-1];
            end
            wire [W-1:0] xm_y = bd[ML-1][W-1:0], xm_md = bd[ML-1][2*W-1:W];
            wire xm_fm = bd[ML-1][2*W], xm_sq = bd[ML-1][2*W+1];
            assign u_sfu_y[gu]  = !xm_fm ? xm_y : (xm_sq && xm_md[W-2:MAN_W] == '0) ? xm_md :
                                  (xm_p[W-2:MAN_W] == '0) ? {W{1'b0}} : xm_p;
            assign u_sfu_ov[gu] = xm_pv;
            // synthesis translate_off
            // the two latencies this stage counts on, checked once the pipelines hold nothing from before the reset
            reg [SL-1:0] vd; reg [ML-1:0] pd; reg [5:0] warm;
            always @(posedge clk) begin
                if (rst_q) begin vd <= '0; pd <= '0; warm <= 6'd0; end
                else begin
                    vd <= {vd[SL-2:0], qr_run & qr_cls[0]};
                    pd <= {pd[ML-2:0], su_ov};
                    if (warm != 6'd40) warm <= warm + 6'd1;
                    if (warm == 6'd40 && (vd[SL-1] !== su_ov)) $fatal(1, "shader_core_fs2: float_sfu's latency is not %0d (XSFU's operand delay)", SL);
                    if (warm == 6'd40 && (pd[ML-1] !== xm_pv)) $fatal(1, "shader_core_fs2: float_mul's latency is not %0d (XSFU)", ML);
                end
            end
            // synthesis translate_on
        end else begin : NXM
            assign u_sfu_y[gu] = su_y; assign u_sfu_ov[gu] = su_ov;
        end
        sincos_sfu #(.EXP_W(EXP_W), .MAN_W(MAN_W), .KSIN(10), .SIN_MEM(SIN_MEM)) u_sc (
            .clk, .rst(rst_q), .in_valid(qr_run & qr_cls[1]), .op(qr_cos), .x(ux[gu]), .out_valid(u_sc_ov[gu]), .result(u_sc_y[gu]));
        // convert: FTOI / FTOU through one signed W+1-bit converter clamped after, ITOF / UTOF sign- / zero-extended,
        // TRUNC FLR CEIL RNDE through float_round (as alu_cluster)
        wire [W:0] fr; wire [W-1:0] i2f_o, rnd_o;
        float_cvt_f2i #(.EXP_W(EXP_W), .MAN_W(MAN_W), .OW(W+1)) u_f2i (.clk, .rst(rst_q), .in_valid(qr_run & qr_cls[2]),
            .fval(ux[gu]), .out_valid(u_cvt_ov[gu]), .ival(fr));
        float_cvt_i2f #(.IW(W+1), .EXP_W(EXP_W), .MAN_W(MAN_W)) u_i2f (.clk, .rst(rst_q), .in_valid(qr_run & qr_cls[2]),
            .ival({(qr_cvtop != 3'd3) & ux[gu][W-1], ux[gu]}), .out_valid(), .result(i2f_o));
        float_round #(.EXP_W(EXP_W), .MAN_W(MAN_W)) u_rnd (.clk, .rst(rst_q), .in_valid(qr_run & qr_cls[2]),
            .mode(qr_cvtop[1:0]), .x(ux[gu]), .out_valid(), .result(rnd_o));
        wire [W-1:0] fti = (~fr[W] & fr[W-1]) ? {1'b0, {(W-1){1'b1}}} : (fr[W] & ~fr[W-1]) ? {1'b1, {(W-1){1'b0}}} : fr[W-1:0];
        wire [W-1:0] ftu = fr[W] ? {W{1'b0}} : fr[W-1:0];
        assign u_cvt_y[gu] = cv_d2[2] ? rnd_o : cv_d2[0] ? i2f_o : cv_d2[1] ? ftu : fti;
        assign int4_a[gu*W +: W] = ux[gu]; assign int4_b[gu*W +: W] = ux2[gu];
        assign u_imul_y[gu] = int4_ym[gu*W +: W];
    end endgenerate
    // the integer unit: IMUL (one DSP per lane, latency 3) and the divider on the 4 quarter-rate lanes.
    // The divider takes a warp's 16 lanes as 4 rounds (DSEQ, ≈ 108
    // cycles, one in flight) and returns them as 4 cycles of 4 lanes. The unit's IOP section is tied off.
    int_unit #(.NL(4), .W(W), .ONE(ONE1), .DSEQ(4)) u_int4 (.clk, .rst(rst_q), .a(int4_a), .b(int4_b),
        .mul_valid(qr_run & qr_cls[3]), .v_mul(int4_vm), .y_mul(int4_ym),
        .div_valid(qr_run & qr_cls[4]), .div_mode(qr_divmode), .div_ready(idiv_ready), .v_div(v_div), .y_div(y_div4),
        .iop_valid(1'b0), .iop_fn(4'd0), .v_iop(), .y_iop());
    // collectors: the 4 results of cycle i land in lanes 4k+i; a collector is full after the 4th cycle and is taken,
    // with the micro-op's meta (queued at the feed start), into ONE shared result queue (LUTRAM): one class
    // per cycle, the longest-waiting class first. Quarter-rate micro-ops issue at most every 4th cycle, so a class's
    // next results begin the cycle after its collector fills (slot 0 is overwritten then: a one-cycle wait keeps the
    // old slot 0 in a shadow), and at most two collectors fill within two cycles — no collector waits more than one
    // cycle (asserted). The queue head drains through the write port on a free cycle; up to QR_INF results per class
    // exist (inf_*), so the queue (4 x QR_INF entries) never overflows, nor a collector's meta queue (QR_INF).
    wire [3:0] qc_ov = {int4_vm, u_cvt_ov[0], u_sc_ov[0], u_sfu_ov[0]};
    wire [3:0] qc_start = qr_start ? {x2_imul, x2_cvt, x2_sc, x2_sfu} : 4'd0;
    wire [3:0] qc_full, qc_wait; reg [3:0] qc_take;                 // collector full / full since the previous cycle / taken
    wire [LW-1:0] qc_col [0:3]; wire [MW-1:0] qc_m [0:3];
    localparam int MQW = $clog2(QR_INF);            // a collector's meta queue: QR_INF entries
    localparam int RQD = 4*QR_INF, RQW = $clog2(RQD);   // the result queue: every class's QR_INF results
    genvar gc;
    generate for (gc = 0; gc < 4; gc = gc + 1) begin : QC
        wire [W-1:0] uy [0:3];
        if (gc == 0) begin assign uy[0] = u_sfu_y[0]; assign uy[1] = u_sfu_y[1]; assign uy[2] = u_sfu_y[2]; assign uy[3] = u_sfu_y[3]; end
        else if (gc == 1) begin assign uy[0] = u_sc_y[0]; assign uy[1] = u_sc_y[1]; assign uy[2] = u_sc_y[2]; assign uy[3] = u_sc_y[3]; end
        else if (gc == 2) begin assign uy[0] = u_cvt_y[0]; assign uy[1] = u_cvt_y[1]; assign uy[2] = u_cvt_y[2]; assign uy[3] = u_cvt_y[3]; end
        else begin assign uy[0] = u_imul_y[0]; assign uy[1] = u_imul_y[1]; assign uy[2] = u_imul_y[2]; assign uy[3] = u_imul_y[3]; end
        reg [1:0] cnt; reg [LW-1:0] col; reg full, waitq, sh_v; reg [4*W-1:0] sh0;
        reg [MW-1:0] mq [0:QR_INF-1]; reg [MQW-1:0] mq_wp, mq_rp;   // meta of the ops fed, in order (QR_INF in flight)
        always @(posedge clk) begin
            if (rst_q) begin cnt <= 2'd0; full <= 1'b0; waitq <= 1'b0; sh_v <= 1'b0; mq_wp <= '0; mq_rp <= '0; end
            else begin
                if (qc_start[gc]) mq_wp <= mq_wp + MQW'(1);
                if (qc_ov[gc]) begin
                    cnt <= cnt + 2'd1;
                    if (cnt == 2'd3) full <= 1'b1;
                    if (cnt == 2'd0 && full && !qc_take[gc]) sh_v <= 1'b1;   // the next stream overwrites slot 0 before the take
                end
                waitq <= full & ~qc_take[gc];
                if (qc_take[gc]) begin full <= 1'b0; sh_v <= 1'b0; mq_rp <= mq_rp + MQW'(1); end
            end
            // the data registers outside the reset branch (the reset replicas would reach ≈ 350
            // col / sh0 clock enables); during a reset they take what the units still deliver, unused
            if (qc_start[gc]) mq[mq_wp] <= x2_meta;
            if (qc_ov[gc]) begin
                for (int k = 0; k < 4; k++) col[(4*k + cnt)*W +: W] <= uy[k];
                if (cnt == 2'd0 && full && !qc_take[gc]) for (int k = 0; k < 4; k++) sh0[k*W +: W] <= col[(4*k)*W +: W];
            end
        end
        reg [LW-1:0] col_o;
        always @* begin col_o = col; if (sh_v) for (int k = 0; k < 4; k++) col_o[(4*k)*W +: W] = sh0[k*W +: W]; end
        assign qc_full[gc] = full; assign qc_wait[gc] = waitq; assign qc_col[gc] = col_o; assign qc_m[gc] = mq[mq_rp];
        // synthesis translate_off
        always @(posedge clk) if (!rst_q && full && waitq && !qc_take[gc]) $fatal(1, "shader_core_fs2: collector %0d waited 2 cycles", gc);
        // synthesis translate_on
    end endgenerate
    (* ram_style="distributed" *) reg [LW+MW+2-1:0] rq [0:RQD-1];   // {class, meta, result}: QR_INF results of each of the 4 classes
    reg [RQW-1:0] rq_wp, rq_rp; reg [RQW:0] rq_cnt;
    reg [1:0] rq_sel; reg rq_push;
    always @* begin
        rq_push = |qc_full; rq_sel = 2'd0; qc_take = 4'd0;
        for (int c = 3; c >= 0; c--) if (qc_full[c]) rq_sel = 2'(c);
        for (int c = 3; c >= 0; c--) if (qc_full[c] && qc_wait[c]) rq_sel = 2'(c);
        qc_take[rq_sel] = rq_push;
    end
    wire bw_q;                                                      // W takes the queue head
    wire [LW+MW+2-1:0] rq_h = rq[rq_rp];
    wire [1:0] bw_qcls = rq_h[LW+MW +: 2];
    always @(posedge clk) begin
        if (rst_q) begin rq_wp <= '0; rq_rp <= '0; rq_cnt <= '0; end
        else begin
            if (rq_push) begin rq[rq_wp] <= {rq_sel, qc_m[rq_sel], qc_col[rq_sel]}; rq_wp <= rq_wp + RQW'(1); end
            if (bw_q) rq_rp <= rq_rp + RQW'(1);
            rq_cnt <= rq_cnt + (RQW+1)'(rq_push) - (RQW+1)'(bw_q);
        end
    end
    // synthesis translate_off
    always @(posedge clk) if (!rst_q && rq_push && rq_cnt == (RQW+1)'(RQD) && !bw_q) $fatal(1, "shader_core_fs2: result queue overflow");
    always @(posedge clk) if (!rst_q && (inf_sfu > QIW'(QR_INF) || inf_sc > QIW'(QR_INF) || inf_cvt > QIW'(QR_INF) || inf_imul > QIW'(QR_INF)))
        $fatal(1, "shader_core_fs2: more than QR_INF quarter-rate micro-ops of a class in flight");
    initial if (QR_INF != 2 && QR_INF != 4) $fatal(1, "shader_core_fs2: QR_INF is 2 or 4");
    // synthesis translate_on
    // divider result (one in flight): the unit returns a warp's 16 lanes as 4 cycles of 4 (lanes 4k + i in cycle i, the
    // feed order), assembled here; the next IDIV waits for this write (inf_idiv) and for the unit (idiv_ready)
    reg idiv_pend; reg [LW-1:0] idiv_y; reg [MW-1:0] idiv_m; reg [1:0] idv_i;
    always @(posedge clk) begin
        if (rst_q) begin idiv_pend <= 1'b0; idv_i <= 2'd0; end
        else begin
            if (x2_valid & x2_idiv) idiv_m <= x2_meta;
            if (v_div) begin idv_i <= idv_i + 2'd1; if (idv_i == 2'd3) idiv_pend <= 1'b1; end
            else if (bw_idiv) idiv_pend <= 1'b0;
        end
        if (v_div) for (int k = 0; k < 4; k++) idiv_y[(4*k + idv_i)*W +: W] <= y_div4[k*W +: W];
    end

    // ================================================================ TEX: coordinate staging, requests, results
    // Coordinate staging: micro-op k writes its component of all 16 lanes (raw register: no modifiers on TEX); the first
    // micro-op zeroes the components outside the instruction's coordinate set (the TMU reads what its mode needs; a
    // zero there costs nothing and keeps X out of the simulation). After the LAST micro-op the sequencer sends ONE
    // request per quad that has an active lane (lanes 4q..4q+3; an inactive quad's lanes hold undefined registers and
    // would only cost TMU time), lowest quad first, then releases the staging (seq_done).
    // A TEX without any active lane sends nothing: its entry is complete at once (null entry).
    reg  [LW-1:0] tstg [0:3]; reg [3:0] tstg_samp; reg [1:0] tstg_mode; reg tstg_bank; reg [15:0] tstg_mk;
    reg  seq_run; reg [3:0] seq_rem; reg [1:0] seq_q;
    always @* begin seq_q = 2'd0; for (int q = 3; q >= 0; q--) if (seq_rem[q]) seq_q = 2'(q); end
    wire [3:0] x2_qm = {|x2_am[15:12], |x2_am[11:8], |x2_am[7:4], |x2_am[3:0]};
    wire seq_acc = seq_run & tex_req_ready;
    wire seq_lastq = ((seq_rem & (seq_rem - 4'd1)) == 4'd0);
    wire tex_c3 = x2_valid & x2_tex & x2_tlast;                     // the staging is complete (the set's last component)
    wire null_done = tex_c3 & (x2_qm == 4'd0);
    assign seq_done = (seq_acc & seq_lastq) | null_done;
    always @(posedge clk) begin
        if (x2_valid & x2_tex) begin
            for (int c = 0; c < 4; c++) begin
                // TEX2: an even component comes from copy 0 (x2_tc), an odd one from copy 1 (x2_tc2) — one source each
                if (TEX2 && c[0]) begin if (2'(c) == x2_tc2) tstg[c] <= br; else if (x2_tfirst) tstg[c] <= '0; end
                else begin if (2'(c) == x2_tc) tstg[c] <= ar; else if (x2_tfirst) tstg[c] <= '0; end
            end
            tstg_samp <= x2_samp; tstg_mode <= x2_mode; tstg_bank <= bank[x2_w]; tstg_mk <= tmk[x2_w];
        end
        if (rst_q) begin seq_run <= 1'b0; seq_rem <= 4'd0; end
        else if (tex_c3) begin seq_run <= (x2_qm != 4'd0); seq_rem <= x2_qm; end
        else if (seq_acc) begin seq_rem <= seq_rem & ~(4'd1 << seq_q); if (seq_lastq) seq_run <= 1'b0; end
    end
    assign tex_req_valid = seq_run; assign tex_req_sampler = {tstg_bank, tstg_samp}; assign tex_req_mode = tstg_mode;
    assign tex_req_mask = tstg_mk[4*seq_q +: 4];
    assign tex_req_cls = TEXW2 & seq_q[0];
    genvar gp;
    generate for (gp = 0; gp < 4; gp = gp + 1) begin : TRQ          // pixel gp of quad seq_q
        assign tex_req_uv[gp*2*W +: W]   = tstg[0][(4*seq_q + gp)*W +: W];
        assign tex_req_uv[gp*2*W+W +: W] = tstg[1][(4*seq_q + gp)*W +: W];
        assign tex_req_zw[gp*2*W +: W]   = tstg[2][(4*seq_q + gp)*W +: W];
        assign tex_req_zw[gp*2*W+W +: W] = tstg[3][(4*seq_q + gp)*W +: W];
    end endgenerate
    // Results return in request order: entry rx_e (the oldest TEX with quads outstanding) expects its active quads
    // lowest first (rx_got marks the received ones); a quad's 16 texels are latched and written over 4 cycles
    // (component c of its 4 pixels -> quad-block rr_q, row {entry, c}) into the result staging = 4 LUTRAM quad-blocks
    // x TEXQ*4 rows x 4 pixels; the entry is complete (tc_done) after its last quad's last row, a null entry at once.
    // rx_e steps over complete null entries. W drains the head entry (rs_rp) one wm component per free write cycle:
    // row {rs_rp, tex_wc} of all 4 quad-blocks = 16 lanes (a null entry's drain writes no lane and clears rd.wm).
    // TEXW2: two writers — writer k latches the quads of index bit 0 = k and writes quad-blocks k and
    // k + 2 (each block's write data is one writer's row: no data mux), so a warp's quads 0 1 2 3 may return back to
    // back; every writer takes exactly 4 cycles from its result, so the quads' last rows land in arrival order and the
    // last quad's last row still completes the entry.
    localparam int NRW = TEXW2 ? 2 : 1;
    reg  [NRW-1:0] rr_run, rr_last; reg [1:0] rr_c [0:NRW-1], rr_q [0:NRW-1]; reg [TQW-1:0] rr_e [0:NRW-1]; reg [LW-1:0] rr_d [0:NRW-1];
    reg  [TQW-1:0] rx_e, rs_rp; reg [3:0] rx_got; reg [TLW-1:0] rx_pend; reg [TEXQ-1:0] tc_done; reg [3:0] tex_done;
    function automatic [3:0] qm_of(input [15:0] l); qm_of = {|l[15:12], |l[11:8], |l[7:4], |l[3:0]}; endfunction
    wire [TMW-1:0] tm_h = tmeta[rs_rp], tm_x = tmeta[rx_e];
    wire [WW-1:0] tm_w = tm_h[TMW-1 -: WW]; wire [4:0] tm_rd = tm_h[24:20]; wire [3:0] tm_wm = tm_h[19:16]; wire [15:0] tm_l = tm_h[15:0];
    wire [3:0] rx_qm = qm_of(tm_x[15:0]), rx_rem = rx_qm & ~rx_got;
    reg  [1:0] rx_next;
    always @* begin rx_next = 2'd0; for (int q = 3; q >= 0; q--) if (rx_rem[q]) rx_next = 2'(q); end
    wire rx_lastq = ((rx_rem & (rx_rem - 4'd1)) == 4'd0);
    wire rx_skip = (rx_pend != '0) & tc_done[rx_e] & (rx_qm == 4'd0);   // a complete null entry at the receive pointer
    wire [3:0] tex_rem = tm_wm & ~tex_done;                          // components of the head still to write
    reg  [1:0] tex_wc;
    always @* begin tex_wc = 2'd0; for (int c = 3; c >= 0; c--) if (tex_rem[c]) tex_wc = 2'(c); end
    assign tex_last = ((tex_rem & (tex_rem - 4'd1)) == 4'd0);
    reg  [4*W-1:0] rr_slice [0:NRW-1];                               // component rr_c of a writer's latched quad's 4 pixels
    always @* for (int k = 0; k < NRW; k++) for (int p = 0; p < 4; p++) rr_slice[k][p*W +: W] = rr_d[k][(p*4 + rr_c[k])*W +: W];
    wire [LW-1:0] tex_y;
    generate for (gq = 0; gq < 4; gq = gq + 1) begin : TR
        localparam int K = TEXW2 ? (gq % 2) : 0;                     // the block's writer
        (* ram_style="distributed" *) reg [4*W-1:0] rmem [0:TEXQ*4-1];
        always @(posedge clk) if (rr_run[K] && rr_q[K] == 2'(gq)) rmem[{rr_e[K], rr_c[K]}] <= rr_slice[K];
        assign tex_y[gq*4*W +: 4*W] = rmem[{rs_rp, tex_wc}];
    end endgenerate
    wire rr_k = TEXW2 & rx_next[0];                                  // the arriving quad's writer
    reg  [NRW-1:0] rr_fin; reg rr_endl; reg [TQW-1:0] rr_ende;       // a writer's last row; ... of an entry's last quad, that entry
    always @* begin
        rr_endl = 1'b0; rr_ende = '0;
        for (int k = 0; k < NRW; k++) begin
            rr_fin[k] = rr_run[k] & (rr_c[k] == 2'd3);
            if (rr_fin[k] & rr_last[k]) begin rr_endl = 1'b1; rr_ende = rr_e[k]; end
        end
    end
    always @(posedge clk) begin
        if (rst_q) begin rr_run <= '0; rx_e <= '0; rx_got <= 4'd0; rx_pend <= '0; rs_rp <= '0; tc_done <= '0; tex_done <= 4'd0;
                       for (int k = 0; k < NRW; k++) rr_c[k] <= 2'd0; end
        else begin
            for (int k = 0; k < NRW; k++) begin
                if (tex_res_valid && rr_k == 1'(k)) begin
                    rr_run[k] <= 1'b1; rr_c[k] <= 2'd0; rr_d[k] <= tex_res_texels; rr_q[k] <= rx_next; rr_e[k] <= rx_e; rr_last[k] <= rx_lastq;
                end else if (rr_run[k]) begin rr_c[k] <= rr_c[k] + 2'd1; if (rr_c[k] == 2'd3) rr_run[k] <= 1'b0; end
            end
            if (tex_res_valid) begin
                if (rx_lastq) begin rx_e <= nxt_t(rx_e); rx_got <= 4'd0; end else rx_got <= rx_got | (4'd1 << rx_next);
            end else if (rx_skip) rx_e <= nxt_t(rx_e);
            rx_pend <= rx_pend + TLW'(s_tex1) - TLW'((tex_res_valid & rx_lastq) | (~tex_res_valid & rx_skip));
            for (int e = 0; e < TEXQ; e++) begin
                if ((rr_endl && rr_ende == TQW'(e)) || (null_done && tm_own == TQW'(e))) tc_done[e] <= 1'b1;
                else if (bw_tex && tex_last && rs_rp == TQW'(e)) tc_done[e] <= 1'b0;
            end
            if (bw_tex) begin tex_done <= tex_last ? 4'd0 : (tex_done | (4'd1 << tex_wc)); if (tex_last) rs_rp <= nxt_t(rs_rp); end
        end
    end
    // synthesis translate_off
    always @(posedge clk) if (!rst_q && tex_res_valid && rr_run[rr_k] && !rr_fin[rr_k]) $fatal(1, "shader_core_fs2: TEX results of one writer less than 4 cycles apart");
    always @(posedge clk) if (!rst_q && (&(rr_fin & rr_last)) && NRW == 2) $fatal(1, "shader_core_fs2: two TEX entries completed in one cycle");
    always @(posedge clk) if (!rst_q && tex_res_valid && (rx_pend == '0 || rx_qm == 4'd0)) $fatal(1, "shader_core_fs2: unexpected TEX result");
    // synthesis translate_on

    // ================================================================ W: the write port
    // scheduled results first (they never collide: the write slots were reserved at S), then one buffered result
    wire cmp_gw  = vc[0] & ~cdl_setp;                               // SET (SETP goes to the predicate)
    wire sched_w = madd_gw | ldv_gw | cmp_gw | sdl_v;
    // the buffered classes decide one cycle AHEAD of their write (otherwise the result queue's head pointer -> its
    // LUTRAM -> the write bus into 48 register-file block RAMs and the 128 x 8 pending clears, and the TEX staging's
    // component select on the same bus, are too long for one cycle at 120 MHz): a decision takes NEXT cycle's
    // write slot — free when wb_busy[1] is clear (an issue now reserves slots two or more cycles out) — and the write
    // bus plus the scoreboard clear come from the w2_* registers that cycle. The queue pop, the divider's clear and the
    // TEX staging's bookkeeping stay at the decision; a buffered result lands one cycle after its decision.
    wire bw_free   = ~wb_busy[1];
    assign bw_q    = bw_free & (rq_cnt != '0);
    assign bw_sfu  = bw_q & (bw_qcls == 2'd0);
    assign bw_sc   = bw_q & (bw_qcls == 2'd1);
    assign bw_cvt  = bw_q & (bw_qcls == 2'd2);
    assign bw_imul = bw_q & (bw_qcls == 2'd3);
    assign bw_idiv = bw_free & ~bw_q & idiv_pend;
    assign bw_tex  = bw_free & ~bw_q & ~idiv_pend & tc_done[rs_rp];
    reg  bn_v; reg [GAW-1:0] bn_a; reg [15:0] bn_l; reg [LW-1:0] bn_d;   // the buffered write decided now
    reg  w2_v; reg [GAW-1:0] w2_a; reg [15:0] w2_l; reg [LW-1:0] w2_d;   // ... written this cycle
    always @* begin
        bn_v = 1'b0; bn_a = '0; bn_l = 16'd0; bn_d = rq_h[LW-1:0];
        if (bw_q)         begin bn_v = 1'b1; {bn_a, bn_l} = rq_h[LW +: MW]; bn_d = rq_h[LW-1:0]; end
        else if (bw_idiv) begin bn_v = 1'b1; {bn_a, bn_l} = idiv_m; bn_d = idiv_y; end
        else if (bw_tex)  begin bn_v = 1'b1; bn_a = {tm_w, tm_rd, tex_wc}; bn_l = tm_l; bn_d = tex_y; end
    end
    always @(posedge clk) begin
        if (rst_q) w2_v <= 1'b0; else w2_v <= bn_v;
        w2_a <= bn_a; w2_l <= bn_l; w2_d <= bn_d;
    end
    always @* begin
        gpr_we = 1'b0; gpr_wa = '0; gpr_wl = 16'd0; gpr_wd = y_madd;
        if (madd_gw)          begin gpr_we = 1'b1; gpr_wa = {mdl_w[MADD_LAT-1], mdl_a[MADD_LAT-1]}; gpr_wl = mdl_l[MADD_LAT-1]; gpr_wd = y_madd; end
        else if (ldv_gw)      begin gpr_we = 1'b1; gpr_wa = {l2_w[L2-1], l2_a[L2-1]}; gpr_wl = l2_l[L2-1]; gpr_wd = y_ldv; end   // LDV2P
        else if (cmp_gw)      begin gpr_we = 1'b1; gpr_wa = {cdl_w, cdl_a}; gpr_wl = cdl_l; gpr_wd = y_cmp; end
        else if (sdl_v)       begin gpr_we = 1'b1; gpr_wa = {sdl_w, sdl_a}; gpr_wl = sdl_l; gpr_wd = sdl_y; end
        else if (w2_v)        begin gpr_we = 1'b1; gpr_wa = w2_a; gpr_wl = w2_l; gpr_wd = w2_d; end
        bw_v = w2_v; {bw_w, bw_a} = w2_a;
        // scheduled results clear the scoreboard one cycle before the write
        ec_v = 1'b0; ec_w = '0; ec_a = '0;
        if (mdl_v[MADD_LAT-2] & (mdl_t[MADD_LAT-2] == 2'd0)) begin ec_v = 1'b1; ec_w = mdl_w[MADD_LAT-2]; ec_a = mdl_a[MADD_LAT-2]; end
        else if (l2_v[L2-2]) begin ec_v = 1'b1; ec_w = l2_w[L2-2]; ec_a = l2_a[L2-2]; end
        else if (x2_valid & (x2_cmp | x2_one)) begin ec_v = 1'b1; ec_w = x2_w; ec_a = {x2_rd, x2_wc}; end
    end
    // synthesis translate_off
    always @(posedge clk) if (!rst_q && (3'(madd_gw) + 3'(ldv_gw) + 3'(cmp_gw) + 3'(sdl_v) > 3'd1))
        $fatal(1, "shader_core_fs2: two scheduled results in one write cycle");
    always @(posedge clk) if (!rst_q && w2_v && sched_w)
        $fatal(1, "shader_core_fs2: a buffered write (decided a cycle ahead) collides with a scheduled one");
    always @(posedge clk) if (!rst_q && (3'(mdl_v[MADD_LAT-2] & (mdl_t[MADD_LAT-2] == 2'd0)) + 3'(l2_v[L2-2]) + 3'(x2_valid & (x2_cmp | x2_one)) > 3'd1))
        $fatal(1, "shader_core_fs2: two scoreboard clears in one cycle");
    always @(posedge clk) if (!rst_q && v_div && idiv_pend && !bw_idiv) $fatal(1, "shader_core_fs2: divider result overrun");
    always @(posedge clk) if (!rst_q && au_r && x2_valid && (x2_madd || x2_ldv || x2_mk != MK_N))
        $fatal(1, "shader_core_fs2: a multiply-add micro-op or an LDV in X2 beside an LDV's automatic second step");
    always @(posedge clk) if (!rst_q && LDVR && x2_valid && x2_madd && x2_mk == MK_L0 && nxt_r(rg_a) == rg_r)
        $fatal(1, "shader_core_fs2: the ring of LDV first results is full");
    // synthesis translate_on

    // ================================================================ profile events (PROF)
    // tlw[w] = the warp's TEXes issued and not yet fully written (from the registered meta write and the registered last
    // drain, so a cycle late on both sides). The per-warp state of cycle t is registered, the issuing warp of t is known
    // a cycle later (x_valid / x_w), and the counts leave it out.
    function automatic [3:0] popw(input [NWARP-1:0] v); popw = 4'd0; for (int i = 0; i < NWARP; i++) popw = popw + 4'(v[i]); endfunction
    generate if (PROF) begin : PRF
        reg [3:0] tlw [0:NWARP-1];
        reg       bwl_d; reg [WW-1:0] bwl_w;
        reg [NWARP-1:0] pa_r, ph_r, tz_r, bz_r; reg [4:0] tl_r;
        reg [NWARP-1:0] tl_nz;
        always @* for (int w = 0; w < NWARP; w++) tl_nz[w] = (tlw[w] != 4'd0);
        wire [NWARP-1:0] isw = x_valid ? (NWARP'(1) << x_w) : '0;
        always @(posedge clk) begin
            if (rst_q) begin for (int w = 0; w < NWARP; w++) tlw[w] <= 4'd0; bwl_d <= 1'b0; prof_ev <= '0; end
            else begin
                bwl_d <= bw_tex & tex_last;
                for (int w = 0; w < NWARP; w++)
                    tlw[w] <= tlw[w] + 4'(tmw_v && tmw_d[TMW-1 -: WW] == WW'(w)) - 4'(bwl_d && bwl_w == WW'(w));
                prof_ev <= {x_valid, popw(pa_r & ph_r & tz_r & ~isw), popw(pa_r & ph_r & ~tz_r & ~isw), popw(pa_r & ~ph_r), popw(bz_r), tl_r};
            end
            bwl_w <= tm_w;
            pa_r <= pa_v; ph_r <= ph_v; tz_r <= tl_nz; bz_r <= busy_v; tl_r <= 5'(tex_live);
        end
    end else begin : NPRF
        always @(posedge clk) prof_ev <= '0;
    end endgenerate

    wire _unused =&{1'b0, vm, vc, u_sfu_ov[3:1], u_sc_ov[3:1], u_cvt_ov[3:1], mdl_last, mdl_f[MADD_LAT-1]};
endmodule
`default_nettype wire
