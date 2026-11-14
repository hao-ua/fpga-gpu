// shader_core_vs2 — board A's vertex core on the scalar fragment core's pattern (the 64-bit ISA, vertex flavour).
//   A warp = 16 lanes = 16 VERTICES (independent: no quads, no derivatives); NWARP
//   warps in flight, each with its own PC / prefetch queue, active mask, predicate, stacks, scoreboard and 32 vec4
//   registers. One instruction = one vec4 operation executed as one MICRO-OP per written component (x, y, z, w order),
//   each micro-op = the same scalar op on all 16 lanes; a source's swizzle is only an address (component swz_k[c] of
//   register rs_k). The stages F (prefetch) | S (select: oldest ready warp, hazards, control flow) | X (register / uniform /
//   attribute addresses) | X2 (operands, routing, dispatch) | W (one write per cycle), the scheduled / buffered write
//   rules, the chains for DP3 / DP4 (two accumulators per warp, bit-identical to float_dot4_sum's (p0 + p1) + (p2 + p3)),
//   the quarter-rate units (SFU, SIN / COS, convert, IMUL, the divider as 4 rounds of 4 lanes; no IOP: no language of
//   the GL <= 2.0 era can express it) and the control flow (IF / ELSE / ENDIF,
//   LOOP / ENDL / BREAK / CONT, BRA, SETP; a stack violation or an IOP stops the warp with err, err_stack sticky) are
//   those of shader_core_fs2 — see its header; so are the one-hot issue grant and the registered head classes.
//   What differs:
//   - INPUTS: 16 vertex attributes per lane. LDV i (i = 0..15, swizzled per component) reads the ATTRIBUTE STORE: per lane
//     a block RAM of {warp, attribute} x vec4, written one attribute of one lane per cycle (va_*) for the warp being filled
//     (in_warp), read in X at {warp, i} with the swizzled component picked in X2 — LDV is a 1-cycle-path op, as LDP.
//     256 uniforms.
//   - LDPX rd.wm, i.swz0, rs1.swz1 (op 57; ES2 Appendix A.5: a vertex shader indexes uniform arrays with ANY
//     integer expression): rd.c = params[(i + rs1[swz1[c]]) mod 256][swz0[c]] PER LANE — a 16-way gather from 16 copies
//     of the uniform store (one block RAM per lane, both banks), addressed in X2 from the lane's register (the int's low
//     8 bits + the immediate), the component picked in X3, written in W one cycle after a 1-cycle-path op (the 2-cycle
//     path: a scheduled write with lat 2). The index register is read through copy 1 (rs1) and checked for hazards as
//     such; the FS core has no LDPX (ES2: constant-index expressions only there).
//   - OUTPUTS: STV i (i = 0..15), STO 0 (gl_Position) and STO 1 (gl_PointSize) write the OUTPUT STORE, per lane a block RAM
//     of {warp, slot[4:0], component} (slot 0..15 the varyings, 16 the position, 17 the point size), 4 micro-ops each,
//     active lanes only. The retiring warp's rows are read through out_sel = {slot, comp} -> out_row (that component of all
//     16 lanes), REGISTERED: valid the cycle after out_sel.
//   - NOT HERE: TEX, DDX / DDY, DISCARD and the kill mask, STOZ, the triangle store, LDV interpolation, the built-in rows.
//     Op 21 (the 32-bit vertex core's BRP) is not an instruction of this core — a scalar branch cannot serve 16 vertices
//     whose predicates disagree; the compilers emit IF / ELSE for both shader stages on the 64-bit ISA. It and the
//     FS-only opcodes decode as NOPs (the assembler rejects them in a vertex program).
// SHALLOW_RAM: the style of the shallow, wide attribute store — block RAM, or LUTs in the one-board build (ONE_CHIP),
// where block RAM is the scarcer resource; the behaviour is the same
`ifndef SHALLOW_RAM
`ifdef ONE_CHIP
`define SHALLOW_RAM "distributed"
`else
`define SHALLOW_RAM "block"
`endif
`endif
`default_nettype none
module shader_core_vs2 #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16,
    parameter int NWARP = 4,                    // warps in flight (2..4)
    parameter int NPARAM = 256,                 // uniforms (vec4)
    parameter int NATTR = 16,                   // vertex attributes (vec4) per lane
    parameter int IRAM_DEPTH = 4096,            // instructions (64-bit words)
    parameter int QD = 4,                       // prefetch queue depth per warp
    parameter bit SFU_SPLIT1B = 1'b1,           // 1: float_sfu's S1 outputs registered once more (the seed ROM's block RAM read
                                                //    straight into the squaring DSP cascade is too slow at 120 MHz)
    parameter bit SFU_SPLIT6 = 1'b1,            // 1: float_sfu's round add in its own stage (latency 11); both on: SFU latency
                                                //    12 in the vertex core, as in the fragment core
    parameter bit SFU_TAB2 = 1'b0,              // 1: the four SFUs' tables in two sfu_tab2 (units 0 + 1, 2 + 3: one dual-port copy per
                                                //    pair instead of one each): block RAM for nothing, bit-identical
    parameter     RCP_MEM = "tools/gen/fp24_rcp_seed.mem", RSQRT_MEM = "tools/gen/fp24_rsqrt_seed.mem",
                  LOG2_MEM = "tools/gen/fp24_log2_lut.mem", EXP2_MEM = "tools/gen/fp24_exp2_lut.mem",
                  SIN_MEM = "tools/gen/fp24_sin_lut.mem"
)(
    input  wire clk, rst,
    // program load (while idle); uniforms: TWO BANKS — a write goes to bank param_bank, a
    // warp reads the bank it was launched with (in_bank), so the next draw's uniforms load while this draw's warps run
    input  wire        iram_we,
    input  wire [$clog2(IRAM_DEPTH)-1:0] iram_waddr,
    input  wire [63:0] iram_wdata,
    input  wire        param_we,
    input  wire        param_bank,
    input  wire [$clog2(NPARAM)-1:0] param_idx,
    input  wire [4*(EXP_W+MAN_W+1)-1:0] param_data,
    // attributes of the warp being filled (in_warp): attribute va_idx of lane va_lane, one vec4 per write (component c at
    // [c*W +: W]), in any order; not in the cycle of in_start (the write would go to the launching warp)
    input  wire        va_we,
    input  wire [3:0]  va_lane,
    input  wire [$clog2(NATTR)-1:0] va_idx,
    input  wire [4*(EXP_W+MAN_W+1)-1:0] va_data,
    // launch (in order): in_ready for warp in_warp, in_start launches it with its active lanes
    output wire        in_ready,
    output wire [1:0]  in_warp,
    input  wire        in_start,
    input  wire        in_bank,                 // the uniform bank of the launching warp
    input  wire [15:0] in_active,
    output wire [NWARP-1:0] busy_warps,         // per warp slot: launched and not retired (a_vertex's draw-parity check)
    // results in launch order: out_valid for the oldest finished warp; its rows are read by out_sel = {slot[4:0],
    // comp[1:0]} (slot 0..15 = STV i, 16 = gl_Position, 17 = gl_PointSize) -> out_row = that component of the 16 lanes
    // (lane l at [l*W +: W]), registered (valid the cycle after out_sel); out_ack retires the warp
    output wire        out_valid,
    input  wire        out_ack,
    input  wire [6:0]  out_sel,
    output wire [16*(EXP_W+MAN_W+1)-1:0] out_row,
    output wire        out_err,
    output reg         err_stack,
    output wire        idle
);
    localparam int W = 1 + EXP_W + MAN_W, TW = 4*W, LW = 16*W;      // scalar, vec4, one component of 16 lanes
    localparam int AW = $clog2(IRAM_DEPTH), PAW = $clog2(NPARAM), WW = (NWARP > 1) ? $clog2(NWARP) : 1;
    localparam int NAW = $clog2(NATTR);                             // attribute index
    localparam int GAW = WW + 7;                                    // {warp, reg[4:0], comp[1:0]}
    localparam int MW = WW + 7 + 16;                                // result meta {warp, scalar address, lanes}
    localparam int ADDSUB_LAT = (MAN_W <= 10) ? 3 : 5, MADD_LAT = 3 + ADDSUB_LAT;
    localparam [W-1:0] ONE1 = (((1 << (EXP_W-1)) - 1) << MAN_W);
    localparam [LW-1:0] ONEL = {16{ONE1}}, ZEROL = '0;
    // opcodes; op 21 (BRP / STOZ), TEX, DDX, DDY and DISCARD are not instructions of this core
    localparam NOP=0,MOV=1,MOVI=2,ADD=3,SUB=4,MUL=5,MAD=6,MIN=7,MAX=8,DP3=9,DP4=10,RCP=11,RSQRT=12,LOG2=13,EXP2=14,
               SETP=15,LDP=16,LDV=17,STV=18,STO=19,BRA=20,OP21=21,LOOP=22,ENDL=23,IFOP=24,ELSEOP=25,ENDIFOP=26,
               BREAKOP=27,CONTOP=28,DISCARD=29,FTOI=30,ITOF=31,DDX=32,DDY=33,TEX=34,SET=35,IADD=36,ISUB=37,ISLT=38,
               ISGE=39,SIN=40,COS=41,TRUNC=42,FLR=43,CEIL=44,RNDE=45,SEL=46,IMUL=47,IDIV=48,ISEQ=49,ISNE=50,IOP=51,
               IMOD=52,UDIV=53,UMOD=54,FTOU=55,UTOF=56,LDPX=57,ENDOP=63;
    // madd micro-op kinds (the chain steps route the madd's operands and target)
    localparam [2:0] MK_N = 0, MK_P1 = 1, MK_P2 = 2, MK_A1 = 3, MK_A2 = 4, MK_F = 5;

    // ================================================================ instruction word fields, classes, helpers
    function automatic [6:0] f_op (input [63:0] w); f_op  = w[63:57]; endfunction
    function automatic [3:0] f_wm (input [63:0] w); f_wm  = w[56:53]; endfunction
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
    function automatic is_madd(input [6:0] op); is_madd = (op==ADD)||(op==SUB)||(op==MUL)||(op==MAD); endfunction
    function automatic is_cmp (input [6:0] op); is_cmp  = (op==MIN)||(op==MAX)||(op==SET); endfunction
    function automatic is_int (input [6:0] op); is_int  = (op==IADD)||(op==ISUB)||(op==ISLT)||(op==ISGE)||(op==ISEQ)||(op==ISNE); endfunction
    function automatic is_one (input [6:0] op);                  // the 1-cycle path (LDV: every attribute)
        is_one = (op==MOV)||(op==MOVI)||(op==LDP)||(op==LDV)||(op==SEL)||is_int(op);
    endfunction
    function automatic is_sfu (input [6:0] op); is_sfu  = (op==RCP)||(op==RSQRT)||(op==LOG2)||(op==EXP2); endfunction
    function automatic is_sc  (input [6:0] op); is_sc   = (op==SIN)||(op==COS); endfunction
    function automatic is_cvt (input [6:0] op); is_cvt  = (op==FTOI)||(op==ITOF)||(op==FTOU)||(op==UTOF)||(op==TRUNC)||(op==FLR)||(op==CEIL)||(op==RNDE); endfunction
    function automatic is_idiv(input [6:0] op); is_idiv = (op==IDIV)||(op==IMOD)||(op==UDIV)||(op==UMOD); endfunction
    function automatic is_ctl (input [6:0] op);
        is_ctl = (op==IFOP)||(op==ELSEOP)||(op==ENDIFOP)||(op==LOOP)||(op==ENDL)||(op==BREAKOP)||(op==CONTOP)||(op==BRA);
    endfunction
    function automatic is_fso (input [6:0] op);                  // fragment-only / foreign opcodes: NOPs here
        is_fso = (op==OP21)||(op==TEX)||(op==DDX)||(op==DDY)||(op==DISCARD);
    endfunction
    function automatic is_out (input [6:0] op); is_out = (op==STO)||(op==STV); endfunction
    function automatic [LW-1:0] applymod(input [LW-1:0] v, input [1:0] m);
        for (int k = 0; k < 16; k++) begin
            applymod[k*W +: W] = v[k*W +: W];
            if (m == 2'b01) applymod[k*W+W-1] = ~v[k*W+W-1];
            else if (m == 2'b10) applymod[k*W+W-1] = 1'b0;
            else if (m == 2'b11) applymod[k*W+W-1] = 1'b1;
        end
    endfunction
    // integer slot (one adder per lane, as shader_core_fs2) and SEL
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
    localparam int CLS_W = 29;
    localparam int C_MADD = 0, C_CMP = 1, C_ONE = 2, C_WR = 3, C_STO = 4, C_END = 5, C_N0 = 6, C_N1 = 7, C_N3 = 8,
                   C_SFU = 9, C_SC = 10, C_CVT = 11, C_BAD = 12, C_IMUL = 13, C_IDIV = 14, C_QR = 15,
                   C_DP = 16, C_DP4 = 17, C_SETP = 18, C_CTL = 19, C_LDPX = 20,
                   // the control ops one bit each (registered head bits like the rest: the 7-bit opcode read out of the
                   // queue's LUTRAM would sit on the S stage's ready path)
                   C_IF = 21, C_ELSE = 22, C_ENDIF = 23, C_LOOP = 24, C_ENDL = 25, C_BRK = 26, C_CONT = 27, C_BRA = 28;
    function automatic [CLS_W-1:0] decode_cls(input [63:0] w);
        reg [6:0] op; op = f_op(w);
        decode_cls = '0;
        decode_cls[C_MADD] = is_madd(op); decode_cls[C_CMP] = is_cmp(op); decode_cls[C_ONE] = is_one(op);
        decode_cls[C_STO] = is_out(op); decode_cls[C_END] = (op == ENDOP);
        decode_cls[C_SFU] = is_sfu(op); decode_cls[C_SC] = is_sc(op); decode_cls[C_CVT] = is_cvt(op);
        // IOP (bit operations, shifts, integer min / max / abs / sign, unsigned compares): not in the hardware (no
        // language of the GL <= 2.0 era can express them, glslc never emits them); an
        // IOP at the head kills its warp like a stack violation (C_BAD)
        decode_cls[C_BAD] = (op == IOP); decode_cls[C_IMUL] = (op == IMUL); decode_cls[C_IDIV] = is_idiv(op);
        // IMUL and the divider are quarter-rate classes (4 lanes x 4 cycles on the 4-lane integer unit)
        decode_cls[C_QR] = is_sfu(op)||is_sc(op)||is_cvt(op)||(op == IMUL)||is_idiv(op);
        decode_cls[C_IF] = (op == IFOP); decode_cls[C_ELSE] = (op == ELSEOP); decode_cls[C_ENDIF] = (op == ENDIFOP);
        decode_cls[C_LOOP] = (op == LOOP); decode_cls[C_ENDL] = (op == ENDL); decode_cls[C_BRK] = (op == BREAKOP);
        decode_cls[C_CONT] = (op == CONTOP); decode_cls[C_BRA] = (op == BRA);
        decode_cls[C_DP] = (op == DP3)||(op == DP4); decode_cls[C_DP4] = (op == DP4);
        decode_cls[C_SETP] = (op == SETP); decode_cls[C_CTL] = is_ctl(op);
        decode_cls[C_LDPX] = (op == LDPX);                          // the 2-cycle path: reads rs1 (the index), not rs0
        decode_cls[C_WR] = is_madd(op)||is_cmp(op)||is_one(op)||is_sfu(op)||is_sc(op)||is_cvt(op)||
                           (op==IMUL)||is_idiv(op)||(op==DP3)||(op==DP4)||(op==LDPX);
        decode_cls[C_N0] = (op==MOVI)||(op==LDP)||(op==LDV)||(op==LDPX)||(op==NOP)||(op==ENDOP)||is_ctl(op)||is_fso(op);
        decode_cls[C_N1] = (op==MOV)||is_out(op)||is_sfu(op)||is_sc(op)||is_cvt(op);
        decode_cls[C_N3] = (op==MAD)||(op==SEL);
    endfunction
    // micro-ops of an instruction (STO / STV 4)
    function automatic [3:0] n_uops(input [CLS_W-1:0] cls, input [3:0] wm);
        if (cls[C_DP4]) n_uops = 4'd4 + 4'(popc(wm));
        else if (cls[C_DP]) n_uops = 4'd3 + 4'(popc(wm));
        else if (cls[C_STO]) n_uops = 4'd4;
        else if (cls[C_WR]) n_uops = 4'(popc(wm));
        else n_uops = 4'd1;
    endfunction
    // the fields of micro-op ui of an instruction, computed one cycle ahead and registered per warp (S starts its
    // hazard lookups from registers): {first, last, dpfin, mk[2:0], rc[1:0], wc[1:0], s0a, s1a, s2a, rda (7 bits each)}
    localparam int UW = 38;
    function automatic [UW-1:0] uop_dec(input [63:0] w, input [CLS_W-1:0] c, input [2:0] ui);
        reg [3:0] wm, nu; reg [2:0] nst, mk; reg dpfin, last; reg [1:0] dp_rc, cur, wc, rc;
        wm = c[C_STO] ? 4'hF : f_wm(w);
        nu = n_uops(c, wm);
        last = (4'(ui) + 4'd1 == nu);
        nst = c[C_DP4] ? 3'd4 : 3'd3;                               // DP: accumulator steps before the finals
        dpfin = c[C_DP] & (ui >= nst);
        dp_rc = (ui == 3'd0) ? 2'd0 : (ui == 3'd1) ? 2'd2 : (ui == 3'd2) ? 2'd1 : 2'd3;
        cur = c[C_DP] ? 2'(ui - nst) : ui[1:0];
        wc = comp_of(wm, cur);
        rc = (c[C_DP] & ~dpfin) ? dp_rc : wc;
        mk = MK_N;
        if (c[C_DP]) mk = dpfin ? MK_F : (ui == 3'd0) ? MK_P1 : (ui == 3'd1) ? MK_P2 : (ui == 3'd2) ? MK_A1 : MK_A2;
        uop_dec = {(ui == 3'd0), last, dpfin, mk, rc, wc,
                   f_rs(w, 2'd0), swz_c(f_swz(w, 2'd0), rc), f_rs(w, 2'd1), swz_c(f_swz(w, 2'd1), rc),
                   f_rs(w, 2'd2), swz_c(f_swz(w, 2'd2), rc), f_rd(w), wc};
    endfunction

    // ================================================================ program, uniforms, attribute store
    (* ram_style="block" *) reg [63:0] iram [0:IRAM_DEPTH-1];
    (* ram_style="block" *) reg [TW-1:0] params [0:2*NPARAM-1];    // {bank, index}
    always @(posedge clk) if (iram_we) iram[iram_waddr] <= iram_wdata;
    always @(posedge clk) if (param_we) params[{param_bank, param_idx}] <= param_data;
    reg [NWARP-1:0] pbank;                                          // each warp's uniform bank (set at its launch)
    reg [WW-1:0] ap, rp;                                            // allocate / retire pointers
    // x_w addresses every lane's GPR copies, attribute RAM, accumulators and the params bank: ≈ 900 loads across the core —
    // replicas made late (phys_opt) route far to the GPR address pins; max_fanout lets synthesis replicate it up front
    (* max_fanout = 32 *) reg [WW-1:0] x_w;
    reg         x_valid; reg [63:0] x_ins; reg [1:0] x_rc, x_wc; reg [2:0] x_mk; reg [15:0] x_am;
    // attribute store: per lane a block RAM of {warp, attribute} x vec4, written for the warp being filled (ap), read in X
    // at {the micro-op's warp, LDV index}; the swizzled component is picked in X2
    wire [16*TW-1:0] aq;                                            // every lane's attribute vec4 (X2)
    genvar gl, gk;
    generate for (gl = 0; gl < 16; gl = gl + 1) begin : AL
        (* ram_style=`SHALLOW_RAM *) reg [TW-1:0] am [0:NWARP*NATTR-1];
        reg [TW-1:0] q;
        always @(posedge clk) begin
            if (va_we && va_lane == 4'(gl)) am[{ap, va_idx}] <= va_data;
            q <= am[{x_w, x_ins[NAW-1:0]}];
        end
        assign aq[gl*TW +: TW] = q;
    end endgenerate

    // ================================================================ stage registers and shared state (declared first)
    reg  [15:0] wb_busy;                           // bit j: a GPR write j cycles from now (from this S cycle)
    reg  x2_valid; reg [WW-1:0] x2_w; reg [1:0] x2_wc; reg [15:0] x2_am; reg [4:0] x2_rd; reg [2:0] x2_mk;
    reg  x2_madd, x2_cmp, x2_setp, x2_one, x2_sto, x2_int, x2_sel, x2_ldp, x2_ldv, x2_movi, x2_sub, x2_ldpx;
    reg  [PAW-1:0] x2_pidx;                         // LDPX: the immediate part of the slot
    reg  x2_qr, x2_sfu, x2_sc, x2_cvt, x2_imul, x2_idiv;
    reg  [1:0] x2_cmpop, x2_m0, x2_m1, x2_m2, x2_sc_c, x2_madm, x2_sfuop, x2_divmode, x2_tgt; reg [2:0] x2_intop, x2_cvtop;
    reg  x2_cos; reg [W-1:0] x2_imm; reg [2:0] x2_cc; reg [4:0] x2_slot;
    reg  [TW-1:0] x2_pq;
    wire [LW-1:0] ar, br, cr;                       // X2 operands (register copies 0, 1, 2)
    wire [LW-1:0] a_m, b_m, c_m;                    // with the source modifiers
    reg  ec_v; reg [WW-1:0] ec_w; reg [6:0] ec_a;  // scoreboard clears: a scheduled result one cycle before its write
    reg  bw_v; reg [WW-1:0] bw_w; reg [6:0] bw_a;  // a buffered result at its write
    reg  [15:0] pred [0:NWARP-1]; reg [NWARP-1:0] pred_pend;
    // chain accumulators per warp in LUTRAM (read in X for the X2 operand network, written at a chain step's landing)
    (* ram_style="distributed" *) reg [LW-1:0] acc1 [0:NWARP-1];
    (* ram_style="distributed" *) reg [LW-1:0] acc2 [0:NWARP-1];
    reg  [LW-1:0] acc1_x, acc2_x; reg [NWARP-1:0] acc1_rdy, acc2_rdy;
    // quarter-rate feeder / in-flight accounting (S reads these)
    reg  [2:0] qr_hold;                            // cycles until the next quarter-rate micro-op may issue
    reg  [1:0] inf_sfu, inf_sc, inf_cvt, inf_imul; reg inf_idiv;
    wire idiv_ready;
    // prefetch (FETCH 2 of shader_core_fs): grant fgo / fsel, the word read in the previous cycle fq_*
    localparam int QCW = $clog2(QD + 1);
    localparam int QPW = (QD > 1) ? $clog2(QD) : 1;             // queue pointers (QD a power of two)
    if ((QD & (QD - 1)) != 0) $error("shader_core_vs2: QD must be a power of 2");
    wire [NWARP-1:0] qreq_v, qlow_v;
    wire [AW-1:0] qfpc_v [0:NWARP-1];
    wire [1:0]    qep_v  [0:NWARP-1];
    reg           fgo; reg [WW-1:0] fsel, f_rr;
    // the RAM's read fr_*, its output register fq_* (the fragment core's two-stage fetch; timing at 120 MHz: the block
    // RAM's latch -> class decode -> the next micro-op -> the 128 : 1 pending lookup -> hz is far too long for one
    // cycle). A word is in the queue a cycle later; q_alloc already counts it from the grant.
    reg  [63:0]   fr_ins, fq_ins; reg fr_v, fq_v; reg [WW-1:0] fr_w, fq_w; reg [1:0] fr_ep, fq_ep; reg [AW-1:0] fr_pc, fq_pc;
    wire [CLS_W-1:0] fq_cls = decode_cls(fq_ins);
    // per-warp S outputs
    wire [NWARP-1:0] rdy_v, busy_v, fin_v, err_v, first_v;
    wire [15:0]   wbm_v [0:NWARP-1];                                // the wb_busy bit the warp's issue would set
    wire [63:0]   ins_v [0:NWARP-1];
    wire [CLS_W-1:0] cls_v [0:NWARP-1];
    wire [1:0]    rc_v [0:NWARP-1], wc_v [0:NWARP-1];
    wire [2:0]    mk_v [0:NWARP-1];
    wire [15:0]   am_v  [0:NWARP-1];
    reg  [WW-1:0] sel; reg issue;

    // ================================================================ warp bookkeeping
    wire launch = in_start & ~busy_v[ap];
    assign in_ready = ~busy_v[ap];
    assign in_warp  = 2'(ap);
    assign out_valid = busy_v[rp] & fin_v[rp];
    wire retire = out_valid & out_ack;
    assign idle = ~|busy_v;
    assign busy_warps = busy_v;
    function automatic [WW-1:0] nxt(input [WW-1:0] p); nxt = (p == WW'(NWARP-1)) ? '0 : p + 1'b1; endfunction
    always @(posedge clk) begin
        if (rst) begin ap <= '0; rp <= '0; err_stack <= 1'b0; pbank <= '0; end
        else begin
            if (launch) begin ap <= nxt(ap); pbank[ap] <= in_bank; end
            if (retire) rp <= nxt(rp);
            if (|err_v) err_stack <= 1'b1;
        end
    end
    // the oldest ready warp first: a one-hot grant (warp w wins when no older warp is ready), sel = its OR-encode, one LUT
    // per bit (a priority chain over the ready bits with binary muxes behind it is too slow in front of x_w and its
    // replicas); sel is read only under issue
    function automatic [NWARP-1:0] older_of(input int w, input int r);  // the warps ahead of w in the order rp, rp+1, ..
        older_of = '0;
        for (int v = 0; v < NWARP; v++) if ((v - r + NWARP) % NWARP < (w - r + NWARP) % NWARP) older_of[v] = 1'b1;
    endfunction
    wire [NWARP-1:0] grant;
    genvar gg;
    generate for (gg = 0; gg < NWARP; gg = gg + 1) begin : GR
        assign grant[gg] = rdy_v[gg] & ~|(rdy_v & older_of(gg, int'(rp)));
    end endgenerate
    always @* begin
        issue = |rdy_v; sel = '0;
        for (int k = 0; k < NWARP; k++) sel = sel | ({WW{grant[k]}} & WW'(k));
    end

    // ================================================================ per-warp state (S stage)
    genvar gw;
    generate for (gw = 0; gw < NWARP; gw = gw + 1) begin : WRP
        reg busy, fin, err;
        reg [15:0] active;
        reg [127:0] pending;                                        // one bit per scalar register {reg, comp}
        reg [2:0]  uidx;                                            // micro-ops of the head instruction issued
        reg [15:0] st_ent [0:7], st_then [0:7]; reg [3:0] ssp;      // IF stack
        reg [15:0] lcnt [0:3]; reg [AW-1:0] lpc [0:3]; reg [2:0] lsp; // loop stack: the levels BELOW the top (see lt_*)
        reg [15:0] lent [0:3], lmask [0:3], cmask [0:3];            // entering / still looping / continued lanes
        // the loop stack's top as REGISTERS (as the fragment core; timing at 120 MHz: `lsp` -> the lsp-1 index -> the
        // loop-live mask / the ENDL decision -> the next PC is too long otherwise): lt_* = entry lsp-1
        // whenever lsp > 0; a push writes the old top back into the arrays at lsp-1, a pop reloads the top from entry
        // lsp-2; BRK / CONT / a repeated ENDL change the top only. lsp_z / lsp_f = lsp == 0 / == 4.
        reg [15:0] lt_cnt, lt_ent, lt_mask, lt_cmask; reg [AW-1:0] lt_pc; reg lsp_z, lsp_f;
        wire       lt_again = (lt_cnt > 16'd1) & (lt_mask != 16'd0);   // the ENDL repeats
        // circular prefetch queue (head = entry q_rp, writes at q_wp; a pop moves the read pointer, not the entries),
        // the classes decoded at the queue write
        reg [63:0] q_ins [0:QD-1]; reg [AW-1:0] q_pc [0:QD-1]; reg [CLS_W-1:0] q_cls [0:QD-1];
        reg [QCW-1:0] q_cnt, q_alloc; reg [AW-1:0] q_fpc; reg [1:0] q_ep;
        reg [QPW-1:0] q_rp, q_wp;
        reg [QPW-1:0] q_rp1, q_rp2;                                 // q_rp + 1 / + 2, registers
        // the head word and the word after it are REGISTERS mirroring q_ins[q_rp] / q_ins[q_rp1] (as the fragment core;
        // timing at 120 MHz: q_rp -> the LUTRAM read -> uop_dec -> the 128 : 1 pending lookup -> hz is 14 logic levels
        // otherwise). A pop moves the next word up and refills the next from entry
        // q_rp2 (or the word being written); a write into an empty / one-entry queue lands in them directly. Cycle-exact.
        reg [63:0] h_ins, n_ins; reg [CLS_W-1:0] n_cls;
        wire [63:0] ins = h_ins;
        wire [AW-1:0] ipc = q_pc[q_rp];
        // the head's classes are a REGISTER, loaded with u (not `q_cls[q_rp]`, an asynchronous LUTRAM read in front of
        // the ready logic); the control ops are class bits too, so nothing on the ready path decodes the opcode
        reg  [CLS_W-1:0] cls;
        wire        iv  = (q_cnt != '0);
        wire [15:0] imm = ins[15:0];
        // the head micro-op's fields (registered: loaded with the next micro-op's at issue, with the next head's at
        // a pop, with the first word's when the queue was empty)
        reg  [UW-1:0] u;
        wire        first = u[37], last = u[36], dpfin = u[35];
        wire [2:0]  mk = u[34:32];
        wire [1:0]  rc = u[31:30], wc = u[29:28];
        wire [6:0]  s0a = u[27:21], s1a = u[20:14], s2a = u[13:7], rda = u[6:0];
        wire [63:0] nx_ins = (q_cnt > QCW'(1)) ? n_ins : fq_ins;
        wire [CLS_W-1:0] nx_cls = (q_cnt > QCW'(1)) ? n_cls : fq_cls;
        wire [UW-1:0] u_same = uop_dec(ins, cls, uidx + 3'd1);
        wire [UW-1:0] u_next = uop_dec(nx_ins, nx_cls, 3'd0);
        wire madd_u = cls[C_MADD] | cls[C_DP];                      // a madd micro-op
        wire gpr_u  = cls[C_WR] & ~(cls[C_DP] & ~dpfin);            // this micro-op writes rd.wc
        // hazards
        wire rd0 = ~cls[C_N0] & ~dpfin;                             // reads rs0 this micro-op
        wire rd1 = (rd0 & ~cls[C_N1]) | cls[C_LDPX];                // rs1: the second operand, or LDPX's index
        wire rd2 = cls[C_N3];
        wire acc_need = (mk == MK_A1) | (mk == MK_F);
        wire acc2_need = (mk == MK_A2) | (mk == MK_F);
        // registered operand hazards (the fragment core's scheme; timing at 120 MHz: pending[s0a] — a 128:1 lookup from
        // the micro-op register — -> stall -> the arbiter -> every warp's pending / mask / stack updates is far too long
        // for one cycle): hz[k] = pending[address k of u] for s0a, s1a, s2a,
        // rda. Recomputed (hc) from the micro-op u is about to load — u_cand = what u gets at an issue or at an empty-queue
        // fill — with this cycle's scoreboard clears and this warp's own set applied; while u holds, the clears of its
        // addresses take the bits down (sets come only from this warp's issue, which reloads u). Cycle-exact.
        wire [UW-1:0] u_cand = (last | (q_cnt == '0)) ? u_next : u_same;
        wire [6:0] ca [0:3], ua [0:3];
        assign ca[0] = u_cand[27:21]; assign ca[1] = u_cand[20:14]; assign ca[2] = u_cand[13:7]; assign ca[3] = u_cand[6:0];
        assign ua[0] = s0a; assign ua[1] = s1a; assign ua[2] = s2a; assign ua[3] = rda;
        wire ecw = ec_v & (ec_w == WW'(gw)), bww = bw_v & (bw_w == WW'(gw));
        wire iss;
        // the scoreboard set lands a cycle after the issue (the fragment core's scheme; timing at 120 MHz: the issue ->
        // rda decode into the 128 pending bits). Cycle-exact:
        // hc sees the set in flight, drained counts it, and no clear of the same register can land in that cycle (a WAW
        // issue needs the earlier write's clear this cycle or before, and only this warp's issue writes)
        reg set_v; reg [6:0] set_a;
        reg [3:0] hz, hc, hclr;
        always @* for (int k = 0; k < 4; k++) begin
            hc[k]   = (pending[ca[k]] & ~((ecw & (ec_a == ca[k])) | (bww & (bw_a == ca[k])))) | (iss & gpr_u & (rda == ca[k])) |
                      (set_v & (set_a == ca[k]));                                                   // last cycle's set, landing now
            hclr[k] = (ecw & (ec_a == ua[k])) | (bww & (bw_a == ua[k]));
        end
        wire stall = (rd0 & hz[0]) | (rd1 & hz[1]) | (rd2 & hz[2]) | (gpr_u & hz[3]) |
                     (acc_need & ~acc1_rdy[gw]) | (acc2_need & ~acc2_rdy[gw]) |
                     ((cls[C_SETP] | cls[C_IF] | cls[C_BRK] | cls[C_CONT]) & pred_pend[gw]);
        // scheduled writes reserve their write cycle; quarter-rate (IMUL, SFU, SIN / COS, converts) and divider
        // results are buffered
        wire sched = gpr_u & (madd_u | cls[C_CMP] | cls[C_ONE] | cls[C_LDPX]);
        wire [4:0] lat = madd_u ? 5'(MADD_LAT) : cls[C_LDPX] ? 5'd2 : 5'd1;   // from X2
        wire wbc = sched & wb_busy[lat + 5'd2];                      // the write lands 2 + lat cycles after S
        wire ubusy = cls[C_QR] & ((qr_hold != 3'd0) | (cls[C_SFU] & inf_sfu[1]) | (cls[C_SC] & inf_sc[1]) |
                                  (cls[C_CVT] & inf_cvt[1]) | (cls[C_IMUL] & inf_imul[1]) |
                                  (cls[C_IDIV] & (inf_idiv | ~idiv_ready)));
        // stack violations and unsupported ops (IOP): the instruction does not issue, the warp stops (err) and retires
        // when drained
        wire sviol = (cls[C_IF] && ssp==4'd8) || ((cls[C_ELSE] || cls[C_ENDIF]) && ssp==4'd0) ||
                     (cls[C_LOOP] && lsp_f) || ((cls[C_ENDL] || cls[C_BRK] || cls[C_CONT]) && lsp_z) || cls[C_BAD];
        wire is_end = cls[C_END];
        // a taken branch flushes the queue one cycle after its issue (q_flush_d): the warp does not issue meanwhile
        // and the stale head is ignored
        reg  q_flush_d; reg [AW-1:0] npc_d;
        wire hv = busy & ~fin & ~err & iv & ~q_flush_d;              // a valid head
        assign rdy_v[gw] = hv & ~is_end & ~sviol & ~stall & ~wbc & ~ubusy;
        wire drained = (pending == '0) & ~set_v & ~pred_pend[gw];
        assign iss = issue & (sel == WW'(gw));
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
        wire q_fet   = fgo & (fsel == WW'(gw));
        wire q_wr    = fq_v & (fq_w == WW'(gw)) & (fq_ep == q_ep) & ~q_flush_d;
        assign qreq_v[gw] = busy & ~fin & ~err & (q_alloc < QCW'(QD));
        assign qlow_v[gw] = q_alloc < QCW'(2);
        assign qfpc_v[gw] = q_fpc; assign qep_v[gw] = q_ep;
        assign ins_v[gw] = ins; assign cls_v[gw] = cls; assign rc_v[gw] = rc; assign wc_v[gw] = wc; assign mk_v[gw] = mk;
        assign am_v[gw] = active;
        assign busy_v[gw] = busy; assign fin_v[gw] = fin; assign err_v[gw] = err; assign first_v[gw] = first;
        assign wbm_v[gw] = sched ? (16'd1 << (lat + 5'd1)) : 16'd0;   // the wb_busy bit this warp's issue would set
        always @(posedge clk) begin
            if (rst) begin busy<=1'b0; fin<=1'b0; err<=1'b0; q_cnt<='0; q_alloc<='0; q_ep<=2'd0; uidx<=3'd0; q_flush_d<=1'b0;
                           q_rp<='0; q_rp1<=QPW'(1); q_rp2<=QPW'(2); q_wp<='0; set_v<=1'b0; end
            else if (launch & (ap == WW'(gw))) begin
                busy<=1'b1; fin<=1'b0; err<=1'b0; q_cnt<='0; q_alloc<='0; q_fpc<='0; q_ep<=q_ep+2'd1; uidx<=3'd0; q_flush_d<=1'b0;
                q_rp<='0; q_rp1<=QPW'(1); q_rp2<=QPW'(2); q_wp<='0;
                pending<='0; set_v<=1'b0; hz<=4'd0; active<=in_active; ssp<=4'd0; lsp<=3'd0; lsp_z<=1'b1; lsp_f<=1'b0;
            end else begin
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
                    end else if (q_wr) begin
                        if (q_cnt == '0) h_ins <= fq_ins;
                        if (q_cnt == QCW'(1)) begin n_ins <= fq_ins; n_cls <= fq_cls; end
                    end
                    if (q_wr) begin q_ins[q_wp]<=fq_ins; q_pc[q_wp]<=fq_pc; q_cls[q_wp]<=fq_cls; q_wp<=q_wp + QPW'(1); end
                end
                // the head micro-op's fields for the next cycle (and the head's classes when the head moves)
                if (iss) begin u <= last ? u_next : u_same; if (last) cls <= nx_cls; end
                else if (q_wr && q_cnt == '0) begin u <= u_next; cls <= nx_cls; end
                if (iss || (q_wr && q_cnt == '0)) hz <= hc; else hz <= hz & ~hclr;   // (u_cand = the value u loads)
                if (hv & sviol) err<=1'b1;
                if (busy & ~fin & drained & ((hv & is_end) | err)) fin<=1'b1;
                if (iss) uidx <= last ? 3'd0 : uidx + 3'd1;
                begin : PEND
                    reg [127:0] pn;
                    pn = pending;
                    if (ec_v && ec_w == WW'(gw)) pn[ec_a] = 1'b0;
                    if (bw_v && bw_w == WW'(gw)) pn[bw_a] = 1'b0;
                    if (set_v) pn[set_a] = 1'b1;                                                  // last cycle's issue (see set_v)
                    pending <= pn;
                    set_v <= iss & gpr_u; set_a <= rda;
                end
                // control flow, masks, stacks at issue
                if (iss) begin
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
                                         active <= active & pred[gw]; end
                    if (cls[C_ELSE])  active <= st_ent[ssp-1] & ~st_then[ssp-1] & loop_live;
                    if (cls[C_ENDIF]) begin active <= st_ent[ssp-1] & loop_live; ssp <= ssp - 1'b1; end
                end
            end
        end
    end endgenerate

    // ---- prefetch grant: warps with fewer than 2 words present or in flight first, round-robin within each class
    always @* begin
        fgo = 1'b0; fsel = f_rr;
        for (int k = NWARP-1; k >= 0; k--)
            if (qreq_v[(f_rr + k) % NWARP]) begin fgo = 1'b1; fsel = WW'((f_rr + k) % NWARP); end
        for (int k = NWARP-1; k >= 0; k--)
            if (qreq_v[(f_rr + k) % NWARP] && qlow_v[(f_rr + k) % NWARP]) begin fgo = 1'b1; fsel = WW'((f_rr + k) % NWARP); end
    end
    always @(posedge clk) begin if (fgo) fr_ins <= iram[qfpc_v[fsel]]; fq_ins <= fr_ins; end
    always @(posedge clk) begin
        if (rst) begin fr_v <= 1'b0; fq_v <= 1'b0; f_rr <= '0; end
        else begin
            fr_v <= fgo; fr_w <= fsel; fr_ep <= qep_v[fsel]; fr_pc <= qfpc_v[fsel];
            fq_v <= fr_v; fq_w <= fr_w; fq_ep <= fr_ep; fq_pc <= fr_pc;
            if (fgo) f_rr <= nxt(fsel);
        end
    end

    // ================================================================ S -> X, the shared counters, predicate / acc flags
    // The issuing warp's fields = AND-OR of the one-hot grant and every warp's bundle: no select encode in front
    // (`ins_v[sel]` and the x_w replicas behind a priority-encoded sel are too slow). Bundle layout, top down:
    // ins 64, am 16, wbm 16, mk 3, rc 2, wc 2, qr, sfu, sc, cvt, imul, idiv.
    localparam int BW = 109;
    wire [BW-1:0] bnd_v [0:NWARP-1];
    generate for (gg = 0; gg < NWARP; gg = gg + 1) begin : BND
        assign bnd_v[gg] = {ins_v[gg], am_v[gg], wbm_v[gg], mk_v[gg], rc_v[gg], wc_v[gg],
                            cls_v[gg][C_QR], cls_v[gg][C_SFU], cls_v[gg][C_SC], cls_v[gg][C_CVT],
                            cls_v[gg][C_IMUL], cls_v[gg][C_IDIV]};
    end endgenerate
    wire [BW-1:0] s_bnd;
    genvar gb;
    generate for (gb = 0; gb < BW; gb = gb + 1) begin : OH
        wire [NWARP-1:0] col;
        for (genvar k2 = 0; k2 < NWARP; k2 = k2 + 1) begin : C assign col[k2] = bnd_v[k2][gb]; end
        assign s_bnd[gb] = |(grant & col);
    end endgenerate
    wire [63:0] s_ins = s_bnd[108:45];
    wire [15:0] s_am = s_bnd[44:29], s_wbm = s_bnd[28:13];
    wire [2:0]  s_mk = s_bnd[12:10];
    wire [1:0]  s_rc = s_bnd[9:8], s_wc = s_bnd[7:6];
    wire        s_qr = s_bnd[5], s_sfu = s_bnd[4], s_sc = s_bnd[3], s_cvt = s_bnd[2], s_imul = s_bnd[1], s_idiv = s_bnd[0];
    wire bw_sfu, bw_sc, bw_cvt, bw_imul, bw_idiv;                  // a buffered class writes this cycle
    wire acc1_land, acc2_land; wire [WW-1:0] acc_land_w;             // a chain step lands in an accumulator
    reg  cdl_v, cdl_setp; reg [WW-1:0] cdl_w; reg [6:0] cdl_a; reg [15:0] cdl_l;
    wire [LW-1:0] y_cmp;
    always @(posedge clk) begin
        if (rst) begin x_valid <= 1'b0; wb_busy <= 16'd0; qr_hold <= 3'd0; inf_sfu <= 2'd0; inf_sc <= 2'd0; inf_cvt <= 2'd0;
                       inf_imul <= 2'd0; inf_idiv <= 1'b0; pred_pend <= '0; acc1_rdy <= '0; acc2_rdy <= '0; end
        else begin
            x_valid <= issue;
            if (issue) begin x_w <= sel; x_ins <= s_ins; x_rc <= s_rc; x_wc <= s_wc; x_mk <= s_mk; x_am <= s_am; end
            // the write lands 2 + lat cycles after this S cycle = 1 + lat cycles after the next one (checked there)
            wb_busy <= (wb_busy >> 1) | s_wbm;
            // quarter-rate micro-ops 4 cycles apart (the feeder is busy 4 cycles from X2); in flight per class
            qr_hold <= s_qr ? 3'd3 : (qr_hold != 3'd0) ? qr_hold - 3'd1 : 3'd0;
            inf_sfu <= inf_sfu + 2'(s_sfu) - 2'(bw_sfu);
            inf_sc  <= inf_sc  + 2'(s_sc)  - 2'(bw_sc);
            inf_cvt <= inf_cvt + 2'(s_cvt) - 2'(bw_cvt);
            inf_imul <= inf_imul + 2'(s_imul) - 2'(bw_imul);
            inf_idiv <= (inf_idiv | s_idiv) & ~bw_idiv;
            // predicate: pending from the SETP's issue until its compare lands. The per-warp flags are keyed on the
            // warp's own grant and fields (no select mux in front of them)
            for (int w = 0; w < NWARP; w++) begin
                if (grant[w] && cls_v[w][C_SETP]) pred_pend[w] <= 1'b1;
                else if (cdl_v && cdl_setp && cdl_w == WW'(w)) pred_pend[w] <= 1'b0;
                if (cdl_v && cdl_setp && cdl_w == WW'(w))
                    for (int l = 0; l < 16; l++) pred[w][l] <= (y_cmp[l*W +: W] != '0);
                // accumulators: not ready from the producing step's issue until it lands
                if (grant[w] && (mk_v[w] == MK_P1 || mk_v[w] == MK_A1)) acc1_rdy[w] <= 1'b0;
                else if (acc1_land && acc_land_w == WW'(w)) acc1_rdy[w] <= 1'b1;
                if (grant[w] && (mk_v[w] == MK_P2 || mk_v[w] == MK_A2)) acc2_rdy[w] <= 1'b0;
                else if (acc2_land && acc_land_w == WW'(w)) acc2_rdy[w] <= 1'b1;
            end
        end
    end

    // ================================================================ X: addresses, pre-decode
    wire [6:0] x_op = f_op(x_ins);
    wire [GAW-1:0] ra [0:2];
    // copy 2 reads rs2 (MAD, SEL) — or rs1 for ADD / SUB, whose addend then arrives on the c operand without a mux
    wire x_addsub = (x_op == ADD) || (x_op == SUB);
    wire [1:0] x_k2 = x_addsub ? 2'd1 : 2'd2;
    generate for (gk = 0; gk < 2; gk = gk + 1) begin : RA
        assign ra[gk] = {x_w, f_rs(x_ins, 2'(gk)), swz_c(f_swz(x_ins, 2'(gk)), x_rc)};
    end endgenerate
    assign ra[2] = {x_w, f_rs(x_ins, x_k2), swz_c(f_swz(x_ins, x_k2), x_rc)};
    wire [PAW-1:0] x_pidx = x_ins[PAW-1:0];
    always @(posedge clk) begin
        acc1_x <= acc1[x_w]; acc2_x <= acc2[x_w];                   // a chain step issues only after its accumulator landed
        x2_valid <= x_valid & ~rst;
        x2_w <= x_w; x2_wc <= x_wc; x2_mk <= x_mk; x2_am <= x_am; x2_rd <= f_rd(x_ins);
        x2_madd <= is_madd(x_op) | (x_op == DP3) | (x_op == DP4);
        x2_tgt <= (x_mk == MK_P1 || x_mk == MK_A1) ? 2'd1 : (x_mk == MK_P2 || x_mk == MK_A2) ? 2'd2 : 2'd0;
        x2_cmp <= is_cmp(x_op); x2_setp <= (x_op == SETP); x2_one <= is_one(x_op);
        x2_sto <= is_out(x_op);
        x2_slot <= (x_op == STV) ? {1'b0, x_ins[3:0]} : {4'b1000, x_ins[0]};   // STV i -> slot i; STO 0 / 1 -> 16 / 17
        x2_int <= is_int(x_op); x2_sel <= (x_op == SEL); x2_ldp <= (x_op == LDP); x2_ldv <= (x_op == LDV);
        x2_movi <= (x_op == MOVI); x2_ldpx <= (x_op == LDPX); x2_pidx <= x_pidx;
        x2_sub <= (x_op == SUB);
        x2_madm <= (x_op == ADD || x_op == SUB) ? 2'd1 : (x_op == MUL) ? 2'd3 : 2'd0;
        x2_cmpop <= (x_op == MIN) ? 2'd0 : (x_op == MAX) ? 2'd1 : 2'd2;
        x2_cc <= x_ins[15:13];                                      // SET / SETP cc [16:13]: 2 SLT 3 SGE 4 SEQ 5 SNE
        x2_intop <= (x_op==IADD) ? 3'd0 : (x_op==ISUB) ? 3'd1 : (x_op==ISLT) ? 3'd2 : (x_op==ISGE) ? 3'd3 : (x_op==ISEQ) ? 3'd4 : 3'd5;
        x2_m0 <= f_mod(x_ins, 2'd0); x2_m1 <= f_mod(x_ins, 2'd1); x2_m2 <= f_mod(x_ins, x_k2);
        x2_imm <= x_ins[4 +: W];
        // the params / attribute component: LDP / LDV read the swizzled one
        x2_sc_c <= swz_c(f_swz(x_ins, 2'd0), x_rc);
        x2_pq <= params[{pbank[x_w], x_pidx}];
        // quarter-rate classes, IMUL, divider
        x2_sfu <= is_sfu(x_op); x2_sc <= is_sc(x_op); x2_cvt <= is_cvt(x_op);
        x2_qr <= is_sfu(x_op) | is_sc(x_op) | is_cvt(x_op) | (x_op == IMUL) | is_idiv(x_op);
        x2_imul <= (x_op == IMUL); x2_idiv <= is_idiv(x_op);
        x2_sfuop <= (x_op == RCP) ? 2'd0 : (x_op == RSQRT) ? 2'd1 : (x_op == LOG2) ? 2'd2 : 2'd3;
        x2_cos <= (x_op == COS);
        x2_cvtop <= (x_op == ITOF) ? 3'd1 : (x_op == FTOU) ? 3'd2 : (x_op == UTOF) ? 3'd3 :
                    (x_op >= TRUNC && x_op <= RNDE) ? 3'(3'd4 + 3'(x_op - TRUNC)) : 3'd0;   // 4..7 = float_round modes
        x2_divmode <= (x_op == IMOD) ? 2'd1 : (x_op == UDIV) ? 2'd2 : (x_op == UMOD) ? 2'd3 : 2'd0;
    end

    // ================================================================ register file: 3 copies x 16 lanes, block RAM
    reg         gpr_we; reg [GAW-1:0] gpr_wa; reg [15:0] gpr_wl; reg [LW-1:0] gpr_wd;
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
    // LDPX: per lane a copy of the uniform store (both banks; written with params), read in X2 at the lane's slot =
    // the immediate + the low 8 bits of its index register (copy 1 = rs1) in the warp's bank; pxq valid in X3
    wire [16*TW-1:0] pxq;
    generate for (gl = 0; gl < 16; gl = gl + 1) begin : PL
        (* ram_style="block" *) reg [TW-1:0] pm [0:2*NPARAM-1];
        reg [TW-1:0] q;
        wire [PAW-1:0] slot = x2_pidx + br[gl*W +: PAW];
        always @(posedge clk) begin
            if (param_we) pm[{param_bank, param_idx}] <= param_data;
            q <= pm[{pbank[x2_w], slot}];
        end
        assign pxq[gl*TW +: TW] = q;
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
    reg  [LW-1:0] at_c;                                             // the attribute component of every lane
    always @* for (int l = 0; l < 16; l++) at_c[l*W +: W] = aq[l*TW + x2_sc_c*W +: W];
    // madd operands: ADD a*1+b, SUB a*1-b (b read through copy 2: it is the c operand), MUL a*b+0, MAD a*b+c; chain
    // steps: products with ZERO / an accumulator as the addend, the DP final acc1*1 + acc2
    reg [LW-1:0] ma, mb, mc;
    always @* begin
        ma = a_m; mb = b_m; mc = c_m;
        case (x2_mk)
            MK_P1, MK_P2: mc = ZEROL;
            MK_A1: mc = acc1_x;
            MK_A2: mc = acc2_x;
            MK_F:  begin ma = acc1_x; mb = ONEL; mc = acc2_x; end
            default:
                case (x2_madm)
                    2'd1: begin mb = ONEL; end
                    2'd3: begin mc = ZEROL; end
                    default: ;
                endcase
        endcase
    end
    wire [15:0] vm, vc;
    wire [LW-1:0] y_madd;
    wire [2:0] cmp_op = (x2_cmpop == 2'd2) ? x2_cc : {1'b0, x2_cmpop};   // float_cmp: 0 MIN 1 MAX 2 SLT 3 SGE 4 SEQ 5 SNE
    generate for (gl = 0; gl < 16; gl = gl + 1) begin : LANE
        float_madd #(EXP_W, MAN_W) u_madd (.clk, .rst, .in_valid(x2_valid & x2_madd), .sub(x2_sub & (x2_mk == MK_N)),
            .a(ma[gl*W +: W]), .b(mb[gl*W +: W]), .c(mc[gl*W +: W]), .out_valid(vm[gl]), .result(y_madd[gl*W +: W]), .prod_o());
        float_cmp #(EXP_W, MAN_W) u_cmp (.clk, .rst, .in_valid(x2_valid & (x2_cmp | x2_setp)), .op(cmp_op),
            .a(a_m[gl*W +: W]), .b(b_m[gl*W +: W]), .out_valid(vc[gl]), .result(y_cmp[gl*W +: W]));
    end endgenerate
    reg [LW-1:0] one_y;                                             // the 1-cycle path
    always @* begin
        one_y = a_m;                                                // MOV
        if (x2_movi) one_y = {16{x2_imm}};
        else if (x2_ldp) one_y = {16{pq_c}};
        else if (x2_ldv) one_y = at_c;
        else if (x2_sel) one_y = selv(ar, br, cr);
        else if (x2_int) one_y = int_alu(ar, br, x2_intop);
    end
    // the 2-cycle path (LDPX): X3 picks the swizzled component of every lane's gathered vec4 (pxq, addressed in X2);
    // the result is written in W the cycle after, its scoreboard bit cleared in X3 (one cycle before the write)
    reg x3_v; reg [WW-1:0] x3_w; reg [6:0] x3_a; reg [15:0] x3_l; reg [1:0] x3_c;
    reg xdl_v; reg [WW-1:0] xdl_w; reg [6:0] xdl_a; reg [15:0] xdl_l; reg [LW-1:0] xdl_y;
    reg [LW-1:0] x3_y;
    always @* for (int l = 0; l < 16; l++) x3_y[l*W +: W] = pxq[l*TW + x3_c*W +: W];
    always @(posedge clk) begin
        if (rst) begin x3_v <= 1'b0; xdl_v <= 1'b0; end
        else begin x3_v <= x2_valid & x2_ldpx; xdl_v <= x3_v; end
        x3_w <= x2_w; x3_a <= {x2_rd, x2_wc}; x3_l <= x2_am; x3_c <= x2_sc_c;
        xdl_w <= x3_w; xdl_a <= x3_a; xdl_l <= x3_l; xdl_y <= x3_y;
    end
    // STO / STV: the output store = one block RAM per lane, {warp, slot[4:0], comp[1:0]}; micro-op wc writes the active
    // lanes; the retiring warp's row at {rp, out_sel} is registered (valid the cycle after out_sel)
    generate for (gl = 0; gl < 16; gl = gl + 1) begin : OL
        (* ram_style="block" *) reg [W-1:0] om [0:NWARP*128-1];
        reg [W-1:0] oq;
        always @(posedge clk) begin
            if (x2_valid && x2_sto && x2_am[gl]) om[{x2_w, x2_slot, x2_wc}] <= a_m[gl*W +: W];
            oq <= om[{rp, out_sel}];
        end
        assign out_row[gl*W +: W] = oq;
    end endgenerate
    wire v_div;
    // delay lines X2 -> W: madd (MADD_LAT, with the target: register / acc1 / acc2), cmp (1), the 1-cycle path (1)
    reg [MADD_LAT-1:0] mdl_v; reg [WW-1:0] mdl_w [0:MADD_LAT-1]; reg [6:0] mdl_a [0:MADD_LAT-1]; reg [15:0] mdl_l [0:MADD_LAT-1];
    reg [1:0] mdl_t [0:MADD_LAT-1];
    reg sdl_v; reg [WW-1:0] sdl_w; reg [6:0] sdl_a; reg [15:0] sdl_l; reg [LW-1:0] sdl_y;
    always @(posedge clk) begin
        if (rst) begin mdl_v <= '0; cdl_v <= 1'b0; sdl_v <= 1'b0; end
        else begin
            mdl_v <= {mdl_v[MADD_LAT-2:0], x2_valid & x2_madd};
            cdl_v <= x2_valid & (x2_cmp | x2_setp);
            sdl_v <= x2_valid & x2_one;
        end
        for (int i = MADD_LAT-1; i > 0; i--) begin mdl_w[i] <= mdl_w[i-1]; mdl_a[i] <= mdl_a[i-1]; mdl_l[i] <= mdl_l[i-1]; mdl_t[i] <= mdl_t[i-1]; end
        mdl_w[0] <= x2_w; mdl_a[0] <= {x2_rd, x2_wc}; mdl_l[0] <= x2_am; mdl_t[0] <= x2_tgt;
        cdl_w <= x2_w; cdl_a <= {x2_rd, x2_wc}; cdl_l <= x2_am; cdl_setp <= x2_setp;
        sdl_w <= x2_w; sdl_a <= {x2_rd, x2_wc}; sdl_l <= x2_am; sdl_y <= one_y;
    end
    // accumulator landings (chain steps): every lane, no write port
    wire mdl_last = mdl_v[MADD_LAT-1];
    assign acc1_land = mdl_last & (mdl_t[MADD_LAT-1] == 2'd1);
    assign acc2_land = mdl_last & (mdl_t[MADD_LAT-1] == 2'd2);
    assign acc_land_w = mdl_w[MADD_LAT-1];
    always @(posedge clk) begin
        if (acc1_land) acc1[acc_land_w] <= y_madd;
        if (acc2_land) acc2[acc_land_w] <= y_madd;
    end

    // ================================================================ quarter-rate feeder and units
    // X2 captures the 16-lane operands and starts the 4-cycle feed; cycle i sends lane 4k+i to unit k
    reg qr_run; reg [1:0] qr_i; reg [LW-1:0] qr_x, qr_x2; reg [4:0] qr_cls; reg [1:0] qr_sfuop; reg qr_cos;
    reg [2:0] qr_cvtop; reg [1:0] qr_divmode;
    wire qr_start = x2_valid & x2_qr;
    wire [MW-1:0] x2_meta = {x2_w, x2_rd, x2_wc, x2_am};
    always @(posedge clk) begin
        if (rst) begin qr_run <= 1'b0; qr_i <= 2'd0; end
        else if (qr_start) begin qr_run <= 1'b1; qr_i <= 2'd0; end
        else if (qr_run) begin qr_i <= qr_i + 2'd1; if (qr_i == 2'd3) qr_run <= 1'b0; end
        if (qr_start) begin
            qr_x <= (x2_imul | x2_idiv) ? ar : a_m; qr_x2 <= br;   // integers: raw registers
            qr_cls <= {x2_idiv, x2_imul, x2_cvt, x2_sc, x2_sfu}; qr_sfuop <= x2_sfuop; qr_cos <= x2_cos;
            qr_cvtop <= x2_cvtop; qr_divmode <= x2_divmode;
        end
    end
    wire [W-1:0] ux [0:3], ux2 [0:3];                               // unit k's operands this feed cycle
    wire [3:0] u_sfu_ov, u_sc_ov, u_cvt_ov;
    wire [W-1:0] u_sfu_y [0:3], u_sc_y [0:3], u_cvt_y [0:3], u_imul_y [0:3];
    wire [4*W-1:0] int4_a, int4_b, int4_ym, y_div4; wire int4_vm;
    reg [2:0] cv_d1, cv_d2;                                         // convert: the op travels with its results
    always @(posedge clk) begin cv_d1 <= qr_cvtop; cv_d2 <= cv_d1; end
    // the SFUs' tables: inside each unit, or (SFU_TAB2) one sfu_tab2 per pair of units
    localparam int SK = (EXP_W >= 7) ? 12 : 8, SKL = (EXP_W >= 7) ? 11 : 8;
    wire [SK-1:0] tr_a [0:3], ts_a [0:3]; wire [SKL-1:0] tl_i [0:3], te_i [0:3];
    wire [MAN_W+1:0] tr_q [0:3], ts_q [0:3]; wire [23:0] tla [0:3], tlb [0:3], tea [0:3], teb [0:3];
    generate if (SFU_TAB2) begin : g_tab
        for (genvar gp = 0; gp < 2; gp++) begin : TP
            sfu_tab2 #(.EXP_W(EXP_W), .MAN_W(MAN_W), .K(SK), .KLOG(SKL), .RCP_MEM(RCP_MEM), .RSQRT_MEM(RSQRT_MEM),
                       .LOG2_MEM(LOG2_MEM), .EXP2_MEM(EXP2_MEM)) u_tab (.clk,
                .rcp_a0(tr_a[2*gp]), .rcp_a1(tr_a[2*gp+1]), .rsq_a0(ts_a[2*gp]), .rsq_a1(ts_a[2*gp+1]),
                .log_i0(tl_i[2*gp]), .log_i1(tl_i[2*gp+1]), .exp_i0(te_i[2*gp]), .exp_i1(te_i[2*gp+1]),
                .rcp_q0(tr_q[2*gp]), .rcp_q1(tr_q[2*gp+1]), .rsq_q0(ts_q[2*gp]), .rsq_q1(ts_q[2*gp+1]),
                .la0(tla[2*gp]), .lb0(tlb[2*gp]), .la1(tla[2*gp+1]), .lb1(tlb[2*gp+1]),
                .ea0(tea[2*gp]), .eb0(teb[2*gp]), .ea1(tea[2*gp+1]), .eb1(teb[2*gp+1]));
        end
    end else begin : g_ntab
        for (genvar gq = 0; gq < 4; gq++) begin : TZ
            assign {tr_q[gq], ts_q[gq], tla[gq], tlb[gq], tea[gq], teb[gq]} = '0;
        end
    end endgenerate
    genvar gu;
    generate for (gu = 0; gu < 4; gu = gu + 1) begin : QU
        assign ux[gu]  = qr_x [(4*gu + qr_i)*W +: W];
        assign ux2[gu] = qr_x2[(4*gu + qr_i)*W +: W];
        float_sfu #(.EXP_W(EXP_W), .MAN_W(MAN_W), .K(SK), .KLOG(SKL), .SPLIT6(SFU_SPLIT6), .SPLIT1B(SFU_SPLIT1B),
            .RCP_MEM(RCP_MEM), .RSQRT_MEM(RSQRT_MEM), .LOG2_MEM(LOG2_MEM), .EXP2_MEM(EXP2_MEM), .TAB_EXT(SFU_TAB2)) u_sfu (
            .clk, .rst, .in_valid(qr_run & qr_cls[0]), .op({1'b0, qr_sfuop}), .x(ux[gu]),
            .out_valid(u_sfu_ov[gu]), .result(u_sfu_y[gu]),
            .t_rcp_a(tr_a[gu]), .t_rsq_a(ts_a[gu]), .t_log_i(tl_i[gu]), .t_exp_i(te_i[gu]),
            .t_rcp_q(tr_q[gu]), .t_rsq_q(ts_q[gu]), .t_la(tla[gu]), .t_lb(tlb[gu]), .t_ea(tea[gu]), .t_eb(teb[gu]));
        sincos_sfu #(.EXP_W(EXP_W), .MAN_W(MAN_W), .KSIN(10), .SIN_MEM(SIN_MEM)) u_sc (
            .clk, .rst, .in_valid(qr_run & qr_cls[1]), .op(qr_cos), .x(ux[gu]), .out_valid(u_sc_ov[gu]), .result(u_sc_y[gu]));
        // convert: FTOI / FTOU through one signed W+1-bit converter clamped after, ITOF / UTOF sign- / zero-extended,
        // TRUNC FLR CEIL RNDE through float_round (as alu_cluster)
        wire [W:0] fr; wire [W-1:0] i2f_o, rnd_o;
        float_cvt_f2i #(.EXP_W(EXP_W), .MAN_W(MAN_W), .OW(W+1)) u_f2i (.clk, .rst, .in_valid(qr_run & qr_cls[2]),
            .fval(ux[gu]), .out_valid(u_cvt_ov[gu]), .ival(fr));
        float_cvt_i2f #(.IW(W+1), .EXP_W(EXP_W), .MAN_W(MAN_W)) u_i2f (.clk, .rst, .in_valid(qr_run & qr_cls[2]),
            .ival({(qr_cvtop != 3'd3) & ux[gu][W-1], ux[gu]}), .out_valid(), .result(i2f_o));
        float_round #(.EXP_W(EXP_W), .MAN_W(MAN_W)) u_rnd (.clk, .rst, .in_valid(qr_run & qr_cls[2]),
            .mode(qr_cvtop[1:0]), .x(ux[gu]), .out_valid(), .result(rnd_o));
        wire [W-1:0] fti = (~fr[W] & fr[W-1]) ? {1'b0, {(W-1){1'b1}}} : (fr[W] & ~fr[W-1]) ? {1'b1, {(W-1){1'b0}}} : fr[W-1:0];
        wire [W-1:0] ftu = fr[W] ? {W{1'b0}} : fr[W-1:0];
        assign u_cvt_y[gu] = cv_d2[2] ? rnd_o : cv_d2[0] ? i2f_o : cv_d2[1] ? ftu : fti;
        assign int4_a[gu*W +: W] = ux[gu]; assign int4_b[gu*W +: W] = ux2[gu];
        assign u_imul_y[gu] = int4_ym[gu*W +: W];
    end endgenerate
    // the integer unit: IMUL (one DSP per lane, latency 3) and the divider on the 4 quarter-rate lanes (as in the
    // fragment core), their results buffered like the SFU's. The divider takes a warp's 16 lanes as 4 rounds (DSEQ,
    // ≈ 108 cycles, one in flight) and returns them as 4 cycles of 4 lanes. The unit's IOP section is tied off.
    int_unit #(.NL(4), .W(W), .ONE(ONE1), .DSEQ(4)) u_int4 (.clk, .rst, .a(int4_a), .b(int4_b),
        .mul_valid(qr_run & qr_cls[3]), .v_mul(int4_vm), .y_mul(int4_ym),
        .div_valid(qr_run & qr_cls[4]), .div_mode(qr_divmode), .div_ready(idiv_ready), .v_div(v_div), .y_div(y_div4),
        .iop_valid(1'b0), .iop_fn(4'd0), .v_iop(), .y_iop());
    // collectors: the 4 results of cycle i land in lanes 4k+i; a collector is full after the 4th cycle and is taken,
    // with the micro-op's meta (queued at the feed start), into ONE shared 8-entry result queue (LUTRAM): one class
    // per cycle, the longest-waiting class first (see shader_core_fs2 for the timing argument; asserted below).
    wire [3:0] qc_ov = {int4_vm, u_cvt_ov[0], u_sc_ov[0], u_sfu_ov[0]};
    wire [3:0] qc_start = qr_start ? {x2_imul, x2_cvt, x2_sc, x2_sfu} : 4'd0;
    wire [3:0] qc_full, qc_wait; reg [3:0] qc_take;                 // collector full / full since the previous cycle / taken
    wire [LW-1:0] qc_col [0:3]; wire [MW-1:0] qc_m [0:3];
    genvar gc;
    generate for (gc = 0; gc < 4; gc = gc + 1) begin : QC
        wire [W-1:0] uy [0:3];
        if (gc == 0) begin assign uy[0] = u_sfu_y[0]; assign uy[1] = u_sfu_y[1]; assign uy[2] = u_sfu_y[2]; assign uy[3] = u_sfu_y[3]; end
        else if (gc == 1) begin assign uy[0] = u_sc_y[0]; assign uy[1] = u_sc_y[1]; assign uy[2] = u_sc_y[2]; assign uy[3] = u_sc_y[3]; end
        else if (gc == 2) begin assign uy[0] = u_cvt_y[0]; assign uy[1] = u_cvt_y[1]; assign uy[2] = u_cvt_y[2]; assign uy[3] = u_cvt_y[3]; end
        else begin assign uy[0] = u_imul_y[0]; assign uy[1] = u_imul_y[1]; assign uy[2] = u_imul_y[2]; assign uy[3] = u_imul_y[3]; end
        reg [1:0] cnt; reg [LW-1:0] col; reg full, waitq, sh_v; reg [4*W-1:0] sh0;
        reg [MW-1:0] mq [0:1]; reg mq_wp, mq_rp;                    // meta of the ops fed, in order
        always @(posedge clk) begin
            if (rst) begin cnt <= 2'd0; full <= 1'b0; waitq <= 1'b0; sh_v <= 1'b0; mq_wp <= 1'b0; mq_rp <= 1'b0; end
            else begin
                if (qc_start[gc]) begin mq[mq_wp] <= x2_meta; mq_wp <= ~mq_wp; end
                if (qc_ov[gc]) begin
                    for (int k = 0; k < 4; k++) col[(4*k + cnt)*W +: W] <= uy[k];
                    cnt <= cnt + 2'd1;
                    if (cnt == 2'd3) full <= 1'b1;
                    if (cnt == 2'd0 && full && !qc_take[gc]) begin   // the next stream overwrites slot 0 before the take
                        for (int k = 0; k < 4; k++) sh0[k*W +: W] <= col[(4*k)*W +: W];
                        sh_v <= 1'b1;
                    end
                end
                waitq <= full & ~qc_take[gc];
                if (qc_take[gc]) begin full <= 1'b0; sh_v <= 1'b0; mq_rp <= ~mq_rp; end
            end
        end
        reg [LW-1:0] col_o;
        always @* begin col_o = col; if (sh_v) for (int k = 0; k < 4; k++) col_o[(4*k)*W +: W] = sh0[k*W +: W]; end
        assign qc_full[gc] = full; assign qc_wait[gc] = waitq; assign qc_col[gc] = col_o; assign qc_m[gc] = mq[mq_rp];
        // synthesis translate_off
        always @(posedge clk) if (!rst && full && waitq && !qc_take[gc]) $fatal(1, "shader_core_vs2: collector %0d waited 2 cycles", gc);
        // synthesis translate_on
    end endgenerate
    (* ram_style="distributed" *) reg [LW+MW+2-1:0] rq [0:7];       // {class, meta, result}
    reg [2:0] rq_wp, rq_rp; reg [3:0] rq_cnt;
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
        if (rst) begin rq_wp <= 3'd0; rq_rp <= 3'd0; rq_cnt <= 4'd0; end
        else begin
            if (rq_push) begin rq[rq_wp] <= {rq_sel, qc_m[rq_sel], qc_col[rq_sel]}; rq_wp <= rq_wp + 3'd1; end
            if (bw_q) rq_rp <= rq_rp + 3'd1;
            rq_cnt <= rq_cnt + 4'(rq_push) - 4'(bw_q);
        end
    end
    // synthesis translate_off
    always @(posedge clk) if (!rst && rq_push && rq_cnt == 4'd8 && !bw_q) $fatal(1, "shader_core_vs2: result queue overflow");
    // synthesis translate_on
    // divider result (one in flight): the unit returns a warp's 16 lanes as 4 cycles of 4 (lanes 4k + i in cycle i, the
    // feed order), assembled here; the next IDIV waits for this write (inf_idiv) and for the unit (idiv_ready)
    reg idiv_pend; reg [LW-1:0] idiv_y; reg [MW-1:0] idiv_m; reg [1:0] idv_i;
    always @(posedge clk) begin
        if (rst) begin idiv_pend <= 1'b0; idv_i <= 2'd0; end
        else begin
            if (x2_valid & x2_idiv) idiv_m <= x2_meta;
            if (v_div) begin idv_i <= idv_i + 2'd1; if (idv_i == 2'd3) idiv_pend <= 1'b1; end
            else if (bw_idiv) idiv_pend <= 1'b0;
        end
        if (v_div) for (int k = 0; k < 4; k++) idiv_y[(4*k + idv_i)*W +: W] <= y_div4[k*W +: W];
    end

    // ================================================================ W: the write port
    // scheduled results first (they never collide: the write slots were reserved at S), then one buffered result
    // a madd result for a register (not an accumulator) / SET (SETP goes to the predicate): REGISTERS set a stage ahead
    // (timing at 120 MHz: the two-term select of the write data fans out to 16 lanes x 3 copies).
    // madd_gw = mdl_last & (mdl_t == 0) of this cycle; cmp_gw = the compare unit's out_valid & ~cdl_setp
    reg madd_gw, cmp_gw;
    always @(posedge clk) begin
        if (rst) begin madd_gw <= 1'b0; cmp_gw <= 1'b0; end
        else begin madd_gw <= mdl_v[MADD_LAT-2] & (mdl_t[MADD_LAT-2] == 2'd0); cmp_gw <= x2_valid & x2_cmp; end
    end
    wire sched_w = madd_gw | cmp_gw | sdl_v | xdl_v;
    assign bw_q    = ~sched_w & (rq_cnt != 4'd0);
    assign bw_sfu  = bw_q & (bw_qcls == 2'd0);
    assign bw_sc   = bw_q & (bw_qcls == 2'd1);
    assign bw_cvt  = bw_q & (bw_qcls == 2'd2);
    assign bw_imul = bw_q & (bw_qcls == 2'd3);
    assign bw_idiv = ~sched_w & ~bw_q & idiv_pend;
    always @* begin
        gpr_we = 1'b0; gpr_wa = '0; gpr_wl = 16'd0; gpr_wd = y_madd;
        bw_v = 1'b0; bw_w = '0; bw_a = '0;
        if (madd_gw)          begin gpr_we = 1'b1; gpr_wa = {mdl_w[MADD_LAT-1], mdl_a[MADD_LAT-1]}; gpr_wl = mdl_l[MADD_LAT-1]; gpr_wd = y_madd; end
        else if (cmp_gw)      begin gpr_we = 1'b1; gpr_wa = {cdl_w, cdl_a}; gpr_wl = cdl_l; gpr_wd = y_cmp; end
        else if (sdl_v)       begin gpr_we = 1'b1; gpr_wa = {sdl_w, sdl_a}; gpr_wl = sdl_l; gpr_wd = sdl_y; end
        else if (xdl_v)       begin gpr_we = 1'b1; gpr_wa = {xdl_w, xdl_a}; gpr_wl = xdl_l; gpr_wd = xdl_y; end
        else if (bw_q)        begin gpr_we = 1'b1; {gpr_wa, gpr_wl} = rq_h[LW +: MW]; gpr_wd = rq_h[LW-1:0]; end
        else if (bw_idiv)     begin gpr_we = 1'b1; {gpr_wa, gpr_wl} = idiv_m; gpr_wd = idiv_y; end
        if (gpr_we & ~sched_w) begin bw_v = 1'b1; {bw_w, bw_a} = gpr_wa; end
    end
    // scheduled results clear the scoreboard one cycle before the write: REGISTERS decoded a stage ahead (timing at
    // 120 MHz: the madd delay line's target compare -> the 128 pending bits of 4 warps) — the madd at stage MADD_LAT-3
    // (= MADD_LAT-2 next cycle), the compare / 1-cycle op in X (= X2 next cycle),
    // LDPX in X2 (= X3 next cycle); the three never meet (checked below)
    always @(posedge clk) begin
        if (rst) begin ec_v <= 1'b0; ec_w <= '0; ec_a <= '0; end
        else if (mdl_v[MADD_LAT-3] & (mdl_t[MADD_LAT-3] == 2'd0)) begin ec_v <= 1'b1; ec_w <= mdl_w[MADD_LAT-3]; ec_a <= mdl_a[MADD_LAT-3]; end
        else if (x_valid & (is_cmp(x_op) | is_one(x_op))) begin ec_v <= 1'b1; ec_w <= x_w; ec_a <= {f_rd(x_ins), x_wc}; end
        else if (x2_valid & x2_ldpx) begin ec_v <= 1'b1; ec_w <= x2_w; ec_a <= {x2_rd, x2_wc}; end
        else begin ec_v <= 1'b0; ec_w <= '0; ec_a <= '0; end
    end
    // synthesis translate_off
    always @(posedge clk) if (!rst && (3'(madd_gw) + 3'(cmp_gw) + 3'(sdl_v) + 3'(xdl_v) > 3'd1))
        $fatal(1, "shader_core_vs2: two scheduled results in one write cycle");
    always @(posedge clk) if (!rst && (3'(mdl_v[MADD_LAT-2] & (mdl_t[MADD_LAT-2] == 2'd0)) + 3'(x2_valid & (x2_cmp | x2_one)) + 3'(x3_v) > 3'd1))
        $fatal(1, "shader_core_vs2: two scoreboard clears in one cycle");
    always @(posedge clk) if (!rst && v_div && idiv_pend && !bw_idiv) $fatal(1, "shader_core_vs2: divider result overrun");
    // synthesis translate_on

    // ================================================================ outputs of the oldest warp
    assign out_err = err_v[rp];
    wire _unused = &{1'b0, vm, vc, u_sfu_ov[3:1], u_sc_ov[3:1], u_cvt_ov[3:1]};
endmodule
`default_nettype wire
